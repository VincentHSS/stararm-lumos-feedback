"""Explicit damping experiments with separately declared model mass coverage.

An operator's unloaded declaration is not evidence that the URDF contains the
complete gripper mass. The baseline records absolute model residuals without
removing a fitted mean, and is never updated while damping is authorized.
"""

import math
import time
from copy import deepcopy

from .baseline import BaselineEstimator
from .config import experimental_ceiling
from .control import Controller

EXPERIMENT_RISE_MW_S = 100.0
BASELINE_GROUPS = {"arm": (0, 1, 2, 3, 4, 5), "gripper": (6,)}


class ExperimentalDampingController(Controller):
    def __init__(self, profile, caps, *, clock=None):
        options = getattr(profile, "experimental_damping", None)
        if (
            getattr(profile, "engineering", False) is not True
            or not isinstance(options, dict)
            or options.get("authorized") is not True
            or options.get("unloaded_confirmed") is not True
            or options.get("native_button_required") is not True
            or "max_mw" not in options
        ):
            raise ValueError("Experimental damping requires its explicit unloaded engineering contract")
        self.ceiling_mw = experimental_ceiling(options.get("max_mw"), getattr(profile, "data", None))
        if self.ceiling_mw is None and getattr(profile, "mass_confirmed", False) is not True:
            raise ValueError("Removing the experimental cap requires confirmed URDF mass coverage")
        super().__init__(profile, caps)
        self.clock = time.monotonic if clock is None else clock
        # Both estimators still observe all seven actual positions and speeds.
        # Only target-arrival admission and the statistics used for output are
        # independent, so moving the gripper cannot qualify an arm baseline.
        self.baseline = BaselineEstimator(profile)
        self._baselines = {"arm": self.baseline, "gripper": BaselineEstimator(profile)}
        self._frozen_baselines = {}
        self._closed_baselines = {}
        self._baseline_collection_closed = False
        self._baseline_floor = [None] * 7
        self._baseline_target_anchors = {name: None for name in BASELINE_GROUPS}
        self._baseline_reasons = {name: "warming_up" for name in BASELINE_GROUPS}
        self._last_seen = None
        self._acceleration_initialized = False
        self._warmup_s = 0.0
        self._warmup_required_s = max(
            max(p["filter_s"], p["acceleration_filter_s"], p["confirm_s"])
            for p in profile.policy
        )
        self._state, self._reason = "waiting_baseline", "warming_up"

    def snapshot(self):
        groups = {
            name: dict(
                frozen=name in self._frozen_baselines,
                reason=self._baseline_reasons[name],
                axis_indices=list(indices),
                baseline=self._group_baseline(name),
            )
            for name, indices in BASELINE_GROUPS.items()
        }
        return dict(
            state=self._state,
            ceiling_mw=self.ceiling_mw,
            effective_caps_mw=self.effective_caps(),
            effective_rise_mw_s=self.effective_rise_rates(),
            baseline_frozen=len(self._frozen_baselines) == len(BASELINE_GROUPS),
            baseline_ready_axes=self._ready_axes(),
            baseline_collection_closed=self._baseline_collection_closed,
            baseline_groups=groups,
            mass_confirmed=bool(getattr(self.profile, "mass_confirmed", False)),
            unloaded_confirmed=True,
            reason=self._reason,
            automatic_hold_accepted=False,
            warmup_s=self._warmup_s,
            warmup_required_s=self._warmup_required_s,
            baseline=self._combined_baseline(groups),
            baseline_coverage_accepted=False,
            motion_margin_source="engineering baseline-floor ratios; no measured moving-load separation",
        )

    def _ready_axes(self):
        return [
            name in self._frozen_baselines
            for name, indices in BASELINE_GROUPS.items() for _ in indices
        ]

    def _group_baseline(self, name):
        frozen = self._frozen_baselines.get(name)
        closed = self._closed_baselines.get(name)
        return deepcopy(frozen if frozen is not None else closed) if (
            frozen is not None or closed is not None
        ) else self._baselines[name].snapshot()

    @staticmethod
    def _combined_baseline(groups):
        """Keep legacy diagnostics without attributing one group's data to another."""
        snapshots = {name: group["baseline"] for name, group in groups.items()}
        combined = deepcopy(snapshots["arm"])
        combined["ready"] = all(baseline["ready"] for baseline in snapshots.values())
        combined["current_window_qualified"] = all(
            baseline["current_window_qualified"] for baseline in snapshots.values()
        )
        for key in (
            "sample_count", "pending_stationary_sample_count", "continuous_stationary_s",
            "retained_stationary_s",
        ):
            combined[key] = min(baseline[key] for baseline in snapshots.values())
        for key in (
            "absolute_residual_max_nm", "absolute_residual_p95_nm",
            "absolute_residual_p99_nm", "recommended_deadzone_nm",
        ):
            values = [None] * 7
            for name, indices in BASELINE_GROUPS.items():
                statistics = snapshots[name][key]
                if statistics is not None:
                    for i in indices:
                        values[i] = statistics[i]
            combined[key] = values if any(value is not None for value in values) else None
        for key in ("coordinate_min", "coordinate_max"):
            values = [None] * 7
            for name, indices in BASELINE_GROUPS.items():
                statistics = snapshots[name]["pose_coverage"][key]
                if statistics is not None:
                    for i in indices:
                        values[i] = statistics[i]
            combined["pose_coverage"][key] = values if any(value is not None for value in values) else None
        return combined

    def _reset_initial_windows(self):
        if not self._baseline_collection_closed:
            for name, estimator in self._baselines.items():
                estimator.gap()
                self._baseline_target_anchors[name] = None

    def _output_state(self):
        if len(self._frozen_baselines) == len(BASELINE_GROUPS):
            return "active", (
                "mass_confirmed_experimental_damping" if getattr(self.profile, "mass_confirmed", False)
                else "unconfirmed_mass_experimental_damping"
            )
        if "arm" in self._frozen_baselines:
            return "partially_active", "arm_active_gripper_baseline_unavailable"
        return "partially_active", "gripper_active_arm_baseline_unavailable"

    def _collect_initial_baselines(self, samples, residual, admission):
        if not isinstance(admission, dict):
            admission = {}
        final_target = admission.get("target")
        tolerance = admission.get("target_tolerance")
        valid_targets = (
            isinstance(final_target, (list, tuple)) and len(final_target) == 7
            and isinstance(tolerance, (list, tuple)) and len(tolerance) == 7
            and all(type(value) in (int, float) and math.isfinite(value) for value in final_target)
            and all(
                type(value) in (int, float) and math.isfinite(value) and value > 0 for value in tolerance
            )
        )
        groups = admission.get("groups")
        changed_any = False
        admitted_any = False
        ready = {}
        for name, indices in BASELINE_GROUPS.items():
            estimator = self._baselines[name]
            if groups is None:
                # Older callers authorize both groups with the common gate.
                allowed = admission.get("allowed") is True
                reason = admission.get("reason", "baseline_admission_missing")
            else:
                group = groups.get(name, {}) if isinstance(groups, dict) else {}
                declared_indices = group.get("axis_indices") if isinstance(group, dict) else None
                valid_group = (
                    isinstance(declared_indices, (list, tuple))
                    and tuple(declared_indices) == indices
                    and all(type(i) is int for i in declared_indices)
                )
                allowed = valid_group and group.get("allowed") is True
                reason = group.get("reason", "baseline_admission_missing") if valid_group else (
                    "baseline_admission_missing"
                )
                if admission.get("common_allowed") is not True:
                    allowed = False
                    reason = admission.get("reason", "baseline_admission_missing")
            if not allowed or not valid_targets:
                estimator.gap()
                self._baseline_target_anchors[name] = None
                self._baseline_reasons[name] = str(reason) if not allowed else "baseline_admission_missing"
                continue
            admitted_any = True
            anchor = self._baseline_target_anchors[name]
            # A legacy common gate described one seven-axis target window.
            # New grouped admissions only restart the corresponding group.
            anchor_indices = range(7) if groups is None else indices
            changed = anchor is None or any(
                abs(final_target[i] - anchor[i]) > tolerance[i] for i in anchor_indices
            )
            changed_any |= changed
            if changed:
                estimator.gap()
                self._baseline_target_anchors[name] = list(final_target)
            if not estimator.update(samples, residual):
                # Target and actual-position anchors share the same continuous
                # seven-axis stationary window.
                self._baseline_target_anchors[name] = list(final_target)
            baseline = estimator.snapshot()
            self._baseline_reasons[name] = (
                "waiting_stable_follow_target" if changed else "collecting_declared_unloaded_baseline"
            )
            if baseline["ready"] and baseline["current_window_qualified"]:
                ready[name] = baseline
        if ready:
            # Close the shared initial window before any output. A group that
            # did not qualify now must wait for a new session; later load or
            # native release can never silently become an unloaded baseline.
            self._baseline_collection_closed = True
            for name, indices in BASELINE_GROUPS.items():
                if name in ready:
                    baseline = deepcopy(ready[name])
                    self._frozen_baselines[name] = baseline
                    for i in indices:
                        self._baseline_floor[i] = baseline["recommended_deadzone_nm"][i]
                    self._baseline_reasons[name] = "baseline_frozen"
                else:
                    self._closed_baselines[name] = deepcopy(self._baselines[name].snapshot())
            self._state, self._reason = self._output_state()
            if len(ready) == len(BASELINE_GROUPS):
                self._reason = "baseline_frozen_output_starts_on_next_new_group"
            self.telemetry.update(requested_mw=[0.0] * 7, experimental=self.snapshot())
        else:
            reason = (
                "waiting_stable_follow_target" if changed_any else "collecting_declared_unloaded_baseline"
            ) if admitted_any else self._baseline_reasons["arm"]
            self._zero_output(reason)

    def effective_caps(self):
        caps = self.caps.effective()
        return caps if self.ceiling_mw is None else [min(self.ceiling_mw, cap) for cap in caps]

    def effective_rise_rates(self):
        rates = [policy["rise_mw_s"] for policy in self.profile.policy]
        return rates if self.ceiling_mw is None else [min(rate, EXPERIMENT_RISE_MW_S) for rate in rates]

    def _zero_output(self, reason, *, invalid=False):
        self.power = [0.0] * 7
        self.confirm = [0.0] * 7
        self.release = [0.0] * 7
        self.active = [False] * 7
        self.stall = [0.0] * 7
        self.hold = False
        self._reason = reason
        self._state = "model_invalid" if invalid or self._baseline_collection_closed else "waiting_baseline"
        self.telemetry.update(
            requested_mw=[0.0] * 7,
            automatic_hold=False,
            experimental=self.snapshot(),
        )

    def gap(self):
        super().gap()
        self.filtered = [0.0] * 7
        self._acceleration_initialized = False
        self._warmup_s = 0.0
        self._reset_initial_windows()
        self.telemetry["model_valid"] = False
        self._zero_output("feedback_gap")

    def inhibit(self, reason):
        """Immediately suppress output without erasing a valid model or baseline."""
        self._reset_initial_windows()
        self._zero_output(str(reason))

    def _invalid_model(self, reason, samples):
        self._warmup_s = 0.0
        self.filtered = [0.0] * 7
        self._reset_initial_windows()
        self.telemetry.update(
            q_sdk=[s.q for s in samples],
            velocity=[s.velocity for s in samples],
            acceleration=list(self.acceleration),
            model_valid=False,
            model_nm=[None] * 7,
            residual_nm=[None] * 7,
            filtered_residual_nm=[None] * 7,
        )
        self._zero_output(reason, invalid=True)
        return False

    def update(self, samples, target, desired, *, baseline_admission=None):
        samples = tuple(samples)
        if len(samples) != 7 or any(
            type(s.sequence) is not int or s.sequence <= 0
            or not all(math.isfinite(x) for x in (s.q, s.velocity, s.torque, s.temperature, s.stamp))
            for s in samples
        ):
            self._zero_output("nonfinite_or_invalid_feedback", invalid=True)
            raise RuntimeError("Experimental damping requires seven finite real feedback samples")
        now = self.clock()
        if not math.isfinite(now) or any(
            not 0 <= now - s.stamp <= a["max_age_s"] for s, a in zip(samples, self.profile.axes)
        ):
            self.gap()
            return self._invalid_model("feedback_not_fresh", samples)
        if self._last_seen is not None and any(
            s.sequence <= old.sequence or s.stamp <= old.stamp
            for s, old in zip(samples, self._last_seen)
        ):
            self.gap()
            return self._invalid_model("feedback_not_new", samples)
        self._last_seen = samples
        if self.previous is None:
            self.previous = samples
            self._zero_output("warming_up")
            return False
        intervals = [s.stamp - old.stamp for s, old in zip(samples, self.previous)]
        if max(intervals) > self.baseline.max_gap_s:
            self.gap()
            self.previous = samples
            return False
        dt = min(intervals)
        if dt <= 0:
            self.gap()
            return False
        raw_acceleration = [(s.velocity - old.velocity) / step for s, old, step in zip(samples, self.previous, intervals)]
        acceleration = []
        for raw, previous, step, policy in zip(raw_acceleration, self.acceleration, intervals, self.profile.policy):
            value = raw if not self._acceleration_initialized else (
                previous + (1 - math.exp(-step / policy["acceleration_filter_s"])) * (raw - previous)
            )
            acceleration.append(value)
        self.previous = samples
        self.acceleration = acceleration
        self._acceleration_initialized = True
        self.telemetry["raw_acceleration"] = raw_acceleration
        if not all(math.isfinite(value) for value in acceleration + raw_acceleration):
            self._zero_output("nonfinite_acceleration", invalid=True)
            raise RuntimeError("Non-finite experimental acceleration estimate")
        if any(abs(value) > axis["max_acceleration"] for value, axis in zip(acceleration, self.profile.axes)):
            return self._invalid_model("filtered_acceleration_outside_model_range", samples)
        if any(abs(s.velocity) > a["max_velocity"] for s, a in zip(samples, self.profile.axes)):
            return self._invalid_model("velocity_outside_model_range", samples)
        q = [self.profile.to_model(i, s.q) for i, s in enumerate(samples[:6])]
        try:
            theory = self.profile.model.evaluate(q, [s.velocity for s in samples[:6]], acceleration[:6]) + [0.0]
            residual = [s.torque - expected for s, expected in zip(samples, theory)]
            if not all(math.isfinite(value) for value in theory + residual):
                raise RuntimeError("Non-finite experimental dynamics/residual state")
        except Exception:
            self._zero_output("model_evaluation_failed", invalid=True)
            raise
        self._warmup_s += dt
        for i, (value, policy) in enumerate(zip(residual, self.profile.policy)):
            self.filtered[i] += (1 - math.exp(-dt / policy["filter_s"])) * (value - self.filtered[i])
        if not all(math.isfinite(value) for value in self.filtered):
            self._zero_output("nonfinite_filtered_residual", invalid=True)
            raise RuntimeError("Non-finite experimental residual filter state")
        self.telemetry = dict(
            q_sdk=[s.q for s in samples], q_urdf=q,
            velocity=[s.velocity for s in samples], acceleration=list(acceleration),
            raw_acceleration=raw_acceleration,
            measured_nm=[s.torque for s in samples], model_nm=theory,
            residual_nm=residual, filtered_residual_nm=list(self.filtered),
            temperature_c=[s.temperature for s in samples],
            model_valid=True, automatic_hold=False, current_ma=[None] * 7,
            pose=self.pose(q),
        )
        if not self._baseline_collection_closed:
            # Runtime proves native release, command progress and arrival for
            # each group before its stationary observations may qualify.
            self._collect_initial_baselines(samples, residual, baseline_admission)
            return True
        if self._warmup_s + 1e-9 < self._warmup_required_s:
            self._zero_output("warming_up")
            return True
        thresholds = []
        caps = self.effective_caps()
        rise_rates = self.effective_rise_rates()
        for i, (sample, policy, axis, cap, rise_rate) in enumerate(
            zip(samples, self.profile.policy, self.profile.axes, caps, rise_rates)
        ):
            if self._baseline_floor[i] is None:
                thresholds.append(None)
                self.power[i] = self.confirm[i] = self.release[i] = 0.0
                self.active[i] = False
                continue
            base = max(policy["deadzone_nm"], self._baseline_floor[i])
            # The extra allowance is deliberately labeled an engineering
            # choice. A stationary baseline cannot characterize moving error.
            motion = (
                2 * base * min(1.0, abs(sample.velocity) / axis["max_velocity"])
                + 2 * base * min(1.0, abs(acceleration[i]) / axis["max_acceleration"])
            )
            threshold = (
                base + motion + policy["velocity_margin"] * abs(sample.velocity)
                + policy["acceleration_margin"] * abs(acceleration[i])
            )
            thresholds.append(threshold)
            magnitude = abs(self.filtered[i])
            if magnitude > threshold:
                self.confirm[i] += dt
                self.release[i] = 0.0
                if self.confirm[i] >= policy["confirm_s"]:
                    self.active[i] = True
            else:
                self.confirm[i] = 0.0
                self.release[i] += dt
                if self.release[i] >= policy["release_s"]:
                    self.active[i] = False
            fraction = min(1.0, max(0.0, (magnitude - threshold) / policy["full_scale_nm"]))
            requested = cap * fraction if self.active[i] else 0.0
            rate = rise_rate if requested > self.power[i] else policy["fall_mw_s"]
            step = rate * dt
            self.power[i] += max(-step, min(step, requested - self.power[i]))
            self.power[i] = min(cap, max(0.0, self.power[i]))
        self.hold = False
        self._state, self._reason = self._output_state()
        self.telemetry.update(
            deadzone_nm=thresholds, requested_mw=list(self.power), experimental=self.snapshot(),
        )
        return True

    def outputs(self, manual_hold):
        # Software HOLD remains disabled with or without an extra experiment
        # cap. Native button-board arbitration belongs to the runtime.
        if len(self.power) != 7 or any(
            type(power) not in (int, float) or not math.isfinite(power) or power < 0
            for power in self.power
        ):
            self._zero_output("nonfinite_or_invalid_power", invalid=True)
            raise RuntimeError("Experimental power state must contain seven finite nonnegative values")
        return [max(0.0, min(cap, power)) for power, cap in zip(self.power, self.effective_caps())], [False] * 7
