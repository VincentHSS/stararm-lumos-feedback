"""Bounded in-memory stationary residual observations and local envelopes.

Stationarity is not evidence of an unloaded arm or of complete model mass.
Nothing here subtracts a fitted mean, changes controller tuning, writes a file,
or authorizes Leader output.
"""

import math
from collections import deque

from .gripper import TypeLJ04041254

POSE_BIN_RAD = math.radians(15)  # Coverage summary resolution, not an acceptance threshold.


def _percentile(values, fraction):
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    low = math.floor(index)
    high = math.ceil(index)
    return ordered[low] + (ordered[high] - ordered[low]) * (index - low)


class BaselineEstimator:
    def __init__(self, profile, *, minimum_stationary_s=2.0, max_samples=4096):
        if not math.isfinite(minimum_stationary_s) or minimum_stationary_s < 2:
            raise ValueError("A stationary baseline requires at least 2 seconds")
        fps = profile.session.get("fps", 30)
        if type(fps) is not int or not 1 <= fps <= 100:
            raise ValueError("Baseline observation requires integer fps in 1..100")
        if type(max_samples) is not int or max_samples < math.ceil(minimum_stationary_s * fps) + 1:
            raise ValueError("Baseline capacity must hold a complete stationary window")
        self.profile = profile
        self.minimum_stationary_s = minimum_stationary_s
        self.capacity = max_samples
        self.max_gap_s = min(min(a["max_age_s"] for a in profile.axes), max(0.05, 3 / fps))
        self.quantization = []
        # Analytic upper bound on |d normalized opening / d motor angle| over
        # this fixed TypeLJ converter's complete physical range.
        y_max = 0.0355 - 0.015393
        grip_jacobian = (
            2
            * (y_max * 0.0355 / math.sqrt(0.001444 - y_max * y_max) + 0.0355)
            * abs(TypeLJ04041254.angle_span / TypeLJ04041254.motor_open_rad)
            / TypeLJ04041254.width_m
        )
        for i, a in enumerate(profile.axes):
            scale = abs(a["sdk_scale"] * a["urdf_scale"]) if i < 6 else grip_jacobian
            self.quantization.append(
                dict(
                    position=2 * a["p_max"] / 65535 * scale,
                    velocity=2 * a["v_max"] / 4095 * scale,
                    torque_nm=2 * a["t_max"] / 4095 * abs(a["torque_scale"]),
                )
            )
        self._rows = deque(maxlen=max_samples)
        self._pending = deque(maxlen=max_samples)
        self._window_rows = deque(maxlen=max_samples)
        self._seen = None
        self._previous = None
        self._anchor = None
        self._continuous_s = 0.0
        self._qualified = False
        self.rejected_nonstationary = 0
        self.rejected_cached_or_gap = 0

    def gap(self):
        """Break continuity while retaining already qualified observations."""
        self._previous = self._anchor = None
        self._pending.clear()
        self._window_rows.clear()
        self._continuous_s = 0.0
        self._qualified = False

    def _pose(self, samples):
        return [self.profile.to_model(i, s.q) for i, s in enumerate(samples[:6])] + [samples[6].q]

    def update(self, samples, residual_nm):
        samples = tuple(samples)
        if len(samples) != 7 or len(residual_nm) != 7 or not all(math.isfinite(x) for x in residual_nm):
            raise ValueError("Baseline requires seven finite residuals and real samples")
        if not all(
            type(s.sequence) is int
            and s.sequence > 0
            and all(math.isfinite(x) for x in (s.q, s.velocity, s.torque, s.stamp))
            for s in samples
        ):
            raise ValueError("Baseline requires finite positions, velocities and real sequence/timestamps")
        if self._seen is not None and any(
            s.sequence <= p.sequence or s.stamp <= p.stamp for s, p in zip(samples, self._seen)
        ):
            self.rejected_cached_or_gap += 1
            self.gap()
            return False
        self._seen = samples
        pose = self._pose(samples)
        stationary = all(abs(s.velocity) <= 4 * q["velocity"] for s, q in zip(samples, self.quantization))
        motor_velocity = samples[6].motor_velocity
        if motor_velocity is None or not math.isfinite(motor_velocity):
            stationary = False
        elif abs(motor_velocity) > 4 * 2 * self.profile.axes[6]["v_max"] / 4095:
            stationary = False  # Opening clamps cannot hide a moving gripper motor.
        if self._anchor is not None and any(
            abs(q - a) > 8 * resolution["position"]
            for q, a, resolution in zip(pose, self._anchor, self.quantization)
        ):
            stationary = False
        if not stationary:
            self.rejected_nonstationary += 1
            self.gap()
            return False
        intervals = [s.stamp - p.stamp for s, p in zip(samples, self._previous)] if self._previous else []
        if intervals and max(intervals) > self.max_gap_s:
            self.rejected_cached_or_gap += 1
            self.gap()
            intervals = []
        credit = min(intervals) if intervals else 0.0
        self._continuous_s += credit
        if self._anchor is None:
            self._anchor = pose
        self._previous = samples
        # Keep absolute observations unchanged. Signed residual/model ranges
        # are separate reference data, never a fitted correction to the model.
        model_nm = [s.torque - residual for s, residual in zip(samples, residual_nm)]
        if not all(math.isfinite(value) for value in model_nm):
            raise ValueError("Baseline requires finite model torque observations")
        row = (pose, [abs(x) for x in residual_nm], credit, list(residual_nm), model_nm)
        if self._qualified:
            self._rows.append(row)
            self._window_rows.append(row)
        else:
            self._pending.append(row)
            if self._continuous_s + 1e-9 >= self.minimum_stationary_s:
                self._rows.extend(self._pending)
                self._window_rows.extend(self._pending)
                self._pending.clear()
                self._qualified = True
        return True

    def snapshot(self):
        count = len(self._rows)
        retained_s = sum(row[2] for row in self._rows)
        ready = count > 1 and retained_s + 1e-9 >= self.minimum_stationary_s
        if ready:
            columns = [[row[1][i] for row in self._rows] for i in range(7)]
            maxima = [max(values) for values in columns]
            p95 = [_percentile(values, 0.95) for values in columns]
            p99 = [_percentile(values, 0.99) for values in columns]
            recommended = [q + 4 * step["torque_nm"] for q, step in zip(p99, self.quantization)]
            lower = [min(row[0][i] for row in self._rows) for i in range(7)]
            upper = [max(row[0][i] for row in self._rows) for i in range(7)]
            bins = len({tuple(math.floor(q / POSE_BIN_RAD) for q in row[0][:6]) for row in self._rows})
            signed_min = [min(row[3][i] for row in self._rows) for i in range(7)]
            signed_max = [max(row[3][i] for row in self._rows) for i in range(7)]
            model_min = [min(row[4][i] for row in self._rows) for i in range(7)]
            model_max = [max(row[4][i] for row in self._rows) for i in range(7)]
        else:
            maxima = p95 = p99 = recommended = lower = upper = None
            signed_min = signed_max = model_min = model_max = None
            bins = 0
        window_s = sum(row[2] for row in self._window_rows)
        window_ready = (
            self._qualified and len(self._window_rows) > 1
            and window_s + 1e-9 >= self.minimum_stationary_s
        )
        window = dict(
            ready=window_ready,
            sample_count=len(self._window_rows),
            retained_stationary_s=window_s,
            coordinate_min=None,
            coordinate_max=None,
            signed_residual_min_nm=None,
            signed_residual_max_nm=None,
            model_min_nm=None,
            model_max_nm=None,
        )
        if window_ready:
            for key, column in (
                ("coordinate", 0), ("signed_residual", 3), ("model", 4),
            ):
                unit = "" if key == "coordinate" else "_nm"
                window[f"{key}_min{unit}"] = [min(row[column][i] for row in self._window_rows) for i in range(7)]
                window[f"{key}_max{unit}"] = [max(row[column][i] for row in self._window_rows) for i in range(7)]
        return dict(
            label="stationary_not_unloaded_verified",
            ready=ready,
            sample_count=count,
            pending_stationary_sample_count=len(self._pending),
            minimum_stationary_s=self.minimum_stationary_s,
            continuous_stationary_s=self._continuous_s,
            retained_stationary_s=retained_s,
            current_window_qualified=self._qualified,
            storage_capacity=self.capacity,
            rejected_nonstationary=self.rejected_nonstationary,
            rejected_cached_or_gap=self.rejected_cached_or_gap,
            stationary_velocity_threshold=[4 * q["velocity"] for q in self.quantization],
            quantization=[dict(q) for q in self.quantization],
            absolute_residual_max_nm=maxima,
            absolute_residual_p95_nm=p95,
            absolute_residual_p99_nm=p99,
            signed_residual_min_nm=signed_min,
            signed_residual_max_nm=signed_max,
            model_min_nm=model_min,
            model_max_nm=model_max,
            current_window_reference=window,
            recommended_deadzone_nm=recommended,
            recommendation_method="absolute residual p99 + 4 decoded torque LSBs; mean is never removed",
            pose_coverage=dict(
                arm_bin_size_rad=POSE_BIN_RAD,
                arm_pose_bins=bins,
                coordinate_min=lower,
                coordinate_max=upper,
                units="URDF radians J1-J6; normalized gripper opening",
                coverage_accepted=False,
            ),
            unloaded_verified=False,
            mass_coverage_verified_by_estimator=False,
            profile_mass_coverage_confirmed=bool(getattr(self.profile, "mass_confirmed", False)),
            force_authorized=False,
            policy_changed=False,
        )
