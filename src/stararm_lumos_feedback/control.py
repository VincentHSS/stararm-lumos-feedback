"""Per-axis residual observer, power mapping and independent stall HOLD."""

import math
import threading

from .config import number
from .transport import advance_velocity_interval


class Caps:
    def __init__(self, common=1200, axes=None):
        self.lock = threading.Lock()
        self.set({"common_mw": common, "axis_mw": [1200] * 7 if axes is None else axes})

    def set(self, data):
        if not isinstance(data, dict) or set(data) != {"common_mw", "axis_mw"}:
            raise ValueError("Only common_mw and axis_mw may be changed")
        common, axes = data["common_mw"], data["axis_mw"]
        number(common, "common_mw", 400, 1500)
        if not isinstance(axes, list) or len(axes) != 7:
            raise ValueError("axis_mw requires seven numbers")
        for value in axes:
            number(value, "axis_mw", 0, 1500)
        with self.lock:
            self.common, self.axes = common, list(axes)

    def get(self):
        with self.lock:
            return dict(common_mw=self.common, axis_mw=list(self.axes))

    def effective(self):
        data = self.get()
        return [min(data["common_mw"], x) for x in data["axis_mw"]]


class Controller:
    def __init__(self, profile, caps):
        self.profile, self.caps = profile, caps
        self.previous = None
        self.velocity_bounds = [None] * 6
        self.filtered = [0.0] * 7
        self.acceleration = [0.0] * 7
        self.confirm = [0.0] * 7
        self.release = [0.0] * 7
        self.active = [False] * 7
        self.power = [0.0] * 7
        self.stall = [0.0] * 7
        self.last_error = None
        self.hold = False
        self.hold_release = 0.0
        self.telemetry = {}

    def gap(self):
        # Gaps cannot accumulate load/stall/release confirmation.
        self.confirm = [0.0] * 7
        self.release = [0.0] * 7
        self.stall = [0.0] * 7
        self.last_error = None
        self.hold_release = 0.0
        self.previous = None
        self.velocity_bounds = [None] * 6

    def update(self, samples, target, desired):
        if self.previous is None:
            self.velocity_bounds = [
                advance_velocity_interval(a, s.velocity)
                for a, s in zip(self.profile.axes[:6], samples[:6])
            ]
            self.previous = samples
            return False
        if any(s.sequence <= p.sequence for s, p in zip(samples, self.previous)):
            return False
        intervals = [s.stamp - p.stamp for s, p in zip(samples, self.previous)]
        dt = min(intervals)
        if dt <= 0:
            return False
        policy, axes = self.profile.policy, self.profile.axes
        bounds = list(self.velocity_bounds)
        for i, (s, old, step, p, a) in enumerate(zip(samples, self.previous, intervals, policy, axes)):
            raw_acceleration = (s.velocity - old.velocity) / step
            if i < 6:
                kernel_step = (
                    s.kernel_timestamp - old.kernel_timestamp
                    if s.kernel_timestamp is not None and old.kernel_timestamp is not None else step
                )
                bounds[i] = advance_velocity_interval(a, s.velocity, self.velocity_bounds[i], kernel_step)
                rejected = bounds[i][0] > bounds[i][1]
            else:
                kernel_step = step
                rejected = abs(raw_acceleration) > a["max_acceleration"]
            if rejected:
                raise RuntimeError(
                    f"Axis {i + 1}: acceleration outside configured model range: "
                    f"raw_a={raw_acceleration:+.12g}, limit={a['max_acceleration']:.12g}, "
                    f"dv={s.velocity - old.velocity:+.12g}, dt={kernel_step:.12g}, "
                    f"previous_v={old.velocity:+.12g}, current_v={s.velocity:+.12g}, "
                    f"sequence={old.sequence}->{s.sequence}, feasible_bounds={bounds[i] if i < 6 else None}"
                )
        acceleration = list(self.acceleration)
        for i, (s, old, step, p, a) in enumerate(zip(samples, self.previous, intervals, policy, axes)):
            raw_acceleration = (s.velocity - old.velocity) / step
            acceleration[i] += (1 - math.exp(-step / p["acceleration_filter_s"])) * (
                raw_acceleration - acceleration[i]
            )
            if not math.isfinite(acceleration[i]) or abs(acceleration[i]) > a["max_acceleration"]:
                raise RuntimeError(
                    f"Axis {i + 1}: model acceleration estimate outside configured range: "
                    f"filtered_a={acceleration[i]:+.12g}, raw_a={raw_acceleration:+.12g}, "
                    f"limit={a['max_acceleration']:.12g}, dt={step:.12g}"
                )
        q = [self.profile.to_model(i, s.q) for i, s in enumerate(samples[:6])]
        theory = self.profile.model.evaluate(q, [s.velocity for s in samples[:6]], acceleration[:6]) + [
            0.0
        ]
        residual = [s.torque - t for s, t in zip(samples, theory)]
        if not all(math.isfinite(value) for value in theory + residual + acceleration):
            raise RuntimeError("Non-finite dynamics/residual state")
        self.acceleration = acceleration
        caps = self.caps.effective()
        thresholds, load = [], []
        for i, (r, s, p, cap) in enumerate(zip(residual, samples, policy, caps)):
            self.filtered[i] += (1 - math.exp(-dt / p["filter_s"])) * (r - self.filtered[i])
            threshold = (
                p["deadzone_nm"]
                + p["velocity_margin"] * abs(s.velocity)
                + p["acceleration_margin"] * abs(self.acceleration[i])
            )
            thresholds.append(threshold)
            magnitude = abs(self.filtered[i])
            load.append(magnitude)
            if magnitude > threshold:
                self.confirm[i] += dt
                self.release[i] = 0
                if self.confirm[i] >= p["confirm_s"]:
                    self.active[i] = True
            else:
                self.confirm[i] = 0
                self.release[i] += dt
                if self.release[i] >= p["release_s"]:
                    self.active[i] = False
            fraction = min(1.0, max(0.0, (magnitude - threshold) / p["full_scale_nm"]))
            requested = cap * fraction if self.active[i] else 0
            step = (p["rise_mw_s"] if requested > self.power[i] else p["fall_mw_s"]) * dt
            self.power[i] += max(-step, min(step, requested - self.power[i]))
            # Lowered/disabled caps take effect immediately, independently of falling rate.
            self.power[i] = min(cap, max(0, self.power[i]))
        # Position error is in URDF radians; the gripper error is normalized opening.
        errors = [
            abs(self.profile.to_model(i, t) - self.profile.to_model(i, s.q))
            for i, (t, s) in enumerate(zip(target, samples))
        ]
        desired_errors = [
            abs(self.profile.to_model(i, t) - self.profile.to_model(i, s.q))
            for i, (t, s) in enumerate(zip(desired, samples))
        ]
        for i, (error, s, p) in enumerate(zip(errors, samples, policy)):
            no_progress = (
                self.last_error is not None and self.last_error[i] - error <= p["hold_progress"] * dt
            )
            stalled = (
                error >= p["hold_error"]
                and load[i] >= p["hold_load_nm"]
                and abs(s.velocity) <= p["hold_speed"]
                and no_progress
            )
            self.stall[i] = self.stall[i] + dt if stalled else 0
        if not self.hold and any(t >= p["hold_confirm_s"] for t, p in zip(self.stall, policy)):
            self.hold = True
            self.hold_release = 0
        elif self.hold:
            released = all(
                error <= p["hold_release_error"] and force <= p["hold_release_load_nm"]
                for error, force, p in zip(desired_errors, load, policy)
            )
            self.hold_release = self.hold_release + dt if released else 0
            if self.hold_release >= max(p["hold_release_s"] for p in policy):
                self.hold = False
                self.stall = [0.0] * 7
        self.last_error, self.previous, self.velocity_bounds = errors, samples, bounds
        self.telemetry = dict(
            q_sdk=[s.q for s in samples],
            q_urdf=q,
            velocity=[s.velocity for s in samples],
            acceleration=list(self.acceleration),
            measured_nm=[s.torque for s in samples],
            model_nm=theory,
            residual_nm=residual,
            filtered_residual_nm=list(self.filtered),
            deadzone_nm=thresholds,
            temperature_c=[s.temperature for s in samples],
            requested_mw=list(self.power),
            automatic_hold=self.hold,
            current_ma=[None] * 7,
            # DM current is unavailable without a verified motor torque constant.
            pose=self.pose(q),
        )
        return True

    def pose(self, q):
        from .model import I3, ZERO, add, mm, mv, rotation

        points, edges = {}, []

        def walk(link, r, p):
            points[link] = p
            for child, idx, (lr, lp), axis in self.profile.model.children.get(link, []):
                rr, pp = mm(r, lr), add(p, mv(r, lp))
                if idx is not None:
                    rr = mm(rr, rotation(axis, q[idx]))
                edges.append([link, child])
                walk(child, rr, pp)

        walk(self.profile.model.root, I3, ZERO)
        return dict(points=points, edges=edges)

    def outputs(self, manual_hold):
        # HOLD is independent of all ordinary caps, including zero.
        return [min(p, cap) for p, cap in zip(self.power, self.caps.effective())], [
            self.hold or (manual_hold and i < 6) for i in range(7)
        ]
