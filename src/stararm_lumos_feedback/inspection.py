"""Read-only DM feedback inspection, without SDK construction or serial access.

The sole transmitted command is the reviewed DM refresh request. Motor status
0 (disabled) is useful diagnostic data, but never evidence for a no-load
baseline. This observer does not control or establish the state of the Leader.
"""

import math
import struct
import threading
import time

from .transport import advance_velocity_interval


class Inspection:
    def __init__(
        self,
        profile,
        *,
        dashboard=None,
        bus=None,
        clock=time.monotonic,
        wall=time.time,
        sleeper=time.sleep,
    ):
        self.profile, self.dashboard = profile, dashboard
        self.clock, self.wall, self.sleeper = clock, wall, sleeper
        self.bus = bus
        self.stop = threading.Event()
        self.samples = [None] * 7
        self.statuses = [None] * 7
        self.temperatures = [None] * 7
        self.counts = [0] * 7
        self.acceleration = [0.0] * 7
        self.velocity_bounds = [None] * 6
        self.requests = 0
        self.parameter_replies = 0
        self.error = None
        self.started = None
        self.finished = None
        self.complete_once = False
        self.coherent = False
        self.last_coherent = None
        if len(profile.axes) != 7:
            raise ValueError("Inspection requires six arm motors and one TypeLJ motor")
        if len({(a["feedback_id"], a["motor_id"]) for a in profile.axes}) != 7:
            raise ValueError("Inspection feedback identities must be unique")
        for axis in profile.axes:
            if type(axis["motor_id"]) is not int or not 0 <= axis["motor_id"] <= 15:
                raise ValueError("Inspection requires reviewed DM motor IDs in 0..15")
        fps = profile.session.get("fps", 30)
        if type(fps) is not int or not 1 <= fps <= 100:
            raise ValueError("Inspection fps must be an integer in 1..100")
        self.period = 1 / fps

    def refresh(self):
        """Request feedback only: never enable, disable, move, or command force."""
        import can

        for axis in self.profile.axes:
            self.bus.send(
                can.Message(
                    arbitration_id=0x7FF,
                    is_extended_id=False,
                    is_fd=False,
                    is_rx=False,
                    data=bytes((axis["motor_id"], 0, 0xCC, 0, 0, 0, 0, 0)),
                ),
                timeout=min(self.period, 0.05),
            )
            self.requests += 1

    def ingest(self, message):
        if message.is_error_frame:
            raise RuntimeError("Inspection received a SocketCAN error frame")
        if (
            message.is_remote_frame
            or message.is_extended_id
            or getattr(message, "is_fd", False)
            or not message.is_rx
            or len(message.data) != 8
        ):
            return
        data = message.data
        matches = [
            i
            for i, axis in enumerate(self.profile.axes)
            if axis["feedback_id"] == message.arbitration_id
            and axis["motor_id"] == (data[0] & 15)
        ]
        if not matches:
            return
        i = matches[0]
        axis = self.profile.axes[i]
        # Match the same verified KT/PMAX/VMAX/TMAX/Imax read response as the
        # active monitor: complete motor ID LE16, 0x33, known RID, positive
        # finite float32 LE. Never count unknown/malformed packets or enabled
        # and fault status bytes as parameters merely because the low ID fits.
        if (
            data[0] == axis["motor_id"]
            and data[1] == 0
            and data[2] == 0x33
            and data[3] in (1, 21, 22, 23, 59)
        ):
            value = struct.unpack("<f", bytes(data[4:8]))[0]
            if math.isfinite(value) and value > 0:
                self.parameter_replies += 1
                return
        age = self.wall() - message.timestamp
        if not math.isfinite(age) or age < -0.02 or age > axis["max_age_s"]:
            raise RuntimeError(f"Axis {i + 1}: inspection queued/stale/invalid CAN timestamp")
        status = data[0] >> 4
        self.statuses[i] = status
        self.temperatures[i] = max(data[6], data[7])
        if status not in (0, 1):
            raise RuntimeError(f"Axis {i + 1}: inspection motor fault/status {status}")
        if self.temperatures[i] >= axis["max_temperature_c"]:
            raise RuntimeError(f"Axis {i + 1}: inspection overtemperature {self.temperatures[i]}")
        previous = self.samples[i]
        if previous is not None and message.timestamp <= previous["kernel_timestamp"]:
            return

        def decode(value, bits, maximum):
            return (value / ((1 << bits) - 1) * 2 - 1) * maximum

        motor_position = decode((data[1] << 8) | data[2], 16, axis["p_max"])
        motor_velocity = decode((data[3] << 4) | (data[4] >> 4), 12, axis["v_max"])
        torque = decode(((data[4] & 15) << 8) | data[5], 12, axis["t_max"])
        torque *= axis["torque_scale"]
        if i == 6:
            self.profile.check_gripper_motor(motor_position, motor_velocity)
        q = self.profile.to_sdk(i, motor_position)
        velocity = self.profile.to_velocity(i, motor_position, motor_velocity)
        if not all(math.isfinite(x) for x in (q, velocity, torque)):
            raise RuntimeError(f"Axis {i + 1}: non-finite inspection state")
        if abs(velocity) > axis["max_velocity"]:
            raise RuntimeError(f"Axis {i + 1}: inspection velocity outside configured range")
        low, high = (self.profile.limits[0][i], self.profile.limits[1][i]) if i < 6 else (0, 1)
        if not low - axis["limit_margin"] <= q <= high + axis["limit_margin"]:
            raise RuntimeError(
                f"Axis {i + 1}: inspection measured position outside limits: "
                f"actual={q:+.12g}, limits=[{low:.12g}, {high:.12g}], "
                f"measurement_margin={axis['limit_margin']:.12g} (SDK coordinates)"
            )
        stamp = self.clock() - max(0, age)
        bounds = advance_velocity_interval(axis, velocity) if i < 6 else None
        if previous is not None:
            if stamp <= previous["stamp"]:
                return
            jump = abs(self.profile.to_model(i, q) - self.profile.to_model(i, previous["q"]))
            if jump > axis["max_jump"]:
                raise RuntimeError(f"Axis {i + 1}: inspection position jump")
            if i == 6 and abs(motor_position - previous["motor_position"]) > self.profile.gripper_transform[
                "max_motor_jump_rad"
            ]:
                raise RuntimeError("Axis 7: inspection raw motor position jump")
            dt = stamp - previous["stamp"]
            acceleration = (velocity - previous["velocity"]) / dt
            kernel_dt = message.timestamp - previous["kernel_timestamp"]
            if i < 6:
                bounds = advance_velocity_interval(axis, velocity, self.velocity_bounds[i], kernel_dt)
                rejected = bounds[0] > bounds[1]
            else:
                rejected = abs(acceleration) > axis["max_acceleration"]
            if rejected:
                raise RuntimeError(
                    f"Axis {i + 1}: inspection acceleration outside configured range: "
                    f"raw_a={acceleration:+.12g}, limit={axis['max_acceleration']:.12g}, "
                    f"dv={velocity - previous['velocity']:+.12g}, dt={kernel_dt:.12g}, "
                    f"previous_v={previous['velocity']:+.12g}, current_v={velocity:+.12g}, "
                    f"code={(data[3] << 4) | (data[4] >> 4)}, feasible_bounds={bounds}"
                )
            filters = getattr(self.profile, "policy", [])
            smoothing = filters[i].get("acceleration_filter_s", 0.05) if filters else 0.05
            if not isinstance(smoothing, (int, float)) or smoothing <= 0:
                smoothing = 0.05
            filtered_acceleration = self.acceleration[i] + (1 - math.exp(-dt / smoothing)) * (
                acceleration - self.acceleration[i]
            )
            if not math.isfinite(filtered_acceleration) or abs(filtered_acceleration) > axis["max_acceleration"]:
                raise RuntimeError(
                    f"Axis {i + 1}: model acceleration estimate outside configured range: "
                    f"filtered_a={filtered_acceleration:+.12g}, raw_a={acceleration:+.12g}, "
                    f"limit={axis['max_acceleration']:.12g}, dt={dt:.12g}"
                )
            self.acceleration[i] = filtered_acceleration
        self.samples[i] = dict(
            q=q,
            velocity=velocity,
            torque=torque,
            stamp=stamp,
            kernel_timestamp=message.timestamp,
            motor_position=motor_position,
            motor_velocity=motor_velocity,
            status=status,
        )
        self.counts[i] += 1
        if i < 6:
            self.velocity_bounds[i] = bounds

    def snapshot(self, *, waiting=False):
        now = self.clock()
        missing = [
            i + 1
            for i, (sample, axis) in enumerate(zip(self.samples, self.profile.axes))
            if sample is None or now - sample["stamp"] > axis["max_age_s"]
        ]
        if missing:
            self.coherent = False
            if waiting:
                return False
            raise RuntimeError(f"Inspection missing/stale CAN feedback: axes {missing}")
        if max(s["stamp"] for s in self.samples) - min(s["stamp"] for s in self.samples) > self.profile.guards[
            "can_skew_s"
        ]:
            self.coherent = False
            if waiting:
                return False
            # A new refresh round arrives one motor at a time. Keep its raw
            # state visible, but never evaluate a mixed group as a dynamics
            # sample. The previous coherent group's lifetime bounds assembly.
            window = min(axis["max_age_s"] for axis in self.profile.axes)
            if self.last_coherent is not None and now - self.last_coherent <= window:
                return False
            raise RuntimeError("Inspection CAN axes are too far apart in time")
        self.complete_once = True
        self.coherent = True
        self.last_coherent = now
        return True

    def summary(self):
        now = self.finished if self.finished is not None else self.clock()
        coverage = [
            s is not None and now - s["stamp"] <= axis["max_age_s"]
            for s, axis in zip(self.samples, self.profile.axes)
        ]
        return dict(
            mode="inspect",
            duration_s=0 if self.started is None else max(0, now - self.started),
            refresh_requests=self.requests,
            valid_frame_counts=list(self.counts),
            ignored_parameter_replies=self.parameter_replies,
            motor_status=list(self.statuses),
            fresh_axis_coverage=coverage,
            coherent_feedback=self.coherent,
            disabled_axes=[i + 1 for i, status in enumerate(self.statuses) if status == 0],
            all_enabled=all(status == 1 for status in self.statuses) and all(coverage) and self.coherent,
            baseline_eligible=False,
            leader_controlled=False,
            leader_output_state="unknown",
            residual_verified=False,
            error=None if self.error is None else str(self.error),
        )

    def publish(self, status):
        if self.dashboard is None:
            return
        now = self.clock()
        state = self.summary()
        state.update(
            status=status,
            requested_mw=[None] * 7,
            predicted_mw=[None] * 7,
            hold_requested=[None] * 7,
            leader_current_ma=[None] * 7,
            current_ma=[None] * 7,
            automatic_hold=None,
            temperature_c=list(self.temperatures),
            q_sdk=[None if s is None else s["q"] for s in self.samples],
            velocity=[None if s is None else s["velocity"] for s in self.samples],
            measured_nm=[None if s is None else s["torque"] for s in self.samples],
            age_ms=[None if s is None else max(0, now - s["stamp"]) * 1000 for s in self.samples],
            model_nm=[None] * 7,
            residual_nm=[None] * 7,
        )
        if self.coherent and all(s is not None for s in self.samples):
            q = [self.profile.to_model(i, s["q"]) for i, s in enumerate(self.samples[:6])]
            theory = self.profile.model.evaluate(
                q, [s["velocity"] for s in self.samples[:6]], self.acceleration[:6]
            ) + [0.0]
            residual = [s["torque"] - value for s, value in zip(self.samples, theory)]
            if not all(math.isfinite(value) for value in theory + residual):
                raise RuntimeError("Inspection dynamics returned non-finite values")
            state.update(q_urdf=q, acceleration=list(self.acceleration), model_nm=theory, residual_nm=residual)
            if hasattr(self.profile.model, "children"):
                from .control import Controller

                state["pose"] = Controller.pose(self, q)
        self.dashboard.publish(**state)

    def run(self, duration=None):
        if duration is not None and (not math.isfinite(duration) or duration <= 0):
            raise ValueError("Inspection duration must be finite and positive")
        if self.started is not None:
            raise RuntimeError("An Inspection instance can only run once")
        if self.bus is None:
            import can

            self.bus = can.Bus(
                interface="socketcan",
                channel=self.profile.session["robot"].get("can_interface", "can0"),
                receive_own_messages=False,
                local_loopback=False,
            )
        self.started = next_refresh = self.clock()
        deadline = self.started + self.profile.guards["startup_feedback_s"]
        finish_at = None if duration is None else self.started + duration
        next_publish = self.started
        try:
            self.publish("inspection_waiting")
            while not self.stop.is_set() and (finish_at is None or self.clock() < finish_at):
                now = self.clock()
                if now >= next_refresh:
                    self.refresh()
                    next_refresh = now + self.period
                remaining = next_refresh - self.clock()
                if finish_at is not None:
                    remaining = min(remaining, finish_at - self.clock())
                was_coherent = self.coherent
                message = self.bus.recv(max(0, min(0.02, remaining)))
                if message is not None:
                    self.ingest(message)
                waiting = not self.complete_once and self.clock() < deadline
                complete = self.snapshot(waiting=waiting)
                if self.clock() >= next_publish or complete and not was_coherent:
                    self.publish("inspection_active" if complete else "inspection_waiting")
                    next_publish = self.clock() + self.period
        except BaseException as exc:
            if not isinstance(exc, KeyboardInterrupt):
                self.error = exc
            self.publish("inspection_stopped" if isinstance(exc, KeyboardInterrupt) else "inspection_error")
            raise
        finally:
            self.finished = self.clock()
            self.bus.shutdown()
        self.publish("inspection_stopped")
        return self.summary()
