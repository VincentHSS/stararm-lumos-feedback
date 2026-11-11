"""Passive DM MIT CAN feedback and a single-owner Fashionstar serial transport."""

import math
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass

ARM_HISTORY_MAX_SAMPLES = 256


def advance_velocity_interval(axis, velocity, previous_interval=None, dt=None):
    """Reachable velocity bin for one linear arm axis; lower > upper rejects.

    Retain the returned intersection across every accepted sample. Expanding
    the previous full bin instead would miss a rapid one-code staircase.
    """
    velocity_lsb = 2 * axis["v_max"] / 4095 * abs(axis["sdk_scale"] * axis["urdf_scale"])
    half_lsb = math.nextafter(velocity_lsb / 2, math.inf)
    lower = math.nextafter(velocity - half_lsb, -math.inf)
    upper = math.nextafter(velocity + half_lsb, math.inf)
    if previous_interval is not None:
        if dt is None or not math.isfinite(dt) or dt <= 0:
            raise ValueError("Velocity interval requires a finite positive kernel interval")
        radius = math.nextafter(axis["max_acceleration"] * dt, math.inf)
        lower = max(lower, math.nextafter(previous_interval[0] - radius, -math.inf))
        upper = min(upper, math.nextafter(previous_interval[1] + radius, math.inf))
    return lower, upper


@dataclass(frozen=True)
class Sample:
    q: float  # SDK radians, or normalized gripper opening
    velocity: float  # URDF rad/s; gripper normalized opening/s
    torque: float  # URDF joint-output Nm; gripper motor-output Nm
    temperature: float
    stamp: float  # monotonic derived from kernel receive timestamp
    sequence: int
    motor_position: float | None = None  # TypeLJ motor radians, before opening clamps
    motor_velocity: float | None = None  # TypeLJ motor rad/s
    kernel_timestamp: float | None = None  # Original SocketCAN receive timestamp


class CanMonitor:
    def __init__(self, profile, bus=None, clock=time.monotonic, wall=time.time, *, mode="monitor"):
        if mode not in ("follow", "monitor", "run"):
            raise ValueError("mode must be follow, monitor or run")
        self.profile, self.clock, self.wall = profile, clock, wall
        self.mode = mode
        # This derivative is a dynamics-model guard. Engineering basic follow
        # does not evaluate that model; its unreviewed acceleration threshold
        # must not reject otherwise valid arm feedback. TypeLJ stays strict.
        self._arm_acceleration_guard = not (
            getattr(profile, "engineering", False) is True and mode == "follow"
        )
        if bus is None:
            import can

            bus = can.Bus(
                interface="socketcan",
                channel=profile.session["robot"].get("can_interface", "can0"),
                receive_own_messages=False,
                local_loopback=False,
            )
        self.bus = bus
        self.samples = [None] * 7
        self.kernel_stamps = [None] * 7
        self.velocity_bounds = [None] * 6
        self._arm_history = [deque(maxlen=ARM_HISTORY_MAX_SAMPLES) for _ in range(6)]
        self.lock = threading.RLock()
        self.error = None
        self.stop = threading.Event()
        self.thread = None
        self.armed = False

    def ingest(self, msg):
        if msg.is_error_frame:
            raise RuntimeError("SocketCAN error frame")
        if (
            msg.is_remote_frame
            or msg.is_extended_id
            or len(msg.data) != 8
            or getattr(msg, "is_fd", False)
            or not msg.is_rx
        ):
            return
        matches = [
            i
            for i, a in enumerate(self.profile.axes)
            if a["feedback_id"] == msg.arbitration_id and a["motor_id"] == (msg.data[0] & 15)
        ]
        if not matches:
            return
        i = matches[0]
        a, b = self.profile.axes[i], msg.data
        # DM register-read responses share the feedback CAN ID. Exclude only
        # verified KT/PMAX/VMAX/TMAX/Imax reads: [motor_id LE16, 0x33, RID,
        # positive finite float32 LE]. SDK startup also reads KT and Imax.
        # Neither parameters nor write acknowledgements are mechanical samples;
        # only this exact read format is ignored. Other status-0 packets still
        # fail once armed. The full ID check must not mask enabled/fault states.
        if b[0] == a["motor_id"] and b[1] == 0 and b[2] == 0x33 and b[3] in (1, 21, 22, 23, 59):
            value = struct.unpack("<f", bytes(b[4:8]))[0]
            if math.isfinite(value) and value > 0:
                return
        now = self.clock()
        age = self.wall() - msg.timestamp
        if not math.isfinite(age) or age < -0.02 or age > a["max_age_s"]:
            raise RuntimeError(f"Axis {i + 1}: queued/stale/invalid CAN timestamp")
        stamp = now - max(0, age)
        status = b[0] >> 4
        if status != 1:
            if not self.armed and status == 0:
                return  # SDK may report disabled motors during its initialization handshake
            raise RuntimeError(f"Axis {i + 1}: disabled or motor fault {status}")
        temperature = max(b[6], b[7])
        if temperature >= a["max_temperature_c"]:
            raise RuntimeError(f"Axis {i + 1}: overtemperature {temperature}")

        def decode(n, bits, maximum):
            return (n / ((1 << bits) - 1) * 2 - 1) * maximum

        p = decode((b[1] << 8) | b[2], 16, a["p_max"])
        velocity_code = (b[3] << 4) | (b[4] >> 4)
        v = decode(velocity_code, 12, a["v_max"])
        torque = decode(((b[4] & 15) << 8) | b[5], 12, a["t_max"]) * a["torque_scale"]
        motor_position, motor_velocity = None, None
        if i == 6:
            motor_position, motor_velocity = p, v
            self.profile.check_gripper_motor(motor_position, motor_velocity)
        q = self.profile.to_sdk(i, p)
        velocity = self.profile.to_velocity(i, p, v)
        if abs(velocity) > a["max_velocity"]:
            raise RuntimeError(f"Axis {i + 1}: velocity outside configured range")
        if i < 6:
            lo, hi = self.profile.limits[0][i], self.profile.limits[1][i]
        else:
            lo, hi = 0, 1
        if not lo - a["limit_margin"] <= q <= hi + a["limit_margin"]:
            raise RuntimeError(f"Axis {i + 1}: measured position outside limits")
        with self.lock:
            previous = self.samples[i]
            if self.kernel_stamps[i] is not None and msg.timestamp <= self.kernel_stamps[i]:
                return  # Compare original timestamps: clock conversion rounding must not refresh a repeat.
            bounds = None
            velocity_lsb = abs(self.profile.to_velocity(i, p, 2 * a["v_max"] / 4095))
            if i < 6:
                bounds = advance_velocity_interval(a, velocity)
            if previous is not None:
                if stamp <= previous.stamp:
                    return  # repeated frames never refresh freshness or confirmation
                # Guard genuine receives between observer updates. Kernel
                # intervals avoid poll/clock-conversion jitter; duplicates
                # were rejected above.
                interval = msg.timestamp - self.kernel_stamps[i]
                delta_velocity = velocity - previous.velocity
                acceleration = delta_velocity / interval
                previous_bounds = self.velocity_bounds[i] if i < 6 else None
                reachable = None
                bin_bounds = bounds
                if i < 6 and self._arm_acceleration_guard:
                    # Every decoded bin represents one velocity-code width.
                    # A fixed half-code offset also covers a floor quantizer:
                    # translation leaves all acceleration constraints unchanged.
                    radius = math.nextafter(a["max_acceleration"] * interval, math.inf)
                    reachable = (
                        math.nextafter(previous_bounds[0] - radius, -math.inf),
                        math.nextafter(previous_bounds[1] + radius, math.inf),
                    )
                    bounds = advance_velocity_interval(a, velocity, previous_bounds, interval)
                    rejected = bounds[0] > bounds[1]
                elif i == 6:
                    # TypeLJ's changing Jacobian is nonlinear; retain its
                    # original point-derivative guard without a bin allowance.
                    rejected = abs(acceleration) > a["max_acceleration"]
                else:
                    rejected = False
                if rejected:
                    raise RuntimeError(
                        f"Axis {i + 1}: acceleration outside configured range: "
                        f"raw_a={acceleration:+.12g}, limit={a['max_acceleration']:.12g}, "
                        f"dv={delta_velocity:+.12g}, dt={interval:.12g}, "
                        f"previous_v={previous.velocity:+.12g}, current_v={velocity:+.12g}, "
                        f"code={velocity_code}, LSB={velocity_lsb:.12g}, "
                        f"sequence={previous.sequence}->{previous.sequence + 1}, armed={self.armed}, "
                        f"feasible_before={previous_bounds}, reachable_bounds={reachable}, "
                        f"bin_bounds={bin_bounds}, feasible_bounds={bounds}"
                    )
                delta = abs(self.profile.to_model(i, q) - self.profile.to_model(i, previous.q))
                if delta > a["max_jump"]:
                    raise RuntimeError(f"Axis {i + 1}: position jump")
                if (
                    i == 6
                    and abs(motor_position - previous.motor_position)
                    > self.profile.gripper_transform["max_motor_jump_rad"]
                ):
                    raise RuntimeError("Axis 7: raw motor position jump")
            self.samples[i] = Sample(
                q,
                velocity,
                torque,
                temperature,
                stamp,
                1 if previous is None else previous.sequence + 1,
                motor_position,
                motor_velocity,
                msg.timestamp,
            )
            self.kernel_stamps[i] = msg.timestamp
            if i < 6:
                self.velocity_bounds[i] = bounds
                history = self._arm_history[i]
                history.append(self.samples[i])
                while history and now - history[0].stamp > a["max_age_s"]:
                    history.popleft()

    def recent_arm_history(self, *, ended_at):
        """Return accepted CAN samples preceding a getter's completion.

        History never supplies telemetry freshness or an SDK timestamp. Both
        the getter window and the current query time retain the original CAN
        age limit; returned immutable snapshots cannot change the live queues.
        """
        if type(ended_at) not in (int, float) or not math.isfinite(ended_at):
            raise ValueError("Arm history requires a finite getter completion time")
        now = self.clock()
        with self.lock:
            if self.error is not None:
                raise RuntimeError(f"CAN monitor stopped: {self.error}") from self.error
            result = []
            for history, axis in zip(self._arm_history, self.profile.axes[:6]):
                while history and now - history[0].stamp > axis["max_age_s"]:
                    history.popleft()
                result.append(tuple(
                    sample for sample in history
                    if 0 <= ended_at - sample.stamp <= axis["max_age_s"]
                    and 0 <= now - sample.stamp <= axis["max_age_s"]
                ))
            return tuple(result)

    def start(self):
        def receive():
            try:
                while not self.stop.is_set():
                    msg = self.bus.recv(0.02)
                    if msg is not None:
                        self.ingest(msg)
            except Exception as exc:
                with self.lock:
                    self.error = exc

        self.thread = threading.Thread(target=receive, name="lumos-passive-can", daemon=True)
        self.thread.start()

    def snapshot(self):
        with self.lock:
            if self.error is not None:
                raise RuntimeError(f"CAN monitor stopped: {self.error}") from self.error
            samples = tuple(self.samples)
        now = self.clock()
        for i, (sample, a) in enumerate(zip(samples, self.profile.axes)):
            if sample is None or now - sample.stamp > a["max_age_s"]:
                raise RuntimeError(f"Axis {i + 1}: missing/stale CAN feedback")
        if max(s.stamp for s in samples) - min(s.stamp for s in samples) > self.profile.guards["can_skew_s"]:
            raise RuntimeError("CAN feedback axes are too far apart in time")
        return samples

    def close(self):
        self.stop.set()
        if self.thread is not None:
            self.thread.join(timeout=0.1)
        self.bus.shutdown()


class LeaderGap(RuntimeError):
    """No complete new Leader reply; cached values must not move the Follower."""

    def __init__(self, message, *, missing_ids=()):
        super().__init__(message)
        self.missing_ids = tuple(missing_ids)


def packet(command, parameters):
    body = b"\x12\x4c" + bytes((command, len(parameters))) + parameters
    return body + bytes((sum(body) & 255,))


class Fashionstar:
    def __init__(
        self, uart, ids, *, enabled, timeout, clock=time.monotonic,
        before_control_write=None, command_button=None, max_output_mw=1500, protect_native_hold=False,
        allow_hold=True,
    ):
        if (type(max_output_mw) not in (int, float) or not math.isfinite(max_output_mw)
                or not 1 <= max_output_mw <= 1500):
            raise ValueError("max_output_mw must be finite and in 1..1500")
        if type(protect_native_hold) is not bool:
            raise ValueError("protect_native_hold must be a boolean")
        if type(allow_hold) is not bool:
            raise ValueError("allow_hold must be a boolean")
        if protect_native_hold:
            # Import only for this opt-in adapter path; ordinary offline CAN
            # monitoring does not require the LeRobot plugin's registry.
            from lerobot_robot_lumos_stararm102.button_protocol import CommandButtonState

            if not isinstance(command_button, CommandButtonState):
                raise ValueError("Native hold protection requires a CommandButtonState RX parser")
        self.uart, self.ids = uart, tuple(ids)
        self.enabled, self.timeout, self.clock = enabled, timeout, clock
        self.lock = threading.RLock()
        self.managed = set()
        self.fault = None
        self.last_commands = {}
        self.before_control_write = before_control_write
        self.command_button = command_button
        self.max_output_mw = max_output_mw
        self.protect_native_hold = protect_native_hold
        self.allow_hold = allow_hold
        self.native_output_unknown = False
        self._native_arm_write_group = None
        self._native_unknown_after_group = None
        self.uart.write_timeout = timeout

    def _write(self, data):
        if self.uart.write(data) != len(data):
            raise RuntimeError("Partial Leader serial write")

    def _forget_native_arm(self):
        confirmed_takeover = self._native_confirmed_takeover()
        if self.managed.intersection(range(6)) and not confirmed_takeover:
            self.native_output_unknown = True
            self._native_unknown_after_group = self.command_button.completed_groups
        self.managed.difference_update(range(6))
        for sid in range(6):
            self.last_commands.pop(sid, None)

    def _native_confirmed_takeover(self):
        complete = (
            self.command_button.verified is True
            and not self.command_button.pending
            and (self.command_button.locked is True or self.command_button.locked is False)
        )
        groups = self.command_button.completed_groups
        if (complete and self.native_output_unknown
                and groups > self._native_unknown_after_group):
            self.native_output_unknown = False
            self._native_unknown_after_group = None
        return complete and (
            self._native_arm_write_group is None or groups > self._native_arm_write_group
        )

    def observe_native_state(self):
        """Bounded RX-only observation; return whether arm software output is allowed."""
        with self.lock:
            return self._native_arm_allowed()

    def receive_native_commands(self):
        """Consume bounded native RX without querying motors or transmitting."""
        with self.lock:
            if self.command_button is None:
                return
            try:
                deadline, remaining = self.clock() + self.timeout, 4096
                # A queued final frame cannot complete an already expired
                # group. Check the original parser deadline before reading.
                self.command_button.update()
                while self.uart.in_waiting:
                    if remaining <= 0 or self.clock() >= deadline:
                        raise LeaderGap("Native button input did not drain before Leader output")
                    received = self.uart.read(min(remaining, self.uart.in_waiting))
                    if not received:
                        raise LeaderGap("Native button input did not drain before Leader output")
                    remaining -= len(received)
                    self.command_button.feed(received)
                if self.clock() >= deadline:
                    raise LeaderGap("Native button input did not drain before Leader output")
                self.command_button.update()
            except Exception as exc:
                self._forget_native_arm()
                if self.fault is None:
                    self.fault = str(exc)
                raise

    def settle_native_commands(self, cancel_check=None):
        """Finish an in-flight RX group before initialization, within its deadline."""
        with self.lock:
            if self.command_button is None:
                return
            while True:
                if cancel_check is not None:
                    cancel_check()
                self.receive_native_commands()
                if not self.command_button.pending:
                    return
                # The parser retains the first byte's original 250 ms group
                # deadline. Polling never starts a new group or extends it.
                time.sleep(0.0005)

    def _native_arm_allowed(self):
        if not self.protect_native_hold:
            return True
        # Observe real bytes again at every write. This remains bounded
        # observation rather than atomic bus arbitration.
        self.receive_native_commands()
        self._native_confirmed_takeover()
        allowed = (
            not self.command_button.pending
            and self.command_button.locked is False
            and self.command_button.verified is True
            and not self.native_output_unknown
        )
        if not allowed:
            # The physical board owns any hold in this state. Even a cleanup
            # release must not undo it, nor retain a command-cache claim.
            self._forget_native_arm()
        return allowed

    def query(self, ids):
        with self.lock:
            if self.fault:
                raise RuntimeError(self.fault)
            if self.command_button is None:
                self.uart.reset_input_buffer()
            else:
                # Native HD boards emit button commands between queries. Drain
                # those RX bytes into their parser, discarding old monitor
                # replies before issuing this query. A UART reset would lose
                # a press/release event; partial button frames survive in the
                # independent parser. No received command is retransmitted.
                deadline = self.clock() + self.timeout
                while self.uart.in_waiting:
                    self.command_button.feed(self.uart.read(min(4096, self.uart.in_waiting)))
                    if self.clock() >= deadline:
                        raise LeaderGap("Leader input did not drain before the new query")
                self.command_button.update()
            self._write(packet(25, bytes((22, 1, len(ids), *ids))))
            deadline, buffer, samples = self.clock() + self.timeout, bytearray(), {}
            while self.clock() < deadline:
                received = self.uart.read(max(1, min(4096, self.uart.in_waiting)))
                if self.command_button is not None:
                    self.command_button.feed(received)
                    self.command_button.update()
                buffer.extend(received)
                while len(buffer) >= 5:
                    starts = [index for header in (b"\x05\x1c", b"\x12\x4c") if (index := buffer.find(header)) >= 0]
                    if not starts:
                        buffer[:] = buffer[-1:]
                        break
                    index = min(starts)
                    if index:
                        del buffer[:index]
                    if len(buffer) < 5:
                        break
                    request = buffer[:2] == b"\x12\x4c"
                    if request and buffer[2:4] != b"\x18\x04":
                        del buffer[0]
                        continue
                    size = buffer[3] + 5
                    if len(buffer) < size:
                        break
                    frame = bytes(buffer[:size])
                    if sum(frame[:-1]) & 255 != frame[-1]:
                        del buffer[0]
                        continue
                    del buffer[:size]
                    # Skip whole native commands so reply-looking payload
                    # bytes cannot become a mechanical monitor packet.
                    if request or frame[2:4] != bytes((22, 16)):
                        continue
                    sid, voltage, current, power, temperature, status, angle, turn = struct.unpack(
                        "<BHHHHBih", frame[4:-1]
                    )
                    if sid not in ids or (angle == -235929599 and turn == 0):
                        continue
                    # Monitor status bit definitions vary by firmware. Do not guess a fault mask.
                    samples[sid] = dict(
                        angle=angle / 10,
                        voltage_mv=voltage,
                        current_ma=current,
                        power_mw=power,
                        temperature_adc=temperature,
                        status=status,
                    )
                if all(sid in samples for sid in ids):
                    return samples
                time.sleep(0.0005)
            missing = sorted(set(ids) - samples.keys())
            raise LeaderGap(f"Leader missing new replies: {missing}", missing_ids=missing)

    def output(self, powers, holds):
        if not self.enabled:
            return
        with self.lock:
            if self.fault:
                raise RuntimeError(self.fault)
            try:
                powers, holds = tuple(powers), tuple(holds)
                if len(powers) != len(self.ids) or len(holds) != len(self.ids):
                    raise ValueError("Leader output requires one power and hold flag per motor")
                for power in powers:
                    if (type(power) not in (int, float) or not math.isfinite(power) or power < 0):
                        raise ValueError("Leader output power must be finite and nonnegative")
                if not self.allow_hold and any(holds):
                    raise ValueError("Leader HOLD is disabled for this output contract")
                if self.max_output_mw < 1500 and any(holds):
                    raise ValueError("Leader HOLD is unavailable below the 1500 mW output contract")
                for sid, power, hold in zip(self.ids, powers, holds):
                    native_allowed = self._native_arm_allowed()
                    if sid in range(6) and not native_allowed:
                        continue
                    if not native_allowed:
                        power, hold = 0, False
                    mode, value = (
                        (0x11, 1500) if hold else (
                            (0x12, min(round(power), math.floor(self.max_output_mw)))
                            if power >= 1 else (0x10, 0)
                        )
                    )
                    if not 0 <= value <= 1500:
                        raise ValueError("Leader output power outside motor contract")
                    command = mode, value
                    if self.last_commands.get(sid) == command:
                        continue
                    if self.before_control_write is not None:
                        self.before_control_write()
                    # A cancellation guard may itself take time. Observe the
                    # board once more immediately before the serial write.
                    native_allowed = self._native_arm_allowed()
                    if sid in range(6) and not native_allowed:
                        continue
                    if not native_allowed:
                        mode, value = 0x10, 0
                        command = mode, value
                    self.managed.add(sid)  # include a motor even if only part of its command is written
                    if self.protect_native_hold and sid in range(6):
                        self._native_arm_write_group = self.command_button.completed_groups
                    self._write(packet(24, struct.pack("<BBH", sid, mode, value)))
                    self.last_commands[sid] = command
            except Exception:
                self.trip("Leader output failed")
                raise

    def trip(self, reason):
        with self.lock:
            self.fault = str(reason)
            self.release()

    def release(self):
        with self.lock:
            failures, pending = [], set()
            if self.protect_native_hold:
                try:
                    self._native_arm_allowed()
                except Exception as exc:
                    failures.append(f"native observation: {exc}")
            for sid in sorted(self.managed):
                try:
                    if sid in range(6) and not self._native_arm_allowed():
                        continue
                    if self.protect_native_hold and sid in range(6):
                        self._native_arm_write_group = self.command_button.completed_groups
                    self._write(packet(24, struct.pack("<BBH", sid, 0x10, 0)))
                except Exception as exc:
                    failures.append(f"{sid}: {exc}")
                    if sid in self.managed:
                        pending.add(sid)
            # A later native group may revoke an earlier failed arm release;
            # never resurrect that axis from this loop's retry accumulator.
            pending.intersection_update(self.managed)
            self.managed = pending  # allow final cleanup to retry an unsuccessful emergency release
            self.last_commands.clear()
            if self.native_output_unknown:
                failures.append("native Leader transition incomplete; residual software output unconfirmed")
            return failures
