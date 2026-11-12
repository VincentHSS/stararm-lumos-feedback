"""Dedicated LeRobot-adapter loop. No recording or SDK monkeypatching."""

import fcntl
import hashlib
import math
import subprocess
import tempfile
import threading
import time
from collections import deque
from contextlib import contextmanager
from pathlib import Path

from .baseline import BaselineEstimator
from .control import Controller
from .motion import CURRENT_STRATEGIES
from .transport import CanMonitor, Fashionstar, LeaderGap

JOINT_KEYS = tuple(f"joint_{i}.pos" for i in range(1, 7))
ACTION_KEYS = JOINT_KEYS + ("gripper.pos",)
# Engineering target limits for normalized opening, independent of SDK MIT
# gains. Full travel takes at least 2 s; a delayed cycle gains no extra travel.
GRIPPER_MAX_OPENING_PER_S = 0.5
GRIPPER_MAX_STEP_OPENING = 0.02
GRIPPER_MAX_DT_S = 0.04


class SocketCANError(RuntimeError):
    """Readable preflight failure with a stable code for the bilingual UI."""

    def __init__(self, code, interface, detail):
        self.code, self.interface, self.detail = code, interface, detail
        super().__init__(f"SocketCAN {interface}: {detail}")


class SessionCancelled(BaseException):
    """Internal cooperative stop, distinct from a hardware/runtime fault."""


class NativeMotionPaused(Exception):
    """A newly observed native hold pauses targets without stopping the session."""


class NativeButtonForceUnavailable(RuntimeError):
    code = "native_button_force_unreviewed"


class BaselineRecaptureError(RuntimeError):
    """A deliberate unloaded capture failed its current-session admission."""

    def __init__(self, code):
        self.code = code
        super().__init__(code)


def validate_socketcan(interface, bitrate):
    try:
        result = subprocess.run(
            ["ip", "-details", "-json", "link", "show", "dev", interface],
            capture_output=True,
            text=True,
            timeout=2,
            check=True,
        )
    except FileNotFoundError as exc:
        raise SocketCANError("can_ip_unavailable", interface, "The ip command is unavailable") from exc
    except subprocess.TimeoutExpired as exc:
        raise SocketCANError("can_query_timeout", interface, "CAN interface query timed out after 2 s") from exc
    except subprocess.CalledProcessError as exc:
        detail = exc.stderr
        if isinstance(detail, bytes):
            detail = detail.decode("utf-8", errors="replace")
        detail = detail.strip() if isinstance(detail, str) else ""
        detail = detail or f"ip link query failed with exit status {exc.returncode}"
        missing = False
        try:
            # sysfs distinguishes an absent interface from command/permission
            # failures without guessing the language or contents of stderr.
            if interface not in ("", ".", "..") and Path(interface).name == interface:
                network = Path("/sys/class/net")
                missing = network.is_dir() and not (network / interface).exists()
        except OSError:
            pass
        code = "can_interface_missing" if missing else "can_query_failed"
        raise SocketCANError(code, interface, detail) from exc
    except OSError as exc:
        raise SocketCANError("can_query_failed", interface, str(exc)) from exc
    import json

    try:
        links = json.loads(result.stdout)
    except json.JSONDecodeError as exc:
        raise SocketCANError("can_query_failed", interface, "ip returned invalid interface JSON") from exc
    if not isinstance(links, list) or len(links) != 1 or not isinstance(links[0], dict):
        code = "can_interface_missing" if links == [] else "can_query_failed"
        raise SocketCANError(code, interface, "ip did not return one matching interface")
    link = links[0]
    info = link.get("linkinfo", {})
    flags = link.get("flags", [])
    if not isinstance(info, dict) or not isinstance(flags, list) or any(
        not isinstance(flag, str) for flag in flags
    ):
        raise SocketCANError("can_query_failed", interface, "ip returned invalid linkinfo or flags fields")
    data = info.get("info_data", {})
    if not isinstance(data, dict):
        raise SocketCANError("can_query_failed", interface, "ip returned invalid CAN controller fields")
    timing = data.get("bittiming", {})
    if not isinstance(timing, dict):
        raise SocketCANError("can_query_failed", interface, "ip returned invalid CAN bittiming fields")
    if info.get("info_kind") != "can":
        raise SocketCANError("can_query_failed", interface, "Interface is not a physical SocketCAN interface")
    if "UP" not in flags:
        raise SocketCANError("can_interface_down", interface, "CAN interface is DOWN")
    if data.get("state") in ("BUS-OFF", "STOPPED"):
        raise SocketCANError("can_bus_fault", interface, f"CAN controller state is {data['state']}")
    actual_bitrate = timing.get("bitrate")
    if actual_bitrate != bitrate:
        raise SocketCANError(
            "can_bitrate_mismatch", interface, f"CAN bitrate mismatch: expected {bitrate}, received {actual_bitrate}"
        )


@contextmanager
def session_lock(interface, port):
    # Separate locks protect each resource even when the other resource differs.
    streams = []
    try:
        for resource in sorted(("can:" + interface, "serial:" + port)):
            suffix = hashlib.sha256(resource.encode()).hexdigest()[:24]
            stream = (Path(tempfile.gettempdir()) / f"lumos-feedback-{suffix}.lock").open("a")
            streams.append(stream)
            try:
                fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as exc:
                raise RuntimeError(f"Feedback resource already in use: {resource}") from exc
        yield
    finally:
        for stream in reversed(streams):
            stream.close()


class LeaderReader:
    """Use the adapter's existing mapping/filter, but replace cached serial reads."""

    def __init__(self, leader, transport, button_status=None):
        self.leader, self.transport = leader, transport
        self.manual_hold = False
        self.latest = {}
        self.button_status = button_status
        self.original_query = leader.leader.send_sync_servo_monitor
        self.original_stop = leader.leader.stop_on_control_mode
        self.command_button = getattr(leader, "hd_button", None)
        if self.command_button is not None and self.command_button.id is not None:
            self.command_button = None
        if self.command_button is None:
            self.command_button = getattr(transport, "command_button", None)
        self._command_button_state()

    def _command_button_state(self):
        if self.command_button is not None:
            with self.transport.lock:
                self.command_button.update()
                self.manual_hold = (
                    self.command_button.pending or self.command_button.locked is not False
                    or getattr(self.transport, "native_output_unknown", False)
                )
                if self.button_status is not None:
                    self.button_status(self.command_button.verified if not self.command_button.pending else None)

    def install(self):
        self.leader.native_button_rx_active = False

        def query(ids, realtime=False):
            self._command_button_state()
            try:
                samples = self.transport.query(ids)
            except LeaderGap as exc:
                if self.button_status is not None and self.leader.config.button_id in exc.missing_ids:
                    self.button_status(False)
                raise
            finally:
                self._command_button_state()
            self.latest = samples
            if self.button_status is not None and self.leader.config.button_id in samples:
                self.button_status(True)
            for sid, sample in samples.items():
                self.leader.leader.servos[sid].angle_monitor = sample["angle"]

        def stop(sid, mode, power):
            if sid in self.leader.config.joint_map:
                self.manual_hold = mode == 0x11
            # The arbiter is the only software force writer. Native board
            # commands are observed, never retransmitted by this process.

        self.leader.leader.send_sync_servo_monitor = query
        self.leader.leader.stop_on_control_mode = stop
        self.leader.native_button_rx_active = (
            self.command_button is not None
            and getattr(self.transport, "command_button", None) is self.command_button
        )

    def restore(self):
        self.leader.native_button_rx_active = False
        if self.leader.leader is not None:
            self.leader.leader.send_sync_servo_monitor = self.original_query
            self.leader.leader.stop_on_control_mode = self.original_stop


class Session:
    def __init__(
        self, profile, robot, leader, caps, *, mode="monitor", dashboard=None, monitor=None, before_enable=None
    ):
        if mode not in ("follow", "monitor", "experiment", "run"):
            raise ValueError("mode must be follow, monitor, experiment or run")
        if getattr(profile, "engineering", False) and mode == "run":
            raise ValueError("Engineering commissioning cannot enable Leader force output")
        if mode == "experiment" and (
            getattr(profile, "engineering", False) is not True
            or not getattr(profile, "experimental_damping", None)
        ):
            raise ValueError("Experimental damping requires its explicit engineering contract")
        self.profile, self.robot, self.leader, self.caps = profile, robot, leader, caps
        self.mode, self.dashboard = mode, dashboard
        self.monitor = monitor
        self.before_enable = before_enable
        if mode == "experiment":
            from .experimental import ExperimentalDampingController

            self.controller = ExperimentalDampingController(profile, caps)
            self.baseline = self.controller.baseline
        else:
            self.controller = Controller(profile, caps)
            self.baseline = BaselineEstimator(profile)
        self.transport = self.reader = None
        self.stop = threading.Event()
        self.watchdog = None
        self.error = None
        self._startup_rx_error = None
        self.heartbeat = time.monotonic()
        self.target = None
        self.desired = None
        self._gripper_last_target = self._gripper_last_stamp = None
        self._original_control_guard = getattr(robot, "before_control_write", None)
        self._control_guard_installed = False
        self._original_startup_callback = getattr(robot, "after_sdk_startup", None)
        self._startup_callback_installed = False
        self._session_offset_sdk_rad = None
        self._sdk_history_ready = False
        self._loop_times = deque(maxlen=200)
        self._loop_overruns = 0
        self._status_interval_s = 1 / profile.session.get("status_rate_hz", 5)
        self._last_status_at = None
        self._status_cache = {}
        # Seed targets and partially successful arm/gripper writes are not a
        # completed follow command. Initial empty-arm baseline admission uses
        # only a full send_action return recorded by this session.
        self._baseline_completed_send = None
        self._baseline_admission_diagnostics = None
        self._baseline_recapture_lock = threading.Lock()
        self._baseline_recapture_pending = threading.Event()
        self._baseline_recapture_state = dict(state="idle", reason="not_requested")

    def baseline_recapture_status(self):
        with self._baseline_recapture_lock:
            return dict(self._baseline_recapture_state)

    def _baseline_recapture_rejection(self):
        """Check cached session/native state; never read serial in HTTP threads."""
        if self.mode != "experiment" or self.stop.is_set() or self.error is not None:
            return "active_experiment_required"
        if not self._sdk_history_ready or self.transport is None or self.reader is None:
            return "sdk_history_required"
        if getattr(self.transport, "fault", None):
            return "sdk_history_required"
        with self.transport.lock:
            button = getattr(self.transport, "command_button", None)
            if (
                button is None or button.verified is not True or button.locked is not False
                or button.pending or self.reader.manual_hold
                or getattr(self.transport, "native_output_unknown", False)
            ):
                return "native_release_required"
        return None

    def can_recapture_baseline(self):
        with self._baseline_recapture_lock:
            if self._baseline_recapture_state["state"] in ("pending", "collecting"):
                return False
            if self._baseline_recapture_rejection() is not None:
                return False
            with self.caps.lock:
                return all(min(self.caps.common, cap) == 0 for cap in self.caps.axes)

    def request_baseline_recapture(self):
        """Queue one explicit unloaded capture; only step() resets the controller."""
        with self._baseline_recapture_lock:
            state = self._baseline_recapture_state["state"]
            if state in ("pending", "collecting"):
                raise BaselineRecaptureError("recapture_" + state)
            reason = self._baseline_recapture_rejection()
            if reason is not None:
                raise BaselineRecaptureError(reason)
            with self.caps.lock:
                if any(min(self.caps.common, cap) != 0 for cap in self.caps.axes):
                    raise BaselineRecaptureError("zero_caps_required")
                self._baseline_recapture_state = dict(state="pending", reason="recapture_queued")
                self._baseline_recapture_pending.set()
            return dict(self._baseline_recapture_state)

    def _abort_baseline_recapture_locked(self, reason):
        self._baseline_recapture_pending.clear()
        self.controller.abort_unloaded_baseline(reason)
        self.baseline = self.controller.baseline
        self._baseline_admission_diagnostics = None
        self._status_cache = {}
        self._last_status_at = None
        self._baseline_recapture_state = dict(state="rejected", reason=reason)

    def _process_baseline_recapture(self):
        """Revalidate the queued contract on the control thread before update."""
        if self.baseline_recapture_status()["state"] not in ("pending", "collecting"):
            return
        # RX is allowed only here. Do not hold the recapture lock across this
        # call: native status publication may acquire the connection lock.
        if self.transport is not None and self.reader is not None:
            self.transport.observe_native_state()
            self.reader._command_button_state()
        with self._baseline_recapture_lock:
            if self._baseline_recapture_state["state"] not in ("pending", "collecting"):
                return
            reason = self._baseline_recapture_rejection()
            with self.caps.lock:
                if reason is None and any(min(self.caps.common, cap) != 0 for cap in self.caps.axes):
                    reason = "zero_caps_required"
                if reason is not None:
                    self._abort_baseline_recapture_locked("recapture_aborted_" + reason)
                    return
                if self._baseline_recapture_pending.is_set():
                    self.controller.reset_unloaded_baseline()
                    self.baseline = self.controller.baseline
                    self._baseline_admission_diagnostics = None
                    self._status_cache = {}
                    self._last_status_at = None
                    self._baseline_recapture_pending.clear()
                    self._baseline_recapture_state = dict(
                        state="collecting", reason="collecting_declared_unloaded_baseline",
                    )

    def _finish_baseline_recapture(self):
        with self._baseline_recapture_lock:
            if self._baseline_recapture_state["state"] != "collecting":
                return
            reason = self._baseline_recapture_rejection()
            with self.caps.lock:
                if reason is None and any(min(self.caps.common, cap) != 0 for cap in self.caps.axes):
                    reason = "zero_caps_required"
                if reason is not None:
                    self._abort_baseline_recapture_locked("recapture_aborted_" + reason)
                elif self.controller._baseline_collection_closed:
                    self._baseline_recapture_state = dict(
                        state="complete", reason="baseline_recapture_complete",
                    )

    @property
    def automatic_hold_accepted(self):
        return self.mode != "experiment" and not getattr(self.profile, "engineering", False)

    def experimental_state(self):
        if self.mode != "experiment":
            return None
        state = dict(self.controller.snapshot())
        if self.reader is not None and self.reader.command_button is not None and self.reader.manual_hold:
            state.update(state="waiting_native_unlock", reason="native_button_unknown_locked_or_incomplete")
        return state

    def experimental_powers(self, powers):
        """Enforce live ordinary caps even if an upstream observer misbehaves."""
        powers = tuple(powers)
        if len(powers) != 7 or any(
            type(power) not in (int, float) or not math.isfinite(power) or power < 0
            for power in powers
        ):
            raise ValueError("Experimental output requires seven finite nonnegative powers")
        ceiling = self.profile.experimental_damping["max_mw"]
        caps = self.caps.effective()
        if ceiling is not None:
            caps = [min(ceiling, cap) for cap in caps]
        return [min(cap, power) for cap, power in zip(caps, powers)]

    def check_motion_write(self):
        self.check_cancelled()
        if self.mode == "experiment" and self._sdk_history_ready and self.transport is not None:
            allowed = self.transport.observe_native_state()
            self.reader._command_button_state()
            if not allowed or self.reader.manual_hold:
                self._baseline_completed_send = None
                self.controller.inhibit("native_button_unknown_locked_or_incomplete")
                raise NativeMotionPaused
        self.check_cancelled()

    def initial_baseline_admission(self, samples):
        """Qualify only the first local unloaded window; never relearn a load."""
        diagnostics = dict(
            axis_names=[f"J{i}" for i in range(1, 7)] + ["gripper"],
            axis_units=["rad"] * 6 + ["normalized_opening"],
            target=None, sent_target=None, measured_position=None,
            target_error=None, target_tolerance=None,
            arrival_error=None, arrival_tolerance=None,
            target_blocked_axes=None, arrival_blocked_axes=None, blocked_axes=None,
        )
        groups = {"arm": list(range(6)), "gripper": [6]}

        def result(allowed, reason):
            admission = dict(
                allowed=allowed, reason=reason, common_allowed=diagnostics["target_error"] is not None,
                **diagnostics,
                groups={
                    name: dict(allowed=allowed, reason=reason, axis_indices=indices.copy())
                    for name, indices in groups.items()
                },
            )
            if diagnostics["target_error"] is not None:
                for name, indices in groups.items():
                    target_waiting = any(abs(target_error[i]) > target_tolerance[i] for i in indices)
                    arrival_waiting = any(abs(arrival_error[i]) > arrival_tolerance[i] for i in indices)
                    admission["groups"][name].update(
                        allowed=not target_waiting and not arrival_waiting,
                        reason="waiting_follow_target" if target_waiting else
                        "waiting_follower_settled" if arrival_waiting else "baseline_admission_allowed",
                    )
            # Retain the last actual admission check across adapter cleanup.
            # Publishing must not recheck native events or infer gripper
            # targets from the next command sent later in this cycle.
            self._baseline_admission_diagnostics = {
                key: value.copy() if isinstance(value, list) else value
                for key, value in admission.items()
            }
            return admission

        def denied(reason):
            return result(False, reason)

        allowed = self.transport.observe_native_state()
        self.reader._command_button_state()
        if not allowed or self.reader.manual_hold:
            self._baseline_completed_send = None
            return denied("waiting_native_unlock")
        sent = self._baseline_completed_send
        if sent is None:
            return denied("waiting_first_follow_command")
        if len(samples) != 7 or len(sent) != 7 or self.desired is None or len(self.desired) != 7:
            return denied("baseline_admission_missing")
        limits = getattr(self.robot, "limits", None)
        if limits is None:
            limits = self.profile.limits
        if (
            not isinstance(limits, (list, tuple)) or len(limits) != 2
            or any(not isinstance(row, (list, tuple)) or len(row) != 6 for row in limits)
            or any(type(value) not in (int, float) or not math.isfinite(value) for row in limits for value in row)
            or any(lo >= hi for lo, hi in zip(*limits))
            or any(type(value) not in (int, float) or not math.isfinite(value) for value in (*sent, *self.desired))
            or any(not math.isfinite(sample.q) for sample in samples)
        ):
            return denied("baseline_admission_missing")
        final_target = [max(lo, min(hi, q)) for q, lo, hi in zip(self.desired[:6], *limits)]
        final_target.append(max(0.0, min(1.0, self.desired[6])))
        # The bound adapter's max step is an arrival tolerance here, not a
        # change to its physical motion limit or its target-rate limiter.
        max_step = getattr(
            getattr(self.robot, "config", None), "max_target_step_deg",
            self.profile.session["robot"].get("max_target_step_deg", 2.0),
        )
        if type(max_step) not in (int, float) or not math.isfinite(max_step) or max_step <= 0:
            return denied("baseline_admission_missing")
        arm_target_tolerance = [
            max(8 * 2 * axis["p_max"] / 65535 * abs(axis["sdk_scale"]), axis["sdk_tolerance"])
            for axis in self.profile.axes[:6]
        ]
        grip_axis = self.profile.axes[6]
        grip_jacobian = self.baseline.quantization[6]["position"] / (2 * grip_axis["p_max"] / 65535)
        grip_tolerance = max(
            GRIPPER_MAX_STEP_OPENING,
            grip_jacobian * self.profile.gripper_transform["sdk_motor_tolerance_rad"],
        )
        target_tolerance = arm_target_tolerance + [grip_tolerance]
        arrival_tolerance = [math.radians(max_step)] * 6 + [grip_tolerance]
        # These signed errors describe the same pre-send samples and complete
        # command already used by the gate. Arm coordinates are SDK radians;
        # the gripper is normalized physical opening, never motor radians.
        target_error = [q - executed for q, executed in zip(final_target, sent)]
        arrival_error = [sample.q - executed for sample, executed in zip(samples, sent)]
        target_blocked = [
            name for name, error, margin in zip(diagnostics["axis_names"], target_error, target_tolerance)
            if abs(error) > margin
        ]
        arrival_blocked = [
            name for name, error, margin in zip(diagnostics["axis_names"], arrival_error, arrival_tolerance)
            if abs(error) > margin
        ]
        diagnostics.update(
            target=final_target, sent_target=list(sent), measured_position=[sample.q for sample in samples],
            target_error=target_error, target_tolerance=target_tolerance,
            arrival_error=arrival_error, arrival_tolerance=arrival_tolerance,
            target_blocked_axes=target_blocked, arrival_blocked_axes=arrival_blocked,
            blocked_axes=[name for name in diagnostics["axis_names"] if name in target_blocked or name in arrival_blocked],
        )
        if any(abs(q - executed) > margin for q, executed, margin in zip(final_target, sent, target_tolerance)):
            return denied("waiting_follow_target")
        if any(abs(sample.q - executed) > margin for sample, executed, margin in zip(samples, sent, arrival_tolerance)):
            return denied("waiting_follower_settled")
        return result(True, "baseline_admission_allowed")

    def limited_gripper_action(self, action, samples):
        now = time.monotonic()
        if self._gripper_last_target is None:
            self._gripper_last_target = samples[6].q
            target = samples[6].q  # First command holds the actual measured opening.
        else:
            elapsed = now - self._gripper_last_stamp
            if not math.isfinite(elapsed):
                raise RuntimeError("Invalid gripper target timing")
            dt = max(0.0, min(GRIPPER_MAX_DT_S, elapsed))
            step = min(GRIPPER_MAX_STEP_OPENING, GRIPPER_MAX_OPENING_PER_S * dt)
            desired = max(0.0, min(1.0, action["gripper.pos"]))
            target = self._gripper_last_target + max(-step, min(step, desired - self._gripper_last_target))
        self._gripper_last_stamp = now
        return dict(action, **{"gripper.pos": target})

    def publish(self, status, samples=None):
        if self.dashboard is None:
            return
        # A late-returning device call must not overwrite a watchdog fault with
        # another active update while cleanup is pending.
        if self.error is not None and not status.startswith("leader_release_failed"):
            status = "stopped"
        telemetry = dict(self.controller.telemetry)
        power, holds = self.controller.outputs(self.reader.manual_hold if self.reader else False)
        if self.mode == "experiment":
            actual = self.experimental_powers(power)
            if self.reader is not None and self.reader.manual_hold:
                actual = [0] * 7
        else:
            actual = [1500 if hold else p for hold, p in zip(holds, power)] if self.mode == "run" else [0] * 7
        now = time.monotonic()
        if self._last_status_at is None or now - self._last_status_at >= self._status_interval_s or status != "active":
            diagnostics = {}
            for adapter in (self.leader, self.robot):
                diagnostics.update(getattr(adapter, "last_action_diagnostics", None) or {})
            if samples is not None:
                diagnostics["actual_sdk_rad"] = [s.q for s in samples[:6]]
            self._status_cache = dict(
                baseline=self.baseline.snapshot(),
                follow_diagnostics=diagnostics,
                loop_timing=dict(
                    target_hz=self.profile.session.get("fps", 30),
                    status_hz=self.profile.session.get("status_rate_hz", 5),
                    actual_hz=(len(self._loop_times) - 1) / (self._loop_times[-1] - self._loop_times[0])
                    if len(self._loop_times) > 1 and self._loop_times[-1] > self._loop_times[0] else None,
                    overruns=self._loop_overruns,
                ),
            )
            self._last_status_at = now
        telemetry.update(
            requested_mw=actual,
            predicted_mw=power,
            hold_requested=holds if self.mode == "run" else [False] * 7,
            automatic_hold=self.controller.hold and self.automatic_hold_accepted,
            automatic_hold_candidate=self.controller.hold,
            automatic_hold_accepted=self.automatic_hold_accepted,
            error=None if self.error is None else str(self.error),
            **self._status_cache,
            gripper_target_limits=dict(
                max_opening_per_s=GRIPPER_MAX_OPENING_PER_S,
                max_step_opening=GRIPPER_MAX_STEP_OPENING,
                max_elapsed_credit_s=GRIPPER_MAX_DT_S,
                source="runtime engineering limits; hardware effects not validated",
            ),
        )
        if self.mode == "experiment":
            telemetry["experimental"] = self.experimental_state()
            telemetry["baseline_admission"] = self._baseline_admission_diagnostics
            telemetry["baseline_recapture"] = self.baseline_recapture_status()
            telemetry["can_recapture_baseline"] = self.can_recapture_baseline()
            native_waiting = telemetry["experimental"]["state"] == "waiting_native_unlock"
            unknown = bool(getattr(self.transport, "native_output_unknown", False))
            telemetry["leader_controlled"] = not native_waiting and not unknown
            telemetry["leader_output_state"] = (
                "unconfirmed_native_transition" if unknown else
                "native_board" if native_waiting else "experimental_requests"
            )
        if status.startswith("stopped") or status.startswith("leader_release_failed"):
            telemetry.update(requested_mw=[0] * 7, hold_requested=[False] * 7)
            if self.mode == "experiment":
                telemetry["leader_controlled"] = False
        telemetry["leader_current_ma"] = (
            [self.reader.latest.get(sid, {}).get("current_ma") for sid in self.transport.ids]
            if self.reader
            else [None] * 7
        )
        if samples is not None:
            telemetry["age_ms"] = [(time.monotonic() - s.stamp) * 1000 for s in samples]
            telemetry["q_sdk"] = [s.q for s in samples]
            telemetry["measured_nm"] = [s.torque for s in samples]
            telemetry["temperature_c"] = [s.temperature for s in samples]
            telemetry["current_ma"] = [None] * 7
            # Geometry remains live; expensive baseline/diagnostic summaries
            # above run at status_rate_hz, independent of safety/control.
            telemetry["pose"] = self.controller.pose(
                [self.profile.to_model(i, s.q) for i, s in enumerate(samples[:6])]
            )
        self.dashboard.publish(status=status, mode=self.mode, **telemetry)

    def check(self):
        self.check_cancelled()
        samples = self.monitor.snapshot()
        self.check_cancelled()
        return samples

    def check_cancelled(self):
        if self._startup_rx_error is not None:
            raise self._startup_rx_error
        # A concurrent watchdog fault must remain an error even when the user
        # also requests stop. Cancellation only suppresses an explicit stop.
        if self.error is not None:
            raise RuntimeError(f"Feedback stopped: {self.error}") from self.error
        if self.stop.is_set():
            raise SessionCancelled

    @contextmanager
    def native_startup_reception(self):
        """Keep the single UART receiver alive while SDK startup blocks.

        No monitor query or control is sent here. The existing transport lock
        serializes this receiver with preflight queries, and the thread is
        joined before the active loop takes over. The native group deadline
        and six-joint confirmation rules remain unchanged.
        """
        transport = self.transport
        if transport is None or transport.command_button is None:
            yield
            return
        transport.settle_native_commands(cancel_check=self.check_cancelled)
        finished = threading.Event()

        def receive():
            try:
                while not finished.is_set() and not self.stop.is_set():
                    transport.receive_native_commands()
                    finished.wait(0.002)
            except Exception as exc:
                self._startup_rx_error = exc
                self.stop.set()

        receiver = threading.Thread(target=receive, name="lumos-startup-uart-rx", daemon=True)
        receiver.start()
        failed = False
        try:
            self.check_cancelled()
            yield
        except BaseException:
            failed = True
            if self._startup_rx_error is not None:
                raise self._startup_rx_error
            raise
        finally:
            finished.set()
            # UART reads are nonblocking and transport polling is bounded.
            # Complete the handoff before any active query or UART cleanup;
            # a timed join must never leave a second reader behind.
            receiver.join()
            if not failed:
                self.check_cancelled()
                transport.settle_native_commands(cancel_check=self.check_cancelled)

    def check_sdk(self, samples):
        self.check_cancelled()
        positions = list(self.robot.arm.get_joint_positions())
        arm_finished = time.monotonic()
        self.check_cancelled()
        if len(positions) != 6 or not all(math.isfinite(x) for x in positions):
            raise RuntimeError("SDK returned invalid positions")
        history = None
        history_allowed = (
            getattr(self.profile, "engineering", False) is True
            and self.mode in ("follow", "experiment") and self._sdk_history_ready
        )
        for i, (q, sample, axis) in enumerate(zip(positions, samples, self.profile.axes)):
            # Allow frame-to-getter timing motion, but never silently infer signs/zeros.
            timing = abs(sample.velocity) * (arm_finished - sample.stamp) / abs(axis["urdf_scale"])
            if abs(q - sample.q) > axis["sdk_tolerance"] + timing:
                if history_allowed and history is None:
                    # A cache comparison must not turn old feedback into fresh
                    # telemetry. Stop on a current CAN fault/staleness first.
                    self.check()
                    recent = getattr(self.monitor, "recent_arm_history", None)
                    history = recent(ended_at=arm_finished) if recent is not None else ((),) * 6
                    self.check_cancelled()
                candidates = history[i] if history is not None else ()
                closest = min(candidates, key=lambda item: abs(q - item.q), default=None)
                if closest is not None and abs(q - closest.q) <= axis["sdk_tolerance"]:
                    continue
                closest_detail = "none" if closest is None else (
                    f"q={closest.q:+.12g} rad, delta={q - closest.q:+.12g} rad, "
                    f"age={arm_finished - closest.stamp:.12g} s, sequence={closest.sequence}"
                )
                raise RuntimeError(
                    f"Axis {i + 1}: passive CAN and SDK coordinates disagree: "
                    f"SDK={q:+.12g} rad, CAN={sample.q:+.12g} rad, "
                    f"delta(SDK-CAN)={q - sample.q:+.12g} rad, "
                    f"sdk_tolerance={axis['sdk_tolerance']:.12g} rad, "
                    f"timing_allowance={timing:.12g} rad, "
                    f"velocity={sample.velocity:+.12g} rad/s, "
                    f"frame_age={arm_finished - sample.stamp:.12g} s, "
                    f"history_window={axis['max_age_s']:.12g} s, "
                    f"history_count={len(candidates)}, closest_history=({closest_detail})"
                )
        started = time.monotonic()
        state = self.robot.arm.get_gripper_state()
        self.check_cancelled()
        finished = time.monotonic()
        sample, axis = samples[6], self.profile.axes[6]
        cfg = self.profile.gripper_transform
        age = float(state.feedback_age_ms) / 1000
        if (
            not state.feedback_valid
            or not state.enabled
            or state.error_code != 1  # DM status nibble 1 is enabled, not a fault code.
            or not math.isfinite(age)
            or not 0 <= age <= axis["max_age_s"]
        ):
            raise RuntimeError("Gripper SDK state is invalid, disabled, faulted or stale")
        motor, velocity = float(state.motor_position_rad), float(state.motor_velocity_rad_s)
        self.profile.check_gripper_motor(motor, velocity)
        if sample.motor_position is None or sample.motor_velocity is None:
            raise RuntimeError("Gripper raw CAN motor coordinates are missing")
        # These SDK ages describe a cache snapshot, never a new CAN sample. Only
        # CanMonitor's kernel timestamp/sequence advance feedback confirmation.
        skew = max(abs(started - age - sample.stamp), abs(finished - age - sample.stamp))
        tolerance = cfg["sdk_motor_tolerance_rad"] + cfg["max_motor_velocity_rad_s"] * skew
        if abs(motor - sample.motor_position) > tolerance:
            raise RuntimeError("Gripper passive CAN and SDK raw motor coordinates disagree")
        distance = float(state.distance_m)
        if not math.isfinite(distance):
            raise RuntimeError("Gripper SDK returned invalid opening distance")
        # This is the public get_gripper_position() calculation, using the same
        # state snapshot as raw motor feedback. GripperState.position is a
        # different, linear motor-angle fraction and must never be used here.
        opening = min(1.0, max(0.0, distance / self.profile.gripper.width_m))
        if abs(opening - sample.q) > axis["sdk_tolerance"] + axis["max_velocity"] * skew:
            raise RuntimeError("Gripper passive CAN and SDK public opening coordinates disagree")

    def current_startup_feedback(self, reference, *, deadline=None):
        """Wait for a complete post-call CAN group, retaining the preflight anchor."""
        self.check_cancelled()
        if (
            getattr(self.profile, "engineering", False) is not True
            or self.mode not in ("follow", "monitor", "experiment")
            or reference is not getattr(self.robot, "startup_reference", None)
            or not isinstance(reference, dict)
            or reference.get("startup_strategy") not in CURRENT_STRATEGIES
            or reference.get("startup_strategy") != getattr(self.robot, "startup_strategy", "current_aligned")
        ):
            raise RuntimeError("Current-pose startup requires its matching validated engineering preflight")
        started = time.monotonic()
        try:
            anchor = tuple(reference["actual_q"])
            stamps = tuple(reference["frame_stamps"])
            tolerances = tuple(reference["tolerance_rad"])
            captured = reference["captured_monotonic"]
            if (
                len(anchor) != 7 or len(stamps) != 7 or len(tolerances) != 6
                or not all(type(v) in (int, float) and math.isfinite(v) for v in (*anchor, *stamps, *tolerances, captured))
                or not max(stamps) <= captured <= started
                or any(not 0 < value <= math.radians(2) for value in tolerances)
            ):
                raise ValueError("invalid reference values")
            if reference["startup_strategy"] == "current_relative":
                offset = tuple(reference["session_offset_sdk_rad"])
                if self._session_offset_sdk_rad is None or offset != self._session_offset_sdk_rad:
                    raise ValueError("relative startup offset changed or was not captured")
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError("Invalid current-pose preflight reference") from exc
        if deadline is not None and (type(deadline) not in (int, float) or not math.isfinite(deadline)):
            raise RuntimeError("Current startup requires a finite SDK readiness deadline")
        deadline = min(started + self.profile.guards["startup_feedback_s"], deadline) if deadline is not None else (
            started + self.profile.guards["startup_feedback_s"]
        )
        leader_target = reference["startup_strategy"] == "leader_target"
        allow_arm_settling = (
            getattr(self.profile, "engineering", False) is True
            and self.mode in ("follow", "experiment") and leader_target
        )
        while True:
            self.check_cancelled()
            if time.monotonic() >= deadline:
                raise RuntimeError("Current startup: complete new enabled CAN feedback not received before deadline")
            try:
                samples = self.monitor.snapshot()
            except RuntimeError:
                if self.monitor.error is not None or time.monotonic() >= deadline:
                    raise
            else:
                self.check_cancelled()
                if all(sample.stamp > max(stamp, started) for sample, stamp in zip(samples, stamps)):
                    for i, (sample, home, tolerance) in enumerate(zip(samples[:6], anchor, tolerances)):
                        axis = self.profile.axes[i]
                        quiet_speed = 4 * 2 * axis["v_max"] / 4095 * abs(axis["sdk_scale"] * axis["urdf_scale"])
                        # Direct basic follow may settle when the SDK enables
                        # and seeds from measured feedback. Physical CAN guards
                        # and the original preflight anchor still bound it.
                        if not allow_arm_settling and abs(sample.velocity) > quiet_speed:
                            raise RuntimeError(
                                f"Axis {i + 1}: current startup pose is moving: "
                                f"velocity={sample.velocity:+.12g} rad/s, "
                                f"quiet_limit={quiet_speed:.12g} rad/s"
                            )
                        lo, hi = self.profile.limits[0][i], self.profile.limits[1][i]
                        margin = axis["limit_margin"]
                        if not lo - margin <= sample.q <= hi + margin:
                            raise RuntimeError(
                                f"Axis {i + 1}: current startup pose is outside joint limits: "
                                f"actual={sample.q:+.12g} rad, limits=[{lo:.12g}, {hi:.12g}] rad, "
                                f"measurement_margin={margin:.12g} rad"
                            )
                        drift_tolerance = min(math.radians(2), tolerance)
                        if abs(sample.q - home) > drift_tolerance:
                            raise RuntimeError(
                                f"Axis {i + 1}: current startup pose drifted from preflight: "
                                f"actual={sample.q:+.12g} rad, reference={home:+.12g} rad, "
                                f"delta={sample.q - home:+.12g} rad, "
                                f"tolerance={drift_tolerance:.12g} rad"
                            )
                    if not 0 <= samples[6].q <= (1 if leader_target else 0.01):
                        raise RuntimeError("Current startup TypeLJ opening is outside its allowed range")
                    grip = samples[6]
                    if (
                        grip.motor_velocity is None
                        or (
                            not leader_target
                            and abs(grip.motor_velocity) > 4 * 2 * self.profile.axes[6]["v_max"] / 4095
                        )
                    ):
                        raise RuntimeError("Current startup gripper motor is moving or unobserved")
                    # All accepted samples are status-1 CAN frames. Once this
                    # complete group is seen, disabled feedback becomes fatal.
                    self.check_cancelled()
                    if time.monotonic() >= deadline:
                        raise RuntimeError("Current startup: enabled CAN feedback arrived after deadline")
                    self.monitor.armed = True
                    return [sample.q for sample in samples]
            if time.monotonic() >= deadline:
                raise RuntimeError("Current startup: complete new enabled CAN feedback not received")
            self.stop.wait(min(0.005, max(0.0, deadline - time.monotonic())))

    def start_watchdog(self):
        def watch():
            while not self.stop.wait(0.005):
                try:
                    self.monitor.snapshot()
                    if time.monotonic() - self.heartbeat > self.profile.guards["cycle_timeout_s"]:
                        raise RuntimeError("Control loop watchdog expired")
                except Exception as exc:
                    self.error = exc
                    try:
                        self.transport.trip(exc)
                    finally:
                        self.publish("stopped")
                    return

        self.watchdog = threading.Thread(target=watch, name="lumos-feedback-watchdog", daemon=True)
        self.watchdog.start()

    def step(self):
        self.check()
        action = self.leader.get_action()  # fresh query or LeaderGap, never cached movement
        self.check_cancelled()
        if set(action) != set(ACTION_KEYS) or not all(math.isfinite(action[k]) for k in ACTION_KEYS):
            raise RuntimeError("Invalid Leader action")
        if self._session_offset_sdk_rad is not None:
            # Preserve the original mapped Leader action and every downstream
            # joint-limit/rate/force check. Only this session's six-axis origin
            # changes; TypeLJ opening has no relative offset.
            action = dict(action)
            for key, offset in zip(JOINT_KEYS, self._session_offset_sdk_rad):
                action[key] += offset
            if not all(math.isfinite(action[key]) for key in JOINT_KEYS):
                raise RuntimeError("Invalid relative Leader action")
        self.desired = [action[k] for k in ACTION_KEYS]
        samples = self.check()
        self.check_sdk(samples)
        # Recheck after any SDK/serial call, before force/motion writes.
        samples = self.check()
        if self.target is None:
            self.target = [s.q for s in samples]
        if self.mode == "experiment":
            self._process_baseline_recapture()
            self.controller.update(
                samples, self.target, self.desired,
                baseline_admission=self.initial_baseline_admission(samples),
            )
            self._finish_baseline_recapture()
        elif self.mode != "follow":
            if self.controller.update(samples, self.target, self.desired):
                self.baseline.update(samples, self.controller.telemetry["residual_nm"])
            else:
                self.baseline.gap()
        frozen = (self.controller.hold and self.automatic_hold_accepted) or self.reader.manual_hold
        if not frozen:
            self.check()
            limited = self.limited_gripper_action(action, samples)
            self.check_cancelled()
            try:
                self.check_motion_write()
                sent = self.robot.send_action(limited)
            except NativeMotionPaused:
                self._baseline_completed_send = None
                # The arm write can succeed before a native event inhibits
                # the separate gripper write. Keep the real limiter origin.
                sent_arm = getattr(self.robot, "_previous_target", None)
                if sent_arm is not None:
                    self.target[:6] = list(sent_arm)
            else:
                self.check_cancelled()
                self.target = [sent[k] for k in ACTION_KEYS]  # actual rate/step/limit-clamped target
                if not all(math.isfinite(x) for x in self.target):
                    raise RuntimeError("Invalid executed target")
                self._gripper_last_target = self.target[6]
                if self.mode == "experiment":
                    self._baseline_completed_send = list(self.target)
        self.check()
        power, holds = self.controller.outputs(self.reader.manual_hold)
        if self.mode == "experiment":
            if self.reader.manual_hold:
                self._baseline_completed_send = None
                self.controller.inhibit("native_button_unknown_locked_or_incomplete")
                power = [0] * 7
            power = self.experimental_powers(power)
            holds = [False] * 7
        self.check_cancelled()
        self.transport.output(power, holds)
        self.check_cancelled()
        if self.mode == "experiment":
            # The output boundary drains native events once more immediately
            # before writing. Reflect any newly observed lock in publication.
            self.reader._command_button_state()
            if self.reader.manual_hold:
                self._baseline_completed_send = None
                self.controller.inhibit("native_button_unknown_locked_or_incomplete")
        self.heartbeat = time.monotonic()
        self.publish("active", samples)

    def run(self, duration=None):
        if duration is not None and (not math.isfinite(duration) or duration <= 0):
            raise ValueError("duration must be finite and positive")
        guards = self.profile.guards
        try:
            self.check_cancelled()
            self.robot.before_control_write = self.check_motion_write
            self._control_guard_installed = True
            if self.monitor is None:
                validate_socketcan(
                    self.profile.session["robot"].get("can_interface", "can0"), guards["can_bitrate"]
                )
            self.check_cancelled()
            self.leader.connection_cancel = self.stop
            self.leader.button_waiting = lambda: self.publish("button_board_waiting")
            try:
                self.leader.connect()
            except RuntimeError as exc:
                if getattr(exc, "code", None) == "connection_cancelled" and self.stop.is_set():
                    raise SessionCancelled from None
                raise
            button_status = getattr(self.dashboard, "button_status", None)
            if button_status is not None and self.leader.config.button_enabled:
                # connect() requires a matching legacy ping or complete native
                # six-joint button command group received from the UART.
                button_status(True)
            button = getattr(self.leader, "hd_button", None)
            command_button = button if button is not None and button.id is None else None
            if command_button is not None and self.mode == "run":
                raise NativeButtonForceUnavailable(
                    "Native HD button board writes Leader controls independently; "
                    "force-output arbitration for this board has not been validated. Use follow/monitor."
                )
            if self.mode == "experiment":
                # A native board can write the bus even when the software
                # board selector is off. This bound contract requires a real
                # complete release event before any experimental arm output.
                required = self.profile.experimental_damping["native_button_required"]
                if required and command_button is None:
                    from lerobot_robot_lumos_stararm102.button_protocol import CommandButtonState

                    command_button = CommandButtonState()
            self.check_cancelled()
            output_options = {}
            if self.mode == "experiment":
                output_options = dict(
                    max_output_mw=(
                        1500 if self.profile.experimental_damping["max_mw"] is None
                        else self.profile.experimental_damping["max_mw"]
                    ),
                    protect_native_hold=command_button is not None,
                    allow_hold=False,
                )
            self.transport = Fashionstar(
                self.leader.uart,
                list(self.leader.config.joint_map) + [6],
                enabled=self.mode in ("run", "experiment"),
                timeout=guards["leader_query_s"],
                before_control_write=self.check_cancelled,
                command_button=command_button,
                **output_options,
            )
            self.reader = LeaderReader(self.leader, self.transport, button_status=button_status)
            self.reader.install()
            self.check_cancelled()
            if self.monitor is None:
                self.monitor = CanMonitor(self.profile, mode="follow" if self.mode == "experiment" else self.mode)
            self.check_cancelled()
            self.monitor.start()
            self.check_cancelled()
            if getattr(self.robot, "startup_strategy", "home") in CURRENT_STRATEGIES:
                if getattr(self.profile, "engineering", False) is not True or self.mode not in ("follow", "monitor", "experiment"):
                    raise RuntimeError("Current-pose startup requires engineering follow or monitor mode")
                self.robot.after_sdk_startup = self.current_startup_feedback
                self._startup_callback_installed = True
            with self.native_startup_reception():
                if self.before_enable is not None:
                    self.before_enable(self.transport)
                    self.check_cancelled()
                if getattr(self.robot, "startup_strategy", "home") == "current_relative":
                    try:
                        reference = self.robot.startup_reference
                        offset = tuple(reference["session_offset_sdk_rad"])
                        if (
                            reference["startup_strategy"] != "current_relative" or len(offset) != 6
                            or not all(type(v) in (int, float) and math.isfinite(v) for v in offset)
                        ):
                            raise ValueError("invalid session offset")
                    except (AttributeError, KeyError, TypeError, ValueError) as exc:
                        raise RuntimeError("Current-relative startup requires its fixed preflight offset") from exc
                    self._session_offset_sdk_rad = offset
                if command_button is not None:
                    self.transport.settle_native_commands(cancel_check=self.check_cancelled)
                self.robot.connect()  # SDK construction can enable motors; monitor already listening
            self.check_cancelled()
            deadline = time.monotonic() + guards["startup_feedback_s"]
            while True:
                self.check_cancelled()
                try:
                    samples = self.monitor.snapshot()
                    self.check_cancelled()
                    break
                except RuntimeError:
                    if self.monitor.error is not None or time.monotonic() >= deadline:
                        raise
                    self.stop.wait(0.005)
            self.monitor.armed = True
            if self._startup_callback_installed:
                from .startup import check_startup_pose

                positions = list(self.robot.arm.get_joint_positions())
                self.check_cancelled()
                if len(positions) != 6 or not all(math.isfinite(q) for q in positions):
                    raise RuntimeError("Current startup SDK returned invalid positions")
                # Initialization can take time. Requery the Leader and require
                # another coherent CAN group before allowing basic following.
                leader = self.transport.query(tuple(range(7)))
                self.check_cancelled()
                query_finished = time.monotonic()
                current = self.current_startup_feedback(self.robot.startup_reference)
                if time.monotonic() - query_finished > min(
                    min(axis["max_age_s"] for axis in self.profile.axes),
                    guards["leader_query_s"] + guards["can_skew_s"],
                ):
                    raise RuntimeError("Current startup Leader/CAN alignment feedback is too far apart in time")
                check_startup_pose(
                    self.profile, current,
                    {sid: sample["angle"] for sid, sample in leader.items()},
                    strategy=self.robot.startup_strategy,
                    **({"session_offset": self._session_offset_sdk_rad} if self._session_offset_sdk_rad is not None else {}),
                )
                seed = self.robot._previous_target
                if seed is None or any(
                    abs(q - initial) > min(math.radians(2), tolerance)
                    for q, initial, tolerance in zip(current, seed, self.robot.startup_reference["tolerance_rad"])
                ):
                    raise RuntimeError("Current startup pose drifted from the fixed SDK seed")
                for i, (q, actual, axis) in enumerate(zip(positions, current, self.profile.axes), 1):
                    if not math.isfinite(q) or abs(q - actual) > axis["sdk_tolerance"]:
                        raise RuntimeError(
                            "Current startup SDK and passive CAN positions disagree: "
                            f"stage=after_leader_recheck, J{i}: "
                            f"SDK={q:.12g} rad, CAN={actual:.12g} rad, "
                            f"delta(SDK-CAN)={q - actual:+.12g} rad, tolerance={axis['sdk_tolerance']:.12g} rad, "
                            f"sdk_all_zero={all(value == 0.0 for value in positions)}"
                        )
                self.check_cancelled()
                samples = self.monitor.snapshot()
            self.check_sdk(samples)
            # SDK construction and its real measured seed have already passed
            # strict admission. Only active engineering follow may compare the
            # asynchronous SDK cache with bounded real CAN history thereafter.
            self._sdk_history_ready = True
            self.heartbeat = started = last_leader = time.monotonic()
            self.start_watchdog()
            fps = self.profile.session.get("fps", 30)
            if type(fps) is not int or not 1 <= fps <= 100:
                raise ValueError("fps must be in 1..100")
            while not self.stop.is_set() and (duration is None or time.monotonic() - started < duration):
                cycle = time.monotonic()
                self._loop_times.append(cycle)
                try:
                    self.step()
                    last_leader = time.monotonic()
                except LeaderGap:
                    self._baseline_completed_send = None
                    self.controller.gap()
                    self.baseline.gap()
                    if self.mode == "experiment":
                        self.transport.output([0] * 7, [False] * 7)
                    samples = self.check()
                    self.publish("leader_gap", samples)
                    if time.monotonic() - last_leader > guards["leader_gap_s"]:
                        raise RuntimeError("Continuous Leader missing-frame timeout")
                    self.heartbeat = time.monotonic()
                elapsed = time.monotonic() - cycle
                if elapsed > 1 / fps:
                    self._loop_overruns += 1
                self.stop.wait(max(0, 1 / fps - elapsed))
                self.check()
            self.check_cancelled()
        except SessionCancelled:
            pass
        except BaseException as exc:
            if not isinstance(exc, KeyboardInterrupt) and self.error is None:
                self.error = exc
            raise
        finally:
            self._sdk_history_ready = False
            self.stop.set()
            if self.mode == "experiment":
                with self._baseline_recapture_lock:
                    if self._baseline_recapture_state["state"] in ("pending", "collecting"):
                        self._abort_baseline_recapture_locked("recapture_aborted_session_stopped")
            if self.watchdog is not None:
                self.watchdog.join(timeout=0.2)
            failures = self.transport.release() if self.transport else []
            if self.reader is not None:
                self.reader.restore()
            if self._control_guard_installed:
                self.robot.before_control_write = self._original_control_guard
                self._control_guard_installed = False
            if self._startup_callback_installed:
                self.robot.after_sdk_startup = self._original_startup_callback
                self.robot.startup_reference = None
                self._startup_callback_installed = False
            self._session_offset_sdk_rad = None
            # Release Leader first, before the SDK cleanup can block.
            try:
                self.publish("stopped" if not failures else "leader_release_failed: " + "; ".join(failures))
            finally:
                # Telemetry must not retain active state during native cleanup,
                # and publication failures must not prevent device cleanup.
                try:
                    self.robot.disconnect()
                finally:
                    try:
                        self.leader.disconnect()
                    finally:
                        if self.monitor is not None:
                            self.monitor.close()
                        self.publish(
                            "stopped" if not failures else "leader_release_failed: " + "; ".join(failures)
                        )
            if failures:
                raise RuntimeError("Leader release could not be confirmed: " + "; ".join(failures))
