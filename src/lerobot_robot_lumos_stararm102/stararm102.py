import math
import time

from lerobot.teleoperators import Teleoperator

from . import _legacy
from .button_protocol import ButtonDetectionCancelled, CommandButtonState, NativeButtonGap
from .common import JOINT_KEYS, features, load_mapping, vector
from .config import StarArm102Config


class NativeButtonRuntimeRequired(RuntimeError):
    code = "native_button_requires_feedback_runtime"
    interface = None

    def __init__(self):
        super().__init__(
            "Native HD buttons require continuous UART command reception; "
            "use the feedback dashboard or feedback follow/monitor runtime"
        )


class StarArm102(Teleoperator):
    config_class = StarArm102Config
    name = "stararm102"

    def __init__(self, config: StarArm102Config):
        self.config = config
        self.uart = None
        self.leader = None
        self.hd_button = None
        self.connection_cancel = None
        self.button_wait_until_cancel = False
        self.button_waiting = None
        self.native_button_rx_active = False
        self.mapping = None
        self._last_sample = None
        self.last_action_diagnostics = None
        self._filter = self._new_filter()
        super().__init__(config)
        if sorted(config.joint_map) != list(range(6)):
            raise ValueError("joint_map must be a permutation of 0..5")
        if len(config.signs) != 6 or any(v not in (-1, 1) for v in config.signs):
            raise ValueError("signs must contain six values, each -1 or 1")
        if not math.isfinite(config.gain) or not 0 < config.gain <= 1:
            raise ValueError("gain must be in (0, 1]")
        if not 7 <= config.button_id <= 253:
            raise ValueError("button_id must be in 7..253")

    def _new_filter(self):
        return _legacy.JointFilter(
            min_cutoff_hz=getattr(self.config, "filter_min_cutoff_hz", _legacy.FILTER_MIN_CUTOFF_HZ),
            beta=getattr(self.config, "filter_beta", _legacy.FILTER_BETA),
        )

    @property
    def action_features(self):
        return features(self.config.gripper)

    @property
    def feedback_features(self):
        return {}

    @property
    def is_connected(self):
        return self.uart is not None and self.uart.is_open and self.leader is not None

    @property
    def is_calibrated(self):
        return self.mapping is not None

    def calibrate(self):
        # This is a paired-arm mapping, not a MotorCalibration file.
        data = load_mapping(self.config.mapping_file, self.config.gripper)
        if data["joint_map"] != self.config.joint_map:
            raise ValueError("joint_map differs from calibration")
        if data["leader_port"] != self.config.port:
            raise ValueError("Leader port differs from calibration; recalibrate or use the original port")
        self.mapping = data

    def connect(self, calibrate=True):
        if self.is_connected:
            raise RuntimeError("StarArm102 already connected")
        self.last_action_diagnostics = None
        self._validate_button_wait()
        self._check_connection_cancelled()
        # Always validate the existing mapping; never run paired-arm calibration implicitly.
        self.calibrate()
        import fashionstar_uart_sdk as uservo
        import serial

        try:
            self.uart = serial.Serial(
                port=self.config.port,
                baudrate=1_000_000,
                parity=serial.PARITY_NONE,
                stopbits=1,
                bytesize=8,
                timeout=0,
            )
            self.leader = uservo.UartServoManager(self.uart)
            for servo_id in list(self.config.joint_map) + ([6] if self.config.gripper else []):
                self._check_connection_cancelled()
                if not _legacy.ping_fresh(self.leader, servo_id):
                    raise RuntimeError(f"StarArm servo {servo_id} did not respond")
            self._check_connection_cancelled()
            self.hd_button = self._detect_button() if self.config.button_enabled else None
            self._last_sample = None
            self._filter = self._new_filter()
            self.configure()
        except Exception:
            self.disconnect()
            raise

    def _check_connection_cancelled(self):
        if self.connection_cancel is not None and self.connection_cancel.is_set():
            raise ButtonDetectionCancelled()

    def _validate_button_wait(self):
        if type(self.button_wait_until_cancel) is not bool:
            raise ValueError("button_wait_until_cancel must be a boolean")
        if self.button_wait_until_cancel and not all(
            callable(getattr(self.connection_cancel, method, None)) for method in ("is_set", "wait")
        ):
            raise ValueError("button_wait_until_cancel requires a connection cancellation event")

    def _detect_button(self):
        self._validate_button_wait()
        self._check_connection_cancelled()
        button_id = self.config.button_id
        if _legacy.ping_fresh(self.leader, button_id):
            self._check_connection_cancelled()
            return _legacy.HDButton(self.leader, button_id, probe=False)
        self._check_connection_cancelled()
        if callable(self.button_waiting):
            self.button_waiting()
        button = CommandButtonState()
        passive_s = 60.0
        deadline = None if self.button_wait_until_cancel else time.monotonic() + passive_s
        while deadline is None or time.monotonic() < deadline:
            self._check_connection_cancelled()
            try:
                # Expire an earlier partial group before reading a new press.
                button.update()
                waiting = self.uart.in_waiting
                if waiting:
                    button.feed(self.uart.read(min(waiting, 4096)))
            except NativeButtonGap:
                # Initial detection may keep listening for a complete new
                # press. An active session treats this exception as fatal.
                continue
            if button.verified:
                self._check_connection_cancelled()
                return button
            if self.connection_cancel is not None:
                if self.connection_cancel.wait(0.02):
                    raise ButtonDetectionCancelled()
            else:
                time.sleep(0.02)
        self._check_connection_cancelled()
        raise _legacy.HDButtonUnavailable(button_id, passive_s=passive_s)

    def configure(self):
        # Serial settings are configured in connect; HD updates happen when sampling.
        pass

    def get_action(self):
        if not self.is_connected:
            raise RuntimeError("StarArm102 is not connected")
        if self.hd_button is not None and self.hd_button.id is None and not self.native_button_rx_active:
            # The ordinary SDK query consumes RX without feeding native button
            # commands. Require the runtime's single-owner receiver first.
            raise NativeButtonRuntimeRequired()
        values = vector(
            _legacy.read_leader(
                self.leader,
                self.config.joint_map,
                self.config.gripper,
                self.hd_button,
            ),
            7 if self.config.gripper else 6,
            "leader feedback",
        )
        now = time.monotonic()
        dt = 0.02 if self._last_sample is None else max(1e-6, now - self._last_sample)
        self._last_sample = now
        filtered = self._filter.update(values[:6], dt)
        mapping = self.mapping
        clipped = [
            max(lo, min(hi, q))
            for q, lo, hi in zip(
                filtered,
                mapping["leader_range_min_deg"],
                mapping["leader_range_max_deg"],
            )
        ]
        targets = [
            home + sign * self.config.gain * math.radians(q - leader_home)
            for q, leader_home, home, sign in zip(
                clipped,
                mapping["leader_home_deg"],
                mapping["lumos_home_rad"],
                self.config.signs,
            )
        ]
        action = dict(zip(JOINT_KEYS, targets))
        if self.config.gripper:
            closed = mapping["gripper_range"]["closed_deg"]
            opened = mapping["gripper_range"]["open_deg"]
            ratio = max(0.0, min(1.0, (values[6] - closed) / (opened - closed)))
            action["gripper.pos"] = 1.0 - ratio if self.config.invert_gripper else ratio
        self.last_action_diagnostics = {
            "raw_leader_deg": values[:6],
            "filtered_leader_deg": filtered.copy(),
            "leader_clipped_deg": clipped.copy(),
            "leader_range_clipped": [q != bounded for q, bounded in zip(filtered, clipped)],
            "mapped_sdk_rad": targets.copy(),
        }
        return action

    def send_feedback(self, feedback):
        # There is no force-feedback channel; the HD button is serviced by get_action.
        pass

    def disconnect(self):
        uart, self.uart = self.uart, None
        self.leader = None
        self.hd_button = None
        self.native_button_rx_active = False
        self.last_action_diagnostics = None
        if uart is not None and uart.is_open:
            uart.close()
