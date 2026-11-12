import logging
import math
import time
from pathlib import Path

from lerobot.cameras import make_cameras_from_configs
from lerobot.robots import Robot

from stararm_lumos_feedback.motion import CURRENT_STRATEGIES

from . import _legacy
from .common import JOINT_KEYS, features, limit_target, load_limits, load_mapping, vector
from .config import LumosConfig
from .gripper_control import TypeLJFollowControl, validate_follow_gains

logger = logging.getLogger(__name__)


class SDKCleanupError(RuntimeError):
    code = "sdk_cleanup_failed"


class Lumos(Robot):
    config_class = LumosConfig
    name = "lumos"

    def __init__(self, config: LumosConfig):
        self.config = config
        self.arm = None
        self.cameras = {}
        self.mapping = None
        self._previous_target = None
        self._last_send = None
        self.last_sent_action = None
        self.last_action_diagnostics = None
        self._gripper_follow_control = None
        # Optional session-owned cancellation guard. Basic LeRobot operation
        # leaves this unset and retains its existing behavior.
        self.before_control_write = None
        self.startup_strategy = "home"
        self.startup_reference = None
        self.after_sdk_startup = None
        self.measured_limit_margin_rad = (0.0,) * 6
        super().__init__(config)
        validate_follow_gains(config.gripper_follow_kp, config.gripper_follow_kd)
        if config.gripper_follow_kp is not None and not config.gripper:
            raise ValueError("gripper_follow gains require gripper=true")
        for value in (
            config.max_speed_deg_per_sec,
            config.max_target_step_deg,
            config.startup_tolerance_rad,
            config.startup_joint2_tolerance_rad,
            config.startup_wait_s,
        ):
            if not math.isfinite(value) or value <= 0:
                raise ValueError("Motion and startup parameters must be finite and positive")
        pair_tolerance = config.startup_pair_tolerance_rad
        if pair_tolerance is not None and (
            type(pair_tolerance) not in (int, float)
            or not math.isfinite(pair_tolerance)
            or pair_tolerance <= 0
        ):
            raise ValueError("startup_pair_tolerance_rad must be None or finite and positive")
        self.limits = load_limits(config.joint_limits_file)
        if config.execute and self.limits is None:
            raise ValueError("Execution requires joint_limits_file with verified SDK-coordinate bounds")
        self.cameras = make_cameras_from_configs(config.cameras)

    @property
    def action_features(self):
        return features(self.config.gripper)

    @property
    def observation_features(self):
        return {
            **self.action_features,
            **{key: (cfg.height, cfg.width, 3) for key, cfg in self.config.cameras.items()},
        }

    @property
    def is_connected(self):
        return self.arm is not None and all(cam.is_connected for cam in self.cameras.values())

    @property
    def is_calibrated(self):
        return self.mapping is not None

    def calibrate(self):
        data = load_mapping(self.config.mapping_file, self.config.gripper)
        if data["can_interface"] != self.config.can_interface:
            raise ValueError("CAN interface differs from calibration")
        self.mapping = data

    def connect(self, calibrate=True):
        if self.arm is not None:
            raise RuntimeError("Lumos already connected")
        self._measured_limit_margins()
        self.last_action_diagnostics = None
        self.calibrate()
        if self.startup_strategy not in ("home", *CURRENT_STRATEGIES):
            raise ValueError("Startup strategy must be home, current_aligned, current_relative or leader_target")
        current_reference = self._current_startup_preflight() if self.startup_strategy in CURRENT_STRATEGIES else None
        if not self.config.sdk_root:
            raise ValueError("sdk_root is required: pass the startouch_sdk source directory")
        sdk_root = _legacy.find_sdk_root(Path(self.config.sdk_root))
        self._gripper_follow_control = (
            TypeLJFollowControl.from_sdk(sdk_root, self.config.gripper_follow_kp, self.config.gripper_follow_kd)
            if self.config.gripper_follow_kp is not None else None
        )
        from startouchclass import SingleArm

        logger.warning("SingleArm construction may ENABLE MOTORS, including in preview mode")
        try:
            if self.before_control_write is not None:
                self.before_control_write()
            with _legacy.sdk_initialization_directory(sdk_root):
                if current_reference is not None:
                    latest = self._current_startup_preflight()
                    if latest[0] is not current_reference[0] or latest[1:] != current_reference[1:]:
                        raise RuntimeError("Current startup preflight changed before SDK construction")
                if self.before_control_write is not None:
                    self.before_control_write()
                self.arm = SingleArm(
                    can_interface_=self.config.can_interface,
                    gripper=self.config.gripper,
                    enable_fd_=False,
                )
            if self.before_control_write is not None:
                self.before_control_write()
            seed = None
            if current_reference is not None:
                # Native construction already enables the motors. The first
                # fixed target can only be sent after a new real CAN group;
                # this is an explicit engineering path, not an atomic hold.
                started = time.monotonic()
                readiness_deadline = min(started + current_reference[5], current_reference[6])
                seed = self._current_startup_positions(current_reference, deadline=readiness_deadline)
                if self.before_control_write is not None:
                    self.before_control_write()
                if time.monotonic() >= readiness_deadline:
                    raise RuntimeError("Current startup SDK readiness deadline expired: stage=before_seed")
                if self.arm.set_joint_raw(seed, [0.0] * 6) is False:
                    raise RuntimeError("SDK rejected current startup seed")
            time.sleep(self.config.startup_wait_s)
            if self.before_control_write is not None:
                self.before_control_write()
            positions = (
                self._current_startup_positions(current_reference, seed=seed)
                if current_reference is not None
                else vector(self.arm.get_joint_positions(), 6, "Lumos feedback")
            )
            if self.before_control_write is not None:
                self.before_control_write()
            if self.config.execute and current_reference is None:
                errors = [abs(a - b) for a, b in zip(positions, self.mapping["lumos_home_rad"])]
                tolerances = [self.config.startup_tolerance_rad] * 6
                tolerances[1] = self.config.startup_joint2_tolerance_rad
                if any(error > tolerance for error, tolerance in zip(errors, tolerances)):
                    raise RuntimeError(
                        "Lumos is away from calibrated home; restore pose or recalibrate "
                        f"(current={positions}, calibrated={self.mapping['lumos_home_rad']}, "
                        f"errors={errors}, tolerances={tolerances})"
                    )
                if not self._measured_positions_within_limits(positions):
                    raise RuntimeError("Measured startup pose is outside configured joint limits")
            self._previous_target = seed if seed is not None else self._bounded_joint_target(positions)
            self._last_send = time.monotonic()
            self.last_sent_action = None
            for cam in self.cameras.values():
                cam.connect()
            self.configure()
        except BaseException:
            self.disconnect()
            raise

    def _measured_limit_margins(self):
        margins = getattr(self, "measured_limit_margin_rad", (0.0,) * 6)
        if (
            not isinstance(margins, (list, tuple)) or len(margins) != 6
            or any(type(value) not in (int, float) or not math.isfinite(value) or value < 0 for value in margins)
        ):
            raise ValueError("measured_limit_margin_rad must contain six finite nonnegative numbers")
        return margins

    def _measured_positions_within_limits(self, positions):
        margins = self._measured_limit_margins()
        return all(
            lo - margin <= q <= hi + margin
            for q, lo, hi, margin in zip(positions, *self.limits, margins)
        )

    def _bounded_joint_target(self, positions):
        # Measurement margins never enlarge a commanded position or the
        # limiter origin. Preserve the original measurements in references.
        if self.limits is None:
            return list(positions)
        return [max(lo, min(hi, q)) for q, lo, hi in zip(positions, *self.limits)]

    def _current_startup_preflight(self):
        """Reject an absent/expired engineering reference before SDK import."""
        reference = self.startup_reference
        if not self.config.execute or not callable(self.after_sdk_startup) or not isinstance(reference, dict):
            raise RuntimeError("Current startup requires a validated preflight and session CAN callback")
        try:
            if reference["startup_strategy"] not in CURRENT_STRATEGIES or reference["startup_strategy"] != self.startup_strategy:
                raise ValueError("strategy mismatch")
            offset = (
                tuple(vector(reference["session_offset_sdk_rad"], 6, "Current relative session offset"))
                if self.startup_strategy == "current_relative" else None
            )
            anchor = vector(reference["actual_q"], 7, "Current startup reference")
            stamps = vector(reference["frame_stamps"], 7, "Current startup frame timestamps")
            tolerances = vector(reference["tolerance_rad"], 6, "Current startup tolerances")
            sdk_tolerances = vector(reference["sdk_tolerance_rad"], 6, "Current startup SDK tolerances")
            captured, expiry = float(reference["captured_monotonic"]), float(reference["valid_until"])
            ready_budget = reference["sdk_ready_budget_s"]
            now = time.monotonic()
            if (
                not all(math.isfinite(v) for v in (captured, expiry))
                or type(ready_budget) not in (int, float)
                or not math.isfinite(ready_budget)
                or not 0 < ready_budget <= 0.1
                or not max(stamps) <= captured <= now <= expiry
                or any(not 0 < tolerance <= math.radians(2) for tolerance in tolerances)
                or any(tolerance <= 0 for tolerance in sdk_tolerances)
                or not 0 <= anchor[6] <= (1 if self.startup_strategy == "leader_target" else 0.01)
                or anchor[1] < -1.57
                or not self._measured_positions_within_limits(anchor)
            ):
                raise ValueError("invalid, expired or out-of-bounds reference")
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError("Current startup requires a valid, fresh preflight reference") from exc
        return reference, tuple(anchor), tuple(tolerances), tuple(sdk_tolerances), offset, ready_budget, expiry

    def _current_startup_positions(self, validated, *, seed=None, deadline=None):
        reference, anchor, tolerances, sdk_tolerances, _offset, _budget, _expiry = validated
        if seed is None and (deadline is None or not math.isfinite(deadline)):
            raise RuntimeError("Current startup requires a fixed SDK readiness deadline")
        last_mismatch = None

        def readiness_expired():
            message = "Current startup SDK readiness deadline expired: stage=before_seed"
            if last_mismatch is not None:
                message += "; last " + last_mismatch
            return RuntimeError(message)

        while True:
            if self.before_control_write is not None:
                self.before_control_write()
            if deadline is not None and time.monotonic() >= deadline:
                raise readiness_expired()
            positions = vector(self.arm.get_joint_positions(), 6, "Current startup SDK positions")
            if self.before_control_write is not None:
                self.before_control_write()
            if deadline is not None and time.monotonic() >= deadline:
                raise readiness_expired()
            # Each attempt obtains a new enabled CAN group after the SDK read;
            # the callback retains every original motion and fault guard.
            try:
                actual = vector(
                    self.after_sdk_startup(reference, deadline=deadline)
                    if deadline is not None else self.after_sdk_startup(reference),
                    7, "Current startup CAN positions",
                )
            except RuntimeError as exc:
                if deadline is not None and time.monotonic() >= deadline and last_mismatch is not None:
                    raise RuntimeError(f"{exc}; last {last_mismatch}") from exc
                raise
            if self.before_control_write is not None:
                self.before_control_write()
            if (
                not 0 <= actual[6] <= (1 if self.startup_strategy == "leader_target" else 0.01)
                or not self._measured_positions_within_limits(actual)
            ):
                raise RuntimeError("Current startup CAN pose is outside configured limits")
            if any(
                abs(q - initial) > min(math.radians(2), tolerance)
                for q, initial, tolerance in zip(actual, anchor, tolerances)
            ):
                raise RuntimeError("Current startup pose drifted from the fixed preflight reference")
            if seed is not None and any(
                abs(q - initial) > min(math.radians(2), tolerance)
                for q, initial, tolerance in zip(actual, seed, tolerances)
            ):
                raise RuntimeError("Current startup pose drifted from the fixed SDK seed")
            if deadline is not None and time.monotonic() >= deadline:
                raise readiness_expired()
            mismatch = None
            for i, (q, actual_q, tolerance) in enumerate(zip(positions, actual, sdk_tolerances), 1):
                if abs(q - actual_q) > tolerance:
                    mismatch = (
                        "Current startup SDK and passive CAN positions disagree: "
                        f"stage={'before_seed' if seed is None else 'after_seed'}, J{i}: "
                        f"SDK={q:.12g} rad, CAN={actual_q:.12g} rad, "
                        f"delta(SDK-CAN)={q - actual_q:+.12g} rad, tolerance={tolerance:.12g} rad, "
                        f"sdk_all_zero={all(value == 0.0 for value in positions)}"
                    )
                    break
            if mismatch is None:
                return self._bounded_joint_target(actual[:6])
            if seed is not None:
                raise RuntimeError(mismatch)
            last_mismatch = mismatch
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise readiness_expired()
            time.sleep(min(0.005, remaining))
            if self.before_control_write is not None:
                self.before_control_write()
            if time.monotonic() >= deadline:
                raise readiness_expired()

    def configure(self):
        # Keep the SDK's own motor/torque configuration and protections unchanged.
        pass

    def get_observation(self):
        if not self.is_connected:
            raise RuntimeError("Lumos is not connected")
        positions = vector(self.arm.get_joint_positions(), 6, "Lumos feedback")
        obs = dict(zip(JOINT_KEYS, positions))
        if self.config.gripper:
            opening = float(self.arm.get_gripper_position())
            if not math.isfinite(opening):
                raise RuntimeError("Invalid gripper feedback")
            obs["gripper.pos"] = opening
        for key, cam in self.cameras.items():
            obs[key] = cam.async_read()
        return obs

    def send_action(self, action):
        if not self.is_connected:
            raise RuntimeError("Lumos is not connected")
        if set(action) != set(self.action_features):
            raise ValueError(f"Expected action keys: {list(self.action_features)}")
        desired = vector([action[key] for key in JOINT_KEYS], 6, "Lumos action")
        requested = desired.copy()
        grip = None
        if self.config.gripper:
            grip = float(action["gripper.pos"])
            if not math.isfinite(grip):
                raise ValueError("Invalid gripper action")
            grip = max(0.0, min(1.0, grip))
        if self.limits is not None:
            desired = [max(lo, min(hi, q)) for q, lo, hi in zip(desired, *self.limits)]
        now = time.monotonic()
        target = limit_target(
            self._previous_target,
            desired,
            now - self._last_send,
            self.config.max_speed_deg_per_sec,
            self.config.max_target_step_deg,
        )
        if self.config.execute:
            if self.before_control_write is not None:
                self.before_control_write()
            if self.arm.set_joint_raw(target, [0.0] * 6) is False:
                raise RuntimeError("SDK rejected joint command")
        # The arm write has succeeded even if a later gripper guard pauses or
        # rejects this action. Future limiting must start at that sent target.
        self._previous_target = target
        self._last_send = now
        self.last_action_diagnostics = {
            "requested_sdk_rad": requested,
            "limit_clipped_sdk_rad": desired.copy(),
            "sent_sdk_rad": target.copy(),
            "joint_limit_clipped": [q != bounded for q, bounded in zip(requested, desired)],
            "rate_limited": [q != bounded for q, bounded in zip(desired, target)],
        }
        if self.config.execute and grip is not None:
            if self.before_control_write is not None:
                self.before_control_write()
            if self._gripper_follow_control is None:
                accepted = self.arm.setGripperPosition(grip)
            else:
                distance, kp, kd = self._gripper_follow_control.command(grip)
                accepted = self.arm.setGripperDistance(distance, kp, kd)
                self.last_action_diagnostics["gripper_position_control"] = dict(
                    distance_m=distance, kp=kp, kd=kd,
                )
            if accepted is False:
                raise RuntimeError("SDK rejected gripper command")
        sent = dict(zip(JOINT_KEYS, target))
        if grip is not None:
            sent["gripper.pos"] = grip
        self.last_sent_action = sent.copy()
        if not self.config.execute:
            logger.info("PREVIEW only, no motion target sent: %s", sent)
        return sent

    def disconnect(self):
        for cam in self.cameras.values():
            try:
                if cam.is_connected:
                    cam.disconnect()
            except Exception:
                logger.exception("Camera disconnect failed")
        arm, self.arm = self.arm, None
        try:
            if arm is not None:
                arm.cleanup()
        except Exception as exc:
            raise SDKCleanupError(f"SDK cleanup failed; actuator state is unconfirmed: {exc}") from exc
        finally:
            self._previous_target = None
            self._last_send = None
            self.last_action_diagnostics = None
            self._gripper_follow_control = None
