"""Pure pose checks before SDK construction can enable the Follower.

Follower samples use SDK radians and normalized gripper opening; Leader samples
use servo degrees. Existing paired calibration is read, never changed. No device
SDK is imported and no device, command writer or calibration routine is created.
"""

import math
from collections.abc import Mapping
from numbers import Real
from pathlib import Path

from .pairing import LEADER_STARTUP_DEFAULTS, ROBOT_STARTUP_DEFAULTS, load_limits, load_mapping


class StartupPoseError(RuntimeError):
    code = "startup_pose_mismatch"
    interface = None

    def __init__(self, detail, *, diagnostics=None):
        self.detail = str(detail)
        self.diagnostics = diagnostics
        super().__init__(self.detail)


def _number(value, label):
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(f"{label} must be a finite number in the documented units")
    return float(value)


def _vector(values, size, label):
    if isinstance(values, (str, bytes, Mapping)):
        raise ValueError(f"{label} must contain {size} numeric values")
    values = tuple(values)
    if len(values) != size:
        raise ValueError(f"{label} must contain {size} numeric values")
    return [_number(value, f"{label}[{index}]") for index, value in enumerate(values)]


def _measurement_margins(profile):
    """Use the bound CAN measurement margins; commands keep strict limits."""
    axes = getattr(profile, "axes", None)
    if axes is None:
        return [0.0] * 6  # Preserve callers without a feedback hardware contract.
    margins = _vector([axis["limit_margin"] for axis in axes[:6]], 6, "measured limit margins")
    if any(margin < 0 for margin in margins):
        raise ValueError("Measured limit margins must be nonnegative")
    return margins


def check_startup_pose(profile, follower_q, leader_angles, *, strategy="home", session_offset=None):
    """Reject misaligned arms and return the first measured-opening target.

    The six Leader targets match ``StarArm102.get_action``'s first sample:
    reorder IDs, clip calibrated degree ranges, apply signs/gain, then convert
    to SDK radians relative to the paired home. The first adaptive-filter sample
    is unchanged by that filter. The default requires both arms near home and
    each other. Engineering current starts require a smaller pair error and
    reject SDK initialization hazards. Current-relative starts capture a
    six-axis session offset once; subsequent checks must pass that fixed offset.
    """
    try:
        return _check(profile, follower_q, leader_angles, strategy=strategy, session_offset=session_offset)
    except StartupPoseError:
        raise
    except (AttributeError, ImportError, KeyError, OSError, TypeError, ValueError, OverflowError) as exc:
        raise StartupPoseError(f"Invalid startup inputs or paired configuration: {exc}") from exc


def alignment_reference(profile):
    """Expose only validated paired homes for display, never motion commands.

    Use the actual startup gate at the calibrated pose so the viewer cannot
    silently draw a goal that startup would reject. Missing calibration leaves
    the ordinary model view available, without leaking machine-local paths.
    """
    try:
        path = Path(profile.session["robot"]["mapping_file"]).expanduser()
        if not path.is_absolute():
            path = Path(profile.paths["session_file"]).parent / path
        mapping = load_mapping(str(path), True)
        homes = _vector(mapping["lumos_home_rad"], 6, "lumos_home_rad")
        ids = mapping["joint_map"]
        leader_homes = _vector(mapping["leader_home_deg"], 6, "leader_home_deg")
        angles = dict(zip(ids, leader_homes))
        angles[6] = 0.0  # Gripper alignment is deliberately outside the gate.
        result = check_startup_pose(profile, homes + [0.5], angles)
        model_homes = _vector(
            [profile.to_model(index, value) for index, value in enumerate(homes)],
            6, "calibrated URDF home",
        )
        return dict(
            available=True,
            source="paired_calibration",
            target_sdk_rad=homes,
            target_urdf_rad=model_homes,
            leader_ids=list(ids),
            leader_home_deg=leader_homes,
            tolerance_rad=result["tolerance_rad"],
            display_only=True,
            gripper_alignment_required=False,
        )
    except (AttributeError, ImportError, KeyError, OSError, TypeError, ValueError, RuntimeError, OverflowError):
        return dict(available=False, reason="paired_calibration_unavailable", display_only=True)


def current_alignment_reference(profile, follower_q):
    """Inverse-map a measured pose for display without changing calibration.

    The returned Leader angles retain the viewer's ``leader_home_deg`` field
    name for compatibility; they describe this measured pose, not new homes.
    Only a reference accepted by the engineering current-aligned gate is shown.
    """
    try:
        actual = _vector(follower_q, 7, "follower_q (SDK radians, normalized opening)")
        robot, leader = profile.session["robot"], profile.session["teleop"]
        path = Path(robot["mapping_file"]).expanduser()
        if not path.is_absolute():
            path = Path(profile.paths["session_file"]).parent / path
        mapping = load_mapping(str(path), True)
        signs = _vector(leader.get("signs", LEADER_STARTUP_DEFAULTS["signs"]), 6, "teleop.signs")
        gain = _number(leader.get("gain", LEADER_STARTUP_DEFAULTS["gain"]), "teleop.gain")
        if any(sign not in (-1, 1) for sign in signs) or not 0 < gain <= 1:
            raise ValueError("Leader signs must be -1/+1 and gain must be in (0, 1]")
        homes = _vector(mapping["lumos_home_rad"], 6, "calibrated lumos_home_rad")
        angles_deg = [
            leader_home + math.degrees((q - home) / (sign * gain))
            for q, home, leader_home, sign in zip(actual[:6], homes, mapping["leader_home_deg"], signs)
        ]
        if any(
            not math.isfinite(angle) or not lo <= angle <= hi
            for angle, lo, hi in zip(
                angles_deg, mapping["leader_range_min_deg"], mapping["leader_range_max_deg"]
            )
        ):
            raise ValueError("Measured Follower pose is outside the calibrated Leader reach")
        ids = mapping["joint_map"]
        angles = dict(zip(ids, angles_deg))
        angles[6] = 0.0
        result = check_startup_pose(profile, actual, angles, strategy="current_aligned")
        model_pose = _vector(
            [profile.to_model(index, value) for index, value in enumerate(actual[:6])],
            6, "current URDF pose",
        )
        return dict(
            available=True,
            source="current_follower_pose",
            target_sdk_rad=actual[:6],
            target_urdf_rad=model_pose,
            leader_ids=list(ids),
            leader_home_deg=angles_deg,
            tolerance_rad=result["tolerance_rad"],
            display_only=True,
            gripper_alignment_required=False,
        )
    except (AttributeError, ImportError, KeyError, OSError, TypeError, ValueError, RuntimeError, OverflowError):
        return dict(available=False, reason="current_follower_pose_unavailable", display_only=True)


def relative_alignment_reference(profile, follower_q):
    """Display a current Follower / calibrated Leader origin pair, never bind it.

    The real session captures its offset from fresh samples at startup. This
    reference only shows a reachable Leader starting pose and never changes or
    substitutes the original paired calibration.
    """
    try:
        actual = _vector(follower_q, 7, "follower_q (SDK radians, normalized opening)")
        path = Path(profile.session["robot"]["mapping_file"]).expanduser()
        if not path.is_absolute():
            path = Path(profile.paths["session_file"]).parent / path
        mapping = load_mapping(str(path), True)
        ids = mapping["joint_map"]
        leader_homes = _vector(mapping["leader_home_deg"], 6, "leader_home_deg")
        if any(
            not lo <= home <= hi for home, lo, hi in zip(
                leader_homes, mapping["leader_range_min_deg"], mapping["leader_range_max_deg"],
            )
        ):
            raise ValueError("Calibrated Leader homes are outside its reachable ranges")
        angles = dict(zip(ids, leader_homes))
        angles[6] = 0.0
        result = check_startup_pose(profile, actual, angles, strategy="current_relative")
        model_pose = _vector(
            [profile.to_model(index, value) for index, value in enumerate(actual[:6])],
            6, "current URDF pose",
        )
        return dict(
            available=True,
            source="current_relative_origin",
            target_sdk_rad=actual[:6],
            target_urdf_rad=model_pose,
            leader_ids=list(ids),
            leader_home_deg=leader_homes,
            tolerance_rad=result["tolerance_rad"],
            display_only=True,
            gripper_alignment_required=False,
        )
    except (AttributeError, ImportError, KeyError, OSError, TypeError, ValueError, RuntimeError, OverflowError):
        return dict(available=False, reason="current_relative_origin_unavailable", display_only=True)


def _check(profile, follower_q, leader_angles, *, strategy, session_offset):
    if strategy not in ("home", "current_aligned", "current_relative", "leader_target"):
        raise ValueError("Startup strategy must be home, current_aligned, current_relative or leader_target")
    if strategy != "current_relative" and session_offset is not None:
        raise ValueError("A session offset is only valid for current_relative startup")
    if strategy != "home" and getattr(profile, "engineering", False) is not True:
        raise ValueError("Current-pose startup requires an engineering profile")
    session = profile.session
    if not isinstance(session, dict):
        raise ValueError("session must be an object")
    robot, leader = session["robot"], session["teleop"]
    if not isinstance(robot, dict) or not isinstance(leader, dict):
        raise ValueError("robot and teleop must be configuration objects")
    if robot.get("type") != "lumos" or leader.get("type") != "stararm102":
        raise ValueError("Lumos / StarArm102 paired configuration required")
    if robot.get("gripper", True) is not True or leader.get("gripper", True) is not True:
        raise ValueError("Seven-axis TypeLJ and Leader gripper configuration required")

    def configured(section, defaults, name):
        if name in section:
            return section[name]
        return defaults[name]

    def existing_path(value):
        if not isinstance(value, (str, Path)) or not str(value):
            raise ValueError("Existing paired mapping and joint-limit paths required")
        path = Path(value).expanduser()
        if not path.is_absolute():
            path = Path(profile.paths["session_file"]).parent / path
        return path.resolve()

    mapping_path = existing_path(robot["mapping_file"])
    if mapping_path != existing_path(leader["mapping_file"]):
        raise ValueError("Both devices must use the same existing paired mapping")
    mapping = load_mapping(str(mapping_path), True)
    limits = load_limits(str(existing_path(robot["joint_limits_file"])))
    lower, upper = limits
    measurement_margins = _measurement_margins(profile)
    joint_map = configured(leader, LEADER_STARTUP_DEFAULTS, "joint_map")
    if (
        not isinstance(joint_map, (list, tuple))
        or len(joint_map) != 6
        or any(type(sid) is not int for sid in joint_map)
        or sorted(joint_map) != list(range(6))
        or list(joint_map) != mapping["joint_map"]
    ):
        raise ValueError("Leader joint_map must match the paired permutation of IDs 0..5")
    if mapping["leader_port"] != configured(leader, LEADER_STARTUP_DEFAULTS, "port"):
        raise ValueError("Leader port differs from the paired calibration")
    if mapping["can_interface"] != configured(robot, ROBOT_STARTUP_DEFAULTS, "can_interface"):
        raise ValueError("CAN interface differs from the paired calibration")
    signs = _vector(configured(leader, LEADER_STARTUP_DEFAULTS, "signs"), 6, "teleop.signs")
    gain = _number(configured(leader, LEADER_STARTUP_DEFAULTS, "gain"), "teleop.gain")
    if any(sign not in (-1, 1) for sign in signs) or not 0 < gain <= 1:
        raise ValueError("Leader signs must be -1/+1 and gain must be in (0, 1]")
    tolerance = _number(
        configured(robot, ROBOT_STARTUP_DEFAULTS, "startup_tolerance_rad"), "startup_tolerance_rad"
    )
    joint2_tolerance = _number(
        configured(robot, ROBOT_STARTUP_DEFAULTS, "startup_joint2_tolerance_rad"), "startup_joint2_tolerance_rad"
    )
    if tolerance <= 0 or joint2_tolerance <= 0:
        raise ValueError("Existing startup tolerances must be positive SDK radians")
    pair_tolerance = configured(robot, ROBOT_STARTUP_DEFAULTS, "startup_pair_tolerance_rad")
    if pair_tolerance is not None:
        pair_tolerance = _number(pair_tolerance, "startup_pair_tolerance_rad")
        if pair_tolerance <= 0:
            raise ValueError("startup_pair_tolerance_rad must be positive SDK radians or None")
    tolerances = [tolerance] * 6
    tolerances[1] = joint2_tolerance
    if strategy != "home":
        step = _number(robot.get("max_target_step_deg", 2), "max_target_step_deg")
        if step <= 0:
            raise ValueError("max_target_step_deg must be positive")
        tolerances = [min(limit, math.radians(step), math.radians(2)) for limit in tolerances]
    pair_tolerances = [pair_tolerance] * 6 if strategy == "home" and pair_tolerance is not None else list(tolerances)

    actual = _vector(follower_q, 7, "follower_q (SDK radians, normalized opening)")
    if not 0 <= actual[6] <= 1:
        raise ValueError("Follower gripper must use normalized opening in [0, 1]")
    if strategy != "home":
        if actual[1] < -1.57:
            raise StartupPoseError(
                "Current-pose startup requires SDK J2 >= -1.57 rad",
                diagnostics=dict(strategy=strategy, reason="joint2_startup", position_rad=actual[1], minimum_rad=-1.57),
            )
        if strategy != "leader_target" and actual[6] > 0.01:
            raise StartupPoseError(
                "Current-pose startup requires closed gripper opening <= 0.01; SDK construction closes it",
                diagnostics=dict(strategy=strategy, reason="gripper_not_closed", opening=actual[6], maximum_opening=0.01),
            )
    if (
        not isinstance(leader_angles, Mapping)
        or any(type(sid) is not int for sid in leader_angles)
        or set(leader_angles) != set(joint_map) | {6}
    ):
        raise ValueError("Leader angles must contain the complete IDs 0..6 in degrees")
    angles = {sid: _number(value, f"Leader ID {sid} angle_deg") for sid, value in leader_angles.items()}
    if strategy == "current_relative" and session_offset is None and any(
        not lo <= angles[sid] <= hi
        for sid, lo, hi in zip(
            joint_map, mapping["leader_range_min_deg"], mapping["leader_range_max_deg"],
        )
    ):
        raise StartupPoseError("Relative startup origin requires raw Leader arm angles inside calibrated ranges")
    homes = _vector(mapping["lumos_home_rad"], 6, "calibrated lumos_home_rad")
    base_target = [
        home
        + sign
        * gain
        * math.radians(max(lo, min(hi, angles[sid])) - leader_home)
        for sid, sign, home, leader_home, lo, hi in zip(
            joint_map,
            signs,
            homes,
            mapping["leader_home_deg"],
            mapping["leader_range_min_deg"],
            mapping["leader_range_max_deg"],
        )
    ]
    if not all(math.isfinite(value) for value in base_target):
        raise ValueError("Mapped Leader targets are not finite SDK radians")
    offset = None
    target = base_target
    if strategy == "leader_target":
        # This is the adapter's existing absolute-target clipping, not a
        # physical limit change. The first real command is still the CAN seed.
        target = [max(lo, min(hi, desired)) for desired, lo, hi in zip(base_target, lower, upper)]
    if strategy == "current_relative":
        origin = [max(lo, min(hi, q)) for q, lo, hi in zip(actual[:6], lower, upper)]
        offset = (
            [q - base for q, base in zip(origin, base_target)]
            if session_offset is None
            else _vector(session_offset, 6, "session_offset (SDK radians)")
        )
        target = origin if session_offset is None else [base + delta for base, delta in zip(base_target, offset)]
        if not all(math.isfinite(value) for value in (*target, *offset)):
            raise ValueError("Session-offset Leader targets are not finite SDK radians")
        # Reconstructing a fixed offset can land a few floating-point ULPs
        # beyond an exact boundary. This is numerical roundoff only, never
        # the CAN measurement margin or permission for an outside target.
        for i, (q, lo, hi) in enumerate(zip(target, lower, upper)):
            roundoff = 4 * max(math.ulp(lo), math.ulp(hi))
            if q < lo and lo - q <= roundoff:
                target[i] = lo
            elif q > hi and q - hi <= roundoff:
                target[i] = hi
    if strategy in ("current_aligned", "current_relative") and target[1] < -1.57:
        raise StartupPoseError(
            "Current-pose startup requires mapped SDK J2 >= -1.57 rad",
            diagnostics=dict(strategy=strategy, reason="joint2_target_startup", position_rad=target[1], minimum_rad=-1.57),
        )
    follower_errors = [abs(q - home) for q, home in zip(actual[:6], homes)]
    leader_errors = [abs(q - home) for q, home in zip(target, homes)]
    pair_errors = [abs(q - desired) for q, desired in zip(actual[:6], target)]
    failed, failed_axes = [], []
    for i, (q, desired, lo, hi, limit, pair_limit, follower_error, leader_error, pair_error) in enumerate(
        zip(actual[:6], target, lower, upper, tolerances, pair_tolerances, follower_errors, leader_errors, pair_errors)
    ):
        home_failed = strategy == "home" and (follower_error > limit or leader_error > limit)
        pair_failed = strategy != "leader_target" and pair_error > pair_limit
        measurement_margin = measurement_margins[i]
        actual_in_limits = lo - measurement_margin <= q <= hi + measurement_margin
        if not actual_in_limits or not lo <= desired <= hi or home_failed or pair_failed:
            checks = []
            if not actual_in_limits:
                checks.append("follower_limit")
            if not lo <= desired <= hi:
                checks.append("target_limit")
            if strategy == "home":
                if follower_error > limit:
                    checks.append("follower_home")
                if leader_error > limit:
                    checks.append("leader_home")
            if pair_failed:
                checks.append("pair")
            failed_axes.append(dict(
                axis=i + 1, failed_checks=checks, tolerance_rad=limit,
                pair_tolerance_rad=pair_limit,
                follower_home_error_rad=follower_error,
                leader_home_error_rad=leader_error, pair_error_rad=pair_error,
                follower_rad=q, target_rad=desired, limits_rad=[lo, hi],
                measured_limit_margin_rad=measurement_margin,
            ))
            failed.append(
                f"J{i + 1}: Follower-home {math.degrees(follower_error):.6f} deg; "
                f"Leader-target-home {math.degrees(leader_error):.6f} deg; "
                f"Follower-target {math.degrees(pair_error):.6f} deg; "
                f"allowed {math.degrees(limit):.6f} deg; "
                f"pair allowed {math.degrees(pair_limit):.6f} deg; "
                f"Follower {q:.9g} rad, target {desired:.9g} rad, limits [{lo:.9g}, {hi:.9g}] rad; "
                f"measurement margin {measurement_margin:.9g} rad"
            )
    if failed:
        raise StartupPoseError(
            "Startup pose mismatch. " + " | ".join(failed),
            diagnostics=dict(strategy=strategy, axes=failed_axes),
        )
    return dict(
        startup_strategy=strategy,
        session_offset_sdk_rad=offset,
        leader_base_target_sdk=base_target,
        follower_q_sdk=actual,
        leader_target_sdk=target + [actual[6]],
        tolerance_rad=tolerances,
        pair_tolerance_rad=pair_tolerances,
        follower_home_error_rad=follower_errors,
        leader_home_error_rad=leader_errors,
        pair_error_rad=pair_errors,
    )
