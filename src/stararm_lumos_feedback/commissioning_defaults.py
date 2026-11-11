"""Engineering starting values for Lumos commissioning without Leader output.

No hardware imports, measurements, file access or mass assumptions occur here.
The caller must restrict the result to inspect/follow/monitor commissioning. A generated
policy is not a measured no-load baseline and cannot authorize Leader output.
"""

import math

from .gripper import TypeLJ04041254

HARDWARE_KEYS = (
    "feedback_id",
    "motor_id",
    "p_max",
    "v_max",
    "t_max",
    "sdk_scale",
    "sdk_offset",
    "urdf_scale",
    "urdf_offset",
    "torque_scale",
)


def _number(value, label, *, positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{label}: a verified finite number is required")
    if positive and value <= 0:
        raise ValueError(f"{label}: must be positive")
    return value


def _hardware(axes):
    if not isinstance(axes, (list, tuple)) or len(axes) != 7:
        raise ValueError("Exactly six verified arm axes and one TypeLJ axis required")
    result, identities = [], set()
    for i, axis in enumerate(axes):
        if not isinstance(axis, dict) or any(key not in axis for key in HARDWARE_KEYS):
            raise ValueError(f"axis {i + 1}: verified IDs, ranges and transforms required")
        entry = {key: axis[key] for key in HARDWARE_KEYS}
        for key in ("feedback_id", "motor_id"):
            maximum = 0x7FF if key == "feedback_id" else 15
            if type(entry[key]) is not int or not 0 <= entry[key] <= maximum:
                raise ValueError(f"axis {i + 1}: invalid {key}")
        identity = entry["feedback_id"], entry["motor_id"]
        if identity in identities:
            raise ValueError("Duplicate verified CAN feedback identity")
        identities.add(identity)
        for key in HARDWARE_KEYS[2:]:
            _number(entry[key], f"axis {i + 1}.{key}", positive=key in ("p_max", "v_max", "t_max"))
        if any(entry[key] == 0 for key in ("sdk_scale", "urdf_scale", "torque_scale")):
            raise ValueError("Coordinate and torque transforms cannot be zero")
        if i < 6 and not math.isclose(
            entry["sdk_scale"] * entry["urdf_scale"] * entry["torque_scale"], 1, rel_tol=1e-6
        ):
            raise ValueError("Verified arm transforms must preserve joint power")
        if i == 6 and any(
            entry[key] != expected
            for key, expected in (("sdk_scale", 1), ("sdk_offset", 0), ("urdf_scale", 1), ("urdf_offset", 0))
        ):
            raise ValueError("This reviewed TypeLJ converter requires raw SDK motor radians")
        result.append(entry)
    return result


def generate(axes, session, sdk_config):
    """Return numeric commissioning defaults and an unaccepted provenance record.

    ``axes`` supplies already verified IDs, live protocol ranges and transforms.
    ``session`` supplies robot max_speed_deg_per_sec/max_target_step_deg and fps.
    ``sdk_config`` is the reviewed YAML dictionary, including pos_vel_mode
    acceleration/deceleration arrays. This function does not establish the truth
    of those inputs. The normal profile/binding checks remain required.
    """
    axes = _hardware(axes)
    if not isinstance(session, dict) or not isinstance(sdk_config, dict):
        raise ValueError("Session and reviewed SDK configuration dictionaries required")
    fps = session.get("fps")
    if type(fps) is not int or not 10 <= fps <= 100:
        raise ValueError("Commissioning defaults require an integer fps in 10..100")
    robot = session.get("robot", {})
    if not isinstance(robot, dict):
        raise ValueError("Session robot settings required")
    speed_deg = _number(robot.get("max_speed_deg_per_sec"), "max_speed_deg_per_sec", positive=True)
    step_deg = _number(robot.get("max_target_step_deg"), "max_target_step_deg", positive=True)
    if sdk_config.get("gripper_control", {}).get("type") != "TypeLJ":
        raise ValueError("Reviewed SDK must explicitly select TypeLJ")
    motion = sdk_config.get("pos_vel_mode", {})
    accelerations, decelerations = motion.get("max_acc_limits"), motion.get("max_dec_limits")
    if any(
        not isinstance(values, (list, tuple)) or len(values) != 6 for values in (accelerations, decelerations)
    ):
        raise ValueError("Reviewed SDK pos_vel_mode requires six acceleration and deceleration limits")
    sdk_acceleration = []
    for i, (acceleration, deceleration) in enumerate(zip(accelerations, decelerations)):
        _number(acceleration, f"SDK acceleration {i + 1}", positive=True)
        _number(deceleration, f"SDK deceleration {i + 1}")
        if deceleration >= 0:
            raise ValueError("SDK deceleration limits must be negative")
        sdk_acceleration.append(max(acceleration, abs(deceleration)))

    period = 1 / fps
    requested_speed = min(math.radians(speed_deg), math.radians(step_deg) / period)
    query = max(0.02, min(0.05, period))
    age = max(0.1, 3 * period)
    guards = dict(
        can_bitrate=1_000_000,
        can_skew_s=min(age / 2, period),
        leader_query_s=query,
        leader_gap_s=max(0.2, 4 * period),
        cycle_timeout_s=max(0.15, 4 * period + 2 * query),
        startup_feedback_s=5.0,
    )
    # On this converter's complete physical interval y=.0355*sin(alpha)-.015393
    # is positive and <=.020107. Bound both trigonometric factors by 1 and the
    # square-root denominator from below; endpoint-only sampling misses the
    # actual interior Jacobian maximum. This bounds coordinates, not load.
    y_max = 0.0355 - 0.015393
    denominator_min = math.sqrt(0.001444 - y_max * y_max)
    gripper_jacobian = (
        2
        * (y_max * 0.0355 / denominator_min + 0.0355)
        * abs(TypeLJ04041254.angle_span / TypeLJ04041254.motor_open_rad)
        / TypeLJ04041254.width_m
    )
    gripper_position_lsb = 2 * axes[6]["p_max"] / 65535
    motor_span = abs(TypeLJ04041254.motor_open_rad)
    motor_speed = min(axes[6]["v_max"], motor_span / (4 * period))
    gripper_transform = dict(
        type=TypeLJ04041254.kind,
        motor_margin_rad=4 * gripper_position_lsb,
        max_motor_jump_rad=min(motor_span / 2, 2 * motor_speed * period + 4 * gripper_position_lsb),
        max_motor_velocity_rad_s=motor_speed,
        sdk_motor_tolerance_rad=8 * gripper_position_lsb,
    )

    policies, quantization = [], []
    for i, axis in enumerate(axes):
        position_lsb_sdk = 2 * axis["p_max"] / 65535 * abs(axis["sdk_scale"])
        velocity_lsb_raw = 2 * axis["v_max"] / 4095
        torque_lsb = 2 * axis["t_max"] / 4095 * abs(axis["torque_scale"])
        if i < 6:
            position_lsb = position_lsb_sdk * abs(axis["urdf_scale"])
            velocity_lsb = velocity_lsb_raw * abs(axis["sdk_scale"] * axis["urdf_scale"])
            protocol_speed = axis["v_max"] * abs(axis["sdk_scale"] * axis["urdf_scale"])
            max_velocity = min(
                protocol_speed, max(2 * requested_speed * abs(axis["urdf_scale"]), 8 * velocity_lsb)
            )
            max_acceleration = max(
                2 * sdk_acceleration[i] * abs(axis["urdf_scale"]), 8 * velocity_lsb / period
            )
            tolerance, margin = 8 * position_lsb_sdk, 4 * position_lsb_sdk
            hold_error = max(16 * position_lsb, 2 * math.radians(step_deg) * abs(axis["urdf_scale"]))
        else:
            position_lsb = position_lsb_sdk * gripper_jacobian
            velocity_lsb = velocity_lsb_raw * gripper_jacobian
            max_velocity = motor_speed * gripper_jacobian
            max_acceleration = (2 * max_velocity + 8 * velocity_lsb) / period
            tolerance, margin = 8 * position_lsb, 4 * position_lsb
            hold_error = 0.25  # One quarter of normalized opening; diagnostic candidate only.
        axis.update(
            max_age_s=age,
            max_temperature_c=60.0,
            max_jump=2 * max_velocity * period + 4 * position_lsb,
            max_velocity=max_velocity,
            max_acceleration=max_acceleration,
            sdk_tolerance=tolerance,
            limit_margin=margin,
        )
        full_scale = 0.1 * axis["t_max"] * abs(axis["torque_scale"])
        policies.append(
            dict(
                deadzone_nm=4 * torque_lsb,
                velocity_margin=0.0,
                acceleration_margin=0.0,
                full_scale_nm=full_scale,
                filter_s=2 * period,
                acceleration_filter_s=3 * period,
                confirm_s=3 * period,
                release_s=5 * period,
                rise_mw_s=1200 / (6 * period),
                fall_mw_s=1200 / (3 * period),
                hold_error=hold_error,
                hold_load_nm=2 * full_scale,
                hold_speed=max(4 * velocity_lsb, 0.05 * max_velocity),
                hold_confirm_s=max(0.3, 10 * period),
                hold_progress=4 * position_lsb / period,
                hold_release_error=0.5 * hold_error,
                hold_release_load_nm=full_scale,
                hold_release_s=max(0.3, 10 * period),
            )
        )
        quantization.append(dict(position=position_lsb, velocity=velocity_lsb, torque_nm=torque_lsb))

    commissioning = dict(
        generator="lumos_monitor_engineering_v1",
        status="engineering_initial",
        allowed_modes=["inspect", "follow", "monitor"],
        force_authorized=False,
        baseline_status="not_measured",
        hardware_effects_validated=False,
        automatic_hold_accepted=False,
        mass_coverage_inferred=False,
        inputs=dict(
            fps=fps,
            max_speed_deg_per_sec=speed_deg,
            max_target_step_deg=step_deg,
            sdk_pos_vel_acceleration_magnitudes=sdk_acceleration,
            protocol=[{key: axis[key] for key in HARDWARE_KEYS} for axis in axes],
        ),
        quantization=quantization,
        rules=dict(
            deadzone="4 decoded torque LSBs only; no measured friction/model-error allowance",
            motion_margins="zero until measured no-load baseline; never interpreted as calibrated",
            plot_scale="10% of decoded torque range, diagnostic only, not an accepted force threshold",
            arm_velocity="min(protocol speed, max(2 * session target rate, 8 velocity LSBs))",
            arm_acceleration="max(2 * reviewed SDK pos_vel acc/dec magnitude, 8 velocity LSBs / period)",
            gripper_velocity="min(protocol VMAX, verified motor travel / 4 control periods)",
            gripper_coordinate_bound="analytic Jacobian upper bound over the complete reviewed motor interval",
            gripper_sdk_force_position_speed="not used: it does not bound ordinary MIT position mode",
            position_checks="jump: 2 control periods of guard velocity + 4 position LSBs; tolerance: 8 LSBs",
            temperature="60 C commissioning software stop choice; manufacturer thermal rating unverified",
            timing="engineering scheduling budgets from session FPS; not measured CAN/serial latency",
            response="filters 2/3 periods; confirm/release 3/5 periods; 1200 mW ramp over 6/3 periods",
            hold="unaccepted diagnostic candidate; must not authorize force or commissioning motion arbitration",
        ),
    )
    return dict(
        axes=axes,
        guards=guards,
        policy=policies,
        gripper_transform=gripper_transform,
        commissioning=commissioning,
    )
