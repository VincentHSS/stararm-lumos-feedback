import math

from stararm_lumos_feedback.pairing import load_limits as load_limits
from stararm_lumos_feedback.pairing import load_mapping as load_mapping

JOINT_KEYS = tuple(f"joint_{i}.pos" for i in range(1, 7))


def vector(values, size: int, name: str) -> list[float]:
    result = [float(v) for v in values]
    if len(result) != size or not all(math.isfinite(v) for v in result):
        raise ValueError(f"{name} must contain {size} finite numbers")
    return result


def features(gripper: bool) -> dict:
    result = dict.fromkeys(JOINT_KEYS, float)
    if gripper:
        result["gripper.pos"] = float
    return result


def limit_target(previous, desired, dt, speed_deg_per_sec, max_step_deg):
    previous = vector(previous, 6, "previous target")
    desired = vector(desired, 6, "desired target")
    if not all(math.isfinite(v) and v > 0 for v in (speed_deg_per_sec, max_step_deg)):
        raise ValueError("Motion limits must be finite and positive")
    if not math.isfinite(dt) or dt < 0:
        raise ValueError("Invalid elapsed time")
    step = math.radians(min(max_step_deg, speed_deg_per_sec * dt))
    return [old + max(-step, min(step, new - old)) for old, new in zip(previous, desired)]
