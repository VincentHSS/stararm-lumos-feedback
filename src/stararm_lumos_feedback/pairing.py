"""Pure paired-calibration inputs shared by startup, display and adapters."""

import json
import math
from pathlib import Path

ROBOT_STARTUP_DEFAULTS = dict(
    can_interface="can0", startup_tolerance_rad=0.15, startup_joint2_tolerance_rad=0.40,
    startup_pair_tolerance_rad=None,
)
LEADER_STARTUP_DEFAULTS = dict(
    port="/dev/ttyUSB0", joint_map=(0, 1, 2, 3, 4, 5), signs=(-1, 1, 1, 1, -1, -1), gain=1.0,
)


def _vector(values, name):
    result = [float(value) for value in values]
    if len(result) != 6 or not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must contain 6 finite numbers")
    return result


def load_mapping(path: str, gripper: bool) -> dict:
    if not path:
        raise ValueError("mapping_file is required; run the original --calibrate first")
    data = json.loads(Path(path).expanduser().read_text(encoding="utf-8"))
    if data.get("format_version") != 1:
        raise ValueError("Unsupported calibration version; run the original --calibrate")
    if data.get("gripper_enabled") is not gripper:
        raise ValueError("gripper must match the calibration file")
    for key in ("leader_home_deg", "leader_range_min_deg", "leader_range_max_deg", "lumos_home_rad"):
        data[key] = _vector(data[key], key)
    if any(hi - lo < 2 for lo, hi in zip(data["leader_range_min_deg"], data["leader_range_max_deg"])):
        raise ValueError("Leader calibration range must be at least two degrees per joint")
    if gripper:
        closed, opened = (float(data["gripper_range"][key]) for key in ("closed_deg", "open_deg"))
        if not all(math.isfinite(value) for value in (closed, opened)):
            raise ValueError("gripper_range must contain 2 finite numbers")
        if abs(opened - closed) < 2:
            raise ValueError("Invalid calibrated gripper range")
    return data


def load_limits(path: str):
    if not path:
        return None
    data = json.loads(Path(path).expanduser().read_text(encoding="utf-8"))
    low, high = _vector(data["lower_rad"], "lower_rad"), _vector(data["upper_rad"], "upper_rad")
    if any(lo >= hi for lo, hi in zip(low, high)):
        raise ValueError("Each lower joint bound must be smaller than its upper bound")
    return low, high
