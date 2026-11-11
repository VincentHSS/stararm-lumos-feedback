"""Opt-in TypeLJ following gains without changing native startup defaults."""

import math
from dataclasses import dataclass

from stararm_lumos_feedback.gripper import TypeLJ04041254

# Smoothly return to the SDK defaults over each endpoint's 10% opening band.
# This is an engineering command schedule, not an enlargement of motor limits.
ENDPOINT_BAND = 0.1


def validate_follow_gains(kp, kd):
    if kp is None and kd is None:
        return
    for label, value, high in (("kp", kp, 20.0), ("kd", kd, 1.0)):
        if type(value) not in (int, float) or not math.isfinite(value) or not 0.1 <= value <= high:
            raise ValueError(f"gripper_follow_{label} must be finite in [0.1, {high}] with both gains set")


@dataclass(frozen=True)
class TypeLJFollowControl:
    kp: float
    kd: float
    endpoint_kp: float
    endpoint_kd: float

    def __post_init__(self):
        validate_follow_gains(self.kp, self.kd)
        if self.kp is None:
            raise ValueError("Explicit TypeLJ following gains required")
        validate_follow_gains(self.endpoint_kp, self.endpoint_kd)
        if self.endpoint_kp is None:
            raise ValueError("Explicit SDK endpoint gains required")

    @classmethod
    def from_sdk(cls, sdk_root, kp, kd):
        import yaml

        data = yaml.safe_load((sdk_root / "src/config/robot_kinematics.yaml").read_text())
        gripper = data.get("gripper_control", {}) if isinstance(data, dict) else {}
        if not isinstance(gripper, dict) or gripper.get("type") != "TypeLJ":
            raise ValueError("Following gains require the reviewed TypeLJ SDK configuration")
        return cls(kp, kd, gripper.get("default_kp"), gripper.get("default_kd"))

    def command(self, opening):
        if type(opening) not in (int, float) or not math.isfinite(opening) or not 0 <= opening <= 1:
            raise ValueError("TypeLJ opening must be finite in [0, 1]")
        phase = min(1.0, min(opening, 1 - opening) / ENDPOINT_BAND)
        weight = phase * phase * (3 - 2 * phase)
        kp = self.endpoint_kp + weight * (self.kp - self.endpoint_kp)
        kd = self.endpoint_kd + weight * (self.kd - self.endpoint_kd)
        return opening * TypeLJ04041254.width_m, kp, kd
