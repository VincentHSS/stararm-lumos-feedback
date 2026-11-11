"""TypeLJ geometry in the reviewed startouch SDK build 04041254.

The public opening is distance / 0.08, not GripperState.position, which
the SDK reports as a linear motor-angle fraction. Torque stays motor Nm.
These constants reproduce that SDK geometry; they are not feedback tuning.
"""

import math


class TypeLJ04041254:
    kind = "typelj_sdk_04041254"
    motor_open_rad = -1.257153
    motor_closed_rad = 0.0
    width_m = 0.08
    angle_origin = 1.0562708347405987
    angle_span = -1.1639963156319377

    @classmethod
    def distance(cls, motor_rad):
        if not math.isfinite(motor_rad):
            raise ValueError("TypeLJ motor position must be finite")
        fraction = min(1.0, max(0.0, motor_rad / cls.motor_open_rad))
        alpha = cls.angle_origin - fraction * cls.angle_span
        y = 0.0355 * math.sin(alpha) - 0.015393
        return 2 * (math.sqrt(0.001444 - y * y) - 0.0355 * math.cos(alpha) - 0.01722)

    @classmethod
    def opening(cls, motor_rad):
        return min(1.0, max(0.0, cls.distance(motor_rad) / cls.width_m))

    @classmethod
    def jacobian(cls, motor_rad):
        if not math.isfinite(motor_rad):
            raise ValueError("TypeLJ motor position must be finite")
        # Physical-domain checks happen before this SDK-compatible clamping.
        if not cls.motor_open_rad <= motor_rad <= cls.motor_closed_rad:
            return 0.0
        alpha = cls.angle_origin - motor_rad / cls.motor_open_rad * cls.angle_span
        y = 0.0355 * math.sin(alpha) - 0.015393
        root = math.sqrt(0.001444 - y * y)
        return (
            2
            * (-y * 0.0355 * math.cos(alpha) / root + 0.0355 * math.sin(alpha))
            * (-cls.angle_span / cls.motor_open_rad)
            / cls.width_m
        )
