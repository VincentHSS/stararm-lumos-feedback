"""Validate real adapter configuration without hardware or LeRobot registry changes."""

import os
import subprocess
import sys
import unittest
from pathlib import Path

SCENARIO = r"""
from dataclasses import dataclass
import math
import sys
import types

@dataclass
class Config:
    id: str = "software-fixture"
    calibration_dir: str = ""

    @classmethod
    def register_subclass(cls, name):
        return lambda implementation: implementation

class Device:
    def __init__(self, config):
        pass

for name, exports in {
    "lerobot": {},
    "lerobot.cameras": {"CameraConfig": Config, "make_cameras_from_configs": lambda configs: {}},
    "lerobot.robots": {"Robot": Device, "RobotConfig": Config},
    "lerobot.teleoperators": {"Teleoperator": Device, "TeleoperatorConfig": Config},
}.items():
    module = types.ModuleType(name)
    module.__dict__.update(exports)
    sys.modules[name] = module

from lerobot_robot_lumos_stararm102.config import LumosConfig
from lerobot_robot_lumos_stararm102.lumos import Lumos

scenario = sys.argv[1]
if scenario == "none":
    for config in (
        LumosConfig(),
        LumosConfig(startup_pair_tolerance_rad=None),
        LumosConfig(startup_pair_tolerance_rad=None,
                    startup_tolerance_rad=0.07, startup_joint2_tolerance_rad=0.12),
    ):
        original_home_tolerances = (config.startup_tolerance_rad, config.startup_joint2_tolerance_rad)
        robot = Lumos(config)
        assert robot.config.startup_pair_tolerance_rad is None
        assert (robot.config.startup_tolerance_rad,
                robot.config.startup_joint2_tolerance_rad) == original_home_tolerances
        assert robot.arm is None
    defaults = LumosConfig()
    assert (defaults.startup_tolerance_rad, defaults.startup_joint2_tolerance_rad) == (0.15, 0.40)
    for field in ("startup_tolerance_rad", "startup_joint2_tolerance_rad"):
        try:
            Lumos(LumosConfig(startup_pair_tolerance_rad=None, **{field: 0.0}))
        except ValueError:
            pass
        else:
            raise AssertionError("None pair tolerance bypassed home validation: " + field)
elif scenario == "positive":
    for tolerance in (0.001, 0.25, math.pi, 1e100):
        config = LumosConfig(startup_pair_tolerance_rad=tolerance,
                             startup_tolerance_rad=0.07, startup_joint2_tolerance_rad=0.12)
        robot = Lumos(config)
        assert robot.config.startup_pair_tolerance_rad == tolerance
        assert (config.startup_tolerance_rad, config.startup_joint2_tolerance_rad) == (0.07, 0.12)
        assert robot.arm is None
elif scenario == "invalid":
    for tolerance in (False, True, 0, 0.0, -1, -0.01, math.nan, math.inf, -math.inf):
        try:
            Lumos(LumosConfig(startup_pair_tolerance_rad=tolerance))
        except ValueError:
            pass
        else:
            raise AssertionError("Invalid pair tolerance accepted: " + repr(tolerance))
else:
    raise AssertionError("Unknown test scenario: " + scenario)

assert "startouchclass" not in sys.modules
assert "fashionstar_uart_sdk" not in sys.modules
"""


class PairConfigTests(unittest.TestCase):
    def scenario(self, name):
        env = os.environ.copy()
        root = Path(__file__).resolve().parents[1]
        env["PYTHONPATH"] = os.pathsep.join((str(root / "src"), env.get("PYTHONPATH", "")))
        result = subprocess.run(
            [sys.executable, "-B", "-c", SCENARIO, name],
            env=env, text=True, capture_output=True, timeout=20,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_optional_none_preserves_existing_home_tolerances_and_validation(self):
        self.scenario("none")

    def test_finite_positive_pair_tolerance_is_allowed_independently(self):
        self.scenario("positive")

    def test_boolean_zero_negative_and_nonfinite_pair_tolerances_are_rejected(self):
        self.scenario("invalid")
