"""Configurable Leader smoothing without connecting either physical arm."""

import importlib.util
import os
import subprocess
import sys
import unittest
from pathlib import Path

SOURCE = Path(__file__).resolve().parents[1] / "src" / "lerobot_robot_lumos_stararm102"
SPEC = importlib.util.spec_from_file_location("leader_filter_fixture", SOURCE / "_legacy.py")
LEGACY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(LEGACY)


ADAPTER_SCENARIO = r"""
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys
import tempfile
import types
from unittest.mock import patch

@dataclass
class Config:
    id: str = "filter-fixture"
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

from lerobot_robot_lumos_stararm102.config import LumosConfig, StarArm102Config
from lerobot_robot_lumos_stararm102.lumos import Lumos, SDKCleanupError
from lerobot_robot_lumos_stararm102.stararm102 import StarArm102
from lerobot_robot_lumos_stararm102 import _legacy

scenario = sys.argv[1]
if scenario == "invalid":
    for name, values in {
        "filter_min_cutoff_hz": [False, True, None, "5", 0, -1, math.nan, math.inf, -math.inf],
        "filter_beta": [False, True, None, ".15", -1, math.nan, math.inf, -math.inf],
    }.items():
        for value in values:
            try:
                StarArm102(StarArm102Config(**{name: value}))
            except ValueError as exc:
                assert name in str(exc)
            else:
                raise AssertionError("Invalid filter parameter accepted: " + repr((name, value)))
    StarArm102(StarArm102Config(filter_min_cutoff_hz=.01, filter_beta=0))
    assert "serial" not in sys.modules and "fashionstar_uart_sdk" not in sys.modules
elif scenario == "defaults":
    config = StarArm102Config()
    leader = StarArm102(config)
    assert config.filter_min_cutoff_hz == _legacy.FILTER_MIN_CUTOFF_HZ == leader._filter.min_cutoff_hz
    assert config.filter_beta == _legacy.FILTER_BETA == leader._filter.beta
elif scenario == "reconnect":
    config = StarArm102Config(filter_min_cutoff_hz=.7, filter_beta=.02,
                             signs=[1]*6, gripper=False, button_enabled=False)
    leader = StarArm102(config)
    leader.mapping = dict(leader_range_min_deg=[-180]*6, leader_range_max_deg=[180]*6,
                          leader_home_deg=[0]*6, lumos_home_rad=[0]*6)
    leader.calibrate = lambda: None
    class UART:
        def __init__(self, **kwargs): self.is_open = True
        def close(self): self.is_open = False
    serial = types.SimpleNamespace(Serial=UART, PARITY_NONE="N")
    sdk = types.SimpleNamespace(UartServoManager=lambda uart: types.SimpleNamespace())
    initial_filter = leader._filter
    with patch.dict(sys.modules, {"serial": serial, "fashionstar_uart_sdk": sdk}), \
         patch.object(_legacy, "ping_fresh", return_value=True):
        leader.connect()
        assert leader._filter is not initial_filter
        assert leader.last_action_diagnostics is None
        assert (leader._filter.min_cutoff_hz, leader._filter.beta) == (.7, .02)
        with patch.object(_legacy, "read_leader", side_effect=[[0]*6, [10]*6]), \
             patch("lerobot_robot_lumos_stararm102.stararm102.time.monotonic", side_effect=[1, 1.01]):
            leader.get_action()
            action = leader.get_action()
        assert 0 < action["joint_1.pos"] < math.radians(10)
        assert all(value == action["joint_1.pos"] for value in action.values())
        used_filter = leader._filter
        assert leader.last_action_diagnostics is not None
        leader.disconnect()
        assert leader.last_action_diagnostics is None
        leader.last_action_diagnostics = {"old_session": True}
        leader.connect()
        assert leader.last_action_diagnostics is None
        assert leader._filter is not used_filter and leader._filter.raw is None
        assert leader._last_sample is None
        assert (leader._filter.min_cutoff_hz, leader._filter.beta) == (.7, .02)
        with patch.object(_legacy, "read_leader", return_value=[40]*6):
            action = leader.get_action()
        assert all(value == math.radians(40) for value in action.values())
        leader.disconnect()
elif scenario == "leader_diagnostics":
    config = StarArm102Config(signs=[1]*6, gripper=False, filter_beta=0)
    leader = StarArm102(config)
    leader.uart = types.SimpleNamespace(is_open=True, close=lambda: None)
    leader.leader = types.SimpleNamespace()
    leader.mapping = dict(leader_range_min_deg=[-10]*6, leader_range_max_deg=[10]*6,
                          leader_home_deg=[0]*6, lumos_home_rad=[0]*6)
    assert leader.last_action_diagnostics is None
    with patch.object(_legacy, "read_leader", side_effect=[[0]*6, [100, -100, 0, 0, 0, 0]]), \
         patch("lerobot_robot_lumos_stararm102.stararm102.time.monotonic", side_effect=[1, 1.01]):
        leader.get_action()
        action = leader.get_action()
    diagnostic = leader.last_action_diagnostics
    assert diagnostic["raw_leader_deg"] == [100, -100, 0, 0, 0, 0]
    assert 10 < diagnostic["filtered_leader_deg"][0] < 100
    assert diagnostic["leader_clipped_deg"] == [10, -10, 0, 0, 0, 0]
    assert diagnostic["leader_range_clipped"] == [True, True, False, False, False, False]
    assert diagnostic["mapped_sdk_rad"] == [action[key] for key in leader.action_features]
    first_filtered = diagnostic["filtered_leader_deg"].copy()
    with patch.object(_legacy, "read_leader", return_value=[100, -100, 0, 0, 0, 0]):
        for index in range(200):
            with patch("lerobot_robot_lumos_stararm102.stararm102.time.monotonic", return_value=1.02 + index*.01):
                latest = leader.get_action()
    assert leader.last_action_diagnostics["filtered_leader_deg"][0] > 99.9
    assert leader.last_action_diagnostics["leader_range_clipped"] == [True, True, False, False, False, False]
    assert latest["joint_1.pos"] == math.radians(10)
    assert latest["joint_2.pos"] == -math.radians(10)
    assert diagnostic["filtered_leader_deg"] == first_filtered
    action["joint_1.pos"] = 999
    assert diagnostic["mapped_sdk_rad"][0] == math.radians(10)
    leader.disconnect()
    assert leader.last_action_diagnostics is None
elif scenario == "robot_diagnostics":
    with tempfile.TemporaryDirectory() as folder:
        limits = Path(folder) / "limits.json"
        limits.write_text(json.dumps(dict(lower_rad=[-1]*6, upper_rad=[1]*6)))
        config = LumosConfig(gripper=False, execute=True, joint_limits_file=str(limits),
                             max_speed_deg_per_sec=100, max_target_step_deg=2)
        robot = Lumos(config)
        commands = []
        robot.arm = types.SimpleNamespace(
            set_joint_raw=lambda q, v: commands.append(q.copy()), cleanup=lambda: None)
        robot._previous_target = [0]*6
        robot._last_send = 10
        assert robot.last_action_diagnostics is None
        requested = [2, -2, .005, 0, .8, -.8]
        action = dict(zip(robot.action_features, requested))
        with patch("lerobot_robot_lumos_stararm102.lumos.time.monotonic", return_value=10.01):
            sent = robot.send_action(action)
        diagnostic = robot.last_action_diagnostics
        bounded = [1, -1, .005, 0, .8, -.8]
        assert diagnostic["requested_sdk_rad"] == requested
        assert diagnostic["limit_clipped_sdk_rad"] == bounded
        assert diagnostic["sent_sdk_rad"] == commands[0] == [sent[key] for key in robot.action_features]
        assert diagnostic["joint_limit_clipped"] == [True, True, False, False, False, False]
        assert diagnostic["rate_limited"] == [True, True, False, False, True, True]
        assert math.isclose(commands[0][0], math.radians(1), abs_tol=1e-12)
        assert commands[0][2] == .005
        first_sent = diagnostic["sent_sdk_rad"].copy()
        for index in range(200):
            with patch("lerobot_robot_lumos_stararm102.lumos.time.monotonic", return_value=10.02 + index*.01):
                robot.send_action(action)
        assert robot.last_action_diagnostics["sent_sdk_rad"] == bounded
        assert robot.last_action_diagnostics["rate_limited"] == [False]*6
        assert robot.last_action_diagnostics["joint_limit_clipped"] == [True, True, False, False, False, False]
        assert diagnostic["sent_sdk_rad"] == first_sent
        action["joint_1.pos"] = 999
        assert diagnostic["requested_sdk_rad"][0] == 2
        def cleanup_failure(): raise RuntimeError("fixture cleanup failure")
        robot.arm.cleanup = cleanup_failure
        try: robot.disconnect()
        except SDKCleanupError: pass
        else: raise AssertionError("SDK cleanup failure was hidden")
        assert robot.last_action_diagnostics is None
        robot.last_action_diagnostics = {"old_session": True}
        def calibrate_failure(): raise RuntimeError("fixture preflight failure")
        robot.calibrate = calibrate_failure
        try: robot.connect()
        except RuntimeError as exc: assert "fixture preflight failure" in str(exc)
        else: raise AssertionError("Fixture connect failure missing")
        assert robot.last_action_diagnostics is None
elif scenario in ("partial_arm_commit", "arm_write_failure"):
    from stararm_lumos_feedback.runtime import NativeMotionPaused
    with tempfile.TemporaryDirectory() as folder:
        limits = Path(folder) / "limits.json"
        limits.write_text(json.dumps(dict(lower_rad=[-4]*6, upper_rad=[4]*6)))
        config = LumosConfig(gripper=True, execute=True, joint_limits_file=str(limits),
                             max_speed_deg_per_sec=100, max_target_step_deg=2)
        robot = Lumos(config)
        commands, grips = [], []
        robot.arm = types.SimpleNamespace(
            set_joint_raw=lambda q, v: commands.append(q.copy()),
            setGripperPosition=lambda grip: grips.append(grip), cleanup=lambda: None)
        robot._previous_target = [0]*6
        robot._last_send = 0
        old_complete = dict(zip(robot.action_features, [0]*6 + [.2]))
        robot.last_sent_action = old_complete.copy()
        robot.last_action_diagnostics = {"previous_complete": True}
        action = dict(zip(robot.action_features, [math.pi]*6 + [.7]))
        if scenario == "partial_arm_commit":
            guard_calls = []
            def guard():
                guard_calls.append(None)
                if len(guard_calls) == 2: raise NativeMotionPaused()
            robot.before_control_write = guard
            with patch("lerobot_robot_lumos_stararm102.lumos.time.monotonic", return_value=.02):
                try: robot.send_action(action)
                except NativeMotionPaused: pass
                else: raise AssertionError("Gripper-boundary pause was not propagated")
            assert len(commands) == 1 and not grips
            assert all(math.isclose(q, math.radians(2), abs_tol=1e-12) for q in commands[0])
            assert robot._previous_target == commands[0] and robot._last_send == .02
            assert robot.last_action_diagnostics["sent_sdk_rad"] == commands[0]
            assert robot.last_sent_action == old_complete
            robot.before_control_write = None
            reverse = dict(zip(robot.action_features, [-math.pi]*6 + [.4]))
            with patch("lerobot_robot_lumos_stararm102.lumos.time.monotonic", return_value=.04):
                result = robot.send_action(reverse)
            assert len(commands) == 2 and grips == [.4]
            assert max(abs(q) for q in commands[1]) < 1e-12
            assert max(abs(a-b) for a, b in zip(commands[1], commands[0])) <= math.radians(2) + 1e-12
            assert robot.last_sent_action == result
            assert robot.last_action_diagnostics["sent_sdk_rad"] == commands[1]
        else:
            robot.arm.set_joint_raw = lambda q, v: False
            with patch("lerobot_robot_lumos_stararm102.lumos.time.monotonic", return_value=.02):
                try: robot.send_action(action)
                except RuntimeError as exc: assert "SDK rejected joint command" in str(exc)
                else: raise AssertionError("Rejected arm write was not propagated")
            assert robot._previous_target == [0]*6 and robot._last_send == 0
            assert robot.last_action_diagnostics == {"previous_complete": True}
            assert robot.last_sent_action == old_complete and not grips
            def fail_joint(q, v): raise RuntimeError("fixture arm send failed")
            robot.arm.set_joint_raw = fail_joint
            with patch("lerobot_robot_lumos_stararm102.lumos.time.monotonic", return_value=.02):
                try: robot.send_action(action)
                except RuntimeError as exc: assert "fixture arm send failed" in str(exc)
                else: raise AssertionError("Failed arm write was not propagated")
            assert robot._previous_target == [0]*6 and robot._last_send == 0
            assert robot.last_action_diagnostics == {"previous_complete": True}
            assert robot.last_sent_action == old_complete and not grips
        robot.disconnect()
else:
    raise AssertionError("Unknown scenario: " + scenario)
"""


class JointFilterConfigurationTests(unittest.TestCase):
    def test_default_parameters_retain_existing_response(self):
        default = LEGACY.JointFilter()
        explicit = LEGACY.JointFilter(min_cutoff_hz=5, beta=.15)
        for index in range(100):
            values = [float(index)] * 6
            self.assertEqual(default.update(values, .01), explicit.update(values, .01))

    def test_cutoff_changes_settling_without_an_initial_pose_transient(self):
        slow = LEGACY.JointFilter(min_cutoff_hz=.5, beta=0)
        responsive = LEGACY.JointFilter(min_cutoff_hz=5, beta=0)
        initial = [30.0] * 6
        self.assertEqual(slow.update(initial, .01), initial)
        self.assertEqual(responsive.update(initial, .01), initial)
        for _ in range(20):
            slow_result = slow.update([40.0] * 6, .01)
            responsive_result = responsive.update([40.0] * 6, .01)
        self.assertLess(40 - responsive_result[0], .1)
        self.assertGreater(40 - slow_result[0], 5)

    def test_beta_reduces_ramp_lag_and_axes_remain_independent(self):
        fixed = LEGACY.JointFilter(min_cutoff_hz=5, beta=0)
        adaptive = LEGACY.JointFilter(min_cutoff_hz=5, beta=.15)
        for index in range(101):
            values = [float(index), 10.0, -20.0, 0.0, 30.0, 40.0]
            fixed_result = fixed.update(values, .01)
            adaptive_result = adaptive.update(values, .01)
        self.assertLess(100 - adaptive_result[0], 100 - fixed_result[0])
        self.assertLess(100 - adaptive_result[0], 1)
        self.assertEqual(adaptive_result[1:], [10, -20, 0, 30, 40])


class LeaderFilterAdapterTests(unittest.TestCase):
    def scenario(self, scenario):
        env = os.environ.copy()
        env["PYTHONPATH"] = os.pathsep.join((str(SOURCE.parent), env.get("PYTHONPATH", "")))
        result = subprocess.run(
            [sys.executable, "-B", "-c", ADAPTER_SCENARIO, scenario],
            env=env, text=True, capture_output=True, timeout=15,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_omitted_config_preserves_existing_filter_defaults(self):
        self.scenario("defaults")

    def test_invalid_filter_config_is_rejected_before_device_access(self):
        self.scenario("invalid")

    def test_mapping_uses_configured_filter_and_reconnect_resets_its_state(self):
        self.scenario("reconnect")

    def test_leader_diagnostics_explain_filtering_and_persistent_calibrated_range_clipping(self):
        self.scenario("leader_diagnostics")

    def test_robot_diagnostics_distinguish_requested_bounded_and_sent_targets(self):
        self.scenario("robot_diagnostics")

    def test_joint_write_is_committed_before_gripper_guard_pauses_and_reverse_is_bounded(self):
        self.scenario("partial_arm_commit")

    def test_rejected_or_failed_joint_write_cannot_commit_target_or_diagnostics(self):
        self.scenario("arm_write_failure")


if __name__ == "__main__":
    unittest.main()
