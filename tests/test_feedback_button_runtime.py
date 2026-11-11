"""Button wiring through real serial parsing and isolated adapter construction.

All replies and device objects are synthetic; no hardware port is opened and
no native SDK controller is constructed.
"""

import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from test_feedback_runtime import UART, FakeLeader, FakeRobot, LiveMonitor, configuration

from stararm_lumos_feedback.control import Caps
from stararm_lumos_feedback.runtime import LeaderReader, Session
from stararm_lumos_feedback.transport import Fashionstar, LeaderGap


class ButtonReaderTests(unittest.TestCase):
    def setUp(self):
        self.uart = UART()
        self.leader = FakeLeader(self.uart)
        self.leader.config.button_enabled = True
        self.leader.config.button_id = 7
        self.status = Mock()
        self.transport = Fashionstar(self.uart, range(7), enabled=False, timeout=0.003)
        self.reader = LeaderReader(self.leader, self.transport, button_status=self.status)
        self.original_query = self.leader.leader.send_sync_servo_monitor
        self.original_stop = self.leader.leader.stop_on_control_mode
        self.reader.install()
        self.addCleanup(self.reader.restore)

    def test_genuine_button_monitor_reply_reports_verified_and_updates_angle(self):
        self.uart.angles[7] = 180
        self.leader.leader.send_sync_servo_monitor(list(range(8)))
        self.status.assert_called_once_with(True)
        self.assertEqual(self.leader.leader.servos[7].angle_monitor, 180)
        self.assertEqual(self.reader.latest[7]["angle"], 180)
        self.assertTrue(all(frame[2] == 25 for frame in self.uart.writes))

    def test_missing_button_reply_reports_false_and_recovery_requires_new_reply(self):
        self.uart.missing = {7}
        with self.assertRaises(LeaderGap) as caught:
            self.leader.leader.send_sync_servo_monitor(list(range(8)))
        self.assertEqual(caught.exception.missing_ids, (7,))
        self.status.assert_called_once_with(False)
        self.assertEqual(self.reader.latest, {})
        self.uart.missing.clear()
        self.leader.leader.send_sync_servo_monitor(list(range(8)))
        self.assertEqual([call.args for call in self.status.call_args_list], [(False,), (True,)])

    def test_joint_only_gap_does_not_report_button_missing_or_accept_partial_cycle(self):
        self.leader.leader.send_sync_servo_monitor(list(range(8)))
        self.status.reset_mock()
        previous = self.reader.latest
        self.uart.missing = {2}
        with self.assertRaises(LeaderGap) as caught:
            self.leader.leader.send_sync_servo_monitor(list(range(8)))
        self.assertEqual(caught.exception.missing_ids, (2,))
        self.status.assert_not_called()
        self.assertIs(self.reader.latest, previous)

    def test_absent_board_is_not_queried_and_cannot_be_reported_verified(self):
        self.leader.config.button_enabled = False
        self.uart.missing = {7}
        self.leader.leader.send_sync_servo_monitor(list(range(7)))
        self.status.assert_not_called()
        self.assertEqual(set(self.reader.latest), set(range(7)))
        for frame in self.uart.writes:
            self.assertEqual(frame[2], 25)
            self.assertEqual(list(frame[7:-1]), list(range(7)))

    def test_hold_request_is_arbitrated_without_direct_serial_control_writes(self):
        self.leader.leader.stop_on_control_mode(0, 0x11, 0)
        self.assertTrue(self.reader.manual_hold)
        self.leader.leader.stop_on_control_mode(0, 0x10, 0)
        self.assertFalse(self.reader.manual_hold)
        self.assertEqual(self.uart.writes, [])
        self.reader.restore()
        self.assertIs(self.leader.leader.send_sync_servo_monitor, self.original_query)
        self.assertIs(self.leader.leader.stop_on_control_mode, self.original_stop)


class ButtonConnectionStatusTests(unittest.TestCase):
    def setUp(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        self.profile = configuration(Path(folder.name))

    def session(self, enabled):
        uart = UART()
        leader = FakeLeader(uart)
        leader.config.button_enabled = enabled
        leader.config.button_id = 7
        dashboard = SimpleNamespace(button_status=Mock(), publish=Mock())
        robot = FakeRobot()
        session = Session(
            self.profile, robot, leader, Caps(), mode="follow", dashboard=dashboard,
            monitor=LiveMonitor(self.profile),
        )
        # Exercise the real Session startup and cleanup, then cancel before the
        # SDK connection boundary. No controller/device construction is needed.
        session.before_enable = lambda transport: session.stop.set()
        return session, leader, robot, dashboard

    def test_successful_enabled_button_ping_is_published_after_connect(self):
        session, leader, robot, dashboard = self.session(True)
        order = []
        dashboard.button_status.side_effect = lambda value: order.append(("verified", value))
        with (
            patch.object(leader, "connect", side_effect=lambda: order.append(("connect", True))),
            patch.object(robot, "connect") as sdk_connect,
        ):
            session.run(0.02)
        self.assertEqual(order, [("connect", True), ("verified", True)])
        dashboard.button_status.assert_called_once_with(True)
        sdk_connect.assert_not_called()
        self.assertFalse(leader.uart.is_open)

    def test_disabled_button_does_not_publish_a_successful_board_detection(self):
        session, leader, robot, dashboard = self.session(False)
        with patch.object(robot, "connect") as sdk_connect:
            session.run(0.02)
        dashboard.button_status.assert_not_called()
        sdk_connect.assert_not_called()
        self.assertFalse(leader.uart.is_open)

    def test_failed_leader_connect_never_reports_a_successful_button_detection(self):
        session, leader, robot, dashboard = self.session(True)
        with (
            patch.object(leader, "connect", side_effect=RuntimeError("synthetic button ping failed")),
            patch.object(robot, "connect") as sdk_connect,
        ):
            with self.assertRaisesRegex(RuntimeError, "synthetic button ping failed"):
                session.run(0.02)
        dashboard.button_status.assert_not_called()
        sdk_connect.assert_not_called()
        self.assertIsNone(session.reader)


ADAPTER_SCENARIO = r'''
from copy import deepcopy
from dataclasses import asdict, dataclass
import json
from pathlib import Path
import sys
import tempfile
import threading
import types
from unittest.mock import patch

# Real configuration decoding and adapter constructors run in an isolated
# subprocess. Stub only LeRobot base/registry classes, never adapter behavior.
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

from test_feedback_runtime import configuration
from lerobot_robot_lumos_stararm102.cli import device_configs, load_session
from lerobot_robot_lumos_stararm102.lumos import Lumos
from lerobot_robot_lumos_stararm102.stararm102 import StarArm102
from stararm_lumos_feedback.config import Profile
from stararm_lumos_feedback.control import Caps
from stararm_lumos_feedback.motion import create_session

scenario = sys.argv[1]
with tempfile.TemporaryDirectory() as folder:
    root = Path(folder)
    configuration(root)
    path = root / "session.json"
    data = json.loads(path.read_text())
    data["teleop"]["button_id"] = 12
    data["teleop"]["button_enabled"] = scenario not in ("true", "default_disabled")
    path.write_text(json.dumps(data))
    profile = Profile(root / "profile.json")
    (root / "binding.json").write_text(json.dumps(profile.binding()))
    original_files = {p: p.read_bytes() for p in root.rglob("*") if p.is_file()}
    original_session = deepcopy(profile.session)
    original_profile = deepcopy(profile.data)
    expected_robot, expected_leader = device_configs(load_session(path))
    override = {"false": False, "true": True, "invalid": 1}.get(scenario)
    expected = expected_leader.button_enabled if override is None else override
    kwargs = {} if scenario.startswith("default_") else {"button_enabled": override}
    if scenario == "wait_until_cancel":
        kwargs["button_wait_until_cancel"] = True
    with (
        patch("stararm_lumos_feedback.config.verify_sdk_imports") as verify,
        patch.object(StarArm102, "connect", side_effect=AssertionError("No serial access allowed")) as leader_connect,
        patch.object(Lumos, "connect", side_effect=AssertionError("No SDK construction allowed")) as robot_connect,
    ):
        if scenario == "invalid":
            try:
                create_session(profile, Caps(), None, threading.Event(), "monitor", **kwargs)
            except ValueError as exc:
                assert "boolean" in str(exc)
            else:
                raise AssertionError("Non-boolean override must fail")
        else:
            cancel = threading.Event()
            session = create_session(profile, Caps(), None, cancel, "monitor", **kwargs)
            assert session.stop is cancel
            assert session.leader.button_wait_until_cancel is (scenario == "wait_until_cancel")
            actual = asdict(session.leader.config)
            expected_config = asdict(expected_leader)
            expected_config["button_enabled"] = expected
            assert actual == expected_config, (actual, expected_config)
            assert actual["button_id"] == 12
            expected_robot.execute = True
            assert asdict(session.robot.config) == asdict(expected_robot)
            assert not session.leader.is_connected and not session.robot.is_connected
        verify.assert_called_once_with(profile)
        leader_connect.assert_not_called()
        robot_connect.assert_not_called()
    assert profile.session == original_session
    assert profile.data == original_profile
    assert {p: p.read_bytes() for p in root.rglob("*") if p.is_file()} == original_files
'''


class ButtonMotionOverrideTests(unittest.TestCase):
    def scenario(self, name):
        root = Path(__file__).resolve().parents[1]
        environment = os.environ.copy()
        environment["PYTHONPATH"] = os.pathsep.join((str(root / "src"), str(root / "tests")))
        result = subprocess.run(
            [sys.executable, "-B", "-c", ADAPTER_SCENARIO, name],
            cwd=root, env=environment, capture_output=True, text=True, timeout=20,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_disabling_overrides_only_the_runtime_button_flag(self):
        self.scenario("false")

    def test_enabling_preserves_the_reviewed_button_id_and_all_other_fields(self):
        self.scenario("true")

    def test_no_override_preserves_enabled_session_choice(self):
        self.scenario("default_enabled")

    def test_no_override_preserves_disabled_session_choice(self):
        self.scenario("default_disabled")

    def test_non_boolean_override_is_rejected_without_writes_or_connection(self):
        self.scenario("invalid")

    def test_explicit_wait_is_runtime_only_and_does_not_modify_bound_files(self):
        self.scenario("wait_until_cancel")

    def test_invalid_wait_option_is_rejected_before_sdk_import(self):
        from stararm_lumos_feedback.motion import create_session

        with patch("stararm_lumos_feedback.config.verify_sdk_imports") as verify:
            for option, cancel in ((1, None), (True, None), (True, SimpleNamespace(is_set=lambda: False))):
                with self.subTest(option=option, cancel=cancel), self.assertRaises(ValueError):
                    create_session(None, Caps(), None, cancel, "monitor", button_wait_until_cancel=option)
            verify.assert_not_called()


if __name__ == "__main__":
    unittest.main()
