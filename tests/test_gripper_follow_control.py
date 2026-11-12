"""Software-only tests of the opt-in TypeLJ gain schedule and SDK boundary."""

import json
import math
import sys
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from lerobot_robot_lumos_stararm102.common import JOINT_KEYS
from lerobot_robot_lumos_stararm102.config import LumosConfig
from lerobot_robot_lumos_stararm102.gripper_control import TypeLJFollowControl, validate_follow_gains
from lerobot_robot_lumos_stararm102.lumos import Lumos


class GripperFollowControlTests(unittest.TestCase):
    def test_legacy_default_and_paired_gain_validation(self):
        validate_follow_gains(None, None)
        for kp, kd in ((None, .1), (8, None), (True, .1), (float("nan"), .1),
                       (8, float("inf")), (0, .1), (20.1, .1), (8, 1.1)):
            with self.subTest(kp=kp, kd=kd), self.assertRaises(ValueError):
                validate_follow_gains(kp, kd)

    def test_middle_opening_uses_distance_metres_and_requested_gains(self):
        control = TypeLJFollowControl(8, .1, 2, .1)
        self.assertEqual(control.command(.5), (.04, 8, .1))
        self.assertEqual(control.command(.3), (.024, 8, .1))

    def test_endpoints_keep_sdk_defaults_and_transition_smoothly(self):
        control = TypeLJFollowControl(8, .3, 2, .1)
        self.assertEqual(control.command(0), (0, 2, .1))
        self.assertEqual(control.command(1), (.08, 2, .1))
        self.assertAlmostEqual(control.command(.05)[1], 5)
        self.assertAlmostEqual(control.command(.05)[2], .2)
        for i in range(1001):
            opening = i / 1000
            distance, kp, kd = control.command(opening)
            self.assertAlmostEqual(distance, .08 * opening)
            self.assertAlmostEqual(kp, control.command(1 - opening)[1])
            self.assertGreaterEqual(kp, 2)
            self.assertLessEqual(kp, 8)
            self.assertGreaterEqual(kd, .1)
            self.assertLessEqual(kd, .3)
        eps = 1e-7
        self.assertLess(abs(control.command(eps)[1] - 2) / eps, .001)
        self.assertLess(abs(control.command(.1 - eps)[1] - 8) / eps, .001)

    def test_bad_opening_is_rejected_before_a_command_can_be_built(self):
        control = TypeLJFollowControl(8, .1, 2, .1)
        for opening in (-.01, 1.01, float("nan"), float("inf"), True):
            with self.subTest(opening=opening), self.assertRaises(ValueError):
                control.command(opening)

    def test_yaml_is_read_without_modifying_startup_defaults(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            path = root / "src/config/robot_kinematics.yaml"
            path.parent.mkdir(parents=True)
            source = "gripper_control: {type: TypeLJ, default_kp: 2.0, default_kd: 0.1}\n"
            path.write_text(source)
            control = TypeLJFollowControl.from_sdk(root, 8, .1)
            self.assertEqual(control.command(0), (0, 2, .1))
            self.assertEqual(path.read_text(), source)
            for bad in ("{type: TypeFZ, default_kp: 2, default_kd: 0.1}",
                        "{type: TypeLJ}", "[]"):
                path.write_text("gripper_control: " + bad)
                with self.assertRaises(ValueError):
                    TypeLJFollowControl.from_sdk(root, 8, .1)


class GripperFollowAdapterTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.limits = Path(self.directory.name) / "limits.json"
        self.limits.write_text(json.dumps(dict(lower_rad=[-1] * 6, upper_rad=[1] * 6)))

    def robot(self, *, execute=True, follow=True):
        robot = Lumos(LumosConfig(
            execute=execute, joint_limits_file=str(self.limits),
            gripper_follow_kp=8 if follow else None, gripper_follow_kd=.1 if follow else None,
        ))
        robot.arm = Mock()
        robot.arm.set_joint_raw.return_value = True
        robot.arm.setGripperPosition.return_value = True
        robot.arm.setGripperDistance.return_value = True
        robot._previous_target = [0] * 6
        robot._last_send = time.monotonic() - .01
        if follow:
            robot._gripper_follow_control = TypeLJFollowControl(8, .1, 2, .1)
        return robot

    @staticmethod
    def action(opening):
        return dict(zip(JOINT_KEYS, [0] * 6), **{"gripper.pos": opening})

    def test_opt_in_keeps_normalized_sent_target_but_sends_sdk_metres(self):
        robot = self.robot()
        result = robot.send_action(self.action(.5))
        self.assertEqual(result["gripper.pos"], .5)
        robot.arm.setGripperDistance.assert_called_once_with(.04, 8, .1)
        robot.arm.setGripperPosition.assert_not_called()
        self.assertEqual(robot.last_action_diagnostics["gripper_position_control"],
                         dict(distance_m=.04, kp=8, kd=.1))

    def test_default_adapter_keeps_legacy_position_interface(self):
        robot = self.robot(follow=False)
        robot.send_action(self.action(.5))
        robot.arm.setGripperPosition.assert_called_once_with(.5)
        robot.arm.setGripperDistance.assert_not_called()

    def test_preview_does_not_send_either_gripper_interface(self):
        robot = self.robot(execute=False)
        robot.send_action(self.action(.5))
        robot.arm.set_joint_raw.assert_not_called()
        robot.arm.setGripperPosition.assert_not_called()
        robot.arm.setGripperDistance.assert_not_called()

    def test_guard_before_gripper_still_prevents_its_write(self):
        robot = self.robot()
        robot.before_control_write = Mock(side_effect=[None, RuntimeError("cancelled")])
        with self.assertRaisesRegex(RuntimeError, "cancelled"):
            robot.send_action(self.action(.5))
        robot.arm.setGripperDistance.assert_not_called()
        self.assertIsNone(robot.last_sent_action)

    def test_sdk_rejection_does_not_record_a_complete_follow_send(self):
        robot = self.robot()
        robot.arm.setGripperDistance.return_value = False
        with self.assertRaisesRegex(RuntimeError, "rejected gripper"):
            robot.send_action(self.action(.5))
        self.assertIsNone(robot.last_sent_action)

    def test_opt_in_requires_gripper_and_valid_gains(self):
        for options in (dict(gripper=False, gripper_follow_kp=8, gripper_follow_kd=.1),
                        dict(gripper_follow_kp=math.inf, gripper_follow_kd=.1)):
            with self.subTest(options=options), self.assertRaises(ValueError):
                Lumos(LumosConfig(**options))

    def test_constructor_still_reads_original_yaml_and_follow_gains_start_at_send(self):
        robot = self.robot()
        arm = robot.arm
        robot.arm = None
        robot.mapping = dict(lumos_home_rad=[0] * 6)
        robot.calibrate = Mock()
        sdk_root = Path(self.directory.name) / "startouch_sdk"
        yaml_path = sdk_root / "src/config/robot_kinematics.yaml"
        yaml_path.parent.mkdir(parents=True)
        original = "gripper_control: {type: TypeLJ, default_kp: 2.0, default_kd: 0.1}\n"
        yaml_path.write_text(original)
        robot.config.sdk_root = str(sdk_root)
        arm.get_joint_positions.return_value = [0] * 6

        def construct(**kwargs):
            self.assertEqual(yaml_path.read_text(), original)
            arm.setGripperDistance.assert_not_called()
            return arm

        with patch.dict(sys.modules, {"startouchclass": SimpleNamespace(SingleArm=construct)}), \
                patch("lerobot_robot_lumos_stararm102.lumos.time.sleep"):
            robot.connect()
        arm.setGripperDistance.assert_not_called()
        robot.send_action(self.action(.5))
        arm.setGripperDistance.assert_called_once_with(.04, 8, .1)
        self.assertEqual(yaml_path.read_text(), original)
