"""Stationary observations and runtime arbitration with synthetic frames only."""

import json
import math
import unittest
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import Mock, patch

from test_feedback_runtime import FakeRobot, FixtureCase, LiveMonitor

from stararm_lumos_feedback.baseline import BaselineEstimator
from stararm_lumos_feedback.control import Caps
from stararm_lumos_feedback.runtime import ACTION_KEYS, Session
from stararm_lumos_feedback.transport import Sample


def observation_profile():
    return SimpleNamespace(
        session=dict(fps=20),
        axes=[
            dict(
                p_max=12.5,
                v_max=10 if i < 3 else 30,
                t_max=28 if i < 3 else 10,
                sdk_scale=1,
                urdf_scale=1,
                torque_scale=1,
                max_age_s=0.15,
            )
            for i in range(7)
        ],
        to_model=lambda i, q: q,
        mass_confirmed=False,
    )


def samples(sequence, *, stamp=None, pose=0, velocity=0, motor_velocity=0):
    stamp = sequence * 0.05 if stamp is None else stamp
    return tuple(
        Sample(
            pose if i < 6 else 0.5,
            velocity,
            0,
            30,
            stamp,
            sequence,
            -0.7014613048553252 if i == 6 else None,
            motor_velocity if i == 6 else None,
        )
        for i in range(7)
    )


class BaselineTests(unittest.TestCase):
    def estimator(self, **kwargs):
        return BaselineEstimator(observation_profile(), **kwargs)

    def stationary_window(self, estimator, *, start=1, count=41, pose=0, residual=-3):
        for seq in range(start, start + count):
            estimator.update(samples(seq, pose=pose), [residual] * 7)

    def test_requires_real_two_second_stationary_window_without_removing_mean(self):
        estimator = self.estimator()
        self.stationary_window(estimator, count=40)
        self.assertFalse(estimator.snapshot()["ready"])
        estimator.update(samples(41), [-3] * 7)
        summary = estimator.snapshot()
        self.assertTrue(summary["ready"])
        self.assertEqual(summary["sample_count"], 41)
        self.assertEqual(summary["absolute_residual_max_nm"], [3] * 7)
        self.assertEqual(summary["absolute_residual_p99_nm"], [3] * 7)
        self.assertGreater(summary["recommended_deadzone_nm"][0], 3)
        self.assertFalse(summary["unloaded_verified"])
        self.assertFalse(summary["force_authorized"])
        self.assertFalse(summary["mass_coverage_verified_by_estimator"])
        self.assertEqual(summary["label"], "stationary_not_unloaded_verified")
        json.dumps(summary, allow_nan=False)

    def test_quantization_half_lsb_zero_velocity_does_not_look_like_motion(self):
        estimator = self.estimator()
        for seq in range(1, 42):
            frame = tuple(
                replace(s, velocity=a["v_max"] / 4095)
                for s, a in zip(samples(seq, motor_velocity=30 / 4095), estimator.profile.axes)
            )
            estimator.update(frame, [0] * 7)
        self.assertTrue(estimator.snapshot()["ready"])

    def test_duplicate_partial_new_and_reordered_frames_break_continuity(self):
        for kind in ("duplicate", "partial", "reordered"):
            estimator = self.estimator()
            self.stationary_window(estimator, count=21)
            frame = samples(21)
            if kind == "partial":
                frame = (frame[0], *samples(22)[1:])
            elif kind == "reordered":
                frame = samples(20, stamp=2)
            self.assertFalse(estimator.update(frame, [0] * 7))
            self.stationary_window(estimator, start=22, count=21)
            with self.subTest(kind=kind):
                self.assertFalse(estimator.snapshot()["ready"])
                self.assertLessEqual(estimator.snapshot()["continuous_stationary_s"], 1.01)

    def test_long_gap_or_leader_gap_never_credits_unobserved_time(self):
        estimator = self.estimator()
        self.stationary_window(estimator, count=21)
        estimator.update(samples(22, stamp=100), [0] * 7)
        self.assertFalse(estimator.snapshot()["ready"])
        self.assertEqual(estimator.snapshot()["continuous_stationary_s"], 0)
        estimator.gap()
        self.assertEqual(estimator.snapshot()["pending_stationary_sample_count"], 0)

    def test_pose_drift_or_raw_gripper_motion_cannot_be_stationary(self):
        estimator = self.estimator()
        self.stationary_window(estimator, count=21)
        self.assertFalse(estimator.update(samples(22, pose=0.01), [0] * 7))
        self.assertFalse(estimator.snapshot()["ready"])
        for seq in range(23, 65):
            self.assertFalse(estimator.update(samples(seq, motor_velocity=1), [0] * 7))
        self.assertFalse(estimator.snapshot()["ready"])

    def test_absolute_statistics_and_pose_coverage_remain_observations(self):
        estimator = self.estimator()
        self.stationary_window(estimator, residual=-2)
        estimator.gap()
        self.stationary_window(estimator, start=42, pose=0.5, residual=4)
        summary = estimator.snapshot()
        self.assertEqual(summary["absolute_residual_max_nm"], [4] * 7)
        self.assertEqual(summary["absolute_residual_p95_nm"], [4] * 7)
        self.assertEqual(summary["pose_coverage"]["arm_pose_bins"], 2)
        self.assertEqual(summary["pose_coverage"]["coordinate_min"][:6], [0] * 6)
        self.assertEqual(summary["pose_coverage"]["coordinate_max"][:6], [0.5] * 6)
        self.assertFalse(summary["pose_coverage"]["coverage_accepted"])
        self.assertFalse(summary["policy_changed"])

    def test_storage_is_bounded_and_invalid_inputs_are_rejected(self):
        estimator = self.estimator(max_samples=41)
        self.stationary_window(estimator, count=200)
        self.assertEqual(estimator.snapshot()["sample_count"], 41)
        with self.assertRaises(ValueError):
            self.estimator(minimum_stationary_s=1)
        with self.assertRaises(ValueError):
            self.estimator(max_samples=40)
        with self.assertRaises(ValueError):
            estimator.update(samples(201), [math.nan] * 7)

    def test_fast_input_cannot_call_a_short_retained_buffer_two_seconds_of_data(self):
        estimator = self.estimator(max_samples=41)
        for seq in range(1, 2102):
            estimator.update(samples(seq, stamp=seq * 0.001), [0] * 7)
        summary = estimator.snapshot()
        self.assertTrue(summary["current_window_qualified"])
        self.assertLess(summary["retained_stationary_s"], 0.1)
        self.assertFalse(summary["ready"])


class SessionCommissioningTests(FixtureCase):
    def session(self, *, engineering=False, mode="monitor", hold=False):
        self.profile.engineering = engineering
        robot = FakeRobot()
        leader = SimpleNamespace(get_action=lambda: dict(zip(ACTION_KEYS, [0] * 7)))
        session = Session(self.profile, robot, leader, Caps(), mode=mode, monitor=LiveMonitor(self.profile))
        session.controller = SimpleNamespace(
            hold=hold,
            telemetry={},
            update=lambda *args: False,
            outputs=lambda manual: ([0] * 7, [hold or manual] * 7),
        )
        session.reader = SimpleNamespace(manual_hold=False, latest={})
        session.transport = SimpleNamespace(output=lambda *args: None, ids=range(7))
        session.check_sdk = lambda sample: None
        return session, robot

    def test_engineering_monitor_hold_is_candidate_but_hd_still_freezes(self):
        session, robot = self.session(engineering=True, hold=True)
        session.step()
        self.assertEqual(len(robot.sent), 1)
        session.reader.manual_hold = True
        session.step()
        self.assertEqual(len(robot.sent), 1)
        self.assertFalse(session.automatic_hold_accepted)

    def test_reviewed_v2_monitor_keeps_existing_automatic_hold_semantics(self):
        session, robot = self.session(hold=True)
        session.step()
        self.assertEqual(robot.sent, [])
        self.assertTrue(session.automatic_hold_accepted)

    def test_engineering_profile_cannot_bypass_cli_to_enable_run(self):
        with self.assertRaisesRegex(ValueError, "cannot enable Leader"):
            self.session(engineering=True, mode="run")

    def test_first_closing_command_holds_real_opening_and_subsequent_steps_are_limited(self):
        for mode in ("follow", "monitor", "run"):
            session, robot = self.session(mode=mode)
            clock = [100.0]
            with patch("stararm_lumos_feedback.runtime.time.monotonic", side_effect=lambda: clock[0]):
                session.step()
                self.assertEqual(robot.sent[-1]["gripper.pos"], 0.5)
                clock[0] += 0.01
                session.step()
                self.assertAlmostEqual(robot.sent[-1]["gripper.pos"], 0.495)
                clock[0] += 1000  # A scheduling delay never earns a large movement.
                session.step()
                self.assertAlmostEqual(robot.sent[-1]["gripper.pos"], 0.475)

    def test_initial_opening_and_normalized_range_are_respected(self):
        session, _ = self.session()
        action = dict(zip(ACTION_KEYS, [0] * 6 + [4]))
        frame = tuple(replace(s, q=0.1) if i == 6 else s for i, s in enumerate(samples(1)))
        with patch("stararm_lumos_feedback.runtime.time.monotonic", return_value=10):
            first = session.limited_gripper_action(action, frame)
        self.assertEqual(first["gripper.pos"], 0.1)
        with patch("stararm_lumos_feedback.runtime.time.monotonic", return_value=11):
            second = session.limited_gripper_action(action, frame)
        self.assertAlmostEqual(second["gripper.pos"], 0.12)

    def test_baseline_hook_and_published_candidate_have_no_force_authority(self):
        session, _ = self.session(engineering=True, hold=True)
        session.controller.telemetry = dict(residual_nm=[3] * 7)
        session.controller.update = lambda *args: True
        session.baseline.update = Mock()
        session.step()
        session.baseline.update.assert_called_once()
        published = {}
        session.dashboard = SimpleNamespace(publish=lambda **values: published.update(values))
        session.publish("active")
        self.assertFalse(published["automatic_hold"])
        self.assertTrue(published["automatic_hold_candidate"])
        self.assertFalse(published["baseline"]["force_authorized"])
        self.assertEqual(published["gripper_target_limits"]["max_opening_per_s"], 0.5)


if __name__ == "__main__":
    unittest.main()
