"""Alternative startup selection is explicit, scoped, and never enables by GET."""

import http.client
import json
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import test_feedback_web_motion as web_motion_tests


class CurrentPoseWebTests(unittest.TestCase):
    setUp = web_motion_tests.WebMotionTests.setUp

    def post(self, body):
        client = http.client.HTTPConnection("127.0.0.1", self.dashboard.server.server_port, timeout=2)
        try:
            client.request("POST", "/api/connection", json.dumps(body), {
                "Content-Type": "application/json", "X-Feedback-Token": self.dashboard.token,
            })
            response = client.getresponse()
            return response.status, json.loads(response.read())
        finally:
            client.close()

    def test_current_selection_rejects_other_modes_or_hidden_configuration(self):
        with patch("stararm_lumos_feedback.motion.create_session") as create:
            for body in (
                dict(action="start", mode="inspect", startup_pose="current_aligned"),
                dict(action="start", mode="run", startup_pose="current_aligned"),
                dict(action="stop", startup_pose="current_aligned"),
                dict(action="start", mode="inspect", startup_pose="leader_target"),
                dict(action="start", mode="run", startup_pose="leader_target"),
                dict(action="stop", startup_pose="leader_target"),
                dict(action="start", mode="follow", startup_pose=True),
                dict(action="start", mode="follow", startup_pose={}),
                dict(action="start", mode="follow", startup_pose="current_aligned", tolerance=10),
            ):
                self.assertEqual(self.post(body)[0], 400, body)
            create.assert_not_called()
        self.assertIsNone(self.manager.worker)

    def test_current_start_passes_snapshot_to_the_same_robot_and_keeps_original_files(self):
        sessions = []
        original = self.profile.path.read_bytes(), self.binding.read_bytes()

        class FakeSession:
            def __init__(self, profile, caps, dashboard, cancel, mode, **options):
                self.robot = SimpleNamespace()
                self.options = options
                self.transport = object()
                sessions.append(self)

            def run(self):
                self.before_enable(self.transport)

        with (
            patch("stararm_lumos_feedback.runtime.validate_socketcan"),
            patch("stararm_lumos_feedback.motion.create_session", side_effect=FakeSession),
            patch("stararm_lumos_feedback.motion.preflight_session_pose") as preflight,
        ):
            for strategy in ("current_aligned", "current_relative", "leader_target"):
                for mode in ("follow", "monitor"):
                    with self.subTest(strategy=strategy, mode=mode):
                        code, state = self.post(dict(action="start", mode=mode, startup_pose=strategy))
                        self.assertEqual(code, 200)
                        self.assertEqual(state["startup_pose"], strategy)
                        self.manager.worker.join(2)
                        self.assertFalse(self.manager.worker.is_alive())
                        self.assertEqual(sessions[-1].options["startup_pose"], strategy)
                        self.assertIs(preflight.call_args.kwargs["robot"], sessions[-1].robot)
                        self.assertIs(preflight.call_args.args[2], sessions[-1].transport)
            self.assertEqual(len(sessions), 6)
        self.assertEqual(original, (self.profile.path.read_bytes(), self.binding.read_bytes()))
        self.assertFalse(self.manager.status()["blocked_modes"] == {})

    def test_current_reference_disappears_as_feedback_expires(self):
        now = [10.0]
        self.dashboard.clock = lambda: now[0]
        self.dashboard.publish(status="inspection_active", mode="inspect", q_sdk=[0.0] * 7, age_ms=[0.0] * 7)
        reference = dict(available=True, display_only=True, source="current_follower_pose")
        with patch("stararm_lumos_feedback.startup.current_alignment_reference", return_value=reference) as calculate:
            self.assertEqual(self.dashboard.snapshot()["current_alignment"], reference)
            calculate.assert_called_once()
            now[0] += self.dashboard.freshness_limit_s + 0.01
            stale = self.dashboard.snapshot()
            self.assertFalse(stale["current_alignment"]["available"])
            self.assertEqual(calculate.call_count, 1)
        self.assertIsNone(self.manager.worker)

    def test_formal_profile_does_not_allow_current_strategy(self):
        self.manager.profile.engineering = False
        self.assertEqual(self.manager.status()["startup_pose_options"], ["home"])
        for strategy in ("current_aligned", "current_relative", "leader_target"):
            with self.subTest(strategy=strategy), self.assertRaises(ValueError):
                self.manager.request("start", "follow", startup_pose=strategy)
        self.assertIsNone(self.manager.worker)

    def test_engineering_strategy_options_keep_home_default_and_force_blocked(self):
        with patch("stararm_lumos_feedback.motion.create_session") as create:
            state = self.manager.status()
            self.assertEqual(state["startup_pose"], "home")
            self.assertEqual(state["startup_pose_options"], [
                "home", "current_aligned", "current_relative", "leader_target",
            ])
            self.assertNotIn("run", state["available_modes"])
            self.assertIn("run", state["blocked_modes"])
            create.assert_not_called()
        self.assertIsNone(self.manager.worker)
