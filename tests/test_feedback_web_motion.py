"""Web motion ownership and pre-enable gating, with all hardware replaced."""

import http.client
import json
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from test_feedback_runtime import configuration

from stararm_lumos_feedback.config import Profile
from stararm_lumos_feedback.connection import InspectionConnection
from stararm_lumos_feedback.control import Caps
from stararm_lumos_feedback.dashboard import Dashboard
from stararm_lumos_feedback.motion import check_cancelled
from stararm_lumos_feedback.startup import StartupPoseError


class WebMotionTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        original = configuration(Path(temporary.name))
        data = json.loads(original.path.read_text())
        data["version"] = 3
        data["mass_coverage"] = "UNCONFIRMED"
        data["commissioning"] = dict(
            status="engineering_initial", force_authorized=False,
            mass_coverage_inferred=False, allowed_modes=["inspect", "follow", "monitor"],
        )
        original.path.write_text(json.dumps(data))
        self.profile = Profile(original.path, purpose="inspect")
        self.binding = original.path.parent / "binding.json"
        self.binding.write_text(json.dumps(self.profile.binding()))
        self.dashboard = Dashboard(Caps(), self.profile, port=0)
        self.dashboard.start()
        self.addCleanup(self.dashboard.close)
        self.manager = InspectionConnection(self.profile, self.binding, self.dashboard)
        self.dashboard.connection = self.manager
        self.addCleanup(self.manager.close)

    def test_unreviewed_force_is_rejected_before_any_worker_or_sdk(self):
        self.assertEqual(self.manager.status()["available_modes"], ["inspect", "follow", "monitor"])
        self.assertEqual(self.manager.status()["blocked_modes"], {"run": "force_profile_unreviewed"})
        with patch("stararm_lumos_feedback.motion.create_session") as create:
            with self.assertRaises(ValueError):
                self.manager.request("start", mode="run")
            create.assert_not_called()
        self.assertIsNone(self.manager.worker)

    def test_http_mode_contract_cannot_modify_configuration_or_stop_with_mode(self):
        requests = []

        class Manager:
            def status(self):
                return dict(state="idle")

            def request(self, action, mode="inspect"):
                requests.append((action, mode))
                return dict(state="starting", mode=mode)

        self.dashboard.connection = Manager()

        def request(body):
            client = http.client.HTTPConnection("127.0.0.1", self.dashboard.server.server_port, timeout=2)
            try:
                client.request("POST", "/api/connection", json.dumps(body), {
                    "Content-Type": "application/json", "X-Feedback-Token": self.dashboard.token,
                })
                response = client.getresponse()
                return response.status, json.loads(response.read())
            finally:
                client.close()

        for invalid in (
            {"action": "start", "mode": "calibrate"},
            {"action": "stop", "mode": "follow"},
            {"action": "start", "mode": "follow", "profile": "/untrusted/profile.json"},
            {"action": "start", "mode": "run", "force_authorized": True},
        ):
            self.assertEqual(request(invalid)[0], 400)
        self.assertEqual(requests, [])
        for mode in ("inspect", "follow", "monitor", "run"):
            self.assertEqual(request({"action": "start", "mode": mode})[0], 200)
        self.assertEqual(request({"action": "stop"})[0], 200)
        self.assertEqual(requests, [("start", mode) for mode in ("inspect", "follow", "monitor", "run")] + [("stop", "inspect")])

    def test_ownership_lasts_through_motion_cleanup_and_requires_explicit_restart(self):
        entered, cleanup, finish = threading.Event(), threading.Event(), threading.Event()
        self.addCleanup(finish.set)
        sessions = []

        class FakeSession:
            def __init__(self, profile, caps, dashboard, cancel, mode, **kwargs):
                self.stop, self.dashboard, self.mode = cancel, dashboard, mode
                self.transport = object()
                sessions.append(self)

            def run(self):
                self.before_enable(self.transport)
                check_cancelled(self.stop)
                self.dashboard.publish(
                    status="active", mode=self.mode, q_sdk=[0.0] * 7, age_ms=[0.0] * 7,
                    error=None, requested_mw=[0] * 7, leader_controlled=False,
                )
                entered.set()
                if not self.stop.wait(2):
                    raise AssertionError("Explicit stop was not delivered")
                cleanup.set()
                if not finish.wait(2):
                    raise AssertionError("Cleanup was not released")

        with (
            patch("stararm_lumos_feedback.runtime.validate_socketcan"),
            patch("stararm_lumos_feedback.motion.create_session", side_effect=FakeSession),
            patch("stararm_lumos_feedback.motion.preflight_pose") as probe,
        ):
            self.manager.request("start", mode="monitor")
            self.assertTrue(entered.wait(2))
            self.assertFalse(self.manager.worker.daemon)
            self.assertEqual(self.manager.status()["mode"], "monitor")
            self.assertEqual(self.manager.status()["state"], "active")
            probe.assert_called_once()
            self.assertIs(probe.call_args.kwargs["leader_transport"], sessions[0].transport)
            self.assertTrue(self.dashboard.caps_writable(self.dashboard.snapshot()))
            with self.assertRaises(RuntimeError):
                self.manager.request("start", mode="follow")
            self.manager.request("stop")
            self.assertTrue(cleanup.wait(2))
            self.assertFalse(self.manager.status()["can_start"])
            self.assertEqual(self.manager.status()["state"], "stopping")
            finish.set()
            self.manager.worker.join(2)
            self.assertFalse(self.manager.worker.is_alive())
            self.assertEqual(self.manager.status()["state"], "stopped")
            self.assertTrue(self.manager.status()["can_start"])
            self.assertEqual(len(sessions), 1)

    def test_pose_failure_never_enables_and_keeps_page_available(self):
        enabled = []
        failure = StartupPoseError("J4: 33 deg", diagnostics=dict(strategy="home", axes=[]))
        failure.display_pose = dict(
            q_sdk=[0.2] * 7, q_urdf=[0.2] * 6, captured_monotonic=time.monotonic(),
        )

        class FakeSession:
            def __init__(self, profile, caps, dashboard, cancel, mode, **kwargs):
                self.stop = cancel

            def run(self):
                self.before_enable(object())
                enabled.append(True)

        with (
            patch("stararm_lumos_feedback.runtime.validate_socketcan"),
            patch("stararm_lumos_feedback.motion.create_session", side_effect=FakeSession),
            patch("stararm_lumos_feedback.motion.preflight_pose", side_effect=failure),
        ):
            self.manager.request("start", mode="follow")
            self.manager.worker.join(2)
        self.assertFalse(enabled)
        self.assertEqual(self.manager.status()["error_code"], "startup_pose_mismatch")
        self.assertIn("J4", self.dashboard.snapshot()["error_detail"])
        state = self.dashboard.snapshot()
        self.assertEqual(state["last_pose"]["q_sdk"], [0.2] * 7)
        self.assertTrue(state["last_pose"]["display_only"])
        self.assertFalse(state["telemetry_fresh"])
        self.assertNotIn("q_sdk", state)
        self.assertEqual(state["error_diagnostics"], failure.diagnostics)
        self.assertEqual(self.manager.status()["error_diagnostics"], failure.diagnostics)
        self.assertEqual(self.manager.status()["state"], "error")
        self.assertTrue(self.manager.status()["can_start"])
        self.assertTrue(self.dashboard.thread.is_alive())

    def test_uncertain_sdk_cleanup_blocks_further_motion_but_keeps_inspection(self):
        class CleanupError(RuntimeError):
            code = "sdk_cleanup_failed"

        class FakeSession:
            def run(self):
                raise CleanupError("SDK cleanup failed; actuator state is unconfirmed")

        with (
            patch("stararm_lumos_feedback.runtime.validate_socketcan"),
            patch("stararm_lumos_feedback.motion.create_session", return_value=FakeSession()) as create,
        ):
            self.manager.request("start", mode="follow")
            self.manager.worker.join(2)
            self.assertFalse(self.manager.worker.is_alive())
            status = self.manager.status()
            self.assertEqual(status["error_code"], "sdk_cleanup_failed")
            self.assertEqual(status["available_modes"], ["inspect"])
            self.assertEqual(status["blocked_modes"], {
                mode: "sdk_cleanup_failed" for mode in ("follow", "monitor", "run")
            })
            for mode in ("follow", "monitor", "run"):
                with self.assertRaisesRegex(RuntimeError, "actuator state is unconfirmed"):
                    self.manager.request("start", mode=mode)
            self.assertEqual(create.call_count, 1)
            self.assertTrue(status["can_start"])
            self.assertTrue(self.dashboard.thread.is_alive())

    def test_stop_during_startup_cannot_enable_after_late_probe_returns(self):
        entered, finish = threading.Event(), threading.Event()
        self.addCleanup(finish.set)
        enabled = []

        def probe(*args, **kwargs):
            entered.set()
            if not finish.wait(2):
                raise AssertionError("Probe was not released")

        class FakeSession:
            def __init__(self, profile, caps, dashboard, cancel, mode, **kwargs):
                self.stop = cancel

            def run(self):
                self.before_enable(object())
                check_cancelled(self.stop)
                enabled.append(True)

        with (
            patch("stararm_lumos_feedback.runtime.validate_socketcan"),
            patch("stararm_lumos_feedback.motion.create_session", side_effect=FakeSession),
            patch("stararm_lumos_feedback.motion.preflight_pose", side_effect=probe),
        ):
            self.manager.request("start", mode="follow")
            self.assertTrue(entered.wait(2))
            self.manager.request("stop")
            self.assertFalse(self.manager.status()["can_start"])
            finish.set()
            self.manager.worker.join(2)
        self.assertFalse(enabled)
        self.assertIsNone(self.manager.status()["error"])
        self.assertEqual(self.manager.status()["state"], "stopped")


if __name__ == "__main__":
    unittest.main()
