"""Button-board selection through real HTTP/session ownership, with fake hardware."""

import http.client
import json
import tempfile
import threading
import unittest
from contextlib import nullcontext
from pathlib import Path
from unittest.mock import patch

from test_feedback_runtime import configuration

from stararm_lumos_feedback.config import Profile
from stararm_lumos_feedback.connection import InspectionConnection
from stararm_lumos_feedback.control import Caps
from stararm_lumos_feedback.dashboard import Dashboard


class ButtonConnectionTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        original = configuration(self.root)
        self.profile = Profile(original.path, purpose="inspect")
        self.binding = self.root / "binding.json"
        self.binding.write_text(json.dumps(self.profile.binding()))
        self.settings = self.root / "local-settings/button-board.json"
        self.dashboard = Dashboard(Caps(), self.profile, port=0)
        self.dashboard.start()
        self.addCleanup(self.dashboard.close)
        self.manager = InspectionConnection(
            self.profile, self.binding, self.dashboard, button_settings=self.settings
        )
        self.dashboard.connection = self.manager
        self.addCleanup(self.manager.close)
        protected = (
            self.profile.path,
            self.binding,
            self.profile.paths["session_file"],
            Path(self.profile.session["robot"]["mapping_file"]),
            Path(self.profile.session["robot"]["joint_limits_file"]),
        )
        self.original_bytes = {path: path.read_bytes() for path in protected}

    def assert_bound_files_unchanged(self):
        for path, expected in self.original_bytes.items():
            self.assertEqual(path.read_bytes(), expected, str(path))

    def post(self, value, *, token=True, origin=None, host=None, raw=False):
        connection = http.client.HTTPConnection("127.0.0.1", self.dashboard.server.server_port, timeout=2)
        headers = {"Content-Type": "application/json"}
        if token is True:
            headers["X-Feedback-Token"] = self.dashboard.token
        elif token is not None:
            headers["X-Feedback-Token"] = token
        if origin is not None:
            headers["Origin"] = origin
        if host is not None:
            headers["Host"] = host
        try:
            connection.request(
                "POST", "/api/button-board", value if raw else json.dumps(value), headers
            )
            response = connection.getresponse()
            return response.status, json.loads(response.read())
        finally:
            connection.close()

    def finish_worker(self):
        self.manager.worker.join(2)
        self.assertFalse(self.manager.worker.is_alive(), "Fake hardware worker failed to finish")

    def test_idle_choice_returns_full_status_persists_only_enabled_and_preserves_bound_inputs(self):
        initial = self.manager.status()
        self.assertEqual(initial["button_board"], dict(enabled=True, id=7, verified=None, can_change=True))
        self.assertFalse(self.settings.exists())
        result = self.manager.set_button_board(False)
        self.assertEqual(result, self.manager.status())
        self.assertEqual(result["state"], "idle")
        self.assertIn("available_modes", result)
        self.assertEqual(result["button_board"], dict(enabled=False, id=7, verified=None, can_change=True))
        self.assertEqual(json.loads(self.settings.read_text()), {"enabled": False})
        self.assertEqual(self.settings.stat().st_mode & 0o777, 0o600)
        restarted = InspectionConnection(
            self.profile, self.binding, self.dashboard, button_settings=self.settings
        )
        self.addCleanup(restarted.close)
        self.assertEqual(restarted.status()["button_board"], result["button_board"])
        self.assert_bound_files_unchanged()

    def test_http_accepts_only_enabled_boolean_and_returns_full_connection_status(self):
        for value in (
            {}, {"id": 7}, {"enabled": 0}, {"enabled": 1}, {"enabled": "false"},
            {"enabled": None}, {"enabled": False, "id": 7}, {"enabled": True, "verified": True},
            {"enabled": False, "can_change": True}, False, [], "false",
        ):
            with self.subTest(value=value):
                self.assertEqual(self.post(value)[0], 400)
                self.assertTrue(self.manager.status()["button_board"]["enabled"])
                self.assertFalse(self.settings.exists())
        for text in ("", "{", '{"enabled": NaN}', " " * 4097):
            with self.subTest(text=text[:30]):
                self.assertEqual(self.post(text, raw=True)[0], 400)
        origin = f"http://127.0.0.1:{self.dashboard.server.server_port}"
        code, result = self.post({"enabled": False}, origin=origin)
        self.assertEqual(code, 200)
        self.assertEqual(result, self.manager.status())
        self.assertFalse(result["button_board"]["enabled"])
        self.assert_bound_files_unchanged()

    def test_http_rejects_missing_wrong_token_foreign_origin_or_host_before_mutation(self):
        for options in (
            {"token": None}, {"token": "wrong-token"},
            {"origin": "https://other.example"},
            {"origin": f"http://localhost:{self.dashboard.server.server_port}"},
            {"host": "other.example"},
        ):
            with self.subTest(options=options):
                self.assertEqual(self.post({"enabled": False}, **options)[0], 403)
                self.assertTrue(self.manager.status()["button_board"]["enabled"])
                self.assertFalse(self.settings.exists())
        self.dashboard.connection = None
        self.assertEqual(self.post({"enabled": False})[0], 409)
        self.assert_bound_files_unchanged()

    def test_failed_atomic_save_keeps_selection_verification_and_settings_bytes(self):
        self.manager.set_button_board(False)
        self.manager.button_board.record(True)
        before = self.manager.status()
        settings_before = self.settings.read_bytes()
        with patch("stararm_lumos_feedback.button_board.os.replace", side_effect=OSError("disk full")):
            with self.assertRaisesRegex(OSError, "disk full"):
                self.manager.set_button_board(True)
            self.assertEqual(self.post({"enabled": True})[0], 400)
        self.assertEqual(self.manager.status(), before)
        self.assertEqual(self.settings.read_bytes(), settings_before)
        self.assertEqual(list(self.settings.parent.iterdir()), [self.settings])
        self.assert_bound_files_unchanged()

    def test_real_worker_prevents_changes_during_start_active_stop_and_after_close(self):
        validating, proceed = threading.Event(), threading.Event()
        active, cleaning, finish = threading.Event(), threading.Event(), threading.Event()
        self.addCleanup(proceed.set)
        self.addCleanup(finish.set)

        def validate(*args):
            validating.set()
            if not proceed.wait(2):
                raise AssertionError("Validation was not released")

        class FakeSession:
            def __init__(self, profile, caps, publication, cancel, mode, **kwargs):
                self.publication, self.cancel = publication, cancel

            def run(self):
                self.publication.publish(status="active", mode="follow")
                active.set()
                if not self.cancel.wait(2):
                    raise AssertionError("Explicit stop was not delivered")
                cleaning.set()
                if not finish.wait(2):
                    raise AssertionError("Cleanup was not released")

        def assert_locked(expected_state):
            status = self.manager.status()
            self.assertEqual(status["state"], expected_state)
            self.assertFalse(status["button_board"]["can_change"])
            self.assertTrue(self.manager.worker.is_alive())
            with self.assertRaisesRegex(RuntimeError, "Stop the hardware session"):
                self.manager.set_button_board(False)
            self.assertEqual(self.post({"enabled": False})[0], 409)
            self.assertFalse(self.settings.exists())

        with (
            patch("stararm_lumos_feedback.runtime.session_lock", return_value=nullcontext()),
            patch("stararm_lumos_feedback.runtime.validate_socketcan", side_effect=validate),
            patch("stararm_lumos_feedback.motion.create_session", side_effect=FakeSession),
        ):
            self.manager.request("start", mode="follow")
            self.assertTrue(validating.wait(2))
            assert_locked("starting")
            proceed.set()
            self.assertTrue(active.wait(2))
            assert_locked("active")
            self.manager.request("stop")
            self.assertTrue(cleaning.wait(2))
            assert_locked("stopping")
            finish.set()
            self.finish_worker()
        self.assertTrue(self.manager.status()["button_board"]["can_change"])
        self.manager.close()
        self.assertFalse(self.manager.status()["button_board"]["can_change"])
        with self.assertRaises(RuntimeError):
            self.manager.set_button_board(False)
        self.assert_bound_files_unchanged()

    def test_live_thread_locks_choice_even_if_published_state_is_idle(self):
        stop = threading.Event()
        self.addCleanup(stop.set)
        worker = threading.Thread(target=lambda: stop.wait(2), daemon=True)
        self.manager.worker = worker
        worker.start()
        self.assertEqual(self.manager.status()["state"], "idle")
        self.assertFalse(self.manager.status()["button_board"]["can_change"])
        with self.assertRaises(RuntimeError):
            self.manager.set_button_board(False)
        stop.set()
        worker.join(2)
        self.assertTrue(self.manager.status()["button_board"]["can_change"])
        self.assert_bound_files_unchanged()

    def test_every_inspection_start_verifies_binding_and_never_verifies_the_leader(self):
        class Observer:
            def __init__(self, *args, **kwargs):
                pass

            def run(self):
                pass

        verify = Profile.verify
        with (
            patch.object(Profile, "verify", autospec=True, side_effect=verify) as checked,
            patch("stararm_lumos_feedback.runtime.session_lock", return_value=nullcontext()),
            patch("stararm_lumos_feedback.runtime.validate_socketcan"),
            patch("stararm_lumos_feedback.inspection.Inspection", Observer),
            patch("stararm_lumos_feedback.motion.create_session") as session,
            patch("stararm_lumos_feedback.motion.preflight_session_pose") as pose,
        ):
            for enabled in (True, False):
                self.manager.set_button_board(enabled)
                self.manager.button_board.record(True)
                self.manager.request("start", mode="inspect")
                self.finish_worker()
                board = self.manager.status()["button_board"]
                self.assertIs(board["enabled"], enabled)
                self.assertIsNone(board["verified"])
            self.assertEqual(checked.call_count, 2)
            for call in checked.call_args_list:
                self.assertEqual(call.args[1], self.binding)
                self.assertEqual(call.args[0].purpose, "inspect")
            session.assert_not_called()
            pose.assert_not_called()
        self.assert_bound_files_unchanged()

    def test_changed_binding_still_blocks_next_start_after_setting_override(self):
        self.manager.set_button_board(False)
        value = json.loads(self.binding.read_text())
        value["sha256"] = "changed"
        self.binding.write_text(json.dumps(value))
        try:
            with (
                patch("stararm_lumos_feedback.runtime.validate_socketcan") as can,
                patch("stararm_lumos_feedback.motion.create_session") as session,
                patch("stararm_lumos_feedback.inspection.Inspection") as observer,
            ):
                self.manager.request("start", mode="follow")
                self.finish_worker()
                self.assertEqual(self.manager.status()["state"], "error")
                self.assertIn("Binding changed", self.manager.status()["error"])
                self.assertIsNone(self.manager.status()["button_board"]["verified"])
                can.assert_not_called()
                session.assert_not_called()
                observer.assert_not_called()
        finally:
            self.binding.write_bytes(self.original_bytes[self.binding])
        self.assert_bound_files_unchanged()

    def test_motion_factory_receives_override_and_publication_records_actual_verification(self):
        selections, observed = [], []
        manager = self.manager

        class FakeSession:
            def __init__(self, profile, caps, publication, cancel, mode, *, button_enabled=None,
                         button_wait_until_cancel=False, **kwargs):
                self.publication, self.enabled = publication, button_enabled
                selections.append((mode, button_enabled, button_wait_until_cancel))

            def run(self):
                self.before_enable(object())
                if self.enabled:
                    for verified in (True, False):
                        self.publication.button_status(verified)
                        observed.append(manager.status()["button_board"]["verified"])

        with (
            patch("stararm_lumos_feedback.runtime.session_lock", return_value=nullcontext()),
            patch("stararm_lumos_feedback.runtime.validate_socketcan"),
            patch("stararm_lumos_feedback.motion.create_session", side_effect=FakeSession),
            patch("stararm_lumos_feedback.motion.preflight_session_pose") as preflight,
        ):
            for mode, enabled in (("follow", True), ("monitor", False)):
                self.manager.set_button_board(enabled)
                self.manager.request("start", mode=mode)
                self.finish_worker()
                board = self.manager.status()["button_board"]
                self.assertIs(board["enabled"], enabled)
                self.assertIs(board["verified"], False if enabled else None)
                self.assertEqual(self.manager.status()["state"], "stopped")
            self.assertEqual(preflight.call_count, 2)
        self.assertEqual(selections, [("follow", True, True), ("monitor", False, True)])
        self.assertEqual(observed, [True, False])
        self.assert_bound_files_unchanged()

    def test_button_specific_no_reply_is_false_while_servo_zero_error_is_unknown(self):
        class ButtonMissing(RuntimeError):
            code = "button_board_no_reply"
            interface = None
            detail = "Button board ID 7 did not respond"

        class FakeSession:
            def __init__(self, profile, caps, publication, cancel, mode, **kwargs):
                pass

            def run(self):
                raise failure

        for failure, expected in (
            (ButtonMissing("Button board ID 7 did not respond"), False),
            (RuntimeError("StarArm servo 0 did not respond"), None),
        ):
            with (
                self.subTest(error=str(failure)),
                patch("stararm_lumos_feedback.runtime.session_lock", return_value=nullcontext()),
                patch("stararm_lumos_feedback.runtime.validate_socketcan"),
                patch("stararm_lumos_feedback.motion.create_session", side_effect=FakeSession),
            ):
                self.manager.button_board.record(True)
                self.manager.request("start", mode="follow")
                self.finish_worker()
                status = self.manager.status()
                self.assertEqual(status["state"], "error")
                self.assertIs(status["button_board"]["verified"], expected)
                self.assertEqual(status["error_code"], getattr(failure, "code", None))
                self.assertIn(str(failure), status["error"])
        self.assert_bound_files_unchanged()


if __name__ == "__main__":
    unittest.main()
