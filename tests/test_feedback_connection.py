"""Persistent dashboard connection boundaries; all hardware is replaced by fakes."""

import json
import signal
import subprocess
import sys
import tempfile
import threading
import unittest
from pathlib import Path
from unittest.mock import patch

from test_feedback_runtime import configuration

from stararm_lumos_feedback.config import Profile
from stararm_lumos_feedback.connection import InspectionConnection
from stararm_lumos_feedback.control import Caps
from stararm_lumos_feedback.dashboard import Dashboard
from stararm_lumos_feedback.runtime import SocketCANError


class ConnectionTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        original = configuration(Path(temporary.name))
        self.profile = Profile(original.path, purpose="inspect")
        self.binding = original.path.parent / "binding.json"
        self.binding.write_text(json.dumps(self.profile.binding()))
        self.dashboard = Dashboard(Caps(), self.profile, port=0)
        self.dashboard.start()
        self.addCleanup(self.dashboard.close)
        self.connection = InspectionConnection(self.profile, self.binding, self.dashboard)
        self.addCleanup(self.connection.close)

    def test_idle_and_invalid_actions_never_open_hardware(self):
        with patch("stararm_lumos_feedback.inspection.Inspection") as observer:
            self.assertEqual(self.connection.status()["state"], "idle")
            self.assertEqual(self.dashboard.state["status"], "connection_idle")
            for action in ("run", "enable", "follow", None):
                with self.assertRaises(ValueError):
                    self.connection.request(action)
            self.connection.request("stop")
            observer.assert_not_called()

    def test_known_native_units_block_all_motion_but_preserve_read_only_connection(self):
        mismatch = [dict(axis=1, parameter="v_max", sdk=8, hardware=10)]
        started = threading.Event()

        class Observer:
            def __init__(self, *args, **kwargs):
                self.stop = threading.Event()

            def run(self):
                started.set()
                self.stop.wait(2)

        with (
            patch("stararm_lumos_feedback.connection.native_motor_range_mismatches", return_value=mismatch),
            patch("stararm_lumos_feedback.runtime.validate_socketcan"),
            patch("stararm_lumos_feedback.inspection.Inspection", Observer),
            patch("stararm_lumos_feedback.motion.create_session") as motion,
        ):
            status = self.connection.status()
            self.assertEqual(status["available_modes"], ["inspect"])
            self.assertEqual(status["sdk_motor_range_mismatches"], mismatch)
            for mode in ("follow", "monitor", "run"):
                self.assertEqual(status["blocked_modes"][mode], "sdk_motor_range_mismatch")
                with self.assertRaisesRegex(ValueError, "SDK=8, hardware=10"):
                    self.connection.request("start", mode)
            self.connection.request("start", "inspect")
            self.assertTrue(started.wait(2))
            self.connection.request("stop")
            self.connection.worker.join(2)
            motion.assert_not_called()

    def test_one_owner_explicit_stop_and_shutdown(self):
        started = threading.Event()

        class Observer:
            def __init__(self, *args, **kwargs):
                self.stop = threading.Event()

            def run(self):
                started.set()
                if not self.stop.wait(2):
                    raise AssertionError("Worker was not explicitly stopped")

        with (
            patch("stararm_lumos_feedback.runtime.validate_socketcan"),
            patch("stararm_lumos_feedback.inspection.Inspection", Observer),
        ):
            self.connection.request("start")
            self.assertTrue(started.wait(2))
            self.assertFalse(self.connection.status()["can_start"])
            with self.assertRaisesRegex(RuntimeError, "already running"):
                self.connection.request("start")
            self.connection.request("stop")
            self.connection.worker.join(2)
            self.assertFalse(self.connection.worker.is_alive())
            self.assertEqual(self.connection.status()["state"], "stopped")
            self.assertEqual(self.dashboard.state["status"], "inspection_stopped")
            self.connection.close()
            with self.assertRaisesRegex(RuntimeError, "closed"):
                self.connection.request("start")

    def test_error_stays_visible_without_retry_and_allows_explicit_retry(self):
        with (
            patch("stararm_lumos_feedback.runtime.validate_socketcan"),
            patch("stararm_lumos_feedback.inspection.Inspection") as observer,
        ):
            observer.return_value.run.side_effect = RuntimeError("CAN unavailable")
            for attempt in range(2):
                self.connection.request("start")
                self.connection.worker.join(2)
                self.assertEqual(self.connection.status()["state"], "error")
                self.assertEqual(self.dashboard.state["error"], "CAN unavailable")
                self.assertEqual(observer.return_value.run.call_count, attempt + 1)
            self.assertTrue(self.dashboard.thread.is_alive())

    def test_binding_is_rechecked_before_any_can_access(self):
        mapping = Path(self.profile.session["robot"]["mapping_file"])
        mapping.write_text(mapping.read_text() + "\n")
        with (
            patch("stararm_lumos_feedback.runtime.validate_socketcan") as interface,
            patch("stararm_lumos_feedback.inspection.Inspection") as observer,
        ):
            self.connection.request("start")
            self.connection.worker.join(2)
            self.assertEqual(self.connection.status()["state"], "error")
            self.assertIn("Binding changed", self.connection.status()["error"])
            interface.assert_not_called()
            observer.assert_not_called()

    def test_serve_startup_is_offline_and_sigterm_runs_cleanup(self):
        script = r"""
import importlib.abc
import sys
class NoHardware(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        roots = ("can", "serial", "startouch", "startouchclass",
                 "stararm_lumos_feedback.runtime", "stararm_lumos_feedback.inspection")
        if any(fullname == root or fullname.startswith(root + ".") for root in roots):
            raise AssertionError("Hardware imported before Connect: " + fullname)
sys.meta_path.insert(0, NoHardware())
from stararm_lumos_feedback.cli import main
main(sys.argv[1:])
"""
        process = subprocess.Popen(
            [
                sys.executable,
                "-B",
                "-u",
                "-c",
                script,
                "serve",
                "--profile",
                str(self.profile.path),
                "--binding",
                str(self.binding),
                "--port",
                "0",
            ],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        self.addCleanup(lambda: process.poll() is None and process.kill())
        # A pipe reader plus bounded event avoids an unbounded test readline.
        ready = threading.Event()
        lines = []

        def read():
            for line in process.stdout:
                lines.append(line)
                if "Ready." in line:
                    ready.set()

        reader = threading.Thread(target=read, daemon=True)
        reader.start()
        self.assertTrue(ready.wait(5), "Serve did not reach offline ready state")
        process.send_signal(signal.SIGTERM)
        process.wait(timeout=5)
        reader.join(1)
        self.assertEqual(process.returncode, 0, process.stderr.read())
        self.assertIn("hardware sessions closed", "".join(lines))
        process.stdout.close()
        process.stderr.close()

    def test_preflight_diagnostics_reach_page_and_clear_on_explicit_retry(self):
        failure = SocketCANError("can_interface_missing", "can0", 'Device "can0" does not exist.')
        started = threading.Event()

        class Observer:
            def __init__(self, *args, **kwargs):
                self.stop = threading.Event()

            def run(self):
                started.set()
                if not self.stop.wait(2):
                    raise AssertionError("Worker was not explicitly stopped")

        with (
            patch("stararm_lumos_feedback.runtime.validate_socketcan", side_effect=[failure, None]),
            patch("stararm_lumos_feedback.inspection.Inspection", Observer),
        ):
            self.connection.request("start")
            self.connection.worker.join(2)
            self.assertFalse(started.is_set())
            for state in (self.connection.status(), self.dashboard.snapshot()):
                self.assertEqual(state["error_code"], "can_interface_missing")
                self.assertEqual(state["error_interface"], "can0")
                self.assertEqual(state["error_detail"], failure.detail)
            self.connection.request("start")
            self.assertTrue(started.wait(2))
            for state in (self.connection.status(), self.dashboard.snapshot()):
                self.assertIsNone(state["error_code"])
                self.assertIsNone(state["error_interface"])
                self.assertIsNone(state["error_detail"])
                self.assertIsNone(state["error"])
            self.connection.request("stop")
            self.connection.worker.join(2)


if __name__ == "__main__":
    unittest.main()
