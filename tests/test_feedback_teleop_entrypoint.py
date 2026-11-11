"""Offline launcher regressions; subprocess dispatch is mocked before any hardware access."""

import io
import json
import sys
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path
from unittest.mock import patch

from lerobot_robot_lumos_stararm102 import cli


class FeedbackTeleopEntrypointTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="feedback entry ")
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.config = self.root / "session 配置.json"
        self.profile = self.root / "feedback profile.json"
        self.binding = self.root / "feedback binding.json"
        self.data = {
            "robot": {
                "type": "lumos", "sdk_root": "sdk", "mapping_file": "calibration.json",
                "joint_limits_file": "limits.json", "gripper": True,
            },
            "teleop": {
                "type": "stararm102", "port": "mock-uart", "mapping_file": "calibration.json",
                "gripper": True, "button_enabled": False,
            },
            "fps": 30,
            "dataset": {"repo_id": "local/entrypoint", "single_task": "offline launcher test"},
        }
        self.write_config()
        (self.root / "limits.json").write_text(
            json.dumps({"lower_rad": [-2.0] * 6, "upper_rad": [2.0] * 6}), encoding="utf-8"
        )
        dispatch = patch.object(cli.subprocess, "call", return_value=0)
        self.dispatch = dispatch.start()
        self.addCleanup(dispatch.stop)
        # A launcher/print regression must never try opening a serial device.
        serial = patch("serial.Serial", side_effect=AssertionError("Unexpected hardware access"))
        serial.start()
        self.addCleanup(serial.stop)

    def write_config(self):
        self.config.write_text(json.dumps(self.data, ensure_ascii=False), encoding="utf-8")

    def feedback_options(self):
        return ["--feedback-profile", str(self.profile), "--feedback-binding", str(self.binding)]

    def expected_feedback(self, mode="monitor"):
        return [
            sys.executable, "-m", "stararm_lumos_feedback.cli", mode,
            "--profile", str(self.profile), "--binding", str(self.binding),
            "--session-config", str(self.config),
        ]

    def invoke(self, mode="teleop", *options):
        output, errors = io.StringIO(), io.StringIO()
        with (
            patch.object(sys, "argv", ["lumos-lerobot", mode, "--config", str(self.config), *options]),
            redirect_stdout(output),
            redirect_stderr(errors),
        ):
            cli.main()
        return output.getvalue(), errors.getvalue()

    def test_feedback_print_defaults_to_monitor_and_preserves_existing_config(self):
        before = self.config.read_bytes()
        with patch.object(cli, "command_for", side_effect=AssertionError("Unexpected ordinary loop")):
            output, errors = self.invoke("teleop", *self.feedback_options(), "--print-command")
        self.assertEqual(json.loads(output), self.expected_feedback())
        self.assertEqual(errors, "")
        self.dispatch.assert_not_called()
        self.assertEqual(self.config.read_bytes(), before)
        self.assertFalse(self.profile.exists())
        self.assertFalse(self.binding.exists())

    def test_feedback_modes_and_session_duration_reach_guarded_cli(self):
        self.data["teleop_time_s"] = 4.5
        self.write_config()
        for mode in ("follow", "monitor", "run"):
            with self.subTest(mode=mode):
                output, _ = self.invoke(
                    "teleop", *self.feedback_options(), "--feedback-mode", mode, "--print-command"
                )
                self.assertEqual(json.loads(output), self.expected_feedback(mode) + ["--seconds", "4.5"])
        self.dispatch.assert_not_called()

    def test_feedback_dispatch_preserves_subprocess_failure_and_reports_session_loop(self):
        self.dispatch.return_value = 17
        with patch("builtins.print") as printer, self.assertRaises(SystemExit) as caught:
            self.invoke("teleop", *self.feedback_options(), "--feedback-mode", "follow")
        self.assertEqual(caught.exception.code, 17)
        self.dispatch.assert_called_once_with(self.expected_feedback("follow"))
        self.assertIn("guarded feedback Session", printer.call_args.args[0])

    def test_feedback_flags_are_rejected_outside_teleop_before_loading_config(self):
        for mode in ("doctor", "calibrate", "preview", "record"):
            with self.subTest(mode=mode), patch.object(cli, "load_session") as load:
                with self.assertRaises(SystemExit) as caught:
                    self.invoke(mode, *self.feedback_options(), "--print-command")
                self.assertEqual(caught.exception.code, 2)
                load.assert_not_called()
        self.dispatch.assert_not_called()

    def test_feedback_rejects_unknown_mode_before_loading_config(self):
        with patch.object(cli, "load_session") as load:
            with self.assertRaises(SystemExit) as caught:
                self.invoke("teleop", *self.feedback_options(), "--feedback-mode", "unknown")
        self.assertEqual(caught.exception.code, 2)
        load.assert_not_called()
        self.dispatch.assert_not_called()

    def test_feedback_paths_must_be_paired_and_mode_cannot_select_feedback_alone(self):
        invalid = (
            ["--feedback-profile", str(self.profile)],
            ["--feedback-binding", str(self.binding)],
            ["--feedback-mode", "monitor"],
            ["--feedback-profile", str(self.profile), "--feedback-mode", "follow"],
            ["--feedback-binding", str(self.binding), "--feedback-mode", "run"],
        )
        for options in invalid:
            with self.subTest(options=options), patch.object(cli, "load_session") as load:
                with self.assertRaises(SystemExit) as caught:
                    self.invoke("teleop", *options, "--print-command")
                self.assertEqual(caught.exception.code, 2)
                load.assert_not_called()
        self.dispatch.assert_not_called()

    def test_feedback_rejects_mismatched_pair_before_dispatch(self):
        self.data["teleop"]["mapping_file"] = "different-calibration.json"
        self.write_config()
        with self.assertRaises(SystemExit) as caught:
            self.invoke("teleop", *self.feedback_options(), "--print-command")
        self.assertEqual(caught.exception.code, 2)
        self.dispatch.assert_not_called()

    def test_existing_print_commands_keep_the_original_lerobot_entrypoints(self):
        data = cli.load_session(self.config)
        for mode in ("preview", "teleop", "record"):
            with self.subTest(mode=mode):
                output, _ = self.invoke(mode, "--print-command")
                command = json.loads(output)
                self.assertEqual(command, cli.command_for(mode, data))
                self.assertIn("lerobot.scripts.lerobot_", command[2])
        self.dispatch.assert_not_called()

    def test_existing_teleop_execution_still_dispatches_original_command(self):
        with self.assertRaises(SystemExit) as caught:
            self.invoke("teleop")
        self.assertEqual(caught.exception.code, 0)
        self.dispatch.assert_called_once_with(cli.command_for("teleop", cli.load_session(self.config)))

    def test_existing_doctor_and_calibration_routes_are_unchanged(self):
        with patch.object(cli, "doctor") as doctor:
            self.invoke("doctor")
        doctor.assert_called_once_with(cli.load_session(self.config))
        with patch.object(cli, "calibrate", return_value=5) as calibrate:
            with self.assertRaises(SystemExit) as caught:
                self.invoke("calibrate")
        self.assertEqual(caught.exception.code, 5)
        calibrate.assert_called_once_with(cli.load_session(self.config))
        self.dispatch.assert_not_called()


if __name__ == "__main__":
    unittest.main()
