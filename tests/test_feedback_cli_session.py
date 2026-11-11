"""Original-config matching and shared CLI motion lifecycle; hardware is replaced."""

import io
import json
import threading
from contextlib import nullcontext, redirect_stderr, redirect_stdout
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from test_feedback_runtime import FixtureCase

from stararm_lumos_feedback.cli import main, validate_session_config
from stararm_lumos_feedback.config import REVIEWED_NATIVE_CORE, REVIEWED_NATIVE_RANGES


class FeedbackSessionConfigTests(FixtureCase):
    def setUp(self):
        super().setUp()
        self.session_path = self.profile.paths["session_file"]
        self.binding = self.profile.path.parent / "entry.binding.json"
        self.binding.write_text(json.dumps(self.profile.binding()))

    def arguments(self, mode="monitor", config=None):
        return [
            mode, "--profile", str(self.profile.path), "--binding", str(self.binding),
            "--session-config", str(config or self.session_path), "--no-web", "--seconds", "0.1",
            "--settings", str(self.session_path.parent / "absent.caps.json"),
        ]

    def test_equivalent_config_copy_normalizes_paths_without_changing_inputs(self):
        data = json.loads(self.session_path.read_text())
        for section, keys in (("robot", ("sdk_root", "mapping_file", "joint_limits_file")),
                              ("teleop", ("mapping_file",))):
            for key in keys:
                data[section][key] = str((self.session_path.parent / data[section][key]).resolve())
        other = self.session_path.parent / "copy.json"
        other.write_text(json.dumps(data))
        before = {path: path.read_bytes() for path in (other, self.session_path, self.binding)}
        validate_session_config(self.profile, other)
        self.assertEqual(before, {path: path.read_bytes() for path in before})

    def test_changed_mapping_direction_or_sdk_is_rejected_before_session_or_lock(self):
        original = json.loads(self.session_path.read_text())
        for section, key, value in (
            ("teleop", "signs", [-1, 1, 1, 1, -1, -1]),
            ("teleop", "mapping_file", "different-calibration.json"),
            ("teleop", "button_enabled", False),
            ("robot", "sdk_root", "different-sdk"),
        ):
            with self.subTest(key=key):
                data = json.loads(json.dumps(original))
                data[section][key] = value
                other = self.session_path.parent / "different.json"
                other.write_text(json.dumps(data))
                with (
                    patch("stararm_lumos_feedback.motion.create_session") as create,
                    patch("stararm_lumos_feedback.runtime.session_lock") as lock,
                    redirect_stderr(io.StringIO()) as errors,
                    self.assertRaises(SystemExit) as caught,
                ):
                    main(self.arguments(config=other))
                self.assertEqual(caught.exception.code, 2)
                self.assertIn("differs from", errors.getvalue())
                create.assert_not_called()
                lock.assert_not_called()

    def test_cli_uses_shared_session_factory_and_original_preflight_hook(self):
        events = []
        robot = object()
        transport = object()
        session = SimpleNamespace(robot=robot)

        def run(seconds):
            events.append(("run", seconds))
            session.before_enable(transport)

        session.run = run
        with (
            patch("stararm_lumos_feedback.runtime.session_lock", return_value=nullcontext()) as lock,
            patch("stararm_lumos_feedback.motion.create_session", return_value=session) as create,
            patch("stararm_lumos_feedback.motion.preflight_session_pose") as preflight,
            redirect_stdout(io.StringIO()),
        ):
            main(self.arguments())
        lock.assert_called_once_with("mock-can", "mock-uart")
        create.assert_called_once()
        profile, caps, dashboard, cancel, mode = create.call_args.args
        self.assertEqual(profile.paths["session_file"], self.session_path)
        self.assertIsNone(dashboard)
        self.assertEqual(mode, "monitor")
        self.assertFalse(cancel.is_set())
        preflight.assert_called_once_with(profile, cancel, transport, robot=robot)
        self.assertEqual(events, [("run", 0.1)])

    def test_shared_motion_factory_preserves_native_range_gate_before_constructors(self):
        from stararm_lumos_feedback.config import SDKMotorRangeError
        from stararm_lumos_feedback.control import Caps
        from stararm_lumos_feedback.motion import create_session

        native = Path(self.profile.session["robot"]["sdk_root"]) / "src/libstartouch.so"
        self.profile.files[str(native)] = REVIEWED_NATIVE_CORE
        for axis, ranges in zip(self.profile.axes, REVIEWED_NATIVE_RANGES):
            axis.update(zip(("p_max", "v_max", "t_max"), ranges))
        for axis in self.profile.axes[:3]:
            axis["v_max"] = 10
        with (
            patch("lerobot_robot_lumos_stararm102.lumos.Lumos") as robot,
            patch("lerobot_robot_lumos_stararm102.stararm102.StarArm102") as leader,
            patch("importlib.import_module") as sdk_import,
            self.assertRaises(SDKMotorRangeError),
        ):
            create_session(self.profile, Caps(), None, threading.Event(), "monitor")
        sdk_import.assert_not_called()
        robot.assert_not_called()
        leader.assert_not_called()
