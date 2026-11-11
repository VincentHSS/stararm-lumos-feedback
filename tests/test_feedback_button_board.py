"""Local button selection persistence without hardware or source edits."""

import json
import stat
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from stararm_lumos_feedback.button_board import ButtonBoard


def profile(**teleop):
    return SimpleNamespace(session=dict(teleop=teleop))


class ButtonBoardTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.path = self.root / "settings" / "button-board.json"

    def test_defaults_and_bound_id_are_used_without_creating_a_file(self):
        board = ButtonBoard(profile(), self.path)
        self.assertEqual(board.get(), dict(enabled=True, id=7, verified=None))
        self.assertFalse(self.path.exists())
        board = ButtonBoard(profile(button_enabled=False, button_id=12))
        self.assertEqual(board.get(), dict(enabled=False, id=12, verified=None))

    def test_persists_only_selection_with_private_permissions_and_restores(self):
        board = ButtonBoard(profile(button_id=12), self.path)
        board.record(True)
        result = board.set(False)
        self.assertEqual(result, dict(enabled=False, id=12, verified=None))
        self.assertEqual(json.loads(self.path.read_text()), dict(enabled=False))
        self.assertEqual(stat.S_IMODE(self.path.stat().st_mode), 0o600)
        restored = ButtonBoard(profile(button_id=12), self.path)
        self.assertEqual(restored.get(), result)
        self.assertEqual(list(self.path.parent.iterdir()), [self.path])
        self.path.chmod(0o644)
        restored.set(True)
        self.assertEqual(stat.S_IMODE(self.path.stat().st_mode), 0o600)

    def test_record_is_memory_only_and_reselection_clears_verification(self):
        board = ButtonBoard(profile(), self.path)
        board.set(True)
        saved = self.path.read_bytes()
        for verified in (True, False, None):
            with self.subTest(verified=verified):
                board.record(verified)
                self.assertIs(board.verified, verified)
                self.assertEqual(self.path.read_bytes(), saved)
        board.record(True)
        board.set(True)
        self.assertIsNone(board.verified)
        self.assertIsNone(ButtonBoard(profile(), self.path).verified)

    def test_replace_failure_retains_memory_verification_and_previous_file(self):
        board = ButtonBoard(profile(), self.path)
        board.set(True)
        board.record(True)
        saved = self.path.read_bytes()
        with patch("stararm_lumos_feedback.button_board.os.replace", side_effect=OSError("save failed")):
            with self.assertRaisesRegex(OSError, "save failed"):
                board.set(False)
        self.assertEqual(board.get(), dict(enabled=True, id=7, verified=True))
        self.assertEqual(self.path.read_bytes(), saved)
        self.assertEqual(list(self.path.parent.iterdir()), [self.path])

    def test_write_failure_retains_memory_and_cleans_up_temporary_file(self):
        board = ButtonBoard(profile(), self.path)
        board.record(False)
        with patch("stararm_lumos_feedback.button_board.json.dump", side_effect=OSError("write failed")):
            with self.assertRaisesRegex(OSError, "write failed"):
                board.set(False)
        self.assertEqual(board.get(), dict(enabled=True, id=7, verified=False))
        self.assertFalse(self.path.exists())
        self.assertEqual(list(self.path.parent.iterdir()), [])

    def test_non_boolean_selection_and_verification_are_rejected(self):
        board = ButtonBoard(profile(), self.path)
        for value in (None, 0, 1, "false", [], {}, 0.0):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    board.set(value)
                if value is not None:
                    with self.assertRaises(ValueError):
                        board.record(value)
        self.assertEqual(board.get(), dict(enabled=True, id=7, verified=None))
        self.assertFalse(self.path.exists())

    def test_malformed_saved_settings_are_not_silently_ignored(self):
        self.path.parent.mkdir()
        invalid = [
            "{", "null", "true", "[]", "{}",
            '{"enabled":false,"id":7}', '{"enabled":false,"verified":true}',
            '{"enabled":null}', '{"enabled":0}', '{"enabled":1}',
            '{"enabled":"false"}',
        ]
        for value in invalid:
            with self.subTest(value=value):
                self.path.write_text(value)
                with self.assertRaises(ValueError):
                    ButtonBoard(profile(), self.path)
                self.assertEqual(self.path.read_text(), value)

    def test_profile_values_are_validated(self):
        for enabled in (None, 0, 1, "true"):
            with self.subTest(enabled=enabled), self.assertRaises(ValueError):
                ButtonBoard(profile(button_enabled=enabled))
        for button_id in (True, 6, 254, 7.0, "7", None):
            with self.subTest(button_id=button_id), self.assertRaises(ValueError):
                ButtonBoard(profile(button_id=button_id))

    def test_original_session_profile_binding_and_calibration_are_unchanged(self):
        original = profile(button_enabled=True, button_id=7)
        before = json.dumps(original.session, sort_keys=True)
        paths = [self.root / name for name in ("session.json", "profile.json", "binding.json", "calibration.json")]
        for path in paths:
            path.write_bytes(b'{"preserve": "original bytes"}\n')
        saved = {path: path.read_bytes() for path in paths}
        board = ButtonBoard(original, self.path)
        board.set(False)
        board.record(True)
        self.assertEqual(json.dumps(original.session, sort_keys=True), before)
        self.assertEqual({path: path.read_bytes() for path in paths}, saved)

    def test_no_path_never_writes_a_setting(self):
        with patch("stararm_lumos_feedback.button_board.tempfile.NamedTemporaryFile") as temporary:
            board = ButtonBoard(profile())
            board.record(True)
            board.set(False)
            board.record(False)
            temporary.assert_not_called()
        self.assertEqual(board.get(), dict(enabled=False, id=7, verified=False))


if __name__ == "__main__":
    unittest.main()
