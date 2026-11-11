"""Commissioning can observe incomplete models without enabling force or rewriting calibration."""

import io
import json
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest.mock import patch

from test_feedback_runtime import configuration

from stararm_lumos_feedback.cli import main, prepare
from stararm_lumos_feedback.config import Profile


class CommissioningTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.profile = configuration(self.root)
        self.output = self.root / "commissioning.local.json"
        session_path = self.root / "session.json"
        session = json.loads(session_path.read_text())
        session["robot"].update(max_speed_deg_per_sec=30, max_target_step_deg=1)
        session_path.write_text(json.dumps(session))
        sdk_config = self.root / "sdk/src/config/robot_kinematics.yaml"
        sdk_config.write_text(
            "gripper_control: {type: TypeLJ}\n"
            "pos_vel_mode:\n"
            "  max_acc_limits: [10, 10, 10, 10, 10, 10]\n"
            "  max_dec_limits: [-10, -10, -10, -10, -10, -10]\n"
        )
        data = json.loads(self.profile.path.read_text())
        data["mass_coverage"] = "UNCONFIRMED"
        for axis in data["axes"]:
            for key in (
                "max_age_s",
                "max_temperature_c",
                "max_jump",
                "max_velocity",
                "max_acceleration",
                "sdk_tolerance",
                "limit_margin",
            ):
                axis[key] = None
        data["policy"] = [{key: None for key in policy} for policy in data["policy"]]
        self.profile.path.write_text(json.dumps(data))

    def prepared(self):
        with redirect_stdout(io.StringIO()):
            prepare(self.profile.path, self.output)
        return Profile(self.output, purpose="inspect")

    def test_prepare_fills_observation_values_without_confirming_mass_or_calibration(self):
        calibration = (self.root / "calibration.json").read_bytes()
        source = self.profile.path.read_bytes()
        profile = self.prepared()
        self.assertEqual(profile.data["version"], 3)
        self.assertTrue(profile.engineering)
        self.assertFalse(profile.mass_confirmed)
        self.assertEqual(profile.commissioning["baseline_status"], "not_measured")
        self.assertFalse(profile.commissioning["force_authorized"])
        for purpose in ("follow", "monitor"):
            self.assertTrue(Profile(self.output, purpose=purpose).engineering)
        self.assertEqual((self.root / "calibration.json").read_bytes(), calibration)
        self.assertEqual(self.profile.path.read_bytes(), source)

    def test_engineering_configuration_never_opens_run_even_with_valid_binding(self):
        profile = self.prepared()
        binding = self.root / "test.binding.json"
        binding.write_text(json.dumps(profile.binding()))
        with patch("stararm_lumos_feedback.config.verify_sdk_imports") as sdk:
            with self.assertRaises(SystemExit) as error:
                main(["run", "--profile", str(self.output), "--binding", str(binding)])
        self.assertEqual(error.exception.code, 2)
        sdk.assert_not_called()
        data = json.loads(self.output.read_text())
        data["commissioning"]["force_authorized"] = True
        data["commissioning"]["allowed_modes"].append("run")
        self.output.write_text(json.dumps(data))
        with self.assertRaisesRegex(ValueError, "force output must remain disabled"):
            Profile(self.output, purpose="run")

    def test_prepare_refuses_overwrite_and_rejects_unknown_motor_ranges(self):
        self.prepared()
        original = self.output.read_bytes()
        with self.assertRaises(FileExistsError):
            prepare(self.profile.path, self.output)
        self.assertEqual(self.output.read_bytes(), original)
        data = json.loads(self.profile.path.read_text())
        data["axes"][0]["t_max"] = None
        self.profile.path.write_text(json.dumps(data))
        other = self.root / "invalid.local.json"
        with self.assertRaisesRegex(ValueError, "verified finite"):
            prepare(self.profile.path, other)
        self.assertFalse(other.exists())

    def test_inspect_binding_is_explicit_and_cli_never_imports_or_constructs_sdk(self):
        profile = self.prepared()
        binding = self.root / "test.binding.json"
        with redirect_stdout(io.StringIO()):
            main(["bind", "--profile", str(self.output), "--binding", str(binding), "--purpose", "inspect"])
        profile.verify(binding)
        with (
            patch("stararm_lumos_feedback.config.verify_sdk_imports") as sdk,
            patch("stararm_lumos_feedback.runtime.validate_socketcan") as interface,
            patch("stararm_lumos_feedback.inspection.Inspection") as inspection,
            redirect_stdout(io.StringIO()),
        ):
            inspection.return_value.run.return_value = {"mode": "inspect"}
            main(
                [
                    "inspect",
                    "--profile",
                    str(self.output),
                    "--binding",
                    str(binding),
                    "--no-web",
                    "--seconds",
                    "1",
                ]
            )
        sdk.assert_not_called()
        interface.assert_called_once_with("mock-can", 1_000_000)
        inspection.return_value.run.assert_called_once_with(1.0)

    def test_inspect_interrupt_does_not_claim_leader_was_released(self):
        profile = self.prepared()
        binding = self.root / "test.binding.json"
        binding.write_text(json.dumps(profile.binding()))
        output = io.StringIO()
        with (
            patch("stararm_lumos_feedback.runtime.validate_socketcan"),
            patch("stararm_lumos_feedback.inspection.Inspection") as inspection,
            redirect_stdout(output),
        ):
            inspection.return_value.run.side_effect = KeyboardInterrupt
            main(["inspect", "--profile", str(self.output), "--binding", str(binding), "--no-web"])
        self.assertIn("Leader was not controlled", output.getvalue())
        self.assertNotIn("released", output.getvalue())


if __name__ == "__main__":
    unittest.main()
