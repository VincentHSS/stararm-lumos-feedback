"""Pure pre-enable pose checks; these tests never construct device SDKs."""

import json
import math
import subprocess
import sys
import tempfile
import unittest
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace

from stararm_lumos_feedback.startup import (
    StartupPoseError,
    alignment_reference,
    check_startup_pose,
    current_alignment_reference,
    relative_alignment_reference,
)


class StartupPoseTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.mapping = dict(
            format_version=1,
            gripper_enabled=True,
            leader_port="mock-uart",
            can_interface="mock-can",
            joint_map=list(range(6)),
            leader_home_deg=[0] * 6,
            leader_range_min_deg=[-90] * 6,
            leader_range_max_deg=[90] * 6,
            lumos_home_rad=[0] * 6,
            gripper_range=dict(closed_deg=0, open_deg=100),
        )
        self.limits = dict(lower_rad=[-2] * 6, upper_rad=[2] * 6)
        self.session = dict(
            robot=dict(
                type="lumos",
                can_interface="mock-can",
                mapping_file="calibration.json",
                joint_limits_file="limits.json",
                gripper=True,
            ),
            teleop=dict(
                type="stararm102",
                port="mock-uart",
                mapping_file="calibration.json",
                gripper=True,
            ),
        )
        self.profile = SimpleNamespace(
            session=self.session, paths={"session_file": self.root / "session.json"},
            to_model=lambda index, value: value,
        )
        self.angles = dict.fromkeys(range(7), 0.0)
        self.q = [0.0] * 6 + [0.35]
        self.write_files()

    def write_files(self):
        (self.root / "calibration.json").write_text(json.dumps(self.mapping))
        (self.root / "limits.json").write_text(json.dumps(self.limits))

    def assert_rejected(self, q=None, angles=None):
        with self.assertRaises(StartupPoseError) as raised:
            check_startup_pose(
                self.profile, self.q if q is None else q, self.angles if angles is None else angles
            )
        error = raised.exception
        self.assertEqual(error.code, "startup_pose_mismatch")
        self.assertIsNone(error.interface)
        self.assertTrue(error.detail)
        return error

    def test_aligned_uses_adapter_defaults_and_preserves_files(self):
        before = {path: path.read_bytes() for path in self.root.iterdir()}
        result = check_startup_pose(self.profile, self.q, self.angles)
        self.assertEqual(result["startup_strategy"], "home")
        self.assertEqual(result["leader_target_sdk"], self.q)
        self.assertEqual(result["tolerance_rad"], [0.15, 0.4, 0.15, 0.15, 0.15, 0.15])
        self.assertEqual(result["pair_error_rad"], [0] * 6)
        self.assertEqual(before, {path: path.read_bytes() for path in self.root.iterdir()})

    def test_omitted_and_null_pair_tolerance_preserve_each_legacy_axis_limit(self):
        self.q[1] = 0.3  # J2 retains its distinct 0.4 rad default.
        omitted = check_startup_pose(self.profile, self.q, self.angles)
        self.session["robot"]["startup_pair_tolerance_rad"] = None
        explicit_null = check_startup_pose(self.profile, self.q, self.angles)
        self.assertEqual(explicit_null, omitted)
        self.assertEqual(omitted["pair_tolerance_rad"], omitted["tolerance_rad"])
        self.q[0] = 0.1
        self.angles[0] = math.degrees(0.1)  # Target -0.1 rad; pair difference 0.2.
        self.assertEqual(self.assert_rejected().diagnostics["axes"][0]["failed_checks"], ["pair"])

    def test_home_pair_override_accepts_separated_arms_without_relaxing_home_distances(self):
        self.q[0] = math.radians(8)
        self.angles[0] = 8  # J1 sign -1: target -8 degrees.
        self.assertEqual(self.assert_rejected().diagnostics["axes"][0]["failed_checks"], ["pair"])
        self.session["robot"]["startup_pair_tolerance_rad"] = math.radians(20)
        before = {path: path.read_bytes() for path in self.root.iterdir()}
        result = check_startup_pose(self.profile, self.q, self.angles)
        self.assertAlmostEqual(result["pair_error_rad"][0], math.radians(16))
        self.assertEqual(result["pair_tolerance_rad"], [math.radians(20)] * 6)
        self.assertEqual(result["tolerance_rad"], [0.15, 0.4, 0.15, 0.15, 0.15, 0.15])
        self.assertEqual(before, {path: path.read_bytes() for path in self.root.iterdir()})
        self.q[0] = 0.16
        self.angles[0] = 0
        self.assertEqual(self.assert_rejected().diagnostics["axes"][0]["failed_checks"], ["follower_home"])
        self.q[0] = 0
        self.angles[0] = -math.degrees(0.16)
        self.assertEqual(self.assert_rejected().diagnostics["axes"][0]["failed_checks"], ["leader_home"])

    def test_home_pair_override_enforces_twenty_degrees_independently_for_joint2(self):
        self.session["robot"]["startup_pair_tolerance_rad"] = math.radians(20)
        self.q[1] = math.radians(10)
        self.angles[1] = -10
        check_startup_pose(self.profile, self.q, self.angles)
        self.angles[1] = -10.1
        axis = self.assert_rejected().diagnostics["axes"][0]
        self.assertEqual(axis["failed_checks"], ["pair"])
        self.assertEqual(axis["tolerance_rad"], 0.4)
        self.assertEqual(axis["pair_tolerance_rad"], math.radians(20))

    def test_home_pair_override_keeps_both_actual_and_target_joint_limits(self):
        self.session["robot"]["startup_pair_tolerance_rad"] = math.radians(20)
        self.limits["upper_rad"][0] = 0.05
        self.write_files()
        self.q[0] = 0.06
        self.assertEqual(self.assert_rejected().diagnostics["axes"][0]["failed_checks"], ["follower_limit"])
        self.q[0] = 0
        self.angles[0] = -math.degrees(0.06)
        self.assertEqual(self.assert_rejected().diagnostics["axes"][0]["failed_checks"], ["target_limit"])

    def test_home_pair_override_cannot_expand_current_alignment_or_relative_drift_caps(self):
        self.session["robot"].update(startup_pair_tolerance_rad=math.radians(20), max_target_step_deg=20)
        self.profile.engineering = True
        self.q[6] = 0
        for strategy in ("current_aligned", "current_relative"):
            with self.subTest(strategy=strategy):
                result = check_startup_pose(self.profile, self.q, self.angles, strategy=strategy)
                self.assertEqual(result["pair_tolerance_rad"], [math.radians(2)] * 6)
                self.assertEqual(result["tolerance_rad"], [math.radians(2)] * 6)
                angles = self.angles.copy()
                angles[0] = -3
                extra = {"session_offset": result["session_offset_sdk_rad"]} if strategy == "current_relative" else {}
                with self.assertRaises(StartupPoseError) as raised:
                    check_startup_pose(self.profile, self.q, angles, strategy=strategy, **extra)
                axis = raised.exception.diagnostics["axes"][0]
                self.assertEqual(axis["failed_checks"], ["pair"])
                self.assertEqual(axis["pair_tolerance_rad"], math.radians(2))

    def test_pair_tolerance_requires_a_finite_positive_number_or_null(self):
        for value in (0, -0.1, True, False, "0.35", math.nan, math.inf):
            with self.subTest(value=value):
                self.session["robot"]["startup_pair_tolerance_rad"] = value
                with self.assertRaisesRegex(StartupPoseError, "startup_pair_tolerance_rad"):
                    check_startup_pose(self.profile, self.q, self.angles)

    def test_leader_target_accepts_far_absolute_target_and_open_gripper_without_an_offset(self):
        self.profile.engineering = True
        self.q = [0.8, -0.3, 0.2, 0.6, -0.2, 0.4, 0.6]
        self.angles[0] = 60
        self.angles[3] = -45
        before = {path: path.read_bytes() for path in self.root.iterdir()}
        result = check_startup_pose(self.profile, self.q, self.angles, strategy="leader_target")
        self.assertEqual(result["startup_strategy"], "leader_target")
        self.assertIsNone(result["session_offset_sdk_rad"])
        self.assertEqual(result["follower_q_sdk"], self.q)
        self.assertAlmostEqual(result["leader_target_sdk"][0], -math.radians(60))
        self.assertAlmostEqual(result["leader_target_sdk"][3], -math.radians(45))
        self.assertGreater(result["pair_error_rad"][0], math.radians(2))
        self.assertEqual(result["tolerance_rad"], [math.radians(2)] * 6)
        self.assertEqual(before, {path: path.read_bytes() for path in self.root.iterdir()})

    def test_leader_target_uses_existing_target_clipping_but_keeps_actual_limits(self):
        self.profile.engineering = True
        self.limits["upper_rad"][0] = 0.05
        self.angles[0] = -80
        self.write_files()
        result = check_startup_pose(self.profile, self.q, self.angles, strategy="leader_target")
        self.assertGreater(result["leader_base_target_sdk"][0], 0.05)
        self.assertEqual(result["leader_target_sdk"][0], 0.05)
        self.q[0] = 0.051
        with self.assertRaises(StartupPoseError) as raised:
            check_startup_pose(self.profile, self.q, self.angles, strategy="leader_target")
        self.assertEqual(raised.exception.diagnostics["axes"][0]["failed_checks"], ["follower_limit"])

    def test_leader_target_retains_actual_joint2_constructor_requirement_only(self):
        self.profile.engineering = True
        self.mapping["leader_range_min_deg"][1] = -180
        self.angles[1] = -100
        self.write_files()
        result = check_startup_pose(self.profile, self.q, self.angles, strategy="leader_target")
        self.assertLess(result["leader_target_sdk"][1], -1.57)
        self.q[1] = math.nextafter(-1.57, -math.inf)
        with self.assertRaises(StartupPoseError) as raised:
            check_startup_pose(self.profile, self.q, self.angles, strategy="leader_target")
        self.assertEqual(raised.exception.diagnostics["reason"], "joint2_startup")

    def test_bound_measurement_margin_accepts_encoder_edges_without_expanding_targets(self):
        self.profile.engineering = True
        margin = 4 * 25 / 65535
        self.profile.axes = [dict(limit_margin=margin) for _ in range(7)]
        self.q[6] = 0
        for strategy in ("home", "current_aligned", "current_relative", "leader_target"):
            for boundary, direction in ((0.0, 1), (-2.0, -1)):
                self.limits["upper_rad"][2] = 0
                self.mapping["lumos_home_rad"][2] = boundary
                self.write_files()
                for excess in (25 / 65535 / 2, margin):
                    with self.subTest(strategy=strategy, boundary=boundary, excess=excess):
                        self.q[2] = boundary + direction * excess
                        actual = self.q.copy()
                        result = check_startup_pose(self.profile, self.q, self.angles, strategy=strategy)
                        self.assertEqual(result["follower_q_sdk"], actual)
                        self.assertEqual(result["leader_target_sdk"][2], boundary)
                        self.assertEqual(self.q, actual)
                        self.assertLessEqual(result["leader_target_sdk"][2], 0)
                        self.assertGreaterEqual(result["leader_target_sdk"][2], -2)
                self.q[2] = boundary + direction * (margin + 25 / 65535)
                with self.subTest(strategy=strategy, boundary=boundary), self.assertRaises(StartupPoseError) as error:
                    check_startup_pose(self.profile, self.q, self.angles, strategy=strategy)
                detail = error.exception.diagnostics["axes"][0]
                self.assertIn("follower_limit", detail["failed_checks"])
                self.assertEqual(detail["measured_limit_margin_rad"], margin)

    def test_measurement_margin_does_not_authorize_an_outside_command_target(self):
        self.profile.engineering = True
        self.profile.axes = [dict(limit_margin=0.0016) for _ in range(7)]
        self.limits["upper_rad"][2] = 0
        self.q[6] = 0
        self.angles[2] = 0.05  # Positive mapping on J3 in this synthetic pair.
        self.write_files()
        for strategy in ("home", "current_aligned"):
            with self.subTest(strategy=strategy), self.assertRaises(StartupPoseError) as error:
                check_startup_pose(self.profile, self.q, self.angles, strategy=strategy)
            self.assertEqual(error.exception.diagnostics["axes"][0]["failed_checks"], ["target_limit"])

    def test_invalid_bound_measurement_margins_are_rejected(self):
        for invalid in (-0.1, math.nan, math.inf, True, "0.01"):
            self.profile.axes = [dict(limit_margin=0) for _ in range(7)]
            self.profile.axes[2]["limit_margin"] = invalid
            with self.subTest(margin=invalid), self.assertRaises(StartupPoseError):
                check_startup_pose(self.profile, self.q, self.angles)

    def test_relative_boundary_offset_recheck_normalizes_only_float_roundoff(self):
        self.profile.engineering = True
        self.profile.axes = [dict(limit_margin=0.0016) for _ in range(7)]
        self.limits["lower_rad"][2], self.limits["upper_rad"][2] = -3.193, 0
        self.q[2], self.q[6] = -3.193 - 0.00019, 0
        self.angles[2] = -67.85
        self.write_files()
        first = check_startup_pose(self.profile, self.q, self.angles, strategy="current_relative")
        second = check_startup_pose(
            self.profile, self.q, self.angles, strategy="current_relative",
            session_offset=first["session_offset_sdk_rad"],
        )
        self.assertEqual(first["leader_target_sdk"][2], -3.193)
        self.assertEqual(second["leader_target_sdk"][2], -3.193)
        self.assertEqual(second["follower_q_sdk"][2], self.q[2])
        self.angles[2] -= 0.001
        with self.assertRaises(StartupPoseError) as error:
            check_startup_pose(
                self.profile, self.q, self.angles, strategy="current_relative",
                session_offset=first["session_offset_sdk_rad"],
            )
        self.assertIn("target_limit", error.exception.diagnostics["axes"][0]["failed_checks"])

    def test_leader_target_requires_engineering_valid_opening_and_no_relative_offset(self):
        with self.assertRaisesRegex(StartupPoseError, "engineering"):
            check_startup_pose(self.profile, self.q, self.angles, strategy="leader_target")
        self.profile.engineering = True
        for opening in (-0.001, 1.001, math.nan):
            with self.subTest(opening=opening), self.assertRaises(StartupPoseError):
                check_startup_pose(self.profile, self.q[:6] + [opening], self.angles, strategy="leader_target")
        with self.assertRaisesRegex(StartupPoseError, "only valid for current_relative"):
            check_startup_pose(self.profile, self.q, self.angles, strategy="leader_target", session_offset=[0] * 6)

    def test_alignment_reference_uses_paired_homes_and_exact_coordinate_conversion(self):
        ids = [5, 3, 1, 0, 4, 2]
        self.mapping.update(
            joint_map=ids,
            leader_home_deg=[10, 20, 30, 40, 50, 60],
            lumos_home_rad=[0.1, -0.2, 0.3, -0.4, 0.5, -0.6],
        )
        self.session["teleop"].update(joint_map=ids, signs=[1, -1, 1, -1, 1, -1], gain=0.5)
        self.session["robot"].update(startup_tolerance_rad=0.07, startup_joint2_tolerance_rad=0.12)
        self.profile.to_model = lambda index, value: value * (-1 if index == 2 else 1) + 0.01
        self.write_files()
        before = {path: path.read_bytes() for path in self.root.iterdir()}
        target = alignment_reference(self.profile)
        self.assertTrue(target["available"])
        self.assertTrue(target["display_only"])
        self.assertFalse(target["gripper_alignment_required"])
        self.assertEqual(target["source"], "paired_calibration")
        self.assertEqual(target["target_sdk_rad"], self.mapping["lumos_home_rad"])
        self.assertEqual(target["target_urdf_rad"], [
            self.profile.to_model(index, value) for index, value in enumerate(self.mapping["lumos_home_rad"])
        ])
        self.assertEqual(target["leader_home_deg"], [10, 20, 30, 40, 50, 60])
        self.assertEqual(target["leader_ids"], ids)
        self.assertEqual(target["tolerance_rad"], [0.07, 0.12, 0.07, 0.07, 0.07, 0.07])
        self.assertNotIn(str(self.root), json.dumps(target))
        self.assertEqual(before, {path: path.read_bytes() for path in self.root.iterdir()})

    def test_alignment_reference_fails_closed_for_invalid_or_outside_home(self):
        self.mapping["lumos_home_rad"][3] = 2.1
        self.write_files()
        self.assertFalse(alignment_reference(self.profile)["available"])
        self.mapping["lumos_home_rad"][3] = 0
        self.write_files()
        self.session["teleop"]["port"] = "not-paired"
        result = alignment_reference(self.profile)
        self.assertEqual(result, dict(
            available=False, reason="paired_calibration_unavailable", display_only=True,
        ))
        self.assertNotIn(str(self.root), json.dumps(result))
        self.session["teleop"]["port"] = "mock-uart"
        self.profile.to_model = lambda index, value: math.nan
        self.assertFalse(alignment_reference(self.profile)["available"])

    def test_follower_home_mismatch_reports_axis_degrees_and_limit(self):
        self.q[4] = 0.2
        error = self.assert_rejected()
        self.assertIn("J5:", error.detail)
        self.assertIn("Follower-home 11.459156 deg", error.detail)
        self.assertIn("allowed 8.594367 deg", error.detail)
        self.assertEqual(error.diagnostics["strategy"], "home")
        axis = error.diagnostics["axes"][0]
        self.assertEqual(axis["axis"], 5)
        self.assertEqual(axis["failed_checks"], ["follower_home", "pair"])
        self.assertEqual(axis["tolerance_rad"], 0.15)

    def test_leader_target_must_itself_be_near_home(self):
        self.angles[2] = 12
        self.q[2] = math.radians(12)
        error = self.assert_rejected()
        self.assertIn("Leader-target-home 12.000000 deg", error.detail)
        self.assertIn("Follower-target 0.000000 deg", error.detail)
        self.assertEqual(error.diagnostics["axes"][0]["failed_checks"], ["follower_home", "leader_home"])

    def test_each_arm_near_home_does_not_allow_pair_misalignment(self):
        self.q[0] = 0.1
        self.angles[0] = math.degrees(0.1)  # Default J1 sign is -1.
        error = self.assert_rejected()
        self.assertIn("Follower-target 11.459156 deg", error.detail)

    def test_nondefault_map_sign_gain_and_degree_conversion(self):
        joint_map = [5, 3, 1, 0, 4, 2]
        signs = [1, -1, 1, -1, 1, -1]
        self.mapping["joint_map"] = joint_map
        self.mapping["leader_home_deg"] = [10, 20, 30, 40, 50, 60]
        self.mapping["lumos_home_rad"] = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]
        self.session["teleop"].update(joint_map=joint_map, signs=signs, gain=0.5)
        targets = []
        for index, sid in enumerate(joint_map):
            self.angles[sid] = self.mapping["leader_home_deg"][index] + 4
            targets.append(self.mapping["lumos_home_rad"][index] + signs[index] * math.radians(2))
        self.write_files()
        result = check_startup_pose(self.profile, targets + [0.35], self.angles)
        for actual, expected in zip(result["leader_target_sdk"][:6], targets):
            self.assertAlmostEqual(actual, expected)

    def test_calibrated_range_clipping_matches_first_adapter_sample(self):
        self.mapping["leader_range_min_deg"][0] = -2
        self.mapping["leader_range_max_deg"][0] = 2
        self.angles[0] = 100
        self.write_files()
        result = check_startup_pose(self.profile, self.q, self.angles)
        self.assertAlmostEqual(result["leader_target_sdk"][0], math.radians(-2))

    def test_tolerance_boundaries_and_joint2_default(self):
        for axis, limit in ((0, 0.15), (1, 0.4)):
            with self.subTest(axis=axis):
                q = self.q.copy()
                q[axis] = limit
                check_startup_pose(self.profile, q, self.angles)
                q[axis] = math.nextafter(limit, math.inf)
                self.assert_rejected(q=q)

    def test_session_tolerance_overrides(self):
        self.session["robot"].update(startup_tolerance_rad=0.05, startup_joint2_tolerance_rad=0.07)
        self.q[0] = 0.05
        self.q[1] = 0.07
        result = check_startup_pose(self.profile, self.q, self.angles)
        self.assertEqual(result["tolerance_rad"], [0.05, 0.07, 0.05, 0.05, 0.05, 0.05])
        self.q[1] = 0.071
        self.assert_rejected()

    def test_actual_follower_and_mapped_target_must_respect_limits(self):
        self.limits["upper_rad"][0] = 0.05
        self.write_files()
        self.q[0] = 0.06
        self.assertIn("limits [-2, 0.05] rad", self.assert_rejected().detail)
        self.q[0] = 0.0
        self.angles[0] = -4
        self.assert_rejected()

    def test_gripper_home_is_excluded_and_actual_opening_is_anchor(self):
        for opening, leader_angle in ((0, 100), (1, 0), (0.73, -300)):
            with self.subTest(opening=opening):
                self.q[6] = opening
                self.angles[6] = leader_angle
                result = check_startup_pose(self.profile, self.q, self.angles)
                self.assertEqual(result["leader_target_sdk"][6], opening)

    def test_malformed_samples_and_wrong_units_reject(self):
        for value in (math.nan, math.inf, -math.inf, True, "0"):
            with self.subTest(value=value):
                q = self.q.copy()
                q[0] = value
                self.assert_rejected(q=q)
                angles = self.angles.copy()
                angles[6] = value
                self.assert_rejected(angles=angles)
        for q in (self.q[:6], self.q + [0], [5] + self.q[1:], self.q[:6] + [80]):
            with self.subTest(q=q):
                self.assert_rejected(q=q)
        missing = self.angles.copy()
        del missing[6]
        self.assert_rejected(angles=missing)
        self.assert_rejected(angles={str(sid): angle for sid, angle in self.angles.items()})

    def test_malformed_paired_configuration_rejects(self):
        original = deepcopy(self.session)
        for update in (
            {"signs": [1] * 5},
            {"signs": [1, 1, 1, 1, 1, 0]},
            {"gain": math.nan},
            {"joint_map": [0, 1, 2, 3, 5, 4]},
            {"mapping_file": "missing.json"},
            {"port": "other-uart"},
        ):
            with self.subTest(update=update):
                self.session["teleop"] = dict(original["teleop"], **update)
                self.assert_rejected()
        self.session["teleop"] = original["teleop"]
        self.session["robot"]["startup_tolerance_rad"] = 0
        self.assert_rejected()

    def test_current_aligned_accepts_matched_pose_away_from_home_without_writes(self):
        self.profile.engineering = True
        self.q[3] = math.radians(33)
        self.q[6] = 0.0
        self.angles[3] = 33
        before = {path: path.read_bytes() for path in self.root.iterdir()}
        self.assert_rejected()
        result = check_startup_pose(self.profile, self.q, self.angles, strategy="current_aligned")
        self.assertEqual(result["startup_strategy"], "current_aligned")
        self.assertEqual(result["leader_target_sdk"], self.q)
        self.assertGreater(result["follower_home_error_rad"][3], 0.15)
        self.assertEqual(result["pair_error_rad"], [0] * 6)
        self.assertEqual(result["tolerance_rad"], [math.radians(2)] * 6)
        self.assertEqual(before, {path: path.read_bytes() for path in self.root.iterdir()})

    def test_current_aligned_requires_explicit_engineering_profile_and_strategy(self):
        self.q[6] = 0.0
        for engineering in (None, False, 1, "true"):
            with self.subTest(engineering=engineering):
                self.profile.engineering = engineering
                with self.assertRaisesRegex(StartupPoseError, "engineering profile"):
                    check_startup_pose(self.profile, self.q, self.angles, strategy="current_aligned")
        self.profile.engineering = True
        for strategy in (None, "current", True, [], {}):
            with self.subTest(strategy=strategy), self.assertRaises(StartupPoseError):
                check_startup_pose(self.profile, self.q, self.angles, strategy=strategy)

    def test_current_aligned_pair_tolerance_is_capped_by_one_target_step(self):
        self.profile.engineering = True
        self.q[6] = 0.0
        for axis in (0, 1, 3):
            with self.subTest(axis=axis):
                q = self.q.copy()
                q[axis] = math.radians(2)
                check_startup_pose(self.profile, q, self.angles, strategy="current_aligned")
                q[axis] = math.nextafter(q[axis], math.inf)
                with self.assertRaises(StartupPoseError):
                    check_startup_pose(self.profile, q, self.angles, strategy="current_aligned")
        self.session["robot"]["max_target_step_deg"] = 20
        result = check_startup_pose(self.profile, self.q, self.angles, strategy="current_aligned")
        self.assertEqual(result["tolerance_rad"], [math.radians(2)] * 6)
        self.session["robot"]["max_target_step_deg"] = 0.5
        result = check_startup_pose(self.profile, self.q, self.angles, strategy="current_aligned")
        self.assertEqual(result["tolerance_rad"], [math.radians(0.5)] * 6)
        self.session["robot"].update(
            max_target_step_deg=1, startup_tolerance_rad=0.01, startup_joint2_tolerance_rad=0.005,
        )
        result = check_startup_pose(self.profile, self.q, self.angles, strategy="current_aligned")
        self.assertEqual(result["tolerance_rad"], [0.01, 0.005, 0.01, 0.01, 0.01, 0.01])
        for invalid in (0, -1, True, "2", math.nan, math.inf):
            with self.subTest(step=invalid), self.assertRaises(StartupPoseError):
                self.session["robot"]["max_target_step_deg"] = invalid
                check_startup_pose(self.profile, self.q, self.angles, strategy="current_aligned")

    def test_current_aligned_keeps_actual_and_mapped_joint_limits(self):
        self.profile.engineering = True
        self.q[6] = 0.0
        self.limits["upper_rad"][0] = 0.03
        self.write_files()
        self.q[0] = 0.031  # Pair mismatch remains less than the two-degree cap.
        with self.assertRaisesRegex(StartupPoseError, "limits"):
            check_startup_pose(self.profile, self.q, self.angles, strategy="current_aligned")
        self.q[0] = 0
        self.angles[0] = -math.degrees(0.031)
        with self.assertRaisesRegex(StartupPoseError, "limits"):
            check_startup_pose(self.profile, self.q, self.angles, strategy="current_aligned")

    def test_current_aligned_rejects_sdk_j2_and_gripper_initialization_hazards(self):
        self.profile.engineering = True
        self.q[1] = -1.57
        self.q[6] = 0.01
        self.angles[1] = math.degrees(self.q[1])
        check_startup_pose(self.profile, self.q, self.angles, strategy="current_aligned")
        self.q[1] = math.nextafter(-1.57, -math.inf)
        with self.assertRaisesRegex(StartupPoseError, "SDK J2"):
            check_startup_pose(self.profile, self.q, self.angles, strategy="current_aligned")
        self.q[1] = -1.56
        self.angles[1] = math.degrees(-1.58)
        with self.assertRaisesRegex(StartupPoseError, "mapped SDK J2"):
            check_startup_pose(self.profile, self.q, self.angles, strategy="current_aligned")
        self.angles[1] = math.degrees(self.q[1])
        self.q[6] = math.nextafter(0.01, math.inf)
        with self.assertRaisesRegex(StartupPoseError, "SDK construction closes") as raised:
            check_startup_pose(self.profile, self.q, self.angles, strategy="current_aligned")
        self.assertEqual(raised.exception.diagnostics, dict(
            strategy="current_aligned", reason="gripper_not_closed",
            opening=self.q[6], maximum_opening=0.01,
        ))

    def test_current_reference_inverts_original_mapping_without_rebasing(self):
        self.profile.engineering = True
        ids, signs = [5, 3, 1, 0, 4, 2], [1, -1, 1, -1, 1, -1]
        self.mapping.update(
            joint_map=ids,
            leader_home_deg=[10, 20, 30, 40, 50, 60],
            lumos_home_rad=[0.1, -0.2, 0.3, -0.4, 0.5, -0.6],
        )
        self.session["teleop"].update(joint_map=ids, signs=signs, gain=0.5)
        self.profile.to_model = lambda index, value: -value + index * 0.01
        self.q = [home + sign * math.radians(10) for home, sign in zip(self.mapping["lumos_home_rad"], signs)] + [0]
        self.write_files()
        before = {path: path.read_bytes() for path in self.root.iterdir()}
        result = current_alignment_reference(self.profile, self.q)
        self.assertTrue(result["available"])
        self.assertEqual(result["source"], "current_follower_pose")
        self.assertEqual(result["target_sdk_rad"], self.q[:6])
        self.assertEqual(result["target_urdf_rad"], [self.profile.to_model(i, q) for i, q in enumerate(self.q[:6])])
        self.assertEqual(result["leader_ids"], ids)
        for angle, home in zip(result["leader_home_deg"], self.mapping["leader_home_deg"]):
            self.assertAlmostEqual(angle, home + 20)
        self.assertTrue(result["display_only"])
        self.assertNotIn(str(self.root), json.dumps(result))
        self.assertEqual(before, {path: path.read_bytes() for path in self.root.iterdir()})

    def test_current_reference_fails_closed_for_unreachable_invalid_or_unsafe_pose(self):
        self.profile.engineering = True
        self.q[6] = 0
        for q in (
            [1.7] + self.q[1:],  # Within Follower limits, beyond the paired Leader range.
            [0, -1.58, 0, 0, 0, 0, 0],
            self.q[:6] + [0.011],
            [math.nan] + self.q[1:],
            self.q[:6],
        ):
            with self.subTest(q=q):
                result = current_alignment_reference(self.profile, q)
                self.assertEqual(result, dict(
                    available=False, reason="current_follower_pose_unavailable", display_only=True,
                ))
                self.assertNotIn(str(self.root), json.dumps(result))
        self.profile.engineering = False
        self.assertFalse(current_alignment_reference(self.profile, self.q)["available"])
        self.profile.engineering = True
        self.session["teleop"]["port"] = "not-paired"
        self.assertFalse(current_alignment_reference(self.profile, self.q)["available"])
        self.session["teleop"]["port"] = "mock-uart"
        self.profile.to_model = lambda index, value: math.inf
        self.assertFalse(current_alignment_reference(self.profile, self.q)["available"])

    def test_relative_initial_capture_supports_pose_unreachable_by_absolute_mapping(self):
        self.profile.engineering = True
        self.mapping["leader_range_min_deg"][1] = -0.2
        self.write_files()
        q = [0.02, -0.3, 0.01, 0.58, -0.02, -0.15, 0.004]
        before = {path: path.read_bytes() for path in self.root.iterdir()}
        self.assertFalse(current_alignment_reference(self.profile, q)["available"])
        result = check_startup_pose(self.profile, q, self.angles, strategy="current_relative")
        self.assertEqual(result["startup_strategy"], "current_relative")
        self.assertEqual(result["session_offset_sdk_rad"], q[:6])
        self.assertEqual(result["leader_base_target_sdk"], [0] * 6)
        self.assertEqual(result["leader_target_sdk"], q)
        self.assertEqual(result["pair_error_rad"], [0] * 6)
        self.assertEqual(result["tolerance_rad"], [math.radians(2)] * 6)
        self.assertEqual(before, {path: path.read_bytes() for path in self.root.iterdir()})

    def test_relative_fixed_offset_follows_leader_delta_without_recapture(self):
        self.profile.engineering = True
        q = [0, -0.3, 0, 0.58, 0, 0, 0.004]
        initial = check_startup_pose(self.profile, q, self.angles, strategy="current_relative")
        offset = tuple(initial["session_offset_sdk_rad"])
        self.angles[3] = 1
        moved = q.copy()
        moved[3] += math.radians(1)
        checked = check_startup_pose(
            self.profile, moved, self.angles, strategy="current_relative", session_offset=offset,
        )
        self.assertEqual(tuple(checked["session_offset_sdk_rad"]), offset)
        self.assertAlmostEqual(checked["leader_base_target_sdk"][3], math.radians(1))
        self.assertAlmostEqual(checked["leader_target_sdk"][3], moved[3])
        self.assertEqual(checked["leader_target_sdk"][6], moved[6])
        self.angles[3] = 4
        with self.assertRaises(StartupPoseError):
            check_startup_pose(self.profile, moved, self.angles, strategy="current_relative", session_offset=offset)
        self.assertEqual(tuple(initial["session_offset_sdk_rad"]), offset)

    def test_relative_offset_respects_nondefault_mapping_signs_gain_and_homes(self):
        self.profile.engineering = True
        ids, signs = [5, 3, 1, 0, 4, 2], [1, -1, 1, -1, 1, -1]
        homes = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]
        self.mapping.update(joint_map=ids, leader_home_deg=[10, 20, 30, 40, 50, 60], lumos_home_rad=homes)
        self.session["teleop"].update(joint_map=ids, signs=signs, gain=0.5)
        self.write_files()
        angles = {sid: home + 4 for sid, home in zip(ids, self.mapping["leader_home_deg"])}
        angles[6] = -200  # Relative arm offset never rebases the gripper.
        q = [0, -0.3, 0, 0.58, 0, -0.15, 0.004]
        result = check_startup_pose(self.profile, q, angles, strategy="current_relative")
        for index, (home, sign) in enumerate(zip(homes, signs)):
            base = home + sign * math.radians(2)
            self.assertAlmostEqual(result["leader_base_target_sdk"][index], base)
            self.assertAlmostEqual(result["session_offset_sdk_rad"][index], q[index] - base)
            self.assertAlmostEqual(result["leader_target_sdk"][index], q[index])
        self.assertEqual(result["leader_target_sdk"][6], q[6])

    def test_relative_offset_validation_cannot_change_other_strategies(self):
        self.profile.engineering = True
        self.q[6] = 0
        for strategy in ("home", "current_aligned"):
            with self.subTest(strategy=strategy), self.assertRaisesRegex(StartupPoseError, "only valid"):
                check_startup_pose(self.profile, self.q, self.angles, strategy=strategy, session_offset=[0] * 6)
        for invalid in ([0] * 5, [0] * 7, "000000", {}, [True] * 6, [math.nan] * 6, [math.inf] * 6):
            with self.subTest(offset=invalid), self.assertRaises(StartupPoseError):
                check_startup_pose(self.profile, self.q, self.angles, strategy="current_relative", session_offset=invalid)
        self.profile.engineering = False
        with self.assertRaisesRegex(StartupPoseError, "engineering profile"):
            check_startup_pose(self.profile, self.q, self.angles, strategy="current_relative")

    def test_relative_initial_origin_requires_raw_leader_range_but_fixed_offset_keeps_clipping(self):
        self.profile.engineering = True
        self.q[6] = 0
        self.angles[6] = 1000  # Arm-only origin restriction does not rebase the gripper.
        for value in (-90, 90):
            with self.subTest(boundary=value):
                self.angles[0] = value
                check_startup_pose(self.profile, self.q, self.angles, strategy="current_relative")
        for value in (math.nextafter(90, math.inf), math.nextafter(-90, -math.inf), 100):
            with self.subTest(outside=value), self.assertRaisesRegex(StartupPoseError, "raw Leader arm angles"):
                self.angles[0] = value
                check_startup_pose(self.profile, self.q, self.angles, strategy="current_relative")
        self.angles[0] = 100
        self.q[0] = -math.pi / 2
        result = check_startup_pose(
            self.profile, self.q, self.angles, strategy="current_relative", session_offset=(0,) * 6,
        )
        self.assertAlmostEqual(result["leader_base_target_sdk"][0], -math.pi / 2)

    def test_relative_fixed_offset_keeps_joint_limits_pair_cap_and_sdk_hazards(self):
        self.profile.engineering = True
        self.q[6] = 0
        self.session["robot"]["max_target_step_deg"] = 20
        offset = [0] * 6
        offset[0] = math.radians(2)
        check_startup_pose(self.profile, self.q, self.angles, strategy="current_relative", session_offset=offset)
        offset[0] = math.nextafter(offset[0], math.inf)
        with self.assertRaises(StartupPoseError):
            check_startup_pose(self.profile, self.q, self.angles, strategy="current_relative", session_offset=offset)
        self.q[0] = 1.99
        offset[0] = 2.01
        with self.assertRaisesRegex(StartupPoseError, "limits"):
            check_startup_pose(self.profile, self.q, self.angles, strategy="current_relative", session_offset=offset)
        self.q[0] = 2.01
        with self.assertRaisesRegex(StartupPoseError, "limits"):
            check_startup_pose(self.profile, self.q, self.angles, strategy="current_relative")
        self.q[0], self.q[1] = 0, -1.58
        with self.assertRaisesRegex(StartupPoseError, "SDK J2"):
            check_startup_pose(self.profile, self.q, self.angles, strategy="current_relative")
        self.q[1] = -1.56
        offset = [0, -1.58, 0, 0, 0, 0]
        with self.assertRaisesRegex(StartupPoseError, "mapped SDK J2"):
            check_startup_pose(self.profile, self.q, self.angles, strategy="current_relative", session_offset=offset)
        self.q[1], self.q[6] = 0, 0.011
        with self.assertRaisesRegex(StartupPoseError, "SDK construction closes"):
            check_startup_pose(self.profile, self.q, self.angles, strategy="current_relative")

    def test_relative_reference_displays_reachable_leader_origin_without_writes(self):
        self.profile.engineering = True
        self.mapping["leader_range_min_deg"][1] = -0.2
        self.write_files()
        q = [0.02, -0.3, 0, 0.58, -0.02, -0.15, 0.004]
        self.profile.to_model = lambda index, value: -value + index * 0.01
        before = {path: path.read_bytes() for path in self.root.iterdir()}
        result = relative_alignment_reference(self.profile, q)
        self.assertTrue(result["available"])
        self.assertEqual(result["source"], "current_relative_origin")
        self.assertEqual(result["target_sdk_rad"], q[:6])
        self.assertEqual(result["target_urdf_rad"], [self.profile.to_model(i, v) for i, v in enumerate(q[:6])])
        self.assertEqual(result["leader_home_deg"], self.mapping["leader_home_deg"])
        self.assertEqual(result["leader_ids"], self.mapping["joint_map"])
        self.assertTrue(result["display_only"])
        self.assertNotIn("session_offset_sdk_rad", result)  # Actual origin is captured only at startup.
        self.assertNotIn(str(self.root), json.dumps(result))
        self.assertEqual(before, {path: path.read_bytes() for path in self.root.iterdir()})
        self.mapping["leader_home_deg"][1] = -1  # Outside the narrow calibrated range.
        self.write_files()
        self.assertFalse(relative_alignment_reference(self.profile, q)["available"])
        self.mapping["leader_home_deg"][1] = 0
        self.write_files()
        self.assertFalse(relative_alignment_reference(self.profile, q[:6] + [0.011])["available"])
        self.profile.engineering = False
        self.assertFalse(relative_alignment_reference(self.profile, q)["available"])

    def test_pure_gate_never_imports_hardware_sdk(self):
        script = """
import builtins
import json
import sys
from pathlib import Path
from types import SimpleNamespace

original_import = builtins.__import__
def no_hardware_import(name, *args, **kwargs):
    if name.split('.')[0] in {'can', 'serial', 'startouch', 'startouchclass', 'fashionstar_uart_sdk'}:
        raise AssertionError('Hardware SDK import: ' + name)
    return original_import(name, *args, **kwargs)

builtins.__import__ = no_hardware_import
from stararm_lumos_feedback.startup import alignment_reference, check_startup_pose, current_alignment_reference, relative_alignment_reference
root = Path(sys.argv[1])
profile = SimpleNamespace(session=json.loads(sys.argv[2]), paths={'session_file': root / 'session.json'}, to_model=lambda i, q: q)
result = check_startup_pose(profile, [0] * 6 + [0.7], dict.fromkeys(range(7), 0))
assert result['leader_target_sdk'][6] == 0.7
assert alignment_reference(profile)['available']
profile.engineering = True
assert check_startup_pose(profile, [0] * 7, dict.fromkeys(range(7), 0), strategy='current_aligned')['startup_strategy'] == 'current_aligned'
assert current_alignment_reference(profile, [0] * 7)['available']
relative = check_startup_pose(profile, [0, -.3, 0, .58, 0, 0, 0], dict.fromkeys(range(7), 0), strategy='current_relative')
assert relative['session_offset_sdk_rad'][3] == .58
assert relative_alignment_reference(profile, [0, -.3, 0, .58, 0, 0, 0])['available']
print('PURE_GATE_OK')
"""
        result = subprocess.run(
            [sys.executable, "-c", script, str(self.root), json.dumps(self.session)],
            capture_output=True,
            text=True,
            timeout=60,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("PURE_GATE_OK", result.stdout)


if __name__ == "__main__":
    unittest.main()
