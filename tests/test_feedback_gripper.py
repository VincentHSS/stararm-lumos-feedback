"""Offline TypeLJ geometry, raw motor guards and SDK-coordinate regressions."""

import json
import math
import unittest
from copy import deepcopy
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from test_feedback_runtime import FakeRobot, FixtureCase, LiveMonitor, dm_frame

from stararm_lumos_feedback.config import GripperMotorError, Profile
from stararm_lumos_feedback.control import Caps
from stararm_lumos_feedback.gripper import TypeLJ04041254
from stararm_lumos_feedback.runtime import Session
from stararm_lumos_feedback.transport import CanMonitor


class GeometryTests(unittest.TestCase):
    def test_native_sdk_reference_positions_are_nonlinear(self):
        # Native pure-converter reference values, without ArmController construction.
        for motor, distance in (
            (0, 0),
            (-0.39489647675608847, 0.02),
            (-0.7014613048553252, 0.04),
            (-0.975894979632439, 0.06),
            (-1.257153, 0.08),
        ):
            with self.subTest(motor=motor):
                self.assertAlmostEqual(TypeLJ04041254.opening(motor), distance / 0.08, places=8)
        self.assertGreater(abs(-0.7014613048553252 / -1.257153 - 0.5), 0.05)

    def test_velocity_is_the_nonlinear_jacobian_not_an_opening_scale(self):
        slopes = []
        for motor in (-1.1, -0.7, -0.2):
            delta = 1e-6
            numerical = (TypeLJ04041254.opening(motor + delta) - TypeLJ04041254.opening(motor - delta)) / (
                2 * delta
            )
            analytic = TypeLJ04041254.jacobian(motor)
            self.assertAlmostEqual(analytic, numerical, places=8)
            self.assertLess(analytic, 0)
            slopes.append(analytic)
        self.assertGreater(max(slopes) - min(slopes), 0.1)
        for invalid in (math.nan, math.inf):
            with self.assertRaises(ValueError):
                TypeLJ04041254.opening(invalid)


class TransformTests(FixtureCase):
    def test_v1_and_unknown_geometry_are_rejected_instead_of_reinterpreted(self):
        with self.assertRaisesRegex(ValueError, "version 2 required"):
            self.rewrite(version=1)
        cfg = dict(self.profile.gripper_transform, type="unknown")
        with self.assertRaisesRegex(ValueError, "Unsupported TypeLJ geometry"):
            self.rewrite(gripper_transform=cfg)
        cfg = dict(self.profile.gripper_transform, max_motor_jump_rad=None)
        with self.assertRaisesRegex(ValueError, "finite number"):
            self.rewrite(gripper_transform=cfg)

    def test_gripper_motor_mapping_and_velocity_preserve_motor_torque_units(self):
        self.assertAlmostEqual(self.profile.to_sdk(6, -0.7014613048553252), 0.5)
        self.assertAlmostEqual(self.profile.to_velocity(6, -0.7, 2), TypeLJ04041254.jacobian(-0.7) * 2)
        self.assertEqual(self.profile.axes[6]["torque_scale"], 1)
        for changes in ({"sdk_scale": -1}, {"sdk_offset": 0.1}):
            axes = deepcopy(self.profile.axes)
            axes[6].update(changes)
            with self.subTest(changes=changes), self.assertRaisesRegex(ValueError, "raw SDK motor radians"):
                self.rewrite(axes=axes)

    def monitor(self):
        self.now, self.wall = 10, 1000
        return CanMonitor(
            self.profile,
            SimpleNamespace(shutdown=lambda: None),
            clock=lambda: self.now,
            wall=lambda: self.wall,
        )

    def test_raw_motor_range_and_velocity_fail_before_clamping(self):
        axis = self.profile.axes[6]
        for position in (0.02, -1.3):
            with self.subTest(position=position), self.assertRaisesRegex(RuntimeError, "raw motor position"):
                self.monitor().ingest(dm_frame(axis, position=position))
        with self.assertRaisesRegex(RuntimeError, "raw motor velocity"):
            self.monitor().ingest(dm_frame(axis, position=0.0003, velocity=15))

    def test_raw_motor_range_error_reports_actual_observed_position_and_unchanged_bounds(self):
        profile = self.profile
        lower = profile.gripper.motor_open_rad - profile.gripper_transform["motor_margin_rad"]
        upper = profile.gripper.motor_closed_rad + profile.gripper_transform["motor_margin_rad"]
        observed = (0x8008 / 65535 * 2 - 1) * 12.5
        self.assertAlmostEqual(observed, 0.003242542153, places=12)
        before = deepcopy((profile.axes, profile.gripper_transform))
        with self.assertRaises(GripperMotorError) as raised:
            profile.check_gripper_motor(observed, 0)
        error = raised.exception
        self.assertIsInstance(error, RuntimeError)
        self.assertEqual(error.code, "gripper_motor_range")
        self.assertIsNone(error.interface)
        self.assertEqual(error.detail["axis"], 7)
        self.assertEqual(error.detail["raw_position_rad"], observed)
        self.assertEqual(error.detail["allowed_min_rad"], lower)
        self.assertEqual(error.detail["allowed_max_rad"], upper)
        self.assertIn("actual 0.00324254215305 rad", str(error))
        self.assertIn("allowed [", str(error))
        self.assertEqual((profile.axes, profile.gripper_transform), before)

    def test_raw_guard_boundaries_remain_inclusive_with_no_range_expansion(self):
        profile = self.profile
        cfg = profile.gripper_transform
        lower = profile.gripper.motor_open_rad - cfg["motor_margin_rad"]
        upper = profile.gripper.motor_closed_rad + cfg["motor_margin_rad"]
        speed = cfg["max_motor_velocity_rad_s"]
        for position in (lower, upper):
            for velocity in (-speed, speed):
                profile.check_gripper_motor(position, velocity)
        for position in (math.nextafter(lower, -math.inf), math.nextafter(upper, math.inf)):
            with self.subTest(position=position), self.assertRaises(GripperMotorError) as raised:
                profile.check_gripper_motor(position, 0)
            self.assertEqual(raised.exception.code, "gripper_motor_range")
        for velocity in (math.nextafter(-speed, -math.inf), math.nextafter(speed, math.inf)):
            with self.subTest(velocity=velocity), self.assertRaises(GripperMotorError) as raised:
                profile.check_gripper_motor(-0.5, velocity)
            detail = raised.exception.detail
            self.assertEqual(raised.exception.code, "gripper_motor_velocity")
            self.assertEqual(detail["raw_velocity_rad_s"], velocity)
            self.assertEqual(detail["allowed_min_rad_s"], -speed)
            self.assertEqual(detail["allowed_max_rad_s"], speed)
            self.assertIn("rad/s", str(raised.exception))

    def test_nonfinite_raw_guard_diagnostics_remain_strict_and_json_safe(self):
        for value in (math.nan, math.inf, -math.inf):
            for is_position in (True, False):
                with self.subTest(value=value, position=is_position), self.assertRaises(
                    GripperMotorError
                ) as raised:
                    self.profile.check_gripper_motor(value if is_position else -0.5, 0 if is_position else value)
                error = raised.exception
                key = "raw_position_rad" if is_position else "raw_velocity_rad_s"
                expected_code = "gripper_motor_range" if is_position else "gripper_motor_velocity"
                self.assertEqual(error.code, expected_code)
                self.assertIsNone(error.detail[key])
                self.assertIn(str(value), str(error))
                json.dumps(error.detail, allow_nan=False)

    def test_raw_motor_jump_does_not_hide_inside_small_normalized_change(self):
        self.profile.gripper_transform["max_motor_jump_rad"] = 0.05
        monitor = self.monitor()
        monitor.ingest(dm_frame(self.profile.axes[6], position=-0.1))
        self.now += 0.01
        self.wall += 0.01
        with self.assertRaisesRegex(RuntimeError, "raw motor position jump"):
            monitor.ingest(dm_frame(self.profile.axes[6], position=-0.3, timestamp=1000.01))

    def test_passive_decoder_converts_velocity_and_keeps_raw_motor_feedback(self):
        monitor = self.monitor()
        monitor.ingest(dm_frame(self.profile.axes[6], position=-0.7, velocity=1, torque=2))
        sample = monitor.samples[6]
        self.assertAlmostEqual(sample.q, TypeLJ04041254.opening(sample.motor_position))
        self.assertAlmostEqual(
            sample.velocity, TypeLJ04041254.jacobian(sample.motor_position) * sample.motor_velocity
        )
        self.assertAlmostEqual(sample.torque, 2, places=2)
        self.assertLess(sample.velocity, 0)
        old = monitor.samples[6]
        monitor.ingest(dm_frame(self.profile.axes[6], position=-0.7, timestamp=1000))
        self.assertEqual(monitor.samples[6], old)

    def session(self):
        robot = FakeRobot()
        session = Session(self.profile, robot, None, Caps(), monitor=LiveMonitor(self.profile))
        return session, robot, session.monitor.snapshot()

    def test_sdk_linear_state_position_is_not_the_public_opening(self):
        session, robot, samples = self.session()
        state = robot.arm.get_gripper_state()
        self.assertNotAlmostEqual(state.position, samples[6].q, places=2)
        session.check_sdk(samples)

    def test_raw_sdk_mismatch_rejected_even_when_public_opening_matches(self):
        session, robot, samples = self.session()
        state = robot.arm.get_gripper_state()
        state.motor_position_rad = -0.3
        robot.arm.get_gripper_state = lambda: state
        with self.assertRaisesRegex(RuntimeError, "raw motor coordinates disagree"):
            session.check_sdk(samples)

    def test_sdk_public_opening_and_freshness_are_checked_separately(self):
        for change, message in (
            ({"distance_m": 0.06}, "public opening coordinates disagree"),
            ({"feedback_age_ms": 200}, "stale"),
            ({"feedback_valid": False}, "invalid"),
            ({"enabled": False}, "disabled"),
            ({"error_code": 8}, "faulted"),
            ({"feedback_age_ms": math.nan}, "stale"),
        ):
            session, robot, samples = self.session()
            state = robot.arm.get_gripper_state()
            state.__dict__.update(change)
            robot.arm.get_gripper_state = lambda: state
            with self.subTest(change=change), self.assertRaisesRegex(RuntimeError, message):
                session.check_sdk(samples)

    def test_sdk_snapshot_does_not_change_can_sequence_or_age(self):
        monitor = self.monitor()
        for i, axis in enumerate(self.profile.axes):
            monitor.ingest(dm_frame(axis, position=-0.7014613048553252 if i == 6 else 0))
        samples = monitor.snapshot()
        session = Session(self.profile, FakeRobot(), None, Caps(), monitor=monitor)
        old = samples[6]
        with patch("stararm_lumos_feedback.runtime.time.monotonic", return_value=self.now):
            session.check_sdk(samples)
        self.assertEqual(monitor.samples[6], old)
        # A valid SDK cache still cannot turn old passive-CAN samples into new ones.
        self.now += 0.2
        self.wall += 0.2
        with self.assertRaisesRegex(RuntimeError, "missing/stale CAN"):
            session.check()
        self.assertEqual(monitor.samples[6], old)


class ClosedObservationTests(FixtureCase):
    """Explicit commissioning evidence affects only the closed-end raw guard."""

    def engineering(self, observation=None, *, include=True, purpose="inspect", p_max=12.5):
        data = deepcopy(self.profile.data)
        data.update(
            version=3,
            mass_coverage="UNCONFIRMED",
            commissioning=dict(
                status="engineering_initial",
                force_authorized=False,
                mass_coverage_inferred=False,
                allowed_modes=["inspect", "follow", "monitor"],
            ),
        )
        data["axes"][6]["p_max"] = p_max
        data["gripper_transform"]["motor_margin_rad"] = 4 * 2 * p_max / 65535
        if include:
            data["commissioning"]["gripper_closed_observation"] = (
                dict(operator_confirmed_closed=True, raw_position_code=0x8008)
                if observation is None else observation
            )
        self.profile.path.write_text(json.dumps(data))
        return Profile(self.profile.path, purpose=purpose)

    @staticmethod
    def motor(code, p_max=12.5):
        return (code / 65535 * 2 - 1) * p_max

    def test_no_observation_keeps_the_original_guard_in_all_profiles(self):
        self.assertEqual(self.profile.gripper_closed_reference_rad, 0)
        with self.assertRaises(GripperMotorError):
            self.profile.check_gripper_motor(self.motor(0x8008), 0)
        for mode in ("inspect", "follow", "monitor"):
            with self.subTest(mode=mode):
                profile = self.engineering(include=False, purpose=mode)
                self.assertEqual(profile.gripper_closed_reference_rad, 0)
                with self.assertRaises(GripperMotorError):
                    profile.check_gripper_motor(self.motor(0x8008), 0)

    def test_confirmed_observation_accepts_only_the_small_closed_extension(self):
        for mode in ("inspect", "follow", "monitor"):
            with self.subTest(mode=mode):
                profile = self.engineering(purpose=mode)
                self.assertEqual(profile.gripper_closed_reference_rad, self.motor(0x8008))
                profile.check_gripper_motor(self.motor(0x8008), 0)
                profile.check_gripper_motor(self.motor(0x800C), 0)
                for motor in (self.motor(0x800D), 0.02, -1.3):
                    with self.subTest(motor=motor), self.assertRaises(GripperMotorError) as raised:
                        profile.check_gripper_motor(motor, 0)
                    self.assertEqual(raised.exception.code, "gripper_motor_range")
                self.assertFalse(profile.mass_confirmed)
                self.assertFalse(profile.commissioning["force_authorized"])
                self.assertFalse(profile.commissioning["mass_coverage_inferred"])

    def test_closed_guard_boundaries_are_inclusive_and_open_end_is_unchanged(self):
        profile = self.engineering()
        margin = profile.gripper_transform["motor_margin_rad"]
        lower = profile.gripper.motor_open_rad - margin
        upper = profile.gripper_closed_reference_rad + margin
        self.assertAlmostEqual(upper, 0.00476844434271762, places=16)
        for position in (lower, upper):
            profile.check_gripper_motor(position, 0)
        for position in (math.nextafter(lower, -math.inf), math.nextafter(upper, math.inf)):
            with self.subTest(position=position), self.assertRaises(GripperMotorError) as raised:
                profile.check_gripper_motor(position, 0)
            self.assertEqual(raised.exception.detail["allowed_min_rad"], lower)
            self.assertEqual(raised.exception.detail["allowed_max_rad"], upper)
        with self.assertRaises(GripperMotorError) as raised:
            profile.check_gripper_motor(self.motor(0x8008), 11)
        self.assertEqual(raised.exception.code, "gripper_motor_velocity")

    def test_observation_object_has_strict_fields_confirmation_and_raw_code_types(self):
        invalid = (
            [], True, "confirmed", {},
            {"operator_confirmed_closed": True},
            {"raw_position_code": 0x8008},
            {"operator_confirmed_closed": True, "raw_position_code": 0x8008, "offset": 0},
        )
        for observation in invalid:
            with self.subTest(observation=observation), self.assertRaises(ValueError):
                self.engineering(observation)
        for confirmed in (False, 0, 1, "true", None, [], {}):
            with self.subTest(confirmed=confirmed), self.assertRaises(ValueError):
                self.engineering(dict(operator_confirmed_closed=confirmed, raw_position_code=0x8008))
        for code in (True, False, 32776.0, "32776", None, -1, 65536, math.nan, math.inf):
            with self.subTest(code=code), self.assertRaises(ValueError):
                self.engineering(dict(operator_confirmed_closed=True, raw_position_code=code))
        # An explicit null object is invalid too; the helper's None means its default object.
        data = deepcopy(self.engineering().data)
        data["commissioning"]["gripper_closed_observation"] = None
        self.profile.path.write_text(json.dumps(data))
        with self.assertRaises(ValueError):
            Profile(self.profile.path, purpose="inspect")

    def test_reference_is_decoded_with_axis7_pmax_and_has_a_16_lsb_software_limit(self):
        profile = self.engineering(p_max=6.25)
        self.assertEqual(profile.gripper_closed_reference_rad, self.motor(0x8008, 6.25))
        observation = dict(operator_confirmed_closed=True, raw_position_code=0x800F)
        profile = self.engineering(observation)
        self.assertLess(profile.gripper_closed_reference_rad, 16 * 25 / 65535)
        for code in (0, 0x7FFF, 0x8010, 65535):
            with self.subTest(code=code), self.assertRaisesRegex(ValueError, "16 position LSBs"):
                self.engineering(dict(operator_confirmed_closed=True, raw_position_code=code))

    def test_run_and_formal_profiles_cannot_use_the_commissioning_observation(self):
        with self.assertRaisesRegex(ValueError, "cannot enable force output"):
            self.engineering(purpose="run")
        data = deepcopy(self.profile.data)
        data["commissioning"] = dict(
            gripper_closed_observation=dict(operator_confirmed_closed=True, raw_position_code=0x8008)
        )
        self.profile.path.write_text(json.dumps(data))
        for mode in ("inspect", "follow", "monitor", "run"):
            with self.subTest(mode=mode), self.assertRaises(ValueError):
                Profile(self.profile.path, purpose=mode)

    def test_observation_changes_neither_opening_velocity_nor_calibration_and_is_bound(self):
        calibration = Path(self.profile.session["robot"]["mapping_file"])
        before = calibration.read_bytes()
        original_axes = deepcopy(self.profile.axes)
        profile = self.engineering()
        self.assertEqual(profile.axes, original_axes)
        for motor in (-1.257153, -0.9, -0.7014613048553252, -0.1, 0, self.motor(0x8008)):
            with self.subTest(motor=motor):
                self.assertEqual(profile.to_sdk(6, motor), self.profile.to_sdk(6, motor))
                self.assertEqual(profile.to_velocity(6, motor, 2), self.profile.to_velocity(6, motor, 2))
        self.assertAlmostEqual(profile.to_sdk(6, self.motor(0x8008)), 0, places=14)
        self.assertEqual(calibration.read_bytes(), before)
        binding = self.root / "closed-observation.binding.json"
        binding.write_text(json.dumps(profile.binding()))
        profile.verify(binding)
        changed = self.engineering(dict(operator_confirmed_closed=True, raw_position_code=0x8009))
        with self.assertRaisesRegex(ValueError, "Binding changed"):
            changed.verify(binding)
        self.assertEqual(calibration.read_bytes(), before)

    def test_sdk_raw_comparison_is_preserved_even_when_both_openings_are_zero(self):
        profile = self.engineering()
        robot = FakeRobot()
        monitor = LiveMonitor(profile)
        session = Session(profile, robot, None, Caps(), monitor=monitor)
        state = robot.arm.get_gripper_state()
        state.motor_position_rad = self.motor(0x8008)
        state.distance_m = 0
        robot.arm.get_gripper_state = lambda: state
        with patch("stararm_lumos_feedback.runtime.time.monotonic", return_value=10):
            samples = list(monitor.snapshot())
            samples[6] = replace(samples[6], q=0, motor_position=self.motor(0x8008))
            session.check_sdk(samples)
            state.motor_position_rad = -0.04
            with self.assertRaisesRegex(RuntimeError, "raw motor coordinates disagree"):
                session.check_sdk(samples)


if __name__ == "__main__":
    unittest.main()
