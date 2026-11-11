"""Independent synthetic regressions for a frozen, local signed reference.

No device SDK is imported and no physical load or force accuracy is claimed.
"""

import math
from copy import deepcopy
from unittest.mock import patch

from test_feedback_runtime import FixtureCase

from stararm_lumos_feedback.control import Caps
from stararm_lumos_feedback.experimental import ExperimentalDampingController
from stararm_lumos_feedback.transport import Sample


class _ResidualRig:
    """Build real model residuals while advancing only an in-memory clock."""

    def __init__(self, profile, *, enabled=True, omit_option=False, mass_confirmed=True):
        self.profile = deepcopy(profile)
        self.profile.engineering = True
        self.profile.mass_confirmed = mass_confirmed
        self.profile.data.update(
            mass_coverage="urdf_includes_typelj" if mass_confirmed else "UNCONFIRMED", payload=None,
        )
        self.profile.experimental_damping = dict(
            authorized=True, unloaded_confirmed=True, max_mw=None if mass_confirmed else 200,
            native_button_required=True,
        )
        if not omit_option:
            self.profile.experimental_damping["local_reference_enabled"] = enabled
        self.profile.policy[3].update(
            full_scale_nm=1.0, filter_s=0.0667, confirm_s=0.1, rise_mw_s=6000,
        )
        self.now, self.sequence = 10.0, 0
        self.caps = Caps()
        self.control = ExperimentalDampingController(self.profile, self.caps, clock=lambda: self.now)
        self.reference = [0.0] * 7
        self.reference[3] = -0.35
        self.home = [0.0] * 6 + [0.5]

    def update(
        self, *, residual=None, positions=None, velocity=None, motor_velocity=0.0,
        torques=None, dt=0.02,
    ):
        self.now += dt
        self.sequence += 1
        positions = self.home.copy() if positions is None else list(positions)
        velocity = [0.0] * 7 if velocity is None else list(velocity)
        residual = self.reference.copy() if residual is None else list(residual)
        q = [self.profile.to_model(i, value) for i, value in enumerate(positions[:6])]
        static = self.profile.model.evaluate(q, [0.0] * 6, [0.0] * 6) + [0.0]
        torque = [a + b for a, b in zip(static, residual)] if torques is None else torques
        samples = tuple(
            Sample(
                positions[i], velocity[i], torque[i], 30, self.now, self.sequence,
                -0.7014613048553252 if i == 6 else None,
                motor_velocity if i == 6 else None, self.now + 1000,
            )
            for i in range(7)
        )
        self.control.update(
            samples, self.home, self.home,
            baseline_admission=dict(
                allowed=True, target=self.home.copy(), target_tolerance=[0.003] * 6 + [0.02],
            ),
        )
        return self.control.outputs(False)[0]

    def freeze(self, references=None):
        references = [self.reference] if references is None else references
        for step in range(150):
            power = self.update(residual=references[step % len(references)])
            if any(power):
                raise AssertionError("Initial reference collection produced Leader output")
            if self.control.snapshot()["baseline_frozen"]:
                return deepcopy(self.control.snapshot()["baseline_groups"])
        raise AssertionError("Synthetic stationary reference did not freeze")

    def run(self, count=40, **kwargs):
        for _ in range(count):
            power = self.update(**kwargs)
        return power

    def local(self):
        return self.control.snapshot()["local_reference"]

    def j4(self, value):
        residual = self.reference.copy()
        residual[3] = value
        return residual


class LocalResidualReferenceTests(FixtureCase):
    def rig(self, **kwargs):
        return _ResidualRig(self.profile, **kwargs)

    def assert_only_j4(self, power):
        self.assertGreater(power[3], 0)
        self.assertEqual(power[:3] + power[4:], [0.0] * 6)

    def test_j4_load_reducing_absolute_residual_still_requests_damping(self):
        rig = self.rig()
        rig.freeze()
        power = rig.run(residual=rig.j4(-0.10))
        self.assert_only_j4(power)
        self.assertLess(abs(rig.control.telemetry["residual_nm"][3]), 0.35)
        self.assertGreater(rig.local()["raw_excess_nm"][3], 0)
        self.assertTrue(rig.local()["used_axes"][3])

    def test_signed_reference_detects_both_sides_and_a_sign_crossing(self):
        for loaded in (-0.10, -0.65, 0.10):
            with self.subTest(loaded=loaded):
                rig = self.rig()
                rig.freeze()
                self.assert_only_j4(rig.run(residual=rig.j4(loaded)))
                self.assertGreater(rig.local()["raw_excess_nm"][3], 0)

    def test_each_frozen_reference_and_small_quantization_noise_stays_zero(self):
        rig = self.rig()
        lsb = 2 * rig.profile.axes[3]["t_max"] / 4095
        references = [rig.j4(-0.35 + offset * lsb) for offset in (-1, 0, 1)]
        rig.freeze(references)
        # These references also stay below the unchanged absolute-channel floor.
        for residual in references + [rig.j4(-0.35 + 3 * lsb), rig.j4(-0.35 - 3 * lsb)]:
            with self.subTest(residual=residual[3]):
                self.assertEqual(rig.run(residual=residual), [0.0] * 7)
                self.assertEqual(rig.local()["raw_excess_nm"], [0.0] * 7)

    def test_original_model_and_signed_raw_residual_are_not_rewritten(self):
        rig = self.rig()
        rig.freeze()
        with patch.object(rig.profile.model, "evaluate", wraps=rig.profile.model.evaluate) as model:
            self.assert_only_j4(rig.run(residual=rig.j4(-0.10)))
        self.assertTrue(model.called)
        for call in model.call_args_list:
            self.assertEqual([len(values) for values in call.args], [6, 6, 6])
        telemetry = rig.control.telemetry
        for actual, measured, predicted in zip(
            telemetry["residual_nm"], telemetry["measured_nm"], telemetry["model_nm"],
        ):
            self.assertAlmostEqual(actual, measured - predicted)
        self.assertAlmostEqual(telemetry["residual_nm"][3], -0.10)
        self.assertAlmostEqual(telemetry["filtered_residual_nm"][3], -0.10, places=4)

    def test_external_load_cannot_update_frozen_reference_or_coverage(self):
        rig = self.rig()
        frozen_groups = rig.freeze()
        before = rig.local()
        self.assert_only_j4(rig.run(count=100, residual=rig.j4(-0.10)))
        self.assertEqual(rig.control.snapshot()["baseline_groups"], frozen_groups)
        after = rig.local()
        for key in (
            "signed_residual_min_nm", "signed_residual_max_nm", "model_min_nm", "model_max_nm",
            "coordinate_min", "coordinate_max",
        ):
            self.assertEqual(after[key], before[key])

    def test_local_freeze_uses_the_latest_continuous_window_not_old_pose_history(self):
        rig = self.rig()
        estimator = rig.control.baseline
        earlier = rig.j4(-0.80)
        theory = rig.profile.model.evaluate([0.0] * 6, [0.0] * 6, [0.0] * 6) + [0.0]
        # Seed an older qualified observation window without authorizing any
        # controller output, then break continuity before the actual startup.
        for _ in range(110):
            rig.now += 0.02
            rig.sequence += 1
            samples = tuple(
                Sample(
                    rig.home[i], 0.0, theory[i] + earlier[i], 30, rig.now, rig.sequence,
                    -0.7014613048553252 if i == 6 else None, 0.0 if i == 6 else None,
                    rig.now + 1000,
                )
                for i in range(7)
            )
            estimator.update(samples, earlier)
        self.assertTrue(estimator.snapshot()["ready"])
        estimator.gap()
        self.assertFalse(estimator.snapshot()["current_window_reference"]["ready"])

        rig.home[0] = 0.4
        rig.freeze()
        history = rig.control.snapshot()["baseline_groups"]["arm"]["baseline"]
        self.assertAlmostEqual(history["signed_residual_min_nm"][3], -0.80)
        self.assertAlmostEqual(history["signed_residual_max_nm"][3], -0.35)
        self.assertAlmostEqual(rig.local()["signed_residual_min_nm"][3], -0.35)
        self.assertAlmostEqual(rig.local()["signed_residual_max_nm"][3], -0.35)
        self.assertAlmostEqual(rig.local()["coordinate_min"][0], 0.4)
        self.assertAlmostEqual(rig.local()["coordinate_max"][0], 0.4)
        # This load is inside the aggregate historical envelope, but outside
        # the current local reference; history must not mask its increment.
        self.assert_only_j4(rig.run(residual=rig.j4(-0.65)))

    def test_any_arm_pose_axis_or_gripper_outside_region_disables_local_channel(self):
        for axis in range(7):
            with self.subTest(axis=axis):
                rig = self.rig()
                rig.freeze()
                positions = rig.home.copy()
                positions[axis] += 1.1 * rig.local()["radius"][axis]
                self.assertEqual(rig.run(residual=rig.j4(-0.10), positions=positions), [0.0] * 7)
                self.assertFalse(rig.local()["eligible"])
                self.assertFalse(rig.local()["in_region"])
                self.assertEqual(rig.local()["excess_nm"], [0.0] * 7)

    def test_pose_radius_is_limited_by_configured_target_step(self):
        self.profile.session["robot"]["max_target_step_deg"] = 0.5
        rig = self.rig()
        rig.freeze()
        self.assertEqual(rig.local()["radius"][:6], [math.radians(0.5)] * 6)
        self.assertAlmostEqual(rig.local()["radius"][6], 0.02)

    def test_absolute_channel_remains_available_outside_the_local_pose_region(self):
        rig = self.rig()
        rig.freeze()
        positions = rig.home.copy()
        positions[5] += 1.1 * rig.local()["radius"][5]
        self.assert_only_j4(rig.run(residual=rig.j4(-1.0), positions=positions))
        self.assertFalse(rig.local()["eligible"])
        self.assertEqual(rig.local()["excess_nm"], [0.0] * 7)

    def test_motion_on_any_axis_cannot_qualify_the_local_channel(self):
        for axis in range(7):
            with self.subTest(axis=axis):
                rig = self.rig()
                rig.freeze()
                velocity = [0.0] * 7
                velocity[axis] = 1.1 * rig.control.baseline.snapshot()["stationary_velocity_threshold"][axis]
                static = rig.profile.model.evaluate([0.0] * 6, [0.0] * 6, [0.0] * 6)
                # Isolate admission from inertial torque changes in this motion-only case.
                with patch.object(rig.profile.model, "evaluate", return_value=static):
                    self.assertEqual(rig.run(residual=rig.j4(-0.10), velocity=velocity), [0.0] * 7)
                self.assertFalse(rig.local()["eligible"])
                self.assertEqual(rig.local()["excess_nm"], [0.0] * 7)

    def test_clamped_opening_cannot_hide_a_moving_raw_gripper_motor(self):
        rig = self.rig()
        rig.freeze()
        speed = 1.1 * 4 * 2 * rig.profile.axes[6]["v_max"] / 4095
        self.assertEqual(rig.run(residual=rig.j4(-0.10), motor_velocity=speed), [0.0] * 7)
        self.assertFalse(rig.local()["eligible"])
        self.assertEqual(rig.local()["excess_nm"], [0.0] * 7)

    def test_returning_from_other_pose_does_not_reuse_global_filter_history(self):
        rig = self.rig()
        rig.freeze()
        positions = rig.home.copy()
        positions[0] += 1.1 * rig.local()["radius"][0]
        # The opposite signed offset stays inside the old absolute floor, but
        # would look loaded on return if the global signed filter were reused.
        self.assertEqual(rig.run(residual=rig.j4(0.35), positions=positions), [0.0] * 7)
        self.assertGreater(rig.control.filtered[3], 0.3)
        for _ in range(40):
            self.assertEqual(rig.update(), [0.0] * 7)
        self.assertEqual(rig.local()["excess_nm"], [0.0] * 7)

    def test_gap_and_native_inhibition_clear_local_confirmation_and_output(self):
        for interruption in ("gap", "inhibit"):
            with self.subTest(interruption=interruption):
                rig = self.rig()
                frozen_groups = rig.freeze()
                self.assert_only_j4(rig.run(residual=rig.j4(-0.10)))
                if interruption == "gap":
                    rig.control.gap()
                else:
                    rig.control.inhibit("waiting_native_unlock")
                self.assertEqual(rig.control.outputs(False)[0], [0.0] * 7)
                self.assertEqual(rig.local()["confirm_s"], [0.0] * 7)
                self.assertEqual(rig.local()["excess_nm"], [0.0] * 7)
                self.assertEqual(rig.local()["warmup_s"], 0.0)
                self.assertEqual(rig.update(residual=rig.j4(-0.10), dt=0.005), [0.0] * 7)
                for _ in range(40):
                    self.assertEqual(rig.update(), [0.0] * 7)
                self.assertEqual(rig.control.snapshot()["baseline_groups"], frozen_groups)
                self.assert_only_j4(rig.run(residual=rig.j4(-0.10)))

    def test_model_change_inside_engineering_region_is_an_extra_noise_allowance(self):
        rig = self.rig()

        def model(q, velocity, acceleration):
            return [0.0, 0.0, 0.0, 1.0 - (0.2 if q[0] > 0.005 else 0.0), 0.0, 0.0]

        with patch.object(rig.profile.model, "evaluate", side_effect=model):
            rig.freeze()
            positions = rig.home.copy()
            positions[0] = 0.01
            torque = [0.0] * 7
            torque[3] = 0.65  # Unchanged measured torque while model torque falls by 0.2 Nm.
            self.assertEqual(rig.run(positions=positions, torques=torque), [0.0] * 7)
            self.assertTrue(rig.local()["eligible"])
            self.assertGreaterEqual(rig.local()["model_allowance_nm"][3] + 1e-12, 0.2)
            self.assertEqual(rig.local()["raw_excess_nm"], [0.0] * 7)
            self.assertAlmostEqual(rig.control.telemetry["residual_nm"][3], -0.15)

    def test_invalid_feedback_immediately_clears_local_output_and_history(self):
        rig = self.rig()
        frozen_groups = rig.freeze()
        self.assert_only_j4(rig.run(residual=rig.j4(-0.10)))
        with self.assertRaisesRegex(RuntimeError, "finite real feedback"):
            rig.update(residual=rig.j4(math.nan))
        self.assertEqual(rig.control.outputs(False)[0], [0.0] * 7)
        self.assertEqual(rig.local()["excess_nm"], [0.0] * 7)
        self.assertEqual(rig.local()["confirm_s"], [0.0] * 7)
        self.assertEqual(rig.local()["warmup_s"], 0.0)
        for _ in range(40):
            self.assertEqual(rig.update(), [0.0] * 7)
        self.assertEqual(rig.control.snapshot()["baseline_groups"], frozen_groups)

    def test_unloading_returns_to_exact_zero_after_a_finite_filter_tail(self):
        rig = self.rig()
        rig.freeze()
        self.assert_only_j4(rig.run(residual=rig.j4(-0.10)))
        for _ in range(50):
            power = rig.update()
            if power == [0.0] * 7 and rig.local()["excess_nm"] == [0.0] * 7:
                break
        else:
            self.fail("Unloaded local distance retained a nonzero damping tail")
        for _ in range(20):
            self.assertEqual(rig.update(), [0.0] * 7)

    def test_local_output_obeys_lowered_and_zero_axis_caps_immediately(self):
        rig = self.rig()
        rig.freeze()
        self.assertGreater(rig.run(residual=rig.j4(-0.10))[3], 50)
        limits = [1200] * 7
        limits[3] = 50
        rig.caps.set(dict(common_mw=400, axis_mw=limits))
        self.assertLessEqual(rig.control.outputs(False)[0][3], 50)
        self.assertLessEqual(rig.run(residual=rig.j4(-0.10))[3], 50)
        limits[3] = 0
        rig.caps.set(dict(common_mw=400, axis_mw=limits))
        self.assertEqual(rig.control.outputs(False)[0], [0.0] * 7)
        self.assertEqual(rig.run(residual=rig.j4(-0.10)), [0.0] * 7)

    def test_local_reference_does_not_bypass_a_larger_configured_deadzone(self):
        rig = self.rig()
        rig.profile.policy[3]["deadzone_nm"] = 0.30
        rig.freeze()
        # The signed change is 0.25 Nm, below the explicitly configured 0.30 Nm deadzone.
        self.assertEqual(rig.run(residual=rig.j4(-0.10)), [0.0] * 7)
        self.assertTrue(rig.local()["eligible"])
        self.assertEqual(rig.local()["raw_excess_nm"], [0.0] * 7)
        self.assertAlmostEqual(rig.local()["effective_noise_margin_nm"][3], 0.30)

    def test_disabled_or_absent_option_keeps_old_absolute_only_behavior(self):
        for options in ({"enabled": False}, {"omit_option": True}):
            with self.subTest(options=options):
                rig = self.rig(**options)
                rig.freeze()
                self.assertEqual(rig.run(residual=rig.j4(-0.10)), [0.0] * 7)
                self.assertFalse(rig.local()["enabled"])
                self.assert_only_j4(rig.run(residual=rig.j4(-1.0)))

    def test_enabling_local_reference_requires_confirmed_mass(self):
        with self.assertRaises(ValueError):
            self.rig(mass_confirmed=False)

    def test_gripper_uses_its_own_absolute_residual_channel(self):
        rig = self.rig()
        rig.freeze()
        residual = rig.reference.copy()
        residual[6] = 1.0
        power = rig.run(residual=residual)
        self.assertEqual(power[:6], [0.0] * 6)
        self.assertGreater(power[6], 0)
        self.assertEqual(rig.local()["raw_excess_nm"][6], 0.0)
        self.assertFalse(rig.local()["used_axes"][6])
