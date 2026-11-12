"""Synthetic experimental-output regressions; no hardware or mass claims."""

import math
from copy import deepcopy
from dataclasses import replace
from unittest.mock import patch

from test_feedback_runtime import FixtureCase

from stararm_lumos_feedback.control import Caps
from stararm_lumos_feedback.experimental import EXPERIMENT_RISE_MW_S, ExperimentalDampingController
from stararm_lumos_feedback.transport import Sample


class ExperimentalDampingTests(FixtureCase):
    def setUp(self):
        super().setUp()
        self.profile.engineering = True
        self.profile.mass_confirmed = False
        self.profile.data.update(mass_coverage="UNCONFIRMED", payload=None)
        self.profile.experimental_damping = dict(
            authorized=True, unloaded_confirmed=True, max_mw=200, native_button_required=True,
        )
        self.now, self.sequence = 10.0, 0
        self.caps = Caps()
        self.control = ExperimentalDampingController(self.profile, self.caps, clock=lambda: self.now)

    def frame(self, *, dt=0.02, residual=0.5, velocity=None, positions=None):
        self.now += dt
        self.sequence += 1
        positions = [0] * 6 if positions is None else positions
        velocity = [0] * 7 if velocity is None else velocity
        q = [self.profile.to_model(i, value) for i, value in enumerate(positions)]
        static = self.profile.model.evaluate(q, [0] * 6, [0] * 6) + [0]
        loads = [residual] * 7 if isinstance(residual, (int, float)) else residual
        return tuple(Sample(
            positions[i] if i < 6 else 0.5, velocity[i], static[i] + loads[i], 30,
            self.now, self.sequence,
            -0.7014613048553252 if i == 6 else None, 0 if i == 6 else None,
            self.now + 1000,
        ) for i in range(7))

    def update(self, *, admission=True, **kwargs):
        if admission is True:
            admission = dict(
                allowed=True, target=[0] * 6 + [0.5], target_tolerance=[0.003] * 6 + [0.02],
            )
        return self.control.update(
            self.frame(**kwargs), [0] * 6 + [0.5], [1] * 7, baseline_admission=admission,
        )

    def arm(self):
        for _ in range(150):
            self.update()
            self.assertEqual(self.control.outputs(False)[0], [0] * 7)
            if self.control.snapshot()["baseline_frozen"]:
                return
        self.fail("Synthetic stationary baseline did not qualify")

    def load(self, residual=5, count=30):
        for _ in range(count):
            self.update(residual=residual)

    def test_explicit_unloaded_engineering_contract_and_power_ceiling_required(self):
        for key, value in (("authorized", False), ("unloaded_confirmed", False), ("native_button_required", False), ("max_mw", 201)):
            with self.subTest(key=key):
                original = self.profile.experimental_damping[key]
                self.profile.experimental_damping[key] = value
                with self.assertRaises(ValueError):
                    ExperimentalDampingController(self.profile, self.caps)
                self.profile.experimental_damping[key] = original

    def test_removing_extra_cap_requires_explicit_complete_urdf_and_no_extra_payload(self):
        self.profile.experimental_damping["max_mw"] = None
        for mass_coverage, payload in (
            ("UNCONFIRMED", None), ("explicit_typelj_payload", {"mass_kg": 1}),
            ("urdf_includes_typelj", {"mass_kg": 1}),
        ):
            with self.subTest(mass_coverage=mass_coverage, payload=payload):
                self.profile.data.update(mass_coverage=mass_coverage, payload=payload)
                with self.assertRaisesRegex(ValueError, "explicit URDF TypeLJ coverage"):
                    ExperimentalDampingController(self.profile, self.caps)
        self.profile.data.update(mass_coverage="urdf_includes_typelj", payload=None)
        self.profile.mass_confirmed = True
        control = ExperimentalDampingController(self.profile, self.caps)
        self.assertIsNone(control.snapshot()["ceiling_mw"])
        self.assertTrue(control.snapshot()["mass_confirmed"])
        del self.profile.experimental_damping["max_mw"]
        with self.assertRaisesRegex(ValueError, "explicit unloaded engineering contract"):
            ExperimentalDampingController(self.profile, self.caps)

    def test_invalid_internal_power_is_rejected_before_clamping_to_an_uncapped_axis(self):
        self.profile.data.update(mass_coverage="urdf_includes_typelj", payload=None)
        self.profile.mass_confirmed = True
        self.profile.experimental_damping["max_mw"] = None
        self.control = ExperimentalDampingController(self.profile, self.caps, clock=lambda: self.now)
        for value in (math.nan, math.inf, -math.inf, -1, False, None):
            with self.subTest(value=value):
                self.control.power = [50] * 6 + [value]
                self.control.confirm = [1] * 7
                with self.assertRaisesRegex(RuntimeError, "finite nonnegative values"):
                    self.control.outputs(True)
                self.assertEqual(self.control.outputs(True), ([0] * 7, [False] * 7))
                self.assertEqual(self.control.confirm, [0] * 7)
        for values in ([0] * 6, [0] * 8):
            self.control.power = values
            with self.assertRaises(RuntimeError):
                self.control.outputs(False)
            self.assertEqual(self.control.outputs(False)[0], [0] * 7)

    def test_uncapped_experiment_uses_only_live_common_and_axis_caps_with_configured_rise(self):
        self.profile.data.update(mass_coverage="urdf_includes_typelj", payload=None)
        self.profile.mass_confirmed = True
        self.profile.experimental_damping["max_mw"] = None
        self.control = ExperimentalDampingController(self.profile, self.caps, clock=lambda: self.now)
        expected = [0, 75, 350, 800, 1200, 1200, 1200]
        self.caps.set(dict(common_mw=1200, axis_mw=[0, 75, 350, 800, 1200, 1500, 1200]))
        self.arm()
        previous = [0] * 7
        for _ in range(50):
            self.update(residual=20)
            power, holds = self.control.outputs(True)
            self.assertTrue(all(
                value - old <= policy["rise_mw_s"] * 0.02 + 1e-9
                for value, old, policy in zip(power, previous, self.profile.policy)
            ))
            self.assertTrue(all(value <= limit for value, limit in zip(power, expected)))
            self.assertEqual(holds, [False] * 7)
            previous = power
        self.assertEqual(power, expected)
        state = self.control.snapshot()
        self.assertIsNone(state["ceiling_mw"])
        self.assertEqual(state["effective_caps_mw"], expected)
        self.assertEqual(state["reason"], "mass_confirmed_experimental_damping")
        self.caps.set(dict(common_mw=400, axis_mw=[0, 75, 350, 800, 1200, 1500, 1200]))
        self.assertEqual(self.control.outputs(True), ([0, 75, 350, 400, 400, 400, 400], [False] * 7))
        self.assertEqual(self.control.snapshot()["effective_caps_mw"], [0, 75, 350, 400, 400, 400, 400])
        self.caps.set(dict(common_mw=400, axis_mw=[0, 75, 350, 0, 1200, 1500, 1200]))
        self.assertEqual(self.control.outputs(True), ([0, 75, 350, 0, 400, 400, 400], [False] * 7))

    def test_snapshot_reports_each_effective_rise_with_legacy_experiment_limit_preserved(self):
        configured = [40, 80, 100, 6000, 50, 200, 900]
        for policy, rate in zip(self.profile.policy, configured):
            policy["rise_mw_s"] = rate
        self.assertEqual(self.control.snapshot()["effective_rise_mw_s"], [40, 80, 100, 100, 50, 100, 100])
        self.profile.data.update(mass_coverage="urdf_includes_typelj", payload=None)
        self.profile.mass_confirmed = True
        self.profile.experimental_damping["max_mw"] = None
        self.control = ExperimentalDampingController(self.profile, self.caps, clock=lambda: self.now)
        self.assertEqual(self.control.snapshot()["effective_rise_mw_s"], configured)

    def test_confirmed_uncapped_j4_alone_reaches_1200_in_point_two_seconds_of_output_ramp(self):
        self.profile.data.update(mass_coverage="urdf_includes_typelj", payload=None)
        self.profile.mass_confirmed = True
        self.profile.experimental_damping["max_mw"] = None
        self.profile.policy[3].update(full_scale_nm=1, filter_s=0.0667, confirm_s=0.1, rise_mw_s=6000)
        self.control = ExperimentalDampingController(self.profile, self.caps, clock=lambda: self.now)
        self.arm()
        load = [0.5] * 7
        load[3] = 20
        ramp_started = None
        previous = 0.0
        for _ in range(100):
            before = self.now
            self.update(dt=0.01, residual=load)
            power, holds = self.control.outputs(True)
            self.assertEqual(power[:3] + power[4:], [0] * 6)
            self.assertEqual(holds, [False] * 7)
            self.assertLessEqual(power[3] - previous, 6000 * 0.01 + 1e-9)
            if power[3] > 0 and ramp_started is None:
                ramp_started = before
                self.assertGreaterEqual(self.control.confirm[3] + 1e-9, 0.1)
                self.assertLessEqual(power[3], 60 + 1e-9)
            previous = power[3]
            if power[3] + 1e-9 >= 1200:
                break
        else:
            self.fail("A confirmed fixed J4 load did not reach its configured ordinary cap")
        self.assertIsNotNone(ramp_started)
        self.assertAlmostEqual(self.now - ramp_started, 0.2)
        self.assertAlmostEqual(power[3], 1200)
        self.assertEqual(self.control.snapshot()["effective_rise_mw_s"][3], 6000)

    def test_baseline_uses_two_seconds_of_new_frames_with_zero_output_and_stays_honest(self):
        for _ in range(50):
            self.update()
            self.assertEqual(self.control.outputs(True), ([0] * 7, [False] * 7))
        self.assertFalse(self.control.snapshot()["baseline_frozen"])
        self.arm()
        state = self.control.snapshot()
        self.assertEqual(state["state"], "active")
        self.assertFalse(state["mass_confirmed"])
        self.assertFalse(state["baseline_coverage_accepted"])
        self.assertFalse(state["baseline"]["unloaded_verified"])
        self.assertGreaterEqual(state["baseline"]["retained_stationary_s"] + 1e-9, 2)

    def test_initial_baseline_requires_explicit_admission_and_does_not_learn_while_native_waits(self):
        for admission, reason in (
            (None, "baseline_admission_missing"),
            ({"allowed": False, "reason": "waiting_native_unlock"}, "waiting_native_unlock"),
            ({"allowed": 1, "target": [0] * 7}, "baseline_admission_missing"),
            ({"allowed": True, "target": [0] * 7, "target_tolerance": [math.nan] * 7}, "baseline_admission_missing"),
        ):
            with self.subTest(admission=admission):
                for _ in range(130):
                    self.update(admission=admission)
                    self.assertEqual(self.control.outputs(False), ([0] * 7, [False] * 7))
                state = self.control.snapshot()
                self.assertFalse(state["baseline_frozen"])
                self.assertEqual(state["reason"], reason)
                self.assertEqual(state["baseline"]["pending_stationary_sample_count"], 0)
        self.arm()

    def test_initial_baseline_window_restarts_after_catchup_or_native_inhibition(self):
        for _ in range(70):
            self.update()
        self.assertGreater(self.control.baseline.snapshot()["continuous_stationary_s"], 1)
        self.control.inhibit("native_button_unknown_locked_or_incomplete")
        self.assertEqual(self.control.baseline.snapshot()["pending_stationary_sample_count"], 0)
        for _ in range(130):
            self.update(admission=dict(allowed=False, reason="waiting_follow_target"))
        self.assertFalse(self.control.snapshot()["baseline_frozen"])
        for _ in range(70):
            self.update()
        self.assertFalse(self.control.snapshot()["baseline_frozen"])
        self.arm()

    def test_final_target_must_remain_stable_for_the_complete_initial_window(self):
        for _ in range(70):
            self.update()
        target = [0] * 6 + [0.5]
        target[3] = math.radians(1)
        admission = dict(allowed=True, target=target, target_tolerance=[0.003] * 6 + [0.02])
        self.update(admission=admission)
        self.assertEqual(self.control.snapshot()["reason"], "waiting_stable_follow_target")
        for _ in range(70):
            self.update(admission=admission)
        self.assertFalse(self.control.snapshot()["baseline_frozen"])
        for _ in range(40):
            self.update(admission=admission)
        self.assertTrue(self.control.snapshot()["baseline_frozen"])

    def test_frozen_baseline_ignores_later_admission_changes_and_never_learns_external_load(self):
        self.arm()
        frozen = deepcopy(self.control.snapshot()["baseline"])
        with patch.object(self.control.baseline, "update", wraps=self.control.baseline.update) as observe:
            for _ in range(130):
                self.update(admission=dict(allowed=False, reason="waiting_follow_target"), residual=5)
            self.control.gap()
            self.control.inhibit("native_button_unknown_locked_or_incomplete")
            for _ in range(50):
                self.update(admission=None, residual=5)
        observe.assert_not_called()
        self.assertEqual(self.control.snapshot()["baseline"], frozen)
        self.assertTrue(any(self.control.outputs(False)[0]))

    def test_frozen_absolute_baseline_never_learns_load_or_subtracts_a_fitted_mean(self):
        self.arm()
        before = deepcopy(self.control.snapshot()["baseline"])
        with patch.object(self.control.baseline, "update", wraps=self.control.baseline.update) as observe:
            self.load(residual=0.5)
            self.assertEqual(self.control.outputs(False)[0], [0] * 7)
            self.load(residual=-0.5)
            self.assertEqual(self.control.outputs(False)[0], [0] * 7)
            self.assertTrue(all(abs(value + 0.5) < 1e-8 for value in self.control.telemetry["filtered_residual_nm"]))
            self.load(residual=5)
        observe.assert_not_called()
        self.assertEqual(self.control.snapshot()["baseline"], before)
        self.assertTrue(all(value >= 0.5 for value in self.control.telemetry["deadzone_nm"]))

    def test_real_filtered_acceleration_and_full_arm_state_reach_the_model(self):
        self.update(velocity=[0.1] * 6 + [0])
        samples = self.frame(velocity=[0.2] * 6 + [0], positions=[0.01, -0.02, 0.03, -0.04, 0.05, -0.06])
        with patch.object(self.profile.model, "evaluate", wraps=self.profile.model.evaluate) as model:
            self.control.update(samples, [0] * 7, [0] * 7)
        expected_q = [self.profile.to_model(i, sample.q) for i, sample in enumerate(samples[:6])]
        model.assert_called_once_with(expected_q, [0.2] * 6, self.control.acceleration[:6])
        self.assertTrue(all(abs(value - 5) < 1e-9 for value in self.control.acceleration[:6]))
        self.assertEqual(self.control.telemetry["model_nm"][6], 0)

    def test_power_is_capped_per_axis_zero_is_respected_and_rise_uses_real_time(self):
        self.arm()
        self.caps.set(dict(common_mw=1500, axis_mw=[0] + [1500] * 6))
        previous = [0] * 7
        for _ in range(150):
            self.update(residual=20)
            power, holds = self.control.outputs(True)
            self.assertEqual(power[0], 0)
            self.assertTrue(all(0 <= value <= 200 for value in power))
            self.assertTrue(all(value - old <= EXPERIMENT_RISE_MW_S * 0.02 + 1e-9 for value, old in zip(power, previous)))
            self.assertEqual(holds, [False] * 7)
            self.assertFalse(self.control.hold)
            previous = power
        self.assertEqual(power[1:], [200] * 6)
        self.caps.set(dict(common_mw=400, axis_mw=[0] * 7))
        self.assertEqual(self.control.outputs(False)[0], [0] * 7)

    def test_residuals_drive_only_the_corresponding_axis_and_gripper_is_independent(self):
        self.arm()
        residual = [0.5] * 7
        residual[2] = 5
        self.load(residual=residual)
        power, _ = self.control.outputs(False)
        self.assertGreater(power[2], 0)
        self.assertEqual(power[:2] + power[3:], [0] * 6)
        residual[2], residual[6] = 0.5, 5
        self.load(residual=residual)
        power, _ = self.control.outputs(False)
        self.assertEqual(power[:6], [0] * 6)
        self.assertGreater(power[6], 0)
        self.assertEqual(self.control.telemetry["model_nm"][6], 0)

    def test_filtered_model_range_violation_zeros_output_without_zeroing_model_input(self):
        self.arm()
        self.load()
        self.assertTrue(any(self.control.outputs(False)[0]))
        self.profile.axes[0]["max_acceleration"] = 44
        samples = self.frame(dt=0.001, velocity=[5] + [0] * 6)
        with patch.object(self.profile.model, "evaluate", wraps=self.profile.model.evaluate) as model:
            self.assertFalse(self.control.update(samples, [0] * 7, [0] * 7))
        model.assert_not_called()
        self.assertGreater(self.control.acceleration[0], 44)
        self.assertGreater(self.control.telemetry["acceleration"][0], 44)
        self.assertEqual(self.control.outputs(True), ([0] * 7, [False] * 7))
        self.assertEqual(self.control.confirm, [0] * 7)
        self.assertEqual(self.control.snapshot()["state"], "model_invalid")

    def test_short_quantized_derivative_is_filtered_without_raw_bin_rejection(self):
        self.arm()
        self.profile.axes[3]["max_acceleration"] = 44
        velocity = [0] * 7
        velocity[3] = -(2055 / 4095 * 2 - 1) * 30
        self.assertTrue(self.update(dt=0.00189065933228, velocity=velocity))
        self.assertGreater(abs(self.control.telemetry["raw_acceleration"][3]), 44)
        self.assertLess(abs(self.control.acceleration[3]), 44)

    def test_repeated_stale_and_nonfinite_feedback_cannot_leave_power_or_confirmation_active(self):
        self.arm()
        self.load()
        last = self.control.previous
        self.assertTrue(any(self.control.outputs(False)[0]))
        self.assertFalse(self.control.update(last, [0] * 7, [0] * 7))
        self.assertEqual(self.control.outputs(False)[0], [0] * 7)
        self.assertEqual(self.control.confirm, [0] * 7)
        self.assertEqual(self.control.snapshot()["reason"], "feedback_not_new")
        stale = self.frame()
        self.now += 0.2
        self.assertFalse(self.control.update(stale, [0] * 7, [0] * 7))
        self.assertEqual(self.control.snapshot()["reason"], "feedback_not_fresh")
        invalid = list(self.frame())
        invalid[0] = replace(invalid[0], velocity=math.nan)
        self.control.power = [100] * 7
        with self.assertRaisesRegex(RuntimeError, "finite real feedback"):
            self.control.update(invalid, [0] * 7, [0] * 7)
        self.assertEqual(self.control.outputs(True), ([0] * 7, [False] * 7))

    def test_gap_preserves_frozen_baseline_but_requires_fresh_warmup_and_load_confirmation(self):
        self.arm()
        frozen = deepcopy(self.control.snapshot()["baseline"])
        self.load()
        self.control.gap()
        self.assertEqual(self.control.outputs(False)[0], [0] * 7)
        self.assertEqual(self.control.snapshot()["baseline"], frozen)
        self.assertFalse(self.update(residual=5))
        self.assertEqual(self.control.outputs(False)[0], [0] * 7)
        self.update(dt=0.005, residual=5)
        self.assertEqual(self.control.outputs(False)[0], [0] * 7)
        self.assertEqual(self.control.snapshot()["reason"], "warming_up")
        self.load()
        self.assertTrue(any(self.control.outputs(False)[0]))
        self.assertEqual(self.control.snapshot()["baseline"], frozen)

    def test_stationary_baseline_is_not_qualified_by_motion_or_cached_samples(self):
        for _ in range(120):
            self.update(velocity=[0.2] * 6 + [0])
        self.assertFalse(self.control.snapshot()["baseline_frozen"])
        last = self.control.previous
        for _ in range(120):
            self.control.update(last, [0] * 7, [0] * 7)
        self.assertFalse(self.control.snapshot()["baseline_frozen"])
        self.assertEqual(self.control.outputs(False)[0], [0] * 7)

    def test_native_or_gap_inhibition_preserves_model_and_frozen_baseline_without_force(self):
        self.arm()
        self.load()
        previous = self.control.previous
        acceleration = list(self.control.acceleration)
        baseline = deepcopy(self.control.snapshot()["baseline"])
        self.control.inhibit("waiting_native_unlock")
        self.assertEqual(self.control.outputs(True), ([0] * 7, [False] * 7))
        self.assertEqual(self.control.confirm, [0] * 7)
        self.assertIs(self.control.previous, previous)
        self.assertEqual(self.control.acceleration, acceleration)
        self.assertEqual(self.control.snapshot()["baseline"], baseline)
        self.assertEqual(self.control.snapshot()["reason"], "waiting_native_unlock")

    def test_nonfinite_model_or_filter_state_releases_before_raising(self):
        self.arm()
        self.load()
        samples = self.frame()
        with patch.object(self.profile.model, "evaluate", return_value=[math.nan] * 6):
            with self.assertRaisesRegex(RuntimeError, "Non-finite experimental dynamics"):
                self.control.update(samples, [0] * 7, [0] * 7)
        self.assertEqual(self.control.outputs(False)[0], [0] * 7)
        self.control.filtered = [-1e308] * 7
        samples = self.frame(residual=1e308)
        self.control.power = [100] * 7
        with self.assertRaisesRegex(RuntimeError, "Non-finite experimental residual filter"):
            self.control.update(samples, [0] * 7, [0] * 7)
        self.assertEqual(self.control.outputs(False)[0], [0] * 7)
