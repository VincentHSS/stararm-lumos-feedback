"""Pure monitor-default generation, with no hardware or local data dependencies."""

import json
import math
import unittest
from copy import deepcopy

from stararm_lumos_feedback.commissioning_defaults import generate


def inputs():
    axes = [
        dict(
            feedback_id=i + 17,
            motor_id=i + 1,
            p_max=12.5,
            v_max=10 if i < 3 else 30,
            t_max=28 if i < 3 else 10,
            sdk_scale=1,
            sdk_offset=0,
            urdf_scale=1,
            urdf_offset=0,
            torque_scale=1,
        )
        for i in range(7)
    ]
    session = dict(fps=30, robot=dict(max_speed_deg_per_sec=50, max_target_step_deg=2))
    sdk = dict(
        gripper_control=dict(type="TypeLJ", force_position_max_velocity_rad_s=20),
        pos_vel_mode=dict(
            max_acc_limits=[28, 28, 28, 22, 26, 26], max_dec_limits=[-28, -28, -28, -22, -26, -26]
        ),
    )
    return axes, session, sdk


class CommissioningDefaultTests(unittest.TestCase):
    def test_generation_is_deterministic_does_not_mutate_and_cannot_authorize_force(self):
        data = inputs()
        original = deepcopy(data)
        output = generate(*data)
        self.assertEqual(data, original)
        self.assertEqual(output, generate(*data))
        self.assertEqual(output["commissioning"]["allowed_modes"], ["inspect", "follow", "monitor"])
        self.assertEqual(output["commissioning"]["status"], "engineering_initial")
        self.assertFalse(output["commissioning"]["force_authorized"])
        self.assertFalse(output["commissioning"]["automatic_hold_accepted"])
        self.assertFalse(output["commissioning"]["mass_coverage_inferred"])
        self.assertEqual(output["commissioning"]["baseline_status"], "not_measured")
        self.assertNotIn("mass_coverage", output)
        json.dumps(output, allow_nan=False)

    def test_no_load_margins_are_explicitly_unmeasured_not_invented_from_torque_range(self):
        axes, session, sdk = inputs()
        output = generate(axes, session, sdk)
        for axis, policy in zip(axes, output["policy"]):
            self.assertAlmostEqual(policy["deadzone_nm"], 4 * 2 * axis["t_max"] / 4095)
            self.assertEqual(policy["velocity_margin"], 0)
            self.assertEqual(policy["acceleration_margin"], 0)
            self.assertAlmostEqual(policy["full_scale_nm"], 0.1 * axis["t_max"])
        self.assertIn("no measured friction/model-error", output["commissioning"]["rules"]["deadzone"])

    def test_arm_guards_follow_session_target_rate_and_sdk_acceleration(self):
        axes, session, sdk = inputs()
        fast = generate(axes, session, sdk)
        session["robot"]["max_speed_deg_per_sec"] = 10
        slow = generate(axes, session, sdk)
        self.assertLess(slow["axes"][0]["max_velocity"], fast["axes"][0]["max_velocity"])
        self.assertLess(slow["axes"][0]["max_jump"], fast["axes"][0]["max_jump"])
        self.assertAlmostEqual(fast["axes"][0]["max_acceleration"], 56)
        sdk["pos_vel_mode"]["max_dec_limits"][0] = -40
        self.assertAlmostEqual(generate(axes, session, sdk)["axes"][0]["max_acceleration"], 80)

    def test_protocol_and_transform_units_are_preserved(self):
        axes, session, sdk = inputs()
        axes[0].update(sdk_scale=-0.5, urdf_scale=-1, torque_scale=2)
        output = generate(axes, session, sdk)
        self.assertAlmostEqual(output["axes"][0]["sdk_tolerance"], 8 * 25 / 65535 * 0.5)
        self.assertAlmostEqual(output["policy"][0]["deadzone_nm"], 4 * 56 / 4095 * 2)
        self.assertEqual(output["axes"][0]["torque_scale"], 2)
        self.assertEqual(output["commissioning"]["inputs"]["protocol"][0]["v_max"], 10)

    def test_jaw_guards_use_reviewed_geometry_not_force_position_mode_setting(self):
        axes, session, sdk = inputs()
        first = generate(axes, session, sdk)
        self.assertAlmostEqual(first["gripper_transform"]["motor_margin_rad"], 4 * 25 / 65535)
        self.assertAlmostEqual(first["gripper_transform"]["max_motor_velocity_rad_s"], 1.257153 / (4 / 30))
        sdk["gripper_control"]["force_position_max_velocity_rad_s"] = 0.1
        second = generate(axes, session, sdk)
        self.assertEqual(first["gripper_transform"], second["gripper_transform"])
        self.assertEqual(first["axes"][6], second["axes"][6])

    def test_all_generated_values_satisfy_the_monitor_numeric_contract(self):
        for fps in (10, 30, 100):
            axes, session, sdk = inputs()
            session["fps"] = fps
            output = generate(axes, session, sdk)
            guards = output["guards"]
            self.assertLess(guards["leader_query_s"], guards["leader_gap_s"])
            self.assertGreater(guards["cycle_timeout_s"], 1 / fps + guards["leader_query_s"])
            for axis, policy in zip(output["axes"], output["policy"]):
                self.assertTrue(all(isinstance(v, (int, float)) and math.isfinite(v) for v in axis.values()))
                self.assertTrue(all(math.isfinite(v) and v >= 0 for v in policy.values()))
                self.assertLess(policy["hold_release_error"], policy["hold_error"])
                self.assertLess(policy["hold_release_load_nm"], policy["hold_load_nm"])
                self.assertGreater(axis["max_velocity"], 0)
                self.assertGreater(axis["max_acceleration"], 0)

    def test_unverified_or_incompatible_inputs_fail_instead_of_becoming_defaults(self):
        cases = []
        data = inputs()
        data[0][0]["v_max"] = None
        cases.append(data)
        data = inputs()
        data[0][1].update(feedback_id=17, motor_id=1)
        cases.append(data)
        data = inputs()
        data[0][0]["torque_scale"] = -1
        cases.append(data)
        data = inputs()
        data[0][6]["sdk_offset"] = 0.1
        cases.append(data)
        data = inputs()
        data[1]["fps"] = True
        cases.append(data)
        data = inputs()
        data[1]["fps"] = 5
        cases.append(data)
        data = inputs()
        data[1]["robot"]["max_target_step_deg"] = 0
        cases.append(data)
        data = inputs()
        data[2]["pos_vel_mode"]["max_dec_limits"][0] = 28
        cases.append(data)
        data = inputs()
        data[2]["gripper_control"]["type"] = "TypeFZ"
        cases.append(data)
        for data in cases:
            with self.subTest(data=data), self.assertRaises(ValueError):
                generate(*data)


if __name__ == "__main__":
    unittest.main()
