"""Bound experiment admission and end-to-end software output arbitration."""

import json
import math
import struct
from copy import deepcopy
from types import SimpleNamespace
from unittest.mock import patch

from test_feedback_runtime import ACTION_KEYS, UART, FakeLeader, FakeRobot, FixtureCase, LiveMonitor

from lerobot_robot_lumos_stararm102.button_protocol import CommandButtonState
from stararm_lumos_feedback.config import Profile
from stararm_lumos_feedback.connection import InspectionConnection
from stararm_lumos_feedback.control import Caps
from stararm_lumos_feedback.runtime import LeaderReader, NativeMotionPaused, Session
from stararm_lumos_feedback.transport import Fashionstar, Sample, packet


class ExperimentSessionTests(FixtureCase):
    def setUp(self):
        super().setUp()
        self.data = deepcopy(self.profile.data)
        self.data.update(version=3, mass_coverage="UNCONFIRMED")
        self.data["commissioning"] = dict(
            status="engineering_initial", force_authorized=False, mass_coverage_inferred=False,
            allowed_modes=["inspect", "follow", "monitor", "experiment"],
            experimental_damping=dict(authorized=True, unloaded_confirmed=True, max_mw=200,
                                      native_button_required=True),
        )

    def profile_for(self, purpose="experiment"):
        self.profile.path.write_text(json.dumps(self.data))
        return Profile(self.profile.path, purpose=purpose)

    def test_experiment_is_explicit_and_never_claims_reviewed_force_or_mass(self):
        profile = self.profile_for()
        self.assertFalse(profile.mass_confirmed)
        self.assertFalse(profile.commissioning["force_authorized"])
        with self.assertRaisesRegex(ValueError, "cannot enable force"):
            Profile(profile.path, purpose="run")
        before = profile.binding()
        self.data["commissioning"]["experimental_damping"]["max_mw"] = 100
        self.assertNotEqual(before, self.profile_for().binding())

    def test_missing_unloaded_authorization_native_observer_or_hardcap_blocks_admission(self):
        original = deepcopy(self.data)
        for key, value in (("authorized", False), ("unloaded_confirmed", False),
                           ("native_button_required", False), ("max_mw", 201), ("max_mw", True), ("max_mw", None)):
            with self.subTest(key=key, value=value):
                self.data = deepcopy(original)
                self.data["commissioning"]["experimental_damping"][key] = value
                with self.assertRaises(ValueError):
                    self.profile_for()
        self.data = original
        del self.data["commissioning"]["experimental_damping"]
        with self.assertRaises(ValueError):
            self.profile_for()

    def test_explicit_complete_urdf_allows_null_extra_cap_without_authorizing_formal_run(self):
        capped = self.profile_for().binding()
        self.data.update(mass_coverage="urdf_includes_typelj", payload=None)
        self.data["commissioning"]["experimental_damping"]["max_mw"] = None
        profile = self.profile_for()
        self.assertTrue(profile.mass_confirmed)
        self.assertIsNone(profile.experimental_damping["max_mw"])
        self.assertFalse(profile.commissioning["force_authorized"])
        self.assertFalse(profile.commissioning["mass_coverage_inferred"])
        self.assertNotEqual(profile.binding(), capped)
        with self.assertRaisesRegex(ValueError, "cannot enable force"):
            Profile(profile.path, purpose="run")
        self.data["payload"] = dict(mass_kg=1)
        with self.assertRaisesRegex(ValueError, "payload=null"):
            self.profile_for()

    def test_regular_follow_configuration_does_not_implicitly_gain_experimental_output(self):
        profile = self.profile_for("follow")
        leader, robot = FakeLeader(UART()), FakeRobot()
        session = Session(profile, robot, leader, Caps(), mode="follow", monitor=LiveMonitor(profile))
        self.assertEqual(type(session.controller).__name__, "Controller")
        self.assertIsNone(session.experimental_state())
        profile.experimental_damping = None
        with self.assertRaisesRegex(ValueError, "explicit engineering"):
            Session(profile, robot, leader, Caps(), mode="experiment")

    def test_explicit_web_mode_available_but_formal_run_stays_blocked(self):
        profile = self.profile_for("inspect")
        dashboard = SimpleNamespace(publish=lambda **state: None)
        connection = InspectionConnection(profile, self.root / "test.binding.json", dashboard)
        self.assertIn("experiment", connection.available_modes())
        self.assertNotIn("run", connection.available_modes())
        self.assertEqual(connection.status()["blocked_modes"]["run"], "force_profile_unreviewed")

    def run_experiment(self, released, *, uncapped=False, common_mw=1200):
        if uncapped:
            self.data.update(mass_coverage="urdf_includes_typelj", payload=None)
            self.data["commissioning"]["experimental_damping"]["max_mw"] = None
        profile = self.profile_for()
        uart = UART()
        write = uart.write
        emitted = False

        def inject(frame):
            nonlocal emitted
            result = write(frame)
            if frame[2] == 25 and released and not emitted:
                uart.buffer.extend(b"".join(packet(24, struct.pack("<BBH", i, 0x10, 0)) for i in range(6)))
                emitted = True
            return result

        uart.write = inject

        class Leader(FakeLeader):
            def get_action(self):
                self.leader.send_sync_servo_monitor(list(range(7)))
                return dict(zip(ACTION_KEYS, [0.2] * 6 + [0.5]))

        leader = Leader(uart)
        leader.config.button_enabled = False
        robot = FakeRobot()
        session = Session(profile, robot, leader, Caps(common=common_mw), mode="experiment", monitor=LiveMonitor(profile))
        # Simulate a faulty upper controller: runtime and wire boundaries must
        # still cap every axis and suppress all HOLD requests.
        session.controller.outputs = lambda manual: ([5000] * 7, [True] * 7)
        session.controller.update = lambda *args, **kwargs: True
        with patch("stararm_lumos_feedback.runtime.validate_socketcan"):
            session.run(0.045)
        commands = [struct.unpack("<BBH", f[4:-1]) for f in uart.writes if f[2] == 24]
        return session, robot, commands

    def test_unknown_native_state_with_selector_off_freezes_and_never_writes_arm_controls(self):
        session, robot, commands = self.run_experiment(False)
        self.assertEqual(robot.sent, [])
        self.assertTrue(session.reader.manual_hold)
        self.assertTrue(all(sid == 6 and power == 0 for sid, mode, power in commands))

    def test_real_release_group_allows_follow_but_final_boundary_caps_and_never_holds(self):
        session, robot, commands = self.run_experiment(True)
        self.assertGreater(len(robot.sent), 0)
        self.assertTrue(session.reader.command_button.verified)
        self.assertTrue(any(mode == 0x12 and power == 200 for _, mode, power in commands))
        self.assertTrue(all(power <= 200 and mode != 0x11 for _, mode, power in commands))

    def test_uncapped_experiment_still_obeys_live_common_cap_and_never_requests_hold(self):
        session, robot, commands = self.run_experiment(True, uncapped=True, common_mw=400)
        self.assertTrue(robot.sent)
        self.assertTrue(any(mode == 0x12 and power == 400 for _, mode, power in commands))
        self.assertTrue(all(power <= 400 and mode != 0x11 for _, mode, power in commands))
        self.assertEqual(session.transport.max_output_mw, 1500)
        self.assertFalse(session.transport.allow_hold)
        self.assertIsNone(session.experimental_state()["ceiling_mw"])
        self.assertEqual(session.experimental_state()["effective_caps_mw"], [400] * 7)

    def test_uncapped_experiment_with_unknown_native_state_cannot_write_arm(self):
        session, robot, commands = self.run_experiment(False, uncapped=True)
        self.assertEqual(robot.sent, [])
        self.assertTrue(session.reader.manual_hold)
        self.assertTrue(all(sid == 6 and power == 0 for sid, mode, power in commands))

    def test_runtime_boundary_rejects_invalid_powers_and_applies_per_axis_caps_without_extra_cap(self):
        self.data.update(mass_coverage="urdf_includes_typelj", payload=None)
        self.data["commissioning"]["experimental_damping"]["max_mw"] = None
        profile = self.profile_for()
        caps = Caps(common=1200, axes=[0, 100, 800, 1500, 400, 1200, 500])
        session = Session(profile, FakeRobot(), FakeLeader(UART()), caps, mode="experiment", monitor=LiveMonitor(profile))
        self.assertEqual(session.experimental_powers([5000] * 7), [0, 100, 800, 1200, 400, 1200, 500])
        for value in (math.nan, math.inf, -math.inf, -1, False, None):
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, "finite nonnegative powers"):
                session.experimental_powers([0] * 6 + [value])
        for values in ([0] * 6, [0] * 8):
            with self.subTest(values=values), self.assertRaises(ValueError):
                session.experimental_powers(values)

    def baseline_session(self, *, released=True):
        profile = self.profile_for()
        uart, robot = UART(), FakeRobot()
        robot.config = SimpleNamespace(max_target_step_deg=2)
        robot.limits = profile.limits
        leader = FakeLeader(uart)
        leader.get_action = lambda: dict(zip(ACTION_KEYS, [0] * 6 + [0.5]))
        button = CommandButtonState()
        if released:
            button.feed(b"".join(packet(24, struct.pack("<BBH", i, 0x10, 0)) for i in range(6)))
        session = Session(profile, robot, leader, Caps(), mode="experiment", monitor=LiveMonitor(profile))
        session.transport = Fashionstar(
            uart, range(7), enabled=True, timeout=0.03, command_button=button,
            protect_native_hold=True, max_output_mw=200, allow_hold=False,
        )
        session.reader = LeaderReader(leader, session.transport)
        session._sdk_history_ready = True
        session.desired = [0] * 6 + [0.5]
        return session, uart, robot, button

    def test_initial_baseline_requires_real_release_then_one_complete_follow_send(self):
        session, _, robot, button = self.baseline_session(released=False)
        samples = session.monitor.snapshot()
        for _ in range(3):
            admission = session.initial_baseline_admission(samples)
            self.assertFalse(admission["allowed"])
            self.assertEqual(admission["reason"], "waiting_native_unlock")
        button.feed(b"".join(packet(24, struct.pack("<BBH", i, 0x10, 0)) for i in range(6)))
        self.assertEqual(session.initial_baseline_admission(samples)["reason"], "waiting_first_follow_command")
        session.step()
        self.assertTrue(robot.sent)
        self.assertEqual(session._baseline_completed_send, [0] * 6 + [0.5])
        self.assertTrue(session.initial_baseline_admission(session.monitor.snapshot())["allowed"])

    def test_catchup_and_actual_arrival_gate_initial_baseline_without_changing_targets(self):
        session, _, _, _ = self.baseline_session()
        session._baseline_completed_send = [0] * 6 + [0.5]
        session.desired[3] = 0.4
        before = session.desired.copy()
        self.assertEqual(
            session.initial_baseline_admission(session.monitor.snapshot())["reason"], "waiting_follow_target",
        )
        session._baseline_completed_send[3] = 0.4
        self.assertEqual(
            session.initial_baseline_admission(session.monitor.snapshot())["reason"], "waiting_follower_settled",
        )
        samples = self.samples(1, q=[0, 0, 0, 0.39, 0, 0, 0.5])
        self.assertTrue(session.initial_baseline_admission(samples)["allowed"])
        self.assertEqual(session.desired, before)

    def test_initial_baseline_uses_limit_clipped_j3_target_and_checks_gripper_catchup(self):
        session, _, _, _ = self.baseline_session()
        low, high = session.robot.limits
        session.robot.limits = low, tuple(0 if i == 2 else value for i, value in enumerate(high))
        session.desired[2] = math.radians(0.7)
        session._baseline_completed_send = [0] * 6 + [0.5]
        admission = session.initial_baseline_admission(session.monitor.snapshot())
        self.assertTrue(admission["allowed"])
        self.assertEqual(admission["target"][2], 0)
        self.assertAlmostEqual(session.desired[2], math.radians(0.7))
        session.desired[6] = 0.8
        self.assertEqual(
            session.initial_baseline_admission(session.monitor.snapshot())["reason"], "waiting_follow_target",
        )
        session._baseline_completed_send[6] = 0.8
        self.assertEqual(
            session.initial_baseline_admission(session.monitor.snapshot())["reason"], "waiting_follower_settled",
        )

    def assert_baseline_groups(self, admission, *, arm_reason, gripper_reason):
        self.assertEqual(admission["groups"], {
            "arm": {
                "allowed": arm_reason == "baseline_admission_allowed",
                "reason": arm_reason,
                "axis_indices": list(range(6)),
            },
            "gripper": {
                "allowed": gripper_reason == "baseline_admission_allowed",
                "reason": gripper_reason,
                "axis_indices": [6],
            },
        })

    def test_gripper_static_error_allows_whole_arm_admission_without_widening_tolerances(self):
        session, _, robot, _ = self.baseline_session()
        session._baseline_completed_send = [0] * 6 + [0.5]
        settled_admission = session.initial_baseline_admission(self.samples(0))
        samples = self.samples(1, q=[0] * 6 + [0.4341])
        before = deepcopy((session.desired, session._baseline_completed_send, robot.limits,
                           session.profile.gripper_transform))

        admission = session.initial_baseline_admission(samples)

        self.assertFalse(admission["allowed"])
        self.assertEqual(admission["reason"], "waiting_follower_settled")
        self.assert_baseline_groups(
            admission, arm_reason="baseline_admission_allowed", gripper_reason="waiting_follower_settled",
        )
        self.assertAlmostEqual(admission["arrival_error"][6], -0.0659)
        # The public fixture's bound motor uncertainty maps to >2% opening.
        # Splitting the groups must preserve that existing tolerance too.
        self.assertGreaterEqual(admission["arrival_tolerance"][6], 0.02)
        self.assertEqual(admission["target_tolerance"], settled_admission["target_tolerance"])
        self.assertEqual(admission["arrival_tolerance"], settled_admission["arrival_tolerance"])
        self.assertEqual(admission["blocked_axes"], ["gripper"])
        self.assertEqual((session.desired, session._baseline_completed_send, robot.limits,
                          session.profile.gripper_transform), before)

    def test_one_arm_joint_not_arrived_blocks_all_six_arm_axes_only(self):
        session, _, _, _ = self.baseline_session()
        session._baseline_completed_send = [0] * 6 + [0.5]
        samples = self.samples(1, q=[0, 0, 0, 0.09, 0, 0, 0.5])

        admission = session.initial_baseline_admission(samples)

        self.assertFalse(admission["allowed"])
        self.assertEqual(admission["reason"], "waiting_follower_settled")
        self.assert_baseline_groups(
            admission, arm_reason="waiting_follower_settled", gripper_reason="baseline_admission_allowed",
        )
        self.assertEqual(admission["blocked_axes"], ["J4"])
        self.assertAlmostEqual(admission["arrival_tolerance"][3], math.radians(2))

    def test_target_limiter_catchup_blocks_only_its_own_baseline_group(self):
        for axis, target, arm_reason, gripper_reason in (
            (3, 0.4, "waiting_follow_target", "baseline_admission_allowed"),
            (6, 0.8, "baseline_admission_allowed", "waiting_follow_target"),
        ):
            with self.subTest(axis=axis):
                session, _, _, _ = self.baseline_session()
                session._baseline_completed_send = [0] * 6 + [0.5]
                session.desired[axis] = target

                admission = session.initial_baseline_admission(self.samples(1))

                self.assertFalse(admission["allowed"])
                self.assertEqual(admission["reason"], "waiting_follow_target")
                self.assert_baseline_groups(admission, arm_reason=arm_reason, gripper_reason=gripper_reason)
                self.assertEqual(admission["target_blocked_axes"], ["J4" if axis == 3 else "gripper"])
                self.assertEqual(admission["arrival_blocked_axes"], [])

    def test_common_admission_prerequisites_block_both_groups_with_same_reason(self):
        for condition, reason in (
            ("native_unconfirmed", "waiting_native_unlock"),
            ("no_complete_send", "waiting_first_follow_command"),
            ("partial_send", "baseline_admission_missing"),
            ("missing_feedback", "baseline_admission_missing"),
            ("invalid_limits", "baseline_admission_missing"),
        ):
            with self.subTest(condition=condition):
                session, _, robot, _ = self.baseline_session(released=condition != "native_unconfirmed")
                session._baseline_completed_send = [0] * 6 + [0.5]
                samples = self.samples(1)
                if condition == "no_complete_send":
                    session._baseline_completed_send = None
                elif condition == "partial_send":
                    session._baseline_completed_send = [0] * 6
                elif condition == "missing_feedback":
                    samples = samples[:6]
                elif condition == "invalid_limits":
                    robot.limits = ([0] * 6, [0] * 6)

                admission = session.initial_baseline_admission(samples)

                self.assertFalse(admission["allowed"])
                self.assertEqual(admission["reason"], reason)
                self.assert_baseline_groups(admission, arm_reason=reason, gripper_reason=reason)
                self.assertIsNone(admission["target_error"])
                self.assertIsNone(admission["arrival_error"])

    def test_both_baseline_groups_allow_once_their_own_targets_arrive(self):
        session, _, _, _ = self.baseline_session()
        session._baseline_completed_send = [0] * 6 + [0.5]

        admission = session.initial_baseline_admission(self.samples(1))

        self.assertTrue(admission["allowed"])
        self.assert_baseline_groups(
            admission, arm_reason="baseline_admission_allowed", gripper_reason="baseline_admission_allowed",
        )
        self.assertEqual(admission["blocked_axes"], [])

    def test_baseline_diagnostics_identify_target_and_arrival_axes_in_their_real_units(self):
        session, _, _, _ = self.baseline_session()
        session.desired = [0, 0, 0, 0.4, 0, 0, 0.8]
        session._baseline_completed_send = [0, 0, 0, 0.2, 0, 0, 0.8]
        samples = self.samples(1, q=[0, 0, 0, 0.19, 0, 0, 0.5])
        before = deepcopy((session.desired, session._baseline_completed_send, samples))

        admission = session.initial_baseline_admission(samples)

        self.assertFalse(admission["allowed"])
        self.assertEqual(admission["reason"], "waiting_follow_target")
        self.assertEqual(admission["axis_names"], ["J1", "J2", "J3", "J4", "J5", "J6", "gripper"])
        self.assertEqual(admission["axis_units"], ["rad"] * 6 + ["normalized_opening"])
        self.assertEqual(admission["target"], session.desired)
        self.assertEqual(admission["sent_target"], session._baseline_completed_send)
        self.assertEqual(admission["measured_position"], [sample.q for sample in samples])
        self.assertAlmostEqual(admission["target_error"][3], 0.2)
        self.assertAlmostEqual(admission["arrival_error"][3], -0.01)
        self.assertAlmostEqual(admission["arrival_error"][6], -0.3)
        self.assertAlmostEqual(admission["arrival_tolerance"][3], math.radians(2))
        self.assertEqual(admission["target_blocked_axes"], ["J4"])
        self.assertEqual(admission["arrival_blocked_axes"], ["gripper"])
        self.assertEqual(admission["blocked_axes"], ["J4", "gripper"])
        self.assertEqual((session.desired, session._baseline_completed_send, samples), before)

        # Catching up the sent arm command changes only the existing gate
        # priority: the same measured gripper mismatch still blocks admission.
        session._baseline_completed_send[3] = 0.4
        samples = self.samples(2, q=[0, 0, 0, 0.39, 0, 0, 0.5])
        admission = session.initial_baseline_admission(samples)
        self.assertEqual(admission["reason"], "waiting_follower_settled")
        self.assertEqual(admission["target_blocked_axes"], [])
        self.assertEqual(admission["blocked_axes"], ["gripper"])
        samples = self.samples(3, q=[0, 0, 0, 0.39, 0, 0, 0.79])
        admission = session.initial_baseline_admission(samples)
        self.assertTrue(admission["allowed"])
        self.assertEqual(admission["blocked_axes"], [])
        self.assertEqual(session.controller.outputs(False), ([0] * 7, [False] * 7))

    def test_missing_baseline_admission_data_is_unknown_instead_of_retaining_old_axis_errors(self):
        session, _, _, _ = self.baseline_session()
        session._baseline_completed_send = [0] * 6 + [0.8]
        session.initial_baseline_admission(session.monitor.snapshot())
        session._baseline_completed_send = None

        admission = session.initial_baseline_admission(session.monitor.snapshot())

        self.assertFalse(admission["allowed"])
        self.assertEqual(admission["reason"], "waiting_first_follow_command")
        for key in (
            "target", "sent_target", "measured_position", "target_error", "target_tolerance",
            "arrival_error", "arrival_tolerance", "target_blocked_axes", "arrival_blocked_axes", "blocked_axes",
        ):
            self.assertIsNone(admission[key])
        self.assertEqual(session._baseline_admission_diagnostics, admission)
        json.dumps(admission, allow_nan=False)

    def test_publication_preserves_last_seven_axis_admission_after_adapter_cleanup_without_rechecking(self):
        session, _, robot, _ = self.baseline_session()
        session._baseline_completed_send = [0] * 6 + [0.8]
        session.desired[6] = 0.8
        samples = session.monitor.snapshot()
        published = []
        session.dashboard = SimpleNamespace(publish=lambda **state: published.append(state))
        with patch.object(session.transport, "observe_native_state", wraps=session.transport.observe_native_state) as observe:
            admission = session.initial_baseline_admission(samples)
            self.assertEqual(admission["reason"], "waiting_follower_settled")
            robot.last_action_diagnostics = dict(sent_sdk_rad=[0] * 6)
            session.publish("active", samples)
            # Adapter cleanup clears ordinary diagnostics. The gate snapshot
            # must retain its actual gripper command and measurements.
            robot.last_action_diagnostics = None
            session.leader.last_action_diagnostics = None
            session._baseline_completed_send[6] = 0.9
            session.publish("stopped")
            self.assertEqual(observe.call_count, 1)

        for state in published:
            self.assertEqual(state["baseline_admission"], admission)
            self.assertEqual(state["baseline_admission"]["sent_target"][6], 0.8)
            self.assertEqual(state["baseline_admission"]["measured_position"][6], 0.5)
            self.assertEqual(state["baseline_admission"]["blocked_axes"], ["gripper"])
        self.assertEqual(published[-1]["requested_mw"], [0] * 7)

    def test_native_rx_rechecked_before_baseline_and_partial_action_cannot_qualify(self):
        session, uart, robot, _ = self.baseline_session()
        session._baseline_completed_send = [0] * 6 + [0.5]
        uart.buffer.extend(b"".join(packet(24, struct.pack("<BBH", i, 0x11, 1500)) for i in range(6)))
        self.assertEqual(
            session.initial_baseline_admission(session.monitor.snapshot())["reason"], "waiting_native_unlock",
        )
        self.assertIsNone(session._baseline_completed_send)
        uart.buffer.extend(b"".join(packet(24, struct.pack("<BBH", i, 0x10, 0)) for i in range(6)))

        def partial(action):
            robot._previous_target = [0] * 6
            robot.last_action_diagnostics = dict(sent_sdk_rad=[0] * 6)
            raise NativeMotionPaused

        robot.send_action = partial
        session.step()
        self.assertIsNone(session._baseline_completed_send)
        self.assertEqual(
            session.initial_baseline_admission(session.monitor.snapshot())["reason"], "waiting_first_follow_command",
        )

    def test_real_release_catchup_and_arrival_precede_complete_stationary_baseline(self):
        session, _, _, button = self.baseline_session(released=False)
        sequence, now = 0, 100.0
        session.controller.clock = lambda: now
        # Keep both groups unqualified through the staged startup checks.
        # Otherwise an already-arrived gripper can validly close the initial
        # unloaded window before this test reaches arm arrival.
        q = [0] * 6 + [0.4]

        def advance(count):
            nonlocal sequence, now
            for _ in range(count):
                sequence += 1
                now += 0.02
                theory = session.profile.model.evaluate(q[:6], [0] * 6, [0] * 6) + [0]
                samples = tuple(Sample(
                    q[i], 0, theory[i], 30, now, sequence,
                    -0.7014613048553252 if i == 6 else None, 0 if i == 6 else None,
                ) for i in range(7))
                session.controller.update(
                    samples, session._baseline_completed_send or q, session.desired,
                    baseline_admission=session.initial_baseline_admission(samples),
                )
                self.assertEqual(session.controller.outputs(False), ([0] * 7, [False] * 7))

        advance(130)  # Stationary for >2 s while the real board is unknown.
        self.assertFalse(session.controller.snapshot()["baseline_frozen"])
        button.feed(b"".join(packet(24, struct.pack("<BBH", i, 0x10, 0)) for i in range(6)))
        session.desired[3] = 0.5
        session._baseline_completed_send = [0] * 6 + [0.5]
        advance(130)  # Follower stationary, but limiter has not caught up.
        self.assertEqual(session.controller.snapshot()["reason"], "waiting_follow_target")
        session._baseline_completed_send[3] = 0.5
        advance(130)  # Sent target arrived; measured Follower still has not.
        self.assertEqual(session.controller.snapshot()["reason"], "waiting_follower_settled")
        q[3] = 0.49
        q[6] = 0.5
        advance(70)
        self.assertFalse(session.controller.snapshot()["baseline_frozen"])
        advance(40)
        self.assertTrue(session.controller.snapshot()["baseline_frozen"])

    def test_new_native_lock_during_sdk_read_prevents_the_next_follower_target(self):
        profile = self.profile_for()
        uart, robot = UART(), FakeRobot()
        leader = FakeLeader(uart)
        leader.get_action = lambda: dict(zip(ACTION_KEYS, [0.2] * 6 + [0.5]))
        button = CommandButtonState()
        button.feed(b"".join(packet(24, struct.pack("<BBH", i, 0x10, 0)) for i in range(6)))
        session = Session(profile, robot, leader, Caps(), mode="experiment", monitor=LiveMonitor(profile))
        session.transport = Fashionstar(uart, range(7), enabled=True, timeout=0.03,
                                      command_button=button, protect_native_hold=True, max_output_mw=200)
        session.reader = LeaderReader(leader, session.transport)
        session._sdk_history_ready = True
        lock = b"".join(packet(24, struct.pack("<BBH", i, 0x11, 1500)) for i in range(6))
        session.check_sdk = lambda samples: uart.buffer.extend(lock)
        session.step()
        self.assertTrue(button.locked)
        self.assertTrue(session.reader.manual_hold)
        self.assertEqual(robot.sent, [])
        self.assertTrue(all(struct.unpack("<BBH", f[4:-1])[0] == 6 for f in uart.writes))

    def test_unknown_output_latch_pauses_even_if_parser_has_an_old_release(self):
        profile = self.profile_for()
        leader = FakeLeader(UART())
        session = Session(profile, FakeRobot(), leader, Caps(), mode="experiment", monitor=LiveMonitor(profile))
        button = CommandButtonState()
        button.feed(b"".join(packet(24, struct.pack("<BBH", i, 0x10, 0)) for i in range(6)))
        session.transport = SimpleNamespace(command_button=button, native_output_unknown=True,
                                            observe_native_state=lambda: False)
        session.reader = LeaderReader(leader, session.transport)
        session._sdk_history_ready = True
        self.assertTrue(session.reader.manual_hold)
        with self.assertRaises(NativeMotionPaused):
            session.check_motion_write()

    def test_explicit_status_frequency_schema_preserves_motion_parameters(self):
        from lerobot_robot_lumos_stararm102.cli import load_session

        path = self.root / "session.json"
        data = json.loads(path.read_text())
        data.update(fps=100, status_rate_hz=5)
        data["robot"].update(max_speed_deg_per_sec=100, max_target_step_deg=2)
        path.write_text(json.dumps(data))
        loaded = load_session(path)
        self.assertEqual(loaded["fps"], 100)
        self.assertEqual(loaded["status_rate_hz"], 5)
        self.assertEqual(loaded["robot"]["max_speed_deg_per_sec"], 100)
        for value in (True, 0, 21, math.inf):
            with self.subTest(value=value), self.assertRaises(ValueError):
                data["status_rate_hz"] = value
                path.write_text(json.dumps(data))
                load_session(path)
