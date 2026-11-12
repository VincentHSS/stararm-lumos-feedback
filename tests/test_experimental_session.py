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
        q = [0] * 6 + [0.5]

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
