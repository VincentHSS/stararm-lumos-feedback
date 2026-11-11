"""Software-only current-pose startup checks; no SDK or hardware is opened."""

import math
import os
import re
import subprocess
import sys
import threading
import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from test_feedback_adapter_boundary import SCENARIO
from test_feedback_runtime import UART, FakeLeader, FakeRobot, FixtureCase, LiveMonitor

from stararm_lumos_feedback.control import Caps
from stararm_lumos_feedback.motion import create_session, preflight_pose, preflight_session_pose
from stararm_lumos_feedback.runtime import Session, SessionCancelled
from stararm_lumos_feedback.transport import Sample


class CurrentPreflightTests(FixtureCase):
    def setUp(self):
        super().setUp()
        self.profile.engineering = True
        self.profile.guards["startup_feedback_s"] = 1.2

    def probe(self, *, speed=0, drift=0, after_query_drift=0, frozen=False, cancel_at=None, strategy="current_aligned", failure=None):
        clock = SimpleNamespace(now=10.0, query_finished=None)
        cancel = threading.Event()
        profile = self.profile

        class Observer:
            period = 0.02

            def __init__(self, profile):
                self.samples = [None] * 7

            def refresh(self):
                pass

            def ingest(self, message):
                position = drift * (clock.now - 10)
                if clock.query_finished is not None:
                    position += after_query_drift
                stamp = clock.query_finished if frozen and clock.query_finished is not None else clock.now
                self.samples = [dict(
                    q=position if i < 6 else 0,
                    motor_position=position,
                    motor_velocity=speed,
                    stamp=stamp,
                ) for i in range(7)]

            def snapshot(self, *, waiting=False):
                return all(s is not None for s in self.samples)

        def receive(timeout):
            clock.now += max(0.001, timeout)
            if cancel_at is not None and clock.now >= cancel_at:
                cancel.set()
            return object()

        def query(ids):
            clock.query_finished = clock.now
            return {sid: {"angle": 0} for sid in ids}

        bus = SimpleNamespace(recv=receive, shutdown=lambda: None)
        gate_result = dict(startup_strategy=strategy, tolerance_rad=[math.radians(2)] * 6)
        if strategy == "current_relative":
            gate_result["session_offset_sdk_rad"] = [0.3] * 6
        with (
            patch.dict(sys.modules, {"can": SimpleNamespace(Bus=lambda **kwargs: bus)}),
            patch("stararm_lumos_feedback.inspection.Inspection", Observer),
            patch("stararm_lumos_feedback.motion.time.monotonic", side_effect=lambda: clock.now),
            patch("stararm_lumos_feedback.startup.check_startup_pose", return_value=gate_result, side_effect=failure) as gate,
        ):
            result = preflight_pose(profile, cancel, SimpleNamespace(query=query), strategy=strategy)
        return result, clock, gate

    def test_half_second_stability_then_new_group_after_leader_query(self):
        result, clock, gate = self.probe()
        self.assertGreaterEqual(clock.query_finished - 10, 0.5)
        self.assertTrue(all(stamp > clock.query_finished for stamp in result["frame_stamps"]))
        self.assertEqual(result["actual_q"], [0] * 7)
        self.assertEqual(result["sdk_tolerance_rad"], [0.02] * 6)
        self.assertEqual(result["sdk_ready_budget_s"], min(0.1, *(axis["max_age_s"] for axis in self.profile.axes)))
        gate.assert_called_once()
        self.assertEqual(gate.call_args.kwargs, {"strategy": "current_aligned"})

    def test_rejected_pose_keeps_measured_display_coordinates_without_accepting_startup(self):
        from stararm_lumos_feedback.startup import StartupPoseError

        failure = StartupPoseError("misaligned")
        with self.assertRaises(StartupPoseError) as raised:
            self.probe(failure=failure)
        self.assertIs(raised.exception, failure)
        self.assertEqual(failure.display_pose["q_sdk"], [0] * 7)
        self.assertEqual(failure.display_pose["q_urdf"], [self.profile.to_model(i, 0) for i in range(6)])
        self.assertGreaterEqual(failure.display_pose["captured_monotonic"], 10.5)
        self.assertNotIn("startup_reference", failure.display_pose)

    def test_motion_or_drift_never_authorizes_startup(self):
        for options in ({"speed": 0.1}, {"drift": 0.02}, {"after_query_drift": 0.01}, {"frozen": True}):
            with self.subTest(options=options), self.assertRaises(RuntimeError):
                self.probe(**options)

    def test_readiness_budget_includes_gripper_can_age_limit(self):
        self.profile.axes[6]["max_age_s"] = 0.0075
        result, _, _ = self.probe()
        self.assertEqual(result["sdk_ready_budget_s"], 0.0075)

    def test_cancellation_during_stability_stops(self):
        from stararm_lumos_feedback.motion import ConnectionCancelled

        with self.assertRaises(ConnectionCancelled):
            self.probe(cancel_at=10.2)

    def test_relative_keeps_initial_gate_offset_and_requires_same_stability(self):
        result, clock, gate = self.probe(strategy="current_relative")
        self.assertEqual(result["session_offset_sdk_rad"], [0.3] * 6)
        self.assertGreaterEqual(clock.query_finished - 10, 0.5)
        self.assertTrue(all(stamp > clock.query_finished for stamp in result["frame_stamps"]))
        self.assertEqual(gate.call_args.kwargs, {"strategy": "current_relative"})
        for options in ({"speed": 0.1}, {"drift": 0.02}, {"frozen": True}):
            with self.subTest(options=options), self.assertRaises(RuntimeError):
                self.probe(strategy="current_relative", **options)

    def test_leader_target_requires_half_second_stability_before_sdk_enable(self):
        result, clock, gate = self.probe(strategy="leader_target")
        self.assertGreaterEqual(clock.query_finished - 10, 0.5)
        self.assertTrue(all(stamp > clock.query_finished for stamp in result["frame_stamps"]))
        self.assertEqual(gate.call_args.kwargs, {"strategy": "leader_target"})
        for options in ({"speed": 0.1}, {"drift": 0.02}, {"frozen": True}):
            with self.subTest(options=options), self.assertRaises(RuntimeError):
                self.probe(strategy="leader_target", **options)

    def test_reference_is_saved_only_after_successful_preflight(self):
        robot = SimpleNamespace(startup_strategy="current_aligned", startup_reference=None)
        result = {"actual_q": [0] * 7}
        cancel, transport = threading.Event(), object()
        with patch("stararm_lumos_feedback.motion.preflight_pose", return_value=result) as probe:
            self.assertIs(preflight_session_pose(self.profile, cancel, transport, robot=robot), result)
        self.assertIs(robot.startup_reference, result)
        probe.assert_called_once_with(self.profile, cancel, leader_transport=transport, strategy="current_aligned")

    def test_strategy_rejected_before_sdk_verification(self):
        for engineering, mode, strategy in (
            (False, "follow", "current_aligned"), (True, "run", "current_aligned"),
            (False, "monitor", "current_relative"), (True, "run", "current_relative"),
            (False, "follow", "leader_target"), (True, "run", "leader_target"),
            (True, "monitor", "invalid"), (True, "follow", None),
        ):
            with self.subTest(engineering=engineering, mode=mode, strategy=strategy):
                self.profile.engineering = engineering
                with patch("stararm_lumos_feedback.config.verify_sdk_imports") as verify:
                    with self.assertRaises(ValueError):
                        create_session(self.profile, Caps(), None, threading.Event(), mode, startup_pose=strategy)
                verify.assert_not_called()


class CurrentRuntimeTests(FixtureCase):
    def setUp(self):
        super().setUp()
        self.profile.engineering = True
        self.robot = SimpleNamespace()
        self.monitor = SimpleNamespace(error=None, armed=False)
        self.session = Session(self.profile, self.robot, object(), Caps(), mode="follow", monitor=self.monitor)
        now = __import__("time").monotonic()
        self.reference = dict(
            startup_strategy="current_aligned", actual_q=[0] * 7, frame_stamps=[now - 0.01] * 7,
            tolerance_rad=[math.radians(2)] * 6, captured_monotonic=now,
        )
        self.robot.startup_reference = self.reference

    def group(self, q=0, speed=0, *, stale_axis=None):
        now = __import__("time").monotonic()
        return tuple(Sample(
            q if i < 6 else 0, speed, 0, 30,
            self.reference["frame_stamps"][i] if i == stale_axis else now, 1,
            0 if i == 6 else None, 0 if i == 6 else None,
        ) for i in range(7))

    def test_waits_for_every_axis_to_be_new(self):
        groups = [self.group(stale_axis=4)]
        self.monitor.snapshot = lambda: groups.pop() if groups else self.group(q=0.001)
        self.assertEqual(self.session.current_startup_feedback(self.reference), [0.001] * 6 + [0])
        self.assertTrue(self.monitor.armed)

    def test_fixed_reference_drift_motion_and_gripper_opening_rejected(self):
        cases = (
            lambda: self.group(q=0.04), lambda: self.group(speed=0.1),
            lambda: self.group()[:6] + (replace(self.group()[6], q=0.02),),
        )
        for group in cases:
            with self.subTest(group=group), self.assertRaises(RuntimeError):
                self.monitor.snapshot = group
                self.session.current_startup_feedback(self.reference)
        self.assertEqual(self.reference["actual_q"], [0] * 7)

    def test_repeated_axis_has_bounded_timeout(self):
        self.profile.guards["startup_feedback_s"] = 0.01
        self.monitor.snapshot = lambda: self.group(stale_axis=6)
        with self.assertRaisesRegex(RuntimeError, "complete new enabled CAN"):
            self.session.current_startup_feedback(self.reference)

    def test_supplied_readiness_deadline_caps_can_wait_and_rejects_late_groups(self):
        for behavior in ("already_expired", "late_group", "exact_deadline_group", "stale_group", "cancel_wait"):
            with self.subTest(behavior=behavior):
                clock = SimpleNamespace(now=10.0, waits=[], snapshots=0)
                self.reference.update(captured_monotonic=9.999, frame_stamps=[9.998] * 7)
                self.monitor.armed = False
                self.session.stop.clear()

                def snapshot():
                    clock.snapshots += 1
                    if behavior == "late_group":
                        clock.now += 0.021
                    if behavior == "exact_deadline_group":
                        clock.now = deadline
                    return tuple(
                        replace(sample, stamp=clock.now if behavior in ("late_group", "exact_deadline_group") else 9.998)
                        for sample in self.group()
                    )

                def wait(duration):
                    clock.waits.append(duration)
                    clock.now += duration
                    if behavior == "cancel_wait":
                        self.session.stop.set()

                self.monitor.snapshot = snapshot
                expected = SessionCancelled if behavior == "cancel_wait" else RuntimeError
                deadline = 9.999 if behavior == "already_expired" else 10.02
                with (
                    patch("stararm_lumos_feedback.runtime.time", SimpleNamespace(monotonic=lambda: clock.now)),
                    patch.object(self.session.stop, "wait", side_effect=wait),
                    self.assertRaises(expected),
                ):
                    self.session.current_startup_feedback(self.reference, deadline=deadline)
                self.assertFalse(self.monitor.armed)
                self.assertTrue(all(0 <= duration <= 0.005 for duration in clock.waits))
                if behavior == "already_expired":
                    self.assertEqual(clock.snapshots, 0)
                elif behavior in ("late_group", "exact_deadline_group"):
                    self.assertEqual(clock.snapshots, 1)
                    self.assertEqual(clock.waits, [])
                elif behavior == "cancel_wait":
                    self.assertEqual(clock.snapshots, 1)
                    self.assertLessEqual(clock.now, 10.005)
                else:
                    self.assertAlmostEqual(clock.now, deadline)
        self.session.stop.clear()

    def test_supplied_readiness_deadline_rejects_invalid_values_before_can_read(self):
        for deadline in (float("nan"), float("inf"), True, "10"):
            with self.subTest(deadline=deadline), patch.object(self.monitor, "snapshot", create=True) as snapshot:
                with self.assertRaisesRegex(RuntimeError, "finite SDK readiness deadline"):
                    self.session.current_startup_feedback(self.reference, deadline=deadline)
                snapshot.assert_not_called()

    def test_cancellation_and_can_fault_stop_before_acceptance(self):
        self.monitor.error = RuntimeError("synthetic CAN fault")
        self.monitor.snapshot = lambda: (_ for _ in ()).throw(self.monitor.error)
        with self.assertRaisesRegex(RuntimeError, "synthetic CAN fault"):
            self.session.current_startup_feedback(self.reference)
        self.session.stop.set()
        with self.assertRaises(SessionCancelled):
            self.session.current_startup_feedback(self.reference)

    def test_relative_callback_requires_original_frozen_offset(self):
        self.robot.startup_strategy = "current_relative"
        self.reference.update(startup_strategy="current_relative", session_offset_sdk_rad=[-0.3] * 6)
        self.session._session_offset_sdk_rad = (-0.3,) * 6
        self.monitor.snapshot = lambda: self.group(q=0.001)
        self.assertEqual(self.session.current_startup_feedback(self.reference), [0.001] * 6 + [0])
        self.reference["session_offset_sdk_rad"][1] += 0.001
        with self.assertRaisesRegex(RuntimeError, "Invalid current-pose preflight"):
            self.session.current_startup_feedback(self.reference)
        self.assertEqual(self.session._session_offset_sdk_rad, (-0.3,) * 6)

    def test_relative_step_offsets_only_six_axes_before_existing_target_path(self):
        base = {f"joint_{i + 1}.pos": -0.1 for i in range(6)}
        base["gripper.pos"] = 0.008
        offset = (0.05, -0.3, 0.2, 0, -0.1, 0.1)
        self.session._session_offset_sdk_rad = offset
        self.session.leader = SimpleNamespace(get_action=lambda: base)
        self.session.reader = SimpleNamespace(manual_hold=False)
        self.session.transport = SimpleNamespace(output=lambda power, holds: None)
        self.session.check = self.group
        self.session.check_sdk = lambda samples: None
        sent = []
        self.robot.send_action = lambda action: sent.append(action.copy()) or action
        self.session.step()
        expected = [-0.1 + value for value in offset]
        self.assertEqual(self.session.desired[:6], expected)
        self.assertEqual(self.session.desired[6], 0.008)
        self.assertEqual([sent[0][f"joint_{i + 1}.pos"] for i in range(6)], expected)
        self.assertEqual(sent[0]["gripper.pos"], 0)  # Existing first-opening hold remains active.
        self.assertEqual(list(base.values()), [-0.1] * 6 + [0.008])

    def test_leader_target_accepts_real_open_and_moving_gripper_without_replacing_samples(self):
        self.robot.startup_strategy = "leader_target"
        self.reference.update(startup_strategy="leader_target", actual_q=[0] * 6 + [0.5])
        for opening, motor_velocity in ((0.5, 0), (0.02, -1.0), (0.8, 1.0)):
            with self.subTest(opening=opening, motor_velocity=motor_velocity):
                self.monitor.snapshot = lambda: self.group(q=0.001)[:6] + (
                    replace(self.group()[6], q=opening, motor_velocity=motor_velocity),
                )
                actual = self.session.current_startup_feedback(self.reference)
                self.assertEqual(actual, [0.001] * 6 + [opening])
                self.assertEqual(self.reference["actual_q"], [0] * 6 + [0.5])
                self.assertIsNone(self.session._session_offset_sdk_rad)

    def test_leader_target_follow_accepts_post_enable_arm_settling_with_fixed_anchor(self):
        self.robot.startup_strategy = "leader_target"
        self.reference.update(startup_strategy="leader_target")
        self.monitor.snapshot = lambda: self.group(q=0.001, speed=0.1)
        actual = self.session.current_startup_feedback(self.reference)
        self.assertEqual(actual, [0.001] * 6 + [0])
        self.assertEqual(self.reference["actual_q"], [0] * 7)
        self.assertIsNone(self.session._session_offset_sdk_rad)
        self.assertTrue(self.monitor.armed)

    def test_post_enable_settling_exception_requires_direct_engineering_follow(self):
        for engineering, mode, strategy, message in (
            (True, "monitor", "leader_target", "pose is moving"),
            (True, "follow", "current_aligned", "pose is moving"),
            (True, "follow", "current_relative", "pose is moving"),
            (False, "follow", "leader_target", "validated engineering preflight"),
            (1, "follow", "leader_target", "validated engineering preflight"),
            (True, "run", "leader_target", "validated engineering preflight"),
        ):
            with self.subTest(engineering=engineering, mode=mode, strategy=strategy):
                self.profile.engineering = engineering
                self.session.mode = mode
                self.robot.startup_strategy = strategy
                self.reference.update(startup_strategy=strategy, session_offset_sdk_rad=[0] * 6)
                self.session._session_offset_sdk_rad = (0,) * 6
                self.monitor.armed = False
                self.monitor.snapshot = lambda: self.group(q=0.001, speed=0.1)
                with self.assertRaisesRegex(RuntimeError, message):
                    self.session.current_startup_feedback(self.reference)
                self.assertFalse(self.monitor.armed)

    def test_startup_motion_and_drift_failures_report_measurement_and_limit(self):
        self.monitor.snapshot = lambda: self.group(q=0.001, speed=0.1)
        with self.assertRaisesRegex(RuntimeError, "Axis 1: current startup pose is moving") as raised:
            self.session.current_startup_feedback(self.reference)
        self.assertIn("velocity=+0.1 rad/s", str(raised.exception))
        self.assertIn("quiet_limit=", str(raised.exception))
        self.robot.startup_strategy = "leader_target"
        self.reference.update(startup_strategy="leader_target")
        self.monitor.snapshot = lambda: self.group(q=0.04, speed=0.1)
        with self.assertRaisesRegex(RuntimeError, "Axis 1: current startup pose drifted from preflight") as raised:
            self.session.current_startup_feedback(self.reference)
        for field in ("actual=+0.04 rad", "reference=+0 rad", "delta=+0.04 rad", "tolerance="):
            self.assertIn(field, str(raised.exception))

    def test_leader_target_arm_settling_retains_physical_position_limits(self):
        self.robot.startup_strategy = "leader_target"
        self.reference.update(startup_strategy="leader_target")
        self.monitor.snapshot = lambda: self.group(q=2, speed=0.1)
        with self.assertRaisesRegex(RuntimeError, "current startup pose is outside joint limits"):
            self.session.current_startup_feedback(self.reference)
        self.assertFalse(self.monitor.armed)

    def test_leader_target_keeps_arm_drift_and_normalized_gripper_bounds(self):
        self.robot.startup_strategy = "leader_target"
        self.reference.update(startup_strategy="leader_target", actual_q=[0] * 6 + [0.5])
        for opening in (-0.01, 1.01, math.nan, math.inf):
            with self.subTest(opening=opening), self.assertRaises(RuntimeError):
                self.monitor.snapshot = lambda: self.group()[:6] + (
                    replace(self.group()[6], q=opening),
                )
                self.session.current_startup_feedback(self.reference)
        self.monitor.snapshot = lambda: self.group(q=0.04)
        with self.assertRaisesRegex(RuntimeError, "drift"):
            self.session.current_startup_feedback(self.reference)

    def test_leader_target_keeps_original_raw_gripper_and_sdk_coordinate_checks(self):
        robot = FakeRobot()
        robot.startup_strategy = "leader_target"
        self.session.robot = robot
        samples = LiveMonitor(self.profile).snapshot()
        self.session.check_sdk(samples)
        mismatched = samples[:6] + (replace(samples[6], motor_position=-0.2),)
        with self.assertRaisesRegex(RuntimeError, "raw motor coordinates disagree"):
            self.session.check_sdk(mismatched)
        robot.arm.get_gripper_state = lambda: SimpleNamespace(
            distance_m=0.04, motor_position_rad=20.0, motor_velocity_rad_s=0,
            feedback_age_ms=0, feedback_valid=True, enabled=True, error_code=1,
        )
        with self.assertRaisesRegex((ValueError, RuntimeError), "motor|travel|range"):
            self.session.check_sdk(samples)

    def test_leader_target_follows_absolute_leader_during_bounded_enable_settling(self):
        from stararm_lumos_feedback.startup import check_startup_pose

        uart = UART()
        uart.angles[0] = 40
        robot, leader, monitor = FakeRobot(), FakeLeader(uart), LiveMonitor(self.profile)
        robot.startup_strategy = "leader_target"
        original_snapshot = monitor.snapshot
        monitor.snapshot = lambda: tuple(
            replace(sample, velocity=0.1 if i < 6 else sample.velocity)
            for i, sample in enumerate(original_snapshot())
        )
        session = Session(self.profile, robot, leader, Caps(), mode="follow", monitor=monitor)
        references, observed_offsets = [], []
        original_send = robot.send_action

        def send_action(action):
            observed_offsets.append(session._session_offset_sdk_rad)
            return original_send(action)

        def preflight(transport):
            import time

            angles = transport.query(tuple(range(7)))
            result = check_startup_pose(
                self.profile, [0] * 6 + [0.5],
                {sid: sample["angle"] for sid, sample in angles.items()}, strategy="leader_target",
            )
            now = time.monotonic()
            robot.startup_reference = dict(
                result, actual_q=[0] * 6 + [0.5], frame_stamps=[now - 0.001] * 7,
                captured_monotonic=now,
            )
            references.append(robot.startup_reference)

        def initialize():
            robot._previous_target = [0] * 6
            uart.angles[0] = 60

        robot.send_action = send_action
        session.before_enable, robot.connect = preflight, initialize
        session.run(0.02)
        self.assertTrue(robot.sent)
        self.assertAlmostEqual(session.desired[0], math.radians(60))
        self.assertEqual(session.desired[1:6], [0] * 5)
        self.assertTrue(all(offset is None for offset in observed_offsets))
        self.assertIsNone(references[0].get("session_offset_sdk_rad"))
        self.assertEqual(references[0]["actual_q"], [0] * 6 + [0.5])
        self.assertIsNone(session._session_offset_sdk_rad)
        self.assertIsNone(robot.startup_reference)
        self.assertTrue(robot.cleaned and monitor.closed)
        self.assertTrue(all(frame[2] == 25 for frame in uart.writes))

    def test_leader_moved_during_sdk_initialization_never_enters_following(self):
        uart = UART()
        robot, leader, monitor = FakeRobot(), FakeLeader(uart), LiveMonitor(self.profile)
        robot.startup_strategy = "current_aligned"
        original_callback = object()
        robot.after_sdk_startup = original_callback
        original_snapshot = monitor.snapshot
        monitor.snapshot = lambda: tuple(
            replace(sample, q=0, motor_position=0 if i == 6 else None)
            for i, sample in enumerate(original_snapshot())
        )
        session = Session(self.profile, robot, leader, Caps(), mode="follow", monitor=monitor)

        def preflight(transport):
            now = __import__("time").monotonic()
            robot.startup_reference = dict(
                startup_strategy="current_aligned", actual_q=[0] * 7,
                frame_stamps=[now - 0.001] * 7, captured_monotonic=now,
                tolerance_rad=[math.radians(2)] * 6,
            )

        def initialize():
            robot._previous_target = [0] * 6
            uart.angles[0] = 20

        session.before_enable = preflight
        robot.connect = initialize
        with self.assertRaisesRegex(RuntimeError, "pair|aligned|J1"):
            session.run(0.01)
        self.assertEqual(robot.sent, [])
        self.assertTrue(robot.cleaned and monitor.closed)
        self.assertIs(robot.after_sdk_startup, original_callback)
        self.assertIsNone(robot.startup_reference)
        self.assertTrue(all(frame[2] == 25 for frame in uart.writes))

    def test_post_connect_sdk_disagreement_reports_first_axis_values_without_follow_targets(self):
        from stararm_lumos_feedback.startup import check_startup_pose

        self.profile.axes[3]["sdk_tolerance"] = 0.0125
        for strategy in ("current_aligned", "current_relative", "leader_target"):
            with self.subTest(strategy=strategy):
                uart = UART()
                robot, leader, monitor = FakeRobot(), FakeLeader(uart), LiveMonitor(self.profile)
                robot.startup_strategy = strategy
                robot.positions[3], robot.positions[5] = -0.03, 0.04
                original_snapshot = monitor.snapshot
                monitor.snapshot = lambda: tuple(
                    replace(sample, q=0, motor_position=0 if i == 6 else None)
                    for i, sample in enumerate(original_snapshot())
                )
                session = Session(self.profile, robot, leader, Caps(), mode="follow", monitor=monitor)

                def preflight(transport):
                    import time

                    angles = transport.query(tuple(range(7)))
                    result = check_startup_pose(
                        self.profile, [0] * 7,
                        {sid: sample["angle"] for sid, sample in angles.items()}, strategy=strategy,
                    )
                    now = time.monotonic()
                    robot.startup_reference = dict(
                        result, actual_q=[0] * 7, frame_stamps=[now - 0.001] * 7,
                        captured_monotonic=now,
                    )

                robot.connect = lambda: setattr(robot, "_previous_target", [0] * 6)
                session.before_enable = preflight
                with self.assertRaisesRegex(RuntimeError, "stage=after_leader_recheck, J4:") as raised:
                    session.run(0.01)
                self.assertIn("sdk_all_zero=False", str(raised.exception))
                match = re.search(
                    r"SDK=([^ ]+) rad, CAN=([^ ]+) rad, delta\(SDK-CAN\)=([^ ]+) rad, tolerance=([^ ]+) rad",
                    str(raised.exception),
                )
                self.assertIsNotNone(match)
                for actual, expected in zip(map(float, match.groups()), (-0.03, 0, -0.03, 0.0125)):
                    self.assertAlmostEqual(actual, expected, places=12)
                self.assertEqual(robot.sent, [])
                self.assertTrue(robot.cleaned and monitor.closed)
                self.assertFalse(uart.is_open)
                self.assertTrue(all(frame[2] == 25 for frame in uart.writes))

    def test_relative_startup_keeps_offset_through_post_init_and_clears_on_stop(self):
        from stararm_lumos_feedback.startup import check_startup_pose

        for move_leader in (False, True):
            with self.subTest(move_leader=move_leader):
                uart = UART()
                robot, leader, monitor = FakeRobot(), FakeLeader(uart), LiveMonitor(self.profile)
                robot.startup_strategy = "current_relative"
                robot.positions = [-0.3] * 6
                robot.gripper = 0
                robot.arm.get_gripper_state = lambda: SimpleNamespace(
                    distance_m=0, motor_position_rad=0, motor_velocity_rad_s=0,
                    feedback_age_ms=0, feedback_valid=True, enabled=True, error_code=1,
                )
                original_snapshot = monitor.snapshot
                monitor.snapshot = lambda: tuple(
                    replace(sample, q=-0.3 if i < 6 else 0, motor_position=0 if i == 6 else None)
                    for i, sample in enumerate(original_snapshot())
                )
                session = Session(self.profile, robot, leader, Caps(), mode="follow", monitor=monitor)
                references = []

                def preflight(transport):
                    import time

                    angles = transport.query(tuple(range(7)))
                    result = check_startup_pose(
                        self.profile, [-0.3] * 6 + [0],
                        {sid: sample["angle"] for sid, sample in angles.items()},
                        strategy="current_relative",
                    )
                    now = time.monotonic()
                    robot.startup_reference = dict(
                        result, actual_q=[-0.3] * 6 + [0], frame_stamps=[now - 0.001] * 7,
                        captured_monotonic=now,
                    )
                    references.append(robot.startup_reference)

                def initialize():
                    robot._previous_target = [-0.3] * 6
                    if move_leader:
                        uart.angles[0] = 20

                session.before_enable, robot.connect = preflight, initialize
                if move_leader:
                    with self.assertRaises(RuntimeError):
                        session.run(0.01)
                    self.assertEqual(robot.sent, [])
                else:
                    session.run(0.01)
                    self.assertTrue(robot.sent)
                    self.assertEqual([robot.sent[0][f"joint_{i + 1}.pos"] for i in range(6)], [-0.3] * 6)
                self.assertEqual(references[0]["session_offset_sdk_rad"], [-0.3] * 6)
                self.assertIsNone(session._session_offset_sdk_rad)
                self.assertIsNone(robot.startup_reference)
                self.assertTrue(robot.cleaned and monitor.closed)


ADAPTER_SCENARIO = SCENARIO.split("class Manager:", 1)[0] + r'''
import copy
import re
import time
from contextlib import contextmanager
from stararm_lumos_feedback.runtime import SessionCancelled
mode = sys.argv[1]
relative=mode.startswith("relative_")
events=[]
clock=types.SimpleNamespace(now=10.0, sleeps=[])
def sleep(duration):
    global cancelled
    clock.now+=duration
    clock.sleeps.append(duration)
    if mode.endswith("readiness_cancel_wait"): cancelled=True
robot_module=sys.modules["lerobot_robot_lumos_stararm102.lumos"]
robot_module.time=types.SimpleNamespace(monotonic=lambda: clock.now, sleep=sleep)
class Arm:
    instances=[]
    def __init__(self, **kwargs):
        self.commands=[]; self.cleaned=False; self.instances.append(self)
    def get_joint_positions(self):
        events.append("getter")
        positions=[-.4 + (.03 if mode == "sdk_disagree" else 0)]*6
        if mode.endswith("sdk_all_zero"): positions=[0.0]*6
        if "readiness_recovers" in mode and events.count("getter")<=2: positions=[0.0]*6
        if mode.endswith(("readiness_cancel_wait","readiness_drift_wait","readiness_limits_wait","readiness_callback_timeout_after_mismatch")): positions=[0.0]*6
        if mode.endswith("readiness_zero_matched"): positions=[0.0]*6
        if mode.endswith("readiness_delayed_getter"): clock.now+=.101
        if mode.endswith("readiness_exact_deadline_getter"): clock.now+=.1
        if mode.endswith("sdk_disagree_axis4") and ("after_seed" not in mode or events.count("getter")>1):
            positions[3],positions[5]=-.43,-.46
        return positions
    def set_joint_raw(self, q, velocities):
        events.append("seed")
        self.commands.append((q.copy(),velocities.copy()))
        return mode != "rejected_seed"
    def cleanup(self): self.cleaned=True
sdk=types.ModuleType("startouchclass"); sdk.SingleArm=Arm
sys.modules["startouchclass"]=sdk
with tempfile.TemporaryDirectory() as folder:
    root=Path(folder); profile=configuration(root)
    rc=LumosConfig(sdk_root=str(root/"sdk"),mapping_file=str(root/"calibration.json"),
        joint_limits_file=str(root/"limits.json"),can_interface="mock-can",execute=True,startup_wait_s=.001)
    robot=Lumos(rc); robot.startup_strategy="current_relative" if relative else "current_aligned"
    now=clock.now
    reference=dict(startup_strategy="current_aligned",actual_q=[-.4]*6+[0],
        frame_stamps=[now-.01]*7,captured_monotonic=now,valid_until=now+5,
        tolerance_rad=[math.radians(2)]*6,sdk_tolerance_rad=[.02]*6,sdk_ready_budget_s=.1)
    if mode.endswith("sdk_disagree_axis4"): reference["sdk_tolerance_rad"][3]=.0125
    if relative:
        reference["startup_strategy"]="current_relative"
        reference["session_offset_sdk_rad"]=[-.4]*6
    if mode=="relative_missing_offset": reference.pop("session_offset_sdk_rad")
    if mode.endswith("readiness_zero_matched"):
        reference["actual_q"]=[0]*7
        if relative: reference["session_offset_sdk_rad"]=[0]*6
    if mode.endswith("readiness_expiry_callback"): reference["valid_until"]=now+.012
    if mode.endswith("readiness_short_budget_callback"): reference["sdk_ready_budget_s"]=.008
    if mode.startswith("invalid_budget_"):
        reference["sdk_ready_budget_s"]={"zero":0,"negative":-.001,"nan":float("nan"),
            "infinity":float("inf"),"too_large":.101,"boolean":True}[mode.removeprefix("invalid_budget_")]
    robot.startup_reference=reference
    cancelled=False
    calls=[]
    deadlines=[]
    guard_calls_after_can=0
    def callback(ref, *, deadline=None):
        global cancelled
        assert ref is reference
        assert events[-1]=="getter","Fresh CAN must follow the SDK getter"
        events.append("can")
        calls.append(ref)
        deadlines.append(deadline)
        if mode == "cancel_seed": cancelled=True
        if mode.endswith("readiness_delayed_callback"): clock.now+=.101
        if mode.endswith("readiness_exact_deadline_callback"): clock.now=deadline
        if mode.endswith("readiness_expiry_callback"): clock.now+=.013
        if mode.endswith("readiness_short_budget_callback"): clock.now+=.009
        if mode.endswith("readiness_can_error"): raise RuntimeError("current startup pose is moving")
        if mode.endswith("readiness_callback_timeout_after_mismatch") and len(calls)>1:
            clock.now=deadline
            raise RuntimeError("complete new enabled CAN feedback not received before deadline")
        shift=.002 if Arm.instances[-1].commands else .001
        if "readiness_recovers" in mode and len(calls)==3: shift=.003
        if mode == "drift" or (mode == "after_wait_drift" and len(calls)>1): shift=.05
        if mode.endswith("readiness_drift_wait") and len(calls)>1: shift=.05
        if mode.endswith("readiness_limits_wait") and len(calls)>1: shift=20
        if mode.endswith("readiness_zero_matched"): return [0]*7
        return [-.4+shift]*6+[0]
    def guard():
        global guard_calls_after_can
        if mode.endswith("readiness_before_seed_deadline") and events and events[-1]=="can":
            guard_calls_after_can+=1
            if guard_calls_after_can==2: clock.now+=.101
        if cancelled: raise SessionCancelled
    robot.after_sdk_startup=callback; robot.before_control_write=guard
    if mode == "missing_reference": robot.startup_reference=None
    if mode == "missing_callback": robot.after_sdk_startup=None
    if mode == "expired": reference["valid_until"]=now-1
    if mode == "invalid_tolerance": reference["tolerance_rad"]=[.2]*6
    @contextmanager
    def initialization_directory(root):
        if mode=="late_expired": reference["valid_until"]=clock.now-1
        yield
    robot_module._legacy.sdk_initialization_directory=initialization_directory
    early=mode in ("missing_reference","missing_callback","expired","invalid_tolerance","late_expired","relative_missing_offset") or mode.startswith("invalid_budget_")
    reference_before=copy.deepcopy(reference)
    if mode in ("success","relative_success") or mode.endswith(("readiness_recovers","readiness_zero_matched")):
        robot.connect()
        expected_seed=[0]*6 if mode.endswith("readiness_zero_matched") else [-.397 if "readiness_recovers" in mode else -.399]*6
        assert robot.arm.commands==[(expected_seed,[0.0]*6)],robot.arm.commands
        assert robot._previous_target==expected_seed
        expected_events=["getter","can"]*(3 if "readiness_recovers" in mode else 1)+["seed","getter","can"]
        assert events==expected_events,events
        assert reference==reference_before,(reference,reference_before)
        assert all(d==deadlines[0] for d in deadlines[:-1]),deadlines
        assert deadlines[-1] is None,deadlines
        assert math.isclose(deadlines[0],now+.1),deadlines
        assert len(clock.sleeps)==(3 if "readiness_recovers" in mode else 1),clock.sleeps
        assert all(duration<=.005 for duration in clock.sleeps),clock.sleeps
        robot.disconnect()
    else:
        failure=None
        try: robot.connect()
        except (RuntimeError,SessionCancelled) as error:
            failure=error
            if mode.endswith("sdk_all_zero"):
                assert "SDK and passive CAN positions disagree: stage=before_seed, J1:" in str(error),str(error)
                assert "sdk_all_zero=True" in str(error),str(error)
            if mode.endswith("sdk_disagree_axis4"):
                stage="after_seed" if "after_seed" in mode else "before_seed"
                assert f"SDK and passive CAN positions disagree: stage={stage}, J4:" in str(error),str(error)
                assert "sdk_all_zero=False" in str(error),str(error)
                match=re.search(r"SDK=([^ ]+) rad, CAN=([^ ]+) rad, delta\(SDK-CAN\)=([^ ]+) rad, tolerance=([^ ]+) rad",str(error))
                assert match,str(error)
                expected_values=(-.43,-.398,-.032,.0125) if stage=="after_seed" else (-.43,-.399,-.031,.0125)
                for actual,expected in zip(map(float,match.groups()),expected_values):
                    assert math.isclose(actual,expected,abs_tol=1e-12),(actual,expected)
        else: raise AssertionError("Invalid startup was accepted")
        if early: assert not Arm.instances
        else:
            assert Arm.instances[-1].cleaned
            expected=1 if mode in ("after_wait_drift","rejected_seed") or "after_seed" in mode else 0
            assert len(Arm.instances[-1].commands)==expected,Arm.instances[-1].commands
        assert robot.arm is None
        if mode.endswith("sdk_all_zero"):
            assert events.count("getter")>1,events
            assert math.isclose(clock.now,now+.1),clock.now
        if "after_seed" in mode: assert events.count("getter")==2,events
        if mode.endswith(("readiness_delayed_callback","readiness_exact_deadline_callback","readiness_expiry_callback","readiness_short_budget_callback","readiness_before_seed_deadline")):
            assert "deadline" in str(failure),str(failure)
            assert events.count("getter")==1,events
        if mode.endswith(("readiness_delayed_getter","readiness_exact_deadline_getter")):
            assert not calls and events==["getter"],events
        if mode.endswith(("readiness_cancel_wait","readiness_can_error")):
            assert events.count("getter")==1,events
        if mode.endswith(("readiness_drift_wait","readiness_limits_wait")):
            assert events.count("getter")==2,events
            assert len(clock.sleeps)==1,clock.sleeps
        if mode.endswith("readiness_callback_timeout_after_mismatch"):
            assert events.count("getter")==2,events
            assert "last Current startup SDK and passive CAN positions disagree: stage=before_seed, J1:" in str(failure),str(failure)
            assert "sdk_all_zero=True" in str(failure),str(failure)
        if "readiness_" in mode or mode.endswith("sdk_all_zero"):
            assert reference==reference_before,(reference,reference_before)
            assert robot.startup_reference is reference
            assert all(duration<=.005 for duration in clock.sleeps),clock.sleeps
            assert all(d==deadlines[0] for d in deadlines),deadlines
'''


LEADER_TARGET_ADAPTER_SCENARIO = SCENARIO.split("class Manager:", 1)[0] + r'''
import copy
import time
from lerobot_robot_lumos_stararm102.common import JOINT_KEYS

mode = sys.argv[1]
clock = types.SimpleNamespace(now=10.0)
events = []
robot_module = sys.modules["lerobot_robot_lumos_stararm102.lumos"]
robot_module.time = types.SimpleNamespace(
    monotonic=lambda: clock.now,
    sleep=lambda duration: setattr(clock, "now", clock.now + duration),
)

class Arm:
    instances = []

    def __init__(self, **kwargs):
        self.commands, self.grips = [], []
        self.cleaned = False
        self.instances.append(self)

    def get_joint_positions(self):
        events.append("sdk")
        return [-.45 if mode == "sdk_disagree" else -.4] * 6

    def set_joint_raw(self, target, velocity):
        events.append("write")
        self.commands.append((target.copy(), velocity.copy()))
        return mode != "rejected_seed"

    def setGripperPosition(self, opening):
        self.grips.append(opening)
        return True

    def cleanup(self):
        self.cleaned = True

sys.modules["startouchclass"] = types.SimpleNamespace(SingleArm=Arm)
with tempfile.TemporaryDirectory() as folder:
    root = Path(folder)
    profile = configuration(root)
    config = LumosConfig(
        sdk_root=str(root / "sdk"), mapping_file=str(root / "calibration.json"),
        joint_limits_file=str(root / "limits.json"), execute=True, can_interface="mock-can",
        startup_wait_s=.001, max_speed_deg_per_sec=30, max_target_step_deg=.25,
    )
    robot = Lumos(config)
    robot.startup_strategy = "leader_target"
    reference = dict(
        startup_strategy="leader_target", actual_q=[-.4] * 6 + [.5],
        frame_stamps=[9.99] * 7, captured_monotonic=10.0, valid_until=15.0,
        tolerance_rad=[math.radians(2)] * 6, sdk_tolerance_rad=[.02] * 6,
        sdk_ready_budget_s=.1,
    )
    if mode == "invalid_initial_opening":
        reference["actual_q"][6] = 1.01
    if mode == "invalid_initial_joint2":
        reference["actual_q"][1] = -1.58
    robot.startup_reference = reference
    before = copy.deepcopy(reference)
    received = []

    def callback(ref, *, deadline=None):
        assert ref is reference
        assert events[-1] == "sdk"
        events.append("can")
        shift = .002 if Arm.instances[-1].commands else .001
        if mode == "seed_drift" and Arm.instances[-1].commands:
            shift = -.034
        opening = .02 if mode == "still_open" else (.3 if mode == "closing" else .5)
        if mode == "invalid_current_opening":
            opening = 1.01
        actual = [-.4 + shift] * 6 + [opening]
        received.append(actual.copy())
        return actual

    robot.after_sdk_startup = callback
    if mode in ("slew", "still_open", "closing"):
        robot.connect()
        arm = robot.arm
        seed = [-.399] * 6
        assert arm.commands == [(seed, [0.0] * 6)], arm.commands
        assert events == ["sdk", "can", "write", "sdk", "can"], events
        assert robot._previous_target == seed
        assert received[0][6] == (.02 if mode == "still_open" else .3 if mode == "closing" else .5)
        assert all(values[6] > .01 for values in received), received
        assert reference == before
        assert reference.get("session_offset_sdk_rad") is None
        if mode == "slew":
            requested = [20.0, -20.0, 1.8, -1.8, 0.0, -.6]
            clipped = [1.9, -1.9, 1.8, -1.8, 0.0, -.6]
            action = dict(zip(robot.action_features, requested + [.5]))
            untouched = action.copy()
            previous = seed
            for index, elapsed in enumerate([.001, .01, .2] + [.02] * 600):
                clock.now += elapsed
                sent = robot.send_action(action)
                target = [sent[key] for key in JOINT_KEYS]
                allowed = math.radians(min(config.max_target_step_deg, config.max_speed_deg_per_sec * elapsed))
                for old, new, desired in zip(previous, target, clipped):
                    assert abs(new - old) <= allowed + 1e-12, (index, old, new, allowed)
                    assert min(old, desired) - 1e-12 <= new <= max(old, desired) + 1e-12
                assert all(lo <= q <= hi for q, lo, hi in zip(target, *robot.limits))
                if index == 0:
                    assert target != clipped
                    assert math.isclose(target[0] - seed[0], math.radians(.03), abs_tol=1e-12)
                previous = target
            assert all(math.isclose(q, goal, abs_tol=1e-12) for q, goal in zip(previous, clipped)), previous
            assert previous[1] < -1.57  # SDK J2 startup restriction applies to the actual seed.
            assert action == untouched
        robot.disconnect()
        assert arm.cleaned and robot.arm is None
        assert robot._previous_target is None and robot._last_send is None
    else:
        try:
            robot.connect()
        except RuntimeError as error:
            if mode == "seed_drift":
                assert "fixed SDK seed" in str(error), str(error)
            if mode == "sdk_disagree":
                assert "SDK and passive CAN positions disagree" in str(error), str(error)
        else:
            raise AssertionError("Invalid leader_target startup was accepted: " + mode)
        if mode.startswith("invalid_initial_"):
            assert not Arm.instances
        else:
            arm = Arm.instances[-1]
            assert arm.cleaned
            assert len(arm.commands) == (1 if mode in ("seed_drift", "rejected_seed") else 0), arm.commands
        assert reference == before
        assert robot.arm is None and robot._previous_target is None
        assert robot.last_sent_action is None
'''


MEASURED_LIMIT_ADAPTER_SCENARIO = SCENARIO.split("class Manager:", 1)[0] + r'''
import copy

strategy, mode = sys.argv[1].split(":", 1)
clock = types.SimpleNamespace(now=10.0)
robot_module = sys.modules["lerobot_robot_lumos_stararm102.lumos"]
robot_module.time = types.SimpleNamespace(
    monotonic=lambda: clock.now,
    sleep=lambda duration: setattr(clock, "now", clock.now + duration),
)
lsb = 25.0 / 65535
margin = 4 * lsb
measured = [0.0] * 6
measured[2] = margin + lsb if mode == "actual_over" else lsb / 2

class Arm:
    instances = []

    def __init__(self, **kwargs):
        self.commands, self.cleaned = [], False
        self.instances.append(self)

    def get_joint_positions(self):
        return measured.copy()

    def set_joint_raw(self, target, velocity):
        self.commands.append(target.copy())
        return True

    def setGripperPosition(self, opening):
        return True

    def cleanup(self):
        self.cleaned = True

sys.modules["startouchclass"] = types.SimpleNamespace(SingleArm=Arm)
with tempfile.TemporaryDirectory() as folder:
    root = Path(folder)
    configuration(root)
    config = LumosConfig(
        sdk_root=str(root / "sdk"), mapping_file=str(root / "calibration.json"),
        joint_limits_file=str(root / "limits.json"), execute=True,
        can_interface="mock-can", startup_wait_s=.001,
    )
    robot = Lumos(config)
    assert tuple(robot.measured_limit_margin_rad) == (0.0,) * 6
    lower, upper = robot.limits
    upper = upper.copy()
    upper[2] = 0.0
    robot.limits = lower, upper
    robot.startup_strategy = strategy
    robot.measured_limit_margin_rad = [margin] * 6
    if mode == "missing":
        del robot.measured_limit_margin_rad
    elif mode == "zero":
        robot.measured_limit_margin_rad = [0.0] * 6
    elif mode.startswith("invalid_"):
        bad = {
            "negative": [-margin] * 6, "nan": [math.nan] * 6,
            "infinity": [math.inf] * 6, "boolean": [True] * 6,
            "short": [margin] * 5, "long": [margin] * 7,
            "string": ["0"] * 6, "none": None,
        }
        robot.measured_limit_margin_rad = bad[mode.removeprefix("invalid_")]
    reference = dict(
        startup_strategy=strategy, actual_q=[0.0, 0.0, lsb / 2, 0.0, 0.0, 0.0, 0.0],
        frame_stamps=[9.99] * 7, captured_monotonic=10.0, valid_until=15.0,
        tolerance_rad=[math.radians(2)] * 6, sdk_tolerance_rad=[.02] * 6,
        sdk_ready_budget_s=.1,
    )
    if mode == "anchor_over":
        reference["actual_q"][2] = margin + lsb
    if strategy == "current_relative":
        reference["session_offset_sdk_rad"] = [0.0] * 6
    robot.startup_reference = reference
    before = copy.deepcopy(reference)
    actual = measured + [0.0]
    robot.after_sdk_startup = lambda ref, **kwargs: actual.copy()
    if mode == "half":
        robot.connect()
        arm = robot.arm
        assert robot._previous_target == [0.0] * 6, robot._previous_target
        assert arm.commands == ([] if strategy == "home" else [[0.0] * 6]), arm.commands
        assert reference == before, (reference, before)
        assert actual[2] == lsb / 2
        assert measured[2] == lsb / 2
        clock.now += .01
        robot.send_action(dict(zip(robot.action_features, [0.0] * 7)))
        assert all(lo <= q <= hi for target in arm.commands for q, lo, hi in zip(target, *robot.limits))
        robot.disconnect()
        assert arm.cleaned and robot.arm is None
    else:
        try:
            robot.connect()
        except (RuntimeError, ValueError) as error:
            if mode.startswith("invalid_"):
                assert "measured_limit_margin_rad" in str(error), str(error)
        else:
            raise AssertionError("Invalid measured limit admission was accepted: " + mode)
        if mode.startswith("invalid_") or (strategy != "home" and mode in ("missing", "zero", "anchor_over")):
            assert not Arm.instances
        else:
            assert Arm.instances[-1].cleaned
            assert not Arm.instances[-1].commands
        assert robot.arm is None
        assert reference == before
'''


class CurrentAdapterTests(unittest.TestCase):
    def scenario(self, name, script=ADAPTER_SCENARIO):
        root = Path(__file__).resolve().parents[1]
        environment = os.environ.copy()
        environment["PYTHONPATH"] = os.pathsep.join((str(root / "src"), str(root / "tests")))
        result = subprocess.run(
            [sys.executable, "-B", "-c", script, name], cwd=root,
            env=environment, capture_output=True, text=True, timeout=20,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_real_post_can_seed_is_fixed_and_home_default_is_bypassed_explicitly(self):
        self.scenario("success")
        self.scenario("relative_success")

    def test_measured_half_lsb_keeps_real_references_and_clamps_all_startup_command_origins(self):
        for strategy in ("home", "current_aligned", "current_relative", "leader_target"):
            with self.subTest(strategy=strategy):
                self.scenario(strategy + ":half", MEASURED_LIMIT_ADAPTER_SCENARIO)

    def test_measured_margin_rejects_outside_actual_and_outside_preflight_anchor(self):
        for strategy in ("home", "current_aligned", "current_relative", "leader_target"):
            for mode in (("actual_over",) if strategy == "home" else ("actual_over", "anchor_over")):
                with self.subTest(strategy=strategy, mode=mode):
                    self.scenario(strategy + ":" + mode, MEASURED_LIMIT_ADAPTER_SCENARIO)

    def test_direct_adapter_use_keeps_zero_measurement_margin_when_missing_or_zero(self):
        for strategy in ("home", "leader_target"):
            for mode in ("missing", "zero"):
                with self.subTest(strategy=strategy, mode=mode):
                    self.scenario(strategy + ":" + mode, MEASURED_LIMIT_ADAPTER_SCENARIO)

    def test_invalid_measured_margins_fail_before_sdk_construction(self):
        for strategy in ("home", "leader_target"):
            for mode in ("negative", "nan", "infinity", "boolean", "short", "long", "string", "none"):
                with self.subTest(strategy=strategy, mode=mode):
                    self.scenario(strategy + ":invalid_" + mode, MEASURED_LIMIT_ADAPTER_SCENARIO)

    def test_leader_target_uses_real_seed_then_slews_to_clipped_absolute_target(self):
        self.scenario("slew", LEADER_TARGET_ADAPTER_SCENARIO)

    def test_leader_target_keeps_open_or_closing_gripper_measurements(self):
        for mode in ("still_open", "closing"):
            with self.subTest(mode=mode):
                self.scenario(mode, LEADER_TARGET_ADAPTER_SCENARIO)

    def test_leader_target_keeps_sdk_seed_and_actual_pose_guards_with_cleanup(self):
        for mode in (
            "seed_drift", "sdk_disagree", "rejected_seed", "invalid_initial_opening",
            "invalid_current_opening", "invalid_initial_joint2",
        ):
            with self.subTest(mode=mode):
                self.scenario(mode, LEADER_TARGET_ADAPTER_SCENARIO)

    def test_invalid_preflight_never_constructs_sdk(self):
        for name in ("missing_reference", "missing_callback", "expired", "invalid_tolerance", "late_expired", "relative_missing_offset"):
            with self.subTest(name=name):
                self.scenario(name)

    def test_drift_disagreement_cancellation_and_seed_rejection_cleanup(self):
        for name in ("drift", "sdk_disagree", "cancel_seed", "after_wait_drift", "rejected_seed"):
            with self.subTest(name=name):
                self.scenario(name)

    def test_sdk_disagreement_reports_first_axis_values_before_any_seed_write(self):
        self.scenario("sdk_disagree_axis4")
        self.scenario("relative_sdk_disagree_axis4")

    def test_sdk_disagreement_after_seed_reports_stage_without_another_target(self):
        self.scenario("after_seed_sdk_disagree_axis4")
        self.scenario("relative_after_seed_sdk_disagree_axis4")

    def test_zero_sdk_cache_is_reported_and_rejected_before_any_seed_write(self):
        self.scenario("sdk_all_zero")
        self.scenario("relative_sdk_all_zero")

    def test_readiness_retries_zero_cache_then_seeds_once_from_matching_fresh_can(self):
        self.scenario("readiness_recovers")
        self.scenario("relative_readiness_recovers")

    def test_genuine_matching_zero_pose_is_accepted(self):
        self.scenario("readiness_zero_matched")
        self.scenario("relative_readiness_zero_matched")

    def test_readiness_deadline_rejects_late_device_reads_and_prewrite_delay(self):
        for mode in ("readiness_delayed_getter", "readiness_delayed_callback", "readiness_expiry_callback",
                     "readiness_short_budget_callback", "readiness_before_seed_deadline",
                     "readiness_exact_deadline_getter", "readiness_exact_deadline_callback",
                     "readiness_callback_timeout_after_mismatch"):
            with self.subTest(mode=mode):
                self.scenario(mode)

    def test_readiness_cancellation_drift_limits_and_can_failure_do_not_retry(self):
        for mode in ("readiness_cancel_wait", "readiness_drift_wait", "readiness_limits_wait", "readiness_can_error"):
            with self.subTest(mode=mode):
                self.scenario(mode)

    def test_invalid_readiness_budget_is_rejected_before_sdk_construction(self):
        for suffix in ("zero", "negative", "nan", "infinity", "too_large", "boolean"):
            with self.subTest(suffix=suffix):
                self.scenario("invalid_budget_" + suffix)


if __name__ == "__main__":
    unittest.main()
