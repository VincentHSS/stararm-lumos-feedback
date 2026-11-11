"""Manual unloaded recapture regressions with fake transports and model frames.

These tests never import a device SDK or claim that a physical arm is unloaded.
"""

import http.client
import json
import struct
import time
import unittest
from copy import deepcopy
from types import SimpleNamespace
from unittest.mock import Mock

import test_feedback_web_motion as web_motion_tests
import test_local_residual_reference as local_reference_tests
from test_feedback_runtime import UART, FakeLeader, FakeRobot, FixtureCase, LiveMonitor

from lerobot_robot_lumos_stararm102.button_protocol import CommandButtonState
from stararm_lumos_feedback.runtime import ACTION_KEYS, LeaderReader, Session
from stararm_lumos_feedback.transport import Fashionstar, packet


class ManualBaselineRecaptureWebTests(unittest.TestCase):
    setUp = web_motion_tests.WebMotionTests.setUp

    def post(self, value, *, token=True, origin=None, raw=False):
        headers = {"Content-Type": "application/json"}
        if token:
            headers["X-Feedback-Token"] = self.dashboard.token
        if origin is not None:
            headers["Origin"] = origin
        client = http.client.HTTPConnection("127.0.0.1", self.dashboard.server.server_port, timeout=2)
        try:
            client.request("POST", "/api/experimental-baseline", value if raw else json.dumps(value), headers)
            response = client.getresponse()
            return response.status, json.loads(response.read())
        finally:
            client.close()

    def fake_manager(self):
        manager = SimpleNamespace(
            request_baseline_recapture=Mock(return_value={
                "state": "active", "baseline_recapture": {"state": "pending"},
            }),
        )
        self.dashboard.connection = manager
        return manager

    def test_only_exact_explicit_boolean_confirmation_can_queue(self):
        manager = self.fake_manager()
        for value in (
            {}, None, True, [], {"unloaded_confirmed": False}, {"unloaded_confirmed": 1},
            {"unloaded_confirmed": "true"}, {"unloaded_confirmed": None},
            {"unloaded_confirmed": True, "mode": "experiment"},
            {"unloaded_confirmed": True, "axis_mw": [0] * 7},
        ):
            with self.subTest(value=value):
                self.assertEqual(self.post(value)[0], 400)
        for raw in ("{", "", "[}"):
            with self.subTest(raw=raw):
                self.assertEqual(self.post(raw, raw=True)[0], 400)
        manager.request_baseline_recapture.assert_not_called()
        code, state = self.post({"unloaded_confirmed": True})
        self.assertEqual(code, 200)
        self.assertEqual(state["baseline_recapture"]["state"], "pending")
        manager.request_baseline_recapture.assert_called_once_with()

    def test_token_and_local_origin_are_required_before_queueing(self):
        manager = self.fake_manager()
        self.assertEqual(self.post({"unloaded_confirmed": True}, token=False)[0], 403)
        self.assertEqual(self.post({"unloaded_confirmed": True}, origin="https://example.invalid")[0], 403)
        manager.request_baseline_recapture.assert_not_called()

    def test_unavailable_or_rejected_session_returns_conflict(self):
        self.dashboard.connection = None
        self.assertEqual(self.post({"unloaded_confirmed": True})[0], 409)
        manager = self.fake_manager()
        manager.request_baseline_recapture.side_effect = RuntimeError("zero_caps_required")
        code, state = self.post({"unloaded_confirmed": True})
        self.assertEqual(code, 409)
        self.assertIn("zero_caps_required", state["error"])

    def test_connection_manager_rejects_idle_other_modes_and_stopping(self):
        self.dashboard.connection = self.manager
        for mode, state in (("experiment", "idle"), ("follow", "active"),
                            ("monitor", "active"), ("experiment", "stopping")):
            self.manager.mode, self.manager.state = mode, state
            with self.subTest(mode=mode, state=state):
                self.assertEqual(self.post({"unloaded_confirmed": True})[0], 409)
        self.assertIsNone(self.manager.worker)

    def test_connection_status_uses_cached_primitives_and_request_releases_connection_lock(self):
        self.addCleanup(setattr, self.manager, "worker", None)
        self.manager.worker = SimpleNamespace(is_alive=lambda: True)
        self.manager.mode, self.manager.state = "experiment", "active"
        self.manager._can_recapture_baseline = True
        calls = []

        def outside_connection_lock(name, result):
            self.assertFalse(self.manager.lock._is_owned(), name + " held the connection lock")
            calls.append(name)
            return result

        self.manager.observer = SimpleNamespace(
            can_recapture_baseline=Mock(side_effect=AssertionError("status must use cached readiness")),
            baseline_recapture_status=Mock(side_effect=AssertionError("status must use cached capture state")),
            request_baseline_recapture=lambda: outside_connection_lock("queue_recapture", {"state": "pending"}),
        )
        with self.manager.lock:
            self.assertTrue(self.manager.status()["can_recapture_baseline"])
            self.manager.publish("active")
        self.assertEqual(calls, [])
        self.manager.request_baseline_recapture()
        self.assertEqual(calls, ["queue_recapture"])
        self.assertFalse(self.manager.status()["can_recapture_baseline"])
        self.assertEqual(self.manager.status()["baseline_recapture"]["state"], "pending")
        # Stop/error publication can recursively snapshot under an outer
        # RLock. An inactive session must not consult its transport then.
        calls.clear()
        with self.manager.lock:
            self.manager.state = "stopping"
            self.manager.publish("stopped")
        self.assertEqual(calls, [])


class ManualBaselineRecaptureRuntimeTests(FixtureCase):
    def make_session(self, *, released=True, zero_caps=True):
        rig = local_reference_tests._ResidualRig(self.profile)
        if zero_caps:
            rig.caps.set(dict(common_mw=1200, axis_mw=[0] * 7))
        uart, robot = UART(), FakeRobot()
        robot.config = SimpleNamespace(max_target_step_deg=2)
        robot.limits = rig.profile.limits
        leader = FakeLeader(uart)
        button = CommandButtonState()
        button.feed(b"".join(packet(24, struct.pack("<BBH", i, 0x10 if released else 0x11, 0))
                             for i in range(6)))
        session = Session(rig.profile, robot, leader, rig.caps, mode="experiment", monitor=LiveMonitor(rig.profile))
        session.controller, session.baseline = rig.control, rig.control.baseline
        session.transport = Fashionstar(
            uart, range(7), enabled=True, timeout=0.03, command_button=button,
            protect_native_hold=True, max_output_mw=1500, allow_hold=False,
        )
        session.reader = LeaderReader(leader, session.transport)
        session._sdk_history_ready = True
        session.target = rig.home.copy()
        session.desired = rig.home.copy()
        session._baseline_completed_send = rig.home.copy()
        return session, rig, uart, robot, button

    def lock_native(self, button):
        button.feed(b"".join(packet(24, struct.pack("<BBH", i, 0x11, 1500)) for i in range(6)))

    def assert_rejected(self, session):
        self.assertFalse(session.can_recapture_baseline())
        with self.assertRaises(RuntimeError):
            session.request_baseline_recapture()

    def test_only_released_initialized_experiment_with_every_axis_zero_is_eligible(self):
        for setting in ("caps", "native", "sdk", "mode", "stop", "error"):
            session, _, _, _, _ = self.make_session(released=setting != "native", zero_caps=setting != "caps")
            if setting == "sdk":
                session._sdk_history_ready = False
            elif setting == "mode":
                session.mode = "follow"
            elif setting == "stop":
                session.stop.set()
            elif setting == "error":
                session.error = RuntimeError("existing fault")
            with self.subTest(setting=setting):
                self.assert_rejected(session)
        session, _, _, _, _ = self.make_session()
        self.assertTrue(session.can_recapture_baseline())
        # Common cap remains 1200; every individual effective cap must be zero.
        session.caps.set(dict(common_mw=1200, axis_mw=[0] * 6 + [1]))
        self.assert_rejected(session)

    def test_request_only_queues_duplicate_rejected_and_control_thread_replaces_alias(self):
        session, rig, uart, robot, _ = self.make_session()
        rig.freeze()
        old_baseline = session.baseline
        frozen = deepcopy(rig.control.snapshot()["baseline_groups"])
        target, desired = session.target, session.desired
        guard = Mock()
        robot.before_control_write = guard
        before_writes = list(uart.writes)
        state = session.request_baseline_recapture()
        self.assertEqual(state["state"], "pending")
        self.assertEqual(rig.control.snapshot()["baseline_groups"], frozen)
        self.assertIs(session.baseline, old_baseline)
        self.assertEqual(uart.writes, before_writes)
        with self.assertRaises(RuntimeError):
            session.request_baseline_recapture()
        session._process_baseline_recapture()
        self.assertIs(session.baseline, session.controller.baseline)
        self.assertIsNot(session.baseline, old_baseline)
        self.assertFalse(session.controller.snapshot()["baseline_frozen"])
        self.assertEqual(session.baseline_recapture_status()["state"], "collecting")
        self.assertIs(session.target, target)
        self.assertIs(session.desired, desired)
        self.assertIs(robot.before_control_write, guard)
        self.assertEqual(robot.sent, [])
        self.assertFalse(robot.cleaned)
        self.assertEqual(uart.writes, before_writes)
        self.assertTrue(session._sdk_history_ready)
        self.assertFalse(session.reader.manual_hold)
        with self.assertRaises(RuntimeError):
            session.request_baseline_recapture()

    def test_cap_or_native_change_while_queued_aborts_without_fallback_to_old_reference(self):
        for change in ("caps", "native"):
            session, rig, _, _, button = self.make_session()
            rig.freeze()
            session.request_baseline_recapture()
            if change == "caps":
                rig.caps.set(dict(common_mw=1200, axis_mw=[0, 0, 0, 1, 0, 0, 0]))
            else:
                self.lock_native(button)
            with self.subTest(change=change):
                session._process_baseline_recapture()
                self.assertEqual(session.baseline_recapture_status()["state"], "rejected")
                state = rig.control.snapshot()
                self.assertTrue(state["baseline_collection_closed"])
                self.assertEqual(state["baseline_ready_axes"], [False] * 7)
                self.assertFalse(rig.local()["available"])
                self.assertEqual(rig.control.outputs(False)[0], [0.0] * 7)

    def test_mid_collection_cap_raise_or_native_lock_aborts_without_automatic_relearning(self):
        for change in ("caps", "native"):
            session, rig, _, _, button = self.make_session()
            rig.freeze()
            session.request_baseline_recapture()
            session._process_baseline_recapture()
            rig.run(count=30, residual=rig.j4(-0.10))
            if change == "caps":
                rig.caps.set(dict(common_mw=1200, axis_mw=[0, 0, 0, 1, 0, 0, 0]))
            else:
                self.lock_native(button)
            with self.subTest(change=change):
                session._process_baseline_recapture()
                self.assertEqual(session.baseline_recapture_status()["state"], "rejected")
                state = rig.control.snapshot()
                self.assertTrue(state["baseline_collection_closed"])
                self.assertEqual(state["baseline_ready_axes"], [False] * 7)
                self.assertFalse(state["local_reference"]["available"])
                rig.caps.set(dict(common_mw=1200, axis_mw=[0] * 7))
                for _ in range(150):
                    self.assertEqual(rig.update(residual=rig.j4(-0.10)), [0.0] * 7)
                self.assertEqual(rig.control.snapshot()["baseline_ready_axes"], [False] * 7)

    def test_completed_recapture_replaces_reference_once_and_never_resets_follower(self):
        session, rig, uart, robot, _ = self.make_session()
        rig.freeze()
        session.request_baseline_recapture()
        session._process_baseline_recapture()
        for _ in range(150):
            self.assertEqual(rig.update(residual=rig.j4(-0.10)), [0.0] * 7)
            session._finish_baseline_recapture()
            if session.baseline_recapture_status()["state"] == "complete":
                break
        else:
            self.fail("A new valid stationary window did not finish manual recapture")
        local = rig.local()
        self.assertAlmostEqual(local["signed_residual_min_nm"][3], -0.10)
        frozen = deepcopy(rig.control.snapshot()["baseline_groups"])
        rig.run(count=150, residual=rig.j4(-0.50))
        session._finish_baseline_recapture()
        self.assertEqual(rig.control.snapshot()["baseline_groups"], frozen)
        self.assertAlmostEqual(rig.local()["signed_residual_min_nm"][3], -0.10)
        self.assertEqual(robot.sent, [])
        self.assertFalse(robot.cleaned)
        self.assertEqual(uart.writes, [])
        self.assertEqual(session._baseline_completed_send, rig.home)
        self.assertEqual(session.target, rig.home)

    def test_cap_or_native_race_after_new_window_cannot_be_reported_as_complete(self):
        for change in ("caps", "native"):
            session, rig, _, _, button = self.make_session()
            rig.freeze()
            session.request_baseline_recapture()
            session._process_baseline_recapture()
            rig.freeze([rig.j4(-0.10)])
            self.assertEqual(session.baseline_recapture_status()["state"], "collecting")
            if change == "caps":
                rig.caps.set(dict(common_mw=1200, axis_mw=[0] * 6 + [1]))
            else:
                self.lock_native(button)
            with self.subTest(change=change):
                session._finish_baseline_recapture()
                self.assertEqual(session.baseline_recapture_status()["state"], "rejected")
                self.assertEqual(rig.control.snapshot()["baseline_ready_axes"], [False] * 7)
                self.assertFalse(rig.local()["available"])
                self.assertEqual(rig.control.outputs(False)[0], [0.0] * 7)

    def test_regular_control_step_consumes_queue_and_continues_existing_follow_targets(self):
        session, rig, uart, robot, _ = self.make_session()
        rig.freeze()
        session.leader.get_action = lambda: dict(zip(ACTION_KEYS, rig.home))
        rig.control.clock = time.monotonic
        robot.connect = Mock(side_effect=AssertionError("Recapture must not reconnect the follower"))
        robot.disconnect = Mock(side_effect=AssertionError("Recapture must not disable the follower"))
        old_baseline = session.baseline
        session.request_baseline_recapture()
        session.step()
        self.assertEqual(session.baseline_recapture_status()["state"], "collecting")
        self.assertIsNot(session.baseline, old_baseline)
        self.assertIs(session.baseline, session.controller.baseline)
        self.assertEqual(robot.sent, [dict(zip(ACTION_KEYS, rig.home))])
        self.assertEqual(session.target, rig.home)
        self.assertEqual(session.desired, rig.home)
        commands = [struct.unpack("<BBH", frame[4:-1]) for frame in uart.writes if frame[2] == 24]
        self.assertTrue(all(mode == 0x10 and power == 0 for _, mode, power in commands))
        robot.connect.assert_not_called()
        robot.disconnect.assert_not_called()


class ManualBaselineRecaptureControllerTests(FixtureCase):
    def test_reset_clears_old_output_confirmation_and_reference_but_not_tuning(self):
        rig = local_reference_tests._ResidualRig(self.profile)
        rig.freeze()
        self.assertGreater(rig.run(residual=rig.j4(-0.10))[3], 0)
        policy, axes = deepcopy(rig.profile.policy), deepcopy(rig.profile.axes)
        rig.caps.set(dict(common_mw=1200, axis_mw=[0] * 7))
        rig.control.reset_unloaded_baseline()
        self.assertEqual(rig.control.outputs(False), ([0.0] * 7, [False] * 7))
        self.assertEqual(rig.control.confirm, [0.0] * 7)
        self.assertEqual(rig.control.filtered, [0.0] * 7)
        state = rig.control.snapshot()
        self.assertFalse(state["baseline_collection_closed"])
        self.assertEqual(state["baseline_ready_axes"], [False] * 7)
        self.assertFalse(state["local_reference"]["available"])
        self.assertEqual(state["local_reference"]["confirm_s"], [0.0] * 7)
        self.assertEqual((rig.profile.policy, rig.profile.axes), (policy, axes))
        for _ in range(40):
            self.assertEqual(rig.update(residual=rig.j4(-0.10)), [0.0] * 7)
        self.assertFalse(rig.control.snapshot()["baseline_frozen"])
        rig.freeze([rig.j4(-0.10)])
        self.assertAlmostEqual(rig.local()["signed_residual_min_nm"][3], -0.10)

    def test_manual_recapture_keeps_arm_and_gripper_admission_independent(self):
        rig = local_reference_tests._ResidualRig(self.profile)
        rig.freeze()
        rig.caps.set(dict(common_mw=1200, axis_mw=[0] * 7))
        rig.control.reset_unloaded_baseline()
        update = rig.control.update

        def arm_only(samples, target, desired, *, baseline_admission):
            baseline_admission = deepcopy(baseline_admission)
            baseline_admission.update(
                allowed=False, common_allowed=True,
                groups={
                    "arm": dict(allowed=True, reason="arrived", axis_indices=list(range(6))),
                    "gripper": dict(allowed=False, reason="not_arrived", axis_indices=[6]),
                },
            )
            return update(samples, target, desired, baseline_admission=baseline_admission)

        rig.control.update = arm_only
        for _ in range(150):
            self.assertEqual(rig.update(residual=rig.j4(-0.10)), [0.0] * 7)
        state = rig.control.snapshot()
        self.assertTrue(state["baseline_groups"]["arm"]["frozen"])
        self.assertFalse(state["baseline_groups"]["gripper"]["frozen"])
        self.assertEqual(state["baseline_ready_axes"], [True] * 6 + [False])
        rig.control.update = update
        for _ in range(150):
            rig.update()
        self.assertFalse(rig.control.snapshot()["baseline_groups"]["gripper"]["frozen"])
