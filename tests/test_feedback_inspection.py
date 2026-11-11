"""Software-only inspection tests: no SDK, serial port, or physical CAN bus."""

import builtins
import math
import struct
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import can

from stararm_lumos_feedback.gripper import TypeLJ04041254
from stararm_lumos_feedback.inspection import Inspection


class Clock:
    def __init__(self):
        self.now = 100.0

    def monotonic(self):
        return self.now

    def wall(self):
        return self.now + 900


def profile():
    axis = dict(
        p_max=12.5,
        v_max=30,
        t_max=10,
        torque_scale=1,
        max_age_s=0.15,
        max_temperature_c=70,
        max_velocity=10,
        max_acceleration=200,
        max_jump=0.5,
        limit_margin=0.001,
        sdk_scale=1,
        urdf_scale=1,
    )
    result = SimpleNamespace(
        axes=[dict(axis, feedback_id=16 + i, motor_id=i + 1) for i in range(7)],
        session=dict(fps=50, robot=dict(can_interface="fake-can")),
        guards=dict(can_skew_s=0.1, startup_feedback_s=0.08),
        policy=[dict(acceleration_filter_s=None) for _ in range(7)],
        limits=([-1.9] * 6, [1.9] * 6),
        gripper_transform=dict(max_motor_jump_rad=0.5),
        model=SimpleNamespace(evaluate=lambda q, v, a: [0.0] * 6),
    )
    result.to_sdk = lambda i, p: TypeLJ04041254.opening(p) if i == 6 else p
    result.to_velocity = lambda i, p, v: TypeLJ04041254.jacobian(p) * v if i == 6 else v
    result.to_model = lambda i, q: q

    def check_gripper_motor(p, v):
        if not -1.26 <= p <= 0.001 or abs(v) > 10:
            raise RuntimeError("Invalid raw gripper motor coordinates")

    result.check_gripper_motor = check_gripper_motor
    return result


def frame(configuration, clock, i=0, *, status=0, position=None, velocity=0, temperature=30):
    axis = configuration.axes[i]
    position = (-0.6 if i == 6 else 0) if position is None else position

    def encode(value, bits, maximum):
        return round((value / maximum + 1) / 2 * ((1 << bits) - 1))

    p = encode(position, 16, axis["p_max"])
    v = encode(velocity, 12, axis["v_max"])
    t = encode(0, 12, axis["t_max"])
    return can.Message(
        arbitration_id=axis["feedback_id"],
        timestamp=clock.wall(),
        is_extended_id=False,
        is_rx=True,
        data=bytes((status << 4 | axis["motor_id"], p >> 8, p & 255, v >> 4,
                    (v & 15) << 4 | t >> 8, t & 255, temperature, temperature)),
    )


class FakeBus:
    def __init__(self, configuration, clock, *, status=0, respond=True, once=False):
        self.configuration, self.clock = configuration, clock
        self.status, self.respond, self.once = status, respond, once
        self.sent = []
        self.incoming = []
        self.closed = False

    def send(self, message, timeout=None):
        self.sent.append(message)
        if self.respond and (not self.once or len(self.sent) <= 7):
            self.incoming.append(frame(self.configuration, self.clock, message.data[0] - 1, status=self.status))

    def recv(self, timeout):
        self.clock.now += 0.0001 if self.incoming else timeout
        return self.incoming.pop(0) if self.incoming else None

    def shutdown(self):
        self.closed = True


class Dashboard:
    def __init__(self):
        self.states = []

    def publish(self, **state):
        self.states.append(state)


class InspectionTests(unittest.TestCase):
    def setUp(self):
        self.profile = profile()
        self.clock = Clock()
        self.dashboard = Dashboard()
        self.bus = FakeBus(self.profile, self.clock)

    def observer(self, **kwargs):
        return Inspection(self.profile, bus=self.bus, dashboard=self.dashboard,
                          clock=self.clock.monotonic, wall=self.clock.wall, **kwargs)

    def test_disabled_feedback_is_visible_without_force_or_baseline_claim(self):
        observer = self.observer()
        result = observer.run(0.05)
        self.assertEqual(result["motor_status"], [0] * 7)
        self.assertTrue(all(result["fresh_axis_coverage"]))
        self.assertEqual(result["disabled_axes"], list(range(1, 8)))
        self.assertFalse(result["baseline_eligible"])
        self.assertFalse(result["all_enabled"])
        active = next(s for s in self.dashboard.states if s["status"] == "inspection_active")
        self.assertEqual(active["requested_mw"], [None] * 7)
        self.assertEqual(active["predicted_mw"], [None] * 7)
        self.assertEqual(active["hold_requested"], [None] * 7)
        self.assertFalse(active["leader_controlled"])
        self.assertEqual(active["leader_output_state"], "unknown")
        self.assertFalse(active["residual_verified"])
        self.assertIn("residual_nm", active)
        self.assertTrue(self.bus.closed)

    def test_sent_frame_set_only_contains_reviewed_refresh_command(self):
        self.observer().run(0.05)
        self.assertGreaterEqual(len(self.bus.sent), 7)
        self.assertEqual({m.data[0] for m in self.bus.sent}, set(range(1, 8)))
        for message in self.bus.sent:
            self.assertEqual(message.arbitration_id, 0x7FF)
            self.assertFalse(message.is_extended_id)
            self.assertFalse(message.is_fd)
            self.assertEqual(message.dlc, 8)
            self.assertEqual(bytes(message.data[1:]), bytes((0, 0xCC, 0, 0, 0, 0, 0)))

    def test_no_sdk_or_serial_modules_imported(self):
        original = builtins.__import__
        forbidden = ("startouch", "startouchclass", "serial", "fashionstar_uart_sdk", "lerobot")

        def guarded(name, *args, **kwargs):
            if name.split(".")[0] in forbidden:
                self.fail(f"Inspection imported hardware control module {name}")
            return original(name, *args, **kwargs)

        with patch("builtins.__import__", side_effect=guarded):
            self.observer().run(0.03)

    def test_enabled_feedback_still_does_not_establish_no_load_baseline(self):
        self.bus.status = 1
        result = self.observer().run(0.04)
        self.assertTrue(result["all_enabled"])
        self.assertFalse(result["baseline_eligible"])

    def test_parameter_response_and_duplicate_do_not_refresh_state(self):
        observer = self.observer()
        incoming = frame(self.profile, self.clock)
        observer.ingest(incoming)
        original = dict(observer.samples[0])
        self.clock.now += 0.05
        observer.ingest(incoming)
        for register, value in ((1, 0.1), (21, 12.5), (22, 10), (23, 28), (59, 20)):
            reply = can.Message(arbitration_id=16, is_extended_id=False, timestamp=self.clock.wall(),
                                data=bytes((1, 0, 0x33, register)) + struct.pack("<f", value))
            observer.ingest(reply)
        self.assertEqual(observer.samples[0], original)
        self.assertEqual(observer.counts[0], 1)
        self.assertEqual(observer.parameter_replies, 5)
        self.assertEqual(observer.statuses[0], original["status"])
        self.assertEqual(observer.temperatures[0], 30)
        self.clock.now += 0.2
        with self.assertRaisesRegex(RuntimeError, "missing/stale CAN"):
            observer.snapshot()

    def test_malformed_and_unverified_parameter_packets_are_not_hidden(self):
        observer = self.observer()
        candidates = [
            (bytes((1, 0, 0x33, register)), value)
            for register in (1, 21, 22, 23, 59)
            for value in (0, -1, math.nan, math.inf)
        ]
        candidates += [(bytes(header), 1) for header in (
            (1, 1, 0x33, 1),  # complete motor ID high byte must match
            (1, 0, 0x55, 1),  # write acknowledgement is not a read response
            (1, 0, 0x32, 1),
            (1, 0, 0x33, 20),  # not a register queried by this integration
        )]
        for header, value in candidates:
            reply = can.Message(arbitration_id=16, is_extended_id=False, timestamp=self.clock.wall(),
                                data=header + struct.pack("<f", value))
            with self.subTest(header=header, value=value), self.assertRaises(RuntimeError):
                observer.ingest(reply)
            self.assertEqual(observer.parameter_replies, 0)
            self.assertEqual(observer.counts, [0] * 7)

    def test_parameter_filter_needs_matching_arbitration_and_full_motor_id(self):
        observer = self.observer()
        wrong_axis = can.Message(arbitration_id=17, is_extended_id=False, timestamp=self.clock.wall(),
                                 data=bytes((1, 0, 0x33, 1)) + struct.pack("<f", 1))
        observer.ingest(wrong_axis)
        self.assertEqual(observer.parameter_replies, 0)
        self.assertEqual(observer.counts, [0] * 7)
        for status in (1, 8, 9, 15):
            reply = can.Message(arbitration_id=16, is_extended_id=False, timestamp=self.clock.wall(),
                                data=bytes((status << 4 | 1, 0, 0x33, 1)) + struct.pack("<f", 1))
            reason = "overtemperature" if status == 1 else f"fault/status {status}"
            with self.subTest(status=status), self.assertRaisesRegex(RuntimeError, reason):
                observer.ingest(reply)
            self.assertEqual(observer.statuses[0], status)
            self.assertEqual(observer.parameter_replies, 0)
            self.assertEqual(observer.counts, [0] * 7)

    def test_fault_status_is_visible_and_stops_with_can_closed(self):
        self.bus.status = 8
        observer = self.observer()
        with self.assertRaisesRegex(RuntimeError, "fault/status 8"):
            observer.run(0.1)
        last = self.dashboard.states[-1]
        self.assertEqual(last["status"], "inspection_error")
        self.assertEqual(last["motor_status"][0], 8)
        self.assertIn("fault/status 8", last["error"])
        self.assertTrue(self.bus.closed)

    def test_no_frames_remain_visible_while_waiting_then_stop_at_deadline(self):
        self.bus.respond = False
        with self.assertRaisesRegex(RuntimeError, "missing/stale"):
            self.observer().run(0.2)
        self.assertTrue(any(s["status"] == "inspection_waiting" for s in self.dashboard.states))
        self.assertEqual(self.dashboard.states[-1]["fresh_axis_coverage"], [False] * 7)
        self.assertEqual(self.dashboard.states[-1]["status"], "inspection_error")
        self.assertTrue(self.bus.closed)

    def test_feedback_becoming_stale_stops_after_complete_initial_snapshot(self):
        self.bus.once = True
        with self.assertRaisesRegex(RuntimeError, "missing/stale"):
            self.observer().run(0.3)
        self.assertEqual(self.dashboard.states[-1]["status"], "inspection_error")
        self.assertTrue(self.bus.closed)

    def test_overtemperature_is_visible_and_stops(self):
        self.bus.respond = False
        self.bus.incoming = [frame(self.profile, self.clock, temperature=75)]
        with self.assertRaisesRegex(RuntimeError, "overtemperature"):
            self.observer().run(0.05)
        self.assertEqual(self.dashboard.states[-1]["temperature_c"][0], 75)
        self.assertTrue(self.bus.closed)

    def test_queued_timestamp_and_invalid_motion_never_become_samples(self):
        observer = self.observer()
        queued = frame(self.profile, self.clock)
        queued.timestamp -= 0.2
        with self.assertRaisesRegex(RuntimeError, "stale/invalid"):
            observer.ingest(queued)
        with self.assertRaisesRegex(RuntimeError, "position outside limits"):
            observer.ingest(frame(self.profile, self.clock, position=2.5))
        with self.assertRaisesRegex(RuntimeError, "velocity outside"):
            observer.ingest(frame(self.profile, self.clock, velocity=20))
        self.assertIsNone(observer.samples[0])

    def test_position_jump_stops_independent_of_disabled_status(self):
        observer = self.observer()
        observer.ingest(frame(self.profile, self.clock))
        self.clock.now += 0.02
        with self.assertRaisesRegex(RuntimeError, "position jump"):
            observer.ingest(frame(self.profile, self.clock, position=0.8))

    def test_next_refresh_round_can_assemble_without_mixed_group_dynamics(self):
        self.profile.guards["can_skew_s"] = 0.005
        observer = self.observer()
        for i in range(7):
            observer.ingest(frame(self.profile, self.clock, i))
        self.assertTrue(observer.snapshot())
        self.clock.now += 0.02
        observer.ingest(frame(self.profile, self.clock, 0))
        self.assertFalse(observer.snapshot())
        observer.publish("inspection_waiting")
        self.assertFalse(self.dashboard.states[-1]["coherent_feedback"])
        self.assertEqual(self.dashboard.states[-1]["residual_nm"], [None] * 7)
        for i in range(1, 7):
            observer.ingest(frame(self.profile, self.clock, i))
        self.assertTrue(observer.snapshot())
        observer.publish("inspection_active")
        self.assertTrue(self.dashboard.states[-1]["coherent_feedback"])
        self.assertTrue(all(x is not None for x in self.dashboard.states[-1]["residual_nm"]))

    def test_group_that_never_finishes_cannot_extend_coherence_deadline(self):
        self.profile.guards["can_skew_s"] = 0.005
        observer = self.observer()
        for i in range(7):
            observer.ingest(frame(self.profile, self.clock, i))
        self.assertTrue(observer.snapshot())
        self.clock.now += 0.02
        observer.ingest(frame(self.profile, self.clock, 0))
        self.assertFalse(observer.snapshot())
        self.clock.now += 0.14
        observer.ingest(frame(self.profile, self.clock, 0))
        with self.assertRaisesRegex(RuntimeError, "missing/stale|too far apart"):
            observer.snapshot()

    def test_run_publishes_completed_groups_after_sequential_refresh_responses(self):
        self.profile.guards["can_skew_s"] = 0.005
        self.observer().run(0.05)
        active = [s for s in self.dashboard.states if s["status"] == "inspection_active"]
        self.assertGreaterEqual(len(active), 2)
        self.assertTrue(all(s["coherent_feedback"] for s in active))
        self.assertTrue(all(all(value is not None for value in s["model_nm"]) for s in active))

    def test_non_physical_frames_and_wrong_identity_are_ignored(self):
        observer = self.observer()
        for changed in (dict(is_rx=False), dict(is_remote_frame=True), dict(is_extended_id=True),
                        dict(is_fd=True), dict(arbitration_id=123), dict(data=b"short")):
            incoming = frame(self.profile, self.clock)
            for key, value in changed.items():
                setattr(incoming, key, value)
            observer.ingest(incoming)
        self.assertEqual(observer.counts, [0] * 7)

    def test_duration_validation_and_keyboard_interrupt_close_bus(self):
        for value in (0, -1, math.inf, math.nan):
            with self.assertRaises(ValueError):
                self.observer().run(value)
        with patch.object(self.bus, "recv", side_effect=KeyboardInterrupt):
            with self.assertRaises(KeyboardInterrupt):
                self.observer().run(0.05)
        self.assertTrue(self.bus.closed)
        self.assertEqual(self.dashboard.states[-1]["status"], "inspection_stopped")


if __name__ == "__main__":
    unittest.main()
