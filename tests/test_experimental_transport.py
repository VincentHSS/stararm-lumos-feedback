"""Packet-level limits and native HD ownership for experimental damping."""

import importlib.util
import math
import struct
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from stararm_lumos_feedback.transport import Fashionstar, LeaderGap, packet

PROTOCOL_NAME = "lerobot_robot_lumos_stararm102.button_protocol"
PROTOCOL_PATH = Path(__file__).resolve().parents[1] / "src/lerobot_robot_lumos_stararm102/button_protocol.py"
SPEC = importlib.util.spec_from_file_location("experimental_native_protocol_fixture", PROTOCOL_PATH)
PROTOCOL = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PROTOCOL)


def button_group(locked=False, joints=range(6)):
    return b"".join(
        packet(24, struct.pack("<BBH", sid, 0x11 if locked else 0x10, 1500 if locked else 0))
        for sid in joints
    )


def commands(uart):
    result = []
    for frame in uart.writes:
        if frame[2] != 24:
            continue
        assert frame[:4] == b"\x12\x4c\x18\x04"
        assert sum(frame[:-1]) & 255 == frame[-1]
        result.append(struct.unpack("<BBH", frame[4:-1]))
    return result


class UART:
    def __init__(self):
        self.buffer, self.writes = bytearray(), []
        self.after_write = None

    @property
    def in_waiting(self):
        return len(self.buffer)

    def read(self, size):
        result = bytes(self.buffer[:size])
        del self.buffer[:size]
        return result

    def write(self, frame):
        self.writes.append(frame)
        if self.after_write is not None:
            self.after_write(frame)
        if frame[2] == 25:
            for sid in frame[7:-1]:
                payload = struct.pack("<BHHHHBih", sid, 12000, 100, 0, 1000, 0, 0, 0)
                reply = b"\x05\x1c" + bytes((22, len(payload))) + payload
                self.buffer.extend(reply + bytes((sum(reply) & 255,)))
        return len(frame)

    def reset_input_buffer(self):
        self.buffer.clear()


class ExperimentalTransportTests(unittest.TestCase):
    def setUp(self):
        # Load the real pure parser without importing the LeRobot registry.
        self.module_patch = patch.dict(sys.modules, {PROTOCOL_NAME: PROTOCOL})
        self.module_patch.start()
        self.addCleanup(self.module_patch.stop)
        self.now = 10.0
        self.uart = UART()
        self.button = PROTOCOL.CommandButtonState(clock=lambda: self.now)

    def transport(self, **kwargs):
        return Fashionstar(
            self.uart, range(7), enabled=True, timeout=.01, clock=lambda: self.now,
            command_button=self.button, max_output_mw=200, protect_native_hold=True, **kwargs,
        )

    def release_board(self):
        self.button.feed(button_group())

    def assert_no_arm_commands(self):
        self.assertFalse(any(sid in range(6) for sid, _mode, _power in commands(self.uart)))

    def test_constructor_requires_a_real_rx_parser_and_valid_output_contract(self):
        for value in (False, True, None, "200", 0, -1, .5, 1501, math.nan, math.inf, -math.inf):
            with self.subTest(max_output_mw=value), self.assertRaises(ValueError):
                Fashionstar(self.uart, range(7), enabled=True, timeout=.01, max_output_mw=value)
        for value in (None, 0, 1, "true"):
            with self.subTest(protect_native_hold=value), self.assertRaises(ValueError):
                Fashionstar(self.uart, range(7), enabled=True, timeout=.01, protect_native_hold=value)
            with self.subTest(allow_hold=value), self.assertRaises(ValueError):
                Fashionstar(self.uart, range(7), enabled=True, timeout=.01, allow_hold=value)
        for parser in (None, SimpleNamespace(pending=False, locked=False, verified=True)):
            with self.subTest(parser=parser), self.assertRaisesRegex(ValueError, "CommandButtonState"):
                Fashionstar(self.uart, range(7), enabled=True, timeout=.01,
                            protect_native_hold=True, command_button=parser)
        self.assertEqual(self.uart.writes, [])

    def test_unknown_state_suppresses_arm_including_zero_releases_and_forces_gripper_zero(self):
        transport = self.transport()
        transport.managed.update(range(7))
        transport.last_commands.update(dict.fromkeys(range(7), (0x12, 100)))
        transport.output([0, 200, 200, 200, 200, 200, 200], [False]*7)
        self.assertEqual(commands(self.uart), [(6, 0x10, 0)])
        self.assertEqual(transport.managed, {6})
        self.assertEqual(transport.last_commands, {6: (0x10, 0)})
        self.assertIsNone(self.button.locked)
        self.assertFalse(self.button.verified)

    def test_initial_unknown_without_prior_arm_output_does_not_claim_residual_output(self):
        transport = self.transport()
        self.assertFalse(transport.observe_native_state())
        self.assertFalse(transport.native_output_unknown)
        self.assertEqual(transport.release(), [])
        self.assertEqual(self.uart.writes, [])

    def test_partial_after_software_output_reports_unconfirmed_cleanup_until_complete_hold(self):
        self.release_board()
        transport = self.transport()
        transport.output([100]*7, [False]*7)
        self.uart.writes.clear()
        self.uart.buffer.extend(button_group(True, [0]))
        self.assertFalse(transport.observe_native_state())
        self.assertTrue(transport.native_output_unknown)
        failures = transport.release()
        self.assertIn("native Leader transition incomplete; residual software output unconfirmed", failures)
        self.assertEqual(commands(self.uart), [(6, 0x10, 0)])
        self.assertTrue(transport.native_output_unknown)
        self.uart.buffer.extend(button_group(True, range(1, 6)))
        self.assertFalse(transport.observe_native_state())
        self.assertTrue(self.button.locked)
        self.assertFalse(transport.native_output_unknown)
        self.assertEqual(transport.release(), [])
        self.assertEqual(commands(self.uart), [(6, 0x10, 0)])

    def test_same_state_complete_release_group_clears_unknown_without_timestamp_or_edge_change(self):
        self.release_board()
        transport = self.transport()
        transport.output([100]*7, [False]*7)
        initial_seen, initial_sequence = self.button.last_seen, self.button.sequence
        self.uart.buffer.extend(button_group(False, [0]))
        self.assertFalse(transport.observe_native_state())
        self.assertTrue(transport.native_output_unknown)
        self.uart.buffer.extend(button_group(False, range(1, 6)))
        self.assertTrue(transport.observe_native_state())
        self.assertFalse(transport.native_output_unknown)
        self.assertEqual(self.button.completed_groups, 2)
        self.assertEqual(self.button.last_seen, initial_seen)
        self.assertEqual(self.button.sequence, initial_sequence)

    def test_corrupt_native_command_after_output_cannot_claim_clean_software_release(self):
        self.release_board()
        transport = self.transport()
        transport.output([100]*7, [False]*7)
        self.uart.writes.clear()
        corrupt = button_group(True, [0])
        self.uart.buffer.extend(corrupt[:-1] + bytes((corrupt[-1] ^ 1,)))
        self.assertFalse(transport.observe_native_state())
        self.assertTrue(transport.native_output_unknown)
        self.assertIn("native Leader transition incomplete; residual software output unconfirmed", transport.release())
        self.assert_no_arm_commands()
        self.uart.buffer.extend(button_group(True))
        self.assertFalse(transport.observe_native_state())
        self.assertFalse(transport.native_output_unknown)
        self.assertEqual(transport.release(), [])

    def test_old_verified_release_cache_cannot_clear_unknown_after_rx_drain_failure(self):
        self.release_board()
        transport = self.transport()
        transport.output([100]*7, [False]*7)
        self.uart.buffer.extend(b"noise"*1000)
        with self.assertRaises(LeaderGap):
            transport.observe_native_state()
        self.assertTrue(transport.native_output_unknown)
        self.assertTrue(self.button.verified)
        self.assertFalse(self.button.pending)
        self.uart.buffer.clear()
        self.assertFalse(transport.observe_native_state())
        self.assertTrue(transport.native_output_unknown)
        self.assertIn("native Leader transition incomplete; residual software output unconfirmed", transport.release())
        self.uart.buffer.extend(button_group())
        self.assertTrue(transport.observe_native_state())
        self.assertFalse(transport.native_output_unknown)

    def test_observation_consumes_rx_and_checks_native_state_without_any_transmission(self):
        transport = self.transport()
        self.uart.buffer.extend(button_group(True))
        self.assertFalse(transport.observe_native_state())
        self.assertTrue(self.button.locked)
        self.uart.buffer.extend(button_group())
        self.assertTrue(transport.observe_native_state())
        self.assertEqual(self.uart.writes, [])

    def test_locked_and_partial_groups_suppress_arm_writes(self):
        for received in (button_group(True), button_group(False, range(5)),
                         b"\x12\x4c", b"\x12\x4c\x18", b"\x12\x4c\x18\x04"):
            with self.subTest(received=received):
                self.uart = UART()
                self.button = PROTOCOL.CommandButtonState(clock=lambda: self.now)
                self.uart.buffer.extend(received)
                transport = self.transport()
                transport.output([200]*7, [False]*7)
                self.assert_no_arm_commands()
                self.assertEqual(commands(self.uart), [(6, 0x10, 0)])

    def test_queued_two_or_three_byte_native_prefix_blocks_previous_released_state(self):
        for size in (2, 3):
            with self.subTest(size=size):
                self.uart = UART()
                self.button = PROTOCOL.CommandButtonState(clock=lambda: self.now)
                self.release_board()
                transport = self.transport()
                transport.output([100]*7, [False]*7)
                self.uart.writes.clear()
                transition = button_group(True)
                self.uart.buffer.extend(transition[:size])
                self.assertFalse(transport.observe_native_state())
                self.assertTrue(self.button.pending)
                self.assertTrue(transport.native_output_unknown)
                self.assertIn("native Leader transition incomplete; residual software output unconfirmed", transport.release())
                self.assert_no_arm_commands()
                self.uart.buffer.extend(transition[size:])
                self.assertFalse(transport.observe_native_state())
                self.assertTrue(self.button.locked)
                self.assertFalse(transport.native_output_unknown)

    def test_only_complete_observed_release_group_authorizes_arm(self):
        transport = self.transport()
        self.uart.buffer.extend(button_group(False, range(5)))
        transport.output([200]*7, [False]*7)
        self.assert_no_arm_commands()
        self.uart.buffer.extend(button_group(False, [5]))
        transport.output([200]*7, [False]*7)
        self.assertIs(self.button.locked, False)
        self.assertTrue(self.button.verified)
        self.assertEqual(commands(self.uart)[1:], [(sid, 0x12, 200) for sid in range(7)])

    def test_power_is_clamped_at_packet_boundary_and_zero_retains_release_mode(self):
        self.release_board()
        transport = self.transport()
        transport.output([0, 199.6, 200, 200.8, 1000, .5, 3000], [False]*7)
        self.assertEqual(commands(self.uart), [
            (0, 0x10, 0), (1, 0x12, 200), (2, 0x12, 200), (3, 0x12, 200),
            (4, 0x12, 200), (5, 0x10, 0), (6, 0x12, 200),
        ])

    def test_fractional_output_cap_cannot_round_above_it(self):
        self.release_board()
        transport = Fashionstar(self.uart, [0], enabled=True, timeout=.01, max_output_mw=1.5)
        transport.output([2], [False])
        self.assertEqual(commands(self.uart), [(0, 0x12, 1)])

    def test_nonfinite_and_negative_power_trip_before_any_partial_damping_update(self):
        for invalid in (math.nan, math.inf, -math.inf, -1, False, None):
            with self.subTest(invalid=invalid):
                self.uart = UART()
                self.release_board()
                transport = self.transport()
                with self.assertRaisesRegex(ValueError, "finite and nonnegative"):
                    transport.output([100]*6 + [invalid], [False]*7)
                self.assertIsNotNone(transport.fault)
                self.assertEqual(self.uart.writes, [])

    def test_low_cap_rejects_hold_instead_of_translating_it(self):
        self.release_board()
        transport = self.transport()
        with self.assertRaisesRegex(ValueError, "HOLD"):
            transport.output([0]*7, [True] + [False]*6)
        self.assertIsNotNone(transport.fault)
        self.assertEqual(self.uart.writes, [])

    def test_trip_and_cleanup_do_not_release_a_native_hold_or_partial_group(self):
        for received in (button_group(True), button_group(True, [0]), b"\x12\x4c\x18\x04"):
            with self.subTest(received=received):
                self.uart = UART()
                self.button = PROTOCOL.CommandButtonState(clock=lambda: self.now)
                self.release_board()
                transport = self.transport()
                transport.output([100]*7, [False]*7)
                self.uart.writes.clear()
                self.uart.buffer.extend(received)
                transport.trip("fixture stop")
                self.assertEqual(commands(self.uart), [(6, 0x10, 0)])
                self.assertEqual(transport.managed, set())
                self.assertEqual(transport.last_commands, {})
                transport.release()
                self.assertEqual(commands(self.uart), [(6, 0x10, 0)])

    def test_native_lock_queued_after_query_is_observed_before_any_arm_write(self):
        self.release_board()
        transport = self.transport()
        transport.query(range(7))
        self.uart.buffer.extend(button_group(True))
        transport.output([100]*7, [False]*7)
        self.assert_no_arm_commands()
        self.assertTrue(self.button.locked)

    def test_native_lock_arriving_during_write_guard_cannot_be_overwritten(self):
        self.release_board()
        def guard():
            self.uart.buffer.extend(button_group(True))
        transport = self.transport(before_control_write=guard)
        transport.output([100]*7, [False]*7)
        self.assert_no_arm_commands()
        self.assertTrue(self.button.locked)

    def test_native_lock_between_axis_writes_stops_remaining_axes_without_cleanup_release(self):
        self.release_board()
        transport = self.transport()
        def after_write(frame):
            if frame[2] == 24 and frame[4] == 0:
                self.uart.buffer.extend(button_group(True))
        self.uart.after_write = after_write
        transport.output([100]*7, [False]*7)
        self.assertEqual(commands(self.uart), [(0, 0x12, 100), (6, 0x10, 0)])
        self.assertEqual(transport.managed, {6})
        transport.trip("fixture stop")
        self.assertEqual(commands(self.uart)[2:], [(6, 0x10, 0)])

    def test_serial_transmit_is_never_fed_back_as_a_native_button_group(self):
        self.release_board()
        observed_at = self.button.last_seen
        self.now += .1
        transport = self.transport()
        transport.output([0]*7, [False]*7)
        self.assertEqual(len(commands(self.uart)), 7)
        self.assertEqual(self.button.sequence, 1)
        self.assertEqual(self.button.last_seen, observed_at)

    def test_native_lock_revokes_earlier_failed_release_retry_ownership(self):
        self.release_board()
        transport = self.transport()
        transport.output([100]*7, [False]*7)
        self.uart.writes.clear()
        original_write = self.uart.write
        def partial_with_native_lock(frame):
            size = original_write(frame)
            if frame[2] == 24 and frame[4] == 0:
                self.uart.buffer.extend(button_group(True))
                return 2
            return size
        self.uart.write = partial_with_native_lock
        failures = transport.release()
        self.assertTrue(failures)
        self.assertTrue(self.button.locked)
        self.assertEqual(transport.managed, set())
        self.assertEqual(transport.last_commands, {})
        before = commands(self.uart)
        transport.release()
        self.assertEqual(commands(self.uart), before)

    def test_input_byte_budget_trips_without_releasing_unconfirmed_arm(self):
        self.release_board()
        transport = self.transport()
        transport.managed.update(range(7))
        self.uart.buffer.extend(b"noise"*1000)
        with self.assertRaisesRegex(LeaderGap, "did not drain"):
            transport.output([100]*7, [False]*7)
        self.assertIsNotNone(transport.fault)
        self.assert_no_arm_commands()
        self.assertEqual(commands(self.uart), [(6, 0x10, 0)])

    def test_input_time_budget_is_checked_after_receiving_a_group(self):
        self.release_board()
        transport = self.transport()
        transport.managed.update(range(7))
        self.uart.buffer.extend(button_group(True))
        ticks = iter((0, .006, .012))
        transport.clock = lambda: next(ticks, .012)
        with self.assertRaisesRegex(LeaderGap, "did not drain"):
            transport.output([100]*7, [False]*7)
        self.assertIsNotNone(transport.fault)
        self.assert_no_arm_commands()
        self.assertEqual(commands(self.uart), [(6, 0x10, 0)])

    def test_expired_partial_group_trips_without_releasing_native_arm(self):
        self.release_board()
        transport = self.transport()
        transport.managed.update(range(7))
        self.button.feed(button_group(True, [0]))
        self.now += .3
        with self.assertRaises(PROTOCOL.NativeButtonGap):
            transport.output([100]*7, [False]*7)
        self.assertIsNotNone(transport.fault)
        self.assert_no_arm_commands()
        self.assertEqual(commands(self.uart), [(6, 0x10, 0)])

    def test_ordinary_default_path_preserves_hold_and_monitor_behavior(self):
        transport = Fashionstar(self.uart, range(7), enabled=True, timeout=.01)
        transport.output([0, 1600, 200, 0, 0, 0, 0], [True] + [False]*6)
        self.assertEqual(commands(self.uart)[:3], [(0, 0x11, 1500), (1, 0x12, 1500), (2, 0x12, 200)])
        transport.release()
        self.assertEqual(commands(self.uart)[7:], [(sid, 0x10, 0) for sid in range(7)])
        self.uart.writes.clear()
        transport = Fashionstar(self.uart, range(7), enabled=False, timeout=.01)
        transport.output([1500]*7, [True]*7)
        transport.release()
        self.assertEqual(self.uart.writes, [])

    def test_native_experiment_without_trial_cap_preserves_packet_power_and_ownership_limits(self):
        transport = Fashionstar(
            self.uart, range(7), enabled=True, timeout=.01, clock=lambda: self.now,
            command_button=self.button, max_output_mw=1500,
            protect_native_hold=True, allow_hold=False,
        )
        transport.output([1200] * 7, [False] * 7)
        self.assert_no_arm_commands()
        self.assertEqual(commands(self.uart), [(6, 0x10, 0)])
        self.uart.writes.clear()
        self.release_board()
        transport.output([0, 400, 1200, 1499.6, 1500, 2000, 1500], [False] * 7)
        self.assertEqual(commands(self.uart), [
            (0, 0x10, 0), (1, 0x12, 400), (2, 0x12, 1200),
            (3, 0x12, 1500), (4, 0x12, 1500), (5, 0x12, 1500), (6, 0x12, 1500),
        ])
        self.uart.writes.clear()
        self.uart.buffer.extend(button_group(True))
        transport.output([1200] * 7, [False] * 7)
        self.assert_no_arm_commands()
        self.assertEqual(commands(self.uart), [(6, 0x10, 0)])

    def test_no_hold_contract_remains_enforced_at_full_protocol_power(self):
        self.release_board()
        transport = Fashionstar(
            self.uart, range(7), enabled=True, timeout=.01, clock=lambda: self.now,
            command_button=self.button, max_output_mw=1500,
            protect_native_hold=True, allow_hold=False,
        )
        with self.assertRaisesRegex(ValueError, "HOLD is disabled"):
            transport.output([1200] * 7, [True] + [False] * 6)
        self.assertFalse(any(mode == 0x11 for _sid, mode, _power in commands(self.uart)))


if __name__ == "__main__":
    unittest.main()
