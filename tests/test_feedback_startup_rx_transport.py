"""Native startup reception stays passive, bounded and on the original deadline."""

import sys
import unittest
from unittest.mock import Mock, patch

from test_experimental_transport import PROTOCOL, PROTOCOL_NAME, UART, button_group

from stararm_lumos_feedback.transport import Fashionstar, LeaderGap


class StartupNativeReceiveTests(unittest.TestCase):
    def setUp(self):
        self.module_patch = patch.dict(sys.modules, {PROTOCOL_NAME: PROTOCOL})
        self.module_patch.start()
        self.addCleanup(self.module_patch.stop)
        self.now = 10.0
        self.uart = UART()
        self.uart.reset_input_buffer = Mock(side_effect=AssertionError("Native RX must not reset input"))
        self.button = PROTOCOL.CommandButtonState(clock=lambda: self.now)
        self.transport = Fashionstar(
            self.uart, range(7), enabled=True, timeout=0.01, clock=lambda: self.now,
            command_button=self.button, protect_native_hold=True,
        )

    def assert_passive(self):
        self.assertEqual(self.uart.writes, [])
        self.uart.reset_input_buffer.assert_not_called()

    def sleep(self, delay):
        self.now += delay

    def test_receive_preserves_split_zero_to_four_and_final_joint_without_transmitting(self):
        self.uart.buffer.extend(button_group(False, range(5)))
        self.transport.receive_native_commands()
        self.assertTrue(self.button.pending)
        self.assertFalse(self.button.verified)
        self.assertEqual(self.button._joints, set(range(5)))
        started = self.button._group_started
        self.now += 0.002
        self.uart.buffer.extend(button_group(False, [5]))
        self.transport.receive_native_commands()
        self.assertFalse(self.button.pending)
        self.assertTrue(self.button.verified)
        self.assertIs(self.button.locked, False)
        self.assertEqual(self.button.completed_groups, 1)
        self.assertEqual(started, 10.0)
        self.assert_passive()

    def test_settle_receives_later_final_joint_without_monitor_queries_or_deadline_reset(self):
        self.uart.buffer.extend(button_group(False, range(5)))
        cancel_check = Mock()
        starts = []

        def later_frame(delay):
            starts.append(self.button._group_started)
            self.sleep(delay)
            if len(starts) == 3:
                self.uart.buffer.extend(button_group(False, [5]))

        with patch("stararm_lumos_feedback.transport.time.sleep", side_effect=later_frame):
            self.transport.settle_native_commands(cancel_check)
        self.assertEqual(starts, [10.0] * 3)
        self.assertGreaterEqual(cancel_check.call_count, 4)
        self.assertTrue(self.button.verified)
        self.assertIs(self.button.locked, False)
        self.assertEqual(self.button.completed_groups, 1)
        self.assert_passive()

    def test_genuinely_missing_final_joint_raises_original_gap_and_forgets_native_output(self):
        self.transport.managed.update(range(7))
        self.transport.last_commands.update(dict.fromkeys(range(7), (0x12, 100)))
        self.uart.buffer.extend(button_group(False, range(5)))
        with patch("stararm_lumos_feedback.transport.time.sleep", side_effect=self.sleep):
            with self.assertRaises(PROTOCOL.NativeButtonGap) as raised:
                self.transport.settle_native_commands()
        self.assertEqual(raised.exception.detail, {"received_joints": list(range(5)), "group_timeout_s": 0.25})
        self.assertGreaterEqual(self.now, 10.25)
        self.assertLess(self.now, 10.251)
        self.assertEqual(self.transport.fault, str(raised.exception))
        self.assertTrue(self.transport.native_output_unknown)
        self.assertEqual(self.transport.managed, {6})
        self.assertEqual(self.transport.last_commands, {6: (0x12, 100)})
        self.assertFalse(self.button.verified)
        self.assertEqual(self.button.completed_groups, 0)
        self.assert_passive()

    def test_settle_uses_only_remaining_original_group_time(self):
        self.button.feed(button_group(False, [0]))
        self.now += 0.2499
        self.uart.buffer.extend(button_group(False, range(1, 5)))
        with patch("stararm_lumos_feedback.transport.time.sleep", side_effect=self.sleep) as sleep:
            with self.assertRaises(PROTOCOL.NativeButtonGap):
                self.transport.settle_native_commands()
        sleep.assert_called_once_with(0.0005)
        self.assertLess(self.now, 10.251)
        self.assert_passive()

    def test_expired_group_is_rejected_before_reading_queued_final_joint(self):
        self.button.feed(button_group(False, range(5)))
        self.now += 0.251
        final = button_group(False, [5])
        self.uart.buffer.extend(final)
        with patch.object(self.uart, "read", wraps=self.uart.read) as read:
            with self.assertRaises(PROTOCOL.NativeButtonGap):
                self.transport.receive_native_commands()
        read.assert_not_called()
        self.assertEqual(bytes(self.uart.buffer), final)
        self.assertIsNotNone(self.transport.fault)
        self.assertEqual(self.button.completed_groups, 0)
        self.assert_passive()

    def test_receive_has_total_4096_byte_budget(self):
        self.uart.buffer.extend(b"x" * 5000)
        with patch.object(self.uart, "read", wraps=self.uart.read) as read:
            with self.assertRaisesRegex(LeaderGap, "did not drain"):
                self.transport.receive_native_commands()
        read.assert_called_once_with(4096)
        self.assertEqual(len(self.uart.buffer), 904)
        self.assertIsNotNone(self.transport.fault)
        self.assert_passive()

    def test_receive_checks_time_budget_after_consuming_bytes(self):
        self.uart.buffer.extend(button_group(True))
        original_read = self.uart.read

        def slow_read(size):
            received = original_read(size)
            self.now += 0.011
            return received

        with patch.object(self.uart, "read", side_effect=slow_read):
            with self.assertRaisesRegex(LeaderGap, "did not drain"):
                self.transport.receive_native_commands()
        self.assertIsNotNone(self.transport.fault)
        self.assert_passive()

    def test_settle_cancellation_propagates_without_resetting_pending_group(self):
        self.uart.buffer.extend(button_group(False, range(5)))
        cancellation = RuntimeError("cancelled before SDK initialization")
        cancel_check = Mock(side_effect=[None, cancellation])
        with patch("stararm_lumos_feedback.transport.time.sleep", side_effect=self.sleep):
            with self.assertRaises(RuntimeError) as raised:
                self.transport.settle_native_commands(cancel_check)
        self.assertIs(raised.exception, cancellation)
        self.assertEqual(cancel_check.call_count, 2)
        self.assertTrue(self.button.pending)
        self.assertEqual(self.button._group_started, 10.0)
        self.assertEqual(self.button.completed_groups, 0)
        self.assert_passive()

    def test_no_native_parser_does_not_read_write_reset_or_invoke_cancellation(self):
        self.transport.command_button = None
        self.uart.buffer.extend(button_group(True))
        cancel_check = Mock()
        with patch.object(self.uart, "read", wraps=self.uart.read) as read:
            self.transport.receive_native_commands()
            self.transport.settle_native_commands(cancel_check)
        read.assert_not_called()
        cancel_check.assert_not_called()
        self.assertEqual(bytes(self.uart.buffer), button_group(True))
        self.assert_passive()

    def test_public_receive_serializes_parser_and_reads_with_transport_owner(self):
        self.uart.buffer.extend(button_group(True))
        original_read = self.uart.read
        original_update = self.button.update

        def locked_read(size):
            self.assertTrue(self.transport.lock._is_owned())
            return original_read(size)

        def locked_update():
            self.assertTrue(self.transport.lock._is_owned())
            return original_update()

        with (
            patch.object(self.uart, "read", side_effect=locked_read),
            patch.object(self.button, "update", side_effect=locked_update),
        ):
            self.transport.receive_native_commands()
        self.assertIs(self.button.locked, True)
        self.assert_passive()
