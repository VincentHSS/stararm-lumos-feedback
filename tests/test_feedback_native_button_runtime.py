"""Native HD edges through real receive parsing; synthetic UART/CAN only."""

import importlib.util
import math
import struct
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from test_feedback_runtime import UART, FakeLeader, FakeRobot, LiveMonitor, configuration, reply

from stararm_lumos_feedback.control import Caps
from stararm_lumos_feedback.runtime import ACTION_KEYS, LeaderReader, Session
from stararm_lumos_feedback.transport import Fashionstar, LeaderGap, packet

# The protocol is a pure module. Load it without importing the adapter package,
# whose registry requires LeRobot, so this suite also runs in the minimal env.
_path = Path(__file__).resolve().parents[1] / "src/lerobot_robot_lumos_stararm102/button_protocol.py"
_spec = importlib.util.spec_from_file_location("_feedback_native_button_runtime_protocol", _path)
_protocol = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = _protocol
_spec.loader.exec_module(_protocol)
CommandButtonState = _protocol.CommandButtonState
NativeButtonGap = _protocol.NativeButtonGap
ButtonDetectionCancelled = _protocol.ButtonDetectionCancelled


def commands(locked, joints=range(6), *, power=0):
    mode = 0x11 if locked else 0x10
    return b"".join(packet(24, struct.pack("<BBH", sid, mode, power)) for sid in joints)


class NativeUART(UART):
    def __init__(self):
        super().__init__()
        self.reset_calls = 0
        self.before_replies = b""
        self.fragment_size = None

    def reset_input_buffer(self):
        self.reset_calls += 1
        super().reset_input_buffer()

    def write(self, frame):
        if frame[2] == 25:
            self.buffer.extend(self.before_replies)
            self.before_replies = b""
        return super().write(frame)

    def read(self, size):
        if self.fragment_size is not None:
            size = min(size, self.fragment_size)
        return super().read(size)


class NativeLeader(FakeLeader):
    def __init__(self, uart, button):
        super().__init__(uart)
        self.hd_button = button
        self.config.button_enabled = True
        self.config.button_id = 7

    def get_action(self):
        # A native board has no monitor ID and issues its own Leader controls.
        self.leader.send_sync_servo_monitor(list(range(7)))
        self.hd_button.update()
        values = [math.radians(self.leader.servos[i].angle_monitor) for i in range(6)]
        values.append(self.leader.servos[6].angle_monitor / 100)
        return dict(zip(ACTION_KEYS, values))


class NativeReaderTests(unittest.TestCase):
    def setUp(self):
        self.now = 0.0
        self.button = CommandButtonState(clock=lambda: self.now)
        self.button.feed(commands(False))
        self.uart = NativeUART()
        self.leader = NativeLeader(self.uart, self.button)
        self.transport = Fashionstar(
            self.uart, range(7), enabled=False, timeout=0.01, command_button=self.button,
        )
        self.status = Mock()
        self.reader = LeaderReader(self.leader, self.transport, button_status=self.status)
        self.reader.install()
        self.addCleanup(self.reader.restore)

    def query(self):
        self.leader.leader.send_sync_servo_monitor(list(range(7)))

    def assert_queries_only(self):
        self.assertTrue(self.uart.writes)
        for frame in self.uart.writes:
            self.assertEqual(frame[2], 25)
            self.assertEqual(list(frame[7:-1]), list(range(7)))

    def test_lock_and_release_edges_between_queries_survive_without_uart_reset(self):
        self.query()
        self.assertFalse(self.reader.manual_hold)
        self.uart.buffer.extend(commands(True))
        self.query()
        self.assertTrue(self.reader.manual_hold)
        self.assertTrue(self.button.locked)
        self.uart.buffer.extend(commands(False))
        self.query()
        self.assertFalse(self.reader.manual_hold)
        self.assertFalse(self.button.locked)
        self.assertEqual(self.button.sequence, 3)
        self.assertEqual(self.uart.reset_calls, 0)
        self.assert_queries_only()

    def test_prequery_old_monitors_cannot_refresh_missing_new_joint_replies(self):
        self.uart.buffer.extend(commands(True))
        for sid in range(7):
            self.uart.buffer.extend(reply(sid, 999))
        self.uart.missing = set(range(7))
        with self.assertRaises(LeaderGap) as caught:
            self.query()
        self.assertEqual(caught.exception.missing_ids, tuple(range(7)))
        self.assertEqual(self.reader.latest, {})
        self.assertTrue(self.reader.manual_hold)
        self.assertTrue(self.button.verified)
        self.assertEqual(self.uart.reset_calls, 0)
        self.assert_queries_only()

    def test_fragmented_commands_mixed_with_monitor_replies_update_the_same_query(self):
        self.uart.fragment_size = 3
        self.transport.timeout = 0.2
        self.uart.before_replies = (
            b"noise" + commands(True, range(3)) + reply(0, 12) + commands(True, range(3, 6))
        )
        self.query()
        self.assertTrue(self.reader.manual_hold)
        self.assertTrue(self.button.locked)
        self.assertEqual(set(self.reader.latest), set(range(7)))
        self.assertEqual(self.reader.latest[0]["angle"], self.uart.angles[0])
        self.assert_queries_only()

    def test_monitor_header_inside_native_power_does_not_block_real_monitor_frames(self):
        self.uart.before_replies = b"".join(
            reply(sid, 99) + commands(True, (sid,), power=0x1C05) for sid in range(6)
        )
        self.query()
        self.assertTrue(self.button.locked)
        self.assertTrue(self.button.verified)
        self.assertTrue(self.reader.manual_hold)
        self.assertEqual(set(self.reader.latest), set(range(7)))
        self.assertEqual(
            [self.reader.latest[sid]["angle"] for sid in range(7)], self.uart.angles[:7],
        )
        self.assert_queries_only()

    def test_partial_native_frame_survives_from_prequery_drain_into_query_reads(self):
        native = commands(True)
        self.uart.buffer.extend(native[:4])
        self.uart.before_replies = native[4:]
        self.query()
        self.assertTrue(self.button.locked)
        self.assertTrue(self.reader.manual_hold)
        self.assertEqual(self.uart.reset_calls, 0)
        self.assert_queries_only()

    def test_bad_checksum_cannot_complete_a_group_and_partial_status_is_unknown(self):
        bad = bytearray(commands(True, (0,)))
        bad[-1] ^= 1
        self.uart.before_replies = bytes(bad) + commands(True, range(1, 6))
        self.query()
        self.assertTrue(self.button.pending)
        self.assertTrue(self.reader.manual_hold)
        self.assertIsNone(self.status.call_args.args[0])
        self.assertEqual(self.button.sequence, 1)
        self.uart.buffer.extend(commands(True, (0,)))
        self.query()
        self.assertTrue(self.button.locked)
        self.assertFalse(self.button.pending)
        self.assertIs(self.status.call_args.args[0], True)
        self.assert_queries_only()

    def test_four_byte_command_prefix_freezes_old_unlocked_state_and_times_out(self):
        self.uart.before_replies = commands(True)[:4]
        self.query()
        self.assertTrue(self.button.pending)
        self.assertTrue(self.reader.manual_hold)
        self.assertIsNone(self.status.call_args.args[0])
        queries = len(self.uart.writes)
        self.now = 0.3
        with self.assertRaises(NativeButtonGap) as caught:
            self.query()
        self.assertEqual(caught.exception.code, "button_board_command_incomplete")
        self.assertEqual(len(self.uart.writes), queries)
        self.assertIsNone(self.button.locked)
        self.assert_queries_only()

    def test_native_transport_never_replays_received_hold_commands(self):
        self.uart.buffer.extend(commands(True))
        self.query()
        self.transport.output([1500] * 7, [True] * 7)
        self.transport.release()
        self.assert_queries_only()
        self.assertEqual(self.transport.managed, set())

    def test_no_board_and_legacy_paths_still_reset_stale_input_and_monitor_normally(self):
        for legacy in (False, True):
            with self.subTest(legacy=legacy):
                uart = NativeUART()
                uart.buffer.extend(commands(True))
                leader = FakeLeader(uart)
                leader.config.button_enabled = legacy
                leader.config.button_id = 7
                leader.hd_button = SimpleNamespace(id=7) if legacy else None
                status = Mock()
                transport = Fashionstar(uart, range(7), enabled=False, timeout=0.01)
                reader = LeaderReader(leader, transport, button_status=status)
                reader.install()
                try:
                    ids = list(range(8 if legacy else 7))
                    leader.leader.send_sync_servo_monitor(ids)
                    self.assertEqual(uart.reset_calls, 1)
                    self.assertFalse(reader.manual_hold)
                    self.assertIsNone(reader.command_button)
                    if legacy:
                        status.assert_called_once_with(True)
                    else:
                        status.assert_not_called()
                finally:
                    reader.restore()


class NativeSessionTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.profile = configuration(Path(temporary.name))
        self.now = 0.0
        self.button = CommandButtonState(clock=lambda: self.now)
        self.button.feed(commands(False))
        self.uart = NativeUART()
        self.uart.angles[:6] = [10] * 6
        self.leader = NativeLeader(self.uart, self.button)
        self.robot = FakeRobot()
        self.monitor = LiveMonitor(self.profile)

    def session(self, mode="follow"):
        return Session(
            self.profile, self.robot, self.leader, Caps(), mode=mode, monitor=self.monitor,
        )

    def attach(self, session):
        session.transport = Fashionstar(
            self.uart, range(7), enabled=False, timeout=0.01, command_button=self.button,
        )
        session.reader = LeaderReader(self.leader, session.transport)
        session.reader.install()
        self.addCleanup(session.reader.restore)

    def test_partial_group_freezes_new_targets_until_complete_unlock(self):
        session = self.session()
        self.attach(session)
        session.step()
        self.assertEqual(len(self.robot.sent), 1)
        self.uart.buffer.extend(commands(True, range(3)))
        session.step()
        self.assertTrue(self.button.pending)
        self.assertTrue(session.reader.manual_hold)
        self.assertEqual(len(self.robot.sent), 1)
        self.uart.buffer.extend(commands(True, range(3, 6)))
        session.step()
        self.assertTrue(self.button.locked)
        self.assertEqual(len(self.robot.sent), 1)
        self.uart.buffer.extend(commands(False))
        session.step()
        self.assertFalse(session.reader.manual_hold)
        self.assertEqual(len(self.robot.sent), 2)
        self.assertTrue(all(frame[2] == 25 for frame in self.uart.writes))

    def test_partial_group_timeout_is_fatal_to_session_without_follower_targets(self):
        session = self.session()
        self.uart.buffer.extend(commands(True, range(3)))

        def advance_clock(_delay):
            self.now += 0.3
            return False

        with (
            patch.object(session, "start_watchdog"),
            patch.object(session.stop, "wait", side_effect=advance_clock),
        ):
            with self.assertRaises(NativeButtonGap) as caught:
                session.run(0.5)
        self.assertEqual(caught.exception.code, "button_board_command_incomplete")
        self.assertIs(session.error, caught.exception)
        self.assertIsNone(self.button.locked)
        self.assertTrue(self.button.pending)
        self.assertFalse(self.button.verified)
        self.assertEqual(self.robot.sent, [])
        self.assertTrue(self.robot.cleaned and self.monitor.closed)
        self.assertFalse(self.uart.is_open)
        self.assertTrue(all(frame[2] == 25 for frame in self.uart.writes))

    def test_unknown_native_state_prevents_a_first_target(self):
        self.button = CommandButtonState(clock=lambda: self.now)
        self.leader.hd_button = self.button
        session = self.session()
        self.attach(session)
        session.step()
        self.assertTrue(session.reader.manual_hold)
        self.assertEqual(self.robot.sent, [])

    def test_one_bad_checksum_command_freezes_targets_without_other_joint_commands(self):
        session = self.session()
        self.attach(session)
        bad = bytearray(commands(True, (0,)))
        bad[-1] ^= 1
        self.uart.before_replies = bytes(bad)
        session.step()
        self.assertTrue(self.button.pending)
        self.assertTrue(session.reader.manual_hold)
        self.assertEqual(self.robot.sent, [])
        self.now = 0.3
        with self.assertRaises(NativeButtonGap):
            session.step()
        self.assertEqual(self.robot.sent, [])
        self.assertTrue(all(frame[2] == 25 for frame in self.uart.writes))

    def test_native_force_mode_fails_before_sdk_connection_or_leader_output(self):
        session = self.session("run")
        with patch.object(self.robot, "connect") as sdk_connect:
            with self.assertRaisesRegex(RuntimeError, "Native HD button board"):
                session.run(0.02)
        sdk_connect.assert_not_called()
        self.assertIsNone(session.transport)
        self.assertIsNone(session.reader)
        self.assertEqual(self.uart.writes, [])
        self.assertEqual(self.robot.sent, [])

    def test_pending_native_detection_receives_cancellation_without_sdk_connect(self):
        session = self.session()

        def cancelled_detection():
            self.assertIs(self.leader.connection_cancel, session.stop)
            self.leader.connection_cancel.set()
            raise ButtonDetectionCancelled()

        with (
            patch.object(self.leader, "connect", side_effect=cancelled_detection),
            patch.object(self.robot, "connect") as sdk_connect,
        ):
            session.run(0.02)
        self.assertIsNone(session.error)
        self.assertTrue(session.stop.is_set())
        sdk_connect.assert_not_called()
        self.assertEqual(self.uart.writes, [])

    def test_detection_wait_notification_publishes_without_motion_or_force(self):
        session = self.session()
        dashboard = SimpleNamespace(publish=Mock(), button_status=Mock())
        session.dashboard = dashboard

        def waiting_then_cancelled():
            self.leader.button_waiting()
            self.leader.connection_cancel.set()
            raise ButtonDetectionCancelled()

        with (
            patch.object(self.leader, "connect", side_effect=waiting_then_cancelled),
            patch.object(self.robot, "connect") as sdk_connect,
        ):
            session.run(0.02)
        waiting = [
            call.kwargs for call in dashboard.publish.call_args_list
            if call.kwargs.get("status") == "button_board_waiting"
        ]
        self.assertEqual(len(waiting), 1)
        self.assertEqual(waiting[0]["requested_mw"], [0] * 7)
        self.assertEqual(waiting[0]["hold_requested"], [False] * 7)
        dashboard.button_status.assert_not_called()
        sdk_connect.assert_not_called()
        self.assertEqual(self.uart.writes, [])
        self.assertEqual(self.robot.sent, [])
        self.assertIsNone(session.error)


if __name__ == "__main__":
    unittest.main()
