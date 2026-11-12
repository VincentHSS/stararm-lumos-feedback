"""Native HD protocol and actual adapter detection with no hardware access."""

import importlib.util
import struct
import sys
import tempfile
import types
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

SOURCE = Path(__file__).resolve().parents[1] / "src/lerobot_robot_lumos_stararm102"


def load_source(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


protocol = load_source("native_hd_protocol_fixture", SOURCE / "button_protocol.py")


def packet(joint, mode=0x10, power=0, *, response=False):
    payload = struct.pack("<BBH", joint, mode, power)
    data = (b"\x05\x1c" if response else b"\x12\x4c") + bytes((24, len(payload))) + payload
    return data + bytes((sum(data) & 255,))


def group(mode=0x10):
    return b"".join(packet(joint, mode) for joint in range(6))


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class NativeButtonProtocolTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.button = protocol.CommandButtonState(clock=self.clock)

    def assert_unknown(self):
        self.assertIsNone(self.button.locked)
        self.assertFalse(self.button.verified)

    def test_initial_state_unknown_and_has_no_legacy_monitor_id(self):
        self.assert_unknown()
        self.assertIsNone(self.button.id)
        self.assertFalse(self.button.pending)
        self.assertEqual(self.button.sequence, 0)

    def test_complete_groups_update_state_and_duplicate_states_keep_sequence(self):
        self.button.feed(group(0x11))
        self.assertTrue(self.button.locked)
        self.assertTrue(self.button.verified)
        self.assertFalse(self.button.pending)
        self.assertEqual(self.button.sequence, 1)
        self.button.feed(group(0x11))
        self.assertEqual(self.button.sequence, 1)
        self.button.feed(group())
        self.assertIs(self.button.locked, False)
        self.assertEqual(self.button.sequence, 2)

    def test_bytewise_fragmentation_and_noise_still_require_all_six_joints(self):
        data = b"unrelated\x12\x00\x4c" + group()
        for byte in data[:-1]:
            self.button.feed(bytes((byte,)))
        self.assert_unknown()
        self.assertTrue(self.button.pending)
        self.button.feed(data[-1:])
        self.assertIs(self.button.locked, False)
        self.assertFalse(self.button.pending)

    def test_duplicate_axes_and_mixed_modes_do_not_confirm_a_state(self):
        self.button.feed(packet(0) * 6)
        self.assert_unknown()
        self.button.feed(b"".join(packet(joint, 0x11) for joint in range(1, 6)))
        self.assert_unknown()
        self.button.feed(packet(0, 0x11))
        self.assertTrue(self.button.locked)

    def test_telemetry_response_payload_cannot_masquerade_as_native_commands(self):
        payload = group()
        response = b"\x05\x1c" + bytes((25, len(payload))) + payload
        response += bytes((sum(response) & 255,))
        self.button.feed(response + packet(7) + packet(0, 0x12))
        self.assert_unknown()
        # Invalid native commands do not confirm a state either.
        self.assertTrue(self.button.pending)

    def test_response_only_does_not_create_pending_state(self):
        self.button.feed(b"".join(packet(joint, response=True) for joint in range(6)))
        self.assert_unknown()
        self.assertFalse(self.button.pending)

    def test_short_native_prefix_freezes_previous_confirmed_state(self):
        self.button.feed(group())
        self.button.feed(b"\x12\x4c\x18")
        self.assertTrue(self.button.pending)
        self.button.feed(b"\x04")
        self.assertTrue(self.button.pending)
        self.assertIs(self.button.locked, False)
        self.clock.advance(0.26)
        with self.assertRaises(protocol.NativeButtonGap) as raised:
            self.button.update()
        self.assert_unknown()
        self.assertTrue(self.button.pending)
        self.assertEqual(raised.exception.code, "button_board_command_incomplete")
        self.assertEqual(raised.exception.detail, {"received_joints": [], "group_timeout_s": 0.25})

    def test_two_and_three_byte_native_prefixes_freeze_until_the_group_completes(self):
        for size in (2, 3):
            with self.subTest(size=size):
                button = protocol.CommandButtonState(clock=self.clock)
                button.feed(group())
                transition = group(0x11)
                button.feed(transition[:size])
                self.assertTrue(button.pending)
                self.assertIs(button.locked, False)
                button.feed(transition[size:])
                self.assertFalse(button.pending)
                self.assertTrue(button.locked)
                self.assertEqual(button.completed_groups, 2)

    def test_disproved_short_prefix_does_not_turn_other_requests_into_native_events(self):
        self.button.feed(group())
        request = b"\x12\x4c\x19\x03\x16\x01\x00"
        request += bytes((sum(request) & 255,))
        self.button.feed(request[:2])
        self.assertTrue(self.button.pending)
        self.button.feed(request[2:])
        self.assertFalse(self.button.pending)
        self.assertIs(self.button.locked, False)
        self.assertEqual(self.button.completed_groups, 1)
        # A genuine native partial group cannot be dismissed by unrelated RX.
        self.button.feed(packet(0, 0x11))
        self.button.feed(request)
        self.assertTrue(self.button.pending)

    def test_corrupt_checksum_freezes_until_a_complete_fresh_group(self):
        self.button.feed(group())
        bad = packet(0, 0x11)
        self.button.feed(bad[:-1] + bytes((bad[-1] ^ 1,)))
        self.assertTrue(self.button.pending)
        self.assertIs(self.button.locked, False)
        self.clock.advance(0.26)
        with self.assertRaises(protocol.NativeButtonGap):
            self.button.update()
        self.assert_unknown()
        self.assertTrue(self.button.pending)
        self.button.feed(group(0x11))
        self.assertTrue(self.button.verified)
        self.assertTrue(self.button.locked)
        self.assertFalse(self.button.pending)

    def test_duplicate_axes_and_mode_switch_do_not_extend_deadline(self):
        self.button.feed(packet(0))
        self.clock.advance(0.15)
        self.button.feed(packet(0) + packet(1, 0x11))
        self.clock.advance(0.11)
        with self.assertRaises(protocol.NativeButtonGap) as raised:
            self.button.update()
        self.assertEqual(raised.exception.detail["received_joints"], [1])

    def test_late_continuation_cannot_complete_a_stale_group(self):
        self.button.feed(b"".join(packet(joint) for joint in range(5)))
        self.clock.advance(0.26)
        with self.assertRaises(protocol.NativeButtonGap):
            self.button.feed(packet(5))
        self.assert_unknown()
        self.button.feed(packet(5))
        self.assert_unknown()
        self.button.feed(b"".join(packet(joint) for joint in range(5)))
        self.assertIs(self.button.locked, False)

    def test_nonfinite_or_nonpositive_deadline_is_rejected(self):
        for deadline in (0, -1, float("inf"), float("nan")):
            with self.subTest(deadline=deadline), self.assertRaises(ValueError):
                protocol.CommandButtonState(group_timeout_s=deadline)


class UART:
    def __init__(self, clock):
        self.clock = clock
        self.is_open = True
        self.rx = bytearray()
        self.scheduled = []
        self.writes = []

    @property
    def in_waiting(self):
        while self.scheduled and self.scheduled[0][0] <= self.clock():
            _, data = self.scheduled.pop(0)
            self.rx.extend(data)
        return len(self.rx)

    def read(self, count):
        data = bytes(self.rx[:count])
        del self.rx[:count]
        return data

    def write(self, data):
        self.writes.append(data)
        raise AssertionError("Native detection must not transmit control commands")

    def reset_input_buffer(self):
        self.rx.clear()

    def close(self):
        self.is_open = False


class Manager:
    def __init__(self, uart, *, legacy=False):
        self.uart, self.legacy = uart, legacy
        self.pings = []
        self.monitor_queries = []
        self.pkt_buffer = SimpleNamespace(empty_buffer=lambda: None, packet_bytes_list=[])
        self.servos = {sid: SimpleNamespace(angle_monitor=0) for sid in range(8)}

    def ping(self, sid):
        self.pings.append(sid)
        return sid != 7 or self.legacy

    def stop_on_control_mode(self, *_args):
        raise AssertionError("Detection must never replay the board's hold/release outputs")

    def send_sync_servo_monitor(self, ids, realtime=False):
        self.monitor_queries.append(list(ids))
        raise AssertionError("Native get_action must not use an unobserved SDK query")


class Cancel:
    def __init__(self, clock):
        self.clock, self.cancelled = clock, False

    def is_set(self):
        return self.cancelled

    def wait(self, seconds):
        self.clock.advance(seconds)
        self.cancelled = True
        return True


class ScheduledCancel:
    def __init__(self, clock, at=None):
        self.clock, self.at, self.cancelled = clock, at, False

    def is_set(self):
        return self.cancelled or (self.at is not None and self.clock() >= self.at)

    def set(self):
        self.cancelled = True

    def wait(self, seconds):
        self.clock.advance(seconds)
        return self.is_set()


class NativeButtonAdapterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        name = "native_hd_adapter_fixture"
        package = types.ModuleType(name)
        package.__path__ = [str(SOURCE)]
        teleoperators = types.ModuleType("lerobot.teleoperators")

        class Teleoperator:
            def __init__(self, _config):
                pass

        teleoperators.Teleoperator = Teleoperator
        config = types.ModuleType(name + ".config")
        config.StarArm102Config = SimpleNamespace
        with patch.dict(sys.modules, {name: package, name + ".config": config,
                                      "lerobot.teleoperators": teleoperators}):
            cls.adapter = load_source(name + ".stararm102", SOURCE / "stararm102.py")

    def setUp(self):
        self.clock = Clock()
        self.uart = UART(self.clock)
        self.manager = Manager(self.uart)
        config = SimpleNamespace(joint_map=list(range(6)), signs=[1] * 6, gain=1,
                                 button_id=7, button_enabled=True, port="fixture", gripper=True,
                                 invert_gripper=False)
        self.leader = self.adapter.StarArm102(config)
        self.leader.calibrate = lambda: None
        self.leader.uart, self.leader.leader = self.uart, self.manager
        factory = self.adapter.CommandButtonState
        self.time_patch = patch.object(self.adapter, "time", SimpleNamespace(
            monotonic=self.clock, sleep=self.clock.advance))
        self.button_patch = patch.object(self.adapter, "CommandButtonState",
                                         side_effect=lambda: factory(clock=self.clock))
        self.time_patch.start()
        self.button_patch.start()
        self.addCleanup(self.time_patch.stop)
        self.addCleanup(self.button_patch.stop)

    def connect(self):
        self.leader.uart = self.leader.leader = None
        serial = SimpleNamespace(Serial=lambda **_kwargs: self.uart, PARITY_NONE="N")
        sdk = SimpleNamespace(UartServoManager=lambda _uart: self.manager)
        with patch.dict(sys.modules, {"serial": serial, "fashionstar_uart_sdk": sdk}):
            self.leader.connect()

    def test_legacy_probe_success_does_not_probe_board_twice_or_notify_native_wait(self):
        self.manager.legacy = True
        self.leader.button_waiting = lambda: self.fail("Legacy board must not await native press")
        button = self.leader._detect_button()
        self.assertEqual(button.id, 7)
        self.assertEqual(self.manager.pings, [7])
        self.assertFalse(self.uart.writes)

    def test_connect_native_fallback_waits_after_probes_and_never_replays_outputs(self):
        def on_waiting():
            self.assertEqual(self.manager.pings, [*range(7), 7, 7, 7])
            self.uart.rx.extend(group(0x11))

        self.leader.button_waiting = on_waiting
        self.connect()
        self.assertTrue(self.leader.hd_button.locked)
        self.assertIsNone(self.leader.hd_button.id)
        self.assertTrue(self.leader.hd_button.verified)
        self.assertFalse(self.uart.writes)

    def test_no_board_never_probes_or_notifies_native_wait(self):
        self.leader.config.button_enabled = False
        self.leader.button_waiting = lambda: self.fail("Absent board must not await native press")
        self.connect()
        self.assertEqual(self.manager.pings, list(range(7)))
        self.assertIsNone(self.leader.hd_button)

    def test_native_detection_timeout_reports_passive_window_and_closes_uart(self):
        with self.assertRaises(self.adapter._legacy.HDButtonUnavailable) as raised:
            self.connect()
        self.assertEqual(raised.exception.detail, {"button_id": 7, "attempts": 3, "passive_s": 60.0})
        self.assertGreaterEqual(self.clock(), 60)
        self.assertLess(self.clock(), 60.1)
        self.assertFalse(self.uart.is_open)
        self.assertIsNone(self.leader.leader)
        self.assertFalse(self.uart.writes)

    def test_initial_native_wait_accepts_press_after_original_ten_second_window(self):
        self.leader.button_waiting = lambda: self.uart.scheduled.append((45.0, group()))
        self.connect()
        self.assertTrue(self.leader.hd_button.verified)
        self.assertIs(self.leader.hd_button.locked, False)
        self.assertGreaterEqual(self.clock(), 45)
        self.assertLess(self.clock(), 60)
        self.assertFalse(self.uart.writes)

    def test_explicit_cancellable_wait_accepts_fresh_complete_group_after_sixty_seconds(self):
        self.leader.button_wait_until_cancel = True
        self.leader.connection_cancel = ScheduledCancel(self.clock)

        def on_waiting():
            # A partial old press cannot combine with the delayed last axis.
            self.uart.scheduled = [(0, group()[:-9]), (75, packet(5)), (76, group(0x11))]

        self.leader.button_waiting = on_waiting
        self.connect()
        self.assertTrue(self.leader.hd_button.verified)
        self.assertTrue(self.leader.hd_button.locked)
        self.assertGreaterEqual(self.clock(), 76)
        self.assertLess(self.clock(), 76.1)
        self.assertFalse(self.uart.writes)

    def test_explicit_cancellable_wait_stops_after_sixty_seconds_without_sdk_or_force(self):
        from test_feedback_runtime import FakeRobot, LiveMonitor, configuration

        from stararm_lumos_feedback.control import Caps
        from stararm_lumos_feedback.runtime import Session

        with tempfile.TemporaryDirectory() as folder:
            profile = configuration(Path(folder))
            robot = FakeRobot()
            preflight = Mock()
            session = Session(profile, robot, self.leader, Caps(), mode="follow",
                              monitor=LiveMonitor(profile), before_enable=preflight)
            self.leader.button_wait_until_cancel = True
            session.stop = ScheduledCancel(self.clock, at=75)
            self.leader.uart = self.leader.leader = None
            serial = SimpleNamespace(Serial=lambda **_kwargs: self.uart, PARITY_NONE="N")
            sdk = SimpleNamespace(UartServoManager=lambda _uart: self.manager)
            with (
                patch.dict(sys.modules, {"serial": serial, "fashionstar_uart_sdk": sdk}),
                patch.object(robot, "connect") as sdk_connect,
            ):
                session.run()
            sdk_connect.assert_not_called()
            preflight.assert_not_called()
            self.assertIsNone(session.error)
            self.assertGreaterEqual(self.clock(), 75)
            self.assertLess(self.clock(), 75.1)
            self.assertFalse(self.uart.is_open)
            self.assertIsNone(self.leader.leader)
            self.assertFalse(self.uart.writes)
            self.assertEqual(robot.sent, [])

    def test_indefinite_wait_requires_cancellation_before_serial_or_board_queries(self):
        self.leader.button_wait_until_cancel = True
        for event in (None, SimpleNamespace(is_set=lambda: False), SimpleNamespace(wait=lambda _: False)):
            with self.subTest(event=event):
                self.leader.connection_cancel = event
                self.leader.uart = self.leader.leader = None
                self.leader.calibrate = Mock(side_effect=AssertionError("Must validate before calibration"))
                with self.assertRaisesRegex(ValueError, "requires a connection cancellation event"):
                    self.leader.connect()
                with self.assertRaisesRegex(ValueError, "requires a connection cancellation event"):
                    self.leader._detect_button()
                self.assertEqual(self.manager.pings, [])
                self.assertFalse(self.uart.writes)

    def test_button_wait_option_rejects_non_boolean_before_queries(self):
        self.leader.button_wait_until_cancel = 1
        self.leader.uart = self.leader.leader = None
        with self.assertRaisesRegex(ValueError, "must be a boolean"):
            self.leader.connect()
        self.assertEqual(self.manager.pings, [])
        self.assertFalse(self.uart.writes)

    def test_native_detection_recovers_with_new_full_group_after_partial_timeout(self):
        def on_waiting():
            self.uart.scheduled = [(0, b"\x12\x4c\x18\x04"), (0.4, group())]

        self.leader.button_waiting = on_waiting
        self.connect()
        self.assertTrue(self.leader.hd_button.verified)
        self.assertIs(self.leader.hd_button.locked, False)
        self.assertGreaterEqual(self.clock(), 0.4)
        self.assertFalse(self.uart.writes)

    def test_cancellation_while_waiting_is_prompt_and_closes_uart(self):
        self.leader.connection_cancel = Cancel(self.clock)
        with self.assertRaises(self.adapter.ButtonDetectionCancelled) as raised:
            self.connect()
        self.assertEqual(raised.exception.code, "connection_cancelled")
        self.assertLess(self.clock(), 0.1)
        self.assertFalse(self.uart.is_open)
        self.assertFalse(self.uart.writes)

    def native_action_fixture(self):
        from test_feedback_runtime import UART as TelemetryUART

        self.uart = TelemetryUART()
        self.manager = Manager(self.uart)
        self.leader.uart, self.leader.leader = self.uart, self.manager
        self.leader.hd_button = self.adapter.CommandButtonState()
        self.leader.hd_button.feed(group())
        self.leader.mapping = dict(
            leader_home_deg=[0] * 6,
            leader_range_min_deg=[-180] * 6,
            leader_range_max_deg=[180] * 6,
            lumos_home_rad=[0] * 6,
            gripper_range=dict(closed_deg=0, open_deg=100),
        )

    def assert_native_action_rejected_without_query(self):
        writes = list(self.uart.writes)
        with self.assertRaises(self.adapter.NativeButtonRuntimeRequired) as raised:
            self.leader.get_action()
        self.assertEqual(raised.exception.code, "native_button_requires_feedback_runtime")
        self.assertIn("follow/monitor", str(raised.exception))
        self.assertFalse(self.manager.monitor_queries)
        self.assertEqual(self.uart.writes, writes)

    def test_ordinary_native_get_action_is_rejected_before_any_serial_query(self):
        self.native_action_fixture()
        self.assertFalse(self.leader.native_button_rx_active)
        self.assert_native_action_rejected_without_query()

    def test_real_native_receiver_enables_actions_and_restore_revokes_permission(self):
        from stararm_lumos_feedback.runtime import ACTION_KEYS, LeaderReader
        from stararm_lumos_feedback.transport import Fashionstar

        self.native_action_fixture()
        transport = Fashionstar(
            self.uart, range(7), enabled=False, timeout=0.01,
            command_button=self.leader.hd_button,
        )
        reader = LeaderReader(self.leader, transport)
        reader.install()
        self.addCleanup(reader.restore)
        self.assertTrue(self.leader.native_button_rx_active)
        action = self.leader.get_action()
        self.assertEqual(set(action), set(ACTION_KEYS))
        self.assertEqual(action["gripper.pos"], 0.5)
        self.assertFalse(reader.manual_hold)
        self.uart.buffer.extend(group(0x11))
        self.leader.get_action()
        self.assertTrue(reader.manual_hold)
        self.assertTrue(self.leader.hd_button.locked)
        self.assertTrue(self.uart.writes)
        self.assertTrue(all(frame[2] == 25 for frame in self.uart.writes))
        reader.restore()
        self.assertFalse(self.leader.native_button_rx_active)
        self.assert_native_action_rejected_without_query()

    def test_receiver_for_another_native_state_does_not_enable_actions(self):
        from stararm_lumos_feedback.runtime import LeaderReader
        from stararm_lumos_feedback.transport import Fashionstar

        self.native_action_fixture()
        transport = Fashionstar(
            self.uart, range(7), enabled=False, timeout=0.01,
            command_button=self.adapter.CommandButtonState(),
        )
        reader = LeaderReader(self.leader, transport)
        reader.install()
        self.addCleanup(reader.restore)
        self.assertFalse(self.leader.native_button_rx_active)
        self.assert_native_action_rejected_without_query()

    def test_disconnect_revokes_native_receive_gate_without_outputs(self):
        self.native_action_fixture()
        self.leader.native_button_rx_active = True
        self.leader.disconnect()
        self.assertFalse(self.leader.native_button_rx_active)
        self.assertFalse(self.uart.is_open)
        self.assertFalse(self.uart.writes)


if __name__ == "__main__":
    unittest.main()
