"""Software-only native command reception during blocking startup calls."""

import struct
import threading
import time
from types import SimpleNamespace

from test_feedback_runtime import ACTION_KEYS, UART, FakeLeader, FakeRobot, FixtureCase, LiveMonitor

from lerobot_robot_lumos_stararm102.button_protocol import CommandButtonState, NativeButtonGap
from stararm_lumos_feedback.control import Caps
from stararm_lumos_feedback.runtime import Session
from stararm_lumos_feedback.transport import Fashionstar, packet


def command_group(ids=range(6), mode=0x10):
    return b"".join(packet(24, struct.pack("<BBH", sid, mode, 6000)) for sid in ids)


class ReadCollisionUART(UART):
    """Yield in reads so an unprotected second reader causes a real overlap."""

    def __init__(self):
        super().__init__()
        self.reading = threading.Lock()
        self.overlap = False

    def read(self, size):
        if not self.reading.acquire(blocking=False):
            self.overlap = True
            raise AssertionError("Concurrent UART readers")
        try:
            threading.Event().wait(0.001)
            return super().read(size)
        finally:
            self.reading.release()


class StartupReceptionTests(FixtureCase):
    def startup_session(self, *, uart=None):
        uart = UART() if uart is None else uart
        button = CommandButtonState()
        button.feed(command_group())
        leader, robot = FakeLeader(uart), FakeRobot()
        leader.hd_button = button
        leader.config.button_enabled = False
        leader.get_action = lambda: dict(zip(ACTION_KEYS, [0] * 6 + [0.5]))
        session = Session(
            self.profile, robot, leader, Caps(), mode="follow", monitor=LiveMonitor(self.profile),
        )
        session.transport = Fashionstar(
            uart, range(7), enabled=False, timeout=0.03, command_button=button,
        )
        return session, uart, button

    def wait_for(self, predicate, timeout=0.2):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            threading.Event().wait(0.002)
        self.assertTrue(predicate(), "Startup reception did not progress before the test deadline")

    def delayed_bytes(self, uart, frames, delay=0.03):
        cancelled = threading.Event()

        def send():
            if not cancelled.wait(delay):
                uart.buffer.extend(frames)

        sender = threading.Thread(target=send, name="test-native-byte-sender")
        sender.start()

        def cleanup():
            cancelled.set()
            sender.join(timeout=0.5)

        self.addCleanup(cleanup)
        return sender

    def watch_receiver(self, session):
        original = session.transport.receive_native_commands
        workers, calls = set(), []
        owner = threading.current_thread()

        def receive():
            current = threading.current_thread()
            if current is not owner:
                workers.add(current)
            calls.append(current)
            return original()

        session.transport.receive_native_commands = receive
        return workers, calls

    def test_pending_group_is_settled_before_entering_blocking_startup(self):
        session, uart, button = self.startup_session()
        button.feed(command_group(range(5), mode=0x11))
        self.assertTrue(button.pending)
        sender = self.delayed_bytes(uart, command_group([5], mode=0x11))

        with session.native_startup_reception():
            self.assertFalse(button.pending)
            self.assertTrue(button.verified)
            self.assertTrue(button.locked)

        sender.join(timeout=0.5)
        self.assertEqual(button.group_timeout_s, 0.25)
        self.assertEqual(button.completed_groups, 2)
        self.assertEqual(uart.writes, [])

    def test_missing_pending_joint_prevents_entry_without_sending_a_follower_target(self):
        session, uart, button = self.startup_session()
        button.feed(command_group(range(5), mode=0x11))
        entered = False

        with self.assertRaises(NativeButtonGap) as raised:
            with session.native_startup_reception():
                entered = True

        self.assertFalse(entered)
        self.assertEqual(raised.exception.detail["received_joints"], list(range(5)))
        self.assertEqual(raised.exception.detail["group_timeout_s"], 0.25)
        self.assertEqual(session.robot.sent, [])
        self.assertEqual(uart.writes, [])

    def test_split_group_completes_during_a_startup_block_longer_than_the_group_deadline(self):
        session, uart, button = self.startup_session()
        workers, _ = self.watch_receiver(session)

        with session.native_startup_reception():
            uart.buffer.extend(command_group(range(5), mode=0x11))
            self.wait_for(lambda: button.pending)
            self.delayed_bytes(uart, command_group([5], mode=0x11))
            # The SDK can remain blocked after the complete group arrived.
            # Its blocking duration must not become the parser's RX gap.
            threading.Event().wait(0.3)
            session.check_cancelled()
            self.assertTrue(button.verified)
            self.assertFalse(button.pending)
            self.assertTrue(button.locked)

        self.assertTrue(workers)
        self.assertTrue(all(not worker.is_alive() for worker in workers))
        self.assertEqual(button.completed_groups, 2)
        self.assertEqual(button.group_timeout_s, 0.25)
        self.assertEqual(uart.writes, [])

    def test_background_partial_group_failure_is_raised_unchanged_and_receiver_is_joined(self):
        session, uart, button = self.startup_session()
        workers, _ = self.watch_receiver(session)
        observed = []

        with self.assertRaises(NativeButtonGap) as raised:
            with session.native_startup_reception():
                uart.buffer.extend(command_group(range(5), mode=0x11))
                self.wait_for(lambda: button.pending)
                threading.Event().wait(0.3)
                try:
                    session.check_cancelled()
                except NativeButtonGap as exc:
                    observed.append(exc)
                    raise

        self.assertEqual(observed, [raised.exception])
        self.assertIs(session._startup_rx_error, raised.exception)
        self.assertEqual(raised.exception.detail["received_joints"], list(range(5)))
        self.assertTrue(workers)
        self.assertTrue(all(not worker.is_alive() for worker in workers))
        self.assertEqual(session.robot.sent, [])
        self.assertEqual(uart.writes, [])

    def test_startup_receiver_and_monitor_queries_share_one_uart_reader_and_stop_on_exit(self):
        session, uart, _ = self.startup_session(uart=ReadCollisionUART())
        workers, calls = self.watch_receiver(session)

        with session.native_startup_reception():
            self.wait_for(lambda: bool(workers))
            for _ in range(5):
                samples = session.transport.query(range(7))
                self.assertEqual(set(samples), set(range(7)))

        count = len(calls)
        threading.Event().wait(0.02)
        self.assertFalse(uart.overlap)
        self.assertEqual(len(calls), count)
        self.assertTrue(all(not worker.is_alive() for worker in workers))
        self.assertTrue(uart.writes)
        self.assertTrue(all(frame[2] == 25 for frame in uart.writes))

    def test_context_exit_raises_the_original_receiver_fault_without_a_cooperative_check(self):
        session, uart, _ = self.startup_session()
        workers, _ = self.watch_receiver(session)

        with self.assertRaises(NativeButtonGap) as raised:
            with session.native_startup_reception():
                uart.buffer.extend(command_group(range(5), mode=0x11))
                self.wait_for(lambda: session._startup_rx_error is not None, timeout=0.5)
                # A blocking native constructor need not call check_cancelled.
                # The context handoff must still surface its receiver fault.

        self.assertIs(raised.exception, session._startup_rx_error)
        self.assertTrue(workers)
        self.assertTrue(all(not worker.is_alive() for worker in workers))
        self.assertEqual(session.robot.sent, [])

    def test_no_native_parser_uses_the_existing_synchronous_startup_path(self):
        session, _, _ = self.startup_session()
        # No serial reception API is needed at all on this ordinary path.
        session.transport = SimpleNamespace(command_button=None)
        threads_before = set(threading.enumerate())

        with session.native_startup_reception():
            session.check_cancelled()
            self.assertEqual(set(threading.enumerate()), threads_before)

        self.assertEqual(set(threading.enumerate()), threads_before)

    def test_run_receives_native_groups_during_both_preflight_and_sdk_constructor_blocks(self):
        session, uart, button = self.startup_session()
        workers = set()
        startup_writes = []

        def blocking_startup(stage):
            before = button.completed_groups
            uart.buffer.extend(command_group(range(5), mode=0x11))
            self.delayed_bytes(uart, command_group([5], mode=0x11))
            threading.Event().wait(0.3)
            self.assertEqual(button.completed_groups, before + 1, stage)
            self.assertTrue(button.locked)
            self.assertFalse(button.pending)
            self.assertEqual(session.robot.sent, [], stage)
            startup_writes.extend(uart.writes)
            uart.buffer.extend(command_group(mode=0x10))
            self.wait_for(lambda: button.locked is False)

        def preflight(transport):
            self.assertIs(transport.command_button, button)
            original = transport.receive_native_commands

            def track():
                if threading.current_thread() is not threading.main_thread():
                    workers.add(threading.current_thread())
                return original()

            transport.receive_native_commands = track
            blocking_startup("preflight")

        session.before_enable = preflight
        session.robot.connect = lambda: blocking_startup("SDK constructor")

        session.run(0.025)

        self.assertEqual(button.group_timeout_s, 0.25)
        self.assertEqual(button.completed_groups, 5)
        self.assertEqual(startup_writes, [])
        self.assertTrue(session.robot.sent)
        self.assertTrue(session.robot.cleaned)
        self.assertTrue(session.monitor.closed)
        self.assertTrue(workers)
        self.assertTrue(all(not worker.is_alive() for worker in workers))
