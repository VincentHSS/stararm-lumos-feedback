"""Receive the native HD board's six-joint command groups without replaying them.

Protocol reference: servodevelop/Star-Arm-102,
python-sdk/102-ld-hd-to-102-fl/dual-pair/stararm102_dual.py, commit
604d60b2ce207be4b372212ae33d80b7facad046 (ButtonCommandUART/CommandButtonState).
https://github.com/servodevelop/Star-Arm-102/blob/604d60b2ce207be4b372212ae33d80b7facad046/python-sdk/102-ld-hd-to-102-fl/dual-pair/stararm102_dual.py

The 0.25 s group deadline is an added local guard against joining old partial
groups. The reference uses a ten-second initial detection window; the local
adapter extends that initial passive wait to sixty seconds to allow handling
the arms and pressing the board. The runtime group deadline remains 0.25 s.
This module has no serial dependency and never transmits commands.
"""

import math
import struct
import time


class NativeButtonGap(RuntimeError):
    code = "button_board_command_incomplete"
    interface = None

    def __init__(self, joints, timeout):
        self.detail = {"received_joints": sorted(joints), "group_timeout_s": timeout}
        super().__init__(
            "Native HD button command group was incomplete; Leader hold state is unconfirmed"
        )


class ButtonDetectionCancelled(RuntimeError):
    code = "connection_cancelled"
    interface = None

    def __init__(self):
        super().__init__("Leader button detection was cancelled")


class CommandButtonState:
    """Decode RX only. A native board itself commands holding on joints 0–5."""

    id = None  # Native boards do not implement the legacy ID 7 monitor.

    def __init__(self, *, clock=time.monotonic, group_timeout_s=0.25):
        if not math.isfinite(group_timeout_s) or group_timeout_s <= 0:
            raise ValueError("group_timeout_s must be finite and positive")
        self.clock, self.group_timeout_s = clock, group_timeout_s
        self.locked = None
        self.verified = False
        self.sequence = 0
        self.completed_groups = 0
        self.last_seen = None
        self._buffer = bytearray()
        self._joints = set()
        self._mode = self._group_started = None
        self._uncertain = False
        self._prefix_uncertain = False

    @property
    def pending(self):
        return self._uncertain or bool(self._joints)

    def _check_deadline(self):
        if self._group_started is None or self.clock() - self._group_started < self.group_timeout_s:
            return
        error = NativeButtonGap(self._joints, self.group_timeout_s)
        self.locked, self.verified, self._uncertain = None, False, True
        self._buffer.clear()
        self._joints.clear()
        self._mode = self._group_started = None
        self._prefix_uncertain = False
        raise error

    def _discard_non_native_prefix(self):
        if self._prefix_uncertain:
            self._prefix_uncertain = self._uncertain = False
            self._group_started = None

    def update(self):
        """Check a partial group's deadline without emitting any output."""
        self._check_deadline()
        return self

    def feed(self, data: bytes):
        """Consume received bytes only; callers must never feed their own TX."""
        self._check_deadline()
        self._buffer.extend(data)
        while len(self._buffer) >= 2:
            header = self._buffer[:2]
            if header not in (b"\x12\x4c", b"\x05\x1c"):
                del self._buffer[0]
                continue
            if len(self._buffer) < 4:
                if header == b"\x12\x4c" and (len(self._buffer) == 2 or self._buffer[2] == 24):
                    if not self._uncertain:
                        self._prefix_uncertain = True
                    self._uncertain = True
                    if self._group_started is None:
                        self._group_started = self.clock()
                else:
                    self._discard_non_native_prefix()
                break
            # Only the native board's exact command is relevant. Responses
            # remain whole frames so bytes inside telemetry cannot be events.
            if header == b"\x12\x4c" and self._buffer[2:4] != b"\x18\x04":
                self._discard_non_native_prefix()
                del self._buffer[0]
                continue
            if header == b"\x12\x4c":
                # Even a truncated or corrupt native command can mean that
                # the board has already changed the real Leader hold state.
                # Freeze until a complete six-axis group confirms the state.
                self._uncertain = True
                self._prefix_uncertain = False
                if self._group_started is None:
                    self._group_started = self.clock()
            size = self._buffer[3] + 5
            if len(self._buffer) < size:
                break
            packet = bytes(self._buffer[:size])
            if sum(packet[:-1]) & 255 != packet[-1]:
                del self._buffer[0]
                continue
            del self._buffer[:size]
            if header != b"\x12\x4c":
                continue
            joint, mode, _power = struct.unpack("<BBH", packet[4:-1])
            if joint not in range(6) or mode not in (0x10, 0x11):
                continue
            if mode != self._mode:
                self._mode = mode
                self._joints.clear()
            self._joints.add(joint)
            if len(self._joints) == 6:
                self.completed_groups += 1
                locked = mode == 0x11
                if locked != self.locked:
                    self.sequence += 1
                self.locked, self.verified, self._uncertain = locked, True, False
                self.last_seen = self.clock()
                self._joints.clear()
                self._mode = self._group_started = None
        return self
