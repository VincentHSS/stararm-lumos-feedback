"""Actual adapter methods with stubbed LeRobot base classes and device constructors.

This checks the feedback boundary without claiming a full LeRobot installation
test. Each scenario runs in a subprocess so stubs cannot pollute other tests.
"""

import importlib.util
import os
import subprocess
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

SCENARIO = r"""
from dataclasses import dataclass
import math
from pathlib import Path
import sys
import tempfile
import types
from unittest.mock import patch

# Only the LeRobot base/registry boundary is stubbed. Production adapter methods,
# mapping, adaptive filter, HD handling and bounded send_action execute unchanged.
@dataclass
class Config:
    id: str = "software-fixture"
    calibration_dir: str = ""
    @classmethod
    def register_subclass(cls, name):
        return lambda implementation: implementation

class Device:
    def __init__(self, config):
        pass

for name, exports in {
    "lerobot": {},
    "lerobot.cameras": {"CameraConfig": Config, "make_cameras_from_configs": lambda configs: {}},
    "lerobot.robots": {"Robot": Device, "RobotConfig": Config},
    "lerobot.teleoperators": {"Teleoperator": Device, "TeleoperatorConfig": Config},
}.items():
    module=types.ModuleType(name)
    module.__dict__.update(exports)
    sys.modules[name]=module

from test_feedback_runtime import configuration, UART, LiveMonitor
from stararm_lumos_feedback.control import Caps
from stararm_lumos_feedback.runtime import Session, ACTION_KEYS
from lerobot_robot_lumos_stararm102.config import LumosConfig, StarArm102Config
from lerobot_robot_lumos_stararm102.lumos import Lumos, SDKCleanupError
from lerobot_robot_lumos_stararm102.stararm102 import StarArm102

class Manager:
    instances=[]
    def __init__(self, uart):
        self.uart=uart
        self.servos={i:types.SimpleNamespace(angle_monitor=999) for i in range(8)}
        self.direct_controls=[]
        self.ping_calls=[]
        self.instances.append(self)
    def ping(self, sid):
        self.ping_calls.append(sid)
        if sys.argv[1] == "no_board" and sid == 7:
            raise AssertionError("Absent button board must not be pinged")
        if sys.argv[1] == "id0_retry" and sid == 0 and self.ping_calls.count(0) == 1:
            return False
        return True
    def send_sync_servo_monitor(self, ids, realtime=False):
        raise AssertionError("Feedback must replace cached reads")
    def stop_on_control_mode(self, *args):
        self.direct_controls.append(args)

class Arm:
    instances=[]
    def __init__(self, **kwargs):
        self.commands=[]
        self.grip_commands=[]
        self.cleaned=False
        self.instances.append(self)
        if mode == "cancel_init": session.stop.set()
    def get_joint_positions(self): return [0]*6
    def get_gripper_position(self): return .5
    def get_gripper_state(self):
        return types.SimpleNamespace(position=.557976, distance_m=.04,
            motor_position_rad=-.7014613048553252, motor_velocity_rad_s=0,
            feedback_age_ms=0, feedback_valid=True, enabled=True, error_code=1)
    def set_joint_raw(self, q, velocities):
        self.commands.append(q.copy())
        if mode == "cancel_joint": session.stop.set()
        return True
    def setGripperPosition(self, value): self.grip_commands.append(value); return True
    def cleanup(self):
        self.cleaned=True
        if mode == "cleanup_fail": raise RuntimeError("synthetic native cleanup fault")

class Camera:
    def __init__(self, fail=False):
        self.is_connected=False
        self.disconnect_calls=0
        self.fail=fail
    def connect(self): self.is_connected=True
    def disconnect(self):
        self.disconnect_calls+=1
        self.is_connected=False
        if self.fail: raise RuntimeError("synthetic camera cleanup fault")

sdk=types.ModuleType("startouchclass"); sdk.SingleArm=Arm
servo=types.ModuleType("fashionstar_uart_sdk"); servo.UartServoManager=Manager
sys.modules["startouchclass"]=sdk; sys.modules["fashionstar_uart_sdk"]=servo

with tempfile.TemporaryDirectory() as folder:
    root=Path(folder); profile=configuration(root)
    rc=LumosConfig(sdk_root=str(root/"sdk"), mapping_file=str(root/"calibration.json"),
                   joint_limits_file=str(root/"limits.json"), can_interface="mock-can", execute=True,
                   startup_wait_s=.001)
    tc=StarArm102Config(mapping_file=str(root/"calibration.json"), port="mock-uart", button_enabled=sys.argv[1]!="no_board",
                       signs=[-1,1,1,1,-1,-1])
    robot,leader=Lumos(rc),StarArm102(tc)
    uart=UART(); uart.angles[:6]=[10]*6
    monitor=LiveMonitor(profile,[3]*7)
    mode=sys.argv[1]
    if mode == "hd": uart.angles[7]=180
    if mode == "cleanup_fail":
        cameras=[Camera(fail=True),Camera()]
        robot.cameras=dict(enumerate(cameras))
    with patch("serial.Serial",return_value=uart):
        session=Session(profile,robot,leader,Caps(),mode="monitor" if mode=="monitor" else "run",monitor=monitor)
        if mode == "cleanup_fail":
            try:
                session.run(.12)
            except SDKCleanupError as exc:
                assert exc.code == "sdk_cleanup_failed"
                assert "actuator state is unconfirmed" in str(exc)
                assert "synthetic native cleanup fault" in str(exc)
                assert isinstance(exc.__cause__, RuntimeError)
            else:
                raise AssertionError("Real Session must surface failed SDK cleanup")
        else:
            session.run(.12)
    assert Arm.instances[-1].cleaned and not uart.is_open
    if mode == "cleanup_fail":
        assert all(camera.disconnect_calls==1 for camera in cameras)
        assert robot.arm is None and robot._previous_target is None and robot._last_send is None
        assert monitor.closed and session.stop.is_set()
        assert session.transport.managed == set()
        sys.exit(0)
    if mode in ("cancel_init", "cancel_joint"):
        assert len(Arm.instances[-1].commands) == (1 if mode == "cancel_joint" else 0)
        assert not Arm.instances[-1].grip_commands
        assert session.error is None
        assert all(frame[2] == 25 for frame in uart.writes)
        sys.exit(0)
    if mode=="hd":
        assert not Arm.instances[-1].commands
        commands=[f for f in uart.writes if f[2]==24]
        import struct
        controls=[struct.unpack("<BBH",f[4:-1]) for f in commands]
        assert all((sid,0x11,1500) in controls for sid in range(6))
        assert (6,0x11,1500) not in controls
    else:
        assert Arm.instances[-1].commands
        first=Arm.instances[-1].commands[0]
        assert first[0]<0 and first[1]>0 and first[4]<0 and first[5]<0
        assert max(map(abs,first)) <= math.radians(2)+1e-9
        assert session.target==[robot.last_sent_action[k] for k in ACTION_KEYS]
    if mode=="monitor": assert all(f[2]==25 for f in uart.writes)
    if mode=="no_board":
        assert Manager.instances[-1].ping_calls == list(range(7))
        assert all(7 not in frame[7:-1] for frame in uart.writes if frame[2]==25)
    if mode=="id0_retry":
        assert Manager.instances[-1].ping_calls == [0,0,*range(1,8)]
"""


class AdapterBoundaryTests(unittest.TestCase):
    def scenario(self, mode):
        env = os.environ.copy()
        env["PYTHONPATH"] = os.pathsep.join((str(Path(__file__).parent), env.get("PYTHONPATH", "")))
        result = subprocess.run(
            [sys.executable, "-B", "-c", SCENARIO, mode], env=env, text=True, capture_output=True, timeout=10
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_actual_adapter_mapping_rate_limit_and_feedback_loop(self):
        self.scenario("run")

    def test_actual_adapter_monitor_never_writes_force(self):
        self.scenario("monitor")

    def test_actual_hd_hold_arbitration_excludes_gripper(self):
        self.scenario("hd")

    def test_no_button_board_does_not_ping_or_query_its_id(self):
        self.scenario("no_board")

    def test_first_servo_probe_can_retry_after_serial_open(self):
        self.scenario("id0_retry")

    def test_cancel_after_sdk_constructor_prevents_targets_and_cleans_up(self):
        self.scenario("cancel_init")

    def test_cancel_after_joint_write_prevents_gripper_write(self):
        self.scenario("cancel_joint")

    def test_native_cleanup_failure_surfaces_after_all_resources_are_closed(self):
        self.scenario("cleanup_fail")


class HDButtonProbeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source = Path(__file__).resolve().parents[1] / "src/lerobot_robot_lumos_stararm102/_legacy.py"
        spec = importlib.util.spec_from_file_location("hd_button_probe_fixture", source)
        cls.legacy = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.legacy)

    def manager(self, replies, servo_id=7):
        replies = iter(replies)
        events = []
        queued = [b"old complete ping reply"]
        partial = [b"old partial reply"]
        serial_pending = [b"old UART reply"]

        def reset_input():
            events.append("uart reset")
            serial_pending.clear()

        def empty_partial():
            events.append("partial reset")
            partial.clear()

        def ping(button_id):
            self.assertEqual(button_id, servo_id)
            self.assertFalse(serial_pending, "A probe retained old UART bytes")
            self.assertFalse(partial, "A probe retained an old partial SDK packet")
            self.assertFalse(queued, "A probe retained an old complete SDK packet")
            events.append(("ping", button_id))
            # Simulate a reply left behind after a failed probe; the next
            # attempt must discard it before issuing another request.
            serial_pending.append(b"previous probe UART reply")
            partial.append(b"previous probe partial reply")
            queued.append(b"previous probe complete reply")
            return next(replies)

        return SimpleNamespace(
            uart=SimpleNamespace(reset_input_buffer=reset_input),
            pkt_buffer=SimpleNamespace(empty_buffer=empty_partial, packet_bytes_list=queued),
            ping=ping,
        ), events

    def test_first_failed_probe_retries_then_stops_on_success(self):
        manager, events = self.manager([False, True])
        button = self.legacy.HDButton(manager, 7)
        self.assertEqual(button.id, 7)
        self.assertIsNone(button.locked)
        self.assertEqual(events, ["uart reset", "partial reset", ("ping", 7)] * 2)

    def test_fresh_servo_probe_retries_with_caches_cleared(self):
        manager, events = self.manager([False, True], servo_id=0)
        self.assertIs(self.legacy.ping_fresh(manager, 0), True)
        self.assertEqual(events, ["uart reset", "partial reset", ("ping", 0)] * 2)

    def test_three_failed_probes_raise_structured_unavailable(self):
        manager, events = self.manager([False] * 3)
        with self.assertRaises(self.legacy.HDButtonUnavailable) as raised:
            self.legacy.HDButton(manager, 7)
        error = raised.exception
        self.assertIsInstance(error, RuntimeError)
        self.assertEqual(error.code, "button_board_no_reply")
        self.assertIsNone(error.interface)
        self.assertEqual(error.detail, {"button_id": 7, "attempts": 3})
        self.assertIn("HD button board ID 7 did not reply", str(error))
        self.assertIn("only when absent", str(error))
        self.assertEqual(events, ["uart reset", "partial reset", ("ping", 7)] * 3)

    def test_optional_cache_interfaces_are_not_required(self):
        calls = []

        def ping(button_id):
            calls.append(button_id)
            return True

        self.legacy.HDButton(SimpleNamespace(ping=ping), 7)
        self.assertEqual(calls, [7])


if __name__ == "__main__":
    unittest.main()
