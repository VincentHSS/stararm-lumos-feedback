"""Software-only CAN/serial/control/session tests; never connect real hardware."""

import http.client
import io
import json
import math
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unittest
import xml.etree.ElementTree as ET
from contextlib import redirect_stdout
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from test_feedback_model import fixture

from stararm_lumos_feedback.cli import main, template
from stararm_lumos_feedback.config import Profile, verify_sdk_imports
from stararm_lumos_feedback.control import Caps, Controller
from stararm_lumos_feedback.dashboard import Dashboard
from stararm_lumos_feedback.runtime import (
    ACTION_KEYS,
    Session,
    SessionCancelled,
    SocketCANError,
    session_lock,
    validate_socketcan,
)
from stararm_lumos_feedback.transport import CanMonitor, Fashionstar, LeaderGap, Sample, packet


def configuration(root):
    sdk = root / "sdk"
    (sdk / "src/config").mkdir(parents=True)
    (sdk / "interface_py").mkdir()
    (sdk / "interface_py/startouchclass.py").write_text("# fake SDK resource, never imported\n")
    (sdk / "src/libstartouch.so").write_bytes(b"fake native resource, never loaded")
    (sdk / "interface_py/startouch.test.so").write_bytes(b"fake pybind resource, never loaded")
    ET.ElementTree(fixture()).write(root / "model.urdf")
    (sdk / "src/config/robot_kinematics.yaml").write_text("gripper_control: {type: TypeLJ}\n")
    mapping = dict(
        format_version=1,
        gripper_enabled=True,
        leader_port="mock-uart",
        can_interface="mock-can",
        joint_map=list(range(6)),
        leader_home_deg=[0] * 6,
        leader_range_min_deg=[-90] * 6,
        leader_range_max_deg=[90] * 6,
        lumos_home_rad=[0] * 6,
        gripper_range=dict(closed_deg=0, open_deg=100),
    )
    (root / "calibration.json").write_text(json.dumps(mapping))
    (root / "limits.json").write_text(json.dumps(dict(lower_rad=[-1.9] * 6, upper_rad=[1.9] * 6)))
    session = dict(
        robot=dict(
            type="lumos",
            sdk_root="sdk",
            can_interface="mock-can",
            mapping_file="calibration.json",
            joint_limits_file="limits.json",
            gripper=True,
        ),
        teleop=dict(
            type="stararm102",
            port="mock-uart",
            mapping_file="calibration.json",
            gripper=True,
            joint_map=list(range(6)),
            button_enabled=True,
        ),
        fps=100,
    )
    (root / "session.json").write_text(json.dumps(session))
    # Synthetic test values, intentionally not distributed as hardware tuning.
    axis = dict(
        feedback_id=16,
        motor_id=1,
        p_max=12.5,
        v_max=30,
        t_max=10,
        sdk_scale=1,
        sdk_offset=0,
        urdf_scale=1,
        urdf_offset=0,
        torque_scale=1,
        max_age_s=0.15,
        max_temperature_c=70,
        max_jump=0.5,
        max_velocity=10,
        max_acceleration=200,
        sdk_tolerance=0.02,
        limit_margin=0.001,
    )
    policy = dict(
        deadzone_nm=0.2,
        velocity_margin=0.1,
        acceleration_margin=0.01,
        full_scale_nm=2,
        filter_s=0.02,
        acceleration_filter_s=0.04,
        confirm_s=0.04,
        release_s=0.04,
        rise_mw_s=10000,
        fall_mw_s=20000,
        hold_error=0.2,
        hold_load_nm=1.5,
        hold_speed=0.1,
        hold_confirm_s=0.06,
        hold_progress=0.01,
        hold_release_error=0.05,
        hold_release_load_nm=0.5,
        hold_release_s=0.06,
    )
    profile = dict(
        version=2,
        session_file="session.json",
        urdf="model.urdf",
        sdk_config="sdk/src/config/robot_kinematics.yaml",
        gravity=[0, 0, -9.80665],
        payload=None,
        mass_coverage="urdf_includes_typelj",
        evidence="Synthetic fixture only; no physical hardware claims",
        gripper_transform=dict(
            type="typelj_sdk_04041254",
            motor_margin_rad=0.001,
            max_motor_jump_rad=0.5,
            max_motor_velocity_rad_s=10,
            sdk_motor_tolerance_rad=0.02,
        ),
        axes=[dict(axis, feedback_id=16 + i, motor_id=i + 1) for i in range(7)],
        policy=[dict(policy) for _ in range(7)],
        guards=dict(
            can_bitrate=1_000_000,
            can_skew_s=0.1,
            leader_query_s=0.003,
            leader_gap_s=0.05,
            cycle_timeout_s=0.1,
            startup_feedback_s=0.1,
        ),
    )
    (root / "profile.json").write_text(json.dumps(profile))
    return Profile(root / "profile.json")


class FixtureCase(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.profile = configuration(self.root)

    def rewrite(self, **values):
        data = deepcopy(self.profile.data)
        data.update(values)
        self.profile.path.write_text(json.dumps(data))
        return Profile(self.profile.path)

    def samples(self, sequence, residual=None, velocity=None, q=None):
        q = [0] * 6 + [0.5] if q is None else q
        velocity = [0] * 7 if velocity is None else velocity
        residual = [0] * 7 if residual is None else residual
        theory = self.profile.model.evaluate(q[:6], velocity[:6], [0] * 6) + [0]
        return tuple(
            Sample(q[i], velocity[i], theory[i] + residual[i], 30, sequence * 0.02, sequence)
            for i in range(7)
        )


class BindingTests(FixtureCase):
    def test_reviewed_sdk_build_is_preferred_and_import_path_restored(self):
        sdk = self.root / "sdk"
        original = sys.path.copy()
        imported = []

        def load(name):
            self.assertEqual(sys.path[0], str(sdk / "interface_py"))
            imported.append(name)
            filename = "startouchclass.py" if name == "startouchclass" else "startouch.test.so"
            return SimpleNamespace(__file__=str(sdk / "interface_py" / filename))

        library = sdk / "src/libstartouch.so"
        with (
            patch("importlib.import_module", side_effect=load),
            patch.object(Path, "read_text", return_value="0000-1000 r-xp 0000 00:00 0 " + str(library)),
        ):
            verify_sdk_imports(self.profile)
        self.assertEqual(imported, ["startouchclass", "startouch"])
        self.assertEqual(sys.path, original)
        with patch("importlib.import_module", side_effect=ImportError("synthetic module load failure")):
            with self.assertRaises(ImportError):
                verify_sdk_imports(self.profile)
        self.assertEqual(sys.path, original)

    def test_modified_reviewed_build_is_rejected_before_import(self):
        (self.root / "sdk/interface_py/startouch.test.so").write_bytes(b"unreviewed rebuild")
        with patch("importlib.import_module") as load:
            with self.assertRaisesRegex(ValueError, "Reviewed SDK build changed"):
                verify_sdk_imports(self.profile)
        load.assert_not_called()

    def test_imported_sdk_copies_and_loaded_native_library_must_match_binding(self):
        sdk = self.root / "sdk"
        copies = self.root / "installed"
        copies.mkdir()
        modules = {}
        for name, relative in (
            ("startouchclass", "interface_py/startouchclass.py"),
            ("startouch", "interface_py/startouch.test.so"),
        ):
            target = copies / Path(relative).name
            target.write_bytes((sdk / relative).read_bytes())
            modules[name] = SimpleNamespace(__file__=str(target))
        library = sdk / "src/libstartouch.so"
        original_native = library.read_bytes()
        with (
            patch("importlib.import_module", side_effect=modules.__getitem__),
            patch.object(Path, "read_text", return_value="0000-1000 r-xp 0000 00:00 0 " + str(library)),
        ):
            verify_sdk_imports(self.profile)
            library.write_bytes(b"different installed SDK")
            with self.assertRaisesRegex(ValueError, "Loaded libstartouch"):
                verify_sdk_imports(self.profile)
            library.write_bytes(original_native)
            native = Path(modules["startouch"].__file__)
            native.write_bytes(b"CMake installed copy with different RUNPATH")
            with self.assertRaisesRegex(ValueError, "Imported startouch"):
                verify_sdk_imports(self.profile)
            native.write_bytes((sdk / "interface_py/startouch.test.so").read_bytes())
            Path(modules["startouchclass"].__file__).write_text("different wrapper")
            with self.assertRaisesRegex(ValueError, "Imported startouchclass"):
                verify_sdk_imports(self.profile)

    def test_same_bytes_at_another_native_path_mixed_missing_or_deleted_maps_are_rejected(self):
        sdk = self.root / "sdk"
        bound = sdk / "src/libstartouch.so"
        foreign = self.root / "another_sdk/src/libstartouch.so"
        foreign.parent.mkdir(parents=True)
        foreign.write_bytes(bound.read_bytes())
        modules = {
            name: SimpleNamespace(__file__=str(sdk / "interface_py" / filename))
            for name, filename in (
                ("startouchclass", "startouchclass.py"), ("startouch", "startouch.test.so"),
            )
        }

        def mapped(path):
            return f"0000-1000 r-xp 0000 00:00 0 {path}"

        for label, maps in (
            ("same bytes at a different path", mapped(foreign)),
            ("mixed SDK libraries", mapped(bound) + "\n" + mapped(foreign)),
            ("no native library", "0000-1000 rw-p 0000 00:00 0 [heap]"),
            ("deleted bound library", mapped(bound) + " (deleted)"),
            ("bound and deleted foreign libraries", mapped(bound) + "\n" + mapped(foreign) + " (deleted)"),
        ):
            with (
                self.subTest(label=label),
                patch("importlib.import_module", side_effect=modules.__getitem__),
                patch.object(Path, "read_text", return_value=maps),
                self.assertRaisesRegex(ValueError, "Loaded libstartouch.so"),
            ):
                verify_sdk_imports(self.profile)

    def test_native_paths_with_spaces_repeated_segments_and_canonical_alias_are_supported(self):
        root = self.root / "profile with spaces"
        root.mkdir()
        profile = configuration(root)
        sdk = root / "sdk"
        bound = sdk / "src/libstartouch.so"
        alias = root / "alias with spaces/libstartouch.so"
        alias.parent.mkdir()
        alias.symlink_to(bound)
        modules = {
            name: SimpleNamespace(__file__=str(sdk / "interface_py" / filename))
            for name, filename in (
                ("startouchclass", "startouchclass.py"), ("startouch", "startouch.test.so"),
            )
        }
        maps = "\n".join(
            f"{index * 4096:04x}-{(index + 1) * 4096:04x} r-xp 0000 00:00 0 {path}"
            for index, path in enumerate((bound, bound, alias))
        )
        original = sys.path.copy()
        with (
            patch("importlib.import_module", side_effect=modules.__getitem__),
            patch.object(Path, "read_text", return_value=maps),
        ):
            verify_sdk_imports(profile)
        self.assertEqual(sys.path, original)

    def test_mounting_gripper_and_loop_timing_rejected_offline(self):
        yaml_path = self.profile.paths["sdk_config"]
        for text in (
            "gripper_control: {type: TypeFZ}\n",
            "gripper_control: {type: TypeLJ}\nkinematics: {mounting: {mode: inverted}}\n",
        ):
            yaml_path.write_text(text)
            with self.assertRaises(ValueError):
                Profile(self.profile.path)
        yaml_path.write_text("gripper_control: {type: TypeLJ}\n")
        with self.assertRaisesRegex(ValueError, "upright"):
            self.rewrite(gravity=[0, 0, 9.80665])
        self.profile.path.write_text(json.dumps(self.profile.data))
        guards = dict(self.profile.guards, cycle_timeout_s=0.005)
        with self.assertRaisesRegex(ValueError, "loop period"):
            self.rewrite(guards=guards)

    def test_binding_detects_calibration_policy_sdk_and_model_edits(self):
        binding = self.root / "test.binding.json"
        binding.write_text(json.dumps(self.profile.binding()))
        self.profile.verify(binding)
        for path in (
            self.root / "calibration.json",
            self.root / "model.urdf",
            self.root / "sdk/src/libstartouch.so",
            self.root / "sdk/interface_py/startouch.test.so",
            self.root / "sdk/interface_py/startouchclass.py",
            self.profile.paths["sdk_config"],
        ):
            old = path.read_bytes()
            path.write_bytes(old + b" ")
            with self.subTest(path=path), self.assertRaisesRegex(ValueError, "Binding changed"):
                Profile(self.profile.path).verify(binding)
            path.write_bytes(old)
        data = deepcopy(self.profile.data)
        data["policy"][0]["deadzone_nm"] += 0.1
        with self.assertRaisesRegex(ValueError, "Binding changed"):
            self.rewrite(policy=data["policy"]).verify(binding)

    def test_scale_preserves_joint_power_and_gripper_is_separate(self):
        axes = deepcopy(self.profile.axes)
        axes[0].update(sdk_scale=-0.5, urdf_scale=-1, torque_scale=2)
        profile = self.rewrite(axes=axes)
        self.assertEqual(profile.to_model(0, profile.to_sdk(0, 2)), 1)
        axes[0]["torque_scale"] = -2
        with self.assertRaisesRegex(ValueError, "joint power"):
            self.rewrite(axes=axes)

    def test_missing_values_mass_confirmation_and_unknown_fields_rejected(self):
        cases = [dict(mass_coverage="UNCONFIRMED"), dict(evidence=""), dict(extra=1)]
        for change in cases:
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.rewrite(**change)
        axes = deepcopy(self.profile.axes)
        axes[0]["p_max"] = None
        with self.assertRaises(ValueError):
            self.rewrite(axes=axes)

    def test_duplicate_can_identity_and_nonphysical_sdk_limits_rejected(self):
        axes = deepcopy(self.profile.axes)
        axes[1] = axes[0].copy()
        with self.assertRaisesRegex(ValueError, "Duplicate"):
            self.rewrite(axes=axes)
        self.profile.path.write_text(json.dumps(self.profile.data))
        (self.root / "limits.json").write_text(json.dumps(dict(lower_rad=[-3] * 6, upper_rad=[3] * 6)))
        with self.assertRaisesRegex(ValueError, "URDF limits"):
            Profile(self.profile.path)

    def test_offline_check_and_bind_never_change_calibration(self):
        before = (self.root / "calibration.json").read_bytes()
        binding = self.root / "test.binding.json"
        with redirect_stdout(io.StringIO()):
            main(["check", "--profile", str(self.profile.path)])
            main(["bind", "--profile", str(self.profile.path), "--binding", str(binding)])
        self.profile.verify(binding)
        self.assertEqual(before, (self.root / "calibration.json").read_bytes())
        with self.assertRaises(SystemExit):
            main(["bind", "--profile", str(self.profile.path), "--binding", str(binding)])

    def test_template_is_deliberately_unconfigured_and_offline_imports_are_independent(self):
        self.assertEqual(template()["mass_coverage"], "UNCONFIRMED")
        code = "import sys; import stararm_lumos_feedback.cli; assert not {'lerobot','serial','can','startouchclass'} & sys.modules.keys()"
        subprocess.run([sys.executable, "-B", "-c", code], check=True, capture_output=True)


def dm_frame(axis, *, position=0, velocity=0, torque=0, temperature=30, status=1, timestamp=1000):
    def encode(value, bits, maximum):
        return round((value / maximum + 1) / 2 * ((1 << bits) - 1))

    p, v, t = (
        encode(position, 16, axis["p_max"]),
        encode(velocity, 12, axis["v_max"]),
        encode(torque, 12, axis["t_max"]),
    )
    return SimpleNamespace(
        arbitration_id=axis["feedback_id"],
        is_error_frame=False,
        is_remote_frame=False,
        is_extended_id=False,
        is_fd=False,
        is_rx=True,
        timestamp=timestamp,
        data=bytes(
            (
                (status << 4) | axis["motor_id"],
                p >> 8,
                p & 255,
                v >> 4,
                ((v & 15) << 4) | (t >> 8),
                t & 255,
                temperature,
                temperature,
            )
        ),
    )


class CanTests(FixtureCase):
    def test_measured_limit_margin_handles_quantization_without_relaxing_targets(self):
        self.profile.axes[0]["limit_margin"] = 0.01
        self.monitor.ingest(dm_frame(self.profile.axes[0], position=-1.905))
        self.assertLess(self.monitor.samples[0].q, self.profile.limits[0][0])
        with self.assertRaisesRegex(RuntimeError, "outside limits"):
            self.monitor.ingest(dm_frame(self.profile.axes[0], position=-1.92, timestamp=1000.01))

    def setUp(self):
        super().setUp()
        self.now, self.wall = 10.0, 1000.0
        self.bus = SimpleNamespace(shutdown=lambda: None)
        self.monitor = CanMonitor(self.profile, self.bus, clock=lambda: self.now, wall=lambda: self.wall)

    def test_decode_real_feedback_and_power_consistent_transform(self):
        for i, axis in enumerate(self.profile.axes):
            self.monitor.ingest(dm_frame(axis, position=-0.5 if i == 6 else 0.2, velocity=1, torque=2))
        samples = self.monitor.snapshot()
        self.assertAlmostEqual(samples[0].q, 0.2, places=3)
        self.assertAlmostEqual(samples[0].velocity, 1, places=2)
        self.assertAlmostEqual(samples[0].torque, 2, places=2)
        self.assertEqual(samples[0].stamp, 10)
        self.assertEqual(samples[0].temperature, 30)

    def test_cached_and_out_of_order_frames_do_not_refresh_age(self):
        frame = dm_frame(self.profile.axes[0])
        self.monitor.ingest(frame)
        old = self.monitor.samples[0]
        self.now += 0.02
        self.wall += 0.02
        self.monitor.ingest(frame)
        self.assertEqual(self.monitor.samples[0], old)
        self.now += 0.2
        self.wall += 0.2
        with self.assertRaisesRegex(RuntimeError, "stale"):
            self.monitor.ingest(frame)

    def test_disabled_initialization_allowed_but_faults_and_running_disable_rejected(self):
        self.monitor.ingest(dm_frame(self.profile.axes[0], status=0))
        self.assertIsNone(self.monitor.samples[0])
        self.monitor.armed = True
        for status in (0, 8, 9, 15):
            with self.subTest(status=status), self.assertRaisesRegex(RuntimeError, "motor fault"):
                self.monitor.ingest(dm_frame(self.profile.axes[0], status=status))

    def test_verified_range_parameter_replies_never_refresh_feedback(self):
        axis = self.profile.axes[0]
        self.monitor.ingest(dm_frame(axis))
        old = self.monitor.samples[0]
        self.monitor.armed = True
        self.now += 0.02
        self.wall += 0.02
        for register, value in ((21, 12.5), (22, 10), (23, 28)):
            reply = dm_frame(axis, timestamp=self.wall)
            reply.data = bytes((axis["motor_id"], 0, 0x33, register)) + struct.pack("<f", value)
            self.monitor.ingest(reply)
            self.assertEqual(self.monitor.samples[0], old)
            self.assertEqual(self.monitor.kernel_stamps[0], 1000)
        self.now += 0.2
        with self.assertRaisesRegex(RuntimeError, "missing/stale"):
            self.monitor.snapshot()

    def test_unknown_or_malformed_parameter_like_packets_do_not_hide_disable(self):
        axis = self.profile.axes[0]
        self.monitor.armed = True
        for register, value in ((20, 12.5), (21, math.nan), (22, math.inf), (23, -1), (23, 0)):
            reply = dm_frame(axis)
            reply.data = bytes((axis["motor_id"], 0, 0x33, register)) + struct.pack("<f", value)
            with self.subTest(register=register, value=value), self.assertRaisesRegex(
                RuntimeError, "motor fault 0"
            ):
                self.monitor.ingest(reply)
        for changed in ((axis["motor_id"], 1, 0x33, 21), (axis["motor_id"], 0, 0x32, 21)):
            reply = dm_frame(axis)
            reply.data = bytes(changed) + struct.pack("<f", 12.5)
            with self.assertRaisesRegex(RuntimeError, "motor fault 0"):
                self.monitor.ingest(reply)

    def test_wrong_ids_nonstate_frames_and_local_transmit_are_ignored(self):
        original = dm_frame(self.profile.axes[0])
        for changed in (
            dict(arbitration_id=0x700),
            dict(data=b"123"),
            dict(is_rx=False),
            dict(is_extended_id=True),
            dict(is_remote_frame=True),
            dict(is_fd=True),
        ):
            frame = SimpleNamespace(**(vars(original) | changed))
            self.monitor.ingest(frame)
            self.assertTrue(all(s is None for s in self.monitor.samples))

    def test_fault_temperature_limit_velocity_timestamp_and_jump(self):
        cases = [
            dict(temperature=80),
            dict(position=3),
            dict(velocity=20),
            dict(timestamp=998),
            dict(timestamp=1001),
        ]
        for changed in cases:
            with self.subTest(changed=changed), self.assertRaises(RuntimeError):
                self.monitor.ingest(dm_frame(self.profile.axes[0], **changed))
        self.monitor.ingest(dm_frame(self.profile.axes[0]))
        self.now += 0.01
        self.wall += 0.01
        with self.assertRaisesRegex(RuntimeError, "jump"):
            self.monitor.ingest(dm_frame(self.profile.axes[0], position=1, timestamp=self.wall))

    def test_missing_gripper_and_axis_skew_are_not_hidden_by_joint_reads(self):
        for axis in self.profile.axes[:6]:
            self.monitor.ingest(dm_frame(axis))
        with self.assertRaisesRegex(RuntimeError, "Axis 7"):
            self.monitor.snapshot()
        self.now += 0.12
        self.wall += 0.12
        self.monitor.ingest(dm_frame(self.profile.axes[6], position=-0.5, timestamp=self.wall))
        with self.assertRaisesRegex(RuntimeError, "apart"):
            self.monitor.snapshot()
        self.now += 0.1
        with self.assertRaisesRegex(RuntimeError, "stale"):
            self.monitor.snapshot()

    def test_can_error_frame_stops_monitor(self):
        frame = dm_frame(self.profile.axes[0])
        frame.is_error_frame = True
        with self.assertRaisesRegex(RuntimeError, "error frame"):
            self.monitor.ingest(frame)


def reply(sid, angle):
    data = struct.pack("<BHHHHBih", sid, 12000, 100, 1200, 1000, 0, round(angle * 10), 0)
    frame = b"\x05\x1c" + bytes((22, len(data))) + data
    return frame + bytes((sum(frame) & 255,))


class UART:
    def __init__(self, **kwargs):
        self.buffer = bytearray()
        self.writes = []
        self.angles = [0] * 6 + [50, 0]
        self.missing = set()
        self.partial = None
        self.is_open = True

    @property
    def in_waiting(self):
        return len(self.buffer)

    def reset_input_buffer(self):
        self.buffer.clear()

    def write(self, frame):
        self.writes.append(frame)
        if self.partial == len(self.writes):
            return 2
        if frame[2] == 25:
            for sid in frame[7:-1]:
                if sid not in self.missing:
                    self.buffer.extend(reply(sid, self.angles[sid]))
        return len(frame)

    def read(self, size):
        result = bytes(self.buffer[:size])
        del self.buffer[:size]
        return result

    def close(self):
        self.is_open = False


class SerialTests(unittest.TestCase):
    def test_cancel_between_axis_writes_does_not_block_cleanup_release(self):
        calls = []

        def guard():
            calls.append(None)
            if len(calls) == 2:
                raise SessionCancelled

        self.transport.before_control_write = guard
        with self.assertRaises(SessionCancelled):
            self.transport.output([1200] * 7, [False] * 7)
        self.assertEqual(len(self.uart.writes), 1)
        self.assertIsNone(self.transport.fault)
        self.assertEqual(self.transport.managed, {0})
        self.assertEqual(self.transport.release(), [])
        self.assertEqual(len(calls), 2)
        self.assertEqual(struct.unpack("<BBH", self.uart.writes[-1][4:-1]), (0, 0x10, 0))

    def test_fragmented_reply_resynchronizes_after_noise(self):
        uart = UART()
        original_write, original_read = uart.write, uart.read

        def fragmented_write(frame):
            result = original_write(frame)
            uart.buffer[:0] = b"noise\x05\x1c\x16\x00\x00"
            return result

        uart.write = fragmented_write
        uart.read = lambda size: original_read(min(3, size))
        transport = Fashionstar(uart, [0], enabled=False, timeout=0.05)
        self.assertEqual(transport.query([0])[0]["angle"], 0)

    def test_release_failure_is_retained_for_cleanup_retry(self):
        uart = UART()
        transport = Fashionstar(uart, [0], enabled=True, timeout=0.005)
        transport.output([1200], [False])
        uart.partial = 2
        self.assertTrue(transport.release())
        self.assertEqual(transport.managed, {0})
        self.assertEqual(transport.release(), [])
        self.assertEqual(transport.managed, set())

    def setUp(self):
        self.uart = UART()
        self.transport = Fashionstar(self.uart, range(7), enabled=True, timeout=0.005)

    def test_true_zero_angle_is_valid_and_query_uses_new_replies(self):
        samples = self.transport.query(list(range(8)))
        self.assertEqual(samples[0]["angle"], 0)
        self.assertEqual(samples[6]["current_ma"], 100)
        self.uart.buffer.extend(reply(0, 90))
        self.uart.missing.add(0)
        with self.assertRaises(LeaderGap):
            self.transport.query(list(range(8)))

    def test_invalid_sentinel_and_checksum_never_become_actions(self):
        self.uart.angles[0] = -23592959.9
        with self.assertRaises(LeaderGap):
            self.transport.query([0])
        self.uart.angles[0] = 0
        original = self.uart.write

        def corrupt(data):
            count = original(data)
            self.uart.buffer[-1] ^= 255
            return count

        self.uart.write = corrupt
        with self.assertRaises(LeaderGap):
            self.transport.query([0])

    def test_zero_uses_release_and_hold_is_1500(self):
        self.transport.output([0] * 7, [True] + [False] * 6)
        commands = [struct.unpack("<BBH", frame[4:-1]) for frame in self.uart.writes]
        self.assertEqual(commands[0], (0, 0x11, 1500))
        self.assertEqual(commands[1], (1, 0x10, 0))
        self.assertNotIn(7, [c[0] for c in commands])

    def test_partial_write_releases_all_attempted_outputs_and_latches_failure(self):
        self.uart.partial = 2
        with self.assertRaisesRegex(RuntimeError, "Partial"):
            self.transport.output([1200] * 7, [False] * 7)
        self.assertEqual(
            [struct.unpack("<BBH", f[4:-1]) for f in self.uart.writes[2:]], [(0, 0x10, 0), (1, 0x10, 0)]
        )
        with self.assertRaises(RuntimeError):
            self.transport.output([1200] * 7, [False] * 7)

    def test_monitor_never_sends_force_commands_including_cleanup(self):
        self.transport.enabled = False
        self.transport.output([1500] * 7, [True] * 7)
        self.transport.release()
        self.assertEqual(self.uart.writes, [])

    def test_protocol_checksum_and_wire_layout(self):
        frame = packet(24, struct.pack("<BBH", 6, 0x12, 1200))
        self.assertEqual(frame[:4], b"\x12\x4c\x18\x04")
        self.assertEqual(frame[-1], sum(frame[:-1]) & 255)


class ControllerTests(FixtureCase):
    def test_nonfinite_model_output_cannot_request_damping(self):
        self.step(1)
        samples = self.samples(2)
        with patch.object(self.profile.model, "evaluate", return_value=[math.inf] * 6):
            with self.assertRaisesRegex(RuntimeError, "Non-finite"):
                self.control.update(samples, self.q, self.q)
        self.assertEqual(self.control.power, [0] * 7)

    def setUp(self):
        super().setUp()
        self.caps = Caps()
        self.control = Controller(self.profile, self.caps)
        self.q = [0] * 6 + [0.5]

    def step(self, sequence, residual=None, velocity=None, desired=None, target=None):
        return self.control.update(
            self.samples(sequence, residual, velocity),
            self.q if target is None else target,
            self.q if desired is None else desired,
        )

    def test_unloaded_whole_arm_model_cancels_each_axis(self):
        for seq in range(1, 20):
            self.step(seq)
        self.assertEqual(self.control.power, [0] * 7)
        self.assertTrue(any(abs(x) > 0.1 for x in self.control.telemetry["model_nm"][:6]))
        self.assertEqual(self.control.telemetry["model_nm"][6], 0)

    def test_j1_j6_and_gripper_have_independent_load_and_caps(self):
        for seq in range(1, 20):
            self.step(seq, [2, 0, 0, 0, 0, -2, 2])
        for i in (0, 5, 6):
            self.assertGreater(self.control.power[i], 900)
        self.assertEqual(self.control.power[1:5], [0] * 4)
        self.caps.set(dict(common_mw=400, axis_mw=[0, 1200, 1200, 1200, 1200, 800, 1200]))
        self.step(20, [2, 0, 0, 0, 0, -2, 2])
        self.assertEqual(self.control.power[0], 0)
        self.assertEqual(self.control.power[5:], [400, 400])

    def test_load_confirmation_rate_limits_and_cached_samples(self):
        self.step(1)
        self.step(2, [4] * 7)
        self.assertEqual(self.control.power, [0] * 7)
        self.step(3, [4] * 7)
        self.assertLessEqual(max(self.control.power), 200.0001)
        saved = self.control.power.copy()
        for _ in range(100):
            self.step(3, [4] * 7)
        self.assertEqual(self.control.power, saved)

    def test_stall_hold_independent_of_zero_caps_and_releases_with_confirmation(self):
        self.caps.set(dict(common_mw=400, axis_mw=[0] * 7))
        target = [1] + self.q[1:]
        for seq in range(1, 15):
            self.step(seq, [3] + [0] * 6, target=target, desired=target)
        self.assertTrue(self.control.hold)
        powers, holds = self.control.outputs(False)
        self.assertEqual(powers, [0] * 7)
        self.assertEqual(holds, [True] * 7)
        for seq in range(15, 40):
            self.step(seq, target=target)
        self.assertFalse(self.control.hold)

    def test_manual_hold_only_arm_and_gap_cannot_release_automatic_hold(self):
        self.assertEqual(self.control.outputs(True)[1], [True] * 6 + [False])
        self.control.hold = True
        self.control.hold_release = 0.05
        self.control.gap()
        self.assertTrue(self.control.hold)
        self.assertEqual(self.control.hold_release, 0)

    def test_acceleration_out_of_range_stops_instead_of_zeroing_model_state(self):
        self.step(1)
        with self.assertRaisesRegex(RuntimeError, "acceleration"):
            self.step(2, velocity=[10] * 7)

    def test_caps_reject_nan_policy_fields_and_bad_ranges(self):
        for changed in (
            dict(common_mw=200, axis_mw=[0] * 7),
            dict(common_mw=1200, axis_mw=[math.nan] * 7),
            dict(common_mw=1200, axis_mw=[1501] * 7),
            dict(common_mw=1200, axis_mw=[0] * 7, deadzone_nm=0),
        ):
            with self.subTest(changed=changed), self.assertRaises(ValueError):
                self.caps.set(changed)


class FakeLeader:
    def __init__(self, uart):
        self.uart = uart
        self.config = SimpleNamespace(joint_map=list(range(6)))
        self.leader = SimpleNamespace(
            send_sync_servo_monitor=lambda *args: None,
            stop_on_control_mode=lambda *args: None,
            servos={i: SimpleNamespace(angle_monitor=0) for i in range(8)},
        )

    def connect(self):
        pass

    def disconnect(self):
        self.uart.close()

    def get_action(self):
        self.leader.send_sync_servo_monitor(list(range(8)))
        hold = self.leader.servos[7].angle_monitor == 180
        self.leader.stop_on_control_mode(0, 0x11 if hold else 0x10, 0)
        values = [math.radians(self.leader.servos[i].angle_monitor) for i in range(6)] + [
            self.leader.servos[6].angle_monitor / 100
        ]
        return dict(zip(ACTION_KEYS, values))


class FakeRobot:
    def __init__(self):
        self.positions = [0] * 6
        self.gripper = 0.5
        self.sent, self.cleaned = [], False
        self.arm = SimpleNamespace(
            get_joint_positions=lambda: self.positions.copy(),
            get_gripper_position=lambda: self.gripper,
            get_gripper_state=lambda: SimpleNamespace(
                position=0.557976,  # SDK legacy motor fraction differs from the public opening.
                distance_m=self.gripper * 0.08,
                motor_position_rad=-0.7014613048553252,
                motor_velocity_rad_s=0,
                feedback_age_ms=0,
                feedback_valid=True,
                enabled=True,
                error_code=1,
            ),
        )

    def connect(self):
        pass

    def send_action(self, action):
        # Simulate the adapter's bounded target; HOLD must use this return value.
        sent = {k: min(0.01, v) if i < 6 else v for i, (k, v) in enumerate(action.items())}
        self.sent.append(sent)
        return sent

    def disconnect(self):
        self.cleaned = True


class LiveMonitor:
    def __init__(self, profile, residual=None):
        self.profile, self.residual = profile, residual or [0] * 7
        self.sequence = 0
        self.error, self.closed = None, False

    def start(self):
        pass

    def close(self):
        self.closed = True

    def snapshot(self):
        if self.error:
            raise self.error
        self.sequence += 1
        q = [0] * 6 + [0.5]
        torque = self.profile.model.evaluate(q[:6], [0] * 6, [0] * 6) + [0]
        return tuple(
            Sample(
                q[i],
                0,
                torque[i] + self.residual[i],
                30,
                time.monotonic(),
                self.sequence,
                -0.7014613048553252 if i == 6 else None,
                0 if i == 6 else None,
            )
            for i in range(7)
        )


class SessionTests(FixtureCase):
    def test_runtime_created_can_monitor_receives_actual_session_mode(self):
        for mode in ("follow", "monitor", "run"):
            with self.subTest(mode=mode):
                session = self.session(mode)
                session.monitor = None
                with (
                    patch("stararm_lumos_feedback.runtime.validate_socketcan"),
                    patch("stararm_lumos_feedback.runtime.CanMonitor", return_value=self.monitor) as create,
                ):
                    session.run(0.02)
                create.assert_called_once_with(self.profile, mode=mode)
                self.assertTrue(self.robot.cleaned and self.monitor.closed)

    def test_before_enable_hook_uses_open_transport_after_monitor_start(self):
        session = self.session("follow")
        order = []

        def preflight(transport):
            order.append("preflight")
            self.assertIs(transport, session.transport)
            self.assertIsNotNone(session.reader)
            self.assertEqual(transport.ids, tuple(range(7)))
            self.assertFalse(transport.enabled)

        session.before_enable = preflight
        with (
            patch.object(self.leader, "connect", side_effect=lambda: order.append("leader")),
            patch.object(self.monitor, "start", side_effect=lambda: order.append("monitor")),
            patch.object(self.robot, "connect", side_effect=lambda: order.append("robot")),
        ):
            session.run(0.02)
        self.assertEqual(order, ["leader", "monitor", "preflight", "robot"])
        self.assertTrue(self.robot.cleaned and self.monitor.closed)

    def test_before_enable_failure_or_cancel_never_constructs_sdk(self):
        for outcome in ("failure", "cancel"):
            with self.subTest(outcome=outcome):
                session = self.session("follow")

                def preflight(transport):
                    self.assertIs(transport, session.transport)
                    if outcome == "failure":
                        raise RuntimeError("synthetic pose mismatch")
                    session.stop.set()

                session.before_enable = preflight
                with patch.object(self.robot, "connect") as connect:
                    if outcome == "failure":
                        with self.assertRaisesRegex(RuntimeError, "synthetic pose mismatch"):
                            session.run(0.2)
                    else:
                        session.run(0.2)
                connect.assert_not_called()
                self.assertEqual(self.robot.sent, [])
                self.assertTrue(self.robot.cleaned and self.monitor.closed)
                self.assertFalse(self.uart.is_open)
                self.assertEqual(session.transport.managed, set())
                self.assertEqual(session.error is None, outcome == "cancel")

    def test_preflight_session_cancellation_never_publishes_error_or_enables(self):
        from stararm_lumos_feedback.motion import ConnectionCancelled, preflight_session_pose

        session = self.session("follow")
        states = []
        session.dashboard = SimpleNamespace(publish=lambda **state: states.append(state))
        session.before_enable = lambda transport: preflight_session_pose(
            self.profile, session.stop, transport
        )

        def cancel_preflight(profile, cancel, *, leader_transport):
            self.assertIs(profile, self.profile)
            self.assertIs(cancel, session.stop)
            self.assertIs(leader_transport, session.transport)
            cancel.set()
            raise ConnectionCancelled("Explicit stop during pose preflight")

        with (
            patch("stararm_lumos_feedback.motion.preflight_pose", side_effect=cancel_preflight) as probe,
            patch.object(self.robot, "connect") as connect,
        ):
            session.run(0.2)
        probe.assert_called_once()
        connect.assert_not_called()
        self.assertEqual(self.robot.sent, [])
        self.assertEqual(self.uart.writes, [])
        self.assertIsNone(session.error)
        self.assertTrue(states)
        self.assertTrue(all(state["status"] == "stopped" and state["error"] is None for state in states))
        self.assertTrue(self.robot.cleaned and self.monitor.closed)
        self.assertFalse(self.uart.is_open)
        self.assertEqual(session.transport.managed, set())
        self.assertIsNone(getattr(self.robot, "before_control_write", None))

    def test_cancel_during_startup_prevents_further_device_calls(self):
        for stage in ("before", "leader", "sdk"):
            with self.subTest(stage=stage):
                session = self.session("follow")
                if stage == "before":
                    session.stop.set()
                with (
                    patch.object(
                        self.leader, "connect", side_effect=session.stop.set if stage == "leader" else None
                    ) as leader_connect,
                    patch.object(
                        self.robot, "connect", side_effect=session.stop.set if stage == "sdk" else None
                    ) as robot_connect,
                    patch.object(self.robot.arm, "get_joint_positions") as getter,
                ):
                    session.run(0.2)
                self.assertEqual(leader_connect.call_count, int(stage != "before"))
                self.assertEqual(robot_connect.call_count, int(stage == "sdk"))
                getter.assert_not_called()
                self.assertEqual(self.robot.sent, [])
                self.assertTrue(self.robot.cleaned and self.monitor.closed)
                self.assertIsNone(session.error)
                self.assertIsNone(getattr(self.robot, "before_control_write", None))

    def test_cancel_while_leader_call_is_blocked_prevents_target_after_return(self):
        session = self.session("run")
        entered, resume = threading.Event(), threading.Event()
        original = self.leader.get_action

        def action():
            entered.set()
            self.assertTrue(resume.wait(1))
            return original()

        def cancel():
            if entered.wait(1):
                session.stop.set()
                resume.set()

        worker = threading.Thread(target=cancel)
        self.leader.get_action = action
        worker.start()
        try:
            session.run(0.3)
        finally:
            resume.set()
            worker.join(1)
        self.assertTrue(entered.is_set())
        self.assertFalse(worker.is_alive())
        self.assertEqual(self.robot.sent, [])
        self.assertTrue(all(frame[2] == 25 for frame in self.uart.writes))
        self.assertTrue(self.robot.cleaned and self.monitor.closed)
        self.assertIsNone(session.error)

    def test_explicit_stop_ends_normal_loop_and_releases_managed_outputs(self):
        session = self.session("run")
        states = []

        def publish(**state):
            states.append(state)
            if state["status"] == "active":
                session.stop.set()

        session.dashboard = SimpleNamespace(publish=publish)
        session.run(0.3)
        self.assertEqual(len(self.robot.sent), 1)
        self.assertEqual(states[-1]["status"], "stopped")
        self.assertIsNone(states[-1]["error"])
        commands = [struct.unpack("<BBH", f[4:-1]) for f in self.uart.writes if f[2] == 24]
        self.assertEqual(commands[-7:], [(i, 0x10, 0) for i in range(7)])
        self.assertTrue(self.robot.cleaned and self.monitor.closed)

    def test_fault_is_not_swallowed_by_concurrent_explicit_stop(self):
        session = self.session("follow")
        session.error = RuntimeError("synthetic watchdog fault")
        session.stop.set()
        with self.assertRaisesRegex(RuntimeError, "synthetic watchdog fault"):
            session.run(0.2)
        self.assertTrue(self.robot.cleaned and self.monitor.closed)

    def test_cancel_after_sdk_joint_read_prevents_gripper_read(self):
        session = self.session("follow")

        def positions():
            session.stop.set()
            return self.robot.positions.copy()

        self.robot.arm.get_joint_positions = positions
        with patch.object(self.robot.arm, "get_gripper_state") as getter:
            session.run(0.2)
        getter.assert_not_called()
        self.assertEqual(self.robot.sent, [])
        self.assertTrue(self.robot.cleaned and self.monitor.closed)
        self.assertIsNone(session.error)

    def test_runtime_error_is_visible_before_sdk_disconnect(self):
        session = self.session("follow")
        states = []
        session.dashboard = SimpleNamespace(publish=lambda **state: states.append(state))
        original = self.leader.get_action

        def action():
            if states:
                raise RuntimeError("synthetic device failure")
            return original()

        def disconnect():
            self.assertTrue(any(state["status"] == "active" for state in states))
            self.assertEqual(states[-1]["status"], "stopped")
            self.assertEqual(states[-1]["error"], "synthetic device failure")
            self.assertEqual(states[-1]["requested_mw"], [0] * 7)
            self.robot.cleaned = True

        self.leader.get_action = action
        self.robot.disconnect = disconnect
        with self.assertRaisesRegex(RuntimeError, "synthetic device failure"):
            session.run(0.2)
        self.assertTrue(self.robot.cleaned and self.monitor.closed)

    def test_watchdog_publishes_fault_while_device_call_is_blocked(self):
        session = self.session("follow")
        states, published = [], threading.Event()

        def publish(**state):
            states.append(state)
            if state["status"] == "stopped" and state["error"]:
                published.set()

        def blocked(action):
            self.assertTrue(published.wait(1), "Watchdog did not publish while device call was blocked")
            self.assertIn("watchdog", states[-1]["error"])
            return action

        session.dashboard = SimpleNamespace(publish=publish)
        self.robot.send_action = blocked
        with self.assertRaisesRegex(RuntimeError, "watchdog"):
            session.run(0.3)
        self.assertTrue(self.robot.cleaned and self.monitor.closed)
        session.publish("active")  # A late update cannot hide the fault.
        self.assertEqual(states[-1]["status"], "stopped")
        self.assertIn("watchdog", states[-1]["error"])

    def test_socketcan_preflight_is_read_only_and_rejects_wrong_bitrate(self):
        link = dict(
            flags=["UP"],
            linkinfo=dict(
                info_kind="can", info_data=dict(state="ERROR-ACTIVE", bittiming=dict(bitrate=1000000))
            ),
        )
        with patch("subprocess.run", return_value=SimpleNamespace(stdout=json.dumps([link]))) as run:
            validate_socketcan("unit-test-can", 1000000)
            self.assertEqual(
                run.call_args.args[0], ["ip", "-details", "-json", "link", "show", "dev", "unit-test-can"]
            )
            with self.assertRaisesRegex(SocketCANError, "bitrate"):
                validate_socketcan("unit-test-can", 500000)

    def test_socketcan_command_failures_preserve_stage_and_stderr(self):
        interface = "unit-test-can"
        failures = (
            (FileNotFoundError("ip missing"), "can_ip_unavailable", None),
            (subprocess.TimeoutExpired(["ip"], 2), "can_query_timeout", None),
            (
                subprocess.CalledProcessError(1, ["ip"], stderr='Device "unit-test-can" does not exist.\n'),
                "can_interface_missing",
                False,
            ),
            (
                subprocess.CalledProcessError(1, ["ip"], stderr="RTNETLINK answers: Operation not permitted\n"),
                "can_query_failed",
                True,
            ),
        )
        for failure, code, exists in failures:
            with (
                self.subTest(code=code),
                patch("subprocess.run", side_effect=failure) as run,
                patch("pathlib.Path.is_dir", return_value=True),
                patch("pathlib.Path.exists", return_value=exists),
                self.assertRaises(SocketCANError) as raised,
            ):
                validate_socketcan(interface, 1_000_000)
            self.assertEqual(raised.exception.code, code)
            self.assertEqual(raised.exception.interface, interface)
            if isinstance(failure, subprocess.CalledProcessError):
                self.assertEqual(raised.exception.detail, failure.stderr.strip())
            self.assertEqual(run.call_count, 1)
            self.assertTrue(run.call_args.kwargs["check"])

    def test_socketcan_reports_down_bitrate_and_bus_fault_without_mutation(self):
        link = dict(
            flags=["UP"],
            linkinfo=dict(
                info_kind="can", info_data=dict(state="ERROR-ACTIVE", bittiming=dict(bitrate=1_000_000))
            ),
        )
        cases = (
            ("down", "can_interface_down"),
            ("bitrate", "can_bitrate_mismatch"),
            ("fault", "can_bus_fault"),
        )
        for change, code in cases:
            changed = deepcopy(link)
            if change == "down":
                changed["flags"] = []
            elif change == "bitrate":
                changed["linkinfo"]["info_data"]["bittiming"]["bitrate"] = 500000
            else:
                changed["linkinfo"]["info_data"]["state"] = "BUS-OFF"
            with (
                self.subTest(code=code),
                patch("subprocess.run", return_value=SimpleNamespace(stdout=json.dumps([changed]))) as run,
                self.assertRaises(SocketCANError) as raised,
            ):
                validate_socketcan("unit-test-can", 1_000_000)
            self.assertEqual(raised.exception.code, code)
            self.assertEqual(
                run.call_args.args[0], ["ip", "-details", "-json", "link", "show", "dev", "unit-test-can"]
            )
            self.assertEqual(run.call_count, 1)

    def test_socketcan_invalid_nested_fields_are_query_errors(self):
        link = dict(
            flags=["UP"],
            linkinfo=dict(
                info_kind="can", info_data=dict(state="ERROR-ACTIVE", bittiming=dict(bitrate=1_000_000))
            ),
        )
        paths = (("linkinfo",), ("flags",), ("linkinfo", "info_data"), ("linkinfo", "info_data", "bittiming"))
        for path in paths:
            for invalid in (None, 1, "invalid"):
                changed = deepcopy(link)
                field = changed
                for key in path[:-1]:
                    field = field[key]
                field[path[-1]] = invalid
                with (
                    self.subTest(path=path, invalid=invalid),
                    patch("subprocess.run", return_value=SimpleNamespace(stdout=json.dumps([changed]))) as run,
                    self.assertRaises(SocketCANError) as raised,
                ):
                    validate_socketcan("unit-test-can", 1_000_000)
                self.assertEqual(raised.exception.code, "can_query_failed")
                self.assertEqual(run.call_count, 1)

    def test_socketcan_sysfs_permission_error_does_not_imply_missing_interface(self):
        failure = subprocess.CalledProcessError(1, ["ip"], stderr="RTNETLINK answers: Operation not permitted")
        for field in ("is_dir", "exists"):
            with (
                self.subTest(field=field),
                patch("subprocess.run", side_effect=failure),
                patch("pathlib.Path.is_dir", return_value=True),
                patch("pathlib.Path." + field, side_effect=PermissionError("sysfs access denied")),
                self.assertRaises(SocketCANError) as raised,
            ):
                validate_socketcan("unit-test-can", 1_000_000)
            self.assertEqual(raised.exception.code, "can_query_failed")
            self.assertEqual(raised.exception.detail, failure.stderr)

    def test_transient_leader_gap_recovers_without_sending_cached_action(self):
        session = self.session("monitor")
        original = self.leader.get_action
        outcomes = [True, False, False, True, False]

        def action():
            if outcomes:
                if outcomes.pop(0):
                    raise LeaderGap("synthetic transient gap")
            return original()

        self.leader.get_action = action
        session.run(0.07)
        self.assertGreater(len(self.robot.sent), 0)
        self.assertLess(len(self.robot.sent), 7)
        self.assertTrue(all(f[2] == 25 for f in self.uart.writes))

    def session(self, mode="run", residual=None):
        self.uart = UART()
        self.robot, self.leader = FakeRobot(), FakeLeader(self.uart)
        self.monitor = LiveMonitor(self.profile, residual)
        return Session(self.profile, self.robot, self.leader, Caps(), mode=mode, monitor=self.monitor)

    def test_real_serial_and_controller_loop_monitor_has_no_force_writes(self):
        session = self.session("monitor", [4] * 7)
        self.uart.angles[7] = 180
        session.run(0.06)
        self.assertEqual(self.robot.sent, [])  # HD freeze still works without a force command
        self.assertTrue(all(frame[2] == 25 for frame in self.uart.writes))
        self.assertTrue(self.robot.cleaned and self.monitor.closed)

    def test_actual_sent_target_and_dynamic_output_cleanup(self):
        session = self.session(residual=[3, 0, 0, 0, 0, 0, 2])
        self.uart.angles[0] = 20
        session.run(0.12)
        self.assertEqual(session.target[0], 0.01)
        commands = [struct.unpack("<BBH", f[4:-1]) for f in self.uart.writes if f[2] == 24]
        self.assertTrue(any(mode == 0x12 and power > 0 for _, mode, power in commands))
        self.assertEqual(commands[-7:], [(i, 0x10, 0) for i in range(7)])
        self.assertTrue(self.robot.cleaned and not self.uart.is_open)

    def test_leader_gap_freezes_follower_and_continuous_gap_exits(self):
        session = self.session()
        self.uart.missing.add(6)
        with self.assertRaisesRegex(RuntimeError, "Continuous Leader"):
            session.run(0.2)
        self.assertEqual(self.robot.sent, [])
        self.assertTrue(self.robot.cleaned and self.monitor.closed)

    def test_sdk_coordinate_mismatch_stops_before_targets(self):
        session = self.session()
        self.robot.positions[0] = 1
        with self.assertRaisesRegex(RuntimeError, "coordinates disagree"):
            session.run(0.03)
        self.assertEqual(self.robot.sent, [])
        self.assertTrue(self.robot.cleaned)

    def test_fault_and_blocked_sdk_watchdog_release_before_cleanup(self):
        session = self.session()

        def blocked(action):
            time.sleep(0.16)
            return action

        self.robot.send_action = blocked
        with self.assertRaisesRegex(RuntimeError, "watchdog"):
            session.run(0.3)
        self.assertIsNotNone(session.transport.fault)
        self.assertTrue(self.robot.cleaned)

    def test_two_sessions_cannot_own_same_resource(self):
        with session_lock("unit-test-can", "unit-test-uart"):
            with self.assertRaisesRegex(RuntimeError, "already in use"):
                with session_lock("unit-test-can", "different-uart"):
                    pass


class SdkHistoryTests(FixtureCase):
    def setUp(self):
        super().setUp()
        self.profile.engineering = True
        self.now, self.wall = 10.0, 1000.0
        self.monitor = CanMonitor(
            self.profile, SimpleNamespace(shutdown=lambda: None),
            clock=lambda: self.now, wall=lambda: self.wall, mode="follow",
        )
        self.monitor.armed = True
        self.robot = FakeRobot()
        self.session = Session(self.profile, self.robot, object(), Caps(), mode="follow", monitor=self.monitor)
        self.session._sdk_history_ready = True  # Simulate completed strict startup admission.
        self.receive([0] * 6)
        self.robot.positions = [sample.q for sample in self.monitor.samples[:6]]

    def receive(self, positions, *, dt=0):
        self.now += dt
        self.wall += dt
        for i, axis in enumerate(self.profile.axes):
            self.monitor.ingest(dm_frame(
                axis, position=positions[i] if i < 6 else -0.7014613048553252,
                timestamp=self.wall,
            ))

    def check_sdk(self, samples=None):
        with patch("stararm_lumos_feedback.runtime.time.monotonic", side_effect=lambda: self.now):
            self.session.check_sdk(self.monitor.snapshot() if samples is None else samples)

    def test_active_engineering_follow_matches_real_older_arm_samples_with_original_tolerance(self):
        self.receive([0.08] * 6, dt=0.01)
        latest = self.monitor.snapshot()
        self.check_sdk()
        self.assertEqual(self.monitor.snapshot(), latest)
        self.assertTrue(all(sample.q > 0.07 for sample in latest[:6]))
        self.assertTrue(all(abs(q) < 0.001 for q in self.robot.positions))
        self.assertEqual(self.robot.sent, [])

    def test_recent_samples_are_not_interpolated_or_given_extra_timing_tolerance(self):
        self.receive([0.08] * 6, dt=0.01)
        self.robot.positions = [0.04] * 6
        with self.assertRaisesRegex(RuntimeError, "coordinates disagree") as raised:
            self.check_sdk()
        for field in (
            "SDK=+0.04", "CAN=", "delta(SDK-CAN)=", "sdk_tolerance=0.02",
            "timing_allowance=", "velocity=", "frame_age=", "history_window=0.15",
            "history_count=2", "closest_history=(q=", "delta=", "age=", "sequence=",
        ):
            self.assertIn(field, str(raised.exception))

    def test_cache_frozen_while_real_arm_moves_stops_when_matching_history_expires(self):
        for step in range(1, 5):
            self.receive([0.05 * step] * 6, dt=0.03)
            self.check_sdk()
        self.receive([0.25] * 6, dt=0.04)
        self.monitor.snapshot()  # Current CAN remains complete and fresh.
        with self.assertRaisesRegex(RuntimeError, "coordinates disagree"):
            self.check_sdk()

    def test_stale_current_can_cannot_be_replaced_with_a_matching_history_value(self):
        self.receive([0.08] * 6, dt=0.01)
        latest = self.monitor.snapshot()
        self.now += 0.2
        self.wall += 0.2
        with self.assertRaisesRegex(RuntimeError, "missing/stale CAN feedback"):
            self.check_sdk(latest)

    def test_history_requires_exact_engineering_follow_and_completed_strict_startup(self):
        self.receive([0.08] * 6, dt=0.01)
        for engineering, mode, ready in (
            (False, "follow", True), (1, "follow", True),
            (True, "monitor", True), (True, "run", True), (True, "follow", False),
        ):
            with self.subTest(engineering=engineering, mode=mode, ready=ready):
                self.profile.engineering, self.session.mode = engineering, mode
                self.session._sdk_history_ready = ready
                with patch.object(self.monitor, "recent_arm_history") as history:
                    with self.assertRaisesRegex(RuntimeError, "coordinates disagree"):
                        self.check_sdk()
                history.assert_not_called()

    def test_history_unavailable_or_monitor_fault_stops_instead_of_accepting_sdk_cache(self):
        self.receive([0.08] * 6, dt=0.01)
        with patch.object(self.monitor, "recent_arm_history", return_value=((),) * 6):
            with self.assertRaisesRegex(RuntimeError, "closest_history=\\(none\\)"):
                self.check_sdk()
        self.monitor.error = RuntimeError("synthetic CAN receive fault")
        with self.assertRaisesRegex(RuntimeError, "synthetic CAN receive fault"):
            self.check_sdk(tuple(self.monitor.samples))

    def test_arm_history_acceptance_does_not_skip_gripper_coordinate_checks(self):
        self.receive([0.08] * 6, dt=0.01)
        self.robot.arm.get_gripper_state = lambda: SimpleNamespace(
            distance_m=0.04, motor_position_rad=-0.2, motor_velocity_rad_s=0,
            feedback_age_ms=0, feedback_valid=True, enabled=True, error_code=1,
        )
        with self.assertRaisesRegex(RuntimeError, "Gripper passive CAN and SDK raw motor coordinates disagree"):
            self.check_sdk()

    def test_no_follower_target_is_sent_when_recent_real_history_does_not_match(self):
        self.receive([0.08] * 6, dt=0.01)
        self.robot.positions = [0.04] * 6
        self.session.leader = SimpleNamespace(get_action=lambda: dict(zip(ACTION_KEYS, [0] * 6 + [0.5])))
        self.session.reader = SimpleNamespace(manual_hold=False)
        self.session.transport = SimpleNamespace(output=lambda power, holds: None)
        with patch("stararm_lumos_feedback.runtime.time.monotonic", side_effect=lambda: self.now):
            with self.assertRaisesRegex(RuntimeError, "coordinates disagree"):
                self.session.step()
        self.assertEqual(self.robot.sent, [])


class DashboardTests(FixtureCase):
    def setUp(self):
        super().setUp()
        self.caps = Caps()
        self.settings = self.root / "current.caps.json"
        self.web = Dashboard(self.caps, self.profile, port=0, settings=self.settings)
        self.web.start()
        self.addCleanup(self.web.close)

    def request(self, method, path, data=None, token=True, origin=None):
        connection = http.client.HTTPConnection("127.0.0.1", self.web.server.server_port, timeout=2)
        headers = {"Content-Type": "application/json"}
        if token:
            headers["X-Feedback-Token"] = self.web.token
        if origin:
            headers["Origin"] = origin
        connection.request(method, path, body=None if data is None else json.dumps(data), headers=headers)
        response = connection.getresponse()
        result = response.status, response.read(), dict(response.getheaders())
        connection.close()
        return result

    def test_caps_only_mutation_and_persistence_without_telemetry(self):
        self.web.publish(status="active", mode="run", q_sdk=[0.0] * 7, age_ms=[0.0] * 7, residual_nm=[10] * 7)
        status, _, _ = self.request("POST", "/api/caps", dict(common_mw=800, axis_mw=[0] * 7))
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(self.settings.read_text()), dict(common_mw=800, axis_mw=[0] * 7))
        status, _, _ = self.request("POST", "/api/caps", dict(common_mw=800, axis_mw=[0] * 7, deadzone_nm=0))
        self.assertEqual(status, 400)
        self.assertEqual(self.request("POST", "/api/policy", {})[0], 404)
        self.assertEqual([p.name for p in self.root.glob("*.caps.json")], ["current.caps.json"])

    def test_cross_origin_and_missing_token_rejected(self):
        value = dict(common_mw=1200, axis_mw=[1200] * 7)
        self.assertEqual(self.request("POST", "/api/caps", value, token=False)[0], 403)
        self.assertEqual(self.request("POST", "/api/caps", value, origin="https://example.com")[0], 403)
        self.assertEqual(self.request("GET", "/api/state", origin="https://example.com")[0], 403)

    def test_bilingual_static_assets_and_in_memory_state(self):
        self.web.publish(status="active", mode="monitor", measured_nm=[1] * 7)
        code, data, headers = self.request("GET", "/api/state")
        self.assertEqual(code, 200)
        self.assertEqual(json.loads(data)["measured_nm"], [1] * 7)
        self.assertEqual(headers["Cache-Control"], "no-store")
        for path in ("/", "/app.js", "/style.css"):
            self.assertEqual(self.request("GET", path)[0], 200)
        self.assertIn("整臂", self.request("GET", "/app.js")[1].decode())
        self.assertIn("Whole-arm", self.request("GET", "/app.js")[1].decode())
        self.assertFalse(self.settings.exists())


if __name__ == "__main__":
    unittest.main()
