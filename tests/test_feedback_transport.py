"""Offline regressions for DM register replies sharing mechanical feedback IDs."""

import math
import struct
from types import SimpleNamespace

from test_feedback_runtime import FixtureCase, dm_frame

from stararm_lumos_feedback.transport import ARM_HISTORY_MAX_SAMPLES, CanMonitor, advance_velocity_interval


class RegisterReplyTests(FixtureCase):
    def setUp(self):
        super().setUp()
        self.now, self.wall = 10.0, 1000.0
        self.monitor = CanMonitor(
            self.profile,
            SimpleNamespace(shutdown=lambda: None),
            clock=lambda: self.now,
            wall=lambda: self.wall,
        )
        self.monitor.armed = True

    def reply(self, axis, register, value=1.0, *, header=None):
        frame = dm_frame(axis, timestamp=self.wall)
        frame.data = bytes(header or (axis["motor_id"], 0, 0x33, register)) + struct.pack("<f", value)
        return frame

    def test_delayed_startup_and_range_reads_never_refresh_any_motor(self):
        # Synthetic KT, PMAX, VMAX, TMAX, Imax values; no hardware is queried.
        registers = ((1, 0.1), (21, 12.5), (22, 10.0), (23, 28.0), (59, 20.0))
        for i, axis in enumerate(self.profile.axes):
            self.monitor.ingest(dm_frame(axis, position=-0.7 if i == 6 else 0))
        before = self.monitor.snapshot()
        stamps = tuple(self.monitor.kernel_stamps)
        self.now += 0.02
        self.wall += 0.02
        for axis in self.profile.axes:
            for register, value in registers:
                with self.subTest(motor=axis["motor_id"], register=register):
                    self.monitor.ingest(self.reply(axis, register, value))
                    self.assertEqual(tuple(self.monitor.samples), before)
                    self.assertEqual(tuple(self.monitor.kernel_stamps), stamps)
        self.now += 0.2
        with self.assertRaisesRegex(RuntimeError, "missing/stale CAN"):
            self.monitor.snapshot()

    def test_parameter_replies_cannot_supply_the_first_mechanical_sample(self):
        for axis in self.profile.axes:
            for register in (1, 21, 22, 23, 59):
                self.monitor.ingest(self.reply(axis, register))
        self.assertEqual(self.monitor.samples, [None] * 7)
        self.assertEqual(self.monitor.kernel_stamps, [None] * 7)
        with self.assertRaisesRegex(RuntimeError, "missing/stale CAN"):
            self.monitor.snapshot()

    def test_malformed_or_unverified_reads_still_fail_when_armed(self):
        axis = self.profile.axes[0]
        for register in (1, 21, 22, 23, 59):
            for value in (0, -1, math.nan, math.inf):
                with self.subTest(register=register, value=value), self.assertRaisesRegex(
                    RuntimeError, "motor fault 0"
                ):
                    self.monitor.ingest(self.reply(axis, register, value))
        for header in (
            (1, 1, 0x33, 1),  # wrong high byte of the complete little-endian motor ID
            (1, 0, 0x55, 1),  # write acknowledgement is not an authorized read reply
            (1, 0, 0x32, 1),
            (1, 0, 0x33, 20),  # a register that this integration does not query
        ):
            with self.subTest(header=header), self.assertRaisesRegex(RuntimeError, "motor fault 0"):
                self.monitor.ingest(self.reply(axis, 1, header=header))

    def test_matching_low_id_nibble_cannot_hide_enabled_or_fault_state(self):
        axis = self.profile.axes[0]
        # The remainder deliberately resembles a read response. Only the exact
        # complete ID is a parameter header, never an enabled/fault status byte.
        for status in (1, 8, 9, 15):
            header = ((status << 4) | axis["motor_id"], 0, 0x33, 1)
            message = "overtemperature" if status == 1 else f"motor fault {status}"
            with self.subTest(status=status), self.assertRaisesRegex(RuntimeError, message):
                self.monitor.ingest(self.reply(axis, 1, header=header))

    def test_arbitration_and_embedded_id_must_both_match_before_filtering(self):
        first, second = self.profile.axes[:2]
        frame = self.reply(first, 1)
        frame.arbitration_id = second["feedback_id"]
        self.monitor.ingest(frame)
        self.assertEqual(self.monitor.samples, [None] * 7)
        self.assertEqual(self.monitor.kernel_stamps, [None] * 7)
        # A real fresh mechanical frame must still create the initial sample.
        self.monitor.ingest(dm_frame(first))
        self.assertEqual(self.monitor.samples[0].sequence, 1)
        self.assertEqual(self.monitor.kernel_stamps[0], 1000)


class AccelerationGuardTests(FixtureCase):
    def setUp(self):
        super().setUp()
        self.now, self.wall = 10.0, 1000.0
        self.monitor = CanMonitor(
            self.profile,
            SimpleNamespace(shutdown=lambda: None),
            clock=lambda: self.now,
            wall=lambda: self.wall,
        )

    def code_frame(self, i, code, *, position=None, timestamp=None):
        frame = dm_frame(
            self.profile.axes[i],
            position=(-0.7 if i == 6 else 0) if position is None else position,
            timestamp=self.wall if timestamp is None else timestamp,
        )
        data = bytearray(frame.data)
        data[3], data[4] = code >> 4, ((code & 15) << 4) | (data[4] & 15)
        frame.data = bytes(data)
        return frame

    def advance(self, dt):
        self.now += dt
        self.wall += dt

    def accepted_state(self, i):
        return (
            self.monitor.samples[i], self.monitor.kernel_stamps[i],
            self.monitor.velocity_bounds[i] if i < 6 else None,
        )

    def assert_state_unchanged(self, i, state):
        self.assertIs(self.monitor.samples[i], state[0])
        self.assertEqual(self.monitor.kernel_stamps[i], state[1])
        if i < 6:
            self.assertIs(self.monitor.velocity_bounds[i], state[2])

    def test_each_axis_rejects_acceleration_between_individual_frames(self):
        for i, axis in enumerate(self.profile.axes):
            with self.subTest(axis=i + 1):
                axis["max_acceleration"] = 0.1
                position = -0.7 if i == 6 else 0
                self.monitor.ingest(dm_frame(axis, position=position, timestamp=self.wall))
                previous = self.monitor.samples[i]
                self.now += 0.01
                self.wall += 0.01
                with self.assertRaisesRegex(RuntimeError, f"Axis {i + 1}: acceleration"):
                    self.monitor.ingest(
                        dm_frame(axis, position=position, velocity=1, timestamp=self.wall)
                    )
                self.assertIs(self.monitor.samples[i], previous)

    def test_same_or_older_kernel_frame_does_not_create_acceleration(self):
        axis = self.profile.axes[0]
        axis["max_acceleration"] = 0.1
        self.monitor.ingest(dm_frame(axis, timestamp=self.wall))
        previous = self.monitor.samples[0]
        for timestamp in (self.wall, self.wall - 0.005):
            self.monitor.ingest(dm_frame(axis, velocity=1, timestamp=timestamp))
            self.assertIs(self.monitor.samples[0], previous)
        self.assertEqual(self.monitor.samples[0].sequence, 1)

    def test_receive_interval_not_poll_jitter_controls_acceleration(self):
        axis = self.profile.axes[0]
        self.monitor.ingest(dm_frame(axis, timestamp=self.wall))
        self.wall += 0.02
        self.now += 0.0001  # Synthetic wall-to-monotonic conversion jitter.
        self.monitor.ingest(dm_frame(axis, velocity=1, timestamp=self.wall))
        self.assertEqual(self.monitor.samples[0].sequence, 2)
        self.assertLess(
            self.monitor.samples[0].velocity / 0.02, axis["max_acceleration"]
        )
        self.assertEqual(self.monitor.samples[0].kernel_timestamp, self.wall)

    def test_one_code_short_interval_jitter_preserves_feasible_arm_sequence(self):
        for i, axis in enumerate(self.profile.axes[:6]):
            axis["v_max"] = 10 if i < 3 else 30
            axis["max_acceleration"] = 44
            self.monitor.ingest(self.code_frame(i, 2047))
            for step in range(40):
                self.advance(0.0001)
                self.monitor.ingest(self.code_frame(i, 2048 if step % 2 == 0 else 2047))
            self.assertEqual(self.monitor.samples[i].sequence, 41)
            lower, upper = self.monitor.velocity_bounds[i]
            self.assertLessEqual(lower, upper)
            self.assertEqual(self.monitor.samples[i].kernel_timestamp, self.wall)

    def test_adjacent_code_staircase_rejects_cumulative_acceleration_in_both_directions_and_scales(self):
        for i, (sdk_scale, urdf_scale) in enumerate(((1, 1), (-0.5, 3), (0.5, -3), (-0.5, -3))):
            axis = self.profile.axes[i]
            axis.update(sdk_scale=sdk_scale, urdf_scale=urdf_scale)
            lsb = 2 * axis["v_max"] / 4095 * abs(sdk_scale * urdf_scale)
            axis["max_acceleration"] = 0.3 * lsb / 0.0001
            direction = -1 if i % 2 else 1
            first = 2048 if direction < 0 else 2047
            self.monitor.ingest(self.code_frame(i, first))
            self.advance(0.0001)
            self.monitor.ingest(self.code_frame(i, first + direction))
            state = self.accepted_state(i)
            self.advance(0.0001)
            with self.subTest(sdk_scale=sdk_scale, urdf_scale=urdf_scale), self.assertRaisesRegex(
                RuntimeError, f"Axis {i + 1}: acceleration outside configured range"
            ):
                self.monitor.ingest(self.code_frame(i, first + 2 * direction))
            self.assert_state_unchanged(i, state)

    def test_large_code_step_rejects_with_numeric_diagnostics_without_commit(self):
        axis = self.profile.axes[3]
        axis["max_acceleration"] = 44
        self.monitor.armed = True
        self.monitor.ingest(self.code_frame(3, 2047))
        state = self.accepted_state(3)
        self.advance(0.0001)
        with self.assertRaisesRegex(RuntimeError, "Axis 4: acceleration outside configured range") as raised:
            self.monitor.ingest(self.code_frame(3, 2050))
        for field in (
            "raw_a=", "limit=44", "dv=", "dt=", "previous_v=", "current_v=", "code=2050",
            "LSB=", "sequence=1->2", "armed=True", "feasible_before=", "feasible_bounds=",
        ):
            self.assertIn(field, str(raised.exception))
        self.assert_state_unchanged(3, state)

    def test_sustained_acceleration_cannot_hide_as_individually_compatible_code_steps(self):
        axis = self.profile.axes[0]
        axis.update(v_max=4095, max_velocity=5000, max_acceleration=6)
        self.monitor.ingest(self.code_frame(0, 0))
        for code in range(1, 5):
            self.advance(0.25)
            self.monitor.ingest(self.code_frame(0, code))
        state = self.accepted_state(0)
        self.advance(0.25)
        with self.assertRaisesRegex(RuntimeError, "acceleration outside configured range"):
            self.monitor.ingest(self.code_frame(0, 5))
        self.assert_state_unchanged(0, state)

    def test_slow_linear_velocity_sequence_is_feasible(self):
        axis = self.profile.axes[0]
        axis["max_acceleration"] = 44
        self.monitor.ingest(self.code_frame(0, 2047))
        for code in range(2048, 2100):
            self.advance(0.001)
            self.monitor.ingest(self.code_frame(0, code))
        self.assertEqual(self.monitor.samples[0].sequence, 53)

    def test_duplicate_or_older_frame_cannot_reset_the_feasible_interval(self):
        axis = self.profile.axes[0]
        axis["max_acceleration"] = 44
        self.monitor.ingest(self.code_frame(0, 2047))
        self.advance(0.0001)
        self.monitor.ingest(self.code_frame(0, 2048))
        state = self.accepted_state(0)
        for timestamp in (self.wall, self.wall - 0.00005):
            self.monitor.ingest(self.code_frame(0, 2049, timestamp=timestamp))
            self.assert_state_unchanged(0, state)
        self.advance(0.0001)
        with self.assertRaisesRegex(RuntimeError, "acceleration outside configured range"):
            self.monitor.ingest(self.code_frame(0, 2049))
        self.assert_state_unchanged(0, state)

    def test_position_jump_does_not_commit_candidate_interval_and_recovery_uses_last_accepted_frame(self):
        axis = self.profile.axes[0]
        axis["max_acceleration"] = 44
        self.monitor.ingest(self.code_frame(0, 2047))
        state = self.accepted_state(0)
        self.advance(0.0001)
        with self.assertRaisesRegex(RuntimeError, "position jump"):
            self.monitor.ingest(self.code_frame(0, 2048, position=0.75))
        self.assert_state_unchanged(0, state)
        self.advance(0.0001)
        self.monitor.ingest(self.code_frame(0, 2048))
        self.assertEqual(self.monitor.samples[0].sequence, 2)
        expected = advance_velocity_interval(
            axis, self.monitor.samples[0].velocity, state[2], self.wall - state[1],
        )
        self.assertEqual(self.monitor.velocity_bounds[0], expected)

    def test_other_guard_failures_leave_interval_sample_and_timestamp_unchanged(self):
        self.monitor.ingest(self.code_frame(0, 2047))
        state = self.accepted_state(0)
        self.advance(0.001)
        for kind in ("fault", "temperature", "velocity", "limits", "stale"):
            frame = self.code_frame(0, 2047)
            data = bytearray(frame.data)
            if kind == "fault":
                data[0] = 0x80 | self.profile.axes[0]["motor_id"]
            elif kind == "temperature":
                data[7] = 100
            elif kind == "velocity":
                frame = self.code_frame(0, 4095)
                data = bytearray(frame.data)
            elif kind == "limits":
                frame = self.code_frame(0, 2047, position=2.5)
                data = bytearray(frame.data)
            elif kind == "stale":
                frame.timestamp = self.wall - 0.2
            frame.data = bytes(data)
            with self.subTest(kind=kind), self.assertRaises(RuntimeError):
                self.monitor.ingest(frame)
            self.assert_state_unchanged(0, state)

    def test_touching_feasible_boundary_accepts_but_clear_excess_rejects(self):
        for sdk_scale, urdf_scale, acceleration in ((1, 1, 4), (-0.5, 3, 6)):
            axis = self.profile.axes[0]
            axis.update(v_max=4095, sdk_scale=sdk_scale, urdf_scale=urdf_scale, max_acceleration=acceleration)
            first_velocity = -4095 * sdk_scale * urdf_scale
            touching_velocity = -4091 * sdk_scale * urdf_scale
            outside_velocity = -4089 * sdk_scale * urdf_scale
            initial = advance_velocity_interval(axis, first_velocity)
            touching = advance_velocity_interval(axis, touching_velocity, initial, 0.5)
            outside = advance_velocity_interval(axis, outside_velocity, initial, 0.5)
            self.assertLessEqual(touching[0], touching[1])
            self.assertGreater(outside[0], outside[1])

    def test_interval_advance_requires_positive_finite_time_but_seed_needs_no_interval(self):
        axis = self.profile.axes[0]
        bounds = advance_velocity_interval(axis, 0)
        self.assertLessEqual(bounds[0], bounds[1])
        for dt in (None, 0, -0.01, math.nan, math.inf):
            with self.subTest(dt=dt), self.assertRaises(ValueError):
                advance_velocity_interval(axis, 0, bounds, dt)

    def test_gripper_one_code_acceleration_guard_remains_unmodified(self):
        axis = self.profile.axes[6]
        axis["max_acceleration"] = 0.1
        self.monitor.ingest(self.code_frame(6, 2047))
        state = self.accepted_state(6)
        self.advance(0.0001)
        with self.assertRaisesRegex(RuntimeError, "Axis 7: acceleration outside configured range") as raised:
            self.monitor.ingest(self.code_frame(6, 2048))
        self.assertIn("feasible_before=None", str(raised.exception))
        self.assertIn("feasible_bounds=None", str(raised.exception))
        self.assertIn("raw_a=", str(raised.exception))
        self.assert_state_unchanged(6, state)


class EngineeringFollowGuardTests(FixtureCase):
    def setUp(self):
        super().setUp()
        self.now, self.wall = 10.0, 1000.0

    def monitor(self, *, engineering=True, **kwargs):
        self.profile.engineering = engineering
        return CanMonitor(
            self.profile,
            SimpleNamespace(shutdown=lambda: None),
            clock=lambda: self.now,
            wall=lambda: self.wall,
            **kwargs,
        )

    def code_frame(self, i, code, *, position=None):
        frame = dm_frame(
            self.profile.axes[i],
            position=(-0.7 if i == 6 else -0.5781) if position is None else position,
            timestamp=self.wall,
        )
        data = bytearray(frame.data)
        data[3], data[4] = code >> 4, ((code & 15) << 4) | (data[4] & 15)
        frame.data = bytes(data)
        return frame

    def advance(self, dt):
        self.now += dt
        self.wall += dt

    def test_observed_axis4_low_speed_sequence_accepts_in_engineering_follow(self):
        axis = self.profile.axes[3]
        axis.update(sdk_scale=-1, urdf_scale=1, v_max=30, max_acceleration=44)
        monitor = self.monitor(mode="follow")
        monitor.armed = True
        monitor.ingest(self.code_frame(3, 2048))
        previous = monitor.samples[3]
        self.advance(0.00189065933228)
        monitor.ingest(self.code_frame(3, 2055))
        current = monitor.samples[3]
        self.assertEqual(current.sequence, 2)
        self.assertEqual(current.q, previous.q)
        self.assertLess(abs(current.velocity), 0.2)
        acceleration = abs(current.velocity - previous.velocity) / (
            current.kernel_timestamp - previous.kernel_timestamp
        )
        self.assertGreater(acceleration, axis["max_acceleration"])
        self.assertEqual(monitor.mode, "follow")

    def test_observed_sequence_remains_strict_in_every_other_mode_or_profile(self):
        axis = self.profile.axes[3]
        axis.update(sdk_scale=-1, urdf_scale=1, v_max=30, max_acceleration=44)
        for mode, engineering in (
            (None, True), ("monitor", True), ("run", True),
            ("follow", False), ("follow", 1), ("follow", "true"), ("follow", None),
        ):
            with self.subTest(mode=mode, engineering=engineering):
                monitor = self.monitor(engineering=engineering, **({"mode": mode} if mode else {}))
                monitor.ingest(self.code_frame(3, 2048))
                previous = monitor.samples[3]
                self.advance(0.00189065933228)
                with self.assertRaisesRegex(RuntimeError, "Axis 4: acceleration outside configured range"):
                    monitor.ingest(self.code_frame(3, 2055))
                self.assertIs(monitor.samples[3], previous)

    def test_invalid_mode_rejects_before_creating_a_can_bus(self):
        for mode in (None, "inspect", "", True):
            with self.subTest(mode=mode), self.assertRaisesRegex(ValueError, "mode must be"):
                CanMonitor(self.profile, mode=mode)

    def test_engineering_follow_skips_derived_acceleration_on_all_six_arm_axes(self):
        monitor = self.monitor(mode="follow")
        for i, axis in enumerate(self.profile.axes[:6]):
            with self.subTest(axis=i + 1):
                axis["max_acceleration"] = 0.1
                monitor.ingest(self.code_frame(i, 2048))
                self.advance(0.001)
                monitor.ingest(self.code_frame(i, 2055))
                self.assertEqual(monitor.samples[i].sequence, 2)

    def test_engineering_follow_retains_arm_fault_freshness_temperature_and_motion_guards(self):
        axis = self.profile.axes[3]
        axis.update(sdk_scale=-1, urdf_scale=1, v_max=30, max_acceleration=44)
        for kind, message in (
            ("fault", "motor fault"), ("disabled", "motor fault"),
            ("temperature", "overtemperature"), ("velocity", "velocity outside"),
            ("limits", "measured position outside"), ("jump", "position jump"),
            ("stale", "queued/stale"),
        ):
            with self.subTest(kind=kind):
                monitor = self.monitor(mode="follow")
                monitor.armed = True
                monitor.ingest(self.code_frame(3, 2048))
                previous = monitor.samples[3]
                previous_stamp = monitor.kernel_stamps[3]
                self.advance(0.00189065933228)
                frame = self.code_frame(3, 4095 if kind == "velocity" else 2055)
                data = bytearray(frame.data)
                if kind in ("fault", "disabled"):
                    data[0] = ((8 if kind == "fault" else 0) << 4) | axis["motor_id"]
                elif kind == "temperature":
                    data[7] = 100
                elif kind in ("limits", "jump"):
                    frame = self.code_frame(3, 2055, position=2.5 if kind == "limits" else 0.2)
                    data = bytearray(frame.data)
                elif kind == "stale":
                    frame.timestamp -= 0.2
                frame.data = bytes(data)
                with self.assertRaisesRegex(RuntimeError, message):
                    monitor.ingest(frame)
                self.assertIs(monitor.samples[3], previous)
                self.assertEqual(monitor.kernel_stamps[3], previous_stamp)
        monitor = self.monitor(mode="follow")
        for i in range(7):
            monitor.ingest(self.code_frame(i, 2048))
        monitor.snapshot()
        self.advance(0.2)
        with self.assertRaisesRegex(RuntimeError, "missing/stale CAN feedback"):
            monitor.snapshot()

    def test_engineering_follow_retains_gripper_acceleration(self):
        axis = self.profile.axes[6]
        axis["max_acceleration"] = 0.1
        monitor = self.monitor(mode="follow")
        monitor.ingest(self.code_frame(6, 2047))
        previous = monitor.samples[6]
        self.advance(0.0001)
        with self.assertRaisesRegex(RuntimeError, "Axis 7: acceleration outside configured range"):
            monitor.ingest(self.code_frame(6, 2048))
        self.assertIs(monitor.samples[6], previous)

    def test_engineering_follow_retains_gripper_raw_and_normalized_guards(self):
        axis = self.profile.axes[6]
        for kind, message in (
            ("motor_range", "raw motor position outside"),
            ("motor_velocity", "raw motor velocity outside"),
            ("velocity", "velocity outside configured"),
            ("jump", "position jump"), ("motor_jump", "raw motor position jump"),
        ):
            with self.subTest(kind=kind):
                axis.update(max_acceleration=200, max_velocity=10, max_jump=0.5)
                self.profile.gripper_transform["max_motor_jump_rad"] = 0.5
                monitor = self.monitor(mode="follow")
                monitor.ingest(self.code_frame(6, 2048))
                previous = monitor.samples[6]
                self.advance(0.02)
                if kind == "velocity":
                    axis["max_velocity"] = 0.01
                elif kind == "jump":
                    axis["max_jump"] = 0.01
                elif kind == "motor_jump":
                    self.profile.gripper_transform["max_motor_jump_rad"] = 0.05
                position = 0.02 if kind == "motor_range" else -0.6
                code = 4095 if kind == "motor_velocity" else 2055 if kind == "velocity" else 2048
                with self.assertRaisesRegex(RuntimeError, message):
                    monitor.ingest(self.code_frame(6, code, position=position))
                self.assertIs(monitor.samples[6], previous)


class ArmHistoryTests(FixtureCase):
    def setUp(self):
        super().setUp()
        self.profile.engineering = True
        self.now, self.wall = 10.0, 1000.0
        self.monitor = CanMonitor(
            self.profile, SimpleNamespace(shutdown=lambda: None),
            clock=lambda: self.now, wall=lambda: self.wall, mode="follow",
        )
        self.monitor.armed = True

    def advance(self, dt):
        self.now += dt
        self.wall += dt

    def frame(self, i=0, **kwargs):
        return dm_frame(self.profile.axes[i], timestamp=self.wall, **kwargs)

    def test_history_contains_only_six_immutable_accepted_axis_sample_sequences(self):
        for i in range(7):
            self.monitor.ingest(self.frame(i, position=-0.7 if i == 6 else 0))
        history = self.monitor.recent_arm_history(ended_at=self.now)
        self.assertIsInstance(history, tuple)
        self.assertEqual(len(history), 6)
        for i, samples in enumerate(history):
            self.assertIsInstance(samples, tuple)
            self.assertEqual(samples, (self.monitor.samples[i],))
        with self.assertRaises(AttributeError):
            history[0][0].q = 1
        self.advance(0.01)
        self.monitor.ingest(self.frame())
        self.assertEqual(len(history[0]), 1)
        self.assertEqual(len(self.monitor.recent_arm_history(ended_at=self.now)[0]), 2)

    def test_count_bound_evicts_oldest_samples_without_expanding_the_time_window(self):
        for _ in range(ARM_HISTORY_MAX_SAMPLES + 30):
            self.monitor.ingest(self.frame())
            self.advance(0.0001)
        history = self.monitor.recent_arm_history(ended_at=self.now)[0]
        self.assertEqual(len(history), ARM_HISTORY_MAX_SAMPLES)
        self.assertEqual(history[0].sequence, 31)
        self.assertEqual(history[-1].sequence, ARM_HISTORY_MAX_SAMPLES + 30)
        self.assertLess(self.now - history[0].stamp, self.profile.axes[0]["max_age_s"])

    def test_original_axis_age_prunes_history_without_refreshing_live_feedback(self):
        self.profile.axes[0]["max_age_s"] = 0.01
        for i in range(7):
            self.monitor.ingest(self.frame(i, position=-0.7 if i == 6 else 0))
        stamp = self.monitor.samples[0].stamp
        self.advance(0.02)
        history = self.monitor.recent_arm_history(ended_at=self.now)
        self.assertEqual(history[0], ())
        self.assertEqual(len(history[1]), 1)
        self.assertEqual(self.monitor.samples[0].stamp, stamp)
        with self.assertRaisesRegex(RuntimeError, "Axis 1: missing/stale"):
            self.monitor.snapshot()

    def test_getter_finish_excludes_later_and_future_frames(self):
        self.monitor.ingest(self.frame())
        first = self.monitor.samples[0]
        getter_finished = self.now
        self.advance(0.01)
        self.monitor.ingest(self.frame(position=0.08))
        self.assertEqual(self.monitor.recent_arm_history(ended_at=getter_finished)[0], (first,))
        self.assertEqual(self.monitor.recent_arm_history(ended_at=getter_finished - 0.001)[0], ())
        # A future caller cutoff does not bypass the query's current-time window.
        self.now = getter_finished
        self.assertEqual(self.monitor.recent_arm_history(ended_at=getter_finished + 0.02)[0], (first,))

    def test_rejected_ignored_and_duplicate_frames_cannot_enter_or_refresh_history(self):
        self.monitor.ingest(self.frame())
        initial = self.monitor.samples[0]
        self.advance(0.01)
        frames = [self.frame(status=8), self.frame(temperature=100), self.frame(velocity=30),
                  self.frame(position=2.5), self.frame(position=0.75)]
        stale = self.frame()
        stale.timestamp -= 0.2
        frames.append(stale)
        for frame in frames:
            with self.assertRaises(RuntimeError):
                self.monitor.ingest(frame)
            self.assertEqual(self.monitor.recent_arm_history(ended_at=self.now)[0], (initial,))
        repeated = self.frame(position=0.1)
        repeated.timestamp = initial.kernel_timestamp
        self.monitor.ingest(repeated)
        register = self.frame()
        register.data = bytes((self.profile.axes[0]["motor_id"], 0, 0x33, 1)) + struct.pack("<f", 0.1)
        self.monitor.ingest(register)
        self.assertEqual(self.monitor.recent_arm_history(ended_at=self.now)[0], (initial,))
        self.advance(0.2)
        self.assertEqual(self.monitor.recent_arm_history(ended_at=self.now)[0], ())

    def test_history_query_rejects_invalid_cutoffs_and_monitor_faults(self):
        for ended_at in (None, True, math.nan, math.inf, "10"):
            with self.subTest(ended_at=ended_at), self.assertRaises(ValueError):
                self.monitor.recent_arm_history(ended_at=ended_at)
        self.monitor.error = RuntimeError("synthetic receive fault")
        with self.assertRaisesRegex(RuntimeError, "CAN monitor stopped: synthetic receive fault"):
            self.monitor.recent_arm_history(ended_at=self.now)
