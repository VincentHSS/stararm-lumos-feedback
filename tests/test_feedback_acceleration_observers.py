"""Sequence-quantization validation remains consistent at observer boundaries."""

from dataclasses import replace
from unittest.mock import patch

from test_feedback_inspection import Clock, frame, profile
from test_feedback_runtime import FixtureCase

from stararm_lumos_feedback.control import Caps, Controller
from stararm_lumos_feedback.inspection import Inspection


class ObserverAccelerationTests(FixtureCase):
    def test_controller_short_filter_rejects_out_of_range_model_estimate_without_committing_state(self):
        self.profile.axes[3].update(v_max=30, max_acceleration=44)
        self.profile.policy[3]["acceleration_filter_s"] = 0.000001
        control = Controller(self.profile, Caps())

        def samples(sequence, upper_bin):
            values = []
            for i, sample in enumerate(self.samples(sequence)):
                axis = self.profile.axes[i]
                lsb = 2 * axis["v_max"] / 4095 * abs(axis["sdk_scale"] * axis["urdf_scale"])
                # Real adjacent velocity-code centres straddle zero. Axis 1
                # also changes, so a later Axis 4 rejection must not commit
                # an earlier axis's candidate filter state.
                velocity = (0.5 if upper_bin else -0.5) * lsb if i in (0, 3) else 0
                values.append(replace(
                    sample, stamp=10 + sequence * 0.0001,
                    kernel_timestamp=1000 + sequence * 0.0001, velocity=velocity,
                ))
            return tuple(values)

        initial = samples(1, False)
        control.update(initial, [s.q for s in initial], [s.q for s in initial])
        previous = control.previous
        acceleration, bounds = control.acceleration.copy(), control.velocity_bounds.copy()
        filtered, powers = control.filtered.copy(), control.power.copy()
        later = samples(2, True)
        # The two velocity bins admit acceleration <=44, but their centre
        # difference with this valid short filter estimates about146.52.
        with patch.object(self.profile.model, "evaluate") as model:
            with self.assertRaisesRegex(RuntimeError, "model acceleration estimate outside configured range"):
                control.update(later, [s.q for s in later], [s.q for s in later])
            model.assert_not_called()
        self.assertIs(control.previous, previous)
        self.assertEqual(control.acceleration, acceleration)
        self.assertEqual(control.velocity_bounds, bounds)
        self.assertEqual(control.filtered, filtered)
        self.assertEqual(control.power, powers)

    def test_inspection_short_filter_rejects_out_of_range_model_estimate_without_committing_state(self):
        configuration, clock = profile(), Clock()
        configuration.axes[3]["max_acceleration"] = 44
        configuration.policy[3]["acceleration_filter_s"] = 0.000001
        observer = Inspection(configuration, clock=clock.monotonic, wall=clock.wall)

        def ingest(code):
            incoming = frame(configuration, clock, i=3)
            data = bytearray(incoming.data)
            data[3] = code >> 4
            data[4] = (code & 15) << 4 | (data[4] & 15)
            incoming.data = data
            observer.ingest(incoming)

        ingest(2047)
        previous = observer.samples[3]
        acceleration, bounds = observer.acceleration.copy(), observer.velocity_bounds.copy()
        counts = observer.counts.copy()
        clock.now += 0.0001
        with self.assertRaisesRegex(RuntimeError, "model acceleration estimate outside configured range"):
            ingest(2048)
        self.assertIs(observer.samples[3], previous)
        self.assertEqual(observer.acceleration, acceleration)
        self.assertEqual(observer.velocity_bounds, bounds)
        self.assertEqual(observer.counts, counts)

    def test_controller_accepts_boundary_toggle_but_rejects_accumulated_staircase(self):
        axis = self.profile.axes[3]
        axis["max_acceleration"] = 44
        lsb = 2 * axis["v_max"] / 4095 * abs(axis["sdk_scale"] * axis["urdf_scale"])
        control = Controller(self.profile, Caps())

        def update(sequence, code):
            samples = tuple(
                replace(s, stamp=sequence * 0.0001, velocity=code * lsb if i == 3 else 0)
                for i, s in enumerate(self.samples(sequence))
            )
            return control.update(samples, [s.q for s in samples], [s.q for s in samples])

        update(1, 0)
        self.assertTrue(update(2, 1))
        saved = control.velocity_bounds.copy()
        with self.assertRaisesRegex(RuntimeError, "Axis 4: acceleration.*dv=.*dt="):
            update(3, 2)
        self.assertEqual(control.velocity_bounds, saved)
        self.assertEqual(control.previous[3].sequence, 2)
        control.gap()
        self.assertEqual(control.velocity_bounds, [None] * 6)

    def test_controller_uses_kernel_receive_interval_despite_monotonic_jitter(self):
        control = Controller(self.profile, Caps())
        initial = tuple(replace(s, kernel_timestamp=1000) for s in self.samples(1))
        control.update(initial, [s.q for s in initial], [s.q for s in initial])
        later = tuple(
            replace(s, stamp=old.stamp + 0.000001, kernel_timestamp=1000.02, velocity=1 if i == 0 else 0)
            for i, (s, old) in enumerate(zip(self.samples(2), initial))
        )
        self.assertTrue(control.update(later, [s.q for s in later], [s.q for s in later]))

    def test_read_only_inspection_keeps_sequence_constraint_and_rejected_state(self):
        configuration, clock = profile(), Clock()
        configuration.axes[3]["max_acceleration"] = 44
        observer = Inspection(configuration, clock=clock.monotonic, wall=clock.wall)

        def ingest(code):
            incoming = frame(configuration, clock, i=3)
            data = bytearray(incoming.data)
            data[3] = code >> 4
            data[4] = (code & 15) << 4 | (data[4] & 15)
            incoming.data = data
            observer.ingest(incoming)

        ingest(2047)
        clock.now += 0.0001
        ingest(2048)
        saved, bounds = observer.samples[3].copy(), observer.velocity_bounds[3]
        clock.now += 0.0001
        with self.assertRaisesRegex(RuntimeError, "Axis 4: inspection acceleration.*dt="):
            ingest(2049)
        self.assertEqual(observer.samples[3], saved)
        self.assertEqual(observer.velocity_bounds[3], bounds)
