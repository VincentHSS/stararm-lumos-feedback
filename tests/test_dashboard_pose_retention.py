"""Retained display poses never become new hardware telemetry."""

import math
from copy import deepcopy
from types import SimpleNamespace

from test_feedback_runtime import FixtureCase

from stararm_lumos_feedback.control import Caps
from stararm_lumos_feedback.dashboard import Dashboard


class DashboardPoseRetentionTests(FixtureCase):
    def setUp(self):
        super().setUp()
        self.now = 10.0
        self.dashboard = Dashboard(Caps(), self.profile, port=0, clock=lambda: self.now)
        # Snapshot and publication checks need no HTTP worker or device modules.
        self.addCleanup(self.dashboard.server.server_close)
        self.q_sdk = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.25]
        self.q_urdf = [self.profile.to_model(i, value) for i, value in enumerate(self.q_sdk[:6])]

    def remember(self, *, captured=None):
        self.assertIs(
            self.dashboard.remember_pose(
                self.q_sdk, self.q_urdf, self.now if captured is None else captured,
            ),
            True,
        )
        return self.retained_pose()

    def retained_pose(self, snapshot=None):
        pose = deepcopy((self.dashboard.snapshot() if snapshot is None else snapshot)["last_pose"])
        if pose is not None:
            pose.pop("age_s", None)
        return pose

    def publication(self, **changes):
        state = dict(
            status="active", mode="monitor", coherent_feedback=True,
            q_sdk=list(self.q_sdk), q_urdf=list(self.q_urdf), age_ms=[0.0, 3.0, 7.0, 20.0, 5.0, 1.0, 10.0],
        )
        state.update(changes)
        return state

    def test_explicit_startup_pose_is_display_only_without_publishing_telemetry(self):
        initial = self.dashboard.snapshot()
        self.assertIsNone(initial.get("last_pose"))
        self.now += 0.05
        pose = self.remember(captured=9.95)
        snapshot = self.dashboard.snapshot()
        self.assertEqual(pose["q_sdk"], self.q_sdk)
        self.assertEqual(pose["q_urdf"], self.q_urdf)
        self.assertEqual(pose["captured_monotonic"], 9.95)
        self.assertIs(pose["display_only"], True)
        self.assertNotIn("q_sdk", snapshot)
        self.assertNotIn("q_urdf", snapshot)
        self.assertFalse(snapshot["telemetry_fresh"])
        self.assertEqual(snapshot["publication_sequence"], initial["publication_sequence"])
        self.assertAlmostEqual(snapshot["age_s"], 0.05)

    def test_explicit_pose_copies_input_vectors(self):
        pose = self.remember()
        self.q_sdk[0], self.q_sdk[6] = 0.9, 0.8
        self.q_urdf[0] = 0.9
        self.assertEqual(self.retained_pose(), pose)

    def test_tuple_vectors_and_zero_capture_time_are_accepted(self):
        self.assertIs(self.dashboard.remember_pose(tuple(self.q_sdk), tuple(self.q_urdf), 0), True)
        pose = self.dashboard.snapshot()["last_pose"]
        self.assertEqual(pose["q_sdk"], self.q_sdk)
        self.assertEqual(pose["q_urdf"], self.q_urdf)
        self.assertEqual(pose["captured_monotonic"], 0)
        self.assertEqual(pose["age_s"], self.now)
        self.assertIs(self.dashboard.remember_pose(self.q_sdk, self.q_urdf, self.now), True)

    def test_invalid_explicit_pose_does_not_replace_previous_capture(self):
        pose = self.remember(captured=9.9)
        sequence = self.dashboard.snapshot()["publication_sequence"]
        invalid = []
        for vector_name, values, size in (("sdk", self.q_sdk, 7), ("urdf", self.q_urdf, 6)):
            vectors = [values[:-1], values + [0.0], None, "0" * size]
            vectors += [[bad] + values[1:] for bad in (None, math.nan, math.inf, -math.inf, True)]
            for vector in vectors:
                sdk, urdf = (vector, self.q_urdf) if vector_name == "sdk" else (self.q_sdk, vector)
                invalid.append((vector_name, sdk, urdf, 9.95))
        invalid += [
            ("timestamp", self.q_sdk, self.q_urdf, captured)
            for captured in (None, math.nan, math.inf, -math.inf, True, -0.01, self.now + 1.0)
        ]
        invalid += [
            ("gripper", self.q_sdk[:-1] + [opening], self.q_urdf, 9.95)
            for opening in (-0.01, 1.01)
        ]
        for label, sdk, urdf, captured in invalid:
            with self.subTest(field=label, sdk=sdk, urdf=urdf, captured=captured):
                result = self.dashboard.remember_pose(sdk, urdf, captured)
                self.assertIs(result, False, "Invalid display pose reported success")
                snapshot = self.dashboard.snapshot()
                self.assertEqual(self.retained_pose(snapshot), pose)
                self.assertEqual(snapshot["publication_sequence"], sequence)
                self.assertFalse(snapshot["telemetry_fresh"])

    def test_only_complete_fresh_publications_update_the_retained_pose(self):
        previous_capture = None
        for index, (status, mode) in enumerate((
            ("active", "follow"), ("active", "monitor"), ("active", "run"),
            ("inspection_active", "inspect"), ("leader_gap", "monitor"),
        )):
            self.now += 0.05
            sdk = [value + 0.01 * index for value in self.q_sdk]
            urdf = [self.profile.to_model(i, value) for i, value in enumerate(sdk[:6])]
            self.dashboard.publish(**self.publication(status=status, mode=mode, q_sdk=sdk, q_urdf=urdf))
            with self.subTest(status=status, mode=mode):
                snapshot = self.dashboard.snapshot()
                pose = snapshot["last_pose"]
                self.assertEqual(pose["q_sdk"], sdk)
                self.assertEqual(pose["q_urdf"], urdf)
                self.assertIs(pose["display_only"], True)
                self.assertTrue(snapshot["telemetry_fresh"])
                self.assertAlmostEqual(pose["captured_monotonic"], self.now - 0.02)
                if previous_capture is not None:
                    self.assertGreater(pose["captured_monotonic"], previous_capture)
                previous_capture = pose["captured_monotonic"]

    def test_partial_invalid_stale_or_incoherent_publications_preserve_the_last_pose(self):
        pose = self.remember(captured=9.9)
        candidates = [
            dict(status=status) for status in ("starting", "stopped", "inspection_waiting", "inspection_error")
        ]
        candidates += [dict(mode=mode) for mode in ("preview", "unknown")]
        candidates += [dict(coherent_feedback=False)]
        for field, values in (("q_sdk", self.q_sdk), ("q_urdf", self.q_urdf)):
            candidates += [{field: values[:-1]}, {field: values + [0.0]}]
            if field == "q_sdk":
                candidates.append({field: None})
            candidates += [{field: [bad] + values[1:]} for bad in (None, math.nan, math.inf, True)]
        candidates += [
            dict(age_ms=ages)
            for ages in (
                None, [0.0] * 6, [0.0] * 8, [None] * 7, [-1.0] * 7,
                [math.nan] * 7, [math.inf] * 7, [True] * 7,
                [self.dashboard.freshness_limit_s * 1000 + 1.0] * 7,
            )
        ]
        for changes in candidates:
            with self.subTest(changes=changes):
                self.now += 0.01
                # Distinct valid coordinates expose accidental cache replacement.
                state = self.publication(
                    q_sdk=[value + 0.05 for value in self.q_sdk],
                    q_urdf=[value + 0.05 for value in self.q_urdf],
                )
                state.update(changes)
                self.dashboard.publish(**state)
                self.assertEqual(self.retained_pose(), pose)
        missing_ages = self.publication()
        del missing_ages["age_ms"]
        self.dashboard.publish(**missing_ages)
        self.assertEqual(self.retained_pose(), pose)

    def test_missing_urdf_coordinates_are_derived_through_the_bound_transform(self):
        self.profile.axes[0].update(urdf_scale=-2.0, urdf_offset=0.15)
        state = self.publication()
        del state["q_urdf"]
        self.dashboard.publish(**state)
        snapshot = self.dashboard.snapshot()
        self.assertTrue(snapshot["telemetry_fresh"])
        self.assertEqual(snapshot["last_pose"]["q_sdk"], self.q_sdk)
        self.assertEqual(
            snapshot["last_pose"]["q_urdf"],
            [self.profile.to_model(i, value) for i, value in enumerate(self.q_sdk[:6])],
        )

    def test_unavailable_model_transform_does_not_interrupt_fresh_telemetry(self):
        def invalid_transform(index, value):
            raise ValueError("Synthetic model conversion failure")

        for profile in (SimpleNamespace(), SimpleNamespace(to_model=invalid_transform)):
            with self.subTest(has_transform=hasattr(profile, "to_model")):
                self.dashboard.profile = profile
                self.now += 0.05
                state = self.publication()
                del state["q_urdf"]
                previous_sequence = self.dashboard.snapshot()["publication_sequence"]
                self.dashboard.publish(**state)
                snapshot = self.dashboard.snapshot()
                self.assertTrue(snapshot["telemetry_fresh"])
                self.assertEqual(snapshot["q_sdk"], self.q_sdk)
                self.assertEqual(snapshot["age_ms"], state["age_ms"])
                self.assertEqual(snapshot["publication_sequence"], previous_sequence + 1)
                self.assertIsNone(snapshot.get("last_pose"))
                self.assertNotIn("q_urdf", snapshot)

    def test_older_or_equal_explicit_samples_do_not_replace_the_last_capture(self):
        pose = self.remember(captured=9.95)
        different_sdk = [value + 0.05 for value in self.q_sdk]
        different_urdf = [value + 0.05 for value in self.q_urdf]
        for captured in (9.9, 9.95):
            with self.subTest(captured=captured):
                self.assertIs(
                    self.dashboard.remember_pose(different_sdk, different_urdf, captured), False,
                )
                self.assertEqual(self.retained_pose(), pose)
        self.dashboard.remember_pose(different_sdk, different_urdf, 9.96)
        latest = self.dashboard.snapshot()["last_pose"]
        self.assertEqual(latest["captured_monotonic"], 9.96)
        self.assertEqual(latest["q_sdk"], different_sdk)

    def test_republished_older_frames_do_not_refresh_the_cached_capture(self):
        pose = self.remember(captured=9.95)
        self.now = 10.05
        self.dashboard.publish(**self.publication(
            q_sdk=[value + 0.05 for value in self.q_sdk],
            q_urdf=[value + 0.05 for value in self.q_urdf],
            age_ms=[110.0] * 7,
        ))
        self.assertTrue(self.dashboard.snapshot()["telemetry_fresh"])
        self.assertEqual(self.retained_pose(), pose)

    def test_clear_stop_and_startup_error_keep_the_original_capture_without_fake_samples(self):
        self.dashboard.publish(**self.publication())
        pose = self.retained_pose()
        for status, extra in (
            ("starting", {}),
            ("stopped", {}),
            ("stopped", {"error": "Synthetic startup failure", "error_code": "startup_pose_mismatch"}),
            ("inspection_error", {"error": "Synthetic read failure"}),
        ):
            with self.subTest(status=status, extra=extra):
                self.now += 1.0
                self.dashboard.publish(status=status, mode="follow", **extra)
                snapshot = self.dashboard.snapshot()
                self.assertEqual(self.retained_pose(snapshot), pose)
                self.assertNotIn("q_sdk", snapshot)
                self.assertNotIn("q_urdf", snapshot)
                self.assertNotIn("age_ms", snapshot)
                self.assertFalse(snapshot["telemetry_fresh"])

    def test_snapshot_aging_does_not_refresh_the_retained_capture(self):
        self.dashboard.publish(**self.publication())
        first = self.dashboard.snapshot()
        pose = self.retained_pose(first)
        self.now += self.dashboard.freshness_limit_s + 0.01
        second = self.dashboard.snapshot()
        self.assertEqual(self.retained_pose(second), pose)
        self.assertAlmostEqual(first["last_pose"]["age_s"], 0.02)
        self.assertAlmostEqual(second["last_pose"]["age_s"], self.now - pose["captured_monotonic"])
        self.assertGreater(second["last_pose"]["age_s"], first["last_pose"]["age_s"])
        self.assertEqual(second["publication_sequence"], first["publication_sequence"])
        self.assertFalse(second["telemetry_fresh"])
