"""Encoder edge measurements remain distinct from strict command limits."""

import math
import time
from types import SimpleNamespace

from test_feedback_runtime import FixtureCase

from stararm_lumos_feedback.control import Caps
from stararm_lumos_feedback.runtime import Session
from stararm_lumos_feedback.transport import Sample


class EnabledMeasurementLimitTests(FixtureCase):
    def setUp(self):
        super().setUp()
        self.profile.engineering = True
        self.profile.limits = tuple(list(row) for row in self.profile.limits)
        self.profile.limits[1][2] = 0
        self.margin = self.profile.axes[2]["limit_margin"] = 4 * 25 / 65535
        now = time.monotonic()
        self.reference = dict(
            startup_strategy="leader_target", actual_q=[0] * 7,
            frame_stamps=[now - 0.01] * 7,
            tolerance_rad=[math.radians(2)] * 6, captured_monotonic=now,
        )
        self.robot = SimpleNamespace(startup_strategy="leader_target", startup_reference=self.reference)
        self.monitor = SimpleNamespace(error=None, armed=False)
        self.session = Session(self.profile, self.robot, object(), Caps(), mode="follow", monitor=self.monitor)

    def group(self, position):
        return tuple(Sample(
            position if i == 2 else 0, 0, 0, 30, time.monotonic(), 1,
            0 if i == 6 else None, 0 if i == 6 else None,
        ) for i in range(7))

    def test_enabled_can_keeps_true_boundary_measurement_within_existing_margin(self):
        for position in (25 / 65535 / 2, self.margin):
            with self.subTest(position=position):
                self.monitor.snapshot = lambda: self.group(position)
                actual = self.session.current_startup_feedback(self.reference)
                self.assertEqual(actual[2], position)
                self.assertEqual(self.reference["actual_q"][2], 0)
                self.assertEqual(self.profile.limits[1][2], 0)
                self.assertTrue(self.monitor.armed)

    def test_enabled_can_outside_existing_measurement_margin_still_rejects(self):
        self.monitor.snapshot = lambda: self.group(self.margin + 25 / 65535)
        with self.assertRaisesRegex(RuntimeError, "outside joint limits.*measurement_margin="):
            self.session.current_startup_feedback(self.reference)
        self.assertFalse(self.monitor.armed)
