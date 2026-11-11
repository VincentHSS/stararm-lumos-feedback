"""Offline native-unit contract regressions; never construct a device controller."""

import http.client
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from test_feedback_runtime import FixtureCase

from stararm_lumos_feedback.config import (
    REVIEWED_NATIVE_CORE,
    REVIEWED_NATIVE_RANGES,
    SDKMotorRangeError,
    native_motor_range_mismatches,
    verify_sdk_imports,
)
from stararm_lumos_feedback.control import Caps
from stararm_lumos_feedback.dashboard import Dashboard


class NativeRangeTests(FixtureCase):
    def mark_reviewed_core(self):
        native = Path(self.profile.session["robot"]["sdk_root"]) / "src/libstartouch.so"
        # Synthetic binding metadata only; no vendor binary is imported.
        self.profile.files[str(native)] = REVIEWED_NATIVE_CORE

    def test_known_core_mismatch_rejected_before_any_native_import(self):
        self.mark_reviewed_core()
        for axis, ranges in zip(self.profile.axes, REVIEWED_NATIVE_RANGES):
            axis.update(zip(("p_max", "v_max", "t_max"), ranges))
        for axis in self.profile.axes[:3]:
            axis["v_max"] = 10.0
        with patch("importlib.import_module") as load:
            with self.assertRaises(SDKMotorRangeError) as caught:
                verify_sdk_imports(self.profile)
        load.assert_not_called()
        self.assertEqual(caught.exception.code, "sdk_motor_range_mismatch")
        self.assertEqual(caught.exception.detail["mismatches"], [
            dict(axis=i, parameter="v_max", sdk=8.0, hardware=10.0) for i in (1, 2, 3)
        ])
        self.assertIn("before SDK construction", str(caught.exception))

    def test_all_native_ranges_checked_without_changing_hardware_profile(self):
        self.mark_reviewed_core()
        for axis, ranges in zip(self.profile.axes, REVIEWED_NATIVE_RANGES):
            axis.update(zip(("p_max", "v_max", "t_max"), ranges))
        self.assertEqual(native_motor_range_mismatches(self.profile), [])
        for i, key in ((0, "p_max"), (3, "v_max"), (6, "t_max")):
            original = self.profile.axes[i][key]
            self.profile.axes[i][key] = original * 1.1
            self.assertEqual(native_motor_range_mismatches(self.profile), [
                dict(axis=i + 1, parameter=key, sdk=original, hardware=original * 1.1)
            ])
            self.assertEqual(self.profile.axes[i][key], original * 1.1)
            self.profile.axes[i][key] = original

    def test_other_core_has_no_invented_static_range_contract(self):
        self.assertEqual(native_motor_range_mismatches(self.profile), [])

    def test_http_rejection_preserves_localized_code_and_numeric_evidence(self):
        mismatch = [dict(axis=1, parameter="v_max", sdk=8, hardware=10)]

        def reject(*args, **kwargs):
            raise SDKMotorRangeError(mismatch)

        dashboard = Dashboard(Caps(), self.profile, port=0)
        dashboard.connection = SimpleNamespace(request=reject)
        dashboard.start()
        self.addCleanup(dashboard.close)
        client = http.client.HTTPConnection("127.0.0.1", dashboard.server.server_port, timeout=2)
        self.addCleanup(client.close)
        client.request("POST", "/api/connection", json.dumps(dict(action="start", mode="follow")), {
            "Content-Type": "application/json", "X-Feedback-Token": dashboard.token,
        })
        response = client.getresponse()
        payload = json.loads(response.read())
        self.assertEqual(response.status, 400)
        self.assertEqual(payload["code"], "sdk_motor_range_mismatch")
        self.assertEqual(payload["detail"], dict(mismatches=mismatch))
