"""Compatibility source/build identity and pre-construction loading; no hardware."""

import hashlib
import json
import threading
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from test_feedback_runtime import FixtureCase

from stararm_lumos_feedback import native_compat as compat
from stararm_lumos_feedback.config import (
    SDKMotorRangeError,
    native_motor_range_mismatches,
    verify_sdk_imports,
)


class NativeCompatibilityTests(FixtureCase):
    def setUp(self):
        super().setUp()
        sdk = Path(self.profile.session["robot"]["sdk_root"])
        self.native = sdk / "src/libstartouch.so"
        self.manifest, self.library = (sdk / item for item in compat.ARTIFACTS)
        self.source = sdk / "test-native-source.cpp"
        self.source.write_text("// synthetic compatibility source, never compiled\n")
        self.source_patch = patch.object(compat, "SOURCE", self.source)
        self.source_patch.start()
        self.addCleanup(self.source_patch.stop)
        self.active_patch = patch.object(compat, "_active", None)
        self.active_patch.start()
        self.addCleanup(self.active_patch.stop)
        self.profile.files[str(self.native)] = compat.CORE_SHA256
        for axis, ranges in zip(self.profile.axes, compat.CORRECTED_RANGES):
            axis.update(zip(("p_max", "v_max", "t_max"), ranges))

    def install(self, **changes):
        self.library.write_bytes(b"synthetic shim, never loaded")
        data = dict(
            version=1, core_sha256=compat.CORE_SHA256,
            source_sha256=hashlib.sha256(self.source.read_bytes()).hexdigest(),
            library_sha256=hashlib.sha256(self.library.read_bytes()).hexdigest(),
        )
        data.update(changes)
        self.manifest.write_text(json.dumps(data))
        for path in (self.manifest, self.library):
            self.profile.files[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()

    def core_sha(self, path):
        return compat.CORE_SHA256 if path == self.native else hashlib.sha256(path.read_bytes()).hexdigest()

    def test_absent_partial_unbound_and_altered_builds_cannot_clear_range_gate(self):
        self.assertIsNone(compat.compatibility_info(self.profile))
        self.assertEqual(len(native_motor_range_mismatches(self.profile)), 3)
        self.install()
        self.assertEqual(native_motor_range_mismatches(self.profile), [])
        for path in (self.library, self.manifest, self.source):
            with self.subTest(path=path):
                original = path.read_bytes()
                path.write_bytes(original + b" ")
                self.assertIsNone(compat.compatibility_info(self.profile))
                self.assertEqual(len(native_motor_range_mismatches(self.profile)), 3)
                path.write_bytes(original)
        self.profile.files.pop(str(self.manifest))
        self.assertIsNone(compat.compatibility_info(self.profile))
        self.install()
        self.library.unlink()
        self.assertIsNone(compat.compatibility_info(self.profile))

    def test_manifest_contract_rejects_wrong_core_source_binary_version_and_extra_fields(self):
        for changes in (
            dict(version=True), dict(version=2), dict(core_sha256="wrong"),
            dict(source_sha256="wrong"), dict(library_sha256="wrong"), dict(unreviewed=True),
        ):
            with self.subTest(changes=changes):
                self.install(**changes)
                self.assertIsNone(compat.compatibility_info(self.profile))

    def test_other_core_and_symlinked_artifacts_are_not_accepted(self):
        self.install()
        self.profile.files[str(self.native)] = "other core"
        self.assertIsNone(compat.compatibility_info(self.profile))
        self.profile.files[str(self.native)] = compat.CORE_SHA256
        copy = self.library.with_suffix(".copy")
        self.library.rename(copy)
        self.library.symlink_to(copy)
        self.assertIsNone(compat.compatibility_info(self.profile))

    def test_correction_still_rejects_any_actual_range_disagreement(self):
        self.install()
        self.profile.axes[0]["v_max"] = 8
        self.assertEqual(native_motor_range_mismatches(self.profile), [
            dict(axis=1, parameter="v_max", sdk=10.0, hardware=8),
        ])

    def test_actual_core_change_and_preloaded_native_fail_before_dlopen(self):
        self.install()
        with patch.object(compat.ctypes, "CDLL") as load:
            with self.assertRaisesRegex(compat.SDKCompatibilityError, "core changed"):
                compat.load_compatibility(self.profile)
            load.assert_not_called()
        for name in ("libstartouch.so", "stararm_motor_ranges.so"):
            with (
                self.subTest(name=name),
                patch.object(compat, "_sha", side_effect=self.core_sha),
                patch.object(compat, "_mapped_libraries", side_effect=lambda n: {self.native} if n == name else set()),
                patch.object(compat.ctypes, "CDLL") as load,
                self.assertRaisesRegex(compat.SDKCompatibilityError, "already loaded"),
            ):
                compat.load_compatibility(self.profile)
            load.assert_not_called()

    def test_failed_actual_codec_probe_never_authorizes_reuse(self):
        self.install()
        library = Mock()
        library.stararm_ranges_verify.return_value = 1
        library.stararm_ranges_error.return_value = b"native encoder still uses 8"
        with (
            patch.object(compat, "_sha", side_effect=self.core_sha),
            patch.object(compat, "_mapped_libraries", return_value=set()),
            patch.object(compat.ctypes, "CDLL", return_value=library),
            self.assertRaisesRegex(compat.SDKCompatibilityError, "encoder still uses 8"),
        ):
            compat.load_compatibility(self.profile)
        self.assertIsNone(compat._active)

    def test_successful_probe_is_before_native_import_and_reuses_only_same_loaded_build(self):
        self.install()
        library = Mock()
        library.stararm_ranges_verify.return_value = 0
        loaded = False

        def maps(name):
            if not loaded:
                return set()
            return {self.native.resolve()} if name == "libstartouch.so" else {self.library.resolve()}

        def probe(path):
            nonlocal loaded
            self.assertEqual(path, bytes(self.native))
            loaded = True
            return 0

        library.stararm_ranges_verify.side_effect = probe
        with (
            patch.object(compat, "_sha", side_effect=self.core_sha),
            patch.object(compat, "_mapped_libraries", side_effect=maps),
            patch.object(compat.ctypes, "CDLL", return_value=library) as load,
            patch("importlib.import_module") as sdk_import,
        ):
            self.assertTrue(compat.load_compatibility(self.profile))
            self.assertTrue(compat.load_compatibility(self.profile))
            load.assert_called_once_with(str(self.library), mode=compat.os.RTLD_NOW | compat.os.RTLD_GLOBAL)
            library.stararm_ranges_verify.assert_called_once()
            sdk_import.assert_not_called()
        with (
            patch.object(compat, "_sha", side_effect=self.core_sha),
            patch.object(compat, "_mapped_libraries", return_value=set()),
            self.assertRaisesRegex(compat.SDKCompatibilityError, "build changed"),
        ):
            compat.load_compatibility(self.profile)

    def test_unexpected_probe_library_path_rejected(self):
        self.install()
        library = Mock()
        library.stararm_ranges_verify.return_value = 0
        with (
            patch.object(compat, "_sha", side_effect=self.core_sha),
            patch.object(compat, "_mapped_libraries", side_effect=[set(), set(), {Path("/other/libstartouch.so")}]),
            patch.object(compat.ctypes, "CDLL", return_value=library),
            self.assertRaisesRegex(compat.SDKCompatibilityError, "unexpected library path"),
        ):
            compat.load_compatibility(self.profile)
        self.assertIsNone(compat._active)

    def test_active_correction_cannot_silently_fall_back_to_an_old_profile(self):
        self.install()
        self.manifest.unlink()
        for active in (None, ((self.native, self.library, "old hash"), object())):
            with (
                self.subTest(active=active is not None),
                patch.object(compat, "_active", active),
                patch.object(compat, "_mapped_libraries", return_value={self.library.resolve()}),
                patch.object(compat.ctypes, "CDLL") as load,
                self.assertRaisesRegex(compat.SDKCompatibilityError, "no longer matches"),
            ):
                compat.load_compatibility(self.profile)
            load.assert_not_called()

    def test_loader_selection_race_in_either_direction_is_rejected_before_sdk_import(self):
        for selected, actually_loaded in ((True, False), (False, True)):
            ranges = compat.CORRECTED_RANGES if selected else compat.NATIVE_RANGES
            for axis, values in zip(self.profile.axes, ranges):
                axis.update(zip(("p_max", "v_max", "t_max"), values))
            with (
                self.subTest(selected=selected),
                patch.object(compat, "compatibility_info", return_value={"selected": True} if selected else None),
                patch.object(compat, "load_compatibility", return_value=actually_loaded),
                patch("importlib.import_module") as sdk_import,
                self.assertRaises(SDKMotorRangeError),
            ):
                verify_sdk_imports(self.profile)
            sdk_import.assert_not_called()

    def test_concurrent_correction_waits_for_original_sdk_import_then_rejects_preloaded_core(self):
        self.install()
        original = SimpleNamespace(
            session=self.profile.session,
            files=dict(self.profile.files),
            axes=deepcopy(self.profile.axes),
        )
        for resource in compat.ARTIFACTS:
            original.files.pop(str(self.native.parent.parent / resource))
        for axis, ranges in zip(original.axes, compat.NATIVE_RANGES):
            axis.update(zip(("p_max", "v_max", "t_max"), ranges))

        import_entered = threading.Event()
        release_import = threading.Event()
        native_loaded = threading.Event()
        contender_probed = threading.Event()

        class ObservedRLock:
            """Observe actual contention without sleep or scheduler assumptions."""

            def __init__(self):
                self.lock = threading.RLock()
                self.contender_blocked = None

            def __enter__(self):
                if threading.current_thread().name == "corrected-profile" and not contender_probed.is_set():
                    acquired = self.lock.acquire(blocking=False)
                    self.contender_blocked = not acquired
                    contender_probed.set()
                    if not acquired:
                        self.lock.acquire()
                else:
                    self.lock.acquire()
                return self

            def __exit__(self, *_):
                self.lock.release()

        lock = ObservedRLock()
        imports = []
        errors = {}
        completed = []
        actual_read_text = Path.read_text
        native_bytes = self.native.read_bytes()

        def read_text(path, *args, **kwargs):
            if str(path) == "/proc/self/maps":
                return f"0000-1000 r-xp 0000 00:00 0 {self.native}\n" if native_loaded.is_set() else ""
            return actual_read_text(path, *args, **kwargs)

        def sdk_import(name):
            imports.append((threading.current_thread().name, name))
            if name == "startouchclass":
                import_entered.set()
                if not release_import.wait(5):
                    raise RuntimeError("test did not release the original SDK import")
                native_loaded.set()
            filename = "startouchclass.py" if name == "startouchclass" else "startouch.test.so"
            return SimpleNamespace(__file__=str(self.native.parent.parent / "interface_py" / filename))

        def run(name, profile):
            try:
                verify_sdk_imports(profile)
                completed.append(name)
            except Exception as exc:
                errors[name] = exc

        original_thread = threading.Thread(target=run, args=("original", original), name="original-profile")
        corrected_thread = threading.Thread(
            target=run, args=("corrected", self.profile), name="corrected-profile",
        )
        with (
            patch.object(compat, "sdk_load_lock", lock),
            patch.object(compat, "_sha", side_effect=self.core_sha),
            patch.object(Path, "read_text", read_text),
            patch(
                "stararm_lumos_feedback.config.digest",
                side_effect=lambda data: compat.CORE_SHA256 if data == native_bytes else hashlib.sha256(data).hexdigest(),
            ),
            patch("importlib.import_module", side_effect=sdk_import),
            patch.object(compat.ctypes, "CDLL") as load,
        ):
            try:
                original_thread.start()
                self.assertTrue(import_entered.wait(5), "original caller did not reach SDK import")
                corrected_thread.start()
                self.assertTrue(contender_probed.wait(5), "corrected caller did not attempt the shared lock")
                self.assertTrue(lock.contender_blocked, "correction overtook the original SDK import")
                self.assertEqual(imports, [("original-profile", "startouchclass")])
                load.assert_not_called()
            finally:
                release_import.set()
                original_thread.join(5)
                if corrected_thread.ident is not None:
                    corrected_thread.join(5)
            self.assertFalse(original_thread.is_alive())
            self.assertFalse(corrected_thread.is_alive())
            load.assert_not_called()

        self.assertEqual(completed, ["original"])
        self.assertEqual(set(errors), {"corrected"})
        self.assertIsInstance(errors["corrected"], compat.SDKCompatibilityError)
        self.assertIn("already loaded", str(errors["corrected"]))
        self.assertEqual(imports, [
            ("original-profile", "startouchclass"), ("original-profile", "startouch"),
        ])
