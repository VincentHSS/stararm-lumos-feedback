"""Offline build/publish boundaries; fixtures never construct SDK devices or open CAN."""

import contextlib
import importlib.util
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("sdk_ranges_builder", ROOT / "tools/build_sdk_motor_ranges.py")
builder = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(builder)
NATIVE_SOURCE = builder.SOURCE_PATH


class MotorRangesBuildTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.sdk = self.root / "startouch_sdk"
        self.native = self.sdk / "src/libstartouch.so"
        self.native.parent.mkdir(parents=True)
        self.native.write_bytes(b"synthetic core; never loaded")
        self.interface = self.sdk / "interface_py"
        self.interface.mkdir()
        self.library = self.interface / builder.LIBRARY_NAME
        self.manifest = self.interface / builder.MANIFEST_NAME
        self.source = self.root / "source.cpp"
        self.source.write_bytes(b"// synthetic compiler input, never loaded\n")
        source = patch.object(builder, "SOURCE_PATH", self.source)
        source.start()
        self.addCleanup(source.stop)

    def mock_build(self, *, probe_code=0, compiler_code=0, change_core=False):
        calls = []

        def run(argv, **kwargs):
            calls.append((argv, kwargs))
            if len(calls) == 1:
                if not compiler_code:
                    Path(argv[argv.index("-o") + 1]).write_bytes(b"synthetic verified library")
                if change_core:
                    self.native.write_bytes(b"changed during compilation")
                return subprocess.CompletedProcess(argv, compiler_code, "", "compiler diagnostic")
            return subprocess.CompletedProcess(argv, probe_code, "", "native codec diagnostic")

        return run, calls

    def fixture_core(self):
        return patch.object(builder, "CORE_SHA256", builder.sha256(self.native))

    def assert_no_artifacts(self):
        self.assertEqual(list(self.interface.iterdir()), [])

    def test_unknown_core_is_rejected_before_compilation_without_modifying_it(self):
        before = self.native.read_bytes()
        with patch.object(builder.subprocess, "run") as run:
            with self.assertRaisesRegex(ValueError, "Unsupported native core SHA-256"):
                builder.build_sdk_motor_ranges(self.sdk)
        run.assert_not_called()
        self.assertEqual(self.native.read_bytes(), before)
        self.assert_no_artifacts()

    def test_existing_library_manifest_or_dangling_link_is_never_overwritten(self):
        for target, dangling in ((self.library, False), (self.manifest, False), (self.library, True)):
            with self.subTest(target=target.name, dangling=dangling):
                if dangling:
                    target.symlink_to(self.root / "missing")
                else:
                    target.write_bytes(b"preserve me")
                with patch.object(builder.subprocess, "run") as run:
                    with self.assertRaisesRegex(FileExistsError, "preserved"):
                        builder.build_sdk_motor_ranges(self.sdk)
                run.assert_not_called()
                if dangling:
                    self.assertTrue(target.is_symlink())
                else:
                    self.assertEqual(target.read_bytes(), b"preserve me")
                target.unlink()

    def test_native_and_interface_symlinks_cannot_write_outside_selected_sdk(self):
        outside = self.root / "outside"
        outside.mkdir()
        other_core = outside / "libstartouch.so"
        other_core.write_bytes(self.native.read_bytes())
        self.native.unlink()
        self.native.symlink_to(other_core)
        with self.assertRaisesRegex(ValueError, "regular file"):
            builder.build_sdk_motor_ranges(self.sdk)
        self.native.unlink()
        self.native.write_bytes(other_core.read_bytes())
        self.interface.rmdir()
        self.interface.symlink_to(outside, target_is_directory=True)
        with self.assertRaisesRegex(ValueError, "interface_py"):
            builder.build_sdk_motor_ranges(self.sdk)
        self.assertEqual(list(outside.iterdir()), [other_core])

    def test_compiler_and_probe_failures_publish_nothing(self):
        for compiler_code, probe_code in ((1, 0), (0, 1), (0, -11)):
            with self.subTest(compiler=compiler_code, probe=probe_code):
                run, calls = self.mock_build(compiler_code=compiler_code, probe_code=probe_code)
                with self.fixture_core(), patch.object(builder.shutil, "which", return_value="/mock/c++"):
                    with patch.object(builder.subprocess, "run", side_effect=run):
                        with self.assertRaisesRegex(RuntimeError, "compilation failed|codec verification failed"):
                            builder.build_sdk_motor_ranges(self.sdk)
                self.assertEqual(len(calls), 1 if compiler_code else 2)
                self.assert_no_artifacts()

    def test_core_change_during_build_is_rejected_before_probe(self):
        run, calls = self.mock_build(change_core=True)
        with self.fixture_core(), patch.object(builder.shutil, "which", return_value="/mock/c++"):
            with patch.object(builder.subprocess, "run", side_effect=run):
                with self.assertRaisesRegex(ValueError, "Unsupported native core SHA-256"):
                    builder.build_sdk_motor_ranges(self.sdk)
        self.assertEqual(len(calls), 1)
        self.assert_no_artifacts()

    def test_success_uses_isolated_global_probe_and_publishes_exact_hash_manifest(self):
        run, calls = self.mock_build()
        with self.fixture_core(), patch.object(builder.shutil, "which", return_value="/mock/c++"):
            with patch.dict(os.environ, {"LD_PRELOAD": "untrusted.so", "LD_AUDIT": "untrusted-audit.so"}):
                with patch.object(builder.subprocess, "run", side_effect=run):
                    manifest = builder.build_sdk_motor_ranges(self.sdk)
        self.assertEqual(manifest, json.loads(self.manifest.read_text(encoding="utf-8")))
        self.assertEqual(manifest, {
            "version": 1,
            "core_sha256": builder.sha256(self.native),
            "source_sha256": builder.sha256(self.source),
            "library_sha256": builder.sha256(self.library),
        })
        self.assertEqual(set(self.interface.iterdir()), {self.library, self.manifest})
        probe, options = calls[1]
        self.assertEqual(probe[:4], [sys.executable, "-I", "-c", builder.PROBE])
        self.assertEqual(probe[-1], str(self.native))
        self.assertNotIn("LD_PRELOAD", options["env"])
        self.assertNotIn("LD_AUDIT", options["env"])
        self.assertIn("os.RTLD_GLOBAL", builder.PROBE)

    def test_publication_race_preserves_conflict_and_removes_our_partial_library(self):
        real_link = os.link
        run, _ = self.mock_build()

        def race(source, target):
            if target == self.manifest:
                target.write_bytes(b"concurrent manifest")
            return real_link(source, target)

        with self.fixture_core(), patch.object(builder.shutil, "which", return_value="/mock/c++"):
            with patch.object(builder.subprocess, "run", side_effect=run), patch.object(builder.os, "link", race):
                with self.assertRaises(FileExistsError):
                    builder.build_sdk_motor_ranges(self.sdk)
        self.assertFalse(self.library.exists())
        self.assertEqual(self.manifest.read_bytes(), b"concurrent manifest")
        self.assertEqual(list(self.interface.iterdir()), [self.manifest])

    def test_cli_failure_returns_nonzero_and_does_not_create_artifacts(self):
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()) as errors:
            self.assertEqual(builder.main(["--sdk-root", str(self.sdk)]), 2)
        self.assertIn("Unsupported native core SHA-256", errors.getvalue())
        self.assert_no_artifacts()


class MotorRangesNativeSurfaceTests(unittest.TestCase):
    def test_source_exports_only_the_reviewed_surface_and_bad_path_never_loads_native(self):
        compiler, nm = shutil.which("c++"), shutil.which("nm")
        if not compiler or not nm or sys.platform != "linux":
            self.skipTest("source ABI verification needs Linux, a C++ compiler and nm")
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            library, exports = root / "ranges.so", root / "exports.map"
            exports.write_text(builder.EXPORTS, encoding="utf-8")
            subprocess.run(
                [compiler, "-std=c++17", "-O2", "-shared", "-fPIC", "-fvisibility=hidden", "-fno-gnu-unique",
                 str(NATIVE_SOURCE), "-o", str(library), "-ldl", "-pthread", f"-Wl,--version-script={exports}"],
                capture_output=True, text=True, timeout=60, check=True,
            )
            symbols = subprocess.run(
                [nm, "-D", "--defined-only", str(library)],
                capture_output=True, text=True, timeout=5, check=True,
            )
            self.assertEqual({line.split()[-1] for line in symbols.stdout.splitlines()}, {
                "_ZN9startouch12damiao_motor18MOTOR_LIMIT_PARAMSE", "stararm_ranges_verify", "stararm_ranges_error",
            })
            probe = subprocess.run(
                [sys.executable, "-I", "-c", builder.PROBE, str(library), "relative-invalid-path"],
                capture_output=True, text=True, timeout=5, check=False,
            )
            self.assertEqual(probe.returncode, 1)
            self.assertIn("absolute native core path", probe.stderr)


if __name__ == "__main__":
    unittest.main()
