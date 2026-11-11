"""Exact SDK source patch plus compiled, device-free constructor semantics.

The fixture is the byte-exact pybind source at SDK commit
040412542c0b6b6fd2bb6dd652c0de6c666631be. Native tests extract its actual constructor
lambda, with/without the patch, into a fake controller. They never load libstartouch
or construct/connect a real hardware controller.
"""

import contextlib
import hashlib
import importlib.util
import io
import os
import shutil
import subprocess
import sys
import sysconfig
import tempfile
import textwrap
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("sdk_gil_patcher", ROOT / "tools/patch_sdk_constructor_gil.py")
patcher = importlib.util.module_from_spec(spec)
spec.loader.exec_module(patcher)
ORIGINAL = (Path(__file__).parent / "fixtures/sdk_0404125_pybind_constructor.cpp").read_bytes()


class SDKConstructorPatchTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.source = self.root / patcher.SOURCE_PATH
        self.source.parent.mkdir()
        self.source.write_bytes(ORIGINAL)

    def test_exact_source_hash_and_reversible_reviewed_diff(self):
        self.assertEqual(hashlib.sha256(ORIGINAL).hexdigest(), patcher.ORIGINAL_SHA256)
        updated = patcher.patched_source(ORIGINAL)
        self.assertEqual(hashlib.sha256(updated).hexdigest(), patcher.PATCHED_SHA256)
        restored = updated.replace(patcher.PATCHED_RETURN, patcher.ORIGINAL_RETURN)
        restored = restored.replace(patcher.PATCHED_MODULE_START, patcher.MODULE_START)
        self.assertEqual(restored, ORIGINAL)
        self.assertEqual(updated.count(b"py::gil_scoped_release release_gil;"), 1)

    def test_explicit_copy_patch_is_idempotent_and_preserves_permissions(self):
        self.source.chmod(0o640)
        untouched = self.root / "unchanged.txt"
        untouched.write_text("unrelated copied resource")
        self.assertEqual(patcher.patch_sdk(self.root), "updated")
        self.assertEqual(self.source.read_bytes(), patcher.patched_source(ORIGINAL))
        self.assertEqual(self.source.stat().st_mode & 0o777, 0o640)
        modified = self.source.stat().st_mtime_ns
        self.assertEqual(patcher.patch_sdk(self.root), "patched")
        self.assertEqual(self.source.stat().st_mtime_ns, modified)
        self.assertEqual(untouched.read_text(), "unrelated copied resource")
        self.assertEqual(set(self.source.parent.iterdir()), {self.source})

    def test_check_distinguishes_original_patched_and_unsupported_without_writes(self):
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(patcher.main([str(self.root), "--check"]), 1)
            self.assertEqual(self.source.read_bytes(), ORIGINAL)
            self.assertEqual(patcher.main([str(self.root)]), 0)
            modified = self.source.stat().st_mtime_ns
            self.assertEqual(patcher.main([str(self.root), "--check"]), 0)
            self.assertEqual(self.source.stat().st_mtime_ns, modified)
            unknown = self.source.read_bytes() + b"// extra change\n"
            self.source.write_bytes(unknown)
            self.assertEqual(patcher.main([str(self.root), "--check"]), 2)
            self.assertEqual(self.source.read_bytes(), unknown)

    def test_unknown_partial_duplicate_and_line_ending_variants_are_refused(self):
        updated = patcher.patched_source(ORIGINAL)
        variants = (
            ORIGINAL + b"// unrelated edit\n",
            ORIGINAL.replace(b"\n", b"\r\n"),
            ORIGINAL.replace(patcher.ORIGINAL_RETURN, patcher.PATCHED_RETURN),
            ORIGINAL.replace(patcher.MODULE_START, patcher.PATCHED_MODULE_START),
            updated.replace(patcher.PATCHED_RETURN, patcher.PATCHED_RETURN * 2),
            updated.replace(patcher.PATCHED_MODULE_START, patcher.PATCHED_MODULE_START * 2),
        )
        for variant in variants:
            with self.subTest(digest=hashlib.sha256(variant).hexdigest()):
                self.source.write_bytes(variant)
                with self.assertRaisesRegex(ValueError, "Unsupported SDK binding source"):
                    patcher.patch_sdk(self.root)
                self.assertEqual(self.source.read_bytes(), variant)

    def test_file_and_directory_symlinks_cannot_patch_another_sdk(self):
        external = self.root / "external"
        external.mkdir()
        external_source = external / self.source.name
        external_source.write_bytes(ORIGINAL)
        self.source.unlink()
        self.source.symlink_to(external_source)
        with self.assertRaisesRegex(ValueError, "regular file"):
            patcher.patch_sdk(self.root)
        self.source.unlink()
        self.source.parent.rmdir()
        self.source.parent.symlink_to(external, target_is_directory=True)
        with self.assertRaisesRegex(ValueError, "regular file"):
            patcher.patch_sdk(self.root)
        self.assertEqual(external_source.read_bytes(), ORIGINAL)

    def test_replace_failure_keeps_original_and_removes_temporary_file(self):
        with patch.object(patcher.os, "replace", side_effect=OSError("replace unavailable")):
            with self.assertRaisesRegex(OSError, "replace unavailable"):
                patcher.patch_sdk(self.root)
        self.assertEqual(self.source.read_bytes(), ORIGINAL)
        self.assertEqual(set(self.source.parent.iterdir()), {self.source})


class SDKConstructorNativeSemanticsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("c++") or shutil.which("g++")
        python_include = Path(sysconfig.get_path("include"))
        pybind_candidates = [
            Path(sysconfig.get_path("purelib")) / "pybind11/include",
            Path(sysconfig.get_path("purelib")) / "torch/include",
            Path("/usr/local/include"), Path("/usr/include"),
        ]
        pybind_include = next((path for path in pybind_candidates if (path / "pybind11/pybind11.h").is_file()), None)
        if compiler is None or not (python_include / "Python.h").is_file() or pybind_include is None:
            raise unittest.SkipTest("native GIL semantics require a C++ compiler and Python/pybind11 headers")
        temporary = tempfile.TemporaryDirectory()
        cls.addClassCleanup(temporary.cleanup)
        cls.build = Path(temporary.name)
        for name, source in (("sdk_gil_original", ORIGINAL), ("sdk_gil_patched", patcher.patched_source(ORIGINAL))):
            text = source.decode("utf-8")
            start = text.index("        .def(py::init([](py::kwargs kwargs) {")
            end = text.index("        }))", start) + len("        }))")
            constructor = text[start:end]
            marker = 'm.attr("_stararm_constructor_gil_release") = 1;' if name.endswith("patched") else ""
            native_source = textwrap.dedent(r'''
                #include <pybind11/pybind11.h>
                #include <atomic>
                #include <chrono>
                #include <stdexcept>
                #include <string>
                #include <thread>
                namespace py = pybind11;
                namespace startouch::damiao_motor { enum class ControlMode { MIT_MODE }; }
                std::atomic<bool> entered{false}, python_progress{false};
                class ArmController {
                public:
                    bool gil_held, saw_python_progress, gripper, fd, dry;
                    std::string interface;
                    ArmController(bool gripper_exist, std::string can_interface, bool enable_fd,
                                  startouch::damiao_motor::ControlMode, bool dry_run)
                        : gil_held(PyGILState_Check()), saw_python_progress(false),
                          gripper(gripper_exist), fd(enable_fd), dry(dry_run), interface(can_interface) {
                        entered.store(true);
                        if (can_interface == "throw") throw std::runtime_error("native constructor failed");
                        if (can_interface == "thread") {
                            auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
                            while (!python_progress.load() && std::chrono::steady_clock::now() < deadline)
                                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                            saw_python_progress = python_progress.load();
                        }
                    }
                };
                PYBIND11_MODULE(MODULE_NAME, m) {
                    MODULE_MARKER
                    m.def("reset", [] { entered.store(false); python_progress.store(false); });
                    m.def("constructor_started", [] { return entered.load(); });
                    m.def("mark_python_progress", [] { python_progress.store(true); });
                    py::class_<ArmController>(m, "ArmController", py::module_local())
                    CONSTRUCTOR_BINDING
                    .def_readonly("gil_held", &ArmController::gil_held)
                    .def_readonly("saw_python_progress", &ArmController::saw_python_progress)
                    .def_readonly("gripper", &ArmController::gripper)
                    .def_readonly("fd", &ArmController::fd)
                    .def_readonly("dry", &ArmController::dry)
                    .def_readonly("interface", &ArmController::interface);
                }
            ''').replace("MODULE_NAME", name).replace("MODULE_MARKER", marker).replace("CONSTRUCTOR_BINDING", constructor)
            cpp = cls.build / f"{name}.cpp"
            cpp.write_text(native_source)
            output = cls.build / f"{name}{sysconfig.get_config_var('EXT_SUFFIX')}"
            command = [
                compiler, "-O0", "-shared", "-std=c++17", "-fPIC", "-pthread",
                f"-I{python_include}", f"-I{pybind_include}", str(cpp), "-o", str(output),
            ]
            result = subprocess.run(command, capture_output=True, text=True, timeout=90)
            if result.returncode:
                raise AssertionError(f"No-device semantic extension did not compile:\n{result.stderr}")

    def test_actual_constructor_lambda_releases_gil_after_casts_and_restores_on_throw(self):
        script = textwrap.dedent('''
            import gc
            import importlib
            import sys
            import threading
            import time
            sys.path.insert(0, sys.argv[1])
            for name, patched in (("sdk_gil_original", False), ("sdk_gil_patched", True)):
                module = importlib.import_module(name)
                assert getattr(module, "_stararm_constructor_gil_release", 0) == int(patched)
                module.reset()
                stop = threading.Event()
                ready = threading.Event()
                def python_receiver():
                    ready.set()
                    while not stop.is_set():
                        if module.constructor_started():
                            module.mark_python_progress()
                            return
                        time.sleep(0.001)
                worker = threading.Thread(target=python_receiver, daemon=True)
                worker.start()
                assert ready.wait(2)
                try:
                    controller = module.ArmController(
                        gripper_exist=True, can_interface="thread", enable_fd=True, dry_run=True)
                finally:
                    stop.set()
                    worker.join(2)
                assert not worker.is_alive()
                assert controller.gil_held is (not patched)
                assert controller.saw_python_progress is patched
                assert controller.gripper and controller.fd and controller.dry
                assert controller.interface == "thread"

                module.reset()
                try:
                    module.ArmController(can_interface=object())
                except (TypeError, RuntimeError):
                    pass
                else:
                    raise AssertionError("Invalid kwargs did not fail conversion")
                assert not module.constructor_started(), "kwargs conversion entered native constructor"

                try:
                    module.ArmController(can_interface="throw")
                except RuntimeError as error:
                    assert str(error) == "native constructor failed"
                else:
                    raise AssertionError("native exception not translated")
                gc.collect()
                # A new kwargs conversion and Python return after unwinding require the GIL again.
                default = module.ArmController()
                assert default.interface == "can0"
                assert not default.gripper and not default.fd and not default.dry
                assert default.gil_held is (not patched)
            print("Original blocks Python; patched native constructor releases GIL; casts/errors preserve it.")
        ''')
        environment = dict(os.environ, PYTHONFAULTHANDLER="1")
        result = subprocess.run(
            [sys.executable, "-B", "-c", script, str(self.build)],
            capture_output=True, text=True, timeout=15, env=environment,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("patched native constructor releases GIL", result.stdout)


if __name__ == "__main__":
    unittest.main()
