"""Build and verify motor ranges in an explicitly selected SDK copy, without hardware.

Only the reviewed 04041254 Linux x86-64 native core is accepted. The vendor core
is never modified. A new process exercises its actual MIT encoder and state
decoder before the compatibility library and hash manifest are published.
Existing artifacts are never overwritten. Regenerate the normal integration
binding after building; software verification does not establish physical results.
"""

import argparse
import hashlib
import json
import os
import platform
import shutil
import stat
import subprocess
import sys
import tempfile
from pathlib import Path

CORE_SHA256 = "0e6fe64a1983883acc2b3bf9b5092c708c0c3d30dc5219cc5fbc7d8474b9932e"
SOURCE_PATH = Path(__file__).resolve().parents[1] / "src/stararm_lumos_feedback/native/sdk_04041254_ranges.cpp"
LIBRARY_NAME = "stararm_motor_ranges.so"
MANIFEST_NAME = "stararm_motor_ranges.json"
EXPORTS = """{
global:
    _ZN9startouch12damiao_motor18MOTOR_LIMIT_PARAMSE;
    stararm_ranges_verify;
    stararm_ranges_error;
local: *;
};
"""
PROBE = """
import ctypes
import os
import sys

library = ctypes.CDLL(sys.argv[1], mode=os.RTLD_NOW | os.RTLD_GLOBAL)
library.stararm_ranges_verify.argtypes = [ctypes.c_char_p]
library.stararm_ranges_verify.restype = ctypes.c_int
library.stararm_ranges_error.argtypes = []
library.stararm_ranges_error.restype = ctypes.c_char_p
result = library.stararm_ranges_verify(os.fsencode(sys.argv[2]))
if result:
    print(library.stararm_ranges_error().decode('utf-8', errors='replace'), file=sys.stderr)
raise SystemExit(result)
"""


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def regular_native(root):
    native = root / "src/libstartouch.so"
    if native.is_symlink() or native.parent.resolve(strict=True) != root / "src":
        raise ValueError("Native core must be a regular file inside the selected SDK copy")
    if not stat.S_ISREG(native.stat().st_mode):
        raise ValueError("Native core must be a regular file inside the selected SDK copy")
    if sha256(native) != CORE_SHA256:
        raise ValueError("Unsupported native core SHA-256; only the reviewed 04041254 .so.22 core is accepted")
    return native


def require_absent(*paths):
    for path in paths:
        if os.path.lexists(path):
            raise FileExistsError(f"Existing SDK compatibility artifact is preserved: {path}")


def build_sdk_motor_ranges(sdk_root):
    """Build into this SDK copy only; never import its Python SDK or construct devices."""
    root = Path(sdk_root).expanduser().resolve(strict=True)
    if not root.is_dir():
        raise ValueError("An explicit SDK copy directory is required")
    interface = root / "interface_py"
    if interface.is_symlink() or interface.resolve(strict=True) != interface or not interface.is_dir():
        raise ValueError("interface_py must be a directory inside the selected SDK copy")
    library, manifest_path = interface / LIBRARY_NAME, interface / MANIFEST_NAME
    require_absent(library, manifest_path)
    native = regular_native(root)
    if sys.platform != "linux" or platform.machine() != "x86_64":
        raise ValueError("The reviewed compatibility ABI supports only Linux x86-64")
    compiler = shutil.which("c++") or shutil.which("g++")
    if compiler is None:
        raise RuntimeError("A C++17 compiler is required; no SDK artifact was created")
    source = SOURCE_PATH.read_bytes()
    source_hash = hashlib.sha256(source).hexdigest()
    with tempfile.TemporaryDirectory(prefix=".stararm-motor-ranges-", dir=interface) as temporary:
        staging = Path(temporary)
        staged_source, staged_library = staging / SOURCE_PATH.name, staging / LIBRARY_NAME
        staged_source.write_bytes(source)
        exports = staging / "exports.map"
        exports.write_text(EXPORTS, encoding="utf-8")
        compiled = subprocess.run(
            [compiler, "-std=c++17", "-O2", "-fPIC", "-shared", "-fvisibility=hidden", "-fno-gnu-unique",
             "-Wall", "-Wextra", "-Werror", str(staged_source), "-o", str(staged_library), "-ldl", "-pthread",
             f"-Wl,--version-script={exports}"],
            capture_output=True, text=True, timeout=60, check=False,
        )
        if compiled.returncode:
            raise RuntimeError(f"C++ compilation failed: {compiled.stderr.strip()[:4096]}")
        regular_native(root)
        # Isolated Python also prevents user/site Python startup hooks from
        # preloading an SDK. Never load the native core into this builder process.
        environment = {k: v for k, v in os.environ.items() if k not in {"LD_PRELOAD", "LD_AUDIT"}}
        probe = subprocess.run(
            [sys.executable, "-I", "-c", PROBE, str(staged_library), str(native)],
            env=environment, capture_output=True, text=True, timeout=20, check=False,
        )
        if probe.returncode:
            raise RuntimeError(f"Device-free native codec verification failed: {probe.stderr.strip()[:4096]}")
        regular_native(root)
        require_absent(library, manifest_path)
        manifest = {
            "version": 1,
            "core_sha256": CORE_SHA256,
            "source_sha256": source_hash,
            "library_sha256": sha256(staged_library),
        }
        staged_manifest = staging / MANIFEST_NAME
        staged_manifest.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
        published = []
        try:
            # Hard links publish complete files atomically and refuse overwrite,
            # including a conflicting file created after require_absent().
            for staged, target in ((staged_library, library), (staged_manifest, manifest_path)):
                os.link(staged, target)
                published.append((staged, target))
        except OSError:
            for staged, target in reversed(published):
                if target.exists() and os.path.samefile(staged, target):
                    target.unlink()
            raise
    return manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sdk-root", required=True, type=Path, help="explicit path to an SDK copy")
    args = parser.parse_args(argv)
    try:
        manifest = build_sdk_motor_ranges(args.sdk_root)
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        print(str(error), file=sys.stderr)
        return 2
    print(json.dumps(manifest, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
