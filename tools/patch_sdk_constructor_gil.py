"""Patch only the reviewed SDK binding source in an explicitly selected SDK copy.

Usage: python tools/patch_sdk_constructor_gil.py /path/to/copied/startouch_sdk
       python tools/patch_sdk_constructor_gil.py /path/to/copied/startouch_sdk --check

The whole source must exactly match the pinned original or this exact patch.
Git metadata is unnecessary for a source copy. --check succeeds only for the exact
patched source and never writes. This does not rebuild, install, import the SDK,
connect hardware, or update calibration/bindings. Rebuild and normally regenerate
the integration binding before using a patched SDK.
"""

import argparse
import hashlib
import os
import stat
import sys
import tempfile
from pathlib import Path

SDK_COMMIT = "040412542c0b6b6fd2bb6dd652c0de6c666631be"
SOURCE_PATH = Path("src/pybind_0825testfast.cpp")
ORIGINAL_SHA256 = "4ef7e78924aaa2b13eb5cf8dc8c93a9903724b2910f2ccaf6f21f8f318aee9fe"
PATCHED_SHA256 = "87419cf38c806dba61f1baa92cc4e3f87ff1e454f8bba9399df2d4ed9d24f0e2"
ORIGINAL_RETURN = b"            return new ArmController(gripper_exist,can_interface, enable_fd,\n"
PATCHED_RETURN = (
    b"            // stararm-lumos: kwargs are converted before releasing the constructor GIL.\n"
    b"            py::gil_scoped_release release_gil;\n" + ORIGINAL_RETURN
)
MODULE_START = b"PYBIND11_MODULE(startouch, m) {\n"
PATCHED_MODULE_START = MODULE_START + b'    m.attr("_stararm_constructor_gil_release") = 1;\n'


def source_state(source):
    """Accept only the complete, byte-exact reviewed source variants."""
    digest = hashlib.sha256(source).hexdigest()
    if digest == ORIGINAL_SHA256:
        return "original"
    if digest == PATCHED_SHA256:
        return "patched"
    raise ValueError(
        f"Unsupported SDK binding source SHA-256 {digest}; expected pinned SDK "
        f"{SDK_COMMIT} or its exact constructor-GIL patch. No file changed."
    )


def patched_source(source):
    """Release after kwargs casts; RAII reacquires before kwargs destruction."""
    if source_state(source) == "patched":
        return source
    if source.count(ORIGINAL_RETURN) != 1 or source.count(MODULE_START) != 1:
        raise ValueError("Pinned constructor/module anchors are not unique. No file changed.")
    result = source.replace(ORIGINAL_RETURN, PATCHED_RETURN).replace(MODULE_START, PATCHED_MODULE_START)
    if source_state(result) != "patched":
        raise ValueError("Patched source does not match the reviewed result. No file changed.")
    return result


def patch_sdk(sdk_copy, *, check=False):
    """Return original/patched/updated; modify only the selected source file."""
    root = Path(sdk_copy).expanduser().resolve(strict=True)
    if not root.is_dir():
        raise ValueError("An explicit SDK copy directory is required. No file changed.")
    target = root / SOURCE_PATH
    if target.is_symlink() or target.resolve(strict=True).parent != root / "src":
        raise ValueError("SDK binding source must be a regular file in the selected SDK. No file changed.")
    if not target.is_file():
        raise ValueError("SDK binding source must be a regular file. No file changed.")
    source = target.read_bytes()
    state = source_state(source)
    if check or state == "patched":
        return state
    updated = patched_source(source)
    fd, temporary = tempfile.mkstemp(prefix=f".{target.name}.", dir=target.parent)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(updated)
            stream.flush()
            os.fsync(stream.fileno())
            os.fchmod(stream.fileno(), stat.S_IMODE(target.stat().st_mode))
        if target.is_symlink() or target.read_bytes() != source:
            raise ValueError("SDK source changed during patching. No file replaced.")
        os.replace(temporary, target)
    finally:
        Path(temporary).unlink(missing_ok=True)
    return "updated"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sdk_copy", type=Path, help="explicit path to the SDK source copy to inspect/patch")
    parser.add_argument("--check", action="store_true", help="verify the exact patch without modifying files")
    args = parser.parse_args(argv)
    try:
        state = patch_sdk(args.sdk_copy, check=args.check)
    except (OSError, ValueError) as error:
        print(str(error), file=sys.stderr)
        return 2
    if state == "original":
        print("Pinned original source; constructor GIL patch is not applied. No file changed.")
        return 1
    print(f"{'Patched' if state == 'updated' else 'Verified exact patch'}: {args.sdk_copy / SOURCE_PATH}")
    print(f"Source SHA-256: {PATCHED_SHA256}")
    if state == "updated":
        print("Source only: rebuild the SDK copy and regenerate the integration binding before use.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
