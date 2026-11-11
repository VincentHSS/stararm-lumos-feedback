"""Bound, core-specific native codec correction; importing never loads a library.

The optional compatibility library supplies the exported motor-range table before
the reviewed native core is loaded. Its verifier exercises actual native codecs
without constructing an arm or opening hardware. This is a local compatibility
layer, not a vendor SDK release or a hardware validation result.
"""

import ctypes
import hashlib
import json
import os
import threading
from pathlib import Path

CORE_SHA256 = "0e6fe64a1983883acc2b3bf9b5092c708c0c3d30dc5219cc5fbc7d8474b9932e"
NATIVE_RANGES = ((12.5, 8.0, 28.0),) * 3 + ((12.5, 30.0, 10.0),) * 4
CORRECTED_RANGES = ((12.5, 10.0, 28.0),) * 3 + ((12.5, 30.0, 10.0),) * 4
SOURCE = Path(__file__).parent / "native/sdk_04041254_ranges.cpp"
ARTIFACTS = ("interface_py/stararm_motor_ranges.json", "interface_py/stararm_motor_ranges.so")
# Selection and native import form one process-wide operation. The outer SDK
# verifier holds this reentrant lock through import; the loader also uses it
# when called directly. Otherwise an original-table caller could lose its
# selection to a corrected-table caller before importing the native core.
sdk_load_lock = threading.RLock()
_active = None


class SDKCompatibilityError(ValueError):
    code = "sdk_compatibility_failed"
    interface = None


def _sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def compatibility_info(profile):
    """Return only a complete, source-matched and bound compatibility build.

    Missing, partial or altered builds cannot clear the original range gate.
    This function is offline; codec verification happens before SDK imports.
    """
    sdk = Path(profile.session["robot"]["sdk_root"])
    native = sdk / "src/libstartouch.so"
    if profile.files.get(str(native)) != CORE_SHA256:
        return None
    manifest, library = (sdk / item for item in ARTIFACTS)
    try:
        if any(not p.is_file() or p.is_symlink() for p in (manifest, library, SOURCE)):
            return None
        if any(_sha(p) != profile.files.get(str(p)) for p in (manifest, library)):
            return None
        data = json.loads(manifest.read_text(encoding="utf-8"))
        expected = dict(
            version=1, core_sha256=CORE_SHA256, source_sha256=_sha(SOURCE),
            library_sha256=_sha(library),
        )
        if data != expected or type(data["version"]) is not int:
            return None
    except (OSError, ValueError, TypeError, KeyError):
        return None
    return dict(
        kind="sdk_04041254_motor_ranges_v1", library=str(library),
        library_sha256=data["library_sha256"], native=str(native),
    )


def _mapped_libraries(name):
    result = set()
    for line in Path("/proc/self/maps").read_text().splitlines():
        fields = line.split(maxsplit=5)
        if len(fields) != 6:
            continue
        pathname = fields[5]
        deleted = pathname.endswith(" (deleted)")
        path = Path(pathname.removesuffix(" (deleted)"))
        if path.name == name:
            if deleted:
                raise SDKCompatibilityError(f"Loaded {name} was deleted; restart with the bound SDK")
            result.add(path.resolve())
    return result


def load_compatibility(profile):
    """Load and verify the source correction before any native arm constructor."""
    global _active
    info = compatibility_info(profile)
    if info is None:
        # A correction selected by the dynamic loader cannot be unloaded or
        # silently reverted by changing files or selecting an old profile.
        with sdk_load_lock:
            if _active is not None or _mapped_libraries("stararm_motor_ranges.so"):
                raise SDKCompatibilityError(
                    "Active SDK correction no longer matches this binding; restart before changing SDK builds"
                )
        return False
    native, library = Path(info["native"]), Path(info["library"])
    # Check the actual core before dlopen, not only binding metadata.
    if _sha(native) != CORE_SHA256:
        raise SDKCompatibilityError("Native core changed; review and regenerate the binding")
    key = (native.resolve(), library.resolve(), info["library_sha256"])
    with sdk_load_lock:
        native_maps = _mapped_libraries("libstartouch.so")
        compat_maps = _mapped_libraries("stararm_motor_ranges.so")
        if _active is not None:
            if _active[0] != key or native_maps != {key[0]} or compat_maps != {key[1]}:
                raise SDKCompatibilityError("Loaded SDK compatibility build changed; restart the process")
            return True
        # GNU unique symbols can be selected at first load only. Never attempt
        # to correct an already running native controller by changing its table.
        if native_maps or compat_maps:
            raise SDKCompatibilityError(
                "SDK already loaded before its range correction; restart before connecting hardware"
            )
        try:
            loaded = ctypes.CDLL(str(library), mode=os.RTLD_NOW | os.RTLD_GLOBAL)
            loaded.stararm_ranges_verify.argtypes = [ctypes.c_char_p]
            loaded.stararm_ranges_verify.restype = ctypes.c_int
            loaded.stararm_ranges_error.argtypes = []
            loaded.stararm_ranges_error.restype = ctypes.c_char_p
            if loaded.stararm_ranges_verify(os.fsencode(native)) != 0:
                detail = loaded.stararm_ranges_error()
                reason = detail.decode("utf-8", errors="replace") if detail else "unknown codec error"
                raise SDKCompatibilityError(f"Native codec correction verification failed: {reason}")
        except (OSError, AttributeError) as exc:
            raise SDKCompatibilityError(f"Cannot load the bound SDK range correction: {exc}") from exc
        if _mapped_libraries("libstartouch.so") != {key[0]} or _mapped_libraries(
            "stararm_motor_ranges.so"
        ) != {key[1]}:
            raise SDKCompatibilityError("Native codec verification loaded an unexpected library path")
        # Keep the handle alive for every subsequent Session in this process.
        _active = (key, loaded)
        return True
