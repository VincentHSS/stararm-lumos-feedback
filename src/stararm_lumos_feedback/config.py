"""Explicit hardware contract and content-addressed local binding. No hardware imports."""

import hashlib
import json
import math
from pathlib import Path

from .gripper import TypeLJ04041254
from .model import JOINTS, DynamicsModel, vector


def number(value, label, low=None, high=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{label}: finite number required (fill the hardware template)")
    if low is not None and value < low or high is not None and value > high:
        raise ValueError(f"{label}: outside [{low}, {high}]")
    return value


def experimental_ceiling(value, profile_data):
    """Validate the optional extra cap independently of ordinary power caps."""
    if value is None:
        if (
            not isinstance(profile_data, dict)
            or profile_data.get("mass_coverage") != "urdf_includes_typelj"
            or "payload" not in profile_data or profile_data["payload"] is not None
        ):
            raise ValueError(
                "Removing the experimental cap requires explicit URDF TypeLJ coverage and payload=null"
            )
        return None
    return number(value, "experimental_damping.max_mw", 1, 200)


def exact(data, keys, label):
    if not isinstance(data, dict) or set(data) != set(keys.split()):
        raise ValueError(f"{label}: expected fields {keys}")


def read_json(path):
    def invalid(value):
        raise ValueError(f"Invalid JSON constant: {value}")

    return json.loads(Path(path).read_text(encoding="utf-8"), parse_constant=invalid)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def canonical(data):
    return json.dumps(data, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


AXIS_KEYS = "feedback_id motor_id p_max v_max t_max sdk_scale sdk_offset urdf_scale urdf_offset torque_scale max_age_s max_temperature_c max_jump max_velocity max_acceleration sdk_tolerance limit_margin"
POLICY_KEYS = "deadzone_nm velocity_margin acceleration_margin full_scale_nm filter_s acceleration_filter_s confirm_s release_s rise_mw_s fall_mw_s hold_error hold_load_nm hold_speed hold_confirm_s hold_progress hold_release_error hold_release_load_nm hold_release_s"
GUARD_KEYS = "can_bitrate can_skew_s leader_query_s leader_gap_s cycle_timeout_s startup_feedback_s"
GRIPPER_KEYS = "type motor_margin_rad max_motor_jump_rad max_motor_velocity_rad_s sdk_motor_tolerance_rad"


class GripperMotorError(RuntimeError):
    """A raw TypeLJ guard failure with JSON-safe diagnostic measurements."""

    interface = None

    def __init__(self, code, message, detail):
        self.code, self.detail = code, detail
        super().__init__(message)


class Profile:
    def __init__(self, path, *, purpose="run"):
        if purpose not in ("inspect", "follow", "monitor", "experiment", "run"):
            raise ValueError("Unknown profile purpose")
        self.purpose = purpose
        self.path = Path(path).expanduser().resolve()
        data = read_json(self.path)
        if not isinstance(data, dict) or data.get("version") not in (2, 3):
            raise ValueError(
                "Feedback profile version 2 required, or version 3 for commissioning: "
                "version 1 assumed a linear TypeLJ opening; "
                "review the motor transform and regenerate the binding"
            )
        keys = "version session_file urdf sdk_config gravity payload mass_coverage evidence axes policy guards gripper_transform"
        if data["version"] == 3:
            keys += " commissioning"
        exact(
            data,
            keys,
            "profile",
        )
        self.data = data
        self.commissioning = data.get("commissioning")
        self.engineering = self.commissioning is not None
        self.experimental_damping = None
        if self.engineering:
            state = self.commissioning
            experimental = state.get("experimental_damping") if isinstance(state, dict) else None
            if experimental is not None:
                experimental_keys = "authorized unloaded_confirmed max_mw native_button_required"
                if isinstance(experimental, dict) and "local_reference_enabled" in experimental:
                    experimental_keys += " local_reference_enabled"
                exact(experimental, experimental_keys, "experimental_damping")
                if (
                    experimental["authorized"] is not True
                    or experimental["unloaded_confirmed"] is not True
                    or experimental["native_button_required"] is not True
                ):
                    raise ValueError("Experimental damping requires explicit authorization and an unloaded declaration")
                experimental_ceiling(experimental["max_mw"], data)
                local_reference = experimental.get("local_reference_enabled", False)
                if type(local_reference) is not bool:
                    raise ValueError("experimental_damping.local_reference_enabled must be a boolean")
                if local_reference and data["mass_coverage"] not in (
                    "urdf_includes_typelj", "explicit_typelj_payload",
                ):
                    raise ValueError("Local residual reference requires confirmed model mass coverage")
                self.experimental_damping = dict(experimental)
            if (
                not isinstance(state, dict)
                or state.get("status") != "engineering_initial"
                or state.get("force_authorized") is not False
                or state.get("mass_coverage_inferred") is not False
                or not isinstance(state.get("allowed_modes"), list)
                or not state["allowed_modes"]
                or any(mode not in ("inspect", "follow", "monitor", "experiment") for mode in state["allowed_modes"])
                or ("experiment" in state["allowed_modes"] and self.experimental_damping is None)
            ):
                raise ValueError("Invalid engineering commissioning contract; force output must remain disabled")
            if purpose == "run" or purpose not in state["allowed_modes"]:
                raise ValueError(
                    "Engineering commissioning profile cannot enable force output. "
                    "Complete model coverage and measured tuning before creating a reviewed force profile"
                )
        if purpose == "experiment" and self.experimental_damping is None:
            raise ValueError("Experimental damping requires its explicit engineering contract")
        self.gripper = TypeLJ04041254
        self.gripper_transform = data["gripper_transform"]
        exact(self.gripper_transform, GRIPPER_KEYS, "gripper_transform")
        if self.gripper_transform["type"] != self.gripper.kind:
            raise ValueError("Unsupported TypeLJ geometry; review the bound SDK converter")
        for key in GRIPPER_KEYS.split()[1:]:
            value = number(self.gripper_transform[key], f"gripper_transform.{key}", 0)
            if key != "motor_margin_rad" and value == 0:
                raise ValueError(f"gripper_transform.{key} must be positive")
        self.paths = {
            key: (self.path.parent / Path(data[key]).expanduser()).resolve()
            for key in ("session_file", "urdf", "sdk_config")
        }
        self.mass_confirmed = data["mass_coverage"] in ("urdf_includes_typelj", "explicit_typelj_payload")
        if not self.mass_confirmed and not (
            self.engineering and purpose != "run" and data["mass_coverage"] == "UNCONFIRMED"
        ):
            raise ValueError("Confirm TypeLJ mass coverage; do not silently omit or double-count it")
        if (data["payload"] is not None) != (data["mass_coverage"] == "explicit_typelj_payload"):
            raise ValueError("Explicit TypeLJ coverage requires its additional payload object")
        if not isinstance(data["evidence"], str) or len(data["evidence"].strip()) < 12:
            raise ValueError("Document motor models, IDs, units, signs and mass source in evidence")
        vector(data["gravity"], 3, "gravity")
        self.model = DynamicsModel(self.paths["urdf"], gravity=data["gravity"], payload=data["payload"])
        self.axes, self.policy, self.guards = data["axes"], data["policy"], data["guards"]
        if len(self.axes) != 7 or len(self.policy) != 7:
            raise ValueError("Exactly six arm axes and one TypeLJ axis required")
        identities = set()
        for i, axis in enumerate(self.axes):
            exact(axis, AXIS_KEYS, f"axis {i + 1}")
            for key in ("feedback_id", "motor_id"):
                value = axis[key]
                if type(value) is not int or not 0 <= value <= (0x7FF if key == "feedback_id" else 15):
                    raise ValueError(f"axis {i + 1}: invalid {key}")
            identity = axis["feedback_id"], axis["motor_id"]
            if identity in identities:
                raise ValueError("Duplicate CAN feedback identity")
            identities.add(identity)
            for key in set(axis) - {"feedback_id", "motor_id"}:
                number(axis[key], f"axis {i + 1}.{key}")
            for key in (
                "p_max",
                "v_max",
                "t_max",
                "max_age_s",
                "max_jump",
                "max_velocity",
                "max_acceleration",
                "sdk_tolerance",
            ):
                if axis[key] <= 0:
                    raise ValueError(f"axis {i + 1}.{key} must be positive")
            number(axis["max_temperature_c"], "temperature", 1, 125)
            number(axis["limit_margin"], "measured position limit margin in SDK coordinates", 0)
            if axis["sdk_scale"] == 0 or axis["urdf_scale"] == 0 or axis["torque_scale"] == 0:
                raise ValueError("Coordinate and torque scales cannot be zero")
            if i < 6 and not math.isclose(
                axis["torque_scale"] * axis["sdk_scale"] * axis["urdf_scale"], 1, rel_tol=1e-6
            ):
                raise ValueError("Arm coordinate/torque transforms must preserve joint power")
            if i == 6 and (axis["urdf_scale"] != 1 or axis["urdf_offset"] != 0):
                raise ValueError("Gripper SDK coordinate is normalized opening; no URDF transform")
            if i == 6 and (axis["sdk_scale"] != 1 or axis["sdk_offset"] != 0):
                raise ValueError(
                    "This reviewed TypeLJ converter requires raw SDK motor radians: "
                    "axis 7 sdk_scale=1 and sdk_offset=0; review a different converter for other coordinates"
                )
        self.gripper_closed_reference_rad = self.gripper.motor_closed_rad
        if self.engineering and "gripper_closed_observation" in self.commissioning:
            observation = self.commissioning["gripper_closed_observation"]
            exact(
                observation, "operator_confirmed_closed raw_position_code",
                "commissioning.gripper_closed_observation",
            )
            if observation["operator_confirmed_closed"] is not True:
                raise ValueError("gripper_closed_observation requires operator_confirmed_closed=true")
            code = observation["raw_position_code"]
            if type(code) is not int or not 0 <= code <= 65535:
                raise ValueError("gripper_closed_observation.raw_position_code must be an integer in 0..65535")
            p_max = self.axes[6]["p_max"]
            reference = (code / 65535 * 2 - 1) * p_max
            position_lsb = 2 * p_max / 65535
            # This is a software limit for a small, explicitly confirmed
            # commissioning observation, not a manufacturer accuracy claim.
            # It changes only the closed-end guard, never either SDK mapping.
            if not 0 < reference <= 16 * position_lsb:
                raise ValueError("gripper_closed_observation must be positive and at most 16 position LSBs")
            self.gripper_closed_reference_rad = reference
        for i, policy in enumerate(self.policy):
            exact(policy, POLICY_KEYS, f"policy {i + 1}")
            for key, value in policy.items():
                number(value, f"policy {i + 1}.{key}", 0)
            for key in (
                "full_scale_nm",
                "filter_s",
                "acceleration_filter_s",
                "confirm_s",
                "release_s",
                "rise_mw_s",
                "fall_mw_s",
                "hold_error",
                "hold_load_nm",
                "hold_confirm_s",
                "hold_release_s",
            ):
                if policy[key] <= 0:
                    raise ValueError(f"policy {i + 1}.{key} must be positive")
            if (
                policy["hold_release_error"] >= policy["hold_error"]
                or policy["hold_release_load_nm"] >= policy["hold_load_nm"]
            ):
                raise ValueError("HOLD release thresholds must be below entry thresholds")
        exact(self.guards, GUARD_KEYS, "guards")
        for key, value in self.guards.items():
            if key == "can_bitrate":
                if value != 1_000_000:
                    raise ValueError("This SDK classic CAN integration requires 1000000 bit/s")
            else:
                number(value, key, 0.001, 10)
        if self.guards["leader_query_s"] >= self.guards["leader_gap_s"]:
            raise ValueError("leader_query_s must be less than leader_gap_s")
        if self.guards["leader_query_s"] >= self.guards["cycle_timeout_s"]:
            raise ValueError("cycle_timeout_s must exceed the bounded Leader query")
        # Session paths use the original launcher's relative-path convention.
        self.session = read_json(self.paths["session_file"])
        fps = self.session.get("fps", 30)
        if type(fps) is not int or not 1 <= fps <= 100:
            raise ValueError("fps must be an integer in 1..100")
        status_rate = self.session.get("status_rate_hz", 5)
        number(status_rate, "status_rate_hz", 1, 20)
        if self.guards["cycle_timeout_s"] <= 1 / fps + self.guards["leader_query_s"]:
            raise ValueError("cycle_timeout_s must exceed the loop period plus Leader query time")
        robot, leader = self.session["robot"], self.session["teleop"]
        if robot.get("type") != "lumos" or leader.get("type") != "stararm102":
            raise ValueError("Expected Lumos / StarArm102 session")
        if robot.get("gripper", True) is not True or leader.get("gripper", True) is not True:
            raise ValueError("This profile requires TypeLJ and Leader gripper")
        for section, keys in (
            (robot, ("mapping_file", "joint_limits_file", "sdk_root")),
            (leader, ("mapping_file",)),
        ):
            for key in keys:
                if not section.get(key):
                    raise ValueError(f"Missing session path: {key}")
                section[key] = str(
                    (self.paths["session_file"].parent / Path(section[key]).expanduser()).resolve()
                )
        if robot["mapping_file"] != leader["mapping_file"]:
            raise ValueError("Both devices must use the same existing calibration")
        expected_sdk_config = Path(robot["sdk_root"]) / "src/config/robot_kinematics.yaml"
        if expected_sdk_config.resolve() != self.paths["sdk_config"]:
            raise ValueError("sdk_config must be the YAML used by this session's SDK")
        import yaml

        sdk_config = yaml.safe_load(self.paths["sdk_config"].read_text(encoding="utf-8"))
        if sdk_config.get("gripper_control", {}).get("type") != "TypeLJ":
            raise ValueError("SDK YAML must explicitly select TypeLJ")
        mounting = sdk_config.get("kinematics", {}).get("mounting", {})
        if mounting.get("mode", "upright") not in ("upright", 0):
            raise ValueError("This hardware profile expects the confirmed upright base")
        if any(abs(a - b) > 0.02 for a, b in zip(data["gravity"], (0, 0, -9.80665))):
            raise ValueError("Feedback gravity must match the confirmed upright installation")
        if "gravity_vector" in mounting:
            configured_gravity = vector(mounting["gravity_vector"], 3, "SDK gravity")
            if any(abs(a - b) > 0.02 for a, b in zip(data["gravity"], configured_gravity)):
                raise ValueError("SDK and feedback base gravity disagree")
        mapping, limits = read_json(robot["mapping_file"]), read_json(robot["joint_limits_file"])
        if mapping.get("format_version") != 1 or mapping.get("gripper_enabled") is not True:
            raise ValueError("Existing paired TypeLJ calibration required")
        if mapping["joint_map"] != leader.get("joint_map", list(range(6))):
            raise ValueError("Leader IDs differ from existing calibration")
        if sorted(mapping["joint_map"]) != list(range(6)):
            raise ValueError("Leader mapping must use IDs 0..5")
        if mapping["leader_port"] != leader.get("port", "/dev/ttyUSB0") or mapping[
            "can_interface"
        ] != robot.get("can_interface", "can0"):
            raise ValueError("Session ports differ from existing calibration")
        low, high = vector(limits["lower_rad"], 6, "lower_rad"), vector(limits["upper_rad"], 6, "upper_rad")
        self.limits = low, high
        for i, (lo, hi) in enumerate(zip(low, high)):
            a = self.axes[i]
            transformed = sorted(
                (lo * a["urdf_scale"] + a["urdf_offset"], hi * a["urdf_scale"] + a["urdf_offset"])
            )
            mlo, mhi = self.model.limits[JOINTS[i]]
            if lo >= hi or transformed[0] < mlo - 1e-6 or transformed[1] > mhi + 1e-6:
                raise ValueError("SDK limits must transform inside the URDF limits")
        self.files = {
            str(p): digest(p.read_bytes())
            for p in [
                self.path,
                *self.paths.values(),
                Path(robot["mapping_file"]),
                Path(robot["joint_limits_file"]),
            ]
        }
        # Bind the actual SDK wrapper/native library, not just a version label.
        sdk = Path(robot["sdk_root"])
        for pattern in (
            "interface_py/startouchclass.py",
            "interface_py/startouch*.so",
            "src/libstartouch.so",
        ):
            matches = sorted(sdk.glob(pattern))
            if not matches:
                raise ValueError(f"Missing SDK runtime resources: {pattern}")
            for p in matches:
                self.files[str(p)] = digest(p.read_bytes())
        # Optional locally built correction is part of the normal SDK binding.
        # It cannot be authorized by changing a version label or a saved hash.
        from .native_compat import ARTIFACTS

        for resource in ARTIFACTS:
            p = sdk / resource
            if p.is_file():
                self.files[str(p)] = digest(p.read_bytes())
        for directory in (sdk / "src/config", *sdk.glob("src/param_csv*")):
            for p in sorted(directory.rglob("*")):
                if p.is_file():
                    self.files[str(p)] = digest(p.read_bytes())

    def binding(self):
        # Includes controller policy/version and all source/resources, so edits require rebinding.
        sources = {str(p.name): digest(p.read_bytes()) for p in sorted(Path(__file__).parent.glob("*.py"))}
        sources.update(
            {
                "web/" + p.relative_to(Path(__file__).parent / "web").as_posix(): digest(p.read_bytes())
                for p in sorted((Path(__file__).parent / "web").rglob("*"))
                if p.is_file()
            }
        )
        sources.update({
            "native/" + p.name: digest(p.read_bytes())
            for p in sorted((Path(__file__).parent / "native").glob("*.cpp"))
        })
        adapters = Path(__file__).parent.parent / "lerobot_robot_lumos_stararm102"
        sources.update({"adapter/" + p.name: digest(p.read_bytes()) for p in sorted(adapters.glob("*.py"))})
        contents = dict(version=2, files=self.files, implementation=sources)
        return dict(**contents, sha256=digest(canonical(contents)))

    def verify(self, path):
        if read_json(path) != self.binding():
            raise ValueError(
                "Binding changed: review model/mapping/policy/SDK/source and regenerate; keep calibration"
            )

    def to_sdk(self, i, raw_position):
        a = self.axes[i]
        position = raw_position * a["sdk_scale"] + a["sdk_offset"]
        return self.gripper.opening(position) if i == 6 else position

    def to_velocity(self, i, raw_position, raw_velocity):
        a = self.axes[i]
        velocity = raw_velocity * a["sdk_scale"]
        if i == 6:
            motor = raw_position * a["sdk_scale"] + a["sdk_offset"]
            return self.gripper.jacobian(motor) * velocity
        return velocity * a["urdf_scale"]

    def check_gripper_motor(self, position, velocity):
        """Check motor radians before the SDK's opening clamps can hide a fault."""
        cfg = self.gripper_transform
        margin = cfg["motor_margin_rad"]
        lower = self.gripper.motor_open_rad - margin
        upper = self.gripper_closed_reference_rad + margin
        if not math.isfinite(position) or not (
            lower <= position <= upper
        ):
            raise GripperMotorError(
                "gripper_motor_range",
                "Axis 7: raw motor position outside TypeLJ physical range: "
                f"actual {position:.12g} rad, allowed [{lower:.12g}, {upper:.12g}] rad",
                dict(
                    axis=7,
                    raw_position_rad=position if math.isfinite(position) else None,
                    raw_position_repr=str(position),
                    allowed_min_rad=lower,
                    allowed_max_rad=upper,
                ),
            )
        if not math.isfinite(velocity) or abs(velocity) > cfg["max_motor_velocity_rad_s"]:
            limit = cfg["max_motor_velocity_rad_s"]
            raise GripperMotorError(
                "gripper_motor_velocity",
                "Axis 7: raw motor velocity outside configured range: "
                f"actual {velocity:.12g} rad/s, allowed [{-limit:.12g}, {limit:.12g}] rad/s",
                dict(
                    axis=7,
                    raw_position_rad=position,
                    raw_velocity_rad_s=velocity if math.isfinite(velocity) else None,
                    raw_velocity_repr=str(velocity),
                    allowed_min_rad_s=-limit,
                    allowed_max_rad_s=limit,
                ),
            )

    def to_model(self, i, sdk_position):
        a = self.axes[i]
        return sdk_position * a["urdf_scale"] + a["urdf_offset"]


# Exact core reviewed in docs/sdk-startup-review.md. Its constructor fixes
# motor types; these ranges drive BOTH native MIT packing and feedback decoding.
# A binding proves file identity, not agreement with the physical motor ranges.
REVIEWED_NATIVE_CORE = "0e6fe64a1983883acc2b3bf9b5092c708c0c3d30dc5219cc5fbc7d8474b9932e"
REVIEWED_NATIVE_RANGES = ((12.5, 8.0, 28.0),) * 3 + ((12.5, 30.0, 10.0),) * 4


def native_motor_range_mismatches(profile):
    """Offline compatibility finding for the exact reviewed core, not hardware detection."""
    native = Path(profile.session["robot"]["sdk_root"]) / "src/libstartouch.so"
    if profile.files.get(str(native)) != REVIEWED_NATIVE_CORE:
        return []  # No range assertion about other core implementations.
    from .native_compat import CORRECTED_RANGES, compatibility_info

    ranges = CORRECTED_RANGES if compatibility_info(profile) else REVIEWED_NATIVE_RANGES
    return _native_range_mismatches(profile, ranges)


def _native_range_mismatches(profile, ranges):
    native = Path(profile.session["robot"]["sdk_root"]) / "src/libstartouch.so"
    if profile.files.get(str(native)) != REVIEWED_NATIVE_CORE:
        return []
    return [
        dict(axis=i + 1, parameter=key, sdk=native_range, hardware=axis[key])
        for i, (axis, axis_ranges) in enumerate(zip(profile.axes, ranges))
        for key, native_range in zip(("p_max", "v_max", "t_max"), axis_ranges)
        if not math.isclose(axis[key], native_range, rel_tol=1e-7, abs_tol=1e-9)
    ]


class SDKMotorRangeError(ValueError):
    code = "sdk_motor_range_mismatch"
    interface = None

    def __init__(self, mismatches):
        self.detail = dict(mismatches=mismatches)
        values = "; ".join(
            f"J{item['axis']} {item['parameter']}: SDK={item['sdk']:g}, hardware={item['hardware']:g}"
            for item in mismatches
        )
        super().__init__(
            "Native SDK motor ranges disagree with reviewed hardware: " + values
            + ". Motion blocked before SDK construction; install a reviewed native range correction. "
            "Read-only inspection remains available."
        )


def verify_sdk_imports(profile):
    """Check installed copies before SingleArm construction can enable motors."""
    from .native_compat import sdk_load_lock

    # Keep the selected range table fixed until native imports complete, even
    # when concurrent callers select different SDK/profile builds.
    with sdk_load_lock:
        _verify_sdk_imports(profile)


def _verify_sdk_imports(profile):
    import importlib
    import sys

    sdk = Path(profile.session["robot"]["sdk_root"])
    resources = (
        ("startouchclass", "interface_py/startouchclass.py"),
        ("startouch", "interface_py/startouch*.so"),
    )
    for _, pattern in resources:
        matches = list(sdk.glob(pattern))
        if not matches or any(digest(p.read_bytes()) != profile.files.get(str(p)) for p in matches):
            raise ValueError("Reviewed SDK build changed; review the build and regenerate the binding")
    mismatches = native_motor_range_mismatches(profile)
    if mismatches:
        raise SDKMotorRangeError(mismatches)
    from .native_compat import CORRECTED_RANGES, load_compatibility

    corrected = load_compatibility(profile)
    # Files may have changed between the offline gate and loader selection.
    # Match the table actually selected in this process in either direction;
    # a missing correction must never silently fall back to the original table.
    mismatches = _native_range_mismatches(
        profile, CORRECTED_RANGES if corrected else REVIEWED_NATIVE_RANGES,
    )
    if mismatches:
        raise SDKMotorRangeError(mismatches)
    # CMake install may rewrite the extension's RUNPATH while leaving its
    # implementation unchanged. Import the exact reviewed build instead of
    # accepting rewritten bytes or asking for a reinstall that repeats it.
    # Never reload an already imported native module in a running process.
    reviewed_path = str(sdk / "interface_py")
    sys.path.insert(0, reviewed_path)
    try:
        for name, pattern in resources:
            module = importlib.import_module(name)
            path = Path(module.__file__).resolve()
            expected = {profile.files.get(str(p)) for p in sdk.glob(pattern)}
            if digest(path.read_bytes()) not in expected:
                raise ValueError(
                    f"Imported {name} differs from the bound SDK build; restart to load the reviewed build"
                )
    finally:
        sys.path.remove(reviewed_path)
    # Native configuration lookup depends on the library's location, so a
    # byte-identical installed copy can still select an unbound SDK config.
    native = sdk / "src/libstartouch.so"
    expected_native = profile.files[str(native)]
    libraries = set()
    for line in Path("/proc/self/maps").read_text().splitlines():
        fields = line.split(maxsplit=5)
        if len(fields) != 6:
            continue
        pathname = fields[5]
        deleted = pathname.endswith(" (deleted)")
        path = Path(pathname.removesuffix(" (deleted)"))
        if path.name != "libstartouch.so":
            continue
        if deleted:
            raise ValueError("Loaded libstartouch.so was deleted; restart with the bound SDK build")
        libraries.add(path.resolve())
    if libraries != {native.resolve()}:
        raise ValueError("Loaded libstartouch.so path differs from the bound SDK; restart with the bound build")
    if digest(native.read_bytes()) != expected_native:
        raise ValueError("Loaded libstartouch.so differs from the bound SDK build")
