"""Fixed-base six-axis inverse dynamics, adapted from StarArm feedback 0.6.2.

Coordinates are URDF joint radians/rad/s/rad/s²; torques are joint-output Nm.
SDK-to-URDF transforms must be verified separately before using SDK telemetry.
There is no hardware or Leader output path in this module.
"""

import argparse
import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

ZERO = (0.0, 0.0, 0.0)
I3 = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
JOINTS = tuple(f"joint{i}" for i in range(1, 7))


def vector(values, size, label):
    values = tuple(values)
    if len(values) != size or any(
        isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in values
    ):
        raise ValueError(f"{label}: expected {size} finite numbers")
    return values


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def scale(s, v):
    return tuple(s * x for x in v)


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def mv(r, v):
    return tuple(dot(row, v) for row in r)


def mm(a, b):
    return tuple(tuple(dot(row, col) for col in zip(*b)) for row in a)


def transpose(r):
    return tuple(zip(*r))


def rotation(axis, angle):
    x, y, z = axis
    c, s, h = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    return (
        (c + x * x * h, x * y * h - z * s, x * z * h + y * s),
        (y * x * h + z * s, c + y * y * h, y * z * h - x * s),
        (z * x * h - y * s, z * y * h + x * s, c + z * z * h),
    )


def xyz(text, default=ZERO):
    return vector(default if text is None else tuple(map(float, text.split())), 3, "URDF vector")


def origin(el):
    if el is None:
        return I3, ZERO
    roll, pitch, yaw = xyz(el.get("rpy"))
    r = mm(mm(rotation((0, 0, 1), yaw), rotation((0, 1, 0), pitch)), rotation((1, 0, 0), roll))
    return r, xyz(el.get("xyz"))


def inertia(values, label):
    xx, xy, xz, yy, yz, zz = vector(values, 6, label)
    # Both I and (trace(I)/2)1-I must be positive semidefinite for a physical
    # inertia tensor. Check all principal minors, including degenerate bodies.
    for a, b, c, d, e, f in (
        (xx, xy, xz, yy, yz, zz),
        ((yy + zz - xx) / 2, -xy, -xz, (xx + zz - yy) / 2, -yz, (xx + yy - zz) / 2),
    ):
        tol = 1e-10 * max(abs(a), abs(b), abs(c), abs(d), abs(e), abs(f), 1e-12)
        if min(a, d, f) < -tol:
            raise ValueError(f"{label}: nonphysical inertia")
        magnitude = max(abs(a), abs(b), abs(c), abs(d), abs(e), abs(f), 1e-12)
        if min(a * d - b * b, a * f - c * c, d * f - e * e) < -tol * magnitude:
            raise ValueError(f"{label}: nonphysical inertia")
        if a * d * f + 2 * b * c * e - a * e * e - d * c * c - f * b * b < -tol * magnitude * magnitude:
            raise ValueError(f"{label}: nonphysical inertia")
    return ((xx, xy, xz), (xy, yy, yz), (xz, yz, zz))


class DynamicsModel:
    def __init__(self, path, *, gravity, payload=None):
        self.path = Path(path).expanduser().resolve()
        data = self.path.read_bytes()
        self.sha256 = hashlib.sha256(data).hexdigest()
        self.gravity = vector(gravity, 3, "base-frame gravity m/s²")
        tree = ET.fromstring(data)
        self.bodies, self.children, self.limits = {}, {}, {}
        self.meshes = []
        for link in tree.findall("link"):
            name = link.get("name")
            if not name or name in self.bodies:
                raise ValueError("Missing or duplicate link name")
            mass_el, inertia_el = link.find("inertial/mass"), link.find("inertial/inertia")
            if mass_el is None or inertia_el is None:
                raise ValueError(f"{name}: explicit mass and inertia required")
            mass = float(mass_el.get("value"))
            if not math.isfinite(mass) or mass < 0:
                raise ValueError(f"{name}: invalid mass")
            r, com = origin(link.find("inertial/origin"))
            tensor = inertia(
                tuple(float(inertia_el.get(k)) for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")), name
            )
            self.bodies[name] = (mass, com, mm(mm(r, tensor), transpose(r)))
            for mesh in link.findall(".//mesh"):
                filename = mesh.get("filename", "")
                local = self.path.parent / filename
                self.meshes.append(
                    dict(
                        reference=filename,
                        present=bool(filename) and "://" not in filename and local.is_file(),
                    )
                )
        parents, names, indices = set(), set(), set()
        for joint in tree.findall("joint"):
            name, kind = joint.get("name"), joint.get("type")
            if not name or name in names:
                raise ValueError("Missing or duplicate joint name")
            names.add(name)
            parent = joint.find("parent").get("link")
            child = joint.find("child").get("link")
            if parent not in self.bodies or child not in self.bodies or child in parents:
                raise ValueError("Invalid URDF tree")
            idx = JOINTS.index(name) if name in JOINTS else None
            if joint.find("mimic") is not None:
                raise ValueError("Mimic joints require a separate coordinate adapter")
            if idx is None and kind != "fixed":
                raise ValueError(f"Unsupported moving branch: {name}")
            if idx is not None:
                if kind != "revolute":
                    raise ValueError(f"{name}: bounded revolute joint required")
                indices.add(idx)
                limit = joint.find("limit")
                if limit is None:
                    raise ValueError(f"{name}: joint limits required")
                low, high = vector(tuple(float(limit.get(k)) for k in ("lower", "upper")), 2, name)
                if low >= high:
                    raise ValueError(f"{name}: invalid limits")
                self.limits[name] = (low, high)
            axis = xyz(None if joint.find("axis") is None else joint.find("axis").get("xyz"), (1, 0, 0))
            norm = math.sqrt(dot(axis, axis))
            if idx is not None and norm < 1e-12:
                raise ValueError(f"{name}: zero axis")
            axis = scale(1 / norm, axis) if norm else axis
            self.children.setdefault(parent, []).append((child, idx, origin(joint.find("origin")), axis))
            parents.add(child)
        roots = set(self.bodies) - parents
        if len(roots) != 1 or indices != set(range(6)):
            raise ValueError("Expected one root and joint1..joint6")
        self.root = roots.pop()
        visited = set()

        def visit(link):
            if link in visited:
                raise ValueError("Cyclic URDF")
            visited.add(link)
            for child, *_ in self.children.get(link, []):
                visit(child)

        visit(self.root)
        if visited != set(self.bodies):
            raise ValueError("Disconnected or cyclic URDF")
        self.payload = None
        if payload is not None:
            if not isinstance(payload, dict) or set(payload) != {"link", "mass_kg", "com_m", "inertia_kg_m2"}:
                raise ValueError("Payload requires link, mass_kg, com_m, inertia_kg_m2")
            link = payload["link"]
            if link not in self.bodies:
                raise ValueError("Unknown payload attachment link")
            mass = vector([payload["mass_kg"]], 1, "payload mass")[0]
            if mass <= 0:
                raise ValueError("Payload mass must be positive")
            com = vector(payload["com_m"], 3, "payload COM in attachment frame")
            tensor = inertia(payload["inertia_kg_m2"], "payload inertia at COM, attachment axes")
            # Keep the original link and add the tool as a fixed body. No mass
            # is silently substituted or inferred from an SDK TCP offset.
            tool = "__feedback_tool__"
            if tool in self.bodies:
                raise ValueError("Reserved payload link name already exists")
            self.bodies[tool] = (mass, com, tensor)
            self.children.setdefault(link, []).append((tool, None, (I3, ZERO), (1, 0, 0)))
            self.payload = payload

    def evaluate(self, q, velocity, acceleration):
        q, velocity, acceleration = (
            vector(x, 6, label)
            for x, label in (
                (q, "q rad"),
                (velocity, "velocity rad/s"),
                (acceleration, "acceleration rad/s²"),
            )
        )
        tau, axes, pivots = [0.0] * 6, {}, {}

        def walk(link, r, p, w, alpha, a, ancestors):
            mass, com, local_inertia = self.bodies[link]
            lever = mv(r, com)
            center = add(p, lever)
            ac = add(add(a, cross(alpha, lever)), cross(w, cross(w, lever)))
            force = scale(mass, add(ac, scale(-1, self.gravity)))
            tensor = mm(mm(r, local_inertia), transpose(r))
            moment = add(mv(tensor, alpha), cross(w, mv(tensor, w)))
            for i in ancestors:
                tau[i] += dot(axes[i], add(moment, cross(add(center, scale(-1, pivots[i])), force)))
            for child, idx, (lr, lp), axis in self.children.get(link, []):
                offset = mv(r, lp)
                rr, pp = mm(r, lr), add(p, offset)
                aa = add(add(a, cross(alpha, offset)), cross(w, cross(w, offset)))
                ww, al, branch = w, alpha, ancestors
                if idx is not None:
                    joint_axis = mv(rr, axis)
                    pivots[idx], axes[idx] = pp, joint_axis
                    relative = scale(velocity[idx], joint_axis)
                    ww = add(w, relative)
                    al = add(add(alpha, scale(acceleration[idx], joint_axis)), cross(w, relative))
                    rr = mm(rr, rotation(axis, q[idx]))
                    branch = ancestors + (idx,)
                walk(child, rr, pp, ww, al, aa, branch)

        walk(self.root, I3, ZERO, ZERO, ZERO, ZERO, ())
        return tau

    def energy(self, q, velocity):
        """Independent Jacobian energy calculation for dynamics validation."""
        q, velocity = vector(q, 6, "q"), vector(velocity, 6, "velocity")
        axes, pivots = {}, {}
        kinetic, potential = 0.0, 0.0

        def walk(link, r, p, ancestors):
            nonlocal kinetic, potential
            mass, com, local_inertia = self.bodies[link]
            center = add(p, mv(r, com))
            v, w = ZERO, ZERO
            for i in ancestors:
                omega = scale(velocity[i], axes[i])
                w = add(w, omega)
                v = add(v, cross(omega, add(center, scale(-1, pivots[i]))))
            tensor = mm(mm(r, local_inertia), transpose(r))
            kinetic += 0.5 * (mass * dot(v, v) + dot(w, mv(tensor, w)))
            potential -= mass * dot(center, self.gravity)
            for child, idx, (lr, lp), axis in self.children.get(link, []):
                rr, pp = mm(r, lr), add(p, mv(r, lp))
                branch = ancestors
                if idx is not None:
                    axes[idx], pivots[idx] = mv(rr, axis), pp
                    rr = mm(rr, rotation(axis, q[idx]))
                    branch = ancestors + (idx,)
                walk(child, rr, pp, branch)

        walk(self.root, I3, ZERO, ())
        return kinetic, potential

    def summary(self):
        return dict(
            urdf_sha256=self.sha256,
            root=self.root,
            joint_order=list(JOINTS),
            limits_rad=self.limits,
            total_mass_kg=sum(x[0] for x in self.bodies.values()),
            gravity_base_m_s2=self.gravity,
            added_payload=self.payload,
            meshes=self.meshes,
            hardware_output=False,
            binding_created=False,
            limitations=[
                "SDK coordinates and torque signs not verified",
                "Gripper/tool mass inclusion must be confirmed",
                "No friction or motor/gear inertia identification",
                "No live feedback freshness or hardware validation",
            ],
        )


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument(
        "--gravity", type=float, nargs=3, required=True, help="Gravity expressed in the base frame, m/s²"
    )
    parser.add_argument(
        "--payload", type=Path, help="Explicit additional rigid tool JSON; never infer tool mass"
    )
    parser.add_argument("--q", type=float, nargs=6, help="URDF joint radians; offline model evaluation only")
    parser.add_argument("--velocity", type=float, nargs=6)
    parser.add_argument("--acceleration", type=float, nargs=6)
    args = parser.parse_args(argv)
    supplied = [args.q is not None, args.velocity is not None, args.acceleration is not None]
    if any(supplied) and not all(supplied):
        parser.error("q, velocity and acceleration must be supplied together in URDF joint coordinates")
    try:
        payload = None if args.payload is None else json.loads(args.payload.read_text(encoding="utf-8"))
        model = DynamicsModel(args.urdf, gravity=args.gravity, payload=payload)
        result = model.summary()
        if args.q is not None:
            result["theoretical_unloaded_torque_nm"] = model.evaluate(
                args.q, args.velocity, args.acceleration
            )
            result["pose_within_urdf_limits"] = all(
                lo <= q <= hi for q, (lo, hi) in zip(args.q, (model.limits[n] for n in JOINTS))
            )
    except (OSError, ValueError, TypeError, AttributeError, ET.ParseError) as error:
        parser.error(str(error))
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
