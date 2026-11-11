"""Read-only URDF description for the dashboard; display geometry never affects dynamics."""

import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

from .model import I3, JOINTS, ZERO, add, mm, mv, rotation, scale, xyz
from .model import origin as transform
from .startup import alignment_reference

CAD_ROOT = Path(__file__).parent / "web/models/lumos-touch-v3"


def _same_arm_frames(bound, cad):
    """Select the bundled shell only when all six arm frames and axes agree.

    The SDK calls the last link 'link6'; the CAD calls it 'gripper_base'.
    Inertias and limits deliberately do not participate in this visual mapping.
    """
    for index, name in enumerate(JOINTS):
        actual, visual = bound.find(f"joint[@name='{name}']"), cad.find(f"joint[@name='{name}']")
        if actual is None or visual is None or actual.get("type") != visual.get("type"):
            return False
        if actual.find("parent").get("link") != visual.find("parent").get("link"):
            return False
        if index < 5 and actual.find("child").get("link") != visual.find("child").get("link"):
            return False
        actual_r, actual_p = transform(actual.find("origin"))
        visual_r, visual_p = transform(visual.find("origin"))
        actual_values = (*actual_p, *(x for row in actual_r for x in row))
        visual_values = (*visual_p, *(x for row in visual_r for x in row))
        if any(not math.isclose(x, y, abs_tol=1e-9) for x, y in zip(actual_values, visual_values)):
            return False
        axes = []
        for joint in (actual, visual):
            axis = xyz(None if joint.find("axis") is None else joint.find("axis").get("xyz"), (1, 0, 0))
            norm = math.sqrt(sum(x * x for x in axis))
            if norm == 0:
                return False
            axes.append(scale(1 / norm, axis))
        if any(not math.isclose(x, y, abs_tol=1e-9) for x, y in zip(*axes)):
            return False
    return True


def description(profile):
    model = profile.model
    tree = ET.fromstring(model.path.read_bytes())
    cad = ET.fromstring((CAD_ROOT / "model.urdf").read_bytes())
    use_cad = _same_arm_frames(tree, cad)
    source = None
    allowed_meshes = set()
    if use_cad:
        tree = cad
        manifest = json.loads((CAD_ROOT / "SOURCE.json").read_text())
        allowed_meshes = {item["path"] for item in manifest["files"] if item["path"].endswith(".STL")}
        source = dict(
            label="LumosTouch V3",
            repository=manifest["repository"],
            commit=manifest["commit"],
            upstream_path=manifest["upstream_path"],
            hardware_shape_verified=False,
            mass_parameters_used=False,
        )

    def origin(element):
        return dict(
            xyz=list(xyz(None if element is None else element.get("xyz"))),
            rpy=list(xyz(None if element is None else element.get("rpy"))),
        )

    links, missing = [], []
    for link in tree.findall("link"):
        visuals = []
        for visual in link.findall("visual"):
            geometry = visual.find("geometry")
            if geometry is None:
                continue
            mesh = geometry.find("mesh")
            if mesh is not None:
                reference = mesh.get("filename", "")
                if use_cad and reference in allowed_meshes:
                    color = visual.find("material/color")
                    visuals.append(
                        dict(
                            origin=origin(visual.find("origin")),
                            geometry=dict(
                                type="mesh",
                                url=f"/assets/models/lumos-touch-v3/{reference}",
                                scale=list(xyz(mesh.get("scale"), (1, 1, 1))),
                            ),
                            color=list(map(float, color.get("rgba").split()))
                            if color is not None
                            else [0.75, 0.75, 0.75, 1],
                        )
                    )
                else:
                    # Never expose arbitrary local filenames from a bound model.
                    missing.append(link.get("name"))
                continue
            shape = None
            for tag, fields in (
                ("box", ("size",)),
                ("cylinder", ("radius", "length")),
                ("sphere", ("radius",)),
            ):
                element = geometry.find(tag)
                if element is None:
                    continue
                shape = {"type": tag}
                for field in fields:
                    shape[field] = (
                        list(xyz(element.get(field))) if field == "size" else float(element.get(field))
                    )
                break
            if shape is not None:
                visuals.append(dict(origin=origin(visual.find("origin")), geometry=shape))
        links.append(dict(name=link.get("name"), visuals=visuals))
    joints = []
    for joint in tree.findall("joint"):
        joints.append(
            dict(
                name=joint.get("name"),
                type=joint.get("type"),
                parent=joint.find("parent").get("link"),
                child=joint.find("child").get("link"),
                origin=origin(joint.find("origin")),
                axis=list(
                    xyz(None if joint.find("axis") is None else joint.find("axis").get("xyz"), (1, 0, 0))
                ),
            )
        )
    # Use the same validated paired home as the alignment reference. Neither
    # this pose nor the generic fallback is telemetry or a robot target.
    alignment = alignment_reference(profile)
    reference = (0.25, 1.05, -1.65, 0.6, 0.0, 0.0)
    preview_q = [
        max(model.limits[name][0], min(model.limits[name][1], q)) for name, q in zip(JOINTS, reference)
    ]
    preview_source = "urdf_reference"
    if alignment.get("available"):
        preview_q = list(alignment["target_urdf_rad"])
        preview_source = "paired_calibration"
    points, edges, children = {}, [], {}
    positions = dict(zip(JOINTS, preview_q))
    gripper_joints = []
    if use_cad:
        # TypeLJ's normalized opening spans 80 mm, whereas this CAD permits 85 mm.
        # Display actual opening distance without changing the hardware adapter.
        gripper_joints = [
            dict(name="gripper_joint1", closed=0.0, open=-0.04),
            dict(name="gripper_joint2", closed=0.0, open=0.04),
        ]
        positions.update({joint["name"]: joint["open"] * 0.55 for joint in gripper_joints})
    for joint in joints:
        children.setdefault(joint["parent"], []).append(joint)
    root = (set(link["name"] for link in links) - {joint["child"] for joint in joints}).pop()

    def walk(link, r, p):
        points[link] = p
        for joint in children.get(link, []):
            child, axis = joint["child"], joint["axis"]
            el = tree.find(f"joint[@name='{joint['name']}']")
            local_r, local_p = transform(el.find("origin"))
            child_r, child_p = mm(r, local_r), add(p, mv(r, local_p))
            q = positions.get(joint["name"], 0.0)
            norm = math.sqrt(sum(x * x for x in axis))
            axis = scale(1 / norm, axis) if norm else axis
            if joint["type"] in ("revolute", "continuous"):
                child_r = mm(child_r, rotation(axis, q))
            elif joint["type"] == "prismatic":
                child_p = add(child_p, mv(child_r, scale(q, axis)))
            edges.append([link, child])
            walk(child, child_r, child_p)

    walk(root, I3, ZERO)
    return dict(
        root=root,
        name=tree.get("name", "Lumos"),
        links=links,
        joints=joints,
        joint_order=list(JOINTS),
        preview_q=preview_q,
        preview_source=preview_source,
        preview_pose=dict(points=points, edges=edges),
        alignment=alignment,
        sdk_to_urdf=[dict(scale=a["urdf_scale"], offset=a["urdf_offset"]) for a in profile.axes[:6]],
        geometry_source="urdf_meshes" if use_cad else "urdf_schematic" if missing else "urdf_primitives",
        missing_mesh_links=sorted(set(missing)),
        source=source,
        gripper_joints=gripper_joints,
        gripper_label="Gripper" if use_cad else "TypeLJ",
        tip_offset_m=0.17334,
        gripper_width_m=0.08,
        visual_dimensions="Original LumosTouch V3 CAD meshes"
        if use_cad
        else "URDF joint locations; schematic housing and TypeLJ shapes",
        display_only=True,
    )
