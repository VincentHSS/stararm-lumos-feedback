"""Offline physics checks; no LeRobot or device SDK is imported."""

import io
import json
import math
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from contextlib import redirect_stdout
from pathlib import Path

from stararm_lumos_feedback.model import JOINTS, DynamicsModel, inertia, main


def fixture():
    tree = ET.Element("robot", name="coupled-six-axis")
    for i in range(7):
        link = ET.SubElement(tree, "link", name=f"link{i}")
        body = ET.SubElement(link, "inertial")
        ET.SubElement(body, "mass", value=str(0.2 + i * 0.1))
        ET.SubElement(body, "origin", xyz="0.09 0.02 -0.03", rpy="0.2 -0.3 0.1")
        ET.SubElement(
            body, "inertia", ixx="0.003", iyy="0.004", izz="0.005", ixy="0.0001", ixz="-0.0002", iyz="0.0003"
        )
        if i:
            joint = ET.SubElement(tree, "joint", name=f"joint{i}", type="revolute")
            ET.SubElement(joint, "parent", link=f"link{i - 1}")
            ET.SubElement(joint, "child", link=f"link{i}")
            ET.SubElement(joint, "origin", xyz="0.12 -0.02 0.07", rpy="0.1 0.2 -0.3")
            ET.SubElement(joint, "axis", xyz=("0 0 2", "0 3 0", "4 0 0")[i % 3])
            ET.SubElement(joint, "limit", lower="-2", upper="2", effort="10", velocity="3")
    return tree


class ModelTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.path = Path(temporary.name) / "model.urdf"
        self.tree = fixture()
        self.write()
        self.q = [0.2, 0.4, -0.5, 0.7, -0.3, 0.1]
        self.v = [0.3, -0.2, 0.1, 0.5, -0.4, 0.2]
        self.a = [-0.1, 0.2, 0.3, -0.2, 0.4, 0.1]
        self.zero = [0.0] * 6

    def write(self):
        ET.ElementTree(self.tree).write(self.path, encoding="utf-8")

    def model(self, gravity=(0, 0, -9.80665), payload=None):
        return DynamicsModel(self.path, gravity=gravity, payload=payload)

    def test_static_torques_equal_potential_gradient_for_multiple_mounts(self):
        for gravity in ((0, 0, -9.80665), (0, 0, 9.80665), (-9.80665, 0, 0)):
            model = self.model(gravity)
            torque = model.evaluate(self.q, self.zero, self.zero)
            for i in range(6):
                plus, minus = self.q.copy(), self.q.copy()
                plus[i] += 1e-6
                minus[i] -= 1e-6
                expected = (model.energy(plus, self.zero)[1] - model.energy(minus, self.zero)[1]) / 2e-6
                self.assertAlmostEqual(torque[i], expected, places=7)

    def test_mass_matrix_is_coupled_symmetric_and_agrees_with_energy(self):
        model = self.model((0, 0, 0))
        columns = []
        for i in range(6):
            acceleration = self.zero.copy()
            acceleration[i] = 1
            columns.append(model.evaluate(self.q, self.zero, acceleration))
        self.assertGreater(max(abs(columns[i][j]) for i in range(6) for j in range(i)), 0.01)
        for i in range(6):
            for j in range(6):
                self.assertAlmostEqual(columns[i][j], columns[j][i], places=10)
        expected = 0.5 * sum(self.v[i] * columns[j][i] * self.v[j] for i in range(6) for j in range(6))
        self.assertAlmostEqual(model.energy(self.q, self.v)[0], expected, places=10)
        self.assertGreater(expected, 0)

    def test_full_dynamics_obeys_mechanical_energy_balance(self):
        model = self.model()
        h = 1e-6
        plus_q = [q + h * v for q, v in zip(self.q, self.v)]
        minus_q = [q - h * v for q, v in zip(self.q, self.v)]
        plus_v = [v + h * a for v, a in zip(self.v, self.a)]
        minus_v = [v - h * a for v, a in zip(self.v, self.a)]
        expected_power = (sum(model.energy(plus_q, plus_v)) - sum(model.energy(minus_q, minus_v))) / (2 * h)
        power = sum(v * tau for v, tau in zip(self.v, model.evaluate(self.q, self.v, self.a)))
        self.assertAlmostEqual(power, expected_power, places=7)

    def test_inverting_gravity_only_changes_gravity_component(self):
        upright, inverted = self.model(), self.model((0, 0, 9.80665))
        g = upright.evaluate(self.q, self.zero, self.zero)
        moving_up = upright.evaluate(self.q, self.v, self.a)
        moving_down = inverted.evaluate(self.q, self.v, self.a)
        for a, b, c in zip(moving_up, moving_down, g):
            self.assertAlmostEqual(a - b, 2 * c, places=10)

    def test_explicit_tool_changes_upstream_gravity_and_keeps_original_body(self):
        tool = dict(
            link="link6", mass_kg=0.7, com_m=[0.1, 0.03, -0.02], inertia_kg_m2=[0.001, 0, 0, 0.001, 0, 0.001]
        )
        base, model = self.model(), self.model(payload=tool)
        self.assertEqual(model.bodies["link6"], base.bodies["link6"])
        self.assertAlmostEqual(model.summary()["total_mass_kg"] - base.summary()["total_mass_kg"], 0.7)
        self.assertNotEqual(
            base.evaluate(self.q, self.zero, self.zero), model.evaluate(self.q, self.zero, self.zero)
        )
        h = 1e-6
        for i in range(6):
            plus, minus = self.q.copy(), self.q.copy()
            plus[i] += h
            minus[i] -= h
            gradient = (model.energy(plus, self.zero)[1] - model.energy(minus, self.zero)[1]) / (2 * h)
            self.assertAlmostEqual(model.evaluate(self.q, self.zero, self.zero)[i], gradient, places=7)

    def test_invalid_inertia_including_triangle_inequality_is_rejected(self):
        for tensor in (
            (-1, 0, 0, 1, 0, 1),
            (1, 2, 0, 1, 0, 1),
            (1, 0, 0, 1, 0, 3),
            (float("nan"), 0, 0, 1, 0, 1),
        ):
            with self.subTest(tensor=tensor), self.assertRaises(ValueError):
                inertia(tensor, "test")

    def test_missing_inertial_data_is_not_silently_massless(self):
        self.tree.find("link").remove(self.tree.find("link/inertial"))
        self.write()
        with self.assertRaisesRegex(ValueError, "explicit mass"):
            self.model()

    def test_unknown_moving_branch_is_rejected(self):
        self.tree.find("joint").set("name", "unmapped")
        self.write()
        with self.assertRaisesRegex(ValueError, "moving branch"):
            self.model()

    def test_disconnected_cycle_is_rejected(self):
        for name in ("cycle1", "cycle2"):
            body = ET.SubElement(ET.SubElement(self.tree, "link", name=name), "inertial")
            ET.SubElement(body, "mass", value="0")
            ET.SubElement(body, "inertia", ixx="0", ixy="0", ixz="0", iyy="0", iyz="0", izz="0")
        for name, parent, child in (("a", "cycle1", "cycle2"), ("b", "cycle2", "cycle1")):
            joint = ET.SubElement(self.tree, "joint", name=name, type="fixed")
            ET.SubElement(joint, "parent", link=parent)
            ET.SubElement(joint, "child", link=child)
        self.write()
        with self.assertRaisesRegex(ValueError, "Disconnected"):
            self.model()

    def test_mimic_is_rejected(self):
        ET.SubElement(self.tree.find("joint"), "mimic", joint="joint2")
        self.write()
        with self.assertRaisesRegex(ValueError, "Mimic"):
            self.model()

    def test_missing_mesh_is_reported_without_fabricated_geometry(self):
        geometry = ET.SubElement(ET.SubElement(self.tree.find("link"), "visual"), "geometry")
        ET.SubElement(geometry, "mesh", filename="meshes/missing.STL")
        self.write()
        summary = self.model().summary()
        self.assertEqual(summary["meshes"], [dict(reference="meshes/missing.STL", present=False)])
        self.assertFalse(summary["hardware_output"] or summary["binding_created"])
        self.assertEqual(summary["joint_order"], list(JOINTS))

    def test_nonfinite_or_wrong_sized_states_are_rejected(self):
        for bad in ([0] * 5, [True] * 6, [math.inf] * 6, [math.nan] * 6):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                self.model().evaluate(bad, self.zero, self.zero)

    def test_tool_requires_full_explicit_inertial_data(self):
        with self.assertRaisesRegex(ValueError, "Payload requires"):
            self.model(payload=dict(link="link6", mass_kg=0.5, com_m=[0, 0, 0]))

    def test_cli_is_offline_and_requires_complete_motion_state(self):
        output = io.StringIO()
        args = ["--urdf", str(self.path), "--gravity", "0", "0", "-9.80665"]
        with redirect_stdout(output):
            main(args)
        self.assertFalse(json.loads(output.getvalue())["hardware_output"])
        with self.assertRaises(SystemExit):
            main(args + ["--q"] + ["0"] * 6)

    def test_offline_package_import_never_imports_hardware_plugin_or_sdk(self):
        # A fresh interpreter catches accidental dependency imports masked by
        # modules loaded by other integration tests in this process.
        code = (
            "import sys; from stararm_lumos_feedback.model import DynamicsModel; "
            "assert not {'lerobot', 'serial', 'startouch', 'startouchclass', "
            "'fashionstar_uart_sdk', 'lerobot_robot_lumos_stararm102'} & sys.modules.keys()"
        )
        subprocess.run([sys.executable, "-B", "-c", code], check=True, capture_output=True)


if __name__ == "__main__":
    unittest.main()
