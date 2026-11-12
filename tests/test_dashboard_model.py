"""Offline geometry and the local asset route do not expose hardware files."""

import hashlib
import http.client
import json
import shutil
import struct
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
from urllib.parse import urlsplit

from test_feedback_model import fixture
from test_feedback_runtime import configuration

from stararm_lumos_feedback.config import Profile
from stararm_lumos_feedback.control import Caps
from stararm_lumos_feedback.dashboard import Dashboard
from stararm_lumos_feedback.model import JOINTS, DynamicsModel
from stararm_lumos_feedback.visualization import CAD_ROOT, description


class DashboardModelTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.profile = configuration(Path(temporary.name))

    def test_offline_geometry_uses_bound_urdf_coordinates_without_fake_telemetry(self):
        model = description(self.profile)
        self.assertTrue(model["display_only"])
        self.assertEqual(len(model["joints"]), 6)
        self.assertEqual(len(model["preview_q"]), 6)
        self.assertEqual(len(model["preview_pose"]["points"]), 7)
        self.assertEqual(len(model["preview_pose"]["edges"]), 6)
        self.assertNotIn("q_sdk", model)
        self.assertNotIn("measured_nm", model)
        for name, q in zip(model["joint_order"], model["preview_q"]):
            low, high = self.profile.model.limits[name]
            self.assertTrue(low <= q <= high)
        self.assertNotIn(str(self.profile.path.parent), json.dumps(model))

    def test_validated_paired_home_is_the_default_display_reference(self):
        with patch("stararm_lumos_feedback.visualization.alignment_reference", return_value=dict(available=False)):
            generic = description(self.profile)
        home = [0.0] * 6
        alignment = dict(available=True, source="paired_calibration", target_urdf_rad=home, display_only=True)
        with patch("stararm_lumos_feedback.visualization.alignment_reference", return_value=alignment):
            model = description(self.profile)
        self.assertEqual(model["preview_q"], home)
        self.assertEqual(model["preview_source"], "paired_calibration")
        self.assertEqual(model["alignment"], alignment)
        self.assertNotEqual(model["preview_pose"]["points"], generic["preview_pose"]["points"])
        self.assertNotIn("q_sdk", model)
        self.assertNotIn("telemetry_fresh", model)
        self.assertTrue(model["display_only"])

    def test_missing_calibration_keeps_an_explicit_bounded_generic_reference(self):
        with patch("stararm_lumos_feedback.visualization.alignment_reference", return_value=dict(available=False)):
            model = description(self.profile)
        self.assertEqual(model["preview_source"], "urdf_reference")
        for name, value in zip(model["joint_order"], model["preview_q"]):
            self.assertTrue(self.profile.model.limits[name][0] <= value <= self.profile.model.limits[name][1])

    @unittest.skipUnless(shutil.which("node"), "Node is required for the display selection check")
    def test_actual_app_retains_complete_measured_pose_and_matches_reference_fk(self):
        web = Path(__file__).resolve().parents[1] / "src/stararm_lumos_feedback/web"
        model = self.profile.path.parent / "display-model.json"
        model.write_text(json.dumps(description(self.profile)))
        script = self.profile.path.parent / "display-test.cjs"
        script.write_text(r"""
const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const app=fs.readFileSync(process.argv[2],'utf8');
const sandbox=vm.createContext({});
vm.runInContext(app.slice(app.indexOf('function displayPose('),app.indexOf('function draw()')),sandbox);
const model=JSON.parse(fs.readFileSync(process.argv[3],'utf8'));
model.sdk_to_urdf[0]={scale:-1,offset:.2};
const pose={q_sdk:[.3,0,0,0,0,0,.25],q_urdf:[-.1,0,0,0,0,0],display_only:true};
let selected=sandbox.displayPose(model,{last_pose:pose});
assert.equal(selected.source,'last');assert.deepEqual(Array.from(selected.q),pose.q_urdf);assert.equal(selected.opening,.25);
assert.equal(sandbox.displayPose(model,{q_sdk:[1,2,3],q_urdf:Array(6).fill(9),last_pose:pose}).source,'last');
assert.equal(sandbox.displayPose(model,{q_sdk:Array(7).fill(NaN),last_pose:pose}).source,'last');
assert.equal(sandbox.displayPose(model,{q_sdk:[.3,0,0,0,0,0,.5],coherent_feedback:false,last_pose:pose}).source,'last');
selected=sandbox.displayPose(model,{q_sdk:[.3,0,0,0,0,0,.5]});
assert.equal(selected.source,'current');assert(Math.abs(selected.q[0]+.1)<1e-12);assert.equal(selected.opening,.5);
selected=sandbox.displayPose(model,{q_sdk:[.3,0,0,0,0,0,.5],q_urdf:[.7,0,0,0,0,0]});
assert.equal(selected.q[0],.7,'URDF positions must not be transformed twice');
selected=sandbox.displayPose(model,{q_urdf:Array(6).fill(9)});
assert.equal(selected.source,model.preview_source);assert.deepEqual(Array.from(selected.q),model.preview_q);
model.preview_source='paired_calibration';
assert.equal(sandbox.displayPose(model,{}).source,'paired_calibration');
const schematic=sandbox.schematicPose(model,model.preview_q,.55);
assert.deepEqual(JSON.parse(JSON.stringify(schematic.edges)),model.preview_pose.edges);
for(const [name,point] of Object.entries(model.preview_pose.points)){
  for(let i=0;i<3;i++)assert(Math.abs(schematic.points[name][i]-point[i])<1e-12,'Display FK differs for '+name);
}
const measured=sandbox.schematicPose(model,pose.q_urdf,pose.q_sdk[6]);
assert.notDeepEqual(JSON.parse(JSON.stringify(measured.points)),model.preview_pose.points);
const source={dataset:{}};
Object.assign(sandbox,{
  modelDefinition:model,state:{last_pose:pose},stateReceivedAt:0,lang:'test',
  copy:{test:{livePose:'LIVE',lastPose:'LAST',referencePose:'REFERENCE',calibratedReferencePose:'CALIBRATED',schematic:'SCHEMATIC',schematicNote:'note'}},
  $:()=>source,renderAlignment(){},document:{querySelectorAll:()=>[]},
  telemetryFresh:()=>true,showAlignmentTarget:false,viewerFailed:false,
  solidViewer:{update(q,opening){sandbox.rendered={q:Array.from(q),opening};}},
});
vm.runInContext(app.slice(app.indexOf('function draw()'),app.indexOf('async function loadModel()')),sandbox);
sandbox.draw();
assert(source.textContent.startsWith('LAST'));assert.equal(source.dataset.live,'false');
assert.deepEqual(sandbox.rendered,{q:pose.q_urdf,opening:.25});
sandbox.state={};sandbox.draw();assert(source.textContent.startsWith('CALIBRATED'));assert.equal(source.dataset.live,'false');
sandbox.state={q_sdk:[.3,0,0,0,0,0,.5]};sandbox.draw();assert(source.textContent.startsWith('LIVE'));assert.equal(source.dataset.live,'true');
model.preview_source='urdf_reference';sandbox.state={};sandbox.draw();assert(source.textContent.startsWith('REFERENCE'));
console.log('DISPLAY_POSE_OK');
""")
        result = subprocess.run(
            [shutil.which("node"), str(script), str(web / "app.js"), str(model)],
            text=True, capture_output=True, timeout=10,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("DISPLAY_POSE_OK", result.stdout)

    def test_preview_runs_with_hardware_modules_forbidden(self):
        profile = Profile(self.profile.path, purpose="inspect")
        binding = self.profile.path.parent / "binding.json"
        binding.write_text(json.dumps(profile.binding()))
        script = """
import importlib.abc
import sys
class NoHardware(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        roots = ("can", "serial", "startouch", "stararm_lumos_feedback.runtime",
                 "stararm_lumos_feedback.inspection", "lerobot_robot_lumos_stararm102.lumos",
                 "lerobot_robot_lumos_stararm102.stararm102")
        if any(fullname == root or fullname.startswith(root + ".") for root in roots):
            raise AssertionError("Hardware module imported: " + fullname)
sys.meta_path.insert(0, NoHardware())
from stararm_lumos_feedback.cli import main
main(sys.argv[1:])
"""
        result = subprocess.run(
            [
                sys.executable,
                "-B",
                "-c",
                script,
                "preview",
                "--profile",
                str(profile.path),
                "--binding",
                str(binding),
                "--port",
                "0",
                "--seconds",
                "0.01",
            ],
            text=True,
            capture_output=True,
            timeout=10,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("Offline model preview: http://127.0.0.1:", result.stdout)

    def test_missing_meshes_are_explicit_and_never_mapped_to_arbitrary_local_paths(self):
        path = self.profile.model.path
        tree = ET.parse(path)
        visual = ET.SubElement(tree.getroot().find("link"), "visual")
        geometry = ET.SubElement(visual, "geometry")
        ET.SubElement(geometry, "mesh", filename="/private/calibration.json")
        tree.write(path)
        model = description(self.profile)
        self.assertEqual(model["geometry_source"], "urdf_schematic")
        self.assertEqual(len(model["missing_mesh_links"]), 1)
        self.assertNotIn("/private", json.dumps(model))
        self.assertEqual(model["links"][0]["visuals"], [])

    def test_model_endpoint_and_packaged_assets_keep_local_files_private(self):
        dashboard = Dashboard(Caps(), self.profile, port=0)
        dashboard.start()
        self.addCleanup(dashboard.close)

        def get(path):
            connection = http.client.HTTPConnection("127.0.0.1", dashboard.server.server_port, timeout=2)
            try:
                connection.request("GET", path)
                response = connection.getresponse()
                return response.status, response.read(), response.getheader("Content-Type")
            finally:
                connection.close()

        code, body, _ = get("/api/model")
        self.assertEqual(code, 200)
        self.assertTrue(json.loads(body)["display_only"])
        code, _, mime = get("/assets/viewer.js")
        self.assertEqual(code, 200)
        self.assertIn("text/javascript", mime)
        code, body, mime = get("/assets/icons/play.svg")
        self.assertEqual(code, 200)
        self.assertIn("image/svg+xml", mime)
        self.assertTrue(body.startswith(b"<svg"))
        for path in (
            "/assets/../../config.py",
            "/assets/../config.py",
            "/assets/%2e%2e/config.py",
            "/assets/../configs/commissioning.local.json",
        ):
            code, _, _ = get(path)
            self.assertEqual(code, 404, path)


class RealCADTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)

    def profile(self, mismatch=None):
        tree = fixture()
        cad = ET.parse(CAD_ROOT / "model.urdf").getroot()
        tree.find("link[@name='link0']").set("name", "base_link")
        tree.find("joint[@name='joint1']/parent").set("link", "base_link")
        for name in JOINTS:
            actual = tree.find(f"joint[@name='{name}']")
            visual = cad.find(f"joint[@name='{name}']")
            for tag in ("origin", "axis"):
                actual.find(tag).attrib.clear()
                actual.find(tag).attrib.update(visual.find(tag).attrib)
        if mismatch == "axis":
            tree.find("joint[@name='joint1']/axis").set("xyz", "0 0 -1")
        if mismatch == "origin":
            tree.find("joint[@name='joint1']/origin").set("xyz", "0.001 0 0")
        if mismatch == "rotation":
            tree.find("joint[@name='joint1']/origin").set("rpy", "0.01 0 0")
        path = self.root / f"model-{mismatch}.urdf"
        ET.ElementTree(tree).write(path)
        axes = [dict(urdf_scale=1, urdf_offset=0) for _ in range(7)]
        axes[0] = dict(urdf_scale=-1, urdf_offset=0.2)
        return SimpleNamespace(
            model=DynamicsModel(path, gravity=(0, 0, -9.80665)),
            axes=axes,
            policy=[{}] * 7,
            purpose="inspect",
        )

    def test_matching_frames_select_pinned_local_cad_and_eighty_mm_gripper(self):
        model = description(self.profile())
        visuals = [v for link in model["links"] for v in link["visuals"]]
        self.assertEqual(model["geometry_source"], "urdf_meshes")
        self.assertEqual((len(visuals), len(model["joints"])), (9, 8))
        self.assertEqual((len(model["preview_pose"]["points"]), len(model["preview_pose"]["edges"])), (9, 8))
        for visual in visuals:
            parsed = urlsplit(visual["geometry"]["url"])
            self.assertEqual((parsed.scheme, parsed.netloc, parsed.query, parsed.fragment), ("", "", "", ""))
            self.assertTrue(parsed.path.startswith("/assets/models/lumos-touch-v3/meshes/"))
            self.assertNotIn("..", Path(parsed.path).parts)
        self.assertEqual([j["open"] for j in model["gripper_joints"]], [-0.04, 0.04])
        self.assertEqual(model["source"]["commit"], "445f46f8756d9b826b17687dbdbb0c439537efcd")
        self.assertEqual(model["source"]["repository"], "https://github.com/VincentHSS/Star-Arm-102-Web-ACT")
        self.assertFalse(model["source"]["mass_parameters_used"])
        self.assertFalse(model["source"]["hardware_shape_verified"])
        self.assertNotIn(str(CAD_ROOT), json.dumps(model))

    def test_mismatched_arm_frames_do_not_select_cad(self):
        for mismatch in ("axis", "origin", "rotation"):
            with self.subTest(mismatch=mismatch):
                model = description(self.profile(mismatch))
                self.assertNotEqual(model["geometry_source"], "urdf_meshes")
                self.assertIsNone(model["source"])

    def test_display_preserves_bound_dynamics_limits_and_mapping(self):
        profile = self.profile()
        bound = profile.model
        q, velocity, acceleration = [0.2] * 6, [0.1] * 6, [0.3] * 6
        before = deepcopy(
            (bound.bodies, bound.limits, profile.axes, bound.evaluate(q, velocity, acceleration))
        )
        model = description(profile)
        self.assertIs(profile.model, bound)
        self.assertEqual(
            (bound.bodies, bound.limits, profile.axes, bound.evaluate(q, velocity, acceleration)), before
        )
        self.assertEqual(model["sdk_to_urdf"][0], dict(scale=-1, offset=0.2))
        self.assertTrue(model["display_only"])

    def test_manifest_matches_all_packaged_cad_bytes(self):
        source = json.loads((CAD_ROOT / "SOURCE.json").read_text())
        self.assertEqual(source["commit"], "445f46f8756d9b826b17687dbdbb0c439537efcd")
        self.assertEqual(len(source["files"]), 11)
        self.assertEqual(sum(item["path"].endswith(".STL") for item in source["files"]), 9)
        for item in source["files"]:
            with self.subTest(path=item["path"]):
                path = (CAD_ROOT / item["path"]).resolve()
                self.assertTrue(path.is_relative_to(CAD_ROOT.resolve()))
                data = path.read_bytes()
                self.assertEqual(len(data), item["bytes"])
                self.assertEqual(hashlib.sha256(data).hexdigest(), item["sha256"])
                blob = f"blob {len(data)}\0".encode() + data
                self.assertEqual(hashlib.sha1(blob).hexdigest(), item["git_blob_sha"])
                if path.suffix.lower() == ".stl":
                    triangles = struct.unpack_from("<I", data, 80)[0]
                    self.assertEqual(triangles, item["triangles"])
                    self.assertEqual(len(data), 84 + 50 * triangles)

    def test_real_stl_route_and_model_keep_local_paths_private(self):
        dashboard = Dashboard(Caps(), self.profile(), port=0)
        dashboard.start()
        self.addCleanup(dashboard.close)
        connection = http.client.HTTPConnection("127.0.0.1", dashboard.server.server_port, timeout=2)
        self.addCleanup(connection.close)
        connection.request("GET", "/api/model")
        response = connection.getresponse()
        model = json.loads(response.read())
        self.assertEqual(response.status, 200)
        self.assertEqual(model["geometry_source"], "urdf_meshes")
        self.assertNotIn(str(self.root), json.dumps(model))
        relative = "meshes/gripper_link1.STL"
        connection.request("GET", "/assets/models/lumos-touch-v3/" + relative)
        response = connection.getresponse()
        self.assertEqual(response.status, 200)
        self.assertIn("model/stl", response.getheader("Content-Type"))
        self.assertEqual(response.read(), (CAD_ROOT / relative).read_bytes())
        connection.request("GET", "/assets/models/lumos-touch-v3/../../../../config.py")
        response = connection.getresponse()
        response.read()
        self.assertEqual(response.status, 404)

    @unittest.skipUnless(shutil.which("node"), "Node is required for the actual renderer check")
    def test_target_ghost_reuses_real_stl_with_independent_pose_in_both_render_paths(self):
        model = self.root / "display-model.json"
        model.write_text(json.dumps(description(self.profile())))
        script = self.root / "renderer-test.mjs"
        script.write_text("""
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import {pathToFileURL} from 'node:url';
import path from 'node:path';

let pending = null, fetchCount = 0;
globalThis.requestAnimationFrame = callback => { pending = callback; };
const flush = () => { const callback = pending; pending = null; callback?.(); };
globalThis.window = {
  devicePixelRatio: 1,
  location: {href: 'http://127.0.0.1:8765/', origin: 'http://127.0.0.1:8765'},
  dispatchEvent() {},
};
globalThis.CustomEvent = class { constructor(type, options) { this.type = type; this.detail = options.detail; } };
const context = {
  clearRect() {}, fillRect() {}, beginPath() {}, closePath() {}, moveTo() {}, lineTo() {},
  fill() {}, stroke() {}, fillText() {}, roundRect() {},
  measureText(value) { return {width: value.length * 8}; },
  createLinearGradient() { return {addColorStop() {}}; },
};
const canvas = {
  style: {}, dataset: {}, width: 720, height: 480, clientWidth: 720, clientHeight: 480,
  addEventListener() {}, dispatchEvent() {},
  getContext(type) { return type === '2d' ? context : null; },
};
globalThis.fetch = async url => {
  fetchCount++;
  const prefix = '/assets/models/lumos-touch-v3/';
  assert(url.pathname.startsWith(prefix));
  const bytes = await fs.readFile(path.join(process.argv[3], url.pathname.slice(prefix.length)));
  return {ok: true, async arrayBuffer() {return bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength);}};
};
const {LumosViewer} = await import(pathToFileURL(process.argv[2]));
const model = JSON.parse(await fs.readFile(process.argv[4], 'utf8'));
const viewer = new LumosViewer(canvas, model);
await viewer.ready;
assert.equal(viewer.meshLoaded, 9);
assert.equal(viewer.meshFailed, 0);
assert.equal(viewer.targetRobot, undefined);
assert.equal(viewer.available, false);
const current = [0.13, -0.8, 1.2, -0.4, 0.2, -0.1];
const target = [-0.4, 0.5, -1, 0.3, -0.2, 0.25];
viewer.update(current, 0.25);
flush();
const labels = viewer.cpuLabels.length;
const currentMatrices = [...viewer.joints].map(([name, joint]) => [name, [...joint.motion.matrixWorld.elements]]);
const baseFaces = viewer.cpuTriangles(720, 480);
assert(baseFaces.length > 1000);
const originalGeometries = new Set();
viewer.robot.traverse(object => { if (object.isMesh) originalGeometries.add(object.geometry); });
viewer.update(current, 0.25, target);
flush();
assert.equal(viewer.targetRobot.visible, true);
assert.deepEqual(viewer.lastTargetQ, target);
assert.equal(viewer.cpuLabels.length, labels);
let targetMeshes = 0;
viewer.targetRobot.traverse(object => {
  assert.equal(Boolean(object.isSprite), false);
  if (!object.isMesh) return;
  targetMeshes++;
  assert(originalGeometries.has(object.geometry));
  assert.equal(object.material.transparent, true);
  assert.equal(object.material.opacity, 0.32);
  assert.equal(object.material.depthWrite, false);
  assert.equal(object.material.color.getHex(), 0x38d8ee);
});
assert.equal(targetMeshes, 9);
for (const [name, elements] of currentMatrices) {
  assert.deepEqual([...viewer.joints.get(name).motion.matrixWorld.elements], elements);
}
viewer.order.forEach((name, index) => {
  const joint = viewer.targetJoints.get(name);
  const actual = joint.motion.quaternion;
  const sine = Math.sin(target[index] / 2);
  assert(Math.abs(actual.x - joint.axis.x * sine) < 1e-12);
  assert(Math.abs(actual.y - joint.axis.y * sine) < 1e-12);
  assert(Math.abs(actual.z - joint.axis.z * sine) < 1e-12);
  assert(Math.abs(actual.w - Math.cos(target[index] / 2)) < 1e-12);
});
for (const mapping of model.gripper_joints) {
  const position = mapping.closed + (mapping.open - mapping.closed) * 0.25;
  const actual = viewer.targetJoints.get(mapping.name);
  assert(Math.abs(actual.motion.position.dot(actual.axis) - position) < 1e-12);
}
const bothFaces = viewer.cpuTriangles(720, 480);
assert(bothFaces.length > baseFaces.length);
assert(bothFaces.some(face => face.alpha === 0.32));
assert(bothFaces.some(face => face.alpha === 1));
assert.equal(new Set(bothFaces).size, bothFaces.length, 'Shared geometry must not overwrite current faces');
viewer.update(current, 0.25, null);
flush();
assert.equal(viewer.targetRobot.visible, false);
assert.equal(viewer.cpuTriangles(720, 480).length, baseFaces.length);
// WebGL receives the same current and independently posed target scene tree.
let gpuCalls = 0;
viewer.renderer = {render(scene, camera) {
  gpuCalls++;
  assert(scene.children.includes(viewer.robot));
  assert(scene.children.includes(viewer.targetRobot));
  assert.equal(viewer.targetRobot.visible, true);
  assert.equal(camera, viewer.camera);
}};
viewer.update(current, 0.8, target);
flush();
assert.equal(gpuCalls, 1);
viewer.renderer = null;
viewer.update(current, 0.8);
flush();
assert.equal(viewer.targetRobot.visible, false);
assert.equal(fetchCount, 9, 'Target must reuse the loaded meshes');
viewer.reset();
assert(Number.isFinite(viewer.distance));
assert(viewer.distance > 0);
console.log('ACTUAL_STL_GHOST_OK');
""")
        web = Path(__file__).resolve().parents[1] / "src/stararm_lumos_feedback/web"
        result = subprocess.run(
            [shutil.which("node"), str(script), str(web / "viewer.js"), str(CAD_ROOT), str(model)],
            text=True,
            capture_output=True,
            timeout=60,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("ACTUAL_STL_GHOST_OK", result.stdout)


if __name__ == "__main__":
    unittest.main()
