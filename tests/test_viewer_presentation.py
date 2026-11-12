"""Measured display interpolation must not predict motion or hide stale data."""

import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


@unittest.skipUnless(shutil.which("node"), "Node is required for the display timeline check")
class ViewerPresentationTests(unittest.TestCase):
    def test_timeline_and_actual_frame_loop_use_only_received_poses(self):
        web = Path(__file__).resolve().parents[1] / "src/stararm_lumos_feedback/web"
        with tempfile.TemporaryDirectory() as directory:
            script = Path(directory) / "presentation.mjs"
            script.write_text(r"""
import assert from 'node:assert/strict';
import {pathToFileURL} from 'node:url';
let pending;
globalThis.requestAnimationFrame = callback => { pending = callback; };
const frame = time => { const callback = pending; pending = null; callback?.(time); };
globalThis.window = {
  devicePixelRatio: 1,
  location: {href: 'http://127.0.0.1:8766/', origin: 'http://127.0.0.1:8766'},
  dispatchEvent() {},
};
globalThis.CustomEvent = class { constructor(type, options) { this.type=type; this.detail=options.detail; } };
const {LumosViewer, PoseTimeline} = await import(pathToFileURL(process.argv[2]));
const zero = [0, 0, 0, 0, 0, 0], one = [1, -1, 2, -2, .4, -.4];
const timeline = new PoseTimeline();
assert.equal(timeline.push(zero, 0, 1000), true);
assert.equal(timeline.push(one, 1, 1200), true);
assert.equal(timeline.push([99,99,99,99,99,99], 1, 1200), false, 'Repeated redraw is not a new measurement');
assert.deepEqual(timeline.sample(1100).q, zero);
assert.deepEqual(timeline.sample(1300).q, one.map(value => value / 2));
assert.equal(timeline.sample(1300).opening, .5);
assert.deepEqual(timeline.sample(1400).q, one);
assert.deepEqual(timeline.sample(3000).q, one, 'Missing packets must not extrapolate');
assert.equal(timeline.pending(1400), false);
timeline.push(one, 1, 1400);
assert.equal(timeline.pending(1400), false, 'Stationary samples must not keep redrawing an identical STL scene');
const pulse = new PoseTimeline();
pulse.push(zero, 0, 1000); pulse.push(one, 1, 1200); pulse.push(zero, 0, 1400);
assert.equal(pulse.pending(1200), true, 'A received motion pulse must not disappear when its endpoints match');
assert.equal(timeline.push(zero, 0, 3000), true);
assert.equal(timeline.samples.length, 1, 'Do not blend across a reception gap');
assert.deepEqual(timeline.sample(3000).q, zero);
for (let time=3200; time<10000; time+=200) timeline.push(one, 1, time);
assert(timeline.samples.length <= 6, 'History must remain bounded');
const context = {
  clearRect() {}, fillRect() {}, beginPath() {}, closePath() {}, moveTo() {}, lineTo() {},
  fill() {}, stroke() {}, fillText() {}, roundRect() {},
  measureText(value) { return {width:value.length*8}; },
  createLinearGradient() { return {addColorStop() {}}; },
};
const canvas = {
  style:{}, dataset:{}, width:720, height:480, clientWidth:720, clientHeight:480,
  addEventListener() {}, dispatchEvent() {},
  getContext(type) { return type==='2d' ? context : null; },
};
const model = {
  root:'base', preview_q:zero, joint_order:Array.from({length:6}, (_, i)=>`j${i}`),
  links:Array.from({length:7}, (_, i)=>({name:i===0?'base':`l${i}`,visuals:[]})),
  joints:Array.from({length:6}, (_, i)=>({name:`j${i}`, type:'revolute',
    parent:i===0?'base':`l${i}`, child:`l${i+1}`, axis:[0,0,1], origin:{xyz:[0,0,.05]}})),
};
const viewer = new LumosViewer(canvas, model);
await viewer.ready;
let rendered=0;
viewer.renderer={render(){ rendered++; }};
viewer.update(zero, 0, null, {live:true,sampleTime:1000}); frame(1000);
viewer.update(one, 1, null, {live:true,sampleTime:1200}); frame(1200);
frame(1300);
const q = viewer.joints.get('j0').motion.quaternion;
assert(Math.abs(q.z-Math.sin(.25))<1e-12, 'Intermediate animation frame must be between actual samples');
assert.deepEqual(viewer.lastQ, one, 'Presentation smoothing must not replace the latest measured sample');
viewer.update(one, 1, null, {live:true,sampleTime:1200});
assert.equal(viewer.poseTimeline.samples.length, 2, 'Duplicate app render must not restart interpolation');
frame(1400);
assert.equal(pending, null, 'The frame loop must idle after the latest measurement');
viewer.update(zero, 0, null, {live:true,sampleTime:1400}); frame(1400);
viewer.update(zero, 0, null, {live:false,sampleTime:1400});
assert.equal(viewer.poseTimeline.samples.length, 0);
assert.equal(viewer.joints.get('j0').motion.quaternion.z, 0, 'Stale and stopped poses freeze to the exact last measurement immediately');
frame(1450);
assert.equal(pending, null, 'Stale state must stop interpolation');
viewer.update(one, 1); frame(1500);
assert(Math.abs(viewer.joints.get('j0').motion.quaternion.z-Math.sin(.5))<1e-12, 'Legacy and reference updates remain exact');
assert(rendered>=6);
// A limited environment without rAF must draw once rather than recursively
// draining a timeline against a clock that cannot advance inside that stack.
delete globalThis.requestAnimationFrame;
viewer.update(zero, 0, null, {live:true,sampleTime:performance.now()});
viewer.update(one, 1, null, {live:true,sampleTime:performance.now()+200});
assert(rendered>=8);

console.log('MEASURED_DISPLAY_TIMELINE_OK');
""")
            result = subprocess.run(
                [shutil.which("node"), str(script), str(web / "viewer.js")],
                text=True,
                capture_output=True,
                timeout=30,
            )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("MEASURED_DISPLAY_TIMELINE_OK", result.stdout)


if __name__ == "__main__":
    unittest.main()
