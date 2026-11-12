"""Execute recapture controls against the real bilingual browser application."""

import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

import test_dashboard_follow_diagnostics as browser_tests

SCENARIO = r'''
  if(scenario==="manual_recapture"){
    run(`
      state.mode="experiment";state.status="active";state.telemetry_fresh=true;stateReceivedAt=nowMs();
      state.token="test-local-token";
      state.connection={...state.connection,state:"active",mode:"experiment",can_start:false,can_stop:true,
        available_modes:["inspect","follow","experiment"],can_recapture_baseline:true,
        baseline_recapture:{state:"idle",reason:"not_requested"}};
      state.experimental={state:"active",effective_caps_mw:Array(7).fill(0),baseline_frozen:true};
      render();
    `);
    const check=nodes.get("baselineRecaptureUnloaded"),button=nodes.get("recaptureBaseline");
    assert.equal(check.disabled,false);
    assert.equal(button.disabled,true,"Unconfirmed unloaded state could start a capture");
    assert.equal(requests.length,0,"Rendering queued an automatic recapture");
    await run('$("recaptureBaseline").listeners.click()');
    assert.equal(requests.length,0,"Click without explicit unloaded confirmation sent a request");
    for(const mutation of [
      'state.telemetry_fresh=false',
      'pageConnected=false',
      'state.experimental.effective_caps_mw[3]=1',
      'state.experimental.effective_caps_mw[3]="0"',
      'state.experimental.effective_caps_mw.pop()',
      'state.connection.can_recapture_baseline=false',
      'state.connection.state="stopping"',
      'state.connection.mode="follow"',
      'state.connection.baseline_recapture.state="pending"',
      'state.connection.baseline_recapture.state="collecting"'
    ]){
      run('pageConnected=true;state.telemetry_fresh=true;stateReceivedAt=nowMs();state.experimental.effective_caps_mw=Array(7).fill(0);state.connection.state="active";state.connection.mode="experiment";state.connection.can_recapture_baseline=true;state.connection.baseline_recapture.state="idle"');
      check.checked=true;
      run(mutation+';renderBaselineRecapture()');
      assert.equal(button.disabled,true,"Invalid runtime state left recapture enabled: "+mutation);
      assert.equal(check.checked,false,"A stale unloaded declaration survived: "+mutation);
      await run('$("recaptureBaseline").listeners.click()');
      assert.equal(requests.length,0,"Invalid runtime state queued a capture: "+mutation);
    }
    run('state.connection.baseline_recapture.state="idle";renderBaselineRecapture()');
    check.checked=true;
    await run('$("baselineRecaptureUnloaded").listeners.change()');
    assert.equal(button.disabled,false);
    run(`fetch=async(path,options)=>{
      assert.equal(path,"/api/experimental-baseline");
      assert.equal(options.headers["X-Feedback-Token"],"test-local-token");
      assert.equal(options.method,"POST");
      requests.push(JSON.parse(options.body));
      assert.equal($("baselineRecaptureUnloaded").checked,false,"Confirmation was not consumed immediately");
      assert.equal($("recaptureBaseline").disabled,true,"A duplicate request remained available");
      return {ok:true,json:async()=>({...state.connection,can_recapture_baseline:false,
        baseline_recapture:{state:"pending",reason:"recapture_queued"}})};
    }`);
    await run('$("recaptureBaseline").listeners.click()');
    assert.equal(requests.length,1);
    assert.deepEqual(requests[0],{unloaded_confirmed:true},"Recapture attempted to write other settings");
    assert.equal(button.disabled,true);
    assert.equal(check.checked,false);
    assert.ok(nodes.get("baselineRecaptureStatus").textContent.includes("已排队"));
    run('lang="en";translate()');
    assert.ok(nodes.get("baselineRecaptureStatus").textContent.includes("queued"));
    assert.ok(run('copy.en.baselineRecaptureConfirm').includes("unloaded"));
    assert.ok(run('copy.zh.baselineRecaptureConfirm').includes("空载"));
    assert.equal(JSON.stringify(run('Object.keys(copy.zh).sort()')),JSON.stringify(run('Object.keys(copy.en).sort()')));
    run('state.connection.baseline_recapture.state="complete";renderBaselineRecapture()');
    assert.ok(nodes.get("baselineRecaptureStatus").textContent.includes("manually restoring caps"));
    assert.equal(requests.length,1,"Completion automatically recaptured or enabled caps");
    run('state.telemetry_fresh=false;renderBaselineRecapture()');
    assert.ok(nodes.get("baselineRecaptureStatus").textContent.includes("Last report"));
  }else if(scenario==="controls"){
'''


@unittest.skipUnless(shutil.which("node"), "Node is required for dashboard checks")
class ManualBaselineRecaptureUITests(unittest.TestCase):
    def test_explicit_confirmation_freshness_zero_caps_pending_and_bilingual_status(self):
        marker = '  if(scenario==="controls"){'
        self.assertEqual(browser_tests.HARNESS.count(marker), 1)
        harness = browser_tests.HARNESS.replace(marker, SCENARIO)
        web = Path(__file__).resolve().parents[1] / "src/stararm_lumos_feedback/web"
        with tempfile.TemporaryDirectory() as directory:
            script = Path(directory) / "manual-recapture.cjs"
            script.write_text(harness, encoding="utf-8")
            result = subprocess.run(
                [shutil.which("node"), str(script), str(web / "index.html"), str(web / "app.js"), "manual_recapture"],
                text=True, capture_output=True, timeout=30,
            )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
