"""Browser-only startup choices must survive retries without enabling hardware."""

import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


@unittest.skipUnless(shutil.which("node"), "Node is required for dashboard checks")
class StartupPreferenceTests(unittest.TestCase):
    def test_preference_lifecycle_and_frozen_request(self):
        web = Path(__file__).resolve().parents[1] / "src/stararm_lumos_feedback/web"
        harness = r'''
const fs=require("fs"),vm=require("vm"),assert=require("assert");
const html=fs.readFileSync(process.argv[2],"utf8"), app=fs.readFileSync(process.argv[3],"utf8");
const key="lumos-startup-pose";
function page(storage=new Map(),denied={}){
  const nodes=new Map(),requests=[];
  class Element{
    constructor(){this.children=[];this.disabled=false;this.textContent="";this.dataset={};this.listeners={};}
    set id(value){this._id=value;nodes.set(value,this);}get id(){return this._id;}
    append(...values){this.children.push(...values);}
    replaceChildren(...values){this.children=values;}
    setAttribute(key,value){this[key]=value;}
    addEventListener(type,listener){this.listeners[type]=listener;}
    querySelectorAll(){return [nodes.get("common"),...Array.from({length:7},(_,i)=>nodes.get("cap"+i))];}
  }
  for(const match of html.matchAll(/id="([^"]+)"/g)){const el=new Element();el.id=match[1];}
  const context=vm.createContext({
    document:{documentElement:{},getElementById:id=>nodes.get(id),createElement:()=>new Element(),querySelectorAll:()=>[]},
    localStorage:{getItem(key){if(denied.read)throw Error("Storage denied");return storage.get(key)??null;},setItem(key,value){if(denied.write)throw Error("Storage denied");storage.set(key,value);}},
    fetch:()=>new Promise(()=>{}),setTimeout(){},console,assert,requests
  });
  const run=source=>vm.runInContext(source,context);
  run(app);
  run(`
    state={status:"stopped",mode:"follow",caps_writable:false,caps:{common_mw:1200,axis_mw:Array(7).fill(1200)},policy:[]};
    pageConnected=true;
    function stopped(){
      state.connection={state:"stopped",mode:"follow",can_start:true,can_stop:false,startup_pose:"home",startup_pose_options:["home","current_aligned","current_relative","leader_target"],available_modes:["inspect","follow","monitor"]};
      render();
    }
    function choose(value){$("startupPose").value=value;$("startupPose").listeners.change();}
    fetch=async(path,options)=>{
      if(path!=="/api/connection")throw Error("Unexpected request");
      requests.push({payload:JSON.parse(options.body),token:options.headers["X-Feedback-Token"]});
      return {ok:false,json:async()=>({error:"Rejected for test"})};
    };
    stopped();
  `);
  return {run,storage,requests,nodes};
}
async function main(){
  for(const saved of [null,"invalid","HOME",'{"startup_pose":"current_relative"}']){
    const storage=new Map(saved===null?[]:[[key,saved]]),p=page(storage);
    assert.equal(p.run("startupPose"),"home");
    assert.equal(p.run("selectedStartupPose"),"home");
    assert.equal(storage.get(key),saved===null?undefined:saved,"Rendering wrote a preference");
  }
  assert.ok(html.includes('<option value="leader_target" data-i18n="startupLeaderTarget">'),"Direct Leader option is absent");
  const direct=page();
  direct.run('choose("leader_target")');
  assert.equal(direct.requests.length,0,"Selecting direct following started hardware");
  assert.equal(direct.run("startupPose"),"leader_target");
  assert.equal(direct.run("selectedStartupPose"),"leader_target");
  assert.equal(direct.nodes.get("startupPoseNote").textContent,direct.run("copy.zh.startupLeaderTargetNote"));
  assert.equal(direct.nodes.get("alignmentStatus").textContent,direct.run("copy.zh.alignmentLeaderTarget"));
  assert.equal(direct.nodes.get("alignmentControls").hidden,true,"Direct startup displayed an alignment target control");
  assert.equal(direct.nodes.get("alignmentTable").hidden,true,"Direct startup displayed an alignment requirement");
  assert.equal(direct.run("alignmentReference()"),null,"Direct startup fell back to the calibrated-home ghost");
  assert.equal(page(direct.storage).run("startupPose"),"leader_target","Explicit direct choice did not survive refresh");
  await direct.run('requestConnection("start","follow")');
  assert.equal(direct.requests.at(-1).payload.startup_pose,"leader_target");
  assert.equal(direct.run("startupPose"),"leader_target","Rejected direct start erased the choice");
  direct.run('$("language").listeners.click()');
  assert.equal(direct.nodes.get("startupPoseNote").textContent,direct.run("copy.en.startupLeaderTargetNote"));
  assert.ok(direct.nodes.get("startupPoseNote").textContent.includes("automatically closes"));
  assert.ok(direct.nodes.get("startupPoseNote").textContent.includes("Leader force output stays disabled"));
  await direct.run('requestConnection("start","monitor")');
  assert.equal(direct.requests.at(-1).payload.startup_pose,"leader_target");
  const beforeRun=direct.requests.length;
  await direct.run('requestConnection("start","run")');
  assert.equal(direct.requests.length,beforeRun,"Direct choice enabled the unavailable force mode");
  direct.run('state.connection.startup_pose_options=["home","current_aligned","current_relative"];render()');
  assert.equal(direct.run("startupPose"),"home","Unsupported direct strategy remained selected");
  assert.equal(direct.storage.get(key),"leader_target","Unsupported server erased the saved direct choice");
  const restored=page();
  restored.run('$("startupPose").value="current_relative"');
  assert.equal(restored.run("startupPose"),"home","Test dispatched an unintended change event");
  await restored.run('requestConnection("start","follow")');
  assert.equal(restored.requests.at(-1).payload.startup_pose,"current_relative","Start ignored a browser-restored visible choice");
  assert.equal(restored.run("startupPose"),"current_relative","Rejected start erased the restored choice");
  assert.equal(restored.storage.get(key),"current_relative","Restored choice was not saved on explicit start");
  assert.equal(page(restored.storage).run("startupPose"),"current_relative","Restored choice did not survive refresh");
  restored.run('$("startupPose").value="home"');
  await restored.run('requestConnection("start","monitor")');
  assert.equal(JSON.stringify(restored.requests.at(-1).payload),JSON.stringify({action:"start",mode:"monitor",startup_pose:"home"}),"Home selection depended on an omitted backend default");
  assert.equal(restored.storage.get(key),"home","Visible home selection was not preserved");
  const storage=new Map(),p=page(storage);
  p.run('choose("current_relative")');
  assert.equal(storage.get(key),"current_relative");
  assert.equal(p.requests.length,0,"Selecting a startup strategy sent a command");
  const refreshed=page(storage);
  assert.equal(refreshed.run("startupPose"),"current_relative","Refresh lost the explicit choice");
  for(const phase of ["stopped","error"]){
    p.run(`state.connection.state="${phase}";state.connection.startup_pose="home";render();`);
    assert.equal(p.run("startupPose"),"current_relative",phase+" erased selection");
    const attempt=p.nodes.get("attemptedStartupPose");
    assert.equal(attempt.hidden,false,"The actual previous strategy was hidden");
    assert.ok(attempt.textContent.includes("校准零位对齐"),"The actual home attempt was misrepresented");
    assert.ok(attempt.textContent.includes("从当前位置相对起步"),"The next selected strategy was missing");
  }
  p.run(`
    var poseFailure={error:"raw diagnostic",error_code:"startup_pose_mismatch",error_diagnostics:{strategy:"home",axes:[{axis:1,failed_checks:["follower_home"],follower_home_error_rad:0.2,tolerance_rad:0.15}]}};
    assert.ok(errorText(poseFailure).includes("从臂距校准零位 11.46°"));
    assert.ok(!errorText(poseFailure).includes("两端差"),"Home-only failure was shown as pair misalignment");
    var gripperFailure={error:"raw diagnostic",error_code:"startup_pose_mismatch",error_diagnostics:{strategy:"current_relative",reason:"gripper_not_closed",opening:0.2,maximum_opening:0.01}};
    assert.ok(errorText(gripperFailure).includes("夹爪开度 20.00%"));
    assert.ok(errorText(gripperFailure).includes("无需为这个错误调整六轴姿态"));
    var limitFailure={error:"raw diagnostic",error_code:"startup_pose_mismatch",error_diagnostics:{strategy:"home",axes:[{axis:3,failed_checks:["target_limit"],target_rad:0.001*Math.PI/180,follower_rad:0,limits_rad:[-Math.PI,0],tolerance_rad:0.15}]}};
    var limitText=errorText(limitFailure);
    assert.ok(limitText.includes("Leader 映射目标超出限位 (实际 0.001°; 限位 -180° … 0°)"),"Target actual value or precise limit range missing: "+limitText);
    assert.ok(!limitText.includes("允许")&&!limitText.includes("8.59°"),"Pure limit error presented pose tolerance as extra joint range");
    var followerLimit={...limitFailure,error_diagnostics:{strategy:"current_relative",axes:[{axis:3,failed_checks:["follower_limit"],target_rad:0,follower_rad:-Math.PI-0.001,limits_rad:[-Math.PI,0],tolerance_rad:0.15}]}};
    assert.ok(errorText(followerLimit).includes("从臂位置超出限位 (实际 -180.0572958°; 限位 -180° … 0°)"),"Follower actual value missing");
    var mixedFailure={...limitFailure,error_diagnostics:{strategy:"home",axes:[{...limitFailure.error_diagnostics.axes[0],failed_checks:["target_limit","pair"],pair_error_rad:0.2}]}};
    var mixedText=errorText(mixedFailure);
    assert.ok(mixedText.includes("限位 -180° … 0°)")&&mixedText.includes("两端差 11.46° (姿态差允许 8.59°)"),"Mixed failure did not restrict tolerance to pose difference: "+mixedText);
    assert.equal((mixedText.match(/姿态差允许/g)||[]).length,1);
    var overrideFailure={...limitFailure,error_diagnostics:{strategy:"home",axes:[{axis:3,failed_checks:["follower_home","pair"],follower_home_error_rad:0.2,pair_error_rad:0.4,tolerance_rad:0.15,pair_tolerance_rad:20*Math.PI/180}]}};
    var overrideText=errorText(overrideFailure);
    assert.ok(overrideText.includes("从臂距校准零位 11.46° (姿态差允许 8.59°)"),"Pair override replaced the home tolerance");
    assert.ok(overrideText.includes("两端差 22.92° (姿态差允许 20.00°)"),"Pair error omitted its independent tolerance");
    lang="en";
    assert.ok(errorText(poseFailure).includes("Follower from calibrated home 11.46°"));
    assert.ok(errorText(gripperFailure).includes("gripper opening is 20.00%"));
    limitText=errorText(limitFailure);
    assert.ok(limitText.includes("mapped Leader target outside limits (actual 0.001°; limits -180° … 0°)"),"English target actual value missing");
    assert.ok(!limitText.includes("allowed")&&!limitText.includes("8.59°"),"English limit error incorrectly appended pose tolerance");
    assert.ok(errorText(followerLimit).includes("Follower position outside limits (actual -180.0572958°; limits -180° … 0°)"),"English Follower actual value missing");
    mixedText=errorText(mixedFailure);
    assert.ok(mixedText.includes("limits -180° … 0°), pair difference 11.46° (pose difference allowed 8.59°)"),"English mixed failure did not restrict pose tolerance");
    assert.equal((mixedText.match(/pose difference allowed/g)||[]).length,1);
    overrideText=errorText(overrideFailure);
    assert.ok(overrideText.includes("Follower from calibrated home 11.46° (pose difference allowed 8.59°)"),"English home tolerance changed with pair override");
    assert.ok(overrideText.includes("pair difference 22.92° (pose difference allowed 20.00°)"),"English pair override tolerance missing");
    lang="zh";
  `);
  await p.run('requestConnection("start","follow")');
  assert.equal(p.requests.at(-1).payload.startup_pose,"current_relative");
  assert.equal(p.run("startupPose"),"current_relative","Rejected start erased selection");
  p.run('fetch=async()=>{throw Error("Network failure")};');
  await p.run('requestConnection("start","monitor")');
  assert.equal(p.run("startupPose"),"current_relative","Failed request erased selection");
  p.run(`
    let completeRequest;
    fetch=(path,options)=>{
      requests.push({payload:JSON.parse(options.body)});
      return new Promise(resolve=>completeRequest=resolve);
    };
    var pendingRequest=requestConnection("start","follow");
    state.connection={...state.connection,state:"starting",startup_pose:"home",can_start:false,can_stop:true};
    render();
  `);
  assert.equal(p.run("startupPose"),"current_relative","Pending render adopted stale server strategy");
  assert.equal(p.nodes.get("startupPose").disabled,true);
  p.run('completeRequest({ok:false,json:async()=>({error:"Rejected for test"})})');
  await p.run("pendingRequest");
  p.run(`
    stopped();state.token="before-render";
    const originalRenderConnection=renderConnection;
    renderConnection=()=>{originalRenderConnection();startupPose="home";state.token="after-render";};
    fetch=async(path,options)=>{
      requests.push({payload:JSON.parse(options.body),token:options.headers["X-Feedback-Token"]});
      return {ok:false,json:async()=>({error:"Rejected for test"})};
    };
  `);
  await p.run('requestConnection("start","follow")');
  assert.equal(p.requests.at(-1).payload.startup_pose,"current_relative","Render changed the submitted startup strategy");
  assert.equal(p.requests.at(-1).token,"before-render","Render changed the captured request token");
  p.run('renderConnection=originalRenderConnection;stopped();');
  for(const phase of ["starting","active","stopping"]){
    p.run(`state.connection={...state.connection,state:"${phase}",startup_pose:"current_aligned",can_start:false,can_stop:true};render();`);
    assert.equal(p.run("startupPose"),"current_aligned","Running strategy is misrepresented");
    assert.equal(p.nodes.get("startupPose").disabled,true);
    assert.equal(storage.get(key),"current_relative","Server strategy overwrote the user's preference");
  }
  p.run("stopped()");
  assert.equal(p.run("startupPose"),"current_relative","Stopping did not restore explicit preference");
  p.run('state.connection={...state.connection,state:"active",mode:"inspect",can_start:false,can_stop:true};render();');
  assert.equal(p.run("startupPose"),"current_relative","Read-only session overwrote the choice");
  await p.run('requestConnection("stop")');
  assert.equal(JSON.stringify(p.requests.at(-1).payload),JSON.stringify({action:"stop"}));
  p.run("stopped()");
  await p.run('requestConnection("start","inspect")');
  assert.equal(JSON.stringify(p.requests.at(-1).payload),JSON.stringify({action:"start",mode:"inspect"}));
  assert.equal(storage.get(key),"current_relative","Stop or inspection erased the preference");
  p.run('stopped();state.connection.startup_pose_options=["home","current_aligned"];render();');
  assert.equal(p.run("startupPose"),"home","Unsupported strategy remained selected");
  p.run('choose("current_relative")');
  await p.run('requestConnection("start","follow")');
  assert.equal(JSON.stringify(p.requests.at(-1).payload),JSON.stringify({action:"start",mode:"follow",startup_pose:"home"}));
  assert.equal(storage.get(key),"current_relative","Unsupported profile erased saved preference");
  p.run('state.connection.startup_pose_options=["home"];render();');
  assert.equal(p.nodes.get("startupPosePanel").hidden,true);
  p.run('delete state.connection;render();');
  assert.equal(p.run("startupPose"),"home","A preview without a connection retained an engineering strategy");
  assert.equal(storage.get(key),"current_relative","Preview erased the explicit preference");
  p.run('stopped();choose("home")');
  assert.equal(storage.get(key),"home");
  assert.equal(page(storage).run("startupPose"),"home","Explicit home did not survive refresh");
  await p.run('requestConnection("start","follow")');
  assert.equal(JSON.stringify(p.requests.at(-1).payload),JSON.stringify({action:"start",mode:"follow",startup_pose:"home"}));
  p.run('choose("current_aligned")');
  assert.equal(storage.get(key),"current_aligned");
  assert.equal(page(storage).run("startupPose"),"current_aligned","Explicit aligned startup did not survive refresh");
  const denied=page(new Map([[key,"current_relative"]]),{read:true,write:true});
  assert.equal(denied.run("startupPose"),"home","Denied storage did not use safe default");
  denied.run('choose("current_relative");$("language").listeners.click();');
  await denied.run('requestConnection("start","follow")');
  assert.equal(denied.requests.at(-1).payload.startup_pose,"current_relative","Storage failure blocked an explicit session choice");
}
main().catch(error=>{console.error(error);process.exitCode=1;});
'''
        with tempfile.TemporaryDirectory() as directory:
            script = Path(directory) / "startup-preferences.cjs"
            script.write_text(harness, encoding="utf-8")
            result = subprocess.run(
                [shutil.which("node"), str(script), str(web / "index.html"), str(web / "app.js")],
                text=True, capture_output=True, timeout=30,
            )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
