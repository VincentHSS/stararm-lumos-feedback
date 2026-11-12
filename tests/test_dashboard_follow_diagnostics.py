"""Exercise the real browser controls and tracking renderer without hardware."""

import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

HARNESS = r'''
const fs=require("fs"),vm=require("vm"),assert=require("assert");
const html=fs.readFileSync(process.argv[2],"utf8"),app=fs.readFileSync(process.argv[3],"utf8");
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
  localStorage:{getItem(){return null;},setItem(){}},fetch:()=>new Promise(()=>{}),setTimeout(){},console,assert,requests
});
const run=source=>vm.runInContext(source,context);
run(app);
run(`
  state={status:"stopped",mode:"follow",caps_writable:false,caps:{common_mw:1200,axis_mw:Array(7).fill(1200)},policy:[],
    profile_status:{engineering:true,mass_confirmed:false,force_ready:false},
    connection:{state:"stopped",mode:"follow",can_start:true,can_stop:false,startup_pose:"home",startup_pose_options:["home","current_aligned","current_relative","leader_target"],available_modes:["inspect","follow","monitor"],blocked_modes:{run:"force_profile_unreviewed"}}};
  pageConnected=true;
  fetch=async(path,options)=>{
    assert.equal(path,"/api/connection");requests.push(JSON.parse(options.body));
    return {ok:false,json:async()=>({error:"Synthetic rejection"})};
  };
  render();
`);
const text=(id,row,column)=>nodes.get(id).children[row].children[column].textContent;
async function main(){
  const scenario=process.argv[4];
  if(scenario==="controls"){
    assert.equal(nodes.get("startFeedback").disabled,true);
    await run('$("startFeedback").listeners.click()');
    assert.equal(requests.length,0,"Unavailable feedback sent a request");
    run('state.connection.available_modes.push("experiment");render()');
    assert.equal(nodes.get("startFeedback").textContent,"启动实验阻尼");
    assert.equal(nodes.get("startFeedback").disabled,false);
    assert.equal(nodes.get("experimentalPanel").hidden,false);
    assert.ok(nodes.get("feedbackBlocked").textContent.includes("正式力反馈尚未就绪"));
    run('$("startupPose").value="leader_target"');
    await run('$("startFeedback").listeners.click()');
    assert.deepEqual(requests.at(-1),{action:"start",mode:"experiment",startup_pose:"leader_target"});
    assert.equal(run('state.profile_status.force_ready'),false);
    run('lang="en";translate()');
    assert.equal(nodes.get("startFeedback").textContent,"Start experimental damping");
    assert.ok(nodes.get("feedbackBlocked").textContent.includes("Formal force feedback is not ready"));
    run('state.connection.blocked_modes.experiment="sdk_motor_range_mismatch";render()');
    assert.equal(nodes.get("startFeedback").disabled,true);
    const count=requests.length;
    await run('$("startFeedback").listeners.click()');
    assert.equal(requests.length,count,"Blocked experimental mode sent a request");
    run('state.connection.available_modes=["inspect","follow","monitor","run"];state.connection.blocked_modes={};render()');
    assert.equal(nodes.get("startFeedback").textContent,"Start force feedback");
    await run('$("startFeedback").listeners.click()');
    assert.deepEqual(requests.at(-1),{action:"start",mode:"run"},"Formal run contract changed");
  }else if(scenario==="experiment"){
    run(`
      state.mode="experiment";state.status="active";state.telemetry_fresh=true;stateReceivedAt=nowMs();
      state.connection={...state.connection,state:"active",mode:"experiment",can_start:false,can_stop:true,available_modes:["inspect","follow","experiment"]};
      state.experimental={state:"waiting_native_unlock",ceiling_mw:200,effective_rise_mw_s:Array(7).fill(100),baseline_frozen:false,mass_confirmed:false,reason:"native_locked"};
      state.requested_mw=Array(7).fill(0);state.hold_requested=Array(7).fill(false);state.leader_controlled=false;
      state.leader_output_state="unconfirmed_native_transition";
      render();
    `);
    assert.ok(nodes.get("status").textContent.includes("跟随＋实验阻尼"));
    assert.ok(nodes.get("experimentalStatus").textContent.includes("暂停从臂新目标"));
    assert.ok(nodes.get("experimentalStatus").textContent.includes("200.00 mW"));
    assert.ok(nodes.get("experimentalStatus").textContent.includes("启动基线已冻结: 否"));
    assert.ok(nodes.get("experimentalRise").textContent.includes("J4 100.00"),"Legacy rise limit was not reported");
    assert.equal(nodes.get("experimentalReason").textContent,"native_locked");
    run('state.experimental.state="waiting_baseline";state.experimental.reason="waiting_follow_target";render()');
    assert.ok(nodes.get("experimentalStatus").textContent.includes("等待跟随到位并静止采集空载基线"));
    assert.ok(nodes.get("experimentalReason").textContent.includes("限速接近 Leader 目标"));
    run('lang="en";translate()');
    assert.ok(nodes.get("experimentalReason").textContent.includes("Approaching the Leader target through rate limits"));
    run('state.experimental.reason="collecting_declared_unloaded_baseline";render()');
    assert.ok(nodes.get("experimentalReason").textContent.includes("zero output"));
    run('lang="zh";state.experimental.state="waiting_native_unlock";translate()');
    assert.equal(run('telemetryFresh()'),true);
    for(let i=0;i<6;i++)assert.equal(text("rows",i,8),"由原生按钮板控制／状态未确认");
    assert.equal(text("rows",6,8),"释放请求","Independent gripper release was hidden");
    run('state.experimental={...state.experimental,state:"active",baseline_frozen:true,reason:"<script>text only</script>"};render()');
    assert.ok(nodes.get("experimentalStatus").textContent.includes("实验阻尼运行中"));
    assert.ok(nodes.get("baseline").textContent.includes("启动基线已冻结: 是"));
    assert.equal(nodes.get("experimentalReason").textContent,"<script>text only</script>");
    for(let i=0;i<6;i++)assert.equal(text("rows",i,8),"由原生按钮板控制／状态未确认","Unconfirmed native transition was shown as release");
    run('lang="en";translate()');
    assert.ok(nodes.get("experimentalStatus").textContent.includes("Experimental damping active"));
    assert.ok(nodes.get("experimentalRise").textContent.includes("Current axis rise rates: J1 100.00"));
    assert.ok(nodes.get("commissioning").textContent.includes("Formal force feedback is not ready"));
    assert.ok(run('copy.en.experimentNote').includes("no automatic HOLD"));
    assert.ok(run('copy.zh.experimentNote').includes("无自动 HOLD"));
    assert.equal(JSON.stringify(run('Object.keys(copy.zh).sort()')),JSON.stringify(run('Object.keys(copy.en).sort()')));
    assert.equal(run('state.profile_status.force_ready'),false);
    for(let i=0;i<6;i++)assert.equal(text("rows",i,8),"Native button board control / state unconfirmed");
    assert.equal(text("rows",6,8),"Release requested");
    run('delete state.leader_output_state;state.experimental.state="waiting_native_unlock";render()');
    for(let i=0;i<6;i++)assert.equal(text("rows",i,8),"Native button board control / state unconfirmed","Native unlock wait was shown as release");
    run('state.experimental.state="active";state.leader_controlled=true;render()');
    for(let i=0;i<7;i++)assert.equal(text("rows",i,8),"Release requested");
    run('state.status="stopped";state.telemetry_fresh=false;render()');
    assert.ok(nodes.get("experimentalStatus").textContent.includes("Last reported experiment state (updates stopped)"),"Stopped output was still presented as a live experiment state");
  }else if(scenario==="confirmed_caps"){
    run(`
      state.mode="experiment";state.status="active";state.telemetry_fresh=true;stateReceivedAt=nowMs();
      state.profile_status={engineering:true,mass_confirmed:true,force_ready:false};
      state.connection={...state.connection,state:"active",mode:"experiment",can_start:false,can_stop:true,available_modes:["inspect","follow","experiment"]};
      state.experimental={state:"active",ceiling_mw:null,baseline_frozen:true,mass_confirmed:true,reason:"mass_confirmed_experimental_damping"};
      state.caps={common_mw:1200,axis_mw:[1500,1000,0,1200,1400,800,1500]};render();
    `);
    assert.ok(nodes.get("commissioning").textContent.includes("模型覆盖已由用户确认"));
    assert.ok(nodes.get("commissioning").textContent.includes("动态阈值及效果仍待验收"));
    assert.ok(nodes.get("feedbackBlocked").textContent.includes("正式力反馈尚未就绪"));
    assert.ok(nodes.get("feedbackBlocked").textContent.includes("模型覆盖已由用户确认"));
    assert.ok(!nodes.get("commissioning").textContent.includes("尚未确认"));
    assert.ok(!nodes.get("feedbackBlocked").textContent.includes("尚未确认"));
    assert.ok(nodes.get("experimentalStatus").textContent.includes("无；采用共同／每轴上限"));
    assert.ok(!nodes.get("experimentalStatus").textContent.includes("0.00 mW"),"Null cap was converted to zero");
    assert.ok(nodes.get("experimentalCaps").textContent.includes("J1 1200.00"));
    assert.ok(nodes.get("experimentalCaps").textContent.includes("J2 1000.00"));
    assert.ok(nodes.get("experimentalCaps").textContent.includes("J3 0.00"));
    assert.ok(nodes.get("experimentalCaps").textContent.includes("夹爪 1200.00"));
    assert.ok(nodes.get("powerNote").textContent.includes("各轴可调 0–1500 mW"));
    assert.ok(!nodes.get("status").textContent.includes("200"));
    assert.ok(!run('copy.zh.experimentNote').includes("200"));
    assert.ok(nodes.get("experimentalRise").textContent.includes("J4 —"),"Missing effective rates were inferred from policy");
    run('state.experimental.effective_rise_mw_s=[6000,5000,4000,6000,3000,2000,null];render()');
    assert.ok(nodes.get("experimentalRise").textContent.includes("各轴当前上升速率: J1 6000.00"));
    assert.ok(nodes.get("experimentalRise").textContent.includes("J4 6000.00"));
    assert.ok(nodes.get("experimentalRise").textContent.includes("夹爪 —"),"Null rate was shown as zero");
    assert.equal(run('state.profile_status.force_ready'),false);
    assert.equal(run('availableModes(state.connection).includes("run")'),false);
    run('state.caps.common_mw=400;render()');
    assert.ok(nodes.get("experimentalCaps").textContent.includes("J1 400.00"),"Current 400 mW common cap was replaced by the 1200 mW default");
    run('state.experimental.effective_caps_mw=[350,300,0,250,200,150,100];render()');
    assert.ok(nodes.get("experimentalCaps").textContent.includes("J1 350.00"),"Backend effective caps were ignored");
    run('delete state.experimental.effective_caps_mw;state.experimental.ceiling_mw=200;render()');
    assert.ok(nodes.get("experimentalStatus").textContent.includes("200.00 mW"),"Legacy finite cap disappeared");
    assert.ok(nodes.get("experimentalCaps").textContent.includes("J1 200.00"),"Legacy finite cap was not applied");
    run('state.experimental.ceiling_mw=null;lang="en";translate()');
    assert.ok(nodes.get("commissioning").textContent.includes("Model coverage is confirmed by the user"));
    assert.ok(nodes.get("feedbackBlocked").textContent.includes("Formal force feedback is not ready"));
    assert.ok(!nodes.get("feedbackBlocked").textContent.includes("unconfirmed"));
    assert.ok(nodes.get("experimentalStatus").textContent.includes("None; common/axis caps apply"));
    assert.ok(nodes.get("experimentalRise").textContent.includes("Current axis rise rates: J1 6000.00"));
    assert.ok(nodes.get("experimentalRise").textContent.includes("Gripper —"));
    assert.ok(nodes.get("powerNote").textContent.includes("each axis from 0–1500 mW"));
    assert.ok(!run('copy.en.experimentNote').includes("200"));
    assert.equal(JSON.stringify(run('Object.keys(copy.zh).sort()')),JSON.stringify(run('Object.keys(copy.en).sort()')));
    run('delete state.experimental.ceiling_mw;render()');
    assert.ok(nodes.get("experimentalStatus").textContent.includes("Additional experimental power cap: —"));
    assert.ok(nodes.get("experimentalCaps").textContent.includes("J1 —"),"Absent metadata was treated as uncapped");
    assert.equal(requests.length,0,"Displaying model confirmation or caps sent a command");
  }else if(scenario==="baseline_admission"){
    run(`
      state.mode="experiment";state.status="active";state.telemetry_fresh=true;stateReceivedAt=nowMs();
      state.experimental={state:"waiting_baseline",baseline_frozen:false,reason:"waiting_follower_settled"};
      state.baseline_admission={allowed:false,reason:"waiting_follower_settled",
        axis_names:["J1","J2","J3","J4","J5","J6","gripper"],
        axis_units:["rad","rad","rad","rad","rad","rad","normalized_opening"],
        target:[0,0,0,-0.3,0,0,0.5],sent_target:[0,0,0,-0.4,0,0,0.5],measured_position:[0,0,0,-0.6,0,0,0.13],
        target_error:[0,0,0,0.1,0,0,0],target_tolerance:[0.01,0.01,0.01,0.01,0.01,0.01,0.01],
        arrival_error:[0,0,0,-0.2,0,0,-0.37],arrival_tolerance:[0.15,0.15,0.15,0.15,0.15,0.15,0.035],
        target_blocked_axes:["J4"],arrival_blocked_axes:["J4","gripper"],blocked_axes:["J4","gripper"]};
      render();
    `);
    assert.equal(nodes.get("baselineAdmissionPanel").hidden,false);
    assert.equal(nodes.get("baselineAdmissionRows").children.length,7);
    for(let i=0;i<7;i++)assert.equal(nodes.get("baselineAdmissionRows").children[i].hidden,![3,6].includes(i));
    assert.equal(text("baselineAdmissionRows",3,1),"-17.19","Arm radians were not converted to degrees");
    assert.equal(text("baselineAdmissionRows",3,4),"5.73 / 0.57");
    assert.equal(text("baselineAdmissionRows",3,5),"-11.46 / 8.59","Arrival error sign was inverted");
    assert.equal(text("baselineAdmissionRows",6,0),"夹爪");
    assert.equal(text("baselineAdmissionRows",6,1),"50.00");
    assert.equal(text("baselineAdmissionRows",6,2),"50.00");
    assert.equal(text("baselineAdmissionRows",6,3),"13.00","Gripper was not converted to percent opening");
    assert.equal(text("baselineAdmissionRows",6,5),"-37.00 / 3.50");
    const gripperRow=nodes.get("baselineAdmissionRows").children[6],gripperCell=gripperRow.children[3];
    run('state.baseline_admission.blocked_axes=["gripper"];state.baseline_admission.measured_position[6]=0.2;render()');
    assert.equal(nodes.get("baselineAdmissionRows").children[6],gripperRow,"Clearing one gate rebuilt other axes");
    assert.equal(gripperRow.children[3],gripperCell,"Polling replaced an admission cell");
    assert.equal(gripperCell.textContent,"20.00");
    assert.equal(nodes.get("baselineAdmissionRows").children[3].hidden,true);
    run('lang="en";translate()');
    assert.equal(text("baselineAdmissionRows",6,0),"Gripper");
    assert.equal(nodes.get("baselineAdmissionColumns").children[5].textContent,"Measured − sent / allowed");
    assert.equal(JSON.stringify(run('Object.keys(copy.zh).sort()')),JSON.stringify(run('Object.keys(copy.en).sort()')));
    run('state.baseline_admission.arrival_tolerance[6]=null;render()');
    assert.equal(nodes.get("baselineAdmissionPanel").hidden,true,"Incomplete gate was presented as a complete diagnostic");
    run('state.baseline_admission.arrival_tolerance[6]=0.035;state.baseline_admission.axis_units[6]="rad";render()');
    assert.equal(nodes.get("baselineAdmissionPanel").hidden,true,"Unknown gripper units were displayed as opening");
    run('state.baseline_admission.axis_units[6]="normalized_opening";state.baseline_admission.allowed=true;render()');
    assert.equal(nodes.get("baselineAdmissionPanel").hidden,true,"Passed gate still claimed to block damping");
    run('state.baseline_admission.allowed=false;state.experimental.baseline_frozen=true;render()');
    assert.equal(nodes.get("baselineAdmissionPanel").hidden,true,"Frozen baseline was shown as awaiting admission");
    run('state.experimental.baseline_frozen=false;state.experimental.state="active";render()');
    assert.equal(nodes.get("baselineAdmissionPanel").hidden,true,"Active damping was shown as blocked by motion");
    run('state.experimental.state="waiting_baseline";state.status="stopped";state.telemetry_fresh=false;render()');
    assert.equal(nodes.get("baselineAdmissionPanel").hidden,true,"Stopped gate was presented as live");
    run('state.status="active";state.telemetry_fresh=true;stateReceivedAt=nowMs();state.baseline_admission.blocked_axes=null;render()');
    assert.equal(nodes.get("baselineAdmissionPanel").hidden,true,"Unavailable gate data was presented as an axis failure");
    assert.equal(requests.length,0,"Displaying baseline diagnostics sent a command");
  }else if(scenario==="partial_baseline"){
    run(`
      state.mode="experiment";state.status="active";state.telemetry_fresh=true;stateReceivedAt=nowMs();
      state.experimental={state:"partially_active",ceiling_mw:null,baseline_frozen:false,
        baseline_collection_closed:true,baseline_ready_axes:[true,true,true,true,true,true,false],
        baseline_groups:{arm:{frozen:true},gripper:{frozen:false}},
        reason:"arm_active_gripper_baseline_unavailable"};
      state.baseline_admission={allowed:false,axis_names:["J1","J2","J3","J4","J5","J6","gripper"],
        axis_units:["rad","rad","rad","rad","rad","rad","normalized_opening"],
        target:Array(7).fill(0.5),sent_target:Array(7).fill(0.5),measured_position:Array(7).fill(0.4341),
        target_error:Array(7).fill(0),target_tolerance:Array(7).fill(0.02),
        arrival_error:Array(7).fill(-0.0659),arrival_tolerance:Array(7).fill(0.02),
        blocked_axes:["J4","gripper"]};render();
    `);
    assert.ok(nodes.get("experimentalStatus").textContent.includes("部分关节组阻尼运行中"));
    assert.ok(nodes.get("experimentalBaselines").textContent.includes("六轴 J1–J6: 已冻结"));
    assert.ok(nodes.get("experimentalBaselines").textContent.includes("夹爪: 启动时未就绪 本次阻尼为 0"));
    assert.equal(nodes.get("baselineAdmissionRows").children[3].hidden,true,"Ready arm was shown as awaiting baseline");
    assert.equal(nodes.get("baselineAdmissionRows").children[6].hidden,false);
    assert.equal(text("baselineAdmissionRows",6,5),"-6.59 / 2.00");
    const row=nodes.get("baselineAdmissionRows").children[6];
    run('lang="en";translate()');
    assert.ok(nodes.get("experimentalBaselines").textContent.includes("Arm J1–J6: Frozen"));
    assert.ok(nodes.get("experimentalBaselines").textContent.includes("Gripper: Unready at startup"));
    assert.ok(nodes.get("experimentalReason").textContent.includes("gripper damping remains zero this session"));
    assert.equal(nodes.get("baselineAdmissionRows").children[6],row,"Language switch rebuilt diagnostic rows");
    assert.equal(requests.length,0,"Group status sent a hardware command");
  }else if(scenario==="local_reference"){
    run(`
      state.mode="experiment";state.status="active";state.telemetry_fresh=true;stateReceivedAt=nowMs();
      state.experimental={state:"active",local_reference:{enabled:true,available:true,in_region:true,
        eligible:true,active:true,source:["none","none","none","local_reference","none","none"],
        used_axes:[false,false,false,true,false,false],excess_nm:[0,0,0,0.2,0,0]}};
      state.requested_mw=[0,0,0,240,0,0,0];render();
    `);
    assert.equal(nodes.get("localReferencePanel").hidden,false);
    assert.equal(nodes.get("localReferenceTable").hidden,false);
    assert.ok(nodes.get("localReferenceStatus").textContent.includes("正在采用局部通道"));
    assert.equal(nodes.get("localReferenceRows").children.length,6,"Gripper entered the arm reference channel");
    assert.equal(text("localReferenceRows",3,1),"局部参考");
    assert.equal(text("localReferenceRows",3,2),"0.20");
    assert.equal(text("localReferenceRows",3,3),"240.00");
    const row=nodes.get("localReferenceRows").children[3],cell=row.children[3];
    run('lang="en";translate()');
    assert.ok(nodes.get("localReferenceStatus").textContent.includes("Local channel selected"));
    assert.equal(text("localReferenceRows",3,1),"Local reference");
    assert.equal(nodes.get("localReferenceRows").children[3],row,"Language change rebuilt reference rows");
    assert.equal(row.children[3],cell);
    run('state.experimental.local_reference.reason="warming_local_reference";state.experimental.local_reference.active=false;render()');
    assert.ok(nodes.get("localReferenceStatus").textContent.includes("Local warmup"),"Warmup was presented as ready");
    run('delete state.experimental.local_reference.reason');
    run('state.experimental.local_reference.in_region=false;state.experimental.local_reference.eligible=false;state.experimental.local_reference.active=false;state.experimental.local_reference.used_axes=Array(6).fill(false);state.experimental.local_reference.source=Array(6).fill("absolute");render()');
    assert.ok(nodes.get("localReferenceStatus").textContent.includes("Outside the reference pose region"));
    assert.equal(text("localReferenceRows",3,2),"—","Out-of-region local excess was presented as eligible");
    run('state.status="stopped";state.telemetry_fresh=false;render()');
    assert.ok(nodes.get("localReferenceStatus").textContent.includes("Last report"));
    assert.equal(nodes.get("localReferenceTable").hidden,true,"Stopped reference was shown as live output");
    run('state.status="active";state.telemetry_fresh=true;stateReceivedAt=nowMs();state.experimental.local_reference={enabled:false};render()');
    assert.ok(nodes.get("localReferenceStatus").textContent.includes("Disabled"));
    run('delete state.experimental.local_reference;render()');
    assert.equal(nodes.get("localReferencePanel").hidden,true,"Legacy state was shown as a local reference");
    assert.equal(JSON.stringify(run('Object.keys(copy.zh).sort()')),JSON.stringify(run('Object.keys(copy.en).sort()')));
    assert.equal(requests.length,0,"Local reference display sent a command");
  }else if(scenario==="tracking"){
    assert.equal(nodes.get("followRows").children.length,6);
    assert.equal(text("followRows",0,1),"—");
    assert.equal(text("followRows",0,6),"—","Missing booleans were treated as unrestricted");
    assert.equal(nodes.get("followFreshness").textContent,"等待跟随数据");
    run(`
      state.follow_diagnostics={raw_leader_deg:[40,1],filtered_leader_deg:[38,1],leader_clipped_deg:[30,1],
        leader_range_clipped:[true,false],mapped_sdk_rad:[Math.PI/2,0.5],requested_sdk_rad:[Math.PI/2,0.5],
        limit_clipped_sdk_rad:[Math.PI/3,0.5],sent_sdk_rad:[Math.PI/6,0.5],actual_sdk_rad:[Math.PI/12,0.5],
        joint_limit_clipped:[true,false],rate_limited:[true,false]};
      state.loop_timing={target_hz:100,status_hz:5,actual_hz:83.4,overruns:7};
      state.status="active";state.telemetry_fresh=true;stateReceivedAt=nowMs();render();
    `);
    assert.equal(text("followRows",0,1),"90.00");assert.equal(text("followRows",0,2),"60.00");
    assert.equal(text("followRows",0,3),"30.00");assert.equal(text("followRows",0,4),"15.00");
    assert.equal(text("followRows",0,5),"15.00");
    assert.ok(text("followRows",0,6).includes("Leader 范围裁剪"));
    assert.ok(text("followRows",0,6).includes("关节限位裁剪"));
    assert.ok(text("followRows",0,6).includes("速率限制"));
    assert.equal(text("followRows",1,6),"无裁剪／限速");
    assert.equal(text("followRows",2,5),"—");
    assert.equal(text("followRawRows",0,1),"40.00");assert.equal(text("followRawRows",0,4),"90.00");
    assert.equal(nodes.get("followFreshness").textContent,"实时跟随数据");
    assert.ok(nodes.get("loopTiming").textContent.includes("83.40 Hz"));
    run('state.status="stopped";state.telemetry_fresh=false;lang="en";translate()');
    assert.equal(nodes.get("followFreshness").textContent,"Last following data; updates stopped");
    assert.ok(text("followRows",0,6).includes("Joint limit clipped"));
    assert.ok(nodes.get("loopTiming").textContent.includes("Overrun cycles: 7"));
    run('state.loop_timing={target_hz:100,status_hz:5};render()');
    assert.ok(nodes.get("loopTiming").textContent.includes("Actual: — Hz"),"Actual rate was inferred from configured rate");
    const followRow=nodes.get("followRows").children[0], followCell=followRow.children[3];
    const telemetryRow=nodes.get("rows").children[0], policyRow=nodes.get("policyRows").children[0];
    run('state.follow_diagnostics.sent_sdk_rad[0]=Math.PI/4;render()');
    assert.equal(nodes.get("followRows").children[0],followRow,"Polling rebuilt the following row");
    assert.equal(followRow.children[3],followCell,"Polling rebuilt the following cell");
    assert.equal(followCell.textContent,"45.00");
    assert.equal(nodes.get("rows").children[0],telemetryRow,"Polling rebuilt telemetry rows");
    assert.equal(nodes.get("policyRows").children[0],policyRow,"Polling rebuilt read-only policy rows");
    const cap=nodes.get("cap0");
    run('dirty=true;$("cap0").value="777";changeLanguage()');
    assert.equal(nodes.get("cap0"),cap,"Language transition replaced the focused input");
    assert.equal(cap.value,"777","Language transition discarded an unapplied cap");
    run('setVisible($("experimentalPanel"),false)');
    assert.equal(nodes.get("experimentalPanel").hidden,true,"Exit motion delayed logical hiding");
    assert.equal(nodes.get("experimentalPanel").inert,true,"Outgoing controls stayed interactive");
    run('setVisible($("experimentalPanel"),true)');
    assert.equal(nodes.get("experimentalPanel").inert,false);
    assert.equal(requests.length,0,"Rendering diagnostics sent a command");
  }else throw Error("Unknown scenario");
}
main().catch(error=>{console.error(error);process.exitCode=1;});
'''


@unittest.skipUnless(shutil.which("node"), "Node is required for dashboard checks")
class FollowDiagnosticsTests(unittest.TestCase):
    def run_scenario(self, scenario):
        web = Path(__file__).resolve().parents[1] / "src/stararm_lumos_feedback/web"
        with tempfile.TemporaryDirectory() as directory:
            script = Path(directory) / "follow-diagnostics.cjs"
            script.write_text(HARNESS, encoding="utf-8")
            result = subprocess.run(
                [shutil.which("node"), str(script), str(web / "index.html"), str(web / "app.js"), scenario],
                text=True, capture_output=True, timeout=30,
            )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_explicit_experiment_start_keeps_formal_run_and_startup_contracts(self):
        self.run_scenario("controls")

    def test_experiment_state_and_unreviewed_mass_are_honestly_displayed_in_both_languages(self):
        self.run_scenario("experiment")

    def test_confirmed_model_uses_current_caps_without_opening_formal_run_or_null_as_zero(self):
        self.run_scenario("confirmed_caps")

    def test_tracking_shows_actual_clipping_lag_and_reported_loop_rate_without_commands(self):
        self.run_scenario("tracking")

    def test_baseline_gate_shows_only_complete_blocking_axes_with_signed_errors_and_correct_units(self):
        self.run_scenario("baseline_admission")

    def test_partial_baseline_reports_ready_arm_and_unavailable_gripper_in_both_languages(self):
        self.run_scenario("partial_baseline")

    def test_local_reference_reports_scope_and_selection_without_commands_or_stale_output(self):
        self.run_scenario("local_reference")
