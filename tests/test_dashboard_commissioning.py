"""Local HTTP and rendering checks for commissioning status and inspect mode."""

import http.client
import json
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from stararm_lumos_feedback.control import Caps
from stararm_lumos_feedback.dashboard import Dashboard


class CommissioningDashboardTests(unittest.TestCase):
    def dashboard(self, purpose="inspect", clock=None, **changes):
        profile = SimpleNamespace(
            purpose=purpose,
            engineering=True,
            mass_confirmed=False,
            commissioning=dict(baseline_status="not_measured"),
            policy=[{} for _ in range(7)],
            paths={"private": "/private/calibration.json"},
            evidence="private hardware details",
        )
        profile.__dict__.update(changes)
        options = {} if clock is None else {"clock": clock}
        dashboard = Dashboard(Caps(), profile, port=0, **options)
        dashboard.start()
        self.addCleanup(dashboard.close)
        return dashboard

    def request(self, dashboard, method="GET", path="/api/state", data=None):
        connection = http.client.HTTPConnection("127.0.0.1", dashboard.server.server_port, timeout=2)
        self.addCleanup(connection.close)
        headers = {"X-Feedback-Token": dashboard.token}
        body = None if data is None else json.dumps(data)
        if data is not None:
            headers["Content-Type"] = "application/json"
        connection.request(method, path, body=body, headers=headers)
        response = connection.getresponse()
        return response.status, json.loads(response.read())

    def test_profile_status_contains_flags_without_paths_or_evidence(self):
        code, state = self.request(self.dashboard())
        self.assertEqual(code, 200)
        self.assertEqual(
            state["profile_status"],
            dict(mass_confirmed=False, engineering=True, baseline_status="not_measured", force_ready=False,
                 experimental_damping_available=False),
        )
        self.assertFalse(state["caps_writable"])
        self.assertEqual(state["mode"], "inspect")
        serialized = json.dumps(state)
        self.assertNotIn("/private", serialized)
        self.assertNotIn("private hardware", serialized)
        self.assertNotIn("evidence", state["profile_status"])

    def test_engineering_profile_cannot_report_force_ready_with_confirmed_mass(self):
        code, state = self.request(self.dashboard("monitor", mass_confirmed=True))
        self.assertEqual(code, 200)
        self.assertFalse(state["profile_status"]["force_ready"])
        self.assertTrue(state["caps_writable"])

    def test_inspect_server_rejects_caps_even_with_valid_origin_and_token(self):
        dashboard = self.dashboard()
        original = dashboard.caps.get()
        code, result = self.request(
            dashboard, "POST", "/api/caps", dict(common_mw=1500, axis_mw=[1500] * 7)
        )
        self.assertEqual(code, 403)
        self.assertEqual(result["code"], "caps_read_only")
        self.assertEqual(dashboard.caps.get(), original)
        dashboard.publish(mode="run", status="active")
        code, _ = self.request(dashboard, "POST", "/api/caps", original)
        self.assertEqual(code, 403)

    def test_inspect_telemetry_mode_also_blocks_caps(self):
        dashboard = self.dashboard("monitor")
        for mode in ("inspect", "preview"):
            dashboard.publish(mode=mode, requested_mw=[None] * 7)
            code, state = self.request(dashboard)
            self.assertEqual(code, 200)
            self.assertFalse(state["caps_writable"])
            code, _ = self.request(dashboard, "POST", "/api/caps", dashboard.caps.get())
            self.assertEqual(code, 403)

    def test_monitor_can_change_only_power_caps(self):
        dashboard = self.dashboard("monitor")
        requested = dict(common_mw=900, axis_mw=[800] * 7)
        code, result = self.request(dashboard, "POST", "/api/caps", requested)
        self.assertEqual(code, 200)
        self.assertEqual(result, requested)
        code, _ = self.request(dashboard, "POST", "/api/caps", dict(requested, deadzone_nm=0))
        self.assertEqual(code, 400)
        self.assertEqual(dashboard.caps.get(), requested)

    def test_legacy_profile_and_untrusted_baseline_text_are_handled(self):
        profile = SimpleNamespace(policy=[{}] * 7)
        dashboard = Dashboard(Caps(), profile, port=0)
        dashboard.start()
        self.addCleanup(dashboard.close)
        _, state = self.request(dashboard)
        self.assertEqual(state["profile_status"]["baseline_status"], "not_recorded")
        self.assertFalse(state["profile_status"]["force_ready"])
        profile.commissioning = {"baseline_status": "/private/evidence.txt"}
        _, state = self.request(dashboard)
        self.assertEqual(state["profile_status"]["baseline_status"], "unknown")
        self.assertNotIn("/private", json.dumps(state))

    def test_get_ages_samples_without_refreshing_publication_or_live_status(self):
        now = [10.0]
        dashboard = self.dashboard("monitor", clock=lambda: now[0], axes=[dict(max_age_s=0.2)] * 7)
        dashboard.publish(status="active", mode="monitor", q_sdk=[0.0] * 7, age_ms=[20.0] * 7)
        _, first = self.request(dashboard)
        self.assertTrue(first["telemetry_fresh"])
        self.assertEqual(first["publication_sequence"], 1)
        self.assertTrue(first["caps_writable"])
        now[0] += 0.05
        _, second = self.request(dashboard)
        self.assertAlmostEqual(second["age_s"], 0.05)
        self.assertAlmostEqual(second["age_ms"][0], 70)
        self.assertEqual(second["publication_sequence"], first["publication_sequence"])
        self.assertTrue(second["telemetry_fresh"])
        now[0] += 0.2
        _, stale = self.request(dashboard)
        self.assertFalse(stale["telemetry_fresh"])
        self.assertFalse(stale["caps_writable"])
        self.assertEqual(stale["status"], "active")
        self.assertEqual(stale["q_sdk"], first["q_sdk"])
        self.assertAlmostEqual(stale["age_ms"][0], 270)
        self.assertEqual(dashboard.state["age_ms"], [20.0] * 7)
        original = dashboard.caps.get()
        code, value = self.request(dashboard, "POST", "/api/caps", dict(common_mw=1500, axis_mw=[1500] * 7))
        self.assertEqual((code, value["code"]), (403, "caps_read_only"))
        self.assertEqual(dashboard.caps.get(), original)
        dashboard.publish(status="active", mode="monitor", q_sdk=[0.0] * 7, age_ms=[0.0] * 7)
        _, recovered = self.request(dashboard)
        self.assertEqual(recovered["publication_sequence"], 2)
        self.assertTrue(recovered["telemetry_fresh"])

    def test_live_requires_complete_coherent_samples_and_an_observation_mode(self):
        dashboard = self.dashboard("monitor", axes=[dict(max_age_s=0.1)] + [dict(max_age_s=0.2)] * 6)
        base = dict(status="active", mode="monitor", q_sdk=[0.0] * 7, age_ms=[0.0] * 7)
        for changes in (
            dict(q_sdk=[0.0] * 6),
            dict(age_ms=[None] * 7),
            dict(age_ms=[101.0] * 7),
            dict(coherent_feedback=False),
            dict(status="stopped"),
            dict(status="inspection_waiting"),
            dict(mode="preview"),
        ):
            dashboard.publish(**dict(base, **changes))
            self.assertFalse(dashboard.snapshot()["telemetry_fresh"], changes)
        dashboard.publish(**dict(base, status="leader_gap"))
        self.assertTrue(dashboard.snapshot()["telemetry_fresh"])

    def test_connection_controls_are_explicit_query_start_stop_only(self):
        dashboard = self.dashboard()
        code, result = self.request(dashboard, "POST", "/api/connection", dict(action="start"))
        self.assertEqual((code, result["code"]), (409, "connection_unavailable"))

        class Manager:
            def __init__(self):
                self.actions = []
                self.state = "idle"

            def status(self):
                return dict(state=self.state, can_start=self.state == "idle", can_stop=self.state == "active")

            def request(self, action):
                self.actions.append(action)
                if action == "start" and self.state != "idle":
                    raise RuntimeError("Query session already started")
                self.state = "active" if action == "start" else "stopped"
                return self.status()

        dashboard.connection = manager = Manager()
        _, state = self.request(dashboard)
        self.assertEqual(state["connection"]["state"], "idle")
        self.assertFalse(state["telemetry_fresh"])
        for invalid in (dict(action="run"), dict(action="start", enable=True), [], dict()):
            code, _ = self.request(dashboard, "POST", "/api/connection", invalid)
            self.assertEqual(code, 400)
        self.assertEqual(manager.actions, [])
        code, started = self.request(dashboard, "POST", "/api/connection", dict(action="start"))
        self.assertEqual((code, started["state"]), (200, "active"))
        code, _ = self.request(dashboard, "POST", "/api/connection", dict(action="start"))
        self.assertEqual(code, 409)
        code, stopped = self.request(dashboard, "POST", "/api/connection", dict(action="stop"))
        self.assertEqual((code, stopped["state"]), (200, "stopped"))
        self.assertEqual(manager.actions, ["start", "start", "stop"])

    @unittest.skipUnless(shutil.which("node"), "Node is required for the dashboard rendering check")
    def test_inspect_rendering_and_language_parity(self):
        web = Path(__file__).resolve().parents[1] / "src/stararm_lumos_feedback/web"
        # A small DOM harness exercises the real renderer without browser or network access.
        harness = r'''
const fs=require("fs"), vm=require("vm"), assert=require("assert");
const nodes=new Map();
class Element {
  constructor(){this.children=[];this.disabled=false;this.textContent="";this.dataset={};this.listeners={};}
  set id(value){this._id=value;nodes.set(value,this);} get id(){return this._id;}
  append(...values){this.children.push(...values);}
  replaceChildren(...values){this.children=values;}
  setAttribute(key,value){this[key]=value;}
  addEventListener(type,listener){this.listeners[type]=listener;}
  querySelectorAll(){return [nodes.get("common"),...Array.from({length:7},(_,i)=>nodes.get("cap"+i)),nodes.get("applyButton")];}
  getContext(){return {clearRect(){}};}
}
for(const id of fs.readFileSync(process.argv[2],"utf8").matchAll(/id="([^"]+)"/g)){const el=new Element();el.id=id[1];}
const button=new Element();button.id="applyButton";
const context=vm.createContext({
  document:{documentElement:{},getElementById:id=>nodes.get(id),createElement:()=>new Element(),querySelectorAll:()=>[]},
  localStorage:{getItem(){return null;},setItem(){}},
  fetch:()=>new Promise(()=>{}),setTimeout(){},console
});
vm.runInContext(fs.readFileSync(process.argv[3],"utf8"),context);
vm.runInContext(`
  if(JSON.stringify(Object.keys(copy.zh).sort())!==JSON.stringify(Object.keys(copy.en).sort()))throw new Error("Translation keys differ");
  state={status:"inspection_active",mode:"inspect",caps_writable:false,caps:{common_mw:1200,axis_mw:Array(7).fill(1200)},
    profile_status:{engineering:true,mass_confirmed:false,force_ready:false,baseline_status:"not_measured"},
    motor_status:Array(7).fill(0),requested_mw:Array(7).fill(null),predicted_mw:Array(7).fill(null),hold_requested:Array(7).fill(null),
    residual_verified:false,policy:Array(7).fill({})};
  pageConnected=true;stateReceivedAt=nowMs();
  for(const language of ["zh","en"]){
    lang=language;translate();
    if(!$("common").disabled||!$("cap0").disabled||!$("applyButton").disabled)throw new Error("Inspect inputs enabled");
    if(!$("commissioning").textContent.includes(copy[lang].forceBlocked))throw new Error("Force blocking banner missing");
    if(!$("commissioning").textContent.includes(copy[lang].massUnconfirmed))throw new Error("Mass warning missing");
    if(!$("modeNote").textContent.includes(copy[lang].inspectNote))throw new Error("Inspect explanation missing");
    const row=$("rows").children[0].children.map(cell=>cell.textContent);
    if(row[2]!==copy[lang].motorDisabled)throw new Error("Disabled motor mislabelled");
    if(row[6]!=="—"||row[7]!=="—")throw new Error("Unknown requests displayed as values");
    if(row[8]!==copy[lang].unknown)throw new Error("Uncontrolled Leader mislabelled");
    state.mode="monitor";state.caps_writable=true;
    state.baseline={ready:true,sample_count:42,continuous_stationary_s:1.4,unloaded_verified:false};render();
    if(!$("baseline").textContent.includes("42")||!$("baseline").textContent.includes(copy[lang].baselineReady))throw new Error("Runtime baseline progress missing");
    if(!$("baseline").textContent.includes(copy[lang].baselineStationaryWarning))throw new Error("Stationarity incorrectly establishes unloaded state");
    state.mode="inspect";state.caps_writable=false;render();
    if($("baseline").textContent.includes("42"))throw new Error("Inspect displayed a sampled baseline");
    state.mode="preview";state.status="preview_ready";render();
    if(!$("modeNote").textContent.includes(copy[lang].previewNote))throw new Error("Preview explanation missing");
    if(!$("capsReadOnly").textContent.includes(copy[lang].previewCaps))throw new Error("Preview caps explanation missing");
    if($("baseline").textContent.includes("42"))throw new Error("Preview displayed a sampled baseline");
    if(!$("common").disabled)throw new Error("Preview caps enabled");
    if($("connectionPanel")&&!$("connectionPanel").hidden)throw new Error("Legacy page displayed unavailable connection controls");
    modelDefinition={geometry_source:"urdf_meshes",source:{label:"LumosTouch V3"},links:[],preview_q:[0,0,0,0,0,0],sdk_to_urdf:[{scale:-1,offset:.2},...Array(5).fill({scale:1,offset:0})]};
    let displayedQ=null,displayedTarget=null;
    solidViewer={geometrySource:"urdf_meshes",meshLoaded:9,meshExpected:9,meshFailed:0,available:true,update(q,opening,target){displayedQ=q;displayedTarget=target;}};
    state.mode="monitor";state.status="active";state.q_sdk=[.3,0,0,0,0,0,.5];state.telemetry_fresh=true;state.caps_writable=true;stateReceivedAt=nowMs();render();
    if($("poseSource").dataset.live!=="true"||!$("poseSource").textContent.includes(copy[lang].livePose))throw new Error("Fresh hardware pose not labelled live");
    if(Math.abs(displayedQ[0]+.1)>1e-12)throw new Error("SDK pose was not transformed exactly once");
    if(!$("showAlignmentTarget").disabled||!$("alignmentTable").hidden||$("alignmentStatus").textContent!==copy[lang].alignmentUnavailable||displayedTarget!==null)throw new Error("Missing alignment reference was not clearly unavailable");
    modelDefinition.alignment={available:true,target_sdk_rad:[.4,-.1,0,0,0,0],target_urdf_rad:[1.2,-.1,0,0,0,0],leader_home_deg:[-30,20,0,0,0,0],leader_ids:[0,1,2,3,4,5],tolerance_rad:Array(6).fill(.2)};
    render();
    if(!$("showAlignmentTarget").checked||$("showAlignmentTarget").disabled||$("alignmentTable").hidden)throw new Error("Calibration target was not enabled by default");
    if(displayedTarget!==modelDefinition.alignment.target_urdf_rad||displayedTarget[0]!==1.2)throw new Error("URDF target was omitted or transformed again");
    if($("alignmentStatus").textContent!==copy[lang].alignmentCurrentFresh)throw new Error("Fresh alignment reference explanation missing");
    const alignmentRow=$("alignmentRows").children[0].children.map(cell=>cell.textContent);
    if(JSON.stringify(alignmentRow)!==JSON.stringify(["J1","17.19","22.92","+5.73","±11.46","0","-30.00"]))throw new Error("Alignment table SDK degrees, signed delta, tolerance or raw Leader home incorrect: "+JSON.stringify(alignmentRow));
    if($("alignmentRows").children[1].children[3].textContent!=="-5.73")throw new Error("Negative remaining turn lost its sign");
    if(!$("alignmentColumns").children[1].textContent.includes("°")||!$("alignmentColumns").children[6].textContent.includes("°"))throw new Error("Follower and Leader degree units missing");
    $("showAlignmentTarget").checked=false;$("showAlignmentTarget").listeners.change();
    if(displayedTarget!==null)throw new Error("Hiding calibration target did not clear viewer ghost");
    $("showAlignmentTarget").checked=true;$("showAlignmentTarget").listeners.change();
    if(displayedTarget!==modelDefinition.alignment.target_urdf_rad)throw new Error("Showing calibration target did not restore viewer ghost");
    state.q_urdf=[.7,0,0,0,0,0];render();
    if(displayedQ[0]!==.7)throw new Error("Existing URDF telemetry was transformed a second time");
    delete state.q_urdf;
    state.telemetry_fresh=false;render();
    if($("poseSource").dataset.live!=="false"||!$("poseSource").textContent.includes(copy[lang].lastPose))throw new Error("Stale hardware pose labelled live");
    if(!$("status").textContent.includes(copy[lang].telemetryStale)||!$("common").disabled)throw new Error("Stale runtime still presents active controls");
    if($("alignmentStatus").textContent!==copy[lang].alignmentNeedsLive||$("alignmentRows").children.some(row=>row.children[1].textContent!=="—"||row.children[3].textContent!=="—"))throw new Error("Stale current pose was used to assess alignment");
    if($("alignmentRows").children[0].children[2].textContent!=="22.92"||displayedTarget!==modelDefinition.alignment.target_urdf_rad)throw new Error("Static target disappeared when live feedback expired");
    state.telemetry_fresh=true;stateReceivedAt=nowMs()-2000;render();
    if($("poseSource").dataset.live!=="false")throw new Error("A hung HTTP request retained a live pose");
    if($("alignmentRows").children[0].children[3].textContent!=="—")throw new Error("Hung HTTP request retained a live alignment delta");
    modelDefinition.alignment={available:false,reason:"paired_calibration_unavailable"};render();
    if($("alignmentStatus").textContent!==copy[lang].alignmentUnavailable||!$("alignmentTable").hidden||$("alignmentRows").children.length||displayedTarget!==null)throw new Error("Unavailable calibration retained a stale target");
    state.connection={state:"active",mode:"inspect",can_stop:true,startup_pose_options:["home","current_aligned"]};
    state.mode="inspect";state.status="inspection_active";stateReceivedAt=nowMs();state.current_alignment={available:true,target_sdk_rad:[.3,0,0,.58,0,0],target_urdf_rad:[.3,0,0,.58,0,0],leader_ids:[0,1,2,3,4,5],leader_home_deg:[-13,0,0,36.5,0,0],tolerance_rad:Array(6).fill(.03)};render();
    $("startupPose").value="current_aligned";$("startupPose").listeners.change();
    if(startupPose!=="current_aligned"||$("startupPose").disabled||$("startupPosePanel").hidden)throw new Error("Current pose reference cannot be selected during read-only inspection");
    if($("alignmentStatus").textContent!==copy[lang].alignmentCurrentPoseFresh||$("alignmentRows").children[3].children[6].textContent!=="36.50")throw new Error("Inverse Leader target not displayed");
    if(displayedTarget!==state.current_alignment.target_urdf_rad)throw new Error("Current-pose ghost does not use validated reference");
    state.connection.startup_pose_options.push("current_relative");state.relative_alignment={...state.current_alignment,source:"current_relative_origin",leader_home_deg:[4.1,.1,-1.7,3.3,2.8,-7.7]};
    $("startupPose").value="current_relative";$("startupPose").listeners.change();
    if($("alignmentStatus").textContent!==copy[lang].alignmentRelativeFresh||!$("startupPoseNote").textContent.includes(copy[lang].startupRelativeNote))throw new Error("Relative startup explanation missing");
    if($("alignmentRows").children[3].children[6].textContent!=="3.30"||displayedTarget!==state.relative_alignment.target_urdf_rad)throw new Error("Relative display changed the original Leader home or current reference");
    state.telemetry_fresh=false;render();
    if(!$("alignmentTable").hidden||displayedTarget!==null)throw new Error("Stale current-pose reference survived");
    $("startupPose").value="home";$("startupPose").listeners.change();delete state.connection;
    stateReceivedAt=nowMs();state.mode="preview";state.status="preview_ready";render();
    if($("poseSource").dataset.live!=="false")throw new Error("Offline preview labelled live");
    state.mode="run";state.status="stopped";render();
    if(!$("status").textContent.includes(copy[lang].stopped)||$("poseSource").dataset.live!=="false")throw new Error("Stopped runtime still labelled active");
    if($("connectionPanel")){
      state.connection={state:"idle",can_start:true,can_stop:false};state.software_version="0.5.0";render();
      if($("connectionPanel").hidden||$("connectHardware").disabled||!$("disconnectHardware").disabled)throw new Error("Idle query controls incorrect");
      if(!$("connectionSummary").textContent.includes("0.5.0"))throw new Error("Software version missing");
      if(!$("buttonBoardPanel").hidden)throw new Error("Legacy server displayed unavailable button board controls");
      state.connection.button_board={enabled:true,id:7,verified:null,can_change:true};render();
      if($("buttonBoardPanel").hidden||!$("buttonBoardEnabled").checked||$("buttonBoardDisabled").checked||$("buttonBoardEnabled").disabled||$("buttonBoardDisabled").disabled)throw new Error("Stopped button board configuration incorrect");
      if(!$("buttonBoardStatus").textContent.includes(copy[lang].buttonBoardUnverified)||!$("buttonBoardStatus").textContent.includes("ID 7")||$("buttonBoardStatus").textContent.includes(copy[lang].buttonBoardVerified))throw new Error("Configured board falsely reported as verified");
      state.connection.button_board.verified=false;render();
      if(!$("buttonBoardStatus").textContent.includes(copy[lang].buttonBoardNoReply)||!$("buttonBoardEnabled").checked)throw new Error("Missing button feedback changed the configured choice");
      state.connection.button_board.verified=true;render();
      if(!$("buttonBoardStatus").textContent.includes(copy[lang].buttonBoardVerified))throw new Error("Actual verified feedback was not displayed");
      state.connection.button_board.enabled=false;render();
      if(!$("buttonBoardDisabled").checked||!$("buttonBoardStatus").textContent.includes(copy[lang].buttonBoardNotRead)||$("buttonBoardStatus").textContent.includes(copy[lang].buttonBoardVerified))throw new Error("Disabled reading incorrectly claimed board verification");
      for(const phase of ["starting","active","stopping"]){
        state.connection.state=phase;state.connection.button_board.can_change=false;render();
        if(!$("buttonBoardEnabled").disabled||!$("buttonBoardDisabled").disabled||$("buttonBoardLocked").hidden)throw new Error("Button board selection writable before session fully stopped");
      }
      const beforeWaitingStatus=state.status;
      state.status="button_board_waiting";state.connection={state:"starting",mode:"follow",can_start:false,can_stop:true,available_modes:["inspect","follow","monitor"],button_board:{enabled:true,id:7,verified:null,can_change:false}};render();
      for(const id of ["status","connectionSummary"]){
        if(!$(id).textContent.includes(copy[lang].button_board_waiting))throw new Error("Button listening instructions missing from "+id);
        if($(id).textContent.includes(copy[lang].connection_active))throw new Error("Waiting for a button was displayed as active");
      }
      for(const key of ["button_board_waiting","buttonBoardNote","button_board_no_reply"]){
        if(/\b60\b/.test(copy[lang][key]))throw new Error("Webpage button instructions retain a fixed listening deadline");
      }
      if($("disconnectHardware").disabled||!$("connectHardware").disabled||!$("startFollowing").disabled||!$("buttonBoardDisabled").disabled)throw new Error("Button waiting lost Stop or allowed another start/configuration change");
      pageConnected=false;render();
      if(!$("connectionSummary").textContent.includes(copy[lang].disconnected)||$("connectionSummary").textContent.includes(copy[lang].button_board_waiting))throw new Error("Disconnected page still claimed active button listening");
      pageConnected=true;state.status=beforeWaitingStatus;render();
      if($("connectionSummary").textContent.includes(copy[lang].button_board_waiting)||!$("connectionSummary").textContent.includes(copy[lang].connection_starting))throw new Error("Button waiting message survived its runtime status");
      state.connection={state:"active",can_start:false,can_stop:true};render();
      if(!$("connectHardware").disabled||$("disconnectHardware").disabled)throw new Error("Active query controls incorrect");
      state.connection={state:"idle",mode:"inspect",available_modes:["inspect","follow","monitor"],blocked_modes:{run:"force_profile_unreviewed"},can_start:true,can_stop:false};render();
      if($("startFollowing").disabled||$("startMonitoring").disabled||!$("startFeedback").disabled)throw new Error("Available modes or blocked force control incorrect");
      if($("disconnectHardware").textContent!==copy[lang].disconnectHardware)throw new Error("Query stop label incorrect");
      if($("connectionMotionNote").textContent!==copy[lang].connectionMotionNote)throw new Error("Motion and control exit explanation missing");
      if($("feedbackBlocked").hidden||$("feedbackBlocked").textContent!==copy[lang].force_profile_unreviewed)throw new Error("Force readiness reason missing");
      for(const mode of ["follow","monitor","run"]){
        state.connection={...state.connection,state:"active",mode,can_start:false,can_stop:true};render();
        if(!$("connectionSummary").textContent.includes(copy[lang][mode])||!$("connectionSummary").textContent.includes(copy[lang][mode+"Note"]))throw new Error("Current motion mode incorrectly described");
        if($("connectionSummary").textContent.includes(copy[lang].connectionQueryNote))throw new Error("Motion described as read-only queries");
        if(!$("status").textContent.includes(copy[lang][mode])||$("modeNote").textContent!==copy[lang][mode+"Note"])throw new Error("Motion mode label missing");
        if(["connectHardware","startFollowing","startMonitoring","startFeedback"].some(id=>!$(id).disabled))throw new Error("Mode can switch during an active session");
        if($("connectionSwitchNote").hidden)throw new Error("Mode switch explanation missing");
        if($("disconnectHardware").textContent!==copy[lang].stopMotion)throw new Error("Motion stop label incorrect");
      }
      state.connection={...state.connection,state:"idle",mode:"inspect",can_start:true,can_stop:false,available_modes:["inspect"]};render();
      if(!$("startFollowing").disabled||!$("startMonitoring").disabled)throw new Error("Unconfigured motion mode enabled");
      for(const code of ["can_interface_missing","can_interface_down","can_bitrate_mismatch","can_bus_fault","can_query_failed","can_ip_unavailable","can_query_timeout"]){
        const diagnostic={error:"raw system error",error_code:code,error_interface:"can0",error_detail:"system detail"};
        Object.assign(state,diagnostic);state.connection={state:"error",can_start:true,can_stop:false,...diagnostic};render();
        const translated=copy[lang][code].replace("{interface}","can0");
        for(const id of ["connectionResult","runtimeError"]){
          if(!$(id).textContent.startsWith(translated))throw new Error("CAN diagnostic was not translated: "+code);
          if($(id).textContent.includes("raw system error"))throw new Error("Raw CAN diagnostic leaked into translated error");
        }
      }
      Object.assign(state,{error:null,error_code:null,error_interface:null,error_detail:null});
      const poseError={error:"raw pose mismatch",error_code:"startup_pose_mismatch",error_interface:null,error_detail:"J4: 0.58 rad"};
      Object.assign(state,poseError);state.connection={state:"error",mode:"follow",can_start:true,can_stop:false,...poseError};render();
      for(const id of ["connectionResult","runtimeError"]){
        if(!$(id).textContent.includes(copy[lang].startup_pose_mismatch)||!$(id).textContent.includes("J4: 0.58 rad"))throw new Error("Pose mismatch lost translation or axis detail");
      }
      const cleanupError={error:"raw cleanup failure",error_code:"sdk_cleanup_failed",error_interface:null,error_detail:"cleanup detail"};
      Object.assign(state,cleanupError);state.connection={state:"error",mode:"follow",can_start:true,can_stop:false,available_modes:["inspect"],blocked_modes:{follow:"sdk_cleanup_failed",monitor:"sdk_cleanup_failed",run:"sdk_cleanup_failed"},...cleanupError};render();
      for(const id of ["connectionResult","runtimeError"]){
        if(!$(id).textContent.includes(copy[lang].sdk_cleanup_failed)||!$(id).textContent.includes("cleanup detail"))throw new Error("Cleanup failure lost translation or detail");
        if($(id).textContent.includes("raw cleanup failure"))throw new Error("Cleanup diagnostic was not translated");
      }
      if($("connectHardware").disabled||["startFollowing","startMonitoring","startFeedback"].some(id=>!$(id).disabled))throw new Error("Cleanup failure did not preserve read-only diagnostics and block motion");
      if($("feedbackBlocked").textContent!==copy[lang].sdk_cleanup_failed)throw new Error("Cleanup failure blocking reason missing");
      const ranges=[{axis:1,parameter:"v_max",sdk:8,hardware:10}];
      state.connection={state:"idle",mode:"inspect",can_start:true,can_stop:false,available_modes:["inspect"],blocked_modes:{follow:"sdk_motor_range_mismatch",monitor:"sdk_motor_range_mismatch",run:"sdk_motor_range_mismatch"},sdk_motor_range_mismatches:ranges};render();
      if($("connectHardware").disabled||["startFollowing","startMonitoring","startFeedback"].some(id=>!$(id).disabled))throw new Error("Native unit mismatch allowed motion or blocked read-only");
      if(!$("feedbackBlocked").textContent.includes(copy[lang].sdk_motor_range_mismatch)||!$("feedbackBlocked").textContent.includes("SDK ±8")||!$("feedbackBlocked").textContent.includes("±10 rad/s"))throw new Error("Native range evidence or translation missing");
      if(!errorText({error:"raw native error",code:"sdk_motor_range_mismatch",detail:{mismatches:ranges}}).includes("±10 rad/s"))throw new Error("Native range rejection lost numeric details");
      Object.assign(state,{error:null,error_code:null,error_interface:null,error_detail:null});
      state.connection={state:"starting",can_start:false,can_stop:true,error:null};render();
      if($("connectionResult").textContent||$("runtimeError").textContent)throw new Error("Old CAN error survived retry");
      pageConnected=false;render();
      if(!$("connectHardware").disabled||!$("disconnectHardware").disabled||!$("common").disabled)throw new Error("Disconnected page retained writable controls");
      pageConnected=true;delete state.connection;
    }
    state.mode="inspect";state.status="inspection_active";
  }
`,context);
assert(nodes.get("capsReadOnly").hidden===false);
vm.runInContext(`(async()=>{
  const requests=[];
  fetch=async(path,options)=>{
    if(path!=="/api/connection")throw new Error("Unexpected HTTP request");
    const payload=JSON.parse(options.body);requests.push(payload);
    return {ok:true,json:async()=>({state:payload.action==="start"?"starting":"stopping",mode:payload.mode??state.connection.mode,can_start:false,can_stop:payload.action==="start",available_modes:["inspect","follow","monitor"],blocked_modes:{run:"force_profile_unreviewed"}})};
  };
  function idle(){
    state.connection={state:"idle",mode:"inspect",can_start:true,can_stop:false,available_modes:["inspect","follow","monitor"],blocked_modes:{run:"force_profile_unreviewed"}};
    pageConnected=true;render();
  }
  for(const [id,mode] of [["connectHardware","inspect"],["startFollowing","follow"],["startMonitoring","monitor"]]){
    idle();await $(id).listeners.click();
    const payload=requests[requests.length-1];
    if(payload.action!=="start"||payload.mode!==mode||Object.keys(payload).length!==(mode==="inspect"?2:3))throw new Error("Wrong mode payload for "+id);
    if(mode!=="inspect"&&payload.startup_pose!=="home")throw new Error("Missing explicit home strategy");
    if(!$("connectHardware").disabled||!$("startFollowing").disabled||!$("startMonitoring").disabled)throw new Error("Starting session allowed parallel start");
  }
  idle();const before=requests.length;
  await $("startFeedback").listeners.click();
  if(requests.length!==before)throw new Error("Blocked feedback request was sent");
  state.connection={...state.connection,state:"active",mode:"follow",can_start:false,can_stop:true};render();
  await $("startMonitoring").listeners.click();
  if(requests.length!==before)throw new Error("Active session allowed mode change");
  await $("disconnectHardware").listeners.click();
  const stop=requests[requests.length-1];
  if(stop.action!=="stop"||Object.keys(stop).length!==1)throw new Error("Stop request carried a start mode");
  idle();state.connection.available_modes.push("run");state.connection.blocked_modes={};render();
  if($("startFeedback").disabled||!$("feedbackBlocked").hidden)throw new Error("Configured feedback mode remained unavailable");
  await $("startFeedback").listeners.click();
  if(requests[requests.length-1].mode!=="run")throw new Error("Configured feedback mode payload missing");
  idle();state.connection.startup_pose_options=["home","current_aligned"];render();
  const beforeStrategy=requests.length;$("startupPose").value="current_aligned";$("startupPose").listeners.change();
  if(requests.length!==beforeStrategy)throw new Error("Selecting alignment enabled hardware");
  await $("startFollowing").listeners.click();
  if(JSON.stringify(requests[requests.length-1])!==JSON.stringify({action:"start",mode:"follow",startup_pose:"current_aligned"}))throw new Error("Explicit current startup choice omitted");
  idle();state.connection.startup_pose_options=["home","current_aligned","current_relative"];render();
  $("startupPose").value="current_relative";$("startupPose").listeners.change();await $("startFollowing").listeners.click();
  if(requests[requests.length-1].startup_pose!=="current_relative")throw new Error("Explicit relative startup choice omitted");
  state.connection={state:"active",mode:"follow",can_start:false,can_stop:true,startup_pose:"current_aligned",startup_pose_options:["home","current_aligned"]};
  startupPose="home";render();
  if(startupPose!=="current_aligned"||$("startupPose").value!=="current_aligned"||!$("startupPose").disabled)throw new Error("Page refresh misrepresented running startup strategy");
  idle();fetch=async()=>({ok:false,json:async()=>({error:"Backend rejected follow",code:"force_profile_unreviewed"})});
  await $("startFollowing").listeners.click();
  if($("connectionResult").textContent!==copy[lang].force_profile_unreviewed)throw new Error("Request rejection code was not translated");
  idle();fetch=async()=>({ok:false,json:async()=>({error:"Backend rejected follow"})});
  await $("startFollowing").listeners.click();
  if(!$("connectionResult").textContent.includes("Backend rejected follow"))throw new Error("Generic request failure disappeared during final render");
  idle();pageConnected=false;const offline=requests.length;
  await $("startFollowing").listeners.click();
  if(requests.length!==offline)throw new Error("Disconnected page sent motion request");
  idle();state.connection.button_board={enabled:true,id:7,verified:null,can_change:true};render();
  const configurationBefore=JSON.stringify({caps:state.caps,profile:state.profile_status,policy:state.policy});
  const boardRequests=[];
  fetch=async(path,options)=>{
    if(path!=="/api/button-board")throw new Error("Button board selection called a different API");
    const payload=JSON.parse(options.body);boardRequests.push(payload);
    return {ok:true,json:async()=>({...state.connection,button_board:{...state.connection.button_board,enabled:payload.enabled,verified:null}})};
  };
  $("buttonBoardDisabled").checked=true;await $("buttonBoardDisabled").listeners.change();
  if(JSON.stringify(boardRequests[0])!==JSON.stringify({enabled:false})||!$("buttonBoardDisabled").checked)throw new Error("Without-board choice payload incorrect");
  $("buttonBoardEnabled").checked=true;await $("buttonBoardEnabled").listeners.change();
  if(JSON.stringify(boardRequests[1])!==JSON.stringify({enabled:true})||!$("buttonBoardEnabled").checked||state.connection.button_board.verified!==null)throw new Error("With-board choice changed verification or sent wrong payload");
  if(JSON.stringify({caps:state.caps,profile:state.profile_status,policy:state.policy})!==configurationBefore)throw new Error("Button board selection changed caps, force readiness or policy");
  for(const phase of ["starting","active","stopping"]){
    state.connection.state=phase;state.connection.button_board.can_change=false;render();
    $("buttonBoardDisabled").checked=true;await $("buttonBoardDisabled").listeners.change();
    if(boardRequests.length!==2||!$("buttonBoardEnabled").checked||!$("buttonBoardDisabled").disabled)throw new Error("Running session changed button board setting");
  }
  state.connection.state="stopped";state.connection.button_board.can_change=true;pageConnected=false;render();
  await requestButtonBoard(false);
  if(boardRequests.length!==2||!$("buttonBoardEnabled").disabled)throw new Error("Offline page sent button board setting");
  pageConnected=true;render();
  let resolveBoard;
  fetch=(path,options)=>new Promise(resolve=>{boardRequests.push(JSON.parse(options.body));resolveBoard=()=>resolve({ok:true,json:async()=>({...state.connection,button_board:{...state.connection.button_board,enabled:false,verified:null}})});});
  const pendingBoard=requestButtonBoard(false);
  if(!$("buttonBoardEnabled").disabled||!$("startFollowing").disabled)throw new Error("Pending button board update left parallel controls writable");
  await requestConnection("start","follow");await requestButtonBoard(false);
  if(boardRequests.length!==3)throw new Error("Button board update allowed a parallel request");
  resolveBoard();await pendingBoard;
  fetch=async()=>({ok:false,json:async()=>({error:"Board setting failed"})});
  await requestButtonBoard(true);
  if(state.connection.button_board.enabled!==false||!$("buttonBoardResult").textContent.includes("Board setting failed"))throw new Error("Failed button board update changed selection or hid error");
})()`,context).catch(error=>{console.error(error);process.exitCode=1;});
'''
        with tempfile.TemporaryDirectory() as temporary:
            script = Path(temporary) / "render.cjs"
            script.write_text(harness)
            result = subprocess.run(
                [shutil.which("node"), str(script), str(web / "index.html"), str(web / "app.js")],
                text=True,
                capture_output=True,
                timeout=5,
            )
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
