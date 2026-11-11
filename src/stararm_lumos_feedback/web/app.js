"use strict";
const copy = {
  zh: {
    description:"整臂空载动力学补偿 · 逐轴剩余力矩阻尼 · 独立堵转 HOLD",
    pose:"Lumos 三维模型", poseNote:"正在读取模型来源。拖动旋转，滚轮缩放。",
    power:"普通阻尼功率上限", powerNote:"实际请求随负载变化；每轴有效上限为共同与该轴上限的较小值。每轴 0 关闭普通阻尼；HOLD 独立请求 1500 mW。mW 是 Leader 请求值。",
    common:"共同上限", apply:"应用上限", telemetry:"实时状态",
    torqueNote:"力矩单位 Nm。残差包含摩擦与模型误差，不能直接换算成准确末端力或重量。夹爪不经过六轴模型。Follower 电流没有已验证的接口，显示 —；Leader 电流来自串口回包。",
    fixed:"固定响应参数", fixedNote:"这些参数只读；修改配置后必须重新生成绑定。前六轴位置/速度/加速度使用 URDF rad、rad/s、rad/s²；夹爪使用归一化开度及其导数。",
    gripper:"夹爪", axis:"关节", saved:"上限已更新", disconnected:"网页连接中断",
    active:"运行中", starting:"正在启动", stopped:"已停止", leader_gap:"Leader 缺帧：暂停新目标",
    preview_ready:"界面已就绪", preview:"离线模型预览", previewNote:"参考姿态仅用于显示；未连接机械臂，不发送运动或力输出命令。", previewCaps:"离线预览，功率设置只读。",
    inspect:"状态查询模式", follow:"基础跟随模式，无 Leader 力输出", monitor:"跟随监测模式，无 Leader 力输出", run:"阻尼模式",
    inspectNote:"仅查询从臂状态；不构造 SDK、不使能、不跟随。Leader 未接管，已有阻尼或保持状态未知。",
    inspection_waiting:"等待完整、同步的新 CAN 回包", inspection_active:"从臂状态查询中",
    inspection_stopped:"状态查询已停止", inspection_error:"状态查询已停止：存在异常",
    engineeringInitial:"工程初始参数；尚未完成实测调校。", massUnconfirmed:"模型质量覆盖尚未确认。", massConfirmed:"模型质量覆盖已确认。", massConfirmedByUser:"模型覆盖已由用户确认；动态阈值及效果仍待验收。",
    forceBlocked:"力输出已禁止。", candidateResidual:"残差仅供观察，尚未验证。",
    baselineLabel:"基线状态", baselineUnknown:"未记录", baselineNotMeasured:"尚未测量", baselineCandidate:"候选数据，未验证", baselineInsufficient:"数据不足", baselineOther:"未确认",
    baselineCaution:"基线状态不代表真机效果已验证。", capsReadOnly:"当前仅查询状态，功率上限不可修改。",
    baselineSampling:"静止窗口采样中", baselineReady:"静止统计已就绪，尚未确认空载", baselineSamples:"有效样本", baselineStationary:"连续静止", baselineStationaryWarning:"静止不等于已确认空载；候选死区尚未用于力输出。",
    hold:"保持", damping:"阻尼", free:"释放请求", unknown:"未知 / 未接管", motorDisabled:"未使能 (0)", motorEnabled:"已使能 (1)", motorFault:"故障 / 状态", residualUnverified:"残差 Nm（未验证）",
    columns:["轴", "SDK 位置", "从臂电机状态", "实测 Nm", "模型 Nm", "残差 Nm", "Leader 请求 mW", "预计 mW", "Leader 状态", "温度 °C", "Follower mA", "Leader mA", "帧龄 ms"]
  },
  en: {
    description:"Whole-arm unloaded dynamics · Per-axis residual damping · Independent stall HOLD",
    pose:"Lumos 3D model", poseNote:"Reading model source. Drag to orbit; scroll to zoom.",
    power:"Ordinary damping power caps", powerNote:"Actual requests vary with load. Each effective cap is the smaller of the common and axis caps. An axis cap of 0 disables ordinary damping; HOLD independently requests 1500 mW. mW is a Leader request.",
    common:"Common cap", apply:"Apply caps", telemetry:"Live state",
    torqueNote:"Torques are Nm. Residuals include friction and model error; they are not accurate end-effector force or weight. The gripper bypasses the arm model. Follower current has no verified interface and is shown as —; Leader current comes from serial feedback.",
    fixed:"Fixed response parameters", fixedNote:"Read only. Configuration edits require a new binding. Arm position/velocity/acceleration use URDF rad, rad/s, rad/s²; the gripper uses normalized opening and its derivatives.",
    gripper:"Gripper", axis:"Axis", saved:"Caps updated", disconnected:"Dashboard connection lost",
    active:"Active", starting:"Starting", stopped:"Stopped", leader_gap:"Leader gap: new targets paused",
    preview_ready:"Interface ready", preview:"Offline model preview", previewNote:"The reference pose is display only. Hardware is disconnected; no motion or force output commands are sent.", previewCaps:"Offline preview; power settings are read only.",
    inspect:"State query mode", follow:"Basic following mode; no Leader force output", monitor:"Following and monitoring mode; no Leader force output", run:"Damping mode",
    inspectNote:"Queries Follower state only; no SDK construction, enabling, or following. Leader is not controlled; any existing damping or hold is unknown.",
    inspection_waiting:"Waiting for complete, coherent new CAN feedback", inspection_active:"Querying Follower state",
    inspection_stopped:"State queries stopped", inspection_error:"State queries stopped: error",
    engineeringInitial:"Engineering starting values; measured tuning is incomplete.", massUnconfirmed:"Model mass coverage is unconfirmed.", massConfirmed:"Model mass coverage is confirmed.", massConfirmedByUser:"Model coverage is confirmed by the user; dynamic thresholds and hardware performance still await acceptance.",
    forceBlocked:"Force output is blocked.", candidateResidual:"Residuals are for observation and remain unverified.",
    baselineLabel:"Baseline status", baselineUnknown:"Not recorded", baselineNotMeasured:"Not measured", baselineCandidate:"Candidate data, unverified", baselineInsufficient:"Insufficient data", baselineOther:"Unconfirmed",
    baselineCaution:"Baseline status does not establish hardware performance validation.", capsReadOnly:"State queries only; power caps cannot be changed.",
    baselineSampling:"Sampling a stationary window", baselineReady:"Stationary statistics ready; no-load state unverified", baselineSamples:"Valid samples", baselineStationary:"Continuously stationary", baselineStationaryWarning:"Stationary does not mean verified unloaded; candidate deadzones are not applied to force output.",
    hold:"Hold", damping:"Damping", free:"Release requested", unknown:"Unknown / not controlled", motorDisabled:"Disabled (0)", motorEnabled:"Enabled (1)", motorFault:"Fault / status", residualUnverified:"Residual Nm (unverified)",
    columns:["Axis", "SDK position", "Follower motor status", "Measured Nm", "Model Nm", "Residual Nm", "Leader request mW", "Predicted mW", "Leader state", "Temp °C", "Follower mA", "Leader mA", "Age ms"]
  }
};
const labels = {
  deadzone_nm:["静态死区 Nm","Static deadzone Nm"], velocity_margin:["速度余量系数","Velocity margin coefficient"], acceleration_margin:["加速度余量系数","Acceleration margin coefficient"], full_scale_nm:["超死区满输出力矩 Nm","Full-output torque above deadzone Nm"], filter_s:["残差滤波 s","Residual filter s"], acceleration_filter_s:["加速度滤波 s","Acceleration filter s"], confirm_s:["负载确认 s","Load confirmation s"], release_s:["负载释放 s","Load release s"], rise_mw_s:["上升速率 mW/s","Rise mW/s"], fall_mw_s:["下降速率 mW/s","Fall mW/s"], hold_error:["HOLD 角差/开度差","HOLD position error"], hold_load_nm:["HOLD 负载 Nm","HOLD load Nm"], hold_speed:["HOLD 最大速度","HOLD max speed"], hold_confirm_s:["HOLD 确认 s","HOLD confirmation s"], hold_progress:["HOLD 追赶速度界限","HOLD progress rate"], hold_release_error:["HOLD 释放角差/开度差","HOLD release error"], hold_release_load_nm:["HOLD 释放负载 Nm","HOLD release load Nm"], hold_release_s:["HOLD 释放确认 s","HOLD release confirmation s"]
};
function readPreference(key) {
  try{return localStorage.getItem(key);}catch{return null;}
}
function writePreference(key,value) {
  try{localStorage.setItem(key,value);}catch{/* Preferences are optional when browser storage is unavailable. */}
}
const validStartupPose = value => ["home","current_aligned","current_relative","leader_target"].includes(value)?value:"home";
let lang = readPreference("lumos-language") === "en" ? "en" : "zh";
Object.assign(copy.zh, {
  experiment:"跟随＋实验阻尼", startExperiment:"启动实验阻尼",
  experimentNote:"六轴与夹爪分别检查跟随到位，七轴实测均须静止至少 2 秒。首组完成后冻结当时已就绪的基线，未就绪组本次保持零阻尼，运行中不自动补采。各轴实际上升速率见下方，无自动 HOLD。单姿态基线未覆盖全部运动范围，实验状态不代表真机效果已验证。",
  experimentPowerNote:"每轴有效上限为共同与该轴上限的较小值；实际请求随负载变化。默认共同／每轴上限 1200 mW，共同可调 400–1500 mW，各轴可调 0–1500 mW；每轴 0 关闭该轴阻尼。实验无自动或软件 HOLD。mW 是 Leader 请求值。",
  formalForceBlocked:"正式力反馈尚未就绪。", experimentState:"实验状态", experimentLastReport:"最后报告的实验状态（当前未更新）", experimentCeiling:"额外实验功率上限", experimentNoExtraCap:"无；采用共同／每轴上限", experimentEffectiveCaps:"各轴当前有效上限", experimentEffectiveRise:"各轴当前上升速率", experimentFrozen:"启动基线已冻结", yes:"是", no:"否",
  experiment_waiting_baseline:"等待跟随到位并静止采集空载基线", experiment_active:"实验阻尼运行中", experiment_model_invalid:"模型无效，实验输出已停止",
  experiment_partially_active:"部分关节组阻尼运行中", baselineArm:"六轴 J1–J6", baselineGripper:"夹爪", baselineGroupFrozen:"已冻结", baselineGroupWaiting:"等待到位与静止", baselineGroupUnavailable:"启动时未就绪 本次阻尼为 0",
  localReferenceTitle:"六轴局部双向参考", localReferenceLastReport:"六轴局部双向参考 最后报告 当前未更新",
  localReferenceEnabled:"已启用", localReferenceDisabled:"未启用 沿用原绝对残差通道", localReferenceWaiting:"已启用 等待冻结参考", localReferenceOutside:"已启用 当前超出参考姿态范围",
  localReferencePaused:"已启用 范围内 局部条件未满足", localReferenceWarming:"已启用 参考范围内 局部预热中", localReferenceEligible:"已启用 参考范围内 局部判定就绪", localReferenceActive:"已启用 参考范围内 正在采用局部通道",
  localReferenceUnknown:"当前条件未确认", localReferenceAbsolute:"绝对残差", localReferenceLocal:"局部参考", localReferenceNone:"未触发", localReferenceUnavailable:"不可用",
  localReferenceColumns:["轴","采用通道","局部有效超量 Nm","最终 Leader 请求 mW"],
  localReferenceNote:"仅在参考姿态附近且七轴静止时可用，夹爪不使用局部通道。每轴取两通道较大有效超量，再经原上限和升降速率，不相加；范围外保留原通道。邻域是工程假设，未验证全行程或真机手感。",
  local_reference_reason_disabled:"局部通道未启用，保留原绝对残差判定。", local_reference_reason_waiting_arm_baseline:"等待六轴首次空载基线冻结。",
  local_reference_reason_local_reference_unavailable:"没有可用的单次连续静止窗口参考，本次不使用局部通道。", local_reference_reason_outside_reference_region:"七轴姿态超出局部参考范围，已清空局部滤波与确认状态。",
  local_reference_reason_reference_motion:"七轴或夹爪原始电机仍有运动，局部通道暂停。", local_reference_reason_warming_local_reference:"当前在参考范围内，正在预热局部滤波与确认。",
  local_reference_reason_local_reference_eligible:"局部条件已满足，逐轴按有效超量和负载确认判断。",
  experiment_reason_arm_active_gripper_baseline_unavailable:"六轴基线已完成，夹爪本次保持零阻尼；修正夹爪跟随后，可在零上限并确认空载时显式重新采集。",
  experiment_reason_gripper_active_arm_baseline_unavailable:"夹爪基线已完成，六轴本次保持零阻尼；六轴跟随到位后，可在零上限并确认空载时显式重新采集。",
  experiment_reason_baseline_admission_missing:"基线采集条件尚未确认，阻尼请求为 0。", experiment_reason_waiting_native_unlock:"等待收到完整真实按钮释放指令。",
  experiment_reason_waiting_first_follow_command:"等待完成首组完整跟随发送，尚不采集基线。", experiment_reason_waiting_follow_target:"正在限速接近 Leader 目标，尚不采集基线。",
  experiment_reason_waiting_follower_settled:"等待从臂实测位置接近已发送目标。", experiment_reason_waiting_stable_follow_target:"Leader 目标有变化，重新等待连续静止。",
  experiment_reason_collecting_declared_unloaded_baseline:"跟随已到位，正在零输出下采集连续空载静止基线。",
  baselineAdmissionTitle:"等待到位的关节", baselineAdmissionNote:"仅显示阻碍基线采集的关节　六轴单位 °　夹爪单位 %",
  baselineAdmissionColumns:["轴","目标","已发送","实测","目标 − 发送 / 允许","实测 − 发送 / 允许"],
  experiment_waiting_native_unlock:"等待按钮板释放：实验输出为 0，暂停从臂新目标；请按按钮至主臂释放，等待收到完整释放指令。",
  experiment_native_locked:"按钮板保持中：实验输出为 0，暂停从臂新目标；请按按钮释放主臂。", nativeLeaderState:"由原生按钮板控制／状态未确认",
  followTitle:"逐轴跟随状态", followTableNote:"六轴角度单位为 °。差值 = 已发送目标 − 实际位置；限位裁剪与速率限制分别显示。以下信息只读，不改变关节限位。",
  followRawDetails:"Leader 原始值与映射明细", followLive:"实时跟随数据", followStopped:"最后跟随数据，当前未更新", followMissing:"等待跟随数据",
  followColumns:["轴","映射目标 °","限位后 °","已发送 °","实际 °","差值 °","限制状态"],
  followRawColumns:["轴","Leader 原始 °","滤波后 °","Leader 裁剪后 °","请求 SDK °"],
  followLeaderClipped:"Leader 范围裁剪", followJointClipped:"关节限位裁剪", followRateLimited:"速率限制", followUnrestricted:"无裁剪／限速",
  loopTarget:"目标", loopStatus:"状态", loopActual:"实际", loopOverruns:"超时周期"
});
Object.assign(copy.en, {
  experiment:"Following + experimental damping", startExperiment:"Start experimental damping",
  experimentNote:"Arm and gripper target arrival are checked separately; all seven measured axes must remain stationary for at least 2 seconds. When the first group qualifies, all groups ready at that time freeze. Other groups keep zero damping for this session and are never learned automatically during output. Actual axis rise rates are shown below, with no automatic HOLD. A single pose does not cover the full motion range, and experiment status does not establish hardware performance validation.",
  experimentPowerNote:"Each effective cap is the smaller of the common and axis caps; actual requests vary with load. Common/axis defaults are 1200 mW. The common cap is adjustable from 400–1500 mW and each axis from 0–1500 mW; 0 disables that axis. Experimental mode has no automatic or software HOLD. mW is a Leader request.",
  formalForceBlocked:"Formal force feedback is not ready.", experimentState:"Experiment state", experimentLastReport:"Last reported experiment state (updates stopped)", experimentCeiling:"Additional experimental power cap", experimentNoExtraCap:"None; common/axis caps apply", experimentEffectiveCaps:"Current effective axis caps", experimentEffectiveRise:"Current axis rise rates", experimentFrozen:"Startup baseline frozen", yes:"Yes", no:"No",
  experiment_waiting_baseline:"Waiting for target arrival and stationary unloaded baseline", experiment_active:"Experimental damping active", experiment_model_invalid:"Model invalid; experimental output stopped",
  experiment_partially_active:"Damping active for qualified groups", baselineArm:"Arm J1–J6", baselineGripper:"Gripper", baselineGroupFrozen:"Frozen", baselineGroupWaiting:"Waiting for arrival and stationarity", baselineGroupUnavailable:"Unready at startup · Zero damping this session",
  localReferenceTitle:"Local bidirectional arm reference", localReferenceLastReport:"Local bidirectional arm reference · Last report · Updates stopped",
  localReferenceEnabled:"Enabled", localReferenceDisabled:"Disabled · Original absolute-residual channel", localReferenceWaiting:"Enabled · Waiting for a frozen reference", localReferenceOutside:"Enabled · Outside the reference pose region",
  localReferencePaused:"Enabled · Within region · Local conditions not met", localReferenceWarming:"Enabled · Within region · Local warmup", localReferenceEligible:"Enabled · Within region · Local check ready", localReferenceActive:"Enabled · Within region · Local channel selected",
  localReferenceUnknown:"Current conditions unconfirmed", localReferenceAbsolute:"Absolute residual", localReferenceLocal:"Local reference", localReferenceNone:"Not triggered", localReferenceUnavailable:"Unavailable",
  localReferenceColumns:["Axis","Selected channel","Local effective excess Nm","Final Leader request mW"],
  localReferenceNote:"Available only near the reference pose while all seven axes are stationary; the gripper has no local channel. Each axis takes the larger eligible excess, then applies the existing caps and ramps; channels are not added. The original channel remains outside this scope. This neighborhood is an engineering assumption, without full-range or hardware-feel validation.",
  local_reference_reason_disabled:"The local channel is disabled; the original absolute-residual check remains.", local_reference_reason_waiting_arm_baseline:"Waiting for the initial unloaded arm baseline to freeze.",
  local_reference_reason_local_reference_unavailable:"No single continuous stationary-window reference is available; the local channel is unavailable for this session.", local_reference_reason_outside_reference_region:"The seven-axis pose is outside the local region; local filter and confirmation state have been cleared.",
  local_reference_reason_reference_motion:"A measured axis or the raw gripper motor is moving; the local channel is paused.", local_reference_reason_warming_local_reference:"Within the reference region; warming the local filter and confirmation.",
  local_reference_reason_local_reference_eligible:"Local conditions are met; each axis still requires effective excess and load confirmation.",
  experiment_reason_arm_active_gripper_baseline_unavailable:"The arm baseline is ready; gripper damping remains zero this session. Correct gripper following, then explicitly recapture at zero caps after confirming the unloaded state.",
  experiment_reason_gripper_active_arm_baseline_unavailable:"The gripper baseline is ready; arm damping remains zero this session. Let the arm reach its target, then explicitly recapture at zero caps after confirming the unloaded state.",
  experiment_reason_baseline_admission_missing:"Baseline admission is unconfirmed; damping requests are 0.", experiment_reason_waiting_native_unlock:"Waiting for a complete real button-release command.",
  experiment_reason_waiting_first_follow_command:"Waiting for the first complete successful following write; baseline collection has not started.", experiment_reason_waiting_follow_target:"Approaching the Leader target through rate limits; baseline collection has not started.",
  experiment_reason_waiting_follower_settled:"Waiting for measured Follower positions to approach the sent target.", experiment_reason_waiting_stable_follow_target:"The Leader target changed; restarting the stationary wait.",
  experiment_reason_collecting_declared_unloaded_baseline:"Following has arrived; collecting continuous declared-unloaded stationary samples with zero output.",
  baselineAdmissionTitle:"Axes still approaching target", baselineAdmissionNote:"Only axes blocking baseline collection are shown · Arm ° · Gripper %",
  baselineAdmissionColumns:["Axis","Target","Sent","Measured","Target − sent / allowed","Measured − sent / allowed"],
  experiment_waiting_native_unlock:"Waiting for button-board release: experimental output is 0 and new Follower targets are paused. Press to release the Leader and wait for a complete release command.",
  experiment_native_locked:"Button board is holding: experimental output is 0 and new Follower targets are paused. Press to release the Leader.", nativeLeaderState:"Native button board control / state unconfirmed",
  followTitle:"Per-axis following", followTableNote:"Arm angles are degrees (°). Difference = sent target − actual position. Joint clipping and rate limiting are shown separately. Read only; joint limits are unchanged.",
  followRawDetails:"Raw Leader values and mapping details", followLive:"Live following data", followStopped:"Last following data; updates stopped", followMissing:"Waiting for following data",
  followColumns:["Axis","Mapped °","After limits °","Sent °","Actual °","Difference °","Limits applied"],
  followRawColumns:["Axis","Raw Leader °","Filtered °","Clipped Leader °","Requested SDK °"],
  followLeaderClipped:"Leader range clipped", followJointClipped:"Joint limit clipped", followRateLimited:"Rate limited", followUnrestricted:"No clipping / rate limit",
  loopTarget:"Target", loopStatus:"Status", loopActual:"Actual", loopOverruns:"Overrun cycles"
});
Object.assign(copy.zh, {resetView:"恢复视角", diagnostics:"连接与工程状态", powerDetails:"功率设置", viewerHint:"拖动旋转 · 滚轮缩放", referencePose:"参考姿态 · 非实时数据", livePose:"实时姿态", lastPose:"最后接收姿态 · 数据已停止", schematic:"URDF 实体示意", modelLoading:"正在加载模型", modelUnavailable:"模型加载失败", rendererFallback:"基础渲染", meshSource:"真实 STL", meshLoading:"正在加载真实 STL", meshIncomplete:"真实网格不完整", meshFailed:"真实网格加载失败", meshFailedCount:"失败", cpuRenderer:"CPU 渲染", meshNote:"网格来源：{source} 原始 STL，包含机械臂和该模型夹爪；显示姿态见左上角状态。拖动旋转，滚轮缩放。", schematicNote:"关节坐标与连杆尺寸来自 Lumos URDF；外壳和 TypeLJ 外形为实体示意。拖动旋转，滚轮缩放。"});
Object.assign(copy.en, {resetView:"Reset view", diagnostics:"Connection and engineering status", powerDetails:"Power settings", viewerHint:"Drag to orbit · Scroll to zoom", referencePose:"Reference pose · Not live data", livePose:"Live pose", lastPose:"Last received pose · Updates stopped", schematic:"URDF schematic solids", modelLoading:"Loading model", modelUnavailable:"Model unavailable", rendererFallback:"Basic rendering", meshSource:"Actual STL", meshLoading:"Loading actual STL", meshIncomplete:"Actual meshes incomplete", meshFailed:"Actual meshes failed to load", meshFailedCount:"failed", cpuRenderer:"CPU rendering", meshNote:"Mesh source: original {source} STL files, including the arm and its model gripper. The upper-left status identifies the displayed pose. Drag to orbit; scroll to zoom.", schematicNote:"Joint frames and link dimensions come from the Lumos URDF. Housings and TypeLJ shapes are schematic solids. Drag to orbit; scroll to zoom."});
Object.assign(copy.zh, {
  button_board_waiting:"等待新款按钮：网页会持续监听，直至收到按键或点击停止。请先支撑主臂，再按一次按钮。",
  showAlignmentTarget:"显示校准目标", alignmentTitle:"六轴对齐参考", alignmentCurrentLegend:"当前姿态", alignmentTargetLegend:"青色半透明：校准目标",
  alignmentCurrentFresh:"角差由新鲜回包计算。校准目标为静态参考，不代表已对齐，也不是实时 Leader 目标。",
  alignmentNeedsLive:"当前没有新鲜姿态，无法判断角差。请先连接真机（只读）；青色目标仅为静态校准参考。",
  alignmentUnavailable:"校准目标不可用：未读取到有效的配对校准参考。", alignmentLoading:"正在读取校准目标。",
  alignmentNote:"Follower 角度采用 SDK 关节方向，单位为度（°）；还需转动 = 校准目标 − 当前。不执行自动回零。进入目标容差不代表可以启动；启动还会检查两臂关节对应和限位。",
  leaderHomeNote:"Leader home 是各 ID 舵机的校准位置（舵机度），不是 Follower 角度。这里只显示保存的校准值，不发送移动命令。",
  alignmentColumns:["轴","当前 SDK °","校准目标 SDK °","还需转动 °","允许差 °","Leader ID","Leader home 舵机 °"]
});
Object.assign(copy.en, {
  button_board_waiting:"Waiting for a native button: listening continues until a press or Stop. Support the Leader, then press once.",
  showAlignmentTarget:"Show calibration target", alignmentTitle:"Six-axis alignment reference", alignmentCurrentLegend:"Current pose", alignmentTargetLegend:"Translucent cyan: calibration target",
  alignmentCurrentFresh:"Differences use fresh feedback. The calibration target is a static reference; it does not indicate alignment or the live Leader target.",
  alignmentNeedsLive:"No fresh pose is available, so differences cannot be assessed. Connect hardware in read-only mode first. The cyan target is a static calibration reference.",
  alignmentUnavailable:"Calibration target unavailable: a valid paired calibration reference could not be read.", alignmentLoading:"Loading calibration target.",
  alignmentNote:"Follower angles use SDK joint directions in degrees (°). Remaining turn = calibration target − current. No automatic homing is performed. Being within target tolerance does not establish readiness to start; startup also checks joint correspondence and limits.",
  leaderHomeNote:"Leader home is each servo ID's calibrated position in servo degrees, not a Follower angle. These are saved calibration values only; no motion commands are sent.",
  alignmentColumns:["Axis","Current SDK °","Calibration target SDK °","Remaining turn °","Allowed difference °","Leader ID","Leader home servo °"]
});
Object.assign(copy.zh, {
  telemetryStale:"遥测已过期", controlsOffline:"网页连接中断，功率设置只读。", controlsStale:"遥测已过期，功率设置只读。",
  connection_idle:"未启动", connection_starting:"正在启动", connection_active:"运行中", connection_stopping:"正在停止", connection_stopped:"已停止", connection_error:"已停止：存在异常", inspection_stopping:"正在停止状态查询",
  connectionTitle:"真机连接与遥操作", connectionStatus:"会话状态", connectHardware:"连接真机（只读）", startFollowing:"启动基础跟随", startMonitoring:"跟随并监测", startFeedback:"启动力反馈", disconnectHardware:"停止采集", stopMotion:"停止并退出控制",
  connectionQueryNote:"仅 CAN 状态查询；不使能、不发送运动目标。",
  followNote:"基础跟随会使能从臂，并按 Leader 姿态实际运动；不向 Leader 输出阻尼。工程基础跟随不使用六轴动力学加速度门禁，其他运动和通信保护保留。",
  monitorNote:"跟随监测会使能从臂并实际跟随，同时计算模型残差和预计阻尼；不向 Leader 输出阻尼。此模式仍执行动力学加速度检查。",
  runNote:"力反馈会使能从臂、实际跟随，并按逐轴剩余力矩向 Leader 请求阻尼。",
  connectionMotionNote:"基础跟随和跟随监测会使能从臂并实际运动；监测模式不输出阻尼。停止会退出控制，可能失去重力支撑；请先支撑从臂。已进入 SDK 的动作不能撤回；此按钮不是机械急停。",
  connectionSwitchNote:"切换模式前，请先停止当前模式。",
  force_profile_unreviewed:"正式力反馈暂不可启动：动态阈值、空载和带载响应尚未验收。模型覆盖状态单独显示；真实 CAD 和联网成功不能确认力反馈就绪。",
  sdk_motor_range_mismatch:"运动暂不可启动：SDK 编码及解码量程与实机不一致，量程修正尚未就绪。需安装并验证匹配实机的底层修正；可连接只读状态查看真实姿态。",
  sdk_compatibility_failed:"SDK 量程修正校验失败，未进入电机使能。请核对绑定和加载顺序，并重启服务后重试。",
  modeUnavailable:"此模式尚未配置，当前不可启动。", connectionRequested:"启动请求已提交", disconnectionRequested:"停止请求已提交", softwareVersion:"软件版本"
});
Object.assign(copy.en, {
  telemetryStale:"Telemetry expired", controlsOffline:"Dashboard disconnected; power settings are read only.", controlsStale:"Telemetry expired; power settings are read only.",
  connection_idle:"Not started", connection_starting:"Starting", connection_active:"Active", connection_stopping:"Stopping", connection_stopped:"Stopped", connection_error:"Stopped: error", inspection_stopping:"Stopping state queries",
  connectionTitle:"Hardware connection and teleoperation", connectionStatus:"Session status", connectHardware:"Connect hardware (read only)", startFollowing:"Start basic following", startMonitoring:"Follow and monitor", startFeedback:"Start force feedback", disconnectHardware:"Stop queries", stopMotion:"Stop and exit control",
  connectionQueryNote:"CAN state queries only; no motor enabling or motion targets.",
  followNote:"Basic following enables the Follower and moves it according to the Leader pose. No damping is sent to the Leader. Engineering basic following omits the arm dynamics acceleration admission; other motion and communication protections remain.",
  monitorNote:"Monitoring enables and moves the Follower while calculating model residuals and predicted damping. No damping is sent to the Leader. Dynamics acceleration checks remain active in this mode.",
  runNote:"Force feedback enables and moves the Follower, and requests Leader damping from each axis residual torque.",
  connectionMotionNote:"Basic following and monitoring enable and move the Follower; monitoring sends no damping. Stopping exits control and may remove gravity support. Support the Follower first. Actions already passed to the SDK cannot be recalled. This button is not a mechanical emergency stop.",
  connectionSwitchNote:"Stop the current mode before switching.",
  force_profile_unreviewed:"Formal force feedback is unavailable: dynamic thresholds and unloaded/loaded response await acceptance. Model coverage is shown separately; actual CAD and a working connection do not establish force readiness.",
  sdk_motor_range_mismatch:"Motion is unavailable: SDK packing and decoding ranges disagree with the hardware, and the range correction is not ready. Install and verify a matching native correction. Read-only connection can show the real pose.",
  sdk_compatibility_failed:"SDK range correction verification failed before motor enabling. Check the binding and load order, then restart the service before retrying.",
  modeUnavailable:"This mode is not configured and cannot be started.", connectionRequested:"Start request submitted", disconnectionRequested:"Stop request submitted", softwareVersion:"Software version"
});
Object.assign(copy.zh, {
  can_interface_missing:"电脑未识别到 CAN 接口 {interface}。请重新插接 USB-CAN，检查数据线和 USB 口，待系统识别后再点击连接。",
  can_interface_down:"CAN 接口 {interface} 尚未启动。请先按硬件说明配置接口，再点击连接。",
  can_bitrate_mismatch:"CAN 接口 {interface} 的波特率与配置不符。请检查接口配置。",
  can_bus_fault:"CAN 接口 {interface} 处于故障或停止状态。请检查机械臂电源、CAN 接线及终端电阻。",
  can_query_failed:"无法读取 CAN 接口 {interface} 的状态。请检查系统网络工具和权限。",
  can_ip_unavailable:"系统缺少 ip 命令。请安装 iproute2 后重试。",
  can_query_timeout:"查询 CAN 接口 {interface} 超时。请检查 USB-CAN 和系统状态后重试。",
  startup_pose_mismatch:"启动前姿态检查未通过，未启动跟随。",
  startupHomeNote:"校准零位对齐要求从臂和 Leader 映射目标均接近各自的原校准零位，同时检查两端差值。仅让两端互相接近仍可能被拒绝。若希望从当前稳定姿态按角度增量跟随，请明确选择“从当前位置相对起步”。",
  sdk_cleanup_failed:"SDK 清理失败，电机状态尚未确认。本页已禁止再次启动运动，请先支撑从臂并排查；仍可使用只读诊断。",
  button_board_no_reply:"未检测到有效 HD 按钮事件或旧款回包。请检查按钮板、串口接线和供电，重新启动检测；只有实物无板时才选择“无按钮板”。",
  button_board_command_incomplete:"新款按钮指令组不完整，按键状态不确定，已停止跟随。请检查连接并重新启动检测。",
  native_button_force_unreviewed:"已识别到自行控制保持的新款按钮板；该板与动态阻尼的输出仲裁尚未完成验证，目前支持基础跟随和监测。",
  gripper_motor_range:"TypeLJ 夹爪原始电机位置超出当前允许范围，未通过运行检查。请核对夹爪实际开合位置与零位。",
  gripper_motor_velocity:"TypeLJ 夹爪原始电机速度超出当前允许范围，未通过运行检查。"
});
Object.assign(copy.en, {
  can_interface_missing:"The computer cannot find CAN interface {interface}. Reconnect USB-CAN and check the data cable and USB port; retry after the system detects it.",
  can_interface_down:"CAN interface {interface} is down. Configure it using the hardware instructions, then retry.",
  can_bitrate_mismatch:"CAN interface {interface} has a different bitrate from the configuration. Check the interface settings.",
  can_bus_fault:"CAN interface {interface} is faulted or stopped. Check arm power, CAN wiring and termination.",
  can_query_failed:"Cannot read CAN interface {interface}. Check the system networking tools and permissions.",
  can_ip_unavailable:"The ip command is missing. Install iproute2, then retry.",
  can_query_timeout:"Querying CAN interface {interface} timed out. Check USB-CAN and the system, then retry.",
  startup_pose_mismatch:"The startup pose check failed. Following was not started.",
  startupHomeNote:"Calibrated-home startup requires both the Follower and mapped Leader target near their original calibrated homes, as well as checking their difference. Bringing the arms close to each other may still fail. To follow angle changes from stable current poses, explicitly select Relative start at current pose.",
  sdk_cleanup_failed:"SDK cleanup failed and motor state is unconfirmed. This page blocks further motion starts. Support the Follower before investigating. Read-only diagnostics remain available.",
  button_board_no_reply:"No valid HD event or legacy reply was detected. Check the button board, serial wiring and power, then restart detection. Select Without button board only when no board is fitted.",
  button_board_command_incomplete:"The native button command group is incomplete and its state is unknown. Following stopped. Check the connection and restart detection.",
  native_button_force_unreviewed:"A native board that controls hold itself was detected. Its arbitration with dynamic damping remains unvalidated; use basic following or monitoring.",
  gripper_motor_range:"The TypeLJ raw motor position is outside the current allowed range. The operating check failed. Check the physical opening and motor zero.",
  gripper_motor_velocity:"The TypeLJ raw motor velocity is outside the current allowed range. The operating check failed."
});
Object.assign(copy.zh, {
  buttonBoardTitle:"HD 按钮板", buttonBoardEnabled:"有按钮板", buttonBoardDisabled:"无按钮板", buttonBoardConfiguration:"当前配置", buttonBoardVerification:"回包验证",
  buttonBoardUnverified:"尚未验证", buttonBoardVerified:"有效回包已验证", buttonBoardNoReply:"未收到有效回包", buttonBoardNotRead:"未启用读取",
  buttonBoardNote:"有板时自动兼容旧款查询和新款主动指令。网页启动会持续监听，直至检测到有效按键或点击停止。新款请等出现监听提示后，先支撑主臂，再按一次按钮；板自身会锁定／释放 Leader。程序监听按键来暂停／恢复从臂目标，不重发新款保持指令。开关不启动力反馈。",
  buttonBoardLocked:"请完全停止当前会话后修改按钮板选择。", buttonBoardSaved:"按钮板选择已保存；回包状态以实际读取为准。"
});
Object.assign(copy.en, {
  buttonBoardTitle:"HD button board", buttonBoardEnabled:"With button board", buttonBoardDisabled:"Without button board", buttonBoardConfiguration:"Current configuration", buttonBoardVerification:"Feedback verification",
  buttonBoardUnverified:"Not yet verified", buttonBoardVerified:"Valid feedback verified", buttonBoardNoReply:"No valid feedback received", buttonBoardNotRead:"Reading disabled",
  buttonBoardNote:"With a board, legacy replies and native commands are detected automatically. Webpage startup continues listening until a valid button event or Stop. For a native board, wait for the listening prompt, support the Leader, then press once. The board locks/releases the Leader itself; software observes the event to pause/resume Follower targets and never retransmits it. This selector does not start force feedback.",
  buttonBoardLocked:"Fully stop the current session before changing the button board setting.", buttonBoardSaved:"Button board choice saved. Feedback status depends on actual reads."
});
Object.assign(copy.zh, {
  startupPoseTitle:"启动方式", startupHome:"校准零位对齐（默认）", startupCurrent:"按当前位置对齐（工程测试）",
  startupPoseNote:"当前姿态方式：先让从臂处于稳定、有支撑的位置并闭合夹爪，再连接只读状态。按下方 Leader 目标角度调整主臂；停止只读后再启动基础跟随，无需把 J4 托到零位。使能时仍可能有短暂位移，尚未真机验证保持效果。",
  alignmentCurrentPoseFresh:"让从臂保持稳定支撑，只调整 Leader 到表中目标角度。该目标按原配对校准逆算，不修改校准；启动会重新采样并检查。",
  alignmentCurrentPoseMissing:"请先连接只读状态。只有姿态新鲜、在限位内、夹爪闭合且 Leader 可达时，才显示当前姿态对齐参考。",
  alignmentCurrentPoseNote:"从臂当前位置就是本次对齐参考。Follower 还需转动为 0；请调整 Leader，勿为对齐而继续手托 J4。启动前要求两臂差不超过单次目标步长（最多 2°）。",
  leaderCurrentNote:"Leader 目标为舵机原始角度（°），按原关节映射、方向、增益和零位逆算。当前显示仅供对齐，未请求从臂运动或保持。",
  alignmentCurrentPoseColumns:["轴","当前 SDK °","当前参考 SDK °","从臂还需转动 °","两臂允许差 °","Leader ID","Leader 目标舵机 °"],
  currentPoseLegend:"青色半透明：从臂当前对齐参考", showCurrentAlignmentTarget:"显示当前对齐参考",
  alignmentCurrentPoseRunning:"当前会话采用当前位置对齐启动。表格为当前姿态的逆映射显示，启动参考已固定；当前由 Leader 控制跟随。"
});
Object.assign(copy.zh, {
  startupRelative:"从当前位置相对起步（工程测试）",
  startupRelativeNote:"让从臂处于稳定、有支撑的位置并闭合夹爪，主臂放在方便操作的稳定姿态。启动时记录两端位置为本次起点，之后按主臂的角度变化跟随，无需把从臂搬到零位或将主臂调到绝对对应角。原方向、比例与限位保留；起点仅本次有效。使能时仍可能短暂移动，保持效果尚未真机验证。",
  alignmentRelativeFresh:"当前姿态可作为相对起步的显示参考。两端分别放稳即可，启动会重新采样固定本次起点；本页没有使能或保持从臂。",
  alignmentRelativeMissing:"请连接只读状态，并确认从臂在限位内、J2 符合启动条件且夹爪闭合。相对起步参考不需要反算到 Leader 的绝对可达角。",
  alignmentRelativeNote:"相对起步无需把两端搬到同一绝对姿态。下表 Leader home 仅作原校准参考；程序在启动时捕获两端起点，再跟随角度增量。",
  alignmentRelativeRunning:"当前会话采用相对起步。本次两端起点已经固定；Leader 的角度变化控制从臂，原校准未改写。",
  alignmentRelativeColumns:["轴","当前 SDK °","当前姿态显示参考 SDK °","从臂还需转动 °","启动检查允许漂移 °","Leader ID","原 Leader home °（仅参考）"]
});
Object.assign(copy.zh, {
  startupLeaderTarget:"直接跟随 Leader（限速接近）",
  startupLeaderTargetNote:"无需启动对齐、回零或预先闭合夹爪。启动会自动闭合从臂夹爪，记录从臂真实姿态作为限速起点，随后逐步接近 Leader 按原校准映射的绝对目标。原校准不变；CAN、故障、温度、关节限位、SDK 启动条件与速率保护保留。基础跟随/监测不输出 Leader 阻尼；实验阻尼需单独启动，采用当前功率上限、无自动 HOLD。正式力反馈尚未就绪；效果尚未真机验证。",
  alignmentLeaderTarget:"无需对齐；启动后限速接近 Leader 目标。当前没有新鲜的 Leader 目标，不显示校准零位或对齐目标。",
  alignmentLeaderTargetNote:"从臂真实姿态只作为限速起点，跟随目标来自 Leader 的绝对位置。",
  leaderTargetCalibrationNote:"沿用原配对校准的关节对应、方向、比例与零位，不建立本次相对偏移。"
});
Object.assign(copy.en, {
  startupPoseTitle:"Startup mode", startupHome:"Calibrated home (default)", startupCurrent:"Align at current pose (engineering)",
  startupPoseNote:"Current-pose mode: place the Follower in a stable, supported pose with the gripper closed, then connect read-only state. Adjust the Leader to the angles below, stop read-only state, then start basic following. J4 need not be held at zero. Brief movement during enabling remains possible; holding has not been verified on hardware.",
  alignmentCurrentPoseFresh:"Keep the Follower stably supported and adjust only the Leader to the target angles. These are inverted from the original paired calibration without changing it. Startup resamples and checks both arms.",
  alignmentCurrentPoseMissing:"Connect read-only state first. A current-pose reference requires fresh feedback, legal joint limits, a closed gripper and a reachable Leader pose.",
  alignmentCurrentPoseNote:"The current Follower pose is the alignment reference. Its remaining turn is zero; adjust the Leader instead of holding J4 at home. Startup requires the pair difference to fit one target step (at most 2°).",
  leaderCurrentNote:"Leader targets are raw servo degrees, inverted through the original joint mapping, signs, gain and homes. This display requests no Follower movement or holding.",
  alignmentCurrentPoseColumns:["Axis","Current SDK °","Current reference SDK °","Follower turn °","Allowed pair difference °","Leader ID","Leader target servo °"],
  currentPoseLegend:"Translucent cyan: current Follower alignment reference", showCurrentAlignmentTarget:"Show current alignment reference",
  alignmentCurrentPoseRunning:"This session started with current-pose alignment. The table inverts the live pose for display; the startup reference was fixed. The Leader now controls following."
});
Object.assign(copy.en, {
  startupRelative:"Relative start at current pose (engineering)",
  startupRelativeNote:"Rest the Follower in a stable, supported pose with its gripper closed, and place the Leader in a stable, comfortable operating pose. Startup captures both origins, then follows Leader angle changes. Neither arm needs to be moved to home or an absolute matching pose. Original signs, gain and limits remain; the origin lasts for this session only. Brief enabling movement remains possible; holding has not been physically verified.",
  alignmentRelativeFresh:"This pose is available as a relative-start display reference. Rest both arms stably; startup resamples and fixes their origins. This display has not enabled or held the Follower.",
  alignmentRelativeMissing:"Connect read-only state and check joint limits, the J2 startup condition and a closed gripper. Relative startup does not require an inverse absolute Leader angle to be reachable.",
  alignmentRelativeNote:"Relative startup does not require matching absolute poses. Leader homes below are original calibration references only. Startup captures both origins and then follows angle changes.",
  alignmentRelativeRunning:"This session uses relative startup with fixed origins. Leader angle changes control the Follower; the original calibration was not rewritten.",
  alignmentRelativeColumns:["Axis","Current SDK °","Live pose display reference SDK °","Follower turn °","Startup check drift tolerance °","Leader ID","Original Leader home ° (reference)"]
});
Object.assign(copy.en, {
  startupLeaderTarget:"Follow Leader directly (approach with rate limits)",
  startupLeaderTargetNote:"No startup alignment, homing or pre-closing the gripper is required. Startup automatically closes the Follower gripper and records the measured Follower pose as the starting point for rate limits, then gradually approaches the absolute Leader target mapped through the original calibration. Calibration is unchanged. CAN, fault, temperature, joint-limit, SDK startup and rate protections remain. For basic following and monitoring, Leader force output stays disabled. Experimental damping requires a separate start, uses current power caps and has no automatic HOLD. Formal force feedback is not ready. Hardware performance is unverified.",
  alignmentLeaderTarget:"No alignment is required; startup approaches the Leader target with rate limits. No fresh Leader target is available here, so no calibrated-home or alignment target is shown.",
  alignmentLeaderTargetNote:"The measured Follower pose only anchors the approach with rate limits. The following target comes from the Leader's absolute position.",
  leaderTargetCalibrationNote:"Original joint mapping, directions, gain and homes remain. No session-relative offset is created."
});
Object.assign(copy.zh, {
  experimentDetails:"实验说明", connectionSettings:"启动设置", controlEyebrow:"控制台", modelEyebrow:"实时三维", outputEyebrow:"阻尼输出", followEyebrow:"跟随诊断",
  displaySmoothing:"平滑姿态显示", previewReadOnly:"新版只读预览 · 当前控制会话保持运行", powerDraft:"有未应用的修改"
});
Object.assign(copy.en, {
  experimentDetails:"About this experiment", connectionSettings:"Startup settings", controlEyebrow:"CONTROL", modelEyebrow:"LIVE VIEW", outputEyebrow:"DAMPING OUTPUT", followEyebrow:"FOLLOWING",
  displaySmoothing:"Smoothed pose display", previewReadOnly:"New interface · Read-only preview of the running session", powerDraft:"Unapplied changes"
});
Object.assign(copy.zh, {
  baselineRecaptureTitle:"当前姿态空载基线", baselineRecaptureAction:"重新采集空载基线", baselineRecaptureSubmitting:"正在提交",
  baselineRecaptureConfirm:"我确认 Follower 当前空载 且未接触桌面或其他物体",
  baselineRecaptureNote:"保持跟随 不退出电机控制 不重放按钮指令。仅在你明确确认空载后重新采集，禁止在受力或接触时使用。完成后功率上限仍为 0，需你另行恢复。新基线仍只覆盖采样姿态附近，不代表全行程或真机手感已验证。",
  baselineRecaptureReady:"确认当前空载后可重新采集 连续静止至少 2 秒",
  baselineRecaptureCaps:"请先将七轴普通阻尼上限设为 0 并应用 再确认当前空载",
  baselineRecaptureInactive:"仅在正在运行的实验阻尼会话中可用",
  baselineRecaptureUnavailable:"当前条件不满足 请等待完整真实按钮释放及新鲜遥测",
  baselineRecaptureOffline:"连接或遥测未更新 暂不可重新采集",
  baselineRecapturePending:"重新采集请求已排队 等待控制循环核对",
  baselineRecaptureCollecting:"正在零阻尼输出下采集 等待跟随到位并连续静止",
  baselineRecaptureComplete:"本次空载基线已冻结 请先检查空载状态再手动恢复功率上限",
  baselineRecaptureRejected:"本次重新采集未完成 请检查当前会话和基线状态",
  baselineRecaptureLastReport:"最后报告 当前未更新",
  baselineRecaptureFailed:"重新采集请求失败",
  active_experiment_required:"需要正在运行的实验阻尼会话", sdk_history_required:"SDK 实测历史尚未就绪 暂不可重新采集",
  native_release_required:"需要当前完整真实按钮释放状态", zero_caps_required:"需要七轴普通阻尼有效上限全部为 0",
  recapture_pending:"重新采集请求已排队 请等待", recapture_collecting:"正在重新采集 请等待",
  session_stopped:"会话已停止 重新采集已取消"
});
Object.assign(copy.en, {
  baselineRecaptureTitle:"Unloaded baseline at the current pose", baselineRecaptureAction:"Recapture unloaded baseline", baselineRecaptureSubmitting:"Submitting",
  baselineRecaptureConfirm:"I confirm the Follower is unloaded and has no contact with the table or other objects",
  baselineRecaptureNote:"Following continues without disabling motors or replaying button commands. Recapture only after explicitly confirming the unloaded state, never under load or contact. Caps remain at 0 afterward and require a separate manual change. The new baseline still covers only the sampled pose neighborhood, without full-range or hardware-feel validation.",
  baselineRecaptureReady:"Confirm the current unloaded state, then remain stationary for at least 2 seconds",
  baselineRecaptureCaps:"Set all seven ordinary damping axis caps to 0 and apply them, then confirm the unloaded state",
  baselineRecaptureInactive:"Available only in an active experimental-damping session",
  baselineRecaptureUnavailable:"Conditions are not met; wait for a complete real button release and fresh telemetry",
  baselineRecaptureOffline:"Connection or telemetry is not updating; recapture is unavailable",
  baselineRecapturePending:"Recapture request queued; awaiting control-loop checks",
  baselineRecaptureCollecting:"Collecting with zero damping output; waiting for target arrival and continuous stationarity",
  baselineRecaptureComplete:"This unloaded baseline is frozen; check the unloaded state before manually restoring caps",
  baselineRecaptureRejected:"Recapture did not complete; check the current session and baseline state",
  baselineRecaptureLastReport:"Last report; updates stopped",
  baselineRecaptureFailed:"Baseline recapture request failed",
  active_experiment_required:"An active experimental-damping session is required", sdk_history_required:"SDK measurement history is not ready for recapture",
  native_release_required:"A current complete real button-release state is required", zero_caps_required:"All seven effective ordinary damping caps must be 0",
  recapture_pending:"A recapture request is already queued; please wait", recapture_collecting:"Recapture is in progress; please wait",
  session_stopped:"The session stopped; recapture was cancelled"
});
let state = null, dirty = false, pageConnected = false;
let modelDefinition = null, solidViewer = null, viewerFailed = false, modelFailed = false;
let showAlignmentTarget = true;
let stateReceivedAt = 0, freshnessTimer = null, connectionRequestPending = false;
let connectionRequestError = null, connectionRequestNotice = "";
let baselineRecaptureRequestPending = false, baselineRecaptureRequestError = null;
let buttonBoardRequestPending = false, buttonBoardRequestError = null, buttonBoardRequestNotice = "";
let selectedStartupPose = validStartupPose(readPreference("lumos-startup-pose"));
let startupPose = selectedStartupPose, pendingStartupPose = null;
const $ = id => document.getElementById(id);
const name = i => i === 6 ? copy[lang].gripper : `J${i+1}`;
const fmt = n => Number.isFinite(n) ? n.toFixed(2) : "—";
const nowMs = () => typeof performance !== "undefined" ? performance.now() : Date.now();
const responseGraceMs = () => Math.max(1000, (state?.freshness_limit_s ?? 0) * 1000);
const telemetryFresh = () => Boolean(pageConnected && ["inspect","follow","monitor","run","experiment"].includes(state?.mode) && ["active","inspection_active","leader_gap"].includes(state?.status) && state?.telemetry_fresh === true && nowMs() - stateReceivedAt < responseGraceMs());
const capsWritable = () => Boolean(pageConnected && state && !["inspect","preview"].includes(state.mode) && state.caps_writable !== false && (!["active","leader_gap"].includes(state.status) || telemetryFresh()));
async function timedFetch(path, options={}, timeoutMs=3000) {
  if(typeof AbortController === "undefined")return fetch(path,options);
  const controller=new AbortController(), timeout=setTimeout(()=>controller.abort(),timeoutMs);
  try{return await fetch(path,{...options,signal:controller.signal});}
  finally{clearTimeout(timeout);}
}
function armFreshnessExpiry() {
  if(freshnessTimer!==null&&typeof clearTimeout!=="undefined")clearTimeout(freshnessTimer);
  freshnessTimer=state?.telemetry_fresh===true?setTimeout(()=>render(),Math.max(1,responseGraceMs()-(nowMs()-stateReceivedAt)+1)):null;
}
function motorState(value, c) {
  if (value === 0) return c.motorDisabled;
  if (value === 1) return c.motorEnabled;
  return Number.isInteger(value) ? `${c.motorFault} ${value}` : "—";
}
function leaderState(power, hold, c, axis) {
  if (state?.mode === "experiment" && axis < 6 && (state.leader_output_state === "unconfirmed_native_transition" || state.experimental?.state === "waiting_native_unlock")) return c.nativeLeaderState;
  if (state?.mode === "experiment" && axis === 6 && telemetryFresh() && Number.isFinite(power) && hold != null) return hold ? c.hold : (power > 0 ? c.damping : c.free);
  if (!telemetryFresh() || state?.mode === "inspect" || state?.leader_controlled === false || !Number.isFinite(power) || hold == null) return c.unknown;
  return hold ? c.hold : (power > 0 ? c.damping : c.free);
}
// Stable rows keep selection/focus intact and avoid rebuilding hundreds of nodes
// on every telemetry poll. Numeric refreshes intentionally do not replay entry motion.
function cells(parent, values, tag="td") {
  if(parent.children.length!==values.length || parent._cellTag!==tag){
    parent.replaceChildren(...values.map(()=>document.createElement(tag)));
    parent._cellTag=tag;
  }
  values.forEach((value,i)=>{
    const text=String(value??"—"), cell=parent.children[i];
    if(cell.textContent!==text)cell.textContent=text;
  });
}
function tableRows(parent, values, decorate) {
  if(parent.children.length!==values.length)parent.replaceChildren(...values.map(()=>document.createElement("tr")));
  values.forEach((row,i)=>{cells(parent.children[i],row);decorate?.(parent.children[i],i);});
}
function setVisible(element, visible) {
  if(!element)return;
  if(element.hidden===!visible)return;
  // Visibility is applied synchronously. An outgoing CSS transition never leaves
  // a hidden control interactive or postpones a protection/error update.
  element.inert=!visible;
  element.hidden=!visible;
}
function motionAllowed() {
  return typeof window!=="undefined" && typeof window.matchMedia==="function" && !window.matchMedia("(prefers-reduced-motion: reduce)").matches;
}
function changeLanguage() {
  lang=lang==="zh"?"en":"zh";writePreference("lumos-language",lang);
  if(motionAllowed()&&typeof document.startViewTransition==="function")document.startViewTransition(()=>translate());
  else translate();
}
function translate() {
  document.documentElement.lang=lang;
  document.querySelectorAll("[data-i18n]").forEach(el => el.textContent=copy[lang][el.dataset.i18n]);
  $("language").textContent=lang === "zh" ? "English" : "中文";
  if(!$("cap0"))$("axisCaps").replaceChildren(...Array.from({length:7}, (_, i) => {
    const label=document.createElement("label"), title=document.createElement("span"), input=document.createElement("input");
    title.id=`capLabel${i}`;title.textContent=name(i);label.append(title); input.id=`cap${i}`; input.type="number"; input.min="0"; input.max="1500"; input.step="1"; input.required=true;
    input.value=state?.caps.axis_mw[i] ?? 1200; input.disabled=!capsWritable(); label.append(input, " mW"); return label;
  }));
  Array.from({length:7},(_,i)=>{if($("capLabel"+i))$("capLabel"+i).textContent=name(i);});
  if(!$("columns").children.length)$("columns").append(document.createElement("tr"));
  cells($("columns").children[0],copy[lang].columns,"th");
  cells($("policyHead"), [copy[lang].axis, ...Array.from({length:7}, (_,i)=>name(i))], "th");
  if(dirty)$("result").textContent=copy[lang].powerDraft;
  render();
}
function sdkRangeDetails(items) {
  if(!Array.isArray(items))return "";
  return items.map(x=>`J${x.axis} ${x.parameter}: SDK ±${x.sdk} / ${lang==="zh"?"实机":"hardware"} ±${x.hardware}${x.parameter==="v_max"?" rad/s":x.parameter==="p_max"?" rad":" Nm"}`).join("; ");
}
function startupStrategyLabel(strategy) {
  const c=copy[lang];
  return strategy==="leader_target"?c.startupLeaderTarget:strategy==="current_relative"?c.startupRelative:strategy==="current_aligned"?c.startupCurrent:c.startupHome;
}
function startupMismatchDetails(diagnostic) {
  if(!["home","current_aligned","current_relative","leader_target"].includes(diagnostic?.strategy))return "";
  if(diagnostic.reason==="gripper_not_closed"&&Number.isFinite(diagnostic.opening)&&Number.isFinite(diagnostic.maximum_opening))return lang==="zh"?`从臂夹爪开度 ${fmt(diagnostic.opening*100)}%，启动要求 ≤${fmt(diagnostic.maximum_opening*100)}%。请先闭合从臂夹爪；SDK 启动时会自动闭合它。无需为这个错误调整六轴姿态。`:`Follower gripper opening is ${fmt(diagnostic.opening*100)}%; startup requires ≤${fmt(diagnostic.maximum_opening*100)}%. Close the Follower gripper first: the SDK closes it during startup. This error does not require adjusting the six arm joints.`;
  if(["joint2_startup","joint2_target_startup"].includes(diagnostic.reason)&&Number.isFinite(diagnostic.position_rad)&&Number.isFinite(diagnostic.minimum_rad)){
    const target=diagnostic.reason==="joint2_target_startup";
    return lang==="zh"?`${target?"Leader 映射目标":"从臂"} J2 为 ${fmt(diagnostic.position_rad*180/Math.PI)}°，SDK 启动要求 ≥${fmt(diagnostic.minimum_rad*180/Math.PI)}°。`:`${target?"Mapped Leader target":"Follower"} J2 is ${fmt(diagnostic.position_rad*180/Math.PI)}°; SDK startup requires ≥${fmt(diagnostic.minimum_rad*180/Math.PI)}°.`;
  }
  if(!Array.isArray(diagnostic.axes))return "";
  const labels=lang==="zh"?{
    follower_home:"从臂距校准零位",leader_home:"Leader 目标距校准零位",pair:"两端差",
    follower_limit:"从臂位置超出限位",target_limit:"Leader 映射目标超出限位",actual:"实际",allowed:"姿态差允许",range:"限位",measurementMargin:"测量余量"
  }:{
    follower_home:"Follower from calibrated home",leader_home:"Leader target from calibrated home",pair:"pair difference",
    follower_limit:"Follower position outside limits",target_limit:"mapped Leader target outside limits",actual:"actual",allowed:"pose difference allowed",range:"limits",measurementMargin:"measurement margin"
  };
  const fields={follower_home:"follower_home_error_rad",leader_home:"leader_home_error_rad",pair:"pair_error_rad"};
  const deg=value=>`${fmt(value*180/Math.PI)}°`;
  const preciseDeg=value=>`${Number((value*180/Math.PI).toPrecision(10))}°`;
  const rows=diagnostic.axes.filter(a=>Number.isInteger(a.axis)&&a.axis>=1&&a.axis<=6&&Array.isArray(a.failed_checks)&&Number.isFinite(a.tolerance_rad)).map(a=>{
    const values=a.failed_checks.flatMap(check=>{
      if(fields[check]&&Number.isFinite(a[fields[check]])){
        const tolerance=check==="pair"&&Number.isFinite(a.pair_tolerance_rad)?a.pair_tolerance_rad:a.tolerance_rad;
        return [`${labels[check]} ${deg(a[fields[check]])} (${labels.allowed} ${deg(tolerance)})`];
      }
      if(["follower_limit","target_limit"].includes(check)&&Array.isArray(a.limits_rad)&&a.limits_rad.length===2&&a.limits_rad.every(Number.isFinite)){
        const position=a[check==="follower_limit"?"follower_rad":"target_rad"];
        const actual=Number.isFinite(position)?`${labels.actual} ${preciseDeg(position)}; `:"";
        const margin=check==="follower_limit"&&Number.isFinite(a.measured_limit_margin_rad)?`; ${labels.measurementMargin} ±${preciseDeg(a.measured_limit_margin_rad)}`:"";
        return [`${labels[check]} (${actual}${labels.range} ${a.limits_rad.map(preciseDeg).join(" … ")}${margin})`];
      }
      return [];
    });
    return values.length?`J${a.axis}: ${values.join(lang==="zh"?"，":", ")}`:"";
  }).filter(Boolean);
  return rows.length?`${startupStrategyLabel(diagnostic.strategy)}: ${rows.join("; ")}`:"";
}
function errorText(source) {
  if(!source?.error)return "";
  const code=source.error_code??source.code, message=copy[lang][code];
  if(!message)return source.error;
  const text=message.replace("{interface}",source.error_interface??"CAN");
  const detail=source.error_detail??source.detail;
  if(code==="startup_pose_mismatch"){
    const diagnostics=startupMismatchDetails(source.error_diagnostics);
    if(diagnostics)return `${text} ${diagnostics}`;
  }
  if(code==="sdk_motor_range_mismatch"&&detail)return `${text} (${sdkRangeDetails(detail.mismatches)})`;
  if(code==="sdk_compatibility_failed")return `${text} (${source.error})`;
  if(code==="button_board_no_reply"&&detail)return `${text} (ID ${detail.button_id}, ${detail.attempts} ${lang==="zh"?"次查询":"queries"})`;
  if(["gripper_motor_range","gripper_motor_velocity"].includes(code)&&detail){
    const velocity=code==="gripper_motor_velocity", value=velocity?detail.raw_velocity_rad_s:detail.raw_position_rad, low=velocity?detail.allowed_min_rad_s:detail.allowed_min_rad, high=velocity?detail.allowed_max_rad_s:detail.allowed_max_rad;
    const fmt=x=>typeof x==="number"&&Number.isFinite(x)?x.toFixed(6):"—";
    return `${text} (${lang==="zh"?"实际":"measured"}: ${fmt(value)}; ${lang==="zh"?"允许":"allowed"}: ${fmt(low)} … ${fmt(high)} ${velocity?"rad/s":"rad"})`;
  }
  return ["can_query_failed","can_bitrate_mismatch","can_bus_fault","startup_pose_mismatch","sdk_cleanup_failed"].includes(code)&&source.error_detail?`${text} (${source.error_detail})`:text;
}
const availableModes = connection => Array.isArray(connection?.available_modes) ? connection.available_modes : ["inspect"];
const modeCanStart = (connection, mode) => connection?.can_start === true && availableModes(connection).includes(mode) && !connection.blocked_modes?.[mode];
const feedbackStartMode = connection => availableModes(connection).includes("experiment") ? "experiment" : "run";
function renderBaselineAdmission() {
  const panel=$("baselineAdmissionPanel"), body=$("baselineAdmissionRows");if(!panel||!body)return;
  const admission=state?.baseline_admission, c=copy[lang];
  const waiting=state?.mode==="experiment"&&["waiting_baseline","partially_active"].includes(state?.experimental?.state)&&state.experimental.baseline_frozen===false&&telemetryFresh()&&admission?.allowed===false;
  const value=(key,i)=>Array.isArray(admission?.[key])?admission[key][i]:null;
  const fields=["target","sent_target","measured_position","target_error","target_tolerance","arrival_error","arrival_tolerance"];
  const blocked=i=>Array.isArray(admission?.blocked_axes)&&(admission.blocked_axes.includes(i)||admission.blocked_axes.includes(value("axis_names",i)));
  const visible=Array.from({length:7},(_,i)=>waiting&&state.experimental?.baseline_ready_axes?.[i]!==true&&blocked(i)&&value("axis_names",i)===(i<6?`J${i+1}`:"gripper")&&value("axis_units",i)===(i<6?"rad":"normalized_opening")&&fields.every(key=>Number.isFinite(value(key,i)))&&value("target_tolerance",i)>=0&&value("arrival_tolerance",i)>=0);
  setVisible(panel,visible.some(Boolean));
  cells($("baselineAdmissionColumns"),c.baselineAdmissionColumns,"th");
  // Keep one row per axis even when its gate clears so polling does not replace
  // the remaining diagnostics or restart their entry transitions.
  tableRows(body,Array.from({length:7},(_,i)=>{
    const factor=i<6?180/Math.PI:100, formatted=key=>fmt(Number.isFinite(value(key,i))?value(key,i)*factor:null);
    return [name(i),formatted("target"),formatted("sent_target"),formatted("measured_position"),`${formatted("target_error")} / ${formatted("target_tolerance")}`,`${formatted("arrival_error")} / ${formatted("arrival_tolerance")}`];
  }),(row,i)=>setVisible(row,visible[i]));
}
function renderExperimental() {
  const panel=$("experimentalPanel");if(!panel)return;
  const experimental=state?.experimental, c=copy[lang];
  setVisible(panel,Boolean(experimental||availableModes(state?.connection).includes("experiment")||state?.mode==="experiment"));
  renderBaselineAdmission();
  renderLocalReference();
  renderBaselineRecapture();
  if(panel.hidden)return;
  const status=typeof experimental?.state==="string"?c["experiment_"+experimental.state]??experimental.state:"—";
  const frozen=typeof experimental?.baseline_frozen==="boolean"?(experimental.baseline_frozen?c.yes:c.no):"—";
  const label=experimental&&!telemetryFresh()?c.experimentLastReport:c.experimentState;
  const ceiling=experimental?.ceiling_mw;
  const ceilingText=ceiling===null?c.experimentNoExtraCap:Number.isFinite(ceiling)?`${fmt(ceiling)} mW`:"—";
  $("experimentalStatus").textContent=`${label}: ${status} · ${c.experimentCeiling}: ${ceilingText} · ${c.experimentFrozen}: ${frozen}`;
  if($("experimentalBaselines")){
    const groups=experimental?.baseline_groups;
    setVisible($("experimentalBaselines"),Boolean(groups));
    $("experimentalBaselines").textContent=groups?[['arm',c.baselineArm],['gripper',c.baselineGripper]].map(([key,title])=>{
      const group=groups[key], detail=group?.frozen===true?c.baselineGroupFrozen:experimental.baseline_collection_closed===true?c.baselineGroupUnavailable:c.baselineGroupWaiting;
      return `${title}: ${detail}`;
    }).join(" · "):"";
  }
  if($("experimentalCaps")){
    const reported=experimental?.effective_caps_mw;
    const caps=Array.from({length:7},(_,i)=>{
      const common=state?.caps?.common_mw, axis=state?.caps?.axis_mw?.[i];
      if(Array.isArray(reported)&&Number.isFinite(reported[i]))return reported[i];
      if(!Number.isFinite(common)||!Number.isFinite(axis)||!(ceiling===null||Number.isFinite(ceiling)))return null;
      return Math.min(common,axis,...(Number.isFinite(ceiling)?[ceiling]:[]));
    });
    $("experimentalCaps").textContent=`${c.experimentEffectiveCaps}: ${caps.map((cap,i)=>`${name(i)} ${fmt(cap)}`).join(" · ")} mW`;
  }
  if($("experimentalRise")){
    const reported=experimental?.effective_rise_mw_s;
    const rates=Array.from({length:7},(_,i)=>Array.isArray(reported)&&Number.isFinite(reported[i])?reported[i]:null);
    $("experimentalRise").textContent=`${c.experimentEffectiveRise}: ${rates.map((rate,i)=>`${name(i)} ${fmt(rate)}`).join(" · ")} mW/s`;
  }
  const reason=experimental?.reason;
  $("experimentalReason").textContent=typeof reason==="string"?c[`experiment_reason_${reason}`]??reason:"";
}
function renderLocalReference() {
  const panel=$("localReferencePanel");if(!panel)return;
  const local=state?.experimental?.local_reference, c=copy[lang], fresh=telemetryFresh()&&state?.mode==="experiment";
  setVisible(panel,Boolean(local));
  if(!local)return;
  const detail=local.enabled===false?c.localReferenceDisabled:local.enabled!==true?c.localReferenceUnknown:!fresh?`${c.localReferenceEnabled} · ${c.localReferenceUnknown}`:local.available!==true?c.localReferenceWaiting:local.in_region===false?c.localReferenceOutside:local.in_region!==true?c.localReferenceUnknown:local.reason==="warming_local_reference"?c.localReferenceWarming:local.eligible!==true?c.localReferencePaused:local.active===true?c.localReferenceActive:c.localReferenceEligible;
  $("localReferenceStatus").textContent=`${fresh?c.localReferenceTitle:c.localReferenceLastReport}: ${detail}`;
  const reason=local.reason;
  $("localReferenceReason").textContent=fresh&&typeof reason==="string"?c[`local_reference_reason_${reason}`]??c[`experiment_reason_${reason}`]??c[`experiment_${reason}`]??reason:"";
  const table=$("localReferenceTable"), visible=fresh&&local.enabled===true&&local.available===true;
  setVisible(table,visible);
  if(!visible)return;
  cells($("localReferenceColumns"),c.localReferenceColumns,"th");
  tableRows($("localReferenceRows"),Array.from({length:6},(_,i)=>{
    const source=local.source?.[i], selected=local.eligible===true&&local.used_axes?.[i]===true;
    const sourceText=selected?c.localReferenceLocal:source==="absolute"?c.localReferenceAbsolute:source==="none"?c.localReferenceNone:c.localReferenceUnavailable;
    return [name(i),sourceText,fmt(local.eligible===true?local.excess_nm?.[i]:null),fmt(state.requested_mw?.[i])];
  }));
}
function baselineRecaptureConditions() {
  const connection=state?.connection, reported=state?.experimental?.effective_caps_mw;
  const active=connection?.state==="active"&&connection.mode==="experiment"&&state?.mode==="experiment"&&state.status==="active";
  const zeroCaps=Array.isArray(reported)&&reported.length===7&&reported.every(cap=>Number.isFinite(cap)&&cap===0);
  const recapture=connection?.baseline_recapture??state?.baseline_recapture;
  const busy=baselineRecaptureRequestPending||["pending","collecting"].includes(recapture?.state);
  return {active,zeroCaps,recapture,busy,allowed:Boolean(active&&zeroCaps&&telemetryFresh()&&connection.can_recapture_baseline===true&&!busy&&!connectionRequestPending&&!buttonBoardRequestPending)};
}
function renderBaselineRecapture() {
  const panel=$("baselineRecapturePanel"), checkbox=$("baselineRecaptureUnloaded"), button=$("recaptureBaseline");
  if(!panel||!checkbox||!button)return;
  const c=copy[lang], {active,zeroCaps,recapture,busy,allowed}=baselineRecaptureConditions();
  checkbox.disabled=!allowed;
  if(!allowed)checkbox.checked=false;
  button.disabled=!allowed||checkbox.checked!==true;
  $("baselineRecaptureButtonLabel").textContent=baselineRecaptureRequestPending?c.baselineRecaptureSubmitting:c.baselineRecaptureAction;
  const statusKey={pending:"baselineRecapturePending",collecting:"baselineRecaptureCollecting",complete:"baselineRecaptureComplete",rejected:"baselineRecaptureRejected"}[recapture?.state];
  const detail=typeof recapture?.reason==="string"?c[recapture.reason.replace(/^recapture_aborted_/,"")]:"";
  const status=statusKey?`${c[statusKey]}${recapture?.state==="rejected"&&detail?` · ${detail}`:""}`:"";
  $("baselineRecaptureStatus").textContent=status?(telemetryFresh()?status:`${c.baselineRecaptureLastReport} · ${status}`):"";
  $("baselineRecaptureHelp").textContent=!pageConnected?c.baselineRecaptureOffline:!active?c.baselineRecaptureInactive:!telemetryFresh()?c.baselineRecaptureOffline:!zeroCaps?c.baselineRecaptureCaps:busy?(recapture?.state==="collecting"?c.baselineRecaptureCollecting:c.baselineRecapturePending):!allowed?c.baselineRecaptureUnavailable:c.baselineRecaptureReady;
  $("baselineRecaptureResult").textContent=errorText(baselineRecaptureRequestError);
}
async function requestBaselineRecapture() {
  const checkbox=$("baselineRecaptureUnloaded");
  if(!baselineRecaptureConditions().allowed||checkbox?.checked!==true){renderBaselineRecapture();return;}
  baselineRecaptureRequestPending=true;baselineRecaptureRequestError=null;checkbox.checked=false;renderBaselineRecapture();
  try{
    const response=await timedFetch("/api/experimental-baseline",{method:"POST",headers:{"Content-Type":"application/json","X-Feedback-Token":state.token??""},body:JSON.stringify({unloaded_confirmed:true})});
    const value=await response.json();
    if(!response.ok){baselineRecaptureRequestError={...value,error:value.error??copy[lang].baselineRecaptureFailed};return;}
    state.connection=value.connection??value;
  }catch(error){baselineRecaptureRequestError={error:error.message};}
  finally{baselineRecaptureRequestPending=false;renderBaselineRecapture();}
}
function renderFollowDiagnostics() {
  if(!$("followRows"))return;
  const c=copy[lang], diagnostics=state?.follow_diagnostics, timing=state?.loop_timing;
  const value=(key,i)=>Array.isArray(diagnostics?.[key])?diagnostics[key][i]:null;
  const angle=(key,i)=>Number.isFinite(value(key,i))?fmt(value(key,i)*180/Math.PI):"—";
  const rows=[], rawRows=[];
  cells($("followColumns"),c.followColumns,"th");cells($("followRawColumns"),c.followRawColumns,"th");
  for(let i=0;i<6;i++){
    const flags=["leader_range_clipped","joint_limit_clipped","rate_limited"].map(key=>value(key,i));
    const limits=[c.followLeaderClipped,c.followJointClipped,c.followRateLimited].filter((_,index)=>flags[index]===true);
    const status=limits.length?limits.join(" · "):flags.every(flag=>flag===false)?c.followUnrestricted:"—";
    const sent=value("sent_sdk_rad",i), actual=value("actual_sdk_rad",i);
    rows.push([name(i),angle("mapped_sdk_rad",i),angle("limit_clipped_sdk_rad",i),angle("sent_sdk_rad",i),angle("actual_sdk_rad",i),Number.isFinite(sent)&&Number.isFinite(actual)?fmt((sent-actual)*180/Math.PI):"—",status]);
    rawRows.push([name(i),fmt(value("raw_leader_deg",i)),fmt(value("filtered_leader_deg",i)),fmt(value("leader_clipped_deg",i)),angle("requested_sdk_rad",i)]);
  }
  tableRows($("followRows"),rows,(row,i)=>{row.dataset.limited=String(["leader_range_clipped","joint_limit_clipped","rate_limited"].some(key=>value(key,i)===true));});
  tableRows($("followRawRows"),rawRows);
  $("followFreshness").textContent=!diagnostics?c.followMissing:telemetryFresh()?c.followLive:c.followStopped;
  $("loopTiming").textContent=`${c.loopTarget}: ${fmt(timing?.target_hz)} Hz · ${c.loopStatus}: ${fmt(timing?.status_hz)} Hz · ${c.loopActual}: ${fmt(timing?.actual_hz)} Hz · ${c.loopOverruns}: ${Number.isFinite(timing?.overruns)?timing.overruns:"—"}`;
}
function renderButtonBoard() {
  const panel=$("buttonBoardPanel"), board=state?.connection?.button_board, c=copy[lang];
  if(!panel)return;
  setVisible(panel,Boolean(board&&typeof board.enabled==="boolean"));
  if(panel.hidden)return;
  const writable=pageConnected&&board.can_change===true&&!connectionRequestPending&&!buttonBoardRequestPending;
  for(const [id,enabled] of [["buttonBoardEnabled",true],["buttonBoardDisabled",false]]){
    const input=$(id);if(input){input.checked=board.enabled===enabled;input.disabled=!writable;}
  }
  const verification=!board.enabled?c.buttonBoardNotRead:board.verified===true?c.buttonBoardVerified:board.verified===false?c.buttonBoardNoReply:c.buttonBoardUnverified;
  if($("buttonBoardStatus"))$("buttonBoardStatus").textContent=`${c.buttonBoardConfiguration}: ${board.enabled?c.buttonBoardEnabled:c.buttonBoardDisabled} · ID ${Number.isInteger(board.id)?board.id:"—"} · ${c.buttonBoardVerification}: ${verification}`;
  if($("buttonBoardLocked"))setVisible($("buttonBoardLocked"),board.can_change!==true);
  if($("buttonBoardResult"))$("buttonBoardResult").textContent=errorText(buttonBoardRequestError)||(buttonBoardRequestNotice?c[buttonBoardRequestNotice]:"");
}
async function requestButtonBoard(enabled) {
  const board=state?.connection?.button_board;
  if(typeof enabled!=="boolean"||!pageConnected||!board||board.can_change!==true||connectionRequestPending||buttonBoardRequestPending){renderButtonBoard();return;}
  if(board.enabled===enabled){renderButtonBoard();return;}
  buttonBoardRequestPending=true;buttonBoardRequestError=null;buttonBoardRequestNotice="";renderConnection();
  try{
    const response=await timedFetch("/api/button-board",{method:"POST",headers:{"Content-Type":"application/json","X-Feedback-Token":state.token??""},body:JSON.stringify({enabled})});
    const value=await response.json();
    if(!response.ok){buttonBoardRequestError={...value,error:value.error??copy[lang].disconnected};return;}
    state.connection=value.connection??value;
    buttonBoardRequestNotice="buttonBoardSaved";
  }catch(error){buttonBoardRequestError={error:error.message};}
  finally{buttonBoardRequestPending=false;renderConnection();}
}
function renderConnection() {
  const panel=$("connectionPanel");if(!panel)return;
  const connection=state?.connection;
  setVisible(panel,Boolean(connection&&typeof connection==="object"));
  if(panel.hidden){startupPose="home";return;}
  const c=copy[lang], summary=$("connectionSummary"), disconnect=$("disconnectHardware"), mode=connection.mode??"inspect";
  if($("softwareVersion"))$("softwareVersion").textContent=state.software_version??"";
  const currentSupported=connection.startup_pose_options?.includes("current_aligned")===true;
  if($("startupPosePanel"))setVisible($("startupPosePanel"),currentSupported);
  if(!currentSupported)startupPose="home";
  else if(connectionRequestPending&&pendingStartupPose!==null)startupPose=pendingStartupPose;
  else if(mode!=="inspect"&&["starting","active","stopping"].includes(connection.state))startupPose=validStartupPose(connection.startup_pose);
  else startupPose=connection.startup_pose_options.includes(selectedStartupPose)?selectedStartupPose:"home";
  if($("startupPose")){
    $("startupPose").value=startupPose;
    $("startupPose").disabled=!pageConnected||connectionRequestPending||buttonBoardRequestPending||(connection.can_stop===true&&mode!=="inspect")||connection.state==="stopping";
  }
  if($("startupPoseNote"))$("startupPoseNote").textContent=startupPose==="leader_target"?c.startupLeaderTargetNote:startupPose==="current_relative"?c.startupRelativeNote:startupPose==="home"?c.startupHomeNote:c.startupPoseNote;
  const attempted=$("attemptedStartupPose");
  if(attempted){
    setVisible(attempted,mode!=="inspect"&&["error","stopped"].includes(connection.state)&&Boolean(connection.startup_pose));
    attempted.textContent=lang==="zh"?`上次启动采用：${startupStrategyLabel(connection.startup_pose)}。下次启动采用：${startupStrategyLabel(startupPose)}。`:`Last attempt: ${startupStrategyLabel(connection.startup_pose)}. Next start: ${startupStrategyLabel(startupPose)}.`;
  }
  const statusLabel=pageConnected?(state.status==="button_board_waiting"?c.button_board_waiting:c["connection_"+connection.state]??connection.state):c.disconnected;
  if(summary)summary.textContent=`${c.connectionStatus}: ${statusLabel} · ${c[mode]??mode}${state.software_version?` · ${c.softwareVersion} ${state.software_version}`:""}`;
  for(const [id,requestedMode] of [["connectHardware","inspect"],["startFollowing","follow"],["startMonitoring","monitor"],["startFeedback",feedbackStartMode(connection)]]){
    const button=$(id);if(!button)continue;
    button.textContent=requestedMode==="experiment"?c.startExperiment:c[id];button.disabled=!pageConnected||connectionRequestPending||buttonBoardRequestPending||!modeCanStart(connection,requestedMode);
    const reason=connection.blocked_modes?.[requestedMode];
    button.title=reason?c[reason]??c.modeUnavailable:!availableModes(connection).includes(requestedMode)?c.modeUnavailable:"";
  }
  if(disconnect){disconnect.textContent=mode==="inspect"?c.disconnectHardware:c.stopMotion;disconnect.disabled=!pageConnected||connectionRequestPending||buttonBoardRequestPending||connection.can_stop!==true;}
  if($("connectionMotionNote"))$("connectionMotionNote").textContent=c.connectionMotionNote;
  if($("connectionSwitchNote"))setVisible($("connectionSwitchNote"),connection.can_stop===true);
  if($("feedbackBlocked")){
    const reason=connection.blocked_modes?.run;
    setVisible($("feedbackBlocked"),Boolean(reason||!availableModes(connection).includes("run")));
    const details=reason==="sdk_motor_range_mismatch"?sdkRangeDetails(connection.sdk_motor_range_mismatches):"";
    const formalUnavailable=availableModes(connection).includes("experiment")||mode==="experiment";
    const coverage=state.profile_status?.mass_confirmed===true?c.massConfirmedByUser:state.profile_status?.mass_confirmed===false?c.massUnconfirmed:"";
    $("feedbackBlocked").textContent=(formalUnavailable&&reason==="force_profile_unreviewed"?`${c.formalForceBlocked} ${coverage}`:reason?c[reason]??c.modeUnavailable:c.modeUnavailable)+(details?` (${details})`:"");
  }
  if($("connectionResult"))$("connectionResult").textContent=errorText(connection)||errorText(connectionRequestError)||(connectionRequestNotice?c[connectionRequestNotice]:"");
  renderButtonBoard();
  renderExperimental();
}
async function requestConnection(action,mode="inspect") {
  if(!pageConnected||!state?.connection||connectionRequestPending||buttonBoardRequestPending)return;
  if(action==="start"&&!modeCanStart(state.connection,mode))return;
  if(action==="stop"&&state.connection.can_stop!==true)return;
  const motionStart=action==="start"&&["follow","monitor","experiment"].includes(mode);
  const visibleStartupPose=$("startupPose")?.value??startupPose;
  const supportedChoice=state.connection.startup_pose_options?.includes(visibleStartupPose)===true;
  const requestedStartupPose=motionStart&&supportedChoice?validStartupPose(visibleStartupPose):"home";
  // Browsers may restore a select value without dispatching its change event.
  // An explicit start submits the visible choice and preserves it for retries.
  if(motionStart&&supportedChoice&&visibleStartupPose!==startupPose){
    selectedStartupPose=requestedStartupPose;startupPose=requestedStartupPose;
    writePreference("lumos-startup-pose",requestedStartupPose);
  }
  const payload=action==="start"?{action,mode}:{action};
  if(motionStart)payload.startup_pose=requestedStartupPose;
  const request={method:"POST",headers:{"Content-Type":"application/json","X-Feedback-Token":state.token??""},body:JSON.stringify(payload)};
  connectionRequestError=null;connectionRequestNotice="";
  pendingStartupPose=motionStart?requestedStartupPose:null;
  connectionRequestPending=true;renderConnection();
  try{
    const response=await timedFetch("/api/connection",request);
    const value=await response.json();if(!response.ok){connectionRequestError={...value,error:value.error??copy[lang].disconnected};return;}
    state.connection=value.connection??value;
    connectionRequestNotice=action==="start"?"connectionRequested":"disconnectionRequested";
  }catch(error){connectionRequestError={error:error.message};}
  finally{connectionRequestPending=false;pendingStartupPose=null;renderConnection();}
}
function render() {
  if (!state) return;
  const c=copy[lang], stale=["active","inspection_active","leader_gap"].includes(state.status)&&!telemetryFresh();
  const statusLabel=stale&&state.status!=="leader_gap"?c.telemetryStale:c[state.status]??state.status;
  const displayMode=state.connection?.mode??state.mode;
  $("status").textContent=pageConnected?`${statusLabel}${stale&&state.status==="leader_gap"?` · ${c.telemetryStale}`:""} · ${c[displayMode] ?? displayMode}`:c.disconnected;
  $("status").dataset.tone=!pageConnected||state.error||state.status==="error"?"error":stale?"stale":telemetryFresh()?"live":"idle";
  if($("previewNotice")){setVisible($("previewNotice"),state.ui_preview===true);$("previewNotice").textContent=c.previewReadOnly;}
  if($("displaySmoothing"))$("displaySmoothing").textContent=c.displaySmoothing;
  renderConnection();
  const profile=state.profile_status ?? {}, warnings=[];
  if (profile.engineering) warnings.push(c.engineeringInitial);
  if (profile.mass_confirmed === true) warnings.push(profile.engineering?c.massConfirmedByUser:c.massConfirmed);
  if (profile.mass_confirmed === false) warnings.push(c.massUnconfirmed);
  if (profile.force_ready === false) warnings.push(availableModes(state.connection).includes("experiment")||displayMode==="experiment"?c.formalForceBlocked:c.forceBlocked);
  const unverified=state.residual_verified === false || profile.engineering || profile.mass_confirmed === false;
  if (unverified) warnings.push(c.candidateResidual);
  $("commissioning").textContent=warnings.join(" "); setVisible($("commissioning"),warnings.length>0);
  $("modeNote").textContent=c[displayMode+"Note"]??"";
  if($("powerNote"))$("powerNote").textContent=displayMode==="experiment"?c.experimentPowerNote:c.powerNote;
  const baseline={not_recorded:c.baselineUnknown,not_measured:c.baselineNotMeasured,candidate:c.baselineCandidate,measured_unverified:c.baselineCandidate,insufficient:c.baselineInsufficient};
  const observedBaseline=!["inspect","preview"].includes(state.mode) && state.baseline;
  if (state.mode==="experiment") {
    $("baseline").textContent=`${c.experimentFrozen}: ${typeof state.experimental?.baseline_frozen==="boolean"?(state.experimental.baseline_frozen?c.yes:c.no):"—"}. ${c.baselineCaution}`;
  } else if (observedBaseline && typeof observedBaseline === "object") {
    const count=Number.isFinite(observedBaseline.sample_count)?observedBaseline.sample_count:0;
    $("baseline").textContent=`${c.baselineLabel}: ${observedBaseline.ready?c.baselineReady:c.baselineSampling} · ${c.baselineSamples}: ${count} · ${c.baselineStationary}: ${fmt(observedBaseline.continuous_stationary_s)} s. ${c.baselineStationaryWarning}`;
  } else {
    $("baseline").textContent=`${c.baselineLabel}: ${baseline[profile.baseline_status] ?? c.baselineOther}. ${c.baselineCaution}`;
  }
  $("runtimeError").textContent=errorText(state);
  renderFollowDiagnostics();
  const writable=capsWritable();
  $("caps").querySelectorAll("input, button").forEach(el=>el.disabled=!writable);
  $("caps").setAttribute("aria-disabled",String(!writable));
  setVisible($("capsReadOnly"),!writable); $("capsReadOnly").textContent=state.ui_preview===true?c.previewReadOnly:!pageConnected?c.controlsOffline:state.mode==="preview"?c.previewCaps:stale&&state.mode!=="inspect"?c.controlsStale:c.capsReadOnly;
  const headings=[...c.columns]; if(unverified) headings[5]=c.residualUnverified;
  if(!$("columns").children.length)$("columns").append(document.createElement("tr"));
  cells($("columns").children[0],headings,"th");
  if (!dirty) { $("common").value=state.caps.common_mw; state.caps.axis_mw.forEach((v,i)=>$("cap"+i).value=v); }
  tableRows($("rows"),Array.from({length:7}, (_,i)=>{
    const p=state.requested_mw?.[i];
    return [name(i),fmt(state.q_sdk?.[i]),motorState(state.motor_status?.[i],c),fmt(state.measured_nm?.[i]),fmt(state.model_nm?.[i]),fmt(state.residual_nm?.[i]),fmt(p),fmt(state.predicted_mw?.[i]),leaderState(p,state.hold_requested?.[i],c,i),fmt(state.temperature_c?.[i]),fmt(state.current_ma?.[i]),fmt(state.leader_current_ma?.[i]),fmt(state.age_ms?.[i])];
  }));
  tableRows($("policyRows"),Object.entries(labels).map(([key,label])=>{
    return [label[lang==="zh"?0:1],...Array.from({length:7},(_,i)=>{const value=state.policy?.[i]?.[key];return Number.isFinite(value)?String(value):"—";})];
  }));
  draw();
}
function alignmentReference() {
  if(startupPose==="leader_target")return null;
  const alignment=startupPose==="current_relative"?(telemetryFresh()?state?.relative_alignment:null):startupPose==="current_aligned"?(telemetryFresh()?state?.current_alignment:null):modelDefinition?.alignment;
  const sixFinite=value=>Array.isArray(value)&&value.length===6&&value.every(Number.isFinite);
  return alignment?.available===true&&[alignment.target_sdk_rad,alignment.target_urdf_rad,alignment.leader_home_deg,alignment.leader_ids,alignment.tolerance_rad].every(sixFinite)&&alignment.tolerance_rad.every(value=>value>=0)?alignment:null;
}
function renderAlignment() {
  const c=copy[lang], alignment=alignmentReference(), available=alignment!==null;
  const direct=startupPose==="leader_target";
  if($("alignmentControls"))setVisible($("alignmentControls"),!direct);
  const relative=startupPose==="current_relative", current=relative||startupPose==="current_aligned";
  const toggle=$("showAlignmentTarget");
  if(toggle){toggle.checked=showAlignmentTarget;toggle.disabled=!available;}
  if($("alignmentTable"))setVisible($("alignmentTable"),available);
  if($("alignmentColumns"))cells($("alignmentColumns"),relative?c.alignmentRelativeColumns:current?c.alignmentCurrentPoseColumns:c.alignmentColumns,"th");
  const currentFresh=available&&telemetryFresh()&&Array.isArray(state?.q_sdk)&&state.q_sdk.length>=6&&state.q_sdk.slice(0,6).every(Number.isFinite);
  const running=currentFresh&&state?.connection?.state==="active"&&["follow","monitor","experiment"].includes(state?.mode);
  if($("alignmentStatus"))$("alignmentStatus").textContent=direct?c.alignmentLeaderTarget:relative?(running?c.alignmentRelativeRunning:currentFresh?c.alignmentRelativeFresh:c.alignmentRelativeMissing):current?(running?c.alignmentCurrentPoseRunning:currentFresh?c.alignmentCurrentPoseFresh:c.alignmentCurrentPoseMissing):!modelDefinition&&!modelFailed?c.alignmentLoading:!available?c.alignmentUnavailable:currentFresh?c.alignmentCurrentFresh:c.alignmentNeedsLive;
  for(const [key,text] of [["alignmentNote",direct?c.alignmentLeaderTargetNote:relative?c.alignmentRelativeNote:current?c.alignmentCurrentPoseNote:c.alignmentNote],["leaderHomeNote",direct?c.leaderTargetCalibrationNote:current&&!relative?c.leaderCurrentNote:c.leaderHomeNote],["alignmentTargetLegend",current?c.currentPoseLegend:c.alignmentTargetLegend],["showAlignmentTarget",current?c.showCurrentAlignmentTarget:c.showAlignmentTarget]])document.querySelectorAll(`[data-i18n="${key}"]`).forEach(el=>el.textContent=text);
  if(!$("alignmentRows"))return;
  if(!available){$("alignmentRows").replaceChildren();return;}
  const degrees=value=>value*180/Math.PI;
  const signed=value=>{const rounded=Math.round(degrees(value)*100)/100;return `${rounded>0?"+":""}${rounded.toFixed(2)}`;};
  tableRows($("alignmentRows"),Array.from({length:6},(_,i)=>{
    return [`J${i+1}`,currentFresh?fmt(degrees(state.q_sdk[i])):"—",fmt(degrees(alignment.target_sdk_rad[i])),currentFresh?signed(alignment.target_sdk_rad[i]-state.q_sdk[i]):"—",`±${fmt(degrees(alignment.tolerance_rad[i]))}`,String(alignment.leader_ids[i]),fmt(alignment.leader_home_deg[i])];
  }));
}
Object.assign(copy.zh,{calibratedReferencePose:"配对校准参考姿态 · 非实时数据"});
Object.assign(copy.en,{calibratedReferencePose:"Paired calibration reference pose · Not live data"});
function displayPose(model,current) {
  const finite=(value,length)=>Array.isArray(value)&&value.length===length&&value.every(Number.isFinite);
  const complete=pose=>finite(pose?.q_sdk,7)&&pose.q_sdk[6]>=0&&pose.q_sdk[6]<=1;
  const hasCurrent=complete(current)&&current.coherent_feedback!==false;
  const cached=current?.last_pose;
  const hasLast=cached?.display_only===true&&complete(cached)&&finite(cached.q_urdf,6);
  const measured=hasCurrent?current:hasLast?cached:null;
  if(!measured)return {q:model.preview_q,opening:0.55,source:model.preview_source??"urdf_reference"};
  const q=finite(measured.q_urdf,6)?measured.q_urdf.slice():measured.q_sdk.slice(0,6).map((value,i)=>value*model.sdk_to_urdf[i].scale+model.sdk_to_urdf[i].offset);
  return {q,opening:measured.q_sdk[6],source:hasCurrent?"current":"last"};
}
function schematicPose(model,q,opening) {
  const identity=[[1,0,0],[0,1,0],[0,0,1]];
  const multiply=(a,b)=>a.map(row=>b[0].map((_,j)=>row.reduce((sum,value,k)=>sum+value*b[k][j],0)));
  const rotate=(r,p)=>r.map(row=>row.reduce((sum,value,i)=>sum+value*p[i],0));
  const add=(a,b)=>a.map((value,i)=>value+b[i]);
  const rotation=(axis,angle)=>{
    const length=Math.hypot(...axis);if(!length)return identity;
    const [x,y,z]=axis.map(value=>value/length),c=Math.cos(angle),s=Math.sin(angle),t=1-c;
    return [[t*x*x+c,t*x*y-s*z,t*x*z+s*y],[t*x*y+s*z,t*y*y+c,t*y*z-s*x],[t*x*z-s*y,t*y*z+s*x,t*z*z+c]];
  };
  const positions=Object.fromEntries(model.joint_order.map((name,i)=>[name,q[i]]));
  for(const joint of model.gripper_joints??[])positions[joint.name]=joint.closed+(joint.open-joint.closed)*opening;
  const children=new Map(),points={},edges=[];
  for(const joint of model.joints){if(!children.has(joint.parent))children.set(joint.parent,[]);children.get(joint.parent).push(joint);}
  const walk=(link,r,p)=>{
    points[link]=p;
    for(const joint of children.get(link)??[]){
      const [roll,pitch,yaw]=joint.origin.rpy;
      const local=multiply(rotation([0,0,1],yaw),multiply(rotation([0,1,0],pitch),rotation([1,0,0],roll)));
      let rr=multiply(r,local),pp=add(p,rotate(r,joint.origin.xyz));
      const angle=positions[joint.name]??0;
      if(["revolute","continuous"].includes(joint.type))rr=multiply(rr,rotation(joint.axis,angle));
      else if(joint.type==="prismatic")pp=add(pp,rotate(rr,joint.axis.map(value=>value*angle/Math.hypot(...joint.axis))));
      edges.push([link,joint.child]);walk(joint.child,rr,pp);
    }
  };
  walk(model.root,identity,[0,0,0]);return {points,edges};
}
function draw() {
  const source=$("poseSource"), c=copy[lang];
  renderAlignment();
  if(!modelDefinition){if(source)source.textContent=modelFailed?c.modelUnavailable:c.modelLoading;return;}
  const realMeshes=modelDefinition.geometry_source==="urdf_meshes";
  const modelLabel=typeof modelDefinition.source?.label==="string"?modelDefinition.source.label:"URDF";
  const note=realMeshes?c.meshNote.replace("{source}",modelLabel):c.schematicNote;
  document.querySelectorAll('[data-i18n="poseNote"]').forEach(el=>{if(el.textContent!==note)el.textContent=note;});
  const displayed=displayPose(modelDefinition,state),{q,opening}=displayed;
  const active=displayed.source==="current"&&telemetryFresh();
  const target=showAlignmentTarget?alignmentReference()?.target_urdf_rad??null:null;
  if(!solidViewer&&!viewerFailed&&typeof window!=="undefined"&&window.LumosViewer){
    try{
      solidViewer=new window.LumosViewer($("pose"),modelDefinition);
      solidViewer.canvas?.addEventListener("lumos-viewer-status",()=>draw());
      solidViewer.ready?.then(()=>draw()).catch(error=>{viewerFailed=true;solidViewer=null;console.error("Viewer model failed",error);draw();});
    }
    catch(error){viewerFailed=true;console.error("Viewer initialization failed",error);}
  }
  if(source){
    const poseLabel=active?c.livePose:["current","last"].includes(displayed.source)?c.lastPose:displayed.source==="paired_calibration"?c.calibratedReferencePose:c.referencePose;
    let geometryLabel=c.schematic;
    if(realMeshes){
      const loaded=Number.isInteger(solidViewer?.meshLoaded)?solidViewer.meshLoaded:0;
      const expected=Number.isInteger(solidViewer?.meshExpected)?solidViewer.meshExpected:(modelDefinition.links??[]).reduce((total,link)=>total+(link.visuals??[]).filter(visual=>visual.geometry?.type==="mesh").length,0);
      const failed=Number.isInteger(solidViewer?.meshFailed)?solidViewer.meshFailed:0;
      const geometrySource=viewerFailed?"mesh_error":solidViewer?.geometrySource??"loading_meshes";
      const statusLabel=geometrySource==="urdf_meshes"?c.meshSource:geometrySource==="mesh_error"?c.meshFailed:geometrySource==="incomplete_meshes"?c.meshIncomplete:c.meshLoading;
      geometryLabel=`${modelLabel} · ${statusLabel} ${loaded}/${expected}${failed?` · ${c.meshFailedCount} ${failed}`:""}`;
    }
    if(solidViewer?.available===false)geometryLabel+=` · ${c.cpuRenderer}`;
    source.textContent=`${poseLabel} · ${geometryLabel}`;
    source.dataset.live=String(active);
    source.dataset.geometry=viewerFailed?"mesh_error":solidViewer?.geometrySource??(realMeshes?"loading_meshes":"urdf_schematic");
  }
  if(solidViewer){solidViewer.update(q,opening,target,{live:active,sampleTime:stateReceivedAt});return;}
  if(!viewerFailed)return; // Wait for the module without claiming a 2D canvas context.
  if(realMeshes)return; // A CAD failure must remain visible, never turn into a schematic arm.
  const canvas=$("pose"),ctx=canvas.getContext("2d");if(!ctx)return;
  const {points,edges}=schematicPose(modelDefinition,q,opening);
  const w=canvas.width,h=canvas.height,project=([x,y,z])=>[w/2+(x*.8-y*.6)*640,h*.82-(z+(x*.6+y*.8)*.25)*640];
  ctx.clearRect(0,0,w,h);ctx.lineCap="round";ctx.lineWidth=25;ctx.strokeStyle="#43cdb7";
  for(const [a,b] of edges){ctx.beginPath();ctx.moveTo(...project(points[a]));ctx.lineTo(...project(points[b]));ctx.stroke();}
  for(const p of Object.values(points)){ctx.fillStyle="#cbdde3";ctx.beginPath();ctx.arc(...project(p),14,0,Math.PI*2);ctx.fill();}
  if(source)source.textContent+=` · ${copy[lang].rendererFallback}`;
}
async function loadModel(){
  try{const response=await timedFetch("/api/model");if(!response.ok)throw new Error("URDF unavailable");modelDefinition=await response.json();draw();}
  catch(error){modelFailed=true;draw();console.error(error);}
}
if(typeof window!=="undefined")window.addEventListener("lumos-viewer-ready",()=>draw());
$("resetView")?.addEventListener("click",()=>solidViewer?.reset());
$("showAlignmentTarget")?.addEventListener("change",()=>{showAlignmentTarget=$("showAlignmentTarget").checked;draw();});
$("startupPose")?.addEventListener("change",()=>{
  const choice=validStartupPose($("startupPose").value);
  if($("startupPose").disabled||!state?.connection?.startup_pose_options?.includes(choice)){render();return;}
  selectedStartupPose=choice;startupPose=choice;writePreference("lumos-startup-pose",choice);render();
});
$("connectHardware")?.addEventListener("click",()=>requestConnection("start","inspect"));
$("startFollowing")?.addEventListener("click",()=>requestConnection("start","follow"));
$("startMonitoring")?.addEventListener("click",()=>requestConnection("start","monitor"));
$("startFeedback")?.addEventListener("click",()=>requestConnection("start",feedbackStartMode(state?.connection)));
$("disconnectHardware")?.addEventListener("click",()=>requestConnection("stop"));
$("baselineRecaptureUnloaded")?.addEventListener("change",()=>renderBaselineRecapture());
$("recaptureBaseline")?.addEventListener("click",()=>requestBaselineRecapture());
$("buttonBoardEnabled")?.addEventListener("change",()=>$("buttonBoardEnabled").checked?requestButtonBoard(true):undefined);
$("buttonBoardDisabled")?.addEventListener("change",()=>$("buttonBoardDisabled").checked?requestButtonBoard(false):undefined);
$("language").addEventListener("click",changeLanguage);
$("caps").addEventListener("input",()=>{dirty=true;$("result").textContent=copy[lang].powerDraft;});
$("caps").addEventListener("submit",async e=>{e.preventDefault();if(!capsWritable()){$("result").textContent=copy[lang].capsReadOnly;return;}try{const response=await timedFetch("/api/caps",{method:"POST",headers:{"Content-Type":"application/json","X-Feedback-Token":state?.token??""},body:JSON.stringify({common_mw:Number($("common").value),axis_mw:Array.from({length:7},(_,i)=>Number($("cap"+i).value))})});const value=await response.json();if(!response.ok)throw new Error(value.code==="caps_read_only"?copy[lang].capsReadOnly:value.error);dirty=false;$("result").textContent=copy[lang].saved;}catch(error){$("result").textContent=error.message;}});
async function poll(){const started=nowMs();try{const response=await timedFetch("/api/state");if(!response.ok)throw new Error();state=await response.json();pageConnected=true;stateReceivedAt=started;armFreshnessExpiry();render();}catch{pageConnected=false;$("status").textContent=copy[lang].disconnected;if(state)render();else draw();}finally{setTimeout(poll,200);}}
translate();loadModel();poll();
