# StarArm102 → Lumos：LeRobot 接入版

[**从安装到遥操作的完整实操教程**](docs/stararm102-lumos.md)

[Lumos 力反馈运行与配置说明](docs/feedback.zh.md) / [Lumos force-feedback runtime and configuration](docs/feedback.md)

**0.6.24 跟随与实验阻尼版本：[真机启动步骤](docs/hardware.zh.md) / [Hardware startup](docs/hardware.md)。**

本仓库恢复至 UI 大改之前的完整 0.6.24 功能，同时撤销 0.6.27–0.6.31 后续变更。
本版夹爪未到位仍可能阻止整组初始基线，也没有同会话显式基线重采入口。

This repository restores the complete pre-redesign 0.6.24 functionality and removes
later 0.6.27–0.6.31 changes. An unsettled gripper can still block the whole initial
baseline, and explicit baseline recapture within the same session is unavailable.

0.6.24 统一启动前后实测位置的边界判断，使用已绑定的各轴 `limit_margin`，与 CAN 检查一致，避免零位附近半个编码步长被后续严格检查再次拒绝。真实测量和基线锚点保留原值；首次位置命令及相对起步参考裁剪到原限位内，Leader 目标上限不变。超出原测量余量仍会拒绝。该修复不需要重新校准，源码变化需要正常重新生成绑定。

0.6.24 使用绑定会话中的速度、单步限制、控制频率和 Leader One Euro 滤波配置。
页面显示实际频率与超时周期，调度目标不保证能够达到；CAN／串口检查及控制逐周期执行。
每轴表显示映射、限位裁剪、实际发送与实测位置，帮助区分校准／限位截断、限速滞后
与伺服偏差。不同轴分别限速，不保证同一时刻到位；校准和厂家限位保持不变。

新增显式 `experiment` 实验阻尼模式：仅在本机绑定配置中明确授权、声明真实空载、
并启用原生按钮事件接收后可用。至少 2 秒连续真实空载静止样本期间输出为零；
冻结各轴绝对剩余力矩 P99＋4 个力矩量化步长作为最小死区，不减均值、不在线学载荷。
仍用整臂 q/v/估计 a 计算 J1–J6 模型力矩，夹爪独立；滤波加速度超出原模型域时，
保留该估计值并立即停止阻尼，重新预热后再判断。确认所用整臂 URDF
完整覆盖 TypeLJ 时，记录 `mass_coverage: urdf_includes_typelj`、`payload: null`，避免重复
叠加夹爪质量。显式实验契约的 `max_mw: null` 表示取消额外实验功率上限；每轴实际
上限为网页共同与该轴上限的较小值，默认均为 1200 mW。共同可调 400–1500 mW，
每轴可调 0–1500 mW，0 关闭该轴阻尼。已确认模型覆盖且 `max_mw: null` 的模式按各轴
`profile.policy[].rise_mw_s` 上升；需针对实际硬件核对该配置。
旧版 1–200 mW 的显式额外上限契约仍兼容，上升速率继续取配置值与 100 mW/s 的较小值。
网页显示实际生效的逐轴上升速率。所有自动及软件 HOLD 请求关闭。
mW 为 Leader 请求值，不是实测机械阻力或安全额定值。

首次空载基线只在收到完整真实按钮释放、至少完成一次完整跟随发送、限速接近结束且实测已接近目标后采集；目标与实测还需连续静止至少 2 秒。等待按钮、支撑姿态或起步接近期间不采集。目标按原关节限位裁剪后判断到位，未扩大行程。冻结后不会因按钮切换或新外载自动重采。该门控修正启动采样时机，单姿态基线仍不能证明整个运动范围没有空载误阻尼。

J4 单独持续静态受载时，只要该轴滤波残差超过当时死区并满足负载确认，就会输出对应
阻尼，无需移动其他关节。功率建立时间由配置的上升速率、滤波及确认时间共同决定。
残差、死区、满输出标尺及最终强度映射不变；小残差仍可能只产生较小请求。
软件验证不等于真机手感验收。

原生 HD 板需收到完整六轴真实释放事件，才允许实验输出；未知、锁定或部分组
暂停新的 Follower 目标并抑制软件输出，不重发按钮保持。每次运动／力命令前检查
已排队事件，但板与软件没有原子互斥保证；部分事件后可能残留的软件输出必须
标为未知，不能声称已释放。退出控制仍可能失去从臂支撑。
实验模式需要独立确认模型覆盖；动态阈值和实机效果仍待验收，工程配置保持
`force_authorized=false`，`run` 继续禁止。模型覆盖声明不等于完成动态验证。
静止基线未覆盖整个姿态和运动范围；残差含摩擦／模型误差，实验阻尼不等于准确外力
或正式力反馈验收。网页仅允许共同及各轴功率上限，不允许修改内部策略。

Version 0.6.24 applies each bound axis `limit_margin` consistently to measured positions before and after startup, matching CAN checks so half an encoder step near zero is not rejected by a later strict measurement check. Raw measurements and baseline anchors are preserved. Initial position commands and relative-start references stay clipped to the original limits; Leader target limits are unchanged. Measurements beyond the existing margin still fail. Recalibration is not required; changed source requires normal rebinding.

Version 0.6.24 uses speed, target-step, control-rate and Leader One Euro filter
settings from the bound session. The page reports achieved frequency and overruns;
the scheduling target is not a guaranteed achieved rate. CAN/serial checks and
control remain per cycle. Per-axis tracking shows mapped, joint-clipped, sent and
measured positions, separating calibration/limit clips from rate lag and servo
error. Axes are rate-limited independently and need not arrive simultaneously.
Calibration and manufacturer limits are unchanged.

The explicit `experiment` mode requires a bound local authorization, an actual
unloaded declaration and native-button event reception. Output remains zero
through at least two seconds of continuous real stationary samples. Per-axis
absolute residual P99 plus four torque LSBs is frozen as a minimum deadzone;
no fitted mean is removed and contact loads are never learned during output.
Whole-arm q/v/estimated-a dynamics still supplies J1-J6 model torques, with
independent gripper torque. A filtered acceleration outside the original model
domain is retained and inhibits damping until valid warmup. When the whole-arm
URDF is confirmed to include TypeLJ, record
`mass_coverage: urdf_includes_typelj` and `payload: null` to avoid adding its mass
twice. An explicit experimental contract with `max_mw: null` removes the additional
experimental ceiling. Each effective cap is the smaller of the webpage common
and axis caps, both defaulting to 1200 mW. The common cap is adjustable from
400–1500 mW and each axis from 0–1500 mW; 0 disables that axis. With confirmed
model coverage and `max_mw: null`, each axis uses its configured
`profile.policy[].rise_mw_s`, reviewed for the actual hardware. Existing explicit 1–200 mW additional-cap
contracts retain the smaller of their configured rise rate and 100 mW/s. The page
shows each axis's effective rise rate. All software and automatic HOLD requests
remain disabled. mW is a Leader request, not measured mechanical resistance or a
safety rating.

The initial unloaded baseline is collected only after a complete real button release, at least one complete successful following write, the end of target-rate catch-up and measured proximity to the target. Both the target and measured pose must remain stationary for at least two seconds. Button waiting, supported startup poses and initial approach cannot qualify. Arrival uses the original joint-limit-clipped target without enlarging travel. Once frozen, the baseline is never relearned on button changes or new external loads. This gate corrects startup sampling timing; a single-pose baseline still cannot establish unloaded separation over the entire motion range.

A sustained static load on J4 alone produces its corresponding damping once that
axis's filtered residual exceeds the current deadzone and satisfies load
confirmation; other joints do not need to move. Power buildup depends on the
configured rise rate, filtering and confirmation time. The
residual, deadzone, full-output torque scale and final intensity mapping are
unchanged, so a small residual may still produce a small request. Software
validation does not establish acceptance of physical damping feel.

A complete real six-axis native release event is required for experimental
output. Unknown, locked or incomplete groups pause new Follower targets and
suppress software commands without replaying native hold. Queued events are
checked before motion/force writes, but no atomic exclusion with the board is
guaranteed. Residual software output after an incomplete transition is reported
as unknown, never as confirmed released. Exiting control can remove Follower
support. Experimental mode requires a separate model-coverage confirmation; dynamic
thresholds and hardware performance still await acceptance. Engineering profiles
keep force_authorized false and `run` blocked. A coverage declaration does not
establish dynamic validation. The stationary baseline does not cover every pose or moving condition;
residuals include friction/model error and are not accurate
external-force measurements or formal force-feedback acceptance. Web controls
remain limited to common/per-axis power caps.

0.6.19 处理工程基础跟随中 SDK 六轴异步缓存与独立 CAN 的时间差。严格启动
比对通过后，运行中若当前值不一致，可在原时效窗口内匹配真实 CAN 样本，仍用原
角度容差；当前 CAN 必须新鲜有效。历史仅存内存，不参与遥测新鲜度；无法据此
证明 SDK 自身缓存年龄。启动种子、其他模式、夹爪及原保护保持严格。

Version 0.6.19 handles the arm SDK cache lag during engineering basic following.
After strict startup agreement, a runtime discrepancy may match an actual recent
CAN sample within the original freshness window and angle tolerance. Current CAN
must still be fresh and valid. History is memory only and cannot refresh telemetry
or establish the SDK cache's own age. Startup seeds, other modes, gripper checks
and the remaining protections stay strict.

0.6.18 仅对工程基础跟随的直接起步，允许使能后的限幅微动；使能前仍需稳定回包，
起点漂移最多 2°、实际速度／位置保护、SDK/CAN 一致性及目标限速继续保留。
其他起步方式和监测模式保留原静止要求。

Version 0.6.18 permits bounded motion after enabling only for engineering basic
following with direct startup. Pre-enable stability, origin drift of at most 2°,
actual velocity/position guards, SDK/CAN agreement and target rate limits remain.
Other startup strategies and monitoring retain their original quiet-pose check.

0.6.17 将六轴动力学加速度门禁与工程基础跟随分开：工程 `follow` 不计算动力学，
不再因该未实测阈值停机；`monitor`／`run`、非工程配置及夹爪的原检查保留。
通信时效、故障、温度、实际限位、速度、位置跳变、SDK/CAN 一致性和目标限速不变。
这不代表真机跟随或力反馈效果已经验证。

Version 0.6.17 separates the arm dynamics acceleration admission from engineering
basic following. Engineering `follow` does not calculate dynamics and no longer
stops on this unmeasured threshold. `monitor`/`run`, non-engineering profiles and
gripper checks retain their previous behavior. Freshness, faults, temperatures,
measured limits, velocity, position jumps, SDK/CAN agreement and target rate limits
remain checked. This does not establish physical following or damping performance.

0.6.16 增加工程基础跟随／监测选项“直接跟随 Leader（限速接近）”
（`leader_target`）。不要求两端对齐、回到校准零位或夹爪预先闭合。程序用新鲜
实测从臂姿态初始化限速起点，再按原关节限位、速度和每步限制接近 Leader
绝对映射目标；没有相对起步偏移，不修改校准。SDK 构造时可能自动闭合夹爪，
使能到首个实测种子之间仍可能短暂运动。通信、故障、温度、实测限位、SDK/CAN
一致性、起点稳定及 SDK J2 构造条件继续检查。该选项不启动力反馈。

Version 0.6.16 adds engineering follow/monitor startup “Follow Leader directly
(rate-limited approach)” (`leader_target`). Neither arm must match the other or
return to home, and the gripper need not be closed beforehand. Fresh measured
Follower positions seed the existing rate limiter, which approaches the absolute
mapped Leader target with original joint, speed and step limits. No relative
offset or calibration change is applied. SDK construction may close the gripper
and brief motion remains possible before the first measured seed. Communication,
fault, temperature, measured limits, SDK/CAN agreement, origin stability and the
SDK J2 construction condition remain checked. This option does not enable force
feedback.

0.6.15 限位报错显示实际角度，姿态差容差只显示在对应的对齐检查中。目标超出关节
限位时，即使两端很接近也不能按零位方式启动；工程相对起步可使用各自当前位置。

Version 0.6.15 reports actual angles for limit failures and shows alignment
tolerances only for alignment checks. A target beyond a joint limit prevents
home startup even when both arms are close. Engineering relative startup uses
each arm's current origin.

可选 `robot.startup_pair_tolerance_rad` 单独设置零位模式的两端允许差；20° 对应
`0.3490658503988659` rad。省略或设为 `null` 沿用原值。此项不改变各自距校准
零位的条件、关节限位，或当前位置模式的最多 2° 启动保护。无需重做配对校准；
改当前设置后正常重生成绑定。

Optional `robot.startup_pair_tolerance_rad` sets the pair-difference threshold
for home mode; 20° is `0.3490658503988659` rad. Omission or `null` keeps the
legacy values. Individual home-distance checks, joint limits and the current-pose
startup cap of at most 2° remain. Preserve calibration and regenerate the binding
after changing this setting.

0.6.14 无实测数据时显示已验证的配对校准参考，不再使用抬高的演示姿态；读取或姿态
预检后，最后实测姿态仅在内存保留，停止／报错后标注为历史数据，不参与启动准入。
启动按下拉框的实际可见值明确提交对齐方式，并分别显示上次尝试及下次选择。
姿态错误区分离开校准零位、两端差、限位、夹爪未闭合及 J2 启动条件；保护阈值不变。

Version 0.6.14 displays the validated paired calibration reference before any
measurement, replacing the raised demonstration pose. Measured or pose-rejected
preflight geometry is retained only in memory and marked historical after stop
or error; it cannot authorize startup. Starts explicitly submit the visible
alignment selection, and the page distinguishes the last attempt from the next
selection. Diagnostics distinguish home distance, pair difference, limits,
unclosed gripper and J2 startup conditions. Guard thresholds are unchanged.

0.6.13 用可审查的 C++ 源码兼容层修正固定 SDK 的真实原生速度编解码：首次加载核心前，
通过 `RTLD_GLOBAL` 提供同名完整 13 类型量程表，仅把 DM4340 的 VMAX 从 8 改为
10 rad/s。原生 getter、MIT encoder 和状态 decoder 的全部 13 类型、正负端点及非零
量化用例已在无设备条件下验证；核心文件和电机寄存器不改。原遥操作能按位置跟随，
因为位置量程一致；旧速度量程仍会影响内部速度及动态项。此前“只能等待厂家修正”
的结论过窄，现已有本地源码修正路线。

Version 0.6.13 supplies a reviewed C++ compatibility layer for the pinned SDK's
actual native velocity codecs. Before the core's first load, `RTLD_GLOBAL`
provides its complete 13-type range table, changing only DM4340 VMAX from 8 to
10 rad/s. Device-free checks passed for the native getter, MIT encoder and state
decoder across all 13 types, signed endpoints and nonzero quantization cases.
The core file and motor registers remain unchanged. Original position following
can work because position ranges agree, while the old velocity range affects
internal velocities and dynamics. A local source correction is now available;
the earlier vendor-only conclusion was too narrow.

在明确指定的 SDK 副本中构建，并正常重新生成绑定；源码交付不包含生成的 `.so`、
manifest、本机配置、校准、绑定或运行数据。运行时绑定源码及产物，先验证实际原生
编解码，再导入 SDK／构造机械臂；已加载的核心、部分或变更产物及旧绑定会被拒绝。
未安装有效修正时，0.6.11 的量程门禁继续拒绝跟随。安装并重新绑定后 `follow/monitor`
可通过该门禁，仍须满足其他启动检查；真机跟随、保持和阻尼尚未验证。模型覆盖须单独确认；`run` 仍因
动态阈值、实测响应及正式原生按钮仲裁未验收而禁止。

Build in an explicitly selected SDK copy and regenerate the normal binding.
Source delivery excludes generated `.so`/manifest files and local setup,
calibration, bindings or runtime data. Runtime binds source and artifacts and
verifies the actual codecs before importing the SDK or constructing an arm.
Previously loaded cores, partial/changed artifacts and old bindings are rejected.
The original range gate still rejects following without a valid correction.
With the correction installed and rebound, `follow/monitor` can pass that gate;
all other startup checks remain. Physical following, holding and damping remain
unvalidated. Model coverage requires separate confirmation; formal `run` remains
blocked pending dynamic-threshold, measured-response and native-button arbitration acceptance.

```bash
python tools/build_sdk_motor_ranges.py --sdk-root /absolute/path/to/sdk-copy/startouch_sdk
lumos-feedback bind --profile configs/commissioning.local.json \
  --binding configs/commissioning.binding.json --purpose monitor --replace
```

Gitee 与 GitHub 均为 `040412542c0b6b6fd2bb6dd652c0de6c666631be`，没有额外 TypeLJ
惯量资料；YAML 的 `0.126` 注释和网页 V3 末端质量不能作为 TypeLJ 参数。保留原校准和
两个 B601 仓库。[构建、量程及模型证据](docs/sdk-startup-review.md)。

Gitee and GitHub share commit `040412542c0b6b6fd2bb6dd652c0de6c666631be` with no
additional TypeLJ inertial evidence. Neither YAML's commented `0.126` nor the V3
viewer's distal mass establishes TypeLJ parameters. Existing calibration and
both B601 repositories are preserved. See [build, range and model evidence](docs/sdk-startup-review.md).

原 `lumos-lerobot teleop` 入口支持显式反馈扩展；不传反馈参数时保留原基础路径。
反馈扩展复用 `StarArm102.get_action`、`Lumos.send_action` 及配对校准映射，由安全
`Session` 外围循环执行。`--feedback-profile` 与 `--feedback-binding` 同时提供；
`--feedback-mode` 可选 `follow/monitor/run`，默认 `monitor`。`--config` 必传，规范化
后的完整会话配置须与 profile 绑定的 session 完全一致。以下仅离线打印计划：

The existing `lumos-lerobot teleop` entry accepts explicit feedback options;
without them its basic path remains unchanged. The extension reuses
`StarArm102.get_action`, `Lumos.send_action` and the paired mapping inside the
safety `Session` loop. Supply `--feedback-profile` and `--feedback-binding`
together. `--feedback-mode` accepts `follow/monitor/run`, defaulting to `monitor`.
The required `--config` must normalize to exactly the profile's bound session.
This command only prints an offline plan:

```bash
lumos-lerobot teleop --config configs/local.json \
  --feedback-profile configs/commissioning.local.json \
  --feedback-binding configs/commissioning.binding.json \
  --feedback-mode monitor --print-command
```

此前的构造 GIL、SDK 目录、首次状态就绪和速度量化保护修正继续保留，见
[SDK 审查](docs/sdk-startup-review.md)。Constructor GIL, SDK layout, first-state
readiness and velocity-quantization protections remain in place; see the
[SDK review](docs/sdk-startup-review.md).

默认仍要求两臂对齐原配对校准起始姿态。工程 `follow/monitor` 提供两个显式选择：
“按当前姿态对齐”（`current_aligned`）按原校准逆算目标对齐 Leader；新增“当前位置
相对起步”（`current_relative`）允许 Leader 放在稳定、可操作的位置，无需达到该绝对
目标。相对模式在 Follower 七轴稳定检查后查询 Leader 原始起点，六轴本次固定偏移为 `Follower 实际位置 −
Leader 原映射目标`，后续目标为原映射目标加该偏移；保留原方向、增益、ID、限位、
速度及单步限制，夹爪不加偏移。偏移仅留内存，退出清除，不改校准或写日志。

两种当前姿态方式均保留 Follower 七轴至少 0.5 秒真实新帧稳定、原限位、SDK J2 ≥ −1.57 rad、
Follower 夹爪归一化开度 ≤ 0.01，以及按本次目标计算的逐轴误差不超过
`min(2°，目标单步限制，原启动容差)`。Leader 起点由查询取得，SDK 构造后再次查询并按固定偏移复核；还要复核新 CAN、漂移和 SDK 一致性，再发送固定
姿态目标并检查 2 秒稳定。Follower 全程保留稳定支撑。构造内先使能的窗口仍存在；
这不是原子保持，尚未经过真机验证，不能据此松手并认为 J4 一定保住。质量覆盖和
原生按钮仲裁仍禁止力输出。

Home alignment remains the default. Engineering `follow/monitor` offers two
explicit choices: `current_aligned` aligns the Leader to inverse targets from the
existing calibration; the new `current_relative` starts from both arms' current
positions with the Leader resting in a stable, usable pose. It checks stable
Follower feedback, queries the raw Leader origin and fixes six session offsets as `actual Follower − original mapped
Leader target`; subsequent targets use the original mapping plus those offsets.
Original signs, gains, IDs, limits, speed and step guards remain active. The
gripper has no offset. Offsets stay in memory, are cleared on exit and never change
calibration or create logs.

Both current-pose choices retain at least 0.5 s of stable new Follower seven-axis frames, existing
limits, SDK J2 ≥ −1.57 rad, normalized Follower gripper opening ≤ 0.01 and per-axis
agreement against this session's target within `min(2 degrees, target step limit,
original startup tolerance)`. The Leader origin is queried, then queried again
after SDK construction and checked using the fixed offset. Fresh CAN, drift and SDK agreement are checked
before a fixed pose target and a two-second stability check. Keep stable Follower
support throughout. Construction still enables before that target: this is not
atomic holding, remains physically unvalidated, and does not establish that
releasing support will hold J4. Mass coverage and native-button arbitration still
block force output.

网页可选择有／无按钮板。新款 HD 板不回应旧款 ID 7 查询；选择“有”后，跟随连接持续等待一次真实按键，点击停止可取消。等候时不使能，从臂姿态预检在完整六轴原生命令组到达后执行。按键前支撑好主臂。软件监听锁放事件暂停／恢复从臂，不重发保持命令。新板目前仅支持本反馈网页／CLI 的 `follow/monitor`；原 LeRobot 动作接口会明确拒绝，`run` 也因输出仲裁未验证而禁止。直接适配器／CLI 保留原 60 秒操作等待上限。

The page supports a present/absent button-board setting. Native HD boards do not answer legacy ID 7 queries. With the board enabled, a following connection keeps listening for a real button event until you press or select Stop. Motors are not enabled while waiting; pose preflight follows a complete six-joint native command group. Support the Leader before pressing. Software observes lock/release events to pause/resume Follower targets without replaying hold commands. Native boards currently require this feedback dashboard/CLI's `follow/monitor`; the original LeRobot action interface refuses them, and `run` is blocked pending output-arbitration validation. Direct adapter/CLI use retains its sixty-second operator window.

三维页面默认叠加青色半透明的配对校准目标。`current_aligned` 显示当前 Follower 姿态及原校准逆算的 Leader 对齐目标；`current_relative` 显示当前 Follower 姿态和 Leader 校准 home 参考，home 不是相对起步的必达目标。只读连接后可对照真实姿态；图中目标不会自动驱动机械臂。SDK 从已绑定的构建目录加载，仍严格核对文件及实际加载库。

The 3D view overlays the paired calibration goal in translucent cyan by default. `current_aligned` shows the current Follower pose and Leader inverse targets from the existing calibration. `current_relative` shows the current Follower pose and Leader calibration-home references; those home values are not required relative-start targets. Read-only connection shows the actual pose. Displayed goals never command the arms automatically. SDK modules load from the bound build directory; file and loaded-library checks remain strict.

```bash
conda activate lumos-lerobot
bash scripts/run_feedback.sh
```

打开 `http://127.0.0.1:8766/`，接好硬件后点击“连接真机（只读）”。服务启动时不自动连接；该按钮只查询 CAN 状态，不构造 SDK、不使能、不跟随、不写 Leader 输出。停止或连接错误后页面保留，重试必须手动点击。停止采集后，可在同一页面选择“基础跟随”或“跟随并监测”；这两项通过启动姿态预检后会使能并实际跟随，不写 Leader 阻尼。“力反馈”仅对已审核配置开放，未显式授权的工程配置禁止力输出；0.6.20 可单独配置受限 `experiment`，不开放正式 `run`。命令行保留相同模式；更改源码或资源后重新生成绑定。源码包不含本机配置/校准/绑定/运行数据，不含构建安装包。

Open `http://127.0.0.1:8766/` and select “Connect hardware (read only)” after connecting the devices. Startup does not connect automatically. This button queries CAN state without constructing the SDK, enabling motors, following, or writing Leader output. The page survives stops and connection errors; retries are explicit. After stopping inspection, select basic following or following with monitoring on the same page. Both enable motors and follow after the startup-pose check, without Leader damping. Force feedback requires a reviewed profile; an ordinary engineering profile blocks force output; 0.6.20 separately supports explicitly authorized bounded `experiment`, without enabling formal `run`. The same modes remain available through the CLI. Regenerate the binding after source/resource edits. Source releases exclude local setup, calibration, bindings, runtime data and built installers.

力反馈核心提供 `lumos-feedback`：整臂动力学补偿、逐轴动态阻尼、独立 HOLD、真实 CAN/串口帧检查及中英文网页，并修正 TypeLJ 非线性开度转换。新增 `prepare` 工程观测配置、无使能的 `inspect` 实时网页、内存静止统计和夹爪目标限速。工程 v3 配置不允许力输出；已审核力反馈配置使用 v2。已读通真机七轴状态，真实跟随和力反馈尚未验证。

The feedback core provides `lumos-feedback`: whole-arm dynamics, per-axis damping, independent HOLD, fresh CAN/serial checks and a bilingual dashboard, with corrected nonlinear TypeLJ opening conversion. It adds `prepare` engineering profiles, a non-enabling `inspect` dashboard, in-memory stationary statistics and gripper target limiting. Ordinary engineering v3 profiles cannot produce force; explicit v3 `experiment` has separate bounded authorization, while reviewed formal force profiles use v2. Genuine seven-axis feedback has been read; physical following and force feedback remain unvalidated.

已准备本机工程配置和 `--purpose inspect` 绑定后，可先离线查看三维页面：

```bash
lumos-feedback preview --profile configs/commissioning.local.json --binding configs/commissioning.binding.json --port 8766
```

打开 `http://127.0.0.1:8766/`。此命令不构造或调用设备 SDK，不访问 CAN/串口，不发送动作或力输出；参考姿态明确标识，不能作为真机遥测。三维页面加载 [Web-ACT 固定提交的 LumosTouch V3 CAD 网格](https://github.com/VincentHSS/Star-Arm-102-Web-ACT/tree/445f46f8756d9b826b17687dbdbb0c439537efcd/Browser_SDK/assets/urdf/fl-v3)。六轴关节坐标与现用 SDK V2 一致，显示使用现有 URDF 角度；动力学质量、限位与校准保持原配置。V3 夹爪外形尚未确认为 TypeLJ 精确几何，显示开口跟随 TypeLJ 实际开度。可旋转、缩放，连接真实遥测后同步姿态。配置与来源说明见[力反馈说明](docs/feedback.zh.md)。

With a local engineering profile and a binding generated using `--purpose inspect`, the command above opens an offline 3D view at `http://127.0.0.1:8766/`. It does not construct or call the device SDK, access CAN/serial, or send motion or force commands. Its labeled reference pose is not hardware telemetry. The view loads [LumosTouch V3 CAD meshes from a pinned Web-ACT commit](https://github.com/VincentHSS/Star-Arm-102-Web-ACT/tree/445f46f8756d9b826b17687dbdbb0c439537efcd/Browser_SDK/assets/urdf/fl-v3). Its six arm joint frames match the current SDK V2 model, so existing URDF angles drive the display. Dynamics masses, physical limits and calibration retain their configured sources. The V3 gripper has not been confirmed as exact TypeLJ geometry; its displayed opening follows actual TypeLJ opening distance. Rotate or zoom the view; its pose follows genuine telemetry when connected. See the [feedback guide](docs/feedback.md) for setup and attribution.

教程按 **环境搭建 → 接线与端口 → 校准 → 遥操作 → 关节数据采集 → 回放** 编写，暂不配置摄像头。

## 设备接入

- `--robot.type=lumos`：Lumos 单臂，CAN / SingleArm SDK。
- `--teleop.type=stararm102`：StarArm102 主臂，UART / FashionStar SDK，保留 HD 按钮和配对校准映射。
- 关节使用绝对位置，单位 rad；夹爪使用 0～1 开度。
- LeRobot 原生命令自动发现本插件，支持遥操作、关节数据采集与回放。

## 安装

在 Ubuntu 22.04 / 24.04 x86_64 的 Python 3.12 环境中：

```bash
git clone https://github.com/VincentHSS/stararm102-lumos-lerobot.git
cd stararm102-lumos-lerobot
bash scripts/install_lerobot_ubuntu.sh
```

首次使用按 [完整教程](docs/stararm102-lumos.md) 完成环境、硬件、校准和限位检查，再启动执行模式。SDK 初始化可能使能电机；基础启动器的 `lumos-lerobot preview` 也会连接 SDK。上面的 `lumos-feedback preview` 为离线三维查看。

## 配置文件入口

除教程中的 LeRobot 原生命令，也保留 `lumos-lerobot` 统一入口：

```bash
lumos-lerobot doctor --config configs/local.json
lumos-lerobot preview --config configs/local.json
```

配置模板见 [configs/example.json](configs/example.json)。该入口从配置文件读取端口和路径，不读取教程中的 `STARARM_PORT` 等环境变量；使用时须保持配置与校准文件一致。`teleop`、`record` 模式会真实发送动作。

## 代码位置

| 文件 | 职责 |
|---|---|
| `src/lerobot_robot_lumos_stararm102/config.py` | 主从臂注册和参数 |
| `stararm102.py` | UART 读取、HD 按钮、滤波、主从映射 |
| `lumos.py` | CAN SDK、实测状态、目标限位限速与发送 |
| `common.py` | 校准文件、字段与公共数学处理 |
| `cli.py` | 配置文件启动入口 |
| `tools/patch_record.py` | 让固定版本录制限速后的实际下发目标 |

## 验证与来源

软件测试与硬件验证范围见 [VALIDATION.md](VALIDATION.md)。完整测试：

```bash
python -m unittest discover -s tests -p 'test_*.py' -v
```

原脚本、原安装脚本和原单元测试来自 [VincentHSS/stararm102-lumos-teleop](https://github.com/VincentHSS/stararm102-lumos-teleop)，原说明保存在 [README_STANDALONE.md](README_STANDALONE.md)。原仓库未声明开源许可证，本仓库不替原作者授予额外的复制、修改或分发许可。
