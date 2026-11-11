# StarArm → Lumos Feedback

StarArm 102 主臂与 Lumos 从臂的遥操作、剩余力矩阻尼和本地三维控制台

Teleoperation, torque-residual damping and a local 3D dashboard for StarArm 102 and Lumos

**Version 0.6.27** · [中文启动](docs/hardware.zh.md) · [Hardware startup](docs/hardware.md) · [软件验证 / Validation](VALIDATION.md)

0.6.27 增加可选 TypeLJ 跟随增益：保留 SDK 初始化默认值，在跟随时通过距离接口发送增益，并在两端各 10% 开度内平滑回到 SDK 默认增益。沿用七轴基线到位诊断；保护阈值和阻尼策略不变。

Version 0.6.27 adds optional TypeLJ following gains through the SDK distance interface. Native startup defaults are preserved, and gains smoothly return to SDK defaults within each endpoint’s 10% opening band. Seven-axis baseline diagnostics remain available; protection thresholds and damping policy are unchanged.

## 中文

本项目沿用 [StarArm102 → Lumos 遥操作](https://github.com/VincentHSS/stararm102-lumos-lerobot) 的通信和配对映射，加入整臂动力学、逐轴阻尼与网页。目标组合为 Fashionstar 串口总线 StarArm 102、Lumos 六轴臂与 TypeLJ 夹爪，底座正装。更换构型或硬件后需核对模型、单位、方向、映射和范围。

- 遥操作复用 LeRobot 适配器、现有配对校准、逐轴目标限速和输入滤波
- J1–J6 使用整臂 URDF、实时姿态、速度和估计加速度计算理论空载力矩；各轴实测力矩减模型力矩后，分别滤波、判断负载、映射功率并限制升降速率
- TypeLJ 夹爪使用自身电机力矩独立生成阻尼，不经过六轴动力学补偿
- 中英文网页显示实测与目标姿态、扭矩、电流、跟随误差和 Leader 请求状态，包含真实 URDF/STL 显示资源、本地图标与过渡动效
- 三维仅在已收姿态间插值，约 200 ms 显示缓冲；失鲜或停止后冻结最后实测姿态，不改变控制频率
- 力反馈控制台的遥测在内存传递，只保留本机绑定和当前设置；本仓库不含标定、绑定、运行数据或构建安装包

普通阻尼按剩余力矩绝对值调节强度，不精确复现带方向的外力。剩余力矩包含摩擦和模型误差，不能直接换算成准确末端力或重量。mW 是 Leader 请求值，不是实测机械阻力。当前实验路径关闭软件和自动 HOLD。

### 模式

| 模式 | 行为 |
|---|---|
| `serve` / `preview` | 页面或模型预览，不自动开始跟随 |
| `inspect` | 读取硬件和检查姿态，不发送跟随目标 |
| `follow` | 基础跟随，不输出软件阻尼 |
| `monitor` | 跟随并计算动力学与剩余力矩，不输出软件阻尼 |
| `experiment` | 显式授权的实验阻尼，需要真实空载声明、按钮释放及启动基线 |
| `run` | 需要单独审核、授权及绑定的正式力反馈配置 |

先完成离线验证、基础跟随和仅监测，再验证实际阻尼。SDK 初始化可能使能电机并闭合夹爪；退出控制可能失去从臂支撑。基础跟随不代表动力学或阻尼已验收，具体启动条件见[硬件指南](docs/hardware.zh.md)。

### 安装和配置

推荐 Linux x86-64、Python 3.12。安装脚本固定 LeRobot 与 Lumos SDK 版本，需网络和安装系统依赖的权限，不会连接机械臂。

```bash
git clone https://github.com/VincentHSS/stararm-lumos-feedback.git
cd stararm-lumos-feedback
# 先激活 Python 3.12 环境
bash scripts/install_lerobot_ubuntu.sh
lumos-feedback --version
```

反馈使用的 SDK 兼容修正及绑定流程见 [SDK 审核说明](docs/sdk-startup-review.md)。按[配对校准指南](docs/stararm102-lumos.md)准备有效校准、真实限位和本机 session，再按[反馈配置说明](docs/feedback.zh.md)填写硬件模板。

```bash
lumos-feedback template --output configs/hardware.local.json
# 完整填写模板，并准备其中引用的 session、SDK、URDF 和校准
lumos-feedback prepare --profile configs/hardware.local.json \
  --output configs/commissioning.local.json
lumos-feedback check --profile configs/commissioning.local.json --purpose inspect
lumos-feedback bind --profile configs/commissioning.local.json \
  --binding configs/commissioning.binding.json --purpose inspect
bash scripts/run_feedback.sh serve
```

打开 [http://127.0.0.1:8766/](http://127.0.0.1:8766/)，明确选择模式，从只读检查和基础跟随开始。模板保留待填写字段，不能直接用于真机。不要复制其他机械臂的标定或参数；修改源码、模型、SDK、映射或策略后，保留原校准并正常重新生成绑定，不手改哈希绕过检查。

网页只允许调整普通功率上限：共同 400–1500 mW，每轴 0–1500 mW，0 关闭该轴；有效上限取共同与该轴上限的较小值，实际请求随负载变化。死区、运动余量、满输出标尺和响应策略不能从网页修改。实验模式的额外上限和授权仍由本机配置决定。

文档：[反馈配置](docs/feedback.zh.md) · [硬件启动](docs/hardware.zh.md) · [SDK 审核](docs/sdk-startup-review.md) · [第三方来源](docs/THIRD_PARTY.md)

## English

This project extends the communication and paired mapping from [StarArm102 → Lumos teleoperation](https://github.com/VincentHSS/stararm102-lumos-lerobot) with whole-arm dynamics, per-axis damping and a web dashboard. The target assembly is a Fashionstar serial-bus StarArm 102 Leader and a six-axis Lumos Follower with a TypeLJ gripper, mounted upright. A different configuration requires review of the model, units, signs, mapping and limits.

- Teleoperation reuses the LeRobot adapters, paired calibration, input filtering and per-axis target limits
- J1–J6 damping subtracts whole-arm URDF dynamics from measured torques using live pose, velocity and estimated acceleration; each residual independently passes filtering, load confirmation, power mapping and slew limits
- Gripper damping uses the TypeLJ motor torque independently of six-axis compensation
- The bilingual dashboard includes sourced URDF/STL geometry, measured and target poses, torque, current, tracking errors and Leader request state, with local icons and transitions
- Smooth presentation interpolates only received poses with about 200 ms of display buffering; stale or stopped state freezes at the latest measurement without changing control scheduling
- Feedback telemetry is memory-only, with local bindings and current settings persisted separately; this repository includes no calibration, bindings, runtime data or built installers

Ordinary damping follows residual magnitude rather than reproducing a precise signed external force. Residuals include friction and model error and cannot directly measure accurate end-effector force or weight. Leader mW values are requests, not measured mechanical resistance. Software and automatic HOLD are disabled in the experimental path.

### Modes

| Mode | Behavior |
|---|---|
| `serve` / `preview` | Dashboard or model preview without automatic following |
| `inspect` | Read hardware and inspect poses without following targets |
| `follow` | Basic following with no software damping |
| `monitor` | Follow and compute dynamics/residuals without software damping |
| `experiment` | Explicit experimental authorization, real unloaded declaration, button release and startup baseline required |
| `run` | Formal feedback requiring a separately reviewed, authorized and bound profile |

Validate offline, then basic following, then monitoring before physical damping. SDK initialization may enable motors and close the gripper; exiting control can remove Follower support. Basic following does not establish dynamics or damping acceptance. See [hardware startup](docs/hardware.md) for admission conditions.

### Installation and configuration

Use Linux x86-64 and an activated Python 3.12 environment. The installation commands above pin LeRobot and Lumos SDK revisions and require network access and permission to install system dependencies. Installation does not connect hardware. Feedback-specific SDK compatibility builds and binding requirements are covered in the [SDK review](docs/sdk-startup-review.md).

Prepare valid paired calibration, verified limits and a local session. Generate and complete the hardware template, then run `prepare`, offline `check` and normal `bind` as shown above. Start `bash scripts/run_feedback.sh serve` and open [http://127.0.0.1:8766/](http://127.0.0.1:8766/); explicitly select a mode, starting with inspection and basic following. Templates contain deliberately unfilled fields and cannot drive hardware as distributed. Preserve calibration and regenerate the binding after source, model, SDK, mapping or policy changes. Never edit hashes to bypass verification.

The page edits only ordinary common/per-axis caps: common 400–1500 mW, each axis 0–1500 mW, with zero disabling that axis. The effective cap is the smaller of the common and axis caps; requests vary with load. Deadzones, motion allowances, full-output scales and response policies cannot be changed through the page. Local authorization and additional experimental ceilings still apply.

Documentation: [Feedback configuration](docs/feedback.md) · [Hardware startup](docs/hardware.md) · [SDK review](docs/sdk-startup-review.md) · [Third-party sources](docs/THIRD_PARTY.md)

## Source layout

| Path | Purpose / 用途 |
|---|---|
| `src/lerobot_robot_lumos_stararm102/` | Communication, paired mapping, LeRobot adapters / 通信、映射与适配器 |
| `src/stararm_lumos_feedback/model.py` | Whole-arm dynamics / 整臂动力学 |
| `src/stararm_lumos_feedback/control.py` | Residual-to-power policy / 剩余力矩功率映射 |
| `src/stararm_lumos_feedback/runtime.py` | Session scheduling and checks / 会话调度与检查 |
| `src/stararm_lumos_feedback/baseline.py` | Stationary unloaded baseline / 静止空载基线 |
| `src/stararm_lumos_feedback/web/` | Dashboard, viewer and resources / 页面、三维显示与资源 |
| `tests/` | Device-free regression tests / 无设备回归测试 |
| `tools/` | Model checks, SDK compatibility and source export / 模型检查、SDK 兼容与源码导出 |

593 software tests passed for 0.6.27. Software results do not establish physical tracking or force-feedback acceptance. See [VALIDATION.md](VALIDATION.md).

0.6.27 已通过 593 项软件测试；这不代表真机跟随或力反馈效果已验收。第三方来源与许可沿用各自声明，本仓库不为模型或全部资产新增统一授权。
