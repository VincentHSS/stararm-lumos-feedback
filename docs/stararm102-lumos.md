# StarArm102 + Lumos 安装与基础遥操作

本指南说明本仓库的环境、配对校准和基础跟随入口。力反馈和网页启动见[反馈说明](feedback.zh.md)，英文说明见 [Feedback guide](feedback.md)。本仓库基于[原 LeRobot 适配项目](https://github.com/VincentHSS/stararm102-lumos-lerobot)扩展，运行命令以本仓库当前 CLI 为准。

## 环境与安装

使用 Linux、Python 3.12 和 SocketCAN。安装器固定 LeRobot 提交 `3991580a5673f5f0219a4e8835a5f973eeccf9a7` 和 Lumos SDK 提交 `040412542c0b6b6fd2bb6dd652c0de6c666631be`，使用当前激活的 Python 环境。系统依赖安装需要 `sudo`，下载需要网络。

已有 Python 3.12 环境时直接激活。使用 Conda 时可创建专用环境：

```bash
conda create -n lumos-feedback python=3.12 -y
conda activate lumos-feedback
git clone https://github.com/VincentHSS/stararm-lumos-feedback.git
cd stararm-lumos-feedback
bash scripts/install_lerobot_ubuntu.sh
```

安装器不会连接机器人。它会在 `.dependencies/` 中保留 SDK 和 LeRobot 源码，安装本包，并在不存在时将 `configs/example.json` 复制为 `configs/local.json`。不要删除 SDK 的 `src/config/`；SDK 必须从同一源码树读取 `robot_kinematics.yaml` 及其引用文件。

安装后执行：

```bash
python -m pip check
lumos-lerobot --help
lumos-feedback --version
```

已有环境可以用 `python -m pip install -e .` 安装本包，但这一步不会准备或修补 SDK。反馈运行要求的 SDK 构建检查见 [SDK 审核说明](sdk-startup-review.md)，不能只安装任意版本后绕过绑定。

## 连接与本机配置

StarArm102 使用 USB/UART，Lumos 使用 CAN。按厂家要求完成供电、总线终端和急停接线，避免多个控制进程同时使用同一机械臂。

```bash
ls -l /dev/serial/by-id/
ip -details link show can0
```

以下示例使用 Leader `/dev/ttyUSB0`、Follower `can0`。CAN 接口存在且尚未启用时：

```bash
sudo ip link set can0 up type can bitrate 1000000
```

已为 UP 且波特率正确时无需重复设置。修改 CAN 配置前先退出控制。串口权限不足时将用户加入 `dialout` 组并重新登录，不要通过 `sudo` 启动另一套 Python 环境。

编辑本机文件 `configs/local.json`，至少核对：

| 配置 | 含义 |
| --- | --- |
| `robot.sdk_root` | 完整 SDK 源码根目录，含 `src/config/robot_kinematics.yaml` |
| `robot.can_interface`、`teleop.port` | 实际 CAN 接口和 Leader 串口 |
| 两端的 `mapping_file` | 同一份配对校准文件 |
| `robot.joint_limits_file` | 实物型号对应的 SDK 坐标限位文件 |
| 两端的 `gripper` | 是否启用夹爪，两端设置须一致 |
| `teleop.joint_map`、`signs`、`gain` | 六轴对应关系、方向及映射比例 |
| `teleop.button_enabled`、`button_id` | 查询式旧按钮板设置，无按钮板才关闭 |

配置中的相对路径以 JSON 文件所在目录为基准。示例 JSON 是格式示例，不是已经验证的实物配置；其中速度、映射及采集字段也需要按用途检查。基础配置的 `execute` 会被 CLI 模式覆盖：`preview` 为 false，`teleop` 和 `record` 为 true。

新款按钮板可自行发送六轴保持/释放指令，可能不回应旧款 ID 查询。反馈入口提供原生命令监听和独立按钮板选择；基础 LeRobot 入口不支持该事件链路。识别到新板时，按[反馈说明](feedback.zh.md)使用对应入口，不要用“无按钮板”掩盖协议差异。

## 配对校准

**校准和基础预览会构造 Lumos SDK。SDK 初始化可能使能电机并关闭夹爪，基础预览不是断电或纯离线状态。** 保持机器人稳定、工作区畅通，急停可及。

新装配首次使用，或关节对应关系改变后，使用本机配置启动配对向导：

```bash
lumos-lerobot calibrate --config configs/local.json
```

该向导记录主从参考姿态、Leader 关节范围和启用时的夹爪开合端点，不下发跟随目标。按提示让两臂处于对应姿态，再缓慢覆盖计划使用的 Leader 范围；不要强行触碰机械限位。它会覆盖配置指定的校准文件，需要保留旧文件时先自行备份。

此基础向导使用查询式旧按钮板设置。原生新板的兼容性必须单独核实，已有有效配对校准可在设备、映射和装配一致时继续使用。分别调用 LeRobot 的单设备校准不能替代本项目配对参考姿态的采集。

默认示例路径为 `~/.config/stararm102_lumos/calibration.json`。校准是本机数据，不上传到源码仓库。修改动力学模型、策略或硬件记录后应重新生成反馈绑定，不能手改哈希绕过检查；这不要求无条件重做仍有效的配对校准。

## 准备 Follower 限位

执行模式需要六轴真实限位，Leader 扫描范围不能替代 Follower 限位。确认实物使用 SDK 的 `FastTouchV2.SLDASM.urdf`，且 `joint1` 至 `joint6` 顺序和方向与 SDK 一致后，可导出：

```bash
python tools/export_joint_limits.py --urdf .dependencies/startouch_sdk/src/config/FastTouchV2.SLDASM.urdf --output lumos_joint_limits.json
```

其他型号应改用对应模型和关节名。导出工具不会覆盖已有文件；使用前核对 `lower_rad`、`upper_rad`，单位为 SDK 关节坐标的 rad。不要扩大厂家限位来掩盖映射或零位错误。

## 离线检查与基础跟随

配对校准和 SDK 文件准备好后，以下检查不打开端口或构造 SDK：

```bash
lumos-lerobot doctor --config configs/local.json
lumos-lerobot preview --config configs/local.json --print-command
```

`doctor` 检查配置、已安装模块、LeRobot 工厂及本机文件，不证明实际通信正常。`--print-command` 只打印计划命令，不启动机器人。

使用查询式旧按钮板或无按钮板，并完成对应设置时，可先运行基础预览：

```bash
lumos-lerobot preview --config configs/local.json
```

基础预览连接硬件，但不发送 Follower 跟随目标；按钮处理仍可能改变 Leader 保持状态。一次缓慢检查一个关节，确认关节顺序、方向和夹爪开合方向。原生新按钮板请改走反馈说明中的支持入口。

预览正常后退出，将 Follower 恢复到配对校准参考姿态附近，再请求基础跟随：

```bash
lumos-lerobot teleop --config configs/local.json
```

该命令会实际使能和跟随，保留基础启动姿态检查及目标限速。默认参数来自配置类或本机 JSON，不应以旧脚本顶部常量为依据。各轴目标独立限速，不保证同时到位。`Ctrl+C` 退出；退出控制可能失去从臂支撑，应先放稳。

## 进入力反馈流程

基础跟随与力反馈配置是两层。本机 session JSON 保存通信和原有映射，反馈 profile 另行记录七轴协议、单位、动力学、输出策略及保护条件。按[反馈说明](feedback.zh.md)建立 profile、离线检查、生成绑定，再依次进行只读观测、基础跟随、仅监测和实验阻尼验证。

在已准备并核验 `configs/commissioning.local.json` 及其绑定后，网页服务入口为：

```bash
bash scripts/run_feedback.sh serve
```

默认页面为 `http://127.0.0.1:8766/`，服务等待显式模式选择。仓库不附带这两份本机文件，首次下载不能直接用示例代替绑定。只看离线模型使用 `lumos-feedback preview`；启动页中的运动模式会构造 SDK 并使能机器人。具体模式差异、按钮板状态及实验授权以反馈说明为准。

采集、训练和回放不属于反馈会话。保留的 `lumos-lerobot record` 入口会实际控制并将数据写到本机，使用前应单独核对数据设置，不要与反馈会话同时运行。历史采集流程可参考[上游教程](https://github.com/VincentHSS/stararm102-lumos-lerobot/blob/main/docs/stararm102-lumos.md)，其中版本和命令需与实际环境核对。

软件验证范围见[验证说明](../VALIDATION.md)。测试通过不能代替本机通信、运动与阻尼感受的验证。
