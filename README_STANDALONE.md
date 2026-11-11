# StarArm102 → Lumos command-line guide

[简体中文](#简体中文) / [English](#english)

## 简体中文

本项目的基础遥操作入口是 `lumos-lerobot`，力反馈和网页入口是 `lumos-feedback`。此源码发布不包含旧的根目录 `stararm102_lumos_teleop.py` 或 Word 操作手册，也不提供脱离 LeRobot 的独立安装方案。

使用 Linux、Python 3.12，先阅读[安装与基础遥操作指南](docs/stararm102-lumos.md)。已经有可用配对校准时，保留原文件，并核对新配置中的设备、关节对应关系和方向。不要把其他机械臂的校准或力反馈配置直接套用。

安装和本机配置完成后，可查看命令帮助：

```bash
lumos-lerobot --help
lumos-feedback --help
```

基础预览和跟随使用：

```bash
lumos-lerobot preview --config configs/local.json
lumos-lerobot teleop --config configs/local.json
```

这两条命令会连接硬件。SDK 初始化可能使能 Lumos 电机并关闭夹爪，基础预览仅表示不发送跟随目标。按钮功能也可能改变 Leader 的保持状态。新款原生按钮板应使用[反馈启动流程](docs/feedback.zh.md)中支持原生事件监听的入口。

力反馈需要另行建立本机硬件记录并生成正常绑定。按[力反馈说明](docs/feedback.zh.md)完成离线模型检查、基础跟随和仅监测后，再验证实验阻尼。`lumos-feedback preview` 是离线模型页，与上述连接硬件的 `lumos-lerobot preview` 不同。

本仓库不包含本机校准、绑定、硬件调参文件或运行数据。软件测试结果见[验证说明](VALIDATION.md)，不等于真机效果已经验证。

## English

Use `lumos-lerobot` for basic teleoperation and `lumos-feedback` for the feedback dashboard and guarded control modes. This source release does not include the former root-level `stararm102_lumos_teleop.py`, the Word manual or a standalone installation without LeRobot.

Use Linux and Python 3.12. Follow the [installation and basic teleoperation guide](docs/stararm102-lumos.md), preserving an existing valid paired calibration and checking device settings, joint correspondence and direction. Do not reuse another assembly's calibration or feedback configuration without verification. The help and launch commands above are the current entry points.

Both basic `preview` and `teleop` connect hardware. SDK construction may enable Lumos motors and close the gripper; basic preview only suppresses following targets. Button handling can also change Leader holding. Native button boards require an entry point with native-event listening, as described in the [feedback guide](docs/feedback.md).

Feedback requires a local hardware profile and a normally generated binding. Complete offline model checks, basic following and monitoring before experimental damping. `lumos-feedback preview` is an offline model page; `lumos-lerobot preview` connects hardware.

Local calibration, bindings, hardware tuning and runtime data are excluded from this repository. [Software validation](VALIDATION.md) does not establish physical following or damping performance.
