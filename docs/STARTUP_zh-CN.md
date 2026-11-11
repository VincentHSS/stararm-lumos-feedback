# StarArm102 + Lumos 启动入口

首次使用请阅读[安装与基础遥操作](stararm102-lumos.md)，完成环境、配对校准、真实限位及关节方向核对。力反馈和三维网页的本机配置、SDK 检查与绑定步骤见[反馈说明](feedback.zh.md)。英文说明见 [Feedback guide](feedback.md)。

本仓库的真实命令入口是 `lumos-lerobot` 和 `lumos-feedback`，不包含旧的根目录遥操作脚本。SDK 初始化可能使能电机并关闭夹爪；基础预览同样会连接 SDK。

已准备并核验本机反馈配置和绑定后，从仓库根目录启动网页：

```bash
bash scripts/run_feedback.sh serve
```

默认打开 `http://127.0.0.1:8766/`，等待显式选择模式。这不是自动开始运动的命令，运动模式仍须通过对应启动检查。本仓库不提供可直接用于任意设备的本机校准、绑定或调参文件。

只看模型、完全不连接硬件时，使用反馈说明中的 `lumos-feedback preview`。它与会连接硬件的 `lumos-lerobot preview` 不同。
