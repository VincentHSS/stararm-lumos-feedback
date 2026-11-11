# Software validation · 0.6.26

## 中文

0.6.26 发布前，581 项软件测试全部通过，无跳过。Ruff、JavaScript 模块语法、Shell 语法及差异空白检查通过。新增诊断测试覆盖七轴目标与实测误差、原门控结果保持不变、停止后保留诊断，以及网页的单位、显隐和字段完整性。发布整理清理说明中的本机历史，不分发本机参数或运行数据。

覆盖范围包括整臂动力学与坐标单位、映射、限位、滤波及目标限速；CAN 时效、故障、温度、位置跳变与 SDK/CAN 一致性；按钮协议、原生事件门控、会话所有权、取消和清理；实验功率与速率契约、加速度观察器和空载基线采集时机；实测边界余量与严格目标边界；网页配置边界、稳定 DOM 和草稿；三维已收样本插值、不外推、失鲜冻结、静止停帧；SDK 构造函数及原生量程修正的源码回归。

使用已安装依赖的 Python 3.12 环境，在源码根目录执行：

```bash
PYTHONPATH=src python -B -m unittest discover -s tests
ruff check src tests tools
node --input-type=module --check < src/stararm_lumos_feedback/web/app.js
node --input-type=module --check < src/stararm_lumos_feedback/web/viewer.js
bash -n scripts/*.sh
git diff --check -- . ':!tests/fixtures/sdk_0404125_pybind_constructor.cpp'
```

测试替换硬件通信，使用合成配置和临时目录，不要求连接或使能机械臂。Node 和 Ruff 是开发检查工具。空白检查排除逐字节保留的上游 SDK 测试夹具，其原有行尾空格属于固定哈希输入。三维资源的来源和哈希见 [SOURCE.json](src/stararm_lumos_feedback/web/models/lumos-touch-v3/SOURCE.json)。

软件测试通过不代表真机跟随、全姿态空载分离、带载响应、原生按钮仲裁或阻尼手感已验收。显示插值不会修复 CAN 缺帧。单姿态空载基线不能证明所有运动状态下均无误阻尼；剩余力矩不是准确末端力。`experiment` 仍为实验功能，`run` 需要另行审核的本机配置与授权。

## English

All 581 software tests passed without skips before the 0.6.26 release. Ruff, JavaScript module syntax, shell syntax and diff whitespace checks also passed. New regressions cover seven-axis errors and tolerances, unchanged admission decisions, diagnostic retention after stop, and web units, visibility and completeness. Publication edits remove machine-specific documentary history; local parameters and runtime data are excluded.

Coverage includes whole-arm dynamics and coordinate conversion; mapping, filtering and target limits; CAN freshness and faults; temperature, position and SDK/CAN agreement; button protocols and native-event gating; session ownership, cancellation and cleanup; experimental power/slew contracts and baseline timing; measured margins versus strict command limits; web API and immutable policies; stable DOM and cap drafts; motion preferences; received-pose interpolation without extrapolation; stale freezing and idle rendering; and source regressions for the SDK constructor and motor-range compatibility patches.

Run the commands above from the source root in a Python 3.12 environment with dependencies installed. Tests replace hardware I/O and use synthetic configuration and temporary directories; they do not require connecting or enabling a robot. Node and Ruff are development tools. The whitespace check excludes the byte-exact upstream SDK fixture because its original trailing spaces are part of the fixed-hash input. Geometry provenance and hashes are recorded in [SOURCE.json](src/stararm_lumos_feedback/web/models/lumos-touch-v3/SOURCE.json).

These results do not establish acceptance of physical tracking, unloaded separation over all poses, loaded response, native-button arbitration or damping feel. Presentation interpolation does not repair CAN loss. A single-pose baseline cannot validate every moving condition, and residual torque is not accurate end-effector force. `experiment` remains experimental; `run` requires a separately reviewed local profile and authorization.
