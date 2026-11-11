# Software validation · 0.6.31

## 中文

0.6.31 发布前，665 项软件测试全部通过，无跳过。新增 15 项回归覆盖显式空载声明、接口鉴权、零上限及原生释放准入、排队与采集中状态变化、取消后不恢复旧参考或自动补学、基线引用替换、跟随会话与目标保留、连接状态锁顺序，以及中英文页面交互。Ruff、JavaScript 模块语法、Shell 语法和差异检查通过。重新采集仍只建立当前姿态附近的参考；这些测试不证明全空间空载分离或真机阻尼手感。

0.6.30 发布前，650 项软件测试全部通过，无跳过。新增 21 项局部参考回归覆盖受力使残差绝对值下降、双向与跨零变化、单次连续窗冻结、全七轴姿态和静止门控、原始夹爪速度、模型变化余量、固定死区不可绕过、独立预热和确认、缺帧／禁止输出／离开区域后的状态清除、有限卸载尾迹、功率上限以及原绝对残差通道兼容性。另有 2 项本机契约和绑定测试、1 项双语页面显示回归。Ruff、JavaScript 模块语法、Shell 语法和差异检查通过。局部邻域仍为工程假设；这些合成测试不证明全空间空载分离或真机阻尼手感。

0.6.29 发布前，626 项软件测试全部通过，无跳过。新增 20 项回归覆盖初始化前补齐在途组、阻塞预检和 SDK 初始化期间持续接收、同批完整组后新半组、真实缺帧与原异常传播、取消、单读者串口锁、接收预算和线程停止交接。保留原 250 ms 半组期限、完整六轴确认和阻尼／运动保护。软件复现可重现并修复初始化暂停接收导致的同类错误，尚未确认本次真机故障的实际帧到达时间，也未验证新版真机手感。

0.6.28 发布前，606 项软件测试全部通过，无跳过。Ruff、JavaScript 模块语法、Shell 语法及差异空白检查通过。新增 13 项回归覆盖六轴／夹爪分组准入与静止窗口、夹爪静差不再阻断已就绪六轴、J4 独立负载响应、公共条件不可被分组覆盖、首组完成后关闭采集且运行中不补学、故障全轴归零，以及中英文分组显示与稳定 DOM。到位阈值、物理范围、整臂动力学、原生按钮及 SDK 初始化保持原实现。

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

All 665 software tests passed without skips before the 0.6.31 release. Fifteen added regressions cover explicit unloaded declarations, endpoint authentication, zero-cap and native-release admission, changes while queued or collecting, cancellation without restoring an old reference or automatic learning, baseline alias replacement, preservation of the following session and targets, connection lock ordering, and bilingual page interaction. Ruff, JavaScript module syntax, shell syntax and diff checks passed. Recapture still establishes only a reference near the current pose; these tests do not establish unloaded separation across the workspace or hardware damping feel.

All 650 software tests passed without skips before the 0.6.30 release. Twenty-one local-reference regressions cover loads that reduce absolute residual, both directions and zero crossing, a single continuous reference window, seven-axis pose and stationary gates, raw gripper velocity, model-change allowances, the fixed-deadzone floor, independent warmup and confirmation, state clearing after gaps/inhibition/out-of-region conditions, finite unloading decay, caps and compatibility with the original absolute channel. Two contract/binding tests and one bilingual dashboard regression were also added. Ruff, JavaScript module syntax, shell syntax and diff checks passed. The local neighborhood remains an engineering assumption; synthetic tests do not establish unloaded separation over the whole workspace or hardware damping feel.

All 626 software tests passed without skips before the 0.6.29 release. Twenty added regressions cover pending-group completion, RX continuity through blocking preflight and SDK initialization, a new partial group following a complete group in one read, real missing frames and original exception propagation, cancellation, single-reader UART locking, receive budgets and stopped/joined handoff. The 250 ms group deadline, six-axis confirmation and existing motion/damping guards remain. The software reproduction fixes a matching error caused by interrupted startup reception; arrival times in the failed physical session and the new hardware behavior remain unverified.

All 606 software tests passed without skips before the 0.6.28 release. Ruff, JavaScript module syntax, shell syntax and diff whitespace checks also passed. Thirteen added regressions cover grouped arm/gripper admission and stationary windows, gripper arrival error without blocking a qualified arm, independent J4 load response, mandatory common admission, collection closing before output without later learning, common-fault zero output, and bilingual group status with stable DOM. Arrival tolerances, physical ranges, whole-arm dynamics, native-button handling and SDK initialization are unchanged.

Coverage includes whole-arm dynamics and coordinate conversion; mapping, filtering and target limits; CAN freshness and faults; temperature, position and SDK/CAN agreement; button protocols and native-event gating; session ownership, cancellation and cleanup; experimental power/slew contracts and baseline timing; measured margins versus strict command limits; web API and immutable policies; stable DOM and cap drafts; motion preferences; received-pose interpolation without extrapolation; stale freezing and idle rendering; and source regressions for the SDK constructor and motor-range compatibility patches.

Run the commands above from the source root in a Python 3.12 environment with dependencies installed. Tests replace hardware I/O and use synthetic configuration and temporary directories; they do not require connecting or enabling a robot. Node and Ruff are development tools. The whitespace check excludes the byte-exact upstream SDK fixture because its original trailing spaces are part of the fixed-hash input. Geometry provenance and hashes are recorded in [SOURCE.json](src/stararm_lumos_feedback/web/models/lumos-touch-v3/SOURCE.json).

These results do not establish acceptance of physical tracking, unloaded separation over all poses, loaded response, native-button arbitration or damping feel. Presentation interpolation does not repair CAN loss. A single-pose baseline cannot validate every moving condition, and residual torque is not accurate end-effector force. `experiment` remains experimental; `run` requires a separately reviewed local profile and authorization.
