# 0.6.13 SDK startup review / SDK 启动审查

The pinned SDK source and binary were reviewed on 2026-11-09. Version 0.6.13 also
passed device-free execution of the actual native range getter, MIT encoder and
state decoder with a source-built range correction. No arm controller was
constructed and no hardware commands were sent for those codec checks. Physical
holding, following and force feedback remain unvalidated.

2026-11-09 审查固定 SDK 源码及二进制；0.6.13 还通过源码构建的量程修正在无设备
条件下执行并验证了实际原生 getter、MIT encoder 和状态 decoder。该编解码验证未
构造机械臂控制器，也未发送硬件命令。真机保持、跟随和力反馈仍未验证。

Current result: the valid source correction and regenerated binding allow
`follow/monitor` to pass the native range gate, while every other startup and
motion guard remains. Missing/partial correction builds retain the original
rejection. Each installation must record its own model-coverage evidence. Passing
this gate does not authorize formal `run`: dynamic thresholds, measured response
and native-button arbitration require separate acceptance. The native-output
window before Python's fixed seed remains and requires stable physical support.

当前结论：有效源码修正及正常重新生成的绑定可使 `follow/monitor` 通过原生量程
门禁，其余启动及运动保护保留。未安装或部分修正构建仍被原门禁拒绝。当前 `run`
仍需单独验收动态阈值、实测响应及正式原生按钮力输出仲裁；通过本项不授权正式
`run`。每套装配应记录自己的模型覆盖证据。Python 首次固定目标前的原生输出窗口
仍存在，必须保留稳定支撑。

Public API review found no per-axis motor-type/range selection, and the normal
upstream build still copies a precompiled core. That finding remains valid, but
the earlier vendor-only repair conclusion was too narrow. The reviewed core's
three table references use the exported `MOTOR_LIMIT_PARAMS` symbol through its
GOT. The source compatibility layer below supplies that table before the first
core load and verifies the actual native codecs, rather than rescaling Python
values or editing the core binary.

公开接口仍没有逐轴类型／量程选择，上游正常构建仍复制预编译核心；这一事实保留，
但此前“只能由厂家修复”的结论过窄。已审查核心对导出 `MOTOR_LIMIT_PARAMS` 的三处
引用均经 GOT 读取。下述源码兼容层在核心首次加载前提供该表，并验证实际原生
编解码，不使用 Python 比例替代，也不修改核心二进制。

Gitee and GitHub main both point to `040412542c0b6b6fd2bb6dd652c0de6c666631be`
(SDK 0.1.9); all five branch tips and the four bundled native-library blob IDs
match. Switching repository addresses does not supply a different core or TypeLJ
inertial model. See the pinned Gitee [motor table](https://gitee.com/lu-ming-robotics---shenzhen--g/startouch_sdk/blob/040412542c0b6b6fd2bb6dd652c0de6c666631be/src/startouch/damiao_motor/dm_motor_constants.hpp#L104),
[binding constructor](https://gitee.com/lu-ming-robotics---shenzhen--g/startouch_sdk/blob/040412542c0b6b6fd2bb6dd652c0de6c666631be/src/pybind_0825testfast.cpp#L68)
and [build](https://gitee.com/lu-ming-robotics---shenzhen--g/startouch_sdk/blob/040412542c0b6b6fd2bb6dd652c0de6c666631be/CMakeLists.txt#L61).

Gitee 与 GitHub main 均为 `040412542c0b6b6fd2bb6dd652c0de6c666631be`（SDK 0.1.9），
五个分支 tip 和四份原生库 blob ID 均一致；切换仓库地址不会取得不同核心或 TypeLJ
惯量模型。固定提交的 Gitee 电机表、绑定构造及构建流程见上述链接。

## Pinned material / 固定资料

### Startup UART reception / 启动串口接收

A software reproduction found that a preflight query can finish its monitor replies after receiving native commands for IDs 0–4, then enter a blocking SDK constructor before consuming ID 5. The unchanged 250 ms parser deadline then expires before queued bytes are read. This reproduces the reported error; it does not establish the arrival timing of the failed physical session. Version 0.6.29 settles pending groups before initialization and keeps RX-only reception alive through preflight and SDK construction, using the same transport lock. The receiver stops and joins before handoff. True incomplete groups still fail; SDK binaries, confirmation rules and damping policy are unchanged.

软件复现发现，预检可能收齐监测回包及按钮 ID 0–4 命令后就进入阻塞的 SDK 构造，尚未读取 ID 5。原 250 ms 期限随后在读取排队字节前到期，可复现用户看到的错误，但不能据此确定失败真机会话的实际到达时间。0.6.29 在初始化前收齐在途组，预检和 SDK 构造期间使用同一传输锁持续只读接收，交接前停止并等待线程退出。真实半组仍会报错；SDK 二进制、六轴确认规则及阻尼策略不变。

### Constructor GIL correction / 构造执行锁修正

The pinned pybind constructor parses Python kwargs and then calls the native
constructor while holding the GIL. The TypeLJ native initialization includes five
20 ms receive waits plus subsequent waits, which can exceed configured CAN
freshness deadlines on its normal quiet-wait path. A Python CAN receiver cannot run during that
call, so genuine early gripper responses can queue and fail freshness checks.
This is a source/binary finding, not a measured hardware timing result.

固定 SDK 的 pybind 构造器在转换完 Python 参数后仍持有 GIL。TypeLJ 初始化含五轮
20 ms 接收等待及后续等待，常规静默等待路径可能超过配置的 CAN 时效上限。
同进程 Python CAN 线程无法在此期间运行，早到的真实夹爪帧可能积压并触发过期。
这项结论来自源码和二进制审查，并非真机构造耗时测量。

Version 0.6.7 supplies `tools/patch_sdk_constructor_gil.py` for an explicit SDK
copy. It accepts only the reviewed binding source (or its exact patched form),
converts all Python arguments before releasing the GIL locally around the pure
C++ constructor, and reacquires it before returning or translating exceptions.
The rebuilt module reports `_stararm_constructor_gil_release == 1`.
The core motor library, gains, model and CAN freshness thresholds are unchanged.
Native construction still cannot be cancelled mid-call and still enables motors.

0.6.7 提供 `tools/patch_sdk_constructor_gil.py`，仅对明确指定的 SDK 副本操作。
工具只接受已审查包装层源码或精确补丁结果：先在持锁状态完成全部 Python 参数转换，
再只在纯 C++ 构造期间释放 GIL，正常返回或抛异常时均先恢复执行锁。
重建模块提供 `_stararm_constructor_gil_release == 1` 标记。电机核心库、增益、模型及
CAN 时效阈值均保留。原生构造仍会使能电机，且不能在调用中途取消。

Keep the original SDK and session intact. Copy the SDK, apply the utility to that
copy, rebuild its extension with the existing CMake/Python environment, point a
local session copy to it, and generate a new binding with the normal `bind`
command. Never copy a new hash into an old binding. Source releases contain the
patch utility and tests, without rebuilt extensions or local settings.

保留原 SDK 和原配置。复制 SDK 后运行工具，用现有 CMake/Python 环境重建副本的
扩展模块，让本地配置副本引用它，再用正常 `bind` 命令重新生成绑定；不得向旧绑定
手填新哈希。源码发布只包含补丁工具和测试，不包含重建扩展或本机设置。

### Native resource lookup and physical SDK layout / 原生资源查找与 SDK 实体布局

The pinned native core's resource lookup depends on a real directory named
`startouch_sdk` along its SDK configuration candidates. Keep the SDK at
`.../<version>/startouch_sdk`, with the configuration at
`.../<version>/startouch_sdk/src/config/robot_kinematics.yaml`. Resolving the SDK
root must retain the `startouch_sdk` directory name; an alias symlink to a version
directory does not supply that physical layout.

固定原生核心的资源查找依赖 SDK 配置候选路径中的实体 `startouch_sdk` 目录。
SDK 应放在 `.../<version>/startouch_sdk`，配置文件为
`.../<version>/startouch_sdk/src/config/robot_kinematics.yaml`。解析后的 SDK 根目录
仍须名为 `startouch_sdk`；用符号链接给版本目录增加别名，不能提供该实体布局。

A directory containing the YAML alone does not establish the required native
resource layout. Copy all required SDK resources into the final physical
`startouch_sdk` directory before rebuilding. RPATH/RUNPATH can embed source/build
locations; after relocation, rebuild there and regenerate the binding normally.
Before any native constructor, resolve and check the selected SDK root and read
`src/config/robot_kinematics.yaml` offline, as shown in the
[hardware guide](hardware.md). An extension import and the GIL-patch marker alone
do not test native resource discovery.

目录中只有 YAML 不足以证明原生资源布局正确。先把所需 SDK 资源复制到最终实体
`startouch_sdk` 目录，再重建；RPATH/RUNPATH 可能嵌入源码或构建路径，若搬迁
须在新位置重新构建并正常生成新绑定。任何原生构造前，按[真机接入说明](hardware.zh.md)
离线解析核对所选 SDK 根目录，并读取 `src/config/robot_kinematics.yaml`。仅导入
扩展和读取 GIL 补丁标记，不会验证原生资源发现。

Missing-configuration failures at this lookup occur before native CAN
initialization and enabling in the constructor. Read-only startup preflight may
already have sent CAN queries. Resource-layout checks do not establish physical
holding, following, damping or the motor state left by another process.

此资源查找阶段的缺配置错误发生在构造器原生 CAN 初始化及使能之前；启动时的只读
姿态预检可能已发送 CAN 查询。资源布局检查不验证实际保持、跟随、阻尼，也不能
确定其他进程留下的电机状态。

### First-seed state-cache readiness / 首次固定目标前的状态缓存就绪

The Python wrapper returns native joint positions unchanged. Native review
confirms that `get_joint` returns six zeros until the control-loop cache becomes
valid, and construction returns without a cache-readiness barrier. An all-zero
SDK vector before the first target therefore does not establish a physical zero
pose. Compare all six SDK positions with fresh independent CAN feedback.

Python 包装层原样返回原生关节位置。原生审查确认 `get_joint` 在控制循环缓存有效前
返回六个零，而构造返回前没有缓存就绪屏障。因此首次目标前取得六个零，不能证明
机械臂实际处于零姿态；应逐轴与独立接收的真实新 CAN 反馈核对。

Version 0.6.10 permits a guarded readiness wait only before the first fixed raw
joint seed of current-pose startup. Its single absolute deadline is no more than
100 ms away, is shortened by all seven CAN age limits and the original fixed
reference's expiry, and is enforced inside the CAN callback and immediately before
the seed. It never refreshes the reference or restarts a per-attempt timeout.
Python sends no joint target until all six SDK positions agree with genuinely new
enabled CAN feedback within unchanged tolerances. Drift, motion, limits,
gripper/CAN failures and cancellation fail immediately. Checks after the seed and
after Leader requery remain immediate, with no readiness retries.

0.6.10 仅允许当前姿态启动的首次固定 raw 关节目标前进行受保护的就绪等待。各次
检查共用一个绝对截止时间，最多 100 ms，并受七轴 CAN 时效上限及原固定参考有效期
进一步限制；CAN 回调内部及首次目标发送前均执行该期限，不更新参考，也不为每次
尝试重置超时。六轴 SDK 与新到达、已使能的真实 CAN 按原容差全部一致前，Python
不发送关节目标。漂移、运动、限位、夹爪/CAN 故障和取消立即失败；首次目标后和
Leader 重采后的检查仍立即执行，不进行就绪重试。

This is a state-publication readiness correction, not deferred hardware activation.
The native constructor has already enabled motors, starts MIT control and closes
the gripper; that output can continue during the wait. Stable physical support is
still required. Neither this finding nor software validation establishes physical
holding, following or damping, and force authorization remains blocked.

这项修正处理状态发布就绪，不是延后硬件使能。原生构造器已经使能电机、启动 MIT
控制并关闭夹爪，等待期间这些输出仍可能继续，必须保留稳定支撑。该发现或软件测试
通过都不证明真机保持、跟随或阻尼已经验证，力输出授权仍被禁止。

- SDK commit: [`040412542c0b6b6fd2bb6dd652c0de6c666631be`](https://github.com/lumos-open/startouch_sdk/tree/040412542c0b6b6fd2bb6dd652c0de6c666631be), wrapper version 0.1.9.
- The examined `src/libstartouch.so` exactly matches that commit's `src/libstartouch.so.22`:
  SHA-256 `0e6fe64a1983883acc2b3bf9b5092c708c0c3d30dc5219cc5fbc7d8474b9932e`.
  CMake copies the OS-selected library to this filename; see
  [`CMakeLists.txt`, lines 61–80](https://github.com/lumos-open/startouch_sdk/blob/040412542c0b6b6fd2bb6dd652c0de6c666631be/CMakeLists.txt#L61-L80).
- The public header declares a private `Impl`; its initialization/control source
  is not distributed in this checkout. Binary findings below are symbol/address
  observations, not invented source-line references.

所审查 `.so` 与固定提交的 `.so.22` 完全一致；CMake 按系统版本复制库，因此工作树中
生成的 `.so` 与仓库默认副本不同，并不意味着未知代码修改。`Impl` 实现未在该源码
包公开，以下二进制证据用符号及地址标识，不伪称存在对应的公开源码行。

## Public API evidence / 公开接口证据

| Reference | What the source establishes / 源码能确认的内容 |
| --- | --- |
| [`startouchclass.py:216–224`](https://github.com/lumos-open/startouch_sdk/blob/040412542c0b6b6fd2bb6dd652c0de6c666631be/interface_py/startouchclass.py#L216-L224) | `SingleArm` directly constructs `ArmController`; arguments are CAN, gripper, FD and `dry_run`. It accepts no initial joint target or deferred-enable option. / 直接构造控制器，没有初始姿态或延迟使能参数。 |
| [`pybind_0825testfast.cpp:68–75`](https://github.com/lumos-open/startouch_sdk/blob/040412542c0b6b6fd2bb6dd652c0de6c666631be/src/pybind_0825testfast.cpp#L68-L75) | The binding fixes MIT mode and forwards `dry_run`. Unknown extra kwargs are not an initial-pose mechanism. / 固定 MIT 模式；不能靠额外 kwargs 预装姿态。 |
| [`interfaces_0825fasttest.hpp:41–53`](https://github.com/lumos-open/startouch_sdk/blob/040412542c0b6b6fd2bb6dd652c0de6c666631be/src/startouch/algorithms/interfaces_0825fasttest.hpp#L41-L53) and [`150–154`](https://github.com/lumos-open/startouch_sdk/blob/040412542c0b6b6fd2bb6dd652c0de6c666631be/src/startouch/algorithms/interfaces_0825fasttest.hpp#L150-L154) | Constructor and motion API, backed by private `Impl`. No public later enable/activate API is declared. / 构造及运动方法由私有实现支持，没有公开的后续使能接口。 |
| [`startouchclass.py:311–324`](https://github.com/lumos-open/startouch_sdk/blob/040412542c0b6b6fd2bb6dd652c0de6c666631be/interface_py/startouchclass.py#L311-L324) | `go_home()` explicitly sends six zeros; `gravity_compensation()` is a separate operation. Neither is called by the Python constructor. / 回 home 和重力模式均是另行调用的方法。 |
| [`startouchclass.py:346–361`](https://github.com/lumos-open/startouch_sdk/blob/040412542c0b6b6fd2bb6dd652c0de6c666631be/interface_py/startouchclass.py#L346-L361) and [`README_API.md:384–410`](https://github.com/lumos-open/startouch_sdk/blob/040412542c0b6b6fd2bb6dd652c0de6c666631be/README_API.md#L384-L410) | Raw targets are published after construction. MIT smoothing remains active, and the internal 400 Hz loop retains the latest target. The wrapper's `True` is not a motor hold acknowledgement. / raw 目标在构造后发布，内部循环保留目标并平滑；包装器返回成功不等于电机已保持。 |
| [`robot_kinematics.yaml:220–231`](https://github.com/lumos-open/startouch_sdk/blob/040412542c0b6b6fd2bb6dd652c0de6c666631be/src/config/robot_kinematics.yaml#L220-L231) | Nonzero MIT P/D gains and enabled gravity feedforward are configured. These are SDK settings, not commissioning validation. / 配置了非零 P/D 与重力前馈，不代表实机已验收。 |
| [`FastTouchV2.SLDASM.urdf:215–219`](https://github.com/lumos-open/startouch_sdk/blob/040412542c0b6b6fd2bb6dd652c0de6c666631be/src/config/FastTouchV2.SLDASM.urdf#L215-L219) | SDK J3 upper command limit is 0 rad. A measured boundary value and a command silently clamped by the SDK must not be described as identical. / SDK J3 命令上限为 0，实测边界值不能与被截断的命令混为一谈。 |

## Native-library evidence / 原生库证据

These addresses apply only to the exact `.so.22` hash above. They were inspected
statically with symbol tables and disassembly, without loading a controller.

以下地址只适用于上述哈希的库，由符号表和反汇编静态取得，未加载控制器。

| Symbol/address | Observed behavior / 观察到的行为 |
| --- | --- |
| `ArmController::Impl::Impl`, `0xbdf98` | Calls `initial_startouch()` before completing construction. / 构造完成前执行硬件初始化。 |
| `initial_startouch`, `0xa0d10`, `0xa0d21`, `0xa0d45`, `0xa0d6f` | Calls timeout setup, error clear, motor-mode switching and timeout setup. These are CAN configuration/control writes, not passive reads. / 设置超时、清错及模式切换，属于硬件写入。 |
| `initial_startouch`, `0xa0da2`, `0xa0dc9` | Calls `enable_all()` and callback configuration. `create_enable_command` at `0x2af21` encodes `0xfc`. / 调用使能及回报配置，编码器确实生成使能命令。 |
| `ArmController::ArmController`, `0xaeba3`, `0xaebc0` | With a real gripper, calls `closeGripper()`, then starts control/monitor threads. `closeGripper`, `0x94cb5`, publishes distance 0. / 有夹爪时发布关闭目标，并启动控制及监测线程。 |
| `Impl` initialization, `0xbe1fe`; `control_loop`, `0x9742b`, `0x97e53` | Command mode starts at 0. Its default branch uses the current sampled joint pose, rather than calling the explicit zero-home operation. This is not a fixed captured startup anchor. / 初始 mode 0 使用当期采样姿态，不是显式回零，也不是固定锁住启动参考。 |
| `control_loop`, `0x98fbf`, `0x99a82`, `0x9917f` | Gravity configuration selects KDL gravity feedforward; the loop publishes MIT commands. No Python `set_joint_raw` call does not mean no native control output. / 内部循环可计算重力前馈并写 MIT；上层尚未发 raw 不代表硬件无控制输出。 |
| `publish_joint_raw_command`, `0x96196`; `set_joint_raw`, `0x9665e` | A raw target selects mode 1 and is published to the running loop. SDK joint-limit clamping still applies. / raw 切换 mode 1，由已运行循环执行，保留 SDK 限位截断。 |
| `identify_gravity_compensation`, `0x947a8`; `publish_kdl_gravity_comp_command`, `0x93080` | Explicit gravity compensation selects mode 5 after construction. / 显式重力补偿在构造后切换 mode 5。 |
| `Impl` constructor, `0xbdff4–0xbe019`, `0xbe97a–0xbe9c6` | Checks signed J2 against −1.57 rad after initialization and can terminate with `exit(1)`. Preconstruction exclusion is necessary; catching Python exceptions afterward cannot replace it. / 初始化后 J2 小于 −1.57 rad 可直接退出进程，必须在构造前排除。 |

## Native velocity-range mismatch and correction / 原生速度量程差异与修正

The evidence below describes the uncorrected core; that state is still rejected
by the range gate when the verified hardware ranges differ. The supplied
correction targets installations whose confirmed RID22 (`VMAX`) is 10 rad/s for
J1–J3 and 30 rad/s for J4–J6; those values are not a universal hardware default.
The passive profile must use the actual verified ranges. Arm velocity conversion
multiplies decoded motor rad/s by `sdk_scale` and `urdf_scale`; the pinned YAML
`motor_signs` are `[1, -1, -1, -1, -1, 1]`. Validate coordinate transforms against
the installed assembly; do not add a reduction or degrees-to-radians conversion
without evidence that the selected coordinate path requires it.

下述证据描述未应用源码修正的核心；实物量程与其不一致时仍由门禁拒绝。提供的修正
针对已确认 J1–J3 RID22（`VMAX`）为 10 rad/s、J4–J6 为 30 rad/s 的装配，这些值
不是通用硬件默认值。被动监听配置必须使用实物已核实的量程。臂部速度转换为解码
电机 rad/s 乘 `sdk_scale` 和 `urdf_scale`；固定 YAML 的 `motor_signs` 为
`[1, -1, -1, -1, -1, 1]`。坐标变换须按实物核对，不能无依据增加减速比或角度转
弧度系数。

The core hardcodes J1–J3 as `DM4340` (type 3), whose compiled table is
`[pMax, vMax, tMax] = [12.5, 8, 28]`; J4–J6 use `DM4310` (type 1), with
`[12.5, 30, 10]`. The header also defines a separate `DM4340_48V` entry with
VMAX 10, but this initialization selects type 3. This is evidence of a range
mismatch, not identification of a different installed motor variant. The
compiled table was checked in the exact library above, rather than inferred
only from the header.

核心把 J1–J3 固定为 `DM4340`（类型 3），编译后的
`[pMax, vMax, tMax] = [12.5, 8, 28]`；J4–J6 使用 `DM4310`（类型 1），量程为
`[12.5, 30, 10]`。头文件另有 VMAX 为 10 的 `DM4340_48V` 条目，但本次初始化
选择类型 3。这证明量程不匹配，不等于确认实物属于另一电机型号。已核对上述精确
二进制中的表值，不只依赖头文件推断。

| Exact native evidence | Consequence / 影响 |
| --- | --- |
| `initial_startouch`, `0xa0bb5–0xa0bc0`; `MOTOR_LIMIT_PARAMS`, `0xc0608–0xc061f` | Creates the first three arm motors as type 3; its compiled PMAX/VMAX/TMAX are 12.5/8/28. / 前三轴固定类型为 3，编译量程为 12.5/8/28。 |
| `CanPacketDecoder::parse_motor_state_data`, `0x2b60c–0x2b653`, `0x2b6a2–0x2b6c2` | Loads velocity range from the motor-type table and decodes 12-bit velocity with it. The register lookup here is RID23 (`TMAX`), not RID22. / 用电机类型表中的速度量程解码；此处读取的是 RID23 力矩量程，不是 RID22。 |
| `control_loop`, `0x96f6c–0x9712a`; `get_joint_velocities`, `0x9463a–0x946c7` | Multiplies decoded velocity by the YAML sign and returns that cache, without correcting its range. SDK J1–J3 velocities are therefore 0.8 times the passive decoder's values for identical codes. / 解码速度只乘方向后进入缓存并对外返回；同码下 SDK 前三轴速度为被动解码值的 0.8 倍。 |
| `DMDeviceCollection::mit_control_one`, `0x30493–0x30497`; `create_mit_control_command`, `0x2b4a1–0x2b4a8` | Passes the motor's fixed type to the MIT encoder. / 将电机固定类型传给 MIT 编码器。 |
| `CanPacketEncoder::pack_mit_control_data`, `0x2b31b–0x2b32c`, `0x2b353–0x2b370`; `double_to_uint`, `0x2b285–0x2b29f` | Encodes desired velocity using the same ±8 range and integer truncation, without consulting RID22. A native +1 rad/s target becomes code 2303, interpreted by a ±10 motor as +1.247863 rad/s. / 目标速度同样按 ±8 编码并截断，不查询 RID22；原生 +1 rad/s 变成码 2303，±10 电机解释为 +1.247863 rad/s。 |
| `control_loop`, `0x97c75–0x97cb3`, `0x991ff–0x99213` | Raw smoothing generates an internal desired velocity and uses it in MIT output. A Python `v_end` of zero does not force every native MIT desired velocity to zero during following. / raw 平滑器产生内部目标速度并用于 MIT；Python 传零末速度，不代表跟随中每帧原生目标速度均为零。 |

The numeric MIT KD field is not itself rescaled by this mismatch. The desired
velocity in the motor's KD-dependent velocity-error term is wrong, so this is a
control-output issue as well as an observation issue. PMAX and the configured
signs agree, which explains why a position-only SDK/CAN readiness check can pass
without detecting this velocity-range problem. Changing a Python display or
passive velocity scale cannot repair the native encoder and internal state.

该不匹配不会直接缩放 MIT KD 数值，但电机 KD 速度误差项中的目标速度已被错误
缩放，因此它同时影响控制输出和观测。PMAX 及方向一致，解释了为何只核对位置的
SDK/CAN 就绪检查可能通过，却发现不了速度量程问题。修改 Python 显示或被动速度
比例，不能修复原生编码器及内部状态。

The public `ArmController` header, pybind method list, `SingleArm` wrapper and
YAML still expose no supported per-axis motor-type or VMAX override. This explains
why ordinary wrapper edits or a reinstall cannot repair the native codecs.
Original position following can nevertheless work because PMAX, position signs
and calibration agree; position motion alone never proved native velocity and
dynamics correct.

公开 `ArmController`、pybind、`SingleArm` 和 YAML 仍没有受支持的逐轴类型或 VMAX
覆盖接口，因此普通包装层修改或重装不能修复原生编解码。原遥操作仍可按位置跟随，
因为 PMAX、位置方向和校准一致；观察到位置运动并不能证明原生速度及动力学正确。

### 0.6.13 source correction / 0.6.13 源码修正

[`native/sdk_04041254_ranges.cpp`](../src/stararm_lumos_feedback/native/sdk_04041254_ranges.cpp)
defines the full 13-entry exported table, preserving every value except type 3
DM4340 VMAX 8 → 10 rad/s. Its `RTLD_GLOBAL` load precedes the native core's first
load, so all three reviewed GOT references resolve to the corrected table. The
core file, motor registers, gains, PMAX/TMAX, signs and calibration are unchanged.
This repairs the native MIT encoder and state decoder themselves; it does not
apply a Python feedback or command ratio.

The actual native `Motor::get_limit_param`, `pack_mit_control_data` and
`parse_motor_state_data` were executed successfully for all 13 types. Verification
covers signed range endpoints, zero, intermediate quantization cases and nonzero
physical velocities, plus decoder status/temperature ABI checks. The verifier
also checks table ownership so an earlier DSO cannot silently supply the symbol.
No arm controller, CAN connection or motor operation is created by these checks.
The result establishes codec behavior for this exact core and correction, not
physical following, holding or damping.

[源码](../src/stararm_lumos_feedback/native/sdk_04041254_ranges.cpp)定义完整 13 项同名
导出表，仅把类型 3 DM4340 的 VMAX 从 8 改为 10 rad/s。兼容库以 `RTLD_GLOBAL`
先于原生核心首次加载，使已审查的三处 GOT 引用均取得修正表。核心文件、电机寄存器、
增益、PMAX/TMAX、方向和校准不改；修正直接作用于原生 MIT encoder 与状态 decoder，
没有对 Python 反馈或命令乘比例。

实际原生 `Motor::get_limit_param`、`pack_mit_control_data` 和
`parse_motor_state_data` 已成功执行全 13 类型验证，覆盖正负量程端点、零、中间
量化值及非零物理速度，另核对 decoder 状态／温度 ABI。还检查表的归属，防止更早
加载的库静默提供同名符号。这些检查不创建机械臂控制器、CAN 连接或电机操作；结论
仅为该精确核心与修正的编解码行为，不证明真机跟随、保持或阻尼。

[`tools/build_sdk_motor_ranges.py`](../tools/build_sdk_motor_ranges.py) accepts an
explicit SDK copy and only the Linux x86-64 core whose SHA-256 is
`0e6fe64a1983883acc2b3bf9b5092c708c0c3d30dc5219cc5fbc7d8474b9932e`.
It compiles the source, verifies the real codecs in a fresh device-free process,
and publishes the library plus a source/core/library-hash manifest under
`interface_py`. Existing artifacts are preserved. Build against the SDK copy
selected by the profile's session, then regenerate the normal binding:

```bash
python tools/build_sdk_motor_ranges.py --sdk-root /absolute/path/to/sdk-copy/startouch_sdk
lumos-feedback bind --profile configs/commissioning.local.json \
  --binding configs/commissioning.binding.json --purpose monitor --replace
```

Runtime binds the C++ source, library and manifest. Before importing the SDK or
constructing an arm it verifies the actual codecs and compares the effective
native ranges with the unchanged hardware profile. It rejects stale bindings,
missing/partial/changed builds, prior native loads and inconsistent process reuse.
A missing valid correction retains the original 0.6.11 range rejection; installing
and rebinding it permits engineering `follow/monitor` to pass that gate. Existing
startup, CAN, fault, temperature, limit, jump, derivative and Leader-gap protections
remain active. Model coverage, measured unloaded/load response and native-board
output arbitration require separate review before formal `run` authorization.
Source delivery includes the C++ and builder, not compiled libraries, generated
manifests, local parameters, calibration, bindings or runtime data.

[构建工具](../tools/build_sdk_motor_ranges.py)仅接受明确指定的 SDK 副本和上述 SHA-256
的 Linux x86-64 核心。它编译源码，在全新无设备进程验证实际编解码，再于
`interface_py` 发布兼容库及记录源码／核心／库哈希的 manifest，不覆盖已有产物。
对 profile 的 session 所选 SDK 副本执行上面的命令，再正常重新生成绑定。

运行时绑定 C++ 源码、库及 manifest，在导入 SDK／构造机械臂前验证实际编解码，
并按真实启用的量程表与未更改的硬件配置比较。旧绑定、缺失／部分／变更构建、提前
加载的原生核心和不一致的进程复用均被拒绝。没有有效修正时保留 0.6.11 门禁；安装
并重新绑定后，工程 `follow/monitor` 可通过该项。原启动、CAN、故障、温度、限位、
跳变、导数及 Leader 缺帧保护保留。模型覆盖、实测空载／带载响应及原生按钮输出仲裁
仍须单独审查，不能以通过量程门禁替代正式 `run` 授权。源码交付包含 C++ 和构建工具，不含编译库、生成的
manifest、本机参数、校准、绑定或运行数据。

For an axis whose verified and native VMAX already agree, this J1–J3 mismatch
cannot explain an acceleration-guard failure on that axis. A derived-acceleration
error alone does not establish physical acceleration or the startup stage;
quantization and receive timing require separate analysis.

若某轴实物与原生 VMAX 已一致，就不能用前三轴的量程差异解释该轴加速度检查失败。
仅凭估计加速度报错，不能确认真实加速度或启动阶段；量化与接收时间问题须独立分析。

### Output before verified feedback / 验证反馈之前的原生输出

The earlier startup restriction also remains unresolved. Native mode 0 repeatedly
uses the current sampled pose, zero desired velocity/acceleration, nonzero MIT
gains and configured gravity feedforward; it does not capture a fixed startup
hold. `Motor::Motor` initializes position/velocity/torque to zero, feedback-valid
to false and its timestamp to zero (`0x2a354–0x2a370`). `update_state` sets valid
feedback and its timestamp (`0x2a441–0x2a47c`). The control-loop arm read checks
vector length, then uses those positions without a per-axis valid/timestamp
barrier (`0x96f53–0x9712a`), and marks its joint cache valid (`0x971d6`).
`Startouch::recv_all` drains available frames until there is no more data
(`0x28b60–0x28c03`); it is not a fresh, enabled all-six-axis handshake. A sampled
native pose can therefore be stale or uninitialized even though mode 0 normally
uses measured positions. This finding does not establish that such a sample
causes any particular reported failure.

此前启动限制同样未解决。原生 mode 0 每轮使用当期采样姿态、零目标速度/加速度、
非零 MIT 增益和所配重力前馈，不捕获固定启动保持。`Motor::Motor` 将位置、速度、
力矩、时间戳初始化为零，反馈有效标记为假（`0x2a354–0x2a370`）；`update_state`
收到状态才设置有效标记及时间戳（`0x2a441–0x2a47c`）。控制循环检查电机列表长度
后直接使用这些位置，没有逐轴有效性/时间戳屏障（`0x96f53–0x9712a`），随后把
关节缓存标记为有效（`0x971d6`）。`Startouch::recv_all` 只读取当前可用帧，直到
无数据（`0x28b60–0x28c03`），不构成六轴新鲜已使能反馈握手。因此即使 mode 0
通常使用实测位置，原生采样仍可能过期或尚未初始化，不能仅据此归因某次报错。

Enabling occurs inside `initial_startouch` before the native loop and before
Python can publish its first fixed target. External CAN guards can reject the
session afterward, but cannot gate that earlier native output. A guaranteed
validate-then-enable fixed hold requires a native deferred-enable/capture-and-hold
handshake or a separately reviewed control implementation. The GIL correction
and first-seed readiness wait do not provide this capability.

使能发生在 `initial_startouch` 内，早于原生循环启动及 Python 首次固定目标。
外部 CAN 检查可以事后拒绝会话，却不能阻止这段更早的原生输出。要保证先验证再
使能并固定保持，需要原生延迟使能/捕获保持握手，或另行审查的控制实现。执行锁
修正及首次目标就绪等待均未提供这项能力。

## Consequences for startup / 对启动流程的约束

No reviewed public or binary evidence requires a return to paired calibration
home. Paired home remains the default policy of this integration. Explicit
engineering `current_aligned` preserves the original absolute mapping;
`current_relative` adds a fixed six-axis session offset to the original mapped
targets. Both preserve stored calibration and require fresh stable feedback, strict
agreement with their session targets, limits, SDK J2 admission and closed gripper
checks. After construction they must compare new CAN and SDK state against the
accepted reference, reject drift, publish a fixed target and check stability
before normal following. They must not adopt a moved pose to conceal startup motion.

没有证据表明 SDK 强制要求回配对 home；home 是本接入的默认策略。显式工程
`current_aligned` 保留原绝对映射，`current_relative` 给原映射目标增加本次固定六轴
偏移；两者均保留已存校准，同时要求真实新帧稳定、严格核对本次目标、限位、SDK J2
和闭合夹爪。构造后须用新 CAN 与 SDK 状态复核已接受参考，拒绝漂移，再发固定目标
并检查稳定，不能把移动后的姿态改作参考来掩盖启动运动。

For `current_relative`, the captured offset is `actual Follower SDK position −
original mapped Leader target`, applied only to six arm joints. Follower seven-axis feedback must be stable for at least 0.5 s. The raw Leader
origin is queried, then queried again after SDK construction and checked using the
same fixed offset. Signs, gains, IDs, limits, speed and step guards remain in
use, and the gripper receives no offset. The offset is constant until cleanup and
stays only in memory. It does not rewrite calibration or create settings, logs or
history. Relative startup is an integration choice for engineering `follow/monitor`,
not a new SDK holding capability or force authorization.

`current_relative` 捕获的偏移为 `Follower 实际 SDK 位置 − Leader 原映射目标`，
仅作用于六个臂部关节。Follower 七轴反馈需至少稳定 0.5 秒，Leader 原始起点由查询
取得，SDK 构造后再次查询并按同一固定偏移核对；原方向、增益、ID、限位、速度及单步
限制继续使用，夹爪不加偏移。偏移保持固定，清理时清除，只留内存，不改校准或创建
设置、日志、历史记录。相对起步是工程 `follow/monitor` 的接入选择，不是 SDK 新增
保持能力，也不授权力输出。

The API cannot preload an atomic hold before enabling: construction enables and
starts its loop before the caller can publish a target. `dry_run=True` has no public
transition to live activation. Current-pose startup therefore still needs stable
physical support, and cannot establish that releasing support will hold J4.
Arbitrary poses also require workspace/contact review; joint limits and pair
agreement alone do not establish collision clearance. These findings neither
authorize force output nor validate TypeLJ mass coverage or native-button arbitration.

该接口不能在使能前预装原子保持，`dry_run=True` 也没有公开的后续真机激活路径。
当前姿态启动仍需稳定支撑，不能据此认定松手后 J4 会保住。任意姿态还需核对工作
空间及接触，限位与主从对齐本身不证明没有碰撞。此审查不授权力输出，也不验证
TypeLJ 质量覆盖或原生按钮与软件力输出的仲裁。

## TypeLJ inertial evidence / TypeLJ 惯量证据

Each installation must establish whether its whole-arm URDF includes the
installed TypeLJ assembly. When coverage is supported, record
`mass_coverage: urdf_includes_typelj` and `payload: null` to avoid adding the same
mass twice. Otherwise keep coverage unconfirmed or provide a separately reviewed
payload model. A configuration declaration is not new manufacturer evidence or
acceptance of dynamic thresholds and hardware performance. The experimental
contract may use `max_mw: null` only with confirmed coverage; this removes the
additional experiment ceiling and uses each configured per-axis rise rate.
Common and per-axis power caps remain active. Formal `run` requires its own
reviewed authorization.

每套装配须先确认整臂 URDF 是否覆盖实装 TypeLJ。覆盖有依据时记录
`mass_coverage: urdf_includes_typelj`、`payload: null`，避免重复添加质量；否则
保持覆盖未确认，或提供单独审查的附加载荷模型。配置声明不是厂家新增证据，也不
代表动态阈值及实机效果已验收。只有确认覆盖后，实验契约才允许 `max_mw: null`；
该值取消额外实验上限，并使用各轴配置的上升速率。共同及每轴功率上限继续生效，
正式 `run` 仍需自己的审核授权。

The pinned [Gitee URDF](https://gitee.com/lu-ming-robotics---shenzhen--g/startouch_sdk/blob/040412542c0b6b6fd2bb6dd652c0de6c666631be/src/config/FastTouchV2.SLDASM.urdf#L337)
is the same `FastTouchV2.SLDASM` model as GitHub. Its link6 has mass 0.5581 kg,
COM `[0.047873, 0.000015, 0.000968]` m and diagonal inertia
`[0.000428, 0.000473, 0.000616]` kg·m², with zero cross terms. It has no separate
gripper branch or statement identifying which gripper that mass includes. The
[YAML payload](https://gitee.com/lu-ming-robotics---shenzhen--g/startouch_sdk/blob/040412542c0b6b6fd2bb6dd652c0de6c666631be/src/config/robot_kinematics.yaml#L6)
has mass zero; its commented `0.126` is not a TypeLJ specification.

The separate [V3 CAD model](https://github.com/VincentHSS/Star-Arm-102-Web-ACT/blob/445f46f8756d9b826b17687dbdbb0c439537efcd/Browser_SDK/assets/urdf/fl-v3/model.urdf#L169)
contains distal masses 0.484919, 0.043757 and 0.043114 kg, totaling 0.571790 kg.
Its [provenance](https://github.com/VincentHSS/Star-Arm-102-Web-ACT/blob/445f46f8756d9b826b17687dbdbb0c439537efcd/THIRD_PARTY.md#L3)
does not identify TypeLJ. Its 85 mm total finger travel and 0.15584 m finger origin
match the SDK's TypeFZ descriptions; TypeLJ is described as 80 mm/0.17334 m.
Neither geometry resemblance nor a display model confirms TypeLJ inertia. Adding
the V3 total to link6 could double-count existing wrist/gripper parts. These static
sources alone do not establish TypeLJ coverage for an installation. Record the
assembly-specific evidence separately. A measured unloaded baseline and reviewed
load response are still required for formal `run`.

固定 [Gitee URDF](https://gitee.com/lu-ming-robotics---shenzhen--g/startouch_sdk/blob/040412542c0b6b6fd2bb6dd652c0de6c666631be/src/config/FastTouchV2.SLDASM.urdf#L337)
与 GitHub 是同一个 `FastTouchV2.SLDASM` 模型。link6 质量 0.5581 kg，重心
`[0.047873, 0.000015, 0.000968]` m，惯量对角
`[0.000428, 0.000473, 0.000616]` kg·m²、交叉项零；没有独立夹爪分支或型号覆盖
声明。[YAML payload](https://gitee.com/lu-ming-robotics---shenzhen--g/startouch_sdk/blob/040412542c0b6b6fd2bb6dd652c0de6c666631be/src/config/robot_kinematics.yaml#L6)
质量为零，注释 `0.126` 不是 TypeLJ 参数。

独立 V3 CAD 末端三连杆质量分别为 0.484919、0.043757、0.043114 kg，合计
0.571790 kg，但来源没有确认 TypeLJ。其双指总行程 85 mm、指根位置 0.15584 m
与 SDK 的 TypeFZ 说明吻合；TypeLJ 说明为 80 mm／0.17334 m。外形相似或展示模型
都不能确认 TypeLJ 惯量，将该合计叠加 link6 还可能重复计入腕部／夹爪部件。
这些静态资料本身没有确定某套装配的 TypeLJ 覆盖，应单独记录对应实物的覆盖依据。
正式 `run` 仍需要实测空载基线和经核对的带载响应。


## Experimental damping rise timing / 实验阻尼上升时序

With confirmed model coverage and `max_mw: null`, each axis uses
`profile.policy[].rise_mw_s` without the additional 100 mW/s experiment limit.
Contracts with a numeric 1–200 mW additional ceiling retain
`min(configured rise, 100 mW/s)`. Telemetry and the page expose the seven actual
effective rise rates. Select and validate rates for the installed hardware;
source delivery does not include an individual machine's tuning.

Synthetic tests exercise J4 alone with a sustained static residual while other
axes remain below threshold. Once filtering and load confirmation permit output,
the ideal power-ramp time is `requested power change / effective rise rate`;
filtering and confirmation add separate delay. No motion by another axis is
required. The steady-state residual-to-power mapping, full-output torque scale,
deadzones, native-board arbitration, ordinary caps and disabled experimental
software/automatic HOLD are unchanged. A small mapped request remains small.
These tests establish software timing behavior, not physical damping feel or
load-response acceptance.

已确认模型覆盖且 `max_mw: null` 时，每轴按 `profile.policy[].rise_mw_s` 上升，
不再叠加 100 mW/s 实验限制。数字 1–200 mW 额外上限契约继续使用
`min(配置上升速率, 100 mW/s)`。遥测和网页显示七轴实际生效上升速率。各轴速率应
按实装硬件选择并验证，源码交付不包含某台机器的调参文件。

合成测试只对 J4 施加持续静态残差，其他轴保持阈值以下。滤波及负载确认允许输出后，
理想功率斜坡时间为 `请求功率变化 / 实际上升速率`，滤波与确认延迟另计，无需移动
其他轴。稳态残差到功率的映射、满输出力矩标尺、死区、原生按钮仲裁、普通上限及
实验模式中关闭的软件／自动 HOLD 均保持不变，小映射请求仍然较小。这些测试只验证
软件时序，不代表真机阻尼手感或带载响应已验收。

## Initial baseline timing / 首次基线采集时机

A stationary supported startup pose must not supply the initial baseline before
native release or while approaching the Leader target. Increasing output rise
rate can expose an existing false residual request sooner; it does not establish
that the residual represents external load.

Initial collection requires a complete real native release, a complete
successful following write, limiter arrival at the original joint-limit-clipped
Leader target, measured proximity, and continuous target/measured stationarity.
The two-second stationary window and absolute residual method remain. No travel
limits, model mass/COM/inertia, torque signs, gain scales or motion speeds are
enlarged. Frozen baselines are never updated from later loads. This gate corrects
collection timing; unloaded separation across other poses requires explicit
unloaded observations and cannot be inferred from one pose.

按钮尚未真实释放或初次跟随尚未到达 Leader 目标时，不能使用支撑姿态冻结首次
基线。加快功率上升可能更快暴露已有的残差误请求，不能据此认定残差代表外部负载。

首次采集要求完整真实原生释放、完整成功跟随发送、限速目标到达原限位裁剪后的
Leader 目标、实测接近，以及目标／实测连续静止；保留两秒窗口与绝对残差方法。
不扩大行程，不修改质量／重心／惯量、力矩方向、映射增益或运动速度。冻结后不再
从后续负载更新基线。该门控校正采集时机，其他姿态的空载分离仍需明确空载观察，
不能由一个姿态推断。


### 0.6.28 baseline admission groups / 基线准入分组

Arm and gripper now have separate arrival admission and startup baseline statistics. Both estimators still require seven-axis measured stationarity. The first successful group closes collection: groups ready in that cycle freeze, and remaining groups keep zero damping until a new session. No baseline is learned while qualified damping runs. The six arm axes still use one whole-arm inverse-dynamics calculation. Physical ranges, tolerances, native button handling and SDK initialization are unchanged.

六轴和夹爪分别检查到位并保存启动基线统计，两个估计器仍要求七轴实测静止。首组成功时关闭采集：当周期已就绪组冻结，其余组本次会话保持零阻尼，不在已启用阻尼期间补学。六轴仍使用整臂逆动力学一次计算。物理范围、允差、原生按钮处理及 SDK 初始化不变。
