# StarArm → Lumos 力反馈 · 0.6.28

0.6.26 新增七轴基线到位诊断，包括夹爪归一化开度。首次基线等待期间，页面显示阻碍轴的目标、发送值、实测值、误差与容差；停止、失鲜或基线已冻结时不作为实时阻碍显示。此次只加诊断，门控顺序、阈值和阻尼策略不变。

0.6.25 更新白底黑蓝界面、本地 Lucide 图标、按钮微交互、显隐与折叠过渡和中英文形变切换。表格只更新变化值，切换语言保留未应用的功率草稿。三维在已收到的姿态间以约 200 ms 显示缓冲插值，不外推；失鲜或停止时立即冻结到最后实测并标记状态，静止时停止连续绘制。真实 STL、动力学、校准、控制频率和保护参数未变。显示平滑不代表控制跟随或力反馈效果已验收。浏览器的“减少动态效果”设置会关闭装饰动效。

0.6.24 统一启动前后实测位置的边界判断，使用已绑定的各轴 `limit_margin`，与 CAN 检查一致，避免零位附近半个编码步长被后续严格检查再次拒绝。真实测量和基线锚点保留原值；首次位置命令及相对起步参考裁剪到原限位内，Leader 目标上限不变。超出原测量余量仍会拒绝。该修复不需要重新校准，源码变化需要正常重新生成绑定。

跟随速度、单步目标上限、调度频率、状态频率及 Leader One Euro 滤波参数由所选会话
配置提供。页面显示实际循环频率与超时周期，调度目标不代表保证达到的频率；CAN／
串口检查及控制仍逐周期执行。每轴表显示映射、限位裁剪、实际发送与实测位置，帮助
区分校准／限位截断、限速滞后与伺服偏差。不同轴分别限速，不保证同一时刻到位；
不改校准或厂家限位。

新增显式 `experiment` 实验阻尼模式：仅在本机绑定配置中明确授权、声明真实空载、
并启用原生按钮事件接收后可用。至少 2 秒连续真实空载静止样本期间输出为零；
冻结各轴绝对剩余力矩 P99＋4 个力矩量化步长作为最小死区，不减均值、不在线学载荷。
仍用整臂 q/v/估计 a 计算 J1–J6 模型力矩，夹爪独立；滤波加速度超出原模型域时，
保留该估计值并立即停止阻尼，重新预热后再判断。须核对实际装配的质量覆盖；若所选
整臂 URDF 已包含 TypeLJ，记录 `mass_coverage: urdf_includes_typelj`、`payload: null`，避免重复
叠加夹爪质量。显式实验契约的 `max_mw: null` 表示取消额外实验功率上限；每轴实际
上限为网页共同与该轴上限的较小值，默认均为 1200 mW。共同可调 400–1500 mW，
每轴可调 0–1500 mW，0 关闭该轴阻尼。已确认模型覆盖且 `max_mw: null` 的模式按各轴
`profile.policy[].rise_mw_s` 上升；速率须根据实际硬件经核对的响应填写，源码发布不包含
本机调试参数。
旧版 1–200 mW 的显式额外上限契约仍兼容，上升速率继续取配置值与 100 mW/s 的较小值。
网页显示实际生效的逐轴上升速率。所有自动及软件 HOLD 请求关闭。
mW 为 Leader 请求值，不是实测机械阻力或安全额定值。

首次空载基线须先收到完整真实按钮释放，并至少完成一次完整跟随发送。六轴与夹爪分别检查目标限速是否完成及实测是否到位；七轴实测（包括夹爪原始电机）仍须连续静止至少 2 秒。首组完成时冻结当周期所有已就绪组，并关闭初始采集窗口；未就绪组本次会话保持零阻尼，运行中不会自动补学。页面分别显示六轴和夹爪资格，夹爪超差不再连带阻断已就绪六轴。等待按钮、支撑姿态及所属组起步接近期间不采集该组。保持原到位阈值与关节限位，冻结后不因按钮切换或新外载重采。单姿态基线仍不能证明整个运动范围没有空载误阻尼。

J4 单独持续静态受载时，只要该轴滤波残差超过当时死区并满足负载确认，就会输出对应
阻尼，无需移动其他关节。纯斜坡时间等于请求功率变化量除以配置的上升速率，滤波与
确认时间另计。上升速率修正在契约允许时移除额外实验限制；残差、死区、满输出标尺
及最终强度映射不变，小残差仍可能只产生较小请求。软件验证不等于真机手感验收。

原生 HD 板需收到完整六轴真实释放事件，才允许实验输出；未知、锁定或部分组
暂停新的 Follower 目标并抑制软件输出，不重发按钮保持。每次运动／力命令前检查
已排队事件，但板与软件没有原子互斥保证；部分事件后可能残留的软件输出必须
标为未知，不能声称已释放。退出控制仍可能失去从臂支撑。
每套装配都须核对模型覆盖、动态阈值及实机效果。工程配置保持 `force_authorized=false`，
不能使用正式 `run`；装配声明不等于完成动态验证。
静止基线未覆盖整个姿态和运动范围；残差含摩擦／模型误差，实验阻尼不等于准确外力
或正式力反馈验收。网页仅允许共同及各轴功率上限，不允许修改内部策略。


[English](feedback.md) / **简体中文**

0.6.19 对通过严格启动比对后的工程基础 `follow`，处理 SDK 六轴异步缓存与
独立 CAN 的时间差。当前帧比对失败时，SDK 位置须在原 max_age_s 内匹配一帧
真实有效 CAN 位置，沿用原 sdk_tolerance，不插值、不扩大角度容差；当前 CAN
仍须新鲜且无错误。有限历史只在内存，不刷新遥测时效。其他模式、启动实测种子
及夹爪检查不变。SDK 没有六轴时间戳，因此这只能证明与近期实测位置一致，
不能证明缓存自身年龄；静止或回到相同位置也不能确定其更新时间。

0.6.18 仅对工程基础 `follow` 的 `leader_target` 起步，允许 SDK 使能后的限幅
微动。使能前仍需新鲜稳定回包；起点漂移不能超过原容差且最多 2°，实际速度／
位置限位、SDK/CAN 一致性、固定实测种子和目标限速继续检查。其他模式及起步
策略仍要求使能后满足原静止条件。

0.6.17 仅在工程基础 `follow` 中跳过六轴动力学差分加速度门禁；该模式不计算
动力学、不输出阻尼。阈值来自 SDK 轨迹设置的工程推导，不是已核实的电机实际
加速度额定值。`monitor`、`run`、其他配置和全部夹爪检查仍严格执行。故障、温度、
CAN 时效、实际限位、速度、位置跳变、SDK/CAN 一致性和目标限速继续保留；
不改变任何数值阈值或校准文件。

0.6.16 增加工程跟随／监测的 `leader_target` 直接起步，取消两端对齐、距校准
零位和夹爪预先闭合门禁。程序用新鲜从臂实测姿态初始化原有限速器，随后按
关节限位裁剪后的 Leader 绝对映射目标接近，不修改校准或设置相对偏移。
SDK 构造时可能自动闭合夹爪，使能时仍可能短暂运动。通信、故障、温度、
实测关节／电机范围、SDK/CAN 一致性、起点稳定及 SDK J2 构造条件继续检查。
未审核力反馈配置仍不可输出阻尼。

0.6.15 在限位错误中显示从臂或映射目标的实际角度。对齐容差只适用于姿态差，
不会扩展关节限位。工程相对起步记录两端各自的起点，保留全部关节限位，
无需搬回校准零位。

会话可选字段 `robot.startup_pair_tolerance_rad` 单独设置零位模式两端允许差
（单位 rad）。省略或 `null` 保留各轴原值；各自距零位
条件、关节限位和当前位置模式最多 2° 的保护不变。修改后正常重生成绑定，
不需要重新校准。

0.6.14 调整启动提示与姿态显示：未读到真机时使用有效配对校准的参考姿态；读取后
即使停止或预检失败，也保留最后实测姿态并明确标为历史数据。缓存只在内存，不恢复
遥测新鲜度、不授权启动，也不保存快照。启动请求明确传递当前下拉框显示的方式，
页面分开显示上次尝试和下次选择。零位模式同时检查两端距原校准零位及两端差，
不能仅凭两端接近判断通过；相对模式的夹爪开度要求仍为 ≤1%，因为 SDK 构造时会
自动闭合夹爪。新的中英文错误提示保留实际数值，所有保护阈值与校准不变。

0.6.13 增加可审查的源码修正，作用于固定 SDK 的真实原生速度编解码。首次加载核心前，
C++ 库通过 `RTLD_GLOBAL` 提供同名完整 13 类型量程表，仅把 DM4340 的 VMAX 从
8 改为 10 rad/s。实际原生 getter、MIT encoder 和状态 decoder 的全部 13 类型、
正负端点及非零量化用例已在无设备条件下验证。核心文件和电机寄存器不改。这是本地
兼容构建，不是厂家新版本，也不代表真机效果已验证。

原遥操作可以按位置目标跟随，因为位置量程及方向与硬件一致；旧速度量程仍会影响
原生速度反馈、目标速度和动力学项。此前“只能等厂家修正”的结论过窄：已审查的原生
量程表由动态引用读取，因此可用此源码兼容层同时修正编解码。

在明确指定的 SDK 副本中构建，再正常重新生成绑定：

```bash
python tools/build_sdk_motor_ranges.py --sdk-root /absolute/path/to/sdk-copy/startouch_sdk
lumos-feedback bind --profile configs/commissioning.local.json \
  --binding configs/commissioning.binding.json --purpose monitor --replace
```

构建工具仅接受已审查的 Linux x86-64 核心，保留已有产物；在独立、无设备进程验证后，
才于 `interface_py` 生成 `stararm_motor_ranges.so` 及 JSON manifest。运行时绑定
C++ 源码、二进制和 manifest，先验证实际编解码，再导入 SDK／构造机械臂；已加载
的原生核心、部分或变更产物和旧绑定均会被拒绝。未安装有效修正时，原量程门禁仍
禁止运动；安装并重新绑定后，`follow/monitor` 可通过该门禁，启动、通信及运动保护
继续保留。真机跟随、保持和阻尼尚未验证。每套装配仍须单独确认模型覆盖；当前 `run` 仍因
动态阈值、实测响应和正式原生 HD 力输出仲裁未验收而禁止。精确证据见 [SDK 审查](sdk-startup-review.md)。

Gitee 与 GitHub 当前同为 `040412542c0b6b6fd2bb6dd652c0de6c666631be`。
[Gitee URDF](https://gitee.com/lu-ming-robotics---shenzhen--g/startouch_sdk/blob/040412542c0b6b6fd2bb6dd652c0de6c666631be/src/config/FastTouchV2.SLDASM.urdf#L337)
和 [YAML](https://gitee.com/lu-ming-robotics---shenzhen--g/startouch_sdk/blob/040412542c0b6b6fd2bb6dd652c0de6c666631be/src/config/robot_kinematics.yaml#L6)
没有额外 TypeLJ 惯量证据。须针对实际装配记录模型覆盖依据，
不把注释 `0.126` 或 V3 展示模型质量当作额外 TypeLJ payload。源码交付不包含编译兼容库、生成的 manifest、
本机参数、校准、绑定或运行数据。

现有 `lumos-lerobot teleop` 入口支持显式反馈参数；不传时保留原基础路径。反馈扩展
复用 `StarArm102.get_action`、`Lumos.send_action` 及配对映射，由安全 `Session`
外围循环执行，不运行原 LeRobot `teleop_loop`。`--feedback-profile PATH` 与
`--feedback-binding PATH` 必须同时提供；`--feedback-mode` 可选 `follow/monitor/run`，
默认 `monitor`。`--config` 必传，规范化后的完整会话配置须与 profile 绑定的 session
完全一致。以下只离线打印计划，不连接硬件：

```bash
lumos-lerobot teleop --config configs/local.json \
  --feedback-profile configs/commissioning.local.json \
  --feedback-binding configs/commissioning.binding.json \
  --feedback-mode monitor --print-command
```

去掉 `--print-command` 才会请求执行，全部保护继续生效。本机独立部署可用
`stararm-lumos-feedback teleop --print-command`；源码目录可用
`bash scripts/run_feedback.sh teleop --print-command`。脚本使用
`configs/session.local.json`，可由 `TELEOP_CONFIG` 指定现有配置；不改原项目安装。

构造 GIL、SDK 目录、首次状态有限等待及累计速度量化区间保护继续保留。构造器可能
在 Python 发送固定目标前就使能电机并关闭夹爪，仍须稳定支撑。启动、姿态预检及
停止说明见[真机接入](hardware.zh.md)和 [SDK 审查](sdk-startup-review.md)。

运行代码已实现：六轴耦合动力学、逐轴剩余力矩阻尼、独立夹爪阻尼、自动 HOLD、
旧板 HD 统一仲裁、真实 CAN/串口帧检查、本机绑定及中英文网页。这是软件实现，尚未验证
真实跟随和力反馈。软件实现及离线测试通过不代表已完成某台实物的上机适配；交付物
不包含本机校准、绑定或真机调试参数。

硬件为构型改变但电机不变的 StarArm（Fashionstar ID 0–6），Lumos 底座正装、TypeLJ、
无额外工具。需核对新 Leader 的对应关系、方向、零位和限位。两个 B601 仓库均未修改，
没有移植 B601 模型、近零姿态保护或数值调参。

## 安装及分阶段接入

使用 Linux、Python 3.12，按[基础教程](stararm102-lumos.md)准备 LeRobot 环境。
`bash scripts/install_lerobot_ubuntu.sh` 安装本项目及 CAN/YAML 依赖，并固定 LeRobot
`3991580a5673f5f0219a4e8835a5f973eeccf9a7` 和 SDK
`040412542c0b6b6fd2bb6dd652c0de6c666631be`（0.1.9）。已有完整环境可执行
`python -m pip install -e .`。

先按基础教程准备 `configs/local.json`、现有配对校准和经核对的 SDK 坐标限位。
不要编造校准，也不要在没有核对的情况下把旧构型校准用于新构型。校准及基础教程的
预览会连接硬件；SDK 构造可能使能 Lumos 电机，基础启动器的 HD 按钮也能保持 Leader。

先建立**本机**硬件记录：

```bash
lumos-feedback template --output configs/feedback.local.json
lumos-feedback discover --can can0 --seconds 5
```

`template` 写未配置的硬件记录并拒绝覆盖。先填入已核对的路径、七轴 ID、实际协议量程、
坐标/力矩转换和来源；不能用 SDK 静态默认值冒充实物参数。其他待测保护/策略字段
不需要为了观测而随意填写，接下来由 `prepare` 生成有明确来源的工程初值。
`discover` 只接收 CAN 并把候选打印到终端，不发送 CAN、不构造 SDK、不使能。
空闲电机可能不主动回包；本版新增的 `inspect` 会发送已核对的纯读状态刷新请求。
不要同时运行两个独立运动控制器。

生成可直接观测的工程配置，再离线检查、绑定并启动只读网页：

```bash
lumos-feedback prepare --profile configs/feedback.local.json --output configs/commissioning.local.json
lumos-feedback check --profile configs/commissioning.local.json --purpose inspect
lumos-feedback bind --profile configs/commissioning.local.json --binding configs/commissioning.binding.json --purpose inspect
lumos-feedback inspect --profile configs/commissioning.local.json --binding configs/commissioning.binding.json
```

`prepare` 拒绝覆盖，保留已有校准及质量覆盖状态，生成完整数值配置 `version: 3`。
初值来自已核对的协议量化、启动器目标速度/单步限幅、SDK 加减速设置及明确标注的工程
观测选择；不是已测空载基线，不是厂家保证的保护阈值。`mass_coverage` 未确认时允许
`inspect/follow/monitor`，模型及残差标为暂定；`run` 在导入设备 SDK 前即被拒绝。
这些初值没有 B601 参数来源，也不能通过网页改成获准的力反馈配置。

`check`、`bind` 不连接硬件。绑定包含已有校准、限位、启动配置、硬件/策略配置、URDF、
SDK YAML/数据/构建资源和力反馈/适配器源码。任一绑定资源变更后须核对并重新生成；
`bind --replace` 只替换指定绑定，始终不改校准。实际运动启动前还检查导入的 SDK 包装器、
扩展和已加载 `libstartouch.so`。先在当前 Python 环境构建 SDK，再生成绑定。

连接硬件前，可用工程配置和 `--purpose inspect` 生成的绑定离线查看模型：

```bash
lumos-feedback preview --profile configs/commissioning.local.json --binding configs/commissioning.binding.json --port 8766
```

打开 `http://127.0.0.1:8766/`。`preview` 只核对本机配置与绑定，不构造或调用设备 SDK，
不打开 CAN 或串口，不发送动作或力输出。页面明确标识模型参考姿态，不伪造真机遥测，
实测力矩、电流和帧龄保持未接入状态。基础启动器的 `lumos-lerobot preview` 会连接
机械臂，此处的 `lumos-feedback preview` 不连接。Ctrl+C 关闭本地预览网页。

三维画面加载 [Web-ACT 固定提交 445f46f8](https://github.com/VincentHSS/Star-Arm-102-Web-ACT/tree/445f46f8756d9b826b17687dbdbb0c439537efcd/Browser_SDK/assets/urdf/fl-v3)
的九份 LumosTouch V3 CAD STL 网格，仅用于显示。六轴关节原点、轴向和坐标旋转与现用
SDK V2 模型一致，直接使用已有 `q_urdf` 角度，不再叠加方向转换或零位偏移。物理限位、
校准及 SDK 动力学质量保持原配置；V3 显示模型较窄的 J2 限位不能截断实测物理角度。

V3 夹爪 CAD 的总行程为 85 mm，尚未确认与实物 TypeLJ 精确一致。显示双指按 TypeLJ
实际开口距离各移动一半、方向相反（总开口最大 80 mm）；这不确认机壳、手指长度、
工具偏移或质量/惯量。随包 `SOURCE.json` 记录文件哈希，
[NOTICE.txt](../src/stararm_lumos_feedback/web/models/lumos-touch-v3/NOTICE.txt)
保留上游来源及授权说明。上游注明模型来自用户提供的 `URDF.zip`，本工程不另行授予
CAD 资产许可。拖动旋转、滚轮缩放，重置按钮
恢复视角。离线参考姿态与真实遥测分别标识；在硬件模式接入真实遥测后，同一视图
根据关节及夹爪反馈同步姿态。

只读检查通过后，按默认校准起始姿态对齐，或使用[真机启动步骤](hardware.zh.md)中
显式选择的工程当前姿态流程。下面的 CLI 示例仍采用默认 home 对齐，分别进行限时
基础跟随和仅监测：

```bash
lumos-feedback follow --profile configs/commissioning.local.json --binding configs/commissioning.binding.json --seconds 15
lumos-feedback monitor --profile configs/commissioning.local.json --binding configs/commissioning.binding.json --seconds 30
```

| 模式 | Lumos 目标跟随 | 观测器 | 软件 Leader 力输出写入 |
| --- | --- | --- | --- |
| `preview` | 无；不打开 SDK/CAN/串口 | 本地模型参考姿态，无真机遥测 | 无；实际输出状态未知 |
| `inspect` | 无；不构造 SDK、不使能 | 纯读 CAN、暂定模型和姿态 | 无；不打开串口，实际输出状态未知 |
| `follow` | 启用 | 关闭；状态及保护检查保留 | 无 |
| `monitor` | 启用 | 模型、残差、预计阻尼/HOLD、内存静止统计 | 无 |
| `run` | 启用；只接受已审核配置，检测到原生 HD 新板时拒绝 | 启用 | 普通阻尼 / 旧板 HD 保持 / 自动 HOLD |

表中力输出列仅表示本程序写入。新款原生 HD 板自行向 Leader 臂部 ID 0–5 发送
保持／释放命令，在 follow/monitor 乃至软件未运行时都会改变阻力。软件只监听这些
命令来冻结／恢复 Follower 新目标，不重发命令；网页按钮板开关不能禁止实物的
原生控制。旧板在三个运动模式下均可用 HD 冻结新目标，只有经过审核的 `run` 才额外
由软件向 ID 0–5 请求保持。检测到原生新板时，`run` 额外拒绝启动，因为其自行发出
命令与软件阻尼的交互尚未验证；工程配置本身也独立禁止 `run`。
当前原生 HD 支持仅适用于本反馈网页／CLI 的 `follow/monitor`，不适用于原
`lerobot-teleoperate` 入口；后者没有持续原生命令监听，识别到新板后拒绝读取动作。

运动模式构造 SDK，可能使能电机，均会实际跟随。工程 `monitor` 的自动 HOLD 仅显示候选，
不会冻结目标；已审核 v2 的 `monitor/run` 保留自动 HOLD 冻结。SDK 最后接受的目标
仍有效，冻结不会取消已进入原生 SDK 的命令。首次夹爪目标锚定当前真实开度，之后
限制为每秒 0.5 个归一化开度、单步不超过 0.02，延迟最多累计 0.04 s 的运动额度。
`--seconds N` 限时，Ctrl+C 清理。

默认启动仍要求两臂对齐原配对校准 home。只有工程 `follow/monitor` 提供页面显式
选择的 `current_aligned`（“按当前姿态对齐”）：先让 Follower 在稳定支撑上静止，
再按页面用原校准逆算出的当前目标对齐 Leader，无需把 J4 托回 home。该过程不创建
新校准，也不改变随后跟随的映射。SDK 构造前要求连续 0.5 秒真实新帧稳定、逐轴
主从误差不超过 `min(2°，目标单步限制，原轴启动容差)`、原限位、SDK J2 ≥ −1.57 rad，以及
Follower 归一化夹爪开度 ≤ 0.01。构造后必须取得新 CAN 状态，复核相对已接受姿态
的漂移和 SDK 一致性，再发送固定 raw 姿态及零目标速度；继续检查 2 秒稳定后，
才进入正常限速跟随。姿态变化会拒绝本次启动，不会被悄悄采用为新参考。

0.6.6 另提供显式 `current_relative`（“当前位置相对起步”），仅供工程
`follow/monitor`。Follower 保留稳定支撑，Leader 放在校准范围内稳定、可操作的位置，
无需对齐逆算的绝对目标。页面显示当前 Follower 姿态及 Leader 校准 home 参考，
不要求到达该 home。预检核对 Follower 稳定反馈并查询 Leader 原始起点，六轴本次固定偏移为 `Follower 实际
SDK 位置 − Leader 原映射目标`；随后目标为 `原映射目标 + 本次固定偏移`，保留原方向、
增益、ID、限位、速度及单步限制。夹爪不加偏移，沿用原映射和限制。Follower 七轴需
至少连续 0.5 秒真实新帧稳定；Leader 起点由查询取得，SDK 构造后再次查询并按同一
固定偏移核对。上述全部当前姿态保护继续生效，包括按偏移后的目标核对原有严格
容差、SDK 准入、夹爪闭合、构造后新 CAN/SDK 与漂移检查、固定 raw 目标及零目标速度，
以及正常跟随前的 2 秒稳定检查。漂移不会更新固定偏移；清理时从内存清除，不写校准、
设置、日志或参数历史。

即使 Follower 姿态通过自身检查，逆算目标仍可能超出某个 Leader 关节的校准范围。
`current_aligned` 会拒绝该姿态；相对模式提供显式替代方式并保留原校准，默认 home
和绝对当前姿态对齐仍可使用。

这些是工程启动流程，不是原子硬件保持。SDK 在构造内部就使能电机并启动自身控制
循环，早于固定目标的发送；启动全过程仍需稳定支撑。软件测试通过或目标被接受，
都不能证明松开支撑后 J4 能保住。当前绝对/相对启动及真实跟随尚未经真机验证，TypeLJ
质量覆盖及原生 HD 的力输出仲裁仍禁止力反馈。精确静态证据及其边界见
[固定版本 SDK 启动审查](sdk-startup-review.md)。

选择有板时，初始只读探测最多尝试三次。旧板通过 ID 7 ping 回包进入原有按钮监测
路径，继续要求持续真实新帧。新板则在等待提示出现后，要求支撑好 Leader 再按一次
HD。网页持续监听至真实完整事件或点击停止；收到覆盖 ID 0–5、同一保持／释放模式
的完整 RX command 24 六轴组后，
才允许构造 SDK／使能。点击停止可取消等待；等待期间不构造 SDK、不使能、不写软件
力输出，姿态预检在确认后重新采集。直接适配器／CLI 保留原 60 秒操作等待上限；
网页延长的只是使能前的操作等待，不放宽帧时效或 0.25 秒半组命令期限。
新板只在按键边沿发命令，不要求后续
周期命令帧。半组命令、具有精确原生请求头的半帧，或相符但校验和错误的帧，立即
使状态变为待确认并冻结新目标；0.25 秒内未补齐有效完整组即报错退出。“无板”跳过
按钮板检测，保留关节缺帧检查，但无法禁止仍接在实物上的新板改变 Leader 阻力。
这些改动不改变动力学观测器。

`monitor` 的基线收集只在七轴真实新帧且持续静止时，在内存中记录残差绝对值的
最大值、p95/p99、姿态覆盖及死区建议；中断会打断连续窗口，不减均值，不自动改策略。
**静止不等于已确认空载**，该统计不能确认 TypeLJ 质量，也不能授权力输出。
`inspect` 允许失能状态，没有基础跟随及空载条件，因而不生成合格基线。

质量覆盖与实测空载/带载参数确认后，建立单独审核的 v2 力反馈配置并重新绑定。
检测到原生 HD 新板时仍禁止 `run`，直到其与软件阻尼交互得到验证。支持的旧板／
无板配置可按以下方式启动：

```bash
lumos-feedback check --profile configs/feedback.reviewed.local.json
lumos-feedback bind --profile configs/feedback.reviewed.local.json --binding configs/feedback.reviewed.binding.json
lumos-feedback run --profile configs/feedback.reviewed.local.json --binding configs/feedback.reviewed.binding.json
```

## 显式硬件配置

已审核力反馈配置不提供通用于各台 Lumos 的力矩/死区/故障阈值。未知模板字段为
`null`，不能直接用于 `run`；先用上述 `prepare` 生成工程观测初值，再根据实物、
厂家资料和空载监测建立审核配置。SDK 默认电机配置不能代替实物身份、
反馈 ID 和量程的核对，TypeLJ 质量覆盖也仍须确认。测试夹具中的数值是合成输入，
不能用于真机调参。

已审核力反馈配置使用 `version: 2`；工程观测配置使用 `version: 3`。旧 v1 对 TypeLJ 错用了线性开度变换，不可直接运行；
使用新版模板重新核对字段并生成绑定，保留已有校准，不手改哈希。

配置内路径相对该配置文件；`session_file` 内的路径沿用原启动器相对启动配置及 `~`
展开规则。`session_file` 指向自己的 `local.json`，`urdf` 指向完整整臂模型，
`sdk_config` 指向启动配置中 SDK 的 `src/config/robot_kinematics.yaml`。
SDK YAML 须选择 TypeLJ、底座正装；基坐标系重力 `gravity` 为
`[0, 0, -9.80665]` m/s²。把资料来源和已核对的坐标约定写进 `evidence`。

确认模型包含 TypeLJ 后，`mass_coverage` 填 `urdf_includes_typelj`；若原模型缺少
夹爪质量，则填 `explicit_typelj_payload` 并提供 `payload`：`link`、`mass_kg`、
`com_m`、`inertia_kg_m2`（顺序 `ixx, ixy, ixz, iyy, iyz, izz`）。重心用安装连杆
坐标，惯量为新增物体重心处、以安装连杆轴向表达的惯量。只补缺失的原装 TypeLJ
质量，避免重复计算。SDK URDF 有七个惯性连杆，含底座共 4.28817 kg，没有单独夹爪
分支，引用的七个 STL 均缺失。[link6](https://gitee.com/lu-ming-robotics---shenzhen--g/startouch_sdk/blob/040412542c0b6b6fd2bb6dd652c0de6c666631be/src/config/FastTouchV2.SLDASM.urdf#L337)
质量为 0.5581 kg、重心为 `[0.047873, 0.000015, 0.000968]` m，但没有 TypeLJ
覆盖声明。同版 [Gitee YAML](https://gitee.com/lu-ming-robotics---shenzhen--g/startouch_sdk/blob/040412542c0b6b6fd2bb6dd652c0de6c666631be/src/config/robot_kinematics.yaml#L6)
payload 质量为零，注释 `0.126` 不是 TypeLJ 参数。V3 展示 CAD 末端三连杆合计
0.571790 kg 也未确认属于 TypeLJ，其 85 mm 行程及 0.15584 m 指根位置与 SDK 的
TypeLJ 80 mm／0.17334 m 说明不同。不能把该合计叠加到 link6，也不能用显示资源
替换已审核的惯性模型。详细来源见 [模型证据](sdk-startup-review.md#typelj-inertial-evidence--typelj-惯量证据)。

`axes` 按 J1–J6、夹爪顺序有七项：

| 字段 | 含义 |
| --- | --- |
| `feedback_id`、`motor_id` | 标准 11 位反馈仲裁 ID、帧内 4 位电机 ID，二者组合不能重复；不要假设命令 ID 就是反馈 ID。 |
| `p_max`、`v_max`、`t_max` | 实际 DM MIT 解码的 ±rad、±rad/s、±Nm 量程。须核对电机型号及固件；SDK 的 `dm_motor_constants.hpp` 可参考，但不代表已识别实物。 |
| `sdk_scale`、`sdk_offset` | 六轴 `q_sdk = p_raw * sdk_scale + sdk_offset`，把方向、零点、实际减速比各计入一次。TypeLJ 此处先转换到夹爪几何公式使用的电机 rad，再由非线性公式转为归一化开度，不能用线性比例代替开度。 |
| `urdf_scale`、`urdf_offset` | 六轴 `q_urdf = q_sdk * urdf_scale + urdf_offset`；夹爪必须为 1、0。SDK 电机符号与 Leader 映射符号是两套约定。 |
| `torque_scale` | `tau_model = tau_raw * torque_scale`。六轴须等于 `1/(sdk_scale*urdf_scale)`，保证关节功率一致；还须确认解码的是实际总轴力矩，不是前馈命令或已扣模型的残差。夹爪独立转换到电机输出轴 Nm，不按归一化开度比例缩放力矩。 |
| `max_age_s`、`max_temperature_c` | 真实接收帧最大时效、转子/MOS 温度阈值；达到温度阈值即停止。 |
| `max_jump`、`max_velocity`、`max_acceleration` | 相邻接收样本最大位置变化、实测速度、估计加速度。六轴单位为 URDF rad、rad/s、rad/s²，夹爪为归一化开度及其导数。超范围停止，不静默清零。 |
| `sdk_tolerance` | CAN 解码位置与 SDK 位置允许差，单位 SDK rad 或夹爪归一化开度。六轴比较另计帧到读取期间的实测运动；此检查不能证明 SDK 到 URDF 的约定正确。 |
| `limit_margin` | 实测位置超出 SDK 限位的显式容差（SDK rad，夹爪归一化开度），用于实际量化/噪声，不放宽命令限位；根据测量填写，不沿用旧臂数值。 |

`gripper_transform.type` 为 `typelj_sdk_04041254`，对应已核对的 SDK 0.1.9 TypeLJ
几何：电机角 0 到 -1.257153 rad 对应开口 0 到 0.08 m；中间行程为非线性，速度使用
几何导数换算。此具名转换要求第七轴 `sdk_scale=1`、`sdk_offset=0`；其他坐标约定
须审查并使用不同转换，不能靠任意缩放绕过电机物理范围。夹爪力矩仍为电机输出轴 Nm。`motor_margin_rad`、
`max_motor_jump_rad`、`max_motor_velocity_rad_s`、`sdk_motor_tolerance_rad` 须显式填写，
用于转换前的电机位置/跳变/速度及 SDK 一致性检查，避免开度截断掩盖异常。
SDK `GripperState.position` 是电机线性比例，不能当作公开的归一化距离；运行时核对
同一快照中的电机角及 `distance_m`，并保留真实 CAN 时间戳作为确认计时来源。

工程配置还可在 `commissioning.gripper_closed_observation` 记录已确认的闭合端：

`gripper_closed_observation` 仅包含 `operator_confirmed_closed: true` 和
`raw_position_code` 两个字段。后者必须填写实际夹爪完全闭合时观测到的 16 位整数；
源码发布不提供实测闭合码或可直接使用的观测配置。

只有在确认夹爪实物完全闭合、且读到稳定真实新帧后才可记录。嵌套字段必须完整，
确认值须为 `true`，原始码须为 16 位整数，按第七轴 `p_max` 解码。闭合参考值须大于
零且最多为 16 个位置 LSB，这是软件调试约束，不是厂家精度说明。闭合端上界为该
观测参考加原有电机位置余量，张开端下界保持原样。
观测不平移遥测、全域偏移、SDK 命令、非线性转换或原配对校准；它属于绑定
配置，修改后须正常重新生成绑定。仅工程 inspect/follow/monitor 接受该字段，正式
力反馈配置拒绝使用，工程 `run` 仍被禁止。这不证明完整实物行程或跟随／阻尼效果。

量程应按实物电机参数读回核对；已发现该版本 SDK 前三轴的静态速度量程可能与实物
参数不同，因此不能直接拿 SDK 速度 getter 或头文件默认值替代真实 CAN 解码量程。

被动接收器支持已检查的**经典 CAN 8 字节 DM MIT 状态**：状态/电机 ID、p16/v12/t12、
两温度字节。使能状态为 1；运动模式启动后禁用或故障会停止；只读 `inspect` 单独允许状态 0
并明确显示失能，故障仍停止。其他协议须改解码适配器，不能
改量程伪装包格式。只有真实接收帧有效；本机发送回显、远程帧、扩展帧、FD 帧及非状态
帧不计入。

`policy` 同样有七项。`deadzone_nm`、`velocity_margin`、`acceleration_margin` 定义
`死区 = 静态死区 + 速度余量系数*abs(v) + 加速度余量系数*abs(a)`。
`filter_s` 滤波有符号残差；`acceleration_filter_s` 滤波根据不同接收时间戳估计的加速度。
`confirm_s`、`release_s` 确认负载及释放。确认后普通输出为
`有效上限 * clamp((abs(滤波残差)-死区)/full_scale_nm, 0, 1)`，并按
`rise_mw_s`/`fall_mw_s` 限制升降速率。降低上限在下一次输出更新立即生效；缓存样本
不能增加负载或堵转确认时间。

HOLD 使用 `hold_error`、`hold_load_nm`、`hold_speed`、`hold_confirm_s` 和
`hold_progress`（仍判为“不再追赶”的最大角差减小速度）。任一轴持续角差、负载、
低速度和追赶不足同时满足后，对**全部七个 Leader 关节**请求 1500 mW 保持。
角差使用原 Follower 经过限速、单步限幅、限位后实际返回的目标。全部期望位置到实测
位置的误差及负载低于 `hold_release_error`、`hold_release_load_nm`，持续
`hold_release_s` 才释放。负载使用滤波残差绝对值；夹爪用自身电机力矩。
普通上限（包括 0）不会关闭 HOLD；旧板的软件手动 HD 只保持臂部 ID 0–5。新板自行
控制这些 Leader 关节，不受这些软件上限约束，当前不支持在 `run` 中使用。

`guards`：本经典 CAN SDK 接入的 `can_bitrate` 为 1000000 bit/s；`can_skew_s` 限制
跨轴接收时差；`leader_query_s` 限制一次新串口查询时间；`leader_gap_s` 限制连续
Leader 缺帧；`cycle_timeout_s` 限制循环心跳；`startup_feedback_s` 限制 SDK 初始化后
等待完整状态的时间。根据实际更新频率填写；心跳阈值须大于一个循环周期加查询时间。
启动器只检查已有 SocketCAN UP 状态和波特率，不配置接口。

## 输出、检查与网页

模型根据**整臂** URDF、实测姿态、速度、估计加速度计算各轴理论空载力矩；每轴
残差为自己的实测力矩减模型力矩，没有平均分配，也没有孤立单连杆计算。
普通阻尼取残差绝对值，不精确复现带方向外力。残差仍包含摩擦、传动和模型误差，
不能准确换算末端力或重量。夹爪力矩不进入六轴模型。

共同/每轴默认上限均为 1200 mW；共同上限 400–1500 mW，每轴 0–1500 mW，有效上限
取二者较小值。mW 是 Leader 请求功率，不是实测机械阻力。在获准的 `run` 配置中，
一个软件写入者统一仲裁普通阻尼、旧板 HD 和自动 HOLD。新板命令是独立硬件写入，
不在此仲裁内，检测到新板会禁止 `run`。普通输出为零时发送释放模式，避免固件对“阻尼功率零”的
特殊解释。按钮 ID7 永远不作为力输出目标。

Leader 使用新的、校验和正确的串口回包，不拿缓存 `angle_monitor` 当新状态。
部分缺帧暂停新目标和确认计时，连续缺帧停止。Fashionstar 没有每次查询序号，因此
无法证明迟到回包属于哪一次请求。CAN 使用真实内核时间戳，检查重复/陈旧/排队帧、
故障、禁用、温度、限位、位置跳变、速度、加速度和跨轴时差。夹爪也必须满足时效，
本实现没有套用 B601 的夹爪过期回包重试。上述首次目标前的 SDK 缓存就绪窗口仍要求
新到达、已使能的 CAN，不延长任何 CAN 时效上限。

独立看门狗在 CAN 故障或循环超时后释放已管理的 Leader 输出；异常和部分写入失败
同样尝试释放，并先于 SDK 清理。释放失败会在清理时重试并报错；通信已断时无法
保证机械上已经释放。原生调用阻塞或占有 Python GIL 时，软件退出/看门狗可能延迟。
资源锁排除本启动器的其他实例，但无关 SDK 控制器不会遵守该锁。SDK 原有可选保护
配置保持原样。

打开 `http://127.0.0.1:8765/`（`--port` 可改端口）。网页切换中英文，显示姿态、
关节力矩、温度、帧龄、Leader 请求功率及电流。没有已验证力矩常数的 Follower 电流
显示 `—`，不编造。`preview`、`inspect` 网页不允许修改上限，服务端同样拒绝；允许模式
下的阻尼参数编辑仅限功率上限，服务端拒绝修改阻尼策略及保护参数。`serve` 页面另提供
上述会话、按钮板及启动方式选择。请求检查本机 Host、来源和会话令牌。遥测只在内存中；
当前上限保存到 `~/.config/stararm_lumos_feedback/current.caps.json`（`--settings` 可改），
`serve` 另保存上述本机按钮板选择，当前姿态启动参考只留在内存。不创建会话 CSV、进程
日志、快照或参数历史。`--no-web` 关闭网页服务。此启动器不录 LeRobot
数据集，原有采集/回放命令独立保留。

## 离线验证与来源

`lumos-feedback-model` 仍为只依赖标准库的离线模型工具：

```bash
PYTHONPATH=src python3 -B -m stararm_lumos_feedback.model \
  --urdf /actual/path/startouch_sdk/src/config/FastTouchV2.SLDASM.urdf \
  --gravity 0 0 -9.80665
PYTHONPATH=src python3 -B -m unittest discover -s tests -p 'test_*.py' -v
PYTHONPATH=src python3 -B -m unittest discover -s tests -p 'test_follow_control.py' -v
```

模型测试核对势能梯度、耦合质量矩阵、能量守恒、多重力方向、显式负载和无效模型。
运行测试使用合成 CAN/串口、故障注入及 HTTP；适配器边界测试运行原有映射、滤波、
HD 和目标限幅，仅替换 LeRobot 基类及硬件构造器，不属于完整框架集成测试。
框架集成测试使用真实 LeRobot 与模拟硬件；软件验证范围及当前结果见
[VALIDATION.md](../VALIDATION.md)，不能据此认定真实跟随或力输出已验收。

基线：遥操作 `454979c7c8b9d5ced15f1964a2d588acf531ae3f`（0.2.0），SDK 如上，
原力反馈 `c4ce25e`（0.6.2），第二个 B601 子目录 `604d60b`（0.7.2）。保留整臂
动力学架构；没有修改两个原 B601 仓库或 SDK。源码测试不证明空载透明度、带载稳定性、
通信可靠性或阻尼手感。完成离线验证 → 基础跟随 → 仅监测 → 真实阻尼/HOLD 检查后，
才能称为该实物已调试完成。

## 可选 TypeLJ 跟随增益

本机 session 的 robot 配置可同时设置 `gripper_follow_kp` 和
`gripper_follow_kd`。两项默认均为 `null`，保留原 `setGripperPosition` 接口。
显式值必须有限，并符合已审核 SDK 的范围：kp 0.1–20、kd 0.1–1。绑定的 SDK
YAML 必须明确选择 TypeLJ，且包含有效的 `default_kp`、`default_kd`。

启用后，通过 `setGripperDistance` 发送归一化物理开度 × 0.08 米，跟随目标和
基线比较仍使用归一化开度。初始化继续使用原 YAML 默认增益；接近两端时，在各自
10% 开度区间内平滑回到默认增益，完全开、闭命令使用默认增益。此功能不修改运动
限值或原始电机范围保护，也不证明空载与负载已可靠分离。按实际装配审核参数，
修改 session 后正常重新生成绑定，并经过真机验证后再采用。网页不能修改这些增益。
