# Third-party sources / 第三方来源

This document records provenance. Component notices retain their own terms; this repository does not grant a new blanket license for third-party code, CAD, models or branding.

本文件记录来源，各组件沿用各自许可声明；本仓库不为第三方代码、CAD、模型或品牌新增统一授权。

| Component / 组件 | Source / 来源 | Included material / 本仓库内容 |
|---|---|---|
| StarArm102 → Lumos integration | [VincentHSS/stararm102-lumos-lerobot](https://github.com/VincentHSS/stararm102-lumos-lerobot) · base `454979c` | LeRobot adapters and paired teleoperation extended by this project / 遥操作与适配器，含本项目扩展 |
| Earlier StarArm feedback architecture | [VincentHSS/stararm-b601-rs-feedback](https://github.com/VincentHSS/stararm-b601-rs-feedback) | Architectural reference / 整臂剩余力矩阻尼架构参考 |
| Lumos StarTouch SDK | [lumos-open/startouch_sdk](https://github.com/lumos-open/startouch_sdk/tree/040412542c0b6b6fd2bb6dd652c0de6c666631be) | Compatibility tools, range correction source, constructor fixture; no SDK binaries / 兼容工具、量程修正源码及构造函数原文，不含 SDK 二进制 |
| LeRobot | [huggingface/lerobot](https://github.com/huggingface/lerobot/tree/3991580a5673f5f0219a4e8835a5f973eeccf9a7) | External dependency / 外部依赖 |
| LumosTouch V3 display geometry | [VincentHSS/Star-Arm-102-Web-ACT](https://github.com/VincentHSS/Star-Arm-102-Web-ACT/tree/445f46f8756d9b826b17687dbdbb0c439537efcd/Browser_SDK/assets/urdf/fl-v3) | URDF/STL display resources and source records / URDF、STL 显示资源与来源记录 |
| Three.js r169 | [mrdoob/three.js](https://github.com/mrdoob/three.js/tree/r169) | Local module and MIT notice / 本地模块及 MIT 声明 |
| Lucide | [lucide-icons/lucide](https://github.com/lucide-icons/lucide) | Local SVG icons and ISC notice / 本地 SVG 图标及 ISC 声明 |

The SDK fixture [sdk_0404125_pybind_constructor.cpp](../tests/fixtures/sdk_0404125_pybind_constructor.cpp) retains byte-exact upstream content for a fixed-hash patch regression. Paths in upstream comments are not distributed machine configuration.

SDK fixture 保留上游原文，供固定哈希补丁回归；其中上游注释路径并非本机配置。

Detailed notices / 详细声明：

- [CAD source and authorization](../src/stararm_lumos_feedback/web/models/lumos-touch-v3/NOTICE.txt)
- [CAD pinned source hashes](../src/stararm_lumos_feedback/web/models/lumos-touch-v3/SOURCE.json)
- [Three.js MIT notice](../src/stararm_lumos_feedback/web/vendor/NOTICE.txt)
- [Lucide ISC notice](../src/stararm_lumos_feedback/web/icons/LICENSE.txt)

Display geometry is separate from the configured dynamics model. Inclusion of the V3 gripper does not verify exact TypeLJ shape, mass coverage or hardware behavior. Dynamics and limits use the reviewed URDF for the installed assembly.

显示几何与动力学配置分开。V3 夹爪显示资源不证明 TypeLJ 外形、质量覆盖或硬件行为；动力学和限位以所用装配的已审核 URDF 为准。
