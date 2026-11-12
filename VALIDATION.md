# 0.6.24 验证范围 / Validation scope

本次还原以保存的 0.6.24 源码发布包为基准，恢复 UI 大改之前的页面和对应功能。
文档中的本机运行记载、调参结果及已不存在的根目录脚本入口已清理；运行源码保持
该发布版本。源码包不包含本机校准、绑定、硬件调参文件或运行数据。

## 软件检查

在准备好的 Python 3.12 / LeRobot 环境中，从仓库根目录运行：

```bash
PYTHONPATH=src python -B -m unittest discover -s tests -p 'test_*.py'
ruff check src tests tools
node --input-type=module --check < src/stararm_lumos_feedback/web/app.js
node --input-type=module --check < src/stararm_lumos_feedback/web/viewer.js
for script in scripts/*.sh; do bash -n "$script"; done
git diff --check
```

本次 576 项 unittest 全部通过，无跳过；Ruff、两份网页 JavaScript 语法检查、三份
shell 脚本语法检查和差异空白检查通过。历史 `test_follow_control` 的导入路径修正为
发布包自带的 `_legacy.py`，不依赖不存在的根目录脚本。运行源码、URDF 和网页资源
共 48 个文件与原 0.6.24 发布包逐字节一致。

本次为整版回退，同时撤销 0.6.27–0.6.31 的后续变更。0.6.24 仍可能因夹爪未到位
阻止整组初始基线，且不支持在同一会话中显式重新采集基线。

测试覆盖整臂动力学、单位与映射、通信保护、
实验阻尼基线门控、输出限幅、启动策略和网页接口。硬件接口使用测试替身或合成数据；
这些检查不会使能机器人，不能证明真机跟随、通信可靠性、空载透明度、带载稳定性
或阻尼手感已经验证。装配、配置或源码变化后须重新生成正常绑定，并按接入指南
完成离线检查、基础跟随、仅监测和实际阻尼验证。

## English

This restoration uses the archived 0.6.24 source release, restoring the dashboard
and corresponding functionality from before the UI redesign. Documentation omits
machine-specific runtime records and tuning, and names the packaged CLI entry
points. Runtime source retains the archived release. Local calibration, bindings,
hardware tuning and runtime data are excluded.

Run the commands above in the prepared Python 3.12 / LeRobot environment. All 576
unittest cases passed without skips. Ruff, syntax checks for both web JavaScript
files and three shell scripts, and diff whitespace checks passed. The historical
`test_follow_control` import now targets the packaged `_legacy.py`, rather than a
missing root script. All 48 runtime-source, URDF and web-resource files are byte-for-byte
identical to the archived 0.6.24 release.

This full-version rollback also removes later 0.6.27–0.6.31 changes. In 0.6.24, an
unsettled gripper can block the initial baseline for the whole group, and explicit
baseline recapture within the same session is unavailable.

Tests cover whole-arm
dynamics, units and mapping, communication guards, experimental-baseline admission,
output limits, startup strategies and dashboard APIs. Hardware boundaries use test
doubles or synthetic data and do not enable a robot. Passing software checks does
not establish physical following, communication reliability, unloaded transparency,
loaded stability or damping feel. Regenerate bindings normally after assembly,
configuration or source changes, and complete offline checks, basic following,
monitoring and physical damping validation using the hardware guide.
