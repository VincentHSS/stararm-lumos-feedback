# StarArm → Lumos force feedback · 0.6.26

Version 0.6.26 adds seven-axis baseline admission diagnostics, including normalized gripper opening. The dashboard shows each blocking axis's target, sent and measured values with errors and tolerances while waiting for the first baseline. Stopped, stale or frozen-baseline sessions do not show this as a live blocker. This is diagnostic only; thresholds, admission ordering and damping policy are unchanged.

Version 0.6.25 adds a white, black and blue interface, local Lucide icons, button feedback, presence and disclosure transitions, and morphing language changes. Tables update changed values without replacing rows; language changes preserve unapplied cap edits. The viewer interpolates received poses with approximately 200 ms of display buffering and never extrapolates. Stale or stopped state immediately freezes to the latest measured pose and retains its state label; stationary scenes stop continuous rendering. Actual STL geometry, dynamics, calibration, control rate and protections are unchanged. Smoother display does not establish tracking or force-feedback performance. Reduced-motion preferences disable decorative animation.

Version 0.6.24 applies each bound axis `limit_margin` consistently to measured positions before and after startup, matching CAN checks so half an encoder step near zero is not rejected by a later strict measurement check. Raw measurements and baseline anchors are preserved. Initial position commands and relative-start references stay clipped to the original limits; Leader target limits are unchanged. Measurements beyond the existing margin still fail. Recalibration is not required; changed source requires normal rebinding.

Following speed, maximum target step, scheduling rate, status rate and Leader
One Euro filter settings come from the selected session configuration. The page
reports measured loop frequency and overruns; a scheduling target is not a
guaranteed achieved rate. CAN/serial checks and control remain per cycle.
Per-axis tracking shows mapped, joint-clipped, sent and measured positions,
separating calibration/limit clips from rate lag and servo error. Axes are
rate-limited independently and need not arrive simultaneously. Calibration and
manufacturer limits are unchanged.

The explicit `experiment` mode requires a bound local authorization, an actual
unloaded declaration and native-button event reception. Output remains zero
through at least two seconds of continuous real stationary samples. Per-axis
absolute residual P99 plus four torque LSBs is frozen as a minimum deadzone;
no fitted mean is removed and contact loads are never learned during output.
Whole-arm q/v/estimated-a dynamics still supplies J1-J6 model torques, with
independent gripper torque. A filtered acceleration outside the original model
domain is retained and inhibits damping until valid warmup. Confirm mass coverage
for the actual assembly. When the selected whole-arm URDF already includes
TypeLJ, record
`mass_coverage: urdf_includes_typelj` and `payload: null` to avoid adding its mass
twice. An explicit experimental contract with `max_mw: null` removes the additional
experimental ceiling. Each effective cap is the smaller of the webpage common
and axis caps, both defaulting to 1200 mW. The common cap is adjustable from
400–1500 mW and each axis from 0–1500 mW; 0 disables that axis. With confirmed
model coverage and `max_mw: null`, each axis uses its configured
`profile.policy[].rise_mw_s`. Set these rates from the reviewed response of the
actual hardware; source releases do not include machine-specific tuning. Existing explicit 1–200 mW additional-cap
contracts retain the smaller of their configured rise rate and 100 mW/s. The page
shows each axis's effective rise rate. All software and automatic HOLD requests
remain disabled. mW is a Leader request, not measured mechanical resistance or a
safety rating.

The initial unloaded baseline is collected only after a complete real button release, at least one complete successful following write, the end of target-rate catch-up and measured proximity to the target. Both the target and measured pose must remain stationary for at least two seconds. Button waiting, supported startup poses and initial approach cannot qualify. Arrival uses the original joint-limit-clipped target without enlarging travel. Once frozen, the baseline is never relearned on button changes or new external loads. This gate corrects startup sampling timing; a single-pose baseline still cannot establish unloaded separation over the entire motion range.

A sustained static load on J4 alone produces its corresponding damping once that
axis's filtered residual exceeds the current deadzone and satisfies load
confirmation; other joints do not need to move. The pure ramp duration is the
requested power change divided by the configured rise rate; filtering and
confirmation add separate time. The rise-rate correction removes the extra
experimental limiter where the contract permits it. The residual, deadzone, full-output torque scale and final intensity mapping are
unchanged, so a small residual may still produce a small request. Software
validation does not establish acceptance of physical damping feel.

A complete real six-axis native release event is required for experimental
output. Unknown, locked or incomplete groups pause new Follower targets and
suppress software commands without replaying native hold. Queued events are
checked before motion/force writes, but no atomic exclusion with the board is
guaranteed. Residual software output after an incomplete transition is reported
as unknown, never as confirmed released. Exiting control can remove Follower
support. Model coverage, dynamic thresholds and hardware performance must be
reviewed for each assembly. Engineering profiles keep `force_authorized=false`
and cannot use formal `run`; an assembly declaration does not establish dynamic
validation. The stationary baseline does not cover every pose or moving condition;
residuals include friction/model error and are not accurate
external-force measurements or formal force-feedback acceptance. Web controls
remain limited to common/per-axis power caps.


**English** / [简体中文](feedback.zh.md)

Version 0.6.19 accounts for the asynchronous six-axis SDK cache in engineering
basic `follow`, after strict startup agreement. If the original latest-frame
comparison fails, the SDK position must match one actually accepted CAN sample
within the original per-axis max_age_s and unchanged sdk_tolerance; current CAN
must remain fresh and error-free. No interpolation or enlarged tolerance is used.
The bounded history stays in memory and cannot refresh telemetry. Other modes,
strict startup seed checks and gripper checks are unchanged. The SDK exposes no
arm timestamps: this confirms recent observed-position agreement, not its cache's
actual publication age; unchanged or revisited positions cannot establish that age.

Version 0.6.18 permits bounded movement after SDK enabling only for engineering
basic `follow` with `leader_target` startup. Preflight still requires stable fresh
feedback. The measured origin may drift by at most its original tolerance, capped
at 2°; actual velocity/position limits, SDK/CAN agreement, the fixed measured seed
and target rate limits remain enforced. Other modes and startup strategies retain
the post-enable quiet-pose admission.

Version 0.6.17 skips the six-axis derived dynamics acceleration admission only
for engineering basic `follow`, which does not calculate dynamics or output
damping. The threshold is a commissioning estimate from SDK trajectory settings,
not a verified physical motor acceleration rating. `monitor`, `run`, other profiles
and all gripper checks remain strict. Faults, temperatures, CAN freshness, measured
limits, velocity, position jumps, SDK/CAN agreement and target rate limits remain
enforced; no numerical thresholds or calibration files are changed.

Version 0.6.16 adds explicit engineering `leader_target` startup for follow and
monitor. It skips alignment, calibrated-home and preclosed-gripper admission.
Fresh measured Follower positions seed the existing rate limiter; subsequent
actions approach the absolute mapped Leader target, clipped to joint limits.
Calibration and relative-start origins are not changed. SDK construction may
automatically close the gripper, and brief enabling movement remains possible.
Communication, faults, temperatures, measured joint/motor limits, SDK/CAN
agreement, stable startup measurements and SDK J2 construction conditions remain
enforced. Force output remains unavailable without a reviewed force profile.

Version 0.6.15 shows actual Follower or mapped-target angles in limit errors.
The alignment tolerance applies only to pose differences and does not extend
joint limits. Engineering current-relative startup captures each arm's origin;
it preserves all joint limits and does not require returning to calibrated home.

The optional session field `robot.startup_pair_tolerance_rad` independently sets
the pair threshold in home mode (in radians). Omission or
`null` preserves each axis's prior value. Individual home-distance conditions,
joint limits and the current-pose cap of at most 2° remain unchanged. This setting
requires a new normal binding, not recalibration.

Version 0.6.14 updates startup diagnostics and pose display. Before measurement,
the viewer uses the validated paired calibration reference. After reading or a
rejected pose preflight, it retains the last measured pose and labels it historical.
This memory-only cache cannot refresh telemetry or authorize startup and creates
no snapshots. Start explicitly submits the visible alignment choice; the page
separates the previous attempt from the next selection. Home mode checks both
distances from calibrated homes and the pair difference, so matching the arms
alone may still fail. Current-pose startup still requires gripper opening ≤1%
because SDK construction closes it. Bilingual errors retain actual values, and
all guard thresholds and calibration remain unchanged.

Version 0.6.13 adds a source-based correction for the pinned SDK's actual native
velocity codecs. Before the core's first load, a C++ library supplies its complete
13-type range table through `RTLD_GLOBAL`, changing only DM4340 VMAX from 8 to
10 rad/s. Device-free checks passed for the actual native getter, MIT encoder and
state decoder across all 13 types, signed endpoints and nonzero quantization
cases. The core file and motor registers are unchanged. This is a local
compatibility build, not a new vendor release or physical validation.

Original teleoperation can follow position targets because its position ranges
and signs agree with the hardware. The old velocity range still affects native
velocity feedback, desired velocity and dynamics. The earlier conclusion that
only a vendor correction could resolve it was too narrow: the reviewed native
table is dynamically referenced, so this source correction can repair both codecs.

Build into an explicitly selected SDK copy, then regenerate the normal binding:

```bash
python tools/build_sdk_motor_ranges.py --sdk-root /absolute/path/to/sdk-copy/startouch_sdk
lumos-feedback bind --profile configs/commissioning.local.json \
  --binding configs/commissioning.binding.json --purpose monitor --replace
```

The builder accepts only the reviewed Linux x86-64 core, preserves existing
artifacts, and creates `interface_py/stararm_motor_ranges.so` plus its JSON
manifest after verification in a separate device-free process. Runtime binds the
C++ source, binary and manifest, verifies the actual codecs before importing the
SDK or constructing an arm, and rejects prior native loads, partial/changed
artifacts and stale bindings. The original mismatch gate still blocks motion
without this valid correction. With it installed and rebound, `follow/monitor`
can pass that gate; startup, communication and motion guards remain active.
Physical following, holding and damping remain unvalidated. Model coverage must be confirmed for each assembly; formal `run` remains blocked pending dynamic-threshold,
measured-response and native-HD arbitration acceptance. See the [SDK review](sdk-startup-review.md) for exact evidence.

Gitee and GitHub currently share commit `040412542c0b6b6fd2bb6dd652c0de6c666631be`.
The [Gitee URDF](https://gitee.com/lu-ming-robotics---shenzhen--g/startouch_sdk/blob/040412542c0b6b6fd2bb6dd652c0de6c666631be/src/config/FastTouchV2.SLDASM.urdf#L337)
and [YAML](https://gitee.com/lu-ming-robotics---shenzhen--g/startouch_sdk/blob/040412542c0b6b6fd2bb6dd652c0de6c666631be/src/config/robot_kinematics.yaml#L6)
provide no extra TypeLJ inertial evidence. Record the coverage decision for the actual assembly with its
supporting evidence. Neither the
commented `0.126` nor V3 viewer mass is used as an additional TypeLJ payload. Source
delivery excludes the compiled correction, its generated manifest and local
parameters, calibration, bindings or runtime data.

The existing `lumos-lerobot teleop` entry supports explicit feedback options.
Without them, its original basic path remains unchanged. The extension reuses
`StarArm102.get_action`, `Lumos.send_action` and the paired mapping inside the
safety `Session` loop; it does not run LeRobot's original `teleop_loop`.
Supply `--feedback-profile PATH` and `--feedback-binding PATH` together.
`--feedback-mode` accepts `follow`, `monitor` or `run`, defaulting to `monitor`.
The required `--config` must normalize to exactly the profile's bound session.
Print an offline plan without connecting hardware:

```bash
lumos-lerobot teleop --config configs/local.json \
  --feedback-profile configs/commissioning.local.json \
  --feedback-binding configs/commissioning.binding.json \
  --feedback-mode monitor --print-command
```

Omit `--print-command` to request execution with all guards intact. The isolated
local deployment also accepts `stararm-lumos-feedback teleop --print-command`;
from source, use `bash scripts/run_feedback.sh teleop --print-command`. The script
uses `configs/session.local.json`, or the existing configuration specified by
`TELEOP_CONFIG`, without changing the original project's installation.

Constructor GIL, SDK-layout, bounded first-state readiness and cumulative
velocity-quantization protections remain active. The constructor can enable
motors and close the gripper before Python sends a fixed target; keep stable
support. See [hardware startup](hardware.md) and the [SDK review](sdk-startup-review.md).

The runtime is implemented: coupled six-axis dynamics, per-axis residual damping,
independent gripper damping, automatic HOLD, legacy-board HD arbitration, fresh CAN/serial
checks, local binding and a bilingual dashboard. This is a software implementation;
physical following and force feedback have not been validated. Passing software
tests does not complete commissioning of an individual arm. No local calibration,
binding or hardware tuning is included in the deliverable.

Hardware: changed StarArm configuration with the existing Fashionstar motors
(IDs 0–6), Lumos upright with TypeLJ and no additional tool. Verify the changed
Leader's correspondence, directions, home and limits. The two B601 repositories
are unchanged; no B601 model, near-zero guard or numeric tuning was carried over.

## Install and stage the setup

Use Linux, Python 3.12 and the existing LeRobot installation procedure in
[the base guide](stararm102-lumos.md). `bash scripts/install_lerobot_ubuntu.sh`
installs this package and its CAN/YAML dependencies, pins LeRobot
`3991580a5673f5f0219a4e8835a5f973eeccf9a7` and SDK
`040412542c0b6b6fd2bb6dd652c0de6c666631be` (0.1.9).
For an already prepared environment, run `python -m pip install -e .`.

First prepare `configs/local.json`, the existing paired calibration and reviewed
SDK-coordinate limits using the base guide. Do not invent a calibration or copy
the old Leader's calibration across a changed assembly without verification.
Calibration and the base guide's preview connect hardware; SDK construction may
enable Lumos motors, and the base launcher's HD button can hold the Leader.

Create a **local** hardware record:

```bash
lumos-feedback template --output configs/feedback.local.json
lumos-feedback discover --can can0 --seconds 5
```

`template` writes an unconfigured record and refuses to overwrite. Fill the reviewed
paths, seven-axis IDs, actual protocol ranges, coordinate/torque transforms and
sources first. Static SDK defaults do not establish installed motor parameters.
Do not invent the remaining guard/policy fields just to observe the arm: `prepare`
produces explicit engineering starting values. `discover` only receives CAN and
prints candidates; it neither transmits nor constructs/enables the SDK. Idle motors
may not stream. The new `inspect` command sends reviewed read-only state refreshes.
Do not run two independent motion controllers on the same hardware.

Generate an observation profile, check and bind offline, then start the read-only dashboard:

```bash
lumos-feedback prepare --profile configs/feedback.local.json --output configs/commissioning.local.json
lumos-feedback check --profile configs/commissioning.local.json --purpose inspect
lumos-feedback bind --profile configs/commissioning.local.json --binding configs/commissioning.binding.json --purpose inspect
lumos-feedback inspect --profile configs/commissioning.local.json --binding configs/commissioning.binding.json
```

`prepare` refuses to overwrite, preserves calibration and mass-coverage status,
and generates a complete numeric `version: 3` profile. Starting values derive from
reviewed protocol quantization, session target rate/step, SDK acceleration settings
and labeled engineering choices. They are neither a measured unloaded baseline nor
manufacturer-rated protection thresholds. Unconfirmed `mass_coverage` permits
`inspect/follow/monitor` with provisional model/residual labels; `run` is rejected
before importing the device SDK. No B601 values are used, and dashboard changes
cannot authorize this profile for force output.

`check` and `bind` do not connect hardware. Binding hashes existing calibration,
limits, session, hardware/policy profile, URDF, SDK YAML/data/build resources and
feedback/adapter source. Review and regenerate after any bound input changes;
`bind --replace` only replaces that binding and never rewrites calibration. Motion
startup also checks the imported wrapper/extension and loaded `libstartouch.so`.
Build the SDK in the active Python environment before binding.

To view the model before connecting hardware, use the offline preview with the
engineering profile and a binding generated with `--purpose inspect`:

```bash
lumos-feedback preview --profile configs/commissioning.local.json --binding configs/commissioning.binding.json --port 8766
```

Open `http://127.0.0.1:8766/`. `preview` checks local configuration and binding,
without constructing or calling the device SDK, opening CAN or serial, or sending
motion or force commands. It displays a labeled model reference pose; this is
not simulated hardware telemetry. Real torque, current and frame-age values
remain unconnected. Unlike the base launcher's `lumos-lerobot preview`, this
command does not connect the arm. Ctrl+C closes the local preview server.

The 3D view loads nine LumosTouch V3 CAD STL meshes from
[Web-ACT commit 445f46f8](https://github.com/VincentHSS/Star-Arm-102-Web-ACT/tree/445f46f8756d9b826b17687dbdbb0c439537efcd/Browser_SDK/assets/urdf/fl-v3).
These are display resources only. The six arm joint origins, axes and frame
rotations match the current SDK V2 model; the display uses the existing
`q_urdf` angles without another sign conversion or zero offset. Physical limits,
calibration and SDK-based dynamics masses remain unchanged. In particular, the
V3 display model's narrower J2 limit must not clip measured physical angles.

The V3 gripper CAD has 85 mm of total modeled travel and is not confirmed as an
exact match for the installed TypeLJ. Its displayed fingers use half the TypeLJ
opening distance in opposite directions (80 mm maximum total opening); this
does not verify the housing, finger length, tool offset or mass/inertia.
The bundled `SOURCE.json` records file hashes, and
[NOTICE.txt](../src/stararm_lumos_feedback/web/models/lumos-touch-v3/NOTICE.txt)
preserves upstream attribution and authorization information. The upstream
project identifies these resources as a user-supplied `URDF.zip`; this project
does not grant a new license for the CAD assets.
Drag to rotate and scroll to zoom; the reset button restores the view. Offline
reference poses are labeled separately from genuine telemetry. When a hardware
mode connects real telemetry, the same view follows joint and gripper feedback.

After read-only inspection passes, use the default calibrated-home alignment or
an explicit engineering current-pose procedure in [hardware startup](hardware.md).
The following CLI examples retain default home alignment; run bounded following
and monitoring stages separately:

```bash
lumos-feedback follow --profile configs/commissioning.local.json --binding configs/commissioning.binding.json --seconds 15
lumos-feedback monitor --profile configs/commissioning.local.json --binding configs/commissioning.binding.json --seconds 30
```

| Mode | Lumos target following | Observer | Software Leader force writes |
| --- | --- | --- | --- |
| `preview` | None; SDK/CAN/serial unopened | Local model reference pose; no hardware telemetry | None; actual output unknown |
| `inspect` | None; no SDK construction/enabling | Read-only CAN, provisional model and pose | None; serial unopened, actual output unknown |
| `follow` | Active | Disabled; state and guards remain active | None |
| `monitor` | Active | Model, residuals, predicted damping/HOLD, in-memory stationary statistics | None |
| `run` | Active; requires a reviewed profile and rejects native HD boards | Active | Ordinary damping / legacy-board HD hold / automatic HOLD |

These force-write columns describe this program. A newer native HD board sends
hold/release commands directly to Leader arm IDs 0–5, changing resistance in
follow/monitor and even while this program is stopped. The software only listens
to those commands to freeze/resume new Follower targets; it never resends them.
The page's board selector cannot disable native hardware control. With a legacy
board, HD freezes new targets in all three motion modes, and only reviewed `run`
also requests software holding on IDs 0–5. Detected native boards are additionally
rejected by `run` because native commands interacting with software damping have
not been validated; engineering profiles already block `run` independently.
Native HD support currently applies to this feedback dashboard/CLI's
`follow/monitor`, not the original `lerobot-teleoperate` entry point. That entry
point lacks the continuous native command listener and rejects action reads
after detecting a native board.

Motion modes construct the SDK, can enable motors and actively follow. Engineering
`monitor` displays automatic HOLD candidates without freezing targets; reviewed v2
`monitor/run` retain automatic HOLD freezing. The last accepted SDK target remains
active: freezing does not cancel a native command already in progress. The first
gripper target anchors to actual opening; subsequent commands are limited to 0.5
normalized opening/s, at most 0.02 per step, with at most 0.04 s of motion credit
after delays. `--seconds N` bounds a session; Ctrl+C initiates cleanup.

The default startup policy still requires paired-home alignment. Only engineering
`follow/monitor` offers the explicit `current_aligned` page choice, “Align at current
pose”. Rest the Follower on stable support, then align the Leader using the page's
current inverse targets from the unchanged calibration. This avoids lifting J4 to
home; it neither creates a new calibration nor changes the subsequent mapping.
Before SDK construction it requires 0.5 s of continuous fresh, stable feedback,
pair errors no greater than `min(2 degrees, target step limit, original axis startup tolerance)`,
existing limits, SDK J2 ≥ −1.57 rad, and normalized Follower gripper opening ≤ 0.01.
After construction, genuinely new CAN state must agree with the accepted pose
and SDK state before publishing a fixed raw pose target with zero target velocity;
a further two-second stability check precedes normal limited following. A changed
pose fails this start instead of silently becoming a new reference.

Version 0.6.6 additionally offers explicit `current_relative`, “Relative start at current pose (engineering)”, for engineering `follow/monitor`. Keep the Follower supported
and put the Leader in a stable, usable pose within its calibrated range; it does
not need to reach the absolute inverse target. The page shows current Follower
pose and Leader calibration-home references, without requiring those home values.
Preflight checks stable Follower feedback, queries the raw Leader origin and
fixes six arm offsets:
`actual Follower SDK position − original mapped Leader target`. Later targets are
`original mapped target + fixed session offset`, retaining the original signs,
gains, IDs, limits, speed and step guards. The gripper has no offset and keeps its
existing mapping and limits. Follower seven-axis feedback must remain continuously fresh and stable for at
least 0.5 s. The Leader origin comes from a query; after SDK construction, another
Leader query is checked against the same fixed offset. All current-pose guards above remain active, including
agreement with the offset-adjusted target within the existing strict tolerance,
SDK admission, closed gripper, post-construction CAN/SDK and drift checks, fixed
raw seeding with zero target velocity and the two-second stability check. Drift
never changes the fixed offset. Cleanup clears it from memory; calibration,
settings, logs and parameter history never receive it.

An inverse target can fall outside a Leader joint's calibrated range even when
the Follower pose passes its own checks. `current_aligned` rejects that pose;
relative startup provides an explicit alternative while preserving the original
calibration. Default home and absolute current alignment remain available.

These are engineering startup procedures, not atomic hardware holds. The SDK
enables motors and starts its own control loop inside construction, before the
fixed target can be published. Keep stable support through startup; neither a
successful software test nor target acceptance establishes that releasing support
will hold J4. Physical current-aligned/relative startup and following remain unvalidated.
Confirm model coverage for the actual assembly. Formal force output still awaits
measured-response and native-HD arbitration acceptance.
See the [pinned SDK startup review](sdk-startup-review.md) for the static evidence
and its limits.

When a board is selected, initial read-only probing makes at most three attempts.
A legacy ID 7 ping reply selects the existing button-monitor path, which still
requires continuous fresh frames. For a native HD board, the waiting prompt asks
the operator to support the Leader and press HD once. The page listens until a
real complete event or Stop. A complete
RX command 24 group must cover all six IDs 0–5 with the same hold/release mode
before the SDK can be constructed/enabled. Stop can cancel the wait; waiting
neither constructs/enables the SDK nor writes software force output; pose preflight
is freshly captured after detection. Direct adapter/CLI use retains its sixty-second
operator window. The webpage wait does not relax frame freshness or the 0.25-second
incomplete-group deadline. A native board emits edges, so no periodic command
frame is required afterward. An incomplete group, a partial frame with the exact
native request header, or a
matching frame with a bad checksum immediately marks the state pending and
freezes new targets; failure to complete a valid group within 0.25 seconds is
fatal. No-board mode skips board detection, retaining joint missing-frame checks.
It cannot prevent a physically connected native board from controlling Leader
resistance. None of these changes modifies the dynamics observer.

The `monitor` baseline collector accepts only genuinely new seven-axis frames in
continuous stationary windows. It reports absolute residual maximum, p95/p99,
pose coverage and deadzone suggestions in memory. Gaps interrupt the window; it
neither subtracts means nor changes policy. **Stationary does not establish
unloaded conditions.** Statistics cannot confirm TypeLJ mass or authorize force.
`inspect` allows disabled motors and does not establish following/unloaded
conditions, so it cannot produce an eligible baseline.

After confirming mass coverage and measured unloaded/loaded parameters, create a
separately reviewed v2 force profile and regenerate binding. `run` still rejects
a detected native HD board until its interaction with software damping is
validated. For a supported legacy/no-board setup:

```bash
lumos-feedback check --profile configs/feedback.reviewed.local.json
lumos-feedback bind --profile configs/feedback.reviewed.local.json --binding configs/feedback.reviewed.binding.json
lumos-feedback run --profile configs/feedback.reviewed.local.json --binding configs/feedback.reviewed.binding.json
```

## Explicit hardware profile

Reviewed force profiles do not ship universal Lumos torque/deadzone/fault thresholds.
Unknown template fields are `null` and cannot be used for `run`. Use `prepare` for
engineering observation, then establish reviewed settings from the actual hardware,
manufacturer data and unloaded monitoring. SDK motor defaults
do not replace checks of actual motor identities, feedback IDs and ranges;
TypeLJ mass coverage also needs confirmation. Test-fixture numbers are synthetic
and must not be used as tuning.

Reviewed force profiles use `version: 2`; engineering observation profiles use `version: 3`. Version 1 incorrectly used a linear TypeLJ
opening transform and is rejected. Review a new template and regenerate the
binding; preserve the existing calibration and never bypass hash checks.

Paths in the profile are relative to that profile; paths inside `session_file`
retain the original launcher's session-relative and `~` rules. Set `session_file`
to your `local.json`, `urdf` to the complete arm model, and `sdk_config` to the
session SDK's `src/config/robot_kinematics.yaml`. The SDK YAML must select TypeLJ
and upright mounting. Base-frame `gravity` is `[0, 0, -9.80665]` m/s². Record
sources and verified conventions in `evidence`.

`mass_coverage` must be `urdf_includes_typelj` after confirmation, or
`explicit_typelj_payload` with a `payload` object containing `link`, `mass_kg`,
`com_m`, `inertia_kg_m2` (`ixx, ixy, ixz, iyy, iyz, izz`). COM uses attachment-link
coordinates; inertia is about the added body's COM in attachment-link axes.
Add only the missing stock TypeLJ mass; do not double-count it. The SDK URDF has
seven inertial links totaling 4.28817 kg including the base, no separate gripper
branch, and seven missing STL references. Its [link6](https://gitee.com/lu-ming-robotics---shenzhen--g/startouch_sdk/blob/040412542c0b6b6fd2bb6dd652c0de6c666631be/src/config/FastTouchV2.SLDASM.urdf#L337)
has mass 0.5581 kg and COM `[0.047873, 0.000015, 0.000968]` m, but no TypeLJ
coverage statement. The matching [Gitee YAML](https://gitee.com/lu-ming-robotics---shenzhen--g/startouch_sdk/blob/040412542c0b6b6fd2bb6dd652c0de6c666631be/src/config/robot_kinematics.yaml#L6)
sets payload mass to zero; its commented `0.126` is not a TypeLJ specification.
The V3 viewer CAD's distal three-link total of 0.571790 kg is also unconfirmed
for TypeLJ: its 85 mm travel and 0.15584 m finger origin differ from TypeLJ's
80 mm/0.17334 m SDK description. Do not add that total to link6 or replace the
reviewed inertial model with display assets. See [model evidence](sdk-startup-review.md#typelj-inertial-evidence--typelj-惯量证据).

`axes` has seven entries in order J1–J6, gripper:

| Fields | Meaning |
| --- | --- |
| `feedback_id`, `motor_id` | Standard 11-bit feedback arbitration ID and the embedded 4-bit motor ID. The pair must be unique. Never assume command IDs equal feedback IDs. |
| `p_max`, `v_max`, `t_max` | Actual DM MIT decode ranges ±radians, ±rad/s, ±Nm. Verify motor variant and firmware. The SDK's `dm_motor_constants.hpp` is a reference, not identification of the installed motors. |
| `sdk_scale`, `sdk_offset` | Arm: `q_sdk = p_raw * sdk_scale + sdk_offset`. Include actual sign, zero and reduction exactly once. For TypeLJ this first converts raw motor radians to the geometry's motor angle; the nonlinear transform then produces normalized opening. A linear ratio cannot replace that transform. |
| `urdf_scale`, `urdf_offset` | `q_urdf = q_sdk * urdf_scale + urdf_offset` for the arm. Gripper requires 1 and 0. SDK signs and Leader signs are separate conventions. |
| `torque_scale` | `tau_model = tau_raw * torque_scale`. For arm joints it must equal `1/(sdk_scale*urdf_scale)` to preserve power. Confirm that the decoded torque represents total actual shaft torque, not a feedforward command or pre-subtracted residual. For the gripper, independently convert to motor output-shaft Nm; do not use normalized-opening gearing to rescale it. |
| `max_age_s`, `max_temperature_c` | Maximum real receive age and motor/MOS temperature; temperature at or above threshold stops. |
| `max_jump`, `max_velocity`, `max_acceleration` | Maximum position change between received samples, measured speed and estimated acceleration. Arm units: URDF rad, rad/s, rad/s². Gripper: normalized opening and derivatives. Out-of-range state stops; it is not silently cleared. |
| `sdk_tolerance` | Allowed discrepancy between decoded CAN and SDK position in SDK radians, or normalized gripper opening. Arm comparison adds measured motion during frame-to-getter delay. This check does not establish the SDK-to-URDF convention. |
| `limit_margin` | Explicit measured-position tolerance beyond SDK limits (SDK radians, gripper normalized opening), to account for actual quantization/noise. It does not relax command limits. Set from measurement rather than old-arm values. |

`gripper_transform.type` is `typelj_sdk_04041254`, using the reviewed SDK 0.1.9
TypeLJ geometry: motor angle 0 to -1.257153 rad maps to opening distance 0 to
0.08 m. Intermediate positions are nonlinear; velocity uses the geometry's
derivative. This named converter requires axis 7 `sdk_scale=1`, `sdk_offset=0`;
other coordinate conventions need a reviewed, different converter, not arbitrary
scaling that bypasses the motor's physical range. Gripper torque remains motor
output-shaft Nm. Explicitly set
`motor_margin_rad`, `max_motor_jump_rad`, `max_motor_velocity_rad_s` and
`sdk_motor_tolerance_rad`. These guard motor position/jumps/speed and SDK agreement
before conversion, so opening clamps cannot hide a motor fault.
SDK `GripperState.position` is a linear motor fraction, not public normalized
distance. Runtime checks motor angle and `distance_m` from the same state snapshot;
genuine CAN timestamps remain the source of confirmation timing.

Engineering profiles may additionally record a confirmed closed endpoint in
`commissioning.gripper_closed_observation`:

`gripper_closed_observation` contains exactly
`operator_confirmed_closed: true` and `raw_position_code`. Fill the latter with
the 16-bit integer observed on the actual fully closed gripper; source releases
do not supply a measured endpoint or a ready-to-use observation profile.

Record this only after confirming the physical gripper is fully closed and
observing stable, genuinely new raw motor frames. The exact nested keys are
required: confirmation must be `true`, and the raw code a 16-bit integer decoded
with axis 7 `p_max`. The resulting positive closed reference must be at most
16 position LSBs above zero. This is a software commissioning constraint, not a
manufacturer accuracy claim. The closed upper guard is that observed reference
plus the existing motor-position margin; the open lower guard remains unchanged.
The observation does not shift telemetry, global offsets, SDK commands, the
nonlinear converter or existing paired calibration. It is part of the bound
profile and requires ordinary rebinding after changes. Only engineering
inspect/follow/monitor accept it; reviewed force profiles reject the field and
engineering `run` remains blocked. The observation validates neither full physical
travel nor following/damping behavior.

Verify decode ranges against parameters read from the installed motors. This SDK's
static first-three-axis velocity range can differ from actual motor settings;
neither its velocity getter nor header defaults can replace actual CAN ranges.

The passive monitor supports the reviewed **classic CAN 8-byte DM MIT state**:
status/motor ID byte, p16/v12/t12, two temperature bytes. Enabled status is 1;
disabled/fault states stop motion modes after startup. Read-only `inspect` explicitly
allows and displays disabled status 0; faults still stop. A different protocol requires a decoder
adapter, not altered ranges to disguise its packet format. Only genuine receive
frames count; local transmit echoes, remote/extended/FD/non-state frames are ignored.

`policy` has seven corresponding entries. `deadzone_nm`, `velocity_margin` and
`acceleration_margin` define
`deadzone = static + velocity_margin*abs(v) + acceleration_margin*abs(a)`.
`filter_s` filters signed residuals; `acceleration_filter_s` filters acceleration
estimated from distinct receive timestamps. `confirm_s` and `release_s` confirm
load/release. Once confirmed, ordinary output is
`effective_cap * clamp((abs(filtered_residual)-deadzone)/full_scale_nm, 0, 1)`.
`rise_mw_s`/`fall_mw_s` limit power changes. Lowered caps apply immediately at the
next output update; no cached sample can increase load/stall confirmation.

HOLD uses `hold_error`, `hold_load_nm`, `hold_speed`, `hold_confirm_s` and
`hold_progress` (maximum reduction in error per second that counts as no progress).
Sustained error, load, low speed and insufficient progress on any axis request
1500 mW holding on **all seven Leader joints**. Error uses the actual target
returned after the existing Follower's speed/step/limit clamps. Release requires
all desired-to-measured errors and loads below `hold_release_error` and
`hold_release_load_nm` for `hold_release_s`. Load is filtered residual magnitude;
the gripper uses its independent motor torque. Ordinary caps, including 0, do not
disable HOLD. Software manual HD hold for a legacy board affects only arm IDs
0–5. A native board controls those Leader joints itself, independently of these
software caps; it is not supported in `run`.

`guards`: `can_bitrate` is 1000000 bit/s for this classic-CAN SDK integration;
`can_skew_s` bounds inter-axis receive skew; `leader_query_s` bounds a fresh
serial query; `leader_gap_s` bounds continuous missing Leader replies;
`cycle_timeout_s` bounds loop heartbeat; `startup_feedback_s` bounds waiting for
complete state after SDK initialization. Choose them using actual update rates.
The heartbeat must exceed one loop period plus query time. The launcher checks
the existing SocketCAN UP state and bitrate; it does not configure the interface.

## Output, safety checks and dashboard

The model computes each axis's theoretical unloaded torque from the **whole**
URDF, measured posture, velocity and estimated acceleration. Each residual is
that axis's measured torque minus its model torque. There is no averaging among
joints or isolated single-link model. Ordinary damping follows magnitude, not
precise signed external force. Friction, gearing and model error remain in the
residual; it is not accurate end-effector force or weight. Gripper torque bypasses
the six-axis model.

Default common/axis caps are 1200 mW. The common cap is 400–1500 mW; each axis is
0–1500 mW. The effective cap is their minimum. These are Leader requested powers,
not measured mechanical resistance. In supported `run` configurations, a single
software writer arbitrates ordinary damping, legacy-board HD and automatic HOLD.
Native board commands are independent hardware writes and are not covered by
this arbitration; detecting such a board blocks `run`. Zero ordinary power sends release mode, avoiding firmware
special meanings of damping with power zero. Button ID7 is never a force target.

Leader input is checked from new checksum-validated serial replies, not cached
`angle_monitor`. A partial sample pauses new targets and confirmation timers;
continuous gaps stop. Fashionstar has no per-query sequence tag, so late replies
cannot be proved to belong to a particular request. CAN uses actual kernel
timestamps, detects repeated/stale/queued frames, faults, disable, temperature,
joint limits, jumps, speed, acceleration and inter-axis skew. Gripper freshness is
mandatory; this implementation adds no B601-style stale-gripper retry. The
first-seed SDK-cache readiness window above still requires fresh enabled CAN
and does not extend any CAN freshness limit.

The independent watchdog releases managed Leader outputs on CAN faults or loop
timeout; exceptions and partial writes also attempt release, before SDK cleanup.
Unsuccessful release is retried at cleanup and reported as an error. Broken
communication cannot guarantee physical release. A blocked native call or a
native call holding the Python GIL may delay software shutdown/watchdog execution.
Resource locks exclude other instances of this launcher; unrelated SDK controllers
do not honor those locks. Existing SDK optional protections are left unchanged.

Open `http://127.0.0.1:8765/` (`--port` changes the port). The page switches
Chinese/English and shows pose, joint torque, temperatures, frame age and Leader
requests/current. Follower current is unavailable without verified torque
constants and is displayed as `—`, never fabricated. `preview` and `inspect` disable cap edits
and the server rejects them. In permitted modes, damping parameter edits are
limited to power caps; server validation rejects damping-policy and guard edits.
The `serve` page separately offers the documented session, button-board and startup
choices. Requests require the local Host, origin and a session token. Telemetry
stays in memory. Current caps persist to
`~/.config/stararm_lumos_feedback/current.caps.json` (override `--settings`);
`serve` also stores its documented local button-board choice. Current-pose startup
references stay in memory. No session CSV, process log, snapshot or parameter
history is created. `--no-web`
disables the server. This launcher does not record LeRobot datasets; existing
record/replay commands remain separate.

## Offline validation and source

`lumos-feedback-model` remains an offline standard-library model tool:

```bash
PYTHONPATH=src python3 -B -m stararm_lumos_feedback.model \
  --urdf /actual/path/startouch_sdk/src/config/FastTouchV2.SLDASM.urdf \
  --gravity 0 0 -9.80665
PYTHONPATH=src python3 -B -m unittest discover -s tests -p 'test_*.py' -v
PYTHONPATH=src python3 -B -m unittest discover -s tests -p 'test_follow_control.py' -v
```

Model tests check energy gradients, coupled mass matrices, energy balance,
different gravity directions, explicit payloads and invalid models. Runtime tests
use synthetic CAN/serial, fault injection and HTTP. Adapter-boundary tests execute
the production mapping, filtering, HD and target clamps with stubbed LeRobot base
classes/device constructors; they are not full framework integration tests.
Framework integration tests use real LeRobot with simulated hardware. See
[VALIDATION.md](../VALIDATION.md) for software-validation scope and current results;
these tests do not establish physical following or force-output acceptance.

Baselines: teleoperation `454979c7c8b9d5ced15f1964a2d588acf531ae3f` (0.2.0), SDK
above, original feedback `c4ce25e` (0.6.2), second B601 subtree `604d60b` (0.7.2).
The original whole-arm dynamics architecture is retained. Neither original B601
repository nor the SDK was modified. Source tests do not validate physical
transparency, load stability, communication reliability or damping feel. Complete
offline validation → basic following → monitoring → physical damping/HOLD checks
on the actual configured hardware before calling the adaptation commissioned.

## Optional TypeLJ following gains

The local session's robot configuration may set both `gripper_follow_kp` and
`gripper_follow_kd`. Both default to `null`, preserving the original
`setGripperPosition` interface. Explicit values must be finite within the
reviewed SDK ranges: kp 0.1–20 and kd 0.1–1. The bound SDK YAML must explicitly
select TypeLJ and contain valid `default_kp`/`default_kd`.

This opt-in path sends normalized physical opening × 0.08 metres through
`setGripperDistance`; it preserves normalized targets and baseline comparisons.
Startup still uses the unchanged YAML defaults. Each endpoint's 10% opening
band smoothly blends following gains back to those defaults; exact open/closed
commands use the default gains. It changes neither motion limits nor physical
motor-range guards, and does not certify unloaded/load separation. Review values
for the actual assembly, regenerate the normal binding after session changes,
and verify physically before adopting them. These gains are not web-editable.
