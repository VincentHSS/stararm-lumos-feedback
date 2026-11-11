# 0.6.28 hardware startup

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

Initial unloaded collection requires a complete real button release and at least one complete successful following write. Arm and gripper target catch-up and measured arrival are admitted separately; all seven measured axes, including the raw gripper motor, must still remain stationary for at least two seconds. When the first group qualifies, every group ready in that cycle freezes and the initial collection window closes. Unready groups keep zero damping for this session and cannot be learned later during output. The dashboard reports arm and gripper status separately; gripper arrival error no longer blocks a qualified arm. Button waiting, supported startup poses and each group’s initial approach cannot qualify that group. Arrival tolerances and physical limits are unchanged. Button changes and external loads never relearn a frozen baseline. A single-pose baseline still cannot establish unloaded separation over the entire motion range.

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


After strict startup agreement, 0.6.19 engineering basic following can match a
lagging SDK arm position to a real CAN history sample within the original age and
angle limits. Current CAN must remain fresh. History is memory only; it does not
prove SDK cache age. Other modes, startup seeds and gripper checks remain strict.

Engineering basic `follow` with direct `leader_target` startup permits bounded
post-enable movement in 0.6.18. Pre-enable stability, origin drift capped at 2°,
actual velocity/position limits, SDK/CAN agreement and the original target rate
limits remain. Other startup strategies and monitoring retain quiet admission.

For engineering basic `follow`, version 0.6.17 omits the six-axis derived dynamics
acceleration admission. It is not used for following targets or damping in this
mode. Monitoring, force output, non-engineering profiles and gripper checks retain
their original acceleration checks. All other motion and communication guards
remain enforced. No threshold enlargement or calibration change is applied.

After installing the reviewed 0.6.13 source-based native velocity-range
correction and regenerating the binding, basic following and following with
monitoring can pass their chosen startup admission. Version 0.6.16 adds the web option “Follow Leader directly
(rate-limited approach)”: neither arm must align or return home, and the gripper
need not be closed beforehand. The SDK may automatically close it. Fresh measured
Follower positions seed the existing rate limiter, which approaches the absolute
Leader target under original joint, speed and step limits without recalibration.
Communication, faults, temperatures, measured limits, SDK/CAN agreement, origin
stability and SDK J2 construction conditions remain checked. Brief motion before
the first seed remains possible. Confirm model coverage for the actual assembly;
formal force feedback still requires acceptance of measured tuning and response.

The older entries below explain the software changes. The original 0.6.11 range
gate passes only with a valid reviewed correction matching the actual hardware.

## 0.6.11 history

Version 0.6.11 blocks a confirmed native SDK range mismatch before enabling.
The pinned core uses ±8 rad/s for J1–J3. On an installation whose verified RID22
is ±10 rad/s, this mismatch affects both native command packing and feedback
decoding. Read-only queries and the real 3D pose remain available; following and
force modes require a valid reviewed correction matching the actual hardware.
Do not mask a discrepancy by changing hardware ranges, Python scaling or binding
hashes. A J1–J3 range mismatch does not explain a derived-acceleration failure on
another axis whose hardware and native ranges already agree.

J1–J6 acceleration protection now retains a feasible velocity quantization interval
across the complete sequence. Expand the previous interval by the unchanged
acceleration limit times the genuine receive interval, intersect the current
one-code bin, and stop on an empty intersection. This avoids a single boundary
code toggle while rejecting a rapid sustained staircase; thresholds, elapsed time
and per-frame allowances are not inflated. The nonlinear gripper derivative guard
and raw acceleration estimation/filtering formulas remain unchanged. The filtered model acceleration must also remain finite and within its original configured limit; otherwise reject without resetting, clipping or committing the candidate state. Quantization false
trips were reproduced offline; an individual hardware failure still requires its
own analysis. Kernel receive times are not motor sampling times. Errors now include numeric
velocity, interval, limit and feasible bounds. Physical holding, following and
damping remain unvalidated.

Version 0.6.10 guards the first fixed joint seed while the SDK state cache becomes
ready. Native review confirms that the getter returns six zeros until the
control-loop cache is valid; the constructor has no readiness barrier before
returning. An all-zero SDK vector before the first target does not establish a
physical zero pose. All six positions must agree with genuinely new independent
CAN feedback within the bound tolerances.

For current-pose startup only, first-seed readiness may wait at most 100 ms,
shortened by all seven CAN age limits and the fixed preflight reference's expiry.
The CAN callback and the check immediately before sending the seed enforce the
same absolute deadline; retries never reset it. Python sends no joint target until
all six SDK positions agree with genuinely new enabled CAN feedback within the
unchanged axis tolerances. Drift, motion, limits, gripper/CAN failures and
cancellation do not retry. Comparisons after the seed and after Leader requery
remain immediate. The native SDK has already enabled motors and may send MIT
control and gripper-closing commands during this wait. Keep stable support;
physical holding, following and damping remain unvalidated.

Version 0.6.9 corrects SDK resource layout and verifies the loaded native library's
path as well as its contents. It retains the constructor GIL correction for the Python CAN receiver;
the original CAN freshness limits remain strict. Preserve the original SDK and
copy it into its final physical location **before patching or rebuilding**. The
SDK root must be a real directory named `startouch_sdk`, using the layout
`.../<version>/startouch_sdk/src/config/robot_kinematics.yaml`. A version directory
alone is not a valid SDK root. A symlink alias does not fix an incorrectly named
physical root. For example:

```bash
LUMOS_SDK_COPY=/path/to/sdk/release-id/startouch_sdk
python tools/patch_sdk_constructor_gil.py "$LUMOS_SDK_COPY"
python tools/patch_sdk_constructor_gil.py "$LUMOS_SDK_COPY" --check
```

Rebuild that copy's extension with the existing CMake/Python 3.12 environment in
that final location. Build RPATH/RUNPATH can contain source or build paths: moving
an already built SDK can retain references to its former location. If moved,
rebuild at the destination rather than copying the old extension into place.
Point a local session copy's `robot.sdk_root` and the profile's SDK/model paths to
it, then generate a new binding with the normal `bind` command. See the
[SDK review](sdk-startup-review.md). Preserve calibration; never edit binding hashes.
The patch leaves the motor core library, gains, model and CAN age limits intact
and does not establish physical holding or following. The page now saves an
explicit startup-strategy preference in this browser across reloads; starting
still requires an explicit button click.

Before calling any native controller constructor, verify the physical SDK layout
and readable YAML without importing the SDK or accessing devices:

```bash
python -B - "$LUMOS_SDK_COPY" <<'PY'
from pathlib import Path
import sys
sdk = Path(sys.argv[1]).expanduser().resolve(strict=True)
if sdk.name != "startouch_sdk" or not sdk.is_dir():
    raise SystemExit("Physical SDK root must be a directory named startouch_sdk")
config = sdk / "src/config/robot_kinematics.yaml"
if not config.read_bytes():
    raise SystemExit("SDK robot_kinematics.yaml is empty")
print(config)
PY
```

The pinned native core uses SDK-directory candidates when locating resources.
Keep the final physical layout as
`.../<version>/startouch_sdk/src/config/robot_kinematics.yaml`; a differently named
root containing the YAML alone does not establish the required layout. Importing
the extension and reading its GIL-patch marker do not exercise native
configuration lookup. A missing-configuration failure at this stage occurs before
the constructor's CAN initialization and motor enabling; read-only preflight CAN
queries may already have run. Correcting the path does not validate physical
holding, following or force feedback.

The page offers **With button board / No button board**. Stop the session fully
before changing it; the choice applies to the next connection. Only the current
choice persists in `~/.config/stararm_lumos_feedback/current.button-board.local.json`;
`serve --button-settings PATH` selects another local file. Pairing calibration,
the original session and force authorization are unchanged. The board ID still
comes from the bound configuration. Leader board ID 7 and Follower gripper motor
ID 7 are devices on separate buses.

Configuration and actual reply verification are separate. Read-only CAN queries
do not inspect the Leader. At a following connection, the initial read-only probe
makes at most three attempts. A legacy board replies to the ID 7 ping and then
must provide continuous fresh button-monitor frames. A newer native HD board
instead emits hold/release commands itself. After the waiting prompt appears,
support the Leader and press HD once. The page keeps listening until a real full
event arrives or you select Stop; operator waiting does not expire after sixty seconds.
Only a complete received
command 24 group covering all six arm IDs 0–5 with the same mode verifies this
board and permits Lumos SDK construction/enabling. Stop cancels this wait; no
SDK construction/enabling or software force output occurs while waiting. This
wait precedes a newly captured pose preflight. Direct adapter/CLI use retains the
sixty-second operator window. The webpage operator wait does not relax frame
freshness or the 0.25-second incomplete-group deadline below. A missing or incomplete reply never changes the
selection automatically.

The software listens to native command groups to freeze/resume new Follower
targets; it does not resend those commands. Native commands occur on button
edges, so no periodic native-button frame is required. An incomplete group, a
partial frame with the exact native request header, or a matching frame with a
bad checksum immediately leaves the button state pending and freezes new
targets. Failure to complete a valid same-mode six-axis group within 0.25 seconds
ends the session. Legacy monitor freshness and all joint missing-frame guards
remain enabled. No-board mode does not detect the board, and the selector cannot
disable a physically connected board's native control of Leader resistance.

The selector does not authorize software force feedback. In follow/monitor modes,
this software sends no Leader damping/hold commands, but a native HD board still
changes Leader resistance directly, even when this program is not running. On
legacy boards, HD freezes new Follower targets in follow/monitor; only reviewed
force mode also requests software manual HOLD on Leader J1–J6, excluding the
gripper. `run` additionally rejects a detected native HD board: interaction
between its own commands and software damping is not validated. Engineering
profiles already reject `run`. Gripper position/velocity errors show raw
measurements and allowed ranges; see the bound commissioning observation below.
Native HD support currently applies to this feedback dashboard/CLI's
`follow/monitor` modes. The original `lerobot-teleoperate` entry point rejects
native-board action reads; it does not provide this continuous command listener.

[中文](hardware.zh.md) · [Detailed configuration and feedback design](feedback.md)

This version targets a paired-calibrated StarArm 102 and upright Lumos with
TypeLJ and no additional tool. Engineering profiles support state queries,
following, monitoring and explicitly authorized experimental damping. Review
model coverage, tuning and hardware response for each assembly. Engineering
profiles cannot use formal `run`; a release number does not validate hardware effects.

## Start the state dashboard

From the source directory, using the existing Python 3.12, SDK and dependencies:

```bash
conda activate lumos-lerobot
bash scripts/run_feedback.sh
```

The default mode is `serve`, at `http://127.0.0.1:8766/`. Startup does not access
hardware; the genuine CAD model shows a reference pose. Connect CAN and power,
check that can0 is UP at 1000000 bit/s, then select “Connect hardware (read only)”.
This only sends DM state refresh requests. It does not construct the SDK, enable
motors, send targets or open Leader serial. Complete new feedback drives live
pose, torque, temperature and age. Incomplete, incoherent or stale seven-axis data
cannot be labeled live.

Show calibrated target is enabled by default. The translucent cyan model shows
the paired Follower startup pose in default home mode. `current_aligned` instead
uses fresh Follower state and Leader inverse targets from the existing calibration.
`current_relative` uses fresh Follower state and displays Leader calibration-home
values as references, without requiring the Leader to reach those values.
Original colors show the actual pose, or a labeled reference when
disconnected. Follower current/goal/remaining values use SDK joint degrees;
remaining = goal − current, not clockwise from the camera. The Leader column
contains servo degrees by ID: original home values in home mode, current inverse
targets in `current_aligned`, and home references in `current_relative`.
They are not Follower joint angles.
Stale or absent feedback hides current differences. Gripper opening need not match
calibration. The displayed goal never commands homing. Proximity to the Follower
goal does not establish that Leader mapping, pair alignment and limit checks pass.

“Stop queries” closes this page's observer; it is not a mechanical emergency stop.
The page retains the last pose with a stopped-data label, and retains error causes.
Retries require an explicit click and reverify the local binding. There is no
automatic rebinding, retry, enabling or motion resumption. Ctrl+C exits the
service; SIGTERM follows the same cleanup path.

`PYTHON_BIN` selects Python. `FEEDBACK_PROFILE`, `FEEDBACK_BINDING` and
`FEEDBACK_PORT` override local startup choices. Source releases exclude machine
setup; another computer needs its own profile and binding as described in the guide.

## Connection diagnostics

SDK imports prefer the profile-bound `interface_py` build directory. Wrapper,
extension and loaded native-library hashes remain strictly checked. CMake install
can rewrite an extension's RUNPATH, changing its file hash; this version loads the
exact bound build instead of relaxing verification. If a different native module
is already loaded, exit and restart; modules are never replaced in a running process.

0.5.1 distinguishes a missing interface, a down interface, bitrate mismatch,
bus fault, failed system query, missing `ip` command and query timeout in both
languages. Use `lsusb` and `ip -details link show can0` for read-only checks.
If USB-CAN or `can0` is absent, reconnect its USB data cable and try another
USB port, then retry after detection. If the interface exists but is down,
configure 1000000 bit/s according to the local wiring and profile. The page never
changes network settings, resets USB or bypasses checks automatically. Leader
serial is a separate USB device; successful CAN queries do not validate it.

## Start following and monitoring from the same page

The page provides Basic following, Following and monitoring, and Force feedback controls.
Stop the current state query before choosing a mode. Only one query or control
session owns the devices, and restart waits for cleanup. Following enables Lumos
and sends motion targets. Monitoring also computes dynamics, residuals, predicted
damping and in-memory stationary statistics; neither mode writes Leader damping.
A native HD board can independently hold/release the Leader in either mode.
Force feedback requires a reviewed profile. The engineering profile shows a
blocked control and its reason; the page cannot change mass coverage or authorization.

Motion first verifies the profile, binding and SDK origin, initializes Leader
queries and CAN monitoring, then checks fresh Follower and mapped Leader poses
against paired homes and limits before constructing the Lumos SDK. The gate uses
existing startup tolerances; the first gripper target remains the measured opening.
Misalignment retains per-axis diagnostics without enabling Lumos, moving to home
automatically or changing calibration. This remains the default home policy;
CLI examples below use it.

### Explicit current-pose alignment for engineering follow/monitor

Choose “Align at current pose” (`current_aligned`) explicitly on the page. This
choice is available only for engineering `follow/monitor`; it does not authorize
force output. Home alignment remains the default, and the original paired
calibration, coordinate mapping, directions and limits remain in use.

1. Let the Follower rest on stable support in the intended starting pose. You do
   not need to lift J4 to the calibration home. Check that this pose and its
   surrounding workspace permit the intended following motion.
2. Use the page's current inverse targets to align Leader IDs 0–5 to that Follower
   pose. These values come from the existing calibration; they are servo degrees,
   not Follower joint degrees. Close the TypeLJ gripper before starting.
3. Stop read-only queries and select basic following or following with monitoring.
   If a native board is present, wait for its detection prompt before pressing HD.
   Keep the Follower's support in place during all startup checks.

Before constructing the SDK, the current-pose gate requires at least 0.5 s of
continuous stable, genuinely new feedback; pair errors at most 2° or the smaller
original per-axis tolerance; existing limits; SDK J2 ≥ −1.57 rad; and Follower
normalized gripper opening ≤ 0.01. Cached, repeated or stale data cannot establish
stability. The startup reference is temporary and never rewrites paired home.

After SDK construction, fresh CAN is checked for drift from the accepted pose and
agreement with SDK feedback. Only then is a fixed raw pose target with zero target
velocity published. Two more seconds of stability checks precede normal following
with the existing rate and step limits. A failed check stops startup instead of
adopting a changed pose or widening a tolerance.

The SDK enables motors and starts its own control loop inside construction,
before this program can publish the fixed target. The sequence is not atomic
holding. Keep stable support: a completed check does not establish that removing
support will hold J4. This new startup procedure has not been verified on physical
hardware. Confirm model coverage for the actual assembly, while measured response
and native-board arbitration still require acceptance before formal force output. The [SDK startup review](sdk-startup-review.md)
records the pinned source and binary evidence.

### Explicit current-position relative startup for engineering follow/monitor

Version 0.6.6 adds the explicit `current_relative` page choice, “Relative start at current pose (engineering)”. It is restricted to engineering `follow/monitor`. Home remains
the default; `current_aligned` remains available. A Follower pose can pass its own
checks while its inverse Leader target lies outside the paired calibration range.
Absolute current alignment rejects that target. Relative startup addresses this
case without changing the stored calibration or requiring the Leader to reach an
unavailable absolute target.

1. Rest the Follower on stable support, close TypeLJ and check the intended workspace.
   Place the Leader in a stable, usable pose within its calibrated range. The page
   shows its calibration-home values as references; relative startup does not
   require reaching them or an inverse absolute target.
2. Explicitly select the relative option, stop read-only queries, then start basic
   following or following with monitoring. If a native board is present, press HD
   only after the detection prompt appears. Keep both arms still while their origins are sampled
   and keep Follower support in place throughout startup.
3. Preflight checks stable Follower feedback and queries the raw Leader origin. For each of six arm joints,
   the fixed session offset is `actual Follower SDK position − original mapped
   Leader target`. Every later target is `original mapped target + fixed offset`.
   This offset remains constant for the session; drift does not update it.

The original directions, gains, IDs, limits, velocity and step guards stay active.
The gripper receives no relative offset and retains its existing opening mapping
and target limiting. Follower seven-axis feedback must remain continuously fresh and stable for at
least 0.5 s. Leader origins are obtained by query; after SDK construction, another
Leader query must agree using the same fixed offset. All current-pose admission and startup checks above remain active,
including per-axis agreement with the offset-adjusted target within
`min(2 degrees, target step limit, original startup tolerance)`, limits, SDK J2 ≥ −1.57 rad, closed
Follower gripper, post-construction CAN/SDK agreement and drift checks, a fixed
raw target with zero target velocity and two seconds of stability before following.
The session offset is cleared on cleanup; it is never written to calibration,
settings, logs or a history file.

The SDK still enables inside construction before the fixed target is sent. This
option does not provide atomic holding or establish that J4 will stay when support
is removed. Physical relative startup and following remain unvalidated. Confirm model coverage for the actual assembly; formal force feedback still awaits measured
response and native-HD arbitration acceptance.

Stop and exit control prevents subsequent targets, releases managed Leader output
and exits the SDK. Calls already inside the SDK cannot be withdrawn. Gravity
support may be lost on exit; support the Follower first. This is not an emergency
stop and does not guarantee mechanical holding. Faults do not trigger automatic
reconnection or motion resumption. The page persists current power caps and its
documented local button-board choice. Current-pose startup references stay in memory.

If SDK cleanup fails, the page reports unconfirmed actuator state and blocks all
further motion starts in that page; read-only diagnostics remain available.
Support the Follower and investigate its actual state. A stopped page does not
establish that motors are disabled.

## Commission from the command line

Exit the query service first. Align both arms with their paired calibration homes
and stop other controllers using the same CAN/serial devices. A companion on a
different web port can still use the same CAN bus. Resource locks exclude only
programs that honor the same locks.

```bash
# Both commands construct the SDK, enable motors and follow physically.
# Neither writes Leader force output; a native HD board acts independently.
bash scripts/run_feedback.sh follow --seconds 15
bash scripts/run_feedback.sh monitor --seconds 30
```

`follow` retains per-frame acceleration, speed, limits, temperature, fault and
communication checks. `monitor` also computes whole-arm dynamics, residuals,
predicted damping and in-memory stationary statistics. An error stops the session;
do not widen guards or edit hashes to bypass checks. After validating following,
mass coverage and multi-pose unloaded/loaded tuning, create a separately reviewed
`configs/feedback.reviewed.local.json` and binding before using `run`.
Engineering profiles are never automatically promoted to force profiles.
Native HD board detection also blocks `run`, independently of profile approval.

## Bound TypeLJ closed-end observation

An engineering `version: 3` profile may record an operator-confirmed fully closed
gripper after stable, genuinely new raw motor frames have been observed:

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
The observation changes no global offset, SDK command, coordinate conversion or
paired calibration. The profile and observation require normal binding
regeneration. Only engineering inspect/follow/monitor accept this field; reviewed
force profiles reject it. This records the guard reference, not validation of
following, damping or the complete physical travel.

## Version, binding and delivery

```bash
PYTHONPATH=src python -B -m stararm_lumos_feedback.cli --version
PYTHONPATH=src python -B -m stararm_lumos_feedback.cli bind \
  --profile configs/commissioning.local.json \
  --binding configs/commissioning.binding.json --purpose inspect --replace
```

Rebinding preserves calibration. Among damping parameters, the dashboard can change
only common/per-axis power caps in permitted modes. Session controls explicitly
start/stop permitted modes and offer the documented button-board and startup
choices; they cannot edit bound hardware, mapping, damping or guard parameters.
No session CSV, process logs, telemetry snapshots or parameter history are created;
local setup, bindings, current caps and the local board choice are retained.
Current-pose startup references stay in memory. Source delivery includes original STL assets and provenance, without
local files, SDK binaries or built installers. See [VALIDATION](../VALIDATION.md)
for software and hardware verification boundaries.
