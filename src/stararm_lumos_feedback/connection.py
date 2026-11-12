"""Single-owner, explicitly selected hardware sessions for the local dashboard.

Serving and construction remain offline. Every start reverifies the profile and
binding. Motion applies the selected startup policy before motor enabling.
"""

import threading

from .button_board import ButtonBoard
from .config import Profile, SDKMotorRangeError, native_motor_range_mismatches


class InspectionConnection:
    def __init__(self, profile, binding, dashboard, *, button_settings=None):
        self.profile, self.binding, self.dashboard = profile, binding, dashboard
        self.lock = threading.RLock()
        self.worker = self.observer = None
        self.cancel = threading.Event()
        self.state, self.error, self.closed = "idle", None, False
        self.mode = "inspect"
        self.startup_pose = "home"
        self.motion_blocked = False
        self.error_code = self.error_interface = self.error_detail = None
        self.error_diagnostics = None
        self.button_board = ButtonBoard(profile, button_settings)
        self.publish("connection_idle", clear=True)

    def publish(self, status, *, clear=False):
        state = {} if clear else self.dashboard.snapshot()
        state.update(
            status=status,
            mode=self.mode,
            error=self.error,
            error_code=self.error_code,
            error_interface=self.error_interface,
            error_detail=self.error_detail,
            error_diagnostics=self.error_diagnostics,
            leader_controlled=False,
            leader_output_state="unknown",
            residual_verified=False,
            baseline_eligible=False,
            requested_mw=[None] * 7,
            predicted_mw=[None] * 7,
            hold_requested=[None] * 7,
        )
        self.dashboard.publish(**state)

    def status(self):
        with self.lock:
            running = self.worker is not None and self.worker.is_alive()
            mismatch = native_motor_range_mismatches(self.profile)
            motion_modes = ("follow", "monitor", "run") + (
                ("experiment",) if getattr(self.profile, "experimental_damping", None) else ()
            )
            blocked = {"run": "force_profile_unreviewed"} if "run" not in self.available_modes() else {}
            if mismatch:
                blocked.update({mode: "sdk_motor_range_mismatch" for mode in motion_modes})
            if self.motion_blocked:
                blocked.update({mode: "sdk_cleanup_failed" for mode in motion_modes})
            return dict(
                state=self.state,
                mode=self.mode,
                startup_pose=self.startup_pose,
                startup_pose_options=["home", "current_aligned", "current_relative", "leader_target"] if getattr(self.profile, "engineering", False) else ["home"],
                available_modes=self.available_modes(),
                blocked_modes=blocked,
                sdk_motor_range_mismatches=mismatch,
                error=self.error,
                error_code=self.error_code,
                error_interface=self.error_interface,
                error_detail=self.error_detail,
                error_diagnostics=self.error_diagnostics,
                can_start=not running and not self.closed,
                can_stop=running and self.state != "stopping",
                button_board=dict(**self.button_board.get(), can_change=self._can_change_button()),
            )

    def _can_change_button(self):
        return (
            not self.closed
            and self.state not in ("starting", "active", "stopping")
            and not (self.worker is not None and self.worker.is_alive())
        )

    def set_button_board(self, enabled):
        with self.lock:
            if not self._can_change_button():
                raise RuntimeError("Stop the hardware session completely before changing the button board setting")
            self.button_board.set(enabled)
            return self.status()

    def available_modes(self):
        if self.motion_blocked or native_motor_range_mismatches(self.profile):
            return ["inspect"]
        modes = ["inspect", "follow", "monitor"]
        if getattr(self.profile, "experimental_damping", None):
            modes.append("experiment")
        if getattr(self.profile, "engineering", False):
            allowed = self.profile.commissioning["allowed_modes"]
            return [mode for mode in modes if mode in allowed]
        if getattr(self.profile, "mass_confirmed", False):
            modes.append("run")
        return modes

    def request(self, action, mode="inspect", *, startup_pose="home"):
        if action not in ("start", "stop"):
            raise ValueError("Only explicit session start/stop is supported")
        if startup_pose not in ("home", "current_aligned", "current_relative", "leader_target"):
            raise ValueError("Unknown startup pose strategy")
        if startup_pose != "home" and (
            action != "start" or mode not in ("follow", "monitor", "experiment") or not getattr(self.profile, "engineering", False)
        ):
            raise ValueError("Alternative startup strategies are limited to engineering follow/monitor sessions")
        if action == "start" and mode not in self.available_modes():
            if self.motion_blocked:
                raise RuntimeError("SDK cleanup failed; actuator state is unconfirmed. Further motion is blocked")
            mismatch = native_motor_range_mismatches(self.profile)
            if mismatch:
                raise SDKMotorRangeError(mismatch)
            raise ValueError("Requested mode is unavailable; force output requires a reviewed profile")
        with self.lock:
            if self.closed:
                raise RuntimeError("Dashboard connection is closed")
            if action == "stop":
                if self.worker is not None and self.worker.is_alive():
                    self.cancel.set()
                    self.state = "stopping"
                    self.publish("inspection_stopping" if self.mode == "inspect" else "stopped")
                return self.status()
            if self.worker is not None and self.worker.is_alive():
                raise RuntimeError("A hardware session is already running; stop it before switching modes")
            self.cancel = threading.Event()
            self.mode = mode
            self.startup_pose = startup_pose
            self.error, self.state = None, "starting"
            self.error_code = self.error_interface = self.error_detail = None
            self.error_diagnostics = None
            self.button_board.record(None)
            self.publish("inspection_waiting" if mode == "inspect" else "starting", clear=True)
            # Native motion/cleanup cannot be interrupted safely. A pending
            # motion owner must survive main-thread shutdown until it returns.
            self.worker = threading.Thread(target=self._run, name="lumos-web-session", daemon=mode == "inspect")
            self.worker.start()
            return self.status()

    def _run(self):
        from .inspection import Inspection
        from .motion import ConnectionCancelled, check_cancelled, create_session, preflight_session_pose
        from .runtime import SessionCancelled, SocketCANError, session_lock, validate_socketcan

        try:
            # Recheck files on every explicit attempt; serving a page cannot
            # authorize a stale profile/binding after local edits.
            profile = Profile(self.profile.path, purpose=self.mode)
            profile.verify(self.binding)
            interface = profile.session["robot"].get("can_interface", "can0")
            port = profile.session["teleop"].get("port", "/dev/ttyUSB0")
            with session_lock(interface, port):
                validate_socketcan(interface, profile.guards["can_bitrate"])
                check_cancelled(self.cancel)
                self.dashboard.profile = profile
                self.dashboard.read_only = self.mode == "inspect"
                if self.mode != "inspect":
                    self._run_motion(profile, create_session, preflight_session_pose, check_cancelled)
                    return
                with self.lock:
                    if self.cancel.is_set():
                        return
                    self.observer = Inspection(profile, dashboard=self.dashboard)
                    self.observer.stop = self.cancel
                    self.state = "active"
                self.observer.run()
        except (ConnectionCancelled, SessionCancelled):
            pass
        except Exception as exc:
            with self.lock:
                self.error, self.state = str(exc), "error"
                self.error_diagnostics = getattr(exc, "diagnostics", None)
                display_pose = getattr(exc, "display_pose", None)
                if isinstance(display_pose, dict):
                    self.dashboard.remember_pose(**display_pose)
                if getattr(exc, "code", None) == "sdk_cleanup_failed":
                    self.motion_blocked = True
                if isinstance(exc, SocketCANError) or getattr(exc, "code", None) in (
                    "startup_pose_mismatch", "sdk_cleanup_failed", "button_board_no_reply",
                    "gripper_motor_range", "gripper_motor_velocity",
                    "button_board_command_incomplete",
                    "native_button_force_unreviewed",
                    "sdk_motor_range_mismatch",
                ):
                    self.error_code = exc.code
                    self.error_interface = getattr(exc, "interface", None)
                    self.error_detail = getattr(exc, "detail", None)
                if self.error_code in ("button_board_no_reply", "button_board_command_incomplete"):
                    self.button_board.record(False)
            self.publish("inspection_error" if self.mode == "inspect" else "stopped")
        finally:
            with self.lock:
                self.observer = None
                if self.state != "error":
                    self.state = "stopped"
                    self.publish("inspection_stopped" if self.mode == "inspect" else "stopped")

    def _run_motion(self, profile, create_session, preflight_session_pose, check_cancelled):
        connection = self

        class Publication:
            def button_status(self, verified):
                with connection.lock:
                    connection.button_board.record(verified)

            def publish(self, **state):
                with connection.lock:
                    if state.get("status") in ("active", "leader_gap"):
                        if connection.cancel.is_set():
                            state.update(status="stopped", requested_mw=[None] * 7, hold_requested=[None] * 7)
                        else:
                            connection.state = "active"
                    elif str(state.get("status", "")).startswith(("stopped", "leader_release_failed")):
                        connection.state = "stopping"
                    connection.dashboard.publish(**state)

        options = {
            "button_enabled": self.button_board.enabled,
            "button_wait_until_cancel": True,
        }
        if self.startup_pose != "home":
            options["startup_pose"] = self.startup_pose
        session = create_session(
            profile, self.dashboard.caps, Publication(), self.cancel, self.mode,
            **options,
        )
        with self.lock:
            self.observer = session
        if self.startup_pose in ("current_aligned", "current_relative", "leader_target"):
            session.before_enable = lambda transport: preflight_session_pose(
                profile, self.cancel, transport, robot=session.robot,
            )
        else:
            session.before_enable = lambda transport: preflight_session_pose(profile, self.cancel, transport)
        check_cancelled(self.cancel)
        session.run()

    def close(self):
        with self.lock:
            self.closed = True
            self.cancel.set()
            worker = self.worker
        if worker is not None:
            worker.join(timeout=3)
            if worker.is_alive():
                raise RuntimeError("Hardware worker has not stopped; resource lock remains held")
