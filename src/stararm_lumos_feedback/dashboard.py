"""Loopback dashboard. Current settings persist; telemetry remains in memory."""

import json
import math
import os
import secrets
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from . import __version__
from .config import read_json


class Dashboard:
    def __init__(self, caps, profile, *, port=8765, settings=None, clock=time.monotonic):
        self.caps, self.profile = caps, profile
        self.clock = clock
        ages = [
            axis["max_age_s"]
            for axis in getattr(profile, "axes", [])
            if isinstance(axis.get("max_age_s"), (int, float))
            and math.isfinite(axis["max_age_s"])
            and axis["max_age_s"] > 0
        ]
        self.freshness_limit_s = min(ages) if ages else 0.25
        self.settings = None if settings is None else Path(settings).expanduser().resolve()
        if self.settings is not None and self.settings.exists():
            self.caps.set(read_json(self.settings))
        self.token = secrets.token_urlsafe(32)
        self.lock = threading.Lock()
        self.settings_lock = threading.Lock()
        self.read_only = getattr(profile, "purpose", None) == "inspect"
        self.state = dict(status="starting", mode=getattr(profile, "purpose", "unknown"))
        self.last_pose = None
        self.published_at = self.clock()
        self.publication_sequence = 0
        self.connection = None
        self.visual_model = None
        if getattr(profile, "model", None) is not None:
            from .visualization import description

            self.visual_model = description(profile)
        dashboard = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def reply(self, code, value, mime="application/json"):
                data = (
                    json.dumps(value, ensure_ascii=False, allow_nan=False).encode()
                    if mime == "application/json"
                    else value
                )
                self.send_response(code)
                self.send_header("Content-Type", mime + "; charset=utf-8")
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Content-Type-Options", "nosniff")
                self.send_header(
                    "Content-Security-Policy",
                    "default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'; frame-ancestors 'none'",
                )
                self.end_headers()
                self.wfile.write(data)

            def trusted(self):
                expected = f"127.0.0.1:{self.server.server_port}"
                return self.headers.get("Host") == expected and self.headers.get("Origin") in (
                    None,
                    "http://" + expected,
                )

            def do_GET(self):
                if not self.trusted():
                    self.reply(403, {"error": "Loopback origin required"})
                    return
                if self.path == "/api/state":
                    state = dashboard.snapshot()
                    self.reply(
                        200,
                        dict(
                            **state,
                            caps=dashboard.caps.get(),
                            token=dashboard.token,
                            policy=dashboard.profile.policy,
                            profile_status=dashboard.profile_status(),
                            caps_writable=dashboard.caps_writable(state),
                        ),
                    )
                elif self.path == "/api/model":
                    if dashboard.visual_model is None:
                        self.reply(404, {"error": "URDF model unavailable"})
                    else:
                        self.reply(200, dashboard.visual_model)
                elif self.path.startswith("/assets/"):
                    # Serve packaged viewer resources only; no SDK/configuration
                    # files or path traversal can be reached through this route.
                    web = (Path(__file__).parent / "web").resolve()
                    asset = (web / self.path[len("/assets/") :]).resolve()
                    mime = {".js": "text/javascript", ".stl": "model/stl", ".txt": "text/plain", ".svg": "image/svg+xml"}.get(
                        asset.suffix.lower()
                    )
                    if not asset.is_relative_to(web) or not asset.is_file() or mime is None:
                        self.reply(404, {"error": "Asset not found"})
                    else:
                        self.reply(200, asset.read_bytes(), mime)
                elif self.path in ("/", "/app.js", "/style.css"):
                    name = "index.html" if self.path == "/" else self.path[1:]
                    mime = {"index.html": "text/html", "app.js": "text/javascript", "style.css": "text/css"}[
                        name
                    ]
                    self.reply(200, (Path(__file__).parent / "web" / name).read_bytes(), mime)
                else:
                    self.reply(404, {"error": "Not found"})

            def do_POST(self):
                if not self.trusted() or self.headers.get("X-Feedback-Token") != dashboard.token:
                    self.reply(403, {"error": "Invalid local origin/token"})
                    return
                if self.path == "/api/experimental-baseline":
                    if dashboard.connection is None:
                        self.reply(409, {"error": "Connection controls unavailable", "code": "connection_unavailable"})
                        return
                    try:
                        size = int(self.headers.get("Content-Length", "0"))
                        if not 0 < size <= 4096:
                            raise ValueError("Invalid request size")
                        self.connection.settimeout(1)
                        value = json.loads(self.rfile.read(size))
                        if (
                            not isinstance(value, dict) or set(value) != {"unloaded_confirmed"}
                            or type(value["unloaded_confirmed"]) is not bool
                            or value["unloaded_confirmed"] is not True
                        ):
                            raise ValueError("An explicit boolean unloaded_confirmed=true declaration is required")
                        self.reply(200, dashboard.connection.request_baseline_recapture())
                    except (ValueError, TypeError, KeyError, OSError) as exc:
                        self.reply(400, {"error": str(exc)})
                    except RuntimeError as exc:
                        error = {"error": str(exc)}
                        if getattr(exc, "code", None):
                            error["code"] = exc.code
                        self.reply(409, error)
                    return
                if self.path == "/api/button-board":
                    if dashboard.connection is None:
                        self.reply(409, {"error": "Connection controls unavailable", "code": "connection_unavailable"})
                        return
                    try:
                        size = int(self.headers.get("Content-Length", "0"))
                        if not 0 < size <= 4096:
                            raise ValueError("Invalid request size")
                        self.connection.settimeout(1)
                        value = json.loads(self.rfile.read(size))
                        if not isinstance(value, dict) or set(value) != {"enabled"} or type(value["enabled"]) is not bool:
                            raise ValueError("Only a boolean enabled setting is supported")
                        self.reply(200, dashboard.connection.set_button_board(value["enabled"]))
                    except (ValueError, TypeError, KeyError, OSError) as exc:
                        self.reply(400, {"error": str(exc)})
                    except RuntimeError as exc:
                        self.reply(409, {"error": str(exc)})
                    return
                if self.path == "/api/connection":
                    if dashboard.connection is None:
                        self.reply(409, {"error": "Connection controls unavailable", "code": "connection_unavailable"})
                        return
                    try:
                        size = int(self.headers.get("Content-Length", "0"))
                        if not 0 < size <= 4096:
                            raise ValueError("Invalid request size")
                        self.connection.settimeout(1)
                        value = json.loads(self.rfile.read(size))
                        if not isinstance(value, dict) or value.get("action") not in ("start", "stop"):
                            raise ValueError("Only start/stop sessions are supported")
                        if set(value) == {"action"}:
                            result = dashboard.connection.request(value["action"])
                        elif set(value) == {"action", "mode"} and value["action"] == "start" and value["mode"] in ("inspect", "follow", "monitor", "experiment", "run"):
                            result = dashboard.connection.request(value["action"], mode=value["mode"])
                        elif (
                            set(value) == {"action", "mode", "startup_pose"}
                            and value["action"] == "start"
                            and value["mode"] in ("follow", "monitor", "experiment")
                            and value["startup_pose"] in ("home", "current_aligned", "current_relative", "leader_target")
                        ):
                            result = dashboard.connection.request(
                                value["action"], mode=value["mode"], startup_pose=value["startup_pose"],
                            )
                        else:
                            raise ValueError("Only start with a supported mode, or stop without a mode, is supported")
                        self.reply(200, result)
                    except (ValueError, TypeError, KeyError, OSError) as exc:
                        error = {"error": str(exc)}
                        if getattr(exc, "code", None):
                            error.update(code=exc.code, detail=getattr(exc, "detail", None))
                        self.reply(400, error)
                    except RuntimeError as exc:
                        self.reply(409, {"error": str(exc)})
                    return
                if self.path != "/api/caps":
                    self.reply(404, {"error": "Unknown writable endpoint"})
                    return
                if not dashboard.caps_writable(dashboard.snapshot()):
                    self.reply(
                        403,
                        {"error": "Current state does not allow Leader power cap changes", "code": "caps_read_only"},
                    )
                    return
                try:
                    size = int(self.headers.get("Content-Length", "0"))
                    if not 0 < size <= 4096:
                        raise ValueError("Invalid request size")
                    self.connection.settimeout(1)
                    value = json.loads(self.rfile.read(size))
                    # Validate before writing. Other policy fields cannot be changed here.
                    from .control import Caps

                    checked = Caps(value["common_mw"], value["axis_mw"])
                    checked.set(value)
                    with dashboard.settings_lock:
                        if dashboard.settings is not None:
                            dashboard.settings.parent.mkdir(parents=True, exist_ok=True)
                            temporary = dashboard.settings.with_name(dashboard.settings.name + ".tmp")
                            with temporary.open("w", encoding="utf-8") as stream:
                                json.dump(checked.get(), stream, ensure_ascii=False, allow_nan=False)
                            os.chmod(temporary, 0o600)
                            os.replace(temporary, dashboard.settings)
                        dashboard.caps.set(checked.get())
                    self.reply(200, dashboard.caps.get())
                except (ValueError, TypeError, KeyError, OSError) as exc:
                    self.reply(400, {"error": str(exc)})

        self.server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
        self.thread = threading.Thread(target=self.server.serve_forever, name="lumos-dashboard", daemon=True)

    def profile_status(self):
        """Expose commissioning flags, never local paths or source evidence."""
        commissioning = getattr(self.profile, "commissioning", None)
        baseline = (
            commissioning.get("baseline_status", "not_measured")
            if isinstance(commissioning, dict)
            else "not_recorded"
        )
        if not isinstance(baseline, str) or len(baseline) > 64 or not baseline.replace("_", "").isalnum():
            baseline = "unknown"
        engineering = bool(getattr(self.profile, "engineering", False))
        mass_confirmed = bool(getattr(self.profile, "mass_confirmed", False))
        return dict(
            mass_confirmed=mass_confirmed,
            engineering=engineering,
            baseline_status=baseline,
            force_ready=mass_confirmed and not engineering,
            experimental_damping_available=bool(getattr(self.profile, "experimental_damping", None)),
        )

    def caps_writable(self, state):
        return (
            not self.read_only
            and state.get("mode") not in ("inspect", "preview")
            and (
                state.get("status") not in ("active", "leader_gap")
                or state.get("telemetry_fresh") is True
            )
        )

    def start(self):
        self.thread.start()

    def publish(self, **state):
        now = self.clock()
        ages = state.get("age_ms")
        if (
            state.get("mode") in ("inspect", "follow", "monitor", "experiment", "run")
            and state.get("status") in ("active", "inspection_active", "leader_gap")
            and state.get("coherent_feedback", True)
            and self._finite_vector(ages, 7)
            and all(0 <= value <= self.freshness_limit_s * 1000 for value in ages)
        ):
            q_sdk, q_urdf = state.get("q_sdk"), state.get("q_urdf")
            if q_urdf is None and self._finite_vector(q_sdk, 7):
                to_model = getattr(self.profile, "to_model", None)
                if callable(to_model):
                    try:
                        q_urdf = [to_model(i, value) for i, value in enumerate(q_sdk[:6])]
                    except (ValueError, TypeError, KeyError, AttributeError, IndexError, OverflowError):
                        # A display cache must never interrupt telemetry publication.
                        q_urdf = None
            pose = self._validated_pose(q_sdk, q_urdf, now - max(ages) / 1000, now)
        else:
            pose = None
        with self.lock:
            self._remember_pose_locked(pose)
            self.state = state
            self.published_at = now
            self.publication_sequence += 1

    @staticmethod
    def _finite_number(value):
        try:
            return type(value) in (int, float) and math.isfinite(value)
        except OverflowError:
            return False

    @classmethod
    def _finite_vector(cls, value, length):
        return (
            isinstance(value, (list, tuple))
            and len(value) == length
            and all(cls._finite_number(item) for item in value)
        )

    def _validated_pose(self, q_sdk, q_urdf, captured_monotonic, now):
        if not (
            self._finite_vector(q_sdk, 7)
            and 0 <= q_sdk[6] <= 1
            and self._finite_vector(q_urdf, 6)
            and self._finite_number(captured_monotonic)
            and 0 <= captured_monotonic <= now
        ):
            return None
        return dict(
            q_sdk=list(q_sdk), q_urdf=list(q_urdf),
            captured_monotonic=captured_monotonic, display_only=True,
        )

    def _remember_pose_locked(self, pose):
        if pose is None or (
            self.last_pose is not None
            and pose["captured_monotonic"] <= self.last_pose["captured_monotonic"]
        ):
            return False
        self.last_pose = pose
        return True

    def remember_pose(self, q_sdk, q_urdf, captured_monotonic):
        """Retain measured geometry for display, independently of current telemetry."""
        pose = self._validated_pose(q_sdk, q_urdf, captured_monotonic, self.clock())
        with self.lock:
            return self._remember_pose_locked(pose)

    def snapshot(self):
        """Age the published samples even when the control thread stops advancing."""
        with self.lock:
            state = dict(self.state)
            age = max(0.0, self.clock() - self.published_at)
            sequence = self.publication_sequence
            last_pose = None if self.last_pose is None else {
                **self.last_pose,
                "q_sdk": list(self.last_pose["q_sdk"]),
                "q_urdf": list(self.last_pose["q_urdf"]),
                "age_s": max(0.0, self.clock() - self.last_pose["captured_monotonic"]),
            }
        if last_pose is not None:
            state["last_pose"] = last_pose
        state.update(
            age_s=age,
            publication_sequence=sequence,
            freshness_limit_s=self.freshness_limit_s,
            software_version=__version__,
        )
        ages = state.get("age_ms")
        if isinstance(ages, (list, tuple)):
            ages = [
                value + age * 1000
                if isinstance(value, (int, float)) and math.isfinite(value) and value >= 0
                else None
                for value in ages
            ]
            state["age_ms"] = ages
        positions = state.get("q_sdk")
        complete = (
            isinstance(positions, (list, tuple))
            and len(positions) == 7
            and all(isinstance(value, (int, float)) and math.isfinite(value) for value in positions)
            and isinstance(ages, list)
            and len(ages) == 7
            and all(value is not None and value <= self.freshness_limit_s * 1000 for value in ages)
        )
        state["telemetry_fresh"] = bool(
            complete
            and age <= self.freshness_limit_s
            and state.get("mode") in ("inspect", "follow", "monitor", "experiment", "run")
            and state.get("status") in ("active", "inspection_active", "leader_gap")
            and state.get("coherent_feedback", True)
        )
        state["connection"] = None if self.connection is None else self.connection.status()
        if getattr(self.profile, "engineering", False):
            from .startup import current_alignment_reference, relative_alignment_reference

            state["current_alignment"] = (
                current_alignment_reference(self.profile, positions)
                if state["telemetry_fresh"]
                else dict(available=False, reason="fresh_feedback_required", display_only=True)
            )
            state["relative_alignment"] = (
                relative_alignment_reference(self.profile, positions)
                if state["telemetry_fresh"]
                else dict(available=False, reason="fresh_feedback_required", display_only=True)
            )
        return state

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=1)
