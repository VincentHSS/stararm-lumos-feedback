"""Lumos force-feedback launcher. Offline commands never import hardware SDKs."""

import argparse
import json
import math
import signal
import threading
import time
from pathlib import Path

from . import __version__
from .config import AXIS_KEYS, GRIPPER_KEYS, GUARD_KEYS, POLICY_KEYS, Profile, read_json
from .gripper import TypeLJ04041254


def template():
    axes = [{key: None for key in AXIS_KEYS.split()} for _ in range(7)]
    axes[6].update(sdk_scale=1, sdk_offset=0, urdf_scale=1, urdf_offset=0)
    return dict(
        version=2,
        session_file="session.local.json",
        urdf="startouch_sdk/src/config/FastTouchV2.SLDASM.urdf",
        sdk_config="startouch_sdk/src/config/robot_kinematics.yaml",
        gravity=[0, 0, -9.80665],
        payload=None,
        mass_coverage="UNCONFIRMED",
        evidence="",
        gripper_transform={
            key: TypeLJ04041254.kind if key == "type" else None for key in GRIPPER_KEYS.split()
        },
        axes=axes,
        policy=[{key: None for key in POLICY_KEYS.split()} for _ in range(7)],
        guards={key: 1_000_000 if key == "can_bitrate" else None for key in GUARD_KEYS.split()},
    )


def discover(interface, seconds):
    import can

    # No arm construction, no motor enabling, no send call. Another operator must
    # already have made the bus produce feedback, otherwise no frames are expected.
    bus = can.Bus(interface="socketcan", channel=interface, receive_own_messages=False)
    frames = {}
    try:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            msg = bus.recv(min(0.1, max(0, deadline - time.monotonic())))
            if (
                msg is None
                or msg.is_error_frame
                or msg.is_remote_frame
                or msg.is_extended_id
                or not msg.is_rx
                or len(msg.data) != 8
            ):
                continue
            identity = msg.arbitration_id, msg.data[0] & 15
            entry = frames.setdefault(identity, dict(feedback_id=identity[0], motor_id=identity[1], count=0))
            entry.update(
                count=entry["count"] + 1,
                status=msg.data[0] >> 4,
                last_hex=bytes(msg.data).hex(),
                kernel_timestamp=msg.timestamp,
            )
    finally:
        bus.shutdown()
    print(
        json.dumps(
            dict(
                receive_only=True,
                candidates=list(frames.values()),
                note="Candidates only: verify IDs, motor type, ranges, coordinates and units against hardware",
            ),
            indent=2,
        )
    )


def prepare(source, output):
    """Create an explicit, boundable observation profile without inventing mass coverage."""
    import yaml

    from .commissioning_defaults import generate

    source, output = source.expanduser().resolve(), output.expanduser().resolve()
    data = read_json(source)
    if data.get("version") not in (2, 3):
        raise ValueError(
            "Prepare requires the reviewed v2 hardware layout or an existing v3 commissioning profile"
        )
    for key in ("session_file", "urdf", "sdk_config"):
        data[key] = str((source.parent / Path(data[key]).expanduser()).resolve())
    session = read_json(data["session_file"])
    sdk_config = yaml.safe_load(Path(data["sdk_config"]).read_text(encoding="utf-8"))
    generated = generate(data["axes"], session, sdk_config)
    data.update(generated)
    data["version"] = 3
    # Never write a partly generated file if validation fails, or overwrite local settings.
    if output.exists():
        raise FileExistsError(f"Local commissioning profile already exists: {output}")
    encoded = json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False) + "\n"
    created = False
    try:
        with output.open("x", encoding="utf-8") as stream:
            created = True
            stream.write(encoded)
        Profile(output, purpose="inspect")
    except Exception:
        if created and output.is_file() and output.read_text(encoding="utf-8") == encoded:
            output.unlink()
        raise
    print(f"Commissioning profile: {output}")
    print(
        "Engineering observation values only; force output blocked, mass coverage unchanged, calibration preserved."
    )


def validate_session_config(profile, session_config):
    """Reject a teleop config that would silently select a different setup.

    Normalize paths with the original launcher so an equivalent local copy is
    accepted. Motion still loads the bound session; this never replaces it.
    """
    from lerobot_robot_lumos_stararm102.cli import load_session

    selected = load_session(session_config)
    bound = load_session(profile.paths["session_file"])
    if selected != bound:
        raise ValueError(
            "--config differs from the feedback profile's bound session_file. "
            "Use that session configuration, or review the profile and regenerate its binding; "
            "calibration is not changed automatically"
        )


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--version", action="version", version=f"lumos-feedback {__version__}")
    sub = parser.add_subparsers(dest="command", required=True)
    p = sub.add_parser("template")
    p.add_argument("--output", required=True, type=Path)
    p = sub.add_parser("discover")
    p.add_argument("--can", default="can0")
    p.add_argument("--seconds", type=float, default=5)
    p = sub.add_parser("prepare")
    p.add_argument("--profile", required=True, type=Path)
    p.add_argument("--output", required=True, type=Path)
    for name in ("check", "bind", "serve", "preview", "inspect", "follow", "monitor", "experiment", "run"):
        p = sub.add_parser(name)
        p.add_argument("--profile", required=True, type=Path)
        if name != "check":
            p.add_argument("--binding", required=True, type=Path)
        if name in ("check", "bind"):
            p.add_argument("--purpose", choices=("inspect", "follow", "monitor", "experiment", "run"), default="run")
        if name == "bind":
            p.add_argument(
                "--replace",
                action="store_true",
                help="Replace only this binding; calibration is never changed",
            )
        if name in ("serve", "preview", "inspect", "follow", "monitor", "experiment", "run"):
            p.add_argument("--seconds", type=float)
            p.add_argument("--port", type=int, default=8765)
        if name in ("follow", "monitor", "experiment", "run"):
            p.add_argument(
                "--session-config", type=Path,
                help="Require the original teleop --config to match this profile's bound session",
            )
        if name in ("serve", "inspect", "follow", "monitor", "experiment", "run"):
            p.add_argument(
                "--settings",
                type=Path,
                default=Path.home() / ".config/stararm_lumos_feedback/current.caps.json",
                help="Persist only current power caps to this local file",
            )
            if name != "serve":
                p.add_argument("--no-web", action="store_true")
            else:
                p.add_argument(
                    "--button-settings", type=Path,
                    default=Path.home() / ".config/stararm_lumos_feedback/current.button-board.local.json",
                    help="Persist only the current dashboard button board selection locally",
                )
    args = parser.parse_args(argv)
    # SIGTERM must follow the same cleanup path as Ctrl+C during a real session.
    previous_term = None
    if threading.current_thread() is threading.main_thread():

        def interrupt(signum, frame):
            raise KeyboardInterrupt

        previous_term = signal.signal(signal.SIGTERM, interrupt)
    try:
        if args.command == "template":
            with args.output.open("x", encoding="utf-8") as stream:
                json.dump(template(), stream, indent=2)
            return
        if args.command == "discover":
            if not 0 < args.seconds <= 60:
                raise ValueError("Discovery seconds must be in (0, 60]")
            discover(args.can, args.seconds)
            return
        if args.command == "prepare":
            prepare(args.profile, args.output)
            return
        purpose = args.purpose if args.command in ("check", "bind") else args.command
        if purpose in ("preview", "serve"):
            purpose = "inspect"
        profile = Profile(args.profile, purpose=purpose)
        if args.command == "check":
            print(
                json.dumps(
                    dict(
                        binding_sha256=profile.binding()["sha256"],
                        model=profile.model.summary(),
                        purpose=purpose,
                        model_mass_confirmed=profile.mass_confirmed,
                        engineering_initial=profile.engineering,
                        hardware_connected=False,
                    ),
                    indent=2,
                )
            )
            return
        if args.command == "bind":
            with args.binding.open("w" if args.replace else "x", encoding="utf-8") as stream:
                json.dump(profile.binding(), stream, indent=2)
            print("Local binding written. Existing calibration preserved; hardware has not been validated.")
            return
        profile.verify(args.binding)
        if getattr(args, "session_config", None) is not None:
            validate_session_config(profile, args.session_config)
        if args.command == "serve":
            from .connection import InspectionConnection
            from .control import Caps
            from .dashboard import Dashboard

            if args.seconds is not None and (not math.isfinite(args.seconds) or args.seconds <= 0):
                raise ValueError("duration must be finite and positive")
            dashboard = Dashboard(Caps(), profile, port=args.port, settings=args.settings)
            connection = InspectionConnection(profile, args.binding, dashboard, button_settings=args.button_settings)
            dashboard.connection = connection
            dashboard.start()
            print(
                f"Lumos feedback {__version__}: http://127.0.0.1:{dashboard.server.server_port}/", flush=True
            )
            print(
                "Ready. Select a mode explicitly in the page. Following enables motors and moves the Follower.",
                flush=True,
            )
            started = time.monotonic()
            try:
                while args.seconds is None or time.monotonic() - started < args.seconds:
                    time.sleep(min(0.2, args.seconds) if args.seconds is not None else 0.2)
            finally:
                try:
                    connection.close()
                finally:
                    dashboard.close()
            return
        if args.command == "preview":
            from .control import Caps
            from .dashboard import Dashboard

            if args.seconds is not None and (not math.isfinite(args.seconds) or args.seconds <= 0):
                raise ValueError("duration must be finite and positive")
            dashboard = Dashboard(Caps(), profile, port=args.port)
            dashboard.publish(
                status="preview_ready",
                mode="preview",
                leader_controlled=False,
                leader_output_state="unknown",
                residual_verified=False,
                baseline_eligible=False,
                requested_mw=[None] * 7,
                predicted_mw=[None] * 7,
                hold_requested=[None] * 7,
            )
            dashboard.start()
            print(f"Offline model preview: http://127.0.0.1:{dashboard.server.server_port}/", flush=True)
            print("URDF reference pose only; no SDK, CAN, serial, motor enabling or motion.", flush=True)
            started = time.monotonic()
            try:
                while args.seconds is None or time.monotonic() - started < args.seconds:
                    time.sleep(min(0.2, args.seconds) if args.seconds is not None else 0.2)
            finally:
                dashboard.close()
            return
        if args.command == "inspect":
            from .control import Caps
            from .dashboard import Dashboard
            from .inspection import Inspection
            from .runtime import session_lock, validate_socketcan

            dashboard = None
            with session_lock(
                profile.session["robot"].get("can_interface", "can0"),
                profile.session["teleop"].get("port", "/dev/ttyUSB0"),
            ):
                try:
                    validate_socketcan(
                        profile.session["robot"].get("can_interface", "can0"),
                        profile.guards["can_bitrate"],
                    )
                    if not args.no_web:
                        dashboard = Dashboard(Caps(), profile, port=args.port)
                        dashboard.start()
                        print(f"Dashboard: http://127.0.0.1:{dashboard.server.server_port}/", flush=True)
                    print(
                        "Inspect: read-only CAN state queries; no SDK, enabling, motion targets or Leader writes.",
                        flush=True,
                    )
                    summary = Inspection(profile, dashboard=dashboard).run(args.seconds)
                    print(json.dumps(summary, ensure_ascii=False, allow_nan=False), flush=True)
                finally:
                    if dashboard is not None:
                        dashboard.close()
            return
        from .control import Caps
        from .dashboard import Dashboard
        from .motion import create_session, preflight_session_pose
        from .runtime import session_lock

        caps, dashboard = Caps(), None
        cancel = threading.Event()
        with session_lock(
            profile.session["robot"].get("can_interface", "can0"),
            profile.session["teleop"].get("port", "/dev/ttyUSB0"),
        ):
            try:
                if args.no_web and args.settings is not None:
                    from .config import read_json

                    if args.settings.exists():
                        caps.set(read_json(args.settings))
                # CLI and web use the same adapter construction and SDK gates.
                # Device constructors do not connect; Session.run owns cleanup.
                session = create_session(profile, caps, None, cancel, args.command)
                session.before_enable = lambda transport: preflight_session_pose(
                    profile, cancel, transport, robot=session.robot,
                )
                if not args.no_web:
                    dashboard = Dashboard(caps, profile, port=args.port, settings=args.settings)
                    dashboard.start()
                    session.dashboard = dashboard
                    print(f"Dashboard: http://127.0.0.1:{dashboard.server.server_port}/", flush=True)
                print(
                    f"Mode {args.command}: SDK construction enables motors; Follower motion is active. "
                    f"Leader force output: {'ENABLED' if args.command in ('run', 'experiment') else 'disabled'}.",
                    flush=True,
                )
                session.run(args.seconds)
            finally:
                if dashboard is not None:
                    dashboard.close()
    except KeyboardInterrupt:
        if args.command == "serve":
            print("Dashboard stopped; hardware sessions closed.")
        elif args.command == "preview":
            print("Offline preview stopped; hardware was not accessed.")
        elif args.command == "inspect":
            print("Read-only inspection stopped; Leader was not controlled.")
        else:
            print("Stopped; managed Leader outputs released where communication permits.")
    except (ValueError, OSError, RuntimeError, ImportError, KeyError, TypeError) as exc:
        parser.exit(2, f"Feedback configuration/runtime error: {exc}\n")
    finally:
        if previous_term is not None:
            signal.signal(signal.SIGTERM, previous_term)


if __name__ == "__main__":
    main()
