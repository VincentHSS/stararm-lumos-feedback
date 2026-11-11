"""Explicit motion setup, with read-only pose checks before SDK construction.

Importing this module does not import hardware packages. The caller owns the CAN
and serial resource locks throughout preflight, motion and cleanup.
"""

import time

CURRENT_STABILITY_S = 0.5
CURRENT_STABILITY_LSB = 4
CURRENT_STRATEGIES = ("current_aligned", "current_relative", "leader_target")


class ConnectionCancelled(Exception):
    pass


def check_cancelled(cancel):
    if cancel.is_set():
        raise ConnectionCancelled("Connection start cancelled")


def preflight_session_pose(profile, cancel, leader_transport, *, robot=None):
    from .runtime import SessionCancelled

    try:
        strategy = getattr(robot, "startup_strategy", "home")
        if strategy == "home":
            result = preflight_pose(profile, cancel, leader_transport=leader_transport)
        else:
            result = preflight_pose(profile, cancel, leader_transport=leader_transport, strategy=strategy)
        if robot is not None:
            robot.startup_reference = result
        return result
    except ConnectionCancelled:
        # Session treats explicit cancellation as normal stop, without briefly
        # publishing it as a hardware error during its cleanup path.
        raise SessionCancelled from None


def preflight_pose(profile, cancel, leader_transport=None, *, strategy="home"):
    _validate_strategy(profile, strategy)
    import can
    import serial

    from .inspection import Inspection
    from .startup import StartupPoseError, check_startup_pose
    from .transport import Fashionstar

    check_cancelled(cancel)
    observer = Inspection(profile)
    bus = can.Bus(
        interface="socketcan",
        channel=profile.session["robot"].get("can_interface", "can0"),
        receive_own_messages=False,
        local_loopback=False,
    )
    observer.bus = bus
    try:
        deadline = time.monotonic() + profile.guards["startup_feedback_s"]
        next_refresh = 0.0
        stable_since = None
        position_min = position_max = None
        previous_stamps = None
        while True:
            check_cancelled(cancel)
            now = time.monotonic()
            if now >= deadline:
                raise RuntimeError("Startup pose check: complete fresh CAN feedback not received")
            if now >= next_refresh:
                observer.refresh()
                next_refresh = now + observer.period
            message = bus.recv(max(0, min(0.02, next_refresh - time.monotonic(), deadline - now)))
            if message is not None:
                observer.ingest(message)
            if not observer.snapshot(waiting=True):
                # Incomplete refresh rounds are expected; stale axes break the
                # stability interval rather than contributing cached coverage.
                if any(
                    s is None or time.monotonic() - s["stamp"] > a["max_age_s"]
                    for s, a in zip(observer.samples, profile.axes)
                ):
                    stable_since = None
                continue
            if strategy == "home":
                break
            stamps = [s["stamp"] for s in observer.samples]
            if previous_stamps is not None and not all(a > b for a, b in zip(stamps, previous_stamps)):
                continue
            previous_stamps = stamps
            positions = [s["motor_position"] for s in observer.samples]
            quiet = all(
                abs(s["motor_velocity"]) <= CURRENT_STABILITY_LSB * 2 * a["v_max"] / 4095
                for s, a in zip(observer.samples, profile.axes)
            )
            if stable_since is None or not quiet:
                stable_since = min(stamps) if quiet else None
                position_min = positions.copy()
                position_max = positions.copy()
            else:
                position_min = [min(a, b) for a, b in zip(position_min, positions)]
                position_max = [max(a, b) for a, b in zip(position_max, positions)]
                if any(
                    hi - lo > CURRENT_STABILITY_LSB * 2 * a["p_max"] / 65535
                    for lo, hi, a in zip(position_min, position_max, profile.axes)
                ):
                    stable_since = min(stamps)
                    position_min = positions.copy()
                    position_max = positions.copy()
            if stable_since is not None and min(stamps) - stable_since >= CURRENT_STABILITY_S:
                break
        check_cancelled(cancel)
        if leader_transport is not None:
            leader = leader_transport.query(tuple(range(7)))
        else:
            port = profile.session["teleop"].get("port", "/dev/ttyUSB0")
            with serial.Serial(
                port=port, baudrate=1_000_000, parity=serial.PARITY_NONE,
                stopbits=1, bytesize=8, timeout=0.002, exclusive=True,
            ) as uart:
                transport = Fashionstar(uart, range(7), enabled=False, timeout=profile.guards["leader_query_s"])
                leader = transport.query(tuple(range(7)))
        check_cancelled(cancel)
        query_finished = time.monotonic()
        if strategy in CURRENT_STRATEGIES:
            # The serial query cannot authorize a pose from before that query.
            # Request and assemble a complete genuinely new CAN group afterward.
            observer.refresh()
            next_refresh = time.monotonic() + observer.period
            aligned_deadline = min(
                deadline,
                query_finished + min(
                    min(axis["max_age_s"] for axis in profile.axes),
                    profile.guards["leader_query_s"] + profile.guards["can_skew_s"],
                ),
            )
            while True:
                check_cancelled(cancel)
                now = time.monotonic()
                if now >= aligned_deadline:
                    raise RuntimeError("Startup pose check: new CAN feedback after Leader query not received")
                if now >= next_refresh:
                    observer.refresh()
                    next_refresh = now + observer.period
                message = bus.recv(max(0, min(0.02, aligned_deadline - now)))
                if message is not None:
                    observer.ingest(message)
                if observer.snapshot(waiting=True) and all(
                    s["stamp"] > query_finished for s in observer.samples
                ):
                    break
            if any(
                max(hi, s["motor_position"]) - min(lo, s["motor_position"])
                > CURRENT_STABILITY_LSB * 2 * a["p_max"] / 65535
                or abs(s["motor_velocity"]) > CURRENT_STABILITY_LSB * 2 * a["v_max"] / 4095
                for s, lo, hi, a in zip(
                    observer.samples,
                    position_min,
                    position_max,
                    profile.axes,
                )
            ):
                raise RuntimeError("Startup pose moved during Leader alignment query")
        # A delayed serial query cannot authorize enabling from old CAN data.
        observer.snapshot(waiting=False)
        measured_q = [sample["q"] for sample in observer.samples]
        try:
            result = check_startup_pose(
                profile, measured_q,
                {sid: sample["angle"] for sid, sample in leader.items()},
                strategy=strategy,
            )
        except StartupPoseError as exc:
            # The complete CAN snapshot passed its own guards. Retain these
            # measurements for display after rejection, without making them
            # active telemetry or an accepted startup reference.
            exc.display_pose = dict(
                q_sdk=measured_q,
                q_urdf=[profile.to_model(i, q) for i, q in enumerate(measured_q[:6])],
                captured_monotonic=min(sample["stamp"] for sample in observer.samples),
            )
            raise
        check_cancelled(cancel)
        captured = time.monotonic()
        return dict(
            result,
            captured_monotonic=captured,
            valid_until=captured + profile.guards["startup_feedback_s"],
            frame_stamps=[sample["stamp"] for sample in observer.samples],
            actual_q=[sample["q"] for sample in observer.samples],
            sdk_tolerance_rad=[axis["sdk_tolerance"] for axis in profile.axes[:6]],
            sdk_ready_budget_s=min(0.1, *(axis["max_age_s"] for axis in profile.axes)),
        )
    finally:
        bus.shutdown()


def _validate_strategy(profile, strategy, mode=None):
    if type(strategy) is not str or strategy not in ("home", *CURRENT_STRATEGIES):
        raise ValueError("startup_pose must be home, current_aligned, current_relative or leader_target")
    if strategy in CURRENT_STRATEGIES and (
        getattr(profile, "engineering", False) is not True
        or (mode is not None and mode not in ("follow", "monitor", "experiment"))
    ):
        raise ValueError("Current-pose startup requires engineering follow or monitor mode")


def create_session(
    profile, caps, dashboard, cancel, mode, *, button_enabled=None, startup_pose="home",
    button_wait_until_cancel=False,
):
    from .config import verify_sdk_imports

    if type(button_wait_until_cancel) is not bool:
        raise ValueError("button_wait_until_cancel must be a boolean")
    if button_wait_until_cancel and not all(
        callable(getattr(cancel, method, None)) for method in ("is_set", "wait")
    ):
        raise ValueError("button_wait_until_cancel requires a connection cancellation event")
    check_cancelled(cancel)
    _validate_strategy(profile, startup_pose, mode)
    verify_sdk_imports(profile)
    from lerobot_robot_lumos_stararm102.cli import device_configs, load_session, validate_pair
    from lerobot_robot_lumos_stararm102.lumos import Lumos
    from lerobot_robot_lumos_stararm102.stararm102 import StarArm102

    from .runtime import Session

    data = load_session(profile.paths["session_file"])
    validate_pair(data)
    robot_config, leader_config = device_configs(data)
    # A reviewed local runtime option, applied only after the original bound
    # session has been loaded and validated. No mapping or profile is rewritten.
    if button_enabled is not None:
        if type(button_enabled) is not bool:
            raise ValueError("button_enabled must be a boolean")
        leader_config.button_enabled = button_enabled
    robot_config.execute = True
    check_cancelled(cancel)
    # These constructors do not connect. SingleArm is constructed only by run().
    session = Session(profile, Lumos(robot_config), StarArm102(leader_config), caps, mode=mode, dashboard=dashboard)
    session.robot.measured_limit_margin_rad = tuple(axis["limit_margin"] for axis in profile.axes[:6])
    session.leader.button_wait_until_cancel = button_wait_until_cancel
    session.robot.startup_strategy = startup_pose
    session.stop = cancel
    return session
