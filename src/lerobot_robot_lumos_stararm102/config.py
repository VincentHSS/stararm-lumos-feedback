from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.robots import RobotConfig
from lerobot.teleoperators import TeleoperatorConfig

from stararm_lumos_feedback.pairing import LEADER_STARTUP_DEFAULTS, ROBOT_STARTUP_DEFAULTS


@RobotConfig.register_subclass("lumos")
@dataclass
class LumosConfig(RobotConfig):
    can_interface: str = ROBOT_STARTUP_DEFAULTS["can_interface"]
    sdk_root: str = ""
    mapping_file: str = ""
    gripper: bool = True
    # This prevents motion commands, NOT motor enabling during SDK construction.
    execute: bool = False
    startup_tolerance_rad: float = ROBOT_STARTUP_DEFAULTS["startup_tolerance_rad"]
    startup_joint2_tolerance_rad: float = ROBOT_STARTUP_DEFAULTS["startup_joint2_tolerance_rad"]
    # None retains each axis's original startup tolerance for the arm pair.
    startup_pair_tolerance_rad: float | None = ROBOT_STARTUP_DEFAULTS["startup_pair_tolerance_rad"]
    startup_wait_s: float = 2.0
    max_speed_deg_per_sec: float = 50.0
    max_target_step_deg: float = 2.0
    # Optional in preview; mandatory for execution. Bounds use SDK joint radians.
    joint_limits_file: str = ""
    cameras: dict[str, CameraConfig] = field(default_factory=dict)


@TeleoperatorConfig.register_subclass("stararm102")
@dataclass
class StarArm102Config(TeleoperatorConfig):
    port: str = LEADER_STARTUP_DEFAULTS["port"]
    mapping_file: str = ""
    gripper: bool = True
    invert_gripper: bool = False
    button_enabled: bool = True
    button_id: int = 7
    joint_map: list[int] = field(default_factory=lambda: list(LEADER_STARTUP_DEFAULTS["joint_map"]))
    signs: list[float] = field(default_factory=lambda: list(LEADER_STARTUP_DEFAULTS["signs"]))
    gain: float = LEADER_STARTUP_DEFAULTS["gain"]
    filter_min_cutoff_hz: float = 5.0
    filter_beta: float = 0.15
