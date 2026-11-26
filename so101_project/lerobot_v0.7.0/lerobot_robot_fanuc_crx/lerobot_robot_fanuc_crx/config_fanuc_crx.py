"""Configuration for the FANUC CRX LeRobot adapter."""

from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.robots.config import RobotConfig

JOINT_NAMES: tuple[str, ...] = ("joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6")


def _placeholder_limits() -> dict[str, list[float]]:
    # PLACEHOLDER software limits in degrees, deliberately narrow. They are NOT the CRX's real ranges.
    # Set them from your robot's spec and keep them inside the controller's DCS limits.
    return {name: [-90.0, 90.0] for name in JOINT_NAMES}


@RobotConfig.register_subclass("fanuc_crx")
@dataclass
class FanucCRXConfig(RobotConfig):
    """FANUC CRX driven by joint position targets (degrees).

    The LeRobot side only sends joint targets and reads joint positions. Motion safety (DCS, speed limits,
    collision detection, e-stop) stays in the robot controller; the checks here are a second layer.
    """

    # "mock": simulated arm for testing without hardware.
    # "existing": your existing connection system (implement ExistingSystemCRXDriver first).
    driver: str = "mock"
    # Address of the existing connection system (robot controller, PLC or bridge PC).
    host: str = "127.0.0.1"
    port: int = 0

    # Software joint limits [min, max] in degrees; targets are clamped into them.
    joint_limits_deg: dict[str, list[float]] = field(default_factory=_placeholder_limits)
    # Largest change from the current commanded position allowed per send_action call, in degrees.
    # At 10 Hz, 1.0 deg/step caps joint speed near 10 deg/s. Set to 0 or a negative value to disable.
    max_step_deg: float = 1.0
    # Mock driver only: joint speed limit used to simulate tracking, in degrees per second.
    mock_max_speed_deg_s: float = 30.0

    # Hold behaviour while remote inference waits or stops:
    # "position": LeRobot resends the last applied joint targets (supports_position_hold).
    # "command": call the driver's own hold()/stop (supports_command_hold), e.g. a controller-side stop.
    hold_mode: str = "position"

    # Optional gripper as "gripper.pos" in [0, 100] (0 = open, 100 = closed). Off by default.
    use_gripper: bool = False

    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    def __post_init__(self):
        super().__post_init__()
        if self.driver not in ("mock", "existing"):
            raise ValueError(f"driver must be 'mock' or 'existing', got {self.driver!r}")
        if self.hold_mode not in ("position", "command"):
            raise ValueError(f"hold_mode must be 'position' or 'command', got {self.hold_mode!r}")
        missing = set(JOINT_NAMES) - set(self.joint_limits_deg)
        if missing:
            raise ValueError(f"joint_limits_deg is missing {sorted(missing)}")
        for name, limits in self.joint_limits_deg.items():
            if len(limits) != 2 or not limits[0] < limits[1]:
                raise ValueError(f"joint_limits_deg[{name!r}] must be [min, max] with min < max, got {limits}")
