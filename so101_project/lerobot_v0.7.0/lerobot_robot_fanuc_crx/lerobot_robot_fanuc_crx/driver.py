"""Low-level CRX connection layer, kept separate from the LeRobot Robot class.

FanucCRX (fanuc_crx.py) talks only to the CRXDriver interface below. To connect real hardware, implement
ExistingSystemCRXDriver on top of the connection system you already use; nothing else needs to change.
"""

import abc
import math
import threading
import time
from dataclasses import dataclass

from .config_fanuc_crx import JOINT_NAMES, FanucCRXConfig


@dataclass
class CRXStatus:
    """Controller state the adapter must check before commanding motion."""

    ready: bool  # remote control enabled, servo on, no alarm, not e-stopped
    message: str = ""


class CRXDriver(abc.ABC):
    """What the LeRobot adapter needs from a CRX connection. All angles are in degrees."""

    @abc.abstractmethod
    def connect(self) -> None: ...

    @abc.abstractmethod
    def disconnect(self) -> None: ...

    @property
    @abc.abstractmethod
    def is_connected(self) -> bool: ...

    @abc.abstractmethod
    def status(self) -> CRXStatus: ...

    @abc.abstractmethod
    def read_joint_positions(self) -> list[float]:
        """Measured joint angles J1..J6."""

    @abc.abstractmethod
    def send_joint_targets(self, targets: list[float]) -> None:
        """Set the joint target J1..J6. Must not block for the motion; the newest target wins."""

    @abc.abstractmethod
    def hold(self) -> None:
        """Stop and keep the current pose (used when hold_mode == "command")."""

    # Gripper is optional; override both when use_gripper is enabled.
    def read_gripper(self) -> float:
        raise NotImplementedError("This driver has no gripper")

    def send_gripper(self, value: float) -> None:
        raise NotImplementedError("This driver has no gripper")


class MockCRXDriver(CRXDriver):
    """Simulated arm: each joint moves toward its target at a limited speed. For tests without hardware."""

    def __init__(self, max_speed_deg_s: float = 30.0, initial: list[float] | None = None):
        self._max_speed = max_speed_deg_s
        self._positions = list(initial) if initial is not None else [0.0] * len(JOINT_NAMES)
        self._targets = list(self._positions)
        self._gripper = 0.0
        self._last_update = time.monotonic()
        self._connected = False
        self._lock = threading.Lock()
        self.hold_calls = 0
        self.fault: str | None = None  # set in tests to simulate an alarm / e-stop

    def _advance(self) -> None:
        now = time.monotonic()
        max_delta = self._max_speed * (now - self._last_update)
        self._last_update = now
        for i, (pos, tgt) in enumerate(zip(self._positions, self._targets, strict=True)):
            self._positions[i] = pos + max(-max_delta, min(max_delta, tgt - pos))

    def connect(self) -> None:
        self._connected = True
        self._last_update = time.monotonic()

    def disconnect(self) -> None:
        self._connected = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    def status(self) -> CRXStatus:
        return CRXStatus(ready=self.fault is None, message=self.fault or "mock ready")

    def read_joint_positions(self) -> list[float]:
        with self._lock:
            self._advance()
            return list(self._positions)

    def send_joint_targets(self, targets: list[float]) -> None:
        with self._lock:
            self._advance()
            self._targets = list(targets)

    def hold(self) -> None:
        with self._lock:
            self._advance()
            self._targets = list(self._positions)
            self.hold_calls += 1

    def read_gripper(self) -> float:
        return self._gripper

    def send_gripper(self, value: float) -> None:
        self._gripper = value


class ExistingSystemCRXDriver(CRXDriver):
    """TODO: bridge to the connection system you already use for the CRX.

    Before implementing, confirm that your system can do the following:
      - read J1..J6 joint angles at the control rate (10-30 Hz; ideally with timestamps)
      - accept streamed joint targets that are not queued as separate stop-and-go moves (latest target wins)
      - stop/hold on request, and report alarm / e-stop / remote-mode state
    Possible routes, depending on the controller options you have (check what is installed):
      FANUC Stream Motion, RMI, an existing PLC/EtherNet/IP bridge, or a vendor PC SDK.
    Keep DCS, speed and collision limits active on the controller; this class must not bypass them.
    """

    def __init__(self, config: FanucCRXConfig):
        self.config = config
        self._connected = False

    def connect(self) -> None:
        raise NotImplementedError("Connect to the CRX through your existing system (host/port in the config)")

    def disconnect(self) -> None:
        self._connected = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    def status(self) -> CRXStatus:
        raise NotImplementedError("Report remote-mode / servo / alarm / e-stop state")

    def read_joint_positions(self) -> list[float]:
        raise NotImplementedError("Read J1..J6 in degrees")

    def send_joint_targets(self, targets: list[float]) -> None:
        raise NotImplementedError("Stream the J1..J6 target in degrees (non-blocking, latest wins)")

    def hold(self) -> None:
        raise NotImplementedError("Stop and hold the current pose on the controller")


def make_driver(config: FanucCRXConfig) -> CRXDriver:
    if config.driver == "mock":
        return MockCRXDriver(max_speed_deg_s=config.mock_max_speed_deg_s)
    return ExistingSystemCRXDriver(config)


def all_finite(values: list[float]) -> bool:
    return all(math.isfinite(v) for v in values)
