"""FANUC CRX as a LeRobot Robot (joint position control).

Contract with LeRobot v0.7.0 remote inference (lerobot/rollout/robot_wrapper.py):
  - Every action key ends in ".pos" and is a position target. That is what position hold requires.
  - send_action returns the targets actually sent (after clamping). The rollout wrapper stores them and,
    on hold, sends them again through send_action.
  - get_observation includes every "<joint>.pos" key, so a hold before the first command can use the
    measured pose.
"""

import logging
import math
from functools import cached_property

from lerobot.cameras import make_cameras_from_configs
from lerobot.lerobot_types import RobotAction, RobotObservation
from lerobot.robots.robot import Robot
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from .config_fanuc_crx import JOINT_NAMES, FanucCRXConfig
from .driver import CRXDriver, all_finite, make_driver

logger = logging.getLogger(__name__)

GRIPPER_KEY = "gripper.pos"


class FanucCRX(Robot):
    config_class = FanucCRXConfig
    name = "fanuc_crx"

    def __init__(self, config: FanucCRXConfig, driver: CRXDriver | None = None):
        super().__init__(config)
        self.config = config
        # Read by lerobot's rollout wrapper; exactly one of these is enabled.
        self.supports_position_hold = config.hold_mode == "position"
        self.supports_command_hold = config.hold_mode == "command"
        self.driver = driver if driver is not None else make_driver(config)
        self.cameras = make_cameras_from_configs(config.cameras)
        self._last_targets: list[float] | None = None

    # -- features ------------------------------------------------------------------------------

    @property
    def _joint_keys(self) -> list[str]:
        return [f"{name}.pos" for name in JOINT_NAMES]

    @cached_property
    def action_features(self) -> dict[str, type]:
        keys = self._joint_keys + ([GRIPPER_KEY] if self.config.use_gripper else [])
        return dict.fromkeys(keys, float)

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        cams = {key: (cfg.height, cfg.width, 3) for key, cfg in self.config.cameras.items()}
        return {**self.action_features, **cams}

    # -- lifecycle -----------------------------------------------------------------------------

    @property
    def is_connected(self) -> bool:
        return self.driver.is_connected and all(cam.is_connected for cam in self.cameras.values())

    @property
    def is_calibrated(self) -> bool:
        return True  # joint zero is owned by the controller (mastering); nothing to calibrate here

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        self.driver.connect()
        try:
            status = self.driver.status()
            if not status.ready:
                raise ConnectionError(f"CRX is not ready for remote motion: {status.message}")
            for cam in self.cameras.values():
                cam.connect()
            # Start from the measured pose so the first step limit is relative to where the arm is.
            self._last_targets = self._read_joints()
        except BaseException:
            self.disconnect()
            raise
        logger.info("%s connected (driver=%s, hold=%s)", self, self.config.driver, self.config.hold_mode)

    def disconnect(self) -> None:
        # Not decorated: also cleans up after a failed connect. The controller keeps its own brake/servo policy.
        for cam in self.cameras.values():
            if cam.is_connected:
                cam.disconnect()
        if self.driver.is_connected:
            self.driver.disconnect()

    # -- I/O -------------------------------------------------------------------------------------

    def _read_joints(self) -> list[float]:
        joints = self.driver.read_joint_positions()
        if len(joints) != len(JOINT_NAMES) or not all_finite(joints):
            raise RuntimeError(f"Invalid joint reading from CRX driver: {joints}")
        return joints

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        obs: RobotObservation = dict(zip(self._joint_keys, self._read_joints(), strict=True))
        if self.config.use_gripper:
            obs[GRIPPER_KEY] = float(self.driver.read_gripper())
        for key, cam in self.cameras.items():
            obs[key] = cam.async_read()
        return obs

    def _safe_targets(self, requested: list[float]) -> list[float]:
        """Clamp to the software joint limits and the per-step limit; reject NaN/inf."""
        if not all_finite(requested):
            raise ValueError(f"Non-finite joint target rejected: {requested}")
        reference = self._last_targets if self._last_targets is not None else self._read_joints()
        step = self.config.max_step_deg
        safe = []
        for name, target, ref in zip(JOINT_NAMES, requested, reference, strict=True):
            lo, hi = self.config.joint_limits_deg[name]
            value = min(max(target, lo), hi)
            if step > 0:
                value = min(max(value, ref - step), ref + step)
            safe.append(value)
        if any(not math.isclose(a, b, abs_tol=1e-9) for a, b in zip(safe, requested, strict=True)):
            logger.debug("CRX target clamped: requested=%s sent=%s", requested, safe)
        return safe

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        if set(action) != set(self.action_features):
            raise ValueError(
                f"fanuc_crx expects exactly {list(self.action_features)}; "
                f"missing {sorted(set(self.action_features) - set(action))}, "
                f"unknown {sorted(set(action) - set(self.action_features))}"
            )
        status = self.driver.status()
        if not status.ready:
            raise RuntimeError(f"CRX not ready, refusing motion: {status.message}")

        targets = self._safe_targets([float(action[key]) for key in self._joint_keys])
        self.driver.send_joint_targets(targets)
        self._last_targets = targets
        applied: RobotAction = dict(zip(self._joint_keys, targets, strict=True))

        if self.config.use_gripper:
            grip = float(action[GRIPPER_KEY])
            if not math.isfinite(grip):
                raise ValueError(f"Non-finite gripper target rejected: {grip}")
            grip = min(max(grip, 0.0), 100.0)
            self.driver.send_gripper(grip)
            applied[GRIPPER_KEY] = grip
        return applied

    def hold(self) -> None:
        """Command hold (hold_mode == "command"): stop on the controller and keep the current pose."""
        if not self.supports_command_hold:
            raise NotImplementedError("fanuc_crx uses position hold; set hold_mode=command to use hold()")
        self.driver.hold()
        self._last_targets = self._read_joints()
