"""Checks for the FANUC CRX adapter using the mock driver (no hardware).

Run: .venv\\Scripts\\python.exe lerobot_robot_fanuc_crx\\tests\\test_fanuc_crx.py
(also works under pytest if installed)
"""

import math
import time

from lerobot.robots.config import RobotConfig
from lerobot.robots.utils import make_robot_from_config
from lerobot.rollout.robot_wrapper import ThreadSafeRobot
from lerobot.utils.import_utils import register_third_party_plugins

JOINTS = [f"joint_{i}.pos" for i in range(1, 7)]


def _make(**overrides):
    register_third_party_plugins()  # discovers lerobot_robot_fanuc_crx like the CLIs do
    config_cls = RobotConfig.get_choice_class("fanuc_crx")
    robot = make_robot_from_config(config_cls(**overrides))
    robot.connect()
    return robot


def _action(values):
    return dict(zip(JOINTS, values, strict=True))


def test_plugin_discovery_and_features():
    robot = _make()
    try:
        assert type(robot).__name__ == "FanucCRX"
        assert list(robot.action_features) == JOINTS
        assert all(key.endswith(".pos") for key in robot.action_features)
        obs = robot.get_observation()
        assert set(JOINTS) <= set(obs)
    finally:
        robot.disconnect()


def test_step_and_joint_limits_clamp():
    robot = _make(max_step_deg=1.0)
    try:
        sent = robot.send_action(_action([50.0, -50.0, 0.5, 0.0, 0.0, 0.0]))
        assert sent["joint_1.pos"] == 1.0 and sent["joint_2.pos"] == -1.0 and sent["joint_3.pos"] == 0.5
        # Joint limit: placeholder limit is +-90 deg; walk toward 120 and expect it to stop at 90.
        for _ in range(200):
            sent = robot.send_action(_action([120.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
        assert sent["joint_1.pos"] == 90.0
    finally:
        robot.disconnect()


def test_rejects_bad_actions():
    robot = _make()
    try:
        for bad in (_action([math.nan, 0, 0, 0, 0, 0]), {"joint_1.pos": 0.0}):
            try:
                robot.send_action(bad)
            except ValueError:
                pass
            else:
                raise AssertionError(f"accepted bad action {bad}")
    finally:
        robot.disconnect()


def test_refuses_motion_when_controller_not_ready():
    robot = _make()
    try:
        robot.driver.fault = "SRVO-001 operator panel e-stop (simulated)"
        try:
            robot.send_action(_action([0.5] * 6))
        except RuntimeError:
            pass
        else:
            raise AssertionError("moved while controller reported a fault")
    finally:
        robot.disconnect()


def test_mock_tracks_targets():
    robot = _make(max_step_deg=5.0, mock_max_speed_deg_s=100.0)
    try:
        robot.send_action(_action([5.0, 0, 0, 0, 0, 0]))
        time.sleep(0.2)  # 100 deg/s * 0.2 s >> 5 deg
        assert abs(robot.get_observation()["joint_1.pos"] - 5.0) < 1e-6
    finally:
        robot.disconnect()


def test_position_hold_contract_with_rollout_wrapper():
    """The same check remote rollout runs (require_hold=True -> configure_hold)."""
    robot = _make(hold_mode="position")
    wrapper = ThreadSafeRobot(robot)
    try:
        wrapper.configure_hold()
        assert wrapper.supports_hold and not wrapper.supports_command_hold
        wrapper.get_observation()
        wrapper.hold()  # before any command: holds the measured pose
        wrapper.send_action(_action([1.0, 0, 0, 0, 0, 0]))
        wrapper.hold()  # resends the last applied target
        assert robot._last_targets[0] == 1.0
        assert wrapper.hardware_failure is None
    finally:
        robot.disconnect()


def test_command_hold_contract_with_rollout_wrapper():
    robot = _make(hold_mode="command")
    wrapper = ThreadSafeRobot(robot)
    try:
        wrapper.configure_hold()
        assert wrapper.supports_hold and wrapper.supports_command_hold
        wrapper.hold()
        assert robot.driver.hold_calls == 1
    finally:
        robot.disconnect()


def test_gripper_optional():
    robot = _make(use_gripper=True)
    try:
        assert "gripper.pos" in robot.action_features
        sent = robot.send_action({**_action([0.0] * 6), "gripper.pos": 150.0})
        assert sent["gripper.pos"] == 100.0
    finally:
        robot.disconnect()


if __name__ == "__main__":
    tests = [(name, fn) for name, fn in globals().items() if name.startswith("test_") and callable(fn)]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            print(f"PASS {name}")
        except Exception as exc:  # noqa: BLE001
            failed += 1
            print(f"FAIL {name}: {type(exc).__name__}: {exc}")
    print(f"{len(tests) - failed}/{len(tests)} passed")
    raise SystemExit(1 if failed else 0)
