"""Print the PushT sim robot descriptor (features, action feature, semantics) for the server config."""
from dataclasses import asdict
import json

from lerobot.envs.configs import PushtEnvConfig
from lerobot.robots.sim_robot import SimRobot, SimRobotConfig

robot = SimRobot(SimRobotConfig(env=PushtEnvConfig()))
d = robot.descriptor
print("robot_type:", robot.robot_type)
print("semantics:", d.semantics)
print("features:", json.dumps([asdict(f) for f in d.features], indent=1, default=str))
print("action_feature:", json.dumps(asdict(d.action_feature), indent=1, default=str))
print("action_features:", robot.action_features)
robot.disconnect()
