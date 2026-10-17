"""SO-100 follower: wrist_roll -30 degrees, hold for webcam check, return.

Run from ~/dev/lerobot with PYTHONPATH=~/dev:
    PYTHONPATH=~/dev python roll_move.py

Safety: max_relative_target=35 clamps every command to 35 units from the
measured present position. Only wrist_roll changes; all other joints are
commanded to their measured present values.
"""

import time

import numpy as np
import torch

from lerobot.common.robot_devices.robots.configs import So100RobotConfig
from lerobot.common.robot_devices.robots.utils import make_robot_from_config

MOTOR_ORDER = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
ROLL_IDX = 4
DELTA = -30.0
HOLD_SECONDS = 8


def fmt(vals) -> str:
    return str({n: round(float(v), 2) for n, v in zip(MOTOR_ORDER, vals)})


def main() -> None:
    cfg = So100RobotConfig()
    cfg.leader_arms = {}
    cfg.cameras = {}
    cfg.max_relative_target = 35

    robot = make_robot_from_config(cfg)
    robot.connect()
    try:
        present = np.array(robot.follower_arms["main"].read("Present_Position"), dtype=np.float32)
        print("Present:", fmt(present))

        goal = present.copy()
        goal[ROLL_IDX] += DELTA
        sent = robot.send_action(torch.tensor(goal))
        print("Sent (after clamp):", [round(float(v), 2) for v in sent])

        print(f"Holding displaced pose for {HOLD_SECONDS}s (webcam window)...")
        time.sleep(HOLD_SECONDS)

        after = robot.follower_arms["main"].read("Present_Position")
        print("After move:", fmt(after))

        robot.send_action(torch.tensor(present))
        time.sleep(1.0)
        final = robot.follower_arms["main"].read("Present_Position")
        print("Final (returned):", fmt(final))
    finally:
        robot.disconnect()
        print("Disconnected.")


if __name__ == "__main__":
    main()
