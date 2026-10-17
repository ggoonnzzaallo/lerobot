"""SO-100 follower: read-only connect test + ONE clamped micro-motion.

Run from ~/dev/lerobot so the relative calibration_dir resolves:
    conda activate lerobot
    cd ~/dev/lerobot
    python move_test.py            # read-only: connect, print joint positions, disconnect
    python move_test.py --move     # additionally: wrist_roll +5 degrees (clamped), then back

Safety: So100RobotConfig.max_relative_target=5 makes send_action() clamp every
command to at most 5 units from the measured present position
(common/robot_devices/robots/manipulator.py: ensure_safe_goal_position).
"""

import sys

import numpy as np
import torch

from lerobot.common.robot_devices.robots.configs import So100RobotConfig
from lerobot.common.robot_devices.robots.utils import make_robot_from_config

MOTOR_ORDER = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


def main() -> None:
    do_move = "--move" in sys.argv

    cfg = So100RobotConfig()
    cfg.leader_arms = {}  # follower only: leader is not plugged in
    cfg.cameras = {}  # no LeRobot cameras; Muse watches via the Mac webcam
    cfg.max_relative_target = 5  # hard clamp: max 5 deg (or 5 gripper units) per command

    robot = make_robot_from_config(cfg)
    robot.connect()  # loads .cache/calibration/so100/main_follower.json, enables follower torque
    try:
        present = robot.follower_arms["main"].read("Present_Position")
        print("Present_Position (deg; gripper is 0-100 linear):")
        for name, val in zip(MOTOR_ORDER, present):
            print(f"  {name:14s} {float(val):8.2f}")

        if do_move:
            goal = np.array(present, dtype=np.float32)
            goal[4] += 5.0  # wrist_roll +5 degrees: visible, no table contact, no load change
            sent = robot.send_action(torch.tensor(goal))
            print("Sent (after clamp):", [round(float(v), 2) for v in sent])

            import time

            time.sleep(1.0)
            after = robot.follower_arms["main"].read("Present_Position")
            print("After:", {n: round(float(v), 2) for n, v in zip(MOTOR_ORDER, after)})

            # Return to the original pose
            robot.send_action(torch.tensor(np.array(present, dtype=np.float32)))
            print("Returned to original pose.")
    finally:
        robot.disconnect()
        print("Disconnected.")


if __name__ == "__main__":
    main()
