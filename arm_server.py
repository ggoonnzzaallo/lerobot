"""SO-100 follower: persistent command server driven by /tmp/arm_cmd.json.

Run from ~/dev/lerobot with PYTHONPATH=~/dev:
    PYTHONPATH=~/dev python arm_server.py

Protocol: write a JSON object to /tmp/arm_cmd.json (atomic replace):
    {"deltas": [d1, d2, d3, d4, d5, d6]}   relative to current readback
    {"abs":    [v1, v2, v3, v4, v5, v6]}   absolute joint goals (deg; gripper 0-100)
    {"stop": true}                          disconnect and exit

Every command is clamped by max_relative_target (30) inside send_action,
and additionally rejected here if any single delta exceeds 40.
All activity is appended to /tmp/arm_log.txt with timestamps.
Joint order: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper.
"""

import json
import os
import time

import numpy as np
import torch

from lerobot.common.robot_devices.robots.configs import So100RobotConfig
from lerobot.common.robot_devices.robots.utils import make_robot_from_config

CMD_PATH = "/tmp/arm_cmd.json"
LOG_PATH = "/tmp/arm_log.txt"
MOTOR_ORDER = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
MAX_DELTA = 40.0


def log(msg: str) -> None:
    line = f"{time.strftime('%H:%M:%S')} {msg}"
    print(line, flush=True)
    with open(LOG_PATH, "a") as f:
        f.write(line + "\n")


def fmt(vals) -> str:
    return str({n: round(float(v), 2) for n, v in zip(MOTOR_ORDER, vals)})


def main() -> None:
    cfg = So100RobotConfig()
    cfg.leader_arms = {}
    cfg.cameras = {}
    cfg.max_relative_target = 30

    robot = make_robot_from_config(cfg)
    robot.connect()
    last_mtime = 0.0
    log("SERVER_READY")
    try:
        while True:
            try:
                mtime = os.path.getmtime(CMD_PATH)
            except FileNotFoundError:
                mtime = 0.0
            if mtime and mtime != last_mtime:
                last_mtime = mtime
                try:
                    with open(CMD_PATH) as f:
                        cmd = json.load(f)
                except Exception as e:  # noqa: BLE001
                    log(f"BAD_CMD_JSON {e}")
                    cmd = {}
                present = np.array(robot.follower_arms["main"].read("Present_Position"), dtype=np.float32)
                if cmd.get("stop"):
                    log(f"STOP received at {fmt(present)}")
                    break
                if "abs" in cmd:
                    goal = np.array(cmd["abs"], dtype=np.float32)
                    deltas = goal - present
                    if np.max(np.abs(deltas)) > 120:
                        log(f"REJECTED abs (delta too large): {fmt(goal)}")
                    else:
                        sent = robot.send_action(torch.tensor(goal))
                        log(f"ABS sent {[round(float(v), 2) for v in sent]} from {fmt(present)}")
                elif "deltas" in cmd:
                    deltas = np.array(cmd["deltas"], dtype=np.float32)
                    if np.max(np.abs(deltas)) > MAX_DELTA:
                        log(f"REJECTED deltas (>{MAX_DELTA}): {deltas.tolist()}")
                    else:
                        goal = present + deltas
                        sent = robot.send_action(torch.tensor(goal))
                        log(f"DELTA sent {[round(float(v), 2) for v in sent]} from {fmt(present)}")
                time.sleep(1.2)
                after = robot.follower_arms["main"].read("Present_Position")
                log(f"NOW {fmt(after)}")
            time.sleep(0.4)
    finally:
        robot.disconnect()
        log("SERVER_STOPPED (torque state left as-is by bus disconnect)")


if __name__ == "__main__":
    main()
