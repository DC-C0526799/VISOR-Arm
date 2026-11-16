#!/usr/bin/env python3
"""
read_arm_position.py - print the SO-101 follower's current joint positions.

Use it to record poses (HOME, DETECTION, pickup, bin positions):
move the arm to the pose, run this script, and copy the values into
the matching calibration/*.json file.

    python read_arm_position.py
"""

import json
from pathlib import Path

from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.motors.motors_bus import Motor, MotorCalibration, MotorNormMode

PORT = "/dev/ttyACM0"
CAL = (
    Path.home()
    / ".cache/huggingface/lerobot/calibration/robots/so_follower"
    / "my_awesome_follower_arm.json"
)

data = json.loads(CAL.read_text())

motors = {
    name: Motor(
        values["id"],
        "sts3215",
        MotorNormMode.RANGE_0_100 if name == "gripper" else MotorNormMode.DEGREES,
    )
    for name, values in data.items()
}
calibration = {name: MotorCalibration(**values) for name, values in data.items()}

bus = FeetechMotorsBus(port=PORT, motors=motors, calibration=calibration)

try:
    bus.connect()
    positions = bus.sync_read("Present_Position")
    print("\nSO-101 CURRENT MOTOR POSITIONS")
    for name, value in positions.items():
        print(f"{name}: {value:.2f}")
    print("\nJSON:")
    print(json.dumps({k: round(float(v), 2) for k, v in positions.items()}, indent=2))
finally:
    if bus.is_connected:
        bus.disconnect()
