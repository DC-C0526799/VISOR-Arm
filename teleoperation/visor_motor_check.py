#!/usr/bin/env python3
"""visor_motor_check.py - read status registers of all SO-101 motors and test-move the elbow.
Usage: python visor_motor_check.py --port COM5 [--id visor_leader_as_follower] [--joint elbow_flex]
"""
import argparse
import importlib
import logging
import time

logging.getLogger("lerobot.robots.utils").setLevel(logging.ERROR)

REGS = ["Torque_Enable", "Present_Position", "Goal_Position", "Min_Position_Limit",
        "Max_Position_Limit", "Present_Load", "Present_Voltage", "Present_Temperature",
        "Status", "Max_Torque_Limit", "Torque_Limit", "Protection_Current", "Overload_Torque"]


def load_robot(port, rid):
    for name in ("lerobot.robots.so101_follower", "lerobot.robots.so_follower", "lerobot.robots"):
        try:
            m = importlib.import_module(name)
            return m.SO101Follower(m.SO101FollowerConfig(port=port, id=rid))
        except (ImportError, AttributeError):
            continue
    raise SystemExit("Could not import SO101Follower")


def rd(bus, reg, motor):
    try:
        return bus.read(reg, motor, normalize=False)
    except TypeError:
        try:
            return bus.read(reg, motor)
        except Exception as e:
            return f"ERR {type(e).__name__}"
    except Exception as e:
        return f"ERR {type(e).__name__}: {str(e)[:60]}"


def dump(bus, title):
    print(f"\n===== {title} =====")
    motors = list(bus.motors)
    print(f"{'register':<22}" + "".join(f"{m[:12]:>14}" for m in motors))
    for reg in REGS:
        print(f"{reg:<22}" + "".join(f"{str(rd(bus, reg, m)):>14}" for m in motors))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", default="COM5")
    p.add_argument("--id", default="visor_leader_as_follower")
    p.add_argument("--joint", default="elbow_flex")
    a = p.parse_args()

    robot = load_robot(a.port, a.id)
    robot.connect(calibrate=False)
    bus = robot.bus
    j = a.joint
    try:
        dump(bus, "STATUS AFTER CONNECT")

        start = rd(bus, "Present_Position", j)
        print(f"\n{j} raw position now: {start}")
        if isinstance(start, (int, float)):
            for delta in (-150, +150):
                goal = int(start + delta)
                print(f"\nCommanding {j} raw goal {goal} (delta {delta}) ...")
                bus.write("Goal_Position", j, goal, normalize=False)
                time.sleep(1.5)
                pos = rd(bus, "Present_Position", j)
                load = rd(bus, "Present_Load", j)
                st = rd(bus, "Status", j)
                moved = isinstance(pos, (int, float)) and abs(pos - start) > 20
                print(f"  -> position {pos}, load {load}, status {st}, MOVED={moved}")
                bus.write("Goal_Position", j, int(start), normalize=False)
                time.sleep(1.5)

        dump(bus, "STATUS AFTER TEST")
    finally:
        robot.disconnect()
        print("\nDone, torque off.")


if __name__ == "__main__":
    main()
