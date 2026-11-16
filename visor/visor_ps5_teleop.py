#!/usr/bin/env python3
"""
visor_ps5_teleop.py  -  VISOR capstone demo
Drive an SO-101 arm (here: the LEADER arm re-used as a follower) with a PS5 DualSense.
Tested target: LeRobot 0.6.x on Raspberry Pi 5 / Linux, pygame 2.x.

CONTROLS (DualSense)
  Left stick  X   -> shoulder_pan   (base rotate)
  Left stick  Y   -> shoulder_lift
  Right stick Y   -> elbow_flex
  Right stick X   -> wrist_roll
  R1 / L1         -> wrist_flex up / down
  R2 / L2         -> gripper open / close (analog, press harder = faster)
  Square          -> toggle SLOW / FAST speed
  Triangle        -> smoothly return to the start pose
  Options         -> return to start pose, then exit (torque off)
  If the controller disconnects, the arm freezes in place until it reconnects.

USAGE
  python visor_ps5_teleop.py --test-pad                      # check controller only, no robot
  python visor_ps5_teleop.py --port /dev/ttyACM1             # run with the arm
  python visor_ps5_teleop.py --port /dev/ttyACM1 --id visor_leader_as_follower

Start with the arm in its folded rest pose - that pose becomes "home" and is where
the arm returns before torque is switched off on exit.
"""

import argparse
import importlib
import logging
import os
import sys
import time

# Headless-friendly SDL (works over SSH with no display)
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

logging.getLogger("lerobot.robots.utils").setLevel(logging.ERROR)  # hide step-clamp spam

import pygame  # noqa: E402
from pygame._sdl2 import controller as sdlc  # noqa: E402

# ----------------------------------------------------------------------------- tuning
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

# Speed at full stick, in "% of calibrated range per second" (LeRobot RANGE_M100_100 units)
FAST_SPEED = {"shoulder_pan": 35, "shoulder_lift": 30, "elbow_flex": 35,
              "wrist_flex": 45, "wrist_roll": 60, "gripper": 80}
SLOW_FACTOR = 0.35
HOME_SPEED = 25          # %/s when returning to start pose
DEADZONE = 0.12

# Flip any joint that moves the "wrong" way for your taste (+1 or -1)
INVERT = {"shoulder_pan": -1, "shoulder_lift": -1, "elbow_flex": -1,
          "wrist_flex": 1, "wrist_roll": 1, "gripper": 1}

# Soft limits (stay a little inside the calibrated range)
LIMITS = {j: (-95.0, 95.0) for j in JOINTS}
LIMITS["gripper"] = (2.0, 98.0)

A = pygame  # constant namespace shortcut
AX_LX, AX_LY = A.CONTROLLER_AXIS_LEFTX, A.CONTROLLER_AXIS_LEFTY
AX_RX, AX_RY = A.CONTROLLER_AXIS_RIGHTX, A.CONTROLLER_AXIS_RIGHTY
AX_L2, AX_R2 = A.CONTROLLER_AXIS_TRIGGERLEFT, A.CONTROLLER_AXIS_TRIGGERRIGHT
BTN_SQUARE, BTN_TRIANGLE = A.CONTROLLER_BUTTON_X, A.CONTROLLER_BUTTON_Y
BTN_L1, BTN_R1 = A.CONTROLLER_BUTTON_LEFTSHOULDER, A.CONTROLLER_BUTTON_RIGHTSHOULDER
BTN_OPTIONS = A.CONTROLLER_BUTTON_START


# ----------------------------------------------------------------------------- controller
def stick(raw: int) -> float:
    x = max(-1.0, min(1.0, raw / 32767.0))
    if abs(x) < DEADZONE:
        return 0.0
    x = (abs(x) - DEADZONE) / (1.0 - DEADZONE) * (1 if x > 0 else -1)
    return x * abs(x)  # squared curve = fine control near centre


def trigger(raw: int) -> float:
    t = max(0.0, min(1.0, raw / 32767.0))
    return 0.0 if t < 0.05 else t


def open_pad():
    sdlc.quit()
    sdlc.init()
    for i in range(sdlc.get_count()):
        if sdlc.is_controller(i):
            pad = sdlc.Controller(i)
            try:
                pad.rumble(0.4, 0.4, 250)
            except Exception:
                pass
            return pad
    return None


class Buttons:
    """Rising-edge detection for buttons."""
    def __init__(self):
        self.prev = {}

    def pressed(self, pad, btn) -> bool:
        now = bool(pad.get_button(btn))
        was = self.prev.get(btn, False)
        self.prev[btn] = now
        return now and not was


def test_pad():
    pygame.init()
    pad = open_pad()
    if pad is None:
        sys.exit("No game controller found. Plug the DualSense in by USB-C or pair it over Bluetooth.")
    print(f"Found controller: {pad.name}. Move sticks / press buttons. Ctrl+C to quit.")
    names = {"LX": AX_LX, "LY": AX_LY, "RX": AX_RX, "RY": AX_RY}
    try:
        while True:
            pygame.event.pump()
            ax = " ".join(f"{k}:{stick(pad.get_axis(v)):+.2f}" for k, v in names.items())
            tr = f"L2:{trigger(pad.get_axis(AX_L2)):.2f} R2:{trigger(pad.get_axis(AX_R2)):.2f}"
            bt = " ".join(n for n, b in [("L1", BTN_L1), ("R1", BTN_R1), ("Square", BTN_SQUARE),
                                         ("Triangle", BTN_TRIANGLE), ("Options", BTN_OPTIONS)]
                          if pad.get_button(b))
            print(f"\r{ax}  {tr}  [{bt:<30}]", end="", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print()


# ----------------------------------------------------------------------------- robot
def make_robot(port: str, robot_id: str, max_step: float):
    last_err = None
    for mod_name in ("lerobot.robots.so101_follower", "lerobot.robots.so_follower", "lerobot.robots"):
        try:
            mod = importlib.import_module(mod_name)
            cls, cfg_cls = mod.SO101Follower, mod.SO101FollowerConfig
            break
        except (ImportError, AttributeError) as e:
            last_err = e
    else:
        sys.exit(f"Could not import SO101Follower from LeRobot: {last_err}")

    cfg = cfg_cls(port=port, id=robot_id)
    if hasattr(cfg, "use_degrees"):
        cfg.use_degrees = False          # body joints in -100..100, gripper 0..100
    if hasattr(cfg, "max_relative_target"):
        cfg.max_relative_target = max_step  # per-command step cap (safety)
    if hasattr(cfg, "disable_torque_on_disconnect"):
        cfg.disable_torque_on_disconnect = True
    return cls(cfg)


def clamp(j, v):
    lo, hi = LIMITS[j]
    return max(lo, min(hi, v))


def move_toward(target, goal, step):
    done = True
    for j in JOINTS:
        d = goal[j] - target[j]
        if abs(d) > step:
            target[j] += step if d > 0 else -step
            done = False
        else:
            target[j] = goal[j]
    return done


def boost_gains(robot, joints, p_gain):
    """LeRobot's follower setup sets P=16 (gentle). The leader's lighter-geared motors
    can't hold/lift with that, so raise P on the joints that fight gravity."""
    bus = robot.bus
    try:
        bus.disable_torque(joints)
        for j in joints:
            bus.write("P_Coefficient", j, p_gain)
        bus.enable_torque(joints)
        got = {j: bus.read("P_Coefficient", j, normalize=False) for j in joints}
        print(f"P gain set: {got}")
    except Exception as e:
        print(f"[!] Could not raise P gain ({e}); continuing with defaults")
        try:
            bus.enable_torque(joints)
        except Exception:
            pass


def send(robot, target):
    robot.send_action({f"{j}.pos": float(target[j]) for j in JOINTS})


def run(args):
    pygame.init()
    pad = open_pad()
    if pad is None:
        sys.exit("No game controller found. Plug the DualSense in by USB-C or pair it over Bluetooth.")
    print(f"Controller: {pad.name}")

    robot = make_robot(args.port, args.id, args.max_step)
    print(f"Connecting to arm on {args.port} (id={args.id}) ...")
    robot.connect()  # will run calibration prompts if this id has never been calibrated
    if args.p_gain > 0:
        boost_gains(robot, ["shoulder_lift", "elbow_flex", "wrist_flex"], args.p_gain)

    obs = robot.get_observation()
    home = {j: float(obs[f"{j}.pos"]) for j in JOINTS}
    target = dict(home)
    print("Start pose saved as HOME:", {j: round(v, 1) for j, v in home.items()})
    print("Ready. Options = exit, Triangle = home, Square = slow/fast.\n")

    btn = Buttons()
    slow = False
    homing = False
    period = 1.0 / args.hz
    last = time.perf_counter()
    last_print = 0.0
    last_reconnect = 0.0

    try:
        while True:
            t0 = time.perf_counter()
            dt = min(t0 - last, 0.1)
            last = t0
            pygame.event.pump()

            # --- controller dropped: hold still, try to reconnect
            if pad is None or not pad.attached():
                if t0 - last_reconnect > 1.0:
                    last_reconnect = t0
                    print("\r[!] Controller disconnected - arm holding position ...      ", end="", flush=True)
                    pad = open_pad()
                    btn = Buttons()
                time.sleep(period)
                continue

            if btn.pressed(pad, BTN_OPTIONS):
                print("\nOptions pressed - returning home and exiting.")
                break
            if btn.pressed(pad, BTN_SQUARE):
                slow = not slow
                try:
                    pad.rumble(0.2, 0.6 if slow else 0.2, 120)
                except Exception:
                    pass
            if btn.pressed(pad, BTN_TRIANGLE):
                homing = True

            cmd = {
                "shoulder_pan": stick(pad.get_axis(AX_LX)),
                "shoulder_lift": stick(pad.get_axis(AX_LY)),
                "elbow_flex": stick(pad.get_axis(AX_RY)),
                "wrist_roll": stick(pad.get_axis(AX_RX)),
                "wrist_flex": float(pad.get_button(BTN_R1)) - float(pad.get_button(BTN_L1)),
                "gripper": trigger(pad.get_axis(AX_R2)) - trigger(pad.get_axis(AX_L2)),
            }
            any_input = any(abs(v) > 0 for v in cmd.values())

            if homing and any_input:
                homing = False  # driver takes over
            if homing:
                if move_toward(target, home, HOME_SPEED * dt):
                    homing = False
            else:
                k = SLOW_FACTOR if slow else 1.0
                for j in JOINTS:
                    target[j] = clamp(j, target[j] + INVERT[j] * cmd[j] * FAST_SPEED[j] * k * dt)

            send(robot, target)

            if t0 - last_print > 0.5:
                last_print = t0
                s = " ".join(f"{j[:5]}:{target[j]:+6.1f}" for j in JOINTS)
                mode = "HOME" if homing else ("SLOW" if slow else "FAST")
                print(f"\r[{mode}] {s}   ", end="", flush=True)

            sleep = period - (time.perf_counter() - t0)
            if sleep > 0:
                time.sleep(sleep)

    except KeyboardInterrupt:
        print("\nCtrl+C - returning home and exiting.")
    finally:
        try:
            t_end = time.perf_counter() + 8.0
            while time.perf_counter() < t_end:
                if move_toward(target, home, HOME_SPEED * period):
                    send(robot, target)
                    time.sleep(0.5)
                    break
                send(robot, target)
                time.sleep(period)
        except Exception as e:
            print(f"Could not return home cleanly: {e}")
        robot.disconnect()
        print("Torque off. Bye.")


def main():
    p = argparse.ArgumentParser(description="PS5 DualSense teleop for an SO-101 arm (LeRobot)")
    p.add_argument("--port", default="COM5", help="serial port of the arm's servo board")
    p.add_argument("--id", default="visor_leader_as_follower", help="LeRobot calibration id")
    p.add_argument("--hz", type=float, default=30.0, help="control loop rate")
    p.add_argument("--max-step", type=float, default=8.0, help="max change per command (safety)")
    p.add_argument("--p-gain", type=int, default=32,
                   help="P gain for shoulder_lift/elbow/wrist_flex (LeRobot default 16; 0 = don't change)")
    p.add_argument("--test-pad", action="store_true", help="only test the controller, no robot")
    args = p.parse_args()
    test_pad() if args.test_pad else run(args)


if __name__ == "__main__":
    main()
