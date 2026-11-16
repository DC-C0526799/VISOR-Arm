from __future__ import annotations

"""
guarded_act_rollout.py  (feedback version)

Runs the standard LeRobot ACT rollout. With GRIPPER_GUARD=1 it adds a
feedback loop on the STS3215 gripper servo:

  1. TORQUE CAP BELOW THE OVERLOAD LIMIT
     The servo trips its overload protection when the load stays above
     its Overload_Torque threshold for about 2 s (Feetech spec). The
     guard lowers the output Torque_Limit under that threshold, so the
     gripper physically cannot squeeze hard enough to trip it.

  2. CONTACT -> HOLD (position feedback + load feedback)
     While closing, the gripper moves in small steps. A grasp is only
     declared when ALL of these hold for GRIPPER_CONTACT_TICKS ticks in
     a row: Present_Load >= GRIPPER_CONTACT_LOAD, the jaw has stopped
     moving, and ACT is asking for a clearly tighter grip. Then the
     target is frozen at the current position instead of pushing on.
     A one-tick load spike while the jaw is moving is NOT a grasp.

  3. RELIEF
     While holding, if the load climbs to GRIPPER_RELIEF_LOAD the target
     is backed off one small step at a time.

  4. RELEASE
     If ACT asks for the gripper to open clearly (more than
     GRIPPER_RELEASE_MARGIN away from the held position), the hold ends
     and ACT can open / re-grasp normally.

  5. FAULT RECOVERY
     If the servo still reports an overload / over-current flag, the
     guard clears it (torque off, re-send the present position, torque
     on - the spec says a new position command clears the flag) and
     carries on, instead of stopping the whole run. Other faults
     (voltage, temperature, sensor) still stop the run.

Default is OFF (standard ACT behaviour). Turn on with:

    export GRIPPER_GUARD=1
    source gripper_settings.sh

Optional settings (defaults in brackets):
    GRIPPER_MIN_OBJECT_POS   [8]    closed-and-empty below this position
    GRIPPER_RELEASE_MARGIN   [15]   open request needed to end a hold
    GRIPPER_CONTACT_TICKS    [3]    consecutive ticks needed for contact
    GRIPPER_STILL_TOL        [0.5]  max jaw movement per tick to be "still"
    GRIPPER_MIN_CLOSE_GAP    [5]    ACT target must be this much tighter
    GRIPPER_MAX_OBJECT_POS   [100]  ignore contact above this position
    GRIPPER_RELIEF_LOAD      [0.9 * GRIPPER_TORQUE_LIMIT]
    GRIPPER_RELIEF_STEP      [1.0]  position units backed off per tick
    GRIPPER_RECOVER_MASK     [0x28] Status bits treated as recoverable
    GRIPPER_MAX_RECOVERIES   [5]    recoveries allowed per run
    GRIPPER_TRACE            [0]    1 = print a trace line twice a second
"""

import importlib
import math
import os
import sys
import time
from collections.abc import Mapping
from dataclasses import dataclass
from functools import wraps
from importlib.metadata import entry_points, version
from numbers import Integral
from typing import Any


def guard_enabled():
    return os.environ.get("GRIPPER_GUARD", "0").strip() == "1"


def finite_number(value, label):
    if isinstance(value, bool):
        raise ValueError(f"{label} must be a number")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{label} must be finite")
    return number


def clamp100(value):
    return min(100.0, max(0.0, value))


@dataclass(frozen=True)
class GuardConfig:
    close_sign: int
    contact_load: float
    torque_limit: int
    max_close_step: float
    hold_offset: float = 0.0

    def __post_init__(self):
        if isinstance(self.close_sign, bool) or self.close_sign not in (-1, 1):
            raise ValueError("close_sign must be -1 or +1")

        contact = finite_number(self.contact_load, "contact_load")
        torque = finite_number(self.torque_limit, "torque_limit")
        step = finite_number(self.max_close_step, "max_close_step")
        offset = finite_number(self.hold_offset, "hold_offset")

        if not 0 < contact <= 1000:
            raise ValueError("contact_load must be in (0, 1000]")
        if not torque.is_integer() or not 1 <= torque <= 1000:
            raise ValueError("torque_limit must be an integer from 1 to 1000")
        if not contact < torque:
            raise ValueError("contact_load must be below torque_limit")
        if not 0 < step <= 100:
            raise ValueError("max_close_step must be in (0, 100]")
        if not 0 <= offset <= step:
            raise ValueError("hold_offset must be between 0 and max_close_step")

        object.__setattr__(self, "close_sign", int(self.close_sign))
        object.__setattr__(self, "contact_load", contact)
        object.__setattr__(self, "torque_limit", int(torque))
        object.__setattr__(self, "max_close_step", step)
        object.__setattr__(self, "hold_offset", offset)

    @classmethod
    def from_env(cls):
        required = (
            "GRIPPER_CLOSE_SIGN",
            "GRIPPER_CONTACT_LOAD",
            "GRIPPER_TORQUE_LIMIT",
            "GRIPPER_MAX_CLOSE_STEP",
        )
        missing = [
            name for name in required
            if not os.environ.get(name, "").strip()
        ]
        if missing:
            raise ValueError(
                "Missing calibrated settings: " + ", ".join(missing)
            )

        return cls(
            close_sign=int(os.environ["GRIPPER_CLOSE_SIGN"]),
            contact_load=float(os.environ["GRIPPER_CONTACT_LOAD"]),
            torque_limit=int(os.environ["GRIPPER_TORQUE_LIMIT"]),
            max_close_step=float(os.environ["GRIPPER_MAX_CLOSE_STEP"]),
            hold_offset=float(os.getenv("GRIPPER_HOLD_OFFSET", "0")),
        )


def decode_sts_load(raw):
    # STS load: bit 10 indicates direction; bits 0..9 contain magnitude.
    # Also accept SDK versions that already decode signed values.
    if isinstance(raw, bool) or not isinstance(raw, Integral):
        raise ValueError("Present_Load must be an integer")
    value = int(raw)
    if not -1023 <= value <= 2047:
        raise ValueError(f"Unexpected Present_Load: {value}")
    return -value if value < 0 else value & 1023


class GripperGuard:
    def __init__(self, config):
        self.config = config
        self.held_position = None

        self.min_object_pos = float(
            os.environ.get("GRIPPER_MIN_OBJECT_POS", "8")
        )
        self.release_margin = float(
            os.environ.get("GRIPPER_RELEASE_MARGIN", "15")
        )
        self.relief_load = float(
            os.environ.get(
                "GRIPPER_RELIEF_LOAD",
                str(0.9 * config.torque_limit),
            )
        )
        self.relief_step = float(
            os.environ.get("GRIPPER_RELIEF_STEP", "1.0")
        )
        self.contact_ticks_needed = int(
            os.environ.get("GRIPPER_CONTACT_TICKS", "3")
        )
        self.still_tol = float(
            os.environ.get("GRIPPER_STILL_TOL", "0.5")
        )
        self.min_close_gap = float(
            os.environ.get("GRIPPER_MIN_CLOSE_GAP", "5")
        )
        self.max_object_pos = float(
            os.environ.get("GRIPPER_MAX_OBJECT_POS", "100")
        )

        self.contact_ticks = 0
        self.prev_position = None
        self.last_gap = 0.0

    def after_fault(self, position):
        # After clearing an overload flag, hold slightly more open than
        # the current position so the jaw stops squeezing.
        self.held_position = clamp100(
            finite_number(position, "gripper position")
            - self.config.close_sign * 2.0
        )

    def apply(self, action, position, load):
        if "gripper.pos" not in action:
            raise KeyError("ACT action must contain gripper.pos")

        position = finite_number(position, "gripper position")
        load = finite_number(load, "gripper load")
        requested = finite_number(action["gripper.pos"], "gripper target")

        if not 0 <= position <= 100:
            raise ValueError("Expected normalized gripper position 0..100")
        if not 0 <= load <= 1023:
            raise ValueError("Unexpected gripper load magnitude")

        sign = self.config.close_sign
        result = dict(action)
        target = clamp100(requested)

        previous = self.prev_position
        self.prev_position = position
        still = (
            previous is not None
            and abs(position - previous) <= self.still_tol
        )

        # ---------- HOLDING an object ----------
        if self.held_position is not None:
            # Positive when ACT wants the jaw MORE OPEN than we hold.
            opening_request = -sign * (target - self.held_position)

            if opening_request > self.release_margin:
                # ACT wants to open: end the hold, handled as normal below.
                self.held_position = None
                self.contact_ticks = 0
            else:
                if load >= self.relief_load:
                    # Squeezing too hard: back off a little (opening
                    # direction is -sign).
                    self.held_position = clamp100(
                        self.held_position - sign * self.relief_step
                    )
                result["gripper.pos"] = self.held_position
                return result

        # ---------- FREE (not holding) ----------
        closing_distance = sign * (target - position)
        self.last_gap = closing_distance

        if closing_distance > 0:
            # A real grasp: ACT clearly asks for a tighter grip, the jaw
            # has STOPPED moving, and the load stays high for several
            # ticks in a row. A single load spike while the jaw is moving
            # (or an empty jaw pressing on its own stop) is NOT a grasp.
            touching = (
                load >= self.config.contact_load
                and still
                and closing_distance >= self.min_close_gap
                and self.min_object_pos <= position <= self.max_object_pos
            )
            self.contact_ticks = self.contact_ticks + 1 if touching else 0

            if self.contact_ticks >= self.contact_ticks_needed:
                hold = position + sign * self.config.hold_offset
                self.held_position = clamp100(hold)
                self.contact_ticks = 0
                result["gripper.pos"] = self.held_position
            else:
                step = min(closing_distance, self.config.max_close_step)
                result["gripper.pos"] = position + sign * step
        else:
            self.contact_ticks = 0
            result["gripper.pos"] = target

        return result


def check_motor_status(bus):
    status = int(bus.read("Status", "gripper", normalize=False))
    if status:
        raise RuntimeError(
            f"Gripper hardware fault: 0x{status:02x}. "
            "An existing fault requires recovery."
        )


def recover_gripper_fault(bus):
    """
    Returns True if an overload / over-current flag was found and cleared,
    False if the servo was healthy. Raises for any other fault.
    """
    status = int(bus.read("Status", "gripper", normalize=False))
    if not status:
        return False

    mask = int(os.environ.get("GRIPPER_RECOVER_MASK", "0x28"), 0)
    if status & ~mask:
        raise RuntimeError(
            f"Gripper hardware fault: 0x{status:02x} is not an "
            "overload / over-current flag; stopping."
        )

    print(
        f"GRIPPER FAULT 0x{status:02x}: clearing by re-sending the "
        "present position",
        flush=True,
    )

    bus.disable_torque("gripper")
    time.sleep(0.2)

    raw = int(bus.read("Present_Position", "gripper", normalize=False))
    bus.write("Goal_Position", "gripper", raw, normalize=False)
    bus.enable_torque("gripper")
    time.sleep(0.2)

    status = int(bus.read("Status", "gripper", normalize=False))
    if status:
        raise RuntimeError(
            f"Gripper fault 0x{status:02x} did not clear. "
            "Power-cycle the arm."
        )

    return True


def configure_gripper_limit(bus, config=None):
    config = config or GuardConfig.from_env()
    motor = bus.motors["gripper"]

    if motor.model.lower() != "sts3215":
        raise RuntimeError("This guard requires an STS3215 gripper")

    norm = getattr(motor.norm_mode, "value", motor.norm_mode)
    if norm != "range_0_100":
        raise RuntimeError("Expected range_0_100 gripper normalization")

    # A flag left over from a previous run is cleared, not fatal.
    recover_gripper_fault(bus)

    mode = int(bus.read("Operating_Mode", "gripper", normalize=False))
    if mode != 0:
        raise RuntimeError("Gripper must use position mode")

    overload = int(
        bus.read("Overload_Torque", "gripper", normalize=False)
    )

    # The volatile Torque_Limit may still hold a lower value from the
    # previous run, so compare against the stored maximum when possible.
    try:
        existing_cap = int(
            bus.read("Max_Torque_Limit", "gripper", normalize=False)
        )
    except Exception:
        existing_cap = int(
            bus.read("Torque_Limit", "gripper", normalize=False)
        )

    # Overload_Torque uses 1%; load and Torque_Limit use 0.1%.
    overload_raw = 10 * overload

    if not 0 < config.contact_load < config.torque_limit < overload_raw:
        raise RuntimeError(
            "Require contact_load < torque_limit < "
            f"10 * Overload_Torque ({overload_raw})"
        )

    if config.torque_limit > existing_cap:
        raise RuntimeError(
            f"Requested cap exceeds existing cap {existing_cap}; "
            "this wrapper will not increase the output limit"
        )

    # Volatile output limit; protection settings remain enabled.
    bus.write(
        "Torque_Limit",
        "gripper",
        config.torque_limit,
        normalize=False,
    )
    actual = int(bus.read("Torque_Limit", "gripper", normalize=False))
    if actual != config.torque_limit:
        raise RuntimeError("Gripper torque-limit readback failed")

    print(
        f"GRIPPER GUARD: contact={config.contact_load}, "
        f"cap={actual}, overload_boundary={overload_raw}",
        flush=True,
    )


def find_follower_class():
    for name in (
        "lerobot.robots.so101_follower",
        "lerobot.robots.so_follower",
    ):
        try:
            module = importlib.import_module(name)
        except ModuleNotFoundError as exc:
            if exc.name and (
                name == exc.name or name.startswith(exc.name + ".")
            ):
                continue
            raise

        cls = getattr(module, "SO101Follower", None)
        if cls is not None:
            return cls

    raise RuntimeError("SO101Follower was not found")


def install_guard(cls, config):
    original_configure = cls.configure
    original_send_action = cls.send_action

    state = {"recoveries": 0, "last_trace": 0.0}
    max_recoveries = int(os.environ.get("GRIPPER_MAX_RECOVERIES", "5"))
    trace = os.environ.get("GRIPPER_TRACE", "0").strip() == "1"

    @wraps(original_configure)
    def configure(self, *args, **kwargs):
        original_configure(self, *args, **kwargs)
        configure_gripper_limit(self.bus, config)
        self._sorting_gripper_guard = GripperGuard(config)

    @wraps(original_send_action)
    def send_action(self, action, *args, **kwargs):
        guard = getattr(self, "_sorting_gripper_guard", None)
        if guard is None:
            raise RuntimeError("Gripper guard was not initialized")

        recovered = recover_gripper_fault(self.bus)
        if recovered:
            state["recoveries"] += 1
            if state["recoveries"] > max_recoveries:
                raise RuntimeError(
                    "Too many gripper overload recoveries; stopping."
                )

        position = self.bus.read("Present_Position", "gripper")
        load = decode_sts_load(
            self.bus.read("Present_Load", "gripper", normalize=False)
        )

        if recovered:
            guard.after_fault(position)

        was_holding = guard.held_position is not None
        guarded_action = guard.apply(action, position, load)
        now_holding = guard.held_position is not None

        if not was_holding and now_holding:
            print(
                f"GRIPPER CONTACT/HOLD: position={position:.3f}, "
                f"load={load}. Arm continues.",
                flush=True,
            )
        elif was_holding and not now_holding:
            print(
                f"GRIPPER RELEASE: position={position:.3f}, "
                f"load={load}.",
                flush=True,
            )

        # Preserve LeRobot's existing motion limits and return value.
        result = original_send_action(self, guarded_action, *args, **kwargs)

        if trace:
            now = time.monotonic()
            if now - state["last_trace"] >= 0.5:
                state["last_trace"] = now
                print(
                    "GRIPPER TRACE "
                    f"pos={position:.2f} "
                    f"policy={float(action['gripper.pos']):.2f} "
                    f"sent={float(guarded_action['gripper.pos']):.2f} "
                    f"load={load} hold={guard.held_position} "
                    f"gap={guard.last_gap:.1f} "
                    f"ticks={guard.contact_ticks}",
                    flush=True,
                )

        return result

    cls.configure = configure
    cls.send_action = send_action


def main():
    matches = list(
        entry_points(group="console_scripts", name="lerobot-rollout")
    )
    if len(matches) != 1:
        raise RuntimeError(
            "Run with Python from your LeRobot environment"
        )

    enabled = guard_enabled()

    if sys.argv[1:] == ["--guard-check"]:
        print("LeRobot:", version("lerobot"))
        print("Guard enabled:", enabled)
        if enabled:
            print("Follower:", find_follower_class().__name__)
            print("Configuration:", GuardConfig.from_env())
        print("Check passed; no motor connection opened.")
        return

    if enabled:
        config = GuardConfig.from_env()
        install_guard(find_follower_class(), config)
    else:
        print(
            "GRIPPER GUARD: DISABLED (standard ACT behaviour)",
            flush=True,
        )

    rollout_main = matches[0].load()
    sys.argv[0] = "lerobot-rollout"
    rollout_main()


if __name__ == "__main__":
    main()
