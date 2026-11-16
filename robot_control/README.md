# VISOR Arm – Robot Control (Stage 3: Continuous Sorting)

`visor_stage3_sort_v2.py` is the top-level controller. It runs the full sorting loop until it's told to stop:

```
HOME → DETECTION → YOLO → ACT pickup → CORRECT BIN → RELEASE → DETECTION → repeat
```

It uses **Stage 1** (`yolo_detection/stage1_detection.py`) for the camera, YOLO, the motor bus and the saved poses. It uses **Stage 2** (`act_policy/stage2_act.py`) to run the ACT pickup through the **gripper overload guard** ([details](../docs/GRIPPER_OVERLOAD_FIX.md)).

## Hybrid control approach

| Part of the cycle | Controlled by |
|---|---|
| HOME, DETECTION, moving to a bin, returning | **Saved joint poses** that Python moves between smoothly |
| Picking up the object | **ACT policy** (`VISOR_ACT_20000`) |
| Bottle or can decision | **YOLO** class from Stage 1 |

## How to run

All the stage scripts must be in the same folder (`~/capstone_vision_ik` on the Pi), because they import each other.

```bash
cd ~/capstone_vision_ik
export GRIPPER_GUARD=1
source gripper_settings.sh
ACT_SECONDS=60 ENABLE_BIN_MOTION=1 python3 visor_stage3_sort_v2.py
```

| Environment variable | Default | Meaning |
|---|---|---|
| `VISOR_ACT_MODEL` | `/home/dileeschandran/models/VISOR_ACT_20000/pretrained_model` | ACT model to use |
| `ACT_SECONDS` | 30 | maximum time allowed for each ACT pickup |
| `ENABLE_BIN_MOTION` | 0 | must be `1` for the arm to actually move to the bins (a safety interlock for first tests) |
| `SINGLE_CYCLE` | 0 | `1` = stop after one object, `0` = continuous sorting |

## Safety features

- **Position feedback:** every joint is read before and after each move, with 3 retries so that one garbled servo packet doesn't stop the run.
- **Normal stop:** Ctrl+C or the dashboard's STOP button finishes the current object, then returns the arm to HOME. This is **not an emergency stop**.
- **Fault handling:** if a motor fault or a feedback failure happens, automatic motion stops instead of guessing at a safe path home.
- **Bin-motion interlock:** with `ENABLE_BIN_MOTION=0`, the loop runs detection and ACT but doesn't move to the bins.
