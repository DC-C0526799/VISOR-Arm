# VISOR Arm – ACT Policy (Stage 2)

ACT (Action Chunking Transformer) imitation learning for picking up plastic bottles and aluminium cans with the SO-101 follower arm. ACT handles the **pickup**. Moving between saved poses and to the bins is done in Python (see [robot_control/](../robot_control/)).

## Files

| File | Purpose |
|---|---|
| `stage2_act.py` | Stage 2: waits for a YOLO detection, then launches the guarded ACT rollout for the pickup. Stage 3 imports it (`act_command()`, `camera_worker()`, `ACT_MODEL`). Running it on its own gives a test dashboard at `/stage2`. |
| `guarded_act_rollout.py` | Wraps LeRobot's standard `lerobot-rollout` and adds the gripper overload guard (`GRIPPER_GUARD=1`) |

## Training

| Item | Value |
|---|---|
| Demonstrations | **40 episodes**, recorded by moving the follower arm with the SO-101 leader arm (teleoperation) |
| Cameras | `camera1` on the gripper and `camera2` with a top view, both 640 × 480 at 15 fps |
| Framework | LeRobot 0.6.2, ACT policy |
| Final model | **`VISOR_ACT_20000`** |

The model is **not stored in this repo** because it's too large for Git. On the Pi it lives at `~/models/VISOR_ACT_20000/pretrained_model`.

## How Stage 2 runs ACT

`act_command()` builds this command, which runs on the Raspberry Pi 5 CPU:

```bash
python guarded_act_rollout.py \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.id=my_awesome_follower_arm \
  --robot.disable_torque_on_disconnect=false \
  --robot.cameras='{"camera1":{"type":"opencv","index_or_path":0,"width":640,"height":480,"fps":15,"fourcc":"MJPG"},
                    "camera2":{"type":"opencv","index_or_path":2,"width":640,"height":480,"fps":15,"fourcc":"MJPG"}}' \
  --policy.path=~/models/VISOR_ACT_20000/pretrained_model \
  --device=cpu --inference.type=sync --fps=15 \
  --duration=30 --interactive=false --return_to_initial_position=false
```

- `disable_torque_on_disconnect=false` keeps the gripper holding the object after ACT finishes, so it doesn't drop it before moving to the bin.
- The YOLO camera is **released before ACT starts**, because ACT needs the same camera.
- Before any motion, `validate_act_setup()` checks that the model folder exists, that the guard script is present and that the `GRIPPER_*` settings are valid.

| Environment variable | Default | Meaning |
|---|---|---|
| `ACT_MAX_SECONDS` | 30 | maximum length of an ACT rollout (Stage 3 uses its own `ACT_SECONDS`) |
| `GRIPPER_GUARD` | 0 | `1` turns on the overload guard |

## Gripper overload fix

During ACT rollouts, the STS3215 gripper servo used to trip its **overload protection** when it squeezed a bottle or can, which stopped the run. ACT commands a gripper *position* tighter than the object, so the servo kept pushing until the load stayed above its overload threshold for about 2 s.

`guarded_act_rollout.py` fixes this with a load and position feedback guard:
- caps the gripper torque below the overload limit
- detects contact and holds the position instead of squeezing harder
- backs off if the load climbs
- clears an overload flag automatically if one still appears

Enable it with:

```bash
export GRIPPER_GUARD=1
source gripper_settings.sh     # GRIPPER_CLOSE_SIGN, GRIPPER_CONTACT_LOAD,
                               # GRIPPER_TORQUE_LIMIT, GRIPPER_MAX_CLOSE_STEP
python guarded_act_rollout.py --guard-check    # checks the settings without moving the arm
```

Full write-up, including how the cause was found: [docs/GRIPPER_OVERLOAD_FIX.md](../docs/GRIPPER_OVERLOAD_FIX.md)
