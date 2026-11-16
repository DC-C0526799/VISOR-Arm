ACT (Action Chunking Transformer) imitation learning for the VISOR Arm project. This module uses demonstrations recorded through the SO-101 leader and follower arms to train robotic pickup and transfer movements for plastic bottles and aluminium cans.

## Files

| File | Purpose |
|---|---|
| `stage2_act.py` | Stage 2: takes the YOLO detection and runs the guarded ACT pickup/transfer (imported by Stage 3) |
| `guarded_act_rollout.py` | Standard LeRobot ACT rollout with an optional gripper overload guard |

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
source gripper_settings.sh
python guarded_act_rollout.py <lerobot-rollout arguments>
```

Full write-up, including how the cause was found: [docs/GRIPPER_OVERLOAD_FIX.md](../docs/GRIPPER_OVERLOAD_FIX.md)
