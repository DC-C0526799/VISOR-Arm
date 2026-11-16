# VISOR Arm – Fusion 360 Designs

The VISOR Arm is built on the open-source **SO-101 robotic arm** ([TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100)). Every part of the arm body is the standard open-source design **except the gripper**, which we redesigned so it holds **round plastic bottles and aluminium cans** reliably.

## Modified parts

| File | Part | What changed |
|---|---|---|
| `Gripper_arm.f3d` | Moving gripper jaw | Redesigned with a **curved gripping surface** that fits cylindrical objects |
| `Wrist_Roll_Follower_20261106.f3d` | Wrist roll / fixed jaw (follower arm) | Changed to match the new moving jaw, so both sides close around a bottle or can |

Open the `.f3d` files in **Autodesk Fusion 360** (File → Open → Open from my computer).

## Why we modified the gripper

The stock SO-101 gripper has flat, narrow jaws. On round objects such as plastic bottles and aluminium cans, flat jaws only touch along a thin line, so the object can **slip or rotate** while the arm lifts and moves it to a bin.

The curved design:
- wraps around the object, so it **touches more of its surface** and stops it rolling out
- needs **less squeezing force** for a secure hold, which also helps with the gripper overload problem (see [docs/GRIPPER_OVERLOAD_FIX.md](../docs/GRIPPER_OVERLOAD_FIX.md))
- keeps the original **Feetech STS3215 servo mounting**, so it fits the standard SO-101 with no other changes

## Printing

Export each body as STL from Fusion 360 (right-click the body → Save as Mesh). Then print them the same way as the other SO-101 parts, using the print settings recommended in the SO-ARM100 repository. Replace only the gripper and wrist-roll parts. All other parts come from the open-source repository.
