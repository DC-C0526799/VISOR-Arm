# VISOR Arm — Project Development Progress

**Project:** AI-Powered Robotic Sorting System  
**Platform:** SO-101 Robotic Arm + Raspberry Pi 5  
**Academic Year:** 2026

## Completed Development Work

- SO-101 robotic arm assembly and motor calibration
- Raspberry Pi 5 and USB camera configuration
- YOLO detection of plastic bottles and aluminium cans
- HOME and DETECTION position programming
- ACT imitation-learning training and initial robot movement testing
- Python-based automatic sorting control development
- Flask-based monitoring dashboard
- MQTT broker installation and initial communication testing
- Raspberry Pi 5 enclosure design in Fusion 360
- Gripper overload error diagnosed and fixed with a load/position feedback guard (`act_policy/guarded_act_rollout.py`), working 12 Nov 2026 — see [docs/GRIPPER_OVERLOAD_FIX.md](docs/GRIPPER_OVERLOAD_FIX.md)
- Continuous sorting loop (`robot_control/visor_stage3_sort_v2.py`) using the `VISOR_ACT_20000` ACT model
- Demonstration to the professor on 13 Nov 2026
- PS5 DualSense teleoperation of the SO-101 leader arm re-used as a follower (`teleoperation/`)
- Source code for Stages 1–3 and teleoperation published to this repository

## Integration and Improvement Work

- PLC and MQTT communication integration
- Reducing YOLO-to-ACT activation delay
- Improving movement smoothness between saved robot positions
- Optimizing robotic pickup and placement
- Measuring repeatable sorting results (success rate, cycle time)
- Improving aluminium can detection accuracy
- Finalizing the curved gripper design for cylindrical objects

## Planned Documentation

- YOLO and ACT configuration details
- MQTT and PLC communication scripts
- Fusion 360 design files
- Hardware photographs and demonstration videos
- Final testing results and performance measurements

## Project Workflow

HOME → DETECTION → YOLO → ACT PICKUP → CORRECT BIN → RELEASE → DETECTION → REPEAT

On a normal stop request, the robot is intended to return to HOME after completing the current operation, provided no motor fault occurs.

## Notes

This progress file records development activities and planned improvements. Individual subsystem tests do not necessarily confirm successful end-to-end operation.
