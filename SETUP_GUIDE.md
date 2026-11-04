# VISOR Arm — Setup Guide

## 1. Introduction

VISOR Arm is an AI-powered robotic sorting system designed to identify and sort plastic bottles and aluminium cans using an SO-101 robotic arm.

The system combines computer vision, imitation learning, robotic arm control, and industrial communication.

## 2. Hardware Requirements

- Raspberry Pi 5
- SO-101 follower robotic arm
- SO-101 leader arm for teleoperation and training
- Feetech servo motors
- USB cameras for object detection and robot observation
- Conveyor system
- Ultrasonic object detection sensor
- Hailo-8L AI accelerator (optional integration)
- Allen-Bradley CompactLogix 5380 PLC (integration in progress)

## 3. Software Requirements

- Raspberry Pi OS
- Python
- LeRobot
- Ultralytics YOLO
- OpenCV
- NumPy
- Flask
- Mosquitto MQTT broker
- Paho MQTT
- pycomm3 for PLC communication

## 4. Project Python Environments

The VISOR Arm development setup uses separate environments for different tasks.

| Environment | Purpose |
|---|---|
| `lerobot` | SO-101 robotic arm control and ACT policy |
| `vision_env` | Computer vision and Hailo experimentation |
| `plc_mqtt` | MQTT and PLC communication |

Exact Python and dependency versions should be verified against the Raspberry Pi installation before reproducing the setup.

## 5. Camera Configuration

The documented camera configuration includes:

| Camera | Device | Resolution |
|---|---|---|
| Gripper camera | `/dev/video0` | 640 × 480 |
| Top camera | `/dev/video2` | 640 × 480 |

Camera indices may change after rebooting or reconnecting USB devices.

## 6. Robotic Arm Connection

The development setup uses:

- SO-101 follower arm: `/dev/ttyACM0`
- SO-101 leader arm: `/dev/ttyACM1`

The robot must be correctly assembled and calibrated before running movement scripts.

## 7. Object Detection

YOLO is used to identify:

- `plastic_bottle`
- `aluminium_can`

The detection system provides the object class to the robotic sorting controller.

## 8. ACT Policy

The ACT imitation-learning policy is used for robotic pickup and transfer movements.

Training demonstrations are recorded using the SO-101 leader and follower arms.

The trained model and its configuration must be available before running ACT inference.

## 9. Automatic Sorting Workflow

1. Move the robot to HOME.
2. Move to the DETECTION position.
3. Detect the object using YOLO.
4. Execute the ACT policy for pickup and transfer.
5. Move toward the selected sorting bin.
6. Release the object.
7. Return to DETECTION.
8. Repeat for the next object.

A normal stop request is intended to finish the current operation and return the arm to HOME when no motor fault occurs.

## 10. MQTT and PLC Integration

The system includes development work involving:

- Mosquitto MQTT broker
- Paho MQTT Python library
- Allen-Bradley CompactLogix 5380 PLC
- pycomm3 communication library

Full PLC integration and testing procedures will be documented separately.

## 11. Safety and Testing

- Verify the robot's calibration before movement.
- Keep the robot's movement area clear.
- Test saved positions at low speeds.
- Check for collisions before enabling automatic sorting.
- Do not treat the software normal-stop button as an emergency stop.
- Verify each subsystem before running a complete sorting cycle.

## 12. Setup Status

This guide describes the documented development configuration. It is not yet a fully verified, reproducible installation procedure.

Exact installation commands, package versions, model paths, and end-to-end test results will be added after verification on the Raspberry Pi 5.
