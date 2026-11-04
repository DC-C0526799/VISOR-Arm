# VISOR-Arm
Visor Arm is an AI-powered robotic sorting system using the SO-101 robotic arm, Raspberry Pi 5, YOLO object detection, ACT imitation learning, and PLC integration to automatically sort plastic bottles and aluminum cans.
VISOR Arm
AI-Powered Robotic Sorting System | Capstone Project 2026
VISOR Arm is a capstone project exploring automated identification and sorting of plastic bottles and aluminium cans using an SO-101 robotic arm, Raspberry Pi 5, computer vision, and imitation learning.

The system combines YOLO-based object detection with an ACT (Action Chunking Transformer) policy for robotic pickup and predefined arm positions for placement into the appropriate collection bin.
Project status: Development and integration. The documentation includes successful camera and object-detection tests and an implementation of a continuous sorting controller. A fully validated end-to-end sorting performance result is not documented yet.

Project objectives
- Identify plastic bottles and aluminium cans using camera-based object detection.
- Control an SO-101 arm using calibrated motor positions.
- Use an ACT policy for pickup and transfer toward the sorting area.
- Place detected objects in their corresponding bins and repeat the cycle.
- Provide a web dashboard with status information and a normal-stop request.
System overview
Camera / Raspberry Pi 5
          |
          v
    YOLO detection
          |
          v
  Classify bottle or can
          |
          v
    ACT robot pickup
          |
          v
 Saved pose: correct bin
          |
          v
     Release object
          |
          v
 Return to detection pose
          |
          +----> Next object
The documented controller describes the cycle as:

HOME → DETECTION → YOLO → ACT → CORRECT BIN → RELEASE → DETECTION → REPEAT

A normal stop requests completion of the current operation and a return to HOME if no motor fault occurs. This is not an emergency stop.
Hardware
Component	Role
SO-101 robotic arm	Object pickup, movement, and placement
Raspberry Pi 5	Main computing platform and camera interface
USB camera	Captures images for object detection
Hailo-8L AI accelerator	Investigated for accelerated inference
Motor controllers and calibrated joints	Arm position feedback and movement
Bottle and can collection bins	Separate sorting destinations


Software and technologies
Technology	Purpose
Python	Robot and application logic
LeRobot	SO-101 robot integration and ACT workflows
Ultralytics YOLO	Bottle and can object detection
ACT policy	Learned robotic pickup and transfer behavior
OpenCV	Camera access and image processing
Flask	Browser-based status dashboard and video feed
Hailo software tools	AI accelerator identification and model inspection


Development and testing
1. Robotic arm setup
The SO-101 arm was assembled and its motors calibrated using the LeRobot SO-101 documentation. Joint feedback was inspected to establish positions used for HOME and DETECTION.
2. Camera validation
The USB camera was identified through Linux video-device utilities. OpenCV was used to capture and save a test image at 640 × 480 resolution.
3. Hailo-8L investigation
The Raspberry Pi's Hailo-8L device was identified, the Hailo Python interface was tested, and a compatible YOLO model's input/output information was inspected. These checks do not by themselves establish that the project's custom bottle-and-can detector is running on the Hailo accelerator.
4. Object detection
The documentation includes example YOLO results showing:
- Plastic bottle: detection confidence 0.75.
- Aluminium can: detection confidence 0.89.
These are individual illustrated test results, not a measured accuracy benchmark.
5. Automatic sorting controller
The documented stage3_auto_sort.py implementation coordinates the detection camera, ACT rollout, selection of bottle/can bin poses, object release, and return to the detection position. It also includes joint-position checks, fault handling, and a Flask dashboard.

The implementation references stage1_detection.py and stage2_act.py; these dependencies must be included and checked before the controller can be reproduced from this repository.
Repository organization
Visor-Arm/
├── robot_control/       # SO-101 control and sorting scripts
├── yolo_detection/      # Object-detection code and examples
├── act_policy/          # ACT integration and training notes
├── mqtt_plc/            # PLC and MQTT integration (as available)
├── fusion360_designs/   # Mechanical design files (as available)
├── hardware/            # Hardware diagrams and assembly notes
├── docs/                # Technical report, figures, and test notes
└── README.md
The directory structure is a proposed organization for the GitHub repository. Not every folder or file is present in the uploaded documentation.

Setup and reproducibility
This repository is being organized from the project development notes. The complete software setup, Python dependencies, robot calibration, ACT model files, and device-specific configuration should be documented and verified before publishing a reproducible installation procedure.

Useful upstream reference: LeRobot SO-101 documentation.
Current limitations and next steps
- Validate the full bottle/can sorting cycle and record repeatable test results.
- Document ACT model training, dataset, and inference configuration.
- Measure detection-to-pickup latency and sorting cycle time.
- Confirm whether inference uses the Hailo accelerator or the Raspberry Pi CPU.
- Add clear system photographs, mechanical designs, wiring diagrams, and demonstration media.
- Document PLC/MQTT integration separately once tested and ready to publish.
Safety and responsible use
Operate the robot in a clear workspace and supervise movement during testing. The dashboard's normal stop is a software request, not a safety-rated emergency-stop mechanism. Verify robot calibration and paths before enabling automatic motion.
Documentation
This README is based on the team's “VISOR arm steps documentation” development notes (59 pages), including assembly photographs, camera setup, detection screenshots, and a draft continuous-sorting Python implementation.
Project information
Project: VISOR Arm
Type: Capstone Project
Year: 2026
Focus: Robotics · Computer Vision · Imitation Learning · Automated Sorting
This README will be updated as the project is tested and completed.



## 📚 Project Documentation

Explore the technical documentation for the VISOR Arm capstone project.

| Document | Description |
|---|---|
| [Technical Report](docs/VISOR_Arm_Technical_Documentation.pdf) | Project overview, development and testing |
| [Setup Guide](docs/SETUP_GUIDE.md) | Hardware and software configuration |
| [System Architecture](docs/SYSTEM_ARCHITECTURE.md) | System components and workflow diagrams |
| [Project Progress](docs/PROJECT_PROGRESS.md) | Completed work and ongoing development |

### Project Modules

- [Robot Control](robot_control/)
- [YOLO Object Detection](yolo_detection/)
- [ACT Imitation Learning](act_policy/)
- [MQTT and PLC Integration](mqtt_plc/)
- [Fusion 360 Designs](fusion360_designs/)
- [Hardware Documentation](hardware/)

