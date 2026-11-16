# VISOR Arm – YOLO Object Detection (Stage 1)

YOLO-based detection of **plastic bottles** and **aluminium cans** on the Raspberry Pi 5. Stage 1 also provides the shared robot connection, the saved poses, the camera thread and the Flask dashboard that Stage 2 and Stage 3 build on.

## File

| File | Purpose |
|---|---|
| `stage1_detection.py` | Loads the YOLO model, runs the camera and detection loop, connects to the SO-101 follower, loads the saved poses, and serves the Flask dashboard and video feed |

## Model and dataset

| Item | Value |
|---|---|
| Classes | `plastic_bottle`, `aluminium_can` |
| Dataset | about 6,267 images, assembled from Roboflow sources |
| Model | Ultralytics **YOLO11n**, custom-trained on an RTX 3050 |
| Inference | `imgsz=640`, confidence threshold `0.50` |
| Initial test mAP50 | bottle **93.8%**, can **4.9%** |

> **Known issue:** the first trained model detected cans poorly (mAP50 of 4.9%). Improving can detection with more or better-balanced can images is listed as future work.

The trained weights (`models/best.pt`) are **not stored in this repo**.

## How detection works

1. The camera at `/dev/video0` (V4L2) is read in a background thread.
2. Each frame goes through YOLO, and the **most confident** detection is kept.
3. An object is only accepted once the same class has stayed in about the same place for **10 frames in a row** (`STABLE_FRAMES`). This stops a single-frame false detection from starting the robot.
4. The confirmed class (`plastic_bottle` or `aluminium_can`) is passed to the sorting controller.

## Files it expects (on the Pi)

`stage1_detection.py` reads everything from `~/capstone_vision_ik`:

```
~/capstone_vision_ik/
├── models/best.pt                      # YOLO weights
├── calibration/home_pose.json
├── calibration/detection_pose.json
├── calibration/sorting_poses.json      # pickup_open/closed, lift_closed, can_bin_closed, bottle_bin_closed
├── calibration/gripper_open.json
├── calibration/gripper_closed.json
├── templates/dashboard.html
└── auto_sort_sequence.py
```
It also reads the LeRobot follower calibration from `~/.cache/huggingface/lerobot/calibration/robots/so_follower/my_awesome_follower_arm.json`. The follower arm is on `/dev/ttyACM0`.

## Dashboard

Running Stage 1 starts a Flask server on **port 5000**:

| Route | Purpose |
|---|---|
| `/` and `/dashboard` | status page |
| `/video_feed` | live camera feed with the detection boxes drawn on |
| `/api/status` | robot state as JSON |
| `/stop` (POST) | normal stop request (not an emergency stop) |

Open `http://<pi-ip>:5000` from any device on the same network.
