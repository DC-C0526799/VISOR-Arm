from auto_sort_sequence import execute_sorting_cycle
from flask import jsonify
from flask import render_template
import json
import time
import threading
from pathlib import Path

import cv2
from flask import Flask, Response, jsonify
from ultralytics import YOLO

from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.motors.motors_bus import (
    Motor, MotorNormMode, MotorCalibration
)

BASE = Path.home() / "capstone_vision_ik"
CAL = Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower/my_awesome_follower_arm.json"

home = json.loads((BASE / "calibration/home_pose.json").read_text())
detect = json.loads((BASE / "calibration/detection_pose.json").read_text())

# Saved sorting positions
sorting = json.loads(
    (BASE / "calibration/sorting_poses.json").read_text()
)

pickup_open = sorting["pickup_open"]
pickup_closed = sorting["pickup_closed"]
lift_closed = sorting["lift_closed"]
can_bin_closed = sorting["can_bin_closed"]
bottle_bin_closed = sorting["bottle_bin_closed"]

gripper_open = json.loads(
    (BASE / "calibration/gripper_open.json").read_text()
)["gripper"]

gripper_closed = json.loads(
    (BASE / "calibration/gripper_closed.json").read_text()
)["gripper"]

print("Sorting positions loaded successfully.")
data = json.loads(CAL.read_text())

motors = {
    name: Motor(
        values["id"], "sts3215",
        MotorNormMode.RANGE_0_100 if name == "gripper"
        else MotorNormMode.DEGREES
    )
    for name, values in data.items()
}

bus = FeetechMotorsBus(
    port="/dev/ttyACM0",
    motors=motors,
    calibration={
        name: MotorCalibration(**values)
        for name, values in data.items()
    }
)

app = Flask(__name__)
model = YOLO(str(BASE / "models/best.pt"))

latest_frame = None
frame_lock = threading.Lock()
stop_event = threading.Event()
emergency_event = threading.Event()
robot_finished = threading.Event()
robot_state = "STARTING"

# Automatic object detection state
detected_object = None
detection_lock = threading.Lock()
stable_count = 0
last_center = None
last_class = None
STABLE_FRAMES = 10
POSITION_TOLERANCE = 12

# Prevent repeated detection of the same object
object_latched = False
missing_frames = 0
RESET_FRAMES = 15
latched_class = None
latched_center = None

# Allowed gripper lag (normalized 0-100 units) while moving.
GRIPPER_MOVE_TOLERANCE = 25


def move_slowly(start, end, duration=12):
    steps = 120

    for i in range(steps + 1):
        if emergency_event.is_set():
            raise RuntimeError("Emergency stop requested")

        t = i / steps
        t = t * t * (3 - 2 * t)

        target = {
            name: start[name] + (end[name] - start[name]) * t
            for name in motors
        }

        bus.sync_write("Goal_Position", target)
        time.sleep(duration / steps)

        if i % 5 == 0:
            actual = {
                name: bus.read("Present_Position", name)
                for name in motors
            }

            for name in motors:
                # A gripper holding an object cannot reach its target
                # exactly, so it gets a looser tolerance than the joints.
                limit = GRIPPER_MOVE_TOLERANCE if name == "gripper" else 8
                if abs(actual[name] - target[name]) > limit:
                    bus.sync_write("Goal_Position", actual)
                    raise RuntimeError(
                        f"Movement error: {name} "
                        f"target={target[name]:.1f} "
                        f"actual={actual[name]:.1f}"
                    )


def camera_worker():
    global latest_frame
    global detected_object, stable_count, last_center, last_class
    global object_latched, missing_frames
    global latched_class, latched_center

    camera = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not camera.isOpened():
        print("Camera unavailable")
        stop_event.set()
        return

    try:
        while not robot_finished.is_set():
            ok, frame = camera.read()
            if not ok:
                time.sleep(0.1)
                continue

            results = model.predict(
                frame,
                imgsz=640,
                conf=0.50,
                device="cpu",
                verbose=False
            )

            annotated = results[0].plot()

            # Track the most confident detected object.
            boxes = results[0].boxes

            if robot_state == "DETECTING" and len(boxes) > 0:
                best = max(
                    boxes,
                    key=lambda b: float(b.conf[0])
                )

                class_id = int(best.cls[0])
                class_name = model.names[class_id]

                x1, y1, x2, y2 = best.xyxy[0].tolist()
                center = (
                    (x1 + x2) / 2,
                    (y1 + y2) / 2
                )

                cx = int(center[0])
                cy = int(center[1])

                cv2.circle(
                    annotated, (cx, cy), 6,
                    (0, 255, 255), -1
                )

                cv2.putText(
                    annotated,
                    f"X:{cx} Y:{cy}",
                    (max(0, cx - 55), max(20, cy - 15)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 255),
                    2
                )

                if (
                    class_name == last_class
                    and last_center is not None
                    and abs(center[0] - last_center[0]) < POSITION_TOLERANCE
                    and abs(center[1] - last_center[1]) < POSITION_TOLERANCE
                ):
                    stable_count += 1
                else:
                    stable_count = 1

                last_center = center
                last_class = class_name

                if object_latched:
                    same_object = (
                        class_name == latched_class
                        and latched_center is not None
                        and abs(center[0] - latched_center[0]) < 40
                        and abs(center[1] - latched_center[1]) < 40
                    )

                    if same_object:
                        missing_frames = 0
                    else:
                        missing_frames += 1
                        if missing_frames >= RESET_FRAMES:
                            with detection_lock:
                                object_latched = False
                                latched_class = None
                                latched_center = None
                            print("OBJECT REMOVED - READY FOR NEXT OBJECT")

                if stable_count >= STABLE_FRAMES:
                    with detection_lock:
                        if not object_latched and detected_object is None:
                            detected_object = class_name
                            object_latched = True
                            latched_class = class_name
                            latched_center = center
                            missing_frames = 0
                            print(
                                f"STATIONARY OBJECT DETECTED: {class_name}"
                            )
            else:
                stable_count = 0
                last_center = None
                last_class = None

                # Reset only after the object has disappeared.
                # Do not reset while the robot is moving.
                if robot_state == "DETECTING":
                    missing_frames += 1

                    if missing_frames >= RESET_FRAMES:
                        with detection_lock:
                            object_latched = False

            ok, encoded = cv2.imencode(
                ".jpg", annotated,
                [cv2.IMWRITE_JPEG_QUALITY, 75]
            )

            if ok:
                with frame_lock:
                    latest_frame = encoded.tobytes()

    finally:
        camera.release()





def move_to_pose(label, target):
    """Move to a calibrated pose and verify the result."""
    global robot_state

    if emergency_event.is_set():
        raise RuntimeError("Movement interrupted")

    robot_state = f"MOVING TO {label}"
    print(robot_state)

    current = {
        name: bus.read("Present_Position", name)
        for name in motors
    }

    move_slowly(current, target, duration=15)

    actual = {
        name: bus.read("Present_Position", name)
        for name in motors
    }

    for name in motors:
        error = abs(actual[name] - target[name])
        if error > 5:
            raise RuntimeError(
                f"Position error at {label}: {name} ({error:.2f})"
            )

    print(f"Reached {label}")


def test_can_sorting_route():
    """Supervised dry run, without gripping an object."""
    global robot_state

    lift_open = lift_closed.copy()
    lift_open["gripper"] = gripper_open

    can_open = can_bin_closed.copy()
    can_open["gripper"] = gripper_open

    detection_open = detect.copy()
    detection_open["gripper"] = gripper_open

    route = [
        ("OPEN GRIPPER AT DETECTION", detection_open),
        ("PICKUP", pickup_open),
        ("LIFT", lift_open),
        ("CAN BIN", can_open),
        ("LIFT", lift_open),
        ("DETECTION", {**detect, "gripper": gripper_open}),
    ]

    for label, target in route:
        if emergency_event.is_set():
            raise RuntimeError("Emergency stop requested")

        robot_state = f"MOVING TO {label}"
        print(robot_state)

        current = {
            name: bus.read("Present_Position", name)
            for name in motors
        }

        move_slowly(current, target, duration=15)

        actual = {
            name: bus.read("Present_Position", name)
            for name in motors
        }

        for name in motors:
            if abs(actual[name] - target[name]) > 5:
                raise RuntimeError(
                    f"Position error at {label}: {name}"
                )

        print(f"Reached {label}")

    robot_state = "DETECTING"
    print("CAN ROUTE COMPLETED - RETURNED TO DETECTION")

def robot_worker():
    global robot_state, detected_object

    try:
        bus.connect(handshake=False)

        current = {
            name: bus.read("Present_Position", name)
            for name in motors
        }

        print("Current robot position:", current)

        # Accept any readable starting position.
        # HOME recovery is performed after torque is enabled.
        for name in motors:
            raw = bus.read("Present_Position", name, normalize=False)
            bus.write("Goal_Position", name, raw, normalize=False)

        bus.enable_torque()

        # STEP 1: Return to HOME from the current position.
        robot_state = "RETURNING TO HOME"
        print(robot_state)

        recovery_home = home.copy()
        recovery_home["gripper"] = current["gripper"]

        move_slowly(current, recovery_home, duration=20)

        current = {
            name: bus.read("Present_Position", name)
            for name in motors
        }

        for name in motors:
            if name != "gripper" and abs(current[name] - home[name]) > 5:
                raise RuntimeError(f"HOME recovery error: {name}")

        print("HOME POSITION REACHED")

        # STEP 2: Move HOME -> DETECTION.
        robot_state = "MOVING TO DETECTION"
        print(robot_state)

        initial_detection = detect.copy()
        initial_detection["gripper"] = current["gripper"]

        move_slowly(current, initial_detection, duration=12)

        actual = {
            name: bus.read("Present_Position", name)
            for name in motors
        }

        for name in motors:
            if abs(actual[name] - initial_detection[name]) > 5:
                raise RuntimeError(f"Detection pose error: {name}")

        robot_state = "DETECTING"
        print("Robot at DETECTION. Live YOLO active.")

        # One supervised dry-run of the aluminum can route.
        # No object is picked up and the gripper stays open.
        print("AUTOMATIC SORTING MODE READY")
        print("Waiting for a stationary object detected by YOLO...")

        while not stop_event.is_set():
            with detection_lock:
                object_name = detected_object
                detected_object = None

            if False and object_name in ("aluminium_can", "plastic_bottle"):
                print(f"AUTOMATIC SORTING TARGET: {object_name}")

                poses = {
                    **sorting,
                    "detect": detect,
                    "gripper_open": gripper_open,
                }

                poses = {
                    **sorting,
                    "detect": detect,
                    "gripper_open": gripper_open,
                }

                def automatic_move(label, pose):
                    # NORMAL STOP is handled after the sorting cycle.
                    # Complete the current route before returning HOME.
                    move_to_pose(label, pose)

                robot_state = f"SORTING {object_name.upper()}"

                execute_sorting_cycle(
                    object_name=object_name,
                    move_to_pose=automatic_move,
                    stop_requested=stop_event.is_set,
                    poses=poses,
                )

                robot_state = "DETECTING"
                print("READY FOR NEXT OBJECT", flush=True)

            time.sleep(0.1)

        robot_state = "RETURNING HOME"
        actual = {
            name: bus.read("Present_Position", name)
            for name in motors
        }

        move_slowly(actual, home)
        robot_state = "HOME"

    except Exception as exc:
        robot_state = f"ERROR: {exc}"
        print(robot_state)

        if bus.is_connected:
            try:
                actual = {
                    name: bus.read("Present_Position", name)
                    for name in motors
                }
                bus.sync_write("Goal_Position", actual)
            except Exception:
                pass

    finally:
        robot_finished.set()
        if bus.is_connected:
            at_home = False
            try:
                actual = {
                    name: bus.read("Present_Position", name)
                    for name in motors
                }
                at_home = all(
                    abs(actual[name] - home[name]) <= 2
                    for name in motors
                )
            except Exception as exc:
                print(f"Cannot verify HOME: {exc}")

            if at_home:
                print("HOME verified. Disabling motor torque.")
                bus.disconnect(disable_torque=True)
            else:
                print("WARNING: Robot is not confirmed at HOME.")
                print("Keeping torque enabled to support the arm.")
                bus.disconnect(disable_torque=False)


def generate_frames():
    while not robot_finished.is_set():
        with frame_lock:
            frame = latest_frame

        if frame is not None:
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n"
                + frame
                + b"\r\n"
            )

        time.sleep(0.1)


@app.route("/")
def index():
    return """
    <html>
    <body style="background:#151515;color:white;text-align:center;font-family:Arial">
        <h2>SO-101 Robot + Live YOLO</h2>
        <img src="/video_feed" style="width:90%;max-width:800px">
        <p>Robot moves HOME to DETECTION and streams YOLO video.</p>
        <p><button onclick="stopRobot()" style="padding:15px;font-size:18px">
            NORMAL STOP - RETURN HOME
        </button></p>
        <p id="status">Robot running</p>
        <script>
        async function stopRobot() {
            document.getElementById("status").innerText = "Return HOME requested";
            const response = await fetch("/stop", {method:"POST"});
            const result = await response.json();
            document.getElementById("status").innerText = result.message;
        }
        </script>
    </body>
    </html>
    """


@app.route("/stop", methods=["POST"])
def normal_stop():
    if robot_state != "DETECTING":
        return jsonify({
            "message": f"STOP unavailable while robot is {robot_state}"
        }), 409

    stop_event.set()
    return jsonify({
        "message": "Normal STOP accepted. Returning HOME."
    })




@app.route("/api/status")
def api_status():
    with detection_lock:
        current_object = last_class if robot_state == "DETECTING" else None

    return jsonify({
        "robot_state": robot_state,
        "detected_object": current_object,
        "sorting_mode": "DETECTION TEST",
        "conveyor": "NOT CONNECTED"
    })

@app.route("/dashboard")
def dashboard():
    return render_template("dashboard.html")

@app.route("/video_feed")
def video_feed():
    return Response(
        generate_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


if __name__ == "__main__":
    threading.Thread(target=camera_worker, daemon=True).start()
    threading.Thread(target=robot_worker, daemon=True).start()

    try:
        app.run(host="0.0.0.0", port=5000, threaded=True)
    except KeyboardInterrupt:
        print("Normal STOP requested.")
    finally:
        stop_event.set()
        print("Waiting for robot to return HOME...")
        robot_finished.wait(timeout=45)
        print("Shutdown requested.")
