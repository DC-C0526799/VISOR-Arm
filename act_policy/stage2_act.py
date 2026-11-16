#!/usr/bin/env python3
"""
SO-101 Stage 2: YOLO -> guarded ACT pickup/transfer.

Stage 3 can import:
    ACT_MODEL
    act_command()
    camera_worker()
    camera_stop
    camera_released

Requires:
    stage1_detection.py
    guarded_act_rollout.py
    Calibrated GRIPPER_* environment settings

Bin placement and release remain in Stage 3.
"""

import math
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import cv2
from flask import jsonify

import stage1_detection as s

from guarded_act_rollout import (
    GuardConfig,
    configure_gripper_limit,
)


# ============================================
# CONFIGURATION
# ============================================

BASE = Path.home() / "capstone_vision_ik"

ACT_MODEL = BASE.parent / "models/VISOR_ACT_20000/pretrained_model"

GUARD_SCRIPT = Path(__file__).resolve().with_name(
    "guarded_act_rollout.py"
)

ACT_MAX_SECONDS = float(
    os.getenv("ACT_MAX_SECONDS", "30")
)

ACT_EXIT_TIMEOUT = 12


# ============================================
# EVENTS AND PROCESS STATE
# ============================================

camera_stop = threading.Event()
camera_released = threading.Event()
camera_ready = threading.Event()

camera_error = None

act_stop = threading.Event()
act_interrupt_sent = threading.Event()
act_running = threading.Event()

act_lock = threading.Lock()
act_process = None


# ============================================
# VALIDATE BEFORE MOTION
# ============================================

def validate_act_setup():

    if (
        not math.isfinite(ACT_MAX_SECONDS)
        or ACT_MAX_SECONDS <= 0
    ):
        raise ValueError(
            "ACT_MAX_SECONDS must be positive and finite"
        )

    if not ACT_MODEL.is_dir():
        raise FileNotFoundError(
            f"Missing ACT model: {ACT_MODEL}"
        )

    if not GUARD_SCRIPT.is_file():
        raise FileNotFoundError(
            f"Missing guarded launcher: {GUARD_SCRIPT}"
        )

    return GuardConfig.from_env()


# ============================================
# GUARDED ACT COMMAND
# ============================================

def act_command():

    validate_act_setup()

    return [
        sys.executable,
        str(GUARD_SCRIPT),

        "--robot.type=so101_follower",

        "--robot.disable_torque_on_disconnect=false",

        "--robot.port=/dev/ttyACM0",

        "--robot.id=my_awesome_follower_arm",

        '--robot.cameras={"camera1":{"type":"opencv","index_or_path":0,"width":640,"height":480,"fps":15,"fourcc":"MJPG"},"camera2":{"type":"opencv","index_or_path":2,"width":640,"height":480,"fps":15,"fourcc":"MJPG"}}',

        f"--policy.path={ACT_MODEL}",

        "--device=cpu",

        "--inference.type=sync",

        "--fps=15",

        f"--duration={ACT_MAX_SECONDS}",

        "--interactive=false",

        "--return_to_initial_position=false",

        "--display_data=false",

        "--play_sounds=false",
    ]


# ============================================
# DETECTION STATE
# ============================================

def reset_detection():

    s.stable_count = 0
    s.last_class = None
    s.last_center = None


# ============================================
# YOLO CAMERA WORKER
# ============================================

def camera_worker():

    global camera_error

    camera_error = None
    camera_ready.clear()

    cam = None

    try:

        cam = cv2.VideoCapture(
            "/dev/video0",
            cv2.CAP_V4L2
        )

        cam.set(
            cv2.CAP_PROP_FRAME_WIDTH,
            640
        )

        cam.set(
            cv2.CAP_PROP_FRAME_HEIGHT,
            480
        )

        if not cam.isOpened():
            raise RuntimeError(
                "YOLO camera unavailable"
            )

        camera_ready.set()

        while (
            not camera_stop.is_set()
            and not s.robot_finished.is_set()
        ):

            ok, frame = cam.read()

            if not ok:
                time.sleep(0.05)
                continue

            result = s.model.predict(
                frame,
                imgsz=640,
                conf=0.50,
                device="cpu",
                verbose=False
            )[0]

            annotated = result.plot()
            boxes = result.boxes

            if (
                s.robot_state == "DETECTING"
                and boxes is not None
                and len(boxes)
            ):

                valid = [
                    box
                    for box in boxes
                    if s.model.names[int(box.cls[0])]
                    in (
                        "aluminium_can",
                        "plastic_bottle"
                    )
                ]

                if valid:

                    best = max(
                        valid,
                        key=lambda box: float(
                            box.conf[0]
                        )
                    )

                    name = s.model.names[
                        int(best.cls[0])
                    ]

                    x1, y1, x2, y2 = (
                        best.xyxy[0].tolist()
                    )

                    center = (
                        (x1 + x2) / 2,
                        (y1 + y2) / 2
                    )

                    stable = (
                        name == s.last_class
                        and s.last_center is not None
                        and abs(
                            center[0] - s.last_center[0]
                        ) < s.POSITION_TOLERANCE
                        and abs(
                            center[1] - s.last_center[1]
                        ) < s.POSITION_TOLERANCE
                    )

                    s.stable_count = (
                        s.stable_count + 1
                        if stable
                        else 1
                    )

                    s.last_class = name
                    s.last_center = center

                    if (
                        s.stable_count
                        >= s.STABLE_FRAMES
                    ):

                        with s.detection_lock:

                            if s.detected_object is None:

                                s.detected_object = name

                                print(
                                    "YOLO TRIGGER:",
                                    name,
                                    flush=True
                                )

                else:
                    reset_detection()

            else:
                reset_detection()

            ok, jpg = cv2.imencode(
                ".jpg",
                annotated,
                [
                    cv2.IMWRITE_JPEG_QUALITY,
                    75
                ]
            )

            if ok:

                with s.frame_lock:
                    s.latest_frame = jpg.tobytes()

    except Exception as exc:

        camera_error = str(exc)

        s.robot_state = f"ERROR: {exc}"
        s.stop_event.set()

        print(
            s.robot_state,
            flush=True
        )

    finally:

        if cam is not None:
            cam.release()

        camera_ready.set()
        camera_released.set()

        print(
            "YOLO camera released",
            flush=True
        )


# ============================================
# ROBOT POSITION FEEDBACK
# ============================================

def read_pose():

    return {
        name: s.bus.read(
            "Present_Position",
            name
        )
        for name in s.motors
    }


# ============================================
# VERIFIED MOVEMENT
# ============================================

def move_and_verify(target, duration, label):

    s.robot_state = label

    print(
        label,
        flush=True
    )

    s.move_slowly(
        read_pose(),
        target,
        duration=duration
    )

    actual = read_pose()

    errors = {
        name: round(
            abs(actual[name] - target[name]),
            2
        )
        for name in s.motors
        if abs(
            actual[name] - target[name]
        ) > 5
    }

    if errors:
        raise RuntimeError(
            f"{label}: position mismatch {errors}"
        )


# ============================================
# REQUEST ACT STOP
# ============================================

def request_act_interrupt():

    act_stop.set()

    with act_lock:

        process = act_process

        if (
            process is not None
            and process.poll() is None
            and not act_interrupt_sent.is_set()
        ):

            try:

                os.killpg(
                    process.pid,
                    signal.SIGINT
                )

                act_interrupt_sent.set()

            except ProcessLookupError:
                pass


# ============================================
# RUN ONE GUARDED ACT PICKUP
# ============================================

def run_act_once(object_name):

    global act_process

    command = act_command()

    if s.bus.is_connected:
        raise RuntimeError(
            "Parent bus must be disconnected before ACT"
        )

    if not camera_released.is_set():
        raise RuntimeError(
            "YOLO camera must be released before ACT"
        )

    s.robot_state = (
        f"ACT PICKUP/TRANSFER: {object_name}"
    )

    env = os.environ.copy()
    env["OMP_NUM_THREADS"] = "4"

    process = None
    operator_stopped = False

    act_interrupt_sent.clear()

    try:

        with act_lock:

            if (
                act_process is not None
                and act_process.poll() is None
            ):
                raise RuntimeError(
                    "An ACT process is already running"
                )

            process = subprocess.Popen(
                command,
                env=env,
                start_new_session=True
            )

            act_process = process
            act_running.set()

        print(
            "Guarded ACT started for",
            object_name,
            flush=True
        )

        while process.poll() is None:

            if (
                act_stop.is_set()
                or s.stop_event.is_set()
            ):

                operator_stopped = True

                request_act_interrupt()

                try:

                    process.wait(
                        timeout=ACT_EXIT_TIMEOUT
                    )

                except subprocess.TimeoutExpired as exc:

                    raise RuntimeError(
                        f"ACT process {process.pid} "
                        "still owns the robot. "
                        "Do not reconnect the bus "
                        "or command HOME."
                    ) from exc

                break

            time.sleep(0.1)

        operator_stopped = (
            operator_stopped
            or act_interrupt_sent.is_set()
        )

        result = process.returncode

        interrupted_codes = (
            -signal.SIGINT,
            128 + signal.SIGINT
        )

        if (
            result != 0
            and not (
                operator_stopped
                and result in interrupted_codes
            )
        ):
            raise RuntimeError(
                f"ACT rollout failed with exit code {result}"
            )

        if operator_stopped:

            s.robot_state = (
                "ACT STOPPED - INSPECT OBJECT "
                "AND TRANSFER POSE"
            )

        else:

            s.robot_state = (
                "ACT ENDED - VERIFY OBJECT "
                "AND TRANSFER POSE"
            )

        print(
            s.robot_state,
            flush=True
        )

    finally:

        with act_lock:

            if (
                process is not None
                and process.poll() is not None
            ):

                if act_process is process:

                    act_process = None
                    act_running.clear()


# ============================================
# STANDALONE STAGE 2 ROBOT WORKER
# ============================================

def robot_worker():

    faulted = False

    try:

        config = validate_act_setup()

        if not camera_ready.wait(timeout=15):
            raise RuntimeError(
                "YOLO camera startup timed out"
            )

        if camera_error:
            raise RuntimeError(camera_error)

        if s.stop_event.is_set():
            return

        s.bus.connect(handshake=False)

        # Apply the gripper output cap before
        # target initialization and torque enabling.
        configure_gripper_limit(
            s.bus,
            config
        )

        for name in s.motors:

            raw = s.bus.read(
                "Present_Position",
                name,
                normalize=False
            )

            s.bus.write(
                "Goal_Position",
                name,
                raw,
                normalize=False
            )

        s.bus.enable_torque()

        current = read_pose()

        move_and_verify(
            dict(
                s.home,
                gripper=current["gripper"]
            ),
            20,
            "RETURNING HOME"
        )

        if not s.stop_event.is_set():

            current = read_pose()

            move_and_verify(
                dict(
                    s.detect,
                    gripper=current["gripper"]
                ),
                12,
                "MOVING TO DETECTION"
            )

            s.robot_state = "DETECTING"

        while not s.stop_event.is_set():

            with s.detection_lock:

                name = s.detected_object
                s.detected_object = None

            if name in (
                "aluminium_can",
                "plastic_bottle"
            ):

                s.robot_state = "HANDOFF TO ACT"

                camera_stop.set()

                if not camera_released.wait(timeout=15):

                    raise RuntimeError(
                        "YOLO camera did not release"
                    )

                if s.stop_event.is_set():
                    break

                # ACT gets exclusive motor-bus ownership.
                s.bus.disconnect(
                    disable_torque=False
                )

                run_act_once(name)

                # Stage 2 ends here.
                # Stage 3 performs bin placement and release.
                return

            time.sleep(0.1)

        if s.bus.is_connected:

            current = read_pose()

            move_and_verify(
                dict(
                    s.home,
                    gripper=current["gripper"]
                ),
                12,
                "RETURNING HOME"
            )

            s.robot_state = "HOME - STOPPED"

    except Exception as exc:

        faulted = True

        s.robot_state = f"ERROR: {exc}"

        print(
            s.robot_state,
            flush=True
        )

    finally:

        camera_stop.set()

        if s.bus.is_connected:

            at_home = False

            try:

                current = read_pose()

                at_home = (
                    not faulted
                    and all(
                        abs(
                            current[name] - s.home[name]
                        ) <= 2
                        for name in s.motors
                    )
                )

            except Exception as exc:

                print(
                    "Cleanup feedback warning:",
                    exc,
                    flush=True
                )

            finally:

                try:

                    s.bus.disconnect(
                        disable_torque=at_home
                    )

                except Exception as exc:

                    print(
                        "Bus cleanup warning:",
                        exc,
                        flush=True
                    )

        s.robot_finished.set()


# ============================================
# ACT STOP ENDPOINT
# ============================================

@s.app.route(
    "/act_stop",
    methods=["POST"]
)
def stop_act():

    if not act_running.is_set():

        return jsonify(
            message="ACT is not currently running"
        ), 409

    request_act_interrupt()

    return jsonify(
        message=(
            "ACT stop requested. "
            "Wait for the process to exit."
        )
    )


# ============================================
# NORMAL STOP ENDPOINT
# ============================================

def request_normal_stop():

    if act_running.is_set():

        return jsonify(
            message="ACT is running. Use STOP ACT."
        )

    s.stop_event.set()

    return jsonify(
        message=(
            "Normal stop requested; "
            "return HOME if no fault occurs."
        )
    )


if "normal_stop" in s.app.view_functions:

    s.app.view_functions[
        "normal_stop"
    ] = request_normal_stop

else:

    s.app.add_url_rule(
        "/stop",
        endpoint="normal_stop",
        view_func=request_normal_stop,
        methods=["POST"]
    )


# ============================================
# DASHBOARD
# ============================================

@s.app.route("/stage2")
def stage2_dashboard():

    return """
<!doctype html>
<html>
<head>
    <title>SO-101 Stage 2</title>
    <meta name="viewport"
          content="width=device-width, initial-scale=1">
</head>

<body style="
    background:#151515;
    color:white;
    text-align:center;
    font-family:Arial;
">

    <h2>SO-101: YOLO + Guarded ACT</h2>

    <img src="/video_feed"
         style="width:90%;max-width:800px">

    <p>Video pauses while ACT owns the cameras.</p>

    <p>
        <button onclick="sendStop('/act_stop')"
                style="padding:16px">
            STOP ACT
        </button>
    </p>

    <p>
        <button onclick="sendStop('/stop')"
                style="padding:16px">
            NORMAL STOP BEFORE ACT
        </button>
    </p>

    <pre id="state"></pre>

    <script>
    async function sendStop(path) {
        try {
            const response = await fetch(
                path,
                {method: 'POST'}
            );

            const data = await response.json();
            alert(data.message);

        } catch (error) {
            alert(String(error));
        }
    }

    setInterval(async () => {
        try {
            const response = await fetch('/api/status');

            document.getElementById('state').textContent =
                JSON.stringify(
                    await response.json(),
                    null,
                    2
                );

        } catch (error) {}
    }, 1000);
    </script>

</body>
</html>
"""


# ============================================
# MAIN
# ============================================

def main():

    # Validate paths/settings before starting hardware workers.
    validate_act_setup()

    s.stop_event.clear()
    s.robot_finished.clear()

    camera_stop.clear()
    camera_released.clear()
    camera_ready.clear()

    act_stop.clear()
    act_interrupt_sent.clear()

    reset_detection()

    with s.detection_lock:
        s.detected_object = None

    print(
        "Open http://PI_IP:5000/stage2",
        flush=True
    )

    threading.Thread(
        target=camera_worker,
        daemon=True
    ).start()

    threading.Thread(
        target=robot_worker,
        daemon=True
    ).start()

    try:

        s.app.run(
            host="0.0.0.0",
            port=5000,
            threaded=True,
            use_reloader=False
        )

    except KeyboardInterrupt:

        print(
            "Normal stop requested",
            flush=True
        )

    finally:

        s.stop_event.set()
        camera_stop.set()

        if act_running.is_set():
            request_act_interrupt()

        if not s.robot_finished.wait(timeout=45):

            print(
                "Robot worker has not finished; "
                "HOME is not confirmed.",
                flush=True
            )

        if act_running.is_set():

            print(
                "ACT has not exited. "
                "Do not reconnect another motor controller.",
                flush=True
            )


if __name__ == "__main__":
    main()
