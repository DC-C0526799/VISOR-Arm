"""
SO-101 CONTINUOUS AUTOMATIC SORTING

Sequence:
HOME -> DETECTION -> YOLO -> ACT
-> CORRECT BIN -> RELEASE -> DETECTION -> REPEAT

Ctrl+C / dashboard STOP:
Request normal stop, finish current operation,
then return HOME if there is no motor fault.

IMPORTANT:
This is a normal stop, NOT an emergency stop.
"""

import os
import time
import threading
import subprocess
from pathlib import Path

from flask import jsonify

import stage1_detection as s
import stage2_act as a


# ============================================
# CONFIGURATION
# ============================================

ACT_SECONDS = float(os.environ.get("ACT_SECONDS", "30"))
# Override Stage 2's default model path with the model previously used for VISOR Arm.
a.ACT_MODEL = Path(os.environ.get(
    "VISOR_ACT_MODEL",
    "/home/dileeschandran/models/VISOR_ACT_20000/pretrained_model",
))

ACT_EXIT_TIMEOUT = 30

BIN_MOVE_SECONDS = 4
RELEASE_SECONDS = 3
RETURN_SECONDS = 5

ENABLE_BIN_MOTION = (
    os.environ.get("ENABLE_BIN_MOTION", "0") == "1"
)

# FALSE = CONTINUOUS SORTING
# TRUE = STOP AFTER ONE CYCLE
SINGLE_CYCLE = os.environ.get("SINGLE_CYCLE", "0") == "1"

stop_requested = threading.Event()
worker_finished = threading.Event()

cycle_count = 0
current_act = None
act_running = threading.Event()

faulted = False


# ============================================
# MOTOR POSITION FEEDBACK
# ============================================

def get_pose():
    pose = {}

    for name in s.motors:
        last_error = None

        for attempt in range(3):
            try:
                pose[name] = s.bus.read(
                    "Present_Position",
                    name
                )
                last_error = None
                break

            except Exception as exc:
                last_error = exc

                print(
                    f"Motor {name}: feedback error "
                    f"(attempt {attempt + 1}/3): {exc}",
                    flush=True
                )

                time.sleep(0.2)

        if last_error is not None:
            raise RuntimeError(
                f"Motor {name} feedback unavailable "
                f"after 3 attempts: {last_error}"
            )

    return pose

# ============================================
# CONNECT ROBOT
# ============================================

def read_raw_position(name):
    # Retry: a single garbled packet should not stop the whole run.
    last_error = None

    for attempt in range(3):
        try:
            return s.bus.read(
                "Present_Position",
                name,
                normalize=False
            )
        except Exception as exc:
            last_error = exc
            print(
                f"Motor {name}: read error "
                f"(attempt {attempt + 1}/3): {exc}",
                flush=True
            )
            time.sleep(0.3)

    raise last_error


def connect_robot():

    if not s.bus.is_connected:

        print(
            "CONNECTING SO-101 MOTOR BUS",
            flush=True
        )

        s.bus.connect(handshake=False)

        # Set targets to actual motor positions
        # before enabling torque.

        for name in s.motors:

            raw_position = read_raw_position(name)

            s.bus.write(
                "Goal_Position",
                name,
                raw_position,
                normalize=False
            )

        s.bus.enable_torque()


# ============================================
# MOVE ROBOT WITH POSITION VERIFICATION
# ============================================

def move_robot(target, duration, label):

    print(
        label,
        flush=True
    )

    s.robot_state = label

    start = get_pose()

    s.move_slowly(
        start,
        target,
        duration=duration
    )

    actual = get_pose()

    errors = {}

    for name in s.motors:

        difference = abs(
            actual[name] - target[name]
        )

        # A gripper holding an object cannot reach its target exactly,
        # so it gets a looser tolerance than the arm joints.
        limit = (
            getattr(s, "GRIPPER_MOVE_TOLERANCE", 25)
            if name == "gripper" else 8
        )

        if difference > limit:
            errors[name] = round(
                difference,
                2
            )

    if errors:

        raise RuntimeError(
            f"{label}: Position mismatch {errors}"
        )

    print(
        label,
        "COMPLETE",
        flush=True
    )


# ============================================
# CAMERA START
# ============================================

def start_detection_camera():

    print(
        "STARTING YOLO CAMERA",
        flush=True
    )

    a.camera_stop.clear()
    a.camera_released.clear()

    with s.detection_lock:
        s.detected_object = None

    if hasattr(s, "stable_count"):
        s.stable_count = 0

    if hasattr(s, "last_class"):
        s.last_class = None

    if hasattr(s, "last_center"):
        s.last_center = None

    camera_thread = threading.Thread(
        target=a.camera_worker,
        daemon=True
    )

    camera_thread.start()
    # Camera failure is reported by the Stage 2 worker; the worker also
    # releases its camera event so the main loop can stop safely.


# ============================================
# CAMERA STOP
# ============================================

def stop_detection_camera():

    print(
        "STOPPING YOLO CAMERA",
        flush=True
    )

    a.camera_stop.set()

    released = a.camera_released.wait(
        timeout=15
    )

    if not released:

        raise RuntimeError(
            "Camera was not released"
        )


# ============================================
# HOME POSITION
# ============================================

def return_home():

    print(
        "RETURNING ROBOT TO HOME",
        flush=True
    )

    connect_robot()

    current = get_pose()

    target = dict(s.home)

    # Keep current gripper position.
    target["gripper"] = current["gripper"]

    move_robot(
        target,
        RETURN_SECONDS,
        "RETURNING HOME"
    )

    s.robot_state = "HOME - STOPPED"

    print(
        "ROBOT REACHED HOME",
        flush=True
    )


# ============================================
# DETECTION POSITION
# ============================================

def return_to_detection():

    print(
        "MOVING ROBOT TO DETECTION POSITION",
        flush=True
    )

    connect_robot()

    current = get_pose()

    target = dict(s.detect)

    target["gripper"] = current["gripper"]

    move_robot(
        target,
        RETURN_SECONDS,
        "RETURNING TO DETECTION"
    )

    print(
        "ROBOT READY FOR DETECTION",
        flush=True
    )


# ============================================
# ACT PICKUP
# ============================================

def run_act(object_name):

    global current_act

    print(
        "ACT STARTED FOR:",
        object_name,
        flush=True
    )

    s.robot_state = (
        f"ACT PICKUP: {object_name}"
    )

    command = list(a.act_command())

    # Replace ACT duration in the command.

    new_command = []
    found_duration = False

    index = 0

    while index < len(command):

        item = command[index]

        if item == "--duration":

            new_command.extend([
                "--duration",
                str(ACT_SECONDS)
            ])

            found_duration = True
            index += 2
            continue

        if item.startswith("--duration="):

            new_command.append(
                f"--duration={ACT_SECONDS}"
            )

            found_duration = True
            index += 1
            continue

        new_command.append(item)
        index += 1

    if not found_duration:

        new_command.extend([
            "--duration",
            str(ACT_SECONDS)
        ])

    # Keep torque enabled after ACT so the
    # gripper does not automatically release.

    torque_option = (
        "--robot.disable_torque_on_disconnect=false"
    )

    new_command = [
        item
        for item in new_command
        if not item.startswith(
            "--robot.disable_torque_on_disconnect="
        )
    ]

    new_command.append(torque_option)

    print(
        "STARTING ACT ROLLOUT",
        flush=True
    )

    act_running.set()

    try:

        current_act = subprocess.Popen(
            new_command,
            start_new_session=True
        )

        result = current_act.wait()

        if result != 0:

            raise RuntimeError(
                f"ACT rollout failed with code {result}"
            )

        print(
            "ACT ROLLOUT FINISHED",
            flush=True
        )

    finally:

        current_act = None
        act_running.clear()


# ============================================
# BIN POSES
# ============================================

def get_bin_poses(object_name):

    if object_name == "plastic_bottle":

        closed_pose = dict(
            s.sorting["bottle_bin_closed"]
        )

        open_pose = dict(
            s.sorting["bottle_bin_open"]
        )

    elif object_name == "aluminium_can":

        closed_pose = dict(
            s.sorting["can_bin_closed"]
        )

        open_pose = dict(
            s.sorting["can_bin_open"]
        )

    else:

        raise RuntimeError(
            f"Unknown object: {object_name}"
        )

    return closed_pose, open_pose


# ============================================
# OBJECT PLACEMENT
# ============================================

def place_object(object_name):

    if not ENABLE_BIN_MOTION:

        raise RuntimeError(
            "Bin motion is disabled. "
            "Set ENABLE_BIN_MOTION=1 after "
            "verifying the movement path."
        )

    print(
        "PLACING OBJECT:",
        object_name,
        flush=True
    )

    connect_robot()

    closed_pose, open_pose = get_bin_poses(
        object_name
    )

    current = get_pose()

    # Preserve ACT gripper position
    # while moving toward the bin.

    closed_pose["gripper"] = current["gripper"]

    move_robot(
        closed_pose,
        BIN_MOVE_SECONDS,
        "MOVING TO SELECTED BIN"
    )

    # Release object.

    move_robot(
        open_pose,
        RELEASE_SECONDS,
        "RELEASING OBJECT"
    )

    time.sleep(1)

    print(
        "OBJECT RELEASE COMPLETE",
        flush=True
    )


# ============================================
# CONTINUOUS SORTING WORKER
# ============================================

def sorting_worker():

    global cycle_count
    global faulted

    camera_active = False
    at_home = False

    try:

        print(
            "AUTOMATIC SORTING STARTED",
            flush=True
        )

        connect_robot()

        # Initial HOME position.

        return_home()
        at_home = True

        if not stop_requested.is_set():

            return_to_detection()
            at_home = False

            start_detection_camera()
            camera_active = True

            s.robot_state = "DETECTING"

        # ====================================
        # CONTINUOUS SORTING LOOP
        # ====================================

        while not stop_requested.is_set():

            if a.camera_released.is_set() and not a.camera_stop.is_set():
                raise RuntimeError("YOLO camera exited before detection; check /dev/video0")

            with s.detection_lock:

                object_name = s.detected_object

                s.detected_object = None

            if object_name not in (
                "plastic_bottle",
                "aluminium_can"
            ):

                time.sleep(0.1)
                continue

            cycle_count += 1

            print(
                "================================",
                flush=True
            )

            print(
                f"SORTING CYCLE {cycle_count}",
                flush=True
            )

            print(
                "DETECTED:",
                object_name,
                flush=True
            )

            print(
                "================================",
                flush=True
            )

            # Stop camera before ACT.

            stop_detection_camera()
            camera_active = False

            if stop_requested.is_set():
                break

            # Disconnect Stage 1 bus.
            # ACT takes control of the robot.

            s.bus.disconnect(
                disable_torque=False
            )

            # Run ACT pickup and transfer.

            run_act(object_name)

            # A stop request during ACT does not abandon the object:
            # the cycle finishes by placing it in the bin first.

            # Reconnect after ACT.

            connect_robot()

            # Proceed directly to the bin-placement sequence.
            # Motor fault detection and movement feedback remain active.

            # Move to selected bin.

            place_object(object_name)

            print(
                "================================",
                flush=True
            )

            print(
                f"CYCLE {cycle_count} COMPLETE",
                flush=True
            )

            print(
                "================================",
                flush=True
            )

            if SINGLE_CYCLE:
                stop_requested.set()

            # STOP pressed: the object is already released,
            # so go straight HOME (handled below the loop).

            if stop_requested.is_set():
                break

            # Otherwise: back to the detection pose, then repeat.

            return_to_detection()

            time.sleep(1.0)  # let the arm settle before detecting

            # Restart YOLO for next object.

            start_detection_camera()
            camera_active = True

            s.robot_state = "DETECTING"

            print(
                "READY FOR NEXT OBJECT",
                flush=True
            )

        # ====================================
        # NORMAL STOP
        # ====================================

        print(
            "NORMAL STOP REQUESTED",
            flush=True
        )

        if camera_active:

            stop_detection_camera()
            camera_active = False

        if not at_home:

            return_home()
            at_home = True

        s.robot_state = "HOME - STOPPED"

        print(
            "SORTING STOPPED AT HOME",
            flush=True
        )

    # ========================================
    # MOTOR / MOVEMENT FAULT
    # ========================================

    except Exception as exc:

        faulted = True

        s.robot_state = (
            "ERROR - AUTOMATIC MOTION HALTED: "
            + str(exc)
        )

        print(
            s.robot_state,
            flush=True
        )

        print(
            "FAULT: Automatic HOME movement "
            "not attempted.",
            flush=True
        )

    # ========================================
    # CLEANUP
    # ========================================

    finally:

        a.camera_stop.set()

        if camera_active:
            a.camera_released.wait(timeout=15)

        if s.bus.is_connected:

            try:

                s.bus.disconnect(
                    disable_torque=(
                        at_home and not faulted
                    )
                )

            except Exception as exc:

                print(
                    "BUS DISCONNECT WARNING:",
                    exc,
                    flush=True
                )

        s.robot_finished.set()
        worker_finished.set()

        print(
            "SORTING WORKER FINISHED",
            flush=True
        )


# ========================================ACT_SECONDS=60 ENABLE_BIN_MOTION=1 python3 visor_stage3_sort_v2.py====
# DASHBOARD
# ============================================

@s.app.route("/stage3")
def stage3_dashboard():

    return """
    <!DOCTYPE html>
    <html>
    <head>
        <title>SO-101 Continuous Sorting</title>
        <meta name="viewport"
              content="width=device-width, initial-scale=1">
    </head>

    <body style="
        background:#171717;
        color:white;
        font-family:Arial;
        text-align:center;
        padding:20px;
    ">

        <h2>SO-101 AUTOMATIC SORTING</h2>

        <h3>Continuous Mode</h3>

        <img
            src="/video_feed"
            style="width:95%;max-width:800px;"
        >

        <p>
            YOLO → ACT → BIN → RELEASE
            → DETECTION → REPEAT
        </p>

        <p>
            Camera may pause during ACT.
        </p>

        <button
            style="
                padding:18px;
                background:#d44;
                color:white;
                border:0;
                font-size:18px;
                border-radius:8px;
            "
            onclick="requestStop()"
        >
            REQUEST NORMAL STOP
        </button>

        <h3>Robot Status</h3>

        <pre id="status"></pre>

        <script>

        async function requestStop() {

            const response = await fetch(
                '/stop',
                {method:'POST'}
            );

            const result = await response.json();

            alert(result.message);
        }

        setInterval(async () => {

            try {

                const response = await fetch(
                    '/api/status'
                );

                const data = await response.json();

                document.getElementById(
                    'status'
                ).textContent = JSON.stringify(
                    data,
                    null,
                    2
                );

            } catch(error) {}

        }, 1000);

        </script>

    </body>
    </html>
    """


# ============================================
# NORMAL STOP BUTTON
# ============================================

def request_normal_stop():

    stop_requested.set()

    print(
        "DASHBOARD NORMAL STOP REQUESTED",
        flush=True
    )

    return jsonify(
        message=(
            "Normal stop requested. "
            "The robot will return HOME "
            "after the current operation "
            "if no motor fault occurs."
        )
    )


# Replace existing Stage 1 stop handler,
# or register one if not already present.

if "normal_stop" in s.app.view_functions:

    s.app.view_functions[
        "normal_stop"
    ] = request_normal_stop

else:

    s.app.add_url_rule(
        "/stop",
        endpoint="stage3_normal_stop",
        view_func=request_normal_stop,
        methods=["POST"]
    )


# ============================================
# MAIN
# ============================================

if __name__ == "__main__":

    if ACT_SECONDS <= 0:

        raise SystemExit(
            "ACT_SECONDS must be greater than zero"
        )

    if not a.ACT_MODEL.exists():

        raise SystemExit(
            f"ACT model not found: {a.ACT_MODEL}"
        )

    s.robot_finished.clear()

    worker = threading.Thread(
        target=sorting_worker,
        daemon=True
    )

    worker.start()

    print(
        "================================",
        flush=True
    )

    print(
        ("SO-101 SORTING (single cycle)" if SINGLE_CYCLE
         else "SO-101 SORTING (continuous until STOP)"),
        flush=True
    )

    print(
        "Dashboard:",
        "http://<RASPBERRY_PI_IP>:5000/stage3",
        flush=True
    )

    print(
        "Press Ctrl+C to request normal stop",
        flush=True
    )

    print(
        "================================",
        flush=True
    )

    try:

        s.app.run(
            host="0.0.0.0",
            port=5000,
            threaded=True,
            use_reloader=False
        )

    except KeyboardInterrupt:

        print(
            "\nCTRL+C RECEIVED",
            flush=True
        )

    finally:

        stop_requested.set()

        print(
            "WAITING FOR ROBOT WORKER...",
            flush=True
        )

        finished = worker_finished.wait(
            timeout=max(
                60,
                ACT_SECONDS + 90
            )
        )

        if finished:

            print(
                "FINAL ROBOT STATUS:",
                s.robot_state,
                flush=True
            )

        else:

            print(
                "WARNING: Robot worker "
                "did not finish.",
                flush=True
            )

            print(
                "HOME POSITION NOT CONFIRMED",
                flush=True
            )
