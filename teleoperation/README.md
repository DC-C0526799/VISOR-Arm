# PS5 Controller Teleoperation

The SO-101 arm is driven live with a **PS5 DualSense controller**. For the demo, the SO-101 **leader arm was re-used as a follower**, so it moves under motor power instead of being moved by hand. Teleoperation runs from a **Windows laptop**, but the script also works on the Raspberry Pi 5 or on Linux.

| File | Purpose |
|---|---|
| `visor_ps5_teleop.py` | Main teleoperation script: reads the controller and drives the arm through LeRobot |
| `visor_motor_check.py` | Diagnostic tool: prints every motor's status registers and test-moves one joint |

---

## Controls

| DualSense input | Arm action |
|---|---|
| Left stick X | `shoulder_pan` (rotates the base) |
| Left stick Y | `shoulder_lift` |
| Right stick Y | `elbow_flex` |
| Right stick X | `wrist_roll` |
| R1 / L1 | `wrist_flex` up / down |
| R2 / L2 | gripper open / close (analog: press harder to move faster) |
| Square | toggle SLOW / FAST speed (the controller rumbles to confirm) |
| Triangle | smoothly return to the start (HOME) pose |
| Options | return to HOME, then exit with torque off |

If the controller disconnects, the arm **freezes in place** and the script keeps trying to reconnect.

---

## Step-by-step process

### Step 1 – Install the software (laptop)
```bash
pip install "lerobot[feetech]" pygame
```
- **LeRobot 0.6.x** talks to the Feetech STS3215 servos.
- **pygame 2.x** reads the controller through SDL's game-controller API, which already knows the DualSense button layout.

### Step 2 – Connect the PS5 controller
- **USB-C:** plug it in.
- **Bluetooth (Windows):** go to Settings → Bluetooth & devices → Add device → Bluetooth. Hold **PS + Create** on the controller until the light bar flashes, then select *Wireless Controller*.

Test the controller on its own before connecting the arm:
```bash
python visor_ps5_teleop.py --test-pad
```
It prints the stick values (`LX LY RX RY`), the triggers (`L2 R2`) and any pressed buttons. This confirms every input works and the dead-zone feels right, with no chance of moving the robot.

### Step 3 – Convert the leader arm into a follower
The leader arm normally has no motor power and is moved by hand. To make it a follower:
1. Power the leader arm's servo board with the follower's power supply, and connect it to the laptop over USB.
2. Find its serial port in Windows Device Manager → Ports (COM & LPT). Ours was **`COM5`**. On the Pi it would be something like `/dev/ttyACM1`.
3. Give it its own LeRobot calibration ID, **`visor_leader_as_follower`**, so it doesn't overwrite the real leader or follower calibration.

### Step 4 – Check the motors
```bash
python visor_motor_check.py --port COM5 --id visor_leader_as_follower --joint elbow_flex
```
This prints a table of every motor's registers: `Torque_Enable`, `Present_Position`, `Present_Load`, `Present_Voltage`, `Present_Temperature`, `Status`, the torque limits and `Overload_Torque`. It then moves the chosen joint ±150 steps and reports whether it actually **MOVED**.
This shows that every servo responds, the voltage is right and there are no fault flags, before teleoperation starts.

### Step 5 – Calibrate and run teleoperation
Place the arm in its **folded rest pose** first. The script saves that pose as **HOME** and returns to it before turning the torque off.
```bash
python visor_ps5_teleop.py --port COM5 --id visor_leader_as_follower
```
The first time this ID is used, LeRobot runs its calibration prompts. After that it connects straight away.

Optional settings:

| Flag | Default | Meaning |
|---|---|---|
| `--hz` | 30 | control loop rate |
| `--max-step` | 8.0 | largest position change allowed per command (safety limit) |
| `--p-gain` | 32 | P gain for shoulder_lift / elbow_flex / wrist_flex (0 = keep LeRobot's default of 16) |

### Step 6 – Tune it
We adjusted these constants at the top of `visor_ps5_teleop.py` until the arm felt natural:
- **`INVERT`:** flips any joint that moved the opposite way to the stick. Pan, shoulder lift and elbow are inverted.
- **`FAST_SPEED` / `SLOW_FACTOR`:** sets each joint's speed at full stick. SLOW mode runs at 35% for fine positioning.
- **`DEADZONE` (0.12) and a squared stick curve:** stops the arm drifting when the stick is centred and gives finer control near the centre.
- **`LIMITS`:** soft limits kept slightly inside the calibrated range (±95 for the joints, 2–98 for the gripper), so the arm never reaches its mechanical end stops.

---

## How it works

```
DualSense ──(USB/Bluetooth)──► pygame (SDL) ──► visor_ps5_teleop.py ──► LeRobot SO101Follower ──► STS3215 servos
```
1. The loop runs at 30 Hz and reads the sticks, triggers and buttons.
2. The stick values (with the dead-zone and squared curve applied) are **velocities**: each tick, `target += direction × stick × speed × dt`.
3. Each target is clamped to the soft limits, then sent to the arm with `robot.send_action()`.
4. LeRobot's `max_relative_target` limits how far any joint can move in one command, as an extra safety layer.
5. On exit (Options, Ctrl+C or an error), the arm moves back to HOME at a slow speed, then disconnects with the torque turned off.

---

## Problems we hit and how we solved them

| Problem | Cause | Fix |
|---|---|---|
| Arm sagged and couldn't lift or hold its position | LeRobot's follower setup sets a gentle P gain of 16. The leader arm's servos are lighter-geared, so they can't hold against gravity at that gain. | `boost_gains()` raises the P gain to 32 on `shoulder_lift`, `elbow_flex` and `wrist_flex`, the joints that work against gravity. |
| Some joints moved the opposite way to the stick | Servo direction and calibration don't match the stick direction | The per-joint `INVERT` table |
| Needed to know whether a joint was dead or just weak | Without measurements it's guesswork | `visor_motor_check.py` reads the status registers and test-moves the joint |
| Jerky movement and drift when the stick was centred | Raw stick noise | Dead-zone plus squared response curve, and a SLOW mode |
| Dangerous if the controller disconnected | The last command would keep running | The arm holds its position until the controller reconnects |
| Console filled with step-clamp warnings | LeRobot logs every time it clamps a step | Logging level for `lerobot.robots.utils` set to ERROR |
| Script needs to run without a display (SSH on the Pi) | pygame expects a display | Uses SDL's `dummy` video and audio drivers |
