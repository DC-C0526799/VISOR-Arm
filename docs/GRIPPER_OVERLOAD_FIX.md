# Gripper Overload Error – Problem, Diagnosis and Fix

**Component:** SO-101 follower arm, gripper servo (Feetech STS3215)
**Script:** [`act_policy/guarded_act_rollout.py`](../act_policy/guarded_act_rollout.py)
**Status:** Fixed. Verified working on 12 Nov 2026 and demonstrated on 13 Nov 2026.

---

## 1. The problem

During ACT rollouts, the arm would reach a plastic bottle or aluminium can and close the gripper on it. A few seconds after the jaw made contact, the gripper servo **shut its torque off and reported a hardware fault**. The LeRobot rollout then stopped, and the arm dropped or never lifted the object.

The fault only happened **when the gripper was holding something**. Closing on empty air worked every time.

## 2. Finding the cause

### Step 1 – Read the servo's own fault flag
We read the gripper's `Status` register after each failure instead of guessing. Every failure showed the same flags:

| Status bit | Meaning | Seen? |
|---|---|---|
| bit 3 (`0x08`) | Over-current | Yes |
| bit 5 (`0x20`) | Overload | Yes |
| bit 0 / 1 / 2 | Voltage / sensor / temperature | No |

So the servo was not overheating and the supply was fine. It was **protecting itself from too much load**. That's why the guard treats `0x28` (bits 3 and 5) as the only recoverable fault mask.

### Step 2 – Check the STS3215 overload rule
According to the Feetech documentation, the STS3215 trips its overload protection when the load stays above its `Overload_Torque` threshold for about **2 seconds**. Once it trips, the servo disables its torque until the flag is cleared.

### Step 3 – Work out why ACT was overloading it
The servo runs in **position mode**. ACT outputs a gripper *position* target, not a force.
The demonstrations were recorded with the leader arm, where the operator squeezes the handle fully closed. ACT learned that, so it commands a jaw position **tighter than the object is wide**.

- With nothing in the gripper, the jaw reaches that position and the load drops back to near zero.
- With a bottle or can in the gripper, the jaw **can never reach the target**. The position controller keeps pushing harder, the load stays high for more than 2 s, and the overload protection trips.

**Root cause:** a position-controlled gripper was commanded to a position it could not physically reach, while the torque limit was still above the overload threshold. There was no feedback telling the controller to stop squeezing once it had a grip.

### Step 4 – Watch it live
We added a trace mode (`GRIPPER_TRACE=1`) that prints the jaw position, the ACT target, the command actually sent, and `Present_Load` twice a second. This confirmed the pattern on contact: **the jaw stops moving, the ACT target keeps asking for tighter, and the load climbs and stays high.** That pattern became the contact-detection rule in the fix.

### What did not work
Quick, untested patches to the rollout code broke the script without solving the problem. We went back to the standard LeRobot rollout and added the fix as a **separate wrapper**. ACT and LeRobot themselves are unchanged.

## 3. The fix – a feedback guard on the gripper

`guarded_act_rollout.py` wraps the normal `lerobot-rollout` command. When `GRIPPER_GUARD=1` is set, it patches the SO-101 follower's `configure()` and `send_action()` so every gripper command passes through a guard first. The guard has five layers:

1. **Torque cap below the overload limit.** At startup it writes a lower `Torque_Limit` to the gripper. It also checks that `contact_load < torque_limit < Overload_Torque`, so the motor physically cannot squeeze hard enough to trip. This is a volatile setting, so the servo's own protection is left untouched.
2. **Contact → hold, using position and load feedback.** While closing, the jaw moves in small steps. A grasp is only declared when all of these are true for 3 ticks in a row:
   - `Present_Load ≥ GRIPPER_CONTACT_LOAD`
   - the jaw has stopped moving
   - ACT is still asking for a clearly tighter grip

   The target is then frozen at the current position rather than pushed further. A one-tick load spike while the jaw is moving does not count.
3. **Relief.** While holding, if the load rises to 90% of the torque cap, the target backs off one small step per tick.
4. **Release.** When ACT clearly asks to open, the hold ends and ACT has normal control again.
5. **Fault recovery.** If an overload or over-current flag still appears, the guard turns torque off, re-sends the present position, and turns torque back on, which clears the flag according to the Feetech spec. The run then continues. It allows a maximum of 5 recoveries per run. Any other fault (voltage, temperature, sensor) still stops the run.

```
ACT policy ──► gripper target ──► GUARD ──► STS3215 gripper
                                   ▲              │
                                   └── position + load feedback
```

## 4. How to run it

```bash
export GRIPPER_GUARD=1
source gripper_settings.sh   # GRIPPER_CLOSE_SIGN, GRIPPER_CONTACT_LOAD,
                             # GRIPPER_TORQUE_LIMIT, GRIPPER_MAX_CLOSE_STEP
python guarded_act_rollout.py --guard-check   # checks settings, no motors moved
python guarded_act_rollout.py <normal lerobot-rollout arguments>
```

Without `GRIPPER_GUARD=1`, the script behaves exactly like a standard ACT rollout.

On the console you'll see:
- `GRIPPER GUARD: contact=..., cap=..., overload_boundary=...` at startup
- `GRIPPER CONTACT/HOLD: position=..., load=...` when an object is gripped
- `GRIPPER RELEASE: ...` when ACT opens the gripper

## 5. Result

With the guard enabled, the gripper holds bottles and cans firmly and the overload fault no longer stops the run. The ACT model and the training data didn't need to change. The guard is also used in the continuous sorting loop (`robot_control/visor_stage3_sort_v2.py`).

## 6. Lessons learned

- **Read the hardware's fault register first.** The `Status` flags pointed straight to overload and ruled out power and temperature.
- **Imitation-learned position targets don't know about force.** A position-mode gripper needs force or load feedback when it grips rigid objects.
- **Wrap, don't edit, the framework.** Keeping LeRobot unchanged and adding a separate guard made the fix easy to test, and easy to switch off.
