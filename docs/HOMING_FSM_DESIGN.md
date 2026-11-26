# Homing FSM — preflight and staged execution plan

## Current implementation (Phase H0, read-only)

`T=216` requests a **Home preflight**, not a Home movement. It is accepted by the JSON-line receive handler only as a read-only query. No Servo Torque, motion target, or calibration register is written.

Prerequisites:
- Motor/Servo FSM `ARMED`;
- fresh and valid all-seven servo feedback;
- every measured servo position is 0..4095;
- every measured velocity magnitude <= 50 (raw servo units);
- ID12 + ID13 is within 64 counts of 4094 (coupling check).

Response on success:

```json
{"T":216,"ok":true,"dry_run":true,"servo_write_enabled":false,"calibration_enabled":false,"home_goals":[2047,2047,null,2047,2047,2047,2047]}
```

Response on preflight failure:

```json
{"T":216,"ok":false,"error":"home_preflight_failed"}
```

Array order: ID11 Base, ID12 Shoulder Driving, ID13 Shoulder Driven, ID14 Elbow, ID15 Wrist, ID16 Roll, ID17 Gripper. `null` at ID13 means no active position command to that servo during the proposed release/relocation stage. It does **not** mean the servo's actual desired final position is undefined; the desired coupled center is 2047+2047=4094. This preview does not validate movement clearance, end-of-motion stability, payload, or servo load.

## Factory source reference

Factory `RoArmM3_moveInit()` sequence:
1. Base to midpoint.
2. Disable ID13 torque; move ID12 toward midpoint; wait within ±120 counts; wait ~1.2 seconds.
3. Call `setMiddlePos(ID13)` / `CalibrationOfs(ID13)` and re-enable ID13 torque.
4. Move Elbow to midpoint, wait ±80; move Wrist, Roll and Gripper toward midpoint.
5. Delay ~1 second.

**Our project decision:** **never perform `CalibrationOfs()` during Home**. Calibration must be a separate explicit function and must undergo its own safety review.

## Proposed Home operation FSM (NOT YET IMPLEMENTED)

- `IDLE -> HOME_PREFLIGHT -> HOME_BASE -> HOME_SHOULDER_RELEASE -> HOME_SHOULDER_MOVE -> HOME_SHOULDER_SETTLE -> HOME_SHOULDER_GOAL_SYNC -> HOME_SHOULDER_RESTORE -> HOME_ELBOW -> HOME_WRIST_ROLL -> HOME_GRIPPER -> HOME_VERIFY -> COMPLETED`.
- Execution should be accepted only in `ARMED` and when hardware output gates permit physical motion. No automatic execution at boot or Servo ON.
- ID13 torque release must occur only after fresh feedback and coupling checks. While ID13 is released, exclude ID13 from SYNC_WRITE. Before re-enabling, write ID13 GOAL_POSITION equal to its *fresh actual position*; verify GOAL and PRESENT, then enable torque and verify.
- Unlike Parking, Home must **stay ARMED** after completion, not transition through `DISARMING`.
- Result reporting should be `HOME/RUNNING`, `HOME/COMPLETED`, `HOME/INTERRUPTED`, `HOME/FAILED`.
- Verify each stage with finite timeout, stable velocity/position counts, joint limits, feedback age, communication error criteria, and physical bench tests. Stop/interrupt actions require explicit safety policy; simply cutting torque under load can cause a drop.
- Keep the existing Parking code/targets unchanged.

## Open design questions before physical Home

1. With `CalibrationOfs(ID13)` disabled, is the existing ID13 origin mechanically aligned with the true center when ID12 reaches 2047? Read actual ID13 and compare against 2047, and avoid applying an aggressive correction.
2. The factory waits for the driving shoulder within ±120 counts; the custom project uses much tighter Parking limits. Determine acceptable Home completion tolerance based on measurements.
3. Does shoulder geometry and arm payload permit ID13 torque release while Home moves? Use mechanical support for supervised tests.
4. Define total and per-stage timeouts, acceleration/step size and limit checks for the custom controller.
5. Position loop status vs operation outcome must stay separate from physical stop/torque state.
6. Home preview in Phase H0 is deliberately read-only; a passing preview is not authorization to run the physical motion.

## Verification (not yet performed)

- Build on ESP-IDF toolchain; static/runtime checks.
- Under `DISARMED`, `T=216` should fail with `home_preflight_failed` (no servo writes).
- Under `ARMED`, stable, coupled feedback should yield `dry_run:true` and goals above.
- While feedback is stale/moving or shoulder is inconsistent, it should reject.
- Repeat existing Servo ON / Parking / T211 regression; no output code changed in H0.
