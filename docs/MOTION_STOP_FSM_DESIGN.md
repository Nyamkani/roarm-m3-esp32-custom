# Motion / Stop FSM design (staged implementation)

## Scope and status

This document proposes a two-layer control model for RoArm-M3. It is **not** a safety-certified emergency-stop implementation.

Implemented on this branch (Phase 1):
- Preserve existing servo FSM and manufacturer-derived PARKING output path.
- Track one read-only PARK operation lifecycle: NONE/IDLE, PARK/RUNNING, PARK/COMPLETED, PARK/INTERRUPTED, PARK/FAILED.
- Expose `motion_state`, `operation`, `operation_status` in `T=211`.
- Mark PARK completed only after DISARMING verifies torque OFF on all seven servos.
- No new motion, safety thresholds, pause/resume, homing, or torque behavior.

## Layer ownership

1. **Servo FSM (existing):** INIT, BOOT_RECOVERY, DISARMED, ARMING, ARMED, PARKING, DISARMING, ERROR. Single ServoTask owns UART1 and ST3215 parameter transactions.
2. **Motion FSM (planned):** IDLE, MOVING, HOMING, PARKING, PAUSING, PAUSED, PROTECTIVE_STOP. Layer must not directly access UART1; hand off requests to ServoTask.
3. **Operation result (Phase 1 PARK only):** operation ID/name, ACCEPTED, RUNNING, COMPLETED, INTERRUPTED, FAILED, stop reason. Current Phase 1 omits operation ID and stop reason; command `accepted` is not physical completion.

Motor state is authoritative for torque/communication. Motion state never proves that physical torque was removed.

## Stop semantics to implement after dedicated hardware review

- **Temporary Stop:** reject new motion, cancel outstanding trajectory, request controlled deceleration and hold current position with Servo ON, verify stopped, then PAUSED; resume via explicit command from fresh actual position.
- **Protective Stop:** block new motion and latch a fault reason; use controlled stop/hold only while sensors and actuator command path remain trustworthy. No automatic resume.
- **Software emergency request:** distinguish from hardware e-stop. Existing `T=0` asks ServoTask to disable torque without Parking; it does not cut physical power synchronously or guarantee a safe mechanical outcome.
- **Hardware e-stop:** separately audit wiring, electrical isolation, gravity/drop hazards, and whether a brake/support is required. Do not claim software stop equals certified e-stop.
- **Servo ON surveillance:** observe unexpected movement, tracking error, feedback freshness; thresholds, confirmation window, and escalation actions require physical measurement and hazard analysis.

## Design invariants

- Never use automatic servo torque OFF as a universal response to unexpected movement: gravity may worsen the hazard.
- Never replay stale targets on resume or servo re-enable.
- No homing or calibration automatically at boot or Servo ON.
- Homing is an explicit motion operation; `CalibrationOfs()` is a separate controlled calibration operation.
- Existing Parking must not change before independent regression and mechanical safety checks.
- Retained operation results are historical; `PARK/COMPLETED` does not mean current Servo ON/OFF state (consult servo FSM).
- `ERROR` does not prove physical torque OFF.
- For future pause implementation define how ST3215 in-flight position commands are cancelled safely before enabling motion.

## Phase 1 verification checklist (not yet physically tested)

1. After boot before first PARK: `T=211` reports `operation=NONE`, `operation_status=IDLE`.
2. During PARK: `PARK/RUNNING` while `state=PARKING`.
3. After all seven torque-off verifications: `PARK/COMPLETED` and `state=DISARMED`.
4. On explicit no-park OFF during PARK: `PARK/INTERRUPTED`, then torque-off status checked separately.
5. On PARK timeout or servo error: `PARK/FAILED`, with servo FSM ERROR treated as potentially energized.
6. Re-arm after a completed PARK does not erase historical result until another operation begins.
7. Verify `T=211` original fields, serial output, boot behavior and Parking unchanged.

## Later implementation phases

Phase 2: define HOMING with joint ordering, coupling checks, and no implicit zero-offset changes.
Phase 3: PAUSE/RESUME with controlled stop and fresh-target restart.
Phase 4: protective-stop monitoring and explicit operator reset.
Phase 5: hardware/software e-stop integration review, fault-injection and timing measurements.
