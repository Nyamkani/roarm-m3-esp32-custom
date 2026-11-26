## ST3215 UART parameter transaction synchronization (experimental)

After three consecutive PARK -> HOME attempts, the second run showed
malformed response headers during all-servo Torque OFF, often with a
leading byte matching the checksum of a previous WRITE status frame.
Another run completed. The exact physical cause remains unproven.

The parameter FSM now:

- For Return Level 1 writes, waits at least 4 ms, drains any received
  optional/delayed WRITE-status bytes, and requires at least 1.5 ms of
  receive quiet; total drain is bounded at 12 ms. This is **not** a
  guarantee that arbitrarily late ACKs cannot occur.
- Increases parameter READ response timeout from 3 ms to 5 ms.
- Permits a single read-only retry for a malformed/timed-out initial
  parameter READ. Existing RAM post-WRITE readback retry remains one.
- Never repeats a Torque/Goal WRITE on these retries and never bypasses
  the returned register value comparison.
- Enters ERROR when the bounded drain / retries still fail.

The UART parameter FSM remains a **single-owner, fail-closed** design.
All HOME goals, Parking targets, and movement FSM ordering remain
unchanged. No build or hardware run has been verified for this change.
A supported arm and physical supply isolation are required before any
further powered testing.

---

## Supervised PARK -> FULL HOME physical test configuration (temporary)

The `feature/homing-fsm-gated` branch now has
`MOTOR_VENDOR_SHOULDER_PARK_ENABLE=1` and
`MOTOR_HOME_ALLOW_FULL_EXECUTION=1` for a **supervised acceptance test only**.
The test firmware must not be reused for general autonomous homing.

Preconditions and order:

1. Build and run the offline `tools/home_reference_source_check.py`.
2. Mechanically support the arm, ensure the physical supply can be isolated,
   and verify all seven torque states after previous ERROR/BOOT_RECOVERY.
3. Only after confirming a stable current position and restored feedback,
   explicitly ARM, then invoke PARK using the existing JSON command.
   PARK succeeds **only** if its final all-servo OFF verification succeeds.
4. Explicitly ARM at PARK, inspect fresh raw encoders, then request FULL HOME.
   The HOME PREPARE phase rejects any servo not within 40 raw counts of
   its expected PARK pose. Shoulder movement checks ID12 in
   [PARK 1023 - 30, HOME 1703 + 30] and ID13 in
   [HOME 2386 - 30, coupled PARK 3071 + 30] each valid feedback cycle.
5. Read T211/T221/T229 during a supervised run. Stop the test on unexpected
   movement or torque uncertainty. ERROR inhibits writes but does NOT
   guarantee motor torque OFF or brake the moving mechanism.

The ST3215 parameter FSM now retries a failed RAM post-WRITE **READ**
once; it does not replay the WRITE or bypass register equality checks.
This mitigates some transient errors, but the root cause of the earlier
ID11 `ESP_ERR_INVALID_RESPONSE` is **not yet proven**, and the fix
has not been validated on hardware.

These numeric limits are scoped to the verified PARK-to-HOME experiment,
not independently established mechanical travel limits. The ServoTask
cannot guarantee collision prevention; it only observes a later
feedback sample. Do not run unsupported or near mechanical hard stops.

---

## Verified boot scan before parking / homing tests

During `INIT`, firmware now reads the operating mode and Torque Enable
register of every servo ID11 through ID17 **twice**, without issuing
Torque/Goal writes. The two passes must agree. A communication failure or
inconsistent pass enters `ERROR`. This intentionally adds startup time.

Use `{"T":230}` to inspect the last verified boot snapshot:

- `scan_verified`: both seven-servo read passes succeeded and matched.
- `torque_on_mask`: bit0=ID11, bit1=ID12, ..., bit6=ID17 (1 means Torque ON at boot scan).
- `mode_mismatch_mask`: same bit mapping, 1 means unexpected operating mode.
- `torque_state_current:false`: the snapshot is historical; it cannot certify live torque.

If any torque remains ON, `BOOT_RECOVERY` is retained; **there is no
automatic torque OFF or servo movement at boot**. With the arm mechanically
supported, the operator can use the existing explicit recovery path, which
currently turns OFF *all seven* servos and verifies readback. It does NOT
yet selectively repair individual servo settings. Servo movement must not
resume just because `scan_verified` is true.

The earlier PARK -> DISARMING failure at ID11 was
`ESP_ERR_INVALID_RESPONSE`, so the torque state was not verified.
The separate ST3215 error diagnostics now distinguish malformed packets
from register readback mismatch. This boot scan is an additional diagnostic,
not a fix for that UART / torque verification failure.

---

## Confirmed mechanical HOME raw pose (authoritative)

The operator explicitly confirmed the desired physical HOME pose was captured
with seven ST3215 raw encoders:

| Servo | Role | HOME reference raw |
|---|---|---:|
| 11 | Base | 2043 |
| 12 | Shoulder driving | 1703 |
| 13 | Shoulder driven | 2386 |
| 14 | Elbow | 2927 |
| 15 | Wrist | 1549 |
| 16 | Roll | 2048 |
| 17 | Gripper | 2046 |

**This is NOT the manufacturer PARK pose.** Previous examples in this
document that identify all-2047 as HOME are superseded by this section.

Active `HOME_GOALS` are `[2043,1703,-1,2927,1549,2048,2046]`;
ID13's raw 2386 is preserved as a *reference and post-movement
verification*, never sent as an active command while its torque is OFF.
Before ID13 servo goal/torque restore, both ID12 (1703 ±10) and ID13
(2386 ±20) must be present with fresh feedback, stable ID13 velocity
and shoulder coupling in tolerance. The 2386 reference does not imply
that mechanical safety is established throughout the path.

The full HOME and vendor shoulder PARK gates are both kept disabled in
the repository baseline after an observed MOVE_SHOULDER timeout and
unexpected near-vertical arm pose. Do not re-enable from this documentation
alone. Revalidate path, support and restore procedure before powered tests.

---

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

## Phase H1 — gated Homing execution (source implemented; NOT physically validated)

- New action command: `{"T":217,"cmd":1}`.
- Default `MOTOR_HOME_EXECUTION_ENABLE=0`. With the default build this action returns `{"ok":false,"status":"motor_unavailable"}` and must **not move**.
- When intentionally enabled in a separately supervised firmware build: require `ARMED` and passing Home preflight, then queue HOMING for the ServoTask. Immediate `accepted` does not mean complete.
- The ServoTask stages are BASE → ID13 Servo OFF → ID12 SHOULDER → nonblocking 1.2s settling → ID13 GOAL=actual, GOAL/PRESENT verification → ID13 Servo ON → ELBOW → WRIST → ROLL → GRIPPER → capture final actual positions as new HOLD → return to `ARMED`.
- No `CalibrationOfs()` or EEPROM zero adjustment.
- Under HOMING, `T=211` publishes `state=HOMING`, `motion_state=HOMING`, `operation=HOME`, `operation_status=RUNNING`. On success `state=ARMED`, `HOME/COMPLETED`.
- Home positions (except passive ID13) are 2047 counts. Goal stepping uses 3 counts per approximately 10ms cycle, stage stability ±10 counts, velocity ±50 for ten feedback cycles. Watchdogs: 90s **per moving joint**, 5s **per nonmoving parameter phase**, 3s **shoulder settling**, with an additional 8-minute hard total cap. Timer resets when Home phase or individual MOVE_REST joint changes. These limits have **not** been validated for Home on hardware.
- `T=0`/explicit no-Parking Servo OFF can interrupt HOMING, causing `HOME/INTERRUPTED`. It is **not** a safety-rated stopping method; the arm can fall. No general Pause or protective-stop policy has been implemented.
- Failure during a partial shoulder-release state can leave physical Servo torque unknown. **Do not attempt active Home on an unsupported arm**. Review error recovery and mechanical support before enabling the gate.
- Servo ON and Parking implementation, including their default safety gates, were not intentionally changed by this feature.

## T=211 Home phase diagnostics (read-only)

- Added `home_phase` (string) and `home_phase_code` (integer) to the existing T211 status response.
- Initial values: `NOT_STARTED` and `0`. Values are retained after failure or interruption as last recorded step; `COMPLETED` (11) is recorded on successful HOME before returning to ARMED. An active HOME has `state=HOMING`.
- Codes: 1 PREPARE, 2 MOVE_BASE, 3 RELEASE_SHOULDER, 4 MOVE_SHOULDER, 5 SETTLE_SHOULDER, 6 SET_SHOULDER_GOAL, 7 VERIFY_SHOULDER_GOAL, 8 VERIFY_SHOULDER_POSITION, 9 RESTORE_SHOULDER, 10 MOVE_ELBOW/MOVE_WRIST/MOVE_ROLL/MOVE_GRIPPER, 11 COMPLETED.
- The snapshot is published at the start of each ServoTask Home iteration; phases can briefly lag an internal transition by one task iteration. When HOME is disabled, the phase remains NOT_STARTED unless an earlier HOME actually ran.
- These diagnostics provide **software bookkeeping only**: neither a stage transition nor `COMPLETED` proves mechanical safety or live servo torque.

## Safety review — ID13 recovery and interrupted HOME

The last confirmed ID13 torque command state is now tracked as `home_id13_last_verified` in `T=211`: `UNKNOWN`, `OFF`, or `ON`. This is **historical confirmation from the parameter FSM, not a real-time sensor reading**. `UNKNOWN` is the default at boot and at each new HOME. On successful ID13 release verification it becomes `OFF`; on successful restore verification it becomes `ON`. In a partial or failed transaction, do **not** infer actual state from the last value.

- HOMING no-park OFF interruptions inhibit position SYNC_WRITE while waiting for the ongoing UART1 transaction to finish. After communications become IDLE, the existing DISARMING path attempts OFF for all seven servos.
- Failure to finish a parameter transaction (including timeout/comm error) transitions to ERROR with output inhibited. It **does not automatically clear torque or guarantee ID13 re-enable**. Manually support the arm, identify which physical torques remain enabled, then follow a separately approved recovery procedure.
- `T=0` and `T=215` are not safety-rated emergency-stop mechanisms. In software ERROR, requests may not trigger a physical torque change.
- A passed HOME preflight does not establish that lifting/releasing ID13 is safe with payload; ensure physical support and supervise first tests.
- **Feature flag stays at `MOTOR_HOME_EXECUTION_ENABLE=0`.** No physical Home test is authorized by this document. Build and regression validation remain outstanding.

## Factory source reference

Factory `RoArmM3_moveInit()` sequence:
1. Base to midpoint.
2. Disable ID13 torque; move ID12 toward midpoint; wait within ±120 counts; wait ~1.2 seconds.
3. Call `setMiddlePos(ID13)` / `CalibrationOfs(ID13)` and re-enable ID13 torque.
4. Move Elbow to midpoint, wait ±80; move Wrist, Roll and Gripper toward midpoint.
5. Delay ~1 second.

**Our project decision:** **never perform `CalibrationOfs()` during Home**. Calibration must be a separate explicit function and must undergo its own safety review.

## Execution plan (H1 code exists behind build-time gate; pending build and physical validation)

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

## 2026-11-27: Joint limits / fault-hold preparation

- `main/home_safety_config.h` provides configurable per-servo raw position
  intervals with `configured` and `mechanically_verified` flags.
  All limits start unconfigured. `MOTOR_HOME_DEVELOPMENT_MODE=1` bypasses
  **only these missing mechanical limits**; raw validity, feedback age,
  coupling, tracking, and HOME execution gates remain enforced.
  For deployment set `MOTOR_HOME_DEVELOPMENT_MODE=0` and establish actual
  verified physical limits before enabling any HOME operation.
  No inferred 2047/2048 physical limits are adopted.
- Homing preview, PREPARE, and Homing moving goal/feedback checks now use
  the configurable limit predicate. This has not been tested on hardware.
- `MOTOR_FAULT_HOLD_OUTPUT_ENABLE=0` is deliberately enforced by a
  compile-time error if set to 1. **No active Current Position Hold on fault
  has been implemented.** The reason: in the current ERROR path the UART
  can be unreliable; ID13 can be torque-off; a goal overwrite alone is not a
  verified physical hold and might generate an unsafe jump.
- A historical HOMING ERROR diagnostic is latched before feedback
  invalidation. The read-only `T=222` response includes
  `development_mode`, `joint_limits_verified`,
  `fault_hold_output_enabled`, `fault_recorded`,
  `feedback_fresh_at_fault`, `id13_last_verified_off_at_fault`,
  `fault_home_phase_code`, and `fault_id13_last_verified_code`.
  Freshness reflects only the last captured snapshot, not a valid live
  command channel or proof that recovery is safe.
- Existing T=211 status fields, ordinary ARM/PARK behavior, and the
  `MOTOR_HOME_EXECUTION_ENABLE=0` safety gate remain unchanged.
- Local build and runtime checks have not yet been performed for this set
  of modifications. After a VS Code ESP-IDF build/flash, with arm stationary:
  `python3 tools/home_safety_readonly_smoke.py --port /dev/ttyUSB0 --expect-state DISARMED`
  (sends T211 and T222 only).

Before implementing active fault hold: define UART transaction ownership
during ERROR, fresh feedback reacquisition, the ID13 torque-off special case,
hold goal readback, post-write verification, and fallback without accidental
torque release. Keep ERROR latched and refuse normal MOVE throughout.

## Supervised hardware test: explicit in-place CURRENT POSITION Hold (T223/T224)

This test is **physically active**, but intentionally does NOT inject an
error, drive to a new position, enable Homing or release shoulder ID13 torque.

- Precondition: manually supervised, mechanically supported stationary arm
  **already in ARMED**, with fresh feedback for all 7 servos, raw values in
  0..4095, velocities within ±50, coupled shoulder sum within 4094±64.
  Opening a USB serial port can reset the MCU and change its servo state.
- `{"T":223,"cmd":1}` queues a current-pose recapture. The ServoTask
  performs the checks and replaces `hold_command_` with the seven *actual*
  raw positions. Existing ARMED cycle then sends the new goals using
  the already enabled ST3215 SYNC_WRITE position path.
- `{"T":224}` reads `status` and `targets`. Status sequence:
  IDLE → PENDING → CAPTURED → WRITE_SENT → FEEDBACK_STABLE.
  Ten post-write feedback cycles must be within 10 raw counts of goals and
  at most 50 raw velocity units. A successful UART transmit is not proof
  of physical torque or safety-rated motion cessation. No command EEPROM
  or mechanical calibration changes occur.
- Command stays unavailable in DISARMED, HOMING, PARKING or ERROR;
  it does not enable ordinary MOVE or circumvent initialization interlocks.
- Run, after VS Code ESP-IDF extension build/flash and independently
  supervised ARM, with physical arm stable:
  `python3 tools/current_hold_hardware_smoke.py --port /dev/ttyUSB0`.
  The script refuses to issue T223 unless T211 reports ARMED.
- `MOTOR_HOME_EXECUTION_ENABLE=0` and `MOTOR_FAULT_HOLD_OUTPUT_ENABLE=0`
  remain unchanged. **Actual Hold during ERROR and ID13 torque-off
  recovery remain unimplemented and unverified.** Never deliberately
  break UART or unplug an active loaded arm to simulate an error.

## Supervised FAULT_HOLD bench step — T225/T226/T227

This is a **real motor-output, software-simulated fault-latch test**, not
an injected UART failure and not an automatic reaction to every HOME error.
Never use on an unsupported, moving or unobserved arm.

- With all seven servos already torque ON and `state=ARMED`, stationary,
  fresh valid feedback and shoulder pair within 4094±64, `{"T":225,"cmd":1}`
  asks the ServoTask to capture all seven CURRENT encoder positions and
  transition to the dedicated `FAULT_HOLD` state. Other movement
  commands are rejected by the state gate. The normal ServoTask UART1
  feedback + SYNC_WRITE pipeline continues sending these captured goals.
- `{"T":227}` exposes the historical goal set and
  `REQUESTED → WRITE_SENT → STABLE` result. STABLE needs ten consecutive
  feedback cycles within ±10 raw counts and ±50 raw velocity. This is NOT
  proof of zero motion under load and is not a safety-rated emergency stop.
- `{"T":226,"cmd":1}` explicitly returns a successful *bench-only* hold
  to ARMED. The `tools/fault_hold_bench_smoke.py` utility leaves the arm
  in FAULT_HOLD by default; `--resume` requests an explicit return after
  STABLE. No automatic resume is performed.
- `T=0` remains an explicit, potentially gravity-dangerous all-servo
  torque release; it is NOT a safety-rated stop. Do not run this test
  without a mechanical support and a planned recovery procedure.
- **Real HOME failure behavior remains unchanged**: fatal communication,
  invalid feedback and uncertain ID13 torque still enter ERROR, without
  claiming a successful Hold. `MOTOR_HOME_EXECUTION_ENABLE=0`,
  `MOTOR_FAULT_HOLD_OUTPUT_ENABLE=0`, calibration false and normal MOVE
  blocked are still intact.
- Verify firmware build/flash using VS Code ESP-IDF extension first;
  source changes have not been built or hardware tested by assistant.
  With supervised `ARMED`: `python3 tools/fault_hold_bench_smoke.py
  --port /dev/ttyUSB0`.

## 2026-11-27 supervised staged physical HOME: BASE_ONLY first

The development branch currently uses:
- `MOTOR_HOME_EXECUTION_ENABLE=1`: deliberate supervised physical BASE_ONLY test build.
- `MOTOR_HOME_COMPILE_CHECK=0`: this is no longer compile-only.
- `MOTOR_HOME_ALLOW_FULL_EXECUTION=0`: **FULL HOME / ID13 TORQUE OFF
  REMAINS BLOCKED** despite the BASE_ONLY execution gate. Do NOT change
  this option without independent verification of shoulder release under
  physical load and an external mechanical support.
- `MOTOR_HOME_DEVELOPMENT_MODE=1`: no verified joint angle envelopes yet.
  This is NOT evidence that the mechanical route is clear.

### Commands and diagnostics

- `{"T":217,"cmd":1}`: arm must already be ARMED and pass the full
  read-only preflight; run BASE_ONLY HOME toward ID11 raw 2047.
  ID12/13 and all other joints keep their captured current position
  (other joints may still receive hold command targets).
  Completion restores ARMED using the measured hold pose. No ID13
  torque-off, goal calibration, or servo EEPROM write.
- `{"T":217,"cmd":2}`: FULL HOME request, DENIED in this build.
  A future explicit separate FULL flag is required for this dangerous path.
- `{"T":228,"cmd":1}`: while HOMING, request current-position FAULT_HOLD
  without a torque release. ServoTask avoids new moving HOME goal
  writes while draining an in-progress UART cycle. Success requires
  complete valid feedback, safe bus ownership and historically verified
  ID13 torque ON. If this cannot be established, ERROR can result.
  This is **not** an emergency stop or a guarantee of physical hold.
- `{"T":229}`: read-only phase/stage, HOME targets, operation outcome
  and fault hold state; `T211`, `T221`, `T222`, `T227` remain usable.
- Deterministic HOME logic faults (tracking error, out-of-bounds raw
  target/feedback and stage timeout) attempt FAULT_HOLD only when UART
  is idle/PROCESS with fresh valid feedback, and ID13 was verified ON.
  Coupling divergence fails closed to ERROR. Actual UART faults and
  release/restore-shoulder failures still go ERROR, never claim hold.

### One-pass supervised integration test

Build/flash via ESP-IDF VS Code extension first. Mechanically support the
arm, clear potential interference and confirm position before torque ON.
Run prior DISARMED/ARMED integration regression as desired.
In an independently verified stable ARMED state:

`python3 tools/home_base_hardware_smoke.py --port /dev/ttyUSB0`

It checks that T217 FULL is rejected, performs the BASE_ONLY physical
move, monitors T211/T229 and verifies ID11 settled near 2047. It
attempts explicit T228 Hold abort on monitoring timeout without torque
release. **Never run this on an unobserved/unsupported arm.**
No physical build or test results have been observed for this commit.

Before FULL HOME is authorized: mechanically confirm the shoulder-release
route and loads, review ID13 readback before restore, external support,
physical joint limits/clearances, and failure behavior during param FSM.
