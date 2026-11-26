# Homing FSM — Test Environment and Regression Checklist

## Scope

The purpose of this environment is **compile/link validation and read-only serial checks** of the Home FSM code while `MOTOR_HOME_EXECUTION_ENABLE=0`. This is NOT a real Home movement test and does not prove the safety of an ID13 release.

### Environment

- Host: computer with ESP-IDF v6.1 installed and activated, Python 3, `pyserial` for serial check.
- Target: ESP32-WROOM-32, `idf.py set-target esp32`.
- Firmware source branch: `feature/homing-fsm-gated`.
- JSON console: UART0 at 115200 bps, JSON objects separated by newlines.
- ST3215 bus UART1 at 1 Mbps remains managed exclusively by ServoTask.
- Make sure physical arm is mechanically supported for all powered tests. Do not open a serial port during motion: some USB serial adapters reset the ESP32 when the port is opened.

## 1. Compile-only gate check (no flash)

```bash
git checkout feature/homing-fsm-gated
git pull --ff-only
. "$HOME/esp/esp-idf/export.sh"  # adjust to local installation
bash tools/home_build_check.sh
```

The script checks the `MOTOR_HOME_EXECUTION_ENABLE=0` and `MOTOR_EEPROM_WRITE_ENABLE=0` macros, sets target `esp32`, and runs `idf.py build`. It DOES NOT flash or run physical motor commands. It does not guarantee safe behavior from other motor output build flags.

PASS: compilation/link succeeds without errors. Do not proceed to physical validation if build fails.

## 2. Manual flash of the confirmed build, ONLY when existing supervised bench setup is safe

Do not blindly flash a motor-connected arm. Check all compile flags, wiring, mechanical support, and startup recovery behavior first. When approved for bench testing:

```bash
idf.py -p /dev/ttyUSB0 flash
```

Replace device node with the one actually used. Firmware can auto-run existing servo initialization; flashing is not part of the compile-only test script.

## 3. Read-only serial smoke test

Install the host dependency (if missing):

```bash
python3 -m pip install pyserial
```

Run after the flashed firmware is known to be safe and stationary. Port opening can reset ESP32; observe safe boot behavior.

```bash
python3 tools/home_readonly_smoke.py --port /dev/ttyUSB0 --expect-state DISARMED --output home-disarmed.jsonl
```

On DISARMED the script verifies:
- `T=211` returns a known FSM state, `operation_status`, and `home_id13_last_verified`;
- `T=216` returns `home_preflight_failed`.

For a board that has **already been explicitly Servo ON under the existing supervised procedure**, and is safely stationary:

```bash
python3 tools/home_readonly_smoke.py --port /dev/ttyUSB0 --expect-state ARMED --output home-armed.jsonl
```

On ARMED the script verifies the read-only semantics of `T=216`: success with `dry_run=true`, `servo_write_enabled=false`, `calibration_enabled=false`, fixed home goals, OR a legitimate failure `home_preflight_failed` if feedback/safety prerequisites fail. A failed preflight in ARMED is **not** a PASS for mechanical readiness; investigate it separately.

**The script sends ONLY T=211 and T=216.** It does not send `T=0`, `T=210`, `T=215`, `T=217`, or any motion commands. It ignores plain ESP-IDF log lines and reads matching JSON responses.

## 4. Separate manual disabled-command check

Do not automate the T217 request while connected to a physical arm. Confirm the firmware build gate is OFF in the exact flashed binary/configuration. Only then, under supervision, manually send:

```json
{"T":217,"cmd":1}
```

Expected reply:

```json
{"ok":false,"status":"motor_unavailable"}
```

Unexpected `accepted` is a **STOP condition**: do not continue further requests until the loaded firmware is identified and the mechanism made safe.

## 5. Existing Servo ON / Parking regression (manual, separately supervised)

Only use established physical test procedure (not this script). Check `T=211`:

| Stage | Expected state | Expected operation | Expected operation_status | Other |
|---|---|---|---|---|
| After cold normal boot | DISARMED | NONE | IDLE | Not always DISARMED if BOOT_RECOVERY or ERROR |
| After Servo ON | ARMED | Last result retained | Last result retained | `torque_off_verified=false` |
| During Parking | PARKING | PARK | RUNNING | Existing movement and sequence unchanged |
| After Parking torque OFF confirmation | DISARMED | PARK | COMPLETED | `torque_off_verified=true` |
| After re-arming | ARMED | PARK | COMPLETED | Historical result retained |

`feedback_valid=false` after Parking is a known separate joint-mapping question, not automatically a transport failure. Collect raw servo feedback for diagnosis later. `T=214` preview currently uses elbow target 3071 while actual Parking uses 3039; do not use preview as a physical goal truth.

## 6. PASS criteria and unresolved limitations

- PASS build/link: ESP-IDF build succeeds for the exact tested branch.
- PASS read-only: no motion commands are sent by scripts; T211/T216 responses match schema.
- PASS regression: manually observed Servo ON/Parking sequence and status results unchanged.
- Home execution remains gated OFF.
- Not validated: physical Home, partial-error recovery, shoulder torque safety, collision envelope, actual stopping distances, hard emergency-stop wiring.
- Any unexpected motion, servo communication ERROR, or unverified ID13 torque state requires stopping the test and checking mechanical support before attempting recovery.

Artifacts to retain: commit SHA, `idf.py --version`, build output, flashed binary identifier, `T=211`/T216 JSONL output, ServoTask logs of manual regression.
