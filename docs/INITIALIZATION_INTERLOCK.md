# Initialization / MOVE interlock — phase 1

Status: implemented in source; build and hardware smoke test of this change are pending.

## Approved architecture

An SBC-KCF request may initiate supervised initialization later. The ESP32 independently decides whether a MOVE is permitted.
Ordinary ARM must not trigger HOME or calibration. Boot must not start physical motion.
The intended path is: read-only boot diagnostics -> authorized HOME -> verified mechanical calibration -> PARK (where safe) -> DISARMED and initialized -> ordinary ARM -> MOVE.

## Implemented

- MotorInterface has a fail-closed initialization_complete flag, initially false.
- CanExecuteMove() requires both ARMED and initialization complete.
- JSON T=102 and T=1041 are rejected if CanExecuteMove() is false.
- Read-only T=211 includes initialization_complete and move_allowed.
- No function currently sets initialized true: calibration and the initialization coordinator are not implemented.
- Existing ARM/PARK/bench HOME gates remain independent; POSITION mode still rejects general motion commands.
- HOME execution remains disabled by default; no EEPROM calibration writes.

## Tests

Build: bash tools/home_build_check.sh
After flashing the latest firmware, when the robot is not moving:
python3 tools/init_readonly_smoke.py --port /dev/ttyUSB0 --expect-state DISARMED
Or with an already safely ARMED robot: change expect-state to ARMED.
Expected: initialization_complete=false, move_allowed=false, PASS.
The script sends T=211 only. Opening serial can reset some ESP32 boards.
The status check alone cannot prove the physical command paths are blocked; further integration tests are required.

## Future

Implement real calibration reference validation and initialization authorization; define persistence, invalidation, emergency behavior and recovery before enabling MOVE.
Never treat HOME/COMPLETED or PARK/COMPLETED alone as proof of calibration validity.
