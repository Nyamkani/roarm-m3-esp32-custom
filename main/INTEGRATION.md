# RoArm-M3 ST3215 Parameter FSM integration (v0.12 fixes)

- `MotorInterface::Initialize()` assigns `parameter_fsm_.SetPort(uart_num_)` before creating ServoTask.
- `MotorInterface::StateInit()` remains in INIT until the scheduler reaches COMPLETE. Any parameter failure enters ERROR.
- `ProcessMotorInitialization()` iterates servo IDs **11–17** using one UART owner (ServoTask). Each mode check runs through `ST3215ParameterFSM::Poll()`.
- `MOTOR_EEPROM_WRITE_ENABLE=0` (default) calls `VerifyOperatingMode(id, PWM_OPEN)`, which only READs and compares. A mismatch fails INIT; it does **not** silently advance or change EEPROM.
- `MOTOR_EEPROM_WRITE_ENABLE=1` calls `SetOperatingMode(id, PWM_OPEN)`; the existing transaction engine checks torque-off, unlocks EEPROM, writes, reads back, and relocks. Only enable this after verifying arm support/safety.
- `MOTOR_INIT_TORQUE_LIMIT_ENABLE=0` (default) leaves the current RAM torque limit unchanged. If enabled, `MOTOR_INIT_TORQUE_LIMIT` is applied with WRITE / READ BACK. Choose its value for the actual mechanism first.
- No automatic torque enable, no HOME move, and `MOTOR_PWM_OUTPUT_ENABLE=0` still prevents PWM output.
- MODE EEPROM write permission and motor output permission are independent.
- `ST3215::MOTOR_COUNT` and `MotorInterface::MOTOR_COUNT` are both 7, so SYNC_WRITE accepts the seven IDs.

## Verification boundaries

This is a **source-level integration**, not proof of successful ESP-IDF compilation or physical servo behavior. With the default read-only mode policy, a factory servo in a mode other than PWM_OPEN will cause an expected INIT ERROR. Servo status return setting, timing, EEPROM behavior, and physical safety need bench validation before enabling any writes/output.
