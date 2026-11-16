# RoArm-M3 ST3215 Parameter FSM integration

Add `st3215_parameter_fsm.h/.cpp` to `main/` (main/CMakeLists.txt uses SRC_DIRS ".", so auto-compiles).

In `main/motor_interface.h`, add:

```cpp
#include "st3215_parameter_fsm.h"
// MotorInterface private:
ST3215ParameterFSM parameter_fsm_{};
```

In `MotorInterface::Initialize()` after `uart_num_ = board.servo_uart;` add:

```cpp
parameter_fsm_.SetPort(uart_num_);
```

Only call `parameter_fsm_.Begin()/SetOperatingMode()/SetTorqueEnable()/SetTorqueLimit()` and `parameter_fsm_.Poll()` from the **same ServoTask** owning the UART. Never call the configuration FSM concurrently with `ProcessCommunicationFSM()`. The MotorInterface INIT scheduler must be built to sequence 7 devices before RUNNING. Do not set an unverified default operating mode or auto-enable torque at startup.

Example inside INIT *once a servo configuration request has been enqueued*:

```cpp
const auto result = parameter_fsm_.Poll();
if (result == ST3215ParameterFSM::Result::FAILED) {
    ESP_LOGE(TAG, "Parameter validation failed: ID=%u err=%d",
             parameter_fsm_.GetServoID(), parameter_fsm_.GetError());
    EnterError();
}
// For SUCCESS, move to the next configuration operation.
```

To start a particular setting from ServoTask:

```cpp
const esp_err_t started = parameter_fsm_.SetOperatingMode(
    id, ST3215ParameterFSM::Mode::POSITION);
// ESP_OK means accepted, NOT finished. Keep calling Poll() each Task tick.
```

**Safety / scope:** This is the *working parameter transaction engine*, not a complete automatic initialization policy. No write occurs unless explicitly queued. The caller must establish servo supported mode, torque-safe state and a 7-servo initialization profile before choosing settings. Position offset, ID, baud are intentionally excluded. EEPROM mode changes check torque disabled; they never switch torque off automatically. READ BACK validates values, but does not prove physical motor movement. A failed lock attempt is FAILED and requires manual recovery. No real servo/ESP-IDF test was performed.

Current GitHub `main` as seen during this task is still 6 motors and IDs 1..6; user's newer 7-servo and FSM changes are local/not visible on the remote. Merge into the user's local 7-servo version, do not overwrite that file with remote sources.
