# Development Updates

RoArm-M3 ESP32 Custom Firmware 개발 기록입니다. **코드 구현**, **검증**, **향후 계획**을 구분해 기술합니다.

## v0.1 — Initial firmware structure

- **Branch:** `main`
- **Commit:** [`56a6ccf`](https://github.com/Nyamkani/roarm-m3-esp32-custom/commit/56a6ccf5ecdcbd20313164e34f468f346503aec6)
- **Commit message:** `v0.1`
- **Date:** 2026-11-16 (KST)
- **Validation:** Hardware testing not yet performed

### Implemented in source

**Board / RTOS**
- ESP-IDF / C++ 프로젝트 구성 및 ESP32 UART 초기화 코드
- FreeRTOS 기반 Servo Task 및 1ms 주기의 통신 FSM 폴링 구조

**ST3215 protocol**
- `PING`, `READ`, `WRITE`, `REG_WRITE`, `ACTION`, `SYNC_READ`, `SYNC_WRITE` 명령 패킷 직렬화 지원
- 응답 ID, 길이, 체크섬 검사 및 피드백 데이터 디코딩 구현
- 실제 6축 피드백 갱신은 개별 `READ`를 순차 실행하는 방식으로 작성
- `SYNC_READ`/`SYNC_WRITE`는 **패킷 생성 기능만 구현**, 실제 다축 통신 흐름에는 아직 연결하지 않음

**Motor communication**
- `MotorInterface`의 `INIT → READY → RUNNING / ERROR` 상태 처리
- `IDLE → SEND_REQUEST → WAIT_RESPONSE → NEXT_MOTOR → COMPLETE` 통신 FSM 및 `RX_RECOVERY` 처리
- 피드백용 `TripleSnapshot` 및 6축 전체 수신 실패 시 피드백 무효화
- 기본 설정: 10ms 주기 목표, 개별 응답 2ms 타임아웃, 6축 전체 5ms 제한, RX 복구 2ms
- 1초 내 통신 실패 3회 시 `ERROR` 상태 전환 로직

**Diagnostics**
- `MOTOR_PERF_LOG_ENABLE` 설정을 통한 1초 주기 로그
- Servo Task 실행 빈도, 정상 완료된 6축 READ 빈도, 평균/최대 사이클 시간, 오류 및 타임아웃 횟수 집계

### Current limitations

- `main.cpp`가 보드 초기화만 수행하며, `MotorInterface::Initialize()` 호출은 아직 연결되지 않음
- Command Snapshot 기반 모터 출력과 6축 `SYNC_WRITE` 실행 미구현
- ST3215 PWM 직접 제어 모드 설정, 외부 PID, 관절 목표 제어 미구현
- 통신 오류 시 PWM 0 긴급 정지 명령 미구현 (`EnterError()`에 TODO 존재)
- 명령 및 피드백 Freshness / 출력 제한 미구현
- 서보 ID·레지스터 해석·실제 응답 시간·6축 100Hz 달성 여부 미검증
- 상위 Orange Pi 5+ / KCF / Physical AI 연동 미구현

### Next development steps

1. `app_main()`과 `MotorInterface` 연동 및 코드 빌드 확인
2. ST3215 출력 모드·레지스터 사양 검증 후 6축 `SYNC_WRITE`와 안전 정지 구현
3. Command / Feedback Freshness 및 제어 입력 제한 추가
4. ESP32 외부 PID와 관절 목표 제어 구현
5. 실제 하드웨어에서 통신 성공률, 6축 사이클 시간, Hz, 지연 및 오류 복구 측정
6. 상위 KCF 애플리케이션 및 Physical AI 동작 명령 인터페이스 연동

> 이 문서는 `v0.1` 커밋 기준의 구현 현황입니다. 아직 실제 모터 제어 성능이나 안전성이 확인된 릴리스가 아닙니다.

## v0.12 — fixed some errors

- **Commit message:** `v0.12 fixed some errors`
- **Based on:** `v0.12` (`ac5fa208`)
- **Validation:** Source review; ESP-IDF build and hardware validation not performed in this change

### Fixes
- INIT no longer enters READY immediately: it waits for `MotorInitState::COMPLETE` and aborts on ERROR.
- Implemented missing START, SET_TORQUE_LIMIT, WAIT_TORQUE_LIMIT, NEXT_MOTOR, COMPLETE and ERROR transitions for sequential initialization of IDs 11–17.
- Passed the configured UART port to `ST3215ParameterFSM` before ServoTask creation.
- Changed `ST3215::MOTOR_COUNT` from 6 to 7 to match the arm and avoid rejecting seven-device SYNC_WRITE.
- Removed unused, declared-only parameter wrappers from `MotorInterface`; `ST3215ParameterFSM` is the transaction owner.
- Added `VerifyOperatingMode()` for READ-only mode validation. With `MOTOR_EEPROM_WRITE_ENABLE=0`, mismatches cause INIT ERROR without EEPROM WRITE.
- Added `MOTOR_INIT_TORQUE_LIMIT_ENABLE=0` default: the 1000-unit sample limit is not automatically written. If explicitly enabled, setting uses WRITE → READ BACK verification.

### Safety and next tests

Default EEPROM mode changes and PWM output remain disabled. Even with no writes, INIT requires seven connected servos with matching configured mode, so a fresh factory setup may fail verification as expected. No torque enable or automatic movement is performed. Before activating EEPROM writes or PWM output, verify mode/register semantics, suspended-arm safety, real servo response timing, and full 7-device 100Hz budget.


## v0.13 follow-up — FSM transition hooks and PID validation

- **Based on:** `v0.13` (`f2f38de`)
- **Validation:** Source-level checks performed; local ESP-IDF build and hardware tests still required for this follow-up.

### Changes
- Added `ChangeState/ExitState/EnterState` for the MotorInterface INIT/READY/RUNNING/ERROR FSM. Hooks run only on actual state changes; initial INIT entry is invoked once during Initialize().
- Added `ChangeCommState/ExitCommState/EnterCommState` for the communication FSM. RX_RECOVERY timestamp/inhibit are set on entry, and WAIT_RESPONSE receive context is released on exit. Existing sequential READ/TX ordering remains unchanged.
- Added the previously missing `InitializePID()` call before ServoTask starts. Initial gains remain zero; PWM output stays compile-time disabled.
- Preserved the drone PIDController2 I-Leak default (0.3/s). Made the I-zone configurable; disabled I-zone scaling for raw encoder counts pending tuning.
- Added non-finite PID output validation before `lround`/PWM encoding. A failed calculation invalidates control and inhibits TX.
- Added edge-triggered control-validity tracking to avoid repeated PID resets while commands remain stale. ERROR and RUNNING entry resets are one-shot; communication failures invalidate control.
- Command freshness (100ms), feedback freshness (30ms), 7-motor limits and communication FSM behavior remain in place.

### Safety / limitations
- No autonomous motor enable, no validated physical safe-stop, no shoulder motor mapping, and no hardware performance verification. `MOTOR_PWM_OUTPUT_ENABLE=0`, `MOTOR_EEPROM_WRITE_ENABLE=0`, and `MOTOR_INIT_TORQUE_LIMIT_ENABLE=0` remain unchanged.
- Firmware build on the developer's ESP-IDF 6.1 setup is required to confirm this follow-up revision.


## v0.14 — unified MotorInterface ARM/DISARM FSM

- Extended `MotorInterfaceState` with DISARMED/ARMING/ARMED/DISARMING, replacing RUNNING. Uses existing EnterState/ExitState hooks on actual state changes.
- READY: READ-only verification of Torque Enable OFF for servo IDs 11–17. Never automatically cuts torque at boot; mismatch enters ERROR.
- DISARMED: keep feedback polling, PID/PWM suppressed. ARMED: command/feedback validated PID and conditional PWM. ARMING/DISARMING: UART-exclusive per-servo torque WRITE + READ BACK via ST3215ParameterFSM.
- Added atomic RequestServoOn/RequestServoOff and GetState, plus VerifyTorqueEnable read-only helper.
- Writes disabled by default (MOTOR_TORQUE_WRITE_ENABLE=0), PWM output stays disabled; no automatic physical movement. Partial ARM failure does not guarantee torque off.
- GitHub source update only; ESP-IDF v6.1 build/hardware timing and safe-stop behavior remain unverified.


## v0.15 — JointMapper API

- **Commit:** [`d6da7fd`](https://github.com/Nyamkani/roarm-m3-esp32-custom/commit/d6da7fd625ef55c7b10a4f2b141ce8f72296de15)
- **Commit message:** `v0.15 add jointmapper api`
- **Validation:** Source integration and build confirmed; hardware validation not performed.

### Implemented

- Added `JointAngles` (base, shoulder, elbow, wrist, roll, gripper) and `ServoPositions` for seven motor encoder values (IDs 11–17).
- Added `JointMapper::ToServoPositions()` / `ToJointAngles()`: joint/encoder bidirectional conversion, range checks and coupled Shoulder feedback consistency.
- Added `MotorInterface::SetJointTargets()` / `GetJointFeedback()` using existing command/feedback snapshots.
- The mapper is a standalone conversion API; ServoTask, communication FSM and UART ownership are unchanged.

### Limitations

- Servo directions, zero offsets, motion ranges and Shoulder tolerance are based on source assumptions; verification with the physical arm is required.
- Encoder mapping endpoints and feedback characteristics must be verified before enabling motor output.

## v0.16 — IK/FK and JSON command interface

- **Branch:** `main`
- **Commit:** [`5aef6eb`](https://github.com/Nyamkani/roarm-m3-esp32-custom/commit/5aef6ebcb5217b50ce0135844e7915d20219be90)
- **Commit message:** `v0.16 add arm kinematics and JSON command interface`
- **Date:** 2026-11-20 (KST)
- **Validation:** ESP-IDF 6.1 build PASS (developer reported); UART communication, physical servo and motion tests not performed.

### Implemented in source

- **`ArmKinematics`**: manufacturer RoArm-M3 geometry-based `Inverse()` (X/Y/Z/T/R/G → joint angles) and `Forward()` (joint angles → X/Y/Z/T/R/G), with invalid and unreachable input rejection.
- **`ArmCommandInterface`**: cJSON-based `T=102` joint targets, `T=1041` Cartesian direct targets; `T=0` clears target and requests asynchronous DISARM; feedback `T=105` routed by Application Task through joint feedback and FK.
- **Target freshness**: `Process()` republishes retained targets every ~20 ms only while `MotorInterfaceState::ARMED`, to satisfy the MotorInterface command freshness window (100 ms).
- **`ArmCommandTask`**: Application Task pinned to Core 1, priority 3, 20 ms loop; receives newline-delimited JSON from UART0 console stdin. Existing `ServoTask` remains on Core 0, priority 5, with UART1 ownership.
- **Build/dependencies**: `main/CMakeLists.txt` registers the new .cpp files; `main/idf_component.yml` adds `espressif/cjson`; `dependencies.lock` records dependency resolution.
- **Architecture**: Orange Pi 5+ is planned to own VLA/LLM, Task/Arm Controller and motion planning, while ESP32 owns Pose execution, IK/FK, joint mapping, MotorInterface, servo communication and safety control.

### Constraints and pending validation

- v0.16 build success **does not** establish UART0 JSON input/output, IK/FK real-arm accuracy, 7-servo feedback, torque handling or position-following performance.
- `T=105` replies use a minimal custom subset, not the manufacturer's complete JSON response schema.
- Nonzero `spd` / `acc` options are rejected. No trajectory interpolation, path planning or Grasp Controller on ESP32.
- `T=0` is a non-immediate DISARM request, **not** a tested hardware E-stop.
- Retained targets have no communication-loss expiration policy. Do not enable physical torque/PWM output until watchdog/timeout, initialization parameter checks, safe stop and motor mapping are verified.
- `MOTOR_TORQUE_WRITE_ENABLE=0`, `MOTOR_PWM_OUTPUT_ENABLE=0`, `MOTOR_EEPROM_WRITE_ENABLE=0` remain safety defaults.
- `test_arm_kinematics.cpp` is not part of the normal ESP-IDF build; IK/FK unit test success must be reported independently.
- cJSON's generated `managed_components/` files were committed in v0.16; consider tracking only the manifest and lockfile in a later repository cleanup.

### Next

1. Verify UART0 JSON reception and responses on the actual ESP32, while UART1 servo ownership stays separate.
2. Validate IK/FK and JointMapper consistency over the supported, physically reachable workspace.
3. Review vendor firmware INIT, torque and operating-mode parameters against the custom PWM/PID architecture before changing EEPROM or enabling output.
4. Validate servo direction, initial position, coupled Shoulder behavior, feedback freshness and safe-stop/command timeout.
5. Test Pose command → IK → joint mapping → PID → servo output and feedback → FK on hardware before implementing SBC motion planning.
