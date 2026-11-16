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
