# RoArm-M3 ESP32 Custom Firmware

[![Status](https://img.shields.io/badge/status-in_development-orange)](#development-status)

Waveshare **RoArm-M3**의 ESP32 제어 보드에서 동작하는 독립적인 **ESP-IDF / C++ 로봇팔 펌웨어** 개발 프로젝트입니다.

제조사 펌웨어 대신 직접 작성한 코드로 **6개 관절(7개 물리 서보)의 피드백 수집, 동기 출력, 제어 주기 및 오류 대응**을 관리하는 것이 목적입니다. 최종적으로는 Physical AI 시스템에서 로봇팔의 저수준 제어 인터페이스로 사용합니다.

## Target

- **Robot arm:** Waveshare RoArm-M3
- **Controller:** RoArm-M3 내장 ESP32 보드 (ESP32-WROOM-32 기반)
- **Actuators:** Feetech ST3215 계열 시리얼 서보 7개 (독립 관절 6축; Shoulder 구동/종동 2개)
- **Servo bus:** Half-duplex UART, 1 Mbps, 8N1 (현재 프로젝트 설정)
- **Firmware:** ESP-IDF v6.1, C++ / FreeRTOS
- **Build system:** ESP-IDF / CMake

서보 ID, 통신 설정 및 레지스터 동작은 실물 하드웨어에서 검증할 예정입니다.

## Development Goals

1. 제조사 제어 기능을 참고하여 독립 펌웨어로 로봇팔 구동 기반을 구현합니다.
2. 7개 서보를 일정 주기로 읽고 제어하며, **Read All → Compute All → Commit All** 형태의 다축 동기 제어를 목표로 합니다.
3. ST3215 내부 위치 제어에 전적으로 의존하지 않고, ESP32에서 피드백 기반 **외부 PID 및 관절 제어**를 구현할 계획입니다. PWM 직접 제어 모드의 실제 지원 범위는 검증이 필요합니다.
4. 통신 지연, 제어 주기, 다축 동기화, 오류 처리 및 정지 동작을 측정·개선합니다.
5. 초기 개발 목표는 제조사 펌웨어와 비교 가능한 기본 제어 기능과 응답성을 확보하는 것입니다. 성능 비교 목표(약 80%)는 개발 지향점이며, 달성 여부는 아직 검증되지 않았습니다.

## Physical AI Integration Plan

이 저장소의 ESP32 펌웨어는 **AI 추론 자체보다 로봇팔의 저수준 제어와 안전 처리**에 집중합니다.

```text
사용자: 작업 목표 지정 / 운용 승인 / 중지 판단
                 │
                 ▼
상위 Physical AI (향후 VLA/VLM 기반 인식·행동 판단)
                 │  목표 동작 / 관절 또는 동작 명령
                 ▼
Orange Pi 5+ / KCF 기반 애플리케이션 (계획)
                 │  통신 및 명령 인터페이스
                 ▼
RoArm-M3 ESP32 Custom Firmware
  - 7개 서보 피드백 수집 / Snapshot
  - 외부 PID 연산 경로 (구현, 물리 동작 미검증)
  - 다축 출력 / 오류 처리 / 안전 제한 (개발 중)
                 │
                 ▼
           ST3215 서보 × 7
```

**판단과 제어의 역할**

- **사용자:** 수행할 작업과 목표를 지정하고, 운용 여부 및 중지·개입을 결정합니다.
- **상위 AI:** 향후 카메라·센서 정보를 활용해 물체 인식, 집기·놓기 등의 동작을 판단하고 구체적인 행동 명령을 생성하도록 계획합니다.
- **ESP32 펌웨어:** 상위 명령을 직접 검증·실행할 수 있는 하위 관절/모터 제어, 상태 피드백 및 오류 대응을 담당합니다. 안전 제한을 AI 판단에만 맡기지 않는 구조를 지향합니다.

현재 버전에는 AI 모델이나 상위 AI 연동 기능이 포함되어 있지 않습니다. ROS/ROS 2는 사용하지 않는 방향으로 설계 중입니다.

## Architecture

```text
app_main / Board initialization
          │
          ▼
MotorInterface (FreeRTOS task + communication FSM)
  ├─ UART TX/RX + response timeout / RX recovery
  ├─ Seven-servo sequential READ
  ├─ INIT: sequential mode verification / optional parameter transaction
  ├─ MotorFeedbackFrame / TripleSnapshot
  ├─ Communication error counter / ERROR state
  ├─ PIDController2 × 7 (zero gains by default; command/feedback freshness)
  ├─ ChangeState / EnterState / ExitState (interface and comm FSM)
  └─ Performance logging (optional)
          │
          ▼
ST3215 (packet serializer / response parser / feedback decoder)
          │
          ▼
7× ST3215 serial servos
```

## Development Status

**v0.13 follow-up — PID / FSM transition consolidation (hardware test pending)**

| Area | Status |
| --- | --- |
| ESP-IDF 프로젝트, UART 보드 초기화, ServoTask 시작 | 소스 구현 |
| ST3215 패킷 생성·응답 파싱 및 7개 서보 피드백 수집 | 소스 구현 |
| 7개 ID (11~17) 순차 INIT 모드 검증 후 READY | 소스 구현 |
| EEPROM 모드 변경 (Unlock → WRITE → READ BACK → Lock) | 구현, 기본 실행 금지 |
| 기본 초기화 모드 정책 | EEPROM WRITE 0, 현재 PWM_OPEN 모드 READ 검증 |
| 토크 제한 설정 | 구현, 초기화 자동 WRITE 기본 비활성화 |
| Command Snapshot → 7개 서보 SYNC_WRITE | 코드 경로 구현, PWM 출력 비활성화 |
| 외부 PID 계산 (PIDController2 × 7) | 구현, 게인 0 기본값 / 물리 출력 비활성화 |
| ChangeState/EnterState/ExitState (interface / comm) | 구현, 전환 시에만 훅 실행 |
| HOME/IDLE, 하드웨어 긴급 정지 | 미구현 |
| 실제 ESP-IDF 빌드 / 실물 서보 검증 | 이번 수정 기준 미검증 |

기본값에서는 현재 모드가 목표 PWM_OPEN과 다르면 INIT이 실패하며, **EEPROM을 자동 변경하지 않습니다.** 토크 활성화 및 PWM 출력도 자동 실행하지 않습니다. 실물 구동 전에는 토크 해제 시 중력 낙하 위험, 직접 PWM 제어 적합성, 통신·정지 안전 조건을 별도로 확인해야 합니다.

## Project Layout

```text
main/
├── main.cpp               # app_main / board initialization
├── gpio_config.*          # board UART initialization
├── pins.h                 # board pin configuration
├── motor_interface.*      # seven-servo init / communication FSM, PID integration
├── pid.h                  # reusable PIDController2 (from kssdrone v1.3)
├── st3215.*               # ST3215 packet protocol and motor data
├── st3215_parameter_fsm.* # parameter READ / WRITE / verify / EEPROM lock
└── shared_snapshot.h      # shared / triple snapshot utilities
```

## Build

ESP-IDF 환경을 활성화한 뒤 프로젝트 루트에서:

```bash
idf.py set-target esp32
idf.py build
```

빌드·플래시·실물 동작 여부는 환경 및 연결된 보드에서 별도로 검증해야 합니다.

개발 내역과 이후 변경 계획은 [update.md](update.md)를 참고하세요.
