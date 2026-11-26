# RoArm-M3 Custom Firmware — JSON Communication Protocol

> **기준 브랜치:** `feature/motion-operation-status` (2026-11-26)
>
> **상태:** 현재 소스 코드에서 확인된 명령과 응답을 기술합니다. 예시 응답의 위치/상태 값은 설명용이며 실제 측정값이 아닙니다. **Home은 T=217로 구현되어 있으나 기본 비활성화(실기 미검증) 상태입니다. Pause / Resume / Protective Stop / Calibration은 미구현이며 명령 번호도 배정하지 않았습니다.**

## 2026-11-27 — Initialization protocol scaffold (feature/homing-fsm-gated)

- `T=218,cmd=1`: reserved future SBC-KCF initialization request. **Always rejected** with `{"ok":false,"status":"motor_unavailable"}` in the present firmware. No HOME, calibration or torque write occurs.
- `T=219`: read-only initialization status. Currently returns `initialization_status="NOT_INITIALIZED"`, `initialization_complete=false`, `calibration_valid=false`, `move_allowed=false`, `execution_enabled=false`.
- `T=211` also exposes `initialization_complete`, `calibration_valid`, and `move_allowed`.
- Initialization coordinator, calibration and automatic progression remain **unimplemented**. No flag is set true on ARM/HOME/PARK. The ESP32 retains authority to reject general MOVE.
- Local test: `python3 tools/init_protocol_smoke.py --port /dev/ttyUSB0 --expect-state DISARMED`; optional refusal test `--check-rejection` only with the known disabled implementation.

## 1. Transport and message framing

- **Host ↔ ESP32:** UART0 표준 콘솔 (stdin/stdout), **115200 bps**, JSON 객체 한 줄(\n 또는 \r) 단위. 한 행에 하나의 JSON 요청을 넣습니다.
- **ESP32 ↔ ST3215:** UART1 **1,000,000 bps**. 별도의 바이너리 서보 프로토콜이며 본 문서의 JSON 프로토콜이 아닙니다.
- **한 줄 최대 크기:** 512 bytes(JSON 문자 데이터, 종료 문자는 별도). 초과 입력은 줄 끝에서 `{"ok":false,"error":"line_too_long"}` 반환.
- **일반 파싱 실패:** `{"ok":false,"error":"invalid_json"}`. JSON 형식은 유효하나 필드 형식/명령이 잘못된 경우 `status` 오류 반환.
- **숫자:** `T`는 정수 값의 JSON number, `cmd`는 숫자 0 또는 1 등 명시된 값. 불리언/문자열은 명령 인자로 인정하지 않습니다.
- **동기 RPC가 아님:** 액션 명령의 `accepted`는 접수만 의미합니다. 완료 여부는 `T=211`을 다시 조회해야 합니다.
- UART0에는 ESP-IDF 로그도 출력될 수 있습니다. 수신 측은 **완전한 JSON 행만 응답으로 파싱**하고 로그 행을 구분해야 합니다.
- 현재 통신에는 메시지 시퀀스 번호, 요청 ID, CRC(응용 계층), 인증, 구독/푸시 스트림, 명령별 완료 응답이 없습니다. 한 요청 후 응답을 확인하는 순차 통신을 권장합니다.

### Request/response patterns

**Action request:**

```text
HOST -> ESP32: {"T":210,"cmd":1}\n
ESP32 -> HOST: {"ok":true,"status":"accepted"}\n
```

**Command rejection:**

```json
{"ok":false,"status":"motor_unavailable"}
```

**Status query (after an action):**

```text
HOST -> ESP32: {"T":211}\n
ESP32 -> HOST: {"T":211, ... }\n
```

> `ok=true` 또는 `accepted`는 물리 동작의 성공/완료나 안전 상태를 보장하지 않습니다. 특히 Servo OFF는 중력 하중 때문에 관절이 낙하할 수 있습니다.

## 2. Implemented command index (SEND → RECV)

| T | SEND JSON | 의미 | RECV 형태 | 요구 상태 / 비고 |
|---|---|---|---|---|
| 0 | `{"T":0}` | 소프트웨어 무Parking Servo OFF **요청** | `{"ok":true,"status":"accepted"}` | 모든 상태에서 접수될 수 있으나 실제 실행/해제는 Servo FSM에 의존; 하드웨어 E-stop 아님 |
| 105 | `{"T":105}` | 현재 FK 기반 툴 자세 조회 | `{"T":105,"x":...,"y":...,"z":...,"t":...,"r":...,"g":...}` 또는 `{"ok":false,"error":"feedback_unavailable"}` | 매핑·FK가 유효해야 함 |
| 210 | `{"T":210,"cmd":1}` | **Servo ON**: 현재 위치로 Hold 준비·검증 후 활성화 | 일반 `accepted` / `motor_unavailable` | **DISARMED**에서만 접수; Servo ON 완료는 `T=211`에서 `state=ARMED` 확인 |
| 210 | `{"T":210,"cmd":0}` | **Parking 후 Servo OFF** 요청 | 일반 `accepted` / `motor_unavailable` | **ARMED**, `MOTOR_VENDOR_SHOULDER_PARK_ENABLE=1` 필요 |
| 211 | `{"T":211}` | Servo 및 Motion/Operation 상태 조회 | `{"T":211, ...}` | 읽기 전용; 자세 변환 실패도 응답 |
| 212 | `{"T":212}` | 현재 엔코더 위치 기반 Hold 사전 확인 | `{"T":212,"ok":true,"positions":[...]}` 또는 `hold_preflight_failed` | **DISARMED**, 신선한 7개 서보 피드백 |
| 213 | `{"T":213}` | Servo ON 정지 상태 사전 확인 (dry-run) | `{"T":213,"ok":true,"dry_run":true,"torque_write_enabled":false,"positions":[...]}` 또는 `arm_preview_failed` | **DISARMED**; 읽기 전용, 실제 ON 아님 |
| 214 | `{"T":214}` | Parking 목표 미리보기 (dry-run) | `{"T":214,"ok":true,"dry_run":true,"torque_write_enabled":false,"park_goals":[...],"max_delta_counts":...}` 또는 `park_preview_failed` | **DISARMED**; 실제 Parking 실행 아님 |
| 215 | `{"T":215,"cmd":0}` | 명시적인 **무Parking Servo OFF** 요청(벤치/복구) | 일반 `accepted` / `motor_unavailable` | **ARMED / ARMING / BOOT_RECOVERY** 및 POSITION·Torque write 빌드 조건 필요 |
| 216 | `{"T":216}` | Home 미리보기 (read-only) | `T=216, dry_run=true` / `home_preflight_failed` | **ARMED**, 정상 결합·정지 피드백 필요 |
| 217 | `{"T":217,"cmd":1}` | Home 이동 요청 (**기본 비활성화**) | 기본 `motor_unavailable`; 실기 게이트 허용 시 `accepted` | **MOTOR_HOME_EXECUTION_ENABLE=0** 기본; 실기 미검증 |
| 102 | `{"T":102,"base":0,"shoulder":0,"elbow":0,"wrist":0,"roll":0,"hand":0}` | 6관절 목표(rad) | **현재 POSITION 빌드에서는** `motor_unavailable` | **미활성화**: 소스에 형식·유효성 검사 구현은 존재 |
| 1041 | `{"T":1041,"x":0,"y":0,"z":0,"t":0,"r":0,"g":0}` | Cartesian 목표(mm/rad) | **현재 POSITION 빌드에서는** `motor_unavailable` | **미활성화**: IK/형식 검사 코드는 존재 |

**주의:** `T=0`은 명령 수신 시 `RequestServoOff()`를 호출하지만, 물리 Servo OFF는 비동기 FSM이 수행합니다. `ERROR`를 포함한 모든 상태에서 안전하게 Servo OFF가 완료된다고 보장하지 않습니다.

### General command acknowledgment / error

일반 액션 명령은 다음 형태를 반환합니다.

```json
{"ok":true,"status":"accepted"}
```

```json
{"ok":false,"status":"invalid_target"}
```

`status` 값:

| 값 | 의미 |
|---|---|
| `accepted` | 요청을 접수함. 완료 보장 아님 |
| `invalid_json` | 명령 API에서 잘못된 JSON 또는 필수 형식 오류 |
| `unsupported_command` | 지원하지 않는 T |
| `invalid_target` | 파라미터 값이나 형식이 올바르지 않음 |
| `motor_unavailable` | 현재 상태/빌드 설정으로 요청 불가 |

별도 프레이밍/파싱 오류는 `status`가 아닌 `error`를 반환할 수 있습니다: `invalid_json`, `line_too_long`, `feedback_unavailable`, `hold_preflight_failed`, `arm_preview_failed`, `park_preview_failed`.

## 3. Servo ON — T=210, cmd=1

**SEND**

```json
{"T":210,"cmd":1}
```

**RECV (immediate; 접수)**

```json
{"ok":true,"status":"accepted"}
```

**후속 확인**

```json
{"T":211}
```

`state="ARMING"`이면 진행 중, `state="ARMED"`이면 모든 서보 Servo ON 검증 통과입니다. `state="ERROR"`면 안전하게 OFF됐다고 추정해서는 안 됩니다.

Servo ON 경로는 최신 피드백 기반 현재 위치 목표를 설정 → 목표/현재 위치 검증 → 7개 Servo ON 검증을 수행합니다. 전원 켰다고 자동 Home으로 이동하지 않습니다.

## 4. Parking / Servo OFF — T=210, cmd=0

**SEND**

```json
{"T":210,"cmd":0}
```

**RECV (immediate)**

```json
{"ok":true,"status":"accepted"}
```

**상태 조회**

```json
{"T":211}
```

진행 중 응답의 **일부 필드**:

```json
{"state":"PARKING","motion_state":"PARKING","operation":"PARK","operation_status":"RUNNING"}
```

완료 후 응답의 **일부 필드**:

```json
{"state":"DISARMED","motion_state":"IDLE","operation":"PARK","operation_status":"COMPLETED","torque_off_verified":true}
```

- `PARK/COMPLETED`는 Parking 동작 후 7개 서보의 Servo OFF 검증까지 완료되었을 때 기록합니다.
- `PARK/INTERRUPTED`는 Parking 중 무Parking OFF 요청으로 중단됐다는 뜻입니다. 이 결과 자체가 Servo OFF 검증을 의미하지 않습니다.
- `PARK/FAILED`는 ERROR 진입 등을 의미할 수 있으며, 실제 Servo 상태는 불명확할 수 있습니다.
- 현 브랜치의 `MOTOR_VENDOR_SHOULDER_PARK_ENABLE` 기본값은 **0**입니다. 이전 실기에서는 로컬에서 수동으로 1로 켜 테스트했습니다. 기본 빌드를 그대로 사용하면 Parking 요청은 거절됩니다.

## 5. Diagnostics — T=211

**SEND**

```json
{"T":211}
```

**RECV (설명용 예시, 나머지 실제 필드 포함)**

```json
{
  "T": 211,
  "state": "DISARMED",
  "motion_state": "IDLE",
  "operation": "PARK",
  "operation_status": "COMPLETED",
  "feedback_valid": false,
  "torque_write_enabled": true,
  "pwm_output_enabled": false,
  "position_output_enabled": true,
  "physical_servo_enabled": true,
  "torque_off_verified": true,
  "vendor_park_enabled": true,
  "control_mode": "POSITION"
}
```

| Field | Type | 설명 |
|---|---|---|
| `T` | integer | 211 |
| `state` | string | 실제 Motor/Servo FSM 상태 |
| `motion_state` | string | **1차 파생 조회 필드**, 현재 독립 Motion FSM은 아님 |
| `operation` | string | 현재/최근 작업: `NONE`, `PARK` |
| `operation_status` | string | `IDLE`, `RUNNING`, `COMPLETED`, `INTERRUPTED`, `FAILED` |
| `home_id13_last_verified` | string | `UNKNOWN`, `OFF`, `ON`: 마지막으로 확인된 HOME 중 ID13 Torque 명령 상태. **실시간 Servo Torque 증명 아님** |
| `feedback_valid` | bool | `GetJointFeedback()` 성공 여부. 단순 UART/서보 수신 유효성만 나타내는 값이 아님 |
| `torque_write_enabled` | bool | 컴파일 설정: Servo Torque 쓰기 허용 여부 |
| `pwm_output_enabled` | bool | 컴파일 설정: PWM 출력 허용 여부 |
| `position_output_enabled` | bool | 컴파일 설정: POSITION 출력 허용 여부 |
| `physical_servo_enabled` | bool | 컴파일 설정: 물리 Servo 허용 여부 |
| `torque_off_verified` | bool | Servo OFF 확인 완료 여부 (Servo 상태와 함께 판단) |
| `vendor_park_enabled` | bool | 컴파일 설정: 제조사 방식 Parking 활성화 |
| `control_mode` | string | `POSITION` 또는 `PWM` |

`state`: `INIT`, `READY`, `DISARMED`, `ARMING`, `ARMED`, `PARKING`, `DISARMING`, `BOOT_RECOVERY`, `ERROR`.

`motion_state` (현재 T211 파생 규칙): `PARKING` → `PARKING`, `DISARMING` → `STOPPING`, `ARMED/DISARMED` → `IDLE`, 나머지는 `UNAVAILABLE`. 이 값은 Servo FSM을 사람이 읽기 쉽게 표시한 것이며, **독립적인 Motion FSM은 아직 없습니다.**

`feedback_valid=false`여도 개별 서보 피드백이 전부 통신 실패라는 뜻은 아닙니다. 예: Shoulder 관절 각도 범위나 ID12/13 결합 관계 변환 실패. 실제 Servo 통신 진단과 관절 좌표 유효성은 구분해야 합니다.

`operation=PARK, operation_status=COMPLETED`는 **최근 Parking 결과**로 남을 수 있습니다. Servo ON을 다시 실행해도 결과는 보존됩니다. 현재 Torque 상태 대신 `state`와 `torque_off_verified`를 확인하세요.

## 6. Preflight/preview queries

### T=212 — Current Hold (read-only)

**SEND:** `{"T":212}`

**RECV (example):**

```json
{"T":212,"ok":true,"positions":[2047,2047,2047,2047,2047,2047,2047]}
```

실패: `{"T":212,"ok":false,"error":"hold_preflight_failed"}`.

### T=213 — Servo ON preview (read-only)

**SEND:** `{"T":213}`

**RECV (example):**

```json
{"T":213,"ok":true,"dry_run":true,"torque_write_enabled":false,"positions":[2047,2047,2047,2047,2047,2047,2047]}
```

실패: `{"T":213,"ok":false,"error":"arm_preview_failed"}`.

`dry_run`의 `torque_write_enabled=false`는 **이 응답이 실제 쓰기 동작을 하지 않는다는 뜻**이며 `T=211`의 같은 키(빌드 설정)와 의미가 다릅니다.

### T=214 — Parking preview (read-only)

**SEND:** `{"T":214}`

**RECV (현 브랜치 코드 기준):**

```json
{"T":214,"ok":true,"dry_run":true,"torque_write_enabled":false,"park_goals":[2047,1023,null,3071,2047,2047,2047],"max_delta_counts":1024}
```

`max_delta_counts`는 현재 엔코더에 따라 변하는 **예시 숫자**입니다. 실패: `{"T":214,"ok":false,"error":"park_preview_failed"}`.

**알려진 차이:** 실제 Parking의 ID14 Elbow 목표는 하드웨어 보정으로 **3039 counts**이지만, `T=214` 미리보기 응답은 현재 소스에서 여전히 **3071 counts**를 사용합니다. 프리뷰와 실행 값이 일치하지 않으므로 **T214만 보고 실제 제어 목표를 추정하면 안 됩니다**. 별도 정정 이슈입니다.

## 7. Pose feedback — T=105

**SEND:** `{"T":105}`

**RECV (형식 예시):**

```json
{"T":105,"x":100.000,"y":0.000,"z":200.000,"t":0.000000,"r":0.000000,"g":0.000000}
```

실패: `{"ok":false,"error":"feedback_unavailable"}`.

| Field | Unit | Meaning |
|---|---|---|
| `x,y,z` | mm | 베이스 좌표계의 엔드이펙터 위치 |
| `t` | rad | tool tilt |
| `r` | rad | tool roll |
| `g` | rad | gripper joint angle |

이는 제조사 T105 전체 응답과 완전히 호환되는 포맷이 아닌 **커스텀 최소 subset**입니다. 실제 센서 원시 엔코더/전류/속도 JSON은 현재 이 명령으로 제공되지 않습니다.

## 8. Servo IDs and POSITION values

Servo ID / 배열 순서는 고정입니다.

| Index | Servo ID | Axis |
|---|---|---|
| 0 | 11 | Base |
| 1 | 12 | Shoulder Driving |
| 2 | 13 | Shoulder Driven |
| 3 | 14 | Elbow |
| 4 | 15 | Wrist |
| 5 | 16 | Roll |
| 6 | 17 | Gripper |

- 서보 엔코더 단위: **counts**, 이 펌웨어의 정상 위치 범위 `0..4095`, 중심 `2047`.
- 이상적인 Shoulder 관계: `ID12 + ID13 ≈ 4094`; 실측 허용 오차와 시차가 존재합니다.
- 관절 명령 형식 (`T=102`)의 각도 단위: **rad**, `base/shoulder/elbow/wrist/roll/hand`.
- Cartesian 명령 (`T=1041`): `x,y,z`는 **mm**, `t,r,g`는 **rad**. 현재 물리 POSITION 명령 출력 경로에서는 **차단**되어 있습니다.
- `spd`, `acc` 비영(0이 아닌) 값의 궤적 제어 지원은 현재 구현되어 있지 않습니다.

## 9. Example host-side transaction (state polling)

```text
SEND {"T":213}
RECV {"T":213,"ok":true,"dry_run":true,"torque_write_enabled":false,"positions":[...]}

SEND {"T":210,"cmd":1}
RECV {"ok":true,"status":"accepted"}

SEND {"T":211}
RECV {"T":211,"state":"ARMING",...}

SEND {"T":211}
RECV {"T":211,"state":"ARMED",...}

SEND {"T":210,"cmd":0}
RECV {"ok":true,"status":"accepted"}

SEND {"T":211}
RECV {"T":211,"state":"PARKING","operation":"PARK","operation_status":"RUNNING",...}

SEND {"T":211}
RECV {"T":211,"state":"DISARMED","operation":"PARK","operation_status":"COMPLETED","torque_off_verified":true,...}
```

위 예시는 **전이 흐름**이며 샘플링 순간에 ARMING/PARKING 중간 상태가 반드시 보이는 것은 아닙니다. `T=210 cmd=0`을 사용하려면 빌드에서 Parking 활성화 및 적절한 실기 안전 검증이 필요합니다.

## 10. Experimental Homing command and future protocol

### T=217 — Home action (implemented but **disabled by default**)

**SEND**: `{"T":217,"cmd":1}`

**RECV with default configuration**: `{"ok":false,"status":"motor_unavailable"}`

To avoid unexpected motor motion, `MOTOR_HOME_EXECUTION_ENABLE` defaults to **0**. If intentionally enabled for separately supervised firmware validation, `ARMED` plus valid Home preflight are required, then `accepted` signifies asynchronous admission only. A subsequent `T=211` can show `HOMING/HOME/RUNNING` and, on successful completion, `ARMED/HOME/COMPLETED`. **Do not enable without a physical support plan and error-recovery review.**

### Planned features (NOT IMPLEMENTED)

아래 기능은 **설계 예정**이며 **JSON 명령 번호와 페이로드를 아직 확정하지 않았습니다**.

| Planned feature | Purpose |
|---|---|
| HOME | Servo ON 상태에서 명시적 초기 자세 이동 (영점 보정 없이) |
| CALIBRATION | 영점 오프셋 보정 전용, 별도 승인/절차 |
| PAUSE | 정상 운전 감속 정지 후 위치 유지 |
| RESUME | 명시적 복구 승인 및 현재 위치 기준 새 궤적 |
| PROTECTIVE STOP | 이상 감지 후 새 명령 차단, 원인/복구 절차 보고 |
| SAFETY STATUS | 정지 사유, 동작 ID, 실제 Servo 상태, 오류 타임스탬프 |

정지 기능은 Servo OFF와 구분해야 합니다. 기존 `T=0`을 안전 인증된 하드웨어 Emergency Stop 또는 Kill Switch로 표시하지 마세요.

## 11. Source of truth

- `main/main.cpp`: JSON-line 수신, T=105/211/212/213/214 응답, 공통 ACK/error.
- `main/arm_command_interface.cpp`: T=0/210/215/102/1041 파싱, 수락 조건, FK 응답.
- `main/motor_interface.cpp` / `.h`: Servo FSM, Parking, operation status 기록, Servo ON 사전 검증.
- `main/joint_mapper.h` / `main/arm_kinematics.h`: 엔코더/관절 변환과 Cartesian 단위.
- `docs/MOTION_STOP_FSM_DESIGN.md`: 추후 Motion/Stop 기능의 설계.

**검증 범위:** 소스 확인 및 문서화만 수행. 신규 펌웨어 빌드/실기 통신 검증은 아직 미실시.
