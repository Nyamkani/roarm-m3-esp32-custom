
#pragma once

#include <cstddef>
#include <atomic>
#include <cstdint>


#include <array>
#include <cstddef>
#include <cstdint>
#include <algorithm>
#include <cmath>

#include "st3215.h"
#include "shared_snapshot.h"
#include "st3215_parameter_fsm.h"
#include "pid.h"
#include "joint_mapper.h"


#include "gpio_config.h"
#include "freertos/task.h"
#include "freertos/FreeRTOS.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_err.h"

#include "driver/uart.h"


#define MOTOR_COUNT 7

#define MOTOR_IO_DIRECT 0
#define MOTOR_IO_TASK   1

#define MOTOR_IO_MODE MOTOR_IO_TASK

#define MOTOR_TASK_PERIOD_MS 1

#define MOTOR_PWM_OUTPUT_ENABLE 0
#define MOTOR_TORQUE_WRITE_ENABLE 1 // SUPERVISED BENCH: physical servo torque


#define MOTOR_CONTROL_PWM       0
#define MOTOR_CONTROL_POSITION  1

#define MOTOR_CONTROL_MODE MOTOR_CONTROL_POSITION

// 실물 POSITION 명령 출력은 별도로 허용
#define MOTOR_POSITION_OUTPUT_ENABLE 1 // SUPERVISED BENCH: hold and park only
#define MOTOR_PHYSICAL_SERVO_ENABLE 1 // SUPERVISED BENCH: explicit physical enable
// The factory-style parking procedure temporarily releases shoulder ID13.
// This requires separate supervised mechanical validation before use.
#define MOTOR_VENDOR_SHOULDER_PARK_ENABLE 0 // temporarily locked after ID12/13 mismatch during real park

#if MOTOR_CONTROL_MODE != MOTOR_CONTROL_PWM && \
    MOTOR_CONTROL_MODE != MOTOR_CONTROL_POSITION
#error "Invalid motor control mode"
#endif

#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_POSITION && \
    MOTOR_EEPROM_WRITE_ENABLE
#error "POSITION mode must not write operating mode EEPROM"
#endif

static constexpr int64_t CONTROL_PERIOD_US = 10000;
static constexpr int64_t CYCLE_BUDGET_US   = MOTOR_CONTROL_MODE == MOTOR_CONTROL_POSITION ? 15000 : 5000;
static constexpr int64_t RESPONSE_TIMEOUT_US = 2000;

// Example IDs. Verify against actual RoArm-M3.
static constexpr uint8_t SERVO_IDS[MOTOR_COUNT] =
{
    11, 12, 13, 14, 15, 16, 17
};
/*
서보 ID	역할
11	Base
12	Shoulder Driving
13	Shoulder Driven
14	Elbow
15	Wrist
16	Roll
17	Gripper
*/


#define MOTOR_PERF_LOG_ENABLE 1
#define MOTOR_PERF_LOG_PERIOD_US 1000000


// 0: verify operating mode by READ only; never unlock/write EEPROM.
// 1: allow SetOperatingMode() to update EEPROM after torque-off check.
#define MOTOR_EEPROM_WRITE_ENABLE 0

// Torque limit must be selected for the actual mechanism before enabling.
// Disabled by default: no implicit RAM writes during motor initialization.
#define MOTOR_INIT_TORQUE_LIMIT_ENABLE 0
static constexpr uint16_t MOTOR_INIT_TORQUE_LIMIT = 1000;

#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_POSITION && \
    (MOTOR_POSITION_OUTPUT_ENABLE || MOTOR_TORQUE_WRITE_ENABLE) && \
    (!MOTOR_PHYSICAL_SERVO_ENABLE || !MOTOR_TORQUE_WRITE_ENABLE || !MOTOR_POSITION_OUTPUT_ENABLE)
#error "Enable all three POSITION physical Servo write gates explicitly"
#endif

enum class ServoReceiveStatus : uint8_t
{
    WAITING,
    COMPLETE,
    TIMEOUT,
    ERROR
};


enum class MotorInterfaceState : uint8_t
{
    INIT,
    READY,
    DISARMED,
    ARMING,
    ARMED,
    PARKING,
    DISARMING,
    BOOT_RECOVERY,
    ERROR
};

// Read-only operation lifecycle. This is diagnostic bookkeeping, not a safety-rated stop.
enum class MotorOperation : uint8_t { NONE = 0, PARK = 1 };
enum class MotorOperationStatus : uint8_t { IDLE = 0, RUNNING = 1, COMPLETED = 2, INTERRUPTED = 3, FAILED = 4 };
struct MotorOperationSnapshot {
    MotorOperation operation;
    MotorOperationStatus status;
};

enum class MotorCommState : uint8_t
{
    IDLE,
    SEND_REQUEST,
    WAIT_RESPONSE,
    NEXT_MOTOR,
    PROCESS,       
    SYNC_WRITE,    
    COMPLETE,
    RX_RECOVERY
};

enum class MotorInitState : uint8_t
{
    START,
    SET_MODE,
    WAIT_MODE,

    SET_TORQUE_DIAG,
    WAIT_TORQUE_DIAG,

    SET_TORQUE_LIMIT,
    WAIT_TORQUE_LIMIT,
    NEXT_MOTOR,
    COMPLETE,
    ERROR
};

struct MotorCommandFrame
{
    MotorCommand motors[MOTOR_COUNT]{};
};

struct MotorFeedbackFrame
{
    MotorFeedback motors[MOTOR_COUNT]{};
};


class MotorInterface
{
public:
    esp_err_t Initialize(const board_handles_t& board);

    ST3215* GetMotor(size_t index);
    void RequestServoOn();
    void RequestServoOff();
    void RequestServoPark();
    MotorInterfaceState GetState() const { return state_public_.load(); }
    bool IsTorqueOffVerified() const { return torque_off_public_.load(); }
    MotorOperationSnapshot GetOperationSnapshot() const {
        const uint16_t packed = operation_snapshot_.load();
        return {static_cast<MotorOperation>(packed & 0xffu),
                static_cast<MotorOperationStatus>((packed >> 8) & 0xffu)};
    }

private:
    static void MainTask(void* arg);
    void MainLoop();

private:
    ST3215 motors_[MOTOR_COUNT]{};

    TaskHandle_t task_handle_ = nullptr;
    std::atomic<bool> task_stop_{false};

    uart_port_t uart_num_ = UART_NUM_MAX;

    esp_err_t SendRequest(const uint8_t* tx_buffer,size_t tx_len,TickType_t timeout_ticks);

    TripleSnapshot<MotorCommandFrame> command_snapshot_;
    TripleSnapshot<MotorFeedbackFrame> feedback_snapshot_;

    static constexpr uint8_t FEEDBACK_ADDRESS = 56;
    static constexpr uint8_t FEEDBACK_SIZE = 15;

    int64_t last_comm_cycle_us_ = 0;

    static constexpr int64_t COMM_PERIOD_US = 10000;

private:
    MotorInitState init_state_ = MotorInitState::START;

    size_t init_motor_index_ = 0;

    uint8_t init_mismatch_count_ = 0;
    uint8_t init_torque_on_count_ = 0;
    bool boot_recheck_mode_after_off_ = false;

    void ProcessMotorInitialization();

private:
    struct ServoReceiveContext
    {
        std::array<uint8_t, ST3215::MAX_PACKET_SIZE> buffer{};

        size_t size = 0;
        size_t expected_packet_size = 0;
        size_t expected_params_size = 0;

        uint8_t expected_id = 0;

        int64_t start_us = 0;
        int64_t completed_us = 0;

        bool active = false;
    };

    ServoReceiveContext rx_context_{};

    static constexpr int64_t RESPONSE_TIMEOUT_US = 2000;

    void BeginReceive(
        uint8_t expected_id,
        size_t expected_params_size
    );

    ServoReceiveStatus ReceiveResponse(
        ST3215Response& response,
        int64_t& completed_us
    );

    void ResetReceive();


private:
    MotorInterfaceState state_ =
        MotorInterfaceState::INIT;
    std::atomic<MotorInterfaceState> state_public_{MotorInterfaceState::INIT};
    enum class ServoRequest : uint8_t { NONE, ON, OFF, PARK };
    std::atomic<ServoRequest> servo_request_{ServoRequest::NONE};
    std::atomic<uint16_t> operation_snapshot_{0}; // operation in low byte, status in high byte
    void SetOperationSnapshot(MotorOperation operation, MotorOperationStatus status) {
        operation_snapshot_.store(static_cast<uint16_t>(operation) |
            (static_cast<uint16_t>(status) << 8));
    }

    MotorCommState comm_state_ =
        MotorCommState::IDLE;

    size_t current_motor_index_ = 0;

    int64_t last_control_cycle_us_ = 0;

    static constexpr int64_t
        CONTROL_PERIOD_US = 10000;

    void ProcessInterfaceFSM();
    void ProcessCommunicationFSM();
    void ProcessControlCycle();

    void ChangeState(MotorInterfaceState next);
    void EnterState(MotorInterfaceState state);
    void ExitState(MotorInterfaceState state);

    void ChangeCommState(MotorCommState next);
    void EnterCommState(MotorCommState state);
    void ExitCommState(MotorCommState state);

    void StateInit();
    void StateReady();
    void StateDisarmed();
    void StateBootRecovery();
    void StateArming();
    void StateArmed();
    void StateParking();
    void StateDisarming();
    void StateError();
    bool PollTorque(bool enabled, bool verify_only);
    size_t torque_index_ = 0;
    bool torque_busy_ = false;
    bool torque_off_verified_ = false;
    std::atomic<bool> torque_off_public_{false};
    enum class ArmPhase : uint8_t { WRITE_HOLD, VERIFY_HOLD, VERIFY_POSE, TORQUE };
    ArmPhase arm_phase_ = ArmPhase::WRITE_HOLD;
    size_t arm_index_ = 0;
    bool arm_busy_ = false;
    bool arm_stop_pending_ = false;
    int64_t arm_start_us_ = 0;
    static constexpr int64_t ARM_TIMEOUT_US = 5000000;
    MotorCommandFrame hold_command_{};
    enum class ParkPhase : uint8_t { RELEASE_SHOULDER, MOVE, SET_SHOULDER_GOAL, VERIFY_SHOULDER_GOAL, VERIFY_SHOULDER_POSITION, RESTORE_SHOULDER };
    ParkPhase park_phase_ = ParkPhase::RELEASE_SHOULDER;
    bool park_busy_ = false;
    bool park_stop_pending_ = false;
    MotorCommandFrame park_command_{};
    bool park_command_initialized_ = false;
    bool park_goal_busy_ = false;
    uint16_t park_shoulder_hold_count_ = 0;
    int64_t park_last_goal_us_ = 0;
    static constexpr int64_t PARK_GOAL_PERIOD_US = 10000;
    static constexpr int32_t PARK_MAX_TRACKING_ERROR = 160;
    static constexpr int32_t PARK_SHOULDER_COUPLING_TOLERANCE = 64;
    int64_t park_start_us_ = 0;
    uint8_t park_stable_count_ = 0;
    uint8_t park_stage_ = 0; // base/shoulder -> wrist -> roll -> elbow -> gripper
    static constexpr int64_t PARK_TIMEOUT_US = 15000000;
    static constexpr int32_t PARK_STEP_COUNTS = 3;
    static constexpr int16_t PARK_GOALS[MOTOR_COUNT] =
        {2047, 1023, -1, 3039, 2047, 2047, 2047};
    bool PollArmPositionChecks();
    bool PollParkShoulder(bool enabled);
    bool IsParkStageServo(size_t index) const;

    void CommIdle();
    void CommSendRequest();
    void CommWaitResponse();
    void CommNextMotor();
    void CommComplete();

private:
    static constexpr int64_t COMM_TIMEOUT_US = 30000;
    static constexpr int64_t ERROR_WINDOW_US = 1000000;
    static constexpr int64_t RX_RECOVERY_US = 2000;
    static constexpr uint32_t ERROR_THRESHOLD = 3;

    int64_t comm_cycle_start_us_ = 0;
    int64_t rx_recovery_start_us_ = 0;

    uint32_t comm_error_count_ = 0;
    int64_t first_error_us_ = 0;
    int64_t last_error_us_ = 0;

    // Prevent normal motor output after a failed cycle.
    bool output_inhibit_ = true;

    void RecordCommunicationError();
    void UpdateCommunicationError();

    void HandleCommunicationFailure();
    void EnterError();
    void CommRxRecovery();

public:
    void SetCommandFrame(const MotorCommandFrame& frame);
    bool SetJointTargets(const JointAngles& target);


    bool GetFeedbackFrame(
        SharedSnapshotFrame<MotorFeedbackFrame>& out);
    bool GetJointFeedback(JointAngles& actual);

    // Read-only preflight: copies fresh servo feedback into hold targets.
    // Does not publish a command or transmit UART writes.
    bool PreparePositionHold(MotorCommandFrame& hold);

    // Read-only ARM dry-run: fresh feedback and near-zero servo velocity.
    // Not a physical torque safety guarantee.
    bool PreviewPositionArm(MotorCommandFrame& hold);

private:
    MotorFeedbackFrame cycle_feedback_{};
    MotorCommandFrame cycle_command_{};

    bool command_valid_ = false;
    bool control_valid_ = false;  // edge-triggered PID reset on validity loss
    bool cycle_write_sent_ = false;

    static constexpr uint8_t PWM_OUTPUT_ADDRESS = 44;
    static constexpr uint8_t PWM_OUTPUT_SIZE = 2;

    void CommProcess();
    void CommSyncWrite();
    esp_err_t SendSyncPWM(
        const MotorCommandFrame& command);

    esp_err_t SendSyncPosition(
        const MotorCommandFrame& command
    );

private:
#if MOTOR_PERF_LOG_ENABLE

    struct MotorPerfStats
    {
        uint32_t task_count = 0;
        uint32_t cycle_count = 0;
        uint32_t error_count = 0;
        uint32_t cycle_timeout_count = 0;

        int64_t cycle_total_us = 0;
        int64_t cycle_max_us = 0;
        int64_t window_start_us = 0;
    };

    MotorPerfStats perf_{};

    void UpdatePerformanceLog();

#endif

private:
    // UART owner: used only inside MotorInterface INIT / ServoTask.
    ST3215ParameterFSM parameter_fsm_{};

private:
    static constexpr int64_t COMMAND_TIMEOUT_US = 100000;
    static constexpr int64_t FEEDBACK_TIMEOUT_US = 30000;

    static constexpr int16_t PWM_MIN = -1000;
    static constexpr int16_t PWM_MAX = 1000;

    bool ValidateCommand(
        const SharedSnapshotFrame<MotorCommandFrame>& frame,
        int64_t now_us) const;

    bool ValidateFeedback(
        const MotorFeedbackFrame& frame,
        int64_t now_us) const;

private:
    PIDController2 motor_pid_[MOTOR_COUNT]{};

    int64_t last_pid_update_us_ = 0;

    void InitializePID();
    void ResetPID();
    bool ProcessPID(float dt);
    void ChangeControlValidity(bool valid);


};
