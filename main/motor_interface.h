
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
#include "calibration_data.h"
#include "home_safety_config.h"


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
#define MOTOR_POSITION_OUTPUT_ENABLE 1 // SUPERVISED BENCH: hold, park and scoped base-only HOME
#define MOTOR_PHYSICAL_SERVO_ENABLE 1 // SUPERVISED BENCH: explicit physical enable
// The factory-style parking procedure temporarily releases shoulder ID13.
// This requires separate supervised mechanical validation before use.
#define MOTOR_VENDOR_SHOULDER_PARK_ENABLE 0 // temporarily locked after ID12/13 mismatch during real park
// Explicitly gated: actual Home motion has NOT passed physical safety validation.
#define MOTOR_HOME_EXECUTION_ENABLE 1 // supervised BASE_ONLY physical test branch
// Independent opt-in for releasing shoulder ID13 and running the full route.
#define MOTOR_HOME_ALLOW_FULL_EXECUTION 0
// Only for deterministic HOME logic faults with verified ID13 torque ON,
// fresh feedback and idle/PROCESS UART ownership. Never for UART faults.
#define MOTOR_HOME_SAFE_FAULT_HOLD_ENABLE 1
// Compile Home FSM internals without accepting T217 or transitioning into HOMING.
#define MOTOR_HOME_COMPILE_CHECK 0 // real BASE_ONLY build; FULL still gated OFF

#if MOTOR_HOME_COMPILE_CHECK && MOTOR_HOME_EXECUTION_ENABLE
#error "Home compile-check configuration must not enable physical Home execution"
#endif

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
    HOMING,
    FAULT_HOLD,
    DISARMING,
    BOOT_RECOVERY,
    ERROR
};

// Read-only operation lifecycle. This is diagnostic bookkeeping, not a safety-rated stop.
enum class MotorOperation : uint8_t { NONE = 0, PARK = 1, HOME = 2 };
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
    void RequestServoHome();
    // One-stage physical HOME and full HOME require separate explicit scope.
    enum class HomeScope : uint8_t { BASE_ONLY, FULL };
    bool RequestServoHomeScoped(HomeScope scope);
    bool RequestHomeHoldAbort();
    bool IsHomeExecutionEnabled() const { return MOTOR_HOME_EXECUTION_ENABLE != 0; }
    bool IsFullHomeExecutionEnabled() const {
        return MOTOR_HOME_EXECUTION_ENABLE && MOTOR_HOME_ALLOW_FULL_EXECUTION;
    }
    bool IsHomeSafeFaultHoldEnabled() const {
        return MOTOR_HOME_SAFE_FAULT_HOLD_ENABLE != 0;
    }
    // Read-only diagnostic: current commanded HOME positions during execution.
    bool GetHomeTargets(MotorCommandFrame& out) {
        return home_target_snapshot_.ReadData(out);
    }

    // Explicit supervised in-place Hold test; never runs in ERROR or HOMING.
    bool RequestCurrentHoldTest();
    // Supervised simulated fault, accepted only from stationary ARMED.
    bool RequestFaultHoldBenchTest();
    bool RequestFaultHoldBenchResume();
    enum class FaultHoldStatus : uint8_t {
        IDLE, REQUESTED, WRITE_SENT, STABLE, FAILED
    };
    FaultHoldStatus GetFaultHoldStatus() const { return fault_hold_status_.load(); }
    bool GetFaultHoldTarget(MotorCommandFrame& out) {
        return fault_hold_target_snapshot_.ReadData(out);
    }
    enum class HoldTestStatus : uint8_t {
        IDLE, PENDING, CAPTURED, WRITE_SENT, FEEDBACK_STABLE, REJECTED, FAILED
    };
    HoldTestStatus GetHoldTestStatus() const { return hold_test_status_.load(); }
    bool GetHoldTestTarget(MotorCommandFrame& out);

    MotorInterfaceState GetState() const { return state_public_.load(); }
    bool IsTorqueOffVerified() const { return torque_off_public_.load(); }
    // Initialization completion is intentionally false until a future
    // independently validated calibration/initialization coordinator exists.
    // This is NOT inferred from an ARM, PARK, or HOME operation result.
    bool IsInitializationComplete() const { return initialization_complete_.load(); }
    // Until a mechanical reference/offset has actually been verified,
    // this stays false. Neither HOME nor PARK is calibration.
    bool IsCalibrationValid() const { return calibration_valid_.load(); }
    uint32_t GetCalibrationSchemaVersion() const { return arm_calibration::kSchemaVersion; }
    bool HasVerifiedCalibrationRecord() const {
        return calibration_data_.mechanical_reference_verified &&
               arm_calibration::IsStructurallyValid(calibration_data_);
    }
    bool IsHomeDeveloperMode() const { return MOTOR_HOME_DEVELOPMENT_MODE != 0; }
    bool AreHomeLimitsVerified() const { return home_safety::AllLimitsVerified(); }
    bool IsFaultHoldOutputEnabled() const { return MOTOR_FAULT_HOLD_OUTPUT_ENABLE != 0; }
    // Latched diagnostic on ERROR entry. Not an active stop or torque proof.
    uint32_t GetHomeFaultDiagnostic() const { return home_fault_diagnostic_.load(); }
    bool CanExecuteMove() const {
        return GetState() == MotorInterfaceState::ARMED &&
               IsInitializationComplete() && IsCalibrationValid();
    }
    // Historical readback of the last confirmed ID13 torque state during HOME.
    // Not a real-time torque sensor; UNKNOWN if no confirmation exists.
    enum class HomeId13Verified : uint8_t { UNKNOWN, OFF, ON };
    HomeId13Verified GetHomeId13LastVerified() const { return home_id13_verified_.load(); }
    // Read-only, last observed Home step. 0=NOT_STARTED, 1..10=HomePhase+1,
    // 11=COMPLETED. Lower byte=phase, upper byte=rest stage (0..3).
    uint16_t GetHomeProgressSnapshot() const { return home_progress_snapshot_.load(); }
    // Read-only copy of the last complete seven-servo feedback cycle.
    // No UART request, command publication, calibration or EEPROM write.
    bool GetRawEncoderSnapshot(MotorFeedbackFrame& out, int64_t& oldest_age_us);

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
    // Fail-closed across reboot. Do not set true until calibration exists.
    std::atomic<bool> initialization_complete_{false};
    std::atomic<bool> calibration_valid_{false};
    // No trusted calibration input path or persistence yet. Starts empty.
    const arm_calibration::CalibrationData calibration_data_{};
    enum class ServoRequest : uint8_t {
        NONE, ON, OFF, PARK, HOME, HOME_HOLD_ABORT,
        HOLD_TEST, FAULT_HOLD_TEST, FAULT_HOLD_RESUME
    };
    std::atomic<FaultHoldStatus> fault_hold_status_{FaultHoldStatus::IDLE};
    SharedSnapshot<MotorCommandFrame> fault_hold_target_snapshot_{};
    uint8_t fault_hold_stable_cycles_ = 0;
    bool fault_hold_bench_test_ = false;
    std::atomic<HoldTestStatus> hold_test_status_{HoldTestStatus::IDLE};
    SharedSnapshot<MotorCommandFrame> hold_test_target_snapshot_{};
    uint8_t hold_test_stable_cycles_ = 0;
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
    void StateHoming();
    void StateFaultHold();
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
    bool IsHomeStageServo(size_t index) const;
    enum class HomePhase : uint8_t {
        PREPARE, MOVE_BASE, RELEASE_SHOULDER, MOVE_SHOULDER, SETTLE_SHOULDER,
        SET_SHOULDER_GOAL, VERIFY_SHOULDER_GOAL,
        VERIFY_SHOULDER_POSITION, RESTORE_SHOULDER, MOVE_REST
    };
    HomePhase home_phase_ = HomePhase::PREPARE;
    void PublishHomeProgress() {
        home_progress_snapshot_.store(
            static_cast<uint16_t>(static_cast<uint8_t>(home_phase_) + 1u) |
            (static_cast<uint16_t>(home_stage_) << 8));
    }
    MotorCommandFrame home_command_{};
    SharedSnapshot<MotorCommandFrame> home_target_snapshot_{};
    std::atomic<HomeScope> home_scope_requested_{HomeScope::BASE_ONLY};
    HomeScope home_scope_active_ = HomeScope::BASE_ONLY;
    bool home_abort_hold_pending_ = false;
    bool TryEnterHomeFaultHold(const MotorFeedbackFrame& feedback,
                               bool operator_abort = false);
    bool TryEnterHomeFaultHoldFromSnapshot(bool operator_abort = false);
    bool home_command_initialized_ = false;
    bool home_busy_ = false;
    bool home_stop_pending_ = false;
    std::atomic<HomeId13Verified> home_id13_verified_{HomeId13Verified::UNKNOWN};
    std::atomic<uint16_t> home_progress_snapshot_{0};
    // bit0=last failure happened in HOMING, bit1=fresh feedback at failure,
    // bit2=ID13 last verified OFF, bits8..15=HomePhase+1, bits16..23=ID13 verification.
    std::atomic<uint32_t> home_fault_diagnostic_{0};
    uint8_t home_stage_ = 0;
    uint8_t home_stable_count_ = 0;
    uint16_t home_shoulder_goal_ = 0;
    int64_t home_start_us_ = 0;
    int64_t home_shoulder_settle_us_ = 0;
    int64_t home_last_goal_us_ = 0;
    // Stage watchdog: reset on HomePhase or MOVE_REST joint change.
    HomePhase home_watchdog_phase_ = HomePhase::PREPARE;
    uint8_t home_watchdog_stage_ = 0;
    int64_t home_watchdog_start_us_ = 0;
    static constexpr int64_t HOME_MAX_TOTAL_US = 480000000; // 8 min hard upper bound
    static constexpr int64_t HOME_MOVE_STAGE_TIMEOUT_US = 90000000; // 90 s per joint
    static constexpr int64_t HOME_PARAMETER_TIMEOUT_US = 5000000; // 5 s
    static constexpr int64_t HOME_SETTLE_TIMEOUT_US = 3000000; // 3 s
    static constexpr int16_t HOME_GOALS[MOTOR_COUNT] =
        {2047, 2047, -1, 2047, 2047, 2047, 2047};

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

    // Read-only Home preflight. Does not change goal, calibration, or servo state.
    bool PreviewPositionHome(MotorCommandFrame& home);

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
