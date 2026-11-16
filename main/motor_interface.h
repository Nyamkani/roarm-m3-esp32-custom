
#pragma once

#include <cstddef>
#include <atomic>
#include <cstdint>


#include <array>
#include <cstddef>
#include <cstdint>

#include "st3215.h"
#include "shared_snapshot.h"
#include "st3215_parameter_fsm.h"


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

static constexpr int64_t CONTROL_PERIOD_US = 10000;
static constexpr int64_t CYCLE_BUDGET_US   = 5000;
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


#define MOTOR_EEPROM_WRITE_ENABLE 0

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
    RUNNING,
    ERROR
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

enum class ParameterState : uint8_t
{
    IDLE,

    READ_CURRENT,
    WAIT_CURRENT,

    CHECK_TORQUE,
    WAIT_TORQUE,

    EEPROM_UNLOCK,
    VERIFY_UNLOCK,

    WRITE_PARAMETER,

    READ_BACK,
    WAIT_READ_BACK,

    EEPROM_LOCK,
    VERIFY_LOCK,

    COMPLETE,
    ERROR
};

enum class ParameterResult : uint8_t
{
    IDLE,
    BUSY,
    SUCCESS,
    FAILED
};

enum class MotorInitState : uint8_t
{
    START,
    SET_MODE,
    WAIT_MODE,
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

    MotorCommState comm_state_ =
        MotorCommState::IDLE;

    size_t current_motor_index_ = 0;

    int64_t last_control_cycle_us_ = 0;

    static constexpr int64_t
        CONTROL_PERIOD_US = 10000;

    void ProcessInterfaceFSM();
    void ProcessCommunicationFSM();
    void ProcessControlCycle();

    void StateInit();
    void StateReady();
    void StateRunning();
    void StateError();

    void CommIdle();
    void CommSendRequest();
    void CommWaitResponse();
    void CommNextMotor();
    void CommComplete();

private:
    static constexpr int64_t COMM_TIMEOUT_US = 5000;
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

    bool GetFeedbackFrame(
        SharedSnapshotFrame<MotorFeedbackFrame>& out);

private:
    MotorFeedbackFrame cycle_feedback_{};
    MotorCommandFrame cycle_command_{};

    bool command_valid_ = false;
    bool cycle_write_sent_ = false;

    static constexpr uint8_t PWM_OUTPUT_ADDRESS = 44;
    static constexpr uint8_t PWM_OUTPUT_SIZE = 2;

    void CommProcess();
    void CommSyncWrite();
    esp_err_t SendSyncPWM(
        const MotorCommandFrame& command);

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

public:
    esp_err_t BeginSetParameter(
        uint8_t servo_id,
        ST3215Register address,
        const uint8_t* data,
        size_t data_size,
        RegisterStorage storage);

    ParameterResult GetParameterResult() const;

    void ProcessParameterFSM();

    esp_err_t SetOperatingMode(
        uint8_t servo_id,
        ST3215Mode mode);

    esp_err_t SetTorqueEnable(
        uint8_t servo_id,
        bool enable);

    esp_err_t SetTorqueLimit(
        uint8_t servo_id,
        uint16_t limit);

private:
    ST3215ParameterFSM parameter_fsm_{};

};
