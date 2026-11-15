
#pragma once

#include <cstddef>
#include <atomic>
#include <cstdint>


#include <array>
#include <cstddef>
#include <cstdint>

#include "st3215.h"
#include "shared_snapshot.h"

#include "gpio_config.h"
#include "freertos/task.h"
#include "freertos/FreeRTOS.h"


#include "esp_timer.h"
#include "esp_err.h"

#include "driver/uart.h"



#define MOTOR_COUNT 6

#define MOTOR_IO_DIRECT 0
#define MOTOR_IO_TASK   1

#define MOTOR_IO_MODE MOTOR_IO_TASK

#define MOTOR_TASK_PERIOD_MS 10


#define MOTOR_PERF_LOG_ENABLE 1
#define MOTOR_PERF_LOG_PERIOD_US 1000000

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
    COMPLETE,
    RX_RECOVERY
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

    MotorFeedbackFrame cycle_feedback_{};

    int64_t last_comm_cycle_us_ = 0;

    static constexpr int64_t COMM_PERIOD_US = 10000;

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
};
