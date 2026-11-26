
#include "motor_interface.h"


static const char* TAG = "motor_interface";

esp_err_t MotorInterface::Initialize(
    const board_handles_t& board)
{
    uart_num_ = board.servo_uart;
    parameter_fsm_.SetPort(uart_num_);

    // Initialize all motor objects
    for (size_t i = 0; i < MOTOR_COUNT; ++i)
    {
        esp_err_t ret = motors_[i].Initialize(SERVO_IDS[i]);

        if (ret != ESP_OK)
            return ret;
    }

    // Configure PID before ServoTask can run; initial INIT entry is one-shot.
    InitializePID();
    EnterState(MotorInterfaceState::INIT);

#if MOTOR_IO_MODE == MOTOR_IO_TASK

    BaseType_t ret = xTaskCreatePinnedToCore(
        MainTask,
        "ServoTask",
        4096,
        this,
        5,
        &task_handle_,
        0
    );

    if (ret != pdPASS)
        return ESP_FAIL;

#endif

    ESP_LOGI(TAG, "Motor interface initialized");
    return ESP_OK;
}

void MotorInterface::RequestServoOn() { servo_request_.store(ServoRequest::ON); }
void MotorInterface::RequestServoOff() { servo_request_.store(ServoRequest::OFF); }
void MotorInterface::RequestServoPark() { servo_request_.store(ServoRequest::PARK); }
void MotorInterface::RequestServoHome() { servo_request_.store(ServoRequest::HOME); }

bool MotorInterface::RequestServoHomeScoped(HomeScope scope)
{
#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_POSITION && MOTOR_HOME_EXECUTION_ENABLE && MOTOR_TORQUE_WRITE_ENABLE && MOTOR_POSITION_OUTPUT_ENABLE
    if (GetState() != MotorInterfaceState::ARMED ||
        (scope == HomeScope::FULL && !MOTOR_HOME_ALLOW_FULL_EXECUTION))
        return false;
    ServoRequest expected = ServoRequest::NONE;
    home_scope_requested_.store(scope);
    return servo_request_.compare_exchange_strong(expected, ServoRequest::HOME);
#else
    (void)scope;
    return false;
#endif
}

bool MotorInterface::RequestHomeHoldAbort()
{
#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_POSITION && MOTOR_HOME_EXECUTION_ENABLE
    if (GetState() != MotorInterfaceState::HOMING)
        return false;
    ServoRequest expected = ServoRequest::NONE;
    return servo_request_.compare_exchange_strong(expected, ServoRequest::HOME_HOLD_ABORT);
#else
    return false;
#endif
}

// This is a supervised stationary HOLD test, not general MOVE or
// autonomous error recovery. The ServoTask owns all UART1 operations.
bool MotorInterface::RequestCurrentHoldTest()
{
#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_POSITION && MOTOR_POSITION_OUTPUT_ENABLE
    if (GetState() != MotorInterfaceState::ARMED)
        return false;
    const auto old_status = hold_test_status_.load();
    if (old_status == HoldTestStatus::PENDING ||
        old_status == HoldTestStatus::CAPTURED ||
        old_status == HoldTestStatus::WRITE_SENT)
        return false;
    ServoRequest expected = ServoRequest::NONE;
    if (!servo_request_.compare_exchange_strong(expected, ServoRequest::HOLD_TEST))
        return false;
    hold_test_status_.store(HoldTestStatus::PENDING);
    return true;
#else
    return false;
#endif
}

bool MotorInterface::RequestFaultHoldBenchTest()
{
#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_POSITION && MOTOR_POSITION_OUTPUT_ENABLE
    if (GetState() != MotorInterfaceState::ARMED)
        return false;
    ServoRequest expected = ServoRequest::NONE;
    if (!servo_request_.compare_exchange_strong(expected, ServoRequest::FAULT_HOLD_TEST))
        return false;
    fault_hold_status_.store(FaultHoldStatus::REQUESTED);
    return true;
#else
    return false;
#endif
}

bool MotorInterface::RequestFaultHoldBenchResume()
{
    if (GetState() != MotorInterfaceState::FAULT_HOLD ||
        !fault_hold_bench_test_ ||
        fault_hold_status_.load() != FaultHoldStatus::STABLE)
        return false;
    ServoRequest expected = ServoRequest::NONE;
    return servo_request_.compare_exchange_strong(
        expected, ServoRequest::FAULT_HOLD_RESUME);
}

bool MotorInterface::GetHoldTestTarget(MotorCommandFrame& out)
{
    return hold_test_target_snapshot_.ReadData(out);
}

ST3215* MotorInterface::GetMotor(size_t index)
{
    if (index >= MOTOR_COUNT)
        return nullptr;

    return &motors_[index];
}

void MotorInterface::SetCommandFrame(
    const MotorCommandFrame& frame)
{
    command_snapshot_.Write(frame);
}

bool MotorInterface::SetJointTargets(
    const JointAngles& target)
{
    ServoPositions positions{};

    // 6개 관절 각도 → 7개 서보 목표 엔코더
    if (!JointMapper::ToServoPositions(target, positions))
    {
        return false;
    }

    MotorCommandFrame command{};

    for (size_t i = 0; i < MOTOR_COUNT; ++i)
    {
        command.motors[i].target_position =
            positions.counts[i];
    }

    // 기존 Snapshot 경로 재사용
    SetCommandFrame(command);

    return true;
}


bool MotorInterface::GetFeedbackFrame(
    SharedSnapshotFrame<MotorFeedbackFrame>& out)
{
    return feedback_snapshot_.Read(out);
}

bool MotorInterface::GetRawEncoderSnapshot(
    MotorFeedbackFrame& out, int64_t& oldest_age_us)
{
    out = {};
    oldest_age_us = -1;

    SharedSnapshotFrame<MotorFeedbackFrame> frame{};
    if (!GetFeedbackFrame(frame))
        return false;

    const int64_t now_us = esp_timer_get_time();
    if (!ValidateFeedback(frame.data, now_us))
        return false;

    int64_t maximum_age_us = 0;
    for (size_t i = 0; i < MOTOR_COUNT; ++i)
    {
        const MotorFeedback& servo = frame.data.motors[i];
        if (servo.position < 0 || servo.position > 4095)
            return false;
        const int64_t age_us = now_us - servo.timestamp_us;
        if (age_us > maximum_age_us)
            maximum_age_us = age_us;
    }

    out = frame.data;
    oldest_age_us = maximum_age_us;
    return true;
}

bool MotorInterface::PreparePositionHold(MotorCommandFrame& hold)
{
    hold = {};

#if MOTOR_CONTROL_MODE != MOTOR_CONTROL_POSITION
    return false;
#else
    if (GetState() != MotorInterfaceState::DISARMED)
        return false;

    SharedSnapshotFrame<MotorFeedbackFrame> feedback{};
    if (!GetFeedbackFrame(feedback))
        return false;

    const int64_t now_us = esp_timer_get_time();
    if (!ValidateFeedback(feedback.data, now_us))
        return false;

    for (size_t i = 0; i < MOTOR_COUNT; ++i)
    {
        const int32_t position = feedback.data.motors[i].position;
        if (position < 0 || position > 4095)
        {
            hold = {};
            return false;
        }
        hold.motors[i].target_position = static_cast<int16_t>(position);
        hold.motors[i].target_velocity = 0;
        hold.motors[i].output = 0;
    }
    return true;
#endif
}

bool MotorInterface::PreviewPositionArm(MotorCommandFrame& hold)
{
    // This preview neither changes command snapshots nor writes to servos.
    if (!PreparePositionHold(hold))
        return false;

#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_POSITION
    SharedSnapshotFrame<MotorFeedbackFrame> feedback{};
    if (!GetFeedbackFrame(feedback) ||
        !ValidateFeedback(feedback.data, esp_timer_get_time()))
    {
        hold = {};
        return false;
    }

    // Do not preview ARM while any joint is visibly moving. This is only
    // a conservative diagnostic gate, not the final torque-on preflight.
    constexpr int32_t kMaxAbsSpeed = 50;
    constexpr int32_t kMaxPositionDrift = 4;
    for (size_t i = 0; i < MOTOR_COUNT; ++i)
    {
        const int32_t velocity = feedback.data.motors[i].velocity;
        const int32_t position = feedback.data.motors[i].position;
        const int32_t captured = hold.motors[i].target_position;
        if (velocity < -kMaxAbsSpeed || velocity > kMaxAbsSpeed ||
            position - captured < -kMaxPositionDrift ||
            position - captured > kMaxPositionDrift)
        {
            hold = {};
            return false;
        }
    }
    return true;
#else
    hold = {};
    return false;
#endif
}

bool MotorInterface::PreviewPositionHome(MotorCommandFrame& home)
{
    home = {};
#if MOTOR_CONTROL_MODE != MOTOR_CONTROL_POSITION
    return false;
#else
    if (GetState() != MotorInterfaceState::ARMED)
        return false;

    SharedSnapshotFrame<MotorFeedbackFrame> snapshot{};
    if (!GetFeedbackFrame(snapshot) ||
        !ValidateFeedback(snapshot.data, esp_timer_get_time()))
        return false;

    // Require a stationary, mechanically coupled shoulder before a future
    // active Home sequence may release ID13. No servo writes here.
    constexpr int32_t kMaxAbsSpeed = 50;
    int32_t shoulder_sum = 0;
    for (size_t i = 0; i < MOTOR_COUNT; ++i)
    {
        const int32_t position = snapshot.data.motors[i].position;
        const int32_t velocity = snapshot.data.motors[i].velocity;
        if (!home_safety::CheckRawJointLimit(i, position) ||
            velocity < -kMaxAbsSpeed || velocity > kMaxAbsSpeed)
            return false;
        if (i == 1 || i == 2) shoulder_sum += position;
    }
    if (std::abs(shoulder_sum - 4094) > PARK_SHOULDER_COUPLING_TOLERANCE)
        return false;

    for (size_t i = 0; i < MOTOR_COUNT; ++i)
    {
        if (HOME_GOALS[i] >= 0 &&
            !home_safety::CheckRawJointLimit(i, HOME_GOALS[i]))
            return false;
        home.motors[i].target_position = HOME_GOALS[i];
        home.motors[i].target_velocity = 0;
        home.motors[i].output = 0;
    }
    return true;
#endif
}

bool MotorInterface::GetJointFeedback(
    JointAngles& actual)
{
    SharedSnapshotFrame<MotorFeedbackFrame> frame{};

    // 최신 피드백 Snapshot 조회
    if (!GetFeedbackFrame(frame))
    {
        return false;
    }

    const int64_t now_us = esp_timer_get_time();

    // 7개 서보의 유효성 및 Freshness 확인
    if (!ValidateFeedback(frame.data, now_us))
    {
        return false;
    }

    ServoPositions positions{};

    for (size_t i = 0; i < MOTOR_COUNT; ++i)
    {
        positions.counts[i] =
            frame.data.motors[i].position;
    }

    // 7개 서보 엔코더 → 6개 실제 관절 각도
    return JointMapper::ToJointAngles(
        positions,
        actual
    );
}

void MotorInterface::MainTask(void* arg)
{
    auto* self = static_cast<MotorInterface*>(arg);
    self->MainLoop();

    self->task_handle_ = nullptr;
    vTaskDelete(nullptr);
}

void MotorInterface::MainLoop()
{
    // This ServoTask is designed for a 1 ms FreeRTOS scheduling period.
    // A 100 Hz tick would truncate pdMS_TO_TICKS(1) to zero and trigger
    // the xTaskDelayUntil(xTimeIncrement > 0) assertion at runtime.
    // Fail at build time instead of silently downsampling the control loop.
    static_assert(pdMS_TO_TICKS(MOTOR_TASK_PERIOD_MS) > 0,
                  "MotorInterface requires a nonzero FreeRTOS tick delay; set CONFIG_FREERTOS_HZ=1000");
    constexpr TickType_t kTaskPeriodTicks = pdMS_TO_TICKS(MOTOR_TASK_PERIOD_MS);
    TickType_t last_wake_time = xTaskGetTickCount();

    while (!task_stop_.load())
    {
#if MOTOR_PERF_LOG_ENABLE
        ++perf_.task_count;
#endif

        ProcessInterfaceFSM();

#if MOTOR_PERF_LOG_ENABLE
        UpdatePerformanceLog();
#endif

        xTaskDelayUntil(
            &last_wake_time,
            kTaskPeriodTicks
        );
    }
}


esp_err_t MotorInterface::SendRequest(
    const uint8_t* tx_buffer,
    size_t tx_len,
    TickType_t timeout_ticks)
{
    if (tx_buffer == nullptr ||
        tx_len == 0 ||
        tx_len > ST3215::MAX_PACKET_SIZE)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // UART TX
    const int written = uart_write_bytes(
        uart_num_,
        tx_buffer,
        tx_len
    );

    if (written < 0 ||
        static_cast<size_t>(written) != tx_len)
    {
        return ESP_FAIL;
    }

    // Wait for UART TX completion
    return uart_wait_tx_done(
        uart_num_,
        timeout_ticks
    );
}

// --------------------------------------------------
// RX initialization
// --------------------------------------------------

void MotorInterface::BeginReceive(
    uint8_t expected_id,
    size_t expected_params_size)
{
    rx_context_ = {};

    rx_context_.expected_id = expected_id;
    rx_context_.expected_params_size = expected_params_size;

    rx_context_.start_us = esp_timer_get_time();
    rx_context_.active = true;
}

// --------------------------------------------------
// RX reset
// --------------------------------------------------

void MotorInterface::ResetReceive()
{
    rx_context_ = {};
}

// --------------------------------------------------
// UART RX polling
// --------------------------------------------------

ServoReceiveStatus MotorInterface::ReceiveResponse(ST3215Response& response, int64_t& completed_us)
{
    completed_us = 0;

    if (!rx_context_.active)
    {
        return ServoReceiveStatus::ERROR;
    }

    auto& rx = rx_context_;

    // Read as much as possible without blocking.
    while (true)
    {
        size_t target_size = 4;

        if (rx.expected_packet_size != 0)
        {
            target_size = rx.expected_packet_size;
        }

        if (rx.size >= target_size)
        {
            // First four bytes are ready.
            if (rx.expected_packet_size == 0)
            {
                if (rx.buffer[0] != 0xFF ||
                    rx.buffer[1] != 0xFF)
                {
                    ResetReceive();
                    return ServoReceiveStatus::ERROR;
                }

                if (rx.buffer[2] != rx.expected_id)
                {
                    ResetReceive();
                    return ServoReceiveStatus::ERROR;
                }

                const size_t length = rx.buffer[3];

                if (length < 2)
                {
                    ResetReceive();
                    return ServoReceiveStatus::ERROR;
                }

                const size_t packet_size = length + 4;

                if (packet_size > rx.buffer.size() ||
                    length != rx.expected_params_size + 2)
                {
                    ResetReceive();
                    return ServoReceiveStatus::ERROR;
                }

                rx.expected_packet_size = packet_size;
                continue;
            }

            // Complete packet received.
            rx.completed_us = esp_timer_get_time();

            const esp_err_t result = ST3215::ParseResponse(
                rx.buffer.data(),
                rx.size,
                rx.expected_id,
                rx.expected_params_size,
                response
            );

            completed_us = rx.completed_us;

            ResetReceive();

            if (result != ESP_OK)
            {
                return ServoReceiveStatus::ERROR;
            }

            return ServoReceiveStatus::COMPLETE;
        }

        const size_t remaining = target_size - rx.size;

        const int received = uart_read_bytes(
            uart_num_,
            rx.buffer.data() + rx.size,
            remaining,
            0
        );

        if (received < 0)
        {
            ResetReceive();
            return ServoReceiveStatus::ERROR;
        }

        if (received == 0)
        {
            if (esp_timer_get_time() - rx.start_us >=
                RESPONSE_TIMEOUT_US)
            {
                ResetReceive();
                return ServoReceiveStatus::TIMEOUT;
            }

            break;
        }

        rx.size += static_cast<size_t>(received);

    }

    return ServoReceiveStatus::WAITING;
}




void MotorInterface::ChangeState(MotorInterfaceState next)
{
    if (state_ == next)
        return;

    // Publish a separate operation result without changing motor behavior.
    // PARK completes only after DISARMING verifies all servo torque is OFF.
    const auto operation = GetOperationSnapshot();
    if (next == MotorInterfaceState::PARKING)
        SetOperationSnapshot(MotorOperation::PARK, MotorOperationStatus::RUNNING);
    else if (state_ == MotorInterfaceState::PARKING &&
             next == MotorInterfaceState::DISARMING && park_stop_pending_)
        SetOperationSnapshot(MotorOperation::PARK, MotorOperationStatus::INTERRUPTED);
    else if (state_ == MotorInterfaceState::DISARMING &&
             next == MotorInterfaceState::DISARMED &&
             operation.operation == MotorOperation::PARK &&
             operation.status == MotorOperationStatus::RUNNING)
        SetOperationSnapshot(MotorOperation::PARK, MotorOperationStatus::COMPLETED);
    else if (next == MotorInterfaceState::HOMING)
        SetOperationSnapshot(MotorOperation::HOME, MotorOperationStatus::RUNNING);
    else if (state_ == MotorInterfaceState::HOMING &&
             next == MotorInterfaceState::ARMED &&
             operation.operation == MotorOperation::HOME &&
             operation.status == MotorOperationStatus::RUNNING)
        SetOperationSnapshot(MotorOperation::HOME, MotorOperationStatus::COMPLETED);
    else if (state_ == MotorInterfaceState::HOMING &&
             next == MotorInterfaceState::DISARMING && home_stop_pending_)
        SetOperationSnapshot(MotorOperation::HOME, MotorOperationStatus::INTERRUPTED);
    else if (next == MotorInterfaceState::ERROR &&
             operation.operation == MotorOperation::HOME &&
             (operation.status == MotorOperationStatus::RUNNING ||
              operation.status == MotorOperationStatus::INTERRUPTED))
        SetOperationSnapshot(MotorOperation::HOME, MotorOperationStatus::FAILED);
    else if (next == MotorInterfaceState::ERROR &&
             operation.operation == MotorOperation::PARK &&
             (operation.status == MotorOperationStatus::RUNNING ||
              operation.status == MotorOperationStatus::INTERRUPTED))
        SetOperationSnapshot(MotorOperation::PARK, MotorOperationStatus::FAILED);

    ExitState(state_);
    state_ = next;
    state_public_.store(next);
    EnterState(next);
}

void MotorInterface::EnterState(MotorInterfaceState state)
{
    switch (state)
    {
        case MotorInterfaceState::INIT:
            init_state_ = MotorInitState::START;
            init_motor_index_ = 0;
            output_inhibit_ = true;
            command_valid_ = false;
            control_valid_ = false;
            init_mismatch_count_ = 0;
            init_torque_on_count_ = 0;
            ResetPID();
            break;
        case MotorInterfaceState::BOOT_RECOVERY:
            // External servos may still have torque enabled after MCU reset.
            // Do not write new positions, re-arm or automatically cut torque.
            torque_off_verified_ = false;
            torque_off_public_.store(false);
            output_inhibit_ = true;
            command_valid_ = false;
            control_valid_ = false;
            ChangeCommState(MotorCommState::IDLE);
            last_comm_cycle_us_ = 0;
            break;
        case MotorInterfaceState::READY:
            torque_off_public_.store(false);
            output_inhibit_ = true;
            torque_index_ = 0;
            torque_busy_ = false;
            torque_off_verified_ = false;
            break;
        case MotorInterfaceState::DISARMED:
            park_stop_pending_ = false;
            hold_command_ = {};
            output_inhibit_ = true;
            command_valid_ = false;
            control_valid_ = false;
            cycle_command_ = {};
            ResetPID();
            ChangeCommState(MotorCommState::IDLE);
            last_comm_cycle_us_ = 0;
            break;
        case MotorInterfaceState::ARMING:
            torque_off_verified_ = false;
            torque_off_public_.store(false);
            output_inhibit_ = true;
            ResetPID();
            arm_phase_ = ArmPhase::WRITE_HOLD;
            arm_index_ = 0;
            arm_busy_ = false;
            arm_stop_pending_ = false;
            arm_start_us_ = esp_timer_get_time();
            torque_index_ = 0;
            torque_busy_ = false;
            break;
        case MotorInterfaceState::ARMED:
            ResetPID();
            control_valid_ = false;
            command_valid_ = false;
            output_inhibit_ = true;
            ChangeCommState(MotorCommState::IDLE);
            last_comm_cycle_us_ = 0;
            break;
        case MotorInterfaceState::FAULT_HOLD:
            // Captured hold_command_ is maintained, but normal MOVE and
            // HOME remain unavailable. No torque write or EEPROM change.
            output_inhibit_ = true;
            fault_hold_stable_cycles_ = 0;
            ChangeCommState(MotorCommState::IDLE);
            last_comm_cycle_us_ = 0;
            break;
        case MotorInterfaceState::PARKING:
            park_stop_pending_ = false;
            park_command_ = {};
            park_command_initialized_ = false;
            park_goal_busy_ = false;
            park_shoulder_hold_count_ = 0;
            park_last_goal_us_ = 0;
            output_inhibit_ = true;
            park_phase_ = ParkPhase::RELEASE_SHOULDER;
            park_start_us_ = esp_timer_get_time();
            park_stable_count_ = 0;
            park_stage_ = 0;
            park_busy_ = false;
            ChangeCommState(MotorCommState::IDLE);
            break;
        case MotorInterfaceState::HOMING:
            home_scope_active_ = home_scope_requested_.load();
            home_abort_hold_pending_ = false;
            home_phase_ = HomePhase::PREPARE;
            home_progress_snapshot_.store(1u); // PREPARE, stage 0
            home_command_ = {};
            home_command_initialized_ = false;
            home_busy_ = false;
            park_busy_ = false; // PollParkShoulder transaction helper
            home_stop_pending_ = false;
            // ARMING has verified torque ON for all 7 before entering ARMED.
            // Historical command confirmation, NOT a live torque sensor.
            home_id13_verified_.store(HomeId13Verified::ON);
            home_stage_ = 0;
            home_stable_count_ = 0;
            home_shoulder_goal_ = 0;
            home_start_us_ = esp_timer_get_time();
            home_shoulder_settle_us_ = 0;
            home_last_goal_us_ = 0;
            home_watchdog_phase_ = HomePhase::PREPARE;
            home_watchdog_stage_ = 0;
            home_watchdog_start_us_ = home_start_us_;
            output_inhibit_ = true;
            ChangeCommState(MotorCommState::IDLE);
            break;
        case MotorInterfaceState::DISARMING:
            torque_off_verified_ = false;
            torque_off_public_.store(false);
            output_inhibit_ = true;
            ResetPID();
            torque_index_ = 0;
            torque_busy_ = false;
            break;
        case MotorInterfaceState::ERROR:
            torque_off_public_.store(false);
            output_inhibit_ = true;
            torque_off_verified_ = false;
            command_valid_ = false;
            control_valid_ = false;
            cycle_command_ = {};
            ResetPID();
            break;
    }
}

void MotorInterface::ExitState(MotorInterfaceState state)
{
    if (state == MotorInterfaceState::ARMED)
        output_inhibit_ = true;
}

void MotorInterface::ChangeCommState(MotorCommState next)
{
    if (comm_state_ == next)
        return;

    ExitCommState(comm_state_);
    comm_state_ = next;
    EnterCommState(next);
}

void MotorInterface::EnterCommState(MotorCommState state)
{
    // Only on transition: avoid restarting the receive/recovery timers per poll.
    if (state == MotorCommState::RX_RECOVERY)
    {
        rx_recovery_start_us_ = esp_timer_get_time();
        output_inhibit_ = true;
    }
}

void MotorInterface::ExitCommState(MotorCommState state)
{
    if (state == MotorCommState::WAIT_RESPONSE)
        ResetReceive();
}

void MotorInterface::ProcessInterfaceFSM()
{
    switch (state_)
    {
        case MotorInterfaceState::INIT: StateInit(); break;
        case MotorInterfaceState::READY: StateReady(); break;
        case MotorInterfaceState::DISARMED: StateDisarmed(); break;
        case MotorInterfaceState::BOOT_RECOVERY: StateBootRecovery(); break;
        case MotorInterfaceState::ARMING: StateArming(); break;
        case MotorInterfaceState::ARMED: StateArmed(); break;
        case MotorInterfaceState::PARKING: StateParking(); break;
        case MotorInterfaceState::HOMING: StateHoming(); break;
        case MotorInterfaceState::FAULT_HOLD: StateFaultHold(); break;
        case MotorInterfaceState::DISARMING: StateDisarming(); break;
        case MotorInterfaceState::ERROR: StateError(); break;
    }
}

void MotorInterface::StateInit()
{
    ProcessMotorInitialization();

    if (state_ != MotorInterfaceState::INIT ||
        init_state_ != MotorInitState::COMPLETE)
    {
        return;
    }

    ESP_LOGI(
        TAG,
        "INIT diagnosis complete: mismatches=%u",
        static_cast<unsigned>(init_mismatch_count_)
    );

    ESP_LOGI(TAG, "INIT torque ON count=%u",
             static_cast<unsigned>(init_torque_on_count_));

    // Prioritize detection of retained torque over operating-mode mismatch.
    // Never force EEPROM writes or falsely label the arm DISARMED.
    if (init_torque_on_count_ != 0)
    {
        boot_recheck_mode_after_off_ = true;
        ESP_LOGW(TAG,
                 "BOOT_RECOVERY: torque ON=%u/7, mode mismatches=%u; "
                 "support arm, then use T=215 cmd=0",
                 static_cast<unsigned>(init_torque_on_count_),
                 static_cast<unsigned>(init_mismatch_count_));
        ChangeCommState(MotorCommState::IDLE);
        current_motor_index_ = 0;
        ChangeState(MotorInterfaceState::BOOT_RECOVERY);
        return;
    }
    if (init_mismatch_count_ != 0)
    {
        ESP_LOGW(TAG, "INIT rejected: operating mode mismatch with torque OFF");
        EnterError();
        return;
    }

    ChangeCommState(MotorCommState::IDLE);
    current_motor_index_ = 0;
    ChangeState(MotorInterfaceState::READY);
}

bool MotorInterface::PollTorque(bool enabled, bool verify_only)
{
    if (torque_index_ >= MOTOR_COUNT)
        return true;
    if (!torque_busy_)
    {
        const uint8_t id = motors_[torque_index_].GetID();
        const esp_err_t ret = verify_only ?
            parameter_fsm_.VerifyTorqueEnable(id, enabled) :
            parameter_fsm_.SetTorqueEnable(id, enabled);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Torque transaction request failed id=%u", static_cast<unsigned>(id));
            if (state_ == MotorInterfaceState::ARMING)
                ChangeState(MotorInterfaceState::DISARMING);
            else
                EnterError();
            return false;
        }
        torque_busy_ = true;
    }
    const auto result = parameter_fsm_.Poll();
    if (result == ST3215ParameterFSM::Result::FAILED)
    {
        ESP_LOGE(TAG, "Torque verification failed id=%u err=%s",
                 static_cast<unsigned>(parameter_fsm_.GetServoID()),
                 esp_err_to_name(parameter_fsm_.GetError()));
        if (state_ == MotorInterfaceState::ARMING)
            ChangeState(MotorInterfaceState::DISARMING);
        else
            EnterError();
        return false;
    }
    if (result == ST3215ParameterFSM::Result::SUCCESS)
    {
        torque_busy_ = false;
        ++torque_index_;
    }
    return torque_index_ >= MOTOR_COUNT;
}

void MotorInterface::StateReady()
{
    // No automatic torque OFF at boot: first verify all seven are OFF by READ.
    if (PollTorque(false, true))
    {
        torque_off_verified_ = true;
        torque_off_public_.store(true);
        ChangeState(MotorInterfaceState::DISARMED);
    }
}

void MotorInterface::StateBootRecovery()
{
    // Never silently switch off torque on MCU reboot: an unsupported arm
    // could fall. Host must explicitly authorize no-park torque release.
    if (comm_state_ == MotorCommState::IDLE &&
        servo_request_.exchange(ServoRequest::NONE) == ServoRequest::OFF)
    {
        ESP_LOGW(TAG, "BOOT_RECOVERY explicit torque OFF requested");
        ChangeState(MotorInterfaceState::DISARMING);
        return;
    }
    UpdateCommunicationError();
    ProcessCommunicationFSM();  // feedback only, no position output
}

void MotorInterface::StateDisarmed()
{
    if (comm_state_ == MotorCommState::IDLE)
    {
        const ServoRequest req = servo_request_.exchange(ServoRequest::NONE);
        if (req == ServoRequest::ON)
        {
#if MOTOR_TORQUE_WRITE_ENABLE
#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_POSITION
            MotorCommandFrame hold{};
            if (torque_off_verified_ && PreviewPositionArm(hold))
            {
                hold_command_ = hold;
                ChangeState(MotorInterfaceState::ARMING);
                return;
            }
            ESP_LOGW(TAG, "ARM rejected: current pose invalid or moving");
#else
            SharedSnapshotFrame<MotorCommandFrame> cmd{};
            SharedSnapshotFrame<MotorFeedbackFrame> fb{};
            const int64_t now = esp_timer_get_time();
            if (torque_off_verified_ &&
                command_snapshot_.Read(cmd) && ValidateCommand(cmd, now) &&
                feedback_snapshot_.Read(fb) && ValidateFeedback(fb.data, now))
            {
                ChangeState(MotorInterfaceState::ARMING);
                return;
            }
            ESP_LOGW(TAG, "ARM rejected: invalid command, feedback or torque OFF");
#endif
#else
            ESP_LOGW(TAG, "ARM rejected: MOTOR_TORQUE_WRITE_ENABLE=0");
#endif
        }
    }
    UpdateCommunicationError();
    ProcessCommunicationFSM();
}

bool MotorInterface::PollArmPositionChecks()
{
    if (arm_index_ >= MOTOR_COUNT) return true;
    const uint8_t id = motors_[arm_index_].GetID();
    const uint16_t goal =
        static_cast<uint16_t>(hold_command_.motors[arm_index_].target_position);
    if (!arm_busy_)
    {
        esp_err_t err = ESP_ERR_INVALID_STATE;
        if (arm_phase_ == ArmPhase::WRITE_HOLD)
            err = parameter_fsm_.SetGoalPosition(id, goal);
        else if (arm_phase_ == ArmPhase::VERIFY_HOLD)
            err = parameter_fsm_.VerifyGoalPosition(id, goal);
        else if (arm_phase_ == ArmPhase::VERIFY_POSE)
            err = parameter_fsm_.VerifyPresentPosition(id, goal);
        if (err != ESP_OK)
        {
            ChangeState(MotorInterfaceState::DISARMING);
            return false;
        }
        arm_busy_ = true;
    }
    const auto result = parameter_fsm_.Poll();
    if (result == ST3215ParameterFSM::Result::FAILED)
    {
        ESP_LOGE(TAG, "ARM goal/pose verification failed servo=%u", id);
        ChangeState(MotorInterfaceState::DISARMING);
        return false;
    }
    if (result == ST3215ParameterFSM::Result::SUCCESS)
    {
        arm_busy_ = false;
        ++arm_index_;
    }
    return arm_index_ >= MOTOR_COUNT;
}

void MotorInterface::StateArming()
{
#if MOTOR_TORQUE_WRITE_ENABLE
    if (esp_timer_get_time() - arm_start_us_ > ARM_TIMEOUT_US)
    {
        ESP_LOGE(TAG, "ARM timeout: torque may be partially enabled");
        // If an existing parameter transaction is in flight, it is unsafe
        // to hand UART1 to rollback while its response remains outstanding.
        // Report ERROR rather than asserting a successful torque release.
        if (arm_busy_ || torque_busy_)
            EnterError();
        else
            ChangeState(MotorInterfaceState::DISARMING);
        return;
    }
    if (servo_request_.exchange(ServoRequest::NONE) == ServoRequest::OFF)
        arm_stop_pending_ = true;

    if (arm_stop_pending_)
    {
        // Never hand UART1 to DISARMING while an ARM transaction is in flight.
        // Finish its current readback, then attempt all-servo OFF rollback.
        if (arm_busy_ || torque_busy_)
        {
            const auto result = parameter_fsm_.Poll();
            if (result == ST3215ParameterFSM::Result::BUSY)
                return;
            if (result == ST3215ParameterFSM::Result::FAILED)
            {
                ESP_LOGE(TAG, "ARM cancellation transaction failed; torque unknown");
                EnterError();
                return;
            }
            arm_busy_ = false;
            torque_busy_ = false;
        }
        ChangeState(MotorInterfaceState::DISARMING);
        return;
    }
#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_POSITION
    if (arm_phase_ != ArmPhase::TORQUE)
    {
        if (!PollArmPositionChecks() ||
            state_ != MotorInterfaceState::ARMING) return;
        arm_index_ = 0;
        arm_busy_ = false;
        if (arm_phase_ == ArmPhase::WRITE_HOLD)
            arm_phase_ = ArmPhase::VERIFY_HOLD;
        else if (arm_phase_ == ArmPhase::VERIFY_HOLD)
            arm_phase_ = ArmPhase::VERIFY_POSE;
        else
            arm_phase_ = ArmPhase::TORQUE;
        return;
    }
#endif
    if (PollTorque(true, false))
    {
        torque_off_verified_ = false;
        torque_off_public_.store(false);
        ESP_LOGI(TAG, "Servo ON verified (all 7)");
        ChangeState(MotorInterfaceState::ARMED);
    }
#else
    EnterError();
#endif
}

bool MotorInterface::IsParkStageServo(size_t index) const
{
    // Follow vendor order rather than sweeping all joints together:
    // base + driving shoulder, then wrist, roll, elbow, gripper.
    switch (park_stage_)
    {
        case 0: return index == 0 || index == 1;
        case 1: return index == 4;
        case 2: return index == 5;
        case 3: return index == 3;
        case 4: return index == 6;
        default: return false;
    }
}

bool MotorInterface::PollParkShoulder(bool enabled)
{
    if (!park_busy_)
    {
        if (parameter_fsm_.SetTorqueEnable(13, enabled) != ESP_OK)
        {
            EnterError();
            return false;
        }
        park_busy_ = true;
    }
    auto result = parameter_fsm_.Poll();
    if (result == ST3215ParameterFSM::Result::FAILED)
    {
        EnterError();
        return false;
    }
    if (result == ST3215ParameterFSM::Result::SUCCESS)
    {
        park_busy_ = false;
        return true;
    }
    return false;
}

void MotorInterface::StateArmed()
{
    if (comm_state_ == MotorCommState::IDLE)
    {
        const ServoRequest request = servo_request_.exchange(ServoRequest::NONE);
        if (request == ServoRequest::OFF)
        {
            ChangeState(MotorInterfaceState::DISARMING);
            return;
        }
        if (request == ServoRequest::FAULT_HOLD_TEST)
        {
            SharedSnapshotFrame<MotorFeedbackFrame> snapshot{};
            const int64_t now_us = esp_timer_get_time();
            bool valid = GetFeedbackFrame(snapshot) &&
                ValidateFeedback(snapshot.data, now_us);
            if (valid)
            {
                const int32_t sum = snapshot.data.motors[1].position +
                    snapshot.data.motors[2].position;
                if (std::abs(sum - 4094) > PARK_SHOULDER_COUPLING_TOLERANCE)
                    valid = false;
                for (size_t i = 0; i < MOTOR_COUNT && valid; ++i)
                {
                    const auto& fb = snapshot.data.motors[i];
                    if (fb.position < 0 || fb.position > 4095 ||
                        std::abs(static_cast<int32_t>(fb.velocity)) > 50)
                        valid = false;
                }
            }
            if (!valid)
            {
                fault_hold_status_.store(FaultHoldStatus::FAILED);
                return;
            }
            for (size_t i = 0; i < MOTOR_COUNT; ++i)
            {
                hold_command_.motors[i].target_position = snapshot.data.motors[i].position;
                hold_command_.motors[i].target_velocity = 0;
                hold_command_.motors[i].output = 0;
            }
            fault_hold_target_snapshot_.Write(hold_command_);
            fault_hold_bench_test_ = true;
            ChangeState(MotorInterfaceState::FAULT_HOLD);
            ESP_LOGW(TAG, "Supervised simulated fault: current-pose FAULT_HOLD");
            return;
        }
        if (request == ServoRequest::HOLD_TEST)
        {
            SharedSnapshotFrame<MotorFeedbackFrame> snapshot{};
            const int64_t now_us = esp_timer_get_time();
            bool valid = GetFeedbackFrame(snapshot) &&
                ValidateFeedback(snapshot.data, now_us);
            if (valid)
            {
                const int32_t sum = snapshot.data.motors[1].position +
                    snapshot.data.motors[2].position;
                if (std::abs(sum - 4094) > PARK_SHOULDER_COUPLING_TOLERANCE)
                    valid = false;
                for (size_t i = 0; i < MOTOR_COUNT && valid; ++i)
                {
                    const MotorFeedback& fb = snapshot.data.motors[i];
                    if (fb.position < 0 || fb.position > 4095 ||
                        std::abs(static_cast<int32_t>(fb.velocity)) > 50)
                        valid = false;
                }
            }
            if (!valid)
            {
                hold_test_status_.store(HoldTestStatus::REJECTED);
                return;
            }
            for (size_t i = 0; i < MOTOR_COUNT; ++i)
            {
                hold_command_.motors[i].target_position =
                    snapshot.data.motors[i].position;
                hold_command_.motors[i].target_velocity = 0;
                hold_command_.motors[i].output = 0;
            }
            hold_test_target_snapshot_.Write(hold_command_);
            hold_test_stable_cycles_ = 0;
            hold_test_status_.store(HoldTestStatus::CAPTURED);
            ESP_LOGI(TAG, "In-place HOLD test captured current pose");
            return;
        }
        if (request == ServoRequest::HOME)
        {
#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_POSITION && MOTOR_HOME_EXECUTION_ENABLE && MOTOR_TORQUE_WRITE_ENABLE && MOTOR_POSITION_OUTPUT_ENABLE
            if (home_scope_requested_.load() == HomeScope::BASE_ONLY ||
                MOTOR_HOME_ALLOW_FULL_EXECUTION)
                ChangeState(MotorInterfaceState::HOMING);
            else
                ESP_LOGE(TAG, "Full Home blocked: shoulder-release gate disabled");
#else
            ESP_LOGW(TAG, "Home execution is disabled at build time");
#endif
            return;
        }
        if (request == ServoRequest::PARK)
        {
#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_POSITION
#if MOTOR_VENDOR_SHOULDER_PARK_ENABLE
            ChangeState(MotorInterfaceState::PARKING);
#else
            ESP_LOGW(TAG, "Park rejected: manufacturer shoulder-release path not validated");
#endif
#else
            ChangeState(MotorInterfaceState::DISARMING);
#endif
            return;
        }
    }
    UpdateCommunicationError();
    ProcessCommunicationFSM();
}

void MotorInterface::StateParking()
{
#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_POSITION && MOTOR_TORQUE_WRITE_ENABLE && MOTOR_POSITION_OUTPUT_ENABLE && MOTOR_VENDOR_SHOULDER_PARK_ENABLE
    if (servo_request_.exchange(ServoRequest::NONE) == ServoRequest::OFF)
        park_stop_pending_ = true;

    if (park_stop_pending_)
    {
        // Do not interrupt a parameter transaction or hand UART to another
        // state while a reply may be in flight.
        output_inhibit_ = true;
        if (park_busy_ || park_goal_busy_)
        {
            const auto result = parameter_fsm_.Poll();
            if (result == ST3215ParameterFSM::Result::FAILED)
            {
                EnterError();
                return;
            }
            if (result != ST3215ParameterFSM::Result::SUCCESS)
                return;
            park_busy_ = false;
            park_goal_busy_ = false;
        }
        if (comm_state_ != MotorCommState::IDLE)
        {
            UpdateCommunicationError();
            ProcessCommunicationFSM();
            return;
        }
        ESP_LOGW(TAG, "Park interrupted: request torque OFF");
        ChangeState(MotorInterfaceState::DISARMING);
        return;
    }
    if (esp_timer_get_time() - park_start_us_ > PARK_TIMEOUT_US)
    {
        ESP_LOGE(TAG, "PARK timeout; servo torque may remain enabled");
        EnterError();
        return;
    }
    if (park_phase_ == ParkPhase::RELEASE_SHOULDER)
    {
        // Verify all servos are fresh and the mechanically coupled shoulder
        // agrees before releasing torque from driven motor ID13.
        if (!park_command_initialized_)
        {
            SharedSnapshotFrame<MotorFeedbackFrame> feedback{};
            if (!feedback_snapshot_.Read(feedback) ||
                !ValidateFeedback(feedback.data, esp_timer_get_time()))
            {
                ESP_LOGE(TAG, "Park aborted: feedback invalid");
                EnterError();
                return;
            }
            const int32_t shoulder_sum =
                static_cast<int32_t>(feedback.data.motors[1].position) +
                static_cast<int32_t>(feedback.data.motors[2].position);
            if (std::abs(shoulder_sum - 4094) >
                PARK_SHOULDER_COUPLING_TOLERANCE)
            {
                ESP_LOGE(TAG, "Park aborted: shoulder coupling mismatch");
                EnterError();
                return;
            }
            for (size_t i = 0; i < MOTOR_COUNT; ++i)
                park_command_.motors[i].target_position =
                    feedback.data.motors[i].position;
            park_command_initialized_ = true;
        }
        if (PollParkShoulder(false))
        {
            park_phase_ = ParkPhase::MOVE;
            ChangeCommState(MotorCommState::IDLE);
            last_comm_cycle_us_ = 0;
        }
        return;
    }
    if (park_phase_ == ParkPhase::MOVE)
    {
        if (park_stable_count_ >= 10 && comm_state_ == MotorCommState::IDLE)
        {
            park_stable_count_ = 0;
            if (park_stage_ < 4)
            {
                ++park_stage_;
                return;
            }
            park_phase_ = ParkPhase::SET_SHOULDER_GOAL;
            park_goal_busy_ = false;
            return;
        }
        UpdateCommunicationError();
        ProcessCommunicationFSM();
        return;
    }
    if (park_phase_ == ParkPhase::SET_SHOULDER_GOAL)
    {
        // Re-load ID13 goal from its actual released position BEFORE
        // re-enabling torque, or an old goal can cause a sudden jump.
        if (!park_goal_busy_)
        {
            SharedSnapshotFrame<MotorFeedbackFrame> feedback{};
            if (!feedback_snapshot_.Read(feedback) ||
                !ValidateFeedback(feedback.data, esp_timer_get_time()))
            {
                ESP_LOGE(TAG, "Park ID13 restore aborted: stale feedback");
                EnterError();
                return;
            }
            const auto pos = feedback.data.motors[2].position;
            if (pos < 0 || pos > 4095)
            {
                EnterError();
                return;
            }
            park_shoulder_hold_count_ = static_cast<uint16_t>(pos);
            if (parameter_fsm_.SetGoalPosition(13, park_shoulder_hold_count_) != ESP_OK)
            {
                EnterError();
                return;
            }
            park_goal_busy_ = true;
        }
        const auto result = parameter_fsm_.Poll();
        if (result == ST3215ParameterFSM::Result::FAILED)
        {
            EnterError();
            return;
        }
        if (result == ST3215ParameterFSM::Result::SUCCESS)
        {
            park_goal_busy_ = false;
            park_phase_ = ParkPhase::VERIFY_SHOULDER_GOAL;
        }
        return;
    }
    if (park_phase_ == ParkPhase::VERIFY_SHOULDER_GOAL)
    {
        if (!park_goal_busy_)
        {
            if (parameter_fsm_.VerifyGoalPosition(13, park_shoulder_hold_count_) != ESP_OK)
            {
                EnterError();
                return;
            }
            park_goal_busy_ = true;
        }
        const auto result = parameter_fsm_.Poll();
        if (result == ST3215ParameterFSM::Result::FAILED)
        {
            ESP_LOGE(TAG, "Park ID13 restore aborted: goal position mismatch");
            EnterError();
            return;
        }
        if (result == ST3215ParameterFSM::Result::SUCCESS)
        {
            park_goal_busy_ = false;
            park_phase_ = ParkPhase::VERIFY_SHOULDER_POSITION;
        }
        return;
    }
    if (park_phase_ == ParkPhase::VERIFY_SHOULDER_POSITION)
    {
        if (!park_goal_busy_)
        {
            if (parameter_fsm_.VerifyPresentPosition(13, park_shoulder_hold_count_) != ESP_OK)
            {
                EnterError();
                return;
            }
            park_goal_busy_ = true;
        }
        const auto result = parameter_fsm_.Poll();
        if (result == ST3215ParameterFSM::Result::FAILED)
        {
            ESP_LOGE(TAG, "Park ID13 restore aborted: present pose drift");
            EnterError();
            return;
        }
        if (result == ST3215ParameterFSM::Result::SUCCESS)
        {
            park_goal_busy_ = false;
            park_phase_ = ParkPhase::RESTORE_SHOULDER;
        }
        return;
    }
    if (PollParkShoulder(true))
    {
        ChangeState(MotorInterfaceState::DISARMING);
    }
#else
    EnterError();
#endif
}

bool MotorInterface::IsHomeStageServo(size_t index) const
{
    if (home_phase_ == HomePhase::MOVE_BASE) return index == 0;
    if (home_phase_ == HomePhase::MOVE_SHOULDER) return index == 1;
    if (home_phase_ == HomePhase::MOVE_REST)
    {
        // Elbow, wrist, roll, gripper in conservative one-at-a-time order.
        switch (home_stage_)
        {
            case 0: return index == 3;
            case 1: return index == 4;
            case 2: return index == 5;
            case 3: return index == 6;
            default: return false;
        }
    }
    return false;
}

bool MotorInterface::TryEnterHomeFaultHold(const MotorFeedbackFrame& feedback,
                                          bool operator_abort)
{
#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_POSITION && MOTOR_HOME_SAFE_FAULT_HOLD_ENABLE && MOTOR_POSITION_OUTPUT_ENABLE
    // No write while parameter FSM or a servo communication cycle owns UART1.
    // ID13 must have historical confirmation of torque ON; never attempt
    // auto-release or auto-enable during an uncertain shoulder phase.
    if (state_ != MotorInterfaceState::HOMING ||
        (comm_state_ != MotorCommState::IDLE &&
         comm_state_ != MotorCommState::PROCESS) ||
        home_busy_ || park_busy_ ||
        home_id13_verified_.load() != HomeId13Verified::ON)
        return false;
    const int64_t now_us = esp_timer_get_time();
    if (!ValidateFeedback(feedback, now_us))
        return false;
    for (size_t i = 0; i < MOTOR_COUNT; ++i)
    {
        if (feedback.motors[i].position < 0 ||
            feedback.motors[i].position > 4095)
            return false;
    }
    if (std::abs(static_cast<int32_t>(feedback.motors[1].position) +
                 static_cast<int32_t>(feedback.motors[2].position) - 4094)
        > PARK_SHOULDER_COUPLING_TOLERANCE)
        return false;

    for (size_t i = 0; i < MOTOR_COUNT; ++i)
    {
        hold_command_.motors[i].target_position = feedback.motors[i].position;
        hold_command_.motors[i].target_velocity = 0;
        hold_command_.motors[i].output = 0;
    }
    fault_hold_target_snapshot_.Write(hold_command_);
    fault_hold_status_.store(FaultHoldStatus::REQUESTED);
    fault_hold_bench_test_ = false; // No T226 resume after real HOME abort/fault.
    const uint32_t id13 = static_cast<uint32_t>(home_id13_verified_.load());
    const uint32_t phase = static_cast<uint32_t>(home_phase_) + 1u;
    home_fault_diagnostic_.store(
        1u | 2u | (id13 == static_cast<uint32_t>(HomeId13Verified::OFF) ? 4u : 0u) |
        (phase << 8) | (id13 << 16));
    // Operator-requested abort is INTERRUPTED; detected HOME faults FAILED.
    SetOperationSnapshot(MotorOperation::HOME,
        operator_abort ? MotorOperationStatus::INTERRUPTED :
                         MotorOperationStatus::FAILED);
    ChangeState(MotorInterfaceState::FAULT_HOLD);
    ESP_LOGW(TAG, "HOME transitioned to latched FAULT_HOLD; ID13 verified ON");
    return true;
#else
    (void)feedback;
    (void)operator_abort;
    return false;
#endif
}

bool MotorInterface::TryEnterHomeFaultHoldFromSnapshot(bool operator_abort)
{
    // Snapshot is from the last fully completed cycle, not the current
    // PROCESS-stage in-progress frame. The explicit abort path waits for
    // the ongoing UART/feedback cycle to finish before using it.
    if (comm_state_ != MotorCommState::IDLE)
        return false;
    SharedSnapshotFrame<MotorFeedbackFrame> snapshot{};
    return GetFeedbackFrame(snapshot) &&
        TryEnterHomeFaultHold(snapshot.data, operator_abort);
}

void MotorInterface::StateHoming()
{
#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_POSITION && (MOTOR_HOME_EXECUTION_ENABLE || MOTOR_HOME_COMPILE_CHECK) && MOTOR_TORQUE_WRITE_ENABLE && MOTOR_POSITION_OUTPUT_ENABLE
    // A single ServoTask owns HomePhase and home_stage_. Publish a packed
    // consistent read-only diagnostic snapshot before working this iteration.
    PublishHomeProgress();
    const ServoRequest home_request = servo_request_.exchange(ServoRequest::NONE);
    if (home_request == ServoRequest::OFF)
        home_stop_pending_ = true;
    if (home_request == ServoRequest::HOME_HOLD_ABORT)
        home_abort_hold_pending_ = true;

    if (home_abort_hold_pending_)
    {
        output_inhibit_ = true;
        // Never interrupt UART parameter transactions. If ID13 is OFF or
        // status/feedback is uncertain, fail closed without asserting hold.
        if (TryEnterHomeFaultHoldFromSnapshot(true))
            return;
        if (home_busy_ || park_busy_ ||
            home_id13_verified_.load() != HomeId13Verified::ON)
        {
            ESP_LOGE(TAG, "HOME hold abort unavailable: ID13 or UART transaction uncertain");
            EnterError();
            return;
        }
        // Communication cycle in progress: wait for it to finish normally.
        // Do not generate a competing UART request.
        if (comm_state_ != MotorCommState::IDLE)
        {
            UpdateCommunicationError();
            ProcessCommunicationFSM();
            return;
        }
        ESP_LOGE(TAG, "HOME hold abort failed: fresh paired feedback unavailable");
        EnterError();
        return;
    }

    if (home_stop_pending_)
    {
        output_inhibit_ = true;
        // Finish any parameter transaction before changing UART ownership.
        if (home_busy_ || park_busy_)
        {
            const auto result = parameter_fsm_.Poll();
            if (result == ST3215ParameterFSM::Result::BUSY) return;
            if (result == ST3215ParameterFSM::Result::FAILED)
            { EnterError(); return; }
            home_busy_ = false;
            park_busy_ = false;
        }
        if (comm_state_ != MotorCommState::IDLE)
        {
            UpdateCommunicationError();
            ProcessCommunicationFSM();
            return;
        }
        ESP_LOGW(TAG, "HOME interrupted by explicit no-park Servo OFF request; ID13 last-verified=%u (not live)",
            static_cast<unsigned>(home_id13_verified_.load()));
        ChangeState(MotorInterfaceState::DISARMING);
        return;
    }

    const int64_t home_now_us = esp_timer_get_time();
    // Watchdog follows the actual stage (including individual MOVE_REST joints).
    // The first iteration after a transition starts a fresh stage budget.
    if (home_watchdog_phase_ != home_phase_ ||
        home_watchdog_stage_ != home_stage_)
    {
        home_watchdog_phase_ = home_phase_;
        home_watchdog_stage_ = home_stage_;
        home_watchdog_start_us_ = home_now_us;
    }

    const bool home_move_stage =
        home_phase_ == HomePhase::MOVE_BASE ||
        home_phase_ == HomePhase::MOVE_SHOULDER ||
        home_phase_ == HomePhase::MOVE_REST;
    const int64_t home_stage_limit_us =
        home_move_stage ? HOME_MOVE_STAGE_TIMEOUT_US :
        (home_phase_ == HomePhase::SETTLE_SHOULDER ?
            HOME_SETTLE_TIMEOUT_US : HOME_PARAMETER_TIMEOUT_US);
    if (home_now_us - home_start_us_ > HOME_MAX_TOTAL_US ||
        home_now_us - home_watchdog_start_us_ > home_stage_limit_us)
    {
        ESP_LOGE(TAG, "HOME timeout phase=%u stage=%u ID13 last-verified=%u (not live); servo torque unknown",
            static_cast<unsigned>(home_phase_),
            static_cast<unsigned>(home_stage_),
            static_cast<unsigned>(home_id13_verified_.load()));
        // Timeout can be a mechanical stall even if UART is healthy.
        // Try the verified torque-ON Current Position Hold only when UART
        // is idle and feedback is fresh; otherwise preserve ERROR semantics.
        if (!TryEnterHomeFaultHoldFromSnapshot())
            EnterError();
        return;
    }

    if (home_phase_ == HomePhase::PREPARE)
    {
        SharedSnapshotFrame<MotorFeedbackFrame> snapshot{};
        if (!feedback_snapshot_.Read(snapshot) ||
            !ValidateFeedback(snapshot.data, esp_timer_get_time()))
        { EnterError(); return; }
        int32_t shoulder_sum = 0;
        for (size_t i = 0; i < MOTOR_COUNT; ++i)
        {
            const int32_t pos = snapshot.data.motors[i].position;
            const int32_t vel = snapshot.data.motors[i].velocity;
            if (!home_safety::CheckRawJointLimit(i, pos) || std::abs(vel) > 50 ||
                (HOME_GOALS[i] >= 0 &&
                 !home_safety::CheckRawJointLimit(i, HOME_GOALS[i])))
            { EnterError(); return; }
            home_command_.motors[i].target_position = static_cast<int16_t>(pos);
            if (i == 1 || i == 2) shoulder_sum += pos;
        }
        if (std::abs(shoulder_sum - 4094) > PARK_SHOULDER_COUPLING_TOLERANCE)
        { EnterError(); return; }
        home_command_initialized_ = true;
        home_target_snapshot_.Write(home_command_);
        home_phase_ = HomePhase::MOVE_BASE;
        return;
    }

    if (home_phase_ == HomePhase::MOVE_BASE ||
        home_phase_ == HomePhase::MOVE_SHOULDER ||
        home_phase_ == HomePhase::MOVE_REST)
    {
        if (home_stable_count_ >= 10 && comm_state_ == MotorCommState::IDLE)
        {
            home_stable_count_ = 0;
            if (home_phase_ == HomePhase::MOVE_BASE)
            {
                if (home_scope_active_ == HomeScope::BASE_ONLY)
                {
                    SharedSnapshotFrame<MotorFeedbackFrame> snapshot{};
                    if (!GetFeedbackFrame(snapshot) ||
                        !ValidateFeedback(snapshot.data, esp_timer_get_time()))
                    { EnterError(); return; }
                    const int32_t shoulder_sum =
                        static_cast<int32_t>(snapshot.data.motors[1].position) +
                        static_cast<int32_t>(snapshot.data.motors[2].position);
                    if (std::abs(shoulder_sum - 4094) >
                        PARK_SHOULDER_COUPLING_TOLERANCE)
                    {
                        if (!TryEnterHomeFaultHoldFromSnapshot())
                            EnterError();
                        return;
                    }
                    for (size_t i = 0; i < MOTOR_COUNT; ++i)
                    {
                        const auto& fb = snapshot.data.motors[i];
                        if (!home_safety::CheckRawJointLimit(i, fb.position) ||
                            std::abs(static_cast<int32_t>(fb.velocity)) > 50)
                        {
                            if (!TryEnterHomeFaultHoldFromSnapshot())
                                EnterError();
                            return;
                        }
                        hold_command_.motors[i].target_position = fb.position;
                    }
                    home_progress_snapshot_.store(11u);
                    ESP_LOGI(TAG, "BASE_ONLY HOME completed without ID13 torque release");
                    ChangeState(MotorInterfaceState::ARMED);
                    return;
                }
                home_phase_ = HomePhase::RELEASE_SHOULDER;
            }
            else if (home_phase_ == HomePhase::MOVE_SHOULDER)
            {
                home_phase_ = HomePhase::SETTLE_SHOULDER;
                home_shoulder_settle_us_ = esp_timer_get_time();
            }
            else if (home_stage_ < 3)
                ++home_stage_;
            else
            {
                // Do not send the pre-Home hold positions when re-entering
                // ARMED. Capture the actual final pose as the new hold.
                SharedSnapshotFrame<MotorFeedbackFrame> snapshot{};
                if (!feedback_snapshot_.Read(snapshot) ||
                    !ValidateFeedback(snapshot.data, esp_timer_get_time()))
                { EnterError(); return; }
                const int32_t sum =
                    static_cast<int32_t>(snapshot.data.motors[1].position) +
                    static_cast<int32_t>(snapshot.data.motors[2].position);
                if (std::abs(sum - 4094) > PARK_SHOULDER_COUPLING_TOLERANCE)
                { EnterError(); return; }
                // Full HOME cannot declare success based on the passive
                // shoulder coupling sum alone. Confirm every servo is at
                // the operator-selected physical HOME reference.
                for (size_t i = 0; i < MOTOR_COUNT; ++i)
                {
                    const int32_t pos = snapshot.data.motors[i].position;
                    const int32_t vel = snapshot.data.motors[i].velocity;
                    const int32_t allowed_error =
                        i == 2 ? HOME_ID13_REFERENCE_TOLERANCE : 10;
                    if (!home_safety::CheckRawJointLimit(i, pos) ||
                        std::abs(pos - HOME_REFERENCE_RAW[i]) > allowed_error ||
                        std::abs(vel) > 50)
                    {
                        ESP_LOGE(TAG,
                            "HOME final reference mismatch id=%u pos=%ld ref=%ld vel=%ld",
                            static_cast<unsigned>(motors_[i].GetID()),
                            static_cast<long>(pos),
                            static_cast<long>(HOME_REFERENCE_RAW[i]),
                            static_cast<long>(vel));
                        EnterError();
                        return;
                    }
                    hold_command_.motors[i].target_position =
                        static_cast<int16_t>(pos);
                }
                ESP_LOGI(TAG, "HOME completed; new current pose captured for hold");
                home_progress_snapshot_.store(11u); // COMPLETED, stage 0
                ChangeState(MotorInterfaceState::ARMED);
            }
            return;
        }
        UpdateCommunicationError();
        ProcessCommunicationFSM();
        return;
    }

    if (home_phase_ == HomePhase::SETTLE_SHOULDER)
    {
        if (esp_timer_get_time() - home_shoulder_settle_us_ >= 1200000 &&
            comm_state_ == MotorCommState::IDLE)
        {
            home_phase_ = HomePhase::SET_SHOULDER_GOAL;
            return;
        }
        UpdateCommunicationError();
        ProcessCommunicationFSM();
        return;
    }

    if (home_phase_ == HomePhase::RELEASE_SHOULDER)
    {
        if (PollParkShoulder(false))
        {
            home_id13_verified_.store(HomeId13Verified::OFF);
            home_phase_ = HomePhase::MOVE_SHOULDER;
            ChangeCommState(MotorCommState::IDLE);
            last_comm_cycle_us_ = 0;
        }
        return;
    }

    // ID13 is passive while ID12 moves. Synchronize its GOAL to PRESENT
    // before re-enabling Servo ON, without any EEPROM zero-offset write.
    if (home_phase_ == HomePhase::SET_SHOULDER_GOAL)
    {
        if (!home_busy_)
        {
            SharedSnapshotFrame<MotorFeedbackFrame> snapshot{};
            if (!feedback_snapshot_.Read(snapshot) ||
                !ValidateFeedback(snapshot.data, esp_timer_get_time()))
            { EnterError(); return; }
            const int32_t id12 = snapshot.data.motors[1].position;
            const int32_t id13 = snapshot.data.motors[2].position;
            if (std::abs(id12 + id13 - 4094) > PARK_SHOULDER_COUPLING_TOLERANCE ||
                std::abs(id12 - HOME_REFERENCE_RAW[1]) > 10 ||
                std::abs(id13 - HOME_REFERENCE_RAW[2]) > HOME_ID13_REFERENCE_TOLERANCE ||
                std::abs(snapshot.data.motors[2].velocity) > 50 ||
                id13 < 0 || id13 > 4095)
            { EnterError(); return; }
            home_shoulder_goal_ = static_cast<uint16_t>(id13);
            home_command_.motors[2].target_position = static_cast<int16_t>(id13);
            if (parameter_fsm_.SetGoalPosition(13, home_shoulder_goal_) != ESP_OK)
            { EnterError(); return; }
            home_busy_ = true;
        }
        const auto result = parameter_fsm_.Poll();
        if (result == ST3215ParameterFSM::Result::FAILED) { EnterError(); return; }
        if (result == ST3215ParameterFSM::Result::SUCCESS)
        {
            home_busy_ = false;
            home_phase_ = HomePhase::VERIFY_SHOULDER_GOAL;
        }
        return;
    }

    if (home_phase_ == HomePhase::VERIFY_SHOULDER_GOAL ||
        home_phase_ == HomePhase::VERIFY_SHOULDER_POSITION)
    {
        if (!home_busy_)
        {
            const esp_err_t err = home_phase_ == HomePhase::VERIFY_SHOULDER_GOAL ?
                parameter_fsm_.VerifyGoalPosition(13, home_shoulder_goal_) :
                parameter_fsm_.VerifyPresentPosition(13, home_shoulder_goal_);
            if (err != ESP_OK) { EnterError(); return; }
            home_busy_ = true;
        }
        const auto result = parameter_fsm_.Poll();
        if (result == ST3215ParameterFSM::Result::FAILED) { EnterError(); return; }
        if (result == ST3215ParameterFSM::Result::SUCCESS)
        {
            home_busy_ = false;
            home_phase_ = home_phase_ == HomePhase::VERIFY_SHOULDER_GOAL ?
                HomePhase::VERIFY_SHOULDER_POSITION : HomePhase::RESTORE_SHOULDER;
        }
        return;
    }

    if (home_phase_ == HomePhase::RESTORE_SHOULDER)
    {
        if (PollParkShoulder(true))
        {
            home_id13_verified_.store(HomeId13Verified::ON);
            home_stage_ = 0;
            home_stable_count_ = 0;
            home_phase_ = HomePhase::MOVE_REST;
            ChangeCommState(MotorCommState::IDLE);
            last_comm_cycle_us_ = 0;
        }
        return;
    }

    EnterError();
#else
    EnterError();
#endif
}

void MotorInterface::StateDisarming()
{
#if MOTOR_TORQUE_WRITE_ENABLE
    if (PollTorque(false, false))
    {
        torque_off_verified_ = true;
        torque_off_public_.store(true);
        if (boot_recheck_mode_after_off_)
        {
            boot_recheck_mode_after_off_ = false;
            ESP_LOGI(TAG, "BOOT_RECOVERY: torque OFF verified; rechecking operating modes");
            ChangeState(MotorInterfaceState::INIT);
            return;
        }
        ChangeState(MotorInterfaceState::DISARMED);
    }
#else
    EnterError();
#endif
}

void MotorInterface::StateFaultHold()
{
#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_POSITION && MOTOR_POSITION_OUTPUT_ENABLE
    const ServoRequest request = servo_request_.exchange(ServoRequest::NONE);
    if (request == ServoRequest::OFF)
    {
        // Explicit release is gravity-sensitive and never automatic.
        ChangeState(MotorInterfaceState::DISARMING);
        return;
    }
    if (request == ServoRequest::FAULT_HOLD_RESUME &&
        fault_hold_bench_test_ &&
        fault_hold_status_.load() == FaultHoldStatus::STABLE)
    {
        fault_hold_bench_test_ = false;
        ChangeState(MotorInterfaceState::ARMED);
        return;
    }
    UpdateCommunicationError();
    ProcessCommunicationFSM();
#else
    EnterError();
#endif
}

void MotorInterface::StateError()
{
    // TX inhibit does not guarantee actual motors are torque OFF.
    output_inhibit_ = true;
}

void MotorInterface::ProcessCommunicationFSM()
{
    while (true)
    {
        if (state_ != MotorInterfaceState::DISARMED &&
            state_ != MotorInterfaceState::BOOT_RECOVERY &&
            state_ != MotorInterfaceState::ARMED &&
            state_ != MotorInterfaceState::PARKING &&
            state_ != MotorInterfaceState::HOMING &&
            state_ != MotorInterfaceState::FAULT_HOLD)
            return;

        if (comm_state_ != MotorCommState::IDLE &&
            comm_state_ != MotorCommState::RX_RECOVERY &&
            comm_cycle_start_us_ > 0 &&
            esp_timer_get_time() - comm_cycle_start_us_
                >= COMM_TIMEOUT_US)
        {
#if MOTOR_PERF_LOG_ENABLE
            ++perf_.cycle_timeout_count;
#endif
            ChangeCommState(MotorCommState::RX_RECOVERY);
            return;
        }

        const MotorCommState previous = comm_state_;

        switch (comm_state_)
        {
            case MotorCommState::IDLE:
                CommIdle();
                break;

            case MotorCommState::SEND_REQUEST:
                CommSendRequest();
                break;

            case MotorCommState::WAIT_RESPONSE:
                CommWaitResponse();
                break;

            case MotorCommState::NEXT_MOTOR:
                CommNextMotor();
                break;

            case MotorCommState::PROCESS:
                CommProcess();
                break;

            case MotorCommState::SYNC_WRITE:
                CommSyncWrite();
                break;

            case MotorCommState::COMPLETE:
                CommComplete();
                return;

            case MotorCommState::RX_RECOVERY:
                CommRxRecovery();
                break;

            default:
                EnterError();
                return;
        }

        if (state_ == MotorInterfaceState::ERROR)
            return;

        // No progress or communication failure -> IDLE.
        if (comm_state_ == previous ||
            comm_state_ == MotorCommState::IDLE)
        {
            return;
        }
    }
}

void MotorInterface::CommIdle()
{
    const int64_t now_us = esp_timer_get_time();

    if (now_us - last_comm_cycle_us_ < COMM_PERIOD_US)
    {
        return;
    }

    last_comm_cycle_us_ = now_us;
    current_motor_index_ = 0;
    cycle_write_sent_ = false;

    comm_cycle_start_us_ = esp_timer_get_time();
    // A new cycle must not inherit valid flags
    // from the previous cycle.
    cycle_feedback_ = {};

    ChangeCommState(MotorCommState::SEND_REQUEST);
}


void MotorInterface::CommSendRequest()
{
    if (current_motor_index_ >= MOTOR_COUNT)
    {
        ChangeCommState(MotorCommState::COMPLETE);
        return;
    }

    ST3215Request request{};

    request.command = ST3215Command::READ;
    request.id = motors_[current_motor_index_].GetID();
    request.address = FEEDBACK_ADDRESS;
    request.data_size = FEEDBACK_SIZE;

    uint8_t tx_buffer[ST3215::MAX_PACKET_SIZE]{};
    size_t tx_len = 0;

    esp_err_t ret = ST3215::Serialize(
        request,
        tx_buffer,
        sizeof(tx_buffer),
        tx_len
    );

    if (ret != ESP_OK)
    {
        HandleCommunicationFailure();
        return;
    }

    ret = uart_flush_input(uart_num_);

    if (ret != ESP_OK)
    {
        HandleCommunicationFailure();
        return;
    }

    ret = SendRequest(
        tx_buffer,
        tx_len,
        pdMS_TO_TICKS(2)
    );

    if (ret != ESP_OK)
    {
        // TX may still be active after a timeout.
        // Do not start another request blindly.
        EnterError();
        return;
    }

    BeginReceive(request.id, request.data_size);

    ChangeCommState(MotorCommState::WAIT_RESPONSE);
}


void MotorInterface::CommWaitResponse()
{
    ST3215Response response{};
    int64_t completed_us = 0;

    const ServoReceiveStatus result =
        ReceiveResponse(response, completed_us);

    if (result == ServoReceiveStatus::WAITING)
        return;

    if (result != ServoReceiveStatus::COMPLETE)
    {
        ChangeCommState(MotorCommState::RX_RECOVERY);
        return;
    }

    MotorFeedback feedback{};

    const esp_err_t ret = ST3215::DecodeFeedback(
        response,
        completed_us,
        feedback
    );

    if (ret != ESP_OK)
    {
        ChangeCommState(MotorCommState::RX_RECOVERY);
        return;
    }

    motors_[current_motor_index_].SetFeedback(feedback);

    cycle_feedback_.motors[current_motor_index_] =
        feedback;

    ChangeCommState(MotorCommState::NEXT_MOTOR);
}

void MotorInterface::CommNextMotor()
{
    ++current_motor_index_;

    if (current_motor_index_ < MOTOR_COUNT)
    {
        ChangeCommState(MotorCommState::SEND_REQUEST);
    }
    else
    {
        ChangeCommState(MotorCommState::PROCESS);
    }
}

void MotorInterface::CommProcess()
{
    const int64_t now_us = esp_timer_get_time();

    // 1. Feedback validation
    if (!ValidateFeedback(cycle_feedback_, now_us))
    {
        HandleCommunicationFailure();
        return;
    }

    // Operator HOME-hold abort: do not send another moving HOME target.
    // CommComplete will publish the complete feedback and StateHoming will
    // capture a hold goal at the next idle ServoTask iteration.
    if (state_ == MotorInterfaceState::HOMING && home_abort_hold_pending_)
    {
        output_inhibit_ = true;
        command_valid_ = false;
        ChangeCommState(MotorCommState::COMPLETE);
        return;
    }

    // DISARMED publishes feedback only; never runs PID or transmits commands.
    if (state_ != MotorInterfaceState::ARMED &&
        state_ != MotorInterfaceState::PARKING &&
        state_ != MotorInterfaceState::HOMING &&
        state_ != MotorInterfaceState::FAULT_HOLD)
    {
        ChangeControlValidity(false);
        command_valid_ = false;
        cycle_command_ = {};
        output_inhibit_ = true;
        ChangeCommState(MotorCommState::COMPLETE);
        return;
    }

#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_POSITION
    // Initial hold, park, and home targets are owned by the Servo FSM, not JSON.
    if (state_ == MotorInterfaceState::PARKING || state_ == MotorInterfaceState::HOMING)
    {
        // Integrate goal from previous commanded position rather than from
        // feedback. Otherwise a 3-count command can stall indefinitely.
        if (state_ == MotorInterfaceState::HOMING)
        {
            if (!home_command_initialized_) { EnterError(); return; }
            // ID13 is passive while ID12 moves. Monitor the physical pairing,
            // not only commanded tracking, to catch a drifting shoulder.
            if (home_phase_ == HomePhase::MOVE_SHOULDER ||
                home_phase_ == HomePhase::SETTLE_SHOULDER)
            {
                const int32_t shoulder_sum =
                    static_cast<int32_t>(cycle_feedback_.motors[1].position) +
                    static_cast<int32_t>(cycle_feedback_.motors[2].position);
                if (std::abs(shoulder_sum - 4094) >
                    PARK_SHOULDER_COUPLING_TOLERANCE)
                {
                    ESP_LOGE(TAG, "HOME shoulder coupling diverged id12=%d id13=%d",
                        static_cast<int>(cycle_feedback_.motors[1].position),
                        static_cast<int>(cycle_feedback_.motors[2].position));
                    if (!TryEnterHomeFaultHold(cycle_feedback_))
                        EnterError();
                    return;
                }
            }
            for (size_t i = 0; i < MOTOR_COUNT; ++i)
            {
                if (HOME_GOALS[i] < 0) continue;
                const int32_t error = static_cast<int32_t>(home_command_.motors[i].target_position) -
                    static_cast<int32_t>(cycle_feedback_.motors[i].position);
                if (!home_safety::CheckRawJointLimit(i, cycle_feedback_.motors[i].position) ||
                    !home_safety::CheckRawJointLimit(i, home_command_.motors[i].target_position) ||
                    std::abs(error) > PARK_MAX_TRACKING_ERROR)
                {
                    if (!TryEnterHomeFaultHold(cycle_feedback_))
                        EnterError();
                    return;
                }
            }
            const bool moving = home_phase_ == HomePhase::MOVE_BASE ||
                home_phase_ == HomePhase::MOVE_SHOULDER ||
                home_phase_ == HomePhase::MOVE_REST;
            if (moving && now_us - home_last_goal_us_ >= PARK_GOAL_PERIOD_US)
            {
                home_last_goal_us_ = now_us;
                for (size_t i = 0; i < MOTOR_COUNT; ++i)
                {
                    if (HOME_GOALS[i] < 0 || !IsHomeStageServo(i)) continue;
                    const int32_t last = home_command_.motors[i].target_position;
                    const int32_t step = std::clamp(
                        static_cast<int32_t>(HOME_GOALS[i]) - last,
                        -PARK_STEP_COUNTS, PARK_STEP_COUNTS);
                    const int32_t next = last + step;
                    if (!home_safety::CheckRawJointLimit(i, next))
                    {
                        if (!TryEnterHomeFaultHold(cycle_feedback_))
                            EnterError();
                        return;
                    }
                    home_command_.motors[i].target_position = static_cast<int16_t>(next);
                }
            }
            cycle_command_ = home_command_;
            home_target_snapshot_.Write(home_command_);
            command_valid_ = true;
            ChangeControlValidity(true);
            output_inhibit_ = false;
            ChangeCommState(MotorCommState::SYNC_WRITE);
            return;
        }
        if (!park_command_initialized_)
        {
            EnterError();
            return;
        }
        // ID13 torque has deliberately been released during vendor PARK.
        // The normal paired-servo position relationship applies while both
        // shoulder servos are engaged, not during this release window.
        // Keep position/velocity feedback and tracking-error checks below;
        // the actual passive ID13 trajectory requires hardware validation
        // before re-enabling the vendor PARK feature flag.
        const int32_t shoulder_sum =
            static_cast<int32_t>(cycle_feedback_.motors[1].position) +
            static_cast<int32_t>(cycle_feedback_.motors[2].position);
        static int64_t last_shoulder_log_us = 0;
        if (now_us - last_shoulder_log_us >= 250000)
        {
            last_shoulder_log_us = now_us;
            ESP_LOGI(TAG,
                "PARK shoulder released: id12=%d id13=%d sum_error=%ld stage=%u",
                static_cast<int>(cycle_feedback_.motors[1].position),
                static_cast<int>(cycle_feedback_.motors[2].position),
                static_cast<long>(shoulder_sum - 4094),
                static_cast<unsigned>(park_stage_));
        }
        for (size_t i = 0; i < MOTOR_COUNT; ++i)
        {
            if (PARK_GOALS[i] < 0) continue;
            const int32_t tracking_error =
                static_cast<int32_t>(park_command_.motors[i].target_position) -
                static_cast<int32_t>(cycle_feedback_.motors[i].position);
            if (std::abs(tracking_error) > PARK_MAX_TRACKING_ERROR)
            {
                ESP_LOGE(TAG, "Park aborted: servo id=%u tracking error=%ld",
                         static_cast<unsigned>(motors_[i].GetID()),
                         static_cast<long>(tracking_error));
                EnterError();
                return;
            }
        }
        if (now_us - park_last_goal_us_ >= PARK_GOAL_PERIOD_US)
        {
            park_last_goal_us_ = now_us;
            for (size_t i = 0; i < MOTOR_COUNT; ++i)
            {
                if (PARK_GOALS[i] < 0 || !IsParkStageServo(i)) continue; // vendor sequence
                const int32_t last = park_command_.motors[i].target_position;
                const int32_t step = std::clamp(
                    static_cast<int32_t>(PARK_GOALS[i]) - last,
                    -PARK_STEP_COUNTS, PARK_STEP_COUNTS);
                park_command_.motors[i].target_position =
                    static_cast<int16_t>(last + step);
            }
        }
        cycle_command_ = park_command_;
    }
    else
    {
        // The bench fault latch must never chase a drifting joint.
        // Abort new writes on excessive tracking deviation; physical
        // torque remains uncertain in ERROR and must be supervised.
        if (state_ == MotorInterfaceState::FAULT_HOLD)
        {
            for (size_t i = 0; i < MOTOR_COUNT; ++i)
            {
                const int32_t actual = cycle_feedback_.motors[i].position;
                const int32_t goal = hold_command_.motors[i].target_position;
                if (actual < 0 || actual > 4095 ||
                    std::abs(actual - goal) > PARK_MAX_TRACKING_ERROR)
                {
                    ESP_LOGE(TAG, "FAULT_HOLD tracking fault id=%u actual=%ld goal=%ld",
                        static_cast<unsigned>(motors_[i].GetID()),
                        static_cast<long>(actual),
                        static_cast<long>(goal));
                    EnterError();
                    return;
                }
            }
        }
        cycle_command_ = hold_command_;
    }
    command_valid_ = true;
    ChangeControlValidity(true);
    output_inhibit_ = false;
    ChangeCommState(MotorCommState::SYNC_WRITE);
    return;
#endif
    // 2. Command validation
    SharedSnapshotFrame<MotorCommandFrame> cmd_frame{};

    command_valid_ =
        command_snapshot_.Read(cmd_frame) &&
        ValidateCommand(cmd_frame, now_us);

    if (!command_valid_)
    {
        ChangeControlValidity(false);
        cycle_command_ = {};
        output_inhibit_ = true;

        ChangeCommState(MotorCommState::SYNC_WRITE);
        return;
    }

    cycle_command_ = cmd_frame.data;
    ChangeControlValidity(true);

#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_POSITION
    // The servo owns the low-level position loop. Never call the ESP32 PID.
    // Physical transmission remains behind MOTOR_POSITION_OUTPUT_ENABLE and ARM.
    for (size_t i = 0; i < MOTOR_COUNT; ++i)
    {
        const int32_t target = cycle_command_.motors[i].target_position;
        if (target < 0 || target > 4095)
        {
            ChangeControlValidity(false);
            output_inhibit_ = true;
            ChangeCommState(MotorCommState::COMPLETE);
            return;
        }
    }
    output_inhibit_ = false;
    ChangeCommState(MotorCommState::SYNC_WRITE);
    return;
#else
    // 3. PID update timing
    if (last_pid_update_us_ == 0)
    {
        last_pid_update_us_ = now_us;
        output_inhibit_ = true;

        ChangeCommState(MotorCommState::SYNC_WRITE);
        return;
    }

    const float dt =
        static_cast<float>(now_us - last_pid_update_us_)
        * 1.0e-6f;

    if (!std::isfinite(dt) ||
        dt <= 0.0f ||
        dt > 0.03f)
    {
        ChangeControlValidity(false);
        output_inhibit_ = true;

        ChangeCommState(MotorCommState::SYNC_WRITE);
        return;
    }

    // 4. PID calculation: never allow non-finite values into integer PWM.
    if (!ProcessPID(dt))
    {
        ChangeControlValidity(false);
        cycle_command_ = {};
        output_inhibit_ = true;
        ChangeCommState(MotorCommState::SYNC_WRITE);
        return;
    }

    last_pid_update_us_ = now_us;

    output_inhibit_ = false;

    ChangeCommState(MotorCommState::SYNC_WRITE);
#endif  // MOTOR_CONTROL_MODE == MOTOR_CONTROL_POSITION
}

esp_err_t MotorInterface::SendSyncPWM(
    const MotorCommandFrame& command)
{
    uint8_t ids[MOTOR_COUNT]{};
    uint8_t pwm_data[MOTOR_COUNT * PWM_OUTPUT_SIZE]{};

    for (size_t i = 0; i < MOTOR_COUNT; ++i)
    {
        ids[i] = motors_[i].GetID();

        const int16_t limited_output = std::clamp(
            command.motors[i].output,
            PWM_MIN,
            PWM_MAX
        );

        const esp_err_t ret = ST3215::EncodePWM(
            limited_output,
            &pwm_data[i * PWM_OUTPUT_SIZE]);

        if (ret != ESP_OK)
            return ret;
    }

    ST3215Request request{};

    request.command = ST3215Command::SYNC_WRITE;
    request.address = PWM_OUTPUT_ADDRESS;
    request.data_size = PWM_OUTPUT_SIZE;
    request.servo_ids = ids;
    request.servo_count = MOTOR_COUNT;
    request.data = pwm_data;
    request.data_len = sizeof(pwm_data);

    uint8_t tx_buffer[ST3215::MAX_PACKET_SIZE]{};
    size_t tx_len = 0;

    const esp_err_t ret = ST3215::Serialize(
        request,
        tx_buffer,
        sizeof(tx_buffer),
        tx_len);

    if (ret != ESP_OK)
        return ret;

    return SendRequest(
        tx_buffer,
        tx_len,
        pdMS_TO_TICKS(2));
}


esp_err_t MotorInterface::SendSyncPosition(
    const MotorCommandFrame& command)
{
    constexpr uint8_t POSITION_ADDRESS = 42;
    constexpr uint8_t POSITION_SIZE = 2;

    uint8_t ids[MOTOR_COUNT]{};
    uint8_t position_data[MOTOR_COUNT * POSITION_SIZE]{};

    size_t servo_count = 0;
    for (size_t i = 0; i < MOTOR_COUNT; ++i)
    {
        // Factory PARK releases ID13 while ID12 drives the coupled shoulder.
        // Do not overwrite ID13's goal during this torque-off MOVE phase.
        if (((state_ == MotorInterfaceState::PARKING &&
            park_phase_ == ParkPhase::MOVE) ||
            (state_ == MotorInterfaceState::HOMING &&
            (home_phase_ == HomePhase::MOVE_SHOULDER ||
             home_phase_ == HomePhase::SETTLE_SHOULDER))) &&
            motors_[i].GetID() == 13)
            continue;

        const int32_t target = command.motors[i].target_position;
        if (target < 0 || target > 4095)
            return ESP_ERR_INVALID_ARG;

        ids[servo_count] = motors_[i].GetID();
        const uint16_t raw = static_cast<uint16_t>(target);
        position_data[servo_count * POSITION_SIZE] =
            static_cast<uint8_t>(raw & 0xFF);
        position_data[servo_count * POSITION_SIZE + 1] =
            static_cast<uint8_t>((raw >> 8) & 0xFF);
        ++servo_count;
    }

    if (servo_count == 0)
        return ESP_ERR_INVALID_STATE;

    ST3215Request request{};

    request.command = ST3215Command::SYNC_WRITE;
    request.address = POSITION_ADDRESS;
    request.data_size = POSITION_SIZE;
    request.servo_ids = ids;
    request.servo_count = servo_count;
    request.data = position_data;
    request.data_len = servo_count * POSITION_SIZE;

    uint8_t tx_buffer[ST3215::MAX_PACKET_SIZE]{};
    size_t tx_len = 0;

    const esp_err_t err = ST3215::Serialize(
        request,
        tx_buffer,
        sizeof(tx_buffer),
        tx_len
    );

    if (err != ESP_OK)
        return err;

    return SendRequest(
        tx_buffer,
        tx_len,
        pdMS_TO_TICKS(2)
    );
}


void MotorInterface::CommSyncWrite()
{
#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_POSITION
    constexpr bool output_enabled =
        MOTOR_POSITION_OUTPUT_ENABLE;
#else
    constexpr bool output_enabled =
        MOTOR_PWM_OUTPUT_ENABLE;
#endif

    if (!output_enabled ||
        (state_ != MotorInterfaceState::ARMED &&
         state_ != MotorInterfaceState::PARKING &&
         state_ != MotorInterfaceState::HOMING &&
         state_ != MotorInterfaceState::FAULT_HOLD) ||
        (servo_request_.load() == ServoRequest::OFF || park_stop_pending_ ||
         home_stop_pending_) ||
        !command_valid_ ||
        output_inhibit_)
    {
        ChangeCommState(MotorCommState::COMPLETE);
        return;
    }

    if (esp_timer_get_time() - comm_cycle_start_us_
        >= CYCLE_BUDGET_US)
    {
#if MOTOR_PERF_LOG_ENABLE
        ++perf_.cycle_timeout_count;
#endif
        HandleCommunicationFailure();
        return;
    }

    esp_err_t err = ESP_FAIL;

#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_POSITION
    err = SendSyncPosition(cycle_command_);
#else
    err = SendSyncPWM(cycle_command_);
#endif

    if (err != ESP_OK)
    {
        EnterError();
        return;
    }

    cycle_write_sent_ = true;
    ChangeCommState(MotorCommState::COMPLETE);
}

void MotorInterface::CommComplete()
{
#if MOTOR_PERF_LOG_ENABLE
    const int64_t elapsed_us =
        esp_timer_get_time() - comm_cycle_start_us_;

    ++perf_.cycle_count;
    perf_.cycle_total_us += elapsed_us;
    if (elapsed_us > perf_.cycle_max_us)
    {
        perf_.cycle_max_us = elapsed_us;
    }
#endif

    // All seven feedback responses were validated.
    feedback_snapshot_.Write(cycle_feedback_);
#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_POSITION
    if (state_ == MotorInterfaceState::FAULT_HOLD)
    {
        bool stable = cycle_write_sent_;
        if (stable)
        {
            for (size_t i = 0; i < MOTOR_COUNT; ++i)
            {
                const int32_t diff =
                    static_cast<int32_t>(hold_command_.motors[i].target_position) -
                    static_cast<int32_t>(cycle_feedback_.motors[i].position);
                const int32_t velocity = cycle_feedback_.motors[i].velocity;
                if (std::abs(diff) > 10 || std::abs(velocity) > 50)
                { stable = false; break; }
            }
        }
        fault_hold_stable_cycles_ = stable ?
            static_cast<uint8_t>(std::min<int>(fault_hold_stable_cycles_ + 1, 10)) : 0;
        fault_hold_status_.store(fault_hold_stable_cycles_ >= 10 ?
            FaultHoldStatus::STABLE : FaultHoldStatus::WRITE_SENT);
    }
    if (state_ == MotorInterfaceState::ARMED)
    {
        const HoldTestStatus status = hold_test_status_.load();
        if ((status == HoldTestStatus::CAPTURED ||
             status == HoldTestStatus::WRITE_SENT) && cycle_write_sent_)
        {
            bool stable = true;
            for (size_t i = 0; i < MOTOR_COUNT; ++i)
            {
                const int32_t diff =
                    static_cast<int32_t>(hold_command_.motors[i].target_position) -
                    static_cast<int32_t>(cycle_feedback_.motors[i].position);
                const int32_t vel = cycle_feedback_.motors[i].velocity;
                if (std::abs(diff) > 10 || std::abs(vel) > 50)
                { stable = false; break; }
            }
            hold_test_stable_cycles_ = stable ?
                static_cast<uint8_t>(std::min<int>(hold_test_stable_cycles_ + 1, 10)) : 0;
            hold_test_status_.store(
                hold_test_stable_cycles_ >= 10 ?
                    HoldTestStatus::FEEDBACK_STABLE : HoldTestStatus::WRITE_SENT);
        }
    }

    if (state_ == MotorInterfaceState::HOMING && home_command_initialized_)
    {
        const bool moving = home_phase_ == HomePhase::MOVE_BASE ||
            home_phase_ == HomePhase::MOVE_SHOULDER ||
            home_phase_ == HomePhase::MOVE_REST;
        if (moving)
        {
            bool stable = true;
            for (size_t i = 0; i < MOTOR_COUNT; ++i)
            {
                if (HOME_GOALS[i] < 0 || !IsHomeStageServo(i)) continue;
                const int32_t diff = HOME_GOALS[i] - cycle_feedback_.motors[i].position;
                const int32_t vel = cycle_feedback_.motors[i].velocity;
                if (diff < -10 || diff > 10 || vel < -50 || vel > 50)
                { stable = false; break; }
            }
            home_stable_count_ = stable ?
                static_cast<uint8_t>(std::min<int>(home_stable_count_ + 1, 10)) : 0;
        }
    }
    if (state_ == MotorInterfaceState::PARKING &&
        park_phase_ == ParkPhase::MOVE)
    {
        bool stable = true;
        for (size_t i = 0; i < MOTOR_COUNT; ++i)
        {
            if (PARK_GOALS[i] < 0 || !IsParkStageServo(i)) continue;
            const int32_t diff = PARK_GOALS[i] - cycle_feedback_.motors[i].position;
            const int32_t vel = cycle_feedback_.motors[i].velocity;
            if (diff < -10 || diff > 10 || vel < -50 || vel > 50)
            {
                stable = false;
                break;
            }
        }
        park_stable_count_ = stable ?
            static_cast<uint8_t>(std::min<int>(park_stable_count_ + 1, 10)) : 0;
    }
#endif

    #if MOTOR_CONTROL_MODE == MOTOR_CONTROL_POSITION
    static int64_t last_log_us = 0;
    const int64_t now_us = esp_timer_get_time();

    if (now_us - last_log_us >= 1000000)
    {
        last_log_us = now_us;

        for (size_t i = 0; i < MOTOR_COUNT; ++i)
        {
            const auto& fb = cycle_feedback_.motors[i];

            ESP_LOGI(
                TAG,
                "POSITION FB id=%u pos=%d vel=%d current=%d valid=%u",
                static_cast<unsigned>(motors_[i].GetID()),
                static_cast<int>(fb.position),
                static_cast<int>(fb.velocity),
                static_cast<int>(fb.current),
                static_cast<unsigned>(fb.valid)
            );
        }
    }
#endif

    ChangeCommState(MotorCommState::IDLE);
}


void MotorInterface::RecordCommunicationError()
{
    const int64_t now_us = esp_timer_get_time();

    if (comm_error_count_ == 0 ||
        now_us - first_error_us_ >= ERROR_WINDOW_US)
    {
        comm_error_count_ = 1;
        first_error_us_ = now_us;
    }
    else
    {
        ++comm_error_count_;
    }

    last_error_us_ = now_us;

    if (comm_error_count_ >= ERROR_THRESHOLD)
    {
        EnterError();
    }
}

void MotorInterface::UpdateCommunicationError()
{
    if (comm_error_count_ == 0)
        return;

    const int64_t now_us = esp_timer_get_time();

    if (now_us - last_error_us_ >= ERROR_WINDOW_US)
    {
        comm_error_count_ = 0;
        first_error_us_ = 0;
        last_error_us_ = 0;
    }
}


void MotorInterface::HandleCommunicationFailure()
{
    if (state_ == MotorInterfaceState::ERROR)
        return;

    ChangeControlValidity(false);
#if MOTOR_PERF_LOG_ENABLE
    ++perf_.error_count;
#endif

    // Stop normal output updates immediately.
    output_inhibit_ = true;

    // Abort the current communication cycle.
    ResetReceive();

    ChangeCommState(MotorCommState::IDLE);
    current_motor_index_ = 0;

    // Invalidate all feedback in this cycle.
    cycle_feedback_ = {};

    for (size_t i = 0; i < MOTOR_COUNT; ++i)
    {
        motors_[i].SetFeedback(MotorFeedback{});
    }

    // Publish invalid feedback.
    feedback_snapshot_.Write(cycle_feedback_);

    // Count this failure once.
    RecordCommunicationError();
}

void MotorInterface::EnterError()
{
    if (state_ == MotorInterfaceState::ERROR)
        return;

    ESP_LOGE(
        TAG,
        "ENTER ERROR: motor_state=%u init_state=%u "
        "servo_index=%u param_result=%u param_error=%s",
        static_cast<unsigned>(state_),
        static_cast<unsigned>(init_state_),
        static_cast<unsigned>(init_motor_index_),
        static_cast<unsigned>(parameter_fsm_.GetResult()),
        esp_err_to_name(parameter_fsm_.GetError())
    );
        
    if (state_ == MotorInterfaceState::FAULT_HOLD)
        fault_hold_status_.store(FaultHoldStatus::FAILED);
    if (hold_test_status_.load() == HoldTestStatus::PENDING ||
        hold_test_status_.load() == HoldTestStatus::CAPTURED ||
        hold_test_status_.load() == HoldTestStatus::WRITE_SENT)
        hold_test_status_.store(HoldTestStatus::FAILED);

    // Capture diagnostic BEFORE EnterError invalidates feedback snapshots.
    // Never attempt to send a new goal here: UART/ID13 torque may be uncertain.
    if (state_ == MotorInterfaceState::HOMING)
    {
        SharedSnapshotFrame<MotorFeedbackFrame> feedback{};
        const bool feedback_fresh = GetFeedbackFrame(feedback) &&
            ValidateFeedback(feedback.data, esp_timer_get_time());
        const uint32_t id13 = static_cast<uint32_t>(home_id13_verified_.load());
        const uint32_t phase = static_cast<uint32_t>(home_phase_) + 1u;
        home_fault_diagnostic_.store(
            1u | (feedback_fresh ? 2u : 0u) |
            (id13 == static_cast<uint32_t>(HomeId13Verified::OFF) ? 4u : 0u) |
            (phase << 8) | (id13 << 16));
    }

    // PID reset is handled once by EnterState(ERROR).

#if MOTOR_PERF_LOG_ENABLE
    // Count immediate fatal errors, but avoid recounting the
    // failure already recorded by HandleCommunicationFailure().
    if (comm_error_count_ < ERROR_THRESHOLD)
    {
        ++perf_.error_count;
    }
#endif

    ChangeState(MotorInterfaceState::ERROR);
    output_inhibit_ = true;

    ChangeCommState(MotorCommState::IDLE);
    current_motor_index_ = 0;

    ResetReceive();

    cycle_feedback_ = {};

    for (size_t i = 0; i < MOTOR_COUNT; ++i)
    {
        motors_[i].SetFeedback(MotorFeedback{});
    }

    feedback_snapshot_.Write(cycle_feedback_);

    // Physical torque may remain ON after partial ARM or communication failure.
    // Safe stop requires later hardware validation.
}

void MotorInterface::CommRxRecovery()
{
    // Discard pending RX bytes.
    const esp_err_t ret = uart_flush_input(uart_num_);

    if (ret != ESP_OK)
    {
        EnterError();
        return;
    }

    const int64_t now_us = esp_timer_get_time();

    if (now_us - rx_recovery_start_us_ <
        RX_RECOVERY_US)
    {
        return;
    }

    // Final cleanup before returning to IDLE.
    if (uart_flush_input(uart_num_) != ESP_OK)
    {
        EnterError();
        return;
    }

    HandleCommunicationFailure();
}

#if MOTOR_PERF_LOG_ENABLE

void MotorInterface::UpdatePerformanceLog()
{
    const int64_t now_us = esp_timer_get_time();

    if (perf_.window_start_us == 0)
    {
        perf_.window_start_us = now_us;
        return;
    }

    const int64_t window_us = now_us - perf_.window_start_us;
    if (window_us < MOTOR_PERF_LOG_PERIOD_US)
    {
        return;
    }

    const double task_hz = static_cast<double>(perf_.task_count) *
        1000000.0 / static_cast<double>(window_us);
    const double cycle_hz = static_cast<double>(perf_.cycle_count) *
        1000000.0 / static_cast<double>(window_us);

    const int64_t cycle_avg_us = (perf_.cycle_count != 0)
        ? perf_.cycle_total_us / perf_.cycle_count : 0;

    ESP_LOGI(TAG,
             "PERF task=%.1fHz cycle=%.1fHz "
             "avg=%lldus max=%lldus "
             "error=%lu timeout=%lu",
             task_hz,
             cycle_hz,
             static_cast<long long>(cycle_avg_us),
             static_cast<long long>(perf_.cycle_max_us),
             static_cast<unsigned long>(perf_.error_count),
             static_cast<unsigned long>(perf_.cycle_timeout_count));

    perf_ = {};
    perf_.window_start_us = now_us;
}

#endif  // MOTOR_PERF_LOG_ENABLE


void MotorInterface::ProcessMotorInitialization()
{
    if (state_ != MotorInterfaceState::INIT)
        return;

    // No UART ownership overlap: this is only invoked during INIT.
    if (init_state_ != MotorInitState::COMPLETE &&
        init_motor_index_ >= MOTOR_COUNT)
    {
        init_state_ = MotorInitState::ERROR;
    }

    switch (init_state_)
    {
        case MotorInitState::START:
            init_motor_index_ = 0;
            init_state_ = MotorInitState::SET_MODE;
            break;

        case MotorInitState::SET_MODE:
        {
            const uint8_t id = motors_[init_motor_index_].GetID();
#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_POSITION

    const esp_err_t err =
        parameter_fsm_.VerifyOperatingMode(
            id, ST3215ParameterFSM::Mode::POSITION);

#elif MOTOR_EEPROM_WRITE_ENABLE

    const esp_err_t err =
        parameter_fsm_.SetOperatingMode(
            id, ST3215ParameterFSM::Mode::PWM_OPEN);

#else

    const esp_err_t err =
        parameter_fsm_.VerifyOperatingMode(
            id, ST3215ParameterFSM::Mode::PWM_OPEN);

#endif
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "INIT mode request failed: servo=%u err=%s",
                         static_cast<unsigned>(id), esp_err_to_name(err));
                init_state_ = MotorInitState::ERROR;
                EnterError();
                return;
            }
            init_state_ = MotorInitState::WAIT_MODE;
            break;
        }

        case MotorInitState::WAIT_MODE:
        {
            const auto result = parameter_fsm_.Poll();

            if (result == ST3215ParameterFSM::Result::SUCCESS)
            {
                init_state_ = MotorInitState::SET_TORQUE_DIAG;
            }
            else if (result == ST3215ParameterFSM::Result::FAILED)
            {
                if (parameter_fsm_.HasValueMismatch())
                {
                    ++init_mismatch_count_;

                    ESP_LOGI(
                        TAG,
                        "INIT mode mismatch accepted for diagnosis: id=%u",
                        static_cast<unsigned>(parameter_fsm_.GetServoID())
                    );

                    init_state_ = MotorInitState::SET_TORQUE_DIAG;
                }
                else
                {
                    ESP_LOGE(
                        TAG,
                        "INIT mode communication failure: id=%u err=%s",
                        static_cast<unsigned>(parameter_fsm_.GetServoID()),
                        esp_err_to_name(parameter_fsm_.GetError())
                    );

                    init_state_ = MotorInitState::ERROR;
                    EnterError();
                }
            }
            break;
        }

        case MotorInitState::SET_TORQUE_DIAG:
        {
            const uint8_t id =
                motors_[init_motor_index_].GetID();

            const esp_err_t err =
                parameter_fsm_.VerifyTorqueEnable(id, false);

            if (err != ESP_OK)
            {
                ESP_LOGE(
                    TAG,
                    "INIT torque read request failed: id=%u",
                    static_cast<unsigned>(id)
                );

                init_state_ = MotorInitState::ERROR;
                EnterError();
                return;
            }

            init_state_ = MotorInitState::WAIT_TORQUE_DIAG;
            break;
        }

        case MotorInitState::WAIT_TORQUE_DIAG:
        {
            const auto result = parameter_fsm_.Poll();

            if (result == ST3215ParameterFSM::Result::SUCCESS)
            {
                init_state_ = MotorInitState::SET_TORQUE_LIMIT;
            }
            else if (result == ST3215ParameterFSM::Result::FAILED)
            {
                if (parameter_fsm_.HasValueMismatch())
                {
                    ++init_torque_on_count_;
                    ESP_LOGW(TAG,
                        "INIT servo=%u torque is ON after MCU restart",
                        static_cast<unsigned>(parameter_fsm_.GetServoID()));
                    init_state_ = MotorInitState::SET_TORQUE_LIMIT;
                }
                else
                {
                    ESP_LOGE(
                        TAG,
                        "INIT torque communication failure: id=%u err=%s",
                        static_cast<unsigned>(parameter_fsm_.GetServoID()),
                        esp_err_to_name(parameter_fsm_.GetError())
                    );

                    init_state_ = MotorInitState::ERROR;
                    EnterError();
                }
            }
            break;
        }


        case MotorInitState::SET_TORQUE_LIMIT:
        {
#if MOTOR_INIT_TORQUE_LIMIT_ENABLE
            const uint8_t id = motors_[init_motor_index_].GetID();
            const esp_err_t err = parameter_fsm_.SetTorqueLimit(
                id, MOTOR_INIT_TORQUE_LIMIT);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "INIT torque-limit request failed: servo=%u err=%s",
                         static_cast<unsigned>(id), esp_err_to_name(err));
                init_state_ = MotorInitState::ERROR;
                EnterError();
                return;
            }
            init_state_ = MotorInitState::WAIT_TORQUE_LIMIT;
#else
            // No torque-limit modification until the limit is chosen for the arm.
            init_state_ = MotorInitState::NEXT_MOTOR;
#endif
            break;
        }

        case MotorInitState::WAIT_TORQUE_LIMIT:
        {
            const auto result = parameter_fsm_.Poll();
            if (result == ST3215ParameterFSM::Result::SUCCESS)
            {
                init_state_ = MotorInitState::NEXT_MOTOR;
            }
            else if (result == ST3215ParameterFSM::Result::FAILED)
            {
                ESP_LOGE(TAG, "INIT torque-limit verification failed: servo=%u err=%s",
                         static_cast<unsigned>(parameter_fsm_.GetServoID()),
                         esp_err_to_name(parameter_fsm_.GetError()));
                init_state_ = MotorInitState::ERROR;
                EnterError();
            }
            break;
        }

        case MotorInitState::NEXT_MOTOR:
            ++init_motor_index_;
            init_state_ = (init_motor_index_ < MOTOR_COUNT)
                            ? MotorInitState::SET_MODE
                            : MotorInitState::COMPLETE;
            break;

        case MotorInitState::COMPLETE:
            // StateInit() performs the transition to READY.
            break;

        case MotorInitState::ERROR:
        default:
            init_state_ = MotorInitState::ERROR;
            EnterError();
            break;
    }
}

bool MotorInterface::ValidateCommand(
    const SharedSnapshotFrame<MotorCommandFrame>& frame,
    int64_t now_us) const
{
    if (!frame.valid)
        return false;

    const int64_t age_us = now_us - frame.timestamp_us;

    return age_us >= 0 && age_us <= COMMAND_TIMEOUT_US;
}

bool MotorInterface::ValidateFeedback(
    const MotorFeedbackFrame& frame,
    int64_t now_us) const
{
    for (size_t i = 0; i < MOTOR_COUNT; ++i)
    {
        const MotorFeedback& fb = frame.motors[i];

        if (!fb.valid)
            return false;

        const int64_t age_us = now_us - fb.timestamp_us;

        if (age_us < 0 || age_us > FEEDBACK_TIMEOUT_US)
            return false;
    }

    return true;
}

void MotorInterface::InitializePID()
{
    for (size_t i = 0; i < MOTOR_COUNT; ++i)
    {
        motor_pid_[i].SetIntegralLeak(0.3f);
        motor_pid_[i].SetIntegralZone(0.0f); // disable angle-unit I-zone for raw encoder counts
        motor_pid_[i].SetGains(
            PIDGains{0.0f, 0.0f, 0.0f}
        );

        motor_pid_[i].SetLimits(
            PIDLimits{
                -100.0f, 100.0f,
                -1000.0f, 1000.0f
            }
        );

        motor_pid_[i].Reset();
    }
}

bool MotorInterface::ProcessPID(float dt)
{
    for (size_t i = 0; i < MOTOR_COUNT; ++i)
    {
        const float target =
            static_cast<float>(
                cycle_command_.motors[i].target_position
            );

        const float actual =
            static_cast<float>(
                cycle_feedback_.motors[i].position
            );

        const float output = motor_pid_[i].Update(
            target,
            actual,
            dt
        );

        if (!std::isfinite(output))
            return false;

        const float limited = std::clamp(
            output,
            static_cast<float>(PWM_MIN),
            static_cast<float>(PWM_MAX)
        );

        cycle_command_.motors[i].output =
            static_cast<int16_t>(std::lround(limited));
    }

    return true;
}

void MotorInterface::ChangeControlValidity(bool valid)
{
    if (control_valid_ == valid)
        return;

    control_valid_ = valid;
    if (!valid)
        ResetPID();
}

void MotorInterface::ResetPID()
{
    for (size_t i = 0; i < MOTOR_COUNT; ++i)
    {
        motor_pid_[i].Reset();
    }

    last_pid_update_us_ = 0;
}