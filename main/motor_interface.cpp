
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

bool MotorInterface::GetFeedbackFrame(
    SharedSnapshotFrame<MotorFeedbackFrame>& out)
{
    return feedback_snapshot_.Read(out);
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
            pdMS_TO_TICKS(1)
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



void MotorInterface::ProcessInterfaceFSM()
{
    switch (state_)
    {
        case MotorInterfaceState::INIT:
            StateInit();
            break;

        case MotorInterfaceState::READY:
            StateReady();
            break;

        case MotorInterfaceState::RUNNING:
            StateRunning();
            break;

        case MotorInterfaceState::ERROR:
            StateError();
            break;
    }
}

void MotorInterface::StateInit()
{
    // Board UART / motor initialization
    // is already performed by Initialize().

    ProcessMotorInitialization();

    // Only enter RUNNING after every servo passed INIT verification.
    if (state_ == MotorInterfaceState::ERROR ||
        init_state_ != MotorInitState::COMPLETE)
    {
        return;
    }

    comm_state_ = MotorCommState::IDLE;
    current_motor_index_ = 0;
    state_ = MotorInterfaceState::READY;
}

void MotorInterface::StateReady()
{
    state_ = MotorInterfaceState::RUNNING;
}

void MotorInterface::StateRunning()
{
    UpdateCommunicationError();
    ProcessCommunicationFSM();

    if (state_ != MotorInterfaceState::RUNNING)
        return;
}

void MotorInterface::StateError()
{
    // Communication recovery / safe output
    // will be implemented after the
    // communication FSM is connected.
}




void MotorInterface::ProcessCommunicationFSM()
{
    while (true)
    {
        if (state_ != MotorInterfaceState::RUNNING)
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
            rx_recovery_start_us_ = esp_timer_get_time();
            comm_state_ = MotorCommState::RX_RECOVERY;
            output_inhibit_ = true;
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

    comm_cycle_start_us_ = esp_timer_get_time();
    // A new cycle must not inherit valid flags
    // from the previous cycle.
    cycle_feedback_ = {};

    comm_state_ = MotorCommState::SEND_REQUEST;
}


void MotorInterface::CommSendRequest()
{
    if (current_motor_index_ >= MOTOR_COUNT)
    {
        comm_state_ = MotorCommState::COMPLETE;
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

    comm_state_ = MotorCommState::WAIT_RESPONSE;
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
        rx_recovery_start_us_ = esp_timer_get_time();
        comm_state_ = MotorCommState::RX_RECOVERY;
        output_inhibit_ = true;
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
        rx_recovery_start_us_ = esp_timer_get_time();
        comm_state_ = MotorCommState::RX_RECOVERY;
        output_inhibit_ = true;
        return;
    }

    motors_[current_motor_index_].SetFeedback(feedback);

    cycle_feedback_.motors[current_motor_index_] =
        feedback;

    comm_state_ = MotorCommState::NEXT_MOTOR;
}

void MotorInterface::CommNextMotor()
{
    ++current_motor_index_;

    if (current_motor_index_ < MOTOR_COUNT)
    {
        comm_state_ = MotorCommState::SEND_REQUEST;
    }
    else
    {
        comm_state_ = MotorCommState::PROCESS;
    }
}

void MotorInterface::CommProcess()
{
    // 6축 피드백 검증
    for (size_t i = 0; i < MOTOR_COUNT; ++i)
    {
        if (!cycle_feedback_.motors[i].valid)
        {
            HandleCommunicationFailure();
            return;
        }
    }

    SharedSnapshotFrame<MotorCommandFrame> cmd_frame{};

    command_valid_ = command_snapshot_.Read(cmd_frame);

    if (command_valid_)
    {
        cycle_command_ = cmd_frame.data;
    }

    output_inhibit_ = false;

    // TODO:
    // 6축 외부 PID 연산
    // 목표 위치와 실제 피드백을 이용한 PWM 계산

    comm_state_ = MotorCommState::SYNC_WRITE;
}

esp_err_t MotorInterface::SendSyncPWM(
    const MotorCommandFrame& command)
{
    uint8_t ids[MOTOR_COUNT]{};
    uint8_t pwm_data[MOTOR_COUNT * PWM_OUTPUT_SIZE]{};

    for (size_t i = 0; i < MOTOR_COUNT; ++i)
    {
        ids[i] = motors_[i].GetID();

        const esp_err_t ret = ST3215::EncodePWM(
            command.motors[i].output,
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


void MotorInterface::CommSyncWrite()
{
#if MOTOR_PWM_OUTPUT_ENABLE

    if (!command_valid_ || output_inhibit_)
    {
        comm_state_ = MotorCommState::COMPLETE;
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

    if (SendSyncPWM(cycle_command_) != ESP_OK)
    {
        EnterError();
        return;
    }

    cycle_write_sent_ = true;

#endif

    comm_state_ = MotorCommState::COMPLETE;
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

    // All six feedback responses were validated.
    feedback_snapshot_.Write(cycle_feedback_);

    // Allow normal output updates again.
    output_inhibit_ = false;

    comm_state_ = MotorCommState::IDLE;
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

#if MOTOR_PERF_LOG_ENABLE
    ++perf_.error_count;
#endif

    // Stop normal output updates immediately.
    output_inhibit_ = true;

    // Abort the current communication cycle.
    ResetReceive();

    comm_state_ = MotorCommState::IDLE;
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

#if MOTOR_PERF_LOG_ENABLE
    // Count immediate fatal errors, but avoid recounting the
    // failure already recorded by HandleCommunicationFailure().
    if (comm_error_count_ < ERROR_THRESHOLD)
    {
        ++perf_.error_count;
    }
#endif

    state_ = MotorInterfaceState::ERROR;
    output_inhibit_ = true;

    comm_state_ = MotorCommState::IDLE;
    current_motor_index_ = 0;

    ResetReceive();

    cycle_feedback_ = {};

    for (size_t i = 0; i < MOTOR_COUNT; ++i)
    {
        motors_[i].SetFeedback(MotorFeedback{});
    }

    feedback_snapshot_.Write(cycle_feedback_);

    // TODO:
    // SendEmergencyStop() - six-axis PWM zero.
    // Implement after verifying ST3215 PWM mode/registers.
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
#if MOTOR_EEPROM_WRITE_ENABLE
            const esp_err_t err = parameter_fsm_.SetOperatingMode(
                id, ST3215ParameterFSM::Mode::PWM_OPEN);
#else
            // EEPROM writes disabled: mode must still be READ-verified.
            const esp_err_t err = parameter_fsm_.VerifyOperatingMode(
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
                init_state_ = MotorInitState::SET_TORQUE_LIMIT;
            }
            else if (result == ST3215ParameterFSM::Result::FAILED)
            {
                ESP_LOGE(TAG, "INIT mode verification failed: servo=%u err=%s",
                         static_cast<unsigned>(parameter_fsm_.GetServoID()),
                         esp_err_to_name(parameter_fsm_.GetError()));
                init_state_ = MotorInitState::ERROR;
                EnterError();
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
