
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




void MotorInterface::ChangeState(MotorInterfaceState next)
{
    if (state_ == next)
        return;

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
            ResetPID();
            break;
        case MotorInterfaceState::READY:
            torque_off_public_.store(false);
            output_inhibit_ = true;
            torque_index_ = 0;
            torque_busy_ = false;
            torque_off_verified_ = false;
            break;
        case MotorInterfaceState::DISARMED:
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
        case MotorInterfaceState::PARKING:
            park_stop_pending_ = false;
            output_inhibit_ = true;
            park_phase_ = ParkPhase::RELEASE_SHOULDER;
            park_start_us_ = esp_timer_get_time();
            park_stable_count_ = 0;
            park_busy_ = false;
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
        case MotorInterfaceState::ARMING: StateArming(); break;
        case MotorInterfaceState::ARMED: StateArmed(); break;
        case MotorInterfaceState::PARKING: StateParking(); break;
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

    if (init_mismatch_count_ != 0)
    {
        ESP_LOGW(
            TAG,
            "INIT rejected: servo configuration mismatch"
        );

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
        if (park_busy_)
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
            park_phase_ = ParkPhase::RESTORE_SHOULDER;
            return;
        }
        UpdateCommunicationError();
        ProcessCommunicationFSM();
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

void MotorInterface::StateDisarming()
{
#if MOTOR_TORQUE_WRITE_ENABLE
    if (PollTorque(false, false))
    {
        torque_off_verified_ = true;
        torque_off_public_.store(true);
        ChangeState(MotorInterfaceState::DISARMED);
    }
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
            state_ != MotorInterfaceState::ARMED &&
            state_ != MotorInterfaceState::PARKING)
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

    // DISARMED publishes feedback only; never runs PID or transmits commands.
    if (state_ != MotorInterfaceState::ARMED &&
        state_ != MotorInterfaceState::PARKING)
    {
        ChangeControlValidity(false);
        command_valid_ = false;
        cycle_command_ = {};
        output_inhibit_ = true;
        ChangeCommState(MotorCommState::COMPLETE);
        return;
    }

#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_POSITION
    // Initial hold and park targets are owned by the Servo FSM, not JSON.
    if (state_ == MotorInterfaceState::PARKING)
    {
        cycle_command_ = {};
        for (size_t i = 0; i < MOTOR_COUNT; ++i)
        {
            const int32_t current = cycle_feedback_.motors[i].position;
            const int32_t goal = PARK_GOALS[i] < 0 ? current : PARK_GOALS[i];
            const int32_t step = std::clamp(
                goal - current, -PARK_STEP_COUNTS, PARK_STEP_COUNTS);
            cycle_command_.motors[i].target_position =
                static_cast<int16_t>(current + step);
        }
    }
    else cycle_command_ = hold_command_;
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

    for (size_t i = 0; i < MOTOR_COUNT; ++i)
    {
        const int32_t target =
            command.motors[i].target_position;

        // 최초 단일 회전 위치 제어 검증 범위
        if (target < 0 || target > 4095)
        {
            return ESP_ERR_INVALID_ARG;
        }

        ids[i] = motors_[i].GetID();

        const uint16_t raw =
            static_cast<uint16_t>(target);

        position_data[i * 2] =
            static_cast<uint8_t>(raw & 0xFF);

        position_data[i * 2 + 1] =
            static_cast<uint8_t>((raw >> 8) & 0xFF);
    }

    ST3215Request request{};

    request.command = ST3215Command::SYNC_WRITE;
    request.address = POSITION_ADDRESS;
    request.data_size = POSITION_SIZE;
    request.servo_ids = ids;
    request.servo_count = MOTOR_COUNT;
    request.data = position_data;
    request.data_len = sizeof(position_data);

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
         state_ != MotorInterfaceState::PARKING) ||
        (servo_request_.load() == ServoRequest::OFF || park_stop_pending_) ||
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
    if (state_ == MotorInterfaceState::PARKING &&
        park_phase_ == ParkPhase::MOVE)
    {
        bool stable = true;
        for (size_t i = 0; i < MOTOR_COUNT; ++i)
        {
            if (PARK_GOALS[i] < 0) continue;
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
                    ++init_mismatch_count_;
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