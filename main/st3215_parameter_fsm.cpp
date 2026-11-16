#include "st3215_parameter_fsm.h"
#include <cstring>

namespace {
constexpr uint8_t RETURN_LEVEL_ADDR = 8;
constexpr uint8_t TORQUE_ENABLE_ADDR = 40;
constexpr uint8_t EEPROM_LOCK_ADDR = 55;
}

bool ST3215ParameterFSM::IsWrite(Step s)
{
    return s == Step::WRITE_UNLOCK || s == Step::WRITE_VALUE ||
           s == Step::WRITE_LOCK;
}
uint8_t ST3215ParameterFSM::ReadAddress(Step s, uint8_t target)
{
    switch (s) {
    case Step::READ_RETURN_LEVEL: return RETURN_LEVEL_ADDR;
    case Step::READ_TORQUE: return TORQUE_ENABLE_ADDR;
    case Step::WRITE_UNLOCK: case Step::READ_UNLOCK:
    case Step::WRITE_LOCK: case Step::READ_LOCK: return EEPROM_LOCK_ADDR;
    default: return target;
    }
}
uint8_t ST3215ParameterFSM::ReadSize(Step s, uint8_t target_size)
{
    switch (s) {
    case Step::READ_RETURN_LEVEL: case Step::READ_TORQUE:
    case Step::WRITE_UNLOCK: case Step::READ_UNLOCK:
    case Step::WRITE_LOCK: case Step::READ_LOCK: return 1;
    default: return target_size;
    }
}

esp_err_t ST3215ParameterFSM::Begin(uint8_t id, Register reg,
                                   const uint8_t* data, size_t size,
                                   Storage storage)
{
    if (result_ == Result::BUSY || unlock_may_have_happened_)
        return ESP_ERR_INVALID_STATE;
    if (id > 0xFD || !data || size == 0 || size > desired_.size())
        return ESP_ERR_INVALID_ARG;
    const auto a = static_cast<uint8_t>(reg);
    const bool valid = (a == 33 && size == 1 && storage == Storage::EEPROM) ||
                       (a == 40 && size == 1 && storage == Storage::RAM) ||
                       (a == 48 && size == 2 && storage == Storage::RAM);
    // ID, baud and zero-offset are excluded from normal runtime settings.
    if (!valid) return ESP_ERR_NOT_SUPPORTED;
    if (a == 33 && data[0] > 3) return ESP_ERR_INVALID_ARG;
    if (a == 40 && data[0] > 1) return ESP_ERR_INVALID_ARG;
    if (a == 48 && (static_cast<uint16_t>(data[0]) |
                    (static_cast<uint16_t>(data[1]) << 8)) > 1000)
        return ESP_ERR_INVALID_ARG;

    id_ = id; address_ = a; size_ = static_cast<uint8_t>(size);
    desired_ = {}; std::memcpy(desired_.data(), data, size);
    storage_ = storage; return_level_ = 1;
    unlock_may_have_happened_ = false;
    failed_pending_relock_ = false;
    error_ = ESP_OK; result_ = Result::BUSY;
    Next(Step::READ_RETURN_LEVEL);
    return ESP_OK;
}

esp_err_t ST3215ParameterFSM::SetOperatingMode(uint8_t id, Mode mode)
{
    const uint8_t value = static_cast<uint8_t>(mode);
    return Begin(id, Register::OPERATING_MODE, &value, 1, Storage::EEPROM);
}
esp_err_t ST3215ParameterFSM::SetTorqueEnable(uint8_t id, bool enabled)
{
    const uint8_t value = enabled ? 1 : 0;
    return Begin(id, Register::TORQUE_ENABLE, &value, 1, Storage::RAM);
}
esp_err_t ST3215ParameterFSM::SetTorqueLimit(uint8_t id, uint16_t limit)
{
    if (limit > 1000) return ESP_ERR_INVALID_ARG;
    const uint8_t value[2] = {static_cast<uint8_t>(limit),
                              static_cast<uint8_t>(limit >> 8)};
    return Begin(id, Register::TORQUE_LIMIT, value, 2, Storage::RAM);
}

void ST3215ParameterFSM::Next(Step step)
{
    step_ = step; io_ = IO::NOT_SENT;
    rx_size_ = 0; expected_rx_size_ = 0; expected_params_ = 0;
}
void ST3215ParameterFSM::Fail(esp_err_t error)
{
    if (error_ == ESP_OK) error_ = error;
    if (unlock_may_have_happened_ &&
        step_ != Step::WRITE_LOCK && step_ != Step::READ_LOCK) {
        failed_pending_relock_ = true;
        Next(Step::WRITE_LOCK);
    } else {
        result_ = Result::FAILED;
    }
}

esp_err_t ST3215ParameterFSM::SendStep()
{
    const bool write = IsWrite(step_);
    const uint8_t address = ReadAddress(step_, address_);
    const uint8_t size = ReadSize(step_, size_);
    uint8_t special_value = 0;
    const uint8_t* data = desired_.data();
    if (step_ == Step::WRITE_UNLOCK || step_ == Step::WRITE_LOCK) {
        special_value = (step_ == Step::WRITE_LOCK) ? 1 : 0;
        data = &special_value;
    }
    ST3215Request req{};
    req.command = write ? ST3215Command::WRITE : ST3215Command::READ;
    req.id = id_; req.address = address; req.data_size = size;
    if (write) { req.data = data; req.data_len = size; }
    uint8_t tx[ST3215::MAX_PACKET_SIZE]{};
    size_t tx_size = 0;
    esp_err_t err = ST3215::Serialize(req, tx, sizeof(tx), tx_size);
    if (err != ESP_OK) return err;

    // TX/RX exclusively owned by this task; discarded bytes are never from
    // another valid transaction. WRITE ACK is expected only at return level 2.
    err = uart_flush_input(uart_);
    if (err != ESP_OK) return err;
    if (step_ == Step::WRITE_UNLOCK) unlock_may_have_happened_ = true;
    const int written = uart_write_bytes(uart_, tx, tx_size);
    if (written < 0 || static_cast<size_t>(written) != tx_size) return ESP_FAIL;
    err = uart_wait_tx_done(uart_, pdMS_TO_TICKS(2));
    if (err != ESP_OK) return err;

    expected_params_ = write ? 0 : size;
    rx_started_us_ = esp_timer_get_time();
    rx_size_ = 0; expected_rx_size_ = 0;
    io_ = (write && return_level_ != 2) ? IO::NO_REPLY_WAIT : IO::WAIT_REPLY;
    return ESP_OK;
}

esp_err_t ST3215ParameterFSM::ReadReply(ST3215Response& response, bool& complete)
{
    complete = false;
    if (esp_timer_get_time() - rx_started_us_ >= RESPONSE_TIMEOUT_US)
        return ESP_ERR_TIMEOUT;
    while (true) {
        size_t wanted = expected_rx_size_ ? expected_rx_size_ : 4;
        if (rx_size_ == wanted) {
            if (!expected_rx_size_) {
                if (rx_[0] != 0xFF || rx_[1] != 0xFF || rx_[2] != id_ ||
                    rx_[3] != static_cast<uint8_t>(expected_params_ + 2))
                    return ESP_ERR_INVALID_RESPONSE;
                expected_rx_size_ = static_cast<size_t>(rx_[3]) + 4;
                continue;
            }
            const esp_err_t err = ST3215::ParseResponse(
                rx_.data(), rx_size_, id_, expected_params_, response);
            if (err != ESP_OK) return err;
            if (response.error != 0) return ESP_ERR_INVALID_RESPONSE;
            complete = true;
            return ESP_OK;
        }
        const int n = uart_read_bytes(uart_, rx_.data() + rx_size_,
                                      wanted - rx_size_, 0);
        if (n < 0) return ESP_FAIL;
        if (n == 0) return ESP_OK;
        rx_size_ += static_cast<size_t>(n);
    }
}

void ST3215ParameterFSM::CompleteStep(const ST3215Response* response)
{
    switch (step_) {
    case Step::READ_RETURN_LEVEL:
        return_level_ = response->params[0];
        if (return_level_ < 1 || return_level_ > 2) {
            Fail(ESP_ERR_INVALID_STATE); return;
        }
        Next(Step::READ_CURRENT); break;
    case Step::READ_CURRENT:
        if (std::memcmp(response->params, desired_.data(), size_) == 0) {
            result_ = Result::SUCCESS;
        } else if (storage_ == Storage::EEPROM) {
            Next(Step::READ_TORQUE);
        } else {
            Next(Step::WRITE_VALUE);
        }
        break;
    case Step::READ_TORQUE:
        if (response->params[0] != 0) {
            Fail(ESP_ERR_INVALID_STATE);
        } else {
            Next(Step::WRITE_UNLOCK);
        }
        break;
    case Step::WRITE_UNLOCK: Next(Step::READ_UNLOCK); break;
    case Step::READ_UNLOCK:
        if (response->params[0] != 0) Fail(ESP_ERR_INVALID_STATE);
        else Next(Step::WRITE_VALUE);
        break;
    case Step::WRITE_VALUE: Next(Step::READ_VALUE); break;
    case Step::READ_VALUE:
        if (std::memcmp(response->params, desired_.data(), size_) != 0) {
            Fail(ESP_ERR_INVALID_RESPONSE);
        } else if (storage_ == Storage::EEPROM) {
            Next(Step::WRITE_LOCK);
        } else {
            result_ = Result::SUCCESS;
        }
        break;
    case Step::WRITE_LOCK: Next(Step::READ_LOCK); break;
    case Step::READ_LOCK:
        if (response->params[0] != 1) {
            Fail(ESP_ERR_INVALID_STATE);
        } else {
            unlock_may_have_happened_ = false;
            result_ = failed_pending_relock_ ? Result::FAILED : Result::SUCCESS;
        }
        break;
    case Step::DONE: break;
    }
}

ST3215ParameterFSM::Result ST3215ParameterFSM::Poll()
{
    if (result_ != Result::BUSY) return result_;
    if (io_ == IO::NOT_SENT) {
        const esp_err_t err = SendStep();
        if (err != ESP_OK) Fail(err);
        return result_;
    }
    if (io_ == IO::NO_REPLY_WAIT) {
        CompleteStep(nullptr);
        return result_;
    }
    ST3215Response response{};
    bool complete = false;
    const esp_err_t err = ReadReply(response, complete);
    if (err != ESP_OK) Fail(err);
    else if (complete) CompleteStep(&response);
    return result_;
}
