#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "driver/uart.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "st3215.h"

// Single owner: call Begin()/Poll() only from the MotorInterface UART task.
// No servo writes are started automatically by this class.
class ST3215ParameterFSM final
{
public:
    enum class Storage : uint8_t { RAM, EEPROM };
    enum class Result : uint8_t { IDLE, BUSY, SUCCESS, FAILED };
    enum class Mode : uint8_t {
        POSITION = 0, SPEED_CLOSED = 1, PWM_OPEN = 2, STEP = 3
    };
    enum class Register : uint8_t {
        POSITION_OFFSET = 31, OPERATING_MODE = 33,
        TORQUE_ENABLE = 40, TORQUE_LIMIT = 48, EEPROM_LOCK = 55
    };

    ST3215ParameterFSM() = default;
    void SetPort(uart_port_t uart) { uart_ = uart; }

    // Requests are copied into owned storage; ESP_OK means accepted, not verified.
    esp_err_t Begin(uint8_t id, Register reg, const uint8_t* data,
                    size_t size, Storage storage, bool verify_only = false);
    esp_err_t SetOperatingMode(uint8_t id, Mode mode);
    // READ + compare only. Never performs WRITE, including EEPROM unlock.
    esp_err_t VerifyOperatingMode(uint8_t id, Mode mode);
    esp_err_t SetTorqueEnable(uint8_t id, bool enabled);
    esp_err_t SetTorqueLimit(uint8_t id, uint16_t limit);
    // Position-offset writes intentionally require an explicit maintenance path.
    Result Poll();
    Result GetResult() const { return result_; }
    esp_err_t GetError() const { return error_; }
    uint8_t GetServoID() const { return id_; }

private:
    enum class Step : uint8_t {
        READ_RETURN_LEVEL, READ_CURRENT, READ_TORQUE,
        WRITE_UNLOCK, READ_UNLOCK, WRITE_VALUE, READ_VALUE,
        WRITE_LOCK, READ_LOCK, DONE
    };
    enum class IO : uint8_t { NOT_SENT, WAIT_REPLY, NO_REPLY_WAIT };

    esp_err_t SendStep();
    esp_err_t ReadReply(ST3215Response& response, bool& complete);
    void CompleteStep(const ST3215Response* response);
    void Fail(esp_err_t error);
    void Next(Step step);
    static bool IsWrite(Step step);
    static uint8_t ReadAddress(Step step, uint8_t target);
    static uint8_t ReadSize(Step step, uint8_t target_size);

    uart_port_t uart_ = UART_NUM_MAX;
    Result result_ = Result::IDLE;
    esp_err_t error_ = ESP_OK;
    Step step_ = Step::READ_RETURN_LEVEL;
    IO io_ = IO::NOT_SENT;

    uint8_t id_ = 0;
    uint8_t address_ = 0;
    uint8_t size_ = 0;
    std::array<uint8_t, 2> desired_{};
    Storage storage_ = Storage::RAM;
    bool verify_only_ = false;
    uint8_t return_level_ = 1;
    bool unlock_may_have_happened_ = false;
    bool failed_pending_relock_ = false;

    int64_t rx_started_us_ = 0;
    std::array<uint8_t, ST3215::MAX_PACKET_SIZE> rx_{};
    size_t rx_size_ = 0;
    size_t expected_rx_size_ = 0;
    uint8_t expected_params_ = 0;
    static constexpr int64_t RESPONSE_TIMEOUT_US = 3000;
};
