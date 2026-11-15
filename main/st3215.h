
#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"
#include "driver/uart.h"

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

static constexpr uint8_t FEEDBACK_ADDRESS = 56;
static constexpr uint8_t FEEDBACK_SIZE = 15;

// --------------------------------------------------
// Motor data
// --------------------------------------------------

struct MotorCommand
{
    int16_t target_position = 0;
    int16_t target_velocity = 0;
    int16_t output = 0;
};

struct MotorFeedback
{
    int16_t position = 0;
    int16_t velocity = 0;
    int16_t current = 0;

    int64_t timestamp_us = 0;
    bool valid = false;
};

// --------------------------------------------------
// ST3215 Protocol
// --------------------------------------------------

enum class ST3215Command : uint8_t
{
    PING       = 0x01,
    READ       = 0x02,
    WRITE      = 0x03,
    REG_WRITE  = 0x04,
    ACTION     = 0x05,
    SYNC_READ  = 0x82,
    SYNC_WRITE = 0x83
};

struct ST3215Request
{
    ST3215Command command = ST3215Command::PING;

    uint8_t id = 0;
    uint8_t address = 0;
    uint8_t data_size = 0;

    const uint8_t* data = nullptr;
    size_t data_len = 0;

    const uint8_t* servo_ids = nullptr;
    size_t servo_count = 0;
};

struct ST3215Response
{
    uint8_t id = 0;
    uint8_t error = 0;

    static constexpr size_t MAX_DATA_SIZE = 122;

    uint8_t params[MAX_DATA_SIZE]{};
    size_t params_len = 0;
};

// --------------------------------------------------
// ST3215 Class
// --------------------------------------------------

class ST3215
{
public:
    static constexpr size_t MOTOR_COUNT = 6;
    static constexpr size_t MAX_PACKET_SIZE = 128;

    static constexpr uint8_t BROADCAST_ID = 0xFE;

public:
    ST3215() = default;
    ~ST3215() = default;

    // Initialization
    esp_err_t Initialize(uint8_t id);

    uint8_t GetID() const;

    // Command
    void SetCommand(const MotorCommand& cmd);
    MotorCommand GetCommand();

    // Feedback
    void SetFeedback(const MotorFeedback& fb);
    MotorFeedback GetFeedback();

    // Protocol serializer
    static esp_err_t Serialize(
        const ST3215Request& request,
        uint8_t* tx_buffer,
        size_t tx_capacity,
        size_t& tx_len
    );


    static esp_err_t ParseResponse(
        const uint8_t* rx_buffer,
        size_t rx_len,
        uint8_t expected_id,
        size_t expected_params_len,
        ST3215Response& response
    );

    static esp_err_t DecodeFeedback(
    const ST3215Response& response,
    int64_t timestamp_us,
    MotorFeedback& feedback
    );

private:
    static esp_err_t BuildPacket(
        uint8_t id,
        ST3215Command command,
        const uint8_t* params,
        size_t params_len,
        uint8_t* tx_buffer,
        size_t tx_capacity,
        size_t& tx_len
    );

    static uint8_t CalculateChecksum(
        const uint8_t* data,
        size_t len
    );

private:
    uint8_t id_ = 0;
    bool initialized_ = false;

    MotorCommand command_{};
    MotorFeedback feedback_{};

    portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
};
