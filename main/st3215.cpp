
#include "st3215.h"

#include <cstring>

namespace
{
constexpr size_t MAX_PARAMS =
    ST3215::MAX_PACKET_SIZE - 6;

bool ValidServoID(uint8_t id)
{
    return id <= 0xFD;
}

bool ValidServoList(
    const uint8_t* ids,
    size_t count)
{
    if (ids == nullptr ||
        count == 0 ||
        count > ST3215::MOTOR_COUNT)
    {
        return false;
    }

    for (size_t i = 0; i < count; ++i)
    {
        if (!ValidServoID(ids[i]))
            return false;

        for (size_t j = 0; j < i; ++j)
        {
            if (ids[i] == ids[j])
                return false;
        }
    }

    return true;
}
uint16_t ReadLE16(const uint8_t* data)
{
    return static_cast<uint16_t>(data[0]) |
        (static_cast<uint16_t>(data[1]) << 8);
}

int16_t DecodeSignMagnitude(
    uint16_t raw,
    uint16_t sign_bit)
{
    const int16_t magnitude =
        static_cast<int16_t>(raw & (sign_bit - 1));

    return (raw & sign_bit) ? -magnitude : magnitude;
}
}

esp_err_t ST3215::DecodeFeedback(
    const ST3215Response& response,
    int64_t timestamp_us,
    MotorFeedback& feedback)
{
    feedback = {};

    if (response.error != 0 ||
        response.params_len != 15 ||
        timestamp_us <= 0)
    {
        return ESP_ERR_INVALID_RESPONSE;
    }

    const uint8_t* data = response.params;

    // Register 56: Present Position
    const uint16_t position = ReadLE16(&data[0]);

    // Register 58: Present Speed
    const uint16_t velocity = ReadLE16(&data[2]);

    // Register 69: Present Current
    const uint16_t current = ReadLE16(&data[13]);

    feedback.position =
        DecodeSignMagnitude(position, 0x8000);

    feedback.velocity =
        DecodeSignMagnitude(velocity, 0x8000);

    feedback.current =
        DecodeSignMagnitude(current, 0x8000);

    feedback.timestamp_us = timestamp_us;
    feedback.valid = true;

    return ESP_OK;
}
// --------------------------------------------------
// Initialization
// --------------------------------------------------

esp_err_t ST3215::Initialize(uint8_t id)
{
    if (id > 0xFD)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (initialized_)
    {
        return ESP_ERR_INVALID_STATE;
    }

    id_ = id;

    command_ = {};
    feedback_ = {};

    initialized_ = true;

    return ESP_OK;
}

uint8_t ST3215::GetID() const
{
    return id_;
}

// --------------------------------------------------
// Command
// --------------------------------------------------

void ST3215::SetCommand(const MotorCommand& cmd)
{
    portENTER_CRITICAL(&mux_);

    command_ = cmd;

    portEXIT_CRITICAL(&mux_);
}

MotorCommand ST3215::GetCommand()
{
    portENTER_CRITICAL(&mux_);

    MotorCommand result = command_;

    portEXIT_CRITICAL(&mux_);

    return result;
}

// --------------------------------------------------
// Feedback
// --------------------------------------------------

void ST3215::SetFeedback(const MotorFeedback& fb)
{
    portENTER_CRITICAL(&mux_);

    feedback_ = fb;

    portEXIT_CRITICAL(&mux_);
}

MotorFeedback ST3215::GetFeedback()
{
    portENTER_CRITICAL(&mux_);

    MotorFeedback result = feedback_;

    portEXIT_CRITICAL(&mux_);

    return result;
}


// --------------------------------------------------
// UART Parse Response
// --------------------------------------------------

esp_err_t ST3215::ParseResponse(
    const uint8_t* rx_buffer,
    size_t rx_len,
    uint8_t expected_id,
    size_t expected_params_len,
    ST3215Response& response)
{
    response = {};

    if (rx_buffer == nullptr ||
        expected_id > 0xFD)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // Header(2) + ID + Length + Error + Checksum
    constexpr size_t MIN_PACKET_SIZE = 6;

    if (rx_len < MIN_PACKET_SIZE ||
        rx_len > MAX_PACKET_SIZE)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    // Header
    if (rx_buffer[0] != 0xFF ||
        rx_buffer[1] != 0xFF)
    {
        return ESP_ERR_INVALID_RESPONSE;
    }

    // Servo ID
    if (rx_buffer[2] != expected_id)
    {
        return ESP_ERR_INVALID_RESPONSE;
    }

    const size_t length = rx_buffer[3];

    if (length < 2 ||
        rx_len != length + 4)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    const size_t params_len = length - 2;

    if (params_len != expected_params_len ||
        params_len > ST3215Response::MAX_DATA_SIZE)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    // ID + LENGTH + ERROR + PARAMETERS
    const uint8_t checksum = CalculateChecksum(
        &rx_buffer[2],
        rx_len - 3
    );

    if (checksum != rx_buffer[rx_len - 1])
    {
        return ESP_ERR_INVALID_CRC;
    }

    response.id = rx_buffer[2];
    response.error = rx_buffer[4];
    response.params_len = params_len;

    if (params_len > 0)
    {
        std::memcpy(
            response.params,
            &rx_buffer[5],
            params_len
        );
    }

    return ESP_OK;
}


// --------------------------------------------------
// Checksum
// --------------------------------------------------
uint8_t ST3215::CalculateChecksum(
    const uint8_t* data,
    size_t len)
{
    uint8_t sum = 0;

    for (size_t i = 0; i < len; ++i)
    {
        sum = static_cast<uint8_t>(
            sum + data[i]);
    }

    return static_cast<uint8_t>(~sum);
}

// --------------------------------------------------
// Common packet builder
// --------------------------------------------------
esp_err_t ST3215::BuildPacket(
    uint8_t id,
    ST3215Command command,
    const uint8_t* params,
    size_t params_len,
    uint8_t* tx_buffer,
    size_t tx_capacity,
    size_t& tx_len)
{
    tx_len = 0;

    if (tx_buffer == nullptr ||
        (params_len > 0 && params == nullptr))
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (params_len > MAX_PARAMS ||
        params_len > 253)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    const size_t packet_len = params_len + 6;

    if (packet_len > tx_capacity)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    tx_buffer[0] = 0xFF;
    tx_buffer[1] = 0xFF;
    tx_buffer[2] = id;
    tx_buffer[3] =
        static_cast<uint8_t>(params_len + 2);
    tx_buffer[4] =
        static_cast<uint8_t>(command);

    if (params_len > 0)
    {
        std::memcpy(
            &tx_buffer[5],
            params,
            params_len
        );
    }

    // ID + LENGTH + INSTRUCTION + PARAMETERS
    tx_buffer[packet_len - 1] =
        CalculateChecksum(
            &tx_buffer[2],
            params_len + 3
        );

    tx_len = packet_len;

    return ESP_OK;
}

// --------------------------------------------------
// Command serializer
// --------------------------------------------------
esp_err_t ST3215::Serialize(
    const ST3215Request& request,
    uint8_t* tx_buffer,
    size_t tx_capacity,
    size_t& tx_len)
{
    tx_len = 0;

    if (tx_buffer == nullptr)
        return ESP_ERR_INVALID_ARG;

    uint8_t params[MAX_PARAMS]{};
    size_t params_len = 0;

    uint8_t packet_id = request.id;

    switch (request.command)
    {
        case ST3215Command::PING:
        case ST3215Command::ACTION:
        {
            if (!ValidServoID(request.id) &&
                request.id != BROADCAST_ID)
            {
                return ESP_ERR_INVALID_ARG;
            }

            // No parameters
            break;
        }

        case ST3215Command::READ:
        {
            if (!ValidServoID(request.id) ||
                request.data_size == 0)
            {
                return ESP_ERR_INVALID_ARG;
            }

            params[0] = request.address;
            params[1] = request.data_size;
            params_len = 2;
            break;
        }

        case ST3215Command::WRITE:
        case ST3215Command::REG_WRITE:
        {
            if ((!ValidServoID(request.id) &&
                 request.id != BROADCAST_ID) ||
                request.data == nullptr ||
                request.data_len == 0)
            {
                return ESP_ERR_INVALID_ARG;
            }

            if (request.data_len > MAX_PARAMS - 1)
                return ESP_ERR_INVALID_SIZE;

            params[0] = request.address;

            std::memcpy(
                &params[1],
                request.data,
                request.data_len
            );

            params_len = request.data_len + 1;
            break;
        }

        case ST3215Command::SYNC_READ:
        {
            if (!ValidServoList(
                    request.servo_ids,
                    request.servo_count) ||
                request.data_size == 0)
            {
                return ESP_ERR_INVALID_ARG;
            }

            packet_id = BROADCAST_ID;

            params[0] = request.address;
            params[1] = request.data_size;

            std::memcpy(
                &params[2],
                request.servo_ids,
                request.servo_count
            );

            params_len = request.servo_count + 2;
            break;
        }

        case ST3215Command::SYNC_WRITE:
        {
            if (!ValidServoList(
                    request.servo_ids,
                    request.servo_count) ||
                request.data_size == 0 ||
                request.data == nullptr)
            {
                return ESP_ERR_INVALID_ARG;
            }

            const size_t block_size =
                static_cast<size_t>(request.data_size);

            const size_t expected_len =
                request.servo_count * block_size;

            const size_t required_params =
                2 + request.servo_count *
                    (block_size + 1);

            if (request.data_len != expected_len ||
                required_params > MAX_PARAMS)
            {
                return ESP_ERR_INVALID_SIZE;
            }

            packet_id = BROADCAST_ID;

            params[0] = request.address;
            params[1] = request.data_size;
            params_len = 2;

            for (size_t i = 0;
                 i < request.servo_count;
                 ++i)
            {
                params[params_len++] =
                    request.servo_ids[i];

                std::memcpy(
                    &params[params_len],
                    &request.data[i * block_size],
                    block_size
                );

                params_len += block_size;
            }

            break;
        }

        default:
            return ESP_ERR_INVALID_ARG;
    }

    return BuildPacket(
        packet_id,
        request.command,
        params,
        params_len,
        tx_buffer,
        tx_capacity,
        tx_len
    );
}

esp_err_t ST3215::EncodePWM(
    int16_t output,
    uint8_t data[2])
{
    if (data == nullptr ||
        output < -1000 ||
        output > 1000)
    {
        return ESP_ERR_INVALID_ARG;
    }

    const uint16_t magnitude =
        static_cast<uint16_t>(
            output < 0 ? -static_cast<int32_t>(output) : output);

    const uint16_t raw =
        static_cast<uint16_t>(
            magnitude | (output < 0 ? 0x0400u : 0u));

    data[0] = static_cast<uint8_t>(raw & 0xFFu);
    data[1] = static_cast<uint8_t>((raw >> 8) & 0xFFu);

    return ESP_OK;
}