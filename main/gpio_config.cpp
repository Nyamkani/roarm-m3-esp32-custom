#include "gpio_config.h"
#include "pins.h"

#include "esp_check.h"
#include "esp_log.h"

static const char* TAG = "board_init";

esp_err_t BoardInitUART(board_handles_t* handles)
{
    ESP_RETURN_ON_FALSE(
        handles != nullptr,
        ESP_ERR_INVALID_ARG,
        TAG,
        "Invalid board handle"
    );

    uart_config_t cfg{};
    cfg.baud_rate = SERVO_UART_BAUDRATE;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;

    QueueHandle_t event_queue = nullptr;

    ESP_RETURN_ON_ERROR(
        uart_driver_install(
            SERVO_UART_NUM,
            SERVO_UART_RX_BUFFER_SIZE,
            SERVO_UART_TX_BUFFER_SIZE,
            SERVO_UART_EVENT_QUEUE_SIZE,
            &event_queue,
            0
        ),
        TAG,
        "Servo UART driver install failed"
    );

    esp_err_t ret = uart_param_config(SERVO_UART_NUM, &cfg);

    if (ret == ESP_OK)
    {
        ret = uart_set_pin(
            SERVO_UART_NUM,
            SERVO_UART_TX,
            SERVO_UART_RX,
            UART_PIN_NO_CHANGE,
            UART_PIN_NO_CHANGE
        );
    }

    if (ret != ESP_OK)
    {
        uart_driver_delete(SERVO_UART_NUM);
        return ret;
    }

    handles->servo_uart = SERVO_UART_NUM;
    handles->servo_uart_queue = event_queue;

    ESP_LOGI(TAG, "Servo UART initialized");

    return ESP_OK;
}

esp_err_t BoardInitAll(board_handles_t* handles)
{
    ESP_RETURN_ON_ERROR(
        BoardInitUART(handles),
        TAG,
        "UART initialization failed"
    );

    ESP_LOGI(TAG, "Board initialization complete");

    return ESP_OK;
}