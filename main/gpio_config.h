#pragma once

#include "esp_err.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef struct
{
    uart_port_t servo_uart;
    QueueHandle_t servo_uart_queue;

} board_handles_t;

// Hardware initialization
esp_err_t BoardInitUART(board_handles_t* handles);
esp_err_t BoardInitAll(board_handles_t* handles);