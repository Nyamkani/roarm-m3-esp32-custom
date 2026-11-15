
#pragma once

#include "driver/gpio.h"
#include "driver/uart.h"

// RoArm-M3
// Waveshare General Driver for Robots
// MCU: ESP32-WROOM-32

// --------------------------------------------------
// ST3215 / ST3235 Serial Bus Servo
// --------------------------------------------------
#define SERVO_UART_NUM       UART_NUM_1
#define SERVO_UART_RX        GPIO_NUM_18
#define SERVO_UART_TX        GPIO_NUM_19
#define SERVO_UART_BAUDRATE  1000000

// UART Buffer
#define SERVO_UART_RX_BUFFER_SIZE  1024
#define SERVO_UART_TX_BUFFER_SIZE  512
#define SERVO_UART_EVENT_QUEUE_SIZE 10

// --------------------------------------------------
// USB / Host Communication (UART0)
// --------------------------------------------------
#define HOST_UART_NUM        UART_NUM_0
#define HOST_UART_RX         GPIO_NUM_3
#define HOST_UART_TX         GPIO_NUM_1
#define HOST_UART_BAUDRATE   115200

// --------------------------------------------------
// I2C Peripheral
// INA219 / IMU / OLED
// --------------------------------------------------
#define I2C_SDA_PIN          GPIO_NUM_32
#define I2C_SCL_PIN          GPIO_NUM_33

// --------------------------------------------------
// External GPIO
// --------------------------------------------------
#define EXT_IO4_PIN          GPIO_NUM_4
#define EXT_IO5_PIN          GPIO_NUM_5



