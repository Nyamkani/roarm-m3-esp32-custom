
idf_component_register(
    SRC_DIRS "."
    INCLUDE_DIRS "."

    PRIV_REQUIRES
        esp_driver_gpio
        esp_driver_uart
        esp_driver_gptimer
        esp_driver_i2c
        esp_driver_spi
        esp_driver_ledc
        esp_driver_mcpwm
        esp_driver_rmt
        esp_adc
        esp_timer
        freertos
)
