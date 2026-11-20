idf_component_register(
    SRCS
        "main.cpp"
        "gpio_config.cpp"
        "motor_interface.cpp"
        "st3215.cpp"
        "st3215_parameter_fsm.cpp"
        "joint_mapper.cpp"
        "arm_kinematics.cpp"
        "arm_command_interface.cpp"

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