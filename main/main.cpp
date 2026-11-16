#include "gpio_config.h"
#include "motor_interface.h"
#include "esp_log.h"

static const char* TAG = "app_main";

static board_handles_t board{};
static MotorInterface motor_interface{};

extern "C" void app_main(void)
{
    if (BoardInitAll(&board) != ESP_OK)
    {
        ESP_LOGE(TAG, "Board initialization failed");
        return;
    }

    if (motor_interface.Initialize(board) != ESP_OK)
    {
        ESP_LOGE(TAG, "Motor interface initialization failed");
        return;
    }

    ESP_LOGI(TAG, "Motor interface started");
}