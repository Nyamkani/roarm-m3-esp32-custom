#include "gpio_config.h"
#include "esp_log.h"

static const char* TAG = "app_main";
static board_handles_t board{};

extern "C" void app_main(void)
{
    if (BoardInitAll(&board) != ESP_OK)
    {
        ESP_LOGE(TAG, "Board initialization failed");
        return;
    }

    ESP_LOGI(TAG, "Board initialization successful");

    // Application initialization (Next step)
}