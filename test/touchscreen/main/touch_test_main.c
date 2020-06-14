#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "../../../touchscreen/include/touchscreen.h"

static const char *TAG = "test_main";

void app_main()
{
    esp_err_t retval = ESP_OK;
    int16_t x;
    int16_t y;
    ESP_LOGI(TAG, "*** Touchscreen Test ***");
    ESP_LOGI(TAG, "...initializing");
    if(ESP_OK != touchscreen_init()) {
        ESP_LOGE(TAG, "Touchscreen Init Error!");
        return;
    }
    ESP_LOGI(TAG, "Touchscreen initialized.");

    while(1) {
        vTaskDelay(10);
        touchscreen_getCoord(0, &x, &y);
        ESP_LOGI(TAG, "coordinate: %d, %d", x, y);
    }
}
