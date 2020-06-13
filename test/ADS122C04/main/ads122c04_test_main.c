#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "../../../ads122c04/ads122c04.h"

static const char *TAG = "test_main";

void conversion_cb(int32_t value)
{
    static int32_t offset = 0;
    if(offset == 0) {
        offset = value;
    }
    ESP_LOGI(TAG, "raw: %d\n", value - offset);
}

void app_main()
{
    esp_err_t retval = ESP_OK;
    int32_t offset = 0;
    ESP_LOGI(TAG, "*** ADS122C04 Test ***");
    ESP_LOGI(TAG, "...initializing ADS122C04");
    if(ESP_OK != ads122c04_init(conversion_cb)) {
        ESP_LOGE(TAG, "ADS122C04 Init Error!");
        return;
    }
    ESP_LOGI(TAG, "ADS122C04 initialized.");

    while(!ads122c04_isInitialized()) {
        vTaskDelay(1);
    }
}
