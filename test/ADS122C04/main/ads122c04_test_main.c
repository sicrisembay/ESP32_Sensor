#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "../../../ads122c04/ads122c04.h"

static const char *TAG = "test_main";
static char ads122c04Str[128];

void app_main()
{
    esp_err_t retval = ESP_OK;
    int32_t offset = 0;
    ESP_LOGI(TAG, "*** ADS122C04 Test ***");
    ESP_LOGI(TAG, "...initializing ADS122C04");
    if(ESP_OK != ads122c04_init()) {
        ESP_LOGE(TAG, "ADS122C04 Init Error!");
        return;
    }
    ESP_LOGI(TAG, "ADS122C04 initialized.");

    while(!ads122c04_isInitialized()) {
        vTaskDelay(10);
    }

    offset = ads122c04_getRaw();

    while(1) {
        vTaskDelay(5);
        ESP_LOGI(TAG, "raw: %d\n", ads122c04_getRaw() - offset);
    }
}
