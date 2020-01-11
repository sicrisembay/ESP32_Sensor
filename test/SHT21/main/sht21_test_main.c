#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "../../../sht21/sht21.h"

static const char *TAG = "test_main";
static char sht21Str[128];

void app_main()
{
    esp_err_t retval = ESP_OK;
    ESP_LOGI(TAG, "*** SHT21 Test ***");
    ESP_LOGI(TAG, "...initializing SHT21");
    if(ESP_OK != sht21_init()) {
        ESP_LOGE(TAG, "SHT21 Init Error!");
        return;
    }
    ESP_LOGI(TAG, "SHT21 initialized.");

    while(!sht21_isInitialized()) {
        vTaskDelay(10);
    }
}
