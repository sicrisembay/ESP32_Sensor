#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "../../../icm20600/icm20600.h"

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
    ESP_LOGI(TAG, "*** ICM20600 Test ***");
    ESP_LOGI(TAG, "...initializing ICM20600");
    if(ESP_OK != icm20600_init()) {
        ESP_LOGE(TAG, "ICM20600 Init Error!");
        return;
    }
    ESP_LOGI(TAG, "ICM20600 initialized.");

    while(!icm20600_isInitialized()) {
        vTaskDelay(1);
    }
}
