#include "esp_log.h"
#include "../../../ina219/ina219.h"

static const char *TAG = "test_main";

void app_main()
{
    ESP_LOGI(TAG, "INA219 Test");
    if(ESP_OK != ina219_init()) {
        ESP_LOGE(TAG, "INA219 Init Error!");
    }
}
