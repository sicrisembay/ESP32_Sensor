#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "../../../ina219/ina219.h"

static const char *TAG = "test_main";
static char ina219Str[128];

void app_main()
{
    esp_err_t retval = ESP_OK;
    ESP_LOGI(TAG, "*** INA219 Test ***");
    ESP_LOGI(TAG, "...initializing INA219");
    if(ESP_OK != ina219_init()) {
        ESP_LOGE(TAG, "INA219 Init Error!");
        return;
    }
    ESP_LOGI(TAG, "INA219 initialized.");

    while(!ina219_isInitialized()) {
        vTaskDelay(10);
    }

    while(1) {
        vTaskDelay(10);
#if(CONFIG_INA219_DEVICE_COUNT >= 1)
        snprintf(ina219Str, sizeof(ina219Str) - 1,
                "Dev1 (%0.2fV, %0.3fA, %0.3fW)",
                ina219_get_bus_voltage(0, &retval),
                ina219_get_current(0, &retval),
                ina219_get_power(0, &retval));
#endif
#if(CONFIG_INA219_DEVICE_COUNT >= 2)
        snprintf(ina219Str, sizeof(ina219Str) - 1,
                "%s, Dev2 (%0.2fV, %0.3fA, %0.3fW)",
                ina219Str,
                ina219_get_bus_voltage(1, &retval),
                ina219_get_current(1, &retval),
                ina219_get_power(1, &retval));
#endif
#if(CONFIG_INA219_DEVICE_COUNT >= 3)
        snprintf(ina219Str, sizeof(ina219Str) - 1,
                "%s, Dev3 (%0.2fV, %0.3fA, %0.3fW)",
                ina219Str,
                ina219_get_bus_voltage(2, &retval),
                ina219_get_current(2, &retval),
                ina219_get_power(2, &retval));
#endif

        ESP_LOGI(TAG, "%s", ina219Str);
    }
}
