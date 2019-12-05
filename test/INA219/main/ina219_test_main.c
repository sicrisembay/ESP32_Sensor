#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "../../../ina219/ina219.h"

static const char *TAG = "test_main";

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
        ESP_LOGI(TAG, "Bus: %0.2fV, Shunt: %0.6fV, Current: %0.3fA, Power: %0.3fW",
                ina219_get_bus_voltage(0, &retval),
                ina219_get_shunt_voltage(0, &retval),
                ina219_get_current(0, &retval),
                ina219_get_power(0, &retval));
    }
}
