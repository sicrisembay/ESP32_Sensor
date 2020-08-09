#include "sdkconfig.h"
#if defined(CONFIG_SENSOR_ICM20600)
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "icm20600.h"
#include "../common/i2c_interface.h"
#include "driver/gpio.h"

/* Command Definition */
typedef struct {
    int32_t rawData;
} icm20600_record_t;

static const char *TAG = "icm20600";
static DRAM_ATTR TaskHandle_t icm20600_task_handle = NULL;
static bool bInit = false;
static icm20600_record_t icm20600_record;

static void IRAM_ATTR _drdy_isr_handler(void * arg)
{
    BaseType_t xHigherPriorityTaskWoken;
 }

static void _icm20600_task(void *pArg)
{
    esp_err_t ret;
    uint32_t notifyFlag;

    /* Install isr service for DRDY */
    ret = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if(ESP_ERR_INVALID_STATE == ret) {
        /* Already installed by someone else */
    } else {
        ESP_ERROR_CHECK(ret);
    }
    ESP_ERROR_CHECK(gpio_set_intr_type(CONFIG_ICM20600_DRDY_IO, GPIO_INTR_NEGEDGE));
    ESP_ERROR_CHECK(gpio_isr_handler_add(CONFIG_ICM20600_DRDY_IO, _drdy_isr_handler,
                    (void*)CONFIG_ICM20600_DRDY_IO));

    /* Initialize ICM20600 registers */
    /// TODO

    bInit = true;
    ESP_LOGI(TAG, "icm20600 task running..\n");

    while(1) {
        if(pdPASS == xTaskNotifyWait(0UL, ULONG_MAX, &notifyFlag, portMAX_DELAY)) {
        }
    }
    /* Should not reach here */
    vTaskDelete(NULL);
}

esp_err_t icm20600_init(void)
{
    esp_err_t retval = ESP_OK;
    gpio_config_t io_conf;

    if(bInit == true) {
        /* Already initialized */
        return ESP_OK;
    }

    /* ICM20600 Data Ready */
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.pin_bit_mask = (1ULL << CONFIG_ICM20600_DRDY_IO);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE; /* External pull-up should be provided */
    retval = gpio_config(&io_conf);
    if(ESP_OK != retval) {
        ESP_LOGE(TAG, "Failed to init DRDY");
        return(retval);
    }
    /* I2C */
    retval = i2c_interface_init();
    if(retval != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init I2C%d", CONFIG_ICM20600_I2C_PORT_NUM);
        return(retval);
    }

#if defined(CONFIG_ICM20600_PINNED_TO_CORE_0)
    if(pdPASS != xTaskCreatePinnedToCore(
                _icm20600_task,                /* the task function */
                "icm20600",                    /* the name of the task */
                CONFIG_ICM20600_STACK_SIZE,    /* stack size */
                NULL,                           /* the 'pvParameters' parameter */
                CONFIG_ICM20600_TASK_PRIORITY, /* FreeRTOS priority */
                &icm20600_task_handle,         /* task handle */
                PRO_CPU_NUM)) {
#elif defined(CONFIG_ICM20600_PINNED_TO_CORE_1)
    if(pdPASS != xTaskCreatePinnedToCore(
                _icm20600_task,                /* the task function */
                "icm20600",                    /* the name of the task */
                CONFIG_ICM20600_STACK_SIZE,    /* stack size */
                NULL,                           /* the 'pvParameters' parameter */
                CONFIG_ICM20600_TASK_PRIORITY, /* FreeRTOS priority */
                &icm20600_task_handle,         /* task handle */
                APP_CPU_NUM)) {
#else
    if(pdPASS != xTaskCreate(
                _icm20600_task,                /* the task function */
                "icm20600",                    /* the name of the task */
                CONFIG_ICM20600_STACK_SIZE,    /* stack size */
                NULL,                           /* the 'pvParameters' parameter */
                CONFIG_ICM20600_TASK_PRIORITY, /* FreeRTOS priority */
                &icm20600_task_handle)) {
#endif
        ESP_LOGE(TAG, "Task creation failed!");
        return ESP_FAIL;
    }

    return (retval);
}

bool icm20600_isInitialized(void)
{
    return (bInit);
}

#endif /* CONFIG_SENSOR_ICM20600 */
