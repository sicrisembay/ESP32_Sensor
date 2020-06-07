#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sht21.h"
#include "../common/i2c_interface.h"

#define ENABLE_DEBUG                    (1)

#define SHT_I2C_DEV_ADDR                (0x40)

#define CMD_TRIG_T_MEAS_HOLD            (0xE3)
#define CMD_TRIG_RH_MEAS_HOLD           (0xE5)
#define CMD_TRIG_T_MEAS_NOHOLD          (0xF3)
#define CMD_TRIG_RH_MEAS_NOHOLD         (0xF5)
#define CMD_WRITE_USER_REG              (0xE6)
#define CMD_READ_USER_REG               (0xE7)
#define CMD_SOFT_RESET                  (0xFE)

typedef enum {
    SHT21_TEMPERATURE = 0,
    SHT21_RH,
    
    N_SHT21_DATA_TYPE
} sht21_data_type_t;

typedef struct {
    float fRelHumidity;
    float fTemperature;
    union {
        uint8_t data;
        struct {
            uint8_t measurement_resolution_b0 : 1;
            uint8_t disable_otp_reload : 1;
            uint8_t enable_oc_heater : 1;
            uint8_t reserved : 3;
            uint8_t status_battery : 1;
            uint8_t measurement_resolution_b1 : 1;
        } field;
    } user_register;
} sht21_dev_t;

static const char *TAG = "sht21";
static DRAM_ATTR TaskHandle_t sht21_task_handle = NULL;
static bool bInit = false;
static sht21_dev_t sht21_dev;

static esp_err_t _sht_readReg(uint8_t * pVal)
{
    esp_err_t retval = ESP_OK;
    uint8_t cmd = CMD_READ_USER_REG;
    
    if(pVal == NULL) {
        ESP_LOGE(TAG, "Invalid Arg!");
        return ESP_ERR_INVALID_ARG;
    }
    
    retval = i2c_interface_read(CONFIG_SHT21_I2C_PORT_NUM, SHT_I2C_DEV_ADDR,
                &cmd, 1, pVal, 1);

    return(retval);
}

static esp_err_t _sht_readData(sht21_data_type_t type, uint16_t * pVal)
{
    esp_err_t retval = ESP_OK;
    uint16_t val = 0;
    uint8_t cmd;
    uint8_t data[3];
    
    if((type >= N_SHT21_DATA_TYPE) || (pVal == NULL)) {
        ESP_LOGE(TAG, "Invalid Arg!");
        return ESP_ERR_INVALID_ARG;
    }
    
    *pVal = 0;
    
    switch(type) {
        case SHT21_TEMPERATURE:
            cmd = CMD_TRIG_T_MEAS_NOHOLD;
            break;
        case SHT21_RH:
            cmd = CMD_TRIG_RH_MEAS_NOHOLD;
            break;
        default:
            break;
    }
    retval = i2c_interface_write(CONFIG_SHT21_I2C_PORT_NUM, SHT_I2C_DEV_ADDR,
                              &cmd, 1, NULL, 0);
    if(ESP_OK != retval) {
        return(retval);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    retval = i2c_interface_read(CONFIG_SHT21_I2C_PORT_NUM, SHT_I2C_DEV_ADDR,
                NULL, 0, data, 3);
    if(ESP_OK != retval) {
        return(retval);
    }
    
    /// TODO: add crc check
    
    val = (((uint16_t)(data[0])) << 8) + (uint16_t)data[1];
    
    if(val & 0x0002) {
        /* RH data */
        if(type == SHT21_TEMPERATURE) {
            ESP_LOGE(TAG, "Invalid data for type temperature!");
            return ESP_FAIL;
        }
    } else {
        /* Temperature data */
        if(type == SHT21_RH) {
            ESP_LOGE(TAG, "Invalid data for type RH!");
            return ESP_FAIL;
        }
    }

    *pVal = val >> 2;
    return ESP_OK;
}

static void _sht21_task(void *pArg)
{
    TickType_t xLastWakeTime;
    uint16_t rawData;
    xLastWakeTime = xTaskGetTickCount();
    esp_err_t retval = ESP_OK;
    TickType_t pollTick;
    bInit = true;

    if(CONFIG_SHT21_POLL_INTERVAL > 0) {
        pollTick = CONFIG_SHT21_POLL_INTERVAL * 1000 / portTICK_PERIOD_MS;
    } else {
        pollTick = 1000 / portTICK_PERIOD_MS;
    }
    
    while(1) {
        vTaskDelayUntil(&xLastWakeTime, pollTick);
        /* Read Temperature */
        retval = _sht_readData(SHT21_TEMPERATURE, &rawData);
        if(ESP_OK == retval) {
            sht21_dev.fTemperature = -46.85f + (175.72 * (float)rawData / 16384);
        } else {
            ESP_LOGE(TAG, "_sht_readData() failed!");
        }
        
        /* Read RH */
        retval = _sht_readData(SHT21_RH, &rawData);
        if(ESP_OK == retval) {
            sht21_dev.fRelHumidity = -6.0f + (125.0 * (float)rawData / 16384);
        } else {
            ESP_LOGE(TAG, "_sht_readData() failed!");
        }
#if (ENABLE_DEBUG == 1)
        ESP_LOGI(TAG, "%0.2f %0.2f", sht21_dev.fTemperature, sht21_dev.fRelHumidity);
#endif
    }
    /* Should not reach here */
    vTaskDelete(NULL);
}

esp_err_t sht21_init(void)
{
    esp_err_t retval = ESP_OK;

    if(bInit == true) {
        /* Already initialized */
        return ESP_OK;
    }
    retval = i2c_interface_init();
    if(retval != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init I2C%d", CONFIG_SHT21_I2C_PORT_NUM);
        return(retval);
    }

    sht21_dev.fRelHumidity = 0.0f;
    sht21_dev.fTemperature = 0.0f;
    /* Read User Register */
    retval = _sht_readReg(&(sht21_dev.user_register.data));
    if(ESP_OK != retval) {
        ESP_LOGE(TAG, "Error reading user register!");
        return(retval);
    }

#if defined(CONFIG_SHT21_PINNED_TO_CORE_0)
    if(pdPASS != xTaskCreatePinnedToCore(
                _sht21_task,                /* the task function */
                "sht21",                    /* the name of the task */
                CONFIG_SHT21_STACK_SIZE,    /* stack size */
                NULL,                       /* the 'pvParameters' parameter */
                CONFIG_SHT21_TASK_PRIORITY, /* FreeRTOS priority */
                &sht21_task_handle,         /* task handle */
                PRO_CPU_NUM)) {
#elif defined(CONFIG_SHT21_PINNED_TO_CORE_1)
    if(pdPASS != xTaskCreatePinnedToCore(
                _sht21_task,                /* the task function */
                "sht21",                    /* the name of the task */
                CONFIG_SHT21_STACK_SIZE,    /* stack size */
                NULL,                       /* the 'pvParameters' parameter */
                CONFIG_SHT21_TASK_PRIORITY, /* FreeRTOS priority */
                &sht21_task_handle,         /* task handle */
                APP_CPU_NUM)) {
#else
    if(pdPASS != xTaskCreate(
                _sht21_task,                /* the task function */
                "sht21",                    /* the name of the task */
                CONFIG_SHT21_STACK_SIZE,    /* stack size */
                NULL,                       /* the 'pvParameters' parameter */
                CONFIG_SHT21_TASK_PRIORITY, /* FreeRTOS priority */
                &sht21_task_handle)) {
#endif
        ESP_LOGE(TAG, "Task creation failed!");
        return ESP_FAIL;
    }
    return ESP_OK;
}

bool sht21_isInitialized(void)
{
    return (bInit);
}