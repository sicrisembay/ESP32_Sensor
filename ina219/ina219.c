#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ina219.h"
#include "../common/i2c_interface.h"

static const char *TAG = "ina219";
static DRAM_ATTR TaskHandle_t ina219_task_handle = NULL;
static bool bInit = false;
static bool updateDly = true;

#define REG_ADDR_CONFIGURATION      (0x00)
#define REG_ADDR_SHUNT_VOLTAGE      (0x01)
#define REG_ADDR_BUS_VOLTAGE        (0x02)
#define REG_ADDR_POWER              (0x03)
#define REG_ADDR_CURRENT            (0x04)
#define REG_ADDR_CALIBRATION        (0x05)

#if (CONFIG_INA219_DEVICE_COUNT >= 1)
#define CURRENT_LSB_DEV1            (float)(0.04096f / ((float)CONFIG_INA219_CAL_FS_DEV1 * (float)CONFIG_INA219_R_DEV1 * 0.001f))
#endif
#if (CONFIG_INA219_DEVICE_COUNT >= 2)
#define CURRENT_LSB_DEV2            (float)(0.04096f / ((float)CONFIG_INA219_CAL_FS_DEV2 * (float)CONFIG_INA219_R_DEV2 * 0.001f))
#endif
#if (CONFIG_INA219_DEVICE_COUNT >= 3)
#define CURRENT_LSB_DEV3            (float)(0.04096f / ((float)CONFIG_INA219_CAL_FS_DEV3 * (float)CONFIG_INA219_R_DEV3 * 0.001f))
#endif

typedef struct {
    union {
        uint16_t data;
        struct {
            uint16_t mode:3;
            uint16_t sadc:4;
            uint16_t badc:4;
            uint16_t pga:2;
            uint16_t brng:1;
            uint16_t reserved:1;
            uint16_t rst:1;
        } field;
    } configuration;
    uint16_t shuntVoltage;
    union {
        uint16_t data;
        struct {
            uint16_t ovf:1;
            uint16_t cnvr:1;
            uint16_t reserved:1;
            uint16_t bd:13;
        } field;
    } busVoltage;
    uint16_t power;
    uint16_t current;
    uint16_t calibration;
} ina219_dev_t;

typedef struct {
    float fShuntVoltage;
    float fBusVoltage;
    float fPower;
    float fCurrent;
} ina219_data_t;

typedef struct {
    uint32_t sync_config_pending    : 1;
} ina219_flag_t;

static ina219_dev_t ina219_dev[CONFIG_INA219_DEVICE_COUNT];
static ina219_data_t ina219_data[CONFIG_INA219_DEVICE_COUNT];
static ina219_flag_t ina219_flag[CONFIG_INA219_DEVICE_COUNT];

static uint8_t ina219_i2c_devAddr[CONFIG_INA219_DEVICE_COUNT] = {
        CONFIG_INA219_ADDR_DEV1,
#if(CONFIG_INA219_DEVICE_COUNT > 1)
        CONFIG_INA219_ADDR_DEV2,
#if(CONFIG_INA219_DEVICE_COUNT > 2)
        CONFIG_INA219_ADDR_DEV3,
#endif
#endif
};

static float ina219_current_lsb[CONFIG_INA219_DEVICE_COUNT] = {
        CURRENT_LSB_DEV1,
#if(CONFIG_INA219_DEVICE_COUNT > 1)
        CURRENT_LSB_DEV2,
#endif
#if(CONFIG_INA219_DEVICE_COUNT > 2)
        CURRENT_LSB_DEV3,
#endif
};

static ina219_dev_t const ina219_dev_default[CONFIG_INA219_DEVICE_COUNT] = {
        {
                .configuration.field.mode = CONFIG_INA219_MODE_DEV1,
                .configuration.field.sadc = CONFIG_INA219_SADC_DEV1,
                .configuration.field.badc = CONFIG_INA219_BADC_DEV1,
                .configuration.field.pga = CONFIG_INA219_PGA_DEV1,
                .configuration.field.brng = CONFIG_INA219_BUS_DEV1,
                .shuntVoltage = 0,
                .busVoltage.data = 0,
                .power = 0,
                .current = 0,
                .calibration = CONFIG_INA219_CAL_FS_DEV1
        },
#if(CONFIG_INA219_DEVICE_COUNT > 1)
        {
                .configuration.field.mode = CONFIG_INA219_MODE_DEV2,
                .configuration.field.sadc = CONFIG_INA219_SADC_DEV2,
                .configuration.field.badc = CONFIG_INA219_BADC_DEV2,
                .configuration.field.pga = CONFIG_INA219_PGA_DEV2,
                .configuration.field.brng = CONFIG_INA219_BUS_DEV2,
                .shuntVoltage = 0,
                .busVoltage.data = 0,
                .power = 0,
                .current = 0,
                .calibration = CONFIG_INA219_CAL_FS_DEV2
        },
#if(CONFIG_INA219_DEVICE_COUNT > 2)
        {
                .configuration.field.mode = CONFIG_INA219_MODE_DEV3,
                .configuration.field.sadc = CONFIG_INA219_SADC_DEV3,
                .configuration.field.badc = CONFIG_INA219_BADC_DEV3,
                .configuration.field.pga = CONFIG_INA219_PGA_DEV3,
                .configuration.field.brng = CONFIG_INA219_BUS_DEV3,
                .shuntVoltage = 0,
                .busVoltage.data = 0,
                .power = 0,
                .current = 0,
                .calibration = CONFIG_INA219_CAL_FS_DEV3
        }
#endif
#endif
};

static float const PGA_DIV[4] = {
        1.0f,
        2.0f,
        4.0f,
        8.0f
};

static uint32_t const CONVERSION_TIME[16] = {
        1,  // 84us --> 1ms (smallest value is kernel tick)
        1,  // 148us
        1,  // 276 us
        1,  // 532 us
        1,
        1,
        1,
        1,
        1,  // 532us
        1,  // 1.06ms
        2,  // 2.13ms
        4,  // 4.26ms
        9,  // 8.51ms
        17, // 17.02ms
        34, // 34.05ms
        68, // 68.1ms
};

static esp_err_t _ina219_writeReg(uint8_t id, uint8_t reg, uint16_t val)
{
    uint8_t invertVal[2];
    esp_err_t retval = ESP_OK;
    if(id >= CONFIG_INA219_DEVICE_COUNT) {
        ESP_LOGE(TAG, "Invalid id!");
        return ESP_ERR_INVALID_ARG;
    }

    invertVal[0] = (uint8_t)((val >> 8) & 0x00FF);
    invertVal[1] = (uint8_t)(val & 0x00FF);

    retval = i2c_interface_write(CONFIG_INA219_I2C_PORT_NUM,
                                 ina219_i2c_devAddr[id],
                                 &reg, 1, invertVal, 2);
    return retval;
}

static esp_err_t _ina219_readReg(uint8_t id, uint8_t reg, uint16_t *pVal)
{
    uint8_t regVal[2];
    esp_err_t retval = ESP_OK;
    if((id >= CONFIG_INA219_DEVICE_COUNT) || (pVal == NULL)) {
        ESP_LOGE(TAG, "Invalid Arg!");
        return ESP_ERR_INVALID_ARG;
    }

    retval = i2c_interface_read(CONFIG_INA219_I2C_PORT_NUM,
                        ina219_i2c_devAddr[id],
                        &reg, 1, regVal, 2);
    if(ESP_OK != retval) {
        *pVal = 0;
        return(retval);
    }

    *pVal = (uint16_t)((regVal[0] << 8) + regVal[1]);
    return ESP_OK;
}

static esp_err_t _ina219_min_conversion_time(uint32_t *pVal)
{
    uint32_t retval = CONVERSION_TIME[15];  /* largest conversion time */
    uint8_t idx = 0;

    if(NULL == pVal) {
        return ESP_ERR_INVALID_ARG;
    }
    if(bInit != true) {
        *pVal = 1U;
        return ESP_ERR_INVALID_STATE;
    }

    for(idx = 0; idx < CONFIG_INA219_DEVICE_COUNT; idx++) {
        if(retval > CONVERSION_TIME[ina219_dev[idx].configuration.field.sadc]) {
            retval = CONVERSION_TIME[ina219_dev[idx].configuration.field.sadc];
        }
        if(retval > CONVERSION_TIME[ina219_dev[idx].configuration.field.badc]) {
            retval = CONVERSION_TIME[ina219_dev[idx].configuration.field.badc];
        }
    }
    *pVal = retval;
    ESP_LOGI(TAG, "Task delay is set to %d", retval);
    return ESP_OK;
}
static void _ina219_task(void *pArg)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    esp_err_t retval = ESP_OK;
    uint32_t dly = 1;
    int16_t i16Temp;
    uint8_t idx;

    bInit = true;

    while(1) {
        if(updateDly) {
            retval = _ina219_min_conversion_time(&dly);
            if(retval == ESP_OK) {
                updateDly = false;
            } else {
                dly = 1;
            }
        }
        vTaskDelayUntil(&xLastWakeTime, dly);
        for(idx = 0; idx < CONFIG_INA219_DEVICE_COUNT; idx++) {
            if(ina219_flag[idx].sync_config_pending) {
                /* Sync Configuration state */
                retval = _ina219_readReg(idx, REG_ADDR_CONFIGURATION, &(ina219_dev[idx].configuration.data));
                if(retval == ESP_OK) {
                    /* Synced */
                    ina219_flag[idx].sync_config_pending = 0;
                }
            }
            /* Read Registers */
            retval = _ina219_readReg(idx, REG_ADDR_BUS_VOLTAGE, &(ina219_dev[idx].busVoltage.data));
            if(ESP_OK == retval) {
                if(ina219_dev[idx].busVoltage.field.cnvr == 0) {
                    /* There is no new data */
                    continue;
                }
                i16Temp = (int16_t)(ina219_dev[idx].busVoltage.field.bd);
                ina219_data[idx].fBusVoltage = (float)(i16Temp) * 0.004f; /* LSB is 4mV */
            } else {
                ESP_LOGE(TAG, "Dev%d: Error Reading Bus Voltage Register!", (idx+1));
            }

            retval = _ina219_readReg(idx, REG_ADDR_SHUNT_VOLTAGE, &(ina219_dev[idx].shuntVoltage));
            if(ESP_OK == retval) {
                i16Temp = (int16_t)(ina219_dev[idx].shuntVoltage);
                ina219_data[idx].fShuntVoltage = (float)(i16Temp) * 0.000010f; /* LSB is 10uV */
            } else {
                ESP_LOGE(TAG, "Dev%d: Error Reading Shunt Voltage Register!", (idx+1));
            }

            /* Note: Reading Power Register also clear Conversion Read */
            retval = _ina219_readReg(idx, REG_ADDR_POWER, &(ina219_dev[idx].power));
            if(ina219_dev[idx].busVoltage.field.ovf) {
                ESP_LOGE(TAG, "Math Overflow Detected!");
                continue;
            }
            if(ESP_OK == retval) {
                i16Temp = (int16_t)(ina219_dev[idx].power);
                ina219_data[idx].fPower = (float)(i16Temp) * ina219_current_lsb[idx] * 20.0f;
            } else {
                ESP_LOGE(TAG, "Dev%d: Error Reading Power Register!", (idx+1));
            }

            retval = _ina219_readReg(idx, REG_ADDR_CURRENT, &(ina219_dev[idx].current));
            if(ESP_OK == retval) {
                i16Temp = (int16_t)(ina219_dev[idx].current);
                ina219_data[idx].fCurrent = (float)(i16Temp) * ina219_current_lsb[idx];
            } else {
                ESP_LOGE(TAG, "Dev%d: Error Reading Current Register!", (idx+1));
            }
        }
    }
    /* Should not reach here */
    vTaskDelete(NULL);
}

esp_err_t ina219_init(void)
{
    esp_err_t retval = ESP_OK;
    i2c_config_t conf;
    uint8_t idx = 0;

    if(bInit == true) {
        /* Already initialized */
        return ESP_OK;
    }
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = CONFIG_INA219_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;       /* external Pull-up resistor is present in board */
    conf.scl_io_num = CONFIG_INA219_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;       /* external Pull-up resistor is present in board */
    conf.master.clk_speed = CONFIG_INA219_I2C_CLK_FREQ;
    retval = i2c_interface_init(CONFIG_INA219_I2C_PORT_NUM, &conf);
    if(retval != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init I2C%d", CONFIG_INA219_I2C_PORT_NUM);
        return(retval);
    }

    for(idx = 0; idx < CONFIG_INA219_DEVICE_COUNT; idx++) {
        ina219_data[idx].fBusVoltage = 0.0f;
        ina219_data[idx].fShuntVoltage = 0.0f;
        ina219_data[idx].fCurrent = 0.0f;
        ina219_data[idx].fPower = 0.0f;

        ina219_flag[idx].sync_config_pending = 0;

        ina219_dev[idx].configuration.data = ina219_dev_default[idx].configuration.data;
        ina219_dev[idx].shuntVoltage = ina219_dev_default[idx].shuntVoltage;
        ina219_dev[idx].busVoltage = ina219_dev_default[idx].busVoltage;
        ina219_dev[idx].power = ina219_dev_default[idx].power;
        ina219_dev[idx].current = ina219_dev_default[idx].current;
        ina219_dev[idx].calibration = ina219_dev_default[idx].calibration;
        retval = _ina219_writeReg(idx, REG_ADDR_CONFIGURATION, ina219_dev[idx].configuration.data);
        if(ESP_OK != retval) {
            ESP_LOGE(TAG, "Dev%d Error writing to Configration Reg!", (idx+1));
            ina219_flag[idx].sync_config_pending = 1;
            return(retval);
        }
        retval = _ina219_writeReg(idx, REG_ADDR_CALIBRATION, ina219_dev[idx].calibration);
        if(ESP_OK != retval) {
            ESP_LOGE(TAG, "Dev%d Error writing to Calibration Reg!", (idx+1));
            return(retval);
        }
    }

    updateDly = true;

#if defined(CONFIG_INA219_PINNED_TO_CORE_0)
    if(pdPASS != xTaskCreatePinnedToCore(
                _ina219_task,               /* the task function */
                "ina219",                   /* the name of the task */
                CONFIG_INA219_STACK_SIZE,   /* stack size */
                NULL,                       /* the 'pvParameters' parameter */
                CONFIG_INA219_TASK_PRIORITY,/* FreeRTOS priority */
                &ina219_task_handle,        /* task handle */
                PRO_CPU_NUM)) {
#elif defined(CONFIG_INA219_PINNED_TO_CORE_1)
    if(pdPASS != xTaskCreatePinnedToCore(
                _ina219_task,               /* the task function */
                "ina219",                   /* the name of the task */
                CONFIG_INA219_STACK_SIZE,   /* stack size */
                NULL,                       /* the 'pvParameters' parameter */
                CONFIG_INA219_TASK_PRIORITY,/* FreeRTOS priority */
                &ina219_task_handle,        /* task handle */
                APP_CPU_NUM)) {
#else
    if(pdPASS != xTaskCreate(
                _ina219_task,               /* the task function */
                "ina219",                   /* the name of the task */
                CONFIG_INA219_STACK_SIZE,   /* stack size */
                NULL,                       /* the 'pvParameters' parameter */
                CONFIG_INA219_TASK_PRIORITY,/* FreeRTOS priority */
                &ina219_task_handle)) {
#endif
        ESP_LOGE(TAG, "Task creation failed!");
        return ESP_FAIL;
    }
    return ESP_OK;
}

float ina219_get_bus_voltage(uint8_t devId, esp_err_t * pErr)
{
    float retval = 0.0f;
    if(bInit != true) {
        if(NULL != pErr) {
            *pErr = ESP_ERR_INVALID_STATE;
        }
        return(retval);
    }
    if(devId < CONFIG_INA219_DEVICE_COUNT) {
        retval = ina219_data[devId].fBusVoltage;
        if(NULL != pErr) {
            *pErr = ESP_OK;
        }
    } else {
        if(NULL != pErr) {
            *pErr = ESP_ERR_INVALID_ARG;
        }
    }
    return(retval);
}

float ina219_get_shunt_voltage(uint8_t devId, esp_err_t * pErr)
{
    float retval = 0.0f;
    if(bInit != true) {
        if(NULL != pErr) {
            *pErr = ESP_ERR_INVALID_STATE;
        }
        return(retval);
    }
    if(devId < CONFIG_INA219_DEVICE_COUNT) {
        retval = ina219_data[devId].fShuntVoltage;
        if(NULL != pErr) {
            *pErr = ESP_OK;
        }
    } else {
        if(NULL != pErr) {
            *pErr = ESP_ERR_INVALID_ARG;
        }
    }
    return(retval);
}

float ina219_get_current(uint8_t devId, esp_err_t * pErr)
{
    float retval = 0.0f;
    if(bInit != true) {
        if(NULL != pErr) {
            *pErr = ESP_ERR_INVALID_STATE;
        }
        return(retval);
    }
    if(devId < CONFIG_INA219_DEVICE_COUNT) {
        retval = ina219_data[devId].fCurrent;
        if(NULL != pErr) {
            *pErr = ESP_OK;
        }
    } else {
        if(NULL != pErr) {
            *pErr = ESP_ERR_INVALID_ARG;
        }
    }
    return(retval);
}

float ina219_get_power(uint8_t devId, esp_err_t * pErr)
{
    float retval = 0.0f;
    if(bInit != true) {
        if(NULL != pErr) {
            *pErr = ESP_ERR_INVALID_STATE;
        }
        return(retval);
    }
    if(devId < CONFIG_INA219_DEVICE_COUNT) {
        retval = ina219_data[devId].fPower;
        if(NULL != pErr) {
            *pErr = ESP_OK;
        }
    } else {
        if(NULL != pErr) {
            *pErr = ESP_ERR_INVALID_ARG;
        }
    }
    return(retval);
}

bool ina219_isInitialized(void)
{
    return(bInit);
}

esp_err_t ina219_set_mode(uint8_t devId, INA219_MODE_T mode)
{
    esp_err_t retval = ESP_OK;
    uint16_t fallback;
    if((devId >= CONFIG_INA219_DEVICE_COUNT) ||
       (mode >= N_INA219_MODE)){
        return ESP_ERR_INVALID_ARG;
    }
    if(bInit != true) {
        return ESP_ERR_INVALID_STATE;
    }
    fallback = ina219_dev[devId].configuration.data;
    ina219_dev[devId].configuration.field.mode = mode;
    retval = _ina219_writeReg(devId, REG_ADDR_CONFIGURATION, ina219_dev[devId].configuration.data);
    if(ESP_OK != retval) {
        ESP_LOGE(TAG, "Dev%d Error writing to Configration Reg!", (devId+1));
        ina219_dev[devId].configuration.data = fallback;
        ina219_flag[devId].sync_config_pending = 1;
        return(retval);
    }
    return ESP_OK;
}

esp_err_t ina219_get_mode(uint8_t devId, INA219_MODE_T * pMode)
{
    if((devId >= CONFIG_INA219_DEVICE_COUNT) ||
       (NULL == pMode)) {
        return ESP_ERR_INVALID_ARG;
    }
    if((ina219_flag[devId].sync_config_pending) ||
       (bInit != true)){
        return ESP_ERR_INVALID_STATE;
    }
    *pMode = ina219_dev[devId].configuration.field.mode;
    return ESP_OK;
}
