#include "sdkconfig.h"
#if defined(CONFIG_SENSOR_ADS122C04)
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ads122c04.h"
#include "../common/i2c_interface.h"
#include "driver/gpio.h"

/* Command Definition */
#define ADS122C04_RESET         (0x06)
#define ADS122C04_START_SYNC    (0x08)
#define ADS122C04_POWERDOWN     (0x02)
#define ADS122C04_RDATA         (0x10)
#define ADS122C04_RREG0         (0x20)
#define ADS122C04_RREG1         (0x24)
#define ADS122C04_RREG2         (0x28)
#define ADS122C04_RREG3         (0x2C)
#define ADS122C04_WREG0         (0x40)
#define ADS122C04_WREG1         (0x44)
#define ADS122C04_WREG2         (0x48)
#define ADS122C04_WREG3         (0x4C)

#define ADS122C04_NOTIFY_RDY_FLAG (0x00000001UL)

typedef union {
    uint8_t byte;
    struct {
        uint8_t PGA_BYPASS  : 1;
        uint8_t GAIN        : 3;
        uint8_t MUX         : 4;
    } bitField;
} REG0;

typedef union {
    uint8_t byte;
    struct {
        uint8_t TS          : 1;
        uint8_t VREF        : 2;
        uint8_t CM          : 1;
        uint8_t MODE        : 1;
        uint8_t DR          : 3;
    } bitField;
} REG1;

typedef union {
    uint8_t byte;
    struct {
        uint8_t IDAC        : 3;
        uint8_t BCS         : 1;
        uint8_t CRC         : 2;
        uint8_t DCNT        : 1;
        uint8_t DRDY        : 1;
    } bitField;
} REG2;

typedef union {
    uint8_t byte;
    struct {
        uint8_t reserved    : 2;
        uint8_t I2MUX       : 3;
        uint8_t I1MUX       : 3;
    } bitField;
} REG3;

typedef struct {
    REG0 reg0;
    REG1 reg1;
    REG2 reg2;
    REG3 reg3;
} ADS122C04_REG;

#if defined(CONFIG_ADS122C04_DATA_INTEGRITY_DISABLE)
#define READ_BUF_SIZE       (3)
#elif defined(CONFIG_ADS122C04_DATA_INTEGRITY_INVERT)
#define READ_BUF_SIZE       (6)
#else
#error "Unsupported Data Integrity Configuration"
#endif

static ADS122C04_REG const reg_default = {
    .reg0.byte = 0x4E,  // AINP=AIN1, AINN=AIN2, GAIN=128, PGA Enabled
    .reg1.byte = 0x0C,
#if defined(CONFIG_ADS122C04_DATA_INTEGRITY_DISABLE)
    .reg2.byte = 0x00,
#elif defined(CONFIG_ADS122C04_DATA_INTEGRITY_INVERT)
    .reg2.byte = 0x10,
#else
#error "Unsupported Data Integrity Configuration"
#endif
    .reg3.byte = 0x00
};

typedef struct {
    int32_t rawData;
    int32_t temperatureCode;
    ADS122C04_REG reg;
    uint32_t errCnt;
} ads122c04_record_t;

static const char *TAG = "ads122c04";
static DRAM_ATTR TaskHandle_t ads122c04_task_handle = NULL;
static bool bInit = false;
static ads122c04_record_t ads122c04_record;
static conversion_cb_t conversion_cb = NULL;

static void IRAM_ATTR _drdy_isr_handler(void * arg)
{
    BaseType_t xHigherPriorityTaskWoken;
    if((uint32_t)arg == CONFIG_ADS122C04_DRDY_IO) {
        xTaskNotifyFromISR(ads122c04_task_handle, ADS122C04_NOTIFY_RDY_FLAG,
                        eSetBits, &xHigherPriorityTaskWoken);
        if(xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    }
}

static void _ads122c04_task(void *pArg)
{
    uint32_t notifyFlag = 0;
    int32_t rawData;
    uint8_t regAddr = 0;
    uint8_t dummy;
    esp_err_t ret = ESP_OK;
    uint8_t dataBuf[READ_BUF_SIZE];

    /* Install isr service for DRDY */
    ret = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if(ESP_ERR_INVALID_STATE == ret) {
        /* Already installed by someone else */
    } else {
        ESP_ERROR_CHECK(ret);
    }
    ESP_ERROR_CHECK(gpio_set_intr_type(CONFIG_ADS122C04_DRDY_IO, GPIO_INTR_NEGEDGE));
    ESP_ERROR_CHECK(gpio_isr_handler_add(CONFIG_ADS122C04_DRDY_IO, _drdy_isr_handler,
                    (void*)CONFIG_ADS122C04_DRDY_IO));

    /* Initialize ADS122C04 registers */
    ads122c04_record.reg.reg0.byte = reg_default.reg0.byte;
    ads122c04_record.reg.reg1.byte = reg_default.reg1.byte;
    ads122c04_record.reg.reg2.byte = reg_default.reg2.byte;
    ads122c04_record.reg.reg3.byte = reg_default.reg3.byte;
    regAddr = ADS122C04_RESET;
    ESP_ERROR_CHECK(i2c_interface_write(CONFIG_ADS122C04_I2C_PORT_NUM,
            CONFIG_ADS122C04_ADDR_DEV, &regAddr, 1, &dummy, 0));
    vTaskDelay(10);
    regAddr = ADS122C04_WREG0;
    ESP_ERROR_CHECK(i2c_interface_write(CONFIG_ADS122C04_I2C_PORT_NUM,
            CONFIG_ADS122C04_ADDR_DEV, &regAddr, 1, &ads122c04_record.reg.reg0.byte, 1));
    ESP_ERROR_CHECK(i2c_interface_write(CONFIG_ADS122C04_I2C_PORT_NUM,
            CONFIG_ADS122C04_ADDR_DEV, &regAddr, 1, &ads122c04_record.reg.reg0.byte, 1));
    regAddr = ADS122C04_WREG1;
    ESP_ERROR_CHECK(i2c_interface_write(CONFIG_ADS122C04_I2C_PORT_NUM,
            CONFIG_ADS122C04_ADDR_DEV, &regAddr, 1, &ads122c04_record.reg.reg1.byte, 1));
    regAddr = ADS122C04_WREG2;
    ESP_ERROR_CHECK(i2c_interface_write(CONFIG_ADS122C04_I2C_PORT_NUM,
            CONFIG_ADS122C04_ADDR_DEV, &regAddr, 1, &ads122c04_record.reg.reg2.byte, 1));
    regAddr = ADS122C04_WREG3;
    ESP_ERROR_CHECK(i2c_interface_write(CONFIG_ADS122C04_I2C_PORT_NUM,
            CONFIG_ADS122C04_ADDR_DEV, &regAddr, 1, &ads122c04_record.reg.reg3.byte, 1));

    regAddr = ADS122C04_START_SYNC;
    ESP_ERROR_CHECK(i2c_interface_write(CONFIG_ADS122C04_I2C_PORT_NUM,
            CONFIG_ADS122C04_ADDR_DEV, &regAddr, 1, &dummy, 0));

    bInit = true;
    ESP_LOGI(TAG, "ads122c04 task running..\n");

    while(1) {
        if(pdPASS == xTaskNotifyWait(0UL, ULONG_MAX, &notifyFlag, portMAX_DELAY)) {
                if(notifyFlag & ADS122C04_NOTIFY_RDY_FLAG) {
                    regAddr = ADS122C04_RDATA;
                    ESP_ERROR_CHECK(i2c_interface_read(CONFIG_ADS122C04_I2C_PORT_NUM,
                        CONFIG_ADS122C04_ADDR_DEV, &regAddr, 1, (unsigned char *)&dataBuf, READ_BUF_SIZE));
#if defined(CONFIG_ADS122C04_DATA_INTEGRITY_DISABLE)
                    rawData = (int32_t)((((uint32_t)dataBuf[0]) << 16) + (((uint32_t)dataBuf[1]) << 8) + ((uint32_t)dataBuf[2]));
#elif defined(CONFIG_ADS122C04_DATA_INTEGRITY_INVERT)
                    /* Check by bitwise-inversion */
                    if(((dataBuf[0] & dataBuf[3]) == 0) &&
                       ((dataBuf[1] & dataBuf[4]) == 0) &&
                       ((dataBuf[2] & dataBuf[5]) == 0)) {
                        rawData = (int32_t)((((uint32_t)dataBuf[0]) << 16) + (((uint32_t)dataBuf[1]) << 8) + ((uint32_t)dataBuf[2]));
                    } else {
                        ESP_LOGE(TAG, "Data integrity failed!");
                        continue;
                    }
#else
#error "Unsupported Data Integrity Configuration"
#endif
                    if(dataBuf[0] & 0x80) {
                        rawData |= 0xFF000000;
                    }
                    ads122c04_record.rawData = rawData;
                    if(conversion_cb != NULL) {
                        (*conversion_cb)(rawData);
                    }
                }
        }
    }
    /* Should not reach here */
    vTaskDelete(NULL);
}

esp_err_t ads122c04_init(conversion_cb_t cb)
{
    esp_err_t retval = ESP_OK;
    gpio_config_t io_conf;

    if(bInit == true) {
        /* Already initialized */
        return ESP_OK;
    }

    conversion_cb = cb;

    /* ADS122C04 Data Ready */
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.pin_bit_mask = (1ULL << CONFIG_ADS122C04_DRDY_IO);
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
        ESP_LOGE(TAG, "Failed to init I2C%d", CONFIG_ADS122C04_I2C_PORT_NUM);
        return(retval);
    }

#if defined(CONFIG_ADS122C04_PINNED_TO_CORE_0)
    if(pdPASS != xTaskCreatePinnedToCore(
                _ads122c04_task,                /* the task function */
                "ads122c04",                    /* the name of the task */
                CONFIG_ADS122C04_STACK_SIZE,    /* stack size */
                NULL,                           /* the 'pvParameters' parameter */
                CONFIG_ADS122C04_TASK_PRIORITY, /* FreeRTOS priority */
                &ads122c04_task_handle,         /* task handle */
                PRO_CPU_NUM)) {
#elif defined(CONFIG_ADS122C04_PINNED_TO_CORE_1)
    if(pdPASS != xTaskCreatePinnedToCore(
                _ads122c04_task,                /* the task function */
                "ads122c04",                    /* the name of the task */
                CONFIG_ADS122C04_STACK_SIZE,    /* stack size */
                NULL,                           /* the 'pvParameters' parameter */
                CONFIG_ADS122C04_TASK_PRIORITY, /* FreeRTOS priority */
                &ads122c04_task_handle,         /* task handle */
                APP_CPU_NUM)) {
#else
    if(pdPASS != xTaskCreate(
                _ads122c04_task,                /* the task function */
                "ads122c04",                    /* the name of the task */
                CONFIG_ADS122C04_STACK_SIZE,    /* stack size */
                NULL,                           /* the 'pvParameters' parameter */
                CONFIG_ADS122C04_TASK_PRIORITY, /* FreeRTOS priority */
                &ads122c04_task_handle)) {
#endif
        ESP_LOGE(TAG, "Task creation failed!");
        return ESP_FAIL;
    }

    return (retval);
}

bool ads122c04_isInitialized(void)
{
    return (bInit);
}

int32_t ads122c04_getRaw(void)
{
    return (ads122c04_record.rawData);
}



#endif /* CONFIG_SENSOR_ADS122C04 */
