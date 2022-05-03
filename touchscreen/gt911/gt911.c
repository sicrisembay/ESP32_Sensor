/*
 * (C) Copyright 2022
 * Sicris Rey Embay, sicris.embay@gmail.com
 */

#include "sdkconfig.h"

#if defined(CONFIG_TC_DRIVER_GT911)

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "../include/touchscreen.h"
#include "gt911.h"
#include "../../common/i2c_interface.h"
#include "esp_log.h"
#include "string.h"

#if defined(CONFIG_TC_DRIVER_DEBUG)
#define GT911_PRINT_CONFIG          (0)
#define GT911_PRINT_TOUCH           (0)
#endif /* #if defined(CONFIG_TC_DRIVER_DEBUG) */
#define GT911_FORCE_UPDATE          (0)

#define _I2C_NUMBER(num)        I2C_NUM_##num
#define I2C_NUMBER(num)         _I2C_NUMBER(num)
#define I2C_MASTER_NUM          I2C_NUMBER(CONFIG_TC_I2C_PORT_NUM)

#define WRITE_BIT                       I2C_MASTER_WRITE
#define READ_BIT                        I2C_MASTER_READ
#define ACK_CHECK_EN                    (0x1)
#define ACK_CHECK_DIS                   (0x0)
#define ACK_VAL                         (0x0)
#define NACK_VAL                        (0x1)
#define TOUCH_NOTIFY_INT_FLAG           (0x00000001)

static const char *TAG = "gt911";
static DRAM_ATTR TaskHandle_t touch_task_handle = NULL;
static DRAM_ATTR QueueHandle_t mutexTouch = NULL;
static bool bInit = false;

static touch_status_t touchStatus;
static touch_packet_t touches[MAX_TOUCHES];
static touch_configuration_info_t configInfo;
static touch_coordinate_info_t coordInfo;
static touch_mode_status_t modeStatus;

#if (GT911_FORCE_UPDATE == 1)
static touch_configuration_info_t const configDefault = {
    .buf = {
        /* 
         * Put your default configuration here
         * Refer to GT911 Programming Guide 
         */
        0x46, 0x40, 0x01, 0xE0, 0x01, 0x0A, 0x05, 0x00, 0x01, 0x08,
        0x28, 0x05, 0x50, 0x32, 0x03, 0x05, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x87, 0x28, 0x0A,
        0x17, 0x15, 0x31, 0x0D, 0x00, 0x00, 0x02, 0xBD, 0x04, 0x24,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x64, 0x32, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E, 0x10,
        0x12, 0x14, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0x00, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x26,
        0x24, 0x22, 0x21, 0x20, 0x1F, 0x1E, 0x1D, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x69
    }
};
#endif /* GT911_FORCE_UPDATE == 1 */

static void IRAM_ATTR _touch_int_isr_handler(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken;
    if((uint32_t) arg == CONFIG_TC_INT_IO_PIN) {
        xTaskNotifyFromISR(touch_task_handle, TOUCH_NOTIFY_INT_FLAG, eSetBits, &xHigherPriorityTaskWoken);
        if(xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    }
}

#if (GT911_FORCE_UPDATE == 1)
static esp_err_t _gt911_reset_factory_config(void)
{
    uint8_t checksum = 0;
    uint32_t i = 0;
    uint8_t temp;
    esp_err_t ret;
    uint8_t regAddr[2] = {0};

    if(!memcmp(configInfo.buf, configDefault.buf, sizeof(touch_configuration_info_t))) {
        /* Existing configuration is the same as factory
           No need to reset */
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Reverting config to factory default.");
    memcpy(configInfo.buf, configDefault.buf, sizeof(touch_configuration_info_t));

    /* Validate Checksum */
    checksum = 0;
    for(i = 0; i < sizeof(touch_configuration_info_t); i++) {
        checksum += configInfo.buf[i];
    }

    if(checksum != 0) {
        ESP_LOGE(TAG, "%s : Invalid checksum", __FUNCTION__);
        return ESP_FAIL;
    } else {
        regAddr[0] = (GT911_REG_CONFIG_VERSION >> 8) & 0x00FF;
        regAddr[1] = GT911_REG_CONFIG_VERSION & 0x00FF;
        ret = i2c_interface_write(CONFIG_TC_I2C_PORT_NUM, CONFIG_TC_SLAVE_ADDR, 
                        (unsigned char *)regAddr, sizeof(regAddr),
                        configInfo.buf, sizeof(touch_configuration_info_t));
        if(ESP_OK != ret) {
            ESP_LOGE(TAG, "%s:%d Failed to write new configuration", __FUNCTION__, __LINE__);
            return(ret);
        }
        /* Tell GT911 to update config */
        temp = 1;
        regAddr[0] = (GT911_REG_CONFIG_FRESH >> 8) & 0x00FF;
        regAddr[1] = GT911_REG_CONFIG_FRESH & 0x00FF;
        ret = i2c_interface_write(CONFIG_TC_I2C_PORT_NUM, CONFIG_TC_SLAVE_ADDR,
                    (unsigned char *)regAddr, sizeof(regAddr),
                    &temp, 1);
        if(ESP_OK != ret) {
            ESP_LOGE(TAG, "%s:%d Failed to write new configuration", __FUNCTION__, __LINE__);
            return(ret);
        }
    }

    ESP_LOGI(TAG, "Revert done.");
    return ESP_OK;
}
#endif /* GT911_FORCE_UPDATE == 1 */

static esp_err_t _gt911_get_config(void)
{
    uint8_t checksum = 0;
    uint32_t i = 0;
    uint8_t regAddr[2] = {0};

    /* Read GT911 configuration */
    ESP_LOGI(TAG, "Reading Configuration Information... ");
    regAddr[0] = (GT911_REG_CONFIG_VERSION >> 8) & 0x00FF;
    regAddr[1] = GT911_REG_CONFIG_VERSION & 0x00FF;
    esp_err_t ret = i2c_interface_read(CONFIG_TC_I2C_PORT_NUM, CONFIG_TC_SLAVE_ADDR,
                        (unsigned char *)regAddr, sizeof(regAddr), 
                        configInfo.buf, sizeof(touch_configuration_info_t));
    if(ESP_OK != ret) {
        ESP_LOGE(TAG, "Failed to read configuration. ret =  0x%02X", ret);
        return (ret);
    }
    /* Check validity */
    checksum = 0;
    for(i = 0; i < sizeof(touch_configuration_info_t); i++) {
        checksum += configInfo.buf[i];
    }
    if(checksum != 0) {
        ESP_LOGE(TAG, "Configuration checksum error");
    } else {
        ESP_LOGI(TAG, "Configuration checksum OK");
    }

#if(GT911_PRINT_CONFIG == 1)
    char strConfigLine[128] = {0};
    char configVal[8] = {0};
    i = 0;
    ESP_LOGI(TAG, "Configuration dump start ...");
    while(i < sizeof(touch_configuration_info_t)) {
        if(((i % 10) == 0) && (i != 0)) {
            ESP_LOGI(TAG, "0x%04X : %s", (GT911_REG_CONFIG_VERSION + i - 10), strConfigLine);
            strConfigLine[0] = 0;
        }
        snprintf(configVal, 7, "0x%02X ", configInfo.buf[i]);
        strcat(strConfigLine, configVal);
        i++;
    }
    ESP_LOGI(TAG, "0x%04X : %s", (GT911_REG_CONFIG_VERSION + i - (i%10)), strConfigLine);
    ESP_LOGI(TAG, "... end of Configuration Dump");
#endif

    return (ret);
}


IRAM_ATTR static void _touch_task(void *pvParam)
{
    uint32_t intStatusFlag = 0;
    gpio_config_t io_conf;
    esp_err_t ret = ESP_OK;
    uint32_t i;
    uint8_t regAddr[2] = {0};

    memset(touches, 0, sizeof(touches));

    /*
     * Reset
     * Note: Must follow bootstrap timing specified in GT911 datasheet
     * See timing diagram to assign slave address 0x5D
     */
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << CONFIG_TC_INT_IO_PIN);
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    /* RESET and INT sequence determines the i2c device address */
    gpio_set_level(CONFIG_TC_RESET_IO_PIN, 0);
    gpio_set_level(CONFIG_TC_INT_IO_PIN, 0);
    vTaskDelay(1/portTICK_PERIOD_MS);
    gpio_set_level(CONFIG_TC_RESET_IO_PIN, 1);
    vTaskDelay(60/portTICK_PERIOD_MS);
    io_conf.mode = GPIO_MODE_INPUT;
#if CONFIG_TC_INT_PULL_UP
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
#else
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
#endif
    io_conf.pin_bit_mask = (1ULL << CONFIG_TC_INT_IO_PIN);
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    /*
     * Install gpio isr service
     */
    ESP_ERROR_CHECK(gpio_set_intr_type(CONFIG_TC_INT_IO_PIN, GPIO_INTR_NEGEDGE));
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
    ESP_ERROR_CHECK(gpio_isr_handler_add(CONFIG_TC_INT_IO_PIN, _touch_int_isr_handler, (void*)CONFIG_TC_INT_IO_PIN));

    bInit = true;
    ESP_LOGI(TAG, "touch task running..\n");

    regAddr[0] = (GT911_REG_PRODUCT_ID_FIRST_BYTE >> 8) & 0x00FF;
    regAddr[1] = GT911_REG_PRODUCT_ID_FIRST_BYTE & 0x00FF;
    ret = i2c_interface_read(CONFIG_TC_I2C_PORT_NUM, CONFIG_TC_SLAVE_ADDR,
                (unsigned char *)regAddr, sizeof(regAddr),
                coordInfo.buf, sizeof(touch_coordinate_info_t));
    ESP_ERROR_CHECK(ret);
#if defined(CONFIG_TC_DRIVER_DEBUG)
    ESP_LOGI(TAG, "Product ID: 0x%02X 0x%02X 0x%02X 0x%02X",
                   coordInfo.prod_id_first_byte, coordInfo.prod_id_second_byte,
                   coordInfo.prod_id_third_byte, coordInfo.prod_id_fourth_byte);
    ESP_LOGI(TAG, "Firmware Version: 0x%04X", coordInfo.firmware_version);
    ESP_LOGI(TAG, "X-Resolution: %d", coordInfo.x_coord_resolution);
    ESP_LOGI(TAG, "Y-Resolution: %d", coordInfo.y_coord_resolution);
    ESP_LOGI(TAG, "Vendor ID: 0x%02X", coordInfo.vendor_id);
#endif /* #if defined(CONFIG_TC_DRIVER_DEBUG) */
    regAddr[0] = (GT911_REG_CMD_STATUS >> 8) & 0x00FF;
    regAddr[1] = GT911_REG_CMD_STATUS & 0x00FF;
    ret = i2c_interface_read(CONFIG_TC_I2C_PORT_NUM, CONFIG_TC_SLAVE_ADDR,
                    (unsigned char *)regAddr, sizeof(regAddr),
                    modeStatus.buf, sizeof(touch_mode_status_t));
    ESP_ERROR_CHECK(ret);
    if(modeStatus.gt911_status == modeStatus.gt911_status_bak) {
        ESP_LOGI(TAG, "Mode: 0x%02X", modeStatus.gt911_status);
    } else {
        ESP_LOGE(TAG, "GT Status consistency Error.");
    }

    _gt911_get_config();

#if (GT911_FORCE_UPDATE == 1)
    _gt911_reset_factory_config();
    vTaskDelay(10);
#endif

    while(1) {
        xTaskNotifyWait(0x00, ULONG_MAX, &intStatusFlag, portMAX_DELAY);
        if(intStatusFlag & TOUCH_NOTIFY_INT_FLAG) {
            /* Read GT911 Data */
            regAddr[0] = (GT911_REG_STATUS >> 8) & 0x00FF;
            regAddr[1] = GT911_REG_STATUS & 0x00FF;
            ret = i2c_interface_read(CONFIG_TC_I2C_PORT_NUM, CONFIG_TC_SLAVE_ADDR,
                        (unsigned char *)regAddr, sizeof(regAddr),
                        &(touchStatus.buf), 1);
            if(ESP_OK != ret) {
                ESP_LOGE(TAG, "I2C read error (line %d)!\n", __LINE__);
                continue;
            }
#if (GT911_PRINT_TOUCH == 1)
            ESP_LOGI(TAG, "Touch Status: Reg 0x814E: 0x%02X", touchStatus.buf);
#endif
            if(touchStatus.buffer_status == 0) {
                continue;
            }

            if((!(touchStatus.number_of_touch_points)) ||
               (touchStatus.number_of_touch_points > MAX_TOUCHES)) {
                for(i = 0; i < MAX_TOUCHES; i++) {
                    touches[i].x = -1;
                    touches[i].y = -1;
                }
                touchStatus.number_of_touch_points = 0;
                /* Clear buffer status */
                touchStatus.buf = 0;
                regAddr[0] = (GT911_REG_STATUS >> 8) & 0x00FF;
                regAddr[1] = GT911_REG_STATUS & 0x00FF;
                ret = i2c_interface_write(CONFIG_TC_I2C_PORT_NUM, CONFIG_TC_SLAVE_ADDR,
                            (unsigned char *)regAddr, sizeof(regAddr),
                            &(touchStatus.buf), 1);
                if(ESP_OK != ret) {
                    ESP_LOGE(TAG, "I2C write error (line %d)!\n", __LINE__);
                    continue;
                }
                continue;
            }

            /*
             * Start of mutually exclusive section
             */
            if(pdTRUE != xSemaphoreTakeRecursive(mutexTouch, 100/portTICK_PERIOD_MS)) {
                ESP_LOGE(TAG, "%s:%d: Failed to get mutex!\n", __FUNCTION__, __LINE__);
                continue;
            }
            regAddr[0] = (GT911_REG_PT1_INFO >> 8) & 0x00FF;
            regAddr[1] = GT911_REG_PT1_INFO & 0x00FF;
            ret = i2c_interface_read(CONFIG_TC_I2C_PORT_NUM, CONFIG_TC_SLAVE_ADDR, 
                        (unsigned char *)regAddr, sizeof(regAddr),
                        &(touches[0].buf[0]), touchStatus.number_of_touch_points * 8);
            /*
             * End of mutually exclusive section
             */
            xSemaphoreGiveRecursive(mutexTouch);

            if(ESP_OK != ret) {
                ESP_LOGE(TAG, "I2C read error (line %d)!\n", __LINE__);
                continue;
            }
            touchStatus.buf = 0;
            regAddr[0] = (GT911_REG_STATUS >> 8) & 0x00FF;
            regAddr[1] = GT911_REG_STATUS & 0x00FF;
            ret = i2c_interface_write(CONFIG_TC_I2C_PORT_NUM, CONFIG_TC_SLAVE_ADDR,
                        (unsigned char *)regAddr, sizeof(regAddr),
                        &(touchStatus.buf), 1);
            if(ESP_OK != ret) {
                ESP_LOGE(TAG, "I2C write error (line %d)!\n", __LINE__);
                continue;
            }
        }
    }
}

esp_err_t touchscreen_init(void)
{
    esp_err_t ret = ESP_OK;
    gpio_config_t io_conf;

    if((bInit != true) && (touch_task_handle == NULL)) {
        /*
         * Initialize INT pin
         */
        io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pin_bit_mask = (1ULL << CONFIG_TC_INT_IO_PIN);
        io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        ret = gpio_config(&io_conf);
        if(ESP_OK != ret) {
            ESP_LOGE(TAG, "%s:%d:Error %d\n", __FUNCTION__, __LINE__, ret);
            return(ret);
        }

        /*
         * Initialize Reset pin
         */
        io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = (1ULL << CONFIG_TC_RESET_IO_PIN);
        io_conf.pull_down_en = GPIO_PULLUP_DISABLE;
        io_conf.pull_up_en = GPIO_PULLDOWN_DISABLE;
        ret = gpio_config(&io_conf);
        if(ESP_OK != ret) {
            ESP_LOGE(TAG, "%s:%d:Error %d\n", __FUNCTION__, __LINE__, ret);
            return(ret);
        }
        /* Hold GT911 in reset */
        gpio_set_level(CONFIG_TC_RESET_IO_PIN, 0);

        /*
         * Initialize I2C
         */
        ret = i2c_interface_init();
        if(ESP_OK != ret) {
            ESP_LOGE(TAG, "%s:%d:Error %d\n", __FUNCTION__, __LINE__, ret);
            return(ret);
        }

        /*
         * Create Mutex
         */
        mutexTouch = xSemaphoreCreateRecursiveMutex();
        if(mutexTouch == NULL) {
            ESP_LOGE(TAG, "touch mutex creation failed\n");
            ret = ESP_FAIL;
            return(ret);
        }

        /*
         * Create Touch Task
         */
#if defined(CONFIG_TC_PINNED_TO_CORE_0)
        if(pdPASS != xTaskCreatePinnedToCore(
                    _touch_task,                /* the task function */
                    "gt911",                   /* the name of the task */
                    CONFIG_TC_TASK_STACK_SIZE,  /* stack size */
                    NULL,                       /* the 'pvParameters' parameter */
                    CONFIG_TC_TASK_PRIORITY,    /* FreeRTOS priority */
                    &touch_task_handle,         /* task handle */
                    PRO_CPU_NUM)) {
#elif defined(CONFIG_TC_PINNED_TO_CORE_1)
        if(pdPASS != xTaskCreatePinnedToCore(
                    _touch_task,                /* the task function */
                    "gt911",                   /* the name of the task */
                    CONFIG_TC_TASK_STACK_SIZE,  /* stack size */
                    NULL,                       /* the 'pvParameters' parameter */
                    CONFIG_TC_TASK_PRIORITY,    /* FreeRTOS priority */
                    &touch_task_handle,         /* task handle */
                    APP_CPU_NUM)) {
#else
        if(pdPASS != xTaskCreate(
                    _touch_task,                /* the task function */
                    "gt911",                   /* the name of the task */
                    CONFIG_TC_TASK_STACK_SIZE,  /* stack size */
                    NULL,                       /* the 'pvParameters' parameter */
                    CONFIG_TC_TASK_PRIORITY,    /* FreeRTOS priority */
                    &touch_task_handle)) {
#endif
            ESP_LOGE(TAG, "touch task creation failed!\n");
            ret = ESP_FAIL;
            return (ret);
        }
    }

    return (ret);
}

esp_err_t touchscreen_getCoord(uint8_t touchId, int16_t * px, int16_t * py)
{
    esp_err_t ret = ESP_OK;

    if(bInit != true) {
        ESP_LOGW(TAG, "Not initialized!\n");
        ret = ESP_ERR_INVALID_STATE;
        return (ret);
    }

    if((touchId >= MAX_TOUCHES) || (px == NULL) || (py == NULL)) {
        ESP_LOGW(TAG, "%s: Invalid Argument!\n", __FUNCTION__);
        ret = ESP_ERR_INVALID_ARG;
        return (ret);
    }

    if(pdTRUE != xSemaphoreTakeRecursive(mutexTouch, 100/portTICK_PERIOD_MS)) {
        ESP_LOGE(TAG, "%s:%d: Failed to get mutex!\n", __FUNCTION__, __LINE__);
        ret = ESP_FAIL;
        return (ret);
    }

    *px = touches[touchId].x;
    *py = touches[touchId].y;

    xSemaphoreGiveRecursive(mutexTouch);

    return ESP_OK;
}
#endif /* CONFIG_TC_DRIVER_GT911 */