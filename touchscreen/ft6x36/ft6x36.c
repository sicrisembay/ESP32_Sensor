/*
 * (C) Copyright 2020
 * Sicris Rey Embay, sicris.embay@gmail.com
 */

#include "sdkconfig.h"

#if defined(CONFIG_TC_DRIVER_FT6336)
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "../include/touchscreen.h"
#include "../../common/i2c_interface.h"
#include "ft6x36.h"
#include "driver/gpio.h"

#define TOUCH_NOTIFY_INT_FLAG           (0x00000001)

static const char *TAG = "ft6336";
static DRAM_ATTR TaskHandle_t touch_task_handle = NULL;
static IRAM_DATA_ATTR QueueHandle_t mutexTouch = NULL;
static bool bInit = false;
static ft6x36_reg_t ft6336_reg;

static touch_pt_t touch_pt[2];

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

static void _touch_task(void * pArg)
{
    uint32_t intStatusFlag = 0;
    esp_err_t ret = ESP_OK;
    uint32_t i;
    uint8_t evtFlag;
    uint8_t regAddr;


    for(i = 0; i < 2; i++) {
        touch_pt[i].evtFlag = FT6X36_EVT_NONE;
        touch_pt[i].x = -1;
        touch_pt[i].y = -1;
    }


    /*
     * Reset
     */
    ESP_ERROR_CHECK(gpio_set_level(CONFIG_TC_RESET_IO_PIN, 0));
    vTaskDelay(100/portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(gpio_set_level(CONFIG_TC_RESET_IO_PIN, 1));

    /*
     * Check Venor ID
     */
    regAddr = FT6X36_REG_LIB_VER_H;
    ESP_ERROR_CHECK(i2c_interface_read(CONFIG_TC_I2C_PORT_NUM, CONFIG_TC_SLAVE_ADDR,
                &regAddr, 1, &(ft6336_reg.regBuf[FT6X36_REG_LIB_VER_H]), 15));
    ESP_LOGI(TAG, "LIB Version 0x%02X%02X\n", ft6336_reg.regBuf[FT6X36_REG_LIB_VER_H], ft6336_reg.regBuf[FT6X36_REG_LIB_VER_L]);
    ESP_LOGI(TAG, "Firmware Version 0x%02X\n", ft6336_reg.regBuf[FT6X36_REG_FIRMID]);
    ESP_LOGI(TAG, "FocalTech Panel ID: 0x%02X\n", ft6336_reg.regBuf[FT6X36_REG_FOCALTECH_ID]);
    ESP_LOGI(TAG, "Release Code: 0x%02X\n", ft6336_reg.regBuf[FT6x36_REG_REL_CODE_ID]);

    /*
     * Install gpio isr service
     */
    ret = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if(ESP_ERR_INVALID_STATE == ret) {
        /* Already installed by someone else */
    } else {
        ESP_ERROR_CHECK(ret);
    }
    ESP_ERROR_CHECK(gpio_set_intr_type(CONFIG_TC_INT_IO_PIN, GPIO_INTR_NEGEDGE));
    ESP_ERROR_CHECK(gpio_isr_handler_add(CONFIG_TC_INT_IO_PIN, _touch_int_isr_handler, (void*)CONFIG_TC_INT_IO_PIN));

    bInit = true;
    ESP_LOGI(TAG, "touch task running..\n");

    while(1) {
        xTaskNotifyWait(0x00, ULONG_MAX, &intStatusFlag, portMAX_DELAY);
        if(intStatusFlag & TOUCH_NOTIFY_INT_FLAG) {
            /* Read FT6X36 Data */
            regAddr = FT6X36_REG_TD_STATUS;
            ret = i2c_interface_read(CONFIG_TC_I2C_PORT_NUM, CONFIG_TC_SLAVE_ADDR,
                    &regAddr, 1, &(ft6336_reg.regBuf[FT6X36_REG_TD_STATUS]), 13);
            if(ESP_OK != ret) {
                ESP_LOGE(TAG, "I2C read error!\n");
            }

            /*
             * Start of mutually exclusive section
             */
            if(pdTRUE != xSemaphoreTakeRecursive(mutexTouch, 100/portTICK_PERIOD_MS)) {
                ESP_LOGE(TAG, "%s:%d: Failed to get mutex!\n", __FUNCTION__, __LINE__);
                continue;
            }

            for(i = 0; i < 2; i++) {
                touch_pt[i].evtFlag = FT6X36_EVT_NONE;
                touch_pt[i].x = -1;
                touch_pt[i].y = -1;
            }

            if(ft6336_reg.regBuf[FT6X36_REG_TD_STATUS] & 0x0F) {
                /* P1 : Check Contact Event */
                evtFlag = (ft6336_reg.regBuf[FT6X36_REG_P1_XH] >> 6) & 0x03;
                if(evtFlag == FT6X36_EVT_CONTACT) {
                    /* Get Touch ID */
                    i = (ft6336_reg.regBuf[FT6X36_REG_P1_YH] >> 4) & 0x0F;
                    if(i != 0x0F) {
                        /* Valid Touch ID */
                        if(i < 2) {
                            touch_pt[i].evtFlag = evtFlag;
                            touch_pt[i].x = ((ft6336_reg.regBuf[FT6X36_REG_P1_XH] & 0x0F) << 8) + 
                                        ft6336_reg.regBuf[FT6X36_REG_P1_XL];
                            touch_pt[i].y = ((ft6336_reg.regBuf[FT6X36_REG_P1_YH] & 0x0F) << 8) + 
                                        ft6336_reg.regBuf[FT6X36_REG_P1_YL];
                         }
                    }
                }
                /* P2 : Check Contact Event */
                evtFlag = (ft6336_reg.regBuf[FT6X36_REG_P2_XH] >> 6) & 0x03;
                if(evtFlag == FT6X36_EVT_CONTACT) {
                    /* Get Touch ID */
                    i = (ft6336_reg.regBuf[FT6X36_REG_P2_YH] >> 4) & 0x0F;
                    if(i != 0x0F) {
                        /* Valid Touch ID */
                        if(i < 2) {
                            touch_pt[i].evtFlag = evtFlag;
                            touch_pt[i].x = ((ft6336_reg.regBuf[FT6X36_REG_P2_XH] & 0x0F) << 8) + 
                                        ft6336_reg.regBuf[FT6X36_REG_P2_XL];
                            touch_pt[i].y = ((ft6336_reg.regBuf[FT6X36_REG_P2_YH] & 0x0F) << 8) + 
                                        ft6336_reg.regBuf[FT6X36_REG_P2_YL];
                         }
                    }
                }
            }
            /*
             * End of mutually exclusive section
             */
            xSemaphoreGiveRecursive(mutexTouch);
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
        io_conf.pull_down_en = 0;
        io_conf.pull_up_en = 0;
        ret = gpio_config(&io_conf);
        ESP_ERROR_CHECK(ret);
        gpio_set_level(CONFIG_TC_RESET_IO_PIN, 1);
        
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
#if defined(CONFIG_MARG_PINNED_TO_CORE_0)
        if(pdPASS != xTaskCreatePinnedToCore(
                    _touch_task,                /* the task function */
                    "ft6336",                   /* the name of the task */
                    CONFIG_TC_TASK_STACK_SIZE,  /* stack size */
                    NULL,                       /* the 'pvParameters' parameter */
                    CONFIG_TC_TASK_PRIORITY,    /* FreeRTOS priority */
                    &touch_task_handle,         /* task handle */
                    PRO_CPU_NUM)) {
#elif defined(CONFIG_MARG_PINNED_TO_CORE_1)
        if(pdPASS != xTaskCreatePinnedToCore(
                    _touch_task,                /* the task function */
                    "ft6336",                   /* the name of the task */
                    CONFIG_TC_TASK_STACK_SIZE,  /* stack size */
                    NULL,                       /* the 'pvParameters' parameter */
                    CONFIG_TC_TASK_PRIORITY,    /* FreeRTOS priority */
                    &touch_task_handle,         /* task handle */
                    APP_CPU_NUM)) {
#else
        if(pdPASS != xTaskCreate(
                    _touch_task,                /* the task function */
                    "ft6336",                   /* the name of the task */
                    CONFIG_TC_TASK_STACK_SIZE,  /* stack size */
                    NULL,                       /* the 'pvParameters' parameter */
                    CONFIG_TC_TASK_PRIORITY,    /* FreeRTOS priority */
                    &touch_task_handle)) {
#endif
            ESP_LOGE(TAG, "touch task creation failed!\n");
            ret = ESP_FAIL;
            return(ret);
        }
    }

    return(ret);
}

esp_err_t touchscreen_getCoord(uint8_t touchId, int16_t * px, int16_t * py)
{
    esp_err_t ret = ESP_OK;

    if(bInit != true) {
        ESP_LOGW(TAG, "Not initialized!\n");
        ret = ESP_ERR_INVALID_STATE;
        return (ret);
    }

    if((touchId >= MAX_TOUCH_PT) || (px == NULL) || (py == NULL)) {
        ESP_LOGW(TAG, "%s: Invalid Argument!\n", __FUNCTION__);
        ret = ESP_ERR_INVALID_ARG;
        return (ret);
    }

    if(pdTRUE != xSemaphoreTakeRecursive(mutexTouch, 100/portTICK_PERIOD_MS)) {
        ESP_LOGE(TAG, "%s:%d: Failed to get mutex!\n", __FUNCTION__, __LINE__);
        ret = ESP_FAIL;
        return (ret);
    }

    *px = touch_pt[touchId].x;
    *py = touch_pt[touchId].y;

    xSemaphoreGiveRecursive(mutexTouch);

    return (ret);
}

#endif /* CONFIG_TC_DRIVER_FT6336 */