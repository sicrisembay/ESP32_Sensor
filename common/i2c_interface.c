#include "stdbool.h"
#include "esp_types.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "i2c_interface.h"

#define WRITE_BIT                       I2C_MASTER_WRITE
#define READ_BIT                        I2C_MASTER_READ
#define ACK_CHECK_EN                    (0x1)
#define ACK_CHECK_DIS                   (0x0)
#define ACK_VAL                         (0x0)
#define NACK_VAL                        (0x1)

static const char* TAG = "i2c_interface";

static bool bInit[I2C_NUM_MAX] = {
        false,
        false
};

esp_err_t i2c_interface_init(i2c_port_t i2c_num, const i2c_config_t * i2c_conf)
{
    esp_err_t retval = ESP_OK;

    if((i2c_num >= I2C_NUM_MAX) || (i2c_conf == NULL)) {
        ESP_LOGE(TAG,"Invalid Arg!");
        return ESP_ERR_INVALID_ARG;
    }
    if(bInit[i2c_num] == true) {
        ESP_LOGI(TAG, "Already initialized.  Skipping initailization.");
        return ESP_OK;
    }
    retval = i2c_param_config(i2c_num, i2c_conf);
    if(retval != ESP_OK) {
        return(retval);
    }
    retval = i2c_driver_install(i2c_num, I2C_MODE_MASTER, 0, 0, 0);
    if(retval != ESP_OK) {
        return (retval);
    }
    bInit[i2c_num] = true;
    ESP_LOGI(TAG, "I2C_MASTER_%d Initialized.", i2c_num);
    return ESP_OK;
}


esp_err_t i2c_interface_write(i2c_port_t i2c_num,
                              unsigned char slave_addr,
                              unsigned char *pReg,
                              unsigned char regLen,
                              unsigned char *pData,
                              unsigned char dataLen)
{
    esp_err_t retval = ESP_OK;
    if((pReg == NULL) && (regLen != 0)) {
        ESP_LOGE(TAG, "Invalid pReg!");
        return ESP_ERR_INVALID_ARG;
    }
    if((pData == NULL) && (dataLen != 0)) {
        ESP_LOGE(TAG, "Invalid pData!");
        return ESP_ERR_INVALID_ARG;
    }
    if(i2c_num >= I2C_NUM_MAX) {
        ESP_LOGE(TAG, "Invalid I2C_NUM!");
        return ESP_ERR_INVALID_ARG;
    }
    if(!bInit[i2c_num]) {
        ESP_LOGE(TAG, "I2C%d not initialized!", i2c_num);
        return ESP_FAIL;
    }
    if((regLen == 0) && (dataLen == 0)) {
        /* Nothing to write */
        return ESP_OK;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    retval = i2c_master_start(cmd);
    if(ESP_OK != retval) {
        i2c_cmd_link_delete(cmd);
        return (retval);
    }

    retval = i2c_master_write_byte(cmd, (slave_addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    if(ESP_OK != retval) {
        i2c_cmd_link_delete(cmd);
        return (retval);
    }

    if(regLen > 0) {
        retval = i2c_master_write(cmd, pReg, regLen, ACK_CHECK_EN);
        if(ESP_OK != retval) {
            i2c_cmd_link_delete(cmd);
            return(retval);
        }
    }

    if(dataLen > 0) {
        retval = i2c_master_write(cmd, pData, dataLen, ACK_CHECK_EN);
        if(ESP_OK != retval) {
            i2c_cmd_link_delete(cmd);
            return(retval);
        }
    }

    retval = i2c_master_stop(cmd);
    if(ESP_OK != retval) {
        i2c_cmd_link_delete(cmd);
        return(retval);
    }

    retval = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    if(ESP_OK != retval) {
        i2c_cmd_link_delete(cmd);
        return(retval);
    }

    i2c_cmd_link_delete(cmd);

    return ESP_OK;
}

esp_err_t i2c_interface_read(i2c_port_t i2c_num,
                             unsigned char slave_addr,
                             unsigned char *pReg,
                             unsigned char regLen,
                             unsigned char *pData,
                             unsigned char dataLen)
{
    esp_err_t retval = ESP_OK;
    if((pReg == NULL) && (regLen != 0)) {
        ESP_LOGE(TAG, "Invalid pReg!");
        return ESP_ERR_INVALID_ARG;
    }
    if((pData == NULL) && (dataLen != 0)) {
        ESP_LOGE(TAG, "Invalid pData!");
        return ESP_ERR_INVALID_ARG;
    }
    if(i2c_num >= I2C_NUM_MAX) {
        ESP_LOGE(TAG, "Invalid I2C_NUM!");
        return ESP_ERR_INVALID_ARG;
    }
    if(!bInit[i2c_num]) {
        ESP_LOGE(TAG, "I2C%d not initialized!", i2c_num);
        return ESP_FAIL;
    }

    i2c_cmd_handle_t cmd;
    
    if(regLen > 0) {
        cmd = i2c_cmd_link_create();
        retval = i2c_master_start(cmd);
        if(ESP_OK != retval) {
            i2c_cmd_link_delete(cmd);
            return (retval);
        }

        retval = i2c_master_write_byte(cmd, (slave_addr << 1) | WRITE_BIT, ACK_CHECK_EN);
        if(ESP_OK != retval) {
            i2c_cmd_link_delete(cmd);
            return (retval);
        }

        retval = i2c_master_write(cmd, pReg, regLen, ACK_CHECK_EN);
        if(ESP_OK != retval) {
            i2c_cmd_link_delete(cmd);
            return(retval);
        }
        
        retval = i2c_master_stop(cmd);
        if(ESP_OK != retval) {
            i2c_cmd_link_delete(cmd);
            return(retval);
        }

        retval = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
        if(ESP_OK != retval) {
            i2c_cmd_link_delete(cmd);
            return(retval);
        }
        i2c_cmd_link_delete(cmd);
    }

    if(dataLen > 0) {
        cmd = i2c_cmd_link_create();

        retval = i2c_master_start(cmd);
        if(ESP_OK != retval) {
            i2c_cmd_link_delete(cmd);
            return(retval);
        }

        retval = i2c_master_write_byte(cmd, slave_addr << 1 | READ_BIT, ACK_CHECK_EN);
        if(ESP_OK != retval) {
            i2c_cmd_link_delete(cmd);
            return(retval);
        }

        if(dataLen > 1) {
            retval = i2c_master_read(cmd, pData, dataLen - 1, ACK_VAL);
            if(ESP_OK != retval) {
                i2c_cmd_link_delete(cmd);
                return (retval);
            }
        }
        retval = i2c_master_read_byte(cmd, pData + dataLen - 1, NACK_VAL);
        if(ESP_OK != retval) {
            i2c_cmd_link_delete(cmd);
            return (retval);
        }
        
        retval = i2c_master_stop(cmd);
        if(ESP_OK != retval) {
            i2c_cmd_link_delete(cmd);
            return (retval);
        }

        retval = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
        if(ESP_OK != retval) {
            i2c_cmd_link_delete(cmd);
            return (retval);
        }
        i2c_cmd_link_delete(cmd);
    }

    return ESP_OK;
}

