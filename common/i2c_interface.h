#ifndef _I2C_INTERFACE_H_
#define _I2C_INTERFACE_H_

#include "esp_err.h"
#include "driver/i2c.h"

esp_err_t i2c_interface_init(i2c_port_t i2c_num, const i2c_config_t * i2c_conf);
esp_err_t i2c_interface_write(i2c_port_t i2c_num,
                              unsigned char slave_addr,
                              unsigned char *pReg,
                              unsigned char regLen,
                              unsigned char *pData,
                              unsigned char dataLen);
esp_err_t i2c_interface_read(i2c_port_t i2c_num,
                             unsigned char slave_addr,
                             unsigned char *pReg,
                             unsigned char regLen,
                             unsigned char *pData,
                             unsigned char dataLen);

#endif /* _I2C_INTERFACE_H_ */
