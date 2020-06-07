#ifndef _I2C_INTERFACE_H_
#define _I2C_INTERFACE_H_

#include "esp_err.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

typedef enum {
#if (CONFIG_SENSOR_I2C_BUS_COUNT >= 1)
    I2C_BUS0 = 0,
#if (CONFIG_SENSOR_I2C_BUS_COUNT >= 2)
    I2C_BUS1,
#endif
#endif
    N_I2C_BUS
} i2c_bus_id_t;

esp_err_t i2c_interface_init(void);
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
