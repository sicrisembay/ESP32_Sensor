#ifndef _INA219_H_
#define _INA219_H_

#include "esp_err.h"

typedef enum {
    POWER_DOWN = 0,
    SHUNT_VOLTAGE_TRIG,
    BUS_VOLTAGE_TRIG,
    SHUNT_BUS_VOLTAGE_TRIG,
    ADC_OFF,
    SHUNT_VOLTAGE_CONTINUOUS,
    BUS_VOLTAGE_CONTINUOUS,
    SHUNT_BUS_VOLTAGE_CONTINUOUS,

    N_INA219_MODE
} INA219_MODE_T;

esp_err_t ina219_init(void);
bool ina219_isInitialized(void);
float ina219_get_bus_voltage(uint8_t devId, esp_err_t * pErr);
float ina219_get_shunt_voltage(uint8_t devId, esp_err_t * pErr);
float ina219_get_current(uint8_t devId, esp_err_t * pErr);
float ina219_get_power(uint8_t devId, esp_err_t * pErr);
esp_err_t ina219_set_mode(uint8_t devId, INA219_MODE_T mode);
esp_err_t ina219_get_mode(uint8_t devId, INA219_MODE_T * pMode);

#endif /* _INA219_H_ */
