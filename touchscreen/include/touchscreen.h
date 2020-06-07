/*
 * (C) Copyright 2020
 * Sicris Rey Embay, sicris.embay@gmail.com
 */

#ifndef COMPONENT_TOUCHSCREEN_H
#define COMPONENT_TOUCHSCREEN_H

#include "sdkconfig.h"
#ifdef CONFIG_SENSOR_TOUCHSCREEN
#include "esp_err.h"

esp_err_t touchscreen_init(void);
esp_err_t touchscreen_getCoord(uint8_t touchId, int16_t * px, int16_t * py);


#endif /* CONFIG_SENSOR_TOUCHSCREEN */
#endif /* COMPONENT_TOUCHSCREEN_H */