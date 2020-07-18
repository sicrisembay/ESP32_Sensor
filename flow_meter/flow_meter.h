#ifndef _FLOW_METER_H_
#define _FLOW_METER_H_

#include "sdkconfig.h"
#if defined(CONFIG_SENSOR_FLOW_METER)
#include "esp_err.h"
#include "fix16.h"

esp_err_t flow_meter_init(void);
fix16_t flow_meter_get_rate(void);
fix16_t flow_meter_get_volume(void);
int16_t flow_meter_get_raw_count(void);
esp_err_t flow_meter_set_calibration(fix16_t flowCal);
esp_err_t flow_meter_tare_zero(void);

#endif /* defined(CONFIG_SENSOR_FLOW_METER) */

#endif /* _FLOW_METER_H_ */