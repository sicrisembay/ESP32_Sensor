#ifndef _ADS122C04_H_
#define _ADS122C04_H_

#include "sdkconfig.h"
#if defined(CONFIG_SENSOR_ADS122C04)

#include "esp_err.h"

esp_err_t ads122c04_init(void);
bool ads122c04_isInitialized(void);
int32_t ads122c04_getRaw(void);

#endif /* CONFIG_SENSOR_ADS122C04 */

#endif /* _ADS122C04_H_ */