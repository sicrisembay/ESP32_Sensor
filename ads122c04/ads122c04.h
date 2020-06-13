#ifndef _ADS122C04_H_
#define _ADS122C04_H_

#include "sdkconfig.h"
#if defined(CONFIG_SENSOR_ADS122C04)

#include "esp_err.h"

typedef void (* conversion_cb_t)(int32_t);

esp_err_t ads122c04_init(conversion_cb_t cb);
bool ads122c04_isInitialized(void);
int32_t ads122c04_getRaw(void);

#endif /* CONFIG_SENSOR_ADS122C04 */

#endif /* _ADS122C04_H_ */