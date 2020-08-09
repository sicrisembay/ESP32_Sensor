#ifndef _ICM20600_H_
#define _ICM20600_H_

#include "sdkconfig.h"
#if defined(CONFIG_SENSOR_ICM20600)

#include "esp_err.h"

esp_err_t icm20600_init(void);
bool icm20600_isInitialized(void);

#endif /* CONFIG_SENSOR_ICM20600 */

#endif /* _ICM20600_H_ */