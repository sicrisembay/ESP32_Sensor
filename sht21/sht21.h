#ifndef _SHT21_H_
#define _SHT21_H_

#include "esp_err.h"

esp_err_t sht21_init(void);
bool sht21_isInitialized(void);
float sht21_get_humidity(esp_err_t * pErr);
float sht21_get_temperature(esp_err_t * pErr);

#endif /* _SHT21_H_ */
