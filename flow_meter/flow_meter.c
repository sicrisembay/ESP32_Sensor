#include "sdkconfig.h"
#if defined(CONFIG_SENSOR_FLOW_METER)
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "flow_meter.h"

#define CONCAT(x,y)             x##y

#define PCNT_UNIT(num)          CONCAT(PCNT_UNIT_, num)
#define FLOW_METER_PCNT_UNIT    PCNT_UNIT(CONFIG_FLOW_METER_PCNT_UNIT)

typedef struct {
    uint32_t timeStamp;
    uint32_t prevTimeStamp;
    fix16_t q16_flowRate;
    fix16_t q16_volume;
    fix16_t q16_volPerPulse;
    int16_t count;
} flow_meter_record_t;

static const char *TAG = "flow_meter";
static flow_meter_record_t flow_meter_record;
static bool bInit = false;

esp_err_t flow_meter_init(void)
{
    pcnt_config_t pcnt_config = {
        .unit = FLOW_METER_PCNT_UNIT,
        .channel = PCNT_CHANNEL_0,
        .pulse_gpio_num = CONFIG_FLOW_METER_IO_PIN,
        .ctrl_gpio_num = PCNT_PIN_NOT_USED,
        .pos_mode = PCNT_COUNT_INC,
#if defined(CONFIG_FLOW_METER_COUNT_ON_BOTH_EDGE)
        .neg_mode = PCNT_COUNT_INC,
#else
        .neg_mode = PCNT_COUNT_DIS,
#endif /* #if defined(CONFIG_FLOW_METER_COUNT_ON_BOTH_EDGE) */
        .lctrl_mode = PCNT_MODE_KEEP,
        .hctrl_mode = PCNT_MODE_KEEP,
        .counter_h_lim = 0x7FFF,
        .counter_l_lim = 0,
    };

    if(bInit == true) {
        /* Already initialized */
        return ESP_OK;
    }
    flow_meter_record.q16_flowRate = 0;
    flow_meter_record.q16_volume = 0;
    flow_meter_record.count = 0;
#if defined(CONFIG_FLOW_METER_COUNT_ON_BOTH_EDGE)
    flow_meter_record.q16_volPerPulse = CONFIG_FLOW_METER_ML_PER_PULSE >> 1;
#else
    flow_meter_record.q16_volPerPulse = CONFIG_FLOW_METER_ML_PER_PULSE;
#endif /* FLOW_METER_ML_PER_PULSE */

    pcnt_unit_config(&pcnt_config);
    pcnt_counter_pause(FLOW_METER_PCNT_UNIT);
    pcnt_counter_clear(FLOW_METER_PCNT_UNIT);
    pcnt_counter_resume(FLOW_METER_PCNT_UNIT);

    bInit = true;

    return ESP_OK;
}

fix16_t flow_meter_get_rate(void)
{
    fix16_t ret = 0;

    return (ret);
}

fix16_t flow_meter_get_volume(void)
{
    esp_err_t ret = ESP_OK;
    if(bInit != true) {
        ESP_LOGE(TAG, "flow_meter_get_volume: Not Initialized!");
        return (-fix16_one);
    }
    ret = pcnt_get_counter_value(FLOW_METER_PCNT_UNIT, &(flow_meter_record.count));
    if(ESP_OK != ret) {
        ESP_LOGE(TAG, "flow_meter_get_volume: pcnt_get_counter_value: Error %d", ret);
        return (-fix16_one);
    }
    /* Calculate volume */
    flow_meter_record.q16_volume = flow_meter_record.q16_volPerPulse * flow_meter_record.count;
    return (flow_meter_record.q16_volume);
}

int16_t flow_meter_get_raw_count(void)
{
    esp_err_t ret = ESP_OK;
    if(bInit != true) {
        ESP_LOGE(TAG, "flow_meter_get_raw_count: Not Initialized!");
        return (-1);
    }
    ret = pcnt_get_counter_value(FLOW_METER_PCNT_UNIT, &(flow_meter_record.count));
    if(ESP_OK != ret) {
        ESP_LOGE(TAG, "flow_meter_get_raw_count: pcnt_get_counter_value: Error %d", ret);
        return (-1);
    }
    /* Calculate volume */
    flow_meter_record.q16_volume = flow_meter_record.q16_volPerPulse * flow_meter_record.count;

    return (flow_meter_record.count);
}

esp_err_t flow_meter_set_calibration(fix16_t flowCal)
{
    return ESP_OK;
}

esp_err_t flow_meter_tare_zero(void)
{
    if(bInit != true) {
        ESP_LOGE(TAG, "flow_meter_tare_zero: Not Initialized!");
        return ESP_ERR_INVALID_STATE;
    }
    flow_meter_record.q16_volume = 0;
    flow_meter_record.count = 0;
    pcnt_counter_pause(FLOW_METER_PCNT_UNIT);
    pcnt_counter_clear(FLOW_METER_PCNT_UNIT);
    pcnt_counter_resume(FLOW_METER_PCNT_UNIT);

    return ESP_OK;
}

#endif /* defined(CONFIG_SENSOR_FLOW_METER) */