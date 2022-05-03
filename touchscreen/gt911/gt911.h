/*
 * (C) Copyright 2022
 * Sicris Rey Embay, sicris.embay@gmail.com
 */

#ifndef TOUCH_GT911_H
#define TOUCH_GT911_H

#include "sdkconfig.h"
#if defined(CONFIG_TC_DRIVER_GT911)

#define MAX_TOUCHES     5

#define GT911_I2C_ADDRESS                   (0x5d) /* Refer to Section 4.2 for proper address setting */

#define GT911_REG_COMMAND                   (0x8040)
#define GT911_REG_CONFIG_VERSION            (0x8047)
#define GT911_REG_CONFIG_FRESH              (0x8100)
#define GT911_REG_PRODUCT_ID_FIRST_BYTE     (0x8140)
#define GT911_REG_STATUS                    (0x814E)
#define GT911_REG_PT1_INFO                  (0x814F)
#define GT911_REG_CMD_STATUS                (0x81A8)

typedef union {
    uint8_t buf;
    struct {
        uint8_t number_of_touch_points : 4;
        uint8_t have_key               : 1;
        uint8_t reserved               : 1;
        uint8_t large_detect           : 1;
        uint8_t buffer_status          : 1;
    };
} touch_status_t;

typedef union {
    uint8_t buf[8];
    struct __attribute__((packed, aligned(1))) {
        uint8_t track_id;
        int16_t x;
        int16_t y;
        uint16_t point_size;
        uint8_t reserved;
    };
} touch_packet_t;

typedef union {
    uint8_t buf[185];
    struct __attribute__((packed, aligned(1))) {
        uint8_t config_version;                             // 0x8047
        int16_t x_output_max;                               // 0x8048
        int16_t y_output_max;                               // 0x804A
        uint8_t touch_number;                               // 0x804C
        union {
            uint8_t buf;
            struct {
                uint8_t int_trig_mechanism  : 2;
                uint8_t sito                : 1;
                uint8_t x2y                 : 1;
                uint8_t stretch_rank        : 2;
                uint8_t sensor_reversal     : 1;
                uint8_t driver_reversal     : 1;
            };
        } module_switch1;                                   // 0x804D
        union {
            uint8_t buf;
            struct {
                uint8_t touch_key           : 1;
                uint8_t hotknot_en          : 1;
                uint8_t approach_en         : 1;
                uint8_t reserved1           : 2;
                uint8_t firstfilter_dis     : 1;
                uint8_t reserved2           : 2;
            };
        } module_switch2;                                   // 0x804E
        union {
            uint8_t buf;
            struct {
                uint8_t dejitterTouchPress  : 4;
                uint8_t dejitterTouchRel    : 4;
            };
        } shake_count;                                      // 0x804F
        union {
            uint8_t buf;
            struct {
                uint8_t normal_filter       : 6;
                uint8_t first_filter        : 2;
            };
        } filter;                                           // 0x8050
        uint8_t large_touch;                                // 0x8051
        uint8_t noise_reduction;                            // 0x8052
        uint8_t screen_touch_level;                         // 0x8053
        uint8_t screen_leave_level;                         // 0x8054
        uint8_t low_power_control;                          // 0x8055
        union {
            uint8_t buf;
            struct {
                uint8_t coordinate_report_rate : 4;
                uint8_t pulse_width_gesture_wake : 4;
            };
        } refresh_rate;                                     // 0x8056
        uint8_t x_threshold;                                // 0x8057
        uint8_t y_threshold;                                // 0x8058
        uint8_t x_speed_limit;                              // 0x8059
        uint8_t y_speed_limit;                              // 0x805A
        union {
            uint16_t buf;
            struct {
                uint16_t space_border_bottom : 4;
                uint16_t space_border_top    : 4;
                uint16_t space_border_right  : 4;
                uint16_t space_border_left   : 4;
            };
        } space;                                            // 0x805B
        uint8_t mini_filter;                                // 0x805D
        uint8_t stretch_r0;                                 // 0x805E
        uint8_t stretch_r1;                                 // 0x805F
        uint8_t stretch_r2;                                 // 0x8060
        uint8_t stretch_rm;                                 // 0x8061
        uint8_t drv_grpA;                                   // 0x8062
        uint8_t drv_grpB;                                   // 0x8063
        uint8_t sensor_num;                                 // 0x8064
        uint8_t freqA_factor;                               // 0x8065
        uint8_t freqB_factor;                               // 0x8066
        uint16_t panel_bit_freq;                            // 0x8067
        uint16_t panel_sensor_time;                         // 0x8069
        uint8_t panel_tx_gain;                              // 0x806B
        uint8_t panel_rx_gain;                              // 0x806C
        uint8_t panel_dump_shift;                           // 0x806D
        uint8_t drv_frame_control;                          // 0x806E
        uint8_t charging_level_up;                          // 0x806F
        uint8_t module_switch3;                             // 0x8070
        uint8_t gesture_dis;                                // 0x8071
        uint8_t gesture_long_press_time;                    // 0x8072
        uint8_t x_y_slope_adjust;                           // 0x8073
        uint8_t gesture_control;                            // 0x8074
        uint8_t gesture_switch1;                            // 0x8075
        uint8_t gesture_switch2;                            // 0x8076
        uint8_t gesture_refresh_rate;                       // 0x8077
        uint8_t gesture_touch_level;                        // 0x8078
        uint8_t new_green_wake_up_level;                    // 0x8079
        uint8_t freq_hop_start;                             // 0x807A
        uint8_t freq_hop_end;                               // 0x807B
        uint8_t noise_detect_time;                          // 0x807C
        uint8_t hopping_flag;                               // 0x807D
        uint8_t hopping_threshold;                          // 0x807E
        uint8_t noise_threshold;                            // 0x807F
        uint8_t define_later1[55];                          // 0x8080 - 0x80B6
        uint8_t sensor_chX[14];                             // 0x80B7 - 0x80C4
        uint8_t reserved_1[16];                             // 0x80C5 - 0x80D4
        uint8_t driver_chX[26];                             // 0x80D5 - 0x80EE
        uint8_t reserved_2[16];                             // 0x80EF - 0x80FE
        uint8_t config_checksum;                            // 0x80FF
    };
} touch_configuration_info_t;

typedef union {
    uint8_t buf[11];
    struct __attribute__((packed, aligned(1))) {
        uint8_t prod_id_first_byte;
        uint8_t prod_id_second_byte;
        uint8_t prod_id_third_byte;
        uint8_t prod_id_fourth_byte;
        uint16_t firmware_version;
        uint16_t x_coord_resolution;
        uint16_t y_coord_resolution;
        uint8_t vendor_id;
    };
} touch_coordinate_info_t;

typedef union {
    uint8_t buf[2];
    struct __attribute__((packed, aligned(1))) {
        uint8_t gt911_status;
        uint8_t gt911_status_bak;
    };
} touch_mode_status_t;

#endif /* CONFIG_TC_DRIVER_GT911 */
#endif /* TOUCH_GT911_H */