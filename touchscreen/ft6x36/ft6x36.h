/*
 * (C) Copyright 2020
 * Sicris Rey Embay, sicris.embay@gmail.com
 */

#ifndef TOUCH_FT6X36_H
#define TOUCH_FT6X36_H

#include "sdkconfig.h"
#ifdef CONFIG_TC_DRIVER_FT6336

#define FT6X36_REG_DEV_MODE         (0x00)
#define FT6X36_REG_TD_STATUS        (0x02)
#define FT6X36_REG_P1_XH            (0x03)
#define FT6X36_REG_P1_XL            (0x04)
#define FT6X36_REG_P1_YH            (0x05)
#define FT6X36_REG_P1_YL            (0x06)
#define FT6X36_REG_P2_XH            (0x09)
#define FT6X36_REG_P2_XL            (0x0A)
#define FT6X36_REG_P2_YH            (0x0B)
#define FT6X36_REG_P2_YL            (0x0C)
#define FT6X36_REG_LIB_VER_H        (0xA1)
#define FT6X36_REG_LIB_VER_L        (0xA2)
#define FT6X36_REG_FIRMID           (0xA6)
#define FT6X36_REG_FOCALTECH_ID     (0xA8)
#define FT6x36_REG_REL_CODE_ID      (0xAF)

#define FOCALTECH_ID                (0x11)

#define MAX_TOUCH_PT                (0x02)

typedef enum {
    FT6X36_EVT_PRESS_DOWN = 0,
    FT6X36_EVT_LIFT_UP,
    FT6X36_EVT_CONTACT,
    FT6X36_EVT_NONE
} ft6x36_evt_t;

typedef struct {
    ft6x36_evt_t evtFlag;
    int16_t x;
    int16_t y;
} touch_pt_t;

typedef struct {
    union {
        uint8_t regBuf[189];
        struct {
            uint8_t dev_mode;       // 0x00
            uint8_t gest_id;        // 0x01
            uint8_t td_status;      // 0x02
            uint8_t p1_xh;          // 0x03
            uint8_t p1_xl;          // 0x04
            uint8_t p1_yh;          // 0x05
            uint8_t p1_yl;          // 0x06
            uint8_t p1_weight;      // 0x07
            uint8_t p1_misc;        // 0x08
            uint8_t p2_xh;          // 0x09
            uint8_t p2_xl;          // 0x0A
            uint8_t p2_yh;          // 0x0B
            uint8_t p2_yl;          // 0x0C
            uint8_t p2_weight;      // 0x0D
            uint8_t p2_misc;        // 0x0E
            uint8_t reserved1[113]; // 0x0F - 0x7F
            uint8_t th_group;       // 0x80
            uint8_t reserved2[4];   // 0x81-0x84
            uint8_t th_diff;        // 0x85
            uint8_t ctrl;           // 0x86
            uint8_t timeEnterMon;   // 0x87
            uint8_t period_active;  // 0x88
            uint8_t period_monitor; // 0x89
            uint8_t reserved3[7];   // 0x8A-0x90
            uint8_t radian_value;   // 0x91
            uint8_t offset_l_r;     // 0x92
            uint8_t offset_u_d;     // 0x93
            uint8_t dist_l_r;       // 0x94
            uint8_t dist_u_d;       // 0x95
            uint8_t dist_zoom;      // 0x96
            uint8_t reserved4[10];  // 0x97-0xA0
            uint8_t lib_ver_h;      // 0xA1
            uint8_t lib_ver_l;      // 0xA2
            uint8_t cipher;         // 0xA3
            uint8_t g_mode;         // 0xA4
            uint8_t pwr_mode;       // 0xA5
            uint8_t firm_id;        // 0xA6
            uint8_t reserved5;      // 0xA7
            uint8_t focaltech_id;   // 0xA8
            uint8_t reserved6[6];   // 0xA9-0xAE
            uint8_t rel_code_id;    // 0xAF
            uint8_t reserved7[12];  // 0xB0-0xBB
            uint8_t state;          // 0xBC
        };
    };
} ft6x36_reg_t;

#endif /* CONFIG_TC_DRIVER_FT6336 */
#endif /* TOUCH_FT6X36_H */