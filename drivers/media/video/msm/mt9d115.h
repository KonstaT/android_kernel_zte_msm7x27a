/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


/*
[SENSOR]
Sensor Model:   MT9D115
Camera Module:
Lens Model:
Driver IC:
PV Size         = 640 x 480
Cap Size        = 2592 x 1944
Output Format   = YUYV
MCLK Speed      = 24M
PV DVP_PCLK     = 28M
Cap DVP_PCLK    = 56M
PV Frame Rate   = 30fps
Cap Frame Rate  = 7.5fps
I2C Slave ID    = 0x78
I2C Mode        = 16Addr, 8Data
*/
#ifndef MT9D115_H
#define MT9D115_H

#include <mach/board.h>
#include <mach/camera.h>
#define MT9D115_SENSOR_ID 0x2580

extern struct mt9d115_reg_t mt9d115_regs;

enum mt9d115_width_t {
    WORD_LEN,
    BYTE_LEN
};

struct mt9d115_i2c_reg_conf {
    unsigned short waddr;
    unsigned short wdata;
    enum mt9d115_width_t width;
    unsigned short mdelay_time;
};

struct mt9d115_reg_t {
    struct mt9d115_i2c_reg_conf const *pll_tbl;
    uint16_t pll_tbl_sz;

    struct mt9d115_i2c_reg_conf const *clk_tbl;
    uint16_t clk_tbl_sz;

    struct mt9d115_i2c_reg_conf const *prevsnap_tbl;
    uint16_t prevsnap_tbl_sz;

    struct mt9d115_i2c_reg_conf const *wb_cloudy_tbl;
    uint16_t wb_cloudy_tbl_sz;

    struct mt9d115_i2c_reg_conf const *wb_daylight_tbl;
    uint16_t wb_daylight_tbl_sz;

    struct mt9d115_i2c_reg_conf const *wb_flourescant_tbl;
    uint16_t wb_flourescant_tbl_sz;

    struct mt9d115_i2c_reg_conf const *wb_incandescent_tbl;
    uint16_t wb_incandescent_tbl_sz;

    struct mt9d115_i2c_reg_conf const *wb_auto_tbl;
    uint16_t wb_auto_tbl_sz;

    struct mt9d115_i2c_reg_conf const *af_tbl;
    uint16_t af_tbl_sz;

    struct mt9d115_i2c_reg_conf const **contrast_tbl;
    uint16_t const *contrast_tbl_sz;

    struct mt9d115_i2c_reg_conf const **brightness_tbl;
    uint16_t const *brightness_tbl_sz;

    struct mt9d115_i2c_reg_conf const **saturation_tbl;
    uint16_t const *saturation_tbl_sz;

    struct mt9d115_i2c_reg_conf const **sharpness_tbl;
    uint16_t const *sharpness_tbl_sz;

    struct mt9d115_i2c_reg_conf const *lens_for_outdoor_tbl;
    uint16_t const lens_for_outdoor_tbl_sz;

    struct mt9d115_i2c_reg_conf const *lens_for_indoor_tbl;
    uint16_t const lens_for_indoor_tbl_sz;
    struct mt9d115_i2c_reg_conf const **brightness_exposure_tbl;
    uint16_t const *brightness_exposure_tbl_sz;
};

#endif /* MT9D115_H */


#if 0
#ifndef CAMSENSOR_MT9D115
#define CAMSENSOR_MT9D115

#define INVMASK(v)  (0xff-v)
//Auto Focus Command
#define MT9D115_CMD_MAIN 0x3022
#define MT9D115_CMD_ACK 0x3023
#define MT9D115_CMD_PARA0 0x3024
#define MT9D115_CMD_PARA1 0x3025
#define MT9D115_CMD_PARA2 0x3026
#define MT9D115_CMD_PARA3 0x3027
#define MT9D115_CMD_PARA4 0x3028
#define MT9D115_CMD_FW_STATUS 0x3029

//Sensor ID
#define MT9D115_SENSOR_ID 0x2580

#define Capture_Framerate 750     //7.5fps capture frame rate
#define g_Preview_FrameRate 3000  //30fps preview frame rate
#endif
#endif
