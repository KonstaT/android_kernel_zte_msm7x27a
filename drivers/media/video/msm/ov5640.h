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
Sensor Model:   OV5640
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

#ifndef CAMSENSOR_OV5640
#define CAMSENSOR_OV5640
#include <linux/types.h>
#include <mach/board.h>
#include <mach/camera.h>


#define INVMASK(v)  (0xff-v)
//#define OV5640Core_WritePREG(pTbl)     OV5640_WritePRegs(pTbl,sizeof(pTbl)/sizeof(pTbl[0]))


/*OV SENSOR SCCB*/
 struct OV5640_WREG{
	uint16_t addr;
	uint16_t data;
	uint8_t mask;
};

enum ov5640_width_t {
    WORD_LEN,
    BYTE_LEN
};


//Auto Focus Command
#define OV5640_CMD_MAIN 0x3022
#define OV5640_CMD_ACK 0x3023
#define OV5640_CMD_PARA0 0x3024
#define OV5640_CMD_PARA1 0x3025
#define OV5640_CMD_PARA2 0x3026
#define OV5640_CMD_PARA3 0x3027
#define OV5640_CMD_PARA4 0x3028
#define OV5640_CMD_FW_STATUS 0x3029

//Sensor ID
#define OV5640_SENSOR_ID 0x5640

#define Capture_Framerate 750     //7.5fps capture frame rate
#define g_Preview_FrameRate 3000  //30fps preview frame rate

extern struct ov5640_reg_t ov5640_regs;


struct ov5640_reg_t {

     struct OV5640_WREG const *init_tbl;    
     uint16_t init_tbl_size;
    
     struct OV5640_WREG const *init_iq_tbl;    
     uint16_t init_iq_tbl_size;
    
     struct OV5640_WREG const *preview_tbl;
     uint16_t preview_tbl_size;

     struct OV5640_WREG const *capture_tbl;
     uint16_t capture_tbl_size;

     struct OV5640_WREG const *exposure_compensation_lv0_tbl;
     uint16_t exposure_compensation_lv0_tbl_size;
     
     struct OV5640_WREG const *exposure_compensation_lv1_tbl;
     uint16_t exposure_compensation_lv1_tbl_size;
     
     struct OV5640_WREG const *exposure_compensation_lv2_tbl;
     uint16_t exposure_compensation_lv2_tbl_size;
     
     struct OV5640_WREG const *exposure_compensation_lv3_tbl;
     uint16_t exposure_compensation_lv3_tbl_size;
     
     struct OV5640_WREG const *exposure_compensation_lv4_tbl;
     uint16_t exposure_compensation_lv4_tbl_size;
    
     struct OV5640_WREG const *ae_average_tbl;
     uint16_t ae_average_tbl_size;
     
     struct OV5640_WREG const *ae_centerweight_tbl;
     uint16_t ae_centerweight_tbl_size;
     
     struct OV5640_WREG const *lens_shading_on_tbl;
     uint16_t lens_shading_on_tbl_size;
     
     struct OV5640_WREG const *lens_shading_off_tbl;
     uint16_t lens_shading_off_tbl_size;
     
     struct OV5640_WREG const *afinit_tbl;
     uint16_t afinit_tbl_size;

     struct OV5640_WREG const *lens_shading_D65_tbl ;
	uint16_t  lens_shading_D65_tbl_size;
	
	struct OV5640_WREG const * lens_shading_A_tbl ;
	uint16_t   lens_shading_A_tbl_size ;
};

#endif /* CAMSENSOR_OV5640 */
