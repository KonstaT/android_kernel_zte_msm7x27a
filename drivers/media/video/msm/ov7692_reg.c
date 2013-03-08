/*
 * drivers/media/video/msm/ov7692_reg_qtech_sunny_fpc.c
 *
 * Refer to drivers/media/video/msm/mt9d112_reg.c
 * For OV7692: 0.3Mp, 1/4-Inch System-On-A-Chip (SOC) CMOS Digital Image Sensor
 *
 * Copyright (C) 2009-2010 ZTE Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Created by jia.jia@zte.com.cn
 */
/* PLL Setup */

#include "ov7692.h"
static struct ov7692_i2c_reg_conf const preview_snapshot_mode_reg_settings_array[] = {
   
    {0x12, 0x80,BYTE_LEN,0},
    {0x0e, 0x08,BYTE_LEN,0},
    {0x69, 0x52,BYTE_LEN,0},
    {0x1e, 0xb3,BYTE_LEN,0},
    {0x48, 0x42,BYTE_LEN,0},
    {0xff, 0x01,BYTE_LEN,0},
    {0xae, 0xa0,BYTE_LEN,0},
    {0xa8, 0x26,BYTE_LEN,0},
    {0xb4, 0xc0,BYTE_LEN,0},
    {0xb5, 0x40,BYTE_LEN,0},
    {0xff, 0x00,BYTE_LEN,0},
    {0x0c, 0x00,BYTE_LEN,0}, //0x00    [6] mirror enable  [7] filp enable
    {0x62, 0x10,BYTE_LEN,0},
    {0x12, 0x00,BYTE_LEN,0},
    {0x17, 0x65,BYTE_LEN,0},
    {0x18, 0xa4,BYTE_LEN,0},
    {0x19, 0x0a,BYTE_LEN,0},
    {0x1a, 0xf6,BYTE_LEN,0},
    {0x3e, 0x30,BYTE_LEN,0},
    {0x64, 0x0a,BYTE_LEN,0},
    {0xff, 0x01,BYTE_LEN,0},
    {0xb4, 0xc0,BYTE_LEN,0},
    {0xff, 0x00,BYTE_LEN,0},
    {0x67, 0x20,BYTE_LEN,0},
    {0x81, 0x3f,BYTE_LEN,0},
    {0xcc, 0x02,BYTE_LEN,0},
    {0xcd, 0x80,BYTE_LEN,0},
    {0xce, 0x01,BYTE_LEN,0},
    {0xcf, 0xe0,BYTE_LEN,0},
    {0xc8, 0x02,BYTE_LEN,0},
    {0xc9, 0x80,BYTE_LEN,0},
    {0xca, 0x01,BYTE_LEN,0},
    {0xcb, 0xe0,BYTE_LEN,0},
    {0xd0, 0x48,BYTE_LEN,0},
    {0x82, 0x03,BYTE_LEN,0},
    {0x0e, 0x00,BYTE_LEN,0},
    {0x70, 0x00,BYTE_LEN,0},
    {0x71, 0x34,BYTE_LEN,0},
    {0x74, 0x28,BYTE_LEN,0},
    {0x75, 0x98,BYTE_LEN,0},
    {0x76, 0x00,BYTE_LEN,0},
    {0x77, 0x64,BYTE_LEN,0},
    {0x78, 0x01,BYTE_LEN,0},
    {0x79, 0xc2,BYTE_LEN,0},
    {0x7a, 0x4e,BYTE_LEN,0},
    {0x7b, 0x1f,BYTE_LEN,0},
    {0x7c, 0x00,BYTE_LEN,0},
    {0x11, 0x00,BYTE_LEN,0},
    {0x20, 0x00,BYTE_LEN,0},
    {0x21, 0x23,BYTE_LEN,0},
    {0x50, 0x9a,BYTE_LEN,0},
    {0x51, 0x80,BYTE_LEN,0},
    {0x4c, 0x7d,BYTE_LEN,0},
    {0x0e, 0x00,BYTE_LEN,0},
    {0x80, 0x7f,BYTE_LEN,0},
    {0x85, 0x10,BYTE_LEN,0},
    {0x86, 0x00,BYTE_LEN,0},
    {0x87, 0x00,BYTE_LEN,0},
    {0x88, 0x00,BYTE_LEN,0},
    {0x89, 0x2a,BYTE_LEN,0},
    {0x8a, 0x26,BYTE_LEN,0},
    {0x8b, 0x22,BYTE_LEN,0},
    {0xbb, 0x7a,BYTE_LEN,0},
    {0xbc, 0x69,BYTE_LEN,0},
    {0xbd, 0x11,BYTE_LEN,0},
    {0xbe, 0x13,BYTE_LEN,0},
    {0xbf, 0x81,BYTE_LEN,0},
    {0xc0, 0x96,BYTE_LEN,0},
    {0xc1, 0x1e,BYTE_LEN,0},
    {0xb7, 0x05,BYTE_LEN,0},
    {0xb8, 0x09,BYTE_LEN,0},
    {0xb9, 0x00,BYTE_LEN,0},
    {0xba, 0x18,BYTE_LEN,0},
    {0x5a, 0x1f,BYTE_LEN,0},
    {0x5b, 0x9f,BYTE_LEN,0},
    {0x5c, 0x6a,BYTE_LEN,0},
    {0x5d, 0x42,BYTE_LEN,0},
    {0xa3, 0x0b,BYTE_LEN,0},
    {0xa4, 0x15,BYTE_LEN,0},
    {0xa5, 0x2a,BYTE_LEN,0},
    {0xa6, 0x51,BYTE_LEN,0},
    {0xa7, 0x63,BYTE_LEN,0},
    {0xa8, 0x74,BYTE_LEN,0},
    {0xa9, 0x83,BYTE_LEN,0},
    {0xaa, 0x91,BYTE_LEN,0},
    {0xab, 0x9e,BYTE_LEN,0},
    {0xac, 0xaa,BYTE_LEN,0},
    {0xad, 0xbe,BYTE_LEN,0},
    {0xae, 0xce,BYTE_LEN,0},
    {0xaf, 0xe5,BYTE_LEN,0},
    {0xb0, 0xf3,BYTE_LEN,0},
    {0xb1, 0xfb,BYTE_LEN,0},
    {0xb2, 0x06,BYTE_LEN,0},
    {0x8c, 0x5c,BYTE_LEN,0},
    {0x8d, 0x11,BYTE_LEN,0},
    {0x8e, 0x12,BYTE_LEN,0},
    {0x8f, 0x19,BYTE_LEN,0},
    {0x90, 0x50,BYTE_LEN,0},
    {0x91, 0x20,BYTE_LEN,0},
    {0x92, 0x96,BYTE_LEN,0},
    {0x93, 0x80,BYTE_LEN,0},
    {0x94, 0x13,BYTE_LEN,0},
    {0x95, 0x1b,BYTE_LEN,0},
    {0x96, 0xff,BYTE_LEN,0},
    {0x97, 0x00,BYTE_LEN,0},
    {0x98, 0x3d,BYTE_LEN,0},
    {0x99, 0x36,BYTE_LEN,0},
    {0x9a, 0x51,BYTE_LEN,0},
    {0x9b, 0x43,BYTE_LEN,0},
    {0x9c, 0xf0,BYTE_LEN,0},
    {0x9d, 0xf0,BYTE_LEN,0},
    {0x9e, 0xf0,BYTE_LEN,0},
    {0x9f, 0xff,BYTE_LEN,0},
    {0xa0, 0x68,BYTE_LEN,0},
    {0xa1, 0x62,BYTE_LEN,0},
    {0xa2, 0x0e,BYTE_LEN,0},
    //===Lens Correction==;;  
    {0x80, 0x7F,BYTE_LEN,0},  //control        
    {0x85, 0x10,BYTE_LEN,0},  //control        
    {0x86, 0x10,BYTE_LEN,0},  //radius         
    {0x87, 0x10,BYTE_LEN,0},  //X              
    {0x88, 0x80,BYTE_LEN,0},  //Y              
    {0x89, 0x2a,BYTE_LEN,0},  //R              
    {0x8a, 0x25,BYTE_LEN,0},  //G              
    {0x8b, 0x25,BYTE_LEN,0},  //B              
    //;;====Color Matrix====;;  
    {0xbb, 0xac,BYTE_LEN,0}, //D7              
    {0xbc, 0xae,BYTE_LEN,0}, //DA              
    {0xbd, 0x02,BYTE_LEN,0}, //03              
    {0xbe, 0x1f,BYTE_LEN,0}, //27              
    {0xbf, 0x93,BYTE_LEN,0}, //B8              
    {0xc0, 0xb1,BYTE_LEN,0}, //DE              
    {0xc1, 0x1A,BYTE_LEN,0},                  
    //===Edge + Denoise====;; 
    {0xb4, 0x06,BYTE_LEN,0},                  
    {0xb5, 0x05,BYTE_LEN,0}, //auto, no meaning
    {0xb6, 0x00,BYTE_LEN,0}, //auto, no meaning
    {0xb7, 0x00,BYTE_LEN,0},                  
    {0xb8, 0x06,BYTE_LEN,0},                  
    {0xb9, 0x02,BYTE_LEN,0},                  
    {0xba, 0x78,BYTE_LEN,0},                  
     //====AEC/AGC target====;;
	/*change AE adjust speed,ZTE_CAM_YGL_20111215*/
    {0x00, 0x40,BYTE_LEN,0},     //manual gain               
    {0x10, 0x80,BYTE_LEN,0},     //manual exposure
    {0x13, 0xf7,BYTE_LEN,0},
    {0x24, 0x88,BYTE_LEN,0},  //0x94,ZTE_YGL_20120105,keep the same as default exposure setting                
    {0x25, 0x78,BYTE_LEN,0},  //0x80                
    {0x26, 0xB5,BYTE_LEN,0},                  
    //=====UV adjust======;;  
    {0x81, 0xff,BYTE_LEN,0},                  
    {0x5A, 0x10,BYTE_LEN,0},                  
    {0x5B, 0xA1,BYTE_LEN,0},                  
    {0x5C, 0x3A,BYTE_LEN,0},                  
    {0x5d, 0x20,BYTE_LEN,0},                  
    //====Gamma====;;         
    #if 0
    {0xa3, 0x05,BYTE_LEN,0},                  
    {0xa4, 0x10,BYTE_LEN,0},                  
    {0xa5, 0x25,BYTE_LEN,0},                  
    {0xa6, 0x46,BYTE_LEN,0},                  
    {0xa7, 0x57,BYTE_LEN,0},                  
    {0xa8, 0x64,BYTE_LEN,0},                  
    {0xa9, 0x70,BYTE_LEN,0},                  
    {0xaa, 0x7c,BYTE_LEN,0},                  
    {0xab, 0x87,BYTE_LEN,0},                  
    {0xac, 0x90,BYTE_LEN,0},                  
    {0xad, 0x9f,BYTE_LEN,0},                  
    {0xae, 0xac,BYTE_LEN,0},                  
    {0xaf, 0xc1,BYTE_LEN,0},                  
    {0xb0, 0xd5,BYTE_LEN,0},                  
    {0xb1, 0xe7,BYTE_LEN,0},                  
    {0xb2, 0x21,BYTE_LEN,0},                  
    #else
    {0xa3, 0x10,BYTE_LEN,0},                  
    {0xa4, 0x1c,BYTE_LEN,0},                  
    {0xa5, 0x30,BYTE_LEN,0},                  
    {0xa6, 0x58,BYTE_LEN,0},                  
    {0xa7, 0x68,BYTE_LEN,0},                  
    {0xa8, 0x76,BYTE_LEN,0},                  
    {0xa9, 0x81,BYTE_LEN,0},                  
    {0xaa, 0x8a,BYTE_LEN,0},                  
    {0xab, 0x92,BYTE_LEN,0},                  
    {0xac, 0x98,BYTE_LEN,0},                  
    {0xad, 0xa4,BYTE_LEN,0},                  
    {0xae, 0xb1,BYTE_LEN,0},                  
    {0xaf, 0xc5,BYTE_LEN,0},                  
    {0xb0, 0xd7,BYTE_LEN,0},                  
    {0xb1, 0xe8,BYTE_LEN,0},                  
    {0xb2, 0x20,BYTE_LEN,0},   
    #endif
    //==Advance==;;           
    {0x8c, 0x52,BYTE_LEN,0},                  
    {0x8d, 0x11,BYTE_LEN,0},                  
    {0x8e, 0x12,BYTE_LEN,0},                  
    {0x8f, 0x19,BYTE_LEN,0},                  
    {0x90, 0x50,BYTE_LEN,0},                  
    {0x91, 0x20,BYTE_LEN,0},                  
    {0x92, 0xb1,BYTE_LEN,0},                  
    {0x93, 0x9a,BYTE_LEN,0},                  
    {0x94, 0x0c,BYTE_LEN,0},                   
    {0x95, 0x0c,BYTE_LEN,0},                   
    {0x96, 0xf0,BYTE_LEN,0},                  
    {0x97, 0x10,BYTE_LEN,0},                  
    {0x98, 0x61,BYTE_LEN,0},                  
    {0x99, 0x63,BYTE_LEN,0},                  
    {0x9a, 0x71,BYTE_LEN,0},                  
    {0x9b, 0x78,BYTE_LEN,0},                  
    {0x9c, 0xf0,BYTE_LEN,0},                  
    {0x9d, 0xf0,BYTE_LEN,0},                  
    {0x9e, 0xf0,BYTE_LEN,0},                  
    {0x9f, 0xff,BYTE_LEN,0},                  
    {0xa0, 0xa7,BYTE_LEN,0},                  
    {0xa1, 0xb0,BYTE_LEN,0},                  
    {0xa2, 0x0f,BYTE_LEN,0}, 
    {0x31, 0x87,BYTE_LEN,0}, //0x87 for 15fps ; 0x83 for 30fps
    
    {0xd2, 0x02,BYTE_LEN,0},  //enable saturation       
	         
    {0xd8, 0x38,BYTE_LEN,0}, //saturation                 
    {0xd9, 0x38,BYTE_LEN,0},     

    {0xb4,0x26, BYTE_LEN, 0},
	{0xb6,0x08, BYTE_LEN, 0},
};

static struct ov7692_i2c_reg_conf const clk_tbl[] = {
  
};

static struct ov7692_i2c_reg_conf const prev_snap_tbl[] = {
  
};


static struct ov7692_i2c_reg_conf const antibanding_50z_tbl[] = {
  //Band 50Hz   
	 {0x14,0x21,BYTE_LEN,0},
	 {0x50,0x9a,BYTE_LEN,0},
};

/* Preview and Snapshot Setup */
static struct ov7692_i2c_reg_conf const antibanding_60z_tbl[] ={
  //Band 60Hz   
	{0x14,0x20,BYTE_LEN,0},
	{0x51,0x80,BYTE_LEN,0},
} ;


static struct ov7692_i2c_reg_conf const effect_off_tbl[] = {
        #if 0
		{0xbe,0x26,BYTE_LEN,0},
		{0xbf,0x7b,BYTE_LEN,0},
		{0xc0,0xac,BYTE_LEN,0},
		{0xbb,0x80,BYTE_LEN,0},
		{0xbc,0x62,BYTE_LEN,0},
		{0xbd,0x1e,BYTE_LEN,0},
		{0xc1,0x1e,BYTE_LEN,0},
		{0x28,0x00,BYTE_LEN,0},
		#else
		{0x81,0xff,BYTE_LEN,0},
		{0xd2,0x06,BYTE_LEN,0},
		{0xda,0x80,BYTE_LEN,0},
		{0xdb,0x80,BYTE_LEN,0},		
		#endif

};

static struct ov7692_i2c_reg_conf const effect_mono_tbl[] = {
        #if 0
		{0xbe,0x00,BYTE_LEN,0},
		{0xbf,0x00,BYTE_LEN,0},
		{0xc0,0x00,BYTE_LEN,0},
		{0xbb,0x00,BYTE_LEN,0},
		{0xbc,0x00,BYTE_LEN,0},
		{0xbd,0x00,BYTE_LEN,0},
		{0xc1,0x00,BYTE_LEN,0},
		{0x28,0x00,BYTE_LEN,0},
		#else
		{0x81,0xff,BYTE_LEN,0},
		{0xd2,0x1e,BYTE_LEN,0},
		{0xda,0x80,BYTE_LEN,0},
		{0xdb,0x80,BYTE_LEN,0},		
		#endif

};

static struct ov7692_i2c_reg_conf const effect_negative_tbl[] = {
        #if 0
		{0xbe,0x26,BYTE_LEN,0},
		{0xbf,0x7b,BYTE_LEN,0},
		{0xc0,0xac,BYTE_LEN,0},
		{0xbb,0x80,BYTE_LEN,0},
		{0xbc,0x62,BYTE_LEN,0},
		{0xbd,0x1e,BYTE_LEN,0},
		{0xc1,0x1e,BYTE_LEN,0},
		{0x28,0x80,BYTE_LEN,0},
		#else
		{0x81,0xff,BYTE_LEN,0},
		{0xd2,0x46,BYTE_LEN,0},
		{0xda,0x80,BYTE_LEN,0},
		{0xdb,0x80,BYTE_LEN,0},		
		#endif


};

static struct ov7692_i2c_reg_conf const effect_sepia_tbl[] = {
        #if 0
		{0xbe,0x26,BYTE_LEN,0},
		{0xbf,0x7b,BYTE_LEN,0},
		{0xc0,0x10,BYTE_LEN,0},
		{0xbb,0x90,BYTE_LEN,0},
		{0xbc,0x28,BYTE_LEN,0},
		{0xbd,0x1e,BYTE_LEN,0},
		{0xc1,0x1e,BYTE_LEN,0},
		{0x28,0x00,BYTE_LEN,0},
		#else
		{0x81,0xff,BYTE_LEN,0},
		{0xd2,0x1e,BYTE_LEN,0},
		{0xda,0x40,BYTE_LEN,0},
		{0xdb,0xa0,BYTE_LEN,0},		
		#endif
};

static struct ov7692_i2c_reg_conf const wb_cloudy_tbl[] = {
	{0x13,0xf5, BYTE_LEN, 0},
	{0x01,0x90, BYTE_LEN, 0},
	{0x02,0xb0, BYTE_LEN, 0},
	{0x03,0x80, BYTE_LEN, 0},
};

static struct ov7692_i2c_reg_conf const wb_daylight_tbl[] = {
	{0x13,0xf5, BYTE_LEN, 0},
	{0x01,0x9a, BYTE_LEN, 0},
	{0x02,0xa0, BYTE_LEN, 0},
	{0x03,0x80, BYTE_LEN, 0},
};

static struct ov7692_i2c_reg_conf const wb_flourescant_tbl[] = {
	{0x13,0xf5, BYTE_LEN, 0},
	{0x01,0xb4, BYTE_LEN, 0},
	{0x02,0x8c, BYTE_LEN, 0},
	{0x03,0x80, BYTE_LEN, 0},
};

static struct ov7692_i2c_reg_conf const wb_incandescent_tbl[] = {
	{0x13,0xf5, BYTE_LEN, 0},
	{0x01,0xc6, BYTE_LEN, 0},
	{0x02,0x80, BYTE_LEN, 0},
	{0x03,0x80, BYTE_LEN, 0},
};

static struct ov7692_i2c_reg_conf const wb_auto_tbl[] = {
        {0x13, 0xf7, BYTE_LEN, 0}
};


static struct ov7692_i2c_reg_conf const contrast_tbl_0[] = {
    
//Contrast 0
	{0xd5,0x20, BYTE_LEN, 0},
	{0xd4,0x20, BYTE_LEN, 0},
	{0xd3,0x00, BYTE_LEN, 0},
};
static struct ov7692_i2c_reg_conf const contrast_tbl_1[] = {
    //Contrast 1 
	{0xd5,0x20, BYTE_LEN, 0},
	{0xd4,0x24, BYTE_LEN, 0},
	{0xd3,0x00, BYTE_LEN, 0},
};
static struct ov7692_i2c_reg_conf const contrast_tbl_2[] = {
//Contrast 2  (Default)  
	{0xd5,0x20, BYTE_LEN, 0},
	{0xd4,0x28, BYTE_LEN, 0},
	{0xd3,0x00, BYTE_LEN, 0},
};

static struct ov7692_i2c_reg_conf const contrast_tbl_3[] = {
//Contrast 3      
	{0xd5,0x20, BYTE_LEN, 0},
	{0xd4,0x2c, BYTE_LEN, 0},
	{0xd3,0x00, BYTE_LEN, 0},
};

static struct ov7692_i2c_reg_conf const contrast_tbl_4[] = {
//Contrast 4  
	{0xd5,0x20, BYTE_LEN, 0},
	{0xd4,0x30, BYTE_LEN, 0},
	{0xd3,0x00, BYTE_LEN, 0},
};

static struct ov7692_i2c_reg_conf const *contrast_tbl[] = {
    contrast_tbl_0,
    contrast_tbl_1,
    contrast_tbl_2,
    contrast_tbl_3,
    contrast_tbl_4,
};

static uint16_t const contrast_tbl_sz[] = {
    ARRAY_SIZE(contrast_tbl_0),
    ARRAY_SIZE(contrast_tbl_1),
    ARRAY_SIZE(contrast_tbl_2),
    ARRAY_SIZE(contrast_tbl_3),
    ARRAY_SIZE(contrast_tbl_4),
};

static struct ov7692_i2c_reg_conf const brightness_tbl_0[] = {
    //Brightness -3  
	{0xd3,0x40, BYTE_LEN, 0},
	{0xdc,0x09, BYTE_LEN, 0},
};

static struct ov7692_i2c_reg_conf const brightness_tbl_1[] = {
    //Brightness -2
	{0xd3,0x20, BYTE_LEN, 0},
	{0xdc,0x09, BYTE_LEN, 0},
};

static struct ov7692_i2c_reg_conf const brightness_tbl_2[] = {
    //Brightness -1
	 {0xd3,0x00, BYTE_LEN, 0},
	 {0xdc,0x09, BYTE_LEN, 0},
};

static struct ov7692_i2c_reg_conf const brightness_tbl_3[] = {
    //Brightness 0 (Default)
	{0xd3,0x10, BYTE_LEN, 0},
	{0xdc,0x01, BYTE_LEN, 0},
};

static struct ov7692_i2c_reg_conf const brightness_tbl_4[] = {
    //Brightness +1 
	{0xd3,0x20, BYTE_LEN, 0},
	{0xdc,0x01, BYTE_LEN, 0},
};

static struct ov7692_i2c_reg_conf const brightness_tbl_5[] = {
    //Brightness +2          
	{0xd3,0x30, BYTE_LEN, 0},
	{0xdc,0x01, BYTE_LEN, 0},
};

static struct ov7692_i2c_reg_conf const brightness_tbl_6[] = {
   //Brightness +3
	{0xd3,0x40, BYTE_LEN, 0},
	{0xdc,0x01, BYTE_LEN, 0},
};

static struct ov7692_i2c_reg_conf const *brightness_tbl[] = {
    brightness_tbl_0,
    brightness_tbl_1,
    brightness_tbl_2,
    brightness_tbl_3,
    brightness_tbl_4,
    brightness_tbl_5,
    brightness_tbl_6,
};

static uint16_t const brightness_tbl_sz[] = {
    ARRAY_SIZE(brightness_tbl_0),
    ARRAY_SIZE(brightness_tbl_1),
    ARRAY_SIZE(brightness_tbl_2),
    ARRAY_SIZE(brightness_tbl_3),
    ARRAY_SIZE(brightness_tbl_4),
    ARRAY_SIZE(brightness_tbl_5),
    ARRAY_SIZE(brightness_tbl_6),
};

static struct ov7692_i2c_reg_conf const saturation_tbl_0[] = {
//Saturation x0.25   
	{0xD8,0x10, BYTE_LEN, 0},
	{0xD9,0x10, BYTE_LEN, 0},
};

static struct ov7692_i2c_reg_conf const saturation_tbl_1[] = {
//Saturation x0.5  
	{0xD8,0x20, BYTE_LEN, 0},
	{0xD9,0x20, BYTE_LEN, 0},
};

static struct ov7692_i2c_reg_conf const saturation_tbl_2[] = {
//Saturation x0.75  
	/*change the default saturation,ZTE_CAM_YGL_20111215*/
	{0xD8,0x38, BYTE_LEN, 0}, //0x40
	{0xD9,0x38, BYTE_LEN, 0}, //0x40
};

static struct ov7692_i2c_reg_conf const saturation_tbl_3[] = {
//Saturation x0.75 
	{0xD8,0x60, BYTE_LEN, 0},
	{0xD9,0x60, BYTE_LEN, 0},
};

static struct ov7692_i2c_reg_conf const saturation_tbl_4[] = {
//Saturation x1 (Default)     
	{0xD8,0x80, BYTE_LEN, 0},
	{0xD9,0x80, BYTE_LEN, 0},
};

static struct ov7692_i2c_reg_conf const *saturation_tbl[] = {
    saturation_tbl_0,
    saturation_tbl_1,
    saturation_tbl_2,
    saturation_tbl_3,
    saturation_tbl_4,
};

static uint16_t const saturation_tbl_sz[] = {
    ARRAY_SIZE(saturation_tbl_0),
    ARRAY_SIZE(saturation_tbl_1),
    ARRAY_SIZE(saturation_tbl_2),
    ARRAY_SIZE(saturation_tbl_3),
    ARRAY_SIZE(saturation_tbl_4),
};

static struct ov7692_i2c_reg_conf const sharpness_tbl_0[] = {
//Sharpness 0    
	{0xb4,0x26, BYTE_LEN, 0},
	{0xb6,0x00, BYTE_LEN, 0},
};

static struct ov7692_i2c_reg_conf const sharpness_tbl_1[] = {
//Sharpness 1
	{0xb4,0x26, BYTE_LEN, 0},
	{0xb6,0x02, BYTE_LEN, 0},
};

static struct ov7692_i2c_reg_conf const sharpness_tbl_2[] = {
//Sharpness_Auto (Default)
	{0xb4,0x26, BYTE_LEN, 0},
	{0xb6,0x04, BYTE_LEN, 0},
};

static struct ov7692_i2c_reg_conf const sharpness_tbl_3[] = {
//Sharpness 3  
	{0xb4,0x26, BYTE_LEN, 0},
	{0xb6,0x06, BYTE_LEN, 0},
};

static struct ov7692_i2c_reg_conf const sharpness_tbl_4[] = {
//Sharpness 4  
	{0xb4,0x26, BYTE_LEN, 0},
	{0xb6,0x08, BYTE_LEN, 0},
};

static struct ov7692_i2c_reg_conf const *sharpness_tbl[] = {
    sharpness_tbl_0,
    sharpness_tbl_1,
    sharpness_tbl_2,
    sharpness_tbl_3,
    sharpness_tbl_4,
};

static uint16_t const sharpness_tbl_sz[] = {
    ARRAY_SIZE(sharpness_tbl_0),
    ARRAY_SIZE(sharpness_tbl_1),
    ARRAY_SIZE(sharpness_tbl_2),
    ARRAY_SIZE(sharpness_tbl_3),
    ARRAY_SIZE(sharpness_tbl_4),
};

static struct ov7692_i2c_reg_conf const exposure_compensation_tbl_0[] = {
	//@@ +1.7EV
	{0x24,0x38, BYTE_LEN, 0},
	{0x25,0x28, BYTE_LEN, 0},
	{0x26,0x81, BYTE_LEN, 0},
};

static struct ov7692_i2c_reg_conf const exposure_compensation_tbl_1[] = {
	//@@ +1.0EV
	{0x24,0x58, BYTE_LEN, 0},
	{0x25,0x48, BYTE_LEN, 0},
	{0x26,0xa2, BYTE_LEN, 0},
};

static struct ov7692_i2c_reg_conf const exposure_compensation_tbl_2[] = {
	//@@ default 
	{0x24,0x88, BYTE_LEN, 0},
	{0x25,0x78, BYTE_LEN, 0},
	{0x26,0xb5, BYTE_LEN, 0}, //0xd3,ZTE_YGL_20120105,adjust the AE speed
};

static struct ov7692_i2c_reg_conf const exposure_compensation_tbl_3[] = {
	//@@ -1.0EV
	{0x24,0xa8, BYTE_LEN, 0},
	{0x25,0x98, BYTE_LEN, 0},
	{0x26,0xd4, BYTE_LEN, 0},
};

static struct ov7692_i2c_reg_conf const exposure_compensation_tbl_4[] = {
	//@@ -1.7EV 
	{0x24,0xc8, BYTE_LEN, 0},
	{0x25,0xb8, BYTE_LEN, 0},
	{0x26,0xf5, BYTE_LEN, 0},
};



static struct ov7692_i2c_reg_conf const *exposure_compensation_tbl[] = {
    exposure_compensation_tbl_0,
    exposure_compensation_tbl_1,
    exposure_compensation_tbl_2,
    exposure_compensation_tbl_3,
    exposure_compensation_tbl_4,
	
};

static uint16_t const exposure_compensation_tbl_sz[] = {
    ARRAY_SIZE( exposure_compensation_tbl_0),
    ARRAY_SIZE( exposure_compensation_tbl_1),
    ARRAY_SIZE( exposure_compensation_tbl_2),
    ARRAY_SIZE( exposure_compensation_tbl_3),
    ARRAY_SIZE( exposure_compensation_tbl_4),
};

struct ov7692_reg_t ov7692_regs = {
    .prev_snap_reg_settings             = preview_snapshot_mode_reg_settings_array,
    .prev_snap_reg_settings_size     = ARRAY_SIZE(preview_snapshot_mode_reg_settings_array),

    .clk_tbl                = clk_tbl,
    .clk_tbl_sz             = ARRAY_SIZE(clk_tbl),

    .prevsnap_tbl           = prev_snap_tbl,
    .prevsnap_tbl_sz        = ARRAY_SIZE(prev_snap_tbl),
    
    .antibanding_50z_tbl           = antibanding_50z_tbl,
    .antibanding_50z_tbl_sz        = ARRAY_SIZE(antibanding_50z_tbl),

    .antibanding_60z_tbl           = antibanding_60z_tbl,
    .antibanding_60z_tbl_sz        = ARRAY_SIZE(antibanding_60z_tbl),

    .effect_off_tbl          =effect_off_tbl,
    .effect_off_tbl_sz     =ARRAY_SIZE(effect_off_tbl),

    .effect_mono_tbl      =effect_mono_tbl,
    .effect_mono_tbl_sz  =ARRAY_SIZE(effect_mono_tbl),

    .effect_negative_tbl   =effect_negative_tbl,
    .effect_negative_tbl_sz  =ARRAY_SIZE(effect_negative_tbl),

    .effect_sepia_tbl          =effect_sepia_tbl,
    .effect_sepia_tbl_sz     =ARRAY_SIZE(effect_sepia_tbl),

    .wb_cloudy_tbl          = wb_cloudy_tbl,
    .wb_cloudy_tbl_sz       = ARRAY_SIZE(wb_cloudy_tbl),

    .wb_daylight_tbl        = wb_daylight_tbl,
    .wb_daylight_tbl_sz     = ARRAY_SIZE(wb_daylight_tbl),

    .wb_flourescant_tbl     = wb_flourescant_tbl,
    .wb_flourescant_tbl_sz  = ARRAY_SIZE(wb_flourescant_tbl),

    .wb_incandescent_tbl    = wb_incandescent_tbl,
    .wb_incandescent_tbl_sz = ARRAY_SIZE(wb_incandescent_tbl),

    .wb_auto_tbl            = wb_auto_tbl,
    .wb_auto_tbl_sz         = ARRAY_SIZE(wb_auto_tbl),

    .contrast_tbl           = contrast_tbl,
    .contrast_tbl_sz        = contrast_tbl_sz,

    .brightness_tbl         = brightness_tbl,
    .brightness_tbl_sz      = brightness_tbl_sz,

    .saturation_tbl         = saturation_tbl,
    .saturation_tbl_sz      = saturation_tbl_sz,

    .sharpness_tbl          = sharpness_tbl,
    .sharpness_tbl_sz       = sharpness_tbl_sz,
	
    .exposure_compensation_tbl = exposure_compensation_tbl,
    .exposure_compensation_tbl_sz = exposure_compensation_tbl_sz,
};



