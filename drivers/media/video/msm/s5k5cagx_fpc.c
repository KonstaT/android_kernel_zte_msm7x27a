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

#define DEBUG

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/leds.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include "s5k5cagx.h"
#include <linux/slab.h>

static int s5k5cagx_pwdn_gpio;
static int s5k5cagx_reset_gpio;

#define FALSE 0
#define TRUE 1

//static unsigned int s5k5cagx_preview_exposure;
//static unsigned int s5k5cagx_gain;
//static unsigned short s5k5cagx_preview_maxlines;
//static int m_60Hz = FALSE;


static int S5K5CAGX_CSI_CONFIG = 0;
#define REG_S5K5CAGX_MCNEX_QTECH_RESET_AND_MISC_CTRL 0x001A
#define S5K5CAGX_MCNEX_QTECH_SOC_RESET               0x0219  /* SOC is in soft reset */
#define S5K5CAGX_MCNEX_QTECH_SOC_NOT_RESET           0x0218  /* SOC is not in soft reset */

#define REG_S5K5CAGX_MCNEX_QTECH_STANDBY_CONTROL     0x0018
#define S5K5CAGX_MCNEX_QTECH_SOC_STANDBY             0x402C  /* SOC is in standby state */

#define TRY_TIME             10

#define S5K5CAGX_GPIO_SWITCH_VAL     0
#define S5K5CAGX_GPIO_SWITCH_CTL    49


struct s5k5cagx_work {
	struct work_struct work;
};
static struct s5k5cagx_work *s5k5cagx_sensorw;
static struct i2c_client    *s5k5cagx_client;
static DECLARE_WAIT_QUEUE_HEAD(s5k5cagx_wait_queue);
DEFINE_MUTEX(s5k5cagx_mutex);

extern int32_t msm_camera_power_backend(enum msm_camera_pwr_mode_t pwr_mode);
extern int msm_camera_clk_switch(const struct msm_camera_sensor_info *data,
                                        uint32_t gpio_switch,
                                         uint32_t switch_val);
/*
 * ZTE_CAM_LJ_20120310
 * Get FTM flag to adjust 
 * the initialize process 
 * of camera
 */
#ifdef CONFIG_ZTE_PLATFORM
#ifdef CONFIG_ZTE_FTM_FLAG_SUPPORT
extern int zte_get_ftm_flag(void);
#endif
#endif

static u8 s5k5cagx_i2c_buf[4];
static u8 s5k5cagx_counter = 0;
//static int16_t s5k5cagx_effect = CAMERA_EFFECT_OFF;
static int is_autoflash = 0;
//static int effect_value = 0;
//static unsigned int SAT_U = 0x40;
//static unsigned int SAT_V = 0x40;

struct __s5k5cagx_ctrl 
{
	const struct msm_camera_sensor_info *sensordata;
	int sensormode;
	uint fps_divider; /* init to 1 * 0x00000400 */
	uint pict_fps_divider; /* init to 1 * 0x00000400 */
	u16 curr_step_pos;
	u16 curr_lens_pos;
	u16 init_curr_lens_pos;
	u16 my_reg_gain;
	u16 my_reg_line_count;
	enum msm_s_resolution prev_res;
	enum msm_s_resolution pict_res;
	enum msm_s_resolution curr_res;
	enum msm_s_test_mode  set_test;
};
static struct __s5k5cagx_ctrl *s5k5cagx_ctrl;
//static int afinit = 1;

extern struct rw_semaphore leds_list_lock;
extern struct list_head leds_list;

static int s5k5cagx_i2c_remove(struct i2c_client *client);
static int s5k5cagx_i2c_probe(struct i2c_client *client,const struct i2c_device_id *id);

static int s5k5cagx_i2c_txdata(u16 saddr,u8 *txdata,int length)
{
	struct i2c_msg msg[] = {
		{
			.addr  = saddr,
				.flags = 0,
				.len = length,
				.buf = txdata,
		},
	};

	if (i2c_transfer(s5k5cagx_client->adapter, msg, 1) < 0)    return -EIO;
	else return 0;
}

static int s5k5cagx_i2c_write(unsigned short saddr, unsigned int waddr,
							unsigned short bdata,u8 trytimes)
{
	int rc = -EIO;
	s5k5cagx_counter = 0;
	s5k5cagx_i2c_buf[0] = (waddr & 0xFF00)>>8;
	s5k5cagx_i2c_buf[1] = (waddr & 0x00FF);
	//s5k5cagx_i2c_buf[2] = (bdata & 0x00FF);
	s5k5cagx_i2c_buf[2] = (bdata & 0xFF00) >> 8;
	s5k5cagx_i2c_buf[3] = (bdata & 0x00FF);


	while ((s5k5cagx_counter < trytimes) &&(rc != 0))
	{
		rc = s5k5cagx_i2c_txdata(saddr, s5k5cagx_i2c_buf, 4);

		if (rc < 0)
		{
			s5k5cagx_counter++;
			pr_err("***--CAMERA-- i2c_write_w failed,i2c addr=0x%x, command addr = 0x%x, val = 0x%x, s=%d, rc=%d!\n",saddr,waddr, bdata,s5k5cagx_counter,rc);
			msleep(4);
		}
	}
	return rc;
}

static int s5k5cagx_i2c_rxdata(unsigned short saddr,
							 unsigned char *rxdata, int length)
{
	struct i2c_msg msgs[] = {
		{
			.addr   = saddr,
				.flags = 0,
				.len   = 2,
				.buf   = rxdata,
		},
		{
			.addr   = saddr,
				.flags = I2C_M_RD,
				.len   = length,
				.buf   = rxdata,
			},
	};

	if (i2c_transfer(s5k5cagx_client->adapter, msgs, 2) < 0) {
		pr_err("s5k5cagx_i2c_rxdata failed!\n");
		return -EIO;
	}

	return 0;
}

static int32_t s5k5cagx_i2c_read_byte(unsigned short   saddr,
									unsigned int raddr, unsigned int *rdata)
{
	int rc = 0;
	unsigned char buf[2];
	//pr_err("+s5k5cagx_i2c_read_byte\n");
	memset(buf, 0, sizeof(buf));

	buf[0] = (raddr & 0xFF00)>>8;
	buf[1] = (raddr & 0x00FF);

	rc = s5k5cagx_i2c_rxdata(saddr, buf, 1);
	if (rc < 0) {
		pr_err("s5k5cagx_i2c_read_byte failed!\n");
		return rc;
	}

	*rdata = buf[0];

	//pr_err("-s5k5cagx_i2c_read_byte\n");
	return rc;
}
#if 1
static int32_t s5k5cagx_i2c_read(unsigned short   saddr,
							   unsigned int raddr, unsigned int *rdata)
{
	int rc = 0;
	unsigned char buf[2];
	//pr_err("+s5k5cagx_i2c_read\n");
	memset(buf, 0, sizeof(buf));

	buf[0] = (raddr & 0xFF00)>>8;
	buf[1] = (raddr & 0x00FF);

	rc = s5k5cagx_i2c_rxdata(saddr, buf, 2);
	if (rc < 0)	return rc;
	*rdata = buf[0] << 8 | buf[1];

	if (rc < 0)
		pr_err("s5k5cagx_i2c_read failed!\n");
	//pr_err("-s5k5cagx_i2c_read\n");
	return rc;
}
#endif

static int32_t S5K5CAGX_WritePRegs(PS5K5CAGX_WREG pTb,int32_t len)
{
	int32_t i, ret = 0;
	uint32_t regv;
	for (i = 0;i < len; i++)
	{
		if (0 == pTb[i].mask) {  
			s5k5cagx_i2c_write(s5k5cagx_client->addr,pTb[i].addr,pTb[i].data,10);
		} else {   
			s5k5cagx_i2c_read_byte(s5k5cagx_client->addr,pTb[i].addr, &regv);
			regv &= pTb[i].mask;
			regv |= (pTb[i].data & (~pTb[i].mask));
			s5k5cagx_i2c_write(s5k5cagx_client->addr, pTb[i].addr, regv, 10);
		}
	}
	return ret;
}
#if 0
static void camera_sw_power_onoff(int v)
{
	if (v == 0) {
		pr_err("camera_sw_power_onoff: down\n");
		s5k5cagx_i2c_write(s5k5cagx_client->addr, 0x3008, 0x42, 10); //power down
	}
	else {
		pr_err("camera_sw_power_onoff: on\n");
		s5k5cagx_i2c_write(s5k5cagx_client->addr, 0x3008, 0x02, 10); //power on
	}
}
#endif
static void s5k5cagx_power_off(void)
{
	pr_err("--CAMERA-- %s ... (Start...)\n",__func__);

	gpio_set_value(s5k5cagx_pwdn_gpio, 1);
	//gpio_set_value(s5k5cagx_reset_gpio, 0);

	//afinit = 0;
	
	pr_err("--CAMERA-- %s ... (End...)\n",__func__);
}

static void s5k5cagx_power_on(void)
{
	pr_err("--CAMERA-- %s ... (Start...)\n",__func__);

	gpio_set_value(s5k5cagx_pwdn_gpio, 0);

	//afinit = 1;

	pr_err("--CAMERA-- %s ... (End...)\n",__func__);
}

static void s5k5cagx_power_reset(void)
{
	pr_err("--CAMERA-- %s ... (Start...)\n",__func__);
	#if 0
	gpio_set_value(s5k5cagx_reset_gpio, 1);   //reset camera reset pin
	mdelay(5);
	gpio_set_value(s5k5cagx_reset_gpio, 0);
	mdelay(5);
	#endif
	gpio_set_value(s5k5cagx_reset_gpio, 1);
	mdelay(10);

	pr_err("--CAMERA-- %s ... (End...)\n",__func__);
}

static int s5k5cagx_probe_readID(const struct msm_camera_sensor_info *data)
{
	int rc = 0;    
	//u32 device_id_high = 0;
	//u32 device_id_low = 0;
	u32 device_id = 0;

	pr_err("--CAMERA-- %s (Start...)\n",__func__);
	pr_err("--CAMERA-- %s sensor poweron,begin to read ID!\n",__func__);

	rc = s5k5cagx_i2c_write(s5k5cagx_client->addr, 0x002C, 0x0000, 10);    
    if (rc < 0)
    {
    	pr_err("--CAMERA-- %s ok , writeI2C failed, rc = 0x%x\r\n", __func__, rc);
        return rc;    
    }
    rc = s5k5cagx_i2c_write(s5k5cagx_client->addr, 0x002E, 0x0040,10);    
    if (rc < 0)
    {
    	pr_err("--CAMERA-- %s ok , writeI2C failed, rc = 0x%x\r\n", __func__, rc);
        return rc;    
    }  
	
	//0x300A ,sensor ID register
	//rc = s5k5cagx_i2c_read_byte(s5k5cagx_client->addr, 0x0F12, &device_id_high);
	rc = s5k5cagx_i2c_read(s5k5cagx_client->addr, 0x0F12, &device_id);
	if (rc < 0)
	{
		pr_err("--CAMERA-- %s ok , readI2C failed, rc = 0x%x\r\n", __func__, rc);
		return rc;   
	}  
	pr_err("--CAMERA-- %s  readID high byte, data = 0x%x\r\n", __func__, device_id);
#if 0
	//0x300B ,sensor ID register
	rc = s5k5cagx_i2c_read_byte(s5k5cagx_client->addr, 0x300B, &device_id_low);
	if (rc < 0)
	{
		pr_err("--CAMERA-- %s ok , readI2C failed,rc = 0x%x\r\n", __func__, rc);
		return rc;   
	}   

	pr_err("--CAMERA-- %s  readID low byte, data = 0x%x\r\n", __func__, device_id_low);
	pr_err("--CAMERA-- %s return ID :0x%x\n", __func__, (device_id_high<<8)+device_id_low);
#endif

#ifdef CONFIG_SENSOR_INFO
	msm_sensorinfo_set_sensor_id(device_id);
#endif

	//0x5640, s5k5cagx chip id
	//if((device_id_high<<8)+device_id_low != S5K5CAGX_SENSOR_ID)
	if(device_id != S5K5CAGX_SENSOR_ID)
	{
		pr_err("--CAMERA-- %s ok , device id error, should be 0x%x\r\n",
			__func__, S5K5CAGX_SENSOR_ID);
		return -EINVAL;
	}
	else
	{
		pr_err("--CAMERA-- %s ok , device id=0x%x\n",__func__,S5K5CAGX_SENSOR_ID);
		return 0;
	}
}

#if 0
static void dump_af_status(void)
{
	int tmp = 0;

	s5k5cagx_i2c_read_byte(s5k5cagx_client->addr, 0x3000, &tmp);
	pr_err("	%s 0x3000 = %x\n", __func__, tmp);
	s5k5cagx_i2c_read_byte(s5k5cagx_client->addr, 0x3001, &tmp);
	pr_err("	%s 0x3001 = %x\n", __func__, tmp);
	s5k5cagx_i2c_read_byte(s5k5cagx_client->addr, 0x3004, &tmp);
	pr_err("	%s 0x3004 = %x\n", __func__, tmp);
	s5k5cagx_i2c_read_byte(s5k5cagx_client->addr, 0x3005, &tmp);
	pr_err("	%s 0x3005 = %x\n", __func__, tmp);
	s5k5cagx_i2c_read_byte(s5k5cagx_client->addr, S5K5CAGX_CMD_FW_STATUS, &tmp);
	pr_err("	%s af_st = %x\n\n", __func__, tmp);
}
#endif
#if 0
static int s5k5cagx_af_setting(void)
{
	int rc = 0;

	pr_err("--CAMERA-- s5k5cagx_af_setting\n");
	rc = S5K5CAGXCore_WritePREG(s5k5cagx_afinit_tbl);   

	if (rc < 0)
	{
		pr_err("--CAMERA-- AF_init failed\n");
		return rc;
	}

	return rc;
}
#endif
static int s5k5cagx_set_flash_light(enum led_brightness brightness)
{
	struct led_classdev *led_cdev;

	pr_err("s5k5cagx_set_flash_light brightness = %d\n", brightness);

	down_read(&leds_list_lock);
	list_for_each_entry(led_cdev, &leds_list, node) {
		if (!strncmp(led_cdev->name, "flashlight", 10)) {
			break;
		}
	}
	up_read(&leds_list_lock);

	if (led_cdev) {
		led_brightness_set(led_cdev, brightness);
	} else {
		pr_err("get flashlight device failed\n");
		return -1;
	}

	return 0;
}

static int s5k5cagx_video_config(void)
{
	int rc = 0;
#if 0	
	u32 deviceid_1 = 0;
	u32 deviceid_2 = 0;
	u32 deviceid_3 = 0;
	u32 deviceid_4 = 0;
	u32 deviceid_5 = 0;
#endif	
	pr_err("--CAMERA-- s5k5cagx_video_config\n");

	pr_err("--CAMERA-- preview in, is_autoflash - 0x%x\n", is_autoflash);

	/* autoflash setting */
	if (is_autoflash == 1) {
		s5k5cagx_set_flash_light(LED_OFF);
	}

	/* preview setting */
	rc = S5K5CAGXCore_WritePREG(s5k5cagx_preview_tbl);   
    if (rc < 0)
    {
    	pr_err("--CAMERA-- %s ok , writeI2C s5k5cagx_preview_tbl failed, rc = 0x%x\r\n", __func__, rc);
        return rc;    
    }
#if 0	
	rc = s5k5cagx_i2c_write(s5k5cagx_client->addr, 0x002C, 0x7000,10);    
    if (rc < 0)
    {
    	pr_err("--CAMERA-- %s ok , writeI2C failed, rc = 0x%x\r\n", __func__, rc);
        return rc;    
    }  
	rc = s5k5cagx_i2c_write(s5k5cagx_client->addr, 0x002E, 0x16B0,10);    
    if (rc < 0)
    {
    	pr_err("--CAMERA-- %s ok , writeI2C failed, rc = 0x%x\r\n", __func__, rc);
        return rc;    
    }  

	rc = s5k5cagx_i2c_read(s5k5cagx_client->addr, 0x0F12, &deviceid_1);
	if (rc < 0)
	{
		pr_err("--CAMERA-- %s ok , readI2C failed, rc = 0x%x\r\n", __func__, rc);
		return rc;   
	}
	pr_err("--CAMERA-- %s ok , deviceid_1 = 0x%x\r\n", __func__, deviceid_1);
	
	rc = s5k5cagx_i2c_read(s5k5cagx_client->addr, 0x0F12, &deviceid_2);
	if (rc < 0)
	{
		pr_err("--CAMERA-- %s ok , readI2C failed, rc = 0x%x\r\n", __func__, rc);
		return rc;   
	}
	pr_err("--CAMERA-- %s ok , deviceid_2 = 0x%x\r\n", __func__, deviceid_2);
	
	rc = s5k5cagx_i2c_read(s5k5cagx_client->addr, 0x0F12, &deviceid_3);
	if (rc < 0)
	{
		pr_err("--CAMERA-- %s ok , readI2C failed, rc = 0x%x\r\n", __func__, rc);
		return rc;   
	}
	pr_err("--CAMERA-- %s ok , deviceid_3 = 0x%x\r\n", __func__, deviceid_3);
	
	rc = s5k5cagx_i2c_read(s5k5cagx_client->addr, 0x0F12, &deviceid_4);
	if (rc < 0)
	{
		pr_err("--CAMERA-- %s ok , readI2C failed, rc = 0x%x\r\n", __func__, rc);
		return rc;   
	}
	pr_err("--CAMERA-- %s ok , deviceid_4 = 0x%x\r\n", __func__, deviceid_4);
	
	rc = s5k5cagx_i2c_read(s5k5cagx_client->addr, 0x0F12, &deviceid_5);
	if (rc < 0)
	{
		pr_err("--CAMERA-- %s ok , readI2C failed, rc = 0x%x\r\n", __func__, rc);
		return rc;   
	}
	pr_err("--CAMERA-- %s ok , deviceid_5 = 0x%x\r\n", __func__, deviceid_5);

#endif	
	return rc;
}


#if 0
static int s5k5cagx_get_preview_exposure_gain(void)
{
    int rc = 0;
    unsigned int ret_l,ret_m,ret_h;
            
    s5k5cagx_i2c_write(s5k5cagx_client->addr,0x3503, 0x07,10);

    //get preview exp & gain
    ret_h = ret_m = ret_l = 0;
    s5k5cagx_preview_exposure = 0;
    s5k5cagx_i2c_read_byte(s5k5cagx_client->addr,0x3500, &ret_h);
    s5k5cagx_i2c_read_byte(s5k5cagx_client->addr,0x3501, &ret_m);
    s5k5cagx_i2c_read_byte(s5k5cagx_client->addr,0x3502, &ret_l);
    s5k5cagx_preview_exposure = (ret_h << 12) + (ret_m << 4) + (ret_l >> 4);
//    printk("preview_exposure=%d\n", s5k5cagx_preview_exposure);

    ret_h = ret_m = ret_l = 0;
    s5k5cagx_preview_maxlines = 0;
    s5k5cagx_i2c_read_byte(s5k5cagx_client->addr,0x380e, &ret_h);
    s5k5cagx_i2c_read_byte(s5k5cagx_client->addr,0x380f, &ret_l);
    s5k5cagx_preview_maxlines = (ret_h << 8) + ret_l;
//    printk("Preview_Maxlines=%d\n", s5k5cagx_preview_maxlines);

    //Read back AGC Gain for preview
    s5k5cagx_gain = 0;
    s5k5cagx_i2c_read_byte(s5k5cagx_client->addr,0x350b, &s5k5cagx_gain);
//    printk("Gain,0x350b=0x%x\n", s5k5cagx_gain);

    return rc;
}
#endif
#if 0
static int s5k5cagx_set_capture_exposure_gain(void)
{
    int rc = 0;
   //calculate capture exp & gain
   
    unsigned char ExposureLow,ExposureMid,ExposureHigh;
    unsigned int ret_l,ret_m,ret_h,Lines_10ms;
    unsigned short ulCapture_Exposure,iCapture_Gain;
    unsigned int ulCapture_Exposure_Gain,Capture_MaxLines;

    ret_h = ret_m = ret_l = 0;
    s5k5cagx_i2c_read_byte(s5k5cagx_client->addr,0x380e, &ret_h);
    s5k5cagx_i2c_read_byte(s5k5cagx_client->addr,0x380f, &ret_l);
    Capture_MaxLines = (ret_h << 8) + ret_l;
    printk("Capture_MaxLines=%d\n", Capture_MaxLines);

    if(m_60Hz == TRUE)
    {
    Lines_10ms = Capture_Framerate * Capture_MaxLines/12000;
    }
    else
    {
    Lines_10ms = Capture_Framerate * Capture_MaxLines/10000;
    }


    if(s5k5cagx_preview_maxlines == 0)
    {
    s5k5cagx_preview_maxlines = 1;
    }

    ulCapture_Exposure = (s5k5cagx_preview_exposure*(Capture_Framerate)*(Capture_MaxLines))/(((s5k5cagx_preview_maxlines)*(g_Preview_FrameRate)));

    iCapture_Gain = s5k5cagx_gain;


    ulCapture_Exposure_Gain = ulCapture_Exposure * iCapture_Gain; 

    if(ulCapture_Exposure_Gain < Capture_MaxLines*16)
    {
    ulCapture_Exposure = ulCapture_Exposure_Gain/16;

    if (ulCapture_Exposure > Lines_10ms)
    {
      ulCapture_Exposure /= Lines_10ms;
      ulCapture_Exposure *= Lines_10ms;
    }
    }
    else
    {
    ulCapture_Exposure = Capture_MaxLines;
    }

    if(ulCapture_Exposure == 0)
    {
    ulCapture_Exposure = 1;
    }

    iCapture_Gain = (ulCapture_Exposure_Gain*2/ulCapture_Exposure + 1)/2;

    ExposureLow = ((unsigned char)ulCapture_Exposure)<<4;

    ExposureMid = (unsigned char)(ulCapture_Exposure >> 4) & 0xff;

    ExposureHigh = (unsigned char)(ulCapture_Exposure >> 12);


    // set capture exp & gain

    s5k5cagx_i2c_write(s5k5cagx_client->addr,0x350b, iCapture_Gain, 10);

	s5k5cagx_i2c_write(s5k5cagx_client->addr,0x3502, ExposureLow, 10);
    s5k5cagx_i2c_write(s5k5cagx_client->addr,0x3501, ExposureMid, 10);
    s5k5cagx_i2c_write(s5k5cagx_client->addr,0x3500, ExposureHigh, 10);
  
//    printk("iCapture_Gain=%d\n", iCapture_Gain);
//    printk("ExposureLow=%d\n", ExposureLow);
//    printk("ExposureMid=%d\n", ExposureMid);
//    printk("ExposureHigh=%d\n", ExposureHigh);

    msleep(250);

    return rc;
}
#endif
static int s5k5cagx_snapshot_config(void)
{
	int rc = 0;
	unsigned int tmp;
	//int capture_len = sizeof(s5k5cagx_capture_tbl) / sizeof(s5k5cagx_capture_tbl[0]);

	pr_err("--CAMERA-- SENSOR_SNAPSHOT_MODE\n");
	//s5k5cagx_get_preview_exposure_gain();//Step1: get preview exposure and gain

	pr_err("--CAMERA-- %s, snapshot in, is_autoflash - 0x%x\n", __func__, is_autoflash);

	if (is_autoflash == 1) {
		s5k5cagx_i2c_read_byte(s5k5cagx_client->addr, 0x350b, &tmp);
		pr_err("--CAMERA-- GAIN VALUE : %x\n", tmp);
		if ((tmp & 0x80) == 0) {
			s5k5cagx_set_flash_light(LED_OFF);
		} else {
			s5k5cagx_set_flash_light(LED_FULL);
		}
	}

	rc = S5K5CAGXCore_WritePREG(s5k5cagx_capture_tbl);  //Step2: change to full size 
	//s5k5cagx_set_capture_exposure_gain();//Step3: calculate and set capture exposure and gain

	return rc;
}

static int s5k5cagx_setting(enum msm_s_reg_update rupdate,enum msm_s_setting rt)
{
	int rc = -EINVAL;
	//unsigned int tmp;
	struct msm_camera_csi_params s5k5cagx_csi_params;

	pr_err("--CAMERA-- %s (Start...), rupdate=%d \n",__func__,rupdate);

	switch (rupdate)
	{
	case S_UPDATE_PERIODIC:
		//camera_sw_power_onoff(0); //standby ???

		//msleep(20);

		s5k5cagx_csi_params.lane_cnt = 1;
		s5k5cagx_csi_params.data_format = CSI_8BIT;
		s5k5cagx_csi_params.lane_assign = 0xe4;
		s5k5cagx_csi_params.dpcm_scheme = 0;
		s5k5cagx_csi_params.settle_cnt = 0x6;

		pr_err("%s: msm_camio_csi_config\n", __func__);

		rc = msm_camio_csi_config(&s5k5cagx_csi_params);				
		S5K5CAGX_CSI_CONFIG = 1;

		msleep(20);

		if (S_RES_PREVIEW == rt) {
			rc = s5k5cagx_video_config();
			msleep(80);
		} else if (S_RES_CAPTURE == rt) {
			rc = s5k5cagx_snapshot_config();
			msleep(200);
		}
		//camera_sw_power_onoff(1); //on

		//msleep(80);

		break; /* UPDATE_PERIODIC */

	case S_REG_INIT:
		pr_err("--CAMERA-- S_REG_INIT (Start)\n");
#if 1
		rc = s5k5cagx_i2c_write(s5k5cagx_client->addr, 0xFCFC, 0xD000, 10);
		if (rc < 0) {
			pr_err("sensor init setting failed 1\n");
			break;
		}
		rc = s5k5cagx_i2c_write(s5k5cagx_client->addr, 0x0010, 0x0001, 10);
		if (rc < 0) {
			pr_err("sensor init setting failed 2\n");
			break;
		}
		rc = s5k5cagx_i2c_write(s5k5cagx_client->addr, 0x1030, 0x0000, 10);
		if (rc < 0) {
			pr_err("sensor init setting failed 3\n");
			break;
		}
		rc = s5k5cagx_i2c_write(s5k5cagx_client->addr, 0x0014, 0x0001, 10);
		if (rc < 0) {
			pr_err("sensor init setting failed 4\n");
			break;
		}
		msleep(100);
		rc = S5K5CAGXCore_WritePREG(s5k5cagx_init_tbl_1);
		if (rc < 0) {
			pr_err("sensor init setting s5k5cagx_init_tbl_1 failed\n");
			break;
		}
		msleep(100);
		rc = S5K5CAGXCore_WritePREG(s5k5cagx_init_tbl_2);
		if (rc < 0) {
			pr_err("sensor init setting s5k5cagx_init_tbl_2 failed\n");
			break;
		}
		msleep(100);
		rc = S5K5CAGXCore_WritePREG(s5k5cagx_init_tbl_3);
		if (rc < 0) {
			pr_err("sensor init setting s5k5cagx_init_tbl_3 failed\n");
			break;
		}
		msleep(1000);

		rc = s5k5cagx_i2c_write(s5k5cagx_client->addr, REG_S5K5CAGX_MCNEX_QTECH_STANDBY_CONTROL, 0x0028, 10);
	    if (rc < 0)
	    {
	    	pr_err("sensor init setting REG_S5K5CAGX_MCNEX_QTECH_STANDBY_CONTROL failed\n");
	        break;
	    }
	    mdelay(100);

#endif

#if 0
		rc = s5k5cagx_i2c_write(s5k5cagx_client->addr, 0x3103, 0x11, 10);
		rc = s5k5cagx_i2c_write(s5k5cagx_client->addr, 0x3008, 0x82, 10);
		msleep(5);

		//set sensor init setting
		pr_err("set sensor init setting\n");
		rc = S5K5CAGXCore_WritePREG(s5k5cagx_init_tbl);
		if (rc < 0) {
			pr_err("sensor init setting failed\n");
			break;
		}

		//set image quality setting
		pr_err("%s: set image quality setting\n", __func__);
		rc = S5K5CAGXCore_WritePREG(s5k5cagx_init_iq_tbl);

		rc =s5k5cagx_i2c_read_byte(s5k5cagx_client->addr, 0x4740, &tmp);			   
		pr_err("--CAMERA-- init 0x4740 value=0x%x\n", tmp);

		if (tmp != 0x21) {
			rc = s5k5cagx_i2c_write(s5k5cagx_client->addr, 0x4740, 0x21, 10);
			msleep(10);
			rc =s5k5cagx_i2c_read_byte(s5k5cagx_client->addr, 0x4740, &tmp);			   
			pr_err("--CAMERA-- WG 0x4740 value=0x%x\n", tmp);
		}

		pr_err("--CAMERA-- AF_init: afinit = %d\n", afinit);
		if (afinit == 1) {
			rc = s5k5cagx_af_setting();
			if (rc < 0) {
				pr_err("--CAMERA-- s5k5cagx_af_setting failed\n");
				break;
			}
			afinit = 0;
		}
#endif
		/* reset fps_divider */
		s5k5cagx_ctrl->fps_divider = 1 * 0x0400;
		pr_err("--CAMERA-- S_REG_INIT (End)\n");
		break; /* case REG_INIT: */

	default:
		break;
	} /* switch (rupdate) */

	pr_err("--CAMERA-- %s (End), rupdate=%d \n",__func__,rupdate);

	return rc;
}

static int s5k5cagx_sensor_open_init(const struct msm_camera_sensor_info *data)
{
	//int rc = -ENOMEM;
	int rc = 0;
	pr_err("--CAMERA-- %s\n",__func__);

	s5k5cagx_ctrl = kzalloc(sizeof(struct __s5k5cagx_ctrl), GFP_KERNEL);
	if (!s5k5cagx_ctrl)
	{
		pr_err("--CAMERA-- kzalloc s5k5cagx_ctrl error !!\n");
		kfree(s5k5cagx_ctrl);
		return rc;
	}

	s5k5cagx_ctrl->fps_divider = 1 * 0x00000400;
	s5k5cagx_ctrl->pict_fps_divider = 1 * 0x00000400;
	s5k5cagx_ctrl->set_test = S_TEST_OFF;
	s5k5cagx_ctrl->prev_res = S_QTR_SIZE;
	s5k5cagx_ctrl->pict_res = S_FULL_SIZE;

	if (data)
		s5k5cagx_ctrl->sensordata = data;

	//s5k5cagx_power_off();

	pr_err("%s: msm_camio_clk_rate_set\n", __func__);

#if defined(CONFIG_MACH_BLADE2)	|| defined(CONFIG_MACH_ATLAS40)
	rc = msm_camera_clk_switch(data, S5K5CAGX_GPIO_SWITCH_CTL, S5K5CAGX_GPIO_SWITCH_VAL);
	if (rc < 0) {
		pr_err("msm_camera_clk_switch fail\n");
		return rc;
	}
#endif	
	
	msm_camio_clk_rate_set(24000000);
	mdelay(5);

	s5k5cagx_power_on();
	mdelay(50);

{
	/*
     * ZTE_CAM_LJ_20110602
     * fix bug of high current in sleep mode
     */
    rc = s5k5cagx_i2c_write(s5k5cagx_client->addr, 0x0028, 0x7000, TRY_TIME);
    if(rc < 0)
    {
        return rc;
    }            
    rc = s5k5cagx_i2c_write(s5k5cagx_client->addr, 0x002A, 0x2824, TRY_TIME);
    if (rc < 0)
    {
        return rc;
    }
    rc = s5k5cagx_i2c_write(s5k5cagx_client->addr, 0x0F12, 0x0001, TRY_TIME);
    if (rc < 0)
    {
        return rc;
    }
    rc = s5k5cagx_i2c_write(s5k5cagx_client->addr, 0x0028, 0x7000, TRY_TIME);
    if(rc < 0)
    {
        return rc;
    }            
    rc = s5k5cagx_i2c_write(s5k5cagx_client->addr, 0x002A, 0x0252, TRY_TIME);
    if (rc < 0)
    {
        return rc;
    }
    rc = s5k5cagx_i2c_write(s5k5cagx_client->addr, 0x0F12, 0x0003, TRY_TIME);
    if (rc < 0)
    {
        return rc;
    }
    rc = s5k5cagx_i2c_write(s5k5cagx_client->addr, 0x0028, 0x7000, TRY_TIME);
    if (rc < 0)
    {
        return rc;
    }
    rc = s5k5cagx_i2c_write(s5k5cagx_client->addr, 0x002A, 0x0254, TRY_TIME);
    if(rc < 0)
    {
        return rc;
    }  
    rc = s5k5cagx_i2c_write(s5k5cagx_client->addr, 0x0F12, 0x0000, TRY_TIME);
    if (rc < 0)
    {
        return rc;
    }
    msleep(133);
    rc = s5k5cagx_i2c_write(s5k5cagx_client->addr, 0x002A, 0x0252, TRY_TIME);
    if(rc < 0)
    {
        return rc;
    }  
    rc = s5k5cagx_i2c_write(s5k5cagx_client->addr, 0x0F12, 0x0002, TRY_TIME);
    if (rc < 0)
    {
        return rc;
    }
}

#if 0
	
	s5k5cagx_power_reset();

    rc = s5k5cagx_i2c_write(s5k5cagx_client->addr, 
                           REG_S5K5CAGX_MCNEX_QTECH_RESET_AND_MISC_CTRL, 
                           S5K5CAGX_MCNEX_QTECH_SOC_RESET,
                           10);
    if (rc < 0)
    {
        pr_err("soft reset failed!\n");
        return rc;
    }

	mdelay(1);

	rc = s5k5cagx_i2c_write(s5k5cagx_client->addr,
                           REG_S5K5CAGX_MCNEX_QTECH_RESET_AND_MISC_CTRL,
                           S5K5CAGX_MCNEX_QTECH_SOC_NOT_RESET & 0x0018,
                           10);
    if (rc < 0)
    {
        pr_err("soft reset failed!\n");
        return rc;
    }

	rc = s5k5cagx_i2c_write(s5k5cagx_client->addr,
                           REG_S5K5CAGX_MCNEX_QTECH_RESET_AND_MISC_CTRL,
                           S5K5CAGX_MCNEX_QTECH_SOC_NOT_RESET,
                           10);
    if (rc < 0)
    {
        pr_err("soft reset failed!\n");
        return rc;
    }

	mdelay(10);

	pr_err("%s: init sequence\n", __func__);

	if (s5k5cagx_ctrl->prev_res == S_QTR_SIZE)
		rc = s5k5cagx_setting(S_REG_INIT, S_RES_PREVIEW);
	else
		rc = s5k5cagx_setting(S_REG_INIT, S_RES_CAPTURE);

	if (rc < 0)
	{
		pr_err("--CAMERA-- %s : s5k5cagx_setting failed. rc = %d\n",__func__,rc);
		kfree(s5k5cagx_ctrl);
		return rc;
	}
#endif
	pr_err("--CAMERA--re_init_sensor ok!!\n");
	return rc;
}

static int s5k5cagx_sensor_release(void)
{
	int rc;
	pr_err("--CAMERA--s5k5cagx_sensor_release!!\n");

	mutex_lock(&s5k5cagx_mutex);

{
        /*
         * ZTE_CAM_LJ_20110602
         * fix bug of high current in sleep mode
         */
        rc = s5k5cagx_i2c_write(s5k5cagx_client->addr, 0x0028, 0x7000, TRY_TIME);
        if(rc < 0)
        {
            return rc;
        }            
        rc = s5k5cagx_i2c_write(s5k5cagx_client->addr, 0x002A, 0x2824, TRY_TIME);
        if (rc < 0)
        {
            return rc;
        }
        rc = s5k5cagx_i2c_write(s5k5cagx_client->addr, 0x0F12, 0x0000, TRY_TIME);
        if (rc < 0)
        {
            return rc;
        }
        rc = s5k5cagx_i2c_write(s5k5cagx_client->addr, 0x0028, 0x7000, TRY_TIME);
        if(rc < 0)
        {
            return rc;
        }            
        rc = s5k5cagx_i2c_write(s5k5cagx_client->addr, 0x002A, 0x0254, TRY_TIME);
        if (rc < 0)
        {
            return rc;
        }
        rc = s5k5cagx_i2c_write(s5k5cagx_client->addr, 0x0F12, 0x0001, TRY_TIME);
        if (rc < 0)
        {
            return rc;
        }
        rc = s5k5cagx_i2c_write(s5k5cagx_client->addr, 0x002A, 0x0252, TRY_TIME);
        if(rc < 0)
        {
            return rc;
        }            
        rc = s5k5cagx_i2c_write(s5k5cagx_client->addr, 0x0F12, 0x0004, TRY_TIME);
        if (rc < 0)
        {
            return rc;
        }
        msleep(133);
        rc = s5k5cagx_i2c_write(s5k5cagx_client->addr, 0x002A, 0x0252, TRY_TIME);
        if (rc < 0)
        {
            return rc;
        }
        rc = s5k5cagx_i2c_write(s5k5cagx_client->addr, 0x0F12, 0x0002, TRY_TIME);
        if (rc < 0)
        {
            return rc;
        }
        msleep(200);
            
    }

	s5k5cagx_power_off();
	mdelay(300);
	
	//kfree(s5k5cagx_ctrl);
	//s5k5cagx_ctrl = NULL;

	mutex_unlock(&s5k5cagx_mutex);
	return 0;
}

static const struct i2c_device_id s5k5cagx_i2c_id[] = {
	{"s5k5cagx", 0},{}
};

static int s5k5cagx_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static int s5k5cagx_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&s5k5cagx_wait_queue);
	return 0;
}

static long s5k5cagx_set_effect(int mode, int effect)
{
	int rc = 0;
	pr_err("--CAMERA-- %s ...(Start)\n",__func__);
#if 0
	switch (mode)
	{
	case SENSOR_PREVIEW_MODE:
		//Context A Special Effects /
		pr_err("--CAMERA-- %s ...SENSOR_PREVIEW_MODE\n",__func__);
		break;

	case SENSOR_SNAPSHOT_MODE:
		// Context B Special Effects /
		pr_err("--CAMERA-- %s ...SENSOR_SNAPSHOT_MODE\n",__func__);
		break;

	default:
		break;
	}

	effect_value = effect;
#endif
#if 1
	switch (effect) 
	{
	case CAMERA_EFFECT_OFF: 
		{
			pr_err("--CAMERA-- %s ...CAMERA_EFFECT_OFF\n",__func__);
			rc = S5K5CAGXCore_WritePREG(s5k5cagx_effect_off_tbl);			 
			break;
		}
	case CAMERA_EFFECT_MONO:
		{
			pr_err("--CAMERA-- %s ...CAMERA_EFFECT_MONO\n",__func__);
			rc = S5K5CAGXCore_WritePREG(s5k5cagx_effect_mono_tbl);
			break;
		}
	case CAMERA_EFFECT_NEGATIVE:
		{
			pr_err("--CAMERA-- %s ...CAMERA_EFFECT_NEGATIVE\n",__func__);
			rc = S5K5CAGXCore_WritePREG(s5k5cagx_effect_negative_tbl);
			break;
		}
	case CAMERA_EFFECT_SEPIA:
		{
			pr_err("--CAMERA-- %s ...CAMERA_EFFECT_SEPIA\n",__func__);
			rc = S5K5CAGXCore_WritePREG(s5k5cagx_effect_sepia_tbl);
			break;
		}

	default:
		{
			pr_err("--CAMERA-- %s ...Default(Not Support)\n",__func__);
		}
	}
	//s5k5cagx_effect = effect;
	//Refresh Sequencer /
	pr_err("--CAMERA-- %s ...(End)\n",__func__);
#endif
	return rc;
}

#if 1
static int s5k5cagx_set_brightness(int8_t brightness)
{
	int rc = 0;
	pr_err("--CAMERA-- %s ...(Start)\n",__func__);
	pr_err("--CAMERA-- %s ...brightness = %d\n",__func__ , brightness);
#if 1
	switch (brightness)
	{
	case CAMERA_BRIGHTNESS_0:
		pr_err("--CAMERA--CAMERA_BRIGHTNESS_0\n");
		rc = S5K5CAGXCore_WritePREG(s5k5cagx_brightness_0_tbl);  					
		break;

	case CAMERA_BRIGHTNESS_1:
		pr_err("--CAMERA--CAMERA_BRIGHTNESS_1\n");
		rc = S5K5CAGXCore_WritePREG(s5k5cagx_brightness_1_tbl);  					
		break;

	case CAMERA_BRIGHTNESS_2:
		pr_err("--CAMERA--CAMERA_BRIGHTNESS_2\n");
		rc = S5K5CAGXCore_WritePREG(s5k5cagx_brightness_2_tbl);					
		break;

	case CAMERA_BRIGHTNESS_3:
		pr_err("--CAMERA--CAMERA_BRIGHTNESS_3\n");
		rc = S5K5CAGXCore_WritePREG(s5k5cagx_brightness_3_tbl);					
		break;

	case CAMERA_BRIGHTNESS_4:
		pr_err("--CAMERA--CAMERA_BRIGHTNESS_4\n");
		rc = S5K5CAGXCore_WritePREG(s5k5cagx_brightness_4_tbl);		
		break;

	case CAMERA_BRIGHTNESS_5:
		pr_err("--CAMERA--CAMERA_BRIGHTNESS_LV5\n");
		rc = S5K5CAGXCore_WritePREG(s5k5cagx_brightness_5_tbl);	
		break;

	case CAMERA_BRIGHTNESS_6:
		pr_err("--CAMERA--CAMERA_BRIGHTNESS_6\n");
		rc = S5K5CAGXCore_WritePREG(s5k5cagx_brightness_6_tbl);	
		break;

	default:
		pr_err("--CAMERA--CAMERA_BRIGHTNESS_ERROR COMMAND\n");
		break;
	}
#endif
	pr_err("--CAMERA-- %s ...(End)\n",__func__);
	return rc;
}
#endif
#if 1
static int s5k5cagx_set_contrast(int contrast)
{
	int rc = 0;
	pr_err("--CAMERA-- %s ...(Start)\n",__func__);
	pr_err("--CAMERA-- %s ...contrast = %d\n",__func__ , contrast);
#if 1
	//if (effect_value == CAMERA_EFFECT_OFF)
	{
		switch (contrast)
		{
		case CAMERA_CONTRAST_0:
			pr_err("--CAMERA--CAMERA_CONTRAST_LV0\n");
			rc = S5K5CAGXCore_WritePREG(s5k5cagx_contrast_0_tbl);
			break;
		case CAMERA_CONTRAST_1:
			pr_err("--CAMERA--CAMERA_CONTRAST_LV1\n");
			rc = S5K5CAGXCore_WritePREG(s5k5cagx_contrast_1_tbl);
			break;
		case CAMERA_CONTRAST_2:
			pr_err("--CAMERA--CAMERA_CONTRAST_LV2\n");
			rc = S5K5CAGXCore_WritePREG(s5k5cagx_contrast_2_tbl);
			break;
		case CAMERA_CONTRAST_3:
			pr_err("--CAMERA--CAMERA_CONTRAST_LV3\n");
			rc = S5K5CAGXCore_WritePREG(s5k5cagx_contrast_3_tbl);
			break;
		case CAMERA_CONTRAST_4:
			pr_err("--CAMERA--CAMERA_CONTRAST_LV4\n");
			rc = S5K5CAGXCore_WritePREG(s5k5cagx_contrast_4_tbl);
			break;
		default:
			pr_err("--CAMERA--CAMERA_CONTRAST_ERROR COMMAND\n");
			break;
		}
	}
	pr_err("--CAMERA-- %s ...(End)\n",__func__);
	 /*
      * Attention
      *
      * Time delay of 100ms or more is required by sensor,
      *
      * Contrast config will have no effect after setting 
      * without time delay of 100ms or more
      */
    mdelay(100);
	#endif

	return rc;
}
#endif

#if 1
static int s5k5cagx_set_sharpness(int sharpness)
{
	int rc = 0;
	pr_err("--CAMERA-- %s ...(Start)\n",__func__);
	pr_err("--CAMERA-- %s ...sharpness = %d\n",__func__ , sharpness);
#if 1
	{
		switch(sharpness)
		{
		case CAMERA_SHARPNESS_0:
			pr_err("--CAMERA--CAMERA_SHARPNESS_0\n");
			rc = S5K5CAGXCore_WritePREG(s5k5cagx_sharpness_0_tbl);			
			break;
		case CAMERA_SHARPNESS_1:
			pr_err("--CAMERA--CAMERA_SHARPNESS_1\n");
			rc = S5K5CAGXCore_WritePREG(s5k5cagx_sharpness_1_tbl);			

			break;
		case CAMERA_SHARPNESS_2:
			pr_err("--CAMERA--CAMERA_SHARPNESS_2\n");
			rc = S5K5CAGXCore_WritePREG(s5k5cagx_sharpness_2_tbl);			

			break;
		case CAMERA_SHARPNESS_3:
			pr_err("--CAMERA--CAMERA_SHARPNESS_3\n");
			rc = S5K5CAGXCore_WritePREG(s5k5cagx_sharpness_3_tbl);			

			break;
		case CAMERA_SHARPNESS_4:
			pr_err("--CAMERA--CAMERA_SHARPNESS_4\n");
			rc = S5K5CAGXCore_WritePREG(s5k5cagx_sharpness_4_tbl);			

			break;
		case CAMERA_SHARPNESS_5:
			pr_err("--CAMERA--CAMERA_SHARPNESS_5\n");
			rc = S5K5CAGXCore_WritePREG(s5k5cagx_sharpness_5_tbl);			

			break;
		case CAMERA_SHARPNESS_6:
			pr_err("--CAMERA--CAMERA_SHARPNESS_6\n");
			rc = S5K5CAGXCore_WritePREG(s5k5cagx_sharpness_6_tbl);			

			break;
		case CAMERA_SHARPNESS_7:
			pr_err("--CAMERA--CAMERA_SHARPNESS_7\n");
			rc = S5K5CAGXCore_WritePREG(s5k5cagx_sharpness_7_tbl);			

			break;
		case CAMERA_SHARPNESS_8:
			pr_err("--CAMERA--CAMERA_SHARPNESS_8\n");
			rc = S5K5CAGXCore_WritePREG(s5k5cagx_sharpness_8_tbl);
			break;
			
		case CAMERA_SHARPNESS_9:
			pr_err("--CAMERA--CAMERA_SHARPNESS_9\n");
			rc = S5K5CAGXCore_WritePREG(s5k5cagx_sharpness_9_tbl);
			break;
		case CAMERA_SHARPNESS_10:
			pr_err("--CAMERA--CAMERA_SHARPNESS_10\n");
			rc = S5K5CAGXCore_WritePREG(s5k5cagx_sharpness_10_tbl);
			break;
		default:
			pr_err("--CAMERA--CAMERA_SHARPNESS_ERROR COMMAND\n");
			break;
		}
	}
#endif
	pr_err("--CAMERA-- %s ...(End)\n",__func__);
	return rc;
}
#endif

#if 1
static int s5k5cagx_set_iso(int saturation)
{
	long rc = 0;
	//int i = 0;
	pr_err("--CAMERA-- %s ...(Start)\n",__func__);
	pr_err("--CAMERA-- %s ...iso = %d\n",__func__ , saturation);
#if 1
	//if (effect_value == CAMERA_EFFECT_OFF)
	{ 
		switch (saturation)
		{
		case CAMERA_ISO_SET_AUTO:
			pr_err("--CAMERA--CAMERA_ISO_SET_AUTO\n");
			rc = S5K5CAGXCore_WritePREG(s5k5cagx_iso_auto_tbl);
			break;
		case CAMERA_ISO_SET_100:
			pr_err("--CAMERA--CAMERA_ISO_SET_100\n");
			rc = S5K5CAGXCore_WritePREG(s5k5cagx_iso_100_tbl);

			break;
		case CAMERA_ISO_SET_200:
			pr_err("--CAMERA--CAMERA_ISO_SET_200\n");
			rc = S5K5CAGXCore_WritePREG(s5k5cagx_iso_200_tbl);

			break;
		case CAMERA_ISO_SET_400:
			pr_err("--CAMERA--CAMERA_ISO_SET_400\n");
			rc = S5K5CAGXCore_WritePREG(s5k5cagx_iso_400_tbl);

			break;
		case CAMERA_ISO_SET_800:
			pr_err("--CAMERA--CAMERA_ISO_SET_800\n");
			rc = S5K5CAGXCore_WritePREG(s5k5cagx_iso_800_tbl);

			break;
		
		default:
			pr_err("--CAMERA--CAMERA_SATURATION_ERROR COMMAND\n");
			break;
		}
	}	

 #endif 	
	pr_err("--CAMERA-- %s ...(End)\n",__func__);

	return rc;
}
#endif

#if 1
static int s5k5cagx_set_saturation(int saturation)
{
	long rc = 0;
	//int i = 0;
	pr_err("--CAMERA-- %s ...(Start)\n",__func__);
	pr_err("--CAMERA-- %s ...saturation = %d\n",__func__ , saturation);
#if 1
	//if (effect_value == CAMERA_EFFECT_OFF)
	{ 
		switch (saturation)
		{
		case CAMERA_SATURATION_0:
			pr_err("--CAMERA--CAMERA_SATURATION_0\n");
			rc = S5K5CAGXCore_WritePREG(s5k5cagx_saturation_0_tbl);
			break;
		case CAMERA_SATURATION_1:
			pr_err("--CAMERA--CAMERA_SATURATION_1\n");
			rc = S5K5CAGXCore_WritePREG(s5k5cagx_saturation_1_tbl);

			break;
		case CAMERA_SATURATION_2:
			pr_err("--CAMERA--CAMERA_SATURATION_2\n");
			rc = S5K5CAGXCore_WritePREG(s5k5cagx_saturation_2_tbl);

			break;
		case CAMERA_SATURATION_3:
			pr_err("--CAMERA--CAMERA_SATURATION_3\n");
			rc = S5K5CAGXCore_WritePREG(s5k5cagx_saturation_3_tbl);

			break;
		case CAMERA_SATURATION_4:
			pr_err("--CAMERA--CAMERA_SATURATION_4\n");
			rc = S5K5CAGXCore_WritePREG(s5k5cagx_saturation_4_tbl);

			break;
		
		default:
			pr_err("--CAMERA--CAMERA_SATURATION_ERROR COMMAND\n");
			break;
		}
	}	
	pr_err("--CAMERA-- %s ...(End)\n",__func__);
     /*
      * Attention
      *
      * Time delay of 100ms or more is required by sensor,
      *
      * Saturation config will have no effect after setting 
      * without time delay of 100ms or more
      */
    mdelay(100);
 #endif 	

	return rc;
}
#endif
#if 1
static long s5k5cagx_set_antibanding(int antibanding)
{
	long rc = 0;
	//int i = 0;
	pr_err("--CAMERA-- %s ...(Start)\n", __func__);
	pr_err("--CAMERA-- %s ...antibanding = %d\n", __func__, antibanding);
#if 1
	switch (antibanding)
	{
	case CAMERA_ANTIBANDING_SET_OFF:
		pr_err("--CAMERA--CAMERA_ANTIBANDING_SET_OFF\n");
		break;

	case CAMERA_ANTIBANDING_SET_60HZ:
		pr_err("--CAMERA--CAMERA_ANTIBANDING_60HZ\n");
		rc = S5K5CAGXCore_WritePREG(s5k5cagx_antibanding_60z_tbl);
		break;

	case CAMERA_ANTIBANDING_SET_50HZ:
		pr_err("--CAMERA--CAMERA_ANTIBANDING_50HZ\n");
		rc = S5K5CAGXCore_WritePREG(s5k5cagx_antibanding_50z_tbl);
		break;

	case CAMERA_ANTIBANDING_SET_AUTO:
		pr_err("--CAMERA--CAMERA_ANTIBANDING_AUTO\n");
		rc = S5K5CAGXCore_WritePREG(s5k5cagx_antibanding_auto_tbl);
		break;

	default:
		pr_err("--CAMERA--CAMERA_ANTIBANDING_ERROR COMMAND\n");
		break;
	}
	#endif
	pr_err("--CAMERA-- %s ...(End)\n",__func__);
	return rc;
}
#endif
#if 0
static long s5k5cagx_set_exposure_mode(int mode)
{
	long rc = 0;
	//int i =0;
	pr_err("--CAMERA-- %s ...(Start)\n",__func__);
	pr_err("--CAMERA-- %s ...mode = %d\n",__func__ , mode);
#if 0	
	switch (mode)
	{
	case CAMERA_SETAE_AVERAGE:
		pr_err("--CAMERA--CAMERA_SETAE_AVERAGE\n");
		rc = S5K5CAGXCore_WritePREG(s5k5cagx_ae_average_tbl);
		break;
	case CAMERA_SETAE_CENWEIGHT:
		pr_err("--CAMERA--CAMERA_SETAE_CENWEIGHT\n");
		rc = S5K5CAGXCore_WritePREG(s5k5cagx_ae_centerweight_tbl);		
		break;
	default:
		pr_err("--CAMERA--ERROR COMMAND OR NOT SUPPORT\n");
		break;
	}
#endif
	pr_err("--CAMERA-- %s ...(End)\n",__func__);
	return rc;
}
#endif
#if 0
static int32_t s5k5cagx_lens_shading_enable(uint8_t is_enable)
{
	int32_t rc = 0;
	pr_err("--CAMERA--%s: ...(Start). enable = %d\n", __func__, is_enable);

	if(is_enable)
	{
		pr_err("%s: enable~!!\n", __func__);
		rc = S5K5CAGXCore_WritePREG(s5k5cagx_lens_shading_on_tbl);
	}
	else
	{
		pr_err("%s: disable~!!\n", __func__);
		rc = S5K5CAGXCore_WritePREG(s5k5cagx_lens_shading_off_tbl);
	}
	pr_err("--CAMERA--%s: ...(End). rc = %d\n", __func__, rc);
	return rc;
}
#endif
static int s5k5cagx_set_sensor_mode(int mode, int res)
{
	int rc = 0;

	pr_err("--CAMERA-- s5k5cagx_set_sensor_mode mode = %d, res = %d\n", mode, res);

	switch (mode)
	{
	case SENSOR_PREVIEW_MODE:
		pr_err("--CAMERA-- SENSOR_PREVIEW_MODE\n");
		rc = s5k5cagx_setting(S_UPDATE_PERIODIC, S_RES_PREVIEW);
		break;

	case SENSOR_SNAPSHOT_MODE:
		pr_err("--CAMERA-- SENSOR_SNAPSHOT_MODE\n");
		rc = s5k5cagx_setting(S_UPDATE_PERIODIC, S_RES_CAPTURE);
		break;

	case SENSOR_RAW_SNAPSHOT_MODE:
	default:
		pr_err("--CAMERA--s5k5cagx_set_sensor_mode no support\n");
		rc = -EINVAL;
		break;
	}

	return rc;
}

/*
 * Auto Focus Trigger
 */
static int32_t s5k5cagx_af_trigger(void)
{

    int32_t rc = 0;
	#if 1
    u32 af_status = 0x0001;
    u32 af_2nd_status = 0x0100;    //added by JSK SEMCO
    uint32_t i = 0;
   // int retry = 0;

    CDBG("%s: entry\n", __func__);
    rc = S5K5CAGXCore_WritePREG(s5k5cagx_afinit_tbl);
    if (rc < 0)
    {
        return rc;
    }
    mdelay(300);

    /*
      * af_status: 0, AF is done successfully
      *            1, AF is being done
      */
  
    for (i = 0; i < 200; ++i)
    {       
        mdelay(100);// 100
        rc = s5k5cagx_i2c_write(s5k5cagx_client->addr,0xFCFC, 0xD000, TRY_TIME);
        if (rc < 0)
        {
         return rc;
        }
        rc = s5k5cagx_i2c_write(s5k5cagx_client->addr,0x002C, 0x7000, TRY_TIME);
        if (rc < 0)
        {
         return rc;
        }
        rc = s5k5cagx_i2c_write(s5k5cagx_client->addr,0x002E, 0x26FE, TRY_TIME);
        if (rc < 0)
        {
         return rc;
        }
        rc = s5k5cagx_i2c_read(s5k5cagx_client->addr, 0x0F12, &af_status);
        if (rc < 0)
        {
         return rc;
        }
  		pr_err(" %s ,i=%d,af_status=%d\r\n",__func__,i,af_status);
        /*af_statue */			    
        /* 0x0001 : Searching */
        /* 0x0002 : AF Fail   */ 
        /* 0x0003 : AF Success*/ 
        if ((0x0002 == af_status)||(0x0003 == af_status))
        {
            break;
        }
    }

	pr_err(" %s ,af_status=%d\r\n",__func__,af_status);
    if (0x0002 == af_status) //need 2nd Scan
    {
        //mdelay(100);
        for(i = 0; i < 200; ++i)
        {
            mdelay(100);
            rc = s5k5cagx_i2c_write(s5k5cagx_client->addr,0xFCFC, 0xD000, TRY_TIME);
            if (rc < 0)
            {
                  return rc;
            }
            rc = s5k5cagx_i2c_write(s5k5cagx_client->addr,0x002C, 0x7000, TRY_TIME);
            if (rc < 0)
            {
                  return rc;
            }
            rc = s5k5cagx_i2c_write(s5k5cagx_client->addr,0x002E, 0x1B2F, TRY_TIME);
            if (rc < 0)
            {
                  return rc;
            }
            rc = s5k5cagx_i2c_read(s5k5cagx_client->addr, 0x0F12, &af_2nd_status);
            if (rc < 0)
            {
                  return rc;
            }
			pr_err(" %s ,22 i=%d,af_status=%d\r\n",__func__,i,af_2nd_status);
            /*af_2nd_statue */       
            /* 0x01xx : 2nd scan not finish  */
            /* 0x00xx : 2nd scan finish */ 
            if(0x0000 == (af_2nd_status & 0xFF00))
            {
                mdelay(100);//mdelay(100);
                return 0;   
            }                   
        }
    }
    else if(0x0003 == af_status) //don't need 2nd scan
    {
        mdelay(100);//mdelay(100);
        return -EIO;//return 0;         
    }
    /*
      * AF is failed
      */
    pr_err(" %s FAILED!\r\n",__func__);  
    return -EIO;
	#endif
	
}

#if 1
static int s5k5cagx_set_wb_oem(uint8_t wb_mode)
{
	int rc = 0;
#if 1
	pr_err("[kylin] %s \r\n",__func__ );
	pr_err("--CAMERA-- %s ...wb_mode = %d\n", __func__, wb_mode);
#if 0
    unsigned int tmp2;
	s5k5cagx_i2c_read_byte(s5k5cagx_client->addr, 0x350b, &tmp2);
	pr_err("--CAMERA-- GAIN VALUE : %x\n", tmp2);
#endif
	switch(wb_mode)
	{
	case CAMERA_WB_MODE_AWB:
		pr_err("--CAMERA--CAMERA_WB_MODE_AWB\n");
		rc = S5K5CAGXCore_WritePREG(s5k5cagx_wb_auto);
		break;

	case CAMERA_WB_MODE_SUNLIGHT:
		pr_err("--CAMERA--CAMERA_WB_MODE_SUNLIGHT\n");
		rc = S5K5CAGXCore_WritePREG(s5k5cagx_wb_daylight);   
		break;
	case CAMERA_WB_MODE_INCANDESCENT:
		pr_err("--CAMERA--CAMERA_WB_MODE_INCANDESCENT\n");
		rc = S5K5CAGXCore_WritePREG(s5k5cagx_wb_incandescent);   	
		break;
	case CAMERA_WB_MODE_FLUORESCENT:
		pr_err("--CAMERA--CAMERA_WB_MODE_FLUORESCENT\n");
		rc = S5K5CAGXCore_WritePREG(s5k5cagx_wb_flourescant);   		
		break;
	case CAMERA_WB_MODE_CLOUDY:
		pr_err("--CAMERA--CAMERA_WB_MODE_CLOUDY\n");
		rc = S5K5CAGXCore_WritePREG(s5k5cagx_wb_cloudy);   	
		break;
	default:
		break;
	}
	    /*
      * Attention
      *
      * Time delay of 100ms or more is required by sensor,
      *
      * WB config will have no effect after setting 
      * without time delay of 100ms or more
      */
    mdelay(100);
#endif

	return rc;
}
#endif
#if 0
static int s5k5cagx_set_touchaec(uint32_t x,uint32_t y)
{
	uint8_t aec_arr[8] = {0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11};
	int idx = 0;
	int i;
	pr_err("[kylin] %s x: %d ,y: %d\r\n",__func__ ,x,y);
	idx = x /2 + y *2;
	pr_err("[kylin] idx: %d\r\n",idx);

	if (x %2 == 0)
	{
		aec_arr[idx] = 0x10 | 0x0a;
	}
	else
	{
		aec_arr[idx] = 0x01 | 0xa0;
	}

	for (i = 0;i < 8; i++) 
	{
		pr_err("write : %x val : %x ", 0x5688 + i, aec_arr[i]); 
		s5k5cagx_i2c_write(s5k5cagx_client->addr, 0x5688 + i, aec_arr[i], 10);
	}

	return 1;
}
#endif
//QRD
#if 1
static int s5k5cagx_set_exposure_compensation(int compensation)
{
	long rc = 0;
#if 1

	pr_err("--CAMERA-- %s ...(Start)\n",__func__);

	pr_err("--CAMERA-- %s ...exposure_compensation = %d\n",__func__ , compensation);

	switch(compensation)
	{
case CAMERA_EXPOSURE_0:
	pr_err("--CAMERA--CAMERA_EXPOSURE_COMPENSATION_0\n");
	rc = S5K5CAGXCore_WritePREG(s5k5cagx_exposure_compensation_0_tbl);   		
	break;
case CAMERA_EXPOSURE_1:
	pr_err("--CAMERA--CAMERA_EXPOSURE_COMPENSATION_1\n");
	rc = S5K5CAGXCore_WritePREG(s5k5cagx_exposure_compensation_1_tbl);   		
	break;
case CAMERA_EXPOSURE_2:
	pr_err("--CAMERA--CAMERA_EXPOSURE_COMPENSATION_2\n");
	rc = S5K5CAGXCore_WritePREG(s5k5cagx_exposure_compensation_2_tbl);   		
	break;
case CAMERA_EXPOSURE_3:
	pr_err("--CAMERA--CAMERA_EXPOSURE_COMPENSATION_3\n");
	rc = S5K5CAGXCore_WritePREG(s5k5cagx_exposure_compensation_3_tbl);   		
	break;
case CAMERA_EXPOSURE_4:
	pr_err("--CAMERA--CAMERA_EXPOSURE_COMPENSATION_4\n");
	rc = S5K5CAGXCore_WritePREG(s5k5cagx_exposure_compensation_4_tbl);   		
	break;
default:
	pr_err("--CAMERA--ERROR CAMERA_EXPOSURE_COMPENSATION\n");
	break;
	}

	pr_err("--CAMERA-- %s ...(End)\n",__func__);
	    mdelay(100);
#endif

	return rc;
}
#endif
#if 0
static int s5k5cagx_sensor_start_af(void)
{
	int i;
	unsigned int af_st = 0;
	unsigned int af_ack = 0;
	unsigned int tmp = 0;
	int rc = 0;
	pr_err("--CAMERA-- %s (Start...)\n",__func__);

	s5k5cagx_i2c_read_byte(s5k5cagx_client->addr, S5K5CAGX_CMD_FW_STATUS,&af_st);
	pr_err("--CAMERA-- %s af_st = %d\n", __func__, af_st);

	s5k5cagx_i2c_write(s5k5cagx_client->addr, S5K5CAGX_CMD_ACK, 0x01, 10);
	s5k5cagx_i2c_write(s5k5cagx_client->addr, S5K5CAGX_CMD_MAIN, 0x03, 10);

	for (i = 0; i < 50; i++) {
		s5k5cagx_i2c_read_byte(s5k5cagx_client->addr,S5K5CAGX_CMD_ACK,&af_ack);
		if (af_ack == 0)
			break;
		msleep(50);
	}
	pr_err("--CAMERA-- %s af_ack = 0x%x\n", __func__, af_ack);

	//	if(af_ack == 0)
	{
		//		mdelay(1000);
		s5k5cagx_i2c_read_byte(s5k5cagx_client->addr, S5K5CAGX_CMD_FW_STATUS,&af_st);
		pr_err("--CAMERA-- %s af_st = %d\n", __func__, af_st);

		if (af_st == 0x10)
		{
			pr_err("--CAMERA-- %s AF ok and release AF setting~!!\n", __func__);
		}
		else {
			pr_err("--CAMERA-- %s AF not ready!!\n", __func__);
		}
	}

	//  s5k5cagx_i2c_write(s5k5cagx_client->addr,S5K5CAGX_CMD_ACK,0x01,10);
	//  s5k5cagx_i2c_write(s5k5cagx_client->addr,S5K5CAGX_CMD_MAIN,0x08,10);
	s5k5cagx_i2c_write(s5k5cagx_client->addr,S5K5CAGX_CMD_ACK,0x01,10);
	s5k5cagx_i2c_write(s5k5cagx_client->addr,S5K5CAGX_CMD_MAIN,0x07,10);

	for (i = 0; i < 70; i++)
	{
		s5k5cagx_i2c_read_byte(s5k5cagx_client->addr, S5K5CAGX_CMD_ACK, &af_ack);
		if (af_ack == 0)
			break;
		msleep(25);
	}

	s5k5cagx_i2c_read_byte(s5k5cagx_client->addr, S5K5CAGX_CMD_PARA0, &tmp);
	pr_err("0x3024 = %x \n", tmp);
	rc = ((tmp == 0) ? 1 : 0);

	s5k5cagx_i2c_read_byte(s5k5cagx_client->addr, S5K5CAGX_CMD_PARA1, &tmp);
	pr_err("0x3025 = %x \n", tmp);
	rc = ((tmp == 0) ? 1 : 0);

	s5k5cagx_i2c_read_byte(s5k5cagx_client->addr, S5K5CAGX_CMD_PARA2, &tmp);
	pr_err("0x3026 = %x \n", tmp);
	rc = ((tmp == 0) ? 1 : 0);

	s5k5cagx_i2c_read_byte(s5k5cagx_client->addr, S5K5CAGX_CMD_PARA3, &tmp);
	pr_err("0x3027 = %x \n", tmp);
	rc = ((tmp == 0) ? 1 : 0) ;

	s5k5cagx_i2c_read_byte(s5k5cagx_client->addr, S5K5CAGX_CMD_PARA4, &tmp);
	pr_err("0x3028 = %x \n", tmp);
	rc = ((tmp == 0) ? 1 : 0) ;

	pr_err("--CAMERA-- %s rc = %d(End...)\n", __func__, rc);
	return rc;
}
#endif
static int s5k5cagx_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cdata;
	long rc = 0;

	if (copy_from_user(&cdata, (void *)argp, sizeof(struct sensor_cfg_data))) 
		return -EFAULT;

	pr_err("--CAMERA-- %s %d\n",__func__,cdata.cfgtype);

	mutex_lock(&s5k5cagx_mutex);
	switch (cdata.cfgtype)
	{
	case CFG_SET_MODE:   // 0
		rc =s5k5cagx_set_sensor_mode(cdata.mode, cdata.rs);
		break;
	case CFG_SET_EFFECT: // 1
		pr_err("--CAMERA-- CFG_SET_EFFECT mode=%d, effect = %d !!\n",cdata.mode, cdata.cfg.effect);
		rc = s5k5cagx_set_effect(cdata.mode, cdata.cfg.effect);
		break;
	case CFG_START:      // 2
		pr_err("--CAMERA-- CFG_START (Not Support) !!\n");
		// Not Support
		break;
	case CFG_PWR_UP:     // 3
		pr_err("--CAMERA-- CFG_PWR_UP (Not Support) !!\n");
		// Not Support
		break;
	case CFG_PWR_DOWN:   // 4
		pr_err("--CAMERA-- CFG_PWR_DOWN (Not Support) \n");
		s5k5cagx_power_off();
		break;
	case CFG_SET_DEFAULT_FOCUS:  // 06
		pr_err("--CAMERA-- CFG_SET_DEFAULT_FOCUS (Not Implement) !!\n");
		break;        
	case CFG_MOVE_FOCUS:     //  07
		pr_err("--CAMERA-- CFG_MOVE_FOCUS (Not Implement) !!\n");
		break;
	case CFG_SET_BRIGHTNESS:     //  12
		pr_err("--CAMERA-- CFG_SET_BRIGHTNESS  !!\n");
		rc = s5k5cagx_set_brightness(cdata.cfg.brightness);
		break;
	case CFG_SET_CONTRAST:     //  13
		pr_err("--CAMERA-- CFG_SET_CONTRAST  !!\n");
		rc = s5k5cagx_set_contrast(cdata.cfg.contrast);
		break;  

	case CFG_SET_ANTIBANDING:     //  17
		pr_err("--CAMERA-- CFG_SET_ANTIBANDING antibanding = %d!!\n", cdata.cfg.antibanding);
		rc = s5k5cagx_set_antibanding(cdata.cfg.antibanding);
		break;
			
	case CFG_SET_SATURATION:     //  30
		pr_err("--CAMERA-- CFG_SET_SATURATION !!\n");
		rc = s5k5cagx_set_saturation(cdata.cfg.saturation);
		break;

	case CFG_SET_SHARPNESS:     //  31
		pr_err("--CAMERA-- CFG_SET_SHARPNESS !!\n");
		rc = s5k5cagx_set_sharpness(cdata.cfg.sharpness);
		break;

	case CFG_SET_WB:
		pr_err("--CAMERA-- CFG_SET_WB!!\n");
		s5k5cagx_set_wb_oem(cdata.cfg.wb_mode);
		rc = 0 ;
		break;
	
	case CFG_SET_AF:
		pr_err("--CAMERA-- CFG_SET_AUTO_FOCUS !\n");
		rc = s5k5cagx_af_trigger();
		break;
			
	case CFG_SET_EXPOSURE_COMPENSATION:
		pr_err("--CAMERA-- CFG_SET_EXPOSURE_COMPENSATION !\n");
		rc = s5k5cagx_set_exposure_compensation(cdata.cfg.exp_compensation);
		break;

	case CFG_SET_ISO:
		pr_err("--CAMERA-- CFG_SET_EXPOSURE_COMPENSATION !\n");
		rc = s5k5cagx_set_iso(cdata.cfg.iso_val);
		break;
		
#if 0
	case CFG_SET_EXPOSURE_MODE:     //  15
		pr_err("--CAMERA-- CFG_SET_EXPOSURE_MODE !!\n");
		//rc = s5k5cagx_set_exposure_mode(cdata.cfg.ae_mode);
		break;
	case CFG_SET_LENS_SHADING:     //  20
		pr_err("--CAMERA-- CFG_SET_LENS_SHADING !!\n");
		rc = s5k5cagx_lens_shading_enable(cdata.cfg.lens_shading);
		break;
	case CFG_SET_TOUCHAEC:
		pr_err("--CAMERA-- CFG_SET_TOUCHAEC!!\n");
		s5k5cagx_set_touchaec(cdata.cfg.aec_cord.x, cdata.cfg.aec_cord.y);
		rc = 0 ;
		break;
	case CFG_SET_AUTO_FOCUS:
		pr_err("--CAMERA-- CFG_SET_AUTO_FOCUS !\n");
		rc = s5k5cagx_sensor_start_af();
		break;
	case CFG_SET_AUTOFLASH:
		pr_err("--CAMERA-- CFG_SET_AUTOFLASH !\n");
		is_autoflash = cdata.cfg.is_autoflash;
		pr_err("[kylin] is autoflash %d\r\n",is_autoflash);
		rc = 0;
		break;

#endif	
	default:
		pr_err("--CAMERA-- %s: Command=%d (Not Implement)!!\n",__func__,cdata.cfgtype);
		rc = -EINVAL;
		break;    
	}
	mutex_unlock(&s5k5cagx_mutex);
	return rc;    
}

static struct i2c_driver s5k5cagx_i2c_driver = {
	.id_table = s5k5cagx_i2c_id,
	.probe  = s5k5cagx_i2c_probe,
	.remove = s5k5cagx_i2c_remove,
	.driver = {
		.name = "s5k5cagx",
	},
};

static int s5k5cagx_probe_init_gpio(const struct msm_camera_sensor_info *data)
{
	int rc = 0;

	pr_err("--CAMERA-- %s Open CAMIO CLK,set to default clk rate!\n",__func__);

	s5k5cagx_pwdn_gpio = data->sensor_pwd;
	s5k5cagx_reset_gpio = data->sensor_reset;

	pr_err("--CAMERA-- %s : sensor_pwd_pin=%d, sensor_reset_pin=%d\n",__func__,data->sensor_pwd,data->sensor_reset);

	rc = gpio_request(data->sensor_pwd, "s5k5cagx");
	pr_err("--CAMERA-- %s : gpio_request=%d, result is %d",__func__,data->sensor_pwd, rc);

	rc |= gpio_request(data->sensor_reset, "s5k5cagx");
	pr_err("--CAMERA-- %s : gpio_request=%d, result is %d",__func__,data->sensor_reset, rc);

	if (rc < 0)
	{
		gpio_free(data->sensor_pwd);
		gpio_free(data->sensor_reset);
		rc = gpio_request(data->sensor_pwd, "s5k5cagx");
		rc |= gpio_request(data->sensor_reset, "s5k5cagx");
	}

	//gpio_direction_output(data->sensor_reset, 1);
	//gpio_direction_output(data->sensor_pwd, 1);

	return rc;
}

static void s5k5cagx_probe_free_gpio(void)
{
	gpio_free(s5k5cagx_pwdn_gpio);
	gpio_free(s5k5cagx_reset_gpio);
	s5k5cagx_pwdn_gpio = 0xFF;
	s5k5cagx_reset_gpio = 0xFF;
}

static int s5k5cagx_sensor_probe(const struct msm_camera_sensor_info *info,struct msm_sensor_ctrl *s)
{
	int rc = -ENOTSUPP;
	pr_err("--CAMERA-- %s (Start...)\n",__func__);
	rc = i2c_add_driver(&s5k5cagx_i2c_driver);
	pr_err("--CAMERA-- i2c_add_driver ret:0x%x,s5k5cagx_client=0x%x\n",
		rc, (unsigned int)s5k5cagx_client);
	if ((rc < 0 ) || (s5k5cagx_client == NULL))
	{
		pr_err("--CAMERA-- i2c_add_driver FAILS!!\n");
		return rc;
	}
	

	rc = s5k5cagx_probe_init_gpio(info);

	//s5k5cagx_power_off();

    rc = msm_camera_power_backend(MSM_CAMERA_PWRUP_MODE);
    if (rc < 0)
    {
        pr_err("%s: camera_power_backend failed!\n", __func__);
        goto probe_fail;
    }
	
	/* SENSOR NEED MCLK TO DO I2C COMMUNICTION, OPEN CLK FIRST*/

#if defined(CONFIG_MACH_BLADE2)	|| defined(CONFIG_MACH_ATLAS40)	
	rc = msm_camera_clk_switch(info, S5K5CAGX_GPIO_SWITCH_CTL, S5K5CAGX_GPIO_SWITCH_VAL);
	if (rc < 0) {
		pr_err("msm_camera_clk_switch fail\n");
		return rc;
	}
#endif	
	
	msm_camio_clk_rate_set(24000000);

	mdelay(5);

	s5k5cagx_power_on();
	mdelay(25);
	
	s5k5cagx_power_reset();
	mdelay(10);
	
	rc = s5k5cagx_probe_readID(info);

    if (rc < 0)
    {
        pr_err("s5k5cagx_probe_readID failed!\n");
        goto probe_fail;
    }
	
#if 1
	s5k5cagx_ctrl = kzalloc(sizeof(struct __s5k5cagx_ctrl), GFP_KERNEL);
	if (!s5k5cagx_ctrl)
	{
		pr_err("--CAMERA-- kzalloc s5k5cagx_ctrl error !!\n");
		kfree(s5k5cagx_ctrl);
		return rc;
	}
 	rc = s5k5cagx_i2c_write(s5k5cagx_client->addr, 
                           REG_S5K5CAGX_MCNEX_QTECH_RESET_AND_MISC_CTRL, 
                           S5K5CAGX_MCNEX_QTECH_SOC_RESET,
                           10);
    if (rc < 0)
    {
        pr_err("soft reset failed!\n");
        goto probe_fail;
    }

	mdelay(1);

	rc = s5k5cagx_i2c_write(s5k5cagx_client->addr,
                           REG_S5K5CAGX_MCNEX_QTECH_RESET_AND_MISC_CTRL,
                           S5K5CAGX_MCNEX_QTECH_SOC_NOT_RESET & 0x0018,
                           10);
    if (rc < 0)
    {
        pr_err("soft reset failed!\n");
        goto probe_fail;
    }

	rc = s5k5cagx_i2c_write(s5k5cagx_client->addr,
                           REG_S5K5CAGX_MCNEX_QTECH_RESET_AND_MISC_CTRL,
                           S5K5CAGX_MCNEX_QTECH_SOC_NOT_RESET,
                           10);
    if (rc < 0)
    {
        pr_err("soft reset failed!\n");
        goto probe_fail;
    }

	mdelay(10);

	rc = s5k5cagx_setting(S_REG_INIT, S_RES_PREVIEW);
	
	if (rc < 0)
	{
		pr_err("--CAMERA-- %s : s5k5cagx_setting failed. rc = %d\n",__func__,rc);
		kfree(s5k5cagx_ctrl);
		goto probe_fail;
	}
#endif

	if (rc < 0)
	{ 
		pr_err("--CAMERA--s5k5cagx_probe_readID Fail !!~~~~!!\n");
		pr_err("--CAMERA-- %s, unregister\n",__func__);

		i2c_del_driver(&s5k5cagx_i2c_driver);
		s5k5cagx_power_off();
		s5k5cagx_probe_free_gpio();
		goto probe_fail;;
	}

	s->s_init = s5k5cagx_sensor_open_init;
	s->s_release = s5k5cagx_sensor_release;
	s->s_config  = s5k5cagx_sensor_config;
	//s->s_AF = s5k5cagx_sensor_set_af;
	//camera_init_flag = true;

	s->s_camera_type = BACK_CAMERA_2D;
	s->s_mount_angle = info->sensor_platform_info->mount_angle;

	//s5k5cagx_power_off();
    s5k5cagx_sensor_release();

	pr_err("--CAMERA-- %s (End...)\n",__func__);
	return 0;
probe_fail:
	pr_err("s5k5cagx_sensor_probe: SENSOR PROBE FAILS!\n");
	i2c_del_driver(&s5k5cagx_i2c_driver);
	s5k5cagx_power_off();
	s5k5cagx_probe_free_gpio();
	return rc;
}

static int s5k5cagx_i2c_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
pr_err("--CAMERA-- %s ... (Start...)\n",__func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		pr_err("--CAMERA--i2c_check_functionality failed\n");
		return -ENOMEM;
	}

	s5k5cagx_sensorw = kzalloc(sizeof(struct s5k5cagx_work), GFP_KERNEL);
	if (!s5k5cagx_sensorw)
	{
		pr_err("--CAMERA--kzalloc failed\n");
		return -ENOMEM;
	}
	i2c_set_clientdata(client, s5k5cagx_sensorw);
	s5k5cagx_init_client(client);
	s5k5cagx_client = client;

	pr_err("--CAMERA-- %s ... (End...)\n",__func__);
	return 0;
}

static int __s5k5cagx_probe(struct platform_device *pdev)
{
/*
 * ZTE_CAM_LJ_20120310
 * Get FTM flag to adjust 
 * the initialize process 
 * of camera
 */
#ifdef CONFIG_ZTE_PLATFORM
#ifdef CONFIG_ZTE_FTM_FLAG_SUPPORT
    if(zte_get_ftm_flag())
    {
        return 0;
    }
#endif
#endif
#if 0
	return msm_camera_drv_start(pdev, s5k5cagx_sensor_probe);
#else
	return msm_camera_drv_start(pdev, s5k5cagx_sensor_probe, 0);
#endif
}

static struct platform_driver msm_camera_driver = {
	.probe = __s5k5cagx_probe,
	.driver = {
		.name = "msm_camera_s5k5cagx",
		.owner = THIS_MODULE,
	},
};

static int __init s5k5cagx_init(void)
{
	//s5k5cagx_i2c_buf[0]=0x5A;
	return platform_driver_register(&msm_camera_driver);
}

module_init(s5k5cagx_init);

MODULE_DESCRIPTION("S5K5CAGX YUV MIPI sensor driver");
MODULE_LICENSE("GPL v2");
