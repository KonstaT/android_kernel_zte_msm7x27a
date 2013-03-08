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
#include "ov5640.h"
#include <linux/slab.h>

static int ov5640_pwdn_gpio;
static int ov5640_reset_gpio;

#define FALSE 0
#define TRUE 1

#if 1
//static int m_60Hz = FALSE;
static unsigned int ov5640_preview_exposure;
static uint16_t ov5640_gain;
static unsigned short ov5640_preview_maxlines;

//ZTE_YGL_CAM_20111221,modifed for SNR
uint16_t YAVG; //kenxu add for improve noise.
uint16_t WB_T; //neil add for detect wb temperature
#endif


/* ZTE_YGL_CAM_20111230
 * add the interface of touch AF 
 */
#define OV5640_AF_WINDOW_FULL_WIDTH  80//64
#define OV5640_AF_WINDOW_FULL_HEIGHT 60//48

uint16_t x_ratio = 0;
uint16_t y_ratio = 0;
int32_t ov5640_TouchAF_x = -1;
int32_t ov5640_TouchAF_y = -1;
static int OV5640_CSI_CONFIG = 0;

struct ov5640_work {
	struct work_struct work;
};
static struct ov5640_work *ov5640_sensorw;
static struct i2c_client    *ov5640_client;
static DECLARE_WAIT_QUEUE_HEAD(ov5640_wait_queue);
DEFINE_MUTEX(ov5640_mutex);

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


static u8 ov5640_i2c_buf[4];
static u8 ov5640_counter = 0;
static int16_t ov5640_effect = CAMERA_EFFECT_OFF;
static int is_autoflash = 0;
static int effect_value = 0;
//static unsigned int SAT_U = 0x40;
//static unsigned int SAT_V = 0x40;

#define OV5640_GPIO_SWITCH_VAL     0
#define OV5640_GPIO_SWITCH_CTL    49


struct __ov5640_ctrl 
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
static struct __ov5640_ctrl *ov5640_ctrl;
static int afinit = 1;

extern struct rw_semaphore leds_list_lock;
extern struct list_head leds_list;


static int ov5640_i2c_remove(struct i2c_client *client);
static int ov5640_i2c_probe(struct i2c_client *client,const struct i2c_device_id *id);

static int ov5640_i2c_txdata(u16 saddr,u8 *txdata,int length)
{
	struct i2c_msg msg[] = {
		{
			.addr  = saddr,
				.flags = 0,
				.len = length,
				.buf = txdata,
		},
	};

	if (i2c_transfer(ov5640_client->adapter, msg, 1) < 0)    return -EIO;
	else return 0;
}

static int ov5640_i2c_write(unsigned short saddr, unsigned int waddr,
							unsigned short bdata,u8 trytimes,
							enum ov5640_width_t width)
{
	int rc = -EIO;
	ov5640_counter = 0;
      switch (width)
    {
        case WORD_LEN:
        {
           ov5640_i2c_buf[0] = (waddr & 0xFF00)>>8;
	     ov5640_i2c_buf[1] = (waddr & 0x00FF);
	     ov5640_i2c_buf[2] = (bdata & 0x00FF);

	     while ((ov5640_counter < trytimes) &&(rc != 0))
	     {
		    rc = ov5640_i2c_txdata(saddr, ov5640_i2c_buf, 3);

		    if (rc < 0)
		    {
			    ov5640_counter++;
			    pr_err("***--CAMERA-- i2c_write_w failed,i2c addr=0x%x, command addr = 0x%x, val = 0x%x, s=%d, rc=%d!\n",saddr,waddr, bdata,ov5640_counter,rc);
			    msleep(4);
		    }
	     };
         }
        break;

        case BYTE_LEN:
        {
             ov5640_i2c_buf[0] = (waddr & 0xFF00)>>8;
	       ov5640_i2c_buf[1] = (waddr & 0x00FF);
	       ov5640_i2c_buf[2] = bdata;

	       while ((ov5640_counter < trytimes) &&(rc != 0))
	       {
		    rc = ov5640_i2c_txdata(saddr, ov5640_i2c_buf, 3);

		    if (rc < 0)
		    {
			    ov5640_counter++;
			    pr_err("***--CAMERA-- i2c_write_w failed,i2c addr=0x%x, command addr = 0x%x, val = 0x%x, s=%d, rc=%d!\n",saddr,waddr, bdata,ov5640_counter,rc);
			    msleep(4);
		    }
	       };
        }
        break;

        default:
        {
        }
        break;
    }
	
	return rc;
}

static int ov5640_i2c_rxdata(unsigned short saddr,
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

	if (i2c_transfer(ov5640_client->adapter, msgs, 2) < 0) {
		pr_err("ov5640_i2c_rxdata failed!\n");
		return -EIO;
	}

	return 0;
}

static int32_t ov5640_i2c_read_byte_1(unsigned short raddr, unsigned short *rdata)
{
	int rc = 0;
	unsigned char buf[2];
	//pr_err("+ov5640_i2c_read_byte\n");
	memset(buf, 0, sizeof(buf));

	buf[0] = (raddr & 0xFF00)>>8;
	buf[1] = (raddr & 0x00FF);

	rc = ov5640_i2c_rxdata(ov5640_client->addr, buf, 1);
	if (rc < 0) {
		pr_err("ov5640_i2c_read_byte failed!\n");
		return rc;
	}

	*rdata = buf[0];

	//pr_err("-ov5640_i2c_read_byte\n");
	return rc;
}

static int32_t ov5640_i2c_read_byte(unsigned short   saddr,
									unsigned int raddr, unsigned int *rdata)
{
	int rc = 0;
	unsigned char buf[2];
	//pr_err("+ov5640_i2c_read_byte\n");
	memset(buf, 0, sizeof(buf));

	buf[0] = (raddr & 0xFF00)>>8;
	buf[1] = (raddr & 0x00FF);

	rc = ov5640_i2c_rxdata(saddr, buf, 1);
	if (rc < 0) {
		pr_err("ov5640_i2c_read_byte failed!\n");
		return rc;
	}

	*rdata = buf[0];

	//pr_err("-ov5640_i2c_read_byte\n");
	return rc;
}
#if 0
static int32_t ov5640_i2c_read(unsigned short   saddr,
							   unsigned int raddr, unsigned int *rdata)
{
	int rc = 0;
	unsigned char buf[2];
	//pr_err("+ov5640_i2c_read\n");
	memset(buf, 0, sizeof(buf));

	buf[0] = (raddr & 0xFF00)>>8;
	buf[1] = (raddr & 0x00FF);

	rc = ov5640_i2c_rxdata(saddr, buf, 2);
	if (rc < 0)	return rc;
	*rdata = buf[0] << 8 | buf[1];

	if (rc < 0)
		pr_err("ov5640_i2c_read failed!\n");
	//pr_err("-ov5640_i2c_read\n");
	return rc;
}
#endif

static int32_t ov5640_i2c_write_b_sensor(unsigned int waddr, unsigned short bdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[3];
	memset(buf, 0, sizeof(buf));
	buf[0] = (waddr & 0xFF00) >> 8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = bdata;
	//pr_err("i2c_write_b addr = 0x%x, val = 0x%x\n", waddr, bdata);
	rc = ov5640_i2c_txdata(ov5640_client->addr, buf, 3);
	if (rc < 0)
		pr_err("i2c_write_b failed, addr = 0x%x, val = 0x%x!\n",
			waddr, bdata);
	return rc;
}

static int32_t OV5640Core_WritePREG(struct OV5640_WREG const *pTb,int32_t len)
{
	int32_t i, ret = 0;
	uint32_t regv;
      
	for (i = 0;i < len; i++)
	{
		if (0 == pTb[i].mask) {  
			ov5640_i2c_write(ov5640_client->addr,pTb[i].addr,pTb[i].data,10,WORD_LEN);
		} else {   
			ov5640_i2c_read_byte(ov5640_client->addr,pTb[i].addr, &regv);
			regv &= pTb[i].mask;
			regv |= (pTb[i].data & (~pTb[i].mask));
			ov5640_i2c_write(ov5640_client->addr, pTb[i].addr, regv, 10 ,WORD_LEN);
		}
	}
	return ret;
}

static void camera_sw_power_onoff(int v)
{
   #if 0
	if (v == 0) {
		pr_err("camera_sw_power_onoff: down\n");
		ov5640_i2c_write(ov5640_client->addr, 0x3008, 0x42, 10,WORD_LEN); //power down
	}
	else {
		pr_err("camera_sw_power_onoff: on\n");
		ov5640_i2c_write(ov5640_client->addr, 0x3008, 0x02, 10,WORD_LEN); //power on
	}
    #endif
    	if (v == 0)
            {
		pr_err("camera_sw_power_onoff: down\n");
		ov5640_i2c_write(ov5640_client->addr, 0x4202, 0x0f, 10,WORD_LEN); //power down
		mdelay(50);
	      }
	else
        {
		pr_err("camera_sw_power_onoff: on\n");
		ov5640_i2c_write(ov5640_client->addr, 0x4202, 0x00, 10,WORD_LEN); //power on
	 }
}
static int ov5640_pwdn_on(void)
{
	pr_err("--CAMERA-- %s ... (Start...)\n",__func__);

	gpio_set_value(ov5640_pwdn_gpio, 1);
	//gpio_set_value(ov5640_reset_gpio, 0);

	//afinit = 0;

	pr_err("--CAMERA-- %s ... (End...)\n",__func__);
    return 0;
}

static int ov5640_pwdn_off(void)
{
	pr_err("--CAMERA-- %s ... (Start...)\n",__func__);

	gpio_set_value(ov5640_pwdn_gpio, 0);
	//gpio_set_value(ov5640_reset_gpio, 0);

	//afinit = 0;

	pr_err("--CAMERA-- %s ... (End...)\n",__func__);
      return 0;
}

#if 0
static int ov5640_power_off(void)
{
      //int rc = 0;
	pr_err("--CAMERA-- %s ... (Start...)\n",__func__);
      
    // neil add for power down mode

	gpio_set_value(ov5640_pwdn_gpio, 1);
	//gpio_set_value(ov5640_reset_gpio, 0);

	//afinit = 0;

	pr_err("--CAMERA-- %s ... (End...)\n",__func__);
    return 0;
}
#endif

#if 0
static int ov5640_power_on(void)
{
        //int rc = 0;
	pr_err("--CAMERA-- %s ... (Start...)\n",__func__);

	gpio_set_value(ov5640_pwdn_gpio, 0);


	//afinit = 1;

	pr_err("--CAMERA-- %s ... (End...)\n",__func__);
    return 0;
}
#endif

static void ov5640_power_reset(void)
{
	pr_err("--CAMERA-- %s ... (Start...)\n",__func__);
	gpio_set_value(ov5640_reset_gpio, 1);   //reset camera reset pin
	mdelay(5);
	gpio_set_value(ov5640_reset_gpio, 0);
	mdelay(5);
	gpio_set_value(ov5640_reset_gpio, 1);
	mdelay(1);

	pr_err("--CAMERA-- %s ... (End...)\n",__func__);
}

static int ov5640_probe_readID(const struct msm_camera_sensor_info *data)
{
	int rc = 0;    
	u32 device_id_high = 0;
	u32 device_id_low = 0;

	pr_err("--CAMERA-- %s (Start...)\n",__func__);
	pr_err("--CAMERA-- %s sensor poweron,begin to read ID!\n",__func__);

	//0x300A ,sensor ID register
	rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x300A, &device_id_high);

	if (rc < 0)
	{
		pr_err("--CAMERA-- %s ok , readI2C failed, rc = 0x%x\r\n", __func__, rc);
		return rc;   
	}  
	pr_err("--CAMERA-- %s  readID high byte, data = 0x%x\r\n", __func__, device_id_high);

	//0x300B ,sensor ID register
	rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x300B, &device_id_low);
	if (rc < 0)
	{
		pr_err("--CAMERA-- %s ok , readI2C failed,rc = 0x%x\r\n", __func__, rc);
		return rc;   
	}   

	pr_err("--CAMERA-- %s  readID low byte, data = 0x%x\r\n", __func__, device_id_low);
	pr_err("--CAMERA-- %s return ID :0x%x\n", __func__, (device_id_high<<8)+device_id_low);

#ifdef CONFIG_SENSOR_INFO
	msm_sensorinfo_set_sensor_id((device_id_high<<8)+device_id_low);
#endif

	//0x5640, ov5640 chip id
	if((device_id_high<<8)+device_id_low != OV5640_SENSOR_ID)
	{
		pr_err("--CAMERA-- %s ok , device id error, should be 0x%x\r\n",
			__func__, OV5640_SENSOR_ID);
		return -EINVAL;
	}
	else
	{
		pr_err("--CAMERA-- %s ok , device id=0x%x\n",__func__,OV5640_SENSOR_ID);
		return 0;
	}
}

#if 0
static void dump_af_status(void)
{
	int tmp = 0;

	ov5640_i2c_read_byte(ov5640_client->addr, 0x3000, &tmp);
	pr_err("	%s 0x3000 = %x\n", __func__, tmp);
	ov5640_i2c_read_byte(ov5640_client->addr, 0x3001, &tmp);
	pr_err("	%s 0x3001 = %x\n", __func__, tmp);
	ov5640_i2c_read_byte(ov5640_client->addr, 0x3004, &tmp);
	pr_err("	%s 0x3004 = %x\n", __func__, tmp);
	ov5640_i2c_read_byte(ov5640_client->addr, 0x3005, &tmp);
	pr_err("	%s 0x3005 = %x\n", __func__, tmp);
	ov5640_i2c_read_byte(ov5640_client->addr, OV5640_CMD_FW_STATUS, &tmp);
	pr_err("	%s af_st = %x\n\n", __func__, tmp);
}
#endif

static int ov5640_af_setting(void)
{
	int rc = 0;

	pr_err("--CAMERA-- ov5640_af_setting\n");
	rc = OV5640Core_WritePREG(ov5640_regs.afinit_tbl,ov5640_regs.afinit_tbl_size);   

	if (rc < 0)
	{
		pr_err("--CAMERA-- AF_init failed\n");
		return rc;
	}

	return rc;
}

static int ov5640_set_flash_light(enum led_brightness brightness)
{
	struct led_classdev *led_cdev;

	pr_err("ov5640_set_flash_light brightness = %d\n", brightness);

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



#if 1
//===============AE start=================
int XVCLK = 2400;	// real clock/10000
int preview_sysclk, preview_HTS, preview_VTS;
int AE_Target = 52;
int AE_high, AE_low;

static int ov5640_get_sysclk(void)
{
//	int8_t buf[2];
	unsigned short  buf[2];
	 // calculate sysclk
	 int Multiplier, PreDiv, VCO, SysDiv, Pll_rdiv, Bit_div2x = 1, sclk_rdiv, sysclk;

	 int sclk_rdiv_map[] = {
		 1, 2, 4, 8};

	ov5640_i2c_read_byte_1(0x3034, buf);
	buf[1] = buf[0] & 0x0f;
	 if (buf[1] == 8 || buf[1] == 10) {
		 Bit_div2x = buf[1] / 2;
	 }

	ov5640_i2c_read_byte_1(0x3035, buf);
	 SysDiv = buf[0] >>4;
	 if(SysDiv == 0) {
		 SysDiv = 16;
	 }

	ov5640_i2c_read_byte_1(0x3036, buf);
	 Multiplier = buf[0];

	ov5640_i2c_read_byte_1(0x3037, buf);
	 PreDiv = buf[0] & 0x0f;
	 Pll_rdiv = ((buf[0] >> 4) & 0x01) + 1;

	ov5640_i2c_read_byte_1(0x3108, buf);
	 buf[1] = buf[0] & 0x03;
	 sclk_rdiv = sclk_rdiv_map[buf[1]]; 

	 VCO = XVCLK * Multiplier / PreDiv;

	 sysclk = VCO / SysDiv / Pll_rdiv * 2 / Bit_div2x / sclk_rdiv;

	 return sysclk;
}

static int ov5640_get_HTS(void)
{
	//int8_t buf[2];
	unsigned short buf[2];
	 // read HTS from register settings
	 int HTS;

	ov5640_i2c_read_byte_1(0x380c, buf);
	ov5640_i2c_read_byte_1(0x380d, &buf[1]);
	 HTS=buf[0];
	 HTS = (HTS<<8) + buf[1];

	 return HTS;
}

static int ov5640_get_VTS(void)
{
	//int8_t buf[2];
	unsigned short  buf[2];
	 // read VTS from register settings
	 int VTS;

	ov5640_i2c_read_byte_1(0x380e, buf);
	printk("%s: 0x380e=0x%x\n", __func__, buf[0]);
	ov5640_i2c_read_byte_1(0x380f, &buf[1]);
	printk("%s: 0x380e=0x%x\n", __func__, buf[1]);
	VTS = buf[0];
	VTS = VTS<<8;
	VTS += (unsigned char)buf[1];
	printk("%s: VTS=0x%x\n", __func__, VTS);

	 return VTS;
}

static int ov5640_set_VTS(int VTS)
{
	unsigned short buf[2];
	 // write VTS to registers
	 

	 //temp = VTS & 0xff;
	 buf[0] = VTS & 0xff;
	 printk("%s: VTS & oxff = 0x%x\n", __func__, buf[0]);
	 ov5640_i2c_write_b_sensor(0x380f, buf[0]);

	 buf[0] = VTS>>8;
	 printk("%s: VTS>>8 = 0x%x\n", __func__, buf[0]);
	 ov5640_i2c_write_b_sensor(0x380e, buf[0]);

	 return 0;
}

/*
static int ov5640_get_shutter(void)
{
	 // read shutter, in number of line period
	 int shutter;
	 //int8_t buf[2];
	 unsigned short  buf[2];
	 int temp;

	ov5640_i2c_read_byte_1(0x3500, buf);
	 shutter = (buf[0] & 0x0f);
	 printk("%s: 0x3500=0x%x\n", __func__, buf[0]);
	 ov5640_i2c_read_byte_1(0x3501, buf);
	 shutter = (shutter<<8) + buf[0];
	 printk("%s: 0x3501=0x%x\n", __func__, buf[0]);
	 ov5640_i2c_read_byte_1(0x3502, buf);
	 temp = buf[0];
	 shutter = (shutter<<4) + (temp>>4);
	 printk("%s: 0x3502=0x%x\n", __func__, buf[0]);

	 return shutter;
}
*/
static int ov5640_set_shutter(int shutter)
{
	 // write shutter, in number of line period
	unsigned short buf[2];
	 int temp;
	 
	 shutter = shutter & 0xffff;

	 temp = shutter & 0x0f;
	 buf[0] = temp<<4;
	 printk("%s: shutter&0x0f <<4 0x%x\n", __func__, buf[0]);
	 ov5640_i2c_write_b_sensor(0x3502, buf[0]);

	 temp = shutter & 0xfff;
	 buf[0] = temp>>4;
	 printk("%s: shutter&0xfff >>4 0x%x\n", __func__, buf[0]);
	 ov5640_i2c_write_b_sensor(0x3501,buf[0]);

	 temp = shutter>>12;
	 buf[0] = temp;
	 printk("%s: shutter>>12 0x%x\n", __func__, buf[0]);
	 ov5640_i2c_write_b_sensor(0x3500, buf[0]);

	 return 0;
}

/*
static int ov5640_get_gain16(void)
{
	 // read gain, 16 = 1x
	 int gain16;
	 //int8_t buf[2];
	 unsigned short buf[2];

	ov5640_i2c_read_byte_1(0x350a, buf);
	 //buf[0] = buf[0] & 0x03;
	 gain16 = buf[0] & 0x03;
	 printk("%s: 0x350a=0x%x\n", __func__, buf[0]);
	 //ov5640_i2c_read_byte_1(0x350b, &buf[1], 1, WORD_LEN);
	 ov5640_i2c_read_byte_1(0x350b, buf);
	 printk("%s: 0x350b=0x%x\n", __func__, (int8_t)buf[0]);
	 printk("%s: 0x350b=%d\n", __func__, buf[0]);

	 gain16 = (gain16<<8) + buf[0];
	 printk("%s: gain16=0x%x\n", __func__, gain16);

	 return gain16;
}
*/
static int ov5640_set_gain16(int gain16)
{
	 // write gain, 16 = 1x
	 int16_t buf[2];
	 //unsigned short buf[2];
	 
	 gain16 = gain16 & 0x3ff;

	 buf[0] = gain16 & 0xff;
	 ov5640_i2c_write_b_sensor(0x350b, buf[0]);

	 buf[0] = gain16>>8;
	 ov5640_i2c_write_b_sensor(0x350a, buf[0]);

	 return 0;
}

static int ov5640_get_light_frequency(void)
{
	 // get banding filter value
	 int light_frequency;
	 unsigned short buf[2];

	ov5640_i2c_read_byte_1(0x3c01, buf);

	 if (buf[0] & 0x80) {
		 // manual
		 ov5640_i2c_read_byte_1(0x3c00, &buf[1]);
		 if (buf[1] & 0x04) {
			 // 50Hz
			 light_frequency = 50;
		 }
		 else {
			 // 60Hz
			 light_frequency = 60;
		 }
	 }
	 else {
		 // auto
		 ov5640_i2c_read_byte_1(0x3c0c, &buf[1]);
		 if (buf[1] & 0x01) {
			 // 50Hz
			 light_frequency = 50;
		 }
		 else {
			 // 60Hz
		 }
	 }
	 return light_frequency;
}


static void ov5640_set_bandingfilter(void)
{
	int16_t buf[2];
	//unsigned short buf[2];
	 int preview_VTS;
	 int band_step60, max_band60, band_step50, max_band50;

	 // read preview PCLK
	 preview_sysclk = ov5640_get_sysclk();

	 // read preview HTS
	 preview_HTS = ov5640_get_HTS();

	 // read preview VTS
	 preview_VTS = ov5640_get_VTS();

	 // calculate banding filter
	 // 60Hz
	 band_step60 = preview_sysclk * 100/preview_HTS * 100/120;
	 buf[0] = band_step60 >> 8;
	 ov5640_i2c_write_b_sensor(0x3a0a, buf[0]);
	 buf[0] = band_step60 & 0xff;
	 ov5640_i2c_write_b_sensor(0x3a0b, buf[0]);

	 max_band60 = (int)((preview_VTS-4)/band_step60);
	 buf[0] = (int8_t)max_band60;
	 ov5640_i2c_write_b_sensor(0x3a0d, buf[0]);

	 // 50Hz
	 band_step50 = preview_sysclk * 100/preview_HTS; 
	 buf[0] = (int8_t)(band_step50 >> 8);
	 ov5640_i2c_write_b_sensor(0x3a08, buf[0]);
	buf[0] = (int8_t)(band_step50 & 0xff);
	 ov5640_i2c_write_b_sensor(0x3a09, buf[0]);

	 max_band50 = (int)((preview_VTS-4)/band_step50);
	 buf[0] = (int8_t)max_band50;
	 ov5640_i2c_write_b_sensor(0x3a0e,buf[0]);
}

/*
static int ov5640_set_AE_target(int target)
{
	int16_t buf[2];
	//unsigned short buf[2];
	 // stable in high
	 int fast_high, fast_low;
	 AE_low = target * 23 / 25;	// 0.92
	 AE_high = target * 27 / 25;	// 1.08

	 fast_high = AE_high<<1;
	 if(fast_high>255)
		 fast_high = 255;

	 fast_low = AE_low>>1;

	buf[0] = (int8_t)AE_high;
	ov5640_i2c_write_b_sensor(0x3a0f, buf[0]);
	buf[0] = (int8_t)AE_low;
	 ov5640_i2c_write_b_sensor(0x3a10, buf[0]);
	 buf[0] = (int8_t)AE_high;
	 ov5640_i2c_write_b_sensor(0x3a1b, buf[0]);
	  buf[0] = (int8_t)AE_low;
	 ov5640_i2c_write_b_sensor(0x3a1e, buf[0]);
	 buf[0] = (int8_t)fast_high;
	 ov5640_i2c_write_b_sensor(0x3a11, buf[0]);
	 buf[0] = (int8_t)fast_low;
	 ov5640_i2c_write_b_sensor(0x3a1f, buf[0]);

	 return 0;
}

*/
#endif



static int ov5640_get_preview_exposure_gain(void)
{
    int rc = 0;
    uint16_t UV, temp;
    uint16_t ret_l,ret_m,ret_h, LencQ_H, LencQ_L;
            
    ov5640_i2c_write_b_sensor(0x3503, 0x07);
    ov5640_i2c_write_b_sensor(0x5196, 0x23); //freeze awb kenxu update @20120516

	ov5640_i2c_read_byte_1(0x3c01, &temp);
	pr_err("0x3c01=0x%x\n", temp);
	ov5640_i2c_read_byte_1(0x3c00, &temp);
	pr_err("0x3c00=0x%x\n", temp);
	ov5640_i2c_read_byte_1(0x3a08, &temp);
	pr_err("0x3a08=0x%x\n", temp);
	ov5640_i2c_read_byte_1(0x3a09, &temp);
	pr_err("0x3a09=0x%x\n", temp);

	ov5640_i2c_read_byte_1(0x3a0a, &temp);
	pr_err("0x3a0a=0x%x\n", temp);
	ov5640_i2c_read_byte_1(0x3a0b, &temp);
	pr_err("0x3a0b=0x%x\n", temp);
		ov5640_i2c_read_byte_1(0x3a0d, &temp);
	pr_err("0x3a0d=0x%x\n", temp);
		ov5640_i2c_read_byte_1(0x3a0e, &temp);
	pr_err("0x3a0e=0x%x\n", temp);


	ov5640_i2c_read_byte_1(0x3034, &temp);
	pr_err("0x3034=0x%x\n", temp);
	ov5640_i2c_read_byte_1(0x3035, &temp);
	pr_err("0x3035=0x%x\n", temp);
		ov5640_i2c_read_byte_1(0x3036, &temp);
	pr_err("0x3036=0x%xn", temp);
		ov5640_i2c_read_byte_1(0x3037, &temp);
	pr_err("0x3037=0x%x\n", temp);


		ov5640_i2c_read_byte_1(0x3108, &temp);
	pr_err("0x3108=0x%x\n", temp);
	ov5640_i2c_read_byte_1(0x3824, &temp);
	pr_err("0x3824=0x%x\n", temp);
		ov5640_i2c_read_byte_1(0x460c, &temp);
	pr_err("0x460c=0x%x\n", temp);
		ov5640_i2c_read_byte_1(0x300e, &temp);
	pr_err("0x300e=0x%x\n", temp);


		ov5640_i2c_read_byte_1(0x380c, &temp);
	pr_err("0x380c=0x%x\n", temp);
	ov5640_i2c_read_byte_1(0x380d, &temp);
	pr_err("0x380d=0x%x\n", temp);
		ov5640_i2c_read_byte_1(0x380e, &temp);
	pr_err("0x380e=0x%x\n", temp);
		ov5640_i2c_read_byte_1(0x380f, &temp);
	pr_err("0x380f=0x%x\n", temp);


			ov5640_i2c_read_byte_1(0x5588, &temp);
	pr_err("0x5588=0x%x\n", temp);
		ov5640_i2c_read_byte_1(0x5583, &temp);
	pr_err("0x5583=0x%x\n", temp);
			ov5640_i2c_read_byte_1(0x5584, &temp);
	pr_err("0x5584=0x%x\n", temp);
	

    #if 1
	//keep saturation same for preview and capture
	ov5640_i2c_read_byte_1(0x558c, &UV);
	ov5640_i2c_read_byte_1(0x5588, &temp);
	temp = temp | 0x40;
	ov5640_i2c_write_b_sensor(0x5588, temp); //Manual UV
	ov5640_i2c_write_b_sensor(0x5583, UV);
	ov5640_i2c_write_b_sensor(0x5584, UV);
	printk("preview_UV=%d\n", UV);
    #endif

	//keep Lenc same for preview and capture
	ov5640_i2c_read_byte_1(0x350a, &LencQ_H);
	ov5640_i2c_read_byte_1(0x350b, &LencQ_L);
	ov5640_i2c_write_b_sensor(0x5054, LencQ_H);
	ov5640_i2c_write_b_sensor(0x5055, LencQ_L);	
	ov5640_i2c_write_b_sensor(0x504d, 0x02);	//Manual mini Q	

    //get preview exp & gain
    ret_h = ret_m = ret_l = 0;
    ov5640_preview_exposure = 0;
    ov5640_i2c_read_byte_1(0x3500, &ret_h);
    ov5640_i2c_read_byte_1(0x3501, &ret_m);
    ov5640_i2c_read_byte_1(0x3502, &ret_l);
    ov5640_preview_exposure = (ret_h << 12) + (ret_m << 4) + (ret_l >> 4);
//    printk("preview_exposure=%d\n", ov5640_preview_exposure);

    ret_h = ret_m = ret_l = 0;
    ov5640_preview_maxlines = 0;
    ov5640_i2c_read_byte_1(0x380e, &ret_h);
    ov5640_i2c_read_byte_1(0x380f, &ret_l);
    ov5640_preview_maxlines = (ret_h << 8) + ret_l;
//    printk("Preview_Maxlines=%d\n", ov5640_preview_maxlines);

    //Read back AGC Gain for preview
    ov5640_gain = 0;
    ov5640_i2c_read_byte_1(0x350b, &ov5640_gain);
    printk("Gain,0x350b=0x%x\n", ov5640_gain);

//ZTE_YGL_CAM_20111221,modifed for SNR
    YAVG = 0;
    ov5640_i2c_read_byte_1(0x56A1, &YAVG);
    WB_T = 0;
    ov5640_i2c_read_byte_1(0x51d0, &WB_T);

 // read preview PCLK	 
 preview_sysclk = ov5640_get_sysclk();	 
 // read preview HTS	 
 preview_HTS = ov5640_get_HTS();
	
  #ifdef ZTE_FIX_WB_ENABLE
	zte_flash_on_fix_wb();
	#endif
	
    return rc;
}

#if 0 //old
{
    int rc = 0;
    unsigned int ret_l,ret_m,ret_h;
            
    ov5640_i2c_write(ov5640_client->addr,0x3503, 0x07,10,WORD_LEN);

    //get preview exp & gain
    ret_h = ret_m = ret_l = 0;
    ov5640_preview_exposure = 0;
    ov5640_i2c_read_byte(ov5640_client->addr,0x3500, &ret_h);
    ov5640_i2c_read_byte(ov5640_client->addr,0x3501, &ret_m);
    ov5640_i2c_read_byte(ov5640_client->addr,0x3502, &ret_l);
    ov5640_preview_exposure = (ret_h << 12) + (ret_m << 4) + (ret_l >> 4);
//    printk("preview_exposure=%d\n", ov5640_preview_exposure);

    ret_h = ret_m = ret_l = 0;
    ov5640_preview_maxlines = 0;
    ov5640_i2c_read_byte(ov5640_client->addr,0x380e, &ret_h);
    ov5640_i2c_read_byte(ov5640_client->addr,0x380f, &ret_l);
    ov5640_preview_maxlines = (ret_h << 8) + ret_l;
//    printk("Preview_Maxlines=%d\n", ov5640_preview_maxlines);

    //Read back AGC Gain for preview
    ov5640_gain = 0;
    ov5640_i2c_read_byte(ov5640_client->addr,0x350b, &ov5640_gain);
//    printk("Gain,0x350b=0x%x\n", ov5640_gain);

//ZTE_YGL_CAM_20111221,modifed for SNR
    YAVG = 0;
    ov5640_i2c_read_byte(ov5640_client->addr,0x56A1, &YAVG);
    WB_T = 0;
    ov5640_i2c_read_byte(ov5640_client->addr,0x51d0, &WB_T);
    return rc;
}
#endif

static int ov5640_set_capture_exposure_gain(void)
{
    int rc = 0;
   //calculate capture exp & gain
   
	 int preview_shutter, preview_gain16;
	 int capture_gain16;
	 int capture_sysclk, capture_HTS, capture_VTS;
	 int light_frequency, capture_bandingfilter, capture_max_band;
	 long int capture_gain16_shutter,capture_shutter;
	 unsigned short average;

	 // read preview shutter	 
	 preview_shutter = ov5640_preview_exposure;	 
	 // read preview gain
        preview_gain16 = ov5640_gain;
	 printk("%s: preview_shutter=0x%x, preview_gain16=0x%x\n", __func__, preview_shutter, preview_gain16);

	// read capture VTS
	 capture_VTS = ov5640_get_VTS();
	 capture_HTS = ov5640_get_HTS();
	 capture_sysclk = ov5640_get_sysclk();
	 printk("%s: capture_VTS=0x%x, capture_HTS=0x%x, capture_sysclk=0x%x\n", __func__, capture_VTS, capture_HTS, capture_sysclk);

	 // get average	  
	 ov5640_i2c_read_byte_1(0x56a1,&average);	 

	 // calculate capture banding filter
	 light_frequency = ov5640_get_light_frequency();
	 if (light_frequency == 60) {
		 // 60Hz
		 capture_bandingfilter = capture_sysclk * 100 / capture_HTS * 100 / 120;
	 }
	 else {
		 // 50Hz
		 capture_bandingfilter = capture_sysclk * 100 / capture_HTS;
	 }
	 capture_max_band = (int)((capture_VTS - 4)/capture_bandingfilter);
	 printk("%s: light_frequency=0x%x, capture_bandingfilter=0x%x, capture_max_band=0x%x\n", __func__, light_frequency, capture_bandingfilter, capture_max_band);
      # if 0
	 // calculate capture shutter/gain16
	 if (average > AE_low && average < AE_high) {
		 // in stable range
		 capture_shutter = preview_shutter * capture_sysclk/preview_sysclk * preview_HTS/capture_HTS * AE_Target / average;
		//capture_gain16_shutter = preview_gain16 * preview_shutter /preview_sysclk * capture_sysclk /capture_HTS * preview_HTS * AE_Target / average;
	 }
	 else {
		 capture_shutter = preview_shutter * capture_sysclk/preview_sysclk * preview_HTS/capture_HTS;
		// capture_gain16_shutter = preview_gain16 * preview_shutter /preview_sysclk * capture_sysclk/capture_HTS* preview_HTS;
	 }
      #endif
	  	 capture_shutter = preview_shutter * capture_sysclk/preview_sysclk * preview_HTS/capture_HTS;
	        capture_gain16_shutter = preview_gain16 * capture_shutter;
	  
	 printk("%s:  preview_gain16=%d, preview_shutter=%d   capture_gain16_shutter=%ld\n", __func__, preview_gain16, preview_shutter,capture_gain16_shutter);
	 printk("%s: capture_sysclk=%d, preview_sysclk=%d, preview_HTS=%d\n", __func__, capture_sysclk, preview_sysclk, preview_HTS);

	 // gain to shutter
	 if(capture_gain16_shutter < (capture_bandingfilter * 16)) {
		 // shutter < 1/100
		 capture_shutter = capture_gain16_shutter/16;
		 if(capture_shutter <1)
			 capture_shutter = 1;

		 capture_gain16 = capture_gain16_shutter/capture_shutter;
		 if(capture_gain16 < 16)
			 capture_gain16 = 16;
	 printk("shutter < 1/100\n");
		 
	 }
	 else {
		 if(capture_gain16_shutter > (capture_bandingfilter*capture_max_band*16)) {
			 // exposure reach max
			 capture_shutter = capture_bandingfilter*capture_max_band;
			 capture_gain16 = capture_gain16_shutter / capture_shutter;
			 	 printk(" exposure reach max\n");
		 }
		 else {
			 // 1/100 < capture_shutter =< max, capture_shutter = n/100
			 capture_shutter = ((int)(capture_gain16_shutter/16/capture_bandingfilter)) * capture_bandingfilter;
			 capture_gain16 = capture_gain16_shutter / capture_shutter;
			  printk("1/100 < capture_shutter =< max, capture_shutter = n/100\n");
		 }
	 }

#if 0 // \D6\F7\B9۲\E2\CA԰汾 2011-12-21 ken
        //kenxu add for reduce noise under dark condition
        if(iCapture_Gain > 32) //gain > 2x, gain / 2, exposure * 2;
        {
          iCapture_Gain = iCapture_Gain / 2;
          ulCapture_Exposure = ulCapture_Exposure * 2;
        }
#endif
    printk("WB_T=%d\n", WB_T);
#if 1
    if(WB_T == 0X02)
  	{
  	  rc = OV5640Core_WritePREG(ov5640_regs.lens_shading_D65_tbl,ov5640_regs.lens_shading_D65_tbl_size);
  	}
    else if (WB_T == 0X06)
  	{
  	 rc = OV5640Core_WritePREG(ov5640_regs.lens_shading_A_tbl,ov5640_regs.lens_shading_A_tbl_size);
  	}
#endif
    //if (flash_led_enable == 0)
    	//{
	    if(capture_gain16 > 54) //reach to max gain 16x, change blc to pass SNR test under low light
         {  
	      	    if(YAVG <=15) //10lux
	      	 {
	      	  capture_shutter = capture_shutter * 2;
	          rc = ov5640_i2c_write_b_sensor(0x4009, 0x50);
	          if (rc < 0)
	          {
	            return rc;
	          }
	      	}
	      	else if(YAVG <=23)//50lux
	        {
	          capture_shutter = capture_shutter * 4/3;
	          rc = ov5640_i2c_write_b_sensor(0x4009, 0x20);
	          if (rc < 0)
	          {
	            return rc;
	          }
	        }            
     }
      	    printk("capture_shutter=%ld\n", capture_shutter);
    	//}

	 // write capture gain
	 ov5640_set_gain16(capture_gain16);

	 // write capture shutter
	 if (capture_shutter > (capture_VTS - 4)) {
		 capture_VTS = capture_shutter + 4;
		 ov5640_set_VTS(capture_VTS);
	 }
	 ov5640_set_shutter(capture_shutter);


    msleep(150);

    return rc;
}

static int ov5640_video_config(void)
{
	int rc = 0;
    int i;
    unsigned short temp;
    unsigned int  af_ack;
	pr_err("--CAMERA-- ov5640_video_config\n");

	pr_err("--CAMERA-- preview in, is_autoflash - 0x%x\n", is_autoflash);

	/* autoflash setting */
	if (is_autoflash == 1) {
		ov5640_set_flash_light(LED_OFF);
	}

	/* preview setting */
	rc = OV5640Core_WritePREG(ov5640_regs.preview_tbl,ov5640_regs.preview_tbl_size);   

	ov5640_i2c_read_byte_1(0x5588, &temp);
	temp = temp & 0xbf;
	ov5640_i2c_write_b_sensor(0x5588, temp); //Auto UV
		
	ov5640_set_bandingfilter();
	

	rc = ov5640_i2c_write(ov5640_client->addr, 0x3000, 0x0020, 1,WORD_LEN);
	if (rc < 0)
	{
		pr_err("%s: failed, rc=%d!\n", __func__, rc);
		return rc;
	}
	mdelay(10);  

	rc = ov5640_i2c_write(ov5640_client->addr, 0x3000,0x0000, 1,WORD_LEN);
	if (rc < 0)
	{
		pr_err("%s: failed, rc=%d!\n", __func__, rc);
		return rc;
	}

	// release focus status		    
	 rc = ov5640_i2c_write(ov5640_client->addr, 0x3023, 0x0001, 1,WORD_LEN);
	 if (rc < 0)
	{
		pr_err("%s: failed, rc=%d!\n", __func__, rc);
		return rc;
	}
				
	rc = ov5640_i2c_write(ov5640_client->addr, 0x3022, 0x0008,1, WORD_LEN);
	if (rc < 0)
	{
		pr_err("%s: failed, rc=%d!\n",__func__, rc);        
		return rc;
	}
	af_ack = 0x0002;//set af_ack value for compare its value from get 
	 for ( i = 0; (i < 200) && (0x0000 != af_ack); ++i)
	{
		af_ack = 0x0002;
		rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x3023, &af_ack);
		if (rc < 0)
		{
			return rc;
		}
			mdelay(15);  
	}
	 rc = ov5640_i2c_write(ov5640_client->addr, 0x3023, 0x0001, 1,WORD_LEN);
	 if (rc < 0)
	{
		return rc;
	}
	rc = ov5640_i2c_write(ov5640_client->addr, 0x3022, 0x0012,1, WORD_LEN);
	if (rc < 0)
	{
		return rc;
	} 
	af_ack = 0x0002;//set af_ack value for compare its value from get 
	 for ( i = 0; (i < 200) && (0x0000 != af_ack); ++i)
	{
		af_ack = 0x0002;
		rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x3023, &af_ack);
				
		if (rc < 0)
		{
			return rc;
		}
			mdelay(15);  
	}
	return rc;
}

static int ov5640_snapshot_config(void)
{
	int rc = 0;
	unsigned int tmp;
	//int capture_len = sizeof(ov5640_capture_tbl) / sizeof(ov5640_capture_tbl[0]);

	pr_err("--CAMERA-- SENSOR_SNAPSHOT_MODE\n");
	ov5640_get_preview_exposure_gain();//Step1: get preview exposure and gain

	pr_err("--CAMERA-- %s, snapshot in, is_autoflash - 0x%x\n", __func__, is_autoflash);

	if (is_autoflash == 1) {
		ov5640_i2c_read_byte(ov5640_client->addr, 0x350b, &tmp);
		pr_err("--CAMERA-- GAIN VALUE : %x\n", tmp);
		if ((tmp & 0x80) == 0) {
			ov5640_set_flash_light(LED_OFF);
		} else {
			ov5640_set_flash_light(LED_FULL);
		}
	}

	rc = OV5640Core_WritePREG(ov5640_regs.capture_tbl,ov5640_regs.capture_tbl_size);  //Step2: change to full size 
	ov5640_set_capture_exposure_gain();//Step3: calculate and set capture exposure and gain

	return rc;
}

static int ov5640_setting(enum msm_s_reg_update rupdate,enum msm_s_setting rt)
{
	int rc = -EINVAL;
	unsigned int tmp;
	struct msm_camera_csi_params ov5640_csi_params;

	pr_err("--CAMERA-- %s (Start...), rupdate=%d \n",__func__,rupdate);

	switch (rupdate)
	{
	case S_UPDATE_PERIODIC:
		camera_sw_power_onoff(0); //standby

		msleep(20);

		ov5640_csi_params.lane_cnt = 2;
		ov5640_csi_params.data_format = CSI_8BIT;
		ov5640_csi_params.lane_assign = 0xe4;
		ov5640_csi_params.dpcm_scheme = 0;
		ov5640_csi_params.settle_cnt = 0x6;

		pr_err("%s: msm_camio_csi_config\n", __func__);

		rc = msm_camio_csi_config(&ov5640_csi_params);				
		OV5640_CSI_CONFIG = 1;

		msleep(20);

		if (S_RES_PREVIEW == rt) {
			rc = ov5640_video_config();
		} else if (S_RES_CAPTURE == rt) {
			rc = ov5640_snapshot_config();
		}
		camera_sw_power_onoff(1); //on

		msleep(20);

		break; /* UPDATE_PERIODIC */

	case S_REG_INIT:
		pr_err("--CAMERA-- S_REG_INIT (Start)\n");

		rc = ov5640_i2c_write(ov5640_client->addr, 0x3103, 0x11, 10,WORD_LEN);
		rc = ov5640_i2c_write(ov5640_client->addr, 0x3008, 0x82, 10,WORD_LEN);
		msleep(5);

		//set sensor init setting
		pr_err("set sensor init setting\n");
		rc = OV5640Core_WritePREG(ov5640_regs.init_tbl,ov5640_regs.init_tbl_size);
		if (rc < 0) {
			pr_err("sensor init setting failed\n");
			break;
		}

		//set image quality setting
		pr_err("%s: set image quality setting\n", __func__);
		rc = OV5640Core_WritePREG(ov5640_regs.init_iq_tbl,ov5640_regs.init_iq_tbl_size);

		rc =ov5640_i2c_read_byte(ov5640_client->addr, 0x4740, &tmp);			   
		pr_err("--CAMERA-- init 0x4740 value=0x%x\n", tmp);

		if (tmp != 0x21) {
			rc = ov5640_i2c_write(ov5640_client->addr, 0x4740, 0x21, 10,WORD_LEN);
			msleep(10);
			rc =ov5640_i2c_read_byte(ov5640_client->addr, 0x4740, &tmp);			   
			pr_err("--CAMERA-- WG 0x4740 value=0x%x\n", tmp);
		}

		pr_err("--CAMERA-- AF_init: afinit = %d\n", afinit);
		if (afinit == 1) {
			rc = ov5640_af_setting();
			if (rc < 0) {
				pr_err("--CAMERA-- ov5640_af_setting failed\n");
				break;
			}
			afinit = 0;
		}

		/* reset fps_divider */
		ov5640_ctrl->fps_divider = 1 * 0x0400;
		pr_err("--CAMERA-- S_REG_INIT (End)\n");
		break; /* case REG_INIT: */

	default:
		break;
	} /* switch (rupdate) */

	pr_err("--CAMERA-- %s (End), rupdate=%d \n",__func__,rupdate);

	return rc;
}

static int ov5640_sensor_open_init(const struct msm_camera_sensor_info *data)
{
	int rc = -ENOMEM;
	pr_err("--CAMERA-- %s\n",__func__);

	ov5640_ctrl = kzalloc(sizeof(struct __ov5640_ctrl), GFP_KERNEL);
	if (!ov5640_ctrl)
	{
		pr_err("--CAMERA-- kzalloc ov5640_ctrl error !!\n");
		kfree(ov5640_ctrl);
		return rc;
	}

#if 0
	ov5640_ctrl->fps_divider = 1 * 0x00000400;
	ov5640_ctrl->pict_fps_divider = 1 * 0x00000400;
	ov5640_ctrl->set_test = S_TEST_OFF;
	ov5640_ctrl->prev_res = S_QTR_SIZE;
	ov5640_ctrl->pict_res = S_FULL_SIZE;

	if (data)
		ov5640_ctrl->sensordata = data;
#endif
#if 0
	ov5640_power_off();
#endif

#if defined(CONFIG_MACH_ATLAS40) ||defined(CONFIG_MACH_BLADE2)	
	rc = msm_camera_clk_switch(data, OV5640_GPIO_SWITCH_CTL, OV5640_GPIO_SWITCH_VAL);
	if (rc < 0) {
		pr_err("msm_camera_clk_switch fail\n");
		return rc;
	}
#endif	

	pr_err("%s: msm_camio_clk_rate_set\n", __func__);

	msm_camio_clk_rate_set(24000000);
	mdelay(5);

	ov5640_pwdn_off();
//ov5640_power_reset();
      mdelay(100);
	pr_err("%s: init sequence\n", __func__);

#if 0
	if (ov5640_ctrl->prev_res == S_QTR_SIZE)
		rc = ov5640_setting(S_REG_INIT, S_RES_PREVIEW);
	else
		rc = ov5640_setting(S_REG_INIT, S_RES_CAPTURE);

	if (rc < 0)
	{
		pr_err("--CAMERA-- %s : ov5640_setting failed. rc = %d\n",__func__,rc);
		kfree(ov5640_ctrl);
		return rc;
	}
#endif
        rc = ov5640_i2c_write(ov5640_client->addr, 0x300e, 0x0045, 1, BYTE_LEN);
        if (rc < 0)
        {
           return rc;
        }
   	mdelay(100);
	pr_err("--CAMERA--re_init_sensor ok!!\n");
	return rc;
}

static int ov5640_sensor_release(void)
{
       int rc = 0;
	pr_err("--CAMERA--ov5640_sensor_release!!\n");

	mutex_lock(&ov5640_mutex);


     // neil add for power down mode
        rc = ov5640_i2c_write(ov5640_client->addr, 0x300e, 0x005d, 1, BYTE_LEN);
        if (rc < 0)
        {
           return rc;
        }
        
	//ov5640_power_off();
	ov5640_pwdn_on();

       msleep(200);
        
	kfree(ov5640_ctrl);
	ov5640_ctrl = NULL;

	mutex_unlock(&ov5640_mutex);
	return 0;
}

static const struct i2c_device_id ov5640_i2c_id[] = {
	{"ov5640", 0},{}
};

static int ov5640_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static int ov5640_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&ov5640_wait_queue);
	return 0;
}

static long ov5640_set_effect(int mode, int effect)
{
	int rc = 0;
      unsigned int  tmp_reg = 0;
	pr_err("--CAMERA-- %s ...(Start)\n",__func__);
#if 1
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

            rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5001, &tmp_reg);
            if (rc < 0)
            {
               return rc;
            }

            /*
             * ZTE_LJ_CAM_20101026
             * fix bug of no preview image after changing effect mode repeatedly
             */
            tmp_reg &= 0x00FF;
            tmp_reg |= 0x0080;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5001, tmp_reg, 1,BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
	switch (effect) 
	{
		case CAMERA_EFFECT_OFF: 
		{
		    pr_err("--CAMERA-- %s ...CAMERA_EFFECT_OFF\n",__func__);
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5583, 0x0040, 1,WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5584, 0x001e, 1, WORD_LEN);//0x28 kenxu update @20120518
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg = 0;
            rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5580, &tmp_reg);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg &= 0x00FF;
            tmp_reg &= 0x0006;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5580, tmp_reg, 1,WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }    
			break;
		}
    case CAMERA_EFFECT_MONO: 
		{
			pr_err("--CAMERA-- %s ...CAMERA_EFFECT_OFF\n",__func__);
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5583, 0x0080, 1,WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5584, 0x0080, 1,WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg = 0;
            rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5580, &tmp_reg);
            if (rc < 0)
            {
               return rc;
            }
	            tmp_reg &= 0x0006;
            tmp_reg |= 0x0018;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5580, tmp_reg, 1,BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }	
			break;
		}

	case CAMERA_EFFECT_SEPIA: 
		{
			pr_err("--CAMERA-- %s ...CAMERA_EFFECT_SEPIA\n",__func__);
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5583, 0x0040, 1,WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5584, 0x00a0, 1,WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg = 0;
            rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5580, &tmp_reg);
            if (rc < 0)
            {
               return rc;
            }
	            tmp_reg &= 0x0006;
            tmp_reg |= 0x0018;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5580, tmp_reg,1, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }	
			break;
		}

	case CAMERA_EFFECT_NEGATIVE:
		{
		    pr_err("--CAMERA-- %s ...CAMERA_EFFECT_NEGATIVE\n",__func__);
		    rc = ov5640_i2c_write(ov5640_client->addr, 0x5583, 0x0040, 1,WORD_LEN);
		    if (rc < 0)
		    {
			return rc;
		    }
		    rc = ov5640_i2c_write(ov5640_client->addr, 0x5584, 0x0028, 1, WORD_LEN);
	            if (rc < 0)
	            {
	               return rc;
	            }
	            tmp_reg = 0;
	            rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5580, &tmp_reg);
	            if (rc < 0)
	            {
	               return rc;
	            }
	            tmp_reg &= 0x0006;
            tmp_reg |= 0x0040;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5580, tmp_reg, 1,BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }	
			break;
		}
	default:
		{
			pr_err("--CAMERA-- %s ...Default(Not Support)\n",__func__);
		}
	}
	ov5640_effect = effect;
	//Refresh Sequencer /
	pr_err("--CAMERA-- %s ...(End)\n",__func__);
#endif
	return rc;
}

#if 1
static int ov5640_set_brightness(int8_t brightness)
{
	int rc = 0;
      unsigned int  tmp_reg = 0;
      
	pr_err("--CAMERA-- %s ...(Start)\n",__func__);
	pr_err("--CAMERA-- %s ...brightness = %d\n",__func__ , brightness);
#if 1
	switch (brightness)
	{
	case CAMERA_BRIGHTNESS_0:
		pr_err("--CAMERA--CAMERA_BRIGHTNESS_LV0\n");
        {
                    tmp_reg = 0;
                    rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5001, &tmp_reg);
                    if (rc < 0)
                    {
                       return rc;
                    }
                    tmp_reg &= 0x00FF;
                    tmp_reg |= 0x0080;
                    rc = ov5640_i2c_write(ov5640_client->addr, 0x5001, tmp_reg, 1, BYTE_LEN);
                    if (rc < 0)
                    {
                       return rc;
                    }
                    rc = ov5640_i2c_write(ov5640_client->addr, 0x5587, 0x0030, 1,WORD_LEN);
                    if (rc < 0)
                    {
                       return rc;
                    }
                    //WT_CAM_20110411 write 5580
                    rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5580, &tmp_reg);
                    if (rc < 0)
                    {
                       return rc;
                    }
                    tmp_reg |= 0x0004;
                    rc = ov5640_i2c_write(ov5640_client->addr, 0x5580, tmp_reg, 1,BYTE_LEN);
                    if (rc < 0)
                    {
                       return rc;
                    }   
                    tmp_reg = 0;
                    rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5588, &tmp_reg);
                    if (rc < 0)
                    {
                       return rc;
                    }
                    tmp_reg &= 0x00FF;
                    tmp_reg |= 0x0008;
                    rc = ov5640_i2c_write(ov5640_client->addr, 0x5588, tmp_reg, 1,BYTE_LEN);
                    if (rc < 0)
                    {
                       return rc;
                    }   
                }

		break;

	case CAMERA_BRIGHTNESS_1:
		pr_err("--CAMERA--CAMERA_BRIGHTNESS_LV1\n");	
        {
                    tmp_reg = 0;
                    rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5001, &tmp_reg);
                    if (rc < 0)
                    {
                       return rc;
                    }
                    tmp_reg &= 0x00FF;
                    tmp_reg |= 0x0080;
                    rc = ov5640_i2c_write(ov5640_client->addr, 0x5001, tmp_reg, 1, BYTE_LEN);
                    if (rc < 0)
                    {
                       return rc;
                    }
                    rc = ov5640_i2c_write(ov5640_client->addr, 0x5587, 0x0020, 1, WORD_LEN);
                    if (rc < 0)
                    {
                       return rc;
                    }
                    //WT_CAM_20110411 write 5580
                   rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5580, &tmp_reg);
                    if (rc < 0)
                    {
                       return rc;
                    }
                    tmp_reg |= 0x0004;
                    rc = ov5640_i2c_write(ov5640_client->addr, 0x5580, tmp_reg,1, BYTE_LEN);
                    if (rc < 0)
                    {
                       return rc;
                    }   
                    tmp_reg = 0;
                    rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5588, &tmp_reg);
                    if (rc < 0)
                    {
                       return rc;
                    }
                    tmp_reg &= 0x00FF;
                    tmp_reg |= 0x0008;
                    rc = ov5640_i2c_write(ov5640_client->addr, 0x5588, tmp_reg, 1, BYTE_LEN);
                    if (rc < 0)
                    {
                       return rc;
                    }   
                }

		break;

	case CAMERA_BRIGHTNESS_2:
		pr_err("--CAMERA--CAMERA_BRIGHTNESS_LV2\n");
        {
                    tmp_reg = 0;
                    rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5001, &tmp_reg);
                    if (rc < 0)
                    {
                       return rc;
                    }
                    tmp_reg &= 0x00FF;
                    tmp_reg |= 0x0080;
                    rc = ov5640_i2c_write(ov5640_client->addr, 0x5001, tmp_reg, 1, BYTE_LEN);
                    if (rc < 0)
                    {
                       return rc;
                    }
                    rc = ov5640_i2c_write(ov5640_client->addr, 0x5587, 0x0010, 1, WORD_LEN);
                    if (rc < 0)
                    {
                       return rc;
                    }
                    //WT_CAM_20110411 write 5580
                    rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5580, &tmp_reg);
                    if (rc < 0)
                    {
                       return rc;
                    }
                    tmp_reg |= 0x0004;
                    rc = ov5640_i2c_write(ov5640_client->addr, 0x5580, tmp_reg, 1, BYTE_LEN);
                    if (rc < 0)
                    {
                        return rc;
                    }             
                    tmp_reg = 0;
                    rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5588, &tmp_reg);
                    if (rc < 0)
                    {
                       return rc;
                    }
                    tmp_reg &= 0x00FF;
                    tmp_reg |= 0x0008;
                    rc = ov5640_i2c_write(ov5640_client->addr, 0x5588, tmp_reg, 1, BYTE_LEN);
                    if (rc < 0)
                    {
                       return rc;
                    }   
                }

		break;

	case CAMERA_BRIGHTNESS_3:
		pr_err("--CAMERA--CAMERA_BRIGHTNESS_LV3\n");
        {
                    tmp_reg = 0;
                    rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5001, &tmp_reg);
                    if (rc < 0)
                    {
                       return rc;
                    }
                    tmp_reg &= 0x00FF;
                    tmp_reg |= 0x0080;
                    rc = ov5640_i2c_write(ov5640_client->addr, 0x5001, tmp_reg, 1, BYTE_LEN);
                    if (rc < 0)
                    {
                       return rc;
                    }
                    rc = ov5640_i2c_write(ov5640_client->addr, 0x5587, 0x0000, 1,WORD_LEN);
                    if (rc < 0)
                    {
                       return rc;
                    }
                    //WT_CAM_20110411 write 5580
                    rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5580, &tmp_reg);
                    if (rc < 0)
                    {
                       return rc;
                    }
                    tmp_reg |= 0x0004;
                    rc = ov5640_i2c_write(ov5640_client->addr, 0x5580, tmp_reg, 1, BYTE_LEN);
                    if (rc < 0)
                    {
                       return rc;
                    }   
                    tmp_reg = 0;
                    rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5588, &tmp_reg);
                    if (rc < 0)
                    {
                       return rc;
                    }
                    tmp_reg &= 0x00F7;
                    rc = ov5640_i2c_write(ov5640_client->addr, 0x5588, tmp_reg, 1,BYTE_LEN);
                    if (rc < 0)
                    {
                       return rc;
                    }  
                    
                }

		break;

	case CAMERA_BRIGHTNESS_4:
		pr_err("--CAMERA--CAMERA_BRIGHTNESS_LV4\n");
        {
                    tmp_reg = 0;
                    rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5001, &tmp_reg);
                    if (rc < 0)
                    {
                       return rc;
                    }
                    tmp_reg &= 0x00FF;
                    tmp_reg |= 0x0080;
                    rc = ov5640_i2c_write(ov5640_client->addr, 0x5001, tmp_reg, 1, BYTE_LEN);
                    if (rc < 0)
                    {
                       return rc;
                    }
                    rc = ov5640_i2c_write(ov5640_client->addr, 0x5587, 0x0010, 1, WORD_LEN);
                    if (rc < 0)
                    {
                       return rc;
                    }
                    //WT_CAM_20110411 write 5580
                   rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5580, &tmp_reg);
                    if (rc < 0)
                    {
                       return rc;
                    }
                    tmp_reg |= 0x0004;
                    rc = ov5640_i2c_write(ov5640_client->addr, 0x5580, tmp_reg, 1, BYTE_LEN);
                    if (rc < 0)
                    {
                       return rc;
                    }   
                    tmp_reg = 0;
                    rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5588, &tmp_reg);
                    if (rc < 0)
                    {
                       return rc;
                    }
                    tmp_reg &= 0x00F7;
                    rc = ov5640_i2c_write(ov5640_client->addr, 0x5588, tmp_reg, 1, BYTE_LEN);
                    if (rc < 0)
                    {
                       return rc;
                    }   
                }

		break;

	case CAMERA_BRIGHTNESS_5:
		pr_err("--CAMERA--CAMERA_BRIGHTNESS_LV5\n");
        {
                    tmp_reg = 0;
                    rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5001, &tmp_reg);
                    if (rc < 0)
                    {
                       return rc;
                    }
                    tmp_reg &= 0x00FF;
                    tmp_reg |= 0x0080;
                    rc = ov5640_i2c_write(ov5640_client->addr, 0x5001, tmp_reg, 1, BYTE_LEN);
                    if (rc < 0)
                    {
                       return rc;
                    }
                    rc = ov5640_i2c_write(ov5640_client->addr, 0x5587, 0x0020, 1, WORD_LEN);
                    if (rc < 0)
                    {
                       return rc;
                    }
                    //WT_CAM_20110411 write 5580
                    rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5580, &tmp_reg);
                    if (rc < 0)
                    {
                       return rc;
                    }
                    tmp_reg |= 0x0004;
                    rc = ov5640_i2c_write(ov5640_client->addr, 0x5580, tmp_reg, 1, BYTE_LEN);
                    if (rc < 0)
                    {
                       return rc;
                    }   
                    tmp_reg = 0;
                    rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5588, &tmp_reg);
                    if (rc < 0)
                    {
                       return rc;
                    }
                    tmp_reg &= 0x00F7;
                    rc = ov5640_i2c_write(ov5640_client->addr, 0x5588, tmp_reg, 1, BYTE_LEN);
                    if (rc < 0)
                    {
                       return rc;
                    }
                }

		break;

	case CAMERA_BRIGHTNESS_6:
		pr_err("--CAMERA--CAMERA_BRIGHTNESS_LV6\n");
        {
                    tmp_reg = 0;
                    rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5001, &tmp_reg);
                    if (rc < 0)
                    {
                       return rc;
                    }
                    tmp_reg &= 0x00FF;
                    tmp_reg |= 0x0080;
                    rc = ov5640_i2c_write(ov5640_client->addr, 0x5001, tmp_reg, 1, BYTE_LEN);
                    if (rc < 0)
                    {
                       return rc;
                    }
                    rc = ov5640_i2c_write(ov5640_client->addr, 0x5587, 0x0030, 1, WORD_LEN);
                    if (rc < 0)
                    {
                       return rc;
                    }
                    //WT_CAM_20110411 write 5580
                    rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5580, &tmp_reg);
                    if (rc < 0)
                    {
                       return rc;
                    }
                    tmp_reg |= 0x0004;
                    rc = ov5640_i2c_write(ov5640_client->addr, 0x5580, tmp_reg, 1, BYTE_LEN);
                    if (rc < 0)
                    {
                       return rc;
                    }   
                    tmp_reg = 0;
                    rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5588, &tmp_reg);
                    if (rc < 0)
                    {
                       return rc;
                    }
                    tmp_reg &= 0x00F7;
                    rc = ov5640_i2c_write(ov5640_client->addr, 0x5588, tmp_reg, 1, BYTE_LEN);
                    if (rc < 0)
                    {
                       return rc;
                    }   
                }

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
static int ov5640_set_contrast(int contrast)
{
	int rc = 0;
      unsigned int  tmp_reg = 0;
	pr_err("--CAMERA-- %s ...(Start)\n",__func__);
	pr_err("--CAMERA-- %s ...contrast = %d\n",__func__ , contrast);
#if 1
	if (effect_value == CAMERA_EFFECT_OFF)
	{
		switch (contrast)
		{
		case CAMERA_CONTRAST_0:
			pr_err("--CAMERA--CAMERA_CONTRAST_LV0\n");
	     {
            //WT_CAM_20110421
            tmp_reg = 0;
            rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5001, &tmp_reg);
            if (rc < 0)
            {
               return rc;
            }

            /*
             * ZTE_LJ_CAM_20101026
             * fix bug of no preview image after changing effect mode repeatedly
             */
            tmp_reg &= 0x00FF;
            tmp_reg |= 0x0080;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5001, tmp_reg,1,BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg = 0;
            rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5580, &tmp_reg);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg &= 0x00FF;
            tmp_reg |= 0x0004;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5580, tmp_reg, 1,WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }            
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5586, 0x0010, 1,WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }            
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5585, 0x0010, 1,WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }            
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5588, 0x0001, 1,WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }            
        }
			break;
		case CAMERA_CONTRAST_1:
			pr_err("--CAMERA--CAMERA_CONTRAST_LV1\n");
		  	//rc = OV5640Core_WritePREG(ov5640_contrast_lv1_tbl);
	  {
            //WT_CAM_20110421
            tmp_reg = 0;
            rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5001, &tmp_reg);
            if (rc < 0)
            {
               return rc;
            }

            /*
             * ZTE_LJ_CAM_20101026
             * fix bug of no preview image after changing effect mode 
                repeatedly
             */
            tmp_reg &= 0x00FF;
            tmp_reg |= 0x0080;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5001, tmp_reg, 1, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg = 0;
            rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5580, &tmp_reg);
     
       if (rc < 0)
            {
               return rc;
            }
            tmp_reg &= 0x00FF;
            tmp_reg |= 0x0004;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5580, tmp_reg, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }            
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5586, 0x0018, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }            
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5585, 0x0018, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }            
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5588, 0x0001, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }            
        }
			break;
		case CAMERA_CONTRAST_2:
			pr_err("--CAMERA--CAMERA_CONTRAST_LV2\n");
	  {
            //WT_CAM_20110421
            tmp_reg = 0;
            rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5001, &tmp_reg);
            if (rc < 0)
            {
               return rc;
            }

            /*
             * ZTE_LJ_CAM_20101026
             * fix bug of no preview image after changing effect mode repeatedly
             */
            tmp_reg &= 0x00FF;
            tmp_reg |= 0x0080;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5001, tmp_reg, 1,BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg = 0;
            rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5580, &tmp_reg);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg &= 0x00FF;
            tmp_reg |= 0x0004;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5580, tmp_reg, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }            
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5586, 0x0020, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }            
                 rc = ov5640_i2c_write(ov5640_client->addr, 0x5585, 0x0000, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }            
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5588, 0x0001, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }            
        }
			break;
		case CAMERA_CONTRAST_3:
			pr_err("--CAMERA--CAMERA_CONTRAST_LV3\n");
	 {  
            //WT_CAM_20110421
            tmp_reg = 0;
            rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5001, &tmp_reg);
            if (rc < 0)
            {
               return rc;
            }

            /*
             * ZTE_LJ_CAM_20101026
             * fix bug of no preview image after changing effect mode repeatedly
             */
            tmp_reg &= 0x00FF;
            tmp_reg |= 0x0080;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5001, tmp_reg,1, BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg = 0;
            rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5580, &tmp_reg);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg &= 0x00FF;
            tmp_reg |= 0x0004;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5580, tmp_reg, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }            
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5586, 0x0024, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }            
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5585, 0x0010, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }            
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5588, 0x0001, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }            
        }
			break;
		case CAMERA_CONTRAST_4:
			pr_err("--CAMERA--CAMERA_CONTRAST_LV4\n");
		{   
            //WT_CAM_20110421
            tmp_reg = 0;
            rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5001, &tmp_reg);
            if (rc < 0)
            {
               return rc;
            }

            /*
             * ZTE_LJ_CAM_20101026
             * fix bug of no preview image after changing effect mode repeatedly
             */
            tmp_reg &= 0x00FF;
            tmp_reg |= 0x0080;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5001, tmp_reg, 1,BYTE_LEN);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg = 0;
            rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5580, &tmp_reg);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg &= 0x00FF;
            tmp_reg |= 0x0004;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5580, tmp_reg, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }            
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5586, 0x002c, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }            
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5585, 0x001c, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }        
            rc = ov5640_i2c_write(ov5640_client->addr, 0x5588, 0x0001, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }            
        }
			break;
            
		default:
			pr_err("--CAMERA--CAMERA_CONTRAST_ERROR COMMAND\n");
			break;
		}
	}
	#endif
	pr_err("--CAMERA-- %s ...(End)\n",__func__);
	return rc;
}
#endif

#if 1
static int ov5640_set_sharpness(int sharpness)
{
	int rc = 0;
	pr_err("--CAMERA-- %s ...(Start)\n",__func__);
	pr_err("--CAMERA-- %s ...sharpness = %d\n",__func__ , sharpness);
#if 1
	if (effect_value == CAMERA_EFFECT_OFF) 
	{
		switch(sharpness)
		{
		case CAMERA_SHARPNESS_0:
			pr_err("--CAMERA--CAMERA_SHARPNESS_LV0\n");
            {
                        rc = ov5640_i2c_write(ov5640_client->addr, 0x5308 ,0x0065, 1, WORD_LEN);
                        if (rc < 0)
                        {
                           return rc;
                        }
                        rc = ov5640_i2c_write(ov5640_client->addr, 0x5302  ,0x0018, 1, WORD_LEN);
                        if (rc < 0)
                        {
                           return rc;
                        }
                        rc = ov5640_i2c_write(ov5640_client->addr, 0x5303 ,0x0000,  1,WORD_LEN);
                        if (rc < 0)
                        {
                           return rc;
                        }
            }

			break;
		case CAMERA_SHARPNESS_1:
			pr_err("--CAMERA--CAMERA_SHARPNESS_LV1\n");			
            {
                        rc = ov5640_i2c_write(ov5640_client->addr, 0x5308 ,0x0065, 1,WORD_LEN);
                        if (rc < 0)
                        {
                           return rc;
                        }
                        rc = ov5640_i2c_write(ov5640_client->addr, 0x5302  ,0x0010,1, WORD_LEN);
                        if (rc < 0)
                        {
                           return rc;
                        }
                        rc = ov5640_i2c_write(ov5640_client->addr, 0x5303  ,0x0000, 1, WORD_LEN);
                        if (rc < 0)
                        {
                           return rc;
                        }
             }
			break;
		case CAMERA_SHARPNESS_2:
			pr_err("--CAMERA--CAMERA_SHARPNESS_LV2\n");			
            {
                        rc = ov5640_i2c_write(ov5640_client->addr, 0x5308 ,0x0025, 1,WORD_LEN);
                        if (rc < 0)
                        {
                           return rc;
                        }
                        rc = ov5640_i2c_write(ov5640_client->addr, 0x5302 ,0x0030, 1,WORD_LEN); //0x10 //kenxu update @20120517 for ourdoor sharpness
                        if (rc < 0)
                        {
                           return rc;
                        }
                        rc = ov5640_i2c_write(ov5640_client->addr, 0x5303 ,0x0000, 1,WORD_LEN); //0x08 //kenxu update @20120517 for ourdoor sharpness
                        if (rc < 0)
                        {
                           return rc;
                        }
            }

			break;
		case CAMERA_SHARPNESS_3:
			pr_err("--CAMERA--CAMERA_SHARPNESS_LV3\n");			
            {
                        rc = ov5640_i2c_write(ov5640_client->addr, 0x5308 ,0x0065, 1,WORD_LEN);
                        if (rc < 0)
                        {
                           return rc;
                        }
                        rc = ov5640_i2c_write(ov5640_client->addr, 0x5302 ,0x0008, 1, WORD_LEN);
                        if (rc < 0)
                        {
                           return rc;
                        }
                        rc = ov5640_i2c_write(ov5640_client->addr, 0x5303 ,0x0000,  1, WORD_LEN);
                        if (rc < 0)
                        {
                           return rc;
                        }
            }

			break;
		case CAMERA_SHARPNESS_4:
			pr_err("--CAMERA--CAMERA_SHARPNESS_LV4\n");			
            {
                        rc = ov5640_i2c_write(ov5640_client->addr, 0x5308 ,0x0065, 1,WORD_LEN);
                        if (rc < 0)
                        {
                           return rc;
                        }
                        rc = ov5640_i2c_write(ov5640_client->addr, 0x5302 ,0x0002, 1,WORD_LEN);
                        if (rc < 0)
                        {
                           return rc;
                        }
                        rc = ov5640_i2c_write(ov5640_client->addr, 0x5303 ,0x0000, 1,WORD_LEN);
                        if (rc < 0)
                        {
                           return rc;
                        }
                    }
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
static int ov5640_set_saturation(int saturation)
{
	long rc = 0;
     unsigned int  tmp_reg = 0;
      
	//int i = 0;
	pr_err("--CAMERA-- %s ...(Start)\n",__func__);
	pr_err("--CAMERA-- %s ...saturation = %d\n",__func__ , saturation);
#if 1
	if (effect_value == CAMERA_EFFECT_OFF)
	{ 
		switch (saturation)
		{
		case CAMERA_SATURATION_0:
			pr_err("--CAMERA--CAMERA_SATURATION_LV0\n");
			//rc = OV5640Core_WritePREG(ov5640_saturation_lv0_tbl);
            {
                        
                        //WT_CAM_20110421
                        tmp_reg = 0;
                        rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5001, &tmp_reg);
                        if (rc < 0)
                        {
                           return rc;
                        }
            
                        /*
                         * ZTE_LJ_CAM_20101026
                         * fix bug of no preview image after changing effect mode repeatedly
                         */
                        tmp_reg &= 0x00FF;
                        tmp_reg |= 0x0080;
                        rc = ov5640_i2c_write(ov5640_client->addr, 0x5001, tmp_reg, 1, BYTE_LEN);
                        if (rc < 0)
                        {
                           return rc;
                        }
                        rc = ov5640_i2c_write(ov5640_client->addr, 0x5583, 0x0020, 1, WORD_LEN);
                        if (rc < 0)
                        {
                           return rc;
                        } 
                             
                        rc = ov5640_i2c_write(ov5640_client->addr, 0x5584, 0x0010, 1,WORD_LEN);
                        if (rc < 0)
                        {
                           return rc;
                        }
                        tmp_reg = 0;
                        rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5580, &tmp_reg);
                        if (rc < 0)
                        {
                           return rc;
                        }
                        tmp_reg &= 0x00FF;
                        tmp_reg |= 0x0002;
                        rc = ov5640_i2c_write(ov5640_client->addr, 0x5580, tmp_reg, 1,WORD_LEN);
                        if (rc < 0)
                        {
                           return rc;
                        }    
                        tmp_reg = 0;
                        
                        rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5588, &tmp_reg);
                        if (rc < 0)
                        {
                           return rc;
                        }
                        tmp_reg &= 0x00FF;
                        tmp_reg |= 0x0040;
                        rc = ov5640_i2c_write(ov5640_client->addr, 0x5588, tmp_reg, 1, WORD_LEN);
                        if (rc < 0)
                        {
                           return rc;
                        }              
                    }

			break;
		case CAMERA_SATURATION_1:
			pr_err("--CAMERA--CAMERA_SATURATION_LV1\n");
			//rc = OV5640Core_WritePREG(ov5640_saturation_lv1_tbl);
            {
                        //WT_CAM_20110421
                        tmp_reg = 0;
                        rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5001, &tmp_reg);
                        if (rc < 0)
                        {
                           return rc;
                        }
            
                        /*
                         * ZTE_LJ_CAM_20101026
                         * fix bug of no preview image after changing effect mode repeatedly
                         */
                        tmp_reg &= 0x00FF;
                        tmp_reg |= 0x0080;
                        rc = ov5640_i2c_write(ov5640_client->addr, 0x5001, tmp_reg, 1,BYTE_LEN);
                        if (rc < 0)
                        {
                           return rc;
                        }
                        rc = ov5640_i2c_write(ov5640_client->addr, 0x5583, 0x0038, 1, WORD_LEN);
                        if (rc < 0)
                        {
                           return rc;
                        } 
                             
                        rc = ov5640_i2c_write(ov5640_client->addr, 0x5584, 0x0020, 1,WORD_LEN);
                        if (rc < 0)
                        {
                           return rc;
                        }            
                        tmp_reg = 0;
                        rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5580, &tmp_reg);
                        if (rc < 0)
                        {
                           return rc;
                        }
                        tmp_reg &= 0x00FF;
                        tmp_reg |= 0x0002;
                        rc = ov5640_i2c_write(ov5640_client->addr, 0x5580, tmp_reg, 1,WORD_LEN);
                        if (rc < 0)
                        {
                           return rc;
                        }  
                        tmp_reg = 0;
                        rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5588, &tmp_reg);
                        if (rc < 0)
                        {
                           return rc;
                        }
                        tmp_reg &= 0x00FF;
                        tmp_reg |= 0x0040;
                        rc = ov5640_i2c_write(ov5640_client->addr, 0x5588, tmp_reg, 1,WORD_LEN);
                        if (rc < 0)
                        {
                           return rc;
                        }      
                    }

			break;
		case CAMERA_SATURATION_2:
			pr_err("--CAMERA--CAMERA_SATURATION_LV2\n");
            {
                        //WT_CAM_20110421
                        tmp_reg = 0;
                        rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5001, &tmp_reg);
                        if (rc < 0)
                        {
                           return rc;
                        }
            
                        /*
                         * ZTE_LJ_CAM_20101026
                         * fix bug of no preview image after changing effect mode repeatedly
                         */
                        tmp_reg &= 0x00FF;
                        tmp_reg |= 0x0080;
                        rc = ov5640_i2c_write(ov5640_client->addr, 0x5001, tmp_reg, 1,BYTE_LEN);
                        if (rc < 0)
                        {
                           return rc;
                        }
                        rc = ov5640_i2c_write(ov5640_client->addr, 0x5583, 0x0040, 1,WORD_LEN);
                        if (rc < 0)
                        {
                           return rc;
                        } 
                        rc = ov5640_i2c_write(ov5640_client->addr, 0x5584, 0x001e, 1,WORD_LEN);//0x28 kenxu update @20120518
                        if (rc < 0)
                        {
                           return rc;
                        }
                        tmp_reg = 0;
                        rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5580, &tmp_reg);
                        if (rc < 0)
                        {
                           return rc;
                        }
                        tmp_reg &= 0x00FF;
                        tmp_reg |= 0x0002;
                     
                        rc = ov5640_i2c_write(ov5640_client->addr, 0x5580, tmp_reg, 1,WORD_LEN);
                        if (rc < 0)
                        {
                           return rc;
                        }
                        tmp_reg = 0;
                        rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5588, &tmp_reg);
                        if (rc < 0)
                        {
                           return rc;
                        }
                        tmp_reg &= 0x00FF;
                        tmp_reg &= 0x00bf;
                        rc = ov5640_i2c_write(ov5640_client->addr, 0x5588, tmp_reg, 1, WORD_LEN);
                        if (rc < 0)
                        {
                           return rc;
                        }
                    }

			break;
		case CAMERA_SATURATION_3:
			pr_err("--CAMERA--CAMERA_SATURATION_LV3\n");
			//rc = OV5640Core_WritePREG(ov5640_saturation_lv3_tbl);
            {
                        //WT_CAM_20110421
                        tmp_reg = 0;
                        rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5001, &tmp_reg);
                        if (rc < 0)
                        {
                           return rc;
                        }
            
                        /*
                         * ZTE_LJ_CAM_20101026
                         * fix bug of no preview image after changing effect mode repeatedly
                         */
                        tmp_reg &= 0x00FF;
                        tmp_reg |= 0x0080;
                        rc = ov5640_i2c_write(ov5640_client->addr, 0x5001, tmp_reg,1, BYTE_LEN);
                        if (rc < 0)
                        {
                           return rc;
                        }
                        rc = ov5640_i2c_write(ov5640_client->addr, 0x5583, 0x0050, 1,WORD_LEN);
                        if (rc < 0)
                        {
                           return rc;
                        } 
                             
                        rc = ov5640_i2c_write(ov5640_client->addr, 0x5584, 0x0038, 1,WORD_LEN);
                        if (rc < 0)
                        {
                           return rc;
                        }
                        tmp_reg = 0;
                        rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5580, &tmp_reg);
                        if (rc < 0)
                        {
                           return rc;
                        }
                        tmp_reg &= 0x00FF;
                        tmp_reg |= 0x0002;
                        rc = ov5640_i2c_write(ov5640_client->addr, 0x5580, tmp_reg, 1, WORD_LEN);
                        if (rc < 0)
                        {
                           return rc;
                        }  
                        tmp_reg = 0;
                        rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5588, &tmp_reg);
                        if (rc < 0)
                        {
                           return rc;
                        }
                        tmp_reg &= 0x00FF;
                        tmp_reg |= 0x0040;
                        rc = ov5640_i2c_write(ov5640_client->addr, 0x5588, tmp_reg, 1, WORD_LEN);
                        if (rc < 0)
                        {
                           return rc;
                        }      
                    }

			break;
		case CAMERA_SATURATION_4:
			pr_err("--CAMERA--CAMERA_SATURATION_LV4\n");
			//rc = OV5640Core_WritePREG(ov5640_saturation_default_lv4_tbl);
            {
                        //WT_CAM_20110421
                        tmp_reg = 0;
                        rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5001, &tmp_reg);
                        if (rc < 0)
                        {
                           return rc;
                        }
            
                        /*
                         * ZTE_LJ_CAM_20101026
                         * fix bug of no preview image after changing effect mode repeatedly
                         */
                        tmp_reg &= 0x00FF;
                        tmp_reg |= 0x0080;
                        rc = ov5640_i2c_write(ov5640_client->addr, 0x5001, tmp_reg, 1,BYTE_LEN);
                        if (rc < 0)
                        {
                           return rc;
                        }
                        rc = ov5640_i2c_write(ov5640_client->addr, 0x5583, 0x0060, 1,WORD_LEN);
                        if (rc < 0)
                        {
                           return rc;
                        }
                                
                        rc = ov5640_i2c_write(ov5640_client->addr, 0x5584, 0x0048, 1, WORD_LEN);
                        if (rc < 0)
                        {
                           return rc;
                        }
                        tmp_reg = 0;
                        rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5580, &tmp_reg);
                        if (rc < 0)
                        {
                           return rc;
                        }
                        tmp_reg &= 0x00FF;
                        tmp_reg |= 0x0002;
                        rc = ov5640_i2c_write(ov5640_client->addr, 0x5580, tmp_reg, 1, WORD_LEN);
                        if (rc < 0)
                        {
                           return rc;
                        }  
                        tmp_reg = 0;
                        rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x5588, &tmp_reg);
                        if (rc < 0)
                        {
                           return rc;
                        }
                        tmp_reg &= 0x00FF;
                        tmp_reg |= 0x0040;
                        rc = ov5640_i2c_write(ov5640_client->addr, 0x5588, tmp_reg, 1, WORD_LEN);
                        if (rc < 0)
                        {
                           return rc;
                        }      
                    }

			break;
		
		default:
			pr_err("--CAMERA--CAMERA_SATURATION_ERROR COMMAND\n");
			break;
		}
	}
	#if 0
	  //for recover saturation level when change special effect
		switch (saturation)
		{
		case CAMERA_SATURATION_LV0:
			pr_err("--CAMERA--CAMERA_SATURATION_LV0\n");
			SAT_U = 0x00;
			SAT_V = 0x00;
			break;
		case CAMERA_SATURATION_LV1:
			pr_err("--CAMERA--CAMERA_SATURATION_LV1\n");
			SAT_U = 0x10;
			SAT_V = 0x10;
			break;
		case CAMERA_SATURATION_LV2:
			pr_err("--CAMERA--CAMERA_SATURATION_LV2\n");
			SAT_U = 0x20;
			SAT_V = 0x20;
			break;
		case CAMERA_SATURATION_LV3:
			pr_err("--CAMERA--CAMERA_SATURATION_LV3\n");
			SAT_U = 0x30;
			SAT_V = 0x30;
			break;
		case CAMERA_SATURATION_LV4:
			pr_err("--CAMERA--CAMERA_SATURATION_LV4\n");
			SAT_U = 0x40;
			SAT_V = 0x40;			break;
		case CAMERA_SATURATION_LV5:
			pr_err("--CAMERA--CAMERA_SATURATION_LV5\n");
			SAT_U = 0x50;
			SAT_V = 0x50;			break;
		case CAMERA_SATURATION_LV6:
			pr_err("--CAMERA--CAMERA_SATURATION_LV6\n");
			SAT_U = 0x60;
			SAT_V = 0x60;
			break;
		case CAMERA_SATURATION_LV7:
			pr_err("--CAMERA--CAMERA_SATURATION_LV7\n");
			SAT_U = 0x70;
			SAT_V = 0x70;			break;
		case CAMERA_SATURATION_LV8:
			pr_err("--CAMERA--CAMERA_SATURATION_LV8\n");
			SAT_U = 0x80;
			SAT_V = 0x80;
			break;		
		default:
			pr_err("--CAMERA--CAMERA_SATURATION_ERROR COMMAND\n");
			break;
		}
        #endif
 #endif 	
	pr_err("--CAMERA-- %s ...(End)\n",__func__);
	return rc;
}
#endif
#if 1
static long ov5640_set_antibanding(int antibanding)
{
	long rc = 0;
       unsigned int  tmp_reg = 0;
	//int i = 0;
	pr_err("--CAMERA-- %s ...(Start)\n", __func__);
	pr_err("--CAMERA-- %s ...antibanding = %d\n", __func__, antibanding);
#if 1
	switch (antibanding)
	{
	case CAMERA_ANTIBANDING_SET_60HZ:		
	{
	    pr_err("--CAMERA--CAMERA_ANTIBANDING_60HZ\n");
            tmp_reg = 0;
            rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x3c01, &tmp_reg);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg |= 0x0080;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3C01, tmp_reg, 1,WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
            
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3C00, 0x0000, 1,WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }

			#if 0
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3A0A, 0x0000, 1,WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3A0B, 0x00f6, 1,WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3A0D, 0x0004, 1,WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
			#endif
        }
		break;

	case CAMERA_ANTIBANDING_SET_50HZ:
	{
	    pr_err("--CAMERA--CAMERA_ANTIBANDING_50HZ\n");        	  
            tmp_reg = 0;
            rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x3c01, &tmp_reg);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg |= 0x0080;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3C01, tmp_reg, 1,WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3C00, 0x0004, 1,WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
			#if 0
             
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3A08, 0x0001, 1,WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3A09, 0x0027, 1,WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3A0E, 0x0003, 1, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
			#endif
            pr_err("50hz");
        }
		break;
		
	case CAMERA_ANTIBANDING_SET_OFF:
	case CAMERA_ANTIBANDING_SET_AUTO:
	{
	   pr_err("--CAMERA--OFF or CAMERA_ANTIBANDING_AUTO\n");

/* ZTE_CAM_LJ_20120309 fix bug that in auto mode,there is still banding */

          tmp_reg = 0;
            rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x3c01, &tmp_reg);
            if (rc < 0)
            {
               return rc;
            }
            tmp_reg |= 0x0080;
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3C01, tmp_reg, 1,WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3C00, 0x0004, 1,WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }

			 #if 0
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3A08, 0x0001, 1,WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3A09, 0x0027, 1,WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3A0E, 0x0003, 1, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
			#endif

	     #if 0 //kenxu mask for 50hz as default 20120525
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3622, 0x0001, 1,WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3635, 0x001c, 1,WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3634, 0x0040, 1,WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3C00, 0x0004, 1,WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3C01, 0x0034, 1,WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3C04, 0x0028, 1,WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3C05, 0x0098, 1,WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3C06, 0x0000, 1,WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3C07, 0x0008, 1,WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3C08, 0x0000, 1,WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3C09, 0x001c, 1,WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x300c, 0x0022, 1,WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3C0A, 0x009c, 1,WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3C0B, 0x0040, 1,WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
	       #endif
        }
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
static long ov5640_set_exposure_mode(int mode)
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
		rc = OV5640Core_WritePREG(ov5640_regs.ae_average_tbl,ov5640_regs.ae_average_tbl_size);
		break;
	case CAMERA_SETAE_CENWEIGHT:
		pr_err("--CAMERA--CAMERA_SETAE_CENWEIGHT\n");
		rc = OV5640Core_WritePREG(ov5640_regs.ae_centerweight_tbl,ov5640_regs.ae_centerweight_tbl_size);		
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
static int32_t ov5640_lens_shading_enable(uint8_t is_enable)
{
	int32_t rc = 0;
	pr_err("--CAMERA--%s: ...(Start). enable = %d\n", __func__, is_enable);

	if(is_enable)
	{
		pr_err("%s: enable~!!\n", __func__);
		rc = OV5640Core_WritePREG(ov5640_regs.lens_shading_on_tbl,ov5640_regs.lens_shading_on_tbl_size);
	}
	else
	{
		pr_err("%s: disable~!!\n", __func__);
		rc = OV5640Core_WritePREG(ov5640_regs.lens_shading_off_tbl,ov5640_regs.lens_shading_off_tbl_size);
	}
	pr_err("--CAMERA--%s: ...(End). rc = %d\n", __func__, rc);
	return rc;
}

static int ov5640_set_sensor_mode(int mode, int res)
{
	int rc = 0;

	pr_err("--CAMERA-- ov5640_set_sensor_mode mode = %d, res = %d\n", mode, res);

	switch (mode)
	{
	case SENSOR_PREVIEW_MODE:
		pr_err("--CAMERA-- SENSOR_PREVIEW_MODE\n");
		rc = ov5640_setting(S_UPDATE_PERIODIC, S_RES_PREVIEW);
		break;

	case SENSOR_SNAPSHOT_MODE:
		pr_err("--CAMERA-- SENSOR_SNAPSHOT_MODE\n");
		rc = ov5640_setting(S_UPDATE_PERIODIC, S_RES_CAPTURE);
		break;

	case SENSOR_RAW_SNAPSHOT_MODE:
	default:
		pr_err("--CAMERA--ov5640_set_sensor_mode no support\n");
		rc = -EINVAL;
		break;
	}

	return rc;
}
#if 1
static int ov5640_set_wb_oem(uint8_t param)
{
	int rc = 0;
      
#if 1

	switch(param)
	{
	case CAMERA_WB_MODE_AWB:

		pr_err("--CAMERA--CAMERA_WB_MODE_AWB\n");
	 {
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3406, 0x0000, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
        }
		break;

	case CAMERA_WB_MODE_SUNLIGHT:
		pr_err("--CAMERA--CAMERA_WB_MODE_SUNLIGHT\n");
	{
        	  rc = ov5640_i2c_write(ov5640_client->addr, 0x3406 ,0x0001, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3400 ,0x0006, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3401 ,0x001c, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3402 ,0x0004, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3403 ,0x0000, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3404 ,0x0005, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3405 ,0x0020, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
          
        }
		break;
	case CAMERA_WB_MODE_INCANDESCENT:
		pr_err("--CAMERA--CAMERA_WB_MODE_INCANDESCENT\n");
	{
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3406 ,0x0001, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3400 ,0x0004, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3401 ,0x0010, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3402 ,0x0004, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3403 ,0x0000, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3404 ,0x0008, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3405 ,0x0040, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
        }
		break;
	case CAMERA_WB_MODE_FLUORESCENT:
		pr_err("--CAMERA--CAMERA_WB_MODE_FLUORESCENT\n");
	{
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3406 ,0x0001, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3400 ,0x0005, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3401 ,0x0048, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3402 ,0x0004, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3403 ,0x0000, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3404 ,0x0007, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3405 ,0x00c0, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
        }
		break;
	case CAMERA_WB_MODE_CLOUDY:
		pr_err("--CAMERA--CAMERA_WB_MODE_CLOUDY\n");  
	{
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3406 ,0x0001, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3400 ,0x0006, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3401 ,0x0048, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3402 ,0x0004, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3403 ,0x0000, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3404 ,0x0004, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3405 ,0x00d3, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
        }
		break;
	default:
		break;
	}
#endif
	return rc;
}
#endif
#if 0
static int ov5640_set_touchaec(uint32_t x,uint32_t y)
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
		ov5640_i2c_write(ov5640_client->addr, 0x5688 + i, aec_arr[i], 10);
	}

	return 1;
}
#endif
//QRD
#if 1
static int ov5640_set_exposure_compensation(int compensation)
{
	long rc = 0;
#if 1
	pr_err("--CAMERA-- %s ...(Start)\n",__func__);

	pr_err("--CAMERA-- %s ...exposure_compensation = %d\n",__func__ , compensation);

	switch(compensation)
	{
case CAMERA_EXPOSURE_0:
	pr_err("--CAMERA--CAMERA_EXPOSURE_COMPENSATION_LV0\n");
	rc = OV5640Core_WritePREG(ov5640_regs.exposure_compensation_lv0_tbl,
                                                 ov5640_regs.exposure_compensation_lv0_tbl_size);        
	break;
case CAMERA_EXPOSURE_1:
	pr_err("--CAMERA--CAMERA_EXPOSURE_COMPENSATION_LV1\n");
	rc = OV5640Core_WritePREG(ov5640_regs.exposure_compensation_lv1_tbl,
                                                 ov5640_regs.exposure_compensation_lv1_tbl_size);          
	break;
case CAMERA_EXPOSURE_2:
	pr_err("--CAMERA--CAMERA_EXPOSURE_COMPENSATION_LV2\n");
	rc = OV5640Core_WritePREG(ov5640_regs.exposure_compensation_lv2_tbl,
                                                 ov5640_regs.exposure_compensation_lv2_tbl_size);          
	break;
case CAMERA_EXPOSURE_3:
	pr_err("--CAMERA--CAMERA_EXPOSURE_COMPENSATION_LV3\n");
	rc = OV5640Core_WritePREG(ov5640_regs.exposure_compensation_lv3_tbl,
                                                  ov5640_regs.exposure_compensation_lv3_tbl_size);          
	break;
case CAMERA_EXPOSURE_4:
	pr_err("--CAMERA--CAMERA_EXPOSURE_COMPENSATION_LV3\n");
	rc = OV5640Core_WritePREG(ov5640_regs.exposure_compensation_lv4_tbl,
                                                 ov5640_regs.exposure_compensation_lv4_tbl_size);          
	break;
default:
	pr_err("--CAMERA--ERROR CAMERA_EXPOSURE_COMPENSATION\n");
	break;
	}

	pr_err("--CAMERA-- %s ...(End)\n",__func__);
#endif
	return rc;
}
#endif

/*
 * ISO Setting
 */
static int32_t ov5640_set_iso(int8_t iso_val)
{
    int32_t rc = 0;

    CDBG("%s: entry: iso_val=%d\n", __func__, iso_val);

    switch (iso_val)
    {
        case CAMERA_ISO_SET_AUTO:
        {
            //WT_CAM_20110428 iso value
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3A18 ,0x0000,1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
            #if 1 // \D6\F7\B9۲\E2\CA԰汾 2011-06-16 ken
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3A19 ,0x00f8, 1, WORD_LEN);
            #else
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3A19 ,0x0040, WORD_LEN);
            #endif
            if (rc < 0)
            {
               return rc;
            }
        }
        break;

        case CAMERA_ISO_SET_HJR:
        {
            //add code here     
        }
        break;

        case CAMERA_ISO_SET_100:
        {
        	 #if 0
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3503 ,0x0002, 1,WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }	
	        rc = ov5640_i2c_write(ov5640_client->addr, 0x350b ,0x0000, 1,WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }	
	        msleep(100);	
	         #endif
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3A18 ,0x0000, 1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3A19 ,0x0020, 1,WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
            #if 0
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3503 ,0x0000, 1,WORD_LEN);

            if (rc < 0)
            {
               return rc;
            }
            #endif
        }
        break;

        case CAMERA_ISO_SET_200:
        {
        	 #if 0
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3503 ,0x0002, 1,WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }	
	        rc = ov5640_i2c_write(ov5640_client->addr, 0x350b ,0x0000, 1,WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }	
	        msleep(100);					
	         #endif				
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3A18 ,0x0000,1, WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3A19 ,0x0040, 1,WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
            #if 0
           rc = ov5640_i2c_write(ov5640_client->addr, 0x3503 ,0x0000, 1,WORD_LEN);

            if (rc < 0)
            {
               return rc;
            }		
            #endif	
        }
        break;

        case CAMERA_ISO_SET_400:
        {
        	 #if 0
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3503 ,0x0002, 1,WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }	
	        rc = ov5640_i2c_write(ov5640_client->addr, 0x350b ,0x0000, 1,WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }	
	        msleep(100);	
	         #endif
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3A18 ,0x0000, 1,WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3A19 ,0x0080, 1,WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
            #if 0
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3503 ,0x0000, 1,WORD_LEN);

            if (rc < 0)
            {
               return rc;
            }			
            #endif
        }
        break;

        case CAMERA_ISO_SET_800:
        {
        	 #if 0
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3503 ,0x0002, 1,WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }	
	        rc = ov5640_i2c_write(ov5640_client->addr, 0x350b ,0x0000, 1,WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }	
	        msleep(100);				
	         #endif		
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3A18 ,0x0000, 1,WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3A19 ,0x00fc, 1,WORD_LEN);
            if (rc < 0)
            {
               return rc;
            }
            #if 0
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3503 ,0x0000,1,WORD_LEN); 

            if (rc < 0)
            {
               return rc;
            }
            #endif
        }
        break;

        default:
        {
            pr_err("%s: parameter error!\n", __func__);
            rc = -EFAULT;
        }     
    }

	return rc;
} 

/*
 * Auto Focus Trigger
 * WT_CAM_20110127 set af register to enable af function 
 */
static int32_t ov5640_af_trigger(void)
{
    int32_t rc;
/*ZTE_YGL_CAM_20111230
  *The new firmwore don't need to judge these regs
  */
#if 0
    int32_t rc_3024;
    int32_t rc_3025;
    int32_t rc_3026;
    int32_t rc_3027;
#endif
    int32_t rc_3028;
    //uint16_t af_status;
    unsigned int  af_ack=0x0002;
    uint32_t i;
    unsigned int /*af_status_3024,af_status_3025, af_status_3026, af_status_3027,*/af_status_3028;
  

    CDBG("%s: entry\n", __func__);
    pr_err("ygl ov5640_af_trigger: x_ratio=%d,y_ratio=%d,ov5640_TouchAF_x=%d,ov5640_TouchAF_y=%d\n", x_ratio,y_ratio,ov5640_TouchAF_x,ov5640_TouchAF_y);
/* ZTE_YGL_CAM_20111230
 * add the interface of touch AF 
 */
   //Use Trig Auto Focus command to start auto focus	

	if(ov5640_TouchAF_x >= 0 && ov5640_TouchAF_y >= 0) {
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3024, (int)ov5640_TouchAF_x, 10,WORD_LEN);
            if (rc < 0)
            {
                pr_err("%s: failed, rc=%d!\n", __func__, rc);
                goto done;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3025, (int)ov5640_TouchAF_y, 10,WORD_LEN);
            if (rc < 0)
            {
                pr_err("%s: failed, rc=%d!\n",__func__, rc);        
                goto done;
            }

            rc = ov5640_i2c_write(ov5640_client->addr, 0x3023, 0x0001, 10,WORD_LEN);
            if (rc < 0)
            {
                pr_err("%s: failed, rc=%d!\n", __func__, rc);
                goto done;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3022, 0x0081, 10,WORD_LEN);
            if (rc < 0)
            {
                pr_err("%s: failed, rc=%d!\n",__func__, rc);        
                goto done;
            }

	    for (i = 0; (i < 200) && (0x0000 != af_ack); ++i)
	    {
	        af_ack = 0x0002;
	        rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x3023, &af_ack);
			
	        if (rc < 0)
	        {
	          pr_err("%s: rc < 0\n", __func__);
	           goto done;
	        }        
		
	        mdelay(15);
		    pr_err("Trig _1 Auto Focus  i  = %d ",i);
	    }
	}else {

#if 0
			rc = ov5640_i2c_write(ov5640_client->addr, 0x3023, 0x0001, 10,WORD_LEN);
            if (rc < 0)
            {
                pr_err("%s: failed, rc=%d!\n", __func__, rc);
                goto done;
            }
            rc = ov5640_i2c_write(ov5640_client->addr, 0x3022, 0x0080, 10,WORD_LEN);
            if (rc < 0)
            {
                pr_err("%s: failed, rc=%d!\n",__func__, rc);        
                goto done;
            }
            /*
              * af_status: = 0x00,  AF is done successfully
              *            != 0x00, AF is being done
              *
              * i: < 100, time delay for tuning AF
              *    >= 100, time out for tuning AF
              */
            for (i = 0; (i < 200) && (0x0000 != af_ack); ++i)
            {
                af_ack = 0x0002;
                rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x3023, &af_ack);
            	
                if (rc < 0)
                {
                      pr_err("%s: rc < 0\n", __func__);
                      goto done;
                }        

                mdelay(15);
            	  //pr_err("else Trig _1 Auto Focus  i  = %d ",i);
            }
		#endif	
	}

        //Use Trig Auto Focus command to start auto focus	

        rc = ov5640_i2c_write(ov5640_client->addr, 0x3023, 0x0001, 1,WORD_LEN);
        if (rc < 0)
        {
            pr_err("%s: failed, rc=%d!\n", __func__, rc);
            goto done;
        }

        rc = ov5640_i2c_write(ov5640_client->addr, 0x3022, 0x0003, 1, WORD_LEN);
        if (rc < 0)
        {
            pr_err("%s: failed, rc=%d!\n",__func__, rc);        
            goto done;
        }
	
        /*
          * af_status: = 0x00,  AF is done successfully
          *            != 0x00, AF is being done
          *
          * i: < 100, time delay for tuning AF
          *    >= 100, time out for tuning AF
          */
        af_ack = 0x0002;//set af_ack value for compare its value from get 

        for (i = 0; (i < 200) && (0x0000 != af_ack); ++i)
        {
            af_ack = 0x0002;
            rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x3023, &af_ack);
        	
            if (rc < 0)
            {
              pr_err("%s: rc < 0\n", __func__);
               goto done;
            }        

            mdelay(15);
            //pr_err("Trig Auto Focus  i  = %d ",i);
        }
    
	rc_3028 = ov5640_i2c_read_byte(ov5640_client->addr, 0x3028, &af_status_3028);
	pr_err("af_status_3028=%d,\n", af_status_3028);
	

	if(0 != af_status_3028) {
		pr_err("Touch AF succeeded! %d,\n", af_status_3028);
		rc = 0;
		goto done;
	}else{
	    pr_err("Touch AF failed: %s: AF af_status_3028=%x,\n", __func__, af_status_3028);
		rc = -EFAULT;
		goto done;
	}

 done:
	ov5640_TouchAF_x = -1;
	ov5640_TouchAF_y = -1;

	pr_err("lijing:af return success\n");

	return 0; //always return success by lijing20120710
	//return rc;
}
    
/* ZTE_YGL_CAM_20111230
  * add the interface of touch AF for froyo
  */
static int32_t ov5640_set_aec_rio(aec_rio_cfg position)
{
    int32_t rc = 0;
    pr_err("ygl ov5640_set_aec_rio: position.preview_width=%d,position.preview_height=%d\n", position.preview_width,position.preview_height);
    pr_err("ygl ov5640_set_aec_rio: position.x=%d,position.y=%d\n", position.x,position.y);

    x_ratio = (position.preview_width/80);//OV5640_AF_WINDOW_FULL_WIDTH 
    y_ratio = (position.preview_height/60);//OV5640_AF_WINDOW_FULL_HEIGHT
    if(x_ratio != 0 && y_ratio != 0){
    	ov5640_TouchAF_x = (position.x / x_ratio);
    	ov5640_TouchAF_y = (position.y / y_ratio);
    }
	
    pr_err("ygl ov5640_set_aec_rio: x_ratio=%d,y_ratio=%d\n", x_ratio,y_ratio);
    return rc;
}   
   

#if 0
static int ov5640_sensor_start_af(void)
{
	int i;
	unsigned int af_st = 0;
	unsigned int af_ack = 0;
	unsigned int tmp = 0;
	int rc = 0;
	pr_err("--CAMERA-- %s (Start...)\n",__func__);

	ov5640_i2c_read_byte(ov5640_client->addr, OV5640_CMD_FW_STATUS,&af_st);
	pr_err("--CAMERA-- %s af_st = %d\n", __func__, af_st);

	ov5640_i2c_write(ov5640_client->addr, OV5640_CMD_ACK, 0x01, 10);
	ov5640_i2c_write(ov5640_client->addr, OV5640_CMD_MAIN, 0x03, 10);

	for (i = 0; i < 50; i++) {
		ov5640_i2c_read_byte(ov5640_client->addr,OV5640_CMD_ACK,&af_ack);
		if (af_ack == 0)
			break;
		msleep(50);
	}
	pr_err("--CAMERA-- %s af_ack = 0x%x\n", __func__, af_ack);

	//	if(af_ack == 0)
	{
		//		mdelay(1000);
		ov5640_i2c_read_byte(ov5640_client->addr, OV5640_CMD_FW_STATUS,&af_st);
		pr_err("--CAMERA-- %s af_st = %d\n", __func__, af_st);

		if (af_st == 0x10)
		{
			pr_err("--CAMERA-- %s AF ok and release AF setting~!!\n", __func__);
		}
		else {
			pr_err("--CAMERA-- %s AF not ready!!\n", __func__);
		}
	}

	//  ov5640_i2c_write(ov5640_client->addr,OV5640_CMD_ACK,0x01,10);
	//  ov5640_i2c_write(ov5640_client->addr,OV5640_CMD_MAIN,0x08,10);
	ov5640_i2c_write(ov5640_client->addr,OV5640_CMD_ACK,0x01,10);
	ov5640_i2c_write(ov5640_client->addr,OV5640_CMD_MAIN,0x07,10);

	for (i = 0; i < 70; i++)
	{
		ov5640_i2c_read_byte(ov5640_client->addr, OV5640_CMD_ACK, &af_ack);
		if (af_ack == 0)
			break;
		msleep(25);
	}

	ov5640_i2c_read_byte(ov5640_client->addr, OV5640_CMD_PARA0, &tmp);
	pr_err("0x3024 = %x \n", tmp);
	rc = ((tmp == 0) ? 1 : 0);

	ov5640_i2c_read_byte(ov5640_client->addr, OV5640_CMD_PARA1, &tmp);
	pr_err("0x3025 = %x \n", tmp);
	rc = ((tmp == 0) ? 1 : 0);

	ov5640_i2c_read_byte(ov5640_client->addr, OV5640_CMD_PARA2, &tmp);
	pr_err("0x3026 = %x \n", tmp);
	rc = ((tmp == 0) ? 1 : 0);

	ov5640_i2c_read_byte(ov5640_client->addr, OV5640_CMD_PARA3, &tmp);
	pr_err("0x3027 = %x \n", tmp);
	rc = ((tmp == 0) ? 1 : 0) ;

	ov5640_i2c_read_byte(ov5640_client->addr, OV5640_CMD_PARA4, &tmp);
	pr_err("0x3028 = %x \n", tmp);
	rc = ((tmp == 0) ? 1 : 0) ;

	pr_err("--CAMERA-- %s rc = %d(End...)\n", __func__, rc);
	return rc;
}
#endif
static int ov5640_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cdata;
	long rc = 0;

	if (copy_from_user(&cdata, (void *)argp, sizeof(struct sensor_cfg_data))) 
		return -EFAULT;

	pr_err("--CAMERA-- %s %d\n",__func__,cdata.cfgtype);

	mutex_lock(&ov5640_mutex);
	switch (cdata.cfgtype)
	{
	case CFG_SET_MODE:   // 0
		rc =ov5640_set_sensor_mode(cdata.mode, cdata.rs);
		break;
	case CFG_SET_EFFECT: // 1
		pr_err("--CAMERA-- CFG_SET_EFFECT mode=%d, effect = %d !!\n",cdata.mode, cdata.cfg.effect);
		rc = ov5640_set_effect(cdata.mode, cdata.cfg.effect);
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
		//ov5640_power_off();
		ov5640_pwdn_on();
		break;
	case CFG_SET_DEFAULT_FOCUS:  // 06
		pr_err("--CAMERA-- CFG_SET_DEFAULT_FOCUS (Not Implement) !!\n");
		break;        
	case CFG_MOVE_FOCUS:     //  07
		pr_err("--CAMERA-- CFG_MOVE_FOCUS (Not Implement) !!\n");
		break;
	case CFG_SET_BRIGHTNESS:     //  12
		pr_err("--CAMERA-- CFG_SET_BRIGHTNESS  !!\n");
		rc = ov5640_set_brightness(cdata.cfg.brightness);
		break;
	case CFG_SET_CONTRAST:     //  13
		pr_err("--CAMERA-- CFG_SET_CONTRAST  !!\n");
		rc = ov5640_set_contrast(cdata.cfg.contrast);
		break;            
	case CFG_SET_EXPOSURE_MODE:     //  15
		pr_err("--CAMERA-- CFG_SET_EXPOSURE_MODE !!\n");
		//rc = ov5640_set_exposure_mode(cdata.cfg.ae_mode);
		break;
	case CFG_SET_ANTIBANDING:     //  17
		//pr_err("--CAMERA-- CFG_SET_ANTIBANDING antibanding = %d!!\n", cdata.cfg.antibanding);
		rc = ov5640_set_antibanding(cdata.cfg.antibanding);
		break;
	case CFG_SET_LENS_SHADING:     //  20
		pr_err("--CAMERA-- CFG_SET_LENS_SHADING !!\n");
		rc = ov5640_lens_shading_enable(cdata.cfg.lens_shading);
		break;		
	case CFG_SET_SATURATION:     //  30
		pr_err("--CAMERA-- CFG_SET_SATURATION !!\n");
		rc = ov5640_set_saturation(cdata.cfg.saturation);
		break;

	case CFG_SET_SHARPNESS:     //  31
		pr_err("--CAMERA-- CFG_SET_SHARPNESS !!\n");
		rc = ov5640_set_sharpness(cdata.cfg.sharpness);
		break;
        
	case CFG_SET_WB:
		pr_err("--CAMERA-- CFG_SET_WB!!\n");
		rc = ov5640_set_wb_oem(cdata.cfg.wb_mode);
		break;
        
	case CFG_SET_ISO:
            pr_err("--CAMERA-- CFG_SET_ISO!!\n");
            rc = ov5640_set_iso(cdata.cfg.iso_val);
            break;
       case CFG_SET_AF:
            pr_err("--CAMERA-- CFG_SET_AF !\n");
            rc = ov5640_af_trigger();
            break;
        /* ZTE_YGL_CAM_20101230
         * add the interface of touch AF for froyo
         */
        case CFG_SET_AEC_RIO:
        {
            rc = ov5640_set_aec_rio(cdata.cfg.aec_rio);
        }
            break;
#if 0
	case CFG_SET_TOUCHAEC:
		pr_err("--CAMERA-- CFG_SET_TOUCHAEC!!\n");
		ov5640_set_touchaec(cdata.cfg.aec_cord.x, cdata.cfg.aec_cord.y);
		rc = 0 ;
		break;
	case CFG_SET_AUTO_FOCUS:
		pr_err("--CAMERA-- CFG_SET_AUTO_FOCUS !\n");
		rc = ov5640_sensor_start_af();
		break;
	case CFG_SET_AUTOFLASH:
		pr_err("--CAMERA-- CFG_SET_AUTOFLASH !\n");
		is_autoflash = cdata.cfg.is_autoflash;
		pr_err("[kylin] is autoflash %d\r\n",is_autoflash);
		rc = 0;
		break;
#endif	
	case CFG_SET_EXPOSURE_COMPENSATION:
		pr_err("--CAMERA-- CFG_SET_EXPOSURE_COMPENSATION !\n");
		rc = ov5640_set_exposure_compensation(cdata.cfg.exp_compensation);
		break;

	default:
		pr_err("--CAMERA-- %s: Command=%d (Not Implement)!!\n",__func__,cdata.cfgtype);
		rc = -EINVAL;
		break;    
	}
	mutex_unlock(&ov5640_mutex);
	return rc;    
}

static struct i2c_driver ov5640_i2c_driver = {
	.id_table = ov5640_i2c_id,
	.probe  = ov5640_i2c_probe,
	.remove = ov5640_i2c_remove,
	.driver = {
		.name = "ov5640",
	},
};

static int ov5640_probe_init_gpio(const struct msm_camera_sensor_info *data)
{
	int rc = 0;

	pr_err("--CAMERA-- %s Open CAMIO CLK,set to default clk rate!\n",__func__);

	ov5640_pwdn_gpio = data->sensor_pwd;
	ov5640_reset_gpio = data->sensor_reset;

	pr_err("--CAMERA-- %s : sensor_pwd_pin=%d, sensor_reset_pin=%d\n",__func__,data->sensor_pwd,data->sensor_reset);

	rc = gpio_request(data->sensor_pwd, "ov5640");
	pr_err("--CAMERA-- %s : gpio_request=%d, result is %d",__func__,data->sensor_pwd, rc);

	rc |= gpio_request(data->sensor_reset, "ov5640");
	pr_err("--CAMERA-- %s : gpio_request=%d, result is %d",__func__,data->sensor_reset, rc);

	if (rc < 0)
	{
		gpio_free(data->sensor_pwd);
		gpio_free(data->sensor_reset);
		rc = gpio_request(data->sensor_pwd, "ov5640");
		rc |= gpio_request(data->sensor_reset, "ov5640");
	}

	gpio_direction_output(data->sensor_reset, 1);
	gpio_direction_output(data->sensor_pwd, 1);

	return rc;
}

static void ov5640_probe_free_gpio(void)
{
	gpio_free(ov5640_pwdn_gpio);
	gpio_free(ov5640_reset_gpio);
	ov5640_pwdn_gpio = 0xFF;
	ov5640_reset_gpio = 0xFF;
}

static int ov5640_sensor_probe_init(const struct msm_camera_sensor_info *data)
{
    int rc = 0;

//if(ov5640_ctrl == NULL){
	ov5640_ctrl = kzalloc(sizeof(struct __ov5640_ctrl), GFP_KERNEL);
	if (!ov5640_ctrl)
	{
		pr_err("--CAMERA-- kzalloc ov5640_ctrl error !!\n");
		kfree(ov5640_ctrl);
		return rc;
	}
//}

	ov5640_ctrl->fps_divider = 1 * 0x00000400;
	ov5640_ctrl->pict_fps_divider = 1 * 0x00000400;
	ov5640_ctrl->set_test = S_TEST_OFF;
	ov5640_ctrl->prev_res = S_QTR_SIZE;
	ov5640_ctrl->pict_res = S_FULL_SIZE;

	if (data)
		ov5640_ctrl->sensordata = data;
	
     rc = msm_camera_power_backend(MSM_CAMERA_PWRUP_MODE);
     if (rc < 0)
     {
         pr_err("%s: camera_power_backend failed!\n", __func__);
         return rc;
     }
     
     /* SENSOR NEED MCLK TO DO I2C COMMUNICTION, OPEN CLK FIRST*/
     
#if defined(CONFIG_MACH_ATLAS40)  ||defined(CONFIG_MACH_BLADE2)			
     rc = msm_camera_clk_switch(data, OV5640_GPIO_SWITCH_CTL, OV5640_GPIO_SWITCH_VAL);
     if (rc < 0) {
         pr_err("msm_camera_clk_switch fail\n");
        goto probe_init_fail;
    }
#endif	
     msm_camio_clk_rate_set(24000000);
 
     mdelay(5);
 
    // ov5640_power_on();
       ov5640_pwdn_off();
       
     ov5640_power_reset();

    rc = ov5640_probe_readID(data);
    
    if (rc < 0)
    { 
        pr_err("--CAMERA--ov5640_probe_readID Fail !!~~~~!!\n");  
        goto probe_init_fail;
    }

	if (ov5640_ctrl->prev_res == S_QTR_SIZE)
		rc = ov5640_setting(S_REG_INIT, S_RES_PREVIEW);
	else
		rc = ov5640_setting(S_REG_INIT, S_RES_CAPTURE);

	if (rc < 0)
	{
		pr_err("--CAMERA-- %s : ov5640_setting failed. rc = %d\n",__func__,rc);
		kfree(ov5640_ctrl);
		goto probe_init_fail;
	}	
	
	return 0;
		
probe_init_fail:
    msm_camera_power_backend(MSM_CAMERA_PWRDWN_MODE);
    return rc;
	
}

static int ov5640_sensor_probe(const struct msm_camera_sensor_info *info,struct msm_sensor_ctrl *s)
{
	int rc = -ENOTSUPP;
	pr_err("--CAMERA-- %s (Start...)\n",__func__);
	rc = i2c_add_driver(&ov5640_i2c_driver);
	pr_err("--CAMERA-- i2c_add_driver ret:0x%x,ov5640_client=0x%x\n",
		rc, (unsigned int)ov5640_client);
	if ((rc < 0 ) || (ov5640_client == NULL))
	{
		pr_err("--CAMERA-- i2c_add_driver FAILS!!\n");
		return rc;
	}

     rc = ov5640_probe_init_gpio(info);
 
   //  ov5640_power_off();
        ov5640_pwdn_on();

     rc = ov5640_sensor_probe_init(info);
     if (rc < 0)
     {
         goto probe_fail;
     }

	s->s_init = ov5640_sensor_open_init;
	s->s_release = ov5640_sensor_release;
	s->s_config  = ov5640_sensor_config;
	//s->s_AF = ov5640_sensor_set_af;
	//camera_init_flag = true;

	s->s_camera_type = BACK_CAMERA_2D;
	s->s_mount_angle = info->sensor_platform_info->mount_angle;

	//ov5640_power_off();
	ov5640_sensor_release();

	pr_err("--CAMERA-- %s (End...)\n",__func__);
	return 0;
    
probe_fail:
	pr_err("ov5640_sensor_probe: SENSOR PROBE FAILS!\n");
	i2c_del_driver(&ov5640_i2c_driver);
	//ov5640_power_off();
	ov5640_pwdn_on();
       gpio_set_value(ov5640_reset_gpio, 0);
	ov5640_probe_free_gpio();
	return rc;
}

static int ov5640_i2c_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	pr_err("--CAMERA-- %s ... (Start...)\n",__func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		pr_err("--CAMERA--i2c_check_functionality failed\n");
		return -ENOMEM;
	}

	ov5640_sensorw = kzalloc(sizeof(struct ov5640_work), GFP_KERNEL);
	if (!ov5640_sensorw)
	{
		pr_err("--CAMERA--kzalloc failed\n");
		return -ENOMEM;
	}
	i2c_set_clientdata(client, ov5640_sensorw);
	ov5640_init_client(client);
	ov5640_client = client;

	pr_err("--CAMERA-- %s ... (End...)\n",__func__);
	return 0;
}

static int __ov5640_probe(struct platform_device *pdev)
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
	return msm_camera_drv_start(pdev, ov5640_sensor_probe);
#else
	return msm_camera_drv_start(pdev, ov5640_sensor_probe,0);
#endif
}

static struct platform_driver msm_camera_driver = {
	.probe = __ov5640_probe,
	.driver = {
		.name = "msm_camera_ov5640",
		.owner = THIS_MODULE,
	},
};

static int __init ov5640_init(void)
{
	//ov5640_i2c_buf[0]=0x5A;
	return platform_driver_register(&msm_camera_driver);
}

module_init(ov5640_init);

MODULE_DESCRIPTION("OV5640 YUV MIPI sensor driver");
MODULE_LICENSE("GPL v2");
