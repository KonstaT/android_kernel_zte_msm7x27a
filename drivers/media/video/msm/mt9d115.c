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
#include "mt9d115.h"
#include <linux/slab.h>

static int mt9d115_pwdn_gpio;
static int mt9d115_reset_gpio;

#define FALSE 0
#define TRUE 1




static int MT9D115_CSI_CONFIG = 0;
#define REG_MT9D115_MCNEX_QTECH_RESET_AND_MISC_CTRL 0x001A
#define MT9D115_MCNEX_QTECH_SOC_RESET               0x0219  /* SOC is in soft reset */
#define MT9D115_MCNEX_QTECH_SOC_NOT_RESET           0x0218  /* SOC is not in soft reset */

#define REG_MT9D115_MCNEX_QTECH_STANDBY_CONTROL     0x0018
#define MT9D115_MCNEX_QTECH_SOC_STANDBY             0x402C  /* SOC is in standby state */

#define TRY_TIME             10
#define REG_MT9D115_MODEL_ID 0x0000
#define REG_MT9D115_STANDBY_CONTROL     0x0018


#define MT9D115_GPIO_SWITCH_CTL     49

#define MT9D115_GPIO_SWITCH_VAL     0


struct mt9d115_work {
	struct work_struct work;
};
static struct mt9d115_work *mt9d115_sensorw;
static struct i2c_client    *mt9d115_client;
static DECLARE_WAIT_QUEUE_HEAD(mt9d115_wait_queue);
DEFINE_MUTEX(mt9d115_mutex);

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


//static u8 mt9d115_i2c_buf[4];
//static u8 mt9d115_counter = 0;
static int is_autoflash = 0;
static int8_t current_brightness = CAMERA_BRIGHTNESS_3;
static int8_t current_exposure = CAMERA_EXPOSURE_2;

struct __mt9d115_ctrl 
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
static struct __mt9d115_ctrl *mt9d115_ctrl;

extern struct rw_semaphore leds_list_lock;
extern struct list_head leds_list;

static int mt9d115_i2c_remove(struct i2c_client *client);
static int mt9d115_i2c_probe(struct i2c_client *client,const struct i2c_device_id *id);

static int mt9d115_i2c_txdata(u16 saddr,u8 *txdata,int length)
{
	struct i2c_msg msg[] = {
		{
			.addr  = saddr,
				.flags = 0,
				.len = length,
				.buf = txdata,
		},
	};

	if (i2c_transfer(mt9d115_client->adapter, msg, 1) < 0)    return -EIO;
	else return 0;
}

#ifdef CONFIG_MACH_ATLAS40
static int32_t mt9d115_i2c_write(unsigned short saddr,
                                      unsigned short waddr,
                                      unsigned short wdata,
                                      enum mt9d115_width_t width)
{
    int32_t rc = -EFAULT;
    unsigned char buf[4];

    memset(buf, 0, sizeof(buf));

    switch (width)
    {
        case WORD_LEN:
        {
            buf[0] = (waddr & 0xFF00) >> 8;
            buf[1] = (waddr & 0x00FF);
            buf[2] = (wdata & 0xFF00) >> 8;
            buf[3] = (wdata & 0x00FF);

            rc = mt9d115_i2c_txdata(saddr << 1, buf, 4);
        }
        break;

        case BYTE_LEN:
        {
            buf[0] = waddr;
            buf[1] = wdata;

            rc = mt9d115_i2c_txdata(saddr << 1, buf, 2);
        }
        break;

        default:
        {
            rc = -EFAULT;
        }
        break;
    }

    if (rc < 0)
    {
        pr_err("%s: waddr = 0x%x, wdata = 0x%x, failed!\n", __func__, waddr, wdata);
    }

    return rc;
}
#else
static int32_t mt9d115_i2c_write(unsigned short saddr,
                                      unsigned short waddr,
                                      unsigned short wdata,
                                      enum mt9d115_width_t width)
{
    int32_t rc = -EFAULT;
    unsigned char buf[4];

    memset(buf, 0, sizeof(buf));

    switch (width)
    {
        case WORD_LEN:
        {
            buf[0] = (waddr & 0xFF00) >> 8;
            buf[1] = (waddr & 0x00FF);
            buf[2] = (wdata & 0xFF00) >> 8;
            buf[3] = (wdata & 0x00FF);

            rc = mt9d115_i2c_txdata(saddr, buf, 4);
        }
        break;

        case BYTE_LEN:
        {
            buf[0] = waddr;
            buf[1] = wdata;

            rc = mt9d115_i2c_txdata(saddr, buf, 2);
        }
        break;

        default:
        {
            rc = -EFAULT;
        }
        break;
    }

    if (rc < 0)
    {
        pr_err("%s: waddr = 0x%x, wdata = 0x%x, failed!\n", __func__, waddr, wdata);
    }

    return rc;
}
#endif

static int32_t mt9d115_i2c_write_table(struct mt9d115_i2c_reg_conf const *reg_conf_tbl,
                                             int len)
{
    uint32_t i;
    int32_t rc = 0;

    for (i = 0; i < len; i++)
    {
        rc = mt9d115_i2c_write(mt9d115_client->addr,
                               reg_conf_tbl[i].waddr,
                               reg_conf_tbl[i].wdata,
                               reg_conf_tbl[i].width);
        if (rc < 0)
        {
            break;
        }

        if (reg_conf_tbl[i].mdelay_time != 0)
        {
            mdelay(reg_conf_tbl[i].mdelay_time);
        }
    }

    return rc;
}

static int mt9d115_i2c_rxdata(unsigned short saddr,
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

	if (i2c_transfer(mt9d115_client->adapter, msgs, 2) < 0) {
		pr_err("mt9d115_i2c_rxdata failed!\n");
		return -EIO;
	}

	return 0;
}
#ifdef CONFIG_MACH_ATLAS40
static int32_t mt9d115_i2c_read_byte(unsigned short   saddr,
									unsigned int raddr, unsigned int *rdata)
{
	int rc = 0;
	unsigned char buf[2];
	
	memset(buf, 0, sizeof(buf));

	buf[0] = (raddr & 0xFF00)>>8;
	buf[1] = (raddr & 0x00FF);

	rc = mt9d115_i2c_rxdata(saddr << 1, buf, 1);
	if (rc < 0) {
		pr_err("mt9d115_i2c_read_byte failed!\n");
		return rc;
	}

	*rdata = buf[0];

	return rc;
}


static int32_t mt9d115_i2c_read(unsigned short   saddr,
							   unsigned int raddr, unsigned int *rdata)
{
	int rc = 0;
	unsigned char buf[2];

	memset(buf, 0, sizeof(buf));

	buf[0] = (raddr & 0xFF00)>>8;
	buf[1] = (raddr & 0x00FF);

	rc = mt9d115_i2c_rxdata(saddr << 1, buf, 2);
	if (rc < 0)	return rc;
	*rdata = buf[0] << 8 | buf[1];

	if (rc < 0)
		pr_err("mt9d115_i2c_read failed!\n");
	return rc;
}
#else
static int32_t mt9d115_i2c_read_byte(unsigned short   saddr,
									unsigned int raddr, unsigned int *rdata)
{
	int rc = 0;
	unsigned char buf[2];
	
	memset(buf, 0, sizeof(buf));

	buf[0] = (raddr & 0xFF00)>>8;
	buf[1] = (raddr & 0x00FF);

	rc = mt9d115_i2c_rxdata(saddr, buf, 1);
	if (rc < 0) {
		pr_err("mt9d115_i2c_read_byte failed!\n");
		return rc;
	}

	*rdata = buf[0];

	return rc;
}


static int32_t mt9d115_i2c_read(unsigned short   saddr,
							   unsigned int raddr, unsigned int *rdata)
{
	int rc = 0;
	unsigned char buf[2];

	memset(buf, 0, sizeof(buf));

	buf[0] = (raddr & 0xFF00)>>8;
	buf[1] = (raddr & 0x00FF);

	rc = mt9d115_i2c_rxdata(saddr, buf, 2);
	if (rc < 0)	return rc;
	*rdata = buf[0] << 8 | buf[1];

	if (rc < 0)
		pr_err("mt9d115_i2c_read failed!\n");
	return rc;
}
#endif

static void mt9d115_power_off(void)
{
	
	pr_err("--CAMERA-- %s ... (Start...)\n",__func__);
         mt9d115_i2c_write(mt9d115_client->addr, 0x0028, 0x0000, WORD_LEN);
	gpio_set_value(mt9d115_pwdn_gpio, 1);

	msleep(100);
	
	pr_err("--CAMERA-- %s ... (End...)\n",__func__);
}

static void mt9d115_power_on(void)
{
	pr_err("--CAMERA-- %s ... (Start...)\n",__func__);

	gpio_set_value(mt9d115_pwdn_gpio, 0);
       msleep(100);	

	pr_err("--CAMERA-- %s ... (End...)\n",__func__);
}
static void mt9d115_power_reset(void)
{
	pr_err("--CAMERA-- %s ... (Start...)\n",__func__);
	
	gpio_set_value(mt9d115_reset_gpio, 1);   //reset camera reset pin
	mdelay(10);	
	
	gpio_set_value(mt9d115_reset_gpio, 0);
	mdelay(10);

	gpio_set_value(mt9d115_reset_gpio, 1);
	mdelay(10);

	pr_err("--CAMERA-- %s ... (End...)\n",__func__);
}

static int mt9d115_probe_readID(const struct msm_camera_sensor_info *data)
{
	int rc = 0;    
	u32 device_id = 0;

	pr_err("--CAMERA-- %s (Start...)\n",__func__);
	pr_err("--CAMERA-- %s sensor poweron,begin to read ID!\n",__func__);

	rc = mt9d115_i2c_read(mt9d115_client->addr, 	REG_MT9D115_MODEL_ID, &device_id);
	if (rc < 0)
	{
		pr_err("--CAMERA-- %s ok , readI2C failed, rc = 0x%x\r\n", __func__, rc);
		return rc;   
	}  
	pr_err("--CAMERA-- %s  readID high byte, data = 0x%x\r\n", __func__, device_id);

#ifdef CONFIG_SENSOR_INFO
	msm_sensorinfo_set_sensor_id(device_id);
#endif

	if(device_id != MT9D115_SENSOR_ID)
	{
		pr_err("--CAMERA-- %s ok , device id error, should be 0x%x\r\n",
			__func__, MT9D115_SENSOR_ID);
		return -EINVAL;
	}
	else
	{
		pr_err("--CAMERA-- %s ok , device id=0x%x\n",__func__,MT9D115_SENSOR_ID);
		return 0;
	}
}

static int mt9d115_set_flash_light(enum led_brightness brightness)
{
	struct led_classdev *led_cdev;

	pr_err("mt9d115_set_flash_light brightness = %d\n", brightness);

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

static int mt9d115_video_config(void)
{
	int rc = 0;

    u32 status = 0;

    int i = 0;

	pr_err("--CAMERA-- mt9d115_video_config\n");

	pr_err("--CAMERA-- preview in, is_autoflash - 0x%x\n", is_autoflash);

	/* autoflash setting */
	if (is_autoflash == 1) {
		mt9d115_set_flash_light(LED_OFF);
	}
	rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0xA115,WORD_LEN);    
    if (rc < 0)
    {
    	pr_err("--CAMERA-- %s ok , writeI2C failed, rc = 0x%x\r\n", __func__, rc);
        return rc;    
    }  
	rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0000,WORD_LEN);    
    if (rc < 0)
    {
    	pr_err("--CAMERA-- %s ok , writeI2C failed, rc = 0x%x\r\n", __func__, rc);
        return rc;    
    }  
	rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0xA103,WORD_LEN);    
    if (rc < 0)
    {
    	pr_err("--CAMERA-- %s ok , writeI2C failed, rc = 0x%x\r\n", __func__, rc);
        return rc;    
    }  
	rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0001,WORD_LEN);    
    if (rc < 0)
    {
    	pr_err("--CAMERA-- %s ok , writeI2C failed, rc = 0x%x\r\n", __func__, rc);
        return rc;    
    }  
    for(i = 0;i < 30;i++) {
        rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0xA104, WORD_LEN);
        if (rc < 0)
        {
            return rc;
        }

        rc = mt9d115_i2c_read(mt9d115_client->addr, 0x0990, &status);
        if (rc < 0)
        {
            return rc;
        }
        if(0x03 == status){
            pr_err("read preview status successfully,i=%d\n",i);
            break;
        }
        else if (29 == i) {
            pr_err("read preview status fail,i=%d,status=0x%x\n",i,status);
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0xA115, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }

            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0000, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }

            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0xA103, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }

            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0001, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            msleep(150);
        }
        msleep(7);
    }
	return rc;
}


static int mt9d115_snapshot_config(void)
{
	int rc = 0;
	unsigned int tmp;

    u32 status = 0;

    int i = 0;
	pr_err("--CAMERA-- SENSOR_SNAPSHOT_MODE\n");
	pr_err("--CAMERA-- %s, snapshot in, is_autoflash - 0x%x\n", __func__, is_autoflash);

	if (is_autoflash == 1) {
		mt9d115_i2c_read_byte(mt9d115_client->addr, 0x350b, &tmp);
		pr_err("--CAMERA-- GAIN VALUE : %x\n", tmp);
		if ((tmp & 0x80) == 0) {
			mt9d115_set_flash_light(LED_OFF);
		} else {
			mt9d115_set_flash_light(LED_FULL);
		}
	}

   rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0xA115, WORD_LEN);
   if (rc < 0)
   {
       return rc;
   }
   
   rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0002, WORD_LEN);
   if (rc < 0)
   {
       return rc;
   }
   
   rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0xA103, WORD_LEN);
   if (rc < 0)
   {
       return rc;
   }
   
   rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0002, WORD_LEN);
   if (rc < 0)
   {
       return rc;
   }
   for(i = 0;i < 30;i++) {
        rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0xA104, WORD_LEN);
        if (rc < 0)
        {
            return rc;
        }

        rc = mt9d115_i2c_read(mt9d115_client->addr, 0x0990, &status);
        if (rc < 0)
        {
            return rc;
        }
        if(0x07 == status){
            pr_err("read snapshot status successfully,i=%d\n",i);
            break;
        }
        else if (29 == i) {
            pr_err("read snapshot status fail,i=%d,status=0x%x\n",i,status);
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0xA115, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }

            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0002, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }

            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0xA103, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }

            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0002, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            msleep(150);
        }
        msleep(7);
    }
	return rc;
}

static int mt9d115_setting(enum msm_s_reg_update rupdate,enum msm_s_setting rt)
{
	int rc = -EINVAL;
	//unsigned int tmp;
	struct msm_camera_csi_params mt9d115_csi_params;

	pr_err("--CAMERA-- %s (Start...), rupdate=%d \n",__func__,rupdate);

	switch (rupdate)
	{
	case S_UPDATE_PERIODIC:
		if(!MT9D115_CSI_CONFIG) {
			mt9d115_csi_params.lane_cnt = 1;
			mt9d115_csi_params.data_format = CSI_8BIT;
			mt9d115_csi_params.lane_assign = 0xe4;
			mt9d115_csi_params.dpcm_scheme = 0;
			mt9d115_csi_params.settle_cnt = 0x6;

			pr_err("%s: msm_camio_csi_config\n", __func__);

			rc = msm_camio_csi_config(&mt9d115_csi_params);				
			MT9D115_CSI_CONFIG = 1;

			msleep(20);

			mt9d115_power_on();			
		}
		
		

		if (S_RES_PREVIEW == rt) {
			pr_err("%s: enter preview mode\n", __func__);
			rc = mt9d115_video_config();
			//msleep(80);
		} else if (S_RES_CAPTURE == rt) {
		       pr_err("%s: enter snapshot mode\n", __func__);
			rc = mt9d115_snapshot_config();
			//msleep(200);
		}
		
		break; /* UPDATE_PERIODIC */

	case S_REG_INIT:
		pr_err("--CAMERA-- S_REG_INIT (Start)\n");


	rc = mt9d115_i2c_write_table(mt9d115_regs.pll_tbl, mt9d115_regs.pll_tbl_sz);
	rc = mt9d115_i2c_write_table(mt9d115_regs.prevsnap_tbl, mt9d115_regs.prevsnap_tbl_sz);

		/* reset fps_divider */
		mt9d115_ctrl->fps_divider = 1 * 0x0400;
		pr_err("--CAMERA-- S_REG_INIT (End)\n");
		break; /* case REG_INIT: */

	default:
		break;
	} /* switch (rupdate) */

	pr_err("--CAMERA-- %s (End), rupdate=%d \n",__func__,rupdate);

	return rc;
}

static int mt9d115_sensor_open_init(const struct msm_camera_sensor_info *data)
{
	//int rc = -ENOMEM;
	int rc = 0;
	pr_err("--CAMERA-- %s\n",__func__);

	mt9d115_ctrl->fps_divider = 1 * 0x00000400;
	mt9d115_ctrl->pict_fps_divider = 1 * 0x00000400;
	mt9d115_ctrl->set_test = S_TEST_OFF;
	mt9d115_ctrl->prev_res = S_QTR_SIZE;
	mt9d115_ctrl->pict_res = S_FULL_SIZE;

	if (data)
		mt9d115_ctrl->sensordata = data;

#if defined(CONFIG_MACH_MOONCAKE2) || defined(CONFIG_MACH_ATLAS40)  
     rc = msm_camera_clk_switch(data, MT9D115_GPIO_SWITCH_CTL, MT9D115_GPIO_SWITCH_VAL);
    if (rc < 0)
    {
        pr_err("%s: msm_camera_clk_switch failed!\n", __func__);
        return rc;
    }
#endif
	msm_camio_clk_rate_set(24000000);
	mdelay(10);
	pr_err("--CAMERA--re_init_sensor ok!!\n");
	return rc;
}

static int mt9d115_sensor_release(void)
{
	// int rc = 0;
        MT9D115_CSI_CONFIG = 0;
	mt9d115_power_off();
	//kfree(mt9d115_ctrl);
	//mt9d115_ctrl = NULL;	
	return 0;
}

static const struct i2c_device_id mt9d115_i2c_id[] = {
	{"mt9d115", 0},{}
};

static int mt9d115_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static int mt9d115_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&mt9d115_wait_queue);
	return 0;
}

static long mt9d115_set_effect(int mode, int effect)
{
    uint16_t __attribute__((unused)) reg_addr;
    uint16_t __attribute__((unused)) reg_val;
    long rc = 0;


    switch (effect)
    {
        case CAMERA_EFFECT_OFF:
        {
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0x2759, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x6440, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0x275B, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x6440, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0xA103, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0005, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }          
        }            
        break;

        case CAMERA_EFFECT_MONO:
        {
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0x2759, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x6441, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0x275B, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x6441, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0xA103, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0005, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }          
        }
        break;

        case CAMERA_EFFECT_NEGATIVE:
        {
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0x2759, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x6443, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0x275B, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x6443, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0xA103, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0005, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }          
        }
        break;

        case CAMERA_EFFECT_SOLARIZE:
        {
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0x2759, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x6444, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0x275B, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x6444, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0xA103, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0005, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }          
        }            
        break;

        case CAMERA_EFFECT_SEPIA:
        {
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0x2759, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x6442, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0x275B, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x6442, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0xA103, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0005, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098E, 0xE886, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x00D8, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098E, 0xEC85, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x001F, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }            
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098E, 0xEC86, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x00D8, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }        
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098E, 0x8400, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0006, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }          
        }         
        break;

        default:
        {
            /* add code here
                 e.g.
                 reg_val = 0xXXXX;

                 rc = mt9d115_i2c_write(mt9d115_client->addr, 0xXXXX, reg_addr, WORD_LEN);
                 if (rc < 0)
                 {
                    return rc;
                 }
               */

            return -EFAULT;
        }
    }


    /*
      * Attention
      *
      * Time delay of 100ms or more is required by sensor,
      *
      * Effect config will have no effect after setting 
      * without time delay of 100ms or more
      */
    mdelay(100);

    return rc;
}

static int32_t mt9d115_set_exposure_brightness(int8_t exposure, int8_t brightness)
{
    int32_t rc = 0;

    pr_err("%s: entry: exposure=%d, brightness=%d\n", __func__, exposure, brightness);

    rc = mt9d115_i2c_write_table(mt9d115_regs.brightness_exposure_tbl[exposure*CAMERA_BRIGHTNESS_MAX+brightness],
                                         mt9d115_regs.brightness_exposure_tbl_sz[exposure*CAMERA_BRIGHTNESS_MAX+brightness]); 

    return rc;
}

static int mt9d115_set_brightness(int8_t brightness)
{

    int32_t rc = 0;

    pr_err("%s: entry: brightness=%d\n", __func__, brightness);

    current_brightness = brightness;

    rc = mt9d115_set_exposure_brightness(current_exposure, 
        current_brightness);

	return rc;

}


static int mt9d115_set_contrast(int contrast)
{
    int32_t rc = 0;

    pr_err("%s: entry: contrast=%d\n", __func__, contrast);

    switch (contrast)
    {
        case CAMERA_CONTRAST_0:
        {
            rc = mt9d115_i2c_write_table(mt9d115_regs.contrast_tbl[0],
                                         mt9d115_regs.contrast_tbl_sz[0]);
        }
        break;

        case CAMERA_CONTRAST_1:
        {
            rc = mt9d115_i2c_write_table(mt9d115_regs.contrast_tbl[1],
                                         mt9d115_regs.contrast_tbl_sz[1]);
        }
        break;

        case CAMERA_CONTRAST_2:
        {
            rc = mt9d115_i2c_write_table(mt9d115_regs.contrast_tbl[2],
                                         mt9d115_regs.contrast_tbl_sz[2]);
        }
        break;
        
        case CAMERA_CONTRAST_3:
        {
            rc = mt9d115_i2c_write_table(mt9d115_regs.contrast_tbl[3],
                                         mt9d115_regs.contrast_tbl_sz[3]);
        }
        break; 

        case CAMERA_CONTRAST_4:
        {
            rc = mt9d115_i2c_write_table(mt9d115_regs.contrast_tbl[4],
                                         mt9d115_regs.contrast_tbl_sz[4]);
        }
        break;

        default:
        {
            pr_err("%s: parameter error!\n", __func__);
            rc = -EFAULT;
        }     
    }

    /*
      * Attention
      *
      * Time delay of 100ms or more is required by sensor,
      *
      * Contrast config will have no effect after setting 
      * without time delay of 100ms or more
      */
    mdelay(100);

	return rc;
}



static int mt9d115_set_sharpness(int sharpness)
{
    int32_t rc = 0;

    pr_err("%s: entry: sharpness=%d\n", __func__, sharpness);

    switch (sharpness)
    {
        case CAMERA_SHARPNESS_0:
        {
            rc = mt9d115_i2c_write_table(mt9d115_regs.sharpness_tbl[0],
                                         mt9d115_regs.sharpness_tbl_sz[0]);
        }
        break;

        case CAMERA_SHARPNESS_1:
        {
            rc = mt9d115_i2c_write_table(mt9d115_regs.sharpness_tbl[1],
                                         mt9d115_regs.sharpness_tbl_sz[1]);
        }
        break;

        case CAMERA_SHARPNESS_2:
        {
            rc = mt9d115_i2c_write_table(mt9d115_regs.sharpness_tbl[2],
                                         mt9d115_regs.sharpness_tbl_sz[2]);
        }
        break;
        
        case CAMERA_SHARPNESS_3:
        {
            rc = mt9d115_i2c_write_table(mt9d115_regs.sharpness_tbl[3],
                                         mt9d115_regs.sharpness_tbl_sz[3]);
        }
        break; 

        case CAMERA_SHARPNESS_4:
        {
            rc = mt9d115_i2c_write_table(mt9d115_regs.sharpness_tbl[4],
                                         mt9d115_regs.sharpness_tbl_sz[4]);
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



static int mt9d115_set_iso(int iso_val)
{
    int32_t rc = 0;

    pr_err("%s: entry: iso_val=%d\n", __func__, iso_val);

    switch (iso_val)
    {
        case CAMERA_ISO_SET_AUTO:
        {
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C,  0xA20D, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0020, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0xA20E, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0080, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0xA103, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0006, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            
            mdelay(200);
            
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C,  0xA20D, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0020, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0xA20E, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0080, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0xA103, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0005, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x3032, 0x0100, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x3034, 0x0100, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x3036, 0x0100, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x3038, 0x0100, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
        }
        break;

        case CAMERA_ISO_SET_HJR:
        {
             pr_err("%s: not supported!\n", __func__);
             rc = -EFAULT;
        }
        break;

        case CAMERA_ISO_SET_100:
        {
             rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C,  0xA20D, WORD_LEN);
             if (rc < 0)
             {
                 return rc;
             }
             rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0020, WORD_LEN);
             if (rc < 0)
             {
                 return rc;
             }
             rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0xA20E, WORD_LEN);
             if (rc < 0)
             {
                 return rc;
             }
             rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0060, WORD_LEN);
             if (rc < 0)
             {
                 return rc;
             }
             rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0xA103, WORD_LEN);
             if (rc < 0)
             {
                 return rc;
             }
             rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0005, WORD_LEN);
             if (rc < 0)
             {
                 return rc;
             }    
             rc = mt9d115_i2c_write(mt9d115_client->addr, 0x3032, 0x0100, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x3034, 0x0100, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x3036, 0x0100, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x3038, 0x0100, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
        }
        break;

        case CAMERA_ISO_SET_200:
        {
             rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C,  0xA20D, WORD_LEN);
             if (rc < 0)
             {
                return rc;
             }
             rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0040, WORD_LEN);
             if (rc < 0)
             {
                return rc;
             }
             rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0xA20E, WORD_LEN);
             if (rc < 0)
             {
                return rc;
             }
             rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0070, WORD_LEN);
             if (rc < 0)
             {
                return rc;
             }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0xA103, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0005, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }    
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x3032, 0x0100, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x3034, 0x0100, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x3036, 0x0100, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x3038, 0x0100, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
        }
        break;

        case CAMERA_ISO_SET_400:
        {
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C,  0xA20D, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0050, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0xA20E, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0080, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0xA103, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0005, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }  
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x3032, 0x0100, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x3034, 0x0100, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x3036, 0x0100, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x3038, 0x0100, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
        }
        break;

        case CAMERA_ISO_SET_800:
        {
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C,  0xA20D, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0050, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0xA20E, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0080, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0xA103, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0005, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }  
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x3032, 0x0200, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x3034, 0x0200, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x3036, 0x0200, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x3038, 0x0200, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
        }
        break;

        default:
        {
            pr_err("%s: parameter error!\n", __func__);
            rc = -EFAULT;
        }     
    }

    /*
      * Attention
      *
      * Time delay of 100ms or more is required by sensor,
      *
      * ISO config will have no effect after setting 
      * without time delay of 100ms or more
      */
    mdelay(100);

	return rc;
}



static int mt9d115_set_saturation(int saturation)
{
    int32_t rc = 0;

    pr_err("%s: entry: saturation=%d\n", __func__, saturation);

    switch (saturation)
    {
        case CAMERA_SATURATION_0:
        {
            rc = mt9d115_i2c_write_table(mt9d115_regs.saturation_tbl[0],
                                         mt9d115_regs.saturation_tbl_sz[0]);
        }
        break;

        case CAMERA_SATURATION_1:
        {
            rc = mt9d115_i2c_write_table(mt9d115_regs.saturation_tbl[1],
                                         mt9d115_regs.saturation_tbl_sz[1]);
        }
        break;

        case CAMERA_SATURATION_2:
        {
            rc = mt9d115_i2c_write_table(mt9d115_regs.saturation_tbl[2],
                                         mt9d115_regs.saturation_tbl_sz[2]);
        }
        break;
        
        case CAMERA_SATURATION_3:
        {
            rc = mt9d115_i2c_write_table(mt9d115_regs.saturation_tbl[3],
                                         mt9d115_regs.saturation_tbl_sz[3]);
        }
        break; 

        case CAMERA_SATURATION_4:
        {
            rc = mt9d115_i2c_write_table(mt9d115_regs.saturation_tbl[4],
                                         mt9d115_regs.saturation_tbl_sz[4]);
        }
        break;

        default:
        {
            pr_err("%s: parameter error!\n", __func__);
            rc = -EFAULT;
        }     
    }

    /*
      * Attention
      *
      * Time delay of 100ms or more is required by sensor,
      *
      * Saturation config will have no effect after setting 
      * without time delay of 100ms or more
      */
    mdelay(100);

	return rc;
}


static long mt9d115_set_antibanding(int antibanding)
{
    int32_t rc = 0;

    pr_err("%s: entry: antibanding=%d\n", __func__, antibanding);

    switch (antibanding)
    {
        case CAMERA_ANTIBANDING_SET_OFF:
        {
            pr_err("%s: CAMERA_ANTIBANDING_SET_OFF NOT supported!\n", __func__);
        }
        break;

        case CAMERA_ANTIBANDING_SET_60HZ:
        {
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0xA118, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0002, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0xA11E, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0002, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0xA124, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0002, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0xA12A, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0002, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0xA404, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }  
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x00A0, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0xA103, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0005, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
        }            
        break;
        case CAMERA_ANTIBANDING_SET_50HZ:
        {
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0xA118, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0002, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0xA11E, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0002, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0xA124, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0002, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }    
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0xA12A, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0002, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }    
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0xA404, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x00E0, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0xA103, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0005, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }  
        }
        break;

        case CAMERA_ANTIBANDING_SET_AUTO:
        {
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0xA118, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0001, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0xA11E, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0001, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            } 
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0xA124, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0000, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }    
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0xA12A, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0001, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }  
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x098C, 0xA103, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }
            rc = mt9d115_i2c_write(mt9d115_client->addr, 0x0990, 0x0005, WORD_LEN);
            if (rc < 0)
            {
                return rc;
            }  
        }
        break;

        default:
        {
            pr_err("%s: parameter error!\n", __func__);
            rc = -EFAULT;
        }     
    }


    /*
      * Attention
      *
      * Time delay of 100ms or more is required by sensor,
      *
      * Antibanding config will have no effect after setting 
      * without time delay of 100ms or more
      */
    mdelay(100);

	return rc;
}

static int mt9d115_set_sensor_mode(int mode, int res)
{
	int rc = 0;

	pr_err("--CAMERA-- mt9d115_set_sensor_mode mode = %d, res = %d\n", mode, res);

	switch (mode)
	{
	case SENSOR_PREVIEW_MODE:
		pr_err("--CAMERA-- SENSOR_PREVIEW_MODE\n");
		rc = mt9d115_setting(S_UPDATE_PERIODIC, S_RES_PREVIEW);
		break;

	case SENSOR_SNAPSHOT_MODE:
		pr_err("--CAMERA-- SENSOR_SNAPSHOT_MODE\n");
		rc = mt9d115_setting(S_UPDATE_PERIODIC, S_RES_CAPTURE);
		break;

	case SENSOR_RAW_SNAPSHOT_MODE:
	default:
		pr_err("--CAMERA--mt9d115_set_sensor_mode no support\n");
		rc = -EINVAL;
		break;
	}

	return rc;
}



static int mt9d115_set_wb_oem(uint8_t wb_mode)
{
    int32_t rc = 0;

    pr_err("%s: entry: wb_mode=%d\n", __func__, wb_mode);

    switch (wb_mode)
    {
        case CAMERA_WB_MODE_AWB:
        {
            rc = mt9d115_i2c_write_table(mt9d115_regs.wb_auto_tbl, 
                                         mt9d115_regs.wb_auto_tbl_sz);
        }
        break;

        case CAMERA_WB_MODE_SUNLIGHT:
        {
            rc = mt9d115_i2c_write_table(mt9d115_regs.wb_daylight_tbl,
                                         mt9d115_regs.wb_daylight_tbl_sz);
        }
        break;

        case CAMERA_WB_MODE_INCANDESCENT:
        {
            rc = mt9d115_i2c_write_table(mt9d115_regs.wb_incandescent_tbl,
                                         mt9d115_regs.wb_incandescent_tbl_sz);
        }
        break;
        
        case CAMERA_WB_MODE_FLUORESCENT:
        {
            rc = mt9d115_i2c_write_table(mt9d115_regs.wb_flourescant_tbl,
                                         mt9d115_regs.wb_flourescant_tbl_sz);
        }
        break; 

        case CAMERA_WB_MODE_CLOUDY:
        {
            rc = mt9d115_i2c_write_table(mt9d115_regs.wb_cloudy_tbl,
                                         mt9d115_regs.wb_cloudy_tbl_sz);
        }
        break;

        default:
        {
            pr_err("%s: parameter error!\n", __func__);
            rc = -EFAULT;
        }     
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

	return rc;
}


static int mt9d115_set_exposure_compensation(int exposure)
{
    long rc = 0;

    pr_err("%s: entry: exposure=%d\n", __func__, exposure);

    current_exposure = exposure;

    rc = (int32_t)mt9d115_set_exposure_brightness(current_exposure, 
        current_brightness);

    return rc;
}

static int mt9d115_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cdata;
	long rc = 0;

	if (copy_from_user(&cdata, (void *)argp, sizeof(struct sensor_cfg_data))) 
		return -EFAULT;

	pr_err("--CAMERA-- %s %d\n",__func__,cdata.cfgtype);

	mutex_lock(&mt9d115_mutex);
	switch (cdata.cfgtype)
	{
	case CFG_SET_MODE:   // 0
		rc =mt9d115_set_sensor_mode(cdata.mode, cdata.rs);
		break;
	case CFG_SET_EFFECT: // 1
		pr_err("--CAMERA-- CFG_SET_EFFECT mode=%d, effect = %d !!\n",cdata.mode, cdata.cfg.effect);
		rc = mt9d115_set_effect(cdata.mode, cdata.cfg.effect);
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
		mt9d115_power_off();
		break;
	case CFG_SET_DEFAULT_FOCUS:  // 06
		pr_err("--CAMERA-- CFG_SET_DEFAULT_FOCUS (Not Implement) !!\n");
		break;        
	case CFG_MOVE_FOCUS:     //  07
		pr_err("--CAMERA-- CFG_MOVE_FOCUS (Not Implement) !!\n");
		break;
	case CFG_SET_BRIGHTNESS:     //  12
		pr_err("--CAMERA-- CFG_SET_BRIGHTNESS  !!\n");
		rc = mt9d115_set_brightness(cdata.cfg.brightness);
		break;
	case CFG_SET_CONTRAST:     //  13
		pr_err("--CAMERA-- CFG_SET_CONTRAST  !!\n");
		rc = mt9d115_set_contrast(cdata.cfg.contrast);
		break;  

	case CFG_SET_ANTIBANDING:     //  17
		pr_err("--CAMERA-- CFG_SET_ANTIBANDING antibanding = %d!!\n", cdata.cfg.antibanding);
		rc = mt9d115_set_antibanding(cdata.cfg.antibanding);
		break;
			
	case CFG_SET_SATURATION:     //  30
		pr_err("--CAMERA-- CFG_SET_SATURATION !!\n");
		rc = mt9d115_set_saturation(cdata.cfg.saturation);
		break;

	case CFG_SET_SHARPNESS:     //  31
		pr_err("--CAMERA-- CFG_SET_SHARPNESS !!\n");
		rc = mt9d115_set_sharpness(cdata.cfg.sharpness);
		break;

	case CFG_SET_WB:
		pr_err("--CAMERA-- CFG_SET_WB!!\n");
		mt9d115_set_wb_oem(cdata.cfg.wb_mode);
		rc = 0 ;
		break;
			
	case CFG_SET_EXPOSURE_COMPENSATION:
		pr_err("--CAMERA-- CFG_SET_EXPOSURE_COMPENSATION !\n");
		rc = mt9d115_set_exposure_compensation(cdata.cfg.exp_compensation);
		break;

	case CFG_SET_ISO:
		pr_err("--CAMERA-- CFG_SET_EXPOSURE_COMPENSATION !\n");
		rc = mt9d115_set_iso(cdata.cfg.iso_val);
		break;
		
	default:
		pr_err("--CAMERA-- %s: Command=%d (Not Implement)!!\n",__func__,cdata.cfgtype);
		rc = -EINVAL;
		break;    
	}
	mutex_unlock(&mt9d115_mutex);
	return rc;    
}

static struct i2c_driver mt9d115_i2c_driver = {
	.id_table = mt9d115_i2c_id,
	.probe  = mt9d115_i2c_probe,
	.remove = mt9d115_i2c_remove,
	.driver = {
		.name = "mt9d115",
	},
};

static int mt9d115_probe_init_gpio(const struct msm_camera_sensor_info *data)
{
	int rc = 0;

	pr_err("--CAMERA-- %s Open CAMIO CLK,set to default clk rate!\n",__func__);

	mt9d115_pwdn_gpio = data->sensor_pwd;
	mt9d115_reset_gpio = data->sensor_reset;

	pr_err("--CAMERA-- %s : sensor_pwd_pin=%d, sensor_reset_pin=%d\n",__func__,data->sensor_pwd,data->sensor_reset);

	rc = gpio_request(data->sensor_pwd, "mt9d115");
	pr_err("--CAMERA-- %s : gpio_request=%d, result is %d",__func__,data->sensor_pwd, rc);

	rc |= gpio_request(data->sensor_reset, "mt9d115");
	pr_err("--CAMERA-- %s : gpio_request=%d, result is %d",__func__,data->sensor_reset, rc);

	if (rc < 0)
	{
		gpio_free(data->sensor_pwd);
		gpio_free(data->sensor_reset);
		rc = gpio_request(data->sensor_pwd, "mt9d115");
		rc |= gpio_request(data->sensor_reset, "mt9d115");
	}

	gpio_direction_output(data->sensor_pwd, 1);

	return rc;
}


static void mt9d115_probe_free_gpio(void)
{
	gpio_free(mt9d115_pwdn_gpio);
	gpio_free(mt9d115_reset_gpio);
	mt9d115_pwdn_gpio = 0xFF;
	mt9d115_reset_gpio = 0xFF;
}

static int mt9d115_sensor_probe(const struct msm_camera_sensor_info *info,struct msm_sensor_ctrl *s)
{
	int rc = -ENOTSUPP;
	pr_err("--CAMERA-- %s (Start...)\n",__func__);
	rc = i2c_add_driver(&mt9d115_i2c_driver);
	pr_err("--CAMERA-- i2c_add_driver ret:0x%x,mt9d115_client=0x%x\n",
		rc, (unsigned int)mt9d115_client);
	if ((rc < 0 ) || (mt9d115_client == NULL))
	{
		pr_err("--CAMERA-- i2c_add_driver FAILS!!\n");
		//return rc;
		return -EIO;
	}	

	rc = mt9d115_probe_init_gpio(info);

	mt9d115_power_on();
	msleep(5);
	rc = msm_camera_power_backend(MSM_CAMERA_PWRUP_MODE);
	mdelay(1);
    	if (rc < 0)
    	{
        	pr_err("%s: camera_power_backend failed!\n", __func__);
        	goto probe_fail;
        	//return rc;
    	}

#if defined(CONFIG_MACH_MOONCAKE2) || defined(CONFIG_MACH_ATLAS40)   
     rc = msm_camera_clk_switch(info, MT9D115_GPIO_SWITCH_CTL, MT9D115_GPIO_SWITCH_VAL);
    if (rc < 0)
    {
        pr_err("%s: msm_camera_clk_switch failed!\n", __func__);
        goto probe_fail;
    }
#endif

	msm_camio_clk_rate_set(24000000);// for MIPI test
	msleep(10);

	mt9d115_power_reset();
	mdelay(5);

	rc = mt9d115_probe_readID(info);

    	if (rc < 0)
    	{
        	pr_err("mt9d115_probe_readID failed!\n");
        	goto probe_fail;
        	//return rc;
    	}

	mt9d115_ctrl = kzalloc(sizeof(struct __mt9d115_ctrl), GFP_KERNEL);
	if (!mt9d115_ctrl)
	{
		pr_err("--CAMERA-- kzalloc mt9d115_ctrl error !!\n");
		kfree(mt9d115_ctrl);
		//return rc;
		goto probe_fail;
	}

	rc = mt9d115_setting(S_REG_INIT, S_RES_PREVIEW);
	
	if (rc < 0)
	{
		pr_err("--CAMERA-- %s : mt9d115_setting failed. rc = %d\n",__func__,rc);
		kfree(mt9d115_ctrl);
		goto probe_fail;
	}


	s->s_init = mt9d115_sensor_open_init;
	s->s_release = mt9d115_sensor_release;
	s->s_config  = mt9d115_sensor_config;
	//s->s_AF = mt9d115_sensor_set_af;
	//camera_init_flag = true;

	s->s_camera_type = BACK_CAMERA_2D;
	s->s_mount_angle = info->sensor_platform_info->mount_angle;


	//mt9d115_power_off(); //lijing
	mt9d115_sensor_release();

	pr_err("--CAMERA-- %s (End...)\n",__func__);
	return 0;

probe_fail:
	pr_err("mt9d115_sensor_probe: SENSOR PROBE FAILS!\n");
	i2c_del_driver(&mt9d115_i2c_driver);
	mt9d115_power_off();
	mt9d115_probe_free_gpio();
	return rc;

}

static int mt9d115_i2c_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	pr_err("--CAMERA-- %s ... (Start...)\n",__func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		pr_err("--CAMERA--i2c_check_functionality failed\n");
		return -ENOMEM;
	}

	mt9d115_sensorw = kzalloc(sizeof(struct mt9d115_work), GFP_KERNEL);
	if (!mt9d115_sensorw)
	{
		pr_err("--CAMERA--kzalloc failed\n");
		return -ENOMEM;
	}
	i2c_set_clientdata(client, mt9d115_sensorw);
	mt9d115_init_client(client);
	mt9d115_client = client;

	pr_err("--CAMERA-- %s ... (End...)\n",__func__);
	return 0;
}

static int __mt9d115_probe(struct platform_device *pdev)
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
	return msm_camera_drv_start(pdev, mt9d115_sensor_probe);
#else
    return msm_camera_drv_start(pdev, mt9d115_sensor_probe,0);
#endif
}

static struct platform_driver msm_camera_driver = {
	.probe = __mt9d115_probe,
	.driver = {
		.name = "msm_camera_mt9d115",
		.owner = THIS_MODULE,
	},
};

static int __init mt9d115_init(void)
{
	
	return platform_driver_register(&msm_camera_driver);
}

module_init(mt9d115_init);

MODULE_DESCRIPTION("MT9D115 YUV MIPI sensor driver");
MODULE_LICENSE("GPL v2");
