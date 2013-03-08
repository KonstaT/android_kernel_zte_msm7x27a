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

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <media/msm_camera.h>
#include <mach/camera.h>
#include <mach/gpio.h>
#include "ov7692.h"

/*=============================================================
	SENSOR REGISTER DEFINES
==============================================================*/
#define Q8    0x00000100

/* Omnivision8810 product ID register address */
#define REG_OV7692_MODEL_ID_MSB                       0x0A
#define REG_OV7692_MODEL_ID_LSB                       0x0B

#define OV7692_MODEL_ID                       0x7692
/* Omnivision8810 product ID */

/* Time in milisecs for waiting for the sensor to reset */
#define OV7692_RESET_DELAY_MSECS    66
#define OV7692_DEFAULT_CLOCK_RATE   24000000
/* Registers*/

/* Color bar pattern selection */
#define OV7692_COLOR_BAR_PATTERN_SEL_REG     0x82
/* Color bar enabling control */
#define OV7692_COLOR_BAR_ENABLE_REG           0x601
/* Time in milisecs for waiting for the sensor to reset*/
#define OV7692_RESET_DELAY_MSECS    66

/* GPIO For Sensor Clock Switch */

#define OV7692_GPIO_SWITCH_CTL     49

#define OV7692_GPIO_SWITCH_VAL     1

static int ov7692_pwdn_gpio;

extern int32_t msm_camera_power_frontend(enum msm_camera_pwr_mode_t pwr_mode);
extern int msm_camera_clk_switch(const struct msm_camera_sensor_info *data,
                                        uint32_t gpio_switch,
                                         uint32_t switch_val);

extern void msm_sensorinfo_set_front_sensor_id(uint16_t id);
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

/*============================================================================
							DATA DECLARATIONS
============================================================================*/

static bool OV7692_CSI_CONFIG;
/* 816x612, 24MHz MCLK 96MHz PCLK */
uint32_t OV7692_FULL_SIZE_WIDTH        = 640;
uint32_t OV7692_FULL_SIZE_HEIGHT       = 480;

uint32_t OV7692_QTR_SIZE_WIDTH         = 640;
uint32_t OV7692_QTR_SIZE_HEIGHT        = 480;

uint32_t OV7692_HRZ_FULL_BLK_PIXELS    = 16;
uint32_t OV7692_VER_FULL_BLK_LINES     = 12;
uint32_t OV7692_HRZ_QTR_BLK_PIXELS     = 16;
uint32_t OV7692_VER_QTR_BLK_LINES      = 12;

struct ov7692_work_t {
	struct work_struct work;
};
static struct  ov7692_work_t *ov7692_sensorw;
static struct  i2c_client *ov7692_client;
struct ov7692_ctrl_t {
	const struct  msm_camera_sensor_info *sensordata;
	uint32_t sensormode;
	uint32_t fps_divider;		/* init to 1 * 0x00000400 */
	uint32_t pict_fps_divider;	/* init to 1 * 0x00000400 */
	uint32_t fps;
	int32_t  curr_lens_pos;
	uint32_t curr_step_pos;
	uint32_t my_reg_gain;
	uint32_t my_reg_line_count;
	uint32_t total_lines_per_frame;
	enum ov7692_resolution_t prev_res;
	enum ov7692_resolution_t pict_res;
	enum ov7692_resolution_t curr_res;
	enum ov7692_test_mode_t  set_test;
	unsigned short imgaddr;
};
static struct ov7692_ctrl_t *ov7692_ctrl;
static DECLARE_WAIT_QUEUE_HEAD(ov7692_wait_queue);
DEFINE_MUTEX(ov7692_mut);

/*=============================================================*/

static int ov7692_i2c_rxdata(unsigned short saddr,
	unsigned char *rxdata, int length)
{
	struct i2c_msg msgs[] = {
		{
			.addr  = saddr,
			.flags = 0,
			.len   = 1,
			.buf   = rxdata,
		},
		{
			.addr  = saddr,
			.flags = I2C_M_RD,
			.len   = 1,
			.buf   = rxdata,
		},
	};
	if (i2c_transfer(ov7692_client->adapter, msgs, 2) < 0) {
		pr_err("ov7692_i2c_rxdata failed!\n");
		return -EIO;
	}
	return 0;
}


static int32_t ov7692_i2c_txdata(unsigned short saddr,
				unsigned char *txdata, int length)
{
	struct i2c_msg msg[] = {
		{
			.addr = saddr,
			.flags = 0,
			.len = 2,
			.buf = txdata,
		 },
	};
	if (i2c_transfer(ov7692_client->adapter, msg, 1) < 0) {
		pr_err("ov7692_i2c_txdata faild 0x%x\n", ov7692_client->addr);
		return -EIO;
	}

	return 0;
}

#if  defined(CONFIG_MACH_ATLAS40)||defined(CONFIG_MACH_BLADE2)
static int32_t ov7692_i2c_read(uint8_t raddr,
	uint8_t *rdata, int rlen)
{
	int32_t rc = 0;
	unsigned char buf[1];

    pr_err("atlas40 ov7692_i2c_read\n");
	if (!rdata)
		return -EIO;
	memset(buf, 0, sizeof(buf));
	buf[0] = raddr;
	rc = ov7692_i2c_rxdata((ov7692_client->addr)>>1, buf, rlen);
	if (rc < 0) {
		pr_err("ov7692_i2c_read 0x%x failed!\n", raddr);
		return rc;
	}
	*rdata = buf[0];
	return rc;
}

static int32_t ov7692_i2c_write(unsigned short saddr,
                                      unsigned short waddr,
                                      unsigned short wdata,
                                      enum ov7692_width_t width)
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

            rc = ov7692_i2c_txdata((saddr)>>1, buf, 4);
        }
        break;

        case BYTE_LEN:
        {
            buf[0] = waddr;
            buf[1] = wdata;

            rc = ov7692_i2c_txdata((saddr)>>1, buf, 2);
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
static int32_t ov7692_i2c_read(uint8_t raddr,
	uint8_t *rdata, int rlen)
{
	int32_t rc = 0;
	unsigned char buf[1];

    pr_err("ov7692_i2c_read\n");
	if (!rdata)
		return -EIO;
	memset(buf, 0, sizeof(buf));
	buf[0] = raddr;
	rc = ov7692_i2c_rxdata(ov7692_client->addr, buf, rlen);
	if (rc < 0) {
		pr_err("ov7692_i2c_read 0x%x failed!\n", raddr);
		return rc;
	}
	*rdata = buf[0];
	return rc;
}

static int32_t ov7692_i2c_write(unsigned short saddr,
                                      unsigned short waddr,
                                      unsigned short wdata,
                                      enum ov7692_width_t width)
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

            rc = ov7692_i2c_txdata(saddr, buf, 4);
        }
        break;

        case BYTE_LEN:
        {
            buf[0] = waddr;
            buf[1] = wdata;

            rc = ov7692_i2c_txdata(saddr, buf, 2);
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

static int32_t ov7692_i2c_write_table(struct ov7692_i2c_reg_conf const *reg_conf_tbl,
                                             int len)
{
    uint32_t i;
    int32_t rc = 0;

    for (i = 0; i < len; i++)
    {
        rc = ov7692_i2c_write(ov7692_client->addr,
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

static int32_t ov7692_sensor_setting(int update_type, int rt)
{

	int32_t rc = 0;
	struct msm_camera_csi_params ov7692_csi_params;
	switch (update_type) {
	case REG_INIT:
		OV7692_CSI_CONFIG = 0;
	
		rc = ov7692_i2c_write(ov7692_client->addr, 0x0e, 0x08, BYTE_LEN);
		return rc;
		break;
	case UPDATE_PERIODIC:
		if (!OV7692_CSI_CONFIG) {
			ov7692_csi_params.lane_cnt = 1;
			ov7692_csi_params.data_format = CSI_8BIT;
			ov7692_csi_params.lane_assign = 0xe4;
			ov7692_csi_params.dpcm_scheme = 0;
			ov7692_csi_params.settle_cnt = 0x14;

			rc = msm_camio_csi_config(&ov7692_csi_params);
			msleep(10);
			rc = ov7692_i2c_write_table(ov7692_regs.prev_snap_reg_settings, ov7692_regs.prev_snap_reg_settings_size);

			OV7692_CSI_CONFIG = 1;
			msleep(20);
			//return rc;
		}


		if (S_RES_PREVIEW == rt) {
			rc = ov7692_i2c_write(ov7692_client->addr,0x0c,0x80,BYTE_LEN);

		} else if (S_RES_CAPTURE == rt) {
			rc = ov7692_i2c_write(ov7692_client->addr,0x0C,0x00,BYTE_LEN);
		}
		break;
	default:
		rc = -EINVAL;
		break;
	}
	return rc;
}

static int32_t ov7692_video_config(int mode)
{
	int32_t rc = 0;
	int rt;
	/* change sensor resolution if needed */
	if(mode == SENSOR_SNAPSHOT_MODE) {
		rt = S_RES_CAPTURE;
		}
	else	{
		rt = S_RES_PREVIEW;
		}

	if (ov7692_sensor_setting(UPDATE_PERIODIC, rt) < 0)
		return rc;
	ov7692_ctrl->curr_res = ov7692_ctrl->prev_res;
	ov7692_ctrl->sensormode = mode;
	return rc;
}

static int32_t ov7692_set_sensor_mode(int mode,
	int res)
{
	int32_t rc = 0;
	switch (mode) {
	case SENSOR_PREVIEW_MODE:
	case SENSOR_SNAPSHOT_MODE:
		rc = ov7692_video_config(mode);
		break;
	case SENSOR_RAW_SNAPSHOT_MODE:
		break;
	default:
		rc = -EINVAL;
		break;
	}
	return rc;
}

static void ov7692_power_on(void)
{
	pr_err("--CAMERA-- %s ... (Start...)\n",__func__);

	gpio_set_value(ov7692_pwdn_gpio, 0);

	//afinit = 1;

	pr_err("--CAMERA-- %s ... (End...)\n",__func__);
}
static int32_t ov7692_power_down(void)
{
	pr_err("--CAMERA-- %s ... (Start...)\n",__func__);

	gpio_set_value(ov7692_pwdn_gpio, 1);
	return 0;
}

static int ov7692_probe_init_sensor(const struct msm_camera_sensor_info *data)
{
	uint8_t model_id_msb, model_id_lsb = 0;
	uint16_t model_id;
	int32_t rc = 0;
	/*The reset pin is not physically connected to the sensor.
	The standby pin will do the reset hence there is no need
	to request the gpio reset*/

	/* Read sensor Model ID: */
	rc = ov7692_i2c_read(REG_OV7692_MODEL_ID_MSB, &model_id_msb, 1);
	if (rc < 0)
		goto init_probe_fail;
	rc = ov7692_i2c_read(REG_OV7692_MODEL_ID_LSB, &model_id_lsb, 1);
	if (rc < 0)
		goto init_probe_fail;
	model_id = (model_id_msb << 8) | ((model_id_lsb & 0x00FF)) ;
	pr_err("ov7692 model_id = 0x%x, 0x%x, 0x%x\n",
		 model_id, model_id_msb, model_id_lsb);
	/* 4. Compare sensor ID to OV7692 ID: */
	if (model_id != OV7692_MODEL_ID) {
		rc = -ENODEV;
		goto init_probe_fail;
	}
#ifdef CONFIG_SENSOR_INFO
	msm_sensorinfo_set_front_sensor_id(model_id);
#endif

	goto init_probe_done;
init_probe_fail:
	pr_warning(" ov7692_probe_init_sensor fails\n");
init_probe_done:
	pr_err(" ov7692_probe_init_sensor finishes\n");
	return rc;
}

int ov7692_sensor_open_init(const struct msm_camera_sensor_info *data)
{
	int32_t rc = 0;

	pr_err("%s: %d\n", __func__, __LINE__);
	pr_err("Calling ov7692_sensor_open_init\n");
	ov7692_ctrl = kzalloc(sizeof(struct ov7692_ctrl_t), GFP_KERNEL);
	if (!ov7692_ctrl) {
		pr_err("ov7692_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}
	ov7692_ctrl->fps_divider = 1 * 0x00000400;
	ov7692_ctrl->pict_fps_divider = 1 * 0x00000400;
	ov7692_ctrl->fps = 30 * Q8;
	ov7692_ctrl->set_test = TEST_OFF;
	ov7692_ctrl->prev_res = QTR_SIZE;
	ov7692_ctrl->pict_res = FULL_SIZE;
	ov7692_ctrl->curr_res = INVALID_SIZE;

	if (data)
		ov7692_ctrl->sensordata = data;

#if defined(CONFIG_MACH_BLADE2) || defined(CONFIG_MACH_ATLAS40)	|| defined(CONFIG_MACH_MOONCAKE2)
	/* enable mclk first */
	rc = msm_camera_clk_switch(data, OV7692_GPIO_SWITCH_CTL, OV7692_GPIO_SWITCH_VAL);
	if (rc < 0) {
		pr_err("msm_camera_clk_switch fail\n");
		goto init_fail;
	}
	msm_camio_clk_rate_set(24000000);
	msleep(20);
#endif 
#if 0
	rc = ov7692_probe_init_sensor(data);
	if (rc < 0) {
		pr_err("Calling ov7692_sensor_open_init fail\n");
		goto init_fail;
	}
#endif

	ov7692_power_on();
/*ZTE_CAM_YGL_20111122,turn on mipi for high current when sleep*/
	rc = ov7692_i2c_write(ov7692_client->addr, 0xFF, 0x01, BYTE_LEN);
        if (rc < 0)
        {
            return rc;
        }
	rc = ov7692_i2c_write(ov7692_client->addr, 0xb4, 0xC0, BYTE_LEN);
        if (rc < 0)
        {
            return rc;
        }
	rc = ov7692_i2c_write(ov7692_client->addr, 0xb5, 0x40, BYTE_LEN);
        if (rc < 0)
        {
            return rc;
        }
	rc = ov7692_i2c_write(ov7692_client->addr, 0xFF, 0x00, BYTE_LEN);
        if (rc < 0)
        {
            return rc;
        }

	rc = ov7692_sensor_setting(REG_INIT, RES_PREVIEW);
	if (rc < 0)
		goto init_fail;
	else
		goto init_done;

init_fail:
	pr_err(" ov7692_sensor_open_init fail\n");
	kfree(ov7692_ctrl);
init_done:
	pr_err("ov7692_sensor_open_init done\n");
	return rc;
}

static int ov7692_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&ov7692_wait_queue);
	return 0;
}

static const struct i2c_device_id ov7692_i2c_id[] = {
	{"ov7692", 0},
	{ }
};

static int ov7692_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	pr_err("ov7692_i2c_probe called!\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("i2c_check_functionality failed\n");
		goto probe_failure;
	}

	ov7692_sensorw = kzalloc(sizeof(struct ov7692_work_t), GFP_KERNEL);
	if (!ov7692_sensorw) {
		pr_err("kzalloc failed.\n");
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, ov7692_sensorw);
	ov7692_init_client(client);
	ov7692_client = client;

	pr_err("ov7692_i2c_probe success! rc = %d\n", rc);
	return 0;

probe_failure:
	pr_err("ov7692_i2c_probe failed! rc = %d\n", rc);
	return rc;
}

static int __exit ov7692_remove(struct i2c_client *client)
{
	struct ov7692_work_t_t *sensorw = i2c_get_clientdata(client);
	free_irq(client->irq, sensorw);
	ov7692_client = NULL;
	kfree(sensorw);
	return 0;
}

static struct i2c_driver ov7692_i2c_driver = {
	.id_table = ov7692_i2c_id,
	.probe  = ov7692_i2c_probe,
	.remove = __exit_p(ov7692_i2c_remove),
	.driver = {
		.name = "ov7692",
	},
};

static long ov7692_set_effect(int mode, int effect)
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
				
			rc = ov7692_i2c_write(ov7692_client->addr, 0x38, 0x50, BYTE_LEN);
			mdelay(70); 			
			rc = ov7692_i2c_write_table(ov7692_regs.effect_off_tbl, ov7692_regs.effect_off_tbl_sz);
			                  
			rc = ov7692_i2c_write(ov7692_client->addr, 0x38, 0x10, BYTE_LEN);
			
			break;
		}
	case CAMERA_EFFECT_MONO:
		{
			pr_err("--CAMERA-- %s ...CAMERA_EFFECT_MONO\n",__func__);

			rc = ov7692_i2c_write(ov7692_client->addr, 0x38, 0x50, BYTE_LEN);
			mdelay(70); 
				
			rc = ov7692_i2c_write_table(ov7692_regs.effect_mono_tbl, ov7692_regs.effect_mono_tbl_sz);
			
			rc = ov7692_i2c_write(ov7692_client->addr, 0x38, 0x10, BYTE_LEN);
			
			break;
		}
	case CAMERA_EFFECT_NEGATIVE:
		{
			pr_err("--CAMERA-- %s ...CAMERA_EFFECT_NEGATIVE\n",__func__);

			rc = ov7692_i2c_write(ov7692_client->addr, 0x38, 0x50, BYTE_LEN);
			mdelay(70); 
			
			rc = ov7692_i2c_write_table(ov7692_regs.effect_negative_tbl, ov7692_regs.effect_negative_tbl_sz);
			
			rc = ov7692_i2c_write(ov7692_client->addr, 0x38, 0x10, BYTE_LEN);
			
			break;
		}
	case CAMERA_EFFECT_SEPIA:
		{
			pr_err("--CAMERA-- %s ...CAMERA_EFFECT_SEPIA\n",__func__);

			rc = ov7692_i2c_write(ov7692_client->addr, 0x38, 0x50, BYTE_LEN);
			mdelay(70); 
			
			rc = ov7692_i2c_write_table(ov7692_regs.effect_sepia_tbl, ov7692_regs.effect_sepia_tbl_sz);
			
			rc = ov7692_i2c_write(ov7692_client->addr, 0x38, 0x10, BYTE_LEN);
			
			break;
		}

	default:
		{
			pr_err("--CAMERA-- %s ...Default(Not Support)\n",__func__);
		}
	}
	//ov7692_effect = effect;
	//Refresh Sequencer /
	pr_err("--CAMERA-- %s ...(End)\n",__func__);
#endif
    mdelay(20); //ZTE_CAM_LJ_20111116 merge delay time from ov7690
	return rc;
}

static int ov7692_set_brightness(int8_t brightness)
{
	int rc = 0;
	pr_err("--CAMERA-- %s ...(Start)\n",__func__);
	pr_err("--CAMERA-- %s ...brightness = %d\n",__func__ , brightness);
#if 1
	switch (brightness)
	{
	case CAMERA_BRIGHTNESS_0:
		pr_err("--CAMERA--CAMERA_BRIGHTNESS_0\n");
		
		rc = ov7692_i2c_write_table(ov7692_regs.brightness_tbl[0],
                                         ov7692_regs.brightness_tbl_sz[0]);
		break;

	case CAMERA_BRIGHTNESS_1:
		pr_err("--CAMERA--CAMERA_BRIGHTNESS_1\n");

		rc = ov7692_i2c_write_table(ov7692_regs.brightness_tbl[1],
                                         ov7692_regs.brightness_tbl_sz[1]);
		break;

	case CAMERA_BRIGHTNESS_2:
		pr_err("--CAMERA--CAMERA_BRIGHTNESS_2\n");
							
		rc = ov7692_i2c_write_table(ov7692_regs.brightness_tbl[2],
                                         ov7692_regs.brightness_tbl_sz[2]);
		break;

	case CAMERA_BRIGHTNESS_3:
		pr_err("--CAMERA--CAMERA_BRIGHTNESS_3\n");
		
		rc = ov7692_i2c_write_table(ov7692_regs.brightness_tbl[3],
                                         ov7692_regs.brightness_tbl_sz[3]);
		break;

	case CAMERA_BRIGHTNESS_4:
		pr_err("--CAMERA--CAMERA_BRIGHTNESS_4\n");
		
		rc = ov7692_i2c_write_table(ov7692_regs.brightness_tbl[4],
                                         ov7692_regs.brightness_tbl_sz[4]);
		break;

	case CAMERA_BRIGHTNESS_5:
		pr_err("--CAMERA--CAMERA_BRIGHTNESS_LV5\n");
		
		rc = ov7692_i2c_write_table(ov7692_regs.brightness_tbl[5],
                                         ov7692_regs.brightness_tbl_sz[5]);
		break;

	case CAMERA_BRIGHTNESS_6:
		pr_err("--CAMERA--CAMERA_BRIGHTNESS_6\n");
		
		rc = ov7692_i2c_write_table(ov7692_regs.brightness_tbl[6],
                                         ov7692_regs.brightness_tbl_sz[6]);
		break;

	default:
		pr_err("--CAMERA--CAMERA_BRIGHTNESS_ERROR COMMAND\n");
		break;
	}
#endif
	pr_err("--CAMERA-- %s ...(End)\n",__func__);
	return rc;
}

#if 1
static int ov7692_set_contrast(int contrast)
{
	int rc = 0;
	uint8_t temp=0;
	pr_err("--CAMERA-- %s ...(Start)\n",__func__);
	pr_err("--CAMERA-- %s ...contrast = %d\n",__func__ , contrast);
#if 1
	//if (effect_value == CAMERA_EFFECT_OFF)
	{
		switch (contrast)
		{
		rc = ov7692_i2c_write(ov7692_client->addr, 0xd2, 0x07, BYTE_LEN);
              if (rc < 0)
              {
                  return rc;
              }  

		case CAMERA_CONTRAST_0:
			pr_err("--CAMERA--CAMERA_CONTRAST_LV0\n");
			
			rc = ov7692_i2c_write_table(ov7692_regs.contrast_tbl[0],
                                         ov7692_regs.contrast_tbl_sz[0]);
			temp=0;
		        rc = ov7692_i2c_read(0xdc, &temp, 1);
		        if (rc < 0)
			    {
			        return rc;
			    }
			 temp |= 0x04;	    
	              	rc = ov7692_i2c_write(ov7692_client->addr, 0xdc, temp, BYTE_LEN);
			
		        if (rc < 0)
		            {
		                return rc;
		            }
			break;
		case CAMERA_CONTRAST_1:
			pr_err("--CAMERA--CAMERA_CONTRAST_LV1\n");
			
			rc = ov7692_i2c_write_table(ov7692_regs.contrast_tbl[1],
                                         ov7692_regs.contrast_tbl_sz[1]);
			temp=0;
		        rc = ov7692_i2c_read(0xdc, &temp, 1);
		        if (rc < 0)
			    {
			        return rc;
			    }
			 temp |= 0x04;	    
	          
			rc = ov7692_i2c_write(ov7692_client->addr, 0xdc, temp, BYTE_LEN);
		        if (rc < 0)
		            {
		                return rc;
		            }
			break;
		case CAMERA_CONTRAST_2:
			pr_err("--CAMERA--CAMERA_CONTRAST_LV2\n");
			
			rc = ov7692_i2c_write_table(ov7692_regs.contrast_tbl[2],
                                         ov7692_regs.contrast_tbl_sz[2]);
			temp=0;
		        rc = ov7692_i2c_read(0xdc, &temp, 1);
		        if (rc < 0)
			    {
			        return rc;
			    }
			temp &= 0xfb; 
	       
			rc = ov7692_i2c_write(ov7692_client->addr, 0xdc, temp, BYTE_LEN);
		        if (rc < 0)
		            {
		                return rc;
		            }
			break;
		case CAMERA_CONTRAST_3:
			pr_err("--CAMERA--CAMERA_CONTRAST_LV3\n");
			
			rc = ov7692_i2c_write_table(ov7692_regs.contrast_tbl[3],
                                         ov7692_regs.contrast_tbl_sz[3]);
			temp=0;
		        rc = ov7692_i2c_read(0xdc, &temp, 1);
		        if (rc < 0)
			    {
			        return rc;
			    }
			temp &= 0xfb; 
	             
			rc = ov7692_i2c_write(ov7692_client->addr, 0xdc, temp, BYTE_LEN);
		        if (rc < 0)
		            {
		                return rc;
		            }
			break;
		case CAMERA_CONTRAST_4:
			pr_err("--CAMERA--CAMERA_CONTRAST_LV4\n");
			
			rc = ov7692_i2c_write_table(ov7692_regs.contrast_tbl[4],
                                         ov7692_regs.contrast_tbl_sz[4]);
			temp=0;
		        rc = ov7692_i2c_read(0xdc, &temp, 1);
		        if (rc < 0)
			    {
			        return rc;
			    }
			temp &= 0xfb; 
	               
		 	rc = ov7692_i2c_write(ov7692_client->addr, 0xdc, temp, BYTE_LEN);
		        if (rc < 0)
		            {
		                return rc;
		            }
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
    //mdelay(100);
	#endif

	return rc;
}
#endif



static int ov7692_set_sharpness(int sharpness)
{
	int rc = 0;
	pr_err("--CAMERA-- %s ...(Start)\n",__func__);
	pr_err("--CAMERA-- %s ...sharpness = %d\n",__func__ , sharpness);

	switch(sharpness)
		{
		case CAMERA_SHARPNESS_0:
			pr_err("--CAMERA--CAMERA_SHARPNESS_0\n");
			
			rc = ov7692_i2c_write_table(ov7692_regs.sharpness_tbl[0],
                                         ov7692_regs.sharpness_tbl_sz[0]);
			break;
		case CAMERA_SHARPNESS_1:
			pr_err("--CAMERA--CAMERA_SHARPNESS_1\n");
			
			rc = ov7692_i2c_write_table(ov7692_regs.sharpness_tbl[1],
                                         ov7692_regs.sharpness_tbl_sz[1]);

			break;
		case CAMERA_SHARPNESS_2:
			pr_err("--CAMERA--CAMERA_SHARPNESS_2\n");
		
			rc = ov7692_i2c_write_table(ov7692_regs.sharpness_tbl[2],
                                         ov7692_regs.sharpness_tbl_sz[2]);

			break;
		case CAMERA_SHARPNESS_3:
			pr_err("--CAMERA--CAMERA_SHARPNESS_3\n");
		
			rc = ov7692_i2c_write_table(ov7692_regs.sharpness_tbl[3],
                                         ov7692_regs.sharpness_tbl_sz[3]);

			break;
		case CAMERA_SHARPNESS_4:
			pr_err("--CAMERA--CAMERA_SHARPNESS_4\n");
			
			rc = ov7692_i2c_write_table(ov7692_regs.sharpness_tbl[4],
                                         ov7692_regs.sharpness_tbl_sz[4]);

			break;
		
		default:
			pr_err("--CAMERA--CAMERA_SHARPNESS_ERROR COMMAND\n");
			break;
		}
	
	pr_err("--CAMERA-- %s ...(End)\n",__func__);
	return rc;
}

#if 1
static int ov7692_set_wb_oem(uint8_t wb_mode)
{
	int rc = 0;

	pr_err("[kylin] %s \r\n",__func__ );
	pr_err("--CAMERA-- %s ...wb_mode = %d\n", __func__, wb_mode);

	switch(wb_mode)
	{
	case CAMERA_WB_MODE_AWB:
		pr_err("--CAMERA--CAMERA_WB_MODE_AWB\n");
				
		rc = ov7692_i2c_write_table(ov7692_regs.wb_auto_tbl,
                                         ov7692_regs.wb_auto_tbl_sz);
		break;

	case CAMERA_WB_MODE_SUNLIGHT:
		pr_err("--CAMERA--CAMERA_WB_MODE_SUNLIGHT\n");
		 
		rc = ov7692_i2c_write_table(ov7692_regs.wb_daylight_tbl,
                                         ov7692_regs.wb_daylight_tbl_sz);
		break;
	case CAMERA_WB_MODE_INCANDESCENT:
		pr_err("--CAMERA--CAMERA_WB_MODE_INCANDESCENT\n");

		rc = ov7692_i2c_write_table(ov7692_regs.wb_incandescent_tbl,
                                         ov7692_regs.wb_incandescent_tbl_sz);
		break;
	case CAMERA_WB_MODE_FLUORESCENT:
		pr_err("--CAMERA--CAMERA_WB_MODE_FLUORESCENT\n");

		rc = ov7692_i2c_write_table(ov7692_regs.wb_flourescant_tbl,
                                         ov7692_regs.wb_flourescant_tbl_sz);
		break;
	case CAMERA_WB_MODE_CLOUDY:
		pr_err("--CAMERA--CAMERA_WB_MODE_CLOUDY\n");

		rc = ov7692_i2c_write_table(ov7692_regs.wb_cloudy_tbl,
                                         ov7692_regs.wb_cloudy_tbl_sz);
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

	return rc;
}
#endif


#if 1
static int ov7692_set_exposure_compensation(int compensation)
{
	long rc = 0;


	pr_err("--CAMERA-- %s ...(Start)\n",__func__);

	pr_err("--CAMERA-- %s ...exposure_compensation = %d\n",__func__ , compensation);

	switch(compensation)
	{
case CAMERA_EXPOSURE_0:
	pr_err("--CAMERA--CAMERA_EXPOSURE_COMPENSATION_0\n");
	
	rc = ov7692_i2c_write_table(ov7692_regs.exposure_compensation_tbl[0],
                                         ov7692_regs.exposure_compensation_tbl_sz[0]);
	break;
case CAMERA_EXPOSURE_1:
	pr_err("--CAMERA--CAMERA_EXPOSURE_COMPENSATION_1\n");
	 
	rc = ov7692_i2c_write_table(ov7692_regs.exposure_compensation_tbl[1],
                                         ov7692_regs.exposure_compensation_tbl_sz[1]);
	break;
case CAMERA_EXPOSURE_2:
	pr_err("--CAMERA--CAMERA_EXPOSURE_COMPENSATION_2\n");
	
	rc = ov7692_i2c_write_table(ov7692_regs.exposure_compensation_tbl[2],
                                         ov7692_regs.exposure_compensation_tbl_sz[2]);
	break;
case CAMERA_EXPOSURE_3:
	pr_err("--CAMERA--CAMERA_EXPOSURE_COMPENSATION_3\n");
	
	rc = ov7692_i2c_write_table(ov7692_regs.exposure_compensation_tbl[3],
                                         ov7692_regs.exposure_compensation_tbl_sz[3]);
	break;
case CAMERA_EXPOSURE_4:
	pr_err("--CAMERA--CAMERA_EXPOSURE_COMPENSATION_4\n");
	 
	rc = ov7692_i2c_write_table(ov7692_regs.exposure_compensation_tbl[4],
                                         ov7692_regs.exposure_compensation_tbl_sz[4]);
	break;
default:
	pr_err("--CAMERA--ERROR CAMERA_EXPOSURE_COMPENSATION\n");
	break;
	}

	pr_err("--CAMERA-- %s ...(End)\n",__func__);
	    //mdelay(100);


	return rc;
}
#endif


static int ov7692_set_iso(int saturation)
{
	long rc = 0;
	//int i = 0;
	pr_err("--CAMERA-- %s ...(Start)\n",__func__);
	pr_err("--CAMERA-- %s ...iso = %d\n",__func__ , saturation);

	//if (effect_value == CAMERA_EFFECT_OFF)
	{ 
		switch (saturation)
		{
		case CAMERA_ISO_SET_AUTO:
			pr_err("--CAMERA--CAMERA_ISO_SET_AUTO\n");
			//rc = OV7692Core_WritePREG(ov7692_iso_auto_tbl);
	    
			rc = ov7692_i2c_write(ov7692_client->addr, 0x14, 0x21, BYTE_LEN);
	            if (rc < 0)
	            {
	                return rc;
	            }

			break;
		case CAMERA_ISO_SET_100:
			pr_err("--CAMERA--CAMERA_ISO_SET_100\n");
			//rc = OV7692Core_WritePREG(ov7692_iso_100_tbl);
			rc = ov7692_i2c_write(ov7692_client->addr, 0x14, 0x01, BYTE_LEN);
		            if (rc < 0)
		            {
		                return rc;
		            }
			break;
		case CAMERA_ISO_SET_200:
			pr_err("--CAMERA--CAMERA_ISO_SET_200\n");
			//rc = OV7692Core_WritePREG(ov7692_iso_200_tbl);
		
			rc = ov7692_i2c_write(ov7692_client->addr, 0x14, 0x11, BYTE_LEN);
		            if (rc < 0)
		            {
		                return rc;
		            }
			break;
		case CAMERA_ISO_SET_400:
			pr_err("--CAMERA--CAMERA_ISO_SET_400\n");
			//rc = OV7692Core_WritePREG(ov7692_iso_400_tbl);
		
			rc = ov7692_i2c_write(ov7692_client->addr, 0x14, 0x21, BYTE_LEN);
		            if (rc < 0)
		            {
		                return rc;
		            }
			break;
		case CAMERA_ISO_SET_800:
			pr_err("--CAMERA--CAMERA_ISO_SET_800\n");
			//rc = OV7692Core_WritePREG(ov7692_iso_800_tbl);
			rc = ov7692_i2c_write(ov7692_client->addr, 0x14, 0x31, BYTE_LEN);
		            if (rc < 0)
		            {
		                return rc;
		            }
			break;
		
		default:
			pr_err("--CAMERA--CAMERA_SATURATION_ERROR COMMAND\n");
			break;
		}
	}	

	pr_err("--CAMERA-- %s ...(End)\n",__func__);

	return rc;
}



static int ov7692_set_saturation(int saturation)
{
	long rc = 0;
	//int i = 0;
	pr_err("--CAMERA-- %s ...(Start)\n",__func__);
	pr_err("--CAMERA-- %s ...saturation = %d\n",__func__ , saturation);
    msleep(100); //ZTE_CAM_LJ_20111116 add this delay for cts 

	//if (effect_value == CAMERA_EFFECT_OFF)
	{ 
	 
	     rc = ov7692_i2c_write(ov7692_client->addr, 0xd2, 0x07, BYTE_LEN);
            if (rc < 0)
            {
                return rc;
            }
		switch (saturation)
		{
		case CAMERA_SATURATION_0:
			pr_err("--CAMERA--CAMERA_SATURATION_0\n");
			
			rc = ov7692_i2c_write_table(ov7692_regs.saturation_tbl[0],
                                         ov7692_regs.saturation_tbl_sz[0]);
			break;
		case CAMERA_SATURATION_1:
			pr_err("--CAMERA--CAMERA_SATURATION_1\n");
			
			rc = ov7692_i2c_write_table(ov7692_regs.saturation_tbl[1],
                                         ov7692_regs.saturation_tbl_sz[1]);
			break;
		case CAMERA_SATURATION_2:
			pr_err("--CAMERA--CAMERA_SATURATION_2\n");
			
			rc = ov7692_i2c_write_table(ov7692_regs.saturation_tbl[2],
                                         ov7692_regs.saturation_tbl_sz[2]);
			break;
		case CAMERA_SATURATION_3:
			pr_err("--CAMERA--CAMERA_SATURATION_3\n");
		
			rc = ov7692_i2c_write_table(ov7692_regs.saturation_tbl[3],
                                         ov7692_regs.saturation_tbl_sz[3]);
			break;
		case CAMERA_SATURATION_4:
			pr_err("--CAMERA--CAMERA_SATURATION_4\n");
			
			rc = ov7692_i2c_write_table(ov7692_regs.saturation_tbl[4],
                                         ov7692_regs.saturation_tbl_sz[4]);
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
    //mdelay(100);
    


	return rc;
}


static long ov7692_set_antibanding(int antibanding)
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
		
		rc = ov7692_i2c_write_table(ov7692_regs.antibanding_60z_tbl,
                                         ov7692_regs.antibanding_60z_tbl_sz);
		break;

	case CAMERA_ANTIBANDING_SET_50HZ:
		pr_err("--CAMERA--CAMERA_ANTIBANDING_50HZ\n");
		
		rc = ov7692_i2c_write_table(ov7692_regs.antibanding_50z_tbl,
                                         ov7692_regs.antibanding_50z_tbl_sz);
		break;

	case CAMERA_ANTIBANDING_SET_AUTO:
		pr_err("--CAMERA--CAMERA_ANTIBANDING_AUTO CAN NOT SUPPORT\n");
		break;

	default:
		pr_err("--CAMERA--CAMERA_ANTIBANDING_ERROR COMMAND\n");
		break;
	}
	#endif
	pr_err("--CAMERA-- %s ...(End)\n",__func__);
	return rc;
}

int ov7692_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cdata;
	long   rc = 0;
	if (copy_from_user(&cdata,
		(void *)argp,
		sizeof(struct sensor_cfg_data)))
		return -EFAULT;
	mutex_lock(&ov7692_mut);
	pr_err("ov7692_sensor_config: cfgtype = %d\n",
	cdata.cfgtype);
	switch (cdata.cfgtype) {
	#if 0
	case CFG_SET_MODE:
		rc = ov7692_set_sensor_mode(cdata.mode,
			cdata.rs);
		break;
	case CFG_PWR_DOWN:
		rc = ov7692_power_down();
		break;
	#endif
	
	case CFG_SET_MODE:   // 0
		rc =ov7692_set_sensor_mode(cdata.mode, cdata.rs);
		break;
	case CFG_SET_EFFECT: // 1
		pr_err("--CAMERA-- CFG_SET_EFFECT mode=%d, effect = %d !!\n",cdata.mode, cdata.cfg.effect);
		rc = ov7692_set_effect(cdata.mode, cdata.cfg.effect);
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
		//ov7692_power_off();
		rc = ov7692_power_down();
		break;
	case CFG_SET_DEFAULT_FOCUS:  // 06
		pr_err("--CAMERA-- CFG_SET_DEFAULT_FOCUS (Not Implement) !!\n");
		break;        
	case CFG_MOVE_FOCUS:     //  07
		pr_err("--CAMERA-- CFG_MOVE_FOCUS (Not Implement) !!\n");
		break;
	case CFG_SET_BRIGHTNESS:     //  12
		pr_err("--CAMERA-- CFG_SET_BRIGHTNESS  !!\n");
		rc = ov7692_set_brightness(cdata.cfg.brightness);
		break;
	case CFG_SET_CONTRAST:     //  13
		pr_err("--CAMERA-- CFG_SET_CONTRAST  !!\n");
		rc = ov7692_set_contrast(cdata.cfg.contrast);
		break;  

	case CFG_SET_ANTIBANDING:     //  17
		pr_err("--CAMERA-- CFG_SET_ANTIBANDING antibanding = %d!!\n", cdata.cfg.antibanding);
		rc = ov7692_set_antibanding(cdata.cfg.antibanding);
		break;
			
	case CFG_SET_SATURATION:     //  30
		pr_err("--CAMERA-- CFG_SET_SATURATION !!\n");
		rc = ov7692_set_saturation(cdata.cfg.saturation);
		break;

	case CFG_SET_SHARPNESS:     //  31
		pr_err("--CAMERA-- CFG_SET_SHARPNESS !!\n");
		rc = ov7692_set_sharpness(cdata.cfg.sharpness);
		break;

	case CFG_SET_WB:
		pr_err("--CAMERA-- CFG_SET_WB!!\n");
		rc = ov7692_set_wb_oem(cdata.cfg.wb_mode);
		break;
#if 0	
	case CFG_SET_AF:
		pr_err("--CAMERA-- CFG_SET_AUTO_FOCUS !\n");
		rc = ov7692_af_trigger();
		break;
#endif			
	case CFG_SET_EXPOSURE_COMPENSATION:
		pr_err("--CAMERA-- CFG_SET_EXPOSURE_COMPENSATION !\n");
		rc = ov7692_set_exposure_compensation(cdata.cfg.exp_compensation);
		break;

	case CFG_SET_ISO:
		pr_err("--CAMERA-- CFG_SET_ISO !\n");
		rc = ov7692_set_iso(cdata.cfg.iso_val);
		break;


	default:
		rc = -EFAULT;
		break;
	}

	mutex_unlock(&ov7692_mut);

	return rc;
}
static int ov7692_sensor_release(void)
{
	int rc = -EBADF;
	mutex_lock(&ov7692_mut);
/*ZTE_CAM_YGL_20111122,turn off mipi for high current when sleep*/
	rc = ov7692_i2c_write(ov7692_client->addr, 0xFF, 0x01, BYTE_LEN);
        if (rc < 0)
        {
            pr_err("ov7692_release i2c write failed\n");
            return rc;
        }
	rc = ov7692_i2c_write(ov7692_client->addr, 0xb4, 0x40, BYTE_LEN);
        if (rc < 0)
        {
            pr_err("ov7692_release i2c write failed\n");
            return rc;
        }
	rc = ov7692_i2c_write(ov7692_client->addr, 0xb5, 0x30, BYTE_LEN);
        if (rc < 0)
        {
            pr_err("ov7692_release i2c write failed\n");
            return rc;
        }
	rc = ov7692_i2c_write(ov7692_client->addr, 0xFF, 0x00, BYTE_LEN);
        if (rc < 0)
        {
            pr_err("ov7692_release i2c write failed\n");
            return rc;
        }
	ov7692_power_down();
	kfree(ov7692_ctrl);
	ov7692_ctrl = NULL;
	pr_err("ov7692_release completed\n");
	mutex_unlock(&ov7692_mut);

	return rc;
}
static int ov7692_probe_init_gpio(const struct msm_camera_sensor_info *data)
{
	int rc = 0;

	pr_err("--CAMERA-- %s Open CAMIO CLK,set to default clk rate!\n",__func__);

	ov7692_pwdn_gpio = data->sensor_pwd;


	pr_err("--CAMERA-- %s : sensor_pwd_pin=%d\n",__func__,data->sensor_pwd);

	rc = gpio_request(data->sensor_pwd, "ov7692");
	pr_err("--CAMERA-- %s : gpio_request=%d, result is %d",__func__,data->sensor_pwd, rc);



	if (rc < 0)
	{
		gpio_free(data->sensor_pwd);

		rc = gpio_request(data->sensor_pwd, "ov7692");

	}


	gpio_direction_output(data->sensor_pwd, 1);

	return rc;
}
static int ov7692_sensor_probe(const struct msm_camera_sensor_info *info,
		struct msm_sensor_ctrl *s)
{
	int rc = 0;
	rc = i2c_add_driver(&ov7692_i2c_driver);
	if (rc < 0 || ov7692_client == NULL) {
		rc = -ENOTSUPP;
		goto probe_fail;
	}
       rc = ov7692_probe_init_gpio(info);

	ov7692_power_on();
	   	
	rc = msm_camera_power_frontend(MSM_CAMERA_PWRUP_MODE);
    if (rc < 0)
    {
        pr_err("%s: camera_power_frontendend failed!\n", __func__);
        goto probe_fail;
    }
#if defined(CONFIG_MACH_BLADE2) || defined(CONFIG_MACH_ATLAS40)|| defined(CONFIG_MACH_MOONCAKE2)     
     rc = msm_camera_clk_switch(info, OV7692_GPIO_SWITCH_CTL, OV7692_GPIO_SWITCH_VAL);
    if (rc < 0)
    {
        pr_err("%s: msm_camera_clk_switch failed!\n", __func__);
        goto probe_fail;
    }
#endif
	msm_camio_clk_rate_set(24000000);
	rc = ov7692_probe_init_sensor(info);
	if (rc < 0)
		goto probe_fail;
	s->s_init = ov7692_sensor_open_init;
	s->s_release = ov7692_sensor_release;
	s->s_config  = ov7692_sensor_config;
	s->s_camera_type = FRONT_CAMERA_2D;
	s->s_mount_angle = 90;
	ov7692_power_down();
	return rc;

probe_fail:
	pr_err("ov7692_sensor_probe: SENSOR PROBE FAILS!\n");
	i2c_del_driver(&ov7692_i2c_driver);
	return rc;
}

static int __ov7692_probe(struct platform_device *pdev)
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
	return msm_camera_drv_start(pdev, ov7692_sensor_probe);
#else
    return msm_camera_drv_start(pdev, ov7692_sensor_probe,1);
#endif
}

static struct platform_driver msm_camera_driver = {
	.probe = __ov7692_probe,
	.driver = {
		.name = "msm_camera_ov7692",
		.owner = THIS_MODULE,
	},
};

static int __init ov7692_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

module_init(ov7692_init);

MODULE_DESCRIPTION("OMNI VGA YUV sensor driver");
MODULE_LICENSE("GPL v2");
