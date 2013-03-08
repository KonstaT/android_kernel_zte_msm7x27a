/*
 *  mma8452.c - Linux kernel modules for 3-Axis Orientation/Motion
 *  Detection Sensor 
 *
 *  Copyright (C) 2010-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/pm.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/input-polldev.h>
#include <linux/earlysuspend.h>
#include <linux/mma854x.h>


/* mma8452 status */
struct mma8452_status {
	u8 mode;
	u8 ctl_reg1;
	int active;
	int position;
};

struct mma8452_data {
	struct early_suspend mma_early_suspend;
};

static struct mma8452_status mma_status;
static struct input_polled_dev *mma8452_idev;
static struct device *hwmon_dev;
static struct i2c_client *mma8452_i2c_client;

static int senstive_mode = MODE_2G;
static int ACCHAL[8][3][3] =
{
   {{ 0, -1,  0}, { 1,  0,	0}, {0, 0,	1}},
   {{-1,  0,  0}, { 0, -1,	0}, {0, 0,	1}},
   {{ 0,  1,  0}, {-1,  0,	0}, {0, 0,	1}},
   {{ 1,  0,  0}, { 0,  1,	0}, {0, 0,	1}},
   
   {{ 0, -1,  0}, {-1,  0,	0}, {0, 0,  -1}},
   {{-1,  0,  0}, { 0,  1,	0}, {0, 0,  -1}},
   {{ 0,  1,  0}, { 1,  0,	0}, {0, 0,  -1}},
   {{ 1,  0,  0}, { 0, -1,	0}, {0, 0,  -1}},
};

static DEFINE_MUTEX(mma8452_lock);

static void mma8452_early_suspend(struct early_suspend *handler);

static void mma8452_early_resume(struct early_suspend *handler);

static int mma8452_adjust_position(short *x, short *y, short *z)
{
   short rawdata[3],data[3];
   int i,j;
   int position = mma_status.position ;
   if(position < 0 || position > 7 )
   		position = 0;
   rawdata [0] = *x ; rawdata [1] = *y ; rawdata [2] = *z ;  
   for(i = 0; i < 3 ; i++)
   {
   	data[i] = 0;
   	for(j = 0; j < 3; j++)
		data[i] += rawdata[j] * ACCHAL[position][i][j];
   }
   *x = data[0];
   *y = data[1];
   *z = data[2];
   return 0;
}

static int mma8452_change_mode(struct i2c_client *client, int mode)
{
		int result;

		mma_status.ctl_reg1 = 0x20;
		result = i2c_smbus_write_byte_data(client, MMA8452_CTRL_REG1, mma_status.ctl_reg1);
		if (result < 0)
		goto out;
		
		mma_status.mode = mode;
		result = i2c_smbus_write_byte_data(client, MMA8452_XYZ_DATA_CFG, mma_status.mode);
		if (result < 0)
			goto out;
		mma_status.active = MMA_STANDBY;
		//enable disable by enable interface
	    /*  
		mma_status.ctl_reg1 |= 0x01;
		result = i2c_smbus_write_byte_data(client, MMA8452_CTRL_REG1,
						   mma_status.ctl_reg1);
		if(result)
				printk("%s MMA845x read i2c error\n",__FUNCTION__);
		
		if (result < 0)
			goto out;
		*/
		mdelay(MODE_CHANGE_DELAY_MS);

		return 0;
out:
		dev_err(&client->dev, "error when init mma8452:(%d)", result);
		return result;
}

/***************************************************************
*
* read sensor data from mma8452
*
***************************************************************/ 				
static int mma8452_read_data(short *x, short *y, short *z)
{

	u8 tmp_data[MMA8452_BUF_SIZE];
	int ret;

	ret = i2c_smbus_read_i2c_block_data(mma8452_i2c_client,
					    MMA8452_OUT_X_MSB, 7, tmp_data);
	if (ret < MMA8452_BUF_SIZE) {
		dev_err(&mma8452_i2c_client->dev, "i2c block read failed\n");
		return -EIO;
	}

	*x = ((tmp_data[0] << 8) & 0xff00) | tmp_data[1];
	*y = ((tmp_data[2] << 8) & 0xff00) | tmp_data[3];
	*z = ((tmp_data[4] << 8) & 0xff00) | tmp_data[5];

	*x = (short)(*x) >> 4;
	*y = (short)(*y) >> 4;
	*z = (short)(*z) >> 4;


	if (mma_status.mode == MODE_4G){
		(*x)=(*x)<<1;
		(*y)=(*y)<<1;
		(*z)=(*z)<<1;
	} else if (mma_status.mode == MODE_8G){
		(*x)=(*x)<<2;
		(*y)=(*y)<<2;
		(*z)=(*z)<<2;
	}

	return 0;
}

static void report_abs(void)
{
	short x, y, z;
	int result;
	int retry = 3;

	mutex_lock(&mma8452_lock);
	if(mma_status.active & (MMA_STANDBY|MMA_SUSPEND))
		goto out;
	/* wait for the data ready */
	do {
		result = i2c_smbus_read_byte_data(mma8452_i2c_client,
					     MMA8452_STATUS);
		retry --;
		msleep(1);
	} while (!(result & MMA8452_STATUS_ZYXDR)&& retry > 0);
    	if(retry == 0)
		goto out;
	if (mma8452_read_data(&x, &y, &z) != 0)
		goto out;
    mma8452_adjust_position(&x, &y, &z);
	input_report_abs(mma8452_idev->input, ABS_X, x);
	input_report_abs(mma8452_idev->input, ABS_Y, y);
	input_report_abs(mma8452_idev->input, ABS_Z, z);
	input_sync(mma8452_idev->input);
out:
	mutex_unlock(&mma8452_lock);
}

static void mma8452_dev_poll(struct input_polled_dev *dev)
{
	report_abs();
} 

static ssize_t mma8452_enable_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct i2c_client *client;
	u8 val;
    int enable;
	
	mutex_lock(&mma8452_lock);
	client = mma8452_i2c_client;
	val = i2c_smbus_read_byte_data(client, MMA8452_CTRL_REG1);  
	if((val & 0x01))
		enable = 1;
	else
		enable = 0;
	mutex_unlock(&mma8452_lock);
	return sprintf(buf, "%d\n", enable);
}

static ssize_t mma8452_enable_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct i2c_client *client;
	int ret;
	unsigned long enable;
	u8 val = 0;
	enable = simple_strtoul(buf, NULL, 10);    
	mutex_lock(&mma8452_lock);
    client = mma8452_i2c_client;
	enable = (enable > 0) ? 1 : 0;
    val = i2c_smbus_read_byte_data(client,MMA8452_CTRL_REG1);
	printk(SENSOR_TAG "%s enable = %d ,val = 0x%x\n",__FUNCTION__,(int)enable,val);
    if(enable && (val&0x01)== 0)
	{  
	   ret = i2c_smbus_write_byte_data(client, MMA8452_CTRL_REG1, val|0x01);  
	   if(!ret){
		   	 mma_status.active = MMA_ACTIVED;
			 printk("mma enable setting active \n");
	    }
	   else{
	   		printk("%s MMA845x read i2c error\n",__FUNCTION__);
	   }
	   	 
	}
	else if(enable == 0  && (val&0x01)== 0x01)
	{
	    ret = i2c_smbus_write_byte_data(client, MMA8452_CTRL_REG1,val & 0xFE);
		if(!ret){
		 mma_status.active = MMA_STANDBY;
		 printk("mma enable setting inactive \n");
		}
	}
	mutex_unlock(&mma8452_lock);
	return count;
}
static ssize_t mma8452_position_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
    int position = 0;
	mutex_lock(&mma8452_lock);
    position = mma_status.position ;
	mutex_unlock(&mma8452_lock);
	return sprintf(buf, "%d\n", position);
}

static ssize_t mma8452_position_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	int  position;
	position = simple_strtoul(buf, NULL, 10);    
	mutex_lock(&mma8452_lock);
    mma_status.position = position;
	mutex_unlock(&mma8452_lock);
	return count;
}

static DEVICE_ATTR(enable, S_IWUSR | S_IRUGO,
		   mma8452_enable_show, mma8452_enable_store);
static DEVICE_ATTR(position, S_IWUSR | S_IRUGO,
		   mma8452_position_show, mma8452_position_store);

static struct attribute *mma8452_attributes[] = {
	&dev_attr_enable.attr,   
	&dev_attr_position.attr,
	NULL
};

static const struct attribute_group mma8452_attr_group = {
	.attrs = mma8452_attributes,
};

static int __devinit mma8452_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	int result, client_id;
	struct input_dev *idev;
	struct i2c_adapter *adapter;
	struct mma8452_data *mma;

	const struct mma8452_platform_data *pdata = client->dev.platform_data;
	
	if (!pdata) {
		dev_err(&client->dev, "platform data for layout is NULL; exiting\n");
		return -EINVAL;
	}


	/* Allocate memory for driver data */
	mma = kzalloc(sizeof(struct mma8452_data), GFP_KERNEL);
	if (!mma) {
		printk(KERN_ERR "mma8452_probe: memory allocation failed.\n");
		return -ENOMEM;
	}

	mma8452_i2c_client = client;

	printk(SENSOR_TAG "come into %s\n",__FUNCTION__);
	adapter = to_i2c_adapter(client->dev.parent);
	result = i2c_check_functionality(adapter,
					 I2C_FUNC_SMBUS_BYTE |
					 I2C_FUNC_SMBUS_BYTE_DATA);
	if (!result)
		goto err_out;
	else
		printk(SENSOR_TAG "i2c_check_functionality() was called success\n");

	client_id = i2c_smbus_read_byte_data(client, MMA8452_WHO_AM_I);

	if (client_id != MMA8452_ID && client_id != MMA8452_ID) {
		dev_err(&client->dev,
			"read chip ID 0x%x is not equal to 0x%x or 0x%x!\n",
			result, MMA8452_ID, MMA8452_ID);
    	printk(SENSOR_TAG "I2C read error\n");
		result = -EINVAL;
		goto err_out;
	}else{
		printk(SENSOR_TAG "read chip ID is :0x%x\n",client_id);
	}

	/* Initialize the MMA8452 chip */
	result = mma8452_change_mode(client, senstive_mode);
	if (result) {
		dev_err(&client->dev,
			"error when init mma8452 chip:(%d)\n", result);
		goto err_out;
	}

	hwmon_dev = hwmon_device_register(&client->dev);
	if (!hwmon_dev) {
		result = -ENOMEM;
		dev_err(&client->dev,
			"error when register hwmon device\n");
		goto err_out;
	}

	mma8452_idev = input_allocate_polled_device();
	if (!mma8452_idev) {
		result = -ENOMEM;
		dev_err(&client->dev, "alloc poll device failed!\n");
		goto err_alloc_poll_device;
	}
	mma8452_idev->poll = mma8452_dev_poll;
	mma8452_idev->poll_interval = POLL_INTERVAL;
	mma8452_idev->poll_interval_min = POLL_INTERVAL_MIN;
	mma8452_idev->poll_interval_max = POLL_INTERVAL_MAX;
	idev = mma8452_idev->input;
	if (client_id == MMA8452_ID)
		idev->name = "mma8452";
	else
		idev->name = "mma8452";
	idev->id.bustype = BUS_I2C;
	idev->evbit[0] = BIT_MASK(EV_ABS);

	input_set_abs_params(idev, ABS_X, -8192, 8191, INPUT_FUZZ, INPUT_FLAT);
	input_set_abs_params(idev, ABS_Y, -8192, 8191, INPUT_FUZZ, INPUT_FLAT);
	input_set_abs_params(idev, ABS_Z, -8192, 8191, INPUT_FUZZ, INPUT_FLAT);

	result = input_register_polled_device(mma8452_idev);
	if (result) {
		dev_err(&client->dev, "register poll device failed!\n");
		goto err_register_polled_device;
	}

	mma->mma_early_suspend.suspend = mma8452_early_suspend;
	mma->mma_early_suspend.resume = mma8452_early_resume;
	register_early_suspend(&mma->mma_early_suspend);
	
    result = sysfs_create_group(&idev->dev.kobj, &mma8452_attr_group);
	if (result) {
		dev_err(&client->dev, "create device file failed!\n");
		result = -EINVAL;
		goto err_register_polled_device;
	}
	mma_status.position = (*pdata).axis_map_cordination;
	return 0;    
err_register_polled_device:
	input_free_polled_device(mma8452_idev);
err_alloc_poll_device:
	hwmon_device_unregister(&client->dev);
err_out:
	return result;
}

static int mma8452_stop_chip(struct i2c_client *client)
{
    int ret = 0;
    if(mma_status.active == MMA_ACTIVED)
	{
	   mma_status.ctl_reg1 = i2c_smbus_read_byte_data(client,
						       MMA8452_CTRL_REG1);
	 	ret = i2c_smbus_write_byte_data(client, MMA8452_CTRL_REG1,
					   mma_status.ctl_reg1 & 0xFE);
		if(ret)
			printk("%s MMA845x read i2c error\n",__FUNCTION__);	
    }
	return ret;
}

static int __devexit mma8452_remove(struct i2c_client *client)
{
	int ret;
	ret = mma8452_stop_chip(client);
	
	hwmon_device_unregister(hwmon_dev);

	return ret;
}

static void mma8452_early_suspend(struct early_suspend *handler)
{
	printk(KERN_INFO "come into %s\n",__func__);

	mma8452_stop_chip(mma8452_i2c_client);
	mma_status.active |= MMA_SUSPEND;

}

static void mma8452_early_resume(struct early_suspend *handler)
{
    int ret = 0;
	printk(KERN_INFO "come into %s\n",__func__);
	
    if(mma_status.active & MMA_ACTIVED){
	   ret = i2c_smbus_write_byte_data(mma8452_i2c_client, MMA8452_CTRL_REG1,
					 	mma_status.ctl_reg1|0x01);
	   if(ret)
			printk("%s MMA845x read i2c error\n",__FUNCTION__);
	   }
	mma_status.active &= (~MMA_SUSPEND);  
}

static const struct i2c_device_id mma8452_id[] = {
	{"mma8452", 0},
        { },
};
MODULE_DEVICE_TABLE(i2c, mma8452_id);

static struct i2c_driver mma8452_driver = {
	.driver = {
		   .name = "mma8452",
		   .owner = THIS_MODULE,
		   },
	.probe = mma8452_probe,
	.remove = __devexit_p(mma8452_remove),
	.id_table = mma8452_id,
};

static int __init mma8452_init(void)
{
	/* register driver */
	int res;
	
    printk(SENSOR_TAG "come into %s\n",__FUNCTION__);
	res = i2c_add_driver(&mma8452_driver);
	if (res < 0) {
		printk(KERN_INFO "add mma8452 i2c driver failed\n");
		return -ENODEV;
	}
	return res;
}

static void __exit mma8452_exit(void)
{
	i2c_del_driver(&mma8452_driver);
}

MODULE_AUTHOR("Chen Gang <gang.chen@freescale.com>");
MODULE_DESCRIPTION("MMA8452 3-Axis Orientation/Motion Detection Sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.3.3");

module_init(mma8452_init);
module_exit(mma8452_exit);
