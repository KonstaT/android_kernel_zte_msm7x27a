/* drivers/input/misc/kxtik.c - KXTIK accelerometer driver
 *
 * Copyright (C) 2012 Kionix, Inc.
 * Written by Kuching Tan <kuchingtan@kionix.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/kxtik.h>
#include <linux/version.h>
#ifdef    CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif /* CONFIG_HAS_EARLYSUSPEND */

#define NAME			"kxtik"
#define G_MAX			8096
/* OUTPUT REGISTERS */
#define XOUT_L			0x06
#define WHO_AM_I		0x0F
/* CONTROL REGISTERS */
#define INT_REL			0x1A
#define CTRL_REG1		0x1B
#define INT_CTRL1		0x1E
#define DATA_CTRL		0x21
/* CONTROL REGISTER 1 BITS */
#define PC1_OFF			0x7F
#define PC1_ON			(1 << 7)
/* Data ready funtion enable bit: set during probe if using irq mode */
#define DRDYE			(1 << 5)
/* INTERRUPT CONTROL REGISTER 1 BITS */
/* Set these during probe if using irq mode */
#define KXTIK_IEL		(1 << 3)
#define KXTIK_IEA		(1 << 4)
#define KXTIK_IEN		(1 << 5)
/* INPUT_ABS CONSTANTS */
#define FUZZ			3
#define FLAT			3

/*
 * The following table lists the maximum appropriate poll interval for each
 * available output data rate (ODR). Adjust by commenting off the ODR entry
 * that you want to omit.
 */
static const struct {
	unsigned int cutoff;
	u8 mask;
} kxtik_odr_table[] = {
/*	{ 3,	ODR800F },
	{ 5,	ODR400F }, */
	{ 10,	ODR200F },
	{ 20,	ODR100F },
	{ 40,	ODR50F  },
	{ 80,	ODR25F  },
	{ 0,	ODR12_5F},
};

struct kxtik_data {
	struct i2c_client *client;
	struct kxtik_platform_data pdata;
	struct input_dev *input_dev;
	struct work_struct irq_work;
	struct workqueue_struct *irq_workqueue;
	unsigned int poll_interval;
	unsigned int poll_delay;
	u8 shift;
	u8 ctrl_reg1;
	u8 data_ctrl;
	u8 int_ctrl;
	atomic_t acc_enabled;
	atomic_t acc_input_event;

#ifdef    CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif /* CONFIG_HAS_EARLYSUSPEND */
};

static int kxtik_i2c_read(struct kxtik_data *tik, u8 addr, u8 *data, int len)
{
	struct i2c_msg msgs[] = {
		{
			.addr = tik->client->addr,
			.flags = tik->client->flags,
			.len = 1,
			.buf = &addr,
		},
		{
			.addr = tik->client->addr,
			.flags = tik->client->flags | I2C_M_RD,
			.len = len,
			.buf = data,
		},
	};

	return i2c_transfer(tik->client->adapter, msgs, 2);
}

static void kxtik_report_acceleration_data(struct kxtik_data *tik)
{
	s16 acc_data[3]; /* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	s16 x, y, z;
	int err;
	struct input_dev *input_dev = tik->input_dev;
	int loop = 10;

	//dev_err(&tik->client->dev, "kxtik_report_acceleration_data\n");

	while(loop) {
	mutex_lock(&input_dev->mutex);
	err = kxtik_i2c_read(tik, XOUT_L, (u8 *)acc_data, 6);
	mutex_unlock(&input_dev->mutex);
		if(err < 0){
			loop--;
			mdelay(1);
		}
		else
			loop = 0;
	}
	if (err < 0) {
		dev_err(&tik->client->dev, "accelerometer data read failed. err(%d)\n", err);
	}
	else {
	x = ((s16) le16_to_cpu(acc_data[tik->pdata.axis_map_x])) >> tik->shift;
	y = ((s16) le16_to_cpu(acc_data[tik->pdata.axis_map_y])) >> tik->shift;
	z = ((s16) le16_to_cpu(acc_data[tik->pdata.axis_map_z])) >> tik->shift;

	if(atomic_read(&tik->acc_input_event) > 0) {
		input_report_abs(tik->input_dev, ABS_X, tik->pdata.negate_x ? -x : x);
		input_report_abs(tik->input_dev, ABS_Y, tik->pdata.negate_y ? -y : y);
		input_report_abs(tik->input_dev, ABS_Z, tik->pdata.negate_z ? -z : z);
		input_sync(tik->input_dev);
	}
}
}

static irqreturn_t kxtik_isr(int irq, void *dev)
{
	struct kxtik_data *tik = dev;
	queue_work(tik->irq_workqueue, &tik->irq_work);
	return IRQ_HANDLED;
}

static void kxtik_irq_work(struct work_struct *work)
{
	struct kxtik_data *tik = container_of(work,	struct kxtik_data, irq_work);
	int err;
	int loop = 10;

	/* data ready is the only possible interrupt type */
	kxtik_report_acceleration_data(tik);

	while(loop) {
	err = i2c_smbus_read_byte_data(tik->client, INT_REL);
		if(err < 0){
			loop--;
			mdelay(1);
		}
		else
			loop = 0;
	}
	if (err < 0)
		dev_err(&tik->client->dev,
			"error clearing interrupt status: %d\n", err);
}

static int kxtik_update_g_range(struct kxtik_data *tik, u8 new_g_range)
{
	switch (new_g_range) {
	case KXTIK_G_2G:
		tik->shift = 4;
		break;
	case KXTIK_G_4G:
		tik->shift = 3;
		break;
	case KXTIK_G_8G:
		tik->shift = 2;
		break;
	default:
		return -EINVAL;
	}

	tik->ctrl_reg1 &= 0xE7;
	tik->ctrl_reg1 |= new_g_range;

	return 0;
}

static int kxtik_update_odr(struct kxtik_data *tik, unsigned int poll_interval)
{
	int err;
	int i;
	u8 odr;

	/* Use the lowest ODR that can support the requested poll interval */
	for (i = 0; i < ARRAY_SIZE(kxtik_odr_table); i++) {
		odr = kxtik_odr_table[i].mask;
		if (poll_interval < kxtik_odr_table[i].cutoff)
			break;
	}

	/* Do not need to update DATA_CTRL_REG register if the ODR is not changed */
	if(tik->data_ctrl == odr)
		return 0;
	else
		tik->data_ctrl = odr;

	/* Do not need to update DATA_CTRL_REG register if the sensor is not currently turn on */
	if(atomic_read(&tik->acc_enabled) > 0) {
		err = i2c_smbus_write_byte_data(tik->client, CTRL_REG1, 0);
		if (err < 0)
			return err;

		err = i2c_smbus_write_byte_data(tik->client, DATA_CTRL, tik->data_ctrl);
		if (err < 0)
			return err;

		err = i2c_smbus_write_byte_data(tik->client, CTRL_REG1, tik->ctrl_reg1 | PC1_ON);
		if (err < 0)
			return err;
	}

	return 0;
}

static int kxtik_device_power_on(struct kxtik_data *tik)
{
	if (tik->pdata.power_on)
		return tik->pdata.power_on();

	return 0;
}

static void kxtik_device_power_off(struct kxtik_data *tik)
{
	if (tik->pdata.power_off)
		tik->pdata.power_off();
}

static int kxtik_power_on_init(struct kxtik_data *tik)
{
	int err;

	/* ensure that PC1 is cleared before updating control registers */
	err = i2c_smbus_write_byte_data(tik->client,
					CTRL_REG1, 0);
	if (err < 0)
		return err;

	err = i2c_smbus_write_byte_data(tik->client,
					DATA_CTRL, tik->data_ctrl);
	if (err < 0)
		return err;

	/* only write INT_CTRL_REG1 if in irq mode */
	if (tik->client->irq) {
		err = i2c_smbus_write_byte_data(tik->client,
						INT_CTRL1, tik->int_ctrl);
		if (err < 0)
			return err;
	}

	if(atomic_read(&tik->acc_enabled) > 0) {
		err = i2c_smbus_write_byte_data(tik->client,
						CTRL_REG1, tik->ctrl_reg1 | PC1_ON);
		if (err < 0)
			return err;
	}
	else {
		err = i2c_smbus_write_byte_data(tik->client,
						CTRL_REG1, tik->ctrl_reg1);
		if (err < 0)
			return err;
	}

	return 0;
}

static int kxtik_operate(struct kxtik_data *tik)
{
	int err;

	err = i2c_smbus_write_byte_data(tik->client, CTRL_REG1, tik->ctrl_reg1 | PC1_ON);
	if (err < 0)
		return err;

	return 0;
}

static int kxtik_standby(struct kxtik_data *tik)
{
	int err;

	err = i2c_smbus_write_byte_data(tik->client, CTRL_REG1, 0);
	if (err < 0)
		return err;

	return 0;
}

static int kxtik_enable(struct kxtik_data *tik)
{
	int err;

		err = kxtik_operate(tik);
		if (err < 0)
		dev_err(&tik->client->dev, "operate mode failed\n");

	atomic_inc(&tik->acc_enabled);
	printk("kxtik_enable. Count = %d\n", atomic_read(&tik->acc_enabled));
	return 0;
}

static void kxtik_disable(struct kxtik_data *tik)
{
	int err;

if(atomic_read(&tik->acc_enabled) > 0) {
	if(atomic_dec_and_test(&tik->acc_enabled)) {
		err = kxtik_standby(tik);
		if (err < 0)
			dev_err(&tik->client->dev, "standby mode failed\n");
	}
}
	printk("kxtik_disable. Count = %d\n", atomic_read(&tik->acc_enabled));
}

static int kxtik_input_open(struct input_dev *input)
{
	struct kxtik_data *tik = input_get_drvdata(input);

	atomic_inc(&tik->acc_input_event);

	return 0;
}

static void kxtik_input_close(struct input_dev *dev)
{
	struct kxtik_data *tik = input_get_drvdata(dev);

	atomic_dec(&tik->acc_input_event);
}

static void __devinit kxtik_init_input_device(struct kxtik_data *tik,
					      struct input_dev *input_dev)
{
	__set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_X, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(input_dev, ABS_Y, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(input_dev, ABS_Z, -G_MAX, G_MAX, FUZZ, FLAT);

	input_dev->name = "kxtik_accel";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &tik->client->dev;
}

static int __devinit kxtik_setup_input_device(struct kxtik_data *tik)
{
	struct input_dev *input_dev;
	int err;

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&tik->client->dev, "input device allocate failed\n");
		return -ENOMEM;
	}

	tik->input_dev = input_dev;

	input_dev->open = kxtik_input_open;
	input_dev->close = kxtik_input_close;
	input_set_drvdata(input_dev, tik);

	kxtik_init_input_device(tik, input_dev);

	err = input_register_device(tik->input_dev);
	if (err) {
		dev_err(&tik->client->dev,
			"unable to register input polled device %s: %d\n",
			tik->input_dev->name, err);
		input_free_device(tik->input_dev);
		return err;
	}

	return 0;
}

/*
 * When IRQ mode is selected, we need to provide an interface to allow the user
 * to change the output data rate of the part.  For consistency, we are using
 * the set_poll method, which accepts a poll interval in milliseconds, and then
 * calls update_odr() while passing this value as an argument.  In IRQ mode, the
 * data outputs will not be read AT the requested poll interval, rather, the
 * lowest ODR that can support the requested interval.  The client application
 * will be responsible for retrieving data from the input node at the desired
 * interval.
 */

/* Returns currently selected poll interval (in ms) */
static ssize_t kxtik_get_poll(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtik_data *tik = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", tik->poll_interval);
}

/* Allow users to select a new poll interval (in ms) */
static ssize_t kxtik_set_poll(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtik_data *tik = i2c_get_clientdata(client);
	struct input_dev *input_dev = tik->input_dev;
	unsigned int interval;

	#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35))
	int error;
	error = kstrtouint(buf, 10, &interval);
	if (error < 0)
		return error;
	#else
	interval = (unsigned int)simple_strtoul(buf, NULL, 10);
	#endif

	/* Lock the device to prevent races with open/close (and itself) */
	mutex_lock(&input_dev->mutex);

	disable_irq(client->irq);

	/*
	 * Set current interval to the greater of the minimum interval or
	 * the requested interval
	 */
	tik->poll_interval = max(interval, tik->pdata.min_interval);
	tik->poll_delay = msecs_to_jiffies(tik->poll_interval);

	kxtik_update_odr(tik, tik->poll_interval);

	enable_irq(client->irq);
	mutex_unlock(&input_dev->mutex);

	return count;
}

/* Allow users to enable/disable the device */
static ssize_t kxtik_set_enable(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtik_data *tik = i2c_get_clientdata(client);
	struct input_dev *input_dev = tik->input_dev;
	unsigned int enable;

	#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35))
	int error;
	error = kstrtouint(buf, 10, &enable);
	if (error < 0)
		return error;
	#else
	enable = (unsigned int)simple_strtoul(buf, NULL, 10);
	#endif

	/* Lock the device to prevent races with open/close (and itself) */
	mutex_lock(&input_dev->mutex);

	if(enable)
		kxtik_enable(tik);
	else
		kxtik_disable(tik);

	mutex_unlock(&input_dev->mutex);

	return count;
}

static DEVICE_ATTR(poll, S_IRUGO|S_IWUSR, kxtik_get_poll, kxtik_set_poll);
static DEVICE_ATTR(enable, S_IWUSR, NULL, kxtik_set_enable);

static struct attribute *kxtik_attributes[] = {
	&dev_attr_poll.attr,
	&dev_attr_enable.attr,
	NULL
};

static struct attribute_group kxtik_attribute_group = {
	.attrs = kxtik_attributes
};

static int __devinit kxtik_verify(struct kxtik_data *tik)
{
	int retval;

	retval = i2c_smbus_read_byte_data(tik->client, WHO_AM_I);
	if (retval < 0)
		dev_err(&tik->client->dev, "error reading WHO_AM_I register!\n");
	else
		retval = retval != 0x05 ? -EIO : 0;

	return retval;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
void kxtik_earlysuspend_suspend(struct early_suspend *h)
{
	struct kxtik_data *tik = container_of(h, struct kxtik_data, early_suspend);
	int err;

	err = kxtik_standby(tik);
	if (err < 0)
	dev_err(&tik->client->dev, "earlysuspend failed to suspend\n");

	return;
}

void kxtik_earlysuspend_resume(struct early_suspend *h)
{
	struct kxtik_data *tik = container_of(h, struct kxtik_data, early_suspend);
	int err;

	if(atomic_read(&tik->acc_enabled) > 0) {
		err = kxtik_operate(tik);
		if (err < 0)
			dev_err(&tik->client->dev, "earlysuspend failed to resume\n");
	}

	return;
}
#endif

static int __devinit kxtik_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	const struct kxtik_platform_data *pdata = client->dev.platform_data;
	struct kxtik_data *tik;
	int err;

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_I2C | I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "client is not i2c capable\n");
		return -ENXIO;
	}

	if (!pdata) {
		dev_err(&client->dev, "platform data is NULL; exiting\n");
		return -EINVAL;
	}

	tik = kzalloc(sizeof(*tik), GFP_KERNEL);
	if (!tik) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		return -ENOMEM;
	}

	tik->client = client;
	tik->pdata = *pdata;

	err = kxtik_device_power_on(tik);
	if (err < 0)
		goto err_free_mem;

	if (pdata->init) {
		err = pdata->init();
		if (err < 0)
			goto err_pdata_power_off;
	}

	err = kxtik_verify(tik);
	if (err < 0) {
		dev_err(&client->dev, "device not recognized\n");
		goto err_pdata_exit;
	}

	i2c_set_clientdata(client, tik);

	atomic_set(&tik->acc_enabled, 0);
	atomic_set(&tik->acc_input_event, 0);

	tik->ctrl_reg1 = tik->pdata.res_12bit | tik->pdata.g_range;
	tik->poll_interval = tik->pdata.poll_interval;
	tik->poll_delay = msecs_to_jiffies(tik->poll_interval);

	kxtik_update_odr(tik, tik->poll_interval);

	err = kxtik_update_g_range(tik, tik->pdata.g_range);
	if (err < 0) {
		dev_err(&client->dev, "invalid g range\n");
		goto err_pdata_exit;
	}

	if (client->irq) {
		err = kxtik_setup_input_device(tik);
		if (err)
			goto err_pdata_exit;

		tik->irq_workqueue = create_workqueue("KXTIK Workqueue");
		INIT_WORK(&tik->irq_work, kxtik_irq_work);
		/* If in irq mode, populate INT_CTRL_REG1 and enable DRDY. */
		tik->int_ctrl |= KXTIK_IEN | KXTIK_IEA;
		tik->ctrl_reg1 |= DRDYE;

		err = request_threaded_irq(client->irq, NULL, kxtik_isr,
					   IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					   "kxtik-irq", tik);
		if (err) {
			dev_err(&client->dev, "request irq failed: %d\n", err);
			goto err_destroy_input;
		}

		err = kxtik_power_on_init(tik);
		if (err) {
			dev_err(&client->dev, "power on init failed: %d\n", err);
			goto err_free_irq;
		}

		err = sysfs_create_group(&client->dev.kobj, &kxtik_attribute_group);
		if (err) {
			dev_err(&client->dev, "sysfs create failed: %d\n", err);
			goto err_free_irq;
		}

	} else {
		dev_err(&client->dev, "irq not defined\n");

		goto err_pdata_exit;
	}

#ifdef    CONFIG_HAS_EARLYSUSPEND
	tik->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	tik->early_suspend.suspend = kxtik_earlysuspend_suspend;
	tik->early_suspend.resume = kxtik_earlysuspend_resume;
	register_early_suspend(&tik->early_suspend);
#endif /* CONFIG_HAS_EARLYSUSPEND */

	return 0;

err_free_irq:
if (client->irq) {
	free_irq(client->irq, tik);
	destroy_workqueue(tik->irq_workqueue);
}
err_destroy_input:
	input_unregister_device(tik->input_dev);
err_pdata_exit:
	if (pdata->exit)
		pdata->exit();
err_pdata_power_off:
	kxtik_device_power_off(tik);
err_free_mem:
	kfree(tik);
	return err;
}

static int __devexit kxtik_remove(struct i2c_client *client)
{
	struct kxtik_data *tik = i2c_get_clientdata(client);

#ifdef    CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&tik->early_suspend);
#endif /* CONFIG_HAS_EARLYSUSPEND */
	sysfs_remove_group(&client->dev.kobj, &kxtik_attribute_group);
	if (client->irq) {
		free_irq(client->irq, tik);
		destroy_workqueue(tik->irq_workqueue);
	}
	input_unregister_device(tik->input_dev);
	if (tik->pdata.exit)
		tik->pdata.exit();
	kxtik_device_power_off(tik);
	kfree(tik);

	return 0;
}

static const struct i2c_device_id kxtik_id[] = {
	{ NAME, 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, kxtik_id);

static struct i2c_driver kxtik_driver = {
	.driver = {
		.name	= NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= kxtik_probe,
	.remove		= __devexit_p(kxtik_remove),
	.id_table	= kxtik_id,
};

static int __init kxtik_init(void)
{
	return i2c_add_driver(&kxtik_driver);
}
module_init(kxtik_init);

static void __exit kxtik_exit(void)
{
	i2c_del_driver(&kxtik_driver);
}
module_exit(kxtik_exit);

MODULE_DESCRIPTION("KXTIK accelerometer driver");
MODULE_AUTHOR("Kuching Tan <kuchingtan@kionix.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("2.4.0");
