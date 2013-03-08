/*drivers/input/keyboard/ft5x0x_ts.c
 *This file is used for FocalTech ft5x0x_ts touchscreen
 *
*/

/*
=======================================================================================================
When		Who	What,Where,Why		Comment			Tag

*/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/input/focaltech_ft5x0x.h>
#include <mach/gpio.h>
#include <linux/fb.h>
#include <asm/uaccess.h>
#include <linux/fs.h>

static struct focaltech_ts_data *ftc_ts = NULL;

#if defined(CONFIG_TOUCHSCREEN_FOCALTECH_FW)
int ftc_update_flag = 0;
int focaltech_fwupdate(struct i2c_client *client, char *pfwfile );
int focaltech_fwupdate_init(struct i2c_client *client);
int focaltech_fwupdate_deinit(struct i2c_client *client);
#endif
static void focaltech_get_vid(struct i2c_client *client,char *p_vid,int *p_fw_ver );

#if defined(CONFIG_TOUCHSCREEN_FOCALTECH_USBNOTIFY)
static int usb_plug_status=0;
#endif


#define ABS_SINGLE_TAP	0x21	/* Major axis of touching ellipse */
#define ABS_TAP_HOLD	0x22	/* Minor axis (omit if circular) */
#define ABS_DOUBLE_TAP	0x23	/* Major axis of approaching ellipse */
#define ABS_EARLY_TAP	0x24	/* Minor axis (omit if circular) */
#define ABS_FLICK	0x25	/* Ellipse orientation */
#define ABS_PRESS	0x26	/* Major axis of touching ellipse */
#define ABS_PINCH 	0x27	/* Minor axis (omit if circular) */



static struct workqueue_struct *focaltech_wq;
static struct i2c_driver focaltech_ts_driver;

struct focaltech_ts_data
{
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct focaltech_finger_data finger_data[5];//ZTE_TS_XYM_20110711
	int touch_number;
	int touch_event;
	int use_irq;
	struct hrtimer timer;
	struct work_struct  work;
	uint16_t max[2];
	struct early_suspend early_suspend;
//	int gpio_irq;
	int (*gpio_init)(int on);
	void (*power)(int on);
	void (*reset)(void);
	void (*irqwake)(bool wake);
	char  fwfile[64];
};


#if defined (CONFIG_TOUCHSCREEN_FOCALTECH_USBNOTIFY)
static int focaltech_ts_event(struct notifier_block *this, unsigned long event,void *ptr)
{
	int ret;

	switch(event)
	{
	case 0:
		//offline
		if ( usb_plug_status != 0 ){
	 		usb_plug_status = 0;
			//printk("ts config change to offline status\n");
			i2c_smbus_write_byte_data( ftc_ts->client, 0x86,0x1);
		}
		break;
	case 1:
		//online
		if ( usb_plug_status != 1 ){
	 		usb_plug_status = 1;
			//printk("ts config change to online status\n");
			i2c_smbus_write_byte_data( ftc_ts->client, 0x86,0x3);
		}
		break;
	default:
		break;
	}

	ret = NOTIFY_DONE;

	return ret;
}

static struct notifier_block ts_notifier = {
	.notifier_call = focaltech_ts_event,
};


static BLOCKING_NOTIFIER_HEAD(ts_chain_head);

int focaltech_register_ts_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&ts_chain_head, nb);
}
EXPORT_SYMBOL_GPL(focaltech_register_ts_notifier);

int focaltech_unregister_ts_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&ts_chain_head, nb);
}
EXPORT_SYMBOL_GPL(focaltech_unregister_ts_notifier);

int focaltech_ts_notifier_call_chain(unsigned long val)
{
	return (blocking_notifier_call_chain(&ts_chain_head, val, NULL)
			== NOTIFY_BAD) ? -EINVAL : 0;
}

#endif


static bool  detect_device(struct i2c_client *client)
{
	int retry;//ret;
	signed int buf;
	
	retry = 3;
	while (retry-- > 0)
	{
		buf = i2c_smbus_read_byte_data(client, FT5X0X_REG_FIRMID);
		if ( buf >= 0 ){
			return true;
		}
		msleep(10);
	}
	printk("wly: focaltech touch is not exsit.\n");
	return false;
}

static int get_screeninfo(uint *xres, uint *yres)
{
	struct fb_info *info;

	info = registered_fb[0];
	if (!info) {
		pr_err("%s: Can not access lcd info \n",__func__);
		return -ENODEV;
	}

	*xres = info->var.xres;
	*yres = info->var.yres;
	printk("%s: xres=%d, yres=%d \n",__func__,*xres,*yres);

	return 1;
}


static int
proc_read_val(char *page, char **start, off_t off, int count, int *eof,
	  void *data)
{
	int len = 0;
	int buf = 0;
	char vid[16];

	len += sprintf(page + len, "%s\n", "touchscreen module");
	len += sprintf(page + len, "name     : %s\n", "FocalTech");
	len += sprintf(page + len, "i2c address  : 0x%x\n", 0x3E);

	focaltech_get_vid( ftc_ts->client, (char *)vid, &buf );
	len += sprintf(page + len, "module : FT5x06 + %s\n", vid);
	len += sprintf(page + len, "firmware : 0x%x\n", buf );

#ifdef CONFIG_TOUCHSCREEN_FOCALTECH_FW
	len += sprintf(page + len, "update flag : 0x%x\n", ftc_update_flag);
#endif

	if (off + count >= len)
		*eof = 1;
	if (len < off)
		return 0;
	*start = page + off;
	return ((count < len - off) ? count : len - off);
}

static int proc_write_val(struct file *file, const char *buffer,
           unsigned long count, void *data)
{
	unsigned long val;
	sscanf(buffer, "%lu", &val);

#ifdef CONFIG_TOUCHSCREEN_FOCALTECH_FW

	printk("ftc Upgrade Start++++++++\n");
	ftc_update_flag=0;
    
	if ( focaltech_fwupdate( ftc_ts->client, ftc_ts->fwfile ) < 0 )
	{
		printk("****ftc fw update failed!\n");
		ftc_update_flag=1;		
		return -EINVAL;
	}
	else
	{
		ftc_update_flag=2;
		pr_info("ftc fw update OK! \n" );
	}
#endif
	return -EINVAL;
 	//return 0;
}
/*
static void release_all_fingers(struct focaltech_ts_data *ts)
{
	int i;
	for(i =0; i< ts->touch_event; i ++ )
	{
		input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, ts->finger_data[i].touch_id);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, ts->finger_data[i].w );
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);		
		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, ts->finger_data[i].x );
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, ts->finger_data[i].y );
		input_mt_sync(ts->input_dev);	
	}
	input_sync(ts->input_dev);
}
*/

static void focaltech_get_vid(
	struct i2c_client *client,
	char *p_vid,
	int *p_fw_ver )
{
	int buf1, buf2;

	if ( !client )
		return;
	
	buf1 = i2c_smbus_read_byte_data(client, FT5X0X_REG_FT5201ID);
	buf2 = i2c_smbus_read_byte_data( client, FT5X0X_REG_FIRMID);
	pr_info("vendor id = 0x%x, fw version = 0x%x\n", buf1, buf2);

	if ( !p_vid || !p_fw_ver )
		return;

	switch (buf1){
	case 0x57:// ³¬ÉùGoworld
		sprintf( p_vid, "Goworld(0x%x)", buf1 );
		break;
	case 0x51:// Å··Æ¹âofilm
		sprintf( p_vid, "ofilm(0x%x)", buf1  );
		break;
	case 0x55:// À³±¦
		sprintf( p_vid, "laibao(0x%x)", buf1  );
		break;
	case 0x5a://
		sprintf( p_vid, "TRULY(0x%x)", buf1  );
		break;
	case 0x5f:// ÓîË³
		sprintf( p_vid, "success(0x%x)", buf1  );
		break;
	case 0x60:// Á¢µÂ
		sprintf( p_vid, "lead(0x%x)", buf1  );
		break;
	case 0x5d:// ±¦Ã÷
		sprintf( p_vid, "BM(0x%x)", buf1  );
		break;
	case 0x8f:// ÆæÃÀ
		sprintf( p_vid, "CMI(0x%x)", buf1  );
		break;
	case 0xA5:
		sprintf( p_vid, "jiaguan(0x%x)", buf1  );
		break;
	default:
		sprintf( p_vid, "unknown(0x%x)", buf1  );
		break;
	}		

	*p_fw_ver = buf2;

	pr_info("vendor: %s, fw =0x%x \n", p_vid, *p_fw_ver);

}


static void focaltech_ts_work_func(struct work_struct *work)
{
	int ret, i;
	uint8_t buf[33];
	struct focaltech_ts_data *ts = container_of(work, struct focaltech_ts_data, work);

	//ret = focaltech_i2c_read(ts->client, 0x00, buf, 33); 
	ret = i2c_smbus_read_i2c_block_data(ts->client, 0x00, 33, buf);
	if (ret < 0){
   		pr_err("%s: focaltech_i2c_read failed, go to poweroff.\n", __func__);
		if (ts->power){
			ts->power(0);
	    	msleep(200);
	    	ts->power(1);
	    	msleep(220);
		}
	}
	else
	{
		ts->touch_number = buf[2]&0x0f;
		ts->touch_event = buf[2] >> 4;
		//printk("xiayc~~~ touch number=%d, event=%d\n", 
		//	ts->touch_number, ts->touch_event);
		for (i = 0; i< 5; i++)
		{
			ts->finger_data[i].x = (uint16_t)((buf[3 + i*6] & 0x0F)<<8 )| (uint16_t)buf[4 + i*6];
			ts->finger_data[i].y = (uint16_t)((buf[5 + i*6] & 0x0F)<<8 )| (uint16_t)buf[6 + i*6];
			ts->finger_data[i].z = buf[7 + i*6];
			ts->finger_data[i].w = buf[8 + i*6];
			ts->finger_data[i].touch_id = buf[5 + i*6] >> 4;
			ts->finger_data[i].event_flag = buf[3 + i*6] >> 6;

		}
		for (i = 0; i< ts->touch_event; i++)
		{
			/*ergate-008*/
			if(ts->finger_data[i].z != 0)
			{
			input_report_key(ts->input_dev, BTN_TOUCH, 1);
			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, ts->finger_data[i].touch_id);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, ts->finger_data[i].z);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, ts->finger_data[i].w );
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, ts->finger_data[i].x );
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, ts->finger_data[i].y );
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, ts->finger_data[i].z);		
			}
			input_mt_sync(ts->input_dev);
			//printk("%s: finger=%d, z=%d, event_flag=%d, touch_id=%d\n", __func__, i, 
			//ts->finger_data[i].z, ts->finger_data[i].event_flag,ts->finger_data[i].touch_id);
		}

		input_sync(ts->input_dev);
	}

	if (ts->use_irq)
		enable_irq(ts->client->irq);
}

static irqreturn_t focaltech_ts_irq_handler(int irq, void *dev_id)
{
	struct focaltech_ts_data *ts = dev_id;

	disable_irq_nosync(ts->client->irq);
	queue_work(focaltech_wq, &ts->work);

	return IRQ_HANDLED;
}

static int focaltech_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret = 0;
	struct focaltech_ts_data *ts=NULL;
	
	ts = i2c_get_clientdata(client);
	disable_irq(client->irq);
	ret = cancel_work_sync(&ts->work);
	if(ret & ts->use_irq)
		enable_irq(client->irq);

	if ( ts->irqwake )
		ts->irqwake(0);

	i2c_smbus_write_byte_data(client, FT5X0X_REG_PMODE, PMODE_HIBERNATE);

	return 0;
}

static int focaltech_ts_resume(struct i2c_client *client)
{
	uint8_t retry=0;
	s32 ret=0;
	struct focaltech_ts_data *ts = i2c_get_clientdata(client);

focaltech_resume_start:	

	if ( ts->irqwake )
		ts->irqwake(1);

	//fix bug: fts failed set reg when usb plug in under suspend mode
#if defined(CONFIG_TOUCHSCREEN_FOCALTECH_USBNOTIFY)
	if(usb_plug_status==1)
		i2c_smbus_write_byte_data( ftc_ts->client, 0x86,0x3);
	else 
		i2c_smbus_write_byte_data( ftc_ts->client, 0x86,0x1);
#endif

	ret = i2c_smbus_read_byte_data(client, FT5X0X_REG_FIRMID );
	if ( 0>ret )
	{
		printk("%s: Fts FW ID read Error: retry=0x%X\n", __func__, retry);
		if ( ++retry < 3 ) goto focaltech_resume_start;
	}

	//release_all_fingers(ts);
	enable_irq(client->irq);
	
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void focaltech_ts_early_suspend(struct early_suspend *h)
{
	struct focaltech_ts_data *ts;
	
	ts = container_of(h, struct focaltech_ts_data, early_suspend);
	focaltech_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void focaltech_ts_late_resume(struct early_suspend *h)
{
	struct focaltech_ts_data *ts;
	ts = container_of(h, struct focaltech_ts_data, early_suspend);
	focaltech_ts_resume(ts->client);
}
#endif

static int focaltech_ts_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	struct focaltech_ts_data *ts=NULL;
	struct focaltech_ts_platform_data *pdata;
	int ret = 0;
	struct proc_dir_entry *dir, *refresh;
	int xres, yres;	// LCD x,y resolution


	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		printk(KERN_ERR "%s: need I2C_FUNC_I2C\n", __func__);
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL)
	{
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	pdata = client->dev.platform_data;
	if (pdata){
//		ts->gpio_irq = pdata->gpio_irq;
		ts->gpio_init = pdata->gpio_init;
		ts->power	= pdata->power;
		ts->reset	= pdata->reset;
		ts->irqwake	= pdata->irqwake;
		memcpy(ts->fwfile,pdata->fwfile,sizeof(pdata->fwfile));
	}

	if ( ts->gpio_init) {
		ret = ts->gpio_init(1);
		if ( ret < 0 ){
			pr_err("%s, gpio init failed!\n", __func__);
			goto err_power_failed;
		}

		if ( ts->reset ) ts->reset();
		msleep(10);

		if (ts->power) ts->power(1);
		msleep(300);
	}

	if ( !detect_device(client) )
		goto err_detect_failed;

	focaltech_get_vid(client, NULL, NULL );

	focaltech_wq= create_singlethread_workqueue("focaltech_wq");
	if(!focaltech_wq){
		ret = -ESRCH;
		pr_err("%s: creare single thread workqueue failed!\n", __func__);
		goto err_create_singlethread;
	}

	INIT_WORK(&ts->work, focaltech_ts_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);
	client->driver = &focaltech_ts_driver;
	ftc_ts = ts;

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		pr_err("%s: Failed to allocate input device\n", __func__);
		goto err_input_dev_alloc_failed;
	}
	
	ts->input_dev->name = "Fts-touchscreen";
	//ts->input_dev->phys = "Fts-touchscreen/input0";

	get_screeninfo(&xres, &yres);

	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(EV_ABS, ts->input_dev->evbit);
	//set_bit(BTN_9, ts->input_dev->keybit);
	set_bit(KEY_HOME, ts->input_dev->keybit);
	set_bit(KEY_MENU, ts->input_dev->keybit);
	set_bit(KEY_BACK, ts->input_dev->keybit);
	set_bit(KEY_SEARCH, ts->input_dev->keybit);
	set_bit(ABS_MT_TRACKING_ID, ts->input_dev->absbit);
	set_bit(ABS_MT_TOUCH_MAJOR, ts->input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, ts->input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, ts->input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, ts->input_dev->absbit);
	set_bit(ABS_MT_ORIENTATION, ts->input_dev->absbit);
	set_bit(ABS_MT_PRESSURE, ts->input_dev->absbit);
	/*ergate-008*/
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, 10, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 127, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, xres, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, yres, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 16, 208, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, 0xFF, 0, 0);
	/*input_set_abs_params(ts->input_dev, ABS_SINGLE_TAP, 0, 5, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_TAP_HOLD, 0, 5, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_EARLY_TAP, 0, 5, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_FLICK, 0, 5, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESS, 0, 5, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_DOUBLE_TAP, 0, 5, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_PINCH, -255, 255, 0, 0);*/

	ret = input_register_device(ts->input_dev);
	if (ret)
	{
		pr_err("%s: Unable to register %s input device\n", __func__, ts->input_dev->name);
		goto err_input_register_device_failed;
	}

    if (client->irq)
    {
		ret = request_irq(client->irq, focaltech_ts_irq_handler, IRQF_TRIGGER_FALLING, "ft5x0x_ts", ts);
		if (ret == 0)
			ts->use_irq = 1;
		else
		{
			dev_err(&client->dev, "request_irq failed\n");
			goto err_input_request_irq_failed;
		}
    }

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = focaltech_ts_early_suspend;
	ts->early_suspend.resume = focaltech_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	dir = proc_mkdir("touchscreen", NULL);
	refresh = create_proc_entry("ts_information", 0777, dir);
	if (refresh) {
		refresh->data		= NULL;
		refresh->read_proc  = proc_read_val;
		refresh->write_proc = proc_write_val;
	}

#if defined(CONFIG_TOUCHSCREEN_FOCALTECH_USBNOTIFY)
	focaltech_register_ts_notifier(&ts_notifier);
#endif

#if defined(CONFIG_TOUCHSCREEN_FOCALTECH_FW)
	ret = focaltech_fwupdate_init(client);
	if ( ret < 0 )
		pr_err("%s: firmware update initialization failed!\n ",__func__);
#endif

	pr_info("%s: Start touchscreen %s in %s mode\n", 
		__func__, ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");
	return 0;

err_input_request_irq_failed:
err_input_register_device_failed:
	input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
	destroy_workqueue(focaltech_wq);
err_create_singlethread:
err_detect_failed:
	if ( ts->gpio_init )
		ts->gpio_init(0);
err_power_failed:
	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
	pr_info("%s exit\n",__func__);
	return ret;
}

static int focaltech_ts_remove(struct i2c_client *client)
{
	struct focaltech_ts_data *ts = i2c_get_clientdata(client);

#if defined(CONFIG_TOUCHSCREEN_FOCALTECH_FW)
	focaltech_fwupdate_deinit(client);
#endif

	unregister_early_suspend(&ts->early_suspend);
	if (ts->use_irq)
		free_irq(client->irq, ts);
	else
		hrtimer_cancel(&ts->timer);
	input_unregister_device(ts->input_dev);

	if (ts->power) ts->power(0);
	if (ts->gpio_init) ts->gpio_init(0);

	kfree(ts);
	return 0;
}




static const struct i2c_device_id focaltech_ts_id[] = {
	{ "ft5x0x_ts", 0 },
	{ }
};

static struct i2c_driver focaltech_ts_driver = {
	.probe		= focaltech_ts_probe,
	.remove		= focaltech_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= focaltech_ts_suspend,
	.resume		= focaltech_ts_resume,
#endif
	.id_table	= focaltech_ts_id,
	.driver 	= {
		.name	= "ft5x0x_ts",
	},
};

static int __devinit focaltech_ts_init(void)
{
	return i2c_add_driver(&focaltech_ts_driver);
}

static void __exit focaltech_ts_exit(void)
{
	i2c_del_driver(&focaltech_ts_driver);
	if (focaltech_wq)
		destroy_workqueue(focaltech_wq);
}

module_init(focaltech_ts_init);
module_exit(focaltech_ts_exit);

MODULE_DESCRIPTION("Fts Touchscreen Driver");
MODULE_LICENSE("GPL");
