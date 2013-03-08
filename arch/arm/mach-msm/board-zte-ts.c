#include <linux/i2c.h>
#include <linux/i2c/sx150x.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <asm/mach-types.h>
#include <mach/msm_iomap.h>
#include <mach/board.h>
#include <mach/irqs-7xxx.h>
#include "devices-msm7x2xa.h"
#include "board-msm7627a.h"

#ifdef CONFIG_TOUCHSCREEN_MXT224
//#include <linux/atmel_maxtouch.h>
#include <linux/input/atmel_qt602240.h>
//extern struct atmel_i2c_platform_data atmel_data;
#endif
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS
#include <linux/input/synaptics_rmi.h> 
#endif
#ifdef CONFIG_TOUCHSCREEN_FOCALTECH
#include <linux/input/focaltech_ft5x0x.h>
#endif

//
//  touchscreen gpio definition
//
#define GPIO_7X27A_TS_IRQ		82
#define GPIO_7X27A_TS_EN		13
#define GPIO_7X27A_TS_RST		12

//
//	touchscreen firmware file name
//
#define FTC_FW_NAME "Ver11_20120409_N880E_9980_ID0x57_ATLAS40_app.bin"
#define SYN_FW_NAME "ATLAS40_PR1115996-s2202_Truly_32323038.img"
#define ATM_FW_NAME ""

//
//	touchscreen virtual key definition
//
#ifdef CONFIG_TOUCHSCREEN_VIRTUAL_KEYS
#define CAP_TS_VKEY_SYNAPTICS "virtualkeys.syna-touchscreen"
#define CAP_TS_VKEY_ATMEL "virtualkeys.atmel-touchscreen"
#define CAP_TS_VKEY_FTS "virtualkeys.Fts-touchscreen"

#define SYNAPTICS_MAX_Y_POSITION	1747
static ssize_t cap_ts_vkeys_show(struct device *dev,	struct device_attribute *attr, char *buf)
{
	//printk("%s, %s\n",__func__,attr->attr.name);
	return sprintf(
		buf,__stringify(EV_KEY) ":" __stringify(KEY_MENU) ":45:850:100:60"
		":" __stringify(EV_KEY) ":" __stringify(KEY_HOME) ":175:850:100:60"
		":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":306:850:100:60"
		":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":437:850:100:60"
		"\n");
}

static struct device_attribute cap_ts_device_attr[] = {
#if defined(CONFIG_TOUCHSCREEN_FOCALTECH)
	{
		.attr = {
			.name = CAP_TS_VKEY_FTS,
			.mode = S_IRUGO,
		},
		.show	= &cap_ts_vkeys_show,
		.store	= NULL,
	},
#endif
#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS)
{
	.attr = {
		.name = CAP_TS_VKEY_SYNAPTICS,
		.mode = S_IRUGO,
	},
	.show	= &cap_ts_vkeys_show,
	.store	= NULL,
},
#endif
#if defined(CONFIG_TOUCHSCREEN_MXT224)
	{
		.attr = {
			.name = CAP_TS_VKEY_ATMEL,
			.mode = S_IRUGO,
		},
		.show	= &cap_ts_vkeys_show,
		.store	= NULL,
	},
#endif
};

struct kobject *android_touch_kobj;
static int cap_ts_vkeys_init(void)
{
	int rc,i;
	struct kobject * cap_ts_properties_kobj=NULL;

	cap_ts_properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (cap_ts_properties_kobj == NULL) {
		printk("%s: subsystem_register failed\n", __func__);
		rc = -ENOMEM;
		return rc;
	}
	android_touch_kobj = cap_ts_properties_kobj;

	for ( i=0; i < ARRAY_SIZE(cap_ts_device_attr); i++ ){
		rc = sysfs_create_file(cap_ts_properties_kobj, &cap_ts_device_attr[i].attr);
		if (rc) {
			printk("%s: sysfs_create_file failed\n", __func__);
			return rc;
		}
	}

	return 0;
}
#endif

static void touchscreen_irqwake( bool wake )
{
	//suspend 
	if ( wake == 0 )
	{
		gpio_direction_output(GPIO_7X27A_TS_IRQ,1);
	}

	// resume
	if ( wake == 1)
	{
		gpio_set_value(GPIO_7X27A_TS_IRQ,0);
		msleep(3);
		gpio_set_value(GPIO_7X27A_TS_IRQ,1);
		msleep(220);
		gpio_direction_input(GPIO_7X27A_TS_IRQ);
	}

	return;
}

static void touchscreen_reset(void)
{
	gpio_direction_output(GPIO_7X27A_TS_RST, 1);

	return;
}

static void touchscreen_power(int on_off)
{
	if ( on_off == 1 )
		gpio_direction_output(GPIO_7X27A_TS_EN, 1);

	if ( on_off == 0 )
		gpio_direction_output(GPIO_7X27A_TS_EN,0);

	return;
}

static int touchscreen_gpio_init(int flag)
{
	int ret = 0;

	//init
	if ( flag == 1 )
	{
		ret = gpio_request(GPIO_7X27A_TS_EN, "touch voltage");
		if ( ret ){
			pr_err("%s, gpio %d request failed!\n", __func__,GPIO_7X27A_TS_EN);
			return -1;
		}

		ret = gpio_request(GPIO_7X27A_TS_RST, "touch voltage");
		if (ret){
			pr_err("%s: gpio %d request is error!\n", __func__, GPIO_7X27A_TS_RST);
			return -1;
		}

		ret = gpio_request(GPIO_7X27A_TS_IRQ, "touch voltage");
		if (ret){
			pr_err("%s: gpio %d request is error!\n", __func__, GPIO_7X27A_TS_IRQ);
			return -1;
		}
	}

	//deinit
	if ( flag == 0)
	{
		gpio_free(GPIO_7X27A_TS_EN);
		gpio_free(GPIO_7X27A_TS_IRQ);
		gpio_free(GPIO_7X27A_TS_RST);
	}

	return 0;

}

//
// i2c device definition
//

#if defined (CONFIG_TOUCHSCREEN_SYNAPTICS)
static struct synaptics_rmi_data synaptics_ts_data = {
	.gpio_init = touchscreen_gpio_init,
	.power	= touchscreen_power,
	.reset	= touchscreen_reset,
	.irqwake = touchscreen_irqwake,
	.max_y_position = SYNAPTICS_MAX_Y_POSITION,	// 0 - no vkey, do nothing
	.fwfile = SYN_FW_NAME,
};
#endif

#if defined (CONFIG_TOUCHSCREEN_FOCALTECH)
static struct focaltech_ts_platform_data focaltech_ts_data = {
	.gpio_init = touchscreen_gpio_init,
	.power	= touchscreen_power,
	.reset	= touchscreen_reset,
	.irqwake = touchscreen_irqwake,
	.fwfile	 = FTC_FW_NAME,
};
#endif

#if defined (CONFIG_TOUCHSCREEN_MXT224)
static struct atmel_platform_data atmel_ts_data = {
	.gpio_init = touchscreen_gpio_init,
	.power	= touchscreen_power,
	.reset	= touchscreen_reset,
	.irqwake = touchscreen_irqwake,
	.fwfile	 = ATM_FW_NAME,
};
#endif

static struct i2c_board_info i2c_touch_devices[] = {
#ifdef CONFIG_TOUCHSCREEN_FOCALTECH	
	{				
		I2C_BOARD_INFO("ft5x0x_ts", 0x3E ),
		.irq = MSM_GPIO_TO_INT(GPIO_7X27A_TS_IRQ),
		.platform_data = &focaltech_ts_data,
	},	
#endif
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS
	{
		I2C_BOARD_INFO("syna-touchscreen", 0x22 ),
		.irq = MSM_GPIO_TO_INT(GPIO_7X27A_TS_IRQ),
		.platform_data = &synaptics_ts_data,
	},
#endif
#ifdef CONFIG_TOUCHSCREEN_MXT224
	{    
		I2C_BOARD_INFO("atmel_qt602240", 0x4a ),
		.platform_data = &atmel_ts_data,
		.irq = MSM_GPIO_TO_INT(GPIO_7X27A_TS_IRQ),
	},   
#endif
};

void __init msm7x27a_ts_init(void)
{

#ifdef CONFIG_TOUCHSCREEN_VIRTUAL_KEYS
	cap_ts_vkeys_init();
#endif

	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
			i2c_touch_devices,
			ARRAY_SIZE(i2c_touch_devices));

	return ;
}

