/*
 * Gadget Driver for Android
 *
 * Copyright (C) 2008 Google, Inc.
 * Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
 * Author: Mike Lockwood <lockwood@android.com>
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
 */

/* #define DEBUG */
/* #define VERBOSE_DEBUG */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/utsname.h>
#include <linux/platform_device.h>

#include <linux/usb/ch9.h>
#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>
#include <linux/usb/android.h>

#include "gadget_chips.h"

/*
 * Kbuild is not very cooperative with respect to linking separately
 * compiled library objects into one module.  So for now we won't use
 * separate compilation ... ensuring init/exit sections work to shrink
 * the runtime footprint, and giving us at least some parts of what
 * a "gcc --combine ... part1.c part2.c part3.c ... " build would.
 */
#include "usbstring.c"
#include "config.c"
#include "epautoconf.c"
#include "composite.c"

#include "f_diag.c"
#include "f_rmnet_smd.c"
#include "f_rmnet_sdio.c"
#include "f_rmnet_smd_sdio.c"
#include "f_rmnet.c"
#include "f_mass_storage.c"
#include "u_serial.c"
#include "u_sdio.c"
#include "u_smd.c"
#include "u_bam.c"
#include "u_rmnet_ctrl_smd.c"
#include "u_ctrl_hsic.c"
#include "u_data_hsic.c"
#include "f_serial.c"
#include "f_acm.c"
#include "f_adb.c"
#include "f_ccid.c"
#include "f_mtp.c"
#include "f_accessory.c"
#define USB_ETH_RNDIS y
#include "f_rndis.c"
#include "rndis.c"
#include "u_ether.c"

MODULE_AUTHOR("Mike Lockwood");
MODULE_DESCRIPTION("Android Composite USB Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

static const char longname[] = "Gadget Android";

/* Default vendor and product IDs, overridden by userspace */
#define VENDOR_ID		0x18D1
#define PRODUCT_ID		0x0001

struct android_usb_function {
	char *name;
	void *config;

	struct device *dev;
	char *dev_name;
	struct device_attribute **attributes;

	/* for android_dev.enabled_functions */
	struct list_head enabled_list;

	/* Optional: initialization during gadget bind */
	int (*init)(struct android_usb_function *, struct usb_composite_dev *);
	/* Optional: cleanup during gadget unbind */
	void (*cleanup)(struct android_usb_function *);

	int (*bind_config)(struct android_usb_function *, struct usb_configuration *);

	/* Optional: called when the configuration is removed */
	void (*unbind_config)(struct android_usb_function *, struct usb_configuration *);
	/* Optional: handle ctrl requests before the device is configured */
	int (*ctrlrequest)(struct android_usb_function *,
					struct usb_composite_dev *,
					const struct usb_ctrlrequest *);
};
/*wangzy 120201*/
#if 1
/*COMMON*/
#define PRODUCT_ID_DIAG_MODEM_NMEA_MS_ADB          		0x1350
#define   PRODUCT_ID_ALL_INTERFACE    PRODUCT_ID_DIAG_MODEM_NMEA_MS_ADB
#define PRODUCT_ID_MS_ADB                      			0x1351
#define PRODUCT_ID_ADB                             		0x1352
#define PRODUCT_ID_MS                               	0x1353
#define PRODUCT_ID_MODEM_MS_ADB         				0x1354
#define PRODUCT_ID_MODEM_MS                 			0x1355
#define PRODUCT_ID_MS_CDROM                 			0x0083
#define PRODUCT_ID_DIAG_NMEA_MODEM   					0x0111
#define PRODUCT_ID_DIAG                           		0x0112	

/*froyo RNDIS*/
#define PRODUCT_ID_RNDIS_MS                 			0x1364
#define PRODUCT_ID_RNDIS_MS_ADB             			0x1364
#define PRODUCT_ID_RNDIS             					0x1365
#define PRODUCT_ID_RNDIS_ADB							0x1373

/*n600 pcui*/
#define PRODUCT_ID_DIAG_MODEM_NEMA_MS_ADB_AT   			0x1366
#define PRODUCT_ID_DIAG_MODEM_NEMA_MS_AT   				0x1367

/*DELL project*/
#define PRODUCT_ID_DIAG_MODEM_NMEA_MS_ADB_DELL          0x1368
#define PRODUCT_ID_MS_ADB_DELL                      	0x1369
#define PRODUCT_ID_MS_DELL                              0x1370
#define PRODUCT_ID_MODEM_MS_ADB_DELL         			0x1371
#define PRODUCT_ID_MODEM_MS_DELL                 		0x1372
#endif
#if 0
//keep it for zero cd implemented in kernel
struct usb_ex_work
{
	struct workqueue_struct *workqueue;
	int    enable_switch;
	int    enable_linux_switch;
	int switch_pid;
	int has_switch;
	int cur_pid;
	int linux_pid;
	struct delayed_work switch_work;
	struct delayed_work linux_switch_work;
	struct delayed_work plug_work;
	spinlock_t lock;
	struct wake_lock	wlock;
};

struct usb_ex_work global_usbwork = {0};
static int create_usb_work_queue(void);
static int destroy_usb_work_queue(void);
static void usb_plug_work(struct work_struct *w);
static void usb_switch_work(struct work_struct *w);
static void usb_switch_os_work(struct work_struct *w);
struct android_usb_product *android_validate_product_id(unsigned short pid);
static void clear_switch_flag(void);
#endif

static int usb_cdrom_is_enable(void);
static int is_cdrom_enabled_after_switch(void);
/* ruanmeisi for ftm */
enum usb_opt_nv_item
{
	NV_BACK_LIGHT_I=77,//nv77 used for config/store usb mode
	NV_FTM_MODE_I = 453// FTM mode
};
enum usb_opt_nv_type
{
	NV_READ=0,
        NV_WRITE
};
int msm_hsusb_get_set_usb_conf_nv_value(uint32_t nv_item,uint32_t value,uint32_t is_write);
int get_ftm_from_tag(void);
static int ftm_mode = 0;
static int got_nv_tag = 0;
static int got_nv_flag = 1;
static int is_ftm_mode(void)
{
	return !!ftm_mode;
}
static void set_ftm_mode(int i)
{
	ftm_mode = i;
	return ;
}
static int is_pid_configed_from_nv(void)
{
	pr_debug("is_pid_configed_from_nv %d\n",!!got_nv_tag);
	return !!got_nv_tag;
}
static void set_pid_from_nv(int i)
{
	got_nv_tag = i;
	return ;
}
static int is_nvflag_enable(void)
{
	pr_debug("is_nvflag_enable %d\n",!!got_nv_flag);
        return !!got_nv_flag;
}
static void set_nvflag(int i)
{
        got_nv_flag = i;
        return ;
}


#define NV_WRITE_SUCCESS 10
static int get_nv(void)
{
	return msm_hsusb_get_set_usb_conf_nv_value(NV_BACK_LIGHT_I,0,NV_READ);
}
static int set_nv(int nv)
{
	int r = msm_hsusb_get_set_usb_conf_nv_value(NV_BACK_LIGHT_I,nv,NV_WRITE);
	return (r == NV_WRITE_SUCCESS)? 0:-1;
}
static int config_pid_from_nv(void)
{
	int i = 0;
	printk("usb: %s\n", __FUNCTION__);
	if (is_ftm_mode()) {
		return 0;
	}
	i = get_nv();
	/*only if nv77 == 4*/
	set_pid_from_nv(i == 4? 1:0);		
	return is_pid_configed_from_nv();
}
static int config_ftm_from_tag(void)
{
	printk("usb: %s, %d\n", __FUNCTION__, __LINE__);
	if (is_ftm_mode()) {
		return 0;
	}

	set_ftm_mode(get_ftm_from_tag());
	printk("usb: %s, %d: ftm_mode %s\n",
	       __FUNCTION__, __LINE__,
	       is_ftm_mode()?"enable":"disable");
	return 0;
}

static ssize_t msm_hsusb_set_pidnv(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf, size_t size)
{
	int value;
	sscanf(buf, "%d", &value);
	set_nv(value);
	return size;
}

static ssize_t msm_hsusb_show_pidnv(struct device *dev,
                                    struct device_attribute *attr,
                                    char *buf)
{
	int i = 0;
	i = scnprintf(buf, PAGE_SIZE, "nv %d\n", get_nv());

	return i;
}

static ssize_t show_ftm_tag(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	int i = 0;
	i = scnprintf(buf, PAGE_SIZE, "%s\n", is_ftm_mode()?"enable":"disable");
	return i;
}

static ssize_t show_nv_tag(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	int i = 0;
	config_pid_from_nv();
	i = scnprintf(buf, PAGE_SIZE, "%s\n", (is_pid_configed_from_nv() && is_nvflag_enable())
		                                                                ?"enable":"disable");
	return i;
}

static char nv_flag[32];
static ssize_t nvflag_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf, size_t size)
{
	strlcpy(nv_flag, buf, sizeof(nv_flag));
	set_nvflag(!strncmp(nv_flag,"enable",strlen("enable")));
	return size;
}

static ssize_t nvflag_show(struct device *dev,
                                    struct device_attribute *attr,
                                    char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", nv_flag);
}

/*end*/
struct android_dev {
	struct android_usb_function **functions;
	struct list_head enabled_functions;
	struct usb_composite_dev *cdev;
	struct device *dev;
	struct android_usb_platform_data *pdata;

	bool enabled;
	bool connected;
	bool sw_connected;
	struct mutex lock; 	/*wangzy 120201*/
	struct work_struct work;
};

static struct class *android_class;
static struct android_dev *_android_dev;
static int android_bind_config(struct usb_configuration *c);
static void android_unbind_config(struct usb_configuration *c);

/* string IDs are assigned dynamically */
#define STRING_MANUFACTURER_IDX		0
#define STRING_PRODUCT_IDX		1
#define STRING_SERIAL_IDX		2

static char manufacturer_string[256];
static char product_string[256];
static char serial_string[256];

/* String Table */
static struct usb_string strings_dev[] = {
	[STRING_MANUFACTURER_IDX].s = manufacturer_string,
	[STRING_PRODUCT_IDX].s = product_string,
	[STRING_SERIAL_IDX].s = serial_string,
	{  }			/* end of list */
};

static struct usb_gadget_strings stringtab_dev = {
	.language	= 0x0409,	/* en-us */
	.strings	= strings_dev,
};

static struct usb_gadget_strings *dev_strings[] = {
	&stringtab_dev,
	NULL,
};

static struct usb_device_descriptor device_desc = {
	.bLength              = sizeof(device_desc),
	.bDescriptorType      = USB_DT_DEVICE,
	.bcdUSB               = __constant_cpu_to_le16(0x0200),
	.bDeviceClass         = USB_CLASS_PER_INTERFACE,
	.idVendor             = __constant_cpu_to_le16(VENDOR_ID),
	.idProduct            = __constant_cpu_to_le16(PRODUCT_ID),
	.bcdDevice            = __constant_cpu_to_le16(0xffff),
	.bNumConfigurations   = 1,
};

/*wangzy 120201*/
static void ftm_iSerialNumber_filter(struct usb_composite_dev *cdev)
{
	if (is_ftm_mode() || is_pid_configed_from_nv()) {
		strings_dev[STRING_SERIAL_IDX].id = 0;
		device_desc.iSerialNumber = 0;
		if (cdev)			
			cdev->desc.iSerialNumber = device_desc.iSerialNumber;
	}
}
struct usb_parameters {
	//char p_serialnumber[64]; //will be used in demand
	char cdrom_enable_after_switch;
};
struct usb_parameters zte_usb_parameters = {

	//.p_serialnumber = {"ZTE_andorid"},
	.cdrom_enable_after_switch = 0,
};

static int current_pid(void)
{
	return device_desc.idProduct;
}
/*end*/
static struct usb_configuration android_config_driver = {
	.label		= "android",
	.unbind		= android_unbind_config,
	.bConfigurationValue = 1,
	.bmAttributes	= USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER,
	.bMaxPower	= 0xFA, /* 500ma */
};

enum android_device_state {
	USB_DISCONNECTED,
	USB_CONNECTED,
	USB_CONFIGURED,
};

static void android_work(struct work_struct *data)
{
	struct android_dev *dev = container_of(data, struct android_dev, work);
	struct usb_composite_dev *cdev = dev->cdev;
	char *disconnected[2] = { "USB_STATE=DISCONNECTED", NULL };
	char *connected[2]    = { "USB_STATE=CONNECTED", NULL };
	char *configured[2]   = { "USB_STATE=CONFIGURED", NULL };
	char **uevent_envp = NULL;
	static enum android_device_state last_uevent, next_state;
	unsigned long flags;

	spin_lock_irqsave(&cdev->lock, flags);
	if (cdev->config) {
		uevent_envp = configured;
		next_state = USB_CONFIGURED;
	} else if (dev->connected != dev->sw_connected) {
		uevent_envp = dev->connected ? connected : disconnected;
		next_state = dev->connected ? USB_CONNECTED : USB_DISCONNECTED;
	}
	dev->sw_connected = dev->connected;
	spin_unlock_irqrestore(&cdev->lock, flags);

	if (uevent_envp) {
		/*
		 * Some userspace modules, e.g. MTP, work correctly only if
		 * CONFIGURED uevent is preceded by DISCONNECT uevent.
		 * Check if we missed sending out a DISCONNECT uevent. This can
		 * happen if host PC resets and configures device really quick.
		 */
		if (((uevent_envp == connected) &&
		      (last_uevent != USB_DISCONNECTED)) ||
		    ((uevent_envp == configured) &&
		      (last_uevent == USB_CONFIGURED))) {
			pr_info("%s: sent missed DISCONNECT event\n", __func__);
			kobject_uevent_env(&dev->dev->kobj, KOBJ_CHANGE,
								disconnected);
			msleep(20);
		}
		/*
		 * Before sending out CONFIGURED uevent give function drivers
		 * a chance to wakeup userspace threads and notify disconnect
		 */
		if (uevent_envp == configured)
			msleep(50);

		kobject_uevent_env(&dev->dev->kobj, KOBJ_CHANGE, uevent_envp);
		last_uevent = next_state;
		pr_info("%s: sent uevent %s\n", __func__, uevent_envp[0]);
	} else {
		pr_info("%s: did not send uevent (%d %d %p)\n", __func__,
			 dev->connected, dev->sw_connected, cdev->config);
	}
}


/*-------------------------------------------------------------------------*/
/* Supported functions initialization */

/* RMNET_SMD */
static int rmnet_smd_function_bind_config(struct android_usb_function *f,
					  struct usb_configuration *c)
{
	return rmnet_smd_bind_config(c);
}

static struct android_usb_function rmnet_smd_function = {
	.name		= "rmnet_smd",
	.bind_config	= rmnet_smd_function_bind_config,
};

/* RMNET_SDIO */
static int rmnet_sdio_function_bind_config(struct android_usb_function *f,
					  struct usb_configuration *c)
{
	return rmnet_sdio_function_add(c);
}

static struct android_usb_function rmnet_sdio_function = {
	.name		= "rmnet_sdio",
	.bind_config	= rmnet_sdio_function_bind_config,
};

/* RMNET_SMD_SDIO */
static int rmnet_smd_sdio_function_init(struct android_usb_function *f,
				 struct usb_composite_dev *cdev)
{
	return rmnet_smd_sdio_init();
}

static void rmnet_smd_sdio_function_cleanup(struct android_usb_function *f)
{
	rmnet_smd_sdio_cleanup();
}

static int rmnet_smd_sdio_bind_config(struct android_usb_function *f,
					  struct usb_configuration *c)
{
	return rmnet_smd_sdio_function_add(c);
}

static struct device_attribute *rmnet_smd_sdio_attributes[] = {
					&dev_attr_transport, NULL };

static struct android_usb_function rmnet_smd_sdio_function = {
	.name		= "rmnet_smd_sdio",
	.init		= rmnet_smd_sdio_function_init,
	.cleanup	= rmnet_smd_sdio_function_cleanup,
	.bind_config	= rmnet_smd_sdio_bind_config,
	.attributes	= rmnet_smd_sdio_attributes,
};

/*rmnet transport string format(per port):"ctrl0,data0,ctrl1,data1..." */
#define MAX_XPORT_STR_LEN 50
static char rmnet_transports[MAX_XPORT_STR_LEN];

static void rmnet_function_cleanup(struct android_usb_function *f)
{
	frmnet_cleanup();
}

static int rmnet_function_bind_config(struct android_usb_function *f,
					 struct usb_configuration *c)
{
	int i;
	int err = 0;
	char *ctrl_name;
	char *data_name;
	char buf[MAX_XPORT_STR_LEN], *b;
	static int rmnet_initialized, ports;

	if (!rmnet_initialized) {
		rmnet_initialized = 1;
		strlcpy(buf, rmnet_transports, sizeof(buf));
		b = strim(buf);
		while (b) {
			ctrl_name = strsep(&b, ",");
			data_name = strsep(&b, ",");
			if (ctrl_name && data_name) {
				err = frmnet_init_port(ctrl_name, data_name);
				if (err) {
					pr_err("rmnet: Cannot open ctrl port:"
						"'%s' data port:'%s'\n",
						ctrl_name, data_name);
					goto out;
				}
				ports++;
			}
		}

		err = rmnet_gport_setup();
		if (err) {
			pr_err("rmnet: Cannot setup transports");
			goto out;
		}
	}

	for (i = 0; i < ports; i++) {
		err = frmnet_bind_config(c, i);
		if (err) {
			pr_err("Could not bind rmnet%u config\n", i);
			break;
		}
	}
out:
	return err;
}

static ssize_t rmnet_transports_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", rmnet_transports);
}

static ssize_t rmnet_transports_store(
		struct device *device, struct device_attribute *attr,
		const char *buff, size_t size)
{
	strlcpy(rmnet_transports, buff, sizeof(rmnet_transports));

	return size;
}

static struct device_attribute dev_attr_rmnet_transports =
					__ATTR(transports, S_IRUGO | S_IWUSR,
						rmnet_transports_show,
						rmnet_transports_store);
static struct device_attribute *rmnet_function_attributes[] = {
					&dev_attr_rmnet_transports,
					NULL };

static struct android_usb_function rmnet_function = {
	.name		= "rmnet",
	.cleanup	= rmnet_function_cleanup,
	.bind_config	= rmnet_function_bind_config,
	.attributes	= rmnet_function_attributes,
};

/* DIAG */
static char diag_clients[32];	    /*enabled DIAG clients- "diag[,diag_mdm]" */
static ssize_t clients_store(
		struct device *device, struct device_attribute *attr,
		const char *buff, size_t size)
{
	strlcpy(diag_clients, buff, sizeof(diag_clients));

	return size;
}

static DEVICE_ATTR(clients, S_IWUSR, NULL, clients_store);
static struct device_attribute *diag_function_attributes[] =
					 { &dev_attr_clients, NULL };

static int diag_function_init(struct android_usb_function *f,
				 struct usb_composite_dev *cdev)
{
	return diag_setup();
}

static void diag_function_cleanup(struct android_usb_function *f)
{
	diag_cleanup();
}

static int diag_function_bind_config(struct android_usb_function *f,
					struct usb_configuration *c)
{
	char *name;
	char buf[32], *b;
	int once = 0, err = -1;
	int (*notify)(uint32_t, const char *);

	strlcpy(buf, diag_clients, sizeof(buf));
	b = strim(buf);

	while (b) {
		notify = NULL;
		name = strsep(&b, ",");
		/* Allow only first diag channel to update pid and serial no */
		if (_android_dev->pdata && !once++)
			notify = _android_dev->pdata->update_pid_and_serial_num;

		if (name) {
			err = diag_function_add(c, name, notify);
			if (err)
				pr_err("diag: Cannot open channel '%s'", name);
		}
	}

	return err;
}

static struct android_usb_function diag_function = {
	.name		= "diag",
	.init		= diag_function_init,
	.cleanup	= diag_function_cleanup,
	.bind_config	= diag_function_bind_config,
	.attributes	= diag_function_attributes,
};

/* SERIAL */
static char serial_transports[32];	/*enabled FSERIAL ports - "tty[,sdio]"*/
static ssize_t serial_transports_store(
		struct device *device, struct device_attribute *attr,
		const char *buff, size_t size)
{
	strlcpy(serial_transports, buff, sizeof(serial_transports));

	return size;
}
static DEVICE_ATTR(transports, S_IWUSR, NULL, serial_transports_store);
/*xingbeilei begin*/
static char max_serial_transports[32];      
static ssize_t max_serial_transports_store(
	struct device *device, struct device_attribute *attr,
	const char *buff, size_t size)
{
        strlcpy(max_serial_transports, buff, sizeof(max_serial_transports));

        return size;
}
static ssize_t max_serial_transports_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
        return snprintf(buf, PAGE_SIZE, "%s\n", max_serial_transports);
}
static DEVICE_ATTR(max_transports, S_IRUGO | S_IWUSR, 
		   max_serial_transports_show, max_serial_transports_store);

static ssize_t modem_enable_show(struct device *dev,
					  struct device_attribute *attr, char *buf)
{
        return snprintf(buf, PAGE_SIZE, "%d\n", gserial_ports[0].enable);
}
static DEVICE_ATTR(modem_enable, S_IRUGO, modem_enable_show, NULL);

static ssize_t nmea_enable_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
        return snprintf(buf, PAGE_SIZE, "%d\n", gserial_ports[1].enable);
}
static DEVICE_ATTR(nmea_enable, S_IRUGO, nmea_enable_show, NULL);

static ssize_t at_enable_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
        return snprintf(buf, PAGE_SIZE, "%d\n", gserial_ports[2].enable);
}
static DEVICE_ATTR(at_enable, S_IRUGO, at_enable_show, NULL);
/*xingbeilei end  */
static struct device_attribute *serial_function_attributes[] =
                                        { &dev_attr_transports,
					  &dev_attr_max_transports, 
					  &dev_attr_modem_enable,
					  &dev_attr_nmea_enable,
					  &dev_attr_at_enable, 
					  NULL };

static void serial_function_cleanup(struct android_usb_function *f)
{
	gserial_cleanup();
}

static int serial_function_bind_config(struct android_usb_function *f,
					struct usb_configuration *c)
{
	char *name;
	char buf[32], *b;
	int err = -1, i;
	static int serial_initialized = 0, ports = 0;

	printk(KERN_ERR"xbl:%s,%d\n",__FUNCTION__,__LINE__);
	if (serial_initialized)
		goto bind_config;

	printk(KERN_ERR"xbl:%s,%d\n",__FUNCTION__,__LINE__);
	serial_initialized = 1;
	strlcpy(buf, max_serial_transports, sizeof(buf));
	b = strim(buf);

	while (b) {
		name = strsep(&b, ",");

		if (name) {
			err = gserial_init_port(ports, name);
			if (err) {
				pr_err("serial: Cannot open port '%s'", name);
				goto out;
			}
			ports++;
		}
	}
	err = gport_setup(c);
	if (err) {
		pr_err("serial: Cannot setup transports");
		goto out;
	}

bind_config:
	//xingbeilei
	strlcpy(buf, serial_transports, sizeof(buf));
	ports = 0;
        b = strim(buf);

        while (b) {
                name = strsep(&b, ",");
                if (name) {
                        ports++;
                }
        }
	//end
	for (i = 0; i < ports; i++) { 
		err = gser_bind_config(c, i);
		if (err) {
			pr_err("serial: bind_config failed for port %d", i);
			goto out;
		}
	}

out:
	return err;
}

static struct android_usb_function serial_function = {
	.name		= "serial",
	.cleanup	= serial_function_cleanup,
	.bind_config	= serial_function_bind_config,
	.attributes	= serial_function_attributes,
};

/* ACM */
static char acm_transports[32];	/*enabled ACM ports - "tty[,sdio]"*/
static ssize_t acm_transports_store(
		struct device *device, struct device_attribute *attr,
		const char *buff, size_t size)
{
	strlcpy(acm_transports, buff, sizeof(acm_transports));

	return size;
}

static DEVICE_ATTR(acm_transports, S_IWUSR, NULL, acm_transports_store);
static struct device_attribute *acm_function_attributes[] = {
		&dev_attr_acm_transports, NULL };

static void acm_function_cleanup(struct android_usb_function *f)
{
	gserial_cleanup();
}

static int acm_function_bind_config(struct android_usb_function *f,
					struct usb_configuration *c)
{
	char *name;
	char buf[32], *b;
	int err = -1, i;
	static int acm_initialized, ports;

	if (acm_initialized)
		goto bind_config;

	acm_initialized = 1;
	strlcpy(buf, acm_transports, sizeof(buf));
	b = strim(buf);

	while (b) {
		name = strsep(&b, ",");

		if (name) {
			err = acm_init_port(ports, name);
			if (err) {
				pr_err("acm: Cannot open port '%s'", name);
				goto out;
			}
			ports++;
		}
	}
	err = acm_port_setup(c);
	if (err) {
		pr_err("acm: Cannot setup transports");
		goto out;
	}

bind_config:
	for (i = 0; i < ports; i++) {
		err = acm_bind_config(c, i);
		if (err) {
			pr_err("acm: bind_config failed for port %d", i);
			goto out;
		}
	}

out:
	return err;
}
static struct android_usb_function acm_function = {
	.name		= "acm",
	.cleanup	= acm_function_cleanup,
	.bind_config	= acm_function_bind_config,
	.attributes	= acm_function_attributes,
};

/* ADB */
static int adb_function_init(struct android_usb_function *f, struct usb_composite_dev *cdev)
{
	return adb_setup();
}

static void adb_function_cleanup(struct android_usb_function *f)
{
	adb_cleanup();
}

static int adb_function_bind_config(struct android_usb_function *f, struct usb_configuration *c)
{
	return adb_bind_config(c);
}

static struct android_usb_function adb_function = {
	.name		= "adb",
	.init		= adb_function_init,
	.cleanup	= adb_function_cleanup,
	.bind_config	= adb_function_bind_config,
};

/* CCID */
static int ccid_function_init(struct android_usb_function *f,
					struct usb_composite_dev *cdev)
{
	return ccid_setup();
}

static void ccid_function_cleanup(struct android_usb_function *f)
{
	ccid_cleanup();
}

static int ccid_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	return ccid_bind_config(c);
}

static struct android_usb_function ccid_function = {
	.name		= "ccid",
	.init		= ccid_function_init,
	.cleanup	= ccid_function_cleanup,
	.bind_config	= ccid_function_bind_config,
};

static int mtp_function_init(struct android_usb_function *f, struct usb_composite_dev *cdev)
{
	return mtp_setup();
}

static void mtp_function_cleanup(struct android_usb_function *f)
{
	mtp_cleanup();
}

static int mtp_function_bind_config(struct android_usb_function *f, struct usb_configuration *c)
{
	return mtp_bind_config(c, false);
}

static int ptp_function_init(struct android_usb_function *f, struct usb_composite_dev *cdev)
{
	/* nothing to do - initialization is handled by mtp_function_init */
	return 0;
}

static void ptp_function_cleanup(struct android_usb_function *f)
{
	/* nothing to do - cleanup is handled by mtp_function_cleanup */
}

static int ptp_function_bind_config(struct android_usb_function *f, struct usb_configuration *c)
{
	return mtp_bind_config(c, true);
}

static int mtp_function_ctrlrequest(struct android_usb_function *f,
						struct usb_composite_dev *cdev,
						const struct usb_ctrlrequest *c)
{
	return mtp_ctrlrequest(cdev, c);
}

static struct android_usb_function mtp_function = {
	.name		= "mtp",
	.init		= mtp_function_init,
	.cleanup	= mtp_function_cleanup,
	.bind_config	= mtp_function_bind_config,
	.ctrlrequest	= mtp_function_ctrlrequest,
};

/* PTP function is same as MTP with slightly different interface descriptor */
static struct android_usb_function ptp_function = {
	.name		= "ptp",
	.init		= ptp_function_init,
	.cleanup	= ptp_function_cleanup,
	.bind_config	= ptp_function_bind_config,
};


struct rndis_function_config {
	u8      ethaddr[ETH_ALEN];
	u32     vendorID;
	char	manufacturer[256];
	bool	wceis;
};

static int rndis_function_init(struct android_usb_function *f, struct usb_composite_dev *cdev)
{
	f->config = kzalloc(sizeof(struct rndis_function_config), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;
	return 0;
}

static void rndis_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
	f->config = NULL;
}

static int rndis_function_bind_config(struct android_usb_function *f,
					struct usb_configuration *c)
{
	int ret;
	struct rndis_function_config *rndis = f->config;

	if (!rndis) {
		pr_err("%s: rndis_pdata\n", __func__);
		return -1;
	}

	pr_info("%s MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", __func__,
		rndis->ethaddr[0], rndis->ethaddr[1], rndis->ethaddr[2],
		rndis->ethaddr[3], rndis->ethaddr[4], rndis->ethaddr[5]);

	ret = gether_setup_name(c->cdev->gadget, rndis->ethaddr, "rndis");
	if (ret) {
		pr_err("%s: gether_setup failed\n", __func__);
		return ret;
	}

	if (rndis->wceis) {
		/* "Wireless" RNDIS; auto-detected by Windows */
		rndis_iad_descriptor.bFunctionClass =
						USB_CLASS_WIRELESS_CONTROLLER;
		rndis_iad_descriptor.bFunctionSubClass = 0x01;
		rndis_iad_descriptor.bFunctionProtocol = 0x03;
		rndis_control_intf.bInterfaceClass =
						USB_CLASS_WIRELESS_CONTROLLER;
		rndis_control_intf.bInterfaceSubClass =	 0x01;
		rndis_control_intf.bInterfaceProtocol =	 0x03;
	}

	return rndis_bind_config(c, rndis->ethaddr, rndis->vendorID,
				    rndis->manufacturer);
}

static void rndis_function_unbind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	gether_cleanup();
}

static ssize_t rndis_manufacturer_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%s\n", config->manufacturer);
}

static ssize_t rndis_manufacturer_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;

	if (size >= sizeof(config->manufacturer))
		return -EINVAL;
	if (sscanf(buf, "%255s", config->manufacturer) == 1)
		return size;
	return -1;
}

static DEVICE_ATTR(manufacturer, S_IRUGO | S_IWUSR, rndis_manufacturer_show,
						    rndis_manufacturer_store);

static ssize_t rndis_wceis_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%d\n", config->wceis);
}

static ssize_t rndis_wceis_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	int value;

	if (sscanf(buf, "%d", &value) == 1) {
		config->wceis = value;
		return size;
	}
	return -EINVAL;
}

static DEVICE_ATTR(wceis, S_IRUGO | S_IWUSR, rndis_wceis_show,
					     rndis_wceis_store);

static ssize_t rndis_ethaddr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *rndis = f->config;
	return snprintf(buf, PAGE_SIZE, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		rndis->ethaddr[0], rndis->ethaddr[1], rndis->ethaddr[2],
		rndis->ethaddr[3], rndis->ethaddr[4], rndis->ethaddr[5]);
}

static ssize_t rndis_ethaddr_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *rndis = f->config;

	if (sscanf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		    (int *)&rndis->ethaddr[0], (int *)&rndis->ethaddr[1],
		    (int *)&rndis->ethaddr[2], (int *)&rndis->ethaddr[3],
		    (int *)&rndis->ethaddr[4], (int *)&rndis->ethaddr[5]) == 6)
		return size;
	return -EINVAL;
}

static DEVICE_ATTR(ethaddr, S_IRUGO | S_IWUSR, rndis_ethaddr_show,
					       rndis_ethaddr_store);

static ssize_t rndis_vendorID_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%04x\n", config->vendorID);
}

static ssize_t rndis_vendorID_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	int value;

	if (sscanf(buf, "%04x", &value) == 1) {
		config->vendorID = value;
		return size;
	}
	return -EINVAL;
}

static DEVICE_ATTR(vendorID, S_IRUGO | S_IWUSR, rndis_vendorID_show,
						rndis_vendorID_store);

static struct device_attribute *rndis_function_attributes[] = {
	&dev_attr_manufacturer,
	&dev_attr_wceis,
	&dev_attr_ethaddr,
	&dev_attr_vendorID,
	NULL
};

static struct android_usb_function rndis_function = {
	.name		= "rndis",
	.init		= rndis_function_init,
	.cleanup	= rndis_function_cleanup,
	.bind_config	= rndis_function_bind_config,
	.unbind_config	= rndis_function_unbind_config,
	.attributes	= rndis_function_attributes,
};


struct mass_storage_function_config {
	struct fsg_config fsg;
	struct fsg_common *common;
};

static int mass_storage_function_init(struct android_usb_function *f,
					struct usb_composite_dev *cdev)
{
	struct mass_storage_function_config *config;
	struct fsg_common *common;
	int err;
	int i, nluns; 

	config = kzalloc(sizeof(struct mass_storage_function_config),
								GFP_KERNEL);
	if (!config)
		return -ENOMEM;

	/*wangzy 120201 increase nluns (1->2) for msc cdrom*/
	nluns = 2;
	nluns = (nluns > FSG_MAX_LUNS)? FSG_MAX_LUNS : nluns;
	config->fsg.nluns = nluns;
	for (i = 0; i < nluns; i++)
		config->fsg.luns[i].removable = 1;
	
	config->fsg.luns[0].cdrom = 0; 
	config->fsg.luns[1].cdrom = 1;
	/*end*/
	
	common = fsg_common_init(NULL, cdev, &config->fsg);
	if (IS_ERR(common)) {
		kfree(config);
		return PTR_ERR(common);
	}

	err = sysfs_create_link(&f->dev->kobj,
				&common->luns[0].dev.kobj,
				"lun");
	if (err) {
		fsg_common_release(&common->ref);
		kfree(config);
		return err;
	}

	config->common = common;
	f->config = config;
	return 0;
}

static void mass_storage_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
	f->config = NULL;
}

static int mass_storage_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	struct mass_storage_function_config *config = f->config;
	return fsg_bind_config(c->cdev, c, config->common);
}

static ssize_t mass_storage_inquiry_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct mass_storage_function_config *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%s\n", config->common->inquiry_string);
}

static ssize_t mass_storage_inquiry_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct mass_storage_function_config *config = f->config;
	if (size >= sizeof(config->common->inquiry_string))
		return -EINVAL;
	if (sscanf(buf, "%28s", config->common->inquiry_string) != 1)
		return -EINVAL;
	return size;
}

static DEVICE_ATTR(inquiry_string, S_IRUGO | S_IWUSR,
					mass_storage_inquiry_show,
					mass_storage_inquiry_store);

static struct device_attribute *mass_storage_function_attributes[] = {
	&dev_attr_inquiry_string,
	NULL
};

static struct android_usb_function mass_storage_function = {
	.name		= "mass_storage",
	.init		= mass_storage_function_init,
	.cleanup	= mass_storage_function_cleanup,
	.bind_config	= mass_storage_function_bind_config,
	.attributes	= mass_storage_function_attributes,
};


static int accessory_function_init(struct android_usb_function *f,
					struct usb_composite_dev *cdev)
{
	return acc_setup();
}

static void accessory_function_cleanup(struct android_usb_function *f)
{
	acc_cleanup();
}

static int accessory_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	return acc_bind_config(c);
}

static int accessory_function_ctrlrequest(struct android_usb_function *f,
						struct usb_composite_dev *cdev,
						const struct usb_ctrlrequest *c)
{
	return acc_ctrlrequest(cdev, c);
}

static struct android_usb_function accessory_function = {
	.name		= "accessory",
	.init		= accessory_function_init,
	.cleanup	= accessory_function_cleanup,
	.bind_config	= accessory_function_bind_config,
	.ctrlrequest	= accessory_function_ctrlrequest,
};


static struct android_usb_function *supported_functions[] = {
	&rmnet_smd_function,
	&rmnet_sdio_function,
	&rmnet_smd_sdio_function,
	&rmnet_function,
	&diag_function,
	&serial_function,
	&adb_function,
	&ccid_function,
	&acm_function,
	&mtp_function,
	&ptp_function,
	&rndis_function,
	&mass_storage_function,
	&accessory_function,
	NULL
};


static int android_init_functions(struct android_usb_function **functions,
				  struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;
	struct android_usb_function *f;
	struct device_attribute **attrs;
	struct device_attribute *attr;
	int err = 0;
	int index = 0;

	for (; (f = *functions++); index++) {
		f->dev_name = kasprintf(GFP_KERNEL, "f_%s", f->name);
		f->dev = device_create(android_class, dev->dev,
				MKDEV(0, index), f, f->dev_name);
		if (IS_ERR(f->dev)) {
			pr_err("%s: Failed to create dev %s", __func__,
							f->dev_name);
			err = PTR_ERR(f->dev);
			goto err_create;
		}

		if (f->init) {
			err = f->init(f, cdev);
			if (err) {
				pr_err("%s: Failed to init %s", __func__,
								f->name);
				goto err_out;
			}
		}

		attrs = f->attributes;
		if (attrs) {
			while ((attr = *attrs++) && !err)
				err = device_create_file(f->dev, attr);
		}
		if (err) {
			pr_err("%s: Failed to create function %s attributes",
					__func__, f->name);
			goto err_out;
		}
	}
	return 0;

err_out:
	device_destroy(android_class, f->dev->devt);
err_create:
	kfree(f->dev_name);
	return err;
}

static void android_cleanup_functions(struct android_usb_function **functions)
{
	struct android_usb_function *f;

	while (*functions) {
		f = *functions++;

		if (f->dev) {
			device_destroy(android_class, f->dev->devt);
			kfree(f->dev_name);
		}

		if (f->cleanup)
			f->cleanup(f);
	}
}

static int
android_bind_enabled_functions(struct android_dev *dev,
			       struct usb_configuration *c)
{
	struct android_usb_function *f;
	int ret;

	list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
		ret = f->bind_config(f, c);
		if (ret) {
			pr_err("%s: %s failed", __func__, f->name);
			return ret;
		}
		pr_err("usb %s: %s success", __func__, f->name);
	}
	return 0;
}

static void
android_unbind_enabled_functions(struct android_dev *dev,
			       struct usb_configuration *c)
{
	struct android_usb_function *f;

	list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
		if (f->unbind_config){
			pr_err("usb %s: %s", __func__, f->name);
			f->unbind_config(f, c);
			}
	}
}

static int android_enable_function(struct android_dev *dev, char *name)
{
	struct android_usb_function **functions = dev->functions;
	struct android_usb_function *f;
	while ((f = *functions++)) {
		if (!strcmp(name, f->name)) {
			list_add_tail(&f->enabled_list, &dev->enabled_functions);
			return 0;
		}
	}
	return -EINVAL;
}

/*-------------------------------------------------------------------------*/
/* /sys/class/android_usb/android%d/ interface */

static ssize_t remote_wakeup_show(struct device *pdev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n",
			!!(android_config_driver.bmAttributes &
				USB_CONFIG_ATT_WAKEUP));
}

static ssize_t remote_wakeup_store(struct device *pdev,
		struct device_attribute *attr, const char *buff, size_t size)
{
	int enable = 0;

	sscanf(buff, "%d", &enable);

	pr_debug("android_usb: %s remote wakeup\n",
			enable ? "enabling" : "disabling");

	if (enable)
		android_config_driver.bmAttributes |= USB_CONFIG_ATT_WAKEUP;
	else
		android_config_driver.bmAttributes &= ~USB_CONFIG_ATT_WAKEUP;

	return size;
}

static ssize_t
functions_show(struct device *pdev, struct device_attribute *attr, char *buf)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	struct android_usb_function *f;
	char *buff = buf;

	list_for_each_entry(f, &dev->enabled_functions, enabled_list)
		buff += snprintf(buff, PAGE_SIZE, "%s,", f->name);
	if (buff != buf)
		*(buff-1) = '\n';
	return buff - buf;
}

static ssize_t
functions_store(struct device *pdev, struct device_attribute *attr,
			       const char *buff, size_t size)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	char *name;
	char buf[256], *b;
	int err;

	INIT_LIST_HEAD(&dev->enabled_functions);

	strlcpy(buf, buff, sizeof(buf));
	b = strim(buf);

	while (b) {
		name = strsep(&b, ",");
		if (name) {
			err = android_enable_function(dev, name);
			if (err)
				pr_err("android_usb: Cannot enable '%s'", name);
		}
	}

	return size;
}

static ssize_t enable_show(struct device *pdev, struct device_attribute *attr,
			   char *buf)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	return snprintf(buf, PAGE_SIZE, "%d\n", dev->enabled);
}

static ssize_t enable_store(struct device *pdev, struct device_attribute *attr,
			    const char *buff, size_t size)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	struct usb_composite_dev *cdev = dev->cdev;
	int enabled = 0;

	sscanf(buff, "%d", &enabled);
	if (enabled && !dev->enabled) {
		/* update values in composite driver's copy of device descriptor */
		printk(KERN_ERR"usb:%s,enabled=%d\n",__FUNCTION__,enabled);
		cdev->desc.idVendor = device_desc.idVendor;
		cdev->desc.idProduct = device_desc.idProduct;
		cdev->desc.bcdDevice = device_desc.bcdDevice;
		cdev->desc.bDeviceClass = device_desc.bDeviceClass;
		cdev->desc.bDeviceSubClass = device_desc.bDeviceSubClass;
		cdev->desc.bDeviceProtocol = device_desc.bDeviceProtocol;
		printk(KERN_ERR"usb:%s idProduct=%x\n",__func__,device_desc.idProduct);
		if (usb_add_config(cdev, &android_config_driver,android_bind_config))
		{	
			printk(KERN_ERR"usb:%s usb_add_config fail\n",__func__);
			return size;			
		}
		
		usb_gadget_connect(cdev->gadget);
		dev->enabled = true;
	} else if (!enabled && dev->enabled) {
		printk(KERN_ERR"usb:%s,enabled=%d\n",__FUNCTION__,enabled);
		usb_gadget_disconnect(cdev->gadget);		
		usb_remove_config(cdev, &android_config_driver);
		dev->enabled = false;
	} else {
		pr_err("android_usb: already %s\n",
				dev->enabled ? "enabled" : "disabled");
	}
	return size;
}


/*for the convenience of usb PID switch, wzy_120306, begin*/
				/*right now it's used to enable adb in FTM*/
#define PRODUCT_ID_DIAG_ADB 0x0213

struct usb_function_info{
	__u16 product_id;
	char* clients;
	char* transports;
	char* functions;
	int start_adbd;
};

/*PID related functions we supported, should be modified*/
static struct usb_function_info function_info[] ={ 
	{
		.product_id	=PRODUCT_ID_DIAG_ADB,
		.clients	= "diag",
		.transports	= NULL,
		.functions	= "diag,adb",
		.start_adbd	= 1,
	},

	/*add more info here if need*/
};

/*get function info from pid, retuen 1(got), 0(miss)*/
static int get_function_info_from_pid(int pid, struct  usb_function_info** info)
{
	int index;
	int size=ARRAY_SIZE(function_info);
	for(index=0; index<size; index++){
		if(pid == function_info[index].product_id){
			*info = &function_info[index];
			pr_err("android_usb: find this pid %04x\n",pid);
			return 1;
			}
		}
	pr_err("android_usb: not support this pid %04x\n",pid);
	return 0;
}

static ssize_t 
switch_pid_store(struct device *pdev, struct device_attribute *attr,
			    const char *buff, size_t size)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_function_info *info;
	int target_pid = 0, got_info=0;	
	char *name;
	char buf[256], *b;
	int err;

	pr_err("android_usb: %s enter", __func__);
	sscanf(buff, "%04x\n", &target_pid);
	if(target_pid)
		got_info = get_function_info_from_pid(target_pid, &info);

	if(got_info){
		/*remove former config*/
		usb_gadget_disconnect(cdev->gadget);
		usb_remove_config(cdev, &android_config_driver);
		msleep(10);

		/*write in new config*/	
		if(info->clients){
			strlcpy(diag_clients, info->clients, sizeof(diag_clients));
			pr_err("android_usb: %s enter, clients=%s",__func__,diag_clients);
			}
		if(info->transports){
			strlcpy(serial_transports, info->transports, sizeof(serial_transports));
			pr_err("android_usb: %s enter, transports=%s",__func__,serial_transports);
			}

		/*change functions is more complex than change clients and transports*/
		if(info->functions)
		{
			pr_err("android_usb: %s enter, functions=%s",__func__,info->functions);
			INIT_LIST_HEAD(&dev->enabled_functions);			
			strlcpy(buf, info->functions, sizeof(buf));
			b = strim(buf);
			while(b){
			name = strsep(&b, ",");
			if(name){
				err = android_enable_function(dev, name);
				if(err)
					pr_err("android_usb: %s Cannot enable '%s'",__func__,name);
				}
			}
		}
	
		cdev->desc.idVendor = device_desc.idVendor;
		cdev->desc.idProduct = target_pid;
		cdev->desc.bcdDevice = device_desc.bcdDevice;
		cdev->desc.bDeviceClass = device_desc.bDeviceClass;
		cdev->desc.bDeviceSubClass = device_desc.bDeviceSubClass;
		cdev->desc.bDeviceProtocol = device_desc.bDeviceProtocol;
		ftm_iSerialNumber_filter(cdev); //xingbeilei_20120512	
		if (usb_add_config(cdev, &android_config_driver,
							android_bind_config))
			return size;

		usb_gadget_connect(cdev->gadget);
	} else {
		pr_err("android_usb: switch pid failed\n");
	}
	return size;
}
/*for the convenience of usb PID switch, end*/


static ssize_t state_show(struct device *pdev, struct device_attribute *attr,
			   char *buf)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	struct usb_composite_dev *cdev = dev->cdev;
	char *state = "DISCONNECTED";
	unsigned long flags;

	if (!cdev)
		goto out;

	spin_lock_irqsave(&cdev->lock, flags);
        if (cdev->config)
		state = "CONFIGURED";
	else if (dev->connected)
		state = "CONNECTED";
	spin_unlock_irqrestore(&cdev->lock, flags);
out:
	return snprintf(buf, PAGE_SIZE, "%s\n", state);
}

/*wangzy 120201*/
static ssize_t show_enable_cdrom(struct device *pdev,
					 struct device_attribute *attr,
					 char *buf)
{
	int i = 0;	
	i = scnprintf(buf, PAGE_SIZE, "is cdrom enabled after switch? [%s]\n",
		      is_cdrom_enabled_after_switch()?"enable":"disable");	
	return i;
}

static int is_cdrom_enabled_after_switch(void)
{
	return zte_usb_parameters.cdrom_enable_after_switch;
}
static void enable_cdrom_after_switch(int enable)
{
	zte_usb_parameters.cdrom_enable_after_switch = !!enable;
}

static ssize_t store_enable_cdrom(struct device *pdev,
					  struct device_attribute *attr,
					  const char *buf, size_t size)
{
	int cdrom_enable = 0;
	sscanf(buf, "%d", &cdrom_enable);

	pr_debug("android_usb: %s cdrom after switch\n",
			cdrom_enable ? "enabling" : "disabling");

	enable_cdrom_after_switch(cdrom_enable? 1 : 0);	
	return size;
}
/*end*/
#define DESCRIPTOR_ATTR(field, format_string)				\
static ssize_t								\
field ## _show(struct device *dev, struct device_attribute *attr,	\
		char *buf)						\
{									\
	return snprintf(buf, PAGE_SIZE,					\
			format_string, device_desc.field);		\
}									\
static ssize_t								\
field ## _store(struct device *dev, struct device_attribute *attr,	\
		const char *buf, size_t size)		       		\
{									\
	int value;					       		\
	if (sscanf(buf, format_string, &value) == 1) {			\
		device_desc.field = value;				\
		return size;						\
	}								\
	return -1;							\
}									\
static DEVICE_ATTR(field, S_IRUGO | S_IWUSR, field ## _show, field ## _store);

#define DESCRIPTOR_STRING_ATTR(field, buffer)				\
static ssize_t								\
field ## _show(struct device *dev, struct device_attribute *attr,	\
		char *buf)						\
{									\
	return snprintf(buf, PAGE_SIZE, "%s", buffer);			\
}									\
static ssize_t								\
field ## _store(struct device *dev, struct device_attribute *attr,	\
		const char *buf, size_t size)		       		\
{									\
	if (size >= sizeof(buffer)) return -EINVAL;			\
	if (sscanf(buf, "%255s", buffer) == 1) {			\
		return size;						\
	}								\
	return -1;							\
}									\
static DEVICE_ATTR(field, S_IRUGO | S_IWUSR, field ## _show, field ## _store);


DESCRIPTOR_ATTR(idVendor, "%04x\n")
DESCRIPTOR_ATTR(idProduct, "%04x\n")
DESCRIPTOR_ATTR(bcdDevice, "%04x\n")
DESCRIPTOR_ATTR(bDeviceClass, "%d\n")
DESCRIPTOR_ATTR(bDeviceSubClass, "%d\n")
DESCRIPTOR_ATTR(bDeviceProtocol, "%d\n")
DESCRIPTOR_STRING_ATTR(iManufacturer, manufacturer_string)
DESCRIPTOR_STRING_ATTR(iProduct, product_string)
DESCRIPTOR_STRING_ATTR(iSerial, serial_string)

static DEVICE_ATTR(functions, S_IRUGO | S_IWUSR, functions_show, functions_store);
static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, enable_show, enable_store);
static DEVICE_ATTR(state, S_IRUGO, state_show, NULL);
static DEVICE_ATTR(remote_wakeup, S_IRUGO | S_IWUSR,
		remote_wakeup_show, remote_wakeup_store);
/*wangzy 120201*/		
static DEVICE_ATTR(switch_pid, S_IRUGO | S_IWUSR, NULL,switch_pid_store);
static DEVICE_ATTR(enable_cdrom, 0664,show_enable_cdrom, store_enable_cdrom);
static DEVICE_ATTR(ftm_tag, 0664, show_ftm_tag, NULL);
static DEVICE_ATTR(nv_tag, 0664, show_nv_tag, NULL);
static DEVICE_ATTR(pidnv, 0664, msm_hsusb_show_pidnv, msm_hsusb_set_pidnv);
static DEVICE_ATTR(nvflag, 0664, nvflag_show, nvflag_store);
/*end*/				   
static struct device_attribute *android_usb_attributes[] = {
	&dev_attr_idVendor,
	&dev_attr_idProduct,
	&dev_attr_bcdDevice,
	&dev_attr_bDeviceClass,
	&dev_attr_bDeviceSubClass,
	&dev_attr_bDeviceProtocol,
	&dev_attr_iManufacturer,
	&dev_attr_iProduct,
	&dev_attr_iSerial,
	&dev_attr_functions,
	&dev_attr_enable,
	&dev_attr_state,
	&dev_attr_remote_wakeup,
	/*wangzy 120201*/
	&dev_attr_enable_cdrom,
	&dev_attr_ftm_tag,
	&dev_attr_nv_tag,
	&dev_attr_pidnv,
	&dev_attr_nvflag,
	&dev_attr_switch_pid, /*wangzy, for the convenience of usb PID switch*/
	/*end*/	
	NULL
};

/*-------------------------------------------------------------------------*/
/* Composite driver */

static int android_bind_config(struct usb_configuration *c)
{
	struct android_dev *dev = _android_dev;
	int ret = 0;

	ret = android_bind_enabled_functions(dev, c);
	if (ret)
		return ret;

	return 0;
}

static void android_unbind_config(struct usb_configuration *c)
{
	struct android_dev *dev = _android_dev;

	android_unbind_enabled_functions(dev, c);
}

static int android_bind(struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;
	struct usb_gadget	*gadget = cdev->gadget;
	int			gcnum, id, ret;
	pr_debug("usb android_bind\n");
	usb_gadget_disconnect(gadget);

	ret = android_init_functions(dev->functions, cdev);
	if (ret)
		return ret;

	/* Allocate string descriptor numbers ... note that string
	 * contents can be overridden by the composite_dev glue.
	 */
	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_MANUFACTURER_IDX].id = id;
	device_desc.iManufacturer = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_PRODUCT_IDX].id = id;
	device_desc.iProduct = id;

	/* Default strings - should be updated by userspace */
	strlcpy(manufacturer_string, "Android",
		sizeof(manufacturer_string) - 1);
	strlcpy(product_string, "Android", sizeof(product_string) - 1);
	strlcpy(serial_string, "0123456789ABCDEF", sizeof(serial_string) - 1);
	
	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_SERIAL_IDX].id = id;
	device_desc.iSerialNumber = id;	
	ftm_iSerialNumber_filter(cdev); //xingbeilei_20120512

	gcnum = usb_gadget_controller_number(gadget);
	if (gcnum >= 0)
		device_desc.bcdDevice = cpu_to_le16(0x0200 + gcnum);
	else {
		/* gadget zero is so simple (for now, no altsettings) that
		 * it SHOULD NOT have problems with bulk-capable hardware.
		 * so just warn about unrcognized controllers -- don't panic.
		 *
		 * things like configuration and altsetting numbering
		 * can need hardware-specific attention though.
		 */
		pr_warning("%s: controller '%s' not recognized\n",
			longname, gadget->name);
		device_desc.bcdDevice = __constant_cpu_to_le16(0x9999);
	}

	usb_gadget_set_selfpowered(gadget);
	dev->cdev = cdev;

	return 0;
}

static int android_usb_unbind(struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;

	cancel_work_sync(&dev->work);
	android_cleanup_functions(dev->functions);
	return 0;
}

static struct usb_composite_driver android_usb_driver = {
	.name		= "android_usb",
	.dev		= &device_desc,
	.strings	= dev_strings,
	.unbind		= android_usb_unbind,
};

static int
android_setup(struct usb_gadget *gadget, const struct usb_ctrlrequest *c)
{
	struct android_dev		*dev = _android_dev;
	struct usb_composite_dev	*cdev = get_gadget_data(gadget);
	struct usb_request		*req = cdev->req;
	struct android_usb_function	*f;
	int value = -EOPNOTSUPP;
	unsigned long flags;

	req->zero = 0;
	req->complete = composite_setup_complete;
	req->length = 0;
	gadget->ep0->driver_data = cdev;

	list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
		if (f->ctrlrequest) {
			value = f->ctrlrequest(f, cdev, c);
			if (value >= 0)
				break;
		}
	}

	/* Special case the accessory function.
	 * It needs to handle control requests before it is enabled.
	 */
	if (value < 0)
		value = acc_ctrlrequest(cdev, c);

	if (value < 0)
		value = composite_setup(gadget, c);

	spin_lock_irqsave(&cdev->lock, flags);
	if (!dev->connected) {
		dev->connected = 1;
		schedule_work(&dev->work);
	}
	else if (c->bRequest == USB_REQ_SET_CONFIGURATION && cdev->config) {
		schedule_work(&dev->work);
	}
	spin_unlock_irqrestore(&cdev->lock, flags);

	return value;
}

static void android_disconnect(struct usb_gadget *gadget)
{
	struct android_dev *dev = _android_dev;
	struct usb_composite_dev *cdev = get_gadget_data(gadget);
	unsigned long flags;

	composite_disconnect(gadget);

	spin_lock_irqsave(&cdev->lock, flags);
	dev->connected = 0;
	schedule_work(&dev->work);
	spin_unlock_irqrestore(&cdev->lock, flags);
}

static int android_create_device(struct android_dev *dev)
{
	struct device_attribute **attrs = android_usb_attributes;
	struct device_attribute *attr;
	int err;

	dev->dev = device_create(android_class, NULL,
					MKDEV(0, 0), NULL, "android0");
	if (IS_ERR(dev->dev))
		return PTR_ERR(dev->dev);

	dev_set_drvdata(dev->dev, dev);

	while ((attr = *attrs++)) {
		err = device_create_file(dev->dev, attr);
		if (err) {
			device_destroy(android_class, dev->dev->devt);
			return err;
		}
	}
	return 0;
}

static void android_destroy_device(struct android_dev *dev)
{
	struct device_attribute **attrs = android_usb_attributes;
	struct device_attribute *attr;

	while ((attr = *attrs++))
		device_remove_file(dev->dev, attr);
	device_destroy(android_class, dev->dev->devt);
}

static int __devinit android_probe(struct platform_device *pdev)
{
	struct android_usb_platform_data *pdata = pdev->dev.platform_data;
	struct android_dev *dev = _android_dev;

	dev_dbg(&pdev->dev, "%s: pdata: %p\n", __func__, pdata);
	dev->pdata = pdata;
	config_ftm_from_tag(); //not used when switch from userspace
	return 0;
}

static struct platform_driver android_platform_driver = {
	.driver = { .name = "android_usb"},
};

static int __init init(void)
{
	struct android_dev *dev;
	int ret;

	android_class = class_create(THIS_MODULE, "android_usb");
	if (IS_ERR(android_class))
		return PTR_ERR(android_class);

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		pr_err("%s(): Failed to alloc memory for android_dev\n",
				__func__);
		class_destroy(android_class);
		return -ENOMEM;
	}
	dev->functions = supported_functions;
	INIT_LIST_HEAD(&dev->enabled_functions);
	INIT_WORK(&dev->work, android_work);

	ret = android_create_device(dev);
	if (ret) {
		pr_err("%s(): android_create_device failed\n", __func__);
		goto err_dev;
	}
	_android_dev = dev;
	mutex_init(&dev->lock); //not used when switch from userspace
	/* Override composite driver functions */
	composite_driver.setup = android_setup;
	composite_driver.disconnect = android_disconnect;

	ret = platform_driver_probe(&android_platform_driver, android_probe);
	if (ret) {
		pr_err("%s(): Failed to register android"
				 "platform driver\n", __func__);
		goto err_probe;
	}
	ret = usb_composite_probe(&android_usb_driver, android_bind);
	if (ret) {
		pr_err("%s(): Failed to register android"
				 "composite driver\n", __func__);
		platform_driver_unregister(&android_platform_driver);
		goto err_probe;
	}
	return ret;

err_probe:
	android_destroy_device(dev);
err_dev:
	kfree(dev);
	class_destroy(android_class);
	return ret;
}
module_init(init);

static void __exit cleanup(void)
{
	usb_composite_unregister(&android_usb_driver);
	class_destroy(android_class);
	kfree(_android_dev);
	_android_dev = NULL;
	//destroy_usb_work_queue();	/*wangzy 120201, uncomment it when enable switch work*/
}
module_exit(cleanup);


#if 0 
//keep it for zero cd implemented in kernel
static int create_usb_work_queue(void)
{
	struct usb_ex_work *p = &global_usbwork;
	if (p->workqueue) {
		printk(KERN_ERR"usb:workqueue has created");
		return 0;
	}
	spin_lock_init(&p->lock);
	p->enable_switch = 1;
	p->enable_linux_switch = 0;
	p->switch_pid = PRODUCT_ID_MS_ADB;
	p->linux_pid = PRODUCT_ID_MS_ADB;
	p->cur_pid = PRODUCT_ID_MS_CDROM;
	p->has_switch = 0;
	p->workqueue = create_singlethread_workqueue("usb_workqueue");
	if (NULL == p->workqueue) {
		printk(KERN_ERR"usb:workqueue created fail");
		p->enable_switch = 0;
		return -1;
	}
	INIT_DELAYED_WORK(&p->switch_work, usb_switch_work);
	INIT_DELAYED_WORK(&p->linux_switch_work, usb_switch_os_work);
	INIT_DELAYED_WORK(&p->plug_work, usb_plug_work);
	wake_lock_init(&p->wlock,
		       WAKE_LOCK_SUSPEND, "usb_switch_wlock");
	return 0;
}



static int destroy_usb_work_queue(void)
{
	struct usb_ex_work *p = &global_usbwork;
	if (NULL != p->workqueue) {
		destroy_workqueue(p->workqueue);
		p->workqueue = NULL;
	}
	wake_lock_destroy(&p->wlock);
	memset(&global_usbwork, 0, sizeof(global_usbwork));
	return 0;
}

static void usb_plug_work(struct work_struct *w)
{
	unsigned long flags;
	int pid = 0;
	struct usb_ex_work *p = container_of(w, struct usb_ex_work, plug_work.work);

	if (!_android_dev) {

		printk(KERN_ERR"usb:%s: %d: _android_dev == NULL\n",
		       __FUNCTION__, __LINE__);
		return ;
	}
	
	spin_lock_irqsave(&p->lock, flags);
	if (!p->has_switch) {
		printk("usb:rms: %s %d: \n", __FUNCTION__, __LINE__);
		spin_unlock_irqrestore(&p->lock, flags);
		return ;
	}
	printk("usb:rms: %s %d: \n", __FUNCTION__, __LINE__);
	p->has_switch = 0;
	pid = p->cur_pid;
	spin_unlock_irqrestore(&p->lock, flags);
	//enable_cdrom(1);
	//DBG("plug work");
	printk("usb:rms %s:%d pid 0x%x cur_pid 0x%x\n",
	       __FUNCTION__, __LINE__, current_pid(), pid);

	mutex_lock(&_android_dev->lock);
	wake_lock(&p->wlock);
	//android_switch_composition((unsigned short)pid);
	wake_unlock(&p->wlock);
	mutex_unlock(&_android_dev->lock);

	return ;
}

static void usb_switch_work(struct work_struct *w)
{
	struct usb_ex_work *p = container_of(w, struct usb_ex_work, switch_work.work);
	unsigned long flags;
	if (!_android_dev) {

		printk(KERN_ERR"usb:%s: %d: _android_dev == NULL\n",
		       __FUNCTION__, __LINE__);
		return ;
	}
	if (!p->enable_switch) {
		return ;
	}
	if (p->has_switch) {
		printk("usb:rms:%s %d: already switch pid 0x%x switch_pid 0x%x\n",
		       __FUNCTION__, __LINE__, current_pid(), p->switch_pid);
		return ;
	}
	spin_lock_irqsave(&p->lock, flags);
//	p->cur_pid = ui->composition->product_id;
	p->has_switch = 1;
	spin_unlock_irqrestore(&p->lock, flags);
//	DBG("auto switch usb mode");
	printk("usb:rms:%s %d: pid 0x%x switch_pid 0x%x\n",
	       __FUNCTION__, __LINE__, current_pid(), p->switch_pid);
	//enable_cdrom(0);

	mutex_lock(&_android_dev->lock);
	wake_lock(&p->wlock);
	//android_switch_composition((unsigned short)p->switch_pid);
	wake_unlock(&p->wlock);
	mutex_unlock(&_android_dev->lock);

	return ;
}

static void usb_switch_os_work(struct work_struct *w)
{
	struct usb_ex_work *p =
		container_of(w, struct usb_ex_work, linux_switch_work.work);
	unsigned long flags;

	if (!_android_dev) {

		printk(KERN_ERR"usb:%s: %d: _android_dev == NULL\n",
		       __FUNCTION__, __LINE__);
		return ;
	}

	if (!p->enable_switch || !p->enable_linux_switch || p->has_switch) {
		//switch  or linux_switch are enable, or we has already switch,return direct
		printk("usb:rms:%s:%d, switch %s: linux switch %s: %s switch\n",
		       __FUNCTION__, __LINE__, p->enable_switch?"enable":"disable",
		       p->enable_linux_switch?"enable":"disable",
		       p->has_switch?"has":"has not");
		return ;
	}
	spin_lock_irqsave(&p->lock, flags);
//	p->cur_pid = ui->composition->product_id;
	p->has_switch = 1;
	spin_unlock_irqrestore(&p->lock, flags);
	printk("usb:rms:%s %d: pid 0x%x linux_pid 0x%x\n",
	       __FUNCTION__, __LINE__, current_pid(), p->linux_pid);

	mutex_lock(&_android_dev->lock);
	wake_lock(&p->wlock);
	//android_switch_composition((unsigned short)p->linux_pid);
	wake_unlock(&p->wlock);
	mutex_unlock(&_android_dev->lock);

	return ;
}

void schedule_cdrom_stop(void)
{
	
	if (NULL == global_usbwork.workqueue) {
		return ;
	}
	queue_delayed_work(global_usbwork.workqueue, &global_usbwork.switch_work, HZ/10);

	return;
}
EXPORT_SYMBOL(schedule_cdrom_stop);
void schedule_linux_os(void)
{
	if (NULL == global_usbwork.workqueue) {
		return ;
	}
	queue_delayed_work(global_usbwork.workqueue,
			   &global_usbwork.linux_switch_work, 0);

	return;
}
EXPORT_SYMBOL(schedule_linux_os);

void schedule_usb_plug(void)
{
	
	if (NULL == global_usbwork.workqueue) {
		return ;
	}
	printk("usb:rms: %s %d: \n", __FUNCTION__, __LINE__);
	queue_delayed_work(global_usbwork.workqueue, &global_usbwork.plug_work, 0);

	return ;
}
EXPORT_SYMBOL(schedule_usb_plug);

static void clear_switch_flag(void)
{
	unsigned long flags;
	struct usb_ex_work *p = &global_usbwork;
	spin_lock_irqsave(&p->lock, flags);
	p->has_switch = 0;
	spin_unlock_irqrestore(&p->lock, flags);

	return ;
}
#endif

static int usb_cdrom_is_enable(void)
{
	return (PRODUCT_ID_MS_CDROM == current_pid()) ? 1:0;
}
int os_switch_is_enable(void) /*switch_pid when linux*/
{
	//struct usb_ex_work *p = &global_usbwork;
	int enable_linux_switch=1; /*can be over written by userspace(p->enable_linux_switch)*/
	return usb_cdrom_is_enable()? enable_linux_switch : 0;  
}
EXPORT_SYMBOL(os_switch_is_enable);

int get_nluns(void)
{
        if (usb_cdrom_is_enable() || is_cdrom_enabled_after_switch()) {
                return 2;
        }
        return 1;
}
EXPORT_SYMBOL(get_nluns);
//end
