/* arch/arm/mach-msm/proc_comm.c
 *
 * Copyright (C) 2007-2008 Google, Inc.
 * Copyright (c) 2009-2011, Code Aurora Forum. All rights reserved.
 * Author: Brian Swetland <swetland@google.com>
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

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/module.h>
#include <mach/msm_iomap.h>
#include <mach/system.h>

#include "proc_comm.h"
#include "smd_private.h"

static inline void notify_other_proc_comm(void)
{
	/* Make sure the write completes before interrupt */
	wmb();
#if defined(CONFIG_ARCH_MSM7X30)
	__raw_writel(1 << 6, MSM_GCC_BASE + 0x8);
#elif defined(CONFIG_ARCH_MSM8X60)
	__raw_writel(1 << 5, MSM_GCC_BASE + 0x8);
#else
	__raw_writel(1, MSM_CSR_BASE + 0x400 + (6) * 4);
#endif
}

#define APP_COMMAND 0x00
#define APP_STATUS  0x04
#define APP_DATA1   0x08
#define APP_DATA2   0x0C

#define MDM_COMMAND 0x10
#define MDM_STATUS  0x14
#define MDM_DATA1   0x18
#define MDM_DATA2   0x1C

static DEFINE_SPINLOCK(proc_comm_lock);
static int msm_proc_comm_disable;

/* Poll for a state change, checking for possible
 * modem crashes along the way (so we don't wait
 * forever while the ARM9 is blowing up.
 *
 * Return an error in the event of a modem crash and
 * restart so the msm_proc_comm() routine can restart
 * the operation from the beginning.
 */
static int proc_comm_wait_for(unsigned addr, unsigned value)
{
	while (1) {
		/* Barrier here prevents excessive spinning */
		mb();
		if (readl_relaxed(addr) == value)
			return 0;

		if (smsm_check_for_modem_crash())
			return -EAGAIN;

		udelay(5);
	}
}

void msm_proc_comm_reset_modem_now(void)
{
	unsigned base = (unsigned)MSM_SHARED_RAM_BASE;
	unsigned long flags;

	spin_lock_irqsave(&proc_comm_lock, flags);

again:
	if (proc_comm_wait_for(base + MDM_STATUS, PCOM_READY))
		goto again;

	writel_relaxed(PCOM_RESET_MODEM, base + APP_COMMAND);
	writel_relaxed(0, base + APP_DATA1);
	writel_relaxed(0, base + APP_DATA2);

	spin_unlock_irqrestore(&proc_comm_lock, flags);

	/* Make sure the writes complete before notifying the other side */
	wmb();
	notify_other_proc_comm();

	return;
}
EXPORT_SYMBOL(msm_proc_comm_reset_modem_now);

#define _NV_WLAN_MAC_ADDRESS_	4678
#define _NV_WLAN_ATHEROS_SPECIFIC_CFG_	3757
int msm_hsusb_get_set_usb_conf_nv_value(uint32_t nv_item,uint32_t value,uint32_t is_write);
int msm_proc_comm(unsigned cmd, unsigned *data1, unsigned *data2)
{
	unsigned base = (unsigned)MSM_SHARED_RAM_BASE;
	unsigned long flags;
	int ret;

	if(cmd == PCOM_CUSTOMER_CMD1){
		if(*data2 & (1<<31)){
			char wifi_addr[6]={0};
		
			ret = msm_hsusb_get_set_usb_conf_nv_value(_NV_WLAN_MAC_ADDRESS_, (uint32_t)wifi_addr, 0);
			if(!ret){
				*data1=(unsigned)(wifi_addr[0]+(wifi_addr[1]<<8)+(wifi_addr[2]<<16)+(wifi_addr[3]<<24));
        			*data2=(unsigned)(wifi_addr[4]+(wifi_addr[5]<<8));	
			}
			return ret;
		}
		else if((*data2>>24) & 0xFF){
			char wifi_atheros_rf_data[24] = {0};
			int wifi_channel_offset = 0;
			
			ret = msm_hsusb_get_set_usb_conf_nv_value(_NV_WLAN_ATHEROS_SPECIFIC_CFG_, (uint32_t)wifi_atheros_rf_data, 0);
			if(!ret){
				wifi_channel_offset = ((*data2 >> 24) - 1)*8;
				*data1 = (unsigned)(wifi_atheros_rf_data[0 + wifi_channel_offset] + (wifi_atheros_rf_data[1 + wifi_channel_offset] << 8) + (wifi_atheros_rf_data[2 + wifi_channel_offset] << 16) + (wifi_atheros_rf_data[3 + wifi_channel_offset] << 24));
				*data2 = (unsigned)(wifi_atheros_rf_data[4 + wifi_channel_offset] + (wifi_atheros_rf_data[5 + wifi_channel_offset] << 8) + (wifi_atheros_rf_data[6 + wifi_channel_offset] << 16) + (wifi_atheros_rf_data[7 + wifi_channel_offset] << 24));
			}		
			return ret;
		}
		else{
		}

	}

	spin_lock_irqsave(&proc_comm_lock, flags);

	if (msm_proc_comm_disable) {
		ret = -EIO;
		goto end;
	}


again:
	if (proc_comm_wait_for(base + MDM_STATUS, PCOM_READY))
		goto again;

	writel_relaxed(cmd, base + APP_COMMAND);
	writel_relaxed(data1 ? *data1 : 0, base + APP_DATA1);
	writel_relaxed(data2 ? *data2 : 0, base + APP_DATA2);

	/* Make sure the writes complete before notifying the other side */
	wmb();
	notify_other_proc_comm();

	if (proc_comm_wait_for(base + APP_COMMAND, PCOM_CMD_DONE))
		goto again;

	if (readl_relaxed(base + APP_STATUS) == PCOM_CMD_SUCCESS) {
		if (data1)
			*data1 = readl_relaxed(base + APP_DATA1);
		if (data2)
			*data2 = readl_relaxed(base + APP_DATA2);
		ret = 0;
	} else {
		ret = -EIO;
	}

	writel_relaxed(PCOM_CMD_IDLE, base + APP_COMMAND);

	switch (cmd) {
	case PCOM_RESET_CHIP:
	case PCOM_RESET_CHIP_IMM:
	case PCOM_RESET_APPS:
		msm_proc_comm_disable = 1;
		printk(KERN_ERR "msm: proc_comm: proc comm disabled\n");
		break;
	}
end:
	/* Make sure the writes complete before returning */
	wmb();
	spin_unlock_irqrestore(&proc_comm_lock, flags);
	return ret;
}
EXPORT_SYMBOL(msm_proc_comm);
