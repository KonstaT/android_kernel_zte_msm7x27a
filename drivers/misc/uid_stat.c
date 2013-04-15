/* drivers/misc/uid_stat.c
 *
 * Copyright (C) 2008 - 2009 Google, Inc.
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

#include <asm/atomic.h>

#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/stat.h>
#include <linux/uid_stat.h>
#include <net/activity_stats.h>

static DEFINE_SPINLOCK(uid_lock);
static LIST_HEAD(uid_list);
static struct proc_dir_entry *parent;

struct uid_stat {
	struct list_head link;
	uid_t uid;
	atomic_t tcp_rcv;
	atomic_t tcp_snd;
};

static struct uid_stat *find_uid_stat(uid_t uid) {
	unsigned long flags;
	struct uid_stat *entry;

	spin_lock_irqsave(&uid_lock, flags);
	list_for_each_entry(entry, &uid_list, link) {
		if (entry->uid == uid) {
			spin_unlock_irqrestore(&uid_lock, flags);
			return entry;
		}
	}
	spin_unlock_irqrestore(&uid_lock, flags);
	return NULL;
}

static int tcp_snd_read_proc(char *page, char **start, off_t off,
				int count, int *eof, void *data)
{
	int len;
	unsigned int bytes;
	char *p = page;
	struct uid_stat *uid_entry = (struct uid_stat *) data;
	if (!data)
		return 0;

	bytes = (unsigned int) (atomic_read(&uid_entry->tcp_snd) + INT_MIN);
	p += sprintf(p, "%u\n", bytes);
	len = (p - page) - off;
	*eof = (len <= count) ? 1 : 0;
	*start = page + off;
	return len;
}

static int tcp_rcv_read_proc(char *page, char **start, off_t off,
				int count, int *eof, void *data)
{
	int len;
	unsigned int bytes;
	char *p = page;
	struct uid_stat *uid_entry = (struct uid_stat *) data;
	if (!data)
		return 0;

	bytes = (unsigned int) (atomic_read(&uid_entry->tcp_rcv) + INT_MIN);
	p += sprintf(p, "%u\n", bytes);
	len = (p - page) - off;
	*eof = (len <= count) ? 1 : 0;
	*start = page + off;
	return len;
}

/* Create a new entry for tracking the specified uid. */
static struct uid_stat *create_stat(uid_t uid) {
	unsigned long flags;
	char uid_s[32];
	struct uid_stat *new_uid;
	struct proc_dir_entry *entry;

	/* Create the uid stat struct and append it to the list. */
	if ((new_uid = kmalloc(sizeof(struct uid_stat), GFP_KERNEL)) == NULL)
		return NULL;

	new_uid->uid = uid;
	/* Counters start at INT_MIN, so we can track 4GB of network traffic. */
	atomic_set(&new_uid->tcp_rcv, INT_MIN);
	atomic_set(&new_uid->tcp_snd, INT_MIN);

	spin_lock_irqsave(&uid_lock, flags);
	list_add_tail(&new_uid->link, &uid_list);
	spin_unlock_irqrestore(&uid_lock, flags);

	sprintf(uid_s, "%d", uid);
	entry = proc_mkdir(uid_s, parent);

	/* Keep reference to uid_stat so we know what uid to read stats from. */
	create_proc_read_entry("tcp_snd", S_IRUGO, entry , tcp_snd_read_proc,
		(void *) new_uid);

	create_proc_read_entry("tcp_rcv", S_IRUGO, entry, tcp_rcv_read_proc,
		(void *) new_uid);

	return new_uid;
}

#ifdef CONFIG_ZTE_PLATFORM
#include <linux/module.h>
#include <linux/mm.h>
enum {
	DEBUG_TCP_SND = 1U << 0,
	DEBUG_TCP_RCV = 1U << 1,
};

static int debug_mask = 0;
module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

char cmdline_buf[PAGE_SIZE]={0};
static int get_cmdline(char * buffer)
{
	int res = 0;
	unsigned int len;
	struct mm_struct *mm = get_task_mm(current);
	if (!mm)
		goto out;
	if (!mm->arg_end)
		goto out_mm;	/* Shh! No looking before we're done */

 	len = mm->arg_end - mm->arg_start;
 
	if (len > PAGE_SIZE)
		len = PAGE_SIZE;
 
	res = access_process_vm(current, mm->arg_start, buffer, len, 0);

	// If the nul at the end of args has been overwritten, then
	// assume application is using setproctitle(3).
	if (res > 0 && buffer[res-1] != '\0' && len < PAGE_SIZE) {
		len = strnlen(buffer, res);
		if (len < res) {
		    res = len;
		} else {
			len = mm->env_end - mm->env_start;
			if (len > PAGE_SIZE - res)
				len = PAGE_SIZE - res;
			res += access_process_vm(current, mm->env_start, buffer+res, len, 0);
			res = strnlen(buffer, res);
		}
	}
out_mm:
	mmput(mm);
out:
	return res;
}
#endif
int uid_stat_tcp_snd(uid_t uid, int size) {
	struct uid_stat *entry;
	activity_stats_update();
	if ((entry = find_uid_stat(uid)) == NULL &&
		((entry = create_stat(uid)) == NULL)) {
			return -1;
	}
	atomic_add(size, &entry->tcp_snd);
	#ifdef CONFIG_ZTE_PLATFORM
	if (debug_mask & DEBUG_TCP_SND) {
	memset(cmdline_buf,0,sizeof(cmdline_buf));
	get_cmdline(cmdline_buf);
	pr_info("TCP_SND: uid=%d(%s) size=%d\n",uid,cmdline_buf,size);
	#endif
	}
	return 0;
}

int uid_stat_tcp_rcv(uid_t uid, int size) {
	struct uid_stat *entry;
	activity_stats_update();
	if ((entry = find_uid_stat(uid)) == NULL &&
		((entry = create_stat(uid)) == NULL)) {
			return -1;
	}
	atomic_add(size, &entry->tcp_rcv);
	
	#ifdef CONFIG_ZTE_PLATFORM
	if (debug_mask & DEBUG_TCP_RCV) {
		memset(cmdline_buf,0,sizeof(cmdline_buf));
		get_cmdline(cmdline_buf);
		pr_info("TCP_RCV: uid=%d(%s) size=%d\n",uid,cmdline_buf,size);
	}
	#endif
	return 0;
}

static int __init uid_stat_init(void)
{
	parent = proc_mkdir("uid_stat", NULL);
	if (!parent) {
		pr_err("uid_stat: failed to create proc entry\n");
		return -1;
	}
	return 0;
}

__initcall(uid_stat_init);
