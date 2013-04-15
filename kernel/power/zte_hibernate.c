
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/suspend.h>
#include <linux/syscalls.h> /* sys_sync */
#include <linux/wakelock.h>
#include <linux/proc_fs.h>
#include "power.h"
#include "../../arch/arm/mach-msm/proc_comm.h"
#include <linux/earlysuspend.h>

int print_hb_wakelock(int type, char *buf, int len, unsigned long on_jiff, unsigned long timeout);
/* ruanmeisi for hibernate */


struct hb_event {
	int hb_event_changed;
	int hb_event;
	struct semaphore sem;
	wait_queue_head_t rqueue;
	struct proc_dir_entry *p;
	struct work_struct work;
	struct work_struct key_work;
};

struct hb_early_suspend {
	struct early_suspend early_suspend0;
	struct early_suspend early_suspend1;
	int    state;
};
struct hb_early_suspend hb_early_suspend;

struct hb_wakelock {
	int hb_wakelock_changed;
	struct semaphore sem;
	wait_queue_head_t rqueue;
	char   buf[PAGE_SIZE];
	int len;
	unsigned long timeout;
	unsigned long on_jiff;
};


static void hb_wakelock_handle(unsigned long data);
int is_in_hibernate(void);

static struct proc_dir_entry *hibernate_proc_dir;
static BLOCKING_NOTIFIER_HEAD(hibernate_chain_head);
static DEFINE_MUTEX(hibernate_lock);

struct hb_event hb_event;

struct hb_wakelock hb_wakelock;
static DEFINE_TIMER(hb_wakelock_timer, hb_wakelock_handle, 0,(unsigned long)&hb_wakelock);


#define ZTE_PROC_COMM_CMD3_HIBERATE_ENTER 20
#define ZTE_PROC_COMM_CMD3_HIBERATE_EXIT 21


static void hiberate_enter_exit(bool enter_exit)
{
	unsigned subcmd=ZTE_PROC_COMM_CMD3_HIBERATE_EXIT; 
	subcmd = enter_exit?ZTE_PROC_COMM_CMD3_HIBERATE_ENTER : ZTE_PROC_COMM_CMD3_HIBERATE_EXIT; 
	printk(KERN_ERR"hibernate: proc_comm to notify arm9 hibernate %s\n",
	       enter_exit? "enter":"exit"); 
	msm_proc_comm(PCOM_CUSTOMER_CMD3, 0, &subcmd);
	return ;
}

void hibernate_late_resume0(struct early_suspend *h)
{
	struct hb_early_suspend *hbe =
		container_of(h, struct hb_early_suspend, early_suspend0);
	hbe->state = 0;
}
void hibernate_state_preparing0(struct early_suspend *h)
{
	struct hb_early_suspend *hbe =
		container_of(h, struct hb_early_suspend, early_suspend0);
	hbe->state = 2;
	return;
}


void hibernate_early_suspend1(struct early_suspend *h)
{
	struct hb_early_suspend *hbe =
		container_of(h, struct hb_early_suspend, early_suspend1);
	hbe->state = 1;
}


void hibernate_state_preparing1(struct early_suspend *h)
{
	struct hb_early_suspend *hbe =
		container_of(h, struct hb_early_suspend, early_suspend1);
	hbe->state = 3;
	return;
}

static char* hbe_state[] = {
	"late_resume",
	"early_suspend",
	"resume->suspend",
	"suspend->resume",
};
static int hbe_read_proc(
        char *page, char **start, off_t off, int count, int *eof, void *data)
{
	struct hb_early_suspend *hbe = &hb_early_suspend;
	int len = 0;
	if (hbe->state >= ARRAY_SIZE(hbe_state)) {
		len = sprintf(page, "%s\n", "unknow");
	} else {
		len = sprintf(page, "%s\n", hbe_state[hbe->state]);
	}
	return len;

}


static void hb_event_changed(struct hb_event *dev, int state)
{
	printk(KERN_ERR"%s %d state %d\n", __FUNCTION__, __LINE__, state);
	if (down_interruptible(&dev->sem))
		goto err;
	
	dev->hb_event_changed = 1;
	dev->hb_event = state;
	wake_up_interruptible(&dev->rqueue);
	up(&dev->sem);
err:
        return ;
}

static void hb_event_work(struct work_struct *w)
{
	struct hb_event *hb = container_of(w, struct hb_event, work);
	hb_event_changed(hb, 1);
}

void alarm_wakeup_hibernate(void)
{
	if (!is_in_hibernate()) {
		return ;
	}
	schedule_work(&hb_event.work);
	return ;
}
EXPORT_SYMBOL_GPL(alarm_wakeup_hibernate);


static void hb_event_key_work(struct work_struct *w)
{
	struct hb_event *hb = container_of(w, struct hb_event, key_work);
	hb_event_changed(hb, 2);
}

void powerkey_press_hibernate(void)
{
	if (!is_in_hibernate()) {
		return ;
	}
	schedule_work(&hb_event.key_work);
	return ;
}
EXPORT_SYMBOL_GPL(powerkey_press_hibernate);

static int hb_event_open(struct inode *inodp, struct file *filp)
{
        return 0;
}

//char * get_wakeup_reason(void);
char *hibernate_event_strings[] = {
	"UNKNOW",
	"ALARM",
	"POWERKEY"
};

static ssize_t hb_event_read(struct file *filp, char __user *buff, size_t count, loff_t *f_pos)
{
	int ret = 0;
	char *reason = NULL;
	struct hb_event* dev = &hb_event;
	if ((filp->f_flags & O_NONBLOCK) && !dev->hb_event_changed) {
		return -EAGAIN;
	}

	if (wait_event_interruptible(dev->rqueue, dev->hb_event_changed))
		goto err;

	if (down_interruptible(&dev->sem))
		goto err;

	//reason = get_wakeup_reason();
	if ((dev->hb_event >= 0)
	    && (dev->hb_event < ARRAY_SIZE(hibernate_event_strings))) {
		reason = hibernate_event_strings[dev->hb_event];
	} else {
		reason = "UNKNOW";
	}
	if (copy_to_user(buff, reason, strlen(reason))) {
		ret = -EFAULT;
	} else {
		ret = strlen(reason);
	}
	dev->hb_event_changed = 0;
	up(&dev->sem);
	return ret;
err:
        return -ERESTARTSYS;
}


static unsigned int hb_event_poll(struct file *filp, struct poll_table_struct *p)
{
	unsigned int mask = 0;
	struct hb_event* dev = &hb_event;
	if (down_interruptible(&dev->sem))
		goto err;

	poll_wait(filp, &dev->rqueue, p);

	if(dev->hb_event_changed) {
		mask |= POLLIN | POLLRDNORM;
	}
	up(&dev->sem);
   return mask;
		
err:
	return -ERESTARTSYS;
}

static int hb_event_release(struct inode *inodp, struct file *filp)
{
	return 0;
}

static struct file_operations hb_event_fops = {
	.open = hb_event_open,
	.read = hb_event_read,
	.write = NULL,
	.poll = hb_event_poll,
	.release = hb_event_release,
};





static void hb_wakelock_handle(unsigned long data)
{
	struct hb_wakelock *hbw = (struct hb_wakelock*)data;
	int ret = 0;

	if (down_interruptible(&hbw->sem))
		goto err;
	memset(hbw->buf, 0, sizeof(hbw->buf));
	hbw->len = 0;
	ret = print_hb_wakelock(WAKE_LOCK_SUSPEND, hbw->buf, sizeof(hbw->buf), hbw->on_jiff, hbw->timeout);

	if (ret) {
		hbw->hb_wakelock_changed = 1;
		hbw->len = ret;
		wake_up_interruptible(&hbw->rqueue);
	}
	up(&hbw->sem);
	if (is_in_hibernate()) {
		mod_timer(&hb_wakelock_timer, jiffies + hbw->timeout/2);
	}
err:
        return ;
}

static void hb_wakelock_hibernate_on(void)
{
	struct hb_wakelock *hbw = &hb_wakelock;
	struct hb_event    *hbe = &hb_event;
	if (down_interruptible(&hbw->sem))
		goto err;

	memset(hbw->buf, 0, sizeof(hbw->buf));
	hbw->len = 0;
	hbw->hb_wakelock_changed = 0;
	hbw->on_jiff = jiffies;
	mod_timer(&hb_wakelock_timer, jiffies + hbw->timeout/2);
	up(&hbw->sem);
	if (down_interruptible(&hbe->sem))
		goto err;
	hbe->hb_event = 0;
	hbe->hb_event_changed = 0;
	up(&hbe->sem);
err:
	return ;
}
static int hb_wakelock_open(struct inode *inodp, struct file *filp)
{
        return 0;
}


static ssize_t hb_wakelock_read(struct file *filp, char __user *buff, size_t count, loff_t *f_pos)
{
	int ret = 0;
	struct hb_wakelock* hbw = &hb_wakelock;
	if ((filp->f_flags & O_NONBLOCK) && !hbw->hb_wakelock_changed) {
		return -EAGAIN;
	}

	if (wait_event_interruptible(hbw->rqueue, hbw->hb_wakelock_changed))
		goto err;

	if (down_interruptible(&hbw->sem))
		goto err;

	if (copy_to_user(buff, hbw->buf, hbw->len)) {
		ret = -EFAULT;
	} else {
		ret = hbw->len;
	}
	hbw->len = 0;
	hbw->hb_wakelock_changed = 0;
	up(&hbw->sem);
	return ret;
err:
        return -ERESTARTSYS;
}


static unsigned int hb_wakelock_poll(struct file *filp, struct poll_table_struct *p)
{
	unsigned int mask = 0;
	struct hb_wakelock* hbw = &hb_wakelock;
	if (down_interruptible(&hbw->sem))
		goto err;

	poll_wait(filp, &hbw->rqueue, p);

	if(hbw->hb_wakelock_changed) {
		mask |= POLLIN | POLLRDNORM;
	}
	up(&hbw->sem);
   return mask;
		
err:
	return -ERESTARTSYS;
}

ssize_t hb_wakelock_write (struct file *fp, const char __user *buf,
				 size_t count, loff_t *pos)
{
	char tmp[16] = {0};
	unsigned long l = 0;
	if (count >= sizeof(tmp)) {
		return -EINVAL;
	}
	if(copy_from_user(tmp, buf, count))
		return -EFAULT;
	l = (unsigned long)simple_strtol(tmp, NULL, 10);
	if (l >0 ) {
		hb_wakelock.timeout = l*HZ;
	} else {
		hb_wakelock.timeout = 2*60*HZ; //2 min
	}
	printk(KERN_ERR"hb_wakelock timeout %lu\n", l);
	return count;
}

static int hb_wakelock_release(struct inode *inodp, struct file *filp)
{
	return 0;
}

static struct file_operations hb_wakelock_fops = {
	.open = hb_wakelock_open,
	.read = hb_wakelock_read,
	.write = hb_wakelock_write,
	.poll = hb_wakelock_poll,
	.release = hb_wakelock_release,
};

struct pm_state {
	int pm_state_changed;
	int pm_state;
	struct semaphore sem;
	wait_queue_head_t rqueue;
	struct proc_dir_entry *p;
};

static struct pm_state pm_state;


static void pm_state_changed(struct pm_state *dev, int state)
{
	printk(KERN_ERR"%s %d\n", __FUNCTION__, __LINE__);
	if (down_interruptible(&dev->sem))
		goto err;
	
	dev->pm_state_changed = 1;
	dev->pm_state = state;
	wake_up_interruptible(&dev->rqueue);
	up(&dev->sem);
err:
        return ;
}

static int pm_suspend_notifier(struct notifier_block *nb,
				unsigned long event,
				void *dummy)
{

	struct pm_state* dev = &pm_state;
	switch (event) {
		case PM_POST_SUSPEND:
			pm_state_changed(dev, 1);
			return NOTIFY_OK;
		default:
			return NOTIFY_DONE;
	}
	return NOTIFY_DONE;
}

static struct notifier_block pm_state_suspend_notifier = {
	.notifier_call = pm_suspend_notifier,
};


static int pm_state_open(struct inode *inodp, struct file *filp)
{
        return 0;
}

//char * get_wakeup_reason(void);

static ssize_t pm_state_read(struct file *filp, char __user *buff, size_t count, loff_t *f_pos)
{
	int ret = 0;
	char *reason = NULL;
	struct pm_state* dev = &pm_state;
	if ((filp->f_flags & O_NONBLOCK) && !dev->pm_state_changed) {
		return -EAGAIN;
	}

	if (wait_event_interruptible(dev->rqueue, dev->pm_state_changed))
		goto err;

	if (down_interruptible(&dev->sem))
		goto err;

	//reason = get_wakeup_reason();
	reason = "unknow";
	if (copy_to_user(buff, reason, strlen(reason))) {
		ret = -EFAULT;
	} else {
		ret = strlen(reason);
	}
	dev->pm_state_changed = 0;
	up(&dev->sem);
	return ret;
err:
        return -ERESTARTSYS;
}


static unsigned int pm_state_poll(struct file *filp, struct poll_table_struct *p)
{
	unsigned int mask = 0;
	struct pm_state* dev = &pm_state;
	if (down_interruptible(&dev->sem))
		goto err;

	poll_wait(filp, &dev->rqueue, p);

	if(dev->pm_state_changed) {
		mask |= POLLIN | POLLRDNORM;
	}
	up(&dev->sem);
   return mask;
		
err:
	return -ERESTARTSYS;
}

static int pm_state_release(struct inode *inodp, struct file *filp)
{
	return 0;
}

static struct file_operations pm_state_fops = {
	.open = pm_state_open,
	.read = pm_state_read,
	.write = NULL,
	.poll = pm_state_poll,
	.release = pm_state_release,
};


static int hibernate_debug_event(struct notifier_block *this,
				unsigned long event, void *ptr)
{
	printk(KERN_ERR"hibernate: %s %d event %lu\n", __FUNCTION__, __LINE__, event);
	return 0;
}


static struct notifier_block hibernate_debug_notifier = {
	.notifier_call = hibernate_debug_event,
};


int register_hibernate_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&hibernate_chain_head, nb);
}
EXPORT_SYMBOL_GPL(register_hibernate_notifier);

int unregister_hibernate_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&hibernate_chain_head, nb);
}
EXPORT_SYMBOL_GPL(unregister_hibernate_notifier);

static int hibernate_notifier_call_chain(unsigned long val)
{
	return (blocking_notifier_call_chain(&hibernate_chain_head, val, NULL)
			== NOTIFY_BAD) ? -EINVAL : 0;
}


static int in_hibernate = 0;
int is_in_hibernate(void)
{
	return in_hibernate;
}
EXPORT_SYMBOL(is_in_hibernate);
static void set_hibernate(int enable)
{
	in_hibernate = enable;
}

static int hibernate_read_proc(
        char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;
	len = sprintf(page, "%s\n",
                      is_in_hibernate()?"on":"off");
	return len;

}

static int hibernate_write_proc(struct file *file, const char __user *buffer,
			     unsigned long count, void *data)
{
	char tmp[16] = {0};
	int len = 0;
	len = count;
	if (count > sizeof(tmp)) {
		len = sizeof(tmp) - 1;
	}
	if(copy_from_user(tmp, buffer, len))
                return -EFAULT;
	mutex_lock(&hibernate_lock);
	if (strstr(tmp, "on")) {
		set_hibernate(1);
		hibernate_notifier_call_chain(1);
		hb_wakelock_hibernate_on();
		hiberate_enter_exit(1);
	} else if (strstr(tmp, "off")) {
		set_hibernate(0);
		hibernate_notifier_call_chain(0);
		hiberate_enter_exit(0);
	}
	mutex_unlock(&hibernate_lock);

	return count;

}




static int __init
init_hibernate_proc(void)
{
	struct hb_event *dev = &hb_event;
	struct proc_dir_entry *p = NULL;
	struct hb_wakelock *hbw  = &hb_wakelock;
	int ret = 0;
	memset(dev, 0, sizeof(*dev));
	sema_init(&dev->sem, 1);
	init_waitqueue_head(&dev->rqueue);
	INIT_WORK(&dev->work, hb_event_work);
	INIT_WORK(&dev->key_work, hb_event_key_work);
	memset(hbw, 0, sizeof(*hbw));
	sema_init(&hbw->sem, 1);
	init_waitqueue_head(&hbw->rqueue);
	hbw->timeout = 2 * 60 * HZ;
	memset(&pm_state, 0, sizeof(pm_state));
	sema_init(&pm_state.sem, 1);
	init_waitqueue_head(&pm_state.rqueue);
	hibernate_proc_dir = proc_mkdir("hibernate", NULL);
	if (!hibernate_proc_dir)
		goto err;
	p = proc_create("hb_event", S_IRUGO, hibernate_proc_dir, &hb_event_fops);
	if (NULL == p)
		goto out_hb_event;

	p = proc_create("hb_wakelock", S_IRUGO, hibernate_proc_dir, &hb_wakelock_fops);
	if (NULL == p)
		goto out_hb_wakelock;
	p = proc_create("pm_state", S_IRUGO, hibernate_proc_dir, &pm_state_fops);
	if (NULL == p)
		goto out_pm_state;


	p = create_proc_entry("hibernate_notify",
				    0, hibernate_proc_dir);
        if (NULL == p) {
		goto out_hibernate_notify;
	}
	p->read_proc = hibernate_read_proc;
	p->write_proc = hibernate_write_proc;

	p->data = NULL;

	ret = register_hibernate_notifier(&hibernate_debug_notifier);
	if (ret)
		goto out_unregister;
	ret = register_pm_notifier(&pm_state_suspend_notifier);
	if (ret)
		goto out_pm_unregister;
	memset(&hb_early_suspend, 0, sizeof(hb_early_suspend));
	hb_early_suspend.early_suspend0.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 20;
	hb_early_suspend.early_suspend0.suspend = hibernate_state_preparing0;
	hb_early_suspend.early_suspend0.resume = hibernate_late_resume0;
	register_early_suspend(&hb_early_suspend.early_suspend0);
	hb_early_suspend.early_suspend1.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1;
	hb_early_suspend.early_suspend1.suspend = hibernate_early_suspend1;
	hb_early_suspend.early_suspend1.resume = hibernate_state_preparing1;
	register_early_suspend(&hb_early_suspend.early_suspend1);

	p = create_proc_entry("early_suspend_state",
				    0, hibernate_proc_dir);
        if (NULL == p) {
		goto out_early_suspend_state;
	}
	p->read_proc = hbe_read_proc;
	p->write_proc = NULL;
	p->data = NULL;

	return 0;
out_early_suspend_state:
	unregister_pm_notifier(&pm_state_suspend_notifier);
out_pm_unregister:
	unregister_hibernate_notifier(&hibernate_debug_notifier);
out_unregister:
	remove_proc_entry("hibernate_notify", hibernate_proc_dir);
out_hibernate_notify:
	remove_proc_entry("pm_state", hibernate_proc_dir);
out_pm_state:
	remove_proc_entry("hb_wakelock", hibernate_proc_dir);
out_hb_wakelock:
	remove_proc_entry("hb_event", hibernate_proc_dir);
out_hb_event:
	remove_proc_entry("hibernate", NULL);

	
err:	
	printk(KERN_ERR"%s %d error ret=%d\n", __FUNCTION__, __LINE__, ret);
	return -1;
}

static void __exit
deinit_hibernate_proc(void)
{
	remove_proc_entry("early_suspend_state", hibernate_proc_dir);
	remove_proc_entry("hibernate_notify", hibernate_proc_dir);
	remove_proc_entry("hb_event", hibernate_proc_dir);
	remove_proc_entry("hb_wakelock", hibernate_proc_dir);
	remove_proc_entry("pm_state", hibernate_proc_dir);
	remove_proc_entry("hibernate", NULL);
	unregister_hibernate_notifier(&hibernate_debug_notifier);
	unregister_early_suspend(&hb_early_suspend.early_suspend1);
	unregister_early_suspend(&hb_early_suspend.early_suspend0);
	unregister_pm_notifier(&pm_state_suspend_notifier);
}
//end

core_initcall(init_hibernate_proc);
module_exit(deinit_hibernate_proc);
