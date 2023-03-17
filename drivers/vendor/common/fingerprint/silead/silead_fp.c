/*
 * @file   silead_fp.c
 * @brief  Contains silead_fp device implementation.
 *
 *
 * Copyright 2016-2018 Slead Inc.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 *
 * ------------------- Revision History ------------------------------
 * <author>    <date>   <version>     <desc>
 * Bill Yu    2018/4/2    0.1.6      Init version
 * Bill Yu    2018/5/2    0.1.7      Fix compile error for some platform
 * Bill Yu    2018/5/20   0.1.8      Default wait 3ms after reset
 * Bill Yu    2018/5/28   0.1.9      Support poll/read if netlink id = 0
 * Bill Yu    2018/6/1    0.2.0      Support wakelock
 * Bill Yu    2018/6/5    0.2.1      Support chip enter power down
 * Bill Yu    2018/6/7    0.2.2      Support create proc node
 * Bill Yu    2018/6/27   0.2.3      Expand pwdn I/F
 * Bill Yu    2018/8/5    0.2.4      Support TP Up/Down I/F
 * Bill Yu    2018/12/5   0.2.5      Add Spreadtrum platform support
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/netlink.h>
#include <linux/cdev.h>
#include <linux/version.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/clk.h>
#include <net/sock.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/fb.h>

#include "silead_fp.h"

#define FP_DEV_NAME "silead_fp"
#define FP_DEV_MAJOR 0	/* assigned */

#define FP_CLASS_NAME "silead_fp"
#define FP_INPUT_NAME "fp-keys"

#define FP_DEV_VERSION "v0.2.5"
#define LOG_TAG "[+silead_fp-] "
#define PERM 0666
#define RETURN_NEG  (-1)

#define BSP_SIL_IRQ_ASYNC  /* IRQ use asynchrous mode. */

/* on some platform, home key has been redefine to KEY_HOMEPAGE! */
/* double check the define on customer board!!!*/
/*   out/target/product/.../system/usr/keylayout/Generic.kl*/
/*  kernel/include/uapi/linux/input.h*/

/*	KEY_HOMEPAGE */
/*	KEY_HOME     */
/*	KEY_MENU     */
/*	KEY_BACK     */
/*	KEY_POWER    */
/*	KEY_CAMERA    */
/*	KEY_VOLUMEUP */
/*	KEY_VOLUMEDOWN */

typedef struct _key_map {
	int key_orig;
	int key_new;
} nav_keymap_t;

#define SL_NAV_REAL_UP     0xFC
#define SL_NAV_REAL_DOWN   0xFB
#define SL_NAV_REAL_RIGHT  0xFE
#define SL_NAV_REAL_LEFT   0xFD


static nav_keymap_t keymap[] = {
	{ NAV_KEY_UP,          SL_NAV_REAL_UP,    }, /*KEY_RESERVED, ignore this key */
	{ NAV_KEY_DOWN,        SL_NAV_REAL_DOWN,  },
	{ NAV_KEY_RIGHT,       SL_NAV_REAL_RIGHT, },
	{ NAV_KEY_LEFT,        SL_NAV_REAL_LEFT,  },
	{ NAV_KEY_CLICK,       KEY_RESERVED,      },
	{ NAV_KEY_DCLICK,      KEY_RESERVED,      },
	{ NAV_KEY_LONGPRESS,   KEY_RESERVED,      },
};

typedef enum _fp_spi_speet_t {
	SPEED_1M = 1 * 1000 * 1000,
	SPEED_LOW = SPEED_1M,
	SPEED_5M = 5 * 1000 * 1000,
	SPEED_MEDIUM = SPEED_5M,
	SPEED_8M = 8 * 1000 * 1000,
	SPEED_9M = 9 * 1000 * 1000,
	SPEED_HIGH = SPEED_8M,
	SPEED_10M = 10 * 1000 * 1000,
} fp_spi_speet_t;


static struct fp_dev_init_t silfp_dev_init_d = {
	.mode = 0,
	.bits = 8,
	.speed = SPEED_HIGH,
	.delay = 100,
	.dev = DEVICE,
	.nl_id = SIFP_NETLINK_ROUTE,
	.dev_id = 0,
};

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

#ifdef BSP_SIL_NETLINK
static int pid;
#endif

#ifdef PROC_NODE
static char vendor_name[PROC_VND_ID_LEN];
#endif

struct class *silfp_class;

static struct workqueue_struct *silfp_wq;

static int silfp_wait_irq(struct silfp_data *fp_dev);
static int silfp_irq_status(struct silfp_data *fp_dev);
static int silfp_resource_deinit(struct silfp_data *fp_dev);

/*fp nav*/
static void sil_report_uevent(struct silfp_data *fp_dev, char *str)
{

	char *envp[2];

	LOG_MSG_DEBUG(D_LOG, "%s enter!\n", __func__);

	envp[0] = str;
	envp[1] = NULL;

	if (fp_dev->nav_dev) {
		LOG_MSG_DEBUG(I_LOG, "%s fp_dev->nav_dev is not null!\n", __func__);
		kobject_uevent_env(&(fp_dev->nav_dev->dev.kobj), KOBJ_CHANGE, envp);
	} else {
		LOG_MSG_DEBUG(E_LOG, "%s fp_dev->nav_dev is null!\n", __func__);
	}
}

#ifdef BSP_SIL_NETLINK
static void silfp_netlink_send(struct silfp_data *fp_dev, const int cmd)
{
	struct nlmsghdr *nlh = NULL;
	struct sk_buff *skb = NULL;
	int ret = 0;

	LOG_MSG_DEBUG(D_LOG, "[%s] enter, send cmd %d\n", __func__, cmd);

	if (!fp_dev->nl_sk) {
		LOG_MSG_DEBUG(E_LOG, "[%s] invalid socket\n", __func__);
		return;
	}

	if (!pid) {
		LOG_MSG_DEBUG(E_LOG, "[%s] invalid PID\n", __func__);
		return;
	}

	/*alloc data buffer for sending to native*/
	/*malloc data space at least 1500 bytes, which is ethernet data length*/
	skb = alloc_skb(NL_MSG_LEN, GFP_ATOMIC);
	if (skb == NULL) {
		return;
	}

	nlh = nlmsg_put(skb, 0, 0, 0, NL_MSG_LEN, 0);
	if (!nlh) {
		LOG_MSG_DEBUG(E_LOG, "[%s] nlmsg_put() failed\n", __func__);
		kfree_skb(skb);
		return;
	}

	NETLINK_CB(skb).portid = 0;
	NETLINK_CB(skb).dst_group = 0;

	*(char *)NLMSG_DATA(nlh) = cmd;
	ret = netlink_unicast(fp_dev->nl_sk, skb, pid, MSG_DONTWAIT);
	if (ret == 0) {
		LOG_MSG_DEBUG(E_LOG, "[%s] send failed\n", __func__);
		kfree_skb(skb);
		return;
	}
	LOG_MSG_DEBUG(D_LOG, "[%s] sent, len=%d\n", __func__, ret);
}

static void silfp_netlink_recv(struct sk_buff *__skb)
{
	struct sk_buff *skb = NULL;
	struct nlmsghdr *nlh = NULL;
	char str[128];

	LOG_MSG_DEBUG(D_LOG, "[%s] enter!\n", __func__);

	skb = skb_get(__skb);
	if (!skb) {
		LOG_MSG_DEBUG(E_LOG, "[%s] skb NULL\n", __func__);
		return;
	}

	/* presume there is 5byte payload at leaset */
	if (skb->len >= NLMSG_SPACE(0)) {
		nlh = nlmsg_hdr(skb);
		memcpy(str, NLMSG_DATA(nlh), sizeof(str));
		if ((strlen(str) == strlen("FP")) && !strncmp(str, "FP", strlen(str))) {
			pid = nlh->nlmsg_pid;
			LOG_MSG_DEBUG(D_LOG, "[%s] PID=%d, msg=%s\n", __func__, pid, str);
		} else {
			LOG_MSG_DEBUG(E_LOG, "[%s] Unknown PID=%d, msg=%s\n", __func__, nlh->nlmsg_pid, str);
		}
	} else {
		LOG_MSG_DEBUG(E_LOG, "[%s] data len incorrect\n", __func__);
	}

	kfree_skb(skb);
}

static int silfp_netlink_init(struct silfp_data *fp_dev)
{
	struct netlink_kernel_cfg cfg;

	LOG_MSG_DEBUG(D_LOG, "[%s] enter!\n", __func__);

	memset(&cfg, 0, sizeof(struct netlink_kernel_cfg));
	cfg.input = silfp_netlink_recv;
	fp_dev->nl_sk = netlink_kernel_create(&init_net, SIFP_NETLINK_ROUTE, &cfg);
	if (fp_dev->nl_sk == NULL) {
		LOG_MSG_DEBUG(E_LOG, "[%s] netlink create failed\n", __func__);
		return RETURN_NEG;
	}
	LOG_MSG_DEBUG(I_LOG, "[%s] netlink create success\n", __func__);
	return 0;
}

static int silfp_netlink_destroy(struct silfp_data *fp_dev)
{
	LOG_MSG_DEBUG(D_LOG, "[%s] enter!\n", __func__);

	if (fp_dev->nl_sk != NULL) {
		netlink_kernel_release(fp_dev->nl_sk);
		fp_dev->nl_sk = NULL;
		return 0;
	}
	LOG_MSG_DEBUG(E_LOG, "[%s] no netlink socket\n", __func__);
	return RETURN_NEG;
}
#else
static void silfp_netlink_send(struct silfp_data *fp_dev, const int cmd)
{
	unsigned long flags;
	struct silfp_msg_list *list;

	list = kzalloc(sizeof(*list), GFP_ATOMIC);
	if (!list) {
		return;
	}

	list->msg = (unsigned char)cmd;
	LOG_MSG_DEBUG(D_LOG, "silfp_poll cmd %d\n", cmd);

	spin_lock_irqsave(&fp_dev->read_lock, flags);
	list_add_tail(&list->list, &fp_dev->msg_q);
	spin_unlock_irqrestore(&fp_dev->read_lock, flags);

	wake_up_interruptible(&fp_dev->read_queue);
}

static int silfp_netlink_init(struct silfp_data *fp_dev)
{
	spin_lock_init(&fp_dev->read_lock);
	init_waitqueue_head(&fp_dev->read_queue);
	INIT_LIST_HEAD(&fp_dev->msg_q);

	return 0;
}

static int silfp_netlink_destroy(struct silfp_data *fp_dev)
{
	struct silfp_msg_list *list, *next;
	unsigned long flags;

	if (fp_dev && (&fp_dev->msg_q)) {
		spin_lock_irqsave(&fp_dev->read_lock, flags);
		list_for_each_entry_safe(list, next, &fp_dev->msg_q, list) {
			list_del(&list->list);
			kfree(list);
		}
		spin_unlock_irqrestore(&fp_dev->read_lock, flags);
	}
	return 0;
}

static unsigned int silfp_poll(struct file *fd, poll_table *wait)
{
	struct silfp_data *fp_dev;
	unsigned int  mask = 0;

	if (!fd) {
		return -EINVAL;
	}

	fp_dev = fd->private_data;
	poll_wait(fd, &fp_dev->read_queue, wait);

	if (!list_empty(&fp_dev->msg_q)) {
		mask |= POLLIN | POLLRDNORM;
	}

	return mask;
}

static ssize_t silfp_read(struct file *fd, char __user *buf, size_t len, loff_t *ptr)
{
	struct silfp_data *fp_dev;
	struct silfp_msg_list *list;
	unsigned long flags;
	int32_t msglen = sizeof(char);

	if (!fd || !buf || (len < msglen)) {
		return -EINVAL;
	}

	fp_dev = fd->private_data;
	if (!&fp_dev->msg_q) {
		return -EINVAL;
	}

	spin_lock_irqsave(&fp_dev->read_lock, flags);
	while (list_empty(&fp_dev->msg_q)) {
		spin_unlock_irqrestore(&fp_dev->read_lock, flags);
		if (wait_event_interruptible(fp_dev->read_queue, !list_empty(&fp_dev->msg_q))) {
			return -EINVAL;
		}
		spin_lock_irqsave(&fp_dev->read_lock, flags);
	}
	/* pick the first one */
	list = list_first_entry(&fp_dev->msg_q,
				struct silfp_msg_list, list);
	spin_unlock_irqrestore(&fp_dev->read_lock, flags);

	if (!list) {
		return -EFAULT;
	}

	if (copy_to_user(buf, &list->msg, msglen)) {
		LOG_MSG_DEBUG(E_LOG, "copy_to fail\n");
		msglen = -EFAULT;
	} else {
		LOG_MSG_DEBUG(D_LOG, "[%s] %d\n", __func__, list->msg);
	}
	spin_lock_irqsave(&fp_dev->read_lock, flags);
	list_del(&list->list);
	kfree(list);
	spin_unlock_irqrestore(&fp_dev->read_lock, flags);

	return msglen;
}
#endif


#ifdef CONFIG_HAS_EARLYSUSPEND
static void silfp_early_suspend(struct early_suspend *es)
{
	struct silfp_data *fp_dev = container_of(es, struct silfp_data, es);

	LOG_MSG_DEBUG(D_LOG, "[%s] enter\n", __func__);

	fp_dev->scr_off = 1;
	silfp_netlink_send(fp_dev, SIFP_NETLINK_SCR_OFF);
}

static void silfp_late_resume(struct early_suspend *es)
{
	struct silfp_data *fp_dev = container_of(es, struct silfp_data, es);

	LOG_MSG_DEBUG(D_LOG, "[%s] enter\n", __func__);

	fp_dev->scr_off = 0;
	silfp_netlink_send(fp_dev, SIFP_NETLINK_SCR_ON);
}
#else

static int silfp_fb_callback(struct notifier_block *notif,
			     unsigned long event, void *data)
{
	struct silfp_data *fp_dev = container_of(notif, struct silfp_data, notif);
	struct fb_event *evdata = data;
	unsigned int blank;
	int retval = 0;

	/* If we aren't interested in this event, skip it immediately ... */
	if (event != FB_EVENT_BLANK /* FB_EARLY_EVENT_BLANK */) {
		return 0;
	}

	blank = *(int *)evdata->data;

	LOG_MSG_DEBUG(D_LOG, "[%s] enter, blank=0x%x\n", __func__, blank);

	switch (blank) {
	case FB_BLANK_UNBLANK:
		LOG_MSG_DEBUG(D_LOG, "[%s] LCD ON\n", __func__);
		fp_dev->scr_off = 0;
		silfp_netlink_send(fp_dev, SIFP_NETLINK_SCR_ON);
		break;

	case FB_BLANK_POWERDOWN:
		LOG_MSG_DEBUG(D_LOG, "[%s] LCD OFF\n", __func__);
		fp_dev->scr_off = 1;
		silfp_netlink_send(fp_dev, SIFP_NETLINK_SCR_OFF);
		break;

	default:
		LOG_MSG_DEBUG(E_LOG, "[%s] Unknown notifier\n", __func__);
		break;
	}
	return retval;
}
#endif

void silfp_irq_disable(struct silfp_data *fp_dev)
{
	unsigned long irqflags;

	LOG_MSG_DEBUG(D_LOG, "[%s] enter\n", __func__);

	spin_lock_irqsave(&fp_dev->irq_lock, irqflags);
	if (!fp_dev->irq_is_disable) {
		fp_dev->irq_is_disable = 1;
		disable_irq_nosync(fp_dev->irq);
		LOG_MSG_DEBUG(I_LOG, "[%s] irq is disable\n", __func__);
	}
	spin_unlock_irqrestore(&fp_dev->irq_lock, irqflags);
}

void silfp_irq_enable(struct silfp_data *fp_dev)
{
	unsigned long irqflags = 0;

	LOG_MSG_DEBUG(D_LOG, "[%s] enter\n", __func__);

	spin_lock_irqsave(&fp_dev->irq_lock, irqflags);
	if (fp_dev->irq_is_disable) {
		enable_irq(fp_dev->irq);
		fp_dev->irq_is_disable = 0;
		reinit_completion(&fp_dev->done);
		LOG_MSG_DEBUG(I_LOG, "[%s] irq is enabled\n", __func__);
	}
	spin_unlock_irqrestore(&fp_dev->irq_lock, irqflags);
}

static int silfp_wait_irq(struct silfp_data *fp_dev)
{
	/* this function is obsolete, for test purpose only.*/
	wait_for_completion(&fp_dev->done);
	/*return wait_for_completion_timeout(&fp_dev->done, msecs_to_jiffies(3000));*/
	return 1;
}

static int silfp_irq_status(struct silfp_data *fp_dev)
{
	/* this function is obsolete, for test purpose only.*/
	if (fp_dev->int_port) {
		return gpio_get_value(fp_dev->int_port);
	}
	return RETURN_NEG;
}

irqreturn_t silfp_irq_handler(int irq, void *dev_id)
{
	struct silfp_data *fp_dev = (struct silfp_data *)dev_id;

	if (fp_dev->irq_ignore) {
		return IRQ_HANDLED;
	}

#ifdef BSP_SIL_IRQ_CONFIRM
	if (gpio_get_value(fp_dev->int_port)) {
#else
	if (true) {
#endif
		wake_lock_timeout(&fp_dev->wakelock, 2 * HZ); /* set a little long for a poor MCU */

#ifdef BSP_SIL_IRQ_ASYNC
		queue_work(silfp_wq, &fp_dev->work);
#else
		silfp_netlink_send(fp_dev, SIFP_NETLINK_IRQ);
#endif
		complete(&fp_dev->done);
	} else {
		LOG_MSG_DEBUG(I_LOG, "[%s] irq ignore\n", __func__);
	}

	return IRQ_HANDLED;
}

void silfp_work_func(struct work_struct *work)
{
	struct silfp_data *fp_dev = container_of(work, struct silfp_data, work);

	LOG_MSG_DEBUG(D_LOG, "[%s] is running\n", __func__);
	silfp_netlink_send(fp_dev, SIFP_NETLINK_IRQ);
}

static int silfp_keyevent(struct silfp_data *fp_dev, struct fp_dev_key_t *pkey)
{
	int ret = -EFAULT;

	LOG_MSG_DEBUG(D_LOG, "[%s] enter!\n", __func__);

	if (!fp_dev->input) {
		LOG_MSG_DEBUG(E_LOG, "[%s] invalid input device!\n", __func__);
		return RETURN_NEG;
	}
	LOG_MSG_DEBUG(I_LOG, "[%s] pkey->value %d, pkey->flag %d\n", __func__, pkey->value, pkey->flag);
	if (IS_KEY_VALID(pkey->value)) {
		/* Translate Click Down/Up key to Click key. */
		switch (pkey->value)
#ifdef NEW_NAV
		{
		case NAV_KEY_DOWN:
			sil_report_uevent(fp_dev, FP_NAV_DOWN);
			LOG_MSG_DEBUG(I_LOG, "%s nav down\n", __func__);
		break;

		case NAV_KEY_UP:
			sil_report_uevent(fp_dev, FP_NAV_UP);
			LOG_MSG_DEBUG(I_LOG, "%s nav up\n", __func__);
		break;

		case NAV_KEY_RIGHT:
			/*sil_report_uevent(fp_dev, FP_NAV_RIGHT);*/
			LOG_MSG_DEBUG(I_LOG, "%s nav right\n", __func__);
		break;

		case NAV_KEY_LEFT:
			/*sil_report_uevent(fp_dev, FP_NAV_LEFT);*/
			LOG_MSG_DEBUG(I_LOG, "%s nav left\n", __func__);
		break;

		default:
			LOG_MSG_DEBUG(E_LOG, "%s unknown nav event: %d\n", __func__, pkey->value);
		break;
		}
	}
	ret = 0;
#endif

#ifdef OLD_NAV
		{
		case NAV_KEY_CLICK_DOWN:
			pkey->value = NAV_KEY_CLICK;
			pkey->flag = NAV_KEY_FLAG_DOWN;
			break;
		case NAV_KEY_CLICK_UP:
			pkey->value = NAV_KEY_CLICK;
			pkey->flag = NAV_KEY_FLAG_UP;
			break;
		default:
			break;
		}

		/* Check the custom define keymap */
		if (fp_dev->keymap_cust.k[pkey->value - NAV_KEY_START]) {
			LOG_MSG_DEBUG(I_LOG, "[%s] custom-key %d\n", __func__,
				      fp_dev->keymap_cust.k[pkey->value - NAV_KEY_START]);
			if (fp_dev->keymap_cust.k[pkey->value - NAV_KEY_START] != KEY_RESERVED) {
				if (pkey->flag == NAV_KEY_FLAG_CLICK) {
					input_report_key(fp_dev->input,
					 fp_dev->keymap_cust.k[pkey->value - NAV_KEY_START], NAV_KEY_FLAG_DOWN);
					input_sync(fp_dev->input);
					input_report_key(fp_dev->input,
					 fp_dev->keymap_cust.k[pkey->value - NAV_KEY_START], NAV_KEY_FLAG_UP);
					input_sync(fp_dev->input);
				} else {
					input_report_key(fp_dev->input,
					fp_dev->keymap_cust.k[pkey->value - NAV_KEY_START], pkey->flag);
					input_sync(fp_dev->input);
				}
			} else {
				/* Here means this key is not set, simply ignore it.*/
			}
			ret = 0;
		}
	}

	for ( int i = 0 ; ret && i < ARRAY_SIZE(keymap) ; i++) {
		if (keymap[i].key_orig == pkey->value) {
			LOG_MSG_DEBUG(I_LOG, "[%s] key_orig %d, key_new %d\n", __func__, keymap[i].key_orig, keymap[i].key_new);
			if (keymap[i].key_new != KEY_RESERVED) {
				if (pkey->flag == NAV_KEY_FLAG_CLICK) {
					LOG_MSG_DEBUG(I_LOG, "[%s] input_report_key1 %d\n", __func__, keymap[i].key_new);
					input_report_key(fp_dev->input, keymap[i].key_new, NAV_KEY_FLAG_DOWN);
					input_sync(fp_dev->input);
					input_report_key(fp_dev->input, keymap[i].key_new, NAV_KEY_FLAG_UP);
					input_sync(fp_dev->input);
				} else {
					LOG_MSG_DEBUG(I_LOG, "[%s] input_report_key2 %d\n", __func__, keymap[i].key_new);
					input_report_key(fp_dev->input, keymap[i].key_new, pkey->flag);
					input_sync(fp_dev->input);
				}
			} else {
				/* Here means this key is not set, simply ignore it.*/
			}
			ret = 0;
		}
	}

	if (ret) {
		LOG_MSG_DEBUG(E_LOG, "[%s] unregister custom-key %d\n", __func__, pkey->value);
		input_report_key(fp_dev->input, pkey->value, pkey->flag);
		input_sync(fp_dev->input);
		ret = 0;
	}
#endif

	return ret;
}

#ifdef PROC_NODE
static int silfp_proc_show(struct seq_file *m, void *v)
{
	LOG_MSG_DEBUG(D_LOG, "[%s] enter!\n", __func__);

	seq_printf(m, "%s\n", vendor_name);
	return 0;
}

static int silfp_proc_open(struct inode *inode, struct file *file)
{
	LOG_MSG_DEBUG(D_LOG, "[%s] enter!\n", __func__);

	return single_open(file, silfp_proc_show, NULL);
}

static const struct file_operations silfp_proc_fops = {
	.owner = THIS_MODULE,
	.read  = seq_read,
	.open  = silfp_proc_open,
	.release = single_release,
};

static int silfp_proc_create_node(struct silfp_data *fp_dev)
{
	if (fp_dev->proc_entry == NULL) {
		fp_dev->proc_root = NULL;
#ifdef PROC_DIR
		fp_dev->proc_root = proc_mkdir(PROC_DIR, NULL);
		if (fp_dev->proc_root == NULL) {
			LOG_MSG_DEBUG(E_LOG, "Create dir %s under /proc error!\n", PROC_DIR);
			goto err_out;
		}
#endif
		fp_dev->proc_entry = proc_create(PROC_NODE, PERM, fp_dev->proc_root, &silfp_proc_fops);
		if (fp_dev->proc_entry == NULL) {
			LOG_MSG_DEBUG(E_LOG, "Create entry %s under /proc/ error!\n", PROC_NODE);
			goto err_out1;
		}
	}

	return 0;

err_out1:
	remove_proc_entry(PROC_NODE, fp_dev->proc_root);
#ifdef PROC_DIR
	remove_proc_entry(PROC_DIR, NULL);
err_out:
#endif
	fp_dev->proc_root = NULL;
	fp_dev->proc_entry = NULL;
	return -ENOMEM;
}

static int silfp_proc_init(struct silfp_data *fp_dev)
{
	if (fp_dev) {
		fp_dev->proc_root = NULL;
		fp_dev->proc_entry = NULL;
	}
	return 0;
}

static void silfp_proc_deinit(struct silfp_data *fp_dev)
{
	if (fp_dev) {
		if (fp_dev->proc_entry) {
			remove_proc_entry(PROC_NODE, fp_dev->proc_root);
		}
#ifdef PROC_DIR
		if (fp_dev->proc_root) {
			remove_proc_entry(PROC_DIR, NULL);
		}
#endif
		fp_dev->proc_root = NULL;
		fp_dev->proc_entry = NULL;
	}
}
#endif

int silfp_input_init(struct silfp_data *fp_dev)
{
	int i, status = 0;

	LOG_MSG_DEBUG(D_LOG, "[%s] enter!\n", __func__);

	fp_dev->input = input_allocate_device();
	if (!fp_dev->input) {
		LOG_MSG_DEBUG(E_LOG, "[%s] input_allocate_device() fail!\n", __func__);
		status = -ENOMEM;
		return status;
	}

	__set_bit(EV_KEY, fp_dev->input->evbit);
	/*__set_bit(KEY_Q, fp_dev->input->keybit); // it will cause Android think this is a physical keyboard.*/
	__set_bit(KEY_HOME, fp_dev->input->keybit);
	__set_bit(KEY_HOMEPAGE, fp_dev->input->keybit);

	__set_bit(KEY_MENU, fp_dev->input->keybit);
	__set_bit(KEY_BACK, fp_dev->input->keybit);
	__set_bit(KEY_CAMERA, fp_dev->input->keybit);

	for (i = 0; i < ARRAY_SIZE(keymap); i++) {
		if (keymap[i].key_new != KEY_RESERVED) {
			__set_bit(keymap[i].key_new, fp_dev->input->keybit);
		}
	}

	for (i = 0; i < ARRAY_SIZE(fp_dev->keymap_cust.k); i++) {
		if (fp_dev->keymap_cust.k[i] && (fp_dev->keymap_cust.k[i] != KEY_RESERVED)) {
			__set_bit(fp_dev->keymap_cust.k[i], fp_dev->input->keybit);
		}
	}

	fp_dev->input->name = FP_INPUT_NAME;
	if (input_register_device(fp_dev->input)) {
		LOG_MSG_DEBUG(E_LOG, "[%s] input_register_device() fail!\n", __func__);
		input_free_device(fp_dev->input);
		fp_dev->input = NULL;
		status = -ENODEV;
	}
	return status;
}

static int silfp_input_deinit(struct silfp_data *fp_dev)
{
	LOG_MSG_DEBUG(D_LOG, "[%s] enter!\n", __func__);

	if (fp_dev->input) {
		input_unregister_device(fp_dev->input);
		fp_dev->input = NULL;
	}
	return 0;
}

static int silfp_input_reinit(struct silfp_data *fp_dev)
{

	LOG_MSG_DEBUG(D_LOG, "[%s] enter!\n", __func__);

	if (fp_dev->input) {
		silfp_input_deinit(fp_dev);
	}
	return silfp_input_init(fp_dev);
}

static int silfp_init(struct silfp_data *fp_dev)
{
	int status = 0;

	LOG_MSG_DEBUG(D_LOG, "[%s] enter!\n", __func__);

	init_completion(&fp_dev->done);
	spin_lock_init(&fp_dev->irq_lock);
	INIT_WORK(&fp_dev->work, silfp_work_func);

	status = silfp_netlink_init(fp_dev);
	if (status == -1) {
		return status;
	}

#if defined(CONFIG_HAS_EARLYSUSPEND)
	LOG_MSG_DEBUG(I_LOG, "[%s] register_early_suspend\n", __func__);
	fp_dev->es.level = (EARLY_SUSPEND_LEVEL_DISABLE_FB - 1);
	fp_dev->es.suspend = silfp_early_suspend;
	fp_dev->es.resume = silfp_late_resume;
	register_early_suspend(&fp_dev->es);
#else
	/* register screen on/off callback */
	fp_dev->notif.notifier_call = silfp_fb_callback;
	fb_register_client(&fp_dev->notif);
#endif

	atomic_set(&fp_dev->spionoff_count, 0);

	return status;
}

static int silfp_resource_deinit(struct silfp_data *fp_dev)
{
	LOG_MSG_DEBUG(D_LOG, "[%s] enter!\n", __func__);

	if (atomic_read(&fp_dev->init)) {
		atomic_dec(&fp_dev->init);

		if (!atomic_read(&fp_dev->init)) {
			LOG_MSG_DEBUG(I_LOG, "[%s] no more users, free gpios\n", __func__);
			silfp_irq_disable(fp_dev);
			free_irq(fp_dev->irq, fp_dev);
			silfp_input_deinit(fp_dev);
			silfp_cleanup(fp_dev);
#ifdef PROC_NODE
			silfp_proc_deinit(fp_dev);
#endif
		}
		silfp_netlink_send(fp_dev, SIFP_NETLINK_DISCONNECT);
	}
	return 0;
}

static void silfp_exit(struct silfp_data *fp_dev)
{
	LOG_MSG_DEBUG(D_LOG, "[%s] enter!\n", __func__);

#ifdef CONFIG_HAS_EARLYSUSPEND
	if (fp_dev->es.suspend) {
		unregister_early_suspend(&fp_dev->es);
	}
#else
	fb_unregister_client(&fp_dev->notif);
#endif

	silfp_set_spi(fp_dev, false); /* release SPI resources */

	if (silfp_wq) {
		destroy_workqueue(silfp_wq);
		silfp_wq = NULL;
	}
	silfp_netlink_destroy(fp_dev);
#ifdef PROC_NODE
	silfp_proc_deinit(fp_dev);
#endif
}

static void silfp_wakelock_ctl(struct silfp_data *fp_dev, unsigned char lock)
{
	if (lock) {
		wake_lock_timeout(&fp_dev->wakelock_hal, 2 * HZ);
	} else {
		wake_unlock(&fp_dev->wakelock_hal);
	}
}

static long silfp_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	int retval = 0;
	struct silfp_data *fp_dev;
	struct fp_dev_key_t key;

	LOG_MSG_DEBUG(D_LOG, "[%s] enter!\n", __func__);

	/* Check type and command number */
	if (_IOC_TYPE(cmd) != SIFP_IOC_MAGIC)
		return -ENOTTY;

	/* Check access direction once here; don't repeat below.
	 * IOC_DIR is from the user perspective, while access_ok is
	 * from the kernel perspective; so they look reversed.
	 */
	if (_IOC_DIR(cmd) & _IOC_READ) {
		err = !access_ok(VERIFY_WRITE,
				 (void __user *)arg, _IOC_SIZE(cmd));
	}
	if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE) {
		err = !access_ok(VERIFY_READ,
				 (void __user *)arg, _IOC_SIZE(cmd));
	}
	if (err) {
		return -EFAULT;
	}

	/* guard against device removal before, or while,
	 * we issue this ioctl.
	 */
	fp_dev = filp->private_data;
	switch (cmd) {
	case SIFP_IOC_INIT:
		LOG_MSG_DEBUG(I_LOG, "[%s] SIFP_IOC_INIT\n", __func__);
		silfp_resource_init(fp_dev, &silfp_dev_init_d);
		if (copy_to_user((void __user *)arg, (void *)&silfp_dev_init_d, sizeof(struct fp_dev_init_t))) {
			retval = -EFAULT;
		}
		break;

	case SIFP_IOC_DEINIT:
		LOG_MSG_DEBUG(I_LOG, "[%s] SIFP_IOC_DEINIT\n", __func__);
		silfp_resource_deinit(fp_dev);
		break;

	case SIFP_IOC_RESET: {
		unsigned char delay = RESET_TIME;
		LOG_MSG_DEBUG(I_LOG, "[%s] SIFP_IOC_RESET\n", __func__);
		if (arg) {
			if (copy_from_user(&delay, (void __user *)arg, sizeof(char))) {
				retval = -EFAULT;
				break;
			}
		}

		if (fp_dev->power_is_off) {
			silfp_hw_poweron(fp_dev);
			mdelay(5);
		}

		fp_dev->irq_ignore = 1;
		if (fp_dev->irq_is_disable) {
			silfp_irq_enable(fp_dev);
			silfp_hw_reset(fp_dev, delay);
			silfp_irq_disable(fp_dev);
		} else {
			silfp_hw_reset(fp_dev, delay);
		}
		fp_dev->irq_ignore = 0;
	}
	break;

	case SIFP_IOC_ENABLE_IRQ:
		LOG_MSG_DEBUG(I_LOG, "[%s] SIFP_IOC_ENABLE_IRQ\n", __func__);
		silfp_irq_enable(fp_dev);
		break;

	case SIFP_IOC_DISABLE_IRQ:
		LOG_MSG_DEBUG(I_LOG, "[%s] SIFP_IOC_DISABLE_IRQ\n", __func__);
		silfp_irq_disable(fp_dev);
		break;

	case SIFP_IOC_CLR_IRQ:
		LOG_MSG_DEBUG(D_LOG, "[%s] SIFP_IOC_CLR_IRQ\n", __func__);
		reinit_completion(&fp_dev->done);
		break;

	case SIFP_IOC_WAIT_IRQ:
		LOG_MSG_DEBUG(D_LOG, "[%s] SIFP_IOC_WAIT_IRQ\n", __func__);
		retval = __put_user(silfp_wait_irq(fp_dev) ? 1 : 0, (__u8 __user *)arg);
		break;

	case SIFP_IOC_IRQ_STATUS:
		LOG_MSG_DEBUG(D_LOG, "[%s] SIFP_IOC_IRQ_STATUS\n", __func__);
		retval = __put_user((char)silfp_irq_status(fp_dev), (__u8 __user *)arg);
		break;

	case SIFP_IOC_KEY_EVENT:
		if (copy_from_user(&key, (struct fp_dev_key_t *)arg, sizeof(struct fp_dev_key_t))) {
			LOG_MSG_DEBUG(E_LOG, "[%s] copy key fail\n", __func__);
			retval = -EFAULT;
		} else {
			retval = silfp_keyevent(fp_dev, &key);
		}
		break;

	case SIFP_IOC_SCR_STATUS:
		if (arg) {
			LOG_MSG_DEBUG(D_LOG, "[%s] SIFP_IOC_SCR_STATUS put v = %d\n", __func__, (u8)(!fp_dev->scr_off));
			retval = __put_user((u8)(!fp_dev->scr_off), (__u8 __user *)arg);
		} else {
			LOG_MSG_DEBUG(D_LOG, "[%s] SIFP_IOC_SCR_STATUS send v = %d\n", __func__, (u8)(!fp_dev->scr_off));
			silfp_netlink_send(fp_dev, fp_dev->scr_off ? SIFP_NETLINK_SCR_OFF : SIFP_NETLINK_SCR_ON);
		}
		break;

	case SIFP_IOC_GET_VER:
		if (copy_to_user((void __user *)arg, (void *)FP_DEV_VERSION, sizeof(char) * 7)) {
			LOG_MSG_DEBUG(E_LOG, "[IOC_GET_VER] copy_to fail\n");
			retval = -EFAULT;
		}
		break;

	case SIFP_IOC_SET_KMAP:
		if (!arg) {
			retval = -EFAULT;
			break;
		}
		if (copy_from_user(&fp_dev->keymap_cust.k, (void __user *)arg, sizeof(struct fp_dev_kmap_t))) {
			LOG_MSG_DEBUG(E_LOG, "[IOC_SET_KMAP] copy_from fail\n");
			retval = -EFAULT;
			break;
		}

		if (silfp_input_reinit(fp_dev)) {
			retval = -EFAULT;
		}
		break;

	case SIFP_IOC_ACQ_SPI:
		LOG_MSG_DEBUG(I_LOG, "[%s] SIFP_IOC_ACQ_SPI\n", __func__);
#ifdef SIL_USE_SPI_BUS
		mt_spi_enable_master_clk(fp_dev->spi);
#else
		silfp_set_spi(fp_dev, true);
#endif
		break;

	case SIFP_IOC_RLS_SPI:
		LOG_MSG_DEBUG(I_LOG, "[%s] SIFP_IOC_RLS_SPI\n", __func__);
#ifdef SIL_USE_SPI_BUS
		mt_spi_enable_master_clk(fp_dev->spi);
#else
		silfp_set_spi(fp_dev, false);
#endif
		break;

	case SIFP_IOC_PKG_SIZE:
		if (arg) {
			LOG_MSG_DEBUG(D_LOG, "[IOC_PKG_SIZE] %d\n", (u8)(PKG_SIZE));
			retval = __put_user((u8)(PKG_SIZE), (__u8 __user *)arg);
		}
		break;

	case SIFP_IOC_DBG_LEVEL:
		LOG_MSG_DEBUG(D_LOG, "[%s] SIFP_IOC_DBG_LEVEL\n", __func__);
		if (arg) {
			unsigned char level = 0;

			if (copy_from_user(&level, (void __user *)arg, sizeof(char)) || level > (uint8_t)A_LOG) {
				retval = -EFAULT;
				break;
			}
			LOG_MSG_DEBUG(I_LOG, "SIFP_IOC_DBG_LEVEL %d-->%d\n", debug_level, level);
			retval = __put_user((u8)(debug_level), (__u8 __user *)arg);
			debug_level = (fp_debug_level_t)level;
		}
		break;

	case SIFP_IOC_WAKELOCK:
		LOG_MSG_DEBUG(D_LOG, "[%s] SIFP_IOC_WAKELOCK\n", __func__);
		if (arg) {
			unsigned char lock = 0;
			if (copy_from_user(&lock, (void __user *)arg, sizeof(char))) {
				LOG_MSG_DEBUG(E_LOG, "[SIFP_IOC_WAKELOCK] copy_from fail\n");
				retval = -EFAULT;
				break;
			}
			silfp_wakelock_ctl(fp_dev, lock);
		}
		break;

	case SIFP_IOC_PWDN:
		LOG_MSG_DEBUG(D_LOG, "[%s] SIFP_IOC_PWDN\n", __func__);
		if (arg) {
			unsigned char flag_avdd = 0;
			if (copy_from_user(&flag_avdd, (void __user *)arg, sizeof(char))) {
				LOG_MSG_DEBUG(E_LOG, "[SIFP_IOC_PWDN] copy_from fail\n");
				retval = -EFAULT;
				break;
			}
		}
		break;

#ifdef PROC_NODE
	case SIFP_IOC_PROC_NODE:
		if (arg) {
			if (copy_from_user(vendor_name, (void __user *)arg, PROC_VND_ID_LEN)) {
				LOG_MSG_DEBUG(E_LOG, "[SIFP_IOC_PROC_NODE] copy_from fail\n");
				retval = -EFAULT;
				break;
			}
			retval = silfp_proc_create_node(fp_dev);
		}
		break;
#endif

	default:
		retval = -ENOTTY;
		break;
	}

	return retval;
}

struct silfp_data *g_fp_dev;
int  silfp_touch_event_handler(struct fp_dev_touch_info *tp_info)
{
	static uint8_t lasttouchmode = 0;

	LOG_MSG_DEBUG(D_LOG, "[%s] enter!\n", __func__);

	if (g_fp_dev == NULL) {
		return 0;
	}
	LOG_MSG_DEBUG(D_LOG, "tp_info %d, %d, %d, %d\n",
		      tp_info->touch_state, tp_info->area_rate, tp_info->x, tp_info->y);
	if (tp_info->touch_state == lasttouchmode) {
		return 0;
	}

	if (tp_info->touch_state == 1) {
		LOG_MSG_DEBUG(D_LOG, "touch finger down\n");
		silfp_netlink_send(g_fp_dev, SIFP_NETLINK_TP_TOUCHDOWN);
		lasttouchmode = tp_info->touch_state;
	} else {
		LOG_MSG_DEBUG(D_LOG, "touch finger up\n");
		silfp_netlink_send(g_fp_dev, SIFP_NETLINK_TP_TOUCHUP);
		lasttouchmode = tp_info->touch_state;
	}

	wake_lock_timeout(&g_fp_dev->wakelock, 10 * HZ);
	return 0;
}
EXPORT_SYMBOL(silfp_touch_event_handler);

#ifdef CONFIG_COMPAT
static long
silfp_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	LOG_MSG_DEBUG(D_LOG, "[%s] enter!\n", __func__);
	return silfp_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define silfp_compat_ioctl NULL
#endif /* CONFIG_COMPAT */

static int silfp_open(struct inode *inode, struct file *filp)
{
	struct silfp_data *fp_dev;
	int status = -ENXIO;
	struct device_node *dnode = NULL;

	LOG_MSG_DEBUG(D_LOG, "[%s] enter\n", __func__);

	mutex_lock(&device_list_lock);

	list_for_each_entry(fp_dev, &device_list, device_entry) {
		if (fp_dev->devt == inode->i_rdev) {
			LOG_MSG_DEBUG(I_LOG, "sileadfp_dev Found!\n");
			status = 0;
			break;
		}
	}
       /*fp_nav*/
	dnode = of_find_compatible_node(NULL, NULL, "zte_fp_nav");
	if (dnode) {
		LOG_MSG_DEBUG(I_LOG, "fp-nav device node found!\n");
		fp_dev->nav_dev = of_find_device_by_node(dnode);
		if (fp_dev->nav_dev)
		{
			LOG_MSG_DEBUG(I_LOG, "fp-nav device uevent found!\n");
		} else {
			LOG_MSG_DEBUG(E_LOG, "fp-nav device uevent not found!\n");
		}
	} else {
		LOG_MSG_DEBUG(E_LOG, "fp-nav device node not found!\n");
	}

	if (status == 0) {
		fp_dev->users++;
		filp->private_data = fp_dev;
		nonseekable_open(inode, filp);
		LOG_MSG_DEBUG(I_LOG, "Succeed to open sileadfp_dev!\n");
	} else {
		LOG_MSG_DEBUG(E_LOG, "silfp: nothing for minor %d\n", iminor(inode));
	}

	mutex_unlock(&device_list_lock);

	return status;
}

static int silfp_release(struct inode *inode, struct file *filp)
{
	struct silfp_data *fp_dev;
	int status = 0;

	LOG_MSG_DEBUG(D_LOG, "[%s] enter\n", __func__);

	mutex_lock(&device_list_lock);
	fp_dev = filp->private_data;
	filp->private_data = NULL;

	/* last close? */
	fp_dev->users--;
	if (!fp_dev->users) {
		silfp_resource_deinit(fp_dev);
	}
	mutex_unlock(&device_list_lock);

	return status;
}

static const struct file_operations silfp_dev_fops = {
	.owner =	THIS_MODULE,
	/* REVISIT switch to aio primitives, so that userspace
	 * gets more complete API coverage.  It'll simplify things
	 * too, except for the locking.
	 */
	.unlocked_ioctl = silfp_ioctl,
	.compat_ioctl   = silfp_compat_ioctl,
	.open           = silfp_open,
	.release        = silfp_release,
#ifndef BSP_SIL_NETLINK
	.read           = silfp_read,
	.poll           = silfp_poll,
#endif
};

/* The main reason to have this class is to make mdev/udev create the
 * /dev/spidevB.C character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */

#if defined(SIL_USE_SPI_BUS)
static int silfp_probe(struct spi_device *spi)
#elif defined(SIL_USE_PLATFORM_BUS)
static int silfp_probe(struct platform_device *spi)
#endif
{
	struct silfp_data *fp_dev;
	int status = 0;

	/*unsigned long		minor;*/
	LOG_MSG_DEBUG(D_LOG, "[%s] enter!\n", __func__);

	fp_dev = kzalloc(sizeof(*fp_dev), GFP_KERNEL);
	if (!fp_dev) {
		return -ENOMEM;
	}

	fp_dev->spi = spi;

	fp_dev->nav_dev = NULL;/*fp_nav*/
	spin_lock_init(&fp_dev->spi_lock);

	wake_lock_init(&fp_dev->wakelock, WAKE_LOCK_SUSPEND, "silfp_wakelock");
	wake_lock_init(&fp_dev->wakelock_hal, WAKE_LOCK_SUSPEND, "silfp_wakelock_hal");
	INIT_LIST_HEAD(&fp_dev->device_entry);

	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */

	if (FP_DEV_MAJOR > 0) {
		fp_dev->devt = MKDEV(FP_DEV_MAJOR, fp_dev->ref++);
		status = register_chrdev_region(fp_dev->devt, 1, FP_DEV_NAME);
	} else {
		status = alloc_chrdev_region(&fp_dev->devt, fp_dev->ref++, 1, FP_DEV_NAME);
	}
	if (status < 0) {
		LOG_MSG_DEBUG(E_LOG, "[%s] request devt fail, ret=%d.\n", __func__, status);
		goto err_devt;
	} else {
		LOG_MSG_DEBUG(I_LOG, "[%s], major=%d, minor=%d\n",
			      __func__, MAJOR(fp_dev->devt), MINOR(fp_dev->devt));
	}

	fp_dev->dev = device_create(silfp_class, &fp_dev->spi->dev, fp_dev->devt, fp_dev, FP_DEV_NAME);
	if (IS_ERR(fp_dev->dev)) {
		LOG_MSG_DEBUG(E_LOG, "[%s] device_create() fail.\n", __func__);
		status = -ENODEV;
		goto err_dev;
	} else {
		mutex_lock(&device_list_lock);
		list_add(&fp_dev->device_entry, &device_list);
		mutex_unlock(&device_list_lock);
		LOG_MSG_DEBUG(I_LOG, "[%s] device create success!\n", __func__);
	}

	cdev_init(&fp_dev->cdev, &silfp_dev_fops);
	fp_dev->cdev.owner = THIS_MODULE;
	status = cdev_add(&fp_dev->cdev, fp_dev->devt, 1);
	if (status) {
		LOG_MSG_DEBUG(E_LOG, "[%s] cdev_add() fail, ret=%d.\n", __func__, status);
		goto err_cdev;
	}

	atomic_set(&fp_dev->init, 0);
	status = silfp_init(fp_dev);
	g_fp_dev = fp_dev;

	if (status) {
		LOG_MSG_DEBUG(E_LOG, "[%s] silfp_init fail ret=%d.\n", __func__, status);
		goto err_cdev;
	}
#ifdef PROC_NODE
	silfp_proc_init(fp_dev);
#endif

#if defined(SIL_USE_SPI_BUS)
	spi_set_drvdata(spi, fp_dev);
#elif defined(SIL_USE_PLATFORM_BUS)
	platform_set_drvdata(spi, fp_dev);
#endif
	LOG_MSG_DEBUG(I_LOG, "[%s]probe ok!\n", __func__);
	return status;

err_cdev:
	list_del(&fp_dev->device_entry);

err_dev:
	unregister_chrdev_region(fp_dev->devt, 1);

err_devt:
#if defined(SIL_USE_SPI_BUS)
	spi_set_drvdata(spi, NULL);
#elif defined(SIL_USE_PLATFORM_BUS)
	platform_set_drvdata(spi, NULL);
#endif
	fp_dev->spi = NULL;
	kfree(fp_dev);
	fp_dev = NULL;

	return status;
}

#if defined(SIL_USE_SPI_BUS)
static int silfp_remove(struct spi_device *spi)
{
	struct silfp_data *fp_dev = spi_get_drvdata(spi);
#elif defined(SIL_USE_PLATFORM_BUS)
static int silfp_remove(struct platform_device *spi)
{
	struct silfp_data *fp_dev = platform_get_drvdata(spi);
#endif
	LOG_MSG_DEBUG(I_LOG, "[%s] enter!\n", __func__);

	wake_lock_destroy(&fp_dev->wakelock);
	wake_lock_destroy(&fp_dev->wakelock_hal);

	spin_lock_irq(&fp_dev->spi_lock);
	fp_dev->spi = NULL;
	spin_unlock_irq(&fp_dev->spi_lock);

	silfp_exit(fp_dev);

	mutex_lock(&device_list_lock);
	list_del(&fp_dev->device_entry);
	device_destroy(silfp_class, fp_dev->devt);
	if (fp_dev->users == 0)
		kfree(fp_dev);
	mutex_unlock(&device_list_lock);
	LOG_MSG_DEBUG(I_LOG, "[%s] exit!\n", __func__);
	return 0;
}

static const struct of_device_id sildev_dt_ids[] = {
	{ .compatible = "sil,silead_fp" },
	{},
};

#if defined(SIL_USE_SPI_BUS)
static struct spi_driver silfp_driver = {
#elif defined(SIL_USE_PLATFORM_BUS)
static struct platform_driver silfp_driver = {
#endif
	.driver = {
		.name  = "silead_fp",
		.owner = THIS_MODULE,
		.of_match_table = sildev_dt_ids,
	},
	.probe  = silfp_probe,
	.remove = silfp_remove,
};

static int __init silfp_dev_init(void)
{
	int status = 0;

	LOG_MSG_DEBUG(D_LOG, "silead_fp_driver, version: %s!\n", FP_DEV_VERSION);

	status = register_chrdev(FP_DEV_MAJOR, "sil", &silfp_dev_fops);
	if (status < 0) {
		LOG_MSG_DEBUG(E_LOG, "[%s] register_chrdev  failed! status = %d\n", __func__, status);
		return status;
	}

	silfp_class = class_create(THIS_MODULE, "silead_fp");

	if (IS_ERR(silfp_class)) {
		LOG_MSG_DEBUG(E_LOG, "[%s] class_create silfp_class failed!\n", __func__);
		unregister_chrdev(FP_DEV_MAJOR, silfp_driver.driver.name);
		return PTR_ERR(silfp_class);
	}
#if defined(SIL_USE_SPI_BUS)
	status = spi_register_driver(&silfp_driver);
	LOG_MSG_DEBUG(I_LOG, "----silead spi_register_driver ok----status : %d\n", status);
#elif defined(SIL_USE_PLATFORM_BUS)
	status = platform_driver_register(&silfp_driver);
	LOG_MSG_DEBUG(I_LOG, "----silead platform_driver_register ok----status : %d\n", status);
#endif
	if (status < 0) {
		class_destroy(silfp_class);
		unregister_chrdev(FP_DEV_MAJOR, silfp_driver.driver.name);
		LOG_MSG_DEBUG(E_LOG, "[%s] register_driver fail! ret=%d.\n", __func__, status);
		return status;
	}
	silfp_wq = create_singlethread_workqueue("silfp_wq");
	LOG_MSG_DEBUG(I_LOG, "[%s]  dev_init success!\n", __func__);
	return status;
}
module_init(silfp_dev_init);

static void __exit silfp_dev_exit(void)
{
	LOG_MSG_DEBUG(D_LOG, "[%s] enter!\n", __func__);

#if defined(SIL_USE_SPI_BUS)
	spi_unregister_driver(&silfp_driver);
#elif defined(SIL_USE_PLATFORM_BUS)
	platform_driver_unregister(&silfp_driver);
#endif
	class_destroy(silfp_class);
	unregister_chrdev(FP_DEV_MAJOR, silfp_driver.driver.name);
}
module_exit(silfp_dev_exit);

MODULE_AUTHOR("Bill Yu <billyu@silead.com>");
MODULE_DESCRIPTION("Silead Fingerprint driver for GSL61XX/GSL62XX series.");
MODULE_LICENSE("GPL");
MODULE_ALIAS("sil:silead_fp");
