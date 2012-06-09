/* arch/arm/mach-msm/smd_tty.c
 *
 * Copyright (C) 2007 Google, Inc.
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

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/wakelock.h>
#include <linux/slab.h>

#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>

#include <mach/msm_smd.h>
#include <asm/mach-types.h>

#define MAX_SMD_TTYS 32
#define SMD_BUF_SIZE (16 * 1024)

static DEFINE_MUTEX(smd_tty_lock);

struct smd_tty_info {
	smd_channel_t *ch;
	struct tty_struct *tty;
	struct wake_lock wake_lock;
	int open_count;
	struct delayed_work tty_delayed_work;
	unsigned char *rx_buffer;
};

static struct smd_tty_info smd_tty[MAX_SMD_TTYS];
static struct workqueue_struct *smd_tty_wq;

/**
 * line discipline callback wrappers
 *
 * The wrappers maintain line discipline references
 * while calling into the line discipline.
 *
 * ldisc_receive_buf  - pass receive data to line discipline
 */

static void smd_readx_cb(void *data, int count, void *ctxt)
{
	struct tty_struct *tty = ctxt;
	struct tty_ldisc *ld;
	if (!tty)
		return;
	ld = tty_ldisc_ref(tty);
	if (ld) {
		if (ld->ops->receive_buf)
			ld->ops->receive_buf(tty, data, 0, count);
		tty_ldisc_deref(ld);
	}
}

static void smd_tty_work_func(struct work_struct *work)
{
	unsigned char *ptr;
        int avail;
        int fail_cnt=0;
        int exit_loop=0;

        struct delayed_work *delayed_work = container_of(work,
                                                struct delayed_work,
                                                work);

        struct smd_tty_info *info = container_of(delayed_work,
                                                struct smd_tty_info,
                                                tty_delayed_work);

	struct tty_struct *tty = info->tty;

	if (!tty)
		return;

	for (;;) {
		if (test_bit(TTY_THROTTLED, &tty->flags))
			break;

		mutex_lock(&smd_tty_lock);
		if (info->ch == 0) {
			mutex_unlock(&smd_tty_lock);
			break;
		}

		avail = smd_read_avail(info->ch);
		if (avail == 0) {
			mutex_unlock(&smd_tty_lock);
			break;
		}

		if(tty->index==7)
		{
			if (smd_readx(info->ch, avail, smd_readx_cb, (void *)tty) != avail) {
				/* shouldn't be possible since we're in interrupt
				** context here and nobody else could 'steal' our
				** characters.
				*/
				printk(KERN_ERR "OOPS - smd_tty_buffer mismatch?!\n");
			}
		}else{
 			avail = tty_prepare_flip_string(tty, &ptr, avail);
           		if ( avail == 0)
                	{
                               	if(fail_cnt)
                        	{
                                	if(smd_read_avail(info->ch) > 2048)
                                	        avail = tty_prepare_flip_string(tty, &ptr, 2048);
                                	//TODO: this is nice but it can cause the the size of the required tty realloc to get bigger
                                	// and bigger since our smd channel is collecting more and more bytes but we have failed to 
                                	// deliver any of them to userland. Leading to permanent failure of the smd channel. 
                                	// we should probably try to send smaller chunks to prevent the realloc all togeather. 


                                	//We have had about enough
                                	//need to get started up later because we have already been notified of a non empty
                                	//buffer, unforunately the SLAB is not going to give in atm
                                	queue_delayed_work(smd_tty_wq, &info->tty_delayed_work,HZ/50);
                                	printk("SMD%s: tty induced slab failure. high probability of smd pipe freeze. chan %d, needed %d, smallish %d\n",__func__, tty->index, smd_read_avail(info->ch),avail);
                                	if(avail==0)
                                        	exit_loop=1;
                        	}
                        fail_cnt++;
			}

          		if (avail && smd_read(info->ch, ptr, avail) != avail) {
                        	/* shouldn't be possible since we're in interrupt
                        	** context here and nobody else could 'steal' our
                        	** characters.
                        	*/
                        	printk(KERN_ERR "OOPS - smd_tty_buffer mismatch?!");
                	}
                }


		if(tty->index!=7)
		{
               		tty_flip_buffer_push(tty);
		}

		wake_lock_timeout(&info->wake_lock, HZ / 2);
		mutex_unlock(&smd_tty_lock);

                if(exit_loop)
                        break;

	}

	/* XXX only when writable and necessary */
	tty_wakeup(tty);
}

static void smd_tty_notify(void *priv, unsigned event)
{
	struct smd_tty_info *info = priv;

	if (event == SMD_EVENT_CLOSE)
		tty_hangup(info->tty);
	if (event != SMD_EVENT_DATA)
		return;

        queue_delayed_work(smd_tty_wq, &info->tty_delayed_work,0);

}

static int smd_tty_open(struct tty_struct *tty, struct file *f)
{
	int res = 0;
	int n = tty->index;
	struct smd_tty_info *info;
	const char *name;

	if(machine_is_htcdiamond() || machine_is_htcraphael() ||
			machine_is_htcblackstone() || machine_is_htctopaz() ||
			machine_is_htcrhodium() || machine_is_htckovsky()) {
		if(n==1) n=7; // map 7 to 1 for android compatability, on GSM
	}

	if (n == 0) {
		name = "SMD_DS";
	} else if (n == 1) {
		name = "SMD_DIAG";
	} else if (n == 7) {
		name = "SMD_DATA1";
	} else if (n == 27) {
		name = "SMD_GPSNMEA";
	} else {
		return -ENODEV;
	}

	
	info = smd_tty + n;

	mutex_lock(&smd_tty_lock);
	wake_lock_init(&info->wake_lock, WAKE_LOCK_SUSPEND, name);
	tty->driver_data = info;

	if (info->open_count++ == 0) {
		info->tty = tty;
		if (info->ch) {
			smd_kick(info->ch);
		} else {
			res = smd_open(name, &info->ch, info, smd_tty_notify);
		}
	}
	mutex_unlock(&smd_tty_lock);

	return res;
}

static void smd_tty_close(struct tty_struct *tty, struct file *f)
{
	struct smd_tty_info *info = tty->driver_data;

	if (info == 0)
		return;

	mutex_lock(&smd_tty_lock);
	if (--info->open_count == 0) {
		info->tty = 0;
		tty->driver_data = 0;
		wake_lock_destroy(&info->wake_lock);
		if (info->ch) {
			smd_close(info->ch);
			info->ch = 0;
		}
	}
	mutex_unlock(&smd_tty_lock);
}

static int smd_tty_write(struct tty_struct *tty, const unsigned char *buf, int len)
{
	struct smd_tty_info *info = tty->driver_data;
	int avail;

	/* if we're writing to a packet channel we will
	** never be able to write more data than there
	** is currently space for
	*/
	avail = smd_write_avail(info->ch);
	if (len > avail)
		len = avail;

	return smd_write(info->ch, buf, len);
}

static int smd_tty_write_room(struct tty_struct *tty)
{
	struct smd_tty_info *info = tty->driver_data;
	return smd_write_avail(info->ch);
}

static int smd_tty_chars_in_buffer(struct tty_struct *tty)
{
	struct smd_tty_info *info = tty->driver_data;
	return smd_read_avail(info->ch);
}

static void smd_tty_unthrottle(struct tty_struct *tty)
{
	struct smd_tty_info *info = tty->driver_data;
        queue_delayed_work(smd_tty_wq, &info->tty_delayed_work,0);
	return;
}

static struct tty_operations smd_tty_ops = {
	.open = smd_tty_open,
	.close = smd_tty_close,
	.write = smd_tty_write,
	.write_room = smd_tty_write_room,
	.chars_in_buffer = smd_tty_chars_in_buffer,
	.unthrottle = smd_tty_unthrottle,
};

static struct tty_driver *smd_tty_driver;

static int __init smd_tty_init(void)
{
	int ret;

	smd_tty_wq = create_singlethread_workqueue("smd_tty");
	if (smd_tty_wq == 0)
		return -ENOMEM;

	smd_tty_driver = alloc_tty_driver(MAX_SMD_TTYS);
	if (smd_tty_driver == 0) {
		destroy_workqueue(smd_tty_wq);
		return -ENOMEM;
	}

	smd_tty_driver->owner = THIS_MODULE;
	smd_tty_driver->driver_name = "smd_tty_driver";
	smd_tty_driver->name = "smd";
	smd_tty_driver->major = 0;
	smd_tty_driver->minor_start = 0;
	smd_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	smd_tty_driver->subtype = SERIAL_TYPE_NORMAL;
	smd_tty_driver->init_termios = tty_std_termios;
	smd_tty_driver->init_termios.c_iflag = 0;
	smd_tty_driver->init_termios.c_oflag = 0;
	smd_tty_driver->init_termios.c_cflag = B115200 | CS8 | CREAD;
	smd_tty_driver->init_termios.c_lflag = 0;
	smd_tty_driver->flags = TTY_DRIVER_RESET_TERMIOS |
		TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	tty_set_operations(smd_tty_driver, &smd_tty_ops);

	ret = tty_register_driver(smd_tty_driver);
	if (ret) return ret;

	/* this should be dynamic */
	tty_register_device(smd_tty_driver, 0, 0);
	INIT_DELAYED_WORK(&smd_tty[0].tty_delayed_work, smd_tty_work_func);
	smd_tty[1].rx_buffer = (unsigned char *)kmalloc(SMD_BUF_SIZE,GFP_KERNEL);
	tty_register_device(smd_tty_driver, 1, 0);
	INIT_DELAYED_WORK(&smd_tty[1].tty_delayed_work, smd_tty_work_func);
	smd_tty[1].rx_buffer = (unsigned char *)kmalloc(SMD_BUF_SIZE,GFP_KERNEL);
	tty_register_device(smd_tty_driver, 7, 0);
	INIT_DELAYED_WORK(&smd_tty[7].tty_delayed_work, smd_tty_work_func);
	smd_tty[7].rx_buffer = (unsigned char *)kmalloc(SMD_BUF_SIZE,GFP_KERNEL);
	tty_register_device(smd_tty_driver, 27, 0);
	INIT_DELAYED_WORK(&smd_tty[27].tty_delayed_work, smd_tty_work_func);
	smd_tty[27].rx_buffer = (unsigned char *)kmalloc(SMD_BUF_SIZE,GFP_KERNEL);

	return 0;
}

module_init(smd_tty_init);
