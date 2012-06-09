/* 
 * Copyright (C) 2009 Google.
 * Copyright (C) 2009 HTC Corporation.
 *
 * Based on board-mahimahi-microp.c
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
*/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/input.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/mutex.h>
#include <linux/jiffies.h>
#include <linux/wakelock.h>
#include <linux/earlysuspend.h>
#include <linux/bma150.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>
#include <mach/mmc.h>
#include <asm/setup.h>
#include <mach/htc_pwrsink.h>
#include "board-htcrhodium.h"
#include <mach/htc_headset.h>

#define DRIVER_NAME "HS_MICROP"

static struct i2c_client *private_microp_client;
static struct wake_lock microp_i2c_wakelock;

#define SYS_ERR(fmt, arg...) \
        printk(KERN_ERR "[" DRIVER_NAME "] (%s) " fmt "\n", __func__, ## arg)

#define SYS_MSG(fmt, arg...) \
        printk(KERN_INFO "[" DRIVER_NAME "] (%s) " fmt "\n", __func__, ## arg)
#if 0
#define DBG_MSG(fmt, arg...) \
        printk(KERN_INFO "##### [" DRIVER_NAME "] (%s) " fmt "\n", \
               __func__, ## arg)
#else
#define DBG_MSG(fmt, arg...) {}
#endif

#define MICROP_I2C_WCMD_GPI_INT_CTL_EN		0x80
#define MICROP_I2C_WCMD_GPI_INT_CTL_DIS		0x81
#define MICROP_I2C_RCMD_GPI_INT_STATUS		0x82
#define MICROP_I2C_RCMD_GPI_STATUS		0x83
#define MICROP_I2C_WCMD_GPI_INT_STATUS_CLR	0x84
#define MICROP_I2C_RCMD_GPI_INT_SETTING		0x85


// TODO: These definitions need to be moved to htc_headset_microp.h.
struct htc_headset_microp_platform_data {
	/* Headset detection */
	int hpin_int;
	unsigned int hpin_irq;
	uint8_t hpin_mask[3];

	/* Remote key detection */
	int remote_int;
	unsigned int remote_irq;

	/* Remote key interrupt enable */
	unsigned int remote_enable_pin;
};
struct microp_i2c_work {
	struct work_struct work;
	struct i2c_client *client;
};

struct htc_headset_microp_info {
	struct microp_i2c_work work;
	struct delayed_work hpin_debounce_work;
	int headset_is_in;
	int is_hpin_pin_stable;
	struct htc_headset_microp_platform_data pdata;
	int hpin_gpio_mask;
	unsigned int hpin_debounce;
	unsigned long hpin_jiffies;
};
static struct htc_headset_microp_info *hi;
 
static char *hex2string(uint8_t *data, int len)
{
	static char buf[101];
	int i;

	i = (sizeof(buf) - 1) / 4;
	if (len > i)
		len = i;

	for (i = 0; i < len; i++)
		sprintf(buf + i * 4, "[%02X]", data[i]);

	return buf;
}

#define I2C_READ_RETRY_TIMES  10
#define I2C_WRITE_RETRY_TIMES 10

static int i2c_read_block(uint8_t addr, uint8_t *data, int length)
{
	int retry;
	int ret;
	struct i2c_msg msgs[] = {
	{
		.addr = private_microp_client->addr,
		.flags = 0,
		.len = 1,
		.buf = &addr,
	},
	{
		.addr = private_microp_client->addr,
		.flags = I2C_M_RD,
		.len = length,
		.buf = data,
	}
	};

	mdelay(1);
	for (retry = 0; retry <= I2C_READ_RETRY_TIMES; retry++) {
		ret = i2c_transfer(private_microp_client->adapter, msgs, 2);
		if (ret == 2) {
			DBG_MSG("R [%02X] = %s", addr,hex2string(data, length));
			return 0;
		}
		msleep(10);
	}

	SYS_ERR("i2c_read_block retry over %d",I2C_READ_RETRY_TIMES);
	return -EIO;
}

#define MICROP_I2C_WRITE_BLOCK_SIZE 21
static int i2c_write_block(uint8_t addr, uint8_t *data, int length)
{
	int retry;
	uint8_t buf[MICROP_I2C_WRITE_BLOCK_SIZE];
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr = private_microp_client->addr,
			.flags = 0,
			.len = length + 1,
			.buf = buf,
		}
	};

	DBG_MSG("W [%02X] = %s", addr,hex2string(data, length));

	if (length + 1 > MICROP_I2C_WRITE_BLOCK_SIZE) {
		SYS_ERR("i2c_write_block length too long");
		return -E2BIG;
	}

	buf[0] = addr;
	memcpy((void *)&buf[1], (void *)data, length);

	mdelay(1);
	for (retry = 0; retry <= I2C_WRITE_RETRY_TIMES; retry++) {
		ret = i2c_transfer(private_microp_client->adapter, msg, 1);
		if (ret == 1)
			return 0;
		msleep(10);
	}
	SYS_ERR("i2c_write_block retry over %d",
			I2C_WRITE_RETRY_TIMES);
	return -EIO;
}

/**
 * GPI functions
 **/

static int microp_read_gpi_status(uint16_t *status)
{
	uint8_t data[2];
	int ret;

	ret = i2c_read_block(MICROP_I2C_RCMD_GPI_STATUS, data, 2);
	if (ret < 0) {
		SYS_ERR(": read failed");
		return -EIO;
	}
	*status = (data[0] << 8) | data[1];
	return 0;
}

static int microp_interrupt_enable(uint16_t interrupt_mask)
{
	uint8_t data[2];
	int ret = -1;

	if (!private_microp_client)	{
		SYS_ERR(":dataset: client is empty");
		return -EIO;
	}

	data[0] = interrupt_mask >> 8;
	data[1] = interrupt_mask & 0xFF;
	ret = i2c_write_block(MICROP_I2C_WCMD_GPI_INT_CTL_EN, data, 2);

	if (ret < 0)
		SYS_ERR(": enable 0x%x interrupt failed", interrupt_mask);
	return ret;
}

static int microp_interrupt_disable(uint16_t interrupt_mask)
{
	uint8_t data[2];
	int ret = -1;

	if (!private_microp_client)	{
		SYS_ERR(":dataset: client is empty");
		return -EIO;
	}

	data[0] = interrupt_mask >> 8;
	data[1] = interrupt_mask & 0xFF;
	ret = i2c_write_block(MICROP_I2C_WCMD_GPI_INT_CTL_DIS, data, 2);

	if (ret < 0)
		SYS_ERR(": disable 0x%x interrupt failed", interrupt_mask);
	return ret;
}

/*
 *Headset Support
*/
static void hpin_debounce_do_work(struct work_struct *work)
{
	uint16_t gpi_status = 0;

	int insert = 0;

	microp_read_gpi_status(&gpi_status);
	insert = (gpi_status & hi->hpin_gpio_mask) ? 1 : 0;

	DBG_MSG(": gpi_status=0x%x hpin_gpio_mask=0x%x insert=%d headset_is_in=%d\n",
		 gpi_status, hi->hpin_gpio_mask, insert, hi->headset_is_in);

	if (insert != hi->headset_is_in) {
		hi->headset_is_in = insert;
		htc_35mm_jack_plug_event(hi->headset_is_in);
		hi->is_hpin_pin_stable = 1;
	}
}
 
/*
 * Interrupt
 */
static irqreturn_t microp_i2c_intr_irq_handler(int irq, void *dev_id)
{
	disable_irq_nosync(hi->pdata.hpin_irq);
	//disable_irq_nosync(private_microp_client->irq);
	schedule_work(&hi->work.work);
	return IRQ_HANDLED;
}

static void microp_i2c_intr_work_func(struct work_struct *work)
{
	uint8_t data[3];
	uint16_t intr_status = 0;
	int ret = 0;


	ret = i2c_read_block(MICROP_I2C_RCMD_GPI_INT_STATUS, data, 2);
	if (ret < 0) {
		SYS_ERR(": read interrupt status fail\n");
	}

	intr_status = data[0]<<8 | data[1];
	ret = i2c_write_block(MICROP_I2C_WCMD_GPI_INT_STATUS_CLR,
			      data, 2);
	if (ret < 0) {
		SYS_ERR(": clear interrupt status fail\n");
	}
	
	DBG_MSG(": microp intr_status=0x%02x\n", intr_status);

	if (intr_status & hi->pdata.hpin_int) {
		hi->is_hpin_pin_stable = 0;
		wake_lock_timeout(&microp_i2c_wakelock, 3*HZ);
		if (!hi->headset_is_in)
			schedule_delayed_work(&hi->hpin_debounce_work,
					msecs_to_jiffies(500));
		else
			schedule_delayed_work(&hi->hpin_debounce_work,
					msecs_to_jiffies(300));
	}

	enable_irq(hi->pdata.hpin_irq);
	//enable_irq(private_microp_client->irq);
}


static int htc_headset_microp_probe(struct platform_device *pdev)
{
	int ret;
	struct htc_headset_microp_platform_data *pdata = NULL;

	pdata = pdev->dev.platform_data;
	private_microp_client = dev_get_drvdata(&pdev->dev);

	printk("%s: addr=0x%02x\n", __func__, private_microp_client->addr);

	hi = kzalloc(sizeof(struct htc_headset_microp_info), GFP_KERNEL);
	if (!hi) {
		SYS_MSG("Failed to allocate memory for headset info");
		return -ENOMEM;
	}

	hi->pdata.hpin_int = pdata->hpin_int;
	hi->pdata.hpin_irq = pdata->hpin_irq;
	if (pdata->hpin_mask[0] || pdata->hpin_mask[1] || pdata->hpin_mask[2])
		memcpy(hi->pdata.hpin_mask, pdata->hpin_mask,
		       sizeof(hi->pdata.hpin_mask));

	//TODO: To avoid reading the board h file we should pass the gpio
	ret = gpio_request(RHODIUM_GPIO_UP_RESET_N, "microp_i2c_wm");
	if (ret < 0) {
		SYS_ERR("failed on request gpio reset");
		goto err_exit;
	}
	ret = gpio_direction_output(RHODIUM_GPIO_UP_RESET_N, 0);
	if (ret < 0) {
		SYS_ERR("failed on gpio_direction_output reset");
		goto err_gpio_reset;
	}

	wake_lock_init(&microp_i2c_wakelock, WAKE_LOCK_SUSPEND,
			 "microp_i2c_present");

	hi->headset_is_in = 0;
	hi->is_hpin_pin_stable = 1;
	hi->work.client = private_microp_client;

	if (hi->pdata.hpin_int) {
		hi->hpin_gpio_mask = pdata->hpin_mask[0] << 16 |
				     pdata->hpin_mask[1] << 8 |
				     pdata->hpin_mask[2];
	}

	if (hi->pdata.hpin_irq) {
		INIT_WORK(&hi->work.work, microp_i2c_intr_work_func);

		ret = request_irq(hi->pdata.hpin_irq,
				  microp_i2c_intr_irq_handler,
				  IRQF_TRIGGER_LOW,
				  "microp_interrupt", NULL);
		if (ret < 0) {
			ret = -EINVAL;
			SYS_ERR("Failed to request Micro-P IRQ (ERROR %d)",
				ret);
			goto err_intr;
		}
		ret = irq_set_irq_wake(hi->pdata.hpin_irq, 1);
		if (ret) {
			SYS_ERR("irq_set_irq_wake failed");
			goto err_intr;
		}
	}

	if (hi->pdata.hpin_int) {
		ret = microp_interrupt_enable(hi->pdata.hpin_int);
		if (ret != 0) {
			SYS_MSG("Failed to enable Micro-P HPIN interrupt");
			goto err_intr;
		}
	}

	INIT_DELAYED_WORK(
		&hi->hpin_debounce_work, hpin_debounce_do_work);

	printk("%s: done\n", __func__);
	return 0;

err_intr:
	wake_lock_destroy(&microp_i2c_wakelock);
err_gpio_reset:
	gpio_free(RHODIUM_GPIO_UP_RESET_N);
err_exit:
	return ret;
}

static int htc_headset_microp_remove(struct platform_device *pdev)
{
	if (hi->pdata.remote_irq)
		free_irq(hi->pdata.remote_irq, 0);

	if (hi->pdata.remote_int)
		microp_interrupt_disable(hi->pdata.remote_int);

	if (hi->pdata.hpin_irq)	
		free_irq(hi->pdata.hpin_irq, 0);

	if (hi->pdata.hpin_int)
		microp_interrupt_disable(hi->pdata.hpin_int);

	kfree(hi);
	return 0;
}

static struct platform_driver htc_headset_microp_driver = {
	.probe	= htc_headset_microp_probe,
	.remove	= htc_headset_microp_remove,
	.driver	= {
		.name	= "HTC_HEADSET_MICROP",
		.owner	= THIS_MODULE,
	},
};

static int __init htc_headset_microp_init(void)
{
	return platform_driver_register(&htc_headset_microp_driver);
}

static void __exit htc_headset_microp_exit(void)
{
	platform_driver_unregister(&htc_headset_microp_driver);
}

module_init(htc_headset_microp_init);
module_exit(htc_headset_microp_exit);

MODULE_DESCRIPTION("HTC Micro-P headset detection driver");
MODULE_LICENSE("GPL");
