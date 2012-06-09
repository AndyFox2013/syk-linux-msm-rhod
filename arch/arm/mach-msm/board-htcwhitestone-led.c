 /* linux/arch/arm/mach-msm/board-htcwhitestone-led.c
 *
 * Copyright (2011) Oliver Gjoneski <ogjoneski@gmail.com>
 *
 * Based on board-htctopaz-led.c:
 * Copyright (2011) Michael Weirauch <mweirauch@xdandroid.com>
 * 
 * Based on leds-microp-htckovsky.c:
 * Copyright (2010) Alexander Tarasikov <alexander.tarasikov@gmail.com>
 * Some code was written by ultrashot at xda-developers
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/microp.h>
#include <linux/microp-ksc.h>
#include <linux/microp-ng.h>
#include <asm/mach-types.h>

static uint8_t g_auto_backlight = 0;

struct workqueue_struct *whitestone_led_wq;
EXPORT_SYMBOL(whitestone_led_wq);
static struct work_struct lcd_work, auto_bl_work;

static void htcwhitestone_led_brightness_update(struct work_struct* work);

static void htcwhitestone_led_brightness_set(struct led_classdev *led_cdev, enum led_brightness brightness);

static struct i2c_client *client = NULL;
static struct i2c_client *ksc_client=NULL;

enum supported_led {LCD};
static struct led_classdev htcwhitestone_leds[] = {
	[LCD] = {
		.name = "lcd-backlight",
		.brightness = 0x90,
		.brightness_set = htcwhitestone_led_brightness_set,
	},
};
/*Worker functions*/

static void htcwhitestone_led_brightness_update(struct work_struct* work)
{
	uint8_t buffer[2] = {0, 0};
	enum led_brightness brightness = htcwhitestone_leds[LCD].brightness;

	if (!client) {
		return;
	}

	buffer[0] = 0x22; // value from .27 microp-klt.c
	buffer[1] = (brightness*9)/255;

	printk(KERN_DEBUG "%s:Setting LCD brightness to %d/10\n", __func__, buffer[1]);

	microp_ng_write(client, buffer, 2);

	return;
}

static void htcwhitestone_auto_backlight_update(struct work_struct *work)
{
	int ret;
	uint8_t buf[3] = { 0, 0, 0 };

	if (!client) {
		return;
	}

	printk(KERN_DEBUG "%s: %s (%d)\n", __func__, g_auto_backlight ? "on" : "off", g_auto_backlight);

	buf[0] = 0x22;
	buf[1] = g_auto_backlight ? 0xf3 : 0xf8;
	buf[2] = g_auto_backlight ? 0xf3 : 0xf8;

	ret = microp_ng_write(client, buf, ARRAY_SIZE(buf));
	if (ret) {
		printk(KERN_ERR "%s: Failed writing auto backlight status (%d)\n",
			__func__, ret);
	}
	return;
}
/*brightness_set callback functions for all htcwhitestone_leds[]*/
static void htcwhitestone_led_brightness_set(struct led_classdev *led_cdev, enum led_brightness brightness)
{

	if ( !strcmp(led_cdev->name, "klt::lcd-bkl") || !strcmp(led_cdev->name, "lcd-backlight"))
		queue_work(whitestone_led_wq, &lcd_work);
	return;

}

/*
 * dev_attr_auto_backlight for lcd-backlight led device
 */

static ssize_t htcwhitestone_auto_backlight_get(struct device *dev,
	struct device_attribute *attr, char *ret_buf)
{
	return sprintf(ret_buf,"%d\n",g_auto_backlight);
}

static ssize_t htcwhitestone_auto_backlight_set(struct device *dev,
	struct device_attribute *attr, const char *in_buf, size_t count)
{
	unsigned long val = simple_strtoul(in_buf, NULL, 10);
	g_auto_backlight = val ? 1 : 0;

	queue_work(whitestone_led_wq, &auto_bl_work);

	return count;
}

static DEVICE_ATTR(auto_backlight, 0644, htcwhitestone_auto_backlight_get,
	htcwhitestone_auto_backlight_set);

static int htcwhitestone_microp_probe(struct platform_device *pdev)
{
	int ret = 0;
	int i;

	printk(KERN_INFO "%s\n", __func__);
	client = dev_get_drvdata(&pdev->dev);
	ksc_client=i2c_new_dummy(client->adapter, 0x67);
//	if((!ksc_client) || (!client)) {
//		printk("%s failed to obtain client or ksc_client\n",__func__);
//	}

	for (i = 0; i < ARRAY_SIZE(htcwhitestone_leds); i++) {
		ret = led_classdev_register(&pdev->dev, &htcwhitestone_leds[i]);
		if (ret < 0) {
			printk(KERN_ERR "%s: Failed registering led %s", __func__,
				htcwhitestone_leds[i].name);
			goto led_fail;
		}
	}

	ret = device_create_file(htcwhitestone_leds[LCD].dev, &dev_attr_auto_backlight);
	if (ret < 0) {
		printk(KERN_ERR "%s: Failed registering auto_backlight attribute (%d)",
			__func__, ret);
		goto led_fail;
	}

	INIT_WORK(&lcd_work, htcwhitestone_led_brightness_update);
	INIT_WORK(&auto_bl_work, htcwhitestone_auto_backlight_update);

	// some defaults at boot
	queue_work(whitestone_led_wq, &lcd_work);

	return 0;

led_fail:
	for (i--; i >= 0; i--) {
		led_classdev_unregister(&htcwhitestone_leds[i]);
	}
	return ret;
}

static int htcwhitestone_microp_remove(struct platform_device *pdev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(htcwhitestone_leds); i++) {
		led_classdev_unregister(&htcwhitestone_leds[i]);
	}
	client = NULL;
	destroy_workqueue(whitestone_led_wq);
	return 0;
}

#if CONFIG_PM
static int htcwhitestone_microp_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	flush_workqueue(whitestone_led_wq);

	return 0;
}

static int htcwhitestone_microp_resume(struct platform_device *pdev)
{

	return 0;
}
#else
#define htcwhitestone_microp_suspend NULL
#define htcwhitestone_microp_resume NULL
#endif

static struct platform_driver htcwhitestone_microp_driver = {
	.probe		= htcwhitestone_microp_probe,
	.remove		= htcwhitestone_microp_remove,
	.suspend	= htcwhitestone_microp_suspend,
	.resume		= htcwhitestone_microp_resume,
	.driver		= {
		.name		= "htcwhitestone-microp-leds",
		.owner		= THIS_MODULE,
	},
};

static int __init htcwhitestone_microp_init(void)
{
	whitestone_led_wq = create_singlethread_workqueue("led_wq");
	if (whitestone_led_wq == 0)
		return -ENOMEM;
	return platform_driver_register(&htcwhitestone_microp_driver);
}

static void __exit htcwhitestone_microp_exit(void)
{
	platform_driver_unregister(&htcwhitestone_microp_driver);
}

module_init(htcwhitestone_microp_init);
module_exit(htcwhitestone_microp_exit)
