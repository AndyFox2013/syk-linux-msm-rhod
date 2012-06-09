 /* linux/arch/arm/mach-msm/board-htcrhodium-led.c
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

#define RHOD_MICROP_KSC_LED_STATE		0x30
#define RHOD_MICROP_KSC_LED_BRIGHTNESS	0x32
#define RHOD_MICROP_KSC_LED_CAPS (1<<0)
#define RHOD_MICROP_KSC_LED_FN   (1<<1)

static uint8_t g_auto_backlight = 0;
static unsigned int g_blink_status=0;
static uint8_t last_keyboard_brightness = 0;

/*led_regime is a module parameter used to control the led_pattern: 2->kernel enforced amber awake/green sleep; 1->kernel enforced amber awake/OFF sleep; 0 (default)->no kernel enforcement/engage userland notifications*/

static unsigned int led_regime=0;
module_param_named(led_regime, led_regime, int, S_IRUGO | S_IWUSR | S_IWGRP);

struct workqueue_struct *led_wq;
EXPORT_SYMBOL(led_wq);
static struct work_struct color_led_work, blink_work, lcd_work, keypad_work, keyboard_work, meta_key_work, auto_bl_work;

static void htcrhodium_color_led_update(struct work_struct* work);
static void htcrhodium_color_led_blink_update(struct work_struct* work);
static void htcrhodium_led_brightness_update(struct work_struct* work);
static void htcrhodium_keypad_brightness_update(struct work_struct* work);
static void htcrhodium_keyboard_brightness_update(struct work_struct* work);
static void htcrhodium_meta_key_brightness_update(struct work_struct* work);


static void htcrhodium_color_led_set(struct led_classdev *led_cdev, enum led_brightness brightness);
static int htcrhodium_color_led_blink_set(struct led_classdev *led_cdev, unsigned long *delay_on, unsigned long *delay_off);
static void htcrhodium_led_brightness_set(struct led_classdev *led_cdev, enum led_brightness brightness);
static void htcrhodium_keypad_brightness_set(struct led_classdev *led_cdev, enum led_brightness brightness);
static void htcrhodium_keyboard_brightness_set(struct led_classdev *led_cdev, enum led_brightness brightness);
static void htcrhodium_meta_key_brightness_set(struct led_classdev *led_cdev, enum led_brightness brightness);

static struct i2c_client *client = NULL;
static struct i2c_client *ksc_client=NULL;

enum led_color {
	COLOR_OFF = 0,
	COLOR_GREEN = 1,
	COLOR_AMBER = 2,
	BLINK_GREEN = 4,
	BLINK_AMBER = 5,
};

static int microp_set_color_led_state(enum led_color led_color_value);
static char* led_color_name(enum led_color color) {
	switch(color) {
		case COLOR_OFF:
			return "OFF";
		case COLOR_GREEN:
			return "GREEN";
		case COLOR_AMBER:
			return "AMBER";
		case BLINK_GREEN:
			return "BLINK GREEN";
		case BLINK_AMBER:
			return "BLINK AMBER";
		default:
			return "UNKNOWN";
	}
}

enum supported_led {AMBER, GREEN, LCD, BUTTONS, KEYBOARD, CAPS, FUNC};

static struct led_classdev htcrhodium_leds[] = {
	[AMBER] = {
		.name = "amber",
		.brightness = 0,
		.brightness_set = htcrhodium_color_led_set,
		.blink_set = htcrhodium_color_led_blink_set,
	},
	[GREEN] = {
		.name = "green",
		.brightness = 0,
		.brightness_set = htcrhodium_color_led_set,
		.blink_set = htcrhodium_color_led_blink_set,
	},
	[LCD] = {
		.name = "lcd-backlight",
		.brightness = 0x90,
		.brightness_set = htcrhodium_led_brightness_set,
	},
	[BUTTONS] = {
		.name = "button-backlight",
		.brightness = LED_OFF,
		.brightness_set = htcrhodium_keypad_brightness_set,
	},
	[KEYBOARD] = {
		.name = "keyboard-backlight",
		.brightness = LED_OFF,
		.brightness_set = htcrhodium_keyboard_brightness_set,
		.default_trigger = "microp-keypad",
	},
	[CAPS] = {
		.name = "caps",
		.brightness = LED_OFF,
		.brightness_set = htcrhodium_meta_key_brightness_set,
	},
	[FUNC] = {
		.name = "func",
		.brightness = LED_OFF,
		.brightness_set = htcrhodium_meta_key_brightness_set,
	},
};
/*Worker functions*/
static void htcrhodium_color_led_update(struct work_struct* work)
{
	if (htcrhodium_leds[AMBER].brightness) {
		microp_set_color_led_state(COLOR_AMBER);
	} else if (htcrhodium_leds[GREEN].brightness) {
		microp_set_color_led_state(COLOR_GREEN);
	} else {
		microp_set_color_led_state(COLOR_OFF);
	}
	return;
}

static void htcrhodium_color_led_blink_update(struct work_struct* work)
{
	if (htcrhodium_leds[AMBER].brightness)
	{
		microp_set_color_led_state(BLINK_AMBER);
		g_blink_status=1;
	}
	else if (htcrhodium_leds[GREEN].brightness)
	{
		microp_set_color_led_state(BLINK_GREEN);
		g_blink_status=1;
	}

	return;
}

static void htcrhodium_led_brightness_update(struct work_struct* work)
{
	uint8_t buffer[2] = {0, 0};
	enum led_brightness brightness = htcrhodium_leds[LCD].brightness;

	if (!client) {
		return;
	}

	buffer[0] = 0x24;
	//Scale from 0 to 9
	buffer[1] = (brightness*9)/255;

	microp_ng_write(client, buffer, 2);

	return;
}

static void htcrhodium_keypad_brightness_update(struct work_struct* work)
{
	enum led_brightness brightness = htcrhodium_leds[BUTTONS].brightness;
	uint8_t buffer[4];

	if (!ksc_client) {
		return;
	}

	if ( brightness ) {
		brightness = 1;
	} else {
		brightness = 0;
	}

	printk("%s: %d\n", __func__, brightness);

	buffer[0] = 0x14;
	buffer[1] = 0x80;
	buffer[2] = brightness;
	microp_ng_write(ksc_client, buffer, 3);
	return;
}

static void htcrhodium_keyboard_brightness_update(struct work_struct* work)
{
	enum led_brightness brightness = htcrhodium_leds[KEYBOARD].brightness;

	uint8_t buffer[4] = {};

	if (!ksc_client) {
		return;
	}

	printk(KERN_DEBUG "%s: brightness=%d\n", __func__, brightness);

	buffer[0] = RHOD_MICROP_KSC_LED_BRIGHTNESS;
	buffer[1] = last_keyboard_brightness;    /* initial brightness */
	buffer[2] = 255;                /* transition duration */
	buffer[3] = brightness;         /* target brightness */
	microp_ng_write(ksc_client, buffer, 4);

	buffer[0] = RHOD_MICROP_KSC_LED_STATE;
	buffer[1] = 0;
	buffer[2] = !!brightness;
	microp_ng_write(ksc_client, buffer, 3);

	last_keyboard_brightness = brightness;
	return;
}

static void htcrhodium_meta_key_brightness_update(struct work_struct *work)
{
	uint8_t buffer[2] = {0, 0};
	struct {
		unsigned led_state:2;
	} state = {0};
	unsigned int bit;

	if (!client) {
		return;
	}

	bit = RHOD_MICROP_KSC_LED_CAPS;
	if (htcrhodium_leds[CAPS].brightness)
		state.led_state |= bit;
	else
		state.led_state &=~bit;

	bit = RHOD_MICROP_KSC_LED_FN;
	if (htcrhodium_leds[FUNC].brightness)
		state.led_state |= bit;
	else
		state.led_state &=~bit;

	buffer[0] = 0x25;
	buffer[1] = state.led_state;

	microp_ng_write(client, buffer, 2);
	return;
}

static void htcrhodium_auto_backlight_update(struct work_struct *work)
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
/*brightness_set callback functions for all htcrhodium_leds[]*/
static void htcrhodium_color_led_set(struct led_classdev *led_cdev, enum led_brightness brightness)
{
	if (( !strcmp(led_cdev->name, "amber") || !strcmp(led_cdev->name, "green") ) && !led_regime)
		queue_work(led_wq, &color_led_work);

	return;
}

static int htcrhodium_color_led_blink_set(struct led_classdev *led_cdev, unsigned long *delay_on, unsigned long *delay_off)
{
	if (( !strcmp(led_cdev->name, "amber") || !strcmp(led_cdev->name, "green") ) && !led_regime)
		return queue_work(led_wq, &blink_work);

	return 0;
}

static void htcrhodium_led_brightness_set(struct led_classdev *led_cdev, enum led_brightness brightness)
{

	if ( !strcmp(led_cdev->name, "klt::lcd-bkl") || !strcmp(led_cdev->name, "lcd-backlight"))
		queue_work(led_wq, &lcd_work);
	return;

}

static void htcrhodium_keypad_brightness_set(struct led_classdev *led_cdev, enum led_brightness brightness)
{

	if ( !strcmp(led_cdev->name, "button-backlight"))
		queue_work(led_wq, &keypad_work);
	return;

}

static void htcrhodium_keyboard_brightness_set(struct led_classdev *led_cdev, enum led_brightness brightness)
{
	if (!strcmp(led_cdev->name, "keyboard-backlight"))
		queue_work(led_wq, &keyboard_work);
}

static void htcrhodium_meta_key_brightness_set(struct led_classdev *led_cdev, enum led_brightness brightness)
{
	if ( !strcmp(led_cdev->name, "caps") || !strcmp(led_cdev->name, "func"))
		queue_work(led_wq, &meta_key_work);

}

/*Blink attributes for color led: callbacks*/
static ssize_t microp_color_led_blink_set(struct device *dev, struct device_attribute *attr, const char *in_buf, size_t count)
{
	unsigned long val = simple_strtoul(in_buf, NULL, 10);
	struct led_classdev *led_cdev = dev_get_drvdata(dev);

	if (val)
	{
		if (led_cdev->brightness && !led_regime)
			led_cdev->blink_set(led_cdev, 0, 0);
	}
	else
		g_blink_status=0;
	return count;

}

static ssize_t microp_color_led_blink_get(struct device *dev, struct device_attribute *attr, char *ret_buf)
{
	return sprintf(ret_buf,"%d\n",g_blink_status);
}

static DEVICE_ATTR(blink, 0666, microp_color_led_blink_get, microp_color_led_blink_set);

/*
 * dev_attr_auto_backlight for lcd-backlight led device
 */

static ssize_t htcrhodium_auto_backlight_get(struct device *dev,
	struct device_attribute *attr, char *ret_buf)
{
	return sprintf(ret_buf,"%d\n",g_auto_backlight);
}

static ssize_t htcrhodium_auto_backlight_set(struct device *dev,
	struct device_attribute *attr, const char *in_buf, size_t count)
{
	unsigned long val = simple_strtoul(in_buf, NULL, 10);
	g_auto_backlight = val ? 1 : 0;

	queue_work(led_wq, &auto_bl_work);

	return count;
}

static DEVICE_ATTR(auto_backlight, 0644, htcrhodium_auto_backlight_get,
	htcrhodium_auto_backlight_set);
/*
 * MicropP functions
 */

static int microp_set_color_led_state(enum led_color led_color_value)
{
	int ret;
	uint8_t buf[5] = { 0, 0, 0, 0, 0 };

	if (!client) {
		return -EAGAIN;
	}

	printk(KERN_DEBUG "%s: color=%s\n", __func__,
		led_color_name(led_color_value));

	if (machine_is_htctopaz()) {
		buf[0] = 0x51;
	} else {
		buf[0] = 0x50; // RHOD
	}
	buf[1] = 0;
	buf[2] = led_color_value;
	buf[3] = 0xff;
	buf[4] = 0xff;

	ret = microp_ng_write(client, buf, 5);
	if (ret) {
		printk(KERN_ERR "%s: Failed setting color led value (%d)\n",
			__func__, ret);
	}
	return ret;
}
/* Following functions need to be used with care, as they sit outside of the workqueue*/
/* Should be used in conjuction with led_wq (also exported) to make sure no race conditions occur*/
/* Needed to provide access to the client (perhaps only ro is needed?)*/
/* Currently, read_wrapper used in microp_keypad */
int microp_ng_write_wrapper(char* name, uint8_t* sendbuf, int len)
{
	if (!(strcmp(name, "ksc")))
		return microp_ng_write(ksc_client, sendbuf, len);
	else if (!(strcmp(name, "klt")))
		return microp_ng_write(client, sendbuf, len);
	return 0;
}
EXPORT_SYMBOL(microp_ng_write_wrapper);
int microp_ng_read_wrapper(char *name, uint8_t id, uint8_t *buf, int len)
{
	if (!(strcmp(name, "ksc")))
		return microp_ng_read(ksc_client, id, buf, len);
	else if (!(strcmp(name, "klt")))
		return microp_ng_read(client, id, buf, len);
	return 0;
}
EXPORT_SYMBOL(microp_ng_read_wrapper);
static int htcrhodium_microp_probe(struct platform_device *pdev)
{
	int ret = 0;
	int i;

	printk(KERN_INFO "%s\n", __func__);
	client = dev_get_drvdata(&pdev->dev);
	ksc_client=i2c_new_dummy(client->adapter, 0x67);
	if((!ksc_client) || (!client)) {
		printk("%s failed to obtain client or ksc_client\n",__func__);
	}

	for (i = 0; i < ARRAY_SIZE(htcrhodium_leds); i++) {
		ret = led_classdev_register(&pdev->dev, &htcrhodium_leds[i]);
		if (ret < 0) {
			printk(KERN_ERR "%s: Failed registering led %s", __func__,
				htcrhodium_leds[i].name);
			goto led_fail;
		}
		if (htcrhodium_leds[i].blink_set) {
			ret = device_create_file(htcrhodium_leds[i].dev, &dev_attr_blink);
			if (ret < 0) {
				printk(KERN_ERR "%s: Failed registering blink attribute %s",
					__func__, htcrhodium_leds[i].name);
				goto led_fail;
			}
		}
	}

	ret = device_create_file(htcrhodium_leds[LCD].dev, &dev_attr_auto_backlight);
	if (ret < 0) {
		printk(KERN_ERR "%s: Failed registering auto_backlight attribute (%d)",
			__func__, ret);
		goto led_fail;
	}

	INIT_WORK(&color_led_work, htcrhodium_color_led_update);
	INIT_WORK(&blink_work, htcrhodium_color_led_blink_update);
	INIT_WORK(&lcd_work, htcrhodium_led_brightness_update);
	INIT_WORK(&keypad_work, htcrhodium_keypad_brightness_update);
	INIT_WORK(&keyboard_work, htcrhodium_keyboard_brightness_update);
	INIT_WORK(&meta_key_work, htcrhodium_meta_key_brightness_update);
	INIT_WORK(&auto_bl_work, htcrhodium_auto_backlight_update);

	// some defaults at boot
	queue_work(led_wq, &color_led_work);
	queue_work(led_wq, &lcd_work);
	queue_work(led_wq, &keypad_work);

	return 0;

led_fail:
	for (i--; i >= 0; i--) {
		led_classdev_unregister(&htcrhodium_leds[i]);
		if (htcrhodium_leds[i].blink_set) {
			device_remove_file(htcrhodium_leds[i].dev, &dev_attr_blink);
		}
	}
	return ret;
}

static int htcrhodium_microp_remove(struct platform_device *pdev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(htcrhodium_leds); i++) {
		led_classdev_unregister(&htcrhodium_leds[i]);
	}
	client = NULL;
	destroy_workqueue(led_wq);
	return 0;
}

#if CONFIG_PM
static int htcrhodium_microp_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	// set color led to OFF when sleeping in led_regime mode 1
	if (led_regime == 1) {
		microp_set_color_led_state(COLOR_OFF);
	}
	// set color led to GREEN when sleeping in led_regime mode 2
	if (led_regime == 2) {
		microp_set_color_led_state(COLOR_GREEN);
	}
	htcrhodium_leds[KEYBOARD].brightness_set(&htcrhodium_leds[KEYBOARD],LED_OFF);
	flush_workqueue(led_wq);

	return 0;
}

static int htcrhodium_microp_resume(struct platform_device *pdev)
{

	// set color led to AMBER when awake in led_regime mode 1 or 2
	if (led_regime) {
		microp_set_color_led_state(COLOR_AMBER);
	}

	return 0;
}
#else
#define htcrhodium_microp_suspend NULL
#define htcrhodium_microp_resume NULL
#endif

static struct platform_driver htcrhodium_microp_driver = {
	.probe		= htcrhodium_microp_probe,
	.remove		= htcrhodium_microp_remove,
	.suspend	= htcrhodium_microp_suspend,
	.resume		= htcrhodium_microp_resume,
	.driver		= {
		.name		= "htcrhodium-microp-leds",
		.owner		= THIS_MODULE,
	},
};

static int __init htcrhodium_microp_init(void)
{
	led_wq = create_singlethread_workqueue("led_wq");
	if (led_wq == 0)
		return -ENOMEM;
	return platform_driver_register(&htcrhodium_microp_driver);
}

static void __exit htcrhodium_microp_exit(void)
{
	platform_driver_unregister(&htcrhodium_microp_driver);
}

module_init(htcrhodium_microp_init);
module_exit(htcrhodium_microp_exit)
