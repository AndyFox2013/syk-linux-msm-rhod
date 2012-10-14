/*
 * android_compat.c - backwards interface compatibility for Android USB gadget
 *
 * Copyright (C) 2012 Paul Kocialkowski <contact@paulk.fr>
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
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/switch.h>

#include <linux/types.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/usb/android_composite.h>

/*-------------------------------------------------------------------------*/
/* android_compat data structures */

struct android_switch_compat {
	/* switch indicating connected/disconnected state */
	struct switch_dev		sw_connected;
	/* switch indicating current configuration */
	struct switch_dev		sw_config;
	/* current connected state for sw_connected */
};

struct android_compat_data {
	struct class *composite_class;
	struct android_switch_compat switch_devs;

	struct platform_device *android_usb_pdev;
	struct platform_device *rndis_pdev;
	struct platform_device *ums_pdev;
};

struct android_compat_data *android_compat_data;

/*-------------------------------------------------------------------------*/
/* enable/disable functions */

static int android_compat_enable_function(struct android_dev *dev, char *name)
{
	int enabled = android_check_function_enabled(dev, name);
	int err;

	if(enabled) {
		printk(KERN_INFO "android_compat_enable_function: %s function already enabled!\n", name);
		return 0;
	}

	android_disable(dev);
	err = android_enable_function(dev, name);
	android_enable(dev);

	return err;
}

static int android_compat_disable_function(struct android_dev *dev, char *name)
{
	int enabled = android_check_function_enabled(dev, name);
	int err;

	if(!enabled) {
		printk(KERN_INFO "android_compat_enable_function: %s function already disabled!\n", name);
		return 0;
	}

	android_disable(dev);
	err = android_disable_function(dev, name);
	if (list_empty(&dev->enabled_functions) != 1) {
		android_enable(dev);
	}

	return err;
}

/*-------------------------------------------------------------------------*/
/* android_adb_enable dev node */

static int adb_enable_open(struct inode *ip, struct file *fp)
{
	struct android_dev *dev = _android_dev;

	printk(KERN_INFO "adb_enable_open: enabling adb function\n");
	return android_compat_enable_function(dev, "adb");
}

static int adb_enable_release(struct inode *ip, struct file *fp)
{
	struct android_dev *dev = _android_dev;

	printk(KERN_INFO "adb_enable_release: disabling adb function\n");
	return android_compat_disable_function(dev, "adb");
}

static const struct file_operations adb_enable_fops = {
	.owner =   THIS_MODULE,
	.open =    adb_enable_open,
	.release = adb_enable_release,
};

static struct miscdevice adb_enable_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "android_adb_enable",
	.fops = &adb_enable_fops,
};

/*-------------------------------------------------------------------------*/
/* usb_composite sys nodes */

static int
composite_uevent(struct device *pdev, struct kobj_uevent_env *env)
{
	struct android_dev *dev = _android_dev;
	struct usb_function *f = dev_get_drvdata(pdev);
	int enabled;

	if (!f) {
		/* this happens when the device is first created */
		return 0;
	}

	enabled = android_check_function_enabled(dev, f->name);

	if(strcmp(f->name, "mass_storage") == 0) {
		if (add_uevent_var(env, "FUNCTION=%s", "usb_mass_storage"))
			return -ENOMEM;
	} else {
		if (add_uevent_var(env, "FUNCTION=%s", f->name))
			return -ENOMEM;
	}
	if (add_uevent_var(env, "ENABLED=%d", enabled))
		return -ENOMEM;
	return 0;
}

static ssize_t composite_enable_show(struct device *pdev, struct device_attribute *attr,
			   char *buf)
{
	struct android_dev *dev = _android_dev;
	struct android_usb_function *function = dev_get_drvdata(pdev);

	int enabled = android_check_function_enabled(dev, function->name);

	return sprintf(buf, "%d\n", enabled);
}

static ssize_t composite_enable_store(struct device *pdev, struct device_attribute *attr,
			    const char *buff, size_t size)
{
	struct android_dev *dev = _android_dev;
	struct android_usb_function *function = dev_get_drvdata(pdev);

	int enable;

	sscanf(buff, "%d", &enable);
	if(enable) {
		android_compat_enable_function(dev, function->name);
	} else {
		android_compat_disable_function(dev, function->name);
	}

	return size;
}

static struct device_attribute composite_dev_attr_enable = {
	.attr = {
		.name = "enable",
		.mode = S_IWUSR | S_IRUGO,

	},
	.show = composite_enable_show,
	.store = composite_enable_store,
};

static int composite_functions_init(void)
{
	struct android_dev *dev = _android_dev;
	struct android_usb_function **functions = dev->functions;
	struct android_usb_function *f;

	int index = 0;
	int err;

	for (index = 0; (f = *functions++); index++) {
		if(strcmp(f->name, "mass_storage") == 0) {
			f->compat_dev = device_create(android_compat_data->composite_class, NULL,
					MKDEV(0, index), f, "usb_mass_storage");

		} else {
			f->compat_dev = device_create(android_compat_data->composite_class, NULL,
					MKDEV(0, index), f, f->name);
		}
		if (IS_ERR(f->compat_dev))
			return PTR_ERR(f->compat_dev);

		err = device_create_file(f->compat_dev, &composite_dev_attr_enable);
		if (err < 0) {
			device_destroy(android_compat_data->composite_class, f->compat_dev->devt);
			return err;
		}

		dev_set_drvdata(f->compat_dev, f);
	}

	return 0;
}

static void composite_functions_cleanup(void)
{
	struct android_dev *dev = _android_dev;
	struct android_usb_function **functions = dev->functions;
	struct android_usb_function *f;

	while (*functions) {
		f = *functions++;

		if (f->compat_dev) {
			device_destroy(android_compat_data->composite_class, f->compat_dev->devt);
			device_remove_file(f->compat_dev, &composite_dev_attr_enable);
		}
	}
}

/*-------------------------------------------------------------------------*/
/* switch sys nodes */

static void
android_compat_switch_work(struct work_struct *data)
{
	struct android_dev *dev = container_of(data, struct android_dev, work);
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_configuration *config = cdev->config;
	int connected;
	unsigned long flags;

	spin_lock_irqsave(&cdev->lock, flags);
	if (dev->connected != android_compat_data->switch_devs.sw_connected.state) {
		connected = dev->connected;
		spin_unlock_irqrestore(&cdev->lock, flags);
		switch_set_state(&android_compat_data->switch_devs.sw_connected, connected);
	} else {
		spin_unlock_irqrestore(&cdev->lock, flags);
	}

	if (config)
		switch_set_state(&android_compat_data->switch_devs.sw_config, config->bConfigurationValue);
	else
		switch_set_state(&android_compat_data->switch_devs.sw_config, 0);
}

/*-------------------------------------------------------------------------*/
/* android plat */

static int android_usb_probe(struct platform_device *pdev)
{
	android_compat_data->android_usb_pdev = pdev;

	return 0;
}

static int rndis_probe(struct platform_device *pdev)
{
	android_compat_data->rndis_pdev = pdev;

	return 0;
}

static int ums_probe(struct platform_device *pdev)
{
	android_compat_data->ums_pdev = pdev;

	return 0;
}


static struct platform_driver android_usb_platform_driver = {
	.driver = { .name = "android_usb", },
	.probe = android_usb_probe,
};


static struct platform_driver rndis_platform_driver = {
	.driver = { .name = "rndis", },
	.probe = rndis_probe,
};

static struct platform_driver ums_platform_driver = {
	.driver = { .name = "usb_mass_storage", },
	.probe = ums_probe,
};

/*-------------------------------------------------------------------------*/
/* usb_mass_storage sysfs nodes */

static int android_compat_usb_mass_storage_init(struct android_usb_function *f,
		struct mass_storage_function_config *config,
		struct fsg_common *common)

{
	int lun_string_size = strlen("lun") + 1;
	int n = FSG_MAX_LUNS;

	while(n > 0) {
		lun_string_size++;
		n /= 10;
	}

	char lun[lun_string_size];
	int err;
	int i;

	for(i=0 ; i < config->fsg.nluns ; i++) {
		snprintf(lun, lun_string_size, "lun%d", i);
		err = sysfs_create_link(&android_compat_data->ums_pdev->dev.kobj,
				&common->luns[i].dev.kobj,
				lun);
		if(err)
			return err;
	}

	return 0;
}

/*-------------------------------------------------------------------------*/
/* android compat */

static int
android_compat_rndis_config(struct rndis_function_config *config)
{
	struct usb_ether_platform_data *rndis_pdata = android_compat_data->rndis_pdev->dev.platform_data;

	config->vendorID = rndis_pdata->vendorID;
	if(rndis_pdata->vendorDescr != NULL)
		strncpy(config->manufacturer, rndis_pdata->vendorDescr, sizeof(config->manufacturer) - 1);
	if(rndis_pdata->ethaddr != NULL)
		memcpy(config->ethaddr, rndis_pdata->ethaddr, sizeof(config->ethaddr) - 1);

	return 0;
}

static int
android_compat_ums_config(struct mass_storage_function_config *config)
{
	struct usb_mass_storage_platform_data *ums_pdata = android_compat_data->ums_pdev->dev.platform_data;
	int i;

	config->fsg.nluns = ums_pdata->nluns;
	for(i=0 ; i < config->fsg.nluns ; i++) {
		config->fsg.luns[i].removable = 1;
	}

	config->fsg.vendor_name = ums_pdata->vendor;
	config->fsg.product_name = ums_pdata->product;
	config->fsg.release = ums_pdata->release;

	return 0;
}

static int
android_compat_config(char *manufacturer_string, int manufacturer_string_len,
		char *product_string, int product_string_len,
		char *serial_string, int serial_string_len)
{
	struct android_usb_platform_data *android_usb_pdata = android_compat_data->android_usb_pdev->dev.platform_data;

	if(android_usb_pdata->manufacturer_name != NULL)
		strncpy(manufacturer_string, android_usb_pdata->manufacturer_name, manufacturer_string_len);
	if(android_usb_pdata->product_name != NULL)
		strncpy(product_string, android_usb_pdata->product_name, product_string_len);
	if(android_usb_pdata->serial_number != NULL)
		strncpy(serial_string, android_usb_pdata->serial_number, serial_string_len);

	return 0;
}

static int android_compat_init(void)
{
	int ret;

	android_compat_data = kzalloc(sizeof(*android_compat_data), GFP_KERNEL);

	ret = misc_register(&adb_enable_device);
	if (ret)
		goto error;

	android_compat_data->composite_class = class_create(THIS_MODULE, "usb_composite");
	if (IS_ERR(android_compat_data->composite_class))
		return PTR_ERR(android_compat_data->composite_class);
	android_compat_data->composite_class->dev_uevent = composite_uevent;

	composite_functions_init();

	android_compat_data->switch_devs.sw_connected.name = "usb_connected";
	ret = switch_dev_register(&android_compat_data->switch_devs.sw_connected);
	if (ret < 0)
		goto error;

	android_compat_data->switch_devs.sw_config.name = "usb_configuration";
	ret = switch_dev_register(&android_compat_data->switch_devs.sw_config);
	if (ret < 0)
		goto error;

	ret = platform_driver_register(&android_usb_platform_driver);
	if (ret < 0)
		goto error;

	ret = platform_driver_register(&rndis_platform_driver);
	if (ret < 0)
		goto error;

	ret = platform_driver_register(&ums_platform_driver);
	if (ret < 0)
		goto error;

	return 0;

error:
	printk(KERN_ERR "android_compat_init: error\n");
	return ret;
}

static void android_compat_cleanup(void)
{
	composite_functions_cleanup();
	class_destroy(android_compat_data->composite_class);

	switch_dev_unregister(&android_compat_data->switch_devs.sw_connected);
	switch_dev_unregister(&android_compat_data->switch_devs.sw_config);

	misc_deregister(&adb_enable_device);

	kfree(android_compat_data);
}

