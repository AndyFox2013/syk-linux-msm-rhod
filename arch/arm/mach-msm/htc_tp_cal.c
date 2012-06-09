/* arch/arm/mach-msm/htc_tp_cal.c
 *
 * Code to extract TP calibration information from ATAG set up 
 * by the bootloader.
 *
 * Copyright (C) 2009 HTC Corporation
 * Author: Zion Huang <Zion_Huang@htc.com>
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/platform_device.h>

#include <asm/setup.h>

/* configuration tags specific to TP */
#define ATAG_TP	0x41387898 /* TP */ //0x89768976//akm //0x57494649//Wifi
#define MAX_CALI_SIZE	0x28U
#define ATAG_TP_DEBUG 0

static unsigned char tp_cal_ram[MAX_CALI_SIZE];
int tp_cap_size;

unsigned char *get_tp_cal_ram(void)
{
	return(tp_cal_ram);
}
EXPORT_SYMBOL(get_tp_cal_ram);

static int __init parse_tag_tp(const struct tag *tag)
{
	unsigned char *dptr = (unsigned char *)(&tag->u);
	unsigned size;

	tp_cap_size = 0;
	size = min((tag->hdr.size - 2) * sizeof(__u32), MAX_CALI_SIZE);
	//size = min((tag->hdr.size - 2) * sizeof(__u16), MAX_CALI_SIZE);
	tp_cap_size = size;
	printk(KERN_INFO "TP Data size = %d , 0x%x, size = %d\n",
			tag->hdr.size, tag->hdr.tag, size);

#if ATAG_TP_DEBUG
	unsigned i;
	unsigned char *ptr;

	ptr = dptr;
	printk(KERN_INFO
	       "TP Data size = %d , 0x%x\n",
	       tag->hdr.size, tag->hdr.tag);
	for (i = 0; i < size; i++)
		printk(KERN_INFO "%02x ", *ptr++);
#endif
	memcpy((void *)tp_cal_ram, (void *)dptr, size);
	return 0;
}

__tagtable(ATAG_TP, parse_tag_tp);

static ssize_t tp_calibration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
    unsigned char *ptr;
	int i, off;
	int points[10];
	if (tp_cap_size != MAX_CALI_SIZE) return 0;
	ptr = get_tp_cal_ram();
	for (i = 0; i < 10; i++) {
		off = i * 4;
		points[i] = ptr[off] | (ptr[off+1] << 8) | (ptr[off+2] << 16) | (ptr[off+3] << 24);
#if ATAG_TP_DEBUG
		printk(KERN_INFO "%d %d %d %d = %d\n",ptr[off], ptr[off+1], ptr[off+2], ptr[off+3], points[i]);
#endif 
	}
	ret = sprintf(buf,"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
		points[2], points[3],
		points[8], points[9],
		points[0], points[1],
		points[4], points[5],
		points[6], points[7]);
	return ret;
}

static DEVICE_ATTR(tp_cal, 0444, tp_calibration_show, NULL);

static struct kobject *and_tp_cal;

static int and_get_tp_cal(void)
{
	int ret ;

	/* Create /sys/android_tp_cal/tp_cal */
	and_tp_cal = kobject_create_and_add("android_tp_cal", NULL);
	if (and_tp_cal == NULL) {
		pr_info("and_get_tp_cal: subsystem_register failed\n");
		ret = -ENOMEM;
		return ret ;
	}

	ret = sysfs_create_file(and_tp_cal, &dev_attr_tp_cal.attr);
	if (ret) {
		pr_info("and_get_tp_cal: sysfs_create_file failed\n");
		kobject_del(and_tp_cal);
	}
	return 0 ;
}

late_initcall(and_get_tp_cal);
