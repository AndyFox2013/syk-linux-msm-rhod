/* linux/arch/arm/mach-msm/devices.c
 *
 * Copyright (C) 2008 Google, Inc.
 * Copyright (C) 2007-2009 HTC Corporation.
 * Author: Thomas Tsai <thomas_tsai@htc.com>
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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

#include <linux/dma-mapping.h>
#include <mach/msm_iomap.h>
#include <mach/dma.h>
#include "gpio_chip.h"
#include "devices.h"
#include <mach/board.h>
#include <mach/board_htc.h>
#include <mach/msm_hsusb.h>
#include <asm/mach-types.h>

#include <asm/mach/flash.h>
#include <asm/setup.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/delay.h>
#include <linux/android_pmem.h>
#include <mach/msm_iomap.h>
#include <mach/mmc.h>

//#include "htc_hw.h"
#include "AudioPara.c"
#include <linux/msm_audio.h>
#include <linux/microp-klt.h>
#include <asm/gpio.h>
#include <mach/msm_smd.h>
#include <mach/msm_rpcrouter.h>
#include <mach/htc_acoustic_wince.h>

#if 1
 #define DHTC(fmt, arg...) printk(KERN_DEBUG "[HTC] %s: " fmt "\n", __FUNCTION__, ## arg)
#else
 #define DHTC(fmt, arg...) do {} while (0)
#endif

int call_vol=5;
module_param(call_vol, int, S_IRUGO | S_IWUSR | S_IWGRP);

static struct android_pmem_platform_data pmem_pdata = {
	.name = "pmem",
	.no_allocator = 1,
	.cached = 1,
};

static struct android_pmem_platform_data pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.no_allocator = 0,
	.cached = 0,
};

static struct android_pmem_platform_data pmem_camera_pdata = {
	.name = "pmem_camera",
	.no_allocator = 1,
	.cached = 0,
};

static struct android_pmem_platform_data pmem_gpu0_pdata = {
	.name = "pmem_gpu0",
	.no_allocator = 1,
	.cached = 0,
	.buffered = 1,
};

static struct android_pmem_platform_data pmem_gpu1_pdata = {
	.name = "pmem_gpu1",
	.no_allocator = 1,
	.cached = 0,
	.buffered = 1,
};

static struct platform_device pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &pmem_pdata },
};

static struct platform_device pmem_adsp_device = {
	.name = "android_pmem",
	.id = 1,
	.dev = { .platform_data = &pmem_adsp_pdata },
};

static struct platform_device pmem_gpu0_device = {
	.name = "android_pmem",
	.id = 2,
	.dev = { .platform_data = &pmem_gpu0_pdata },
};

static struct platform_device pmem_gpu1_device = {
	.name = "android_pmem",
	.id = 3,
	.dev = { .platform_data = &pmem_gpu1_pdata },
};


static struct platform_device pmem_camera_device = {
	.name = "android_pmem",
	.id = 4,
	.dev = { .platform_data = &pmem_camera_pdata },
};

/*
static struct resource ram_console_resource[] = {
	{
		.flags	= IORESOURCE_MEM,
	}
};

static struct platform_device ram_console_device = {
	.name = "ram_console",
	.id = -1,
	.num_resources  = ARRAY_SIZE(ram_console_resource),
	.resource       = ram_console_resource,
};*/


static struct resource resources_hw3d[] = {
	{
		.start	= 0xA0000000,
		.end	= 0xA00fffff,
		.flags	= IORESOURCE_MEM,
		.name	= "regs",
	},
	{
		.flags	= IORESOURCE_MEM,
		.name	= "smi",
	},
	{
		.flags	= IORESOURCE_MEM,
		.name	= "ebi",
	},
	{
		.start	= INT_GRAPHICS,
		.end	= INT_GRAPHICS,
		.flags	= IORESOURCE_IRQ,
		.name	= "gfx",
	},
};

static struct platform_device hw3d_device = {
	.name		= "msm_hw3d",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(resources_hw3d),
	.resource	= resources_hw3d,
};

void __init msm_add_mem_devices(struct msm_pmem_setting *setting)
{
	struct resource *res;

	if (setting->pmem_size) {
		pmem_pdata.start = setting->pmem_start;
		pmem_pdata.size = setting->pmem_size;
		platform_device_register(&pmem_device);
	}

	if (setting->pmem_adsp_size) {
		pmem_adsp_pdata.start = setting->pmem_adsp_start;
		pmem_adsp_pdata.size = setting->pmem_adsp_size;
		platform_device_register(&pmem_adsp_device);
	}

	if (setting->pmem_gpu0_size && setting->pmem_gpu1_size) {
		pmem_gpu0_pdata.start = setting->pmem_gpu0_start;
		pmem_gpu0_pdata.size = setting->pmem_gpu0_size;
		platform_device_register(&pmem_gpu0_device);

		pmem_gpu1_pdata.start = setting->pmem_gpu1_start;
		pmem_gpu1_pdata.size = setting->pmem_gpu1_size;
		platform_device_register(&pmem_gpu1_device);

		res = platform_get_resource_byname(&hw3d_device, IORESOURCE_MEM,
						   "smi");
		res->start = setting->pmem_gpu0_start;
		res->end = res->start + setting->pmem_gpu0_size - 1;

		res = platform_get_resource_byname(&hw3d_device, IORESOURCE_MEM,
						   "ebi");
		res->start = setting->pmem_gpu1_start;
		res->end = res->start + setting->pmem_gpu1_size - 1;
		platform_device_register(&hw3d_device);
	}

	if (setting->pmem_camera_size) {
		pmem_camera_pdata.start = setting->pmem_camera_start;
		pmem_camera_pdata.size = setting->pmem_camera_size;
		platform_device_register(&pmem_camera_device);
	}


/*	if (setting->ram_console_size) {
		ram_console_resource[0].start = setting->ram_console_start;
		ram_console_resource[0].end = setting->ram_console_start
			+ setting->ram_console_size - 1;
		platform_device_register(&ram_console_device);
	}*/
}

/******************************************************************************
 * Acoustic driver settings
 ******************************************************************************/
static struct msm_rpc_endpoint *mic_endpoint = NULL;

static void amss_5225_mic_bias_callback(bool on) {
	  struct {
			  struct rpc_request_hdr hdr;
			  uint32_t data;
	  } req;

	  if (!mic_endpoint)
			  mic_endpoint = msm_rpc_connect(0x30000061, 0x0, 0);
	  if (!mic_endpoint) {
			  printk(KERN_ERR "%s: couldn't open rpc endpoint\n", __func__);
			  return;
	  }
	  req.data=cpu_to_be32(on);
	  msm_rpc_call(mic_endpoint, 0x1c, &req, sizeof(req), 5 * HZ);
}
#if CONFIG_MSM_AMSS_VERSION == 5225
static struct htc_acoustic_wce_amss_data amss_5225_acoustic_data = {
	.volume_table = (MSM_SHARED_RAM_BASE+0xfc300),
	.ce_table = (MSM_SHARED_RAM_BASE+0xfc600),
	.adie_table = (MSM_SHARED_RAM_BASE+0xfd000),
	.codec_table = (MSM_SHARED_RAM_BASE+0xfdc00),
	.mic_offset = (MSM_SHARED_RAM_BASE+0xfed00),
	.voc_cal_field_size = 10,
	.mic_bias_callback = amss_5225_mic_bias_callback,
};
#endif
struct htc_acoustic_wce_amss_data amss_6120_acoustic_data = {
	.volume_table = (MSM_SHARED_RAM_BASE+0xfc300),
	.ce_table = (MSM_SHARED_RAM_BASE+0xfc600),
	.adie_table = (MSM_SHARED_RAM_BASE+0xf8000),
	.codec_table = (MSM_SHARED_RAM_BASE+0xf9000),
	.mic_offset = (MSM_SHARED_RAM_BASE+0xfb9c0),
	.voc_cal_field_size = 11,
	.mic_bias_callback = amss_5225_mic_bias_callback,
};

struct platform_device acoustic_device = {
	.name = "htc_acoustic_wince",
	.id = -1,
	.dev = {
		.platform_data = &amss_6120_acoustic_data,
		},
};

//htc_hw.c
static int force_cdma=0;
module_param(force_cdma, int, S_IRUGO | S_IWUSR | S_IWGRP);

static int handsfree=1;
module_param(handsfree, int, S_IRUGO | S_IWUSR | S_IWGRP);

static ssize_t machine_variant_show(struct class *class, struct class_attribute *attr,
			char *buf)

{
	char machine_variant[8];
	int i;
	if(!machine_is_htcrhodium())
		return 0;
	
	/* RHOD model stored at MSM_SPL_BASE + 0x81068 as wchar */
	/* Expected format 'RHODn00' */
	for(i=0; i < 7; i++)
	{
		machine_variant[i] = (char)*(unsigned short*)(MSM_SPL_BASE + 0x81068 + i*2);
	}
	machine_variant[7] = 0;


	return sprintf(buf, "%s\n", machine_variant);

}

int get_machine_variant_type(void)
{
	int machine_variant_type = MACHINE_VARIANT_UNDEFINED;
	char machine_variant[10];

	if(machine_is_htcrhodium() &&
	   (machine_variant_show(NULL, NULL, machine_variant) >= 5))
	{
		switch (machine_variant[4]) {
			case '1':
				machine_variant_type = MACHINE_VARIANT_RHOD_1XX;
				break;
			case '2':
				machine_variant_type = MACHINE_VARIANT_RHOD_2XX;
				break;
			case '3':
				machine_variant_type = MACHINE_VARIANT_RHOD_3XX;
				break;
			case '4':
				machine_variant_type = MACHINE_VARIANT_RHOD_4XX;
				break;
			case '5':
				machine_variant_type = MACHINE_VARIANT_RHOD_5XX;
				break;
		}
	} 

	return machine_variant_type;
}
EXPORT_SYMBOL(get_machine_variant_type);

static ssize_t test_store(struct class *class, struct class_attribute *attr,
			const char *buf, size_t count)
{
	int v;
	sscanf(buf, "%d", &v);
	return 1;
}

static ssize_t flash_store(struct class *class, struct class_attribute *attr,
			const char *buf, size_t count)
{
	int v;
	sscanf(buf, "%d", &v);
	gpio_set_value(0x3a, !!v);
	return 1;
}

static ssize_t radio_show(struct class *class, struct class_attribute *attr,
			char *buf)
{
	char *radio_type = ((machine_is_htcraphael_cdma() || machine_is_htcraphael_cdma500()) || 
	                    machine_is_htcdiamond_cdma() || force_cdma) ? "CDMA" : "GSM";
	return sprintf(buf, "%s\n", radio_type);
}

static ssize_t machtype_show(struct class *class, struct class_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%d\n", machine_arch_type);
}

extern unsigned int __amss_version; // amss_para.c
static ssize_t amss_show(struct class *class, struct class_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%d\n", __amss_version);
}

static struct class_attribute htc_hw_class_attrs[] = {
	__ATTR_RO(radio),
	__ATTR_RO(machtype),
	__ATTR_RO(amss),
	__ATTR_RO(machine_variant),
	__ATTR(flash, 0222, NULL, flash_store),
	__ATTR(test,0222, NULL, test_store),
	__ATTR_NULL,
};

static ssize_t gsmphone_show(struct class *class, struct class_attribute *attr,
			char *buf) {
	return sprintf(buf, "%d\n", __machine_arch_type);
}
 
// these are for compatability with the vogue ril
static struct class_attribute vogue_hw_class_attrs[] = {
	__ATTR_RO(gsmphone),
	__ATTR_NULL, 
};

static struct class vogue_hw_class = {
	.name = "vogue_hw",
	.class_attrs = vogue_hw_class_attrs,
};

static struct class htc_hw_class = {
	.name = "htc_hw",
	.class_attrs = htc_hw_class_attrs,
};

static int htc_hw_probe(struct platform_device *pdev)
{
	int ret=0;
//	htc_hw_pdata = (htc_hw_pdata_t *)pdev->dev.platform_data;
	ret = class_register(&htc_hw_class);
	ret = class_register(&vogue_hw_class);
	if (ret)
		printk(KERN_ERR "%s: class init failed: %d\n", __func__, ret);
	DHTC("done");
	return ret;
}

static struct platform_driver htc_hw_driver = {
	.probe = htc_hw_probe,
	.driver = {
		.name = "htc_hw",
		.owner = THIS_MODULE,
	},
};

static int __init htc_hw_init(void)
{
	DHTC("Initializing HTC hardware platform driver");
	return platform_driver_register(&htc_hw_driver);
}

module_init(htc_hw_init);

