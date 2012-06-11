/* linux/arch/arm/mach-msm/board-trout-mmc.c
** Author: Brian Swetland <swetland@google.com>
*/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/wifi_tiwlan.h>
#include <linux/skbuff.h>

#include <asm/mach-types.h>
//#include <asm/mach/mmc.h>
#include <asm/gpio.h>
#include <asm/io.h>

#include <mach/vreg.h>
#include <mach/htc_pwrsink.h>
#include <mach/mmc.h>

#include "devices.h"

#define DEBUG_SDSLOT_VDD 1

extern int msm_add_sdcc(unsigned int controller, struct mmc_platform_data *plat,
			unsigned int stat_irq, unsigned long stat_irq_flags);

/* This struct holds the device-specific numbers and tables */
static struct mmc_dev_data {
	unsigned sdcard_status_gpio;
	char sdcard_device_id:3;
	unsigned wifi_power_gpio1;
	unsigned wifi_power_gpio2;
	unsigned *sdcard_on_gpio_table;
	unsigned *sdcard_off_gpio_table;
	int sdcard_on_gpio_table_size;
	int sdcard_off_gpio_table_size;
	unsigned *wifi_on_gpio_table;
	unsigned *wifi_off_gpio_table;
	int wifi_on_gpio_table_size;
	int wifi_off_gpio_table_size;
} mmc_pdata;

/* ---- COMMON ---- */
static void config_gpio_table(unsigned *table, int len)
{
	int n;
	for (n = 0; n < len; n++) {
		gpio_tlmm_config(table[n], 0);
	}
}


/* ---- SDCARD ---- */

static unsigned sdcard_on_gpio_table_gsm[] = {
/*               num,alt                                                   */
	GPIO_CFG( 62, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* CLK */
	GPIO_CFG( 63, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* CMD */
	GPIO_CFG( 64, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* DAT3 */
	GPIO_CFG( 65, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* DAT2 */
	GPIO_CFG( 66, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* DAT1 */
	GPIO_CFG( 67, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* DAT0 */
};

static unsigned sdcard_off_gpio_table_gsm[] = {
/*                   num,alt                                                   */
	GPIO_CFG( 62, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* CLK */
	GPIO_CFG( 63, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* CMD */
	GPIO_CFG( 64, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* DAT3 */
	GPIO_CFG( 65, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* DAT2 */
	GPIO_CFG( 66, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* DAT1 */
	GPIO_CFG( 67, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* DAT0 */
};

static unsigned sdcard_on_gpio_table_cdma[] = {
	GPIO_CFG( 88, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), /* CLK */
	GPIO_CFG( 89, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), /* CMD */
	GPIO_CFG( 90, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), /* DAT3 */
	GPIO_CFG( 91, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), /* DAT2 */
	GPIO_CFG( 92, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), /* DAT1 */
	GPIO_CFG( 93, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), /* DAT0 */
};

static unsigned sdcard_off_gpio_table_cdma[] = {
	GPIO_CFG( 88, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* CLK */
	GPIO_CFG( 89, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* CMD */
	GPIO_CFG( 90, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* DAT3 */
	GPIO_CFG( 91, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* DAT2 */
	GPIO_CFG( 92, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* DAT1 */
	GPIO_CFG( 93, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* DAT0 */
};

/* ---- WIFI ---- */

static unsigned wifi_on_gpio_table_gsm[] = {
	GPIO_CFG( 51, 1, GPIO_CFG_INPUT , GPIO_CFG_PULL_UP, GPIO_CFG_2MA), /* DAT3 */
	GPIO_CFG( 52, 1, GPIO_CFG_INPUT , GPIO_CFG_PULL_UP, GPIO_CFG_2MA), /* DAT2 */
	GPIO_CFG( 53, 1, GPIO_CFG_INPUT , GPIO_CFG_PULL_UP, GPIO_CFG_2MA), /* DAT1 */
	GPIO_CFG( 54, 1, GPIO_CFG_INPUT , GPIO_CFG_PULL_UP, GPIO_CFG_2MA), /* DAT0 */
	GPIO_CFG( 55, 1, GPIO_CFG_INPUT , GPIO_CFG_PULL_UP, GPIO_CFG_2MA), /* CMD */
	GPIO_CFG( 56, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* CLK */

	GPIO_CFG( 29, 0, GPIO_CFG_INPUT , GPIO_CFG_PULL_UP, GPIO_CFG_2MA), /* WLAN IRQ */
};

static unsigned wifi_off_gpio_table_gsm[] = {
	GPIO_CFG( 51, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* DAT3 */
	GPIO_CFG( 52, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* DAT2 */
	GPIO_CFG( 53, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* DAT1 */
	GPIO_CFG( 54, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* DAT0 */
	GPIO_CFG( 55, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* CMD */
	GPIO_CFG( 56, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* CLK */

	GPIO_CFG( 29, 0, GPIO_CFG_INPUT , GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* WLAN IRQ */
};

static unsigned wifi_on_gpio_table_cdma[] = {
	GPIO_CFG( 51, 1, GPIO_CFG_INPUT , GPIO_CFG_PULL_UP, GPIO_CFG_2MA), /* DAT3 */
	GPIO_CFG( 52, 1, GPIO_CFG_INPUT , GPIO_CFG_PULL_UP, GPIO_CFG_2MA), /* DAT2 */
	GPIO_CFG( 53, 1, GPIO_CFG_INPUT , GPIO_CFG_PULL_UP, GPIO_CFG_2MA), /* DAT1 */
	GPIO_CFG( 54, 1, GPIO_CFG_INPUT , GPIO_CFG_PULL_UP, GPIO_CFG_2MA), /* DAT0 */
	GPIO_CFG( 55, 1, GPIO_CFG_INPUT , GPIO_CFG_PULL_UP, GPIO_CFG_2MA), /* CMD */
	GPIO_CFG( 56, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* CLK */

	GPIO_CFG( 29, 0, GPIO_CFG_INPUT , GPIO_CFG_PULL_UP, GPIO_CFG_2MA), /* WLAN IRQ */
};

static unsigned wifi_off_gpio_table_cdma[] = {
	GPIO_CFG( 51, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* DAT3 */
	GPIO_CFG( 52, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* DAT2 */
	GPIO_CFG( 53, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* DAT1 */
	GPIO_CFG( 54, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* DAT0 */
	GPIO_CFG( 55, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* CMD */
	GPIO_CFG( 56, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* CLK */
	GPIO_CFG( 29, 0, GPIO_CFG_INPUT , GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* WLAN IRQ */
};

static uint opt_disable_sdcard;
static uint opt_disable_wifi;

static int __init disablesdcard_setup(char *str)
{
	int cal = simple_strtol(str, NULL, 0);

	opt_disable_sdcard = cal;
	return 1;
}

__setup("disable_sdcard=", disablesdcard_setup);

static int __init disablewifi_setup(char *str)
{
	int cal = simple_strtol(str, NULL, 0);

	opt_disable_wifi = cal;
	return 1;
}

__setup("disable_wifi=", disablewifi_setup);

static struct vreg *vreg_sdslot;	/* SD slot power */

struct mmc_vdd_xlat {
	int mask;
	int level;
};

#if 0
static struct mmc_vdd_xlat mmc_vdd_table[] = {
	{MMC_VDD_165_195, 1800},
	{MMC_VDD_20_21, 2050},
	{MMC_VDD_21_22, 2150},
	{MMC_VDD_22_23, 2250},
	{MMC_VDD_23_24, 2350},
	{MMC_VDD_24_25, 2450},
	{MMC_VDD_25_26, 2550},
	{MMC_VDD_26_27, 2650},
	{MMC_VDD_27_28, 2750},
	{MMC_VDD_28_29, 2850},
	{MMC_VDD_29_30, 2950},
};
#endif

static unsigned int sdslot_vdd = 0xffffffff;
static unsigned int sdslot_vreg_enabled;

static uint32_t sdslot_switchvdd(struct device *dev, unsigned int vdd)
{
	int rc;

	BUG_ON(!vreg_sdslot);

	if (vdd == sdslot_vdd)
		return 0;

	sdslot_vdd = vdd;

	if (vdd == 0) {
#if DEBUG_SDSLOT_VDD
		printk("%s: Disabling SD slot power\n", __func__);
#endif
		config_gpio_table(mmc_pdata.sdcard_off_gpio_table,
				  mmc_pdata.sdcard_off_gpio_table_size);
		vreg_disable(vreg_sdslot);
		sdslot_vreg_enabled = 0;
		return 0;
	}

	if (!sdslot_vreg_enabled) {
#if DEBUG_SDSLOT_VDD
		printk("%s: Enabling SD slot power (%d)\n", __func__, sdslot_vdd);
#endif
		rc = vreg_enable(vreg_sdslot);
		if (rc) {
			printk(KERN_ERR "%s: Error enabling vreg (%d)\n",
			       __func__, rc);
		}

		config_gpio_table(mmc_pdata.sdcard_on_gpio_table,
				  mmc_pdata.sdcard_on_gpio_table_size);
		sdslot_vreg_enabled = 1;
	}

/*	for (i = 0; i < ARRAY_SIZE(mmc_vdd_table); i++) {
		if (mmc_vdd_table[i].mask == (1 << vdd)) {
#if DEBUG_SDSLOT_VDD
			printk("%s: Setting level to %u\n",
			        __func__, mmc_vdd_table[i].level);
#endif
			rc = vreg_set_level(vreg_sdslot,
					    mmc_vdd_table[i].level);
			if (rc) {
				printk(KERN_ERR
				       "%s: Error setting vreg level (%d)\n",
				       __func__, rc);
			}
			return 0;
		}
	}
*/

	printk(KERN_ERR "%s: Invalid VDD %d specified\n", __func__, vdd);
	return 0;
}

static unsigned int sdslot_status(struct device *dev)
{
	unsigned int status;

	// For Diamond devices the MMC (MoviNAND) is built-in and always connected
	if (machine_is_htcdiamond() || machine_is_htcdiamond_cdma()) {
		return 1;
	}

	status = (unsigned int)gpio_get_value(mmc_pdata.sdcard_status_gpio);
	return (!status);
}

#define RAPH_MMC_VDD	MMC_VDD_165_195 | MMC_VDD_20_21 | MMC_VDD_21_22 \
			| MMC_VDD_22_23 | MMC_VDD_23_24 | MMC_VDD_24_25 \
			| MMC_VDD_25_26 | MMC_VDD_26_27 | MMC_VDD_27_28 \
			| MMC_VDD_28_29 | MMC_VDD_29_30

static struct mmc_platform_data sdslot_data = {
	.ocr_mask = RAPH_MMC_VDD,
	.status = sdslot_status,
	.translate_vdd = sdslot_switchvdd,
};

static struct vreg *vreg_wifi_osc;	/* WIFI 32khz oscilator */
static struct vreg *vreg_wifi_2;	/* WIFI foo? */
static struct vreg *vreg_wifi_3;	/* WIFI tre? */
static int wifi_cd = 0;		/* WIFI virtual 'card detect' status */

#ifdef CONFIG_MMC_EMBEDDED_SDIO
static struct sdio_embedded_func wifi_func = {
	.f_class = SDIO_CLASS_WLAN,
	.f_maxblksize = 512,
};

static struct sdio_cis ti_wifi_cis = {
	.vendor = 0x104,
	.device = 0x9066,
	.blksize = 512,
	/*.max_dtr      = 24000000,  Max of chip - no worky on Trout */
	.max_dtr = 11000000,
};

static struct sdio_cccr ti_wifi_cccr = {
	 .multi_block = 0,
	 .low_speed = 0,
	 .wide_bus = 1,
	 .high_power = 0,
	 .high_speed = 0,
};

static struct embedded_sdio_data ti_wifi_emb_data = {
	.cis = &ti_wifi_cis,
	.cccr = &ti_wifi_cccr,
	.funcs = &wifi_func,
	.num_funcs = 1,
};
#endif
#if 0
static struct sdio_cis bcm_wifi_cis = {
	.vendor = 0x02d0,
	.device = 0x4329,
	.blksize = 512,
	//.max_dtr      = 24000000, // Max of chip - no worky on Trout 
	.max_dtr = 11000000,
};
#endif

static struct sdio_cccr bcm_wifi_cccr = {
        .sdio_vsn       = 2,
        .multi_block    = 1,
        .low_speed      = 0,
        .wide_bus       = 0,
        .high_power     = 1,
        .high_speed     = 1,
};

#ifdef CONFIG_MMC_EMBEDDED_SDIO
static struct embedded_sdio_data bcm_wifi_emb_data = {
//	.cis = &bcm_wifi_cis,
        .cccr = &bcm_wifi_cccr,
//	.funcs = &wifi_func,
//	.num_funcs = 1,
};
#endif

static void (*wifi_status_cb) (int card_present, void *dev_id);
static void *wifi_status_cb_devid;

static int
wifi_status_register(void (*callback) (int card_present, void *dev_id),
		     void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

static unsigned int wifi_status(struct device *dev)
{
	return wifi_cd;
}

// trout_wifi_set_carddetect() is hard-coded in wlan driver...
int trout_wifi_set_carddetect(int val)
{
	printk("%s: %d\n", __func__, val);
	wifi_cd = val;
	if (wifi_status_cb) {
		wifi_status_cb(val, wifi_status_cb_devid);
	} else
		printk(KERN_WARNING "%s: Nobody to notify\n", __func__);
	return 0;
}

#ifndef CONFIG_WIFI_CONTROL_FUNC
EXPORT_SYMBOL(trout_wifi_set_carddetect);
#endif

static int wifi_power_state;

//XXX: trout_wifi_power() is hard-coded in wlan driver
int trout_wifi_power(int on)
{
	int rc;

	printk("%s: %d\n", __func__, on);

	if (on) {
		config_gpio_table(mmc_pdata.wifi_on_gpio_table,
				  mmc_pdata.wifi_on_gpio_table_size);

		rc = vreg_enable(vreg_wifi_osc);
		if (rc)
			return rc;
		rc = vreg_enable(vreg_wifi_2);
		if (rc)
			return rc;

		if(machine_is_htctopaz()) {
			rc = vreg_enable(vreg_wifi_3);
			if (rc)
				return rc;
		}
		mdelay(100);
		htc_pwrsink_set(PWRSINK_WIFI, 70);
		gpio_direction_output( mmc_pdata.wifi_power_gpio1, 0 );
		if( mmc_pdata.wifi_power_gpio2>=0) {
		  mdelay(50);
		  gpio_direction_output( mmc_pdata.wifi_power_gpio2, 0 );
		}
		mdelay(200);


	} else {
		config_gpio_table(mmc_pdata.wifi_off_gpio_table,
				  mmc_pdata.wifi_off_gpio_table_size);
		htc_pwrsink_set(PWRSINK_WIFI, 0);
	}
	gpio_direction_output(mmc_pdata.wifi_power_gpio1, on );
	mdelay(50);
	if( mmc_pdata.wifi_power_gpio2>=0) {
	  gpio_direction_output(mmc_pdata.wifi_power_gpio2, on );
	}
	if (!machine_is_htcrhodium())
	{
		/* Only used for TI WLAN */
		gpio_direction_input(29);
		irq_set_irq_wake(gpio_to_irq(29), on);
	}
	mdelay(150);

	if (!on) {
		if(machine_is_htctopaz()) {
			vreg_disable(vreg_wifi_3);
			//These vregs shuts the phone off raph/diam(/blac?)
			//So don't disable it for them.
			//The radio chip is fair enough not to drain everything anyway.
			//vreg_disable(vreg_wifi_osc);
			//vreg_disable(vreg_wifi_2);
		}
	}
	wifi_power_state = on;
	return 0;
}
#ifndef CONFIG_WIFI_CONTROL_FUNC
EXPORT_SYMBOL(trout_wifi_power);
#endif

static int wifi_reset_state;
int trout_wifi_reset(int on)
{
	printk("%s: %d\n", __func__, on);
//      gpio_set_value( TROUT_GPIO_CFG_WIFI_PA_RESETX, !on );
	wifi_reset_state = on;
	gpio_direction_output(mmc_pdata.wifi_power_gpio1, on);
	mdelay(100);
	gpio_direction_output(mmc_pdata.wifi_power_gpio1, !on);
	return 0;
}

#ifndef CONFIG_WIFI_CONTROL_FUNC
EXPORT_SYMBOL(trout_wifi_reset);
#endif

static struct wifi_platform_data bcm_pdata = {
	.name = "brcmfmac",
	.set_power = trout_wifi_power,
	.set_carddetect = trout_wifi_set_carddetect,
	.set_reset = trout_wifi_reset,
};

static struct resource resources_bcm[] = {
	{
		.start	= MSM_GPIO_TO_INT(29),
		.end	= MSM_GPIO_TO_INT(29),
		.flags	= IORESOURCE_IRQ,
		.name = "bcm4329_wlan_irq",
	},
};

struct platform_device bcm_plat = {
	.name		= "brcmfmac",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resources_bcm),
	.resource	= resources_bcm,
	.dev		= {
		.platform_data = &bcm_pdata,
	},
};

#define PREALLOC_WLAN_NUMBER_OF_SECTIONS	4
#define PREALLOC_WLAN_NUMBER_OF_BUFFERS		160
#define PREALLOC_WLAN_SECTION_HEADER		24

#define WLAN_SECTION_SIZE_0	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_1	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_2	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 512)
#define WLAN_SECTION_SIZE_3	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 1024)

#define WLAN_SKB_BUF_NUM	16

static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];

typedef struct wifi_mem_prealloc_struct {
	void *mem_ptr;
	unsigned long size;
} wifi_mem_prealloc_t;

static wifi_mem_prealloc_t wifi_mem_array[PREALLOC_WLAN_NUMBER_OF_SECTIONS] = {
	{ NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_1 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_2 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_3 + PREALLOC_WLAN_SECTION_HEADER) }
};

void *bcm_prealloc(int section, unsigned long size)
{
	if (section == PREALLOC_WLAN_NUMBER_OF_SECTIONS)
		return wlan_static_skb;
	if ((section < 0) || (section > PREALLOC_WLAN_NUMBER_OF_SECTIONS))
		return NULL;
	if (wifi_mem_array[section].size < size)
		return NULL;
	return wifi_mem_array[section].mem_ptr;
}

int bcm_init_wifi_mem(void)
{
	int i;

	for(i=0;( i < WLAN_SKB_BUF_NUM );i++) {
		if (i < (WLAN_SKB_BUF_NUM/2))
			wlan_static_skb[i] = dev_alloc_skb(4096);
		else
			wlan_static_skb[i] = dev_alloc_skb(8192);
	}
	for(i=0;( i < PREALLOC_WLAN_NUMBER_OF_SECTIONS );i++) {
		wifi_mem_array[i].mem_ptr = kmalloc(wifi_mem_array[i].size,
							GFP_KERNEL);
		if (wifi_mem_array[i].mem_ptr == NULL)
			return -ENOMEM;
	}
	return 0;
}
EXPORT_SYMBOL(bcm_prealloc);

/* bcm_wlan_power_ hardcoded in bcm4329 driver */
void bcm_wlan_power_off(unsigned power_mode)
{
	printk("%s: power_mode %d\n", __FUNCTION__, power_mode);

	switch (power_mode) {
	case 1:
		/* Unload driver */
		trout_wifi_power(0);
		trout_wifi_set_carddetect(0);
		msleep_interruptible(100);
		break;
	case 2:
		/* Stop driver */
		trout_wifi_power(0);
		break;
	default:
		printk("%s: ERROR unsupported power_mode %d\n", __FUNCTION__,
		       power_mode);
		break;
	}
}

EXPORT_SYMBOL(bcm_wlan_power_off);

void bcm_wlan_power_on(unsigned power_mode)
{
	printk("%s: power_mode %d\n", __FUNCTION__, power_mode);

	switch (power_mode) {
	case 1:
		/* Load driver */
		trout_wifi_power(1);
		trout_wifi_set_carddetect(1);
		msleep_interruptible(100);
		break;
	case 2:
		/* Start driver */
		trout_wifi_power(1);
		break;
	default:
		printk("%s: ERROR unsupported power_mode %d\n", __FUNCTION__,
		       power_mode);
		break;
	}
}

EXPORT_SYMBOL(bcm_wlan_power_on);

static struct mmc_platform_data wifi_data = {
	.ocr_mask = MMC_VDD_28_29,
	.status = wifi_status,
	.register_status_notify = wifi_status_register,
#ifdef CONFIG_MMC_EMBEDDED_SDIO
	.embedded_sdio = &ti_wifi_emb_data,
#endif
};

static struct mmc_dev_data cdma_mmc_pdata = {
	.sdcard_status_gpio = 38,
	.sdcard_device_id = 3,
	.wifi_power_gpio1 = 102,
	.wifi_power_gpio2 = 103,
	// gpio config tables
	.sdcard_on_gpio_table = sdcard_on_gpio_table_cdma,
	.sdcard_off_gpio_table = sdcard_off_gpio_table_cdma,
	.wifi_on_gpio_table = wifi_on_gpio_table_cdma,
	.wifi_off_gpio_table = wifi_off_gpio_table_cdma,
	// gpio config table sizes
	.sdcard_on_gpio_table_size = ARRAY_SIZE(sdcard_on_gpio_table_cdma),
	.sdcard_off_gpio_table_size = ARRAY_SIZE(sdcard_off_gpio_table_cdma),
	.wifi_on_gpio_table_size = ARRAY_SIZE(wifi_on_gpio_table_cdma),
	.wifi_off_gpio_table_size = ARRAY_SIZE(wifi_off_gpio_table_cdma),
};

static struct mmc_dev_data gsm_mmc_pdata = {
	.sdcard_status_gpio = 23,
	.sdcard_device_id = 2,
	.wifi_power_gpio1 = 102,
	.wifi_power_gpio2 = 103,
	// gpio config tables
	.sdcard_on_gpio_table = sdcard_on_gpio_table_gsm,
	.sdcard_off_gpio_table = sdcard_off_gpio_table_gsm,
	.wifi_on_gpio_table = wifi_on_gpio_table_gsm,
	.wifi_off_gpio_table = wifi_off_gpio_table_gsm,
	// table sizes
	.sdcard_on_gpio_table_size = ARRAY_SIZE(sdcard_on_gpio_table_gsm),
	.sdcard_off_gpio_table_size = ARRAY_SIZE(sdcard_off_gpio_table_gsm),
	.wifi_on_gpio_table_size = ARRAY_SIZE(wifi_on_gpio_table_gsm),
	.wifi_off_gpio_table_size = ARRAY_SIZE(wifi_off_gpio_table_gsm),
};

static int mmc_request_gpios(void) {
	int n, w, ret = 0;
	for (n = 0; n < 6; n++) {
		ret = gpio_request(GPIO_PIN(mmc_pdata.sdcard_on_gpio_table[n]), "MSM SDCC");
		if (ret)
			goto free_sd;
	}
	printk("%s: requested sd card gpios\n", __func__);
	
	for (w = 0; w < 7; w++) {
		ret = gpio_request(GPIO_PIN(mmc_pdata.wifi_on_gpio_table[w]), "MSM SDCC");
		if (ret)
			goto free_wifi;
	}
	printk("%s: requested wifi mmc gpios\n", __func__);

	if (mmc_pdata.sdcard_status_gpio > 0) {
		ret = gpio_request(mmc_pdata.sdcard_status_gpio, "MSM SDCC");
		if (ret)
			printk("failed to get sdcard status gpio: %d\n", mmc_pdata.sdcard_status_gpio);
	}

	if (mmc_pdata.wifi_power_gpio1 > 0) {
		ret = gpio_request(mmc_pdata.wifi_power_gpio1, "MSM SDCC");
		if (ret)
			printk("failed to get wifi_power_gpio1: %d\n", mmc_pdata.wifi_power_gpio1);
	}

	if (mmc_pdata.wifi_power_gpio2 > 0) {
		ret = gpio_request(mmc_pdata.wifi_power_gpio2, "MSM SDCC");
	if (ret)
			printk("failed to get wifi_power_gpio2: %d\n", mmc_pdata.wifi_power_gpio2);
	}
	printk("%s: requested misc mmc gpios\n", __func__);
	return 0;

free_wifi:
	for (--w; w >= 0; w--) {
		gpio_free(GPIO_PIN(mmc_pdata.wifi_on_gpio_table[w]));
	}
free_sd:
	for (--n; n >= 0; n--) {
		gpio_free(GPIO_PIN(mmc_pdata.sdcard_on_gpio_table[n]));
	}
	return ret;
}

int __init init_mmc(void)
{
	wifi_status_cb = NULL;

	sdslot_vreg_enabled = 0;

	switch(__machine_arch_type) {
		case MACH_TYPE_HTCRHODIUM:
#ifdef CONFIG_MMC_EMBEDDED_SDIO
			wifi_data.embedded_sdio=&bcm_wifi_emb_data;
#endif
			gsm_mmc_pdata.wifi_power_gpio1 = 93;
		case MACH_TYPE_HTCTOPAZ:
		case MACH_TYPE_HTCWHITESTONE:
			gsm_mmc_pdata.sdcard_status_gpio=38;
			gsm_mmc_pdata.wifi_power_gpio2 = 0;
		case MACH_TYPE_HTCRAPHAEL:
		case MACH_TYPE_HTCDIAMOND:
		case MACH_TYPE_HTCBLACKSTONE:
			mmc_pdata = gsm_mmc_pdata;
			break;
		case MACH_TYPE_HTCDIAMOND_CDMA:
			mmc_pdata = gsm_mmc_pdata;
			/* MoviNAND returns 1.8V in OCR, but (for whatever reason)
			 * doesn't work at 1.8V
			 */
			sdslot_data.ocr_mask = MMC_VDD_29_30;
			break;
		case MACH_TYPE_HTCKOVSKY:
			cdma_mmc_pdata.sdcard_status_gpio = 94;
		case MACH_TYPE_HTCRAPHAEL_CDMA:
		case MACH_TYPE_HTCRAPHAEL_CDMA500:
			mmc_pdata = cdma_mmc_pdata;
			break;
		default:
			printk(KERN_ERR "Unsupported device for mmc driver\n");
			return -1;
			break;
	}

	mmc_request_gpios();

	vreg_sdslot = vreg_get(0, "gp6");
	if (IS_ERR(vreg_sdslot))
		return PTR_ERR(vreg_sdslot);

	vreg_wifi_osc = vreg_get(0, "msmp");
	if (IS_ERR(vreg_wifi_osc))
		return PTR_ERR(vreg_wifi_osc);

	vreg_wifi_2 = vreg_get(0, "msme1");
	if (IS_ERR(vreg_wifi_2))
		return PTR_ERR(vreg_wifi_2);

	vreg_wifi_3 = vreg_get(0, "rftx");
	if (IS_ERR(vreg_wifi_3))
		return PTR_ERR(vreg_wifi_3);

	if (!opt_disable_wifi)
	{
		printk(KERN_INFO "MMC: WiFi device enable\n");
		msm_add_sdcc(1, &wifi_data, 0, 0);
		platform_device_register(&bcm_plat);
	}
	else
		printk(KERN_INFO "MMC: WiFi device disabled\n");

	if (!opt_disable_sdcard)
	{
		printk(KERN_INFO "MMC: SD-Card interface enable\n");
		irq_set_irq_wake(MSM_GPIO_TO_INT(mmc_pdata.sdcard_status_gpio), 1);
		msm_add_sdcc(mmc_pdata.sdcard_device_id, &sdslot_data,
				MSM_GPIO_TO_INT(mmc_pdata.sdcard_status_gpio),
				IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING);
	} else
		printk(KERN_INFO "MMC: SD-Card interface disabled\n");

	bcm_init_wifi_mem();
	return 0;
}

#if defined(CONFIG_DEBUG_FS)
static int mmc_dbg_wifi_reset_set(void *data, u64 val)
{
	trout_wifi_reset((int)val);
	return 0;
}

static int mmc_dbg_wifi_reset_get(void *data, u64 * val)
{
	*val = wifi_reset_state;
	return 0;
}

static int mmc_dbg_wifi_cd_set(void *data, u64 val)
{
	trout_wifi_set_carddetect((int)val);
	return 0;
}

static int mmc_dbg_wifi_cd_get(void *data, u64 * val)
{
	*val = wifi_cd;
	return 0;
}

static int mmc_dbg_wifi_pwr_set(void *data, u64 val)
{
	trout_wifi_power((int)val);
	return 0;
}

static int mmc_dbg_wifi_pwr_get(void *data, u64 * val)
{

	*val = wifi_power_state;
	return 0;
}

static int mmc_dbg_sd_pwr_set(void *data, u64 val)
{
	sdslot_switchvdd(NULL, (unsigned int)val);
	return 0;
}

static int mmc_dbg_sd_pwr_get(void *data, u64 * val)
{
	*val = sdslot_vdd;
	return 0;
}

static int mmc_dbg_sd_cd_set(void *data, u64 val)
{
	return -ENOSYS;
}

static int mmc_dbg_sd_cd_get(void *data, u64 * val)
{
	*val = sdslot_status(NULL);
	return 0;
}

static int mmc_dbg_wifi1_gpio_set(void *data, u64 val)
{
	gpio_direction_output(mmc_pdata.wifi_power_gpio1, val);
	return 0;
}

static int mmc_dbg_wifi1_gpio_get(void *data, u64 * val)
{
	*val = 0;
	return 0;
}

static int mmc_dbg_wifi2_gpio_set(void *data, u64 val)
{
	if (mmc_pdata.wifi_power_gpio2 >= 0)
		gpio_direction_output(mmc_pdata.wifi_power_gpio2, val);
	return 0;
}

static int mmc_dbg_wifi2_gpio_get(void *data, u64 * val)
{
	*val = 0;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(mmc_dbg_wifi_reset_fops,
			mmc_dbg_wifi_reset_get,
			mmc_dbg_wifi_reset_set, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(mmc_dbg_wifi_cd_fops,
			mmc_dbg_wifi_cd_get, mmc_dbg_wifi_cd_set, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(mmc_dbg_wifi_pwr_fops,
			mmc_dbg_wifi_pwr_get, mmc_dbg_wifi_pwr_set, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(mmc_dbg_wifi1_gpio_fops,
			mmc_dbg_wifi1_gpio_get,
			mmc_dbg_wifi1_gpio_set, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(mmc_dbg_wifi2_gpio_fops,
			mmc_dbg_wifi2_gpio_get,
			mmc_dbg_wifi2_gpio_set, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(mmc_dbg_sd_pwr_fops,
			mmc_dbg_sd_pwr_get, mmc_dbg_sd_pwr_set, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(mmc_dbg_sd_cd_fops,
			mmc_dbg_sd_cd_get, mmc_dbg_sd_cd_set, "%llu\n");

static int __init mmc_dbg_init(void)
{
	struct dentry *dent;

	dent = debugfs_create_dir("mmc_dbg", 0);
	if (IS_ERR(dent))
		return PTR_ERR(dent);

	debugfs_create_file("wifi_gpio1", 0644, dent, NULL,
			    &mmc_dbg_wifi1_gpio_fops);
	debugfs_create_file("wifi_gpio2", 0644, dent, NULL,
			    &mmc_dbg_wifi2_gpio_fops);
	debugfs_create_file("wifi_reset", 0644, dent, NULL,
			    &mmc_dbg_wifi_reset_fops);
	debugfs_create_file("wifi_cd", 0644, dent, NULL, &mmc_dbg_wifi_cd_fops);
	debugfs_create_file("wifi_pwr", 0644, dent, NULL,
			    &mmc_dbg_wifi_pwr_fops);

	debugfs_create_file("sd_pwr", 0644, dent, NULL, &mmc_dbg_sd_pwr_fops);
	debugfs_create_file("sd_cd", 0644, dent, NULL, &mmc_dbg_sd_cd_fops);

	return 0;
}

device_initcall(mmc_dbg_init);

#endif
