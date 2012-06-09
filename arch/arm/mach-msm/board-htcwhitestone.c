/* linux/arch/arm/mach-msm/board-htcwhitestone.c
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/android_pmem.h>

#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/i2c.h>
#include <linux/mm.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
//#include <asm/mach/mmc.h>
#include <asm/setup.h>
#include <linux/platform_data/msm_serial_hs.h>

#include <mach/board.h>
#include <mach/htc_battery.h>
#include <mach/msm_iomap.h>
#include <mach/system.h>
#include <mach/msm_fb.h>
#include <mach/msm_hsusb.h>
#include <mach/vreg.h>

#include <mach/gpio.h>
#include <mach/io.h>
#include <linux/delay.h>
#include <linux/gpio_keys.h>

#include <linux/microp-keypad.h>
#include <linux/microp-ng.h>
#include <mach/board_htc.h>
#include <mach/htc_headset.h>

#include "proc_comm_wince.h"
#include "devices.h"
//#include "htc_hw.h"
#include "board-htcwhitestone.h"
//#include "htc-usb.h"

#include <linux/usb/composite.h>
#include <linux/usb/android_composite.h>

extern int init_mmc(void);

#ifdef CONFIG_SERIAL_MSM_HS
static struct msm_serial_hs_platform_data msm_uart_dm2_pdata = {
	.rx_wakeup_irq = MSM_GPIO_TO_INT(21),
	.inject_rx_on_wakeup = 1,
	.rx_to_inject = 0x32,
};
#endif

static int usb_phy_init_seq_raph100[] = {
	0x40, 0x31, /* Leave this pair out for USB Host Mode */
	0x1D, 0x0D,
	0x1D, 0x10,
	-1
};

static struct platform_device htcwhitestone_microp_leds = {
	.id = -1,
	.name = "htcwhitestone-microp-leds",
};

static struct platform_device* htcwhitestone_microp_clients[] = {
	&htcwhitestone_microp_leds,
};

static uint16_t micropklt_compatible_versions[] = {
	0x0386
};

static struct microp_platform_data htcwhitestone_microp_led_audio_pdata = {
	.version_reg = 0x30,
	.clients = htcwhitestone_microp_clients,
	.nclients = ARRAY_SIZE(htcwhitestone_microp_clients),
	.comp_versions = micropklt_compatible_versions,
	.n_comp_versions = ARRAY_SIZE(micropklt_compatible_versions),
};
/******************************************************************************
 * GPIO
 ******************************************************************************/
static struct gpio_keys_button whitestone_button_table[] = {
	{KEY_MENU,	WHIT100_VOLUP_KEY,	1,	"Menu",		EV_KEY, 1,	0},
	{KEY_BACK,	WHIT100_VOLDOWN_KEY,	1,	"Back",		EV_KEY, 1,	0},
	{KEY_END,	WHIT100_SEND_KEY,	1,	"Back",		EV_KEY, 1,	0},
	{KEY_POWER,     WHIT100_POWER_KEY,	1,	"Power button",	EV_KEY, 1,	0},
};

static struct gpio_keys_platform_data gpio_keys_data = {
	.buttons = whitestone_button_table,
	.nbuttons=ARRAY_SIZE(whitestone_button_table),
};

static struct platform_device gpio_keys = {
	.name = "gpio-keys",
	.dev  = {
		.platform_data = &gpio_keys_data,
	},
	.id   = -1,
};
/******************************************************************************
 * USB
 ******************************************************************************/
static unsigned ulpi_on_gpio_table[] = {
	GPIO_CFG(0x6f, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
	GPIO_CFG(0x70, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
	GPIO_CFG(0x71, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
	GPIO_CFG(0x72, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
	GPIO_CFG(0x73, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
	GPIO_CFG(0x74, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
	GPIO_CFG(0x75, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
	GPIO_CFG(0x76, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
	GPIO_CFG(0x77, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
	GPIO_CFG(0x78, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
	GPIO_CFG(0x79, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
};

static unsigned ulpi_off_gpio_table[] = {
	GPIO_CFG(0x6f, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
	GPIO_CFG(0x70, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
	GPIO_CFG(0x71, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
	GPIO_CFG(0x72, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
	GPIO_CFG(0x73, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
	GPIO_CFG(0x74, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
	GPIO_CFG(0x75, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
	GPIO_CFG(0x76, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
	GPIO_CFG(0x77, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
	GPIO_CFG(0x78, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
	GPIO_CFG(0x79, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA),
};

static void usb_gpio_init(void)
{
	if (gpio_request(0x6f, "ulpi_data_0"))
		pr_err("failed to request gpio ulpi_data_0\n");
	if (gpio_request(0x70, "ulpi_data_1"))
		pr_err("failed to request gpio ulpi_data_1\n");
	if (gpio_request(0x71, "ulpi_data_2"))
		pr_err("failed to request gpio ulpi_data_2\n");
	if (gpio_request(0x72, "ulpi_data_3"))
		pr_err("failed to request gpio ulpi_data_3\n");
	if (gpio_request(0x73, "ulpi_data_4"))
		pr_err("failed to request gpio ulpi_data_4\n");
	if (gpio_request(0x74, "ulpi_data_5"))
		pr_err("failed to request gpio ulpi_data_5\n");
	if (gpio_request(0x75, "ulpi_data_6"))
		pr_err("failed to request gpio ulpi_data_6\n");
	if (gpio_request(0x76, "ulpi_data_7"))
		pr_err("failed to request gpio ulpi_data_7\n");
	if (gpio_request(0x77, "ulpi_dir"))
		pr_err("failed to request gpio ulpi_dir\n");
	if (gpio_request(0x78, "ulpi_next"))
		pr_err("failed to request gpio ulpi_next\n");
	if (gpio_request(0x79, "ulpi_stop"))
		pr_err("failed to request gpio ulpi_stop\n");
}

static int usb_config_gpio(int config)
{
	int pin, rc;

	if (config) {
		for (pin = 0; pin < ARRAY_SIZE(ulpi_on_gpio_table); pin++) {			
			rc = gpio_tlmm_config(ulpi_on_gpio_table[pin],
					      GPIO_CFG_ENABLE);
			if (rc) {
				printk(KERN_ERR
				       "%s: gpio_tlmm_config(%#x)=%d\n",
				       __func__, ulpi_off_gpio_table[pin], rc);
				return -EIO;
			}
		}
	} else {
		for (pin = 0; pin < ARRAY_SIZE(ulpi_off_gpio_table); pin++) {
			rc = gpio_tlmm_config(ulpi_off_gpio_table[pin],
					      GPIO_CFG_ENABLE);
			if (rc) {
				printk(KERN_ERR
				       "%s: gpio_tlmm_config(%#x)=%d\n",
				       __func__, ulpi_on_gpio_table[pin], rc);
				return -EIO;
			}
		}
	}

	return 0;
}

static void usb_phy_shutdown(void)
{
	/* is mdelay needed ? */
	gpio_set_value(WHIT100_USBPHY_RST, 1);
	mdelay(1);
	gpio_set_value(WHIT100_USBPHY_RST, 0);
	mdelay(1);
}
static void usb_phy_reset(void)
{
	printk("%s: %s\n", __FILE__, __func__);
	usb_phy_shutdown();
	gpio_set_value(WHIT100_USBPHY_RST, 0); 
	mdelay(3);
	gpio_set_value(WHIT100_USBPHY_RST, 1);
	mdelay(3);
	usb_config_gpio(1);

}

static struct msm_otg_platform_data msm_otg_pdata = {
//         .rpc_connect    = hsusb_rpc_connect,
 	.phy_reset = usb_phy_reset,
#ifndef CONFIG_USB_EHCI_MSM_72K
 //       .pmic_vbus_notif_init         = msm_hsusb_pmic_notif_init,
#else
         .vbus_power = msm_hsusb_vbus_power,
#endif
//         .core_clk                = 1,
         .pemp_level              = PRE_EMPHASIS_WITH_20_PERCENT,
         .cdr_autoreset           = CDR_AUTO_RESET_DISABLE,
         .drv_ampl                = HS_DRV_AMPLITUDE_DEFAULT,
         .se1_gating              = SE1_GATING_DISABLE,
//       .chg_vbus_draw           = hsusb_chg_vbus_draw,
//         .chg_connected           = hsusb_chg_connected,
//         .chg_init                = hsusb_chg_init,
//         .ldo_enable              = msm_hsusb_ldo_enable,
//         .ldo_init                = msm_hsusb_ldo_init,
//         .ldo_set_voltage         = msm_hsusb_ldo_set_voltage,
};

static struct resource resources_gadget_peripheral[] = {
         {
                .start  = MSM_HSUSB_PHYS,
                .end    = MSM_HSUSB_PHYS + SZ_1K - 1,
                .flags  = IORESOURCE_MEM,
         },
         {
                .start  = INT_USB_HS,
                .end    = INT_USB_HS,
                .flags  = IORESOURCE_IRQ,
         },
};

static u64 dma_mask = 0xffffffffULL;
static struct platform_device msm_device_gadget_peripheral = {
        .name           = "msm_hsusb",
        .id             = -1,
        .num_resources  = ARRAY_SIZE(resources_gadget_peripheral),
        .resource       = resources_gadget_peripheral,
        .dev            = {
                .dma_mask               = &dma_mask,
                .coherent_dma_mask      = 0xffffffffULL,
         },
};

static struct msm_hsusb_gadget_platform_data msm_gadget_pdata = {
	.is_phy_status_timer_on = 1,
//	.self_powered
};

#if 0 //defined(CONFIG_MSM_CAMERA) && defined(CONFIG_MT9P012)

static struct msm_gpio_config camera_off_gpio_table[] = {
    /* CAMERA */
    DEX_GPIO_CFG(0, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA, 0), /* DAT0 */
    DEX_GPIO_CFG(1, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA, 0), /* DAT1 */
    DEX_GPIO_CFG(2, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA, 0), /* DAT2 */
    DEX_GPIO_CFG(3, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA, 0), /* DAT3 */
    DEX_GPIO_CFG(4, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA, 0), /* DAT4 */
    DEX_GPIO_CFG(5, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA, 0), /* DAT5 */
    DEX_GPIO_CFG(6, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA, 0), /* DAT6 */
    DEX_GPIO_CFG(7, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA, 0), /* DAT7 */
    DEX_GPIO_CFG(8, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA, 0), /* DAT8 */
    DEX_GPIO_CFG(9, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA, 0), /* DAT9 */
    //DEX_GPIO_CFG(10, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA, 0), /* DAT10 */
    //DEX_GPIO_CFG(11, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA, 0), /* DAT11 */
    DEX_GPIO_CFG(12, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA, 0), /* PCLK */
    DEX_GPIO_CFG(13, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA, 0), /* HSYNC_IN */
    DEX_GPIO_CFG(14, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA, 0), /* VSYNC_IN */
    DEX_GPIO_CFG(15, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA, 0), /* MCLK */
};

static struct msm_gpio_config camera_on_gpio_table[] = {
    /* CAMERA */
    DEX_GPIO_CFG(0, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA, 0), /* DAT0 */
    DEX_GPIO_CFG(1, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA, 0), /* DAT1 */
    DEX_GPIO_CFG(2, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA, 0), /* DAT2 */
    DEX_GPIO_CFG(3, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA, 0), /* DAT3 */
    DEX_GPIO_CFG(4, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA, 0), /* DAT4 */
    DEX_GPIO_CFG(5, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA, 0), /* DAT5 */
    DEX_GPIO_CFG(6, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA, 0), /* DAT6 */
    DEX_GPIO_CFG(7, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA, 0), /* DAT7 */
    DEX_GPIO_CFG(8, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA, 0), /* DAT8 */
    DEX_GPIO_CFG(9, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA, 0), /* DAT9 */
    //DEX_GPIO_CFG(10, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA, 0), /* DAT10 */
    //DEX_GPIO_CFG(11, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA, 0), /* DAT11 */
    DEX_GPIO_CFG(12, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA, 0), /* PCLK */
    DEX_GPIO_CFG(13, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA, 0), /* HSYNC_IN */
    DEX_GPIO_CFG(14, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA, 0), /* VSYNC_IN */
    DEX_GPIO_CFG(15, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA, 0), /* MCLK */
};

static void config_gpio_table(struct msm_gpio_config *table, int len)
{
    int n;
    struct msm_gpio_config id;
    for(n = 0; n < len; n++) {
        id = table[n];
        msm_gpio_set_function( id );
    }
}

static void config_camera_on_gpios(int Gpio30Value)
{
    struct msm_dex_command dex = { .cmd = PCOM_PMIC_REG_ON, .has_data=1 };

    /* VDDD 1.8V */
    dex.data=0x100;
    msm_proc_comm_wince(&dex,0);
    mdelay(20);
    /* VDDIO 2.65V */
    dex.data=0x400000;
    msm_proc_comm_wince(&dex,0);
    mdelay(20);
    /* VDDA 2.85V */
    dex.data=0x20;
    msm_proc_comm_wince(&dex,0);
    mdelay(20);

    /* Select camera (front or back) */
    gpio_direction_output(WHIT100_CAM_PWR1, Gpio30Value);

    config_gpio_table(camera_on_gpio_table,
        ARRAY_SIZE(camera_on_gpio_table));
}

static void config_camera_off_gpios(void)
{
    struct msm_dex_command dex = { .cmd = PCOM_PMIC_REG_OFF, .has_data=1 };

    /* VDDD 1.8V */
    dex.data=0x100;
    msm_proc_comm_wince(&dex,0);
    mdelay(20);
    /* VDDIO 2.65V */
    dex.data=0x400000;
    msm_proc_comm_wince(&dex,0);
    mdelay(20);
    /* VDDA 2.85V */
    dex.data=0x20;
    msm_proc_comm_wince(&dex,0);
    mdelay(20);

    /* Reset camera select gpio to main (back) camera */
    gpio_direction_output(WHIT100_CAM_PWR1, 1);

    config_gpio_table(camera_off_gpio_table,
        ARRAY_SIZE(camera_off_gpio_table));
}

#ifdef CONFIG_MT9P012
static void config_camera_on_gpios_mt9p012(void)
{
    config_camera_on_gpios(0);
}
#endif

#endif

static struct i2c_board_info i2c_devices[] = {
	{
		// LED & Backlight controller
		I2C_BOARD_INFO("microp-ng", 0x66),
		.platform_data = &htcwhitestone_microp_led_audio_pdata
	},
#if defined(CONFIG_MSM_CAMERA) && defined(CONFIG_MT9P012)
	{		
		I2C_BOARD_INFO("mt9p012", 0x6c >> 1),
	},
#endif
};

#define SND(num, desc) { .name = desc, .id = num }
static struct snd_endpoint snd_endpoints_list[] = {
	SND(0, "HANDSET"),
	SND(1, "SPEAKER"),
	SND(2, "HEADSET"),
	SND(3, "BT"),
	SND(44, "BT_EC_OFF"),
	SND(10, "HEADSET_AND_SPEAKER"),
	SND(256, "CURRENT"),

};
#undef SND

static struct msm_snd_endpoints htcwhitestone_snd_endpoints = {
        .endpoints = snd_endpoints_list,
        .num = ARRAY_SIZE(snd_endpoints_list),
};

static struct platform_device htcwhitestone_snd = {
	.name = "msm_snd",
	.id = -1,
	.dev	= {
		.platform_data = &htcwhitestone_snd_endpoints,
	},
};

#if 0 //CONFIG_HTC_HEADSET
static void htcwhitestone_h2w_config_cpld(int route);
static void htcwhitestone_h2w_init_cpld(void);
static struct h2w_platform_data htcwhitestone_h2w_data = {
	.cable_in1              = WHIT100_CABLE_IN1,
	.cable_in2              = WHIT100_CABLE_IN2,
	.h2w_clk                = WHIT100_H2W_CLK,
	.h2w_data               = WHIT100_H2W_DATA,
	.debug_uart             = H2W_UART3,
	.config_cpld            = htcwhitestone_h2w_config_cpld,
	.init_cpld              = htcwhitestone_h2w_init_cpld,
	.headset_mic_35mm       = WHIT100_AUD_HSMIC_DET_N,
};

static void htcwhitestone_h2w_config_cpld(int route)
{
	printk(KERN_INFO "%s: route=%d TODO\n", __func__, route);
	switch (route) {
		case H2W_UART3:
			//gpio_set_value(0, 1); 	/*TODO wrong GPIO*/
			break;
		case H2W_GPIO:
			//gpio_set_value(0, 0); 	/*TODO wrong GPIO*/
			break;
	}
}

static void htcwhitestone_h2w_init_cpld(void)
{
	htcwhitestone_h2w_config_cpld(H2W_UART3);
	gpio_set_value(htcwhitestone_h2w_data.h2w_clk, 0);
	gpio_set_value(htcwhitestone_h2w_data.h2w_data, 0);
}

static struct platform_device htcwhitestone_h2w = {
	.name           = "h2w",
	.id             = -1,
	.dev            = {
		.platform_data  = &htcwhitestone_h2w_data,
	},
};
#endif

static struct platform_device htcwhitestone_bt_rfkill = {
	.name = "htcraphael_rfkill",
	.id = -1,
};

static struct platform_device touchscreen = {
	.name		= "tssc-manager",
	.id		= -1,
};

#if 0 //def CONFIG_MSM_CAMERA

#ifdef CONFIG_MT9P012
static struct msm_camera_device_platform_data msm_camera_device_data_mt9p012 = {
    .camera_gpio_on  = config_camera_on_gpios_mt9p012,
    .camera_gpio_off = config_camera_off_gpios,
    .ioext.mdcphy = MSM_MDC_PHYS,
    .ioext.mdcsz  = MSM_MDC_SIZE,
    .ioext.appphy = MSM_CLK_CTL_PHYS,
    .ioext.appsz  = MSM_CLK_CTL_SIZE,
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9p012_data = {
    .sensor_name    = "mt9p012",
    .sensor_reset   = WHIT100_CAM1_RST,
    .sensor_pwd     = WHIT100_CAM_PWR1,
    .vcm_pwd        = WHIT100_CAM_VCMPDP,
    .pdata          = &msm_camera_device_data_mt9p012,
};

static struct platform_device msm_camera_sensor_mt9p012 = {
    .name           = "msm_camera_mt9p012",
    .dev            = {
	    .platform_data = &msm_camera_sensor_mt9p012_data,
    },
};
#endif

#endif

static struct platform_device msm_device_rtc = {
	.name = "msm_rtc",
	.id = -1,
};

static struct platform_device htc_hw = {
	.name = "htc_hw",
	.id = -1,
};

static struct platform_device msm_device_htc_battery = {
        .name = "htc_battery",
        .id = -1,
};

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
};

static char *usb_functions_rndis[] = {
	"rndis",
};

static char *usb_functions_ums[] = {
	"usb_mass_storage",
};

static char *usb_functions_ums_adb[] = {
	"usb_mass_storage",
	"adb",
};

static char *usb_functions_all[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
	"usb_mass_storage",
	"adb",
};


static struct android_usb_product usb_products[] = {
	{
		.product_id	= 0x0ffe,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis),
		.functions	= usb_functions_rndis,
	},	
	{
		.product_id	= 0x0c01,
		.num_functions	= ARRAY_SIZE(usb_functions_ums),
		.functions	= usb_functions_ums,
	},
	{
		.product_id	= 0x0c02,
		.num_functions	= ARRAY_SIZE(usb_functions_ums_adb),
		.functions	= usb_functions_ums_adb,
	},
};

static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns		= 1,
	.vendor		= "HTC",
	.product	= "XDA",
	.release	= 0x0100,
};

static struct platform_device usb_mass_storage_device = {
	.name	= "usb_mass_storage",
	.id	= -1,
	.dev	= {
		.platform_data = &mass_storage_pdata,
	},
};

#ifdef CONFIG_USB_ANDROID_RNDIS
static struct usb_ether_platform_data rndis_pdata = {
	/* ethaddr is filled by board_serialno_setup */
	.vendorID	= 0x18d1,
	.vendorDescr	= "HTC",
};

static struct platform_device rndis_device = {
	.name	= "rndis",
	.id	= -1,
	.dev	= {
		.platform_data = &rndis_pdata,
	},
};
#endif


static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= 0x0bb4,
	.product_id	= 0x0c01,
	.version	= 0x0100,
	.serial_number		= "000000000000",
	.product_name		= "XDA",
	.manufacturer_name	= "HTC",
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};


static struct platform_device *devices[] __initdata = {
	&msm_device_smd,
	&msm_device_nand,
	&msm_device_i2c,
	&msm_device_rtc,
	&htc_hw,
	&msm_device_htc_battery,
	&htcwhitestone_snd,
	&msm_device_otg,
	&msm_device_gadget_peripheral,
#ifdef CONFIG_HTC_HEADSET
	//~ &htcwhitestone_h2w,
#endif
	//~ &htcwhitestone_bt_rfkill,
	&touchscreen,
	&gpio_keys,
#ifdef CONFIG_SERIAL_MSM_HS
	&msm_device_uart_dm2,
#endif
#ifdef CONFIG_MT9P012
	//~ &msm_camera_sensor_mt9p012,
#endif
#ifdef CONFIG_USB_ANDROID
#ifdef CONFIG_USB_ANDROID_RNDIS
	&rndis_device,
#endif
	&usb_mass_storage_device,
	&android_usb_device,
#endif
};

extern struct sys_timer msm_timer;

static void __init htcwhitestone_init_irq(void)
{
	msm_init_irq();
}

static struct msm_acpu_clock_platform_data halibut_clock_data = {
	.acpu_switch_time_us = 50,
	.max_speed_delta_khz = 256000,
	.vdd_switch_time_us = 62,
	.power_collapse_khz = 19200,
	.wait_for_irq_khz = 128000,
	.max_axi_khz = 160000,
};

void msm_serial_debug_init(unsigned int base, int irq, 
			   const char *clkname, int signal_irq);

/*static htc_hw_pdata_t msm_htc_hw_pdata = {
	.set_vibrate = msm_proc_comm_wince_vibrate,
	.battery_smem_offset = 0xfc110,
	.battery_smem_field_size = 2,
};*/

static smem_batt_t msm_battery_pdata = {
	.gpio_battery_detect = WHIT100_BAT_IN,
	.gpio_charger_enable = WHIT100_CHARGE_EN_N,
	.gpio_charger_current_select = WHIT100_USB_AC_PWR,
	.smem_offset = 0xfc110,
	.smem_field_size = 2,
};

void msm_init_pmic_vibrator(void);
static void msm_proc_comm_wince_vibrate(int val)
{
	struct msm_dex_command vibra;

	if (val == 0) {
		vibra.cmd = PCOM_VIBRA_OFF;
		msm_proc_comm_wince(&vibra, 0);
	} else if (val > 0) {
		if (val == 1 || val > 0xb22)
			val = 0xb22;
		writel(val, MSM_SHARED_RAM_BASE + 0xfc130);
		vibra.cmd = PCOM_VIBRA_ON;
		msm_proc_comm_wince(&vibra, 0);
	}
}

static void __init htcwhitestone_init(void)
{
	int i;

	ram_console_resource[0].start = 0x8e0000;
	ram_console_resource[0].end = 0x8e0000 + 0x20000 - 1;
	platform_device_register(&ram_console_device);

	msm_acpu_clock_init(&halibut_clock_data);
	msm_proc_comm_wince_init();

	//htc_hw.dev.platform_data = &msm_htc_hw_pdata;
	msm_device_htc_battery.dev.platform_data = &msm_battery_pdata;


	//msm_device_hsusb.dev.platform_data = &msm_hsusb_pdata;
	msm_device_otg.dev.platform_data = &msm_otg_pdata;
	msm_device_gadget_peripheral.dev.platform_data = &msm_gadget_pdata;
	
	usb_gpio_init();
	
	platform_add_devices(devices, ARRAY_SIZE(devices));
	i2c_register_board_info(0, i2c_devices, ARRAY_SIZE(i2c_devices));
	//msm_add_usb_devices(usb_phy_reset, NULL, usb_phy_init_seq_raph100);
	init_mmc();
#ifdef CONFIG_SERIAL_MSM_HS
	msm_device_uart_dm2.dev.platform_data = &msm_uart_dm2_pdata;
#endif
	msm_init_pmic_vibrator();

	/* TODO: detect vbus and correctly notify USB about its presence 
	 * For now we just declare that VBUS is present at boot and USB
	 * copes, but this is not ideal.
	 */
//	msm_hsusb_set_vbus_state(1);


	/* A little vibrating welcome */
	for (i = 0; i < 2; i++) {
		msm_proc_comm_wince_vibrate(1);
		mdelay(150);
		msm_proc_comm_wince_vibrate(0);
		mdelay(75);
	}
}

static void __init htcwhitestone_map_io(void)
{
	msm_map_common_io();
//	iotable_init(trout_io_desc, ARRAY_SIZE(trout_io_desc));

#ifdef CONFIG_MSM_DEBUG_UART3
	/* route UART3 to the "H2W" extended usb connector */
//	writeb(0x80, TROUT_CPLD_BASE + 0x00);
#endif
	msm_clock_a11_fixup();
	msm_clock_init(msm_clocks_7x01a, msm_num_clocks_7x01a);
}

static void __init htcwhitestone_init_early(void)
{
       arch_ioremap_caller = __msm_ioremap_caller;
}

static void __init htcwhitestone_fixup(struct tag *tags, char **cmdline, struct meminfo *mi)
{
	mi->nr_banks = 2;
	mi->bank[0].start = PAGE_ALIGN(PHYS_OFFSET);
	//mi->bank[0].size = (104 * 1024 * 1024);
	mi->bank[0].size = (102 * 1024 * 1024);

	/* Whitestone is 1x256MB monodie. So contiguous memory. */
	mi->bank[1].start = PAGE_ALIGN(PHYS_OFFSET + 0x08000000);
	mi->bank[1].size = (128 * 1024 * 1024)-50*1024*1024; // See pmem.c

	printk(KERN_INFO "%s: nr_banks = %d\n", __func__, mi->nr_banks);
	printk(KERN_INFO "%s: bank0 start=%08lx, size=%08lx\n", __func__,
		mi->bank[0].start, mi->bank[0].size);
	printk(KERN_INFO "%s: bank1 start=%08lx, size=%08lx\n", __func__,
		mi->bank[1].start, mi->bank[1].size);
}

MACHINE_START(HTCWHITESTONE, "HTC Whitestone cellphone")
	.fixup 		= htcwhitestone_fixup,
	.atag_offset 	= 0x100,
	.map_io		= htcwhitestone_map_io,
	.init_irq	= htcwhitestone_init_irq,
	.init_machine	= htcwhitestone_init,
	.init_early     = htcwhitestone_init_early,
	.timer		= &msm_timer,
MACHINE_END
