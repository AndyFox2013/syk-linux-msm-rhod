/* linux/arch/arm/mach-msm/board-htcrhodium.c
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
#include <linux/export.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/mm.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>

//#include <mach/msm72k_otg.h>

#include "gpio_chip.h"
#include <mach/gpio.h>
#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/gpio.h>
#include <asm/mach/flash.h>
#include <linux/mmc/sdio_ids.h>
#include <asm/setup.h>
//#include <mach/msm_serial_hs.h>

#include <mach/board.h>
#include <mach/htc_battery.h>
#include <mach/msm_iomap.h>
#include <mach/msm_hsusb.h>
//#include <mach/msm_serial_hs.h>
#include <mach/msm7200a_rfkill.h>
#include <linux/bma150.h>
#include <linux/capella_cm3602.h>

#include <mach/system.h>
#include <mach/msm_fb.h>
#include <mach/msm_ts.h>
#include <mach/vreg.h>

#include <mach/gpio.h>
#include <mach/io.h>
#include <linux/delay.h>
#include <linux/gpio_keys.h>
#ifdef CONFIG_HTC_HEADSET
#include <mach/htc_headset.h>
#include <mach/htc_headset_microp.h>
#endif

#include <mach/tpa2016d2.h>
#include "devices.h" 
#include "clock-wince.h"
#include <linux/microp-keypad.h>
#include <linux/microp-ng.h>
//#include <mach/board_htc.h>

#include "proc_comm_wince.h"
//#include "devices.h"
//#include "htc_hw.h"
//#include "gpio_chip.h"
#include "board-htcrhodium.h"
#include <linux/usb/composite.h>
#include <linux/usb/android_composite.h>
#include <linux/platform_data/msm_serial_hs.h>
#if defined(CONFIG_SERIAL_BCM_BT_LPM)
#include <mach/bcm_bt_lpm.h>
#include <linux/serial_core.h>
#endif

#include <mach/htc_acoustic_wince.h>

extern int htcrhodium_init_mmc(void);
int hot_boot = 1;
EXPORT_SYMBOL(hot_boot);
/******************************************************************************
 * MicroP Keypad
 ******************************************************************************/
static int htcrhodium_microp_keymap[] = {
	KEY_B, //KEY_RESERVED, // invalid
	KEY_BACK, //Back key
	KEY_Q,
	KEY_HOME, //Mute button on back of device
	KEY_CAMERA,
	KEY_A,
	KEY_F,
	KEY_S,
	KEY_D,
	KEY_SEND, //Send Key
	// 10
	KEY_MENU, //Windows Key
	KEY_S, //KEY_RESERVED, // 0x0b   //Unknown
	KEY_I,
	KEY_K,
	KEY_J,
	KEY_H,
	KEY_G,
	KEY_A,
	KEY_4,
	KEY_1, //KEY_RESERVED, // 0x13  //Unknown
	// 20
	KEY_2, //KEY_RESERVED, // 0x14	//Unknown
	KEY_L,
	KEY_I,
	KEY_P,
	KEY_O,
	KEY_B,
	KEY_9,
	KEY_8,
	KEY_N,
	KEY_ENTER,
	// 30
	KEY_M,
	KEY_C,
	KEY_V,
	KEY_0,
	KEY_U,
	KEY_E,
	KEY_R,
	KEY_Q,
	KEY_T,
	KEY_Y,
	// 40
	KEY_W,
	KEY_UP,  //ARROW KEY
	KEY_1,
	KEY_2,
	KEY_DOWN, //KEY_LEFT,	//KEY_DOWN,   //ITS KEY DOWN FOR SURE!!!! STILL DOESN't GO DOWN 
	KEY_3,
	KEY_4,
	KEY_5,
	KEY_LEFT, 	//KEY_LEFT,	//ARROW KEY
	KEY_6,
	// 50
	KEY_2,			//KEY_RESERVED, // 0x32 //Unknown
	KEY_SPACE,
	KEY_BACKSPACE,
	KEY_7,
	KEY_RIGHT,		//KEY_UNKNOWN,  // ARROW KEY
	KEY_SPACE, //UNKNOWN
	KEY_X,	//KEY_COMMA, //UNKNOWN
	KEY_EMAIL,
	KEY_DOT,
	KEY_FN,  //Doesn't do anything?
	// 60
	KEY_LEFTSHIFT,
	KEY_Z,
	KEY_X,
	KEY_COMMA,
	KEY_COMPOSE, //Brings up search box?
	KEY_C, //KEY_SLASH,  //Unknown
	KEY_COMMA,
	KEY_6, 			//Unknown
	KEY_8,			//Unknown
	KEY_1, //KEY_RESERVED, // 0x45	//Unknown
	// 70
	KEY_2, //KEY_RESERVED, // 0x46	//Unknown
	KEY_P, //KEY_EMAIL,	//Unknown
};

static int htcrhodium_init_microp_keypad(struct device *dev)
{
	int ret;

	printk(KERN_DEBUG "%s\n", __func__);

	ret = gpio_request(RHODIUM_KPD_IRQ, "Keyboard");
	if (ret)
		return ret;
	ret = gpio_direction_input(RHODIUM_KPD_IRQ);
	if (ret)
		gpio_free(RHODIUM_KPD_IRQ);

	return ret;
}

static void htcrhodium_exit_microp_keypad(struct device *dev) {
	printk(KERN_DEBUG "%s\n", __func__);

	gpio_free(RHODIUM_KPD_IRQ);
}

static struct microp_keypad_platform_data htcrhodium_keypad_data = {
	.read_modifiers = true,
	.gpio_clamshell = -1,
	.init = htcrhodium_init_microp_keypad,
	.exit = htcrhodium_exit_microp_keypad,
	.irq_keypress = MSM_GPIO_TO_INT(RHODIUM_KPD_IRQ),
	.keypad_scancodes = htcrhodium_microp_keymap,
	.keypad_scancodes_size = ARRAY_SIZE(htcrhodium_microp_keymap),
};

static struct platform_device htcrhodium_keypad = {
	.name = "microp-keypad",
	.id = -1,
	.dev = {
		.platform_data = &htcrhodium_keypad_data,
	},
};

#if 0
static struct resource msm_serial0_resources[] = {
	{
		.start	= INT_UART1,
		.end	= INT_UART1,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= MSM_UART1_PHYS,
		.end	= MSM_UART1_PHYS + MSM_UART1_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};
#endif

#if 0
static struct platform_device msm_serial0_device = {
	.name	= "msm_serial",
	.id	= 0,
	.num_resources	= ARRAY_SIZE(msm_serial0_resources),
	.resource	= msm_serial0_resources,
};
#endif

/******************************************************************************
* Bluetooth
******************************************************************************/
#ifdef CONFIG_SERIAL_MSM_HS
static struct msm_serial_hs_platform_data msm_uart_dm2_pdata = {
	.rx_wakeup_irq = MSM_GPIO_TO_INT(MSM7200A_UART2DM_RX),
	.inject_rx_on_wakeup = 0,
# if defined(CONFIG_SERIAL_BCM_BT_LPM)
	.exit_lpm_cb = bcm_bt_lpm_exit_lpm_locked,
# endif
};

#if defined(CONFIG_SERIAL_BCM_BT_LPM)
static struct bcm_bt_lpm_platform_data bcm_bt_lpm_pdata = {
	.gpio_wake = RHODIUM_BT_WAKE,
	.gpio_host_wake = RHODIUM_BT_HOST_WAKE,
	.request_clock_off_locked = msm_hs_request_clock_off_locked,
	.request_clock_on_locked = msm_hs_request_clock_on_locked,
};

struct platform_device bcm_bt_lpm_device = {
	.name = "bcm_bt_lpm",
	.id = 0,
	.dev = {
		.platform_data = &bcm_bt_lpm_pdata,
	},
};
# endif
#endif

static struct msm_gpio bt_init_gpio_table_rhodium[] = {
	{	.gpio_cfg = GPIO_CFG(RHODIUM_BT_nRST, 0,
			GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
		.label = "RHODIUM_BT_nRST" },
	{	.gpio_cfg = GPIO_CFG(RHODIUM_BT_SHUTDOWN_N, 0,
			GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
		.label = "RHODIUM_BT_SHUTDOWN_N" },
	{	.gpio_cfg = GPIO_CFG(RHODIUM_BT_HOST_WAKE, 0,
			GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
		.label = "RHODIUM_BT_HOST_WAKE" },
	{	.gpio_cfg = GPIO_CFG(RHODIUM_BT_WAKE, 0,
			GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
		.label = "RHODIUM_BT_WAKE" },
};

static int htcrhodium_bt_power(void *data, bool blocked)
{
	printk(KERN_DEBUG "%s(%s)\n", __func__, blocked ? "off" : "on");

	if (!blocked) {
		gpio_direction_output(RHODIUM_BT_nRST, 1);
		msleep(150);
		gpio_direction_output(RHODIUM_BT_SHUTDOWN_N, 1);
	} else {
		gpio_direction_output(RHODIUM_BT_SHUTDOWN_N, 0);
		msleep(150);
		gpio_direction_output(RHODIUM_BT_nRST, 0);
	}
	return 0;
}

static int htcrhodium_bt_init(struct platform_device *pdev)
{
	int rc;

	printk(KERN_DEBUG "%s\n", __func__);

	rc = msm_gpios_request(bt_init_gpio_table_rhodium,
		ARRAY_SIZE(bt_init_gpio_table_rhodium));
	if (rc)
		goto fail_setup_gpio;

	msm_gpios_disable(bt_init_gpio_table_rhodium,
		ARRAY_SIZE(bt_init_gpio_table_rhodium));

	return 0;

fail_setup_gpio:
	printk(KERN_ERR "%s: failed with %d\n", __func__, rc);
	return rc;
}

static void htcrhodium_bt_exit(struct platform_device *pdev) {
	printk(KERN_DEBUG "%s\n", __func__);

	msm_gpios_disable_free(bt_init_gpio_table_rhodium,
		ARRAY_SIZE(bt_init_gpio_table_rhodium));
}

#define BDADDR_STR_SIZE 18
static char bdaddr[BDADDR_STR_SIZE];

static const char* htcrhodium_get_btaddr(void) {
	unsigned char *b = (unsigned char *)(MSM_SPL_BASE + 0x81C34);
	memset(bdaddr, 0, sizeof(bdaddr));

	snprintf(bdaddr, BDADDR_STR_SIZE, "%02X:%02X:%02X:%02X:%02X:%02X",
		b[5], b[4], b[3], b[2], b[1], b[0]);

	return bdaddr;
}

static struct msm7200a_rfkill_pdata htcrhodium_rfkill_data = {
	.init = htcrhodium_bt_init,
	.exit = htcrhodium_bt_exit,
	.set_power = htcrhodium_bt_power,
	.get_bdaddr = htcrhodium_get_btaddr,
	.uart_number = 2,
	.rfkill_name = "bcm4325",
	.configure_bt_pcm = 1,
};

static struct platform_device htcrhodium_rfkill = {
	.name = "msm7200a_rfkill",
	.id = -1,
	.dev = {
		.platform_data = &htcrhodium_rfkill_data,
	},
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
	printk("%s: %s\n", __FILE__, __func__);
	gpio_set_value(RHODIUM_USBPHY_RST, 1); 
	mdelay(3);
	gpio_set_value(RHODIUM_USBPHY_RST, 0);
	mdelay(3);
}

int usb_phy_reset(void  __iomem *regs)
{
	printk("%s: %s\n", __FILE__, __func__);
	usb_phy_shutdown();
	gpio_set_value(RHODIUM_USBPHY_RST, 0); 
	mdelay(3);
	gpio_set_value(RHODIUM_USBPHY_RST, 1);
	mdelay(3);

	return 0;
}

static struct resource resources_otg[] = {
	{
		.start	= MSM_HSUSB_PHYS,
		.end	= MSM_HSUSB_PHYS + MSM_HSUSB_SIZE,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_USB_HS,
		.end	= INT_USB_HS,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_otg = {
	.name		= "msm_otg",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resources_otg),
	.resource	= resources_otg,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

static void hsusb_setup_gpio(enum usb_switch_control mode){
	switch(mode){
	case USB_SWITCH_PERIPHERAL:
	case USB_SWITCH_HOST:
		usb_config_gpio(1);
		break;
	case USB_SWITCH_DISABLE:
		usb_config_gpio(0);
		break;
	default:
		printk(KERN_WARNING "%s: FIXME! value for mode? %u ?\n", __func__, mode);
	}
}

static void hsusb_chg_connected(enum chg_type chg_type){
	printk("Rhodium: Charger (dis)connected, type == %x\n", chg_type);
	switch(chg_type){
	case USB_CHG_TYPE__SDP:
		notify_usb_connected(1);
		break;
	case USB_CHG_TYPE__CARKIT:
	case USB_CHG_TYPE__WALLCHARGER:
		notify_usb_connected(2);
		break;
	case USB_CHG_TYPE__INVALID:
		notify_usb_connected(0);
		break;
	default:
		printk(KERN_WARNING "%s: FIXME! value for chg_type? %u ?\n", __func__, chg_type);
	}
}

static struct msm_otg_platform_data msm_otg_pdata = {
 	.phy_reset = usb_phy_reset,
         .pemp_level              = PRE_EMPHASIS_WITH_20_PERCENT,
         .cdr_autoreset           = CDR_AUTO_RESET_DISABLE,
         .drv_ampl                = HS_DRV_AMPLITUDE_DEFAULT,
         .se1_gating              = SE1_GATING_DISABLE,
         .chg_connected           = hsusb_chg_connected,
         .setup_gpio              = hsusb_setup_gpio,
};

static struct msm_hsusb_gadget_platform_data msm_gadget_pdata = {
	.is_phy_status_timer_on = 1,
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

/******************************************************************************
 * END USB
 ******************************************************************************/

static struct tpa2016d2_platform_data tpa2016d2_data = {
    .gpio_tpa2016_spk_en = RHODIUM_SPKR_PWR,
};


static struct platform_device htcrhodium_microp_leds = {
	.id = -1,
	.name = "htcrhodium-microp-leds",
};

static struct platform_device htcrhodium_microp_audio = {
	.id = -1,
	.name = "htcrhodium-microp-audio",
};


#ifdef CONFIG_HTC_HEADSET
static struct htc_headset_microp_platform_data htc_headset_microp_data = {
	.hpin_mask		= {0x00, 0x00, 0x01},	/* READ_GPI_STATE_HPIN */
	.hpin_int		= 0xFF,		/* IRQ_HEADSETIN */
	.hpin_irq		= MSM_GPIO_TO_INT(RHODIUM_GSENSOR_MOT),
};

static struct platform_device htcrhodium_microp_35mm = {
	.id = -1,
	.name = "HTC_HEADSET_MICROP",
	.dev = {
		.platform_data = &htc_headset_microp_data,
	},
};
#endif

/*When such time comes to migrate to htc_ls_microp as lightsensor driver

static struct platform_device htcrhodium_microp_ls = {
	.id = -1,
	.name = "lightsensor_microp",
};
*/
static struct platform_device* htcrhodium_microp_clients[] = {
	&htcrhodium_microp_leds,
	&htcrhodium_microp_audio,
//	&htcrhodium_microp_ls,
	// this must be last; dropped when variant is not RHODW(RHOD400/RHOD500)
#ifdef CONFIG_HTC_HEADSET
	&htcrhodium_microp_35mm,
#endif
};

static uint16_t micropklt_compatible_versions[] = {
	RHOD_MICROP_KLT_VERSION
};

static struct microp_platform_data htcrhodium_microp_led_audio_pdata = {
	.version_reg = RHOD_MICROP_KLT_VERSION_REG,
	.clients = htcrhodium_microp_clients,
	.nclients = ARRAY_SIZE(htcrhodium_microp_clients),
	.comp_versions = micropklt_compatible_versions,
	.n_comp_versions = ARRAY_SIZE(micropklt_compatible_versions),
};
/******************************************************************************
 * G-Sensor (Bosch BMA150)
 ******************************************************************************/
// TODO: GPIO and/or VREG setup on suspend/resume?
static int htcrhodium_bma150_power_on(void)
{
	printk(KERN_DEBUG "%s\n", __func__);
	return 0;
}

static void htcrhodium_bma150_power_off(void)
{
	printk(KERN_DEBUG "%s\n", __func__);
}

static struct bma150_platform_data htcrhodium_bma150_pdata = {
	.power_on = htcrhodium_bma150_power_on,
	.power_off = htcrhodium_bma150_power_off,
};

/******************************************************************************/
static struct i2c_board_info i2c_devices[] = {
	{
		I2C_BOARD_INFO("bma150", 0x70>>1),
		.platform_data = &htcrhodium_bma150_pdata,
	},
	{
		// LED & Backlight controller
		I2C_BOARD_INFO("microp-ng", 0x66),
		.platform_data = &htcrhodium_microp_led_audio_pdata
	},
	{		
		I2C_BOARD_INFO("mt9t013", 0x6c),
		/* .irq = TROUT_GPIO_TO_INT(TROUT_GPIO_CAM_BTN_STEP1_N), */
	},
	{		
		I2C_BOARD_INFO("tpa2016d2", 0xb0>>1),
		.platform_data = &tpa2016d2_data,
	},
//	{		
//		I2C_BOARD_INFO("audience_A1010", 0xf4>>1),
//	},
	{		
		I2C_BOARD_INFO("adc3001", 0x30>>1),
	},
};

#define SND(num, desc) { .name = desc, .id = num }
static struct snd_endpoint snd_endpoints_list[] = {
	SND(0, "HANDSET"),
	SND(1, "SPEAKER"),
	SND(2, "HEADSET"),
	SND(2, "NO_MIC_HEADSET"),
	SND(3, "BT"),
	SND(3, "BT_EC_OFF"),
	SND(13, "SPEAKER_MIC"),
	SND(0x11, "IDLE"),
	SND(256, "CURRENT"),
};
#undef SND

static struct msm_snd_endpoints htcrhodium_snd_endpoints = {
        .endpoints = snd_endpoints_list,
        .num = ARRAY_SIZE(snd_endpoints_list),
};

static struct platform_device htcrhodium_snd = {
	.name = "msm_snd",
	.id = -1,
	.dev	= {
		.platform_data = &htcrhodium_snd_endpoints,
	},
};

/* Proximity Sensor (Capella_CM3502)*/
static int capella_cm3602_power(int pwr_device, uint8_t enable)
{
	int rc = 0;
	if (pwr_device != PS_PWR_ON) return 0;
	/* TODO eolsen Add Voltage reg control */

	if ((get_machine_variant_type() < MACHINE_VARIANT_RHOD_4XX)) {
		rc = gpio_direction_output(RHODIUM_GPIO_PROXIMITY_EN_GSM, enable);
	} else {
		rc = gpio_direction_output(RHODIUM_GPIO_PROXIMITY_EN, enable);
	}

	return rc;
}

static struct capella_cm3602_platform_data capella_cm3602_pdata = {
	.power = capella_cm3602_power,
	.p_en  = RHODIUM_GPIO_PROXIMITY_EN,
	.p_out = RHODIUM_GPIO_PROXIMITY_INT_N
};

static struct capella_cm3602_platform_data capella_cm3602_pdata_gsm = {
	.power = capella_cm3602_power,
	.p_en  = RHODIUM_GPIO_PROXIMITY_EN_GSM,
	.p_out = RHODIUM_GPIO_PROXIMITY_INT_N
};

static struct platform_device capella_cm3602 = {
	.name = CAPELLA_CM3602,
	.id = -1,
	.dev = {
		.platform_data = &capella_cm3602_pdata
	}
};
/* End Proximity Sensor (Capella_CM3502)*/

static struct platform_device touchscreen = {
	.name		= "tssc-manager",
	.id		= -1,
};

#ifdef CONFIG_HTC_HEADSET

static void h2w_config_cpld(int route);
static void h2w_init_cpld(void);
static struct h2w_platform_data rhodium_h2w_data = {
	.cable_in1		= RHODIUM_CABLE_IN1,
	.cable_in2		= RHODIUM_CABLE_IN2,
	.h2w_clk		= RHODIUM_H2W_CLK,
	.h2w_data		= RHODIUM_H2W_DATA,
	.debug_uart 		= H2W_UART3,
	.config_cpld 		= h2w_config_cpld,
	.init_cpld 		= h2w_init_cpld,
	.headset_mic_35mm	= 17,
};

static void h2w_config_cpld(int route)
{
	switch (route) {
	case H2W_UART3:
		gpio_set_value(RHODIUM_H2W_UART_MUX, 1);
		break;
	case H2W_GPIO:
		gpio_set_value(RHODIUM_H2W_UART_MUX, 0);
		break;
	}
}

static void h2w_init_cpld(void)
{
	h2w_config_cpld(H2W_UART3);
	gpio_set_value(rhodium_h2w_data.h2w_data, 0);
	gpio_set_value(rhodium_h2w_data.h2w_clk, 0);
}

static struct platform_device rhodium_h2w = {
	.name		= "h2w",
	.id		= -1,
	.dev		= {
		.platform_data	= &rhodium_h2w_data,
	},
};
#endif

static struct msm_ts_platform_data htcrhodium_ts_pdata = {
	.min_x		= 296,
	.max_x		= 3800,
	.min_y		= 296,
	.max_y		= 3600, // leave room for zoombar (was 3800)
	.min_press	= 0,
	.max_press	= 256,
	.inv_x		= 4096,
	.inv_y		= 4096,
};

#if defined(CONFIG_MSM_CAMERA) && defined(CONFIG_MT9T013)

static unsigned camera_off_gpio_table[] = {
    /* CAMERA */
	GPIO_CFG(0, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), /* DAT0 */
	GPIO_CFG(1, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), /* DAT1 */
	GPIO_CFG(2, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), /* DAT2 */
	GPIO_CFG(3, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), /* DAT3 */
	GPIO_CFG(4, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), /* DAT4 */
	GPIO_CFG(5, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), /* DAT5 */
	GPIO_CFG(6, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), /* DAT6 */
	GPIO_CFG(7, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), /* DAT7 */
	GPIO_CFG(8, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), /* DAT8 */
	GPIO_CFG(9, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), /* DAT9 */
	GPIO_CFG(10, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), /* DAT10 */
	GPIO_CFG(11, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), /* DAT11 */
	GPIO_CFG(12, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), /* PCLK */
	GPIO_CFG(13, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), /* HSYNC_IN */
	GPIO_CFG(14, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), /* VSYNC_IN */
	GPIO_CFG(15, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA), /* MCLK */
};

static unsigned camera_on_gpio_table[] = {
    /* CAMERA */
	GPIO_CFG(0, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT0 */
	GPIO_CFG(1, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT1 */
	GPIO_CFG(2, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT2 */
	GPIO_CFG(3, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT3 */
	GPIO_CFG(4, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT4 */
	GPIO_CFG(5, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT5 */
	GPIO_CFG(6, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT6 */
	GPIO_CFG(7, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT7 */
	GPIO_CFG(8, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT8 */
	GPIO_CFG(9, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT9 */
	GPIO_CFG(10, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT10 */
	GPIO_CFG(11, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT11 */
	GPIO_CFG(12, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* PCLK */
	GPIO_CFG(13, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* HSYNC_IN */
	GPIO_CFG(14, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* VSYNC_IN */
	GPIO_CFG(15, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA), /* MCLK */
};

static void cam_gpio_init(void)
{
	if (gpio_request(0, "cam_data_0"))
		pr_err("failed to request gpio cam_data_0\n");
	if (gpio_request(1, "cam_data_1"))
		pr_err("failed to request gpio cam_data_1\n");
	if (gpio_request(2, "cam_data_2"))
		pr_err("failed to request gpio cam_data_2\n");
	if (gpio_request(3, "cam_data_3"))
		pr_err("failed to request gpio cam_data_3\n");
	if (gpio_request(4, "cam_data_4"))
		pr_err("failed to request gpio cam_data_4\n");
	if (gpio_request(5, "cam_data_5"))
		pr_err("failed to request gpio cam_data_5\n");
	if (gpio_request(6, "cam_data_6"))
		pr_err("failed to request gpio cam_data_6\n");
	if (gpio_request(7, "cam_data_7"))
		pr_err("failed to request gpio cam_data_7\n");
	if (gpio_request(8, "cam_data_8"))
		pr_err("failed to request gpio cam_data_8\n");
	if (gpio_request(9, "cam_data_9"))
		pr_err("failed to request gpio cam_data_9\n");
	if (gpio_request(10, "cam_data_10"))
		pr_err("failed to request gpio cam_data_10\n");
	if (gpio_request(11, "cam_data_11"))
		pr_err("failed to request gpio cam_data_11\n");
	if (gpio_request(12, "cam_pclk"))
		pr_err("failed to request gpio cam_pclk\n");
	if (gpio_request(13, "cam_hsync"))
		pr_err("failed to request gpio cam_hsync\n");
	if (gpio_request(14, "cam_vsync"))
		pr_err("failed to request gpio cam_vsync\n");
	if (gpio_request(15, "cam_mclk"))
		pr_err("failed to request gpio cam_mclk\n");
	if(gpio_request(RHODIUM_CAM_PWR1, "cam_pwr1"))
		pr_err("failed to request gpio cam_pwr1\n");
	if(gpio_request(107, "cam_unkwn"))
		pr_err("failed to request gpio cam_uknwn\n");
//	if(gpio_request(RHODIUM_MT9T013_RST, "cam_mtrst"))
//		pr_err("failed to request gpio cam_mtrst\n");
	if(gpio_request(RHODIUM_VGACAM_RST, "cam_vgarst"))
		pr_err("failed to request gpio cam_vgarst\n");
	if(gpio_request(RHODIUM_VCM_PDP, "cam_vcm"))
		pr_err("failed to request gpio cam_vcm\n");
}


static void config_gpio_table(unsigned *table, int len)
{
        int n;
        unsigned id;
        for(n = 0; n < len; n++) {
                id = table[n];
                gpio_tlmm_config(id,GPIO_CFG_ENABLE);
        }
}

static void config_camera_on_gpios(int bFrontCamera)
{
	struct msm_dex_command dex = { .cmd = PCOM_PMIC_REG_ON, .has_data=1 };

    /* VCM VDDD 1.8V */
	dex.data=0x8000;
	msm_proc_comm_wince(&dex,0);
    mdelay(20);
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

    config_gpio_table(camera_on_gpio_table,
            ARRAY_SIZE(camera_on_gpio_table));

    if ( (get_machine_variant_type() == MACHINE_VARIANT_RHOD_4XX) ||
         (get_machine_variant_type() == MACHINE_VARIANT_RHOD_5XX) ) {
        /* GPIO 26 is used to power up sensor */
        printk("%s CDMA\n", __func__);
        gpio_tlmm_config( GPIO_CFG(RHODIUM_CAM_PWR1_CDMA, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), 0 );
	gpio_direction_output(RHODIUM_CAM_PWR1_CDMA,0);
    } else {
        /* GPIO 30 is used to select sensor (1 == Front sensor, 0 = Back sensor) */
        printk("%s NON-CDMA\n", __func__);
        gpio_tlmm_config(GPIO_CFG(RHODIUM_CAM_PWR1     , 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), bFrontCamera);
	gpio_direction_output(RHODIUM_CAM_PWR1, bFrontCamera);
    }
    gpio_tlmm_config( GPIO_CFG(107                 , 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), 0 );  // Unknown function
    gpio_tlmm_config( GPIO_CFG(RHODIUM_MT9T013_RST , 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), 0 );  // Used in DevPowerUpMainSensor (reset pin)
    gpio_tlmm_config( GPIO_CFG(RHODIUM_VGACAM_RST  , 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), 0 );  // Used in DevPowerUpVideoSensor (reset pin)
    gpio_tlmm_config( GPIO_CFG(RHODIUM_VCM_PDP     , 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), 0 );  // Used in DevPullVCMPDPin
}

static void config_camera_off_gpios(void)
{
    struct msm_dex_command dex = { .cmd = PCOM_PMIC_REG_OFF, .has_data=1 };

    /* VCM VDDD 1.8V */
	dex.data=0x8000;
	msm_proc_comm_wince(&dex,0);
    mdelay(20);
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

    config_gpio_table(camera_off_gpio_table,
            ARRAY_SIZE(camera_off_gpio_table));

    if ( (get_machine_variant_type() == MACHINE_VARIANT_RHOD_4XX) ||
         (get_machine_variant_type() == MACHINE_VARIANT_RHOD_5XX) ) {
        /* Power off camera sensor */
        gpio_direction_output(RHODIUM_CAM_PWR1_CDMA, 1);
    } else {
        /* Reset camera select gpio to main (back) camera */
        gpio_direction_output(RHODIUM_CAM_PWR1, 1);
    }
}

#ifdef CONFIG_MT9T013
static void config_camera_on_gpios_mt9t013(void)
{
    config_camera_on_gpios(0);
}
#endif

// Will be used by front sensor
#if 0
static void config_camera_on_gpios_mt9v113(void)
{
    config_camera_on_gpios(1);
}
#endif

static struct msm_camera_device_platform_data msm_camera_mt9t013_device_data = {
        .camera_gpio_on  = config_camera_on_gpios_mt9t013,
        .camera_gpio_off = config_camera_off_gpios,
        .ioext.mdcphy = MSM_MDC_PHYS,
        .ioext.mdcsz  = MSM_MDC_SIZE,
        .ioext.appphy = MSM_CLK_CTL_PHYS,
        .ioext.appsz  = MSM_CLK_CTL_SIZE,
};

#ifdef CONFIG_MT9T013
static struct msm_camera_sensor_info msm_camera_sensor_mt9t013_data = {
        .sensor_name    = "mt9t013",
        .sensor_reset   = RHODIUM_MT9T013_RST,
//        .sensor_pwd     = RHODIUM_CAM_PWR1, // Only used for ov8810 & ov9665 sensors
        .vcm_pwd        = RHODIUM_VCM_PDP,
        .vcm_reversed_polarity = 1,
        .pdata          = &msm_camera_mt9t013_device_data,
};

static struct platform_device msm_camera_sensor_mt9t013 = {
        .name           = "msm_camera_mt9t013",
        .dev            = {
                .platform_data = &msm_camera_sensor_mt9t013_data,
        },
};

#endif
#endif

static struct gpio_keys_button rhodium_button_table[] = {
        /*KEY           GPIO                    ACTIVE_LOW      DESCRIPTION             type    wakeup  debounce*/
        {KEY_END,       RHODIUM_END_KEY,        1,              "End",                  EV_KEY, 1,      0},
        {KEY_VOLUMEUP,  RHODIUM_VOLUMEUP_KEY,   1,              "Volume Up",            EV_KEY, 0,      0},
        {KEY_VOLUMEDOWN,RHODIUM_VOLUMEDOWN_KEY, 1,              "Volume Down",          EV_KEY, 0,      0},
        {KEY_POWER,     RHODIUM_POWER_KEY,      1,              "Power button",         EV_KEY, 1,      0},
};

static struct gpio_keys_platform_data gpio_keys_data = {
	.buttons = rhodium_button_table,
	.nbuttons=ARRAY_SIZE(rhodium_button_table), 
};

static struct platform_device gpio_keys = {
	.name = "gpio-keys",
	.dev  = {
		.platform_data = &gpio_keys_data,
	},
	.id   = -1,
};

static struct platform_device msm_device_rtc = {
	.name = "msm_rtc",
	.id = -1,
};


static struct platform_device msm_device_htc_battery = {
        .name = "htc_battery",
        .id = -1,
};

static struct platform_device htc_hw = {
	.name = "htc_hw",
	.id = -1,
};

/***********************************************************************
 * LEGACY USB (not needed for new drivers, but used in GB compat layer
 ***********************************************************************/
static char *usb_functions_rndis[] = {
	"rndis",
};

static char *usb_functions_rndis_adb[] = {
	"rndis",
	"adb",
};

static char *usb_functions_ums[] = {
	"usb_mass_storage",
};

static char *usb_functions_ums_adb[] = {
	"usb_mass_storage",
	"adb",
};

static char *usb_functions_all[] = {
	"rndis",
	"usb_mass_storage",
	"adb",
};


static struct android_usb_product usb_products[] = {

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
	{
		.product_id	= 0x0ffe,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis),
		.functions	= usb_functions_rndis,
	},
	{
		.product_id	= 0x0ffc,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_adb),
		.functions	= usb_functions_rndis_adb,
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
/**********************************************************************/

static struct platform_device *devices[] __initdata = {
	&msm_device_smd,
	&msm_device_nand,
	&msm_device_i2c,
	&msm_device_rtc,
	&htc_hw,
#ifdef CONFIG_MSM_SMEM_BATTCHG
//	&msm_device_htc_battery_smem,
#endif
#ifdef CONFIG_MT9T013
	&msm_camera_sensor_mt9t013,
#endif
	&htcrhodium_snd,
	&htcrhodium_keypad,
	&touchscreen,
	&gpio_keys,
	&htcrhodium_rfkill,
	&msm_device_otg,
	&msm_device_gadget_peripheral,
	&msm_device_htc_battery,
	&capella_cm3602,
#ifdef CONFIG_SERIAL_MSM_HS
	&msm_device_uart_dm2,
#endif
#ifdef CONFIG_HTC_HEADSET
	&rhodium_h2w,
#endif
#ifdef CONFIG_USB_G_ANDROID
	&rndis_device,
	&usb_mass_storage_device,
	&android_usb_device,
#endif
#ifdef CONFIG_SERIAL_BCM_BT_LPM
	&bcm_bt_lpm_device,
#endif
	&acoustic_device,
};
extern struct sys_timer msm_timer;

static void __init htcrhodium_init_irq(void)
{
	msm_init_irq();
}

static struct msm_acpu_clock_platform_data halibut_clock_data = {
	.acpu_switch_time_us = 50,
	.max_speed_delta_khz = 256000,
	.vdd_switch_time_us = 62,
	.power_collapse_khz = 19200,
	.wait_for_irq_khz = 19200,
	.max_axi_khz = 160000,
};

void msm_serial_debug_init(unsigned int base, int irq, 
			   const char *clkname, int signal_irq);

/*
static htc_hw_pdata_t msm_htc_hw_pdata = {
	.set_vibrate = blac_set_vibrate,
	.battery_smem_offset = 0xfc110,
	.battery_smem_field_size = 2,
};
*/

static smem_batt_t msm_battery_pdata = {
	.gpio_battery_detect = RHODIUM_BAT_IRQ,
	.gpio_charger_enable = RHODIUM_CHARGE_EN_N,
	.gpio_charger_current_select = RHODIUM_USB_AC_PWR,
	//.gpio_ac_detect = RHODIUM_AC_DETECT,
	.smem_offset = 0xfc110,
	.smem_field_size = 2,
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

extern int init_mmc(void);

extern void htcrhodium_set_headset_amp(bool enable);

static struct htc_acoustic_wce_board_data htcrhodium_acoustic_data = {
	.set_headset_amp = htcrhodium_set_headset_amp,
    .set_speaker_amp = tpa2016d2_set_power,
};

static void __init htcrhodium_init(void)
{
	int i;
	struct htc_acoustic_wce_amss_data *acoustic_pdata;

	ram_console_resource[0].start = 0x8e0000;
	ram_console_resource[0].end = 0x8e0000 + 0x20000 - 1;
	platform_device_register(&ram_console_device);

	msm_acpu_clock_init(&halibut_clock_data);
	msm_proc_comm_wince_init();

//	msm_device_htc_hw.dev.platform_data = &msm_htc_hw_pdata;
	msm_device_htc_battery.dev.platform_data = &msm_battery_pdata;
	msm_device_touchscreen.dev.platform_data = &htcrhodium_ts_pdata;

	//msm_device_hsusb.dev.platform_data = &msm_hsusb_pdata;
	msm_device_otg.dev.platform_data = &msm_otg_pdata;
	msm_device_gadget_peripheral.dev.platform_data = &msm_gadget_pdata;
	
	usb_gpio_init();
#if defined(CONFIG_MSM_CAMERA) && defined(CONFIG_MT9T013)
	cam_gpio_init();
#endif
//	usb_config_gpio(1);
//	usb_phy_reset();
	

	gpio_request(87, "kpd power");
	gpio_tlmm_config(GPIO_CFG(87, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA),
					      GPIO_CFG_ENABLE);
	gpio_set_value(87, 1);

#ifdef CONFIG_SERIAL_MSM_HS
	msm_device_uart_dm2.dev.platform_data = &msm_uart_dm2_pdata;
#endif

	// Set acoustic device specific parameters
	acoustic_device.dev.platform_data = &amss_6120_acoustic_data;
	acoustic_pdata = acoustic_device.dev.platform_data;
	acoustic_pdata->mic_bias_callback = NULL;
	htc_acoustic_wce_board_data = &htcrhodium_acoustic_data;

	if ( (get_machine_variant_type() < MACHINE_VARIANT_RHOD_4XX) ) {
		/* Don't register htc_headset_microp for non 35mm variants. */
		htcrhodium_microp_led_audio_pdata.nclients =
			ARRAY_SIZE(htcrhodium_microp_clients) - 1;
		capella_cm3602.dev.platform_data = &capella_cm3602_pdata_gsm;
	}

	platform_add_devices(devices, ARRAY_SIZE(devices));
	i2c_register_board_info(0, i2c_devices, ARRAY_SIZE(i2c_devices));

//	msm_add_usb_devices(&htcrhodium_hsusb_board_pdata);

	init_mmc();

	msm_init_pmic_vibrator();

	/* A little vibrating welcome */
	for (i = 0; i < 2; i++) {
		msm_proc_comm_wince_vibrate(1);
		mdelay(150);
		msm_proc_comm_wince_vibrate(0);
		mdelay(75);
	}
}

static void __init htcrhodium_map_io(void)
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

static void __init htcrhodium_init_early(void)
{
       arch_ioremap_caller = __msm_ioremap_caller;
}

static void __init htcrhodium_fixup(struct tag *tags, char **cmdline, struct meminfo *mi)
{
	mi->nr_banks = 2;
	mi->bank[0].start = PAGE_ALIGN(PHYS_OFFSET);
	mi->bank[0].size = 99*1024*1024;
	mi->bank[1].start = PAGE_ALIGN(PHYS_OFFSET + 0x10000000);
	mi->bank[1].size = 128*1024*1024-50*1024*1024; // see pmem.c for the value

	printk(KERN_INFO "%s: nr_banks = %d\n", __func__, mi->nr_banks);
	printk(KERN_INFO "%s: bank0 start=%08lx, size=%08lx\n", __func__,
		(long unsigned int)mi->bank[0].start, mi->bank[0].size);
	printk(KERN_INFO "%s: bank1 start=%08lx, size=%08lx\n", __func__,
		(long unsigned int)mi->bank[1].start, mi->bank[1].size);
}

static int __init setup_hot_boot(char *str)
{
	get_option(&str, &hot_boot);
	return 0;
}

early_param("hot_boot", setup_hot_boot);

MACHINE_START(HTCRHODIUM, "HTC Rhodium cellphone")
	.fixup = htcrhodium_fixup,
	.atag_offset = 0x100,
	.map_io = htcrhodium_map_io,
	.init_early     = htcrhodium_init_early,
	.init_irq = htcrhodium_init_irq,
	.init_machine = htcrhodium_init,
	.timer = &msm_timer,
MACHINE_END
