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
#endif
#include "devices.h" 
#include "clock-wince.h"
#include <linux/microp-keypad.h>
//#include <mach/board_htc.h>

#include "proc_comm_wince.h"
//#include "devices.h"
//#include "htc_hw.h"
//#include "gpio_chip.h"
#include "board-htctopaz.h"
#include <linux/usb/composite.h>
#include <linux/usb/android_composite.h>
#include <linux/platform_data/msm_serial_hs.h>
#if defined(CONFIG_SERIAL_BCM_BT_LPM)
#include <mach/bcm_bt_lpm.h>
#include <linux/serial_core.h>
#endif

#include <mach/htc_acoustic_wince.h>

extern int htctopaz_init_mmc(void);


#ifdef CONFIG_SERIAL_MSM_HS
static struct msm_serial_hs_platform_data msm_uart_dm2_pdata = {
	.wakeup_irq = MSM_GPIO_TO_INT(21), //TODO: 21 (UART2dm_rx) on 27 but looks like BT_HOST_WAKE to me 
	.inject_rx_on_wakeup = 0,
#if defined(CONFIG_SERIAL_BCM_BT_LPM)
	.exit_lpm_cb = bcm_bt_lpm_exit_lpm_locked,
#endif
};

#if defined(CONFIG_SERIAL_BCM_BT_LPM)
static struct bcm_bt_lpm_platform_data bcm_bt_lpm_pdata = {
	.gpio_wake = TOPA100_BT_WAKE,
	.gpio_host_wake = TOPA100_BT_HOST_WAKE,
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
#endif
#endif

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

static int usb_phy_init_seq_rhod[] = {
        0x40, 0x31, /* Leave this pair out for USB Host Mode */
        0x1D, 0x0D,
        0x1D, 0x10,
	0x5, 0xA,
        -1
};

static void usb_phy_shutdown(void)
{
	printk("%s: %s\n", __FILE__, __func__);
	gpio_set_value(TOPA100_USBPHY_RST, 1); 
	gpio_set_value(TOPA100_USBPHY_RST, 0);
}

static int usb_phy_reset(void  __iomem *regs)
{
	printk("%s: %s\n", __FILE__, __func__);
	usb_phy_shutdown();
	gpio_set_value(TOPA100_USBPHY_RST, 0); 
	mdelay(3);
	gpio_set_value(TOPA100_USBPHY_RST, 1);
	mdelay(3);
	usb_config_gpio(1);

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

static struct platform_device msm_device_otg = {
	.name		= "msm_otg",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resources_otg),
	.resource	= resources_otg,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

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

static struct msm_hsusb_gadget_platform_data msm_gadget_pdata = {
	.phy_init_seq = usb_phy_init_seq_rhod,
	.phy_reset = usb_phy_reset, 
         .is_phy_status_timer_on = 1,
//	.self_powered
};

static struct platform_device raphael_rfkill = {
	.name = "htcraphael_rfkill",
	.id = -1,
};

static struct i2c_board_info i2c_devices[] = {
/*	{
		//gsensor 0x38
		I2C_BOARD_INFO("bma150", 0x70>>1),
	},*/
	{
		// LED & Backlight controller
		I2C_BOARD_INFO("microp-klt", 0x66),
	},
/*	{
		// Keyboard controller for RHOD
		I2C_BOARD_INFO("microp-ksc", 0x67),
	},*/
	{		
		I2C_BOARD_INFO("mt9p012", 0x6c),
		/* .irq = TROUT_GPIO_TO_INT(TROUT_GPIO_CAM_BTN_STEP1_N), */
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


#if 0
static struct platform_device rhod_prox = {
    .name       = "rhodium_proximity",
};
#endif

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
	if(gpio_request(TOPA100_CAM_PWR1, "cam_pwr1"))
		pr_err("failed to request gpio cam_pwr1\n");
	if(gpio_request(107, "cam_unkwn"))
		pr_err("failed to request gpio cam_uknwn\n");
//	if(gpio_request(RHODIUM_MT9T013_RST, "cam_mtrst"))
//		pr_err("failed to request gpio cam_mtrst\n");
//	if(gpio_request(TOPA100_CAM1_RST, "cam_vgarst"))
//		pr_err("failed to request gpio cam_vgarst\n");
	if(gpio_request(TOPA100_CAM_VCMPDP, "cam_vcm"))
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

static void config_camera_on_gpios(int Gpio30Value)
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

   /* Select camera (front or back) */
    gpio_direction_output(TOPA100_CAM_PWR1, Gpio30Value);

    config_gpio_table(camera_on_gpio_table,
        ARRAY_SIZE(camera_on_gpio_table));
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

  /* Reset camera select gpio to main (back) camera */
    gpio_direction_output(TOPA100_CAM_PWR1, 1);

    config_gpio_table(camera_off_gpio_table,
            ARRAY_SIZE(camera_off_gpio_table));
}

#ifdef CONFIG_MT9P012
static void config_camera_on_gpios_mt9p012(void)
{
    config_camera_on_gpios(0);
}
#endif

 

static struct msm_camera_device_platform_data msm_camera_mt9p012_device_data = {
        .camera_gpio_on  = config_camera_on_gpios_mt9p012,
        .camera_gpio_off = config_camera_off_gpios,
        .ioext.mdcphy = MSM_MDC_PHYS,
        .ioext.mdcsz  = MSM_MDC_SIZE,
        .ioext.appphy = MSM_CLK_CTL_PHYS,
        .ioext.appsz  = MSM_CLK_CTL_SIZE,
};

#ifdef CONFIG_MT9P012
static struct msm_camera_sensor_info msm_camera_sensor_mt9p012_data = {
        .sensor_name    = "mt9p012",
        .sensor_reset   = TOPA100_CAM1_RST,
	.sensor_pwd     = TOPA100_CAM_PWR1,
        .vcm_pwd        = TOPA100_CAM_VCMPDP,
        .pdata          = &msm_camera_mt9p012_device_data,
};

static struct platform_device msm_camera_sensor_mt9p012 = {
        .name           = "msm_camera_mt9p012",
        .dev            = {
                .platform_data = &msm_camera_sensor_mt9p012_data,
        },
};

#endif
#endif


static struct gpio_keys_button rhodium_button_table[] = {
        /*KEY           GPIO                    ACTIVE_LOW      DESCRIPTION             type    wakeup  debounce*/
        {KEY_VOLUMEUP,  TOPA100_VOLUP_KEY,   1,              "Volume Up",            EV_KEY, 0,      0},
        {KEY_VOLUMEDOWN,TOPA100_VOLDOWN_KEY, 1,              "Volume Down",          EV_KEY, 0,      0},
        {KEY_POWER,     TOPA100_POWER_KEY,      1,              "Power button",         EV_KEY, 1,      0},
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

static struct platform_device msm_device_htc_battery = {
        .name = "htc_battery",
        .id = -1,
};

static struct platform_device htc_hw = {
	.name = "htc_hw",
	.id = -1,
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
#ifdef CONFIG_MSM_SMEM_BATTCHG
//	&msm_device_htc_battery_smem,
#endif
#ifdef CONFIG_MT9P012
	&msm_camera_sensor_mt9p012,
#endif
	&htcrhodium_snd,
	&touchscreen,
	&gpio_keys,
	&raphael_rfkill,
	&msm_device_otg,
	&msm_device_gadget_peripheral,
	&msm_device_htc_battery,
#ifdef CONFIG_SERIAL_MSM_HS
	&msm_device_uart_dm2,
#endif
	// &rhod_prox,	
#ifdef CONFIG_HTC_HEADSET
//	&rhodium_h2w,
#endif
#ifdef CONFIG_USB_ANDROID
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



static smem_batt_t msm_battery_pdata = {
	.gpio_battery_detect = TOPA100_BAT_IN,
	.gpio_charger_enable = TOPA100_CHARGE_EN_N,
	.gpio_charger_current_select = TOPA100_USB_AC_PWR,
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

static struct htc_acoustic_wce_board_data htcrhodium_acoustic_data = {
//	.set_headset_amp = htcrhodium_set_headset_amp,
//    .set_speaker_amp = tpa2016d2_set_power,
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
	cam_gpio_init();
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

static void __init htcrhodium_fixup(struct machine_desc *desc,
		struct tag *tags, char **cmdline, struct meminfo *mi)
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

MACHINE_START(HTCTOPAZ, "HTC Topaz cellphone")
	.fixup = htcrhodium_fixup,
	.boot_params = 0x10000100,
	.map_io = htcrhodium_map_io,
	.init_irq = htcrhodium_init_irq,
	.init_machine = htcrhodium_init,
	.timer = &msm_timer,
MACHINE_END
