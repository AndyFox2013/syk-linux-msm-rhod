/* linux/arch/arm/mach-msm/board-htcwhitestone-panel.c
 * Based on board-htctopaz-panel.c.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/leds.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/microp-klt.h>

#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/mach-types.h>

#include <mach/msm_fb.h>
#include <mach/vreg.h>
#include <linux/microp-klt.h>

#include "board-htcwhitestone.h"
#include "proc_comm_wince.h"
#include "devices.h"

#define REG_WAIT	(0xffff)

static struct nov_regs {
	unsigned reg;
	unsigned val;
} eid_es3_init_seq[] = {
	{0x5100, 0x00}, 
	{0x1100, 0x01},
	{REG_WAIT, 100},
	
	{0x4e00, 0},
	{0x180,	 2},
	{0x880,	 0},
	{0x2080, 0x33},
	{0x2680, 0x78},
	{0x2880, 0x3e},
	{0x2980,4},
	{0x2C80, 0x22},
	{0x2e80, 0},
	{0xde80, 2},
	{0x3a00, 0x55},
	{0x3500, 0},
	{0x4400, 0}, 
	{0x4401, 0},
	{0x5e00, 0},
	{0x6a01, 0},
	{0x6a02, 1},
	{0x5301, 0x10},
	{0x5500, 2},
	{0x6a17, 1},
	{0x6a18, 0xff},
	{0x2900, 1},
	{0x5300, 0x2c},
	{0x5e03, 0x1},

//	{0x1100, 0x00}, // EXIT_SLEEP_MODE ("Sleep Out")
	{REG_WAIT, 120}, // REG_WAIT (must wait 120ms after "Sleep Out")
};

static struct nov_regs eid_es3_deinit_seq[] = {
	{0x2800, 0x01}, 
	{0x5300, 0x28},
	{0x5500, 0},
	{0x5300, 0x8},
	{0x5e03, 0},
	{0x5300, 0},
	{0x1000, 1},	
};

//static struct clk *gp_clk;
static struct vreg *vreg_lcd_1;	/* LCD1 */
static struct vreg *vreg_lcd_2;	/* LCD2 */


static int htcwhitestone_mddi_client_init(
	struct msm_mddi_bridge_platform_data *bridge_data,
	struct msm_mddi_client_data *client_data)
{
	int i;
	unsigned reg, val;

	printk(KERN_DEBUG "%s\n", __func__);

	client_data->auto_hibernate(client_data, 0);

	for (i = 0; i < ARRAY_SIZE(eid_es3_init_seq); i++) {
		reg = cpu_to_le32(eid_es3_init_seq[i].reg);
		val = cpu_to_le32(eid_es3_init_seq[i].val);
		if (reg == REG_WAIT)
			msleep(val);
		else
			client_data->remote_write(client_data, val, reg);
	}

	client_data->auto_hibernate(client_data, 1);

	return 0;
}

static int htcwhitestone_mddi_client_uninit(
	struct msm_mddi_bridge_platform_data *bridge_data,
	struct msm_mddi_client_data *client_data)
{
	int i;
	unsigned reg, val;

	printk(KERN_DEBUG "%s\n", __func__);

	for (i = 0; i < ARRAY_SIZE(eid_es3_deinit_seq); i++) {
		reg = cpu_to_le32(eid_es3_deinit_seq[i].reg);
		val = cpu_to_le32(eid_es3_deinit_seq[i].val);
		if (reg == REG_WAIT)
			msleep(val);
		else
			client_data->remote_write(client_data, val, reg);
	}

	return 0;
}

static int htcwhitestone_mddi_panel_blank(
	struct msm_mddi_bridge_platform_data *bridge_data,
	struct msm_mddi_client_data *client_data)
{
	printk(KERN_DEBUG "%s\n", __func__);

	//~ micropklt_panel_suspend();

	return 0;
}

static int htcwhitestone_mddi_panel_unblank(
	struct msm_mddi_bridge_platform_data *bridge_data,
	struct msm_mddi_client_data *client_data)
{
	printk(KERN_DEBUG "%s\n", __func__);

	// SET_DISPLAY_ON
//	client_data->remote_write(client_data, 0x00, 0x2900);
	// WRCTRLD
//	client_data->remote_write(client_data, 0x2c, 0x5300);

	//~ micropklt_panel_resume();

	return 0;
}

static void htcwhitestone_mddi_power_client(
	struct msm_mddi_client_data *client_data,
	int on)
{
	struct msm_dex_command dex;

	printk(KERN_DEBUG "%s(%s)\n", __func__, on ? "on" : "off");

	if (on) {
		vreg_enable(vreg_lcd_1);
		vreg_enable(vreg_lcd_2);
		mdelay(50);

		gpio_set_value(WHIT100_LCD_RESET, 1);
		msleep(5);

		dex.cmd=PCOM_PMIC_REG_ON;
		dex.has_data=1;
		dex.data=0x80;
		msm_proc_comm_wince(&dex,0);
		msleep(5);

		dex.data=0x2000;
		msm_proc_comm_wince(&dex,0);
		msleep(5);

		gpio_set_value(WHIT100_LCD_RESET, 0);
		msleep(10);
		gpio_set_value(WHIT100_LCD_RESET, 1);
		msleep(10);

	} else {
		gpio_set_value(WHIT100_LCD_RESET, 0);

		msleep(10);

		dex.cmd=PCOM_PMIC_REG_OFF;
		dex.has_data=1;
		dex.data=0x2000;
		msm_proc_comm_wince(&dex,0);
		msleep(5);

		dex.data=0x80;
		msm_proc_comm_wince(&dex,0);
		msleep(5);

		msleep(5);

		vreg_disable(vreg_lcd_1);
		vreg_disable(vreg_lcd_2);
		mdelay(50);
	}
}

extern struct resource resources_msm_fb[];

static struct msm_mddi_bridge_platform_data novatec_client_data = {
	.init = htcwhitestone_mddi_client_init,
	.uninit = htcwhitestone_mddi_client_uninit,
	.blank = htcwhitestone_mddi_panel_blank,
	.unblank = htcwhitestone_mddi_panel_unblank,
	.fb_data = {
		.xres = 480,
		.yres = 800,
		.output_format = 0,
	},
#if 0
	.panel_conf = {
		.caps = MSMFB_CAP_CABC,
	},
#endif
};

static struct msm_mddi_platform_data mddi_pdata = {
	.vsync_irq = MSM_GPIO_TO_INT(WHIT100_LCD_VSYNC),
	.power_client = htcwhitestone_mddi_power_client,
	.fb_resource = resources_msm_fb,
	.num_clients = 2,
	.client_platform_data = {
		{
			// unconfirmed
			.product_id = (0xb9f6 << 16 | 0x5580),
			.name = "mddi_c_b9f6_5582",
			.id = 0,
			.client_data = &novatec_client_data,
			.clk_rate = 0,
		},
		{
			// unconfirmed
			.product_id = (0xb9f6 << 16 | 0x5582),
			.name = "mddi_c_b9f6_5582",
			.id = 0,
			.client_data = &novatec_client_data,
			.clk_rate = 0,
		}
	},
};

int __init htcwhitestone_init_panel(void)
{
	int rc, panel_id;

	if(!machine_is_htcwhitestone()) {
		printk(KERN_INFO "%s: panel does not apply to this device, aborted\n", __func__);
		return -1;
	}

        panel_id = readl(MSM_SPL_BASE+0x81034);
	printk("Panel type detected: %x\n",panel_id);

	printk(KERN_INFO "%s: Initializing panel\n", __func__);

#if 0
	gp_clk = clk_get(NULL, "gp_clk");
	if (IS_ERR(gp_clk)) {
		printk(KERN_ERR "%s: could not get gp clock\n", __func__);
		gp_clk = NULL;
	}
	rc = clk_set_rate(gp_clk, 19200000);
	if (rc)
	{
		printk(KERN_ERR "%s: set clock rate failed\n", __func__);
	}
#endif
	gpio_request(WHIT100_LCD_RESET, "panel reset");

	gpio_direction_output(WHIT100_LCD_RESET, 1);

	vreg_lcd_1 = vreg_get(0, "gp2");
	if (IS_ERR(vreg_lcd_1))
		return PTR_ERR(vreg_lcd_1);

	vreg_lcd_2 = vreg_get(0, "gp4");
	if (IS_ERR(vreg_lcd_2))
		return PTR_ERR(vreg_lcd_2);

	rc = gpio_request(WHIT100_LCD_VSYNC, "vsync");
	if (rc)
		return rc;
	rc = gpio_direction_input(WHIT100_LCD_VSYNC);
	if (rc)
		return rc;

	rc = platform_device_register(&msm_device_mdp);
	if (rc)
		return rc;
	msm_device_mddi0.dev.platform_data = &mddi_pdata;
	return platform_device_register(&msm_device_mddi0);
}

device_initcall(htcwhitestone_init_panel);
