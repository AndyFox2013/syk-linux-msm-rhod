/* linux/arch/arm/mach-msm/board-htcrhod-panel.c
 * Based on board-trout-panel.c by: Brian Swetland <swetland@google.com>
 * Remodelled based on board-supersonic-panel.c by: Jay Tu <jay_tu@htc.com>
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/leds.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/microp-klt.h>

#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/mach-types.h>

#include <mach/msm_fb.h>
#include <mach/vreg.h>
#include <linux/microp-klt.h>

#include "board-htctopaz.h"
#include "board-htcrhodium.h"
#include "proc_comm_wince.h"
#include "devices.h"

#define REG_WAIT (0xffff)

int panel_id;

static struct cabc_t {
	struct led_classdev lcd_backlight;
	struct msm_mddi_client_data *client_data;
	struct mutex lock;
	unsigned long status;
} cabc;

enum {
	GATE_ON = 1 << 0,
};

struct nov_regs {
	unsigned reg;
	unsigned val;
} nov_init_seq[] = {
	/* Auo Rhod100 */
	{0x1100, 0x01},
	{0x4e00, 0x00},
	{0x3a00, 0x05},
	{REG_WAIT, 0x1},	// delay needed only on AUO panels
	{0x3b00, 0x00},		// hsp/vsp high trigger
	{0x3b02, 0x00},		// vbp 1 clk
	{0x3b03, 0x00},		// vfp 1 clk
	{0x3b04, 0x00},		// hbp 1 clk
	{0x3b05, 0x00},		// hfp 1 clk
	{0x3500, 0x02},
	{0x4400, 0x00},
	{0x4401, 0x00},
	{0x5100, 0x40},
	{0x5e00, 0x00},
	{0x6a01, 0x00},
	{0x6a02, 0x01},
	{0x5301, 0x10},
	{0x5500, 0x02},
	{0x6a17, 0x01},
	{0x6a18, 0x40},
	{0x2900, 0x01},
	{0x5300, 0x2c},
	{0x5303, 0x01},
};

struct nov_regs nov_deinit_seq[] = {

	{0x2800, 0x01},		// display off
   	{0x5300, 0x28},
    	{0x5500, 0x00},
    	{0x5300, 0x08},
    	{0x5E03, 0x00},
    	{0x5300, 0x00},
	{0x1000, 0x01},		// sleep-in
};

struct nov_regs nov_init_seq1[] = {
	/* EID Rhod100,210,	// Table modified by WisTilt2
	       400,500 */
	{0x2900, 0x01},		// display on
	{0x1200, 0x01},		// partial mode
	{0x1100, 0x01},		// sleep-out
	{0x1300, 0x01},		// normal mode
	{0x3500, 0x00},		// tearing effect
	{0x3a00, 0x55},		// bits per pixel
	{0x3b00, 0x00},		// hsp/vsp high trigger
	{0x3b02, 0x01},		// vbp 1 clk
	{0x3b03, 0x01},		// vfp 1 clk
	{0x3b04, 0x01},		// hbp 1 clk
	{0x3b05, 0x01},		// hfp 1 clk
	{0x4405, 0x00},		// set tear line 1st param
	{0x4401, 0x00},		// set tear line 2nd param
	{0x4e00, 0x00},		// set spi/i2c mode
	{0x5100, 0x40},		// display brightness
	{0x5301, 0x10},		// led control pins
	{0x5302, 0x01},		// labc dimming on/off
	{0x5303, 0x01},		// labc rising dimming style
	{0x5304, 0x01},		// labc falling dimming style
	{0x5500, 0x02},		// cabc enable & image type
	{0x5e00, 0x04},		// cabc minimum brightness
	{0x5e03, 0x05},		// labc adc & hysteresis enable
	{0x5e04, 0x01},		// labc reference voltage 1.7v
	{0x6a01, 0x00},		// pwm duty/freq control
	{0x6a02, 0x01},		// pwm freq
	{0x6a17, 0x00},		// cabc pwm force off
	{0x6a18, 0x40},		// cabc pwm duty
	{0x6f00, 0x00},		// clear ls lsb
	{0x6f01, 0x00},		// clear ls msb
	{0x6f02, 0x00},		// force cabc pwm off
	{0x6f03, 0x00},		// force pwm duty
	{0x5300, 0x3c},		// set autobl bit during init
};

struct nov_regs nov_init_seq2[] = {
	/* EID Rhod300 */	// Table modified by WisTilt2
	{0x2900, 0x01},		// display on
	{0x1200, 0x01},		// partial mode
	{0x1100, 0x01},		// sleep-out
	{0x1300, 0x01},		// normal mode
	{0x3500, 0x00},		// tearing effect
	{0x3a00, 0x55},		// bits per pixel
	{0x3b00, 0x00},		// hsp/vsp high trigger
	{0x3b02, 0x01},		// vbp 1 clk
	{0x3b03, 0x01},		// vfp 1 clk
	{0x3b04, 0x01},		// hbp 1 clk
	{0x3b05, 0x01},		// hfp 1 clk
	{0x4405, 0x00},		// set tear line 1st param
	{0x4401, 0x00},		// set tear line 2nd param
	{0x4e00, 0x00},		// set spi/i2c mode
	{0x5100, 0x20},		// display brightness
	{0x5301, 0x10},		// led control pins
	{0x5302, 0x01},		// labc dimming on/off
	{0x5303, 0x01},		// labc rising dimming style
	{0x5304, 0x01},		// labc falling dimming style
	{0x5500, 0x02},		// cabc enable & image type
	{0x5e00, 0x04},		// cabc minimum brightness
	{0x5e03, 0x05},		// labc adc & hysteresis enable
	{0x5e04, 0x01},		// labc reference voltage 1.7v
	{0x6a01, 0x00},		// pwm duty/freq control
	{0x6a02, 0x01},		// pwm freq
	{0x6a17, 0x00},		// cabc pwm force off
	{0x6a18, 0xff},		// cabc pwm duty
	{0x6f00, 0x00},		// clear ls lsb
	{0x6f01, 0x00},		// clear ls msb
	{0x6f02, 0x00},		// force cabc pwm off
	{0x6f03, 0x00},		// force pwm duty
	{0x5300, 0x3c},		// set autobl bit during init
};


//static struct clk *gp_clk;
static struct vreg *vreg_lcd_1;	/* LCD1 */
static struct vreg *vreg_lcd_2;	/* LCD2 */

static int htcrhod_mddi_client_init(
	struct msm_mddi_bridge_platform_data *bridge_data,
	struct msm_mddi_client_data *client_data)
{
	int i;
	unsigned reg, val;

	switch (panel_id)	//Added by WisTilt2 - Panel detect
	{
	case	0x14:	/* EID - Rhod100,210,400,500 */

		client_data->auto_hibernate(client_data, 0);
		for (i = 0; i < ARRAY_SIZE(nov_init_seq1); i++)
		{
			reg = cpu_to_le32(nov_init_seq1[i].reg);
			val = cpu_to_le32(nov_init_seq1[i].val);
			if (reg == REG_WAIT)
				mdelay(val);
			else
				client_data->remote_write(client_data, val, reg);
		}
		break;

	case	0x15:	/* EID - Rhod300 */

		client_data->auto_hibernate(client_data, 0);
		for (i = 0; i < ARRAY_SIZE(nov_init_seq2); i++)
		{
			reg = cpu_to_le32(nov_init_seq2[i].reg);
			val = cpu_to_le32(nov_init_seq2[i].val);
			if (reg == REG_WAIT)
				mdelay(val);
			else
				client_data->remote_write(client_data, val, reg);
		}
		break;

	case	0x01:	/* AUO - Rhod100 */
	case	0x07:	/* AUO - Rhod100 */
	case	0x13:	/* AUO - Rhod100 */

		client_data->auto_hibernate(client_data, 0);
		for (i = 0; i < ARRAY_SIZE(nov_init_seq); i++)
		{
			reg = cpu_to_le32(nov_init_seq[i].reg);
			val = cpu_to_le32(nov_init_seq[i].val);
			if (reg == REG_WAIT)
				mdelay(val);
			else
				client_data->remote_write(client_data, val, reg);
		}
		break;

	default:
		printk(KERN_WARNING "%s: Unknown panel_id %x detected!\n", __func__, panel_id);
		client_data->auto_hibernate(client_data, 0);
		for (i = 0; i < ARRAY_SIZE(nov_init_seq1); i++)	//Default to EID on unknown
		{
			reg = cpu_to_le32(nov_init_seq1[i].reg);
			val = cpu_to_le32(nov_init_seq1[i].val);
			if (reg == REG_WAIT)
				mdelay(val);
			else
				client_data->remote_write(client_data, val, reg);
		}
	}
	client_data->auto_hibernate(client_data, 1);
	return 0;
}

static int htcrhod_mddi_client_uninit(
	struct msm_mddi_bridge_platform_data *bridge_data,
	struct msm_mddi_client_data *client_data)
{
	int i;
	unsigned reg, val;
	for (i = 0; i < ARRAY_SIZE(nov_deinit_seq); i++) {
		reg = cpu_to_le32(nov_deinit_seq[i].reg);
		val = cpu_to_le32(nov_deinit_seq[i].val);
		client_data->remote_write(client_data, val, reg);
		mdelay(5);
	}
	return 0;
}

static int htcrhod_mddi_panel_blank(
	struct msm_mddi_bridge_platform_data *bridge_data,
	struct msm_mddi_client_data *client_data)
{
	return 0;
}

static int htcrhod_mddi_panel_unblank(
	struct msm_mddi_bridge_platform_data *bridge_data,
	struct msm_mddi_client_data *client_data)
{
	client_data->remote_write(client_data, 0x01, 0x2900);	// display on
	client_data->remote_write(client_data, 0x2c, 0x5300);	// toggle autobl bit
	return 0;
}
static void htcrhod_mddi_power_client(
	struct msm_mddi_client_data *client_data,
	int on)
{
	printk(KERN_DEBUG "%s (%s)\n", __func__, on ? "on" : "off");

	if (on) {
		if (get_machine_variant_type() != MACHINE_VARIANT_RHOD_4XX
			&& get_machine_variant_type() != MACHINE_VARIANT_RHOD_5XX) {
			vreg_enable(vreg_lcd_1);
			vreg_enable(vreg_lcd_2);
			mdelay(5);
		}
	} else {
		if (get_machine_variant_type() != MACHINE_VARIANT_RHOD_4XX
			&& get_machine_variant_type() != MACHINE_VARIANT_RHOD_5XX) {
			vreg_disable(vreg_lcd_1);
			vreg_disable(vreg_lcd_2);
		}
	}
}

extern struct resource resources_msm_fb[];

static struct msm_mddi_bridge_platform_data novatec_client_data = {
	.init = htcrhod_mddi_client_init,
	.uninit = htcrhod_mddi_client_uninit,
	.blank = htcrhod_mddi_panel_blank,
	.unblank = htcrhod_mddi_panel_unblank,
	.fb_data = {
		.xres = 480,
		.yres = 800,
		.output_format = 0,
	},
};

static struct msm_mddi_platform_data mddi_pdata = {
	.vsync_irq = MSM_GPIO_TO_INT(RHOD_LCD_VSYNC),
	.power_client = htcrhod_mddi_power_client,
	.fb_resource = resources_msm_fb,
	.num_clients = 2,
	.client_platform_data = {
		{
			// rhod+topa
			.product_id = (0xb9f6 << 16 | 0x5580),
			.name = "mddi_c_b9f6_5582",
			.id = 0,
			.client_data = &novatec_client_data,
			.clk_rate = 0,
		},
		{
			// rhod+topa
			.product_id = (0xb9f6 << 16 | 0x5582),
			.name = "mddi_c_b9f6_5582",
			.id = 0,
			.client_data = &novatec_client_data,
			.clk_rate = 0,
		}
	},
};

static void suc_set_brightness(struct led_classdev *led_cdev,
				enum led_brightness val)
{
	struct msm_mddi_client_data *client = cabc.client_data;
	unsigned int shrink_br = val;

	printk(KERN_DEBUG "set brightness = %d\n", val);
	if (test_bit(GATE_ON, &cabc.status) == 0)
		return;

	if (val < 30)
		shrink_br = 5;
	else if ((val >= 30) && (val <= 143))
		shrink_br = 104 * (val - 30) / 113 + 5;
	else
		shrink_br = 145 * (val - 144) / 111 + 110;
	mutex_lock(&cabc.lock);
	client->remote_write(client, 0, 0x5500);
	client->remote_write(client, shrink_br, 0x5100);	
	mutex_unlock(&cabc.lock);
}

static enum led_brightness
suc_get_brightness(struct led_classdev *led_cdev)
{
	struct msm_mddi_client_data *client = cabc.client_data;
	return client->remote_read(client, 0x5100);
}

#define DEFAULT_BRIGHTNESS 100
static void suc_backlight_switch(int on)
{
	enum led_brightness val;
	if (on) {
		printk(KERN_DEBUG "turn on backlight\n");
		set_bit(GATE_ON, &cabc.status);
		val = cabc.lcd_backlight.brightness;

		/* LED core uses get_brightness for default value
 		If the physical layer is not ready, we should
		not count on it */
		if (val == 0)
			val = DEFAULT_BRIGHTNESS;
		suc_set_brightness(&cabc.lcd_backlight, val);
	} else {
		clear_bit(GATE_ON, &cabc.status);
		suc_set_brightness(&cabc.lcd_backlight, 0);
	}
}
static unsigned mddi_read_addr;
static ssize_t mddi_remote_read_get(struct device *dev,
	struct device_attribute *attr, char *ret_buf)
{
	struct msm_mddi_client_data *client = cabc.client_data;
	return sprintf(ret_buf,"0x%x\n",client->remote_read(client, mddi_read_addr));
}

static ssize_t mddi_remote_read_set(struct device *dev,
	struct device_attribute *attr, const char *in_buf, size_t count)
{
	mddi_read_addr = simple_strtoul(in_buf, NULL, 16);
	printk(KERN_DEBUG "%s: setting mddi_read_addr = %x", __func__, mddi_read_addr);

	return count;
}

static DEVICE_ATTR(remote_read, 0666, mddi_remote_read_get, mddi_remote_read_set);

static int suc_backlight_probe(struct platform_device *pdev)
{
	int err = -EIO;
	printk(KERN_DEBUG "%s", __func__);
	mutex_init(&cabc.lock);
	cabc.client_data = pdev->dev.platform_data;
	cabc.lcd_backlight.name = "suc-backlight";
	cabc.lcd_backlight.brightness_set = suc_set_brightness;
	cabc.lcd_backlight.brightness = 0;
	err = led_classdev_register(&pdev->dev, &cabc.lcd_backlight);
	if (err)
		goto err_register_lcd_bl;
	err = device_create_file(cabc.lcd_backlight.dev, &dev_attr_remote_read);
	if (err < 0) {
		printk(KERN_ERR "%s: Failed registering suc_backlight attribute (%d)",
			__func__, err);
		goto err_register_lcd_bl;
	}

	printk(KERN_DEBUG "%s successful\n", __func__);	
	return 0;

err_register_lcd_bl:
	led_classdev_unregister(&cabc.lcd_backlight);
	return err;
}

static struct platform_driver suc_backlight_driver = {
	.probe = suc_backlight_probe,
	.driver = {
		.owner = THIS_MODULE,
	},
};

int __init htcrhod_init_panel(void)
{
	int rc;

	if(!machine_is_htcrhodium()) {
		printk(KERN_INFO "%s: panel does not apply to this device, aborted\n", __func__);
		return 0;
	}

        panel_id = readl(MSM_SPL_BASE+0x81034);
	printk("Panel type detected: %x\n",panel_id);

	printk(KERN_INFO "%s: Initializing panel\n", __func__);

	rc = platform_device_register(&msm_device_mdp);
	if (rc)
		return rc;

	vreg_lcd_1 = vreg_get(0, "gp2");
	if (IS_ERR(vreg_lcd_1))
		return PTR_ERR(vreg_lcd_1);

	vreg_lcd_2 = vreg_get(0, "gp4");
	if (IS_ERR(vreg_lcd_2))
		return PTR_ERR(vreg_lcd_2);

	rc = gpio_request(RHOD_LCD_VSYNC, "vsync");
	if (rc)
		return rc;
	rc = gpio_direction_input(RHOD_LCD_VSYNC);
	if (rc)
		return rc;

	suc_backlight_driver.driver.name = "nov_cabc";

	rc = platform_driver_register(&suc_backlight_driver);
	if (rc)
		return rc;

	msm_device_mddi0.dev.platform_data = &mddi_pdata;
	return platform_device_register(&msm_device_mddi0);
}
device_initcall(htcrhod_init_panel);
