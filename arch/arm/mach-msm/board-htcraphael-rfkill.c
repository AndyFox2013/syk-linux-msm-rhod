/*
 * Copyright (C) 2008 Google, Inc.
 * Author: Nick Pelly <npelly@google.com>
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

/* Control bluetooth power for trout platform */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <asm/gpio.h>
#include <mach/vreg.h>
#include <linux/err.h>
#include <asm/mach-types.h>

#include "board-htcraphael.h"
//#include "board-htckovsky.h"
#include "board-htcrhodium.h"
#include "proc_comm_wince.h" /* ?? GPIO ?? */

/* Kovsky */
#define KOVS_GPIO_BT_POWER	32
#define KOVS_GPIO_BT_ROUTER	63

#include <linux/rfkill.h>

static struct rfkill *bt_rfk;
static const char bt_name[] = "brf6300";
static struct vreg *vreg_bt;

/* for BRCM BCM4325 modules */
#define BDADDR_STR_SIZE 18

static char bdaddr[BDADDR_STR_SIZE];

module_param_string(bdaddr, bdaddr, sizeof(bdaddr), 0644);
MODULE_PARM_DESC(bdaddr, "bluetooth address");

static int read_bdaddr(void)
{
    unsigned char *b = (unsigned char *)(MSM_SPL_BASE+0x81C34);
    memset(bdaddr, 0, sizeof(bdaddr));

    snprintf(bdaddr, BDADDR_STR_SIZE, "%02X:%02X:%02X:%02X:%02X:%02X",
        b[5], b[4], b[3], b[2], b[1], b[0]);

    return 0;
}



/* ---- COMMON ---- */
static void config_gpio_table(unsigned *table, int len)
{
	int n;
	unsigned id;
	for(n = 0; n < len; n++) {
		id = table[n];
		gpio_tlmm_config(id, 0);
	}
}

static unsigned bt_on_gpio_table_raph100[] = {
    GPIO_CFG(RAPH100_UART2DM_RTS,  4, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), /* RTS */
    GPIO_CFG(RAPH100_UART2DM_CTS,  4, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_8MA), /* CTS */
    GPIO_CFG(RAPH100_UART2DM_RX,   4, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_8MA), /* RX */ 
    GPIO_CFG(RAPH100_UART2DM_TX,   2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), /* TX */
};
static unsigned bt_off_gpio_table_raph100[] = {
    GPIO_CFG(RAPH100_UART2DM_RTS,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), /* RTS */
    GPIO_CFG(RAPH100_UART2DM_CTS,  0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_UP, GPIO_CFG_8MA), /* CTS */
    GPIO_CFG(RAPH100_UART2DM_RX,   0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_UP, GPIO_CFG_8MA), /* RX */ 
    GPIO_CFG(RAPH100_UART2DM_TX,   0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), /* TX */
};

/* Rhodium gpios */
static unsigned bt_init_gpio_table_rhodium[] = {
    GPIO_CFG(RHODIUM_BT_nRST,       0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
    GPIO_CFG(RHODIUM_BT_SHUTDOWN_N, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
    GPIO_CFG(RHODIUM_BT_HOST_WAKE,  0, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
    GPIO_CFG(RHODIUM_BT_WAKE,       0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
};

static unsigned bt_pcm_on_gpio_table_rhodium[] = {
    GPIO_CFG(RHODIUM_PCM_DOUT,      1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* PCM_DOUT */
    GPIO_CFG(RHODIUM_PCM_DIN,       1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),  /* PCM_DIN */
    GPIO_CFG(RHODIUM_PCM_SYNC,      2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* PCM_SYNC */
    GPIO_CFG(RHODIUM_PCM_CLK,       2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* PCM_CLK */
};

static unsigned bt_pcm_off_gpio_table_rhodium[] = {
    GPIO_CFG(RHODIUM_PCM_DOUT,      1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* PCM_DOUT */
    GPIO_CFG(RHODIUM_PCM_DIN,       1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),  /* PCM_DIN */
    GPIO_CFG(RHODIUM_PCM_SYNC,      2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* PCM_SYNC */
    GPIO_CFG(RHODIUM_PCM_CLK,       2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* PCM_CLK */
};
/* End of Rhodium gpios */

static int bluetooth_set_power(void *data, bool blocked)
{
//	int rc;
	if (!blocked) {
		printk("   bluetooth rfkill state ON\n");
		config_gpio_table(bt_on_gpio_table_raph100,ARRAY_SIZE(bt_on_gpio_table_raph100));

		switch(__machine_arch_type){
		case MACH_TYPE_HTCRHODIUM:
			config_gpio_table(bt_pcm_on_gpio_table_rhodium,ARRAY_SIZE(bt_pcm_on_gpio_table_rhodium));
			gpio_direction_output(RHODIUM_BT_nRST,1);
			msleep(150);
			gpio_direction_output(RHODIUM_BT_SHUTDOWN_N,1);
		break;
		default:
			printk("RFKILL: unknown device!\n");
		}
	}
	else{
		printk("   bluetooth rfkill state   OFF\n");
		config_gpio_table(bt_off_gpio_table_raph100,ARRAY_SIZE(bt_off_gpio_table_raph100));
		switch(__machine_arch_type) {
		case MACH_TYPE_HTCRHODIUM:
			config_gpio_table(bt_pcm_off_gpio_table_rhodium,ARRAY_SIZE(bt_pcm_off_gpio_table_rhodium));
			gpio_direction_output(RHODIUM_BT_SHUTDOWN_N,0);
			msleep(150);
			gpio_direction_output(RHODIUM_BT_nRST,0);
		break;
		default:
			printk("RFKILL: unknown device!\n");
		}
	}
	return 0;
}

static struct rfkill_ops rfk_ops = {
	.set_block = bluetooth_set_power,
};

static int htcraphael_rfkill_probe(struct platform_device *pdev)
{
	int rc = 0;

	printk("BT RFK probe\n");

	if ( machine_is_htcrhodium() ) {
		/* Read BDADDR from smem */
		read_bdaddr();
		/* Initial GPIO setting for BCM4325 module */
		config_gpio_table(bt_init_gpio_table_rhodium,ARRAY_SIZE(bt_init_gpio_table_rhodium));
	}

	/* default to bluetooth off */
	bluetooth_set_power(NULL, RFKILL_USER_STATE_SOFT_BLOCKED);

	bt_rfk = rfkill_alloc(bt_name, &pdev->dev, RFKILL_TYPE_BLUETOOTH, &rfk_ops, NULL);
	if (!bt_rfk)
		return -ENOMEM;

	
	rfkill_init_sw_state(bt_rfk, true);

	/* userspace cannot take exclusive control */
#if 0
	bt_rfk->user_claim_unsupported = 1;
	bt_rfk->user_claim = 0;
	bt_rfk->toggle_radio = bluetooth_set_power;
#endif
	rc = rfkill_register(bt_rfk);

	if (rc)
		rfkill_destroy(bt_rfk);
	return rc;
}

static struct platform_driver htcraphael_rfkill_driver = {
	.probe = htcraphael_rfkill_probe,
	.driver = {
		.name = "htcraphael_rfkill",
		.owner = THIS_MODULE,
	},
};

static int htcraphael_rfkill_init(void)
{
	printk("BT RFK register\n");
	vreg_bt=vreg_get(0, "rftx");
	if(IS_ERR(vreg_bt))
		return PTR_ERR(vreg_bt);
	return platform_driver_register(&htcraphael_rfkill_driver);
}

module_init(htcraphael_rfkill_init);
MODULE_DESCRIPTION("htcraphael rfkill");
MODULE_AUTHOR("Nick Pelly <npelly@google.com>");
MODULE_LICENSE("GPL");

