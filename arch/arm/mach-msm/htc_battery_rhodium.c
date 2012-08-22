/* arch/arm/mach-msm/htc_battery_rhodium.c
 * by: Abel C. Laura <abel.laura@gmail.com> based on htc_battery.c
 *
 * Copyright (C) 2008 HTC Corporation.
 * Copyright (C) 2008 Google, Inc.
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/wakelock.h>
#include <asm/gpio.h>
#include <mach/board.h>
#include <asm/mach-types.h>
#include <mach/board_htc.h>
#include <linux/io.h>

#include "proc_comm_wince.h"
#include "board-htcrhodium.h"
#include <mach/htc_battery.h>
#include <mach/htc_battery_rhodium.h>

static struct wake_lock vbus_wake_lock;
static int bat_suspended = 0;
static int batt_vref = 0, batt_vref_half = 0;

#define TRACE_BATT 0

#if TRACE_BATT
#include <linux/rtc.h>

#define BATT(x...) do { \
struct timespec ts; \
struct rtc_time tm; \
getnstimeofday(&ts); \
rtc_time_to_tm(ts.tv_sec, &tm); \
printk(KERN_INFO "[BATT] " x); \
printk(" at %lld (%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n", \
ktime_to_ns(ktime_get()), tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, \
tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec); \
} while (0)
#else
#define BATT(x...) do {} while (0)
#endif



/* module debugger */
#define HTC_BATTERY_DEBUG		1
#define BATTERY_PREVENTION		1

/* Enable this will shut down if no battery */
#define ENABLE_BATTERY_DETECTION	0

#define GET_ADC_VREF        readl(MSM_SHARED_RAM_BASE + 0xFC0E0)
#define GET_ADC_0_5_VREF    readl(MSM_SHARED_RAM_BASE + 0xFC0E4)
#define GET_VBUS_STATUS		readl(MSM_SHARED_RAM_BASE + 0xFC00C)	
#define GPIO_BATTERY_DETECTION		RHODIUM_BAT_IRQ
#define GPIO_BATTERY_CHARGER_EN		RHODIUM_CHARGE_EN_N
#define GPIO_BATTERY_CHARGER_CURRENT	RHODIUM_USB_AC_PWR

typedef enum {
	DISABLE = 0,
	ENABLE_SLOW_CHG,
	ENABLE_FAST_CHG
} batt_ctl_t;

/* This order is the same as htc_power_supplies[]
 * And it's also the same as htc_cable_status_update()
 */
typedef enum {
	CHARGER_BATTERY = 0,
	CHARGER_USB,
	CHARGER_AC
} charger_type_t;

const char *charger_tags[] = {"none", "USB", "AC"};

struct battery_info_reply {
	u32 batt_id;		/* Battery ID from ADC */
	u32 batt_vol;		/* Battery voltage from ADC */
	u32 batt_temp;		/* Battery Temperature (C) from formula and ADC */
	u32 batt_current;	/* Battery current from ADC */
	u32 level;		/* formula */
	u32 charging_source;	/* 0: no cable, 1:usb, 2:AC */
	u32 charging_enabled;	/* 0: Disable, 1: Enable */
	u32 full_bat;		/* Full capacity of battery (mAh) */
};

struct htc_battery_info {
	int present;
	unsigned long update_time;

	/* lock to protect the battery info */
	struct mutex lock;

	struct battery_info_reply rep;
	smem_batt_t *resources;			
};
static struct work_struct htc_cable_notify_work;

static struct htc_battery_info htc_batt_info;

static unsigned int cache_time = 1000;

static int htc_battery_initial = 0;
static int htc_get_batt_info( struct battery_info_reply *buffer );
static int init_battery_settings( struct battery_info_reply *buffer ) {
	//int usb_id=0;
	struct msm_dex_command dex;

	dex.cmd = PCOM_GET_BATTERY_DATA;
	msm_proc_comm_wince(&dex, 0);

	/* TODO: Need to find out what to do with full_bat and half adc_ref */
	batt_vref = GET_ADC_VREF;
	batt_vref_half = GET_ADC_0_5_VREF;
	if ( buffer == NULL )
		return -EINVAL;

	if ( htc_get_batt_info( buffer ) < 0 )
		return -EINVAL;

	mutex_lock( &htc_batt_info.lock );

	buffer->full_bat = 1500000;

	mutex_unlock(&htc_batt_info.lock);	
	//usb_id = gpio_get_value(0x29);
	/* If we boot with USB cable in, then VBUS detection will set source to AC.
		We will let the usb driver correct us later. */
	if (!htc_battery_initial) {
		if(GET_VBUS_STATUS) {
			htc_batt_info.rep.charging_source = CHARGER_AC;
		} else {
			htc_batt_info.rep.charging_source = CHARGER_BATTERY;
		}
	}
	//BATT("GPIO 0x29 = %d", usb_id);
	BATT("Init: VREF=%d; HVREF=%d; full_bat=%d\n",
			batt_vref, batt_vref_half, buffer->full_bat);

	return 0;
}
static int get_battery_id(int raw_batt_id ) {
		/* TODO: Find out what to do with IDs. Right now we treat them all the same.*/
		if ((raw_batt_id > 0x368) && (raw_batt_id < 0x485)) {
			return 1;
		} else if ((raw_batt_id > 0x224) && (raw_batt_id <= 0x367)) {
			return 2;
		} else if ((raw_batt_id > 0x102) && (raw_batt_id <= 0x223)) {
			return 3;
		} else if ((raw_batt_id > 0x486) && (raw_batt_id <= 0x5dc)) {
			return 4;
		}
	return 0xFF;
}

static enum power_supply_property htc_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
};

static enum power_supply_property htc_power_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *supply_list[] = {
	"battery",
};

/* HTC dedicated attributes */
static ssize_t htc_battery_show_property(struct device *dev,
					  struct device_attribute *attr,
					  char *buf);

static int htc_power_get_property(struct power_supply *psy, 
				    enum power_supply_property psp,
				    union power_supply_propval *val);

static int htc_battery_get_property(struct power_supply *psy, 
				    enum power_supply_property psp,
				    union power_supply_propval *val);

static struct power_supply htc_power_supplies[] = {
	{
		.name = "battery",
		.type = POWER_SUPPLY_TYPE_BATTERY,
		.properties = htc_battery_properties,
		.num_properties = ARRAY_SIZE(htc_battery_properties),
		.get_property = htc_battery_get_property,
	},
	{
		.name = "usb",
		.type = POWER_SUPPLY_TYPE_USB,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = htc_power_properties,
		.num_properties = ARRAY_SIZE(htc_power_properties),
		.get_property = htc_power_get_property,
	},
	{
		.name = "ac",
		.type = POWER_SUPPLY_TYPE_MAINS,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = htc_power_properties,
		.num_properties = ARRAY_SIZE(htc_power_properties),
		.get_property = htc_power_get_property,
	},
};

static int g_usb_online;

/* -------------------------------------------------------------------------- */

#if defined(CONFIG_DEBUG_FS)
int htc_battery_set_charging(batt_ctl_t ctl);
static int batt_debug_set(void *data, u64 val)
{
	return htc_battery_set_charging((batt_ctl_t) val);
}

static int batt_debug_get(void *data, u64 *val)
{
	return -ENOSYS;
}

DEFINE_SIMPLE_ATTRIBUTE(batt_debug_fops, batt_debug_get, batt_debug_set, "%llu\n");
static int __init batt_debug_init(void)
{
	struct dentry *dent;

	dent = debugfs_create_dir("htc_battery", 0);
	if (IS_ERR(dent))
		return PTR_ERR(dent);

	debugfs_create_file("charger_state", 0644, dent, NULL, &batt_debug_fops);

	return 0;
}

device_initcall(batt_debug_init);
#endif

static int init_batt_gpio(void)
{

	if (gpio_request(GPIO_BATTERY_DETECTION, "batt_detect") < 0)
		goto gpio_failed;
	if (gpio_request(GPIO_BATTERY_CHARGER_EN, "charger_en") < 0)
		goto gpio_failed;
	if (gpio_request(GPIO_BATTERY_CHARGER_CURRENT, "charge_current") < 0)
		goto gpio_failed;

	return 0;

gpio_failed:	
	return -EINVAL;
	
}

/* 
 *	battery_charging_ctrl - battery charing control.
 * 	@ctl:			battery control command
 *
 */
static int battery_charging_ctrl(batt_ctl_t ctl)
{
	int result = 0;


	switch (ctl) {
	case DISABLE:
		BATT("charger OFF");
		/* 0 for enable; 1 disable */
		result = gpio_direction_output(GPIO_BATTERY_CHARGER_EN, 1);
		break;
	case ENABLE_SLOW_CHG:
		BATT("charger ON (SLOW)");
		result = gpio_direction_output(GPIO_BATTERY_CHARGER_CURRENT, 0);
		result = gpio_direction_output(GPIO_BATTERY_CHARGER_EN, 0);
		break;
	case ENABLE_FAST_CHG:
		BATT("charger ON (FAST)");
		result = gpio_direction_output(GPIO_BATTERY_CHARGER_CURRENT, 1);
		result = gpio_direction_output(GPIO_BATTERY_CHARGER_EN, 0);
		break;
	default:
		printk(KERN_ERR "Not supported battery ctr called.!\n");
		result = -EINVAL;
		break;
	}
	
	return result;
}

int htc_battery_set_charging(batt_ctl_t ctl)
{
	int rc;
	
	if ((rc = battery_charging_ctrl(ctl)) < 0)
		goto result;
	
	if (!htc_battery_initial) {
		htc_batt_info.rep.charging_enabled = ctl & 0x3;
	} else {
		mutex_lock(&htc_batt_info.lock);
		htc_batt_info.rep.charging_enabled = ctl & 0x3;
		mutex_unlock(&htc_batt_info.lock);
	}
result:	
	return rc;
}

int htc_battery_status_update(u32 curr_level)
{
	int notify;
	if (!htc_battery_initial)
		return 0;

	mutex_lock(&htc_batt_info.lock);
	notify = (htc_batt_info.rep.level != curr_level);
	htc_batt_info.rep.level = curr_level;
	mutex_unlock(&htc_batt_info.lock);

	if (notify)
		power_supply_changed(&htc_power_supplies[CHARGER_BATTERY]);
	return 0;
}

int htc_cable_status_update(int status)
{
	int rc = 0;
	unsigned last_source;

	if (!htc_battery_initial)
		return 0;
	
	if (status < CHARGER_BATTERY || status > CHARGER_AC) {
		BATT("%s: Not supported cable status received!", __func__);
		return -EINVAL;
	}
	mutex_lock(&htc_batt_info.lock);
	/* A9 reports USB charging when helf AC cable in and China AC charger. */
	/* Work arround: notify userspace AC charging first,
	and notify USB charging again when receiving usb connected notificaiton from usb driver. */
	last_source = htc_batt_info.rep.charging_source;
	if (status == CHARGER_USB && g_usb_online == 0)
		htc_batt_info.rep.charging_source = CHARGER_AC;
	else {
		htc_batt_info.rep.charging_source  = status;
		/* usb driver will not notify usb offline. */
		if (status == CHARGER_BATTERY && g_usb_online == 1)
			g_usb_online = 0;
	}

	/* TODO: Don't call usb driver again with the same cable status. */
	msm_hsusb_set_vbus_state(status == CHARGER_USB);

	if (htc_batt_info.rep.charging_source != last_source) {
		if (htc_batt_info.rep.charging_source == CHARGER_USB ||
			htc_batt_info.rep.charging_source == CHARGER_AC) {
		wake_lock(&vbus_wake_lock);
	} else {
		/* give userspace some time to see the uevent and update
		 * LED state or whatnot...
		 */
		wake_lock_timeout(&vbus_wake_lock, HZ / 2);
	}
		if (htc_batt_info.rep.charging_source == CHARGER_BATTERY || last_source == CHARGER_BATTERY)
	power_supply_changed(&htc_power_supplies[CHARGER_BATTERY]);
		if (htc_batt_info.rep.charging_source == CHARGER_USB || last_source == CHARGER_USB)
	power_supply_changed(&htc_power_supplies[CHARGER_USB]);
		if (htc_batt_info.rep.charging_source == CHARGER_AC || last_source == CHARGER_AC)
	power_supply_changed(&htc_power_supplies[CHARGER_AC]);
	}
	mutex_unlock(&htc_batt_info.lock);

	return rc;
}

/* A9 reports USB charging when helf AC cable in and China AC charger. */
/* Work arround: notify userspace AC charging first,
and notify USB charging again when receiving usb connected notification from usb driver. */
void notify_usb_connected(int online)
{
	BATT("%s: online=%d, g_usb_online=%d", __func__, online, g_usb_online);

	if (g_usb_online != online) {
		g_usb_online = online;
		schedule_work(&htc_cable_notify_work);
	}
}
static void usb_status_notifier_func(int online)
{
	mutex_lock(&htc_batt_info.lock);
	if (online && htc_batt_info.rep.charging_source == CHARGER_AC) {
		mutex_unlock(&htc_batt_info.lock);
		htc_cable_status_update(CHARGER_USB);
		mutex_lock(&htc_batt_info.lock);
	} else if (online) {
		BATT("warning: usb connected but charging source=%d", htc_batt_info.rep.charging_source);
	}
	mutex_unlock(&htc_batt_info.lock);
}

static int htc_get_batt_smem_info(struct battery_info_reply *buffer)
{
	signed int new_level=0;
	signed int old_level=0;
	signed int charge_delta=0;
	int charge=0;
	int filter=0;
	short *smem_values = NULL;
	struct msm_dex_command dex;

	dex.cmd = PCOM_GET_BATTERY_DATA;
	msm_proc_comm_wince(&dex, 0);

	smem_values = (short *)(MSM_SHARED_RAM_BASE + htc_batt_info.resources->smem_offset);

	BATT("raw_id=%d, raw_temp=%d, raw_volt=%d, raw_charge=%d, raw_discharge=%d",
	smem_values[0], smem_values[1], smem_values[2], smem_values[3], smem_values[4]);

	buffer->batt_id = get_battery_id(smem_values[0]);
	buffer->batt_temp = temp_lut[smem_values[1]];
	buffer->batt_vol = ((smem_values[2]) * 1371) / batt_vref;

	if (buffer->charging_source == CHARGER_USB)
		charge = (smem_values[3] * 306)/batt_vref;
	else
		charge = (smem_values[3] * 293)/batt_vref;

	buffer->batt_current = (smem_values[4] * 811) / batt_vref;

	if (!htc_battery_initial) {
		/* Since our algo requires a previous value. Lets take a quick estimate. 
			This is not accurate by any means, but at least it will let the correction
			algo take less steps to an accurate reading. If it's below the critical minimum
			the statically set to level 5.*/
		if (buffer->batt_vol > BATT_VOLTAGE_EMPTY) 
			buffer->level = (100 * (buffer->batt_vol - BATT_VOLTAGE_EMPTY))/(BATT_VOLTAGE_FULL - BATT_VOLTAGE_EMPTY);
		else 
			buffer->level = 5;
		BATT("Initial Level %d", buffer->level);
	}
	old_level = buffer->level;

	if (buffer->batt_current > charge) {
		charge_delta = buffer->batt_vol + ((2 * (buffer->batt_current - charge))/10);		
	} else if (buffer->batt_current < charge) {
		charge_delta = buffer->batt_vol - ((2 * (charge - buffer->batt_current))/10);
	} else if (buffer->batt_current == charge) {
		charge_delta = 0;
	}

	if (charge_delta <= BATT_VOLTAGE_EMPTY)
		new_level = 0;
	else 
		new_level = (100 * (charge_delta - BATT_VOLTAGE_EMPTY))/(BATT_VOLTAGE_FULL - BATT_VOLTAGE_EMPTY);

	if (new_level > old_level) {
		filter = new_level - old_level;
		if (filter < BATT_LVL_FILTER) 
			buffer->level = old_level + 1; 
		else
			buffer->level = old_level + filter/BATT_LVL_FILTER;
	} else 	if (new_level < old_level) {
		filter = old_level - new_level;
		if (filter < BATT_LVL_FILTER)
			buffer->level = old_level - 1; 
		else
			buffer->level = old_level - filter/BATT_LVL_FILTER; 
	} else if (new_level == old_level) {
		buffer->level = old_level;
	}

	BATT("Chg En %d: Chg src %d:Old Lvl %d, New Lvl %d Level %d Corr_Vol %d",buffer->charging_enabled, buffer->charging_source, old_level, new_level,buffer->level,charge_delta);
	if (buffer->batt_vol >= BATT_VOLTAGE_MAX) buffer->level = 100;
	if (buffer->level > 100) buffer->level = 100;
	if (buffer->level < 0) buffer->level = 0;

	if (buffer->level !=old_level)
		power_supply_changed(&htc_power_supplies[CHARGER_BATTERY]);

	//FIXME: Verify this is needed, and the correct place to do it
	//make sure that if we are discharging the current is negative
	if ( buffer->charging_source == CHARGER_BATTERY && buffer->batt_current > 0)
	    buffer->batt_current= 0 - buffer->batt_current;

	BATT("id=%d, level=%d, temp=%d, volt=%d, charge=%d, current=%d ",
	buffer->batt_id, buffer->level, buffer->batt_temp, buffer->batt_vol, charge, buffer->batt_current);
	return 0;
}

static int htc_get_batt_info(struct battery_info_reply *buffer)
{
	int chg_source;
	int chg_enabled;
	int current_voltage;

	// 3.X change to match previous battery driver, in .27 this update was called in proc_comm_wince.c
	// Note: usb notifications seem to be broken, we will never get to CHARGER_USB?
	htc_cable_status_update(GET_VBUS_STATUS);

	mutex_lock(&htc_batt_info.lock);
	htc_get_batt_smem_info(buffer);
	chg_source = buffer->charging_source;
	chg_enabled = buffer->charging_enabled;
	current_voltage = buffer->batt_vol;
	mutex_unlock(&htc_batt_info.lock);

	if (chg_source == CHARGER_BATTERY) {
		if (chg_enabled!=DISABLE) htc_battery_set_charging(DISABLE);
	} else {
			if ((buffer->level < 100) && (chg_enabled!=ENABLE_FAST_CHG))
				htc_battery_set_charging(ENABLE_FAST_CHG);
			if (buffer->level == 100) {
				if (current_voltage >= BATT_VOLTAGE_MAX) htc_battery_set_charging(ENABLE_SLOW_CHG);
			}
	}

	return 0;
}

/* -------------------------------------------------------------------------- */
static int htc_power_get_property(struct power_supply *psy, 
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	charger_type_t charger;
	
	mutex_lock(&htc_batt_info.lock);
	charger = htc_batt_info.rep.charging_source;
	mutex_unlock(&htc_batt_info.lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_MAINS)
			val->intval = (charger ==  CHARGER_AC ? 1 : 0);
		else if (psy->type == POWER_SUPPLY_TYPE_USB)
			val->intval = (charger ==  CHARGER_USB ? 1 : 0);
		else
			val->intval = 0;
		break;
	default:
		return -EINVAL;
	}
	
	return 0;
}

static int htc_battery_get_charging_status(void)
{
	u32 level;
	charger_type_t charger;	
	int ret;
	
	mutex_lock(&htc_batt_info.lock);
	charger = htc_batt_info.rep.charging_source;
	
	switch (charger) {
	case CHARGER_BATTERY:
		ret = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case CHARGER_USB:
	case CHARGER_AC:
		level = htc_batt_info.rep.level;
		if (level == 100)
			ret = POWER_SUPPLY_STATUS_FULL;
		else
			ret = POWER_SUPPLY_STATUS_CHARGING;
		break;
	default:
		ret = POWER_SUPPLY_STATUS_UNKNOWN;
	}
	mutex_unlock(&htc_batt_info.lock);
	return ret;
}

static int htc_battery_get_property(struct power_supply *psy, 
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = htc_battery_get_charging_status();
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = htc_batt_info.present;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		mutex_lock(&htc_batt_info.lock);
		val->intval = htc_batt_info.rep.level;
		mutex_unlock(&htc_batt_info.lock);
		break;
	default:		
		return -EINVAL;
	}
	
	return 0;
}

#define HTC_BATTERY_ATTR(_name)							\
{										\
	.attr = { .name = #_name, .mode = S_IRUGO},	\
	.show = htc_battery_show_property,					\
	.store = NULL,								\
}

static struct device_attribute htc_battery_attrs[] = {
	HTC_BATTERY_ATTR(batt_id),
	HTC_BATTERY_ATTR(batt_vol),
	HTC_BATTERY_ATTR(batt_temp),
	HTC_BATTERY_ATTR(batt_current),
	HTC_BATTERY_ATTR(charging_source),
	HTC_BATTERY_ATTR(charging_enabled),
	HTC_BATTERY_ATTR(full_bat),
};

enum {
	BATT_ID = 0,
	BATT_VOL,
	BATT_TEMP,
	BATT_CURRENT,
	CHARGING_SOURCE,
	CHARGING_ENABLED,
	FULL_BAT,
};


static ssize_t htc_battery_set_delta(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	int rc;
	unsigned long delta = 0;
	
	delta = simple_strtoul(buf, NULL, 10);

	if (delta > 100)
		return -EINVAL;

	if (rc < 0)
		return rc;
	return count;
}

static struct device_attribute htc_set_delta_attrs[] = {
	__ATTR(delta, S_IWUSR | S_IWGRP, NULL, htc_battery_set_delta),
};

static int htc_battery_create_attrs(struct device * dev)
{
	int i, j, rc;
	
	for (i = 0; i < ARRAY_SIZE(htc_battery_attrs); i++) {
		rc = device_create_file(dev, &htc_battery_attrs[i]);
		if (rc)
			goto htc_attrs_failed;
	}

	for (j = 0; j < ARRAY_SIZE(htc_set_delta_attrs); j++) {
		rc = device_create_file(dev, &htc_set_delta_attrs[j]);
		if (rc)
			goto htc_delta_attrs_failed;
	}
	
	goto succeed;
	
htc_attrs_failed:
	while (i--)
		device_remove_file(dev, &htc_battery_attrs[i]);
htc_delta_attrs_failed:
	while (j--)
		device_remove_file(dev, &htc_set_delta_attrs[i]);
succeed:	
	return rc;
}

static ssize_t htc_battery_show_property(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	int i = 0;
	const ptrdiff_t off = attr - htc_battery_attrs;

	/* check cache time to decide if we need to update */
	if (htc_batt_info.update_time &&
            time_before(jiffies, htc_batt_info.update_time +
                                msecs_to_jiffies(cache_time)))
                goto dont_need_update;
	
	if (htc_get_batt_info(&htc_batt_info.rep) < 0) {
		printk(KERN_ERR "%s: rpc failed!!!\n", __FUNCTION__);
	} else {
		htc_batt_info.update_time = jiffies;
	}
dont_need_update:
	mutex_lock(&htc_batt_info.lock);
	switch (off) {
	case BATT_ID:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       htc_batt_info.rep.batt_id);
		break;
	case BATT_VOL:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       htc_batt_info.rep.batt_vol);
		break;
	case BATT_TEMP:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       htc_batt_info.rep.batt_temp);
		break;
	case BATT_CURRENT:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       htc_batt_info.rep.batt_current);
		break;
	case CHARGING_SOURCE:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       htc_batt_info.rep.charging_source);
		break;
	case CHARGING_ENABLED:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       htc_batt_info.rep.charging_enabled);
		break;		
	case FULL_BAT:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       htc_batt_info.rep.full_bat);
		break;
	default:
		i = -EINVAL;
	}	
	mutex_unlock(&htc_batt_info.lock);
	
	return i;
}

static int htc_battery_thread(void *data)
{
	daemonize("battery");
	allow_signal(SIGKILL);

	while (!signal_pending((struct task_struct *)current)) {
		msleep(10000);
		//htc_get_batt_info(&htc_batt_info.rep);
		if (!bat_suspended && !htc_get_batt_info(&htc_batt_info.rep)) {
			htc_batt_info.update_time = jiffies;
		}
	}
	return 0;
}
static void htc_cable_notify_do_work(struct work_struct *work)
{
	if (g_usb_online!=2) usb_status_notifier_func(g_usb_online);
}

static int htc_battery_probe(struct platform_device *pdev)
{
	int i, rc;

	htc_batt_info.resources = (smem_batt_t *)pdev->dev.platform_data;

	if (!htc_batt_info.resources) {
		printk(KERN_ERR "%s: no pdata resources!\n", __FUNCTION__);
		return -EINVAL;
	}

	/* init battery gpio */
	if ((rc = init_batt_gpio()) < 0) {
		printk(KERN_ERR "%s: init battery gpio failed!\n", __FUNCTION__);
		return rc;
	}

	/* init structure data member */
	htc_batt_info.update_time 	= jiffies;
	/* A9 will NOT shutdown the phone if battery is pluged out. Not sure if we should */
	htc_batt_info.present 		= gpio_get_value(GPIO_BATTERY_DETECTION);

	/* init power supplier framework */
	for (i = 0; i < ARRAY_SIZE(htc_power_supplies); i++) {
		rc = power_supply_register(&pdev->dev, &htc_power_supplies[i]);
		if (rc)
			printk(KERN_ERR "Failed to register power supply (%d)\n", rc);	
	}

	/* create htc detail attributes */
	htc_battery_create_attrs(htc_power_supplies[CHARGER_BATTERY].dev);
	INIT_WORK(&htc_cable_notify_work, htc_cable_notify_do_work);
	/* init static battery settings like initial level */
	if ( init_battery_settings( &htc_batt_info.rep ) < 0)
		printk(KERN_ERR "%s: init battery settings failed\n", __FUNCTION__);

	htc_battery_initial = 1;

//	htc_batt_info.rep.charging_source = CHARGER_BATTERY;
//	if (htc_get_batt_info(&htc_batt_info.rep) < 0)
//		printk(KERN_ERR "%s: get info failed\n", __FUNCTION__);

//	if (htc_rpc_set_delta(1) < 0)
//		printk(KERN_ERR "%s: set delta failed\n", __FUNCTION__);
	htc_batt_info.update_time = jiffies;
	kernel_thread(htc_battery_thread, NULL, CLONE_KERNEL);

	return 0;
}

#if CONFIG_PM
static int htc_battery_suspend(struct platform_device* device, pm_message_t mesg)
{
	bat_suspended = 1;
	return 0;
}

static int htc_battery_resume(struct platform_device* device)
{
	bat_suspended = 0;
	return 0;
}
#else
 #define htc_battery_suspend NULL
 #define htc_battery_resume NULL
#endif

static struct platform_driver htc_battery_driver = {
	.probe	= htc_battery_probe,
	.driver	= {
		.name	= MODULE_NAME,
		.owner	= THIS_MODULE,
	},
	.suspend = htc_battery_suspend,
	.resume = htc_battery_resume,
};

static int __init htc_battery_init(void)
{
	wake_lock_init(&vbus_wake_lock, WAKE_LOCK_SUSPEND, "vbus_present");
	mutex_init(&htc_batt_info.lock);
	platform_driver_register(&htc_battery_driver);
	return 0;
}

module_init(htc_battery_init);
MODULE_DESCRIPTION("HTC Battery Driver");
MODULE_LICENSE("GPL");

