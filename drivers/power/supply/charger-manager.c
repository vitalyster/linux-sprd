/*
 * Copyright (C) 2011 Samsung Electronics Co., Ltd.
 * MyungJoo Ham <myungjoo.ham@samsung.com>
 *
 * This driver enables to monitor battery health and control charger
 * during suspend-to-mem.
 * Charger manager depends on other devices. register this later than
 * the depending devices.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
**/

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/io.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/rtc.h>
#include <linux/reboot.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/power/charger-manager.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/of.h>
#include <linux/thermal.h>
#include <linux/pmic-voter.h>
#include <linux/usb/phy.h>
#include <vendor/common/zte_misc.h>

/*
 * Default termperature threshold for charging.
 * Every temperature units are in tenth of centigrade.
 */
#define CM_DEFAULT_RECHARGE_TEMP_DIFF	50
#define CM_DEFAULT_CHARGE_TEMP_MAX	500
#define CM_CAP_CYCLE_TRACK_TIME		15
#define CM_UVLO_OFFSET			50000
#define CM_FORCE_SET_FUEL_CAP_FULL	1000
#define CM_LOW_TEMP_REGION		100
#define CM_LOW_TEMP_SHUTDOWN_VALTAGE	3200000
#define CM_TRACK_CAPACITY_SHUTDOWN_START_VOLTAGE	3500000
#define CM_TRACK_CAPACITY_START_VOLTAGE	3650000
#define CM_TRACK_CAPACITY_START_CURRENT	30000
#define CM_TRACK_CAPACITY_KEY0		0x20160726
#define CM_TRACK_CAPACITY_KEY1		0x15211517
#define CM_TRACK_CAPACITY_VOLTAGE_OFFSET	5000
#define CM_TRACK_CAPACITY_CURRENT_OFFSET	5000
#define CM_TRACK_HIGH_TEMP_THRESHOLD	450
#define CM_TRACK_LOW_TEMP_THRESHOLD	150
#define CM_TRACK_TIMEOUT_THRESHOLD	108000
#define CM_TRACK_START_CAP_THRESHOLD	200
#define CM_CAP_ONE_PERCENT		10
#define CM_HCAP_DECREASE_STEP		8
#define CM_HCAP_THRESHOLD		1001
#define CM_CAP_FULL_PERCENT		1000
#define CM_CAP_MAGIC_NUM		0x5A5AA5A5
#define CM_CAPACITY_LEVEL_CRITICAL	0
#define CM_CAPACITY_LEVEL_LOW		15
#define CM_CAPACITY_LEVEL_NORMAL	85
#define CM_CAPACITY_LEVEL_FULL		100

#define CM_TRACK_FILE_PATH "/mnt/vendor/battery/calibration_data/.battery_file"

extern int musb_lpm_usb_disconnect(void);

static const char * const default_event_names[] = {
	[CM_EVENT_UNKNOWN] = "Unknown",
	[CM_EVENT_BATT_FULL] = "Battery Full",
	[CM_EVENT_BATT_IN] = "Battery Inserted",
	[CM_EVENT_BATT_OUT] = "Battery Pulled Out",
	[CM_EVENT_BATT_OVERHEAT] = "Battery Overheat",
	[CM_EVENT_BATT_COLD] = "Battery Cold",
	[CM_EVENT_EXT_PWR_IN_OUT] = "External Power Attach/Detach",
	[CM_EVENT_CHG_START_STOP] = "Charging Start/Stop",
	[CM_EVENT_OTHERS] = "Other battery events"
};

static const char * const jeita_type_names[] = {
	[CM_JEITA_DCP] = "cm-dcp-jeita-temp-table",
	[CM_JEITA_SDP] = "cm-sdp-jeita-temp-table",
	[CM_JEITA_CDP] = "cm-cdp-jeita-temp-table",
	[CM_JEITA_UNKNOWN] = "cm-unknown-jeita-temp-table",
	[CM_JEITA_FCHG] = "cm-fchg-jeita-temp-table",
};

static char *charger_manager_supplied_to[] = {
	"audio-ldo",
};

/*
 * Regard CM_JIFFIES_SMALL jiffies is small enough to ignore for
 * delayed works so that we can run delayed works with CM_JIFFIES_SMALL
 * without any delays.
 */
#define	CM_JIFFIES_SMALL	(2)

/* If y is valid (> 0) and smaller than x, do x = y */
#define CM_MIN_VALID(x, y)	x = (((y > 0) && ((x) > (y))) ? (y) : (x))

/*
 * Regard CM_RTC_SMALL (sec) is small enough to ignore error in invoking
 * rtc alarm. It should be 2 or larger
 */
#define CM_RTC_SMALL		(2)

#define UEVENT_BUF_SIZE		32

static LIST_HEAD(cm_list);
static DEFINE_MUTEX(cm_list_mtx);

/* About in-suspend (suspend-again) monitoring */
static struct alarm *cm_timer;

static bool cm_suspended;
static bool cm_timer_set;
static unsigned long cm_suspend_duration_ms;
static enum cm_event_types cm_event_type;
static char *cm_event_msg;

/* About normal (not suspended) monitoring */
static unsigned long polling_jiffy = ULONG_MAX; /* ULONG_MAX: no polling */
static unsigned long next_polling; /* Next appointed polling time */
static struct workqueue_struct *cm_wq; /* init at driver add */
static struct delayed_work cm_monitor_work; /* init at driver add */

static bool allow_charger_enable;
static bool is_charger_mode;
static void cm_notify_type_handle(struct charger_manager *cm, enum cm_event_types type, char *msg);
static void cm_update_charger_type_status(struct charger_manager *cm);

#ifdef CONFIG_VENDOR_POWER_VOTER
static int charger_get_prop_by_name(const char *name, enum power_supply_property psp, int *data);
static int charger_set_prop_by_name(const char *name, enum power_supply_property psp, int data);
#endif

static int __init boot_calibration_mode(char *str)
{
	if (!str)
		return 0;

	if (!strncmp(str, "cali", strlen("cali")) ||
	    !strncmp(str, "autotest", strlen("autotest")))
		allow_charger_enable = true;
	else if (!strncmp(str, "charger", strlen("charger")))
		is_charger_mode =  true;

	return 0;
}
__setup("androidboot.mode=", boot_calibration_mode);

static void cm_cap_remap_init_boundary(struct charger_desc *desc, int index,
				       struct device *dev)
{

	if (index == 0) {
		desc->cap_remap_table[index].lb = (desc->cap_remap_table[index].lcap) * 1000;
		desc->cap_remap_total_cnt = desc->cap_remap_table[index].lcap;
	} else {
		desc->cap_remap_table[index].lb = desc->cap_remap_table[index - 1].hb +
			(desc->cap_remap_table[index].lcap -
			 desc->cap_remap_table[index - 1].hcap) * 1000;
		desc->cap_remap_total_cnt += (desc->cap_remap_table[index].lcap -
					      desc->cap_remap_table[index - 1].hcap);
	}

	desc->cap_remap_table[index].hb = desc->cap_remap_table[index].lb +
		(desc->cap_remap_table[index].hcap - desc->cap_remap_table[index].lcap) *
		desc->cap_remap_table[index].cnt * 1000;

	desc->cap_remap_total_cnt +=
		(desc->cap_remap_table[index].hcap - desc->cap_remap_table[index].lcap) *
		desc->cap_remap_table[index].cnt;

	dev_info(dev, "%s, cap_remap_table[%d].lb =%d,cap_remap_table[%d].hb = %d\n",
		 __func__, index, desc->cap_remap_table[index].lb, index,
		 desc->cap_remap_table[index].hb);
}

/*
 * cm_capacity_remap - remap fuel_cap
 * @ fuel_cap: cap from fuel gauge
 * Return the remapped cap
 */
static int cm_capacity_remap(struct charger_manager *cm, int fuel_cap)
{
	int i, temp, cap = 0;

	if (!cm->desc->cap_remap_table)
		return fuel_cap;

	if (fuel_cap < 0) {
		fuel_cap = 0;
		return 0;
	} else if (fuel_cap >  CM_CAP_FULL_PERCENT) {
		fuel_cap  = CM_CAP_FULL_PERCENT;
		return fuel_cap;
	}

	temp = fuel_cap * cm->desc->cap_remap_total_cnt;

	for (i = 0; i < cm->desc->cap_remap_table_len; i++) {
		if (temp <= cm->desc->cap_remap_table[i].lb) {
			if (i == 0)
				cap = DIV_ROUND_CLOSEST(temp, 100);
			else
				cap = DIV_ROUND_CLOSEST((temp -
					cm->desc->cap_remap_table[i - 1].hb), 100) +
					cm->desc->cap_remap_table[i - 1].hcap * 10;
			break;
		} else if (temp <= cm->desc->cap_remap_table[i].hb) {
			cap = DIV_ROUND_CLOSEST((temp - cm->desc->cap_remap_table[i].lb),
						cm->desc->cap_remap_table[i].cnt * 100)
				+ cm->desc->cap_remap_table[i].lcap * 10;
			break;
		}

		if (i == cm->desc->cap_remap_table_len - 1 && temp > cm->desc->cap_remap_table[i].hb)
			cap = DIV_ROUND_CLOSEST((temp - cm->desc->cap_remap_table[i].hb), 100)
				+ cm->desc->cap_remap_table[i].hcap;

	}

	return cap;
}

/*
 * cm_capacity_unmap - unmap remapped cap to real fuel gauge cap
 * @remmaped_cap: remapped_cap from cm_capacity_remap function
 * Return real fuel gauge cap
 */
static int cm_capacity_unmap(struct charger_manager *cm, int cap)
{
	int fuel_cap, i;

	if (!cm->desc->cap_remap_table)
		return cap;

	for (i = cm->desc->cap_remap_table_len - 1; i >= 0; i--) {
		if (cap >= (cm->desc->cap_remap_table[i].hcap * 10)) {
			fuel_cap = (cap - cm->desc->cap_remap_table[i].hcap * 10) * 100 +
				cm->desc->cap_remap_table[i].hb;
			break;
		} else if (cap >= (cm->desc->cap_remap_table[i].lcap * 10)) {
			fuel_cap = (cap - cm->desc->cap_remap_table[i].lcap * 10) *
				cm->desc->cap_remap_table[i].cnt * 100 +
				cm->desc->cap_remap_table[i].lb;
			break;
		}

		if (i == 0 && cap <= cm->desc->cap_remap_table[i].lcap * 10) {
			fuel_cap = cap * 100;
			break;
		}
	}

	fuel_cap  = DIV_ROUND_CLOSEST(fuel_cap, cm->desc->cap_remap_total_cnt);

	return fuel_cap;
}

static int cm_init_cap_remap_table(struct charger_desc *desc,
				   struct device *dev)
{

	struct device_node *np = dev->of_node;
	const __be32 *list;
	int i, size;

	list = of_get_property(np, "cm-cap-remap-table", &size);
	if (!list || !size) {
		dev_err(dev, "%s  get cm-cap-remap-table fail\n", __func__);
		return 0;
	}

	desc->cap_remap_table_len = size / (3 * sizeof(__be32));
	desc->cap_remap_table = devm_kzalloc(dev, sizeof(struct cap_remap_table) *
				(desc->cap_remap_table_len + 1), GFP_KERNEL);
	if (!desc->cap_remap_table) {
		dev_err(dev, "%s, get cap_remap_table fail\n", __func__);
		return -ENOMEM;
	}
	for (i = 0; i < desc->cap_remap_table_len; i++) {
		desc->cap_remap_table[i].lcap = be32_to_cpu(*list++);
		desc->cap_remap_table[i].hcap = be32_to_cpu(*list++);
		desc->cap_remap_table[i].cnt = be32_to_cpu(*list++);

		cm_cap_remap_init_boundary(desc, i, dev);

		dev_info(dev, " %s,cap_remap_table[%d].lcap= %d,cap_remap_table[%d].hcap = %d,"
			 "cap_remap_table[%d].cnt= %d\n",
		       __func__, i, desc->cap_remap_table[i].lcap,
			i, desc->cap_remap_table[i].hcap, i, desc->cap_remap_table[i].cnt);
	}

	if (desc->cap_remap_table[desc->cap_remap_table_len - 1].hcap != 100)
		desc->cap_remap_total_cnt +=
			(100 - desc->cap_remap_table[desc->cap_remap_table_len - 1].hcap);

	dev_info(dev, "%s, cap_remap_total_cnt =%d, cap_remap_table_len = %d\n",
	       __func__, desc->cap_remap_total_cnt, desc->cap_remap_table_len);

	return 0;
}

/**
 * is_batt_present - See if the battery presents in place.
 * @cm: the Charger Manager representing the battery.
 */
static bool is_batt_present(struct charger_manager *cm)
{
	union power_supply_propval val;
	struct power_supply *psy;
	bool present = false;
	int i, ret;

	switch (cm->desc->battery_present) {
	case CM_BATTERY_PRESENT:
		present = true;
		break;
	case CM_NO_BATTERY:
		break;
	case CM_FUEL_GAUGE:
		psy = power_supply_get_by_name(cm->desc->psy_fuel_gauge);
		if (!psy)
			break;

		ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_PRESENT,
				&val);
		if (ret == 0 && val.intval)
			present = true;
		power_supply_put(psy);
		break;
	case CM_CHARGER_STAT:
		for (i = 0; cm->desc->psy_charger_stat[i]; i++) {
			psy = power_supply_get_by_name(
					cm->desc->psy_charger_stat[i]);
			if (!psy) {
				dev_err(cm->dev, "Cannot find power supply \"%s\"\n",
					cm->desc->psy_charger_stat[i]);
				continue;
			}

			ret = power_supply_get_property(psy,
				POWER_SUPPLY_PROP_PRESENT, &val);
			power_supply_put(psy);
			if (ret == 0 && val.intval) {
				present = true;
				break;
			}
		}
		break;
	}

	return present;
}

/**
 * is_ext_pwr_online - See if an external power source is attached to charge
 * @cm: the Charger Manager representing the battery.
 *
 * Returns true if at least one of the chargers of the battery has an external
 * power source attached to charge the battery regardless of whether it is
 * actually charging or not.
 */
static bool is_ext_pwr_online(struct charger_manager *cm)
{
	union power_supply_propval val;
	struct power_supply *psy;
	bool online = false;
	int i, ret;

	/* If at least one of them has one, it's yes. */
	for (i = 0; cm->desc->psy_charger_stat[i]; i++) {
		psy = power_supply_get_by_name(cm->desc->psy_charger_stat[i]);
		if (!psy) {
			dev_err(cm->dev, "Cannot find power supply \"%s\"\n",
					cm->desc->psy_charger_stat[i]);
			continue;
		}

		ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_ONLINE,
				&val);
		power_supply_put(psy);
		if (ret == 0 && val.intval) {
			online = true;
			break;
		}
	}

	return online;
}

 /**
  * get_batt_uA - Get the current level of the battery
  * @cm: the Charger Manager representing the battery.
  * @uA: the current level returned.
  *
  * Returns 0 if there is no error.
  * Returns a negative value on error.
  */
static int get_batt_uA(struct charger_manager *cm, int *uA)
{
	union power_supply_propval val;
	struct power_supply *fuel_gauge;
	int ret;

	fuel_gauge = power_supply_get_by_name(cm->desc->psy_fuel_gauge);
	if (!fuel_gauge)
		return -ENODEV;

	ret = power_supply_get_property(fuel_gauge,
					POWER_SUPPLY_PROP_CURRENT_AVG, &val);
	power_supply_put(fuel_gauge);
	if (ret)
		return ret;

	*uA = val.intval;
	return 0;
}

 /**
  * get_batt_cur_now - Get the current level of the battery
  * @cm: the Charger Manager representing the battery.
  * @uA: the current level returned.
  *
  * Returns 0 if there is no error.
  * Returns a negative value on error.
  */
static int get_batt_cur_now(struct charger_manager *cm, int *uA)
{
	union power_supply_propval val;
	struct power_supply *fuel_gauge;
	int ret;

	fuel_gauge = power_supply_get_by_name(cm->desc->psy_fuel_gauge);
	if (!fuel_gauge)
		return -ENODEV;

	ret = power_supply_get_property(fuel_gauge,
					POWER_SUPPLY_PROP_CURRENT_NOW, &val);
	power_supply_put(fuel_gauge);
	if (ret)
		return ret;

	*uA = val.intval;
	return 0;
}

/**
 *
 * get_batt_uV - Get the voltage level of the battery
 * @cm: the Charger Manager representing the battery.
 * @uV: the voltage level returned.
 *
 * Returns 0 if there is no error.
 * Returns a negative value on error.
 */
static int get_batt_uV(struct charger_manager *cm, int *uV)
{
	union power_supply_propval val;
	struct power_supply *fuel_gauge;
	int ret;

	fuel_gauge = power_supply_get_by_name(cm->desc->psy_fuel_gauge);
	if (!fuel_gauge)
		return -ENODEV;

	ret = power_supply_get_property(fuel_gauge,
				POWER_SUPPLY_PROP_VOLTAGE_AVG, &val);
	power_supply_put(fuel_gauge);
	if (ret)
		return ret;

	*uV = val.intval;
	return 0;
}
/*
 * get_batt_ocv - Get the battery ocv
 * level of the battery.
 * @cm: the Charger Manager representing the battery.
 * @uV: the voltage level returned.
 *
 * Returns 0 if there is no error.
 * Returns a negative value on error.
 */
static int get_batt_ocv(struct charger_manager *cm, int *ocv)
{
	union power_supply_propval val;
	struct power_supply *fuel_gauge;
	int ret;

	fuel_gauge = power_supply_get_by_name(cm->desc->psy_fuel_gauge);
	if (!fuel_gauge)
		return -ENODEV;

	ret = power_supply_get_property(fuel_gauge,
				POWER_SUPPLY_PROP_VOLTAGE_OCV, &val);
	power_supply_put(fuel_gauge);
	if (ret)
		return ret;

	*ocv = val.intval;
	return 0;
}

/*
 * get_batt_now - Get the battery voltage now
 * level of the battery.
 * @cm: the Charger Manager representing the battery.
 * @uV: the voltage level returned.
 *
 * Returns 0 if there is no error.
 * Returns a negative value on error.
 */
static int get_batt_vol_now(struct charger_manager *cm, int *ocv)
{
	union power_supply_propval val;
	struct power_supply *fuel_gauge;
	int ret;

	fuel_gauge = power_supply_get_by_name(cm->desc->psy_fuel_gauge);
	if (!fuel_gauge)
		return -ENODEV;

	ret = power_supply_get_property(fuel_gauge,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
	power_supply_put(fuel_gauge);
	if (ret)
		return ret;

	*ocv = val.intval;
	return 0;
}

static int get_batt_boot_vol(struct charger_manager *cm, int *boot_volt)
{
	union power_supply_propval val;
	struct power_supply *fuel_gauge;
	int ret;

	fuel_gauge = power_supply_get_by_name(cm->desc->psy_fuel_gauge);
	if (!fuel_gauge)
		return -ENODEV;

	ret = power_supply_get_property(fuel_gauge,
				POWER_SUPPLY_PROP_VOLTAGE_BOOT, &val);
	power_supply_put(fuel_gauge);
	if (ret)
		return ret;

	*boot_volt = val.intval;
	return 0;
}
/**
 * get_batt_cap - Get the cap level of the battery
 * @cm: the Charger Manager representing the battery.
 * @uV: the cap level returned.
 *
 * Returns 0 if there is no error.
 * Returns a negative value on error.
 */
static int get_batt_cap(struct charger_manager *cm, int *cap)
{
	union power_supply_propval val;
	struct power_supply *fuel_gauge;
	int ret;

	fuel_gauge = power_supply_get_by_name(cm->desc->psy_fuel_gauge);
	if (!fuel_gauge)
		return -ENODEV;

	val.intval = CM_CAPACITY;
	ret = power_supply_get_property(fuel_gauge,
				POWER_SUPPLY_PROP_CAPACITY, &val);
	power_supply_put(fuel_gauge);
	if (ret)
		return ret;

	*cap = val.intval;
	dev_info(cm->dev, "get_batt_cap: %d\n", *cap);
	return 0;
}

/**
 * get_batt_total_cap - Get the total capacity level of the battery
 * @cm: the Charger Manager representing the battery.
 * @uV: the total_cap level returned.
 *
 * Returns 0 if there is no error.
 * Returns a negative value on error.
 */
static int get_batt_total_cap(struct charger_manager *cm, u32 *total_cap)
{
	union power_supply_propval val;
	struct power_supply *fuel_gauge;
	int ret;

	fuel_gauge = power_supply_get_by_name(cm->desc->psy_fuel_gauge);
	if (!fuel_gauge)
		return -ENODEV;

	ret = power_supply_get_property(fuel_gauge,
					POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN,
					&val);
	power_supply_put(fuel_gauge);
	if (ret)
		return ret;

	*total_cap = val.intval;

	return 0;
}

/**
 * get_batt_energy_now - Get the energy_now of the battery
 * @cm: the Charger Manager representing the battery.
 * @value: the energy_now returned.
 *
 * Returns 0 if there is no error.
 * Returns a negative value on error.
 */
static int get_batt_energy_now(struct charger_manager *cm, int *value)
{
	union power_supply_propval val;
	struct power_supply *fuel_gauge;
	int ret;

	fuel_gauge = power_supply_get_by_name(cm->desc->psy_fuel_gauge);
	if (!fuel_gauge)
		return -ENODEV;

	ret = power_supply_get_property(fuel_gauge,
					POWER_SUPPLY_PROP_ENERGY_NOW,
					&val);
	power_supply_put(fuel_gauge);
	if (ret)
		return ret;

	*value = val.intval;

	return 0;
}

/*
 * get_boot_cap - Get the battery boot capacity
 * of the battery.
 * @cm: the Charger Manager representing the battery.
 * @cap: the battery capacity returned.
 *
 * Returns 0 if there is no error.
 * Returns a negative value on error.
 */
static int get_boot_cap(struct charger_manager *cm, int *cap)
{
	union power_supply_propval val;
	struct power_supply *fuel_gauge;
	int ret;

	fuel_gauge = power_supply_get_by_name(cm->desc->psy_fuel_gauge);
	if (!fuel_gauge)
		return -ENODEV;

	val.intval = CM_BOOT_CAPACITY;
	ret = power_supply_get_property(fuel_gauge,
				POWER_SUPPLY_PROP_CAPACITY, &val);
	power_supply_put(fuel_gauge);
	if (ret)
		return ret;

	*cap = val.intval;
	return 0;
}

/**
 * set_batt_total_cap - Set the total_cap level of the battery
 * @cm: the Charger Manager representing the battery.
 * @total_cap: the total_cap level to set.
 *
 * Returns 0 if there is no error.
 * Returns a negative value on error.
 */
static int set_batt_total_cap(struct charger_manager *cm, int total_cap)
{
	union power_supply_propval val;
	struct power_supply *fuel_gauge;
	int ret;

	fuel_gauge = power_supply_get_by_name(cm->desc->psy_fuel_gauge);
	if (!fuel_gauge)
		return -ENODEV;

	val.intval = total_cap * 1000;
	ret = power_supply_set_property(fuel_gauge,
					POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN,
					&val);
	power_supply_put(fuel_gauge);
	if (ret)
		dev_err(cm->dev, "failed to set battery capacity\n");

	return ret;
}

/**
 * get_charger_type - Get the charger type
 * @cm: the Charger Manager representing the battery.
 * @type: the charger type returned.
 *
 * Returns 0 if there is no error.
 * Returns a negative value on error.
 */
static int get_charger_type(struct charger_manager *cm, u32 *type)
{
	union power_supply_propval val;
	struct power_supply *psy;
	int ret = -EINVAL, i;

	for (i = 0; cm->desc->psy_charger_stat[i]; i++) {
		psy = power_supply_get_by_name(cm->desc->psy_charger_stat[i]);
		if (!psy) {
			dev_err(cm->dev, "Cannot find power supply \"%s\"\n",
				cm->desc->psy_charger_stat[i]);
			continue;
		}

		ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_USB_TYPE,
						&val);
		power_supply_put(psy);
		if (ret == 0) {
			*type = val.intval;
			break;
		}
	}

	return ret;
}

/**
 * set_batt_cap - Set the cap level of the battery
 * @cm: the Charger Manager representing the battery.
 *
 * Returns 0 if there is no error.
 * Returns a negative value on error.
 */
static int set_batt_cap(struct charger_manager *cm, int cap)
{
	union power_supply_propval val;
	struct power_supply *fuel_gauge;
	int ret;

	dev_info(cm->dev, "set_batt_cap: %d\n", cap);
	fuel_gauge = power_supply_get_by_name(cm->desc->psy_fuel_gauge);
	if (!fuel_gauge) {
		dev_err(cm->dev, "can not find fuel gauge device\n");
		return -ENODEV;
	}

	val.intval = cap;
	ret = power_supply_set_property(fuel_gauge, POWER_SUPPLY_PROP_CAPACITY,
					&val);
	power_supply_put(fuel_gauge);
	if (ret)
		dev_err(cm->dev, "failed to save current battery capacity\n");

	return ret;
}
/**
 * get_charger_voltage - Get the charging voltage from fgu
 * @cm: the Charger Manager representing the battery.
 * @cur: the charging input voltage returned.
 *
 * Returns 0 if there is no error.
 * Returns a negative value on error.
 */
static int get_charger_voltage(struct charger_manager *cm, int *vol)
{
	union power_supply_propval val;
	struct power_supply *fuel_gauge;
	int ret = -ENODEV;

	if (!is_ext_pwr_online(cm))
		return 0;

	fuel_gauge = power_supply_get_by_name(cm->desc->psy_fuel_gauge);
	if (!fuel_gauge) {
		dev_err(cm->dev, "Cannot find power supply  %s\n",
			cm->desc->psy_fuel_gauge);
		return	ret;
	}

	ret = power_supply_get_property(fuel_gauge,
					POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
					&val);
	power_supply_put(fuel_gauge);
	if (ret == 0)
		*vol = val.intval;

	return ret;
}

/**
 * adjust_fuel_cap - Adjust the fuel cap level
 * @cm: the Charger Manager representing the battery.
 * @cap: the adjust fuel cap level.
 *
 * Returns 0 if there is no error.
 * Returns a negative value on error.
 */
static int adjust_fuel_cap(struct charger_manager *cm, int cap)
{
	union power_supply_propval val;
	struct power_supply *fuel_gauge;
	int ret;

	fuel_gauge = power_supply_get_by_name(cm->desc->psy_fuel_gauge);
	if (!fuel_gauge)
		return -ENODEV;

	val.intval = cap;
	ret = power_supply_set_property(fuel_gauge,
					POWER_SUPPLY_PROP_CALIBRATE, &val);
	power_supply_put(fuel_gauge);
	if (ret)
		dev_err(cm->dev, "failed to adjust fuel cap\n");

	return ret;
}

/**
 * get_charger_current - Get the charging current from charging ic
 * @cm: the Charger Manager representing the battery.
 * @cur: the charging current returned.
 *
 * Returns 0 if there is no error.
 * Returns a negative value on error.
 */
static int get_charger_current(struct charger_manager *cm, int *cur)
{
	union power_supply_propval val;
	struct power_supply *psy;
	int i, ret = -ENODEV;

	/* If at least one of them has one, it's yes. */
	for (i = 0; cm->desc->psy_charger_stat[i]; i++) {
		psy = power_supply_get_by_name(cm->desc->psy_charger_stat[i]);
		if (!psy) {
			dev_err(cm->dev, "Cannot find power supply \"%s\"\n",
				cm->desc->psy_charger_stat[i]);
			continue;
		}

		ret = power_supply_get_property(psy,
						POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
						&val);
		power_supply_put(psy);
		if (ret == 0) {
			*cur = val.intval;
			break;
		}
	}

	return ret;
}

/**
 * get_charger_limit_current - Get the charging limit current from charging ic
 * @cm: the Charger Manager representing the battery.
 * @cur: the charging input limit current returned.
 *
 * Returns 0 if there is no error.
 * Returns a negative value on error.
 */
static int get_charger_limit_current(struct charger_manager *cm, int *cur)
{
	union power_supply_propval val;
	struct power_supply *psy;
	int i, ret = -ENODEV;

	/* If at least one of them has one, it's yes. */
	for (i = 0; cm->desc->psy_charger_stat[i]; i++) {
		psy = power_supply_get_by_name(cm->desc->psy_charger_stat[i]);
		if (!psy) {
			dev_err(cm->dev, "Cannot find power supply \"%s\"\n",
				cm->desc->psy_charger_stat[i]);
			continue;
		}

		ret = power_supply_get_property(psy,
						POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
						&val);
		power_supply_put(psy);
		if (ret == 0) {
			*cur = val.intval;
			break;
		}
	}

	return ret;
}

/**
 * is_charging - Returns true if the battery is being charged.
 * @cm: the Charger Manager representing the battery.
 */
static bool is_charging(struct charger_manager *cm)
{
	int i, ret;
	bool charging = false;
	struct power_supply *psy;
	union power_supply_propval val;

	/* If there is no battery, it cannot be charged */
	if (!is_batt_present(cm))
		return false;

	/* If at least one of the charger is charging, return yes */
	for (i = 0; cm->desc->psy_charger_stat[i]; i++) {
		/* 1. The charger sholuld not be DISABLED */
		if (cm->emergency_stop)
			continue;
		if (!cm->charger_enabled)
			continue;

		psy = power_supply_get_by_name(cm->desc->psy_charger_stat[i]);
		if (!psy) {
			dev_err(cm->dev, "Cannot find power supply \"%s\"\n",
					cm->desc->psy_charger_stat[i]);
			continue;
		}

		/* 2. The charger should be online (ext-power) */
		ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_ONLINE,
				&val);
		if (ret) {
			dev_warn(cm->dev, "Cannot read ONLINE value from %s\n",
				 cm->desc->psy_charger_stat[i]);
			power_supply_put(psy);
			continue;
		}
		if (val.intval == 0) {
			power_supply_put(psy);
			continue;
		}

		/*
		 * 3. The charger should not be FULL, DISCHARGING,
		 * or NOT_CHARGING.
		 */
		ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_STATUS,
				&val);
		power_supply_put(psy);
		if (ret) {
			dev_warn(cm->dev, "Cannot read STATUS value from %s\n",
				 cm->desc->psy_charger_stat[i]);
			continue;
		}
		if (val.intval == POWER_SUPPLY_STATUS_FULL ||
				val.intval == POWER_SUPPLY_STATUS_DISCHARGING ||
				val.intval == POWER_SUPPLY_STATUS_NOT_CHARGING)
			continue;

		/* Then, this is charging. */
		charging = true;
		break;
	}

	return charging;
}


#define FULL_BATT_GAP_TO_TOPOFF_UV 50000

/**
 * is_full_charged - Returns true if the battery is fully charged.
 * @cm: the Charger Manager representing the battery.
 */
static bool is_full_charged(struct charger_manager *cm)
{
	struct charger_desc *desc = cm->desc;
	union power_supply_propval val;
	struct power_supply *fuel_gauge;
	bool is_full = false;
	int ret = 0;
	int uV, uA;
#ifdef CONFIG_CHARGER_BQ2560X
	struct power_supply *chg_psy = NULL;
	int i = 0;
	bool is_charger_ic_terminated = false;
#endif

	/* If there is no battery, it cannot be charged */
	if (!is_batt_present(cm))
		return false;

	fuel_gauge = power_supply_get_by_name(cm->desc->psy_fuel_gauge);
	if (!fuel_gauge)
		return false;

	if (desc->fullbatt_full_capacity > 0) {
		val.intval = 0;

		/* Not full if capacity of fuel gauge isn't full */
		ret = power_supply_get_property(fuel_gauge,
				POWER_SUPPLY_PROP_CHARGE_FULL, &val);
		if (!ret && val.intval > desc->fullbatt_full_capacity) {
			is_full = true;
			goto out;
		}
	}

#ifdef CONFIG_CHARGER_BQ2560X
	for (i = 0; cm->desc->psy_charger_stat[i]; i++) {
		chg_psy = power_supply_get_by_name(cm->desc->psy_charger_stat[i]);
		if (!chg_psy) {
			dev_err(cm->dev, "Cannot find power supply \"%s\"\n",
					cm->desc->psy_charger_stat[i]);
			continue;
		}

		ret = power_supply_get_property(chg_psy, POWER_SUPPLY_PROP_STATUS,
				&val);
		power_supply_put(chg_psy);
		if (ret) {
			dev_warn(cm->dev, "Cannot read STATUS value from %s\n",
				 cm->desc->psy_charger_stat[i]);
			continue;
		}

		if (val.intval == POWER_SUPPLY_STATUS_FULL)
			is_charger_ic_terminated = true;
	}

	if (is_charger_ic_terminated) {
		if (cm->health == POWER_SUPPLY_HEALTH_WARM) {
			dev_info(cm->dev, "charger ic report warm full! health: %d\n", cm->health);
			is_full = true;
			cm->desc->force_set_full = true;
		} else if (cm->health == POWER_SUPPLY_HEALTH_COOL || cm->health == POWER_SUPPLY_HEALTH_GOOD) {
			adjust_fuel_cap(cm, CM_FORCE_SET_FUEL_CAP_FULL);

			dev_info(cm->dev, "charger ic reprot normal full! health: %d\n", cm->health);
			is_full = true;
			cm->desc->force_set_full = true;
		}
	}
	goto out;
#endif

	/* Full, if it's over the fullbatt voltage */
	if (desc->fullbatt_uV > 0 && desc->fullbatt_uA > 0) {
		ret = get_batt_uV(cm, &uV);
		if (ret)
			goto out;

		ret = get_batt_uA(cm, &uA);
		if (ret)
			goto out;

		if (cm->health == POWER_SUPPLY_HEALTH_WARM) {
			if (uV >= (desc->fullbatt_warm_uV - FULL_BATT_GAP_TO_TOPOFF_UV) &&
				(uA < 0 || uA <= desc->fullbatt_uA)) {
				dev_info(cm->dev, "warm full! vbat %d, ibat %d, health: %d\n",
					uV, uA, cm->health);
				is_full = true;
				cm->desc->force_set_full = true;
			}
			goto out;
		}

		if (uV >= desc->fullbatt_uV && uA <= desc->fullbatt_uA && uA > 0) {
			if (++desc->trigger_cnt > 1) {
				if (cm->desc->cap >= CM_CAP_FULL_PERCENT) {
					if (desc->trigger_cnt == 2)
						adjust_fuel_cap(cm, CM_FORCE_SET_FUEL_CAP_FULL);
					is_full = true;
				} else {
					is_full = false;
					adjust_fuel_cap(cm, CM_FORCE_SET_FUEL_CAP_FULL);
				}
				cm->desc->force_set_full = true;
				dev_info(cm->dev, "normal full! is_full: %d, vbat %d, ibat %d, health: %d\n",
					is_full, uV, uA, cm->health);
			} else {
				is_full = false;
			}
			goto out;
		} else {
			is_full = false;
			desc->trigger_cnt = 0;
			goto out;
		}
	}

	/* Full, if the capacity is more than fullbatt_soc */
	if (desc->fullbatt_soc > 0) {
		val.intval = 0;

		ret = power_supply_get_property(fuel_gauge,
				POWER_SUPPLY_PROP_CAPACITY, &val);
		if (!ret && val.intval >= desc->fullbatt_soc) {
			is_full = true;
			goto out;
		}
	}

out:
	power_supply_put(fuel_gauge);
	return is_full;
}

/**
 * is_polling_required - Return true if need to continue polling for this CM.
 * @cm: the Charger Manager representing the battery.
 */
static bool is_polling_required(struct charger_manager *cm)
{
	switch (cm->desc->polling_mode) {
	case CM_POLL_DISABLE:
		return false;
	case CM_POLL_ALWAYS:
		return true;
	case CM_POLL_EXTERNAL_POWER_ONLY:
		return is_ext_pwr_online(cm);
	case CM_POLL_CHARGING_ONLY:
		return is_charging(cm);
	default:
		dev_warn(cm->dev, "Incorrect polling_mode (%d)\n",
			 cm->desc->polling_mode);
	}

	return false;
}

static int try_charger_enable_by_psy(struct charger_manager *cm, bool enable)
{
	struct charger_desc *desc = cm->desc;
	union power_supply_propval val;
	struct power_supply *psy;
	int i, err;

	for (i = 0; desc->psy_charger_stat[i]; i++) {
		psy = power_supply_get_by_name(desc->psy_charger_stat[i]);
		if (!psy) {
			dev_err(cm->dev, "Cannot find power supply \"%s\"\n",
				desc->psy_charger_stat[i]);
			continue;
		}

		val.intval = enable;
		err = power_supply_set_property(psy, POWER_SUPPLY_PROP_STATUS,
						&val);
		power_supply_put(psy);
		if (err)
			return err;
	}

	return 0;
}

/**
 * try_charger_enable - Enable/Disable chargers altogether
 * @cm: the Charger Manager representing the battery.
 * @enable: true: enable / false: disable
 *
 * Note that Charger Manager keeps the charger enabled regardless whether
 * the charger is charging or not (because battery is full or no external
 * power source exists) except when CM needs to disable chargers forcibly
 * bacause of emergency causes; when the battery is overheated or too cold.
 */
static int try_charger_enable(struct charger_manager *cm, bool enable)
{
	int err = 0;

	/* Ignore if it's redundant command */
	if (enable == cm->charger_enabled)
		return 0;

	if (enable) {
		if (cm->emergency_stop)
			return -EAGAIN;

		/*
		 * Enable charge is permitted in calibration mode
		 * even if use fake battery.
		 * So it will not return in calibration mode.
		 */
		if (!is_batt_present(cm) && !allow_charger_enable)
			return 0;
		/*
		 * Save start time of charging to limit
		 * maximum possible charging time.
		 */
		cm->charging_start_time = ktime_to_ms(ktime_get());
		cm->charging_end_time = 0;

		err = try_charger_enable_by_psy(cm, enable);
	} else {
		/*
		 * Save end time of charging to maintain fully charged state
		 * of battery after full-batt.
		 */
		cm->charging_start_time = 0;
		cm->charging_end_time = ktime_to_ms(ktime_get());

		err = try_charger_enable_by_psy(cm, enable);
	}

	if (!err)
		cm->charger_enabled = enable;

	return err;
}

/**
 * try_charger_restart - Restart charging.
 * @cm: the Charger Manager representing the battery.
 *
 * Restart charging by turning off and on the charger.
 */
static int try_charger_restart(struct charger_manager *cm)
{
	int err;

	if (cm->emergency_stop)
		return -EAGAIN;

	err = try_charger_enable(cm, false);
	if (err)
		return err;

	return try_charger_enable(cm, true);
}

/**
 * uevent_notify - Let users know something has changed.
 * @cm: the Charger Manager representing the battery.
 * @event: the event string.
 *
 * If @event is null, it implies that uevent_notify is called
 * by resume function. When called in the resume function, cm_suspended
 * should be already reset to false in order to let uevent_notify
 * notify the recent event during the suspend to users. While
 * suspended, uevent_notify does not notify users, but tracks
 * events so that uevent_notify can notify users later after resumed.
 */
static void uevent_notify(struct charger_manager *cm, const char *event)
{
	static char env_str[UEVENT_BUF_SIZE + 1] = "";
	static char env_str_save[UEVENT_BUF_SIZE + 1] = "";

	if (cm_suspended) {
		/* Nothing in suspended-event buffer */
		if (env_str_save[0] == 0) {
			if (!strncmp(env_str, event, UEVENT_BUF_SIZE))
				return; /* status not changed */
			strncpy(env_str_save, event, UEVENT_BUF_SIZE);
			return;
		}

		if (!strncmp(env_str_save, event, UEVENT_BUF_SIZE))
			return; /* Duplicated. */
		strncpy(env_str_save, event, UEVENT_BUF_SIZE);
		return;
	}

	if (event == NULL) {
		/* No messages pending */
		if (!env_str_save[0])
			return;

		strncpy(env_str, env_str_save, UEVENT_BUF_SIZE);
		kobject_uevent(&cm->dev->kobj, KOBJ_CHANGE);
		env_str_save[0] = 0;

		return;
	}

	/* status not changed */
	if (!strncmp(env_str, event, UEVENT_BUF_SIZE))
		return;

	/* save the status and notify the update */
	strncpy(env_str, event, UEVENT_BUF_SIZE);
	kobject_uevent(&cm->dev->kobj, KOBJ_CHANGE);

	dev_info(cm->dev, "%s\n", event);
}

/**
 * fullbatt_vchk - Check voltage drop some times after "FULL" event.
 * @work: the work_struct appointing the function
 *
 * If a user has designated "fullbatt_vchkdrop_ms/uV" values with
 * charger_desc, Charger Manager checks voltage drop after the battery
 * "FULL" event. It checks whether the voltage has dropped more than
 * fullbatt_vchkdrop_uV by calling this function after fullbatt_vchkrop_ms.
 */
static void fullbatt_vchk(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct charger_manager *cm = container_of(dwork,
			struct charger_manager, fullbatt_vchk_work);
	struct charger_desc *desc = cm->desc;
	int batt_ocv, err, diff;

	if (charger_policy_get_status() == true) {
		dev_info(cm->dev, "%s: ignore fullbatt_vchk fucntion when policy is enabled\n", __func__);
		return;
	}

	/* remove the appointment for fullbatt_vchk */
	cm->fullbatt_vchk_jiffies_at = 0;

	if (!desc->fullbatt_vchkdrop_uV || !desc->fullbatt_vchkdrop_ms)
		return;

	err = get_batt_ocv(cm, &batt_ocv);
	if (err) {
		dev_err(cm->dev, "%s: get_batt_ocV error(%d)\n", __func__, err);
		return;
	}

#ifdef CONFIG_VENDOR_POWER_VOTER
	if (cm->health == POWER_SUPPLY_HEALTH_WARM)
		diff = desc->fullbatt_warm_uV - batt_ocv;
	else
#endif
		diff = desc->fullbatt_uV - batt_ocv;

	if (diff < 0)
		return;

	dev_info(cm->dev, "VBATT dropped %duV after full-batt\n", diff);

	if (diff >= desc->fullbatt_vchkdrop_uV) {
		try_charger_restart(cm);
		uevent_notify(cm, "Recharging");
	}
}

/**
 * check_charging_duration - Monitor charging/discharging duration
 * @cm: the Charger Manager representing the battery.
 *
 * If whole charging duration exceed 'charging_max_duration_ms',
 * cm stop charging to prevent overcharge/overheat. If discharging
 * duration exceed 'discharging _max_duration_ms', charger cable is
 * attached, after full-batt, cm start charging to maintain fully
 * charged state for battery.
 */
static int check_charging_duration(struct charger_manager *cm)
{
	struct charger_desc *desc = cm->desc;
	u64 curr = ktime_to_ms(ktime_get());
	u64 duration;
	int batt_ocv, diff, ret = false;

	if (!desc->charging_max_duration_ms &&
			!desc->discharging_max_duration_ms)
		return ret;

	if (cm->charging_status != 0 &&
	    !(cm->charging_status & CM_CHARGE_DURATION_ABNORMAL))
		return ret;

	ret = get_batt_ocv(cm, &batt_ocv);
	if (ret) {
		dev_err(cm->dev, "failed to get battery OCV\n");
		return ret;
	}
	diff = desc->fullbatt_uV - batt_ocv;

	if (cm->charger_enabled) {
		duration = curr - cm->charging_start_time;

		if (duration > desc->charging_max_duration_ms &&
		    diff < desc->fullbatt_vchkdrop_uV) {
			dev_info(cm->dev, "Charging duration exceed %ums\n",
				 desc->charging_max_duration_ms);
			uevent_notify(cm, "Discharging");
			try_charger_enable(cm, false);
			cm->charging_status |= CM_CHARGE_DURATION_ABNORMAL;
			ret = true;
		}
	} else if (is_ext_pwr_online(cm) && !cm->charger_enabled &&
		(cm->charging_status & CM_CHARGE_DURATION_ABNORMAL)) {
		duration = curr - cm->charging_end_time;

		if (duration > desc->discharging_max_duration_ms &&
				is_ext_pwr_online(cm)) {
			dev_info(cm->dev, "Discharging duration exceed %ums\n",
				 desc->discharging_max_duration_ms);
			uevent_notify(cm, "Recharging");
			try_charger_enable(cm, true);
			cm->charging_status &= ~CM_CHARGE_DURATION_ABNORMAL;
			ret = true;
		}
	}

	if (cm->charging_status & CM_CHARGE_DURATION_ABNORMAL) {
		dev_info(cm->dev, "Charging duration is still exceed\n");
		return true;
	}

	return ret;
}

static int cm_get_battery_temperature_by_psy(struct charger_manager *cm,
					int *temp)
{
	struct power_supply *fuel_gauge;
	int ret;

	fuel_gauge = power_supply_get_by_name(cm->desc->psy_fuel_gauge);
	if (!fuel_gauge)
		return -ENODEV;

	ret = power_supply_get_property(fuel_gauge,
				POWER_SUPPLY_PROP_TEMP,
				(union power_supply_propval *)temp);
	power_supply_put(fuel_gauge);

	return ret;
}

static int cm_get_battery_temperature(struct charger_manager *cm,
					int *temp)
{
	int ret = 0;

	if (!cm->desc->measure_battery_temp)
		return -ENODEV;

#ifdef CONFIG_THERMAL
	if (cm->tzd_batt) {
		ret = thermal_zone_get_temp(cm->tzd_batt, temp);
		if (!ret)
			/* Calibrate temperature unit */
			*temp /= 100;
	} else
#endif
	{
		/* if-else continued from CONFIG_THERMAL */
		*temp = cm->desc->temperature;
	}

	return ret;
}

static int cm_check_thermal_status(struct charger_manager *cm)
{
	struct charger_desc *desc = cm->desc;
	int temp, upper_limit, lower_limit;
	int ret = 0;

	ret = cm_get_battery_temperature(cm, &temp);
	if (ret) {
		/* FIXME:
		 * No information of battery temperature might
		 * occur hazadous result. We have to handle it
		 * depending on battery type.
		 */
		dev_err(cm->dev, "Failed to get battery temperature\n");
		return 0;
	}

	upper_limit = desc->temp_max;
	lower_limit = desc->temp_min;

	if (cm->emergency_stop) {
		upper_limit -= desc->temp_diff;
		lower_limit += desc->temp_diff;
	}

	if (temp >= upper_limit) {
		ret = CM_EVENT_BATT_OVERHEAT;
		cm->health = POWER_SUPPLY_HEALTH_OVERHEAT;
	} else if (temp <= lower_limit) {
		ret = CM_EVENT_BATT_COLD;
		cm->health = POWER_SUPPLY_HEALTH_COLD;
	}

	return ret;
}

static int cm_check_charge_voltage(struct charger_manager *cm)
{
	struct charger_desc *desc = cm->desc;
	struct power_supply *fuel_gauge;
	union power_supply_propval val;
	int ret, charge_vol;

	if (!desc->charge_voltage_max || !desc->charge_voltage_drop)
		return -EINVAL;

	if (cm->charging_status != 0 &&
	    !(cm->charging_status & CM_CHARGE_VOLTAGE_ABNORMAL))
		return -EINVAL;

	fuel_gauge = power_supply_get_by_name(desc->psy_fuel_gauge);
	if (!fuel_gauge)
		return -ENODEV;

	ret = power_supply_get_property(fuel_gauge,
				POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
				&val);
	power_supply_put(fuel_gauge);
	if (ret)
		return ret;

	charge_vol = val.intval;

	if (cm->charger_enabled && charge_vol > desc->charge_voltage_max) {
		dev_info(cm->dev, "Charging voltage is larger than %d\n",
			 desc->charge_voltage_max);
		uevent_notify(cm, "Discharging");
		try_charger_enable(cm, false);
		cm->charging_status |= CM_CHARGE_VOLTAGE_ABNORMAL;
		power_supply_changed(cm->charger_psy);
		return 0;
	} else if (is_ext_pwr_online(cm) && !cm->charger_enabled &&
		   charge_vol <= (desc->charge_voltage_max - desc->charge_voltage_drop) &&
		   (cm->charging_status & CM_CHARGE_VOLTAGE_ABNORMAL)) {
		dev_info(cm->dev, "Charging voltage is less than %d, recharging\n",
			 desc->charge_voltage_max - desc->charge_voltage_drop);
		uevent_notify(cm, "Recharging");
		try_charger_enable(cm, true);
		cm->charging_status &= ~CM_CHARGE_VOLTAGE_ABNORMAL;
		power_supply_changed(cm->charger_psy);
		return 0;
	} else if (cm->charging_status & CM_CHARGE_VOLTAGE_ABNORMAL) {
		dev_info(cm->dev, "Charging voltage is still abnormal\n");
		return 0;
	}

	return -EINVAL;
}

static int cm_check_charge_health(struct charger_manager *cm)
{
	struct charger_desc *desc = cm->desc;
	struct power_supply *psy;
	union power_supply_propval val;
	int health = POWER_SUPPLY_HEALTH_UNKNOWN;
	int ret, i;

	if (cm->charging_status != 0 &&
	    !(cm->charging_status & CM_CHARGE_HEALTH_ABNORMAL))
		return -EINVAL;

	for (i = 0; desc->psy_charger_stat[i]; i++) {
		psy = power_supply_get_by_name(desc->psy_charger_stat[i]);
		if (!psy) {
			dev_err(cm->dev, "Cannot find power supply \"%s\"\n",
				desc->psy_charger_stat[i]);
			continue;
		}

		ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_HEALTH,
						&val);
		power_supply_put(psy);
		if (ret)
			return ret;
		health = val.intval;
	}

	if (health == POWER_SUPPLY_HEALTH_UNKNOWN)
		return -ENODEV;

	if (cm->charger_enabled && health != POWER_SUPPLY_HEALTH_GOOD) {
		dev_info(cm->dev, "Charging health is not good\n");
		uevent_notify(cm, "Discharging");
		try_charger_enable(cm, false);
		cm->charging_status |= CM_CHARGE_HEALTH_ABNORMAL;
		return 0;
	} else if (is_ext_pwr_online(cm) && !cm->charger_enabled &&
		health == POWER_SUPPLY_HEALTH_GOOD &&
		(cm->charging_status & CM_CHARGE_HEALTH_ABNORMAL)) {
		dev_info(cm->dev, "Charging health is recover good\n");
		uevent_notify(cm, "Recharging");
		try_charger_enable(cm, true);
		cm->charging_status &= ~CM_CHARGE_HEALTH_ABNORMAL;
		return 0;
	} else if (cm->charging_status & CM_CHARGE_HEALTH_ABNORMAL) {
		dev_info(cm->dev, "Charging health is still abnormal\n");
		return 0;
	}

	return -EINVAL;
}

static int cm_feed_watchdog(struct charger_manager *cm)
{
	union power_supply_propval val;
	struct power_supply *psy;
	int err, i;

	if (!cm->desc->wdt_interval)
		return 0;

	for (i = 0; cm->desc->psy_charger_stat[i]; i++) {
		psy = power_supply_get_by_name(cm->desc->psy_charger_stat[i]);
		if (!psy) {
			dev_err(cm->dev, "Cannot find power supply \"%s\"\n",
				cm->desc->psy_charger_stat[i]);
			continue;
		}

		val.intval = cm->desc->wdt_interval;
		err = power_supply_set_property(psy,
						POWER_SUPPLY_PROP_FEED_WATCHDOG,
						&val);
		power_supply_put(psy);
		if (err)
			return err;
	}

	return 0;
}

enum cm_manager_jeita_status {
	STATUS_BELOW_T0 = 0,
	STATUS_T0_TO_T1,
	STATUS_T1_TO_T2,
	STATUS_T2_TO_T3,
	STATUS_T3_TO_T4,
	STATUS_ABOVE_T4,
};

static bool cm_manager_adjust_current(struct charger_manager *cm,
				      int jeita_status)
{
	struct charger_desc *desc = cm->desc;
	int term_volt = 0, target_cur = 0;

#ifdef CONFIG_VENDOR_POWER_VOTER
	struct charger_policy *policy = &(cm->policy);
#else
	union power_supply_propval val;
	struct power_supply *psy = NULL;
	int i = 0;
	int ret = -ENODEV;
#endif

	if (cm->charging_status != 0 &&
	    !(cm->charging_status & (CM_CHARGE_TEMP_OVERHEAT | CM_CHARGE_TEMP_COLD))) {
		dev_info(cm->dev, "%s: charging_status: 0x%x, abnormal but not temp abnormal, do nothing\n",
					__func__, cm->charging_status);
		return true;
	}

	if (jeita_status > desc->jeita_tab_size)
		jeita_status = desc->jeita_tab_size;

	if (jeita_status == 0 || jeita_status == desc->jeita_tab_size) {
		dev_warn(cm->dev,
			 "stop charging due to battery overheat or cold\n");
		uevent_notify(cm, "Discharging");
		try_charger_enable(cm, false);

		if (jeita_status == 0)
			cm->charging_status |= CM_CHARGE_TEMP_COLD;
		else
			cm->charging_status |= CM_CHARGE_TEMP_OVERHEAT;

		return false;
	}

	term_volt = desc->jeita_tab[jeita_status].term_volt;
	target_cur = desc->jeita_tab[jeita_status].current_ua;

#ifndef CONFIG_VENDOR_POWER_VOTER
	if (cm->desc->thm_adjust_cur >= 0 &&
	    cm->desc->thm_adjust_cur < target_cur) {
		target_cur = cm->desc->thm_adjust_cur;
		dev_info(cm->dev, "thermal current %d is less than jeita current %d\n",
			cm->desc->thm_adjust_cur, desc->jeita_tab[jeita_status].current_ua);
	}
#endif

	dev_info(cm->dev, "jeita: target terminate voltage = %d, target current = %d\n",
		 term_volt, target_cur);

#ifdef CONFIG_VENDOR_POWER_VOTER
		vote(policy->fcv_votable, JEITA_SETTING_VOTER, true, term_volt);
		vote(policy->fcc_votable, JEITA_SETTING_VOTER, true, target_cur);
#else
	for (i = 0; cm->desc->psy_charger_stat[i]; i++) {
		psy = power_supply_get_by_name(cm->desc->psy_charger_stat[i]);
		if (!psy) {
			dev_err(cm->dev, "Cannot find power supply \"%s\"\n",
				cm->desc->psy_charger_stat[i]);
			continue;
		}

		val.intval = term_volt;
		ret = power_supply_set_property(psy,
					POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
					&val);
		if (ret) {
			power_supply_put(psy);
			dev_err(cm->dev,
				"failed to set terminate voltage, ret = %d\n",
				ret);
			continue;
		}

		val.intval = target_cur;
		ret = power_supply_set_property(psy,
					POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
					&val);
		power_supply_put(psy);
		if (ret) {
			dev_err(cm->dev,
				"failed to set charge current, ret = %d\n",
				ret);
			continue;
		}
	}

	if (ret)
		return false;
#endif


	try_charger_enable(cm, true);
	cm->charging_status &= ~(CM_CHARGE_TEMP_OVERHEAT | CM_CHARGE_TEMP_COLD);
	return true;
}

static int cm_manager_get_jeita_status(struct charger_manager *cm, int cur_temp)
{
	struct charger_desc *desc = cm->desc;
	static int jeita_status;
	int i;

	for (i = desc->jeita_tab_size - 1; i >= 0; i--) {
		if ((cur_temp >= desc->jeita_tab[i].temp && i > 0) ||
		    (cur_temp > desc->jeita_tab[i].temp && i == 0)) {
			break;
		}
	}

	switch (i) {
	case 4:
		jeita_status = STATUS_ABOVE_T4;
		break;
	case 3:
		if (jeita_status != STATUS_ABOVE_T4 ||
		    cur_temp <= desc->jeita_tab[4].recovery_temp)
			jeita_status = STATUS_T3_TO_T4;
		break;

	case 2:
		if ((jeita_status != STATUS_T3_TO_T4 ||
		     cur_temp <= desc->jeita_tab[3].recovery_temp) &&
		    (jeita_status != STATUS_T1_TO_T2 ||
		     cur_temp >= desc->jeita_tab[2].recovery_temp))
			jeita_status = STATUS_T2_TO_T3;
		break;

	case 1:
		if (jeita_status != STATUS_T0_TO_T1 ||
		     cur_temp >= desc->jeita_tab[1].recovery_temp)
			jeita_status = STATUS_T1_TO_T2;
		break;

	case 0:
		if (jeita_status != STATUS_BELOW_T0 ||
		    cur_temp >= desc->jeita_tab[0].recovery_temp)
			jeita_status = STATUS_T0_TO_T1;
		break;

	default:
		jeita_status = STATUS_BELOW_T0;
		break;
	}

	return jeita_status;
}

static int cm_manager_jeita_current_monitor(struct charger_manager *cm)
{
	struct charger_desc *desc = cm->desc;
	static int last_jeita_status = -1, temp_up_trigger, temp_down_trigger;
	int cur_jeita_status;
	static bool is_normal = true;
	bool is_charger_online = false;

	if (!desc->jeita_tab_size)
		return 0;

	is_charger_online = is_ext_pwr_online(cm);

	if (desc->jeita_disabled) {
		if (last_jeita_status != STATUS_T2_TO_T3) {
			dev_info(cm->dev, "Disable jeita and force jeita state to STATUS_T2_TO_T3\n");
			last_jeita_status = STATUS_T2_TO_T3;
			desc->thm_adjust_cur = -EINVAL;
			if (is_charger_online)
				cm_manager_adjust_current(cm, last_jeita_status);
		}

		return 0;
	}

	cur_jeita_status = cm_manager_get_jeita_status(cm, desc->temperature);

	dev_info(cm->dev, "current-last jeita status: %d-%d, current temperature: %d, chg online: %d\n",
		 cur_jeita_status, last_jeita_status, desc->temperature, is_charger_online);

	if (!is_charger_online)
		return 0;

	/*
	 * We should give a initial jeita status with adjusting the charging
	 * current when pluging in the cabel.
	 */
	if (last_jeita_status == -1) {
		is_normal = cm_manager_adjust_current(cm, cur_jeita_status);
		last_jeita_status = cur_jeita_status;
		goto out;
	}

	if (cur_jeita_status > last_jeita_status) {
		temp_down_trigger = 0;

		if (++temp_up_trigger >= 1) {
			is_normal = cm_manager_adjust_current(cm,
							      cur_jeita_status);
			last_jeita_status = cur_jeita_status;
		}
	} else if (cur_jeita_status < last_jeita_status) {
		temp_up_trigger = 0;

		if (++temp_down_trigger >= 1) {
			is_normal = cm_manager_adjust_current(cm,
							      cur_jeita_status);
			last_jeita_status = cur_jeita_status;
		}
	} else {
		temp_up_trigger = 0;
		temp_down_trigger = 0;
	}

out:
	if (!is_normal) {
		dev_info(cm->dev, "%s: is_normal: %d, charging_status: 0x%x\n",
					__func__, is_normal, cm->charging_status);
		return -EAGAIN;
	}

	return 0;
}

#ifdef CONFIG_VENDOR_POWER_VOTER
#define NEED_PARALLEL_CHG_CURRENT_UA	1500000

static bool should_enable_parallel_charger(struct charger_manager *cm)
{
	struct charger_desc *desc = cm->desc;
	int uV, uA;
	int ret;
	static unsigned int count = 0;

	if (is_ext_pwr_online(cm) == false) {
		dev_info(cm->dev, "%s: charger is not online, return false\n", __func__);
		return false;
	}

	if (charger_policy_get_status() == true) {
		dev_info(cm->dev, "%s: policy is enabled, return false\n", __func__);
		return false;
	}

	if (!is_batt_present(cm)) {
		dev_info(cm->dev, "%s: battery is not present, return false\n", __func__);
		return false;
	}

	if (cm->emergency_stop) {
		dev_info(cm->dev, "%s: emergency_stop, return false\n", __func__);
		return false;
	}

	if (!cm->desc->psy_hardware2) {
		dev_info(cm->dev, "%s: psy_hardware2 is NULL, return false\n", __func__);
		return false;
	}

	if (cm->policy.usb_phy->chg_type == SDP_TYPE || cm->policy.usb_phy->chg_type == UNKNOWN_TYPE) {
		dev_info(cm->dev, "%s: chg_type is %d, return false\n", __func__, cm->policy.usb_phy->chg_type);
		return false;
	}

	if (desc->fullbatt_uV <= 0) {
		dev_info(cm->dev, "%s: fullbatt_uV is invalid %d, return false\n", __func__, desc->fullbatt_uV);
		return false;
	}

	if (cm_manager_get_jeita_status(cm, desc->temperature) != STATUS_T2_TO_T3) {
		dev_info(cm->dev, "%s: current jeita status is unnormal, return false\n", __func__);
		return false;
	}

	if (get_client_vote(cm->policy.fcc_votable, THERMAL_SETTING_VOTER) < NEED_PARALLEL_CHG_CURRENT_UA
		&& get_client_vote(cm->policy.fcc_votable, THERMAL_SETTING_VOTER) > 0) {
		dev_info(cm->dev, "%s: thermal limit below %dmA, return false\n", __func__,
						NEED_PARALLEL_CHG_CURRENT_UA / 1000);
		return false;
	}

	ret = get_batt_uV(cm, &uV);
	if (ret)
		goto out;

	ret = get_batt_uA(cm, &uA);
	if (ret)
		goto out;

	if (uV >= desc->fullbatt_uV && uA < NEED_PARALLEL_CHG_CURRENT_UA)
		count++;
	else
		count = 0;

	if (count > 3) {
		dev_info(cm->dev, "%s: ### no need paraller charging, vbat %d, ibat %d, count %d, return false\n",
			__func__, uV, uA, count);
		return false;
	}

	dev_info(cm->dev, "%s: need paraller charging, vbat %d, ibat %d, count %d, return true\n",
			__func__, uV, uA, count);
	return true;

out:
	dev_info(cm->dev, "%s: error occur, return false\n", __func__);
	return false;
}

#endif

/**
 * _cm_monitor - Monitor the temperature and return true for exceptions.
 * @cm: the Charger Manager representing the battery.
 *
 * Returns true if there is an event to notify for the battery.
 * (True if the status of "emergency_stop" changes)
 */
static bool _cm_monitor(struct charger_manager *cm)
{
	int temp_alrt = 0, ret;
	int i;
#ifdef CONFIG_VENDOR_POWER_VOTER
	struct charger_policy *policy = &(cm->policy);
#endif

	/* Feed the charger watchdog if necessary */
	ret = cm_feed_watchdog(cm);
	if (ret) {
		dev_warn(cm->dev, "Failed to feed charger watchdog\n");
		return false;
	}

	for (i = 0; i < cm->desc->num_charger_regulators; i++) {
		if (cm->desc->charger_regulators[i].externally_control) {
			dev_info(cm->dev,
				 "Charger has been controlled externally, so no need monitoring\n");
			return false;
		}
	}

	temp_alrt = cm_check_thermal_status(cm);

	/* It has been stopped already */
	if (temp_alrt && cm->emergency_stop) {
		dev_warn(cm->dev,
			 "Emergency stop, temperature alert = %d\n", temp_alrt);
		return false;
	}

#ifdef CONFIG_VENDOR_POWER_VOTER
	cm->should_en_parallel_chg = should_enable_parallel_charger(cm);

	if (cm->parallel_charge_enabled && !cm->should_en_parallel_chg) {
		rerun_election(policy->fcv_votable);
		rerun_election(policy->fcc_votable);
		rerun_election(policy->usb_icl_votable);
		rerun_election(policy->topoff_votable);
		rerun_election(policy->battery_charging_enabled_votable);
		cm->parallel_charge_enabled = false;
	} else if (!cm->parallel_charge_enabled && cm->should_en_parallel_chg) {
		rerun_election(policy->fcv_votable);
		rerun_election(policy->fcc_votable);
		rerun_election(policy->usb_icl_votable);
		rerun_election(policy->topoff_votable);
		rerun_election(policy->battery_charging_enabled_votable);
		cm->parallel_charge_enabled = true;
	}
#endif
	/*
	 * Adjust the charging current according to current battery
	 * temperature jeita table.
	 */
	ret = cm_manager_jeita_current_monitor(cm);
	if (ret) {
		dev_warn(cm->dev,
			 "Errors orrurs when adjusting charging current\n");
		return false;
	}

	/*
	 * Check temperature whether overheat or cold.
	 * If temperature is out of range normal state, stop charging.
	 */
	if (temp_alrt) {
		cm->emergency_stop = temp_alrt;
		dev_info(cm->dev,
			"Temperature is out of range normal state, stop charging\n");
		if (!try_charger_enable(cm, false))
			uevent_notify(cm, default_event_names[temp_alrt]);
	/*
	 * Check if the charge voltage is in the normal range.
	 */
	} else if (!cm->emergency_stop && !cm_check_charge_voltage(cm)) {
		dev_info(cm->dev,
			"Stop charging/Recharging due to charge voltage changes\n");
	/*
	 * Check if the charge health is in the normal mode.
	 */
	} else if (!cm->emergency_stop && !cm_check_charge_health(cm)) {
		dev_info(cm->dev,
			"Stop charging/Recharging due to charge health changes\n");
	/*
	 * Check whole charging duration and discharing duration
	 * after full-batt.
	 */
	} else if (!cm->emergency_stop && check_charging_duration(cm)) {
		dev_info(cm->dev,
			"Charging/Discharging duration is out of range\n");
	/*
	 * Check dropped voltage of battery. If battery voltage is more
	 * dropped than fullbatt_vchkdrop_uV after fully charged state,
	 * charger-manager have to recharge battery.
	 */
	} else if (!cm->emergency_stop && is_ext_pwr_online(cm) &&
			!cm->charger_enabled) {
		dev_info(cm->dev, "Check dropped voltage of battery\n");
		fullbatt_vchk(&cm->fullbatt_vchk_work.work);

	/*
	 * Check whether fully charged state to protect overcharge
	 * if charger-manager is charging for battery.
	 */
	} else if (!cm->emergency_stop && is_full_charged(cm) &&
			cm->charger_enabled) {
		dev_info(cm->dev, "EVENT_HANDLE: Battery Fully Charged\n");
		uevent_notify(cm, default_event_names[CM_EVENT_BATT_FULL]);

		try_charger_enable(cm, false);
		power_supply_changed(cm->charger_psy);

		fullbatt_vchk(&cm->fullbatt_vchk_work.work);
	} else {
		cm->emergency_stop = 0;
		cm->charging_status = 0;
		if (is_ext_pwr_online(cm)) {
			dev_info(cm->dev, "No emergency stop, charging\n");
			if (!try_charger_enable(cm, true))
				uevent_notify(cm, "CHARGING");
		}
	}

	return true;
}

/**
 * cm_monitor - Monitor every battery.
 *
 * Returns true if there is an event to notify from any of the batteries.
 * (True if the status of "emergency_stop" changes)
 */
static bool cm_monitor(void)
{
	bool stop = false;
	struct charger_manager *cm;

	mutex_lock(&cm_list_mtx);

	list_for_each_entry(cm, &cm_list, entry) {
		if (_cm_monitor(cm))
			stop = true;
	}

	mutex_unlock(&cm_list_mtx);

	return stop;
}

/**
 * _setup_polling - Setup the next instance of polling.
 * @work: work_struct of the function _setup_polling.
 */
static void _setup_polling(struct work_struct *work)
{
	unsigned long min = ULONG_MAX;
	struct charger_manager *cm;
	bool keep_polling = false;
	unsigned long _next_polling;

	mutex_lock(&cm_list_mtx);

	list_for_each_entry(cm, &cm_list, entry) {
		if (is_polling_required(cm) && cm->desc->polling_interval_ms) {
			keep_polling = true;

			if (min > cm->desc->polling_interval_ms)
				min = cm->desc->polling_interval_ms;
		}
	}

	polling_jiffy = msecs_to_jiffies(min);
	if (polling_jiffy <= CM_JIFFIES_SMALL)
		polling_jiffy = CM_JIFFIES_SMALL + 1;

	if (!keep_polling)
		polling_jiffy = ULONG_MAX;
	if (polling_jiffy == ULONG_MAX)
		goto out;

	WARN(cm_wq == NULL, "charger-manager: workqueue not initialized"
			    ". try it later. %s\n", __func__);

	/*
	 * Use mod_delayed_work() iff the next polling interval should
	 * occur before the currently scheduled one.  If @cm_monitor_work
	 * isn't active, the end result is the same, so no need to worry
	 * about stale @next_polling.
	 */
	_next_polling = jiffies + polling_jiffy;

	if (time_before(_next_polling, next_polling)) {
		mod_delayed_work(cm_wq, &cm_monitor_work, polling_jiffy);
		next_polling = _next_polling;
	} else {
		if (queue_delayed_work(cm_wq, &cm_monitor_work, polling_jiffy))
			next_polling = _next_polling;
	}
out:
	mutex_unlock(&cm_list_mtx);
}
static DECLARE_WORK(setup_polling, _setup_polling);

/**
 * cm_monitor_poller - The Monitor / Poller.
 * @work: work_struct of the function cm_monitor_poller
 *
 * During non-suspended state, cm_monitor_poller is used to poll and monitor
 * the batteries.
 */
static void cm_monitor_poller(struct work_struct *work)
{
	cm_monitor();
	schedule_work(&setup_polling);
}

/**
 * fullbatt_handler - Event handler for CM_EVENT_BATT_FULL
 * @cm: the Charger Manager representing the battery.
 */
static void fullbatt_handler(struct charger_manager *cm)
{
	struct charger_desc *desc = cm->desc;

	if (!desc->fullbatt_vchkdrop_uV || !desc->fullbatt_vchkdrop_ms)
		goto out;

	if (cm_suspended)
		device_set_wakeup_capable(cm->dev, true);

	mod_delayed_work(cm_wq, &cm->fullbatt_vchk_work,
			 msecs_to_jiffies(desc->fullbatt_vchkdrop_ms));
	cm->fullbatt_vchk_jiffies_at = jiffies + msecs_to_jiffies(
				       desc->fullbatt_vchkdrop_ms);

	if (cm->fullbatt_vchk_jiffies_at == 0)
		cm->fullbatt_vchk_jiffies_at = 1;

out:
	dev_info(cm->dev, "EVENT_HANDLE: Battery Fully Charged\n");
	uevent_notify(cm, default_event_names[CM_EVENT_BATT_FULL]);
}

/**
 * battout_handler - Event handler for CM_EVENT_BATT_OUT
 * @cm: the Charger Manager representing the battery.
 */
static void battout_handler(struct charger_manager *cm)
{
	if (cm_suspended)
		device_set_wakeup_capable(cm->dev, true);

	if (!is_batt_present(cm)) {
		dev_emerg(cm->dev, "Battery Pulled Out!\n");
		try_charger_enable(cm, false);
		uevent_notify(cm, default_event_names[CM_EVENT_BATT_OUT]);
	} else {
		dev_emerg(cm->dev, "Battery Pulled in!\n");

		if (cm->charging_status) {
			dev_emerg(cm->dev, "Charger status abnormal, stop charge!\n");
			try_charger_enable(cm, false);
		} else {
			try_charger_enable(cm, true);
		}

		uevent_notify(cm, default_event_names[CM_EVENT_BATT_IN]);
	}
}

static bool cm_charger_is_support_fchg(struct charger_manager *cm)
{
	struct charger_desc *desc = cm->desc;
	struct power_supply *psy;
	union power_supply_propval val;
	int ret, i;

	if (!desc->psy_fast_charger_stat)
		return false;

	for (i = 0; desc->psy_fast_charger_stat[i]; i++) {
		psy = power_supply_get_by_name(desc->psy_fast_charger_stat[i]);

		if (!psy) {
			dev_err(cm->dev, "Cannot find power supply \"%s\"\n",
				desc->psy_fast_charger_stat[i]);
			continue;
		}

		ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CHARGE_TYPE,
						&val);
		power_supply_put(psy);
		if (!ret)
			return val.intval == POWER_SUPPLY_CHARGE_TYPE_FAST;
	}

	return false;
}

/**
 * fast_charge_handler - Event handler for CM_EVENT_FAST_CHARGE
 * @cm: the Charger Manager representing the battery.
 */
static void fast_charge_handler(struct charger_manager *cm)
{
	if (cm_suspended)
		device_set_wakeup_capable(cm->dev, true);

	if (!is_ext_pwr_online(cm))
		return;

	if (cm_charger_is_support_fchg(cm)) {
		cm->desc->charge_voltage_max =
			cm->desc->fast_charge_voltage_max;
		cm->desc->charge_voltage_drop =
			cm->desc->fast_charge_voltage_drop;
		if (!cm->desc->disable_charger_type_jeita)
			cm->desc->jeita_tab =
				cm->desc->jeita_tab_array[CM_JEITA_FCHG];
	}
}

/**
 * misc_event_handler - Handler for other evnets
 * @cm: the Charger Manager representing the battery.
 * @type: the Charger Manager representing the battery.
 */
static void misc_event_handler(struct charger_manager *cm,
			enum cm_event_types type)
{
	int ret;

	if (cm_suspended)
		device_set_wakeup_capable(cm->dev, true);

	if (cm->emergency_stop)
		cm->emergency_stop = 0;

	if (cm->charging_status)
		cm->charging_status = 0;

	cm->desc->thm_adjust_cur = -EINVAL;

	if (is_ext_pwr_online(cm)) {
		try_charger_enable(cm, true);

		if (!cm->desc->disable_charger_type_jeita) {
			ret = get_charger_type(cm, &cm->desc->charger_type);
			if (ret)
				return;

			switch (cm->desc->charger_type) {
			case POWER_SUPPLY_USB_TYPE_DCP:
				cm->desc->jeita_tab =
					cm->desc->jeita_tab_array[CM_JEITA_DCP];
				break;

			case POWER_SUPPLY_USB_TYPE_SDP:
				cm->desc->jeita_tab =
					cm->desc->jeita_tab_array[CM_JEITA_SDP];
				break;

			case POWER_SUPPLY_USB_TYPE_CDP:
				cm->desc->jeita_tab =
					cm->desc->jeita_tab_array[CM_JEITA_CDP];
				break;

			default:
				cm->desc->jeita_tab =
					cm->desc->jeita_tab_array[CM_JEITA_UNKNOWN];
			}
		}

		cm->desc->charge_voltage_max =
			cm->desc->normal_charge_voltage_max;
		cm->desc->charge_voltage_drop =
			cm->desc->normal_charge_voltage_drop;

		if (cm->desc->jeita_tab_size) {
			int cur_jeita_status;

			cur_jeita_status =
				cm_manager_get_jeita_status(cm, cm->desc->temperature);
			cm_manager_adjust_current(cm, cur_jeita_status);
		}
	} else {
		try_charger_enable(cm, false);
	}

	cm_update_charger_type_status(cm);

	if (cm->desc->force_set_full)
		cm->desc->force_set_full = false;

	if (is_polling_required(cm) && cm->desc->polling_interval_ms)
		schedule_work(&setup_polling);
	uevent_notify(cm, default_event_names[type]);
}

/*
static int wireless_get_property(struct power_supply *psy, enum power_supply_property
				 psp, union power_supply_propval *val)
{
	int ret = 0;
	struct wireless_data *data = container_of(psy->desc, struct  wireless_data, psd);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = data->WIRELESS_ONLINE;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int ac_get_property(struct power_supply *psy, enum power_supply_property psp,
			   union power_supply_propval *val)
{
	int ret = 0;
	struct ac_data *data = container_of(psy->desc, struct ac_data, psd);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = data->AC_ONLINE;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int usb_get_property(struct power_supply *psy, enum power_supply_property psp,
			    union power_supply_propval *val)
{
	int ret = 0;
	struct usb_data *data = container_of(psy->desc, struct usb_data, psd);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = data->USB_ONLINE;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}*/

int charger_manager_get_ship_mode(struct charger_manager *cm)
{
	struct power_supply *psy = NULL;
	union power_supply_propval val = {0,};
	int rc = 1;

	if (cm && cm->desc && cm->desc->psy_hardware) {
		psy = power_supply_get_by_name(cm->desc->psy_hardware);
		if (!psy) {
			dev_err(cm->dev, "no %s psy!\n", cm->desc->psy_hardware);
			return rc;
		}
	} else {
		dev_err(cm->dev, "no psy_hardware node!\n", rc);
		return rc;
	}

	rc = power_supply_get_property(psy,
			POWER_SUPPLY_PROP_SET_SHIP_MODE, &val);
	if (rc < 0) {
		dev_err(cm->dev, "get ship mode failed, rc = %d\n", rc);
		power_supply_put(psy);
		return rc;
	}

	dev_info(cm->dev, "%s get_ship_mode, val %d\n", __func__, val.intval);
	power_supply_put(psy);

	return val.intval;
}

int charger_manager_set_ship_mode(struct charger_manager *cm, int enable)
{
	union power_supply_propval val = {0,};
	int rc = 0;

	val.intval = enable;

	if (cm && cm->desc && cm->desc->psy_hardware) {
		charger_set_prop_by_name(cm->desc->psy_hardware,
					POWER_SUPPLY_PROP_SET_SHIP_MODE, val.intval);
		if (cm->desc->psy_hardware2)
			charger_set_prop_by_name(cm->desc->psy_hardware2,
					POWER_SUPPLY_PROP_SET_SHIP_MODE, val.intval);
	} else {
		dev_err(cm->dev, "no psy_hardware node!\n", rc);
		return -ENODEV;
	}

	dev_info(cm->dev, "%s set_ship_mode, val %d\n", __func__, val.intval);

	return 0;
}

#define CUTOFF_VOLTAGE_UV 3400000

static int charger_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct charger_manager *cm = power_supply_get_drvdata(psy);
	struct power_supply *fuel_gauge = NULL;
	unsigned int total_cap = 0;
	int chg_cur = 0;
	int ret = 0;
	int i;
	int cap_new, batt_uV = 0;
	bool is_full = false;
#ifdef CONFIG_VENDOR_POWER_VOTER
	struct charger_policy *policy = &(cm->policy);
#endif

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		is_full = is_full_charged(cm);
		if (is_charging(cm) && !is_full) {
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		} else if (is_ext_pwr_online(cm)) {
			if (is_full || cm->desc->force_set_full)
				val->intval = POWER_SUPPLY_STATUS_FULL;
			else
				val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		} else {
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		}

		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = cm->health;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		if (is_batt_present(cm))
			val->intval = 1;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		ret = get_batt_uV(cm, &val->intval);
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		fuel_gauge = power_supply_get_by_name(cm->desc->psy_fuel_gauge);
		if (!fuel_gauge) {
			ret = -ENODEV;
			break;
		}
		ret = power_supply_get_property(fuel_gauge,
				POWER_SUPPLY_PROP_CURRENT_AVG, val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = get_batt_vol_now(cm, &val->intval);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = get_batt_cur_now(cm, &val->intval);
		#ifdef ZTE_FEATURE_PV_AR
		if (!ret)
			val->intval *= (-1);
		#endif
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = cm->desc->temperature;
		break;
	case POWER_SUPPLY_PROP_TEMP_AMBIENT:
		return cm_get_battery_temperature(cm, &val->intval);
	case POWER_SUPPLY_PROP_CAPACITY:
		if (!is_batt_present(cm)) {
			/* There is no battery. Assume 100% */
			val->intval = 100;
			break;
		}

		if (cm->desc->cap_debug != -1)
			val->intval = cm->desc->cap_debug;
		else {
			/* Raise cap a bit, default by *101/96 + 0.5, the affect is real 94.6% raised to 100%*/
			cap_new  = cm->desc->cap;
			cap_new = DIV_ROUND_CLOSEST(cap_new * 101, cm->desc->fullbatt_advance_level);
			dev_info(cm->dev, "capacity orig: %d, new: %d\n", cm->desc->cap, cap_new);

			if (cap_new > 100)
				cap_new = 100;

			if (cm->desc->cap <= 0) {
				get_batt_vol_now(cm, &batt_uV);
				if (batt_uV >= CUTOFF_VOLTAGE_UV) {
					cap_new = 1;
					dev_info(cm->dev, "## force report soc 1, vbat %d\n", batt_uV);
				}
			}
			val->intval = cap_new;
		}
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		if (!is_batt_present(cm)) {
			/* There is no battery. Assume 100% */
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
			break;
		}
		val->intval = DIV_ROUND_CLOSEST(cm->desc->cap, 10);
		if (val->intval > 100)
			val->intval = 100;
		else if (val->intval < 0)
			val->intval = 0;

		if (val->intval == CM_CAPACITY_LEVEL_FULL)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
		else if (val->intval > CM_CAPACITY_LEVEL_NORMAL)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_HIGH;
		else if (val->intval > CM_CAPACITY_LEVEL_LOW)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
		else if (val->intval > CM_CAPACITY_LEVEL_CRITICAL)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
		else
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		if (is_ext_pwr_online(cm))
			val->intval = 1;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		fuel_gauge = power_supply_get_by_name(
					cm->desc->psy_fuel_gauge);
		if (!fuel_gauge) {
			ret = -ENODEV;
			break;
		}

		ret = power_supply_get_property(fuel_gauge,
						POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN,
						val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		if (is_charging(cm)) {
			fuel_gauge = power_supply_get_by_name(
					cm->desc->psy_fuel_gauge);
			if (!fuel_gauge) {
				ret = -ENODEV;
				break;
			}

			ret = power_supply_get_property(fuel_gauge,
						POWER_SUPPLY_PROP_CHARGE_NOW,
						val);
			if (ret) {
				val->intval = 1;
				ret = 0;
			} else {
				/* If CHARGE_NOW is supplied, use it */
				val->intval = (val->intval > 0) ?
						val->intval : 1;
			}
		} else {
			val->intval = 0;
		}
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		for (i = 0; cm->desc->psy_charger_stat[i]; i++) {
			psy = power_supply_get_by_name(
					cm->desc->psy_charger_stat[i]);
			if (!psy) {
				dev_err(cm->dev, "Cannot find power supply \"%s\"\n",
					cm->desc->psy_charger_stat[i]);
				continue;
			}

			ret = power_supply_get_property(psy,
				POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT, val);
			power_supply_put(psy);
			if (ret) {
				dev_err(cm->dev, "set charge current failed\n");
				continue;
			}
		}
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		for (i = 0; cm->desc->psy_charger_stat[i]; i++) {
			psy = power_supply_get_by_name(
					cm->desc->psy_charger_stat[i]);
			if (!psy) {
				dev_err(cm->dev, "Cannot find power supply \"%s\"\n",
					cm->desc->psy_charger_stat[i]);
				continue;
			}

			ret = power_supply_get_property(psy,
				POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, val);
			power_supply_put(psy);
			if (ret) {
				dev_err(cm->dev, "set charge limit current failed\n");
				continue;
			}
		}
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		fuel_gauge = power_supply_get_by_name(
					cm->desc->psy_fuel_gauge);
		if (!fuel_gauge) {
			ret = -ENODEV;
			break;
		}

		ret = power_supply_get_property(fuel_gauge,
						POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN,
						val);
		if (ret) {
			val->intval = 1;
			ret = 0;
		} else {
			/* If CHARGE_COUNTER is supplied, use it */
			val->intval = val->intval > 0 ? (val->intval / 1000) : 1;
			val->intval = (cm->desc->cap * val->intval);
		}
		break;

	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
#ifdef CONFIG_VENDOR_POWER_VOTER
		val->intval =  get_client_vote(policy->fcc_votable, BATTERY_SETTING_VOTER);
#else
		for (i = 0; cm->desc->psy_charger_stat[i]; i++) {
			psy = power_supply_get_by_name(cm->desc->psy_charger_stat[i]);
			if (!psy) {
				dev_err(cm->dev, "Cannot find power supply \"%s\"\n",
					cm->desc->psy_charger_stat[i]);
				continue;
			}

			ret = power_supply_get_property(psy,
							POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
							val);
			if (!ret) {
				power_supply_put(psy);
				break;
			}

			ret = power_supply_get_property(psy,
							POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
							val);
			if (!ret) {
				power_supply_put(psy);
				break;
			}
		}
#endif
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = 0;
		fuel_gauge = power_supply_get_by_name(
					cm->desc->psy_fuel_gauge);
		if (!fuel_gauge) {
			ret = -ENODEV;
			break;
		}

		ret = power_supply_get_property(fuel_gauge,
						POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN,
						val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		ret = get_charger_current(cm, &chg_cur);
		if (ret) {
			dev_err(cm->dev, "get chg_cur error.\n");
			break;
		}
		chg_cur = chg_cur / 1000;

		ret = get_batt_total_cap(cm, &total_cap);
		if (ret) {
			dev_err(cm->dev, "failed to get total cap.\n");
			break;
		}
		total_cap = total_cap / 1000;

		val->intval =
			((1000 - cm->desc->cap) * total_cap / 1000) * 3600 / chg_cur;

		break;
	case POWER_SUPPLY_PROP_SET_SHIP_MODE:
		val->intval = charger_manager_get_ship_mode(cm);
		break;
	case POWER_SUPPLY_PROP_BATTERY_ID:
		fuel_gauge = power_supply_get_by_name(
			cm->desc->psy_fuel_gauge);
		if (!fuel_gauge) {
			ret = -ENODEV;
			break;
		}
		ret = power_supply_get_property(fuel_gauge,
				POWER_SUPPLY_PROP_BATTERY_ID,
				val);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
#ifdef CONFIG_VENDOR_POWER_VOTER
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = get_client_vote(policy->usb_icl_votable, POLICY_SETTING_VOTER);
		val->intval = (val->intval < 0) ? 1 : 0;
		break;
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		val->intval = get_client_vote(policy->battery_charging_enabled_votable, POLICY_SETTING_VOTER);
		val->intval = (val->intval < 0) ? 1 : 0;
		break;
#endif
	default:
		return -EINVAL;
	}
	if (fuel_gauge)
		power_supply_put(fuel_gauge);
	return ret;
}

static int
charger_set_property(struct power_supply *psy,
		     enum power_supply_property psp,
		     const union power_supply_propval *val)
{
	struct charger_manager *cm = power_supply_get_drvdata(psy);
	union power_supply_propval thermal_val;
	int ret = 0;
	int i;
#ifdef CONFIG_VENDOR_POWER_VOTER
	struct charger_policy *policy = &(cm->policy);
#else
	int cur_jeita_status;
#endif

	if (psp != POWER_SUPPLY_PROP_SET_SHIP_MODE && !is_ext_pwr_online(cm))
		return -ENODEV;

	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		for (i = 0; cm->desc->psy_charger_stat[i]; i++) {
			psy = power_supply_get_by_name(
					cm->desc->psy_charger_stat[i]);
			if (!psy) {
				dev_err(cm->dev, "Cannot find power supply \"%s\"\n",
					cm->desc->psy_charger_stat[i]);
				continue;
			}

			ret = power_supply_set_property(psy,
				POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT, val);
			power_supply_put(psy);
			if (ret) {
				dev_err(cm->dev, "set charge current failed\n");
				continue;
			}
		}
		break;

	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		for (i = 0; cm->desc->psy_charger_stat[i]; i++) {
			psy = power_supply_get_by_name(
					cm->desc->psy_charger_stat[i]);
			if (!psy) {
				dev_err(cm->dev, "Cannot find power supply \"%s\"\n",
					cm->desc->psy_charger_stat[i]);
				continue;
			}

			ret = power_supply_set_property(psy,
				POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, val);
			power_supply_put(psy);
			if (ret) {
				dev_err(cm->dev, "set charge limit current failed\n");
				continue;
			}
		}
		break;

	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		if (!cm->desc->thermal_control_en) {
			dev_info(cm->dev, "%s thermal control disabled\n", __func__);
			break;
		}

		if (val->intval < 0) {
			dev_info(cm->dev, "%s thermal control invalid argument\n", __func__);
			break;
		}

		cm->desc->thm_adjust_cur = val->intval;
		thermal_val.intval = val->intval;

#ifdef CONFIG_VENDOR_POWER_VOTER
#ifdef CONFIG_CHARGER_BQ2560X
		if (cm->desc->thermal_limit == THERMAL_LIMIT_IN) {
			vote(policy->usb_icl_votable, THERMAL_SETTING_VOTER, true, val->intval);
		} else if (cm->desc->thermal_limit == THERMAL_LIMIT_OUT) {
			vote(policy->fcc_votable, THERMAL_SETTING_VOTER, true, val->intval);
		} else {
			vote(policy->usb_icl_votable, THERMAL_SETTING_VOTER, true, val->intval);
			vote(policy->fcc_votable, THERMAL_SETTING_VOTER, true, val->intval);
		}
		dev_info(cm->dev, "%s thermal control,vote: usb_icl %d, fcc %d.\n",
					__func__, val->intval, val->intval);
#else
		vote(policy->usb_icl_votable, THERMAL_SETTING_VOTER, true, val->intval);
		vote(policy->fcc_votable, THERMAL_SETTING_VOTER, true, val->intval);
		dev_info(cm->dev, "%s thermal control fcc and icl vote: %d\n", __func__, val->intval);
#endif
#else
		for (i = 0; cm->desc->psy_charger_stat[i]; i++) {
			psy = power_supply_get_by_name(cm->desc->psy_charger_stat[i]);
			if (!psy) {
				dev_err(cm->dev, "Cannot find power supply \"%s\"\n",
					cm->desc->psy_charger_stat[i]);
				continue;
			}

			ret = power_supply_set_property(psy,
							POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
							(const union power_supply_propval *)&thermal_val);
			if (!ret) {
				power_supply_put(psy);
				break;
			}

			if (cm->desc->jeita_tab_size) {
				cur_jeita_status = cm_manager_get_jeita_status(cm, cm->desc->temperature);
				if (val->intval > cm->desc->jeita_tab[cur_jeita_status].current_ua)
					thermal_val.intval = cm->desc->jeita_tab[cur_jeita_status].current_ua;
			}

			ret = power_supply_set_property(psy,
							POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
							(const union power_supply_propval *)&thermal_val);
			if (!ret) {
				power_supply_put(psy);
				break;
			}
		}
#endif
		break;
	case POWER_SUPPLY_PROP_SET_SHIP_MODE:
		charger_manager_set_ship_mode(cm, val->intval);
		break;
#ifdef CONFIG_VENDOR_POWER_VOTER
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		vote_debug("policy: Set charging enable %d\n", val->intval);
		vote(policy->usb_icl_votable, POLICY_SETTING_VOTER, (bool)!val->intval, 0);
		break;
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		vote_debug("policy: Set battery charging enable %d\n", val->intval);
		vote(policy->battery_charging_enabled_votable, POLICY_SETTING_VOTER,
							(val->intval ? false : true), 0);
		if (val->intval == 0) {
			charger_set_prop_by_name(cm->desc->psy_hardware,
				POWER_SUPPLY_PROP_SET_WATCHDOG_TIMER, 0);
			if (cm)
				cm->is_battery_charging_enabled = true;
		} else {
			charger_set_prop_by_name(cm->desc->psy_hardware,
				POWER_SUPPLY_PROP_SET_WATCHDOG_TIMER, cm->desc->wdt_interval);
			if (cm)
				cm->is_battery_charging_enabled = false;
		}

		break;
#endif
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int charger_property_is_writeable(struct power_supply *psy,
					 enum power_supply_property psp)
{
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
	case POWER_SUPPLY_PROP_SET_SHIP_MODE:
		/* fall through */
#ifdef CONFIG_VENDOR_POWER_VOTER
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		/* fall through */
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
#endif
		ret = 1;
		break;

	default:
		ret = 0;
	}

	return ret;
}
#define NUM_CHARGER_PSY_OPTIONAL	(4)

/*
static enum power_supply_property wireless_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};*/

static enum power_supply_property default_charger_props[] = {
	/* Guaranteed to provide */
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_SET_SHIP_MODE,
	POWER_SUPPLY_PROP_BATTERY_ID,
	POWER_SUPPLY_PROP_TECHNOLOGY,
#ifdef CONFIG_VENDOR_POWER_VOTER
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED,
#endif
	/*
	 * Optional properties are:
	 * POWER_SUPPLY_PROP_CHARGE_NOW,
	 * POWER_SUPPLY_PROP_CURRENT_NOW,
	 * POWER_SUPPLY_PROP_TEMP, and
	 * POWER_SUPPLY_PROP_TEMP_AMBIENT,
	 */
};

/* wireless_data initialization */
/*
static struct wireless_data wireless_main = {
	.psd = {
		.name = "wireless",
		.type =	POWER_SUPPLY_TYPE_WIRELESS,
		.properties = wireless_props,
		.num_properties = ARRAY_SIZE(wireless_props),
		.get_property = wireless_get_property,
	},
	.WIRELESS_ONLINE = 0,
};*/

/* ac_data initialization */
/*
static struct ac_data ac_main = {
	.psd = {
		.name = "ac",
		.type = POWER_SUPPLY_TYPE_MAINS,
		.properties = ac_props,
		.num_properties = ARRAY_SIZE(ac_props),
		.get_property = ac_get_property,
	},
	.AC_ONLINE = 0,
};
*/

/* usb_data initialization */
/*
static struct usb_data usb_main = {
	.psd = {
		.name = "usb",
		.type = POWER_SUPPLY_TYPE_USB,
		.properties = usb_props,
		.num_properties = ARRAY_SIZE(usb_props),
		.get_property = usb_get_property,
	},
	.USB_ONLINE = 0,
};*/

static const struct power_supply_desc psy_default = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = default_charger_props,
	.num_properties = ARRAY_SIZE(default_charger_props),
	.get_property = charger_get_property,
	.set_property = charger_set_property,
	.property_is_writeable	= charger_property_is_writeable,
	.no_thermal = true,
};

static void cm_update_charger_type_status(struct charger_manager *cm)
{
/*
	if (is_ext_pwr_online(cm)) {
		if (cm->desc->charger_type == POWER_SUPPLY_USB_TYPE_DCP) {
			wireless_main.WIRELESS_ONLINE = 0;
			usb_main.USB_ONLINE = 0;
			ac_main.AC_ONLINE = 1;
		} else {
			wireless_main.WIRELESS_ONLINE = 0;
			ac_main.AC_ONLINE = 0;
			usb_main.USB_ONLINE = 1;
		}
	} else {
		wireless_main.WIRELESS_ONLINE = 0;
		ac_main.AC_ONLINE = 0;
		usb_main.USB_ONLINE = 0;

	}
*/
	;
}

/**
 * cm_setup_timer - For in-suspend monitoring setup wakeup alarm
 *		    for suspend_again.
 *
 * Returns true if the alarm is set for Charger Manager to use.
 * Returns false if
 *	cm_setup_timer fails to set an alarm,
 *	cm_setup_timer does not need to set an alarm for Charger Manager,
 *	or an alarm previously configured is to be used.
 */
static bool cm_setup_timer(void)
{
	struct charger_manager *cm;
	unsigned int wakeup_ms = UINT_MAX;
	int timer_req = 0;

	if (time_after(next_polling, jiffies))
		CM_MIN_VALID(wakeup_ms,
			jiffies_to_msecs(next_polling - jiffies));

	mutex_lock(&cm_list_mtx);
	list_for_each_entry(cm, &cm_list, entry) {
		unsigned int fbchk_ms = 0;

		/* fullbatt_vchk is required. setup timer for that */
		if (cm->fullbatt_vchk_jiffies_at) {
			fbchk_ms = jiffies_to_msecs(cm->fullbatt_vchk_jiffies_at
						    - jiffies);
			if (time_is_before_eq_jiffies(
				cm->fullbatt_vchk_jiffies_at) ||
				msecs_to_jiffies(fbchk_ms) < CM_JIFFIES_SMALL) {
				fullbatt_vchk(&cm->fullbatt_vchk_work.work);
				fbchk_ms = 0;
			}
		}
		CM_MIN_VALID(wakeup_ms, fbchk_ms);

		/* Skip if polling is not required for this CM */
		if (!is_polling_required(cm) && !cm->emergency_stop)
			continue;
		timer_req++;
		if (cm->desc->polling_interval_ms == 0)
			continue;
		CM_MIN_VALID(wakeup_ms, cm->desc->polling_interval_ms);
	}
	mutex_unlock(&cm_list_mtx);

	if (timer_req && cm_timer) {
		ktime_t now, add;

		/*
		 * Set alarm with the polling interval (wakeup_ms)
		 * The alarm time should be NOW + CM_RTC_SMALL or later.
		 */
		if (wakeup_ms == UINT_MAX ||
			wakeup_ms < CM_RTC_SMALL * MSEC_PER_SEC)
			wakeup_ms = 2 * CM_RTC_SMALL * MSEC_PER_SEC;

		pr_info("Charger Manager wakeup timer: %u ms\n", wakeup_ms);

		now = ktime_get_boottime();
		add = ktime_set(wakeup_ms / MSEC_PER_SEC,
				(wakeup_ms % MSEC_PER_SEC) * NSEC_PER_MSEC);
		alarm_start(cm_timer, ktime_add(now, add));

		cm_suspend_duration_ms = wakeup_ms;

		return true;
	}
	return false;
}

/**
 * charger_extcon_notifier - receive the state of charger cable
 *			when registered cable is attached or detached.
 *
 * @self: the notifier block of the charger_extcon_notifier.
 * @event: the cable state.
 * @ptr: the data pointer of notifier block.
 */
static int charger_extcon_notifier(struct notifier_block *self,
			unsigned long event, void *ptr)
{
	struct charger_cable *cable =
		container_of(self, struct charger_cable, nb);

	/*
	 * The newly state of charger cable.
	 * If cable is attached, cable->attached is true.
	 */
	cable->attached = event;

#ifdef CONFIG_VENDOR_POWER_VOTER
	if (cable->cm->policy.usb_power_phy)
		power_supply_changed(cable->cm->policy.usb_power_phy);
#endif
	pr_info("%s: usb cable attached: %d\n", __func__, cable->attached);

	/*
	 * Setup monitoring to check battery state
	 * when charger cable is attached.
	 */
	if (cable->attached && is_polling_required(cable->cm)) {
		cancel_work_sync(&setup_polling);
		schedule_work(&setup_polling);
	}

	return NOTIFY_DONE;
}

/**
 * charger_extcon_init - register external connector to use it
 *			as the charger cable
 *
 * @cm: the Charger Manager representing the battery.
 * @cable: the Charger cable representing the external connector.
 */
static int charger_extcon_init(struct charger_manager *cm,
		struct charger_cable *cable)
{
	int ret;

	/*
	 * Charger manager use Extcon framework to identify
	 * the charger cable among various external connector
	 * cable (e.g., TA, USB, MHL, Dock).
	 */
	cable->nb.notifier_call = charger_extcon_notifier;
	ret = devm_extcon_register_notifier(cm->dev, cable->extcon_dev,
					    EXTCON_USB, &cable->nb);
	if (ret < 0)
		dev_err(cm->dev, "Cannot register extcon_dev for (cable: %s)\n",
			cable->name);

	return ret;
}

/**
 * charger_manager_register_extcon - Register extcon device to recevie state
 *				     of charger cable.
 * @cm: the Charger Manager representing the battery.
 *
 * This function support EXTCON(External Connector) subsystem to detect the
 * state of charger cables for enabling or disabling charger(regulator) and
 * select the charger cable for charging among a number of external cable
 * according to policy of H/W board.
 */
static int charger_manager_register_extcon(struct charger_manager *cm)
{
	struct charger_desc *desc = cm->desc;
	struct charger_regulator *charger;
	int ret;
	int i;
	int j;

	for (i = 0; i < desc->num_charger_regulators; i++) {
		charger = &desc->charger_regulators[i];

		charger->consumer = regulator_get(cm->dev,
					charger->regulator_name);
		if (IS_ERR(charger->consumer)) {
			dev_err(cm->dev, "Cannot find charger(%s)\n",
				charger->regulator_name);
			return PTR_ERR(charger->consumer);
		}
		charger->cm = cm;

		for (j = 0; j < charger->num_cables; j++) {
			struct charger_cable *cable = &charger->cables[j];

			ret = charger_extcon_init(cm, cable);
			if (ret < 0) {
				dev_err(cm->dev, "Cannot initialize charger(%s)\n",
					charger->regulator_name);
				return ret;
			}
			cable->charger = charger;
			cable->cm = cm;
		}
	}

	return 0;
}

/* help function of sysfs node to control charger(regulator) */
static ssize_t charger_name_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct charger_regulator *charger
		= container_of(attr, struct charger_regulator, attr_name);

	return sprintf(buf, "%s\n", charger->regulator_name);
}

static ssize_t charger_state_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct charger_regulator *charger
		= container_of(attr, struct charger_regulator, attr_state);
	int state = 0;

	if (!charger->externally_control)
		state = regulator_is_enabled(charger->consumer);

	return sprintf(buf, "%s\n", state ? "enabled" : "disabled");
}

static ssize_t jeita_control_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct charger_regulator *charger =
		container_of(attr, struct charger_regulator,
			     attr_jeita_control);
	struct charger_desc *desc = charger->cm->desc;

	return sprintf(buf, "%d\n", !desc->jeita_disabled);
}

static ssize_t jeita_control_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int ret;
	struct charger_regulator *charger =
		container_of(attr, struct charger_regulator,
			     attr_jeita_control);
	struct charger_desc *desc = charger->cm->desc;
	bool enabled;

	ret =  kstrtobool(buf, &enabled);
	if (ret)
		return ret;

	desc->jeita_disabled = !enabled;

	return count;
}

static ssize_t charger_stop_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct charger_regulator *charger
		= container_of(attr, struct charger_regulator,
			       attr_stop_charge);
	bool stop_charge;

	stop_charge = is_charging(charger->cm);

	return sprintf(buf, "%d\n", !stop_charge);
}

static ssize_t charger_stop_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct charger_regulator *charger
		= container_of(attr, struct charger_regulator,
					attr_stop_charge);
	struct charger_manager *cm = charger->cm;
	int stop_charge, ret;

	ret = sscanf(buf, "%d", &stop_charge);
	if (!ret)
		return -EINVAL;

	if (!is_ext_pwr_online(cm))
		return -EINVAL;

	if (!stop_charge) {
		ret = try_charger_enable(cm, true);
		if (ret) {
			dev_err(cm->dev, "failed to start charger.\n");
			return ret;
		}
		charger->externally_control = false;
	} else {
		ret = try_charger_enable(cm, false);
		if (ret) {
			dev_err(cm->dev, "failed to stop charger.\n");
			return ret;
		}
		charger->externally_control = true;
	}

	power_supply_changed(cm->charger_psy);

	return count;
}

static ssize_t charger_externally_control_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct charger_regulator *charger = container_of(attr,
			struct charger_regulator, attr_externally_control);

	return sprintf(buf, "%d\n", charger->externally_control);
}

static ssize_t charger_externally_control_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct charger_regulator *charger
		= container_of(attr, struct charger_regulator,
					attr_externally_control);
	struct charger_manager *cm = charger->cm;
	struct charger_desc *desc = cm->desc;
	int i;
	int ret;
	int externally_control;
	int chargers_externally_control = 1;

	ret = sscanf(buf, "%d", &externally_control);
	if (ret == 0) {
		ret = -EINVAL;
		return ret;
	}

	if (!externally_control) {
		charger->externally_control = 0;
		return count;
	}

	for (i = 0; i < desc->num_charger_regulators; i++) {
		if (&desc->charger_regulators[i] != charger &&
			!desc->charger_regulators[i].externally_control) {
			/*
			 * At least, one charger is controlled by
			 * charger-manager
			 */
			chargers_externally_control = 0;
			break;
		}
	}

	if (!chargers_externally_control) {
		if (cm->charger_enabled) {
			try_charger_enable(charger->cm, false);
			charger->externally_control = externally_control;
			try_charger_enable(charger->cm, true);
		} else {
			charger->externally_control = externally_control;
		}
	} else {
		dev_warn(cm->dev,
			 "'%s' regulator should be controlled in charger-manager because charger-manager must need at least one charger for charging\n",
			 charger->regulator_name);
	}

	return count;
}

/**
 * charger_manager_register_sysfs - Register sysfs entry for each charger
 * @cm: the Charger Manager representing the battery.
 *
 * This function add sysfs entry for charger(regulator) to control charger from
 * user-space. If some development board use one more chargers for charging
 * but only need one charger on specific case which is dependent on user
 * scenario or hardware restrictions, the user enter 1 or 0(zero) to '/sys/
 * class/power_supply/battery/charger.[index]/externally_control'. For example,
 * if user enter 1 to 'sys/class/power_supply/battery/charger.[index]/
 * externally_control, this charger isn't controlled from charger-manager and
 * always stay off state of regulator.
 */
static int charger_manager_register_sysfs(struct charger_manager *cm)
{
	struct charger_desc *desc = cm->desc;
	struct charger_regulator *charger;
	int chargers_externally_control = 1;
	char buf[11];
	char *str;
	int ret;
	int i;

	/* Create sysfs entry to control charger(regulator) */
	for (i = 0; i < desc->num_charger_regulators; i++) {
		charger = &desc->charger_regulators[i];

		snprintf(buf, 10, "charger.%d", i);
		str = devm_kzalloc(cm->dev,
				sizeof(char) * (strlen(buf) + 1), GFP_KERNEL);
		if (!str)
			return -ENOMEM;

		strcpy(str, buf);

		charger->attrs[0] = &charger->attr_name.attr;
		charger->attrs[1] = &charger->attr_state.attr;
		charger->attrs[2] = &charger->attr_externally_control.attr;
		charger->attrs[3] = &charger->attr_stop_charge.attr;
		charger->attrs[4] = &charger->attr_jeita_control.attr;
		charger->attrs[5] = NULL;
		charger->attr_g.name = str;
		charger->attr_g.attrs = charger->attrs;

		sysfs_attr_init(&charger->attr_name.attr);
		charger->attr_name.attr.name = "name";
		charger->attr_name.attr.mode = 0444;
		charger->attr_name.show = charger_name_show;

		sysfs_attr_init(&charger->attr_state.attr);
		charger->attr_state.attr.name = "state";
		charger->attr_state.attr.mode = 0444;
		charger->attr_state.show = charger_state_show;

		sysfs_attr_init(&charger->attr_stop_charge.attr);
		charger->attr_stop_charge.attr.name = "stop_charge";
		charger->attr_stop_charge.attr.mode = 0644;
		charger->attr_stop_charge.show = charger_stop_show;
		charger->attr_stop_charge.store = charger_stop_store;

		sysfs_attr_init(&charger->attr_jeita_control.attr);
		charger->attr_jeita_control.attr.name = "jeita_control";
		charger->attr_jeita_control.attr.mode = 0644;
		charger->attr_jeita_control.show = jeita_control_show;
		charger->attr_jeita_control.store = jeita_control_store;

		sysfs_attr_init(&charger->attr_externally_control.attr);
		charger->attr_externally_control.attr.name
				= "externally_control";
		charger->attr_externally_control.attr.mode = 0644;
		charger->attr_externally_control.show
				= charger_externally_control_show;
		charger->attr_externally_control.store
				= charger_externally_control_store;

		if (!desc->charger_regulators[i].externally_control ||
				!chargers_externally_control)
			chargers_externally_control = 0;

		dev_info(cm->dev, "'%s' regulator's externally_control is %d\n",
			 charger->regulator_name, charger->externally_control);

		ret = sysfs_create_group(&cm->charger_psy->dev.kobj,
					&charger->attr_g);
		if (ret < 0) {
			dev_err(cm->dev, "Cannot create sysfs entry of %s regulator\n",
				charger->regulator_name);
			return ret;
		}
	}

	if (chargers_externally_control) {
		dev_err(cm->dev, "Cannot register regulator because charger-manager must need at least one charger for charging battery\n");
		return -EINVAL;
	}

	return 0;
}

static int cm_init_thermal_data(struct charger_manager *cm,
		struct power_supply *fuel_gauge)
{
	struct charger_desc *desc = cm->desc;
	union power_supply_propval val;
	int ret;

	/* Verify whether fuel gauge provides battery temperature */
	ret = power_supply_get_property(fuel_gauge,
					POWER_SUPPLY_PROP_TEMP, &val);

	if (!ret) {
		cm->charger_psy_desc.properties[cm->charger_psy_desc.num_properties] =
				POWER_SUPPLY_PROP_TEMP;
		cm->charger_psy_desc.num_properties++;
		cm->desc->measure_battery_temp = true;
	}
#ifdef CONFIG_THERMAL
	if (desc->thermal_zone) {
		cm->tzd_batt =
			thermal_zone_get_zone_by_name(desc->thermal_zone);
		if (IS_ERR(cm->tzd_batt))
			return PTR_ERR(cm->tzd_batt);

		/* Use external thermometer */
		cm->charger_psy_desc.properties[cm->charger_psy_desc.num_properties] =
				POWER_SUPPLY_PROP_TEMP_AMBIENT;
		cm->charger_psy_desc.num_properties++;
		cm->desc->measure_battery_temp = true;
		ret = 0;
	}
#endif
	if (cm->desc->measure_battery_temp) {
		/* NOTICE : Default allowable minimum charge temperature is 0 */
		if (!desc->temp_max)
			desc->temp_max = CM_DEFAULT_CHARGE_TEMP_MAX;
		if (!desc->temp_diff)
			desc->temp_diff = CM_DEFAULT_RECHARGE_TEMP_DIFF;
	}

	return ret;
}

static int cm_parse_jeita_table(struct charger_desc *desc,
				struct device *dev,
				const char *np_name,
				struct charger_jeita_table **cur_table)
{
	struct device_node *np = dev->of_node;
	struct charger_jeita_table *table;
	const __be32 *list;
	int i, size;

	list = of_get_property(np, np_name, &size);
	if (!list || !size)
		return 0;

	desc->jeita_tab_size = size / (4 * sizeof(__be32));
	table = devm_kzalloc(dev, sizeof(struct charger_jeita_table) *
				(desc->jeita_tab_size + 1), GFP_KERNEL);
	if (!table)
		return -ENOMEM;

	pr_info("[%s]: jeita table size is %d\n", np_name, desc->jeita_tab_size);
	pr_info("***************************\n");
	for (i = 0; i < desc->jeita_tab_size; i++) {
		table[i].temp = be32_to_cpu(*list++) - 1000;
		table[i].recovery_temp = be32_to_cpu(*list++) - 1000;
		table[i].current_ua = be32_to_cpu(*list++);
		table[i].term_volt = be32_to_cpu(*list++);
		pr_info("[%s]: index[%d] = %ddec, %ddec, %dmA, %dmV\n", np_name, i,
			table[i].temp / 10, table[i].recovery_temp / 10,
			table[i].current_ua / 1000, table[i].term_volt / 1000);
	}
	pr_info("***************************\n");
	*cur_table = table;

	return 0;
}

static int cm_init_jeita_table(struct charger_desc *desc, struct device *dev)
{
	int ret, i;

	if (desc->disable_charger_type_jeita) {
		ret = cm_parse_jeita_table(desc,
				   dev,
				   "cm-jeita-temp-table",
				   &desc->jeita_tab);
		if (ret) {
			dev_info(dev, "%s init jeita table fail\n", __func__);
			return ret;
		}
	} else {
		for (i = CM_JEITA_DCP; i < CM_JEITA_MAX; i++) {
			ret = cm_parse_jeita_table(desc,
						   dev,
						   jeita_type_names[i],
						   &desc->jeita_tab_array[i]);
			if (ret)
				return ret;
		}
		desc->jeita_tab = desc->jeita_tab_array[CM_JEITA_UNKNOWN];
	}

	return 0;
}

static const struct of_device_id charger_manager_match[] = {
	{
		.compatible = "charger-manager",
	},
	{},
};

static void cm_track_capacity_work(struct work_struct *work)
{
	struct charger_manager *cm = container_of(work,
						  struct charger_manager,
						  track.track_capacity_work.work);
	u32 total_cap, capacity, check_capacity, file_buf[2];
	struct file *filep;
	loff_t pos = 0;
	static int retry_cnt = 5;
	int ret;

	filep = filp_open(CM_TRACK_FILE_PATH,
			  O_RDWR | O_CREAT,
			  S_IRUGO | S_IWUSR);
	if (IS_ERR(filep)) {
		dev_warn(cm->dev, "failed to open track file.\n");
		if (cm->track.state == CAP_TRACK_INIT && retry_cnt > 0) {
			dev_err(cm->dev, "track file not ready.\n");
			retry_cnt--;
			queue_delayed_work(system_power_efficient_wq,
					   &cm->track.track_capacity_work,
					   5 * HZ);
		} else {
			cm->track.state = CAP_TRACK_ERR;
		}
		return;
	}

	ret = get_batt_total_cap(cm, &total_cap);
	if (ret) {
		dev_err(cm->dev, "failed to get total cap.\n");
		goto out;
	}
	total_cap = total_cap / 1000;

	switch (cm->track.state) {
	case CAP_TRACK_INIT:
		/*
		 * When the capacity tracking function starts to work,
		 * need to read the last saved capacity value from the
		 * file system, for security reasons we need to decrypt,
		 * in contrast, when writing data to the file system,
		 * we need to encrypt it.
		 */
		cm->track.state = CAP_TRACK_IDLE;
		if (kernel_read(filep, (char *)&file_buf, sizeof(file_buf), &pos) < 0) {
			dev_err(cm->dev, "track file is empty or read error\n");
			goto out;
		}

		capacity = file_buf[0] ^ CM_TRACK_CAPACITY_KEY0;
		check_capacity = file_buf[1] ^ CM_TRACK_CAPACITY_KEY1;
		if (capacity != check_capacity) {
			dev_err(cm->dev, "track file data error.\n");
			goto out;
		}

		if (abs(total_cap - capacity) < total_cap / 2)
			set_batt_total_cap(cm, capacity);
		break;

	case CAP_TRACK_DONE:
		cm->track.state = CAP_TRACK_IDLE;
		file_buf[0] = total_cap ^ CM_TRACK_CAPACITY_KEY0;
		file_buf[1] = total_cap ^ CM_TRACK_CAPACITY_KEY1;
		ret = kernel_write(filep, &file_buf, sizeof(file_buf), &pos);
		if (ret < 0) {
			dev_err(cm->dev, "write file_buf data error\n");
			goto out;
		}
		break;

	default:
		cm->track.state = CAP_TRACK_IDLE;
		break;
	}

out:
	if (!IS_ERR(filep))
		filp_close(filep, NULL);
}

static void cm_track_capacity_monitor(struct charger_manager *cm)
{
	int cur_now, ret;
	int capacity, clbcnt, ocv, boot_volt, batt_uV;
	u32 total_cap;

	if (!cm->track.cap_tracking)
		return;

	if (!is_batt_present(cm)) {
		dev_err(cm->dev, "battery is not present, cancel monitor.\n");
		return;
	}

	if (cm->desc->temperature > CM_TRACK_HIGH_TEMP_THRESHOLD ||
	    cm->desc->temperature < CM_TRACK_LOW_TEMP_THRESHOLD) {
		dev_err(cm->dev, "exceed temperature range, cancel monitor.\n");
		return;
	}

	ret = get_batt_cur_now(cm, &cur_now);
	if (ret) {
		dev_err(cm->dev, "failed to get relax current.\n");
		return;
	}

	ret = get_batt_uV(cm, &batt_uV);
	if (ret) {
		dev_err(cm->dev, "failed to get battery voltage.\n");
		return;
	}
	ret = get_batt_ocv(cm, &ocv);
	if (ret) {
		dev_err(cm->dev, "get ocv error\n");
		return;
	}

	ret = get_batt_boot_vol(cm, &boot_volt);
	if (ret) {
		dev_err(cm->dev, "get boot voltage error\n");
		return;
	}
	/*
	 * If the capacity tracking monitor in idle state, we will
	 * record the start battery coulomb. when the capacity
	 * tracking monitor meet end condition, also will record
	 * the end battery coulomb, we can calculate the actual
	 * battery capacity by delta coulomb.
	 * if the following formula , we will replace the standard
	 * capacity with the calculated actual capacity.
	 * formula:
	 * abs(current_capacity -capacity) < capacity / 2
	 */
	switch (cm->track.state) {
	case CAP_TRACK_ERR:
		dev_err(cm->dev, "track status error, cancel monitor.\n");
		return;

	case CAP_TRACK_IDLE:
		/*
		 * The capacity tracking monitor start condition is
		 * divided into two types:
		 * 1.poweroff charging mode:
		 * the boot voltage is less than 3500000uv, because
		 * we set the ocv minimum value is 3400000uv, so the
		 * the tracking start voltage value we set needs to
		 * be infinitely close to the shutdown value.
		 * 2.power on normal mode:
		 * the current less than 30000ua and the voltage
		 * less than 3650000uv. When meet the above conditions,
		 * the battery is almost empty, which is the result of
		 * multiple test data, so this point suitable as a
		 * starting condition.
		 */
		if (is_charger_mode) {
			if (boot_volt > CM_TRACK_CAPACITY_SHUTDOWN_START_VOLTAGE ||
			    ocv > CM_TRACK_CAPACITY_START_VOLTAGE) {
				dev_info(cm->dev, "not satisfy shutdown start condition.\n");
				return;
			}
		} else {
			if (abs(cur_now) > CM_TRACK_CAPACITY_START_CURRENT ||
			    ocv > CM_TRACK_CAPACITY_START_VOLTAGE) {
				dev_info(cm->dev, "not satisfy power on start condition.\n");
				return;
			}
		}

		/*
		 * Parse the capacity table to look up the correct capacity percent
		 * according to current battery's corresponding OCV values.
		 */
		if (is_charger_mode)
			cm->track.start_cap = power_supply_ocv2cap_simple(cm->desc->cap_table,
							      cm->desc->cap_table_len,
							      boot_volt);
		else
			cm->track.start_cap = power_supply_ocv2cap_simple(cm->desc->cap_table,
							      cm->desc->cap_table_len,
							      ocv);
		cm->track.start_cap *= 10;
		/*
		 * When the capacity tracking start condition is met,
		 * the battery is almost empty,so we set a starting
		 * threshold, if it is greater than it will not enable
		 * the capacity tracking function, now we set the capacity
		 * tracking monitor initial percentage threshold to 20%.
		 */
		if (cm->track.start_cap > CM_TRACK_START_CAP_THRESHOLD) {
			cm->track.start_cap = 0;
			dev_info(cm->dev,
				 "does not satisfy the track start condition, start_cap = %d\n",
				 cm->track.start_cap);
			return;
		}

		ret = get_batt_energy_now(cm, &clbcnt);
		if (ret) {
			dev_err(cm->dev, "failed to get energy now.\n");
			return;
		}

		cm->track.start_time =
			ktime_divns(ktime_get_boottime(), NSEC_PER_SEC);
		cm->track.start_clbcnt = clbcnt;
		cm->track.state = CAP_TRACK_UPDATING;
		break;

	case CAP_TRACK_UPDATING:
		if ((ktime_divns(ktime_get_boottime(), NSEC_PER_SEC) -
		     cm->track.start_time) > CM_TRACK_TIMEOUT_THRESHOLD) {
			cm->track.state = CAP_TRACK_IDLE;
			dev_err(cm->dev, "track capacity time out.\n");
			return;
		}

		/*
		 * When the capacity tracking end condition is met,
		 * the battery voltage is almost full, so we use full
		 * stop charging condition as the the capacity
		 * tracking end condition.
		 */
		if (batt_uV > cm->track.end_vol &&
		    cur_now < cm->track.end_cur) {
			ret = get_batt_energy_now(cm, &clbcnt);
			if (ret) {
				dev_err(cm->dev, "failed to get energy now.\n");
				return;
			}

			ret = get_batt_total_cap(cm, &total_cap);
			if (ret) {
				dev_err(cm->dev, "failed to get relax voltage.\n");
				return;
			}
			total_cap = total_cap / 1000;

			/*
			 * Due to the capacity tracking function started, the
			 * coulomb amount corresponding to the initial
			 * percentage was not counted, so we need to
			 * compensate initial coulomb with following
			 * formula, we assume that coulomb and capacity
			 * are directly proportional.
			 *
			 * For example:
			 * if capacity tracking function started,  the battery
			 * percentage is 3%, we will count the capacity from
			 * 3% to 100%, it will discard capacity from 0% to 3%
			 * so we use "total_cap * (start_cap / 100)" to
			 * compensate.
			 *
			 * formula:
			 * capacity = total_cap * (start_cap / 100) + capacity
			 */
			capacity = (clbcnt - cm->track.start_clbcnt) / 1000;
			capacity =
				(total_cap * cm->track.start_cap) / 1000 + capacity;

			if (abs(capacity - total_cap) < total_cap / 2) {
				set_batt_total_cap(cm, capacity);
				cm->track.state = CAP_TRACK_DONE;
				queue_delayed_work(system_power_efficient_wq,
						   &cm->track.track_capacity_work,
						   0);
				dev_info(cm->dev,
					 "track capacity is done capacity = %d\n",
					 capacity);
			} else {
				cm->track.state = CAP_TRACK_IDLE;
				dev_info(cm->dev,
					 "less than half standard capacity.\n");
			}
		}
		break;

	default:
		break;
	}
}

#define FULLBATT_WARM_VOLTAGE_UV 4100000
#define FULLBATT_ADVANCE_LEVEL 960
static struct charger_desc *of_cm_parse_desc(struct device *dev)
{
	struct charger_desc *desc;
	struct device_node *np = dev->of_node;
	u32 poll_mode = CM_POLL_DISABLE;
	u32 battery_stat = CM_NO_BATTERY;
	u32 num_chgs = 0;
	int ret;
	u32 val = 0;

	desc = devm_kzalloc(dev, sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return ERR_PTR(-ENOMEM);

	of_property_read_string(np, "cm-name", &desc->psy_name);

	of_property_read_u32(np, "cm-poll-mode", &poll_mode);
	desc->polling_mode = poll_mode;

	of_property_read_u32(np, "cm-poll-interval",
				&desc->polling_interval_ms);

	of_property_read_u32(np, "cm-fullbatt-vchkdrop-ms",
					&desc->fullbatt_vchkdrop_ms);
	of_property_read_u32(np, "cm-fullbatt-vchkdrop-volt",
					&desc->fullbatt_vchkdrop_uV);
	of_property_read_u32(np, "cm-fullbatt-voltage", &desc->fullbatt_uV);
	of_property_read_u32(np, "cm-fullbatt-current", &desc->fullbatt_uA);
	of_property_read_u32(np, "cm-fullbatt-soc", &desc->fullbatt_soc);
	of_property_read_u32(np, "cm-fullbatt-capacity",
					&desc->fullbatt_full_capacity);
	of_property_read_u32(np, "cm-shutdown-voltage", &desc->shutdown_voltage);
	of_property_read_u32(np, "cm-tickle-time-out", &desc->trickle_time_out);
	of_property_read_u32(np, "cm-one-cap-time", &desc->cap_one_time);
	of_property_read_u32(np, "cm-wdt-interval", &desc->wdt_interval);

	of_property_read_u32(np, "cm-battery-stat", &battery_stat);
	desc->battery_present = battery_stat;

	if (of_property_read_u32(np, "cm-fullbatt-warm-voltage", &val) >= 0)
		desc->fullbatt_warm_uV = val;
	else
		desc->fullbatt_warm_uV = FULLBATT_WARM_VOLTAGE_UV;

	if (of_property_read_u32(np, "cm-fullbatt-comp-voltage", &val) >= 0)
		desc->fullbatt_comp_uV = val;
	else
		desc->fullbatt_comp_uV = 0;

	if (of_property_read_u32(np, "cm-thermal-limit", &val) >= 0)
		desc->thermal_limit = val;
	else
		desc->thermal_limit = 0;

	if (of_property_read_u32(np, "cm-fullbatt-advance-level", &val) >= 0)
		desc->fullbatt_advance_level = val;
	else
		desc->fullbatt_advance_level = FULLBATT_ADVANCE_LEVEL;

	pr_info("poll_mode: %d, polling_interval_ms: %d, fullbatt_vchkdrop_ms: %d, fullbatt_vchkdrop_uV: %d, "
		"fullbatt_uV: %d, fullbatt_uA: %d, fullbatt_soc: %d, fullbatt_full_capacity: %d, "
		"shutdown_voltage: %d, trickle_time_out: %d, cap_one_time: %d, wdt_interval: %d, "
		"battery_present: %d, fullbatt_warm_uV: %d, fullbatt_comp_uV: %d, thermal_limit: %d, fullbatt_advance_level: %d\n",
		desc->polling_mode, desc->polling_interval_ms, desc->fullbatt_vchkdrop_ms, desc->fullbatt_vchkdrop_uV,
		desc->fullbatt_uV, desc->fullbatt_uA, desc->fullbatt_soc, desc->fullbatt_full_capacity,
		desc->shutdown_voltage, desc->trickle_time_out, desc->cap_one_time, desc->wdt_interval,
		desc->battery_present, desc->fullbatt_warm_uV, desc->fullbatt_comp_uV, desc->thermal_limit,
		desc->fullbatt_advance_level);
	/* chargers */
	of_property_read_u32(np, "cm-num-chargers", &num_chgs);
	if (num_chgs) {
		/* Allocate empty bin at the tail of array */
		desc->psy_charger_stat = devm_kzalloc(dev, sizeof(char *)
						* (num_chgs + 1), GFP_KERNEL);
		if (desc->psy_charger_stat) {
			int i;
			for (i = 0; i < num_chgs; i++)
				of_property_read_string_index(np, "cm-chargers",
						i, &desc->psy_charger_stat[i]);
		} else {
			return ERR_PTR(-ENOMEM);
		}
	}

	/* fast chargers */
	of_property_read_u32(np, "cm-num-fast-chargers", &num_chgs);
	if (num_chgs) {
		/* Allocate empty bin at the tail of array */
		desc->psy_fast_charger_stat = devm_kzalloc(dev, sizeof(char *)
						* (num_chgs + 1), GFP_KERNEL);
		if (desc->psy_fast_charger_stat) {
			int i;

			for (i = 0; i < num_chgs; i++)
				of_property_read_string_index(np, "cm-fast-chargers",
						i, &desc->psy_fast_charger_stat[i]);
		} else {
			return ERR_PTR(-ENOMEM);
		}
	}

	ret = of_property_read_string(np, "cm-hardware-psy", &desc->psy_hardware);
	if (ret) {
		pr_info("No cm-hardware-psy config, ret=%d\n", ret);
		desc->psy_hardware = NULL;
	} else
		pr_info("cm-hardware-psy config: %s!\n", desc->psy_hardware);

	ret = of_property_read_string(np, "cm-hardware2-psy", &desc->psy_hardware2);
	if (ret) {
		pr_info("No cm-hardware2-psy config, ret=%d\n", ret);
		desc->psy_hardware2 = NULL;
	} else
		pr_info("cm-hardware2-psy config: %s!\n", desc->psy_hardware2);

	of_property_read_string(np, "cm-fuel-gauge", &desc->psy_fuel_gauge);

	of_property_read_string(np, "cm-thermal-zone", &desc->thermal_zone);

	of_property_read_u32(np, "cm-battery-cold", &desc->temp_min);
	if (of_get_property(np, "cm-battery-cold-in-minus", NULL))
		desc->temp_min *= -1;
	of_property_read_u32(np, "cm-battery-hot", &desc->temp_max);
	of_property_read_u32(np, "cm-battery-temp-diff", &desc->temp_diff);

	of_property_read_u32(np, "cm-charging-max",
				&desc->charging_max_duration_ms);
	of_property_read_u32(np, "cm-discharging-max",
				&desc->discharging_max_duration_ms);
	of_property_read_u32(np, "cm-charge-voltage-max",
			     &desc->normal_charge_voltage_max);
	of_property_read_u32(np, "cm-charge-voltage-drop",
			     &desc->normal_charge_voltage_drop);
	of_property_read_u32(np, "cm-fast-charge-voltage-max",
			     &desc->fast_charge_voltage_max);
	of_property_read_u32(np, "cm-fast-charge-voltage-drop",
			     &desc->fast_charge_voltage_drop);

	desc->disable_charger_type_jeita =
		of_property_read_bool(np, "cm-disable-charger-type-jeita");

	desc->charging_max_duration_ms = 0;
	desc->discharging_max_duration_ms = 0;
	pr_info("psy_fuel_gauge: %s, temp_min: %d, temp_max: %d, temp_diff: %d, "
		"charging_max_duration_ms: %d, discharging_max_duration_ms: %d, "
		"normal_charge_voltage_max: %d, normal_charge_voltage_drop: %d, "
		"fast_charge_voltage_max: %d, fast_charge_voltage_drop: %d, "
		"disable_charger_type_jeita: %d\n",
		desc->psy_fuel_gauge, desc->temp_min, desc->temp_max, desc->temp_diff,
		desc->charging_max_duration_ms, desc->discharging_max_duration_ms,
		desc->normal_charge_voltage_max, desc->normal_charge_voltage_drop,
		desc->fast_charge_voltage_max, desc->fast_charge_voltage_drop,
		desc->disable_charger_type_jeita);
	/* Initialize the jeita temperature table. */
	ret = cm_init_jeita_table(desc, dev);
	if (ret)
		return ERR_PTR(ret);

	ret = cm_init_cap_remap_table(desc, dev);
	if (ret)
		dev_info(dev, "%s init cap remap table fail\n", __func__);

	/* battery charger regualtors */
	desc->num_charger_regulators = of_get_child_count(np);
	if (desc->num_charger_regulators) {
		struct charger_regulator *chg_regs;
		struct device_node *child;

		chg_regs = devm_kzalloc(dev, sizeof(*chg_regs)
					* desc->num_charger_regulators,
					GFP_KERNEL);
		if (!chg_regs)
			return ERR_PTR(-ENOMEM);

		desc->charger_regulators = chg_regs;

		for_each_child_of_node(np, child) {
			struct charger_cable *cables;
			struct device_node *_child;

			of_property_read_string(child, "cm-regulator-name",
					&chg_regs->regulator_name);

			/* charger cables */
			chg_regs->num_cables = of_get_child_count(child);
			if (chg_regs->num_cables) {
				cables = devm_kzalloc(dev, sizeof(*cables)
						* chg_regs->num_cables,
						GFP_KERNEL);
				if (!cables) {
					of_node_put(child);
					return ERR_PTR(-ENOMEM);
				}

				chg_regs->cables = cables;

				for_each_child_of_node(child, _child) {
					of_property_read_string(_child,
					"cm-cable-name", &cables->name);
					of_property_read_u32(_child,
					"cm-cable-min",
					&cables->min_uA);
					of_property_read_u32(_child,
					"cm-cable-max",
					&cables->max_uA);

					if (of_property_read_bool(_child, "extcon")) {
						struct device_node *np;

						np = of_parse_phandle(_child, "extcon", 0);
						if (!np)
							return ERR_PTR(-ENODEV);

						cables->extcon_dev = extcon_find_edev_by_node(np);
						of_node_put(np);
						if (IS_ERR(cables->extcon_dev))
							return ERR_PTR(PTR_ERR(cables->extcon_dev));
					}
					cables++;
				}
			}
			chg_regs++;
		}
	}
	return desc;
}

static inline struct charger_desc *cm_get_drv_data(struct platform_device *pdev)
{
	if (pdev->dev.of_node)
		return of_cm_parse_desc(&pdev->dev);
	return dev_get_platdata(&pdev->dev);
}

static int cm_get_bat_info(struct charger_manager *cm)
{
	struct power_supply_battery_info info = { };
	struct power_supply_battery_ocv_table *table;
	union power_supply_propval prop = {0,};
	int ret = 0;

	if (cm->charger_psy) {
		ret = power_supply_get_property(cm->charger_psy, POWER_SUPPLY_PROP_BATTERY_ID, &prop);
		if (ret)
			pr_err("failed to battery id index, error:%d.\n", ret);
		else {
			cm->batt_id_index = prop.intval;
			dev_info(cm->dev, "%s: charger-manager get battery id index: %d\n", __func__,
				cm->batt_id_index);
		}
	}

	ret = power_supply_get_battery_info(cm->charger_psy, &info, cm->batt_id_index);
	if (ret) {
		dev_err(cm->dev, "failed to get battery information\n");
		return ret;
	}

	cm->desc->internal_resist = info.factory_internal_resistance_uohm / 1000;

	/*
	 * For CHARGER MANAGER device, we only use one ocv-capacity
	 * table in normal temperature 20 Celsius.
	 */
	table = power_supply_find_ocv2cap_table(&info, 20, &cm->desc->cap_table_len);
	if (!table)
		return -EINVAL;

	cm->desc->cap_table = devm_kmemdup(cm->dev, table,
				     cm->desc->cap_table_len * sizeof(*table),
				     GFP_KERNEL);
	if (!cm->desc->cap_table) {
		power_supply_put_battery_info(cm->charger_psy, &info);
		return -ENOMEM;
	}

	power_supply_put_battery_info(cm->charger_psy, &info);

	return 0;
}

static void cm_track_capacity_init(struct charger_manager *cm)
{
	INIT_DELAYED_WORK(&cm->track.track_capacity_work,
			  cm_track_capacity_work);
	cm->track.end_vol =
		cm->desc->fullbatt_uV - CM_TRACK_CAPACITY_VOLTAGE_OFFSET;
	cm->track.end_cur =
		cm->desc->fullbatt_uA + CM_TRACK_CAPACITY_CURRENT_OFFSET;
	cm->track.state = CAP_TRACK_INIT;
	queue_delayed_work(system_power_efficient_wq,
			   &cm->track.track_capacity_work,
			   5 * HZ);
}

static void zte_cm_update_battery_health_status(struct charger_manager *cm, int temp)
{
	int cur_jeita_status = 0, bat_temp_diff = 10;
	static int pre_temp = 250, pre_health = POWER_SUPPLY_HEALTH_GOOD;

	cur_jeita_status = cm_manager_get_jeita_status(cm, temp);
	if (cur_jeita_status == STATUS_BELOW_T0)
		cm->health = POWER_SUPPLY_HEALTH_COLD;
	else if (cur_jeita_status <= STATUS_T1_TO_T2)
		cm->health = POWER_SUPPLY_HEALTH_COOL;
	else if (cur_jeita_status == STATUS_ABOVE_T4)
		cm->health = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (cur_jeita_status == STATUS_T3_TO_T4)
		cm->health = POWER_SUPPLY_HEALTH_WARM;
	else if (cur_jeita_status & CM_CHARGE_VOLTAGE_ABNORMAL)
		cm->health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	else
		cm->health = POWER_SUPPLY_HEALTH_GOOD;

	if (temp >= 550)
		bat_temp_diff = 10;
	else
		bat_temp_diff = 20;

	if ((cm->health != pre_health) || (abs(temp - pre_temp) >= bat_temp_diff)) {
		dev_info(cm->dev, "update battery health and temp, health:%d, temp:%d,%d\n",
			cm->health, temp, pre_temp);
		pre_temp = temp;
		pre_health = cm->health;
		power_supply_changed(cm->charger_psy);
	}
}

static void cm_batt_works(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct charger_manager *cm = container_of(dwork,
				struct charger_manager, cap_update_work);
	struct timespec64 cur_time;
	int batt_uV, batt_ocV, bat_uA, fuel_cap, chg_sts, ret;
	int period_time, flush_time, cur_temp, board_temp;
	int chg_cur = 0, chg_limit_cur = 0, charger_input_vol = 0;
	static int last_fuel_cap = CM_CAP_MAGIC_NUM;

	ret = get_batt_uV(cm, &batt_uV);
	if (ret) {
		dev_err(cm->dev, "get_batt_uV error.\n");
		return;
	}

	ret = get_batt_ocv(cm, &batt_ocV);
	if (ret) {
		dev_err(cm->dev, "get_batt_ocV error.\n");
		return;
	}

	ret = get_batt_uA(cm, &bat_uA);
	if (ret) {
		dev_err(cm->dev, "get bat_uA error.\n");
		return;
	}

	ret = get_charger_voltage(cm, &charger_input_vol);
	if (ret)
		dev_warn(cm->dev, "failed to get charger input voltage\n");

	ret = get_batt_cap(cm, &fuel_cap);
	if (ret) {
		dev_err(cm->dev, "get fuel_cap error.\n");
		return;
	}
	fuel_cap = cm_capacity_remap(cm, fuel_cap);

	ret = get_charger_current(cm, &chg_cur);
	if (ret) {
		dev_err(cm->dev, "get chg_cur error.\n");
		return;
	}

	ret = get_charger_limit_current(cm, &chg_limit_cur);
	if (ret) {
		dev_err(cm->dev, "get chg_limit_cur error.\n");
		return;
	}

	ret = cm_get_battery_temperature_by_psy(cm, &cur_temp);
	if (ret) {
		dev_err(cm->dev, "failed to get battery temperature\n");
		return;
	}

	cm->desc->temperature = cur_temp;
	zte_cm_update_battery_health_status(cm, cm->desc->temperature);

	ret = cm_get_battery_temperature(cm, &board_temp);
	if (ret)
		dev_warn(cm->dev, "failed to get board temperature\n");

	if (cur_temp <= CM_LOW_TEMP_REGION &&
	    batt_uV <= CM_LOW_TEMP_SHUTDOWN_VALTAGE) {
		if (cm->desc->low_temp_trigger_cnt++ > 1)
			fuel_cap = 0;
	} else if (cm->desc->low_temp_trigger_cnt != 0) {
		cm->desc->low_temp_trigger_cnt = 0;
	}

	if (fuel_cap > CM_CAP_FULL_PERCENT)
		fuel_cap = CM_CAP_FULL_PERCENT;
	else if (fuel_cap < 0)
		fuel_cap = 0;

	if (last_fuel_cap == CM_CAP_MAGIC_NUM)
		last_fuel_cap = fuel_cap;

	cur_time = ktime_to_timespec64(ktime_get_boottime());

	if (is_full_charged(cm))
		chg_sts = POWER_SUPPLY_STATUS_FULL;
	else if (is_charging(cm))
		chg_sts = POWER_SUPPLY_STATUS_CHARGING;
	else if (is_ext_pwr_online(cm))
		chg_sts = POWER_SUPPLY_STATUS_NOT_CHARGING;
	else
		chg_sts = POWER_SUPPLY_STATUS_DISCHARGING;

	/*
	 * Record the charging time when battery
	 * capacity is larger than 99%.
	 */
	if (chg_sts == POWER_SUPPLY_STATUS_CHARGING) {
		if (cm->desc->cap >= 986) {
			cm->desc->trickle_time =
				cur_time.tv_sec - cm->desc->trickle_start_time;
		} else {
			cm->desc->trickle_start_time = cur_time.tv_sec;
			cm->desc->trickle_time = 0;
		}
	} else {
		cm->desc->trickle_start_time = cur_time.tv_sec;
		cm->desc->trickle_time = cm->desc->trickle_time_out +
				cm->desc->cap_one_time;
	}

	flush_time = cur_time.tv_sec - cm->desc->update_capacity_time;
	period_time = cur_time.tv_sec - cm->desc->last_query_time;
	cm->desc->last_query_time = cur_time.tv_sec;

	if (cm->desc->force_set_full && is_ext_pwr_online(cm))
		cm->desc->charger_status = POWER_SUPPLY_STATUS_FULL;
	else
		cm->desc->charger_status = chg_sts;

	dev_info(cm->dev, "battery voltage = %d, OCV = %d, current = %d, "
		 "capacity = %d, charger status = %d, force set full = %d, "
		 "charging current = %d, charging limit current = %d, "
		 "battery temperature = %d,board temperature = %d, "
		 "track state = %d, charger type = %d, thm_adjust_cur = %d, "
		 "charger input voltage = %d\n",
		 batt_uV, batt_ocV, bat_uA, fuel_cap, cm->desc->charger_status,
		 cm->desc->force_set_full, chg_cur, chg_limit_cur, cur_temp, board_temp,
		 cm->track.state, cm->desc->charger_type, cm->desc->thm_adjust_cur,
		 charger_input_vol);

	dev_info(cm->dev, "charger_status = %d, last_fuel_cap = %d, fuel_cap = %d, "
		"desc->cap = %d, period_time = %d, flush_time = %d\n", cm->desc->charger_status,
		last_fuel_cap, fuel_cap, cm->desc->cap, period_time, flush_time);

	switch (cm->desc->charger_status) {
	case POWER_SUPPLY_STATUS_CHARGING:
		last_fuel_cap = fuel_cap;
		if (fuel_cap < cm->desc->cap) {
			if (bat_uA >= 0) {
				fuel_cap = cm->desc->cap;
			} else {
				/*
				 * The percentage of electricity is not
				 * allowed to change by 1% in cm->desc->cap_one_time.
				 */
				if (period_time < cm->desc->cap_one_time &&
					(cm->desc->cap - fuel_cap) >= 5)
					fuel_cap = cm->desc->cap - 5;
				/*
				 * If wake up from long sleep mode,
				 * will make a percentage compensation based on time.
				 */
				if ((cm->desc->cap - fuel_cap) >=
				    (flush_time / cm->desc->cap_one_time) * 10)
					fuel_cap = cm->desc->cap -
						(flush_time / cm->desc->cap_one_time) * 10;
			}
		} else if (fuel_cap > cm->desc->cap) {
			if (period_time < cm->desc->cap_one_time &&
					(fuel_cap - cm->desc->cap) >= 5)
				fuel_cap = cm->desc->cap + 5;

			if ((fuel_cap - cm->desc->cap) >=
			    (flush_time / cm->desc->cap_one_time) * 10)
				fuel_cap = cm->desc->cap +
					(flush_time / cm->desc->cap_one_time) * 10;
		}

		if (cm->desc->cap >= 985 && cm->desc->cap <= 994 &&
		    fuel_cap >= CM_CAP_FULL_PERCENT)
			fuel_cap = 994;
		/*
		 * Record 99% of the charging time.
		 * if it is greater than 1500s,
		 * it will be mandatory to display 100%,
		 * but the background is still charging.
		 */
		if (cm->desc->cap >= 995 &&
		    cm->desc->trickle_time >= cm->desc->trickle_time_out &&
		    cm->desc->trickle_time_out > 0 &&
		    bat_uA > 0)
			cm->desc->force_set_full = true;

		break;

	case POWER_SUPPLY_STATUS_NOT_CHARGING:
	case POWER_SUPPLY_STATUS_DISCHARGING:
		/*
		 * In not charging status,
		 * the cap is not allowed to increase.
		 */
		if (fuel_cap >= cm->desc->cap) {
			last_fuel_cap = fuel_cap;
			fuel_cap = cm->desc->cap;
		} else if (cm->desc->cap >= CM_HCAP_THRESHOLD) {
			if (last_fuel_cap - fuel_cap >= CM_HCAP_DECREASE_STEP) {
				if (cm->desc->cap - fuel_cap >= CM_CAP_ONE_PERCENT)
					fuel_cap = cm->desc->cap - CM_CAP_ONE_PERCENT;
				else
					fuel_cap = cm->desc->cap - CM_HCAP_DECREASE_STEP;

				last_fuel_cap -= CM_HCAP_DECREASE_STEP;
			} else {
				fuel_cap = cm->desc->cap;
			}
		} else {
			if (period_time < cm->desc->cap_one_time &&
			    (cm->desc->cap - fuel_cap) >= 5)
				fuel_cap = cm->desc->cap - 5;
			/*
			 * If wake up from long sleep mode,
			 * will make a percentage compensation based on time.
			 */
			if ((cm->desc->cap - fuel_cap) >=
			    (flush_time / cm->desc->cap_one_time) * 10)
				fuel_cap = cm->desc->cap -
					(flush_time / cm->desc->cap_one_time) * 10;
			else if (cm->desc->cap - fuel_cap > CM_CAP_ONE_PERCENT)
				fuel_cap = cm->desc->cap - CM_CAP_ONE_PERCENT;
		}
		break;

	case POWER_SUPPLY_STATUS_FULL:
		last_fuel_cap = fuel_cap;
		cm->desc->update_capacity_time = cur_time.tv_sec;
		if (cm->health == POWER_SUPPLY_HEALTH_WARM) {
			if ((batt_ocV < (cm->desc->fullbatt_warm_uV - cm->desc->fullbatt_vchkdrop_uV))
					&& (bat_uA < 0)) {
				cm->desc->force_set_full = false;
				dev_info(cm->dev, "force_set_full %d, health %d, batt_ocv %d, bat_uA %d\n",
					cm->desc->force_set_full, cm->health, batt_ocV, bat_uA);
			}
		} else if ((batt_ocV < (cm->desc->fullbatt_uV - cm->desc->fullbatt_vchkdrop_uV - 10000))
		    && (bat_uA < 0)) {
			cm->desc->force_set_full = false;
			dev_info(cm->dev, "force_set_full %d, batt_ocv %d, bat_uA %d.\n",
				cm->desc->force_set_full, batt_ocV, bat_uA);
		}

		if (is_ext_pwr_online(cm) && cm->health != POWER_SUPPLY_HEALTH_WARM) {
			if (fuel_cap != CM_CAP_FULL_PERCENT)
				fuel_cap = CM_CAP_FULL_PERCENT;

			if (fuel_cap > cm->desc->cap)
				fuel_cap = cm->desc->cap + 5;
		}

		break;
	default:
		break;
	}

	if (batt_uV <= cm->desc->shutdown_voltage - CM_UVLO_OFFSET) {
		set_batt_cap(cm, 0);
		dev_err(cm->dev, "WARN: batt_uV less than uvlo, will shutdown\n");
		orderly_poweroff(true);
	}

	dev_info(cm->dev, "battery cap = %d, charger manager cap = %d\n",
		 fuel_cap, cm->desc->cap);

	if (fuel_cap != cm->desc->cap) {
		if (DIV_ROUND_CLOSEST(fuel_cap, 10) != DIV_ROUND_CLOSEST(cm->desc->cap, 10)) {
			cm->desc->cap = fuel_cap;
			cm->desc->update_capacity_time = cur_time.tv_sec;
			power_supply_changed(cm->charger_psy);
		}

		cm->desc->cap = fuel_cap;
		set_batt_cap(cm, cm_capacity_unmap(cm, cm->desc->cap));
	}

	if (batt_uV >= CUTOFF_VOLTAGE_UV) {
		if (cm->desc->cap == 0) {
			cm->desc->cap = 1;
			dev_info(cm->dev, "force report soc 1, vbat %d\n", batt_uV);
			power_supply_changed(cm->charger_psy);
		}
	}
	queue_delayed_work(system_power_efficient_wq,
			   &cm->cap_update_work,
			   CM_CAP_CYCLE_TRACK_TIME * HZ);

	cm_track_capacity_monitor(cm);
}

#ifdef CONFIG_VENDOR_POWER_VOTER
static int charger_set_prop_by_name(const char *name, enum power_supply_property psp, int data)
{
	struct power_supply *psy = NULL;
	union power_supply_propval val = {0, };
	int rc = 0;

	if (name == NULL) {
		vote_error("psy name is NULL!!\n");
		goto failed_loop;
	}

	psy = power_supply_get_by_name(name);
	if (!psy) {
		vote_error("get %s psy failed!!\n", name);
		goto failed_loop;
	}

	val.intval = data;

	rc = power_supply_set_property(psy,
				psp, &val);
	if (rc < 0) {
		vote_error("Failed to set %s property:%d rc=%d\n", name, psp, rc);
		power_supply_put(psy);
		return rc;
	}

	power_supply_put(psy);

	return 0;

failed_loop:
	return -EINVAL;
}

static int charger_get_prop_by_name(const char *name, enum power_supply_property psp, int *data)
{
	struct power_supply *psy = NULL;
	union power_supply_propval val = {0, };
	int rc = 0;

	if (name == NULL) {
		vote_error("psy name is NULL!!\n");
		goto failed_loop;
	}

	psy = power_supply_get_by_name(name);
	if (!psy) {
		vote_error("get %s psy failed!!\n", name);
		goto failed_loop;
	}

	rc = power_supply_get_property(psy,
				psp, &val);
	if (rc < 0) {
		vote_error("Failed to get %s property:%d rc=%d\n", name, psp, rc);
		power_supply_put(psy);
		return rc;
	}

	*data = val.intval;

	power_supply_put(psy);

	return 0;

failed_loop:
	return -EINVAL;
}


static int interface_psy_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *pval)
{
	struct charger_manager *cm = power_supply_get_drvdata(psy);
	struct charger_policy *policy = &(cm->policy);
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		pval->intval = get_client_vote(policy->fcc_votable, CAS_SETTING_VOTER);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		pval->intval = get_client_vote(policy->fcv_votable, CAS_SETTING_VOTER);
		break;
	case POWER_SUPPLY_PROP_INPUT_SUSPEND:
		pval->intval = get_client_vote(policy->usb_icl_votable, CAS_SETTING_VOTER);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		pval->intval = get_client_vote(policy->topoff_votable, CAS_SETTING_VOTER);
		break;
	case POWER_SUPPLY_PROP_RECHARGE_SOC:
		pval->intval = get_client_vote(policy->recharge_soc_votable, CAS_SETTING_VOTER);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		pval->intval = get_client_vote(policy->usb_icl_votable, POLICY_SETTING_VOTER);
		pval->intval = (pval->intval < 0) ? 1 : 0;
		break;
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		pval->intval = get_client_vote(policy->battery_charging_enabled_votable, POLICY_SETTING_VOTER);
		pval->intval = (pval->intval < 0) ? 1 : 0;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		pval->intval = get_client_vote(policy->fcc_votable, ZTE_THERMAL_SETTING_VOTER);
		break;
	default:
		vote_debug("interface unsupported property %d\n", psp);
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int interface_psy_set_property(struct power_supply *psy,
				enum power_supply_property psp,
				const union power_supply_propval *pval)
{
	int rc = 0;
	struct charger_manager *cm = power_supply_get_drvdata(psy);
	struct charger_policy *policy = &(cm->policy);

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		vote_debug("CAS: Set fcc %d\n", pval->intval);
		if (pval->intval)
			vote(policy->fcc_votable, CAS_SETTING_VOTER, true, pval->intval);
		else
			vote(policy->fcc_votable, CAS_SETTING_VOTER, false, 0);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		vote_debug("CAS: Set fcv %d\n", pval->intval);
		if (pval->intval)
			vote(policy->fcv_votable, CAS_SETTING_VOTER, true, pval->intval);
		else
			vote(policy->fcv_votable, CAS_SETTING_VOTER, false, 0);
		break;
	case POWER_SUPPLY_PROP_INPUT_SUSPEND:
		vote_debug("CAS: Set input suspend %d\n", pval->intval);
		vote(policy->usb_icl_votable, CAS_SETTING_VOTER,
					!!pval->intval, pval->intval ? 100000 : 0);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		vote_debug("CAS: Set topoff %d\n", pval->intval);
		if (pval->intval)
			vote(policy->topoff_votable, CAS_SETTING_VOTER, true, pval->intval);
		else
			vote(policy->topoff_votable, CAS_SETTING_VOTER, false, 0);
		break;
	case POWER_SUPPLY_PROP_RECHARGE_SOC:
		vote_debug("CAS: Set recharge soc/voltage %d\n", pval->intval);
		if (pval->intval >= 100) {
			vote(policy->recharge_voltage_votable, CAS_SETTING_VOTER, true, pval->intval);
		} else if ((pval->intval < 100) && (pval->intval > 0)) {
			vote(policy->recharge_soc_votable, CAS_SETTING_VOTER, true, pval->intval);
		} else {
			vote(policy->recharge_voltage_votable, CAS_SETTING_VOTER, false, 0);
			vote(policy->recharge_soc_votable, CAS_SETTING_VOTER, false, 0);
		}
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		vote_debug("CAS: Set charging enable %d\n", pval->intval);
		vote(policy->usb_icl_votable, POLICY_SETTING_VOTER, (bool)!pval->intval, 0);
		break;
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		vote_debug("CAS: Set battery charging enable %d\n", pval->intval);
		vote(policy->battery_charging_enabled_votable, POLICY_SETTING_VOTER,
							(pval->intval ? false : true), 0);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		if (pval->intval < 0) {
			dev_info(cm->dev, "%s thermal control invalid argument\n", __func__);
			vote(policy->fcc_votable, ZTE_THERMAL_SETTING_VOTER, false, 0);
			break;
		}
		vote(policy->fcc_votable, ZTE_THERMAL_SETTING_VOTER, true, pval->intval);
		dev_info(cm->dev, "%s zte_thermal control fcc vote: %d\n", __func__, pval->intval);
		break;
	default:
		vote_debug("CAS: interface unsupported property %d\n", psp);
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int interface_property_is_writeable(struct power_supply *psy,
					enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case POWER_SUPPLY_PROP_INPUT_SUSPEND:
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_RECHARGE_SOC:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		return 1;
	default:
		break;
	}

	return 0;
}

static void interface_external_power_changed(struct power_supply *psy)
{
	vote_debug("power supply changed\n");
}

static enum power_supply_property interface_psy_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_SUSPEND,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_RECHARGE_SOC,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
};

static const struct power_supply_desc interface_psy_desc = {
	.name = "interface",
	.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.properties = interface_psy_props,
	.num_properties = ARRAY_SIZE(interface_psy_props),
	.get_property = interface_psy_get_property,
	.set_property = interface_psy_set_property,
	.external_power_changed = interface_external_power_changed,
	.property_is_writeable = interface_property_is_writeable,
};

static int hook_psy_dumper_vote_status(struct charger_manager *cm)
{
	struct charger_policy *policy = &(cm->policy);

	vote_debug("FV: %s[%d], FCC: %s[%d], TOPOFF: %s[%d], "
				"USB_ICL: %s[%d], RECH_SOC: %s[%d], RECH_VOLTAGE: %s[%d], "
				"CHARGING_ENABLE: %s[%d]\n",
			get_effective_client(policy->fcv_votable),
			get_effective_result(policy->fcv_votable) / 1000,
			get_effective_client(policy->fcc_votable),
			get_effective_result(policy->fcc_votable) / 1000,
			get_effective_client(policy->topoff_votable),
			get_effective_result(policy->topoff_votable),
			get_effective_client(policy->usb_icl_votable),
			get_effective_result(policy->usb_icl_votable) / 1000,
			get_effective_client(policy->recharge_soc_votable),
			get_effective_result(policy->recharge_soc_votable),
			get_effective_client(policy->recharge_voltage_votable),
			get_effective_result(policy->recharge_voltage_votable) / 1000,
			get_effective_client(policy->battery_charging_enabled_votable),
			get_effective_result(policy->battery_charging_enabled_votable));

	return 0;
}

static int hook_psy_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *pval)
{
	struct charger_manager *cm = power_supply_get_drvdata(psy);
	int ret, val1 = 0, val2 = 0;

	ret = charger_get_prop_by_name(cm->desc->psy_hardware, psp, &val1);

	if (ret) {
		dev_err(cm->dev, "get prop %s failed\n", cm->desc->psy_hardware);
		return ret;
	}

	pval->intval = val1;
	if (cm->desc->psy_hardware2) {
		ret = charger_get_prop_by_name(cm->desc->psy_hardware2, psp, &val2);

		if (ret) {
			dev_err(cm->dev, "get prop %s failed\n", cm->desc->psy_hardware2);
			return ret;
		}

		switch (psp) {
		case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
			/* fall through */
		case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
			pval->intval += val2;
			break;
		default:
			break;
		}
	}

	return ret;
}

static int hook_psy_set_property(struct power_supply *psy,
				enum power_supply_property psp,
				const union power_supply_propval *pval)
{
	struct charger_manager *cm = power_supply_get_drvdata(psy);
	struct charger_policy *policy = &(cm->policy);

	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		vote(policy->fcv_votable, OTHER_SETTING_VOTER, true, pval->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		vote(policy->fcc_votable, OTHER_SETTING_VOTER, true, pval->intval);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		vote(policy->usb_icl_votable, OTHER_SETTING_VOTER, true, pval->intval);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		vote(policy->battery_charging_enabled_votable, OTHER_SETTING_VOTER, true, pval->intval);

		/* start tuing vindpm after enable charigng function when plug in*/
		if (pval->intval) {
			charger_set_prop_by_name(cm->desc->psy_hardware,
						POWER_SUPPLY_PROP_TUNING_VINDPM, pval->intval);
			if (cm->desc->psy_hardware2)
				charger_set_prop_by_name(cm->desc->psy_hardware2,
						POWER_SUPPLY_PROP_TUNING_VINDPM, pval->intval);
		}
		break;
	case POWER_SUPPLY_PROP_FEED_WATCHDOG:
		if (cm->desc->psy_hardware2)
			charger_set_prop_by_name(cm->desc->psy_hardware2,
				POWER_SUPPLY_PROP_FEED_WATCHDOG, pval->intval);

		charger_set_prop_by_name(cm->desc->psy_hardware,
			POWER_SUPPLY_PROP_FEED_WATCHDOG, pval->intval);
		hook_psy_dumper_vote_status(cm);
		break;
	default:
		return -EINVAL;
	}


	return 0;
}

static int hook_property_is_writeable(struct power_supply *psy,
					enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		return 1;
	default:
		break;
	}

	return 0;
}

static void hook_external_power_changed(struct power_supply *psy)
{
	vote_debug("power supply changed\n");
}

static enum power_supply_property hook_psy_props[] = {
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
};

static const struct power_supply_desc hook_psy_desc = {
	.name = HOOK_PSY_NAME,
	.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.properties = hook_psy_props,
	.num_properties = ARRAY_SIZE(hook_psy_props),
	.get_property = hook_psy_get_property,
	.set_property = hook_psy_set_property,
	.external_power_changed = hook_external_power_changed,
	.property_is_writeable = hook_property_is_writeable,
};

static int usb_property_is_writeable(struct power_supply *psy,
					enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_LPM_USB_DISCON:
		return 1;
	default:
		break;
	}

	return 0;
}

static int usb_psy_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *pval)
{
	struct charger_manager *cm = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		pval->intval = !!cm->policy.limit;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		if (!!cm->policy.limit &&
			(cm->policy.usb_phy->chg_type == SDP_TYPE ||
			cm->policy.usb_phy->chg_type == CDP_TYPE ||
			cm->policy.usb_phy->chg_type == ACA_TYPE))
			pval->intval = 1;
		else
			pval->intval = 0;
		break;
	case POWER_SUPPLY_PROP_LPM_USB_DISCON:
		pval->intval = 1;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int usb_psy_set_property(struct power_supply *psy,
		     enum power_supply_property psp,
		    const union power_supply_propval *pval)
{

	switch (psp) {
	case POWER_SUPPLY_PROP_LPM_USB_DISCON:
		if (pval->intval == 0)
		musb_lpm_usb_disconnect();
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void usb_external_power_changed(struct power_supply *psy)
{
	vote_debug("usb power supply changed\n");
}

static enum power_supply_property usb_psy_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_LPM_USB_DISCON,
};

static const struct power_supply_desc usb_psy_desc = {
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = usb_psy_props,
	.num_properties = ARRAY_SIZE(usb_psy_props),
	.get_property = usb_psy_get_property,
	.set_property = usb_psy_set_property,
	.property_is_writeable = usb_property_is_writeable,
	.external_power_changed = usb_external_power_changed,
};

static int ac_psy_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *pval)
{
	struct charger_manager *cm = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (!!cm->policy.limit && (cm->policy.usb_phy->chg_type == DCP_TYPE ||
			cm->policy.usb_phy->chg_type == NON_DCP_TYPE ||
			cm->policy.usb_phy->chg_type == UNKNOWN_TYPE))
			pval->intval = 1;
		else
			pval->intval = 0;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void ac_external_power_changed(struct power_supply *psy)
{
	vote_debug("ac power supply changed\n");
}

static enum power_supply_property ac_psy_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static const struct power_supply_desc ac_psy_desc = {
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_USB_DCP,
	.properties = ac_psy_props,
	.num_properties = ARRAY_SIZE(ac_psy_props),
	.get_property = ac_psy_get_property,
	.external_power_changed = ac_external_power_changed,
};

static int charger_manager_init_psy(struct charger_manager *cm)
{
	struct charger_policy *policy = &(cm->policy);
	struct power_supply_config batt_cfg = {};

	batt_cfg.drv_data = cm;
	batt_cfg.of_node = cm->dev->of_node;
	policy->interface_psy = devm_power_supply_register(cm->dev,
					   &interface_psy_desc,
					   &batt_cfg);
	if (IS_ERR(policy->interface_psy)) {
		vote_error("Couldn't register interface_psy power supply\n");
		return PTR_ERR(policy->interface_psy);
	}

	policy->hook_psy = devm_power_supply_register(cm->dev,
					   &hook_psy_desc,
					   &batt_cfg);
	if (IS_ERR(policy->hook_psy)) {
		vote_error("Couldn't register hook_psy power supply\n");
		return PTR_ERR(policy->hook_psy);
	}

	policy->usb_power_phy = devm_power_supply_register(cm->dev,
						   &usb_psy_desc,
						   &batt_cfg);
	if (IS_ERR(policy->usb_power_phy)) {
		vote_error("Couldn't register usb_power_phy power supply\n");
		return PTR_ERR(policy->usb_power_phy);
	}

	policy->ac_power_phy = devm_power_supply_register(cm->dev,
						   &ac_psy_desc,
						   &batt_cfg);
	if (IS_ERR(policy->ac_power_phy)) {
		vote_error("Couldn't register ac_power_phy power supply\n");
		return PTR_ERR(policy->ac_power_phy);
	}

	return 0;
}


static int charger_fcc_vote_callback(struct votable *votable,
			void *data, int max_fcc_ua, const char *client)
{
	struct charger_manager *cm = (struct charger_manager *)data;
	int ret;

	vote_debug("callback set fcc %d\n", max_fcc_ua);

	if (cm->should_en_parallel_chg) {
		charger_set_prop_by_name(cm->desc->psy_hardware,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT, max_fcc_ua / 2);

		ret = charger_set_prop_by_name(cm->desc->psy_hardware2,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT, max_fcc_ua / 2);

		if (ret) {
			dev_err(cm->dev, "set prop %s failed\n", cm->desc->psy_hardware2);
			return ret;
		}
		dev_info(cm->dev, "set chg #2 %s CONSTANT_CHARGE_CURRENT val %d\n",
			cm->desc->psy_hardware2, max_fcc_ua/2);
	} else {
		charger_set_prop_by_name(cm->desc->psy_hardware,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT, max_fcc_ua);

		if (max_fcc_ua == 0)
			charger_set_prop_by_name(cm->desc->psy_hardware2,
				POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT, max_fcc_ua);
	}

	return 0;
}

static int charger_fcv_vote_callback(struct votable *votable,
			void *data, int max_fcv_uv, const char *client)
{
	struct charger_manager *cm = (struct charger_manager *)data;
	int ret;

	vote_debug("callback set fcv: %d\n", max_fcv_uv);
	max_fcv_uv += cm->desc->fullbatt_comp_uV;
	vote_debug("callback set fcv + fullbatt_comp_uV: %d\n", max_fcv_uv);
	charger_set_prop_by_name(cm->desc->psy_hardware,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE, max_fcv_uv);

	if (cm->should_en_parallel_chg) {
		ret = charger_set_prop_by_name(cm->desc->psy_hardware2,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE, max_fcv_uv);

		if (ret) {
			dev_err(cm->dev, "set prop %s failed\n", cm->desc->psy_hardware2);
			return ret;
		}
		dev_info(cm->dev, "set chg #2 %s CONSTANT_CHARGE_VOLTAGE val %d\n",
			cm->desc->psy_hardware2, max_fcv_uv);
	}

	return 0;
}

static int charger_topoff_vote_callback(struct votable *votable,
			void *data, int min_topoff_ua, const char *client)
{
	struct charger_manager *cm = (struct charger_manager *)data;
	int ret;

	vote_debug("callback set topoff %d\n", min_topoff_ua);

	if (cm->should_en_parallel_chg) {
		charger_set_prop_by_name(cm->desc->psy_hardware,
					POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT, min_topoff_ua / 2);

		ret = charger_set_prop_by_name(cm->desc->psy_hardware2,
			POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT, min_topoff_ua / 2);

		if (ret) {
			dev_err(cm->dev, "set %s prop CHARGE_TERM_CURRENT failed\n", cm->desc->psy_hardware2);
			return ret;
		}
		dev_info(cm->dev, "set chg #2 %s CHARGE_TERM_CURRENT val %d\n",
			cm->desc->psy_hardware2, min_topoff_ua / 2);
	} else
		charger_set_prop_by_name(cm->desc->psy_hardware,
			POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT, min_topoff_ua);

	return 0;
}

static int charger_recharge_soc_vote_callback(struct votable *votable,
			void *data, int recharge_soc, const char *client)
{
	struct charger_manager *cm = (struct charger_manager *)data;

	vote_debug("callback set recharge_soc %d\n", recharge_soc);

	charger_set_prop_by_name(cm->desc->psy_hardware,
			POWER_SUPPLY_PROP_RECHARGE_SOC, recharge_soc);

	return 0;
}

static int charger_recharge_voltage_vote_callback(struct votable *votable,
			void *data, int recharge_voltage, const char *client)
{
	struct charger_manager *cm = (struct charger_manager *)data;

	vote_debug("callback set recharge_voltage %d\n", recharge_voltage);

	charger_set_prop_by_name(cm->desc->psy_hardware,
			POWER_SUPPLY_PROP_RECHARGE_SOC, recharge_voltage);

	return 0;
}

static int charger_usb_icl_vote_callback(struct votable *votable,
			void *data, int max_icl_ua, const char *client)
{
	struct charger_manager *cm = (struct charger_manager *)data;
	int ret;

	vote_debug("callback set usb_icl %d\n", max_icl_ua);

	if (cm->should_en_parallel_chg) {
		charger_set_prop_by_name(cm->desc->psy_hardware,
				POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, max_icl_ua / 2);

		ret = charger_set_prop_by_name(cm->desc->psy_hardware2,
					POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, max_icl_ua / 2);

		if (ret) {
			dev_err(cm->dev, "set prop %s failed\n", cm->desc->psy_hardware2);
			return ret;
		}
		dev_info(cm->dev, "set chg #2 %s INPUT_CURRENT_LIMIT val %d\n",
			cm->desc->psy_hardware2, max_icl_ua / 2);
	} else
		charger_set_prop_by_name(cm->desc->psy_hardware,
				POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, max_icl_ua);

	return 0;
}

static int charger_battery_charging_enabled_vote_callback(struct votable *votable,
			void *data, int enable, const char *client)
{
	struct charger_manager *cm = (struct charger_manager *)data;

	vote_debug("callback set enable %d\n", enable);

	charger_set_prop_by_name(cm->desc->psy_hardware,
			POWER_SUPPLY_PROP_STATUS, enable);

	if (cm->desc->psy_hardware2)
		charger_set_prop_by_name(cm->desc->psy_hardware2,
				POWER_SUPPLY_PROP_STATUS, cm->should_en_parallel_chg);

	return 0;
}

/*
static int charger_manager_policy_check_vote_enable(struct charger_manager *cm)
{
	struct charger_policy *policy = &(cm->policy);
	struct charger_desc *desc = cm->desc;

	if (desc->psy_charger_stat[0] != NULL) {
		strncpy(policy->hardware_pyh, desc->psy_charger_stat[0],
					strlen(desc->psy_charger_stat[0]));

		strncpy(policy->hardware_pyh, desc->psy_charger_stat[0],
					strlen(desc->psy_charger_stat[0]));
	}

	return 0;
}
*/

static int charger_manager_policy_init_default_voter(struct charger_manager *cm)
{
	struct charger_policy *policy = &(cm->policy);
	int rc = 0;

	policy->fcc_votable = create_votable("FCC", VOTE_MIN,
					charger_fcc_vote_callback,
					cm);
	if (IS_ERR(policy->fcc_votable)) {
		rc = PTR_ERR(policy->fcc_votable);
		vote_error("create %s failed\n", "fcc_votable");
		goto destroy_votable_loop;
	}

	policy->fcv_votable = create_votable("FV", VOTE_MIN,
					charger_fcv_vote_callback,
					cm);
	if (IS_ERR(policy->fcv_votable)) {
		rc = PTR_ERR(policy->fcv_votable);
		vote_error("create %s failed\n", "fcv_votable");
		goto destroy_votable_loop;
	}

	policy->topoff_votable = create_votable("TOPOFF", VOTE_MAX,
					charger_topoff_vote_callback,
					cm);
	if (IS_ERR(policy->topoff_votable)) {
		rc = PTR_ERR(policy->topoff_votable);
		vote_error("create %s failed\n", "topoff_votable");
		goto destroy_votable_loop;
	}

	policy->recharge_soc_votable = create_votable("RECH_SOC", VOTE_MIN,
					charger_recharge_soc_vote_callback,
					cm);
	if (IS_ERR(policy->recharge_soc_votable)) {
		rc = PTR_ERR(policy->recharge_soc_votable);
		vote_error("create %s failed\n", "recharge_soc_votable");
		goto destroy_votable_loop;
	}

	policy->recharge_voltage_votable = create_votable("RECH_VOLTAGE", VOTE_MAX,
					charger_recharge_voltage_vote_callback,
					cm);
	if (IS_ERR(policy->recharge_voltage_votable)) {
		rc = PTR_ERR(policy->recharge_voltage_votable);
		vote_error("create %s failed\n", "recharge_voltage_votable");
		goto destroy_votable_loop;
	}

	policy->usb_icl_votable = create_votable("USB_ICL", VOTE_MIN,
					charger_usb_icl_vote_callback,
					cm);
	if (IS_ERR(policy->usb_icl_votable)) {
		rc = PTR_ERR(policy->usb_icl_votable);
		vote_error("create %s failed\n", "usb_icl_votable");
		goto destroy_votable_loop;
	}

	policy->battery_charging_enabled_votable = create_votable("CHARGING_ENABLE", VOTE_MIN,
					charger_battery_charging_enabled_vote_callback,
					cm);
	if (IS_ERR(policy->battery_charging_enabled_votable)) {
		rc = PTR_ERR(policy->battery_charging_enabled_votable);
		vote_error("create %s failed\n", "battery_charging_enabled_votable");
		goto destroy_votable_loop;
	}

	vote(policy->usb_icl_votable, DEFAULT_SETTING_VOTER,
										true, DEFAULT_SETTING_ICL);

	vote(policy->topoff_votable, DEFAULT_SETTING_VOTER,
										true, DEFAULT_SETTING_TOPOFF);

	vote(policy->recharge_soc_votable, DEFAULT_SETTING_VOTER,
										true, DEFAULT_SETTING_RECHARGE_SOC);

	vote(policy->recharge_voltage_votable, DEFAULT_SETTING_VOTER,
										true, DEFAULT_SETTING_RECHARGE_VOLTAGE);

	vote(policy->battery_charging_enabled_votable, DEFAULT_SETTING_VOTER,
										true, 1);
	return 0;
destroy_votable_loop:
	if (policy->fcc_votable)
		destroy_votable(policy->fcc_votable);
	if (policy->fcv_votable)
		destroy_votable(policy->fcv_votable);
	if (policy->topoff_votable)
		destroy_votable(policy->topoff_votable);
	if (policy->recharge_soc_votable)
		destroy_votable(policy->recharge_soc_votable);
	if (policy->recharge_voltage_votable)
		destroy_votable(policy->recharge_voltage_votable);
	if (policy->usb_icl_votable)
		destroy_votable(policy->usb_icl_votable);
	if (policy->battery_charging_enabled_votable)
		destroy_votable(policy->battery_charging_enabled_votable);

	return -EINVAL;
}

static int charger_manager_policy_init_battery_voter(struct charger_manager *cm)
{
	struct charger_policy *policy = &(cm->policy);
	struct power_supply_battery_info bat_info = {0, };
	int ret = 0;

	ret = power_supply_get_battery_info(policy->hook_psy, &bat_info, 0);
	if (ret) {
		vote_debug("no battery information is supplied\n");

		/*
		 * If no battery information is supplied, we should set
		 * default charge termination current to 100 mA, and default
		 * charge termination voltage to 4.2V.
		 */
		policy->battery_current.sdp_limit = 500000;
		policy->battery_current.sdp_cur = 500000;
		policy->battery_current.dcp_limit = 1000000;
		policy->battery_current.dcp_cur = 500000;
		policy->battery_current.cdp_limit = 1000000;
		policy->battery_current.cdp_cur = 500000;
		policy->battery_current.unknown_limit = 1000000;
		policy->battery_current.unknown_cur = 500000;

		policy->charge_term_current_ua = 150000;
		policy->constant_charge_current_max_ua = 500000;
		policy->constant_charge_voltage_max_uv = 4200000;
	} else {
		policy->battery_current.sdp_limit = bat_info.cur.sdp_limit;
		policy->battery_current.sdp_cur = bat_info.cur.sdp_cur;
		vote_debug("sdp usbin %d, battin %d.\n",
					bat_info.cur.sdp_limit, bat_info.cur.sdp_cur);

		policy->battery_current.dcp_limit = bat_info.cur.dcp_limit;
		policy->battery_current.dcp_cur = bat_info.cur.dcp_cur;
		vote_debug("dcp usbin %d, battin %d.\n",
					bat_info.cur.dcp_limit, bat_info.cur.dcp_cur);

		policy->battery_current.cdp_limit = bat_info.cur.cdp_limit;
		policy->battery_current.cdp_cur = bat_info.cur.cdp_cur;
		vote_debug("cdp usbin %d, battin %d.\n",
					bat_info.cur.cdp_limit, bat_info.cur.cdp_cur);

		policy->battery_current.unknown_limit = bat_info.cur.unknown_limit;
		policy->battery_current.unknown_cur = bat_info.cur.unknown_cur;
		vote_debug("unknown usbin %d, battin %d.\n",
					bat_info.cur.unknown_limit, bat_info.cur.unknown_cur);

		policy->charge_term_current_ua = bat_info.charge_term_current_ua;
		policy->constant_charge_current_max_ua = bat_info.constant_charge_current_max_ua;
		policy->constant_charge_voltage_max_uv = bat_info.constant_charge_voltage_max_uv;
		vote_debug("charge_term_current_ua %d, constant curr max %d, constant vol max %d\n",
					bat_info.charge_term_current_ua, bat_info.constant_charge_current_max_ua,
					bat_info.constant_charge_voltage_max_uv);
	}

	power_supply_put_battery_info(policy->hook_psy, &bat_info);
	vote(policy->usb_icl_votable, BATTERY_SETTING_VOTER,
									true, policy->battery_current.dcp_limit);

	vote(policy->topoff_votable, BATTERY_SETTING_VOTER,
									true, policy->charge_term_current_ua / 1000);

	vote(policy->fcc_votable, BATTERY_SETTING_VOTER,
									true, policy->constant_charge_current_max_ua);

	vote(policy->fcv_votable, BATTERY_SETTING_VOTER,
									true, policy->constant_charge_voltage_max_uv);

	return 0;
}

static void policy_usb_change_handler_work(struct work_struct *work)
{
	int usb_icl = 0, battery_fcc = 0;
	struct charger_policy *info =
			container_of(work, struct charger_policy, usb_changed_work.work);

	struct charger_manager *cm =
			container_of(info, struct charger_manager, policy);

	cm->should_en_parallel_chg = false;
	cm->parallel_charge_enabled = false;

	if (info->limit) {
		vote_debug("sprd charger usb plug in\n");
		/* set current limitation and start to charge */
		switch (info->usb_phy->chg_type) {
		case SDP_TYPE:
			usb_icl = info->battery_current.sdp_limit;
			battery_fcc = info->battery_current.sdp_cur;
			break;
		case DCP_TYPE:
			usb_icl = info->battery_current.dcp_limit;
			battery_fcc = info->battery_current.dcp_cur;
			break;
		case CDP_TYPE:
			usb_icl = info->battery_current.cdp_limit;
			battery_fcc = info->battery_current.cdp_cur;
			break;
		case NON_DCP_TYPE:
		/* nonstandard dcp charger usb default parameters */
		default:
			usb_icl = info->battery_current.unknown_limit;
			battery_fcc = info->battery_current.unknown_cur;
		}

		vote_debug("chg_type: %d, usb_icl %d, fcc %d\n", info->usb_phy->chg_type, usb_icl, battery_fcc);

		charger_set_prop_by_name(cm->desc->psy_hardware,
							POWER_SUPPLY_PROP_SET_WATCHDOG_TIMER, cm->desc->wdt_interval);

		vote(info->fcc_votable, CHARGER_TYPE_VOTER, true, battery_fcc);

		vote(info->usb_icl_votable, CHARGER_TYPE_VOTER, true, usb_icl);

		vote(info->topoff_votable, BATTERY_SETTING_VOTER,
									true, info->charge_term_current_ua / 1000);
	} else {
		/* Stop charging */
		vote_debug("sprd charger usb plug out\n");

		charger_set_prop_by_name(cm->desc->psy_hardware,
							POWER_SUPPLY_PROP_SET_WATCHDOG_TIMER, 0);

		vote(info->fcc_votable, CHARGER_TYPE_VOTER, false, 0);

		vote(info->usb_icl_votable, CHARGER_TYPE_VOTER, true, 100000);
		vote(info->usb_icl_votable, THERMAL_SETTING_VOTER, false, 0);
		vote(info->fcc_votable, THERMAL_SETTING_VOTER, false, 0);
	}

	cm_notify_event(info->hook_psy, CM_EVENT_CHG_START_STOP, NULL);
}


static int policy_usb_change_callback(struct notifier_block *nb,
				       unsigned long limit, void *data)
{
	struct charger_policy *info =
		container_of(nb, struct charger_policy, usb_notify);

	info->limit = limit;

	vote_debug("policy_usb_change_callback: %d\n", (uint)limit);

	schedule_delayed_work(&info->usb_changed_work, msecs_to_jiffies(100));

	return NOTIFY_OK;
}

static int charger_manager_policy_init(struct charger_manager *cm)
{
	int ret = 0;

	charger_manager_init_psy(cm);

	charger_manager_policy_init_default_voter(cm);

	charger_manager_policy_init_battery_voter(cm);

	INIT_DELAYED_WORK(&(cm->policy.usb_changed_work), policy_usb_change_handler_work);

	cm->policy.usb_phy = devm_usb_get_phy_by_phandle(cm->dev, "phys", 0);
	if (IS_ERR(cm->policy.usb_phy)) {
		vote_error("######@failed to find USB phy %d\n", (int)PTR_ERR(cm->policy.usb_phy));
		return PTR_ERR(cm->policy.usb_phy);
	}

	cm->policy.usb_notify.notifier_call = policy_usb_change_callback;
	ret = usb_register_notifier(cm->policy.usb_phy, &cm->policy.usb_notify);
	if (ret) {
		vote_error("failed to register notifier:%d\n", ret);
		return ret;
	}

	return 0;
}
#endif

static void charge_debug_works(struct work_struct *work)
{
	struct delayed_work *dwork = NULL;
	struct charger_manager *cm = NULL;

	if (!work)
		return;

	dwork = to_delayed_work(work);
	if (!dwork)
		return;

	cm = container_of(dwork,
			struct charger_manager, charge_debug_work);
	if (!cm)
		return;

	cm->desc->cap_debug = cm->desc->cap_buff_debug;
	pr_info("%s: enter, set cap_debug %d\n", __func__, cm->desc->cap_debug);

	power_supply_changed(cm->charger_psy);
}

int cap_debug_set(const char *val, const void *arg)
{
	struct charger_manager *cm = (struct charger_manager *) arg;
	int cap_debug = -1;
	int delay_secs = 0;
	int num = 0;

	if (!cm) {
		pr_info("%s: cm is null\n", __func__);
		return -EINVAL;
	}

	num = sscanf(val, "%d %d", &delay_secs, &cap_debug);

	pr_info("%s: set cap_debug %d after %ds\n", __func__, cap_debug, delay_secs);

	if (num != 2) {
		pr_err("%s: wrong args num %d. usage: echo <delay> <cap> > cap_debug\n", __func__, num);
		return -EINVAL;
	}

	cm->desc->cap_buff_debug = cap_debug;

	if (!cm_wq)
		return -EFAULT;
	queue_delayed_work(cm_wq, &cm->charge_debug_work, msecs_to_jiffies(delay_secs * 1000));

	return 0;
}

int cap_debug_get(char *val, const void *arg)
{
	struct charger_manager *cm = (struct charger_manager *) arg;

	if (!cm) {
			pr_info("%s: cm is null\n", __func__);
			return -EINVAL;
	}

	pr_info("%s: get cap_debug: %d\n", __func__, cm->desc->cap_debug);

	return snprintf(val, PAGE_SIZE, "%d", cm->desc->cap_debug);

	return 0;
}

struct zte_misc_ops cap_debug_node = {
	.node_name = "cap_debug",
	.set = cap_debug_set,
	.get = cap_debug_get,
	.free = NULL,
	.arg = NULL,
};

int thermal_control_en_set(const char *val, const void *arg)
{
	struct charger_manager *cm = (struct charger_manager *) arg;
	int thermal_control_en = -1;
	int num = 0;

	if (!cm) {
		pr_info("%s: cm is null\n", __func__);
		return -EINVAL;
	}

	num = sscanf(val, "%d", &thermal_control_en);

	pr_info("%s: set thermal_control_en %d\n", __func__, thermal_control_en);

	if (num != 1) {
		pr_err("%s: wrong args num %d. usage: echo <0 or 1> > thermal_control_en\n", __func__, num);
		return -EINVAL;
	}

	if (thermal_control_en >= 1)
		cm->desc->thermal_control_en = true;
	else
		cm->desc->thermal_control_en = false;

	return 0;
}

int thermal_control_en_get(char *val, const void *arg)
{
	struct charger_manager *cm = (struct charger_manager *) arg;

	if (!cm) {
			pr_info("%s: cm is null\n", __func__);
			return -EINVAL;
	}

	pr_info("%s: get thermal_control_en: %d\n", __func__, cm->desc->thermal_control_en);

	return snprintf(val, PAGE_SIZE, "%d", cm->desc->thermal_control_en);

	return 0;
}

struct zte_misc_ops thermal_control_en_node = {
	.node_name = "thermal_control_en",
	.set = thermal_control_en_set,
	.get = thermal_control_en_get,
	.free = NULL,
	.arg = NULL,
};

bool zte_screen_on = false;
int zte_screen_on_get(char *val, const void *arg)
{
	return snprintf(val, PAGE_SIZE, "%u", zte_screen_on);
}

static int zte_screen_on_set(const char *val, const void *arg)
{
	struct charger_manager *cm = (struct charger_manager *) arg;

	if (cm != NULL  && val != NULL) {
		if (sscanf(val, "%d", &zte_screen_on) != 1) {
			pr_err("%s: sscanf screen_on failed\n", __func__);
			return -EINVAL;
		}

		pr_info("%s: set screen_on node %d\n", __func__, zte_screen_on);
		if (cm->policy.usb_phy->chg_type != UNKNOWN_TYPE)
			if (cm->charger_psy)
				power_supply_changed(cm->charger_psy);
	} else {
		pr_err("%s: cm is null\n", __func__);
		return -EINVAL;
	}
	return 0;
}

struct zte_misc_ops zte_screen_on_node = {
	.node_name = "screen_on",
	.set = zte_screen_on_set,
	.get = zte_screen_on_get,
	.free = NULL,
	.arg = NULL,
};

static int charger_manager_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct charger_desc *desc = cm_get_drv_data(pdev);
	struct charger_manager *cm;
	int ret, i = 0;
	union power_supply_propval val;
	struct power_supply *fuel_gauge;
	struct power_supply_config psy_cfg = {};
	struct timespec64 cur_time;
#ifdef CONFIG_VENDOR_POWER_VOTER
	unsigned int min = 0, max = 0;
#endif

	if (IS_ERR(desc)) {
		dev_err(&pdev->dev, "No platform data (desc) found\n");
		return PTR_ERR(desc);
	}

	pr_info("%s enter\n", __func__);

	cm = devm_kzalloc(&pdev->dev, sizeof(*cm), GFP_KERNEL);
	if (!cm)
		return -ENOMEM;

	/* Basic Values. Unspecified are Null or 0 */
	cm->dev = &pdev->dev;
	cm->desc = desc;
	psy_cfg.drv_data = cm;
	cm->health = POWER_SUPPLY_HEALTH_GOOD;
	cm->desc->cap_debug = -1;
	cm->desc->thermal_control_en = true;
	cm->batt_id_index = 0;

	/* Initialize alarm timer */
	if (alarmtimer_get_rtcdev()) {
		cm_timer = devm_kzalloc(cm->dev, sizeof(*cm_timer), GFP_KERNEL);
		if (!cm_timer)
			return -ENOMEM;
		alarm_init(cm_timer, ALARM_BOOTTIME, NULL);
	}

	/*
	 * Some of the following do not need to be errors.
	 * Users may intentionally ignore those features.
	 */
	if (desc->fullbatt_uV == 0) {
		dev_info(&pdev->dev, "Ignoring full-battery voltage threshold as it is not supplied\n");
	}

	if (desc->fullbatt_uA == 0)
		dev_info(&pdev->dev, "Ignoring full-battery current threshold as it is not supplied\n");

	if (!desc->fullbatt_vchkdrop_ms || !desc->fullbatt_vchkdrop_uV) {
		dev_info(&pdev->dev, "Disabling full-battery voltage drop checking mechanism as it is not supplied\n");
		desc->fullbatt_vchkdrop_ms = 0;
		desc->fullbatt_vchkdrop_uV = 0;
	}
	if (desc->fullbatt_soc == 0) {
		dev_info(&pdev->dev, "Ignoring full-battery soc(state of charge) threshold as it is not supplied\n");
	}
	if (desc->fullbatt_full_capacity == 0) {
		dev_info(&pdev->dev, "Ignoring full-battery full capacity threshold as it is not supplied\n");
	}

	if (!desc->charger_regulators || desc->num_charger_regulators < 1) {
		dev_err(&pdev->dev, "charger_regulators undefined\n");
		return -EINVAL;
	}

	if (!desc->psy_charger_stat || !desc->psy_charger_stat[0]) {
		dev_err(&pdev->dev, "No power supply defined\n");
		return -EINVAL;
	}

	if (!desc->psy_fuel_gauge) {
		dev_err(&pdev->dev, "No fuel gauge power supply defined\n");
		return -EINVAL;
	}

	if (desc->polling_interval_ms == 0 ||
	    msecs_to_jiffies(desc->polling_interval_ms) <= CM_JIFFIES_SMALL) {
		dev_err(&pdev->dev, "polling_interval_ms is too small\n");
		return -EINVAL;
	}

	if (!desc->charging_max_duration_ms ||
			!desc->discharging_max_duration_ms) {
		dev_info(&pdev->dev, "Cannot limit charging duration checking mechanism to prevent overcharge/overheat and control discharging duration\n");
		desc->charging_max_duration_ms = 0;
		desc->discharging_max_duration_ms = 0;
	}

	if (!desc->charge_voltage_max || !desc->charge_voltage_drop) {
		dev_info(&pdev->dev, "Cannot validate charge voltage\n");
		desc->charge_voltage_max = 0;
		desc->charge_voltage_drop = 0;
	}

	platform_set_drvdata(pdev, cm);

	memcpy(&cm->charger_psy_desc, &psy_default, sizeof(psy_default));

	if (!desc->psy_name)
		strncpy(cm->psy_name_buf, psy_default.name, PSY_NAME_MAX);
	else
		strncpy(cm->psy_name_buf, desc->psy_name, PSY_NAME_MAX);
	cm->charger_psy_desc.name = cm->psy_name_buf;

	/* Allocate for psy properties because they may vary */
	cm->charger_psy_desc.properties = devm_kzalloc(&pdev->dev,
				sizeof(enum power_supply_property)
				* (ARRAY_SIZE(default_charger_props) +
				NUM_CHARGER_PSY_OPTIONAL), GFP_KERNEL);
	if (!cm->charger_psy_desc.properties)
		return -ENOMEM;

	memcpy(cm->charger_psy_desc.properties, default_charger_props,
		sizeof(enum power_supply_property) *
		ARRAY_SIZE(default_charger_props));
	cm->charger_psy_desc.num_properties = psy_default.num_properties;

	/* Find which optional psy-properties are available */
	fuel_gauge = power_supply_get_by_name(desc->psy_fuel_gauge);
	if (!fuel_gauge) {
		dev_err(&pdev->dev, "Cannot find power supply \"%s\"\n",
			desc->psy_fuel_gauge);
		return -ENODEV;
	}
	if (!power_supply_get_property(fuel_gauge,
					  POWER_SUPPLY_PROP_CHARGE_NOW, &val)) {
		cm->charger_psy_desc.properties[cm->charger_psy_desc.num_properties] =
				POWER_SUPPLY_PROP_CHARGE_NOW;
		cm->charger_psy_desc.num_properties++;
	}
	if (!power_supply_get_property(fuel_gauge,
					  POWER_SUPPLY_PROP_CURRENT_NOW,
					  &val)) {
		cm->charger_psy_desc.properties[cm->charger_psy_desc.num_properties] =
				POWER_SUPPLY_PROP_CURRENT_NOW;
		cm->charger_psy_desc.num_properties++;
	}

	ret = get_boot_cap(cm, &cm->desc->cap);
	dev_info(&pdev->dev, "%s: init desc->cap = %d\n", __func__, cm->desc->cap);
	cm->desc->cap = cm_capacity_remap(cm, cm->desc->cap);

	if (ret) {
		dev_err(&pdev->dev, "Failed to get initial battery capacity\n");
		return -EPROBE_DEFER;
	}

#ifdef CONFIG_VENDOR_POWER_VOTER
	cm->should_en_parallel_chg = false;
	cm->parallel_charge_enabled = false;
	charger_manager_policy_init(cm);
#endif

	/* Check if charger's supplies are present at probe */
	for (i = 0; desc->psy_charger_stat[i]; i++) {
		struct power_supply *psy;

		psy = power_supply_get_by_name(desc->psy_charger_stat[i]);
		if (!psy) {
			dev_err(&pdev->dev, "Cannot find power supply \"%s\"\n",
				desc->psy_charger_stat[i]);
			return -EPROBE_DEFER;
		}
		power_supply_put(psy);
	}

	cm->desc->thm_adjust_cur = -EINVAL;

	ret = cm_get_battery_temperature_by_psy(cm, &cm->desc->temperature);
	if (ret) {
		dev_err(cm->dev, "failed to get battery temperature\n");
		return ret;
	}

	cur_time = ktime_to_timespec64(ktime_get_boottime());
	cm->desc->update_capacity_time = cur_time.tv_sec;
	cm->desc->last_query_time = cur_time.tv_sec;

	ret = cm_init_thermal_data(cm, fuel_gauge);
	if (ret) {
		dev_err(&pdev->dev, "Failed to initialize thermal data\n");
		cm->desc->measure_battery_temp = false;
	}
	cm->desc->measure_battery_temp = false;
	power_supply_put(fuel_gauge);

	INIT_DELAYED_WORK(&cm->fullbatt_vchk_work, fullbatt_vchk);
	INIT_DELAYED_WORK(&cm->cap_update_work, cm_batt_works);
	INIT_DELAYED_WORK(&cm->charge_debug_work, charge_debug_works);

	psy_cfg.of_node = np;
	cm->charger_psy = power_supply_register(&pdev->dev,
						&cm->charger_psy_desc,
						&psy_cfg);
	if (IS_ERR(cm->charger_psy)) {
		dev_err(&pdev->dev, "Cannot register charger-manager with name \"%s\"\n",
			cm->charger_psy_desc.name);
		return PTR_ERR(cm->charger_psy);
	}
	cm->charger_psy->supplied_to = charger_manager_supplied_to;
	cm->charger_psy->num_supplicants =
		ARRAY_SIZE(charger_manager_supplied_to);
/*
	wireless_main.psy = power_supply_register(&pdev->dev, &wireless_main.psd, NULL);
	if (IS_ERR(wireless_main.psy)) {
		dev_err(&pdev->dev, "Cannot register wireless_main.psy with name \"%s\"\n",
			wireless_main.psd.name);
		return PTR_ERR(wireless_main.psy);

	}

	ac_main.psy = power_supply_register(&pdev->dev, &ac_main.psd, NULL);
	if (IS_ERR(ac_main.psy)) {
		dev_err(&pdev->dev, "Cannot register usb_main.psy with name \"%s\"\n",
			ac_main.psd.name);
		return PTR_ERR(ac_main.psy);

	}

	usb_main.psy = power_supply_register(&pdev->dev, &usb_main.psd, NULL);
	if (IS_ERR(usb_main.psy)) {
		dev_err(&pdev->dev, "Cannot register usb_main.psy with name \"%s\"\n",
			usb_main.psd.name);
		return PTR_ERR(usb_main.psy);

	}
*/

	/* Register extcon device for charger cable */
	ret = charger_manager_register_extcon(cm);
	if (ret < 0) {
		dev_err(&pdev->dev, "Cannot initialize extcon device\n");
		goto err_reg_extcon;
	}

	/* Register sysfs entry for charger(regulator) */
	ret = charger_manager_register_sysfs(cm);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"Cannot initialize sysfs entry of regulator\n");
		goto err_reg_sysfs;
	}

	/* Add to the list */
	mutex_lock(&cm_list_mtx);
	list_add(&cm->entry, &cm_list);
	mutex_unlock(&cm_list_mtx);

	/*
	 * Charger-manager is capable of waking up the systme from sleep
	 * when event is happened through cm_notify_event()
	 */
	device_init_wakeup(&pdev->dev, true);
	device_set_wakeup_capable(&pdev->dev, false);

	if (cm_event_type)
		cm_notify_type_handle(cm, cm_event_type, cm_event_msg);

	/*
	 * Charger-manager have to check the charging state right after
	 * tialization of charger-manager and then update current charging
	 * state.
	 */
	cm_monitor();

	schedule_work(&setup_polling);

	cm->track.cap_tracking =
		device_property_read_bool(&pdev->dev, "cm-capacity-track");

	/* zte add, always disable cap_tracking function */
	cm->track.cap_tracking = false;

	if (cm->track.cap_tracking) {
		ret = cm_get_bat_info(cm);
		if (ret) {
			dev_err(&pdev->dev, "Failed to get battery information\n");
			goto err_reg_sysfs;
		}

		cm_track_capacity_init(cm);
	}

	queue_delayed_work(system_power_efficient_wq, &cm->cap_update_work, CM_CAP_CYCLE_TRACK_TIME * HZ);

#ifdef CONFIG_VENDOR_POWER_VOTER
	if (!IS_ERR(cm->policy.usb_phy)) {
		vote_debug("chg_state: %d\n", cm->policy.usb_phy->chg_state);
		if (cm->policy.usb_phy->chg_state == USB_CHARGER_PRESENT) {
			usb_phy_get_charger_current(cm->policy.usb_phy, &min, &max);
			cm->policy.limit = min;
			vote_debug("charger_current limit: %d\n", min);
			schedule_delayed_work(&cm->policy.usb_changed_work, msecs_to_jiffies(100));
		}
	}
#endif

	cm->is_battery_charging_enabled = false;
	zte_misc_register_callback(&cap_debug_node, cm);
	zte_misc_register_callback(&thermal_control_en_node, cm);
	zte_misc_register_callback(&zte_screen_on_node, cm);

	pr_info("%s probe success\n", __func__);
	return 0;

err_reg_sysfs:
	for (i = 0; i < desc->num_charger_regulators; i++) {
		struct charger_regulator *charger;

		charger = &desc->charger_regulators[i];
		sysfs_remove_group(&cm->charger_psy->dev.kobj,
				&charger->attr_g);
	}
err_reg_extcon:
	for (i = 0; i < desc->num_charger_regulators; i++)
		regulator_put(desc->charger_regulators[i].consumer);

	power_supply_unregister(cm->charger_psy);

	return ret;
}

static int charger_manager_remove(struct platform_device *pdev)
{
	struct charger_manager *cm = platform_get_drvdata(pdev);
	struct charger_desc *desc = cm->desc;
	int i = 0;

	/* Remove from the list */
	mutex_lock(&cm_list_mtx);
	list_del(&cm->entry);
	mutex_unlock(&cm_list_mtx);

	cancel_work_sync(&setup_polling);
	cancel_delayed_work_sync(&cm_monitor_work);
	cancel_delayed_work_sync(&cm->cap_update_work);
	cancel_delayed_work_sync(&cm->charge_debug_work);
	if (cm->track.cap_tracking)
		cancel_delayed_work_sync(&cm->track.track_capacity_work);

	for (i = 0 ; i < desc->num_charger_regulators ; i++)
		regulator_put(desc->charger_regulators[i].consumer);

	power_supply_unregister(cm->charger_psy);

	try_charger_enable(cm, false);

	return 0;
}

static void charger_manager_shutdown(struct platform_device *pdev)
{
	struct charger_manager *cm = platform_get_drvdata(pdev);

	set_batt_cap(cm, cm_capacity_unmap(cm, cm->desc->cap));
}

static const struct platform_device_id charger_manager_id[] = {
	{ "charger-manager", 0 },
	{ },
};
MODULE_DEVICE_TABLE(platform, charger_manager_id);

static int cm_suspend_noirq(struct device *dev)
{
	if (device_may_wakeup(dev)) {
		device_set_wakeup_capable(dev, false);
		return -EAGAIN;
	}

	return 0;
}

static int cm_suspend_prepare(struct device *dev)
{
	struct charger_manager *cm = dev_get_drvdata(dev);

	if (!cm_suspended)
		cm_suspended = true;

	/*
	 * In some situation, the system is not sleep between
	 * the charger polling interval - 15s, it maybe occur
	 * that charger manager will feed watchdog, but the
	 * system has no work to do to suspend, and charger
	 * manager also suspend. In this function, it will
	 * cancel cm_monito_work, it cause that this time can't
	 * feed watchdog until the next polling time, this means
	 * that charger manager feed watchdog per 15s usually,
	 * but this time need 30s, and the charger IC(fan54015)
	 * watchdog timeout to reset.
	 */
	if (is_ext_pwr_online(cm))
		cm_feed_watchdog(cm);

	if (cm && cm->is_battery_charging_enabled)
		cm_timer_set = false;
	else
		cm_timer_set = cm_setup_timer();

	if (cm_timer_set) {
		cancel_work_sync(&setup_polling);
		cancel_delayed_work_sync(&cm_monitor_work);
		cancel_delayed_work(&cm->fullbatt_vchk_work);
		cancel_delayed_work_sync(&cm->cap_update_work);
		cancel_delayed_work(&cm->charge_debug_work);
		if (cm->track.cap_tracking)
			cancel_delayed_work_sync(&cm->track.track_capacity_work);
	}

	return 0;
}

static void cm_suspend_complete(struct device *dev)
{
	struct charger_manager *cm = dev_get_drvdata(dev);

	if (cm_suspended)
		cm_suspended = false;

	if (cm_timer_set) {
		ktime_t remain;

		alarm_cancel(cm_timer);
		cm_timer_set = false;
		remain = alarm_expires_remaining(cm_timer);
		if (remain > 0)
			cm_suspend_duration_ms -= ktime_to_ms(remain);
		schedule_work(&setup_polling);
	}

	_cm_monitor(cm);
	cm_batt_works(&cm->cap_update_work.work);

	/* Re-enqueue delayed work (fullbatt_vchk_work) */
	if (cm->fullbatt_vchk_jiffies_at) {
		unsigned long delay = 0;
		unsigned long now = jiffies + CM_JIFFIES_SMALL;

		if (time_after_eq(now, cm->fullbatt_vchk_jiffies_at)) {
			delay = (unsigned long)((long)now
				- (long)(cm->fullbatt_vchk_jiffies_at));
			delay = jiffies_to_msecs(delay);
		} else {
			delay = 0;
		}

		/*
		 * Account for cm_suspend_duration_ms with assuming that
		 * timer stops in suspend.
		 */
		if (delay > cm_suspend_duration_ms)
			delay -= cm_suspend_duration_ms;
		else
			delay = 0;

		queue_delayed_work(cm_wq, &cm->fullbatt_vchk_work,
				   msecs_to_jiffies(delay));
	}
	device_set_wakeup_capable(cm->dev, false);
}

static const struct dev_pm_ops charger_manager_pm = {
	.prepare	= cm_suspend_prepare,
	.suspend_noirq	= cm_suspend_noirq,
	.complete	= cm_suspend_complete,
};

static struct platform_driver charger_manager_driver = {
	.driver = {
		.name = "charger-manager",
		.pm = &charger_manager_pm,
		.of_match_table = charger_manager_match,
	},
	.probe = charger_manager_probe,
	.remove = charger_manager_remove,
	.shutdown = charger_manager_shutdown,
	.id_table = charger_manager_id,
};

static int __init charger_manager_init(void)
{
	cm_wq = create_freezable_workqueue("charger_manager");
	INIT_DELAYED_WORK(&cm_monitor_work, cm_monitor_poller);

	return platform_driver_register(&charger_manager_driver);
}
late_initcall(charger_manager_init);

static void __exit charger_manager_cleanup(void)
{
	destroy_workqueue(cm_wq);
	cm_wq = NULL;

	platform_driver_unregister(&charger_manager_driver);
}
module_exit(charger_manager_cleanup);

/**
 * cm_notify_type_handle - charger driver handle charger event
 * @cm: the Charger Manager representing the battery
 * @type: type of charger event
 * @msg: optional message passed to uevent_notify function
 */
static void cm_notify_type_handle(struct charger_manager *cm, enum cm_event_types type, char *msg)
{
	switch (type) {
	case CM_EVENT_BATT_FULL:
		fullbatt_handler(cm);
		break;
	case CM_EVENT_BATT_IN:
	case CM_EVENT_BATT_OUT:
		battout_handler(cm);
		break;
	case CM_EVENT_EXT_PWR_IN_OUT ... CM_EVENT_CHG_START_STOP:
		pr_info("event CM_EVENT_CHG_START_STOP\n");
		misc_event_handler(cm, type);
		break;
	case CM_EVENT_UNKNOWN:
	case CM_EVENT_OTHERS:
		uevent_notify(cm, msg ? msg : default_event_names[type]);
		break;
	case CM_EVENT_FAST_CHARGE:
		fast_charge_handler(cm);
		break;
	default:
		dev_err(cm->dev, "%s: type not specified\n", __func__);
		break;
	}

	power_supply_changed(cm->charger_psy);
}

/**
 * cm_notify_event - charger driver notify Charger Manager of charger event
 * @psy: pointer to instance of charger's power_supply
 * @type: type of charger event
 * @msg: optional message passed to uevent_notify function
 */
void cm_notify_event(struct power_supply *psy, enum cm_event_types type,
		     char *msg)
{
	struct charger_manager *cm;
	bool found_power_supply = false;

	if (psy == NULL)
		return;

	mutex_lock(&cm_list_mtx);
	list_for_each_entry(cm, &cm_list, entry) {
		if (match_string(cm->desc->psy_charger_stat, -1,
				 psy->desc->name) >= 0 ||
		    match_string(cm->desc->psy_fast_charger_stat,
				 -1, psy->desc->name) >= 0 ||
		    match_string(&cm->desc->psy_fuel_gauge,
				 -1, psy->desc->name) >= 0) {
			found_power_supply = true;
			break;
		}
	}
	mutex_unlock(&cm_list_mtx);

	if (!found_power_supply) {
		cm_event_msg = msg;
		cm_event_type = type;
		return;
	}

	cm_notify_type_handle(cm, type, msg);
}
EXPORT_SYMBOL_GPL(cm_notify_event);

MODULE_AUTHOR("MyungJoo Ham <myungjoo.ham@samsung.com>");
MODULE_DESCRIPTION("Charger Manager");
MODULE_LICENSE("GPL");
