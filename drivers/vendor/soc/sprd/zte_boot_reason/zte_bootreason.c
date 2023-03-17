#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/errno.h>
#include <linux/debugfs.h>
#include <linux/device.h>

#define BOOT_REASON_MAX_LEN 20
static char boot_reason[BOOT_REASON_MAX_LEN] = "NONE";

static int __init zte_get_bootreason(char *str)
{
	snprintf(boot_reason, BOOT_REASON_MAX_LEN, "%s", str);
	return 1;
}

__setup("zte_bootreason=", zte_get_bootreason);

static ssize_t zte_power_reason_show(struct kobject *kobj, struct kobj_attribute *attr,
			     char *buf)
{
	int ret = 0;
	ret = snprintf(buf, BOOT_REASON_MAX_LEN, "%s\n", boot_reason);
	return ret;
}

static struct kobj_attribute zte_poweron_reason_attr =
__ATTR_RO(zte_power_reason);

static struct attribute *zte_power_reason_attributes[] = {
	&zte_poweron_reason_attr.attr,
	NULL
};

static const struct attribute_group zte_power_reason_attribute_group = {
        .attrs  = zte_power_reason_attributes,
};

static int __init zte_boot_reason_init(void)
{
	int err = 0;
	struct kobject *zte_power_reason_kobj;
	zte_power_reason_kobj = kobject_create_and_add("power_reason", NULL);
	if (!zte_power_reason_kobj) {
		pr_err("%s() - Unable to create zte_power_reason_kobj.\n",  __func__);
		return -ENOMEM;
	}

	err = sysfs_create_group(zte_power_reason_kobj, &zte_power_reason_attribute_group);
	if (err != 0) {
		pr_err("%s - zte_power_reason_attribute_group failed.\n", __func__);
		kobject_put(zte_power_reason_kobj);
		return err;
	}
	return err;
}

module_init(zte_boot_reason_init);
