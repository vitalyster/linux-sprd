#ifndef __LINUX_SC2720_USB_CHARGER_INCLUDED
#define __LINUX_SC2720_USB_CHARGER_INCLUDED

#include <linux/delay.h>
#include <linux/regmap.h>
#include <uapi/linux/usb/charger.h>

#define SC2720_CHARGE_STATUS		0xe14
#define BIT_NON_DCP_INT			BIT(12)
#define BIT_CHG_DET_DONE		BIT(11)
#define BIT_SDP_INT			BIT(7)
#define BIT_DCP_INT			BIT(6)
#define BIT_CDP_INT			BIT(5)

static enum usb_charger_type sc27xx_charger_detect(struct regmap *regmap)
{
	enum usb_charger_type type;
	u32 status = 0, val;
	int ret, cnt = 10;

	do {
		ret = regmap_read(regmap, SC2720_CHARGE_STATUS, &val);
		if (ret)
			return UNKNOWN_TYPE;

		if (val & BIT_CHG_DET_DONE) {
			status = val & (BIT_CDP_INT | BIT_DCP_INT | BIT_SDP_INT);
			break;
		}

		msleep(200);
	} while (--cnt > 0);

	switch (status) {
	case BIT_CDP_INT:
		type = CDP_TYPE;
		break;
	case BIT_DCP_INT:
		type = DCP_TYPE;
		break;
	case BIT_SDP_INT:
		type = SDP_TYPE;
		break;
	default:
		type = UNKNOWN_TYPE;
	}

	if (type == UNKNOWN_TYPE && (val & BIT_CHG_DET_DONE)) {
		if ((val & BIT_NON_DCP_INT) == BIT_NON_DCP_INT)
			type = NON_DCP_TYPE;
	}

	pr_info("%s: cnt=%d, type=%d\n", __func__, cnt, type);
	return type;
}

#endif
