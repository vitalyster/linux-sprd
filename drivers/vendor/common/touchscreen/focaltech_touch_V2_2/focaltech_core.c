/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2012-2018, FocalTech Systems, Ltd., all rights reserved.
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
/*****************************************************************************
*
* File Name: focaltech_core.c
*
* Author: Focaltech Driver Team
*
* Created: 2016-08-08
*
* Abstract: entrance for focaltech ts driver
*
* Version: V1.0
*
*****************************************************************************/

/*****************************************************************************
* Included header files
*****************************************************************************/
#include "focaltech_core.h"
#ifdef CONFIG_ADF
#include <video/adf_notifier.h>
#elif defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#define FTS_SUSPEND_LEVEL 1	/* Early-suspend level */
#endif
#ifdef CONFIG_TS_FTS_MTK_INTERFACE
#include "focaltech_mtk_interface.h"
#endif

#include "tpd_sys.h"
/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define FTS_DRIVER_NAME                     "fts_ts"
#define INTERVAL_READ_REG                   100	/* unit:ms */
#define TIMEOUT_READ_REG                    1000	/* unit:ms */
#define TIMEOUT_RESUME_READ_REG                    1500	/* unit:ms */

#define FTS_VTG_MIN_UV                      2600000
#define FTS_VTG_MAX_UV                      3300000
#define FTS_I2C_VTG_MIN_UV                  1800000
#define FTS_I2C_VTG_MAX_UV                  1800000

#ifdef CONFIG_TS_FTS_MTK_INTERFACE
#define FTS_PINCTRL_IRQ_STATE "eint_as_int"
#define FTS_PINCTRL_RST_0_STATE "rst_output0"
#define FTS_PINCTRL_RST_1_STATE "rst_output1"
#endif


/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
struct fts_ts_data *fts_data;
/*****************************************************************************
* Static function prototypes
*****************************************************************************/
static void fts_release_all_finger(void);
static int fts_ts_suspend(struct device *dev);
static int fts_ts_resume(struct device *dev);

/****************************************************************************
*				zhangjian add for common interface
*****************************************************************************/
extern int tpd_get_gesture_mode(void);
extern int tpd_set_gesture_mode(int gesture_mode);

static int tpd_init_tpinfo(struct tpd_classdev_t *cdev)
{
	u8 fwver_in_chip = 0;
	u8 vendorid_in_chip = 0;
	u8 chipid_in_chip = 0;
	u8 retry = 0;

	while (retry++ < 5) {
		fts_i2c_read_reg(fts_data->client, FTS_REG_CHIP_ID, &chipid_in_chip);
		fts_i2c_read_reg(fts_data->client, FTS_REG_VENDOR_ID, &vendorid_in_chip);
		fts_i2c_read_reg(fts_data->client, FTS_REG_FW_VER, &fwver_in_chip);
		if ((chipid_in_chip != 0) && (vendorid_in_chip != 0) && (fwver_in_chip != 0)) {
			FTS_DEBUG("chip_id = %x,vendor_id =%x,fw_version=%x .",
				  chipid_in_chip, vendorid_in_chip, fwver_in_chip);
			break;
		}
		FTS_DEBUG("chip_id = %x,vendor_id =%x,fw_version=%x .",
			  chipid_in_chip, vendorid_in_chip, fwver_in_chip);
		msleep(20);
	}

	strlcpy(cdev->ic_tpinfo.tp_name, "Focal", 10);
	cdev->ic_tpinfo.chip_model_id = TS_CHIP_FOCAL;

	cdev->ic_tpinfo.chip_part_id = chipid_in_chip;
	cdev->ic_tpinfo.module_id = vendorid_in_chip;
	cdev->ic_tpinfo.chip_ver = 0;
	cdev->ic_tpinfo.firmware_ver = fwver_in_chip;
	cdev->ic_tpinfo.i2c_type = 0;
	cdev->ic_tpinfo.i2c_addr = 0x38;

	return 0;
}

#if FTS_GESTURE_EN
static int tpd_get_wakegesture(struct tpd_classdev_t *cdev)
{
	cdev->b_gesture_enable = tpd_get_gesture_mode();
	return 0;
}

static int tpd_enable_wakegesture(struct tpd_classdev_t *cdev, int enable)
{
	tpd_set_gesture_mode(enable);
	return enable;
}
#endif

static int fts_i2c_reg_read(struct tpd_classdev_t *cdev, char addr, u8 *data, int len)
{
	return fts_i2c_read(fts_data->client, &addr, 1, data, len);
}
static int fts_i2c_reg_write(struct tpd_classdev_t *cdev, char addr, u8 *data, int len)
{
	int i;

	if (len == 1) {
		goto WRITE_ONE_REG;
	} else {
		char *tmpbuf;

		tmpbuf = kzalloc(len + 1, GFP_KERNEL);
		if (!tmpbuf) {
			pr_info("allocate memory failed!\n");
			return -ENOMEM;
		}
		tmpbuf[0] = addr & 0xFF;
		for (i = 1; i <= len; i++) {
			tmpbuf[i] = data[i - 1];
		}
		fts_i2c_write(fts_data->client, tmpbuf, len);
		kfree(tmpbuf);
		return 0;
	}
WRITE_ONE_REG:
	return fts_i2c_write_reg(fts_data->client, addr, *data);
}
static int fts_tp_fw_upgrade(struct tpd_classdev_t *cdev, char *fw_name, int fwname_len)
{
	char fwname[FILE_NAME_LENGTH];
	struct fts_ts_data *ts_data = fts_data;
	struct input_dev *input_dev = ts_data->input_dev;
	struct i2c_client *client = ts_data->client;

	if ((fwname_len <= 1) || (fwname_len >= FILE_NAME_LENGTH)) {
		FTS_ERROR("fw bin name's length(%d) fail", fwname_len);
		return -EINVAL;
	}
	memset(fwname, 0, sizeof(fwname));
	snprintf(fwname, sizeof(fwname), "%s", fw_name);
	fwname[fwname_len - 1] = '\0';
	FTS_INFO("fwname is %s", fwname);
	mutex_lock(&input_dev->mutex);
	ts_data->fw_loading = 1;
	fts_irq_disable();
#if FTS_ESDCHECK_EN
	fts_esdcheck_switch(DISABLE);
#endif

	fts_upgrade_bin(client, fwname, 0);

#if FTS_ESDCHECK_EN
	fts_esdcheck_switch(ENABLE);
#endif
	fts_irq_enable();
	ts_data->fw_loading = 0;
	mutex_unlock(&input_dev->mutex);

	return 0;
}

static int fts_tp_suspend_show(struct tpd_classdev_t *cdev)
{
	cdev->tp_suspend = fts_data->suspended;
	return cdev->tp_suspend;
}

static int fts_set_tp_suspend(struct tpd_classdev_t *cdev, u8 suspend_node, int enable)
{
	int ret = 0;

	if (enable) {
		if (!fts_data->fb_tpd_suspend_flag) {
			FTS_DEBUG("lcd off notifier.");
			ret = cancel_work_sync(&fts_data->fb_tpd_resume_work);
			if (!ret)
				FTS_DEBUG("cancel fb_tpd_resume_wq err = %d", ret);
			fts_ts_suspend(&fts_data->client->dev);
			fts_data->fb_tpd_suspend_flag = 1;
		}
	} else {
		if (fts_data->fb_tpd_suspend_flag) {
			if (suspend_node == PROC_SUSPEND_NODE) {
				fts_reset_proc(200);
			}
			ret = queue_work(fts_data->fb_tpd_resume_wq, &fts_data->fb_tpd_resume_work);
			if (!ret) {
				FTS_ERROR("start tpd_resume_wq failed");
				return ret;
			}
		}
	}
	cdev->tp_suspend = fts_data->suspended;
	return cdev->tp_suspend;
}

static int tpd_register_fw_class(struct fts_ts_data *data)
{
	tpd_fw_cdev.private = (void *)data;
	tpd_fw_cdev.get_tpinfo = tpd_init_tpinfo;
	#if FTS_GESTURE_EN
	tpd_fw_cdev.get_gesture = tpd_get_wakegesture;
	tpd_fw_cdev.wake_gesture = tpd_enable_wakegesture;
	#endif
	tpd_fw_cdev.tp_i2c_reg_read = fts_i2c_reg_read;
	tpd_fw_cdev.tp_i2c_reg_write = fts_i2c_reg_write;
	tpd_fw_cdev.tp_fw_upgrade = fts_tp_fw_upgrade;
	tpd_fw_cdev.tp_suspend_show = fts_tp_suspend_show;
	tpd_fw_cdev.set_tp_suspend = fts_set_tp_suspend;
	return 0;
}

/**************** add for common interface end************************************/

/*****************************************************************************
*  Name: fts_wait_tp_to_valid
*  Brief: Read chip id until TP FW become valid(Timeout: TIMEOUT_READ_REG),
*         need call when reset/power on/resume...
*  Input:
*  Output:
*  Return: return 0 if tp valid, otherwise return error code
*****************************************************************************/
int fts_wait_tp_to_valid(struct i2c_client *client)
{
	int ret = 0;
	int cnt = 0;
	u8 reg_value = 0;
	u8 chip_id = fts_data->ic_info.ids.chip_idh;

	do {
		ret = fts_i2c_read_reg(client, FTS_REG_CHIP_ID, &reg_value);
		if ((ret < 0) || (reg_value != chip_id)) {
			FTS_DEBUG("TP Not Ready, ReadData = 0x%x", reg_value);
		} else if (reg_value == chip_id) {
			FTS_INFO("TP Ready, Device ID = 0x%x", reg_value);
			return 0;
		}
		cnt++;
		msleep(INTERVAL_READ_REG);
	} while ((cnt * INTERVAL_READ_REG) < TIMEOUT_RESUME_READ_REG);

	return -EIO;
}

/************************************************************************
* Name: fts_get_chip_types
* Brief: verity chip id and get chip type data
* Input:
* Output:
* Return: return 0 if success, otherwise return error code
***********************************************************************/
static int fts_get_chip_types(struct fts_ts_data *ts_data, u8 id_h, u8 id_l, bool fw_valid)
{
	int i = 0;
	struct ft_chip_t ctype[] = FTS_CHIP_TYPE_MAPPING;
	u32 ctype_entries = sizeof(ctype) / sizeof(struct ft_chip_t);

	if ((id_h == 0x0) || (id_l == 0x0)) {
		FTS_ERROR("id_h/id_l is 0");
		return -EINVAL;
	}

	FTS_DEBUG("verify id:0x%02x%02x", id_h, id_l);
	for (i = 0; i < ctype_entries; i++) {
		if (fw_valid == VALID) {
			if ((id_h == ctype[i].chip_idh) && (id_l == ctype[i].chip_idl))
				break;
		} else {
			if (((id_h == ctype[i].rom_idh) && (id_l == ctype[i].rom_idl))
			    || ((id_h == ctype[i].pb_idh) && (id_l == ctype[i].pb_idl))
			    || ((id_h == ctype[i].bl_idh) && (id_l == ctype[i].bl_idl)))
				break;
		}
	}

	if (i >= ctype_entries) {
		return -ENODATA;
	}

	ts_data->ic_info.ids = ctype[i];
	return 0;
}

/*****************************************************************************
*  Name: fts_get_ic_information
*  Brief:
*  Input:
*  Output:
*  Return: return 0 if success, otherwise return error code
*****************************************************************************/
static int fts_get_ic_information(struct fts_ts_data *ts_data)
{
	int ret = 0;
	int cnt = 0;
	u8 chip_id[2] = { 0 };
	u8 id_cmd[4] = { 0 };
	u32 id_cmd_len = 0;
	struct i2c_client *client = ts_data->client;

	ts_data->ic_info.is_incell = FTS_CHIP_IDC;
	ts_data->ic_info.hid_supported = FTS_HID_SUPPORTTED;
	do {
		ret = fts_i2c_read_reg(client, FTS_REG_CHIP_ID, &chip_id[0]);
		ret = fts_i2c_read_reg(client, FTS_REG_CHIP_ID2, &chip_id[1]);
		if ((ret < 0) || (chip_id[0] == 0x0) || (chip_id[1] == 0x0)) {
			FTS_DEBUG("i2c read invalid, read:0x%02x%02x", chip_id[0], chip_id[1]);
		} else {
			ret = fts_get_chip_types(ts_data, chip_id[0], chip_id[1], VALID);
			if (ret != 0)
				FTS_DEBUG("TP not ready, read:0x%02x%02x", chip_id[0], chip_id[1]);
			else
				break;
		}

		cnt++;
		msleep(INTERVAL_READ_REG);
	} while ((cnt * INTERVAL_READ_REG) < TIMEOUT_READ_REG);

	if ((cnt * INTERVAL_READ_REG) >= TIMEOUT_READ_REG) {
		FTS_INFO("fw is invalid, need read boot id");
		if (ts_data->ic_info.hid_supported) {
			fts_i2c_hid2std(client);
		}

		id_cmd[0] = FTS_CMD_START1;
		id_cmd[1] = FTS_CMD_START2;
		ret = fts_i2c_write(client, id_cmd, 2);
		if (ret < 0) {
			FTS_ERROR("start cmd write fail");
			return ret;
		}

		msleep(FTS_CMD_START_DELAY);
		id_cmd[0] = FTS_CMD_READ_ID;
		id_cmd[1] = id_cmd[2] = id_cmd[3] = 0x00;
		if (ts_data->ic_info.is_incell)
			id_cmd_len = FTS_CMD_READ_ID_LEN_INCELL;
		else
			id_cmd_len = FTS_CMD_READ_ID_LEN;
		ret = fts_i2c_read(client, id_cmd, id_cmd_len, chip_id, 2);
		if ((ret < 0) || (chip_id[0] == 0x0) || (chip_id[1] == 0x0)) {
			FTS_ERROR("read boot id fail");
			return -EIO;
		}
		ret = fts_get_chip_types(ts_data, chip_id[0], chip_id[1], INVALID);
		if (ret < 0) {
			FTS_ERROR("can't get ic informaton");
			return ret;
		}
	}

	FTS_INFO("get ic information, chip id = 0x%02x%02x",
		 ts_data->ic_info.ids.chip_idh, ts_data->ic_info.ids.chip_idl);

	return 0;
}

/*****************************************************************************
*  Name: fts_tp_state_recovery
*  Brief: Need execute this function when reset
*  Input:
*  Output:
*  Return:
*****************************************************************************/
void fts_tp_state_recovery(struct i2c_client *client)
{
	FTS_FUNC_ENTER();
	/* wait tp stable */
	fts_wait_tp_to_valid(client);
	/* recover TP charger state 0x8B */
	/* recover TP glove state 0xC0 */
	/* recover TP cover state 0xC1 */
	fts_ex_mode_recovery(client);
	/* recover TP gesture state 0xD0 */
#if FTS_GESTURE_EN
	fts_gesture_recovery(client);
#endif
	FTS_FUNC_EXIT();
}

/*****************************************************************************
*  Name: fts_reset_proc
*  Brief: Execute reset operation
*  Input: hdelayms - delay time unit:ms
*  Output:
*  Return:
*****************************************************************************/
int fts_reset_proc(int hdelayms)
{
	FTS_FUNC_ENTER();
#ifdef CONFIG_TS_FTS_MTK_INTERFACE
	fts_rst_output(0);
#else
	gpio_direction_output(fts_data->pdata->reset_gpio, 0);
#endif

	msleep(20);

#ifdef CONFIG_TS_FTS_MTK_INTERFACE
	fts_rst_output(1);
#else
	gpio_direction_output(fts_data->pdata->reset_gpio, 1);
#endif
	if (hdelayms) {
		msleep(hdelayms);
	}

	FTS_FUNC_EXIT();
	return 0;
}

/*****************************************************************************
*  Name: fts_irq_disable
*  Brief: disable irq
*  Input:
*  Output:
*  Return:
*****************************************************************************/
void fts_irq_disable(void)
{
	unsigned long irqflags;

	FTS_FUNC_ENTER();
	spin_lock_irqsave(&fts_data->irq_lock, irqflags);

	if (!fts_data->irq_disabled) {
		disable_irq_nosync(fts_data->irq);
		fts_data->irq_disabled = true;
	}

	spin_unlock_irqrestore(&fts_data->irq_lock, irqflags);
	FTS_FUNC_EXIT();
}

/*****************************************************************************
*  Name: fts_irq_enable
*  Brief: enable irq
*  Input:
*  Output:
*  Return:
*****************************************************************************/
void fts_irq_enable(void)
{
	unsigned long irqflags = 0;

	FTS_FUNC_ENTER();
	spin_lock_irqsave(&fts_data->irq_lock, irqflags);

	if (fts_data->irq_disabled) {
		enable_irq(fts_data->irq);
		fts_data->irq_disabled = false;
	}

	spin_unlock_irqrestore(&fts_data->irq_lock, irqflags);
	FTS_FUNC_EXIT();
}

/*****************************************************************************
* Power Control
*****************************************************************************/
static int fts_gpio_ldo_power_init(struct fts_ts_data *data)
{
	int rc;

	FTS_FUNC_ENTER();
	data->ldo_avdd = regulator_get(&data->client->dev, "ldo-avdd");
	if (IS_ERR(data->ldo_avdd)) {
		rc = PTR_ERR(data->ldo_avdd);
		FTS_ERROR("Regulator get failed ldo_avdd rc=%d", rc);
	}

	if (regulator_count_voltages(data->ldo_avdd) > 0) {
		rc = regulator_set_voltage(data->vdd, FTS_I2C_VTG_MIN_UV, FTS_I2C_VTG_MIN_UV);
		if (rc) {
			FTS_ERROR("Regulator set_vtg failed vdd rc=%d", rc);
			goto reg_avdd_put;
		}
	}
	data->ldo_iovdd = regulator_get(&data->client->dev, "ldo-iovdd");
	if (IS_ERR(data->ldo_iovdd)) {
		rc = PTR_ERR(data->ldo_iovdd);
		FTS_ERROR("Regulator get failed ldo_avdd rc=%d", rc);
		goto reg_avdd_set_vtg;
	}

	if (regulator_count_voltages(data->ldo_iovdd) > 0) {
		rc = regulator_set_voltage(data->ldo_iovdd, FTS_I2C_VTG_MIN_UV, FTS_I2C_VTG_MAX_UV);
		if (rc) {
			FTS_ERROR("Regulator set_vtg failed vdd rc=%d", rc);
			goto reg_iovdd_put;
		}
	}
	FTS_FUNC_EXIT();
	return 0;

reg_iovdd_put:
	regulator_put(data->ldo_iovdd);
reg_avdd_set_vtg:
	if (regulator_count_voltages(data->ldo_avdd) > 0)
		regulator_set_voltage(data->ldo_avdd, 0, FTS_I2C_VTG_MAX_UV);
reg_avdd_put:
	regulator_put(data->ldo_avdd);
	FTS_FUNC_EXIT();
	return rc;
}

static int fts_power_source_init(struct fts_ts_data *data)
{
	int ret = 0;

	FTS_FUNC_ENTER();
	fts_gpio_ldo_power_init(data);
	#ifdef FTS_VDD2V9_EN
	data->vdd = regulator_get(&data->client->dev, "vdd28");
	#else
	data->vdd = regulator_get(&data->client->dev, "vdd");
	#endif
	if (IS_ERR(data->vdd)) {
		ret = PTR_ERR(data->vdd);
		FTS_ERROR("get vdd regulator failed,ret=%d", ret);
	} else {
		if (regulator_count_voltages(data->vdd) > 0) {
			#ifdef FTS_VDD2V9_EN
			FTS_DEBUG("%s regulator vdd 2900000!", __func__);
			ret = regulator_set_voltage(data->vdd, 2900000, 2900000);
			#else
			ret = regulator_set_voltage(data->vdd, FTS_VTG_MIN_UV, FTS_VTG_MAX_UV);
			#endif
			if (ret) {
				FTS_ERROR("vdd regulator set_vtg failed ret=%d", ret);
				goto err_set_vtg_vdd;
			}
		}
	}

	data->vcc_i2c = regulator_get(&data->client->dev, "vcc_i2c");
	if (IS_ERR(data->vcc_i2c)) {
		ret = PTR_ERR(data->vcc_i2c);
		FTS_ERROR("ret vcc_i2c regulator failed,ret=%d", ret);
	} else {
		if (regulator_count_voltages(data->vcc_i2c) > 0) {
			ret = regulator_set_voltage(data->vcc_i2c, FTS_I2C_VTG_MIN_UV, FTS_I2C_VTG_MAX_UV);
			if (ret) {
				FTS_ERROR("vcc_i2c regulator set_vtg failed ret=%d", ret);
				goto err_set_vtg_vcc;
			}
		}
	}
	FTS_FUNC_EXIT();
	return 0;

err_set_vtg_vcc:
	if (!IS_ERR(data->vcc_i2c))
		regulator_put(data->vcc_i2c);

	if (!IS_ERR(data->vdd)) {
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0, FTS_VTG_MAX_UV);
	}
err_set_vtg_vdd:
	if (!IS_ERR(data->vdd))
		regulator_put(data->vdd);

	FTS_FUNC_EXIT();
	return ret;
}

#if FTS_POWER_SOURCE_CUST_EN
static int fts_power_source_release(struct fts_ts_data *data)
{
	if (data == NULL)
		return 0;

	if (!IS_ERR(data->vdd))
		regulator_put(data->vdd);

	if (!IS_ERR(data->vcc_i2c))
		regulator_put(data->vcc_i2c);

	if (!IS_ERR(data->ldo_avdd))
		regulator_put(data->ldo_avdd);

	if (!IS_ERR(data->ldo_iovdd))
		regulator_put(data->ldo_iovdd);

	return 0;
}
#endif

static int fts_power_source_ctrl(struct fts_ts_data *data, int enable)
{
	int ret = 0;

	FTS_FUNC_ENTER();
	if (enable) {
		if (data->power_disabled) {
			FTS_DEBUG("regulator enable !");
			if (!IS_ERR(data->vdd)) {
				ret = regulator_enable(data->vdd);
				if (ret) {
					FTS_ERROR("enable vdd regulator failed,ret=%d", ret);
				}
			}

			if (!IS_ERR(data->vcc_i2c)) {
				ret = regulator_enable(data->vcc_i2c);
				if (ret) {
					FTS_ERROR("enable vcc_i2c regulator failed,ret=%d", ret);
				}
			}

			if (!IS_ERR(data->ldo_avdd)) {
				ret = regulator_enable(data->ldo_avdd);
				if (ret) {
					FTS_ERROR("Regulator ldo_avdd enable failed rc=%d", ret);
				}
			}

			if (!IS_ERR(data->ldo_iovdd)) {
				ret = regulator_enable(data->ldo_iovdd);
				if (ret) {
					FTS_ERROR("Regulator ldo_iovdd enable failed rc=%d", ret);
				}
			}
			data->power_disabled = false;
		}
	} else {
		if (!data->power_disabled) {
			FTS_DEBUG("regulator disable !");
			if (!IS_ERR(data->vdd)) {
				ret = regulator_disable(data->vdd);
				if (ret) {
					FTS_ERROR("disable vdd regulator failed,ret=%d", ret);
				}
			}

			if (!IS_ERR(data->vcc_i2c)) {
				ret = regulator_disable(data->vcc_i2c);
				if (ret) {
					FTS_ERROR("disable vcc_i2c regulator failed,ret=%d", ret);
				}
			}

			if (!IS_ERR(data->ldo_avdd)) {
				ret = regulator_disable(data->ldo_avdd);
				if (ret) {
					FTS_ERROR("Regulator ldo_avdd disable failed rc=%d", ret);
				}
			}

			if (!IS_ERR(data->ldo_iovdd)) {
				ret = regulator_disable(data->ldo_iovdd);
				if (ret) {
					FTS_ERROR("Regulator ldo_iovdd disable failed rc=%d", ret);
				}
			}
			data->power_disabled = true;
		}
	}

	FTS_FUNC_EXIT();
	return ret;
}

#if FTS_PINCTRL_EN
/*****************************************************************************
*  Name: fts_pinctrl_init
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static int fts_pinctrl_init(struct fts_ts_data *ts)
{
	int ret = 0;
	struct i2c_client *client = ts->client;

	ts->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(ts->pinctrl)) {
		FTS_ERROR("Failed to get pinctrl, please check dts");
		ret = PTR_ERR(ts->pinctrl);
		goto err_pinctrl_get;
	}

	ts->pins_active = pinctrl_lookup_state(ts->pinctrl, "pmx_ts_active");
	if (IS_ERR_OR_NULL(ts->pins_active)) {
		FTS_ERROR("Pin state[active] not found");
		ret = PTR_ERR(ts->pins_active);
		goto err_pinctrl_lookup;
	}

	ts->pins_suspend = pinctrl_lookup_state(ts->pinctrl, "pmx_ts_suspend");
	if (IS_ERR_OR_NULL(ts->pins_suspend)) {
		FTS_ERROR("Pin state[suspend] not found");
		ret = PTR_ERR(ts->pins_suspend);
		goto err_pinctrl_lookup;
	}

#ifdef CONFIG_TS_FTS_MTK_INTERFACE
	ts->pinctrl_int_input
		= pinctrl_lookup_state(ts->pinctrl, FTS_PINCTRL_IRQ_STATE);
	if (IS_ERR_OR_NULL(ts->pinctrl_int_input)) {
		ret = PTR_ERR(ts->pinctrl_int_input);
		FTS_ERROR("Can not lookup %s pinstate %d", FTS_PINCTRL_IRQ_STATE, ret);
		goto err_pinctrl_lookup;
	}

	ts->pinctrl_rst_output0
		= pinctrl_lookup_state(ts->pinctrl, FTS_PINCTRL_RST_0_STATE);
	if (IS_ERR_OR_NULL(ts->pinctrl_rst_output0)) {
		ret = PTR_ERR(ts->pinctrl_rst_output0);
		FTS_ERROR("Can not lookup %s pinstate %d", FTS_PINCTRL_RST_0_STATE, ret);
		goto err_pinctrl_lookup;
	}

	ts->pinctrl_rst_output1
		= pinctrl_lookup_state(ts->pinctrl, FTS_PINCTRL_RST_1_STATE);
	if (IS_ERR_OR_NULL(ts->pinctrl_rst_output1)) {
		ret = PTR_ERR(ts->pinctrl_rst_output1);
		FTS_ERROR("Can not lookup %s pinstate %d", FTS_PINCTRL_RST_1_STATE, ret);
		goto err_pinctrl_lookup;
	}

	fts_pdata = ts;
#endif

	ts->pins_release = pinctrl_lookup_state(ts->pinctrl, "pmx_ts_release");
	if (IS_ERR_OR_NULL(ts->pins_release)) {
		FTS_ERROR("Pin state[release] not found");
		ret = PTR_ERR(ts->pins_release);
	}

	return 0;
err_pinctrl_lookup:
	devm_pinctrl_put(ts->pinctrl);
err_pinctrl_get:
	ts->pinctrl = NULL;
	ts->pins_release = NULL;
	ts->pins_suspend = NULL;
	ts->pins_active = NULL;
#ifdef CONFIG_TS_FTS_MTK_INTERFACE
	ts->pinctrl_int_input = NULL;
	ts->pinctrl_rst_output0 = NULL;
	ts->pinctrl_rst_output1 = NULL;
#endif

	return ret;
}

static int fts_pinctrl_select_normal(struct fts_ts_data *ts)
{
	int ret = 0;

	if (ts->pinctrl && ts->pins_active) {
		ret = pinctrl_select_state(ts->pinctrl, ts->pins_active);
		if (ret < 0) {
			FTS_ERROR("Set normal pin state error:%d", ret);
		}
	}

	return ret;
}

static int fts_pinctrl_select_suspend(struct fts_ts_data *ts)
{
	int ret = 0;

	if (ts->pinctrl && ts->pins_suspend) {
		ret = pinctrl_select_state(ts->pinctrl, ts->pins_suspend);
		if (ret < 0) {
			FTS_ERROR("Set suspend pin state error:%d", ret);
		}
	}

	return ret;
}

static int fts_pinctrl_select_release(struct fts_ts_data *ts)
{
	int ret = 0;

	if (ts->pinctrl) {
		if (IS_ERR_OR_NULL(ts->pins_release)) {
			devm_pinctrl_put(ts->pinctrl);
			ts->pinctrl = NULL;
		} else {
			ret = pinctrl_select_state(ts->pinctrl, ts->pins_release);
			if (ret < 0)
				FTS_ERROR("Set gesture pin state error:%d", ret);
		}
	}

	return ret;
}
#endif /* FTS_PINCTRL_EN */

/*****************************************************************************
*  Reprot related
*****************************************************************************/
#if (FTS_DEBUG_EN && (FTS_DEBUG_LEVEL == 2))
char g_sz_debug[1024] = { 0 };

static void fts_show_touch_buffer(u8 *buf, int point_num)
{
	int len = point_num * FTS_ONE_TCH_LEN;
	int count = 0;
	int i;

	memset(g_sz_debug, 0, 1024);
	if (len > (fts_data->pnt_buf_size - 3)) {
		len = fts_data->pnt_buf_size - 3;
	} else if (len == 0) {
		len += FTS_ONE_TCH_LEN;
	}
	count += snprintf(g_sz_debug, PAGE_SIZE, "%02X,%02X,%02X", buf[0], buf[1], buf[2]);
	for (i = 0; i < len; i++) {
		count += snprintf(g_sz_debug + count, PAGE_SIZE, ",%02X", buf[i + 3]);
	}
	FTS_DEBUG("buffer: %s", g_sz_debug);
}
#endif

/*****************************************************************************
 *  Name: fts_release_all_finger
 *  Brief: report all points' up events, release touch
 *  Input:
 *  Output:
 *  Return:
 *****************************************************************************/
static void fts_release_all_finger(void)
{
	struct input_dev *input_dev = fts_data->input_dev;
#if FTS_MT_PROTOCOL_B_EN
	u32 finger_count = 0;
#endif

	FTS_FUNC_ENTER();
	mutex_lock(&fts_data->report_mutex);
#if FTS_MT_PROTOCOL_B_EN
	for (finger_count = 0; finger_count < fts_data->pdata->max_touch_number; finger_count++) {
		input_mt_slot(input_dev, finger_count);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
	}
#else
	input_mt_sync(input_dev);
#endif
	input_report_key(input_dev, BTN_TOUCH, 0);
	input_sync(input_dev);

	mutex_unlock(&fts_data->report_mutex);
	FTS_FUNC_EXIT();
}

/************************************************************************
 * Name: fts_input_report_key
 * Brief: report key event
 * Input: events info
 * Output:
 * Return: return 0 if success
 ***********************************************************************/
static int fts_input_report_key(struct fts_ts_data *data, int index)
{
	u32 ik;
	int id = data->events[index].id;
	int x = data->events[index].x;
	int y = data->events[index].y;
	int flag = data->events[index].flag;
	u32 key_num = data->pdata->key_number;

	if (!KEY_EN(data)) {
		return -EINVAL;
	}

	#ifdef FTS_COMBINATION_KEY
		if (TOUCH_IN_KEY(x, data->pdata->comb_key_x_coord[0])) {
			if (EVENT_DOWN(flag)) {
				data->key_down = true;
				input_report_key(data->input_dev, data->pdata->comb_key_x_coord[1], 1);
				input_report_key(data->input_dev, data->pdata->comb_key_x_coord[2], 1);
				FTS_DEBUG("comb-Key(%d, %d) DOWN!", x, y);
			} else {
				data->key_down = false;
				input_report_key(data->input_dev, data->pdata->comb_key_x_coord[1], 0);
				input_report_key(data->input_dev, data->pdata->comb_key_x_coord[2], 0);
				FTS_DEBUG("comb-Key(%d, %d) UP!", x, y);
			}

			return 0;
		}
	#endif

	for (ik = 0; ik < key_num; ik++) {
		if (TOUCH_IN_KEY(x, data->pdata->key_x_coords[ik])) {
			if (EVENT_DOWN(flag)) {
				data->key_down = true;
				input_report_key(data->input_dev, data->pdata->keys[ik], 1);
				FTS_DEBUG("Key%d(%d, %d) DOWN!", ik, x, y);
			} else {
				data->key_down = false;
				input_report_key(data->input_dev, data->pdata->keys[ik], 0);
				FTS_DEBUG("Key%d(%d, %d) Up!", ik, x, y);
			}
			return 0;
		}
	}

	FTS_ERROR("invalid touch for key, [%d](%d, %d)", id, x, y);
	return -EINVAL;
}

#if FTS_MT_PROTOCOL_B_EN
static int fts_input_report_b(struct fts_ts_data *data)
{
	int i = 0;
	int uppoint = 0;
	int touchs = 0;
	bool va_reported = false;
	u32 max_touch_num = data->pdata->max_touch_number;
	u32 key_y_coor = data->pdata->key_y_coord;
	struct ts_event *events = data->events;
	/* print tp point */
	static char finger_down[FTS_MAX_POINTS_SUPPORT] = {0};
	int log_x = 0;
	int log_y = 0;

	for (i = 0; i < data->touch_point; i++) {
		if (KEY_EN(data) && TOUCH_IS_KEY(events[i].y, key_y_coor)) {
			fts_input_report_key(data, i);
			continue;
		}
		if ((events[i].x < data->pdata->limit_x_left) || (events[i].x > data->pdata->limit_x_right) ||
		    (events[i].y < data->pdata->limit_y_top)) {
			if (EVENT_DOWN(events[i].flag)) {
				touchs |= BIT(events[i].id);
				data->touchs |= BIT(events[i].id);
				FTS_INFO("touch id: %d, coord [%d:%d] beyond the limit.",
					 events[i].id, events[i].x, events[i].y);
				continue;
			}
		}

		if (events[i].id >= max_touch_num)
			continue;

		va_reported = true;
		input_mt_slot(data->input_dev, events[i].id);

		if (EVENT_DOWN(events[i].flag)) {
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, true);

#if FTS_REPORT_PRESSURE_EN
			if (events[i].p <= 0) {
				events[i].p = 0x3f;
			}
			input_report_abs(data->input_dev, ABS_MT_PRESSURE, events[i].p);
#endif
			if (events[i].area <= 0) {
				events[i].area = 0x09;
			}
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, events[i].area);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, events[i].x);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, events[i].y);

			touchs |= BIT(events[i].id);
			data->touchs |= BIT(events[i].id);
			/* print tp point */
			if (!finger_down[events[i].id]) {
				log_x = events[i].x;
				log_y = events[i].y;
				finger_down[events[i].id] = 1;
				FTS_INFO("touch down id: %d, coord [%d:%d]", events[i].id, log_x, log_y);
			}

		} else {
			uppoint++;
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
			data->touchs &= ~BIT(events[i].id);
			/* print tp point */
			if (finger_down[events[i].id] == 1) {
				log_x = events[i].x;
				log_y = events[i].y;
				finger_down[events[i].id] = 0;
				FTS_INFO("touch up id: %d, coord [%d:%d]", events[i].id, log_x, log_y);
			}
		}
	}

	if (unlikely(data->touchs ^ touchs)) {
		for (i = 0; i < max_touch_num; i++) {
			if (BIT(i) & (data->touchs ^ touchs)) {
				va_reported = true;
				input_mt_slot(data->input_dev, i);
				input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
			}
		}
	}
	data->touchs = touchs;

	if (va_reported) {
		/* touchs==0, there's no point but key */
		if (EVENT_NO_DOWN(data) || (!touchs)) {
			input_report_key(data->input_dev, BTN_TOUCH, 0);
			/* print tp point */
			FTS_INFO("all touch up");
		} else {
			input_report_key(data->input_dev, BTN_TOUCH, 1);
		}
	}

	input_sync(data->input_dev);
	return 0;
}

#else
static int fts_input_report_a(struct fts_ts_data *data)
{
	int i = 0;
	int touchs = 0;
	bool va_reported = false;
	u32 key_y_coor = data->pdata->key_y_coord;
	struct ts_event *events = data->events;

	for (i = 0; i < data->touch_point; i++) {
		if (KEY_EN(data) && TOUCH_IS_KEY(events[i].y, key_y_coor)) {
			fts_input_report_key(data, i);
			continue;
		}

		va_reported = true;
		if (EVENT_DOWN(events[i].flag)) {
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, events[i].id);
#if FTS_REPORT_PRESSURE_EN
			if (events[i].p <= 0) {
				events[i].p = 0x3f;
			}
			input_report_abs(data->input_dev, ABS_MT_PRESSURE, events[i].p);
#endif
			if (events[i].area <= 0) {
				events[i].area = 0x09;
			}
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, events[i].area);

			input_report_abs(data->input_dev, ABS_MT_POSITION_X, events[i].x);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, events[i].y);

			input_mt_sync(data->input_dev);
			touchs++;
		}
	}

	/* last point down, current no point but key */
	if (data->touchs && !touchs) {
		va_reported = true;
	}
	data->touchs = touchs;

	if (va_reported) {
		if (EVENT_NO_DOWN(data)) {
			input_report_key(data->input_dev, BTN_TOUCH, 0);
			input_mt_sync(data->input_dev);
		} else {
			input_report_key(data->input_dev, BTN_TOUCH, 1);
		}
	}

	input_sync(data->input_dev);
	return 0;
}
#endif

/*****************************************************************************
*  Name: fts_read_touchdata
*  Brief:
*  Input:
*  Output:
*  Return: return 0 if succuss
*****************************************************************************/
static int fts_read_touchdata(struct fts_ts_data *data)
{
	int ret = 0;
	int i = 0;
	u8 pointid;
	int base;
	struct ts_event *events = data->events;
	int max_touch_num = data->pdata->max_touch_number;
	u8 *buf = data->point_buf;
	struct i2c_client *client = data->client;

#if FTS_GESTURE_EN
	if (fts_gesture_readdata(data) == 0) {
		FTS_INFO("succuss to get gesture data in irq handler");
		return 1;
	}
#endif

#if FTS_PSENSOR_EN
	if ((fts_sensor_read_data(data) != 0) && (data->suspended == 1)) {
		ret = 1;
		return ret;
	}
#endif

#if FTS_POINT_REPORT_CHECK_EN
	fts_prc_queue_work(data);
#endif

	data->point_num = 0;
	data->touch_point = 0;

	memset(buf, 0xFF, data->pnt_buf_size);
	buf[0] = 0x00;

	ret = fts_i2c_read(data->client, buf, 1, buf, data->pnt_buf_size);
	if (ret < 0) {
		FTS_ERROR("read touchdata failed, ret:%d", ret);
		return ret;
	}
	data->point_num = buf[FTS_TOUCH_POINT_NUM] & 0x0F;

	if (data->ic_info.is_incell) {
		if ((data->point_num == 0x0F) && (buf[1] == 0xFF) && (buf[2] == 0xFF)
		    && (buf[3] == 0xFF) && (buf[4] == 0xFF) && (buf[5] == 0xFF) && (buf[6] == 0xFF)) {
			FTS_INFO("touch buff is 0xff, need recovery state");
			fts_tp_state_recovery(client);
			return -EIO;
		}
	}

	if (data->point_num > max_touch_num) {
		FTS_INFO("invalid point_num(%d)", data->point_num);
		return -EIO;
	}
#if (FTS_DEBUG_EN && (FTS_DEBUG_LEVEL == 2))
	fts_show_touch_buffer(buf, data->point_num);
#endif

	for (i = 0; i < max_touch_num; i++) {
		base = FTS_ONE_TCH_LEN * i;

		pointid = (buf[FTS_TOUCH_ID_POS + base]) >> 4;
		if (pointid >= FTS_MAX_ID)
			break;
		else if (pointid >= max_touch_num) {
			FTS_ERROR("ID(%d) beyond max_touch_number", pointid);
			return -EINVAL;
		}

		data->touch_point++;

		events[i].x = ((buf[FTS_TOUCH_X_H_POS + base] & 0x0F) << 8) +
			      (buf[FTS_TOUCH_X_L_POS + base] & 0xFF);
		events[i].y = ((buf[FTS_TOUCH_Y_H_POS + base] & 0x0F) << 8) +
			      (buf[FTS_TOUCH_Y_L_POS + base] & 0xFF);
		events[i].flag = buf[FTS_TOUCH_EVENT_POS + base] >> 6;
		events[i].id = buf[FTS_TOUCH_ID_POS + base] >> 4;
		events[i].area = buf[FTS_TOUCH_AREA_POS + base] >> 4;
		events[i].p = buf[FTS_TOUCH_PRE_POS + base];

		if (EVENT_DOWN(events[i].flag) && (data->point_num == 0)) {
			FTS_INFO("abnormal touch data from fw");
			return -EIO;
		}
	}
	if (data->touch_point == 0) {
		FTS_INFO("no touch point information");
		return -EIO;
	}

	return 0;
}

/*****************************************************************************
*  Name: fts_report_event
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static void fts_report_event(struct fts_ts_data *data)
{
#if FTS_MT_PROTOCOL_B_EN
	fts_input_report_b(data);
#else
	fts_input_report_a(data);
#endif
}

/*****************************************************************************
*  Name: fts_ts_interrupt
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static irqreturn_t fts_ts_interrupt(int irq, void *data)
{
	int ret = 0;
	struct fts_ts_data *ts_data = (struct fts_ts_data *)data;

	if (!ts_data) {
		FTS_ERROR("[INTR]: Invalid fts_ts_data");
		return IRQ_HANDLED;
	}
#if FTS_ESDCHECK_EN
	fts_esdcheck_set_intr(1);
#endif

	ret = fts_read_touchdata(ts_data);
	if (ret == 0) {
		mutex_lock(&ts_data->report_mutex);
		fts_report_event(ts_data);
		mutex_unlock(&ts_data->report_mutex);
	}
#if FTS_ESDCHECK_EN
	fts_esdcheck_set_intr(0);
#endif

	return IRQ_HANDLED;
}

/*****************************************************************************
*  Name: fts_irq_registration
*  Brief:
*  Input:
*  Output:
*  Return: return 0 if succuss, otherwise return error code
*****************************************************************************/
static int fts_irq_registration(struct fts_ts_data *ts_data)
{
	int ret = 0;
	struct fts_ts_platform_data *pdata = ts_data->pdata;

	ts_data->irq = gpio_to_irq(pdata->irq_gpio);
	FTS_INFO("irq in ts_data:%d irq in client:%d", ts_data->irq, ts_data->client->irq);
	if (ts_data->irq != ts_data->client->irq)
		FTS_ERROR("IRQs are inconsistent, please check <interrupts> & <focaltech,irq-gpio> in DTS");

	if (pdata->irq_gpio_flags == 0)
		pdata->irq_gpio_flags = IRQF_TRIGGER_FALLING;
	FTS_INFO("irq flag:%x", pdata->irq_gpio_flags);
	ret = request_threaded_irq(ts_data->irq, NULL, fts_ts_interrupt,
				   pdata->irq_gpio_flags | IRQF_ONESHOT, ts_data->client->name, ts_data);

	return ret;
}

/*****************************************************************************
*  Name: fts_input_init
*  Brief: input device init
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static int fts_input_init(struct fts_ts_data *ts_data)
{
	int ret = 0;
	int key_num = 0;
	struct fts_ts_platform_data *pdata = ts_data->pdata;
	struct input_dev *input_dev;
	int point_num;

	FTS_FUNC_ENTER();

	input_dev = input_allocate_device();
	if (!input_dev) {
		FTS_ERROR("Failed to allocate memory for input device");
		return -ENOMEM;
	}

	/* Init and register Input device */
	input_dev->name = FTS_DRIVER_NAME;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &ts_data->client->dev;

	input_set_drvdata(input_dev, ts_data);

	__set_bit(EV_SYN, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	if (pdata->have_key) {
		FTS_INFO("set key capabilities");
		for (key_num = 0; key_num < pdata->key_number; key_num++)
			input_set_capability(input_dev, EV_KEY, pdata->keys[key_num]);
	}
#if FTS_MT_PROTOCOL_B_EN
	input_mt_init_slots(input_dev, pdata->max_touch_number, INPUT_MT_DIRECT);
#else
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 0x0f, 0, 0);
#endif
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, pdata->x_min, pdata->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, pdata->y_min, pdata->y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 0xFF, 0, 0);
#if FTS_REPORT_PRESSURE_EN
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 0xFF, 0, 0);
#endif
	point_num = pdata->max_touch_number;
	ts_data->pnt_buf_size = point_num * FTS_ONE_TCH_LEN + 3;
	ts_data->point_buf = kzalloc(ts_data->pnt_buf_size, GFP_KERNEL);
	if (!ts_data->point_buf) {
		FTS_ERROR("failed to alloc memory for point buf!");
		ret = -ENOMEM;
		goto err_point_buf;
	}

	ts_data->events = kzalloc((point_num * sizeof(struct ts_event)), GFP_KERNEL);
	if (!ts_data->events) {

		FTS_ERROR("failed to alloc memory for point events!");
		ret = -ENOMEM;
		goto err_event_buf;
	}

	ret = input_register_device(input_dev);
	if (ret) {
		FTS_ERROR("Input device registration failed");
		goto err_input_reg;
	}

	ts_data->input_dev = input_dev;

	FTS_FUNC_EXIT();
	return 0;

err_input_reg:
	kfree_safe(ts_data->events);

err_event_buf:
	kfree_safe(ts_data->point_buf);

err_point_buf:
	input_set_drvdata(input_dev, NULL);
	input_free_device(input_dev);
	input_dev = NULL;

	FTS_FUNC_EXIT();
	return ret;
}

/*****************************************************************************
*  Name: fts_gpio_configure
*  Brief: Configure IRQ&RESET GPIO
*  Input:
*  Output:
*  Return: return 0 if succuss
*****************************************************************************/
static int fts_gpio_configure(struct fts_ts_data *data)
{
	int ret = 0;

	FTS_FUNC_ENTER();
	/* request irq gpio */
	if (gpio_is_valid(data->pdata->irq_gpio)) {
		ret = gpio_request(data->pdata->irq_gpio, "fts_irq_gpio");
		if (ret) {
			FTS_ERROR("[GPIO]irq gpio request failed");
			goto err_irq_gpio_req;
		}

#ifdef CONFIG_TS_FTS_MTK_INTERFACE
		ret = fts_irq_direction_input();
#else
		ret = gpio_direction_input(data->pdata->irq_gpio);
#endif
		if (ret) {
			FTS_ERROR("[GPIO]set_direction for irq gpio failed");
			goto err_irq_gpio_dir;
		}
	}

	/* request reset gpio */
	if (gpio_is_valid(data->pdata->reset_gpio)) {
		ret = gpio_request(data->pdata->reset_gpio, "fts_reset_gpio");
		if (ret) {
			FTS_ERROR("[GPIO]reset gpio request failed");
			goto err_irq_gpio_dir;
		}
#ifdef CONFIG_TS_FTS_MTK_INTERFACE
		fts_rst_output(1);
#else
		ret = gpio_direction_output(data->pdata->reset_gpio, 1);
#endif
		if (ret) {
			FTS_ERROR("[GPIO]set_direction for reset gpio failed");
			goto err_reset_gpio_dir;
		}
	}
	/* request power gpio */
	if (gpio_is_valid(data->pdata->power_gpio)) {
		ret = gpio_request(data->pdata->power_gpio, "fts_power_gpio");
		if (ret) {
			FTS_ERROR("[GPIO]power gpio request failed");
			goto err_reset_gpio_dir;
		}

		ret = gpio_direction_output(data->pdata->power_gpio, 1);
		if (ret) {
			FTS_ERROR("[GPIO]set_direction for power gpio failed");
			goto err_power_gpio_dir;
		}
	}
	FTS_FUNC_EXIT();
	return 0;
err_power_gpio_dir:
	if (gpio_is_valid(data->pdata->power_gpio))
		gpio_free(data->pdata->power_gpio);
err_reset_gpio_dir:
	if (gpio_is_valid(data->pdata->reset_gpio))
		gpio_free(data->pdata->reset_gpio);
err_irq_gpio_dir:
	if (gpio_is_valid(data->pdata->irq_gpio))
		gpio_free(data->pdata->irq_gpio);
err_irq_gpio_req:
	FTS_FUNC_EXIT();
	return ret;
}

/*****************************************************************************
*  Name: fts_get_dt_coords
*  Brief:
*  Input:
*  Output:
*  Return: return 0 if succuss, otherwise return error code
*****************************************************************************/
static int fts_get_dt_coords(struct device *dev, char *name, struct fts_ts_platform_data *pdata)
{
	int ret = 0;
	u32 coords[FTS_COORDS_ARR_SIZE] = { 0 };
	struct property *prop;
	struct device_node *np = dev->of_node;
	int coords_size;

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	coords_size = prop->length / sizeof(u32);
	if (coords_size != FTS_COORDS_ARR_SIZE) {
		FTS_ERROR("invalid:%s, size:%d", name, coords_size);
		return -EINVAL;
	}

	ret = of_property_read_u32_array(np, name, coords, coords_size);
	if (ret && (ret != -EINVAL)) {
		FTS_ERROR("Unable to read %s", name);
		return -ENODATA;
	}

	if (!strcmp(name, "focaltech,display-coords")) {
		pdata->x_min = coords[0];
		pdata->y_min = coords[1];
		pdata->x_max = coords[2];
		pdata->y_max = coords[3];
	} else {
		FTS_ERROR("unsupported property %s", name);
		return -EINVAL;
	}

	FTS_INFO("display x(%d %d) y(%d %d)", pdata->x_min, pdata->x_max, pdata->y_min, pdata->y_max);
	return 0;
}

/*****************************************************************************
*  Name: fts_parse_dt
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static int fts_parse_dt(struct device *dev, struct fts_ts_platform_data *pdata)
{
	int ret = 0;
	struct device_node *np = dev->of_node;
	u32 temp_val;

	FTS_FUNC_ENTER();

	ret = fts_get_dt_coords(dev, "focaltech,display-coords", pdata);
	if (ret < 0)
		FTS_ERROR("Unable to get display-coords");

	/* key */
	pdata->have_key = of_property_read_bool(np, "focaltech,have-key");
	if (pdata->have_key) {
		ret = of_property_read_u32(np, "focaltech,key-number", &pdata->key_number);
		if (ret)
			FTS_ERROR("Key number undefined!");

		ret = of_property_read_u32_array(np, "focaltech,keys", pdata->keys, pdata->key_number);
		if (ret)
			FTS_ERROR("Keys undefined!");
		else if (pdata->key_number > FTS_MAX_KEYS)
			pdata->key_number = FTS_MAX_KEYS;

		ret = of_property_read_u32(np, "focaltech,key-y-coord", &pdata->key_y_coord);
		if (ret)
			FTS_ERROR("Key Y Coord undefined!");

		ret = of_property_read_u32_array(np, "focaltech,key-x-coords",
						 pdata->key_x_coords, pdata->key_number);
		if (ret)
			FTS_ERROR("Key X Coords undefined!");

#ifdef FTS_COMBINATION_KEY
		ret = of_property_read_u32_array(np, "focaltech,comb-key-x-coords",
						 pdata->comb_key_x_coord, 3);
		if (ret)
			FTS_INFO("combKey X Coords undefined!");

		FTS_INFO("combKey X Coords %x %x %x", pdata->comb_key_x_coord[0],
				pdata->comb_key_x_coord[1], pdata->comb_key_x_coord[2]);
#endif

		FTS_INFO("VK(%d): (%d, %d, %d), [%d, %d, %d][%d]",
			 pdata->key_number, pdata->keys[0], pdata->keys[1], pdata->keys[2],
			 pdata->key_x_coords[0], pdata->key_x_coords[1], pdata->key_x_coords[2],
			 pdata->key_y_coord);
	}

	/* reset, irq gpio info */
	pdata->reset_gpio = of_get_named_gpio_flags(np, "focaltech,reset-gpio", 0, &pdata->reset_gpio_flags);
	if (pdata->reset_gpio < 0)
		FTS_ERROR("Unable to get reset_gpio");

	pdata->irq_gpio = of_get_named_gpio_flags(np, "focaltech,irq-gpio", 0, &pdata->irq_gpio_flags);
	if (pdata->irq_gpio < 0)
		FTS_ERROR("Unable to get irq_gpio");

	pdata->power_gpio = of_get_named_gpio_flags(np, "focaltech,power-gpio", 0, &pdata->power_gpio_flags);
	if (pdata->power_gpio < 0)
		FTS_ERROR("power_gpio");
	ret = of_property_read_u32(np, "focaltech,max-touch-number", &temp_val);
	if (ret == 0) {
		if (temp_val < 2)
			pdata->max_touch_number = 2;
		else if (temp_val > FTS_MAX_POINTS_SUPPORT)
			pdata->max_touch_number = FTS_MAX_POINTS_SUPPORT;
		else
			pdata->max_touch_number = temp_val;
	} else {
		FTS_ERROR("Unable to get max-touch-number");
		pdata->max_touch_number = FTS_MAX_POINTS_SUPPORT;
	}

	FTS_INFO("max touch number:%d, irq gpio:%d, reset gpio:%d",
		 pdata->max_touch_number, pdata->irq_gpio, pdata->reset_gpio);
	ret = of_property_read_u32(np, "focaltech,edge_limit_x_width", &temp_val);
	if (ret == 0) {
		if (temp_val > FTS_MAX_EDGE_LIMIT_WIDTH) {
			temp_val = FTS_MAX_EDGE_LIMIT_WIDTH;
		}
		pdata->limit_x_left = temp_val;
		pdata->limit_x_right = pdata->x_max - temp_val;
	} else {
		FTS_ERROR("Unable to get edge_limit_x_width");
		pdata->limit_x_left = 0;
		pdata->limit_x_right = pdata->x_max;
	}
	ret = of_property_read_u32(np, "focaltech,edge_limit_y_width", &temp_val);
	if (ret == 0) {
		if (temp_val > FTS_MAX_EDGE_LIMIT_WIDTH) {
			temp_val = FTS_MAX_EDGE_LIMIT_WIDTH;
		}
		pdata->limit_y_top = temp_val;
		pdata->limit_y_bottom = pdata->y_max - temp_val;
	} else {
		FTS_ERROR("Unable to get edge_limit_y_width");
		pdata->limit_y_top = 0;
		pdata->limit_y_bottom = pdata->y_max;
	}
	FTS_FUNC_EXIT();
	return 0;
}

#if defined(CONFIG_ADF) || defined(CONFIG_FB)
static void fts_tpd_resume_work_callback(struct work_struct *work)
{
	struct fts_ts_data *fts_data = container_of(work, struct fts_ts_data, fb_tpd_resume_work);

	FTS_FUNC_ENTER();
	fts_ts_resume(&fts_data->client->dev);
	fts_data->fb_tpd_suspend_flag = 0;
}
#endif

#if defined(CONFIG_ADF)
/*
 * touchscreen's suspend and resume state should rely on screen state,
 * as fb_notifier and early_suspend are all disabled on our platform,
 * we can only use adf_event now
 */
static int fts_adf_event_handler(struct notifier_block *nb, unsigned long action, void *data)
{

	struct adf_notifier_event *event = data;
	int adf_event_data = 0;
	int ret = 0;
	struct fts_ts_data *fts_data = container_of(nb, struct fts_ts_data, adf_notifier);

	if (action != ADF_EVENT_BLANK)
		return NOTIFY_DONE;

	adf_event_data = *(int *)event->data;
	FTS_INFO("receive adf event with adf_event_data=%d", adf_event_data);

	switch (adf_event_data) {
	case DRM_MODE_DPMS_ON:
		if (fts_data->fb_tpd_suspend_flag) {
			ret = queue_work(fts_data->fb_tpd_resume_wq, &fts_data->fb_tpd_resume_work);
			if (!ret) {
				FTS_DEBUG("start fb_tpd_resume_wq failed");
				return ret;
			}
		}

		break;
	case DRM_MODE_DPMS_OFF:
		ret = cancel_work_sync(&fts_data->fb_tpd_resume_work);
		if (!ret)
			FTS_DEBUG("cancel fb_tpd_resume_wq err = %d", ret);
		fts_ts_suspend(&fts_data->client->dev);
		fts_data->fb_tpd_suspend_flag = 1;

		break;
	default:
		FTS_ERROR("receive adf event with error data, adf_event_data=%d",
			adf_event_data);
		break;
	}
	return NOTIFY_OK;
}

#elif defined(CONFIG_FB)
/*****************************************************************************
*  Name: fb_notifier_callback
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	int err = 0;
	struct fts_ts_data *fts_data = container_of(self, struct fts_ts_data, fb_notif);

#if 0	/* zhangjian modify */
	if (evdata && evdata->data && event == FB_EVENT_BLANK && fts_data && fts_data->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			fts_ts_resume(&fts_data->client->dev);
		else if (*blank == FB_BLANK_POWERDOWN)
			fts_ts_suspend(&fts_data->client->dev);
	}
#else
	if (evdata && evdata->data && event == FB_EVENT_BLANK && fts_data && fts_data->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK) {
			if (fts_data->fb_tpd_suspend_flag) {
				err = queue_work(fts_data->fb_tpd_resume_wq, &fts_data->fb_tpd_resume_work);
				if (!err) {
					FTS_DEBUG("start fb_tpd_resume_wq failed");
					return err;
				}
			}
		} else if (*blank == FB_BLANK_POWERDOWN) {
			FTS_DEBUG("lcd off notifier.");
			err = cancel_work_sync(&fts_data->fb_tpd_resume_work);
			if (!err)
				FTS_DEBUG("cancel fb_tpd_resume_wq err = %d", err);
			fts_ts_suspend(&fts_data->client->dev);
			fts_data->fb_tpd_suspend_flag = 1;
		}
	}
#endif
	return 0;
}

#elif defined(CONFIG_HAS_EARLYSUSPEND)
/*****************************************************************************
*  Name: fts_ts_early_suspend
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static void fts_ts_early_suspend(struct early_suspend *handler)
{
	struct fts_ts_data *data = container_of(handler,
						struct fts_ts_data,
						early_suspend);

	fts_ts_suspend(&data->client->dev);
}

/*****************************************************************************
*  Name: fts_ts_late_resume
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static void fts_ts_late_resume(struct early_suspend *handler)
{
	struct fts_ts_data *data = container_of(handler,
						struct fts_ts_data,
						early_suspend);

	fts_ts_resume(&data->client->dev);
}
#endif

/*****************************************************************************
*  Name: fts_ts_probe
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static int fts_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	struct fts_ts_platform_data *pdata;
	struct fts_ts_data *ts_data;

	FTS_FUNC_ENTER();
	/* zhangjian add for TP compat */
	if (tpd_fw_cdev.TP_have_registered) {
		FTS_DEBUG("TP have registered by other TP.");
		FTS_FUNC_EXIT();
		return -EPERM;
	}
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		FTS_ERROR("I2C not supported");
		FTS_FUNC_EXIT();
		return -ENODEV;
	}

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			FTS_ERROR("Failed to allocate memory for platform data");
			return -ENOMEM;
		}
		ret = fts_parse_dt(&client->dev, pdata);
		if (ret)
			FTS_ERROR("[DTS]DT parsing failed");
	} else {
		pdata = client->dev.platform_data;
	}

	if (!pdata) {
		FTS_ERROR("no ts platform data found");
		return -EINVAL;
	}

	ts_data = devm_kzalloc(&client->dev, sizeof(*ts_data), GFP_KERNEL);
	if (!ts_data) {
		FTS_ERROR("Failed to allocate memory for fts_data");
		FTS_FUNC_EXIT();
		devm_kfree(&client->dev, pdata);
		return -ENOMEM;
	}

	fts_data = ts_data;
	ts_data->client = client;
	ts_data->pdata = pdata;
	i2c_set_clientdata(client, ts_data);

	ts_data->ts_workqueue = create_singlethread_workqueue("fts_wq");
	if (ts_data->ts_workqueue == NULL) {
		FTS_ERROR("failed to create fts workqueue");
	}

	spin_lock_init(&ts_data->irq_lock);
	mutex_init(&ts_data->report_mutex);

	ret = fts_input_init(ts_data);
	if (ret) {
		FTS_ERROR("fts input initialize fail");
		goto err_input_init;
	}
	ret = fts_power_source_init(ts_data);
	if (ret) {
		FTS_ERROR("fail to get vdd/vcc_i2c regulator");
		goto err_power_init;
	}

	ts_data->power_disabled = true;
	ret = fts_power_source_ctrl(ts_data, ENABLE);
	if (ret) {
		FTS_ERROR("fail to enable vdd/vcc_i2c regulator");
		goto err_power_ctrl;
	}
#if FTS_PINCTRL_EN
	ret = fts_pinctrl_init(ts_data);
	if (ret == 0) {
		fts_pinctrl_select_normal(ts_data);
	} else {
		FTS_ERROR("fail to fts pinctrl init");
		goto err_pinctrl_init;
	}
#endif


	ret = fts_gpio_configure(ts_data);
	if (ret) {
		FTS_ERROR("[GPIO]Failed to configure the gpios");
		goto err_gpio_config;
	}
#if (!FTS_CHIP_IDC)
	fts_reset_proc(200);
#endif

	ret = fts_get_ic_information(ts_data);
	if (ret) {
		FTS_ERROR("not focal IC, unregister driver");
		goto err_irq_req;
	}

#if FTS_PSENSOR_EN
	if (fts_sensor_init(ts_data) != 0) {
		FTS_ERROR("fts_sensor_init failed!");
		FTS_FUNC_EXIT();
		return 0;
	}
#endif

#if FTS_APK_NODE_EN
	ret = fts_create_apk_debug_channel(ts_data);
	if (ret) {
		FTS_ERROR("create apk debug node fail");
	}
#endif

#if FTS_SYSFS_NODE_EN
	ret = fts_create_sysfs(client);
	if (ret) {
		FTS_ERROR("create sysfs node fail");
	}
#endif

#if FTS_POINT_REPORT_CHECK_EN
	ret = fts_point_report_check_init(ts_data);
	if (ret) {
		FTS_ERROR("init point report check fail");
	}
#endif

	ret = fts_ex_mode_init(client);
	if (ret) {
		FTS_ERROR("init glove/cover/charger fail");
	}
	tpd_register_fw_class(ts_data);	/* zhangjian add */
#if FTS_GESTURE_EN
	ret = fts_gesture_init(ts_data);
	if (ret) {
		FTS_ERROR("init gesture fail");
	}
#endif

#if FTS_TEST_EN
	ret = fts_test_init(client);
	if (ret) {
		FTS_ERROR("init production test fail");
	}
#endif

#if FTS_ESDCHECK_EN
	ret = fts_esdcheck_init(ts_data);
	if (ret) {
		FTS_ERROR("init esd check fail");
	}
#endif

	ret = fts_irq_registration(ts_data);
	if (ret) {
		FTS_ERROR("request irq failed");
		goto err_irq_req;
	}
#if FTS_AUTO_UPGRADE_EN
	ret = fts_fwupg_init(ts_data);
	if (ret) {
		FTS_ERROR("init fw upgrade fail");
	}
#endif

#if defined(CONFIG_ADF)
	ts_data->fb_tpd_suspend_flag = 0;
	ts_data->fb_tpd_resume_wq = create_singlethread_workqueue("Fts_touch_resume");
	INIT_WORK(&ts_data->fb_tpd_resume_work, fts_tpd_resume_work_callback);
	ts_data->adf_notifier.notifier_call = fts_adf_event_handler;
	/* make sure we're the first to suspend/resume */
	ts_data->adf_notifier.priority = 1000;
	ret = adf_register_client(&ts_data->adf_notifier);
	if (ret < 0)
		FTS_ERROR("register adf notifier fail, cannot sleep when screen off");
	else
		FTS_INFO("synaptics tp register adf notifier succeed");
#elif defined(CONFIG_FB)
	ts_data->fb_tpd_suspend_flag = 0;
	ts_data->fb_tpd_resume_wq = create_singlethread_workqueue("Fts_touch_resume");
	INIT_WORK(&ts_data->fb_tpd_resume_work, fts_tpd_resume_work_callback);
	ts_data->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&ts_data->fb_notif);
	if (ret) {
		FTS_ERROR("[FB]Unable to register fb_notifier: %d", ret);
	}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	ts_data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + FTS_SUSPEND_LEVEL;
	ts_data->early_suspend.suspend = fts_ts_early_suspend;
	ts_data->early_suspend.resume = fts_ts_late_resume;
	register_early_suspend(&ts_data->early_suspend);
#endif
	tpd_fw_cdev.TP_have_registered = true;	/* zhangjian add for TP compat */
	FTS_FUNC_EXIT();
	return 0;

err_irq_req:
	if (gpio_is_valid(pdata->reset_gpio))
		gpio_free(pdata->reset_gpio);
	if (gpio_is_valid(pdata->irq_gpio))
		gpio_free(pdata->irq_gpio);
	if (gpio_is_valid(ts_data->pdata->power_gpio))
		gpio_free(ts_data->pdata->power_gpio);
err_gpio_config:
#if FTS_POWER_SOURCE_CUST_EN
#if FTS_PINCTRL_EN
	fts_pinctrl_select_release(ts_data);
err_pinctrl_init:
#endif
	fts_power_source_ctrl(ts_data, DISABLE);
err_power_ctrl:
	fts_power_source_release(ts_data);
err_power_init:
#else
err_power_ctrl:
err_power_init:
#endif
	kfree_safe(ts_data->point_buf);
	kfree_safe(ts_data->events);
	input_unregister_device(ts_data->input_dev);
err_input_init:
	if (ts_data->ts_workqueue)
		destroy_workqueue(ts_data->ts_workqueue);
	devm_kfree(&client->dev, ts_data);
	devm_kfree(&client->dev, pdata);
	FTS_FUNC_EXIT();
	return ret;
}

/*****************************************************************************
*  Name: fts_ts_remove
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static int fts_ts_remove(struct i2c_client *client)
{
	struct fts_ts_data *ts_data = i2c_get_clientdata(client);

	FTS_FUNC_ENTER();

#if FTS_PSENSOR_EN
	fts_sensor_remove(ts_data);
#endif

#if FTS_POINT_REPORT_CHECK_EN
	fts_point_report_check_exit(ts_data);
#endif

#if FTS_APK_NODE_EN
	fts_release_apk_debug_channel(ts_data);
#endif

#if FTS_SYSFS_NODE_EN
	fts_remove_sysfs(client);
#endif

	fts_ex_mode_exit(client);

#if FTS_AUTO_UPGRADE_EN
	fts_fwupg_exit(ts_data);
#endif

#if FTS_TEST_EN
	fts_test_exit(client);
#endif

#if FTS_ESDCHECK_EN
	fts_esdcheck_exit(ts_data);
#endif

#if FTS_GESTURE_EN
	fts_gesture_exit(client);
#endif

#if defined(CONFIG_ADF)
	adf_unregister_client(&ts_data->adf_notifier);
#elif defined(CONFIG_FB)
	if (fb_unregister_client(&ts_data->fb_notif))
		FTS_ERROR("Error occurred while unregistering fb_notifier.");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&ts_data->early_suspend);
#endif

	free_irq(ts_data->irq, ts_data);
	input_unregister_device(ts_data->input_dev);

	if (gpio_is_valid(ts_data->pdata->reset_gpio))
		gpio_free(ts_data->pdata->reset_gpio);

	if (gpio_is_valid(ts_data->pdata->irq_gpio))
		gpio_free(ts_data->pdata->irq_gpio);

	if (gpio_is_valid(ts_data->pdata->power_gpio))
		gpio_free(ts_data->pdata->power_gpio);
	if (ts_data->ts_workqueue)
		destroy_workqueue(ts_data->ts_workqueue);

#if FTS_POWER_SOURCE_CUST_EN
#if FTS_PINCTRL_EN
	fts_pinctrl_select_release(ts_data);
#endif
	fts_power_source_ctrl(ts_data, DISABLE);
	fts_power_source_release(ts_data);
#endif

	kfree_safe(ts_data->point_buf);
	kfree_safe(ts_data->events);

	devm_kfree(&client->dev, ts_data);

	FTS_FUNC_EXIT();
	return 0;
}

/*****************************************************************************
*  Name: fts_ts_suspend
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static int fts_ts_suspend(struct device *dev)
{
	int ret = 0;
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);
	struct input_dev *input_dev = ts_data->psensor_pdata->input_psensor_dev;

	FTS_FUNC_ENTER();

	if (ts_data->suspended) {
		FTS_INFO("Already in suspend state");
		return 0;
	}

	if (ts_data->fw_loading) {
		FTS_INFO("fw upgrade in process, can't suspend");
		return 0;
	}

	mutex_lock(&input_dev->mutex);

#if FTS_ESDCHECK_EN
	fts_esdcheck_suspend();
#endif

#if FTS_GESTURE_EN
	if (fts_gesture_suspend(ts_data->client) == 0) {
		ts_data->suspended = true;
		mutex_unlock(&input_dev->mutex);
		return 0;
	}
#endif

#if FTS_PSENSOR_EN
	if (fts_sensor_suspend(ts_data) != 0) {
		mutex_unlock(&input_dev->mutex);
		return 0;
	}
#endif

	fts_irq_disable();

	/* TP enter sleep mode */
	ret = fts_i2c_write_reg(ts_data->client, FTS_REG_POWER_MODE, FTS_REG_POWER_MODE_SLEEP_VALUE);
	if (ret < 0)
		FTS_ERROR("set TP to sleep mode fail, ret=%d", ret);

#if FTS_POWER_SOURCE_CUST_EN
	ret = fts_power_source_ctrl(ts_data, DISABLE);
	if (ret < 0) {
		FTS_ERROR("power off fail, ret=%d", ret);
	}
	if (gpio_is_valid(ts_data->pdata->power_gpio)) {
		FTS_INFO("[GPIO]power power off");
		ret = gpio_direction_output(ts_data->pdata->power_gpio, 0);
		if (ret) {
			FTS_ERROR("[GPIO]power off fail, ret=%d", ret);
		}
	}
#if FTS_PINCTRL_EN
	fts_pinctrl_select_suspend(ts_data);
#endif
#endif

	ts_data->suspended = true;
	mutex_unlock(&input_dev->mutex);

	FTS_FUNC_EXIT();
	return 0;
}

/*****************************************************************************
*  Name: fts_ts_resume
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static int fts_ts_resume(struct device *dev)
{
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);
	struct input_dev *input_dev = ts_data->psensor_pdata->input_psensor_dev;

	FTS_FUNC_ENTER();
	if (!ts_data->suspended) {
		FTS_DEBUG("Already in awake state");
		return 0;
	}

	fts_release_all_finger();

	mutex_lock(&input_dev->mutex);

#if FTS_POWER_SOURCE_CUST_EN
	fts_power_source_ctrl(ts_data, ENABLE);
	if (gpio_is_valid(ts_data->pdata->power_gpio)) {
		FTS_INFO("[GPIO]power power on");
		gpio_direction_output(ts_data->pdata->power_gpio, 1);
	}
#if FTS_PINCTRL_EN
	fts_pinctrl_select_normal(ts_data);
#endif
#endif

	if (!ts_data->ic_info.is_incell) {
		fts_reset_proc(200);
	}

	fts_tp_state_recovery(ts_data->client);

#if FTS_ESDCHECK_EN
	fts_esdcheck_resume();
#endif

#if FTS_GESTURE_EN
	if (fts_gesture_resume(ts_data->client) == 0) {
		ts_data->suspended = false;
		mutex_unlock(&input_dev->mutex);
		return 0;
	}
#endif

#if FTS_PSENSOR_EN
	if (fts_sensor_resume(ts_data) != 0) {
		mutex_unlock(&input_dev->mutex);
		FTS_FUNC_EXIT();
		return 0;
	}
#endif

	ts_data->suspended = false;
	fts_irq_enable();
	mutex_unlock(&input_dev->mutex);

	FTS_FUNC_EXIT();
	return 0;
}

/*****************************************************************************
* I2C Driver
*****************************************************************************/
static const struct i2c_device_id fts_ts_id[] = {
	{FTS_DRIVER_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, fts_ts_id);

static struct of_device_id fts_match_table[] = {
	{.compatible = "focaltech,fts",},
	{},
};

static struct i2c_driver fts_ts_driver = {
	.probe = fts_ts_probe,
	.remove = fts_ts_remove,
	.driver = {
		.name = FTS_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = fts_match_table,
	},
	.id_table = fts_ts_id,
};

/*****************************************************************************
*  Name: fts_ts_init
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static int __init fts_ts_init(void)
{
	int ret = 0;

	FTS_FUNC_ENTER();
	ret = i2c_add_driver(&fts_ts_driver);
	if (ret != 0) {
		FTS_ERROR("Focaltech touch screen driver init failed!");
	}
	FTS_FUNC_EXIT();
	return ret;
}

/*****************************************************************************
*  Name: fts_ts_exit
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static void __exit fts_ts_exit(void)
{
	i2c_del_driver(&fts_ts_driver);
}

module_init(fts_ts_init);
module_exit(fts_ts_exit);

MODULE_AUTHOR("FocalTech Driver Team");
MODULE_DESCRIPTION("FocalTech Touchscreen Driver");
MODULE_LICENSE("GPL v2");
