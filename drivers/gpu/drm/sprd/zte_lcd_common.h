#ifndef ZTE_LCD_COMMON_H
#define ZTE_LCD_COMMON_H

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/ctype.h>
#include <linux/debugfs.h>
#include <linux/sysfs.h>
#include <linux/proc_fs.h>
#include <linux/kobject.h>
#include <linux/of.h>
#include <linux/delay.h>

#ifdef CONFIG_ZTE_LCD_COMMON_FUNCTION
#include "sprd_panel.h"

#include "sprd_dsi.h"
struct mipi_dsi_device;


#ifdef CONFIG_ZTE_LCD_REG_DEBUG
#define ZTE_REG_LEN 64
#define REG_MAX_LEN 16 /*one lcd reg display info max length*/
enum {	/* read or write mode */
	REG_WRITE_MODE = 0,
	REG_READ_MODE
};
struct zte_lcd_reg_debug {
	char is_read_mode;  /*if 1 read ,0 write*/
	/*bool is_hs_mode;*/    /*if 1 hs mode, 0 lp mode*/
	char dtype;
	unsigned char length;
	char rbuf[ZTE_REG_LEN];
	char wbuf[ZTE_REG_LEN];
	char reserved[ZTE_REG_LEN];
};
#endif

#ifdef CONFIG_ZTE_LCD_CABC3_EXTREME_POWER_SAVE
enum {	/* lcd cabc mode */
	LCD_CABC_OFF_MODE = 0,
	LCD_CABC_LOW_MODE,
	LCD_CABC_MEDIUM_MODE,
	LCD_CABC_HIGH_MODE
};
#endif

#ifdef CONFIG_ZTE_LCD_BACKLIGHT_LEVEL_CURVE
enum {	/* lcd curve mode */
	CURVE_MATRIX_MAX_350_LUX = 1,
	CURVE_MATRIX_MAX_400_LUX,
	CURVE_MATRIX_MAX_450_LUX
};
#endif

struct zte_lcd_ctrl_data {
#ifdef CONFIG_ZTE_LCDBL_I2C_CTRL_VSP_VSN
	u32 lcd_bl_vsp_vsn_voltage;
	bool lcd_bl_vsp_vsn_enhance_disable;
#endif
#ifdef CONFIG_ZTE_LCD_BACKLIGHT_LEVEL_CURVE
	u32 lcd_bl_curve_mode;
	int (*zte_convert_brightness)(int level, u32 bl_max);
#endif
	const char *lcd_panel_name;
	const char *lcd_init_code_version;
	struct kobject *kobj;
	u32 lcd_esd_num;
	char lcd_bl_register_len;
	u32 lcd_bl_min_value;
	char lcd_bl_register_len_is_ilitek;
#ifdef CONFIG_ZTE_LCD_HBM_CTRL
	u32 lcd_hbm_mode;
	u32 lcd_hdr_on;
#endif
#ifdef CONFIG_ZTE_LCD_ESD_TWICE
	bool is_lcd_esd_twice;
#endif
#ifdef CONFIG_ZTE_LCD_DCSBL_CABC_GRADIENT
	bool close_dynamic_dimming;
#endif
	bool lcd_powerdown_for_shutdown;
	bool have_tp_gesture_power_off_seq;
	bool lcd_ctrl_tp_resetpin;
	bool reset_before_vsp;
	bool reset_down_before_vsp;
	u32  reset_down_delay_time;
	bool backlight_ctrl_by_lcdic_pwm;/*backlight source ctrl by lcdic or platform*/

#ifdef CONFIG_ZTE_LCD_DISABLE_SSC
	bool is_disable_ssc;
#endif
#ifdef CONFIG_ZTE_LCD_DELAY_OPEN_BL
	u32 lcd_delay_open_bl_value;
#endif
#ifdef CONFIG_ZTE_LCD_SUPPORT_ER68577
	bool is_support_er68577;
#endif
};

void zte_lcd_common_func(struct mipi_dsi_device *dispc, struct sprd_panel *panel);

const char *zte_get_lcd_panel_name(void);

#endif /*CONFIG_ZTE_LCD_COMMON_FUNCTION*/


/*WARNING: Single statement macros should not use a do {} while (0) loop*/
#define ZTE_LCD_INFO(fmt, args...) {pr_info("[SPRD_LCD]"fmt, ##args); }
#define ZTE_LCD_ERROR(fmt, args...) {pr_err("[SPRD_LCD][Error]"fmt, ##args); }

#endif

