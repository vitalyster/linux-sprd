/*
 * Ilitek TouchScreen oem config.
 */

 #ifndef _ILITEK_CONFIG_H_
 #define _ILITEK_CONFIG_H_

/* Options */
#define TDDI_INTERFACE			BUS_I2C /* BUS_I2C(0x18) or BUS_SPI(0x1C) */
#define VDD_VOLTAGE			1800000
#define VCC_VOLTAGE			1800000
#define SPI_CLK				9      /* follow by clk list */
#define SPI_RETRY			5
#define IRQ_GPIO_NUM			66
#define TR_BUF_SIZE			(2*K) /* Buffer size of touch report */
#define TR_BUF_LIST_SIZE		(1*K) /* Buffer size of touch report */
#define SPI_TX_BUF_SIZE			4096
#define SPI_RX_BUF_SIZE			4096
#define WQ_ESD_DELAY			4000
#define WQ_BAT_DELAY			2000
#define AP_INT_TIMEOUT			600 /*600ms*/
#define MP_INT_TIMEOUT			5000 /*5s*/
#define MT_B_TYPE			ENABLE
#define TDDI_RST_BIND			DISABLE
#define MT_PRESSURE			DISABLE
#define ENABLE_WQ_ESD			DISABLE
#define ENABLE_WQ_BAT			DISABLE
#define ENABLE_GESTURE			ENABLE
#define REGULATOR_POWER			DISABLE
#define TP_SUSPEND_PRIO			ENABLE
#define RESUME_BY_DDI			DISABLE
#define BOOT_FW_UPDATE			ENABLE
#define MP_INT_LEVEL			DISABLE

/*if current interface is spi, must to hostdownload */
#if (TDDI_INTERFACE == BUS_SPI)
#define HOST_DOWN_LOAD			ENABLE
#else
#define HOST_DOWN_LOAD			DISABLE
#endif

/* Plaform compatibility */
#define CONFIG_PLAT_SPRD		DISABLE
#define I2C_DMA_TRANSFER		DISABLE
#define SPI_DMA_TRANSFER_SPLIT		ENABLE

 /* define the width and heigth of a screen. */
#define TOUCH_SCREEN_X_MIN			0
#define TOUCH_SCREEN_Y_MIN			0
#define TOUCH_SCREEN_X_MAX			720
#define TOUCH_SCREEN_Y_MAX			1600
#define MAX_TOUCH_NUM				10

/* define the range on panel */
#define TPD_HEIGHT				2048
#define TPD_WIDTH				2048
#define ILITEK_EDGE_LEVEL
#define ILITEK_REPORT_BY_ZTE_ALGO
#ifdef ILITEK_REPORT_BY_ZTE_ALGO
#define ili_left_edge_limit_v		6
#define ili_right_edge_limit_v		6
#define ili_left_edge_limit_h		6
#define ili_right_edge_limit_h		6
#define ili_left_edge_long_pess_v		20
#define ili_right_edge_long_pess_v	20
#define ili_left_edge_long_pess_h		40
#define ili_right_edge_long_pess_h	20
#define ili_long_press_max_count		80
#define ili_edge_long_press_check	1
#endif
#define DONT_NEED_EX_MODE_CMD

#define ILI_MODULE_NUM	1
#define ILI_VENDOR_ID_0 0x4
#define ILI_VENDOR_ID_1 0
#define ILI_VENDOR_ID_2 0
#define ILI_VENDOR_ID_3 0

#define ILI_VENDOR_0_NAME                         "tongxingda"
#define ILI_VENDOR_1_NAME                         "unknown"
#define ILI_VENDOR_2_NAME                         "unknown"
#define ILI_VENDOR_3_NAME                         "unknown"
#endif

