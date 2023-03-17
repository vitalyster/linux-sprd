//#include <asm/uaccess.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/math64.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/vmalloc.h>
//#include <linux/sched.h>
#include "stk5xxx.h"
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif

/*#define STK_QUALCOMM*/
/*#define CLASSDEVICE*/
#define STK_SPREADTRUM

#define STK_HEADER_VERSION          "0.0.2"

/*****************************************************************************
 * Global variable
 *****************************************************************************/
#define STK_INTERRUPT_MODE
 /*#define STK_POLLING_MODE*/

#ifdef STK_QUALCOMM
#include <linux/sensors.h>
#undef STK_SPREADTRUM
#elif defined STK_MTK
#undef STK_INTERRUPT_MODE
#undef STK_POLLING_MODE
#elif defined STK_SPREADTRUM
#include <linux/limits.h>
#include <linux/version.h>
/*
#undef STK_INTERRUPT_MODE
#define STK_POLLING_MODE
*/
#elif defined STK_ALLWINNER
#undef STK_INTERRUPT_MODE
#define STK_POLLING_MODE
#endif /* STK_QUALCOMM, STK_MTK, STK_SPREADTRUM, or STK_ALLWINNER */


#define STK5XXX_NAME    "sar_sensor"
#define STKINITERR    -1
static const u16 STK_ID[1] = { STK5013_ID };

struct stk_data {

	struct spi_device *spi;
	struct i2c_client *client;
	const struct stk_bus_ops *bops;
	struct mutex                i2c_lock;           /* mutex lock for register R/W */
	u8 *spi_buffer;        /* SPI buffer, used for SPI transfer. */
	atomic_t                    enabled;            /* chip is enabled or not */
	bool                        temp_enable;        /* record current power status. For Suspend/Resume used. */
	u8                          power_mode;
	u16                         pid;
	u8                          recv;
	stk5xxx_prx_nearby_type	last_nearby;
	int						    last_data[6];
#ifdef STK_INTERRUPT_MODE
	struct work_struct          stk_work;
	int                         int_pin;
	int                         irq;
#elif defined STK_POLLING_MODE
	struct work_struct          stk_work;
	struct hrtimer              stk_timer;
	ktime_t                     poll_delay;
#endif /* STK_INTERRUPT_MODE, STK_POLLING_MODE */
#if defined STK_QUALCOMM || defined STK_SPREADTRUM
#ifdef STK_QUALCOMM
#ifdef CLASSDEVICE
	struct sensors_classdev     sar_cdev;
#endif /* CLASSDEVICE*/
	u64                         fifo_start_ns;
#endif /* STK_QUALCOMM */
	struct input_dev            *input_dev;   /* data */
	ktime_t                     timestamp;
#elif defined STK_MTK
	struct sar_hw               hw;
	struct hwmsen_convert       cvt;
#endif /* STK_QUALCOMM, STK_SPREADTRUM, STK_MTK */
};
struct stk_data *stk_sar_ptr;
static char chip_info[20];

/*
#define STK_SAR_TAG                 "[stkSAR]"
#define STK_SAR_FUN(f)              printk(KERN_INFO STK_SAR_TAG" %s\n", __FUNCTION__)
#define dev_info(fmt, args...)   printk(KERN_INFO STK_SAR_TAG" %s/%d: "fmt"\n", __FUNCTION__, __LINE__, ##args)
#define dev_err(fmt, args...)   printk(KERN_ERR STK_SAR_TAG" %s/%d: "fmt"\n", __FUNCTION__, __LINE__, ##args)
*/
struct stk_bus_ops {
	int (*read)(struct stk_data*, unsigned short, unsigned char*);
	int (*read_block)(struct stk_data*, unsigned short, int, unsigned char*);
	int (*write)(struct stk_data*, unsigned short, unsigned char*);
};

#define STK_REG_READ(stk, addr, val)            (stk->bops->read(stk, addr, val))
#define STK_REG_READ_BLOCK(stk, addr, len, val) (stk->bops->read_block(stk, addr, len, val))
#define STK_REG_WRITE(stk, addr, val)           (stk->bops->write(stk, addr, val))

typedef struct {
	uint8_t     regBwsel;
	int         sample_rate_us;
} _stkODRMap;

#define STK_MIN_DELAY_US 2000   /* stkODRTable[4].sample_rate_us */
#define STK_MAX_DELAY_US 32000  /* stkODRTable[0].sample_rate_us */


#define STK_QUALCOMM_VERSION "0.0.1"
int stk_i2c_probe(struct i2c_client *client, const struct stk_bus_ops *stk5xxx_bus_ops);
int stk_i2c_remove(struct i2c_client *client);

#ifdef CONFIG_OF
static struct of_device_id stk5xxx_match_table[] = {
	{.compatible = "stk,stk5xxx", },
	{}
};
#endif /* CONFIG_OF */
#define STK_DRV_I2C_VERSION "0.0.1"
struct attribute_group stk_attribute_sar_group;
#define STK_C_VERSION       "0.0.1"
static void stk_clr_intr(struct stk_data *stk);
void stk_set_enable(struct stk_data *stk, char enable);
void stk_read_flag(struct stk_data *stk);
void stk_read_sar_data(struct stk_data *stk);
void stk_data_initialize(struct stk_data *stk);
int stk_get_pid(struct stk_data *stk);
/*
int stk_show_all_reg(struct stk_data* stk, char* show_buffer);
*/
int stk_reg_init(struct stk_data *stk);
int stk5xxx_suspend(struct device *dev);
int stk5xxx_resume(struct device *dev);
void stk_report_sar_data(struct stk_data *stk);
#if defined STK_INTERRUPT_MODE || defined STK_POLLING_MODE
void stk_work_queue(struct work_struct *work);
#endif /* defined STK_INTERRUPT_MODE || defined STK_POLLING_MODE */
#ifdef STK_INTERRUPT_MODE
int stk_irq_setup(struct stk_data *stk);
void stk_exit_irq_setup(struct stk_data *stk);
#endif /* STK_INTERRUPT_MODE */

void stk_data_initialize(struct stk_data *stk)
{
	atomic_set(&stk->enabled, 0);
	memset(stk->last_data, 0, sizeof(stk->last_data));
	stk->last_nearby = STK5XXX_PRX_NEAR_BY;
	dev_info(&stk->client->dev, "done");
}

static int stk_sw_reset(struct stk_data *stk)
{
	int err = 0;

	u16 reg = STK_SOFT_RESET_REG_ADDR;
	u32 val = STK_SOFT_RESET_REG_SOFT_RESET_CMD;

	err = STK_REG_WRITE(stk, reg, (u8 *)&val);

	if (err)
		return err;

	usleep_range(1000, 2000);
	return 0;
}

int stk_get_pid(struct stk_data *stk)
{
	int err = 0;
	u32 val = 0;

	err = STK_REG_READ(stk, STK_CHIP_INDEX_ADDR, (u8 *)&val);
	dev_err(&stk->client->dev, "PID reg=0x%x", val);

	if (err) {
		dev_err(&stk->client->dev, "failed to read PID");
		return -EIO;
	}

	stk->pid = (val & STK_CHIP_INDEX_CHIP_ID_MASK) >> STK_CHIP_INDEX_CHIP_ID_SHIFT;

	return err;
}
/*
int stk_show_all_reg(struct stk_data *stk, char *show_buffer)
{
	int reg_num, reg_count = 0;
	int return_size = 0;
	u32 val = 0;
	u16 reg_array[] = {
		STK_TRIGGER_REG_ADDR,
		STK_TRIGGER_CMD_REG_ADDR,
		STK_RXIO0_MUX_REG_ADDR,
		STK_RXIO1_MUX_REG_ADDR,
		STK_RXIO2_MUX_REG_ADDR,
		STK_RXIO3_MUX_REG_ADDR,
		STK_RXIO4_MUX_REG_ADDR,
		STK_RXIO5_MUX_REG_ADDR,
		STK_ADC_DEGLITCH_ADDR,
		STK_MOVEMENT_0_REG_ADDR,
		STK_MOVEMENT_2_REG_ADDR,
		STK_MOVEMENT_3_REG_ADDR,
		STK_SCAN_FACTOR_ADDR,
		STK_DOZE_MODE_ADDR,
		STK_SCAN_PERIOD_ADDR,
		STK_AFE_CTRL_PH0_REG_ADDR,
		STK_AFE_CTRL_PH00_REG_ADDR,


		STK_AFE_CTRL_PH00_REG_ADDR,
		STK_AFE_CTRL_PH1_REG_ADDR,
		STK_AFE_CTRL_PH11_REG_ADDR,
		STK_AFE_CTRL_PH2_REG_ADDR,
		STK_AFE_CTRL_PH22_REG_ADDR,
		STK_AFE_CTRL_PH3_REG_ADDR,
		STK_AFE_CTRL_PH33_REG_ADDR,
		STK_AFE_CTRL_PH4_REG_ADDR,
		STK_AFE_CTRL_PH44_REG_ADDR,
		STK_AFE_CTRL_PH5_REG_ADDR,
		STK_AFE_CTRL_PH55_REG_ADDR,

		STK_TCON0_REG_ADDR,
		STK_TCON1_REG_ADDR,

		STK_REGFILT0PH0_REG_ADDR,
		STK_REGSTATEDET0PH0_REG_ADDR,
		STK_REGSTATEDET2PH0_REG_ADDR,
		STK_REGSTATEDET3PH0_REG_ADDR,

		STK_TRIGGER_REG_ADDR,
		STK_REGADVDIG1PH0_REG_ADDR,
		STK_REGFILT0PH1_REG_ADDR,

		STK_TRIGGER_CMD_REG_ADDR,
		STK_REGSTATEDET0PH0_REG_ADDR,
		STK_IRQ_SOURCE_ENABLE_REG_ADDR,

	};


	if (show_buffer == NULL)
		return STKINITERR;

	reg_num = sizeof(reg_array) / sizeof(u16);

	for (reg_count = 0; reg_count < reg_num; reg_count++) {
		STK_REG_READ(stk, reg_array[reg_count], (u8 *)&val);
		return_size += snprintf(buf+return_size, 32, "reg[0x%x] = 0x%x\n", reg_array[reg_count], val);
	}

	return return_size;
}
*/
int stk_reg_init(struct stk_data *stk)
{
	int err = 0;
	u16 reg_count = 0;
	u16 reg_num = 0;
	u16 reg;
	u32 val;


	stk5xxx_register_table stk5xxx_default_register_table[] = {
		{STK_TRIGGER_REG_ADDR,          STK_TRIGGER_REG_COMPEN_DISABLE_ALL},
		{STK_TRIGGER_CMD_REG_ADDR,      STK_TRIGGER_CMD_REG_PHEN_ENABLE    },

		{STK_RXIO0_MUX_REG_ADDR,        STK_RXIO0_MUX_REG_VALUE            },
		{STK_RXIO1_MUX_REG_ADDR,        STK_RXIO1_MUX_REG_VALUE            },
		{STK_RXIO2_MUX_REG_ADDR,        STK_RXIO2_MUX_REG_VALUE            },
		{STK_RXIO3_MUX_REG_ADDR,        STK_RXIO3_MUX_REG_VALUE            },
		{STK_RXIO4_MUX_REG_ADDR,        STK_RXIO4_MUX_REG_VALUE            },
		{STK_RXIO5_MUX_REG_ADDR,        STK_RXIO5_MUX_REG_VALUE            },

		{STK_ADC_DEGLITCH_ADDR,         STK_ADC_DEGLITCH_VALUE             },
		{STK_MOVEMENT_0_REG_ADDR,        STK_MOVEMENT_0_REG_VALUE            },
		{STK_MOVEMENT_2_REG_ADDR,        STK_MOVEMENT_2_REG_VALUE            },
		{STK_MOVEMENT_3_REG_ADDR,        STK_MOVEMENT_3_REG_VALUE            },

		{STK_SCAN_FACTOR_ADDR,            STK_SCAN_FACTOR_VALUE            },
		{STK_DOZE_MODE_ADDR,             STK_DOZE_MODE_VALUE            },
		{STK_SCAN_PERIOD_ADDR,          STK_SCAN_PERIOD_VALUE            },

		{STK_AFE_CTRL_PH0_REG_ADDR,     STK_AFE_CTRL_PH0_REG_VALUE         },
		{STK_AFE_CTRL_PH00_REG_ADDR,    STK_AFE_CTRL_PH00_REG_VALUE        },


		{STK_AFE_CTRL_PH00_REG_ADDR,             STK_AFE_CTRL_PH00_REG_VALUE  },
		{STK_AFE_CTRL_PH1_REG_ADDR,              STK_AFE_CTRL_PH1_REG_VALUE   },
		{STK_AFE_CTRL_PH11_REG_ADDR,             STK_AFE_CTRL_PH11_REG_VALUE  },
		{STK_AFE_CTRL_PH2_REG_ADDR,              STK_AFE_CTRL_PH2_REG_VALUE   },
		{STK_AFE_CTRL_PH22_REG_ADDR,             STK_AFE_CTRL_PH22_REG_VALUE  },
		{STK_AFE_CTRL_PH3_REG_ADDR,              STK_AFE_CTRL_PH3_REG_VALUE   },
		{STK_AFE_CTRL_PH33_REG_ADDR,             STK_AFE_CTRL_PH33_REG_VALUE  },
		{STK_AFE_CTRL_PH4_REG_ADDR,              STK_AFE_CTRL_PH4_REG_VALUE   },
		{STK_AFE_CTRL_PH44_REG_ADDR,             STK_AFE_CTRL_PH44_REG_VALUE  },
		{STK_AFE_CTRL_PH5_REG_ADDR,              STK_AFE_CTRL_PH5_REG_VALUE   },
		{STK_AFE_CTRL_PH55_REG_ADDR,             STK_AFE_CTRL_PH55_REG_VALUE  },
		{STK_ANA_CTRL0_REG_ADDR,        STK_ANA_CTRL0_REG_VALUE            },
		{STK_ANA_CTRL1_REG_ADDR,        STK_ANA_CTRL1_REG_VALUE            },

		{STK_S_AND_H_REG_ADDR,          STK_S_AND_H_REG_VALUE              },

		{STK_RX_TIMING0_REG_ADDR,       STK_RX_TIMING0_REG_VALUE           },
		{STK_RX_TIMING1_REG_ADDR,       STK_RX_TIMING1_REG_VALUE           },
		{STK_RX_TIMING2_REG_ADDR,       STK_RX_TIMING2_REG_VALUE           },
		{STK_RX_TIMING3_REG_ADDR,       STK_RX_TIMING3_REG_VALUE           },
		{STK_RX_TIMING4_REG_ADDR,       STK_RX_TIMING4_REG_VALUE           },

		{STK_TCON0_REG_ADDR,            STK_TCON0_REG_VALUE                },
		{STK_TCON1_REG_ADDR,            STK_TCON1_REG_VALUE                },
		{STK_REGFILT0PH0_REG_ADDR,               STK_REGFILT0PH0_REG_VALUE    },
		{STK_REGSTATEDET0PH0_REG_ADDR,           STK_REGSTATEDET0PH0_REG_VALUE},
		{STK_REGSTATEDET2PH0_REG_ADDR,           STK_REGSTATEDET2PH0_REG_VALUE},
		{STK_REGSTATEDET3PH0_REG_ADDR,           STK_REGSTATEDET3PH0_REG_VALUE},
		{STK_RX_GAIN_REG_ADDR,                   STK_RX_GAIN_REG_VALUE        },
		{STK_REGADVDIG1PH0_REG_ADDR,             STK_REGADVDIG1PH0_REG_VALUE  },
		{STK_REGFILT0PH1_REG_ADDR,               STK_REGFILT0PH1_REG_VALUE    },
		{STK_REGSTATEDET0PH1_REG_ADDR,           STK_REGSTATEDET0PH1_REG_VALUE},
		{STK_REGSTATEDET2PH1_REG_ADDR,           STK_REGSTATEDET2PH1_REG_VALUE},
		{STK_REGSTATEDET3PH1_REG_ADDR,           STK_REGSTATEDET3PH1_REG_VALUE},
		{STK_REGADVDIG0PH1_REG_ADDR,             STK_REGADVDIG0PH1_REG_VALUE  },
		{STK_REGADVDIG1PH1_REG_ADDR,             STK_REGADVDIG1PH1_REG_VALUE  },
		{STK_REGFILT0PH2_REG_ADDR,               STK_REGFILT0PH2_REG_VALUE    },
		{STK_REGSTATEDET0PH2_REG_ADDR,           STK_REGSTATEDET0PH2_REG_VALUE},
		{STK_REGSTATEDET2PH2_REG_ADDR,           STK_REGSTATEDET2PH2_REG_VALUE},
		{STK_REGSTATEDET3PH2_REG_ADDR,           STK_REGSTATEDET3PH2_REG_VALUE},
		{STK_REGADVDIG0PH2_REG_ADDR,             STK_REGADVDIG0PH2_REG_VALUE  },
		{STK_REGADVDIG1PH2_REG_ADDR,             STK_REGADVDIG1PH2_REG_VALUE  },
		{STK_REGFILT0PH3_REG_ADDR,               STK_REGFILT0PH3_REG_VALUE    },
		{STK_REGSTATEDET0PH3_REG_ADDR,           STK_REGSTATEDET0PH3_REG_VALUE},
		{STK_REGSTATEDET2PH3_REG_ADDR,           STK_REGSTATEDET2PH3_REG_VALUE},
		{STK_REGSTATEDET3PH3_REG_ADDR,           STK_REGSTATEDET3PH3_REG_VALUE},
		{STK_REGADVDIG0PH3_REG_ADDR,             STK_REGADVDIG0PH3_REG_VALUE  },
		{STK_REGADVDIG1PH3_REG_ADDR,             STK_REGADVDIG1PH3_REG_VALUE  },
		{STK_REGFILT0PH4_REG_ADDR,               STK_REGFILT0PH4_REG_VALUE    },
		{STK_REGSTATEDET0PH4_REG_ADDR,           STK_REGSTATEDET0PH4_REG_VALUE},
		{STK_REGSTATEDET2PH4_REG_ADDR,           STK_REGSTATEDET2PH4_REG_VALUE},
		{STK_REGSTATEDET3PH4_REG_ADDR,           STK_REGSTATEDET3PH4_REG_VALUE},
		{STK_REGADVDIG0PH4_REG_ADDR,             STK_REGADVDIG0PH4_REG_VALUE  },
		{STK_REGADVDIG1PH4_REG_ADDR,             STK_REGADVDIG1PH4_REG_VALUE  },
		{STK_REGFILT0PH5_REG_ADDR,               STK_REGFILT0PH5_REG_VALUE    },
		{STK_REGSTATEDET0PH5_REG_ADDR,           STK_REGSTATEDET0PH5_REG_VALUE},
		{STK_REGSTATEDET2PH5_REG_ADDR,           STK_REGSTATEDET2PH5_REG_VALUE},
		{STK_REGSTATEDET3PH5_REG_ADDR,           STK_REGSTATEDET3PH5_REG_VALUE},
		{STK_REGADVDIG0PH5_REG_ADDR,             STK_REGADVDIG0PH5_REG_VALUE  },
		{STK_REGADVDIG1PH5_REG_ADDR,             STK_REGADVDIG1PH5_REG_VALUE  },
		{STK_REGREFCORRA_REG_ADDR,               STK_REGREFCORRA_REG_VALUE    },
		{STK_REGDBGVARSEL_REG_ADDR,              STK_REGDBGVARSEL_REG_VALUE   },


		{STK_TCON0_REG_ADDR,            STK_TCON0_REG_VALUE                },
		{STK_TCON1_REG_ADDR,            STK_TCON1_REG_VALUE                },

		{STK_TRIGGER_REG_ADDR,          STK_TRIGGER_REG_INIT_ALL           },
		{STK_TRIGGER_CMD_REG_ADDR,     STK_TRIGGER_CMD_REG_INIT_ALL        },
	};

#ifdef STK_INTERRUPT_MODE
	stk5xxx_register_table stk5xxx_default_ps_thd_table[] = {
		{STK_REGSTATEDET0PH0_REG_ADDR,  STK_REGSTATEDET0PH0_REG_VALUE},
		/*{STK_REGSTATEDET0PH1_REG_ADDR,  STK_REGSTATEDET0PH1_REG_VALUE},*/
		/*{STK_REGSTATEDET0PH2_REG_ADDR,  STK_REGSTATEDET0PH2_REG_VALUE},*/
		/*{STK_REGSTATEDET0PH3_REG_ADDR,  STK_REGSTATEDET0PH3_REG_VALUE},*/
		{STK_IRQ_SOURCE_ENABLE_REG_ADDR,  STK_IRQ_SOURCE_ENABLE_REG_VALUE}
	};
#endif /* STK_INTERRUPT_MODE */

	/* SW reset */
	err = stk_sw_reset(stk);

	reg_num = sizeof(stk5xxx_default_register_table) / sizeof(stk5xxx_register_table);

	for (reg_count = 0; reg_count < reg_num; reg_count++) {
		reg = stk5xxx_default_register_table[reg_count].address;
		val = stk5xxx_default_register_table[reg_count].value;
		err = STK_REG_WRITE(stk, reg, (u8 *)&val);
		if (err)
			return err;
	}

#ifdef STK_INTERRUPT_MODE

	reg_num = sizeof(stk5xxx_default_ps_thd_table) / sizeof(stk5xxx_register_table);

	for (reg_count = 0; reg_count < reg_num; reg_count++) {
		reg = stk5xxx_default_ps_thd_table[reg_count].address;
		val = stk5xxx_default_ps_thd_table[reg_count].value;
		err = STK_REG_WRITE(stk, reg, (u8 *)&val);
		if (err)
			return err;
	}
#endif /* STK_INTERRUPT_MODE */

	/* set power down for default*/
	stk_set_enable(stk, 1);

	return 0;
}

int stk5xxx_suspend(struct device *dev)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct stk_data *stk = i2c_get_clientdata(client);

	if (atomic_read(&stk->enabled)) {
		stk_set_enable(stk, 0);
		stk->temp_enable = true;
	} else
		stk->temp_enable = false;

	return 0;
}

int stk5xxx_resume(struct device *dev)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct stk_data *stk = i2c_get_clientdata(client);

	if (stk->temp_enable)
		stk_set_enable(stk, 1);

	stk->temp_enable = false;
	return 0;
}

#if defined STK_INTERRUPT_MODE || defined STK_POLLING_MODE
void stk_work_queue(struct work_struct *work)
{
	struct stk_data *stk = container_of(work, struct stk_data, stk_work);

	dev_err(&stk->client->dev, "stk_work_queue");
	stk_clr_intr(stk);
	stk_read_sar_data(stk);
	stk_report_sar_data(stk);

#ifdef STK_INTERRUPT_MODE
	enable_irq(stk->irq);
#elif defined STK_POLLING_MODE
	/*hrtimer_forward_now(&stk->stk_timer, stk->poll_delay);*/
#endif /* defined STK_INTERRUPT_MODE || defined STK_POLLING_MODE */
}
#endif /* defined STK_INTERRUPT_MODE || defined STK_POLLING_MODE */

#ifdef STK_INTERRUPT_MODE
static irqreturn_t stk_irq_handler(int irq, void *data)
{
	struct stk_data *stk = data;

	dev_err(&stk->client->dev, "stk_irq_handler");
	disable_irq_nosync(irq);
	schedule_work(&stk->stk_work);
	return IRQ_HANDLED;
}

int stk_irq_setup(struct stk_data *stk)
{
	int irq = 0;

	gpio_direction_input(stk->int_pin);
	irq = gpio_to_irq(stk->int_pin);

	if (irq < 0) {
		dev_err(&stk->client->dev, "gpio_to_irq(%d) failed", stk->int_pin);
		return STKINITERR;
	}

	stk->irq = irq;
	dev_err(&stk->client->dev, "irq #=%d, int pin=%d", stk->irq, stk->int_pin);

	irq = request_irq(stk->irq, stk_irq_handler, IRQF_TRIGGER_FALLING, "stk_sar_irq", stk);

	if (irq < 0) {
		dev_err(&stk->client->dev, "request_irq(%d) failed for %d", stk->irq, irq);
		return STKINITERR;
	}

	return irq;
}

void stk_exit_irq_setup(struct stk_data *stk)
{
	free_irq(stk->irq, stk->input_dev);
}

#elif defined STK_POLLING_MODE
static enum hrtimer_restart stk_timer_func(struct hrtimer *timer)
{
	struct stk_data *stk = container_of(timer, struct stk_data, stk_timer);

	schedule_work(&stk->stk_work);
	hrtimer_forward_now(&stk->stk_timer, stk->poll_delay);
	return HRTIMER_RESTART;
}
#endif /* STK_INTERRUPT_MODE, STK_POLLING_MODE */

static int stk_reg_write(struct stk_data *stk, u16 reg, u8 *val)
{
	int error = 0;

	u8 buffer_inverse[6] = { reg >> 8, reg & 0xff, val[3], val[2], val[1], val[0] };

	struct i2c_msg msgs = {
			.addr = stk->client->addr,
			.flags = stk->client->flags & I2C_M_TEN,
			.len = 6,
			.buf = buffer_inverse
	};


	mutex_lock(&stk->i2c_lock);
	error = i2c_transfer(stk->client->adapter, &msgs, 1);
	mutex_unlock(&stk->i2c_lock);

	if (error == 1) {
		error = 0;
	} else if (error < 0) {
		dev_err(&stk->client->dev, "transfer failed to write reg:0x%x , error=%d", reg, error);
		return -EIO;
	}

	dev_err(&stk->client->dev, "size error in write reg:0x%x with error=%d", reg, error);

	return error;
}

static int stk_reg_read(struct stk_data *stk, u16 reg, int len, u8 *val)
{
	int error = 0;
	int i = 0;
	u16 reg_inverse = (reg & 0x00FF) << 8 | (reg & 0xFF00) >> 8;
	int read_length = (len <= 0) ? 1 * 4 : len * 4;
	u8 buffer_inverse[99] = { 0 };

	struct i2c_msg msgs[2] = {
		{
			.addr = stk->client->addr,
			.flags = 0,
			.len = 2,
			.buf = (u8 *)&reg_inverse
		},
		{
			.addr = stk->client->addr,
			.flags = I2C_M_RD,
			.len = read_length,
			.buf = buffer_inverse
		}
	};

	mutex_lock(&stk->i2c_lock);
	error = i2c_transfer(stk->client->adapter, msgs, 2);
	mutex_unlock(&stk->i2c_lock);

	if (error == 2) {
		error = 0;
		for (i = 0; i < read_length; i++) {
			val[i] = buffer_inverse[read_length - 1 - i];
		}
	} else if (error < 0) {
		dev_err(&stk->client->dev, "transfer failed to read reg:0x%x with len:%d, error=%d", reg, len, error);
		return -EIO;
	}

	dev_err(&stk->client->dev, "size error in reading reg:0x%x with len:%d, error=%d", reg, len, error);

	return error;
}

static ssize_t stk_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct stk_data *stk = dev_get_drvdata(dev);
	char en;

	en = atomic_read(&stk->enabled);
	return scnprintf(buf, PAGE_SIZE, "%d\n", en);
}

static ssize_t stk_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct stk_data *stk = dev_get_drvdata(dev);
	unsigned int data;
	int error;

	error = kstrtouint(buf, 10, &data);

	if (error) {
		dev_err(&stk->client->dev, "kstrtoul failed, error=%d", error);
		return error;
	}

	dev_err(&stk->client->dev, "stk_enable_store, data=%d", data);

	if ((data == 1) || (data == 0))
		stk_set_enable(stk, data);
	else
		dev_err(&stk->client->dev, "invalid argument, en=%d", data);

	return count;
}

static ssize_t stk_reg_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{

	struct stk_data *stk = dev_get_drvdata(dev);
	int err = 0;
	u32 reg_address = 0;
	u32 val = 0;

	if (sscanf(buf,  "%x %x",  &reg_address,  &val) != 2) {
		pr_err("%s - The number of data are wrong\n", __func__);
		return -EINVAL;
	}

	err = STK_REG_WRITE(stk, reg_address, (u8 *)&val);
	if (err) {
		pr_err("%s - stk_reg_store write err!\n", __func__);
		return err;
	}

	return count;
}

static ssize_t stk_flag_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct stk_data *stk = dev_get_drvdata(dev);

	dev_err(&stk->client->dev, "stk_flag_show");
	/*stk_clr_intr(stk);*/
	/*stk_read_flag(stk);*/
	return scnprintf(buf, PAGE_SIZE, "flag=%d\n", stk->last_nearby);
}

static ssize_t stk_value_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct stk_data *stk = dev_get_drvdata(dev);

	dev_err(&stk->client->dev, "stk_value_show");
	/*stk_clr_intr(stk);*/
	/*stk_read_sar_data(stk);*/
	return scnprintf(buf, PAGE_SIZE, "val[0]=%d\n", stk->last_data[0]);
}

static ssize_t stk_allreg_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct stk_data *stk = dev_get_drvdata(dev);
	int reg_num, reg_count = 0;
	int return_size = 0;
	u32 val = 0;
	u16 reg_array[] = {
		STK_TRIGGER_REG_ADDR,
		STK_TRIGGER_CMD_REG_ADDR,
		STK_RXIO0_MUX_REG_ADDR,
		STK_RXIO1_MUX_REG_ADDR,
		STK_RXIO2_MUX_REG_ADDR,
		STK_RXIO3_MUX_REG_ADDR,
		STK_RXIO4_MUX_REG_ADDR,
		STK_RXIO5_MUX_REG_ADDR,
		STK_ADC_DEGLITCH_ADDR,
		STK_MOVEMENT_0_REG_ADDR,
		STK_MOVEMENT_2_REG_ADDR,
		STK_MOVEMENT_3_REG_ADDR,
		STK_SCAN_FACTOR_ADDR,
		STK_DOZE_MODE_ADDR,
		STK_SCAN_PERIOD_ADDR,
		STK_AFE_CTRL_PH0_REG_ADDR,
		STK_AFE_CTRL_PH00_REG_ADDR,


		STK_AFE_CTRL_PH00_REG_ADDR,
		STK_AFE_CTRL_PH1_REG_ADDR,
		STK_AFE_CTRL_PH11_REG_ADDR,
		STK_AFE_CTRL_PH2_REG_ADDR,
		STK_AFE_CTRL_PH22_REG_ADDR,
		STK_AFE_CTRL_PH3_REG_ADDR,
		STK_AFE_CTRL_PH33_REG_ADDR,
		STK_AFE_CTRL_PH4_REG_ADDR,
		STK_AFE_CTRL_PH44_REG_ADDR,
		STK_AFE_CTRL_PH5_REG_ADDR,
		STK_AFE_CTRL_PH55_REG_ADDR,

		STK_TCON0_REG_ADDR,
		STK_TCON1_REG_ADDR,

		STK_REGFILT0PH0_REG_ADDR,
		STK_REGSTATEDET0PH0_REG_ADDR,
		STK_REGSTATEDET2PH0_REG_ADDR,
		STK_REGSTATEDET3PH0_REG_ADDR,

		STK_TRIGGER_REG_ADDR,
		STK_REGADVDIG1PH0_REG_ADDR,
		STK_REGFILT0PH1_REG_ADDR,

		STK_TRIGGER_CMD_REG_ADDR,
		STK_REGSTATEDET0PH0_REG_ADDR,
		STK_IRQ_SOURCE_ENABLE_REG_ADDR,
/*raw_data reg*/
		STK_REG_RAW_PH0_REG_ADDR,
		STK_REG_RAW_PH1_REG_ADDR,
		STK_REG_RAW_PH2_REG_ADDR,
		STK_REG_RAW_PH3_REG_ADDR,
		STK_REG_RAW_PH4_REG_ADDR,
		STK_REG_RAW_PH5_REG_ADDR,

/*base_data reg*/
		STK_REG_BASE_PH0_REG_ADDR,
		STK_REG_BASE_PH1_REG_ADDR,
		STK_REG_BASE_PH2_REG_ADDR,
		STK_REG_BASE_PH3_REG_ADDR,
		STK_REG_BASE_PH4_REG_ADDR,
		STK_REG_BASE_PH5_REG_ADDR,

/*diff_data reg*/
		STK_REG_DELTA_PH0_REG_ADDR,
		STK_REG_DELTA_PH1_REG_ADDR,
		STK_REG_DELTA_PH2_REG_ADDR,
		STK_REG_DELTA_PH3_REG_ADDR,
		STK_REG_DELTA_PH4_REG_ADDR,
		STK_REG_DELTA_PH5_REG_ADDR,

	};

	reg_num = sizeof(reg_array) / sizeof(u16);

	for (reg_count = 0; reg_count < reg_num; reg_count++) {
		STK_REG_READ(stk, reg_array[reg_count], (u8 *)&val);
		return_size += snprintf(buf+return_size, 32, "reg[0x%x] = 0x%x\n", reg_array[reg_count], val);
	}

	return return_size;
}

static ssize_t stk_chipinfo_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct stk_data *stk = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "pid=0x%x\n", stk->pid);
}

static u8 diff_ch_num = 0;
static ssize_t stk_diff_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk_data *stk = dev_get_drvdata(dev);
	int32_t delta_val = 0, baseline_val = 0, raw_val = 0;

	STK_REG_READ(stk, STK_REG_RAW_PH0_REG_ADDR + diff_ch_num * 4, (u8 *)&raw_val);
	STK_REG_READ(stk, STK_REG_BASE_PH0_REG_ADDR + diff_ch_num * 4, (u8 *)&baseline_val);
	STK_REG_READ(stk, STK_REG_DELTA_PH0_REG_ADDR + diff_ch_num * 4, (u8 *)&delta_val);
	raw_val /= 128;
	baseline_val /= 128;
	delta_val /= 128;
	pr_err("%s read ph%d raw_rag:0x%x=%d, base_rag:0x%x=%d, delta_rag:0x%x=%d\n",
			__func__,  diff_ch_num, STK_REG_RAW_PH0_REG_ADDR + diff_ch_num * 4, raw_val,
			STK_REG_BASE_PH0_REG_ADDR + diff_ch_num * 4, baseline_val,
			STK_REG_DELTA_PH0_REG_ADDR + diff_ch_num * 4, delta_val);
	return snprintf(buf, 64, "%d,%d,%d\n", raw_val, baseline_val, delta_val);

}

static ssize_t stk_diff_phx_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int phx = 1;

	if (sscanf(buf, "%d", &phx) != 1) {
		pr_err("%s - The number of data are wrong\n", __func__);
		return -EINVAL;
	}
	diff_ch_num = phx;

	return count;
}

static ssize_t stk_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk_data *stk = dev_get_drvdata(dev);

	if (stk->last_nearby == STK5XXX_PRX_NEAR_BY) {
		return snprintf(buf, 64, "1\n");
	} else {
		return snprintf(buf, 64, "0\n");
	}
}

static DEVICE_ATTR(enable, 0664, stk_enable_show, stk_enable_store);
static DEVICE_ATTR(value, 0444, stk_value_show, NULL);
static DEVICE_ATTR(flag, 0444, stk_flag_show, NULL);
static DEVICE_ATTR(reg, 0664, stk_allreg_show, stk_reg_store);
static DEVICE_ATTR(chipinfo, 0444, stk_chipinfo_show, NULL);
static DEVICE_ATTR(diff, 0664, stk_diff_show, stk_diff_phx_store);
static DEVICE_ATTR(status, 0444, stk_status_show, NULL);


static struct attribute *stk_attribute_sar[] = {
	&dev_attr_enable.attr,
	&dev_attr_value.attr,
	&dev_attr_flag.attr,
	&dev_attr_reg.attr,
	&dev_attr_chipinfo.attr,
	&dev_attr_diff.attr,
	&dev_attr_status.attr,
	NULL
};

struct attribute_group stk_attribute_sar_group = {
	.name = STK5XXX_NAME,
	.attrs = stk_attribute_sar,
};

static int stk_read(struct stk_data *stk, unsigned short addr, unsigned char *val)
{
	return stk_reg_read(stk, addr, 0, val);
}

static int stk_read_block(struct stk_data *stk, unsigned short addr, int len, unsigned char *val)
{
	return stk_reg_read(stk, addr, len, val);
}

static int stk_write(struct stk_data *stk, unsigned short addr, unsigned char *val)
{
	return stk_reg_write(stk, addr, val);
}

/* Bus operations */
static const struct stk_bus_ops stk5xxx_bus_ops = {
	.read = stk_read,
	.read_block = stk_read_block,
	.write = stk_write,
};

static int stk5xxx_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return stk_i2c_probe(client, &stk5xxx_bus_ops);
}

static int stk5xxx_i2c_remove(struct i2c_client *client)
{
	return stk_i2c_remove(client);
}

static int stk5xxx_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strlcpy(info->type, STK5XXX_NAME, sizeof(info->type));
	return 0;
}

#ifdef CONFIG_PM_SLEEP
/*
 * @brief: Suspend function for dev_pm_ops.
 *
 * @param[in] dev: struct device *
 *
 * @return: 0
 */
static int stk5xxx_i2c_suspend(struct device *dev)
{
	return stk5xxx_suspend(dev);
}

/*
 * @brief: Resume function for dev_pm_ops.
 *
 * @param[in] dev: struct device *
 *
 * @return: 0
 */
static int stk5xxx_i2c_resume(struct device *dev)
{
	return stk5xxx_resume(dev);
}

static const struct dev_pm_ops stk5xxx_pm_ops = {
	.suspend = stk5xxx_i2c_suspend,
	.resume = stk5xxx_i2c_resume,
};
#endif /* CONFIG_PM_SLEEP */

#ifdef CONFIG_ACPI
static const struct acpi_device_id stk5xxx_acpi_id[] = {
	{"STK5XXX", 0},
	{}
};
MODULE_DEVICE_TABLE(acpi, stk5xxx_acpi_id);
#endif /* CONFIG_ACPI */

static const struct i2c_device_id stk5xxx_i2c_id[] = {
	{STK5XXX_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, stk5xxx_i2c_id);

static struct i2c_driver stk5xxx_i2c_driver = {
	.probe = stk5xxx_i2c_probe,
	.remove = stk5xxx_i2c_remove,
	.detect = stk5xxx_i2c_detect,
	.id_table = stk5xxx_i2c_id,
	.class = I2C_CLASS_HWMON,
	.driver = {
	.owner = THIS_MODULE,
	.name = STK5XXX_NAME,
#ifdef CONFIG_PM_SLEEP
	.pm = &stk5xxx_pm_ops,
#endif
#ifdef CONFIG_ACPI
	.acpi_match_table = ACPI_PTR(stk5xxx_acpi_id),
#endif /* CONFIG_ACPI */
#ifdef CONFIG_OF
	.of_match_table = stk5xxx_match_table,
#endif/* CONFIG_OF */
	}
};

struct stk5xxx_platform_data {
	unsigned char   direction;
	int             interrupt_int1_pin;
};

static struct stk5xxx_platform_data stk_plat_data = {
	.direction = 1,
	.interrupt_int1_pin = 120,
};

#ifdef CLASSDEVICE
/* information read by HAL */
static struct sensors_classdev stk_cdev = {
	.name = "stk5xxx",
	.vendor = "Sensortek",
	.version = 1,
	.type = 5013,
	.max_range = "1", /* 4G mode: 4.0f*9.81f=39.24f */
	.resolution = "1", /* 4G mode,12-bit resolution: 9.81f/512.f=0.01916f */
	.sensor_power = "1",
	.min_delay = STK_MIN_DELAY_US,
	.max_delay = STK_MAX_DELAY_US,
	.delay_msec = 16,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.max_latency = 0,
	.flags = 0, /* SENSOR_FLAG_CONTINUOUS_MODE */
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
	.sensors_enable_wakeup = NULL,
	.sensors_set_latency = NULL,
	.sensors_flush = NULL,
	.sensors_calibrate = NULL,
	.sensors_write_cal_params = NULL,
};

static int stk_cdev_sensors_enable(struct sensors_classdev *sensors_cdev,
	unsigned int enabled)
{
	struct stk_data *stk = container_of(sensors_cdev, struct stk_data, sar_cdev);

	if (enabled == 0) {
		stk_set_enable(stk, 0);
	} else if (enabled == 1) {
		stk_set_enable(stk, 1);
	} else {
		dev_err(&stk->client->dev, "Invalid vlaue of input, input=%d", enabled);
		return -EINVAL;
	}

	return 0;
}

static int stk_cdev_sensors_poll_delay(struct sensors_classdev *sensors_cdev,
	unsigned int delay_msec)
{
	struct stk_data *stk = container_of(sensors_cdev, struct stk_data, sar_cdev);
#ifdef STK_INTERRUPT_MODE
	/* do nothing */
#elif defined STK_POLLING_MODE
	stk->poll_delay = ns_to_ktime(delay_msec * 1000 * NSEC_PER_USEC);
#endif /* STK_INTERRUPT_MODE, STK_POLLING_MODE */
	dev_info(&stk->client->dev, "stk_cdev_sensors_poll_delay ms=%d", delay_msec);
	return 0;
}


static int stk_cdev_sensors_enable_wakeup(struct sensors_classdev *sensors_cdev,
	unsigned int enable)
{
	struct stk_data *stk = container_of(sensors_cdev, struct stk_data, sar_cdev);

	dev_info(&stk->client->dev, "enable=%d", enable);
	return 0;
}

static int stk_cdev_sensors_flush(struct sensors_classdev *sensors_cdev)
{
	struct stk_data *stk = container_of(sensors_cdev, struct stk_data, sar_cdev);

	dev_info(&stk->client->dev, "stk_cdev_sensors_flush");
	return 0;
}
#endif /*CLASSDEVICE*/

static int stk5xxx_ps_set_thd(struct stk_data *stk)
{
	int err = 0;
	u16 reg;
	u32 val;

	reg = STK_REGSTATEDET0PH0_REG_ADDR;
	val = STK_REGSTATEDET0PH0_REG_VALUE;
	err = STK_REG_WRITE(stk, reg, (u8 *)&val);
	if (err) {
		dev_err(&stk->client->dev, "stk5xxx_ps_set_thd [0] fail");
		return err;
	}


	reg = STK_REGSTATEDET0PH1_REG_ADDR;
	val = STK_REGSTATEDET0PH1_REG_VALUE;
	err = STK_REG_WRITE(stk, reg, (u8 *)&val);
	if (err) {
		dev_err(&stk->client->dev, "stk5xxx_ps_set_thd [1] fail");
		return err;
	}

	reg = STK_REGSTATEDET0PH2_REG_ADDR;
	val = STK_REGSTATEDET0PH2_REG_VALUE;
	err = STK_REG_WRITE(stk, reg, (u8 *)&val);
	if (err) {
		dev_err(&stk->client->dev, "stk5xxx_ps_set_thd [2] fail");
		return err;
	}

	reg = STK_REGSTATEDET0PH3_REG_ADDR;
	val = STK_REGSTATEDET0PH3_REG_VALUE;
	err = STK_REG_WRITE(stk, reg, (u8 *)&val);
	if (err) {
		dev_err(&stk->client->dev, "stk5xxx_ps_set_thd [3] fail");
		return err;
	}

	return 0;
}

static void stk_clr_intr(struct stk_data *stk)
{
	u16 reg;
	u32 val;

	reg = STK_IRQ_SOURCE_ADDR;
	STK_REG_READ(stk, reg, (u8 *)&val);

	dev_err(&stk->client->dev, "stk_clr_intr::  state = 0x%x", val);
}

void stk_read_flag(struct stk_data *stk)
{
	u16 reg;
	u32 flag;
	/*read flag*/
	reg = STK_REG_IRQ_STATE0_REG_ADDR;
	STK_REG_READ(stk, reg, (u8 *)&flag);
	dev_info(&stk->client->dev, "stk_read_flag:: Read flag =0x%x", flag);

	if (flag & STK_REG_IRQ_STATE0_REG_PROX_STATE_MASK) {
		stk->last_nearby = STK5XXX_PRX_NEAR_BY;
	} else {
		stk->last_nearby = STK5XXX_PRX_FAR_AWAY;
	}
}

void stk_read_sar_data(struct stk_data *stk)
{
	u16 reg;
	u32 val[6];
	int output_data[6] = { 0 };
	int i = 0;

	dev_info(&stk->client->dev, "stk5xxx_ps_get_data start");
	/*stk->timestamp = ktime_get_boottime();*/

	reg = STK_REG_DELTA_PH0_REG_ADDR;
	STK_REG_READ(stk, reg, (u8 *)&val[0]);

	reg = STK_REG_DELTA_PH1_REG_ADDR;
	STK_REG_READ(stk, reg, (u8 *)&val[1]);

	reg = STK_REG_DELTA_PH2_REG_ADDR;
	STK_REG_READ(stk, reg, (u8 *)&val[2]);

	reg = STK_REG_DELTA_PH3_REG_ADDR;
	STK_REG_READ(stk, reg, (u8 *)&val[3]);

	reg = STK_REG_DELTA_PH4_REG_ADDR;
	STK_REG_READ(stk, reg, (u8 *)&val[4]);

	reg = STK_REG_DELTA_PH5_REG_ADDR;
	STK_REG_READ(stk, reg, (u8 *)&val[5]);

	for (i = 0; i < 6; i++) {
		if (val[i] & 0x80000000) {
			/*2's complement = 1's complement +1*/
			output_data[i] = ((~val[i] + 1) & 0xFFFFFF80) >> 7;
			output_data[i] *= -1;
		} else {
			output_data[i] = (int)((val[i] & 0xFFFFFF80) >> 7);
		}

		dev_info(&stk->client->dev, "stk_read_sar_data:: delta[%d] = %d(0x%X)", i, output_data[i], val[i]);
		stk->last_data[i] = output_data[i];
	}

	/*read flag*/
	stk_read_flag(stk);
}

void stk_report_sar_data(struct stk_data *stk)
{
	if (!stk->input_dev) {
		dev_err(&stk->client->dev, "No input device for sar data");
		return;
	}

	/*input_report_abs(stk->input_dev, ABS_DISTANCE, stk->last_data[0]);*/
	input_report_abs(stk->input_dev, ABS_RX, stk->last_nearby);
	input_sync(stk->input_dev);
	dev_info(&stk->client->dev, "stk_report_sar_data:: near/far flag =0x%x", stk->last_nearby);
}


void stk_set_enable(struct stk_data *stk, char enable)
{
	u16 reg = 0;
	u32 val = 0;

	dev_err(&stk->client->dev, "stk_set_enable en=%d", enable);
	if (enable) {
		stk5xxx_ps_set_thd(stk);

		reg = STK_TRIGGER_CMD_REG_ADDR;
		val = STK_TRIGGER_CMD_REG_EXIT_PAUSE_MODE;
		STK_REG_WRITE(stk, reg, (u8 *)&val);

#ifdef STK_INTERRUPT_MODE
		/* do nothing */
#elif defined STK_POLLING_MODE
		hrtimer_start(&stk->stk_timer, stk->poll_delay, HRTIMER_MODE_REL);
#endif /* STK_INTERRUPT_MODE, STK_POLLING_MODE */
	} else {
#ifdef STK_INTERRUPT_MODE
		/* do nothing */
#elif defined STK_POLLING_MODE
		hrtimer_cancel(&stk->stk_timer);
#endif /* STK_INTERRUPT_MODE, STK_POLLING_MODE */
		stk->last_nearby = STK5XXX_PRX_NEAR_BY;

		reg = STK_TRIGGER_CMD_REG_ADDR;
		val = STK_TRIGGER_CMD_REG_ENTER_PAUSE_MODE;
		STK_REG_WRITE(stk, reg, (u8 *)&val);
	}
	atomic_set(&stk->enabled, enable);

	/*reset int and nf flag*/
	stk_clr_intr(stk);
	stk_read_sar_data(stk);

	dev_err(&stk->client->dev, "stk_set_enable DONE");
}

static int stk_input_setup(struct stk_data *stk)
{
	int err = 0;

	/* input device: setup for sar */
	stk->input_dev = input_allocate_device();

	if (!stk->input_dev) {
		dev_err(&stk->client->dev, "input_allocate_device for sar failed");
		return -ENOMEM;
	}

	stk->input_dev->name = STK5XXX_NAME;
	stk->input_dev->id.bustype = BUS_I2C;
	input_set_capability(stk->input_dev, EV_ABS, ABS_RX);
	input_set_capability(stk->input_dev, EV_ABS, ABS_RY);
	input_set_capability(stk->input_dev, EV_ABS, ABS_RZ);

	stk->input_dev->dev.parent = &stk->client->dev;
	input_set_drvdata(stk->input_dev, stk);

	err = input_register_device(stk->input_dev);

	if (err) {
		dev_err(&stk->client->dev, "Unable to register input device: %s", stk->input_dev->name);
		input_free_device(stk->input_dev);
		return err;
	}
	return 0;
}

/*
 * @brief:
 *
 * @param[in/out] stk: struct stk_data *
 *
 * @return:
 *      0: Success
 *      others: Fail
 */
static int stk_init_qualcomm(struct stk_data *stk)
{
	int err = 0;

	if (stk_input_setup(stk)) {
		return STKINITERR;
	}

	/* sysfs: create file system */
	err = sysfs_create_group(&stk->client->dev.kobj,
		&stk_attribute_sar_group);
	if (err) {
		dev_err(&stk->client->dev, "Fail in sysfs_create_group, err=%d", err);
		goto err_sysfs_creat_group;
	}
#ifdef CLASSDEVICE
	stk->sar_cdev = stk_cdev;
	stk->sar_cdev.name = "stk5xxx";
	/*mark*/
	stk->sar_cdev.sensors_enable = stk_cdev_sensors_enable;
	stk->sar_cdev.sensors_poll_delay = stk_cdev_sensors_poll_delay;
	stk->sar_cdev.sensors_enable_wakeup = stk_cdev_sensors_enable_wakeup;
	stk->sar_cdev.sensors_flush = stk_cdev_sensors_flush;

	err = sensors_classdev_register(&stk->input_dev->dev, &stk->sar_cdev);
#endif /* CLASSDEVICE*/
	if (err) {
		dev_err(&stk->client->dev, "Fail in sensors_classdev_register, err=%d", err);
		goto err_sensors_classdev_register;
	}

	return 0;

err_sensors_classdev_register:
	sysfs_remove_group(&stk->client->dev.kobj, &stk_attribute_sar_group);
err_sysfs_creat_group:
	input_free_device(stk->input_dev);
	input_unregister_device(stk->input_dev);
	return STKINITERR;
}

/*
 * @brief: Exit qualcomm related settings safely.
 *
 * @param[in/out] stk: struct stk_data *
 */
static void stk_exit_qualcomm(struct stk_data *stk)
{
#ifdef CLASSDEVICE
	sensors_classdev_unregister(&stk->sar_cdev);
#endif /* CLASSDEVICE*/
	sysfs_remove_group(&stk->client->dev.kobj,
		&stk_attribute_sar_group);
	input_free_device(stk->input_dev);
	input_unregister_device(stk->input_dev);
}

#ifdef CONFIG_OF
/*
 * @brief: Parse data in device tree
 *
 * @param[in] dev: struct device *
 * @param[in/out] pdata: struct stk5xxx_platform_data *
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static int stk_parse_dt(struct device *dev,
	struct stk5xxx_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	const int *p;
	uint32_t int_flags;

	p = of_get_property(np, "stk,direction", NULL);

	if (p)
		pdata->direction = be32_to_cpu(*p);

	pdata->interrupt_int1_pin = of_get_named_gpio_flags(np,
		"stk5xxx,irq-gpio", 0, &int_flags);

	if (pdata->interrupt_int1_pin < 0) {
		dev_err(dev, "Unable to read stk5xxx,irq-gpio");
#ifdef STK_INTERRUPT_MODE
		return pdata->interrupt_int1_pin;
#else /* no STK_INTERRUPT_MODE */
		return 0;
#endif /* STK_INTERRUPT_MODE */
	}

	return 0; /* SUCCESS */
}
#else
static int stk_parse_dt(struct device *dev,
	struct stk5xxx_platform_data *pdata)
{
	return -ENODEV
}
#endif /* CONFIG_OF */

/*
 * @brief: Get platform data
 *
 * @param[in/out] stk: struct stk_data *
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static int get_platform_data(struct stk_data *stk)
{
	int err = 0;
	struct stk5xxx_platform_data *stk_platdata;

	if (stk->client->dev.of_node) {
		dev_info(&stk->client->dev,  "get_platform_data!\n");
		stk_platdata = devm_kzalloc(&stk->client->dev,
			sizeof(struct stk5xxx_platform_data), GFP_KERNEL);

		if (!stk_platdata) {
			dev_err(&stk->client->dev,  "Failed to allocate memory!\n");
			return -ENOMEM;
		}

		err = stk_parse_dt(&stk->client->dev, stk_platdata);

		if (err) {
			dev_err(&stk->client->dev,  "stk_parse_dt err=%d!\n", err);
			return err;
		}
	} else {
		if (stk->client->dev.platform_data != NULL) {
			dev_err(&stk->client->dev,  "probe with platform data.\n");
			stk_platdata = stk->client->dev.platform_data;
		} else {
			dev_err(&stk->client->dev, "probe with private platform data.\n");
			stk_platdata = &stk_plat_data;
		}
	}

#ifdef STK_INTERRUPT_MODE
	stk->int_pin = stk_platdata->interrupt_int1_pin;
#endif /* STK_INTERRUPT_MODE */
	/*stk->direction = stk_platdata->direction;*/
	return 0;
}

static struct class sar_sensor_class = {
	.name = "sarsensor",
	.owner = THIS_MODULE,
};

static ssize_t delay_show(struct class *class,
		struct class_attribute *attr,
		char *buf)
{
	dev_info(&stk_sar_ptr->client->dev, "delay_show");
	return snprintf(buf, 8, "%d\n", 200);
}

static ssize_t delay_store(struct class *class,
		struct class_attribute *attr,
		const char *buf, size_t count)
{
	dev_info(&stk_sar_ptr->client->dev, "delay_store");
	return count;
}

static CLASS_ATTR_RW(delay);

static ssize_t enable_show(struct class *class,
		struct class_attribute *attr,
		char *buf)
{
	dev_info(&stk_sar_ptr->client->dev, "enable_show");
	return snprintf(buf, 8, "%d\n", 1);
}

static ssize_t enable_store(struct class *class,
		struct class_attribute *attr,
		const char *buf, size_t count)
{
	dev_info(&stk_sar_ptr->client->dev, "enable_store");
	return count;
}

static CLASS_ATTR_RW(enable);

static ssize_t chip_info_show(struct class *class,
		struct class_attribute *attr,
		char *buf)
{
	dev_info(&stk_sar_ptr->client->dev, "chip_info_show, chip_info = %s\n", chip_info);
	return snprintf(buf, 25, "%s", chip_info);
}

static CLASS_ATTR_RO(chip_info);

static ssize_t status_show(struct class *class,
		struct class_attribute *attr,
		char *buf)
{
	dev_info(&stk_sar_ptr->client->dev, "status_show,status = %d\n", stk_sar_ptr->last_nearby);
	if (stk_sar_ptr->last_nearby == STK5XXX_PRX_NEAR_BY) {
		return snprintf(buf, 64, "1\n");
	} else {
		return snprintf(buf, 64, "0\n");
	}
}

static CLASS_ATTR_RO(status);

static ssize_t batch_show(struct class *class,
		struct class_attribute *attr,
		char *buf)
{
	dev_info(&stk_sar_ptr->client->dev, "batch_show sar sensor\n");
	return snprintf(buf, 64, "200\n");
}

static ssize_t batch_store(struct class *class,
		struct class_attribute *attr,
		const char *buf, size_t count)
{
	dev_info(&stk_sar_ptr->client->dev, "batch_store sar sensor\n");
	return count;
}
static CLASS_ATTR_RW(batch);

static ssize_t flush_show(struct class *class,
		struct class_attribute *attr,
		char *buf)
{
	dev_info(&stk_sar_ptr->client->dev, "flush_show sar sensor\n");
	return snprintf(buf, 64, "0\n");
}

static ssize_t flush_store(struct class *class,
		struct class_attribute *attr,
		const char *buf, size_t count)
{
	dev_info(&stk_sar_ptr->client->dev, "flush_store sar sensor\n");
	return count;
}
static CLASS_ATTR_RW(flush);

static ssize_t diff_show(struct class *class,
		struct class_attribute *attr,
		char *buf)
{
	struct stk_data *stk = stk_sar_ptr;
	int32_t delta_val = 0, baseline_val = 0, raw_val = 0;

	STK_REG_READ(stk, STK_REG_RAW_PH0_REG_ADDR + diff_ch_num * 4, (u8 *)&raw_val);
	STK_REG_READ(stk, STK_REG_BASE_PH0_REG_ADDR + diff_ch_num * 4, (u8 *)&baseline_val);
	STK_REG_READ(stk, STK_REG_DELTA_PH0_REG_ADDR + diff_ch_num * 4, (u8 *)&delta_val);
	raw_val /= 128;
	baseline_val /= 128;
	delta_val /= 128;
	pr_err("%s read ph%d raw_rag:0x%x=%d, base_rag:0x%x=%d, delta_rag:0x%x=%d\n",
			__func__,  diff_ch_num, STK_REG_RAW_PH0_REG_ADDR + diff_ch_num * 4, raw_val,
			STK_REG_BASE_PH0_REG_ADDR + diff_ch_num * 4, baseline_val,
			STK_REG_DELTA_PH0_REG_ADDR + diff_ch_num * 4, delta_val);
	return snprintf(buf, 64, "%d,%d,%d\n", raw_val, baseline_val, delta_val);
}

static ssize_t diff_store(struct class *class,
		struct class_attribute *attr,
		const char *buf, size_t count)
{
	int phx = 1;

	if (sscanf(buf, "%d", &phx) != 1) {
		pr_err("%s - The number of data are wrong\n", __func__);
		return -EINVAL;
	}
	diff_ch_num = phx;

	return count;
}
static CLASS_ATTR_RW(diff);

/*
 * @brief: Probe function for i2c_driver.
 *
 * @param[in] client: struct i2c_client *
 * @param[in] stk_bus_ops: const struct stk_bus_ops *
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
int stk_i2c_probe(struct i2c_client *client, const struct stk_bus_ops *stk5xxx_bus_ops)
{
	int err = 0;
	struct stk_data *stk;

	dev_info(&client->dev, "STK_HEADER_VERSION: %s ", STK_HEADER_VERSION);
	dev_info(&client->dev, "STK_C_VERSION: %s ", STK_C_VERSION);
	dev_info(&client->dev, "STK_DRV_I2C_VERSION: %s ", STK_DRV_I2C_VERSION);
	dev_info(&client->dev, "STK_QUALCOMM_VERSION: %s ", STK_QUALCOMM_VERSION);

	if (client == NULL) {
		return -ENOMEM;
	} else if (stk5xxx_bus_ops == NULL) {
		dev_err(&client->dev, "cannot get stk_bus_ops. EXIT");
		return -EIO;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = i2c_get_functionality(client->adapter);
		dev_err(&client->dev, "i2c_check_functionality error, functionality=0x%x", err);
		return -EIO;
	}

	snprintf(chip_info, sizeof(chip_info), "%s", "sensortek");

	/* kzalloc: allocate memory and set to zero. */
	stk = kzalloc(sizeof(struct stk_data), GFP_KERNEL);

	if (!stk) {
		dev_err(&client->dev, "memory allocation error");
		return -ENOMEM;
	}

	client->addr = 0x28;
	stk->client = client;

	stk->bops = stk5xxx_bus_ops;
	i2c_set_clientdata(client, stk);
	mutex_init(&stk->i2c_lock);

	if (get_platform_data(stk))
		goto err_free_mem;

	err = stk_get_pid(stk);
	if (err)
		goto err_free_mem;

	dev_info(&client->dev, "PID 0x%x", stk->pid);


	stk_data_initialize(stk);

#ifdef STK_INTERRUPT_MODE
	INIT_WORK(&stk->stk_work, stk_work_queue);

	if (gpio_request(stk->int_pin, "stk_sar_int")) {
		dev_err(&client->dev, "gpio_request failed");
		goto err_free_mem;
	}

	err = stk_irq_setup(stk);
	if (err < 0) {
		goto err_cancel_work_sync;
	}
#elif defined STK_POLLING_MODE
	INIT_WORK(&stk->stk_work, stk_work_queue);
	hrtimer_init(&stk->stk_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	stk->poll_delay = ns_to_ktime(STK_POLLING_TIME * NSEC_PER_USEC);/*ktime_set(1, 0);*/
	stk->stk_timer.function = stk_timer_func;
#endif /* STK_INTERRUPT_MODE, STK_POLLING_MODE */

	if (stk_reg_init(stk)) {
		dev_err(&client->dev, "stk5xxx initialization failed");
		goto err_exit;
	}
	if (stk_init_qualcomm(stk)) {
		dev_err(&client->dev, "stk_init_qualcomm failed");
		goto err_exit;
	}

	stk_sar_ptr = stk;
/*add class sysfs*/
	err = class_register(&sar_sensor_class);
	if (err < 0) {
		dev_info(&client->dev, "Create fsys class failed (%d)\n", err);
		return err;
	}

	err = class_create_file(&sar_sensor_class, &class_attr_delay);
	if (err < 0) {
		dev_info(&client->dev, "Create delay file failed (%d)\n", err);
		goto err_class_creat;
	}

	err = class_create_file(&sar_sensor_class, &class_attr_enable);
	if (err < 0) {
		dev_info(&client->dev, "Create enable file failed (%d)\n", err);
		goto err_class_creat;
	}

	err = class_create_file(&sar_sensor_class, &class_attr_chip_info);
	if (err < 0) {
		dev_info(&client->dev, "Create chip_info file failed (%d)\n", err);
		goto err_class_creat;
	}

	err = class_create_file(&sar_sensor_class, &class_attr_batch);
	if (err < 0) {
		dev_info(&client->dev, "Create batch file failed (%d)\n", err);
		goto err_class_creat;
	}

	err = class_create_file(&sar_sensor_class, &class_attr_flush);
	if (err < 0) {
		dev_info(&client->dev, "Create flush file failed (%d)\n", err);
		goto err_class_creat;
	}

	err = class_create_file(&sar_sensor_class, &class_attr_diff);
	if (err < 0) {
		dev_info(&client->dev, "Create diff file failed (%d)\n", err);
		goto err_class_creat;
	}

/*ZTE add sys common status node*/
	err = class_create_file(&sar_sensor_class, &class_attr_status);
	if (err < 0) {
		dev_info(&client->dev, "Create status file failed (%d)\n", err);
		goto err_class_creat;
	}

	dev_info(&client->dev, "Success");
	return 0;

err_exit:
#ifdef STK_INTERRUPT_MODE
	stk_exit_irq_setup(stk);
err_cancel_work_sync:
	gpio_free(stk->int_pin);
	cancel_work_sync(&stk->stk_work);
#elif defined STK_POLLING_MODE
	hrtimer_try_to_cancel(&stk->stk_timer);
	cancel_work_sync(&stk->stk_work);
#endif /* STK_INTERRUPT_MODE, STK_POLLING_MODE */
err_free_mem:
	mutex_destroy(&stk->i2c_lock);
	kfree(stk);
	return err;
err_class_creat:
	dev_info(&client->dev, "unregister sar_sensor_class.\n");
	class_unregister(&sar_sensor_class);
	return err;
}

/*
 * @brief: Remove function for i2c_driver.
 *
 * @param[in] client: struct i2c_client *
 *
 * @return: 0
 */
int stk_i2c_remove(struct i2c_client *client)
{
	struct stk_data *stk = i2c_get_clientdata(client);

	stk_exit_qualcomm(stk);
#ifdef STK_INTERRUPT_MODE
	stk_exit_irq_setup(stk);
	gpio_free(stk->int_pin);
	cancel_work_sync(&stk->stk_work);
#elif defined STK_POLLING_MODE
	hrtimer_try_to_cancel(&stk->stk_timer);
	cancel_work_sync(&stk->stk_work);
#endif /* STK_INTERRUPT_MODE, STK_POLLING_MODE */
	mutex_destroy(&stk->i2c_lock);
	kfree(stk);
	return 0;
}

static int32_t __init stk5xxx_init(void)
{
	return i2c_add_driver(&stk5xxx_i2c_driver);
}

static void __exit stk5xxx_exit(void)
{
	return i2c_del_driver(&stk5xxx_i2c_driver);
}

late_initcall(stk5xxx_init);
module_exit(stk5xxx_exit);

MODULE_AUTHOR("Sensortek");
MODULE_DESCRIPTION("stk5xxx sar driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(STK_QUALCOMM_VERSION);
