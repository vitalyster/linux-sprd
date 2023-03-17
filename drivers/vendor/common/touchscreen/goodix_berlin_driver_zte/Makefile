# Makefile for the Himax touchscreen drivers.
ccflags-y += -I$(srctree)/drivers/vendor/common/touchscreen/
ccflags-y += -I$(srctree)/drivers/vendor/common/touchscreen/goodix_berlin_driver_zte/firmware_config/$(CONFIG_TS_FIRMWARE)/

obj-$(CONFIG_TOUCHSCREEN_GOODIX_BRL_I2C)        += goodix_brl_i2c.o
obj-$(CONFIG_TOUCHSCREEN_GOODIX_BRL)            += goodix_core.o
goodix_core-y   :=  goodix_ts_core.o goodix_brl_hw.o goodix_cfg_bin.o goodix_ts_utils.o goodix_brl_fwupdate.o
obj-$(CONFIG_TOUCHSCREEN_GOODIX_TS_TOOLS)       += goodix_ts_tools.o
obj-$(CONFIG_TOUCHSCREEN_GOODIX_TS_GESTURE)     += goodix_ts_gesture.o
obj-$(CONFIG_TOUCHSCREEN_GOODIX_BRL)            += goodix_common_interface.o
obj-$(CONFIG_TOUCHSCREEN_GOODIX_BRL_INSPECT)     += goodix_ts_inspect.o
