#
# Makefile for nfc devices
#
pn553-objs = nfc.o sn1xx.o pn8xt.o
obj-$(CONFIG_NXP_NFC_ESE_DEVICE_PM) += pn553.o
ccflags-$(CONFIG_NXP_NFC_SN1XX)  := -DNFC_PLATFORM=sn1xx
ccflags-$(CONFIG_NXP_NFC_PN8XT_PM)  := -DNFC_PLATFORM=pn8xt