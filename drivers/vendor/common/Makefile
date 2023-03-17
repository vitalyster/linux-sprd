
# Makefile for the common kernel device drivers.
obj-$(CONFIG_TOUCHSCREEN_VENDOR)	+= touchscreen/
obj-$(CONFIG_NXP_TFA9894_SMARTPA)	+= nxp_tfa9894/
obj-$(CONFIG_POWER_VENDOR_COMMON)	+= power/
obj-y   += vcfg/
#for AWINIC AW881XXSmart PA
obj-$(CONFIG_SND_SMARTPA_AW881XX) += awinic/
obj-$(CONFIG_SND_SOC_TFA98XX) += tfa9873/
obj-$(CONFIG_VENDOR_SQC_CHARGER)   += sqc/
obj-y   += sound/

#for sensor
obj-$(CONFIG_VENDOR_SOC_SENSOR)   += sensor/

#####Fingerprint
#Goodix
obj-$(CONFIG_PLATFORM_FINGERPRINT_GOODIX)    += fingerprint/goodix/
#Silead
obj-$(CONFIG_PLATFORM_FINGERPRINT_SILEAD)    += fingerprint/silead/
#Cdfinger
obj-$(CONFIG_PLATFORM_FINGERPRINT_CDFINGER)  += fingerprint/cdfinger/
#Chipone
obj-$(CONFIG_PLATFORM_FINGERPRINT_CHIPONE)   += fingerprint/chipone/
#Sunwave
obj-$(CONFIG_PLATFORM_FINGERPRINT_SUNWAVE)   += fingerprint/sunwave/
