ifeq ($(BSP_BUILD_DT_OVERLAY),y)

ifeq ($(BSP_BUILD_ANDROID_OS),y)

ifeq ($(BSP_BUILD_FAMILY),sharkl3)
dtbo-$(CONFIG_ARCH_SPRD) += \
	sp9863a-3h10-overlay.dtbo

sp9863a-3h10-overlay.dtbo-base := sp9863a.dtb
endif

endif #//BSP_BUILD_ANDROID_OS

else #//AiAiAi or No DTBO
dtb-$(CONFIG_ARCH_SPRD) += \
	sp9863a.dtb
dtbo-$(CONFIG_ARCH_SPRD) += \
	sp9863a-3h10-overlay.dtbo

endif #//BSP_BUILD_DT_OVERLAY

always		:= $(dtb-y) $(dtbo-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb *.dtbo
