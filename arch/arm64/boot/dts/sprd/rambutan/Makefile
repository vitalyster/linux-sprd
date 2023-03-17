ifeq ($(BSP_BUILD_DT_OVERLAY),y)

ifeq ($(BSP_BUILD_ANDROID_OS),y)
dtbo-$(CONFIG_ARCH_SPRD) += \
	ums512-1h10-overlay.dtbo

ums512-1h10-overlay.dtbo-base := ums512.dtb

endif #//BSP_BUILD_ANDROID_OS

else #//AiAiAi or No DTBO
dtb-$(CONFIG_ARCH_SPRD) += \
	ums512.dtb
dtbo-$(CONFIG_ARCH_SPRD) += \
	ums512-1h10-overlay.dtbo

endif #//BSP_BUILD_DT_OVERLAY

always		:= $(dtb-y) $(dtbo-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb *.dtbo
