# SPDX-License-Identifier: GPL-2.0
ifeq ($(CONFIG_OF),y)
ifeq ($(BSP_BUILD_DT_OVERLAY),y)

ifeq ($(BSP_BUILD_ANDROID_OS),y)

ifeq ($(BSP_BUILD_FAMILY),sharkl3)
dtbo-$(CONFIG_ARCH_SPRD) += \
	sp9863a-1h10_go_32b-overlay.dtbo

sp9863a-1h10_go_32b-overlay.dtbo-base := sp9863a.dtb
endif

ifeq ($(BSP_BUILD_FAMILY),pike2)
dtbo-$(CONFIG_ARCH_SPRD) += sp7731e-1h10-overlay.dtbo

sp7731e-1h10-overlay.dtbo-base := sp7731e.dtb
endif

ifeq ($(BSP_BUILD_FAMILY),sharkle)
dtbo-$(CONFIG_ARCH_SPRD) += sp9832e-1h10_go-overlay.dtbo

sp9832e-1h10_go-overlay.dtbo-base := sp9832e.dtb
endif

endif #//BSP_BUILD_ANDROID_OS

endif #//BSP_BUILD_DT_OVERLAY
endif

dtstree		:= $(srctree)/$(src)
dtb-$(CONFIG_OF_ALL_DTBS) := $(patsubst $(dtstree)/%.dts,%.dtb, $(wildcard $(dtstree)/*.dts))

DTB_NAMES := $(subst $\",,$(CONFIG_BUILD_ARM_APPENDED_DTB_IMAGE_NAMES))
ifneq ($(DTB_NAMES),)
DTB_LIST := $(addsuffix .dtb,$(DTB_NAMES)) $(dtbo-y)
else
DTB_LIST := $(dtb-y) $(dtbo-y)
endif

targets += dtbs dtbs_install
targets += $(DTB_LIST)

always		:= $(DTB_LIST)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb *.dtbo
