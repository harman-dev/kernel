LOCAL_PATH := $(call my-dir)
BCM43xx_PATH := $(LOCAL_PATH)

$(info building bcmdhd module for $(COMBO_CHIP))
ifneq ($(findstring 4359, $(COMBO_CHIP)), $(empty))
$(info building bcmdhd module for PCIE)
$(eval $(call build_kernel_module,$(LOCAL_PATH)/,bcmdhd,CONFIG_BCMDHD=m CONFIG_BCMDHD_SDIO= CONFIG_BCMDHD_PCIE=y CONFIG_BCM4359=y CONFIG_DHD_USE_SCHED_SCAN=))
else
$(info trying to build bcmdhd driver for unknown chip, please enable it here...)
endif
