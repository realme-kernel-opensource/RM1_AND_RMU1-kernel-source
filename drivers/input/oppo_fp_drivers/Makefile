ifeq ($(CONFIG_OPPO_FINGERPRINT),y)
ifneq ($(filter MSM_17151 MSM_18031 MSM_18301 MSM_18305, $(OPPO_TARGET_DEVICE)),)
obj-y        += oppo_fp_common/
else
obj-y        += oppo_fp_common/
obj-y        += fpc/
endif
ifneq ($(filter MSM_17011 MSM_17015 MSM_17081 MSM_17085 MSM_18316, $(OPPO_TARGET_DEVICE)),)
obj-y        += goodix_fp/
endif
ifneq ($(filter full_oppo6763 full_oppo6771, $(shell echo $(TARGET_PRODUCT) | sed -e 's/_1.*//g')),)
obj-y        += goodix_fp/
endif
ifneq ($(filter MSM_18005 MSM_18323, $(OPPO_TARGET_DEVICE)),)
obj-y        += silead/
endif
endif
obj-y        += oppo_secure_common/
