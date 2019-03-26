ccflags-$(CONFIG_ARCH_QCOM) += -I$(srctree)/drivers/gpu/msm/
ccflags-$(CONFIG_MACH_MT6771) += -I$(srctree)/drivers/misc/mediatek/include/ -I$(srctree)/drivers/misc/mediatek/base/power/include/	\
	-I$(srctree)/drivers/misc/mediatek/base/power/mcdi/include -I$(srctree)/drivers/misc/mediatek/sched/	\
	-I$(srctree)/drivers/misc/mediatek/include/mt-plat/mt6771/include/mach/

ifdef CONFIG_ARCH_QCOM
hypnus_ops-y := hypnus_msm_op.o
else
hypnus_ops-y := hypnus_mtk_op.o
endif

oppo_hypnus-objs := hypnus_main.o hypnus_dev.o hypnus_sysfs.o
oppo_hypnus-objs += $(hypnus_ops-y)

obj-$(CONFIG_OPPO_HYPNUS) += oppo_hypnus.o
