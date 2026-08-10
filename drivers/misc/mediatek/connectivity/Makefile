# Connectivity combo driver
# In-tree port of the 3.18 (android_kernel_meizu_m5c) connectivity stack:
#   common/common_detect + common/conn_soc (wmt/stp core for SOC_CONSYS) + gps
# The 4.9 BSP moved connectivity to a standalone repo (connadp shims); that
# repo is not available, so the 3.18 in-tree driver is ported instead.
# The connadp shim objects (common/connectivity_build_in_adapter.o,
# common/wmt_build_in_adapter.o) are intentionally NOT linked: they duplicate
# symbols (e.g. gConEmiPhyBase) provided by the real driver.
ifneq ($(KERNELRELEASE),)
subdir-ccflags-y += -D MTK_WCN_REMOVE_KERNEL_MODULE
ifeq ($(CONFIG_ARM64), y)
subdir-ccflags-y += -D CONFIG_MTK_WCN_ARM64
endif

ifeq ($(CONFIG_MTK_CONN_LTE_IDC_SUPPORT),y)
    subdir-ccflags-y += -D WMT_IDC_SUPPORT=1
else
    subdir-ccflags-y += -D WMT_IDC_SUPPORT=0
endif
    subdir-ccflags-y += -D MTK_WCN_WMT_STP_EXP_SYMBOL_ABSTRACT

    # Include paths needed by the ported code. mt6735 uses the eccci1 CCCI:
    # port_ipc.h / ccci_ipc_task_ID.h / ccci_config.h live there.
    subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/eccci1/$(MTK_PLATFORM)
    subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/eccci1/
    subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/eemcs/
    subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/conn_md/include

    obj-y += common/
    obj-$(CONFIG_MTK_COMBO_WIFI) += wlan/
    obj-$(CONFIG_MTK_COMBO_GPS) += gps/
    obj-n := dummy.o

# Otherwise we were called directly from the command line;
# invoke the kernel build system.
else
    KERNELDIR ?= /lib/modules/$(shell uname -r)/build
    PWD  := $(shell pwd)
default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules
endif
