#
# Makefile for the linux kernel.
#
subdir-ccflags-y += -Werror
ifdef CONFIG_MTPROF
ccflags-y += -Idrivers/misc/mediatek/mtprof/
endif
ifdef CONFIG_MTK_RAM_CONSOLE
ccflags-y += -Idrivers/misc/mediatek/include/mt-plat/
endif

obj-y                          := main.o version.o mounts.o
# Early boot stage markers that survive a WDT reset (SPM/DRAM breadcrumbs)
# and the forge_enable_cpuxgpt timer start.
obj-y                          += forge_m681_marker.o
# Boot banner printed at late_initcall_sync with live data
# (kernel/CPU/RAM/eMMC/DT model), to the console and pstore/ramoops.
obj-y                          += forge_m681_fetch.o
obj-y                          += noinitramfs.o
obj-$(CONFIG_BLK_DEV_INITRD)   += initramfs.o
obj-$(CONFIG_GENERIC_CALIBRATE_DELAY) += calibrate.o

ifneq ($(CONFIG_ARCH_INIT_TASK),y)
obj-y                          += init_task.o
endif

mounts-y			:= do_mounts.o
mounts-$(CONFIG_BLK_DEV_RAM)	+= do_mounts_rd.o
mounts-$(CONFIG_BLK_DEV_INITRD)	+= do_mounts_initrd.o
mounts-$(CONFIG_BLK_DEV_MD)	+= do_mounts_md.o

# dependencies on generated files need to be listed explicitly
$(obj)/version.o: include/generated/compile.h

# compile.h changes depending on hostname, generation number, etc,
# so we regenerate it always.
# mkcompile_h will make sure to only update the
# actual file if its content has changed.

       chk_compile.h = :
 quiet_chk_compile.h = echo '  CHK     $@'
silent_chk_compile.h = :
include/generated/compile.h: FORCE
	@$($(quiet)chk_compile.h)
	$(Q)$(CONFIG_SHELL) $(srctree)/scripts/mkcompile_h $@ \
	"$(UTS_MACHINE)" "$(CONFIG_SMP)" "$(CONFIG_PREEMPT)" "$(CC) $(KBUILD_CFLAGS)"
