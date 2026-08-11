dtb-$(CONFIG_ARCH_MEDIATEK) += mt6755-evb.dtb
dtb-$(CONFIG_ARCH_MEDIATEK) += mt6795-evb.dtb
dtb-$(CONFIG_ARCH_MEDIATEK) += mt8173-evb.dtb
dtb-$(CONFIG_ARCH_MEDIATEK) += auto2712m1v1-ivi-boot.dtb
dtb-$(CONFIG_ARCH_MEDIATEK) += auto2712m1v1-ivi_agl-vp1-boot.dtb
dtb-$(CONFIG_ARCH_MEDIATEK) += auto2712m1v1-initramfs.dtb
dtb-$(CONFIG_ARCH_MEDIATEK) += auto2712m1v1-ivi-nand-boot.dtb
dtb-$(CONFIG_MACH_MT6735M) += k37mv1_bsp_k49.dtb
dtb-$(CONFIG_MACH_MT6755) += wt6755_66_sz_l.dtb
# M3 Note international (l681): second DTB for the same Image.gz
dtb-$(CONFIG_MACH_MT6755) += hq6755_66_b1a_l.dtb

ifeq ($(strip $(CONFIG_BUILD_ARM64_APPENDED_DTB_IMAGE)), y)
dtb-y += $(addsuffix .dtb, $(notdir $(subst $\",,$(CONFIG_BUILD_ARM64_APPENDED_DTB_IMAGE_NAMES))))
endif

ifeq ($(strip $(CONFIG_BUILD_ARM64_DTB_OVERLAY_IMAGE)), y)
dtb-y += $(addsuffix .dtb, $(notdir $(subst $\",,$(CONFIG_BUILD_ARM64_DTB_OVERLAY_IMAGE_NAMES))))
endif

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb
