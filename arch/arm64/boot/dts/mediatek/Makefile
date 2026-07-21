dtb-$(CONFIG_ARCH_MEDIATEK) += mt6795-evb.dtb
dtb-$(CONFIG_ARCH_MEDIATEK) += mt8173-evb.dtb
# M3 Note unified kernel: both board DTBs; each gets appended to the same
# Image.gz by the build recipe (one boot image per board).
dtb-$(CONFIG_M3NOTE_UNIFIED) += wt6755_66_sz_l.dtb hq6755_66_b1a_l.dtb

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb
