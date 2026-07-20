ccflags-y += -I$(srctree)/drivers/misc/mediatek/pmic \
             -I$(srctree)/drivers/misc/mediatek/usb_c/inc

# Mediatek PMIC MT63XX specific files
obj-$(CONFIG_MTK_PMIC_CHIP_MT6353)          += pmic.o pmic_irq.o upmu_common.o pmic_regulator.o pmic_api.o pmic_api_buck.o pmic_api_ldo.o regulator_codegen.o pmic_auxadc.o pmic_chr_type_det.o pmic_initial_setting.o pmic_efuse.o pmic_throttling_dlpt.o pmic_pinctl.o pmic_init351.o
# mt6311.o is not built: it relies on the 3.18 MTK-extended i2c_client
# (->ext_flag / ->timing / I2C_*_FLAG), which the 4.4 i2c_client lacks. The
# three query functions it would export (is_mt6311_exist /
# is_mt6311_sw_ready / get_mt6311_i2c_ch_num) are stubbed in
# base/power/mt6755/mt_pmic_stub.c, so the ext-buck paths report "absent"
# and use the internal bucks.
# obj-$(CONFIG_MTK_PMIC_CHIP_MT6353) += mt6311.o
