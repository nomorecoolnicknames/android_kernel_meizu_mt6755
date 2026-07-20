/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

/*
 * MT6351 PMIC regulator (LDO/BUCK) table - 4.4 new-arch port.
 *
 * CONTRACT: in the 4.4 new-arch the regulator *framework* (probe / DT match /
 * regulator_register, i.e. mtk_regulator_init()) lives in the chip-agnostic
 * common layer (common/upmu_regulator.c).  That code consumes a set of
 * chip-provided codegen symbols declared in include/regulator_codegen.h:
 *     mt_ldos[]  mt_ldos_size  pmic_regulator_ldo_matches[]  ..._size
 *     mt_bucks[] mt_bucks_size pmic_regulator_buck_matches[] ..._size
 *     mtk_regulator_{enable,disable,is_enabled,get/set_voltage_sel,list_voltage}
 * This file supplies exactly those, populated with the AUTHENTIC MT6351 rail
 * data (selectors + voltage tables + EN/VOSEL register flags) ported from the
 * confirmed-bootable 3.10 stock pmic.c (lines ~894-1833) and cross-checked
 * against DCT PMIC_MT6351PMUMP.cmp (8 customer LDOs).
 *
 * SIMPLIFICATION vs MT6355 (documented gap): MT6355 ships ~2MB of generated
 * per-rail ops/accessors (pmic_api.c + regulator_codegen.c).  Here every rail
 * shares one generic mtk_regulator_ops that drives pmic_{get,set}_register_value
 * via the rail's en_reg/vol_reg PMU flag.  The per-rail callbacks
 * (en_cb/vol_cb/da_vol_cb) are left NULL: this means the DT
 * "regulator-default-on" fast path in common/upmu_regulator.c
 * (pmic_regulator_cust_dts_parser, which calls mt_ldos[i].en_cb) must not be
 * exercised until those are filled - section 5.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/of.h>

#include <mach/upmu_common.h>
#include <mach/upmu_hw.h>
#include <mach/mt_pm_ldo.h>

#include "include/pmic.h"

#ifndef PMIC_EN_VOL
#define PMIC_EN_VOL 9
#endif

/* local buck index enum (mt_pm_ldo.h has no MT6351 buck enum) */
enum {
	MT6351_BUCK_VCORE = 0,
	MT6351_BUCK_VGPU,
	MT6351_BUCK_VMODEM,
	MT6351_BUCK_VMD1,
	MT6351_BUCK_VSRAM_MD,
	MT6351_BUCK_VS1,
	MT6351_BUCK_VS2,
	MT6351_BUCK_VPA,
	MT6351_BUCK_VSRAM_PROC,
	MT6351_BUCK_COUNT_END,
};

/* forward decls for sysfs callbacks */
static ssize_t show_LDO_STATUS(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t store_LDO_STATUS(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size);
static ssize_t show_LDO_VOLTAGE(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t store_LDO_VOLTAGE(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t size);
static struct regulator_ops mtk_regulator_ops;

/* ============================ regulator ops ============================ *
 * NON-static: also satisfy the extern declarations in regulator_codegen.h.
 */
static unsigned char mtk_ldo_reverse_sel(const char *name, unsigned char sel)
{
	if ((strcmp(name, "va18") == 0) | (strcmp(name, "vtcxo24") == 0) |
	    (strcmp(name, "vtcxo28") == 0) | (strcmp(name, "vcn28") == 0) |
	    (strcmp(name, "vxo22") == 0) | (strcmp(name, "vbif28") == 0)) {
		if (sel == 0)
			return 3;
		else if (sel == 1)
			return 2;
		else if (sel == 2)
			return 1;
		else if (sel == 3)
			return 0;
	}
	return sel;
}

int mtk_regulator_enable(struct regulator_dev *rdev)
{
	const struct regulator_desc *rdesc = rdev->desc;
	struct mtk_regulator *mreg = container_of(rdesc, struct mtk_regulator, desc);

	if (mreg->en_reg != 0)
		pmic_set_register_value(mreg->en_reg, 1);
	return 0;
}

int mtk_regulator_disable(struct regulator_dev *rdev)
{
	const struct regulator_desc *rdesc = rdev->desc;
	struct mtk_regulator *mreg = container_of(rdesc, struct mtk_regulator, desc);

	if (rdev->use_count == 0)
		return -1;
	if (mreg->en_reg != 0)
		pmic_set_register_value(mreg->en_reg, 0);
	return 0;
}

int mtk_regulator_is_enabled(struct regulator_dev *rdev)
{
	const struct regulator_desc *rdesc = rdev->desc;
	struct mtk_regulator *mreg = container_of(rdesc, struct mtk_regulator, desc);

	return pmic_get_register_value(mreg->en_reg);
}

int mtk_regulator_get_voltage_sel(struct regulator_dev *rdev)
{
	const struct regulator_desc *rdesc = rdev->desc;
	struct mtk_regulator *mreg = container_of(rdesc, struct mtk_regulator, desc);
	unsigned char regVal = 0;

	if (mreg->desc.n_voltages != 1 && mreg->vol_reg != 0) {
		regVal = pmic_get_register_value(mreg->vol_reg);
		if (mreg->pvoltages != NULL)
			regVal = mtk_ldo_reverse_sel(mreg->desc.name, regVal);
	}
	return regVal;
}

int mtk_regulator_set_voltage_sel(struct regulator_dev *rdev, unsigned int selector)
{
	const struct regulator_desc *rdesc = rdev->desc;
	struct mtk_regulator *mreg = container_of(rdesc, struct mtk_regulator, desc);

	if (mreg->vol_reg != 0)
		pmic_set_register_value(mreg->vol_reg, selector);
	return 0;
}

int mtk_regulator_list_voltage(struct regulator_dev *rdev, unsigned int selector)
{
	const struct regulator_desc *rdesc = rdev->desc;
	struct mtk_regulator *mreg = container_of(rdesc, struct mtk_regulator, desc);
	const int *pVoltage;
	int voltage = 0;

	if (mreg->desc.n_voltages != 1) {
		if (mreg->vol_reg != 0 && mreg->pvoltages != NULL) {
			pVoltage = (const int *)mreg->pvoltages;
			selector = mtk_ldo_reverse_sel(mreg->desc.name, selector);
			voltage = pVoltage[selector];
		} else if (mreg->vol_reg != 0) {
			voltage = mreg->desc.min_uV + mreg->desc.uV_step * selector;
		}
	} else {
		pVoltage = (const int *)mreg->pvoltages;
		voltage = pVoltage[0];
	}
	return voltage;
}

static struct regulator_ops mtk_regulator_ops = {
	.enable = mtk_regulator_enable,
	.disable = mtk_regulator_disable,
	.is_enabled = mtk_regulator_is_enabled,
	.get_voltage_sel = mtk_regulator_get_voltage_sel,
	.set_voltage_sel = mtk_regulator_set_voltage_sel,
	.list_voltage = mtk_regulator_list_voltage,
};

/* sysfs (EM) callbacks */
static ssize_t show_LDO_STATUS(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mtk_regulator *mreg = container_of(attr, struct mtk_regulator, en_att);

	return sprintf(buf, "%u\n", pmic_get_register_value(mreg->en_reg));
}
static ssize_t store_LDO_STATUS(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	return size;
}
static ssize_t show_LDO_VOLTAGE(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mtk_regulator *mreg = container_of(attr, struct mtk_regulator, voltage_att);
	const int *pVoltage;
	unsigned short regVal;
	unsigned int ret_value = 0;

	if (mreg->desc.n_voltages != 1 && mreg->vol_reg != 0) {
		regVal = pmic_get_register_value(mreg->vol_reg);
		if (mreg->pvoltages != NULL) {
			pVoltage = (const int *)mreg->pvoltages;
			regVal = mtk_ldo_reverse_sel(mreg->desc.name, regVal);
			ret_value = pVoltage[regVal];
		} else {
			ret_value = mreg->desc.min_uV + mreg->desc.uV_step * regVal;
		}
	} else {
		pVoltage = (const int *)mreg->pvoltages;
		ret_value = pVoltage[0];
	}
	return sprintf(buf, "%u\n", ret_value / 1000);
}
static ssize_t store_LDO_VOLTAGE(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t size)
{
	return size;
}

/* codegen-compatible GEN macros (shared ops) */
#define MT6351_LDO(_name, en, vol, array, use, mode)	\
	{	\
		.desc = {	\
			.name = #_name,	\
			.n_voltages = ARRAY_SIZE(array),	\
			.ops = &mtk_regulator_ops,	\
			.type = REGULATOR_VOLTAGE,	\
		},	\
		.init_data = { .constraints = { .valid_ops_mask = (mode), }, },	\
		.en_att = __ATTR(_name##_status, 0664, show_LDO_STATUS, store_LDO_STATUS),	\
		.voltage_att = __ATTR(_name##_voltage, 0664, show_LDO_VOLTAGE, store_LDO_VOLTAGE),	\
		.pvoltages = (void *)(array),	\
		.en_reg = (PMU_FLAGS_LIST_ENUM)(en),	\
		.vol_reg = (PMU_FLAGS_LIST_ENUM)(vol),	\
		.isUsedable = (use),	\
	}

#define MT6351_BUCK(_name, en, vol, min, max, step)	\
	{	\
		.desc = {	\
			.name = #_name,	\
			.n_voltages = ((max) - (min)) / (step) + 1,	\
			.ops = &mtk_regulator_ops,	\
			.type = REGULATOR_VOLTAGE,	\
			.min_uV = (min),	\
			.uV_step = (step),	\
		},	\
		.init_data = { .constraints = { .valid_ops_mask = PMIC_EN_VOL, }, },	\
		.en_att = __ATTR(_name##_status, 0664, show_LDO_STATUS, store_LDO_STATUS),	\
		.voltage_att = __ATTR(_name##_voltage, 0664, show_LDO_VOLTAGE, store_LDO_VOLTAGE),	\
		.en_reg = (PMU_FLAGS_LIST_ENUM)(en),	\
		.vol_reg = (PMU_FLAGS_LIST_ENUM)(vol),	\
		.qi_en_reg = (PMU_FLAGS_LIST_ENUM)(en),	\
		.qi_vol_reg = (PMU_FLAGS_LIST_ENUM)(vol),	\
		.isUsedable = 1,	\
	}

/* ======================= MT6351 voltage tables ======================= */
static const int mt6351_VA18_voltages[] = { 1800000, 2200000, 2375000, 2800000, };
static const int mt6351_VCAMA_voltages[] = { 1500000, 1800000, 2500000, 2800000, };
static const int mt6351_VCN33_voltages[] = { 3300000, 3400000, 3500000, 3600000, };
static const int mt6351_VEFUSE_voltages[] = {
	1200000, 1300000, 1700000, 1800000, 1860000, 2760000, 3000000, 3100000,
};
static const int mt6351_VSIM1_voltages[] = {
	1200000, 1300000, 1700000, 1800000, 1860000, 2760000, 3000000, 3100000,
};
static const int mt6351_VEMC33_voltages[] = { 3000000, 3300000, };
static const int mt6351_VMCH_voltages[] = { 3000000, 3300000, };
static const int mt6351_VMC_voltages[] = {
	1200000, 1300000, 1500000, 1800000, 2000000, 2900000, 3000000, 3300000,
};
static const int mt6351_VCAM_AF_voltages[] = {
	1200000, 1300000, 1500000, 1800000, 2000000, 2800000, 3000000, 3300000,
};
static const int mt6351_VCAMD_voltages[] = {
	900000, 950000, 1000000, 1050000, 1100000, 1200000, 1210000,
};
static const int mt6351_VA10_voltages[] = {
	900000, 950000, 1000000, 1050000, 1200000, 1500000, 1800000,
};
static const int mt6351_VGP3_voltages[] = {
	1000000, 1050000, 1100000, 1220000, 1300000, 1500000, 1800000, 1810000,
};
static const int mt6351_VCAM_IO_voltages[] = {
	900000, 950000, 1000000, 1050000, 1200000, 1500000, 1800000,
};
static const int mt6351_1v8_voltages[] = { 1800000, };
static const int mt6351_2v8_voltages[] = { 2800000, };
static const int mt6351_3v3_voltages[] = { 3300000, };
static const int mt6351_rf18_voltages[] = {
	1000000, 1050000, 1100000, 1220000, 1300000, 1500000, 1800000, 1810000,
};

/* ============== LDO table (codegen symbol mt_ldos, authentic data) ============== */
struct mtk_regulator mt_ldos[] = {
	MT6351_LDO(va18, PMIC_RG_VA18_EN, PMIC_RG_VA18_VOSEL, mt6351_VA18_voltages, 1, PMIC_EN_VOL),
	MT6351_LDO(vtcxo24, PMIC_RG_VTCXO24_EN, PMIC_RG_VTCXO24_VOSEL, mt6351_VA18_voltages, 1, PMIC_EN_VOL),
	MT6351_LDO(vtcxo28, PMIC_RG_VTCXO28_EN, PMIC_RG_VTCXO28_VOSEL, mt6351_VA18_voltages, 1, PMIC_EN_VOL),
	MT6351_LDO(vcn28, PMIC_RG_VCN28_EN, PMIC_RG_VCN28_VOSEL, mt6351_VA18_voltages, 1, PMIC_EN_VOL),
	MT6351_LDO(vcama, PMIC_RG_VCAMA_EN, PMIC_RG_VCAMA_VOSEL, mt6351_VCAMA_voltages, 1, PMIC_EN_VOL),
	MT6351_LDO(vusb33, PMIC_RG_VUSB33_EN, NULL, mt6351_3v3_voltages, 1, PMIC_EN),
	MT6351_LDO(vsim1, PMIC_RG_VSIM1_EN, PMIC_RG_VSIM1_VOSEL, mt6351_VSIM1_voltages, 1, PMIC_EN_VOL),
	MT6351_LDO(vsim2, PMIC_RG_VSIM2_EN, PMIC_RG_VSIM2_VOSEL, mt6351_VSIM1_voltages, 1, PMIC_EN_VOL),
	MT6351_LDO(vemc, PMIC_RG_VEMC_EN, PMIC_RG_VEMC_VOSEL, mt6351_VEMC33_voltages, 1, PMIC_EN_VOL),
	MT6351_LDO(vmch, PMIC_RG_VMCH_EN, PMIC_RG_VMCH_VOSEL, mt6351_VMCH_voltages, 1, PMIC_EN_VOL),
	MT6351_LDO(vio28, PMIC_RG_VIO28_EN, NULL, mt6351_2v8_voltages, 1, PMIC_EN),
	MT6351_LDO(vibr, PMIC_RG_VIBR_EN, PMIC_RG_VIBR_VOSEL, mt6351_VCAM_AF_voltages, 1, PMIC_EN_VOL),
	MT6351_LDO(vcamd, PMIC_RG_VCAMD_EN, PMIC_RG_VCAMD_VOSEL, mt6351_VCAMD_voltages, 1, PMIC_EN_VOL),
	MT6351_LDO(vrf18, PMIC_RG_VRF18_EN, PMIC_RG_VRF18_VOSEL, mt6351_rf18_voltages, 1, PMIC_EN_VOL),
	MT6351_LDO(vio18, PMIC_RG_VIO18_EN, NULL, mt6351_1v8_voltages, 1, PMIC_EN),
	MT6351_LDO(vcn18, PMIC_RG_VCN18_EN, PMIC_RG_VCN18_VOSEL, mt6351_VCAM_IO_voltages, 1, PMIC_EN_VOL),
	MT6351_LDO(vcamio, PMIC_RG_VCAMIO_EN, PMIC_RG_VCAMIO_VOSEL, mt6351_VCAM_IO_voltages, 1, PMIC_EN_VOL),
	MT6351_LDO(vsram_proc, PMIC_RG_VSRAM_PROC_EN, PMIC_BUCK_VSRAM_PROC_VOSEL, mt6351_2v8_voltages, 1, PMIC_EN_VOL),
	MT6351_LDO(vxo22, PMIC_RG_VXO22_EN, PMIC_RG_VXO22_VOSEL, mt6351_VA18_voltages, 1, PMIC_EN_VOL),
	MT6351_LDO(vrf12, PMIC_RG_VRF12_EN, PMIC_RG_VRF12_VOSEL, mt6351_VCAM_IO_voltages, 1, PMIC_EN_VOL),
	MT6351_LDO(va10, PMIC_RG_VA10_EN, PMIC_RG_VA10_VOSEL, mt6351_VA10_voltages, 1, PMIC_EN_VOL),
	MT6351_LDO(vdram, PMIC_RG_VDRAM_EN, PMIC_RG_VDRAM_VOSEL, mt6351_VCAMD_voltages, 1, PMIC_EN_VOL),
	MT6351_LDO(vmipi, PMIC_RG_VMIPI_EN, PMIC_RG_VMIPI_VOSEL, mt6351_VA10_voltages, 1, PMIC_EN_VOL),
	MT6351_LDO(vgp3, PMIC_RG_VGP3_EN, PMIC_RG_VGP3_VOSEL, mt6351_VGP3_voltages, 1, PMIC_EN_VOL),
	MT6351_LDO(vbif28, PMIC_RG_VBIF28_EN, PMIC_RG_VBIF28_VOSEL, mt6351_VA18_voltages, 1, PMIC_EN_VOL),
	MT6351_LDO(vefuse, PMIC_RG_VEFUSE_EN, PMIC_RG_VEFUSE_VOSEL, mt6351_VEFUSE_voltages, 1, PMIC_EN_VOL),
	MT6351_LDO(vcn33_bt, PMIC_RG_VCN33_EN_BT, PMIC_RG_VCN33_VOSEL, mt6351_VCN33_voltages, 1, PMIC_EN_VOL),
	MT6351_LDO(vcn33_wifi, PMIC_RG_VCN33_EN_WIFI, PMIC_RG_VCN33_VOSEL, mt6351_VCN33_voltages, 1, PMIC_EN_VOL),
	MT6351_LDO(vldo28, PMIC_RG_VLDO28_EN_0, NULL, mt6351_2v8_voltages, 1, PMIC_EN),
	MT6351_LDO(vmc, PMIC_RG_VMC_EN, PMIC_RG_VMC_VOSEL, mt6351_VMC_voltages, 1, PMIC_EN_VOL),
	MT6351_LDO(vldo28_0, PMIC_RG_VLDO28_EN_0, NULL, mt6351_2v8_voltages, 1, PMIC_EN),
	MT6351_LDO(vldo28_1, PMIC_RG_VLDO28_EN_1, NULL, mt6351_2v8_voltages, 1, PMIC_EN),
};
int mt_ldos_size = ARRAY_SIZE(mt_ldos);

struct mtk_regulator mt_bucks[] = {
	MT6351_BUCK(vcore, PMIC_BUCK_VCORE_EN, PMIC_BUCK_VCORE_VOSEL, 600000, 1393750, 6250),
	MT6351_BUCK(vgpu, PMIC_BUCK_VGPU_EN, PMIC_BUCK_VGPU_VOSEL, 600000, 1393750, 6250),
	MT6351_BUCK(vmodem, PMIC_BUCK_VMODEM_EN, PMIC_BUCK_VMODEM_VOSEL, 600000, 1393750, 6250),
	MT6351_BUCK(vmd1, PMIC_BUCK_VMD1_EN, PMIC_BUCK_VMD1_VOSEL, 600000, 1393750, 6250),
	MT6351_BUCK(vsram_md, PMIC_BUCK_VSRAM_MD_EN, PMIC_BUCK_VSRAM_MD_VOSEL, 600000, 1393750, 6250),
	MT6351_BUCK(vs1, PMIC_BUCK_VS1_EN, PMIC_BUCK_VS1_VOSEL, 600000, 1393750, 6250),
	MT6351_BUCK(vs2, PMIC_BUCK_VS2_EN, PMIC_BUCK_VS2_VOSEL, 600000, 1393750, 6250),
	MT6351_BUCK(vpa, PMIC_BUCK_VPA_EN, PMIC_BUCK_VPA_VOSEL, 600000, 1393750, 6250),
	MT6351_BUCK(vsram_proc, PMIC_BUCK_VSRAM_PROC_EN, PMIC_BUCK_VSRAM_PROC_VOSEL, 600000, 1393750, 6250),
};
int mt_bucks_size = ARRAY_SIZE(mt_bucks);

/* ======================= of_regulator_match tables ======================= */
#define LDO_OF_MATCH(_dtname, _id)	\
	{ .name = #_dtname, .driver_data = &mt_ldos[MT6351_POWER_LDO_##_id], }
#define BUCK_OF_MATCH(_dtname, _id)	\
	{ .name = #_dtname, .driver_data = &mt_bucks[MT6351_BUCK_##_id], }

struct of_regulator_match pmic_regulator_ldo_matches[] = {
	LDO_OF_MATCH(ldo_va18, VA18),
	LDO_OF_MATCH(ldo_vtcxo24, VTCXO24),
	LDO_OF_MATCH(ldo_vtcxo28, VTCXO28),
	LDO_OF_MATCH(ldo_vcn28, VCN28),
	LDO_OF_MATCH(ldo_vcama, VCAMA),
	LDO_OF_MATCH(ldo_vusb33, VUSB33),
	LDO_OF_MATCH(ldo_vsim1, VSIM1),
	LDO_OF_MATCH(ldo_vsim2, VSIM2),
	LDO_OF_MATCH(ldo_vemc, VEMC),
	LDO_OF_MATCH(ldo_vmch, VMCH),
	LDO_OF_MATCH(ldo_vio28, VIO28),
	LDO_OF_MATCH(ldo_vibr, VIBR),
	LDO_OF_MATCH(ldo_vcamd, VCAMD),
	LDO_OF_MATCH(ldo_vrf18, VRF18),
	LDO_OF_MATCH(ldo_vio18, VIO18),
	LDO_OF_MATCH(ldo_vcn18, VCN18),
	LDO_OF_MATCH(ldo_vcamio, VCAMIO),
	LDO_OF_MATCH(ldo_vsram_proc, VSRAM_PROC),
	LDO_OF_MATCH(ldo_vxo22, VXO22),
	LDO_OF_MATCH(ldo_vrf12, VRF12),
	LDO_OF_MATCH(ldo_va10, VA10),
	LDO_OF_MATCH(ldo_vdram, VDRAM),
	LDO_OF_MATCH(ldo_vmipi, VMIPI),
	LDO_OF_MATCH(ldo_vgp3, VGP3),
	LDO_OF_MATCH(ldo_vbif28, VBIF28),
	LDO_OF_MATCH(ldo_vefuse, VEFUSE),
	LDO_OF_MATCH(ldo_vcn33_bt, VCN33_BT),
	LDO_OF_MATCH(ldo_vcn33_wifi, VCN33_WIFI),
	LDO_OF_MATCH(ldo_vldo28, VLDO28),
	LDO_OF_MATCH(ldo_vmc, VMC),
	LDO_OF_MATCH(ldo_vldo28_0, VLDO28_0),
	LDO_OF_MATCH(ldo_vldo28_1, VLDO28_1),
};
int pmic_regulator_ldo_matches_size = ARRAY_SIZE(pmic_regulator_ldo_matches);

struct of_regulator_match pmic_regulator_buck_matches[] = {
	BUCK_OF_MATCH(buck_vcore, VCORE),
	BUCK_OF_MATCH(buck_vgpu, VGPU),
	BUCK_OF_MATCH(buck_vmodem, VMODEM),
	BUCK_OF_MATCH(buck_vmd1, VMD1),
	BUCK_OF_MATCH(buck_vsram_md, VSRAM_MD),
	BUCK_OF_MATCH(buck_vs1, VS1),
	BUCK_OF_MATCH(buck_vs2, VS2),
	BUCK_OF_MATCH(buck_vpa, VPA),
	BUCK_OF_MATCH(buck_vsram_proc, VSRAM_PROC),
};
int pmic_regulator_buck_matches_size = ARRAY_SIZE(pmic_regulator_buck_matches);
