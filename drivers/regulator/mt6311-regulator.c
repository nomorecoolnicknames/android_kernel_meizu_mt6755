/*
 * Copyright (c) 2015 MediaTek Inc.
 * Author: Henry Chen <henryc.chen@mediatek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/mt6311.h>
#include <linux/slab.h>
#include "mt6311-regulator.h"

static const struct regmap_config mt6311_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = MT6311_FQMTR_CON4,
};

/* Default limits measured in millivolts and milliamps */
#define MT6311_MIN_UV		600000
#define MT6311_MAX_UV		1393750
#define MT6311_STEP_UV		6250

static const struct regulator_linear_range buck_volt_range[] = {
	REGULATOR_LINEAR_RANGE(MT6311_MIN_UV, 0, 0x7f, MT6311_STEP_UV),
};

/* This bind is read-only: VPROC is the CPU rail, and a wrong write can
 * brown-out or over-volt the cores, so every op that writes the chip is
 * refused until forge_vproc_write=1 (default 0; no kernel consumer can
 * change it as a side effect). Readback ops stay open. The live setpoint
 * may be CON13 (stock writes VOSEL_ON), while this driver's vsel_reg is
 * CON12, another reason the set path stays off. */
static int forge_vproc_write;
module_param(forge_vproc_write, int, 0644);

static int forge_guard(const char *op)
{
	if (forge_vproc_write)
		return 0;
	pr_err("[FORGE_VPROC] %s BLOCKED (stage-1 read-only; mt6311_regulator.forge_vproc_write=1 arms writes)\n",
	       op);
	return -EPERM;
}

static int mt6311_set_voltage_sel_guarded(struct regulator_dev *rdev,
					  unsigned int sel)
{
	int ret = forge_guard("set_voltage_sel");

	return ret ? ret : regulator_set_voltage_sel_regmap(rdev, sel);
}

static int mt6311_enable_guarded(struct regulator_dev *rdev)
{
	int ret = forge_guard("enable");

	return ret ? ret : regulator_enable_regmap(rdev);
}

static int mt6311_disable_guarded(struct regulator_dev *rdev)
{
	int ret = forge_guard("disable");

	return ret ? ret : regulator_disable_regmap(rdev);
}

static const struct regulator_ops mt6311_buck_ops = {
	.list_voltage = regulator_list_voltage_linear_range,
	.map_voltage = regulator_map_voltage_linear_range,
	.set_voltage_sel = mt6311_set_voltage_sel_guarded,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_time_sel = regulator_set_voltage_time_sel,
	.enable = mt6311_enable_guarded,
	.disable = mt6311_disable_guarded,
	.is_enabled = regulator_is_enabled_regmap,
};

static const struct regulator_ops mt6311_ldo_ops = {
	.enable = mt6311_enable_guarded,
	.disable = mt6311_disable_guarded,
	.is_enabled = regulator_is_enabled_regmap,
};

#define MT6311_BUCK(_id) \
{\
	.name = #_id,\
	.ops = &mt6311_buck_ops,\
	.of_match = of_match_ptr(#_id),\
	.regulators_node = of_match_ptr("regulators"),\
	.type = REGULATOR_VOLTAGE,\
	.id = MT6311_ID_##_id,\
	.n_voltages = (MT6311_MAX_UV - MT6311_MIN_UV) / MT6311_STEP_UV + 1,\
	.min_uV = MT6311_MIN_UV,\
	.uV_step = MT6311_STEP_UV,\
	.owner = THIS_MODULE,\
	.linear_ranges = buck_volt_range, \
	.n_linear_ranges = ARRAY_SIZE(buck_volt_range), \
	.enable_reg = MT6311_VDVFS11_CON9,\
	.enable_mask = MT6311_PMIC_VDVFS11_EN_MASK,\
	.vsel_reg = MT6311_VDVFS11_CON12,\
	.vsel_mask = MT6311_PMIC_VDVFS11_VOSEL_MASK,\
}

#define MT6311_LDO(_id) \
{\
	.name = #_id,\
	.ops = &mt6311_ldo_ops,\
	.of_match = of_match_ptr(#_id),\
	.regulators_node = of_match_ptr("regulators"),\
	.type = REGULATOR_VOLTAGE,\
	.id = MT6311_ID_##_id,\
	.owner = THIS_MODULE,\
	.enable_reg = MT6311_LDO_CON3,\
	.enable_mask = MT6311_PMIC_RG_VBIASN_EN_MASK,\
}

static const struct regulator_desc mt6311_regulators[] = {
	MT6311_BUCK(VDVFS),
	MT6311_LDO(VBIASN),
};

/*
 * I2C driver interface functions
 */
static int mt6311_i2c_probe(struct i2c_client *i2c,
		const struct i2c_device_id *id)
{
	struct regulator_config config = { };
	struct regulator_dev *rdev;
	struct regmap *regmap;
	int i, ret;
	unsigned int data;

	regmap = devm_regmap_init_i2c(i2c, &mt6311_regmap_config);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		dev_err(&i2c->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	ret = regmap_read(regmap, MT6311_SWCID, &data);
	if (ret < 0) {
		dev_err(&i2c->dev, "Failed to read DEVICE_ID reg: %d\n", ret);
		return ret;
	}

	switch (data) {
	case MT6311_E1_CID_CODE:
	case MT6311_E2_CID_CODE:
	case MT6311_E3_CID_CODE:
		break;
	default:
		dev_err(&i2c->dev, "Unsupported device id = 0x%x.\n", data);
		return -ENODEV;
	}

	for (i = 0; i < MT6311_MAX_REGULATORS; i++) {
		config.dev = &i2c->dev;
		config.regmap = regmap;

		rdev = devm_regulator_register(&i2c->dev,
			&mt6311_regulators[i], &config);
		if (IS_ERR(rdev)) {
			dev_err(&i2c->dev,
				"Failed to register MT6311 regulator\n");
			return PTR_ERR(rdev);
		}
	}

	/* One read-only dump of the VDVFS11 state at bind, so the boot log
	 * carries the live VPROC setpoint and the range the driver assumes
	 * (expected CON9 = 0x1d, CON10/11 = 0x88, CON12 = CON13 = 0x40 ->
	 * 1.000 V). Reads only. */
	{
		unsigned int c9 = 0, c10 = 0, c11 = 0, c12 = 0, c13 = 0;

		regmap_read(regmap, MT6311_VDVFS11_CON9, &c9);
		regmap_read(regmap, MT6311_VDVFS11_CON10, &c10);
		regmap_read(regmap, MT6311_VDVFS11_CON11, &c11);
		regmap_read(regmap, MT6311_VDVFS11_CON12, &c12);
		regmap_read(regmap, MT6311_VDVFS11_CON13, &c13);
		pr_err("[FORGE_VPROC] MT6311 bound: SWCID=0x%02x CON9=0x%02x CON10=0x%02x CON11=0x%02x CON12(VOSEL)=0x%02x->%duV CON13(VOSEL_ON)=0x%02x->%duV range=%d..%duV step=%duV writes=%s\n",
		       data, c9, c10, c11,
		       c12, MT6311_MIN_UV + (c12 & 0x7f) * MT6311_STEP_UV,
		       c13, MT6311_MIN_UV + (c13 & 0x7f) * MT6311_STEP_UV,
		       MT6311_MIN_UV, MT6311_MAX_UV, MT6311_STEP_UV,
		       forge_vproc_write ? "ARMED" : "BLOCKED");
	}

	return 0;
}

static const struct i2c_device_id mt6311_i2c_id[] = {
	{"mt6311", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, mt6311_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id mt6311_dt_ids[] = {
	{ .compatible = "mediatek,mt6311-regulator",
	  .data = &mt6311_i2c_id[0] },
	/* the board DTB names the external buck (i2c-3 @0x6b)
	 * "mediatek,ext_buck" */
	{ .compatible = "mediatek,ext_buck",
	  .data = &mt6311_i2c_id[0] },
	{},
};
MODULE_DEVICE_TABLE(of, mt6311_dt_ids);
#endif

static struct i2c_driver mt6311_regulator_driver = {
	.driver = {
		.name = "mt6311",
		.of_match_table = of_match_ptr(mt6311_dt_ids),
	},
	.probe = mt6311_i2c_probe,
	.id_table = mt6311_i2c_id,
};

module_i2c_driver(mt6311_regulator_driver);

MODULE_AUTHOR("Henry Chen <henryc.chen@mediatek.com>");
MODULE_DESCRIPTION("Regulator device driver for Mediatek MT6311");
MODULE_LICENSE("GPL v2");
