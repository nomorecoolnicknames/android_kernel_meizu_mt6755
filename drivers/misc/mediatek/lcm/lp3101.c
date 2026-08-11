#include <linux/i2c.h>
#include <linux/pinctrl/consumer.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/module.h>

static struct i2c_client *lp3101_i2c_client = NULL;
static struct pinctrl *lp3101_pinctrl = NULL;
static struct pinctrl_state *state_enn_low = NULL;
static struct pinctrl_state *state_enn_high = NULL;
static struct pinctrl_state *state_enp_low = NULL;
static struct pinctrl_state *state_enp_high = NULL;

int lp3101_write_bytes(unsigned char addr, unsigned char value)
{
	int ret;
	unsigned char buf[2];

	if (!lp3101_i2c_client) {
		pr_err("lp3101 write data fail 0 !!\n");
		return -1;
	}

	buf[0] = addr;
	buf[1] = value;
	ret = i2c_master_send(lp3101_i2c_client, buf, 2);
	if (ret < 0) {
		pr_err("lp3101 write data fail 1 !!\n");
	}
	return ret;
}
EXPORT_SYMBOL(lp3101_write_bytes);

int tps65132_write_bytes(unsigned char addr, unsigned char value)
{
	return lp3101_write_bytes(addr, value);
}
EXPORT_SYMBOL(tps65132_write_bytes);

void lcd_bais_enn_enable(char enable)
{
	int ret;
	if (!lp3101_pinctrl || IS_ERR(lp3101_pinctrl)) {
		pr_err("[lp3101] pinctrl is not initialized for ENN enable!\n");
		return;
	}
	if (enable) {
		if (state_enn_high && !IS_ERR(state_enn_high)) {
			ret = pinctrl_select_state(lp3101_pinctrl, state_enn_high);
			if (ret < 0) {
				pr_err("[name:lp3101&][lp3101]Can not find lcd_bias_enn_high, set fail!!!\n");
			} else {
				pr_debug("[name:lp3101&][lp3101]enn high\n");
			}
		} else {
			pr_err("[name:lp3101&][lp3101]Can not find lcd_bias_enn_high, set fail!!!\n");
		}
	} else {
		if (state_enn_low && !IS_ERR(state_enn_low)) {
			ret = pinctrl_select_state(lp3101_pinctrl, state_enn_low);
			if (ret < 0) {
				pr_err("[name:lp3101&][lp3101]Can not find lcd_bias_enn_low, set fail!!!\n");
			} else {
				pr_debug("[name:lp3101&][lp3101]enn low\n");
			}
		} else {
			pr_err("[name:lp3101&][lp3101]Can not find lcd_bias_enn_low, set fail!!!\n");
		}
	}
}
EXPORT_SYMBOL(lcd_bais_enn_enable);

void lcd_bais_enp_enable(char enable)
{
	int ret;
	if (!lp3101_pinctrl || IS_ERR(lp3101_pinctrl)) {
		pr_err("[lp3101] pinctrl is not initialized for ENP enable!\n");
		return;
	}
	if (enable) {
		if (state_enp_high && !IS_ERR(state_enp_high)) {
			ret = pinctrl_select_state(lp3101_pinctrl, state_enp_high);
			if (ret < 0) {
				pr_err("[name:lp3101&][lp3101]Can not find lcd_bias_enp_high, set fail!!!\n");
			} else {
				pr_debug("[name:lp3101&][lp3101]enp high\n");
			}
		} else {
			pr_err("[name:lp3101&][lp3101]Can not find lcd_bias_enp_high, set fail!!!\n");
		}
	} else {
		if (state_enp_low && !IS_ERR(state_enp_low)) {
			ret = pinctrl_select_state(lp3101_pinctrl, state_enp_low);
			if (ret < 0) {
				pr_err("[name:lp3101&][lp3101]Can not find lcd_bias_enp_low, set fail!!!\n");
			} else {
				pr_debug("[name:lp3101&][lp3101]enp low\n");
			}
		} else {
			pr_err("[name:lp3101&][lp3101]Can not find lcd_bias_enp_low, set fail!!!\n");
		}
	}
}
EXPORT_SYMBOL(lcd_bais_enp_enable);

static int lp3101_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	lp3101_i2c_client = client;
	return 0;
}

static int lp3101_remove(struct i2c_client *client)
{
	lp3101_i2c_client = NULL;
	return 0;
}

static const struct i2c_device_id lp3101_id[] = {
	{ "lp3101", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lp3101_id);

#ifdef CONFIG_OF
static const struct of_device_id lp3101_of_match[] = {
	{ .compatible = "mediatek,i2c_lcd_bias" },
	{ }
};
MODULE_DEVICE_TABLE(of, lp3101_of_match);
#endif

static struct i2c_driver lp3101_i2c_driver = {
	.driver = {
		.name = "lp3101",
#ifdef CONFIG_OF
		.of_match_table = lp3101_of_match,
#endif
	},
	.probe = lp3101_probe,
	.remove = lp3101_remove,
	.id_table = lp3101_id,
};

static int lp3101_pinctrl_probe(struct platform_device *pdev)
{
	struct pinctrl *p = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(p)) {
		pr_err("[lp3101] devm_pinctrl_get failed: %ld\n", PTR_ERR(p));
		lp3101_pinctrl = NULL;
		return PTR_ERR(p);
	}
	lp3101_pinctrl = p;

	state_enn_low = pinctrl_lookup_state(lp3101_pinctrl, "lcd_bias_enn_low");
	state_enn_high = pinctrl_lookup_state(lp3101_pinctrl, "lcd_bias_enn_high");
	state_enp_low = pinctrl_lookup_state(lp3101_pinctrl, "lcd_bias_enp_low");
	state_enp_high = pinctrl_lookup_state(lp3101_pinctrl, "lcd_bias_enp_high");

	return 0;
}

static int lp3101_pinctrl_remove(struct platform_device *pdev)
{
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id lp3101_pinctrl_ids[] = {
	/* The stock DTB node is "mediatek,lcd_bais_pinctrl" (sic, stock spells
	 * it "bais"), as matched by the 3.18 lp3101.c.
	 */
	{ .compatible = "mediatek,lcd_bais_pinctrl" },
	{ }
};
#endif

static struct platform_driver lp3101_pinctrl_driver = {
	.probe = lp3101_pinctrl_probe,
	.remove = lp3101_pinctrl_remove,
	.driver = {
		.name = "lp3101_pinctrl",
#ifdef CONFIG_OF
		.of_match_table = lp3101_pinctrl_ids,
#endif
	},
};

static int __init lp3101_i2c_init(void)
{
	int ret;
	ret = platform_driver_register(&lp3101_pinctrl_driver);
	if (ret) {
		pr_err("failed to register lp3101 pinctrl driver\n");
		return ret;
	}

	ret = i2c_add_driver(&lp3101_i2c_driver);
	if (ret < 0) {
		pr_err("failed to register lp3101 i2c driver\n");
		platform_driver_unregister(&lp3101_pinctrl_driver);
		return ret;
	}
	pr_info("[lp3101]lp3101_i2c_init success\n");
	return 0;
}

static void __exit lp3101_i2c_exit(void)
{
	i2c_del_driver(&lp3101_i2c_driver);
	platform_driver_unregister(&lp3101_pinctrl_driver);
}

module_init(lp3101_i2c_init);
module_exit(lp3101_i2c_exit);

MODULE_AUTHOR("Antigravity");
MODULE_DESCRIPTION("Meizu M5c LP3101 Bias Driver");
MODULE_LICENSE("GPL");
