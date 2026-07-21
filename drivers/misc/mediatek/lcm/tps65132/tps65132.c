/*
 * TPS65132 LCD bias (AVDD/AVEE) I2C provider for the l681 (hq6755_66_b1a_l)
 * panels, 4.4 device-tree port of the l681 3.10 lcm/tps65132 driver.
 *
 * 3.10 registered the client with i2c_register_board_info(0, 0x3e); on 4.4
 * the DrvGen cust.dtsi of hq6755_66_b1a_l.dws already emits
 * &i2c0 { i2c_lcd_bias@3e { compatible = "mediatek,i2c_lcd_bias"; }; },
 * so the driver binds by compatible (the same one the m681 panel's embedded
 * provider matches). Built only when "tps65132" is in CONFIG_CUSTOM_KERNEL_LCM.
 */
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>

#define I2C_ID_NAME "tps65132"

static struct i2c_client *tps65132_i2c_client;

static int tps65132_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	pr_info("tps65132: probe addr=0x%x\n", client->addr);
	tps65132_i2c_client = client;
	return 0;
}

static int tps65132_remove(struct i2c_client *client)
{
	if (tps65132_i2c_client == client)
		tps65132_i2c_client = NULL;
	return 0;
}

int tps65132_write_bytes(unsigned char addr, unsigned char value)
{
	struct i2c_client *client = tps65132_i2c_client;
	char write_data[2] = { addr, value };
	int ret;

	if (!client) {
		pr_err("tps65132: i2c client is not ready\n");
		return -ENODEV;
	}
	ret = i2c_master_send(client, write_data, 2);
	if (ret < 0)
		pr_err("tps65132: write 0x%02x=0x%02x failed %d\n", addr, value, ret);
	return ret;
}
EXPORT_SYMBOL(tps65132_write_bytes);

static const struct of_device_id tps65132_of_match[] = {
	{ .compatible = "mediatek,i2c_lcd_bias" },
	{ .compatible = "mediatek,I2C_LCD_BIAS" },
	{},
};

static const struct i2c_device_id tps65132_id[] = {
	{ I2C_ID_NAME, 0 },
	{}
};

static struct i2c_driver tps65132_iic_driver = {
	.id_table = tps65132_id,
	.probe = tps65132_probe,
	.remove = tps65132_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = I2C_ID_NAME,
		.of_match_table = tps65132_of_match,
	},
};

module_i2c_driver(tps65132_iic_driver);

MODULE_DESCRIPTION("TPS65132 LCD bias I2C provider (l681)");
MODULE_LICENSE("GPL");
