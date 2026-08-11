// SPDX-License-Identifier: GPL-2.0
#include <linux/input-polldev.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/m3note_board.h>
#include <linux/platform_device.h>

/* Stock MT6351 TOPSTATUS and MT6755 keypad scan register offsets. */
#define MT6351_TOPSTATUS 0x0220
#define KP_MEM1 0x0004
#define KP_EN 0x0024
extern unsigned int pmic_read_interface(unsigned int reg, unsigned int *value,
				       unsigned int mask, unsigned int shift);
struct m681_keys {
	void __iomem *base;
	u32 codes[4];
	bool pressed[4];
};

/* Failed reads do not synthesize press or release events. */
static bool m681_keys_decode(unsigned int error, u32 status, u16 matrix,
			     bool matrix_enabled, bool pressed[4])
{
	if (error || status > 0xffff || !matrix_enabled)
		return false;
	pressed[0] = !(matrix & BIT(0));
	pressed[1] = !(matrix & BIT(1));
	pressed[2] = !(status & BIT(1));
	pressed[3] = !(status & BIT(2));
	return true;
}

static void m681_keys_poll(struct input_polled_dev *poll)
{
	struct m681_keys *keys = poll->private;
	bool pressed[4];
	u32 status = 0;
	unsigned int error;
	unsigned int i;
	error = pmic_read_interface(MT6351_TOPSTATUS, &status, 0xffff, 0);
	if (!m681_keys_decode(error, status, readw(keys->base + KP_MEM1),
			     readw(keys->base + KP_EN) & 1, pressed))
		return;
	for (i = 0; i < ARRAY_SIZE(pressed); i++) {
		if (keys->pressed[i] == pressed[i])
			continue;
		keys->pressed[i] = pressed[i];
		input_report_key(poll->input, keys->codes[i], pressed[i]);
	}
	input_sync(poll->input);
}

static int m681_keys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct m681_keys *keys;
	struct resource *res;
	struct input_polled_dev *poll;
	u32 map[72], count, status = 0;
	int error;
	u32 seen = 0;
	unsigned int i;

	/* M681 and L681 share the MT6351 TOPSTATUS PWRKEY/HOMEKEY bits and the
	 * two-cell kpd matrix, but each board's DCT assigns them differently:
	 * M681 (wt6755_66_sz_l): matrix VOLDOWN, VOLUP; PMIC POWER, HOME.
	 * L681 (hq6755_66_b1a_l): matrix VOLDOWN, HOME; PMIC POWER, VOLUP
	 * (kpd-sw-rstkey 115). Codes come from the DT; the board comes from the
	 * DT machine compatible (m3note_board_id()). */
	if (!m3note_board_is(M3NOTE_BOARD_M681) &&
	    !m3note_board_is(M3NOTE_BOARD_L681))
		return -ENODEV;
	keys = devm_kzalloc(dev, sizeof(*keys), GFP_KERNEL);
	if (!keys)
		return -ENOMEM;
	if (of_property_read_u32(np, "mediatek,kpd-hw-map-num", &count) ||
	    count != ARRAY_SIZE(map) ||
	    of_property_read_u32_array(np, "mediatek,kpd-hw-init-map", map, ARRAY_SIZE(map)) ||
	    of_property_read_u32(np, "mediatek,kpd-sw-pwrkey", &keys->codes[2]) ||
	    of_property_read_u32(np, "mediatek,kpd-sw-rstkey", &keys->codes[3]))
		return -EINVAL;
	keys->codes[0] = map[0];
	keys->codes[1] = map[1];
	/* Refuse defaults: the four codes must be exactly the four keys of the
	 * board, each once. */
	for (i = 0; i < ARRAY_SIZE(keys->codes); i++) {
		u32 bit = keys->codes[i] == KEY_VOLUMEDOWN ? 1 :
			  keys->codes[i] == KEY_VOLUMEUP ? 2 :
			  keys->codes[i] == KEY_POWER ? 4 :
			  keys->codes[i] == KEY_HOME ? 8 : 0;

		if (!bit || (seen & bit))
			return -EINVAL;
		seen |= bit;
	}
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res || res->start != 0x10010000 ||
	    resource_size(res) < KP_EN + sizeof(u16))
		return -EINVAL;
	keys->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(keys->base))
		return PTR_ERR(keys->base);
	error = pmic_read_interface(MT6351_TOPSTATUS, &status, 0xffff, 0);
	if (error || status > 0xffff)
		return -EPROBE_DEFER;
	if (!(readw(keys->base + KP_EN) & 1)) {
		dev_err(dev, "bootloader keypad scan disabled; refusing register writes\n");
		return -ENODEV;
	}
	poll = devm_input_allocate_polled_device(dev);
	if (!poll)
		return -ENOMEM;
	poll->private = keys;
	poll->poll = m681_keys_poll;
	poll->poll_interval = 20;
	/* The stock input name: Android picks /vendor/usr/keylayout/mtk-kpd.kl
	 * (HOME with WAKE) by it; any other name falls back to Generic.kl,
	 * where scan code 102 is MOVE_HOME, not HOME. */
	poll->input->name = "mtk-kpd";
	poll->input->phys = "m681-keys/input0";
	poll->input->id.bustype = BUS_HOST;
	input_set_capability(poll->input, EV_KEY, KEY_VOLUMEDOWN);
	input_set_capability(poll->input, EV_KEY, KEY_VOLUMEUP);
	input_set_capability(poll->input, EV_KEY, KEY_POWER);
	input_set_capability(poll->input, EV_KEY, KEY_HOME);
	error = input_register_polled_device(poll);
	if (error)
		return error;
	dev_info(dev, "physical keys registered (20ms, read-only, awake only)\n");
	return 0;
}
static const struct of_device_id m681_keys_match[] = {
	{ .compatible = "mediatek,mt6755-keypad" },
	{ }
};
MODULE_DEVICE_TABLE(of, m681_keys_match);
static struct platform_driver m681_keys_driver = {
	.probe = m681_keys_probe,
	.driver = { .name = "m681-physical-keys", .of_match_table = m681_keys_match },
};
module_platform_driver(m681_keys_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("M681 read-only physical keypad input");
