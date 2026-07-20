/*
 * MT6311 external buck presence probe: read-only, off by default.
 *
 * MT6351 has no VPROC buck (upmu_hw.h defines VCORE/VGPU/VMODEM/VMD1/
 * VSRAM_MD/VS1/VS2/VPA/VSRAM_PROC/K only). The stock kernel drives the CPU
 * rail through the MT6311 external buck (mt6311_set_vdvfs11_vosel_on) and
 * only VSRAM_PROC through the PMIC, so CPU voltage scaling needs the
 * MT6311.
 *
 * It does two 1-byte I2C reads and prints them; nothing is written.
 *
 *   echo 1 > /sys/module/mt_extbuck_probe/parameters/forge_extbuck_probe
 *
 * Detection as in the stock mt6311_hw_component_detect(): chip_id =
 * (CID << 8) | SWCID, valid = 0x0110 / 0x0120 / 0x0130.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/string.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/i2c.h>

#define FEB_TAG			"[FORGE_EXTBUCK] "
#define FEB_DT_COMPATIBLE	"mediatek,ext_buck"
#define FEB_REG_CID		0x00
#define FEB_REG_SWCID		0x01
#define FEB_CID_E1		0x0110
#define FEB_CID_E2		0x0120
#define FEB_CID_E3		0x0130

/* One byte out (register index), one byte back. No write anywhere. */
static int forge_extbuck_read(struct i2c_adapter *adap, u16 addr,
			      u8 reg, u8 *val)
{
	struct i2c_msg msgs[2];
	u8 wbuf = reg;
	int ret;

	/* Always zero a hand-built i2c_msg: the MTK ext_flag/timing fields
	 * must not be stack garbage. */
	memset(msgs, 0, sizeof(msgs));

	msgs[0].addr = addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &wbuf;

	msgs[1].addr = addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = val;

	ret = i2c_transfer(adap, msgs, 2);
	if (ret == 2)
		return 0;

	return (ret < 0) ? ret : -EIO;
}

static void forge_extbuck_run(void)
{
	struct device_node *np;
	struct device_node *parent;
	struct i2c_adapter *adap;
	unsigned int chip_id;
	u32 addr = 0;
	u8 cid = 0;
	u8 swcid = 0;
	int ret;

	np = of_find_compatible_node(NULL, NULL, FEB_DT_COMPATIBLE);
	if (!np) {
		pr_err(FEB_TAG "no DT node compatible=\"%s\" => nothing to probe\n",
		       FEB_DT_COMPATIBLE);
		return;
	}

	if (of_property_read_u32(np, "reg", &addr)) {
		pr_err(FEB_TAG "node %s has no reg property\n", np->full_name);
		of_node_put(np);
		return;
	}

	parent = of_get_parent(np);
	adap = parent ? of_find_i2c_adapter_by_node(parent) : NULL;
	if (!adap) {
		pr_err(FEB_TAG "node %s: parent is not a live i2c adapter\n",
		       np->full_name);
		of_node_put(parent);
		of_node_put(np);
		return;
	}

	pr_err(FEB_TAG "node=%s i2c-%d addr=0x%02x\n",
	       np->full_name, adap->nr, addr);

	ret = forge_extbuck_read(adap, (u16)addr, FEB_REG_CID, &cid);
	if (ret) {
		pr_err(FEB_TAG "CID read FAILED ret=%d => NO ACK at 0x%02x (buck ABSENT or bus/rail down)\n",
		       ret, addr);
		goto out;
	}

	ret = forge_extbuck_read(adap, (u16)addr, FEB_REG_SWCID, &swcid);
	if (ret) {
		pr_err(FEB_TAG "SWCID read FAILED ret=%d (CID=0x%02x did ACK)\n",
		       ret, cid);
		goto out;
	}

	chip_id = ((unsigned int)cid << 8) | swcid;

	if (chip_id == FEB_CID_E1 || chip_id == FEB_CID_E2 ||
	    chip_id == FEB_CID_E3)
		pr_err(FEB_TAG "CID=0x%02x SWCID=0x%02x chip_id=0x%04x => MT6311 PRESENT\n",
		       cid, swcid, chip_id);
	else
		pr_err(FEB_TAG "CID=0x%02x SWCID=0x%02x chip_id=0x%04x not in {0110,0120,0130} => UNKNOWN DEVICE\n",
		       cid, swcid, chip_id);

out:
	i2c_put_adapter(adap);
	of_node_put(parent);
	of_node_put(np);
}

static int forge_extbuck_probe_set(const char *val,
				   const struct kernel_param *kp)
{
	int n = 0;

	if (kstrtoint(val, 0, &n))
		return -EINVAL;

	if (n == 1)
		forge_extbuck_run();
	else
		pr_err(FEB_TAG "op=%d ignored (write 1 to probe)\n", n);

	return 0;
}

static const struct kernel_param_ops forge_extbuck_probe_ops = {
	.set = forge_extbuck_probe_set,
};

module_param_cb(forge_extbuck_probe, &forge_extbuck_probe_ops, NULL, 0200);
MODULE_PARM_DESC(forge_extbuck_probe,
		 "write 1: read MT6311 CID/SWCID over i2c (read-only, no rail is touched)");
