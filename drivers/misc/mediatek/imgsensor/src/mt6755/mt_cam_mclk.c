/*
 * ISP_MCLK1/2_EN provider for MT6755.
 *
 * Without an MT6755 cameraisp driver these symbols resolved to no-ops in
 * base/power/mt6755/mt_misc_stubs.c, and with no master clock every sensor
 * detect I2C transaction NAKs. This provider drives the real clock and
 * reports failures.
 *
 * Register map (stock 3.10):
 *   - ISP_MCLK1_EN: bit29 of ISP_ADDR+0x4200; MCLK2 = +0x4600. ISP_ADDR
 *     maps 0x15004000, so phys = 0x15008200/0x15008600 = SENINF base
 *     (0x15008000, the kd_camera_hw1 reg) + 0x0200/0x0600; the MT6757
 *     camera_isp.c uses the same SENINF0_BASE+0x0200 bit29 layout.
 *   - ISP_EnableClock: the clock/power chain stock holds while touching
 *     this MMIO, in this order:
 *       CG_SCP_SYS_DIS -> CG_DISP0_SMI_COMMON -> CG_SCP_SYS_ISP ->
 *       CG_IMAGE_CAM_SMI -> CG_IMAGE_CAM_CAM -> CG_IMAGE_SEN_TG ->
 *       CG_IMAGE_SEN_CAM -> CG_IMAGE_CAM_SV -> CG_IMAGE_LARB2_SMI
 *     The mt6755.dtsi "mediatek,ispsys" node carries this list under these
 *     clock-names, wired to clk-mt6755-pg (SCP_SYS_* MTCMOS-as-clock) and
 *     the imgsys CG provider.
 *
 * MMIO behind an off MTCMOS stalls the AXI bus, so the SENINF write happens
 * only after every clock in the chain reported success. Any failure is
 * logged ([FORGE_CAM] pr_err), unwinds and leaves the MMIO untouched.
 * Nothing here runs at boot: the callers sit behind the forge_cam_power
 * gate in kd_camera_hw.c (default 0).
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/m3note_board.h>

#define PFX "[FORGE_CAM] MCLK: "

/* camtg rate, from kd_sensorlist.c */
extern unsigned long forge_cam_camtg_rate(void);

/* Stock order - see header comment. Index constants for readability. */
static const char * const forge_mclk_clk_names[] = {
	"CG_SCP_SYS_DIS",
	"CG_DISP0_SMI_COMMON",
	"CG_SCP_SYS_ISP",
	"CG_IMAGE_CAM_SMI",
	"CG_IMAGE_CAM_CAM",
	"CG_IMAGE_SEN_TG",
	"CG_IMAGE_SEN_CAM",
	"CG_IMAGE_CAM_SV",
	"CG_IMAGE_LARB2_SMI",
};
#define FORGE_MCLK_NCLKS ARRAY_SIZE(forge_mclk_clk_names)

static struct clk *forge_mclk_clk[FORGE_MCLK_NCLKS];
static void __iomem *forge_seninf_base;	/* kd_camera_hw1 reg 0: 0x15008000 */
static bool forge_mclk_ready;		/* all clocks + iomap resolved */

/* TOPCKGEN CLK_CFG_2 readback and optional ungate. Stock clk-mt6755
 * registers camtg_sel with its pdn bit (gate = 7), so clk_prepare_enable()
 * in CAMERA_HW_Open clears PDN_CAMTG. The MUX_UPD table here carries
 * INVALID_MUX_GATE for every top mux, so that call is a no-op and bit7
 * keeps whatever LK/reset left; a set bit stops both CMMCLK pads. Default:
 * read only, the MCLK-ON marker prints CLK_CFG_2 and the bit.
 * forge_cam_camtg_ungate=1 also clears bit7 at MCLK-ON (as stock's gate
 * op). One-way: MCLK-OFF never re-gates. */
static void __iomem *forge_topck_base;	/* mediatek,mt6755-topckgen reg 0 */
#define FORGE_TOPCK_CLK_CFG_2	0x0060
#define FORGE_CAMTG_PDN_BIT	BIT(7)

static int forge_cam_camtg_ungate;
module_param(forge_cam_camtg_ungate, int, 0644);
MODULE_PARM_DESC(forge_cam_camtg_ungate, "m681: 1 = clear TOPCKGEN PDN_CAMTG (CLK_CFG_2 bit7) at MCLK-ON, replaying the gate op stock's camtg_sel carries; 0 (default) = print-only");

static DEFINE_MUTEX(forge_mclk_mutex);
static int forge_mclk_users[2];		/* per-MCLK refcount (MCLK1, MCLK2) */
static int forge_mclk_chain_users;	/* users of the clock/power chain */

/* SENINF offsets: +0x200 = MCLK1 (CMMCLK0 pad, main), +0x600 = MCLK2
 * (CMMCLK1 pad, sub). Bit29 = MCLK output enable (stock + mt6757). */
static const u32 forge_mclk_reg_off[2] = { 0x0200, 0x0600 };
#define FORGE_MCLK_EN_BIT	BIT(29)

/*
 * TG clock select. The 3.10 L681 kernel leaves SENINF TG1/TG2 PH_CNT
 * (SENINF+0x200/0x600) at 0x00000001 on every ISP release, and
 * ISP_MCLK1_EN only ORs bit29 into that; here bits[1:0] read 0 at MCLK-ON.
 * With this knob >= 0, MCLK-ON writes bits[1:0] = knob (3.10 value: 1)
 * before bit29. -1 (default) = leave as is.
 */
/*
 * Do what the camera HAL does before it powers a sensor (libcam.halsensor
 * SeninfDrvImp::setMclk1): SENINF+0x000 |= 0x400 | 0x800; TGx_SEN_CK
 * (+0x204): CLKRS[13:8] = 0, CLKFL[5:0] = (cnt + 1) >> 1, CLKCNT[21:16] =
 * cnt; TGx_PH_CNT (+0x200): bit2 = !(cnt & 1), bit28 = clkPol,
 * TGCLK_SEL[1:0] = 1, bit6 = padPclkInv, bit31 (PCEN) = pcEn; then
 * usleep(100). ISP_MCLK1_EN only adds bit29, so a kernel-only detect never
 * starts the TG phase counter (PCEN).
 * cnt is picked so MCLK = camtg / (cnt + 1) is closest to 24 MHz (48 MHz
 * root -> 1, 26 MHz root -> 0); clkPol and padPclkInv = 0. 1 (default) =
 * on for the known M3 Note boards.
 */
static int forge_cam_hal_mclk = 1;
module_param(forge_cam_hal_mclk, int, 0644);
MODULE_PARM_DESC(forge_cam_hal_mclk, "M3 Note: 1 = program SENINF TG like HAL setMclk (PCEN, TGCLK_SEL=1, SEN_CK, TOP 0xC00) at MCLK-ON; 0 = only bit29");

static int forge_cam_tgclk_sel = -1;
module_param(forge_cam_tgclk_sel, int, 0644);
MODULE_PARM_DESC(forge_cam_tgclk_sel, "l681: -1 = leave SENINF TGx_PH_CNT[1:0]; 0..3 = write it at MCLK-ON (3.10 l681 leaves 1)");

/* Read-only SENINF register dump (the 3.10 ISP RegDump seninf ranges). */
static int forge_cam_seninf_dump_set(const char *val, const struct kernel_param *kp)
{
	static const u32 ranges[][2] = {
		{ 0x000, 0x0C0 }, { 0x100, 0x1BC }, { 0x200, 0x208 },
		{ 0x300, 0x310 }, { 0x600, 0x608 },
	};
	int r;
	u32 off;

	if (!forge_seninf_base) {
		pr_err(PFX "SENINF not mapped (arm the camera first)\n");
		return 0;
	}
	if (!forge_mclk_chain_users)
		pr_err(PFX "SENINF dump with the ISP clock chain OFF - values may read 0\n");
	for (r = 0; r < ARRAY_SIZE(ranges); r++)
		for (off = ranges[r][0]; off <= ranges[r][1]; off += 16)
			pr_err(PFX "SENINF+0x%03x: %08x %08x %08x %08x\n", off,
			       readl(forge_seninf_base + off),
			       readl(forge_seninf_base + off + 4),
			       readl(forge_seninf_base + off + 8),
			       readl(forge_seninf_base + off + 12));
	return 0;
}

static const struct kernel_param_ops forge_cam_seninf_dump_ops = {
	.set = forge_cam_seninf_dump_set,
};
module_param_cb(forge_cam_seninf_dump, &forge_cam_seninf_dump_ops, NULL, 0200);
MODULE_PARM_DESC(forge_cam_seninf_dump, "l681: write anything to log SENINF 0x000-0x0C0/0x100-0x1BC/0x200-0x208/0x300-0x310/0x600-0x608 (read-only)");

/* Called from CAMERA_HW_probe (kd_sensorlist.c). Failure is not fatal to
 * the probe: /dev nodes must still appear so the gap stays observable. */
int forge_cam_mclk_init(struct platform_device *pdev)
{
	struct device_node *isp;
	int i, missing = 0;

	isp = of_find_compatible_node(NULL, NULL, "mediatek,ispsys");
	if (!isp) {
		pr_err(PFX "no mediatek,ispsys node - provider DISABLED\n");
		return -ENODEV;
	}
	for (i = 0; i < FORGE_MCLK_NCLKS; i++) {
		forge_mclk_clk[i] = of_clk_get_by_name(isp, forge_mclk_clk_names[i]);
		if (IS_ERR(forge_mclk_clk[i])) {
			pr_err(PFX "clk %s unresolved (%ld)\n",
			       forge_mclk_clk_names[i],
			       PTR_ERR(forge_mclk_clk[i]));
			missing++;
		}
	}
	of_node_put(isp);

	forge_seninf_base = of_iomap(pdev->dev.of_node, 0);
	if (!forge_seninf_base) {
		pr_err(PFX "of_iomap(kd_camera_hw1 reg0/SENINF) FAILED\n");
		missing++;
	}

	/* Not counted in `missing`: losing the CLK_CFG_2 print must not take
	 * the whole MCLK provider down with it. */
	{
		struct device_node *topck;

		topck = of_find_compatible_node(NULL, NULL,
						"mediatek,mt6755-topckgen");
		if (topck) {
			forge_topck_base = of_iomap(topck, 0);
			of_node_put(topck);
		}
		if (!forge_topck_base)
			pr_err(PFX "topckgen NOT mapped - CLK_CFG_2/PDN_CAMTG state will be UNREADABLE (its print is absent, not \"0\")\n");
	}

	if (missing) {
		pr_err(PFX "provider DISABLED (%d missing pieces) - every ISP_MCLK*_EN will refuse loudly; a sensor NAK in that state is NOT sensor identity evidence\n",
		       missing);
		return -ENODEV;
	}
	forge_mclk_ready = true;
	pr_info(PFX "provider ready (seninf mapped, %zu clks incl SCP_SYS_ISP)\n",
		FORGE_MCLK_NCLKS);
	return 0;
}

/* Enable the stock chain (power domain + CGs). Caller holds the mutex. */
static int forge_mclk_chain_get(void)
{
	int i, ret;

	if (forge_mclk_chain_users > 0) {
		forge_mclk_chain_users++;
		return 0;
	}
	for (i = 0; i < FORGE_MCLK_NCLKS; i++) {
		ret = clk_prepare_enable(forge_mclk_clk[i]);
		if (ret) {
			pr_err(PFX "clk_prepare_enable(%s) FAILED %d - unwinding, SENINF MMIO NOT touched\n",
			       forge_mclk_clk_names[i], ret);
			while (--i >= 0)
				clk_disable_unprepare(forge_mclk_clk[i]);
			return ret;
		}
	}
	forge_mclk_chain_users = 1;
	return 0;
}

/* Caller holds the mutex. */
static void forge_mclk_chain_put(void)
{
	int i;

	if (forge_mclk_chain_users <= 0) {
		pr_err(PFX "chain_put underflow\n");
		return;
	}
	if (--forge_mclk_chain_users == 0)
		for (i = FORGE_MCLK_NCLKS - 1; i >= 0; i--)
			clk_disable_unprepare(forge_mclk_clk[i]);
}

/* HAL SeninfDrvImp::setMclk1/2 on TG idx (body unchanged from the inline
 * version it replaces). Caller holds the mutex and the chain. */
static void forge_mclk_hal_tg(int idx)
{
	unsigned long rate = forge_cam_camtg_rate();
	u32 cnt = rate > 36000000 ? 1 : 0;
	u32 off = forge_mclk_reg_off[idx];
	u32 ck, ph;

	writel(readl(forge_seninf_base) | 0xC00, forge_seninf_base);
	ck = readl(forge_seninf_base + off + 4);
	ck &= ~(0x3Fu << 8 | 0x3Fu | 0x3Fu << 16);
	ck |= ((cnt + 1) >> 1) | (cnt << 16);
	writel(ck, forge_seninf_base + off + 4);
	ph = readl(forge_seninf_base + off);
	ph &= ~(BIT(2) | BIT(28) | BIT(6) | 0x3u);
	ph |= ((cnt & 1) ? 0 : BIT(2)) | 1u | BIT(31);
	writel(ph, forge_seninf_base + off);
	usleep_range(100, 150);
	pr_err(PFX "MCLK%d HAL-style TG: TOP=0x%08x PH_CNT=0x%08x SEN_CK=0x%08x (cnt=%u, camtg=%lu)\n",
	       idx + 1, readl(forge_seninf_base),
	       readl(forge_seninf_base + off),
	       readl(forge_seninf_base + off + 4), cnt, rate);
}

/*
 * Camera debug knobs. The three probes are 0 by default,
 * forge_cam_hal_seninf_init is 1. No PMIC writes: the PMIC is only read
 * (pmic_read_interface).
 *
 * Register map:
 *   PMIC MT6351 (mach/mt6351_regs.h):
 *     LDO_VCAMA_CON0 0x0A12 / CON1 0x0A14, VCAMD 0x0A40/0x0A42,
 *     VCAMIO 0x0A58/0x0A5A, VGP3 0x0A80/0x0A82. CON0: RG_EN b1, MODE_CTRL
 *     b2, ON_CTRL b3 (VCAMA/VGP3 only), DA_QI_MODE b8, DA_QI_STB b14,
 *     DA_QI_EN b15. CON1: DA_QI_OCFB_EN b10.
 *     OCSTATUS1 0x0212: VCAMA b11; OCSTATUS2 0x0214: VGP3 b4, VCAMIO b11,
 *     VCAMD b15.
 *   SPM 0x10006000: PWR_STATUS +0x180, PWR_STATUS_2ND +0x184, ISP = bit5.
 *   IMGSYS 0x15000000: CG_CON = sta +0x0, set +0x4, clr +0x8; 1 = gated.
 *     LARB2 b0, CAM_SMI b5, CAM_CAM b6, SEN_TG b7, SEN_CAM b8 (parent
 *     camtg_sel), CAM_SV b9.
 *   TOPCKGEN 0x10000000: CLK_CFG_2 +0x60, camtg_sel [1:0], PDN bit7; meter
 *     CLK_MISC_CFG_0 +0x104, CLK_MISC_CFG_1 +0x108, CLK_DBG_CFG +0x10C,
 *     CLK26CALI_0 +0x220, CLK26CALI_1 +0x224.
 *   GPIO 0x10005000: DIN = +0x200 + (pin / 32) * 0x10; GPIO109 +0x230
 *     bit13, GPIO161 +0x250 bit1. IES (IOCFG_3 0x10002600): GPIO109 bit10,
 *     GPIO161 bit9.
 *   SENINF 0x15008000 (window 0x1000): TOP +0x000, TG1 PH_CNT/SEN_CK
 *     +0x200/+0x204, TG2 +0x600/+0x604.
 */
extern unsigned int pmic_read_interface(unsigned int RegNum, unsigned int *val,
					unsigned int MASK, unsigned int SHIFT);
extern int mt_get_gpio_mode(unsigned long pin);
extern int mt_set_gpio_mode(unsigned long pin, unsigned long mode);
extern int mt_get_gpio_dir(unsigned long pin);
extern int mt_set_gpio_dir(unsigned long pin, unsigned long dir);
extern int mt_get_gpio_in(unsigned long pin);
extern int mt_get_gpio_ies(unsigned long pin);
extern int mt_get_gpio_pull_enable(unsigned long pin);
extern int mt_set_gpio_pull_enable(unsigned long pin, unsigned long enable);
extern int mt_get_gpio_pull_select(unsigned long pin);
extern int mt_set_gpio_pull_select(unsigned long pin, unsigned long select);

#define FORGE_GPIO_PHYS		0x10005000
#define FORGE_SPM_PHYS		0x10006000
#define FORGE_IMGSYS_PHYS	0x15000000
#define FORGE_PROBE_SAMPLES	4096

static int forge_cam_probe, forge_cam_fmeter;
/*
 * Default on. The kernel detect (SET_DRIVER/CHECK_IS_ALIVE) runs without
 * any HAL SENINF setup; without these bits the CMMCLK pads GPIO161/109 do
 * not toggle and the sensors NAK, although the meter reads 48 MHz, ISP is
 * powered, the CGs are open and PH_CNT is 0xa0000001. The camera HAL's
 * SeninfDrvImp::init (M681 and L681) does, on the 0x15008000/0x4000
 * mapping: +0x120/0x520/0x920/0xD20 |= bit31, +0x100/0x500/0x900 |= 1, and
 * enumerateSensor_Locked calls setMclk1 and setMclk2 with the same
 * arguments. MCLK-OFF leaves the bits, as stock does (SeninfDrvImp::uninit
 * makes no SENINF store at these offsets). The writes are ORs, so repeated
 * MCLK-ON is idempotent. 0 = without them.
 */
static int forge_cam_hal_seninf_init = 1;
module_param(forge_cam_hal_seninf_init, int, 0644);
MODULE_PARM_DESC(forge_cam_hal_seninf_init, "l681+m681 §5.43/§5.45: 1 (default) = at MCLK-ON replay HAL SeninfDrvImp::init (SENINF +0x100/0x500/0x900 |= 1, +0x120/0x520/0x920/0xD20 |= bit31) and start the other TG like enumerateSensor's setMclk2; never cleared at MCLK-OFF (stock uninit does not); 0 = s1 behaviour (rollback)");
static bool forge_cam_hal_twin[2];	/* TG started as the other one's twin */

/* As the HAL's SeninfDrvImp::init: +0x120/+0x520/+0x920/+0xD20 |=
 * 0x80000000, then +0x100/+0x500/+0x900 |= 1. Mutex and chain held. */
static void forge_cam_hal_seninf_bits(void)
{
	static const u16 csi[] = { 0x120, 0x520, 0x920, 0xD20 };
	static const u16 ctl[] = { 0x100, 0x500, 0x900 };
	int i;

	for (i = 0; i < ARRAY_SIZE(csi); i++)
		writel(readl(forge_seninf_base + csi[i]) | BIT(31),
		       forge_seninf_base + csi[i]);
	for (i = 0; i < ARRAY_SIZE(ctl); i++)
		writel(readl(forge_seninf_base + ctl[i]) | 1,
		       forge_seninf_base + ctl[i]);
	pr_err(PFX "HAL SeninfDrvImp::init bits set: +0x100=0x%08x +0x500=0x%08x +0x900=0x%08x +0x120=0x%08x +0x520=0x%08x +0x920=0x%08x +0xD20=0x%08x\n",
	       readl(forge_seninf_base + 0x100), readl(forge_seninf_base + 0x500),
	       readl(forge_seninf_base + 0x900), readl(forge_seninf_base + 0x120),
	       readl(forge_seninf_base + 0x520), readl(forge_seninf_base + 0x920),
	       readl(forge_seninf_base + 0xD20));
}

/*
 * Mutex held, chain held (forge_mclk_chain_users > 0), so IMGSYS and
 * SENINF are powered. Only reads.
 * DIN sampling: 4096 back-to-back readl pairs of the two DIN words, no
 * delay. One APB read takes far longer than half a 24 MHz period (21 ns),
 * and the bus clock is not derived from univpll_d26, so the sample phase
 * walks across the MCLK period: a toggling pad gives ~50 % ones and many
 * value changes between consecutive samples; a dead pad gives 0 (or 4096)
 * and 0 changes. The loop time is printed so the sample interval is known.
 */
static void forge_cam_probe_dump(const char *why)
{
	static const struct { u16 con0, con1; const char *name; } ldo[] = {
		{ 0x0A12, 0x0A14, "VCAMA" }, { 0x0A40, 0x0A42, "VCAMD" },
		{ 0x0A58, 0x0A5A, "VCAMIO" }, { 0x0A80, 0x0A82, "VGP3" },
	};
	void __iomem *spm, *img, *gpio;
	unsigned int v, c1, oc1 = 0, oc2 = 0;
	int i, ones161 = 0, ones109 = 0, tr161 = 0, tr109 = 0;
	u32 a, b, pa = 0, pb = 0;
	u64 t0, dt;

	pr_err(PFX "PROBE begin (%s) chain_users=%d mclk_users=%d/%d\n", why,
	       forge_mclk_chain_users, forge_mclk_users[0], forge_mclk_users[1]);
	for (i = 0; i < ARRAY_SIZE(ldo); i++) {
		v = c1 = 0;
		pmic_read_interface(ldo[i].con0, &v, 0xFFFF, 0);
		pmic_read_interface(ldo[i].con1, &c1, 0xFFFF, 0);
		pr_err(PFX "PROBE pmic %s CON0[0x%04x]=0x%04x RG_EN=%u MODE_CTRL=%u ON_CTRL(b3)=%u QI_MODE=%u STB=%u QI_EN=%u CON1[0x%04x]=0x%04x QI_OCFB=%u\n",
		       ldo[i].name, ldo[i].con0, v, (v >> 1) & 1, (v >> 2) & 1,
		       (v >> 3) & 1, (v >> 8) & 1, (v >> 14) & 1, (v >> 15) & 1,
		       ldo[i].con1, c1, (c1 >> 10) & 1);
	}
	pmic_read_interface(0x0212, &oc1, 0xFFFF, 0);
	pmic_read_interface(0x0214, &oc2, 0xFFFF, 0);
	pr_err(PFX "PROBE pmic OCSTATUS1=0x%04x OCSTATUS2=0x%04x OC: VCAMA=%u VCAMD=%u VCAMIO=%u VGP3=%u\n",
	       oc1, oc2, (oc1 >> 11) & 1, (oc2 >> 15) & 1, (oc2 >> 11) & 1,
	       (oc2 >> 4) & 1);

	spm = ioremap(FORGE_SPM_PHYS, 0x1000);
	if (spm) {
		u32 s = readl(spm + 0x180), s2 = readl(spm + 0x184);

		pr_err(PFX "PROBE PWR_STATUS=0x%08x 2ND=0x%08x ISP(bit5)=%u/%u\n",
		       s, s2, (s >> 5) & 1, (s2 >> 5) & 1);
		iounmap(spm);
	}
	img = ioremap(FORGE_IMGSYS_PHYS, 0x1000);
	if (img) {
		v = readl(img);
		pr_err(PFX "PROBE IMG_CG_CON=0x%08x (1=gated) LARB2=%u CAM_SMI=%u CAM_CAM=%u SEN_TG=%u SEN_CAM=%u CAM_SV=%u\n",
		       v, v & 1, (v >> 5) & 1, (v >> 6) & 1, (v >> 7) & 1,
		       (v >> 8) & 1, (v >> 9) & 1);
		iounmap(img);
	}
	if (forge_topck_base) {
		v = readl(forge_topck_base + FORGE_TOPCK_CLK_CFG_2);
		pr_err(PFX "PROBE CLK_CFG_2=0x%08x camtg_sel=%u PDN_CAMTG=%u\n",
		       v, v & 3, (v >> 7) & 1);
	}
	pr_err(PFX "PROBE SENINF TOP=0x%08x TG1 PH_CNT=0x%08x SEN_CK=0x%08x TG2 PH_CNT=0x%08x SEN_CK=0x%08x +0x100=0x%08x +0x120=0x%08x\n",
	       readl(forge_seninf_base), readl(forge_seninf_base + 0x200),
	       readl(forge_seninf_base + 0x204), readl(forge_seninf_base + 0x600),
	       readl(forge_seninf_base + 0x604), readl(forge_seninf_base + 0x100),
	       readl(forge_seninf_base + 0x120));
	pr_err(PFX "PROBE pads: GPIO161 mode=%d ies=%d pullen=%d pullsel=%d; GPIO109 mode=%d ies=%d pullen=%d pullsel=%d\n",
	       mt_get_gpio_mode(161 | 0x80000000), mt_get_gpio_ies(161 | 0x80000000),
	       mt_get_gpio_pull_enable(161 | 0x80000000),
	       mt_get_gpio_pull_select(161 | 0x80000000),
	       mt_get_gpio_mode(109 | 0x80000000), mt_get_gpio_ies(109 | 0x80000000),
	       mt_get_gpio_pull_enable(109 | 0x80000000),
	       mt_get_gpio_pull_select(109 | 0x80000000));

	gpio = ioremap(FORGE_GPIO_PHYS, 0x1000);
	if (!gpio) {
		pr_err(PFX "PROBE GPIO ioremap FAILED - no DIN sampling\n");
		return;
	}
	t0 = ktime_get_ns();
	for (i = 0; i < FORGE_PROBE_SAMPLES; i++) {
		a = (readl(gpio + 0x250) >> 1) & 1;	/* GPIO161 */
		b = (readl(gpio + 0x230) >> 13) & 1;	/* GPIO109 */
		ones161 += a;
		ones109 += b;
		if (i) {
			tr161 += a != pa;
			tr109 += b != pb;
		}
		pa = a;
		pb = b;
	}
	dt = ktime_get_ns() - t0;
	iounmap(gpio);
	pr_err(PFX "PROBE DIN %d samples in %llu ns: GPIO161 ones=%d changes=%d; GPIO109 ones=%d changes=%d (toggling MCLK: ones ~%d, changes >> 0)\n",
	       FORGE_PROBE_SAMPLES, dt, ones161, tr161, ones109, tr109,
	       FORGE_PROBE_SAMPLES / 2);
}

/*
 * E2. TOPCKGEN frequency meter on hf_fcamtg_ck, the stock procedure:
 * 3.10 mach/mt6755/mt_clkdbg.c:479-522 fmeter_freq() (built, Makefile:91),
 * identical in 4.4 base/power/mt6755/mt_clkdbg.c:490-533 (not built here,
 * Makefile:59). Ckgen id 7 = HF_FCAMTG_CK (3.10 mt_clkdbg.c:360, 4.4 :371).
 * AP registers only; CLK_DBG_CFG, CLK_MISC_CFG_0/1 and CLK26CALI_0 are saved
 * before and written back after (stock restores the first three). No other
 * user of the meter is live in this build: _mt_get_cpu_freq() has no caller
 * (dvfs_get_cpu_freq unused, mt_cpufreq.c:1391/1421 under #if 0).
 * Mutex held. Returns kHz, 0 if the meter did not finish.
 */
static u32 forge_cam_fmeter_camtg(void)
{
	void __iomem *t = forge_topck_base;
	u32 m0, m1, dbg, cali0, freq = 0;
	int i;

	m0 = readl(t + 0x104);
	m1 = readl(t + 0x108);
	dbg = readl(t + 0x10C);
	cali0 = readl(t + 0x220);
	writel(m0 & ~(0xFFu << 24), t + 0x104);		/* divider k1 = 0 (/1) */
	writel(readl(t + 0x10C) | BIT(0), t + 0x10C);	/* ckgen meter */
	writel((readl(t + 0x10C) & ~(0x3Fu << 8)) | (7u << 8), t + 0x10C);
	writel(readl(t + 0x220) | BIT(12), t + 0x220);	/* fmeter_en */
	writel(readl(t + 0x220) & ~BIT(4), t + 0x220);	/* start */
	writel(readl(t + 0x220) | BIT(12) | BIT(4), t + 0x220);
	for (i = 0; i < 100 && (readl(t + 0x220) & BIT(4)); i++)
		udelay(20);
	if (!(readl(t + 0x220) & BIT(4)))
		freq = (readl(t + 0x224) & 0xFFFF) * 26000 / 1024;
	writel(dbg, t + 0x10C);
	writel(m0, t + 0x104);
	writel(m1, t + 0x108);
	writel(cali0, t + 0x220);
	return freq;
}

static void forge_cam_fmeter_report(const char *why)
{
	if (!forge_topck_base) {
		pr_err(PFX "FMETER: topckgen not mapped\n");
		return;
	}
	pr_err(PFX "FMETER (%s) hf_fcamtg_ck=%u kHz (camtg_rate=%lu Hz; 48 MHz root -> ~48000, 26 MHz -> ~26000, 0 = dead or meter timeout)\n",
	       why, forge_cam_fmeter_camtg(), forge_cam_camtg_rate());
}

/* E1/E2 knobs: write 1 = one report. With the ISP chain held (HOLD or HAL
 * stream) it runs now; otherwise it is armed and runs once at the next
 * MCLK-ON, after bit29. Reads (plus the meter registers) only. */
static int forge_cam_oneshot_set(const char *val, const struct kernel_param *kp)
{
	int *knob = kp->arg;
	int ret = param_set_int(val, kp);

	if (ret || !*knob)
		return ret;
	if (!m3note_board_known_is(M3NOTE_BOARD_L681) || !forge_mclk_ready) {
		pr_err(PFX "%s refused: not l681 or provider unavailable\n",
		       kp->name);
		*knob = 0;
		return 0;
	}
	mutex_lock(&forge_mclk_mutex);
	if (forge_mclk_chain_users > 0) {
		if (knob == &forge_cam_probe)
			forge_cam_probe_dump("request");
		else
			forge_cam_fmeter_report("request");
		*knob = 0;
	} else {
		pr_err(PFX "%s armed: ISP chain OFF now, one report at the next MCLK-ON\n",
		       kp->name);
	}
	mutex_unlock(&forge_mclk_mutex);
	return 0;
}

static const struct kernel_param_ops forge_cam_oneshot_ops = {
	.set = forge_cam_oneshot_set,
	.get = param_get_int,
};
module_param_cb(forge_cam_probe, &forge_cam_oneshot_ops, &forge_cam_probe, 0644);
MODULE_PARM_DESC(forge_cam_probe, "l681 §5.43 E1: 1 = one read-only report (MT6351 cam LDO CON0/CON1/OC, SPM PWR_STATUS, IMG CG_CON, CLK_CFG_2, SENINF TG, 4096 DIN samples of GPIO161/109); now if the ISP chain is held, else at the next MCLK-ON");
module_param_cb(forge_cam_fmeter, &forge_cam_oneshot_ops, &forge_cam_fmeter, 0644);
MODULE_PARM_DESC(forge_cam_fmeter, "l681 §5.43 E2: 1 = one TOPCKGEN meter reading of hf_fcamtg_ck (id 7), meter registers restored; now if the ISP chain is held, else at the next MCLK-ON");

/*
 * E3. Is the input path of the MCLK pads readable at all? Only with no
 * MCLK user and the ISP chain off (mutex held throughout, so no MCLK-ON can
 * interleave). Per pad: save mode/dir/pull; DIR = in FIRST (b79032c4 left
 * the pads dir = out, and mode 0 with dir out would drive DOUT); pull-up,
 * mode 0 (GPIO), read DIN; pull-down, read DIN; then mode, pull select,
 * pull enable and dir back. Nothing is ever driven: the pad only sees its
 * own ~tens-of-kOhm pull, so with the camera rails off the worst case is a
 * few uA into the sensor's clamp diode (which may also pull a pull-up
 * reading to 0 - that reading is then "inconclusive", not "IES off").
 * The IES bit itself is read too (read-only).
 */
static int forge_cam_ies_test_set(const char *val, const struct kernel_param *kp)
{
	static const unsigned long pads[] = { 161, 109 };
	int i;

	if (!m3note_board_known_is(M3NOTE_BOARD_L681)) {
		pr_err(PFX "IES test refused: not l681\n");
		return 0;
	}
	mutex_lock(&forge_mclk_mutex);
	if (forge_mclk_users[0] || forge_mclk_users[1] || forge_mclk_chain_users) {
		pr_err(PFX "IES test refused: MCLK in use (users %d/%d chain %d) - release HOLD first\n",
		       forge_mclk_users[0], forge_mclk_users[1],
		       forge_mclk_chain_users);
		mutex_unlock(&forge_mclk_mutex);
		return 0;
	}
	for (i = 0; i < ARRAY_SIZE(pads); i++) {
		unsigned long pin = pads[i] | 0x80000000;
		int mode = mt_get_gpio_mode(pin), dir = mt_get_gpio_dir(pin);
		int pen = mt_get_gpio_pull_enable(pin);
		int psel = mt_get_gpio_pull_select(pin);
		int ies = mt_get_gpio_ies(pin), up, dn;

		if (mode < 0 || dir < 0 || pen < 0 || psel < 0) {
			pr_err(PFX "IES GPIO%lu: state unreadable (mode %d dir %d pullen %d pullsel %d) - skipped\n",
			       pads[i], mode, dir, pen, psel);
			continue;
		}
		mt_set_gpio_dir(pin, 0);		/* GPIO_DIR_IN */
		mt_set_gpio_pull_select(pin, 1);	/* GPIO_PULL_UP */
		mt_set_gpio_pull_enable(pin, 1);
		mt_set_gpio_mode(pin, 0);
		udelay(50);
		up = mt_get_gpio_in(pin);
		mt_set_gpio_pull_select(pin, 0);	/* GPIO_PULL_DOWN */
		udelay(50);
		dn = mt_get_gpio_in(pin);
		mt_set_gpio_mode(pin, mode);
		mt_set_gpio_pull_select(pin, psel);
		mt_set_gpio_pull_enable(pin, pen);
		mt_set_gpio_dir(pin, dir);
		pr_err(PFX "IES GPIO%lu: ies=%d DIN pull-up=%d pull-down=%d -> %s; restored mode=%d dir=%d pullen=%d pullsel=%d\n",
		       pads[i], ies, up, dn,
		       (up == 1 && dn == 0) ? "input path WORKS" :
		       (up == 0 && dn == 0) ? "reads 0 both ways (IES off, or clamped by the unpowered sensor)" :
		       "stuck/odd",
		       mt_get_gpio_mode(pin), mt_get_gpio_dir(pin),
		       mt_get_gpio_pull_enable(pin),
		       mt_get_gpio_pull_select(pin));
	}
	mutex_unlock(&forge_mclk_mutex);
	return 0;
}

static const struct kernel_param_ops forge_cam_ies_test_ops = {
	.set = forge_cam_ies_test_set,
};
module_param_cb(forge_cam_ies_test, &forge_cam_ies_test_ops, NULL, 0200);
MODULE_PARM_DESC(forge_cam_ies_test, "l681 §5.43 E3: write 1 with MCLK off = GPIO161/109 as GPIO input with pull-up then pull-down, read DIN, restore mode/pull/dir; never drives the pad");

static void forge_mclk_set(int idx, int en)
{
	u32 val;

	mutex_lock(&forge_mclk_mutex);
	if (en) {
		if (!forge_mclk_ready) {
			pr_err(PFX "MCLK%d EN requested but provider unavailable - sensor gets NO master clock; the coming I2C NAK is a CLOCK problem, not sensor identity\n",
			       idx + 1);
			goto out;
		}
		if (forge_mclk_users[idx]++ == 0) {
			if (forge_mclk_chain_get()) {
				forge_mclk_users[idx]--;
				goto out;
			}
			/* CLK_CFG_2 read (+ optional ungate) BEFORE bit29 so the
			 * marker shows the state the sensor actually powered up
			 * against. RMW mirrors how stock's gate op writes this
			 * register; the only other writer of 0x60 is a mux-sel
			 * change from SET_MCLK_PLL, which this leg itself issues
			 * before power-on, never concurrently. */
			if (forge_topck_base) {
				u32 cfg = readl(forge_topck_base +
						FORGE_TOPCK_CLK_CFG_2);

				if ((cfg & FORGE_CAMTG_PDN_BIT) &&
				    forge_cam_camtg_ungate) {
					writel(cfg & ~FORGE_CAMTG_PDN_BIT,
					       forge_topck_base +
					       FORGE_TOPCK_CLK_CFG_2);
					pr_err(PFX "CLK_CFG_2 was 0x%08x: PDN_CAMTG SET -> UNGATED by knob, now 0x%08x\n",
					       cfg,
					       readl(forge_topck_base +
						     FORGE_TOPCK_CLK_CFG_2));
				} else {
					pr_err(PFX "CLK_CFG_2=0x%08x PDN_CAMTG(bit7)=%s\n",
					       cfg,
					       (cfg & FORGE_CAMTG_PDN_BIT) ?
					       "GATED (camtg root OFF - no MCLK can reach any sensor; forge_cam_camtg_ungate=0 so NOT touching it)" :
					       "clear (root running)");
				}
			}
			if (forge_cam_hal_seninf_init &&
			    (m3note_board_known_is(M3NOTE_BOARD_L681) ||
			     m3note_board_known_is(M3NOTE_BOARD_M681)))
				forge_cam_hal_seninf_bits();
			if (forge_cam_hal_mclk &&
			    (m3note_board_known_is(M3NOTE_BOARD_L681) ||
			     m3note_board_known_is(M3NOTE_BOARD_M681))) {
				forge_mclk_hal_tg(idx);
				/* HAL enumerateSensor calls setMclk1 AND setMclk2
				 * with the same arguments; only the TG, no bit29
				 * (that one is the kernel power-on's, per socket) */
				if (forge_cam_hal_seninf_init &&
				    (m3note_board_known_is(M3NOTE_BOARD_L681) ||
				     m3note_board_known_is(M3NOTE_BOARD_M681)) &&
				    forge_mclk_users[!idx] == 0 &&
				    !forge_cam_hal_twin[!idx]) {
					forge_mclk_hal_tg(!idx);
					forge_cam_hal_twin[!idx] = true;
				}
			}
			val = readl(forge_seninf_base + forge_mclk_reg_off[idx]);
			if (forge_cam_tgclk_sel >= 0 && forge_cam_tgclk_sel <= 3) {
				val = (val & ~0x3u) | (u32)forge_cam_tgclk_sel;
				pr_err(PFX "MCLK%d TGCLK_SEL[1:0] := %d (knob)\n",
				       idx + 1, forge_cam_tgclk_sel);
			}
			writel(val | FORGE_MCLK_EN_BIT,
			       forge_seninf_base + forge_mclk_reg_off[idx]);
			val = readl(forge_seninf_base + forge_mclk_reg_off[idx]);
			/* One marker with everything the clock check needs:
			 *   +0x200  MCLK enable (bit29)
			 *   +0x204  SENINF_TGx_SEN_CK divider
			 *   camtg   the actual parent rate
			 * Reading +0x204 is safe here for the same reason +0x200
			 * is: the whole clock/power chain is held. */
			pr_err(PFX "MCLK%d ON seninf+0x%03x=0x%08x (bit29 %s) sen_ck=0x%08x camtg=%lu Hz\n",
			       idx + 1, forge_mclk_reg_off[idx], val,
			       (val & FORGE_MCLK_EN_BIT) ? "set" : "STUCK LOW",
			       readl(forge_seninf_base + forge_mclk_reg_off[idx] + 4),
			       forge_cam_camtg_rate());
			if (forge_cam_probe &&
			    m3note_board_known_is(M3NOTE_BOARD_L681)) {
				forge_cam_probe_dump("MCLK-ON");
				forge_cam_probe = 0;
			}
			if (forge_cam_fmeter &&
			    m3note_board_known_is(M3NOTE_BOARD_L681)) {
				forge_cam_fmeter_report("MCLK-ON");
				forge_cam_fmeter = 0;
			}
		}
	} else {
		if (forge_mclk_users[idx] <= 0) {
			forge_mclk_users[idx] = 0;
			goto out;	/* off while off: no-op, stock does this */
		}
		if (--forge_mclk_users[idx] == 0) {
			val = readl(forge_seninf_base + forge_mclk_reg_off[idx]);
			/* HAL setMclk(pcEn=0) clears PCEN with the clock */
			if (forge_cam_hal_mclk &&
			    (m3note_board_known_is(M3NOTE_BOARD_L681) ||
			     m3note_board_known_is(M3NOTE_BOARD_M681)))
				val &= ~BIT(31);
			writel(val & ~FORGE_MCLK_EN_BIT,
			       forge_seninf_base + forge_mclk_reg_off[idx]);
			pr_info(PFX "MCLK%d OFF seninf+0x%03x=0x%08x\n",
				idx + 1, forge_mclk_reg_off[idx],
				readl(forge_seninf_base + forge_mclk_reg_off[idx]));
			/* the twin TG started for this one: PCEN off with it */
			if (forge_cam_hal_twin[!idx] && forge_mclk_users[!idx] == 0) {
				u32 off = forge_mclk_reg_off[!idx];

				writel(readl(forge_seninf_base + off) & ~BIT(31),
				       forge_seninf_base + off);
				pr_err(PFX "MCLK%d twin TG PCEN off: PH_CNT=0x%08x\n",
				       !idx + 1, readl(forge_seninf_base + off));
			}
			forge_cam_hal_twin[!idx] = false;
			forge_mclk_chain_put();
		}
	}
out:
	mutex_unlock(&forge_mclk_mutex);
}

/* int (not BOOL) mirrors the retired mt_misc_stubs signatures; callers pass
 * 0/1 and arm64 zero-extends, so linking against the BOOL-typed externs in
 * kd_camera_hw.c is safe (same convention the stubs used). */
void ISP_MCLK1_EN(int En)
{
	forge_mclk_set(0, En);
}

void ISP_MCLK2_EN(int En)
{
	forge_mclk_set(1, En);
}
