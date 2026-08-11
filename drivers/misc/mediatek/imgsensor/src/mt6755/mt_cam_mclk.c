/*
 * ISP_MCLK1/2_EN provider for mt6755.
 *
 * The stock owner of ISP_MCLK*_EN (camera_isp.c) does not provide them
 * here, and a no-op MCLK means every sensor-detect I2C transaction NAKs
 * (no master clock). This provider does the real thing, and where it
 * cannot, it logs why.
 *
 * Register map (stock):
 *   - ISP_MCLK1_EN: bit29 of ISP_ADDR+0x4200; MCLK2 = +0x4600. ISP_ADDR
 *     maps 0x15004000, so phys = 0x15008200/0x15008600 = SENINF base
 *     (0x15008000, = the kd_camera_hw1 reg) + 0x0200/0x0600. The mt6757
 *     camera_isp.c uses the same SENINF0_BASE+0x0200 bit29 layout.
 *   - ISP_EnableClock: the clock/power chain that stock holds while
 *     touching this MMIO, in exactly this order:
 *       CG_SCP_SYS_DIS -> CG_DISP0_SMI_COMMON -> CG_SCP_SYS_ISP ->
 *       CG_IMAGE_CAM_SMI -> CG_IMAGE_CAM_CAM -> CG_IMAGE_SEN_TG ->
 *       CG_IMAGE_SEN_CAM -> CG_IMAGE_CAM_SV -> CG_IMAGE_LARB2_SMI
 *     The mt6755.dtsi "mediatek,ispsys" node carries exactly this list under
 *     these clock-names, wired to clk-mt6755-pg (SCP_SYS_* MTCMOS-as-clock)
 *     and the imgsys CG provider.
 *
 * MMIO behind an off MTCMOS wedges the AXI bus, so the SENINF write
 * happens only after every clock in the chain reported success. Any failure
 * is logged ([FORGE_CAM] pr_err), unwinds, and leaves the MMIO untouched.
 * Nothing here runs at boot: the only callers sit behind the
 * forge_cam_power gate in kd_camera_hw.c (default 0).
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
#include <linux/moduleparam.h>

#define PFX "[FORGE_CAM] MCLK: "

/* actual camtg frequency, from kd_sensorlist.c (built-in) */
extern unsigned long forge_cam_camtg_rate(void);

/* Stock order, see the header comment. Index constants for readability. */
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

/* TOPCKGEN CLK_CFG_2 visibility and optional ungate.
 * Stock clk-mt6755 registers camtg_sel with its pdn bit (gate = 7), so on
 * stock clk_prepare_enable(camtg_sel) in CAMERA_HW_Open clears PDN_CAMTG
 * and the sensor-clock root runs. The MUX_UPD table here carries
 * INVALID_MUX_GATE for every top mux, so the same call is a hardware
 * no-op and bit7 keeps whatever LK/reset left. A set bit starves both
 * CMMCLK pads at once, even with SENINF bit29, the TG divider, the pad
 * mux and every rail correct.
 * Default is read-only: the MCLK-ON marker prints CLK_CFG_2 and the bit.
 * forge_cam_camtg_ungate=1 additionally clears bit7 at MCLK-ON (replaying
 * stock's gate op). One-way: MCLK-OFF never re-gates. */
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
 * TG clock select. The stock l681 kernel leaves SENINF TG1/TG2 PH_CNT
 * (ISP_ADDR+0x4200/0x4600 = SENINF+0x200/0x600) at 0x00000001 on every ISP
 * release, and ISP_MCLK1_EN only ORs bit29 into that. Here MCLK-ON reads
 * 0x20000000: bits[1:0] = 0. With this knob >= 0, MCLK-ON writes
 * bits[1:0] = knob (stock value: 1) before bit29.
 * -1 (default) = leave as is.
 */
/*
 * Do what the camera HAL does before it powers a sensor
 * (libcam.halsensor.so SeninfDrvImp::setMclk1): SENINF+0x000 |= 0x400 |
 * 0x800; TGx_SEN_CK (+0x204): CLKRS[13:8] = 0, CLKFL[5:0] = (cnt + 1) >> 1,
 * CLKCNT[21:16] = cnt; TGx_PH_CNT (+0x200): bit2 = !(cnt & 1), bit28 =
 * clkPol, TGCLK_SEL[1:0] = 1, bit6 = padPclkInv, bit31 (PCEN) = pcEn; then
 * usleep(100). ISP_MCLK1_EN only adds bit29, so a kernel-only detect never
 * starts the TG phase counter (PCEN) and the sensors NAK.
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

/* Read-only SENINF register dump: the stock ISP RegDump seninf ranges. */
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

/* the board is the DT machine */
static bool forge_cam_m3note_board(void)
{
	return of_machine_is_compatible("meizu,m681") ||
	       of_machine_is_compatible("meizu,l681");
}

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

/* Enable the stock clock chain (power domain + CGs). Caller holds the mutex. */
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

/* HAL SeninfDrvImp::setMclk1/2 on TG idx (as a helper so the SENINF init
 * step can start the other TG too). Mutex and chain held. */
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
 * The SENINF part of the HAL's SeninfDrvImp::init, before setMclk
 * (libcam.halsensor.so): +0x120/+0x520/+0x920/+0xD20 |= bit31, then
 * +0x100/+0x500/+0x900 |= 1. MCLK reaches the pads and both sensors answer
 * only with these bits plus setMclk1/2 with PCEN. SoC registers only.
 * Default 1 on M3 Note; 0 = only the TG step. Mutex and chain held.
 */
static int forge_cam_hal_seninf_init = 1;
module_param(forge_cam_hal_seninf_init, int, 0644);
MODULE_PARM_DESC(forge_cam_hal_seninf_init, "M3 Note: 1 (default) = at MCLK-ON replay HAL SeninfDrvImp::init (SENINF +0x100/0x500/0x900 |= 1, +0x120/0x520/0x920/0xD20 |= bit31) and start the other TG like enumerateSensor's setMclk2; 0 = n7 TG only");
static bool forge_cam_hal_twin[2];	/* TG started as the other one's twin */

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
			if (forge_cam_hal_seninf_init && forge_cam_m3note_board())
				forge_cam_hal_seninf_bits();
			if (forge_cam_hal_mclk && forge_cam_m3note_board()) {
				forge_mclk_hal_tg(idx);
				/* HAL enumerateSensor calls setMclk1 AND setMclk2
				 * with the same arguments; only the TG, no bit29
				 * (that one is the kernel power-on's, per socket) */
				if (forge_cam_hal_seninf_init &&
				    forge_mclk_users[!idx] == 0 &&
				    !forge_cam_hal_twin[!idx]) {
					forge_mclk_hal_tg(!idx);
					forge_cam_hal_twin[!idx] = true;
				}
			}
			val = readl(forge_seninf_base + forge_mclk_reg_off[idx]);
			val = readl(forge_seninf_base + forge_mclk_reg_off[idx]);
			if (forge_cam_tgclk_sel >= 0 && forge_cam_tgclk_sel <= 3) {
				val = (val & ~0x3u) | (u32)forge_cam_tgclk_sel;
				pr_err(PFX "MCLK%d TGCLK_SEL[1:0] := %d (knob)\n",
				       idx + 1, forge_cam_tgclk_sel);
			}
			writel(val | FORGE_MCLK_EN_BIT,
			       forge_seninf_base + forge_mclk_reg_off[idx]);
			val = readl(forge_seninf_base + forge_mclk_reg_off[idx]);
			/* One marker carries what the clock check needs:
			 *   +0x200  MCLK enable (bit29)
			 *   +0x204  SENINF_TGx_SEN_CK, the TG divider
			 *   camtg   the actual parent rate
			 * Reading +0x204 is safe here for the same reason +0x200 is: the whole
			 * clock/power chain is held. */
			pr_err(PFX "MCLK%d ON seninf+0x%03x=0x%08x (bit29 %s) sen_ck=0x%08x camtg=%lu Hz\n",
			       idx + 1, forge_mclk_reg_off[idx], val,
			       (val & FORGE_MCLK_EN_BIT) ? "set" : "STUCK LOW",
			       readl(forge_seninf_base + forge_mclk_reg_off[idx] + 4),
			       forge_cam_camtg_rate());
		}
	} else {
		if (forge_mclk_users[idx] <= 0) {
			forge_mclk_users[idx] = 0;
			goto out;	/* off while off: no-op, stock does this */
		}
		if (--forge_mclk_users[idx] == 0) {
			val = readl(forge_seninf_base + forge_mclk_reg_off[idx]);
			/* HAL setMclk(pcEn=0) clears PCEN with the clock */
			if (forge_cam_hal_mclk && forge_cam_m3note_board())
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
