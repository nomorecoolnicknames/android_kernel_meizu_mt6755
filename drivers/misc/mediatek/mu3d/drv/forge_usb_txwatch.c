/*
 * TX (IN) side stall watch for the mu3d gadget. The OUT/RX path has
 * counters (forge_usb_epstat) and reset-bracket snapshots; this sampler
 * covers the TX path without touching the other driver files.
 *
 * Everything is behind two default-off module params; with both at 0 the
 * only cost is a dormant 2 s delayed work that re-arms and returns.
 */

#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include "musb_core.h"
#include "mu3d_hal_osal.h"
#include "mu3d_hal_hw.h"

#define FORGE_TXW_NEP		5	/* TX eps 1..4 (MAX_EP_NUM=8: 4Tx+4Rx) */
#define FORGE_TXW_PERIOD_MS	2000
#define FORGE_TXW_STALL_N	3	/* samples with no progress => STALL */
#define FORGE_TXW_REPOKE	5	/* re-poke every N stalled samples */

static int forge_usb_txwatch;
module_param(forge_usb_txwatch, int, 0644);
MODULE_PARM_DESC(forge_usb_txwatch,
		 "FORGE m681: 1 = sample TX QMU state every 2s, log on change/stall (default 0)");

static int forge_usb_txheal;
module_param(forge_usb_txheal, int, 0644);
MODULE_PARM_DESC(forge_usb_txheal,
		 "FORGE m681: 1 = on TX stall, poke QMU_Q_RESUME on the stalled ep (default 0)");

struct forge_txw_ep {
	u32 reqlen;
	u32 tqcsr;
	u32 tqcpr;
	u32 stall_n;		/* consecutive no-progress samples */
	u32 pokes;
};

static struct forge_txw_ep forge_txw[FORGE_TXW_NEP];
static u32 forge_txw_errlatch[4];	/* TQERRIR0, RQERRIR0, RQERRIR1, QEMIR */
static void forge_txw_work_fn(struct work_struct *w);
static DECLARE_DELAYED_WORK(forge_txw_work, forge_txw_work_fn);

static void forge_txw_sample(struct musb *musb)
{
	unsigned long flags;
	u32 lat[4];
	int i;

	spin_lock_irqsave(&musb->lock, flags);

	/* S3 lesson: MAC reads with the clock gated are AXI-hazardous or
	 * return reset defaults.  Checked inside the lock so the suspend
	 * work cannot gate the clock between check and read.
	 */
	if (!musb->is_clk_on || !musb->gadget_driver
	    || musb->g.speed == USB_SPEED_UNKNOWN) {
		spin_unlock_irqrestore(&musb->lock, flags);
		return;
	}

	lat[0] = os_readl(U3D_TQERRIR0);
	lat[1] = os_readl(U3D_RQERRIR0);
	lat[2] = os_readl(U3D_RQERRIR1);
	lat[3] = os_readl(U3D_QEMIR);

	for (i = 1; i < FORGE_TXW_NEP; i++) {
		struct musb_ep *mep = &musb->endpoints[i].ep_in;
		struct forge_txw_ep *s = &forge_txw[i];
		struct list_head *pos;
		u32 reqlen = 0, tqcsr, tqcpr;
		bool progress;

		list_for_each(pos, &mep->req_list)
			reqlen++;
		tqcsr = os_readl(USB_QMU_TQCSR(i));
		tqcpr = os_readl(USB_QMU_TQCPR(i));

		/* Progress = anything moved.  A stalled ep holds requests
		 * while the completion pointer freezes; an idle ep has
		 * reqlen == 0 and never counts as stalled.
		 */
		progress = (reqlen != s->reqlen) || (tqcpr != s->tqcpr);

		if (reqlen && !progress) {
			s->stall_n++;
			if (s->stall_n == FORGE_TXW_STALL_N)
				pr_notice("[FORGE_USB] txwatch ep%d STALL n=%u reqlen=%u tqcsr=%08x tqcpr=%08x tqerrir0=%08x qemir=%08x\n",
					  i, s->stall_n, reqlen, tqcsr, tqcpr,
					  lat[0], lat[3]);
			if (forge_usb_txheal
			    && s->stall_n >= FORGE_TXW_STALL_N
			    && ((s->stall_n - FORGE_TXW_STALL_N)
				% FORGE_TXW_REPOKE) == 0) {
				os_writel(USB_QMU_TQCSR(i), QMU_Q_RESUME);
				s->pokes++;
				pr_notice("[FORGE_USB] txheal ep%d poke#%u tqcsr was %08x\n",
					  i, s->pokes, tqcsr);
			}
		} else {
			if (s->stall_n >= FORGE_TXW_STALL_N)
				pr_notice("[FORGE_USB] txwatch ep%d recovered after n=%u (pokes=%u)\n",
					  i, s->stall_n, s->pokes);
			s->stall_n = 0;
		}

		s->reqlen = reqlen;
		s->tqcsr = tqcsr;
		s->tqcpr = tqcpr;
	}

	spin_unlock_irqrestore(&musb->lock, flags);

	/* Error latches: report edges only, outside the lock. */
	for (i = 0; i < 4; i++) {
		if (lat[i] != forge_txw_errlatch[i]) {
			pr_notice("[FORGE_USB] txwatch errlatch[%d] %08x -> %08x (TQERRIR0/RQERRIR0/RQERRIR1/QEMIR)\n",
				  i, forge_txw_errlatch[i], lat[i]);
			forge_txw_errlatch[i] = lat[i];
		}
	}
}

static void forge_txw_work_fn(struct work_struct *w)
{
	if (forge_usb_txwatch && _mu3d_musb)
		forge_txw_sample(_mu3d_musb);

	schedule_delayed_work(&forge_txw_work,
			      msecs_to_jiffies(FORGE_TXW_PERIOD_MS));
}

static int __init forge_txw_init(void)
{
	schedule_delayed_work(&forge_txw_work,
			      msecs_to_jiffies(FORGE_TXW_PERIOD_MS));
	return 0;
}
late_initcall(forge_txw_init);
