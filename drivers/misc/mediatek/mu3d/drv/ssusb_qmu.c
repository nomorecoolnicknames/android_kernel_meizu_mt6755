/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifdef USE_SSUSB_QMU
#include <linux/spinlock.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/ratelimit.h>
#include <linux/workqueue.h>

#include "musb_core.h"
#include "mu3d_hal_osal.h"
#include "mu3d_hal_qmu_drv.h"
#include "mu3d_hal_hw.h"
#include "ssusb_qmu.h"

/*
 * QMU descriptor-walk guard.
 *
 * The 4.4 vendor drop replaced every BUG_ON() in this file with WARN_ON()
 * and left no return path behind it, so each "this must never happen"
 * check warns and then executes the statement it just complained about. In
 * qmu_done_rx() that is fatal:
 *
 *   1. the walk flags !TGPD_GET_NEXT(gpd)                 (warn, continue)
 *   2. gpd = TGPD_GET_NEXT(gpd)                           => NULL
 *   3. gpd = gpd_phys_to_virt(NULL, USB_RX, ep)
 *        = rx_gpd_map[ep].p_desc - rx_gpd_map[ep].p_desc_dma
 *        = PAGE_OFFSET - PHYS_OFFSET  = 0xffffffbfc0000000 -- non-NULL, so
 *          the vendor's `if (!gpd)` check below cannot catch it
 *   4. the next loop-top TGPD_IS_FLAGS_HWO(gpd) reads that address, which
 *      sits in the hole below the 39-bit TTBR1 base => data abort with
 *      DFSC 0x04 (level 0 translation fault), ESR 0x96000004. (A plain NULL
 *      would fault at level 1 on this VA_BITS=39/4K kernel.)
 *
 * With CONFIG_PANIC_ON_OOPS=y that Oops in softirq resets the SoC.
 *
 * Every guarded exit completes the pending request with -EPROTO first: a
 * bare return would abandon the usb_request already on musb_ep->req_list
 * (its status stays -EINPROGRESS, functionfs never completes the
 * read/write) and leave Rx_gpd_last[] on the damaged descriptor, wedging
 * the endpoint. The giveback uses the musb_g_giveback() this walk already
 * calls mid-loop (annotated __releases/__acquires ep->musb->lock;
 * qmu_done_tasklet is its context). f_fs.c returns req->status to userspace
 * verbatim, so what matters is that the request completes at all; -EPROTO
 * only distinguishes it from f_fs's own queue-failure -EIO.
 *
 * Default 1. Disarm at runtime or from the command line:
 *   echo 0 > /sys/module/musb_hdrc/parameters/forge_qmu_guard
 * or musb_hdrc.forge_qmu_guard=0 on the kernel command line.
 *
 * This knob arms only the GPD-walk guard (early exit + giveback, the
 * branches tagged RXD- and TXD- below). Scheduling musb->error_recovery for
 * RXQ_ZLPERR and RXQ_EMPTY is separate, behind forge_qmu_recover; dmesg
 * tags them apart (guard=%d vs recover=%d).
 *
 * Every guarded site prints an unconditional, ratelimited pr_notice before
 * deciding, so the disarmed leg still names the branch that fired.
 * pr_notice is used rather than qmu_printk() because the latter is masked
 * off by musb_core.c's default debug_level.
 */
static int forge_qmu_guard = 1;
module_param(forge_qmu_guard, int, 0644);
MODULE_PARM_DESC(forge_qmu_guard,
		 "FORGE m681: 1 = give the pending request back with -EPROTO and abandon the QMU GPD walk instead of continuing with a damaged descriptor (default, device-proven 2026-08-20); 0 = stock warn-and-continue.  Walk guard ONLY -- see forge_qmu_recover for the ZLPERR/RX-EMPTY half");

/* RXD-NOREQ hits this boot (the line itself prints 1 per 256) */
static unsigned int forge_qmu_rxd_noreq;
module_param(forge_qmu_rxd_noreq, uint, 0444);
MODULE_PARM_DESC(forge_qmu_rxd_noreq, "RXD-NOREQ (request list ran empty mid-walk) count");

/*
 * Lets RXQ_ZLPERR / RXQ_EMPTY schedule the existing error_recovery tasklet
 * (flush + re-arm) instead of being latched and dropped as in stock; it
 * arms nothing in the walk.
 *
 * The recovery body is heavy. qmu_error_recovery() runs under
 * spin_lock_irqsave(&musb->lock), and for the decoded endpoint
 * _ex_mu3d_hal_flush_qmu():
 *
 *   1. mu3d_hal_stop_qmu() - if the queue is active it writes QMU_Q_STOP
 *      and then wait_for_value(RQCSR, QMU_Q_ACTIVE, 0, 10, 100) with
 *      mdelay(), so an unstoppable queue busy-waits up to one second with
 *      interrupts disabled, plus a WARN_ON(1) backtrace;
 *   2. free_gpd() - memsets the whole GPD ring for that endpoint, zeroing
 *      any descriptor the host is mid-transfer into;
 *   3. Rx_gpd_end/Rx_gpd_last reset to head and RQSAR rewritten;
 *   4. every request still on req_list re-inserted from offset 0 (on the RX
 *      leg request->actual is not reset first, unlike the TX leg).
 *
 * RXQ_ZLPERR is a genuine error latch. RXQ_EMPTY is reached whenever the
 * host sends while no GPD is armed, i.e. in the gap between two f_fs reads,
 * many times per second; recovering on each would memset the ring under a
 * live adb stream and, with req_list empty, re-arm nothing, so the next
 * packet raises RXQ_EMPTY again: a self-sustaining flush loop.
 *
 * 1 = arm RXQ_ZLPERR, 2 = also RXQ_EMPTY. Default 0: level 2 does not fix
 * the OUT stall, and RX-empty, the normal end-of-queue condition here,
 * storms and floods the kernel log. Settable at runtime (0644).
 */
static int forge_qmu_recover;
module_param(forge_qmu_recover, int, 0644);
MODULE_PARM_DESC(forge_qmu_recover,
		 "FORGE m681: 0 = stock clear-the-latch-and-continue; 1 = schedule the QMU error_recovery tasklet (flush + re-arm) on RXQ_ZLPERR only (default); 2 = also on RXQ_EMPTY (UNTESTED, runtime only -- RX-empty is a normal end-of-queue condition here and recovering on it flushes the ring under a live transfer).  Independent of forge_qmu_guard; both levels spend forge_qmu_recover_budget");

/*
 * Rate limit for the recovery above. Each latch that schedules
 * error_recovery spends one credit; at zero the latch is still printed but
 * no longer recovers, so a condition that repeats at interrupt rate
 * degrades to the stock behaviour instead of a flush storm. Large but
 * finite, since RXQ_EMPTY fires dozens of times per second. Negative =
 * unlimited (diagnostic only). Writable, so a live device can be topped up:
 *   echo 16 > /sys/module/musb_hdrc/parameters/forge_qmu_recover_budget
 */
static int forge_qmu_recover_budget = 100000;
module_param(forge_qmu_recover_budget, int, 0644);
MODULE_PARM_DESC(forge_qmu_recover_budget,
		 "FORGE m681: number of forge-initiated QMU error recoveries still allowed since boot (16 by default, negative = unlimited).  Spent by RXQ_ZLPERR and, at forge_qmu_recover=2, RXQ_EMPTY");

/*
 * Returns true and spends one credit when `level` is armed and budget is
 * left.  Called only from qmu_exception_interrupt(), i.e. from the QMU
 * interrupt path, so the non-atomic decrement has no second writer beyond a
 * sysfs store.
 */
static bool forge_qmu_recover_take(int level)
{
	if (forge_qmu_recover < level)
		return false;
	if (forge_qmu_recover_budget == 0)
		return false;
	if (forge_qmu_recover_budget > 0)
		forge_qmu_recover_budget--;
	return true;
}

/*
 * A GPD is only ever legal inside the ring init_gpd_list() allocated for
 * that endpoint.  This is the check the vendor's `if (!gpd)` was reaching
 * for and missing: gpd_phys_to_virt() never returns NULL, it returns a
 * wild pointer, so NULL-testing its result can never fire.
 */
static bool forge_qmu_gpd_in_ring(TGPD *gpd, USB_DIR dir, u8 ep_num)
{
	GPD_R *ring;

	if (!gpd || ep_num >= 15)
		return false;

	ring = (dir == USB_RX) ? &Rx_gpd_List[ep_num] : &Tx_gpd_List[ep_num];
	if (!ring->pStart || !ring->pEnd)
		return false;

	return (void *)gpd >= (void *)ring->pStart &&
	       (void *)gpd < (void *)ring->pEnd;
}

/* Sanity CR check in */
/*
 *   1. Find the last gpd HW has executed and update Tx_gpd_last[]
 *   2. Set the flag for txstate to know that TX has been completed
 *
 *   ported from proc_qmu_tx() from test driver.
 *
 *   caller:qmu_interrupt after getting QMU done interrupt and TX is raised
*/
void qmu_done_tx(struct musb *musb, u8 ep_num, unsigned long flags)
{
	TGPD *gpd = Tx_gpd_last[ep_num];
	/* QMU GPD address --> CPU DMA address */
	TGPD *gpd_current = (TGPD *) (uintptr_t) (os_readl(USB_QMU_TQCPR(ep_num)));
	struct musb_ep *musb_ep = &musb->endpoints[ep_num].ep_in;
	struct usb_request *request = NULL;
	struct musb_request *req = NULL;
	static DEFINE_RATELIMIT_STATE(ratelimit_tx, 1 * HZ, 3);

	/*Transfer PHY addr got from QMU register to VIR addr */
	gpd_current = gpd_phys_to_virt((void *)gpd_current, USB_TX, ep_num);

	/*
	 *  gpd or Last       gdp_current
	 *  |                  |
	 *  |->  GPD1 --> GPD2 --> GPD3 --> GPD4 --> GPD5 -|
	 *  |----------------------------------------------|
	 */

	qmu_printk(K_DEBUG, "[TXD] %s EP %d, Last=%p, Current=%p, End=%p\n",
		   __func__, ep_num, gpd, gpd_current, Tx_gpd_end[ep_num]);

	/*gpd_current should at least point to the next GPD to the previous last one. */
	if (gpd == gpd_current) {
		if (__ratelimit(&ratelimit_tx))
			qmu_printk(K_ERR, "[TXD] %s gpd(%p) == gpd_current(%p)\n", __func__, gpd,
			   gpd_current);
		return;
	}

	/*
	 * TX has no equivalent of the RX `!gpd || !gpd_current` guard, so the
	 * TGPD_IS_FLAGS_HWO() below is the first unchecked dereference on this
	 * path; Tx_gpd_last[] is NULL until the queue is armed. Same knob,
	 * same "print always, act only when armed" shape.
	 */
	if (!forge_qmu_gpd_in_ring(gpd, USB_TX, ep_num) ||
	    !forge_qmu_gpd_in_ring(gpd_current, USB_TX, ep_num)) {
		pr_notice_ratelimited(
			"[FORGE_QMU] TXD-ENTRY ep%d gpd=%p gpd_current=%p outside ring [%p,%p) guard=%d\n",
			ep_num, gpd, gpd_current, Tx_gpd_List[ep_num < 15 ? ep_num : 0].pStart,
			Tx_gpd_List[ep_num < 15 ? ep_num : 0].pEnd, forge_qmu_guard);
		if (forge_qmu_guard) {
			/*
			 * Nothing has been fetched off req_list yet on this
			 * path (req is still NULL), so take the head and
			 * complete it with an error instead of leaving it
			 * queued at -EINPROGRESS forever.
			 */
			req = next_request(musb_ep);
			if (req)
				musb_g_giveback(musb_ep, &req->request, -EPROTO);
			return;
		}
	}

	if (TGPD_IS_FLAGS_HWO(gpd)) {
		/*
		 * Not an early return: the while below re-tests
		 * !TGPD_IS_FLAGS_HWO(gpd) and runs zero iterations, so nothing
		 * is dereferenced past this point; a return would only suppress
		 * the TQCSR/TQSAR/TQCPR/QCR0-3/QGCSR dump further down, which
		 * shows what left HWO set. Print and fall through.
		 */
		pr_notice_ratelimited(
			"[FORGE_QMU] TXD-HWO ep%d gpd=%p still owned by HW, TQCPR=%x guard=%d (walk is a no-op, falling through to the register dump)\n",
			ep_num, gpd, os_readl(USB_QMU_TQCPR(ep_num)), forge_qmu_guard);
		qmu_printk(K_DEBUG, "[TXD] %s HWO=1, CPR=%x\n", __func__,
			   os_readl(USB_QMU_TQCPR(ep_num)));
		WARN_ON(1);
	}

	while (gpd != gpd_current && !TGPD_IS_FLAGS_HWO(gpd)) {
#if defined(CONFIG_USB_MU3D_DRV_36BIT)
		qmu_printk(K_DEBUG,
			"[TXD] gpd=%p ->HWO=%d, BPD=%d, Next_GPD=%lx, DataBuffer=%lx,BufferLen=%d request=%p\n",
			gpd, (u32) TGPD_GET_FLAG(gpd),
			(u32) TGPD_GET_FORMAT(gpd), (uintptr_t) TGPD_GET_NEXT_TX(gpd),
			(uintptr_t) TGPD_GET_DATA_TX(gpd), (u32) TGPD_GET_BUF_LEN(gpd), req);

		if (!TGPD_GET_NEXT_TX(gpd)) {
			/*
			 * The vendor code already breaks here; unlike the RX
			 * copy, TX never advances into the NULL.
			 */
			pr_notice_ratelimited(
				"[FORGE_QMU] TXD-NEXTNULL36 ep%d gpd=%p next=0 (vendor break, no advance) guard=%d\n",
				ep_num, gpd, forge_qmu_guard);
			qmu_printk(K_ERR, "[TXD][ERROR] Next GPD is null!!\n");
			/* BUG_ON(1); */
			break;
		}

		gpd = TGPD_GET_NEXT_TX(gpd);

#else
		qmu_printk(K_DEBUG,
			"[TXD] gpd=%p ->HWO=%d, BPD=%d, Next_GPD=%lx, DataBuffer=%lx,BufferLen=%d request=%p\n",
			gpd, (u32) TGPD_GET_FLAG(gpd),
			(u32) TGPD_GET_FORMAT(gpd), (uintptr_t) TGPD_GET_NEXT(gpd),
			(uintptr_t) TGPD_GET_DATA(gpd), (u32) TGPD_GET_BUF_LEN(gpd), req);

		if (!TGPD_GET_NEXT(gpd)) {
			/* vendor code already breaks before advancing */
			pr_notice_ratelimited(
				"[FORGE_QMU] TXD-NEXTNULL ep%d gpd=%p next=0 (vendor break, no advance) guard=%d\n",
				ep_num, gpd, forge_qmu_guard);
			qmu_printk(K_ERR, "[TXD][ERROR] Next GPD is null!!\n");
			/* BUG_ON(1); */
			break;
		}

		gpd = TGPD_GET_NEXT(gpd);

#endif

		gpd = gpd_phys_to_virt(gpd, USB_TX, ep_num);

		/*
		 * TX has no validity check at all on the translated pointer
		 * (not even the `if (!gpd)` the RX copy has), and everything
		 * below dereferences it.
		 */
		if (!forge_qmu_gpd_in_ring(gpd, USB_TX, ep_num)) {
			pr_notice_ratelimited(
				"[FORGE_QMU] TXD-XLATE ep%d translated gpd=%p outside ring [%p,%p) guard=%d\n",
				ep_num, gpd, Tx_gpd_List[ep_num < 15 ? ep_num : 0].pStart,
				Tx_gpd_List[ep_num < 15 ? ep_num : 0].pEnd, forge_qmu_guard);
			if (forge_qmu_guard) {
				/*
				 * `req` here is the request the PREVIOUS
				 * iteration already gave back (or NULL on the
				 * first one) -- it must not be reused.  Re-read
				 * the head of req_list, which is the one this
				 * aborted iteration would have completed.
				 */
				req = next_request(musb_ep);
				if (req)
					musb_g_giveback(musb_ep, &req->request,
							-EPROTO);
				return;
			}
		}

		/* trying to give_back the request to gadget driver. */
		req = next_request(musb_ep);
		if (!req) {
			qmu_printk(K_INFO, "[TXD] %s Cannot get next request of %d, but QMU has done.\n",
				   __func__, ep_num);
			return;
		}

		request = &req->request;

		Tx_gpd_last[ep_num] = gpd;
		musb_g_giveback(musb_ep, request, 0);
		req = next_request(musb_ep);
		if (req != NULL)
			request = &req->request;
	}

	if (gpd != gpd_current && TGPD_IS_FLAGS_HWO(gpd)) {
		qmu_printk(K_ERR, "[TXD][ERROR] EP%d TQCSR=%x, TQSAR=%x, TQCPR=%x\n",
			   ep_num, os_readl(USB_QMU_TQCSR(ep_num)), os_readl(USB_QMU_TQSAR(ep_num)),
			   os_readl(USB_QMU_TQCPR(ep_num)));

		qmu_printk(K_ERR, "[TXD][ERROR] QCR0=%x, QCR1=%x, QCR2=%x, QCR3=%x, QGCSR=%x\n",
			   os_readl(U3D_QCR0), os_readl(U3D_QCR1), os_readl(U3D_QCR2),
			   os_readl(U3D_QCR3), os_readl(U3D_QGCSR));
#if defined(CONFIG_USB_MU3D_DRV_36BIT)
		qmu_printk(K_ERR, "[TXD][ERROR] HWO=%d, BPD=%d, Next_GPD=%lx\n",
			   (u32) TGPD_GET_FLAG(gpd),
			   (u32) TGPD_GET_FORMAT(gpd), (uintptr_t) TGPD_GET_NEXT_TX(gpd));

		qmu_printk(K_ERR, "[TXD][ERROR] DataBuffer=%lx, BufferLen=%d, Endpoint=%d\n",
			   (uintptr_t) TGPD_GET_DATA_TX(gpd), (u32) TGPD_GET_BUF_LEN(gpd),
			   (u32) TGPD_GET_EPaddr(gpd));
#else
		qmu_printk(K_ERR, "[TXD][ERROR] HWO=%d, BPD=%d, Next_GPD=%lx\n",
			   (u32) TGPD_GET_FLAG(gpd),
			   (u32) TGPD_GET_FORMAT(gpd), (uintptr_t) TGPD_GET_NEXT(gpd));

		qmu_printk(K_ERR, "[TXD][ERROR] DataBuffer=%lx, BufferLen=%d, Endpoint=%d\n",
			   (uintptr_t) TGPD_GET_DATA(gpd), (u32) TGPD_GET_BUF_LEN(gpd),
			   (u32) TGPD_GET_EPaddr(gpd));
#endif
	}

	qmu_printk(K_DEBUG, "[TXD] %s EP%d, Last=%p, End=%p, complete\n", __func__,
		   ep_num, Tx_gpd_last[ep_num], Tx_gpd_end[ep_num]);

	if (req != NULL) {
		if (request->length == 0) {
			u32 val = 0;

			qmu_printk(K_DEBUG, "[TXD] ==Send ZLP== %p\n", req);

			if (wait_for_value_us
			    (USB_END_OFFSET(req->epnum, U3D_TX1CSR0), TX_FIFOEMPTY, TX_FIFOEMPTY, 1,
			     10) == RET_SUCCESS)
				qmu_printk(K_DEBUG, "Tx[%d] 0x%x\n", req->epnum,
					   USB_ReadCsr32(U3D_TX1CSR0, req->epnum));
			else {
				qmu_printk(K_CRIT, "Tx[%d] NOT FIFOEMPTY 0x%x\n", req->epnum,
					   USB_ReadCsr32(U3D_TX1CSR0, req->epnum));
				return;
			}

			/*Disable Tx_DMAREQEN */
			val = USB_ReadCsr32(U3D_TX1CSR0, req->epnum) & ~TX_DMAREQEN;

			mb();	/* avoid context swtich */

			USB_WriteCsr32(U3D_TX1CSR0, req->epnum, val);

			val = USB_ReadCsr32(U3D_TX1CSR0, req->epnum) | TX_TXPKTRDY;

			mb();	/* avoid context swtich */

			USB_WriteCsr32(U3D_TX1CSR0, req->epnum, val);

			qmu_printk(K_DEBUG,
				   "[TXD] Giveback ZLP of EP%d, actual:%d, length:%d %p\n",
				   req->epnum, request->actual, request->length, request);

			musb_g_giveback(musb_ep, request, 0);
		}
	}
}

/*
 *   When receiving RXQ done interrupt, qmu_interrupt calls this function.
 *
 *  1. Traverse GPD/BD data structures to count actual transferred length.
 *  2. Set the done flag to notify rxstate_qmu() to report status to upper gadget driver.
 *
 *   ported from proc_qmu_rx() from test driver.
 *
 *   caller:qmu_interrupt after getting QMU done interrupt and TX is raised
 *
*/
/* See the epstat block in musb_gadget.c. Counting the RX-done interrupt
 * here tells "the controller stopped reporting finished transfers" from
 * "the function driver stopped handing them back". */
#define FORGE_USB_NEP 16
extern u32 forge_usb_qmu_rx[FORGE_USB_NEP];
extern u32 forge_usb_qmu_err[FORGE_USB_NEP];
extern u32 forge_usb_qmu_rxempty[FORGE_USB_NEP];

void qmu_done_rx(struct musb *musb, u8 ep_num, unsigned long flags)
{
	TGPD *gpd = Rx_gpd_last[ep_num];
	/* QMU GPD address --> CPU DMA address */
	TGPD *gpd_current = (TGPD *) (uintptr_t) (os_readl(USB_QMU_RQCPR(ep_num)));
	struct musb_ep *musb_ep = &musb->endpoints[ep_num].ep_out;
	struct usb_request *request = NULL;
	struct musb_request *req;

	static DEFINE_RATELIMIT_STATE(ratelimit_rx, 1 * HZ, 3);

	/* below EVERY declaration in this function, including the static
	 * ratelimit state: this file builds -Werror=declaration-after-statement */
	if (ep_num < FORGE_USB_NEP)
		forge_usb_qmu_rx[ep_num]++;

	/* trying to give_back the request to gadget driver. */
	req = next_request(musb_ep);
	if (!req) {
		qmu_printk(K_ERR, "[RXD] %s Cannot get next request of %d, but QMU has done.\n",
			   __func__, ep_num);
		return;
	}

	request = &req->request;

	/*Transfer PHY addr got from QMU register to VIR addr */
	gpd_current = gpd_phys_to_virt(gpd_current, USB_RX, ep_num);

	qmu_printk(K_DEBUG, "[RXD] %s EP%d, Last=%p, Current=%p, End=%p\n",
		   __func__, ep_num, gpd, gpd_current, Rx_gpd_end[ep_num]);

	/*gpd_current should at least point to the next GPD to the previous last one. */
	if (gpd == gpd_current) {
		if (__ratelimit(&ratelimit_rx)) {
			qmu_printk(K_ERR, "[RXD][ERROR] %s gpd(%p) == gpd_current(%p)\n", __func__, gpd,
				   gpd_current);
			qmu_printk(K_ERR, "[RXD][ERROR] EP%d RQCSR=%x, RQSAR=%x, RQCPR=%x, RQLDPR=%x\n",
				   ep_num, os_readl(USB_QMU_RQCSR(ep_num)), os_readl(USB_QMU_RQSAR(ep_num)),
				   os_readl(USB_QMU_RQCPR(ep_num)), os_readl(USB_QMU_RQLDPR(ep_num)));
			qmu_printk(K_ERR, "[RXD][ERROR] QCR0=%x, QCR1=%x, QCR2=%x, QCR3=%x, QGCSR=%x\n",
				   os_readl(U3D_QCR0), os_readl(U3D_QCR1), os_readl(U3D_QCR2),
				   os_readl(U3D_QCR3), os_readl(U3D_QGCSR));
#if defined(CONFIG_USB_MU3D_DRV_36BIT)
		qmu_printk(K_INFO, "[RXD][ERROR] HWO=%d, Next_GPD=%lx ,DataBufLen=%d, DataBuf=%lx\n",
			   (u32) TGPD_GET_FLAG(gpd), (uintptr_t) TGPD_GET_NEXT_RX(gpd),
			   (u32) TGPD_GET_DataBUF_LEN(gpd), (uintptr_t) TGPD_GET_DATA_RX(gpd));
#else
			qmu_printk(K_INFO, "[RXD][ERROR] HWO=%d, Next_GPD=%lx ,DataBufLen=%d, DataBuf=%lx\n",
				   (u32) TGPD_GET_FLAG(gpd), (uintptr_t) TGPD_GET_NEXT(gpd),
				   (u32) TGPD_GET_DataBUF_LEN(gpd), (uintptr_t) TGPD_GET_DATA(gpd));
#endif
			qmu_printk(K_INFO, "[RXD][ERROR] RecvLen=%d, Endpoint=%d\n",
				   (u32) TGPD_GET_BUF_LEN(gpd), (u32) TGPD_GET_EPaddr(gpd));
		}
		return;
	}

	if (!gpd || !gpd_current) {
		qmu_printk(K_ERR,
		   "[RXD][ERROR] %s EP%d, gpd=%p, gpd_current=%p, ishwo=%d, rx_gpd_last=%p, RQCPR=0x%x\n",
		   __func__, ep_num, gpd, gpd_current,
		   ((gpd == NULL) ? 999 : TGPD_IS_FLAGS_HWO(gpd)),
		   Rx_gpd_last[ep_num],
		   os_readl(USB_QMU_RQCPR(ep_num)));
		return;
	}

	if (TGPD_IS_FLAGS_HWO(gpd)) {
		/*
		 * Not an early return, as in the TX copy: the while() below
		 * re-tests !TGPD_IS_FLAGS_HWO(gpd), so it runs zero iterations
		 * and nothing is dereferenced; returning would only drop the
		 * RQCSR/RQSAR/RQCPR/RQLDPR + QCR0-3/QGCSR dump at the bottom.
		 */
		pr_notice_ratelimited(
			"[FORGE_QMU] RXD-HWO ep%d gpd=%p still owned by HW guard=%d (walk is a no-op, falling through to the register dump)\n",
			ep_num, gpd, forge_qmu_guard);
		qmu_printk(K_ERR, "[RXD][ERROR] HWO=1!!\n");
		WARN_ON(1);
	}

	while (gpd != gpd_current && !TGPD_IS_FLAGS_HWO(gpd)) {
		DEV_UINT32 rcv_len = (DEV_UINT32) TGPD_GET_BUF_LEN(gpd);
		DEV_UINT32 buf_len = (DEV_UINT32) TGPD_GET_DataBUF_LEN(gpd);

		if (rcv_len > buf_len)
			qmu_printk(K_ERR, "[RXD][ERROR] %s rcv(%d) > buf(%d) AUK!?\n", __func__,
				   rcv_len, buf_len);
#if defined(CONFIG_USB_MU3D_DRV_36BIT)
		qmu_printk(K_DEBUG,
			   "[RXD] gpd=%p ->HWO=%d, Next_GPD=%p, RcvLen=%d, BufLen=%d, pBuf=%p\n",
			   gpd, TGPD_GET_FLAG(gpd), TGPD_GET_NEXT_RX(gpd), rcv_len, buf_len,
			   TGPD_GET_DATA_RX(gpd));
#else
		qmu_printk(K_DEBUG,
			   "[RXD] gpd=%p ->HWO=%d, Next_GPD=%p, RcvLen=%d, BufLen=%d, pBuf=%p\n",
			   gpd, TGPD_GET_FLAG(gpd), TGPD_GET_NEXT(gpd), rcv_len, buf_len,
			   TGPD_GET_DATA(gpd));
#endif
		request->actual += rcv_len;
#if defined(CONFIG_USB_MU3D_DRV_36BIT)
		if (!TGPD_GET_NEXT_RX(gpd) || !TGPD_GET_DATA_RX(gpd)) {
			/*
			 * Not built: CONFIG_USB_MU3D_DRV_36BIT is not set.
			 * Guarded identically so the two branches cannot drift.
			 */
			pr_notice_ratelimited(
				"[FORGE_QMU] RXD-WALK36 ep%d gpd=%p next=%lx data=%lx guard=%d\n",
				ep_num, gpd, (uintptr_t) TGPD_GET_NEXT_RX(gpd),
				(uintptr_t) TGPD_GET_DATA_RX(gpd), forge_qmu_guard);
			qmu_printk(K_ERR, "[RXD][ERROR] %s EP%d ,gpd=%p\n", __func__, ep_num,
				   gpd);
			WARN_ON(1);
			if (forge_qmu_guard) {
				/* see the non-36BIT arm below for the reasoning */
				musb_g_giveback(musb_ep, request, -EPROTO);
				return;
			}
		}

		gpd = TGPD_GET_NEXT_RX(gpd);
#else
		if (!TGPD_GET_NEXT(gpd) || !TGPD_GET_DATA(gpd)) {
			/*
			 * Falling through would execute
			 * `gpd = TGPD_GET_NEXT(gpd)`, the NULL this check just
			 * found, and two instructions later dereference the
			 * translated wild pointer. Return before the advance: the
			 * descriptor chain is damaged, there is nothing left to
			 * reap on this endpoint this interrupt, and every
			 * remaining statement in the loop body operates on it.
			 * (A break would have the same effect; return states the
			 * intent of the vendor's removed BUG_ON.)
			 */
			pr_notice_ratelimited(
				"[FORGE_QMU] RXD-WALK ep%d gpd=%p next=%lx data=%lx rcv=%d buf=%d RQCSR=%x RQCPR=%x guard=%d\n",
				ep_num, gpd, (uintptr_t) TGPD_GET_NEXT(gpd),
				(uintptr_t) TGPD_GET_DATA(gpd), rcv_len, buf_len,
				os_readl(USB_QMU_RQCSR(ep_num)),
				os_readl(USB_QMU_RQCPR(ep_num)), forge_qmu_guard);
			qmu_printk(K_ERR, "[RXD][ERROR] %s EP%d ,gpd=%p\n", __func__, ep_num,
				   gpd);
			WARN_ON(1);
			if (forge_qmu_guard) {
				/*
				 * `request` is the head of req_list: it was
				 * taken by next_request() at function entry or
				 * at the bottom of the previous iteration, and
				 * has NOT been given back yet in this one, so
				 * it is still queued at -EINPROGRESS.  Complete
				 * it with an error before leaving, otherwise
				 * functionfs waits on it forever (that is what
				 * hung userspace at the end of the boot
				 * animation on the armed 08-18 image).  actual
				 * carries whatever this GPD contributed; f_fs
				 * returns status, not actual, when status != 0.
				 */
				musb_g_giveback(musb_ep, request, -EPROTO);
				return;
			}
		}

		gpd = TGPD_GET_NEXT(gpd);
#endif
		gpd = gpd_phys_to_virt(gpd, USB_RX, ep_num);

		if (!gpd) {
			qmu_printk(K_ERR, "[RXD][ERROR] %s EP%d ,gpd=%p\n", __func__, ep_num,
				   gpd);
			WARN_ON(1);
		}

		/*
		 * The vendor check above cannot fire: gpd_phys_to_virt()
		 * computes p_desc + (paddr - p_desc_dma) and so returns
		 * PAGE_OFFSET - PHYS_OFFSET (0xffffffbfc0000000, in the hole
		 * below TTBR1) for a NULL input, never NULL. Test ring
		 * membership instead, and leave the vendor line untouched so
		 * the disarmed leg keeps the stock behaviour.
		 */
		if (!forge_qmu_gpd_in_ring(gpd, USB_RX, ep_num)) {
			pr_notice_ratelimited(
				"[FORGE_QMU] RXD-XLATE ep%d translated gpd=%p outside ring [%p,%p) guard=%d\n",
				ep_num, gpd, Rx_gpd_List[ep_num < 15 ? ep_num : 0].pStart,
				Rx_gpd_List[ep_num < 15 ? ep_num : 0].pEnd, forge_qmu_guard);
			if (forge_qmu_guard) {
				/*
				 * Same as the walk branch above: `request` is
				 * still the un-given-back head of req_list.
				 * The translated pointer is the wild one, so
				 * Rx_gpd_last[] is deliberately NOT advanced.
				 */
				musb_g_giveback(musb_ep, request, -EPROTO);
				return;
			}
		}

		Rx_gpd_last[ep_num] = gpd;
		musb_g_giveback(musb_ep, request, 0);
		req = next_request(musb_ep);
		/*
		 * The vendor code does `request = &req->request` with no NULL
		 * test. offsetof(struct musb_request, request) is 0, so an
		 * exhausted list silently sets request = NULL and the next
		 * iteration faults on `request->actual += rcv_len`. The same
		 * knob closes it.
		 */
		if (!req) {
			/*
			 * Routine on USB traffic: count every hit, print the
			 * 1st and every 256th.
			 */
			if (++forge_qmu_rxd_noreq == 1 ||
			    (forge_qmu_rxd_noreq % 256) == 0)
				pr_notice("[FORGE_QMU] RXD-NOREQ ep%d request list ran empty mid-walk (gpd=%p) guard=%d count=%u (1 line per 256, musb_hdrc.forge_qmu_rxd_noreq)\n",
					  ep_num, gpd, forge_qmu_guard,
					  forge_qmu_rxd_noreq);
			/*
			 * The ONLY guarded exit with no giveback, and the one
			 * case where that is right: next_request() returned
			 * NULL, i.e. req_list is empty, so there is nothing
			 * left queued to orphan.  The giveback for the request
			 * this iteration reaped already happened four lines up.
			 */
			if (forge_qmu_guard)
				return;
		}
		request = &req->request;
	}

	if (gpd != gpd_current && TGPD_IS_FLAGS_HWO(gpd)) {
		qmu_printk(K_ERR, "[RXD][ERROR] gpd=%p\n", gpd);

		qmu_printk(K_ERR, "[RXD][ERROR] EP%d RQCSR=%x, RQSAR=%x, RQCPR=%x, RQLDPR=%x\n",
			   ep_num, os_readl(USB_QMU_RQCSR(ep_num)), os_readl(USB_QMU_RQSAR(ep_num)),
			   os_readl(USB_QMU_RQCPR(ep_num)), os_readl(USB_QMU_RQLDPR(ep_num)));

		qmu_printk(K_ERR, "[RXD][ERROR] QCR0=%x, QCR1=%x, QCR2=%x, QCR3=%x, QGCSR=%x\n",
			   os_readl(U3D_QCR0), os_readl(U3D_QCR1), os_readl(U3D_QCR2),
			   os_readl(U3D_QCR3), os_readl(U3D_QGCSR));
#if defined(CONFIG_USB_MU3D_DRV_36BIT)
		qmu_printk(K_INFO, "[RXD][ERROR] HWO=%d, Next_GPD=%lx ,DataBufLen=%d, DataBuf=%lx\n",
			   (u32) TGPD_GET_FLAG(gpd), (uintptr_t) TGPD_GET_NEXT_RX(gpd),
			   (u32) TGPD_GET_DataBUF_LEN(gpd), (uintptr_t) TGPD_GET_DATA_RX(gpd));
#else
		qmu_printk(K_INFO, "[RXD][ERROR] HWO=%d, Next_GPD=%lx ,DataBufLen=%d, DataBuf=%lx\n",
			   (u32) TGPD_GET_FLAG(gpd), (uintptr_t) TGPD_GET_NEXT(gpd),
			   (u32) TGPD_GET_DataBUF_LEN(gpd), (uintptr_t) TGPD_GET_DATA(gpd));

#endif
		qmu_printk(K_INFO, "[RXD][ERROR] RecvLen=%d, Endpoint=%d\n",
			   (u32) TGPD_GET_BUF_LEN(gpd), (u32) TGPD_GET_EPaddr(gpd));
	}

	qmu_printk(K_DEBUG, "[RXD] %s EP%d, Last=%p, End=%p, complete\n", __func__,
		   ep_num, Rx_gpd_last[ep_num], Rx_gpd_end[ep_num]);
}

void qmu_done_tasklet(unsigned long data)
{
	unsigned int qmu_val;
	unsigned int i;
	unsigned long flags;
	struct musb *musb = (struct musb *)data;

	spin_lock_irqsave(&musb->lock, flags);

	qmu_val = musb->qmu_done_intr;

	musb->qmu_done_intr = 0;

	for (i = 1; i <= MAX_QMU_EP; i++) {
		if (qmu_val & QMU_RX_DONE(i))
			qmu_done_rx(musb, i, flags);
		if (qmu_val & QMU_TX_DONE(i))
			qmu_done_tx(musb, i, flags);
	}
	spin_unlock_irqrestore(&musb->lock, flags);
}

/* Rescue for a stalled receive queue.
 *
 * The end state of the OUT stall is always the same: ep1 RQCSR=0 (queue not
 * active), hwo=0 (no descriptor armed), Rx_gpd_last == Rx_gpd_end == RQCPR
 * (the driver believes it has reaped everything), and ~20 OUT requests
 * queued and never given back. adbd blocks on those forever, so every
 * later transfer fails too; only `adb reconnect`, which re-enables the
 * endpoint, heals it.
 *
 *  - Re-running the reaping walk cannot help: qmu_done_rx() returns at once
 *    when gpd == gpd_current, which is that state. Hardware consumed all
 *    48 descriptors while fewer done interrupts arrived, so the completions
 *    were lost, not late.
 *
 *  - The requests have to be re-armed, which the vendor's
 *    qmu_error_recovery() body already does (flush, then walk req_list and
 *    insert a fresh GPD per queued request). Its trigger (RXQ_ZLPERR /
 *    RX-EMPTY) is wrong: RX-EMPTY is the normal end-of-queue condition
 *    here.
 *
 * Same remedy, different trigger: this poll looks for the one state that
 * cannot occur in healthy operation - requests outstanding and the queue
 * inactive - and requires it on two consecutive checks, so an idle gap
 * between transfers is never mistaken for a stall. The poll only runs while
 * something is outstanding, and forge_qmu_rxstall_budget caps the rescues
 * per boot. If transfers still fail while RXPOLL-REARM lines appear, the
 * re-armed descriptors are being lost the same way.
 * Disable: `echo 0 > /sys/module/musb_hdrc/parameters/forge_qmu_rxstall`
 * (0644, live).
 */
static int forge_qmu_rxstall = 250;
module_param(forge_qmu_rxstall, int, 0644);
MODULE_PARM_DESC(forge_qmu_rxstall,
		 "m681: poll period in ms for the stalled-RX-queue rescue (0 = off)");

static int forge_qmu_rxstall_budget = 2000;
module_param(forge_qmu_rxstall_budget, int, 0644);
MODULE_PARM_DESC(forge_qmu_rxstall_budget,
		 "m681: stalled-RX rescues still allowed this boot");

static unsigned int forge_qmu_rxstall_hits;
module_param(forge_qmu_rxstall_hits, uint, 0444);
MODULE_PARM_DESC(forge_qmu_rxstall_hits, "m681: stalled-RX rescues performed");

/* musb_gadget.c owns these; see the epstat block there */
extern u32 forge_usb_out_queued[FORGE_USB_NEP];
extern u32 forge_usb_out_done[FORGE_USB_NEP];

/*
 * musb->endpoints[] has MUSB_C_NUM_EPS (9) entries, not FORGE_USB_NEP (16);
 * the counter arrays in musb_gadget.c are the wider ones.
 */
#define FORGE_QMU_RXSTALL_NEP \
	((MUSB_C_NUM_EPS < FORGE_USB_NEP) ? MUSB_C_NUM_EPS : FORGE_USB_NEP)

static u8 forge_qmu_rxstall_strike[FORGE_QMU_RXSTALL_NEP];

static void forge_qmu_rxstall_fn(struct work_struct *w);
static DECLARE_DELAYED_WORK(forge_qmu_rxstall_work, forge_qmu_rxstall_fn);

/*
 * Re-arm every request still queued on one OUT endpoint.  Caller holds
 * musb->lock with interrupts off; this mirrors the RX arm of
 * qmu_error_recovery() above, which runs under the same lock and does the
 * same busy-wait inside _ex_mu3d_hal_flush_qmu().
 */
static void forge_qmu_rx_rearm(struct musb *musb, u8 ep_num)
{
	struct musb_ep *musb_ep = &musb->endpoints[ep_num].ep_out;
	struct musb_request *request;
	unsigned int n = 0;

	_ex_mu3d_hal_flush_qmu(ep_num, USB_RX);

	list_for_each_entry(request, &musb_ep->req_list, list) {
		if (request->request.dma == DMA_ADDR_INVALID)
			continue;
		_ex_mu3d_hal_insert_transfer_gpd(request->epnum, USB_RX,
						 request->request.dma,
						 request->request.length,
						 true, true, false,
						 (musb_ep->type ==
						  USB_ENDPOINT_XFER_ISOC ? 0 : 1),
						 musb_ep->end_point.maxpacket);
		n++;
	}

	mu3d_hal_resume_qmu(ep_num, USB_RX);
	/*
	 * The flush above STOPPED the queue, so RESUME on its own need not
	 * restart it.  mu3d_hal_start_qmu() is idempotent -- it returns early
	 * when the queue is already ACTIVE -- so this cannot double-start.
	 */
	if (!(os_readl(USB_QMU_RQCSR(ep_num)) & QMU_Q_ACTIVE))
		mu3d_hal_start_qmu(ep_num, USB_RX);

	pr_notice("[FORGE_QMU] RXPOLL-REARM ep%d re-armed %u queued request(s) RQCSR=%08x budget=%d\n",
		  ep_num, n, os_readl(USB_QMU_RQCSR(ep_num)),
		  forge_qmu_rxstall_budget);
}

static void forge_qmu_rxstall_fn(struct work_struct *w)
{
	struct musb *musb = _mu3d_musb;
	unsigned long flags;
	int outstanding = 0;
	int ep;

	if (!musb || forge_qmu_rxstall <= 0)
		return;

	spin_lock_irqsave(&musb->lock, flags);
	for (ep = 1; ep < FORGE_QMU_RXSTALL_NEP; ep++) {
		struct musb_ep *musb_ep = &musb->endpoints[ep].ep_out;

		if (!musb_ep->desc || list_empty(&musb_ep->req_list) ||
		    forge_usb_out_queued[ep] == forge_usb_out_done[ep]) {
			forge_qmu_rxstall_strike[ep] = 0;
			continue;
		}

		outstanding = 1;

		if (os_readl(USB_QMU_RQCSR(ep)) & QMU_Q_ACTIVE) {
			forge_qmu_rxstall_strike[ep] = 0;
			continue;
		}
		if (++forge_qmu_rxstall_strike[ep] < 2)
			continue;

		forge_qmu_rxstall_strike[ep] = 0;
		if (forge_qmu_rxstall_budget <= 0)
			continue;
		forge_qmu_rxstall_budget--;
		forge_qmu_rxstall_hits++;
		forge_qmu_rx_rearm(musb, ep);
	}
	spin_unlock_irqrestore(&musb->lock, flags);

	if (outstanding)
		schedule_delayed_work(&forge_qmu_rxstall_work,
				      msecs_to_jiffies(forge_qmu_rxstall));
}

/*
 * Started from musb_gadget_queue() when an OUT request arms a descriptor, so
 * the poll exists only while there is traffic; forge_qmu_rxstall_fn() stops
 * re-arming itself as soon as nothing is outstanding.
 */
void forge_qmu_rxstall_kick(void)
{
	if (forge_qmu_rxstall > 0 &&
	    !delayed_work_pending(&forge_qmu_rxstall_work))
		schedule_delayed_work(&forge_qmu_rxstall_work,
				      msecs_to_jiffies(forge_qmu_rxstall));
}

void qmu_error_recovery(unsigned long data)
{
#ifdef USE_SSUSB_QMU
	u8 ep_num;
	USB_DIR dir;
	unsigned long flags;
	struct musb *musb = (struct musb *)data;
	struct musb_ep *musb_ep;
	struct musb_request *request;
	bool is_len_err = false;
	int i = 0;

	spin_lock_irqsave(&musb->lock, flags);
	ep_num = 0;
	if ((musb->error_wQmuVal & RXQ_CSERR_INT) || (musb->error_wQmuVal & RXQ_LENERR_INT)) {
		dir = USB_RX;
		for (i = 1; i <= MAX_QMU_EP; i++) {
			if (musb->error_wErrVal & QMU_RX_CS_ERR(i)) {
				qmu_printk(K_ERR, "mu3d_hal_resume_qmu Rx %d checksum error!\r\n",
					   i);
				ep_num = i;
				break;
			}

			if (musb->error_wErrVal & QMU_RX_LEN_ERR(i)) {
				qmu_printk(K_ERR, "mu3d_hal_resume_qmu RX EP%d Recv Length error\n",
					   i);
				ep_num = i;
				is_len_err = true;
				break;
			}
		}
	} else if ((musb->error_wQmuVal & TXQ_CSERR_INT) || (musb->error_wQmuVal & TXQ_LENERR_INT)) {
		dir = USB_TX;
		for (i = 1; i <= MAX_QMU_EP; i++) {
			if (musb->error_wErrVal & QMU_TX_CS_ERR(i)) {
				qmu_printk(K_ERR, "mu3d_hal_resume_qmu Tx %d checksum error!\r\n",
					   i);
				ep_num = i;
				break;
			}

			if (musb->error_wErrVal & QMU_TX_LEN_ERR(i)) {
				qmu_printk(K_ERR, "mu3d_hal_resume_qmu TX EP%d Recv Length error\n",
					   i);
				ep_num = i;
				is_len_err = true;
				break;
			}
		}
	} else if (forge_qmu_recover &&
		   (musb->error_wQmuVal & (RXQ_ZLPERR_INT | RXQ_EMPTY_INT))) {
		/*
		 * Without this branch, scheduling error_recovery for a ZLPERR /
		 * RX-EMPTY would be a no-op: the two branches above only decode
		 * CSERR/LENERR, so ep_num would stay 0 and the function would
		 * fall through to "Error but ep_num == 0!". Decoding the
		 * endpoint here lets those latches reuse the existing flush +
		 * re-arm body below. error_wErrVal carries U3D_RQERRIR1 for
		 * ZLPERR and U3D_QEMIR for EMPTY; both use the same BIT16<<n
		 * encoding for the RX side.
		 */
		dir = USB_RX;
		for (i = 1; i <= MAX_QMU_EP; i++) {
			if ((musb->error_wQmuVal & RXQ_ZLPERR_INT) &&
			    (musb->error_wErrVal & QMU_RX_ZLP_ERR(i))) {
				ep_num = i;
				break;
			}
			if ((musb->error_wQmuVal & RXQ_EMPTY_INT) &&
			    (musb->error_wErrVal & QMU_RX_EMPTY(i))) {
				ep_num = i;
				break;
			}
		}
		pr_notice_ratelimited(
			"[FORGE_QMU] RECOVER-RX ep%d from wQmuVal=0x%x wErrVal=0x%x (ZLPERR/RX-EMPTY) recover=%d\n",
			ep_num, musb->error_wQmuVal, musb->error_wErrVal,
			forge_qmu_recover);
	}

	if (ep_num == 0) {
		qmu_printk(K_ERR, "Error but ep_num == 0!\r\n");
		goto done;
	}

	_ex_mu3d_hal_flush_qmu(ep_num, dir);
	/* mu3d_hal_restart_qmu(ep_num, dir); */

	if (dir == USB_TX)
		musb_ep = &musb->endpoints[ep_num].ep_in;
	else
		musb_ep = &musb->endpoints[ep_num].ep_out;

	list_for_each_entry(request, &musb_ep->req_list, list) {
		qmu_printk(K_ERR, "%s : request 0x%p length(%d)\n", __func__, request,
			   request->request.length);

		if (request->request.dma != DMA_ADDR_INVALID) {
			if (request->tx) {
				qmu_printk(K_ERR, "[TX] %s gpd=%p, epnum=%d, len=%d\n", __func__,
					   Tx_gpd_end[ep_num], ep_num, request->request.length);
				request->request.actual = request->request.length;
				if (request->request.length > 0) {
					u32 txcsr;

					if (is_len_err == true) {
						_ex_mu3d_hal_insert_transfer_gpd(request->epnum,
										 USB_TX,
										 request->
										 request.dma, 4096,
										 true, true, false,
										 ((musb_ep->type ==
										   USB_ENDPOINT_XFER_ISOC)
										  ? 0 : 1),
										 musb_ep->
										 end_point.maxpacket);
					} else {
						_ex_mu3d_hal_insert_transfer_gpd(request->epnum,
										 USB_TX,
										 request->
										 request.dma,
										 request->
										 request.length,
										 true, true, false,
										 ((musb_ep->type ==
										   USB_ENDPOINT_XFER_ISOC)
										  ? 0 : 1),
										 musb_ep->
										 end_point.maxpacket);
					}

					/*Enable Tx_DMAREQEN */
					txcsr =
					    USB_ReadCsr32(U3D_TX1CSR0,
							  request->epnum) | TX_DMAREQEN;

					mb();	/* avoid context swtich */

					USB_WriteCsr32(U3D_TX1CSR0, request->epnum, txcsr);

				} else if (request->request.length == 0) {
					qmu_printk(K_DEBUG, "[TX] Send ZLP\n");
				}
			} else {
				qmu_printk(K_ERR, "[RX] %s, gpd=%p, epnum=%d, len=%d\n",
					   __func__, Rx_gpd_end[ep_num], ep_num,
					   request->request.length);
				if (is_len_err == true) {
					_ex_mu3d_hal_insert_transfer_gpd(request->epnum, USB_RX,
									 request->request.dma, 4096,
									 true, true, false,
									 (musb_ep->type ==
									  USB_ENDPOINT_XFER_ISOC ? 0
									  : 1),
									 musb_ep->
									 end_point.maxpacket);
				} else {
					_ex_mu3d_hal_insert_transfer_gpd(request->epnum, USB_RX,
									 request->request.dma,
									 request->request.length,
									 true, true, false,
									 (musb_ep->type ==
									  USB_ENDPOINT_XFER_ISOC ? 0
									  : 1),
									 musb_ep->
									 end_point.maxpacket);
				}
			}
		}
	}

	mu3d_hal_resume_qmu(ep_num, dir);
done:
	spin_unlock_irqrestore(&musb->lock, flags);
#endif
}

void qmu_exception_interrupt(struct musb *musb, DEV_UINT32 wQmuVal)
{
	u32 wErrVal;
	int i = (int)wQmuVal;

	if (wQmuVal & RXQ_CSERR_INT)
		qmu_printk(K_ERR, "==Rx %d checksum error==\n", i);

	if (wQmuVal & RXQ_LENERR_INT)
		qmu_printk(K_ERR, "==Rx %d length error==\n", i);

	if (wQmuVal & TXQ_CSERR_INT)
		qmu_printk(K_ERR, "==Tx %d checksum error==\n", i);

	if (wQmuVal & TXQ_LENERR_INT)
		qmu_printk(K_ERR, "==Tx %d length error==\n", i);

	if ((wQmuVal & RXQ_CSERR_INT) || (wQmuVal & RXQ_LENERR_INT)) {
		wErrVal = os_readl(U3D_RQERRIR0);
		qmu_printk(K_DEBUG, "Rx Queue error in QMU mode![0x%x]\r\n", (unsigned int)wErrVal);
		for (i = 1; i <= MAX_QMU_EP; i++) {
			/* count per-endpoint RX errors, so a stall with errors
			 * is never mistaken for a silent one */
			if ((wErrVal & (QMU_RX_CS_ERR(i) | QMU_RX_LEN_ERR(i))) &&
			    i < FORGE_USB_NEP)
				forge_usb_qmu_err[i]++;
			if (wErrVal & QMU_RX_CS_ERR(i))
				qmu_printk(K_ERR, "Rx %d checksum error!\r\n", i);

			if (wErrVal & QMU_RX_LEN_ERR(i))
				qmu_printk(K_ERR, "RX EP%d Recv Length error\n", i);
		}
		os_writel(U3D_RQERRIR0, wErrVal);
		musb->error_wQmuVal = wQmuVal;
		musb->error_wErrVal = wErrVal;
		tasklet_schedule(&musb->error_recovery);
	}

	if (wQmuVal & RXQ_ZLPERR_INT) {
		wErrVal = os_readl(U3D_RQERRIR1);
		qmu_printk(K_DEBUG, "Rx Queue error in QMU mode![0x%x]\r\n", (unsigned int)wErrVal);
		for (i = 1; i <= MAX_QMU_EP; i++) {
			if (wErrVal & QMU_RX_ZLP_ERR(i)) {
				/* count ZLPERR in qmu_err as well (it was only
				 * printed at K_DEBUG) */
				if (i < FORGE_USB_NEP)
					forge_usb_qmu_err[i]++;
				/*FIXME: should _NOT_ got this error. But now just accept. */
				qmu_printk(K_DEBUG, "RX EP%d Recv ZLP\n", i);
			}
		}
		os_writel(U3D_RQERRIR1, wErrVal);
		/*
		 * The vendor line above is the whole stock handler: the latch
		 * is cleared and the endpoint is left as the ZLP error put it
		 * ("should _NOT_ got this error. But now just accept."). Hand
		 * the two words to qmu_error_recovery, as for RXQ_CSERR.
		 */
		if (wErrVal) {
			pr_notice_ratelimited(
				"[FORGE_QMU] RXQ_ZLPERR RQERRIR1=0x%x recover=%d budget=%d\n",
				(unsigned int)wErrVal, forge_qmu_recover,
				forge_qmu_recover_budget);
			if (forge_qmu_recover_take(1)) {
				musb->error_wQmuVal = wQmuVal;
				musb->error_wErrVal = wErrVal;
				tasklet_schedule(&musb->error_recovery);
			}
		}
	}

	if ((wQmuVal & TXQ_CSERR_INT) || (wQmuVal & TXQ_LENERR_INT)) {
		wErrVal = os_readl(U3D_TQERRIR0);
		qmu_printk(K_DEBUG, "Tx Queue error in QMU mode![0x%x]\r\n", (unsigned int)wErrVal);
		for (i = 1; i <= MAX_QMU_EP; i++) {
			if (wErrVal & QMU_TX_CS_ERR(i))
				qmu_printk(K_ERR, "Tx %d checksum error!\r\n", i);

			if (wErrVal & QMU_TX_LEN_ERR(i))
				qmu_printk(K_ERR, "Tx %d buffer length error!\r\n", i);
		}
		os_writel(U3D_TQERRIR0, wErrVal);
		musb->error_wQmuVal = wQmuVal;
		musb->error_wErrVal = wErrVal;
		tasklet_schedule(&musb->error_recovery);
	}

	if ((wQmuVal & RXQ_EMPTY_INT) || (wQmuVal & TXQ_EMPTY_INT)) {
		DEV_UINT32 wEmptyVal = os_readl(U3D_QEMIR);
		DEV_UINT32 wRxEmpty = 0;

		/* RX-queue-empty on an OUT endpoint: the host sent more than
		 * we had GPDs armed for. It used to be visible only at
		 * K_DEBUG; count it per endpoint, always. */
		for (i = 1; i <= MAX_QMU_EP; i++)
			if (wEmptyVal & QMU_RX_EMPTY(i)) {
				wRxEmpty |= QMU_RX_EMPTY(i);
				if (i < FORGE_USB_NEP)
					forge_usb_qmu_rxempty[i]++;
			}

		qmu_printk(K_DEBUG, "%s Empty in QMU mode![0x%x]\r\n",
			   (wQmuVal & TXQ_EMPTY_INT) ? "TX" : "RX", wEmptyVal);
		os_writel(U3D_QEMIR, wEmptyVal);
		/*
		 * Same treatment as RXQ_ZLPERR above, but only for the RX-empty
		 * bits. TX-empty is left alone: it is a normal end-of-queue
		 * condition on the IN side and flushing on it would be a re-arm
		 * storm.
		 */
		if (wRxEmpty) {
			pr_notice_ratelimited(
				"[FORGE_QMU] RXQ_EMPTY QEMIR=0x%x rx=0x%x recover=%d budget=%d\n",
				(unsigned int)wEmptyVal, (unsigned int)wRxEmpty,
				forge_qmu_recover, forge_qmu_recover_budget);
			if (forge_qmu_recover_take(2)) {
				musb->error_wQmuVal = wQmuVal;
				musb->error_wErrVal = wEmptyVal;
				tasklet_schedule(&musb->error_recovery);
			}
		}
	}
}

#endif
