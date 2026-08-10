/*
 * Copyright (C) 2016 MediaTek Inc.
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

/******************************************************************************
 * pwrap_hal_v1.c - Linux pmic_wrapper Driver,hardware_dependent driver
 *
 *
 * DESCRIPTION:
 *     This file provid the other drivers PMIC wrapper relative functions
 *
 ******************************************************************************/
#include <linux/spinlock.h>
#include <linux/forge_rtc_soc.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/io.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif
#include <mt-plat/mt_pmic_wrap.h>
/* #include <mach/mt_typedefs.h> */
#include "pwrap_hal.h"
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/m3note_board.h>

extern bool forge_pmic_adc_consume(u32 adr, u32 value);

/*
 * Read-only WACS2 on the LK-initialised wrapper. LK brings pwrap up
 * (INIT_DONE2) and the kernel never runs pwrap_init() (no reset/SPI-clock/
 * DIO/cipher/sistrobe re-init). In addition no PMIC write leaves the AP:
 * pwrap_wacs2_hal() drops write commands (counted, first one logged), so
 * every PMIC write is a no-op unless explicitly allowed. Allowed: the
 * touch rail enable bits, VLDO28 CON3/CON4 (see mt_pmic_stub.c), and the
 * owned tokens below. WACS2 is wired only if (a) the infracfg_ao
 * PMIC_TMR/PMIC_AP clock gates read open (STA0 0x10001090 bits 0/1 == 0) and
 * (b) INIT_DONE2 is set; otherwise pwrap_wacs2() keeps failing with -5 and no
 * pwrap register is touched again. FLOG marks: 0xA1 aux 0x01..0x04,
 * 0xA1 aux 0x10|sta (gates closed, pwrap untouched), 0xA1 aux 0x20 (INIT_DONE2
 * clear, WACS2 not wired), 0xA1 aux 0x2F (infracfg_ao not mappable).
 * Telemetry: /sys/module/pwrap_hal_v1/parameters/.
 */
#define FORGE_PWRAP_LOOP_CAP	200000	/* ~40-200 ms of MMIO polls; the 10 ms
					 * sched_clock timeout also stops a loop,
					 * this caps it if CNTVCT were frozen */
static unsigned int forge_pwrap_blocked_writes;
module_param_named(blocked_writes, forge_pwrap_blocked_writes, uint, 0444);
static unsigned int forge_pwrap_last_blocked;	/* adr << 16 | wdata */
module_param_named(last_blocked, forge_pwrap_last_blocked, uint, 0444);
static unsigned int forge_pwrap_reads;
module_param_named(reads, forge_pwrap_reads, uint, 0444);
static unsigned int forge_pwrap_read_fail;
module_param_named(read_fail, forge_pwrap_read_fail, uint, 0444);
static int forge_pwrap_state;	/* 0 not probed, 1 wired, -1 gates closed, -2 no INIT_DONE2 */
module_param_named(state, forge_pwrap_state, int, 0444);
/* RTC recovery flag (below, next to pwrap_hal_init()): a one-write token
 * like the allowlist tokens of mt_pmic_stub.c: its owner task, the
 * expected address and the expected value; armed by forge_rtc_write()
 * under wrp_lock, consumed by that one write, dropped after it in any
 * case. */
static struct task_struct *forge_rtc_owner;
static u32 forge_rtc_exp_adr, forge_rtc_exp_val;

/* under wrp_lock: true once for the owner's expected write, never in IRQ */
static bool forge_rtc_token_consume(u32 adr, u32 wdata)
{
	if (!forge_rtc_owner || forge_rtc_owner != current || in_interrupt() ||
	    adr != forge_rtc_exp_adr || wdata != forge_rtc_exp_val)
		return false;
	forge_rtc_owner = NULL;
	return true;
}
int forge_rtc_recovery_flag(int set);
void forge_m681_mark_aux(u8 stage, u32 aux);	/* init/forge_m681_marker.c */

#define PMIC_WRAP_DEVICE "pmic_wrap"

/*-----start-- global variable-----------------------------------------------*/
#ifdef CONFIG_OF
void __iomem *pwrap_base;
static void __iomem *topckgen_reg_base;
static void __iomem *infracfg_ao_reg_base;
static void __iomem *gpio_reg_base;
static void __iomem *sleep_reg_base;
#endif

static struct mt_pmic_wrap_driver *mt_wrp;

static DEFINE_SPINLOCK(wrp_lock);
static DEFINE_SPINLOCK(wrp_lock_isr);


/*-----start--Internal API---------------------------------------------------*/
static s32 _pwrap_init_dio(u32 dio_en);
static s32 _pwrap_init_cipher(void);
static s32 _pwrap_init_reg_clock(u32 regck_sel);
static s32 _pwrap_wacs2_nochk(u32 write, u32 adr, u32 wdata, u32 *rdata);
static s32 _pwrap_reset_spislv(void);
/*-----end --Internal API---------------------------------------------------*/

#ifdef CONFIG_OF
static int pwrap_of_iomap(void);
static void pwrap_of_iounmap(void);
#endif


#ifdef PMIC_WRAP_NO_PMIC
/*-pwrap debug--------------------------------------------------------------------------*/
void pwrap_dump_all_register(void)
{
}

/*********************************************************************************/
/* extern API for PMIC driver, INT related control, this INT is for PMIC chip to AP */
/*********************************************************************************/
u32 mt_pmic_wrap_eint_status(void)
{
	PWRAPLOG("[PMIC2]first of all PMIC_WRAP_STAUPD_GRPEN=0x%x\n",
		 WRAP_RD32(PMIC_WRAP_STAUPD_GRPEN));
	PWRAPLOG("[PMIC2]first of all PMIC_WRAP_EINT_STA=0x%x\n", WRAP_RD32(PMIC_WRAP_EINT_STA));
	return WRAP_RD32(PMIC_WRAP_EINT_STA);
}

void mt_pmic_wrap_eint_clr(int offset)
{
	if ((offset < 0) || (offset > 3))
		PWRAPERR("clear EINT flag error, only 0-3 bit\n");
	else
		WRAP_WR32(PMIC_WRAP_EINT_CLR, (1 << offset));

}
/* -------------------------------------------------------- */
/* Function : pwrap_wacs2_hal() */
/* Description : */
/* Parameter : */
/* Return : */
/* -------------------------------------------------------- */
static s32 pwrap_wacs2_hal(u32 write, u32 adr, u32 wdata, u32 *rdata)
{
	PWRAPERR("there is no PMIC real chip,PMIC_WRAP do Nothing\n");
	return 0;
}

/*
 *pmic_wrap init,init wrap interface
 *
 */
s32 pwrap_init(void)
{
	PWRAPERR("There is no PMIC real chip, PMIC_WRAP do Nothing.\n");
	return 0;
}

s32 pwrap_read_nochk(u32 adr, u32 *rdata)
{
	return pwrap_wacs2_hal(0, adr, 0, rdata);
}

s32 pwrap_write_nochk(u32 adr, u32 wdata)
{
	return pwrap_wacs2_hal(1, adr, wdata, 0);
}

/*Interrupt handler function*/
static irqreturn_t mt_pmic_wrap_irq(int irqno, void *dev_id)
{
	unsigned long flags = 0;

	PWRAPFUC();
	PWRAPREG("dump pwrap register\n");
	spin_lock_irqsave(&wrp_lock, flags);
	/* *----------------------------------------------------------------------- */
	pwrap_dump_all_register();
	/* raise the priority of WACS2 for AP */
	WRAP_WR32(PMIC_WRAP_HARB_HPRIO, WACS2);

	/* *----------------------------------------------------------------------- */
	/* clear interrupt flag */
	WRAP_WR32(PMIC_WRAP_INT0_CLR, 0xffffffff);
	WRAP_WR32(PMIC_WRAP_INT1_CLR, 0xffffffff);
	PWRAPREG("INT flag 0x%x\n", WRAP_RD32(PMIC_WRAP_INT0_FLG_RAW));
	PWRAPREG("INT flag 0x%x\n", WRAP_RD32(PMIC_WRAP_INT1_FLG_RAW));
	/* BUG_ON(1); */
	spin_unlock_irqrestore(&wrp_lock, flags);
	return IRQ_HANDLED;
}

static u32 pwrap_read_test(void)
{
	return 0;
}

static u32 pwrap_write_test(void)
{
	return 0;
}

static void pwrap_ut(u32 ut_test)
{
	switch (ut_test) {
	case 1:
		/* pwrap_wacs2_para_test(); */
		pwrap_write_test();
		break;
	case 2:
		/* pwrap_wacs2_para_test(); */
		pwrap_read_test();
		break;

	default:
		PWRAPREG("default test.\n");
		break;
	}
}

/*---------------------------------------------------------------------------*/
static s32 mt_pwrap_show_hal(char *buf)
{
	PWRAPFUC();
	return snprintf(buf, PAGE_SIZE, "%s\n", "no implement");
}

/*---------------------------------------------------------------------------*/
static s32 mt_pwrap_store_hal(const char *buf, size_t count)
{
	u32 reg_value = 0;
	u32 reg_addr = 0;
	u32 return_value = 0;
	u32 ut_test = 0;

	if (!strncmp(buf, "-h", 2)) {
		PWRAPREG
		    ("PWRAP debug: [-dump_reg][-trace_wacs2][-init][-rdpmic][-wrpmic][-readtest][-writetest]\n");
		PWRAPREG("PWRAP UT: [1][2]\n");
	}
	/* ----------pwrap debug---------- */
	else if (!strncmp(buf, "-dump_reg", 9)) {
		pwrap_dump_all_register();
	} else if (!strncmp(buf, "-trace_wacs2", 12)) {
		/* pwrap_trace_wacs2(); */
	} else if (!strncmp(buf, "-init", 5)) {
		return_value = pwrap_init();
		if (return_value == 0)
			PWRAPREG("pwrap_init pass,return_value=%d\n", return_value);
		else
			PWRAPREG("pwrap_init fail,return_value=%d\n", return_value);
	} else if (!strncmp(buf, "-rdpmic", 7) && (1 == sscanf(buf + 7, "%x", &reg_addr))) {
		/* pwrap_read_reg_on_pmic(reg_addr); */
	} else if (!strncmp(buf, "-wrpmic", 7)
		   && (2 == sscanf(buf + 7, "%x %x", &reg_addr, &reg_value))) {
		/* pwrap_write_reg_on_pmic(reg_addr,reg_value); */
	} else if (!strncmp(buf, "-readtest", 9)) {
		pwrap_read_test();
	} else if (!strncmp(buf, "-writetest", 10)) {
		pwrap_write_test();
	}
	/* ----------pwrap UT---------- */
	else if (!strncmp(buf, "-ut", 3) && (1 == sscanf(buf + 3, "%d", &ut_test))) {
		pwrap_ut(ut_test);
	} else {
		PWRAPREG("wrong parameter\n");
	}
	return count;
}
/*---------------------------------------------------------------------------*/
#else				/* Else of #ifdef PMIC_WRAP_NO_PMIC */
/*-pwrap debug--------------------------------------------------------------------------*/
static inline void pwrap_dump_ap_register(void)
{
	u32 i = 0;

	PWRAPREG("dump pwrap register, base=0x%p\n", PMIC_WRAP_BASE);
	PWRAPREG("address     :   3 2 1 0    7 6 5 4    B A 9 8    F E D C\n");
	for (i = 0; i <= PMIC_WRAP_REG_RANGE; i += 4) {
		PWRAPREG("offset 0x%.3x:0x%.8x 0x%.8x 0x%.8x 0x%.8x\n", i * 4,
			 WRAP_RD32(PMIC_WRAP_BASE + (i * 4) + 0),
			 WRAP_RD32(PMIC_WRAP_BASE + (i * 4) + 0x4),
			 WRAP_RD32(PMIC_WRAP_BASE + (i * 4) + 0x8),
			 WRAP_RD32(PMIC_WRAP_BASE + (i * 4) + 0xC));
	}
}

static inline void pwrap_dump_pmic_register(void)
{
#if 0
	u32 i = 0;
	u32 reg_addr = 0;
	u32 reg_value = 0;

	PWRAPREG("dump dewrap register\n");
	for (i = 0; i <= 14; i++) {
		reg_addr = (DEW_BASE + i * 4);
		reg_value = pwrap_read_nochk(reg_addr, &reg_value);
		PWRAPREG("0x%x=0x%x\n", reg_addr, reg_value);
	}
#endif
}

void pwrap_dump_all_register(void)
{
	pwrap_dump_ap_register();
	pwrap_dump_pmic_register();
}

/******************************************************************************
pmic wrapper timeout
******************************************************************************/
#define PWRAP_TIMEOUT
/* use the same API name with kernel driver */
/* however,the timeout API in preloader/lk use tick instead of ns */
#ifdef PWRAP_TIMEOUT
static u64 _pwrap_get_current_time(void)
{
	return sched_clock();	/* /TODO: fix me */
}

static bool _pwrap_timeout_ns(u64 start_time_ns, u64 timeout_time_ns)
{
	u64 cur_time = 0;
	u64 elapse_time = 0;

	/* get current tick */
	cur_time = _pwrap_get_current_time();	/* ns */

	/* avoid timer over flow exiting in FPGA env */
	if (cur_time < start_time_ns) {
		PWRAPERR("@@@@Timer overflow! start%lld cur timer%lld\n", start_time_ns, cur_time);
		start_time_ns = cur_time;
		timeout_time_ns = 10000 * 1000;	/* 10000us */
		PWRAPERR("@@@@reset timer! start%lld setting%lld\n", start_time_ns,
			 timeout_time_ns);
	}

	elapse_time = cur_time - start_time_ns;

	/* check if timeout */
	if (timeout_time_ns <= elapse_time) {
		/* timeout */
		PWRAPERR
		    ("@@@@Timeout: elapse time%lld, start%lld , current%lld, setting timer%lld\n",
		     elapse_time, start_time_ns, cur_time, timeout_time_ns);
		return true;
	}
	return false;
}

static u64 _pwrap_time2ns(u64 time_us)
{
	return time_us * 1000;
}

#else
static u64 _pwrap_get_current_time(void)
{
	return 0;
}

static BOOL _pwrap_timeout_ns(u64 start_time_ns, u64 elapse_time)
{
	return false;
}

static u64 _pwrap_time2ns(u64 time_us)
{
	return 0;
}

#endif				/* End of #ifdef PWRAP_TIMEOUT */

/* ##################################################################### */
/* define macro and inline function (for do while loop) */
/* ##################################################################### */
typedef u32(*loop_condition_fp) (u32);	/* define a function pointer */

static inline u32 wait_for_fsm_idle(u32 x)
{
	return GET_WACS0_FSM(x) != WACS_FSM_IDLE;
}

static inline u32 wait_for_fsm_vldclr(u32 x)
{
	return GET_WACS0_FSM(x) != WACS_FSM_WFVLDCLR;
}

static inline u32 wait_for_sync(u32 x)
{
	return GET_SYNC_IDLE0(x) != WACS_SYNC_IDLE;
}

static inline u32 wait_for_idle_and_sync(u32 x)
{
	return (GET_WACS2_FSM(x) != WACS_FSM_IDLE) || (GET_SYNC_IDLE2(x) != WACS_SYNC_IDLE);
}

static inline u32 wait_for_wrap_idle(u32 x)
{
	return (GET_WRAP_FSM(x) != 0x0) || (GET_WRAP_CH_DLE_RESTCNT(x) != 0x0);
}

static inline u32 wait_for_wrap_state_idle(u32 x)
{
	return GET_WRAP_AG_DLE_RESTCNT(x) != 0;
}

static inline u32 wait_for_man_idle_and_noreq(u32 x)
{
	return (GET_MAN_REQ(x) != MAN_FSM_NO_REQ) || (GET_MAN_FSM(x) != MAN_FSM_IDLE);
}

static inline u32 wait_for_man_vldclr(u32 x)
{
	return GET_MAN_FSM(x) != MAN_FSM_WFVLDCLR;
}

static inline u32 wait_for_cipher_ready(u32 x)
{
	return x != 3;
}

static inline u32 wait_for_stdupd_idle(u32 x)
{
	return GET_STAUPD_FSM(x) != 0x0;
}

static inline u32 wait_for_state_ready_init(loop_condition_fp fp, u32 timeout_us,
					    void *wacs_register, u32 *read_reg)
{
	u64 start_time_ns = 0, timeout_ns = 0;
	u32 reg_rdata = 0x0;

	start_time_ns = _pwrap_get_current_time();
	timeout_ns = _pwrap_time2ns(timeout_us);
	do {
		if (_pwrap_timeout_ns(start_time_ns, timeout_ns)) {
			PWRAPLOG("wait_for_state_ready_init timeout when waiting for idle\n");
			return E_PWR_WAIT_IDLE_TIMEOUT;
		}
		reg_rdata = WRAP_RD32(wacs_register);
	} while (fp(reg_rdata));	/* IDLE State */
	if (read_reg)
		*read_reg = reg_rdata;
	return 0;
}

static inline u32 wait_for_state_idle_init(loop_condition_fp fp, u32 timeout_us,
					   void *wacs_register, void *wacs_vldclr_register,
					   u32 *read_reg)
{
	u64 start_time_ns = 0, timeout_ns = 0;
	u32 reg_rdata;

	start_time_ns = _pwrap_get_current_time();
	timeout_ns = _pwrap_time2ns(timeout_us);
	do {
		if (_pwrap_timeout_ns(start_time_ns, timeout_ns)) {
			PWRAPLOG("wait_for_state_idle_init timeout when waiting for idle\n");
			pwrap_dump_ap_register();
			/* pwrap_trace_wacs2(); */
			/* BUG_ON(1); */
			return E_PWR_WAIT_IDLE_TIMEOUT;
		}
		reg_rdata = WRAP_RD32(wacs_register);
		/* if last read command timeout,clear vldclr bit */
		/* read command state machine:FSM_REQ-->wfdle-->WFVLDCLR;write:FSM_REQ-->idle */
		switch (GET_WACS0_FSM(reg_rdata)) {
		case WACS_FSM_WFVLDCLR:
			WRAP_WR32(wacs_vldclr_register, 1);
			PWRAPLOG("WACS_FSM = PMIC_WRAP_WACS_VLDCLR\n");
			break;
		case WACS_FSM_WFDLE:
			PWRAPLOG("WACS_FSM = WACS_FSM_WFDLE\n");
			break;
		case WACS_FSM_REQ:
			PWRAPLOG("WACS_FSM = WACS_FSM_REQ\n");
			break;
		default:
			break;
		}
	} while (fp(reg_rdata));	/* IDLE State */
	if (read_reg)
		*read_reg = reg_rdata;
	return 0;
}

static inline u32 wait_for_state_idle(loop_condition_fp fp, u32 timeout_us, void *wacs_register,
				      void *wacs_vldclr_register, u32 *read_reg)
{
	u64 start_time_ns = 0, timeout_ns = 0;
	u64 loop_counter = 0;
	u32 reg_rdata;

	start_time_ns = _pwrap_get_current_time();
	timeout_ns = _pwrap_time2ns(timeout_us);
	do {
		loop_counter++;
		if (_pwrap_timeout_ns(start_time_ns, timeout_ns) ||
		    loop_counter > FORGE_PWRAP_LOOP_CAP) {
			PWRAPLOG
			    ("wait_for_state_idle timeout when waiting for idle, loop_counter = %lld\n",
			     loop_counter);
			pwrap_dump_ap_register();
			/* pwrap_trace_wacs2(); */
			/* BUG_ON(1); */
			return E_PWR_WAIT_IDLE_TIMEOUT;
		}
		reg_rdata = WRAP_RD32(wacs_register);
		if (GET_INIT_DONE0(reg_rdata) != WACS_INIT_DONE) {
			PWRAPLOG("initialization isn't finished\n");
			return E_PWR_NOT_INIT_DONE;
		}
		/* if last read command timeout,clear vldclr bit */
		/* read command state machine:FSM_REQ-->wfdle-->WFVLDCLR;write:FSM_REQ-->idle */
		switch (GET_WACS0_FSM(reg_rdata)) {
		case WACS_FSM_WFVLDCLR:
			WRAP_WR32(wacs_vldclr_register, 1);
			PWRAPLOG("WACS_FSM = PMIC_WRAP_WACS_VLDCLR\n");
			break;
		case WACS_FSM_WFDLE:
			PWRAPLOG("WACS_FSM = WACS_FSM_WFDLE\n");
			break;
		case WACS_FSM_REQ:
			PWRAPLOG("WACS_FSM = WACS_FSM_REQ\n");
			break;
		default:
			break;
		}
	} while (fp(reg_rdata));	/* IDLE State */
	if (read_reg)
		*read_reg = reg_rdata;
	return 0;
}

static inline u32 wait_for_state_ready(loop_condition_fp fp, u32 timeout_us, void *wacs_register,
				       u32 *read_reg)
{
	u64 start_time_ns = 0, timeout_ns = 0;
	u64 loop_count = 0;
	u32 reg_rdata;

	start_time_ns = _pwrap_get_current_time();
	timeout_ns = _pwrap_time2ns(timeout_us);
	do {
		loop_count++;
		if (_pwrap_timeout_ns(start_time_ns, timeout_ns) ||
		    loop_count > FORGE_PWRAP_LOOP_CAP) {
			PWRAPLOG("timeout when waiting for idle, loop_count = %lld\n", loop_count);
			pwrap_dump_ap_register();
			return E_PWR_WAIT_IDLE_TIMEOUT;
		}
		reg_rdata = WRAP_RD32(wacs_register);

		if (GET_INIT_DONE0(reg_rdata) != WACS_INIT_DONE) {
			PWRAPLOG("initialization isn't finished\n");
			return E_PWR_NOT_INIT_DONE;
		}
	} while (fp(reg_rdata));	/* IDLE State */
	if (read_reg)
		*read_reg = reg_rdata;
	return 0;
}

/********************************************************************************************/
/* extern API for PMIC driver, INT related control, this INT is for PMIC chip to AP */
/********************************************************************************************/
u32 mt_pmic_wrap_eint_status(void)
{
	return WRAP_RD32(PMIC_WRAP_EINT_STA);
}

void mt_pmic_wrap_eint_clr(int offset)
{
	if ((offset < 0) || (offset > 3))
		PWRAPERR("clear EINT flag error, only 0-3 bit\n");
	else
		WRAP_WR32(PMIC_WRAP_EINT_CLR, (1 << offset));

	PWRAPREG("clear EINT flag mt_pmic_wrap_eint_status=0x%x\n", WRAP_RD32(PMIC_WRAP_EINT_STA));
}

/* -------------------------------------------------------- */
/* Function : pwrap_wacs2() */
/* Description : */
/* Parameter : */
/* Return : */
/* -------------------------------------------------------- */
static s32 pwrap_wacs2_hal(u32 write, u32 adr, u32 wdata, u32 *rdata)
{
	u32 reg_rdata = 0;
	u32 wacs_write = 0;
	u32 wacs_adr = 0;
	u32 wacs_cmd = 0;
	u32 return_value = 0;
	unsigned long flags = 0;


	/* Check argument validation */
	if ((write & ~(0x1)) != 0)
		return E_PWR_INVALID_RW;
	if ((adr & ~(0xffff)) != 0)
		return E_PWR_INVALID_ADDR;
	if ((wdata & ~(0xffff)) != 0)
		return E_PWR_INVALID_WDAT;

	spin_lock_irqsave(&wrp_lock, flags);

	/* Read-only WACS2: drop the write, report the first one. The one
	 * exception is the touch rail enable, VLDO28 CON3/CON4 (0x0AA2/0x0AA4);
	 * mt_pmic_stub.c only passes a bit-1 read-modify-write there
	 * (touch_rail=0 drops it again). The owned tokens of
	 * forge_pmic_adc_consume() (mt_pmic_stub.c) pass the battery ADC request
	 * and clocks and the allowlisted field rows (DVFS rail setpoints, connsys
	 * VCN EN/ON_CTRL). */
	if (write && adr != 0x0AA2 && adr != 0x0AA4 &&
	    !forge_rtc_token_consume(adr, wdata) &&
	    !forge_pmic_adc_consume(adr, wdata)) {
		forge_pwrap_blocked_writes++;
		forge_pwrap_last_blocked = (adr << 16) | wdata;
		spin_unlock_irqrestore(&wrp_lock, flags);
		if (forge_pwrap_blocked_writes == 1)
			pr_warn(PWRAPTAG "m681 4.9 read-only WACS2: PMIC write adr=0x%04x wdata=0x%04x dropped (count in /sys/module/pwrap_hal_v1/parameters/blocked_writes)\n",
				adr, wdata);
		/* direct pwrap_write callers (the keywatch) see that it
		 * did not happen. pmic_config_* writes outside the K6 rows are
		 * dropped in mt_pmic_stub.c before they get here (0 returned);
		 * a refused K6 row write returns its error to the caller. */
		return E_PWR_WRITE_BLOCKED;
	}

	/* Check IDLE & INIT_DONE in advance */
	return_value =
	    wait_for_state_idle(wait_for_fsm_idle, TIMEOUT_WAIT_IDLE, PMIC_WRAP_WACS2_RDATA,
				PMIC_WRAP_WACS2_VLDCLR, 0);
	if (return_value != 0) {
		PWRAPLOG("wait_for_fsm_idle fail,return_value=%d\n", return_value);
		goto FAIL;
	}
	wacs_write = write << 31;
	wacs_adr = (adr >> 1) << 16;
	wacs_cmd = wacs_write | wacs_adr | wdata;

	WRAP_WR32(PMIC_WRAP_WACS2_CMD, wacs_cmd);
	if (write == 0) {
		if (NULL == rdata) {
			PWRAPLOG("rdata is a NULL pointer\n");
			return_value = E_PWR_INVALID_ARG;
			WRAP_WR32(PMIC_WRAP_WACS2_VLDCLR, 1);
			goto FAIL;
		}
		return_value =
		    wait_for_state_ready(wait_for_fsm_vldclr, TIMEOUT_READ, PMIC_WRAP_WACS2_RDATA,
					 &reg_rdata);
		if (return_value != 0) {
			PWRAPLOG("wait_for_fsm_vldclr fail,return_value=%d\n", return_value);
			return_value += 1;	/* E_PWR_NOT_INIT_DONE_READ or E_PWR_WAIT_IDLE_TIMEOUT_READ */
			goto FAIL;
		}

		*rdata = GET_WACS0_RDATA(reg_rdata);
		WRAP_WR32(PMIC_WRAP_WACS2_VLDCLR, 1);
	}

FAIL:
	if (return_value != 0)
		forge_pwrap_read_fail++;
	else
		forge_pwrap_reads++;
	spin_unlock_irqrestore(&wrp_lock, flags);

	if (return_value != 0) {
		PWRAPLOG("pwrap_wacs2 fail,return_value=%d\n", return_value);
		PWRAPLOG("timeout:BUG_ON here\n");
	}

	return return_value;
}

/* ****************************************************************************** */
/* --internal API for pwrap_init------------------------------------------------- */
/* ****************************************************************************** */

/* -------------------------------------------------------- */
/* Function : _pwrap_wacs2_nochk() */
/* Description : */
/* Parameter : */
/* Return : */
/* -------------------------------------------------------- */
s32 pwrap_read_nochk(u32 adr, u32 *rdata)
{
	return _pwrap_wacs2_nochk(0, adr, 0, rdata);
}

s32 pwrap_write_nochk(u32 adr, u32 wdata)
{
	return _pwrap_wacs2_nochk(1, adr, wdata, 0);
}

static s32 _pwrap_wacs2_nochk(u32 write, u32 adr, u32 wdata, u32 *rdata)
{
	u32 reg_rdata = 0x0;
	u32 wacs_write = 0x0;
	u32 wacs_adr = 0x0;
	u32 wacs_cmd = 0x0;
	u32 return_value = 0x0;
	/* PWRAPFUC(); */
	/* Check argument validation */
	if ((write & ~(0x1)) != 0)
		return E_PWR_INVALID_RW;
	if ((adr & ~(0xffff)) != 0)
		return E_PWR_INVALID_ADDR;
	if ((wdata & ~(0xffff)) != 0)
		return E_PWR_INVALID_WDAT;

	/* Check IDLE */
	return_value =
	    wait_for_state_ready_init(wait_for_fsm_idle, TIMEOUT_WAIT_IDLE, PMIC_WRAP_WACS2_RDATA,
				      0);
	if (return_value != 0) {
		PWRAPLOG("_pwrap_wacs2_nochk write command fail,return_value=%x\n", return_value);
		return return_value;
	}

	wacs_write = write << 31;
	wacs_adr = (adr >> 1) << 16;
	wacs_cmd = wacs_write | wacs_adr | wdata;
	WRAP_WR32(PMIC_WRAP_WACS2_CMD, wacs_cmd);

	if (write == 0) {
		if (NULL == rdata) {
			PWRAPLOG("rdata is a NULL pointer\n");
			WRAP_WR32(PMIC_WRAP_WACS2_VLDCLR, 1);
			return E_PWR_INVALID_ARG;
		}
		/* wait for read data ready */
		return_value =
		    wait_for_state_ready_init(wait_for_fsm_vldclr, TIMEOUT_READ,
					      PMIC_WRAP_WACS2_RDATA, &reg_rdata);
		if (return_value != 0) {
			PWRAPLOG("wait_for_fsm_vldclr fail, return_value=%d\n", return_value);
			return_value += 1;	/* E_PWR_NOT_INIT_DONE_READ or E_PWR_WAIT_IDLE_TIMEOUT_READ */
			return return_value;
		}
		*rdata = GET_WACS0_RDATA(reg_rdata);
		WRAP_WR32(PMIC_WRAP_WACS2_VLDCLR, 1);
	}
	return 0;
}

static void _pwrap_set_rest(void)
{
	WRAP_WR32(INFRA_GLOBALCON_RST2_SET, 0x1);	/* Write 1 set */
	WRAP_WR32(INFRA_GLOBALCON_RST2_CLR, 0x1);	/* Write 1 clr */
}

static void _pwrap_set_spi_clk(void)
{
#ifdef ULPOSC
	WRAP_WR32(ULPOSC_CON, 0x0001);	/* Enable ulposc */
	udelay(1);
	WRAP_WR32(ULPOSC_CON, 0x0003);	/* Reset ulposc */
	udelay(30);
	WRAP_WR32(ULPOSC_CON, 0x0001);	/* Release reset ulposc */
	udelay(130);
	WRAP_WR32(ULPOSC_CON, 0x0005);
	WRAP_WR32(CLK_CFG_5_SET, 0x0002);	/* 52MHz [1:0] */
	udelay(20);
	WRAP_WR32(PMICW_CLOCK_CTRL, 0x0000);	/* pmicw_sw_en */
	WRAP_WR32(CLK_CFG_UPDATE, 0x00080000);	/* Clock configure update */
	WRAP_WR32(MODULE_SW_CG_0_SET, 0x0000000F);	/* sys_ck cg enable */
	WRAP_WR32(MODULE_SW_CG_0_CLR, 0x0000000F);	/* sys_ck cg disable */
#else
	WRAP_WR32(CLK_CFG_5_CLR, 0x0001);	/* 26MHz [1:0] */
	WRAP_WR32(MODULE_SW_CG_0_SET, 0x0000000F);	/* sys_ck cg enable */
	WRAP_WR32(MODULE_SW_CG_0_CLR, 0x0000000F);	/* sys_ck cg disable */
#endif
}

static void _pwrap_set_DCM(void)
{
	/* S32 sub_return = 0; */

	WRAP_WR32(PMIC_WRAP_DCM_EN, 3);	/* enable CRC DCM and PWRAP DCM */
	WRAP_WR32(PMIC_WRAP_DCM_DBC_PRD, DISABLE);	/* No debounce */
#if 0
	/* 0x10001074 bit[22] = 1??b0 to disable pmic clock DCM */
	sub_return = WRAP_RD32(PMIC_CLOCK_DCM);
	WRAP_WR32(PMIC_CLOCK_DCM, (sub_return & 0xFFBFFFFF));
	/* PWRAPERR("Enable DCM before = %x, after= %x\n", sub_return, WRAP_RD32(PMIC_CLOCK_DCM)); */

	/* 0x10001F0C bit[25] = 1??b1 to disable apb clock gating mechanism. */
	sub_return = WRAP_RD32(APB_CLOCK_GATING);
	WRAP_WR32(APB_CLOCK_GATING, ((sub_return & 0xFDFFFFFF) | 0x02000000));
	/* PWRAPERR("disable apb clock gating before =gg %x, after= %x\n", sub_return, WRAP_RD32(APB_CLOCK_GATING)); */
#endif
}
/* -------------------------------------------------------- */
/* Function : _pwrap_init_dio() */
/* Description :call it in pwrap_init,mustn't check init done */
/* Parameter : */
/* Return : */
/* -------------------------------------------------------- */
static s32 _pwrap_init_dio(u32 dio_en)
{
	u32 arb_en_backup = 0x0;
	u32 rdata = 0x0;
	u32 return_value = 0;

	/* PWRAPFUC(); */
	arb_en_backup = WRAP_RD32(PMIC_WRAP_HIPRIO_ARB_EN);
	WRAP_WR32(PMIC_WRAP_HIPRIO_ARB_EN, WACS2);	/* only WACS2 */

	pwrap_write_nochk(PMIC_DEW_DIO_EN_ADDR, dio_en);

	/* Check IDLE & INIT_DONE in advance */
	return_value =
	    wait_for_state_ready_init(wait_for_idle_and_sync, TIMEOUT_WAIT_IDLE,
				      PMIC_WRAP_WACS2_RDATA, 0);
	if (return_value != 0) {
		PWRAPLOG("_pwrap_init_dio fail,return_value=%x\n", return_value);
		return return_value;
	}
	/* Enable AP DIO mode */
	WRAP_WR32(PMIC_WRAP_DIO_EN, dio_en);
	/* Read Test */

	pwrap_read_nochk(PMIC_DEW_READ_TEST_ADDR, &rdata);
	if (rdata != PMIC_DEFAULT_VALUE_READ_TEST) {
		PWRAPLOG("[Dio_mode][Read Test] fail,dio_en = %x, READ_TEST rdata=%x, exp=0x5aa5\n",
			 dio_en, rdata);
		return E_PWR_READ_TEST_FAIL;
	}

	WRAP_WR32(PMIC_WRAP_HIPRIO_ARB_EN, arb_en_backup);
	return 0;
}

/* -------------------------------------------------------- */
/* Function : _pwrap_init_cipher() */
/* Description : */
/* Parameter : */
/* Return : */
/* -------------------------------------------------------- */
static s32 _pwrap_init_cipher(void)
{
	u32 arb_en_backup = 0;
	u32 rdata = 0;
	u32 return_value = 0;
	u32 start_time_ns = 0, timeout_ns = 0;
	/* PWRAPFUC(); */
	arb_en_backup = WRAP_RD32(PMIC_WRAP_HIPRIO_ARB_EN);

	WRAP_WR32(PMIC_WRAP_HIPRIO_ARB_EN, WACS2);	/* only WACS0 */

	WRAP_WR32(PMIC_WRAP_CIPHER_SWRST, 1);
	WRAP_WR32(PMIC_WRAP_CIPHER_SWRST, 0);
	WRAP_WR32(PMIC_WRAP_CIPHER_KEY_SEL, 1);
	WRAP_WR32(PMIC_WRAP_CIPHER_IV_SEL, 2);
	WRAP_WR32(PMIC_WRAP_CIPHER_EN, 1);

	/* Config CIPHER @ PMIC */
	pwrap_write_nochk(PMIC_DEW_CIPHER_SWRST_ADDR, 0x1);
	pwrap_write_nochk(PMIC_DEW_CIPHER_SWRST_ADDR, 0x0);
	pwrap_write_nochk(PMIC_DEW_CIPHER_KEY_SEL_ADDR, 0x1);
	pwrap_write_nochk(PMIC_DEW_CIPHER_IV_SEL_ADDR, 0x2);
	pwrap_write_nochk(PMIC_DEW_CIPHER_EN_ADDR, 0x1);


	/* wait for cipher data ready@AP */
#ifndef CONFIG_MTK_FPGA
	return_value =
	    wait_for_state_ready_init(wait_for_cipher_ready, TIMEOUT_WAIT_IDLE,
				      PMIC_WRAP_CIPHER_RDY, 0);
#else
	return_value =
	    wait_for_state_ready_init(wait_for_cipher_ready, 0xFFFFFFFF, PMIC_WRAP_CIPHER_RDY, 0);
#endif
	if (return_value != 0) {
		PWRAPLOG("wait for cipher data ready@AP fail,return_value=%x\n", return_value);
		return return_value;
	}
	/* wait for cipher data ready@PMIC */
	/* Bounded: with the MT6353 pwrap HAL on MT6351 hardware
	 * PMIC_DEW_CIPHER_RDY never asserts, and the original code only logged
	 * and never broke out, hanging postcore_initcall. Return
	 * E_PWR_INIT_CIPHER on timeout so pwrap_hal_init propagates the error
	 * (postcore_initcall tolerates it); wacs2_hal is already set above, so
	 * the boot continues with un-encrypted pwrap. */
	start_time_ns = _pwrap_get_current_time();
	timeout_ns = _pwrap_time2ns(0xFFFFFF);
	do {
		if (_pwrap_timeout_ns(start_time_ns, timeout_ns)) {
			PWRAPERR("M681_PWRAP: cipher_ready@PMIC TIMEOUT (likely MT6351/MT6353 key mismatch) rdata=0x%x\n",
				 rdata);
			return E_PWR_INIT_CIPHER;
		}
		pwrap_read_nochk(PMIC_DEW_CIPHER_RDY_ADDR, &rdata);
	} while (rdata != 0x1);	/* cipher_ready */

	pwrap_write_nochk(PMIC_DEW_CIPHER_MODE_ADDR, 0x1);

	/* wait for cipher mode idle */
	return_value =
	    wait_for_state_ready_init(wait_for_idle_and_sync, TIMEOUT_WAIT_IDLE,
				      PMIC_WRAP_WACS2_RDATA, 0);
	if (return_value != 0) {
		PWRAPLOG("wait for cipher mode idle fail,return_value=%x\n", return_value);
		return return_value;
	}
	WRAP_WR32(PMIC_WRAP_CIPHER_MODE, 1);

	/* Read Test */
	pwrap_read_nochk(PMIC_DEW_READ_TEST_ADDR, &rdata);
	if (rdata != PMIC_DEFAULT_VALUE_READ_TEST) {
		PWRAPLOG("_pwrap_init_cipher,read test error,error code=%x, rdata=%x\n", 1, rdata);
		return E_PWR_READ_TEST_FAIL;
	}

	WRAP_WR32(PMIC_WRAP_HIPRIO_ARB_EN, arb_en_backup);
	return 0;
}

/* -------------------------------------------------------- */
/* Function : _pwrap_init_sistrobe() */
/* Description : Initialize SI_CK_CON and SIDLY */
/* Parameter : */
/* Return : */
/* -------------------------------------------------------- */
static s32 _pwrap_init_sistrobe(void)
{
	u32 arb_en_backup = 0;
	u32 rdata = 0;
	s32 ind = 0;
	u32 result_faulty = 0;
	u64 result = 0, tmp1 = 0, tmp2 = 0;
	s32 leading_one = 0;
	s32 tailing_one = 0;
	u32 ret = 0;
#ifdef ULPOSC
	u64 inds = 0, indf = 0;
#endif
	arb_en_backup = WRAP_RD32(PMIC_WRAP_HIPRIO_ARB_EN);

	WRAP_WR32(PMIC_WRAP_HIPRIO_ARB_EN, WACS2);	/* only WACS2 */

#ifdef ULPOSC
	/* --------------------------------------------------------------------- */
	/* Scan all possible input strobe by READ_TEST */
	/* --------------------------------------------------------------------- */
	result = 0;
	result_faulty = 0;
	inds = 0;
	indf = 0;
	for (ind = 0; ind < 44; ind++) {	/* 44 sampling clock edge */
		WRAP_WR32(PMIC_WRAP_SI_CK_CON, (ind >> 2) & 0xf);
		WRAP_WR32(PMIC_WRAP_SIDLY, 0x3 - (ind & 0x3));

		_pwrap_wacs2_nochk(0, PMIC_DEW_READ_TEST_ADDR, 0, &rdata);
		if (rdata == PMIC_DEFAULT_VALUE_READ_TEST) {
			PWRAPERR
			    ("_pwrap_init_sistrobe [Read Test of %s] pass,index=%d rdata=%x\n",
			     PMIC_CHIP_STR, ind, rdata);
			result |= (0x1 << ind);
		} else {
			PWRAPERR
			    ("_pwrap_init_sistrobe [Read Test of %s] tuning,index=%d rdata=%x\n",
			     PMIC_CHIP_STR, ind, rdata);
		}
	}

	/* --------------------------------------------------------------------- */
	/* Locate the leading one and trailing one */
	/* --------------------------------------------------------------------- */
	for (ind = 43; ind >= 0; ind--) {
		if (ind > 20) {
			indf = (0x1 << 20);
			indf = (indf << ((ind - 20)%32));
		} else {
			indf = (0x1 << (ind % 32));
		}
		inds = result & indf;
		if (inds)
			break;
	}
	leading_one = ind;

	for (ind = 0; ind < 44; ind++) {
		if (ind > 20) {
			indf = (0x1 << 20);
			indf = (indf << ((ind - 20)%32));
		} else {
			indf = (0x1 << (ind % 32));
		}
		inds = result & indf;
		if (inds)
			break;
	}
	tailing_one = ind;

	/* --------------------------------------------------------------------- */
	/* Check the continuity of pass range */
	/* --------------------------------------------------------------------- */
	tmp1 = (0x1 << (leading_one + 1)) - 1;
	tmp2 = (0x1 << tailing_one) - 1;
	if ((tmp1 - tmp2) != result) {
		PWRAPERR("_pwrap_init_sistrobe Fail, result = %llx, leading_one:%d, tailing_one:%d\n",
		     result, leading_one, tailing_one);
		result_faulty = 0x1;
	}
	/* --------------------------------------------------------------------- */
	/* Config SICK and SIDLY to the middle point of pass range */
	/* --------------------------------------------------------------------- */
	ind = (leading_one + tailing_one) / 2;
	WRAP_WR32(PMIC_WRAP_SI_CK_CON, (ind >> 2) & 0xf);
	WRAP_WR32(PMIC_WRAP_SIDLY, 0x3 - (ind & 0x3));

#else				/* Else of #ifdef ULPOSC */

	/* --------------------------------------------------------------------- */
	/* Scan all possible input strobe by READ_TEST */
	/* --------------------------------------------------------------------- */
	for (ind = 0; ind < 24; ind++) {	/* 24 sampling clock edge */
		WRAP_WR32(PMIC_WRAP_SI_CK_CON, (ind >> 2) & 0x7);
		WRAP_WR32(PMIC_WRAP_SIDLY, 0x3 - (ind & 0x3));

		_pwrap_wacs2_nochk(0, PMIC_DEW_READ_TEST_ADDR, 0, &rdata);
		if (rdata == PMIC_DEFAULT_VALUE_READ_TEST) {
			PWRAPERR("_pwrap_init_sistrobe [Read Test of %s] pass,index=%d rdata=%x\n",
			     PMIC_CHIP_STR, ind, rdata);
			result |= (0x1 << ind);
		} else {
			PWRAPERR
			    ("_pwrap_init_sistrobe [Read Test of %s] tuning,index=%d rdata=%x\n",
			     PMIC_CHIP_STR, ind, rdata);
		}
	}

	/* --------------------------------------------------------------------- */
	/* Locate the leading one and trailing one of PMIC 1/2 */
	/* --------------------------------------------------------------------- */
	for (ind = 23; ind >= 0; ind--) {
		if (result & (0x1 << ind))
			break;

	}
	leading_one = ind;

	for (ind = 0; ind < 24; ind++) {
		if (result & (0x1 << ind))
			break;
	}
	tailing_one = ind;

	/* --------------------------------------------------------------------- */
	/* Check the continuity of pass range */
	/* --------------------------------------------------------------------- */
	tmp1 = (0x1 << (leading_one + 1)) - 1;
	tmp2 = (0x1 << tailing_one) - 1;
	if ((tmp1 - tmp2) != result) {
		PWRAPERR("_pwrap_init_sistrobe Fail, result = %llx, leading_one:%d, tailing_one:%d\n",
		     result, leading_one, tailing_one);
		result_faulty = 0x1;
	}
	/* --------------------------------------------------------------------- */
	/* Config SICK and SIDLY to the middle point of pass range */
	/* --------------------------------------------------------------------- */
	if (result_faulty == 0) {
		ind = (leading_one + tailing_one) / 2;
		WRAP_WR32(PMIC_WRAP_SI_CK_CON, (ind >> 2) & 0x7);
		WRAP_WR32(PMIC_WRAP_SIDLY, 0x3 - (ind & 0x3));
		/* --------------------------------------------------------------------- */
		/* Restore */
		/* --------------------------------------------------------------------- */
		WRAP_WR32(PMIC_WRAP_HIPRIO_ARB_EN, arb_en_backup);
	} else {
		PWRAPERR("_pwrap_init_sistrobe Fail,result_faulty=%x\n", result_faulty);
		ret = result_faulty;
	}
	return ret;
#endif				/* End of #ifdef ULPOSC */
}

/* -------------------------------------------------------- */
/* Function : _pwrap_reset_spislv() */
/* Description : */
/* Parameter : */
/* Return : */
/* -------------------------------------------------------- */
static s32 _pwrap_reset_spislv(void)
{
	u32 ret = 0;
	u32 return_value = 0;
	/* PWRAPFUC(); */
	/* This driver does not using _pwrap_switch_mux */
	/* because the remaining requests are expected to fail anyway */

	WRAP_WR32(PMIC_WRAP_HIPRIO_ARB_EN, DISABLE_ALL);
	WRAP_WR32(PMIC_WRAP_WRAP_EN, DISABLE);
	WRAP_WR32(PMIC_WRAP_MUX_SEL, MANUAL_MODE);
	WRAP_WR32(PMIC_WRAP_MAN_EN, ENABLE);
	WRAP_WR32(PMIC_WRAP_DIO_EN, DISABLE);

	WRAP_WR32(PMIC_WRAP_MAN_CMD, (OP_WR << 13) | (OP_CSL << 8));	/* 0x2100 */
	WRAP_WR32(PMIC_WRAP_MAN_CMD, (OP_WR << 13) | (OP_OUTS << 8));	/* 0x2800//to reset counter */
	WRAP_WR32(PMIC_WRAP_MAN_CMD, (OP_WR << 13) | (OP_CSH << 8));	/* 0x2000 */
	WRAP_WR32(PMIC_WRAP_MAN_CMD, (OP_WR << 13) | (OP_OUTS << 8));
	WRAP_WR32(PMIC_WRAP_MAN_CMD, (OP_WR << 13) | (OP_OUTS << 8));
	WRAP_WR32(PMIC_WRAP_MAN_CMD, (OP_WR << 13) | (OP_OUTS << 8));
	WRAP_WR32(PMIC_WRAP_MAN_CMD, (OP_WR << 13) | (OP_OUTS << 8));

	return_value =
	    wait_for_state_ready_init(wait_for_sync, TIMEOUT_WAIT_IDLE, PMIC_WRAP_WACS2_RDATA, 0);
	if (return_value != 0) {
		PWRAPLOG("_pwrap_reset_spislv fail,return_value=%x\n", return_value);
		ret = E_PWR_TIMEOUT;
		goto timeout;
	}

	WRAP_WR32(PMIC_WRAP_MAN_EN, DISABLE);
	WRAP_WR32(PMIC_WRAP_MUX_SEL, WRAPPER_MODE);

timeout:
	WRAP_WR32(PMIC_WRAP_MAN_EN, DISABLE);
	WRAP_WR32(PMIC_WRAP_MUX_SEL, WRAPPER_MODE);
	return ret;
}

#if 0
static void __pwrap_soft_reset(void)
{
	PWRAPLOG("start reset wrapper\n");
	PWRAP_SOFT_RESET;
	PWRAPLOG("the reset register =%x\n", WRAP_RD32(INFRA_GLOBALCON_RST0));
	PWRAPLOG("PMIC_WRAP_STAUPD_GRPEN =0x%x,it should be equal to 0xc\n",
		 WRAP_RD32(PMIC_WRAP_STAUPD_GRPEN));
	/* clear reset bit */
	PWRAP_CLEAR_SOFT_RESET_BIT;
}
#endif

static s32 _pwrap_init_reg_clock(u32 regck_sel)
{
	PWRAPFUC();

	pwrap_write_nochk(PMIC_DEW_RDDMY_NO_ADDR, 0x8);
	WRAP_WR32(PMIC_WRAP_RDDMY, 0x08);

	/* Config SPI Waveform according to reg clk */
	if (regck_sel == 1) {	/* 18MHz in 6328, 26Mhz in BBCHip */
		WRAP_WR32(PMIC_WRAP_CSHEXT_READ, 0x0);
		/* Wait data written into register =>
		   4T_PMIC: consists of CSHEXT_WRITE_START+EXT_CK+CSHEXT_WRITE_END+CSLEXT_START */
		WRAP_WR32(PMIC_WRAP_CSHEXT_WRITE, 0x33);
		WRAP_WR32(PMIC_WRAP_CSLEXT_START, 0x0);
		WRAP_WR32(PMIC_WRAP_CSLEXT_END, 0x0);	/* for PMIC site synchronizer use */
	} else {		/* Safe mode */
		WRAP_WR32(PMIC_WRAP_CSHEXT_WRITE, 0xff);
		WRAP_WR32(PMIC_WRAP_CSHEXT_READ, 0xff);
		WRAP_WR32(PMIC_WRAP_CSLEXT_START, 0xf);
		WRAP_WR32(PMIC_WRAP_CSLEXT_END, 0xf);
	}

	return 0;
}

static void _pwrap_init_status_update(void)
{
#if 0
	sub_return = pwrap_write_nochk(PMIC_DEW_CRC_EN_ADDR, ENABLE);
	if (sub_return != 0) {
		PWRAPLOG("Enable CRC fail, return=%x\n", sub_return);
		return E_PWR_INIT_ENABLE_CRC;
	}
	WRAP_WR32(PMIC_WRAP_WACS2_EN, PMIC_DEW_CRC_VAL_ADDR);
#endif
	WRAP_WR32(PMIC_WRAP_STAUPD_GRPEN, 0x75);
#if 0
	WRAP_WR32(PMIC_WRAP_CRC_EN, ENABLE);
#endif
	WRAP_WR32(PMIC_WRAP_EINT_STA0_ADR, PMIC_CPU_INT_STA_ADDR);

	WRAP_WR32(PMIC_WRAP_ADC_CMD_ADDR, PMIC_LDO_LDO_RSV1_RO_ADDR);
	WRAP_WR32(PMIC_WRAP_PWRAP_ADC_CMD, 0x0100);
	WRAP_WR32(PMIC_WRAP_ADC_RDATA_ADDR, PMIC_AUXADC_ADC_OUT_MDRT_ADDR);
	WRAP_WR32(PMIC_WRAP_MD_ADC_RDATA_ADDR_LATEST, PMIC_AUXADC_ADC_OUT_MDRT_ADDR);
	WRAP_WR32(PMIC_WRAP_MD_ADC_RDATA_ADDR_WP, PMIC_AUXADC_MDBG_W_PTR_ADDR);
	WRAP_WR32(PMIC_WRAP_MD_ADC_RDATA_ADDR0, PMIC_AUXADC_BUF_OUT_00_ADDR);
	WRAP_WR32(PMIC_WRAP_MD_ADC_RDATA_ADDR1, PMIC_AUXADC_BUF_OUT_01_ADDR);
	WRAP_WR32(PMIC_WRAP_MD_ADC_RDATA_ADDR2, PMIC_AUXADC_BUF_OUT_02_ADDR);
	WRAP_WR32(PMIC_WRAP_MD_ADC_RDATA_ADDR3, PMIC_AUXADC_BUF_OUT_03_ADDR);
	WRAP_WR32(PMIC_WRAP_MD_ADC_RDATA_ADDR4, PMIC_AUXADC_BUF_OUT_04_ADDR);
	WRAP_WR32(PMIC_WRAP_MD_ADC_RDATA_ADDR5, PMIC_AUXADC_BUF_OUT_05_ADDR);
	WRAP_WR32(PMIC_WRAP_MD_ADC_RDATA_ADDR6, PMIC_AUXADC_BUF_OUT_06_ADDR);
	WRAP_WR32(PMIC_WRAP_MD_ADC_RDATA_ADDR7, PMIC_AUXADC_BUF_OUT_07_ADDR);
	WRAP_WR32(PMIC_WRAP_MD_ADC_RDATA_ADDR8, PMIC_AUXADC_BUF_OUT_08_ADDR);
	WRAP_WR32(PMIC_WRAP_MD_ADC_RDATA_ADDR9, PMIC_AUXADC_BUF_OUT_09_ADDR);
	WRAP_WR32(PMIC_WRAP_MD_ADC_RDATA_ADDR10, PMIC_AUXADC_BUF_OUT_10_ADDR);
	WRAP_WR32(PMIC_WRAP_MD_ADC_RDATA_ADDR11, PMIC_AUXADC_BUF_OUT_11_ADDR);
	WRAP_WR32(PMIC_WRAP_MD_ADC_RDATA_ADDR12, PMIC_AUXADC_BUF_OUT_12_ADDR);
	WRAP_WR32(PMIC_WRAP_MD_ADC_RDATA_ADDR13, PMIC_AUXADC_BUF_OUT_13_ADDR);
	WRAP_WR32(PMIC_WRAP_MD_ADC_RDATA_ADDR14, PMIC_AUXADC_BUF_OUT_14_ADDR);
	WRAP_WR32(PMIC_WRAP_MD_ADC_RDATA_ADDR15, PMIC_AUXADC_BUF_OUT_15_ADDR);
	WRAP_WR32(PMIC_WRAP_MD_ADC_RDATA_ADDR16, PMIC_AUXADC_BUF_OUT_16_ADDR);
	WRAP_WR32(PMIC_WRAP_MD_ADC_RDATA_ADDR17, PMIC_AUXADC_BUF_OUT_17_ADDR);
	WRAP_WR32(PMIC_WRAP_MD_ADC_RDATA_ADDR18, PMIC_AUXADC_BUF_OUT_18_ADDR);
	WRAP_WR32(PMIC_WRAP_MD_ADC_RDATA_ADDR19, PMIC_AUXADC_BUF_OUT_19_ADDR);
	WRAP_WR32(PMIC_WRAP_MD_ADC_RDATA_ADDR20, PMIC_AUXADC_BUF_OUT_20_ADDR);
	WRAP_WR32(PMIC_WRAP_MD_ADC_RDATA_ADDR21, PMIC_AUXADC_BUF_OUT_21_ADDR);
	WRAP_WR32(PMIC_WRAP_MD_ADC_RDATA_ADDR22, PMIC_AUXADC_BUF_OUT_22_ADDR);
	WRAP_WR32(PMIC_WRAP_MD_ADC_RDATA_ADDR23, PMIC_AUXADC_BUF_OUT_23_ADDR);
	WRAP_WR32(PMIC_WRAP_MD_ADC_RDATA_ADDR24, PMIC_AUXADC_BUF_OUT_24_ADDR);
	WRAP_WR32(PMIC_WRAP_MD_ADC_RDATA_ADDR25, PMIC_AUXADC_BUF_OUT_25_ADDR);
	WRAP_WR32(PMIC_WRAP_MD_ADC_RDATA_ADDR26, PMIC_AUXADC_BUF_OUT_26_ADDR);
	WRAP_WR32(PMIC_WRAP_MD_ADC_RDATA_ADDR27, PMIC_AUXADC_BUF_OUT_27_ADDR);
	WRAP_WR32(PMIC_WRAP_MD_ADC_RDATA_ADDR28, PMIC_AUXADC_BUF_OUT_28_ADDR);
	WRAP_WR32(PMIC_WRAP_MD_ADC_RDATA_ADDR29, PMIC_AUXADC_BUF_OUT_29_ADDR);
	WRAP_WR32(PMIC_WRAP_MD_ADC_RDATA_ADDR30, PMIC_AUXADC_BUF_OUT_30_ADDR);
	WRAP_WR32(PMIC_WRAP_MD_ADC_RDATA_ADDR31, PMIC_AUXADC_BUF_OUT_31_ADDR);
}

static void _pwrap_adjust_priority(void)
{
	WRAP_WR32(PMIC_WRAP_PRIORITY_USER_SEL_0, 0x6543C210);
	WRAP_WR32(PMIC_WRAP_PRIORITY_USER_SEL_1, 0xFEDBA987);
	WRAP_WR32(PMIC_WRAP_ARBITER_OUT_SEL_0, 0x87654210);
	WRAP_WR32(PMIC_WRAP_ARBITER_OUT_SEL_1, 0xFED3CBA9);
}

static void _pwrap_set_starvation(void)
{
	/* latency is smaller than 10us, pop up to group"1" first */
	WRAP_WR32(PMIC_WRAP_HARB_HPRIO, 0x0007);	/* [0]:MD_HW [1]:C2K_HW [2]:MD_DVFS _HW */
	/* starvation enable */
	/* [0]=MD_HW, latency= 2.6us, target= (2.6/1.2)=2 */
	WRAP_WR32(PMIC_WRAP_STARV_COUNTER_0, 0x0402);	/* [10]: enable starvation, [9:0]: target setting */
	/* [1]=C2K_HW, latency= 2.6us, target= (2.6/1.2)=2 */
	WRAP_WR32(PMIC_WRAP_STARV_COUNTER_1, 0x0402);	/* [10]: enable starvation, [9:0]: target setting */
	/* [2]=MD_DVFS_HW,latency= 5.2us, target= (5.2/1.2)-1=3 */
	WRAP_WR32(PMIC_WRAP_STARV_COUNTER_2, 0x0403);	/* [10]: enable starvation, [9:0]: target setting */
	/* [3]=WACS1(MD_SW1), latency=20us, target= (20/1.2)-1=25 */
	WRAP_WR32(PMIC_WRAP_STARV_COUNTER_3, 0x0414);	/* [10]: enable starvation, [9:0]: target setting */
	/* [4]=WACS0(MD_SW0), latency=40us, target= (40/1.2)-1=32 */
	WRAP_WR32(PMIC_WRAP_STARV_COUNTER_4, 0x0420);	/* [10]: enable starvation, [9:0]: target setting */
	/* [5]=SPM_HW, latency=40us, target= (40/1.2)-1=32 */
	WRAP_WR32(PMIC_WRAP_STARV_COUNTER_5, 0x0420);	/* [10]: enable starvation, [9:0]: target setting */
	/* [6]=WACS3(C2K_SW), latency=40us, target= (40/1.2)-1=32 */
	WRAP_WR32(PMIC_WRAP_STARV_COUNTER_6, 0x0420);	/* [10]: enable starvation, [9:0]: target setting */
	/* [7]=DCXO_CONNINF, latency=50us, target= (50/1.2)-1=40 */
	WRAP_WR32(PMIC_WRAP_STARV_COUNTER_7, 0x0428);	/* [10]: enable starvation, [9:0]: target setting */
	/* [8]=DCXO_NFCINF, latency=50us, target= (50/1.2)-1=40 */
	WRAP_WR32(PMIC_WRAP_STARV_COUNTER_8, 0x0428);	/* [10]: enable starvation, [9:0]: target setting */
	/* [9]=MD_ADCINF, latency=29us, target= (29/1.2)-1= 23 */
	WRAP_WR32(PMIC_WRAP_STARV_COUNTER_9, 0x0417);	/* [10]: enable starvation, [9:0]: target setting */
	/* [10]=STAUPD, latency=428us, target= (428/1.2)-1=355 */
	WRAP_WR32(PMIC_WRAP_STARV_COUNTER_10, 0x0563);	/* [10]: enable starvation, [9:0]: target setting */
	/* [11]=GPS_HW, latency=150us, target= (150/1.2)-1=124 */
	WRAP_WR32(PMIC_WRAP_STARV_COUNTER_11, 0x047C);	/* [10]: enable starvation, [9:0]: target setting */
	/* [12]=WACS2(AP_SW), latency=1000us, target= (1000/1.2)-1=832 */
	WRAP_WR32(PMIC_WRAP_STARV_COUNTER_12, 0x0740);	/* [10]: enable starvation, [9:0]: target setting */
}

static void _pwrap_set_en(void)
{
	WRAP_WR32(PMIC_WRAP_HIPRIO_ARB_EN, 0x01fff);
	WRAP_WR32(PMIC_WRAP_WACS0_EN, ENABLE);
	WRAP_WR32(PMIC_WRAP_WACS1_EN, ENABLE);
	WRAP_WR32(PMIC_WRAP_WACS3_EN, ENABLE);
	WRAP_WR32(PMIC_WRAP_STAUPD_PRD, 0x5);	/* 100us */
	WRAP_WR32(PMIC_WRAP_WDT_UNIT, 0xf);
	WRAP_WR32(PMIC_WRAP_WDT_SRC_EN, 0xffffffff);
	WRAP_WR32(PMIC_WRAP_TIMER_EN, 0x1);
	/* WRAP_WR32(PMIC_WRAP_INT0_EN, 0xffffffff); */
	WRAP_WR32(PMIC_WRAP_INT0_EN, 0xfffffffD);	/* FIX ME */
	WRAP_WR32(PMIC_WRAP_INT1_EN, 0x0001ffff);
}

static void _pwrap_set_init_done(void)
{
	WRAP_WR32(PMIC_WRAP_INIT_DONE0, ENABLE);	/* MD SW0 */
	WRAP_WR32(PMIC_WRAP_INIT_DONE1, ENABLE);	/* MD SW1 */
	WRAP_WR32(PMIC_WRAP_INIT_DONE2, ENABLE);	/* AP SW */
	WRAP_WR32(PMIC_WRAP_INIT_DONE3, ENABLE);	/* C2K SW */
}
/*
 * pmic_wrap init,init wrap interface
 */
s32 pwrap_init(void)
{
	s32 sub_return = 0;
	s32 sub_return1 = 0;
	u32 rdata = 0x0;
	/* DRAM breadcrumbs and the MT6351-vs-MT6353 probe. The pr_emerg markers
	 * below only survive if printk reaches the console ring; on an early
	 * postcore hang that never flushes. forge_m681_mark_aux() writes the
	 * DRAM marker slots directly (no printk), so a hang is still localised
	 * from recovery. */
	extern void forge_m681_mark_aux(u8 stage, u32 aux);
	extern void forge_m681_diag(unsigned int off, u32 val);
	u32 dew_swcid = 0xdead, dew_rt6353 = 0xdead, dew_rt6351 = 0xdead;

	/* M681_PWRAP step markers on the console: if the kernel hangs, the last
	 * printed marker names the step. pr_emerg so they print at any loglevel. */
	pr_emerg("M681_PWRAP: pwrap_init ENTER\n");
	forge_m681_mark_aux(0xA0, 0x01);	/* stage 0xA0 = pwrap_init; aux = step */
	PWRAPFUC();

#ifdef CONFIG_OF
	pr_emerg("M681_PWRAP: step of_iomap\n");
	sub_return = pwrap_of_iomap();
	if (sub_return) {
		pr_emerg("M681_PWRAP: of_iomap FAIL rc=%x\n", sub_return);
		return sub_return;
	}
#endif


#ifndef CONFIG_MTK_FPGA
	/* ############################### */
	/* toggle PMIC_WRAP and pwrap_spictl reset */
	/* ############################### */
	pr_emerg("M681_PWRAP: step set_rest\n");
	_pwrap_set_rest();

	/* ############################### */
	/* Set SPI_CK_freq  52MHz or 26MHz */
	/* ############################### */
	pr_emerg("M681_PWRAP: step set_spi_clk\n");
	_pwrap_set_spi_clk();
#endif
	/* ############################### */
	/* Enable DCM */
	/* ############################### */
	pr_emerg("M681_PWRAP: step set_DCM\n");
	_pwrap_set_DCM();

	/* ############################### */
	/* Reset all SPISLV */
	/* ############################### */
	pr_emerg("M681_PWRAP: step reset_spislv\n");
	sub_return = _pwrap_reset_spislv();
	if (sub_return != 0) {
		pr_emerg("M681_PWRAP: reset_spislv FAIL rc=%x\n", sub_return);
		PWRAPLOG("error,_pwrap_reset_spislv fail,sub_return=%x\n", sub_return);
		return E_PWR_INIT_RESET_SPI;
	}
	/* ############################### */
	/* Enable WACS2 */
	/* ############################### */
	pr_emerg("M681_PWRAP: step enable WACS2\n");
	WRAP_WR32(PMIC_WRAP_WRAP_EN, ENABLE);	/* enable wrap */
	WRAP_WR32(PMIC_WRAP_HIPRIO_ARB_EN, WACS2);	/* Only WACS2 */
	WRAP_WR32(PMIC_WRAP_WACS2_EN, ENABLE);

	/* ############################### */
	/* Input data calibration flow */
	/* ############################### */
	pr_emerg("M681_PWRAP: step init_sistrobe\n");
	sub_return = _pwrap_init_sistrobe();
	if (sub_return != 0) {
		pr_emerg("M681_PWRAP: init_sistrobe FAIL rc=%x\n", sub_return);
		PWRAPLOG("error,DrvPWRAP_InitSiStrobe fail,sub_return=%x\n", sub_return);
		return E_PWR_INIT_SIDLY;
	}
	/* ############################### */
	/* SPI Waveform Configuration */
	/* ############################### */
	/* 0:safe mode, 1:18MHz */
	pr_emerg("M681_PWRAP: step init_reg_clock\n");
	sub_return = _pwrap_init_reg_clock(1);
	if (sub_return != 0) {
		pr_emerg("M681_PWRAP: init_reg_clock FAIL rc=%x\n", sub_return);
		PWRAPLOG("error,_pwrap_init_reg_clock fail,sub_return=%x\n", sub_return);
		return E_PWR_INIT_REG_CLOCK;
	}
	/* ############################### */
	/* Enable DIO mode */
	/* ############################### */
	pr_emerg("M681_PWRAP: step init_dio\n");
	sub_return = _pwrap_init_dio(1);
	if (sub_return != 0) {
		pr_emerg("M681_PWRAP: init_dio FAIL rc=%x\n", sub_return);
		PWRAPLOG("_pwrap_init_dio test error,error code=%x, sub_return=%x\n", 0x11,
			 sub_return);
		return E_PWR_INIT_DIO;
	}
	/* ############################### */
	/* Enable Encryption */
	/* ############################### */
	pr_emerg("M681_PWRAP: step init_cipher\n");
	forge_m681_mark_aux(0xA0, 0x0C);	/* about to enter cipher init */
	sub_return = _pwrap_init_cipher();
	if (sub_return != 0) {
		/* Do not return here: cipher init is expected to fail with the MT6353
		 * pwrap HAL on MT6351 hardware (key mismatch). Continue with
		 * un-encrypted SPI access. The WACS2 path is bounded by its own
		 * TIMEOUT_WAIT_IDLE, so later reads/writes either succeed via the
		 * LK-initialized pwrap or fail fast with E_PWR_NOT_INIT_DONE. */
		pr_emerg("M681_PWRAP: init_cipher FAILED rc=%x (continuing degraded)\n", sub_return);
		PWRAPLOG("Enable Encryption fail, return=%x (degraded mode)\n", sub_return);
	} else {
		pr_emerg("M681_PWRAP: init_cipher OK\n");
	}
	/* ############################### */
	/* Write test using WACS2 */
	/* ############################### */
	pr_emerg("M681_PWRAP: step write_test\n");
	sub_return = pwrap_write_nochk(PMIC_DEW_WRITE_TEST_ADDR, PMIC_WRITE_TEST_VALUE);
	sub_return1 = pwrap_read_nochk(PMIC_DEW_WRITE_TEST_ADDR, &rdata);
	if (rdata != PMIC_WRITE_TEST_VALUE) {
		pr_emerg("M681_PWRAP: write_test FAIL rdata=0x%x exp=0x%x w=0x%x r=0x%x\n",
			 rdata, PMIC_WRITE_TEST_VALUE, sub_return, sub_return1);
		PWRAPLOG("write test error,rdata=0x%x,exp=0xa55a,sub_return=0x%x,sub_return1=0x%x\n",
		     rdata, sub_return, sub_return1);
		/* BOOT-UNBLOCK: continue, do not return. Could be a hash/cipher
		 * side-effect from the un-encrypted path; subsequent pwrap ops
		 * will likely fail with E_PWR_NOT_INIT_DONE and be handled by
		 * callers just like the pre-#93 stub state. */
	} else {
		pr_emerg("M681_PWRAP: write_test OK\n");
	}

	/* ############################### */
	/* Status update function initialization */
	/* 1. Signature Checking using CRC (CRC 0 only) */
	/* 2. EINT update */
	/* 3. Read back Auxadc thermal data for GPD */
	/* 4. Read back Auxadc thermal data for LTE */
	/* ############################### */
	pr_emerg("M681_PWRAP: step status_update\n");
	_pwrap_init_status_update();

	/* ############################### */
	/*  PMIC_WRAP priority adjusted    */
	/* ############################### */
	pr_emerg("M681_PWRAP: step adjust_priority\n");
	_pwrap_adjust_priority();

	/* ############################### */
	/* PMIC_WRAP starvation setting */
	/* ############################### */
	pr_emerg("M681_PWRAP: step set_starvation\n");
	_pwrap_set_starvation();

	/* ############################### */
	/* PMIC_WRAP enables */
	/* ############################### */
	pr_emerg("M681_PWRAP: step set_en\n");
	_pwrap_set_en();

	/* ############################### */
	/* Initialization Done */
	/* ############################### */
	pr_emerg("M681_PWRAP: step set_init_done\n");
	_pwrap_set_init_done();

	/* Diagnostic probe: pwrap is initialized, so a wacs2 read reflects what
	 * the SoC-side wrapper gets back from the PMIC over SPI. Read three
	 * registers:
	 *   SWCID       0x0202  (identical offset on MT6351 and MT6353) = chip id
	 *   READ_TEST   0x0342  (MT6353 offset) -> 0x5aa5 IFF map is MT6353
	 *   READ_TEST   0x02F4  (MT6351 offset) -> 0x5aa5 IFF map is MT6351
	 * Whichever READ_TEST returns 0x5aa5 (PMIC_DEFAULT_VALUE_READ_TEST) names
	 * the register map of the real silicon. Pure reads, no writes. Values
	 * also land in diag words at page 0x28/0x2c/0x30 (dd if=/dev/mem base
	 * 0x44800000). */
	pwrap_read_nochk(0x0202, &dew_swcid);
	pwrap_read_nochk(0x0342, &dew_rt6353);
	pwrap_read_nochk(0x02F4, &dew_rt6351);
	pr_emerg("M681_PWRAP: PROBE SWCID=0x%x READ_TEST[MT6353@0x342]=0x%x READ_TEST[MT6351@0x2F4]=0x%x (0x5aa5 => that map is correct)\n",
		 dew_swcid, dew_rt6353, dew_rt6351);
	forge_m681_diag(0x28, dew_swcid);
	forge_m681_diag(0x2c, dew_rt6353);
	forge_m681_diag(0x30, dew_rt6351);

#ifdef CONFIG_OF
	pwrap_of_iounmap();
#endif

	forge_m681_mark_aux(0xA0, 0x14);	/* pwrap_init reached DONE (binary channel) */
	pr_emerg("M681_PWRAP: pwrap_init DONE\n");
	return 0;
}

static irqreturn_t mt_pmic_wrap_irq(int irqno, void *dev_id)
{
	unsigned long flags = 0;

	PWRAPFUC();
	PWRAPREG("dump pwrap register\n");
	spin_lock_irqsave(&wrp_lock_isr, flags);
	/* *----------------------------------------------------------------------- */
	pwrap_dump_all_register();
	/* raise the priority of WACS2 for AP */
	WRAP_WR32(PMIC_WRAP_HARB_HPRIO, WACS2);

	/* *----------------------------------------------------------------------- */
	PWRAPREG("INT flag 0x%x\n", WRAP_RD32(PMIC_WRAP_INT0_FLG_RAW));
	PWRAPREG("INT flag 0x%x\n", WRAP_RD32(PMIC_WRAP_INT1_FLG_RAW));
	/* clear interrupt flag */
	WRAP_WR32(PMIC_WRAP_INT0_CLR, 0xffffffff);
	WRAP_WR32(PMIC_WRAP_INT1_CLR, 0xffffffff);
	/*BUG_ON(1);*/
	spin_unlock_irqrestore(&wrp_lock_isr, flags);
	return IRQ_HANDLED;
}


static u32 pwrap_read_test(void)
{
	u32 rdata = 0;
	u32 return_value = 0;
	u32 ret = 0;

	/* Read Test */
	return_value = pwrap_read(PMIC_DEW_READ_TEST_ADDR, &rdata);
	if (rdata != PMIC_DEFAULT_VALUE_READ_TEST) {
		PWRAPREG("Read Test fail,rdata=0x%x, exp=0x5aa5,return_value=0x%x\n", rdata,
			 return_value);
		ret = E_PWR_READ_TEST_FAIL;
	} else {
		PWRAPREG("Read Test pass,return_value=%d\n", return_value);
		/* return 0; */
	}

	return ret;
}

static u32 pwrap_write_test(void)
{
	u32 rdata = 0;
	u32 sub_return = 0;
	u32 sub_return1 = 0;
	u32 ret = 0;

	/* ############################### */
	/* Write test using WACS2 */
	/* ############################### */
	sub_return = pwrap_write(PMIC_DEW_WRITE_TEST_ADDR, PMIC_WRITE_TEST_VALUE);
	PWRAPREG("after %s pwrap_write\n", PMIC_CHIP_STR);
	sub_return1 = pwrap_read(PMIC_DEW_WRITE_TEST_ADDR, &rdata);
	if ((rdata != PMIC_WRITE_TEST_VALUE) || (sub_return != 0) || (sub_return1 != 0)) {
		PWRAPREG("write test error,rdata=0x%x,exp=0xa55a,sub_return=0x%x,sub_return1=0x%x\n",
		     rdata, sub_return, sub_return1);
		ret = E_PWR_INIT_WRITE_TEST;
	} else {
		PWRAPREG("write %s Test pass\n", PMIC_CHIP_STR);
	}

	return ret;
}

static void pwrap_int_test(void)
{
	u32 rdata1 = 0;
	u32 rdata2 = 0;

	while (1) {
		rdata1 = WRAP_RD32(PMIC_WRAP_EINT_STA);
		pwrap_read(PMIC_CPU_INT_STA_ADDR, &rdata2);
		PWRAPREG("Pwrap INT status check,PMIC_WRAP_EINT_STA=0x%x,PMIC_CPU_INT_STA_ADDR[0x01B4]=0x%x\n",
		     rdata1, rdata2);
		msleep(500);
	}
}

#ifndef USER_BUILD_KERNEL
static void pwrap_read_reg_on_pmic(u32 reg_addr)
{
	u32 reg_value = 0;
	u32 return_value = 0;
	/* PWRAPFUC(); */
	return_value = pwrap_read(reg_addr, &reg_value);
	PWRAPREG("0x%x=0x%x,return_value=%x\n", reg_addr, reg_value, return_value);
}

static void pwrap_write_reg_on_pmic(u32 reg_addr, u32 reg_value)
{
	u32 return_value = 0;

	PWRAPREG("write 0x%x to register 0x%x\n", reg_value, reg_addr);
	return_value = pwrap_write(reg_addr, reg_value);
	return_value = pwrap_read(reg_addr, &reg_value);
	/* PWRAPFUC(); */
	PWRAPREG("the result:0x%x=0x%x,return_value=%x\n", reg_addr, reg_value, return_value);
}
#endif

static void pwrap_ut(u32 ut_test)
{
	switch (ut_test) {
	case 1:
		/* pwrap_wacs2_para_test(); */
		pwrap_write_test();
		break;
	case 2:
		/* pwrap_wacs2_para_test(); */
		pwrap_read_test();
		break;

	default:
		PWRAPREG("default test.\n");
		break;
	}
}

/*---------------------------------------------------------------------------*/
static s32 mt_pwrap_show_hal(char *buf)
{
	PWRAPFUC();
	return snprintf(buf, PAGE_SIZE, "%s\n", "no implement");
}

/*---------------------------------------------------------------------------*/
static s32 mt_pwrap_store_hal(const char *buf, size_t count)
{
#ifndef USER_BUILD_KERNEL
	u32 reg_value = 0;
	u32 reg_addr = 0;
#endif
	u32 return_value = 0;
	u32 ut_test = 0;

	if (!strncmp(buf, "-h", 2)) {
		PWRAPREG
		    ("PWRAP debug: [-dump_reg][-trace_wacs2][-init][-rdpmic][-wrpmic][-readtest][-writetest]\n");
		PWRAPREG("PWRAP UT: [1][2]\n");
	}
	/* ----------pwrap debug---------- */
	else if (!strncmp(buf, "-dump_reg", 9)) {
		pwrap_dump_all_register();
	} else if (!strncmp(buf, "-trace_wacs2", 12)) {
		/* pwrap_trace_wacs2(); */
	} else if (!strncmp(buf, "-init", 5) || !strncmp(buf, "-writetest", 10) ||
		   !strncmp(buf, "-int", 4) || !strncmp(buf, "-ut", 3)) {
		/* never re-init pwrap, no write/loop tests */
		pr_warn(PWRAPTAG "m681 4.9: '%.12s' refused (read-only WACS2, LK owns pwrap init)\n",
			buf);
	} else if (0) {
		return_value = pwrap_init();
		if (return_value == 0)
			PWRAPREG("pwrap_init pass,return_value=%d\n", return_value);
		else
			PWRAPREG("pwrap_init fail,return_value=%d\n", return_value);
	}
#ifndef USER_BUILD_KERNEL
	else if (!strncmp(buf, "-rdpmic", 7) && (1 == sscanf(buf + 7, "%x", &reg_addr))) {
		pwrap_read_reg_on_pmic(reg_addr);
	} else if (!strncmp(buf, "-wrpmic", 7)
		   && (2 == sscanf(buf + 7, "%x %x", &reg_addr, &reg_value))) {
		pwrap_write_reg_on_pmic(reg_addr, reg_value);
	}
#endif
	else if (!strncmp(buf, "-readtest", 9)) {
		pwrap_read_test();
	} else if (!strncmp(buf, "-writetest", 10)) {
		pwrap_write_test();
	} else if (!strncmp(buf, "-int", 4)) {
		pwrap_int_test();
	}
	/* ----------pwrap UT---------- */
	else if (!strncmp(buf, "-ut", 3) && (1 == sscanf(buf + 3, "%d", &ut_test))) {
		pwrap_ut(ut_test);
	} else {
		PWRAPREG("wrong parameter\n");
	}
	return count;
}

/*---------------------------------------------------------------------------*/
#endif				/* End of #if PMIC_WRAP_NO_PMIC */

#ifdef CONFIG_OF
static int pwrap_of_iomap(void)
{
	/*
	 * Map the address of the following register base:
	 * TOPCKGEN, INFRACFG_AO, GPIO, SLEEP
	 */

	struct device_node *topckgen_node;
	struct device_node *infracfg_ao_node;
	struct device_node *gpio_node;
	struct device_node *sleep_node;

	topckgen_node = of_find_compatible_node(NULL, NULL, "mediatek,TOPCKGEN");
	if (!topckgen_node) {
		pr_info("get TOPCKGEN failed\n");
		return -ENODEV;
	}

	topckgen_reg_base = of_iomap(topckgen_node, 0);
	if (!topckgen_reg_base) {
		pr_info("TOPCKGEN iomap failed\n");
		return -ENOMEM;
	}

	infracfg_ao_node = of_find_compatible_node(NULL, NULL, "mediatek,INFRACFG_AO");
	if (!infracfg_ao_node) {
		pr_info("get INFRACFG_AO failed\n");
		return -ENODEV;
	}

	infracfg_ao_reg_base = of_iomap(infracfg_ao_node, 0);
	if (!infracfg_ao_reg_base) {
		pr_info("INFRACFG_AO iomap failed\n");
		return -ENOMEM;
	}

	gpio_node = of_find_compatible_node(NULL, NULL, "mediatek,GPIO");
	if (!gpio_node) {
		pr_info("get GPIO failed\n");
		return -ENODEV;
	}

	gpio_reg_base = of_iomap(gpio_node, 0);
	if (!gpio_reg_base) {
		pr_info("GPIO iomap failed\n");
		return -ENOMEM;
	}

	sleep_node = of_find_compatible_node(NULL, NULL, "mediatek,SLEEP");
	if (!sleep_node) {
		pr_info("get SLEEP failed\n");
		return -ENODEV;
	}

	sleep_reg_base = of_iomap(sleep_node, 0);
	if (!sleep_reg_base) {
		pr_info("SLEEp iomap failed\n");
		return -ENOMEM;
	}

	pr_info("TOPCKGEN reg: 0x%p\n", topckgen_reg_base);
	pr_info("INFRACFG_AO reg: 0x%p\n", infracfg_ao_reg_base);
	pr_info("GPIO reg: 0x%p\n", gpio_reg_base);
	pr_info("SLEEP reg: 0x%p\n", sleep_reg_base);

	return 0;
}

static void pwrap_of_iounmap(void)
{
	iounmap(topckgen_reg_base);
	iounmap(infracfg_ao_reg_base);
	iounmap(gpio_reg_base);
	iounmap(sleep_reg_base);
}
#endif

#define VERSION     "Revision"
static int is_pwrap_init_done(void)
{
	int ret = 0;

	ret = WRAP_RD32(PMIC_WRAP_INIT_DONE2);
	if (ret != 0)
		return 0;

	/* LK did not leave INIT_DONE2 set. Running the full pwrap_init() here
	 * (reset, SPI clock, DIO, cipher, sistrobe) is not allowed, so WACS2
	 * stays unwired. */
	return E_PWR_NOT_INIT_DONE;
}

/*
 * Early "boot into recovery" marker in the PMIC RTC, so that a kernel dying
 * before the eMMC exists (and with it the BCB bootguard of
 * init/forge_m681_marker.c) still lands in recovery. Board-gated.
 *
 * LK boot_mode_select calls Check_RTC_Recovery_Mode, (RTC_PDN1 & 0x30) ==
 * 0x10, and then sets g_boot_mode = RECOVERY_BOOT; rtc_boot_check()
 * consumes the bit afterwards. It is the same field the stock
 * rtc_mark_recovery() sets (RTC_FAC_RESET = {RTC_PDN1, 0x3, 4}).
 * Written the way the LK setter does: RTC_PROT 0x586a, trigger, RTC_PROT
 * 0x9136, trigger, RTC_PDN1 bit4 set/clear, trigger, where a trigger is
 * RTC_WRTGR = 1 and a bounded wait for RTC_BBPU CBUSY (bit 6); then
 * RTC_PROT 0 and a trigger lock the write interface again, as the stock
 * rtc_set_writeif(false). Each write passes the read-only WACS2 gate on
 * its own token (forge_rtc_write()): this task, that address, that value,
 * once; nothing else gets through meanwhile.
 *
 * When: right after WACS2 is wired (postcore), if forge_ar_rtc says so:
 *   forge_ar_rtc=-1 (default) - armed on the l681 board (DT "meizu,l681")
 *                               only; m681 keeps it off;
 *   forge_ar_rtc=1 / 0        - armed / not armed, l681 board only: on
 *                               m681 or an unknown board 1 is refused
 *                               (kernel command line).
 * The kernel never clears it by itself: from then on any reset (panic, WDT,
 * plain reboot) boots recovery once, and LK consumes the flag.
 * rtc_guard=0 leaves the RTC alone;
 * /sys/module/pwrap_hal_v1/parameters/rtc_flag_test (write 1 / 0) sets or
 * clears it by hand, to test the RTC path alone (with the BCB cleared) and
 * the BCB path alone (with the RTC flag cleared); l681 board only, -EPERM
 * elsewhere.
 */
#define FORGE_RTC_BBPU		0x4000
#define FORGE_RTC_PDN1		0x402c
#define FORGE_RTC_PROT		0x4036
#define FORGE_RTC_WRTGR		0x403c
#define FORGE_RTC_PDN1_RECOVERY	0x10
/* the stock FG SOC spare field, RTC_AL_HOU bits 14:8 (mt_rtc_hw.h:101,104-106) */
#define FORGE_RTC_AL_HOU	0x401c
#define FORGE_RTC_FG_SOC_MASK	0x7f00

/* every RTC write sequence of this file: the recovery flag and the SOC */
static DEFINE_MUTEX(forge_rtc_lock);
/* a relock (RTC_PROT 0) failed: the next SOC set relocks even when it has
 * nothing to write; under forge_rtc_lock */
static bool forge_rtc_relock_pending;

static int forge_rtc_guard = 1;
module_param_named(rtc_guard, forge_rtc_guard, int, 0644);
static int forge_ar_rtc = -1;
core_param(forge_ar_rtc, forge_ar_rtc, int, 0444);
static unsigned int forge_rtc_pdn1;	/* PDN1 read back after the last write */
module_param_named(rtc_pdn1, forge_rtc_pdn1, uint, 0444);

/* one gated write: arm the token, write, drop the token (consumed or not) */
static s32 forge_rtc_write(u32 adr, u32 val)
{
	unsigned long flags;
	s32 ret;

	spin_lock_irqsave(&wrp_lock, flags);
	forge_rtc_exp_adr = adr;
	forge_rtc_exp_val = val;
	forge_rtc_owner = current;
	spin_unlock_irqrestore(&wrp_lock, flags);
	ret = pwrap_wacs2_hal(1, adr, val, NULL);
	spin_lock_irqsave(&wrp_lock, flags);
	forge_rtc_owner = NULL;
	spin_unlock_irqrestore(&wrp_lock, flags);
	return ret;
}

static s32 forge_rtc_trigger(void)
{
	u32 bbpu = 0;
	s32 ret = forge_rtc_write(FORGE_RTC_WRTGR, 1);
	int i;

	for (i = 0; !ret && i < 1000; i++) {
		ret = pwrap_wacs2_hal(0, FORGE_RTC_BBPU, 0, &bbpu);
		if (!ret && !(bbpu & 0x40))
			return 0;
		udelay(10);
	}
	return ret ? ret : -ETIMEDOUT;
}

int forge_rtc_recovery_flag(int set)
{
	u32 pdn1 = 0, now = 0;
	s32 ret, relock;

	if (forge_pwrap_state != 1 || !forge_rtc_guard)
		return -ENODEV;
	mutex_lock(&forge_rtc_lock);
	ret = forge_rtc_write(FORGE_RTC_PROT, 0x586a);
	if (!ret)
		ret = forge_rtc_trigger();
	if (!ret)
		ret = forge_rtc_write(FORGE_RTC_PROT, 0x9136);
	if (!ret)
		ret = forge_rtc_trigger();
	if (!ret)
		ret = pwrap_wacs2_hal(0, FORGE_RTC_PDN1, 0, &pdn1);
	if (!ret)
		ret = forge_rtc_write(FORGE_RTC_PDN1,
				      set ? pdn1 | FORGE_RTC_PDN1_RECOVERY :
					    pdn1 & ~FORGE_RTC_PDN1_RECOVERY);
	if (!ret)
		ret = forge_rtc_trigger();
	/* lock the write interface again, whatever happened above */
	relock = forge_rtc_write(FORGE_RTC_PROT, 0);
	if (!relock)
		relock = forge_rtc_trigger();
	forge_rtc_relock_pending = relock != 0;
	if (relock)
		pr_emerg("[FORGE_M681] RTC recovery flag: relock RTC_PROT 0 failed (%d)\n",
			 relock);
	if (!ret)
		ret = pwrap_wacs2_hal(0, FORGE_RTC_PDN1, 0, &now);
	forge_rtc_pdn1 = now;
	mutex_unlock(&forge_rtc_lock);
	forge_m681_mark_aux(0xA2, (set ? 0x10000 : 0) | (now & 0xffff));
	pr_emerg("[FORGE_M681] RTC recovery flag %s: PDN1 0x%04x -> 0x%04x (%d)%s\n",
		 set ? "set" : "cleared", pdn1, now, ret,
		 set && (now & 0x30) != 0x10 ? " - LK will NOT see it (bit 5 set)" : "");
	return ret;
}
EXPORT_SYMBOL(forge_rtc_recovery_flag);

/*
 * The published battery percentage in the stock FG SOC field, so that it
 * survives a reboot as on stock (forge_chg_observe reads it at init and
 * writes it on every change and at reboot). The field is RTC_AL_HOU bits
 * 14:8; bits 4:0 are the power-on alarm hour and 15, 7:5 belong to other
 * users, hence read-modify-write. Stock semantics: 0 = no value, a
 * percentage of 1 or less is stored as 1, above 100 is refused. Same
 * unlock / trigger / relock sequence and the same token gate as the
 * recovery flag, under the same mutex; process context only (the token
 * refuses in_interrupt). CONFIG_MTK_RTC is off here, so the stock
 * get/set_rtc_spare_fg_value() are ({0;}) macros and are not used.
 */
static int forge_rtc_soc_val = -1;	/* last read or written, -1 never */
module_param_named(rtc_soc, forge_rtc_soc_val, int, 0444);

int forge_rtc_soc_get(void)
{
	u32 v = 0;
	s32 ret;

	if (forge_pwrap_state != 1 || in_interrupt())
		return -ENODEV;
	mutex_lock(&forge_rtc_lock);
	ret = pwrap_wacs2_hal(0, FORGE_RTC_AL_HOU, 0, &v);
	mutex_unlock(&forge_rtc_lock);
	if (ret)
		return ret < 0 ? ret : -EIO;
	forge_rtc_soc_val = (v & FORGE_RTC_FG_SOC_MASK) >> 8;
	return forge_rtc_soc_val;
}
EXPORT_SYMBOL(forge_rtc_soc_get);

int forge_rtc_soc_set(int soc)
{
	u32 v = 0, nv, back = 0;
	s32 ret, relock;

	if (soc < 0 || soc > 100)
		return -EINVAL;
	if (soc <= 1)
		soc = 1;
	if (forge_pwrap_state != 1 || !forge_rtc_guard || in_interrupt())
		return -ENODEV;
	mutex_lock(&forge_rtc_lock);
	/* already there: no unlock at all */
	ret = pwrap_wacs2_hal(0, FORGE_RTC_AL_HOU, 0, &v);
	nv = (v & ~FORGE_RTC_FG_SOC_MASK) | ((u32)soc << 8);
	if (!ret && nv == v && !forge_rtc_relock_pending) {
		forge_rtc_soc_val = soc;
		mutex_unlock(&forge_rtc_lock);
		return 0;
	}
	ret = forge_rtc_write(FORGE_RTC_PROT, 0x586a);
	if (!ret)
		ret = forge_rtc_trigger();
	if (!ret)
		ret = forge_rtc_write(FORGE_RTC_PROT, 0x9136);
	if (!ret)
		ret = forge_rtc_trigger();
	if (!ret)
		ret = pwrap_wacs2_hal(0, FORGE_RTC_AL_HOU, 0, &v);
	nv = (v & ~FORGE_RTC_FG_SOC_MASK) | ((u32)soc << 8);
	if (!ret && nv != v) {
		ret = forge_rtc_write(FORGE_RTC_AL_HOU, nv);
		if (!ret)
			ret = forge_rtc_trigger();
	}
	/* lock the write interface again, whatever happened above */
	relock = forge_rtc_write(FORGE_RTC_PROT, 0);
	if (!relock)
		relock = forge_rtc_trigger();
	forge_rtc_relock_pending = relock != 0;
	if (!ret)
		ret = pwrap_wacs2_hal(0, FORGE_RTC_AL_HOU, 0, &back);
	if (!ret && back != nv)
		ret = -EIO;
	if (!ret)
		forge_rtc_soc_val = (back & FORGE_RTC_FG_SOC_MASK) >> 8;
	mutex_unlock(&forge_rtc_lock);
	if (ret || relock)	/* a stuck relock is retried every pass */
		pr_notice_ratelimited("[FORGE_M681] RTC SOC %d: write %d, relock %d (AL_HOU 0x%04x -> 0x%04x, read back 0x%04x)\n",
			  soc, ret, relock, v, nv, back);
	/* pwrap returns positive E_PWR_* codes; callers get -errno */
	if (!ret)
		ret = relock;
	return ret > 0 ? -EIO : ret;
}
EXPORT_SYMBOL(forge_rtc_soc_set);

bool forge_rtc_relock_needed(void)
{
	return READ_ONCE(forge_rtc_relock_pending);
}
EXPORT_SYMBOL(forge_rtc_relock_needed);

/* Hand check: write N (0..100) to store it, then read it back. */
static int forge_rtc_soc_test_set(const char *val, const struct kernel_param *kp)
{
	int v, ret = kstrtoint(val, 0, &v);

	if (ret)
		return ret;
	ret = forge_rtc_soc_set(v);
	pr_notice("[FORGE_M681] rtc_soc_test: set %d -> %d, read back %d\n",
		  v, ret, forge_rtc_soc_get());
	return ret;
}
static const struct kernel_param_ops forge_rtc_soc_test_ops = {
	.set = forge_rtc_soc_test_set,
};
module_param_cb(rtc_soc_test, &forge_rtc_soc_test_ops, NULL, 0200);

static void __init forge_rtc_flag_boot(void)
{
	bool l681 = m3note_board_is(M3NOTE_BOARD_L681);
	int arm = forge_ar_rtc < 0 ? l681 : forge_ar_rtc != 0;

	if (arm && !l681) {
		pr_emerg("[FORGE_M681] RTC recovery flag at postcore: forge_ar_rtc=%d refused, not the l681 board (-EPERM)\n",
			 forge_ar_rtc);
		return;
	}

	pr_emerg("[FORGE_M681] RTC recovery flag at postcore: %s (forge_ar_rtc=%d, board %s)\n",
		 arm ? "arming" : "not armed", forge_ar_rtc,
		 l681 ? "l681" : m3note_board_is(M3NOTE_BOARD_M681) ? "m681" : "unknown");
	if (arm)
		forge_rtc_recovery_flag(1);
}

/* Hardware check of the LK path: write 1 to set the flag, 0 to clear it. */
static int forge_rtc_flag_test_set(const char *val, const struct kernel_param *kp)
{
	int v, ret = kstrtoint(val, 0, &v);

	if (ret)
		return ret;
	if (!m3note_board_is(M3NOTE_BOARD_L681)) {
		pr_emerg("[FORGE_M681] rtc_flag_test: refused, not the l681 board (-EPERM)\n");
		return -EPERM;
	}
	return forge_rtc_recovery_flag(v != 0);
}
static const struct kernel_param_ops forge_rtc_flag_test_ops = {
	.set = forge_rtc_flag_test_set,
};
module_param_cb(rtc_flag_test, &forge_rtc_flag_test_ops, NULL, 0200);

static int __init pwrap_hal_init(void)
{
	s32 ret = 0;
#ifdef CONFIG_OF
	u32 pwrap_irq;
	struct device_node *pwrap_node;

	pwrap_node = of_find_compatible_node(NULL, NULL, "mediatek,pwrap");
	if (!pwrap_node) {
		pr_info("PWRAP get node failed\n");
		return -ENODEV;
	}

	pwrap_base = of_iomap(pwrap_node, 0);
	if (!pwrap_base) {
		pr_info("PWRAP iomap failed\n");
		return -ENOMEM;
	}

	pwrap_irq = irq_of_parse_and_map(pwrap_node, 0);
	if (!pwrap_irq) {
		pr_info("PWRAP get irq fail\n");
		return -ENODEV;
	}
	pr_info("PWRAP reg: 0x%p,  irq: %d\n", pwrap_base, pwrap_irq);
#endif

	PWRAPLOG("HAL init: version %s\n", VERSION);
	mt_wrp = get_mt_pmic_wrap_drv();
	mt_wrp->store_hal = mt_pwrap_store_hal;
	mt_wrp->show_hal = mt_pwrap_show_hal;
	/* Postcore init as in the 3.18 tree, which runs the same file on the same
	 * LK: LK sets INIT_DONE2, pwrap_init() is skipped entirely and consumers
	 * use the LK-configured WACS2. The DEW/accessor map uses the MT6351
	 * offsets. The 0xA1 step markers below name the step if this hangs
	 * (INIT_DONE2 MMIO vs request_irq). The INIT_DONE2==0 fallback and the
	 * '-init' sysfs trigger are not built (see the file header). */
	{
		extern void forge_m681_mark_aux(u8 stage, u32 aux);
		struct device_node *ao;
		void __iomem *ao_base;
		u32 sta0;

		/* First make sure the wrapper is clocked, then that LK initialised it;
		 * only then wire WACS2 (reads only, see top). No pwrap register is read
		 * before the infracfg_ao gate check. */
		forge_m681_mark_aux(0xA1, 0x01);
		ao = of_find_compatible_node(NULL, NULL, "mediatek,infracfg_ao");
		ao_base = ao ? of_iomap(ao, 0) : NULL;
		of_node_put(ao);
		if (!ao_base) {
			pr_emerg("M681_PWRAP: 4.9 infracfg_ao not mapped - WACS2 left unwired\n");
			forge_pwrap_state = -1;
			forge_m681_mark_aux(0xA1, 0x2F);
			return 0;
		}
		sta0 = readl(ao_base + 0x90);	/* MODULE_SW_CG_0_STA: 1 = gated */
		iounmap(ao_base);
		if (sta0 & 0x3) {	/* PMIC_TMR (bit0) / PMIC_AP (bit1) gated */
			pr_emerg("M681_PWRAP: 4.9 infra CG0 STA=0x%08x: pwrap clock gated - WACS2 left unwired, no pwrap MMIO\n",
				 sta0);
			forge_pwrap_state = -1;
			forge_m681_mark_aux(0xA1, 0x10 | (sta0 & 0xF));
			return 0;
		}

		forge_m681_mark_aux(0xA1, 0x02);	/* before INIT_DONE2 MMIO */
		if (is_pwrap_init_done() != 0) {
			pr_emerg("M681_PWRAP: 4.9 INIT_DONE2 clear (LK did not init pwrap) - no pwrap_init, WACS2 left unwired\n");
			forge_pwrap_state = -2;
			forge_m681_mark_aux(0xA1, 0x20);
			return 0;
		}

		forge_m681_mark_aux(0xA1, 0x03);	/* before request_irq */
#ifdef PMIC_WRAP_NO_PMIC
#else
		ret = request_irq(MT_PMIC_WRAP_IRQ_ID, mt_pmic_wrap_irq,
				  IRQF_TRIGGER_HIGH, PMIC_WRAP_DEVICE, 0);
#endif
		if (ret) {
			PWRAPERR("register IRQ failed (%d)\n", ret);
			pr_emerg("M681_PWRAP: 4.9 request_irq(%d) failed %d - WACS2 still wired\n",
				 pwrap_irq, ret);
			ret = 0;
		}
		mt_wrp->wacs2_hal = pwrap_wacs2_hal;
		forge_pwrap_state = 1;
		forge_m681_mark_aux(0xA1, 0x04);	/* WACS2 wired, read-only */
		pr_emerg("M681_PWRAP: 4.9 pwrap_hal_init: INIT_DONE2 set by LK, CG0 STA=0x%08x, WACS2 wired READ-ONLY (no pwrap_init, writes dropped)\n",
			 sta0);
		forge_rtc_flag_boot();	/* early recovery marker, l681 by default */
	}
	return ret;
}
postcore_initcall(pwrap_hal_init);

