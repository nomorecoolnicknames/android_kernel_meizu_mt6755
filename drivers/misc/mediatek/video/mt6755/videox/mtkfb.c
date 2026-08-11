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

#include <generated/autoconf.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/kthread.h>
#include <linux/vmalloc.h>
#include "disp_assert_layer.h"
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/suspend.h>
#include <linux/of_fdt.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/dma-buf.h>
#include <linux/uaccess.h>
#include <linux/atomic.h>
#include <linux/err.h>
#include <linux/workqueue.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
/* #include <asm/mach-types.h> */
#include <asm/cacheflush.h>
#include <linux/io.h>
#include "ion_drv.h"
#include <mt-plat/dma.h>
#include <mt-plat/mt_gpio.h>
/* #include <mach/irqs.h> */
#include <linux/dma-mapping.h>
#include <linux/compat.h>
#include <mt-plat/aee.h>
#include <mt-plat/mt_boot.h>
#include <linux/m3note_board.h>
#include <linux/moduleparam.h>
#include "disp_debug.h"
#include "ddp_hal.h"
#include "disp_log.h"
#include "disp_lcm.h"
#include "mtkfb.h"
#include "mtkfb_console.h"
#include "mtkfb_fence.h"
#include "mtkfb_debug.h"
#include "mtkfb_info.h"
#include "ddp_ovl.h"
#include "disp_drv_platform.h"
#include "primary_display.h"
#include "ddp_dump.h"
#include "disp_recorder.h"
#include "fbconfig_kdebug.h"
#include "mtk_ovl.h"
#include "mt_boot.h"
#include "disp_helper.h"
#include "disp_dts_gpio.h"
#include "disp_recovery.h"
#include "ddp_clkmgr.h"
#ifdef CONFIG_LCDKIT_DRIVER
#include "lcdkit_fb_util.h"
#endif
#include "ddp_dsi.h"
#include "ddp_reg.h"
#ifdef CONFIG_LOG_JANK
#include <huawei_platform/log/log_jank.h>
#endif

/* static variable */
static u32 MTK_FB_XRES;
static u32 MTK_FB_YRES;
static u32 MTK_FB_BPP;
static u32 MTK_FB_PAGES;
static u32 fb_xres_update;
static u32 fb_yres_update;
static size_t mtkfb_log_on = true;

static int sem_flipping_cnt = 1;
static int sem_early_suspend_cnt = 1;
static int vsync_cnt;
static const struct timeval FRAME_INTERVAL = { 0, 30000 };	/* 33ms */

static bool no_update;

/* display works now -> disable the early-FB WHITE diagnostic markers
 * (they deliberately filled the boot framebuffer white + an isolated const
 * layer to prove the path; now just a white flash / glitch between LK logo
 * and bootanimation). */
#define M6_EARLY_FB_WHITE_MARKER 0
#define M6_EARLY_FB_WHITE_MARKER_TRIGGER 0
#define M6_EARLY_FB_CONST_LAYER_ISOLATION 0
#define M6_EARLY_FB_DIAG_DELAY_MS 30000
/* 0 disables the periodic early-fb diag DISPERR flood (was 10). */
#define M6_EARLY_FB_DIAG_REPORT_LIMIT 0
#define M6_EARLY_FB_DIAG_PROC_NAME "m6_mtkfb_early_diag"
#define M6_OVL_CONST_WHITE_MAGIC_KEY 0x006d3657
struct m6_mtkfb_pipe_snapshot {
	bool valid;
	u32 dl_valid0;
	u32 dl_ready0;
	u32 dl_valid1;
	u32 dl_ready1;
	u32 mutex_inten;
	u32 mutex_intsta;
	u32 mutex_en;
	u32 mutex_mod;
	u32 mutex_sof;
	u32 rdma_inten;
	u32 rdma_intsta;
	u32 rdma_global;
	u32 rdma_size0;
	u32 rdma_size1;
	u32 rdma_target;
	u32 rdma_mem_con;
	u32 rdma_mem_start;
	u32 rdma_pitch;
	u32 rdma_fifo_con;
	u32 rdma_fifo_log;
	u32 rdma_debug_sel;
	u32 rdma_in_p;
	u32 rdma_in_l;
	u32 rdma_out_p;
	u32 rdma_out_l;
	u32 cmdq_rdma_sof;
	u32 cmdq_rdma_eof;
	u32 cmdq_mutex0_eof;
	u32 cmdq_dsi0_sof;
	u32 cmdq_dsi0_eof;
	u32 cmdq_mdp_dsi0_te_sof;
};

struct m6_mtkfb_early_diag_state {
	bool filled;
	bool marker_pending;
	bool fb_triggered;
	bool const_attempted;
	bool const_triggered;
	bool const_ovl_snapshot_valid;
	size_t bytes;
	void *va;
	dma_addr_t pa;
	u32 sample_first;
	u32 sample_mid;
	u32 sample_last;
	u32 xres;
	u32 yres;
	u32 bpp;
	u32 pages;
	u32 line;
	u32 yoffset;
	u32 line_length;
	u32 fb_fmt;
	u32 fb_pitch;
	u32 width;
	u32 height;
	u8 layer_id;
	int fb_cfg_ret;
	int fb_trigger_ret;
	int const_cfg_ret;
	int const_trigger_ret;
	struct m6_ovl_config_snapshot const_ovl_snapshot;
	struct m6_mtkfb_pipe_snapshot const_pipe_pre;
	struct m6_mtkfb_pipe_snapshot const_pipe_cfg;
	struct m6_mtkfb_pipe_snapshot const_pipe_trigger;
	struct m6_dsi_live_snapshot const_dsi_pre;
	struct m6_dsi_live_snapshot const_dsi_cfg;
	struct m6_dsi_live_snapshot const_dsi_trigger;
};
static struct m6_mtkfb_early_diag_state m6_early_fb_diag;
static unsigned int m6_early_fb_diag_reports;
static bool m6_early_fb_diag_started;
static bool m6_early_fb_diag_proc_registered;
static void m6_mtkfb_early_diag_work(struct work_struct *work);
static DECLARE_DELAYED_WORK(m6_mtkfb_early_diag_work_item,
			    m6_mtkfb_early_diag_work);
static disp_session_input_config session_input;

/* macro definiton */
#define ALIGN_TO(x, n)  (((x) + ((n) - 1)) & ~((n) - 1))
#define MTK_FB_XRESV (ALIGN_TO(MTK_FB_XRES, MTK_FB_ALIGNMENT))
#define MTK_FB_YRESV (MTK_FB_YRES * MTK_FB_PAGES)	/* For page flipping */
#define MTK_FB_BYPP  ((MTK_FB_BPP + 7) >> 3)
#define MTK_FB_LINE  (ALIGN_TO(MTK_FB_XRES, MTK_FB_ALIGNMENT) * MTK_FB_BYPP)
#define MTK_FB_SIZE  (MTK_FB_LINE * MTK_FB_YRES)
#define MTK_FB_SIZEV (MTK_FB_LINE * MTK_FB_YRES * MTK_FB_PAGES)
#define ASSERT_LAYER    (DDP_OVL_LAYER_MUN-1)
#define DISP_DEFAULT_UI_LAYER_ID (DDP_OVL_LAYER_MUN-1)
#define DISP_CHANGED_UI_LAYER_ID (DDP_OVL_LAYER_MUN-2)

#define CHECK_RET(expr)    \
do {                   \
	int ret = (expr);  \
	ASSERT(0 == ret);  \
} while (0)

#define MTKFB_LOG(fmt, arg...) \
	do { \
		if (mtkfb_log_on) \
			DISPMSG(fmt, ##arg); \
	} while (0)
/* always show this debug info while the global debug log is off */
#define MTKFB_LOG_DBG(fmt, arg...) \
	do { \
		if (!mtkfb_log_on) \
			DISPMSG(fmt, ##arg); \
	} while (0)

#define MTKFB_FUNC()	\
	do { \
		if (mtkfb_log_on) \
			DISPMSG("[Func]%s\n", __func__); \
	} while (0)

#define PRNERR(fmt, args...)   DISPMSG(fmt, ## args)

/* --------------------------------------------------------------------------- */
/* local variables */
/* --------------------------------------------------------------------------- */
struct notifier_block pm_nb;
unsigned int EnableVSyncLog = 0;
unsigned long fb_pa = 0;
atomic_t has_pending_update = ATOMIC_INIT(0);
struct fb_overlay_layer video_layerInfo;
uint32_t dbr_backup = 0;
uint32_t dbg_backup = 0;
uint32_t dbb_backup = 0;
bool fblayer_dither_needed = false;
bool is_ipoh_bootup = false;
struct fb_info *mtkfb_fbi;
struct fb_overlay_layer fb_layer_context;
mtk_dispif_info_t dispif_info[MTKFB_MAX_DISPLAY_COUNT];
unsigned int FB_LAYER = 2;
bool is_early_suspended = false;
atomic_t OverlaySettingDirtyFlag = ATOMIC_INIT(0);
atomic_t OverlaySettingApplied = ATOMIC_INIT(0);
unsigned int PanDispSettingPending = 0;
unsigned int PanDispSettingDirty = 0;
unsigned int PanDispSettingApplied = 0;
unsigned int need_esd_check = 0;
unsigned int lcd_fps = 6000;
wait_queue_head_t screen_update_wq;
char mtkfb_lcm_name[256] = { 0 };


DEFINE_SEMAPHORE(sem_flipping);
DEFINE_SEMAPHORE(sem_early_suspend);
DEFINE_SEMAPHORE(sem_overlay_buffer);

/* --------------------------------------------------------------------------- */
/* local function declarations */
/* --------------------------------------------------------------------------- */
static int mtkfb_set_par(struct fb_info *fbi);
static int init_framebuffer(struct fb_info *info);
static int mtkfb_get_overlay_layer_info(struct fb_overlay_layer_info *layerInfo);

#ifdef CONFIG_OF
static int _parse_tag_videolfb(void);
#endif

void mtkfb_log_enable(int enable)
{
	mtkfb_log_on = enable;
	MTKFB_LOG("mtkfb log %s\n", enable ? "enabled" : "disabled");
}

/*
 * ---------------------------------------------------------------------------
 * fbdev framework callbacks and the ioctl interface
 * ---------------------------------------------------------------------------
 */
/* Called each time the mtkfb device is opened */
static int mtkfb_open(struct fb_info *info, int user)
{
#ifdef CONFIG_LOG_JANK
		LOG_JANK_D(JLID_KERNEL_LCD_OPEN,"%s", "JL_KERNEL_LCD_OPEN");
#endif
	DISPFUNC();
	MSG_FUNC_ENTER();
	MSG_FUNC_LEAVE();
	return 0;
}

/* Called when the mtkfb device is closed. We make sure that any pending
 * gfx DMA operations are ended, before we return. */
static int mtkfb_release(struct fb_info *info, int user)
{

	DISPFUNC();

	MSG_FUNC_ENTER();
	MSG_FUNC_LEAVE();
	return 0;
}

/* Store a single color palette entry into a pseudo palette or the hardware
 * palette if one is available. For now we support only 16bpp and thus store
 * the entry only to the pseudo palette.
 */
#if 0
static int mtkfb_setcolreg(u_int regno, u_int red, u_int green,
			   u_int blue, u_int transp, struct fb_info *info)
{
	int r = 0;
	unsigned bpp, m;


	MSG_FUNC_ENTER();

	bpp = info->var.bits_per_pixel;
	m = 1 << bpp;
	if (regno >= m) {
		r = -EINVAL;
		goto exit;
	}

	switch (bpp) {
	case 16:
		/* RGB 565 */
		((u32 *) (info->pseudo_palette))[regno] =
		    ((red & 0xF800) | ((green & 0xFC00) >> 5) | ((blue & 0xF800) >> 11));
		break;
	case 32:
		/* ARGB8888 */
		((u32 *) (info->pseudo_palette))[regno] =
		    (0xff000000) |
		    ((red & 0xFF00) << 8) | ((green & 0xFF00)) | ((blue & 0xFF00) >> 8);
		break;

		/* TODO: RGB888, BGR888, ABGR8888 */

	default:
	DISPERR("set color info fail, bpp=%d\n", bpp);
	}

exit:
	MSG_FUNC_LEAVE();
	return r;
}
#endif
static void mtkfb_blank_resume(void)
{
	int ret;

	if (disp_helper_get_stage() != DISP_HELPER_STAGE_NORMAL)
		return;

	DISPMSG("[FB Driver] enter blank_resume\n");

	ret = primary_display_resume();

	if (ret) {
		DISPERR("primary display resume failed\n");
		return;
	}

	DISPMSG("[FB Driver] leave blank_resume\n");

}

static void mtkfb_blank_suspend(void)
{
	int ret = 0;

	if (disp_helper_get_stage() != DISP_HELPER_STAGE_NORMAL)
		return;

	DISPMSG("[FB Driver] enter blank_suspend\n");

	msleep(30);

	ret = primary_display_suspend();

	if (ret < 0) {
		DISPERR("primary display suspend failed\n");
		return;
	}

	DISPMSG("[FB Driver] leave blank_suspend\n");
}

/* fb_blank (FBIOBLANK) is wired unconditionally, as in the mt6735 mtkfb,
 * not only with CONFIG_PM_AUTOSLEEP=y. */
#if 1
static int mtkfb_blank(int blank_mode, struct fb_info *info)
{
	DISPERR("M6 mtkfb blank: mode=%d bypass=%d sleep=%d\n",
		blank_mode, bypass_blank, primary_display_is_sleepd());
	switch (blank_mode) {
	case FB_BLANK_UNBLANK:
	case FB_BLANK_NORMAL:
		if (bypass_blank) {
			DISPERR("FB_BLANK_UNBLANK bypass_blank %d\n", bypass_blank);
			break;
		}
		mtkfb_blank_resume();
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
		break;
	case FB_BLANK_POWERDOWN:
		if (bypass_blank) {
			DISPERR("FB_BLANK_POWERDOWN bypass_blank %d\n", bypass_blank);
			break;
		}
		mtkfb_blank_suspend();
		break;
	default:
		return -EINVAL;
	}

	return 0;
}
#endif

int mtkfb_set_backlight_level(unsigned int level)
{
#ifdef CONFIG_LOG_JANK
	static int bk_state=0;
#endif
	//MTKFB_FUNC();
#ifdef CONFIG_LOG_JANK
    if(level ==0)
    {
      bk_state=1;
	  /*qcom did't have this JLID_HWC_LCD_BACKLIGHT_OFF command*/
      //LOG_JANK_V(JLID_HWC_LCD_BACKLIGHT_OFF, "%s", "JL_HWC_LCD_BACKLIGHT_OFF");
    }
    if((bk_state==1)&&(level >0))
    {
      bk_state=0;
      LOG_JANK_D(JLID_KERNEL_LCD_BACKLIGHT_ON,"%s,%d", "JL_KERNEL_LCD_BACKLIGHT_ON",level);
    }
#endif
	DISPDBG("mtkfb_set_backlight_level:%d Start\n", level);
	primary_display_setbacklight(level);
	DISPDBG("mtkfb_set_backlight_level End\n");
	return 0;
}
EXPORT_SYMBOL(mtkfb_set_backlight_level);

int mtkfb_set_backlight_mode(unsigned int mode)
{
	MTKFB_FUNC();
	if (down_interruptible(&sem_flipping)) {
		DISPERR("[FB Driver] can't get semaphore:%d\n", __LINE__);
		return -ERESTARTSYS;
	}
	sem_flipping_cnt--;
	if (down_interruptible(&sem_early_suspend)) {
		DISPERR("[FB Driver] can't get semaphore:%d\n", __LINE__);
		sem_flipping_cnt++;
		up(&sem_flipping);
		return -ERESTARTSYS;
	}

	sem_early_suspend_cnt--;
	if (primary_display_is_sleepd())
		goto End;

	/* DISP_SetBacklight_mode(mode); */
End:
	sem_flipping_cnt++;
	sem_early_suspend_cnt++;
	up(&sem_early_suspend);
	up(&sem_flipping);
	return 0;
}
EXPORT_SYMBOL(mtkfb_set_backlight_mode);

int mtkfb_set_backlight_pwm(int div)
{
	MTKFB_FUNC();
	if (down_interruptible(&sem_flipping)) {
		DISPERR("[FB Driver] can't get semaphore:%d\n", __LINE__);
		return -ERESTARTSYS;
	}
	sem_flipping_cnt--;
	if (down_interruptible(&sem_early_suspend)) {
		DISPERR("[FB Driver] can't get semaphore:%d\n", __LINE__);
		sem_flipping_cnt++;
		up(&sem_flipping);
		return -ERESTARTSYS;
	}
	sem_early_suspend_cnt--;
	if (primary_display_is_sleepd())
		goto End;
	/* DISP_SetPWM(div); */
End:
	sem_flipping_cnt++;
	sem_early_suspend_cnt++;
	up(&sem_early_suspend);
	up(&sem_flipping);
	return 0;
}
EXPORT_SYMBOL(mtkfb_set_backlight_pwm);

int mtkfb_get_backlight_pwm(int div, unsigned int *freq)
{
	/* DISP_GetPWM(div, freq); */
	return 0;
}
EXPORT_SYMBOL(mtkfb_get_backlight_pwm);

void mtkfb_waitVsync(void)
{
	if (primary_display_is_sleepd()) {
		DISPMSG("[MTKFB_VSYNC]:mtkfb has suspend, return directly\n");
		msleep(20);
		return;
	}
	vsync_cnt++;
#ifdef CONFIG_MTK_FPGA
	msleep(20);
#else
	primary_display_wait_for_vsync(NULL);
#endif
	vsync_cnt--;
}
EXPORT_SYMBOL(mtkfb_waitVsync);

static int _convert_fb_layer_to_disp_input(struct fb_overlay_layer *src, disp_input_config *dst)
{

	dst->layer_id = src->layer_id;

	if (!src->layer_enable) {
		dst->layer_enable = 0;
		return 0;
	}

	switch (src->src_fmt) {
	case MTK_FB_FORMAT_YUV422:
		dst->src_fmt = DISP_FORMAT_YUV422;
		break;

	case MTK_FB_FORMAT_RGB565:
		dst->src_fmt = DISP_FORMAT_RGB565;
		break;

	case MTK_FB_FORMAT_RGB888:
		dst->src_fmt = DISP_FORMAT_RGB888;
		break;

	case MTK_FB_FORMAT_BGR888:
		dst->src_fmt = DISP_FORMAT_BGR888;
		break;

	case MTK_FB_FORMAT_ARGB8888:
		dst->src_fmt = DISP_FORMAT_ARGB8888;
		break;

	case MTK_FB_FORMAT_ABGR8888:
		dst->src_fmt = DISP_FORMAT_ABGR8888;
		break;

	case MTK_FB_FORMAT_XRGB8888:
		dst->src_fmt = DISP_FORMAT_XRGB8888;
		break;

	case MTK_FB_FORMAT_XBGR8888:
		dst->src_fmt = DISP_FORMAT_XBGR8888;
		break;

	case MTK_FB_FORMAT_UYVY:
		dst->src_fmt = DISP_FORMAT_UYVY;
		break;
	case MTK_FB_FORMAT_RGBA8888:
		dst->src_fmt = DISP_FORMAT_RGBA8888;
		break;

	case MTK_FB_FORMAT_BGRA8888:
		dst->src_fmt = DISP_FORMAT_BGRA8888;
		break;

	default:
		DISPERR("Invalid color format: 0x%x\n", src->src_fmt);
		return -1;
	}

	dst->src_base_addr = src->src_base_addr;
	dst->security = src->security;
	dst->src_phy_addr = src->src_phy_addr;
	DISPDBG("_convert_fb_layer_to_disp_input, dst->addr=0x%p\n", dst->src_phy_addr);

	dst->isTdshp = src->isTdshp;
	dst->next_buff_idx = src->next_buff_idx;
	dst->identity = src->identity;
	dst->connected_type = src->connected_type;

	/* set Alpha blending */
	dst->alpha = src->alpha;
	if (MTK_FB_FORMAT_ARGB8888 == src->src_fmt || MTK_FB_FORMAT_ABGR8888 == src->src_fmt)
		dst->alpha_enable = true;
	else
		dst->alpha_enable = false;


	/* set src width, src height */
	dst->src_offset_x = src->src_offset_x;
	dst->src_offset_y = src->src_offset_y;
	dst->src_width = src->src_width;
	dst->src_height = src->src_height;
	dst->tgt_offset_x = src->tgt_offset_x;
	dst->tgt_offset_y = src->tgt_offset_y;
	dst->tgt_width = src->tgt_width;
	dst->tgt_height = src->tgt_height;
	if (dst->tgt_width > dst->src_width)
		dst->tgt_width = dst->src_width;
	if (dst->tgt_height > dst->src_height)
		dst->tgt_height = dst->src_height;

	dst->src_pitch = src->src_pitch;

	/* set color key */
	dst->src_color_key = src->src_color_key;
	dst->src_use_color_key = src->src_use_color_key;

	/* data transferring is triggerred in MTKFB_TRIG_OVERLAY_OUT */
	dst->layer_enable = src->layer_enable;

#if 1
	DISPDBG("%s:id=%u,en=%u,next_idx=%u,vaddr=%p,pa=%p,srcfmt=%u,dstfmt=%u,pitch=%u,x=%u,y=%u,w=%u,h=%u\n",
	     __func__, dst->layer_id, dst->layer_enable, dst->next_buff_idx, dst->src_base_addr,
	     dst->src_phy_addr, src->src_fmt, dst->src_fmt, dst->src_pitch, dst->src_offset_x,
	     dst->src_offset_y, dst->src_width, dst->src_height);
	DISPDBG("%s:target xoff=%u, target yoff=%u, target w=%u, target h=%u, aen=%u\n",
	     __func__, dst->tgt_offset_x, dst->tgt_offset_y, dst->tgt_width, dst->tgt_height,
	     dst->alpha_enable);
#endif
	return 0;
}

static void m6_mtkfb_schedule_early_diag_report(void)
{
	/* was an unconditional 30s x10 DISPERR flood that wrapped the dmesg
	 * ring (hid early touch probe) and spammed logcat on a clean device.
	 * Display is up now; gate it off. Re-enable by raising the LIMIT. */
#if (M6_EARLY_FB_DIAG_REPORT_LIMIT == 0)
	return;
#endif

	if (m6_early_fb_diag_started)
		return;

	m6_early_fb_diag_started = true;
	schedule_delayed_work(&m6_mtkfb_early_diag_work_item,
			      msecs_to_jiffies(M6_EARLY_FB_DIAG_DELAY_MS));
}

static void m6_mtkfb_early_diag_work(struct work_struct *work)
{
	const struct m6_mtkfb_early_diag_state *d = &m6_early_fb_diag;
	unsigned int report = m6_early_fb_diag_reports++;

	DISPERR("M6 mtkfb early-diag[%u]: filled=%u pending=%u fb_trigger=%u const_attempt=%u const_trigger=%u bytes=%zu va=%p pa=0x%pa sample=%08x/%08x/%08x fb_ret=%d/%d const_ret=%d/%d yoff=%u line=%u fmt=0x%x pitch=%u wh=%u/%u layer=%u screen=%u/%u bpp=%u pages=%u fbline=%u\n",
		report, d->filled, d->marker_pending, d->fb_triggered,
		d->const_attempted, d->const_triggered, d->bytes, d->va,
		&d->pa, d->sample_first, d->sample_mid, d->sample_last,
		d->fb_cfg_ret, d->fb_trigger_ret, d->const_cfg_ret,
		d->const_trigger_ret, d->yoffset, d->line_length,
		d->fb_fmt, d->fb_pitch, d->width, d->height, d->layer_id,
		d->xres, d->yres, d->bpp, d->pages, d->line);

	if (m6_early_fb_diag_reports < M6_EARLY_FB_DIAG_REPORT_LIMIT)
		schedule_delayed_work(&m6_mtkfb_early_diag_work_item,
				      msecs_to_jiffies(M6_EARLY_FB_DIAG_DELAY_MS));
}

static void m6_mtkfb_capture_pipe_snapshot(struct m6_mtkfb_pipe_snapshot *snap)
{
	if (!snap)
		return;

	memset(snap, 0, sizeof(*snap));
	snap->valid = true;
	snap->dl_valid0 = DISP_REG_GET(DISP_REG_CONFIG_DISP_DL_VALID_0);
	snap->dl_ready0 = DISP_REG_GET(DISP_REG_CONFIG_DISP_DL_READY_0);
	snap->dl_valid1 = DISP_REG_GET(DISP_REG_CONFIG_DISP_DL_VALID_1);
	snap->dl_ready1 = DISP_REG_GET(DISP_REG_CONFIG_DISP_DL_READY_1);
	snap->mutex_inten = DISP_REG_GET(DISP_REG_CONFIG_MUTEX_INTEN);
	snap->mutex_intsta = DISP_REG_GET(DISP_REG_CONFIG_MUTEX_INTSTA);
	snap->mutex_en = DISP_REG_GET(DISP_REG_CONFIG_MUTEX_EN(0));
	snap->mutex_mod = DISP_REG_GET(DISP_REG_CONFIG_MUTEX_MOD(0));
	snap->mutex_sof = DISP_REG_GET(DISP_REG_CONFIG_MUTEX_SOF(0));
	snap->rdma_inten = DISP_REG_GET(DISP_REG_RDMA_INT_ENABLE);
	snap->rdma_intsta = DISP_REG_GET(DISP_REG_RDMA_INT_STATUS);
	snap->rdma_global = DISP_REG_GET(DISP_REG_RDMA_GLOBAL_CON);
	snap->rdma_size0 = DISP_REG_GET(DISP_REG_RDMA_SIZE_CON_0);
	snap->rdma_size1 = DISP_REG_GET(DISP_REG_RDMA_SIZE_CON_1);
	snap->rdma_target = DISP_REG_GET(DISP_REG_RDMA_TARGET_LINE);
	snap->rdma_mem_con = DISP_REG_GET(DISP_REG_RDMA_MEM_CON);
	snap->rdma_mem_start = DISP_REG_GET(DISP_REG_RDMA_MEM_START_ADDR);
	snap->rdma_pitch = DISP_REG_GET(DISP_REG_RDMA_MEM_SRC_PITCH);
	snap->rdma_fifo_con = DISP_REG_GET(DISP_REG_RDMA_FIFO_CON);
	snap->rdma_fifo_log = DISP_REG_GET(DISP_REG_RDMA_FIFO_LOG);
	snap->rdma_debug_sel = DISP_REG_GET(DISP_REG_RDMA_DEBUG_OUT_SEL);
	snap->rdma_in_p = DISP_REG_GET(DISP_REG_RDMA_IN_P_CNT);
	snap->rdma_in_l = DISP_REG_GET(DISP_REG_RDMA_IN_LINE_CNT);
	snap->rdma_out_p = DISP_REG_GET(DISP_REG_RDMA_OUT_P_CNT);
	snap->rdma_out_l = DISP_REG_GET(DISP_REG_RDMA_OUT_LINE_CNT);
	snap->cmdq_rdma_sof = cmdqCoreGetEvent(CMDQ_EVENT_DISP_RDMA0_SOF);
	snap->cmdq_rdma_eof = cmdqCoreGetEvent(CMDQ_EVENT_DISP_RDMA0_EOF);
	snap->cmdq_mutex0_eof = cmdqCoreGetEvent(CMDQ_EVENT_MUTEX0_STREAM_EOF);
	snap->cmdq_dsi0_sof = cmdqCoreGetEvent(CMDQ_EVENT_DISP_DSI0_SOF);
	snap->cmdq_dsi0_eof = cmdqCoreGetEvent(CMDQ_EVENT_DISP_DSI0_EOF);
	snap->cmdq_mdp_dsi0_te_sof = cmdqCoreGetEvent(CMDQ_EVENT_MDP_DSI0_TE_SOF);
}

static void m6_mtkfb_proc_print_pipe_snapshot(struct seq_file *m,
					      const char *tag,
					      const struct m6_mtkfb_pipe_snapshot *s)
{
	seq_printf(m, "%s_pipe valid=%u dl=0x%x/0x%x/0x%x/0x%x mutex=0x%x/0x%x/0x%x/0x%x/0x%x\n",
		tag, s->valid, s->dl_valid0, s->dl_ready0, s->dl_valid1,
		s->dl_ready1, s->mutex_inten, s->mutex_intsta, s->mutex_en,
		s->mutex_mod, s->mutex_sof);
	seq_printf(m, "%s_rdma valid=%u irq=0x%x/0x%x global=0x%x size=%u/%u target=%u mem=0x%x/0x%x pitch=%u fifo=0x%x/0x%x dbgsel=0x%x in=%u/%u out=%u/%u\n",
		tag, s->valid, s->rdma_inten, s->rdma_intsta, s->rdma_global,
		s->rdma_size0 & 0xfff, s->rdma_size1 & 0xfffff,
		s->rdma_target, s->rdma_mem_con, s->rdma_mem_start,
		s->rdma_pitch, s->rdma_fifo_con, s->rdma_fifo_log,
		s->rdma_debug_sel, s->rdma_in_p, s->rdma_in_l,
		s->rdma_out_p, s->rdma_out_l);
	seq_printf(m, "%s_cmdq valid=%u rdma_sof=%u rdma_eof=%u mutex0_eof=%u dsi0_sof=%u dsi0_eof=%u mdp_dsi0_te_sof=%u\n",
		tag, s->valid, s->cmdq_rdma_sof, s->cmdq_rdma_eof,
		s->cmdq_mutex0_eof, s->cmdq_dsi0_sof, s->cmdq_dsi0_eof,
		s->cmdq_mdp_dsi0_te_sof);
}

static void m6_mtkfb_proc_print_dsi_snapshot(struct seq_file *m,
					     const char *tag,
					     const struct m6_dsi_live_snapshot *s)
{
	seq_printf(m, "%s_dsi valid=%u start=0x%x sta=0x%x irq=0x%x/0x%x mode=0x%x txrx=0x%x ps=0x%x vm=0x%x bist=0x%x/0x%x dbgsel=0x%x\n",
		tag, s->valid, s->start, s->status, s->inten, s->intsta,
		s->mode, s->txrx, s->psctrl, s->vm_cmd, s->bist_pattern,
		s->bist_con, s->debug_sel);
	seq_printf(m, "%s_dsi_timing v=0x%x/0x%x/0x%x/0x%x h=0x%x/0x%x/0x%x/0x%x/0x%x phy=0x%x/0x%x/0x%x time=0x%x/0x%x/0x%x/0x%x\n",
		tag, s->vsa, s->vbp, s->vfp, s->vact, s->hsa, s->hbp,
		s->hfp, s->bllp, s->hstx_ckl, s->phy_lccon,
		s->phy_ld0con, s->phy_syncon, s->phy_timecon0,
		s->phy_timecon1, s->phy_timecon2, s->phy_timecon3);
	seq_printf(m, "%s_dsi_state dbg=0x%x/0x%x/0x%x/0x%x/0x%x/0x%x state=0x%x/0x%x/0x%x/0x%x vm_payload=0x%x/0x%x/0x%x/0x%x/0x%x/0x%x/0x%x/0x%x\n",
		tag, s->state_dbg[0], s->state_dbg[1], s->state_dbg[2],
		s->state_dbg[3], s->state_dbg[4], s->state_dbg[5],
		s->state_dbg[6], s->state_dbg[7], s->state_dbg[8],
		s->state_dbg[9], s->vm_payload[0], s->vm_payload[1],
		s->vm_payload[2], s->vm_payload[3], s->vm_payload[4],
		s->vm_payload[5], s->vm_payload[6], s->vm_payload[7]);
	seq_printf(m, "%s_mipitx lanes=0x%x/0x%x/0x%x/0x%x/0x%x top/bg/con=0x%x/0x%x/0x%x pll=0x%x/0x%x/0x%x/0x%x/0x%x/0x%x/0x%x rgs/gpi/pull/sel=0x%x/0x%x/0x%x/0x%x sw=0x%x/0x%x/0x%x dbg=0x%x/0x%x/0x%x\n",
		tag, s->mipitx_lane_c, s->mipitx_lane0, s->mipitx_lane1,
		s->mipitx_lane2, s->mipitx_lane3, s->mipitx_top,
		s->mipitx_bg, s->mipitx_con, s->mipitx_pll[0],
		s->mipitx_pll[1], s->mipitx_pll[2], s->mipitx_pll[3],
		s->mipitx_pll[4], s->mipitx_pll[5], s->mipitx_pll[6],
		s->mipitx_rgs, s->mipitx_gpi, s->mipitx_pull,
		s->mipitx_phy_sel, s->mipitx_sw_ctrl, s->mipitx_sw0,
		s->mipitx_sw1, s->mipitx_dbg, s->mipitx_apb,
		s->mipitx_apb_async);
}

static int m6_mtkfb_early_diag_proc_show(struct seq_file *m, void *v)
{
	const struct m6_mtkfb_early_diag_state *d = &m6_early_fb_diag;
	struct m6_mtkfb_pipe_snapshot live_pipe;
	struct m6_dsi_live_snapshot live_dsi;
	struct m6_ovl_config_snapshot snap;
	unsigned int layer;
	int have;

	m6_mtkfb_capture_pipe_snapshot(&live_pipe);
	dsi_m6_capture_live_snapshot(&live_dsi);

	seq_printf(m, "version=3 delay_ms=%u report_limit=%u reports=%u\n",
		M6_EARLY_FB_DIAG_DELAY_MS, M6_EARLY_FB_DIAG_REPORT_LIMIT,
		m6_early_fb_diag_reports);
	seq_printf(m, "filled=%u pending=%u fb_trigger=%u const_attempt=%u const_trigger=%u\n",
		d->filled, d->marker_pending, d->fb_triggered,
		d->const_attempted, d->const_triggered);
	seq_printf(m, "fb bytes=%zu va=%p pa=0x%pa sample=%08x/%08x/%08x screen=%u/%u bpp=%u pages=%u line=%u\n",
		d->bytes, d->va, &d->pa, d->sample_first, d->sample_mid,
		d->sample_last, d->xres, d->yres, d->bpp, d->pages, d->line);
	seq_printf(m, "handoff fb_ret=%d/%d const_ret=%d/%d yoff=%u line=%u fmt=0x%x pitch=%u wh=%u/%u layer=%u magic=0x%x\n",
		d->fb_cfg_ret, d->fb_trigger_ret, d->const_cfg_ret,
		d->const_trigger_ret, d->yoffset, d->line_length,
		d->fb_fmt, d->fb_pitch, d->width, d->height, d->layer_id,
		M6_OVL_CONST_WHITE_MAGIC_KEY);
	seq_printf(m, "const_ovl have=%u seq=%u enabled=0x%x first=%u scanned=0x%x/0x%x dst=%u/%u sec=%u cmdq=%u direct=%u bypass_pq=%u\n",
		d->const_ovl_snapshot_valid, d->const_ovl_snapshot.seq,
		d->const_ovl_snapshot.enabled_layers,
		d->const_ovl_snapshot.first_global_layer,
		d->const_ovl_snapshot.scanned_before,
		d->const_ovl_snapshot.scanned_after,
		d->const_ovl_snapshot.dst_w, d->const_ovl_snapshot.dst_h,
		d->const_ovl_snapshot.has_sec_layer, d->const_ovl_snapshot.cmdq,
		d->const_ovl_snapshot.direct, d->const_ovl_snapshot.bypass_pq);
	if (d->const_ovl_snapshot_valid) {
		for (layer = 0; layer < ARRAY_SIZE(d->const_ovl_snapshot.layer); layer++) {
			const struct m6_ovl_layer_snapshot *l =
				&d->const_ovl_snapshot.layer[layer];

			seq_printf(m, "const_ovl_l%u valid=%u en=%u global=%u source=%u larc=%u fmt=0x%x bpp=%u sec=%u alpha=%u/%u key=%u/0x%x con=0x%x clr=0x%x\n",
				layer, l->valid, l->enabled, l->global_layer,
				l->source, l->larc, l->fmt, l->bpp,
				l->security, l->aen, l->alpha, l->key_en,
				l->key, l->con, l->clr);
			seq_printf(m, "const_ovl_l%u src_xywh=%u/%u/%u/%u dst_xywh=%u/%u/%u/%u hw_dst_h=%u bounds=%u addr=0x%lx final=0x%lx visible_last=0x%lx pitch_end=0x%lx pitch=%u\n",
				layer, l->src_x, l->src_y, l->src_w, l->src_h,
				l->dst_x, l->dst_y, l->dst_w, l->dst_h,
				l->hw_dst_h, l->bounds_profile, l->addr,
				l->final_addr, l->visible_last, l->pitch_end,
				l->src_pitch);
		}
	}
	m6_mtkfb_proc_print_pipe_snapshot(m, "const_pre",
					  &d->const_pipe_pre);
	m6_mtkfb_proc_print_dsi_snapshot(m, "const_pre",
					 &d->const_dsi_pre);
	m6_mtkfb_proc_print_pipe_snapshot(m, "const_cfg",
					  &d->const_pipe_cfg);
	m6_mtkfb_proc_print_dsi_snapshot(m, "const_cfg",
					 &d->const_dsi_cfg);
	m6_mtkfb_proc_print_pipe_snapshot(m, "const_trigger",
					  &d->const_pipe_trigger);
	m6_mtkfb_proc_print_dsi_snapshot(m, "const_trigger",
					 &d->const_dsi_trigger);
	m6_mtkfb_proc_print_pipe_snapshot(m, "live", &live_pipe);
	m6_mtkfb_proc_print_dsi_snapshot(m, "live", &live_dsi);

	have = ovl_m6_get_last_config_snapshot(&snap);
	seq_printf(m, "ovl have=%d seq=%u enabled=0x%x first=%u scanned=0x%x/0x%x dst=%u/%u sec=%u cmdq=%u direct=%u bypass_pq=%u\n",
		have, have ? snap.seq : 0, have ? snap.enabled_layers : 0,
		have ? snap.first_global_layer : 0,
		have ? snap.scanned_before : 0, have ? snap.scanned_after : 0,
		have ? snap.dst_w : 0, have ? snap.dst_h : 0,
		have ? snap.has_sec_layer : 0, have ? snap.cmdq : 0,
		have ? snap.direct : 0, have ? snap.bypass_pq : 0);
	if (!have)
		return 0;

	for (layer = 0; layer < ARRAY_SIZE(snap.layer); layer++) {
		const struct m6_ovl_layer_snapshot *l = &snap.layer[layer];

		seq_printf(m, "ovl_l%u valid=%u en=%u global=%u source=%u larc=%u fmt=0x%x bpp=%u sec=%u alpha=%u/%u key=%u/0x%x con=0x%x clr=0x%x\n",
			layer, l->valid, l->enabled, l->global_layer,
			l->source, l->larc, l->fmt, l->bpp, l->security,
			l->aen, l->alpha, l->key_en, l->key, l->con, l->clr);
		seq_printf(m, "ovl_l%u src_xywh=%u/%u/%u/%u dst_xywh=%u/%u/%u/%u hw_dst_h=%u bounds=%u addr=0x%lx final=0x%lx visible_last=0x%lx pitch_end=0x%lx pitch=%u\n",
			layer, l->src_x, l->src_y, l->src_w, l->src_h,
			l->dst_x, l->dst_y, l->dst_w, l->dst_h,
			l->hw_dst_h, l->bounds_profile, l->addr,
			l->final_addr, l->visible_last, l->pitch_end,
			l->src_pitch);
	}

	return 0;
}

static int m6_mtkfb_early_diag_proc_open(struct inode *inode,
					 struct file *file)
{
	return single_open(file, m6_mtkfb_early_diag_proc_show, NULL);
}

static const struct file_operations m6_mtkfb_early_diag_proc_fops = {
	.owner = THIS_MODULE,
	.open = m6_mtkfb_early_diag_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static void m6_mtkfb_register_early_diag_proc(void)
{
	if (m6_early_fb_diag_proc_registered)
		return;

	if (!proc_create(M6_EARLY_FB_DIAG_PROC_NAME, S_IRUGO, NULL,
			 &m6_mtkfb_early_diag_proc_fops)) {
		DISPERR("M6 mtkfb early-diag: proc_create %s failed\n",
			M6_EARLY_FB_DIAG_PROC_NAME);
		return;
	}
	m6_early_fb_diag_proc_registered = true;
	DISPERR("M6 mtkfb early-diag: proc /proc/%s registered\n",
		M6_EARLY_FB_DIAG_PROC_NAME);
}

static void m6_mtkfb_unregister_early_diag_proc(void)
{
	if (!m6_early_fb_diag_proc_registered)
		return;

	remove_proc_entry(M6_EARLY_FB_DIAG_PROC_NAME, NULL);
	m6_early_fb_diag_proc_registered = false;
}

static void __maybe_unused m6_mtkfb_record_fb_trigger(const struct mtkfb_device *fbdev,
				       const struct fb_info *fbi,
				       const struct fb_overlay_layer *fb_layer,
				       int cfg_ret, int trigger_ret)
{
	struct m6_mtkfb_early_diag_state *d = &m6_early_fb_diag;

	d->fb_triggered = true;
	d->fb_cfg_ret = cfg_ret;
	d->fb_trigger_ret = trigger_ret;
	if (fbdev) {
		d->va = fbdev->fb_va_base;
		d->pa = fbdev->fb_pa_base;
		d->bytes = fbdev->fb_size_in_byte;
	}
	if (fbi) {
		d->yoffset = fbi->var.yoffset;
		d->line_length = fbi->fix.line_length;
	}
	if (fb_layer) {
		d->fb_fmt = fb_layer->src_fmt;
		d->fb_pitch = fb_layer->src_pitch;
		d->width = fb_layer->src_width;
		d->height = fb_layer->src_height;
		d->layer_id = fb_layer->layer_id;
	}
	m6_mtkfb_schedule_early_diag_report();
}

static void __maybe_unused m6_mtkfb_config_const_white_marker(struct fb_info *fbi,
					       const struct fb_overlay_layer *fb_layer)
{
#if M6_EARLY_FB_CONST_LAYER_ISOLATION
	struct m6_mtkfb_early_diag_state *d = &m6_early_fb_diag;
	disp_session_input_config *const_session;
	disp_input_config *input;
	struct fb_var_screeninfo *var;
	int cfg_ret = -ENOMEM;
	int trigger_ret = 0;
	u8 layer_id;

	if (!fbi || !d->marker_pending || d->const_triggered)
		return;

	var = &fbi->var;
	layer_id = fb_layer ? fb_layer->layer_id :
		primary_display_get_option("FB_LAYER");
	d->const_attempted = true;
	m6_mtkfb_capture_pipe_snapshot(&d->const_pipe_pre);
	dsi_m6_capture_live_snapshot(&d->const_dsi_pre);

	const_session = kzalloc(sizeof(*const_session), GFP_KERNEL);
	if (!const_session)
		goto out;

	if (!is_DAL_Enabled()) {
		input = &const_session->config[const_session->config_layer_num++];
		input->layer_id = primary_display_get_option("ASSERT_LAYER");
		input->layer_enable = 0;
		input->next_buff_idx = -1;
	}

	input = &const_session->config[const_session->config_layer_num++];
	input->layer_id = layer_id;
	input->layer_enable = 1;
	input->buffer_source = DISP_BUFFER_ALPHA;
	input->security = DISP_NORMAL_BUFFER;
	input->src_fmt = DISP_FORMAT_RGB888;
	input->src_alpha = DISP_ALPHA_ONE;
	input->dst_alpha = DISP_ALPHA_ONE;
	input->yuv_range = DISP_YUV_BT601_FULL;
	input->layer_rotation = DISP_ORIENTATION_0;
	input->layer_type = DISP_LAYER_2D;
	input->video_rotation = DISP_ORIENTATION_0;
	input->next_buff_idx = -1;
	input->src_color_key = M6_OVL_CONST_WHITE_MAGIC_KEY;
	input->src_pitch = var->xres;
	input->src_width = var->xres;
	input->src_height = var->yres;
	input->tgt_width = var->xres;
	input->tgt_height = var->yres;
	input->alpha_enable = 1;
	input->alpha = 0xff;
	input->sur_aen = 0;
	input->src_use_color_key = 0;
	input->src_direct_link = 0;

	cfg_ret = primary_display_config_input_multiple(const_session);
	m6_mtkfb_capture_pipe_snapshot(&d->const_pipe_cfg);
	dsi_m6_capture_live_snapshot(&d->const_dsi_cfg);
	if (!cfg_ret)
		d->const_ovl_snapshot_valid =
			!!ovl_m6_get_last_config_snapshot(&d->const_ovl_snapshot);
	trigger_ret = primary_display_trigger(1, NULL, 0);
	m6_mtkfb_capture_pipe_snapshot(&d->const_pipe_trigger);
	dsi_m6_capture_live_snapshot(&d->const_dsi_trigger);
	d->const_triggered = true;
	kfree(const_session);

out:
	d->const_cfg_ret = cfg_ret;
	d->const_trigger_ret = trigger_ret;
	DISPERR("M6 mtkfb const-white-trigger: cfg_ret=%d trigger_ret=%d layer=%u key=0x%x wh=%u/%u pitch=%u yoff=%u line=%u\n",
		cfg_ret, trigger_ret, layer_id, M6_OVL_CONST_WHITE_MAGIC_KEY,
		var->xres, var->yres, var->xres, var->yoffset,
		fbi->fix.line_length);
	aee_sram_printk("M6F const cfg=%d tr=%d l=%u wh=%u/%u key=%x\n",
		cfg_ret, trigger_ret, layer_id, var->xres, var->yres,
		M6_OVL_CONST_WHITE_MAGIC_KEY);
	m6_mtkfb_schedule_early_diag_report();
#endif
}
#if 0
static int _overlay_info_convert(struct fb_overlay_layer *src, OVL_CONFIG_STRUCT *dst)
{
	unsigned int layerpitch = 0;
	unsigned int layerbpp = 0;

	dst->layer = src->layer_id;

	if (!src->layer_enable) {
		dst->layer_en = 0;
		dst->isDirty = true;
		return 0;
	}

	switch (src->src_fmt) {
	case MTK_FB_FORMAT_YUV422:
		dst->fmt = UFMT_YUYV;
		layerpitch = 2;
		layerbpp = 16;
		break;

	case MTK_FB_FORMAT_RGB565:
		dst->fmt = UFMT_RGB565;
		layerpitch = 2;
		layerbpp = 16;
		break;

	case MTK_FB_FORMAT_RGB888:
		dst->fmt = UFMT_RGB888;
		layerpitch = 3;
		layerbpp = 24;
		break;

	case MTK_FB_FORMAT_BGR888:
		dst->fmt = UFMT_BGR888;
		layerpitch = 3;
		layerbpp = 24;
		break;

	case MTK_FB_FORMAT_ARGB8888:
		dst->fmt = UFMT_ARGB8888;
		layerpitch = 4;
		layerbpp = 32;
		break;

	case MTK_FB_FORMAT_ABGR8888:
		/* dst->fmt = eABGR8888; */
		dst->fmt = UFMT_ABGR8888;
		layerpitch = 4;
		layerbpp = 32;
		break;
	case MTK_FB_FORMAT_XRGB8888:
		dst->fmt = UFMT_XRGB8888;
		layerpitch = 4;
		layerbpp = 32;
		break;

	case MTK_FB_FORMAT_XBGR8888:
		dst->fmt = UFMT_XBGR8888;
		layerpitch = 4;
		layerbpp = 32;
		break;

	case MTK_FB_FORMAT_UYVY:
		dst->fmt = UFMT_UYVY;
		layerpitch = 2;
		layerbpp = 16;
		break;

	default:
		DISPERR("Invalid color format: 0x%x\n", src->src_fmt);
		return -1;
	}

	dst->vaddr = (unsigned long)src->src_base_addr;
	dst->security = src->security;
/* set overlay will not use fence+ion handle */
#if 0
/* #if defined (MTK_FB_ION_SUPPORT) */
	if (src->src_phy_addr != NULL)
		dst->addr = (unsigned int)src->src_phy_addr;
	else
		dst->addr = mtkfb_query_buf_mva(src->layer_id, (unsigned int)src->next_buff_idx);

#else
	dst->addr = (unsigned long)src->src_phy_addr;
#endif
	dst->isTdshp = src->isTdshp;
	dst->buff_idx = src->next_buff_idx;
	dst->identity = src->identity;
	dst->connected_type = src->connected_type;

	/* set Alpha blending */
	dst->alpha = src->alpha;
	if (MTK_FB_FORMAT_ARGB8888 == src->src_fmt || MTK_FB_FORMAT_ABGR8888 == src->src_fmt)
		dst->aen = TRUE;
	else
		dst->aen = FALSE;


	/* set src width, src height */
	dst->src_x = src->src_offset_x;
	dst->src_y = src->src_offset_y;
	dst->src_w = src->src_width;
	dst->src_h = src->src_height;
	dst->dst_x = src->tgt_offset_x;
	dst->dst_y = src->tgt_offset_y;
	dst->dst_w = src->tgt_width;
	dst->dst_h = src->tgt_height;
	if (dst->dst_w > dst->src_w)
		dst->dst_w = dst->src_w;
	if (dst->dst_h > dst->src_h)
		dst->dst_h = dst->src_h;

	dst->src_pitch = src->src_pitch * layerpitch;
	/* set color key */
	dst->key = src->src_color_key;
	dst->keyEn = src->src_use_color_key;
	/* data transferring is triggerred in MTKFB_TRIG_OVERLAY_OUT */
	dst->layer_en = src->layer_enable;
	dst->isDirty = true;

#if 1
	DISPMSG("%s:id=%u,en=%u,next_idx=%u,vaddr=0x%lx,paddr=0x%lx,fmt=%u,pitch=%u,xoff=%u,yoff=%u,w=%u,h=%u\n",
		__func__, dst->layer, dst->layer_en, dst->buff_idx, dst->addr, dst->vaddr, dst->fmt,
		dst->src_pitch, dst->src_x, dst->src_y, dst->src_w, dst->src_h);
	DISPMSG("%s:target xoff=%u, target yoff=%u, target w=%u, target h=%u, aen=%u\n",
	     __func__, dst->dst_x, dst->dst_y, dst->dst_w, dst->dst_h, dst->aen);
#endif

	return 0;
}
#endif
static int mtkfb_pan_display_impl(struct fb_var_screeninfo *var, struct fb_info *info)
{
	uint32_t offset = 0;
	uint32_t paStart = 0;
	char *vaStart = NULL, *vaEnd = NULL;
	int ret = 0;
	unsigned int src_pitch = 0;
	disp_session_input_config *session_input;
	disp_input_config *input;

	/* DISPFUNC(); */

	if (no_update) {
		DISPMSG("the first time of mtkfb_pan_display_impl will be ignored\n");
		return ret;
	}

	DISPDBG("pan_display: offset(%u,%u), res(%u,%u), resv(%u,%u)\n",
		  var->xoffset, var->yoffset, info->var.xres, info->var.yres,
		  info->var.xres_virtual, info->var.yres_virtual);

	info->var.yoffset = var->yoffset;
	offset = var->yoffset * info->fix.line_length;
	paStart = fb_pa + offset;
	vaStart = info->screen_base + offset;
	vaEnd = vaStart + info->var.yres * info->fix.line_length;

	session_input = kzalloc(sizeof(*session_input), GFP_KERNEL);
	if (!session_input)
		BUG();

	/* pan display use layer 0 */
	input = &session_input->config[0];
	input->layer_id = 0;
	input->src_phy_addr = (void *)((unsigned long)paStart);
	input->src_base_addr = (void *)((unsigned long)vaStart);
	input->layer_id = primary_display_get_option("FB_LAYER");
	input->layer_enable = 1;
	input->src_offset_x = 0;
	input->src_offset_y = 0;
	input->src_width = var->xres;
	input->src_height = var->yres;
	input->tgt_offset_x = 0;
	input->tgt_offset_y = 0;
	input->tgt_width = var->xres;
	input->tgt_height = var->yres;

	switch (var->bits_per_pixel) {
	case 16:
		input->src_fmt = DISP_FORMAT_RGB565;
		break;
	case 24:
		input->src_fmt = DISP_FORMAT_RGB888;
		break;
	case 32:
		input->src_fmt =
		    (0 == var->blue.offset) ? DISP_FORMAT_BGRA8888 : DISP_FORMAT_RGBX8888;

		break;
	default:
		DISPERR("Invalid color format bpp: 0x%d\n", var->bits_per_pixel);
		kfree(session_input);
		return -1;
	}
	input->alpha_enable = false;

	input->alpha = 0xFF;
	input->next_buff_idx = -1;
	src_pitch = ALIGN_TO(var->xres, MTK_FB_ALIGNMENT);
	input->src_pitch = src_pitch;

	session_input->config_layer_num++;

	if (!is_DAL_Enabled()) {
		/* disable font layer(layer3) drawed in lk */
		session_input->config[1].layer_id = primary_display_get_option("ASSERT_LAYER");
		session_input->config[1].next_buff_idx = -1;
		session_input->config[1].layer_enable = 0;
		session_input->config_layer_num++;
	}
	ret = primary_display_config_input_multiple(session_input);
	ret = primary_display_trigger(true, NULL, 0);

	kfree(session_input);

	return ret;
}


/* Set fb_info.fix fields and also updates fbdev.
 * When calling this fb_info.var must be set up already.
 */
static void set_fb_fix(struct mtkfb_device *fbdev)
{
	struct fb_info *fbi = fbdev->fb_info;
	struct fb_fix_screeninfo *fix = &fbi->fix;
	struct fb_var_screeninfo *var = &fbi->var;
	struct fb_ops *fbops = fbi->fbops;

	strncpy(fix->id, MTKFB_DRIVER, sizeof(fix->id));
	fix->type = FB_TYPE_PACKED_PIXELS;

	switch (var->bits_per_pixel) {
	case 16:
	case 24:
	case 32:
		fix->visual = FB_VISUAL_TRUECOLOR;
		break;
	case 1:
	case 2:
	case 4:
	case 8:
		fix->visual = FB_VISUAL_PSEUDOCOLOR;
		break;
	default:
		ASSERT(0);
	}

	fix->accel = FB_ACCEL_NONE;
	fix->line_length = ALIGN_TO(var->xres_virtual, MTK_FB_ALIGNMENT) * var->bits_per_pixel / 8;
	fix->smem_len = fbdev->fb_size_in_byte;
	fix->smem_start = fbdev->fb_pa_base;

	fix->xpanstep = 0;
	fix->ypanstep = 1;

	fbops->fb_fillrect = cfb_fillrect;
	fbops->fb_copyarea = cfb_copyarea;
	fbops->fb_imageblit = cfb_imageblit;
}


/* Check values in var, try to adjust them in case of out of bound values if
 * possible, or return error.
 */
static int mtkfb_check_var(struct fb_var_screeninfo *var, struct fb_info *fbi)
{
	unsigned int bpp;
	unsigned long max_frame_size;
	unsigned long line_size;

	struct mtkfb_device *fbdev = (struct mtkfb_device *)fbi->par;

	/* DISPFUNC(); */

	DISPDBG("mtkfb_check_var,xres=%u,yres=%u,x_virt=%u,y_virt=%u,xoffset=%u,yoffset=%u,bits_per_pixel=%u)\n",
		  var->xres, var->yres, var->xres_virtual, var->yres_virtual,
		  var->xoffset, var->yoffset, var->bits_per_pixel);

	bpp = var->bits_per_pixel;

	if (bpp != 16 && bpp != 24 && bpp != 32) {
		MTKFB_LOG("[%s]unsupported bpp: %d", __func__, bpp);
		return -1;
	}

	switch (var->rotate) {
	case 0:
	case 180:
		var->xres = MTK_FB_XRES;
		var->yres = MTK_FB_YRES;
		break;
	case 90:
	case 270:
		var->xres = MTK_FB_YRES;
		var->yres = MTK_FB_XRES;
		break;
	default:
		return -1;
	}

	if (var->xres_virtual < var->xres)
		var->xres_virtual = var->xres;
	if (var->yres_virtual < var->yres)
		var->yres_virtual = var->yres;

	max_frame_size = fbdev->fb_size_in_byte;
	DISPDBG("fbdev->fb_size_in_byte=0x%08lx\n", fbdev->fb_size_in_byte);
	line_size = var->xres_virtual * bpp / 8;

	if (line_size * var->yres_virtual > max_frame_size) {
		/* Try to keep yres_virtual first */
		line_size = max_frame_size / var->yres_virtual;
		var->xres_virtual = line_size * 8 / bpp;
		if (var->xres_virtual < var->xres) {
			/* Still doesn't fit. Shrink yres_virtual too */
			var->xres_virtual = var->xres;
			line_size = var->xres * bpp / 8;
			var->yres_virtual = max_frame_size / line_size;
		}
	}
	DISPDBG("mtkfb_check_var,xres=%u,yres=%u,x_virt=%u,y_virl=%u,xoffset=%u,yoffset=%u,bits_per_pixel=%u)\n",
		var->xres, var->yres, var->xres_virtual, var->yres_virtual,
		var->xoffset, var->yoffset, var->bits_per_pixel);
	if (var->xres + var->xoffset > var->xres_virtual)
		var->xoffset = var->xres_virtual - var->xres;
	if (var->yres + var->yoffset > var->yres_virtual)
		var->yoffset = var->yres_virtual - var->yres;

	DISPDBG("mtkfb_check_var,xres=%u,yres=%u,x_virt=%u,y_virt=%u,xoffset=%u,yoffset=%u,bits_per_pixel=%u)\n",
		var->xres, var->yres, var->xres_virtual, var->yres_virtual,
		var->xoffset, var->yoffset, var->bits_per_pixel);

	if (16 == bpp) {
		var->red.offset = 11;
		var->red.length = 5;
		var->green.offset = 5;
		var->green.length = 6;
		var->blue.offset = 0;
		var->blue.length = 5;
		var->transp.offset = 0;
		var->transp.length = 0;
	} else if (24 == bpp) {
		var->red.length = var->green.length = var->blue.length = 8;
		var->transp.length = 0;

		/* Check if format is RGB565 or BGR565 */
		if((8 != var->green.offset) ||
		(16 != var->red.offset + var->blue.offset) ||
		(16 != var->red.offset && 0 != var->red.offset))
		return -EINVAL;

	} else if (32 == bpp) {
		var->red.length = var->green.length = var->blue.length = var->transp.length = 8;

		/* Check if format is ARGB565 or ABGR565 */
		if(8 != var->green.offset || 24 != var->transp.offset ||
		(16 != var->red.offset + var->blue.offset) ||
		(16 != var->red.offset && 0 != var->red.offset))
		return -EINVAL;
	}

	var->red.msb_right = var->green.msb_right = var->blue.msb_right = var->transp.msb_right = 0;

	if (var->activate & FB_ACTIVATE_NO_UPDATE)
		no_update = true;
	else
		no_update = false;


	var->activate = FB_ACTIVATE_NOW;

	var->height = UINT_MAX;
	var->width = UINT_MAX;
	var->grayscale = 0;
	var->nonstd = 0;

	var->pixclock = UINT_MAX;
	var->left_margin = UINT_MAX;
	var->right_margin = UINT_MAX;
	var->upper_margin = UINT_MAX;
	var->lower_margin = UINT_MAX;
	var->hsync_len = UINT_MAX;
	var->vsync_len = UINT_MAX;

	var->vmode = FB_VMODE_NONINTERLACED;
	var->sync = 0;

	MSG_FUNC_LEAVE();
	return 0;
}



/* Switch to a new mode. The parameters for it has been check already by
 * mtkfb_check_var.
 */
static int mtkfb_set_par(struct fb_info *fbi)
{
	struct fb_var_screeninfo *var = &fbi->var;
	struct mtkfb_device *fbdev = (struct mtkfb_device *)fbi->par;
	struct fb_overlay_layer fb_layer;
	u32 bpp = var->bits_per_pixel;
	disp_session_input_config *session_input;
	disp_input_config *input;

	/* DISPFUNC(); */
	memset(&fb_layer, 0, sizeof(struct fb_overlay_layer));
	switch (bpp) {
	case 16:
		fb_layer.src_fmt = MTK_FB_FORMAT_RGB565;
		fb_layer.src_use_color_key = 1;
		fb_layer.src_color_key = 0xFF000000;
		break;

	case 24:
		fb_layer.src_use_color_key = 1;
		fb_layer.src_fmt = (0 == var->blue.offset) ?
		    MTK_FB_FORMAT_RGB888 : MTK_FB_FORMAT_BGR888;
		fb_layer.src_color_key = 0xFF000000;
		break;

	case 32:
		fb_layer.src_use_color_key = 0;
		DISPDBG("set_par,var->blue.offset=%d\n", var->blue.offset);
		fb_layer.src_fmt = (0 == var->blue.offset) ?
		    MTK_FB_FORMAT_RGBA8888 : MTK_FB_FORMAT_BGRA8888;
		fb_layer.src_color_key = 0;
		break;

	default:
		fb_layer.src_fmt = MTK_FB_FORMAT_UNKNOWN;
		DISPERR("[%s]unsupported bpp: %d", __func__, bpp);
		return -1;
	}


	set_fb_fix(fbdev);

	fb_layer.layer_id = primary_display_get_option("FB_LAYER");
	fb_layer.layer_enable = 1;
	fb_layer.src_base_addr =
	    (void *)((unsigned long)fbdev->fb_va_base + var->yoffset * fbi->fix.line_length);
	DISPDBG("fb_pa=0x%08lx, var->yoffset=0x%08x,fbi->fix.line_length=0x%08x\n", fb_pa,
		var->yoffset, fbi->fix.line_length);
	fb_layer.src_phy_addr = (void *)(fb_pa + var->yoffset * fbi->fix.line_length);
	fb_layer.src_direct_link = 0;
	fb_layer.src_offset_x = fb_layer.src_offset_y = 0;
	fb_layer.src_pitch = ALIGN_TO(var->xres, MTK_FB_ALIGNMENT);
	fb_layer.src_width = fb_layer.tgt_width = var->xres;
	fb_layer.src_height = fb_layer.tgt_height = var->yres;
	fb_layer.tgt_offset_x = fb_layer.tgt_offset_y = 0;
	fb_layer.alpha = 0xff;
	/* fb_layer.src_color_key = 0; */
	fb_layer.layer_rotation = MTK_FB_ORIENTATION_0;
	fb_layer.layer_type = LAYER_2D;
	DISPDBG("mtkfb_set_par, fb_layer.src_fmt=%x\n", fb_layer.src_fmt);

	session_input = kzalloc(sizeof(*session_input), GFP_KERNEL);
	if (!session_input)
		goto out;

	session_input->config_layer_num = 0;

	if (!is_DAL_Enabled()) {
		DISPDBG("AEE is not enabled, will disable layer 3\n");
		input = &session_input->config[session_input->config_layer_num++];
		input->layer_id = primary_display_get_option("ASSERT_LAYER");
		input->layer_enable = 0;
	} else {
		DISPDBG("AEE is enabled, should not disable layer 3\n");
	}

	input = &session_input->config[session_input->config_layer_num++];
	_convert_fb_layer_to_disp_input(&fb_layer, input);
	{
		int cfg_ret;
		int __maybe_unused trigger_ret = 0;

		cfg_ret = primary_display_config_input_multiple(session_input);
#if M6_EARLY_FB_WHITE_MARKER_TRIGGER
		if (m6_early_fb_diag.marker_pending &&
		    !m6_early_fb_diag.fb_triggered) {
			trigger_ret = primary_display_trigger(1, NULL, 0);
			m6_mtkfb_record_fb_trigger(fbdev, fbi, &fb_layer,
						   cfg_ret, trigger_ret);
			DISPERR("M6 mtkfb fb-marker-trigger: cfg_ret=%d trigger_ret=%d pa=0x%pa yoff=%u line=%u fmt=0x%x pitch=%u wh=%u/%u\n",
				cfg_ret, trigger_ret, &fbdev->fb_pa_base,
				var->yoffset, fbi->fix.line_length,
				fb_layer.src_fmt, fb_layer.src_pitch,
				fb_layer.src_width, fb_layer.src_height);
			aee_sram_printk("M6F trig cfg=%d tr=%d y=%u fmt=%x wh=%u/%u\n",
				cfg_ret, trigger_ret, var->yoffset,
				fb_layer.src_fmt, fb_layer.src_width,
				fb_layer.src_height);
			m6_mtkfb_config_const_white_marker(fbi, &fb_layer);
		}
#endif
	}
	kfree(session_input);

out:
	/* backup fb_layer information. */
	memcpy(&fb_layer_context, &fb_layer, sizeof(fb_layer));

	MSG_FUNC_LEAVE();
	return 0;
}


static int mtkfb_soft_cursor(struct fb_info *info, struct fb_cursor *cursor)
{

	return 0;
}

static int mtkfb_get_overlay_layer_info(struct fb_overlay_layer_info *layerInfo)
{
	return 0;
}

void mtkfb_dump_layer_info(void)
{
}


uint32_t color = 0;
unsigned int mtkfb_fm_auto_test(void)
{
	unsigned int result = 0;
	unsigned int i = 0;
	unsigned long fbVirAddr;
	uint32_t fbsize;
	int r = 0;
	unsigned int *fb_buffer;
	struct mtkfb_device *fbdev = (struct mtkfb_device *)mtkfb_fbi->par;
	struct fb_var_screeninfo var;

	fbVirAddr = (unsigned long)fbdev->fb_va_base;
	fb_buffer = (unsigned int *)fbVirAddr;
	memcpy(&var, &(mtkfb_fbi->var), sizeof(var));
	var.activate = FB_ACTIVATE_NOW;
	var.bits_per_pixel = 32;
	var.transp.offset = 24;
	var.transp.length = 8;
	var.red.offset = 16;
	var.red.length = 8;
	var.green.offset = 8;
	var.green.length = 8;
	var.blue.offset = 0;
	var.blue.length = 8;

	r = mtkfb_check_var(&var, mtkfb_fbi);
	if (r != 0)
		PRNERR("failed to mtkfb_check_var\n");

	mtkfb_fbi->var = var;

#if 0
	r = mtkfb_set_par(mtkfb_fbi);

	if (r != 0)
		PRNERR("failed to mtkfb_set_par\n");
#endif
	if (color == 0)
		color = 0xFF00FF00;
	fbsize =
	    ALIGN_TO(DISP_GetScreenWidth(),
		     MTK_FB_ALIGNMENT) * DISP_GetScreenHeight() * MTK_FB_PAGES;
	for (i = 0; i < fbsize; i++)
		*fb_buffer++ = color;
#if 0
	if (!primary_display_is_video_mode())
		primary_display_trigger(1, NULL, 0);
#endif
	mtkfb_pan_display_impl(&mtkfb_fbi->var, mtkfb_fbi);
	msleep(100);

	result = primary_display_lcm_ATA();

	if (result == 0)
		DISPERR("ATA LCM failed\n");
	else
		DISPMSG("ATA LCM passed\n");


	return result;
}



static int mtkfb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	DISP_STATUS ret = 0;
	int r = 0;

	/* DISPFUNC(); */
	/* / M: dump debug mmprofile log info */
	DISPDBG("mtkfb_ioctl, info=%p, cmd nr=0x%08x, cmd size=0x%08x\n", info,
		(unsigned int)_IOC_NR(cmd), (unsigned int)_IOC_SIZE(cmd));

	switch (cmd) {

	case MTKFB_GET_FRAMEBUFFER_MVA:
		return copy_to_user(argp, &fb_pa, sizeof(fb_pa)) ? -EFAULT : 0;
		/* remain this for engineer mode dfo multiple resolution */

	case MTKFB_GET_DISPLAY_IF_INFORMATION:
		{
			int displayid = 0;

			if (copy_from_user(&displayid, (void __user *)arg, sizeof(displayid))) {
				MTKFB_LOG("[FB]: copy_from_user failed! line:%d\n", __LINE__);
				return -EFAULT;
			}

			if (displayid > (MTKFB_MAX_DISPLAY_COUNT - 1) || displayid < 0) {
				DISPERR("[FB]: invalid display id:%d\n", displayid);
				return -EFAULT;
			}

			if (displayid == 0) {
				dispif_info[displayid].displayWidth = primary_display_get_width();
				dispif_info[displayid].displayHeight = primary_display_get_height();

				dispif_info[displayid].lcmOriginalWidth =
				    primary_display_get_original_width();
				dispif_info[displayid].lcmOriginalHeight =
				    primary_display_get_original_height();
				dispif_info[displayid].displayMode =
				    primary_display_is_video_mode() ? 0 : 1;
			} else {
				DISPERR("information for displayid: %d is not available now\n",
					displayid);
				return -EFAULT;
			}

			if (copy_to_user((void __user *)arg, &(dispif_info[displayid]), sizeof(mtk_dispif_info_t))) {
				MTKFB_LOG("[FB]: copy_to_user failed! line:%d\n", __LINE__);
				r = -EFAULT;
			}

			return r;
		}

	case MTKFB_POWEROFF:
		{
			MTKFB_FUNC();
			if (primary_display_is_sleepd()) {
				DISPMSG("[FB Driver] is still in MTKFB_POWEROFF!!!\n");
				return r;
			}

			DISPMSG("[FB Driver] enter MTKFB_POWEROFF\n");
			ret = primary_display_suspend();
			if (ret < 0)
				DISPERR("primary display suspend failed\n");
			DISPMSG("[FB Driver] leave MTKFB_POWEROFF\n");

			is_early_suspended = true;	/* no care */
			return r;
		}

	case MTKFB_POWERON:
		{
			MTKFB_FUNC();
			if (primary_display_is_alive()) {
				DISPMSG("[FB Driver] is still in MTKFB_POWERON!!!\n");
				return r;
			}
			DISPMSG("[FB Driver] enter MTKFB_POWERON\n");
			primary_display_resume();
			DISPMSG("[FB Driver] leave MTKFB_POWERON\n");
			is_early_suspended = false;	/* no care */
			return r;
		}
	case MTKFB_GET_POWERSTATE:
		{
			int power_state;

			if (primary_display_is_sleepd())
				power_state = 0;
			else
				power_state = 1;

			if (copy_to_user(argp, &power_state, sizeof(power_state))) {
				pr_err("MTKFB_GET_POWERSTATE failed\n");
				return -EFAULT;
			}

			return 0;
		}

	case MTKFB_CONFIG_IMMEDIATE_UPDATE:
		{
			MTKFB_LOG("[%s] MTKFB_CONFIG_IMMEDIATE_UPDATE, enable = %lu\n", __func__,
				  arg);
			if (down_interruptible(&sem_early_suspend)) {
				MTKFB_LOG("[mtkfb_ioctl] can't get semaphore:%d\n", __LINE__);
				return -ERESTARTSYS;
			}
			sem_early_suspend_cnt--;
			/* DISP_WaitForLCDNotBusy(); */
			/* ret = DISP_ConfigImmediateUpdate((BOOL)arg); */
			/* sem_early_suspend_cnt++; */
			up(&sem_early_suspend);
			return r;
		}

	case MTKFB_CAPTURE_FRAMEBUFFER:
		{
			unsigned long dst_pbuf = 0;
			unsigned long *src_pbuf = NULL;
			unsigned int pixel_bpp = info->var.bits_per_pixel / 8;
			unsigned int fbsize = DISP_GetScreenHeight() * DISP_GetScreenWidth() * pixel_bpp;

			if (copy_from_user(&dst_pbuf, (void __user *)arg, sizeof(dst_pbuf))) {
				MTKFB_LOG("[FB]: copy_from_user failed! line:%d\n", __LINE__);
				r = -EFAULT;
			} else {
				src_pbuf = vmalloc(fbsize);
				if (!src_pbuf) {
					MTKFB_LOG("[FB]: vmalloc capture src_pbuf failed! line:%d\n", __LINE__);
					r = -EFAULT;
				} else {
					dprec_logger_start(DPREC_LOGGER_WDMA_DUMP, 0, 0);
					primary_display_capture_framebuffer_ovl((unsigned long)src_pbuf, UFMT_BGRA8888);
					dprec_logger_done(DPREC_LOGGER_WDMA_DUMP, 0, 0);
					if (copy_to_user((unsigned long *)dst_pbuf, src_pbuf, fbsize)) {
						MTKFB_LOG("[FB]: copy_to_user failed! line:%d\n", __LINE__);
						r = -EFAULT;
					}
				}
				vfree(src_pbuf);
			}

			return r;
		}

	case MTKFB_SLT_AUTO_CAPTURE:
		{
			struct fb_slt_catpure capConfig;
			char *dst_buffer;
			unsigned int fb_size;

			if (copy_from_user(&capConfig, (void __user *)arg, sizeof(capConfig))) {
				MTKFB_LOG("[FB]: copy_from_user failed! line:%d\n", __LINE__);
				r = -EFAULT;
			} else {
				unsigned int format;

				switch (capConfig.format) {
				case MTK_FB_FORMAT_RGB888:
					format = UFMT_RGB888;
					break;
				case MTK_FB_FORMAT_BGR888:
					format = UFMT_BGR888;
					break;
				case MTK_FB_FORMAT_ARGB8888:
					format = UFMT_ARGB8888;
					break;
				case MTK_FB_FORMAT_RGB565:
					format = UFMT_RGB565;
					break;
				case MTK_FB_FORMAT_UYVY:
					format = UFMT_UYVY;
					break;
				case MTK_FB_FORMAT_ABGR8888:
				default:
					format = UFMT_ABGR8888;
					break;
				}

				dst_buffer = (char *)capConfig.outputBuffer;
				fb_size = DISP_GetScreenWidth() * DISP_GetScreenHeight() * 4;
				if (!capConfig.outputBuffer) {
					MTKFB_LOG("[FB]: vmalloc capture outputBuffer  failed! line:%d\n",
						__LINE__);
					r = -EFAULT;
				} else {
					capConfig.outputBuffer = vmalloc(fb_size);
					if (!capConfig.outputBuffer) {
						MTKFB_LOG("[FB]: vmalloc capConfig outputBuffer failed! line:%d\n",
						__LINE__);
						r = -EFAULT;
					}
					primary_display_capture_framebuffer_ovl((unsigned long)
									capConfig.outputBuffer,
									format);
					if (copy_to_user(dst_buffer, (char *)capConfig.outputBuffer, fb_size)) {
						MTKFB_LOG("[FB]: copy_to_user failed!line:%d\n", __LINE__);
						r = -EFAULT;
					}
					vfree((char *)capConfig.outputBuffer);
				}
			}

			return r;
		}
	case MTKFB_GET_OVERLAY_LAYER_INFO:
		{
			struct fb_overlay_layer_info layerInfo;

			MTKFB_LOG(" mtkfb_ioctl():MTKFB_GET_OVERLAY_LAYER_INFO\n");

			if (copy_from_user(&layerInfo, (void __user *)arg, sizeof(layerInfo))) {
				MTKFB_LOG("[FB]: copy_from_user failed! line:%d\n", __LINE__);
				return -EFAULT;
			}
			if (mtkfb_get_overlay_layer_info(&layerInfo) < 0) {
				MTKFB_LOG("[FB]: Failed to get overlay layer info\n");
				return -EFAULT;
			}
			if (copy_to_user((void __user *)arg, &layerInfo, sizeof(layerInfo))) {
				MTKFB_LOG("[FB]: copy_to_user failed! line:%d\n", __LINE__);
				r = -EFAULT;
			}
			return r;
		}
	case MTKFB_SET_OVERLAY_LAYER:
		{		/* no function */
			struct fb_overlay_layer *layerInfo;
			disp_input_config *input;

			layerInfo = kmalloc(sizeof(*layerInfo), GFP_KERNEL);
			if (!layerInfo)
				return -ENOMEM;

			if (copy_from_user(layerInfo, (void __user *)arg, sizeof(*layerInfo))) {
				MTKFB_LOG("[FB]: copy_from_user failed! line:%d\n", __LINE__);
				kfree(layerInfo);
				r = -EFAULT;
			} else {
				/* in early suspend mode ,will not update buffer index, info SF by return value */
				if (primary_display_is_sleepd()) {
					DISPMSG
					    ("[FB] error, set overlay in early suspend ,skip!\n");
					kfree(layerInfo);
					return MTKFB_ERROR_IS_EARLY_SUSPEND;
				}
				memset((void *)&session_input, 0, sizeof(session_input));
				if (layerInfo->layer_id >= TOTAL_OVL_LAYER_NUM) {
					DISPAEE("MTKFB_SET_OVERLAY_LAYER ,layer_id invalid=%d\n",
						layerInfo->layer_id);
					kfree(layerInfo);
					return -EFAULT;
				} else {
					input = &session_input.config[session_input.config_layer_num++];
					_convert_fb_layer_to_disp_input(layerInfo, input);
				}
				primary_display_config_input_multiple(&session_input);
				primary_display_trigger(1, NULL, 0);
			}
			kfree(layerInfo);

			return r;
		}

	case MTKFB_ERROR_INDEX_UPDATE_TIMEOUT:
		{
			DISPMSG("[DDP] mtkfb_ioctl():MTKFB_ERROR_INDEX_UPDATE_TIMEOUT\n");
			/* call info dump function here */
			/* mtkfb_dump_layer_info(); */
			return r;
		}

	case MTKFB_ERROR_INDEX_UPDATE_TIMEOUT_AEE:
		{
			DISPMSG("[DDP] mtkfb_ioctl():MTKFB_ERROR_INDEX_UPDATE_TIMEOUT\n");
			/* call info dump function here */
			/* mtkfb_dump_layer_info(); */
			return r;
		}

	case MTKFB_SET_VIDEO_LAYERS:
		{
			struct mmp_fb_overlay_layers {
				struct fb_overlay_layer Layer0;
				struct fb_overlay_layer Layer1;
				struct fb_overlay_layer Layer2;
				struct fb_overlay_layer Layer3;
			};

			struct fb_overlay_layer *layerInfo;
			int layerInfo_size = sizeof(struct fb_overlay_layer) * VIDEO_LAYER_COUNT;

			MTKFB_LOG(" mtkfb_ioctl():MTKFB_SET_VIDEO_LAYERS\n");

			layerInfo = kmalloc(layerInfo_size, GFP_KERNEL);
			if (!layerInfo)
				return -ENOMEM;



			if (copy_from_user(layerInfo, (void __user *)arg, layerInfo_size)) {
				MTKFB_LOG("[FB]: copy_from_user failed! line:%d\n", __LINE__);
				r = -EFAULT;
			} else {
				int32_t i;
				disp_input_config *input;

				memset((void *)&session_input, 0, sizeof(session_input));

				for (i = 0; i < VIDEO_LAYER_COUNT; ++i) {
					if (layerInfo[i].layer_id >= TOTAL_OVL_LAYER_NUM) {
						DISPAEE("MTKFB_SET_VIDEO_LAYERS, layer_id invalid=%d\n",
						     layerInfo[i].layer_id);
						continue;
					}

					input =
					    &session_input.config[session_input.config_layer_num++];
					_convert_fb_layer_to_disp_input(&layerInfo[i], input);

				}
				primary_display_config_input_multiple(&session_input);
				primary_display_trigger(1, NULL, 0);
			}
			kfree(layerInfo);
			return r;
		}

	case MTKFB_TRIG_OVERLAY_OUT:
		{
			MTKFB_LOG(" mtkfb_ioctl():MTKFB_TRIG_OVERLAY_OUT\n");
			primary_display_trigger(1, NULL, 0);
			return 0;
		}

	case MTKFB_META_RESTORE_SCREEN:
		{
			struct fb_var_screeninfo var;

			if (copy_from_user(&var, argp, sizeof(var)))
				return -EFAULT;

			info->var.yoffset = var.yoffset;
			init_framebuffer(info);

			return mtkfb_pan_display_impl(&var, info);
		}


	case MTKFB_GET_DEFAULT_UPDATESPEED:
		{
			unsigned int speed = 0;

			MTKFB_LOG("[MTKFB] get default update speed\n");
			/* DISP_Get_Default_UpdateSpeed(&speed); */

			DISPMSG("[MTKFB EM]MTKFB_GET_DEFAULT_UPDATESPEED is %d\n", speed);
			return copy_to_user(argp, &speed, sizeof(speed)) ? -EFAULT : 0;
		}

	case MTKFB_GET_CURR_UPDATESPEED:
		{
			unsigned int speed = 0;

			MTKFB_LOG("[MTKFB] get current update speed\n");
			/* DISP_Get_Current_UpdateSpeed(&speed); */

			DISPMSG("[MTKFB EM]MTKFB_GET_CURR_UPDATESPEED is %d\n", speed);
			return copy_to_user(argp, &speed, sizeof(speed)) ? -EFAULT : 0;
		}

	case MTKFB_CHANGE_UPDATESPEED:
		{
			unsigned int speed;

			MTKFB_LOG("[MTKFB] change update speed\n");

			if (copy_from_user(&speed, (void __user *)arg, sizeof(speed))) {
				MTKFB_LOG("[FB]: copy_from_user failed! line:%d\n", __LINE__);
				r = -EFAULT;
			} else {
				/* DISP_Change_Update(speed); */

				DISPMSG("[MTKFB EM]MTKFB_CHANGE_UPDATESPEED is %d\n", speed);

			}
			return r;
		}

	case MTKFB_AEE_LAYER_EXIST:
		{
			int dal_en = is_DAL_Enabled();

			/* DISPMSG("[MTKFB] isAEEEnabled=%d\n", isAEEEnabled); */
			return copy_to_user(argp, &dal_en, sizeof(dal_en)) ? -EFAULT : 0;
		}
	case MTKFB_LOCK_FRONT_BUFFER:
		return 0;
	case MTKFB_UNLOCK_FRONT_BUFFER:
		return 0;

	case MTKFB_FACTORY_AUTO_TEST:
		{
			unsigned int result = 0;

			DISPMSG("factory mode: lcm auto test\n");
			result = mtkfb_fm_auto_test();
			return copy_to_user(argp, &result, sizeof(result)) ? -EFAULT : 0;
		}
	case MTKFB_META_SHOW_BOOTLOGO:
		{
			struct mtkfb_device *fbdev;
			int i;

			disp_input_config *input;

			DISPMSG("MTKFB_META_SHOW_BOOTLOGO\n");
			memset((void *)&session_input, 0, sizeof(session_input));
			fbdev = (struct mtkfb_device *)mtkfb_fbi->par;
			for (i = 0; i < 2; i++) {

				input = &session_input.config[session_input.config_layer_num++];

				input->layer_enable = 1;
				input->src_fmt = DISP_FORMAT_RGBA8888;
				input->src_offset_x = 0;
				input->src_offset_y = 0;
				input->src_width = MTK_FB_XRES;
				input->src_height = MTK_FB_YRES;
				input->tgt_offset_x = 0;
				input->tgt_offset_y = 0;
				input->tgt_width = MTK_FB_XRES;
				input->tgt_height = MTK_FB_YRES;

				input->src_pitch = ALIGN_TO(MTK_FB_XRES, MTK_FB_ALIGNMENT) * 4;
				input->alpha_enable = 1;
				input->alpha = 0xff;
				input->next_buff_idx = -1;
			}

			input = &session_input.config[0];
			input->layer_id = 0;
			input->src_phy_addr = (void *)((unsigned long)fbdev->fb_pa_base);

			input = &session_input.config[1];
			input->layer_id = 3;
			input->src_phy_addr =
			    (void *)((unsigned long)(fbdev->fb_pa_base +
			    (ALIGN_TO(MTK_FB_XRES, MTK_FB_ALIGNMENT) *
			     ALIGN_TO(MTK_FB_YRES, MTK_FB_ALIGNMENT) * 4)));

			primary_display_config_input_multiple(&session_input);
			primary_display_trigger(1, NULL, 0);

			return 0;
		}

	default:
		pr_err("mtkfb_ioctl Not support, info=0x%p, cmd=0x%08x, arg=0x%08lx\n", info,
		       (unsigned int)cmd, arg);
		return -EINVAL;
	}
}

#ifdef CONFIG_COMPAT

static void compat_convert(struct compat_fb_overlay_layer *compat_info,
			   struct fb_overlay_layer *info)
{
	info->layer_id = compat_info->layer_id;
	info->layer_enable = compat_info->layer_enable;
	info->src_base_addr = (void *)((unsigned long)compat_info->src_base_addr);
	info->src_phy_addr = (void *)((unsigned long)compat_info->src_phy_addr);
	info->src_direct_link = compat_info->src_direct_link;
	info->src_fmt = compat_info->src_fmt;
	info->src_use_color_key = compat_info->src_use_color_key;
	info->src_color_key = compat_info->src_color_key;
	info->src_pitch = compat_info->src_pitch;
	info->src_offset_x = compat_info->src_offset_x;
	info->src_offset_y = compat_info->src_offset_y;
	info->src_width = compat_info->src_width;
	info->src_height = compat_info->src_height;
	info->tgt_offset_x = compat_info->tgt_offset_x;
	info->tgt_offset_y = compat_info->tgt_offset_y;
	info->tgt_width = compat_info->tgt_width;
	info->tgt_height = compat_info->tgt_height;
	info->layer_rotation = compat_info->layer_rotation;
	info->layer_type = compat_info->layer_type;
	info->video_rotation = compat_info->video_rotation;

	info->isTdshp = compat_info->isTdshp;
	info->next_buff_idx = compat_info->next_buff_idx;
	info->identity = compat_info->identity;
	info->connected_type = compat_info->connected_type;

	info->security = compat_info->security;
	info->alpha_enable = compat_info->alpha_enable;
	info->alpha = compat_info->alpha;
	info->fence_fd = compat_info->fence_fd;
	info->ion_fd = compat_info->ion_fd;
}


static int mtkfb_compat_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	struct fb_overlay_layer layerInfo;
	long ret = 0;

	pr_debug("[FB Driver] mtkfb_compat_ioctl, cmd=0x%08x, cmd nr=0x%08x, cmd size=0x%08x\n", cmd,
	       (unsigned int)_IOC_NR(cmd), (unsigned int)_IOC_SIZE(cmd));

	switch (cmd) {
	case COMPAT_MTKFB_GET_FRAMEBUFFER_MVA:
		{
			compat_uint_t __user *data32;
			__u32 data;

			data32 = compat_ptr(arg);
			data = (__u32) fb_pa;
			if (put_user(data, data32)) {
				pr_err("MTKFB_FRAMEBUFFER_MVA failed\n");
				ret = -EFAULT;
			}
			pr_debug("MTKFB_FRAMEBUFFER_MVA success 0x%lx\n", fb_pa);
			return ret;
		}
	case COMPAT_MTKFB_GET_DISPLAY_IF_INFORMATION:
		{
			compat_uint_t __user *data32;
			compat_uint_t displayid = 0;
			compat_mtk_dispif_info_t compat_dispif_info;

			memset(&compat_dispif_info, 0, sizeof(compat_mtk_dispif_info_t));
			data32 = compat_ptr(arg);

			if (get_user(displayid, data32)) {
				pr_err("COMPAT_MTKFB_GET_DISPLAY_IF_INFORMATION failed\n");
				return -EFAULT;
			}
			if (displayid > MTKFB_MAX_DISPLAY_COUNT) {
				pr_err("[FB]: invalid display id:%d\n", displayid);
				return -EFAULT;
			}
			if (displayid == 0) {
				dispif_info[displayid].displayWidth = primary_display_get_width();
				dispif_info[displayid].displayHeight = primary_display_get_height();

				dispif_info[displayid].lcmOriginalWidth =
					primary_display_get_original_width();
				dispif_info[displayid].lcmOriginalHeight =
					primary_display_get_original_height();
				dispif_info[displayid].displayMode =
					primary_display_is_video_mode() ? 0 : 1;
			} else {
				DISPERR("information for displayid: %d is not available now\n",
				displayid);
				return -EFAULT;
			}
			compat_dispif_info.displayWidth =
				dispif_info[displayid].displayWidth;
			compat_dispif_info.displayHeight =
				dispif_info[displayid].displayHeight;
			compat_dispif_info.lcmOriginalWidth =
				dispif_info[displayid].lcmOriginalWidth;
			compat_dispif_info.lcmOriginalHeight =
				dispif_info[displayid].lcmOriginalHeight;
			compat_dispif_info.displayMode =
				dispif_info[displayid].displayMode;
			if (copy_to_user((void __user *)arg,
				&(compat_dispif_info), sizeof(compat_mtk_dispif_info_t))) {
				pr_err("[FB]: copy_to_user failed! line:%d\n", __LINE__);
				return -EFAULT;
			}
			break;
		}
	case COMPAT_MTKFB_POWEROFF:
		{
			ret = mtkfb_ioctl(info, MTKFB_POWEROFF, arg);
			break;
		}

	case COMPAT_MTKFB_POWERON:
		{
			ret = mtkfb_ioctl(info, MTKFB_POWERON, arg);
			break;
		}
	case COMPAT_MTKFB_GET_POWERSTATE:
		{
			compat_uint_t __user *data32;
			int power_state = 0;

			data32 = compat_ptr(arg);
			if (primary_display_is_sleepd())
				power_state = 0;
			else
				power_state = 1;
			if (put_user(power_state, data32)) {
				pr_err("MTKFB_GET_POWERSTATE failed\n");
				ret = -EFAULT;
			}
			pr_debug("MTKFB_GET_POWERSTATE success %d\n", power_state);
			break;
		}
	case COMPAT_MTKFB_CAPTURE_FRAMEBUFFER:
		{
			compat_ulong_t __user *data32;
			unsigned long *pbuf = NULL;
			compat_ulong_t l;

			data32 = compat_ptr(arg);
			pbuf = compat_alloc_user_space(sizeof(unsigned long));

			if (!pbuf) {
				DISPERR("[FB]: vmalloc capture src_pbuf failed! line:%d\n", __LINE__);
				ret  = -EFAULT;
			} else {
				ret = get_user(l, data32);
				ret |= put_user(l, pbuf);
				primary_display_capture_framebuffer_ovl(*pbuf, UFMT_BGRA8888);
				}
			break;
		}
	case COMPAT_MTKFB_TRIG_OVERLAY_OUT:
		{
			arg = (unsigned long)compat_ptr(arg);
			ret = mtkfb_ioctl(info, MTKFB_TRIG_OVERLAY_OUT, arg);
			break;
		}
	case 0:
		{
			arg = (unsigned long)compat_ptr(arg);
			ret = mtkfb_ioctl(info, MTKFB_META_RESTORE_SCREEN, arg);
			break;
		}

	case COMPAT_MTKFB_SET_OVERLAY_LAYER:
	{
		struct compat_fb_overlay_layer *compat_layerInfo;
		disp_input_config *input;

		compat_layerInfo = kmalloc(sizeof(*compat_layerInfo), GFP_KERNEL);

		if (!compat_layerInfo)
			return -ENOMEM;

		MTKFB_LOG(" mtkfb_compat_ioctl():MTKFB_SET_OVERLAY_LAYER\n");

		arg = (unsigned long)compat_ptr(arg);
		if (copy_from_user(compat_layerInfo, (void __user *)arg, sizeof(*compat_layerInfo))) {
				MTKFB_LOG("[FB Driver]: copy_from_user failed! line:%d\n",
					  __LINE__);
			ret = -EFAULT;
		} else {

			compat_convert(compat_layerInfo, &layerInfo);

			/* in early suspend mode ,will not update buffer index, info SF by return value */
			if (primary_display_is_sleepd()) {
				pr_debug("[FB Driver] error, set overlay in early suspend ,skip!\n");
				kfree(compat_layerInfo);
				return MTKFB_ERROR_IS_EARLY_SUSPEND;
			}
			memset((void *)&session_input, 0, sizeof(session_input));
			if (layerInfo.layer_id >= TOTAL_OVL_LAYER_NUM) {
				DISPAEE("COMPAT_MTKFB_SET_OVERLAY_LAYER, layer_id invalid:%d\n",
					  layerInfo.layer_id);
			} else {
				input = &session_input.config[session_input.config_layer_num++];
				_convert_fb_layer_to_disp_input(&layerInfo, input);
			}
			primary_display_config_input_multiple(&session_input);
			/* primary_display_trigger(1, NULL, 0); */
		}
		kfree(compat_layerInfo);
	}
		break;

	case COMPAT_MTKFB_SET_VIDEO_LAYERS:
	{
		struct compat_fb_overlay_layer *compat_layerInfo;
		int compat_layerInfo_size = sizeof(struct compat_fb_overlay_layer) * VIDEO_LAYER_COUNT;

		compat_layerInfo = kmalloc(compat_layerInfo_size, GFP_KERNEL);
		if (!compat_layerInfo)
			return -ENOMEM;

		MTKFB_LOG(" mtkfb_compat_ioctl():MTKFB_SET_VIDEO_LAYERS\n");

		if (copy_from_user(compat_layerInfo, (void __user *)arg, compat_layerInfo_size)) {
			MTKFB_LOG("[FB Driver]: copy_from_user failed! line:%d\n", __LINE__);
			ret = -EFAULT;
		} else {
			int32_t i;
			/* mutex_lock(&OverlaySettingMutex); */
			disp_input_config *input;

			memset((void *)&session_input, 0, sizeof(session_input));

			for (i = 0; i < VIDEO_LAYER_COUNT; ++i) {
				compat_convert(&compat_layerInfo[i], &layerInfo);
				if (layerInfo.layer_id >= TOTAL_OVL_LAYER_NUM) {
					DISPAEE("COMPAT_MTKFB_SET_VIDEO_LAYERS, layer_id invalid=%d\n",
						     layerInfo.layer_id);
					continue;
				}
				input =
				    &session_input.config[session_input.config_layer_num++];
				_convert_fb_layer_to_disp_input(&layerInfo, input);
			}
			/* is_ipoh_bootup = false; */
			/* atomic_set(&OverlaySettingDirtyFlag, 1); */
			/* atomic_set(&OverlaySettingApplied, 0); */
			/* mutex_unlock(&OverlaySettingMutex); */
			/* MMProfileLogStructure(MTKFB_MMP_Events.SetOverlayLayers, MMProfileFlagEnd,
						 layerInfo, struct mmp_fb_overlay_layers); */
			primary_display_config_input_multiple(&session_input);
			/* primary_display_trigger(1, NULL, 0); */
		}
		kfree(compat_layerInfo);
	}
		break;
	case COMPAT_MTKFB_AEE_LAYER_EXIST:
		{
			int dal_en = is_DAL_Enabled();
			compat_ulong_t __user *data32;

			data32 = compat_ptr(arg);
			if (put_user(dal_en, data32)) {
				pr_err("MTKFB_GET_POWERSTATE failed\n");
				ret = -EFAULT;
			}
			break;
		}
	case COMPAT_MTKFB_FACTORY_AUTO_TEST:
		{
			compat_ulong_t __user *data32;
			unsigned long result = 0;

			result = mtkfb_fm_auto_test();
			DISPMSG("factory mode: lcm auto test\n");
			data32 = compat_ptr(arg);
			if (put_user(result, data32)) {
				pr_err("MTKFB_GET_POWERSTATE failed\n");
				ret = -EFAULT;
			}
			break;
			/*return copy_to_user(argp, &result, sizeof(result)) ? -EFAULT : 0;*/
		}
	case 1:
		{
			arg = (unsigned long)compat_ptr(arg);
			ret = mtkfb_ioctl(info, MTKFB_META_SHOW_BOOTLOGO, arg);
			break;
		}
	default:
		/* NOTHING DIFFERENCE with standard ioctl calling */
		arg = (unsigned long)compat_ptr(arg);
		ret = mtkfb_ioctl(info, cmd, arg);
		break;
	}

	return ret;
}
#endif

static int mtkfb_pan_display_proxy(struct fb_var_screeninfo *var, struct fb_info *info)
{
#ifdef CONFIG_MTPROF_APPLAUNCH	/* eng enable, user disable */
	LOG_PRINT(ANDROID_LOG_INFO, "AppLaunch", "mtkfb_pan_display_proxy.\n");
#endif
	return mtkfb_pan_display_impl(var, info);
}

/* Callback table for the frame buffer framework. Some of these pointers
 * will be changed according to the current setting of fb_info->accel_flags.
 */
static struct fb_ops mtkfb_ops = {
	.owner = THIS_MODULE,
	.fb_open = mtkfb_open,
	.fb_release = mtkfb_release,
#if 0
	.fb_setcolreg = mtkfb_setcolreg,
#endif
	.fb_pan_display = mtkfb_pan_display_proxy,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	.fb_cursor = mtkfb_soft_cursor,
	.fb_check_var = mtkfb_check_var,
	.fb_set_par = mtkfb_set_par,
	.fb_ioctl = mtkfb_ioctl,
#ifdef CONFIG_COMPAT
	.fb_compat_ioctl = mtkfb_compat_ioctl,
#endif
#if 1
	.fb_blank = mtkfb_blank,
#endif
};

/*
 * ---------------------------------------------------------------------------
 * Sysfs interface
 * ---------------------------------------------------------------------------
 */

static int mtkfb_register_sysfs(struct mtkfb_device *fbdev)
{

	return 0;
}

static void mtkfb_unregister_sysfs(struct mtkfb_device *fbdev)
{
}

/*
 * ---------------------------------------------------------------------------
 * LDM callbacks
 * ---------------------------------------------------------------------------
 */
/* Initialize system fb_info object and set the default video mode.
 * The frame buffer memory already allocated by lcddma_init
 */
static int mtkfb_fbinfo_init(struct fb_info *info)
{
	struct mtkfb_device *fbdev = (struct mtkfb_device *)info->par;
	struct fb_var_screeninfo var;
	int r = 0;

	DISPFUNC();

	BUG_ON(!fbdev->fb_va_base);
	info->fbops = &mtkfb_ops;
	info->flags = FBINFO_FLAG_DEFAULT;
	info->screen_base = (char *)fbdev->fb_va_base;
	info->screen_size = fbdev->fb_size_in_byte;
	info->pseudo_palette = fbdev->pseudo_palette;

	r = fb_alloc_cmap(&info->cmap, 32, 0);
	if (r != 0)
		DISPERR("unable to allocate color map memory\n");

	/* setup the initial video mode (RGB565) */

	memset(&var, 0, sizeof(var));

	var.xres = MTK_FB_XRES;
	var.yres = MTK_FB_YRES;
	var.xres_virtual = MTK_FB_XRESV;
	var.yres_virtual = MTK_FB_YRESV;
	pr_debug("mtkfb_fbinfo_init var.xres=%d,var.yres=%d,var.xres_virtual=%d,var.yres_virtual=%d\n",
	     var.xres, var.yres, var.xres_virtual, var.yres_virtual);
	/* use 32 bit framebuffer as default */
	var.bits_per_pixel = 32;

	var.transp.offset = 24;
	var.red.length = 8;
#if 0
	var.red.offset = 16;
	var.red.length = 8;
	var.green.offset = 8;
	var.green.length = 8;
	var.blue.offset = 0;
	var.blue.length = 8;
#else
	var.red.offset = 0;
	var.red.length = 8;
	var.green.offset = 8;
	var.green.length = 8;
	var.blue.offset = 16;
	var.blue.length = 8;
#endif

	var.width = DISP_GetActiveWidth();
	var.height = DISP_GetActiveHeight();

	var.activate = FB_ACTIVATE_NOW;

	r = mtkfb_check_var(&var, info);
	if (r != 0)
		DISPERR("failed to mtkfb_check_var\n");

	info->var = var;

	r = mtkfb_set_par(info);
	if (r != 0)
		DISPERR("failed to mtkfb_set_par\n");

	MSG_FUNC_LEAVE();
	return r;
}

/* Release the fb_info object */
static void mtkfb_fbinfo_cleanup(struct mtkfb_device *fbdev)
{
	MSG_FUNC_ENTER();

	fb_dealloc_cmap(&fbdev->fb_info->cmap);

	MSG_FUNC_LEAVE();
}
/* fast memset for hw test tool */
void DISP_memset_io(volatile void __iomem *dst, int c, size_t count)
{
	u32 qc = (u8)c;

	qc |= qc << 8;
	qc |= qc << 16;

	while (count && !IS_ALIGNED((unsigned long)dst, 8)) {
		__raw_writeb(c, dst);
		dst++;
		count--;
	}
	while (count >= 4) {
		__raw_writel(qc, dst);
		dst += 4;
		count -= 4;
	}

	while (count) {
		__raw_writeb(c, dst);
		dst++;
		count--;
	}
}
/* Init frame buffer content as 3 R/G/B color bars for debug */
static int init_framebuffer(struct fb_info *info)
{
	unsigned int size;

	void *buffer = info->screen_base + info->var.yoffset * info->fix.line_length;

	/* clean whole frame buffer as black */
	/*memset_io(buffer, 0, info->screen_size);*/

	size = info->var.xres_virtual * info->var.yres * info->var.bits_per_pixel/8;
	if ((info->var.yres + info->var.yoffset <= info->var.yres_virtual) &&
			info->var.yoffset >= 0)
			DISP_memset_io(buffer, 0, size);

	return 0;
}

static void m6_mtkfb_fill_early_marker(struct mtkfb_device *fbdev, const char *tag)
{
#if M6_EARLY_FB_WHITE_MARKER
	volatile void __iomem *base;
	size_t bytes;
	u32 first = 0, mid = 0, last = 0;

	if (!fbdev || !fbdev->fb_va_base || !fbdev->fb_size_in_byte)
		return;

	base = (volatile void __iomem *)fbdev->fb_va_base;
	bytes = fbdev->fb_size_in_byte;
	DISP_memset_io(base, 0xff, bytes);
	wmb();

	first = __raw_readl(base);
	if (bytes >= 8)
		mid = __raw_readl(base + bytes / 2);
	if (bytes >= 4)
		last = __raw_readl(base + bytes - 4);

	DISPERR("M6 mtkfb fb-marker[%s]: fill=0xff bytes=%zu va=%p pa=0x%pa sample=%08x/%08x/%08x x=%u y=%u bpp=%u pages=%u line=%u\n",
		tag, bytes, fbdev->fb_va_base, &fbdev->fb_pa_base,
		first, mid, last, MTK_FB_XRES, MTK_FB_YRES, MTK_FB_BPP,
		MTK_FB_PAGES, MTK_FB_LINE);
	aee_sram_printk("M6F fill %s b=%zu pa=%pa s=%08x/%08x/%08x\n",
		tag, bytes, &fbdev->fb_pa_base, first, mid, last);
	m6_early_fb_diag.filled = true;
	m6_early_fb_diag.marker_pending = true;
	m6_early_fb_diag.bytes = bytes;
	m6_early_fb_diag.va = fbdev->fb_va_base;
	m6_early_fb_diag.pa = fbdev->fb_pa_base;
	m6_early_fb_diag.sample_first = first;
	m6_early_fb_diag.sample_mid = mid;
	m6_early_fb_diag.sample_last = last;
	m6_early_fb_diag.xres = MTK_FB_XRES;
	m6_early_fb_diag.yres = MTK_FB_YRES;
	m6_early_fb_diag.bpp = MTK_FB_BPP;
	m6_early_fb_diag.pages = MTK_FB_PAGES;
	m6_early_fb_diag.line = MTK_FB_LINE;
	m6_mtkfb_schedule_early_diag_report();
#endif
}


/* Free driver resources. Can be called to rollback an aborted initialization
 * sequence.
 */
static void mtkfb_free_resources(struct mtkfb_device *fbdev, int state)
{
	int r = 0;

	switch (state) {
	case MTKFB_ACTIVE:
		r = unregister_framebuffer(fbdev->fb_info);
		ASSERT(0 == r);
		/* lint -fallthrough */
	case 5:
		mtkfb_unregister_sysfs(fbdev);
		/* lint -fallthrough */
	case 4:
		mtkfb_fbinfo_cleanup(fbdev);
		/* lint -fallthrough */
	case 3:
		/* DISP_CHECK_RET(DISP_Deinit()); */
		/* lint -fallthrough */
	case 2:
#ifndef CONFIG_MTK_FPGA
		dma_free_coherent(0, fbdev->fb_size_in_byte, fbdev->fb_va_base, fbdev->fb_pa_base);
#endif
		/* lint -fallthrough */
	case 1:
		dev_set_drvdata(fbdev->dev, NULL);
		framebuffer_release(fbdev->fb_info);
		/* lint -fallthrough */
	case 0:
		/* nothing to free */
		break;
	default:
		BUG();
	}
}

void disp_get_fb_address(unsigned long *fbVirAddr, unsigned long *fbPhysAddr)
{
	struct mtkfb_device *fbdev = (struct mtkfb_device *)mtkfb_fbi->par;

	*fbVirAddr =
	    (unsigned long)fbdev->fb_va_base + mtkfb_fbi->var.yoffset * mtkfb_fbi->fix.line_length;
	*fbPhysAddr =
	    (unsigned long)fbdev->fb_pa_base + mtkfb_fbi->var.yoffset * mtkfb_fbi->fix.line_length;
}

#if 0
static int mtkfb_fbinfo_modify(struct fb_info *info)
{
	struct fb_var_screeninfo var;
	int r = 0;

	memcpy(&var, &(info->var), sizeof(var));
	var.activate = FB_ACTIVATE_NOW;
	var.bits_per_pixel = 32;
	var.transp.offset = 24;
	var.transp.length = 8;
	var.red.offset = 16;
	var.red.length = 8;
	var.green.offset = 8;
	var.green.length = 8;
	var.blue.offset = 0;
	var.blue.length = 8;
	var.yoffset = var.yres;

	r = mtkfb_check_var(&var, info);
	if (r != 0)
		PRNERR("failed to mtkfb_check_var\n");

	info->var = var;

	r = mtkfb_set_par(info);
	if (r != 0)
		PRNERR("failed to mtkfb_set_par\n");

	return r;
}
#endif
#if 0
static void _mtkfb_draw_point(unsigned int addr, unsigned int x, unsigned int y, unsigned int color)
{

}
#endif
static void _mtkfb_draw_block(unsigned long addr, unsigned int x, unsigned int y, unsigned int w,
			      unsigned int h, unsigned int color)
{
	int i = 0;
	int j = 0;
	unsigned long start_addr;

	start_addr = addr + MTK_FB_XRESV * 4 * y + x * 4;
	for (j = 0; j < h; j++) {
		for (i = 0; i < w; i++)
			mt_reg_sync_writel(color, (start_addr + i * 4 + j * MTK_FB_XRESV * 4));
	}
}

char *mtkfb_find_lcm_driver(void)
{
	_parse_tag_videolfb();
	DISPMSG("%s, %s\n", __func__, mtkfb_lcm_name);
	/* pr_emerg so the LK-handoff lcm name lands in the kernel ring (DISPMSG
	 * only goes to the dprec display log buffer). */
	pr_emerg("[FORGE_DISP] v195 mtkfb_find_lcm_driver: name='%s' len=%zu\n",
		 mtkfb_lcm_name, strlen(mtkfb_lcm_name));
	return mtkfb_lcm_name;
}


static int _mtkfb_internal_test(unsigned long va, unsigned int w, unsigned int h)
{
	/* this is for debug, used in bring up day */
	unsigned int i = 0;
	unsigned int color = 0;
	int _internal_test_block_size = 120;

	for (i = 0; i < w * h / _internal_test_block_size / _internal_test_block_size; i++) {
		color = (i & 0x1) * 0xff;
		/* color += ((i&0x2)>>1)*0xff00; */
		/* color += ((i&0x4)>>2)*0xff0000; */
		color += 0xff000000U;
		_mtkfb_draw_block(va,
				  i % (w / _internal_test_block_size) * _internal_test_block_size,
				  i / (w / _internal_test_block_size) * _internal_test_block_size,
				  _internal_test_block_size, _internal_test_block_size, color);
	}
	/* unsigned long ttt = get_current_time_us(); */
	/* for(i=0;i<1000;i++) */

	primary_display_trigger(1, NULL, 0);

	/* ttt = get_current_time_us()-ttt; */
	/*return 0;*/

	_internal_test_block_size = 20;
	for (i = 0; i < w * h / _internal_test_block_size / _internal_test_block_size; i++) {
		color = (i & 0x1) * 0xff;
		color += ((i & 0x2) >> 1) * 0xff00;
		color += ((i & 0x4) >> 2) * 0xff0000;
		color += 0xff000000U;
		_mtkfb_draw_block(va,
				  i % (w / _internal_test_block_size) * _internal_test_block_size,
				  i / (w / _internal_test_block_size) * _internal_test_block_size,
				  _internal_test_block_size, _internal_test_block_size, color);
	}
	primary_display_trigger(1, NULL, 0);
	_internal_test_block_size = 30;
	for (i = 0; i < w * h / _internal_test_block_size / _internal_test_block_size; i++) {
		color = (i & 0x1) * 0xff;
		color += ((i & 0x2) >> 1) * 0xff00;
		color += ((i & 0x4) >> 2) * 0xff0000;
		color += 0xff000000U;
		_mtkfb_draw_block(va,
				  i % (w / _internal_test_block_size) * _internal_test_block_size,
				  i / (w / _internal_test_block_size) * _internal_test_block_size,
				  _internal_test_block_size, _internal_test_block_size, color);
	}
	primary_display_trigger(1, NULL, 0);

	return 0;
}


#ifdef CONFIG_OF
struct tag_videolfb {
	u64 fb_base;
	u32 islcmfound;
	u32 fps;
	u32 vram;
	char lcmname[1];	/* this is the minimum size */
};
unsigned int islcmconnected = 0;
unsigned int is_lcm_inited = 0;
unsigned int vramsize = 0;
phys_addr_t fb_base = 0;
static int is_videofb_parse_done;
/*
 * LK's atag,videolfb* properties are read from the unflattened /chosen
 * node (as the mt6735 mtkfb does): of_scan_flat_dt()/of_get_flat_dt_prop()
 * are __init in 4.9, while mtkfb_get_fb_base()/mtkfb_find_lcm_driver()
 * are not.
 */
int __parse_tag_videolfb_extra(struct device_node *node)
{
	uint32_t *prop;
	int size = 0;
	u32 fb_base_h, fb_base_l;

	prop = (uint32_t *)of_get_property(node, "atag,videolfb-fb_base_h", NULL);
	if (!prop)
		return -1;
	fb_base_h = of_read_number(prop, 1);

	prop = (uint32_t *)of_get_property(node, "atag,videolfb-fb_base_l", NULL);
	if (!prop)
		return -1;
	fb_base_l = of_read_number(prop, 1);

	fb_base = ((u64) fb_base_h << 32) | (u64) fb_base_l;

	prop = (uint32_t *)of_get_property(node, "atag,videolfb-islcmfound", NULL);
	if (!prop)
		return -1;
	islcmconnected = of_read_number(prop, 1);

	prop = (uint32_t *)of_get_property(node, "atag,videolfb-islcm_inited", NULL);
	if (!prop)
		is_lcm_inited = 1;
	else
		is_lcm_inited = of_read_number(prop, 1);

	prop = (uint32_t *)of_get_property(node, "atag,videolfb-fps", NULL);
	if (!prop)
		return -1;
	lcd_fps = of_read_number(prop, 1);
	if (0 == lcd_fps)
		lcd_fps = 6000;

	prop = (uint32_t *)of_get_property(node, "atag,videolfb-vramSize", NULL);
	if (!prop)
		return -1;
	vramsize = of_read_number(prop, 1);

	prop = (uint32_t *)of_get_property(node, "atag,videolfb-fb_base_l", NULL);
	if (!prop)
		return -1;
	fb_base_l = of_read_number(prop, 1);

	prop = (uint32_t *)of_get_property(node, "atag,videolfb-lcmname", &size);
	if (!prop)
		return -1;
	if (size >= sizeof(mtkfb_lcm_name)) {
		DISPMSG("%s: error to get lcmname size=%d\n", __func__, size);
		return -1;
	}
	memset((void *)mtkfb_lcm_name, 0, sizeof(mtkfb_lcm_name));
	strncpy((char *)mtkfb_lcm_name, (char *)prop, sizeof(mtkfb_lcm_name));
	mtkfb_lcm_name[size] = '\0';
	pr_debug("__parse_tag_videolfb_extra done\n");
	return 0;
}

int __parse_tag_videolfb(struct device_node *node)
{
	struct tag_videolfb *videolfb_tag = NULL;

	videolfb_tag = (struct tag_videolfb *)of_get_property(node, "atag,videolfb", NULL);
	if (videolfb_tag) {
		memset((void *)mtkfb_lcm_name, 0, sizeof(mtkfb_lcm_name));
		strcpy((char *)mtkfb_lcm_name, videolfb_tag->lcmname);
		mtkfb_lcm_name[strlen(videolfb_tag->lcmname)] = '\0';

		lcd_fps = videolfb_tag->fps;
		if (0 == lcd_fps)
			lcd_fps = 6000;

		islcmconnected = videolfb_tag->islcmfound;
		vramsize = videolfb_tag->vram;
		fb_base = videolfb_tag->fb_base;
		is_lcm_inited = 1;
		return 0;
	}
	DISPMSG("[DT][videolfb] videolfb_tag not found\n");
	return -1;
}


static int _parse_tag_videolfb(void)
{
	int ret;
	struct device_node *node;

	DISPMSG("[DT][videolfb]isvideofb_parse_done = %d\n", is_videofb_parse_done);

	if (is_videofb_parse_done)
		return 0;

	node = of_find_node_by_path("/chosen");
	if (!node)
		node = of_find_node_by_path("/chosen@0");
	if (node) {
		ret = __parse_tag_videolfb(node);
		if (ret)
			ret = __parse_tag_videolfb_extra(node);
		of_node_put(node);
		if (!ret)
			goto found;
	} else {
		DISPMSG("[DT][videolfb] of_chosen not found\n");
	}
	return -1;

found:
	is_videofb_parse_done = 1;
	DISPMSG("[DT][videolfb] islcmfound = %d\n", islcmconnected);
	DISPMSG("[DT][videolfb] is_lcm_inited = %d\n", is_lcm_inited);
	DISPMSG("[DT][videolfb] fps        = %d\n", lcd_fps);
	DISPMSG("[DT][videolfb] fb_base    = 0x%pa\n", &fb_base);
	DISPMSG("[DT][videolfb] vram       = %d\n", vramsize);
	DISPMSG("[DT][videolfb] lcmname    = %s\n", mtkfb_lcm_name);
	return 0;
}

phys_addr_t mtkfb_get_fb_base(void)
{
	_parse_tag_videolfb();
	return fb_base;
}
EXPORT_SYMBOL(mtkfb_get_fb_base);

size_t mtkfb_get_fb_size(void)
{
	_parse_tag_videolfb();
	return vramsize;
}
EXPORT_SYMBOL(mtkfb_get_fb_size);
#endif

/* used when early porting, test pan display*/
int pan_display_test(int frame_num, int bpp)
{
	int i, j;
	int Bpp = bpp / 8;
	unsigned char *fb_va;
	unsigned long fb_pa;
	unsigned int fb_size;
	int w, h, fb_h;
	int yoffset_max;
	int yoffset;

	mtkfb_fbi->var.yoffset = 0;
	disp_get_fb_address((unsigned long *)&fb_va, &fb_pa);
	fb_size = mtkfb_fbi->fix.smem_len;
	w = mtkfb_fbi->var.xres;
	h = mtkfb_fbi->var.yres;
	fb_h = fb_size / (w * Bpp) - 10;

	DISPMSG("%s: frame_num=%d,bpp=%d, w=%d,h=%d,fb_h=%d\n",
		__func__, frame_num, bpp, w, h, fb_h);

	for (i = 0; i < fb_h; i++)
		for (j = 0; j < w; j++) {
			int x = (i * w + j) * Bpp;

			fb_va[x++] = (i + j) % 256;
			fb_va[x++] = (i + j) % 256;
			fb_va[x++] = (i + j) % 256;
			if (Bpp == 4)
				fb_va[x++] = 255;
		}

	mtkfb_fbi->var.bits_per_pixel = bpp;

	yoffset_max = fb_h - h;
	yoffset = 0;

	for (i = 0; i < frame_num; i++, yoffset += 10) {

		if (yoffset >= yoffset_max)
			yoffset = 0;

		mtkfb_fbi->var.xoffset = 0;
		mtkfb_fbi->var.yoffset = yoffset;
		mtkfb_pan_display_impl(&mtkfb_fbi->var, mtkfb_fbi);
	}

	return 0;
}

/* #define FPGA_DEBUG_PAN */
#ifdef FPGA_DEBUG_PAN
static struct task_struct *test_task;
static int update_test_kthread(void *data)
{
	/* struct sched_param param = { .sched_priority = RTPM_PRIO_SCRN_UPDATE }; */
	/* sched_setscheduler(current, SCHED_RR, &param); */
	unsigned int i = 0, j = 0;
	unsigned long fb_va;
	unsigned long fb_pa;
	unsigned int *fb_start;
	unsigned int fbsize = primary_display_get_height() * primary_display_get_width();

	mtkfb_fbi->var.yoffset = 0;
	disp_get_fb_address(&fb_va, &fb_pa);

	for (;;) {

		if (kthread_should_stop())
			break;
		msleep(1000);	/* 2s */
		pr_debug("update test thread work,offset = %d\n", i);

		mtkfb_fbi->var.yoffset = 0;
		disp_get_fb_address(&fb_va, &fb_pa);
		fb_start = (unsigned int *)fb_va;
		for (j = 0; j < fbsize; j++) {
			*fb_start = (0x55) << ((i % 4) * 8);
			fb_start++;
		}
		mtkfb_pan_display_impl(&mtkfb_fbi->var, mtkfb_fbi);
		i++;
	}

	pr_debug("exit update_test_kthread()\n");
	return 0;
}
#endif

static int mtkfb_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtkfb_device *fbdev = NULL;
	struct fb_info *fbi;
	int init_state;
	int r = 0;
	long dts_gpio_state = 0;

	printk(KERN_EMERG "[FORGE_DISP] mtkfb_probe ENTRY (raw printk)\n"); WARN_ON(1);
	pr_debug("mtkfb_probe\n");

	_parse_tag_videolfb();

	init_state = 0;
	/* repo call DTS gpio module, if not necessary, invoke nothing */
	dts_gpio_state = disp_dts_gpio_init_repo(pdev);
	if (dts_gpio_state != 0)
		dev_err(&pdev->dev, "retrieve GPIO DTS failed.");

	fbi = framebuffer_alloc(sizeof(struct mtkfb_device), dev);
	if (!fbi) {
		DISPERR("unable to allocate memory for device info\n");
		r = -ENOMEM;
		goto cleanup;
	}
	mtkfb_fbi = fbi;

	fbdev = (struct mtkfb_device *)fbi->par;
	fbdev->fb_info = fbi;
	fbdev->dev = dev;
	dev_set_drvdata(dev, fbdev);

	DISPMSG("mtkfb_probe: fb_pa = 0x%pa\n", &fb_base);

	disp_hal_allocate_framebuffer(fb_base, (fb_base + vramsize - 1),
				      (unsigned long *)&fbdev->fb_va_base, &fb_pa);
	fbdev->fb_pa_base = fb_base;

	primary_display_set_frame_buffer_address((unsigned long)fbdev->fb_va_base, fb_pa);
	/* Keep the stock takeover handoff: forcing is_lcm_inited=0 here makes
	 * disp_lcm_init(force=1) initialise the panel and DSI before the path is
	 * ready and hangs mtkfb_probe. */
	primary_display_init(mtkfb_find_lcm_driver(), lcd_fps, is_lcm_inited);

	init_state++;		/* 1 */
	MTK_FB_XRES = DISP_GetScreenWidth();
	MTK_FB_YRES = DISP_GetScreenHeight();
	fb_xres_update = MTK_FB_XRES;
	fb_yres_update = MTK_FB_YRES;

	MTK_FB_BPP = DISP_GetScreenBpp();
	MTK_FB_PAGES = DISP_GetPages();
	DISPMSG
	    ("MTK_FB_XRES=%d, MTKFB_YRES=%d, MTKFB_BPP=%d, MTK_FB_PAGES=%d, MTKFB_LINE=%d, MTKFB_SIZEV=%d\n",
	     MTK_FB_XRES, MTK_FB_YRES, MTK_FB_BPP, MTK_FB_PAGES, MTK_FB_LINE, MTK_FB_SIZEV);
	fbdev->fb_size_in_byte = MTK_FB_SIZEV;

	/* Allocate and initialize video frame buffer */
	DISPMSG("[FB Driver] fbdev->fb_pa_base = 0x%pa, fbdev->fb_va_base = 0x%p\n",
		  &(fbdev->fb_pa_base), fbdev->fb_va_base);

	if (!fbdev->fb_va_base) {
		DISPERR("unable to allocate memory for frame buffer\n");
		r = -ENOMEM;
		goto cleanup;
	}
	m6_mtkfb_fill_early_marker(fbdev, "pre-fbinfo-set-par");
	init_state++;		/* 2 */

	r = mtkfb_fbinfo_init(fbi);
	if (r) {
		DISPERR("mtkfb_fbinfo_init fail, r = %d\n", r);
		goto cleanup;
	}
	init_state++;		/* 4 */
	DISPMSG("\nmtkfb_fbinfo_init done\n");

	if (disp_helper_get_stage() == DISP_HELPER_STAGE_NORMAL) {
		/* dal_init should after mtkfb_fbinfo_init, otherwise layer 3 will show dal background color */
		DAL_STATUS ret;
		unsigned long fbVA = (unsigned long)fbdev->fb_va_base;
		unsigned long fbPA = fb_pa;
		/* / DAL init here */
		fbVA += DISP_GetFBRamSize();
		fbPA += DISP_GetFBRamSize();
		ret = DAL_Init(fbVA, fbPA);
	}

	if (disp_helper_get_stage() != DISP_HELPER_STAGE_NORMAL)
		_mtkfb_internal_test((unsigned long)fbdev->fb_va_base, MTK_FB_XRES, MTK_FB_YRES);


	r = mtkfb_register_sysfs(fbdev);
	if (r) {
		DISPERR("mtkfb_register_sysfs fail, r = %d\n", r);
		goto cleanup;
	}
	init_state++;		/* 5 */

	pr_emerg("[FORGE_DISP] mtkfb_probe: register_framebuffer r=%d\n", r);
	r = register_framebuffer(fbi);
	if (r != 0) {
		DISPERR("register_framebuffer failed\n");
		goto cleanup;
	}
#ifdef CONFIG_LCDKIT_DRIVER
            lcdkit_fb_create_sysfs(&fbi->dev->kobj);
            DISPMSG("[%s]:lcdkit fb create sysfs done",__func__);
#endif
#ifdef FPGA_DEBUG_PAN
	test_task = kthread_create(update_test_kthread, NULL, "update_test_kthread");
	wake_up_process(test_task);
#endif

	if (disp_helper_get_stage() != DISP_HELPER_STAGE_NORMAL)
		primary_display_diagnose();


	/*this function will get fb_heap base address to ion for management frame buffer */
	ion_drv_create_FB_heap(mtkfb_get_fb_base(), mtkfb_get_fb_size());

	fbdev->state = MTKFB_ACTIVE;

	MSG_FUNC_LEAVE();
	return 0;

cleanup:
	mtkfb_free_resources(fbdev, init_state);

	pr_debug("mtkfb_probe end\n");
	return r;
}

/* Called when the device is being detached from the driver */
static int mtkfb_remove(struct platform_device *pdev)
{
	struct mtkfb_device *fbdev = dev_get_drvdata(&pdev->dev);
	enum mtkfb_state saved_state = fbdev->state;

	MSG_FUNC_ENTER();
	/* FIXME: wait till completion of pending events */
#ifdef CONFIG_LCDKIT_DRIVER
        lcdkit_fb_remove_sysfs(&fbdev->fb_info->dev->kobj);
        DISPMSG("[%s]:lcdkit fb remove end\n",__func__);
#endif

	fbdev->state = MTKFB_DISABLED;
	mtkfb_free_resources(fbdev, saved_state);

	MSG_FUNC_LEAVE();
	return 0;
}

/* ro.sf.hwrotation for the ROM's init. The orientation of Android is up
 * to SurfaceFlinger alone: the OVL flip is announced to the HWC (caps
 * is_output_rotated) and compensated there; it only shows before SF runs
 * (the boot logo). So the value is the stock one of each board, whatever
 * the OVL does: l681 180 (stock build.prop ro.sf.hwrotation=180), m681 0.
 * Kernels without this file flip l681 in the OVL, and the ROM keeps 180. */
static int forge_sf_hwrotation_get(char *buffer, const struct kernel_param *kp)
{
	return scnprintf(buffer, PAGE_SIZE, "%d\n",
			 m3note_board_is(M3NOTE_BOARD_L681) ? 180 : 0);
}

static const struct kernel_param_ops forge_sf_hwrotation_ops = {
	.get = forge_sf_hwrotation_get,
};
module_param_cb(forge_sf_hwrotation, &forge_sf_hwrotation_ops, NULL, 0444);

/* PM suspend */
static int mtkfb_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	MSG_FUNC_ENTER();
	MTKFB_LOG("[FB Driver] mtkfb_suspend(): 0x%x\n", mesg.event);
/*	ovl2mem_wait_done();*/

	/* Without this device-suspend hook nothing guarantees the display is
	 * quiesced before slp_suspend_ops_enter -> spm_go_to_sleep gates the
	 * MM/DISP domain (screen-off DISP_OVL0 M4U fault -> RGU reset). This runs
	 * inside dpm_suspend, i.e. before the SPM gate. If the panel is still
	 * alive (S3 entry without a prior FBIOBLANK), do the full clean teardown
	 * here; if already asleep, the DISP domain is off: do not touch display
	 * MMIO, only leave a ram_console breadcrumb. */
	if (!primary_display_is_sleepd()) {
		int r;

		pr_emerg("[FORGE_S3] mtkfb_suspend: display ALIVE -> primary_display_suspend\n");
		r = primary_display_suspend();
		if (r)
			pr_emerg("[FORGE_S3] mtkfb_suspend: primary_display_suspend ret=%d\n", r);
	} else {
		pr_emerg("[FORGE_S3] mtkfb_suspend: display already SLEPT (no MMIO)\n");
	}

	MSG_FUNC_LEAVE();
	return 0;
}

bool mtkfb_is_suspend(void)
{
	return primary_display_is_sleepd();
}
EXPORT_SYMBOL(mtkfb_is_suspend);

int mtkfb_ipoh_restore(struct notifier_block *nb, unsigned long val, void *ign)
{
	switch (val) {
	case PM_HIBERNATION_PREPARE:
		DISPMSG("[FB Driver] mtkfb_ipoh_restore PM_HIBERNATION_PREPARE\n");
		return NOTIFY_DONE;
	case PM_RESTORE_PREPARE:
		primary_display_ipoh_restore();
		DISPMSG("[FB Driver] mtkfb_ipoh_restore PM_RESTORE_PREPARE\n");
		return NOTIFY_DONE;
	case PM_POST_HIBERNATION:
		DISPMSG("[FB Driver] mtkfb_ipoh_restore PM_POST_HIBERNATION\n");
		return NOTIFY_DONE;
	}
	return NOTIFY_OK;
}

int mtkfb_ipo_init(void)
{
	pm_nb.notifier_call = mtkfb_ipoh_restore;
	pm_nb.priority = 0;
	register_pm_notifier(&pm_nb);
	return 0;
}

static void mtkfb_shutdown(struct platform_device *pdev)
{
	MTKFB_LOG("[FB Driver] mtkfb_shutdown()\n");
	/* mt65xx_leds_brightness_set(MT65XX_LED_TYPE_LCD, LED_OFF); */
	if (!lcd_fps)
		msleep(30);
	else
		msleep(2 * 100000 / lcd_fps);	/* Delay 2 frames. */

	if (primary_display_is_sleepd()) {
		MTKFB_LOG("mtkfb has been power off\n");
		return;
	}
	primary_display_suspend();
	MTKFB_LOG("[FB Driver] leave mtkfb_shutdown\n");
}

void mtkfb_clear_lcm(void)
{
}
/*
#ifdef CONFIG_HAS_EARLYSUSPEND
static void mtkfb_early_suspend(struct early_suspend *h)
{
	int ret = 0;

	if (disp_helper_get_stage() != DISP_HELPER_STAGE_NORMAL)
		return;

	DISPMSG("[FB Driver] enter early_suspend\n");


	msleep(30);

	ret = primary_display_suspend();

	if (ret < 0) {
		DISPERR("primary display suspend failed\n");
		return;
	}

	DISPMSG("[FB Driver] leave early_suspend\n");

	return;
}
#endif
*/
/* PM resume */
static int mtkfb_resume(struct platform_device *pdev)
{
	MSG_FUNC_ENTER();
	MTKFB_LOG("[FB Driver] mtkfb_resume()\n");
	MSG_FUNC_LEAVE();
	return 0;
}
/*
#ifdef CONFIG_HAS_EARLYSUSPEND
static void mtkfb_late_resume(struct early_suspend *h)
{
	int ret = 0;

	if (disp_helper_get_stage() != DISP_HELPER_STAGE_NORMAL)
		return;

	DISPMSG("[FB Driver] enter late_resume\n");

	ret = primary_display_resume();

	if (ret) {
		DISPERR("primary display resume failed\n");
		return;
	}

	DISPMSG("[FB Driver] leave late_resume\n");

#endif
*/
/*---------------------------------------------------------------------------*/
#ifdef CONFIG_PM
/*---------------------------------------------------------------------------*/
int mtkfb_pm_suspend(struct device *device)
{
	/* pr_debug("calling %s()\n", __func__); */

	struct platform_device *pdev = to_platform_device(device);

	BUG_ON(pdev == NULL);

	return mtkfb_suspend(pdev, PMSG_SUSPEND);
}

int mtkfb_pm_resume(struct device *device)
{
	/* pr_debug("calling %s()\n", __func__); */

	struct platform_device *pdev = to_platform_device(device);

	BUG_ON(pdev == NULL);

	return mtkfb_resume(pdev);
}

int mtkfb_pm_freeze(struct device *device)
{
	primary_display_esd_check_enable(0);
	return 0;
}

int mtkfb_pm_restore_noirq(struct device *device)
{
	/* disphal_pm_restore_noirq(device); */
	DISPMSG("%s: %d\n", __func__, __LINE__);
	is_ipoh_bootup = true;
#if 0
	if (disp_helper_get_option(DISP_OPT_DYNAMIC_SWITCH_MMSYSCLK))
		ddp_clk_prepare_enable(MM_VENCPLL);
	ddp_clk_prepare_enable(DISP_MTCMOS_CLK);
	ddp_clk_prepare_enable(DISP0_SMI_COMMON);
	ddp_clk_prepare_enable(DISP0_SMI_LARB0);
#else
	dpmgr_path_power_on(primary_get_dpmgr_handle(), CMDQ_DISABLE);
#endif
	DISPMSG("%s: %d\n", __func__, __LINE__);
	return 0;

}

/*---------------------------------------------------------------------------*/
#else				/*CONFIG_PM */
/*---------------------------------------------------------------------------*/
#define mtkfb_pm_suspend NULL
#define mtkfb_pm_resume  NULL
#define mtkfb_pm_restore_noirq NULL
#define mtkfb_pm_freeze NULL
/*---------------------------------------------------------------------------*/
#endif				/*CONFIG_PM */
/*---------------------------------------------------------------------------*/
static const struct of_device_id mtkfb_of_ids[] = {
	{.compatible = "mediatek,MTKFB",},
	/* The DTS node is lowercase "mediatek,mtkfb" (mt6755.dtsi); OF matching
	 * is case-sensitive. */
	{.compatible = "mediatek,mtkfb",},
	{}
};

static const struct dev_pm_ops mtkfb_pm_ops = {
	.suspend = mtkfb_pm_suspend,
	.resume = mtkfb_pm_resume,
	.freeze = mtkfb_pm_freeze,
	.thaw = mtkfb_pm_resume,
	.poweroff = mtkfb_pm_suspend,
	.restore = mtkfb_pm_resume,
	.restore_noirq = mtkfb_pm_restore_noirq,
};

static struct platform_driver mtkfb_driver = {
	.probe = mtkfb_probe,
	.remove = mtkfb_remove,
	.suspend = mtkfb_suspend,
	.resume = mtkfb_resume,
	.shutdown = mtkfb_shutdown,
	.driver = {
		   .name = MTKFB_DRIVER,
#ifdef CONFIG_PM
		   .pm = &mtkfb_pm_ops,
#endif
		   .bus = &platform_bus_type,
		   .of_match_table = mtkfb_of_ids,
		   },
};

#if 0
#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend mtkfb_early_suspend_handler = {
	.level = EARLY_SUSPEND_LEVEL_DISABLE_FB,
	.suspend = mtkfb_early_suspend,
	.resume = mtkfb_late_resume,
};
#endif
#endif


int mtkfb_get_debug_state(char *stringbuf, int buf_len)
{
	int len = 0;
	struct mtkfb_device *fbdev = (struct mtkfb_device *)mtkfb_fbi->par;

	unsigned long va = (unsigned long)fbdev->fb_va_base;
	unsigned long mva = (unsigned long)fbdev->fb_pa_base;
	unsigned long pa = fbdev->fb_pa_base;
	unsigned int resv_size = vramsize;

	len +=
	    scnprintf(stringbuf + len, buf_len - len,
		      "|--------------------------------------------------------------------------------------|\n");
	len += scnprintf(stringbuf + len, buf_len - len, "********MTKFB Driver General Information********\n");
	len +=
	    scnprintf(stringbuf + len, buf_len - len,
		      "|Framebuffer VA:0x%lx, PA:0x%lx, MVA:0x%lx, Reserved Size:0x%08x|%d\n", va,
		      pa, mva, resv_size, resv_size);
	len +=
	    scnprintf(stringbuf + len, buf_len - len, "|xoffset=%d, yoffset=%d\n",
		      mtkfb_fbi->var.xoffset, mtkfb_fbi->var.yoffset);
	len +=
	    scnprintf(stringbuf + len, buf_len - len, "|framebuffer line alignment(for gpu)=%d\n",
		      MTK_FB_ALIGNMENT);
	len +=
	    scnprintf(stringbuf + len, buf_len - len,
		      "|xres=%d, yres=%d,bpp=%d,pages=%d,linebytes=%d,total size=%d\n", MTK_FB_XRES,
		      MTK_FB_YRES, MTK_FB_BPP, MTK_FB_PAGES, MTK_FB_LINE, MTK_FB_SIZEV);
	/* use extern in case DAL_LOCK is hold, then can't get any debug info */
	len +=
	    scnprintf(stringbuf + len, buf_len - len, "|AEE Layer is %s\n",
		      is_DAL_Enabled() ? "enabled" : "disabled");

	return len;
}
//add by yufangfang 
struct pinctrl *lcm_pinctl_pinctrl;
struct pinctrl_state *lcm_pinctl_vsp_high, *lcm_pinctl_vsp_low, *lcm_pinctl_vsn_high, *lcm_pinctl_vsn_low, *lcm_pinctl_rst_high, *lcm_pinctl_rst_low;
static int lcm_pinctl_gpio_probe(struct platform_device *pdev);
void lcm_pinctl_gpio_output(int pin, int level) ;

static void m6_lcm_gpio_dump(const char *tag)
{
	static unsigned int suppress_count;

	if (suppress_count++ < 16)
		printk(KERN_ERR
		       "[lcm_pinctl] M6 gpio[%s] raw mt_get_gpio reads suppressed to avoid hardcode dump_stack flood\n",
		       tag);
}

static int lcm_pinctl_gpio_probe(struct platform_device *pdev)
{
	int ret;
	printk ("[lcm_pinctl %d] mt_lcm_pinctl_pinctrl+++++++++++++++++\n", pdev->id);
	lcm_pinctl_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(lcm_pinctl_pinctrl)) {
		ret = PTR_ERR(lcm_pinctl_pinctrl);
		dev_err(&pdev->dev, "fwq Cannot find lcm_pinctl lcm_pinctl_pinctrl!\n");
		return ret;
	}
	lcm_pinctl_vsp_high = pinctrl_lookup_state(lcm_pinctl_pinctrl, "vsp-pullhigh");
	if (IS_ERR(lcm_pinctl_vsp_high)) {
		ret = PTR_ERR(lcm_pinctl_vsp_high);
		dev_err(&pdev->dev, "fwq Cannot find lcm_pinctl pinctrl vsp-pullhigh!\n");
		return ret;
	}
	lcm_pinctl_vsp_low = pinctrl_lookup_state(lcm_pinctl_pinctrl, "vsp-pulllow");
	if (IS_ERR(lcm_pinctl_vsp_low)) {
		ret = PTR_ERR(lcm_pinctl_vsp_low);
		dev_err(&pdev->dev, "fwq Cannot find lcm_pinctl pinctrl vsp-pulllow!\n");
		return ret;
	}
	lcm_pinctl_vsn_high = pinctrl_lookup_state(lcm_pinctl_pinctrl, "vsn-pullhigh");
	if (IS_ERR(lcm_pinctl_vsn_high)) {
		ret = PTR_ERR(lcm_pinctl_vsn_high);
		dev_err(&pdev->dev, "fwq Cannot find lcm_pinctl pinctrl vsn-pullhigh!\n");
		return ret;
	}
	lcm_pinctl_vsn_low = pinctrl_lookup_state(lcm_pinctl_pinctrl, "vsn-pulllow");
	if (IS_ERR(lcm_pinctl_vsn_low)) {
		ret = PTR_ERR(lcm_pinctl_vsn_low);
		dev_err(&pdev->dev, "fwq Cannot find lcm_pinctl pinctrl vsn-pulllow!\n");
		return ret;
	}

	lcm_pinctl_rst_high = pinctrl_lookup_state(lcm_pinctl_pinctrl, "rst-pullhigh");
	if (IS_ERR(lcm_pinctl_rst_high)) {
		ret = PTR_ERR(lcm_pinctl_rst_high);
		dev_err(&pdev->dev, "fwq Cannot find lcm_pinctl pinctrl rst-pullhigh!\n");
		return ret;
	}
	lcm_pinctl_rst_low = pinctrl_lookup_state(lcm_pinctl_pinctrl, "rst-pulllow");
	if (IS_ERR(lcm_pinctl_rst_low)) {
		ret = PTR_ERR(lcm_pinctl_rst_low);
		dev_err(&pdev->dev, "fwq Cannot find lcm_pinctl pinctrl rst-pulllow!\n");
		return ret;
	}

	
	printk ("[lcm_pinctl %d] mt_lcm_pinctl_pinctrl----------\n", pdev->id);
	m6_lcm_gpio_dump("probe");
	return 0;
}

static int lcm_pinctl_select_state_checked(const char *name,
					   struct pinctrl_state *state)
{
	int ret;

	if (IS_ERR_OR_NULL(lcm_pinctl_pinctrl) || IS_ERR_OR_NULL(state)) {
		printk(KERN_ERR
		       "[lcm_pinctl] M6 output skipped: state=%s pctrl=%p state_ptr=%p\n",
		       name, lcm_pinctl_pinctrl, state);
		m6_lcm_gpio_dump(name);
		return -ENODEV;
	}

	ret = pinctrl_select_state(lcm_pinctl_pinctrl, state);
	printk(KERN_ERR "[lcm_pinctl] M6 output state=%s ret=%d\n", name, ret);
	m6_lcm_gpio_dump(name);
	return ret;
}

void lcm_pinctl_gpio_output(int pin, int level) //pin 0->vsp 1>vsn 2->rst   0->pull_down   1->pull_up 
{
	printk ("[lcm_pinctl] lcm_pinctl_output pin = %d, level = %d\n", pin, level);
	if (pin == 0) {
		if (level)
			lcm_pinctl_select_state_checked("vsp-pullhigh",
							lcm_pinctl_vsp_high);
		else
			lcm_pinctl_select_state_checked("vsp-pulllow",
							lcm_pinctl_vsp_low);
	} else if (pin == 1){
		if (level)
			lcm_pinctl_select_state_checked("vsn-pullhigh",
							lcm_pinctl_vsn_high);
		else
			lcm_pinctl_select_state_checked("vsn-pulllow",
							lcm_pinctl_vsn_low);
	} else if (pin == 2){
		if (level)
			lcm_pinctl_select_state_checked("rst-pullhigh",
							lcm_pinctl_rst_high);
		else
			lcm_pinctl_select_state_checked("rst-pulllow",
							lcm_pinctl_rst_low);
	} else {
		printk(KERN_ERR "[lcm_pinctl] M6 output invalid pin=%d level=%d\n",
		       pin, level);
	}
}



struct of_device_id lcm_pinctl_gpio_of_match[] = {
	{ .compatible = "mediatek,lcm_pinctl", },
};

struct platform_device lcm_pinctl_gpio_device = {
	.name		= "lcm_pinctl_gpio",
	.id			= -1,
};
static struct platform_driver lcm_pinctl_gpio_driver = {
	.probe = lcm_pinctl_gpio_probe,
	.driver = {
			.name = "lcm_pinctl_gpio",
			.owner = THIS_MODULE,
			.of_match_table = lcm_pinctl_gpio_of_match,
	},
};

/* Register both the driver and the device */
int __init mtkfb_init(void)
{
	int r = 0;

	MSG_FUNC_ENTER();
	pr_emerg("[FORGE_DISP] mtkfb_init ENTRY\n");
	DISPMSG("mtkfb_init Enter\n");
	{ extern void forge_m681_mark(unsigned char); forge_m681_mark(0xC7); }	/* mtkfb/display init marker */
	{ extern int forge_display_gate_skip(const char *who); if (forge_display_gate_skip("mtkfb_init")) return 0; }	/* per-build NONRST2 self-healing display gate */
	if (platform_driver_register(&lcm_pinctl_gpio_driver) != 0) {
		printk("unable to register lcm_pinctl gpio driver.\n");
		r = -ENODEV;
		goto exit;
	}
	if (platform_driver_register(&mtkfb_driver)) {
		PRNERR("failed to register mtkfb driver\n");
		r = -ENODEV;
		platform_driver_unregister(&lcm_pinctl_gpio_driver);
		goto exit;
	}
#if 0
#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&mtkfb_early_suspend_handler);
#endif
#endif
	PanelMaster_Init();
	DBG_Init();
	m6_mtkfb_register_early_diag_proc();
	mtkfb_ipo_init();
exit:
	MSG_FUNC_LEAVE();
	pr_emerg("[FORGE_DISP] mtkfb_init LEAVE\n");
	DISPMSG("mtkfb_init LEAVE\n");
	return r;
}


static void __exit mtkfb_cleanup(void)
{
	MSG_FUNC_ENTER();

	platform_driver_unregister(&mtkfb_driver);
	platform_driver_unregister(&lcm_pinctl_gpio_driver);
#if 0
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&mtkfb_early_suspend_handler);
#endif
#endif
	/*PanelMaster_Deinit();*/
	DBG_Deinit();
	m6_mtkfb_unregister_early_diag_proc();

	MSG_FUNC_LEAVE();
}


module_init(mtkfb_init);
module_exit(mtkfb_cleanup);

MODULE_DESCRIPTION("MEDIATEK framebuffer driver");
MODULE_AUTHOR("Xuecheng Zhang <Xuecheng.Zhang@mediatek.com>");
MODULE_LICENSE("GPL");
