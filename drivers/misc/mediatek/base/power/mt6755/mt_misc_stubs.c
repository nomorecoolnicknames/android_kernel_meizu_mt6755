/*
 * MT6755 link stubs.
 *
 * No-op definitions for symbols whose subsystems are not built but whose
 * consumers still reference them. The linker resolves by name, so
 * simplified C signatures (plain int/uint instead of the original
 * enum/struct argument types) are enough and avoid pulling in each
 * subsystem's headers; the stubs ignore their arguments. Remove a stub when
 * its subsystem is built:
 *
 *   aee_nested_printf .............. AEE (mrdump)
 *   cpuhvfs_*_dvfsp_semaphore ...... CPU hybrid DVFS
 *   mt_get_charger_type ............ charger (reports UNKNOWN = 0)
 *   mt_irq_get/set_* ............... MTK SYS_CIRQ (suspend wakeup source)
 *   mtk_is_pep30_en_unlock ......... USB-C PD charging
 *   mtk_rgu_mcu_cache_preserve ..... RGU MCU cache preserve
 *   mtk_wdt_dfd_* .................. WDT DFD (DRAM flush debug)
 *   mt_pause_armpll ................ ARMPLL pause (cpufreq)
 *   mt_ppm_sysboost_* .............. PPM (performance manager)
 *   primary_display_* .............. display stack
 *   bmi160_acc_i2c_client .......... BMI160 accelerometer
 */
#include <extd_info.h>
struct dprec_logger_event;
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/export.h>
#include <linux/errno.h>	/* ION stubs below return -ENODEV */

int cpuhvfs_get_dvfsp_semaphore(int user) { return 0; }
EXPORT_SYMBOL(cpuhvfs_get_dvfsp_semaphore);
void cpuhvfs_release_dvfsp_semaphore(int user) { }
EXPORT_SYMBOL(cpuhvfs_release_dvfsp_semaphore);
#ifdef CONFIG_MTK_MT6755_CPUIDLE
/* spm_v2/mt_idle.c pauses the hybrid-DVFS processor around SODI / deep idle
 * (mt_cpufreq_hybrid.h declares these because the header sets
 * CONFIG_HYBRID_CPU_DVFS for MT6755). mt_cpufreq_hybrid.o is not built, so
 * there is no DVFS processor running: nothing to pause, 0 = paused. */
int cpuhvfs_pause_dvfsp_running(int src) { return 0; }	/* enum pause_src */
void cpuhvfs_unpause_dvfsp_to_run(int src) { }
#endif

#ifndef CONFIG_MTK_SMART_BATTERY
/* Only without the battery stack: with CONFIG_MTK_SMART_BATTERY=y
 * battery_common_fg_20.c (mt_get_charger_type) and switch_charging.c
 * (mtk_chr_is_charger_exist) provide these. */
int mt_get_charger_type(void) { return 0; }
EXPORT_SYMBOL(mt_get_charger_type);

/* mtk_chr_is_charger_exist: no charger driver; the RTC power-off path sees no charger */
int mtk_chr_is_charger_exist(unsigned char *exist) { if (exist) *exist = 0; return 0; }
EXPORT_SYMBOL(mtk_chr_is_charger_exist);
#endif /* !CONFIG_MTK_SMART_BATTERY */

/* is_set_transport_mode / set_shippingmode: battery/rt5081 shipping mode not built */
bool is_set_transport_mode(void) { return false; }
EXPORT_SYMBOL(is_set_transport_mode);
void set_shippingmode(void) { }
EXPORT_SYMBOL(set_shippingmode);

u32 mt_irq_get_pending_hw(unsigned int hwirq) { return 0; }
EXPORT_SYMBOL(mt_irq_get_pending_hw);
u32 mt_irq_get_pending_vec(u32 start_irq) { return 0; }
EXPORT_SYMBOL(mt_irq_get_pending_vec);
u32 mt_irq_get_pol_hw(u32 hwirq) { return 0; }
EXPORT_SYMBOL(mt_irq_get_pol_hw);
void mt_irq_set_pending_hw(unsigned int hwirq) { }
EXPORT_SYMBOL(mt_irq_set_pending_hw);

bool mtk_is_pep30_en_unlock(void) { return false; }
EXPORT_SYMBOL(mtk_is_pep30_en_unlock);

int mtk_rgu_mcu_cache_preserve(int enable) { return 0; }
EXPORT_SYMBOL(mtk_rgu_mcu_cache_preserve);

int mtk_wdt_dfd_count_en(int value) { return 0; }
EXPORT_SYMBOL(mtk_wdt_dfd_count_en);
int mtk_wdt_dfd_timeout(int value) { return 0; }
EXPORT_SYMBOL(mtk_wdt_dfd_timeout);
int mtk_wdt_dfd_thermal1_dis(int value) { return 0; }
EXPORT_SYMBOL(mtk_wdt_dfd_thermal1_dis);
int mtk_wdt_dfd_thermal2_dis(int value) { return 0; }
EXPORT_SYMBOL(mtk_wdt_dfd_thermal2_dis);

int mt_pause_armpll(unsigned int pll, unsigned int pause) { return 0; }
EXPORT_SYMBOL(mt_pause_armpll);

void mt_ppm_sysboost_core(int user, unsigned int core_num) { }
EXPORT_SYMBOL(mt_ppm_sysboost_core);
void mt_ppm_sysboost_freq(int user, unsigned int freq) { }
EXPORT_SYMBOL(mt_ppm_sysboost_freq);
/* mt_cpufreq.c reports its tables and limit callback to PPM and kicks PBM
 * from its hotcpu path; ppm_v2 is MT6757 only. Without PPM, mt_cpufreq's
 * idx_opp_ppm_limit stays -1, which means no limit. */
void mt_ppm_set_dvfs_table(unsigned int cpu, void *tbl,
			   unsigned int num, int type) { }
EXPORT_SYMBOL(mt_ppm_set_dvfs_table);
void mt_ppm_register_client(int client, void (*limit)(void)) { }
EXPORT_SYMBOL(mt_ppm_register_client);
void mt_ppm_dlpt_kick_PBM(void *cluster_status, unsigned int cluster_num) { }
EXPORT_SYMBOL(mt_ppm_dlpt_kick_PBM);

/* primary_display_* stubs: referenced by non-display code (ppm/perf) when
 * CONFIG_MTK_FB is off. */

#ifndef CONFIG_MTK_FB
/* Without MTK_FB there is no primary_display.c, but the SMI VR/slow-motion
 * fps coupling in smi_common.c references these. Guarded, so they drop out
 * when MTK_FB (the real primary_display.c) is enabled. */
unsigned int primary_display_get_fps(void) { return 0; }
EXPORT_SYMBOL(primary_display_get_fps);
int primary_display_force_set_vsync_fps(unsigned int fps) { return 0; }
EXPORT_SYMBOL(primary_display_force_set_vsync_fps);
#endif

#ifndef CONFIG_MTK_AAL_SUPPORT
/* The LEDS backlight driver (leds/mt6755/mtk_leds.c) calls the display-PQ
 * backlight-sync hook, which is defined only by the AAL engine (CONFIG_MTK_AAL_SUPPORT,
 * off). Stub it so MTK_LEDS links; backlight still works, it just doesn't drive AAL
 * gamma/CABC adaptation. Auto-disabled when AAL is re-enabled. */
void disp_pq_notify_backlight_changed(int bl_1024) { }
EXPORT_SYMBOL(disp_pq_notify_backlight_changed);
#endif

/* rtc_read_pwron_alarm now from real CONFIG_MTK_RTC (mtk_rtc_common.c) */

void *bmi160_acc_i2c_client;
EXPORT_SYMBOL(bmi160_acc_i2c_client);

/* ION stubs: CONFIG_ION is not set, but mtkfb_fence.c uses ION
 * unconditionally. */

/* m4u/mt6755 config_mau — inside #if 0 in m4u_hw.c; referenced by m4u/2.0/m4u.c
 * via the MTK_M4U_T_CONFIG_MAU ioctl (never triggered at boot). */
int config_mau(void *mau) { return 0; }
EXPORT_SYMBOL(config_mau);

/* SPM SODI display low-power hooks. With MTK_MT6755_CPUIDLE the real ones
 * (spm_v2/mt_spm_sodi.c, mt_spm_sodi3p0.c) are built; they only record the
 * flags. */
#ifndef CONFIG_MTK_MT6755_CPUIDLE
void spm_enable_sodi3(int vdo_mode) { }
EXPORT_SYMBOL(spm_enable_sodi3);
void spm_enable_sodi(int vdo_mode) { }
EXPORT_SYMBOL(spm_enable_sodi);
void spm_sodi_mempll_pwr_mode(int mode) { }
EXPORT_SYMBOL(spm_sodi_mempll_pwr_mode);
void spm_sodi_set_vdo_mode(int mode) { }
EXPORT_SYMBOL(spm_sodi_set_vdo_mode);
#endif

/* External display (HDMI/MHL): not present on this board */
int external_display_frame_cfg(void *p) { return 0; }
EXPORT_SYMBOL(external_display_frame_cfg);
void external_display_control_init(void) { }
EXPORT_SYMBOL(external_display_control_init);
int ext_disp_get_max_layer(void) { return 0; }
EXPORT_SYMBOL(ext_disp_get_max_layer);
int external_display_switch_mode(int mode, void *session, int need_lock) { return 0; }
EXPORT_SYMBOL(external_display_switch_mode);
int external_display_wait_for_vsync(void *config, void *session) { return 0; }
EXPORT_SYMBOL(external_display_wait_for_vsync);

int mt_usb2jtag_resume(void) { return 0; }
EXPORT_SYMBOL(mt_usb2jtag_resume);

const struct EXTD_DRIVER *EXTD_HDMI_Driver(void) { return NULL; }
EXPORT_SYMBOL(EXTD_HDMI_Driver);
const struct EXTD_DRIVER *EXTD_EPD_Driver(void) { return NULL; }
EXPORT_SYMBOL(EXTD_EPD_Driver);
int external_display_config_input(void *cfg, unsigned int idx, unsigned int sid) { return 0; }



void dprec_logger_event_init(struct dprec_logger_event *p, char *name, unsigned int level, unsigned int enable, unsigned int type) {}
EXPORT_SYMBOL(dprec_logger_event_init);
void dprec_logger_dump_reset(void) {}
EXPORT_SYMBOL(dprec_logger_dump_reset);
char *dprec_logger_get_dump_addr(void) { return NULL; }
EXPORT_SYMBOL(dprec_logger_get_dump_addr);
unsigned int dprec_logger_get_dump_len(void) { return 0; }
EXPORT_SYMBOL(dprec_logger_get_dump_len);
int dprec_logger_get_result_string_all(char *buf, int len) { return 0; }
EXPORT_SYMBOL(dprec_logger_get_result_string_all);
int dprec_logger_get_buf(unsigned int type, char *buf, int len) { return 0; }
EXPORT_SYMBOL(dprec_logger_get_buf);
char debug_buffer[16 * 1024];
EXPORT_SYMBOL(debug_buffer);
unsigned int gCaptureWdmaLayerEnable;
EXPORT_SYMBOL(gCaptureWdmaLayerEnable);
unsigned int gCaptureRdmaLayerEnable;
EXPORT_SYMBOL(gCaptureRdmaLayerEnable);
void dprec_mmp_dump(void) {}
EXPORT_SYMBOL(dprec_mmp_dump);
int dprec_logger_pr(unsigned int type, char *fmt, ...) { return 0; }
EXPORT_SYMBOL(dprec_logger_pr);
void dprec_reg_op(void *cmdq, unsigned int reg, unsigned int val, unsigned int mask) {}
EXPORT_SYMBOL(dprec_reg_op);
void init_log_buffer(void) {}
EXPORT_SYMBOL(init_log_buffer);
void PanelMaster_Init(void) {}
EXPORT_SYMBOL(PanelMaster_Init);
void dprec_logger_dump(char *string) {}
EXPORT_SYMBOL(dprec_logger_dump);
void dprec_logger_init(unsigned int buf_size, unsigned int log_size) {}
EXPORT_SYMBOL(dprec_logger_init);
void dprec_logger_reset(unsigned int source) {}
EXPORT_SYMBOL(dprec_logger_reset);

int is_buffer_init;
EXPORT_SYMBOL(is_buffer_init);
void dprec_start(struct dprec_logger_event *event, unsigned int val1, unsigned int val2) {}
EXPORT_SYMBOL(dprec_start);
void dprec_done(struct dprec_logger_event *event, unsigned int val1, unsigned int val2) {}
EXPORT_SYMBOL(dprec_done);
void dprec_trigger(struct dprec_logger_event *event, unsigned int val1, unsigned int val2) {}
EXPORT_SYMBOL(dprec_trigger);
#ifndef CONFIG_MTK_SYNC	/* real definitions live in mediatek/sync/mtk_sync.c when MTK_SYNC=y */
void *fence_create(int fd) { return NULL; }
EXPORT_SYMBOL(fence_create);
#endif

void dprec_logger_start(struct dprec_logger_event *event) {}
EXPORT_SYMBOL(dprec_logger_start);
void dprec_logger_done(struct dprec_logger_event *event) {}
EXPORT_SYMBOL(dprec_logger_done);
#ifndef CONFIG_MTK_SYNC	/* real definitions live in mediatek/sync/mtk_sync.c when MTK_SYNC=y */
void *timeline_create(const char *name) { return NULL; }
EXPORT_SYMBOL(timeline_create);
void timeline_inc(void *timeline, unsigned int val) {}
EXPORT_SYMBOL(timeline_inc);
#endif
void dprec_logger_reset_all(void) {}
EXPORT_SYMBOL(dprec_logger_reset_all);
void dprec_handle_option(const char *opt) {}
EXPORT_SYMBOL(dprec_handle_option);
void dprec_mmp_init(void) {}
EXPORT_SYMBOL(dprec_mmp_init);
void disp_init_ui_logging(void) {}
EXPORT_SYMBOL(disp_init_ui_logging);
unsigned int mmp_log_call_ratio;
EXPORT_SYMBOL(mmp_log_call_ratio);

unsigned int gCapturePriLayerDownX, gCapturePriLayerDownY, gCapturePriLayerNum, gCapturePriLayerEnable;
EXPORT_SYMBOL(gCapturePriLayerDownX);
EXPORT_SYMBOL(gCapturePriLayerDownY);
EXPORT_SYMBOL(gCapturePriLayerNum);
EXPORT_SYMBOL(gCapturePriLayerEnable);

void dprec_submit(struct dprec_logger_event *event, unsigned int val1, unsigned int val2) {}
EXPORT_SYMBOL(dprec_submit);
int external_display_get_info(void *info) { return 0; }
EXPORT_SYMBOL(external_display_get_info);
void dprec_mmp_dump_wdma_layer(void *cfg) {}
EXPORT_SYMBOL(dprec_mmp_dump_wdma_layer);
void dprec_mmp_dump_ovl_layer(void *cfg) {}
EXPORT_SYMBOL(dprec_mmp_dump_ovl_layer);

int fbconfig_get_esd_check(void) { return 0; }
EXPORT_SYMBOL(fbconfig_get_esd_check);
void dprec_logger_trigger(struct dprec_logger_event *event) {}
EXPORT_SYMBOL(dprec_logger_trigger);
void Panel_Master_DDIC_config(void) {}
EXPORT_SYMBOL(Panel_Master_DDIC_config);
void m6_led_dump_backlight_truth(void) {}
EXPORT_SYMBOL(m6_led_dump_backlight_truth);
void app_info_set(const char *name, unsigned int val) {}
EXPORT_SYMBOL(app_info_set);
void external_display_trigger(void *cfg) {}
EXPORT_SYMBOL(external_display_trigger);

void dprec_mmp_dump_rdma_layer(void *cfg) {}
EXPORT_SYMBOL(dprec_mmp_dump_rdma_layer);
void exit_pd_by_cmdq(void *handle) {}
EXPORT_SYMBOL(exit_pd_by_cmdq);
void enter_pd_by_cmdq(void *handle) {}
EXPORT_SYMBOL(enter_pd_by_cmdq);
void dprec_init(void) {}
EXPORT_SYMBOL(dprec_init);
int dprec_logger_get_result_value(unsigned int source, void *fps) { return 0; }
EXPORT_SYMBOL(dprec_logger_get_result_value);

void dprec_logger_frame_seq_end(struct dprec_logger_event *event) {}
EXPORT_SYMBOL(dprec_logger_frame_seq_end);
int dprec_option_enabled(unsigned int option) { return 0; }
EXPORT_SYMBOL(dprec_option_enabled);
unsigned int dprec_get_vsync_count(void) { return 0; }
EXPORT_SYMBOL(dprec_get_vsync_count);
void dprec_logger_frame_seq_begin(struct dprec_logger_event *event) {}
EXPORT_SYMBOL(dprec_logger_frame_seq_begin);
void dprec_stub_irq(unsigned int irq) {}
EXPORT_SYMBOL(dprec_stub_irq);
void dprec_stub_event(unsigned int event) {}
EXPORT_SYMBOL(dprec_stub_event);

/* PBM (power budget manager) stub for the eccci modem: the eccci core calls
 * init_md_section_level(KR_MD1) once to register the modem with PBM, which
 * is not built for MT6755 (pbm_v1/ is MT6757 only, mt_pbm.o is commented
 * out). The modem then does not take part in power budgeting. The real
 * prototype takes enum pbm_kicker. Remove when PBM is built. */
void init_md_section_level(int kicker) {}
EXPORT_SYMBOL(init_md_section_level);
/* Same here: the mt6755 CLDMA platform calls this on modem power on/off
 * (eccci/mt6755/cldma_platform.c:740,835) to raise/drop the modem's power budget. */
void kicker_pbm_by_md(int kicker, bool status) {}
EXPORT_SYMBOL(kicker_pbm_by_md);

/* Camera sensor stubs, only for configs without the camera stack: with
 * CONFIG_MTK_IMGSENSOR the real MCLK provider (imgsensor/src/mt6755/
 * mt_cam_mclk.c) owns these symbols, and no-ops would leave every sensor
 * without its master clock. */
#if !IS_ENABLED(CONFIG_MTK_IMGSENSOR)
void ISP_MCLK1_EN(int en) {}
void ISP_MCLK2_EN(int en) {}
#endif
unsigned int abist_meter(unsigned int meter) { return 0; }
unsigned int mmdvfs_get_stable_isp_clk(void) { return 0; }
