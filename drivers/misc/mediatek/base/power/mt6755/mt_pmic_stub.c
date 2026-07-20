/*
 * MT6755 residual PMIC stubs.
 *
 * The MT6353 PMIC driver (drivers/misc/mediatek/pmic/mt6353/, built with
 * CONFIG_MTK_PMIC_NEW_ARCH + CONFIG_MTK_PMIC_CHIP_MT6353) provides the bulk
 * of the PMIC API: pmic_read_interface / pmic_config_interface(+_nolock)
 * (pmic.c), pmic_set/get_register_value(+_nolock) (upmu_common.c),
 * upmu_set/get_reg_value, pmic_lock/unlock (pmic.c).
 *
 * What remains stubbed here, and why:
 *  - pmic_config_interface_nospinlock / pmic_set_register_value_nospinlock:
 *    not provided by the mt6353 driver; only referenced by callers that
 *    are not built.
 *  - pmic_force_vcore_pwm: vcore DVFS is not built.
 *  - upmu_is_chr_det: not exported by the mt6353 driver build; charger
 *    presence comes from pmic_get_register_value(PMIC_RGS_CHRDET) in the
 *    chr_type_det path.
 */
#include <linux/kernel.h>
#include <linux/export.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <mach/upmu_hw.h>
#include <mt-plat/upmu_common.h>

unsigned int pmic_config_interface_nospinlock(unsigned int RegNum, unsigned int val,
					      unsigned int MASK, unsigned int SHIFT)
{
	return 0;
}
EXPORT_SYMBOL(pmic_config_interface_nospinlock);

unsigned short pmic_set_register_value_nospinlock(PMU_FLAGS_LIST_ENUM flagname, unsigned int val)
{
	return 0;
}
EXPORT_SYMBOL(pmic_set_register_value_nospinlock);

int pmic_force_vcore_pwm(bool enable)
{
	return 0;
}
EXPORT_SYMBOL(pmic_force_vcore_pwm);

#ifndef CONFIG_MTK_SMART_BATTERY
/* Charger presence: "no charger" without the battery stack; with
 * CONFIG_MTK_SMART_BATTERY=y battery_common_fg_20.c provides it. */
bool upmu_is_chr_det(void)
{
	return false;
}
EXPORT_SYMBOL(upmu_is_chr_det);
#endif /* !CONFIG_MTK_SMART_BATTERY */
/*
 * pwrap_base, mt_pmic_wrap_eint_status and mt_pmic_wrap_eint_clr come from
 * the pwrap chip HAL, pmic_wrap/mt6755/pwrap_hal_v1.c
 * (CONFIG_MTK_PMIC_WRAP_HAL).
 */

/*
 * MT6311 external buck presence queries for the mt6353 driver's ext-buck
 * helpers. mt6311.c is not built (it needs the 3.18 MTK-extended
 * i2c_client); "absent / not ready" makes the helpers use the internal
 * bucks. The CPU DVFS code reaches the MT6311 directly on i2c-3
 * (forge_vrail.c).
 */
int is_mt6311_exist(void)
{
	return 0;
}
EXPORT_SYMBOL(is_mt6311_exist);

int is_mt6311_sw_ready(void)
{
	return 0;
}
EXPORT_SYMBOL(is_mt6311_sw_ready);

int get_mt6311_i2c_ch_num(void)
{
	return -1;
}
EXPORT_SYMBOL(get_mt6311_i2c_ch_num);
