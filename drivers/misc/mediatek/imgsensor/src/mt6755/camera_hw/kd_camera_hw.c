#include <linux/m3note_board.h>
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
/* Port of the stock wt6755_66_sz_l kd_camera_hw.c (Meizu 3.10). Changes:
 * xlog.h/mt_typedefs.h dropped (kd_camera_typedef.h provides BOOL/kal
 * types); AF_EN uses pinctrl states; the forge_cam_power knob (default 0)
 * gates all power-on. */
#include <linux/module.h>
#include "kd_camera_typedef.h"

#include "kd_camera_hw.h"


#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_feature.h"




/******************************************************************************
 * Debug configuration
******************************************************************************/
#define PFX "[kd_camera_hw]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(PFX fmt, ##arg)

#define DEBUG_CAMERA_HW_K
#ifdef DEBUG_CAMERA_HW_K
#define PK_DBG PK_DBG_FUNC
#define PK_ERR(fmt, arg...)         pr_err(fmt, ##arg)
#define PK_XLOG_INFO(fmt, args...) \
		do {    \
		   pr_debug(PFX fmt, ##arg); \
		} while (0)
#else
#define PK_DBG(a, ...)
#define PK_ERR(a, ...)
#define PK_XLOG_INFO(fmt, args...)
#endif


#define IDX_PS_MODE 1
#define IDX_PS_ON   2
#define IDX_PS_OFF  3


#define IDX_PS_CMRST 0
#define IDX_PS_CMPDN 4


extern void ISP_MCLK1_EN(BOOL En);
extern void ISP_MCLK2_EN(BOOL En);
extern void ISP_MCLK3_EN(BOOL En);

/* Camera power gate, default 0: no rail, pin or MMIO activity and no
 * /dev/kd_camera_hw* nodes (the camera HAL tries to power the sensors on at
 * every boot). Enable with
 *   echo 1 > /sys/module/kd_camera_hw/parameters/forge_cam_power
 * then restart the camera HAL (or reopen the camera app). */
/* kd_sensorlist.c: arm = create /dev/kd_camera_hw* + map MCLK + get
 * regulators. The 0 -> non-0 transition of forge_cam_power is the only
 * trigger. */
extern int forge_cam_arm(void);

static int forge_cam_power; /* = 0 */
static int forge_cam_power_set(const char *val, const struct kernel_param *kp)
{
	int prev = forge_cam_power;
	int req, ret;

	ret = kstrtoint(val, 0, &req);
	if (ret)
		return ret;
	forge_cam_power = req;
	/* Arming (0 -> non-0) creates /dev/kd_camera_hw*, maps the ISP MCLK
	 * provider and resolves the VCAM regulators. A default boot does none
	 * of that, so the always-running camera HAL finds no node. Power
	 * sequencing is gated by the same flag. */
	if (req && !prev) {
		pr_err("[FORGE_CAM] forge_cam_power %d->%d: ARMING camera (create nodes + map MCLK + resolve regulators)\n",
		       prev, req);
		forge_cam_arm();
	}
	return 0;
}
static const struct kernel_param_ops forge_cam_power_ops = {
	.set = forge_cam_power_set,
	.get = param_get_int,
};
module_param_cb(forge_cam_power, &forge_cam_power_ops, &forge_cam_power, 0644);
MODULE_PARM_DESC(forge_cam_power, "m681: 0 (default) = INERT (no camera nodes/clock/regulators at boot); set 1 (watched) to arm = create /dev/kd_camera_hw* + map MCLK, then power stays gated by this same flag");

/* Lets the in-kernel detect trigger (forge_cam_detect in kd_sensorlist.c)
 * refuse to run while the gate is closed. */
int forge_cam_power_enabled(void)
{
	return forge_cam_power;
}


u32 pinSetIdx = 0;		/* default main sensor */
u32 pinSet[3][8] = {
	/* for main sensor */
	{CAMERA_CMRST_PIN,
	 CAMERA_CMRST_PIN_M_GPIO,	/* mode */
	 GPIO_OUT_ONE,		/* ON state */
	 GPIO_OUT_ZERO,		/* OFF state */
	 CAMERA_CMPDN_PIN,
	 CAMERA_CMPDN_PIN_M_GPIO,
	 GPIO_OUT_ONE,
	 GPIO_OUT_ZERO,
	 },
	/* for sub sensor */
	{CAMERA_CMRST1_PIN,
	 CAMERA_CMRST1_PIN_M_GPIO,
	 GPIO_OUT_ONE,
	 GPIO_OUT_ZERO,
	 CAMERA_CMPDN1_PIN,
	 CAMERA_CMPDN1_PIN_M_GPIO,
	 GPIO_OUT_ONE,
	 GPIO_OUT_ZERO,
	 },
};
#ifndef CONFIG_MTK_LEGACY
#define CUST_AVDD AVDD - AVDD
#define CUST_DVDD DVDD - AVDD
#define CUST_DOVDD DOVDD - AVDD
#define CUST_AFVDD AFVDD - AVDD
#define CUST_SUB_DVDD SUB_DVDD - AVDD
#define CUST_MAIN2_DVDD MAIN2_DVDD - AVDD
#endif


PowerCust PowerCustList = {
	{
	 {GPIO_UNSUPPORTED, GPIO_MODE_GPIO, Vol_Low},	/* for AVDD; */
	 {GPIO_UNSUPPORTED, GPIO_MODE_GPIO, Vol_Low},	/* for DVDD; */
	 {GPIO_UNSUPPORTED, GPIO_MODE_GPIO, Vol_Low},	/* for DOVDD; */
	 {GPIO_UNSUPPORTED, GPIO_MODE_GPIO, Vol_Low},	/* for AFVDD; */
	 {GPIO_UNSUPPORTED, GPIO_MODE_GPIO, Vol_Low},	/* for SUB_DVDD; */
/* {GPIO_SUPPORTED, GPIO_MODE_GPIO, Vol_Low}, */
	 }
};



/*
 * The L681 rows below go first and two of their names equal M681 ones
 * ("ov5670mipiraw"; hi553 the generic one), so on any other board the
 * first-match lookup starts after them.
 */
#if defined(OV13853_MIPI_RAW_OFG_AL1518)
#define FORGE_L681_ROW_OFG 1
#else
#define FORGE_L681_ROW_OFG 0
#endif
#if defined(OV13853_MIPI_RAW_SY_AL1518)
#define FORGE_L681_ROW_SY 1
#else
#define FORGE_L681_ROW_SY 0
#endif
#if defined(OV5670_MIPI_RAW_QH_AL1518)
#define FORGE_L681_ROW_QH 1
#else
#define FORGE_L681_ROW_QH 0
#endif
#if defined(HI553_MIPI_RAW_HQ_AL1518)
#define FORGE_L681_ROW_HQ 1
#else
#define FORGE_L681_ROW_HQ 0
#endif
#define FORGE_L681_PWR_ROWS \
	(FORGE_L681_ROW_OFG + FORGE_L681_ROW_SY + FORGE_L681_ROW_QH + FORGE_L681_ROW_HQ)

static int forge_cam_pwr_first(void)
{
	return m3note_board_known_is(M3NOTE_BOARD_L681) ? 0 : FORGE_L681_PWR_ROWS;
}

PowerUp PowerOnList = {
	{
	/* L681 power sequences, values from the stock hq6755_66_b1a_l
	 * kd_camera_hw.c (L681 stock drives CMRST). Listed first and guarded
	 * by the L681 sensor defines: lookup is a first-match strcmp on the
	 * drvname and the L681 ov5670/hi553 strings equal the generic ones.
	 * AFVDD rows are inert on 4.4 (the hwpoweron AFVDD branch is #if 0). */
#if defined(OV13853_MIPI_RAW_OFG_AL1518)
	 {SENSOR_DRVNAME_OV13853_MIPI_RAW_OFG_AL1518,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {AVDD, Vol_2800, 0},
	   {DOVDD, Vol_1800, 0},
	   {DVDD, Vol_1200, 1},
	   {AFVDD, Vol_2800, 5},
	   {PDN, Vol_Low, 5},
	   {PDN, Vol_High, 0},
	   {RST, Vol_Low, 1},
	   {RST, Vol_High, 0},
	   },
	  },
#endif
#if defined(OV13853_MIPI_RAW_SY_AL1518)
	 {SENSOR_DRVNAME_OV13853_MIPI_RAW_SY_AL1518,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {AVDD, Vol_2800, 0},
	   {DOVDD, Vol_1800, 0},
	   {DVDD, Vol_1200, 1},
	   {AFVDD, Vol_2800, 5},
	   {PDN, Vol_Low, 5},
	   {PDN, Vol_High, 0},
	   {RST, Vol_Low, 1},
	   {RST, Vol_High, 0},
	   },
	  },
#endif
#if defined(OV5670_MIPI_RAW_QH_AL1518)
	 {SENSOR_DRVNAME_OV5670_MIPI_RAW_QH_AL1518,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {PDN, Vol_Low, 1},
	   {RST, Vol_Low, 1},
	   {AVDD, Vol_2800, 1},
	   {DOVDD, Vol_1800, 1},
	   {DVDD, Vol_1200, 1},
	   {PDN, Vol_Low, 4},
	   {PDN, Vol_High, 0},
	   {RST, Vol_Low, 1},
	   {RST, Vol_High, 0}
	   },
	  },
#endif
#if defined(HI553_MIPI_RAW_HQ_AL1518)
	 {SENSOR_DRVNAME_HI553_MIPI_RAW_HQ_AL1518,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {DOVDD, Vol_1800, 1},
	   {AVDD, Vol_2800, 1},
	   {DVDD, Vol_1200, 1},
	   {PDN, Vol_Low, 0},
	   {PDN, Vol_High, 0},
	   {SensorMCLK, Vol_High, 0},
	   {AFVDD, Vol_2800, 5},
	   {RST, Vol_Low, 10},
	   {RST, Vol_High, 0}
	   },
	  },
#endif
	 {SENSOR_DRVNAME_OV13853_MIPI_RAW,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {PDN, Vol_Low, 0},
	   /* No RST rows: the stock kernel never drives CMRST for any
	    * sensor on this board. */
	   {AVDD, Vol_2800, 0},
	   {DOVDD, Vol_1800, 0},
	   {DVDD, Vol_1200, 10},
	   {PDN, Vol_Low, 0},
	   {PDN, Vol_High, 1},
//	   {AFVDD, Vol_2800, 0},	//wangjie, tp open
	   {AF_EN, Vol_Low, 0},
	   {AF_EN, Vol_High, 7},
	   },
	  },//wangjie, back cam
	 {SENSOR_DRVNAME_S5K5E8YX_MIPI_RAW,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {PDN, Vol_Low, 0},
	   /* No RST rows, as above. */
	   {AVDD, Vol_2800, 5},
	   {DVDD, Vol_1200, 5},
	   {DOVDD, Vol_1800, 15},
	   {PDN, Vol_Low, 0},
	   {PDN, Vol_High, 1},
	   },
	  },//wangjie, front cam
	 {SENSOR_DRVNAME_OV13853QTECH_MIPI_RAW,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {PDN, Vol_Low, 0},
	   {RST, Vol_Low, 0},	/* GPIO110 reset, see wt6755_66_sz_l.dts cam0_rst0 */
	   {AVDD, Vol_2800, 0},
	   {DOVDD, Vol_1800, 0},
	   {DVDD, Vol_1200, 10},
	   {PDN, Vol_Low, 0},
	   {PDN, Vol_High, 1},
	   {RST, Vol_High, 1},	/* release reset after PDN */
//	   {AFVDD, Vol_2800, 0},	//wangjie, tp open
	   {AF_EN, Vol_Low, 0},
	   {AF_EN, Vol_High, 7},
	   },
	  },//wangjie, qtech back cam
	 {SENSOR_DRVNAME_OV13853OFILM_MIPI_RAW,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {PDN, Vol_Low, 0},
	   {RST, Vol_Low, 0},	/* GPIO110 reset, see wt6755_66_sz_l.dts cam0_rst0 */
	   {AVDD, Vol_2800, 0},
	   {DOVDD, Vol_1800, 0},
	   {DVDD, Vol_1200, 10},
	   {PDN, Vol_Low, 0},
	   {PDN, Vol_High, 1},
	   {RST, Vol_High, 1},	/* release reset after PDN */
//	   {AFVDD, Vol_2800, 0},	//jerryxie, tp open
	   {AF_EN, Vol_Low, 0},
	   {AF_EN, Vol_High, 7},
	   },
	  },//jerryxie, ofilm back cam
	  {SENSOR_DRVNAME_OV13853SUNNY2_MIPI_RAW,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {PDN, Vol_Low, 0},
	   {RST, Vol_Low, 0},	/* GPIO110 reset, see wt6755_66_sz_l.dts cam0_rst0 */
	   {AVDD, Vol_2800, 0},
	   {DOVDD, Vol_1800, 0},
	   {DVDD, Vol_1200, 10},
	   {PDN, Vol_Low, 0},
	   {PDN, Vol_High, 1},
	   {RST, Vol_High, 1},	/* release reset after PDN */
//	   {AFVDD, Vol_2800, 0},	//jerryxie, tp open
	   {AF_EN, Vol_Low, 0},
	   {AF_EN, Vol_High, 7},
	   },
	  },//allenyao, sunny2 back cam
	 {SENSOR_DRVNAME_OV5670_MIPI_RAW,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {PDN, Vol_Low, 0},
	   {RST, Vol_Low, 0},	/* GPIO110 reset, see wt6755_66_sz_l.dts cam0_rst0 */
	   {AVDD, Vol_2800, 0},
	   {DOVDD, Vol_1800, 0},
	   {DVDD, Vol_1200, 0},
	   {PDN, Vol_Low, 0},
	   {PDN, Vol_High, 1},
	   {RST, Vol_High, 1},	/* release reset after PDN */
	   },
	  },//wangjie, Kingcome front cam
	 {SENSOR_DRVNAME_S5K5E2YA_MIPI_RAW,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {DOVDD, Vol_1800, 0},
	   {AVDD, Vol_2800, 0},
	   {DVDD, Vol_1200, 0},
	   {AFVDD, Vol_2800, 5},
	   {PDN, Vol_Low, 4},
	   {PDN, Vol_High, 0},
	   {RST, Vol_Low, 1},
	   {RST, Vol_High, 0},
	   },
	  },
	 {SENSOR_DRVNAME_S5K2P8_MIPI_RAW,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {DOVDD, Vol_1800, 0},
	   {AVDD, Vol_2800, 0},
	   {DVDD, Vol_1200, 0},
	   {AFVDD, Vol_2800, 5},
	   {PDN, Vol_Low, 4},
	   {PDN, Vol_High, 0},
	   {RST, Vol_Low, 1},
	   {RST, Vol_High, 0},
	   },
	  },
	 {SENSOR_DRVNAME_OV5648_MIPI_RAW,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {DOVDD, Vol_1800, 1},
	   {AVDD, Vol_2800, 1},
	   {DVDD, Vol_1500, 1},
	   {AFVDD, Vol_2800, 5},
	   {PDN, Vol_Low, 1},
	   {RST, Vol_Low, 1},
	   {PDN, Vol_High, 0},
	   {RST, Vol_High, 0}
	   },
	  },
	 {SENSOR_DRVNAME_OV16825_MIPI_RAW,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {DOVDD, Vol_1800, 0},
	   {AVDD, Vol_2800, 0},
	   {DVDD, Vol_1200, 0},
	   {AFVDD, Vol_2800, 5},
	   {PDN, Vol_Low, 0},
	   {RST, Vol_Low, 0},
	   {RST, Vol_High, 0},
	   },
	  },
	 {SENSOR_DRVNAME_IMX135_MIPI_RAW,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {AVDD, Vol_2800, 10},
	   {DOVDD, Vol_1800, 10},
	   {DVDD, Vol_1000, 10},
	   {AFVDD, Vol_2800, 5},
	   {PDN, Vol_Low, 0},
	   {PDN, Vol_High, 0},
	   {RST, Vol_Low, 0},
	   {RST, Vol_High, 0}
	   },
	  },
	 {SENSOR_DRVNAME_OV8858_MIPI_RAW,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {PDN, Vol_Low, 0},
	   {RST, Vol_Low, 0},
	   {DOVDD, Vol_1800, 1},
	   {AVDD, Vol_2800, 1},
	   {DVDD, Vol_1200, 5},
	   {AFVDD, Vol_2800, 1},
	   {PDN, Vol_High, 1},
	   {RST, Vol_High, 2}
	   },
	  },
	  {SENSOR_DRVNAME_IMX258_MIPI_RAW,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {PDN, Vol_Low, 0},
	   {RST, Vol_Low, 0},
	   {DOVDD, Vol_1800, 0},
	   {AVDD, Vol_2800, 0},
	   {DVDD, Vol_1200, 0},
	   {AFVDD, Vol_2800, 1},
	   {PDN, Vol_High, 0},
	   {RST, Vol_High, 0}
	   },
	  },
	  {SENSOR_DRVNAME_IMX214_MIPI_RAW,
	  {
	   {SensorMCLK, Vol_High, 0},
	   {AVDD, Vol_2800, 10},
	   {DOVDD, Vol_1800, 10},
	   {DVDD, Vol_1000, 10},
	   {AFVDD, Vol_2800, 5},
	   {PDN, Vol_Low, 0},
	   {PDN, Vol_High, 0},
	   {RST, Vol_Low, 0},
	   {RST, Vol_High, 0}
	   },
	  },
	 /* add new sensor before this line */
	 {NULL,},
	 }
};



#ifndef CONFIG_MTK_LEGACY

#define VOL_2800 2800000
#define VOL_1800 1800000
#define VOL_1500 1500000
#define VOL_1200 1200000
#define VOL_1220 1220000
#define VOL_1000 1000000

/* GPIO Pin control*/
struct platform_device *cam_plt_dev = NULL;
struct pinctrl *camctrl = NULL;
struct pinctrl_state *cam0_pnd_h = NULL;/* main cam */
struct pinctrl_state *cam0_pnd_l = NULL;
struct pinctrl_state *cam0_rst_h = NULL;
struct pinctrl_state *cam0_rst_l = NULL;
struct pinctrl_state *cam1_pnd_h = NULL;/* sub cam */
struct pinctrl_state *cam1_pnd_l = NULL;
struct pinctrl_state *cam1_rst_h = NULL;
struct pinctrl_state *cam1_rst_l = NULL;
struct pinctrl_state *cam2_pnd_h = NULL;/* main2 cam */
struct pinctrl_state *cam2_pnd_l = NULL;
struct pinctrl_state *cam2_rst_h = NULL;
struct pinctrl_state *cam2_rst_l = NULL;
struct pinctrl_state *cam_ldo_vcama_h = NULL;/* for AVDD */
struct pinctrl_state *cam_ldo_vcama_l = NULL;
struct pinctrl_state *cam_ldo_vcamd_h = NULL;/* for DVDD */
struct pinctrl_state *cam_ldo_vcamd_l = NULL;
struct pinctrl_state *cam_ldo_vcamio_h = NULL;/* for DOVDD */
struct pinctrl_state *cam_ldo_vcamio_l = NULL;
struct pinctrl_state *cam_ldo_vcamaf_h = NULL;/* for AFVDD */
struct pinctrl_state *cam_ldo_vcamaf_l = NULL;
struct pinctrl_state *cam_ldo_sub_vcamd_h = NULL;/* for SUB_DVDD */
struct pinctrl_state *cam_ldo_sub_vcamd_l = NULL;
struct pinctrl_state *cam_ldo_main2_vcamd_h = NULL;/* for MAIN2_DVDD */
struct pinctrl_state *cam_ldo_main2_vcamd_l = NULL;
/* AF motor enable (DCT GPIO21): the stock Flyme DTB drives it with the
 * pinctrl states "cam0_af_en0/1" on kd_camera_hw1. */
struct pinctrl_state *cam0_af_en_h = NULL;
struct pinctrl_state *cam0_af_en_l = NULL;
/* CMMCLK pad mux (stock DCT: GPIO161 = CMMCLK0 -> main MCLK1, GPIO109 =
 * CMMCLK1 -> sub MCLK2). The stock kernel relied on the DCT boot-time pad
 * init; here the mux is selected explicitly around ISP_MCLK*_EN. _off parks
 * the pad as GPIO output low. */
struct pinctrl_state *cam0_mclk_on = NULL;
struct pinctrl_state *cam0_mclk_off = NULL;
struct pinctrl_state *cam1_mclk_on = NULL;
struct pinctrl_state *cam1_mclk_off = NULL;


int mtkcam_gpio_init(struct platform_device *pdev)
{
	int ret = 0;

	camctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(camctrl)) {
		dev_err(&pdev->dev, "Cannot find camera pinctrl!");
		ret = PTR_ERR(camctrl);
	}
	/*Cam0 Power/Rst Ping initialization */
	cam0_pnd_h = pinctrl_lookup_state(camctrl, "cam0_pnd1");
	if (IS_ERR(cam0_pnd_h)) {
		ret = PTR_ERR(cam0_pnd_h);
		pr_debug("%s : pinctrl err, cam0_pnd_h\n", __func__);
	}

	cam0_pnd_l = pinctrl_lookup_state(camctrl, "cam0_pnd0");
	if (IS_ERR(cam0_pnd_l)) {
		ret = PTR_ERR(cam0_pnd_l);
		pr_debug("%s : pinctrl err, cam0_pnd_l\n", __func__);
	}


	cam0_rst_h = pinctrl_lookup_state(camctrl, "cam0_rst1");
	if (IS_ERR(cam0_rst_h)) {
		ret = PTR_ERR(cam0_rst_h);
		pr_debug("%s : pinctrl err, cam0_rst_h\n", __func__);
	}

	cam0_rst_l = pinctrl_lookup_state(camctrl, "cam0_rst0");
	if (IS_ERR(cam0_rst_l)) {
		ret = PTR_ERR(cam0_rst_l);
		pr_debug("%s : pinctrl err, cam0_rst_l\n", __func__);
	}

	/*Cam1 Power/Rst Ping initialization */
	cam1_pnd_h = pinctrl_lookup_state(camctrl, "cam1_pnd1");
	if (IS_ERR(cam1_pnd_h)) {
		ret = PTR_ERR(cam1_pnd_h);
		pr_debug("%s : pinctrl err, cam1_pnd_h\n", __func__);
	}

	cam1_pnd_l = pinctrl_lookup_state(camctrl, "cam1_pnd0");
	if (IS_ERR(cam1_pnd_l)) {
		ret = PTR_ERR(cam1_pnd_l);
		pr_debug("%s : pinctrl err, cam1_pnd_l\n", __func__);
	}


	cam1_rst_h = pinctrl_lookup_state(camctrl, "cam1_rst1");
	if (IS_ERR(cam1_rst_h)) {
		ret = PTR_ERR(cam1_rst_h);
		pr_debug("%s : pinctrl err, cam1_rst_h\n", __func__);
	}


	cam1_rst_l = pinctrl_lookup_state(camctrl, "cam1_rst0");
	if (IS_ERR(cam1_rst_l)) {
		ret = PTR_ERR(cam1_rst_l);
		pr_debug("%s : pinctrl err, cam1_rst_l\n", __func__);
	}
	/*Cam2 Power/Rst Ping initialization */
	cam2_pnd_h = pinctrl_lookup_state(camctrl, "cam2_pnd1");
	if (IS_ERR(cam2_pnd_h)) {
		ret = PTR_ERR(cam2_pnd_h);
		pr_debug("%s : pinctrl err, cam2_pnd_h\n", __func__);
	}

	cam2_pnd_l = pinctrl_lookup_state(camctrl, "cam2_pnd0");
	if (IS_ERR(cam2_pnd_l)) {
		ret = PTR_ERR(cam2_pnd_l);
		pr_debug("%s : pinctrl err, cam2_pnd_l\n", __func__);
	}


	cam2_rst_h = pinctrl_lookup_state(camctrl, "cam2_rst1");
	if (IS_ERR(cam2_rst_h)) {
		ret = PTR_ERR(cam2_rst_h);
		pr_debug("%s : pinctrl err, cam2_rst_h\n", __func__);
	}


	cam2_rst_l = pinctrl_lookup_state(camctrl, "cam2_rst0");
	if (IS_ERR(cam2_rst_l)) {
		ret = PTR_ERR(cam2_rst_l);
		pr_debug("%s : pinctrl err, cam2_rst_l\n", __func__);
	}

	/*externel LDO enable */
	cam_ldo_vcama_h = pinctrl_lookup_state(camctrl, "cam_ldo_vcama_1");
	if (IS_ERR(cam_ldo_vcama_h)) {
		ret = PTR_ERR(cam_ldo_vcama_h);
		pr_debug("%s : pinctrl err, cam_ldo_vcama_h\n", __func__);
	}

	cam_ldo_vcama_l = pinctrl_lookup_state(camctrl, "cam_ldo_vcama_0");
	if (IS_ERR(cam_ldo_vcama_l)) {
		ret = PTR_ERR(cam_ldo_vcama_l);
		pr_debug("%s : pinctrl err, cam_ldo_vcama_l\n", __func__);
	}

	cam_ldo_vcamd_h = pinctrl_lookup_state(camctrl, "cam_ldo_vcamd_1");
	if (IS_ERR(cam_ldo_vcamd_h)) {
		ret = PTR_ERR(cam_ldo_vcamd_h);
		pr_debug("%s : pinctrl err, cam_ldo_vcamd_h\n", __func__);
	}

	cam_ldo_vcamd_l = pinctrl_lookup_state(camctrl, "cam_ldo_vcamd_0");
	if (IS_ERR(cam_ldo_vcamd_l)) {
		ret = PTR_ERR(cam_ldo_vcamd_l);
		pr_debug("%s : pinctrl err, cam_ldo_vcamd_l\n", __func__);
	}

	cam_ldo_vcamio_h = pinctrl_lookup_state(camctrl, "cam_ldo_vcamio_1");
	if (IS_ERR(cam_ldo_vcamio_h)) {
		ret = PTR_ERR(cam_ldo_vcamio_h);
		pr_debug("%s : pinctrl err, cam_ldo_vcamio_h\n", __func__);
	}

	cam_ldo_vcamio_l = pinctrl_lookup_state(camctrl, "cam_ldo_vcamio_0");
	if (IS_ERR(cam_ldo_vcamio_l)) {
		ret = PTR_ERR(cam_ldo_vcamio_l);
		pr_debug("%s : pinctrl err, cam_ldo_vcamio_l\n", __func__);
	}

	cam_ldo_vcamaf_h = pinctrl_lookup_state(camctrl, "cam_ldo_vcamaf_1");
	if (IS_ERR(cam_ldo_vcamaf_h)) {
		ret = PTR_ERR(cam_ldo_vcamaf_h);
		pr_debug("%s : pinctrl err, cam_ldo_vcamaf_h\n", __func__);
	}

	cam_ldo_vcamaf_l = pinctrl_lookup_state(camctrl, "cam_ldo_vcamaf_0");
	if (IS_ERR(cam_ldo_vcamaf_l)) {
		ret = PTR_ERR(cam_ldo_vcamaf_l);
		pr_debug("%s : pinctrl err, cam_ldo_vcamaf_l\n", __func__);
	}

	cam_ldo_sub_vcamd_h = pinctrl_lookup_state(camctrl, "cam_ldo_sub_vcamd_1");
	if (IS_ERR(cam_ldo_sub_vcamd_h)) {
		ret = PTR_ERR(cam_ldo_sub_vcamd_h);
		pr_debug("%s : pinctrl err, cam_ldo_sub_vcamd_h\n", __func__);
	}

	cam_ldo_sub_vcamd_l = pinctrl_lookup_state(camctrl, "cam_ldo_sub_vcamd_0");
	if (IS_ERR(cam_ldo_sub_vcamd_l)) {
		ret = PTR_ERR(cam_ldo_sub_vcamd_l);
		pr_debug("%s : pinctrl err, cam_ldo_sub_vcamd_l\n", __func__);
	}

	cam_ldo_main2_vcamd_h = pinctrl_lookup_state(camctrl, "cam_ldo_main2_vcamd_1");
	if (IS_ERR(cam_ldo_main2_vcamd_h)) {
		ret = PTR_ERR(cam_ldo_main2_vcamd_h);
		pr_debug("%s : pinctrl err, cam_ldo_main2_vcamd_h\n", __func__);
	}

	cam_ldo_main2_vcamd_l = pinctrl_lookup_state(camctrl, "cam_ldo_main2_vcamd_0");
	if (IS_ERR(cam_ldo_main2_vcamd_l)) {
		ret = PTR_ERR(cam_ldo_main2_vcamd_l);
		pr_debug("%s : pinctrl err, cam_ldo_main2_vcamd_l\n", __func__);
	}

	/* AF motor enable states (stock DTB names cam0_af_en0/1) */
	cam0_af_en_h = pinctrl_lookup_state(camctrl, "cam0_af_en1");
	if (IS_ERR(cam0_af_en_h))
		pr_debug("%s : pinctrl err, cam0_af_en_h\n", __func__);
	cam0_af_en_l = pinctrl_lookup_state(camctrl, "cam0_af_en0");
	if (IS_ERR(cam0_af_en_l))
		pr_debug("%s : pinctrl err, cam0_af_en_l\n", __func__);

	/* CMMCLK pad mux states (see the declaration). Missing states are
	 * reported at use time (hwpoweron), not here. */
	cam0_mclk_on = pinctrl_lookup_state(camctrl, "cam0_mclk_on");
	if (IS_ERR(cam0_mclk_on))
		pr_debug("%s : pinctrl err, cam0_mclk_on\n", __func__);
	cam0_mclk_off = pinctrl_lookup_state(camctrl, "cam0_mclk_off");
	if (IS_ERR(cam0_mclk_off))
		pr_debug("%s : pinctrl err, cam0_mclk_off\n", __func__);
	cam1_mclk_on = pinctrl_lookup_state(camctrl, "cam1_mclk_on");
	if (IS_ERR(cam1_mclk_on))
		pr_debug("%s : pinctrl err, cam1_mclk_on\n", __func__);
	cam1_mclk_off = pinctrl_lookup_state(camctrl, "cam1_mclk_off");
	if (IS_ERR(cam1_mclk_off))
		pr_debug("%s : pinctrl err, cam1_mclk_off\n", __func__);

	return ret;
}

/*
 * L681: set the CMMCLK pads to the boot mode the L681 DCT gives them
 * (GPIO161_MODE = GPIO109_MODE = GPIO_MODE_01, DIR_OUT); the stock kernel
 * relied on that boot setting and never muxed them itself. Written
 * directly after the pinctrl state, then read back and logged. Camera pads
 * only.
 */
extern int mt_set_gpio_mode(unsigned long pin, unsigned long mode);
extern int mt_set_gpio_dir(unsigned long pin, unsigned long dir);
extern int mt_get_gpio_mode(unsigned long pin);
extern int mt_get_gpio_dir(unsigned long pin);

static void forge_cam_l681_mclk_pad(unsigned long pad)
{
	unsigned long pin = pad | 0x80000000;

	if (!m3note_board_known_is(M3NOTE_BOARD_L681))
		return;
	mt_set_gpio_mode(pin, 1);	/* GPIO_MODE_01 = CMMCLK0/1 */
	mt_set_gpio_dir(pin, 1);	/* GPIO_DIR_OUT */
	pr_err("[FORGE_CAM] l681 MCLK pad GPIO%lu -> mode=%d dir=%d\n",
	       pad, mt_get_gpio_mode(pin), mt_get_gpio_dir(pin));
}

int mtkcam_gpio_set(int PinIdx, int PwrType, int Val)
{
	int ret = 0;
	if (IS_ERR(camctrl)) {
		return -1;
	}
	switch (PwrType) {
	case RST:
		/* Log the pin step so the line state can be read from the
		 * log. */
		pr_err("[FORGE_CAM] PIN RST idx=%d val=%d h_ok=%d l_ok=%d\n",
		       PinIdx, Val, !IS_ERR_OR_NULL(cam0_rst_h), !IS_ERR_OR_NULL(cam0_rst_l));
		if (PinIdx == 0) {
			if (Val == 0 && !IS_ERR(cam0_rst_l))
				pinctrl_select_state(camctrl, cam0_rst_l);
			else if (Val == 1 && !IS_ERR(cam0_rst_h))
				pinctrl_select_state(camctrl, cam0_rst_h);
		} else if (PinIdx == 1) {
			if (Val == 0 && !IS_ERR(cam1_rst_l))
				pinctrl_select_state(camctrl, cam1_rst_l);
			else if (Val == 1 && !IS_ERR(cam1_rst_h))
				pinctrl_select_state(camctrl, cam1_rst_h);
		} else {
			if (Val == 0 && !IS_ERR(cam2_rst_l))
				pinctrl_select_state(camctrl, cam2_rst_l);
			else if (Val == 1 && !IS_ERR(cam2_rst_h))
				pinctrl_select_state(camctrl, cam2_rst_h);
		}
		break;
	case PDN:
		/* Log the pin step so the line state can be read from the
		 * log. */
		pr_err("[FORGE_CAM] PIN PDN idx=%d val=%d h_ok=%d l_ok=%d\n",
		       PinIdx, Val, !IS_ERR_OR_NULL(cam0_pnd_h), !IS_ERR_OR_NULL(cam0_pnd_l));
		if (PinIdx == 0) {
			if (Val == 0 && !IS_ERR(cam0_pnd_l))
				pinctrl_select_state(camctrl, cam0_pnd_l);
			else if (Val == 1 && !IS_ERR(cam0_pnd_h))
				pinctrl_select_state(camctrl, cam0_pnd_h);
		} else if (PinIdx == 1) {
			if (Val == 0 && !IS_ERR(cam1_pnd_l))
				pinctrl_select_state(camctrl, cam1_pnd_l);
			else if (Val == 1 && !IS_ERR(cam1_pnd_h))
				pinctrl_select_state(camctrl, cam1_pnd_h);
		} else {
			if (Val == 0 && !IS_ERR(cam2_pnd_l))
				pinctrl_select_state(camctrl, cam2_pnd_l);
			else if (Val == 1 && !IS_ERR(cam2_pnd_h))
				pinctrl_select_state(camctrl, cam2_pnd_h);
		}
		break;
	case AVDD:
		if (Val == 0 && !IS_ERR(cam_ldo_vcama_l))
			pinctrl_select_state(camctrl, cam_ldo_vcama_l);
		else if (Val == 1 && !IS_ERR(cam0_rst_h))
			pinctrl_select_state(camctrl, cam_ldo_vcama_h);
		break;
	case DVDD:
		if (Val == 0 && !IS_ERR(cam_ldo_vcamd_l))
			pinctrl_select_state(camctrl, cam_ldo_vcamd_l);
		else if (Val == 1 && !IS_ERR(cam_ldo_vcamd_h))
			pinctrl_select_state(camctrl, cam_ldo_vcamd_h);
		break;
	case DOVDD:
		if (Val == 0 && !IS_ERR(cam_ldo_vcamio_l))
			pinctrl_select_state(camctrl, cam_ldo_vcamio_l);
		else if (Val == 1 && !IS_ERR(cam_ldo_vcamio_h))
			pinctrl_select_state(camctrl, cam_ldo_vcamio_h);
		break;
	case AFVDD:
		if (Val == 0 && !IS_ERR(cam_ldo_vcamaf_l))
			pinctrl_select_state(camctrl, cam_ldo_vcamaf_l);
		else if (Val == 1 && !IS_ERR(cam_ldo_vcamaf_h))
			pinctrl_select_state(camctrl, cam_ldo_vcamaf_h);
		break;
	case SUB_DVDD:
		if (Val == 0 && !IS_ERR(cam_ldo_sub_vcamd_l))
			pinctrl_select_state(camctrl, cam_ldo_sub_vcamd_l);
		else if (Val == 1 && !IS_ERR(cam_ldo_sub_vcamd_h))
			pinctrl_select_state(camctrl, cam_ldo_sub_vcamd_h);
		break;
	case MAIN2_DVDD:
		if (Val == 0 && !IS_ERR(cam_ldo_main2_vcamd_l))
			pinctrl_select_state(camctrl, cam_ldo_main2_vcamd_l);
		else if (Val == 1 && !IS_ERR(cam_ldo_main2_vcamd_h))
			pinctrl_select_state(camctrl, cam_ldo_main2_vcamd_h);
		break;
	default:
		PK_DBG("PwrType(%d) is invalid !!\n", PwrType);
		break;
	};

	PK_DBG("PinIdx(%d) PwrType(%d) val(%d)\n", PinIdx, PwrType, Val);

	return ret;
}

BOOL hwpoweron(PowerInformation pwInfo, char *mode_name)
{
	if (pwInfo.PowerType == AVDD) {
		if (PowerCustList.PowerCustInfo[CUST_AVDD].Gpio_Pin == GPIO_UNSUPPORTED) {
			if (TRUE != _hwPowerOn(pwInfo.PowerType, pwInfo.Voltage)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
				return FALSE;
			}
		} else {
			if (mtkcam_gpio_set(pinSetIdx, CUST_AVDD, PowerCustList.PowerCustInfo[CUST_AVDD].Voltage)) {
					PK_DBG("[CAMERA CUST_AVDD] set gpio failed!!\n");
			}
		}
	} else if (pwInfo.PowerType == DVDD) {
		if (pinSetIdx == 2) {
			if (PowerCustList.PowerCustInfo[CUST_MAIN2_DVDD].Gpio_Pin == GPIO_UNSUPPORTED) {
				/* PK_DBG("[CAMERA SENSOR] Sub camera VCAM_D power on"); */
				if (pwInfo.Voltage == Vol_1200) {
					pwInfo.Voltage = Vol_1220;
					PK_DBG("[CAMERA SENSOR] Main2 camera VCAM_D power 1.2V to 1.22V\n");
				}
				if (TRUE != _hwPowerOn(MAIN2_DVDD, pwInfo.Voltage)) {
					PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					return FALSE;
				}
			} else {
				if (mtkcam_gpio_set(pinSetIdx, CUST_MAIN2_DVDD, PowerCustList.PowerCustInfo[CUST_MAIN2_DVDD].Voltage)) {
						PK_DBG("[CAMERA CUST_MAIN2_DVDD] set gpio failed!!\n");
				}
			}
		} else if (pinSetIdx == 1) {
			if (PowerCustList.PowerCustInfo[CUST_SUB_DVDD].Gpio_Pin == GPIO_UNSUPPORTED) {
				int req = (int)pwInfo.Voltage;
				bool ok;

				/* PK_DBG("[CAMERA SENSOR] Sub camera VCAM_D power on"); */
				if (pwInfo.Voltage == Vol_1200) {
					/* The live VCAMD ladder is the MT6353 one
					 * (pmic/mt6353/regulator_codegen.c vcamd_voltages[] =
					 * {900000, 1000000, 1100000, 1220000, 1300000,
					 * 1500000, 1800000}, DT ceiling 1.8 V): mt6755.dtsi
					 * includes the PMIC dtsi by
					 * CONFIG_MTK_PMIC_CHIP_MT6353, so mt6351.dtsi is not
					 * built. 1210000 is not a step, so
					 * regulator_map_voltage() cannot satisfy an exact
					 * [1210000,1210000] window (-EIO). 1220000 is what
					 * the main DVDD path asks for on the same physical
					 * LDO (vcamd-supply and vcamd_sub-supply both resolve
					 * to mt_pmic_vcamd_ldo_reg); anything else loses to
					 * regulator_check_consumers(). MediaTek's own
					 * sequence selects 1.22 V and trims -20 mV via
					 * PMIC_RG_VCAMD_CAL, landing at ~1.20 V. */
					pwInfo.Voltage = Vol_1220;
				}
				ok = (TRUE == _hwPowerOn(SUB_DVDD, pwInfo.Voltage));
				/* Print the number rather than leaving it to be inferred from an
				 * absence.  The 07-20 guard's real defect was that it fell
				 * through SILENTLY when the value did not match, so the log
				 * carried no trace of the value that actually killed the rail;
				 *  then inferred that value wrongly.  Rail readback is
				 * included because our own request alone would have read a
				 * plausible-looking 1210000 and told nobody anything. */
				PK_ERR("[FORGE_CAM] SUB_DVDD req=%d -> %d (ladder step; rail now %d, ok=%d)\n",
				       req, (int)pwInfo.Voltage, forge_cam_vcamd_uv(), ok);
				if (!ok) {
					PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					return FALSE;
				}
			} else {
				if (mtkcam_gpio_set(pinSetIdx, CUST_SUB_DVDD, PowerCustList.PowerCustInfo[CUST_SUB_DVDD].Voltage)) {
					PK_DBG("[CAMERA CUST_SUB_DVDD] set gpio failed!!\n");
				}
			}
		} else {
			if (PowerCustList.PowerCustInfo[CUST_DVDD].Gpio_Pin == GPIO_UNSUPPORTED) {
				/* PK_DBG("[CAMERA SENSOR] Main camera VCAM_D power on"); */
				if (TRUE != _hwPowerOn(pwInfo.PowerType, pwInfo.Voltage)) {
					PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					return FALSE;
				}
			} else {
				if (mtkcam_gpio_set(pinSetIdx, CUST_DVDD, PowerCustList.PowerCustInfo[CUST_DVDD].Voltage)) {
					PK_DBG("[CAMERA CUST_DVDD] set gpio failed!!\n");
				}
			}
		}
	} else if (pwInfo.PowerType == DOVDD) {
		if (PowerCustList.PowerCustInfo[CUST_DOVDD].Gpio_Pin == GPIO_UNSUPPORTED) {
			if (TRUE != _hwPowerOn(pwInfo.PowerType, pwInfo.Voltage)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
				return FALSE;
			}
		} else {
			if (mtkcam_gpio_set(pinSetIdx, CUST_DOVDD, PowerCustList.PowerCustInfo[CUST_DOVDD].Voltage)) {
					PK_DBG("[CAMERA CUST_DOVDD] set gpio failed!!\n");
			}

		}
	} else if (pwInfo.PowerType == AFVDD) {
#if 0
		PK_DBG("[CAMERA SENSOR] Skip AFVDD setting\n");
		if (PowerCustList.PowerCustInfo[3].Gpio_Pin == GPIO_UNSUPPORTED) {
			if (TRUE != hwPowerOn(pwInfo.PowerType, pwInfo.Voltage * 1000, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
				return FALSE;
			}
		} else {
			if (mt_set_gpio_mode
			    (PowerCustList.PowerCustInfo[3].Gpio_Pin,
			     PowerCustList.PowerCustInfo[3].Gpio_Mode)) {
				PK_DBG("[CAMERA SENSOR] set gpio mode failed!!\n");
			}
			if (mt_set_gpio_dir(PowerCustList.PowerCustInfo[3].Gpio_Pin, GPIO_DIR_OUT)) {
				PK_DBG("[CAMERA SENSOR] set gpio dir failed!!\n");
			}
			if (mt_set_gpio_out
			    (PowerCustList.PowerCustInfo[3].Gpio_Pin,
			     PowerCustList.PowerCustInfo[3].Voltage)) {
				PK_DBG("[CAMERA SENSOR] set gpio failed!!\n");
			}

			if (PowerCustList.PowerCustInfo[4].Gpio_Pin != GPIO_UNSUPPORTED) {
				mdelay(5);
				if (mt_set_gpio_mode
				    (PowerCustList.PowerCustInfo[3].Gpio_Pin,
				     PowerCustList.PowerCustInfo[3].Gpio_Mode)) {
					PK_DBG("[CAMERA SENSOR] set gpio mode failed!!\n");
				}
				if (mt_set_gpio_dir
				    (PowerCustList.PowerCustInfo[3].Gpio_Pin, GPIO_DIR_OUT)) {
					PK_DBG("[CAMERA SENSOR] set gpio dir failed!!\n");
				}
				if (mt_set_gpio_out
				    (PowerCustList.PowerCustInfo[3].Gpio_Pin,
				     PowerCustList.PowerCustInfo[3].Voltage)) {
					PK_DBG("[CAMERA SENSOR] set gpio failed!!\n");
				}
			}
		}
#endif
	} else if (pwInfo.PowerType == PDN) {
		/* PK_DBG("hwPowerOn: PDN %d\n", pwInfo.Voltage); */
		mtkcam_gpio_set(pinSetIdx, PDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]);


		if (pwInfo.Voltage == Vol_High) {
			if (mtkcam_gpio_set(pinSetIdx, PDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_ON])) {
				PK_DBG("[CAMERA SENSOR] set gpio failed!!\n");
			}
		} else {
			if (mtkcam_gpio_set(pinSetIdx, PDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF])) {

				PK_DBG("[CAMERA SENSOR] set gpio failed!!\n");
			}
		}
	} else if (pwInfo.PowerType == RST) {
		mtkcam_gpio_set(pinSetIdx, RST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
		if (pwInfo.Voltage == Vol_High) {
			if (mtkcam_gpio_set(pinSetIdx, RST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON])) {
				PK_DBG("[CAMERA SENSOR] set gpio failed!!\n");
			}
		} else {
			if (mtkcam_gpio_set(pinSetIdx, RST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF])) {

				PK_DBG("[CAMERA SENSOR] set gpio failed!!\n");
			}
		}

	} else if (pwInfo.PowerType == AF_EN) {//wangjie, af en pin
		/* AF motor enable = GPIO21 via the pinctrl states
		 * cam0_af_en0/1, as in the stock DTB. */
		if (pwInfo.Voltage == Vol_High && !IS_ERR_OR_NULL(cam0_af_en_h))
			pinctrl_select_state(camctrl, cam0_af_en_h);
		else if (pwInfo.Voltage != Vol_High && !IS_ERR_OR_NULL(cam0_af_en_l))
			pinctrl_select_state(camctrl, cam0_af_en_l);
		else
			pr_err("[FORGE_CAM] AF_EN(%d): no cam0_af_en pinctrl state\n",
			       pwInfo.Voltage);
	} else if (pwInfo.PowerType == SensorMCLK) {
		/* Mux the CMMCLK pad, then enable the SENINF MCLK output
		 * (mt_cam_mclk.c). Both failure paths are logged. */
		if (pinSetIdx == 0) {
			/* PK_DBG("Sensor MCLK1 On"); */
			if (!IS_ERR_OR_NULL(cam0_mclk_on))
				pinctrl_select_state(camctrl, cam0_mclk_on);
			else
				pr_err("[FORGE_CAM] MCLK1: no cam0_mclk_on pinctrl state - CMMCLK0 pad (GPIO161) mux is UNDEFINED, a sensor NAK here is not identity evidence\n");
			forge_cam_l681_mclk_pad(161);
			ISP_MCLK1_EN(TRUE);
		} else if (pinSetIdx == 1) {
			/* PK_DBG("Sensor MCLK2 On"); */
			if (!IS_ERR_OR_NULL(cam1_mclk_on))
				pinctrl_select_state(camctrl, cam1_mclk_on);
			else
				pr_err("[FORGE_CAM] MCLK2: no cam1_mclk_on pinctrl state - CMMCLK1 pad (GPIO109) mux is UNDEFINED, a sensor NAK here is not identity evidence\n");
			forge_cam_l681_mclk_pad(109);
			ISP_MCLK2_EN(TRUE);
		}
	} else {
	}
	if (pwInfo.Delay > 0)
		mdelay(pwInfo.Delay);
	return TRUE;
}



BOOL hwpowerdown(PowerInformation pwInfo, char *mode_name)
{
	if (pwInfo.PowerType == AVDD) {
		if (PowerCustList.PowerCustInfo[CUST_AVDD].Gpio_Pin == GPIO_UNSUPPORTED) {
			if (TRUE != _hwPowerDown(pwInfo.PowerType)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
				return FALSE;
			}
		} else {
			if (mtkcam_gpio_set(pinSetIdx, CUST_AVDD, 1-PowerCustList.PowerCustInfo[CUST_DOVDD].Voltage)) {
					PK_DBG("[CAMERA CUST_AVDD] set gpio failed!!\n");/* 1-voltage for reverse*/
			}
		}
	} else if (pwInfo.PowerType == DVDD) {
		if (PowerCustList.PowerCustInfo[1].Gpio_Pin == GPIO_UNSUPPORTED) {
			if (pinSetIdx == 1) {
				if (TRUE != _hwPowerDown(SUB_DVDD)) {
					PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					return FALSE;
				}
			} else if (TRUE != _hwPowerDown(pwInfo.PowerType)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
				return FALSE;
			} else {
			}
		} else {
#if 0
			if (mt_set_gpio_mode
			    (PowerCustList.PowerCustInfo[1].Gpio_Pin,
			     PowerCustList.PowerCustInfo[1].Gpio_Mode)) {
				PK_DBG("[CAMERA LENS] set gpio mode failed!!\n");
			}
			if (mt_set_gpio_dir(PowerCustList.PowerCustInfo[1].Gpio_Pin, GPIO_DIR_OUT)) {
				PK_DBG("[CAMERA LENS] set gpio dir failed!!\n");
			}
			if (mt_set_gpio_out
			    (PowerCustList.PowerCustInfo[1].Gpio_Pin,
			     PowerCustList.PowerCustInfo[1].Voltage)) {
				PK_DBG("[CAMERA LENS] set gpio failed!!\n");
			}
#endif
		}
	} else if (pwInfo.PowerType == DOVDD) {
		if (PowerCustList.PowerCustInfo[2].Gpio_Pin == GPIO_UNSUPPORTED) {
			if (TRUE != _hwPowerDown(pwInfo.PowerType)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
				return FALSE;
			}
		} else {
			if (mtkcam_gpio_set(pinSetIdx, CUST_DOVDD, 1-PowerCustList.PowerCustInfo[CUST_DOVDD].Voltage)) {
				PK_DBG("[CAMERA CUST_AVDD] set gpio failed!!\n");/* 1-voltage for reverse*/
			}
		}
	} else if (pwInfo.PowerType == AFVDD) {
#if 0
		PK_DBG("[CAMERA SENSOR] Skip AFVDD setting\n");
		if (PowerCustList.PowerCustInfo[3].Gpio_Pin == GPIO_UNSUPPORTED) {
			if (TRUE != hwPowerDown(pwInfo.PowerType, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
				return FALSE;
			}
		} else {
			if (mt_set_gpio_mode
			    (PowerCustList.PowerCustInfo[3].Gpio_Pin,
			     PowerCustList.PowerCustInfo[3].Gpio_Mode)) {
				PK_DBG("[CAMERA LENS] set gpio mode failed!!\n");
			}
			if (mt_set_gpio_dir(PowerCustList.PowerCustInfo[3].Gpio_Pin, GPIO_DIR_OUT)) {
				PK_DBG("[CAMERA LENS] set gpio dir failed!!\n");
			}
			if (mt_set_gpio_out
			    (PowerCustList.PowerCustInfo[3].Gpio_Pin,
			     PowerCustList.PowerCustInfo[3].Voltage)) {
				PK_DBG("[CAMERA LENS] set gpio failed!!\n");
			}

			if (PowerCustList.PowerCustInfo[4].Gpio_Pin != GPIO_UNSUPPORTED) {
				mdelay(5);
				if (mt_set_gpio_mode
				    (PowerCustList.PowerCustInfo[3].Gpio_Pin,
				     PowerCustList.PowerCustInfo[3].Gpio_Mode)) {
					PK_DBG("[CAMERA LENS] set gpio mode failed!!\n");
				}
				if (mt_set_gpio_dir
				    (PowerCustList.PowerCustInfo[3].Gpio_Pin, GPIO_DIR_OUT)) {
					PK_DBG("[CAMERA LENS] set gpio dir failed!!\n");
				}
				if (mt_set_gpio_out
				    (PowerCustList.PowerCustInfo[3].Gpio_Pin,
				     PowerCustList.PowerCustInfo[3].Voltage)) {
					PK_DBG("[CAMERA LENS] set gpio failed!!\n");
				}
			}
		}
#endif
	} else if (pwInfo.PowerType == PDN) {
		PK_DBG("hwPowerDown: PDN %d\n", pwInfo.Voltage);


		if (pwInfo.Voltage == Vol_High) {
			if (mtkcam_gpio_set(pinSetIdx, PDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_ON])) {
				PK_DBG("[CAMERA SENSOR] set gpio failed!!\n");
			}
		} else {
			if (mtkcam_gpio_set(pinSetIdx, PDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF])) {

				PK_DBG("[CAMERA SENSOR] set gpio failed!!\n");
			}
		}
	} else if (pwInfo.PowerType == RST) {
		PK_DBG("hwPowerDown: RST %d\n", pwInfo.Voltage);
		if (pwInfo.Voltage == Vol_High) {
			if (mtkcam_gpio_set(pinSetIdx, RST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON])) {
				PK_DBG("[CAMERA SENSOR] set gpio failed!!\n");
			}
		} else {
			if (mtkcam_gpio_set(pinSetIdx, RST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF])) {

				PK_DBG("[CAMERA SENSOR] set gpio failed!!\n");
			}
		}

	} else if (pwInfo.PowerType == AF_EN) {//wangjie, af en pin
		/* AF motor enable off via pinctrl (see hwpoweron). */
		if (!IS_ERR_OR_NULL(cam0_af_en_l))
			pinctrl_select_state(camctrl, cam0_af_en_l);
		else
			pr_err("[FORGE_CAM] AF_EN off: no cam0_af_en0 pinctrl state\n");
	} else if (pwInfo.PowerType == SensorMCLK) {
		/* Stop the SENINF output first, then park the pad. */
		if (pinSetIdx == 0) {
			ISP_MCLK1_EN(FALSE);
			if (!IS_ERR_OR_NULL(cam0_mclk_off))
				pinctrl_select_state(camctrl, cam0_mclk_off);
		} else if (pinSetIdx == 1) {
			ISP_MCLK2_EN(FALSE);
			if (!IS_ERR_OR_NULL(cam1_mclk_off))
				pinctrl_select_state(camctrl, cam1_mclk_off);
		}
	} else {
	}
	return TRUE;
}




int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, BOOL On,
		       char *mode_name)
{

	int pwListIdx, pwIdx;
	BOOL sensorInPowerList = KAL_FALSE;

	/* power gate: see forge_cam_power above */
	if (!forge_cam_power) {
		pr_err("[FORGE_CAM] kdCISModulePowerOn(%s, on=%d) GATED OFF (forge_cam_power=0)\n",
		       currSensorName ? currSensorName : "?", On);
		return -EPERM;
	}

	if (DUAL_CAMERA_MAIN_SENSOR == SensorIdx) {
		pinSetIdx = 0;
	} else if (DUAL_CAMERA_SUB_SENSOR == SensorIdx) {
		pinSetIdx = 1;
	} else if (DUAL_CAMERA_MAIN_2_SENSOR == SensorIdx) {
		pinSetIdx = 2;
	}
	/* power ON */
	if (On) {
		PK_DBG("kdCISModulePowerOn -on:currSensorName=%s\n", currSensorName);
		PK_DBG("kdCISModulePowerOn -on:pinSetIdx=%d\n", pinSetIdx);

		for (pwListIdx = forge_cam_pwr_first(); pwListIdx < 24; pwListIdx++) {
			if (currSensorName && (PowerOnList.PowerSeq[pwListIdx].SensorName != NULL)
			    && (0 ==
				strcmp(PowerOnList.PowerSeq[pwListIdx].SensorName,
				       currSensorName))) {
				PK_DBG("sensorIdx:%d\n", SensorIdx);

				sensorInPowerList = KAL_TRUE;

				for (pwIdx = 0; pwIdx < 16; pwIdx++) {
					if (PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].
					    PowerType != VDD_None) {
						if (hwpoweron
						    (PowerOnList.PowerSeq[pwListIdx].
						     PowerInfo[pwIdx], mode_name) == FALSE)
							goto _kdCISModulePowerOn_exit_;
					} else {
						/* PK_DBG("pwIdx=%d\n", pwIdx); */
						break;
					}
				}
				break;
			} else if (PowerOnList.PowerSeq[pwListIdx].SensorName == NULL) {
				break;
			} else {
			}
		}
#if 0
		/* Temp solution: default power on/off sequence */
		if (KAL_FALSE == sensorInPowerList) {
			PK_DBG("Default power on sequence");

			if (pinSetIdx == 0) {
				ISP_MCLK1_EN(1);
			} else if (pinSetIdx == 1) {
				ISP_MCLK2_EN(1);
			}
			/* First Power Pin low and Reset Pin Low */
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
				if (mt_set_gpio_mode
				    (pinSet[pinSetIdx][IDX_PS_CMPDN],
				     pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_MODE])) {
					PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");
				}
				if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_DIR_OUT)) {
					PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");
				}
				if (mt_set_gpio_out
				    (pinSet[pinSetIdx][IDX_PS_CMPDN],
				     pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF])) {
					PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");
				}
			}

			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
				if (0 == pinSetIdx) {
#ifndef CONFIG_MTK_MT6306_SUPPORT
					if (mt_set_gpio_mode
					    (pinSet[pinSetIdx][IDX_PS_CMRST],
					     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_MODE])) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");
					}
					if (mt_set_gpio_dir
					    (pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
					}
					if (mt_set_gpio_out
					    (pinSet[pinSetIdx][IDX_PS_CMRST],
					     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF])) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
					}
#else
					if (mt6306_set_gpio_dir
					    (pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
					}
					if (mt6306_set_gpio_out
					    (pinSet[pinSetIdx][IDX_PS_CMRST],
					     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF])) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
					}
#endif
				} else {
					if (mt_set_gpio_mode
					    (pinSet[pinSetIdx][IDX_PS_CMRST],
					     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_MODE])) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");
					}
					if (mt_set_gpio_dir
					    (pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
					}
					if (mt_set_gpio_out
					    (pinSet[pinSetIdx][IDX_PS_CMRST],
					     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF])) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
					}
				}
			}
			/* VCAM_IO */
			if (TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800*1000, mode_name)) {
				PK_DBG
				    ("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d\n",
				     CAMERA_POWER_VCAM_D2);
				goto _kdCISModulePowerOn_exit_;
			}
			/* VCAM_A */
			if (TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800*1000, mode_name)) {
				PK_DBG
				    ("[CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %d\n",
				     CAMERA_POWER_VCAM_A);
				goto _kdCISModulePowerOn_exit_;
			}

			if (TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1800*1000, mode_name)) {
				PK_DBG
				    ("[CAMERA SENSOR] Fail to enable digital power (VCAM_D), power id = %d\n",
				     CAMERA_POWER_VCAM_D);
				goto _kdCISModulePowerOn_exit_;
			}
			/* AF_VCC */
			if (TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800*1000, mode_name)) {
				PK_DBG
				    ("[CAMERA SENSOR] Fail to enable analog power (VCAM_AF), power id = %d\n",
				     CAMERA_POWER_VCAM_A2);
				goto _kdCISModulePowerOn_exit_;
			}

			mdelay(5);

			/* enable active sensor */
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
				if (mt_set_gpio_mode
				    (pinSet[pinSetIdx][IDX_PS_CMPDN],
				     pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_MODE])) {
					PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");
				}
				if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_DIR_OUT)) {
					PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");
				}
				if (mt_set_gpio_out
				    (pinSet[pinSetIdx][IDX_PS_CMPDN],
				     pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_ON])) {
					PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");
				}
			}

			mdelay(1);

			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
				if (0 == pinSetIdx) {
#ifndef CONFIG_MTK_MT6306_SUPPORT
					if (mt_set_gpio_mode
					    (pinSet[pinSetIdx][IDX_PS_CMRST],
					     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_MODE])) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");
					}
					if (mt_set_gpio_dir
					    (pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
					}
					if (mt_set_gpio_out
					    (pinSet[pinSetIdx][IDX_PS_CMRST],
					     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON])) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
					}
#else
					if (mt6306_set_gpio_dir
					    (pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
					}
					if (mt6306_set_gpio_out
					    (pinSet[pinSetIdx][IDX_PS_CMRST],
					     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON])) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
					}
#endif

				} else {
					if (mt_set_gpio_mode
					    (pinSet[pinSetIdx][IDX_PS_CMRST],
					     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_MODE])) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");
					}
					if (mt_set_gpio_dir
					    (pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
					}
					if (mt_set_gpio_out
					    (pinSet[pinSetIdx][IDX_PS_CMRST],
					     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON])) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
					}
				}
			}




		}
		/*
		   if(pinSetIdx==0)
		   for(;;)
		   {}
		 */
		/*
		   if(pinSetIdx==1)
		   for(;;)
		   {}
		 */
 #endif
	} else {		/* power OFF */
		for (pwListIdx = forge_cam_pwr_first(); pwListIdx < 24; pwListIdx++) {
			if (currSensorName && (PowerOnList.PowerSeq[pwListIdx].SensorName != NULL)
			    && (0 ==
				strcmp(PowerOnList.PowerSeq[pwListIdx].SensorName,
				       currSensorName))) {
				PK_DBG("kdCISModulePowerOn get in---\n");
				PK_DBG("sensorIdx:%d\n", SensorIdx);

				sensorInPowerList = KAL_TRUE;

				for (pwIdx = 15; pwIdx >= 0; pwIdx--) {
					if (PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].
					    PowerType != VDD_None) {
						if (hwpowerdown
						    (PowerOnList.PowerSeq[pwListIdx].
						     PowerInfo[pwIdx], mode_name) == FALSE)
							goto _kdCISModulePowerOn_exit_;
						if (pwIdx > 0) {
							if (PowerOnList.PowerSeq[pwListIdx].
							    PowerInfo[pwIdx - 1].Delay > 0)
								mdelay(PowerOnList.
								       PowerSeq[pwListIdx].
								       PowerInfo[pwIdx - 1].Delay);
						}
					} else {
						PK_DBG("pwIdx=%d\n", pwIdx);
					}
				}
			} else if (PowerOnList.PowerSeq[pwListIdx].SensorName == NULL) {
				break;
			} else {
			}
		}
#if 0
		/* Temp solution: default power on/off sequence */
		if (KAL_FALSE == sensorInPowerList) {
			PK_DBG("Default power off sequence");

			if (pinSetIdx == 0) {
				ISP_MCLK1_EN(0);
			} else if (pinSetIdx == 1) {
				ISP_MCLK2_EN(0);
			}
			/* Set Power Pin low and Reset Pin Low */
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
				if (mt_set_gpio_mode
				    (pinSet[pinSetIdx][IDX_PS_CMPDN],
				     pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_MODE])) {
					PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");
				}
				if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_DIR_OUT)) {
					PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");
				}
				if (mt_set_gpio_out
				    (pinSet[pinSetIdx][IDX_PS_CMPDN],
				     pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF])) {
					PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");
				}
			}


			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
				if (0 == pinSetIdx) {
#ifndef CONFIG_MTK_MT6306_SUPPORT
					if (mt_set_gpio_mode
					    (pinSet[pinSetIdx][IDX_PS_CMRST],
					     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_MODE])) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");
					}
					if (mt_set_gpio_dir
					    (pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
					}
					if (mt_set_gpio_out
					    (pinSet[pinSetIdx][IDX_PS_CMRST],
					     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF])) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
					}
#else
					if (mt6306_set_gpio_dir
					    (pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
					}
					if (mt6306_set_gpio_out
					    (pinSet[pinSetIdx][IDX_PS_CMRST],
					     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF])) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
					}
#endif
				} else {
					if (mt_set_gpio_mode
					    (pinSet[pinSetIdx][IDX_PS_CMRST],
					     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_MODE])) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");
					}
					if (mt_set_gpio_dir
					    (pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
					}
					if (mt_set_gpio_out
					    (pinSet[pinSetIdx][IDX_PS_CMRST],
					     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF])) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
					}
				}
			}


			if (TRUE != hwPowerDown(CAMERA_POWER_VCAM_D, mode_name)) {
				PK_DBG
				    ("[CAMERA SENSOR] Fail to OFF core power (VCAM_D), power id = %d\n",
				     CAMERA_POWER_VCAM_D);
				goto _kdCISModulePowerOn_exit_;
			}
			/* VCAM_A */
			if (TRUE != hwPowerDown(CAMERA_POWER_VCAM_A, mode_name)) {
				PK_DBG
				    ("[CAMERA SENSOR] Fail to OFF analog power (VCAM_A), power id= (%d)\n",
				     CAMERA_POWER_VCAM_A);
				/* return -EIO; */
				goto _kdCISModulePowerOn_exit_;
			}
			/* VCAM_IO */
			if (TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2, mode_name)) {
				PK_DBG
				    ("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d\n",
				     CAMERA_POWER_VCAM_D2);
				/* return -EIO; */
				goto _kdCISModulePowerOn_exit_;
			}
			/* AF_VCC */
			if (TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2, mode_name)) {
				PK_DBG
				    ("[CAMERA SENSOR] Fail to OFF AF power (VCAM_AF), power id = %d\n",
				     CAMERA_POWER_VCAM_A2);
				/* return -EIO; */
				goto _kdCISModulePowerOn_exit_;
			}







		}
#endif
	}			/*  */

	return 0;

_kdCISModulePowerOn_exit_:
	return -EIO;
}

#else
BOOL hwpoweron(PowerInformation pwInfo, char *mode_name)
{
	if (pwInfo.PowerType == AVDD) {
		if (PowerCustList.PowerCustInfo[0].Gpio_Pin == GPIO_UNSUPPORTED) {
			if (TRUE != hwPowerOn(pwInfo.PowerType, pwInfo.Voltage * 1000, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
				return FALSE;
			}
		} else {
			if (mt_set_gpio_mode
			    (PowerCustList.PowerCustInfo[0].Gpio_Pin,
			     PowerCustList.PowerCustInfo[0].Gpio_Mode)) {
				PK_DBG("[CAMERA SENSOR] set gpio mode failed!!\n");
			}
			if (mt_set_gpio_dir(PowerCustList.PowerCustInfo[0].Gpio_Pin, GPIO_DIR_OUT)) {
				PK_DBG("[CAMERA SENSOR] set gpio dir failed!!\n");
			}
			if (mt_set_gpio_out
			    (PowerCustList.PowerCustInfo[0].Gpio_Pin,
			     PowerCustList.PowerCustInfo[0].Voltage)) {
				PK_DBG("[CAMERA SENSOR] set gpio failed!!\n");
			}
		}
	} else if (pwInfo.PowerType == DVDD) {
		if (PowerCustList.PowerCustInfo[1].Gpio_Pin == GPIO_UNSUPPORTED) {
			if (pinSetIdx == 1) {
				/* PK_DBG("[CAMERA SENSOR] Sub camera VCAM_D power on"); */
				if (pwInfo.Voltage == Vol_1200) {
					pwInfo.Voltage = Vol_1220;
					PK_DBG("[CAMERA SENSOR] Sub camera VCAM_D power 1.2V to 1.22V\n");
				}
				if (TRUE !=
				    hwPowerOn(SUB_CAMERA_POWER_VCAM_D, pwInfo.Voltage * 1000,
					      mode_name)) {
					PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					return FALSE;
				}
			} else {
				/* PK_DBG("[CAMERA SENSOR] Main camera VCAM_D power on"); */
				if (TRUE !=
				    hwPowerOn(pwInfo.PowerType, pwInfo.Voltage * 1000, mode_name)) {
					PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					return FALSE;
				}
			}
		} else {
			if (mt_set_gpio_mode
			    (PowerCustList.PowerCustInfo[1].Gpio_Pin,
			     PowerCustList.PowerCustInfo[1].Gpio_Mode)) {
				PK_DBG("[CAMERA SENSOR] set gpio mode failed!!\n");
			}
			if (mt_set_gpio_dir(PowerCustList.PowerCustInfo[1].Gpio_Pin, GPIO_DIR_OUT)) {
				PK_DBG("[CAMERA SENSOR] set gpio dir failed!!\n");
			}
			if (mt_set_gpio_out
			    (PowerCustList.PowerCustInfo[1].Gpio_Pin,
			     PowerCustList.PowerCustInfo[1].Voltage)) {
				PK_DBG("[CAMERA SENSOR] set gpio failed!!\n");
			}
		}
	} else if (pwInfo.PowerType == DOVDD) {
		if (PowerCustList.PowerCustInfo[2].Gpio_Pin == GPIO_UNSUPPORTED) {
			if (TRUE != hwPowerOn(pwInfo.PowerType, pwInfo.Voltage * 1000, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
				return FALSE;
			}
		} else {
			if (mt_set_gpio_mode
			    (PowerCustList.PowerCustInfo[2].Gpio_Pin,
			     PowerCustList.PowerCustInfo[2].Gpio_Mode)) {
				PK_DBG("[CAMERA SENSOR] set gpio mode failed!!\n");
			}
			if (mt_set_gpio_dir(PowerCustList.PowerCustInfo[2].Gpio_Pin, GPIO_DIR_OUT)) {
				PK_DBG("[CAMERA SENSOR] set gpio dir failed!!\n");
			}
			if (mt_set_gpio_out
			    (PowerCustList.PowerCustInfo[2].Gpio_Pin,
			     PowerCustList.PowerCustInfo[2].Voltage)) {
				PK_DBG("[CAMERA SENSOR] set gpio failed!!\n");
			}
		}
	} else if (pwInfo.PowerType == AFVDD) {
#if 0
		PK_DBG("[CAMERA SENSOR] Skip AFVDD setting\n");
		if (PowerCustList.PowerCustInfo[3].Gpio_Pin == GPIO_UNSUPPORTED) {
			if (TRUE != hwPowerOn(pwInfo.PowerType, pwInfo.Voltage * 1000, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
				return FALSE;
			}
		} else {
			if (mt_set_gpio_mode
			    (PowerCustList.PowerCustInfo[3].Gpio_Pin,
			     PowerCustList.PowerCustInfo[3].Gpio_Mode)) {
				PK_DBG("[CAMERA SENSOR] set gpio mode failed!!\n");
			}
			if (mt_set_gpio_dir(PowerCustList.PowerCustInfo[3].Gpio_Pin, GPIO_DIR_OUT)) {
				PK_DBG("[CAMERA SENSOR] set gpio dir failed!!\n");
			}
			if (mt_set_gpio_out
			    (PowerCustList.PowerCustInfo[3].Gpio_Pin,
			     PowerCustList.PowerCustInfo[3].Voltage)) {
				PK_DBG("[CAMERA SENSOR] set gpio failed!!\n");
			}

			if (PowerCustList.PowerCustInfo[4].Gpio_Pin != GPIO_UNSUPPORTED) {
				mdelay(5);
				if (mt_set_gpio_mode
				    (PowerCustList.PowerCustInfo[3].Gpio_Pin,
				     PowerCustList.PowerCustInfo[3].Gpio_Mode)) {
					PK_DBG("[CAMERA SENSOR] set gpio mode failed!!\n");
				}
				if (mt_set_gpio_dir
				    (PowerCustList.PowerCustInfo[3].Gpio_Pin, GPIO_DIR_OUT)) {
					PK_DBG("[CAMERA SENSOR] set gpio dir failed!!\n");
				}
				if (mt_set_gpio_out
				    (PowerCustList.PowerCustInfo[3].Gpio_Pin,
				     PowerCustList.PowerCustInfo[3].Voltage)) {
					PK_DBG("[CAMERA SENSOR] set gpio failed!!\n");
				}
			}
		}
#endif
	} else if (pwInfo.PowerType == PDN) {
		/* PK_DBG("hwPowerOn: PDN %d\n", pwInfo.Voltage); */

		if (mt_set_gpio_mode
		    (pinSet[pinSetIdx][IDX_PS_CMPDN],
		     pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_MODE])) {
			PK_DBG("[CAMERA SENSOR] set gpio mode failed!!\n");
		}
		if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_DIR_OUT)) {
			PK_DBG("[CAMERA SENSOR] set gpio dir failed!!\n");
		}
		if (pwInfo.Voltage == Vol_High) {
			if (mt_set_gpio_out
			    (pinSet[pinSetIdx][IDX_PS_CMPDN],
			     pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_ON])) {
				PK_DBG("[CAMERA SENSOR] set gpio failed!!\n");
			}
		} else {
			if (mt_set_gpio_out
			    (pinSet[pinSetIdx][IDX_PS_CMPDN],
			     pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF])) {
				PK_DBG("[CAMERA SENSOR] set gpio failed!!\n");
			}
		}
	} else if (pwInfo.PowerType == RST) {
		/* PK_DBG("hwPowerOn: RST %d\n", pwInfo.Voltage); */

		if (pinSetIdx == 0) {
#ifndef CONFIG_MTK_MT6306_SUPPORT
			if (mt_set_gpio_mode
			    (pinSet[pinSetIdx][IDX_PS_CMRST],
			     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_MODE])) {
				PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");
			}
			if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)) {
				PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
			}
			if (mt_set_gpio_out
			    (pinSet[pinSetIdx][IDX_PS_CMRST],
			     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF])) {
				PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
			}
			if (pwInfo.Voltage == Vol_High) {
				if (mt_set_gpio_out
				    (pinSet[pinSetIdx][IDX_PS_CMRST],
				     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON])) {
					PK_DBG("[CAMERA LENS] set gpio failed!!\n");
				}
			} else {
				if (mt_set_gpio_out
				    (pinSet[pinSetIdx][IDX_PS_CMRST],
				     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF])) {
					PK_DBG("[CAMERA LENS] set gpio failed!!\n");
				}
			}
#else
			if (mt6306_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)) {
				PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
			}
			if (pwInfo.Voltage == Vol_High) {
				if (mt6306_set_gpio_out
				    (pinSet[pinSetIdx][IDX_PS_CMRST],
				     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON])) {
					PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
				}
			} else {
				if (mt6306_set_gpio_out
				    (pinSet[pinSetIdx][IDX_PS_CMRST],
				     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF])) {
					PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
				}
			}
#endif
		} else if (pinSetIdx == 1) {
			if (mt_set_gpio_mode
			    (pinSet[pinSetIdx][IDX_PS_CMRST],
			     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_MODE])) {
				PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");
			}
			if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)) {
				PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
			}
			if (mt_set_gpio_out
			    (pinSet[pinSetIdx][IDX_PS_CMRST],
			     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF])) {
				PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
			}
			if (pwInfo.Voltage == Vol_High) {
				if (mt_set_gpio_out
				    (pinSet[pinSetIdx][IDX_PS_CMRST],
				     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON])) {
					PK_DBG("[CAMERA SENSOR] set gpio failed!!\n");
				}
			} else {
				if (mt_set_gpio_out
				    (pinSet[pinSetIdx][IDX_PS_CMRST],
				     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF])) {
					PK_DBG("[CAMERA SENSOR] set gpio failed!!\n");
				}
			}
		}


	} else if (pwInfo.PowerType == AF_EN) {//wangjie, af en pin
		/* AF motor enable = GPIO21 via the pinctrl states
		 * cam0_af_en0/1, as in the stock DTB. */
		if (pwInfo.Voltage == Vol_High && !IS_ERR_OR_NULL(cam0_af_en_h))
			pinctrl_select_state(camctrl, cam0_af_en_h);
		else if (pwInfo.Voltage != Vol_High && !IS_ERR_OR_NULL(cam0_af_en_l))
			pinctrl_select_state(camctrl, cam0_af_en_l);
		else
			pr_err("[FORGE_CAM] AF_EN(%d): no cam0_af_en pinctrl state\n",
			       pwInfo.Voltage);
	} else if (pwInfo.PowerType == SensorMCLK) {
		/* Mux the CMMCLK pad, then enable the SENINF MCLK output
		 * (mt_cam_mclk.c). Both failure paths are logged. */
		if (pinSetIdx == 0) {
			/* PK_DBG("Sensor MCLK1 On"); */
			if (!IS_ERR_OR_NULL(cam0_mclk_on))
				pinctrl_select_state(camctrl, cam0_mclk_on);
			else
				pr_err("[FORGE_CAM] MCLK1: no cam0_mclk_on pinctrl state - CMMCLK0 pad (GPIO161) mux is UNDEFINED, a sensor NAK here is not identity evidence\n");
			forge_cam_l681_mclk_pad(161);
			ISP_MCLK1_EN(TRUE);
		} else if (pinSetIdx == 1) {
			/* PK_DBG("Sensor MCLK2 On"); */
			if (!IS_ERR_OR_NULL(cam1_mclk_on))
				pinctrl_select_state(camctrl, cam1_mclk_on);
			else
				pr_err("[FORGE_CAM] MCLK2: no cam1_mclk_on pinctrl state - CMMCLK1 pad (GPIO109) mux is UNDEFINED, a sensor NAK here is not identity evidence\n");
			forge_cam_l681_mclk_pad(109);
			ISP_MCLK2_EN(TRUE);
		}
	} else {
	}
	if (pwInfo.Delay > 0)
		mdelay(pwInfo.Delay);
	return TRUE;
}



BOOL hwpowerdown(PowerInformation pwInfo, char *mode_name)
{
	if (pwInfo.PowerType == AVDD) {
		if (PowerCustList.PowerCustInfo[0].Gpio_Pin == GPIO_UNSUPPORTED) {
			if (TRUE != hwPowerDown(pwInfo.PowerType, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
				return FALSE;
			}
		} else {
			if (mt_set_gpio_mode
			    (PowerCustList.PowerCustInfo[0].Gpio_Pin,
			     PowerCustList.PowerCustInfo[0].Gpio_Mode)) {
				PK_DBG("[CAMERA LENS] set gpio mode failed!!\n");
			}
			if (mt_set_gpio_dir(PowerCustList.PowerCustInfo[0].Gpio_Pin, GPIO_DIR_OUT)) {
				PK_DBG("[CAMERA LENS] set gpio dir failed!!\n");
			}
			if (mt_set_gpio_out
			    (PowerCustList.PowerCustInfo[0].Gpio_Pin,
			     PowerCustList.PowerCustInfo[0].Voltage)) {
				PK_DBG("[CAMERA LENS] set gpio failed!!\n");
			}
		}
	} else if (pwInfo.PowerType == DVDD) {
		if (PowerCustList.PowerCustInfo[1].Gpio_Pin == GPIO_UNSUPPORTED) {
			if (pinSetIdx == 1) {
				if (TRUE != hwPowerDown(PMIC_APP_SUB_CAMERA_POWER_D, mode_name)) {
					PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					return FALSE;
				}
			} else if (TRUE != hwPowerDown(pwInfo.PowerType, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
				return FALSE;
			} else {
			}
		} else {
			if (mt_set_gpio_mode
			    (PowerCustList.PowerCustInfo[1].Gpio_Pin,
			     PowerCustList.PowerCustInfo[1].Gpio_Mode)) {
				PK_DBG("[CAMERA LENS] set gpio mode failed!!\n");
			}
			if (mt_set_gpio_dir(PowerCustList.PowerCustInfo[1].Gpio_Pin, GPIO_DIR_OUT)) {
				PK_DBG("[CAMERA LENS] set gpio dir failed!!\n");
			}
			if (mt_set_gpio_out
			    (PowerCustList.PowerCustInfo[1].Gpio_Pin,
			     PowerCustList.PowerCustInfo[1].Voltage)) {
				PK_DBG("[CAMERA LENS] set gpio failed!!\n");
			}
		}
	} else if (pwInfo.PowerType == DOVDD) {
		if (PowerCustList.PowerCustInfo[2].Gpio_Pin == GPIO_UNSUPPORTED) {
			if (TRUE != hwPowerDown(pwInfo.PowerType, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
				return FALSE;
			}
		} else {
			if (mt_set_gpio_mode
			    (PowerCustList.PowerCustInfo[2].Gpio_Pin,
			     PowerCustList.PowerCustInfo[2].Gpio_Mode)) {
				PK_DBG("[CAMERA LENS] set gpio mode failed!!\n");
			}
			if (mt_set_gpio_dir(PowerCustList.PowerCustInfo[2].Gpio_Pin, GPIO_DIR_OUT)) {
				PK_DBG("[CAMERA LENS] set gpio dir failed!!\n");
			}
			if (mt_set_gpio_out
			    (PowerCustList.PowerCustInfo[2].Gpio_Pin,
			     PowerCustList.PowerCustInfo[2].Voltage)) {
				PK_DBG("[CAMERA LENS] set gpio failed!!\n");
			}
		}
	} else if (pwInfo.PowerType == AFVDD) {
#if 0
		PK_DBG("[CAMERA SENSOR] Skip AFVDD setting\n");
		if (PowerCustList.PowerCustInfo[3].Gpio_Pin == GPIO_UNSUPPORTED) {
			if (TRUE != hwPowerDown(pwInfo.PowerType, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
				return FALSE;
			}
		} else {
			if (mt_set_gpio_mode
			    (PowerCustList.PowerCustInfo[3].Gpio_Pin,
			     PowerCustList.PowerCustInfo[3].Gpio_Mode)) {
				PK_DBG("[CAMERA LENS] set gpio mode failed!!\n");
			}
			if (mt_set_gpio_dir(PowerCustList.PowerCustInfo[3].Gpio_Pin, GPIO_DIR_OUT)) {
				PK_DBG("[CAMERA LENS] set gpio dir failed!!\n");
			}
			if (mt_set_gpio_out
			    (PowerCustList.PowerCustInfo[3].Gpio_Pin,
			     PowerCustList.PowerCustInfo[3].Voltage)) {
				PK_DBG("[CAMERA LENS] set gpio failed!!\n");
			}

			if (PowerCustList.PowerCustInfo[4].Gpio_Pin != GPIO_UNSUPPORTED) {
				mdelay(5);
				if (mt_set_gpio_mode
				    (PowerCustList.PowerCustInfo[3].Gpio_Pin,
				     PowerCustList.PowerCustInfo[3].Gpio_Mode)) {
					PK_DBG("[CAMERA LENS] set gpio mode failed!!\n");
				}
				if (mt_set_gpio_dir
				    (PowerCustList.PowerCustInfo[3].Gpio_Pin, GPIO_DIR_OUT)) {
					PK_DBG("[CAMERA LENS] set gpio dir failed!!\n");
				}
				if (mt_set_gpio_out
				    (PowerCustList.PowerCustInfo[3].Gpio_Pin,
				     PowerCustList.PowerCustInfo[3].Voltage)) {
					PK_DBG("[CAMERA LENS] set gpio failed!!\n");
				}
			}
		}
#endif
	} else if (pwInfo.PowerType == PDN) {
		PK_DBG("hwPowerDown: PDN %d\n", pwInfo.Voltage);

		if (mt_set_gpio_mode
		    (pinSet[pinSetIdx][IDX_PS_CMPDN],
		     pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_MODE])) {
			PK_DBG("[CAMERA LENS] set gpio mode failed!!\n");
		}
		if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_DIR_OUT)) {
			PK_DBG("[CAMERA LENS] set gpio dir failed!!\n");
		}
		if (pwInfo.Voltage == Vol_High) {
			if (mt_set_gpio_out
			    (pinSet[pinSetIdx][IDX_PS_CMPDN],
			     pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_ON])) {
				PK_DBG("[CAMERA LENS] set gpio failed!!\n");
			}
			msleep(1);
		} else {
			if (mt_set_gpio_out
			    (pinSet[pinSetIdx][IDX_PS_CMPDN],
			     pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF])) {
				PK_DBG("[CAMERA LENS] set gpio failed!!\n");
			}
			msleep(1);
		}
	} else if (pwInfo.PowerType == RST) {
		PK_DBG("hwPowerDown: RST %d\n", pwInfo.Voltage);
		if (pinSetIdx == 0) {
#ifndef CONFIG_MTK_MT6306_SUPPORT
			if (mt_set_gpio_mode
			    (pinSet[pinSetIdx][IDX_PS_CMRST],
			     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_MODE])) {
				PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");
			}
			if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)) {
				PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
			}
			if (mt_set_gpio_out
			    (pinSet[pinSetIdx][IDX_PS_CMRST],
			     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF])) {
				PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
			}
			if (pwInfo.Voltage == Vol_High) {
				if (mt_set_gpio_out
				    (pinSet[pinSetIdx][IDX_PS_CMRST],
				     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON])) {
					PK_DBG("[CAMERA LENS] set gpio failed!!\n");
				}
			} else {
				if (mt_set_gpio_out
				    (pinSet[pinSetIdx][IDX_PS_CMRST],
				     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF])) {
					PK_DBG("[CAMERA LENS] set gpio failed!!\n");
				}
			}
#else
			if (mt6306_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)) {
				PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
			}
			if (pwInfo.Voltage == Vol_High) {
				if (mt6306_set_gpio_out
				    (pinSet[pinSetIdx][IDX_PS_CMRST],
				     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON])) {
					PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
				}
			} else {
				if (mt6306_set_gpio_out
				    (pinSet[pinSetIdx][IDX_PS_CMRST],
				     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF])) {
					PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
				}
			}
#endif
		} else if (pinSetIdx == 1) {
			if (mt_set_gpio_mode
			    (pinSet[pinSetIdx][IDX_PS_CMRST],
			     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_MODE])) {
				PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");
			}
			if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)) {
				PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
			}
			if (mt_set_gpio_out
			    (pinSet[pinSetIdx][IDX_PS_CMRST],
			     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF])) {
				PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
			}
			if (pwInfo.Voltage == Vol_High) {
				if (mt_set_gpio_out
				    (pinSet[pinSetIdx][IDX_PS_CMRST],
				     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON])) {
					PK_DBG("[CAMERA LENS] set gpio failed!!\n");
				}
			} else {
				if (mt_set_gpio_out
				    (pinSet[pinSetIdx][IDX_PS_CMRST],
				     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF])) {
					PK_DBG("[CAMERA LENS] set gpio failed!!\n");
				}
			}
		}

	} else if (pwInfo.PowerType == AF_EN) {//wangjie, af en pin
		/* AF motor enable off via pinctrl (see hwpoweron). */
		if (!IS_ERR_OR_NULL(cam0_af_en_l))
			pinctrl_select_state(camctrl, cam0_af_en_l);
		else
			pr_err("[FORGE_CAM] AF_EN off: no cam0_af_en0 pinctrl state\n");
	} else if (pwInfo.PowerType == SensorMCLK) {
		/* Stop the SENINF output first, then park the pad. */
		if (pinSetIdx == 0) {
			ISP_MCLK1_EN(FALSE);
			if (!IS_ERR_OR_NULL(cam0_mclk_off))
				pinctrl_select_state(camctrl, cam0_mclk_off);
		} else if (pinSetIdx == 1) {
			ISP_MCLK2_EN(FALSE);
			if (!IS_ERR_OR_NULL(cam1_mclk_off))
				pinctrl_select_state(camctrl, cam1_mclk_off);
		}
	} else {
	}
	return TRUE;
}




int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, BOOL On,
		       char *mode_name)
{

	int pwListIdx, pwIdx;
	BOOL sensorInPowerList = KAL_FALSE;

	/* power gate: see forge_cam_power above */
	if (!forge_cam_power) {
		pr_err("[FORGE_CAM] kdCISModulePowerOn(%s, on=%d) GATED OFF (forge_cam_power=0)\n",
		       currSensorName ? currSensorName : "?", On);
		return -EPERM;
	}

	if (DUAL_CAMERA_MAIN_SENSOR == SensorIdx) {
		pinSetIdx = 0;
	} else if (DUAL_CAMERA_SUB_SENSOR == SensorIdx) {
		pinSetIdx = 1;
	} else if (DUAL_CAMERA_MAIN_2_SENSOR == SensorIdx) {
		pinSetIdx = 2;
	}
	/* power ON */
	if (On) {
		PK_DBG("kdCISModulePowerOn -on:currSensorName=%s\n", currSensorName);
		PK_DBG("kdCISModulePowerOn -on:pinSetIdx=%d\n", pinSetIdx);

		for (pwListIdx = forge_cam_pwr_first(); pwListIdx < 24; pwListIdx++) {
			if (currSensorName && (PowerOnList.PowerSeq[pwListIdx].SensorName != NULL)
			    && (0 ==
				strcmp(PowerOnList.PowerSeq[pwListIdx].SensorName,
				       currSensorName))) {
				PK_DBG("sensorIdx:%d\n", SensorIdx);

				sensorInPowerList = KAL_TRUE;

				for (pwIdx = 0; pwIdx < 16; pwIdx++) {
					if (PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].
					    PowerType != VDD_None) {
						if (hwpoweron
						    (PowerOnList.PowerSeq[pwListIdx].
						     PowerInfo[pwIdx], mode_name) == FALSE)
							goto _kdCISModulePowerOn_exit_;
					} else {
						/* PK_DBG("pwIdx=%d\n", pwIdx); */
						break;
					}
				}
				break;
			} else if (PowerOnList.PowerSeq[pwListIdx].SensorName == NULL) {
				break;
			} else {
			}
		}

		/* Temp solution: default power on/off sequence */
		if (KAL_FALSE == sensorInPowerList) {
			PK_DBG("Default power on sequence");

			if (pinSetIdx == 0) {
				ISP_MCLK1_EN(1);
			} else if (pinSetIdx == 1) {
				ISP_MCLK2_EN(1);
			}
			/* First Power Pin low and Reset Pin Low */
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
				if (mt_set_gpio_mode
				    (pinSet[pinSetIdx][IDX_PS_CMPDN],
				     pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_MODE])) {
					PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");
				}
				if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_DIR_OUT)) {
					PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");
				}
				if (mt_set_gpio_out
				    (pinSet[pinSetIdx][IDX_PS_CMPDN],
				     pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF])) {
					PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");
				}
			}

			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
				if (0 == pinSetIdx) {
#ifndef CONFIG_MTK_MT6306_SUPPORT
					if (mt_set_gpio_mode
					    (pinSet[pinSetIdx][IDX_PS_CMRST],
					     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_MODE])) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");
					}
					if (mt_set_gpio_dir
					    (pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
					}
					if (mt_set_gpio_out
					    (pinSet[pinSetIdx][IDX_PS_CMRST],
					     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF])) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
					}
#else
					if (mt6306_set_gpio_dir
					    (pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
					}
					if (mt6306_set_gpio_out
					    (pinSet[pinSetIdx][IDX_PS_CMRST],
					     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF])) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
					}
#endif
				} else {
					if (mt_set_gpio_mode
					    (pinSet[pinSetIdx][IDX_PS_CMRST],
					     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_MODE])) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");
					}
					if (mt_set_gpio_dir
					    (pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
					}
					if (mt_set_gpio_out
					    (pinSet[pinSetIdx][IDX_PS_CMRST],
					     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF])) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
					}
				}
			}
			/* VCAM_IO */
			if (TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800*1000, mode_name)) {
				PK_DBG
				    ("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d\n",
				     CAMERA_POWER_VCAM_D2);
				goto _kdCISModulePowerOn_exit_;
			}
			/* VCAM_A */
			if (TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800*1000, mode_name)) {
				PK_DBG
				    ("[CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %d\n",
				     CAMERA_POWER_VCAM_A);
				goto _kdCISModulePowerOn_exit_;
			}

			if (TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1800*1000, mode_name)) {
				PK_DBG
				    ("[CAMERA SENSOR] Fail to enable digital power (VCAM_D), power id = %d\n",
				     CAMERA_POWER_VCAM_D);
				goto _kdCISModulePowerOn_exit_;
			}
			/* AF_VCC */
			if (TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800*1000, mode_name)) {
				PK_DBG
				    ("[CAMERA SENSOR] Fail to enable analog power (VCAM_AF), power id = %d\n",
				     CAMERA_POWER_VCAM_A2);
				goto _kdCISModulePowerOn_exit_;
			}

			mdelay(5);

			/* enable active sensor */
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
				if (mt_set_gpio_mode
				    (pinSet[pinSetIdx][IDX_PS_CMPDN],
				     pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_MODE])) {
					PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");
				}
				if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_DIR_OUT)) {
					PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");
				}
				if (mt_set_gpio_out
				    (pinSet[pinSetIdx][IDX_PS_CMPDN],
				     pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_ON])) {
					PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");
				}
			}

			mdelay(1);

			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
				if (0 == pinSetIdx) {
#ifndef CONFIG_MTK_MT6306_SUPPORT
					if (mt_set_gpio_mode
					    (pinSet[pinSetIdx][IDX_PS_CMRST],
					     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_MODE])) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");
					}
					if (mt_set_gpio_dir
					    (pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
					}
					if (mt_set_gpio_out
					    (pinSet[pinSetIdx][IDX_PS_CMRST],
					     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON])) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
					}
#else
					if (mt6306_set_gpio_dir
					    (pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
					}
					if (mt6306_set_gpio_out
					    (pinSet[pinSetIdx][IDX_PS_CMRST],
					     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON])) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
					}
#endif

				} else {
					if (mt_set_gpio_mode
					    (pinSet[pinSetIdx][IDX_PS_CMRST],
					     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_MODE])) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");
					}
					if (mt_set_gpio_dir
					    (pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
					}
					if (mt_set_gpio_out
					    (pinSet[pinSetIdx][IDX_PS_CMRST],
					     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON])) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
					}
				}
			}




		}
		/*
		   if(pinSetIdx==0)
		   for(;;)
		   {}
		 */
		/*
		   if(pinSetIdx==1)
		   for(;;)
		   {}
		 */
	} else {		/* power OFF */
		for (pwListIdx = forge_cam_pwr_first(); pwListIdx < 24; pwListIdx++) {
			if (currSensorName && (PowerOnList.PowerSeq[pwListIdx].SensorName != NULL)
			    && (0 ==
				strcmp(PowerOnList.PowerSeq[pwListIdx].SensorName,
				       currSensorName))) {
				PK_DBG("kdCISModulePowerOn get in---\n");
				PK_DBG("sensorIdx:%d\n", SensorIdx);

				sensorInPowerList = KAL_TRUE;

				for (pwIdx = 15; pwIdx >= 0; pwIdx--) {
					if (PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].
					    PowerType != VDD_None) {
						if (hwpowerdown
						    (PowerOnList.PowerSeq[pwListIdx].
						     PowerInfo[pwIdx], mode_name) == FALSE)
							goto _kdCISModulePowerOn_exit_;
						if (pwIdx > 0) {
							if (PowerOnList.PowerSeq[pwListIdx].
							    PowerInfo[pwIdx - 1].Delay > 0)
								mdelay(PowerOnList.
								       PowerSeq[pwListIdx].
								       PowerInfo[pwIdx - 1].Delay);
						}
					} else {
						PK_DBG("pwIdx=%d\n", pwIdx);
					}
				}
			} else if (PowerOnList.PowerSeq[pwListIdx].SensorName == NULL) {
				break;
			} else {
			}
		}

		/* Temp solution: default power on/off sequence */
		if (KAL_FALSE == sensorInPowerList) {
			PK_DBG("Default power off sequence");

			if (pinSetIdx == 0) {
				ISP_MCLK1_EN(0);
			} else if (pinSetIdx == 1) {
				ISP_MCLK2_EN(0);
			}
			/* Set Power Pin low and Reset Pin Low */
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
				if (mt_set_gpio_mode
				    (pinSet[pinSetIdx][IDX_PS_CMPDN],
				     pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_MODE])) {
					PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");
				}
				if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_DIR_OUT)) {
					PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");
				}
				if (mt_set_gpio_out
				    (pinSet[pinSetIdx][IDX_PS_CMPDN],
				     pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF])) {
					PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");
				}
			}


			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
				if (0 == pinSetIdx) {
#ifndef CONFIG_MTK_MT6306_SUPPORT
					if (mt_set_gpio_mode
					    (pinSet[pinSetIdx][IDX_PS_CMRST],
					     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_MODE])) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");
					}
					if (mt_set_gpio_dir
					    (pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
					}
					if (mt_set_gpio_out
					    (pinSet[pinSetIdx][IDX_PS_CMRST],
					     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF])) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
					}
#else
					if (mt6306_set_gpio_dir
					    (pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
					}
					if (mt6306_set_gpio_out
					    (pinSet[pinSetIdx][IDX_PS_CMRST],
					     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF])) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
					}
#endif
				} else {
					if (mt_set_gpio_mode
					    (pinSet[pinSetIdx][IDX_PS_CMRST],
					     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_MODE])) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");
					}
					if (mt_set_gpio_dir
					    (pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
					}
					if (mt_set_gpio_out
					    (pinSet[pinSetIdx][IDX_PS_CMRST],
					     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF])) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
					}
				}
			}


			if (TRUE != hwPowerDown(CAMERA_POWER_VCAM_D, mode_name)) {
				PK_DBG
				    ("[CAMERA SENSOR] Fail to OFF core power (VCAM_D), power id = %d\n",
				     CAMERA_POWER_VCAM_D);
				goto _kdCISModulePowerOn_exit_;
			}
			/* VCAM_A */
			if (TRUE != hwPowerDown(CAMERA_POWER_VCAM_A, mode_name)) {
				PK_DBG
				    ("[CAMERA SENSOR] Fail to OFF analog power (VCAM_A), power id= (%d)\n",
				     CAMERA_POWER_VCAM_A);
				/* return -EIO; */
				goto _kdCISModulePowerOn_exit_;
			}
			/* VCAM_IO */
			if (TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2, mode_name)) {
				PK_DBG
				    ("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d\n",
				     CAMERA_POWER_VCAM_D2);
				/* return -EIO; */
				goto _kdCISModulePowerOn_exit_;
			}
			/* AF_VCC */
			if (TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2, mode_name)) {
				PK_DBG
				    ("[CAMERA SENSOR] Fail to OFF AF power (VCAM_AF), power id = %d\n",
				     CAMERA_POWER_VCAM_A2);
				/* return -EIO; */
				goto _kdCISModulePowerOn_exit_;
			}







		}
	}			/*  */

	return 0;

_kdCISModulePowerOn_exit_:
	return -EIO;
}
#endif
EXPORT_SYMBOL(kdCISModulePowerOn);


/* !-- */
/*  */
