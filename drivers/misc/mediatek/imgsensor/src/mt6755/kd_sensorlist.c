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

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>   /* proc file use */
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <sync_write.h>
#include <linux/types.h>
#include "kd_camera_hw.h"
#include "kd_camera_typedef.h"
#ifdef CONFIG_MTK_CLKMGR
#include <mach/mt_clkmgr.h>
#else
#include <linux/clk.h>
#endif
#ifdef CONFIG_MTK_PMIC_CHIP_MT6353
#include <mt-plat/upmu_common.h>
#endif


#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_feature.h"
#include "kd_imgsensor_errcode.h"

#include "kd_sensorlist.h"
#ifdef CONFIG_HUAWEI_DSM
#include "mtk_camera_dsm.h"
#endif

#ifdef CONFIG_MTK_SMI_EXT
#include "mmdvfs_mgr.h"
#endif
#ifdef CONFIG_OF
/* device tree */
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/m3note_board.h>
#include <linux/of_irq.h>
#endif
/* #define CONFIG_COMPAT */
#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif

/*jijin.wang add camera otp check*/
static struct kobject *mtk_eeprom_device = NULL;
static char back_otp_checksum_info[20] = {0};
static char front_otp_checksum_info[20] = {0};

#ifndef CONFIG_MTK_CLKMGR
/*CCF*/
struct clk *g_camclk_camtg_sel;
struct clk *g_camclk_scam_sel;
struct clk *g_camclk_univpll_d26;
struct clk *g_camclk_univpll2_d2;
/* 26 MHz root, so the camtg parent can be moved to the frequency both
 * fitted sensors declare (.mclk = 24, "suggest 24 or 26"). Provided by the
 * board DTS override, not by the base node. */
struct clk *g_camclk_clk26m;
#endif
/* kernel standard for PMIC*/
#if !defined(CONFIG_MTK_LEGACY)
/* PMIC */
#include <linux/regulator/consumer.h>
#endif

/* Module-info strings the sensor drivers fill in (ov13853*, s5k5e8yx,
 * ov5670: "SUNNY_OV13853", "JSL_S5K5E8"...). Stock keeps them in the Meizu
 * drivers/misc/dev_info.c (/dev/dev_info), which is not carried here. */
char camera_f_info[20] = {"NO THIS DEVICE"};
char camera_b_info[20] = {"NO THIS DEVICE"};

#define PROC_CAMERA_INFO "driver/camera_info"
#define camera_info_size 128
#define PDAF_DATA_SIZE 4096
char mtk_ccm_name[camera_info_size] = {0};
static unsigned int gDrvIndex = 0;
#ifdef CONFIG_MTK_SMI_EXT
static int current_mmsys_clk = MMSYS_CLK_MEDIUM;
#endif

static DEFINE_SPINLOCK(kdsensor_drv_lock);

/* Move these defines to kd_camera_hw.h, so they can be project-dependent //Jessy @2014/06/04
#define SUPPORT_I2C_BUS_NUM1        0
#define SUPPORT_I2C_BUS_NUM2        2
*/

/* The following is to avoid build error of project: mt6755_fpga related //Jessy @2014/06/04 */
#ifndef SUPPORT_I2C_BUS_NUM1
    #define SUPPORT_I2C_BUS_NUM1        0
#endif
#ifndef SUPPORT_I2C_BUS_NUM2
    #define SUPPORT_I2C_BUS_NUM2        2
#endif

//Main2 support
#ifndef SUPPORT_I2C_BUS_NUM3
    #define SUPPORT_I2C_BUS_NUM3        SUPPORT_I2C_BUS_NUM2
#endif

#define CAMERA_HW_DRVNAME1  "kd_camera_hw"
#define CAMERA_HW_DRVNAME2  "kd_camera_hw_bus2"

#if defined(CONFIG_MTK_LEGACY)
static struct i2c_board_info i2c_devs1 __initdata = {I2C_BOARD_INFO(CAMERA_HW_DRVNAME1, 0xfe>>1)};
static struct i2c_board_info i2c_devs2 __initdata = {I2C_BOARD_INFO(CAMERA_HW_DRVNAME2, 0xfe>>1)};
#endif

#if !defined(CONFIG_MTK_LEGACY)
    /*PMIC*/
	struct regulator *regVCAMA = NULL;
	struct regulator *regVCAMD = NULL;
	struct regulator *regVCAMIO = NULL;
	struct regulator *regVCAMAF = NULL;
	struct regulator *regSubVCAMD = NULL;
	struct regulator *regMain2VCAMD = NULL;
#endif

#define FEATURE_CONTROL_MAX_DATA_SIZE 128000

struct device *sensor_device = NULL;
#define SENSOR_WR32(addr, data)    mt65xx_reg_sync_writel(data, addr)    /* For 89 Only.   // NEED_TUNING_BY_PROJECT */
/* #define SENSOR_WR32(addr, data)    iowrite32(data, addr)    // For 89 Only.   // NEED_TUNING_BY_PROJECT */
#define SENSOR_RD32(addr)          ioread32(addr)
/******************************************************************************
 * Debug configuration
******************************************************************************/
#define PFX "[kd_sensorlist]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(fmt, ##arg)
#define PK_INF(fmt, args...)     pr_debug(PFX "[%s] " fmt, __FUNCTION__, ##args)

#undef DEBUG_CAMERA_HW_K
/* #define DEBUG_CAMERA_HW_K */
#ifdef DEBUG_CAMERA_HW_K
#define PK_DBG PK_DBG_FUNC
#define PK_ERR(fmt, arg...)         pr_err(fmt, ##arg)
#define PK_XLOG_INFO(fmt, args...) \
        do {    \
            pr_debug(fmt, ##args); \
        } while (0)
#else
#define PK_DBG(a, ...)
#define PK_ERR(fmt, arg...)             pr_err(fmt, ##arg)
#define PK_XLOG_INFO(fmt, args...)

#endif

#define M6_CAM_DIAG(fmt, arg...) \
	pr_warn_ratelimited("[M6_PERIPH_DIAG][CAM] " fmt, ##arg)

static const char *m6_cam_power_type_name(PowerType type)
{
	switch (type) {
	case AVDD:
		return "AVDD";
	case DVDD:
		return "DVDD";
	case DOVDD:
		return "DOVDD";
	case AFVDD:
		return "AFVDD";
	case SUB_DVDD:
		return "SUB_DVDD";
	case MAIN2_DVDD:
		return "MAIN2_DVDD";
	default:
		return "UNKNOWN";
	}
}

/*******************************************************************************
* Proifling
********************************************************************************/
#define PROFILE 1
#if PROFILE
static struct timeval tv1, tv2;
/*******************************************************************************
*
********************************************************************************/
inline void KD_IMGSENSOR_PROFILE_INIT(void)
{
    do_gettimeofday(&tv1);
}

/*******************************************************************************
*
********************************************************************************/
inline void KD_IMGSENSOR_PROFILE(char *tag)
{
    unsigned long TimeIntervalUS;

    spin_lock(&kdsensor_drv_lock);

    do_gettimeofday(&tv2);
    TimeIntervalUS = (tv2.tv_sec - tv1.tv_sec) * 1000000 + (tv2.tv_usec - tv1.tv_usec);
    tv1 = tv2;

    spin_unlock(&kdsensor_drv_lock);
    PK_DBG("[%s]Profile = %lu\n", tag, TimeIntervalUS);
}
#else
inline static void KD_IMGSENSOR_PROFILE_INIT(void) {}
inline static void KD_IMGSENSOR_PROFILE(char *tag) {}
#endif

/*******************************************************************************
*
********************************************************************************/
extern int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, BOOL On, char *mode_name);
/* extern ssize_t strobe_VDIrq(void);  //cotta : add for high current solution */

/*******************************************************************************
*
********************************************************************************/
#ifndef CONFIG_OF
static struct platform_device camerahw_platform_device = {
    .name = "image_sensor",
    .id = 0,
    .dev = {
    .coherent_dma_mask = DMA_BIT_MASK(32),
    }
};
#endif
struct platform_device* pCamerahw_platform_device;
/* forge_cam_power gate accessor (camera_hw/kd_camera_hw.c). Declared
 * early so the i2c probes below can defer node creation until armed. */
extern int forge_cam_power_enabled(void);
static struct i2c_client *g_pstI2Cclient;
static struct i2c_client *g_pstI2Cclient2;

/* 81 is used for V4L driver */
static dev_t g_CAMERA_HWdevno = MKDEV(250, 0);
static dev_t g_CAMERA_HWdevno2;
static struct cdev *g_pCAMERA_HW_CharDrv;
static struct cdev *g_pCAMERA_HW_CharDrv2;
static struct class *sensor_class;
static struct class *sensor2_class;

static atomic_t g_CamHWOpend;
static atomic_t g_CamHWOpend2;
static atomic_t g_CamHWOpening;
static atomic_t g_CamDrvOpenCnt;
static atomic_t g_CamDrvOpenCnt2;

/* static u32 gCurrI2CBusEnableFlag = 0; */
static u32 gI2CBusNum = SUPPORT_I2C_BUS_NUM1;

/* If the camera_main i2c node never probed, g_pstI2Cclient stays NULL
 * and the first sensor-ID read would oops in iReadRegI2C. Every accessor
 * below checks this first: a bus without a client answers -ENODEV, like a
 * sensor that does not ACK. */
static bool kd_i2c_client_missing(void)
{
	struct i2c_client *client = gI2CBusNum == SUPPORT_I2C_BUS_NUM1 ?
				    g_pstI2Cclient : g_pstI2Cclient2;

	if (client)
		return false;
	pr_err_ratelimited("[FORGE_CAM] i2c bus %u: no camera i2c client probed, access refused\n",
			   gI2CBusNum);
	return true;
}

#define SET_I2CBUS_FLAG(_x_)        ((1<<_x_)|(gCurrI2CBusEnableFlag))
#define CLEAN_I2CBUS_FLAG(_x_)      ((~(1<<_x_))&(gCurrI2CBusEnableFlag))

static DEFINE_MUTEX(kdCam_Mutex);
static BOOL bSesnorVsyncFlag = FALSE;
static ACDK_KD_SENSOR_SYNC_STRUCT g_NewSensorExpGain = {128, 128, 128, 128, 1000, 640, 0xFF, 0xFF, 0xFF, 0};


extern MULTI_SENSOR_FUNCTION_STRUCT2 kd_MultiSensorFunc;
static MULTI_SENSOR_FUNCTION_STRUCT2 *g_pSensorFunc = &kd_MultiSensorFunc;
/* static SENSOR_FUNCTION_STRUCT *g_pInvokeSensorFunc[KDIMGSENSOR_MAX_INVOKE_DRIVERS] = {NULL,NULL}; */
/* static BOOL g_bEnableDriver[KDIMGSENSOR_MAX_INVOKE_DRIVERS] = {FALSE,FALSE}; */
BOOL g_bEnableDriver[KDIMGSENSOR_MAX_INVOKE_DRIVERS] = {FALSE, FALSE};
SENSOR_FUNCTION_STRUCT *g_pInvokeSensorFunc[KDIMGSENSOR_MAX_INVOKE_DRIVERS] = {NULL, NULL};
/* static CAMERA_DUAL_CAMERA_SENSOR_ENUM g_invokeSocketIdx[KDIMGSENSOR_MAX_INVOKE_DRIVERS] = {DUAL_CAMERA_NONE_SENSOR,DUAL_CAMERA_NONE_SENSOR}; */
/* static char g_invokeSensorNameStr[KDIMGSENSOR_MAX_INVOKE_DRIVERS][32] = {KDIMGSENSOR_NOSENSOR,KDIMGSENSOR_NOSENSOR}; */
CAMERA_DUAL_CAMERA_SENSOR_ENUM g_invokeSocketIdx[KDIMGSENSOR_MAX_INVOKE_DRIVERS] = {DUAL_CAMERA_NONE_SENSOR, DUAL_CAMERA_NONE_SENSOR};
char g_invokeSensorNameStr[KDIMGSENSOR_MAX_INVOKE_DRIVERS][32] = {KDIMGSENSOR_NOSENSOR, KDIMGSENSOR_NOSENSOR};
/* static int g_SensorExistStatus[3]={0,0,0}; */
static wait_queue_head_t kd_sensor_wait_queue;
bool setExpGainDoneFlag = 0;
static unsigned int g_CurrentSensorIdx;
static unsigned int g_IsSearchSensor;
#ifdef CONFIG_HUAWEI_DSM
void camera_report_dsm_err_imgsensor(int type, int err_num , char* sensorName);
//move camera_dsm_log_buff and Macro of BUFFER_SIZE to head file
void camera_report_dsm_err_i2c(u16 i2cId, u16 reg_addr, char* description);
static int  Check_I2C = 0;
#endif
/*=============================================================================

=============================================================================*/
/*******************************************************************************
* i2c relative start
* migrate new style i2c driver interfaces required by Kirby 20100827
********************************************************************************/
static const struct i2c_device_id CAMERA_HW_i2c_id[] = {{CAMERA_HW_DRVNAME1, 0}, {} };
static const struct i2c_device_id CAMERA_HW_i2c_id2[] = {{CAMERA_HW_DRVNAME2, 0}, {} };



/*******************************************************************************
* general camera image sensor kernel driver
*******************************************************************************/
/* l681 main camera HAL slot (kd_sensorlist.h FORGE_L681_CAM_LIST): 3 =
 * sunny2 0xd856 (default), 0 = ov13853 0xd853. Read on each list lookup,
 * so set it before the camera service starts. */
static int forge_cam_l681_main_slot = 3;

static int forge_cam_l681_slot_set(const char *val, const struct kernel_param *kp)
{
	int v, ret = kstrtoint(val, 0, &v);

	if (ret)
		return ret;
	if (v != 0 && v != 3)
		return -EINVAL;
	forge_cam_l681_main_slot = v;
	return 0;
}

static const struct kernel_param_ops forge_cam_l681_slot_ops = {
	.set = forge_cam_l681_slot_set,
	.get = param_get_int,
};
module_param_cb(forge_cam_l681_main_slot, &forge_cam_l681_slot_ops,
		&forge_cam_l681_main_slot, 0644);
MODULE_PARM_DESC(forge_cam_l681_main_slot,
		 "l681: HAL slot of the main camera, 3 = sunny2 0xd856 (default) or 0 = ov13853 0xd853");

/* the ID the l681 main sensor driver reports for its slot */
u32 forge_cam_l681_main_id(void)
{
	return READ_ONCE(forge_cam_l681_main_slot) ? OV13853SUNNY2_SENSOR_ID :
						      OV13853_SENSOR_ID;
}
EXPORT_SYMBOL(forge_cam_l681_main_id);

UINT32 kdGetSensorInitFuncList(ACDK_KD_SENSOR_INIT_FUNCTION_STRUCT **ppSensorList)
{
    if (NULL == ppSensorList)
    {
    PK_ERR("[kdGetSensorInitFuncList]ERROR: NULL ppSensorList\n");
    return 1;
    }
    *ppSensorList = &kdSensorList[0];
#ifdef FORGE_L681_CAM_LIST
    if (m3note_board_is(M3NOTE_BOARD_L681))
	*ppSensorList = READ_ONCE(forge_cam_l681_main_slot) ?
			&kdSensorListL681Slot3[0] : &kdSensorListL681Slot0[0];
#endif
    M6_CAM_DIAG("CAM_SENSOR_LIST ptr=%p first_id=0x%08x first_name=%s max=%u\n",
        *ppSensorList, (*ppSensorList)[0].SensorId, (*ppSensorList)[0].drvname,
        MAX_NUM_OF_SUPPORT_SENSOR);
    return 0;
} /* kdGetSensorInitFuncList() */


/*******************************************************************************
*iMultiReadReg
********************************************************************************/
int iMultiReadReg(u16 a_u2Addr , u8 *a_puBuff , u16 i2cId, u8 number)
{
    int  i4RetValue = 0;
    char puReadCmd[2] = {(char)(a_u2Addr >> 8) , (char)(a_u2Addr & 0xFF)};

    if (kd_i2c_client_missing())
        return -ENODEV;

    if (gI2CBusNum == SUPPORT_I2C_BUS_NUM1) {
    spin_lock(&kdsensor_drv_lock);

    g_pstI2Cclient->addr = (i2cId >> 1);

    spin_unlock(&kdsensor_drv_lock);

    /*  */
    i4RetValue = i2c_master_send(g_pstI2Cclient, puReadCmd, 2);
    if (i4RetValue != 2) {
        PK_ERR("[CAMERA SENSOR] I2C send failed, addr = 0x%x, data = 0x%x !!\n", a_u2Addr,  *a_puBuff);
        return -1;
    }
    /*  */
    i4RetValue = i2c_master_recv(g_pstI2Cclient, (char *)a_puBuff, number);
    if (i4RetValue != 1) {
        PK_ERR("[CAMERA SENSOR] I2C read failed!!\n");
        return -1;
    }
    }
    else {
    spin_lock(&kdsensor_drv_lock);
    g_pstI2Cclient2->addr = (i2cId >> 1);
    spin_unlock(&kdsensor_drv_lock);
    /*  */
    i4RetValue = i2c_master_send(g_pstI2Cclient2, puReadCmd, 2);
    if (i4RetValue != 2) {
        PK_ERR("[CAMERA SENSOR] I2C send failed, addr = 0x%x, data = 0x%x !!\n", a_u2Addr,  *a_puBuff);
        return -1;
    }
    /*  */
    i4RetValue = i2c_master_recv(g_pstI2Cclient2, (char *)a_puBuff, number);
    if (i4RetValue != 1) {
        PK_ERR("[CAMERA SENSOR] I2C read failed!!\n");
        return -1;
    }
    }
    return 0;
}


/*******************************************************************************
* iReadReg
********************************************************************************/
int iReadReg(u16 a_u2Addr , u8 *a_puBuff , u16 i2cId)
{
    int  i4RetValue = 0;
    char puReadCmd[2] = {(char)(a_u2Addr >> 8) , (char)(a_u2Addr & 0xFF)};

    if (kd_i2c_client_missing())
        return -ENODEV;

    if (gI2CBusNum == SUPPORT_I2C_BUS_NUM1) {
    spin_lock(&kdsensor_drv_lock);

    g_pstI2Cclient->addr = (i2cId >> 1);
    do {} while (0); /* plain i2c: no ext_flag/timing fields */

    /* Remove i2c ack error log during search sensor */
    if (g_IsSearchSensor == 1)
        do {} while (0); /* plain i2c: no ext_flag/timing fields */
    else
        do {} while (0); /* plain i2c: no ext_flag/timing fields */


    spin_unlock(&kdsensor_drv_lock);

    /*  */
    i4RetValue = i2c_master_send(g_pstI2Cclient, puReadCmd, 2);
    if (i4RetValue != 2) {
#ifdef CONFIG_HUAWEI_DSM
	if (g_IsSearchSensor != 1){
        if(Check_I2C == 2)
    	    camera_report_dsm_err_i2c(i2cId,a_u2Addr,"iReadReg I2C send failed!");
    }
#endif
        PK_ERR("[CAMERA SENSOR] I2C send failed, addr = 0x%x, data = 0x%x !!\n", a_u2Addr,  *a_puBuff);
        return -1;
    }
    /*  */
    i4RetValue = i2c_master_recv(g_pstI2Cclient, (char *)a_puBuff, 1);
    if (i4RetValue != 1) {
#ifdef CONFIG_HUAWEI_DSM
	if (g_IsSearchSensor != 1){
        if(Check_I2C == 2)
            camera_report_dsm_err_i2c(i2cId,a_u2Addr,"iReadReg I2C read failed!");
    }
#endif
        PK_ERR("[CAMERA SENSOR] I2C read failed!!\n");
        return -1;
    }
    }
    else {
    spin_lock(&kdsensor_drv_lock);
    g_pstI2Cclient2->addr = (i2cId >> 1);

    /* Remove i2c ack error log during search sensor */
    if (g_IsSearchSensor == 1)
        do {} while (0); /* plain i2c: no ext_flag/timing fields */
    else
        do {} while (0); /* plain i2c: no ext_flag/timing fields */
    spin_unlock(&kdsensor_drv_lock);
    /*  */
    i4RetValue = i2c_master_send(g_pstI2Cclient2, puReadCmd, 2);
    if (i4RetValue != 2) {
#ifdef CONFIG_HUAWEI_DSM
	if (g_IsSearchSensor != 1){
        if(Check_I2C == 2)
	        camera_report_dsm_err_i2c(i2cId,a_u2Addr,"iReadReg I2C send failed!");
    }
#endif
        PK_ERR("[CAMERA SENSOR] I2C send failed, addr = 0x%x, data = 0x%x !!\n", a_u2Addr,  *a_puBuff);
        return -1;
    }
    /*  */
    i4RetValue = i2c_master_recv(g_pstI2Cclient2, (char *)a_puBuff, 1);
    if (i4RetValue != 1) {
#ifdef CONFIG_HUAWEI_DSM
	if (g_IsSearchSensor != 1){
        if(Check_I2C == 2)
	        camera_report_dsm_err_i2c(i2cId,a_u2Addr,"iReadReg I2C read failed!");
    }
#endif
        PK_ERR("[CAMERA SENSOR] I2C read failed!!\n");
        return -1;
	}
	}
	return 0;
}
/******************************************************************
*jijin.wang add check af i2c is ture
*******************************************************************/
int s4AF_ReadReg_I2C(u16 i2cId)
{
	int i4RetValue = 0;
	char pBuff[2];

	if (!g_pstI2Cclient)
		return 0;	/* no camera_main client: "AF read failed" */
	g_pstI2Cclient->addr = (i2cId >> 1);
	i4RetValue = i2c_master_recv(g_pstI2Cclient,pBuff, 2);
	if (i4RetValue < 0) {
		PK_ERR("Check AF I2C read failed!!\n");
		return 0;
	}
	return 1;
}
EXPORT_SYMBOL(s4AF_ReadReg_I2C);
/******************************************************************************
 *	*jijin.wang add iReadRegI2C_OTP for hi553 read otp data start
 *		*******************************************************************************/
int iReadRegI2C_OTP(u8 *a_pSendData , u16 a_sizeSendData, u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId)
{
	int  i4RetValue = 0;                   
	struct i2c_msg msg[2];


	if (kd_i2c_client_missing())
		return -ENODEV;

	if (gI2CBusNum == SUPPORT_I2C_BUS_NUM1) 
	{
		spin_lock(&kdsensor_drv_lock);
		g_pstI2Cclient->addr = (i2cId >> 1);
		do {} while (0); /* plain i2c: no ext_flag/timing fields */


		/* Remove i2c ack error log during search sensor */
		PK_ERR("g_pstI2Cclient->ext_flag: %d", g_IsSearchSensor);
		if (g_IsSearchSensor == 1)
		  do {} while (0); /* plain i2c: no ext_flag/timing fields */
		else
		  do {} while (0); /* plain i2c: no ext_flag/timing fields */
		spin_unlock(&kdsensor_drv_lock);

		/*  */
		msg[0].addr = g_pstI2Cclient->addr;
		msg[0].flags = g_pstI2Cclient->flags & I2C_M_TEN;
		msg[0].len = a_sizeSendData;
		msg[0].buf = a_pSendData; 
		do {} while (0); /* plain i2c: no ext_flag/timing fields */
		do {} while (0); /* plain i2c: no ext_flag/timing fields */

		msg[1].addr = g_pstI2Cclient->addr;
		msg[1].flags = g_pstI2Cclient->flags & I2C_M_TEN;
		msg[1].flags |= I2C_M_RD;
		msg[1].len = a_sizeRecvData;
		msg[1].buf = a_pRecvData;
		do {} while (0); /* plain i2c: no ext_flag/timing fields */
		do {} while (0); /* plain i2c: no ext_flag/timing fields */

		i4RetValue = i2c_transfer(g_pstI2Cclient->adapter, msg, 2);
		if (!i4RetValue) {
			PK_ERR("[CAMERA SENSOR] I2C read failed!!, Addr = 0x%x in %d\n", a_pSendData[0],__LINE__);
			return -1;
		}                      
	}
	else
	{
		spin_lock(&kdsensor_drv_lock);
		g_pstI2Cclient2->addr = (i2cId >> 1);


		/* Remove i2c ack error log during search sensor */
		/* PK_ERR("g_pstI2Cclient2->ext_flag: %d", g_IsSearchSensor); */
		if (g_IsSearchSensor == 1)
		  do {} while (0); /* plain i2c: no ext_flag/timing fields */
		else
		  do {} while (0); /* plain i2c: no ext_flag/timing fields */
		spin_unlock(&kdsensor_drv_lock);

		/*        */
		msg[0].addr = g_pstI2Cclient2->addr;
		msg[0].flags = g_pstI2Cclient2->flags & I2C_M_TEN;
		msg[0].len = a_sizeSendData ;
		msg[0].buf = a_pSendData;
		do {} while (0); /* plain i2c: no ext_flag/timing fields */
		do {} while (0); /* plain i2c: no ext_flag/timing fields */

		msg[1].addr = g_pstI2Cclient2->addr;
		msg[1].flags = g_pstI2Cclient2->flags & I2C_M_TEN;
		msg[1].flags |= I2C_M_RD;
		msg[1].len = a_sizeRecvData ;
		msg[1].buf = a_pRecvData;
		do {} while (0); /* plain i2c: no ext_flag/timing fields */
		do {} while (0); /* plain i2c: no ext_flag/timing fields */

		i4RetValue = i2c_transfer(g_pstI2Cclient2->adapter, msg, 2);
		if (!i4RetValue) {
			PK_ERR("[CAMERA SENSOR] I2C read failed!!, Addr = 0x%x in %d\n", a_pSendData[0],__LINE__);
			return -1;
		}
	}
	return 0;
}
EXPORT_SYMBOL(iReadRegI2C_OTP);
/******************************************************************************
 *	*jijin.wang add iReadRegI2C_OTP for hi553 read otp data end
 *		*******************************************************************************/
/*******************************************************************************
* iReadRegI2C
********************************************************************************/
int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId)
{
    int  i4RetValue = 0;
    u8 reg0 = a_sizeSendData > 0 ? a_pSendData[0] : 0;
    u8 reg1 = a_sizeSendData > 1 ? a_pSendData[1] : 0;
    if (kd_i2c_client_missing())
        return -ENODEV;

    if (gI2CBusNum == SUPPORT_I2C_BUS_NUM1) {
    spin_lock(&kdsensor_drv_lock);
    g_pstI2Cclient->addr = (i2cId >> 1);
    do {} while (0); /* plain i2c: no ext_flag/timing fields */

    /* Remove i2c ack error log during search sensor */
    /* PK_ERR("g_pstI2Cclient->ext_flag: %d", g_IsSearchSensor); */
    if (g_IsSearchSensor == 1)
        do {} while (0); /* plain i2c: no ext_flag/timing fields */
    else
        do {} while (0); /* plain i2c: no ext_flag/timing fields */

    spin_unlock(&kdsensor_drv_lock);
    /*  */
    i4RetValue = i2c_master_send(g_pstI2Cclient, a_pSendData, a_sizeSendData);
    if (i4RetValue != a_sizeSendData) {
#ifdef CONFIG_HUAWEI_DSM
	if (g_IsSearchSensor != 1){
        if(Check_I2C == 2)
	        camera_report_dsm_err_i2c(i2cId,a_pSendData[0],"iReadRegI2C I2C send failed!");
    }
#endif
        PK_ERR("[M6_CAM] i2c send fail bus=%u client=main adapter=%d addr=0x%x reg=0x%02x%02x ret=%d want=%u search=%u timing=%u ext=0x%x\n",
            gI2CBusNum, g_pstI2Cclient->adapter ? g_pstI2Cclient->adapter->nr : -1,
            i2cId, reg0, reg1, i4RetValue, a_sizeSendData,
            g_IsSearchSensor, 0U, 0U);
        PK_ERR("[CAMERA SENSOR] I2C send failed!!, Addr = 0x%x\n", a_pSendData[0]);
        return -1;
    }

    i4RetValue = i2c_master_recv(g_pstI2Cclient, (char *)a_pRecvData, a_sizeRecvData);
	    if (i4RetValue != a_sizeRecvData) {
#ifdef CONFIG_HUAWEI_DSM
	if (g_IsSearchSensor != 1){
        if(Check_I2C == 2)
	        camera_report_dsm_err_i2c(i2cId,a_pSendData[0],"iReadRegI2C I2C read failed!");
    }
#endif
        PK_ERR("[M6_CAM] i2c recv fail bus=%u client=main adapter=%d addr=0x%x reg=0x%02x%02x ret=%d want=%u search=%u timing=%u ext=0x%x\n",
            gI2CBusNum, g_pstI2Cclient->adapter ? g_pstI2Cclient->adapter->nr : -1,
            i2cId, reg0, reg1, i4RetValue, a_sizeRecvData,
            g_IsSearchSensor, 0U, 0U);
	        PK_ERR("[CAMERA SENSOR] I2C read failed!!\n");
	        return -1;
	    }
	    if (g_IsSearchSensor == 1)
	        PK_ERR("[M6_CAM] i2c recv ok bus=%u client=main adapter=%d addr=0x%x reg=0x%02x%02x len=%u first=0x%02x timing=%u ext=0x%x\n",
	            gI2CBusNum, g_pstI2Cclient->adapter ? g_pstI2Cclient->adapter->nr : -1,
	            i2cId, reg0, reg1, a_sizeRecvData,
	            a_sizeRecvData > 0 ? a_pRecvData[0] : 0,
	            0U, 0U);
	    }
    else{
    spin_lock(&kdsensor_drv_lock);
    g_pstI2Cclient2->addr = (i2cId >> 1);

    /* Remove i2c ack error log during search sensor */
    /* PK_ERR("g_pstI2Cclient2->ext_flag: %d", g_IsSearchSensor); */
    if (g_IsSearchSensor == 1)
        do {} while (0); /* plain i2c: no ext_flag/timing fields */
    else
        do {} while (0); /* plain i2c: no ext_flag/timing fields */
    spin_unlock(&kdsensor_drv_lock);
    i4RetValue = i2c_master_send(g_pstI2Cclient2, a_pSendData, a_sizeSendData);
    if (i4RetValue != a_sizeSendData) {
#ifdef CONFIG_HUAWEI_DSM
	if (g_IsSearchSensor != 1){
        if(Check_I2C == 2)
	        camera_report_dsm_err_i2c(i2cId,a_pSendData[0],"iReadRegI2C I2C send failed!");
    }
#endif
        PK_ERR("[M6_CAM] i2c send fail bus=%u client=bus2 adapter=%d addr=0x%x reg=0x%02x%02x ret=%d want=%u search=%u timing=%u ext=0x%x\n",
            gI2CBusNum, g_pstI2Cclient2->adapter ? g_pstI2Cclient2->adapter->nr : -1,
            i2cId, reg0, reg1, i4RetValue, a_sizeSendData,
            g_IsSearchSensor, 0U, 0U);
        PK_ERR("[CAMERA SENSOR] I2C send failed!!, Addr = 0x%x\n", a_pSendData[0]);
        return -1;
    }

    i4RetValue = i2c_master_recv(g_pstI2Cclient2, (char *)a_pRecvData, a_sizeRecvData);
	    if (i4RetValue != a_sizeRecvData) {
#ifdef CONFIG_HUAWEI_DSM
	if (g_IsSearchSensor != 1){
        if(Check_I2C == 2)
	        camera_report_dsm_err_i2c(i2cId,a_pSendData[0],"iReadRegI2C I2C read failed!");
    }
#endif
        PK_ERR("[M6_CAM] i2c recv fail bus=%u client=bus2 adapter=%d addr=0x%x reg=0x%02x%02x ret=%d want=%u search=%u timing=%u ext=0x%x\n",
            gI2CBusNum, g_pstI2Cclient2->adapter ? g_pstI2Cclient2->adapter->nr : -1,
            i2cId, reg0, reg1, i4RetValue, a_sizeRecvData,
            g_IsSearchSensor, 0U, 0U);
	        PK_ERR("[CAMERA SENSOR] I2C read failed!!\n");
	        return -1;
	    }
	    if (g_IsSearchSensor == 1)
	        PK_ERR("[M6_CAM] i2c recv ok bus=%u client=bus2 adapter=%d addr=0x%x reg=0x%02x%02x len=%u first=0x%02x timing=%u ext=0x%x\n",
	            gI2CBusNum, g_pstI2Cclient2->adapter ? g_pstI2Cclient2->adapter->nr : -1,
	            i2cId, reg0, reg1, a_sizeRecvData,
	            a_sizeRecvData > 0 ? a_pRecvData[0] : 0,
	            0U, 0U);
	    }
    return 0;
}


/*******************************************************************************
* iWriteReg
********************************************************************************/
int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId)
{
    int  i4RetValue = 0;
    int u4Index = 0;
    u8 *puDataInBytes = (u8 *)&a_u4Data;
    int retry = 3;

    char puSendCmd[6] = {(char)(a_u2Addr >> 8) , (char)(a_u2Addr & 0xFF) ,
    0 , 0 , 0 , 0};

/* PK_DBG("Addr : 0x%x,Val : 0x%x\n",a_u2Addr,a_u4Data); */

    /* KD_IMGSENSOR_PROFILE_INIT(); */
    if (kd_i2c_client_missing())
        return -ENODEV;

    spin_lock(&kdsensor_drv_lock);

    if (gI2CBusNum == SUPPORT_I2C_BUS_NUM1) {
    g_pstI2Cclient->addr = (i2cId >> 1);
    do {} while (0); /* plain i2c: no ext_flag/timing fields */
    }
    else {
    g_pstI2Cclient2->addr = (i2cId >> 1);
    do {} while (0); /* plain i2c: no ext_flag/timing fields */
    }
    spin_unlock(&kdsensor_drv_lock);


    if (a_u4Bytes > 2)
    {
    PK_ERR("[CAMERA SENSOR] exceed 2 bytes\n");
    return -1;
    }

    if (a_u4Data >> (a_u4Bytes << 3))
    {
    PK_DBG("[CAMERA SENSOR] warning!! some data is not sent!!\n");
    }

    for (u4Index = 0; u4Index < a_u4Bytes; u4Index += 1)
    {
    puSendCmd[(u4Index + 2)] = puDataInBytes[(a_u4Bytes - u4Index-1)];
    }
    /*  */
    do {
       if (gI2CBusNum == SUPPORT_I2C_BUS_NUM1) {
        i4RetValue = i2c_master_send(g_pstI2Cclient, puSendCmd, (a_u4Bytes + 2));
       }
       else {
        i4RetValue = i2c_master_send(g_pstI2Cclient2, puSendCmd, (a_u4Bytes + 2));
       }
    if (i4RetValue != (a_u4Bytes + 2)) {
#ifdef CONFIG_HUAWEI_DSM
    //if(Check_I2C == 2)
       //camera_report_dsm_err_i2c(i2cId,a_u2Addr,"iWriteReg I2C send failed!");
#endif
    PK_ERR("[CAMERA SENSOR] I2C send failed addr = 0x%x, data = 0x%x !!\n", a_u2Addr, a_u4Data);
	i4RetValue = -1;
    }
    else {
	i4RetValue = 0;
        break;
    }
    uDELAY(50);
    } while ((retry--) > 0);
    /* KD_IMGSENSOR_PROFILE("iWriteReg"); */
	return i4RetValue;
}

int kdSetI2CBusNum(u32 i2cBusNum) {
    //Main2 Support
    //if ((i2cBusNum != SUPPORT_I2C_BUS_NUM2) && (i2cBusNum != SUPPORT_I2C_BUS_NUM1)) {
    if ((i2cBusNum != SUPPORT_I2C_BUS_NUM3) && (i2cBusNum != SUPPORT_I2C_BUS_NUM2) && (i2cBusNum != SUPPORT_I2C_BUS_NUM1)) {
    PK_ERR("[kdSetI2CBusNum] i2c bus number is not correct(%d)\n", i2cBusNum);
    return -1;
    }
    spin_lock(&kdsensor_drv_lock);
    gI2CBusNum = i2cBusNum;
    spin_unlock(&kdsensor_drv_lock);

    return 0;
}

void kdSetI2CSpeed(u32 i2cSpeed)
{
     if (gI2CBusNum == SUPPORT_I2C_BUS_NUM1) {
    spin_lock(&kdsensor_drv_lock);
    do {} while (0); /* plain i2c: no ext_flag/timing fields */
    spin_unlock(&kdsensor_drv_lock);
     }
     else{
    spin_lock(&kdsensor_drv_lock);
    do {} while (0); /* plain i2c: no ext_flag/timing fields */
    spin_unlock(&kdsensor_drv_lock);
     }

}

/*******************************************************************************
* kdReleaseI2CTriggerLock
********************************************************************************/
int kdReleaseI2CTriggerLock(void)
{
    int ret = 0;

    /* ret = mt_wait4_i2c_complete(); */

    /* if (ret < 0 ) { */
    /* PK_DBG("[error]wait i2c fail\n"); */
    /* } */

    return ret;
}
/*******************************************************************************
* iBurstWriteReg
********************************************************************************/
/* The 3.18 MTK DMA multi-transfer packed-length model
 * (len = (count<<16)|transfer_length with I2C_DMA_FLAG, phys-addr buf) does
 * not exist in the i2c-mtk bus driver here (no ext_flag on
 * i2c_client/i2c_msg; the bus driver bounce-buffers DMA internally). Send
 * each transfer_length chunk as a normal buffered transfer. Returns
 * transfer_length on success to keep the caller's
 * (ret&0xffff)!=transfer_length retry check intact. */
static int m6_i2c_burst_send(struct i2c_client *client, u8 *vbuf, u32 bytes, u16 transfer_length)
{
    u32 off;
    int ret;

    if (transfer_length == 0 || (bytes % transfer_length))
        return -1;
    for (off = 0; off < bytes; off += transfer_length) {
        ret = i2c_master_send(client, vbuf + off, transfer_length);
        if (ret != transfer_length)
            return ret;
    }
    return transfer_length;
}

#define MAX_CMD_LEN          255
int iBurstWriteReg_multi(u8 *pData, u32 bytes, u16 i2cId, u16 transfer_length)
{

    dma_addr_t phyAddr;
    u8 *buf = NULL;
    u32 old_addr = 0;
    int ret = 0;
    int retry = 0;

    if (kd_i2c_client_missing())
        return -ENODEV;

    if (gI2CBusNum == SUPPORT_I2C_BUS_NUM1) {
    if (bytes > MAX_CMD_LEN) {
        PK_ERR("[iBurstWriteReg] exceed the max write length\n");
        return 1;
    }

    phyAddr = 0;

    buf = dma_alloc_coherent(&(pCamerahw_platform_device->dev) , bytes, (dma_addr_t *)&phyAddr, GFP_KERNEL);

    if (NULL == buf) {
        PK_ERR("[iBurstWriteReg] Not enough memory\n");
        return -1;
    }
    memset(buf, 0, bytes);
    memcpy(buf, pData, bytes);
    /* PK_DBG("[iBurstWriteReg] bytes = %d, phy addr = 0x%x\n", bytes, phyAddr ); */
	PK_DBG("transfer_length %d\n",bytes == transfer_length ? transfer_length : (((bytes/transfer_length)<<16)|transfer_length));

    old_addr = g_pstI2Cclient->addr;
    spin_lock(&kdsensor_drv_lock);
       g_pstI2Cclient->addr = (i2cId >> 1);
       do {} while (0); /* plain i2c: no ext_flag/timing fields */
       do {} while (0); /* plain i2c: no ext_flag/timing fields */
    spin_unlock(&kdsensor_drv_lock);

    ret = 0;
    retry = 3;
    do {
        ret = m6_i2c_burst_send(g_pstI2Cclient, buf, bytes, transfer_length);
        retry--;
        if ((ret&0xffff) != transfer_length) {
        PK_ERR("Error sent I2C ret = %d\n", ret);
        }
    } while (((ret&0xffff) != transfer_length) && (retry > 0));

    dma_free_coherent(&(pCamerahw_platform_device->dev), bytes, buf, phyAddr);
    spin_lock(&kdsensor_drv_lock);
    g_pstI2Cclient->addr = old_addr;
    spin_unlock(&kdsensor_drv_lock);
    }
    else
    {
    if (bytes > MAX_CMD_LEN) {
        PK_DBG("[iBurstWriteReg] exceed the max write length\n");
        return 1;
    }
    phyAddr = 0;
    buf = dma_alloc_coherent(&(pCamerahw_platform_device->dev), bytes, (dma_addr_t *)&phyAddr, GFP_KERNEL);

    if (NULL == buf) {
        PK_ERR("[iBurstWriteReg] Not enough memory\n");
        return -1;
    }
    memset(buf, 0, bytes);
    memcpy(buf, pData, bytes);
    /* PK_DBG("[iBurstWriteReg] bytes = %d, phy addr = 0x%x\n", bytes, phyAddr ); */

    old_addr = g_pstI2Cclient2->addr;
    spin_lock(&kdsensor_drv_lock);
    g_pstI2Cclient2->addr = (i2cId >> 1);
    do {} while (0); /* plain i2c: no ext_flag/timing fields */
    do {} while (0); /* plain i2c: no ext_flag/timing fields */
    spin_unlock(&kdsensor_drv_lock);
    ret = 0;
    retry = 3;
    do {
        ret = m6_i2c_burst_send(g_pstI2Cclient2, buf, bytes, transfer_length);
        retry--;
        if ((ret&0xffff) != transfer_length) {
        PK_ERR("Error sent I2C ret = %d\n", ret);
        }
    } while (((ret&0xffff) != transfer_length) && (retry > 0));


    dma_free_coherent(&(pCamerahw_platform_device->dev), bytes, buf, phyAddr);
    spin_lock(&kdsensor_drv_lock);
    g_pstI2Cclient2->addr = old_addr;
    spin_unlock(&kdsensor_drv_lock);
    }
    return 0;
}

int iBurstWriteReg(u8 *pData, u32 bytes, u16 i2cId) {
    return  iBurstWriteReg_multi(pData, bytes, i2cId, bytes);
}


/*******************************************************************************
* iMultiWriteReg
********************************************************************************/

int iMultiWriteReg(u8 *pData, u16 lens, u16 i2cId)
{
    int ret = 0;

    if (kd_i2c_client_missing())
        return -ENODEV;

    if (gI2CBusNum == SUPPORT_I2C_BUS_NUM1) {
    g_pstI2Cclient->addr = (i2cId >> 1);
    do {} while (0); /* plain i2c: no ext_flag/timing fields */
    ret = i2c_master_send(g_pstI2Cclient, pData, lens);
    }
    else {
    g_pstI2Cclient2->addr = (i2cId >> 1);
    do {} while (0); /* plain i2c: no ext_flag/timing fields */
    ret = i2c_master_send(g_pstI2Cclient2, pData, lens);
    }

    if (ret != lens) {
    PK_ERR("Error sent I2C ret = %d\n", ret);
    }
    return 0;
}


/*******************************************************************************
* iWriteRegI2C
********************************************************************************/
int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId)
{
    int  i4RetValue = 0;
    int retry = 3;

/* PK_DBG("Addr : 0x%x,Val : 0x%x\n",a_u2Addr,a_u4Data); */

    /* KD_IMGSENSOR_PROFILE_INIT(); */
    if (kd_i2c_client_missing())
        return -ENODEV;

    spin_lock(&kdsensor_drv_lock);
    if (gI2CBusNum == SUPPORT_I2C_BUS_NUM1) {
    g_pstI2Cclient->addr = (i2cId >> 1);
    do {} while (0); /* plain i2c: no ext_flag/timing fields */
    }
    else {
    g_pstI2Cclient2->addr = (i2cId >> 1);
    do {} while (0); /* plain i2c: no ext_flag/timing fields */
    }
    spin_unlock(&kdsensor_drv_lock);
    /*  */

    do {
    if (gI2CBusNum == SUPPORT_I2C_BUS_NUM1) {
        i4RetValue = i2c_master_send(g_pstI2Cclient, a_pSendData, a_sizeSendData);
    }
    else {
        i4RetValue = i2c_master_send(g_pstI2Cclient2, a_pSendData, a_sizeSendData);
    }
    if (i4RetValue != a_sizeSendData) {
#ifdef CONFIG_HUAWEI_DSM
    //if(Check_I2C == 2)
    //camera_report_dsm_err_i2c(i2cId,a_pSendData[0],"iWriteRegI2C I2C send failed!");
#endif
        PK_ERR("[CAMERA SENSOR] I2C send failed!!, Addr = 0x%x, Data = 0x%x\n", a_pSendData[0], a_pSendData[1]);
    }
    else {
        break;
    }
    uDELAY(50);
    } while ((retry--) > 0);
    /* KD_IMGSENSOR_PROFILE("iWriteRegI2C"); */
    return 0;
}

#ifdef CONFIG_HUAWEI_DSM
void camera_report_dsm_err_imgsensor(int type, int err_num , char* sensorName)
{
	ssize_t len = 0;
	int rc = 0;

	memset(camera_dsm_log_buff, 0, MTK_CAMERA_DSM_BUFFER_SIZE);

	/* camera record error info according to err type */
	switch(type)
	{
		case DSM_CAMERA_OTP_ERR:
			/* report otp error infomation */
			len += snprintf(camera_dsm_log_buff+len, MTK_CAMERA_DSM_BUFFER_SIZE-len, "[mtk_sensor]OTP error.No effective OTP info.\n");
			if (len >= MTK_CAMERA_DSM_BUFFER_SIZE -1)
			{
				pr_err("write camera_dsm_log_buff overflow.\n");
				return ;
			}
			break;

		case DSM_CAMERA_CHIP_ID_NOT_MATCH:
			/* report read id error infomation */
			len += snprintf(camera_dsm_log_buff+len, MTK_CAMERA_DSM_BUFFER_SIZE-len, "[imgsensor:%s]Chip ID DON'T MATCH.\n",sensorName);
			if (len >= MTK_CAMERA_DSM_BUFFER_SIZE -1)
			{
				pr_err("write camera_dsm_log_buff overflow.\n");
				return ;
			}
			break;

		default:
			break;
	}

	rc = camera_report_dsm_err( type, err_num, camera_dsm_log_buff);
	if (rc < 0)
	{
	     pr_err("%s. report dsm err fail.\n", __func__);
	     return ;
	}
	return ;
}

void camera_report_dsm_err_i2c(u16 i2cId, u16 reg_addr, char* description)
{
	ssize_t len = 0;
	int rc = 0;

	memset(camera_dsm_log_buff, 0, MTK_CAMERA_DSM_BUFFER_SIZE);

	len += snprintf(camera_dsm_log_buff+len, MTK_CAMERA_DSM_BUFFER_SIZE-len, "[SlaveId:%x,RegAddr:%x]%s\n",i2cId,reg_addr,description);
	if (len >= MTK_CAMERA_DSM_BUFFER_SIZE -1)
	{
		pr_err("write camera_dsm_log_buff overflow.\n");
		return ;
	}

	rc = camera_report_dsm_err( DSM_CAMERA_I2C_ERR, -1, camera_dsm_log_buff);
	if (rc < 0)
	{
	     pr_err("%s. report dsm err fail.\n", __func__);
	     return ;
	}
	return ;
}
#endif

/*******************************************************************************
* sensor function adapter
********************************************************************************/
#define KD_MULTI_FUNCTION_ENTRY()   /* PK_XLOG_INFO("[%s]:E\n",__FUNCTION__) */
#define KD_MULTI_FUNCTION_EXIT()    /* PK_XLOG_INFO("[%s]:X\n",__FUNCTION__) */
/*  */
MUINT32
kdSetI2CSlaveID(MINT32 i, MUINT32 socketIdx, MUINT32 firstSet) {
unsigned long long FeaturePara[4];
MUINT32 FeatureParaLen = 0;
    FeaturePara[0] = socketIdx;
    FeaturePara[1] = firstSet;
    FeatureParaLen = sizeof(unsigned long long)*2;
    return g_pInvokeSensorFunc[i]->SensorFeatureControl(SENSOR_FEATURE_SET_SLAVE_I2C_ID, (MUINT8 *)FeaturePara, (MUINT32 *)&FeatureParaLen);
}

/*  */
MUINT32
kd_MultiSensorOpen(void)
{
MUINT32 ret = ERROR_NONE;
MINT32 i = 0;

    KD_MULTI_FUNCTION_ENTRY();
    /* from hear to tail */
    /* for ( i = KDIMGSENSOR_INVOKE_DRIVER_0 ; i < KDIMGSENSOR_MAX_INVOKE_DRIVERS ; i++ ) { */
    /* from tail to head. */
    for (i = (KDIMGSENSOR_MAX_INVOKE_DRIVERS-1); i >= KDIMGSENSOR_INVOKE_DRIVER_0; i--) {
    if (g_bEnableDriver[i] && g_pInvokeSensorFunc[i]) {
        if (0 != (g_CurrentSensorIdx & g_invokeSocketIdx[i])) {
        /* turn on power */
        ret = kdCISModulePowerOn((CAMERA_DUAL_CAMERA_SENSOR_ENUM)g_invokeSocketIdx[i], (char *)g_invokeSensorNameStr[i], true, CAMERA_HW_DRVNAME1);
        if (ERROR_NONE != ret) {
        PK_ERR("[%s]", __func__);
        return ret;
        }
        /* wait for power stable */
        mDELAY(10);
        KD_IMGSENSOR_PROFILE("kdModulePowerOn");

#if 0
        if (DUAL_CAMERA_MAIN_SENSOR == g_invokeSocketIdx[i] || DUAL_CAMERA_SUB_SENSOR == g_invokeSocketIdx[i] || DUAL_CAMERA_MAIN_2_SENSOR == g_invokeSocketIdx[i]) {
        spin_lock(&kdsensor_drv_lock);
        gI2CBusNum = SENSOR_I2C_BUS_NUM[g_invokeSocketIdx[i]];
        spin_unlock(&kdsensor_drv_lock);
        PK_XLOG_INFO("kd_MultiSensorOpen: switch I2C BUS%d\n", gI2CBusNum);
        }
#else
        if (DUAL_CAMERA_MAIN_2_SENSOR == g_invokeSocketIdx[i]) {
            spin_lock(&kdsensor_drv_lock);
            gI2CBusNum = SUPPORT_I2C_BUS_NUM3;
            spin_unlock(&kdsensor_drv_lock);
            PK_XLOG_INFO("kd_MultiSensorOpen: switch I2C BUS%d\n", gI2CBusNum);
        }

        else if (DUAL_CAMERA_SUB_SENSOR == g_invokeSocketIdx[i]) {
            spin_lock(&kdsensor_drv_lock);
            gI2CBusNum = SUPPORT_I2C_BUS_NUM2;
            spin_unlock(&kdsensor_drv_lock);
            PK_XLOG_INFO("kd_MultiSensorOpen: switch I2C BUS%d\n", gI2CBusNum);
        }
        else {
            spin_lock(&kdsensor_drv_lock);
            gI2CBusNum = SUPPORT_I2C_BUS_NUM1;
            spin_unlock(&kdsensor_drv_lock);
            PK_XLOG_INFO("kd_MultiSensorOpen: switch I2C BUS%d\n", gI2CBusNum);
        }
#endif
        /*  */
        /* set i2c slave ID */
        /* KD_SET_I2C_SLAVE_ID(i,g_invokeSocketIdx[i],IMGSENSOR_SET_I2C_ID_STATE); */
        /*  */
        ret = g_pInvokeSensorFunc[i]->SensorOpen();
        if (ERROR_NONE != ret) {
#ifdef CONFIG_HUAWEI_DSM
	camera_report_dsm_err_imgsensor(DSM_CAMERA_CHIP_ID_NOT_MATCH,ret,g_invokeSensorNameStr[i]);
#endif
        kdCISModulePowerOn((CAMERA_DUAL_CAMERA_SENSOR_ENUM)g_invokeSocketIdx[i], (char *)g_invokeSensorNameStr[i], false, CAMERA_HW_DRVNAME1);
        PK_ERR("SensorOpen");
        return ret;
        }
        /* set i2c slave ID */
        /* SensorOpen() will reset i2c slave ID */
        /* KD_SET_I2C_SLAVE_ID(i,g_invokeSocketIdx[i],IMGSENSOR_SET_I2C_ID_FORCE); */
    }
    }
    }
    KD_MULTI_FUNCTION_EXIT();
    return ERROR_NONE;
}
/*  */

MUINT32
kd_MultiSensorGetInfo(
MUINT32 *pScenarioId[2],
MSDK_SENSOR_INFO_STRUCT *pSensorInfo[2],
MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData[2])
{
    MUINT32 ret = ERROR_NONE;
    u32 i = 0;

    KD_MULTI_FUNCTION_ENTRY();
    for (i = KDIMGSENSOR_INVOKE_DRIVER_0; i < KDIMGSENSOR_MAX_INVOKE_DRIVERS; i++) {
	    if (g_bEnableDriver[i] && g_pInvokeSensorFunc[i]) {
	        if (DUAL_CAMERA_MAIN_SENSOR == g_invokeSocketIdx[i]) {
	        ret = g_pInvokeSensorFunc[i]->SensorGetInfo((MSDK_SCENARIO_ID_ENUM)(*pScenarioId[0]), pSensorInfo[0], pSensorConfigData[0]);
	        }
	        else if ((DUAL_CAMERA_MAIN_2_SENSOR == g_invokeSocketIdx[i]) || (DUAL_CAMERA_SUB_SENSOR == g_invokeSocketIdx[i])) {
	        ret = g_pInvokeSensorFunc[i]->SensorGetInfo((MSDK_SCENARIO_ID_ENUM)(*pScenarioId[1]), pSensorInfo[1], pSensorConfigData[1]);
	        }

	        if (ERROR_NONE != ret) {
	        PK_ERR("[%s]\n", __func__);
	        return ret;
	        }

	    }
    }
    KD_MULTI_FUNCTION_EXIT();
    return ERROR_NONE;
}

/*  */

MUINT32
kd_MultiSensorGetResolution(
MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution[2])
{
    MUINT32 ret = ERROR_NONE;
    u32 i = 0;
    KD_MULTI_FUNCTION_ENTRY();
    PK_INF("g_bEnableDriver[%d][%d]\n", g_bEnableDriver[0], g_bEnableDriver[1]);
    for (i = KDIMGSENSOR_INVOKE_DRIVER_0; i < KDIMGSENSOR_MAX_INVOKE_DRIVERS; i++) {
    if (g_bEnableDriver[i] && g_pInvokeSensorFunc[i]) {
        if (DUAL_CAMERA_MAIN_SENSOR == g_invokeSocketIdx[i]) {
        ret = g_pInvokeSensorFunc[i]->SensorGetResolution(pSensorResolution[0]);
        }
        else if ((DUAL_CAMERA_MAIN_2_SENSOR == g_invokeSocketIdx[i]) || (DUAL_CAMERA_SUB_SENSOR == g_invokeSocketIdx[i])) {
        ret = g_pInvokeSensorFunc[i]->SensorGetResolution(pSensorResolution[1]);
        }

        if (ERROR_NONE != ret) {
        PK_ERR("[%s]\n", __func__);
        return ret;
        }
    }
    }

    KD_MULTI_FUNCTION_EXIT();
    return ERROR_NONE;
}


/*  */
MUINT32
kd_MultiSensorFeatureControl(
CAMERA_DUAL_CAMERA_SENSOR_ENUM InvokeCamera,
MSDK_SENSOR_FEATURE_ENUM FeatureId,
MUINT8 *pFeaturePara,
MUINT32 *pFeatureParaLen)
{
    MUINT32 ret = ERROR_NONE;
    u32 i = 0;
    KD_MULTI_FUNCTION_ENTRY();
    if (FeatureId == SENSOR_FEATURE_CHECK_SENSOR_ID) {
        PK_ERR("[M6_CAM] feature CHECK_SENSOR_ID entry invoke=%d para=%p lenp=%p gDrvIndex=0x%08x\n",
            InvokeCamera, pFeaturePara, pFeatureParaLen, gDrvIndex);
    }
    for (i = KDIMGSENSOR_INVOKE_DRIVER_0; i < KDIMGSENSOR_MAX_INVOKE_DRIVERS; i++) {
    if (g_bEnableDriver[i] && g_pInvokeSensorFunc[i]) {

        if (InvokeCamera == g_invokeSocketIdx[i]) {

#if 0
        if (DUAL_CAMERA_MAIN_SENSOR == g_invokeSocketIdx[i] || DUAL_CAMERA_SUB_SENSOR == g_invokeSocketIdx[i] || DUAL_CAMERA_MAIN_2_SENSOR == g_invokeSocketIdx[i]) {
            spin_lock(&kdsensor_drv_lock);
            gI2CBusNum = SENSOR_I2C_BUS_NUM[g_invokeSocketIdx[i]];
            spin_unlock(&kdsensor_drv_lock);
            PK_XLOG_INFO("kd_MultiSensorOpen: switch I2C BUS%d\n", gI2CBusNum);
        }
#else
        //Main2 Support
        if (DUAL_CAMERA_MAIN_2_SENSOR == g_invokeSocketIdx[i]) {
            spin_lock(&kdsensor_drv_lock);
            gI2CBusNum = SUPPORT_I2C_BUS_NUM3;
            spin_unlock(&kdsensor_drv_lock);
        }
        else if (DUAL_CAMERA_SUB_SENSOR == g_invokeSocketIdx[i]) {
            spin_lock(&kdsensor_drv_lock);
            gI2CBusNum = SUPPORT_I2C_BUS_NUM2;
            spin_unlock(&kdsensor_drv_lock);
            /* PK_XLOG_INFO("kd_MultiSensorFeatureControl: switch I2C BUS2\n"); */
        }
        else {
            spin_lock(&kdsensor_drv_lock);
            gI2CBusNum = SUPPORT_I2C_BUS_NUM1;
            spin_unlock(&kdsensor_drv_lock);
             /* PK_XLOG_INFO("kd_MultiSensorFeatureControl: switch I2C BUS1\n"); */
        }
#endif
        /*  */
        /* set i2c slave ID */
        /* KD_SET_I2C_SLAVE_ID(i,g_invokeSocketIdx[i],IMGSENSOR_SET_I2C_ID_STATE); */
        /*  */
        if (FeatureId == SENSOR_FEATURE_CHECK_SENSOR_ID) {
            PK_ERR("[M6_CAM] feature CHECK_SENSOR_ID call i=%u socket=%d bus=%u name=%s func=%p retLen=%u\n",
                i, g_invokeSocketIdx[i], gI2CBusNum,
                g_invokeSensorNameStr[i], g_pInvokeSensorFunc[i],
                pFeatureParaLen ? *pFeatureParaLen : 0);
        }
        ret = g_pInvokeSensorFunc[i]->SensorFeatureControl(FeatureId, pFeaturePara, pFeatureParaLen);
        if (FeatureId == SENSOR_FEATURE_CHECK_SENSOR_ID) {
            PK_ERR("[M6_CAM] feature CHECK_SENSOR_ID ret i=%u socket=%d bus=%u name=%s ret=%u sensorID=0x%08x retLen=%u\n",
                i, g_invokeSocketIdx[i], gI2CBusNum,
                g_invokeSensorNameStr[i], ret,
                pFeaturePara ? *(MUINT32 *)pFeaturePara : 0,
                pFeatureParaLen ? *pFeatureParaLen : 0);
        }
        if (ERROR_NONE != ret) {
            PK_ERR("[%s]\n", __func__);
            return ret;
        }
        }
    }
    }
    KD_MULTI_FUNCTION_EXIT();
    return ERROR_NONE;
}

/*  */
MUINT32
kd_MultiSensorControl(
CAMERA_DUAL_CAMERA_SENSOR_ENUM InvokeCamera,
MSDK_SCENARIO_ID_ENUM ScenarioId,
MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
    MUINT32 ret = ERROR_NONE;
    u32 i = 0;
    KD_MULTI_FUNCTION_ENTRY();
    for (i = KDIMGSENSOR_INVOKE_DRIVER_0; i < KDIMGSENSOR_MAX_INVOKE_DRIVERS; i++) {
    if (g_bEnableDriver[i] && g_pInvokeSensorFunc[i]) {
        if (InvokeCamera == g_invokeSocketIdx[i]) {

#if 0
        if (DUAL_CAMERA_MAIN_SENSOR == g_invokeSocketIdx[i] || DUAL_CAMERA_SUB_SENSOR == g_invokeSocketIdx[i] || DUAL_CAMERA_MAIN_2_SENSOR == g_invokeSocketIdx[i]) {
        spin_lock(&kdsensor_drv_lock);
        gI2CBusNum = SENSOR_I2C_BUS_NUM[g_invokeSocketIdx[i]];
        spin_unlock(&kdsensor_drv_lock);
        PK_XLOG_INFO("kd_MultiSensorOpen: switch I2C BUS%d\n", gI2CBusNum);
        }
#else
        if (DUAL_CAMERA_MAIN_2_SENSOR == g_invokeSocketIdx[i]) {
        spin_lock(&kdsensor_drv_lock);
        gI2CBusNum = SUPPORT_I2C_BUS_NUM3;
        spin_unlock(&kdsensor_drv_lock);
        }
        else if (DUAL_CAMERA_SUB_SENSOR == g_invokeSocketIdx[i]) {
        spin_lock(&kdsensor_drv_lock);
        gI2CBusNum = SUPPORT_I2C_BUS_NUM2;
        spin_unlock(&kdsensor_drv_lock);
         /* PK_XLOG_INFO("kd_MultiSensorControl: switch I2C BUS2\n"); */
        }
        else {
        spin_lock(&kdsensor_drv_lock);
        gI2CBusNum = SUPPORT_I2C_BUS_NUM1;
        spin_unlock(&kdsensor_drv_lock);
         /* PK_XLOG_INFO("kd_MultiSensorControl: switch I2C BUS1\n"); */
        }
#endif
        /*  */
        /* set i2c slave ID */
        /* KD_SET_I2C_SLAVE_ID(i,g_invokeSocketIdx[i],IMGSENSOR_SET_I2C_ID_STATE); */
        /*  */
        g_pInvokeSensorFunc[i]->ScenarioId = ScenarioId;
        memcpy(&g_pInvokeSensorFunc[i]->imageWindow, pImageWindow, sizeof(ACDK_SENSOR_EXPOSURE_WINDOW_STRUCT));
        memcpy(&g_pInvokeSensorFunc[i]->sensorConfigData, pSensorConfigData, sizeof(ACDK_SENSOR_CONFIG_STRUCT));
        ret = g_pInvokeSensorFunc[i]->SensorControl(ScenarioId, pImageWindow, pSensorConfigData);
        if (ERROR_NONE != ret) {
        PK_ERR("ERR:SensorControl(), i =%d\n", i);
        return ret;
        }
    }
    }
    }
    KD_MULTI_FUNCTION_EXIT();


    /* js_tst FIXME */
    /* if (DUAL_CHANNEL_I2C) { */
    /* trigger dual channel i2c */
    /* } */
    /* else { */
    if (g_bEnableDriver[1]) { /* drive 2 or more sensor simultaneously */
    MUINT8 frameSync = 0;
    MUINT32 frameSyncSize = 0;
        kd_MultiSensorFeatureControl(g_invokeSocketIdx[1], SENSOR_FEATURE_SUSPEND, &frameSync, &frameSyncSize);
        mDELAY(10);
        kd_MultiSensorFeatureControl(g_invokeSocketIdx[1], SENSOR_FEATURE_RESUME, &frameSync, &frameSyncSize);
    }
    /* } */


    return ERROR_NONE;
}
/*  */
MUINT32
kd_MultiSensorClose(void)
{
    MUINT32 ret = ERROR_NONE;
    u32 i = 0;
    KD_MULTI_FUNCTION_ENTRY();
    for (i = KDIMGSENSOR_INVOKE_DRIVER_0; i < KDIMGSENSOR_MAX_INVOKE_DRIVERS; i++) {
    if (g_bEnableDriver[i] && g_pInvokeSensorFunc[i]) {
      if (0 != (g_CurrentSensorIdx & g_invokeSocketIdx[i])) {
#if 0
          if (DUAL_CAMERA_MAIN_SENSOR == g_invokeSocketIdx[i] || DUAL_CAMERA_SUB_SENSOR == g_invokeSocketIdx[i] || DUAL_CAMERA_MAIN_2_SENSOR == g_invokeSocketIdx[i]) {
          spin_lock(&kdsensor_drv_lock);
          gI2CBusNum = SENSOR_I2C_BUS_NUM[g_invokeSocketIdx[i]];
          spin_unlock(&kdsensor_drv_lock);
          PK_XLOG_INFO("kd_MultiSensorOpen: switch I2C BUS%d\n", gI2CBusNum);
          }
#else


        if (DUAL_CAMERA_MAIN_2_SENSOR == g_invokeSocketIdx[i]) {
        spin_lock(&kdsensor_drv_lock);
        gI2CBusNum = SUPPORT_I2C_BUS_NUM3;
        spin_unlock(&kdsensor_drv_lock);
        }
        else if (DUAL_CAMERA_SUB_SENSOR == g_invokeSocketIdx[i]) {
        spin_lock(&kdsensor_drv_lock);
        gI2CBusNum = SUPPORT_I2C_BUS_NUM2;
        spin_unlock(&kdsensor_drv_lock);
         PK_XLOG_INFO("kd_MultiSensorClose: switch I2C BUS2\n");
        }
        else {
        spin_lock(&kdsensor_drv_lock);
        gI2CBusNum = SUPPORT_I2C_BUS_NUM1;
        spin_unlock(&kdsensor_drv_lock);
         PK_XLOG_INFO("kd_MultiSensorClose: switch I2C BUS1\n");
        }
#endif
        ret = g_pInvokeSensorFunc[i]->SensorClose();

        /* Change the close power flow to close power in this function & */
        /* directly call kdCISModulePowerOn to close the specific sensor */
        /* The original flow will close all opened sensors at once */
        kdCISModulePowerOn((CAMERA_DUAL_CAMERA_SENSOR_ENUM)g_invokeSocketIdx[i], (char *)g_invokeSensorNameStr[i], false, CAMERA_HW_DRVNAME1);

        if (ERROR_NONE != ret) {
        PK_ERR("[%s]", __func__);
        return ret;
        }
      }
    }
    }
    KD_MULTI_FUNCTION_EXIT();
    return ERROR_NONE;
}
/*  */
MULTI_SENSOR_FUNCTION_STRUCT2  kd_MultiSensorFunc =
{
    kd_MultiSensorOpen,
    kd_MultiSensorGetInfo,
    kd_MultiSensorGetResolution,
    kd_MultiSensorFeatureControl,
    kd_MultiSensorControl,
    kd_MultiSensorClose
};


/*******************************************************************************
* kdModulePowerOn
********************************************************************************/
int
kdModulePowerOn(
CAMERA_DUAL_CAMERA_SENSOR_ENUM socketIdx[KDIMGSENSOR_MAX_INVOKE_DRIVERS],
char sensorNameStr[KDIMGSENSOR_MAX_INVOKE_DRIVERS][32],
BOOL On,
char *mode_name)
{
MINT32 ret = ERROR_NONE;
u32 i = 0;

    for (i = KDIMGSENSOR_INVOKE_DRIVER_0; i < KDIMGSENSOR_MAX_INVOKE_DRIVERS; i++) {
    if (g_bEnableDriver[i]) {
        /* PK_XLOG_INFO("[%s][%d][%d][%s][%s]\r\n",__FUNCTION__,g_bEnableDriver[i],socketIdx[i],sensorNameStr[i],mode_name); */
        ret = kdCISModulePowerOn(socketIdx[i], sensorNameStr[i], On, mode_name);
        if (ERROR_NONE != ret) {
        PK_ERR("[%s]", __func__);
        return ret;
        }
    }
    }
    return ERROR_NONE;
}

/*******************************************************************************
* kdSetDriver
********************************************************************************/
int kdSetDriver(unsigned int *pDrvIndex)
{
    ACDK_KD_SENSOR_INIT_FUNCTION_STRUCT *pSensorList = NULL;
    u32 drvIdx[KDIMGSENSOR_MAX_INVOKE_DRIVERS] = {0, 0};
    u32 i;
    u32 listIdx;

    /* set driver for MAIN or SUB sensor */
    PK_INF("pDrvIndex:0x%08x/0x%08x\n", pDrvIndex[KDIMGSENSOR_INVOKE_DRIVER_0], pDrvIndex[KDIMGSENSOR_INVOKE_DRIVER_1]);
    PK_ERR("[M6_CAM] kdSetDriver enter raw0=0x%08x raw1=0x%08x\n",
        pDrvIndex[KDIMGSENSOR_INVOKE_DRIVER_0], pDrvIndex[KDIMGSENSOR_INVOKE_DRIVER_1]);
    gDrvIndex = pDrvIndex[KDIMGSENSOR_INVOKE_DRIVER_0];

    if (0 != kdGetSensorInitFuncList(&pSensorList))
    {
    PK_ERR("[M6_CAM] ERROR:kdGetSensorInitFuncList()\n");
    return -EIO;
    }

    for (listIdx = 0; listIdx < 4 && listIdx < MAX_NUM_OF_SUPPORT_SENSOR; listIdx++) {
    PK_ERR("[M6_CAM] list[%u] id=0x%08x name=%s init=%p\n",
        listIdx, pSensorList[listIdx].SensorId,
        pSensorList[listIdx].drvname,
        (void *)pSensorList[listIdx].SensorInit);
    }

    for (i = KDIMGSENSOR_INVOKE_DRIVER_0; i < KDIMGSENSOR_MAX_INVOKE_DRIVERS; i++) {
    /*  */
    spin_lock(&kdsensor_drv_lock);
    g_bEnableDriver[i] = FALSE;
    g_invokeSocketIdx[i] = (CAMERA_DUAL_CAMERA_SENSOR_ENUM)((pDrvIndex[i] & KDIMGSENSOR_DUAL_MASK_MSB)>>KDIMGSENSOR_DUAL_SHIFT);
    spin_unlock(&kdsensor_drv_lock);
    drvIdx[i] = (pDrvIndex[i] & KDIMGSENSOR_DUAL_MASK_LSB);
    /*  */
    if (DUAL_CAMERA_NONE_SENSOR == g_invokeSocketIdx[i]) {
        PK_ERR("[M6_CAM] invoke[%d] none raw=0x%08x drvIdx=%u\n", i, pDrvIndex[i], drvIdx[i]);
        continue;
    }
#if 0
            if (DUAL_CAMERA_MAIN_SENSOR == g_invokeSocketIdx[i] || DUAL_CAMERA_SUB_SENSOR == g_invokeSocketIdx[i] || DUAL_CAMERA_MAIN_2_SENSOR == g_invokeSocketIdx[i]) {
            spin_lock(&kdsensor_drv_lock);
            gI2CBusNum = SENSOR_I2C_BUS_NUM[g_invokeSocketIdx[i]];
            spin_unlock(&kdsensor_drv_lock);
            PK_XLOG_INFO("kd_MultiSensorOpen: switch I2C BUS%d\n", gI2CBusNum);
            }
#else

    if (DUAL_CAMERA_MAIN_2_SENSOR == g_invokeSocketIdx[i]) {
        spin_lock(&kdsensor_drv_lock);
        gI2CBusNum = SUPPORT_I2C_BUS_NUM3;
        spin_unlock(&kdsensor_drv_lock);
    }
    else if (DUAL_CAMERA_SUB_SENSOR == g_invokeSocketIdx[i]) {
        spin_lock(&kdsensor_drv_lock);
        gI2CBusNum = SUPPORT_I2C_BUS_NUM2;
        spin_unlock(&kdsensor_drv_lock);
         /* PK_XLOG_INFO("kdSetDriver: switch I2C BUS2\n"); */
    }
    else {
        spin_lock(&kdsensor_drv_lock);
        gI2CBusNum = SUPPORT_I2C_BUS_NUM1;
        spin_unlock(&kdsensor_drv_lock);
         /* PK_XLOG_INFO("kdSetDriver: switch I2C BUS1\n"); */
    }
#endif
    PK_XLOG_INFO("[kdSetDriver]g_invokeSocketIdx[%d] = %d\n", i, g_invokeSocketIdx[i]);
    PK_XLOG_INFO("[kdSetDriver]drvIdx[%d] = %d\n", i, drvIdx[i]);
    PK_ERR("[M6_CAM] invoke[%d] socket=%d drvIdx=%u i2c_bus=%u max=%u\n",
        i, g_invokeSocketIdx[i], drvIdx[i], gI2CBusNum, MAX_NUM_OF_SUPPORT_SENSOR);
    /*  */
    if (MAX_NUM_OF_SUPPORT_SENSOR > drvIdx[i]) {
        PK_ERR("[M6_CAM] selected invoke=%d drvIdx=%u id=0x%08x name=%s init=%p\n",
            i, drvIdx[i], pSensorList[drvIdx[i]].SensorId,
            pSensorList[drvIdx[i]].drvname,
            (void *)pSensorList[drvIdx[i]].SensorInit);
        if (NULL == pSensorList[drvIdx[i]].SensorInit) {
        PK_ERR("[M6_CAM] ERROR:kdSetDriver null init invoke=%d drvIdx=%u name=%s\n",
            i, drvIdx[i], pSensorList[drvIdx[i]].drvname);
        return -EIO;
        }

        pSensorList[drvIdx[i]].SensorInit(&g_pInvokeSensorFunc[i]);
        if (NULL == g_pInvokeSensorFunc[i]) {
        PK_ERR("[M6_CAM] ERROR:NULL g_pSensorFunc[%d] drvIdx=%u name=%s\n",
            i, drvIdx[i], pSensorList[drvIdx[i]].drvname);
        return -EIO;
        }
        /*  */
        spin_lock(&kdsensor_drv_lock);
        g_bEnableDriver[i] = TRUE;
        spin_unlock(&kdsensor_drv_lock);
        /* get sensor name */
        memcpy((char *)g_invokeSensorNameStr[i], (char *)pSensorList[drvIdx[i]].drvname, sizeof(pSensorList[drvIdx[i]].drvname));
        /* return sensor ID */
        /* pDrvIndex[0] = (unsigned int)pSensorList[drvIdx].SensorId; */
        PK_XLOG_INFO("[kdSetDriver] :[%d][%d][%d][%s][%lu]\n", i, g_bEnableDriver[i], g_invokeSocketIdx[i], g_invokeSensorNameStr[i], sizeof(pSensorList[drvIdx[i]].drvname));
        PK_ERR("[M6_CAM] invoke[%d] enabled=%d socket=%d name=%s func=%p\n",
            i, g_bEnableDriver[i], g_invokeSocketIdx[i],
            g_invokeSensorNameStr[i], g_pInvokeSensorFunc[i]);
    } else {
        PK_ERR("[M6_CAM] ERROR:drvIdx out of range invoke=%d drvIdx=%u max=%u raw=0x%08x\n",
            i, drvIdx[i], MAX_NUM_OF_SUPPORT_SENSOR, pDrvIndex[i]);
    }
    }
    PK_ERR("[M6_CAM] kdSetDriver exit ok gDrvIndex=0x%08x\n", gDrvIndex);
    return 0;
}

int kdSetCurrentSensorIdx(unsigned int idx)
{
    g_CurrentSensorIdx = idx;
    return 0;
}
/*******************************************************************************
* kdGetSocketPostion
********************************************************************************/
int
kdGetSocketPostion(unsigned int *pSocketPos) {
    PK_XLOG_INFO("[%s][%d] \r\n", __func__, *pSocketPos);
    switch (*pSocketPos) {
    case DUAL_CAMERA_MAIN_SENSOR:
        /* ->this is a HW layout dependent */
        /* ToDo */
        *pSocketPos = IMGSENSOR_SOCKET_POS_RIGHT;
        break;
    case DUAL_CAMERA_MAIN_2_SENSOR:
        *pSocketPos = IMGSENSOR_SOCKET_POS_LEFT;
        break;
    default:
    case DUAL_CAMERA_SUB_SENSOR:
        *pSocketPos = IMGSENSOR_SOCKET_POS_NONE;
        break;
    }
    return 0;
}
/*******************************************************************************
* kdSetSensorSyncFlag
********************************************************************************/
int kdSetSensorSyncFlag(BOOL bSensorSync)
{
    spin_lock(&kdsensor_drv_lock);

    bSesnorVsyncFlag = bSensorSync;
    spin_unlock(&kdsensor_drv_lock);
/* PK_DBG("[Sensor] kdSetSensorSyncFlag:%d\n", bSesnorVsyncFlag); */

    /* strobe_VDIrq(); //cotta : added for high current solution */

    return 0;
}

/*******************************************************************************
* kdCheckSensorPowerOn
********************************************************************************/
int kdCheckSensorPowerOn(void)
{
    if (atomic_read(&g_CamHWOpening) == 0) {
    return 0;
    }
    else {/* sensor power on */
    return 1;
    }
}

/*******************************************************************************
* kdSensorSyncFunctionPtr
********************************************************************************/
/* ToDo: How to separate main/main2....who is caller? */
int kdSensorSyncFunctionPtr(void)
{
    unsigned int FeatureParaLen = 0;
    /* PK_DBG("[Sensor] kdSensorSyncFunctionPtr1:%d %d %d\n", g_NewSensorExpGain.uSensorExpDelayFrame, g_NewSensorExpGain.uSensorGainDelayFrame, g_NewSensorExpGain.uISPGainDelayFrame); */
    mutex_lock(&kdCam_Mutex);
    if (NULL == g_pSensorFunc) {
    PK_ERR("ERROR:NULL g_pSensorFunc\n");
    mutex_unlock(&kdCam_Mutex);
    return -EIO;
    }
    /* PK_DBG("[Sensor] Exposure time:%d, Gain = %d\n", g_NewSensorExpGain.u2SensorNewExpTime,g_NewSensorExpGain.u2SensorNewGain ); */
    /* exposure time */
    if (g_NewSensorExpGain.uSensorExpDelayFrame == 0) {
    FeatureParaLen = 2;
    g_pSensorFunc->SensorFeatureControl(DUAL_CAMERA_MAIN_SENSOR, SENSOR_FEATURE_SET_ESHUTTER, (unsigned char *)&g_NewSensorExpGain.u2SensorNewExpTime, (unsigned int *) &FeatureParaLen);
    g_NewSensorExpGain.uSensorExpDelayFrame = 0xFF; /* disable */
    }
    else if (g_NewSensorExpGain.uSensorExpDelayFrame != 0xFF) {
    g_NewSensorExpGain.uSensorExpDelayFrame--;
    }

    /* exposure gain */
    if (g_NewSensorExpGain.uSensorGainDelayFrame == 0) {
    FeatureParaLen = 2;
    g_pSensorFunc->SensorFeatureControl(DUAL_CAMERA_MAIN_SENSOR, SENSOR_FEATURE_SET_GAIN, (unsigned char *)&g_NewSensorExpGain.u2SensorNewGain, (unsigned int *) &FeatureParaLen);
    g_NewSensorExpGain.uSensorGainDelayFrame = 0xFF; /* disable */
     }
    else if (g_NewSensorExpGain.uSensorGainDelayFrame != 0xFF) {
    g_NewSensorExpGain.uSensorGainDelayFrame--;
     }

    /* if the delay frame is 0 or 0xFF, stop to count */
    if ((g_NewSensorExpGain.uISPGainDelayFrame != 0xFF) && (g_NewSensorExpGain.uISPGainDelayFrame != 0)) {
		spin_lock(&kdsensor_drv_lock);
		g_NewSensorExpGain.uISPGainDelayFrame--;
		spin_unlock(&kdsensor_drv_lock);
    }
    mutex_unlock(&kdCam_Mutex);
    return 0;
}

/*******************************************************************************
* kdGetRawGainInfo
********************************************************************************/
int kdGetRawGainInfoPtr(UINT16 *pRAWGain)
{
    *pRAWGain = 0x00;
    *(pRAWGain+1) = 0x00;
    *(pRAWGain+2) = 0x00;
    *(pRAWGain+3) = 0x00;

    if (g_NewSensorExpGain.uISPGainDelayFrame == 0)    {  /* synchronize the isp gain */
    *pRAWGain = g_NewSensorExpGain.u2ISPNewRGain;
    *(pRAWGain+1) = g_NewSensorExpGain.u2ISPNewGrGain;
    *(pRAWGain+2) = g_NewSensorExpGain.u2ISPNewGbGain;
    *(pRAWGain+3) = g_NewSensorExpGain.u2ISPNewBGain;
/* PK_DBG("[Sensor] ISP Gain:%d\n", g_NewSensorExpGain.u2ISPNewRGain, g_NewSensorExpGain.u2ISPNewGrGain, */
/* g_NewSensorExpGain.u2ISPNewGbGain, g_NewSensorExpGain.u2ISPNewBGain); */
    spin_lock(&kdsensor_drv_lock);
    g_NewSensorExpGain.uISPGainDelayFrame = 0xFF; /* disable */
    spin_unlock(&kdsensor_drv_lock);
    }

    return 0;
}




int kdSetExpGain(CAMERA_DUAL_CAMERA_SENSOR_ENUM InvokeCamera)
{
    unsigned int FeatureParaLen = 0;
    PK_DBG("[kd_sensorlist]enter kdSetExpGain\n");
    if (NULL == g_pSensorFunc) {
    PK_ERR("ERROR:NULL g_pSensorFunc\n");

    return -EIO;
    }

    setExpGainDoneFlag = 0;
    FeatureParaLen = 2;
    g_pSensorFunc->SensorFeatureControl(InvokeCamera, SENSOR_FEATURE_SET_ESHUTTER, (unsigned char *)&g_NewSensorExpGain.u2SensorNewExpTime, (unsigned int *) &FeatureParaLen);
    g_pSensorFunc->SensorFeatureControl(InvokeCamera, SENSOR_FEATURE_SET_GAIN, (unsigned char *)&g_NewSensorExpGain.u2SensorNewGain, (unsigned int *) &FeatureParaLen);

    setExpGainDoneFlag = 1;
    PK_DBG("[kd_sensorlist]before wake_up_interruptible\n");
    wake_up_interruptible(&kd_sensor_wait_queue);
    PK_DBG("[kd_sensorlist]after wake_up_interruptible\n");

    return 0;   /* No error. */

}

/*******************************************************************************
*
********************************************************************************/
static UINT32 ms_to_jiffies(MUINT32 ms)
{
    return ((ms * HZ + 512) >> 10);
}


int kdSensorSetExpGainWaitDone(int *ptime)
{
    int timeout;
    PK_DBG("[kd_sensorlist]enter kdSensorSetExpGainWaitDone: time: %d\n", *ptime);
    timeout = wait_event_interruptible_timeout(
        kd_sensor_wait_queue,
        (setExpGainDoneFlag & 1),
        ms_to_jiffies(*ptime));

    PK_DBG("[kd_sensorlist]after wait_event_interruptible_timeout\n");
    if (timeout == 0) {
    PK_ERR("[kd_sensorlist] kdSensorSetExpGainWait: timeout=%d\n", *ptime);

    return -EAGAIN;
    }

    return 0;   /* No error. */

}




/*******************************************************************************
* adopt_CAMERA_HW_Open
********************************************************************************/
inline static int adopt_CAMERA_HW_Open(void)
{
    UINT32 err = 0;
#ifdef CONFIG_HUAWEI_DSM
	Check_I2C = 2;
#endif
    KD_IMGSENSOR_PROFILE_INIT();
    /* power on sensor */
    /* if (atomic_read(&g_CamHWOpend) == 0  ) { */
    /* move into SensorOpen() for 2on1 driver */
    /* turn on power */
    /* kdModulePowerOn((CAMERA_DUAL_CAMERA_SENSOR_ENUM*) g_invokeSocketIdx, g_invokeSensorNameStr,true, CAMERA_HW_DRVNAME); */
    /* wait for power stable */
    /* mDELAY(10); */
    /* KD_IMGSENSOR_PROFILE("kdModulePowerOn"); */
    /*  */
     if (g_pSensorFunc) {
        err = g_pSensorFunc->SensorOpen();
        if (ERROR_NONE != err) {
            /*Multiopen fail would close power.*/
            //kdModulePowerOn((CAMERA_DUAL_CAMERA_SENSOR_ENUM *) g_invokeSocketIdx, g_invokeSensorNameStr, false, CAMERA_HW_DRVNAME1);
            PK_ERR("ERROR:SensorOpen(), turn off power\n");
        }
    }
    else {
        PK_DBG(" ERROR:NULL g_pSensorFunc\n");
    }

    KD_IMGSENSOR_PROFILE("SensorOpen");
    /* } */
    /* else { */
    /* PK_ERR("adopt_CAMERA_HW_Open Fail, g_CamHWOpend = %d\n ",atomic_read(&g_CamHWOpend) ); */
    /* } */

    /* if (err == 0 ) { */
    /* atomic_set(&g_CamHWOpend, 1); */

    /* } */

    return err ?  -EIO:err;
}   /* adopt_CAMERA_HW_Open() */

/*******************************************************************************
* adopt_CAMERA_HW_CheckIsAlive
********************************************************************************/
inline static int adopt_CAMERA_HW_CheckIsAlive(void)
{
    UINT32 err = 0;
    UINT32 err1 = 0;
    UINT32 i = 0;
    MUINT32 sensorID = 0;
    MUINT32 retLen = 0;

    KD_IMGSENSOR_PROFILE_INIT();
    PK_ERR("[M6_CAM] check_alive entry gDrvIndex=0x%08x g_pSensorFunc=%p invoke0=%p invoke1=%p enable=%d/%d socket=%d/%d name=%s/%s\n",
        gDrvIndex, g_pSensorFunc, g_pInvokeSensorFunc[0], g_pInvokeSensorFunc[1],
        g_bEnableDriver[0], g_bEnableDriver[1],
        g_invokeSocketIdx[0], g_invokeSocketIdx[1],
        g_invokeSensorNameStr[0], g_invokeSensorNameStr[1]);
    /* power on sensor */
    err1 = kdModulePowerOn((CAMERA_DUAL_CAMERA_SENSOR_ENUM *)g_invokeSocketIdx, g_invokeSensorNameStr, true, CAMERA_HW_DRVNAME1);
    PK_ERR("[M6_CAM] check_alive power_on ret=%u\n", err1);
    /* wait for power stable */
    mDELAY(10);
    KD_IMGSENSOR_PROFILE("kdModulePowerOn");

    g_CurrentSensorIdx = 0;
    g_IsSearchSensor = 1;
    if(gDrvIndex == 0x10000)
    {
        memset(mtk_ccm_name,0,camera_info_size);
    }

    if (g_pSensorFunc) {
    for (i = KDIMGSENSOR_INVOKE_DRIVER_0; i < KDIMGSENSOR_MAX_INVOKE_DRIVERS; i++) {
        if (DUAL_CAMERA_NONE_SENSOR != g_invokeSocketIdx[i]) {
        sensorID = 0;
        retLen = 0;
        PK_ERR("[M6_CAM] check_alive probe i=%u socket=%d name=%s enable=%d invokeFunc=%p\n",
            i, g_invokeSocketIdx[i], g_invokeSensorNameStr[i],
            g_bEnableDriver[i], g_pInvokeSensorFunc[i]);
        err = g_pSensorFunc->SensorFeatureControl(g_invokeSocketIdx[i], SENSOR_FEATURE_CHECK_SENSOR_ID, (MUINT8 *)&sensorID, &retLen);
        if (sensorID == 0) {    /* not implement this feature ID */
            PK_DBG(" Not implement!!, use old open function to check\n");
            err = ERROR_SENSOR_CONNECT_FAIL;
        }
        else if (sensorID == 0xFFFFFFFF) {    /* fail to open the sensor */
            PK_DBG(" No Sensor Found");
            err = ERROR_SENSOR_CONNECT_FAIL;
        }
        else {
            //Check_I2C += 1;/*close due to add 3rd front camera,i2c write fail*/
            PK_INF(" Sensor found ID = 0x%x\n", sensorID);
            snprintf(mtk_ccm_name,sizeof(mtk_ccm_name),"%s CAM[%d]:%s;",mtk_ccm_name,g_invokeSocketIdx[i],g_invokeSensorNameStr[i]);
            err = ERROR_NONE;
        }
        if (ERROR_NONE != err)
        {
            PK_DBG("ERROR:adopt_CAMERA_HW_CheckIsAlive(), No imgsensor alive\n");
        }
        PK_ERR("[M6_CAM] check_alive result i=%u socket=%d name=%s err=%u sensorID=0x%08x retLen=%u\n",
            i, g_invokeSocketIdx[i], g_invokeSensorNameStr[i], err,
            sensorID, retLen);
        }
    }
    }
    else {
    PK_ERR("[M6_CAM] check_alive ERROR:NULL g_pSensorFunc\n");
    }

    /* reset sensor state after power off */
	if (g_pSensorFunc)
		err1 = g_pSensorFunc->SensorClose();
    if (ERROR_NONE != err1) {
    PK_DBG("SensorClose\n");
    }
    /*  */
    kdModulePowerOn((CAMERA_DUAL_CAMERA_SENSOR_ENUM *)g_invokeSocketIdx, g_invokeSensorNameStr, false, CAMERA_HW_DRVNAME1);
    PK_ERR("[M6_CAM] check_alive power_off done err=%u close_ret=%u\n", err, err1);
    /*  */
    KD_IMGSENSOR_PROFILE("CheckIsAlive");

    g_IsSearchSensor = 0;

    PK_ERR("[M6_CAM] check_alive exit ret=%d\n", err ? -EIO : err);
    return err ?  -EIO:err;
}   /* adopt_CAMERA_HW_Open() */


/*******************************************************************************
* adopt_CAMERA_HW_GetResolution
********************************************************************************/
inline static int adopt_CAMERA_HW_GetResolution(void *pBuf)
{
    /* ToDo: remove print */
    ACDK_SENSOR_PRESOLUTION_STRUCT *pBufResolution =  (ACDK_SENSOR_PRESOLUTION_STRUCT *)pBuf;
	ACDK_SENSOR_RESOLUTION_INFO_STRUCT* pRes[2] = { NULL, NULL };
    PK_XLOG_INFO("[CAMERA_HW] adopt_CAMERA_HW_GetResolution, pBuf: %p\n", pBuf);
	pRes[0] = (ACDK_SENSOR_RESOLUTION_INFO_STRUCT* )kmalloc(sizeof(MSDK_SENSOR_RESOLUTION_INFO_STRUCT), GFP_KERNEL);
	if (pRes[0] == NULL) {
		PK_ERR(" ioctl allocate mem failed\n");
		return -ENOMEM;
	}
	pRes[1] = (ACDK_SENSOR_RESOLUTION_INFO_STRUCT* )kmalloc(sizeof(MSDK_SENSOR_RESOLUTION_INFO_STRUCT), GFP_KERNEL);
	if (pRes[1] == NULL) {
		kfree(pRes[0]);
		PK_ERR(" ioctl allocate mem failed\n");
		return -ENOMEM;
	}


    if (g_pSensorFunc) {
		g_pSensorFunc->SensorGetResolution(pRes);
		if (copy_to_user((void __user *) (pBufResolution->pResolution[0]) , (void *)pRes[0] , sizeof(MSDK_SENSOR_RESOLUTION_INFO_STRUCT))) {
			PK_ERR("copy to user failed\n");
		}
		if (copy_to_user((void __user *) (pBufResolution->pResolution[1]) , (void *)pRes[1] , sizeof(MSDK_SENSOR_RESOLUTION_INFO_STRUCT))) {
			PK_ERR("copy to user failed\n");
		}
    }
    else {
    PK_DBG("[CAMERA_HW]ERROR:NULL g_pSensorFunc\n");
    }
	if (pRes[0] != NULL) {
		kfree(pRes[0]);
	}
	if (pRes[1] != NULL) {
		kfree(pRes[1]);
	}

    return 0;
}   /* adopt_CAMERA_HW_GetResolution() */


/*******************************************************************************
* adopt_CAMERA_HW_GetInfo
********************************************************************************/
inline static int adopt_CAMERA_HW_GetInfo(void *pBuf)
{
    ACDK_SENSOR_GETINFO_STRUCT *pSensorGetInfo = (ACDK_SENSOR_GETINFO_STRUCT *)pBuf;
    MSDK_SENSOR_INFO_STRUCT *pInfo[2];
    MSDK_SENSOR_CONFIG_STRUCT *pConfig[2];
    MUINT32 *pScenarioId[2];
    u32 i,j = 0;

    for (i = 0; i < 2; i++) {
	    pInfo[i] = NULL;
	    pConfig[i] =  NULL;
	    pScenarioId[i] =  &(pSensorGetInfo->ScenarioId[i]);
    }


	pInfo[0] = kmalloc(sizeof(MSDK_SENSOR_INFO_STRUCT), GFP_KERNEL);
	pInfo[1] = kmalloc(sizeof(MSDK_SENSOR_INFO_STRUCT), GFP_KERNEL);
	pConfig[0] = kmalloc(sizeof(MSDK_SENSOR_CONFIG_STRUCT), GFP_KERNEL);
	pConfig[1] = kmalloc(sizeof(MSDK_SENSOR_CONFIG_STRUCT), GFP_KERNEL);

	if (pInfo[0] == NULL || pInfo[1] == NULL || pConfig[0] == NULL || pConfig[1] == NULL){
		PK_ERR(" ioctl allocate mem failed\n");
		for (j = 0; j < 2; j++) {
			if (pInfo[j] != NULL)
				kfree(pInfo[j]);
			if (pConfig[j] != NULL)
				kfree(pConfig[j]);
		}
		return -ENOMEM;
	}

	memset(pInfo[0], 0, sizeof(MSDK_SENSOR_INFO_STRUCT));
	memset(pInfo[1], 0, sizeof(MSDK_SENSOR_INFO_STRUCT));
	memset(pConfig[0], 0, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	memset(pConfig[1], 0, sizeof(MSDK_SENSOR_CONFIG_STRUCT));

    if (NULL == pSensorGetInfo) {
    PK_DBG("[CAMERA_HW] NULL arg.\n");
	for (j = 0; j < 2; j++) {
		if (pInfo[j] != NULL)
			kfree(pInfo[j]);
		if (pConfig[j] != NULL)
			kfree(pConfig[j]);
	}
    return -EFAULT;
    }

    if ((NULL == pSensorGetInfo->pInfo[0]) || (NULL == pSensorGetInfo->pInfo[1]) ||
    (NULL == pSensorGetInfo->pConfig[0]) || (NULL == pSensorGetInfo->pConfig[1]))  {
    PK_DBG("[CAMERA_HW] NULL arg.\n");
	for (j = 0; j < 2; j++) {
		if (pInfo[j] != NULL)
			kfree(pInfo[j]);
		if (pConfig[j] != NULL)
			kfree(pConfig[j]);
	}
    return -EFAULT;
    }

    if (g_pSensorFunc) {
	    g_pSensorFunc->SensorGetInfo(pScenarioId, pInfo, pConfig);
    }
    else {
	    PK_DBG("[CAMERA_HW]ERROR:NULL g_pSensorFunc\n");
    }



    for (i = 0; i < 2; i++) {
	    /* SenorInfo */
	    if (copy_to_user((void __user *)(pSensorGetInfo->pInfo[i]), (void *)pInfo[i] , sizeof(MSDK_SENSOR_INFO_STRUCT))) {
	        PK_DBG("[CAMERA_HW][info] ioctl copy to user failed\n");
			for (j = 0; j < 2; j++) {
			  	if (pInfo[j] != NULL) kfree(pInfo[j]);
			  	if (pConfig[j] != NULL) kfree(pConfig[j]);
				pInfo[j] = NULL;
				pConfig[j] = NULL;
			}
	        return -EFAULT;
	    }

	    /* SensorConfig */
	    if (copy_to_user((void __user *) (pSensorGetInfo->pConfig[i]) , (void *)pConfig[i] , sizeof(MSDK_SENSOR_CONFIG_STRUCT))) {
	        PK_DBG("[CAMERA_HW][config] ioctl copy to user failed\n");
			for (j = 0; j < 2; j++) {
			  	if (pInfo[j] != NULL) kfree(pInfo[j]);
			  	if (pConfig[j] != NULL) kfree(pConfig[j]);
				pInfo[j] = NULL;
				pConfig[j] = NULL;
			}
	        return -EFAULT;
	    }
    }
	for (j = 0; j < 2; j++) {
		if (pInfo[j] != NULL) kfree(pInfo[j]);
		if (pConfig[j] != NULL) kfree(pConfig[j]);
		pInfo[j] = NULL;
		pConfig[j] = NULL;
	}

    return 0;
}   /* adopt_CAMERA_HW_GetInfo() */

/*******************************************************************************
* adopt_CAMERA_HW_GetInfo
********************************************************************************/
MSDK_SENSOR_INFO_STRUCT ginfo[2];
MSDK_SENSOR_INFO_STRUCT ginfo1[2];
MSDK_SENSOR_INFO_STRUCT ginfo2[2];
MSDK_SENSOR_INFO_STRUCT ginfo3[2];
MSDK_SENSOR_INFO_STRUCT ginfo4[2];
/* adopt_CAMERA_HW_GetInfo() */
inline static int adopt_CAMERA_HW_GetInfo2(void *pBuf)
{
    IMAGESENSOR_GETINFO_STRUCT *pSensorGetInfo = (IMAGESENSOR_GETINFO_STRUCT *)pBuf;
    ACDK_SENSOR_INFO2_STRUCT *pSensorInfo = NULL;//{0};
    MUINT32 IDNum = 0;
    MSDK_SENSOR_INFO_STRUCT *pInfo[2] = {NULL,NULL};
    MSDK_SENSOR_CONFIG_STRUCT  *pConfig[2] = {NULL,NULL};
    MSDK_SENSOR_INFO_STRUCT *pInfo1[2] = {NULL,NULL};
    MSDK_SENSOR_CONFIG_STRUCT  *pConfig1[2]= {NULL,NULL};
    MSDK_SENSOR_INFO_STRUCT *pInfo2[2] = {NULL,NULL};
    MSDK_SENSOR_CONFIG_STRUCT  *pConfig2[2]= {NULL,NULL};
    MSDK_SENSOR_INFO_STRUCT *pInfo3[2] = {NULL,NULL};
    MSDK_SENSOR_CONFIG_STRUCT  *pConfig3[2] = {NULL,NULL};
    MSDK_SENSOR_INFO_STRUCT *pInfo4[2] = {NULL,NULL};
    MSDK_SENSOR_CONFIG_STRUCT  *pConfig4[2]= {NULL,NULL};
    MSDK_SENSOR_RESOLUTION_INFO_STRUCT  *psensorResolution[2] = {NULL,NULL};

    MUINT32 ScenarioId[2], *pScenarioId[2];
    u32 i = 0;
    PK_DBG("[adopt_CAMERA_HW_GetInfo2]Entry\n");


    if (NULL == pSensorGetInfo) {
    PK_DBG("[CAMERA_HW] NULL arg.\n");
    return -EFAULT;
    }
    if (NULL == g_pSensorFunc) {
    PK_DBG("[CAMERA_HW]ERROR:NULL g_pSensorFunc\n");
    return -EFAULT;
    }

	for (i = 0; i < 2; i++) {
	   pInfo[i] =  &ginfo[i];
	pInfo[i]->SensorHorFOV = 0;
	pInfo[i]->SensorVerFOV = 0;
	   pConfig[i] =  kmalloc(sizeof(MSDK_SENSOR_CONFIG_STRUCT), GFP_KERNEL);
	   pInfo1[i] =	&ginfo1[i];
	   pConfig1[i] =  kmalloc(sizeof(MSDK_SENSOR_CONFIG_STRUCT), GFP_KERNEL);
	   pInfo2[i] =	&ginfo2[i];
	   pConfig2[i] = kmalloc(sizeof(MSDK_SENSOR_CONFIG_STRUCT), GFP_KERNEL);
	   pInfo3[i] =	&ginfo3[i];
	   pConfig3[i] =  kmalloc(sizeof(MSDK_SENSOR_CONFIG_STRUCT), GFP_KERNEL);
	   pInfo4[i] =	&ginfo4[i];
	   pConfig4[i] =  kmalloc(sizeof(MSDK_SENSOR_CONFIG_STRUCT), GFP_KERNEL);
	   psensorResolution[i] = kmalloc(sizeof(MSDK_SENSOR_RESOLUTION_INFO_STRUCT), GFP_KERNEL);
	   pScenarioId[i] =  &ScenarioId[i];
	}
	pSensorInfo = kmalloc(sizeof(ACDK_SENSOR_INFO2_STRUCT), GFP_KERNEL);

	if (pConfig[0] == NULL || pConfig[1] == NULL || pConfig1[0] == NULL || pConfig1[1] == NULL ||
		pConfig2[0] == NULL || pConfig2[1] == NULL || pConfig3[0] == NULL || pConfig3[1] == NULL ||
		pConfig4[0] == NULL || pConfig4[1] == NULL || pSensorInfo ==NULL ||
		psensorResolution[0] == NULL || psensorResolution[1]==NULL) {
		PK_ERR(" ioctl allocate mem failed\n");
		for (i = 0; i < 2; i++) {
			if(pConfig[i] != NULL) kfree(pConfig[i]);
			if(pConfig1[i] != NULL) kfree(pConfig1[i]);
			if(pConfig2[i] != NULL) kfree(pConfig2[i]);
			if(pConfig3[i] != NULL) kfree(pConfig3[i]);
			if(pConfig4[i] != NULL) kfree(pConfig4[i]);
			if(psensorResolution[i] != NULL) kfree(psensorResolution[i]);
			pConfig[i] = NULL;
			pConfig1[i] = NULL;
			pConfig2[i] = NULL;
			pConfig3[i] =  NULL;
			pConfig4[i] =  NULL;
			psensorResolution[i] = NULL;
		}
		kfree(pSensorInfo);
		pSensorInfo = NULL;
		return -ENOMEM;
	}

    PK_DBG("[CAMERA_HW][Resolution] %p\n", pSensorGetInfo->pSensorResolution);

    /* TO get preview value */
    ScenarioId[0] = ScenarioId[1] = MSDK_SCENARIO_ID_CAMERA_PREVIEW;
    g_pSensorFunc->SensorGetInfo(pScenarioId, pInfo, pConfig);
    /*  */
    ScenarioId[0] = ScenarioId[1] = MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG;
    g_pSensorFunc->SensorGetInfo(pScenarioId, pInfo1, pConfig1);
    /*  */
    ScenarioId[0] = ScenarioId[1] = MSDK_SCENARIO_ID_VIDEO_PREVIEW;
    g_pSensorFunc->SensorGetInfo(pScenarioId, pInfo2, pConfig2);
    /*  */
    ScenarioId[0] = ScenarioId[1] = MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO;
    g_pSensorFunc->SensorGetInfo(pScenarioId, pInfo3, pConfig3);
    /*  */
    ScenarioId[0] = ScenarioId[1] = MSDK_SCENARIO_ID_SLIM_VIDEO;
    g_pSensorFunc->SensorGetInfo(pScenarioId, pInfo4, pConfig4);
    /* To set sensor information */
    if (DUAL_CAMERA_MAIN_SENSOR == pSensorGetInfo->SensorId) {
    IDNum = 0;
    }
    else
    {
    IDNum = 1;
    }
    /* Basic information */
    pSensorInfo->SensorPreviewResolutionX                 = pInfo[IDNum]->SensorPreviewResolutionX;
    pSensorInfo->SensorPreviewResolutionY                 = pInfo[IDNum]->SensorPreviewResolutionY;
    pSensorInfo->SensorFullResolutionX                    = pInfo[IDNum]->SensorFullResolutionX;
    pSensorInfo->SensorFullResolutionY                    = pInfo[IDNum]->SensorFullResolutionY;
    pSensorInfo->SensorClockFreq                          = pInfo[IDNum]->SensorClockFreq;
    pSensorInfo->SensorCameraPreviewFrameRate             = pInfo[IDNum]->SensorCameraPreviewFrameRate;
    pSensorInfo->SensorVideoFrameRate                     = pInfo[IDNum]->SensorVideoFrameRate;
    pSensorInfo->SensorStillCaptureFrameRate              = pInfo[IDNum]->SensorStillCaptureFrameRate;
    pSensorInfo->SensorWebCamCaptureFrameRate             = pInfo[IDNum]->SensorWebCamCaptureFrameRate;
    pSensorInfo->SensorClockPolarity                      = pInfo[IDNum]->SensorClockPolarity;
    pSensorInfo->SensorClockFallingPolarity               = pInfo[IDNum]->SensorClockFallingPolarity;
    pSensorInfo->SensorClockRisingCount                   = pInfo[IDNum]->SensorClockRisingCount;
    pSensorInfo->SensorClockFallingCount                  = pInfo[IDNum]->SensorClockFallingCount;
    pSensorInfo->SensorClockDividCount                    = pInfo[IDNum]->SensorClockDividCount;
    pSensorInfo->SensorPixelClockCount                    = pInfo[IDNum]->SensorPixelClockCount;
    pSensorInfo->SensorDataLatchCount                     = pInfo[IDNum]->SensorDataLatchCount;
    pSensorInfo->SensorHsyncPolarity                      = pInfo[IDNum]->SensorHsyncPolarity;
    pSensorInfo->SensorVsyncPolarity                      = pInfo[IDNum]->SensorVsyncPolarity;
    pSensorInfo->SensorInterruptDelayLines                = pInfo[IDNum]->SensorInterruptDelayLines;
    pSensorInfo->SensorResetActiveHigh                    = pInfo[IDNum]->SensorResetActiveHigh;
    pSensorInfo->SensorResetDelayCount                    = pInfo[IDNum]->SensorResetDelayCount;
    pSensorInfo->SensroInterfaceType                      = pInfo[IDNum]->SensroInterfaceType;
    pSensorInfo->SensorOutputDataFormat                   = pInfo[IDNum]->SensorOutputDataFormat;
    pSensorInfo->SensorMIPILaneNumber                     = pInfo[IDNum]->SensorMIPILaneNumber;
    pSensorInfo->CaptureDelayFrame                        = pInfo[IDNum]->CaptureDelayFrame;
    pSensorInfo->PreviewDelayFrame                        = pInfo[IDNum]->PreviewDelayFrame;
    pSensorInfo->VideoDelayFrame                          = pInfo[IDNum]->VideoDelayFrame;
    pSensorInfo->HighSpeedVideoDelayFrame                 = pInfo[IDNum]->HighSpeedVideoDelayFrame;
    pSensorInfo->SlimVideoDelayFrame                      = pInfo[IDNum]->SlimVideoDelayFrame;
    pSensorInfo->Custom1DelayFrame                        = pInfo[IDNum]->Custom1DelayFrame;
    pSensorInfo->Custom2DelayFrame                        = pInfo[IDNum]->Custom2DelayFrame;
    pSensorInfo->Custom3DelayFrame                        = pInfo[IDNum]->Custom3DelayFrame;
    pSensorInfo->Custom4DelayFrame                        = pInfo[IDNum]->Custom4DelayFrame;
    pSensorInfo->Custom5DelayFrame                        = pInfo[IDNum]->Custom5DelayFrame;
    pSensorInfo->YUVAwbDelayFrame                         = pInfo[IDNum]->YUVAwbDelayFrame;
    pSensorInfo->YUVEffectDelayFrame                      = pInfo[IDNum]->YUVEffectDelayFrame;
    pSensorInfo->SensorGrabStartX_PRV                     = pInfo[IDNum]->SensorGrabStartX;
    pSensorInfo->SensorGrabStartY_PRV                     = pInfo[IDNum]->SensorGrabStartY;
    pSensorInfo->SensorGrabStartX_CAP                     = pInfo1[IDNum]->SensorGrabStartX;
    pSensorInfo->SensorGrabStartY_CAP                     = pInfo1[IDNum]->SensorGrabStartY;
    pSensorInfo->SensorGrabStartX_VD                      = pInfo2[IDNum]->SensorGrabStartX;
    pSensorInfo->SensorGrabStartY_VD                      = pInfo2[IDNum]->SensorGrabStartY;
    pSensorInfo->SensorGrabStartX_VD1                     = pInfo3[IDNum]->SensorGrabStartX;
    pSensorInfo->SensorGrabStartY_VD1                     = pInfo3[IDNum]->SensorGrabStartY;
    pSensorInfo->SensorGrabStartX_VD2                     = pInfo4[IDNum]->SensorGrabStartX;
    pSensorInfo->SensorGrabStartY_VD2                     = pInfo4[IDNum]->SensorGrabStartY;
    pSensorInfo->SensorDrivingCurrent                     = pInfo[IDNum]->SensorDrivingCurrent;
    pSensorInfo->SensorMasterClockSwitch                  = pInfo[IDNum]->SensorMasterClockSwitch;
    pSensorInfo->AEShutDelayFrame                         = pInfo[IDNum]->AEShutDelayFrame;
    pSensorInfo->AESensorGainDelayFrame                   = pInfo[IDNum]->AESensorGainDelayFrame;
    pSensorInfo->AEISPGainDelayFrame                      = pInfo[IDNum]->AEISPGainDelayFrame;
    pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount   = pInfo[IDNum]->MIPIDataLowPwr2HighSpeedTermDelayCount;
    pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = pInfo[IDNum]->MIPIDataLowPwr2HighSpeedSettleDelayCount;
	pSensorInfo->MIPIDataLowPwr2HSSettleDelayM0           = pInfo[IDNum]->MIPIDataLowPwr2HighSpeedSettleDelayCount;
	pSensorInfo->MIPIDataLowPwr2HSSettleDelayM1           = pInfo1[IDNum]->MIPIDataLowPwr2HighSpeedSettleDelayCount;
	pSensorInfo->MIPIDataLowPwr2HSSettleDelayM2           = pInfo2[IDNum]->MIPIDataLowPwr2HighSpeedSettleDelayCount;
	pSensorInfo->MIPIDataLowPwr2HSSettleDelayM3           = pInfo3[IDNum]->MIPIDataLowPwr2HighSpeedSettleDelayCount;
	pSensorInfo->MIPIDataLowPwr2HSSettleDelayM4           = pInfo4[IDNum]->MIPIDataLowPwr2HighSpeedSettleDelayCount;
    pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount    = pInfo[IDNum]->MIPICLKLowPwr2HighSpeedTermDelayCount;
    pSensorInfo->SensorWidthSampling                      = pInfo[IDNum]->SensorWidthSampling;
    pSensorInfo->SensorHightSampling                      = pInfo[IDNum]->SensorHightSampling;
    pSensorInfo->SensorPacketECCOrder                     = pInfo[IDNum]->SensorPacketECCOrder;
    pSensorInfo->MIPIsensorType                           = pInfo[IDNum]->MIPIsensorType;
    pSensorInfo->IHDR_LE_FirstLine                        = pInfo[IDNum]->IHDR_LE_FirstLine;
    pSensorInfo->IHDR_Support                             = pInfo[IDNum]->IHDR_Support;
    pSensorInfo->SensorModeNum                            = pInfo[IDNum]->SensorModeNum;
    pSensorInfo->SettleDelayMode                          = pInfo[IDNum]->SettleDelayMode;
    pSensorInfo->PDAF_Support                             = pInfo[IDNum]->PDAF_Support;
	pSensorInfo->IMGSENSOR_DPCM_TYPE_PRE                  = pInfo[IDNum]->DPCM_INFO;
	pSensorInfo->IMGSENSOR_DPCM_TYPE_CAP                  = pInfo1[IDNum]->DPCM_INFO;
	pSensorInfo->IMGSENSOR_DPCM_TYPE_VD                   = pInfo2[IDNum]->DPCM_INFO;
	pSensorInfo->IMGSENSOR_DPCM_TYPE_VD1                  = pInfo3[IDNum]->DPCM_INFO;
	pSensorInfo->IMGSENSOR_DPCM_TYPE_VD2                  = pInfo4[IDNum]->DPCM_INFO;
	/*Per-Frame conrol suppport or not */
	pSensorInfo->PerFrameCTL_Support                      = pInfo[IDNum]->PerFrameCTL_Support;
	/*SCAM number*/
	pSensorInfo->SCAM_DataNumber                          = pInfo[IDNum]->SCAM_DataNumber;
	pSensorInfo->SCAM_DDR_En                              = pInfo[IDNum]->SCAM_DDR_En;
	pSensorInfo->SCAM_CLK_INV                             = pInfo[IDNum]->SCAM_CLK_INV;
	pSensorInfo->SCAM_DEFAULT_DELAY                      = pInfo[IDNum]->SCAM_DEFAULT_DELAY;
	pSensorInfo->SCAM_CRC_En                             = pInfo[IDNum]->SCAM_CRC_En;
	pSensorInfo->SCAM_SOF_src                            = pInfo[IDNum]->SCAM_SOF_src;
	pSensorInfo->SCAM_Timout_Cali                        = pInfo[IDNum]->SCAM_Timout_Cali;
	pSensorInfo->SensorHorFOV                            = pInfo[IDNum]->SensorHorFOV;
	pSensorInfo->SensorVerFOV                            = pInfo[IDNum]->SensorVerFOV;

    /* TO get preview value */
    ScenarioId[0] = ScenarioId[1] = MSDK_SCENARIO_ID_CUSTOM1;
    g_pSensorFunc->SensorGetInfo(pScenarioId, pInfo, pConfig);
    /*  */
    ScenarioId[0] = ScenarioId[1] = MSDK_SCENARIO_ID_CUSTOM2;
    g_pSensorFunc->SensorGetInfo(pScenarioId, pInfo1, pConfig1);
    /*  */
    ScenarioId[0] = ScenarioId[1] = MSDK_SCENARIO_ID_CUSTOM3;
    g_pSensorFunc->SensorGetInfo(pScenarioId, pInfo2, pConfig2);
    /*  */
    ScenarioId[0] = ScenarioId[1] = MSDK_SCENARIO_ID_CUSTOM4;
    g_pSensorFunc->SensorGetInfo(pScenarioId, pInfo3, pConfig3);
    /*  */
    ScenarioId[0] = ScenarioId[1] = MSDK_SCENARIO_ID_CUSTOM5;
    g_pSensorFunc->SensorGetInfo(pScenarioId, pInfo4, pConfig4);
    /* To set sensor information */
    if (DUAL_CAMERA_MAIN_SENSOR == pSensorGetInfo->SensorId) {
    IDNum = 0;
    }
    else
    {
    IDNum = 1;
    }
    pSensorInfo->SensorGrabStartX_CST1                    = pInfo[IDNum]->SensorGrabStartX;
    pSensorInfo->SensorGrabStartY_CST1                    = pInfo[IDNum]->SensorGrabStartY;
    pSensorInfo->SensorGrabStartX_CST2                    = pInfo1[IDNum]->SensorGrabStartX;
    pSensorInfo->SensorGrabStartY_CST2                    = pInfo1[IDNum]->SensorGrabStartY;
    pSensorInfo->SensorGrabStartX_CST3                    = pInfo2[IDNum]->SensorGrabStartX;
    pSensorInfo->SensorGrabStartY_CST3                    = pInfo2[IDNum]->SensorGrabStartY;
    pSensorInfo->SensorGrabStartX_CST4                    = pInfo3[IDNum]->SensorGrabStartX;
    pSensorInfo->SensorGrabStartY_CST4                    = pInfo3[IDNum]->SensorGrabStartY;
    pSensorInfo->SensorGrabStartX_CST5                    = pInfo4[IDNum]->SensorGrabStartX;
    pSensorInfo->SensorGrabStartY_CST5                    = pInfo4[IDNum]->SensorGrabStartY;

    if (copy_to_user((void __user *)(pSensorGetInfo->pInfo), (void *)(pSensorInfo), sizeof(ACDK_SENSOR_INFO2_STRUCT))) {
	    PK_DBG("[CAMERA_HW][info] ioctl copy to user failed\n");
		for (i = 0; i < 2; i++) {
			if(pConfig[i] != NULL) kfree(pConfig[i]);
			if(pConfig1[i] != NULL) kfree(pConfig1[i]);
			if(pConfig2[i] != NULL) kfree(pConfig2[i]);
			if(pConfig3[i] != NULL) kfree(pConfig3[i]);
			if(pConfig4[i] != NULL) kfree(pConfig4[i]);
			if(psensorResolution[i] != NULL) kfree(psensorResolution[i]);
			pConfig[i] = NULL;
			pConfig1[i] = NULL;
			pConfig2[i] = NULL;
			pConfig3[i] =  NULL;
			pConfig4[i] =  NULL;
			psensorResolution[i] = NULL;
		}
   		kfree(pSensorInfo);
		pSensorInfo = NULL;

    return -EFAULT;
    }

    /* Step2 : Get Resolution */
    g_pSensorFunc->SensorGetResolution(psensorResolution);
    PK_DBG("[CAMERA_HW][Pre]w=0x%x, h = 0x%x\n", psensorResolution[0]->SensorPreviewWidth, psensorResolution[0]->SensorPreviewHeight);
    PK_DBG("[CAMERA_HW][Full]w=0x%x, h = 0x%x\n", psensorResolution[0]->SensorFullWidth, psensorResolution[0]->SensorFullHeight);
    PK_DBG("[CAMERA_HW][VD]w=0x%x, h = 0x%x\n", psensorResolution[0]->SensorVideoWidth, psensorResolution[0]->SensorVideoHeight);

    if (DUAL_CAMERA_MAIN_SENSOR == pSensorGetInfo->SensorId) {
    /* Resolution */
    PK_DBG("[adopt_CAMERA_HW_GetInfo2]Resolution\n");
    if (copy_to_user((void __user *) (pSensorGetInfo->pSensorResolution) , (void *)psensorResolution[0] , sizeof(MSDK_SENSOR_RESOLUTION_INFO_STRUCT))) {
       PK_DBG("[CAMERA_HW][Resolution] ioctl copy to user failed\n");
	   for (i = 0; i < 2; i++) {
		   if(pConfig[i] != NULL) kfree(pConfig[i]);
		   if(pConfig1[i] != NULL) kfree(pConfig1[i]);
		   if(pConfig2[i] != NULL) kfree(pConfig2[i]);
		   if(pConfig3[i] != NULL) kfree(pConfig3[i]);
		   if(pConfig4[i] != NULL) kfree(pConfig4[i]);
		   if(psensorResolution[i] != NULL) kfree(psensorResolution[i]);
		   pConfig[i] = NULL;
		   pConfig1[i] = NULL;
		   pConfig2[i] = NULL;
		   pConfig3[i] =  NULL;
		   pConfig4[i] =  NULL;
		   psensorResolution[i] = NULL;
	   }
   		kfree(pSensorInfo);
		pSensorInfo = NULL;
       return -EFAULT;
    }
    } else{
     /* Resolution */
     if (copy_to_user((void __user *) (pSensorGetInfo->pSensorResolution) , (void *)psensorResolution[1] , sizeof(MSDK_SENSOR_RESOLUTION_INFO_STRUCT))) {
        PK_DBG("[CAMERA_HW][Resolution] ioctl copy to user failed\n");
		for (i = 0; i < 2; i++) {
			if(pConfig[i] != NULL) kfree(pConfig[i]);
			if(pConfig1[i] != NULL) kfree(pConfig1[i]);
			if(pConfig2[i] != NULL) kfree(pConfig2[i]);
			if(pConfig3[i] != NULL) kfree(pConfig3[i]);
			if(pConfig4[i] != NULL) kfree(pConfig4[i]);
			if(psensorResolution[i] != NULL) kfree(psensorResolution[i]);
			pConfig[i] = NULL;
			pConfig1[i] = NULL;
			pConfig2[i] = NULL;
			pConfig3[i] =  NULL;
			pConfig4[i] =  NULL;
			psensorResolution[i] = NULL;
		}
			kfree(pSensorInfo);
		pSensorInfo = NULL;
        return -EFAULT;
    }
    }

	for (i = 0; i < 2; i++) {
		if(pConfig[i] != NULL) kfree(pConfig[i]);
		if(pConfig1[i] != NULL) kfree(pConfig1[i]);
		if(pConfig2[i] != NULL) kfree(pConfig2[i]);
		if(pConfig3[i] != NULL) kfree(pConfig3[i]);
		if(pConfig4[i] != NULL) kfree(pConfig4[i]);
		if(psensorResolution[i] != NULL) kfree(psensorResolution[i]);
		   pConfig[i] = NULL;
		   pConfig1[i] = NULL;
		   pConfig2[i] = NULL;
		   pConfig3[i] =  NULL;
		   pConfig4[i] =  NULL;
		   psensorResolution[i] = NULL;
	}
	kfree(pSensorInfo);
	pSensorInfo = NULL;

    return 0;
}   /* adopt_CAMERA_HW_GetInfo() */


/*******************************************************************************
* adopt_CAMERA_HW_Control
********************************************************************************/
inline static int adopt_CAMERA_HW_Control(void *pBuf)
{
    int ret = 0;
    ACDK_SENSOR_CONTROL_STRUCT *pSensorCtrl = (ACDK_SENSOR_CONTROL_STRUCT *)pBuf;
    MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT imageWindow;
    MSDK_SENSOR_CONFIG_STRUCT sensorConfigData;
    memset(&imageWindow, 0, sizeof(ACDK_SENSOR_EXPOSURE_WINDOW_STRUCT));
    memset(&sensorConfigData, 0, sizeof(ACDK_SENSOR_CONFIG_STRUCT));

    if (NULL == pSensorCtrl) {
    PK_DBG("[CAMERA_HW] NULL arg.\n");
    return -EFAULT;
    }

    if (NULL == pSensorCtrl->pImageWindow || NULL == pSensorCtrl->pSensorConfigData) {
    PK_DBG("[CAMERA_HW] NULL arg.\n");
    return -EFAULT;
    }

    if (copy_from_user((void *)&imageWindow , (void *) pSensorCtrl->pImageWindow, sizeof(ACDK_SENSOR_EXPOSURE_WINDOW_STRUCT))) {
    PK_DBG("[CAMERA_HW][pFeatureData32] ioctl copy from user failed\n");
    return -EFAULT;
    }

    if (copy_from_user((void *)&sensorConfigData , (void *) pSensorCtrl->pSensorConfigData, sizeof(ACDK_SENSOR_CONFIG_STRUCT))) {
    PK_DBG("[CAMERA_HW][pFeatureData32] ioctl copy from user failed\n");
    return -EFAULT;
    }

    /*  */
    if (g_pSensorFunc) {
    ret = g_pSensorFunc->SensorControl(pSensorCtrl->InvokeCamera, pSensorCtrl->ScenarioId, &imageWindow, &sensorConfigData);
    }
    else {
    PK_DBG("[CAMERA_HW]ERROR:NULL g_pSensorFunc\n");
    }

    /*  */
    if (copy_to_user((void __user *) pSensorCtrl->pImageWindow, (void *)&imageWindow , sizeof(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT))) {
    PK_DBG("[CAMERA_HW][imageWindow] ioctl copy to user failed\n");
    return -EFAULT;
    }

    /*  */
    if (copy_to_user((void __user *) pSensorCtrl->pSensorConfigData, (void *)&sensorConfigData , sizeof(MSDK_SENSOR_CONFIG_STRUCT))) {
    PK_DBG("[CAMERA_HW][imageWindow] ioctl copy to user failed\n");
    return -EFAULT;
    }
    return ret;
} /* adopt_CAMERA_HW_Control */

/*******************************************************************************
* adopt_CAMERA_HW_FeatureControl
********************************************************************************/
inline static int  adopt_CAMERA_HW_FeatureControl(void *pBuf)
{
    ACDK_SENSOR_FEATURECONTROL_STRUCT *pFeatureCtrl = (ACDK_SENSOR_FEATURECONTROL_STRUCT *)pBuf;
    unsigned int FeatureParaLen = 0;
    void *pFeaturePara = NULL;
#if 0
    ACDK_SENSOR_GROUP_INFO_STRUCT *pSensorGroupInfo = NULL;
    char kernelGroupNamePtr[128];
    unsigned char *pUserGroupNamePtr = NULL;
#endif
	ACDK_KD_SENSOR_SYNC_STRUCT *pSensorSyncInfo = NULL;
    signed int ret = 0;



    if (NULL == pFeatureCtrl) {
    PK_ERR(" NULL arg.\n");
    return -EFAULT;
    }

    if (SENSOR_FEATURE_SINGLE_FOCUS_MODE == pFeatureCtrl->FeatureId || SENSOR_FEATURE_CANCEL_AF == pFeatureCtrl->FeatureId
    || SENSOR_FEATURE_CONSTANT_AF == pFeatureCtrl->FeatureId || SENSOR_FEATURE_INFINITY_AF == pFeatureCtrl->FeatureId) {/* YUV AF_init and AF_constent and AF_single has no params */
    }
    else
    {
		if (NULL == pFeatureCtrl->pFeaturePara || NULL == pFeatureCtrl->pFeatureParaLen) {
			PK_ERR(" NULL arg.\n");
			return -EFAULT;
		}
		if (copy_from_user((void *)&FeatureParaLen , (void *) pFeatureCtrl->pFeatureParaLen, sizeof(unsigned int))) {
			PK_ERR(" ioctl copy from user failed\n");
			return -EFAULT;
		}

		/* data size exam */
		if (FeatureParaLen > FEATURE_CONTROL_MAX_DATA_SIZE) {
			PK_ERR(" exceed data size limitation\n");
			return -EFAULT;
		}

		pFeaturePara = kmalloc(FeatureParaLen, GFP_KERNEL);
		if (NULL == pFeaturePara) {
			PK_ERR(" ioctl allocate mem failed\n");
			return -ENOMEM;
		}
		memset(pFeaturePara, 0x0, FeatureParaLen);
    }

    /* copy from user */
    switch (pFeatureCtrl->FeatureId)
    {
    case SENSOR_FEATURE_SET_ESHUTTER:
    case SENSOR_FEATURE_SET_GAIN:
        /* reset the delay frame flag */
        spin_lock(&kdsensor_drv_lock);
        g_NewSensorExpGain.uSensorExpDelayFrame = 0xFF;
        g_NewSensorExpGain.uSensorGainDelayFrame = 0xFF;
        g_NewSensorExpGain.uISPGainDelayFrame = 0xFF;
        spin_unlock(&kdsensor_drv_lock);
    case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
    case SENSOR_FEATURE_SET_REGISTER:
    case SENSOR_FEATURE_GET_REGISTER:
    case SENSOR_FEATURE_SET_CCT_REGISTER:
    case SENSOR_FEATURE_SET_ENG_REGISTER:
    case SENSOR_FEATURE_SET_ITEM_INFO:
    case SENSOR_FEATURE_GET_ITEM_INFO:
    case SENSOR_FEATURE_GET_ENG_INFO:
    case SENSOR_FEATURE_SET_VIDEO_MODE:
    case SENSOR_FEATURE_SET_YUV_CMD:
    case SENSOR_FEATURE_MOVE_FOCUS_LENS:
    case SENSOR_FEATURE_SET_AF_WINDOW:
    case SENSOR_FEATURE_SET_CALIBRATION_DATA:
    case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
    case SENSOR_FEATURE_GET_EV_AWB_REF:
    case SENSOR_FEATURE_GET_SHUTTER_GAIN_AWB_GAIN:
    case SENSOR_FEATURE_SET_AE_WINDOW:
    case SENSOR_FEATURE_GET_EXIF_INFO:
    case SENSOR_FEATURE_GET_DELAY_INFO:
    case SENSOR_FEATURE_GET_AE_AWB_LOCK_INFO:
    case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
    case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
    case SENSOR_FEATURE_SET_TEST_PATTERN:
    case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
    case SENSOR_FEATURE_SET_OB_LOCK:
    case SENSOR_FEATURE_SET_SENSOR_OTP_AWB_CMD:
    case SENSOR_FEATURE_SET_SENSOR_OTP_LSC_CMD:
    case SENSOR_FEATURE_GET_TEMPERATURE_VALUE:
    case SENSOR_FEATURE_SET_FRAMERATE:
    case SENSOR_FEATURE_SET_HDR:
    case SENSOR_FEATURE_GET_CROP_INFO:
    case SENSOR_FEATURE_GET_VC_INFO:
    case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
	case SENSOR_FEATURE_SET_HDR_SHUTTER:
    case SENSOR_FEATURE_GET_AE_FLASHLIGHT_INFO:
    case SENSOR_FEATURE_GET_TRIGGER_FLASHLIGHT_INFO: /* return TRUE:play flashlight */
    case SENSOR_FEATURE_SET_YUV_3A_CMD: /* para: ACDK_SENSOR_3A_LOCK_ENUM */
    case SENSOR_FEATURE_SET_AWB_GAIN:
    case SENSOR_FEATURE_SET_MIN_MAX_FPS:
    case SENSOR_FEATURE_GET_PDAF_INFO:
    case SENSOR_FEATURE_GET_PDAF_DATA:
    case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
    case SENSOR_FEATURE_SET_PDAF:
        /*  */
        if (copy_from_user((void *)pFeaturePara , (void *) pFeatureCtrl->pFeaturePara, FeatureParaLen)) {
        kfree(pFeaturePara);
        PK_ERR("[CAMERA_HW][pFeaturePara] ioctl copy from user failed\n");
        return -EFAULT;
        }
        break;
     case SENSOR_FEATURE_SET_SENSOR_SYNC:    /* Update new sensor exposure time and gain to keep */
        if (copy_from_user((void *)pFeaturePara , (void *) pFeatureCtrl->pFeaturePara, FeatureParaLen)) {
	 kfree(pFeaturePara);
         PK_ERR("[CAMERA_HW][pFeaturePara] ioctl copy from user failed\n");
         return -EFAULT;
    }
    /* keep the information to wait Vsync synchronize */
    pSensorSyncInfo = (ACDK_KD_SENSOR_SYNC_STRUCT *)pFeaturePara;
    spin_lock(&kdsensor_drv_lock);
    g_NewSensorExpGain.u2SensorNewExpTime = pSensorSyncInfo->u2SensorNewExpTime;
    g_NewSensorExpGain.u2SensorNewGain = pSensorSyncInfo->u2SensorNewGain;
    g_NewSensorExpGain.u2ISPNewRGain = pSensorSyncInfo->u2ISPNewRGain;
    g_NewSensorExpGain.u2ISPNewGrGain = pSensorSyncInfo->u2ISPNewGrGain;
    g_NewSensorExpGain.u2ISPNewGbGain = pSensorSyncInfo->u2ISPNewGbGain;
    g_NewSensorExpGain.u2ISPNewBGain = pSensorSyncInfo->u2ISPNewBGain;
    g_NewSensorExpGain.uSensorExpDelayFrame = pSensorSyncInfo->uSensorExpDelayFrame;
    g_NewSensorExpGain.uSensorGainDelayFrame = pSensorSyncInfo->uSensorGainDelayFrame;
    g_NewSensorExpGain.uISPGainDelayFrame = pSensorSyncInfo->uISPGainDelayFrame;
    /* AE smooth not change shutter to speed up */
    if ((0 == g_NewSensorExpGain.u2SensorNewExpTime) || (0xFFFF == g_NewSensorExpGain.u2SensorNewExpTime)) {
        g_NewSensorExpGain.uSensorExpDelayFrame = 0xFF;
    }

    if (g_NewSensorExpGain.uSensorExpDelayFrame == 0) {
        FeatureParaLen = 2;
        g_pSensorFunc->SensorFeatureControl(pFeatureCtrl->InvokeCamera, SENSOR_FEATURE_SET_ESHUTTER, (unsigned char *)&g_NewSensorExpGain.u2SensorNewExpTime, (unsigned int *) &FeatureParaLen);
        g_NewSensorExpGain.uSensorExpDelayFrame = 0xFF; /* disable */
    }
    else if (g_NewSensorExpGain.uSensorExpDelayFrame != 0xFF) {
        g_NewSensorExpGain.uSensorExpDelayFrame--;
    }
    /* exposure gain */
    if (g_NewSensorExpGain.uSensorGainDelayFrame == 0) {
        FeatureParaLen = 2;
        g_pSensorFunc->SensorFeatureControl(pFeatureCtrl->InvokeCamera, SENSOR_FEATURE_SET_GAIN, (unsigned char *)&g_NewSensorExpGain.u2SensorNewGain, (unsigned int *) &FeatureParaLen);
        g_NewSensorExpGain.uSensorGainDelayFrame = 0xFF; /* disable */
    }
    else if (g_NewSensorExpGain.uSensorGainDelayFrame != 0xFF) {
        g_NewSensorExpGain.uSensorGainDelayFrame--;
    }
    /* if the delay frame is 0 or 0xFF, stop to count */
    if ((g_NewSensorExpGain.uISPGainDelayFrame != 0xFF) && (g_NewSensorExpGain.uISPGainDelayFrame != 0)) {
        g_NewSensorExpGain.uISPGainDelayFrame--;
    }
	spin_unlock(&kdsensor_drv_lock);



     break;
#if 0
    case SENSOR_FEATURE_GET_GROUP_INFO:
        if (copy_from_user((void *)pFeaturePara , (void *) pFeatureCtrl->pFeaturePara, FeatureParaLen)) {
        kfree(pFeaturePara);
        PK_DBG("[CAMERA_HW][pFeaturePara] ioctl copy from user failed\n");
        return -EFAULT;
        }
        pSensorGroupInfo = (ACDK_SENSOR_GROUP_INFO_STRUCT *)pFeaturePara;
        pUserGroupNamePtr = pSensorGroupInfo->GroupNamePtr;
        /*  */
        if (NULL == pUserGroupNamePtr) {
        kfree(pFeaturePara);
        PK_DBG("[CAMERA_HW] NULL arg.\n");
        return -EFAULT;
        }
        pSensorGroupInfo->GroupNamePtr = kernelGroupNamePtr;
        break;
#endif
    case SENSOR_FEATURE_SET_ESHUTTER_GAIN:
        if (copy_from_user((void *)pFeaturePara , (void *) pFeatureCtrl->pFeaturePara, FeatureParaLen)) {
        PK_ERR("[CAMERA_HW][pFeaturePara] ioctl copy from user failed\n");
		kfree(pFeaturePara);
        return -EFAULT;
        }
        /* keep the information to wait Vsync synchronize */
        pSensorSyncInfo = (ACDK_KD_SENSOR_SYNC_STRUCT *)pFeaturePara;
        spin_lock(&kdsensor_drv_lock);
        g_NewSensorExpGain.u2SensorNewExpTime = pSensorSyncInfo->u2SensorNewExpTime;
        g_NewSensorExpGain.u2SensorNewGain = pSensorSyncInfo->u2SensorNewGain;
        spin_unlock(&kdsensor_drv_lock);
        kdSetExpGain(pFeatureCtrl->InvokeCamera);
        break;
    /* copy to user */
    case SENSOR_FEATURE_GET_RESOLUTION:
    case SENSOR_FEATURE_GET_PERIOD:
    case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
    case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
    case SENSOR_FEATURE_GET_CONFIG_PARA:
    case SENSOR_FEATURE_GET_GROUP_COUNT:
    case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
    /* do nothing */
    case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
    case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
    case SENSOR_FEATURE_SINGLE_FOCUS_MODE:
    case SENSOR_FEATURE_CANCEL_AF:
    case SENSOR_FEATURE_CONSTANT_AF:
    default:
        break;
    }

	/*in case that some structure are passed from user sapce by ptr */
	switch (pFeatureCtrl->FeatureId) {
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
		{
			MUINT32 *pValue = NULL;
			unsigned long long *pFeaturePara_64 = (unsigned long long *)pFeaturePara;
			pValue = kmalloc(sizeof(MUINT32), GFP_KERNEL);
			if (pValue == NULL) {
				PK_ERR(" ioctl allocate mem failed\n");
				kfree(pFeaturePara);
				return -ENOMEM;
			}

			memset(pValue, 0x0, sizeof(MUINT32));
			if (FeatureParaLen < sizeof(long long)) {
				PK_ERR("FeatureParaLen is too small %d\n", FeatureParaLen);
				kfree(pValue);
				kfree(pFeaturePara);
				return -ENOMEM;
			}
			*(pFeaturePara_64 + 1) = (uintptr_t)pValue;
			/*PK_DBG("[CAMERA_HW] %p %p %p\n",
			       (void *)(uintptr_t) (*(pFeaturePara_64 + 1)),
			       (void *)pFeaturePara_64, (void *)(pValue));*/
			if (g_pSensorFunc) {
				ret =
				    g_pSensorFunc->SensorFeatureControl(pFeatureCtrl->InvokeCamera,
									pFeatureCtrl->FeatureId,
									(unsigned char *)
									pFeaturePara,
									(unsigned int *)
									&FeatureParaLen);
			} else {
				PK_ERR("[CAMERA_HW]ERROR:NULL g_pSensorFunc\n");
			}
			*(pFeaturePara_64 + 1) = *pValue;
			kfree(pValue);
		}
		break;
	case SENSOR_FEATURE_GET_AE_STATUS:
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
	case SENSOR_FEATURE_GET_TEMPERATURE_VALUE:
	case SENSOR_FEATURE_GET_AF_STATUS:
	case SENSOR_FEATURE_GET_AWB_STATUS:
	case SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS:
	case SENSOR_FEATURE_GET_AE_MAX_NUM_METERING_AREAS:
	case SENSOR_FEATURE_GET_TRIGGER_FLASHLIGHT_INFO:
	case SENSOR_FEATURE_GET_SENSOR_N3D_STREAM_TO_VSYNC_TIME:
	case SENSOR_FEATURE_GET_PERIOD:
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
		{

			if (g_pSensorFunc) {
				ret =
				    g_pSensorFunc->SensorFeatureControl(pFeatureCtrl->InvokeCamera,
									pFeatureCtrl->FeatureId,
									(unsigned char *)
									pFeaturePara,
									(unsigned int *)
									&FeatureParaLen);
			} else {
				PK_DBG("[CAMERA_HW]ERROR:NULL g_pSensorFunc\n");
			}
		}
		break;
	case SENSOR_FEATURE_GET_AE_AWB_LOCK_INFO:
	case SENSOR_FEATURE_AUTOTEST_CMD:
		{
			MUINT32 *pValue0 = NULL;
			MUINT32 *pValue1 = NULL;
			unsigned long long *pFeaturePara_64 = (unsigned long long *)pFeaturePara;
			pValue0 = kmalloc(sizeof(MUINT32), GFP_KERNEL);
			pValue1 = kmalloc(sizeof(MUINT32), GFP_KERNEL);

			if (pValue0 == NULL || pValue1 == NULL) {
				PK_ERR(" ioctl allocate mem failed\n");
				kfree(pValue0);
				kfree(pValue1);
				kfree(pFeaturePara);
				return -ENOMEM;
			}
			memset(pValue1, 0x0, sizeof(MUINT32));
			memset(pValue0, 0x0, sizeof(MUINT32));
			*(pFeaturePara_64) = (uintptr_t)pValue0;
			if (FeatureParaLen < sizeof(long long)) {
				PK_ERR("FeatureParaLen is too small %d\n", FeatureParaLen);
				kfree(pValue0);
				kfree(pValue1);
				kfree(pFeaturePara);
				return -ENOMEM;
			}
			*(pFeaturePara_64 + 1) = (uintptr_t)pValue1;
			PK_DBG("[CAMERA_HW] %p %p %p\n",
			       (void *)(uintptr_t) (*(pFeaturePara_64 + 1)),
			       (void *)pFeaturePara_64, (void *)(pValue0));
			if (g_pSensorFunc) {
				ret =
				    g_pSensorFunc->SensorFeatureControl(pFeatureCtrl->InvokeCamera,
									pFeatureCtrl->FeatureId,
									(unsigned char *)
									pFeaturePara,
									(unsigned int *)
									&FeatureParaLen);
			} else {
				PK_DBG("[CAMERA_HW]ERROR:NULL g_pSensorFunc\n");
			}
			*(pFeaturePara_64) = *pValue0;
			*(pFeaturePara_64 + 1) = *pValue1;
			kfree(pValue0);
			kfree(pValue1);
		}
		break;


	case SENSOR_FEATURE_GET_EV_AWB_REF:
		{
			SENSOR_AE_AWB_REF_STRUCT *pAeAwbRef = NULL;
			unsigned long long *pFeaturePara_64 = (unsigned long long *)pFeaturePara;
			void *usr_ptr = (void*)(uintptr_t)(*(pFeaturePara_64));
			pAeAwbRef = kmalloc(sizeof(SENSOR_AE_AWB_REF_STRUCT), GFP_KERNEL);
			if (pAeAwbRef == NULL) {
				PK_ERR(" ioctl allocate mem failed\n");
				kfree(pFeaturePara);
				return -ENOMEM;
			}
			memset(pAeAwbRef, 0x0, sizeof(SENSOR_AE_AWB_REF_STRUCT));
			*(pFeaturePara_64) = (uintptr_t)pAeAwbRef;
			if (g_pSensorFunc) {
				ret =
				    g_pSensorFunc->SensorFeatureControl(pFeatureCtrl->InvokeCamera,
									pFeatureCtrl->FeatureId,
									(unsigned char *)
									pFeaturePara,
									(unsigned int *)
									&FeatureParaLen);
			} else {
				PK_DBG("[CAMERA_HW]ERROR:NULL g_pSensorFunc\n");
			}
			if (copy_to_user
			    ((void __user *)usr_ptr, (void *)pAeAwbRef,
			     sizeof(SENSOR_AE_AWB_REF_STRUCT))) {
				PK_DBG("[CAMERA_HW]ERROR: copy_to_user fail \n");
			}
			kfree(pAeAwbRef);
			*(pFeaturePara_64) = (uintptr_t)usr_ptr;
		}
		break;

	case SENSOR_FEATURE_GET_CROP_INFO:
		{
			SENSOR_WINSIZE_INFO_STRUCT *pCrop = NULL;
			unsigned long long *pFeaturePara_64 = (unsigned long long *)pFeaturePara;
			void *usr_ptr = (void *)(uintptr_t) (*(pFeaturePara_64 + 1));
			pCrop = kmalloc(sizeof(SENSOR_WINSIZE_INFO_STRUCT), GFP_KERNEL);
			if (pCrop == NULL) {
				PK_ERR(" ioctl allocate mem failed\n");
				kfree(pFeaturePara);
				return -ENOMEM;
			}
			memset(pCrop, 0x0, sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			if (FeatureParaLen < sizeof(long long)) {
				PK_ERR("FeatureParaLen is too small %d\n", FeatureParaLen);
				kfree(pCrop);
				kfree(pFeaturePara);
				return -ENOMEM;
			}
			*(pFeaturePara_64 + 1) = (uintptr_t)pCrop;
			if (g_pSensorFunc) {
				ret =
				    g_pSensorFunc->SensorFeatureControl(pFeatureCtrl->InvokeCamera,
									pFeatureCtrl->FeatureId,
									(unsigned char *)
									pFeaturePara,
									(unsigned int *)
									&FeatureParaLen);
			} else {
				PK_DBG("[CAMERA_HW]ERROR:NULL g_pSensorFunc\n");
			}
			//PK_DBG("[CAMERA_HW]crop =%d\n",framerate);

			if (copy_to_user
			    ((void __user *)usr_ptr, (void *)pCrop,
			     sizeof(SENSOR_WINSIZE_INFO_STRUCT))) {
				PK_DBG("[CAMERA_HW]ERROR: copy_to_user fail \n");
			}
			kfree(pCrop);
			*(pFeaturePara_64 + 1) = (uintptr_t)usr_ptr;
		}
		break;

	case SENSOR_FEATURE_GET_VC_INFO:
		{
			SENSOR_VC_INFO_STRUCT *pVcInfo = NULL;
			unsigned long long *pFeaturePara_64 = (unsigned long long *)pFeaturePara;
			void *usr_ptr = (void *)(uintptr_t) (*(pFeaturePara_64 + 1));
			pVcInfo = kmalloc(sizeof(SENSOR_VC_INFO_STRUCT), GFP_KERNEL);
			if (pVcInfo == NULL) {
				PK_ERR(" ioctl allocate mem failed\n");
				kfree(pFeaturePara);
				return -ENOMEM;
			}
			memset(pVcInfo, 0x0, sizeof(SENSOR_VC_INFO_STRUCT));
			if (FeatureParaLen < sizeof(long long)) {
				PK_ERR("FeatureParaLen is too small %d\n", FeatureParaLen);
				kfree(pVcInfo);
				kfree(pFeaturePara);
				return -ENOMEM;
			}
			*(pFeaturePara_64 + 1) = (uintptr_t)pVcInfo;
			if (g_pSensorFunc) {
				ret =
				    g_pSensorFunc->SensorFeatureControl(pFeatureCtrl->InvokeCamera,
									pFeatureCtrl->FeatureId,
									(unsigned char *)
									pFeaturePara,
									(unsigned int *)
									&FeatureParaLen);
			} else {
				PK_DBG("[CAMERA_HW]ERROR:NULL g_pSensorFunc\n");
			}

			if (copy_to_user
			    ((void __user *)usr_ptr, (void *)pVcInfo,
			     sizeof(SENSOR_VC_INFO_STRUCT))) {
				PK_DBG("[CAMERA_HW]ERROR: copy_to_user fail \n");
			}
			kfree(pVcInfo);
			*(pFeaturePara_64 + 1) = (uintptr_t)usr_ptr;
		}
		break;

	case SENSOR_FEATURE_GET_PDAF_INFO:
		{

#if 1
			SET_PD_BLOCK_INFO_T *pPdInfo = NULL;
			unsigned long long *pFeaturePara_64 = (unsigned long long *)pFeaturePara;
			void *usr_ptr = (void *)(uintptr_t) (*(pFeaturePara_64 + 1));
			pPdInfo = kmalloc(sizeof(SET_PD_BLOCK_INFO_T), GFP_KERNEL);
			if (pPdInfo == NULL) {
				PK_ERR(" ioctl allocate mem failed\n");
				kfree(pFeaturePara);
				return -ENOMEM;
			}
			memset(pPdInfo, 0x0, sizeof(SET_PD_BLOCK_INFO_T));
			if (FeatureParaLen < sizeof(long long)) {
				PK_ERR("FeatureParaLen is too small %d\n", FeatureParaLen);
				kfree(pPdInfo);
				kfree(pFeaturePara);
				return -ENOMEM;
			}
			*(pFeaturePara_64 + 1) = (uintptr_t)pPdInfo;
			if (g_pSensorFunc) {
				ret =
				    g_pSensorFunc->SensorFeatureControl(pFeatureCtrl->InvokeCamera,
									pFeatureCtrl->FeatureId,
									(unsigned char *)
									pFeaturePara,
									(unsigned int *)
									&FeatureParaLen);
			} else {
				PK_DBG("[CAMERA_HW]ERROR:NULL g_pSensorFunc\n");
			}

			if (copy_to_user
			    ((void __user *)usr_ptr, (void *)pPdInfo,
			     sizeof(SET_PD_BLOCK_INFO_T))) {
				PK_DBG("[CAMERA_HW]ERROR: copy_to_user fail \n");
			}
			kfree(pPdInfo);
			*(pFeaturePara_64 + 1) = (uintptr_t)usr_ptr;
#endif
		}
		break;

	case SENSOR_FEATURE_SET_AF_WINDOW:
	case SENSOR_FEATURE_SET_AE_WINDOW:
		{
			MUINT32 *pApWindows = NULL;
			unsigned long long *pFeaturePara_64 = (unsigned long long *)pFeaturePara;
			void *usr_ptr = (void *)(uintptr_t) (*(pFeaturePara_64));
			pApWindows = kmalloc(sizeof(MUINT32) * 6, GFP_KERNEL);
			if (pApWindows == NULL) {
				PK_ERR(" ioctl allocate mem failed\n");
				kfree(pFeaturePara);
				return -ENOMEM;
			}
			memset(pApWindows, 0x0, sizeof(MUINT32) * 6);
			*(pFeaturePara_64) = (uintptr_t)pApWindows;

			if (copy_from_user
			    ((void *)pApWindows, (void *)usr_ptr, sizeof(MUINT32) * 6)) {
				PK_ERR("[CAMERA_HW]ERROR: copy from user fail \n");
			}
			if (g_pSensorFunc) {
				ret =
				    g_pSensorFunc->SensorFeatureControl(pFeatureCtrl->InvokeCamera,
									pFeatureCtrl->FeatureId,
									(unsigned char *)
									pFeaturePara,
									(unsigned int *)
									&FeatureParaLen);
			} else {
				PK_ERR("[CAMERA_HW]ERROR:NULL g_pSensorFunc\n");
			}
			kfree(pApWindows);
			*(pFeaturePara_64) = (uintptr_t)usr_ptr;
		}
		break;

	case SENSOR_FEATURE_GET_EXIF_INFO:
		{
			SENSOR_EXIF_INFO_STRUCT *pExif = NULL;
			unsigned long long *pFeaturePara_64 = (unsigned long long *)pFeaturePara;
			void *usr_ptr =  (void *)(uintptr_t) (*(pFeaturePara_64));
			pExif = kmalloc(sizeof(SENSOR_EXIF_INFO_STRUCT), GFP_KERNEL);
			if (pExif == NULL) {
				PK_ERR(" ioctl allocate mem failed\n");
				kfree(pFeaturePara);
				return -ENOMEM;
			}
			memset(pExif, 0x0, sizeof(SENSOR_EXIF_INFO_STRUCT));
			*(pFeaturePara_64) = (uintptr_t)pExif;
			if (g_pSensorFunc) {
				ret =
				    g_pSensorFunc->SensorFeatureControl(pFeatureCtrl->InvokeCamera,
									pFeatureCtrl->FeatureId,
									(unsigned char *)
									pFeaturePara,
									(unsigned int *)
									&FeatureParaLen);
			} else {
				PK_DBG("[CAMERA_HW]ERROR:NULL g_pSensorFunc\n");
			}

			if (copy_to_user
			    ((void __user *)usr_ptr, (void *)pExif,
			     sizeof(SENSOR_EXIF_INFO_STRUCT))) {
				PK_DBG("[CAMERA_HW]ERROR: copy_to_user fail \n");
			}
			kfree(pExif);
			*(pFeaturePara_64) = (uintptr_t)usr_ptr;
		}
		break;


	case SENSOR_FEATURE_GET_SHUTTER_GAIN_AWB_GAIN:
		{

			SENSOR_AE_AWB_CUR_STRUCT *pCurAEAWB = NULL;
			unsigned long long *pFeaturePara_64 = (unsigned long long *)pFeaturePara;
			void *usr_ptr = (void *)(uintptr_t) (*(pFeaturePara_64));
			pCurAEAWB = kmalloc(sizeof(SENSOR_AE_AWB_CUR_STRUCT), GFP_KERNEL);
			if (pCurAEAWB == NULL) {
				PK_ERR(" ioctl allocate mem failed\n");
				kfree(pFeaturePara);
				return -ENOMEM;
			}
			memset(pCurAEAWB, 0x0, sizeof(SENSOR_AE_AWB_CUR_STRUCT));
			*(pFeaturePara_64) = (uintptr_t)pCurAEAWB;
			if (g_pSensorFunc) {
				ret =
				    g_pSensorFunc->SensorFeatureControl(pFeatureCtrl->InvokeCamera,
									pFeatureCtrl->FeatureId,
									(unsigned char *)
									pFeaturePara,
									(unsigned int *)
									&FeatureParaLen);
			} else {
				PK_DBG("[CAMERA_HW]ERROR:NULL g_pSensorFunc\n");
			}

			if (copy_to_user
			    ((void __user *)usr_ptr, (void *)pCurAEAWB,
			     sizeof(SENSOR_AE_AWB_CUR_STRUCT))) {
				PK_DBG("[CAMERA_HW]ERROR: copy_to_user fail \n");
			}
			kfree(pCurAEAWB);
			*(pFeaturePara_64) = (uintptr_t)usr_ptr;
		}
		break;

	case SENSOR_FEATURE_GET_DELAY_INFO:
		{
			SENSOR_DELAY_INFO_STRUCT *pDelayInfo = NULL;
			unsigned long long *pFeaturePara_64 = (unsigned long long *)pFeaturePara;
			void *usr_ptr = (void *)(uintptr_t) (*(pFeaturePara_64));
			pDelayInfo = kmalloc(sizeof(SENSOR_DELAY_INFO_STRUCT), GFP_KERNEL);

			if (pDelayInfo == NULL) {
				PK_ERR(" ioctl allocate mem failed\n");
				kfree(pFeaturePara);
				return -ENOMEM;
			}
			memset(pDelayInfo, 0x0, sizeof(SENSOR_DELAY_INFO_STRUCT));
			*(pFeaturePara_64) = (uintptr_t)pDelayInfo;
			if (g_pSensorFunc) {
				ret =
				    g_pSensorFunc->SensorFeatureControl(pFeatureCtrl->InvokeCamera,
									pFeatureCtrl->FeatureId,
									(unsigned char *)
									pFeaturePara,
									(unsigned int *)
									&FeatureParaLen);
			} else {
				PK_DBG("[CAMERA_HW]ERROR:NULL g_pSensorFunc\n");
			}

			if (copy_to_user
			    ((void __user *)usr_ptr, (void *)pDelayInfo,
			     sizeof(SENSOR_DELAY_INFO_STRUCT))) {
				PK_DBG("[CAMERA_HW]ERROR: copy_to_user fail \n");
			}
			kfree(pDelayInfo);
			*(pFeaturePara_64) = (uintptr_t)usr_ptr;

		}
		break;


	case SENSOR_FEATURE_GET_AE_FLASHLIGHT_INFO:
		{
			SENSOR_FLASHLIGHT_AE_INFO_STRUCT *pFlashInfo = NULL;
			unsigned long long *pFeaturePara_64 = (unsigned long long *)pFeaturePara;
			void *usr_ptr = (void *)(uintptr_t) (*(pFeaturePara_64));
			pFlashInfo = kmalloc(sizeof(SENSOR_FLASHLIGHT_AE_INFO_STRUCT), GFP_KERNEL);

			if (pFlashInfo == NULL) {
				PK_ERR(" ioctl allocate mem failed\n");
				kfree(pFeaturePara);
				return -ENOMEM;
			}
			memset(pFlashInfo, 0x0, sizeof(SENSOR_FLASHLIGHT_AE_INFO_STRUCT));
			*(pFeaturePara_64) = (uintptr_t)pFlashInfo;
			if (g_pSensorFunc) {
				ret =
				    g_pSensorFunc->SensorFeatureControl(pFeatureCtrl->InvokeCamera,
									pFeatureCtrl->FeatureId,
									(unsigned char *)
									pFeaturePara,
									(unsigned int *)
									&FeatureParaLen);
			} else {
				PK_DBG("[CAMERA_HW]ERROR:NULL g_pSensorFunc\n");
			}

			if (copy_to_user
			    ((void __user *)usr_ptr, (void *)pFlashInfo,
			     sizeof(SENSOR_FLASHLIGHT_AE_INFO_STRUCT))) {
				PK_DBG("[CAMERA_HW]ERROR: copy_to_user fail \n");
			}
			kfree(pFlashInfo);
			*(pFeaturePara_64) = (uintptr_t)usr_ptr;

		}
		break;


	case SENSOR_FEATURE_GET_PDAF_DATA:
		{
			char *pPdaf_data = NULL;

			unsigned long long *pFeaturePara_64=(unsigned long long *) pFeaturePara;
			void *usr_ptr = (void *)(uintptr_t)(*(pFeaturePara_64 + 1));
			#if 1
			pPdaf_data = kmalloc(sizeof(char) * PDAF_DATA_SIZE, GFP_KERNEL);
			if (pPdaf_data == NULL) {
				PK_ERR(" ioctl allocate mem failed\n");
				kfree(pFeaturePara);
				return -ENOMEM;
			}
			memset(pPdaf_data, 0xff, sizeof(char) * PDAF_DATA_SIZE);
			if (FeatureParaLen < 3 * sizeof(int)) {
				PK_ERR("FeatureParaLen is too small %d\n", FeatureParaLen);
				kfree(pPdaf_data);
				kfree(pFeaturePara);
				return -ENOMEM;
			}
			if (pFeaturePara_64 != NULL) {
				*(pFeaturePara_64 + 1) = (uintptr_t)pPdaf_data;//*(pFeaturePara_64 + 1) = (uintptr_t)pPdaf_data;
			}
			if (g_pSensorFunc) {
				ret =
				    g_pSensorFunc->SensorFeatureControl(pFeatureCtrl->InvokeCamera,
									pFeatureCtrl->FeatureId,
									(unsigned char *)
									pFeaturePara,
									(unsigned int *)
									&FeatureParaLen);
			} else {
				PK_DBG("[CAMERA_HW]ERROR:NULL g_pSensorFunc\n");
			}

			if (copy_to_user
			    ((void __user *)usr_ptr, (void *)pPdaf_data,
			     (kal_uint32) (*(pFeaturePara_64 + 2)))) {
				PK_DBG("[CAMERA_HW]ERROR: copy_to_user fail \n");
			}
			kfree(pPdaf_data);
			*(pFeaturePara_64 + 1) =(uintptr_t) usr_ptr;

#endif
		}
		break;
	default:

		if (g_pSensorFunc) {
			ret =
			    g_pSensorFunc->SensorFeatureControl(pFeatureCtrl->InvokeCamera,
								pFeatureCtrl->FeatureId,
								(unsigned char *)pFeaturePara,
								(unsigned int *)&FeatureParaLen);
		} else {
			PK_DBG("[CAMERA_HW]ERROR:NULL g_pSensorFunc\n");
		}

		break;
    }
    /* copy to user */
    switch (pFeatureCtrl->FeatureId)
    {
    case SENSOR_FEATURE_SET_ESHUTTER:
    case SENSOR_FEATURE_SET_GAIN:
    case SENSOR_FEATURE_SET_GAIN_AND_ESHUTTER:
    case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
    case SENSOR_FEATURE_SET_REGISTER:
    case SENSOR_FEATURE_SET_CCT_REGISTER:
    case SENSOR_FEATURE_SET_ENG_REGISTER:
    case SENSOR_FEATURE_SET_ITEM_INFO:
    /* do nothing */
    case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
    case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
    case SENSOR_FEATURE_GET_PDAF_DATA:
        break;
    /* copy to user */
    case SENSOR_FEATURE_GET_EV_AWB_REF:
    case SENSOR_FEATURE_GET_SHUTTER_GAIN_AWB_GAIN:
    case SENSOR_FEATURE_GET_EXIF_INFO:
    case SENSOR_FEATURE_GET_DELAY_INFO:
    case SENSOR_FEATURE_GET_AE_AWB_LOCK_INFO:
    case SENSOR_FEATURE_GET_RESOLUTION:
    case SENSOR_FEATURE_GET_PERIOD:
    case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
    case SENSOR_FEATURE_GET_REGISTER:
    case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
    case SENSOR_FEATURE_GET_CONFIG_PARA:
    case SENSOR_FEATURE_GET_GROUP_COUNT:
    case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
    case SENSOR_FEATURE_GET_ITEM_INFO:
    case SENSOR_FEATURE_GET_ENG_INFO:
    case SENSOR_FEATURE_GET_AF_STATUS:
    case SENSOR_FEATURE_GET_AE_STATUS:
    case SENSOR_FEATURE_GET_AWB_STATUS:
    case SENSOR_FEATURE_GET_AF_INF:
    case SENSOR_FEATURE_GET_AF_MACRO:
    case SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS:
    case SENSOR_FEATURE_GET_TRIGGER_FLASHLIGHT_INFO: /* return TRUE:play flashlight */
    case SENSOR_FEATURE_SET_YUV_3A_CMD: /* para: ACDK_SENSOR_3A_LOCK_ENUM */
    case SENSOR_FEATURE_GET_AE_FLASHLIGHT_INFO:
    case SENSOR_FEATURE_GET_AE_MAX_NUM_METERING_AREAS:
    case SENSOR_FEATURE_CHECK_SENSOR_ID:
    case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
    case SENSOR_FEATURE_SET_TEST_PATTERN:
    case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
    case SENSOR_FEATURE_GET_TEMPERATURE_VALUE:
    case SENSOR_FEATURE_SET_FRAMERATE:
    case SENSOR_FEATURE_SET_HDR:
    case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
    case SENSOR_FEATURE_GET_CROP_INFO:
    case SENSOR_FEATURE_GET_VC_INFO:
    case SENSOR_FEATURE_SET_MIN_MAX_FPS:
    case SENSOR_FEATURE_GET_PDAF_INFO:
    case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
	case SENSOR_FEATURE_SET_ISO:
    case SENSOR_FEATURE_SET_PDAF:
        /*  */
        if (copy_to_user((void __user *) pFeatureCtrl->pFeaturePara, (void *)pFeaturePara , FeatureParaLen)) {
        kfree(pFeaturePara);
        PK_DBG("[CAMERA_HW][pSensorRegData] ioctl copy to user failed\n");
        return -EFAULT;
        }
        break;
#if 0
    /* copy from and to user */
    case SENSOR_FEATURE_GET_GROUP_INFO:
        /* copy 32 bytes */
        if (copy_to_user((void __user *) pUserGroupNamePtr, (void *)kernelGroupNamePtr , sizeof(char)*32)) {
        kfree(pFeaturePara);
        PK_DBG("[CAMERA_HW][pFeatureReturnPara32] ioctl copy to user failed\n");
        return -EFAULT;
        }
        pSensorGroupInfo->GroupNamePtr = pUserGroupNamePtr;
        if (copy_to_user((void __user *) pFeatureCtrl->pFeaturePara, (void *)pFeaturePara , FeatureParaLen)) {
        kfree(pFeaturePara);
        PK_DBG("[CAMERA_HW][pFeatureReturnPara32] ioctl copy to user failed\n");
        return -EFAULT;
        }
        break;
#endif
    default:
        break;
    }

    kfree(pFeaturePara);
    if (copy_to_user((void __user *) pFeatureCtrl->pFeatureParaLen, (void *)&FeatureParaLen , sizeof(unsigned int))) {
    PK_DBG("[CAMERA_HW][pFeatureParaLen] ioctl copy to user failed\n");
    return -EFAULT;
    }
    return ret;
}   /* adopt_CAMERA_HW_FeatureControl() */


/*******************************************************************************
* adopt_CAMERA_HW_Close
********************************************************************************/
inline static int adopt_CAMERA_HW_Close(void)
{
    /* if (atomic_read(&g_CamHWOpend) == 0) { */
    /* return 0; */
    /* } */
    /* else if(atomic_read(&g_CamHWOpend) == 1) { */
    if (g_pSensorFunc) {
        g_pSensorFunc->SensorClose();
    }
    else {
        PK_DBG("[CAMERA_HW]ERROR:NULL g_pSensorFunc\n");
    }
    /* power off sensor */
    /* Marked by Jessy Lee. Should close power in kd_MultiSensorClose function
     * The following function will close all opened sensors.
     */
    /* kdModulePowerOn((CAMERA_DUAL_CAMERA_SENSOR_ENUM*)g_invokeSocketIdx, g_invokeSensorNameStr, false, CAMERA_HW_DRVNAME1); */
    /* } */
    /* atomic_set(&g_CamHWOpend, 0); */

    atomic_set(&g_CamHWOpening, 0);

    /* reset the delay frame flag */
    spin_lock(&kdsensor_drv_lock);
    g_NewSensorExpGain.uSensorExpDelayFrame = 0xFF;
    g_NewSensorExpGain.uSensorGainDelayFrame = 0xFF;
    g_NewSensorExpGain.uISPGainDelayFrame = 0xFF;
    spin_unlock(&kdsensor_drv_lock);

    return 0;
}   /* adopt_CAMERA_HW_Close() */

#ifdef CONFIG_MTK_CLKMGR
inline static int kdSetSensorMclk(int *pBuf)
{
/* #ifndef CONFIG_ARM64 */
    int ret = 0;
    ACDK_SENSOR_MCLK_STRUCT *pSensorCtrl = (ACDK_SENSOR_MCLK_STRUCT *)pBuf;

    PK_DBG("[CAMERA SENSOR] kdSetSensorMclk on=%d, freq= %d\n", pSensorCtrl->on, pSensorCtrl->freq);
	if (1 == pSensorCtrl->on
	    && (0 < (pSensorCtrl->freq) && (pSensorCtrl->freq) < MCLK_MAX_GROUP)) {
		clkmux_sel(MT_MUX_CAMTG, pSensorCtrl->freq, "CAMERA_SENSOR");
	}
    return ret;
/* #endif */
}
#else

/*******************************************************************************
* Common Clock Framework (CCF)
********************************************************************************/
static inline void Get_ccf_clk(struct platform_device *pdev)
{
	if (pdev == NULL) {
		PK_ERR("[%s] pdev is null\n", __func__);
		return;
	}
	/* get all possible using clocks.
	 * A missing topckgen index is logged instead of BUG_ON at probe. The
	 * ERR_PTR value is kept (not nulled) so the MCLK path fails visibly
	 * instead of hitting the clk_*(NULL) == 0 false pass. */
	g_camclk_camtg_sel = devm_clk_get(&pdev->dev, "TOP_CAMTG_SEL");
	if (IS_ERR(g_camclk_camtg_sel))
		pr_err("[FORGE_CAM] devm_clk_get(TOP_CAMTG_SEL) FAILED %ld\n",
		       PTR_ERR(g_camclk_camtg_sel));
	g_camclk_scam_sel = devm_clk_get(&pdev->dev, "TOP_MUX_SCAM");
	if (IS_ERR(g_camclk_scam_sel))
		pr_err("[FORGE_CAM] devm_clk_get(TOP_MUX_SCAM) FAILED %ld\n",
		       PTR_ERR(g_camclk_scam_sel));
	g_camclk_univpll_d26 = devm_clk_get(&pdev->dev, "TOP_UNIVPLL_D26");
	if (IS_ERR(g_camclk_univpll_d26))
		pr_err("[FORGE_CAM] devm_clk_get(TOP_UNIVPLL_D26) FAILED %ld\n",
		       PTR_ERR(g_camclk_univpll_d26));
	g_camclk_univpll2_d2 = devm_clk_get(&pdev->dev, "TOP_UNIVPLL2_D2");
	if (IS_ERR(g_camclk_univpll2_d2))
		pr_err("[FORGE_CAM] devm_clk_get(TOP_UNIVPLL2_D2) FAILED %ld\n",
		       PTR_ERR(g_camclk_univpll2_d2));
	/* Deliberately NOT added to Check_ccf_clk(): a board whose DTS lacks
	 * the clk26m entry must keep the existing 48 MHz path working rather
	 * than losing the whole MCLK mux. The knob checks it at use time. */
	g_camclk_clk26m = devm_clk_get(&pdev->dev, "TOP_CLK26M");
	if (IS_ERR(g_camclk_clk26m))
		pr_err("[FORGE_CAM] devm_clk_get(TOP_CLK26M) FAILED %ld - forge_cam_mclk_parent=0 will REFUSE, not silently no-op\n",
		       PTR_ERR(g_camclk_clk26m));

	return;
}

static inline int Check_ccf_clk(void)
{
	/* BUG_ON -> logged failure (see the Get_ccf_clk note) */
	if (IS_ERR(g_camclk_camtg_sel) || IS_ERR(g_camclk_univpll_d26) ||
	    IS_ERR(g_camclk_univpll2_d2) || IS_ERR(g_camclk_scam_sel)) {
		pr_err("[FORGE_CAM] camera CCF clocks missing - MCLK mux unavailable\n");
		return -EINVAL;
	}
	return 0;
}

/* camtg rate for the MCLK-ON marker in mt_cam_mclk.c. Both translation
 * units are built-in (CONFIG_MTK_IMGSENSOR=y), so a plain extern is enough.
 * Returns 0 when the handle never resolved. */
unsigned long forge_cam_camtg_rate(void)
{
	if (IS_ERR_OR_NULL(g_camclk_camtg_sel))
		return 0;
	return clk_get_rate(g_camclk_camtg_sel);
}

static inline int kdSetSensorMclk(int *pBuf)
{
	int ret = 0;
#ifndef CONFIG_MTK_FPGA
	ACDK_SENSOR_MCLK_STRUCT *pSensorCtrl = (ACDK_SENSOR_MCLK_STRUCT *)pBuf;

	PK_DBG("[CAMERA SENSOR] CCF kdSetSensorMclk on=%d, freq= %d\n", pSensorCtrl->on, pSensorCtrl->freq);

	if (Check_ccf_clk())
		return -EINVAL;
	if (1 == pSensorCtrl->on) {
			if (pSensorCtrl->freq == 1 /*CAM_PLL_48_GROUP */)
				   ret = clk_set_parent(g_camclk_camtg_sel, g_camclk_univpll_d26);
			else if (pSensorCtrl->freq == 2 /*CAM_PLL_52_GROUP */)
				   ret = clk_set_parent(g_camclk_camtg_sel, g_camclk_univpll2_d2);
	}
#endif
    return ret;

}

#endif

inline static int kdSetSensorGpio(int *pBuf)
{
    int ret = 0;
#if 0// serial cam DVT
    unsigned int temp = 0;
    IMGSENSOR_GPIO_STRUCT *pSensorgpio = (IMGSENSOR_GPIO_STRUCT *)pBuf;

#define CAMERA_PIN_RDN0_A           GPIO172 | 0x80000000
#define CAMERA_PIN_RDP0_A           GPIO173 | 0x80000000
#define CAMERA_PIN_RDN1_A           GPIO174 | 0x80000000
#define CAMERA_PIN_RCN_A            GPIO176 | 0x80000000
#define CAMERA_PIN_RCP_A            GPIO177 | 0x80000000




#define CAMERA_PIN_CSI0_A_MODE_MIPI    GPIO_MODE_01
#define CAMERA_PIN_CSI0_A_MODE_SCAM    GPIO_MODE_02


    PK_DBG("[CAMERA SENSOR] kdSetSensorGpio enable=%d, type=%d\n",
    pSensorgpio->GpioEnable, pSensorgpio->SensroInterfaceType);
    /* Please use DCT to set correct GPIO setting (below message only for debug) */
    if (pSensorgpio->SensroInterfaceType == SENSORIF_SERIAL)
    {
    if (pSensorgpio->GpioEnable == 1)
    {
		mt_set_gpio_mode(CAMERA_PIN_RDN0_A,CAMERA_PIN_CSI0_A_MODE_SCAM);
		mt_set_gpio_mode(CAMERA_PIN_RDP0_A,CAMERA_PIN_CSI0_A_MODE_SCAM);
		mt_set_gpio_mode(CAMERA_PIN_RDN1_A,CAMERA_PIN_CSI0_A_MODE_SCAM);
		mt_set_gpio_mode(CAMERA_PIN_RCN_A,CAMERA_PIN_CSI0_A_MODE_SCAM);
		mt_set_gpio_mode(CAMERA_PIN_RCP_A,CAMERA_PIN_CSI0_A_MODE_SCAM);

    }
    else
    {
		mt_set_gpio_mode(CAMERA_PIN_RDN0_A,CAMERA_PIN_CSI0_A_MODE_SCAM);
		mt_set_gpio_mode(CAMERA_PIN_RDP0_A,CAMERA_PIN_CSI0_A_MODE_SCAM);
		mt_set_gpio_mode(CAMERA_PIN_RDN1_A,CAMERA_PIN_CSI0_A_MODE_SCAM);
		mt_set_gpio_mode(CAMERA_PIN_RCN_A,CAMERA_PIN_CSI0_A_MODE_SCAM);
		mt_set_gpio_mode(CAMERA_PIN_RCP_A,CAMERA_PIN_CSI0_A_MODE_SCAM);

    }
    }
#endif
    return ret;
}

/* PMIC */
#if !defined(CONFIG_MTK_LEGACY)
bool Get_Cam_Regulator(void)
{
	struct device_node *node = NULL, *kd_node;
	if (1) {
		/* check if customer camera node defined */
		node = of_find_compatible_node(NULL, NULL, "mediatek,camera_hw");

		if (node) {
			kd_node = sensor_device->of_node;
			sensor_device->of_node = node;
			/* name = of_get_property(node, "MAIN_CAMERA_POWER_A", NULL); */
			 regSubVCAMD = regulator_get(sensor_device, "vcamd_sub"); /*check customer definition*/
			if (regSubVCAMD == NULL) {
			    if (regVCAMA == NULL) {
				    regVCAMA = regulator_get(sensor_device, "vcama");
			    }
			    if (regVCAMD == NULL) {
				    regVCAMD = regulator_get(sensor_device, "vcamd");
			    }
			    if (regVCAMIO == NULL) {
				    regVCAMIO = regulator_get(sensor_device, "vcamio");
			    }
			    if (regVCAMAF == NULL) {
				    regVCAMAF = regulator_get(sensor_device, "vcamaf");
			    }
			} else{
				PK_DBG("Camera customer regulator!\n");
			    if (regVCAMA == NULL) {
				    regVCAMA = regulator_get(sensor_device, "vcama");
			    }
			    if (regVCAMD == NULL) {
				    regVCAMD = regulator_get(sensor_device, "vcamd");
			    }
				if (regSubVCAMD == NULL) {
				    regSubVCAMD = regulator_get(sensor_device, "vcamd_sub");
			    }
			    if (regVCAMIO == NULL) {
				    regVCAMIO = regulator_get(sensor_device, "vcamio");
			    }
			    if (regVCAMAF == NULL) {
				    regVCAMAF = regulator_get(sensor_device, "vcamaf");
			    }
			    if (regMain2VCAMD == NULL) {
				    regMain2VCAMD = regulator_get(sensor_device, "vcamd_main2");
			    }
			    /* restore original dev.of_node */
			}
			 sensor_device->of_node = kd_node;
		} else{
			PK_ERR("regulator get cust camera node failed!\n");
			return FALSE;
		}

		return TRUE;
	}
	return FALSE;
}


#ifdef CONFIG_MACH_MT6755
/* Camera rails are switched as the stock kernel does it (MT6351 regulator
 * in power/mt6755/pmic.c): AVDD = VCAMA, DOVDD = VCAMIO for both sensors,
 * DVDD = VCAMD for the main sensor and VGP3 at 1.22 V for the front one
 * (stock rewrites 1.2 V to 1.22 V there). VOSEL with the stock table index
 * first, then EN; off = EN 0; VCAMA/VCAMIO are shared, so they are
 * reference-counted like the stock regulator core did. Any other rail or
 * voltage is refused. Every write goes through the mt_pmic_stub allowlist,
 * which accepts these fields at these codes only. */
#include <mt-plat/upmu_common.h>
#include <mach/upmu_hw.h>

struct forge_cam_rail {
	const char *name;
	PMU_FLAGS_LIST_ENUM en, vosel;
	unsigned int code;	/* stock table index */
	int users;
};

enum { FORGE_RAIL_VCAMA, FORGE_RAIL_VCAMD, FORGE_RAIL_VCAMIO, FORGE_RAIL_VGP3 };

static struct forge_cam_rail forge_cam_rails[] = {
	[FORGE_RAIL_VCAMA] = { "VCAMA 2.8V", MT6351_PMIC_RG_VCAMA_EN, MT6351_PMIC_RG_VCAMA_VOSEL, 3 },
	[FORGE_RAIL_VCAMD] = { "VCAMD 1.2V", MT6351_PMIC_RG_VCAMD_EN, MT6351_PMIC_RG_VCAMD_VOSEL, 5 },
	[FORGE_RAIL_VCAMIO] = { "VCAMIO 1.8V", MT6351_PMIC_RG_VCAMIO_EN, MT6351_PMIC_RG_VCAMIO_VOSEL, 6 },
	[FORGE_RAIL_VGP3] = { "VGP3 1.22V", MT6351_PMIC_RG_VGP3_EN, MT6351_PMIC_RG_VGP3_VOSEL, 3 },
};
static DEFINE_MUTEX(forge_cam_rail_lock);

/* rail for (type, voltage) as stock maps it, or -1 */
static int forge_cam_rail_of(PowerType type, int volt)
{
	if (type == AVDD && volt == Vol_2800)
		return FORGE_RAIL_VCAMA;
	if (type == DVDD && volt == Vol_1200)
		return FORGE_RAIL_VCAMD;
	if (type == SUB_DVDD && (volt == Vol_1200 || volt == Vol_1220))
		return FORGE_RAIL_VGP3;
	if (type == DOVDD && volt == Vol_1800)
		return FORGE_RAIL_VCAMIO;
	return -1;
}

static bool forge_cam_rail_on(PowerType type, int volt)
{
	int i = forge_cam_rail_of(type, volt);
	struct forge_cam_rail *r;
	bool ok = true;

	if (i < 0) {
		pr_err("[FORGE_CAM] rail type %d %d uV: not a stock camera rail, refused\n",
		       type, volt);
		return false;
	}
	r = &forge_cam_rails[i];
	mutex_lock(&forge_cam_rail_lock);
	if (r->users == 0) {
		if (pmic_set_register_value(r->vosel, r->code) ||
		    pmic_set_register_value(r->en, 1))
			ok = false;
		pr_err("[FORGE_CAM] rail %s on: %s (VOSEL %u EN %u)\n", r->name,
		       ok ? "ok" : "REFUSED", pmic_get_register_value(r->vosel),
		       pmic_get_register_value(r->en));
	}
	if (ok)
		r->users++;
	mutex_unlock(&forge_cam_rail_lock);
	return ok;
}

static bool forge_cam_rail_off(PowerType type)
{
	int i;
	bool ok = true;

	for (i = 0; i < ARRAY_SIZE(forge_cam_rails); i++) {
		struct forge_cam_rail *r = &forge_cam_rails[i];

		/* off carries no voltage: match the type the way _on did */
		if (!((type == AVDD && i == FORGE_RAIL_VCAMA) ||
		      (type == DVDD && i == FORGE_RAIL_VCAMD) ||
		      (type == SUB_DVDD && i == FORGE_RAIL_VGP3) ||
		      (type == DOVDD && i == FORGE_RAIL_VCAMIO)))
			continue;
		mutex_lock(&forge_cam_rail_lock);
		if (r->users > 0 && --r->users == 0) {
			ok = !pmic_set_register_value(r->en, 0);
			pr_err("[FORGE_CAM] rail %s off: %s\n", r->name,
			       ok ? "ok" : "REFUSED");
		}
		mutex_unlock(&forge_cam_rail_lock);
		return ok;
	}
	pr_err("[FORGE_CAM] rail type %d off: not a stock camera rail\n", type);
	return false;
}
#endif

bool _hwPowerOn(PowerType type, int powerVolt)
{
    bool ret = FALSE;
	struct regulator *reg = NULL;
	int reg_enabled = -1;

	PK_DBG("[_hwPowerOn]powertype:%d powerId:%d\n", type, powerVolt);
#ifdef CONFIG_MACH_MT6755
	return forge_cam_rail_on(type, powerVolt);
#endif
    if (type == AVDD) {
	reg = regVCAMA;
    } else if (type == DVDD) {
	reg = regVCAMD;
    } else if (type == DOVDD) {
	reg = regVCAMIO;
    } else if (type == AFVDD) {
	reg = regVCAMAF;
    } else if (type == SUB_DVDD) {
	reg = regSubVCAMD;
    } else if (type == MAIN2_DVDD) {
	reg = regMain2VCAMD;
    } else {
	M6_CAM_DIAG("CAM_RAIL_ON bad_type type=%d uV=%d\n", type, powerVolt);
    	return ret;
    }

	M6_CAM_DIAG("CAM_RAIL_ON enter type=%s(%d) uV=%d reg=%p is_err=%d\n",
		m6_cam_power_type_name(type), type, powerVolt, reg, IS_ERR(reg));

	if (!IS_ERR(reg)) {
#ifdef CONFIG_MTK_PMIC_CHIP_MT6353

		if (type == DVDD && powerVolt == Vol_1200){
			PK_DBG("[_hwPowerOn] PMIC_CHIP_MT6353 DVDD 1.2v\n");
			powerVolt = Vol_1220;
			if (regulator_set_voltage(reg , powerVolt, powerVolt) != 0) {
				M6_CAM_DIAG("CAM_RAIL_ON set_voltage_fail type=%s(%d) uV=%d reg=%p\n",
					m6_cam_power_type_name(type), type, powerVolt, reg);
				PK_ERR("[_hwPowerOn]fail to regulator_set_voltage, powertype:%d powerId:%d\n", type, powerVolt);
				return ret;
			}
			if(pmic_set_register_value(PMIC_RG_VCAMD_CAL,0x1))//-20mv
			{
				M6_CAM_DIAG("CAM_RAIL_ON pmic_cal_fail type=%s(%d) uV=%d reg=%p\n",
					m6_cam_power_type_name(type), type, powerVolt, reg);
				PK_ERR("[_hwPowerOn]fail to set PMIC_RG_VCAMD_CAL, powertype:%d powerId:%d\n", type, powerVolt);
				return ret;
			}
		} else {
			if (regulator_set_voltage(reg , powerVolt, powerVolt) != 0) {
				M6_CAM_DIAG("CAM_RAIL_ON set_voltage_fail type=%s(%d) uV=%d reg=%p\n",
					m6_cam_power_type_name(type), type, powerVolt, reg);
				PK_ERR("[_hwPowerOn]fail to regulator_set_voltage, powertype:%d powerId:%d\n", type, powerVolt);
				return ret;
			}
		}
#else
		if (regulator_set_voltage(reg , powerVolt, powerVolt) != 0) {
			M6_CAM_DIAG("CAM_RAIL_ON set_voltage_fail type=%s(%d) uV=%d reg=%p\n",
				m6_cam_power_type_name(type), type, powerVolt, reg);
			PK_ERR("[_hwPowerOn]fail to regulator_set_voltage, powertype:%d powerId:%d\n", type, powerVolt);
			return ret;
	    }
#endif
		if (regulator_enable(reg) != 0) {
			M6_CAM_DIAG("CAM_RAIL_ON enable_fail type=%s(%d) uV=%d reg=%p\n",
				m6_cam_power_type_name(type), type, powerVolt, reg);
			PK_ERR("[_hwPowerOn]fail to regulator_enable, powertype:%d powerId:%d\n", type, powerVolt);
			return ret;
		}
		ret = true;
    } else {
		M6_CAM_DIAG("CAM_RAIL_ON invalid_reg type=%s(%d) uV=%d reg=%p\n",
			m6_cam_power_type_name(type), type, powerVolt, reg);
		PK_ERR("[_hwPowerOn]IS_ERR_OR_NULL powertype:%d reg %p\n", type,reg);
		return ret;
    }

	if (!IS_ERR(reg))
		reg_enabled = regulator_is_enabled(reg);
	M6_CAM_DIAG("CAM_RAIL_ON exit type=%s(%d) uV=%d ret=%d enabled=%d reg=%p\n",
		m6_cam_power_type_name(type), type, powerVolt, ret,
		reg_enabled, reg);
	/* Dummy-regulator check: when the MT6351 vcam LDOs are not registered,
	 * regulator_get can hand back a dummy whose enable "succeeds" with no real
	 * rail, and the sensor then NAKs. A real MT6351 vcam LDO reads back the
	 * requested voltage. */
	if (ret) {
		int rb = IS_ERR(reg) ? -1 : regulator_get_voltage(reg);

		if (rb != powerVolt)
			pr_err("[FORGE_CAM] RAIL %s(%d): enable 'ok' but readback=%duV (asked %duV) - rail is NOT real (vcam LDOs unregistered? skip_regulator?); sensor NAK in this state is NOT identity evidence\n",
			       m6_cam_power_type_name(type), type, rb, powerVolt);
	}
	return ret;
}

/* Read back the shared VCAMD rail. Main DVDD and sub DVDD are two
 * consumer handles on one physical LDO (vcamd-supply and vcamd_sub-supply
 * both resolve to mt_pmic_vcamd_ldo_reg), so either handle reads the same
 * rail; regSubVCAMD is used because the caller is the sub power-on path.
 * Returns the rail in microvolts, or a negative errno / -1 when the handle
 * is bad.
 *
 * A consumer's latched min_uV/max_uV lives inside struct regulator, which
 * is private to drivers/regulator/, so only the rail is reported. */
int forge_cam_vcamd_uv(void)
{
	if (IS_ERR_OR_NULL(regSubVCAMD))
		return -1;
	return regulator_get_voltage(regSubVCAMD);
}

bool _hwPowerDown(PowerType type)
{
    bool ret = FALSE;
	struct regulator *reg = NULL;
	int reg_enabled = -1;

#ifdef CONFIG_MACH_MT6755
	return forge_cam_rail_off(type);
#endif

	if (type == AVDD) {
	 reg = regVCAMA;
	 } else if (type == DVDD) {
	 reg = regVCAMD;
	 } else if (type == DOVDD) {
	 reg = regVCAMIO;
	 } else if (type == AFVDD) {
	 reg = regVCAMAF;
	 } else if (type == SUB_DVDD) {
	 reg = regSubVCAMD;
	 } else if (type == MAIN2_DVDD) {
	 reg = regMain2VCAMD;
	 } else {
		M6_CAM_DIAG("CAM_RAIL_OFF bad_type type=%d\n", type);
		return ret;
	 }


	M6_CAM_DIAG("CAM_RAIL_OFF enter type=%s(%d) reg=%p is_err=%d\n",
		m6_cam_power_type_name(type), type, reg, IS_ERR(reg));

    if (!IS_ERR(reg)) {
			if (regulator_is_enabled(reg) != 0) {
				PK_DBG("[_hwPowerDown]%d is enabled\n", type);
				if (regulator_disable(reg) != 0) {
					M6_CAM_DIAG("CAM_RAIL_OFF disable_fail type=%s(%d) reg=%p\n",
						m6_cam_power_type_name(type), type, reg);
					PK_ERR("[_hwPowerDown]fail to regulator_disable, powertype: %d\n\n", type);
					return ret;
				}
			}
			ret = true;
    } else {
			M6_CAM_DIAG("CAM_RAIL_OFF invalid_reg type=%s(%d) reg=%p\n",
				m6_cam_power_type_name(type), type, reg);
			PK_ERR("[_hwPowerDown]%d fail to power down  due to regVCAM == NULL\n", type);
			return ret;
    }
	if (!IS_ERR(reg))
		reg_enabled = regulator_is_enabled(reg);
	M6_CAM_DIAG("CAM_RAIL_OFF exit type=%s(%d) ret=%d enabled=%d reg=%p\n",
		m6_cam_power_type_name(type), type, ret, reg_enabled, reg);
    return ret;
}


#endif

#ifdef CONFIG_COMPAT

static int compat_get_acdk_sensor_getinfo_struct(
        COMPAT_ACDK_SENSOR_GETINFO_STRUCT __user *data32,
        ACDK_SENSOR_GETINFO_STRUCT __user *data)
{
    compat_uint_t i;
    compat_uptr_t p;
    int err;

    err = get_user(i, &data32->ScenarioId[0]);
    err |= put_user(i, &data->ScenarioId[0]);
    err = get_user(i, &data32->ScenarioId[1]);
    err |= put_user(i, &data->ScenarioId[1]);
    err = get_user(p, &data32->pInfo[0]);
    err |= put_user(compat_ptr(p), &data->pInfo[0]);
    err = get_user(p, &data32->pInfo[1]);
    err |= put_user(compat_ptr(p), &data->pInfo[1]);
    err = get_user(p, &data32->pInfo[0]);
    err |= put_user(compat_ptr(p), &data->pConfig[0]);
    err = get_user(p, &data32->pInfo[1]);
    err |= put_user(compat_ptr(p), &data->pConfig[1]);

    return err;
}

static int compat_put_acdk_sensor_getinfo_struct(
        COMPAT_ACDK_SENSOR_GETINFO_STRUCT __user *data32,
        ACDK_SENSOR_GETINFO_STRUCT __user *data)
{
    compat_uint_t i;
    int err;

    err = get_user(i, &data->ScenarioId[0]);
    err |= put_user(i, &data32->ScenarioId[0]);
    err = get_user(i, &data->ScenarioId[1]);
    err |= put_user(i, &data32->ScenarioId[1]);
    return err;
}

static int compat_get_imagesensor_getinfo_struct(
        COMPAT_IMAGESENSOR_GETINFO_STRUCT __user *data32,
        IMAGESENSOR_GETINFO_STRUCT __user *data)
{
    compat_uptr_t p;
    compat_uint_t i;
    int err;

    err = get_user(i, &data32->SensorId);
    err |= put_user(i, &data->SensorId);
    err |= get_user(p, &data32->pInfo);
    err |= put_user(compat_ptr(p), &data->pInfo);
    err |= get_user(p, &data32->pSensorResolution);
    err |= put_user(compat_ptr(p), &data->pSensorResolution);
    return err;
}

static int compat_put_imagesensor_getinfo_struct(
        COMPAT_IMAGESENSOR_GETINFO_STRUCT __user *data32,
        IMAGESENSOR_GETINFO_STRUCT __user *data)
{
    /* compat_uptr_t p; */
    compat_uint_t i;
    int err;

    err = get_user(i, &data->SensorId);
    err |= put_user(i, &data32->SensorId);
    /* Assume pointer is not change */
#if 0
    err |= get_user(p, &data->pInfo);
    err |= put_user(p, &data32->pInfo);
    err |= get_user(p, &data->pSensorResolution);
    err |= put_user(p, &data32->pSensorResolution);  */
#endif
    return err;
}

static int compat_get_acdk_sensor_featurecontrol_struct(
        COMPAT_ACDK_SENSOR_FEATURECONTROL_STRUCT __user *data32,
        ACDK_SENSOR_FEATURECONTROL_STRUCT __user *data)
{
    compat_uptr_t p;
    compat_uint_t i;
    int err;

    err = get_user(i, &data32->InvokeCamera);
    err |= put_user(i, &data->InvokeCamera);
    err |= get_user(i, &data32->FeatureId);
    err |= put_user(i, &data->FeatureId);
    err |= get_user(p, &data32->pFeaturePara);
    err |= put_user(compat_ptr(p), &data->pFeaturePara);
    err |= get_user(p, &data32->pFeatureParaLen);
    err |= put_user(compat_ptr(p), &data->pFeatureParaLen);
    return err;
}

static int compat_put_acdk_sensor_featurecontrol_struct(
        COMPAT_ACDK_SENSOR_FEATURECONTROL_STRUCT __user *data32,
        ACDK_SENSOR_FEATURECONTROL_STRUCT __user *data)
{
    MUINT8 *p;
    MUINT32 *q;
    compat_uint_t i;
    int err;

    err = get_user(i, &data->InvokeCamera);
    err |= put_user(i, &data32->InvokeCamera);
    err |= get_user(i, &data->FeatureId);
    err |= put_user(i, &data32->FeatureId);
    /* Assume pointer is not change */

    err |= get_user(p, &data->pFeaturePara);
    err |= put_user(ptr_to_compat(p), &data32->pFeaturePara);
    err |= get_user(q, &data->pFeatureParaLen);
    err |= put_user(ptr_to_compat(q), &data32->pFeatureParaLen);

    return err;
}

static int compat_get_acdk_sensor_control_struct(
        COMPAT_ACDK_SENSOR_CONTROL_STRUCT __user *data32,
        ACDK_SENSOR_CONTROL_STRUCT __user *data)
{
    compat_uptr_t p;
    compat_uint_t i;
    int err;

    err = get_user(i, &data32->InvokeCamera);
    err |= put_user(i, &data->InvokeCamera);
    err |= get_user(i, &data32->ScenarioId);
    err |= put_user(i, &data->ScenarioId);
    err |= get_user(p, &data32->pImageWindow);
    err |= put_user(compat_ptr(p), &data->pImageWindow);
    err |= get_user(p, &data32->pSensorConfigData);
    err |= put_user(compat_ptr(p), &data->pSensorConfigData);
    return err;
}

static int compat_put_acdk_sensor_control_struct(
        COMPAT_ACDK_SENSOR_CONTROL_STRUCT __user *data32,
        ACDK_SENSOR_CONTROL_STRUCT __user *data)
{
    /* compat_uptr_t p; */
    compat_uint_t i;
    int err;

    err = get_user(i, &data->InvokeCamera);
    err |= put_user(i, &data32->InvokeCamera);
    err |= get_user(i, &data->ScenarioId);
    err |= put_user(i, &data32->ScenarioId);
    /* Assume pointer is not change */
#if 0
    err |= get_user(p, &data->pImageWindow);
    err |= put_user(p, &data32->pImageWindow);
    err |= get_user(p, &data->pSensorConfigData);
    err |= put_user(p, &data32->pSensorConfigData);
#endif
    return err;
}

static int compat_get_acdk_sensor_resolution_info_struct(
        COMPAT_ACDK_SENSOR_PRESOLUTION_STRUCT __user *data32,
        ACDK_SENSOR_PRESOLUTION_STRUCT __user *data)
{
    int err;
    compat_uptr_t p;
    err = get_user(p, &data32->pResolution[0]);
    err |= put_user(compat_ptr(p), &data->pResolution[0]);
    err = get_user(p, &data32->pResolution[1]);
    err |= put_user(compat_ptr(p), &data->pResolution[1]);

    /* err = copy_from_user((void*)data, (void*)data32, sizeof(compat_uptr_t) * 2); */
    /* err = copy_from_user((void*)data[0], (void*)data32[0], sizeof(ACDK_SENSOR_RESOLUTION_INFO_STRUCT)); */
    /* err = copy_from_user((void*)data[1], (void*)data32[1], sizeof(ACDK_SENSOR_RESOLUTION_INFO_STRUCT)); */
    return err;
}

static int compat_put_acdk_sensor_resolution_info_struct(
	COMPAT_ACDK_SENSOR_PRESOLUTION_STRUCT __user *data32,
	ACDK_SENSOR_PRESOLUTION_STRUCT __user *data)
{
	int err = 0;
    /* err = copy_to_user((void*)data, (void*)data32, sizeof(compat_uptr_t) * 2); */
    /* err = copy_to_user((void*)data[0], (void*)data32[0], sizeof(ACDK_SENSOR_RESOLUTION_INFO_STRUCT)); */
    /* err = copy_to_user((void*)data[1], (void*)data32[1], sizeof(ACDK_SENSOR_RESOLUTION_INFO_STRUCT)); */
    return err;
}



static long CAMERA_HW_Ioctl_Compat(struct file *filp, unsigned int cmd, unsigned long arg)
{
    long ret;
    bool m6TraceCompat = false;

    if (!filp->f_op || !filp->f_op->unlocked_ioctl)
    return -ENOTTY;

    switch (cmd) {
    case COMPAT_KDIMGSENSORIOC_X_GETINFO:
    case COMPAT_KDIMGSENSORIOC_X_GETINFO2:
    case COMPAT_KDIMGSENSORIOC_X_GETRESOLUTION2:
    case COMPAT_KDIMGSENSORIOC_X_FEATURECONCTROL:
    case COMPAT_KDIMGSENSORIOC_X_CONTROL:
    case KDIMGSENSORIOC_T_OPEN:
    case KDIMGSENSORIOC_T_CLOSE:
    case KDIMGSENSORIOC_T_CHECK_IS_ALIVE:
    case KDIMGSENSORIOC_X_SET_DRIVER:
    case KDIMGSENSORIOC_X_GET_SOCKET_POS:
    case KDIMGSENSORIOC_X_GETINFO:
    case KDIMGSENSORIOC_X_GETINFO2:
    case KDIMGSENSORIOC_X_FEATURECONCTROL:
    case KDIMGSENSORIOC_X_CONTROL:
        m6TraceCompat = true;
        break;
    default:
        break;
    }

    if (m6TraceCompat) {
        PK_ERR("[M6_CAM] compat ioctl entry cmd=0x%08x dir=%u size=%u arg=0x%lx\n",
            cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd), arg);
    }

    switch (cmd) {
    case COMPAT_KDIMGSENSORIOC_X_GETINFO:
    {
    COMPAT_ACDK_SENSOR_GETINFO_STRUCT __user *data32;
    ACDK_SENSOR_GETINFO_STRUCT __user *data;
    int err;
    PK_DBG("[CAMERA SENSOR] CAOMPAT_KDIMGSENSORIOC_X_GETINFO E\n");

    data32 = compat_ptr(arg);
    data = compat_alloc_user_space(sizeof(*data));
    if (data == NULL)
        return -EFAULT;

    err = compat_get_acdk_sensor_getinfo_struct(data32, data);
    if (err)
        return err;

    ret = filp->f_op->unlocked_ioctl(filp, KDIMGSENSORIOC_X_GETINFO,(unsigned long)data);
    err = compat_put_acdk_sensor_getinfo_struct(data32, data);

    if(err != 0)
        PK_DBG("[CAMERA SENSOR] compat_put_acdk_sensor_getinfo_struct failed\n");
    return ret;
    }
    case COMPAT_KDIMGSENSORIOC_X_FEATURECONCTROL:
    {
    COMPAT_ACDK_SENSOR_FEATURECONTROL_STRUCT __user *data32;
    ACDK_SENSOR_FEATURECONTROL_STRUCT __user *data;
    int err;
    PK_DBG("[CAMERA SENSOR] CAOMPAT_KDIMGSENSORIOC_X_FEATURECONCTROL\n");

    data32 = compat_ptr(arg);
    data = compat_alloc_user_space(sizeof(*data));
    if (data == NULL)
        return -EFAULT;

    err = compat_get_acdk_sensor_featurecontrol_struct(data32, data);
    if (err)
        return err;

    ret = filp->f_op->unlocked_ioctl(filp, KDIMGSENSORIOC_X_FEATURECONCTROL, (unsigned long)data);
    err = compat_put_acdk_sensor_featurecontrol_struct(data32, data);


    if (err != 0)
        PK_ERR("[CAMERA SENSOR] compat_put_acdk_sensor_getinfo_struct failed\n");
    return ret;
    }
    case COMPAT_KDIMGSENSORIOC_X_CONTROL:
    {
    COMPAT_ACDK_SENSOR_CONTROL_STRUCT __user *data32;
    ACDK_SENSOR_CONTROL_STRUCT __user *data;
    int err;
    PK_DBG("[CAMERA SENSOR] CAOMPAT_KDIMGSENSORIOC_X_CONTROL\n");

    data32 = compat_ptr(arg);
    data = compat_alloc_user_space(sizeof(*data));
    if (data == NULL)
        return -EFAULT;

    err = compat_get_acdk_sensor_control_struct(data32, data);
    if (err)
        return err;
    ret = filp->f_op->unlocked_ioctl(filp, KDIMGSENSORIOC_X_CONTROL, (unsigned long)data);
    err = compat_put_acdk_sensor_control_struct(data32, data);

    if (err != 0)
        PK_ERR("[CAMERA SENSOR] compat_put_acdk_sensor_getinfo_struct failed\n");
    return ret;
    }
    case COMPAT_KDIMGSENSORIOC_X_GETINFO2:
    {
    COMPAT_IMAGESENSOR_GETINFO_STRUCT __user *data32;
    IMAGESENSOR_GETINFO_STRUCT __user *data;
    int err;
    PK_DBG("[CAMERA SENSOR] CAOMPAT_KDIMGSENSORIOC_X_GETINFO2\n");

    data32 = compat_ptr(arg);
    data = compat_alloc_user_space(sizeof(*data));
    if (data == NULL)
        return -EFAULT;

    err = compat_get_imagesensor_getinfo_struct(data32, data);
    if (err)
        return err;
    ret = filp->f_op->unlocked_ioctl(filp, KDIMGSENSORIOC_X_GETINFO2, (unsigned long)data);
    err = compat_put_imagesensor_getinfo_struct(data32, data);

    if (err != 0)
        PK_ERR("[CAMERA SENSOR] compat_put_acdk_sensor_getinfo_struct failed\n");
    return ret;
    }
    case COMPAT_KDIMGSENSORIOC_X_GETRESOLUTION2:
    {
    COMPAT_ACDK_SENSOR_PRESOLUTION_STRUCT __user *data32;
    ACDK_SENSOR_PRESOLUTION_STRUCT __user *data;
    int err;
    PK_DBG("[CAMERA SENSOR] KDIMGSENSORIOC_X_GETRESOLUTION2\n");

    data32 = compat_ptr(arg);
    data = compat_alloc_user_space(sizeof(data));
    if (data == NULL)
        return -EFAULT;
    PK_DBG("[CAMERA SENSOR] compat_get_acdk_sensor_resolution_info_struct\n");
    err = compat_get_acdk_sensor_resolution_info_struct(data32, data);
    if (err)
        return err;
    PK_DBG("[CAMERA SENSOR] unlocked_ioctl\n");
    ret = filp->f_op->unlocked_ioctl(filp, KDIMGSENSORIOC_X_GETRESOLUTION2, (unsigned long)data);

	err = compat_put_acdk_sensor_resolution_info_struct(data32, data);
    if (err != 0)
        PK_ERR("[CAMERA SENSOR] compat_get_Acdk_sensor_resolution_info_struct failed\n");
    return ret;
    }
    /* Data in the following commands is not required to be converted to kernel 64-bit & user 32-bit */
    case KDIMGSENSORIOC_T_OPEN:
    case KDIMGSENSORIOC_T_CLOSE:
    case KDIMGSENSORIOC_T_CHECK_IS_ALIVE:
    case KDIMGSENSORIOC_X_SET_DRIVER:
    case KDIMGSENSORIOC_X_GET_SOCKET_POS:
    case KDIMGSENSORIOC_X_SET_I2CBUS:
    case KDIMGSENSORIOC_X_RELEASE_I2C_TRIGGER_LOCK:
    case KDIMGSENSORIOC_X_SET_SHUTTER_GAIN_WAIT_DONE:
    case KDIMGSENSORIOC_X_SET_MCLK_PLL:
    case KDIMGSENSORIOC_X_SET_CURRENT_SENSOR:
    case KDIMGSENSORIOC_X_SET_GPIO:
    case KDIMGSENSORIOC_X_GET_ISP_CLK:
    ret = filp->f_op->unlocked_ioctl(filp, cmd, arg);
    if (m6TraceCompat) {
        PK_ERR("[M6_CAM] compat ioctl passthrough exit cmd=0x%08x ret=%ld\n",
            cmd, ret);
    }
    return ret;

    default:
    PK_ERR("[M6_CAM] compat ioctl default cmd=0x%08x\n", cmd);
    return -ENOIOCTLCMD;
    }
}


#endif

/*******************************************************************************
* CAMERA_HW_Ioctl
********************************************************************************/

static long CAMERA_HW_Ioctl(
    struct file *a_pstFile,
    unsigned int a_u4Command,
    unsigned long a_u4Param
)
{

    int i4RetValue = 0;
    void *pBuff = NULL;
    u32 *pIdx = NULL;
    bool m6TraceIoctl = false;

    mutex_lock(&kdCam_Mutex);

    switch (a_u4Command) {
    case KDIMGSENSORIOC_X_SET_DRIVER:
    case KDIMGSENSORIOC_T_OPEN:
    case KDIMGSENSORIOC_X_GET_SOCKET_POS:
    case KDIMGSENSORIOC_X_GETINFO:
    case KDIMGSENSORIOC_X_GETINFO2:
    case KDIMGSENSORIOC_X_GETRESOLUTION2:
    case KDIMGSENSORIOC_X_FEATURECONCTROL:
    case KDIMGSENSORIOC_X_CONTROL:
    case KDIMGSENSORIOC_T_CHECK_IS_ALIVE:
    case KDIMGSENSORIOC_T_CLOSE:
        m6TraceIoctl = true;
        break;
    default:
        break;
    }

    if (m6TraceIoctl) {
        PK_ERR("[M6_CAM] ioctl entry cmd=0x%08x dir=%u size=%u param=0x%lx\n",
            a_u4Command, _IOC_DIR(a_u4Command), _IOC_SIZE(a_u4Command),
            a_u4Param);
    }

    if (_IOC_NONE == _IOC_DIR(a_u4Command)) {
    }
    else {
    pBuff = kmalloc(_IOC_SIZE(a_u4Command), GFP_KERNEL);

    if (NULL == pBuff) {
        PK_DBG("[CAMERA SENSOR] ioctl allocate mem failed\n");
        i4RetValue = -ENOMEM;
        goto CAMERA_HW_Ioctl_EXIT;
    }

    if (_IOC_WRITE & _IOC_DIR(a_u4Command)) {
        if (copy_from_user(pBuff , (void *) a_u4Param, _IOC_SIZE(a_u4Command))) {
        kfree(pBuff);
        PK_DBG("[CAMERA SENSOR] ioctl copy from user failed\n");
        i4RetValue =  -EFAULT;
        goto CAMERA_HW_Ioctl_EXIT;
        }
    }
    }

    pIdx = (u32 *)pBuff;
    switch (a_u4Command) {

#if 0
    case KDIMGSENSORIOC_X_POWER_ON:
        i4RetValue = kdModulePowerOn((CAMERA_DUAL_CAMERA_SENSOR_ENUM) *pIdx, true, CAMERA_HW_DRVNAME);
        break;
    case KDIMGSENSORIOC_X_POWER_OFF:
        i4RetValue = kdModulePowerOn((CAMERA_DUAL_CAMERA_SENSOR_ENUM) *pIdx, false, CAMERA_HW_DRVNAME);
        break;
#endif
    case KDIMGSENSORIOC_X_SET_DRIVER:
        PK_ERR("[M6_CAM] ioctl SET_DRIVER size=%u pBuff=%p raw0=0x%08x raw1=0x%08x\n",
            _IOC_SIZE(a_u4Command), pBuff, pIdx ? pIdx[0] : 0, pIdx ? pIdx[1] : 0);
        i4RetValue = kdSetDriver((unsigned int *)pBuff);
        PK_ERR("[M6_CAM] ioctl SET_DRIVER ret=%d\n", i4RetValue);
        break;
    case KDIMGSENSORIOC_T_OPEN:
        i4RetValue = adopt_CAMERA_HW_Open();
        break;
    case KDIMGSENSORIOC_X_GETINFO:
        i4RetValue = adopt_CAMERA_HW_GetInfo(pBuff);
        break;
    case KDIMGSENSORIOC_X_GETRESOLUTION2:
        i4RetValue = adopt_CAMERA_HW_GetResolution(pBuff);
        break;
    case KDIMGSENSORIOC_X_GETINFO2:
        i4RetValue = adopt_CAMERA_HW_GetInfo2(pBuff);
        break;
    case KDIMGSENSORIOC_X_FEATURECONCTROL:
        i4RetValue = adopt_CAMERA_HW_FeatureControl(pBuff);
        break;
    case KDIMGSENSORIOC_X_CONTROL:
        i4RetValue = adopt_CAMERA_HW_Control(pBuff);
        break;
    case KDIMGSENSORIOC_T_CLOSE:
        i4RetValue = adopt_CAMERA_HW_Close();
        break;
    case KDIMGSENSORIOC_T_CHECK_IS_ALIVE:
        i4RetValue = adopt_CAMERA_HW_CheckIsAlive();
        break;
    case KDIMGSENSORIOC_X_GET_SOCKET_POS:
        i4RetValue = kdGetSocketPostion((unsigned int *)pBuff);
        break;
    case KDIMGSENSORIOC_X_SET_I2CBUS:
        /* i4RetValue = kdSetI2CBusNum(*pIdx); */
        break;
    case KDIMGSENSORIOC_X_RELEASE_I2C_TRIGGER_LOCK:
        /* i4RetValue = kdReleaseI2CTriggerLock(); */
        break;

    case KDIMGSENSORIOC_X_SET_SHUTTER_GAIN_WAIT_DONE:
	/* i4RetValue = kdSensorSetExpGainWaitDone((int *)pBuff); */
        break;

    case KDIMGSENSORIOC_X_SET_CURRENT_SENSOR:
        i4RetValue = kdSetCurrentSensorIdx(*pIdx);
        break;

    case KDIMGSENSORIOC_X_SET_MCLK_PLL:
        i4RetValue = kdSetSensorMclk(pBuff);
        break;

    case KDIMGSENSORIOC_X_SET_GPIO:
        i4RetValue = kdSetSensorGpio(pBuff);
        break;

    case KDIMGSENSORIOC_X_GET_ISP_CLK:
#ifdef CONFIG_MTK_SMI_EXT
	PK_DBG("KDIMGSENSORIOC_X_GET_ISP_CLK current_mmsys_clk=%d\n", current_mmsys_clk);
	if(mmdvfs_get_stable_isp_clk() == MMSYS_CLK_HIGH)
		*(unsigned int*)pBuff = 364;
	else
#endif
		*(unsigned int*)pBuff = 286;
	break;
    default:
        PK_DBG("No such command %d\n",a_u4Command);
        PK_ERR("[M6_CAM] ioctl default unknown cmd=0x%08x dir=%u size=%u\n",
            a_u4Command, _IOC_DIR(a_u4Command), _IOC_SIZE(a_u4Command));
        i4RetValue = -EPERM;
        break;

    }

    if (_IOC_READ & _IOC_DIR(a_u4Command)) {
    if (copy_to_user((void __user *) a_u4Param , pBuff , _IOC_SIZE(a_u4Command))) {
        kfree(pBuff);
        PK_DBG("[CAMERA SENSOR] ioctl copy to user failed\n");
        i4RetValue =  -EFAULT;
        goto CAMERA_HW_Ioctl_EXIT;
    }
    }

    kfree(pBuff);
CAMERA_HW_Ioctl_EXIT:
    if (m6TraceIoctl || i4RetValue) {
        PK_ERR("[M6_CAM] ioctl exit cmd=0x%08x ret=%d size=%u\n",
            a_u4Command, i4RetValue, _IOC_SIZE(a_u4Command));
    }
    mutex_unlock(&kdCam_Mutex);
    return i4RetValue;
}
#ifdef CONFIG_MTK_SMI_EXT
int mmsys_clk_change_cb(int ori_clk_mode, int new_clk_mode)
{
	PK_DBG("mmsys_clk_change_cb ori: %d, new: %d, current_mmsys_clk %d\n", ori_clk_mode, new_clk_mode, current_mmsys_clk);
	current_mmsys_clk = new_clk_mode;
	return 1;
}
#endif
/*******************************************************************************
*
********************************************************************************/
/*  */
/* below is for linux driver system call */
/* change prefix or suffix only */
/*  */

/*******************************************************************************
 * RegisterCAMERA_HWCharDrv
 * #define
 * Main jobs:
 * 1.check for device-specified errors, device not ready.
 * 2.Initialize the device if it is opened for the first time.
 * 3.Update f_op pointer.
 * 4.Fill data structures into private_data
 * CAM_RESET
********************************************************************************/
static int CAMERA_HW_Open(struct inode *a_pstInode, struct file *a_pstFile)
{
    /* reset once in multi-open */
    if (atomic_read(&g_CamDrvOpenCnt) == 0) {
    /* default OFF state */
    /* MUST have */
    /* kdCISModulePowerOn(DUAL_CAMERA_MAIN_SENSOR,"",true,CAMERA_HW_DRVNAME1); */
    /* kdCISModulePowerOn(DUAL_CAMERA_SUB_SENSOR,"",true,CAMERA_HW_DRVNAME1); */

    /* kdCISModulePowerOn(DUAL_CAMERA_MAIN_SENSOR,"",false,CAMERA_HW_DRVNAME1); */
    /* kdCISModulePowerOn(DUAL_CAMERA_SUB_SENSOR,"",false,CAMERA_HW_DRVNAME1); */
#ifdef CONFIG_MTK_SMI_EXT
	mmdvfs_register_mmclk_switch_cb(mmsys_clk_change_cb, MMDVFS_CLIENT_ID_ISP);
#endif
    }

    /*  */
    atomic_inc(&g_CamDrvOpenCnt);

#ifdef CONFIG_MTK_CLKMGR
	enable_mux(MT_MUX_CAMTG, "CAMERA_SENSOR");
#else
#ifndef CONFIG_MTK_FPGA
	clk_prepare_enable(g_camclk_camtg_sel);
	clk_prepare_enable(g_camclk_scam_sel);
#endif
#endif

    return 0;
}

/*******************************************************************************
  * RegisterCAMERA_HWCharDrv
  * Main jobs:
  * 1.Deallocate anything that "open" allocated in private_data.
  * 2.Shut down the device on last close.
  * 3.Only called once on last time.
  * Q1 : Try release multiple times.
********************************************************************************/
static int CAMERA_HW_Release(struct inode *a_pstInode, struct file *a_pstFile)
{
    atomic_dec(&g_CamDrvOpenCnt);
#ifdef CONFIG_MTK_SMI_EXT
	current_mmsys_clk = MMSYS_CLK_MEDIUM;
#endif

#ifdef CONFIG_MTK_CLKMGR
	disable_mux(MT_MUX_CAMTG, "CAMERA_SENSOR");
#else
#ifndef CONFIG_MTK_FPGA
	clk_disable_unprepare(g_camclk_camtg_sel);
	clk_disable_unprepare(g_camclk_scam_sel);
#endif
#endif

return 0;
}

static const struct file_operations g_stCAMERA_HW_fops =
{
    .owner = THIS_MODULE,
    .open = CAMERA_HW_Open,
    .release = CAMERA_HW_Release,
    .unlocked_ioctl = CAMERA_HW_Ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = CAMERA_HW_Ioctl_Compat,
#endif

};

#define CAMERA_HW_DYNAMIC_ALLOCATE_DEVNO 1
/*******************************************************************************
* RegisterCAMERA_HWCharDrv
********************************************************************************/
inline static int RegisterCAMERA_HWCharDrv(void)
{

#if CAMERA_HW_DYNAMIC_ALLOCATE_DEVNO
    if (alloc_chrdev_region(&g_CAMERA_HWdevno, 0, 1, CAMERA_HW_DRVNAME1))
    {
    PK_DBG("[CAMERA SENSOR] Allocate device no failed\n");

    return -EAGAIN;
    }
#else
    if (register_chrdev_region(g_CAMERA_HWdevno , 1 , CAMERA_HW_DRVNAME1))
    {
    PK_DBG("[CAMERA SENSOR] Register device no failed\n");

    return -EAGAIN;
    }
#endif

    /* Allocate driver */
    g_pCAMERA_HW_CharDrv = cdev_alloc();

    if (NULL == g_pCAMERA_HW_CharDrv)
    {
    unregister_chrdev_region(g_CAMERA_HWdevno, 1);

    PK_DBG("[CAMERA SENSOR] Allocate mem for kobject failed\n");

    return -ENOMEM;
    }

    /* Attatch file operation. */
    cdev_init(g_pCAMERA_HW_CharDrv, &g_stCAMERA_HW_fops);

    g_pCAMERA_HW_CharDrv->owner = THIS_MODULE;

    /* Add to system */
    if (cdev_add(g_pCAMERA_HW_CharDrv, g_CAMERA_HWdevno, 1))
    {
    PK_DBG("[mt6516_IDP] Attatch file operation failed\n");

    unregister_chrdev_region(g_CAMERA_HWdevno, 1);

    return -EAGAIN;
    }

    sensor_class = class_create(THIS_MODULE, "sensordrv");
    if (IS_ERR(sensor_class)) {
    int ret = PTR_ERR(sensor_class);
    PK_DBG("Unable to create class, err = %d\n", ret);
    return ret;
    }
    sensor_device = device_create(sensor_class, NULL, g_CAMERA_HWdevno, NULL, CAMERA_HW_DRVNAME1);

    return 0;
}

/*******************************************************************************
* UnregisterCAMERA_HWCharDrv
********************************************************************************/
inline static void UnregisterCAMERA_HWCharDrv(void)
{
    /* Release char driver */
	if(g_pCAMERA_HW_CharDrv != NULL)
		cdev_del(g_pCAMERA_HW_CharDrv);

    unregister_chrdev_region(g_CAMERA_HWdevno, 1);
	 if (!IS_ERR(sensor_class)) {
		device_destroy(sensor_class, g_CAMERA_HWdevno);
		class_destroy(sensor_class);
	}
}
/*******************************************************************************
 * i2c relative start
********************************************************************************/
/*******************************************************************************
* CAMERA_HW_i2c_probe
********************************************************************************/
static int CAMERA_HW_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int i4RetValue = 0;
    PK_DBG("[CAMERA_HW] Attach I2C\n");

    /* get sensor i2c client */
    spin_lock(&kdsensor_drv_lock);
    g_pstI2Cclient = client;
    /* set I2C clock rate */
    do {} while (0); /* plain i2c: no ext_flag/timing fields */
    do {} while (0); /* plain i2c: no ext_flag */

    spin_unlock(&kdsensor_drv_lock);
    PK_ERR("[M6_CAM] i2c_probe main name=%s addr=0x%x adapter=%d timing=%u ext=0x%x flags=0x%x\n",
        client->name, client->addr, client->adapter ? client->adapter->nr : -1,
        0U, 0U, client->flags);

    /* The i2c ACK above checks the sensor wiring at every boot, but creating
     * /dev/kd_camera_hw here would let the always-running camera HAL
     * (provider@2.4) use a camera that is not powered. Defer the node and the
     * regulators to the armed path (forge_cam_arm, via echo 1 >
     * .../forge_cam_power). */
    if (!forge_cam_power_enabled()) {
        PK_ERR("[FORGE_CAM] INERT: /dev/kd_camera_hw NOT created, regulators not grabbed (forge_cam_power=0); arm to create\n");
        return 0;
    }

    /* Register char driver */
    i4RetValue = RegisterCAMERA_HWCharDrv();

    if (i4RetValue) {
    PK_ERR("[CAMERA_HW] register char device failed!\n");
    return i4RetValue;
    }

    /* spin_lock_init(&g_CamHWLock); */
#if !defined(CONFIG_MTK_LEGACY)
    Get_Cam_Regulator();
#endif

    PK_ERR("[M6_CAM] i2c_probe main done ret=%d client=%p\n",
        i4RetValue, g_pstI2Cclient);
    PK_DBG("[CAMERA_HW] Attached!!\n");
    return 0;
}


/*******************************************************************************
* CAMERA_HW_i2c_remove
********************************************************************************/
static int CAMERA_HW_i2c_remove(struct i2c_client *client)
{
	UnregisterCAMERA_HWCharDrv();
    return 0;
}

#ifdef CONFIG_OF
    static const struct of_device_id CAMERA_HW_i2c_of_ids[] = {
    { .compatible = "mediatek,camera_main", },
    {}
    };
#endif

struct i2c_driver CAMERA_HW_i2c_driver = {
    .probe = CAMERA_HW_i2c_probe,
    .remove = CAMERA_HW_i2c_remove,
    .driver = {
    .name = CAMERA_HW_DRVNAME1,
    .owner = THIS_MODULE,

#ifdef CONFIG_OF
    .of_match_table = CAMERA_HW_i2c_of_ids,
#endif
    },
    .id_table = CAMERA_HW_i2c_id,
};


/*******************************************************************************
* i2c relative end
*****************************************************************************/



/*******************************************************************************
 * RegisterCAMERA_HWCharDrv
 * #define
 * Main jobs:
 * 1.check for device-specified errors, device not ready.
 * 2.Initialize the device if it is opened for the first time.
 * 3.Update f_op pointer.
 * 4.Fill data structures into private_data
 * CAM_RESET
********************************************************************************/
static int CAMERA_HW_Open2(struct inode *a_pstInode, struct file *a_pstFile)
{
    /*  */
     if (atomic_read(&g_CamDrvOpenCnt2) == 0) {
     /* kdCISModulePowerOn(DUAL_CAMERA_MAIN_2_SENSOR,"",true,CAMERA_HW_DRVNAME2); */

    /* kdCISModulePowerOn(DUAL_CAMERA_MAIN_2_SENSOR,"",false,CAMERA_HW_DRVNAME2); */
    }
    atomic_inc(&g_CamDrvOpenCnt2);
    return 0;
}

/*******************************************************************************
  * RegisterCAMERA_HWCharDrv
  * Main jobs:
  * 1.Deallocate anything that "open" allocated in private_data.
  * 2.Shut down the device on last close.
  * 3.Only called once on last time.
  * Q1 : Try release multiple times.
********************************************************************************/
static int CAMERA_HW_Release2(struct inode *a_pstInode, struct file *a_pstFile)
{
    atomic_dec(&g_CamDrvOpenCnt2);

    return 0;
}


static const struct file_operations g_stCAMERA_HW_fops0 =
{
    .owner = THIS_MODULE,
    .open = CAMERA_HW_Open2,
    .release = CAMERA_HW_Release2,
    .unlocked_ioctl = CAMERA_HW_Ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = CAMERA_HW_Ioctl_Compat,
#endif

};



/*******************************************************************************
* RegisterCAMERA_HWCharDrv
********************************************************************************/
inline static int RegisterCAMERA_HWCharDrv2(void)
{
    struct device *sensor_device = NULL;
    UINT32 major;

#if CAMERA_HW_DYNAMIC_ALLOCATE_DEVNO
    if (alloc_chrdev_region(&g_CAMERA_HWdevno2, 0, 1, CAMERA_HW_DRVNAME2))
    {
    PK_DBG("[CAMERA SENSOR] Allocate device no failed\n");

    return -EAGAIN;
    }
#else
    if (register_chrdev_region(g_CAMERA_HWdevno2 , 1 , CAMERA_HW_DRVNAME2))
    {
    PK_DBG("[CAMERA SENSOR] Register device no failed\n");

    return -EAGAIN;
    }
#endif

    major = MAJOR(g_CAMERA_HWdevno2);
    g_CAMERA_HWdevno2 = MKDEV(major, 0);

    /* Allocate driver */
    g_pCAMERA_HW_CharDrv2 = cdev_alloc();

    if (NULL == g_pCAMERA_HW_CharDrv2)
    {
    unregister_chrdev_region(g_CAMERA_HWdevno2, 1);

    PK_DBG("[CAMERA SENSOR] Allocate mem for kobject failed\n");

    return -ENOMEM;
    }

    /* Attatch file operation. */
    cdev_init(g_pCAMERA_HW_CharDrv2, &g_stCAMERA_HW_fops0);

    g_pCAMERA_HW_CharDrv2->owner = THIS_MODULE;

    /* Add to system */
    if (cdev_add(g_pCAMERA_HW_CharDrv2, g_CAMERA_HWdevno2, 1))
    {
    PK_DBG("[mt6516_IDP] Attatch file operation failed\n");

    unregister_chrdev_region(g_CAMERA_HWdevno2, 1);

    return -EAGAIN;
    }

    sensor2_class = class_create(THIS_MODULE, "sensordrv2");
    if (IS_ERR(sensor2_class)) {
    int ret = PTR_ERR(sensor2_class);
    PK_DBG("Unable to create class, err = %d\n", ret);
    return ret;
    }
    sensor_device = device_create(sensor2_class, NULL, g_CAMERA_HWdevno2, NULL, CAMERA_HW_DRVNAME2);

    return 0;
}

inline static void UnregisterCAMERA_HWCharDrv2(void)
{
    /* Release char driver */
	if(g_pCAMERA_HW_CharDrv2 != NULL)
		cdev_del(g_pCAMERA_HW_CharDrv2);

    unregister_chrdev_region(g_CAMERA_HWdevno2, 1);
	if (!IS_ERR(sensor2_class)) {
		device_destroy(sensor2_class, g_CAMERA_HWdevno2);
		class_destroy(sensor2_class);
	}
}


/*******************************************************************************
* CAMERA_HW_i2c_probe
********************************************************************************/
static int CAMERA_HW_i2c_probe2(struct i2c_client *client, const struct i2c_device_id *id)
{
    int i4RetValue = 0;
    PK_DBG("[CAMERA_HW] Attach I2C0\n");

    spin_lock(&kdsensor_drv_lock);

    /* get sensor i2c client */
    g_pstI2Cclient2 = client;

    /* set I2C clock rate */
    do {} while (0); /* plain i2c: no ext_flag/timing fields */
    do {} while (0); /* plain i2c: no ext_flag */
    spin_unlock(&kdsensor_drv_lock);
    PK_ERR("[M6_CAM] i2c_probe bus2 name=%s addr=0x%x adapter=%d timing=%u ext=0x%x flags=0x%x\n",
        client->name, client->addr, client->adapter ? client->adapter->nr : -1,
        0U, 0U, client->flags);

    /* Keep the bus2 ACK above, defer the node until armed. */
    if (!forge_cam_power_enabled()) {
        PK_ERR("[FORGE_CAM] INERT: /dev/kd_camera_hw_bus2 NOT created (forge_cam_power=0); arm to create\n");
        return 0;
    }

    /* Register char driver */
    i4RetValue = RegisterCAMERA_HWCharDrv2();

    if (i4RetValue) {
    PK_ERR("[CAMERA_HW] register char device failed!\n");
    return i4RetValue;
    }

    /* spin_lock_init(&g_CamHWLock); */

    PK_ERR("[M6_CAM] i2c_probe bus2 done ret=%d client=%p\n",
        i4RetValue, g_pstI2Cclient2);
    PK_DBG("[CAMERA_HW] Attached!!\n");
    return 0;
}

/*******************************************************************************
* CAMERA_HW_i2c_remove
********************************************************************************/
static int CAMERA_HW_i2c_remove2(struct i2c_client *client)
{
	UnregisterCAMERA_HWCharDrv2();
    return 0;
}


/*******************************************************************************
* I2C Driver structure
********************************************************************************/
#ifdef CONFIG_OF
    static const struct of_device_id CAMERA_HW2_i2c_driver_of_ids[] = {
	{ .compatible = "mediatek,camera_sub", },
	{}
    };
#endif

struct i2c_driver CAMERA_HW_i2c_driver2 = {
    .probe = CAMERA_HW_i2c_probe2,
    .remove = CAMERA_HW_i2c_remove2,
    .driver = {
    .name = CAMERA_HW_DRVNAME2,
    .owner = THIS_MODULE,
#if 1
#ifdef CONFIG_OF
    .of_match_table = CAMERA_HW2_i2c_driver_of_ids,
#endif
#endif
    },
    .id_table = CAMERA_HW_i2c_id2,
};

/*******************************************************************************
* Knob-gated in-kernel sensor detect.
*
* Stock detect is HAL-driven: SET_MCLK_PLL -> SET_DRIVER -> CHECK_IS_ALIVE
* ioctls per sensorlist candidate. This trigger replays that ioctl flow from
* the kernel side, with [FORGE_CAM] markers:
*
*   echo 1 > /sys/module/kd_camera_hw/parameters/forge_cam_power
*   echo 1 > /sys/module/kd_sensorlist/parameters/forge_cam_detect   # main
*   echo 2 > .../forge_cam_detect                                    # sub
*   (3 = both; write refused with -EPERM while forge_cam_power=0)
*
* Order: forge_cam_scan=3 is release and prints nothing unless a hold is
* already up, and forge_cam_detect drops the rails when it finishes. The only
* correct order is
*     forge_cam_scan=2  (hold up)  ->  reads  ->  forge_cam_scan=3  (release)
* with no detect in between: a detect between them releases the hold, and
* the release then reports on an unpowered bus.
*
* A candidate only counts as found when its driver's CHECK_SENSOR_ID I2C
* read returns the real ID. forge_cam_mclk_parent selects the camtg root
* (0 = clk26m 26MHz, 1 = univpll_d26 48MHz = LK default and stock's
* CAM_PLL_48_GROUP, 2 = univpll2_d2 52MHz, -1 = leave untouched) and is
* applied by detect, scan and hold alike.
********************************************************************************/
extern int forge_cam_power_enabled(void);	/* camera_hw/kd_camera_hw.c */
extern int forge_cam_mclk_init(struct platform_device *pdev); /* mt_cam_mclk.c */

/* camtg parent select. LK parks camtg on univpll_d26 (CLK_CFG_2 =
 * 0x00010001 at boot).
 *
 *   -1 = leave the mux untouched (honest "no write", for a true baseline)
 *    0 = clk26m       26 MHz  <- both fitted sensors declare .mclk = 24
 *    1 = univpll_d26  48 MHz  <- LK default and stock's CAM_PLL_48_GROUP
 *    2 = univpll2_d2  52 MHz
 *
 * A module parameter plus a runtime clk_set_parent: a reboot restores
 * whatever LK sets.
 */
#define FORGE_CAM_PARENT_KEEP	(-1)
static int forge_cam_mclk_parent = 1;

static int forge_cam_apply_camtg_parent(const char *tag)
{
	struct clk *p;
	const char *name;
	unsigned long before, after;
	int ret;

	if (IS_ERR_OR_NULL(g_camclk_camtg_sel)) {
		pr_err("[FORGE_CAM] %s camtg: TOP_CAMTG_SEL never resolved - parent NOT set\n",
		       tag);
		return -ENODEV;
	}
	before = clk_get_rate(g_camclk_camtg_sel);

	if (forge_cam_mclk_parent == FORGE_CAM_PARENT_KEEP) {
		pr_err("[FORGE_CAM] %s camtg: parent LEFT UNTOUCHED, rate=%lu Hz\n",
		       tag, before);
		return 0;
	}
	switch (forge_cam_mclk_parent) {
	case 0:
		p = g_camclk_clk26m;
		name = "clk26m";
		break;
	case 1:
		p = g_camclk_univpll_d26;
		name = "univpll_d26";
		break;
	case 2:
		p = g_camclk_univpll2_d2;
		name = "univpll2_d2";
		break;
	default:
		pr_err("[FORGE_CAM] %s camtg: bad parent %d (-1=keep 0=clk26m 1=univpll_d26 2=univpll2_d2)\n",
		       tag, forge_cam_mclk_parent);
		return -EINVAL;
	}
	if (IS_ERR_OR_NULL(p)) {
		pr_err("[FORGE_CAM] %s camtg: parent %s (%d) UNRESOLVED - REFUSING; rate stays %lu Hz\n",
		       tag, name, forge_cam_mclk_parent, before);
		return -ENODEV;
	}
	ret = clk_set_parent(g_camclk_camtg_sel, p);
	after = clk_get_rate(g_camclk_camtg_sel);
	if (ret) {
		pr_err("[FORGE_CAM] %s camtg: clk_set_parent(%s) FAILED %d; rate %lu -> %lu Hz\n",
		       tag, name, ret, before, after);
		return ret;
	}
	pr_err("[FORGE_CAM] %s camtg: parent=%s sel=%d rate %lu -> %lu Hz\n",
	       tag, name, forge_cam_mclk_parent, before, after);
	return 0;
}

/* Applying on write makes the knob an instrument on its own: one echo, one
 * marker with the real rate, no detect needed to learn what the mux does. */
static int forge_cam_mclk_parent_set(const char *val,
				     const struct kernel_param *kp)
{
	int req, ret;

	ret = kstrtoint(val, 0, &req);
	if (ret)
		return ret;
	if (req < FORGE_CAM_PARENT_KEEP || req > 2)
		return -EINVAL;
	*(int *)kp->arg = req;
	return forge_cam_apply_camtg_parent("KNOB");
}

static const struct kernel_param_ops forge_cam_mclk_parent_ops = {
	.set = forge_cam_mclk_parent_set,
	.get = param_get_int,
};
module_param_cb(forge_cam_mclk_parent, &forge_cam_mclk_parent_ops,
		&forge_cam_mclk_parent, 0644);
MODULE_PARM_DESC(forge_cam_mclk_parent, "m681 camtg root: -1=leave untouched, 0=clk26m 26MHz, 1=univpll_d26 48MHz (LK default, stock group), 2=univpll2_d2 52MHz. Applied on write AND by detect/scan/hold.");

static int forge_cam_detect_socket(int socket)
{
	ACDK_KD_SENSOR_INIT_FUNCTION_STRUCT *pSensorList = NULL;
	unsigned int idxArr[KDIMGSENSOR_MAX_INVOKE_DRIVERS];
	u32 listIdx;
	int found = -1;

	if (kdGetSensorInitFuncList(&pSensorList)) {
		pr_err("[FORGE_CAM] DETECT socket=%d: no sensor list\n", socket);
		return -EIO;
	}
	/* main is i2c bus 1 (g_pstI2Cclient), sub and main2 bus 2 (kdSetDriver) */
	if (!(socket == DUAL_CAMERA_MAIN_SENSOR ? g_pstI2Cclient : g_pstI2Cclient2)) {
		pr_err("[FORGE_CAM] DETECT socket=%d: no camera i2c client probed (%s DT node), nothing to ask\n",
		       socket, socket == DUAL_CAMERA_MAIN_SENSOR ? "camera_main" : "camera_sub (bus 2)");
		return -ENODEV;
	}
	for (listIdx = 0; listIdx < MAX_NUM_OF_SUPPORT_SENSOR; listIdx++) {
		/* the end of a list is the {0, {0}, NULL} row; the l681 list has
		 * HAL slots without a driver before it (FORGE_L681_CAM_LIST) */
		if (!pSensorList[listIdx].SensorId)
			break;
		if (!pSensorList[listIdx].SensorInit)
			continue;
		idxArr[0] = ((unsigned int)socket << KDIMGSENSOR_DUAL_SHIFT) | listIdx;
		idxArr[1] = 0;	/* invoke[1] = DUAL_CAMERA_NONE_SENSOR */
		pr_err("[FORGE_CAM] DETECT try socket=%d idx=%u name=%s\n",
		       socket, listIdx, pSensorList[listIdx].drvname);
		if (kdSetDriver(idxArr)) {
			pr_err("[FORGE_CAM] DETECT socket=%d idx=%u: kdSetDriver failed\n",
			       socket, listIdx);
			continue;
		}
		if (adopt_CAMERA_HW_CheckIsAlive() == 0) {
			pr_err("[FORGE_CAM] DETECT FOUND socket=%d idx=%u name=%s expected_id=0x%08x\n",
			       socket, listIdx, pSensorList[listIdx].drvname,
			       pSensorList[listIdx].SensorId);
			found = (int)listIdx;
			break;
		}
	}
	if (found < 0)
		pr_err("[FORGE_CAM] DETECT socket=%d: NO candidate answered. Check the [FORGE_CAM] MCLK/RAIL markers above FIRST - a clock/rail failure NAKs every candidate and proves nothing about sensor identity\n",
		       socket);
	return found;
}

/* i2c sweep inside the camera power window. A userspace sweep on
 * /dev/i2c-2 cannot do this: the rails are only up for the ~45 ms a
 * candidate's power sequence holds them, so the bus would read empty. This
 * powers the main socket with a real sensor's sequence, sweeps every 7-bit
 * address on the client's own adapter, and powers down. A pre-power pass
 * runs first so anything that is permanently powered (the EEPROMs) can be
 * told apart from anything the rails bring up.
 * The i2c_msg is memset before use: uninitialised fields corrupt the
 * transfer. */
/* Which socket the hold powers:
 *
 *   forge_cam_hold_socket  1 = main (default, today's behaviour), 2 = sub
 *   forge_cam_hold_idx     candidate index into the sensor list, i.e. WHICH
 *                          driver's power sequence runs.  Default 1 = plain
 *                          ov13853, correct for main.  For sub, take the index
 *                          from any DETECT run: it prints "try socket=N idx=U
 *                          name=..." for every candidate.
 *
 * The chosen driver's name is printed at hold-on, so a wrong index is
 * visible in the log. */
static int forge_cam_hold_socket = 1;
module_param(forge_cam_hold_socket, int, 0644);
MODULE_PARM_DESC(forge_cam_hold_socket, "m681: socket the next forge_cam_scan=2 powers -- 1=main (default), 2=sub/front");

static int forge_cam_hold_idx = 1;
module_param(forge_cam_hold_idx, int, 0644);
MODULE_PARM_DESC(forge_cam_hold_idx, "m681: sensor-list candidate index whose power sequence the hold runs (default 1 = plain ov13853, correct for main; for sub take the idx a DETECT run printed)");

/* Latched at hold-on and reused at hold-off.  Powering socket 2 up and then
 * socket 1 down would strand the front rails exactly as a failed power-on
 * would -- the same class of stranding the ret != ERROR_NONE branch guards. */
static int forge_cam_held_socket = 1;
static int forge_cam_held_idx = 1;

static bool forge_cam_held;
static void forge_cam_hold(bool on);

/* Auto-release.  Mode 2 energises AVDD/DOVDD/DVDD on the camera module and
 * nothing in the kernel is obliged to take them down again: there is no
 * process to exit, no fd to close, no PM hook on this path.  A session that
 * ends without a mode-3 write would leave the module powered until reboot,
 * heating it and draining the pack.  The timer is the backstop; it is
 * cancelled by an explicit release, and re-armed by each new hold. */
#define FORGE_CAM_HOLD_MAX_MS (10 * 60 * 1000)
static void forge_cam_hold_expire(struct work_struct *w);
static DECLARE_DELAYED_WORK(forge_cam_hold_work, forge_cam_hold_expire);

static void forge_cam_hold_expire(struct work_struct *w)
{
	pr_err("[FORGE_CAM] HOLD auto-release after %d min: the rails were still up\n",
	       FORGE_CAM_HOLD_MAX_MS / 60000);
	forge_cam_hold(false);
}

static int forge_cam_sweep_once(struct i2c_adapter *adap, const char *tag)
{
	int a, found = 0;

	for (a = 0x08; a <= 0x77; a++) {
		struct i2c_msg msg;
		u8 b = 0;
		int ret;

		memset(&msg, 0, sizeof(msg));
		msg.addr = (u16)a;
		msg.flags = I2C_M_RD;
		msg.len = 1;
		msg.buf = &b;
		ret = i2c_transfer(adap, &msg, 1);
		if (ret == 1) {
			pr_err("[FORGE_CAM] SCAN[%s] ACK addr=0x%02x (7-bit) = 0x%02x (8-bit)\n",
			       tag, a, a << 1);
			found++;
		}
	}
	pr_err("[FORGE_CAM] SCAN[%s] done: %d device(s) on adapter %d\n",
	       tag, found, adap->nr);
	return found;
}

static void forge_cam_scan_bus(void)
{
	unsigned int idxArr[2];
	struct i2c_adapter *adap;
	int pre, post;

	if (!g_pstI2Cclient || !g_pstI2Cclient->adapter) {
		pr_err("[FORGE_CAM] SCAN refused: no main i2c client bound\n");
		return;
	}
	adap = g_pstI2Cclient->adapter;

	/* The mux is set on every leg, so a sweep runs at the same camtg root
	 * as the detect it is compared with. */
	forge_cam_apply_camtg_parent("SCAN");

	pre = forge_cam_sweep_once(adap, "rails-off");

	/* idx 1 = ov13853mipiraw: a real entry, so a real power sequence runs.
	 * kdCam_Mutex for the same reason as forge_cam_hold(): kdSetDriver
	 * rewrites driver-wide invoke state that the ioctl path owns, and the
	 * camera HAL is running on this device. */
	mutex_lock(&kdCam_Mutex);
	idxArr[0] = ((unsigned int)DUAL_CAMERA_MAIN_SENSOR << KDIMGSENSOR_DUAL_SHIFT) | 1;
	idxArr[1] = 0;
	if (kdSetDriver(idxArr)) {
		pr_err("[FORGE_CAM] SCAN: kdSetDriver failed; rails-on pass skipped\n");
		mutex_unlock(&kdCam_Mutex);
		return;
	}
	kdModulePowerOn((CAMERA_DUAL_CAMERA_SENSOR_ENUM *)g_invokeSocketIdx,
			g_invokeSensorNameStr, true, CAMERA_HW_DRVNAME1);
	mutex_unlock(&kdCam_Mutex);
	mDELAY(50);
	post = forge_cam_sweep_once(adap, "rails-on");
	kdModulePowerOn((CAMERA_DUAL_CAMERA_SENSOR_ENUM *)g_invokeSocketIdx,
			g_invokeSensorNameStr, false, CAMERA_HW_DRVNAME1);
	/* This power-down drops the rails whether or not a hold was in effect, so
	 * the flag has to follow the hardware.  Leaving it set would make the next
	 * mode-2 a no-op ("already ON") and the probe would then sweep an
	 * unpowered bus and read "nothing there" -- indistinguishable from proof
	 * that the module is absent. */
	if (forge_cam_held) {
		forge_cam_held = false;
		cancel_delayed_work(&forge_cam_hold_work);
		pr_err("[FORGE_CAM] SCAN: the sweep's power-down also cleared the HOLD state\n");
	}

	pr_err("[FORGE_CAM] SCAN verdict: rails-off=%d rails-on=%d -- a rails-on count of 0 with a nonzero rails-off count means the bus and controller are fine and the sensor domain is not answering\n",
	       pre, post);
}

static void forge_cam_hold(bool on)
{
	unsigned int idxArr[2];
	int ret;

	if (on == forge_cam_held) {
		pr_err("[FORGE_CAM] HOLD already %s\n", on ? "ON" : "OFF");
		return;
	}

	/* kdSetDriver rewrites the driver-wide invoke state (g_invokeSocketIdx,
	 * g_invokeSensorNameStr, g_pSensorFunc).  The ioctl path takes
	 * kdCam_Mutex around exactly that, and the camera HAL runs on this
	 * device at every boot, so a sysfs write must take it too or the HAL can
	 * be left driving a different sensor's function table than it selected. */
	mutex_lock(&kdCam_Mutex);

	/*  on = take the knobs; off = use whatever was latched at
	 * hold-on, so a knob edited mid-hold cannot power down the other
	 * socket and strand this one. */
	if (on) {
		if (forge_cam_hold_socket != DUAL_CAMERA_MAIN_SENSOR &&
		    forge_cam_hold_socket != DUAL_CAMERA_SUB_SENSOR) {
			pr_err("[FORGE_CAM] HOLD: bad forge_cam_hold_socket=%d (1=main 2=sub)\n",
			       forge_cam_hold_socket);
			mutex_unlock(&kdCam_Mutex);
			return;
		}
		if (forge_cam_hold_idx < 0 ||
		    forge_cam_hold_idx >= MAX_NUM_OF_SUPPORT_SENSOR) {
			pr_err("[FORGE_CAM] HOLD: bad forge_cam_hold_idx=%d\n",
			       forge_cam_hold_idx);
			mutex_unlock(&kdCam_Mutex);
			return;
		}
		forge_cam_held_socket = forge_cam_hold_socket;
		forge_cam_held_idx = forge_cam_hold_idx;
		/* Same root as detect and scan: until now only detect set the
		 * mux, so a hold inherited whatever the last leg left. */
		forge_cam_apply_camtg_parent("HOLD");
	}

	idxArr[0] = ((unsigned int)forge_cam_held_socket << KDIMGSENSOR_DUAL_SHIFT) |
		    (unsigned int)forge_cam_held_idx;
	idxArr[1] = 0;
	if (kdSetDriver(idxArr)) {
		pr_err("[FORGE_CAM] HOLD: kdSetDriver failed (socket=%d idx=%d)\n",
		       forge_cam_held_socket, forge_cam_held_idx);
		mutex_unlock(&kdCam_Mutex);
		return;
	}
	ret = kdModulePowerOn((CAMERA_DUAL_CAMERA_SENSOR_ENUM *)g_invokeSocketIdx,
			      g_invokeSensorNameStr, on, CAMERA_HW_DRVNAME1);
	if (ret != ERROR_NONE) {
		/* Do NOT record a state the hardware may not be in.  Believing a
		 * failed power-on would make a later release a no-op and strand
		 * whatever rails did come up. */
		pr_err("[FORGE_CAM] HOLD %s FAILED: kdModulePowerOn ret=%d; state unchanged (still %s)\n",
		       on ? "ON" : "OFF", ret, forge_cam_held ? "ON" : "OFF");
		mutex_unlock(&kdCam_Mutex);
		return;
	}
	forge_cam_held = on;
	mutex_unlock(&kdCam_Mutex);

	if (on)
		schedule_delayed_work(&forge_cam_hold_work,
				      msecs_to_jiffies(FORGE_CAM_HOLD_MAX_MS));
	else
		cancel_delayed_work(&forge_cam_hold_work);

	/*  name the socket AND its bus.  The old text always said
	 * "main ... /dev/i2c-2", which is what let a front-bus sweep be read
	 * as a front-camera result. */
	pr_err("[FORGE_CAM] HOLD %s: socket=%d (%s) idx=%d name=%s rails/MCLK/reset %s; sweep %s from userspace\n",
	       on ? "ON" : "OFF", forge_cam_held_socket,
	       forge_cam_held_socket == DUAL_CAMERA_SUB_SENSOR ? "SUB/front" : "MAIN/rear",
	       forge_cam_held_idx, g_invokeSensorNameStr[0],
	       on ? "held up" : "released",
	       forge_cam_held_socket == DUAL_CAMERA_SUB_SENSOR ? "/dev/i2c-1" : "/dev/i2c-2");
}

static int forge_cam_scan_set(const char *val, const struct kernel_param *kp)
{
	int req = 0, ret;

	ret = kstrtoint(val, 0, &req);
	if (ret)
		return ret;
	if (req == 0) {
		*(int *)kp->arg = req;
		return 0;
	}
	/* Mode 3 is the RELEASE, and it must never be gated on forge_cam_power.
	 * Refusing it there created a trap: disarming the power knob after a hold
	 * left no software path to drop the rails, so the "make it safe" action
	 * did the opposite and the module stayed energised until reboot. */
	if (req != 3 && !forge_cam_power_enabled()) {
		/* Store nothing: the knob must not read back a mode that never ran. */
		pr_err("[FORGE_CAM] SCAN refused: forge_cam_power=0 (the sequence would be gated off and the sweep would read an unpowered bus)\n");
		return -EPERM;
	}
	*(int *)kp->arg = req;
	if (req == 1) {
		forge_cam_scan_bus();
	} else if (req == 2) {
		/* HOLD: power the module and LEAVE the rails up, so /dev/i2c-2 can
		 * be driven from userspace.  The sweep proved three devices wake on
		 * this domain (VCM 0x0c, EEPROMs 0x50/0x58) while the sensor stays
		 * silent, and identifying the module now means reading its EEPROM --
		 * which a 45 ms power window cannot support. */
		forge_cam_hold(true);
	} else if (req == 3) {
		forge_cam_hold(false);
	} else {
		pr_err("[FORGE_CAM] SCAN: unknown request %d (1=sweep 2=hold-on 3=hold-off)\n", req);
		return -EINVAL;
	}
	return 0;
}

static const struct kernel_param_ops forge_cam_scan_ops = {
	.set = forge_cam_scan_set,
	.get = param_get_int,
};
static int forge_cam_scan;
module_param_cb(forge_cam_scan, &forge_cam_scan_ops, &forge_cam_scan, 0644);
MODULE_PARM_DESC(forge_cam_scan, "m681: 1=sweep the main camera i2c adapter (rails off, then held on); 2=power the module and HOLD the rails up for userspace /dev/i2c-2 work; 3=release (needs forge_cam_power=1)");

static int forge_cam_detect_set(const char *val, const struct kernel_param *kp)
{
	int req = 0, ret;

	ret = kstrtoint(val, 0, &req);
	if (ret)
		return ret;
	*(int *)kp->arg = req;
	if (req == 0)
		return 0;
	if (!forge_cam_power_enabled()) {
		pr_err("[FORGE_CAM] DETECT refused: forge_cam_power=0 (every candidate would 'NAK' at the gate, poisoning the log). Enable the power gate first\n");
		return -EPERM;
	}
	forge_cam_apply_camtg_parent("DETECT");
	if (req & 1)
		forge_cam_detect_socket(DUAL_CAMERA_MAIN_SENSOR);
	if (req & 2)
		forge_cam_detect_socket(DUAL_CAMERA_SUB_SENSOR);
	return 0;
}

static const struct kernel_param_ops forge_cam_detect_ops = {
	.set = forge_cam_detect_set,
	.get = param_get_int,
};
static int forge_cam_detect;
module_param_cb(forge_cam_detect, &forge_cam_detect_ops, &forge_cam_detect, 0644);
MODULE_PARM_DESC(forge_cam_detect, "m681: write 1=detect main, 2=detect sub, 3=both (needs forge_cam_power=1)");

/*******************************************************************************
* forge_cam_arm: the only place that creates the camera /dev nodes and maps
* the ISP MCLK provider. Called from the forge_cam_power setter
* (camera_hw/kd_camera_hw.c) on the 0->non-0 transition, i.e. from an explicit
* `echo 1 > /sys/module/kd_camera_hw/parameters/forge_cam_power`. Until then a
* default boot does nothing here. Idempotent. Enables no clock/domain:
* forge_cam_mclk_init only maps + resolves; SENINF bit29 / SCP_SYS_ISP enable
* happens later in the ISP_MCLK*_EN power path, itself gated.
********************************************************************************/
int forge_cam_arm(void)
{
	static bool armed;
	int ret;

	if (armed)
		return 0;

	if (pCamerahw_platform_device)
		forge_cam_mclk_init(pCamerahw_platform_device);
	else
		PK_ERR("[FORGE_CAM] ARM: no platform_device saved - MCLK provider NOT mapped\n");

	ret = RegisterCAMERA_HWCharDrv();
	if (ret) {
		PK_ERR("[FORGE_CAM] ARM: RegisterCAMERA_HWCharDrv failed %d - /dev/kd_camera_hw NOT created\n", ret);
		return ret;
	}
	RegisterCAMERA_HWCharDrv2();
#if !defined(CONFIG_MTK_LEGACY)
	Get_Cam_Regulator();
#endif
	armed = true;
	PK_ERR("[FORGE_CAM] ARMED: /dev/kd_camera_hw{,_bus2} created, MCLK provider mapped, regulators resolved; power stays gated by forge_cam_power\n");
	return 0;
}

/*******************************************************************************
* CAMERA_HW_probe
********************************************************************************/
static int CAMERA_HW_probe(struct platform_device *pdev)
{
#ifndef CONFIG_MTK_CLKMGR
		Get_ccf_clk(pdev);
#endif
	/* Do not map the MCLK provider or create any /dev node at boot.
	 * forge_cam_mclk_init (SENINF iomap + SCP_SYS_ISP clock resolve) and the
	 * char-device creation run only when armed (forge_cam_arm, from the
	 * forge_cam_power 0->1 setter), so a default boot touches no camera
	 * clock/domain and exposes no node to the camera HAL. i2c_add_driver
	 * stays, so the sensor i2c ACK still prints. */
	pr_err("[FORGE_CAM] Phase-A INERT: forge_cam_power=0 -> no /dev/kd_camera_hw*, no ISP MCLK/domain mapped. echo 1 > /sys/module/kd_camera_hw/parameters/forge_cam_power to arm\n");

#if !defined(CONFIG_MTK_LEGACY)/*GPIO Pin control*/
	mtkcam_gpio_init(pdev);
#endif

	pCamerahw_platform_device = pdev;
    return i2c_add_driver(&CAMERA_HW_i2c_driver);
}

/*******************************************************************************
* CAMERA_HW_remove()
********************************************************************************/
static int CAMERA_HW_remove(struct platform_device *pdev)
{
    i2c_del_driver(&CAMERA_HW_i2c_driver);
    return 0;
}

/*******************************************************************************
*CAMERA_HW_suspend()
********************************************************************************/
static int CAMERA_HW_suspend(struct platform_device *pdev, pm_message_t mesg)
{
    return 0;
}

/*******************************************************************************
  * CAMERA_HW_DumpReg_To_Proc()
  * Used to dump some critical sensor register
  ********************************************************************************/
static int CAMERA_HW_resume(struct platform_device *pdev)
{
    return 0;
}

/*******************************************************************************
* CAMERA_HW_remove
********************************************************************************/
static int CAMERA_HW_probe2(struct platform_device *pdev)
{
    return i2c_add_driver(&CAMERA_HW_i2c_driver2);
}

/*******************************************************************************
* CAMERA_HW_remove()
********************************************************************************/
static int CAMERA_HW_remove2(struct platform_device *pdev)
{
    i2c_del_driver(&CAMERA_HW_i2c_driver2);
    return 0;
}

static int CAMERA_HW_suspend2(struct platform_device *pdev, pm_message_t mesg)
{
    return 0;
}

/*******************************************************************************
  * CAMERA_HW_DumpReg_To_Proc()
  * Used to dump some critical sensor register
  ********************************************************************************/
static int CAMERA_HW_resume2(struct platform_device *pdev)
{
    return 0;
}

/*=======================================================================
  * platform driver
  *=======================================================================*/
/* It seems we don't need to use device tree to register device cause we just use i2C part */
/* You can refer to CAMERA_HW_probe & CAMERA_HW_i2c_probe */

#if 1
#ifdef CONFIG_OF
static const struct of_device_id CAMERA_HW2_of_ids[] = {
    { .compatible = "mediatek,camera_hw2", },
    {}
};
#endif
#endif

static struct platform_driver g_stCAMERA_HW_Driver2 = {
    .probe      = CAMERA_HW_probe2,
    .remove     = CAMERA_HW_remove2,
    .suspend    = CAMERA_HW_suspend2,
    .resume     = CAMERA_HW_resume2,
    .driver     = {
    .name   = "image_sensor_bus2",
    .owner  = THIS_MODULE,
#if 1
#ifdef CONFIG_OF
    .of_match_table = CAMERA_HW2_of_ids,
#endif
#endif

    }
};

/*******************************************************************************
* iWriteTriggerReg
********************************************************************************/
#if 0
int iWriteTriggerReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId)
{
    int  i4RetValue = 0;
    int u4Index = 0;
    u8 *puDataInBytes = (u8 *)&a_u4Data;
    int retry = 3;
    char puSendCmd[6] = {(char)(a_u2Addr >> 8) , (char)(a_u2Addr & 0xFF) , 0 , 0 , 0 , 0};



    SET_I2CBUS_FLAG(gI2CBusNum);

    if (gI2CBusNum == SUPPORT_I2C_BUS_NUM1) {
    spin_lock(&kdsensor_drv_lock);
    g_pstI2Cclient->addr = (i2cId >> 1);
    spin_unlock(&kdsensor_drv_lock);
    }
    else {
    spin_lock(&kdsensor_drv_lock);
    g_pstI2Cclient2->addr = (i2cId >> 1);
    spin_unlock(&kdsensor_drv_lock);
    }


    if (a_u4Bytes > 2) {
    PK_DBG("[CAMERA SENSOR] exceed 2 bytes\n");
    return -1;
    }

    if (a_u4Data >> (a_u4Bytes << 3)) {
    PK_DBG("[CAMERA SENSOR] warning!! some data is not sent!!\n");
    }

    for (u4Index = 0; u4Index < a_u4Bytes; u4Index += 1) {
    puSendCmd[(u4Index + 2)] = puDataInBytes[(a_u4Bytes - u4Index-1)];
    }

    do {
    if (gI2CBusNum == SUPPORT_I2C_BUS_NUM1) {
        i4RetValue = mt_i2c_master_send(g_pstI2Cclient, puSendCmd, (a_u4Bytes + 2), I2C_3DCAMERA_FLAG);
        if (i4RetValue < 0) {
        PK_DBG("[CAMERA SENSOR][ERROR]set i2c bus 1 master fail\n");
        CLEAN_I2CBUS_FLAG(gI2CBusNum);
        break;
        }
    }
    else {
        i4RetValue = mt_i2c_master_send(g_pstI2Cclient2, puSendCmd, (a_u4Bytes + 2), I2C_3DCAMERA_FLAG);
        if (i4RetValue < 0) {
        PK_DBG("[CAMERA SENSOR][ERROR]set i2c bus 0 master fail\n");
        CLEAN_I2CBUS_FLAG(gI2CBusNum);
        break;
        }
    }

    if (i4RetValue != (a_u4Bytes + 2)) {
        PK_DBG("[CAMERA SENSOR] I2C send failed addr = 0x%x, data = 0x%x !!\n", a_u2Addr, a_u4Data);
    }
    else {
        break;
    }
    uDELAY(50);
    } while ((retry--) > 0);

    return i4RetValue;
}
#endif
#if 0  /* linux-3.10 procfs API changed */
/*******************************************************************************
  * CAMERA_HW_Read_Main_Camera_Status()
  * Used to detect main camera status
  ********************************************************************************/
static int  CAMERA_HW_Read_Main_Camera_Status(char *page, char **start, off_t off,
                                               int count, int *eof, void *data)
{
    char *p = page;
    int len = 0;
    p += sprintf(page, "%d\n", g_SensorExistStatus[0]);

    PK_DBG("g_SensorExistStatus[0] = %d\n", g_SensorExistStatus[0]);
    *start = page + off;
    len = p - page;
    if (len > off)
    len -= off;
    else
    len = 0;
    return len < count ? len  : count;

}
/*******************************************************************************
  * CAMERA_HW_Read_Sub_Camera_Status()
  * Used to detect main camera status
  ********************************************************************************/
static int  CAMERA_HW_Read_Sub_Camera_Status(char *page, char **start, off_t off,
                                               int count, int *eof, void *data)
{
    char *p = page;
    int len = 0;
    p += sprintf(page, "%d\n", g_SensorExistStatus[1]);

    PK_DBG(" g_SensorExistStatus[1] = %d\n", g_SensorExistStatus[1]);
    *start = page + off;
    len = p - page;
    if (len > off)
    len -= off;
    else
    len = 0;
    return len < count ? len  : count;

}
/*******************************************************************************
  * CAMERA_HW_Read_3D_Camera_Status()
  * Used to detect main camera status
  ********************************************************************************/
static int  CAMERA_HW_Read_3D_Camera_Status(char *page, char **start, off_t off,
                                               int count, int *eof, void *data)
{
    char *p = page;
    int len = 0;
    p += sprintf(page, "%d\n", g_SensorExistStatus[2]);

    PK_DBG("g_SensorExistStatus[2] = %d\n", g_SensorExistStatus[2]);
    *start = page + off;
    len = p - page;
    if (len > off)
    len -= off;
    else
    len = 0;
    return len < count ? len  : count;

}
#endif


/*******************************************************************************
  * CAMERA_HW_DumpReg_To_Proc()
  * Used to dump some critical sensor register
  ********************************************************************************/
static ssize_t  CAMERA_HW_DumpReg_To_Proc(struct file *file, char __user *data, size_t len, loff_t *ppos)
{
    return 0;
}
static ssize_t  CAMERA_HW_DumpReg_To_Proc2(struct file *file, char __user *data, size_t len, loff_t *ppos)
{
    return 0;
}

static ssize_t  CAMERA_HW_DumpReg_To_Proc3(struct file *file, char __user *data, size_t len, loff_t *ppos)
{
    return 0;
}

/*******************************************************************************
  * CAMERA_HW_Reg_Debug()
  * Used for sensor register read/write by proc file
  ********************************************************************************/
static ssize_t  CAMERA_HW_Reg_Debug(struct file *file, const char *buffer, size_t count,
                         loff_t *data)
{
    char regBuf[64] = {'\0'};
    u32 u4CopyBufSize = (count < (sizeof(regBuf) - 1)) ? (count) : (sizeof(regBuf) - 1);

    MSDK_SENSOR_REG_INFO_STRUCT sensorReg;
    memset(&sensorReg, 0, sizeof(MSDK_SENSOR_REG_INFO_STRUCT));

    if (copy_from_user(regBuf, buffer, u4CopyBufSize))
    return -EFAULT;

    if (sscanf(regBuf, "%x %x",  &sensorReg.RegAddr, &sensorReg.RegData) == 2) {
    if (g_pSensorFunc != NULL) {
        g_pSensorFunc->SensorFeatureControl(DUAL_CAMERA_MAIN_SENSOR, SENSOR_FEATURE_SET_REGISTER, (MUINT8 *)&sensorReg, (MUINT32 *)sizeof(MSDK_SENSOR_REG_INFO_STRUCT));
        g_pSensorFunc->SensorFeatureControl(DUAL_CAMERA_MAIN_SENSOR, SENSOR_FEATURE_GET_REGISTER, (MUINT8 *)&sensorReg, (MUINT32 *)sizeof(MSDK_SENSOR_REG_INFO_STRUCT));
        PK_ERR("M6REGDBG write addr = 0x%08x, data = 0x%08x\n", sensorReg.RegAddr, sensorReg.RegData);
    }
    }
    else if (sscanf(regBuf, "%x", &sensorReg.RegAddr) == 1) {
    if (g_pSensorFunc != NULL) {
        g_pSensorFunc->SensorFeatureControl(DUAL_CAMERA_MAIN_SENSOR, SENSOR_FEATURE_GET_REGISTER, (MUINT8 *)&sensorReg, (MUINT32 *)sizeof(MSDK_SENSOR_REG_INFO_STRUCT));
        PK_ERR("M6REGDBG read addr = 0x%08x, data = 0x%08x\n", sensorReg.RegAddr, sensorReg.RegData);
    }
    }

    return count;
}


static ssize_t  CAMERA_HW_Reg_Debug2(struct file *file, const char *buffer, size_t count,
                                     loff_t *data)
{
    char regBuf[64] = {'\0'};
    u32 u4CopyBufSize = (count < (sizeof(regBuf) - 1)) ? (count) : (sizeof(regBuf) - 1);

    MSDK_SENSOR_REG_INFO_STRUCT sensorReg;
    memset(&sensorReg, 0, sizeof(MSDK_SENSOR_REG_INFO_STRUCT));

    if (copy_from_user(regBuf, buffer, u4CopyBufSize))
    return -EFAULT;

    if (sscanf(regBuf, "%x %x",  &sensorReg.RegAddr, &sensorReg.RegData) == 2) {
    if (g_pSensorFunc != NULL) {
        g_pSensorFunc->SensorFeatureControl(DUAL_CAMERA_MAIN_2_SENSOR, SENSOR_FEATURE_SET_REGISTER, (MUINT8 *)&sensorReg, (MUINT32 *)sizeof(MSDK_SENSOR_REG_INFO_STRUCT));
        g_pSensorFunc->SensorFeatureControl(DUAL_CAMERA_MAIN_2_SENSOR, SENSOR_FEATURE_GET_REGISTER, (MUINT8 *)&sensorReg, (MUINT32 *)sizeof(MSDK_SENSOR_REG_INFO_STRUCT));
        PK_ERR("M6REGDBG write addr = 0x%08x, data = 0x%08x\n", sensorReg.RegAddr, sensorReg.RegData);
    }
    }
    else if (sscanf(regBuf, "%x", &sensorReg.RegAddr) == 1) {
    if (g_pSensorFunc != NULL) {
        g_pSensorFunc->SensorFeatureControl(DUAL_CAMERA_MAIN_2_SENSOR, SENSOR_FEATURE_GET_REGISTER, (MUINT8 *)&sensorReg, (MUINT32 *)sizeof(MSDK_SENSOR_REG_INFO_STRUCT));
        PK_ERR("M6REGDBG read addr = 0x%08x, data = 0x%08x\n", sensorReg.RegAddr, sensorReg.RegData);
    }
    }

    return count;
}

static ssize_t  CAMERA_HW_Reg_Debug3(struct file *file, const char *buffer, size_t count,
                                     loff_t *data)
{
    char regBuf[64] = {'\0'};
    u32 u4CopyBufSize = (count < (sizeof(regBuf) - 1)) ? (count) : (sizeof(regBuf) - 1);

    MSDK_SENSOR_REG_INFO_STRUCT sensorReg;
    memset(&sensorReg, 0, sizeof(MSDK_SENSOR_REG_INFO_STRUCT));

    if (copy_from_user(regBuf, buffer, u4CopyBufSize))
    return -EFAULT;

    if (sscanf(regBuf, "%x %x",  &sensorReg.RegAddr, &sensorReg.RegData) == 2) {
    if (g_pSensorFunc != NULL) {
        g_pSensorFunc->SensorFeatureControl(DUAL_CAMERA_SUB_SENSOR, SENSOR_FEATURE_SET_REGISTER, (MUINT8 *)&sensorReg, (MUINT32 *)sizeof(MSDK_SENSOR_REG_INFO_STRUCT));
        g_pSensorFunc->SensorFeatureControl(DUAL_CAMERA_SUB_SENSOR, SENSOR_FEATURE_GET_REGISTER, (MUINT8 *)&sensorReg, (MUINT32 *)sizeof(MSDK_SENSOR_REG_INFO_STRUCT));
        PK_ERR("M6REGDBG write addr = 0x%08x, data = 0x%08x\n", sensorReg.RegAddr, sensorReg.RegData);
    }
    }
    else if (sscanf(regBuf, "%x", &sensorReg.RegAddr) == 1) {
    if (g_pSensorFunc != NULL) {
        g_pSensorFunc->SensorFeatureControl(DUAL_CAMERA_SUB_SENSOR, SENSOR_FEATURE_GET_REGISTER, (MUINT8 *)&sensorReg, (MUINT32 *)sizeof(MSDK_SENSOR_REG_INFO_STRUCT));
        PK_ERR("M6REGDBG read addr = 0x%08x, data = 0x%08x\n", sensorReg.RegAddr, sensorReg.RegData);
    }
    }

    return count;
}

/*=======================================================================
  * platform driver
  *=======================================================================*/

/* It seems we don't need to use device tree to register device cause we just use i2C part */
/* You can refer to CAMERA_HW_probe & CAMERA_HW_i2c_probe */
#ifdef CONFIG_OF
static const struct of_device_id CAMERA_HW_of_ids[] = {
	{ .compatible = "mediatek,camera_hw", },
    {}
};
#endif

static struct platform_driver g_stCAMERA_HW_Driver = {
    .probe      = CAMERA_HW_probe,
    .remove     = CAMERA_HW_remove,
    .suspend    = CAMERA_HW_suspend,
    .resume     = CAMERA_HW_resume,
    .driver     = {
    .name   = "image_sensor",
    .owner  = THIS_MODULE,
#ifdef CONFIG_OF
    .of_match_table = CAMERA_HW_of_ids,
#endif
    }
};

#ifndef CONFIG_OF
static struct platform_device camerahw2_platform_device = {
    .name = "image_sensor_bus2",
    .id = 0,
    .dev = {
    }
};
#endif


static  struct file_operations fcamera_proc_fops = {
    .read = CAMERA_HW_DumpReg_To_Proc,
    .write = CAMERA_HW_Reg_Debug
};
static  struct file_operations fcamera_proc_fops2 = {
    .read = CAMERA_HW_DumpReg_To_Proc2,
    .write = CAMERA_HW_Reg_Debug2
};
static  struct file_operations fcamera_proc_fops3 = {
    .read = CAMERA_HW_DumpReg_To_Proc3,
    .write = CAMERA_HW_Reg_Debug3
};

/* Camera information */
static int subsys_camera_info_read(struct seq_file *m, void *v)
{
   PK_ERR("subsys_camera_info_read %s\n",mtk_ccm_name);
   seq_printf(m, "%s\n",mtk_ccm_name);
   return 0;
};

static int proc_camera_info_open(struct inode *inode, struct file *file)
{
    return single_open(file, subsys_camera_info_read, NULL);
};

static  struct file_operations fcamera_proc_fops1 = {
    .owner = THIS_MODULE,
    .open  = proc_camera_info_open,
    .read  = seq_read,
};

/*add camera otp checksum start*/
void mtk_eeprom_hw_otp_check_set(char *name, int otp_flag)
{
        PK_ERR("%s %d\n", __func__,__LINE__);
        if(!strcmp("back",name)){
                memset(back_otp_checksum_info,0,20*sizeof(char));
                if (otp_flag == 0) {
                        snprintf(back_otp_checksum_info,12,"%s","otp:back_ok\n");
                } else {
                        snprintf(back_otp_checksum_info,13,"%s","otp:back_err\n");
                }
        }else{
                memset(front_otp_checksum_info,0,20*sizeof(char));
                if (otp_flag == 0) {
                        snprintf(front_otp_checksum_info,13,"%s","otp:front_ok\n");
                } else {
                        snprintf(front_otp_checksum_info,14,"%s","otp:front_err\n");
                }
        }
}
EXPORT_SYMBOL(mtk_eeprom_hw_otp_check_set);

static ssize_t mtk_eeprom_hw_otp_checksum_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
        ssize_t rc = 0;
        sprintf(buf,"%s\n%s\n",back_otp_checksum_info,front_otp_checksum_info);
        rc = strlen(buf) + 1;
        return rc;
}
static DEVICE_ATTR(otp, 0444, mtk_eeprom_hw_otp_checksum_show, NULL);
static int32_t mtk_eeprom_init_device_name(void)
{
        int32_t rc = 0;
        PK_DBG("%s %d\n", __func__,__LINE__);
        if(mtk_eeprom_device != NULL){
                PK_DBG("android_camera_otp already created\n");
                return 0;
        }
        mtk_eeprom_device = kobject_create_and_add("android_camera_otp", NULL);
        if (mtk_eeprom_device == NULL) {
                PK_DBG("%s: subsystem_register failed\n", __func__);
                rc = -ENOMEM;
                return rc ;
        }
        rc = sysfs_create_file(mtk_eeprom_device, &dev_attr_otp.attr);
        if (rc) {
                pr_err("%s: sysfs_create_file failed\n", __func__);
                kobject_del(mtk_eeprom_device);
        }

        return 0 ;
}
/*add camera otp checksum end*/


/*=======================================================================
  * CAMERA_HW_i2C_init()
  *=======================================================================*/
static int __init CAMERA_HW_i2C_init(void)
{

#if 0
    struct proc_dir_entry *prEntry;
#endif
#if defined(CONFIG_MTK_LEGACY)
    /* i2c_register_board_info(CAMERA_I2C_BUSNUM, &kd_camera_dev, 1); */
    if(SUPPORT_I2C_BUS_NUM1 != SUPPORT_I2C_BUS_NUM2)  //Main2 Support
    {
        i2c_register_board_info(SUPPORT_I2C_BUS_NUM1, &i2c_devs1, 1);
        i2c_register_board_info(SUPPORT_I2C_BUS_NUM2, &i2c_devs2, 1);
    }
    else
    {
        i2c_register_board_info(SUPPORT_I2C_BUS_NUM1, &i2c_devs1, 1);
        i2c_register_board_info(SUPPORT_I2C_BUS_NUM3, &i2c_devs2, 1);
    }
#endif
    PK_DBG("[camerahw_probe] start\n");

#ifndef CONFIG_OF
	int ret = 0;
    ret = platform_device_register(&camerahw_platform_device);
    if (ret) {
    PK_ERR("[camerahw_probe] platform_device_register fail\n");
    return ret;
    }

    ret = platform_device_register(&camerahw2_platform_device);
    if (ret) {
    PK_ERR("[camerahw2_probe] platform_device_register fail\n");
    return ret;
    }
#endif

    if (platform_driver_register(&g_stCAMERA_HW_Driver)) {
    PK_ERR("failed to register CAMERA_HW driver\n");
    return -ENODEV;
    }
    if (platform_driver_register(&g_stCAMERA_HW_Driver2)) {
    PK_ERR("failed to register CAMERA_HW driver\n");
    return -ENODEV;
    }
/* FIX-ME: linux-3.10 procfs API changed */
#ifdef CONFIG_HUAWEI_DSM
    //camera_dsm_get_client();
#endif
#if 1
    proc_create("driver/camsensor", 0666, NULL, &fcamera_proc_fops);
    proc_create("driver/camsensor2", 0666, NULL, &fcamera_proc_fops2);
    proc_create("driver/camsensor3", 0666, NULL, &fcamera_proc_fops3);

    /* Camera information */
    memset(mtk_ccm_name,0,camera_info_size);
    proc_create(PROC_CAMERA_INFO, 0, NULL, &fcamera_proc_fops1);

#else
    /* Register proc file for main sensor register debug */
    prEntry = create_proc_entry("driver/camsensor", 0, NULL);
    if (prEntry) {
    prEntry->read_proc = CAMERA_HW_DumpReg_To_Proc;
    prEntry->write_proc = CAMERA_HW_Reg_Debug;
    }
    else {
    PK_ERR("add /proc/driver/camsensor entry fail\n");
    }

    /* Register proc file for main_2 sensor register debug */
    prEntry = create_proc_entry("driver/camsensor2", 0, NULL);
    if (prEntry) {
    prEntry->read_proc = CAMERA_HW_DumpReg_To_Proc;
    prEntry->write_proc = CAMERA_HW_Reg_Debug2;
    }
    else {
    PK_ERR("add /proc/driver/camsensor2 entry fail\n");
    }

    /* Register proc file for sub sensor register debug */
    prEntry = create_proc_entry("driver/camsensor3", 0, NULL);
    if (prEntry) {
        prEntry->read_proc = CAMERA_HW_DumpReg_To_Proc;
        prEntry->write_proc = CAMERA_HW_Reg_Debug3;
    }
    else {
        PK_ERR("add /proc/driver/camsensor entry fail\n");
    }

    /* Register proc file for main sensor register debug */
    prEntry = create_proc_entry("driver/maincam_status", 0, NULL);
    if (prEntry) {
    prEntry->read_proc = CAMERA_HW_Read_Main_Camera_Status;
    prEntry->write_proc = NULL;
    }
    else {
    PK_ERR("add /proc/driver/maincam_status entry fail\n");
    }

    /* Register proc file for sub sensor register debug */
    prEntry = create_proc_entry("driver/subcam_status", 0, NULL);
    if (prEntry) {
    prEntry->read_proc = CAMERA_HW_Read_Sub_Camera_Status;
    prEntry->write_proc = NULL;
    }
    else {
    PK_ERR("add /proc/driver/subcam_status entry fail\n");
    }

    /* Register proc file for 3d sensor register debug */
    prEntry = create_proc_entry("driver/3dcam_status", 0, NULL);
    if (prEntry) {
    prEntry->read_proc = CAMERA_HW_Read_3D_Camera_Status;
    prEntry->write_proc = NULL;
    }
    else {
    PK_ERR("add /proc/driver/3dcam_status entry fail\n");
    }

#endif
    atomic_set(&g_CamHWOpend, 0);
    atomic_set(&g_CamHWOpend2, 0);
    atomic_set(&g_CamDrvOpenCnt, 0);
    atomic_set(&g_CamDrvOpenCnt2, 0);
    atomic_set(&g_CamHWOpening, 0);

/*add otp checksum*/
    mtk_eeprom_init_device_name();

    return 0;
}

/*=======================================================================
  * CAMERA_HW_i2C_exit()
  *=======================================================================*/
static void __exit CAMERA_HW_i2C_exit(void)
{
    platform_driver_unregister(&g_stCAMERA_HW_Driver);
    platform_driver_unregister(&g_stCAMERA_HW_Driver2);
}


EXPORT_SYMBOL(kdSetSensorSyncFlag);
EXPORT_SYMBOL(kdSensorSyncFunctionPtr);
EXPORT_SYMBOL(kdGetRawGainInfoPtr);

module_init(CAMERA_HW_i2C_init);
module_exit(CAMERA_HW_i2C_exit);

MODULE_DESCRIPTION("CAMERA_HW driver");
MODULE_AUTHOR("Jackie Su <jackie.su@Mediatek.com>");
MODULE_LICENSE("GPL");
