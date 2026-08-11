/*
 * l681 Sunny/AL1518 OV13853 module EEPROM (i2c-2, 8-bit 0xA0): the two byte
 * readers the sensor driver uses for the module info and the OTP, from the
 * cam_cal ov13853sy_eeprom.c driver; its CAM_CAL character device and its
 * initcall are not taken.
 */
#include <linux/kernel.h>
#include <linux/types.h>

#define OV13853SY_DEVICE_ID	0xA0
#define EEPROM_I2C_SPEED	100

extern int iReadRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u8 *a_pRecvData,
		       u16 a_sizeRecvData, u16 i2cId);
extern void kdSetI2CSpeed(u16 i2cSpeed);

bool sy_byteread_cmos_sensor(unsigned char SLAVEID, unsigned short addr, unsigned char *data)
{
	char puSendCmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

	kdSetI2CSpeed(EEPROM_I2C_SPEED);
	if (iReadRegI2C(puSendCmd, 2, data, 1, SLAVEID) < 0) {
		pr_err("ov13853sy_eeprom [CAM_CAL_SY] fail ov13853sy_byteread_cmos_sensor addr =0x%x, data = 0x%x\n",
		       addr, *data);
		return false;
	}
	pr_debug("ov13853sy_eeprom [CAM_CAL_SY] ov13853sy_cmos_sensor addr =0x%x, data = 0x%x\n",
		 addr, *data);
	return true;
}

int sy_selective_read_region(u32 addr, u8 *data, u16 i2c_id, u32 size)
{
	unsigned short curAddr = (unsigned short)addr;
	u8 *buff = data;
	u32 size_to_read = size;
	int ret = 0;

	while (size_to_read > 0) {
		if (sy_byteread_cmos_sensor(OV13853SY_DEVICE_ID, curAddr, buff)) {
			curAddr += 1;
			buff += 1;
			size_to_read -= 1;
			ret += 1;
		} else {
			break;
		}
	}
	pr_debug("ov13853sy_eeprom sy_selective_read_region addr =%x size %d readSize = %d\n",
		 addr, size, ret);
	return ret;
}
