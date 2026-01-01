#ifndef _APP_CALIB_H
#define _APP_CALIB_H

#include "..\\..\\rlDevice\\Include\\dTypedef.h"
struct rlCalib
{
	uint16_t Pp;
	uint16_t Pn;
	uint16_t U;
	uint16_t I1;
	uint16_t I2;
	uint16_t DelayI1;
	uint16_t DelayI2;
	uint16_t I1OffSet;
	uint16_t I2OffSet;
	uint16_t crc;
	

};
extern struct rlCalib rlCalib;

void doCalibration(uint8_t cal_option,uint32_t watt_p);
void getRLCalib(void);
#endif
