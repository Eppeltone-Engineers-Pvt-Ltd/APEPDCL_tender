//-------- include device specific files----------------
#include "..\\..\\rlDevice\\Include\\dDeltaSigmaADC.h"
#include "..\\..\\rlDevice\\Include\\dI2c.h"
#include "math.h"
//-----------------------------------------------------

//-------- include app specific files-------------------
#include "..\\Include\\AppCalibration.h"
#include "..\\Include\\AppBlockDailyLS.h"
#include "..\\Include\\AppVariables.h"
#include "..\\Include\\AppEeprom.h"
#include "..\\Include\\AppMisc.h"
#include "..\\Include\\AppConfig.h"

#define ADCHANNEL0	0x00
#define ADCHANNEL1	0x01
#define ADCHANNEL2	0x02
#define ADCHANNEL3	0x03


struct rlCalib rlCalib;

static uint16_t getPhaseDelay(uint32_t chPower,uint32_t watt_p,uint16_t olddelay);

void doCalibration(uint8_t cal_option,uint32_t watt_p)
{

	Ins.PhPower=Ins.PhPower/10;
	Ins.NeuPower=Ins.NeuPower/10;
	switch(cal_option)
	{
		//------------- upf phase calibration-----------
		case 0:
			rlCalib.U=REF_U*rlCalib.U/Ins.Voltage;
			rlCalib.I1=REF_I*rlCalib.I1/Ins.PhCurrent;
			
			rlCalib.Pp=(uint64_t)Ins.PhPower*rlCalib.Pp/watt_p;
			rlCalib.I2OffSet=Ins.NeuCurrent;
			
		break;
		//**********************************************
		
		//------------- upf ct calibration--------------
		case 1:
			rlCalib.I2=REF_I*rlCalib.I2/Ins.NeuCurrent;
			rlCalib.Pn=(uint64_t)Ins.NeuPower*rlCalib.Pn/watt_p;
			rlCalib.I1OffSet=Ins.PhCurrent;
			
		break;
		//**********************************************
		
		//------------- lg phase calibration------------
		case 2:
			
			rlCalib.DelayI1=getPhaseDelay(Ins.PhPower,watt_p,rlCalib.DelayI1);
			DeltaSigmaADC_AdjustPhaseDelay(ADCHANNEL1,rlCalib.DelayI1);

		break;
		//**********************************************
		
		//------------- lg ct calibration-----------
		case 3:
			rlCalib.DelayI2=getPhaseDelay(Ins.NeuPower,watt_p,rlCalib.DelayI2);
			DeltaSigmaADC_AdjustPhaseDelay(ADCHANNEL2,rlCalib.DelayI2);
		break;
		//**********************************************
	}
	
	rlCalib.crc=Crc_Cal((uint8_t*)&rlCalib,CALIB_LENGTH-2);
	write_eeprom((uint8_t*)&rlCalib,CAL_CONST,CALIB_LENGTH);
	write_eeprom((uint8_t*)&rlCalib,CAL_CONST_2,CALIB_LENGTH);
}


static uint16_t getPhaseDelay(uint32_t chPower,uint32_t watt_p,uint16_t olddelay)
{
	float pfdiff;
	
	pfdiff=chPower/(watt_p*2.0f);
	
	pfdiff=acos(pfdiff)*57.296;
	pfdiff=60-pfdiff;
	pfdiff=pfdiff/0.024f;
	olddelay=olddelay+pfdiff;// new delay
	
	return olddelay;
}
void getRLCalib(void)
{
	read_eeprom((uint8_t*)&rlCalib,CAL_CONST,CALIB_LENGTH);	
	if(Crc_Cal((uint8_t*)&rlCalib,CALIB_LENGTH))
	{
		read_eeprom((uint8_t*)&rlCalib,CAL_CONST_2,CALIB_LENGTH);	
		if(Crc_Cal((uint8_t*)&rlCalib,CALIB_LENGTH))
		{		
			rlCalib.U	=24659;
			rlCalib.I1	=30176;
			rlCalib.I2	=30488;
			rlCalib.Pp	=13211;
			rlCalib.Pn	=13511;
			rlCalib.DelayI1=694;
			rlCalib.DelayI2=661;
			rlCalib.I1OffSet=18;
			rlCalib.I2OffSet=1;
		}
	}
	

}

