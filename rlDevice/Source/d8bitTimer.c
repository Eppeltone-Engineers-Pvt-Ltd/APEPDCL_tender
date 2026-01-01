#include "..\\include\\dIODefine.h"
#include "..\\include\\dUser_includes.h"
#include "..\\include\\d8bitTimer.h"
#include "..\\include\\dIoctrl.h"

//-------- include app specific files----------------
#include "..\\..\\rlApplication\\Include\\AppMacros.h"
#include "..\\..\\rlApplication\\Include\\AppVariables.h"
#include "..\\..\\rlApplication\\Include\\AppConfig.h"
//------------------------------------------------------
#define WAIT_CYCLES		(12000000UL/32768)
void updatePulse(void);
//uint8_t i=0;

void init_Timer8bit0(uint8_t mSec)
{
	PR02H  	|= 0x0C;
	PR12H  	|= 0x0C;
	start_Timer8bit0(mSec);
}

void init_Timer8bit1(uint8_t mSec)
{
	PR03L  	|= 0x0C;
	PR13L  	|= 0x0C;
	start_Timer8bit1(mSec);
}

void start_Timer8bit0(uint16_t mSec)
{
	
	TRTCR0	= 0x10;

	TRTCR0	|= 0x80;

	TRTMD0 	= 0x07;
    TRTCMP0 = 256U*mSec;
	IF2H_bit.no2=0;
	MK2H_bit.no2=0;
	TSTART00=1;
	
}
void start_Timer8bit1(uint8_t mSec)
{
	TRTCR1	= 0x10;
	TRTCR1	|= 0x80;
	TRTMD1 	= 0x07;
	TRTCMP1 = 256U*mSec;
	IF3L_bit.no2=0;
	MK3L_bit.no2=0;
	TSTART10=1;
}
void stop_Timer8bit0(void)
{
	uint16_t i;
	MK2H_bit.no2=1;
	TSTART00 = 0U;//x00;	
	for(i=0;i<WAIT_CYCLES;i++);
	
	TRTCR0&=~0x10;

	IF2H_bit.no2=0;
//	TRTCMP0 = 0;
		
}
void stop_Timer8bit1(void)
{
	uint16_t i;
	MK3L_bit.no2=1;
	TSTART10=0x00;	
	for(i=0;i<WAIT_CYCLES;i++);

	TRTCR1&=~0x10;
	IF3L_bit.no2=0;
	
}


#pragma interrupt Interrupt_16BIT_TIMER0(vect=INTIT00,bank=RB2)
static void Interrupt_16BIT_TIMER0(void)
{
	
	NOP();
	mcu_flag|=ONE_SECOND_OVER_FLAG;
	updatePulse();
	/*if(i==0)
	{
		i=1;
		SWITCH_ON_PULSE_LED;
	}
	else
	{
		i=0;
		SWITCH_OFF_PULSE_LED;
	}*/
}

#pragma interrupt Interrupt_16BIT_TIMER1(vect=INTIT10,bank=RB2)
static void Interrupt_16BIT_TIMER1(void)
{
	
	NOP();
	mcu_flag|=EIGHT_BIT_TIMER1_FLAG;
	
	
	
}

void updatePulse(void)
{
	uint16_t no_pulse;	
	activePulsePowerSumNM+=activePowerNM;
	if(activePulsePowerSumNM>=ONE_SECOND_PULSE_VAL)
	{
		no_pulse=activePulsePowerSumNM/ONE_SECOND_PULSE_VAL;
		activePulsePowerSumNM=activePulsePowerSumNM-ONE_SECOND_PULSE_VAL*no_pulse;
		no_of_pulses_in_nm_Active+=no_pulse;
		
	}
	apperentPulsePowerSumNM+=apperentPowerNM;
	if(apperentPulsePowerSumNM>=ONE_SECOND_PULSE_VAL)
	{
		no_pulse=apperentPulsePowerSumNM/ONE_SECOND_PULSE_VAL;
		apperentPulsePowerSumNM=apperentPulsePowerSumNM-ONE_SECOND_PULSE_VAL*no_pulse;
		no_of_pulses_in_nm_App+=no_pulse;
	}	

}
