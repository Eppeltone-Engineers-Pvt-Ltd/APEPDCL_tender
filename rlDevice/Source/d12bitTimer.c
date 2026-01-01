#include "..\\include\\dIODefine.h"
#include "..\\include\\dUser_includes.h"
#include "..\\include\\dIOCtrl.h"
#include "..\\include\\dTypedef.h"

//-------- include app specific files----------------
#include "..\\..\\rlApplication\\Include\\AppMacros.h"
#include "..\\..\\rlApplication\\Include\\AppVariables.h"
#include "..\\..\\rlApplication\\Include\\AppConfig.h"
//------------------------------------------------------

//-------- include dlms specific files----------------
//-------- include dlms specific files----------------
#include "..\\..\\rlDlms\\DLMS_User.h"
#include "..\\..\\rlDlms\\meter_app\\r_dlms_data_meter.h"				/* DLMS Data Definitions */


static uint8_t pushholdtime=0;
#if (defined(IS_DLMS_ENABLED_EEPL) && IS_DLMS_ENABLED_EEPL == 1)
static uint16_t dlms_timer_callback_count=0;
#endif
#if (defined(IRDA_COM_ENABLE) && (IRDA_COM_ENABLE == 1))
uint16_t time_out_for_irda_rx=0;	
#endif
static uint32_t activePulsePowerSum;
static uint8_t pulse_led_on_counter;



#define ONE_PULSE_VALUE		115200000UL	//0x1B77400UL
#define PULSE_LED_ON_TIME	50		//(~50ms)
#define PUSH_WAIT_TIME		50		//(~50ms)
#define TICK_UPADTE_TIME	500		//500ms


void init_Timer12bit(unsigned char mSec)
{
	TMKAEN  = 1;
	ITMC    = 0x0000;
	TMKAMK  = 1;
	TMKAIF  = 0;
	TMKAPR0 = 0;
	TMKAPR1 = 0;
	ITMC    = 0x001F;
	TMKAIF  = 0;
	TMKAMK  = 0;
	ITMC |= 0x8000;

}

#pragma interrupt Interrupt_12BIT_TIMER(vect=INTIT,bank=RB3)
static void Interrupt_12BIT_TIMER(void)
{

	activePulsePowerSum+=activePulsePower;
	
	if(activePulsePowerSum>=ONE_PULSE_VALUE)
	{
		activePulsePowerSum-=ONE_PULSE_VALUE;
		SWITCH_ON_PULSE_LED;
		pulse_led_on_counter=0;
	}
	
	if(GET_PULSE_STATUS)
	{
		if(++pulse_led_on_counter>=PULSE_LED_ON_TIME)
			SWITCH_OFF_PULSE_LED;
	}
	
	if(GET_PUSH_BUTTON_STATUS)
	{
		if(pushholdtime++>=PUSH_WAIT_TIME)
		{
			pushholdtime=0;
			if((mcu_flag &PUSH_BUTTON_PRESSED_FLAG)==0)
				mcu_flag|=PUSH_BUTTON_RELEASED;
			mcu_flag|=PUSH_BUTTON_PRESSED_FLAG;
		}
	}
	else
	{
		mcu_flag&=~PUSH_BUTTON_PRESSED_FLAG;
	}
	#if (defined(IS_DLMS_ENABLED_EEPL) && IS_DLMS_ENABLED_EEPL == 1)
	#if (defined(UART_0_ENABLE) && (UART_0_ENABLE == 1))
	if(++dlms_timer_callback_count>=TICK_UPADTE_TIME)
	{
		dlms_timer_callback_count=0;
		R_OBIS_Timer_ConstInterruptCallback();
	}
	
	PhysicalTimeoutCount();	
	#endif
	#endif
	#if (defined(IRDA_COM_ENABLE) && (IRDA_COM_ENABLE == 1))
	#if (defined(UART_2_ENABLE) && (UART_2_ENABLE == 1))
	if(++time_out_for_irda_rx>=50)
	{
		if(mcu_flag&COMM_RECEIVE_START)
			mcu_flag &= ~COMM_RECEIVE_START;
		time_out_for_irda_rx=0;
	}	
	#endif
	#endif
}		

void ClearPulseAccumulation(void)
{
	activePulsePowerSum=0;
	pulse_led_on_counter=0;
	activePulsePowerSumNM=0;
	apperentPulsePowerSumNM=0;
	SWITCH_OFF_PULSE_LED;
}
void setPulse(uint8_t setflag)
{
	if(setflag)
	{
		power_fail_state|=PULSE_ON_FLAG;
		
	}
	if(power_fail_state&PULSE_ON_FLAG)
	{
		pulse_led_on_counter=0;
		SWITCH_ON_PULSE_LED;
	}
}
