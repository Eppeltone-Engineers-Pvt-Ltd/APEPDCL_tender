//-------- include device specific files----------------
#include "..\\..\\rlDevice\\Include\\dClock.h"
#include "..\\..\\rlDevice\\Include\\dUser_includes.h"
#include "..\\..\\rlDevice\\Include\\dRtc.h"
#include "..\\..\\rlDevice\\Include\\dDeltaSigmaADC.h"
#include "..\\..\\rlDevice\\Include\\dUart.h"
#include "..\\..\\rlDevice\\Include\\dWatchDog.h"
#include "..\\..\\rlDevice\\Include\\dVBat.h"
#include "..\\..\\rlDevice\\Include\\dMisc.h"
#include "..\\..\\rlDevice\\Include\\dIOCtrl.h"
#include "..\\..\\rlDevice\\Include\\d8bitTimer.h"
#include "..\\..\\rlDevice\\Include\\dDisplay.h"
#include "..\\..\\rlDevice\\Include\\dI2c.h"
#include "..\\..\\rlDevice\\Include\\d12BitTimer.h"
//-----------------------------------------------------

//-------- include app specific files-------------------
#include "..\\Include\\AppLcd.h"
#include "..\\Include\\AppPowerOnInit.h"
#include "..\\Include\\AppCommunication.h"
#include "..\\Include\\AppMisc.h"
#include "..\\Include\\AppVariables.h"
#include "..\\Include\\AppBatteryManagement.h"
#include "..\\Include\\AppTampers.h"
#include "..\\Include\\AppConfig.h"
#include "..\\Include\\AppEeprom.h"
#include "..\\Include\\AppCustomerProtocol.h"
//------------------------------------------------------

//-------- include dlms specific files----------------
#include "..\\..\\rlDlms\\DLMS_User.h"


void production_prepareLowPower(uint8_t initRestoreValue);


void production_process(void)
{
	uint32_t noClear=0;
	mcu_flag|=(START_MEASUREMENT+START_SAMPLE_BUFFER);
	mcu_flag |= COMM_RECEIVE_ENABLE;       //Enable reception
	EI();
	
	
	setParaBuff(DISPLAY_MODE_PRODUCTION);
	setParamIndex(MSG_FW_INDEX,1);
	refresh_time=2;
	validateRCR2();
	validateRCR1();

	ls_nm_start_loc=0xFFFF;
	
	
	//---------------------------------------
	DLMS_Initialize();
	//---------------------------------------
	//start_Timer8bit0(1);
	SWITCH_ON_COMM_VCC;
	
	while(1)
	{
		//Switch2FilClock();
		setPulse(0);
		WDT_Restart(0xAC);
		if(power_fail_state&RTC_BAT_STATUS_FLAG)
			reEnable32KCrystal();
		
		if(mcu_flag&ONE_SECOND_OVER_FLAG)
		{
			
			
			getRtcCounter(RTC_NO_CHECK_MODE);
			//-------------------------------------
			if(refresh_time--==0)
			{
				
				setNextParaSeq();
				mcu_flag&=~DISPLAY_CHANGE_FLAG;
							
				read_eeprom((uint8_t*)&noClear,IS_MEMORY_CLEARED,4);
				if(power_fail_state&RTC_BAT_STATUS_FLAG)
					setParamIndex(MSG_RTCBAT_STATUS,1);
				else if(display_flag&DISP_REV)
					setParamIndex(MSG_REV_PROD,1);
				else if(0x01020304!=noClear)
					setParamIndex(MSG_NO_CLEAR,1);
				else
					setParamIndex(*displayParamPtr,0);

				
				switch(*displayParamPtr)
				{
					case 1:
					case 4:
					case 7:
					case 9:
						refresh_time=1;
					break;
					default:
						refresh_time=0;
					break;
				}
				
					
			}	
			if(ModeID==DISPLAY_MODE_PROD_PUSH)
			{
				if(pushModeTime>00)
				{
					pushModeTime--;
					refresh_time=0x01;
				}
			}
			
			if(mcu_flag&PUSH_BUTTON_RELEASED)
			{
				pushModeTime=10;
				if(!(ModeID==DISPLAY_MODE_PROD_PUSH))
				{
					setParaBuff(DISPLAY_MODE_PROD_PUSH);
					setParamIndex(MSG_PUSH_INDEX,1);
				}
				refresh_time=0x00;
				
				mcu_flag&=~PUSH_BUTTON_RELEASED;
			}
			if(pushModeTime==0)
			{
				if(DISPLAY_MODE_PROD_PUSH==ModeID)
				{			
					setParaBuff(DISPLAY_MODE_PRODUCTION);
					setParamIndex(MSG_AUTO_INDEX,1);	
				}		
			}
			if(DISPLAY_MODE_PROD_PUSH!=ModeID)
			{
				if(GET_COVER_STATUS)
				{
					if(ModeID!=DISPLAY_MODE_AUTO_COVER)
						setParaBuff(DISPLAY_MODE_AUTO_COVER);
				}
			}

			
			
			Display(PRODUCTION_CODE);
			//-------------------------------------
			mcu_flag&=~ONE_SECOND_OVER_FLAG;
			if((mcu_flag&POWER_STABLE_FLAG)==0)
			{
				if(powerStableCounter<5)
					powerStableCounter++;
				else
				{
					mcu_flag|=POWER_STABLE_FLAG;
					powerStableCounter=0;
				}
			}
			
		}
		
		
	//-------------------measurement----------
		if(mcu_flag&DO_MEASUREMENT)
		{
			mcu_flag&=~DO_MEASUREMENT;
			
			//do calculation here
			doMeasurement(PRODUCTION_CODE);
			if(GET_MAG_STATUS)
				display_flag|=DISP_MAG;
			
		}
		if(mcu_flag&DO_FREQ_MEASUREMENT)
		{
			mcu_flag&=~DO_FREQ_MEASUREMENT;
			Calculate_Line_Frq();
		}
	//----------------------------------------
	
	//----------------------------------------
		if(mcu_flag&COMM_RECEIVE_OVER)
		{
			mcu_flag&=~COMM_RECEIVE_OVER;
			
			#if (defined(IRDA_TYPE_METER_HP) && (IRDA_TYPE_METER_HP == 1))
				if(RxTxBuffer[0]==START_OF_CHAR_OPTICAL)
					Communication(PRODUCTION_CODE);
				else if(RxTxBuffer[0]==START_OF_CHAR_IRDA)
					CustomerProtocol();
			#endif
			#if (defined(IRDA_TYPE_METER_HP) && (IRDA_TYPE_METER_HP == 0))
				if(RxTxBuffer[0]==START_OF_CHAR_OPTICAL)
					Communication(PRODUCTION_CODE);
			#endif			
		}
	//----------------------------------------
	
	//----------------------------------------
	#if (defined(IS_DLMS_ENABLED_EEPL) && (IS_DLMS_ENABLED_EEPL == 1))
		/* DLMS process of request and response message */
	#if (defined(UART_0_ENABLE) && (UART_0_ENABLE == 1))		
		DLMSMessageProcess();
	#endif
	#endif
	//----------------------------------------
		
	//----------------------------------------
		if((mcu_flag&POWER_DOWN_FLAG)||(GET_MAINS_STATUS==0))
		{	
			powerStableCounter=0;
			mcu_flag&=~POWER_DOWN_FLAG;
			production_prepareLowPower(RESTORE_VALUE_PFAIL_NORMAL);
		}
	//----------------------------------------
		
	}
	
}

void production_prepareLowPower(uint8_t initRestoreValue)
{

	uint8_t no_of_wake_up=3,power_restore=initRestoreValue;
	start_Timer8bit0(2);
	shutDownPheripheral();
	
	setParamIndex(MSG_FW_INDEX,1);
	while(no_of_wake_up--)
	{
		//Switch2FilClock();
		switch2Low_Power_MODE(LOW_POWER_STOP);
		if((GET_PUSH_BUTTON_STATUS==0)&&(GET_COVER_STATUS==0))
		{
			if(GET_MAINS_STATUS==0)
			{
				R_LCD_PowerOff();
				stop_Timer8bit0();
				mcu_flag|=POWER_DOWN_FLAG;
				switch2Low_Power_MODE(LOW_POWER_STOP);
			}
			
		}
		if(GET_COVER_STATUS)
		{
			setParamIndex(MSG_C_OPEN_INDEX,1);
		}
		
		if((GET_COVER_STATUS)||(GET_PUSH_BUTTON_STATUS))
		{
			Display(PRODUCTION_CODE);
			R_LCD_PowerOn();
		}
		if(mcu_flag&POWER_DOWN_FLAG)
		{
			no_of_wake_up=1;			
			power_restore=5;
		}
		else
			no_of_wake_up=3;
		if(++power_restore>5)
			if(GET_MAINS_STATUS)
				WDT_Restart(0x00);
	}

	WDT_Restart(0x00);	
}
