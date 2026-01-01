//-------- include device specific files----------------
#include "..\\..\\rlDevice\\Include\\dI2c.h"
//-----------------------------------------------------

//-------- include app specific files-------------------
#include "..\\Include\\AppLcd.h"
#include "..\\Include\\AppVariables.h"
#include "..\\Include\\AppEeprom.h"
#include "..\\Include\\AppMacros.h"
#include "..\\Include\\AppTampers.h"
#include "..\\Include\\AppConfig.h"
#include "..\\Include\\AppMisc.h"


uint8_t doCalanderAction(void)
{
	
	if(InsSave.timeCounter>=ScheduleActionDateTime.CalanderDateTime )
	{
		setParamIndex(MSG_ACT00_INDEX,1);
		Display(USER_CODE);
		#if (defined(IRDA_TYPE_METER_HP) && (IRDA_TYPE_METER_HP == 1))
			read_eeprom((uint8_t *)&RxTxBuffer,PASSIVE_ZONE_TIME,25);
			write_eeprom((uint8_t *)&RxTxBuffer,ZONE_TIME,25);
		#elif (defined(IRDA_TYPE_METER_HP) && (IRDA_TYPE_METER_HP == 0))
			read_eeprom((uint8_t *)&RxTxBuffer,PASSIVE_ZONE_TIME,57);
			write_eeprom((uint8_t *)&RxTxBuffer,ZONE_TIME,57);
			RxTxBuffer[0] =getSingleActionFlag(1);
		#else
			#error "macro not defined."
		#endif
		if(RxTxBuffer[0]==0xAC)
		{
			RxTxBuffer[0]=0xAB;
		}
		else 
		{
			RxTxBuffer[0]=0xFF;
			mcu_flag &=~ PASSIVE_CAL_CHECK_FLAG;
			
		}
		write_eeprom(RxTxBuffer,PASSIVE_SEA_ACT_TIME+4,1);
		getSingleActionFlag(1);
		
	//	logSWTamper(155);
		return RECORD_DO_BILL_DATA;
		
	}
	else
		return 0;
		
}

uint8_t doBillSingleAction(void)
{
	if(InsSave.timeCounter>=ScheduleActionDateTime.BillDateTime )
	{
		setParamIndex(MSG_ACT01_INDEX,1);
		Display(USER_CODE);
		
		RxTxBuffer[0] =getSingleActionFlag(0);
		if(RxTxBuffer[0]==0xAC)
			RxTxBuffer[0]=0xAB;
		else 
		{
			RxTxBuffer[0]=0xFF;
			mcu_flag &=~ SINGLE_ACTION_CHECK_FLAG;			
		}
			
		write_eeprom((uint8_t *)&RxTxBuffer,PRE_BILL_SINGLE_ACTION+4,1);
		getSingleActionFlag(0);
		return RECORD_DO_BILL_DATA;
	}
	else
		return 0;
}
