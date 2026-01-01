//-------- include device specific files----------------
#include "..\\..\\rlDevice\\Include\\dI2c.h"
#include "..\\..\\rlDevice\\Include\\dIOCtrl.h"
#include "..\\..\\rlDevice\\Include\\dRtc.h"

//-----------------------------------------------------

//-------- include app specific files-------------------
#include "..\\Include\\AppLcd.h"
#include "..\\Include\\AppVariables.h"
#include "..\\Include\\AppEeprom.h"
#include "..\\Include\\AppMacros.h"
#include "..\\Include\\AppScheduleAction.h"
#include "..\\Include\\AppMisc.h"
#include "..\\Include\\AppBilling.h"
#include "..\\Include\\AppMD.h"
#include "..\\Include\\AppBlockDailyLS.h"
#include "..\\Include\\AppTampers.h"
#include "..\\Include\\AppIntervalkWh.h"
#include "..\\Include\\AppConfig.h"
#include "..\\Include\\AppTOD.h"



//----------------dlms specific files---------------------------------
#include "..\\..\\rlDlms\\meter_app\\r_dlms_data_meter.h"				/* DLMS Data Definitions */
void updatePfailEvent(void);


void OneSecondTask(void)
{
	uint8_t task=0;
	uint8_t task_RD=0;
	uint8_t clear_nm_clock=0;
	uint32_t nowkWh=InsSave.CumkWh+InsSave.ZkWhCounter/METER_CONSTANT;

	InsSave.timeCounter++;
		
	isRAMOK();	
	if(mcu_flag&POWER_STABLE_FLAG)
	{
		InsSave.lsAvgV=InsSave.lsAvgV+Ins.Voltage;
		InsSave.lsAvgC=InsSave.lsAvgC+Ins.EffectiveI;
		InsSave.PowerOn30++;
	}
		
	if((((nowkWh-prekWh) >=NORMAL_KWH_THRESHOLD_LL)&&((nowkWh-prekWh) <=NORMAL_KWH_THRESHOLD_UL))||(((nowkWh-prekWhinterval) >=NORMAL_KWH_THRESHOLD_LL)&&((nowkWh-prekWhinterval) <=NORMAL_KWH_THRESHOLD_UL)))
	{
		savekWh(nowkWh);
		prekWh=nowkWh;
		prekWhinterval=nowkWh;
		
	}
		
	
	if(((mcu_flag&POWER_STABLE_FLAG)==0)&&isPowerfailLog(USER_CODE))
		task|=RECORD_POWER_FAIL_DATA;
	if((InsSave.timeCounter/300)!=(prevTimeCounter /300))
		getRtcCounter(RTC_NO_CHECK_MODE);
		
	if((InsSave.timeCounter/(3600/md_period ))!=(prevTimeCounter /(3600/md_period)))
		task|=RECORD_MD_DATA;


	if((InsSave.timeCounter/60)!=(prevTimeCounter /60))  // Convert time
		task_RD|=RECORD_RD_MD_DATA;


	if((InsSave.timeCounter/(3600/ls_period) )!=(prevTimeCounter/(3600/ls_period)))    
		task|=RECORD_LS_DATA ;
  
	if((InsSave.timeCounter/86400)!=(prevTimeCounter/86400))
	{
		if(getNextBillingDate()&&(InsSave.DailyDayCount>0||InsSave.IntCount>0))
		{
			task|=RECORD_BILL_DATA;
		}
		
		write_eeprom((uint8_t *)&clear_nm_clock,BAT_NM_LOCK_COUNT_LOC,1);
	}
	
	if(mcu_flag & SINGLE_ACTION_CHECK_FLAG )
	{
		task|=doBillSingleAction();
	}
		
	if(mcu_flag & PASSIVE_CAL_CHECK_FLAG )
	{
		task|=doCalanderAction();
	}		
		
	if(task)
	{
		mcu_flag &= ~COMM_RECEIVE_ENABLE;
		SaveEEPROM(PARA_WRITE_BEFORE_STATE ,PARA_TYPE_INSTANT); // save at before location when MD, LS or month over
	}
	
	if(task&RECORD_MD_DATA)
		Compute_MD(BILL_TYPE_AUTO);
	else if(task&RECORD_MAN_MD_DATA)
		Compute_MD(BILL_TYPE_MAN);
	else if	(task_RD&RECORD_RD_MD_DATA)
		Compute_RD_MD(BILL_TYPE_AUTO);

	

	if(new_md_period!=md_period)
	{
		
		if((((InsSave.timeCounter%3600)%1800)/(3600/new_md_period))==(((InsSave.timeCounter%3600)%1800)/(3600/md_period)))
		{
			md_period=updateMDInterval(MD_PERIOD_CHANGE_FLAG,md_period);
		}
	}		
	
	if(task&RECORD_LS_DATA)
	{
		doLs();
		if((task&RECORD_BILL_DATA)==0)
		{
			if((InsSave.timeCounter/86400)!=(prevTimeCounter/86400))
			{
				if(getNextBillingDate()&&(InsSave.DailyDayCount>0||InsSave.IntCount>0))
				{
					task|=RECORD_BILL_DATA;
				}
			}		
		}
	}
	
	if(task&RECORD_BILL_DATA)
		CheckBillingOver(BILL_TYPE_AUTO,getNextBillingDate());
	
	if(task&RECORD_DO_BILL_DATA)
	{
		Compute_MD(BILL_TYPE_MAN);
		CheckBillingOver(BILL_TYPE_MAN,InsSave.timeCounter );
	}

	if(task&RECORD_POWER_FAIL_DATA)
	{
		powerfailLogging();
		updatePfailEvent();
	}
	
	prevTimeCounter=InsSave.timeCounter;
	
	if(task)
	{
		R_OBIS_Class07_BlockloadUpdate();
		SaveEEPROM(PARA_WRITE_AFTER_STATE ,PARA_TYPE_PFAIL_INSTANT); // save at current location
		SaveEEPROM(PARA_WRITE_AFTER_STATE  ,PARA_TYPE_INSTANT); // save at after location
		
		mcu_flag |= COMM_RECEIVE_ENABLE;
		#if (defined(IS_DISPLAY_CONFIGURABLE) && (IS_DISPLAY_CONFIGURABLE == 1))
			validatelcdRAMBuff();
		#endif
	}
	
	//#if(defined(UART_0_ENABLE) && (UART_0_ENABLE==1))
//	Tamperlog_AP();	
	Tamperlog();	
//	#endif

//	#if(defined(UART_2_ENABLE) && (UART_2_ENABLE==1))


//	#endif
	
	
	//-------------------------------------
	CheckParaTimeOut();
	Display(USER_CODE);
	//-------------------------------------
	
	
	if((mcu_flag&POWER_STABLE_FLAG)==0)
	{
		if(powerStableCounter<5)
		{
			powerStableCounter++;
			if(powerStableCounter==5)
				SaveEEPROM(PARA_WRITE_BEFORE_STATE ,PARA_TYPE_INSTANT); // save at before location when MD, LS or month over
		}
		else
		{
			power_fail_state|=SAVE_LEVEL1_KWH_STATE_OPEN;			
			mcu_flag|=POWER_STABLE_FLAG;
			powerStableCounter=0;
			SWITCH_ON_RTC_VDD;
			SaveEEPROM(PARA_WRITE_AFTER_STATE  ,PARA_TYPE_INSTANT); // save at after location
			SaveEEPROM(PARA_WRITE_AFTER_STATE ,PARA_TYPE_PFAIL_INSTANT); // save at current location
			
		}
	}
	else 
	{
		if((power_fail_state&SAVE_LEVEL1_KWH_STATE_OPEN)==0)
		{
			if(powerStableCounter<10)
				powerStableCounter++;
			else
			{
				power_fail_state|=SAVE_LEVEL2_KWH_STATE_OPEN;
				powerStableCounter=0;
			}
		}
		
	}
		
	isRAMOK();	
	getSetPfailData(ENERGY_W_RAM);
}	

