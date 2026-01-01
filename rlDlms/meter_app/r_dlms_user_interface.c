/******************************************************************************
* DISCLAIMER

* This software is supplied by Renesas Electronics Corporation and is only
* intended for use with Renesas products. No other uses are authorized.

* This software is owned by Renesas Electronics Corpofration and is protected under
* all applicable laws, including copyright laws.

* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES
* REGARDING THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY,
* INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
* PARTICULAR PURPOSE AND NON-INFRINGEMENT.  ALL SUCH WARRANTIES ARE EXPRESSLY
* DISCLAIMED.

* TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS
* ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE
* FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES
* FOR ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR ITS
* AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.

* Renesas reserves the right, without notice, to make changes to this
* software and to discontinue the availability of this software.
* By using this software, you agree to the additional terms and
* conditions found by accessing the following link:
* http://www.renesas.com/disclaimer
******************************************************************************/
/* Copyright (C) 2016 Renesas Electronics Corporation. All rights reserved.  */
/******************************************************************************
* File Name    : r_dlms_user_interface.c
* Version      : 1.00
* Device(s)    : None
* Tool-Chain   :
* H/W Platform : Unified Energy Meter Platform
* Description  :
******************************************************************************
* History : DD.MM.YYYY Version Description
*         : 01.03.2016
******************************************************************************/

/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
/* DLMS */
#include "..\\r_dlms_typedef.h"				/* DLMS Type Definitions */
#include "r_dlms_data.h"				/* DLMS Data Definitions */

#include "..\\Objects\\r_dlms_obis.h"				/* DLMS OBIS Definitions */
#include "..\\Objects\\r_dlms_obis_ic.h"				/* DLMS OBIS IC Definitions */
#include "..\\DLMS_User.h"					/* DLMS User Definitions */
#include "r_dlms_user_interface.h"		/* DLMS User Interface Definitions */
#include "r_dlms_data_meter.h"
#include "..\\r_cg_macrodriver.h"

extern  date_time_t g_Class20_ac_time;
extern Unsigned8 g_Class20_week_name1[8],g_Class20_week_name2[8];//g_Class20_week_name3[8];
extern Unsigned8 g_Class20_season_name1[8],g_Class20_season_name2[8];//g_Class20_season_name3[8];




//---------------Device specific-------------
#include "..\\..\\rlDevice\\include\\dRtc.h"
#include "..\\..\\rlDevice\\include\\dI2c.h"
#include "..\\..\\rlDevice\\Include\\dWatchDog.h"
#include "..\\..\\rlDevice\\Include\\dIOdefine.h"

//---------------Application specific-------------
#include "..\\..\\rlApplication\\Include\\AppVariables.h"
#include "..\\..\\rlApplication\\Include\\AppLcd.h"
#include "..\\..\\rlApplication\\Include\\AppMisc.h"
#include "..\\..\\rlApplication\\Include\\AppEeprom.h"
#include "..\\..\\rlApplication\\Include\\AppBilling.h"
#include "..\\..\\rlApplication\\Include\\AppTampers.h"
#include "..\\..\\rlApplication\\Include\\AppBlockDailyLS.h"
#include "..\\..\\rlApplication\\Include\\AppTod.h"
#include "..\\..\\rlApplication\\Include\\AppMD.h"
#include "..\\..\\rlApplication\\Include\\AppNMTask.h"
#include "..\\..\\rlApplication\\Include\\AppConfig.h"

//-------------------------------------------


/* Standard Lib */
#include <string.h>						/* String Standard Lib */

#include "format.h"


/******************************************************************************
Macro definitions
******************************************************************************/

/******************************************************************************
Typedef definitions
******************************************************************************/

/******************************************************************************
Imported global variables and functions (from other files)
******************************************************************************/
extern date_time_t			g_Class04_ActiveMDCaptureTime;
extern date_time_t			g_Class04_ApparentMDCaptureTime;

/* Class ID = 07 - Profile generic class */
#if (defined(USED_CLASS_07) && USED_CLASS_07 == 1)
extern Unsigned8					g_Class07_Obj0_Capture_Enable;
extern Unsigned32				g_Class07_Blockload_CapturePeriod;
extern Unsigned32					g_Class07_Blockload_EntriesInUse;
extern Unsigned16					g_Class07_Blockload_CurrentEntry;
extern  Unsigned32					g_Class07_Blockload_MaxEntries;
//extern class07_blockload_entry_t	g_Class07_BlockLoadBuffer[];

extern  Unsigned32				g_Class07_Dailyload_CapturePeriod;
extern Unsigned32					g_Class07_Dailyload_EntriesInUse;
extern Unsigned16					g_Class07_Dailyload_CurrentEntry;
extern  Unsigned32					g_Class07_Dailyload_MaxEntries;
//extern class07_dailyload_entry_t	g_Class07_DailyLoadBuffer[];

extern Unsigned32					g_Class07_Billing_EntriesInUse;
extern Unsigned16					g_Class07_Billing_CurrentEntry;
extern  Unsigned32					g_Class07_Billing_MaxEntries;
//extern class07_billing_entry_t	g_Class07_BillingBuffer[];

extern Unsigned32					g_Class07_Event_EntriesInUse[];
extern Unsigned16					g_Class07_Event_CurrentEntry[];
extern Unsigned32					g_Class07_Event_MaxEntries[];
//extern class07_event_entry_t		g_Class07_EventBuffer[CLASS07_EVENT_MAX_PROFILE][CLASS07_EVENT_MAX_ENTRY];
//extern STORAGE_EEPROM_HEADER 		g_storage_header;								/* Storage Header */
#endif


extern  Unsigned8 secret1[];
extern Unsigned8 secret2[];
extern class07_billing_entry_t g_Class07_BillingBuffer;
unsigned char set_rtc_time;
unsigned char self_diagnostics_status, dlc_status;
 

extern unsigned char ldn_name[];
extern unsigned char manfact_name[];
extern unsigned char meter_rating[];
extern unsigned char firmware_version[];

//extern void convert_data_string(unsigned long int tmp_long, unsigned char size);

/******************************************************************************
Exported global variables and functions (to be accessed by other files)
******************************************************************************/

/******************************************************************************
Private global variables and functions
******************************************************************************/
/* RL78/G13 Day Of Week Driver */
const Unsigned8 g_DayOfWeek[7] =
{	
	DATE_WEEK_SUN,		/* 0 is Sunday */
	DATE_WEEK_MON,		/* 1 is Monday */
	DATE_WEEK_TUE,		/* 2 is Tuesday */
	DATE_WEEK_WED,		/* 3 is Wednesday */
	DATE_WEEK_THU,		/* 4 is Thusday */
	DATE_WEEK_FRI,		/* 5 is Friday */
	DATE_WEEK_SAT,		/* 6 is Saturday */
	
};
/******************************************************************************
* Function Name : R_USER_Set_RTC_timezone
* Interface     : void R_USER_Set_RTC_timezone(Unsigned16)
* Description   : Set RTC Time Zone
* Arguments     : Time zone value in Unsigned16
* Function Calls: TBD
* Return Value  : None
******************************************************************************/
void R_USER_Set_RTC_timezone(void)
{
	//g_Class08_RTC_TimeZone
	
	
}
/******************************************************************************

******************************************************************************/
uint32_t R_USER_ConvertrDLMStoAppRTC(const date_time_t *p_date_time,Apprtc * rtc)
{
	uint16_t timeout=p_date_time->year_high;
	timeout<<=8;
	timeout+=p_date_time->year_low;
	rtc->year=timeout%2000;
	rtc->month=p_date_time->month;
	rtc->day=p_date_time->day_of_month;
	rtc->hour=p_date_time->hour;
	rtc->min=p_date_time->minute;
	rtc->sec=p_date_time->second;
	return ConvertTimeToTimeCounter(rtc);
	
}

/******************************************************************************
* Function Name : R_USER_GetRTCTime
* Interface     : void R_USER_GetRTCTime(date_time_t *p_date_time,uint32_t timeCounter)
* Description   : Get RTC date time
* Arguments     : date_time_t * p_date_time: Buffer to store RTC date time
* Function Calls: TBD
* Return Value  : None
******************************************************************************/
void R_USER_GetRTCTime(date_time_t *p_date_time,uint32_t timeCounter)
{
	uint16_t timeout;		
	Apprtc rtctime; /* RTC value for Driver IF */

	/* Check parameters */
	if (p_date_time == NULL)
	{
		return;
	}

	ConvertTimeCounterToTime(timeCounter,DATE_VAL,&rtctime);


	/* Convert to date_time_t structure */
	/* HIGH of Year */
	timeout = 2000 + rtctime.year;
	p_date_time->year_high         = (Unsigned8)(timeout >> 8);
	/* LOW  of Year */
	p_date_time->year_low          = (Unsigned8)timeout;
	p_date_time->month             = (Unsigned8)rtctime.month;					/* Month */
	p_date_time->day_of_month      = (Unsigned8)rtctime.day;					/* Day */
	p_date_time->day_of_week       = (Unsigned8)g_DayOfWeek[rtctime.week];		/* Friday */
	p_date_time->hour              = (Unsigned8)rtctime.hour;					/* Hour */
	p_date_time->minute            = (Unsigned8)rtctime.min;					/* Minute */
	p_date_time->second            = (Unsigned8)rtctime.sec;					/* Second */
	/* Not support */
	p_date_time->hundredths        = (Unsigned8)TIME_HUNDREDTHS_NOT_SPECIFIED;
	/* Initial at GMT +7 */
	p_date_time->deviation_high    = 0x80;//(Unsigned8)(((Unsigned16)7 * 60) >> 8);
	p_date_time->deviation_low     = 0x00;//(Unsigned8)(((Unsigned16)7 * 60));
	/* Daylight saving active */
	p_date_time->clock_status      =0x00;// (Unsigned8)CLOCK_STATUS_DAYLIGHT_SAVING_ACTIVE;
}

/******************************************************************************
* Function Name : R_USER_SetRTCTime
* Interface     : void R_USER_SetRTCTime(date_time_t date_time)
* Description   : Set RTC date time
* Arguments     : date_time_t date_time: Date time value to set
* Function Calls: TBD
* Return Value  : None
******************************************************************************/
void R_USER_SetRTCTime(date_time_t date_time)
{
	uint8_t  i;      /* Counter */
	uint16_t timeout;	/* Timeout counter to ensure the system not crash */
	Apprtc rtctime;	/* RTC value for Driver IF */

	/* Get the RTC value from date_time */
	rtctime.day     = date_time.day_of_month;
	rtctime.month   = date_time.month;
	timeout         = ((Unsigned16)date_time.year_high << 8);
	timeout        += (Unsigned16)date_time.year_low;
	rtctime.year    = (Unsigned8)((timeout % 100));
	rtctime.hour    = date_time.hour;
	rtctime.min     = date_time.minute;
	rtctime.sec     = date_time.second;
	for (i = 0; i < 7; i++)
	{
		if (g_DayOfWeek[i] == date_time.day_of_week)
		{
			break;
		}
	}
	if (i == 7)
	{
		i = 0;
	}
	rtctime.week    = i;


	logSWTamper(151);
	init_Rtc(1);
	Rtc_SetCounterValue(&rtctime,TIME_VAL);
	Rtc_SetCounterValue(&rtctime,DATE_VAL);
	InsSave.timeCounter=getRtcCounter(RTC_WRITE_MODE);
	
}

/******************************************************************************
* Function Name : R_USER_GetSpecificTime
* Interface     : Unsigned8 R_USER_GetSpecificTime()
* Description   : Get Specific Time
* Arguments     :
*               : Unsigned16 time_id,
*               : Unsigned8* p_date_time
* Function Calls: T.B.D
* Return Value  : Unsigned8: 1: error; 0 : OK
******************************************************************************/
Unsigned8 R_USER_GetSpecificTime(Unsigned16 time_id, date_time_t *p_date_time)
{
	Unsigned8 result = 1;
	Unsigned8 support_log_class7 = 1;
	Unsigned32 billdate;
	Unsigned16 lastbillloc=GetBillLoc(0);


	switch(time_id)
	{
		case EM_CURRENT_TIME:
			R_USER_GetRTCTime(p_date_time,InsSave.timeCounter);
			result = 0;
			break;
		case EM_LASTBILLING_TIME:
			{
				if (support_log_class7 == 1)
				{
					read_eeprom((uint8_t*)&billdate,lastbillloc+25,4);
					R_USER_GetRTCTime(p_date_time,billdate);
				}
			}
			result = 0;
			break;
		case EM_LAST_ACTIVE_MD_TIME:
			{
				if (support_log_class7 == 1)
				{
					read_eeprom((uint8_t*)&billdate,lastbillloc+14,4);
					R_USER_GetRTCTime(p_date_time,billdate);
					
				}
			}
			result = 0;
			break;
		case EM_LAST_APPARENT_MD_TIME:
			{
				if (support_log_class7 == 1)
				{
					read_eeprom((uint8_t*)&billdate,lastbillloc+20,4);
					R_USER_GetRTCTime(p_date_time,billdate);
				}
			}
			result = 0;
			break;
		/* And more … */
		default:
			return 1;
	}

	return result;
}

/******************************************************************************
* Function Name : R_USER_SetSpecificTime
* Interface     : Unsigned8 R_USER_SetSpecificTime()
* Description   : Set Specific Time
* Arguments     :
*               : Unsigned16 time_id,
*               : Unsigned8 date_time
* Function Calls: T.B.D
* Return Value  : Unsigned8: 1: error; 0 : OK
******************************************************************************/
Unsigned8 R_USER_SetSpecificTime(Unsigned16 time_id, date_time_t date_time)
{
	Unsigned8 result = 1;

	switch(time_id)
	{
		case EM_CURRENT_TIME:
			R_USER_SetRTCTime(date_time);
			result = 0;
			break;
		case EM_LASTBILLING_TIME:
		case EM_LAST_ACTIVE_MD_TIME:
		case EM_LAST_APPARENT_MD_TIME:
			result = 1; // Not allow to change
			break;
		/* And more … */
		default:
			return 1;
	}

	return result;
}

/******************************************************************************
* Function Name : R_USER_WDT_Restart
* Interface     : void R_USER_WDT_Restart(void)
* Description   : Watchdog timer restart
* Arguments     : None
* Function Calls: EM_GetActivePower
* Return Value  : Integer32
******************************************************************************/
void R_USER_WDT_Restart(void)
{
	
	WDT_Restart(0xAC);
}



Integer16 R_USER_GetNamePlate(Unsigned16 nameplate_id, Unsigned8* p_data)
{
	Integer16 result_len = -1;
	Unsigned16	u16;
//	Integer32 temp = 0;
//	Unsigned32	addr;
	Unsigned8 category_name[]="C3";
	
	if(p_data == NULL)
	{
		return -1;
	}

	switch(nameplate_id)
	{
		case EM_NAME_PLATE_LOGICAL_NAME:
		
			
			result_len = strlen((void*)ldn_name);
			memcpy((Unsigned8 *)p_data,(Unsigned8 *)ldn_name, result_len);
			break;
		case EM_NAME_PLATE_METER_SERIAL:
			result_len = getMeterSerial(p_data,SR_TYPE_ASCII);
			break;
		case EM_NAME_PLATE_MANUFACT_NAME:
			result_len = strlen((void*)manfact_name);
			memcpy((Unsigned8 *)p_data,(Unsigned8 *)manfact_name, result_len);
			break;
		case EM_NAME_PLATE_FIRMWARE_NAME:
			result_len = strlen((void*)firmware_version);
			memcpy((Unsigned8 *)p_data,(Unsigned8 *)firmware_version, result_len);
			break;
		case EM_NAME_PLATE_METER_TYPE:
			//ATTR_TYPE_LONG_UNSIGNED
			/* (2 bytea) a value of 5 indicates 1P-2W */
			*p_data = 5;
			*(p_data+1) = 0;
			result_len = 2;
			break;
		case EM_NAME_PLATE_CATEGORY_NAME:
			//p_data = "C3";
			result_len = strlen((void*)category_name);
			memcpy((Unsigned8 *)p_data,(Unsigned8 *)category_name, result_len);
			break;
		case EM_NAME_PLATE_CURRRATE_NAME:
			//p_data = (void *)meter_rating;// "Ib-Imax";
			result_len = strlen((void*)meter_rating);
			memcpy((Unsigned8 *)p_data,(Unsigned8 *)meter_rating, result_len);
			break;
		case EM_NAME_PLATE_YEAR_OF_MANUFACT:
			//ATTR_TYPE_LONG_UNSIGNED
			result_len=0;
			read_eeprom((uint8_t *)&result_len,SERIAL_NO_LOC+16+1,1);
			
			u16 = 2000 + result_len;
			
			*p_data++ = (Unsigned8)(u16 &0xFF);
			*p_data++ = (Unsigned8)((u16>>8) &0xFF);
			result_len = 2;
			break;

		/* And more … */
		default:
			return -1;
	}
	
	return result_len;
}

/******************************************************************************
* Function Name : R_USER_GetEMData
* Interface     : Integer32 R_USER_GetEMData(Unsigned16 em_data)
* Description   : Get EM data from EM
* Arguments     : Unsigned16 em_data: ID for EM data
* Function Calls: T.B.D
* Return Value  : Integer32
******************************************************************************/
Integer32 R_USER_GetEMData(Unsigned16 em_data)
{
	Integer32 result = 0;
	//Integer32 temp = 0;
	//static Integer32 old_result_kwh = 0;
	//static Integer32 old_result_kvah = 0;
	switch(em_data)
	{
		
		case EM_POWER_ON_DURATION:
			/*Total power on duration in minutes*/
			result= InsSave.MonthPowerOnDuration+InsSave.PowerOn30;
			result=result/60;
			break;
		case EM_CUCUMLARIVE_POWER_ON_DURATION:
		/*Cumulative power on duration in minutes*/
			result=InsSave.CumPowerOnDuration+InsSave.PowerOn30;
			result=result/60;
			break;
		case EM_ACTIVE_POWER:
			result = Ins.EffectiveP;
			break;
		case EM_REACTIVE_POWER:
			result = 0;
			break;
		case EM_APPARENT_POWER:
			result = Ins.AppPower;
			break;

		case EM_ACTIVE_MD:			
			R_USER_GetRTCTime(&g_Class04_ActiveMDCaptureTime,InsSave.BillMD.Current.kW_Date);
			result = InsSave.BillMD.Current.kW;			
			break;
		case EM_ACTIVE_MD_TZ1:
			R_USER_GetRTCTime(&g_Class04_ActiveMDCaptureTime,InsSave.TODEnergy.TOD[0].TODMD.kW_Date);
			result = InsSave.TODEnergy.TOD[0].TODMD.kW;			
			break;
			
		case EM_ACTIVE_MD_TZ2:
			R_USER_GetRTCTime(&g_Class04_ActiveMDCaptureTime,InsSave.TODEnergy.TOD[1].TODMD.kW_Date);
			result = InsSave.TODEnergy.TOD[1].TODMD.kW;			
			break;
			
		case EM_ACTIVE_MD_TZ3:
			R_USER_GetRTCTime(&g_Class04_ActiveMDCaptureTime,InsSave.TODEnergy.TOD[2].TODMD.kW_Date);
			result = InsSave.TODEnergy.TOD[2].TODMD.kW;			
			break;
			
		case EM_ACTIVE_MD_TZ4:
			R_USER_GetRTCTime(&g_Class04_ActiveMDCaptureTime,InsSave.TODEnergy.TOD[3].TODMD.kW_Date);
			result = InsSave.TODEnergy.TOD[3].TODMD.kW;			
			break;
			
		case EM_ACTIVE_MD_TZ5:
			R_USER_GetRTCTime(&g_Class04_ActiveMDCaptureTime,InsSave.TODEnergy.TOD[4].TODMD.kW_Date);
			result = InsSave.TODEnergy.TOD[4].TODMD.kW;			
			break;
		
		case EM_ACTIVE_MD_TZ6:
			R_USER_GetRTCTime(&g_Class04_ActiveMDCaptureTime,InsSave.TODEnergy.TOD[5].TODMD.kW_Date);
			result = InsSave.TODEnergy.TOD[5].TODMD.kW;			
			break;
		case EM_ACTIVE_MD_TZ7:
			R_USER_GetRTCTime(&g_Class04_ActiveMDCaptureTime,InsSave.TODEnergy.TOD[6].TODMD.kW_Date);
			result = InsSave.TODEnergy.TOD[6].TODMD.kW;			
			break;
		case EM_ACTIVE_MD_TZ8:
			R_USER_GetRTCTime(&g_Class04_ActiveMDCaptureTime,InsSave.TODEnergy.TOD[7].TODMD.kW_Date);
			result = InsSave.TODEnergy.TOD[7].TODMD.kW;			
			break;			
			
		case EM_REACTIVE_MD:
			break;
		case EM_APPARENT_MD:
			R_USER_GetRTCTime(&g_Class04_ApparentMDCaptureTime,InsSave.BillMD.Current.kVA_Date);
			result = InsSave.BillMD.Current.kVA;			
			break;
			
		case EM_APPARENT_MD_TZ1:
			R_USER_GetRTCTime(&g_Class04_ApparentMDCaptureTime,InsSave.TODEnergy.TOD[0].TODMD.kVA_Date);
			result = InsSave.TODEnergy.TOD[0].TODMD.kVA;
			break;
			
		case EM_APPARENT_MD_TZ2:
			R_USER_GetRTCTime(&g_Class04_ApparentMDCaptureTime,InsSave.TODEnergy.TOD[1].TODMD.kVA_Date);
			result = InsSave.TODEnergy.TOD[1].TODMD.kVA;
			break;
			
		case EM_APPARENT_MD_TZ3:
			R_USER_GetRTCTime(&g_Class04_ApparentMDCaptureTime,InsSave.TODEnergy.TOD[2].TODMD.kVA_Date);
			result = InsSave.TODEnergy.TOD[2].TODMD.kVA;
			break;
			
		case EM_APPARENT_MD_TZ4:
			R_USER_GetRTCTime(&g_Class04_ApparentMDCaptureTime,InsSave.TODEnergy.TOD[3].TODMD.kVA_Date);
			result = InsSave.TODEnergy.TOD[3].TODMD.kVA;
			break;
			
		case EM_APPARENT_MD_TZ5:
			R_USER_GetRTCTime(&g_Class04_ApparentMDCaptureTime,InsSave.TODEnergy.TOD[4].TODMD.kVA_Date);
			result = InsSave.TODEnergy.TOD[4].TODMD.kVA;
			break;
			
		case EM_APPARENT_MD_TZ6:
			R_USER_GetRTCTime(&g_Class04_ApparentMDCaptureTime,InsSave.TODEnergy.TOD[5].TODMD.kVA_Date);
			result = InsSave.TODEnergy.TOD[5].TODMD.kVA;
			break;
		case EM_APPARENT_MD_TZ7:
			R_USER_GetRTCTime(&g_Class04_ApparentMDCaptureTime,InsSave.TODEnergy.TOD[6].TODMD.kVA_Date);
			result = InsSave.TODEnergy.TOD[6].TODMD.kVA;
			break;
		case EM_APPARENT_MD_TZ8:
			R_USER_GetRTCTime(&g_Class04_ApparentMDCaptureTime,InsSave.TODEnergy.TOD[7].TODMD.kVA_Date);
			result = InsSave.TODEnergy.TOD[7].TODMD.kVA;
			break;			

		case EM_ACTIVE_TARIFF:
			break;
		case EM_REACTIVE_TARIFF:
			break;
		case EM_APPARENT_TARIFF:
			break;
		case EM_AVERAGE_VOLTAGE:
			
			#if (defined(OLD_AVG_U_AVG_C_LOGIC) && (OLD_AVG_U_AVG_C_LOGIC == 1))
				result = InsSave.lsAvgV/100;
				if(InsSave.PowerOn30>0)
					result = result/InsSave.PowerOn30;
			#else
				result=InsSave.timeCounter%(3600/ls_period);
				if(result)
					result=InsSave.lsAvgV/result;
				result=result/100;
			#endif
			result=result*100;
				
			break;
		case EM_BLOCK_AVG_PF:
			result=InsSave.kWhCounter*100/METER_CONSTANT;
			if(InsSave.kVAhCounter>0)
				result = result/(InsSave.kVAhCounter/METER_CONSTANT);
			else
				result=0;
			if(result>100)
				result=100;
			break;
		case EM_VOLTAGE:
			result = Ins.Voltage;
			break;

		case EM_CURRENT:
			result = Ins.EffectiveI;
			break;
		case EM_PHASE_CURRENT:
			result = Ins.PhCurrent;
			break;
		case EM_NEUTRAL_CURRENT:
			result = Ins.NeuCurrent;
			break;

		case EM_AVERAGE_POWER_FACTOR:
			result=getAvgPf();
			break;
			
		case EM_POWER_FACTOR:
			result=Ins.PowerFactor;
			if((Ins.LeadLag==1)&&(Ins.EffectiveP>STARTING_POWER_THRESHOLD_L)&&((display_flag&DISP_NM)==0))
				result=0x100000000L-result;
			
			break;

		case EM_LINE_FREQ:
			result = Ins.Frequency;
			break;

		case EM_DEMAND_INTEGRATION_PERIOD:
			/* Exchange from minutes to seconds*/
			result = 3600/md_period;
			break;
		case EM_PROFILE_CAPTURE_PERIOD:
			/* Exchange from minutes to seconds*/
			result = 3600/ls_period;
			break;
			
		case EM_CUCUMLARIVE_PROGAM_CNT:
			result=TamperRecord.sw_log_count;
			break;
						
		case EM_CUCUMLARIVE_BILLING_CNT:
			result = InsSave.MDResetCount;
			break;
			
		case EM_CUCUMLARIVE_TAMPER_CNT:
			result = getTotalTamperCount();
			break;

		case EM_BLOCK_ENERGY_KWH:
			result = InsSave.kWhCounter/METER_CONSTANT;
			break;
		case EM_CUCUMLARIVE_ENERGY_KWH:
			result = InsSave.CumkWh+InsSave.ZkWhCounter/METER_CONSTANT;
			break;
		case EM_CUCUMLARIVE_ENERGY_KWH_TZ1:
			result = InsSave.TODEnergy.TOD[0].kWh;
			#if (defined(GUJRAT_METER_0) && (GUJRAT_METER_0 == 1))
			if(currentZoneID==0)
				result+=InsSave.ExpZkWhCounter/METER_CONSTANT;
			#else
			if(currentZoneID==0)
				result+=InsSave.ZkWhCounter/METER_CONSTANT;
			#endif
			break;
		case EM_CUCUMLARIVE_ENERGY_KWH_TZ2:
			result = InsSave.TODEnergy.TOD[1].kWh;
			#if (defined(GUJRAT_METER_0) && (GUJRAT_METER_0 == 1))
			if(currentZoneID==1)
				result+=InsSave.ExpZkWhCounter/METER_CONSTANT;
			#else
			if(currentZoneID==1)
				result+=InsSave.ZkWhCounter/METER_CONSTANT;
			#endif
			
			break;
		case EM_CUCUMLARIVE_ENERGY_KWH_TZ3:
			result = InsSave.TODEnergy.TOD[2].kWh;
			#if (defined(GUJRAT_METER_0) && (GUJRAT_METER_0 == 1))
			if(currentZoneID==2)
				result+=InsSave.ExpZkWhCounter/METER_CONSTANT;
			#else
			if(currentZoneID==2)
				result+=InsSave.ZkWhCounter/METER_CONSTANT;
			#endif
			
			break;
		case EM_CUCUMLARIVE_ENERGY_KWH_TZ4:
			result = InsSave.TODEnergy.TOD[3].kWh;
			#if (defined(GUJRAT_METER_0) && (GUJRAT_METER_0 == 1))
			if(currentZoneID==3)
				result+=InsSave.ExpZkWhCounter/METER_CONSTANT;
			#else
			if(currentZoneID==3)
				result+=InsSave.ZkWhCounter/METER_CONSTANT;
			#endif
			
			break;
		case EM_CUCUMLARIVE_ENERGY_KWH_TZ5:
			result = InsSave.TODEnergy.TOD[4].kWh;
			#if (defined(GUJRAT_METER_0) && (GUJRAT_METER_0 == 1))
			if(currentZoneID==4)
				result+=InsSave.ExpZkWhCounter/METER_CONSTANT;
			#else
			if(currentZoneID==4)
				result+=InsSave.ZkWhCounter/METER_CONSTANT;
			#endif
			
			break;
		case EM_CUCUMLARIVE_ENERGY_KWH_TZ6:
			result = InsSave.TODEnergy.TOD[5].kWh;
			#if (defined(GUJRAT_METER_0) && (GUJRAT_METER_0 == 1))
			if(currentZoneID==5)
				result+=InsSave.ExpZkWhCounter/METER_CONSTANT;
			#else
			if(currentZoneID==5)
				result+=InsSave.ZkWhCounter/METER_CONSTANT;
			#endif
			
			break;
		case EM_CUCUMLARIVE_ENERGY_KWH_TZ7:
			result = InsSave.TODEnergy.TOD[6].kWh;
			#if (defined(GUJRAT_METER_0) && (GUJRAT_METER_0 == 1))
			if(currentZoneID==6)
				result+=InsSave.ExpZkWhCounter/METER_CONSTANT;
			#else
			if(currentZoneID==6)
				result+=InsSave.ZkWhCounter/METER_CONSTANT;
			#endif
			
			break;
		case EM_CUCUMLARIVE_ENERGY_KWH_TZ8:
			result = InsSave.TODEnergy.TOD[7].kWh;
			#if (defined(GUJRAT_METER_0) && (GUJRAT_METER_0 == 1))
			if(currentZoneID==7)
				result+=InsSave.ExpZkWhCounter/METER_CONSTANT;
			#else
			if(currentZoneID==7)
				result+=InsSave.ZkWhCounter/METER_CONSTANT;
			#endif
			
			break;			
		case EM_BLOCK_ENERGY_KVAH:
			result =InsSave.kVAhCounter/METER_CONSTANT;
			break;
			
		case EM_CUCUMLARIVE_ENERGY_KVAH:
			result = InsSave.CumkVAh+InsSave.ZkVAhCounter/METER_CONSTANT;

			break;
		case EM_CUCUMLARIVE_ENERGY_KVAH_TZ1:
			result = InsSave.TODEnergy.TOD[0].kVAh;
			#if (defined(GUJRAT_METER_0) && (GUJRAT_METER_0 == 1))
			if(currentZoneID==0)
				result+=InsSave.ExpZkVAhCounter/METER_CONSTANT;
			#else
			if(currentZoneID==0)
				result+=InsSave.ZkVAhCounter/METER_CONSTANT;
			#endif
				
			
			break;
		case EM_CUCUMLARIVE_ENERGY_KVAH_TZ2:
			result = InsSave.TODEnergy.TOD[1].kVAh;
			#if (defined(GUJRAT_METER_0) && (GUJRAT_METER_0 == 1))
			if(currentZoneID==1)
				result+=InsSave.ExpZkVAhCounter/METER_CONSTANT;
			#else
			if(currentZoneID==1)
				result+=InsSave.ZkVAhCounter/METER_CONSTANT;
			#endif
			
			break;
		case EM_CUCUMLARIVE_ENERGY_KVAH_TZ3:
			result = InsSave.TODEnergy.TOD[2].kVAh;
			#if (defined(GUJRAT_METER_0) && (GUJRAT_METER_0 == 1))
			if(currentZoneID==2)
				result+=InsSave.ExpZkVAhCounter/METER_CONSTANT;
			#else
			if(currentZoneID==2)
				result+=InsSave.ZkVAhCounter/METER_CONSTANT;
			#endif
			
			break;
		case EM_CUCUMLARIVE_ENERGY_KVAH_TZ4:
			result = InsSave.TODEnergy.TOD[3].kVAh;
			if(currentZoneID==3)
				result+=InsSave.ExpZkVAhCounter/METER_CONSTANT;
			
			break;		
		case EM_CUCUMLARIVE_ENERGY_KVAH_TZ5:
			result = InsSave.TODEnergy.TOD[4].kVAh;
			#if (defined(GUJRAT_METER_0) && (GUJRAT_METER_0 == 1))
			if(currentZoneID==4)
				result+=InsSave.ExpZkVAhCounter/METER_CONSTANT;
			#else
			if(currentZoneID==4)
				result+=InsSave.ZkVAhCounter/METER_CONSTANT;
			#endif
			
			break;
		case EM_CUCUMLARIVE_ENERGY_KVAH_TZ6:
			result = InsSave.TODEnergy.TOD[5].kVAh;
			#if (defined(GUJRAT_METER_0) && (GUJRAT_METER_0 == 1))
			if(currentZoneID==5)
				result+=InsSave.ExpZkVAhCounter/METER_CONSTANT;
			#else
			if(currentZoneID==5)
				result+=InsSave.ZkVAhCounter/METER_CONSTANT;
			#endif
			
			break;		
		case EM_CUCUMLARIVE_ENERGY_KVAH_TZ7:
			result = InsSave.TODEnergy.TOD[6].kVAh;
			#if (defined(GUJRAT_METER_0) && (GUJRAT_METER_0 == 1))
			if(currentZoneID==6)
				result+=InsSave.ExpZkVAhCounter/METER_CONSTANT;
			#else
			if(currentZoneID==6)
				result+=InsSave.ZkVAhCounter/METER_CONSTANT;
			#endif
			
			break;		
		case EM_CUCUMLARIVE_ENERGY_KVAH_TZ8:
			result = InsSave.TODEnergy.TOD[7].kVAh;
			#if (defined(GUJRAT_METER_0) && (GUJRAT_METER_0 == 1))
			if(currentZoneID==7)
				result+=InsSave.ExpZkVAhCounter/METER_CONSTANT;
			#else
			if(currentZoneID==7)
				result+=InsSave.ZkVAhCounter/METER_CONSTANT;
			#endif
			
			break;		
			
		case EM_AVL_BILLING_PERIOD:
			if(InsSave.MDResetCount>=HISTORY_RECORD)
				result = HISTORY_RECORD;
			else
				result = InsSave.MDResetCount;
			break;
		case EM_INTERNAL_DIAGNOSTICS:
			read_eeprom((uint8_t*)&result,RTC_BAT_STATUS_LOC,1);
			if(((result&0xAB)==0xAB)||(Ins.Voltage<10000))
			{
				result=0;
				result |=0x04;
			}
			
		//	if((power_fail_state&RTC_BAT_STATUS_FLAG)==0)
		//		result |=0x01;
			else if (LVDVRTCF == 1 )
			{
			result |=0x01;
			}
			else
			result=0;
		//	result |=0x02;
		//	if(GET_BLE_STATUS)
		//		result|=0x08;
			break;
		case EM_DOWNLOAD_COMPLETE:
			result = dlc_status;
			break;
		case EM_CUCUMLARIVE_EXP_ENERGY_KWH:
			result = 0;//InsSave.ExpCumkWh+InsSave.ExpZkWhCounter/METER_CONSTANT;
			break;
		case EM_CUCUMLARIVE_EXP_ENERGY_KVAH:
			result = 0;//InsSave.ExpCumkVAh+InsSave.ExpZkVAhCounter/METER_CONSTANT;
			break;
		case EM_PRE_BILL_DATE_READ:
			result=getAutoBillDay();
			
			break;
		case EM_BLOCK_ENERGY_KWH_EXP:
			result = 0;//InsSave.ExpkWhCounter/METER_CONSTANT;
			break;
		case EM_BLOCK_AVG_CURENT:
			#if (defined(OLD_AVG_U_AVG_C_LOGIC) && (OLD_AVG_U_AVG_C_LOGIC == 1))
			result = InsSave.lsAvgC;
			if(InsSave.PowerOn30>0)
				result = result/InsSave.PowerOn30;
			#else
				result=InsSave.timeCounter%(3600/ls_period);
				if(result)
					result=InsSave.lsAvgC/result;
			#endif
			result = result/10;	
			
			break;
			
		case EM_BLOCK_AVG_LOAD:
			result = InsSave.kWhCounter/METER_CONSTANT;
			result=result*ls_period;
			break;
			
		case LAG_ONLY_CONFIG:
			result = lag_only_value;
			break;	
		/* And more … */
		default:
			break;
	}

	return result;
}

/******************************************************************************
* Function Name : R_USER_SetEMData
* Interface     : Integer8 R_USER_SetEMData(Unsigned16 em_data, Unsigned8 *p_em_value)
* Description   : Set Em data to EM
* Arguments     : Unsigned16 em_data: ID for EM data
* Function Calls: T.B.D
* Return Value  : 0: OK, -1: not supported, -2: error
******************************************************************************/
Integer8 R_USER_SetEMData(Unsigned16 em_data, Unsigned8 *p_em_value)
{
	/* Result of response, def. is not Matched */
	Unsigned8 response_result = TYPE_UNMATCH;
	Unsigned16 tmp_long=0;
	date_t * tt;
	
	switch(em_data)
	{
		/*
		* TODO: Put your code HERE to set em_value to EM
		* base on em_data, see "ID For EM data" on
		* r_dlms_data_meter.h
		*/
		case EM_DOWNLOAD_COMPLETE: //USER CODE			
			tmp_long =(Unsigned16 )p_em_value;
			//Set_PP(tmp_long,6);			
			response_result = DATA_ACCESS_RESULT_SUCCESS;
			break;
		case EM_DEMAND_INTEGRATION_PERIOD: //USER CODE
			/* Exchange from minutes to seconds*/
			
			
			tmp_long=getByteValue((Unsigned8 *)p_em_value,0,2);
			new_md_period =3600/tmp_long;
			updateMDInterval(MD_PERIOD_SET_FLAG,tmp_long);
			logSWTamper(152);
			response_result = DATA_ACCESS_RESULT_SUCCESS;
			break;
		case EM_PROFILE_CAPTURE_PERIOD: //USER CODE
			/* Exchange from minutes to seconds*/
			
			
			tmp_long=getByteValue((Unsigned8 *)p_em_value,0,2);
			
			tmp_long=3600/tmp_long;
			
			write_eeprom((uint8_t *)&tmp_long,LS_IP,1);
			if(tmp_long!=ls_period )
			{
				ls_period=tmp_long;
				ClearLsData();
			}
			
			logSWTamper(153);
			response_result = DATA_ACCESS_RESULT_SUCCESS;
			break;
		case EM_BILL_DATE: //USER CODE
			tt=(date_t *)p_em_value;
			write_eeprom((unsigned char *)&tt->day_of_month ,PRE_BILL_DATE,1);
			response_result = DATA_ACCESS_RESULT_SUCCESS;
			break;
		case EM_CUCUMLARIVE_TAMPER_CNT:
			{
			}
			break;
		case EM_CUCUMLARIVE_BILLING_CNT:
			{
			}
			break;
		case EM_CUCUMLARIVE_PROGAM_CNT:
			{
			}
			break;

		case LAG_ONLY_CONFIG:
			{
			tmp_long=getByteValue((Unsigned8 *)p_em_value,0,2);
			lag_only_value=tmp_long;
			write_eeprom((uint8_t *)&tmp_long,LAG_ONLY_LOC,1);
			logSWTamper(191);
			response_result = DATA_ACCESS_RESULT_SUCCESS;
			break;
			}

			break;	
		default:
			break;
	}

	return response_result;}

#if 0 //RENESAS not reqiured as it is already in the R_OBIS_GetEMData
#if (defined(USED_CLASS_04) && USED_CLASS_04 == 1)
/* Class04 - Extended Register */
/******************************************************************************
* Function Name : R_OBIS_Class04_GetActiveMD
* Interface     : Unsigned8 R_OBIS_Class04_GetActiveMD()
* Description   : Get Active MD
* Arguments     : Integer32             *p_value,
*				  date_time_t         *p_time,
* Function Calls: T.B.D
* Return Value  : Unsigned8: 0: OK, else: NG
******************************************************************************/
Unsigned8 R_OBIS_Class04_GetActiveMD(
	Integer32             *p_value,
	date_time_t         *p_time
)
{
	return 0;
}

/******************************************************************************
* Function Name : R_OBIS_Class04_GetApparentMD
* Interface     : Unsigned8 R_OBIS_Class04_GetApparentMD()
* Description   : Get Apparent MD
* Arguments     : Integer32             *p_value,
*				  date_time_t         *p_time,
* Function Calls: T.B.D
* Return Value  : Unsigned8: 0: OK, else: NG
******************************************************************************/
Unsigned8 R_OBIS_Class04_GetApparentMD(
	Integer32             *p_value,
	date_time_t         *p_time
)
{

	return 0;
}
#endif /* #if (defined(USED_CLASS_04) && USED_CLASS_04 == 1) */
#endif

#if (defined(USED_CLASS_07) && USED_CLASS_07 == 1)
/* Class07 - Generic profile, Blockload profile */

/******************************************************************************
* Function Name : R_OBIS_Class07_GetBlockLoadEntry
* Interface     : Unsigned8 R_OBIS_Class07_GetBlockLoadEntry()
* Description   : Get One Block Load Entry
* Arguments     :
*				class07_blockload_entry_t		*buf,
*				Unsigned16						index
* Function Calls: T.B.D
* Return Value  : Unsigned8: 0: OK, else: NG
******************************************************************************/
Unsigned16 R_OBIS_Class07_GetBlockLoadEntry(
	class07_blockload_entry_t		*buf,
	Unsigned16						index
)
{

	Unsigned32	addr;
	Unsigned32 time_count;
	Unsigned8 lsBuffer[LS_EVENT_LENGTH+4],isDateinNM=0;

	addr=GetLsLoadLoc(index / (24*ls_period));
	
	// get day date
	
	if((nm_ls_end_index==0)||(index / (24*ls_period)<nm_ls_start_index/(24*ls_period)))
	{
		read_eeprom((uint8_t *)&lsBuffer, addr, 4);
		
	}
	else
	{
		getNMStartDate(lsBuffer);
		isDateinNM=1;
	}
	
	time_count=(index % (24*ls_period));
	time_count=time_count*(3600/ls_period);
	time_count=time_count+getByteValue(lsBuffer, 0, 4);
	
	R_USER_GetRTCTime(&buf->clock_value,time_count);

	addr+=4;
	addr += ( LS_EVENT_LENGTH * (index % (24*ls_period)) );
	
	
	if(index<nm_ls_start_index)
	{
		read_eeprom((uint8_t *)&lsBuffer, addr, LS_EVENT_LENGTH);
	}
	else
	{
		
		getNMIntervalLS(lsBuffer,((index % (24*ls_period))-nm_ls_start_index%(24*ls_period))*LS_EVENT_LENGTH+4*isDateinNM);	
	}
		
	buf->voltage_value = getByteValue(lsBuffer, 0, 1)*200;// Block AvgV
	time_count=getByteValue(lsBuffer, 1, 4);
	buf->kWh_value = time_count&0x0FFF;// Block kWh
	time_count>>=12;
	buf->blockkVAh_value=getByteValue(lsBuffer, 4, 2);
	buf->avg_current =time_count&0x0FFF; // Block Avg Current
	buf->avg_current=buf->avg_current*10;
//	buf->pf=buf->kWh_value*ls_period;
	if(buf->kWh_value>0)
		buf->pf=buf->kWh_value*100/buf->blockkVAh_value;
	else
		buf->pf=0;
	if(buf->pf>100)
		buf->pf=100;

//	buf->blockkVAh_value=getByteValue(lsBuffer, 4, 2);

	return 0;
}

/* Class07 - Generic profile, Dailyload profile */

/******************************************************************************
* Function Name : R_OBIS_Class07_GetDailyLoadEntry
* Interface     : Unsigned8 R_OBIS_Class07_GetDailyLoadEntry()
* Description   : Get One Daily Load Entry
* Arguments     :
*				class07_dailyload_entry_t		*buf,
*				Unsigned16						index
* Function Calls: T.B.D
* Return Value  : Unsigned8: 0: OK, else: NG
******************************************************************************/
Unsigned16 R_OBIS_Class07_GetDailyLoadEntry(
	class07_dailyload_entry_t		*buf,
	Unsigned16						index
)
{
	
	Unsigned32	addr;
	Unsigned32 time_count;
	Unsigned8 lsBuffer[LS_DAILY_LENGTH];
	
	addr=GetDailyLoc(index);
	
	
	read_eeprom((uint8_t *)&lsBuffer, addr, LS_DAILY_LENGTH);
	
	time_count=getByteValue(lsBuffer, 0, 4);
	
	R_USER_GetRTCTime(&buf->clock_value,time_count);

	buf->kWh_value = getByteValue(lsBuffer, 4, 4);// kWh
	buf->kVAh_value = getByteValue(lsBuffer, 8, 4);// KVAh
	buf->daykW_value = getByteValue(lsBuffer, 12, 2); // day kW MD
	buf->total_power=getByteValue(lsBuffer,14, 4)/60;
	

	return 0;
}



/* Class07 - Generic profile, Billing profile */

/******************************************************************************
* Function Name : R_OBIS_Class07_GetBillingEntry
* Interface     : Unsigned8 R_OBIS_Class07_GetBillingEntry()
* Description   : Get One Billing Entry
* Arguments     :
*				class07_billing_entry_t		*buf,
*				Unsigned16						index
* Function Calls: T.B.D
* Return Value  : Unsigned8: 0: OK, else: NG
******************************************************************************/
Unsigned16 R_OBIS_Class07_GetBillingEntry(
	class07_billing_entry_t			*buf,
	Unsigned16						index
)
{
	Unsigned32	addr;
	Unsigned32 time_count;
	Unsigned8 billBuffer[BILLING_DATA_LENGTH];
	Unsigned8 max_zone_no=8;

	
	addr=GetLastBillLoc(index);
	read_eeprom((uint8_t *)&billBuffer, addr, BILLING_DATA_LENGTH);

	
	if((index+1)==g_Class07_Billing_EntriesInUse)
	{
		
		buf->active_energy=InsSave.CumkWh;
		buf->active_energy=buf->active_energy+InsSave.ZkWhCounter/METER_CONSTANT;

		buf->exp_active_energy=InsSave.ExpCumkWh;
		buf->exp_active_energy=buf->exp_active_energy+InsSave.ExpZkWhCounter/METER_CONSTANT;

				
		buf->apparent_energy=InsSave.CumkVAh;
		buf->apparent_energy=buf->apparent_energy+InsSave.ZkVAhCounter/METER_CONSTANT;

		buf->exp_apparent_energy=InsSave.ExpCumkVAh;
		buf->exp_apparent_energy=buf->exp_apparent_energy+InsSave.ExpZkVAhCounter/METER_CONSTANT;
		
		buf->total_power=(InsSave.MonthPowerOnDuration+InsSave.PowerOn30)/60;
		
		buf->active_MD=InsSave.BillMD.Current.kW;
		R_USER_GetRTCTime(&buf->active_MD_DT,InsSave.BillMD.Current.kW_Date);
		
		buf->apparent_MD=InsSave.BillMD.Current.kVA;
		
		R_USER_GetRTCTime(&buf->apparent_MD_DT,InsSave.BillMD.Current.kVA_Date);

		buf->power_factor=getAvgPf();
		
		
		R_USER_GetRTCTime(&buf->bill_date,InsSave.timeCounter);
		
		
		while(max_zone_no--)
		{
			
			buf->active_energyTZ[max_zone_no]=InsSave.TODEnergy.TOD[max_zone_no].kWh;
			buf->apparent_energyTZ[max_zone_no]=InsSave.TODEnergy.TOD[max_zone_no].kVAh;
			
			if(max_zone_no==currentZoneID)
			{
				#if (defined(GUJRAT_METER_0) && (GUJRAT_METER_0 == 1))
				{
					buf->active_energyTZ[max_zone_no]=buf->active_energyTZ[max_zone_no]+InsSave.ExpZkWhCounter/METER_CONSTANT;
					buf->apparent_energyTZ[max_zone_no]=buf->apparent_energyTZ[max_zone_no]+InsSave.ExpZkVAhCounter/METER_CONSTANT;
				}
				#else
				{
					buf->active_energyTZ[max_zone_no]=buf->active_energyTZ[max_zone_no]+InsSave.ZkWhCounter/METER_CONSTANT;
					buf->apparent_energyTZ[max_zone_no]=buf->apparent_energyTZ[max_zone_no]+InsSave.ZkVAhCounter/METER_CONSTANT;
					
				}
				#endif
			}
			
			buf->active_TZ[max_zone_no].MD=InsSave.TODEnergy.TOD[max_zone_no].TODMD.kW;
			R_USER_GetRTCTime(&buf->active_TZ[max_zone_no].MD_DT,InsSave.TODEnergy.TOD[max_zone_no].TODMD.kW_Date);
			
			buf->apparent_TZ[max_zone_no].MD=InsSave.TODEnergy.TOD[max_zone_no].TODMD.kVA;
			R_USER_GetRTCTime(&buf->apparent_TZ[max_zone_no].MD_DT,InsSave.TODEnergy.TOD[max_zone_no].TODMD.kVA_Date);
		}		
	}
	else
	{
		buf->active_energy=getByteValue(billBuffer,0, 4);
		buf->exp_active_energy=getByteValue(billBuffer,30, 4);
		buf->apparent_energy=getByteValue(billBuffer,4, 4);
		buf->exp_apparent_energy=getByteValue(billBuffer,34, 4);
		
		buf->total_power=getByteValue(billBuffer,8, 4)/60;
		
		buf->active_MD=getByteValue(billBuffer,12, 2);
		
		time_count=getByteValue(billBuffer,14, 4);
		R_USER_GetRTCTime(&buf->active_MD_DT,time_count);
		
		buf->apparent_MD=getByteValue(billBuffer,18, 2);
		time_count=getByteValue(billBuffer,20, 4);
		R_USER_GetRTCTime(&buf->apparent_MD_DT,time_count);

		buf->power_factor=getByteValue(billBuffer,24, 1);
		
		
		time_count=getByteValue(billBuffer, 25, 4);
		R_USER_GetRTCTime(&buf->bill_date,time_count);
		
		addr=GetTODBillLoc(index);
		
		while(max_zone_no--)
		{
			read_eeprom((uint8_t *)&billBuffer, addr+max_zone_no*ZONE_LEN, ZONE_LEN);
			
			
			buf->active_TZ[max_zone_no].MD=getByteValue(billBuffer,0, 2);	//kW
			buf->apparent_TZ[max_zone_no].MD=getByteValue(billBuffer,2, 2);	//kVA
			
			R_USER_GetRTCTime(&buf->active_TZ[max_zone_no].MD_DT,getByteValue(billBuffer,4, 4));	//kW Date
			
			
			R_USER_GetRTCTime(&buf->apparent_TZ[max_zone_no].MD_DT,getByteValue(billBuffer,8, 4));	//kVA Date

			buf->active_energyTZ[max_zone_no]=getByteValue(billBuffer,12, 4);	//kWh
			buf->apparent_energyTZ[max_zone_no]=getByteValue(billBuffer,16, 4);	//kVAh
			
		}
	}
	
	
	
	
	return 0;
}

Unsigned16 R_USER_Class07_GetEventEntry(
	Unsigned32						event_type,
	class07_event_entry_t			*buf,
	Unsigned16						index
)
{

	Unsigned32 time_count;
	Unsigned8 tBuffer[TAMPER_DATA_LENGTH];
	if(event_type==9)
		getEventLog(tBuffer,3,	index);
	else if(event_type==10)
		getEventLog(tBuffer,0,	index);
	else if(event_type==13)
		getEventLog(tBuffer,1,	index);
	else if(event_type==14)
		getEventLog(tBuffer,2,	index);
	else if((event_type==11))
		getPfailLog(tBuffer,index);
	else
		getSWLog(tBuffer,index);
	
	time_count=getByteValue(tBuffer, 0, 4);
	
	R_USER_GetRTCTime(&buf->clock_value,time_count);
	buf->event_code=getByteValue(tBuffer, 4, 2);
	buf->ph_current_value = getByteValue(tBuffer, 6, 4);
	//buf->neu_current_value = getByteValue(tBuffer, 10, 4);
	buf->voltage_value = getByteValue(tBuffer, 14, 2);
	buf->power_factor = getByteValue(tBuffer, 16, 1);
	if(buf->power_factor&0x80)
	{
		buf->power_factor&=~0x80;
		buf->power_factor=0x100000000L-buf->power_factor;
	}	
	buf->active_energy = getByteValue(tBuffer, 17, 4);
	buf->apparent_energy = getByteValue(tBuffer, 21, 4);
	return 0;
}


Unsigned16 R_OBIS_Class07_GetInstantEntry(class07_instant_entry_t *buf)
{
	// rtc
	R_USER_GetRTCTime(&buf->clock_value,InsSave.timeCounter);
	// voltage
	buf->voltage_value = R_USER_GetEMData(EM_VOLTAGE);

	buf->PhCurr_value = R_USER_GetEMData(EM_PHASE_CURRENT);
	buf->NuCurr_value = R_USER_GetEMData(EM_NEUTRAL_CURRENT);

	buf->Pf_value= R_USER_GetEMData(EM_POWER_FACTOR);

	buf->Frq_value = R_USER_GetEMData(EM_LINE_FREQ);
	buf->PowerS_value = R_USER_GetEMData(EM_APPARENT_POWER);
	buf->PowerP_value = R_USER_GetEMData(EM_ACTIVE_POWER);
	buf->kWh_value = R_USER_GetEMData(EM_CUCUMLARIVE_ENERGY_KWH);
	buf->kVAh_value = R_USER_GetEMData(EM_CUCUMLARIVE_ENERGY_KVAH);
	buf->kW_MD_value=(Unsigned32)R_USER_GetEMData(EM_ACTIVE_MD);
	
	memcpy((Unsigned8 *)&buf->kW_MD_clock,(Unsigned8 *)&g_Class04_ActiveMDCaptureTime,12);

	buf->kVA_MD_value= R_USER_GetEMData(EM_APPARENT_MD);
	memcpy((Unsigned8 *)&buf->kVA_MD_clock,(Unsigned8 *)&g_Class04_ApparentMDCaptureTime,12);
	buf->PowerOn_value = R_USER_GetEMData(EM_CUCUMLARIVE_POWER_ON_DURATION);
	buf->TamperCount_value = (Unsigned16)R_USER_GetEMData(EM_CUCUMLARIVE_TAMPER_CNT);
	buf->BillCount_value = (Unsigned16)R_USER_GetEMData(EM_CUCUMLARIVE_BILLING_CNT);
	buf->ProgramCount_value = (Unsigned16)R_USER_GetEMData(EM_CUCUMLARIVE_PROGAM_CNT);
//	buf->ExpkWh_value = R_USER_GetEMData(EM_CUCUMLARIVE_EXP_ENERGY_KWH);
//	buf->ExpkVAh_value = R_USER_GetEMData(EM_CUCUMLARIVE_EXP_ENERGY_KVAH);
			
	return 0;
}






/******************************************************************************
* Function Name    : static uint8_t R_OBIS_Class07_AddEventEntry(Unsigned8 event_type, Unsigned16 event_code)
* Description      : Event add energy log to EEPROM,
* Arguments        : Unsigned8 event_type, Unsigned16 event_code
* Functions Called : T.B.D
* Return Value     : Unsigned8: 0: OK, else: NG
******************************************************************************/
uint8_t R_OBIS_Class07_AddEventEntry(uint8_t event_type, uint16_t event_code)
{

	return 0;
}
#endif /* #if (defined(USED_CLASS_07) && USED_CLASS_07 == 1) */

#if (defined(USED_CLASS_09) && USED_CLASS_09 == 1)
/* Class09 - Script table */
/******************************************************************************
* Function Name : R_USER_Class09_ScriptExecute
* Interface     : Unsigned8 R_USER_Class09_ScriptExecute()
* Description   : Script Execute
* Arguments     :
*               : Unsigned16 ID,
* Function Calls: T.B.D
* Return Value  : Unsigned8: 1: error; 0 : OK
******************************************************************************/
Unsigned8 R_USER_Class09_ScriptExecute(const uint8_t* bptr)
{
	//0,   0,   10,   0,   1, 255
	//0,   0,   10,   0,  100, 255
	if((bptr[0]==0)&&(bptr[1]==0)&&(bptr[2]==10)&&(bptr[3]==0)&&(bptr[5]==255))
	{
		if(bptr[4]==1)
		{
			logSWTamper(166);
			setSingleActionFlag(0,0,getSingleActionFlag(0)+1);
			mcu_flag|=SINGLE_ACTION_CHECK_FLAG;
			ScheduleActionDateTime.BillDateTime=InsSave.timeCounter;
		}
		else if(bptr[4]==100)
		{
			logSWTamper(155);
			setSingleActionFlag(1,0,getSingleActionFlag(1)+1);
			mcu_flag|=PASSIVE_CAL_CHECK_FLAG;
			ScheduleActionDateTime.CalanderDateTime=InsSave.timeCounter;
			
		}
	}
	
	return 1;
}

#endif /* #if (defined(USED_CLASS_09) && USED_CLASS_09 == 1) */

#if (defined(USED_CLASS_15) && USED_CLASS_15 == 1)
/* Class15 -  Association LN */
/******************************************************************************
* Function Name : R_USER_Class15_Secret_Backup
* Interface     : Unsigned8 R_USER_Class15_Secret_Backup()
* Description   : LLS/HLS Secret_Backup
* Arguments     : 
*	Unsigned8             *p_secret,
*	Unsigned16            length,
*	Unsigned8             mechanism_id,
* Function Calls: T.B.D
* Return Value  : Unsigned8: 0: OK, else: NG
******************************************************************************/
Unsigned8 R_USER_Class15_Secret_Backup(
	Unsigned8             *p_secret,
	Unsigned16            length,
	Unsigned8             mechanism_id
)
{
	uint16_t loc,crc;
	uint8_t status=0,secretlen=0;
	uint8_t secretBuff[DLMS_HLS_SECRET_SIZE+2];
	
	/* Check params */
	if (p_secret == NULL || length == 0 || mechanism_id == MECHANISM_ID0_LOWEST_SECURITY)
	{
		status= 1;
	}
	if (mechanism_id == MECHANISM_ID1_LOW_SECURITY)
	{
		//CLK> Pls do some check to ensure the LLS length is within EEPROM storage expected space
		if (length>DLMS_LLS_SECRET_SIZE)
		{
			status=1;
		}
		
		memcpy(secretBuff, p_secret, length);
		memset(secretBuff+length, 0,DLMS_LLS_SECRET_SIZE-length);
		memcpy(secret1, secretBuff, DLMS_LLS_SECRET_SIZE);
		
		secretlen=DLMS_LLS_SECRET_SIZE;
		loc=DLMS_LLS_SECRET_ADDR;
		logSWTamper(161);
		
	}
	else // HLS
	{
		//CLK> Pls do some check to ensure the HLS length is within EEPROM storage expected space
		if (length>DLMS_HLS_SECRET_SIZE)
		{
			status= 1;
		}

		memcpy(secretBuff, p_secret, length);
		memset(secretBuff+length, 0, DLMS_HLS_SECRET_SIZE-length);
		memcpy(secret2, secretBuff, DLMS_HLS_SECRET_SIZE);
		
		secretlen=DLMS_HLS_SECRET_SIZE;
		loc=DLMS_HLS_SECRET_ADDR;
		logSWTamper(162);
		
		
	}
	
	if(status==0)
	{
		crc = Crc_Cal(secretBuff,secretlen);
		secretBuff[secretlen]=crc;
		secretBuff[secretlen+1]=crc>>8;
		write_eeprom(secretBuff,loc,secretlen+2);
		
	}
	return status;
}

/******************************************************************************
* Function Name : R_USER_Class15_Secret_Restore
* Interface     : Unsigned8 R_USER_Class15_Secret_Restore()
* Description   : LLS/HLS Secret Restore
* Arguments     : 
*	Unsigned8             *p_secret,
*	Unsigned16            length,
*	Unsigned8             mechanism_id,
* Function Calls: T.B.D
* Return Value  : Unsigned8: 0: OK, else: NG
******************************************************************************/
Unsigned8 R_USER_Class15_Secret_Restore(
	Unsigned8             *p_secret,
	Unsigned16            length,
	Unsigned8             mechanism_id
)
{
	uint16_t loc;
	uint8_t secretBuff[DLMS_HLS_SECRET_SIZE+2];
	uint8_t status=0,secretlen=0;
	/* Check params */
	if (p_secret == NULL || length == 0 || mechanism_id == MECHANISM_ID0_LOWEST_SECURITY)
	{
		status=1;
	}
	if (mechanism_id == MECHANISM_ID1_LOW_SECURITY)
	{
		//CLK> Pls do some check to ensure the LLS length is within EEPROM storage expected space
		if (DLMS_LLS_SECRET_SIZE >length)
		{
			status=1;
		}
		secretlen=DLMS_LLS_SECRET_SIZE;
		loc=DLMS_LLS_SECRET_ADDR;
		
	}
	else // HLS
	{
		//CLK> Pls do some check to ensure the HLS length is within EEPROM storage expected space
		if (DLMS_HLS_SECRET_SIZE != length)
		{
			
			status=1;
		}
		secretlen=DLMS_HLS_SECRET_SIZE;
		loc=DLMS_HLS_SECRET_ADDR;
	}
	
	if(status==0)
	{
		read_eeprom(secretBuff,loc,secretlen+2);
		loc = Crc_Cal(secretBuff,secretlen+2);
		if(loc==0)
			memcpy(p_secret, secretBuff, secretlen);
	}
	return status; //Return 0 for OK 
}

#endif /* #if (defined(USED_CLASS_15) && USED_CLASS_15 == 1) */


#if (defined(USED_CLASS_20) && USED_CLASS_20 == 1)
/* Class 20 - Activity Calendar */


/******************************************************************************
* Function Name : R_USER_Class20_GetTariffSeasonTable
* Interface     : Unsigned8 R_USER_Class20_GetTariffSeasonTable()
* Description   : Get Season Table
* Arguments     : season_profile_t    *buf,
*				  Unsigned16          index,
*				  Unsigned16          is_active
* Function Calls: T.B.D
* Return Value  : Unsigned8: 1: NG, 0: OK
******************************************************************************/
Unsigned8 R_USER_Class20_GetTariffSeasonTable(
	season_profile_t    *buf,
	Unsigned16          index,
	Unsigned16          is_active
)
{
	Apprtc rtc;
	uint16_t season_name_loc[]={TOD_PCALANDER_NAME,TOD_CALANDER_NAME};
	uint16_t season_time_loc[]={PASSIVE_ZONE_SEASON,ZONE_SEASON};
	
	uint16_t loc;
	uint8_t  season_len = sizeof(g_Class20_season_name1);
	uint8_t  week_len = sizeof(g_Class20_week_name1);
	

	loc=season_name_loc[is_active]+8;
	read_eeprom(buf[0].p_season_profile_name,loc, season_len);
	loc=loc+8;
	read_eeprom(buf[0].p_week_profile_name, loc, week_len);
	loc=loc+8;
	
	loc=loc+7;
	season_len = sizeof(g_Class20_season_name2);
	week_len = sizeof(g_Class20_week_name2);
	read_eeprom(buf[1].p_season_profile_name, loc, season_len);
	loc=loc+8;
	read_eeprom(buf[1].p_week_profile_name, loc, week_len);
	
	loc=season_time_loc[is_active];
	
	ConvertTimeCounterToTime(InsSave.timeCounter,TIME_VAL,&rtc);
	read_eeprom((uint8_t*)&rtc.month,loc+1,1);
	rtc.day= getAutoBillDay();
	

	rtc.hour= 0;
	rtc.min= 0;
	rtc.sec= 0;
	
	R_USER_GetRTCTime(buf[0].p_season_start,ConvertTimeToTimeCounter(&rtc));	

	loc=loc+2;
	
	ConvertTimeCounterToTime(InsSave.timeCounter,TIME_VAL,&rtc);
	read_eeprom((uint8_t*)&rtc.month,loc+1,1);
	rtc.day= getAutoBillDay();

	rtc.hour= 0;
	rtc.min= 0;
	rtc.sec= 0;
	R_USER_GetRTCTime(buf[1].p_season_start,ConvertTimeToTimeCounter(&rtc));	
	
	return 0;
}

/******************************************************************************
* Function Name : R_USER_Class20_GetTariffWeekTable
* Interface     : Unsigned8 R_USER_Class20_GetTariffWeekTable()
* Description   : Get Week Table
* Arguments     : week_profile_t     *buf,
*				  Unsigned16          index,
*				  Unsigned16          is_active
* Function Calls: T.B.D
* Return Value  : Unsigned8: 1: NG, 0: OK
******************************************************************************/
Unsigned8 R_USER_Class20_GetTariffWeekTable(
	week_profile_t      *buf,
	Unsigned16          index,
	Unsigned16          is_active
)
{
	uint16_t week_loc[]={TOD_PCALANDER_NAME+16,TOD_CALANDER_NAME+16};
	uint8_t  name_len = sizeof(buf->p_week_profile_name);
	uint8_t  weekday_len = 7;
	uint16_t ee_addr = week_loc[is_active] + (index * (name_len + weekday_len));
	
	read_eeprom(buf->p_week_profile_name, ee_addr, name_len);
	
	return 0;
}

/******************************************************************************
* Function Name : R_USER_Class20_GetTariffDayTable
* Interface     : Unsigned8 R_USER_Class20_GetTariffDayTable()
* Description   : Get Day Table
* Arguments     : day_profile_t      *buf,
*				  Unsigned16          index,
*				  Unsigned16          is_active
* Function Calls: T.B.D
* Return Value  : Unsigned8: 1: NG, 0: OK
******************************************************************************/
Unsigned8 R_USER_Class20_GetTariffDayTable(
	day_profile_t       *buf,
	Unsigned16          index,
	Unsigned16          is_active
)
{

	uint16_t day_loc[]={PASSIVE_ZONE_TIME,ZONE_TIME};
	uint16_t loc;
	uint8_t i,max_zone_no,daySlotno;
	day_profile_action_t *bptr;
	uint32_t time_count = 0;
	uint8_t day_profile_no=0;

	while(day_profile_no<2)
	{
		max_zone_no=getMaxZoneNo(day_profile_no);
		loc=day_loc[is_active]+day_profile_no*25;
		if(is_active==0)
			max_zone_no=8;
		
		bptr=buf[day_profile_no].day_schedule;
		for(i=0;i<max_zone_no;i++)
		{
			read_eeprom((uint8_t*)&time_count,loc+i*2,2);
			read_eeprom((uint8_t*)&daySlotno,loc+17+i,1);
			
			
			bptr->p_start_time->hour=time_count/60;
			bptr->p_start_time->minute=time_count%60;
			bptr->p_start_time->second=0x00;
			bptr->script_selector=daySlotno;
			bptr++;
		}

		buf[day_profile_no].nr_day_schedule=max_zone_no;
		day_profile_no++;
	}

	return 0;
}

#if (defined(USED_CLASS_20_SEASON_PROFILE) && USED_CLASS_20_SEASON_PROFILE == 1)
/******************************************************************************
* Function Name : R_USER_Class20_ChangSeason
* Interface     : void R_USER_Class20_ChangSeason()
* Description   : Notify to EM about change season
* Arguments     :
*				  date_time_t current_time,
* Function Calls: T.B.D
* Return Value  : None
******************************************************************************/
void R_USER_Class20_ChangSeason(const season_profile_t * buff)
{
	uint16_t SeaTime_loc[]={PASSIVE_ZONE_SEASON};
	uint16_t SeaName_loc[]={TOD_PCALANDER_NAME+8};

	uint8_t zBuffer[8];
	const season_profile_t *ptr=buff;
	
	zBuffer[0]=ptr->p_season_start->day_of_month;
	zBuffer[1]=ptr->p_season_start->month;
	zBuffer[2]=(ptr+1)->p_season_start->day_of_month;
	zBuffer[3]=(ptr+1)->p_season_start->month;
	
	zBuffer[4]=2;
	zBuffer[5]=1;
	zBuffer[6]=2;
	write_eeprom(zBuffer,SeaTime_loc[0],7);
	write_eeprom(buff->p_season_profile_name,SeaName_loc[0],buff->season_profile_name_len %9);
	write_eeprom(buff->p_week_profile_name,SeaName_loc[0],buff->week_profile_name_len %9);
	return;
}
#endif /* #if (defined(USED_CLASS_20_SEASON_PROFILE) && USED_CLASS_20_SEASON_PROFILE == 1) */
#if (defined(USED_CLASS_20_WEEK_PROFILE) && USED_CLASS_20_WEEK_PROFILE == 1)
/******************************************************************************
* Function Name : R_USER_Class20_ChangeWeekDay
* Interface     : void R_USER_Class20_ChangeWeekDay()
* Description   : Notify to EM about change WeekDay
* Arguments     :
*				  date_time_t current_time,
* Function Calls: T.B.D
* Return Value  : None
******************************************************************************/
void R_USER_Class20_ChangeWeekDay(date_time_t current_time)
{
	/* Option: User code START */
	/* Option: User code END */
	return;
}

Unsigned8 R_USER_Class20_SetTariffDayTablePassive(const day_profile_t  *buf)
{
	uint16_t day_loc[]={PASSIVE_ZONE_TIME};
	uint8_t i,max_zone_no=0;
	day_profile_action_t *bptr;
	uint32_t time_count = 0;
	uint8_t zBuffer[26]={0};
	uint8_t day_profile_no=0;
	
	if(buf->day_id>1)
		day_profile_no=1;
	
	bptr=buf[0].day_schedule;
	for(i=0;i<8;i++)
	{
		
		if(bptr->script_selector==0)
		{
			time_count=0x00;
			
			zBuffer[i*2]=time_count;
			zBuffer[i*2+1]=time_count>>8;
			zBuffer[17+i]=0x00;
		
		}
		else
		{
			max_zone_no++;
			
			time_count=bptr->p_start_time->hour*60+bptr->p_start_time->minute;
			
			if((time_count%(60/md_period))==0)
			{
				zBuffer[i*2]=time_count;
				zBuffer[i*2+1]=time_count>>8;
				zBuffer[17+i]=bptr->script_selector;
			}
			else
				return 0;
			
		}
		bptr++;
	}
		
	zBuffer[16]=max_zone_no;
	write_eeprom(zBuffer,day_loc[0]+day_profile_no*25,25);
	
	return 1;
}
#endif /* #if (defined(USED_CLASS_20_WEEK_PROFILE) && USED_CLASS_20_WEEK_PROFILE == 1) */
Unsigned8 R_USER_Class20_SETACTION_Time(Unsigned32 date_time,Unsigned8 actionType)
{	
	
	
	if((date_time>=InsSave.timeCounter)||(actionType==2)||(actionType==3))
	{
			
		if(actionType==0)
		{
			logSWTamper(155);
			setSingleActionFlag(1,date_time,0xAB);
		}
		else
		{
			
	    	
			if(actionType==2)
			{
				write_eeprom((uint8_t *)&date_time ,PRE_BILL_DATE,1);
				logSWTamper(254);
				return 0;
			}
			else
			{
				if(actionType==3)
				{
					date_time=InsSave.timeCounter;
					logSWTamper(166);
				}
				else
					logSWTamper(154);
				setSingleActionFlag(0,date_time,0xAB);
				
			}
			
		}
		
		R_OBIS_Class07_EventUpdate();
		
	}
	return 0;
}

#endif /* #if (defined(USED_CLASS_20) && USED_CLASS_20 == 1) */
