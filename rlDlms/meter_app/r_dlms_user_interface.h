/******************************************************************************
  Copyright (C) 2016 Renesas Electronics Corporation, All Rights Reserved.
*******************************************************************************
* File Name    : r_dlms_user_interface.h
* Version      : 1.00
* Description  :
******************************************************************************
* History : DD.MM.YYYY Version Description
*         : 01.03.2016
******************************************************************************/

#ifndef _R_DLMS_USER_INTERFACE_H
#define _R_DLMS_USER_INTERFACE_H

/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#include "..\\r_dlms_typedef.h"				/* DLMS Type Definitions */
#include "..\\Objects\\r_dlms_obis.h"				/* DLMS OBIS Definitions */
#include "..\\DLMS_User.h"					/* DLMS User Definitions */

//---------------Application specific-------------
#include "..\\..\\rlApplication\\Include\\AppVariables.h"



/******************************************************************************
Macro definitions
******************************************************************************/
/* Common macro */

/************************************************/
/* Storage information for User interface data  */
/************************************************/
/* Wrapper defination for ouside memory access */

/* ID For Name plate data */
#define EM_NAME_PLATE_LOGICAL_NAME			(0x01)
#define EM_NAME_PLATE_METER_SERIAL			(0x02)
#define EM_NAME_PLATE_MANUFACT_NAME			(0x03)
#define EM_NAME_PLATE_FIRMWARE_NAME			(0x04)
#define EM_NAME_PLATE_METER_TYPE			(0x05)
#define EM_NAME_PLATE_CATEGORY_NAME			(0x06)
#define EM_NAME_PLATE_CURRRATE_NAME			(0x07)
#define EM_NAME_PLATE_YEAR_OF_MANUFACT		(0x08)

#define EM_BLOCKLOAD_PERIOD					(0x72)
/* ID For Time */
#define EM_CURRENT_TIME						(0x11)
#define EM_LASTBILLING_TIME					(0x12)
#define EM_LAST_ACTIVE_MD_TIME				(0x13)
#define EM_LAST_APPARENT_MD_TIME			(0x14)


/* ID For Script Execute */
#define EM_SCRIPT_ACTION_TZ1				(0x01)
#define EM_SCRIPT_ACTION_TZ2				(0x02)
#define EM_SCRIPT_ACTION_TZ3				(0x03)
#define EM_SCRIPT_ACTION_TZ4				(0x04)
/******************************************************************************
Typedef definitions
******************************************************************************/
/* Same as RTC Date Time */
typedef struct tagDLMS_RTCDateTime
{
	/* Total: 8 bytes */
	Unsigned8	Sec;		/* Second */
	Unsigned8	Min;		/* Minute */
	Unsigned8	Hour;		/* Hour */
	Unsigned8	Day;		/* Day */
	Unsigned8	Week;		/* Day of week */
	Unsigned8	Month;		/* Month */
	Unsigned8	Year;		/* Year (ony 2 ending digit) */
	
	Unsigned8	reserved;	/* Reserved Byte (Padding) - NO USE */
	
} DLMS_RTC_DATE_TIME;

/******************************************************************************
Variable Externs
******************************************************************************/

/******************************************************************************
Functions Prototypes
******************************************************************************/
/* I/F to Time Access */
Unsigned8 R_USER_GetSpecificTime(Unsigned16 time_id, date_time_t *p_date_time);
Unsigned8 R_USER_SetSpecificTime(Unsigned16 time_id, date_time_t date_time);

/* I/F to WDT */
void R_USER_WDT_Restart(void);

/* I/F to DLMS name plate */
Integer16 R_USER_GetNamePlate(Unsigned16 nameplate_id, Unsigned8* p_data);

/* I/F to EM APIs*/
Integer32 R_USER_GetEMData(Unsigned16 em_data);
Integer8 R_USER_SetEMData(Unsigned16 em_data, Unsigned8 *p_em_value);

/* I/F to Event notification */
void R_USER_Event_Report(Unsigned8 event_type, Unsigned16 event_code);

void R_OBIS_Update_Secret(void);

#if (defined(USED_CLASS_04) && USED_CLASS_04 == 1)
/* Class04 - Extended Register */
Unsigned8 R_OBIS_Class04_GetActiveMD(
	Integer32           *p_value,
	date_time_t         *p_time
);

Unsigned8 R_OBIS_Class04_GetApparentMD(
	Integer32           *p_value,
	date_time_t         *p_time
);
#endif



#if (defined(USED_CLASS_09) && USED_CLASS_09 == 1)
/* Class09 - Script table */
Unsigned8 R_USER_Class09_ScriptExecute(const uint8_t* bptr);

#endif /* #if (defined(USED_CLASS_09) && USED_CLASS_09 == 1) */

#if (defined(USED_CLASS_15) && USED_CLASS_15 == 1)
/* Class15 -  Association LN */
Unsigned8 R_USER_Class15_Secret_Backup(
	Unsigned8             *p_secret,
	Unsigned16            length,
	Unsigned8             mechanism_id
);

Unsigned8 R_USER_Class15_Secret_Restore(
	Unsigned8             *p_secret,
	Unsigned16            length,
	Unsigned8             mechanism_id
);

#endif /* #if (defined(USED_CLASS_15) && USED_CLASS_15 == 1) */

#if (defined(USED_CLASS_20) && USED_CLASS_20 == 1)
/* Class20 - Activity Calendar */
Unsigned8 R_USER_Class20_SETACTION_Time(Unsigned32 timeCounter,Unsigned8 actionType);
Unsigned8 R_USER_Class20_GetTariffSeasonTable(
	season_profile_t    *buf,
	Unsigned16          index,
	Unsigned16          is_active
);
Unsigned8 R_USER_Class20_GetTariffWeekTable(
	week_profile_t      *buf,
	Unsigned16          index,
	Unsigned16          is_active
);
Unsigned8 R_USER_Class20_GetTariffDayTable(
	day_profile_t       *buf,
	Unsigned16          index,
	Unsigned16          is_active
);
Unsigned8 R_USER_Class20_SetTariffDayTablePassive(const day_profile_t  *buf);
Unsigned8 R_USER_Class20_Activate_PassiveCalendar(class20_child_record_t *p_child_record_20);

#if (defined(USED_CLASS_20_SEASON_PROFILE) && USED_CLASS_20_SEASON_PROFILE == 1)
void R_USER_Class20_ChangSeason(const season_profile_t * buff);
#endif /* #if (defined(USED_CLASS_20_SEASON_PROFILE) && USED_CLASS_20_SEASON_PROFILE == 1) */

#if (defined(USED_CLASS_20_WEEK_PROFILE) && USED_CLASS_20_WEEK_PROFILE == 1)
void R_USER_Class20_ChangeWeekDay(date_time_t current_time);

#endif /* #if (defined(USED_CLASS_20_WEEK_PROFILE) && USED_CLASS_20_WEEK_PROFILE == 1) */

#endif /* #if (defined(USED_CLASS_20) && USED_CLASS_20 == 1) */
void R_USER_Set_RTC_timezone(void);
uint32_t R_USER_ConvertrDLMStoAppRTC(const date_time_t *p_date_time,Apprtc * rtc);
#endif /* _R_DLMS_USER_INTERFACE_H */

