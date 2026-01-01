/******************************************************************************
* DISCLAIMER

* This software is supplied by Renesas Electronics Corporation and is only
* intended for use with Renesas products. No other uses are authorized.

* This software is owned by Renesas Electronics Corporation and is protected under
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
/* Copyright (C) 2013 Renesas Electronics Corporation. All rights reserved.  */
/******************************************************************************
* File Name    : r_dlms_data_meter.c
* Version      : 1.00
* Device(s)    : None
* Tool-Chain   :
* H/W Platform : Unified Energy Meter Platform
* Description  :
******************************************************************************
* History : DD.MM.YYYY Version Description
*         : 18.06.2013
******************************************************************************/

/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/

/* DLMS */
#include "..\\r_dlms_typedef.h"				/* DLMS Type Definitions */
#include "r_dlms_data.h"				/* DLMS Data Definitions */
#include "r_dlms_data_meter.h"				/* DLMS Data Definitions */
#include "..\\Objects\\r_dlms_obis.h"				/* DLMS OBIS Definitions */
#include "..\\Objects\\r_dlms_obis_ic.h"				/* DLMS OBIS IC Definitions */
#include "r_dlms_data_ic.h"
#include "..\\DLMS_User.h"					/* DLMS User Definitions */
#include "r_dlms_user_interface.h"		/* DLMS User Interface Definitions */
/* Standard Lib */
#include <string.h>						/* String Standard Lib */

//---------------Device specific-------------

#include "..\\..\\rlDevice\\include\\dI2c.h"


//---------------Application specific-------------
#include "..\\..\\rlApplication\\Include\\AppVariables.h"
#include "..\\..\\rlApplication\\Include\\AppBlockDailyLS.h"
#include "..\\..\\rlApplication\\Include\\AppMacros.h"
#include "..\\..\\rlApplication\\Include\\AppTampers.h"
#include "..\\..\\rlApplication\\Include\\AppScheduleAction.h"
#include "..\\..\\rlApplication\\Include\\AppLCD.h"
#include "..\\..\\rlApplication\\Include\\AppMisc.h"
#include "..\\..\\rlApplication\\Include\\AppBilling.h"
#include "..\\..\\rlApplication\\Include\\AppEEprom.h"
#include "..\\..\\rlApplication\\Include\\AppConfig.h"


///* EM Core Component */
//#include "em_operation.h"	/* EM Core Operation APIs */
//#include "em_calibration.h"	/* EM Calibration APIs */
//#include "em_measurement.h"	/* EM Measurement APIs */

/******************************************************************************
Macro definitions
******************************************************************************/
//USER VARIABLE
extern  Unsigned8 secret1[];
extern Unsigned8 secret2[];
extern const execution_time_date_t g_Class22_bill_exetime[];
extern date_time_t				g_Class20_ac_time;
extern int pf_reg4, kw_reg4;
extern unsigned long int all_tamper_cnt;//meter_no,  //USER
extern unsigned long int load_val[2], disp_load_val[2], last_hr_load_val[2];
extern unsigned long int power_on_time,cum_on_time;
extern unsigned int avg_pf;
//extern unsigned char string[10]; //USER
extern unsigned int md_intgr_val, survey_intgr_val; //USER
extern unsigned long int md_reset_cnt;
extern unsigned long int kwmd_date,kvamd_date;
extern date_time_t tmp_time_string;
extern unsigned int kwmd_val,kvamd_val;
extern unsigned int kwmd_time,kvamd_time;
extern unsigned long int demand_volt;
extern unsigned char billing_day;
//extern unsigned char md_reset_b;
/******************************************************************************
Typedef definitions
******************************************************************************/


/******************************************************************************
Imported global variables and functions (from other files)

******************************************************************************/
#if (defined(USED_CLASS_01) && USED_CLASS_01 == 1)
extern Unsigned16		g_Class01_EventCode[7];// = {69,101,121,201,251,0,0};
#endif

extern Unsigned32		g_Class01_Blockload_interval;// = CLASS07_BLOCKLOAD_INTERVAL;
/*
 * Imported from r_dlms_data_ic.c
 * For data access
 *
 * All of following variables are linked into each specified-child-table.
 * Refer to each child-table in r_dlms_data_ic.c
 */
/* Class ID = 01 - Data Class */


/* Class ID = 03 - Register Class */
#if (defined(USED_CLASS_03) && USED_CLASS_03 == 1)
/* Block load profile parameter */
/* Billing profile parameter */
extern date_time_t		g_Class03_BillingDate;
#endif

/* Class ID = 04 - Extended Register Class */
#if (defined(USED_CLASS_04) && USED_CLASS_04 == 1)
/* Block load profile parameter */
extern date_time_t			g_Class04_ActiveMDCaptureTime;
extern date_time_t			g_Class04_ApparentMDCaptureTime;
#endif

extern Unsigned8					g_Class07_Obj0_Capture_Enable;
extern Unsigned32				g_Class07_Blockload_CapturePeriod;
extern Unsigned32					g_Class07_Blockload_EntriesInUse;
extern Unsigned16					g_Class07_Blockload_CurrentEntry;

extern const Unsigned32				g_Class07_Dailyload_CapturePeriod;
extern Unsigned32					g_Class07_Dailyload_EntriesInUse;
extern Unsigned16					g_Class07_Dailyload_CurrentEntry;

extern Unsigned32					g_Class07_Billing_EntriesInUse;
extern Unsigned16					g_Class07_Billing_CurrentEntry;

extern Unsigned32					g_Class07_Event_EntriesInUse[];
extern Unsigned32					g_Class07_Event_MaxEntries[];
extern Unsigned16					g_Class07_Event_CurrentEntry[];


/* Class ID = 07 - Profile generic class */
#if (defined(USED_CLASS_07) && USED_CLASS_07 == 1)
extern class07_blockload_entry_t	g_Class07_BlockLoadBuffer;

extern class07_dailyload_entry_t	g_Class07_DailyLoadBuffer;

extern class07_billing_entry_t		g_Class07_BillingBuffer;

extern class07_event_entry_t		g_Class07_EventBuffer;//[CLASS07_EVENT_MAX_PROFILE];//[CLASS07_EVENT_MAX_ENTRY];

extern const class07_capture_object_t 	g_Class07_BlockloadCaptureObjects[];
extern const class01_child_record_t 	g_ChildTableClass01[];
extern const class03_child_record_t 	g_ChildTableClass03[];
extern const class04_child_record_t 	g_ChildTableClass04[];
extern const class07_child_record_t 	g_ChildTableClass07[];
extern const Unsigned16 	g_Class07_BlockloadCaptureObjectsLength;
extern const Unsigned16 	g_ChildTableLengthClass01;
extern const Unsigned16 	g_ChildTableLengthClass03;
extern const Unsigned16 	g_ChildTableLengthClass04;

extern class07_bufferUnion_t	class07_bufferUnion;
#endif

/* Class ID = 15, Association LN Child-table */
#if (defined(USED_CLASS_15) && USED_CLASS_15 == 1)
extern const class15_child_record_t g_ChildTableClass15[];  /* Association LN List      */
extern const Unsigned16 g_ChildTableLengthClass15;  		/* Association LN Length    */
#endif

/* Class ID = 64 - Security setup class */
#if (defined(USED_CLASS_64) && USED_CLASS_64 == 1)
extern const class64_child_record_t g_ChildTableClass64[];  /* Security setup reference list   */
extern const Unsigned16 g_ChildTableLengthClass64;  		/* Security setup reference length */
#endif
/******************************************************************************
Exported global variables and functions (to be accessed by other files)
******************************************************************************/

/******************************************************************************
Private global variables and functions
******************************************************************************/
/* RCT current time */
/* Stick counter in second */
static Unsigned32 rtc_stick = 0;

/******************************************************************************
* Function Name : R_OBIS_IsValidDateTime
* Interface     : Unsigned8 R_OBIS_IsValidDateTime(
*               :     Unsigned8 *p_date_time,
*               : );
* Description   : Check valid date time or not
* Arguments     : Unsigned8 *p_date_time
* Function Calls: None
* Return Value  : Unsigned8, 0: invalid, 1 : valid
******************************************************************************/
Unsigned8 R_OBIS_IsValidDateTime(Unsigned8 *p_date_time)
{
	Unsigned8 temp;

	if(*p_date_time++ != ATTR_TYPE_OCTET_STRING)
	{
		return 0;
	}
	if(*p_date_time++ != sizeof(date_time_t))
	{
		return 0;
	}
	p_date_time++; // Ignore year highbyte
	p_date_time++; // Ignore year lowbyte

	temp = *p_date_time++;
	if(temp > 12 && temp < DATE_MONTH_DAYLIGHT_SAVINGS_END) // Month
	{
		return 0;
	}
	temp = *p_date_time++;
	if(temp > 31 && temp < DATE_DAY_2ND_LAST_OF_MONTH) // day of month
	{
		return 0;
	}
	temp = *p_date_time++;
	if(temp > 7 && temp < NOT_SPECIFIED) // day of week
	{
		return 0;
	}
	temp = *p_date_time++;
	if(temp > 23 && temp < NOT_SPECIFIED) // hour
	{
		return 0;
	}
	temp = *p_date_time++;
	if(temp > 59 && temp < NOT_SPECIFIED) // minute
	{
		return 0;
	}
	temp = *p_date_time++;
	if(temp > 59 && temp < NOT_SPECIFIED) // second
	{
		return 0;
	}
	temp = *p_date_time++;
	if(temp > 99 && temp < NOT_SPECIFIED) // hundred
	{
		return 0;
	}
	return 1;
}

/******************************************************************************
* Function Name : R_OBIS_IsValidDate
* Interface     : Unsigned8 R_OBIS_IsValidDate(
*               :     Unsigned8 *p_date,
*               : );
* Description   : Check valid date time or not
* Arguments     : Unsigned8 *p_date
* Function Calls: None
* Return Value  : Unsigned8, 0: invalid, 1 : valid
******************************************************************************/
Unsigned8 R_OBIS_IsValidDate(Unsigned8 *p_date)
{
	Unsigned8 temp;

	if(*p_date++ != ATTR_TYPE_OCTET_STRING)
	{
		return 0;
	}
	if(*p_date++ != 5)
	{
		return 0;
	}
	p_date++; // Ignore year highbyte
	p_date++; // Ignore year lowbyte

	temp = *p_date++;
	if(temp > 12 && temp < DATE_MONTH_DAYLIGHT_SAVINGS_END) // Month
	{
		return 0;
	}
	temp = *p_date++;
	if(temp > 31 && temp < DATE_DAY_2ND_LAST_OF_MONTH) // day of month
	{
		return 0;
	}
	temp = *p_date++;
	//if((temp > 7 && temp < NOT_SPECIFIED)||(temp == 0)) // day of week
	//{
	//	return 0;
	//}

	return 1;
}

/******************************************************************************
* Function Name : R_OBIS_IsValidTime
* Interface     : Unsigned8 R_OBIS_IsValidTime(
*               :     Unsigned8 *p_time,
*               : );
* Description   : Check valid date time or not
* Arguments     : Unsigned8 *p_time
* Function Calls: None
* Return Value  : Unsigned8, 0: invalid, 1 : valid
******************************************************************************/
Unsigned8 R_OBIS_IsValidTime(Unsigned8 *p_time)
{
	Unsigned8 temp;

	if(*p_time++ != ATTR_TYPE_OCTET_STRING)
	{
		return 0;
	}
	if(*p_time++ != 4)
	{
		return 0;
	}

	temp = *p_time++;
	if(temp > 23 && temp < NOT_SPECIFIED) // hour
	{
		return 0;
	}
	temp = *p_time++;
	if(temp > 59 && temp < NOT_SPECIFIED) // minute
	{
		return 0;
	}
	temp = *p_time++;
	if(temp > 59 && temp < NOT_SPECIFIED) // second
	{
		return 0;
	}
	temp = *p_time++;
	if(temp > 99 && temp < NOT_SPECIFIED) // hundred
	{
		return 0;
	}
	return 1;
}

/******************************************************************************
* Function Name : R_OBIS_CompareDateTime
* Interface     : Integer8 R_OBIS_CompareDateTime(
*               :     Unsigned8 *p_src_date_time,
*               :     Unsigned8 *p_des_date_time
*               : );
* Description   : Compare date time
* Arguments     : Unsigned8 * p_src_date_time: Src date
*               : Unsigned8 * p_des_date_time: Des date
* Function Calls: None
* Return Value  : Integer8, -1: src < des, 1: src > des, 0: src = des
******************************************************************************/
Integer8 R_OBIS_CompareDateTime(
	Unsigned8 *p_src_date_time,
	Unsigned8 *p_des_date_time
)
{
	Unsigned8	i;
	Integer8	result = 1;

	/*
	 * Checking following:
	 * year_high, year_low, month, day_of_month
	 * hour, minute, second, hundredths of second
	 */
	for (i = 0; i < 9; i++)
	{
		/* skip day_of_week */
		if (i == 4)
		{
			continue;
		}

		if (p_src_date_time[i] != NOT_SPECIFIED &&
			p_des_date_time[i] != NOT_SPECIFIED)
		{
			if (p_src_date_time[i] < p_des_date_time[i])
			{
				return -1;
			}
			else if (p_src_date_time[i] > p_des_date_time[i])
			{
				return 1;
			}
			result = 0;
		}
	}

	/* all equal */
	return result;
}



#if (defined(USED_CLASS_07) && USED_CLASS_07 == 1)
/******************************************************************************
* Function Name : R_OBIS_Class07_FilterOneScalerEntry
* Interface     : Unsigned8 R_OBIS_Class07_FilterOneScalerEntry(
*               :     buffer_t            *buf,
*               :     Unsigned16          index,
*               :     Unsigned16          scaler_nr,
*               :     Unsigned8           *p_out_buf,
*               :     Unsigned16          *p_out_len
*               : );
* Description   : Filter 1 entry of buffer
* Arguments     : buffer_t            *buf;							[In] Pointer to buffer_t
*               : Unsigned16          index,						[In] Entry index in buffer
*               : Unsigned16          scaler_nr,					[In] Number of scaler unit
*               : Unsigned8           *p_out_buf,					[Out] Data pointer for get
*               : Unsigned16          *p_out_len 					[Out] Data length pointer for get
* Function Calls: TBD
* Return Value  : Unsigned16 actual encode length
******************************************************************************/
Unsigned16 R_OBIS_Class07_FilterOneScalerEntry (
	buffer_t            *buf,
	Unsigned16          index,
	Unsigned16          scaler_nr,
	Unsigned8           *p_out_buf,
	Unsigned16          *p_out_len
)
{
	Unsigned16 org_len, i;
	Unsigned16 length;
	scaler_unit_t *p_scaler_unit = NULL;

	org_len = *p_out_len;
	p_scaler_unit = (scaler_unit_t *)buf->p_buffer;
	/* Encode each element of entry */
	*p_out_buf++ = ATTR_TYPE_STRUCTURE;
	*p_out_buf++ = (Unsigned8)(buf->to_value - buf->from_value + 1);
	*p_out_len  += 2;

	for (i = 0; i < scaler_nr; i++)
	{
		if (buf->from_value <= (i+1) && (i+1) <= buf->to_value)
		{
			length = R_OBIS_EncodeScalerUnit(
				p_out_buf,								/* Output buffer */
				OBIS_SERVICE_DATA_BUFFER_LENGTH,		/* Max length */
				&p_scaler_unit[i]						/* Point to the scaler_unit value */
				);

			*p_out_len += length;
			p_out_buf  += length;
		}
	}

	return (*p_out_len - org_len);
}




/******************************************************************************
* Function Name : R_OBIS_Class07_FilterOneNamePlateEntry
* Interface     : Unsigned8 R_OBIS_Class07_FilterOneNamePlateEntry(
*               :     buffer_t            *buf,
*               :     Unsigned16          index,
*               :     Unsigned8           *p_out_buf,
*               :     Unsigned16          *p_out_len
*               : );
* Description   : Filter 1 entry of buffer
* Arguments     : buffer_t            *buf;							[In] Pointer to buffer_t
*               : Unsigned16          index,						[In] Entry index in buffer
*               : Unsigned8           *p_out_buf,					[Out] Data pointer for get
*               : Unsigned16          *p_out_len 					[Out] Data length pointer for get
* Function Calls: TBD
* Return Value  : Unsigned16 actual encode length
******************************************************************************/
Unsigned16 R_OBIS_Class07_FilterOneNamePlateEntry (
	buffer_t            *buf,
	Unsigned16          index,
	Unsigned8           *p_out_buf,
	Unsigned16          *p_out_len
)
{
	Unsigned16 org_len;
	Unsigned16 length;
//	Integer32 temp;

	org_len = *p_out_len;
	/* Encode each element of entry */
	*p_out_buf++ = ATTR_TYPE_STRUCTURE;
	*p_out_buf++ = (Unsigned8)(buf->to_value - buf->from_value + 1);
	*p_out_len  += 2;
	{
	//	date_time_t current_time;
		Unsigned8 buffer[30];
		Integer16   size;
		void        *pbuffer;

		// Unsigned8 rlt = R_USER_GetSpecificTime(EM_CURRENT_TIME, &current_time);
		// if (rlt != 0)
		// {
			// return 0;
		// }

		/* 1. Meter serial number */
		if (buf->from_value <= 1 && 1 <= buf->to_value)
		{
			memset(buffer, 0, 16);
			pbuffer = buffer;
			size =  R_USER_GetNamePlate(EM_NAME_PLATE_METER_SERIAL, pbuffer);
			if (size < 0)
			{
				return 0;
			}

			length = R_OBIS_EncodeAttribute(
				p_out_buf,									/* Output buffer, pointed to g_ServiceBuffer */
				OBIS_SERVICE_DATA_BUFFER_LENGTH,			/* Max length of g_ServiceBuffer */
				ATTR_TYPE_VISIBLE_STRING,						/* Type */
				(Unsigned8 *)pbuffer,						/* Buffer */
				(Unsigned16)size							/* Length */
			);
			*p_out_len += length;
			p_out_buf  += length;
		}

		/* 2. Manufacturer name */
		if (buf->from_value <= 2 && 2 <= buf->to_value)
		{
			memset(buffer, 0, 30);
			pbuffer = buffer;
			size =  R_USER_GetNamePlate(EM_NAME_PLATE_MANUFACT_NAME, pbuffer);
			if (size < 0)
			{
				return 0;
			}

			length = R_OBIS_EncodeAttribute(
				p_out_buf,									/* Output buffer, pointed to g_ServiceBuffer */
				OBIS_SERVICE_DATA_BUFFER_LENGTH,			/* Max length of g_ServiceBuffer */
				ATTR_TYPE_VISIBLE_STRING,						/* Type */
				(Unsigned8 *)pbuffer,						/* Buffer */
				(Unsigned16)size							/* Length */
			);
			*p_out_len += length;
			p_out_buf  += length;
		}

		/* 3. Firmware version for meter */
		if (buf->from_value <= 3 && 3 <= buf->to_value)
		{
			memset(buffer, 0, 16);
			pbuffer = buffer;
			size =  R_USER_GetNamePlate(EM_NAME_PLATE_FIRMWARE_NAME, pbuffer);
			if (size < 0)
			{
				return 0;
			}

			length = R_OBIS_EncodeAttribute(
				p_out_buf,									/* Output buffer, pointed to g_ServiceBuffer */
				OBIS_SERVICE_DATA_BUFFER_LENGTH,			/* Max length of g_ServiceBuffer */
				ATTR_TYPE_VISIBLE_STRING,						/* Type */
				(Unsigned8 *)pbuffer,						/* Buffer */
				(Unsigned16)size							/* Length */
			);
			*p_out_len += length;
			p_out_buf  += length;
		}

		/* 4. Meter type */
		if (buf->from_value <= 4 && 4 <= buf->to_value)
		{
			memset(buffer, 0, 16);
			pbuffer = buffer;
			size =  R_USER_GetNamePlate(EM_NAME_PLATE_METER_TYPE, pbuffer);
			if (size < 0)
			{
				return 0;
			}

			length = R_OBIS_EncodeAttribute(
				p_out_buf,						/* Output buffer, pointed to g_ServiceBuffer */
				OBIS_SERVICE_DATA_BUFFER_LENGTH,			/* Max length of g_ServiceBuffer */
				ATTR_TYPE_LONG_UNSIGNED,				/* Type */
				(Unsigned8 *)pbuffer,					/* Buffer */
				(Unsigned16)size					/* Length */
			);
			*p_out_len += length;
			p_out_buf  += length;
		}
		
		/* 5. Category */
		if (buf->from_value <= 5 && 5 <= buf->to_value)
		{
			memset(buffer, 0, 16);
			pbuffer = buffer;
			size =  R_USER_GetNamePlate(EM_NAME_PLATE_CATEGORY_NAME, pbuffer);
			if (size < 0)
			{
				return 0;
			}

			length = R_OBIS_EncodeAttribute(
				p_out_buf,									/* Output buffer, pointed to g_ServiceBuffer */
				OBIS_SERVICE_DATA_BUFFER_LENGTH,			/* Max length of g_ServiceBuffer */
				ATTR_TYPE_VISIBLE_STRING,						/* Type */
				(Unsigned8 *)pbuffer,						/* Buffer */
				(Unsigned16)size							/* Length */
			);
			*p_out_len += length;
			p_out_buf  += length;
		}

		/* 6. Current rating */
		if (buf->from_value <= 6 && 6 <= buf->to_value)
		{
			memset(buffer, 0, 16);
			pbuffer = buffer;
			size =  R_USER_GetNamePlate(EM_NAME_PLATE_CURRRATE_NAME, pbuffer);
			if (size < 0)
			{
				return 0;
			}

			length = R_OBIS_EncodeAttribute(
				p_out_buf,									/* Output buffer, pointed to g_ServiceBuffer */
				OBIS_SERVICE_DATA_BUFFER_LENGTH,			/* Max length of g_ServiceBuffer */
				ATTR_TYPE_VISIBLE_STRING,						/* Type */
				(Unsigned8 *)pbuffer,						/* Buffer */
				(Unsigned16)size							/* Length */
			);
			*p_out_len += length;
			p_out_buf  += length;
		}
		/* 7. Meter year of manufacture */
		if (buf->from_value <= 7 && 7 <= buf->to_value)
		{
			memset(buffer, 0, 16);
			pbuffer = buffer;
			size =  R_USER_GetNamePlate(EM_NAME_PLATE_YEAR_OF_MANUFACT, pbuffer);
			if (size < 0)
			{
				return 0;
			}

			length = R_OBIS_EncodeAttribute(
				p_out_buf,									/* Output buffer, pointed to g_ServiceBuffer */
				OBIS_SERVICE_DATA_BUFFER_LENGTH,			/* Max length of g_ServiceBuffer */
				ATTR_TYPE_LONG_UNSIGNED,						/* Type */
				(Unsigned8 *)pbuffer,						/* Buffer */
				(Unsigned16)size							/* Length */
			);
			*p_out_len += length;
			p_out_buf  += length;
		}
	}

	return (*p_out_len - org_len);
}

#endif /* (defined(USED_CLASS_07) && USED_CLASS_07 == 1) */


#if (defined(USED_CLASS_20) && USED_CLASS_20 == 1)
/******************************************************************************
* Function Name : R_OBIS_Class20_Activate_PassiveCalendar
* Interface     : Unsigned8 R_OBIS_Class20_Activate_PassiveCalendar()
* Description   : Activate Passive Calendar
* Arguments     :
*				  Unsigned16          child_index,
* Function Calls: T.B.D
* Return Value  : Unsigned8: 1: NG, 0: OK
******************************************************************************/
Unsigned8 R_OBIS_Class20_Activate_PassiveCalendar(void)
{
	R_USER_Class20_SETACTION_Time(InsSave.timeCounter,0);
	return 0;
}

#endif /* (defined(USED_CLASS_22) && USED_CLASS_22 == 1) */

/******************************************************************************
* Function Name : R_USER_Event_Report
* Interface     : void R_USER_Event_Report( Unsigned8 event_type, Unsigned16 event_code )
* Description   : Update Event code and profile
* Arguments     : Unsigned8 event_type, 
*               : Unsigned16 event_code
* Function Calls: T.B.D
* Return Value  : void
******************************************************************************/
void R_USER_Event_Report(Unsigned8 event_type, Unsigned16 event_code)
{
	/* Check prameter */
	if (event_type > NON_ROLLER_EVENTS)
	{
		return;
	}
	/* Turn on flag for event notification */
	CONNMGR_ModifyAllEventFlag((Unsigned16)(1<<event_type), 0x00);

//#if (defined(USED_CLASS_07) && USED_CLASS_07 == 1)
//	{
////		Comment USER
//		class07_event_entry_t *p_event_entry = NULL;
//		Unsigned16 current_index = 0;
//
//		/* Rotate down the entry position */
//		g_Class07_Event_CurrentEntry[event_type]++;
//
//		if (g_Class07_Event_CurrentEntry[event_type] >= CLASS07_EVENT_MAX_ENTRY)
//		{
//			g_Class07_Event_CurrentEntry[event_type] = 0;
//		}
//		/* Count up number of entries are used */
//		if (g_Class07_Event_EntriesInUse[event_type] < CLASS07_EVENT_MAX_ENTRY)
//		{
//			g_Class07_Event_EntriesInUse[event_type]++;
//		}
//
//		/*
//		* Update Dailyload profile
//		* Based on meter data
//		*/
//		current_index = g_Class07_Event_CurrentEntry[event_type];
//		p_event_entry = &g_Class07_EventBuffer[event_type][current_index];
//		/* Capture 1 entry data */
//		memcpy(&(p_event_entry->clock_value), &current_RTCtime,  sizeof(date_time_t));
//		p_event_entry->event_code = event_code;
//		p_event_entry->current_value = R_USER_GetEMData(EM_CURRENT);
//		p_event_entry->voltage_value = R_USER_GetEMData(EM_VOLTAGE);
//		p_event_entry->power_factor = R_USER_GetEMData(EM_POWER_FACTOR);
//		p_event_entry->active_energy = R_USER_GetEMData(EM_CUCUMLARIVE_ENERGY_KWH);
////		END USER
//	}
//#endif

//#if (defined(USED_CLASS_01) && USED_CLASS_01 == 1)
//	/* Event code */
//	g_Class01_EventCode[event_type] = event_code;
//#endif

	return;
}

/******************************************************************************
* Function Name : R_OBIS_Timer_ConstInterruptCallback
* Interface     : void R_OBIS_Timer_ConstInterruptCallback(void)
* Description   : Const Callback (0.5 s) for internal processing
* Arguments     : None
* Function Calls: None
* Return Value  : None
******************************************************************************/
void R_OBIS_Timer_ConstInterruptCallback(void)
{
	/* Temp stick counter */
	//static Unsigned8 rtc_temp_stick = 0;

	/* Multiply by 2, because RTC is 0.5s interval, modify this when RTC interval changed */
	/*rtc_temp_stick++;

	if (rtc_temp_stick >= 2)
	{
		rtc_temp_stick = 0;
		rtc_stick ++;
	}*/
	rtc_stick ++;
}
#if (defined(USED_CLASS_07) && USED_CLASS_07 == 1)
Unsigned16 R_OBIS_Class07_FilterOneProfileEntry(
	buffer_t 			*buf,
	Unsigned16          child_index,
	Unsigned8           *p_out_buf,
	Unsigned16          *p_out_len,
	Unsigned8 			*p_buf_entry,
	Unsigned8 			*p_size_seq
)
{
	Unsigned16 org_len;
	Unsigned16 length;
	
	
	//-------------------
	attr_type_t attribute_type;
	Unsigned8 attribute_size;
	Unsigned8 attribute_index;
	
	//-------------------

	org_len = *p_out_len;
	
	
	
	
	/* Encode each element of entry */
	*p_out_buf++ = ATTR_TYPE_STRUCTURE;
	*p_out_buf++ = (Unsigned8)(buf->to_value - buf->from_value + 1);
	*p_out_len  += 2;
		
	
	for(attribute_index=1;attribute_index<=buf->to_value;attribute_index++)
	{
		getOBISIndex(child_index,attribute_index-1,&attribute_type,&attribute_size);
		
		if(attribute_index>=buf->from_value)
		{
			length = R_OBIS_EncodeAttribute(
				p_out_buf,									/* Output buffer, pointed to g_ServiceBuffer */
				OBIS_SERVICE_DATA_BUFFER_LENGTH,			/* Max length of g_ServiceBuffer */
				 attribute_type,						/* Type */
				p_buf_entry,	/* Buffer */
				attribute_size											/* Length */
			);
			*p_out_len += length;
			p_out_buf  += length;
			
		}
			
		p_buf_entry+=p_size_seq[attribute_index-1];
	}
		

	return (*p_out_len - org_len);
	
}


Unsigned8 getOBISIndex(Unsigned8 child_index,Unsigned8 object_index,attr_type_t *attribute_type,Unsigned8 *attribute_size)
{
	Unsigned16 class_no_objects;
	const Unsigned8 *profile_obis	=g_ChildTableClass07[child_index].p_capture_objects[object_index].logical_name;
	const Unsigned8 *target_obis	=g_ChildTableClass01[21].logical_name;
	Unsigned8 class_id				=g_ChildTableClass07[child_index].p_capture_objects[object_index].class_id;
	Unsigned8 attr_id				=g_ChildTableClass07[child_index].p_capture_objects[object_index].attr_id;
	
	if((class_id==8)||((class_id==4)&&(attr_id==5))||((profile_obis[0]==1)&&(profile_obis[1]==0)&&(profile_obis[2]==0)&&(profile_obis[3]==9)&&(profile_obis[4]==0)&&(profile_obis[5]==255)))
	{
		*attribute_type=ATTR_TYPE_OCTET_STRING;
		*attribute_size=12;
		return 0;
	}
	if(class_id==4)
		class_no_objects=g_ChildTableLengthClass04;
	else if(class_id==1)
			class_no_objects=g_ChildTableLengthClass01;
	else
		class_no_objects=g_ChildTableLengthClass03;
		
	while(class_no_objects--)
	{
		if(class_id==1)
		{
			
			target_obis=g_ChildTableClass01[class_no_objects].logical_name;
		}
		else if(class_id==4)
			target_obis=g_ChildTableClass04[class_no_objects].logical_name;
		else
		{
			
			target_obis=g_ChildTableClass03[class_no_objects].logical_name;
		}		
		if(target_obis[0]==profile_obis[0])
			if(target_obis[1]==profile_obis[1])
				if(target_obis[2]==profile_obis[2])
					if(target_obis[3]==profile_obis[3])
						if(target_obis[4]==profile_obis[4])
							if(target_obis[5]==profile_obis[5])
								break;
	}
	if(class_id==1)
	{
		*attribute_type=g_ChildTableClass01[class_no_objects].value.choice.type;
		*attribute_size=g_ChildTableClass01[class_no_objects].value.choice.size;
	}
	else if(class_id==4)
	{
		*attribute_type=g_ChildTableClass04[class_no_objects].value.choice.type;
		*attribute_size=g_ChildTableClass04[class_no_objects].value.choice.size;
	}

	else
	{
		*attribute_type=g_ChildTableClass03[class_no_objects].value.choice.type;
		*attribute_size=g_ChildTableClass03[class_no_objects].value.choice.size;
		
	}

	return 0;
}



/******************************************************************************
* Function Name : R_OBIS_Class07_BlockLoadUpdate_Pause
* Interface     : void R_OBIS_Class07_BlockLoadUpdate_Pause(void)
* Description   : Prepare before scan buffer
* Arguments     : None
* Function Calls: None
* Return Value  : None
******************************************************************************/
void R_OBIS_Class07_BlockLoadUpdate_Pause(void)
{
	/* Mark to start async service, pause the capturing */
	g_Class07_Obj0_Capture_Enable = 0;
}

/******************************************************************************
* Function Name : R_OBIS_Class07_BlockLoadUpdate_Start
* Interface     : void R_OBIS_Class07_BlockLoadUpdate_Start(void)
* Description   : Restart for async transfer
* Arguments     : None
* Function Calls: None
* Return Value  : None
******************************************************************************/
void R_OBIS_Class07_BlockLoadUpdate_Start(void)
{
	/* Reset counter of async service for Block Load Profile (object 0) */
	g_Class07_Obj0_Capture_Enable = 1;
}
#endif

void R_USER_GetRTCTime(date_time_t *p_date_time,Unsigned32 timeCounter);
/******************************************************************************
* Function Name : R_OBIS_Class07_BlockloadUpdate
* Interface     : void R_OBIS_Class07_BlockloadUpdate( void )
* Description   : Update Block load profile
* Arguments     : void
* Function Calls: T.B.D
* Return Value  : void
******************************************************************************/
void R_OBIS_Class07_BlockloadUpdate(void)
{
	g_Class07_Blockload_EntriesInUse=getLsEntries();
	
	if(InsSave.DailyDayCount>DAILYLS_TOTAL_DAYS)
		g_Class07_Dailyload_EntriesInUse = DAILYLS_TOTAL_DAYS;
	else
		g_Class07_Dailyload_EntriesInUse = InsSave.DailyDayCount;
	
	if(InsSave.MDResetCount>=HISTORY_RECORD)
		g_Class07_Billing_EntriesInUse = HISTORY_RECORD+1;
	else
		g_Class07_Billing_EntriesInUse = InsSave.MDResetCount+1; 
		
	
	R_USER_Class15_Secret_Restore(secret1,DLMS_LLS_SECRET_SIZE,MECHANISM_ID1_LOW_SECURITY);	
	R_USER_Class15_Secret_Restore(secret2,DLMS_HLS_SECRET_SIZE,MECHANISM_ID2_HIGH_SECURITY);
	
	g_Class07_Blockload_CapturePeriod=3600/ls_period;
	
	return;
}

//******************************************************************************/
void R_OBIS_Class07_EventUpdate(void)
{
	uint16_t EventEntries;
	Apprtc rtc;
	time_t *tptr;
	date_t *dptr;
	
	g_Class07_Event_EntriesInUse[0]=getCurrentLogCount(0);

	if(InsSave.PFailCount>PFAIL_EVENTS )
		EventEntries = PFAIL_EVENTS*2;
	else
		EventEntries = InsSave.PFailCount*2;
	
	g_Class07_Event_EntriesInUse[1]=EventEntries;

	if(TamperRecord.sw_log_count>SW_EVENTS)
		EventEntries = SW_EVENTS;
	else
		EventEntries = TamperRecord.sw_log_count;
	
	g_Class07_Event_EntriesInUse[2]=EventEntries;
	g_Class07_Event_EntriesInUse[3]=getCurrentLogCount(1);
	g_Class07_Event_EntriesInUse[4]=getCurrentLogCount(2);
	g_Class07_Event_EntriesInUse[5]=getCurrentLogCount(3);
	g_Class07_Event_EntriesInUse[6]=MAX_TAMPER_COUNT;
	g_Class07_Event_EntriesInUse[7]=0x00;
	
	for(EventEntries=0;EventEntries<8;EventEntries++)
		g_Class07_Event_MaxEntries[EventEntries]=getEventMaxEntries(EventEntries);
	

	R_USER_GetRTCTime(&g_Class20_ac_time,ScheduleActionDateTime.CalanderDateTime);
	
	ConvertTimeCounterToTime(ScheduleActionDateTime.BillDateTime,TIME_VAL,&rtc);
	
	
	tptr=(time_t*)g_Class22_bill_exetime[0].p_time;
	tptr->second=rtc.sec;
	tptr->minute=rtc.min;
	tptr->hour=rtc.hour;
	
	dptr=(date_t*)g_Class22_bill_exetime[0].p_date;
	EventEntries=2000+rtc.year;
	dptr->year_high=EventEntries>>8;
	dptr->year_low=EventEntries;
	dptr->month=rtc.month;
	dptr->day_of_month=rtc.day;
	dptr->day_of_week=rtc.week;
	#if (defined(IS_DLMS_ENABLED_EEPL) && (IS_DLMS_ENABLED_EEPL == 1))
	#if (defined(IRDA_TYPE_METER_HP) && (IRDA_TYPE_METER_HP == 1))
		if(TamperRecord.lastEventCode[0]!=0)
			read_eeprom((uint8_t*)&g_Class01_EventCode[0],TamperRecord.lastEventCode[0]+4,2);
		else
			g_Class01_EventCode[0]=0;
		if(TamperRecord.lastEventCode[1]!=0)
			read_eeprom((uint8_t*)&g_Class01_EventCode[3],TamperRecord.lastEventCode[1]+4,2);
		else
			g_Class01_EventCode[3]=0;

		if(TamperRecord.lastEventCode[2]!=0)
			read_eeprom((uint8_t*)&g_Class01_EventCode[4],TamperRecord.lastEventCode[2]+4,2);
		else
			g_Class01_EventCode[4]=0;

		if(TamperRecord.lastEventCode[3]!=0)
			read_eeprom((uint8_t*)&g_Class01_EventCode[5],TamperRecord.lastEventCode[3]+4,2);
		else
			g_Class01_EventCode[5]=0;	
	#endif
	#endif
	
	if(TamperRecord.sw_log_count>0)
		read_eeprom((uint8_t*)&g_Class01_EventCode[2],SOFT_CHANGE_LOG_LOC+((TamperRecord.sw_log_count-1)%SW_EVENTS)*5+4,1);
	else
		g_Class01_EventCode[2]=0;
		
	g_Class01_EventCode[1]=102;
	
	return;
}

/******************************************************************************
* Function Name : R_OBIS_Class07_BillingUpdate
* Interface     : void R_OBIS_Class07_BillingUpdate( void )
* Description   : Update Billing profile
* Arguments     : void
* Function Calls: T.B.D
* Return Value  : void
******************************************************************************/
void R_OBIS_Class07_BillingUpdate(void)
{
	uint32_t billdate=0;
#if (defined(USED_CLASS_03) && USED_CLASS_03 == 1)
	//read_eeprom((uint8_t*)&billdate,GetBillLoc(0)+25,4);
	R_USER_GetRTCTime(&g_Class03_BillingDate,billdate);
#endif

	return;
}
