/******************************************************************************
  Copyright (C) 2011 Renesas Electronics Corporation, All Rights Reserved.
*******************************************************************************
* File Name    : r_dlms_data.h
* Version      : 1.00
* Description  :
******************************************************************************
* History : DD.MM.YYYY Version Description
*         : 21.02.2013
******************************************************************************/

#ifndef _R_DLMS_DATA_H
#define _R_DLMS_DATA_H

/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#include "..\\r_dlms_typedef.h"				/* DLMS Type Definitions       */
#include "..\\Objects\\r_dlms_obis.h"				/* DLMS OBIS Definitions       */
#include "r_dlms_data_meter.h"				/* DLMS Data meter Definitions */
#include "..\\DLMS_User.h"					/* DLMS User Definitions       */
/* Application */



/******************************************************************************
Macro definitions
******************************************************************************/
/* Class ID = 07 - Profile generic class */
//#define CLASS07_BLOCKLOAD_MAX_ENTRY			2976		/* Max number of entries */
//#define CLASS07_DAILYLOAD_MAX_ENTRY			366		/* Max number of entries */
#define CLASS07_BILLING_MAX_ENTRY			7		/* Max number of entries */

#define CLASS07_EVENT_MAX_PROFILE			5		/* Max index of profile objects (from 0 ~ max) */
#define CLASS07_EVENT_MAX_ENTRY				25//100		/* Max number of entries */

/* Recording interval 1, for blockload profile (in second) */
#define CLASS07_BLOCKLOAD_INTERVAL			1800// for minutes USER
#define CLASS07_DAILYLOAD_INTERVAL			86400 /* 24 h*/
/******************************************************************************
Typedef definitions
******************************************************************************/
typedef struct tag_class07_instant_entry_t
{
	date_time_t	clock_value;		/* Clock */
	Integer32	voltage_value;		/*Voltage */
	Integer32	PhCurr_value;		/*Ph  Current */
	Integer32	NuCurr_value;		/*Nu Current */
	Integer32	Pf_value;			/*Pf */
	Integer32	Frq_value;			/*Frq */
	Integer32	PowerS_value;		/*apparent power */
	Integer32	PowerP_value;		/*active power */
	Integer32	kWh_value;			/*cummulative kWh */
	Integer32	kVAh_value;			/*cummulative kVAh */
	Integer32	kW_MD_value;		/*kW MD */
	date_time_t	kW_MD_clock;		/*kW MD Clock */
	Integer32	kVA_MD_value;		/*kVA MD */
	date_time_t	kVA_MD_clock;		/*kVA MD Clock */
	Integer32	PowerOn_value;		/*total power on duration */
	Integer32	TamperCount_value;	/*total tamper count*/
	Integer32	BillCount_value;	/*bill count*/
	Integer32	ProgramCount_value;	/*program count*/
	Integer32	ExpkWh_value;
	Integer32	ExpkVAh_value;
}class07_instant_entry_t;

typedef struct tag_class07_blockload_entry_t
{
	date_time_t	clock_value;		/* Clock */
	Integer32	voltage_value;		/* Average Voltage */
	Integer32	kWh_value;		/* Block Energy � kWh  */
	Integer32	blockkVAh_value;/* Block Energy � kVAh */
	Integer32	avg_current;		/* Block Energy � kVAh */
	Integer32	pf;				/* avg. power factor*/
//	Integer32	blockkVAh_value;/* Block Energy � kVAh */
} class07_blockload_entry_t;

typedef struct tag_class07_dailyload_entry_t
{
	date_time_t	clock_value;		/* Clock */
	Integer32	kWh_value;		/* Cumulative Energy � kWh */
	Integer32	kVAh_value;		/* Cumulative Energy � kVAh */
	Integer32	daykW_value;		/* day  kW MD*/
//	date_time_t	kW_MD_clock;		/*kVA MD Clock */
//	Integer32		active_MD;			/* MD kW */
//	date_time_t		active_MD_DT;
	Integer32	total_power;			/* Total power ON duration in minutes */
} class07_dailyload_entry_t;

typedef struct tag_md_billing_entry_t
{
	Integer32		MD;			/* MD kW */
	date_time_t		MD_DT;
}billmd;
typedef struct tag_class07_billing_entry_t
{
	date_time_t		bill_date;				/* Billing date */
	Integer32		power_factor;			/* System power factor for billing */
	Integer32		active_energy;			/* Cumulative Energy � kWh */
	Integer32		active_energyTZ[8];		/* Cumulative Energy kWh TZ1 ~ TZ6 */
	Integer32		apparent_energy;		/* Cumulative Energy � kVAh */
	Integer32		apparent_energyTZ[8];		/* Cumulative Energy kVAh TZ1 ~ TZ6 */
	Integer32		active_MD;			/* MD kW */
	date_time_t		active_MD_DT;
	Integer32		apparent_MD;			/* MD kVA */
	date_time_t		apparent_MD_DT;
	Integer32		total_power;			/* Total power ON duration in minutes */
	billmd			active_TZ[8];			/* MD kW */
	billmd			apparent_TZ[8];			/* MD kW */
	Integer32		exp_active_energy;			/* Cumulative Export Energy - kWh */	
	Integer32		exp_apparent_energy;			/* Cumulative Export Energy - kVAh */	
	Integer32		tamper_count;			/* Billing tamper count  */
} class07_billing_entry_t;


typedef struct tag_class07_event_entry_t
{
	date_time_t		clock_value;			/* Clock */
	Unsigned16		event_code;			/* Event code  */
	Integer32		ph_current_value;			/* Current  */
	//Integer32		neu_current_value;			/* Current  */
	Integer32		voltage_value;			/* Voltage  */
	Integer32		power_factor;			/* Signed Power Factor */
	Integer32		active_energy;			/* Cumulative Energy - kWh */
	Integer32		apparent_energy;			/* Cumulative Export Energy - kWh */
} class07_event_entry_t;

typedef struct tag_class07_event_powerfail_entry_t
{
	date_time_t		clock_value;			/* Clock */
	Unsigned16		event_code;			/* Event code  */
} class07_event_entry_powerfail_t;


typedef union tag_class07_bufferUnion_t
{
	class07_blockload_entry_t		g_Class07_BlockLoadBuffer;
	class07_dailyload_entry_t		g_Class07_DailyLoadBuffer;
	class07_billing_entry_t			g_Class07_BillingBuffer;
	class07_event_entry_t			g_Class07_EventBuffer;
	class07_event_entry_powerfail_t g_Class07_EventBuffer_Powerfail;
	class07_instant_entry_t			g_Class07_InstantBuffer;
	
}class07_bufferUnion_t;
/******************************************************************************
Variable Externs
******************************************************************************/

/******************************************************************************
Functions Prototypes
******************************************************************************/

/* Action Method */
Unsigned8 R_OBIS_ActionObjectMethod(
	st_Cosem_Method_Desc *cosem_method_desc,/* [In] COSEM Method Descriptor */
	service_type_t service_type,			/* [In] Service type */
	Unsigned16 child_id,					/* [In] Child ID in the child object table */
	Unsigned8 *p_out_buf,					/* [Out] Encoded output data */
	Unsigned16 *p_out_len,					/* [Out] Encoded length */
	Unsigned8 *p_data,						/* [In] Data from client */
	Unsigned16 data_len,					/* [In] Data length */
	Unsigned8 *block_transfer				/* [Out] Indicate whether block transfer is used or not */
);

/* Get Attribute */
Unsigned8 R_OBIS_GetObjectAttr(
	st_Cosem_Attr_Desc *cosem_attr_desc,	/* [In] COSEM Obj Descriptor */
	service_type_t service_type,			/* [In] Service type */
	Unsigned16 child_id,					/* [In] Child ID in the child object table */
	Unsigned8 *p_out_buf,					/* [Out] Encoded output data */
	Unsigned16 *p_out_len,					/* [Out] Encoded length */
	Unsigned8 *block_transfer,				/* [Out] Indicate whether block transfer is used or not */
	Unsigned8 *p_data						/* [In] Data from client */
);

/* Set Attribute */
Unsigned8 R_OBIS_SetObjectAttr(
	st_Cosem_Attr_Desc *cosem_attr_desc,	/* [In] COSEM Obj Descriptor */
	service_type_t service_type,			/* [In] Service type */
	Unsigned16 child_id,					/* [In] Child ID in the child object table */
	Unsigned8 *p_data,						/* [In] Data from client */
	Unsigned16 data_len,					/* [In] Data length */
	Unsigned8 block_transfer				/* [In] Indicate whether block transfer is used or not */
);

/* Get Attribute from EEPROM */
Unsigned8 R_OBIS_E2PRGetData(
	attr_access_t		*p_attr_access,	/* Access method option			*/
	attr_type_t 		attr_type,		/* Choice selection				*/
	Integer16			attr_size,		/* Buffer of value				*/
	Unsigned8			*p_out_buf,		/* Data pointer for get			*/
	Unsigned16			*p_out_len		/* Data length pointer for get	*/
);

/* Set Attribute to EEPROM*/
Unsigned8 R_OBIS_E2PRSetData(
	attr_access_t		*p_attr_access,	/* Access method option			*/
	attr_type_t 		attr_type,		/* Choice selection				*/
	Integer16			attr_size,		/* Buffer of value				*/
	Unsigned8			*pdata			/* Data from COSEM Appl layer	*/
);

/* Get 1 block of buffer */
Unsigned8 R_OBIS_BufferBlockGet(
	void				*buffer,		/* [In] pointer to buffer list	*/
	attr_access_t		*p_attr_access,	/* [In] Access method option	*/
	Unsigned16			block_index,	/* [In] Block index in buffer	*/
	Unsigned16			block_size,		/* [In] Block size in byte		*/
	Unsigned8			*p_out_buf,		/* [Out] Data pointer for get	*/
	Unsigned16			*p_out_len		/* [Out] length pointer for get	*/
);

/* Set to 1 block of buffer */
Unsigned8 R_OBIS_BufferBlockSet(
	void				*buffer,		/* [In] pointer to buffer list	 */
	attr_access_t		*p_attr_access,	/* [In] Access method option	 */
	Unsigned16			block_index,	/* [In] Block index in buffer	 */
	Unsigned16			block_size,		/* [In] Block size in byte		 */
	Unsigned8			*p_in_buf,		/* [In] Data pointer for set	 */
	Unsigned16			*p_in_len		/* [Out] length pointer after set */
);

/* Class07 - Profile Generic, buffer encode */
Unsigned8 R_OBIS_Class07_BufferScan (
	buffer_t            *buf,
	Unsigned8           selective_access,
	Unsigned16          child_index,
	Unsigned8           *p_selector_buffer
);
Unsigned8 R_OBIS_Class07_BufferFilter (
	buffer_t            *buf,
	Unsigned16          child_index,
	Unsigned16          request_entries_nr,
	Unsigned8           *p_out_buf,
	Unsigned16          *p_out_len
);

/* Class26 - Utility Table, buffer encode */
Unsigned8 R_OBIS_Class26_BufferScan (
	buffer_t            *buf,
	Unsigned8           selective_access,
	Unsigned16          child_index,
	Unsigned8           *p_selector_buffer
);
Unsigned8 R_OBIS_Class26_BufferFilter (
	buffer_t            *buf,
	Unsigned16          child_index,
	Unsigned16          request_entries_nr,
	Unsigned8           *p_out_buf,
	Unsigned16          *p_out_len
);

/* Buffer Scan By User Func */
Unsigned8 R_OBIS_BufferScanByUserFunc (
	buffer_t            *buf,
	Unsigned8           selective_access,
	Unsigned16          master_index,
	Unsigned16          child_index,
	Unsigned8           *p_selector_buffer
);

/* Buffer Filter By User Func */
Unsigned8 R_OBIS_BufferFilterByUserFunc (
	buffer_t            *buf,
	Unsigned16          master_index,
	Unsigned16          child_index,
	Unsigned16          request_entries_nr,
	Unsigned8           *p_out_buf,
	Unsigned16          *p_out_len
);


#if (defined(USED_CLASS_07) && USED_CLASS_07 == 1)
/* Class07 - Generic profile, Blockload profile */
Unsigned16 R_OBIS_Class07_GetBlockLoadEntry(
	class07_blockload_entry_t		*buf,
	Unsigned16						index
);


/* Class07 - Generic profile, Dailyload profile */
Unsigned16 R_OBIS_Class07_GetDailyLoadEntry(
	class07_dailyload_entry_t		*buf,
	Unsigned16						index
);


/* Class07 - Generic profile, Billing profile */
Unsigned16 R_OBIS_Class07_GetBillingEntry(
	class07_billing_entry_t			*buf,
	Unsigned16						index
);


/* Class07 - Generic profile, Event profile */
Unsigned16 R_USER_Class07_GetEventEntry(
	Unsigned32						event_type,
	class07_event_entry_t			*buf,
	Unsigned16						index
);


Unsigned16 R_OBIS_Class07_GetInstantEntry(class07_instant_entry_t *buf);


Unsigned8 R_OBIS_Class07_AddEventEntry(Unsigned8 event_type, Unsigned16 event_code);

#endif

/* Ack of async restart */
void R_OBIS_DataAsyncServiceRestart(void);
extern void set_passive_tod_zone(void);

#endif /* _R_DLMS_DATA_H */

