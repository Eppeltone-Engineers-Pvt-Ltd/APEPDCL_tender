/******************************************************************************
  Copyright (C) 2011 Renesas Electronics Corporation, All Rights Reserved.
*******************************************************************************
* File Name    : r_dlms_obis_ic.h
* Version      : 1.00
* Description  :
******************************************************************************
* History : DD.MM.YYYY Version Description
*         : 21.02.2013
******************************************************************************/

#ifndef _R_DLMS_OBIS_IC_H
#define _R_DLMS_OBIS_IC_H

/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#include "..\\r_dlms_typedef.h"				/* DLMS Type Definitions */
#include "..\\DLMS_User.h"					/* DLMS User Definitions */

/******************************************************************************
Macro definitions
******************************************************************************/
/* For choice_t type */
#define CHOICE_NULL_DATA              {ATTR_TYPE_NULL_DATA             ,    0}	/* Null data (no data)                                */
#define CHOICE_BOOLEAN                {ATTR_TYPE_BOOLEAN               ,    1}	/* Boolean                                            */
#define CHOICE_BIT_STRING(size)       {ATTR_TYPE_BIT_STRING            , size}	/* An ordered sequence of boolean values              */
#define CHOICE_DOUBLE_LONG            {ATTR_TYPE_DOUBLE_LONG           ,    4}	/* Integer32                                          */
#define CHOICE_DOUBLE_LONG_UNSIGNED   {ATTR_TYPE_DOUBLE_LONG_UNSIGNED  ,    4}	/* Unsigned32                                         */
#define CHOICE_OCTET_STRING(size)     {ATTR_TYPE_OCTET_STRING          , size}	/* An ordered sequence of octets (8 bit bytes)        */
#define CHOICE_VISIBLE_STRING(size)   {ATTR_TYPE_VISIBLE_STRING        , size}	/* An ordered sequence of ASCII characters            */
#define CHOICE_UTF8_STRING(size)      {ATTR_TYPE_UTF8_STRING           , size}	/* An ordered sequence of characters encoded as UTF-8 */
#define CHOICE_BCD                    {ATTR_TYPE_BCD                   ,    1}	/* Binary coded decimal                               */
#define CHOICE_INTEGER                {ATTR_TYPE_INTEGER               ,    1}	/* Integer8                                           */
#define CHOICE_LONG                   {ATTR_TYPE_LONG                  ,    2}	/* Integer16                                          */
#define CHOICE_UNSIGNED               {ATTR_TYPE_UNSIGNED              ,    1}	/* Unsigned8                                          */
#define CHOICE_LONG_UNSIGNED          {ATTR_TYPE_LONG_UNSIGNED         ,    2}	/* Unsigned16                                         */
#define CHOICE_LONG64                 {ATTR_TYPE_LONG64                ,    8}	/* Integer64                                          */
#define CHOICE_LONG64_UNSIGNED        {ATTR_TYPE_LONG64_UNSIGNED       ,    8}	/* Unsigned64                                         */
#define CHOICE_ENUM                   {ATTR_TYPE_ENUM                  ,    1}	/* The elements of the enumeration type               */
#define CHOICE_FLOAT32                {ATTR_TYPE_FLOAT32               ,    4}	/* OCTET STRING (SIZE(4))                             */
#define CHOICE_FLOAT64                {ATTR_TYPE_FLOAT64               ,    8}	/* OCTET STRING (SIZE(8))                             */
#define CHOICE_DATE_TIME              {ATTR_TYPE_DATE_TIME             ,   12}	/* OCTET STRING (SIZE(12))                            */
#define CHOICE_DATE                   {ATTR_TYPE_DATE                  ,    5}	/* OCTET STRING (SIZE(5))                             */
#define CHOICE_TIME                   {ATTR_TYPE_TIME                  ,    4}	/* OCTET STRING (SIZE(4))                             */
#define CHOICE_ARRAY                  {ATTR_TYPE_ARRAY                 ,   -1}	/* The elements of the array                          */
#define CHOICE_STRUCTURE              {ATTR_TYPE_STRUCTURE             ,   -1}	/* The elements of the structure                      */
#define CHOICE_COMPACT_ARRAY          {ATTR_TYPE_COMPACT_ARRAY         ,   -1}	/* The elements of the compact array                  */

/* For value_t, status_t */
#define VALUE(choice, buffer)				{choice, buffer}
#define STATUS(choice, buffer)				{choice, buffer}

/* For dyn_value_t, dyn_status_t, dyn_date_time_t */
#define DYN_VALUE(choice, buffer, access)	{choice, buffer, access}
#define DYN_STATUS(choice, buffer, access)	{choice, buffer, access}
#define DYN_DATE_TIME(buffer, access)		{CHOICE_OCTET_STRING(sizeof(date_time_t)), buffer, access}

/* For date_t type */
#define DATE_YEAR_NOT_SPECIFIED 			(0xFFFF)	/* The year is not specified       */
#define DATE_MONTH_DAYLIGHT_SAVINGS_END		(0xFD)		/* Month is daylight savings end   */
#define DATE_MONTH_DAYLIGHT_SAVINGS_BEGIN	(0xFE)		/* Month is daylight savings begin */
#define DATE_MONTH_NOT_SPECIFIED 			(0xFF)		/* Month is not specified          */
#define DATE_DAY_2ND_LAST_OF_MONTH 			(0xFD)		/* 2nd last day of month           */
#define DATE_DAY_LAST_OF_MONTH 				(0xFE)		/* Last day of month               */
#define DATE_DAY_NOT_SPECIFIED 				(0xFF)		/* Day is not specified            */
#define DATE_DAY_RESERVED_START 			(0xE0)		/* Reserved (start � end),         */
#define DATE_DAY_RESERVED_END 				(0xFC)		/* from 0xE0 to 0xFC               */
#define DATE_WEEK_MON 						(0x01)		/* Week is Monday                  */
#define DATE_WEEK_TUE 						(0x02)		/* Week is Tuesday                 */
#define DATE_WEEK_WED 						(0x03)		/* Week is Wednesday               */
#define DATE_WEEK_THU 						(0x04)		/* Week is Thursday                */
#define DATE_WEEK_FRI 						(0x05)		/* Week is Friday                  */
#define DATE_WEEK_SAT 						(0x06)		/* Week is Saturday                */
#define DATE_WEEK_SUN 						(0x07)		/* Week is Sunday                  */
#define DATE_WEEK_NOT_SPECIFIED 			(0xFF)		/* Week is not specified           */

/* For time_t type */
#define TIME_HOUR_NOT_SPECIFIED 			(0xFF)		/* Hour is not specified.          */
#define TIME_MINUTE_NOT_SPECIFIED 			(0xFF)		/* Minute is not specified.        */
#define TIME_SECOND_NOT_SPECIFIED 			(0xFF)		/* Second is not specified.        */
#define TIME_HUNDREDTHS_NOT_SPECIFIED 		(0xFF)		/* Hundredths is not specified.    */
#define NOT_SPECIFIED 						(0xFF)		/* generic value is not specified.    */

/* For deviation element */
#define DEVIATION_NOT_SPECIFIED 			(0x8000)	/* Deviation is not specified.     */

/* For clock_status_t */
#define CLOCK_STATUS_INVALID_VALUE 			(0x01)		/* Clock status set bit invalid_value          */
#define CLOCK_STATUS_DOUBTFUL_VALUE 		(0x02)		/* Clock status set bit doubtful_value         */
#define CLOCK_STATUS_DIFFERENT_CLOCK_BASE	(0x04)		/* Clock status set bit different_clock_base   */
#define CLOCK_STATUS_INVALID_CLOCK_STATUS	(0x08)		/* Clock status set bit invalid_clock_status   */
#define CLOCK_STATUS_DAYLIGHT_SAVING_ACTIVE	(0x80)		/* Clock status set bit daylight_saving_active */

/* For access_option_t */
#define ATTR_ACCESS_MEMORY					{ACCESS_MTD_MEM			, 0					, 0}			/* Auto read from internal memory                 */
#define ATTR_ACCESS_USERFUNC				{ACCESS_MTD_USERFUNC	, 0					, 0}			/* Read through function R_OBIS_GetObjectAttr()   */
#define ATTR_ACCESS_E2PR(start_addr,size)	{ACCESS_MTD_E2PR		, start_addr		, size}			/* Auto read from EEPROM                          */

/******************************************************************************
Typedef definitions
******************************************************************************/
/* Attribute data types */
typedef enum tag_attr_type_t
{
	/* Simple Data Type */
	ATTR_TYPE_NULL_DATA				= 0  ,	/* Null data (no data)                                */
	ATTR_TYPE_BOOLEAN               = 3  ,  /* Boolean                                            */
	ATTR_TYPE_BIT_STRING            = 4  ,  /* An ordered sequence of boolean values              */
	ATTR_TYPE_DOUBLE_LONG           = 5  ,  /* Integer32                                          */
	ATTR_TYPE_DOUBLE_LONG_UNSIGNED  = 6  ,  /* Unsigned32                                         */
	ATTR_TYPE_OCTET_STRING          = 9  ,  /* An ordered sequence of octets (8 bit bytes)        */
	ATTR_TYPE_VISIBLE_STRING        = 10 ,  /* An ordered sequence of ASCII characters            */
	ATTR_TYPE_UTF8_STRING           = 12 ,  /* An ordered sequence of characters encoded as UTF-8 */
	ATTR_TYPE_BCD                   = 13 ,  /* Binary coded decimal                               */
	ATTR_TYPE_INTEGER               = 15 ,  /* Integer8                                           */
	ATTR_TYPE_LONG                  = 16 ,  /* Integer16                                          */
	ATTR_TYPE_UNSIGNED              = 17 ,  /* Unsigned8                                          */
	ATTR_TYPE_LONG_UNSIGNED         = 18 ,  /* Unsigned16                                         */
	ATTR_TYPE_LONG64                = 20 ,  /* Integer64                                          */
	ATTR_TYPE_LONG64_UNSIGNED       = 21 ,  /* Unsigned64                                         */
	ATTR_TYPE_ENUM                  = 22 ,  /* The elements of the enumeration type               */
	ATTR_TYPE_FLOAT32               = 23 ,  /* OCTET STRING (SIZE(4))                             */
	ATTR_TYPE_FLOAT64               = 24 ,  /* OCTET STRING (SIZE(8))                             */
	ATTR_TYPE_DATE_TIME             = 25 ,  /* OCTET STRING (SIZE(12))                            */
	ATTR_TYPE_DATE                  = 26 ,  /* OCTET STRING (SIZE(5))                             */
	ATTR_TYPE_TIME                  = 27 ,  /* OCTET STRING (SIZE(4))                             */

	/* Complex Data Type */
	ATTR_TYPE_ARRAY                 = 1  ,  /* The elements of the array                          */
	ATTR_TYPE_STRUCTURE             = 2  ,  /* The elements of the structure                      */
	ATTR_TYPE_COMPACT_ARRAY         = 19   /* The elements of the compact array                  */
} attr_type_t;

/* Date type */
typedef struct tag_date_t
{
	Unsigned8 year_high;                    /* HI and LO of year. Interpreted as long-unsigned,   */
	Unsigned8 year_low;                     /* Range is 0�0xFFFF-1. 0xFFFF mean not specified.    */
	Unsigned8 month;                        /* Month. Interpreted as unsigned. 1...12             */
	Unsigned8 day_of_month;                 /* Day. Interpreted as unsigned. 1...31               */
	Unsigned8 day_of_week;                  /* Week. Interpreted as unsigned. 1...7               */
} date_t;

/* Time type */
typedef struct tag_time_t
{
	Unsigned8 hour;                         /* Hour. Interpreted as unsigned. 0...23              */
	Unsigned8 minute;                       /* Minute. Interpreted as unsigned. 0...59            */
	Unsigned8 second;                       /* Hour. Interpreted as unsigned. 0...59              */
	Unsigned8 hundredths;                   /* Hundredths. Interpreted as unsigned. 0...99        */
} time_t;

/* Clock status */
typedef struct tag_clock_status_t
{
	Unsigned8 invalid_value			:1;     /* Indicate the time could NOT be recovered after
	                                         * an incident.                                       */
	Unsigned8 doubtful_value		:1;     /* Indicate the time could be recovered after an
	                                         * incident but the value cannot be guaranteed.       */
	Unsigned8 different_clock_base	:1;     /* Bit is set if the basic timing information for
	                                         * the clock at the actual moment is taken from a
											 * timing source different from the source specified
											 * in clock_base                                      */
	Unsigned8 invalid_clock_status	:1;     /* This bit indicates that at least one bit of the
	                                         * clock status is invalid. Some bits may be correct. */
	Unsigned8 reserved0				:1;     /* Reserved (no-use)                                  */
	Unsigned8 reserved1				:1;     /* Reserved (no-use)                                  */
	Unsigned8 reserved2				:1;     /* Reserved (no-use)                                  */
	Unsigned8 daylight_saving_active:1;     /* Flag set to true: the transmitted time contains the
                                             * daylight saving deviation (summer time).
											 * Flag set to false: the transmitted time does not
											 * contain daylight saving deviation (normal time).   */
} clock_status_t;

/* Clock base */
typedef enum tag_clock_base_t
{
	CLOCK_BASE_NOT_DEFINED	         = 0,
	CLOCK_BASE_INTERNAL_CRYSTAL      = 1,
	CLOCK_BASE_MAINS_FREQUENCY_50_HZ = 2,
	CLOCK_BASE_MAINS_FREQUENCY_60_HZ = 3,
	CLOCK_BASE_GPS                   = 4,
	CLOCK_BASE_RADIO_CONTROLLED      = 5
} clock_base_t;

/* Date time type */
typedef struct tag_date_time_t
{
	Unsigned8 year_high;                    /* HI and LO of year. Interpreted as long-unsigned,   */
	Unsigned8 year_low;                     /* Range is 0�0xFFFF-1. 0xFFFF mean not specified.    */
	Unsigned8 month;                        /* Month. Interpreted as unsigned. 1...12             */
	Unsigned8 day_of_month;                 /* Day. Interpreted as unsigned. 1...31               */
	Unsigned8 day_of_week;                  /* Week. Interpreted as unsigned. 1...7               */
	Unsigned8 hour;                         /* Hour. Interpreted as unsigned. 0...23              */
	Unsigned8 minute;                       /* Minute. Interpreted as unsigned. 0...59            */
	Unsigned8 second;                       /* Hour. Interpreted as unsigned. 0...59              */
	Unsigned8 hundredths;                   /* Hundredths. Interpreted as unsigned. 0...99        */
	Unsigned8 deviation_high;               /* Interpreted as long,                               */
	Unsigned8 deviation_low;                /* Range -720�720, in minutes of local time to GMT.   */
	Unsigned8 clock_status;                 /* The clock status for date time value.              */
} date_time_t;

/* Physical unit */
typedef enum tag_physical_unit_t
{
	PHY_UNIT_YEAR                          = 1  , /* Year                                         */
	PHY_UNIT_MONTH                         = 2  , /* Month                                        */
	PHY_UNIT_WEEK                          = 3  , /* Week. 7*24*60*60 s                           */
	PHY_UNIT_DAY                           = 4  , /* Day. 24*60*60 s                              */
	PHY_UNIT_HOUR                          = 5  , /* Hour. 60*60 s                                */
	PHY_UNIT_MIN                           = 6  , /* Minute. 60 s                                 */
	PHY_UNIT_SECOND                        = 7  , /* Second                                       */
	PHY_UNIT_DEGREE                        = 8  , /* Phase angle in degree (�)                    */
	PHY_UNIT_DEGREE_CELSIUS                = 9  , /* Temperature (T) in �C = K - 273.15           */
	PHY_UNIT_CURRENCY                      = 10 , /* (local) Currency                             */
	PHY_UNIT_METRE                         = 11 , /* Metre (m)                                    */
	PHY_UNIT_METRE_PER_SEC                 = 12 , /* Metre per second (m/s)                       */
	PHY_UNIT_CUBIC_METRE_V                 = 13 , /* Cubic metre (m^3) for volume (V).            */
	PHY_UNIT_CUBIC_METRE_C                 = 14 , /* Cubic metre (m^3). Corrected volume          */
	PHY_UNIT_CUBIC_METRE_PER_HOUR_V        = 15 , /* Cubic metre (m^3) per hour.                  */
	PHY_UNIT_CUBIC_METRE_PER_HOUR_C        = 16 , /* Corrected cubic metre (m^3) per hour.        */
	PHY_UNIT_CUBIC_METRE_PER_DAY_V         = 17 , /* Cubic metre (m^3) per day.                   */
	PHY_UNIT_CUBIC_METRE_PER_DAY_C         = 18 , /* Corrected cubic metre (m^3) per day.         */
	PHY_UNIT_LITRE                         = 19 , /* Litre (l). 10^-3 m^3                         */
	PHY_UNIT_KILOGRAM                      = 20 , /* Kilogram (kg).                               */
	PHY_UNIT_NEWTON                        = 21 , /* Newton (N). Unit of force (F)                */
	PHY_UNIT_NEWTON_METER                  = 22 , /* Newtonmeter (Nm). J = Nm = Ws                */
	PHY_UNIT_PASCAL                        = 23 , /* Pascal (Pa). N/m^2                           */
	PHY_UNIT_BAR                           = 24 , /* Bar. 10^5 (N/m^2)                            */
	PHY_UNIT_JOULE                         = 25 , /* Joule. J = Nm = Ws                           */
	PHY_UNIT_JOULE_PER_HOUR                = 26 , /* Joule per hour. J/(60*60s)                   */
	PHY_UNIT_WATT                          = 27 , /* Watt (W). W = J/s                            */
	PHY_UNIT_VOLT_AMPERE                   = 28 , /* Volt-Ampere (VA).                            */
	PHY_UNIT_VAR                           = 29 , /* Var (of reactive power)                      */
	PHY_UNIT_WATT_HOUR                     = 30 , /* Watt-hour (Wh). W*(60*60s)                   */
	PHY_UNIT_VOLT_AMPERE_HOUR              = 31 , /* Volt-ampere-hour (VAh). VA*(60*60s)          */
	PHY_UNIT_VAR_HOUR                      = 32 , /* VAr-hour (VArh). VAr*(60*60s)                */
	PHY_UNIT_AMPERE                        = 33 , /* Ampere (A)                                   */
	PHY_UNIT_COULOMB                       = 34 , /* Coulomb. C = As                              */
	PHY_UNIT_VOLT                          = 35 , /* Volt (V)                                     */
	PHY_UNIT_VOLT_PER_METRE                = 36 , /* Volt per metre (V/m).                        */
	PHY_UNIT_FARAD                         = 37 , /* Farad (F). C/V = As/V                        */
	PHY_UNIT_OHM                           = 38 , /* Ohm. O = V/A                                 */
	PHY_UNIT_OHM_METRE                     = 39 , /* Ohm metre (Om)                               */
	PHY_UNIT_WEBER                         = 40 , /* Weber. Wb = Vs                               */
	PHY_UNIT_TESLA                         = 41 , /* Tesla (T). Wb/m^2                            */
	PHY_UNIT_AMPERE_PER_METRE              = 42 , /* Ampere per metre. A/m                        */
	PHY_UNIT_HENRY                         = 43 , /* Henry. H = Wb/A                              */
	PHY_UNIT_HERTZ                         = 44 , /* Hertz (Hz).                                  */
	PHY_UNIT_PULSE_PER_WATT_HOUR           = 45 , /* 1Pulse/Wh                                    */
	PHY_UNIT_PULSE_PER_VAR_HOUR            = 46 , /* 1Pulse/VArh                                  */
	PHY_UNIT_PULSE_PER_VA_HOUR             = 47 , /* 1Pulse/VAh                                   */
	PHY_UNIT_VOLT_SQUARED_HOURS            = 48 , /* Volt-squared-hours. V^2(60*60s)              */
	PHY_UNIT_AMPERE_SQUARED_HOURS          = 49 , /* Ampere-squared-hours. A^2(60*60s)            */
	PHY_UNIT_KILOGRAM_PER_SECOND           = 50 , /* Kilogram per second. Kg/s                    */
	PHY_UNIT_SIEMENS                       = 51 , /* Siemens (S). 1/O                             */
	PHY_UNIT_KELVIN                        = 52 , /* Kelvin (K)                                   */
	PHY_UNIT_PULSE_PER_VOLT_SQUARED_HOUR   = 53 , /* 1/(V^2h)                                     */
	PHY_UNIT_PULSE_PER_AMPERE_SQUARED_HOUR = 54 , /* 1/(A^2h)                                     */
	PHY_UNIT_PULSE_PER_VOLUME              = 55 , /* 1/m^3                                        */
	PHY_UNIT_PERCENTAGE                    = 56 , /* %                                            */
	PHY_UNIT_AMPERE_HOUR                   = 57 , /* Ah                                           */
	PHY_UNIT_WATT_HOUR_PER_VOLUME          = 60 , /* Wh/m^3. 3,6*103 J/m3                         */
	PHY_UNIT_JOULE_PER_VOLUME              = 61 , /* J/m^3                                        */
	PHY_UNIT_MOLE_PERCENT                  = 62 , /* Mole percent                                 */
	PHY_UNIT_G_PER_VOLUME                  = 63 , /* g/m^3                                        */
	PHY_UNIT_PASCAL_SECOND                 = 64 , /* Pa s                                         */
	PHY_UNIT_JOULE_PER_KILOGRAM            = 65 , /* J/kg                                         */
	PHY_UNIT_DECIBELS_METRE                = 70 , /* dBm                                          */
	PHY_UNIT_RESERVED                      = 253, /* Reserved                                     */
	PHY_UNIT_OTHER_UNIT                    = 254, /* Other unit                                   */
	PHY_UNIT_NONE                          = 255 /* No unit                                      */
} physical_unit_t;

/* Choice type */
typedef struct tag_choice_t
{
	attr_type_t	type;	/* Type of the choice   */
	Integer8	size;	/* Size of choosen type */
} choice_t;

/* Access method */
typedef enum tag_attr_access_method_t
{
	ACCESS_MTD_MEM = 0,				/* Auto read from internal memory */
	ACCESS_MTD_USERFUNC,			/* Read through function R_OBIS_GetObjectAttr(). */
	ACCESS_MTD_E2PR,				/* Auto read from EEPROM */
} attr_access_method_t;

/* Access option type */
typedef struct tag_attr_access_t
{
	attr_access_method_t method;	/* Access method */
	Unsigned16 addr;				/* Start addr in EEPROM  */
	Unsigned16 size;				/* Size in EEPROM  */
} attr_access_t;

/* Dynamic value type (Support 3 selective methods) */
typedef struct tag_dyn_value_t
{
	choice_t		choice;			/* Choice of type */
	void 			*buffer;		/* Buffer of value */
	attr_access_t	access;			/* Access option of value */
} dyn_value_t, dyn_status_t, dyn_date_time_t;

/* Value, status type */
typedef struct tag_value_t
{
	choice_t		choice;			/* Choice of type */
	void 			*buffer;		/* Buffer of value */
} value_t, status_t;

/* Scaler Unit */
typedef struct tag_scaler_unit_t
{
	Integer8		scaler;			/* This is the exponent (to the base of 10) of the multiplication factor. */
	physical_unit_t	unit;			/* Enumeration defining the physical unit. See 4.5.3.6 for more detail.   */
} scaler_unit_t;

/* Buffer scan function prototype */
typedef void (*fp_buff_scan_func_t)(
	void                *p_buffer,					/* [In] Buffer of value                           */
	attr_access_t       *p_access,					/* [In] Pointer to access method option           */
	Unsigned8           selective_access,			/* [In] Selective access                          */
	Unsigned16          child_index,				/* [In] Child ID                                  */
	Unsigned8           *p_selector_buffer,			/* [In] Pointer to the selector (from Client)     */
	Unsigned16          *p_one_entry_len,			/* [Out] Length of encode 1 entry after filter    */
	Unsigned16          *p_nr_entries,				/* [Out] Number of entries need to filter         */
	void                *p_filter_info 				/* [Out] Data pointer to filter info (need CAST)  */
);

/* Buffer filter function prototype */
typedef void (*fp_buff_filter_func_t)(
	void                *p_buffer,					/* [In] Buffer of value                           */
	attr_access_t       *p_access,					/* [In] Pointer to access method option           */
	Unsigned16          child_index,				/* [In] Child ID                                  */
	void                *p_filter_info, 			/* [In] Data pointer to filter info (need CAST)   */
	Unsigned16          remain_entries_nr,			/* [In] Number of remain entries in buffer        */
	Unsigned16          request_entries_nr,			/* [In] Number of entries need to filter          */
	Unsigned16          *p_completed_entries_nr,	/* [Out] Number of actual entries to filter       */
	Unsigned8           *p_out_buf,					/* [Out] Data pointer for get                     */
	Unsigned16          *p_out_len 					/* [Out] Data length pointer for get              */
);

/* Buffer generic function prototype */
typedef void (*fp_buff_generic_func_t)(void);

/* Buffer type */
typedef struct tag_buffer_t
{
	void 						*p_buffer;						/* Buffer of value                                     */
	attr_access_t				*p_access;						/* Pointer to access method option                     */
	Unsigned16					*p_current_buf_index;			/* Pointer to index of current entries in buffer       */
	Unsigned16					async_entries_per_block;		/* Number of entries per block to transfer             */
	Unsigned16					one_entry_len;					/* Length of encode 1 entry after filter               */
	Unsigned16					async_remain_entries;			/* Number of remain entries in run-time                */
	Unsigned32					from_entry;						/* First entry to retrieve in run-time                 */
	Unsigned32					to_entry;						/* Last entry to retrieve in run-time                  */
	Unsigned16					from_value;						/* Index of first value to retrieve in run-time        */
	Unsigned16					to_value;						/* Index of last value to retrieve  in run-time        */
} buffer_t;

/* Sort method */
typedef enum tag_sort_method_t
{
	SORT_METHOD_FIFO = 1,           /* fifo (first in first out) */
	SORT_METHOD_LIFO,               /* lifo (last in first out)  */
	SORT_METHOD_LARGEST,            /* largest                   */
	SORT_METHOD_SMALLEST,           /* smallest                  */
	SORT_METHOD_NEAREST_TO_ZERO,    /* nearest_to_zero           */
	SORT_METHOD_FAREST_FROM_ZERO   /* farest_from_zero          */
} sort_method_t;

/* SAP assignment element */
typedef struct tag_asslist_entry_t
{
	Unsigned16	sap;
	value_t		logical_device_name;
} asslist_entry_t;

/* Security policy */
typedef enum tag_security_policy_t
{
	SECURITY_NONE = 0,					/* nothing */
	SECURITY_AUTHENTICATED,				/*all messages to be authenticated */
	SECURITY_ENCRYPTED,					/* all messages to be encrypted */
	SECURITY_AUTHENTICATED_ENCRYPTED	/*all messages to be authenticated and encrypted  */
} security_policy_t;

/* Security suit */
typedef enum tag_security_suit_t
{
	SECURITY_AES_SUIT = 0				/* AES-GCM-128 for authenticated encryption and AES-128 for key wrapping */
} security_suit_t;

/* Security policy */
typedef enum tag_service_t
{
	SERVICE_WRITE_ATTRIBUTE = 1,		/* Write attribute */
	SERVICE_EXECUTE_METHOD				/* Execute specific method */
} service_t;

/* Action_specification type */
typedef struct tag_script_action_spec_t
{
	service_t service_id;
	Unsigned16 class_id;
	Unsigned8 logical_name[6];
	Integer8 index;					/* Index of attribute or specific method */
	value_t  parameter;				/* Parameter for ACTION/SET */
} script_action_spec_t;

/* Script type */
typedef struct tag_script_t
{
	Unsigned16 script_id;
	script_action_spec_t * p_actions;
	Unsigned8 nr_actions;				/* Element number of actions */
} script_t;

/* Schedule table entry type */
typedef struct tag_schedule_table_entry_t
{
	Unsigned16 index;					/* Index */
	Unsigned8 *p_enable;				/* Enable */
	Unsigned8 script_logical_name[6];	/* The logical name of the �Script table� object */
	Unsigned16 script_selector;			/* Script_identifier of the script to be executed */
	time_t * p_switch_time;				/* switch_time */
	Unsigned16 validity_window;			/* validity_window */
	Unsigned8 exec_weekdays;			/* Day of week. Interpreted as unsigned. 1...7 */
	Unsigned8 exec_specdays;			/* Link to the IC �Special days table�, day_id; */
	date_t * p_begin_date;				/* begin_date */
	date_t * p_end_date;				/* end_date */
} schedule_table_entry_t;

/* Special day entry type */
typedef struct tag_spec_day_entry_t
{
	Unsigned16 index;				/* Index */
	date_t *p_specialday_date;		/* specialday_date */
	Unsigned8 day_id;				/* exec_specdays in the related object of IC �Schedule� */
} spec_day_entry_t;

/* Seanson profile type */
typedef struct tag_season_profile_t
{
	Unsigned8 *p_season_profile_name;
	Unsigned16 season_profile_name_len;
	date_time_t *p_season_start;
	Unsigned8 *p_week_profile_name;		/* The week_profile active in this season  */
	Unsigned16 week_profile_name_len;	/* The week_profile name len  */
} season_profile_t;

/* Week profile type */
typedef struct tag_week_profile_t
{
	Unsigned8 *p_week_profile_name;
	Unsigned8 week_profile_name_len;
	Unsigned8 Monday;				/* The day_profile valid on Monday */
	Unsigned8 Tuesday;				/* The day_profile valid on Tuesday */
	Unsigned8 Wednesday; 			/* The day_profile valid onWednesday */
	Unsigned8 Thursday;				/* The day_profile valid on Thursday */
	Unsigned8 Friday;				/* The day_profile valid on Friday */
	Unsigned8 Saturday; 			/* The day_profile valid on Saturday */
	Unsigned8 Sunday;				/* The day_profile valid on Sunday */
} week_profile_t;

/* Day profile action type */
typedef struct tag_day_profile_action_t
{
	time_t * p_start_time;					/* start time */
	Unsigned8 script_logical_name[6];		/* The logical name of the �Script table� object  */
	Unsigned16 script_selector;				/* Script_identifier of the script to be executed */
} day_profile_action_t;

/* Day profile type */
typedef struct tag_day_profile_t
{
	Unsigned8 day_id;						/* day id */
	day_profile_action_t *day_schedule;		/* day_schedule */
	Unsigned8 nr_day_schedule;				/* no. of day_schedule */
} day_profile_t;

/* Value definition  type */
typedef struct tag_value_definition_t
{
	Unsigned16 class_id;					/* class id */
	Unsigned8 logical_name[6];				/* logical_name */
	Integer16 index;						/* index */
} value_definition_t;

/* Action item  type */
typedef struct tag_action_item_t
{
	Unsigned8 script_logical_name[6];		/* The logical name of the �Script table� object  */
	Unsigned16 script_selector;				/* Script_identifier of the script to be executed */
} action_item_t;

/* Action set  type */
typedef struct tag_action_set_t
{
	action_item_t *action_up;				/* Action up   */
	action_item_t *action_down;				/* Action down */
} action_set_t;

/* Single Action */
typedef enum tag_single_action_t
{
	SINGLE_ACTION_1_TIME_WILDCARD = 1,		/* Size of execution_time = 1; wildcard in date allowed */
	SINGLE_ACTION_N_SAMETIME,				/* Size of execution_time = n; all timevalues are the same, wildcards in date not allowed */
	SINGLE_ACTION_N_SAMETIME_WILDCARDS,		/* Size of execution_time = n; all timevalues are the same, wildcards in date are allowed */
	SINGLE_ACTION_N_TIME,					/* Size of execution_time = n; timevalues may be different, wildcards in date not allowed */
	SINGLE_ACTION_N_TIME_WILDCARDS 		/* Size of execution_time = n; timevalues may be different, wildcards in date are allowed  */
} single_action_t;

/* Response time */
typedef struct tag_execution_time_date_t
{
	time_t *p_time;
	date_t *p_date;
} execution_time_date_t;

/* IEC protocal mode */
typedef enum tag_protocal_mode_t
{
	PROTOCAL_IEC = 0,						/* According to IEC 62056-21(modes A�E) */
	PROTOCAL_DLMSUA, 						/* According to Clause 8 of DLMS UA 1000-2 Ed.7.0 */
	PROTOCAL_NOT_SPECIFIED					/* Protocol not specified */
} protocal_mode_t;

/* Baud rate */
typedef enum tag_baud_rate_t
{
	BAUD_300 = 0,							/* 300 baud    */
	BAUD_600,								/* 600 baud    */
	BAUD_1200,								/* 1200 baud   */
	BAUD_2400,								/* 2400 baud   */
	BAUD_4800,								/* 4800 baud   */
	BAUD_9600,								/* 9600 baud   */
	BAUD_19200,								/* 19200 baud  */
	BAUD_38400,								/* 38400 baud  */
	BAUD_57600,								/* 57600 baud  */
	BAUD_115200							/* 115200 baud */
} baud_rate_t;

/* Response time */
typedef enum tag_response_time_t
{
	TIME_20_MS = 0,		/* 20 ms */
	TIME_200_MS		/* 200 ms */
} response_time_t;

/* object definition type */
typedef struct tag_obj_def_t
{
	Unsigned16  class_id;					/* Class ID          */
	Unsigned8 logical_name[6];				/* The logical name  */
} obj_def_t;

/* register act mask type */
typedef struct tag_register_act_mask_t
{
	value_t   mask_name;					/* mask_name           */
	Unsigned8  *p_index_list;				/* index_list          */
	Unsigned8 nr_index_list;				/* size of  index_list */
} register_act_mask_t;

/* table cell definition type */
typedef struct tag_table_cell_def_t
{
	Unsigned16  class_id;					/* Class ID          */
	Unsigned8 logical_name[6];				/* The logical name  */
	Unsigned8 *p_group_E;					/* group_E_values    */
	Unsigned8  nr_group_E;					/* number of group_E */
	Integer8  attribute_index;				/* attribute_index   */
} table_cell_def_t;

/* mapping table type */
typedef struct tag_mapping_table_t
{
	Unsigned8  ref_table_id;				/* ref_table_id                           */
	Unsigned16 *p_entry_list;				/* first_entry or table_entries           */
	Unsigned8 nr_entry_list;				/* number of entries, 1 for first entry   */
} mapping_table_t;

/* image to activate info element type */
typedef struct tag_img_to_activate_info_t
{
	dyn_value_t  image_size;					/* image_size           (Unsigned 32)  */
	dyn_value_t  image_id;						/* image_identification (octect string)*/
	dyn_value_t  image_signature;				/* image_signature      (octect string)*/
} img_to_activate_info_t ;

/*  image transfer status type */
typedef enum tag_img_transfer_status_t
{
	IMG_TRANS_NOT_INIT = 0,					/* Image transfer not initiated  */
	IMG_TRANS_INIT,							/* Image transfer initiated      */
	IMG_VERIFY_INIT,						/* Image verification initiated  */
	IMG_VERIFY_SUCCESS,						/* Image verification successful */
	IMG_VERIFY_FAIL,						/* Image verification failed     */
	IMG_ACTIVATE_INIT,						/* Image activation initiated    */
	IMG_ACTIVATE_SUCCESS,					/* Image activation successful   */
	IMG_ACTIVATE_FAIL						/* Image activation failed       */
} img_transfer_status_t;

/* control_state type */
typedef enum tag_control_state_t
{
	CONTROL_DISCONNECT = 0,					/* Disconnected */
	CONTROL_CONNECT,						/* Connected    */
	CONTROL_RECONNECT_READY				/* Ready_for_reconnection */
} control_state_t;

/* control_mode type */
typedef enum tag_control_mode_t
{
	CONTROL_MODE_NONE = 0,					/* Always in 'connected' state */
	CONTROL_MODE_BCFG_DE,					/* Disconnection: Remote (b, c), manual (f), local (g) Reconnection: Remote (d), manual (e),  */
	CONTROL_MODE_BCFG_AE,					/* Disconnection: Remote (b, c), manual (f), local (g) Reconnection: Remote (a), manual (e)   */
	CONTROL_MODE_BCG_DE,					/* Disconnection: Remote (b, c), manual (-), local (g) Reconnection: Remote (d), manual (e)   */
	CONTROL_MODE_BCG_AE,					/* Disconnection: Remote (b, c), manual (-), local (g) Reconnection: Remote (a), manual (e)   */
	CONTROL_MODE_BCFG_DEH,					/* Disconnection: Remote (b, c), manual (f), local (g) Reconnection: Remote (d), manual (e), local (h)  */
	CONTROL_MODE_BCG_DEH					/* Disconnection: Remote (b, c), manual (-), local (g) Reconnection: Remote (d), manual (e), local (h)  */
} control_mode_t;

/* value_definition type  */
typedef struct tag_value_def_t
{
	Unsigned16  class_id;					/* Class ID          */
	Unsigned8 logical_name[6];				/* The logical name  */
	Integer8  attribute_index;				/* attribute_index   */
} value_def_t;

/* emergency profile type  */
typedef struct tag_eme_profile_t
{
	Unsigned16  eme_profile_id;				/* emergency_profile_id       */
	date_time_t *p_eme_act_time;			/* emergency_activation_time  */
	Unsigned32 eme_duration;				/*emergency_duration          */
} eme_profile_t;

/* working mode type*/
typedef enum tag_iec_working_mode_t
{
	IEC_INACTIVE = 0,				/* Line dedicated to the device */
	IEC_ACTIVE_ALWAYS				/* Shared line management, limited number of calls allowed            */
	/* Others (2...127) reserve */
	/* Others (128...250) manufacturer specific modes */
} iec_working_mode_t;

/* fatal error type*/
typedef enum tag_fatal_error_t
{
	ERROR_NONE = 0,							/* No-error */
	ERROR_T_EP_1F,							/* t-EP-1F  */
	ERROR_T_EP_2F,							/* t-EP-2F  */
	ERROR_T_EP_4F,							/* t-EP-4F  */
	ERROR_T_EP_5F,							/* t-EP-5F  */
	ERROR_ET_1F,							/* eT-1F    */
	ERROR_ET_2F,							/* eT-2F    */
	ERROR_E_EP_3F,							/* e-EP-3F  */
	ERROR_E_EP_4F,							/* e-EP-4F  */
	ERROR_E_EP_5F,							/* e-EP-5F  */
	ERROR_E_EL_2F,							/* e-EL-2F  */
} fatal_error_t;

/* initialization string element  */
typedef struct tag_init_string_element_t
{
	Unsigned8 * p_request;					/*  request              */
	Unsigned8 nr_request;					/*  size of request      */
	Unsigned8 * p_response;					/*  response             */
	Unsigned8 nr_response;					/*  size of response     */
	Unsigned16 * p_delay_after_response;	/*  delay_after_response */
} init_string_element_t;

/* working mode type*/
typedef enum tag_working_mode_t
{
	WORK_MODE_DEDICATED = 0,				/* Line dedicated to the device */
	WORK_MODE_SHARE_CALLS,					/* Shared line management, limited number of calls allowed            */
	WORK_MODE_SHARE_SUCCESS_CALLS,			/* Shared line management, limited number of successful calls allowed */
	WORK_MODE_NO_CONNECT					/* Currently no modem connected */
	/* Others (200...255) manufacturer specific modes */
} working_mode_t;

/* window element type  */
typedef struct tag_window_element_t
{
	date_time_t * p_start_time;				/* start time */
	date_time_t * p_end_time;				/* end time   */
} window_element_t;

/* windows_status_t*/
typedef enum tag_windows_status_t
{
	WINDOWS_INACTIVE = 0,					/* Inactive  */
	WINDOWS_ACTIVE,							/* Active    */
	WINDOWS_LOCKED							/* Locked    */
} windows_status_t;

/* nr_rings_t type  */
typedef struct tag_nr_rings_t
{
	Unsigned8 nr_rings_in_window;			/* nr_rings_in_window     */
	Unsigned8 nr_rings_out_of_window;		/* nr_rings_out_of_window */
} nr_rings_t;

/* call type*/
typedef enum tag_call_type_t
{
	CALL_NORMAL = 0,						/* normal CSD call */
	CALL_WAKEUP							/* wake-up request */
} call_type_t;

/* allowed_callers_element type  */
typedef struct tag_allowed_callers_element_t
{
	Unsigned8 * p_caller_id;				/* caller_id        */
	Unsigned8 caller_id_len;				/* caller_id length */
	call_type_t call_type;					/* call_type */
} allowed_callers_element_t;

/* Dial mode type*/
typedef enum tag_dial_mode_t
{
	DIAL_MODE_NO_AUTO = 0,					/* no auto dialling */
	DIAL_MODE_AUTO_ANYTIME,					/* auto dialling allowed anytime */
	DIAL_MODE_AUTO_WITHIN_TIME,				/* auto dialling allowed within the validity time of the calling window */
	DIAL_MODE_AUTO_REGULAR,					/* �regular� auto dialling allowed within the validity time; �alarm� initiated auto dialling allowed anytime  */
	DIAL_MODE_AUTO_SMS_PLMN,				/* SMS sending via Public Land Mobile Network (PLMN) */
	DIAL_MODE_AUTO_SMS_PSTN,				/* SMS sending via PSTN */
	DIAL_MODE_EMAIL						/* email sending */
	/* Others (200...255) manufacturer specific modes */
} dial_mode_t;


/* ZigBee Stack Profile type */
typedef enum tag_zigbee_stack_profile_t
{
	ZIGBEE = 1,
	ZIGBEE_PRO
} zigbee_stack_profile_t;

/* ZigBee Device Status type */
typedef union tag_devices_status_t
{
	Unsigned8 byte;
	struct
	{
		Unsigned8 reserved:3;
		Unsigned8 SEP_transmitting:1;
		Unsigned8 authorised_after_swapped_out:1;
		Unsigned8 unauthorised_on_PAN_reported:1;
		Unsigned8 active_reporting_PAN:1;
		Unsigned8 authorised_on_PAN:1;
	}bits;
} zigbee_device_status_t;

/* ZigBee Active Device type */
typedef struct tag_active_devices_t
{
	Unsigned8 				mac_address[8];
	zigbee_device_status_t 	status;
	Integer8 				max_RSSI;
	Integer8 				average_RSSI;
	Integer8 				min_RSSI;
	Unsigned8 				max_LQI;
	Unsigned8 				average_LQI;
	Unsigned8 				min_LQI;
	date_time_t 			last_comm_date_time;
	Unsigned8 				number_of_hops;
	Unsigned8 				transmission_failures;
	Unsigned8 				transmission_successes;
	Unsigned8 				application_version;
	Unsigned8 				stack_version;
} zigbee_active_devices_t;

/* ZigBee Device type type */
typedef enum tag_device_type_t
{
	ELECTRICITY_METER = 0,
	GAS_METER,
	WATER_METER,
	THERMAL_METER,
	PRESSURE_METER,
	HEAT_METER,
	COOLING_METER,
	ELECTRIC_VEHICLE_CHARGING_METER,
	PV_GENERATION_METER,
	WIND_TURBINE_GENERATION_METER,
	WATER_TURBINE_GENERATION_METER,
	MICRO_GENERATION_METER,
	SOLAR_HOT_WATER_GENERATION_METER,
	ZIGBEE_CONTROLLED_LOAD_SWITCH,
	ZIGBEE_BASED_BOOST_BUTTON = 14,
	IHD = 128,
	RANGE_EXTENDER,
	CONSUMER_ACCESS_DEVICE,
	THERMOSTAT,
	PREPAYMENT_TERMINAL,
	ZIGBEE_CONTROLLED_LOAD_SWITCH_SUB,
	ZIGBEE_BASED_BOOST_BUTTON_SUB = 134
} zigbee_device_type_t;

/* ZigBee Key type type */
typedef enum tag_key_type_t
{
	PRECONFIGURED_KEY = 0,
	INSTALL_CODE
} zigbee_key_type_t;

/* ZigBee Register Device type */
typedef struct tag_register_device_t
{
	Unsigned8 				ieee_address[8];
	zigbee_key_type_t 		key_type;
	Unsigned8 				key[16];
	zigbee_device_type_t 	device_type;
} zigbee_register_device_t;

/* ZigBee Device to Backup  type */
typedef struct tag_device_to_backup_t
{
	Unsigned8 mac_address[8];
	Unsigned8 hashed_TC_link_key[16];
} zigbee_device_to_backup_t;

/*ZigBee Mirror Control type*/
typedef union tag_mirror_control_t
{
	struct
	{
		Unsigned8 bytes0;
		Unsigned8 bytes1;
	} bytes;
	struct
	{
		Unsigned8 clear_logs:1;									/* Byte07 */
		Unsigned8 clear_billing_info:1;							/* Byte06 */
		Unsigned8 clear_MPAN:1;									/* Byte05 */
		Unsigned8 clear_zigbee_attributes:1;					/* Byte04 */
		Unsigned8 clear_demand_maxdemand_register:1;			/* Byte03 */
		Unsigned8 clear_consumption_register_indexes:1;			/* Byte02 */
		Unsigned8 clear_all_mirror_data:1;						/* Byte01 */
		Unsigned8 force_gas_meter_removal:1;					/* Byte00 */

		Unsigned8 action_all:1;									/* Byte17 */
		Unsigned8 reserved:6;									/* Byte16-11 */
		Unsigned8 clear_OTA_firmware_waiting:1;					/* Byte10 */
	} bits;
} zigbee_mirror_control_t;

/* Request type enumeration (GET/SET/ACTION) */
typedef enum tag_req_type_t
{
	REQ_TYPE_GET    = 0,	/* GET request     */
	REQ_TYPE_SET    = 1,    /* SET request     */
	REQ_TYPE_ACTION = 2    /* ACTION request  */
} req_type_t;

/* Service type enumeration (NORMAL/BLOCK/LIST) */
typedef enum tag_service_type_t
{
	SERVICE_TYPE_NORMAL  = 1,	/* NORMAL request     */
	SERVICE_TYPE_BLOCK   = 2,   /* BLOCK request      */
	SERVICE_TYPE_LIST    = 3   /* WITH-LIST request  */
} service_type_t;

/* A function pointer type for Distributor */
typedef Unsigned8 (*fp_distributor_t)(
	Unsigned16  		master_index,	/* [In] Id of the row in master list */
	Unsigned16  		child_index,	/* [In] Id of the row in child list */
	req_type_t			req_type,		/* [In] Request type (GET/SET/ACTION) */
	service_type_t		service_type,	/* [In] Service type (NORMAL/BLOCK/LIST) */
	st_Cosem_Attr_Desc	*p_descriptor,	/* [In] COSEM object descriptor */
	Unsigned8			*pdata,			/* [In] Data in case of SET/ACTION */
	Unsigned8			*p_out_buf,		/* [Out] Output buffer for encoded data */
	Unsigned16			*p_out_len		/* [Out] Output buffer length for encoded data */
);

/******************************************************************************
Variable Externs
******************************************************************************/

/******************************************************************************
Functions Prototypes
******************************************************************************/

/*
 * Async processing functions for object_list
 */

/* Restart */
void R_OBIS_AsyncRestart(void);

/* Data Update */
void R_OBIS_IntervalProcessing(void);

/*
 * Common functions for distribute attribute
 */

/* Distribute attribute value by internal memory */
Unsigned8 R_OBIS_DistributeByMem(
	attr_type_t 		attr_type,				/* Type */
	Integer16			attr_size,				/* Size */
	void 				*attr_buffer,			/* Buffer of value */
	req_type_t			req_type, 				/* Request type */
	Unsigned8			*p_is_block_transfer,	/* Buffer indicate use block transfer or not */
	Unsigned8			*pdata,					/* Data from COSEM Appl layer */
	Unsigned8			*p_out_buf,				/* Data pointer for get */
	Unsigned16			*p_out_len				/* Data length pointer for get */
);

/* Distribute attribute value by user function */
Unsigned8 R_OBIS_DistributeByUserFunc(
	Unsigned16  		child_index,			/* Child ID */
	req_type_t			req_type,				/* Request type */
	service_type_t		service_type,			/* Service type */
	Unsigned8			*p_is_block_transfer,	/* Buffer indicate use block transfer or not */
	st_Cosem_Attr_Desc	*p_descriptor,			/* Pointer to COSEM descriptor */
	Unsigned8			*pdata,					/* Data from COSEM Appl layer */
	Unsigned8			*p_out_buf,				/* Data pointer for get */
	Unsigned16			*p_out_len				/* Data length pointer for get */
);

/* Distribute attribute value by access to EEPROM */
Unsigned8 R_OBIS_DistributeByE2PR(
	attr_access_t		*p_attr_access,			/* Access method option			*/
	attr_type_t 		attr_type,				/* Type */
	Integer16			attr_size,				/* Size */
	req_type_t			req_type,				/* Request type */
	Unsigned8			*p_is_block_transfer,	/* Buffer indicate use block transfer or not */
	Unsigned8			*pdata,					/* Data from COSEM Appl layer */
	Unsigned8			*p_out_buf,				/* Data pointer for get */
	Unsigned16			*p_out_len				/* Data length pointer for get */
);

/* Distribute attribute value */
Unsigned8 R_OBIS_DistributeAttrBuffer(
	attr_access_t *p_attr_access,				/* Access method option */
	attr_type_t 		attr_type,				/* Type */
	Integer16			attr_size,				/* Size */
	void 				*attr_buffer,			/* Buffer of value */
	Unsigned16  		child_index,			/* Child ID */
	req_type_t			req_type,				/* Request type */
	service_type_t		service_type,			/* Service type */
	st_Cosem_Attr_Desc	*p_descriptor,			/* Pointer to COSEM descriptor */
	Unsigned8			*pdata,					/* Data from COSEM Appl layer */
	Unsigned8			*p_out_buf,				/* Data pointer for get */
	Unsigned16			*p_out_len				/* Data length pointer for get */
);

/*
 * Common functions for data encoding
 */

/* Encode attribute value */
Unsigned16 R_OBIS_EncodeAttribute(
	Unsigned8 	*buf,
	Unsigned16 	buf_len,
	attr_type_t	attr_type,
	Unsigned8	*value_buf,
	Unsigned16 	value_buf_len
);

/* Decode attribute value */
Unsigned16 R_OBIS_DecodeAttribute(
	Unsigned8	*value_buf,
	Unsigned16 	value_buf_len,
	attr_type_t	attr_type,
	Unsigned8 	*buf,
	Unsigned16 	buf_len
);

/* Encode scaler_unit */
Unsigned16 R_OBIS_EncodeScalerUnit(
    Unsigned8   	*buf,
    Unsigned16  	buf_len,
    scaler_unit_t   *p_scaler_unit
);

/* Encode buffer_t */
Unsigned8 R_OBIS_EncodeGenericBuffer(
    buffer_t            *buf,						/* [In] Pointer to buffer_t                       */
    Unsigned8           b_selective_access,			/* [In] selective_access                          */
    Unsigned16          master_index,				/* [In] Id of the row in master list              */
    Unsigned16          child_index,				/* [In] Id of the row in child list               */
    Unsigned8           *p_selector_buffer,			/* [In] Pointer to the selector (from Client)     */
    service_type_t      service_type,				/* [In] Service type                              */
    Unsigned8           *p_out_buf,					/* [Out] Data pointer for get                     */
    Unsigned16          *p_out_len					/* [Out] Data length pointer for get              */
);

/*
 * Distributor for all classes
 */

/* Distributor for Data class */
Unsigned8 R_OBIS_DistributeDataClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
);

/* Distributor for Register class */
Unsigned8 R_OBIS_DistributeRegisterClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
);

/* Distributor for XRegister class */
Unsigned8 R_OBIS_DistributeXRegisterClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
);

/* Distributor for DRegister class */
Unsigned8 R_OBIS_DistributeDRegisterClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
);

/* Distributor for Register Activation class */
Unsigned8 R_OBIS_DistributeRegisterActivationClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
);

/* Distributor for Profile generic class */
Unsigned8 R_OBIS_DistributeProfileGenericClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
);

/* Distributor for Clock class */
Unsigned8 R_OBIS_DistributeClockClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
);

/* Distributor for Script Table class */
Unsigned8 R_OBIS_DistributeScriptTableClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
);

/* Distributor for Schedule class */
Unsigned8 R_OBIS_DistributeScheduleClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
);

/* Distributor for Special Days Table class */
Unsigned8 R_OBIS_DistributeSpecialDaysTableClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
);

/* Distributor for Association LN class */
Unsigned8 R_OBIS_DistributeAssociationLNClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
);

/* Distributor for SAP assignment class */
Unsigned8 R_OBIS_DistributeSAPassignmentClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
);

/* Distributor for Image Transfer class */
Unsigned8  R_OBIS_DistributeImageTransferClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
);

/* Distributor for IEC Local Port Setup */
Unsigned8 R_OBIS_DistributeIECLocalPortSetupClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
);

/* Distributor for Activity Calendar class */
Unsigned8 R_OBIS_DistributeActivityCalendarClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
);

/* Distributor for Register Monitor class */
Unsigned8 R_OBIS_DistributeRegisterMonitorClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
);

/* Distributor for Single Action Schedule */
Unsigned8 R_OBIS_DistributeSingleActionScheduleClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
);

/* Distributor for IEC HDLC Setup */
Unsigned8 R_OBIS_DistributeIECHDLCSetupClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
);

/* Distributor for IEC Twisted Pair Setup */
Unsigned8 R_OBIS_DistributeIECTwistedPairClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
);

/* Distributor for Utility Tables class */
Unsigned8 R_OBIS_DistributeUtilityTablesClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
);

/* Distributor for Modem Config class */
Unsigned8 R_OBIS_DistributeModemConfigClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
);

/* Distributor for Auto Answer class */
Unsigned8 R_OBIS_DistributeAutoAnswerClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
);

/* Distributor for Auto Connect class */
Unsigned8 R_OBIS_DistributeAutoConnectClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
);

/* Distributor for TCP UDP setup class */
Unsigned8 R_OBIS_DistributeTCPUDPSetupClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
);

/* Distributor for Register Table class */
Unsigned8 R_OBIS_DistributeRegisterTableClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
);

/* Distributor for Status Mapping class */
Unsigned8 R_OBIS_DistributeStatusMappingClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
);

/* Distributor for Security Setup class */
Unsigned8 R_OBIS_DistributeSecuritySetupClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
);

/* Distributor for Disconnect Control class */
Unsigned8 R_OBIS_DistributeDisconnectControlClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
);

/* Distributor for Limiter class */
Unsigned8 R_OBIS_DistributeLimiterClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
);

/* Distributor for ZigBee SAS Startup class */
Unsigned8 R_OBIS_DistributeZigBeeSASStartupClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
);

/* Distributor for ZigBee SAS Join class */
Unsigned8 R_OBIS_DistributeZigBeeSASJoinClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
);

/* Distributor for ZigBee SAS APS Fragment class */
Unsigned8 R_OBIS_DistributeZigBeeSASAPSFragmentClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
);

/* Distributor for ZigBee Network Control class */
Unsigned8 R_OBIS_DistributeZigBeeNetworkControlClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
);

/* Distributor for ZigBee Tunnel Setup class */
Unsigned8 R_OBIS_DistributeZigBeeTunnelSetupClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
);
void Set_PP(unsigned int , unsigned char );
#endif /* _R_DLMS_OBIS_IC_H */

