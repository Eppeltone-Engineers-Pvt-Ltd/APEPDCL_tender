/******************************************************************************
  Copyright (C) 2011 Renesas Electronics Corporation, All Rights Reserved.
*******************************************************************************
* File Name    : format.h
* Version      : 1.00
* Description  : Storage Format Header file
* Device       : EEPROM, Atmel AT24C32, 4KBytes
* Note         : NEED TO CHANGE THIS FILE WHEN
*              :   . EM CORE TYPE DEFINITION IS CHANGED, OR
*              :   . EEPROM START ADDRESS IS CHANGED
* Caution      : DATA ALIGNMENT
*              :    DATA ALIGNMENT ON THIS FORMAT IS 2-BYTES ALIGNMENT, EVEN ADDRESS.
*              :    PLEASE DON'T CHANGE TO OTHER ALIGNMENT SETTING.
*              :    WHEN CHANGE THE DATA ALIGNMENT SETTING, IT WILL DAMAGE THE FORMAT
*              :    ON EEPROM
*              :
*              : BIT FIELD FORMAT
*              :    BIT FIELD FORMAT IS FROM LSB FIRST
*              :    PLEASE DON'T CHANGE THIS SETTING ON THE COMPILE OPTION (IF SUPPORTED)
******************************************************************************
* History : DD.MM.YYYY Version Description
******************************************************************************/

#ifndef _STORAGE_EEPROM_FORMAT_H
#define _STORAGE_EEPROM_FORMAT_H

/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#include "..\\..\\rlDevice\\include\\dTypedef.h"

/******************************************************************************
Typedef definitions
******************************************************************************/

/* EEPROM Storage Header */
typedef struct tagStorageEEPROMHeader
{
	/* Total: 24 + 6 Bytes */
	uint32_t	header_code;				/* (4 bytes) Header Code (EM Formatted Header Code)  */
	uint16_t	usage;                      /* (2 bytes) Usage (number of bytes)                 */
	uint16_t	num_of_config;              /* (2 bytes) Number of EM Configuration bytes        */
	uint16_t	num_of_calib;               /* (2 bytes) Number of EM Calibration bytes          */
	uint16_t	num_of_sys_state;           /* (2 bytes) Number of EM System state bytes         */
	uint16_t	num_of_rtc_backup;          /* (2 bytes) Number of RTC Backup bytes              */
	uint16_t	num_of_platform_info;       /* (2 bytes) Number of EM Platform Information       */
	uint16_t	num_of_energy_log;          /* (2 bytes) Number of EM Energy Data Log Record     */
	uint16_t	num_of_tamper_log;          /* (2 bytes) Number of EM Tamper Data Log Record     */
	uint16_t	cur_energy_log_index;		/* (2 bytes) Current Energy Log Index                */
	uint16_t	cur_tamper_log_index;		/* (2 bytes) Current Tamper Log Index                */

	uint32_t	power_on_min;				/* (4 bytes) Power on duration in minute             */
	uint16_t	program_cnt;				/* (2 bytes) Programming counter                     */
	
} STORAGE_EEPROM_HEADER;

/* EEPROM CRC Code */
typedef struct tagStorageEEPROMCRC
{
	/* Total: 2 Bytes */
	uint16_t	calibration;				/* (2 bytes) CRC Code for EM Calib                   */

} STORAGE_EEPROM_CRC;

/* RTC Date Time */
typedef struct tagRTCDateTime
{
	/* Total: 8 bytes */
	uint8_t	Sec;		/* Second */
	uint8_t	Min;		/* Minute */
	uint8_t	Hour;		/* Hour */
	uint8_t	Day;		/* Day */
	uint8_t	Week;		/* Day of week */
	uint8_t	Month;		/* Month */
	uint8_t	Year;		/* Year (ony 2 ending digit) */
	
	uint8_t	reserved;	/* Reserved Byte (Padding) - NO USE */
	
} RTC_DATE_TIME;

/* Platform Information */
typedef struct tagPlatformUserInfo
{
	/* Total: 24 bytes */
	uint8_t				unit_code[8];			/* Unit code */
	uint8_t				type_of_meter[4];		/* Type of meter */
	RTC_DATE_TIME		manufacture_date;		/* Manufacture date */
	uint16_t			number_of_reset;		/* Number of reset */
	uint16_t			lcd_scroll_period;		/* LCD Scrolling period */
	uint8_t				tamper_indicator;		/* Tamper Indicator */
	
	uint8_t				reserved;				/* Reserved Byte (Padding) - NO USE */
	
} PLATFORM_INFO;

/* One Month Energy Data Log - DLMS */
typedef struct tagOneMonthEnergyDataLog
{
	/* Total: 72 + 8 bytes */
	struct
	{	
		uint32_t	active;						/* Active */
		uint32_t	apparent;					/* Apparent */
	} present_max_demand;					/* Present max demand */
	
	struct
	{
		int32_t	active;						/* Active */
		int32_t	apparent;					/* Apparent */
	} present_energy_total;					/* Present energy total */
	
	struct
	{
		int32_t	active;						/* Active */
		int32_t	apparent;					/* Apparent */
	} present_energy_tariff[6];				/* Present energy tariff (6 tariffs) */

	int32_t power_factor;
	int32_t total_powerON_duration;
	RTC_DATE_TIME	present_rtc_log;		/* Present RTC monthly log */
	
} ONE_MONTH_ENERGY_DATA_LOG;

/* One Daily Energy Data Log - DLMS */
typedef struct tagOneDailyEnergyDataLog
{
	/* Total: 8 + 8 bytes */
	struct
	{
		int32_t	active;						/* Active */
		int32_t	apparent;					/* Apparent */
	} present_energy_total;					/* Present energy total */

	RTC_DATE_TIME	present_rtc_log;		/* Present RTC monthly log */
	
} ONE_DAILY_ENERGY_DATA_LOG;

/* One Block load Energy Data Log - DLMS */
typedef struct tagOneBlockEnergyDataLog
{
	/* Total: 12 + 8 bytes */
	struct
	{
		int32_t	active;						/* Active */
		int32_t	apparent;					/* Apparent */
	} present_energy_total;					/* Present energy total */

	int32_t voltage;
	RTC_DATE_TIME	present_rtc_log;		/* Present RTC monthly log */
	
} ONE_BLOCK_ENERGY_DATA_LOG;


/* One Event Energy Data Log - DLMS */
typedef struct tagOneEventEnergyDataLog
{
	/* Total: 16 + 8 + 2 bytes */
	int32_t	current;
	int32_t	voltage;
	int32_t	power_factor;
	int32_t	active;						/* Active */
	RTC_DATE_TIME	present_rtc_log;		/* Present RTC monthly log */
	uint16_t	eventID;
} ONE_EVENT_ENERGY_DATA_LOG;

/* One Tamper Data Log
 * Caution:
 *   On the CubeSuite 1.5d / Compile Option / Data Control,
 *   The bit field format is now start from LSB bit first,
 *   When change the "Assign bit field structure from MSB"
 *   to "Yes", the bit order of (tamper_type) in below structure
 *   is changed to MSB frist, NOT CORRECT for storage format.
 *   SO, PLEASE DON'T CHANGE IT TO YES.
 */
typedef struct tagOneTamperDataLog
{
	/* Total: 10 bytes*/ 
	RTC_DATE_TIME	tamper_rtc_time;		/* Tamper RTC time */
	
	struct
	{
		uint8_t	reverse_current	:1;			/* (LSB Bit, Bit 0) - Reverse Current Indicator Bit */	
		uint8_t	earth_connected	:1;			/* (         Bit 1) - Earth Connected Indicator Bit */
		uint8_t	neutral_missing	:1;			/* (         Bit 2) - Neutral missing Indicator Bit */
		uint8_t	case_open	    :1;			/* (         Bit 3) - Case Open Indicator Bit       */
		uint8_t	magnet	        :1;			/* (         Bit 4) - Magnet Indicator Bit          */
		uint8_t	reserved		:3;			/* (Reserved Bits ) - (NO USE) */
		
	} tamper_type;							/* Tamper Type */
	
	uint8_t reserved;					/* Reserved Byte (Padding) - NO USE */
	
} ONE_TAMPER_DATA_LOG;

/******************************************************************************
Macro definitions
******************************************************************************/

/* EEPROM Information */
#define STORAGE_EEPROM_START_ADDR						EPR_DEVICE_START_ADDR	/* EEPROM Start address */
#define STORAGE_EEPROM_SIZE								EPR_DEVICE_SIZE			/* EEPROM Size */
#define STORAGE_EEPROM_PAGESIZE							EPR_DEVICE_PAGESIZE		/* EEPROM Page Size */

/* EEPROM Storage Type */
#define STORAGE_EEPROM_TYPE_INTEGER8					0		/* uint8_t            */
#define STORAGE_EEPROM_TYPE_ARRAY_INTEGER8				1		/* array of uint8_t   */
#define STORAGE_EEPROM_TYPE_INTEGER16					2		/* uint16_t	          */
#define STORAGE_EEPROM_TYPE_ARRAY_INTEGER16				3		/* array of uint16_t  */
#define STORAGE_EEPROM_TYPE_INTEGER32					4		/* uint32_t           */
#define STORAGE_EEPROM_TYPE_ARRAY_INTEGER32				5		/* array of uint32_t  */
#define STORAGE_EEPROM_TYPE_FLOAT32						6		/* float32_t          */
#define STORAGE_EEPROM_TYPE_ARRAY_FLOAT32				7		/* array of float32_t */
#define STORAGE_EEPROM_TYPE_STRUCT						8		/* (struct)           */
#define STORAGE_EEPROM_TYPE_ARRAY_STRUCT				9		/* (array of struct)  */

/* EEPROM Group Enable/Disable Macro */
#define STORAGE_EEPROM_HEADER_GROUP						1
#define STORAGE_EEPROM_CRC_GROUP						1
#define STORAGE_EEPROM_CONFIG_GROUP						1
#define STORAGE_EEPROM_CALIB_GROUP						1
#define STORAGE_EEPROM_RTC_TIME_BACKUP_GROUP			1
#define STORAGE_EEPROM_PLATFORM_INFO_GROUP				1
#define STORAGE_EEPROM_SYSTEM_STATE_GROUP				1
#define STORAGE_EEPROM_ENERGY_DATA_LOG_GROUP			1
#define STORAGE_EEPROM_TAMPER_DATA_LOG_GROUP			1
// DLMS - adding
#define STORAGE_EEPROM_DLMS_SECRET_GROUP				1
#define STORAGE_EEPROM_BLOCK_DATA_LOG_GROUP				1
#define STORAGE_EEPROM_DAILY_DATA_LOG_GROUP				1
#define STORAGE_EEPROM_EVENT_DATA_LOG_GROUP				1

//#define BLOCKLOAD_MAX_ENTRY			20		/* Max number of entries */
//#define DAILYLOAD_MAX_ENTRY			32		/* Max number of entries */
//#define BILLING_MAX_ENTRY			12		/* Max number of entries */

//#define EVENT_MAX_PROFILE			5		/* Max index of profile objects (from 0 ~ max) */
//#define EVENT_MAX_ENTRY				10		/* Max number of entries */

/* Recording interval 1, for blockload profile (in second) */
//#define BLOCKLOAD_INTERVAL			15
//#define DAILYLOAD_INTERVAL			86400 	/* 24 h*/
//#define BILLING_INTERVAL			2592000	/* 1 month */

/* EEPROM Header Structure Location + Size
 * 11 Macro Items */
#if STORAGE_EEPROM_HEADER_GROUP

#define	STORAGE_EEPROM_HEADER_ADDR						(STORAGE_EEPROM_START_ADDR + 0x0000)									/* Offset 0x0000 */
#define STORAGE_EEPROM_HEADER_SIZE						(sizeof(STORAGE_EEPROM_HEADER))											/* 30 Bytes */
#define STORAGE_EEPROM_HEADER_TYPE						STORAGE_EEPROM_TYPE_STRUCT												/* Structure */
	
	/* Header Code */
	#define STORAGE_EEPROM_HEADER_HEADER_CODE_ADRR			(STORAGE_EEPROM_HEADER_ADDR                   + 0)					/* First Element */
	#define STORAGE_EEPROM_HEADER_HEADER_CODE_SIZE			(sizeof(uint32_t))
	#define STORAGE_EEPROM_HEADER_HEADER_CODE_TYPE			STORAGE_EEPROM_TYPE_INTEGER32

	/* Usage */
	#define STORAGE_EEPROM_HEADER_USAGE_ADRR				(STORAGE_EEPROM_HEADER_HEADER_CODE_ADRR       + STORAGE_EEPROM_HEADER_HEADER_CODE_SIZE)
	#define STORAGE_EEPROM_HEADER_USAGE_SIZE				(sizeof(uint16_t))
	#define STORAGE_EEPROM_HEADER_USAGE_TYPE				STORAGE_EEPROM_TYPE_INTEGER16

	/* Number of configuration bytes */
	#define STORAGE_EEPROM_HEADER_NUM_CONFIG_ADRR			(STORAGE_EEPROM_HEADER_USAGE_ADRR             + STORAGE_EEPROM_HEADER_USAGE_SIZE)
	#define STORAGE_EEPROM_HEADER_NUM_CONFIG_SIZE			(sizeof(uint16_t))
	#define STORAGE_EEPROM_HEADER_NUM_CONFIG_TYPE			STORAGE_EEPROM_TYPE_INTEGER16

	/* Number of calibration bytes */
	#define STORAGE_EEPROM_HEADER_NUM_CALIB_ADRR			(STORAGE_EEPROM_HEADER_NUM_CONFIG_ADRR        + STORAGE_EEPROM_HEADER_NUM_CONFIG_SIZE)
	#define STORAGE_EEPROM_HEADER_NUM_CALIB_SIZE			(sizeof(uint16_t))
	#define STORAGE_EEPROM_HEADER_NUM_CALIB_TYPE			STORAGE_EEPROM_TYPE_INTEGER16

	/* Number of system state bytes */
	#define STORAGE_EEPROM_HEADER_NUM_SYSTEM_STATE_ADRR		(STORAGE_EEPROM_HEADER_NUM_CALIB_ADRR         + STORAGE_EEPROM_HEADER_NUM_CALIB_SIZE)
	#define STORAGE_EEPROM_HEADER_NUM_SYSTEM_STATE_SIZE		(sizeof(uint16_t))
	#define STORAGE_EEPROM_HEADER_NUM_SYSTEM_STATE_TYPE		STORAGE_EEPROM_TYPE_INTEGER16

	/* Number of RTC Backup bytes */
	#define STORAGE_EEPROM_HEADER_NUM_RTC_BACKUP_ADRR		(STORAGE_EEPROM_HEADER_NUM_SYSTEM_STATE_ADRR  + STORAGE_EEPROM_HEADER_NUM_SYSTEM_STATE_SIZE)
	#define STORAGE_EEPROM_HEADER_NUM_RTC_BACKUP_SIZE		(sizeof(uint16_t))
	#define STORAGE_EEPROM_HEADER_NUM_RTC_BACKUP_TYPE		STORAGE_EEPROM_TYPE_INTEGER16
	
	/* Number of plaform information bytes */
	#define STORAGE_EEPROM_HEADER_NUM_PLATFORM_INFO_ADRR	(STORAGE_EEPROM_HEADER_NUM_RTC_BACKUP_ADRR    + STORAGE_EEPROM_HEADER_NUM_RTC_BACKUP_SIZE)
	#define STORAGE_EEPROM_HEADER_NUM_PLATFORM_INFO_SIZE	(sizeof(uint16_t))
	#define STORAGE_EEPROM_HEADER_NUM_PLATFORM_INFO_TYPE	STORAGE_EEPROM_TYPE_INTEGER16

	/* Number of energy log record */
	#define STORAGE_EEPROM_HEADER_NUM_ENERGY_LOG_ADRR		(STORAGE_EEPROM_HEADER_NUM_PLATFORM_INFO_ADRR + STORAGE_EEPROM_HEADER_NUM_PLATFORM_INFO_SIZE)
	#define STORAGE_EEPROM_HEADER_NUM_ENERGY_LOG_SIZE		(sizeof(uint16_t))
	#define STORAGE_EEPROM_HEADER_NUM_ENERGY_LOG_TYPE		STORAGE_EEPROM_TYPE_INTEGER16

	/* Number of tamper log record */
	#define STORAGE_EEPROM_HEADER_NUM_TAMPER_LOG_ADRR		(STORAGE_EEPROM_HEADER_NUM_ENERGY_LOG_ADRR    + STORAGE_EEPROM_HEADER_NUM_ENERGY_LOG_SIZE)
	#define STORAGE_EEPROM_HEADER_NUM_TAMPER_LOG_SIZE		(sizeof(uint16_t))
	#define STORAGE_EEPROM_HEADER_NUM_TAMPER_LOG_TYPE		STORAGE_EEPROM_TYPE_INTEGER16

	/* Current energy log index */
	#define STORAGE_EEPROM_HEADER_ENERGY_LOG_INDEX_ADRR		(STORAGE_EEPROM_HEADER_NUM_TAMPER_LOG_ADRR    + STORAGE_EEPROM_HEADER_NUM_TAMPER_LOG_SIZE)
	#define STORAGE_EEPROM_HEADER_ENERGY_LOG_INDEX_SIZE		(sizeof(uint16_t))
	#define STORAGE_EEPROM_HEADER_ENERGY_LOG_INDEX_TYPE		STORAGE_EEPROM_TYPE_INTEGER16

	/* Current tamper log index */
	#define STORAGE_EEPROM_HEADER_TAMPER_LOG_INDEX_ADRR		(STORAGE_EEPROM_HEADER_ENERGY_LOG_INDEX_ADRR  + STORAGE_EEPROM_HEADER_ENERGY_LOG_INDEX_SIZE)
	#define STORAGE_EEPROM_HEADER_TAMPER_LOG_INDEX_SIZE		(sizeof(uint16_t))
	#define STORAGE_EEPROM_HEADER_TAMPER_LOG_INDEX_TYPE		STORAGE_EEPROM_TYPE_INTEGER16

	/* Power on duration in minute */
	#define STORAGE_EEPROM_HEADER_POWER_ON_DURATION_ADRR		(STORAGE_EEPROM_HEADER_TAMPER_LOG_INDEX_ADRR    + STORAGE_EEPROM_HEADER_TAMPER_LOG_INDEX_SIZE)
	#define STORAGE_EEPROM_HEADER_POWER_ON_DURATION_SIZE		(sizeof(uint32_t))
	#define STORAGE_EEPROM_HEADER_POWER_ON_DURATION_TYPE		STORAGE_EEPROM_TYPE_INTEGER32

	/* Programming counter */
	#define STORAGE_EEPROM_HEADER_PROGRAMMING_COUNTER_ADRR		(STORAGE_EEPROM_HEADER_POWER_ON_DURATION_ADRR    + STORAGE_EEPROM_HEADER_POWER_ON_DURATION_SIZE)
	#define STORAGE_EEPROM_HEADER_PROGRAMMING_COUNTER_SIZE		(sizeof(uint16_t))
	#define STORAGE_EEPROM_HEADER_PROGRAMMING_COUNTER_TYPE		STORAGE_EEPROM_TYPE_INTEGER16

#endif /* STORAGE_EEPROM_HEADER_GROUP */

/* EEPROM CRC Structure Location + Size
 * 1 Macro Items */
#if STORAGE_EEPROM_CRC_GROUP

#define	STORAGE_EEPROM_CRC_ADDR							(STORAGE_EEPROM_START_ADDR + 0x0022)									/* Offset 0x0022 */
#define STORAGE_EEPROM_CRC_SIZE							(sizeof(STORAGE_EEPROM_CRC))											/* 2 Bytes */
#define STORAGE_EEPROM_CRC_TYPE							STORAGE_EEPROM_TYPE_STRUCT												/* Structure */
	
	/* Calibration */
	#define STORAGE_EEPROM_CRC_CALIBRATION_ADRR			(STORAGE_EEPROM_CRC_ADDR                   	+ 0)						/* First Element */
	#define STORAGE_EEPROM_CRC_CALIBRATION_SIZE			(sizeof(uint16_t))
	#define STORAGE_EEPROM_CRC_CALIBRATION_TYPE			STORAGE_EEPROM_TYPE_INTEGER16

#endif /* STORAGE_EEPROM_CRC_GROUP */

/* EEPROM EM Core Structure location + size
 * 11 Macro Items */
#if STORAGE_EEPROM_CONFIG_GROUP

#define STORAGE_EEPROM_CONFIG_ADDR						(STORAGE_EEPROM_START_ADDR + 0x002A)									/* Offset 0x002A */
#define STORAGE_EEPROM_CONFIG_SIZE						(sizeof(EM_CONFIG) - sizeof(EM_TIME_SLOT *)   + sizeof(EM_TIME_SLOT)*6)	/* 38 Bytes */
#define STORAGE_EEPROM_CONFIG_TYPE						STORAGE_EEPROM_TYPE_STRUCT												/* Structure */

	/* Voltage low */
	#define STORAGE_EEPROM_CONFIG_VOLTAGE_LOW_ADDR			(STORAGE_EEPROM_CONFIG_ADDR                   + 0)					/* First Element */
	#define STORAGE_EEPROM_CONFIG_VOLTAGE_LOW_SIZE			(sizeof(uint16_t))
	#define STORAGE_EEPROM_CONFIG_VOLTAGE_LOW_TYPE			STORAGE_EEPROM_TYPE_INTEGER16

	/* Voltage high */
	#define STORAGE_EEPROM_CONFIG_VOLTAGE_HIGH_ADDR			(STORAGE_EEPROM_CONFIG_VOLTAGE_LOW_ADDR       + STORAGE_EEPROM_CONFIG_VOLTAGE_LOW_SIZE)
	#define STORAGE_EEPROM_CONFIG_VOLTAGE_HIGH_SIZE			(sizeof(uint16_t))
	#define STORAGE_EEPROM_CONFIG_VOLTAGE_HIGH_TYPE			STORAGE_EEPROM_TYPE_INTEGER16

	/* Current high */
	#define STORAGE_EEPROM_CONFIG_CURRENT_HIGH_ADDR			(STORAGE_EEPROM_CONFIG_VOLTAGE_HIGH_ADDR      + STORAGE_EEPROM_CONFIG_VOLTAGE_HIGH_SIZE)
	#define STORAGE_EEPROM_CONFIG_CURRENT_HIGH_SIZE			(sizeof(uint8_t))
	#define STORAGE_EEPROM_CONFIG_CURRENT_HIGH_TYPE			STORAGE_EEPROM_TYPE_INTEGER8

	/* Freq low */
	#define STORAGE_EEPROM_CONFIG_FREQ_LOW_ADDR				(STORAGE_EEPROM_CONFIG_CURRENT_HIGH_ADDR      + STORAGE_EEPROM_CONFIG_CURRENT_HIGH_SIZE)
	#define STORAGE_EEPROM_CONFIG_FREQ_LOW_SIZE				(sizeof(uint8_t))
	#define STORAGE_EEPROM_CONFIG_FREQ_LOW_TYPE				STORAGE_EEPROM_TYPE_INTEGER8

	/* Freq high */
	#define STORAGE_EEPROM_CONFIG_FREQ_HIGH_ADDR			(STORAGE_EEPROM_CONFIG_FREQ_LOW_ADDR          + STORAGE_EEPROM_CONFIG_FREQ_LOW_SIZE)
	#define STORAGE_EEPROM_CONFIG_FREQ_HIGH_SIZE			(sizeof(uint8_t))
	#define STORAGE_EEPROM_CONFIG_FREQ_HIGH_TYPE			STORAGE_EEPROM_TYPE_INTEGER8

	/* Max demand period */
	#define STORAGE_EEPROM_CONFIG_MDM_PERIOD_ADDR			(STORAGE_EEPROM_CONFIG_FREQ_HIGH_ADDR         + STORAGE_EEPROM_CONFIG_FREQ_HIGH_SIZE)
	#define STORAGE_EEPROM_CONFIG_MDM_PERIOD_SIZE			(sizeof(uint8_t))
	#define STORAGE_EEPROM_CONFIG_MDM_PERIOD_TYPE			STORAGE_EEPROM_TYPE_INTEGER8

	/* Earth connected threshold */
	#define STORAGE_EEPROM_CONFIG_EARTH_THRESHOLD_ADDR		(STORAGE_EEPROM_CONFIG_MDM_PERIOD_ADDR        + STORAGE_EEPROM_CONFIG_MDM_PERIOD_SIZE)
	#define STORAGE_EEPROM_CONFIG_EARTH_THRESHOLD_SIZE		(sizeof(float32_t))
	#define STORAGE_EEPROM_CONFIG_EARTH_THRESHOLD_TYPE		STORAGE_EEPROM_TYPE_FLOAT32

	/* Number of tariff */
	#define STORAGE_EEPROM_CONFIG_NUM_TARIFF_ADDR			(STORAGE_EEPROM_CONFIG_EARTH_THRESHOLD_ADDR   + STORAGE_EEPROM_CONFIG_EARTH_THRESHOLD_SIZE)
	#define STORAGE_EEPROM_CONFIG_NUM_TARIFF_SIZE			(sizeof(uint8_t))
	#define STORAGE_EEPROM_CONFIG_NUM_TARIFF_TYPE			STORAGE_EEPROM_TYPE_INTEGER8

	/* Number of time slot */
	#define STORAGE_EEPROM_CONFIG_NUM_TIMESLOT_ADDR			(STORAGE_EEPROM_CONFIG_NUM_TARIFF_ADDR        + STORAGE_EEPROM_CONFIG_NUM_TARIFF_SIZE)
	#define STORAGE_EEPROM_CONFIG_NUM_TIMESLOT_SIZE			(sizeof(uint8_t))
	#define STORAGE_EEPROM_CONFIG_NUM_TIMESLOT_TYPE			STORAGE_EEPROM_TYPE_INTEGER8
	
	/* Times lot array, 6 elements,
	 * 1 element is a EM_TIME_SLOT struct 
	 * EM_TIME_SLOT is defined in em_type.h, structure as below
	 *   . First byte  (uint8_t) is time_start.hour
	 *   . Second byte (uint8_t) is time_start.min
	 *   . Third byte  (uint8_t) is tariff_no
	 *   . Fourth byte (uint8_t) is reserved (NO USE)
	 */
 	#define STORAGE_EEPROM_CONFIG_TIMESLOT_ARRAY_ADDR		(STORAGE_EEPROM_CONFIG_NUM_TIMESLOT_ADDR      + STORAGE_EEPROM_CONFIG_NUM_TIMESLOT_SIZE)
	#define STORAGE_EEPROM_CONFIG_TIMESLOT_ARRAY_SIZE		(sizeof(EM_TIME_SLOT) * 6)
	#define STORAGE_EEPROM_CONFIG_TIMESLOT_ARRAY_TYPE		STORAGE_EEPROM_TYPE_ARRAY_STRUCT
	
#endif /* STORAGE_EEPROM_CONFIG_GROUP */

/* EEPROM EM Calibration Structure location + size 
 * 15 Macro Items */
#if STORAGE_EEPROM_CALIB_GROUP
#define STORAGE_EEPROM_CALIB_ADDR						(STORAGE_EEPROM_START_ADDR + 0x006A)									/* Offset 0x006A */
#define STORAGE_EEPROM_CALIB_SIZE						(sizeof(EM_CALIBRATION) - sizeof(float32_t *) * 4 + \
														 sizeof(uint8_t) * 2 * 6 + \
														 sizeof(float32_t) * 4 * 6)												/* 118 Bytes */
#define STORAGE_EEPROM_CALIB_TYPE						STORAGE_EEPROM_TYPE_STRUCT												/* Structure */

	/* Sampling frequency */
	#define STORAGE_EEPROM_CALIB_SAMPLING_FREQUENCY_ADDR	(STORAGE_EEPROM_CALIB_ADDR                    + 0)					/* First Element */
	#define STORAGE_EEPROM_CALIB_SAMPLING_FREQUENCY_SIZE	(sizeof(float32_t))
	#define STORAGE_EEPROM_CALIB_SAMPLING_FREQUENCY_TYPE	STORAGE_EEPROM_TYPE_FLOAT32
	
	/* Meter constant */
	#define STORAGE_EEPROM_CALIB_METER_CONSTANT_ADDR		(STORAGE_EEPROM_CALIB_SAMPLING_FREQUENCY_ADDR + STORAGE_EEPROM_CALIB_SAMPLING_FREQUENCY_SIZE)
	#define STORAGE_EEPROM_CALIB_METER_CONSTANT_SIZE		(sizeof(uint16_t))
	#define STORAGE_EEPROM_CALIB_METER_CONSTANT_TYPE		STORAGE_EEPROM_TYPE_INTEGER16

	/* Pulse on time */
	#define STORAGE_EEPROM_CALIB_PULSE_ON_TIME_ADDR			(STORAGE_EEPROM_CALIB_METER_CONSTANT_ADDR     + STORAGE_EEPROM_CALIB_METER_CONSTANT_SIZE)
	#define STORAGE_EEPROM_CALIB_PULSE_ON_TIME_SIZE			(sizeof(uint16_t))
	#define STORAGE_EEPROM_CALIB_PULSE_ON_TIME_TYPE			STORAGE_EEPROM_TYPE_INTEGER16
	
	/* RTC compensation default offset */
	#define STORAGE_EEPROM_CALIB_RTC_COMP_OFFSET_ADDR		(STORAGE_EEPROM_CALIB_PULSE_ON_TIME_ADDR     + STORAGE_EEPROM_CALIB_METER_CONSTANT_SIZE)
	#define STORAGE_EEPROM_CALIB_RTC_COMP_OFFSET_SIZE		(sizeof(float32_t))
	#define STORAGE_EEPROM_CALIB_RTC_COMP_OFFSET_TYPE		STORAGE_EEPROM_TYPE_FLOAT32

	/* VRMS Coefficient */
	#define STORAGE_EEPROM_CALIB_COEFF_VRMS_ADDR			(STORAGE_EEPROM_CALIB_RTC_COMP_OFFSET_ADDR      + STORAGE_EEPROM_CALIB_RTC_COMP_OFFSET_SIZE)
	#define STORAGE_EEPROM_CALIB_COEFF_VRMS_SIZE			(sizeof(float32_t))
	#define STORAGE_EEPROM_CALIB_COEFF_VRMS_TYPE			STORAGE_EEPROM_TYPE_FLOAT32

	/* I1RMS Coefficient */
	#define STORAGE_EEPROM_CALIB_COEFF_I1RMS_ADDR			(STORAGE_EEPROM_CALIB_COEFF_VRMS_ADDR         + STORAGE_EEPROM_CALIB_COEFF_VRMS_SIZE)
	#define STORAGE_EEPROM_CALIB_COEFF_I1RMS_SIZE			(sizeof(float32_t))
	#define STORAGE_EEPROM_CALIB_COEFF_I1RMS_TYPE			STORAGE_EEPROM_TYPE_FLOAT32

	/* I2RMS Coefficient */
	#define STORAGE_EEPROM_CALIB_COEFF_I2RMS_ADDR			(STORAGE_EEPROM_CALIB_COEFF_I1RMS_ADDR        + STORAGE_EEPROM_CALIB_COEFF_I1RMS_SIZE)
	#define STORAGE_EEPROM_CALIB_COEFF_I2RMS_SIZE			(sizeof(float32_t))
	#define STORAGE_EEPROM_CALIB_COEFF_I2RMS_TYPE			STORAGE_EEPROM_TYPE_FLOAT32

	/* (I1) Active Power Coefficient */
	#define STORAGE_EEPROM_CALIB_COEFF_ACTIVE_POWER_ADDR	(STORAGE_EEPROM_CALIB_COEFF_I2RMS_ADDR        + STORAGE_EEPROM_CALIB_COEFF_I2RMS_SIZE)
	#define STORAGE_EEPROM_CALIB_COEFF_ACTIVE_POWER_SIZE	(sizeof(float32_t))
	#define STORAGE_EEPROM_CALIB_COEFF_ACTIVE_POWER_TYPE	STORAGE_EEPROM_TYPE_FLOAT32

	/* (I1) Reactive Power Coefficient */
	#define STORAGE_EEPROM_CALIB_COEFF_REACTIVE_POWER_ADDR	(STORAGE_EEPROM_CALIB_COEFF_ACTIVE_POWER_ADDR + STORAGE_EEPROM_CALIB_COEFF_ACTIVE_POWER_SIZE)
	#define STORAGE_EEPROM_CALIB_COEFF_REACTIVE_POWER_SIZE	(sizeof(float32_t))
	#define STORAGE_EEPROM_CALIB_COEFF_REACTIVE_POWER_TYPE	STORAGE_EEPROM_TYPE_FLOAT32

	/* (I1) Apparent Power Coefficient */
	#define STORAGE_EEPROM_CALIB_COEFF_APPARENT_POWER_ADDR	(STORAGE_EEPROM_CALIB_COEFF_REACTIVE_POWER_ADDR + STORAGE_EEPROM_CALIB_COEFF_REACTIVE_POWER_SIZE)
	#define STORAGE_EEPROM_CALIB_COEFF_APPARENT_POWER_SIZE	(sizeof(float32_t))
	#define STORAGE_EEPROM_CALIB_COEFF_APPARENT_POWER_TYPE	STORAGE_EEPROM_TYPE_FLOAT32

	/* (I2) Active Power Coefficient */
	#define STORAGE_EEPROM_CALIB_COEFF_ACTIVE_POWER2_ADDR	(STORAGE_EEPROM_CALIB_COEFF_APPARENT_POWER_ADDR + STORAGE_EEPROM_CALIB_COEFF_APPARENT_POWER_SIZE)
	#define STORAGE_EEPROM_CALIB_COEFF_ACTIVE_POWER2_SIZE	(sizeof(float32_t))
	#define STORAGE_EEPROM_CALIB_COEFF_ACTIVE_POWER2_TYPE	STORAGE_EEPROM_TYPE_FLOAT32

	/* (I2) Reactive Power Coefficient */
	#define STORAGE_EEPROM_CALIB_COEFF_REACTIVE_POWER2_ADDR	(STORAGE_EEPROM_CALIB_COEFF_ACTIVE_POWER2_ADDR  + STORAGE_EEPROM_CALIB_COEFF_ACTIVE_POWER2_SIZE)
	#define STORAGE_EEPROM_CALIB_COEFF_REACTIVE_POWER2_SIZE	(sizeof(float32_t))
	#define STORAGE_EEPROM_CALIB_COEFF_REACTIVE_POWER2_TYPE	STORAGE_EEPROM_TYPE_FLOAT32

	/* (I2) Apparent Power Coefficient */
	#define STORAGE_EEPROM_CALIB_COEFF_APPARENT_POWER2_ADDR	(STORAGE_EEPROM_CALIB_COEFF_REACTIVE_POWER2_ADDR + STORAGE_EEPROM_CALIB_COEFF_REACTIVE_POWER2_SIZE)
	#define STORAGE_EEPROM_CALIB_COEFF_APPARENT_POWER2_SIZE	(sizeof(float32_t))
	#define STORAGE_EEPROM_CALIB_COEFF_APPARENT_POWER2_TYPE	STORAGE_EEPROM_TYPE_FLOAT32
	
	/* HW Phase Correction Degree */
	#define STORAGE_EEPROM_CALIB_HW_DEGREE_ADDR				(STORAGE_EEPROM_CALIB_COEFF_APPARENT_POWER2_ADDR + STORAGE_EEPROM_CALIB_COEFF_APPARENT_POWER2_SIZE)
	#define STORAGE_EEPROM_CALIB_HW_DEGREE_SIZE				(sizeof(float32_t))
	#define STORAGE_EEPROM_CALIB_HW_DEGREE_TYPE				STORAGE_EEPROM_TYPE_FLOAT32

	/* HW Phase Correction Delay Select */
	#define STORAGE_EEPROM_CALIB_HW_DELAY_SELECT_ADDR		(STORAGE_EEPROM_CALIB_HW_DEGREE_ADDR            + STORAGE_EEPROM_CALIB_HW_DEGREE_SIZE)
	#define STORAGE_EEPROM_CALIB_HW_DELAY_SELECT_SIZE		(sizeof(uint8_t))
	#define STORAGE_EEPROM_CALIB_HW_DELAY_SELECT_TYPE		STORAGE_EEPROM_TYPE_INTEGER8

	/* HW Phase Correction Reserved - NO USE */
	#define STORAGE_EEPROM_CALIB_HW_RESERVED_ADDR			(STORAGE_EEPROM_CALIB_HW_DELAY_SELECT_ADDR      + STORAGE_EEPROM_CALIB_HW_DELAY_SELECT_SIZE)
	#define STORAGE_EEPROM_CALIB_HW_RESERVED_SIZE			(sizeof(uint8_t))
	#define STORAGE_EEPROM_CALIB_HW_RESERVED_TYPE			STORAGE_EEPROM_TYPE_INTEGER8

	/* SW Phase Correction Degree (I1) Array, 6 elements
	 * 1 element is a float32_t type */
	#define STORAGE_EEPROM_CALIB_SW_DEGREE_I1_ARRAY_ADDR	(STORAGE_EEPROM_CALIB_HW_RESERVED_ADDR          + STORAGE_EEPROM_CALIB_HW_RESERVED_SIZE)
	#define STORAGE_EEPROM_CALIB_SW_DEGREE_I1_ARRAY_SIZE	(sizeof(float32_t) * 6)
	#define STORAGE_EEPROM_CALIB_SW_DEGREE_I1_ARRAY_TYPE	STORAGE_EEPROM_TYPE_ARRAY_FLOAT32

	/* SW Phase Correction Degree (I2) Array, 6 elements
	 * 1 element is a float32_t type */
	#define STORAGE_EEPROM_CALIB_SW_DEGREE_I2_ARRAY_ADDR	(STORAGE_EEPROM_CALIB_SW_DEGREE_I1_ARRAY_ADDR   + STORAGE_EEPROM_CALIB_SW_DEGREE_I1_ARRAY_SIZE)
	#define STORAGE_EEPROM_CALIB_SW_DEGREE_I2_ARRAY_SIZE	(sizeof(float32_t) * 6)
	#define STORAGE_EEPROM_CALIB_SW_DEGREE_I2_ARRAY_TYPE	STORAGE_EEPROM_TYPE_ARRAY_FLOAT32
	
	/* SW Phase Channel Gain Array, 6 elements
	 * 1 element is a float32_t type */
	#define STORAGE_EEPROM_CALIB_SW_GAIN_I1_ARRAY_ADDR		(STORAGE_EEPROM_CALIB_SW_DEGREE_I2_ARRAY_ADDR   + STORAGE_EEPROM_CALIB_SW_DEGREE_I2_ARRAY_SIZE)
	#define STORAGE_EEPROM_CALIB_SW_GAIN_I1_ARRAY_SIZE		(sizeof(float32_t) * 6)
	#define STORAGE_EEPROM_CALIB_SW_GAIN_I1_ARRAY_TYPE		STORAGE_EEPROM_TYPE_ARRAY_FLOAT32
	
	/* SW Neutral Channel Gain Array, 6 elements
	 * 1 element is a float32_t type */
	#define STORAGE_EEPROM_CALIB_SW_GAIN_I2_ARRAY_ADDR		(STORAGE_EEPROM_CALIB_SW_GAIN_I1_ARRAY_ADDR     + STORAGE_EEPROM_CALIB_SW_GAIN_I1_ARRAY_SIZE)
	#define STORAGE_EEPROM_CALIB_SW_GAIN_I2_ARRAY_SIZE		(sizeof(float32_t) * 6)
	#define STORAGE_EEPROM_CALIB_SW_GAIN_I2_ARRAY_TYPE		STORAGE_EEPROM_TYPE_ARRAY_FLOAT32
	
#endif /* STORAGE_EEPROM_CALIB_GROUP */

/* EEPROM RTC Time Backup Location + Size 
 * 1 Macro Item */
#if STORAGE_EEPROM_RTC_TIME_BACKUP_GROUP

#define STORAGE_EEPROM_RTC_TIME_BACKUP_ADDR				(STORAGE_EEPROM_START_ADDR + 0x010A)									/* Offset 0x010A */
#define STORAGE_EEPROM_RTC_TIME_BACKUP_SIZE				(sizeof(RTC_DATE_TIME))													/* 8 Bytes */
#define STORAGE_EEPROM_RTC_TIME_BACKUP_TYPE				STORAGE_EEPROM_TYPE_STRUCT												/* Structure */

#endif /* STORAGE_EEPROM_RTC_TIME_BACKUP_GROUP */

/* EEPROM System State Location + Size
 * 1 Macro Item */
#if STORAGE_EEPROM_SYSTEM_STATE_GROUP

#define STORAGE_EEPROM_SYSTEM_STATE_ADDR				(STORAGE_EEPROM_START_ADDR + 0x012A)									/* Offset 0x012A */
#define STORAGE_EEPROM_SYSTEM_STATE_SIZE				129																		/* 117 Bytes */
#define STORAGE_EEPROM_SYSTEM_STATE_TYPE				STORAGE_EEPROM_TYPE_ARRAY_INTEGER8										/* Array of uint8_t */

#endif /* STORAGE_EEPROM_SYSTEM_STATE_GROUP */

/* EEPROM Platform Information
 * 6 Macro Items */
#if STORAGE_EEPROM_PLATFORM_INFO_GROUP

#define STORAGE_EEPROM_PLATFORM_INFO_ADDR					(STORAGE_EEPROM_START_ADDR + 0x020A)								/* Offset 0x020A */
#define STORAGE_EEPROM_PLATFORM_INFO_SIZE					(sizeof(PLATFORM_INFO))												/* 24 Bytes */
#define STORAGE_EEPROM_PLATFORM_INFO_TYPE					STORAGE_EEPROM_TYPE_STRUCT											/* Structure */

	/* Unit code */
	#define STORAGE_EEPROM_PLATFORM_INFO_UNIT_CODE_ADDR			(STORAGE_EEPROM_PLATFORM_INFO_ADDR                + 0)			/* First Element */
	#define STORAGE_EEPROM_PLATFORM_INFO_UNIT_CODE_SIZE			(sizeof(uint8_t) * UNITCODE_SIZE)
	#define STORAGE_EEPROM_PLATFORM_INFO_UNIT_CODE_TYPE			STORAGE_EEPROM_TYPE_ARRAY_INTEGER8

	/* Type of meter */
	#define STORAGE_EEPROM_PLATFORM_INFO_TYPE_OF_METER_ADDR		(STORAGE_EEPROM_PLATFORM_INFO_UNIT_CODE_ADDR      + STORAGE_EEPROM_PLATFORM_INFO_UNIT_CODE_SIZE)
	#define STORAGE_EEPROM_PLATFORM_INFO_TYPE_OF_METER_SIZE		(sizeof(uint8_t) * TYPE_OF_METER_SIZE)
	#define STORAGE_EEPROM_PLATFORM_INFO_TYPE_OF_METER_TYPE		STORAGE_EEPROM_TYPE_ARRAY_INTEGER8

	/* Manufacture Date */
	#define STORAGE_EEPROM_PLATFORM_INFO_MANUFACTURE_DATE_ADDR	(STORAGE_EEPROM_PLATFORM_INFO_TYPE_OF_METER_ADDR  + STORAGE_EEPROM_PLATFORM_INFO_TYPE_OF_METER_SIZE)
	#define STORAGE_EEPROM_PLATFORM_INFO_MANUFACTURE_DATE_SIZE	(sizeof(RTC_DATE_TIME))
	#define STORAGE_EEPROM_PLATFORM_INFO_MANUFACTURE_DATE_TYPE	STORAGE_EEPROM_TYPE_STRUCT
	
	/* Number of reset */
	#define STORAGE_EEPROM_PLATFORM_INFO_NUMBER_OF_RESET_ADDR	(STORAGE_EEPROM_PLATFORM_INFO_MANUFACTURE_DATE_ADDR + STORAGE_EEPROM_PLATFORM_INFO_MANUFACTURE_DATE_SIZE)
	#define STORAGE_EEPROM_PLATFORM_INFO_NUMBER_OF_RESET_SIZE	(sizeof(uint16_t))
	#define STORAGE_EEPROM_PLATFORM_INFO_NUMBER_OF_RESET_TYPE	STORAGE_EEPROM_TYPE_INTEGER16
	
	/* LCD Scroll Period */
	#define STORAGE_EEPROM_PLATFORM_INFO_LCD_SCROLL_PERIOD_ADDR	(STORAGE_EEPROM_PLATFORM_INFO_NUMBER_OF_RESET_ADDR  + STORAGE_EEPROM_PLATFORM_INFO_NUMBER_OF_RESET_SIZE)
	#define STORAGE_EEPROM_PLATFORM_INFO_LCD_SCROLL_PERIOD_SIZE	(sizeof(uint16_t))
	#define STORAGE_EEPROM_PLATFORM_INFO_LCD_SCROLL_PERIOD_TYPE	STORAGE_EEPROM_TYPE_INTEGER16	

	/* Tamper Indicator */
	#define STORAGE_EEPROM_PLATFORM_INFO_TAMPER_INDICATOR_ADDR	(STORAGE_EEPROM_PLATFORM_INFO_LCD_SCROLL_PERIOD_ADDR  + STORAGE_EEPROM_PLATFORM_INFO_LCD_SCROLL_PERIOD_SIZE)
	#define STORAGE_EEPROM_PLATFORM_INFO_TAMPER_INDICATOR_SIZE	(sizeof(uint8_t))
	#define STORAGE_EEPROM_PLATFORM_INFO_TAMPER_INDICATOR_TYPE	STORAGE_EEPROM_TYPE_INTEGER8	

	/* Logical device name */
	#define STORAGE_EEPROM_PLATFORM_INFO_LDN_ADDR	(STORAGE_EEPROM_PLATFORM_INFO_TAMPER_INDICATOR_ADDR  + STORAGE_EEPROM_PLATFORM_INFO_TAMPER_INDICATOR_SIZE + 1) // +1 padding
	#define STORAGE_EEPROM_PLATFORM_INFO_LDN_SIZE	(sizeof(uint8_t) * NAME_SIZE)
	#define STORAGE_EEPROM_PLATFORM_INFO_LDN_TYPE	STORAGE_EEPROM_TYPE_ARRAY_INTEGER8	

	/* Manufacture name */
	#define STORAGE_EEPROM_PLATFORM_INFO_MANUFACTURE_ADDR	(STORAGE_EEPROM_PLATFORM_INFO_LDN_ADDR  + STORAGE_EEPROM_PLATFORM_INFO_LDN_SIZE)
	#define STORAGE_EEPROM_PLATFORM_INFO_MANUFACTURE_SIZE	(sizeof(uint8_t) * NAME_SIZE)
	#define STORAGE_EEPROM_PLATFORM_INFO_MANUFACTURE_TYPE	STORAGE_EEPROM_TYPE_ARRAY_INTEGER8	
	
	/* Firmware version name name */
	#define STORAGE_EEPROM_PLATFORM_INFO_FIRMWARE_ADDR	(STORAGE_EEPROM_PLATFORM_INFO_MANUFACTURE_ADDR  + STORAGE_EEPROM_PLATFORM_INFO_MANUFACTURE_SIZE)
	#define STORAGE_EEPROM_PLATFORM_INFO_FIRMWARE_SIZE	(sizeof(uint8_t) * NAME_SIZE)
	#define STORAGE_EEPROM_PLATFORM_INFO_FIRMWARE_TYPE	STORAGE_EEPROM_TYPE_ARRAY_INTEGER8	

	/* Category name */
	#define STORAGE_EEPROM_PLATFORM_INFO_CATEGORY_ADDR	(STORAGE_EEPROM_PLATFORM_INFO_FIRMWARE_ADDR  + STORAGE_EEPROM_PLATFORM_INFO_FIRMWARE_SIZE)
	#define STORAGE_EEPROM_PLATFORM_INFO_CATEGORY_SIZE	(sizeof(uint8_t) * NAME_SIZE)
	#define STORAGE_EEPROM_PLATFORM_INFO_CATEGORY_TYPE	STORAGE_EEPROM_TYPE_ARRAY_INTEGER8	
	
	/* Current rating name */
	#define STORAGE_EEPROM_PLATFORM_INFO_CURRENT_RATE_ADDR	(STORAGE_EEPROM_PLATFORM_INFO_CATEGORY_ADDR  + STORAGE_EEPROM_PLATFORM_INFO_CATEGORY_SIZE)
	#define STORAGE_EEPROM_PLATFORM_INFO_CURRENT_RATE_SIZE	(sizeof(uint8_t) * NAME_SIZE)
	#define STORAGE_EEPROM_PLATFORM_INFO_CURRENT_RATE_TYPE	STORAGE_EEPROM_TYPE_ARRAY_INTEGER8	
	
#endif /* STORAGE_EEPROM_PLATFORM_INFO_GROUP */

/* EEPROM Monthly Data Log (12 Months)
 * 1 Macro Item */
#if STORAGE_EEPROM_ENERGY_DATA_LOG_GROUP

#define STORAGE_EEPROM_ENERGY_DATA_LOG_ADDR			(STORAGE_EEPROM_START_ADDR + 0x0274)										/* Offset 0x0274 */
#define STORAGE_EEPROM_ENERGY_DATA_LOG_SIZE			(sizeof(ONE_MONTH_ENERGY_DATA_LOG) *  BILLING_MAX_ENTRY)					/* 1280 Bytes */
#define STORAGE_EEPROM_ENERGY_DATA_LOG_TYPE			STORAGE_EEPROM_TYPE_ARRAY_STRUCT											/* Array of Structure */

#endif /* STORAGE_EEPROM_ENERGY_DATA_LOG_GROUP */

/* EEPROM Tamper Data Log (200 Events)
 * 1 Macro Item */
#if STORAGE_EEPROM_TAMPER_DATA_LOG_GROUP

#define STORAGE_EEPROM_TAMPER_DATA_LOG_ADDR			(STORAGE_EEPROM_START_ADDR + 0x0774)										/* Offset 0x0774 */
#define STORAGE_EEPROM_TAMPER_DATA_LOG_SIZE			(sizeof(ONE_TAMPER_DATA_LOG) * 200)											/* 2000 Bytes */
#define STORAGE_EEPROM_TAMPER_DATA_LOG_TYPE			STORAGE_EEPROM_TYPE_ARRAY_STRUCT											/* Array of Structure */

#endif /* STORAGE_EEPROM_TAMPER_DATA_LOG_GROUP */


/* DLMS secret
 * 2 Macro Item */
#if STORAGE_EEPROM_DLMS_SECRET_GROUP

#define STORAGE_EEPROM_LLS_SECRET_ADDR			(STORAGE_EEPROM_START_ADDR + 0x1100)										/* Offset 0x1100 */
#define STORAGE_EEPROM_LLS_SECRET_SIZE			(sizeof(uint8_t) *  16)					/* 16 Bytes */
#define STORAGE_EEPROM_LLS_SECRET_TYPE			STORAGE_EEPROM_TYPE_ARRAY_INTEGER8											/* Array of integer 8 */
#define STORAGE_EEPROM_HLS_SECRET_ADDR			(STORAGE_EEPROM_LLS_SECRET_ADDR + STORAGE_EEPROM_LLS_SECRET_SIZE)
#define STORAGE_EEPROM_HLS_SECRET_SIZE			(sizeof(uint8_t) *  16)					/* 16 Bytes */
#define STORAGE_EEPROM_HLS_SECRET_TYPE			STORAGE_EEPROM_TYPE_ARRAY_INTEGER8											/* Array of integer 8 */

#endif /* STORAGE_EEPROM_ENERGY_DATA_LOG_GROUP */

/* EEPROM Daily Data Log (32 Events)
 * 1 Macro Item */
#if STORAGE_EEPROM_DAILY_DATA_LOG_GROUP
/* In used entries */
#define STORAGE_EEPROM_DAILY_USED_ADDR				(STORAGE_EEPROM_START_ADDR + 0x1200)
#define STORAGE_EEPROM_DAILY_USED_SIZE				(sizeof(uint32_t))
#define STORAGE_EEPROM_DAILY_USED_TYPE				STORAGE_EEPROM_TYPE_INTEGER16

/* Index entry */
#define STORAGE_EEPROM_DAILY_INDEX_ADDR				(STORAGE_EEPROM_DAILY_USED_ADDR + STORAGE_EEPROM_DAILY_USED_SIZE)
#define STORAGE_EEPROM_DAILY_INDEX_SIZE				(sizeof(uint16_t))
#define STORAGE_EEPROM_DAILY_INDEX_TYPE				STORAGE_EEPROM_TYPE_INTEGER16

#define STORAGE_EEPROM_DAILY_DATA_LOG_ADDR			(STORAGE_EEPROM_DAILY_INDEX_ADDR + STORAGE_EEPROM_DAILY_INDEX_SIZE)
#define STORAGE_EEPROM_DAILY_DATA_LOG_SIZE			(sizeof(ONE_DAILY_ENERGY_DATA_LOG) * DAILYLOAD_MAX_ENTRY)					/* x16: 512 Bytes */
#define STORAGE_EEPROM_DAILY_DATA_LOG_TYPE			STORAGE_EEPROM_TYPE_ARRAY_STRUCT											/* Array of Structure */

#endif /* STORAGE_EEPROM_DAILY_DATA_LOG_GROUP */

/* EEPROM Block load Data Log
 * 1 Macro Item */
#if STORAGE_EEPROM_BLOCK_DATA_LOG_GROUP
/* In used entries */
#define STORAGE_EEPROM_BLOCK_USED_ADDR				(STORAGE_EEPROM_START_ADDR + 0x1500)
#define STORAGE_EEPROM_BLOCK_USED_SIZE				(sizeof(uint32_t))
#define STORAGE_EEPROM_BLOCK_USED_TYPE				STORAGE_EEPROM_TYPE_INTEGER16

/* Index entry */
#define STORAGE_EEPROM_BLOCK_INDEX_ADDR				(STORAGE_EEPROM_BLOCK_USED_ADDR + STORAGE_EEPROM_BLOCK_USED_SIZE)
#define STORAGE_EEPROM_BLOCK_INDEX_SIZE				(sizeof(uint16_t))
#define STORAGE_EEPROM_BLOCK_INDEX_TYPE				STORAGE_EEPROM_TYPE_INTEGER16


#define STORAGE_EEPROM_BLOCK_DATA_LOG_ADDR			(STORAGE_EEPROM_BLOCK_INDEX_ADDR + STORAGE_EEPROM_BLOCK_INDEX_SIZE)
#define STORAGE_EEPROM_BLOCK_DATA_LOG_SIZE			(sizeof(ONE_BLOCK_ENERGY_DATA_LOG) *  BLOCKLOAD_MAX_ENTRY)					/* x20: 400 Bytes */
#define STORAGE_EEPROM_BLOCK_DATA_LOG_TYPE			STORAGE_EEPROM_TYPE_ARRAY_STRUCT											/* Array of Structure */

#endif /* STORAGE_EEPROM_BLOCK_DATA_LOG_GROUP */

/* EEPROM Event Data Log
 * 1 Macro Item */
#if STORAGE_EEPROM_EVENT_DATA_LOG_GROUP
/* In used entries */
#define STORAGE_EEPROM_EVENT_USED_ADDR				(STORAGE_EEPROM_START_ADDR + 0x1800)										/* Offset 0x1800 */
#define STORAGE_EEPROM_EVENT_USED_SIZE				(sizeof(uint32_t) * EVENT_MAX_PROFILE)														/* Bytes */
#define STORAGE_EEPROM_EVENT_USED_TYPE				STORAGE_EEPROM_TYPE_ARRAY_STRUCT											/* Array of Structure */
/* Index entry */
#define STORAGE_EEPROM_EVENT_INDEX_ADDR				(STORAGE_EEPROM_EVENT_USED_ADDR + STORAGE_EEPROM_EVENT_USED_SIZE)										/* Offset 0x1900 */
#define STORAGE_EEPROM_EVENT_INDEX_SIZE				(sizeof(uint16_t) * EVENT_MAX_PROFILE)														/* Bytes */
#define STORAGE_EEPROM_EVENT_INDEX_TYPE				STORAGE_EEPROM_TYPE_ARRAY_STRUCT											/* Array of Structure */


#define STORAGE_EEPROM_EVENT_DATA_LOG_ADDR			(STORAGE_EEPROM_EVENT_INDEX_ADDR + STORAGE_EEPROM_EVENT_INDEX_SIZE)										/* Offset 0x1A00 */
#define STORAGE_EEPROM_EVENT_DATA_LOG_SIZE			(sizeof(ONE_EVENT_ENERGY_DATA_LOG) * EVENT_MAX_PROFILE *EVENT_MAX_ENTRY)						/* x26 Bytes */
#define STORAGE_EEPROM_EVENT_DATA_LOG_TYPE			STORAGE_EEPROM_TYPE_ARRAY_STRUCT											/* Array of Structure */

#endif /* STORAGE_EEPROM_EVENT_DATA_LOG_GROUP */



/******************************************************************************
Variable Externs
******************************************************************************/

/******************************************************************************
Functions Prototypes
******************************************************************************/

#endif /* _STORAGE_EEPROM_FORMAT_H */
