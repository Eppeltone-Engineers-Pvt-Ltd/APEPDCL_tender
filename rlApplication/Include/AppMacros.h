#ifndef _APP_MACROS_H
#define _APP_MACROS_H

#include "AppConfig.h"
#if (defined(IRDA_TYPE_METER_HP) && (IRDA_TYPE_METER_HP>1))
#error "invalid macros"
#endif
//------------------------------------
#if (defined(IRDA_TYPE_METER_HP) && (IRDA_TYPE_METER_HP==0))
#define MAX_COMM_BUFFER_LENGTH  64
#define MAX_CMD_RX_LENGTH  32
#endif
#if (defined(IRDA_TYPE_METER_HP) && (IRDA_TYPE_METER_HP==1))
#define MAX_COMM_BUFFER_LENGTH_1  64
#define MAX_COMM_BUFFER_LENGTH  300
#define MAX_CMD_RX_LENGTH  160
#endif

//------------------------------------

#define RECORD_MAN_MD_DATA		0x01
#define RECORD_MD_DATA			0x02
#define RECORD_LS_DATA			0x04
#define RECORD_BILL_DATA		0x08
#define RECORD_PFAIL_DATA		0x10
#define RECORD_DAILY_DATA 		0x20
#define RECORD_DO_BILL_DATA		0x40
#define RECORD_POWER_FAIL_DATA	0x80

#define RECORD_RD_MD_DATA			0x01

#define START_OF_CHAR_OPTICAL    0x45
#define START_OF_CHAR_IRDA       0x3A



#define STRUCT_INSSAVE_SIZE 		250


#define PUSH_BUTTON_PRESSED_FLAG	0x00000001UL
#define ONE_SECOND_OVER_FLAG		0x00000002UL
#define	COMM_RECEIVE_ENABLE			0x00000004UL
#define	COMM_RECEIVE_START			0x00000008UL
#define	COMM_RECEIVE_OVER			0x00000010UL
#define	DISPLAY_CHANGE_FLAG			0x00000020UL
#define PUSH_BUTTON_RELEASED      	0x00000040UL	
#define POWER_DOWN_FLAG				0x00000080UL
#define POWER_STABLE_FLAG			0x00000100UL
#define START_MEASUREMENT			0x00000200UL
#define DO_MEASUREMENT				0x00000400UL
#define START_SAMPLE_BUFFER			0x00000800UL
#define PH_POWER_REV				0x00001000UL
#define NU_POWER_REV				0x00002000UL
#define SINGLE_ACTION_CHECK_FLAG	0x00004000UL
#define PASSIVE_CAL_CHECK_FLAG		0x00008000UL
#define DO_FREQ_MEASUREMENT			0x00010000UL
#define RTC_SECOND_FLAG				0x00020000UL
#define START_LS_NM_MODE			0x00040000UL
#define EIGHT_BIT_TIMER1_FLAG		0x00080000UL
#define PROCESS_MAG_FLAG			0x00100000UL
#define SAVE_AT_NM_EXIT_FLAG		0x00200000UL
#define	COMM_TRANSMIT_FLAG			0x00400000UL
#define	GET_SHUNT_CURRENT_FLAG		0x00800000UL
#define	ADC_NO_LOAD_FIRST_STAGE		0x01000000UL
#define	EXPORT_MODE					0x02000000UL
#define	SHOW_MAG_SYMBOL				0x04000000UL
#define	HARMONICS_PRESENT			0x08000000UL	// share with adc_load_second_stage
#define	ADC_LOAD_SECOND_STAGE		0x08000000UL
#define	PUSH_RELEASED_NM_MODE		0x10000000UL
#define	START_PULSE_NM_MODE			0x20000000UL
#define	PULSE_NM_MODE				0x40000000UL
#define	ZERO_CROSS_MODE				0x80000000UL


//kwh Saving option
#define	SAVE_LEVEL1_KWH_STATE_OPEN		0x01
#define	SAVE_LEVEL1_KWH_STATE_CLOSE		0x02
#define	SAVE_LEVEL2_KWH_STATE_OPEN		0x04
#define	PULSE_ON_FLAG					0x08
#define	FORCE_POWER_FAIL				0x10
#define	RTC_BAT_STATUS_FLAG				0x20
#define	CRYSTAL_REENABLED_FLAG			0x40
#define	SAVE_LEVEL2_KWH_STATE_CLOSE		0x80







//------------------------------------
#define HISTORY_RECORD          		6
#define MAX_TAMPER_COUNT				11
#define TAMPER_GROUP_COUNT				4
#define PFAIL_EVENTS					50
#define	PFAILSAVEDATA_LEN				34
#define	BACKUPTAMPER_LEN				60
#define SW_EVENTS						20
//------------------------------------


//------------------------------------

#define DLMS_LLS_SECRET_SIZE			8
#define DLMS_HLS_SECRET_SIZE			16

#define SR_TYPE_LCD				0x00
#define SR_TYPE_ASCII			0x01
#define SR_TYPE_NUM				0x02


#define MD_LEN                	28
#define ZONE_LEN                20
#define BILLING_DATA_LENGTH     40
#define TOD_LEN                 (ZONE_LEN*8U)
#define TAMPER_DATA_LENGTH      25
#define SOFTCH_DATA_LENGTH      39U
#define BILL_RESET_TYPE_POS		29
#define BILL_TYPE_AUTO			0x00
#define BILL_TYPE_MAN			0x01
//------------------------------------


//CurrentTamperStatusFlag

/*#define TAMPER_NEU_MISS     0x0001
#define TAMPER_OVER_LOAD    0x0002
#define TAMPER_MAG          0x0004
#define TAMPER_REV          0x0008
#define TAMPER_AC_DC        0x0010
#define TAMPER_EARTH        0x0020
#define TAMPER_OU         	0x0040
#define TAMPER_COVEROPEN    0x0080
#define TAMPER_LOW_U    		0x0100
#define TAMPER_FREQUENCY    0x0200
#define TAMPER_LOW_PF       0x0400*/


#define TAMPER_NEU_MISS     0x0001
#define TAMPER_REV          0x0002
#define TAMPER_EARTH        0x0004
#define TAMPER_COVEROPEN    0x0008
#define TAMPER_MAG          0x0010
#define TAMPER_AC_DC        0x0020
#define TAMPER_OVER_LOAD    0x0040
#define TAMPER_LOW_PF       0x0080
#define TAMPER_FREQUENCY    0x0100
#define TAMPER_OU           0x0200
#define TAMPER_LOW_U        0x0400





//------------------------------------



//--------------harmonic--------------
#define SAMPLE_COUNT_50HZ		262
#define SAMPLE_COUNT_250HZ	52


#define MAX_ZONE_NO				8
#define DAILYLS_TOTAL_DAYS		60

//------------------------------------

// common defines

#define RAM_OK 0xBEEF


//------------------------------------
#define LS_EVENT_LENGTH          6U
#define LS_DAY_LENGTH           (48UL*LS_EVENT_LENGTH+4)
#define LS_TOTAL_DAYS           60UL
#define DAY_CROSSOVER_SECONDS   86400L
#define LS_DAILY_LENGTH			18	
//------------------------------------

//------------------------------------
#define	PARA_READ_STATE						0x00
#define	PARA_WRITE_BEFORE_STATE		0x01
#define	PARA_WRITE_AFTER_STATE		0x02
#define PARA_TYPE_DAILY_STATE 		0x03 // save another copy daily in case everything else fails 


#define PARA_TYPE_INSTANT				0x01
#define PARA_TYPE_TAMPER				0x02
#define PARA_TYPE_PFAIL_INSTANT         0x03 // power fail save 
#define PARA_TYPE_TAMPER_AP				0x04

#define PARA_TYPE_MD_KW 				0x20 // 
//------------------------------------

#define ENERGY_TYPE_KWH 			0x00 
#define ENERGY_TYPE_KVAH 			0x01

#define DISPLAY_MODE1_WAIT_TIME		0
#define DISPLAY_MODE2_WAIT_TIME		5
#define DISPLAY_MODE3_WAIT_TIME		5
#define DISPLAY_PUSH_EXIT_WAIT_TIME	5

#define DISPLAY_MODE1_HOLD_TIME		60
#define DISPLAY_MODE2_HOLD_TIME		60
#define DISPLAY_MODE3_HOLD_TIME		60

#define DISPLAY_PUSH_MODE3_HOLD_TIME	60
#if (defined(IRDA_TYPE_METER_HP) && (IRDA_TYPE_METER_HP == 0))
#define PERMANENT_MAG_HOLD_TIME	30
#define MAG_HOLD_TIME	30
#define MAG_DC_DETECT_TIME 30
#endif
#if (defined(IRDA_TYPE_METER_HP) && (IRDA_TYPE_METER_HP == 1)&& (IRDA_TYPE_METER_AP == 0))
#define PERMANENT_MAG_HOLD_TIME	90
#define MAG_HOLD_TIME	90
#define MAG_DC_DETECT_TIME 30
#endif
#if (defined(IRDA_TYPE_METER_HP) && (IRDA_TYPE_METER_HP == 1)&& (IRDA_TYPE_METER_AP == 1))
#define PERMANENT_MAG_HOLD_TIME	8
#define MAG_HOLD_TIME	8
#define MAG_DC_DETECT_TIME 30
#endif


#define HAR_HZ_TYPE_50	0x00
#define HAR_HZ_TYPE_250	0x01


#define MD_PERIOD_CHANGE_FLAG	0x01
#define MD_PERIOD_SET_FLAG		0x02
#define MD_PERIOD_READ_FLAG		0x04

#define DECIMAL_SEPERATOR 0x2E
#define RECORD_SEPERATOR  0x1E
#define FIELD_SEPERATOR   0x1F


#endif
