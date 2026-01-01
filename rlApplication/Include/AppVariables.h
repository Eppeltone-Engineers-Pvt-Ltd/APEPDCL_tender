#ifndef _APP_VARIABLES_H
#define _APP_VARIABLES_H


//-------- include device specific files----------------
#include "..\\..\\rlDevice\\Include\\dTypedef.h"
//-----------------------------------------------------

//-------- include app specific files----------------
#include "AppMacros.h"
#include "AppConfig.h"
//------------------------------------------------------


//----------------4 bytes data--------------
extern uint8_t ramCHECK1;
extern uint32_t activePulsePower,apparentPulsePower;
extern uint32_t activePulsePowerSumNM,activePowerNM;
extern uint32_t apperentPulsePowerSumNM,apperentPowerNM;
extern uint32_t  lsAvgVolt;
extern volatile uint32_t mcu_flag;
extern uint32_t prekWh,prevTimeCounter,prekWhinterval;
extern uint32_t nmInfo[3];
//-----------------------------------------

//--------------2 bytes data-----------------
extern uint8_t ramCHECK2,isforcedUinity;
extern uint16_t TamperTime_counter[MAX_TAMPER_COUNT];
extern uint16_t TamperTime_counter_AP[MAX_TAMPER_COUNT];
extern uint16_t pushModeTime;
extern uint16_t display_flag,ls_nm_start_loc,nm_ls_start_index,nm_ls_end_index,CurrentTamperStatusFlag,CurrentTamperStatusFlag_AP;
extern uint16_t nm_false_current_detect_count;
extern volatile uint16_t mag_toggle_count_duration,mag_pulse_width,is_dc_magnet_exist_duration,peakVoltage;
extern volatile uint16_t no_of_pulses_in_nm_Active,no_of_pulses_in_nm_App;
//extern uint16_t test_kwh_counter;

//----------------------------------

//---------------rtc----------------------
extern uint8_t ramCHECK3;
extern uint8_t RxTxBuffer[MAX_COMM_BUFFER_LENGTH];
//----------------------------------------

//----------------------------------------
extern uint8_t md_period,ls_period,push_button_hold_time,pf_by_sample,new_md_period,lag_only_value;
extern uint8_t switch_to_normal_mode,powerStableCounter;
extern volatile uint8_t mag_hold_time,mag_toggle_count,cum_mag_toggle_count,power_fail_state,permanent_mag_wait_time,received_uart;
extern volatile uint8_t is_vfail_with_load;
//--------------display-------------------
extern uint8_t ramCHECK4;
extern uint8_t refresh_time,ModeID,button_hold_time,currentZoneID,currentSeasonID;
extern const uint8_t *displayParamPtr;
extern uint8_t ramCHECK5;
extern uint8_t harChU,harChI,har_wait_time;
#if(defined(IRDA_TYPE_METER_AP) && (IRDA_TYPE_METER_AP == 1))
extern uint8_t gblserialnumberLCD[8];
extern uint32_t kWh_History[6],kW_MD_History[6];
#endif
//----------------------------------------


extern struct
{   

	uint16_t	Voltage     	;
	uint8_t 	PowerFactor 	;
	uint8_t 	LeadLag     	;	
	uint32_t  	PhCurrent   	;
	uint32_t  	NeuCurrent  	;
	uint32_t 	PhPower     	;
	uint32_t 	NeuPower    	;
	uint32_t 	AppPower    	;
	uint32_t  	Frequency   	;
	uint32_t  	EffectiveI   	;
	uint32_t  	EffectiveP   	;
    
}Ins;

struct MD
{
	uint16_t kW;
	uint16_t kVA;
	uint32_t kW_Date;
	uint32_t kVA_Date;
};


struct billMD
{
	uint32_t CumkWMD;
	uint32_t CumkVAMD;

	
	struct MD Current;
	
	
};

struct TOD
{
	struct MD TODMD;
	uint32_t kWh;
	uint32_t kVAh;
};

extern struct
{
	uint32_t RD_KW;
	uint32_t RD_KVA;
}Ins_RD_KW;

struct TOD_8
{
	struct TOD TOD[8];
};

extern struct
{
	uint32_t  	CumkWh				;
	uint32_t  	CumkVAh				;
	struct billMD	BillMD			;		//116 bytes
	struct TOD_8	TODEnergy		;		//64 bytes
	uint32_t    MonthPowerOnDuration;
	uint32_t	CumPowerOnDuration;
	uint16_t 	DayCount			;
	uint16_t	DailyDayCount		;
	uint16_t  	PFailCount			;
	uint8_t		MDResetCount		;
	uint8_t 	IntCount			;
	uint32_t  	ExpCumkWh			;
	uint32_t  	ExpCumkVAh			;
	uint32_t  	NonSolarCumkWh		;
	uint32_t  	timeCounter			; // real time
	uint32_t  	lsAvgV				;
	uint32_t  	lsAvgC				;
	uint32_t   	 DailyPowerOnDuration;
	uint16_t  	kWhCounter			;
	uint16_t  	kVAhCounter			;
	uint16_t  	kWhCounterMD		;
	uint16_t  	kVAhCounterMD		;
	uint16_t  	ZkWhCounter			;
	uint16_t  	ZkVAhCounter		;
	
	uint16_t  	ExpZkWhCounter		;
	uint16_t  	ExpZkVAhCounter		;
	uint16_t  	ExpkWhCounter		;
	uint32_t  	daykWMD_dt			;
	uint16_t  	daykWMD				;
	uint16_t  	PowerOn30			;
	uint16_t  	crc2				;
	
}InsSave;


extern union 
{   
	uint32_t long_data;
	struct
	{ 
		uint16_t lsb;
		uint16_t msb;
	}Short;
	uint8_t byte[4];
}Data;

#if (defined(IRDA_TYPE_METER_HP) && (IRDA_TYPE_METER_HP == 1))
extern struct 
{
	uint16_t 	TamperCount[MAX_TAMPER_COUNT];
	uint8_t  	eventcounts[MAX_TAMPER_COUNT];
	uint8_t		bill_tamper_status;
	uint8_t		Tamper_Once;
	uint16_t 	sw_log_count;
	uint16_t  	TamperStatusFlag	; //33
	uint16_t 	lastEventCode[TAMPER_GROUP_COUNT];
	uint16_t 	lastEventAddr[MAX_TAMPER_COUNT]; //51
	uint16_t	lastOccRes[2]; //55
	uint16_t	MonthTamperStatus; //57
	uint16_t 	crc;		//59
	
}TamperRecord_AP;	// (9*2+10+4+18+4+4)
#endif

#if (defined(IRDA_TYPE_METER_HP) && (IRDA_TYPE_METER_HP == 1))
extern struct 
{
	uint16_t 	TamperCount[MAX_TAMPER_COUNT];
	uint8_t  	eventcounts[MAX_TAMPER_COUNT];
	uint8_t		bill_tamper_status;
	uint8_t		Tamper_Once;
	uint16_t 	sw_log_count;
	uint16_t  	TamperStatusFlag	; //33
	uint16_t 	lastEventCode[TAMPER_GROUP_COUNT];
	uint16_t 	lastEventAddr[MAX_TAMPER_COUNT]; //51
	uint16_t	lastOccRes[2]; //55
	uint16_t	MonthTamperStatus; //57
	uint16_t 	crc;		//59
	
}TamperRecord;	// (9*2+10+4+18+4+4)
#elif (defined(IRDA_TYPE_METER_HP) && (IRDA_TYPE_METER_HP == 0))
extern struct 
{
	uint16_t 	TamperCount[MAX_TAMPER_COUNT];
	uint8_t  	eventcounts[TAMPER_GROUP_COUNT];//only 0,1,2
	uint16_t 	sw_log_count;
	uint16_t  	TamperStatusFlag	;
	uint16_t 	lastEventCode[TAMPER_GROUP_COUNT];
	uint16_t 	lastEventAddr[MAX_TAMPER_COUNT];
	uint16_t	lastOccRes[2];
	uint16_t	MonthTamperStatus;
	uint16_t 	crc;		
	
}TamperRecord;
#else
#error "wrong macro selection at variable.c"
#endif


typedef struct 
{
    uint8_t sec;
    uint8_t min;
    uint8_t hour;
    uint8_t day;
    uint8_t week;
    uint8_t month;
    uint8_t year;
    uint8_t reserve;
} Apprtc;



extern struct 
{
	uint32_t CalanderDateTime;
	uint32_t BillDateTime;
	
}ScheduleActionDateTime;

#endif

