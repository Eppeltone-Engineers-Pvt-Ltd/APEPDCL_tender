//-------- include device specific files----------------
#include "..\\..\\rlDevice\\Include\\dTypedef.h"
//-----------------------------------------------------

//-------- include app specific files----------------
#include "..\\include\\AppMacros.h"
#include "..\\include\\AppConfig.h"

//------------------------------------------------------


//----------------4 bytes data--------------
uint8_t ramCHECK1='R';
uint32_t activePulsePower,apparentPulsePower;
uint32_t activePulsePowerSumNM,activePowerNM;
uint32_t apperentPulsePowerSumNM,apperentPowerNM;
volatile uint32_t mcu_flag;
uint32_t prekWh,prevTimeCounter,prekWhinterval;
uint32_t nmInfo[3];
//-----------------------------------------

//--------------2 bytes data-----------------
uint8_t ramCHECK2='A';
uint16_t TamperTime_counter[MAX_TAMPER_COUNT];
uint16_t TamperTime_counter_AP[MAX_TAMPER_COUNT];
uint16_t pushModeTime;
uint16_t display_flag=0,ls_nm_start_loc,nm_ls_start_index,nm_ls_end_index,CurrentTamperStatusFlag=0,CurrentTamperStatusFlag_AP=0;
uint16_t nm_false_current_detect_count;
volatile uint16_t mag_toggle_count_duration=0,mag_pulse_width,is_dc_magnet_exist_duration=0,peakVoltage;
volatile uint16_t no_of_pulses_in_nm_Active,no_of_pulses_in_nm_App;

//----------------------------------

//---------------rtc----------------------
uint8_t ramCHECK3='M',isforcedUinity=0;
uint8_t RxTxBuffer[MAX_COMM_BUFFER_LENGTH];
//----------------------------------------

//----------------------------------------
uint8_t md_period=2,ls_period,push_button_hold_time=0x00,pf_by_sample,new_md_period,lag_only_value;
uint8_t switch_to_normal_mode,powerStableCounter=0;
volatile uint8_t mag_hold_time,mag_toggle_count=0,cum_mag_toggle_count=0,power_fail_state,permanent_mag_wait_time,received_uart;
volatile uint8_t is_vfail_with_load;


//--------------display-------------------
uint8_t ramCHECK4='O';
uint8_t refresh_time=0,ModeID=0,button_hold_time=0,currentZoneID,currentSeasonID;
const uint8_t *displayParamPtr;
uint8_t ramCHECK5='K';
uint8_t harChU,harChI,har_wait_time;
#if(defined(IRDA_TYPE_METER_AP) && (IRDA_TYPE_METER_AP == 1))
uint8_t gblserialnumberLCD[8];
uint32_t kWh_History[6],kW_MD_History[6];
#endif
//----------------------------------------


struct
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

struct
{
	uint32_t RD_KW;
	uint32_t RD_KVA;
}Ins_RD_KW;

struct TOD
{
	struct MD TODMD; 
	uint32_t kWh;
	uint32_t kVAh;
};

struct TOD_8
{
	struct TOD TOD[8];
};

//#pragma section bss EngRam
struct
{
	uint32_t  	CumkWh				;
	uint32_t  	CumkVAh				;
	struct billMD	BillMD			;		//116 bytes
	struct TOD_8	TODEnergy		;		//64 bytes
	uint32_t    MonthPowerOnDuration;
	uint32_t	CumPowerOnDuration	;
	uint16_t 	DayCount			;
	uint16_t 	DailyDayCount		;
	uint16_t  	PFailCount			;
	uint8_t		MDResetCount		;
	uint8_t 	IntCount			;
	uint32_t  	ExpCumkWh			;
	uint32_t  	ExpCumkVAh			;
	uint32_t  	NonSolarCumkWh		;
	uint32_t  	timeCounter			; // real time
	uint32_t  	lsAvgV				;
	uint32_t  	lsAvgC				;
	uint32_t    	DailyPowerOnDuration;
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
//#pragma section

union 
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
struct 
{
	uint16_t 	TamperCount[MAX_TAMPER_COUNT];
	uint8_t  	eventcounts[MAX_TAMPER_COUNT];
	uint8_t		bill_tamper_status;
	uint8_t		Tamper_Once;
	uint16_t 	sw_log_count;
	uint16_t  	TamperStatusFlag	;
	uint16_t 	lastEventCode[TAMPER_GROUP_COUNT];
	uint16_t 	lastEventAddr[MAX_TAMPER_COUNT];
	uint16_t	lastOccRes[2];
	uint16_t	MonthTamperStatus;
	uint16_t 	crc;		
	
}TamperRecord_AP;	// (9*2+10+4+18+4+4)
#endif

#if (defined(IRDA_TYPE_METER_HP) && (IRDA_TYPE_METER_HP == 1))
struct 
{
	uint16_t 	TamperCount[MAX_TAMPER_COUNT];
	uint8_t  	eventcounts[MAX_TAMPER_COUNT];
	uint8_t		bill_tamper_status;
	uint8_t		Tamper_Once;
	uint16_t 	sw_log_count;
	uint16_t  	TamperStatusFlag;
	uint16_t 	lastEventCode[TAMPER_GROUP_COUNT];
	uint16_t 	lastEventAddr[MAX_TAMPER_COUNT];
	uint16_t	lastOccRes[2];
	uint16_t	MonthTamperStatus;
	uint16_t 	crc;		
	
}TamperRecord;	// (9*2+10+4+18+4+4)
#elif (defined(IRDA_TYPE_METER_HP) && (IRDA_TYPE_METER_HP == 0))
struct 
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



struct 
{
	uint32_t CalanderDateTime;
	uint32_t BillDateTime;
	
}ScheduleActionDateTime;
