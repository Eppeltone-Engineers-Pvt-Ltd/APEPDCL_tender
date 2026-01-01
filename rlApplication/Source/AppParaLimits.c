//-------- include device specific files----------------
#include "..\\..\\rlDevice\\Include\\dRtc.h"
#include "..\\..\\rlDevice\\Include\\dWatchDog.h"
#include "..\\..\\rlDevice\\Include\\dTypedef.h"
#include "..\\..\\rlDevice\\Include\\dI2c.h"
//-----------------------------------------------------

//-------- include app specific files----------------
#include "..\\include\\AppVariables.h"
#include "..\\include\\AppParaLimits.h"
#include "..\\Include\\AppMisc.h"
#include "..\\Include\\AppIntervalkWh.h"
#include "..\\Include\\AppEeprom.h"
#include "..\\Include\\AppConfig.h"




//----------------------------------------------------
#define KWH_OVER_LIMIT 300
#define PULSE_COUNTER_LIMIT	64000U	// 20KWH
#define MD_LIMIT			23000	// 23KW

#define LIMIT_KWH			0
#define LIMIT_MD			1
#define RAM_CHECK1			2
#define RAM_CHECK2			3
#define RAM_CHECK3			4
#define RAM_CHECK4			5
#define RAM_CHECK5			6



void islimitOverflow(void)
{
	uint8_t status=1;
	//-----------------kWh---------------------
	
		if(InsSave.CumkWh>prekWh)
			if((InsSave.CumkWh-prekWh)>KWH_OVER_LIMIT)
				status=0;
		if(prekWh>(InsSave.CumkWh+InsSave.ZkWhCounter/METER_CONSTANT))
			if((prekWh-(InsSave.CumkWh+InsSave.ZkWhCounter/METER_CONSTANT))>KWH_OVER_LIMIT)
				status=0;			
		
	//-----------------------------------------
	
	//------------------rtc forward jump-------
	
		//if(InsSave.timeCounter)
		
		
	//-----------------------------------------
	
	//------------------md---------------------
		// imports
	//	if(InsSave.daykWMD>MD_LIMIT)
	//		InsSave.daykWMD=0;
		if(InsSave.BillMD.Current.kW>MD_LIMIT)
			InsSave.BillMD.Current.kW=0;

		if(InsSave.BillMD.Current.kVA>MD_LIMIT)
			InsSave.BillMD.Current.kVA=0;
			
		if(InsSave.kWhCounter>PULSE_COUNTER_LIMIT)
			InsSave.kWhCounter=0;
		if(InsSave.kVAhCounter>PULSE_COUNTER_LIMIT)
			InsSave.kVAhCounter=0;
			
		if(InsSave.kWhCounterMD>PULSE_COUNTER_LIMIT)
			InsSave.kWhCounterMD=0;
			
		if(InsSave.kVAhCounterMD>PULSE_COUNTER_LIMIT)
			InsSave.kVAhCounterMD=0;
			
		if(InsSave.ZkWhCounter>PULSE_COUNTER_LIMIT)
			InsSave.ZkWhCounter=0;
			
		if(InsSave.ZkVAhCounter>PULSE_COUNTER_LIMIT)
			InsSave.ZkVAhCounter=0;

		// exports
		if(InsSave.ExpZkWhCounter>PULSE_COUNTER_LIMIT)
			InsSave.ExpZkWhCounter=0;
			
		if(InsSave.ExpZkVAhCounter>PULSE_COUNTER_LIMIT)
			InsSave.ExpZkVAhCounter=0;
			
			
	//-----------------------------------------
	
	
	if(status==0)
		WDT_Restart(0x00);
}

