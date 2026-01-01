//-------- include device specific files----------------
#include "..\\..\\rlDevice\\Include\\dI2c.h"
//-----------------------------------------------------

//-------- include app specific files-------------------
#include "..\\Include\\AppMD.h"
#include "..\\Include\\AppVariables.h"
#include "..\\Include\\AppEeprom.h"
#include "..\\Include\\AppLcd.h"
#include "..\\Include\\AppMisc.h"
#include "..\\Include\\AppTOD.h"
#include "..\\Include\\AppConfig.h"


/****************** Maximum Demand ********************************************/
void Compute_MD(uint8_t reset_option)
{   
		
		uint16_t energykWh=InsSave.kWhCounterMD;
		uint16_t energykVAh=InsSave.kVAhCounterMD;
		uint16_t ZenergykWh=InsSave.ZkWhCounter;
		uint16_t ZenergykVAh=InsSave.ZkVAhCounter;
		uint16_t ZenergyExpkWh=InsSave.ExpZkWhCounter;
		uint16_t ZenergyExpkVAh=InsSave.ExpZkVAhCounter;
		
		
		uint32_t logtimeCounter=prevTimeCounter-(prevTimeCounter%(3600/md_period));
		//uint8_t preSeason=0;
		
		logtimeCounter=logtimeCounter+(3600/md_period);
			
		
		energykWh=energykWh/METER_CONSTANT;
		energykVAh=energykVAh/METER_CONSTANT;
		ZenergykWh=ZenergykWh/METER_CONSTANT;
		ZenergykVAh=ZenergykVAh/METER_CONSTANT;
		ZenergyExpkWh=ZenergyExpkWh/METER_CONSTANT;
		ZenergyExpkVAh=ZenergyExpkVAh/METER_CONSTANT;
		
		
		
		if(energykWh>energykVAh)
		{
			energykVAh=energykWh;
			InsSave.kVAhCounterMD=InsSave.kWhCounterMD;
		}
		
		if(ZenergykWh>ZenergykVAh)
		{
			ZenergykVAh=ZenergykWh;
			InsSave.ZkVAhCounter=InsSave.ZkWhCounter;
		}
		
		if(ZenergyExpkWh>ZenergyExpkVAh)
		{
			ZenergyExpkVAh=ZenergyExpkWh;
			InsSave.ExpZkVAhCounter=InsSave.ExpZkWhCounter;
		}		
		
		InsSave.CumkWh+=ZenergykWh;
		InsSave.ExpCumkWh+=ZenergyExpkWh;
		#if (defined(GUJRAT_METER_0) && (GUJRAT_METER_0 == 1))
			InsSave.TODEnergy.TOD[currentZoneID].kWh+=ZenergyExpkWh;
		#else
			InsSave.TODEnergy.TOD[currentZoneID].kWh+=ZenergykWh;
		#endif
		if(((logtimeCounter%86400)>=72000)||(((logtimeCounter%86400)<=18000)))
			InsSave.NonSolarCumkWh+=ZenergyExpkWh;
		
		
		if(reset_option==BILL_TYPE_AUTO)
		{			
			if((energykWh*md_period) >InsSave.BillMD.Current.kW)
			{
				if(InsSave.BillMD.Current.kW)
				{
					if(InsSave.BillMD.CumkWMD>=InsSave.BillMD.Current.kW)
						InsSave.BillMD.CumkWMD-=InsSave.BillMD.Current.kW;
				}
				
				InsSave.BillMD.Current.kW=energykWh*md_period;
				InsSave.BillMD.Current.kW_Date=logtimeCounter;
				InsSave.BillMD.CumkWMD+=InsSave.BillMD.Current.kW;
			}
			if(energykWh*md_period >InsSave.daykWMD)
			{
				InsSave.daykWMD=energykWh*md_period;
				InsSave.daykWMD_dt=logtimeCounter;
			}
			
			
			#if (defined(GUJRAT_METER_0) && (GUJRAT_METER_0 == 1))
			{
				if((ZenergyExpkWh*md_period) >InsSave.TODEnergy.TOD[currentZoneID].TODMD.kW)
				{
					InsSave.TODEnergy.TOD[currentZoneID].TODMD.kW=ZenergyExpkWh*md_period;
					InsSave.TODEnergy.TOD[currentZoneID].TODMD.kW_Date=logtimeCounter;
				}			
			}
			#else
			{
				if((energykWh*md_period) >InsSave.TODEnergy.TOD[currentZoneID].TODMD.kW)
				{
					InsSave.TODEnergy.TOD[currentZoneID].TODMD.kW=energykWh*md_period;
					InsSave.TODEnergy.TOD[currentZoneID].TODMD.kW_Date=logtimeCounter;
				}				
			}
			#endif
		}
		
		
		InsSave.CumkVAh+=ZenergykVAh;
		InsSave.ExpCumkVAh+=ZenergyExpkVAh;
		#if (defined(GUJRAT_METER_0) && (GUJRAT_METER_0 == 1))
			InsSave.TODEnergy.TOD[currentZoneID].kVAh+=ZenergyExpkVAh;
		#else
			InsSave.TODEnergy.TOD[currentZoneID].kVAh+=ZenergykVAh;
		#endif
		
		if(reset_option==BILL_TYPE_AUTO)
		{
			if((energykVAh*md_period)>InsSave.BillMD.Current.kVA)
			{
				if(InsSave.BillMD.Current.kVA)
				{
					if(InsSave.BillMD.CumkVAMD>=InsSave.BillMD.Current.kVA)
						InsSave.BillMD.CumkVAMD-=InsSave.BillMD.Current.kVA;
				}				
				InsSave.BillMD.Current.kVA=energykVAh*md_period;
				InsSave.BillMD.Current.kVA_Date=logtimeCounter;
				InsSave.BillMD.CumkVAMD+=InsSave.BillMD.Current.kVA;
			}
			#if (defined(GUJRAT_METER_0) && (GUJRAT_METER_0 == 1))
			{
				if((ZenergyExpkVAh*md_period)>InsSave.TODEnergy.TOD[currentZoneID].TODMD.kVA)
				{
				
					InsSave.TODEnergy.TOD[currentZoneID].TODMD.kVA=ZenergyExpkVAh*md_period;
					InsSave.TODEnergy.TOD[currentZoneID].TODMD.kVA_Date=logtimeCounter;
					
				}
			}
			#else
			{
				if((energykVAh*md_period)>InsSave.TODEnergy.TOD[currentZoneID].TODMD.kVA)
				{
				
					InsSave.TODEnergy.TOD[currentZoneID].TODMD.kVA=energykVAh*md_period;
					InsSave.TODEnergy.TOD[currentZoneID].TODMD.kVA_Date=logtimeCounter;
					
				}				
			}
			#endif
		}
		
		if(reset_option==BILL_TYPE_AUTO)
		{
			InsSave.kWhCounterMD=InsSave.kWhCounterMD-energykWh*METER_CONSTANT;
			InsSave.kVAhCounterMD=InsSave.kVAhCounterMD-energykVAh*METER_CONSTANT;
		}
		
		InsSave.ZkWhCounter=InsSave.ZkWhCounter-ZenergykWh*METER_CONSTANT;
		InsSave.ZkVAhCounter=InsSave.ZkVAhCounter-ZenergykVAh*METER_CONSTANT;
		
		InsSave.ExpZkWhCounter=InsSave.ExpZkWhCounter-ZenergyExpkWh*METER_CONSTANT;
		InsSave.ExpZkVAhCounter=InsSave.ExpZkVAhCounter-ZenergyExpkVAh*METER_CONSTANT;
		
		
		InsSave.MonthPowerOnDuration+=InsSave.PowerOn30;
		InsSave.CumPowerOnDuration+=InsSave.PowerOn30;
		InsSave.DailyPowerOnDuration+=InsSave.PowerOn30;
		
#if (defined(OLD_AVG_U_AVG_C_LOGIC) && (OLD_AVG_U_AVG_C_LOGIC == 1))		
		
		if(InsSave.PowerOn30!=0)
			InsSave.lsAvgV=InsSave.lsAvgV/InsSave.PowerOn30;
		if(InsSave.PowerOn30!=0)
			InsSave.lsAvgC=InsSave.lsAvgC/InsSave.PowerOn30;
#endif			
			
		InsSave.PowerOn30=0;
		currentZoneID=getCurrentZoneNo(InsSave.timeCounter);
		
	//--------------------------------------------------
//	write_eeprom(0,RD_TIME_LOC,1);
	
}

//uint8_t  minute_counter;
void Compute_RD_MD(uint8_t reset_option)
{   
		
	uint16_t energykWh=InsSave.kWhCounterMD;
	uint16_t energykVAh=InsSave.kVAhCounterMD;
	
		
		
	//	read_eeprom(minute_counter,RD_TIME_LOC,1);
	//	minute_counter++;
	//	write_eeprom(minute_counter,RD_TIME_LOC,1);


		energykWh=energykWh/METER_CONSTANT;
		energykVAh=energykVAh/METER_CONSTANT;

		Ins_RD_KW.RD_KW=energykWh*md_period;
		Ins_RD_KW.RD_KVA=energykVAh*md_period;
	
}
