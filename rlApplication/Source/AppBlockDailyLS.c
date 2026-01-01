//-------- include device specific files----------------
#include "..\\..\\rlDevice\\Include\\dI2c.h"
//-----------------------------------------------------

//-------- include app specific files-------------------
#include "..\\Include\\AppBlockDailyLS.h"
#include "..\\Include\\AppVariables.h"
#include "..\\Include\\AppEeprom.h"
#include "..\\Include\\AppMisc.h"
#include "..\\Include\\AppConfig.h"

//-------- include dlms specific files-------------------
#include "..\\..\\rlDlms\\meter_app\\r_dlms_data_meter.h"

const uint16_t gbltotaldays[]={LS_TOTAL_DAYS,LS_TOTAL_DAYS/2};
/*--------------------------------------------------------------------------*/
void Load_Survey(void)
{   
	
	uint16_t loc;
	uint8_t lslength=0;
	uint16_t energykWh=InsSave.kWhCounter/METER_CONSTANT;
	uint16_t energykVAh=InsSave.kVAhCounter/METER_CONSTANT;
	uint16_t energyExpkWh=InsSave.ExpkWhCounter/320;
	uint16_t ls_day_length=24*ls_period*LS_EVENT_LENGTH+4; 
	uint32_t lsValues;
		
	

	loc=LOAD_SURVEY_LOC+ (InsSave.DayCount%gbltotaldays[ls_period/3])*ls_day_length;
	

    
	prevTimeCounter=prevTimeCounter+(3600/ls_period );
	if(InsSave.IntCount==0)
	{

		makeByte(prevTimeCounter ,lslength,4);
		lslength=4;
	}
	else
		loc=loc+InsSave.IntCount*LS_EVENT_LENGTH+4;
		
		
	if(energykWh>energykVAh)
	{
		energykVAh=energykWh;
		InsSave.kVAhCounter=InsSave.kWhCounter;

	}
		
#if (defined(OLD_AVG_U_AVG_C_LOGIC) && (OLD_AVG_U_AVG_C_LOGIC == 1))	
	if(InsSave.PowerOn30!=0)
		InsSave.lsAvgV=InsSave.lsAvgV/InsSave.PowerOn30;
	if(InsSave.PowerOn30!=0)
		InsSave.lsAvgC=InsSave.lsAvgC/InsSave.PowerOn30;
	
#else
	InsSave.lsAvgV=InsSave.lsAvgV/(3600/ls_period);
	InsSave.lsAvgC=InsSave.lsAvgC/(3600/ls_period);
#endif	
	
	InsSave.lsAvgC=InsSave.lsAvgC/100;
		
	makeByte(InsSave.lsAvgV/200,lslength,1);
	lslength=lslength+1;

	lsValues=(InsSave.lsAvgC&0x0FFF);	//12 bits
	lsValues<<=12;
	lsValues|=(energykWh&0x0FFF);			//12 bits
	
	
	
	makeByte(lsValues,lslength,3);
	lslength=lslength+3;
	makeByte(energykVAh,lslength,2);
	lslength=lslength+2;

	write_eeprom(RxTxBuffer,loc,lslength);
		
  
	InsSave.IntCount++;	
	if(InsSave.IntCount>=24*ls_period)
	{
		logDaily00hhEnergy();
	}
		

	InsSave.kWhCounter =InsSave.kWhCounter-energykWh*METER_CONSTANT;
	InsSave.kVAhCounter =InsSave.kVAhCounter-energykVAh*METER_CONSTANT;
	InsSave.ExpkWhCounter =InsSave.ExpkWhCounter-energyExpkWh*320;
		
	InsSave.MonthPowerOnDuration+=InsSave.PowerOn30;
	InsSave.CumPowerOnDuration+=InsSave.PowerOn30;
	InsSave.DailyPowerOnDuration+=InsSave.PowerOn30;
		
	InsSave.lsAvgV=0;
	InsSave.lsAvgC=0;
	InsSave.PowerOn30=0;
	


}

/*--------------------------------------------------------------------------*/
uint16_t GetLsLoadLoc(uint8_t dayno)
{ 
	uint16_t addr;
	uint16_t ls_day_length=24*ls_period*LS_EVENT_LENGTH+4; 

	if(InsSave.DayCount>=gbltotaldays[ls_period/3])
	{
		if(InsSave.IntCount==0)
			addr=InsSave.DayCount%gbltotaldays[ls_period/3];
		else
			addr=InsSave.DayCount%gbltotaldays[ls_period/3]+1;
	}
	else
		addr=0;

	addr=addr+dayno;
	addr=addr%gbltotaldays[ls_period/3];

	addr=LOAD_SURVEY_LOC+addr*ls_day_length; 
	return addr; 
}
/*--------------------------------------------------------------------------*/
uint16_t GetDailyLoc(uint16_t dayno)
{ 
	uint16_t addr;


	if(InsSave.DailyDayCount>DAILYLS_TOTAL_DAYS)
		addr=InsSave.DailyDayCount%DAILYLS_TOTAL_DAYS;
	else
		addr=0;
	
	

	addr=addr+dayno;
	addr=addr%DAILYLS_TOTAL_DAYS;

	addr=DAILY_KWH_LOC+addr*LS_DAILY_LENGTH; 

	return addr; 
	
}

/*--------------------------------------------------------------------------*/
void doLs(void)
{
uint32_t backuptimeCounter =prevTimeCounter; 
	if((InsSave.DailyDayCount==0)&&((InsSave.IntCount==0))&&(prevTimeCounter==0))
		prevTimeCounter=InsSave.timeCounter-InsSave.timeCounter%86400;

	
	prevTimeCounter=prevTimeCounter-(prevTimeCounter%(3600/ls_period));
	
	while(InsSave.timeCounter/(3600/ls_period)!=prevTimeCounter/(3600/ls_period))
	{

		Load_Survey();
		
	}
	
	prevTimeCounter=backuptimeCounter;
	
}

uint16_t getLsEntries(void)
{
	
uint16_t entries;


	if(InsSave.DayCount>=gbltotaldays[ls_period/3])
	{
		if(InsSave.IntCount>0)
			entries=gbltotaldays[ls_period/3]-1;
		else
			entries=gbltotaldays[ls_period/3];
	}
	else
		entries=InsSave.DayCount;

	entries=entries*24*ls_period+InsSave.IntCount;

	return entries;

}


void ClearLsData(void)
{
	uint8_t lsDayno;
	uint32_t backuplsAvgV=0;
	uint32_t backuplsAvgC=0;
	uint16_t energykWh=InsSave.kWhCounter;
	uint16_t energykVAh=InsSave.kVAhCounter;
	
	
	backuplsAvgV=InsSave.lsAvgV;
	backuplsAvgC=InsSave.lsAvgC;
	
	fillComBufferZero();

	for(lsDayno=0;lsDayno<LS_TOTAL_DAYS;lsDayno++ )
		write_eeprom((uint8_t *)&RxTxBuffer ,LOAD_SURVEY_LOC+lsDayno*64,64);

	prevTimeCounter=InsSave.timeCounter-(InsSave.timeCounter%DAY_CROSSOVER_SECONDS);
	InsSave.IntCount=0;
	InsSave.DayCount=0;
	SaveEEPROM(PARA_WRITE_BEFORE_STATE ,PARA_TYPE_INSTANT);	
	InsSave.lsAvgV=0;
	InsSave.lsAvgC=0;
	InsSave.kWhCounter=0;
	InsSave.kVAhCounter=0;
	
	doLs(); 

	InsSave.lsAvgV=backuplsAvgV;
	InsSave.lsAvgC=backuplsAvgC;
	InsSave.kWhCounter=energykWh;
	InsSave.kVAhCounter=energykVAh;
	
	prevTimeCounter=InsSave.timeCounter;
	SaveEEPROM(PARA_WRITE_AFTER_STATE  ,PARA_TYPE_INSTANT); 
	SaveEEPROM(PARA_WRITE_AFTER_STATE ,PARA_TYPE_PFAIL_INSTANT);
	R_OBIS_Class07_BlockloadUpdate();	
}

void getByRange(uint8_t obj_id, uint32_t time_start, uint32_t time_end, uint32_t *idx_start, uint32_t *idx_stop)
{
	uint32_t start=0;
	uint32_t period;
	uint16_t loc = 0,baseloc=LOAD_SURVEY_LOC;
	uint8_t entryavailable=0;
	uint16_t totaldays=gbltotaldays[ls_period/3];
	uint16_t day_length=24*ls_period*LS_EVENT_LENGTH+4; 
	
	uint16_t startEntry=0,totalEntry=getLsEntries();
	
	*idx_start=0;
	*idx_stop=0;
	
	if(obj_id == 2)
	{
		loc = GetLsLoadLoc(0); // oldest day location
		period=3600/ls_period;
		

		
	}
	else if(obj_id == 4)
	{
		loc = GetDailyLoc(0);	// oldest day location
		period=86400;
		baseloc=DAILY_KWH_LOC;
		totaldays=DAILYLS_TOTAL_DAYS;
		day_length=LS_DAILY_LENGTH;
		if(InsSave.DailyDayCount>DAILYLS_TOTAL_DAYS)
			totalEntry=DAILYLS_TOTAL_DAYS;
		else
			totalEntry=InsSave.DailyDayCount;
		
		
	}
	read_eeprom((uint8_t*) &start, loc, 4);	// oldest day date-time
	time_start=time_start-time_start%period;
	time_end=time_end-time_end%period;

	
	for(startEntry=0;startEntry<totalEntry;startEntry++)
	{
		
		if((start>=time_start)&&(entryavailable==0))
		{
			*idx_start=startEntry+1;
			entryavailable=1;
		}
		if((start<=time_end)&&(entryavailable))
		{
			*idx_stop=startEntry+1;
			
		}
		if(start%86400==0)
		{
			loc=loc+day_length;
			if(loc>=(baseloc+totaldays*day_length))
				loc=baseloc;
			read_eeprom((uint8_t*) &start, loc, 4);	// oldest day date-time
		}
		else
		{
			if(obj_id==2)
				start=start+period;
		}
	}
	
	if(*idx_stop<*idx_start)
	{
		*idx_stop=0;
		*idx_start=0;
	}
}

void logDaily00hhEnergy(void)
{
	uint16_t loc;
	loc=DAILY_KWH_LOC+(InsSave.DailyDayCount%DAILYLS_TOTAL_DAYS)*LS_DAILY_LENGTH;
	prevTimeCounter=InsSave.timeCounter-(InsSave.timeCounter%DAY_CROSSOVER_SECONDS);
	InsSave.DailyPowerOnDuration+=InsSave.PowerOn30;
	makeByte(prevTimeCounter,0,4);
	makeByte(InsSave.CumkWh ,4,4);
	makeByte(InsSave.CumkVAh ,8,4);
	makeByte(InsSave.daykWMD,12,2);
	makeByte(InsSave.DailyPowerOnDuration,14,4); //4 bytes
//	makeByte(InsSave.daykWMD_dt,14,4);
//	makeByte(InsSave.ExpCumkWh ,12,4);
	write_eeprom(RxTxBuffer,loc,LS_DAILY_LENGTH);
	InsSave.DayCount++;
	InsSave.IntCount=0;
	InsSave.DailyDayCount++;
	InsSave.daykWMD=0;
	InsSave.DailyPowerOnDuration=0;
	
}
