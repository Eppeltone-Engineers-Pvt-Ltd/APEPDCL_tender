//-------- include device specific files----------------
#include "..\\..\\rlDevice\\Include\\dI2c.h"
//-----------------------------------------------------

//-------- include app specific files-------------------
#include "..\\Include\\AppTod.h"
#include "..\\Include\\AppVariables.h"
#include "..\\Include\\AppEeprom.h"
#include "..\\Include\\AppLcd.h"
#include "..\\Include\\AppMD.h"
#include "..\\Include\\AppConfig.h"
#include "..\\Include\\AppMisc.h"
#include "..\\Include\\AppBilling.h"

/**************** Billing TOD Data ********************************************/
void TodBilling(void)
{ 
	uint8_t i;
	uint16_t billloc;
  
	billloc=TOD_BILL_LOC+(InsSave.MDResetCount%HISTORY_RECORD)*TOD_LEN;
	write_eeprom((uint8_t*)&InsSave.TODEnergy.TOD[0].TODMD.kW,billloc,TOD_LEN);
  
	for(i=0;i<8;i++)
	{
		InsSave.TODEnergy.TOD[i].TODMD.kW=0x00;
		InsSave.TODEnergy.TOD[i].TODMD.kW_Date=0x00;
		InsSave.TODEnergy.TOD[i].TODMD.kVA=0x00;
		InsSave.TODEnergy.TOD[i].TODMD.kVA_Date=0x00;
	}
}
/*----------------------------------------------------------------------------*/

uint8_t getCurrentSeason(uint32_t timeZoneCounter)
{
	uint8_t newseason=0;
	uint32_t seasonStartTime1=0;
	uint32_t seasonStartTime2=0;
	Apprtc rtc;
	
	ConvertTimeCounterToTime(InsSave.timeCounter,TIME_VAL,&rtc);

	read_eeprom((uint8_t*)&newseason,ZONE_SEASON+1,1);
	rtc.day=getAutoBillDay();
	rtc.month=newseason;
	rtc.sec=0x00;
	rtc.min=0x00;
	rtc.hour=0x00;
	
	seasonStartTime1=ConvertTimeToTimeCounter(&rtc);
	
	read_eeprom((uint8_t*)&newseason,ZONE_SEASON+3,1);

	rtc.day=getAutoBillDay();
	rtc.month=newseason;
	
	rtc.sec=0x00;
	rtc.min=0x00;
	rtc.hour=0x00;
	
	seasonStartTime2=ConvertTimeToTimeCounter(&rtc);

	if(getMaxSeasonNo()>1)
	{
		newseason=0;
		if(timeZoneCounter>=seasonStartTime1)
			read_eeprom((uint8_t*)&newseason,ZONE_SEASON+5,1);
			
		if(timeZoneCounter>=seasonStartTime2)
			read_eeprom((uint8_t*)&newseason,ZONE_SEASON+6,1);
		
			
	}
	else
		newseason=1;
	
	newseason=newseason-1;
	
	return (newseason%2);
		
}

/*uint8_t isZoneChanged(uint32_t timeZoneCounter)
{
	uint8_t newZoneId;
	newZoneId=getCurrentZoneNo(timeZoneCounter);
	if(newZoneId!=currentZoneID)
		return 1;
	else 
		return 0;
}*/
uint8_t getCurrentZoneNo(uint32_t timeZoneCounter)
{
  uint16_t EEPROMZoneTime[8];
  uint16_t ZoneTime[4],dayCounter;
  uint8_t ZoneNo,MaxZoneNo;
  
  dayCounter=(timeZoneCounter/60) % 1440;
  
  read_eeprom((uint8_t *)&EEPROMZoneTime,ZONE_TIME+currentSeasonID*25,16);
  MaxZoneNo=getMaxZoneNo(currentSeasonID);
  ZoneTime[2]=0;
  ZoneTime[3]=0;
  for(ZoneNo=1;ZoneNo<MaxZoneNo;ZoneNo++)
  {
    ZoneTime[0]=EEPROMZoneTime[ZoneNo-1];
    ZoneTime[1]=EEPROMZoneTime[ZoneNo];
    if(ZoneTime[1]<ZoneTime[0])
    {
      ZoneTime[3]=ZoneTime[1];
      ZoneTime[1]=1440;
      ZoneTime[2]=0;
    }
    if((dayCounter>=ZoneTime[0]&&dayCounter<ZoneTime[1])||(dayCounter>=ZoneTime[2]&&dayCounter<ZoneTime[3]))
    break;
  }
	if(currentSeasonID==0)
		read_eeprom((uint8_t *)&ZoneNo,ZONE_NO+ZoneNo,1);
	else
		read_eeprom((uint8_t *)&ZoneNo,ZONE_NO2+ZoneNo,1);
		
  ZoneNo=ZoneNo-1;
  return (ZoneNo%8);
}
/*--------------------------------------------------------------------------*/

uint16_t GetTODBillLoc(uint8_t eventno)
{   
	
  uint16_t addr; 
  
	if(InsSave.MDResetCount>HISTORY_RECORD)
		addr=InsSave.MDResetCount%HISTORY_RECORD;
	else
		addr=0;

	addr=addr+eventno;
	addr=addr%HISTORY_RECORD;
	
	addr=TOD_BILL_LOC+addr*TOD_LEN; 
	
  return addr; 
}

uint8_t getMaxZoneNo(uint8_t seasonNO)
{
  uint8_t MaxZoneNo=1;
  read_eeprom((uint8_t *)&MaxZoneNo,ZONE_TIME+seasonNO*25+16,1);

  if(MaxZoneNo>MAX_ZONE_NO)
    MaxZoneNo=MAX_ZONE_NO;    
	
  return MaxZoneNo;
}

uint8_t getMaxSeasonNo(void)
{
  uint8_t MaxSeasonNo=1;
  
  read_eeprom((uint8_t *)&MaxSeasonNo,ZONE_SEASON+4,1);

  if(MaxSeasonNo>2)
    MaxSeasonNo=2;    
	
  return MaxSeasonNo;
}

uint32_t getTODConsumption(uint8_t zoneNo,uint8_t EnergyType)
{
	uint32_t curEnergy=0;
  
	if(ENERGY_TYPE_KWH==EnergyType)	
		curEnergy=InsSave.TODEnergy.TOD[zoneNo].kWh;
	else
		curEnergy=InsSave.TODEnergy.TOD[zoneNo].kVAh;
		
	if(currentZoneID==zoneNo)
	{
		if(ENERGY_TYPE_KWH==EnergyType)	
			curEnergy+=InsSave.ZkWhCounter/METER_CONSTANT;	
		else
			curEnergy+=InsSave.ZkVAhCounter/METER_CONSTANT;	
		
	}

	return curEnergy;
}

uint16_t GetTODLatestBillLoc(uint8_t eventno)
{   
  int16_t addr; 
  
  addr=(InsSave.MDResetCount%HISTORY_RECORD)-eventno % HISTORY_RECORD;
  if(addr<=0)
    addr=HISTORY_RECORD+addr;
  
  addr=addr-1;
  addr=TOD_BILL_LOC+addr*TOD_LEN; 
  return addr; 
}
