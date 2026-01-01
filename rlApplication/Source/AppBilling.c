//-------- include device specific files----------------

#include "..\\..\\rlDevice\\Include\\dI2c.h"
//-----------------------------------------------------

//-------- include app specific files-------------------
#include "..\\Include\\AppBilling.h"
#include "..\\Include\\AppVariables.h"
#include "..\\Include\\AppEeprom.h"
#include "..\\Include\\AppLcd.h"
#include "..\\Include\\AppMD.h"
#include "..\\Include\\AppMisc.h"
#include "..\\Include\\AppMacros.h"
#include "..\\Include\\AppTOD.h"
#include "..\\Include\\AppOneSecondTask.h"
#include "..\\Include\\AppBlockDailyLS.h"
#include "..\\Include\\AppConfig.h"

//----------------dlms specific files---------------------------------
#include "..\\..\\rlDlms\\meter_app\\r_dlms_data_meter.h"

void CheckBillingOver(uint8_t resetType,uint32_t billdate)
{   
	uint32_t prevEnergy[2];
	uint8_t avg_pf;
	uint16_t loc;
	uint8_t preSeason=0;
	uint8_t RxTxBuffer_tamper[1];
	

      
      
	read_eeprom((uint8_t*)prevEnergy,GetBillLoc(0),8);

	avg_pf=0;
	if(InsSave.CumkVAh>prevEnergy[1])
		avg_pf=(InsSave.CumkWh-prevEnergy[0])*100/(InsSave.CumkVAh-prevEnergy[1]);
		
	if(avg_pf>100)
		avg_pf=100;
	
	#if (defined(IRDA_TYPE_METER_HP) && (IRDA_TYPE_METER_HP == 1))
		if(InsSave.CumkWh>=prevEnergy[0])
			prevEnergy[0]=InsSave.CumkWh-prevEnergy[0];
		else
			prevEnergy[0]=0;
			
		if(InsSave.CumkVAh>=prevEnergy[1])
			prevEnergy[1]=InsSave.CumkVAh-prevEnergy[1];
		else
			prevEnergy[1]=0;		
	#endif


	makeByte(InsSave.CumkWh,0,4); //4 bytes
	makeByte(InsSave.CumkVAh,4,4); //4 bytes
	makeByte(InsSave.MonthPowerOnDuration,8,4); //4 bytes
	makeByte(InsSave.BillMD.Current.kW,12,2); //2 bytes
	makeByte(InsSave.BillMD.Current.kW_Date,14,4); //4 bytes
	makeByte(InsSave.BillMD.Current.kVA,18,2); //2 bytes
	makeByte(InsSave.BillMD.Current.kVA_Date,20,4); //4 bytes
	RxTxBuffer[24]=avg_pf;                            //1 byte
	makeByte(billdate ,25,4);                       //4 bytes
	RxTxBuffer[29]=resetType;       //reset type                  //1 byte
	#if (defined(IRDA_TYPE_METER_HP) && (IRDA_TYPE_METER_HP == 0))
		makeByte(InsSave.ExpCumkWh,30,4); //4 bytes
		makeByte(InsSave.ExpCumkVAh,34,4); //4 bytes
	#elif (defined(IRDA_TYPE_METER_HP) && (IRDA_TYPE_METER_HP == 1))
		makeByte(prevEnergy[0],31,4); //4 bytes Cum kWh
		makeByte(prevEnergy[1],35,4); //4 bytes Cum kVAh
	#endif
	
	//read_eeprom((uint8_t *)&Data.long_data,TAMPER_STATUS_TEMP,1);
 // status_temp=Data.byte[0];
//	makeByte(TamperRecord.Tamper_Once,39,1);	//1 byte //2 bytes tamper status
	makeByte(TamperRecord_AP.Tamper_Once,39,1);	//1 byte //2 bytes tamper status
		//makeByte(TamperRecord.TamperStatusFlag,39,2);	
	//makeByte(0xFF,39,1);	//2 bytes tamper status		
	makeByte(0,40,1);	//2 bytes tamper status		

	loc=BILL_DATA_LOC+(InsSave.MDResetCount%HISTORY_RECORD)*BILLING_DATA_LENGTH;
	write_eeprom(RxTxBuffer,loc,BILLING_DATA_LENGTH);  //update the billing kwh
	TodBilling();
	InsSave.MDResetCount++;
      
	//-------------------------------------------------------------------------------------------------

	// cummulative md
	//InsSave.BillMD.CumkWMD+=InsSave.BillMD.Current.kW;
	//InsSave.BillMD.CumkVAMD+=InsSave.BillMD.Current.kVA;
	
	//-------- clear mds after billing done-------------
	InsSave.BillMD.Current.kW=0;
	InsSave.BillMD.Current.kVA=0;
	InsSave.BillMD.Current.kW_Date=0;
	InsSave.BillMD.Current.kVA_Date=0;
	InsSave.MonthPowerOnDuration=0;
//	TamperRecord.Tamper_Once=0;	
	//TamperRecord.Tamper_Once=TamperRecord.bill_tamper_status;
	TamperRecord_AP.Tamper_Once=TamperRecord_AP.bill_tamper_status;
	
	//--------------------------------------------------
	preSeason=currentSeasonID;
	currentSeasonID=getCurrentSeason(InsSave.timeCounter);
	if(preSeason!=currentSeasonID)
	{
		currentZoneID=getCurrentZoneNo(InsSave.timeCounter);
	}
	
			
	R_OBIS_Class07_BillingUpdate();
//#if(defined(IRDA_TYPE_METER_AP) && (IRDA_TYPE_METER_AP == 1))	
//	getlatestkWh_History();
//#endif	
}
/*--------------------------------------------------------------------------*/

uint16_t GetBillLoc(uint8_t eventno)
{   
  int16_t addr; 
  
  addr=(InsSave.MDResetCount%HISTORY_RECORD)-eventno % HISTORY_RECORD;
  if(addr<=0)
    addr=HISTORY_RECORD+addr;
  
  addr=addr-1;
  addr=BILL_DATA_LOC+addr*BILLING_DATA_LENGTH; 
  return addr; 
}

uint16_t GetLastBillLoc(uint8_t eventno)
{   
  uint16_t addr; 
  
	if(InsSave.MDResetCount>HISTORY_RECORD)
		addr=InsSave.MDResetCount%HISTORY_RECORD;
	else
		addr=0;

	addr=addr+eventno;
	addr=addr%HISTORY_RECORD;
	
	addr=BILL_DATA_LOC+addr*BILLING_DATA_LENGTH; 
	
	
  return addr; 
}
/*--------------------------------------------------------------------------*/

void doBilling(void)
{
 
  prevTimeCounter=InsSave.timeCounter;
	OneSecondTask();
   
}


uint8_t getLastBillnumber(void)
{
	uint8_t billno=0;
	
	if(InsSave.MDResetCount>HISTORY_RECORD)
		billno=HISTORY_RECORD-1;
	else if(InsSave.MDResetCount>0)
		billno=InsSave.MDResetCount-1;
	return billno;
}
uint8_t getAvgPf(void)
{
	uint32_t prevEnergy[2];
	uint32_t curkWh=InsSave.CumkWh+InsSave.ZkWhCounter/METER_CONSTANT;
	uint32_t curkVAh=InsSave.CumkVAh+InsSave.ZkVAhCounter/METER_CONSTANT;
  	uint8_t avg_pf;
  
		
	read_eeprom((uint8_t*)prevEnergy,GetBillLoc(0),8);

	avg_pf=0;

	
	if(curkVAh>prevEnergy[1])
		avg_pf=(curkWh-prevEnergy[0])*100/(curkVAh-prevEnergy[1]);	
	if(avg_pf>100)
		avg_pf=100;
	
	return avg_pf;
}

void initPowerOnDuration(void)
{
	//read_eeprom((unsigned char*)&DLMS_MonthPowerON_Time,GetMdLoc(0),4);
	//DLMS_MonthPowerON_Time=DLMS_MonthPowerON_Time/60;
	//read_eeprom((unsigned char*)&DLMS_PowerON_Time,GetMdLoc(0)+MD_LEN+8,4);
	//DLMS_PowerON_Time=DLMS_PowerON_Time/60;
}

uint32_t getNextBillingDate(void)
{
		uint32_t preBillDateTime,nextBilldateTime;
		Apprtc rtc;
    	ConvertTimeCounterToTime(InsSave.timeCounter,TIME_VAL,&rtc);
	
		rtc.day=getAutoBillDay();
		
	
		rtc.sec=0x00;
		rtc.min=0x00;
		rtc.hour=0x00;
		nextBilldateTime=ConvertTimeToTimeCounter(&rtc); //next billing date   
		
	
		if(InsSave.MDResetCount==0)
		{
			read_eeprom((uint8_t*)&preBillDateTime,GetLsLoadLoc(0),4);
		}
		else
		{
			read_eeprom((uint8_t*)&preBillDateTime,GetBillLoc(0)+25,4);
		}
		
		preBillDateTime=preBillDateTime-(preBillDateTime%DAY_CROSSOVER_SECONDS);
		
		if((nextBilldateTime<=preBillDateTime)||(InsSave.timeCounter<nextBilldateTime))
			nextBilldateTime=0;
			
		
		return nextBilldateTime;
}
void ClearBillMD(void)
{
	uint8_t no_of_zones,max_zones=8;
	for(no_of_zones=0;no_of_zones<max_zones;no_of_zones++)
	{
		InsSave.TODEnergy.TOD[no_of_zones].TODMD.kW=0;
		InsSave.TODEnergy.TOD[no_of_zones].TODMD.kVA=0;
		InsSave.TODEnergy.TOD[no_of_zones].TODMD.kW_Date=0;
		InsSave.TODEnergy.TOD[no_of_zones].TODMD.kVA_Date=0;
	}
	
}

//uint32_t GetPastMD(uint8_t valueType,uint8_t EnergyType)
//{
//	uint32_t pastMD[2];
//	pastMD[0]=0;
//	pastMD[1]=0;
//	read_eeprom((uint8_t*)&pastMD[0],GetBillLoc(0)+12+EnergyType*6,2);
//	read_eeprom((uint8_t*)&pastMD[1],GetBillLoc(1)+12+EnergyType*6,2);
	
//	if(valueType==DATE_VAL)
//	{
//		if(pastMD[0]>=pastMD[1])
//			read_eeprom((uint8_t*)&pastMD[0],GetBillLoc(0)+14+EnergyType*6,4);
//		else
//			read_eeprom((uint8_t*)&pastMD[0],GetBillLoc(1)+14+EnergyType*6,4);
//	}
//	else
//	{
//		if(pastMD[1]>=pastMD[0])
//			pastMD[0]=pastMD[1];
//	}
	
//	return pastMD[0];
	
//}


uint8_t getAutoBillDay(void)
{
	uint8_t billday=1;
	read_eeprom((uint8_t*)&billday,PRE_BILL_DATE,1);
	if(billday>31)
		billday=1;
	return billday;
	
}
#if 0
#if(defined(IRDA_TYPE_METER_AP) && (IRDA_TYPE_METER_AP == 1))
void getlatestkWh_History(void)
{
	uint8_t billno=0;
	uint16_t loc;
	while(billno<6)
	{
		loc=GetBillLoc(billno++);
		read_eeprom((uint8_t*)&kWh_History,loc,4);
		read_eeprom((uint8_t*)&kW_MD_History,loc+12,2);
	}
	
}
#endif
#endif
