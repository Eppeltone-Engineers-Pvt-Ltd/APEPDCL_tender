//-------- include device specific files----------------
#include "..\\..\\rlDevice\\Include\\dI2c.h"

//-------- include app specific files-------------------
#include "..\\Include\\AppEeprom.h"
#include "..\\Include\\AppMisc.h"
#include "..\\Include\\AppVariables.h"
#include "..\\Include\\AppIntervalkWh.h"


static uint8_t intervalCount=0;
void savekWh(uint32_t kwh)
{
	uint16_t crc;
	uint8_t kwhbuff[8];
	kwhbuff[0]=kwh;
	kwhbuff[1]=kwh>>8;
	kwhbuff[2]=kwh>>16;
	kwhbuff[3]=kwh>>24;
	crc=Crc_Cal(kwhbuff,4);
	kwhbuff[4]=crc;
	kwhbuff[5]=crc>>8;

	if(intervalCount>=MAX_INTERVAL_COUNT)
		intervalCount=0;
	
	write_eeprom(kwhbuff ,INTERVAL_KWH_LOC+intervalCount*6,6);
	intervalCount++;
}



uint32_t readIntervalkWh(void)
{
	uint32_t prevkWh=0,currkWh=0,iTemp=0;
	uint8_t crc_ok_count=0, jTemp,isOK=1;
	int8_t index=0;
	uint8_t kwhBuffer[MAX_INTERVAL_COUNT*6];
	intervalCount=0;
	
	read_eeprom(kwhBuffer ,INTERVAL_KWH_LOC,MAX_INTERVAL_COUNT*6);
	while(iTemp<MAX_INTERVAL_COUNT)
	{
	
		if(!Crc_Cal(&kwhBuffer[iTemp*6],6))
		{
			
			kwhBuffer[crc_ok_count*5]=kwhBuffer[iTemp*6];	
			kwhBuffer[crc_ok_count*5+1]=kwhBuffer[iTemp*6+1];
			kwhBuffer[crc_ok_count*5+2]=kwhBuffer[iTemp*6+2];
			kwhBuffer[crc_ok_count*5+3]=kwhBuffer[iTemp*6+3];
			kwhBuffer[crc_ok_count*5+4]=iTemp; 
			crc_ok_count++;
		}

		iTemp++;
	}
	
	for(jTemp=0;jTemp<crc_ok_count;jTemp++ )
	{
		Data.byte[0]=kwhBuffer[jTemp*5] ;
		Data.byte[1]=kwhBuffer[jTemp*5+1] ;
		Data.byte[2]=kwhBuffer[jTemp*5+2] ;
		Data.byte[3]=kwhBuffer[jTemp*5+3] ;
		index=kwhBuffer[jTemp*5+4];
		currkWh=Data.long_data;   
		for(iTemp=jTemp+1;iTemp<crc_ok_count;iTemp++ )
		{
			Data.byte[0]=kwhBuffer[iTemp*5] ;
			Data.byte[1]=kwhBuffer[iTemp*5+1] ;
			Data.byte[2]=kwhBuffer[iTemp*5+2] ;
			Data.byte[3]=kwhBuffer[iTemp*5+3] ;
			
			if(currkWh<Data.long_data )
			{
		
				prevkWh=Data.long_data;  
				kwhBuffer[jTemp*5]=Data.byte[0] ;
				kwhBuffer[jTemp*5+1]=Data.byte[1] ;
				kwhBuffer[jTemp*5+2]=Data.byte[2] ;
				kwhBuffer[jTemp*5+3]=Data.byte[3] ;
				
				kwhBuffer[jTemp*5+4]=kwhBuffer[iTemp*5+4];
				
				Data.long_data=currkWh;
				
				kwhBuffer[iTemp*5]=Data.byte[0] ;
				kwhBuffer[iTemp*5+1]=Data.byte[1] ;
				kwhBuffer[iTemp*5+2]=Data.byte[2] ;
				kwhBuffer[iTemp*5+3]=Data.byte[3] ;				
				kwhBuffer[iTemp*5+4]=index;
				index=kwhBuffer[jTemp*5+4];
				currkWh=prevkWh; 
			}
			
		}
	}
		
	for(iTemp=0;iTemp<crc_ok_count;iTemp++   )
	{
		Data.byte[0] =kwhBuffer[iTemp*5];
		Data.byte[1] =kwhBuffer[iTemp*5+1];
		Data.byte[2] =kwhBuffer[iTemp*5+2];
		Data.byte[3] =kwhBuffer[iTemp*5+3];
		
		
		currkWh=Data.long_data;
		intervalCount=kwhBuffer[iTemp*5+4];
		isOK=1;
		for(jTemp=iTemp+1;jTemp<crc_ok_count;jTemp++   )
		{
				Data.byte[0] =kwhBuffer[jTemp*5];
				Data.byte[1] =kwhBuffer[jTemp*5+1];
				Data.byte[2] =kwhBuffer[jTemp*5+2];
				Data.byte[3] =kwhBuffer[jTemp*5+3];
			
				prevkWh=Data.long_data;
				if(currkWh>(prevkWh+MAX_KWH_PER_INTERVAL*jTemp ))
					isOK=0;
		}
		if(isOK==1 )
			break;
	}
	intervalCount++; 	
	return currkWh; 
}

//uint8_t getInterval(void)
//{
//	return intervalCount;
//}