//-------- include device specific files----------------
#include "..\\..\\rlDevice\\Include\\dUser_includes.h"
#include "..\\..\\rlDevice\\Include\\dIOdefine.h"
#include "..\\..\\rlDevice\\Include\\dI2c.h"
#include "..\\..\\rlDevice\\Include\\dUart.h"
#include "..\\..\\rlDevice\\Include\\dRtc.h"
#include "..\\..\\rlDevice\\Include\\dWatchDog.h"
#include "..\\..\\rlDevice\\Include\\dIOCtrl.h"
#include "..\\..\\rlDevice\\Include\\d12BitTimer.h"
//-----------------------------------------------------

//-------- include app specific files-------------------
#include "..\\Include\\AppCommunication.h"
#include "..\\Include\\AppCalibration.h"
#include "..\\Include\\AppLcd.h"
#include "..\\Include\\AppVariables.h"
#include "..\\Include\\AppEeprom.h"
#include "..\\Include\\AppMacros.h"
#include "..\\Include\\AppBlockDailyLS.h"
#include "..\\Include\\AppConfig.h"
#include "..\\Include\\AppMisc.h"
#include "..\\include\\AppParaLimits.h"
#include "..\\include\\AppIntervalkWh.h"
#include "..\\include\\AppTampers.h"
#include "..\\Include\\AppCustomerProtocol.h"
#include "..\\Include\\AppBilling.h"



//----------------dlms specific files---------------------------------
#include "..\\..\\rlDlms\\meter_app\\r_dlms_data_meter.h"				/* DLMS Data Definitions */
#include "..\\..\\rlDlms\\meter_app\\r_dlms_user_interface.h"				/* DLMS Data Definitions */
#include "..\\..\\rlDlms\\objects\\r_dlms_obis.h"				/* DLMS Data Definitions */




#define		NO_BYTES		0
#define		HEADER_0        1
#define		HEADER_1        2
#define		HEADER_2        3
#define		DATA_BYTE       4

#define		HEADER_0_CHR	'E'				
#define   	HEADER_1_CHR	'E'
#define   	HEADER_2_CHR	'O'

#define		FOOTER_0		0x0D
#define		FOOTER_1		0x0A

static void addHeaderFooterAndSend(void);
static void ClearEEpromMemory(uint16_t startloc);
static uint32_t getPower2Cal(void);

#define		MACRO_FLASH_NO_OF_CATEGORY	0x410
#define		MACRO_FLASH_FW_VER	0x400

const uint16_t eeprom_default_set[]={
PRE_BILL_SINGLE_ACTION,
PASSIVE_SEA_ACT_TIME,
MD_IP_LOC,
LS_IP,
PRE_BILL_DATE,
TOD_CALANDER_NAME,
ZONE_TIME,
ZONE_TIME2,
ZONE_SEASON,
TOD_PCALANDER_NAME,
PASSIVE_ZONE_TIME,
PASSIVE_ZONE_TIME2,
PASSIVE_ZONE_SEASON,
IS_MEMORY_CLEARED,
DLMS_LLS_SECRET_ADDR,
DLMS_HLS_SECRET_ADDR,
KNO_LOC,
LAG_ONLY_LOC,
};


void Communication(uint8_t isCodeType)
{   
    mcu_flag &= ~COMM_RECEIVE_ENABLE;       //Disable reception
		
    
    if(dUartBuf.RxCount>=8)
    {   
		dUartBuf.RxCount=0;
		Write_Command(isCodeType);
				
    }
    else
    {   
		RxTxBuffer[NO_BYTES] = 5;
		RxTxBuffer[DATA_BYTE] = 'N';
		RxTxBuffer[DATA_BYTE+1] = 'O';
		RxTxBuffer[DATA_BYTE+2] = 'T';
		RxTxBuffer[DATA_BYTE+3] = 'O';
		RxTxBuffer[DATA_BYTE+4] = 'K';
    }
    
	
    mcu_flag |= COMM_RECEIVE_ENABLE;       //Enable reception
}
/*----------------------------------------------------------------------------*/


/***************** Write data into meter **************************************/
void Write_Command(uint8_t isCodeType)
{   
		
	uint32_t pwatt=0;
    uint8_t send_response=0,isdoReset=0;


	if((RxTxBuffer[0]=='E')&&(RxTxBuffer[1]=='E')&&(RxTxBuffer[2]=='O')&&(RxTxBuffer[3]=='Z')&&(RxTxBuffer[4]=='A')&&((RxTxBuffer[5]=='P')||(RxTxBuffer[5]==0))&&(RxTxBuffer[6]==0x0D)&&(RxTxBuffer[7]==0x0A) )
	{
	//	if(isCodeType==PRODUCTION_CODE)
	//	{
			if(RxTxBuffer[5]=='P')
				ClearEEpromMemory(EEP_START_LOC);
			else if(RxTxBuffer[5]==0)
				ClearEEpromMemory(INTERVAL_KWH_LOC);
			send_response=1;
			isdoReset=1;
	//	}

	}
	else if((RxTxBuffer[0]=='E')&&(RxTxBuffer[1]=='E')&&(RxTxBuffer[2]=='O')&&(RxTxBuffer[3]=='R')&&(RxTxBuffer[4]=='S')&&(RxTxBuffer[5]=='T')&&(RxTxBuffer[6]=='R')&&(RxTxBuffer[7]=='T')&&(RxTxBuffer[8]=='C')&&(RxTxBuffer[9]==0x0D)&&(RxTxBuffer[10]==0x0A) )
	{
		init_Rtc(1);
		send_response=1;
	}
	else if((RxTxBuffer[0]=='E')&&(RxTxBuffer[1]=='E')&&(RxTxBuffer[2]=='O')&&(RxTxBuffer[3]=='C')&&(RxTxBuffer[4]=='L')&&(RxTxBuffer[5]=='R')&&(RxTxBuffer[6]=='C')&&(RxTxBuffer[7]=='V')&&(RxTxBuffer[8]=='R')&&(RxTxBuffer[9]==0x0D)&&(RxTxBuffer[10]==0x0A))
	{
		ClearCopen();
		send_response=1;

	}
	else if((RxTxBuffer[0]=='E')&&(RxTxBuffer[1]=='E')&&(RxTxBuffer[2]=='O')&&(RxTxBuffer[3]=='0')&&(RxTxBuffer[4]=='0')&&(RxTxBuffer[5]=='D')&&(RxTxBuffer[6]=='1')&&(RxTxBuffer[13]==0x0D)&&(RxTxBuffer[14]==0x0A) )
	{
		//set time
		Apprtc rtc;
		
		send_response=RxTxBuffer[8]-0x30;
		send_response=(RxTxBuffer[7]-0x30)*10+send_response;
		rtc.sec=send_response;

		send_response=RxTxBuffer[10]-0x30;
		send_response=(RxTxBuffer[9]-0x30)*10+send_response;
		rtc.min=send_response;

		send_response=RxTxBuffer[12]-0x30;
		send_response=(RxTxBuffer[11]-0x30)*10+send_response;
		rtc.hour=send_response;						

		Rtc_SetCounterValue(&rtc,TIME_VAL);
		send_response=1;			
	}
	else if((RxTxBuffer[0]=='E')&&(RxTxBuffer[1]=='E')&&(RxTxBuffer[2]=='O')&&(RxTxBuffer[3]=='0')&&(RxTxBuffer[4]=='0')&&(RxTxBuffer[5]=='D')&&(RxTxBuffer[6]=='0')&&(RxTxBuffer[13]==0x0D)&&(RxTxBuffer[14]==0x0A) )
	{
		//set date
		Apprtc rtc;
		send_response=RxTxBuffer[8]-0x30;
		send_response=(RxTxBuffer[7]-0x30)*10+send_response;
		rtc.day=send_response;

		send_response=RxTxBuffer[10]-0x30;
		send_response=(RxTxBuffer[9]-0x30)*10+send_response;
		rtc.month=send_response;

		send_response=RxTxBuffer[12]-0x30;
		send_response=(RxTxBuffer[11]-0x30)*10+send_response;
		rtc.year=send_response;

		if(isCodeType!=PRODUCTION_CODE)
			logSWTamper(151);
		else
			write_eeprom(&rtc.month,SERIAL_NO_LOC+16,2);
			
		
		Rtc_SetCounterValue(&rtc,DATE_VAL);
		send_response=1;			
	}
	else if((RxTxBuffer[0]=='E')&&(RxTxBuffer[1]=='E')&&(RxTxBuffer[2]=='O')&&(RxTxBuffer[3]=='C')&&(RxTxBuffer[4]=='W')&&(RxTxBuffer[10]==0x0D)&&(RxTxBuffer[11]==0x0A) )
	{
		if(isCodeType==PRODUCTION_CODE)
		{
			read_eeprom((uint8_t*)&pwatt,IS_MEMORY_CLEARED,4);
			if(0x01020304==pwatt)
			{
				pwatt=getPower2Cal();

				if(Ins.EffectiveI>4000)
				{
					if(Ins.PhCurrent>Ins.NeuCurrent)
						doCalibration(0,pwatt);
					else
						doCalibration(1,pwatt);
					send_response=1;
				}
			}
		}

	}
	else if((RxTxBuffer[0]=='E')&&(RxTxBuffer[1]=='E')&&(RxTxBuffer[2]=='O')&&(RxTxBuffer[3]=='C')&&(RxTxBuffer[4]=='P')&&(RxTxBuffer[10]==0x0D)&&(RxTxBuffer[11]==0x0A) )
	{
		if(isCodeType==PRODUCTION_CODE)
		{
			//shunt 0.5 lag
			pwatt=getPower2Cal();

			if(Ins.EffectiveI>4000)
			{
				if(Ins.PhCurrent>Ins.NeuCurrent)
					doCalibration(2,pwatt);
				else
					doCalibration(3,pwatt);

				send_response=1;
			}
		}
	}
	else if((RxTxBuffer[0]=='E')&&(RxTxBuffer[1]=='E')&&(RxTxBuffer[2]=='O')&&(RxTxBuffer[3]=='E')&&(RxTxBuffer[4]=='2')&&(RxTxBuffer[5]=='M')&&(RxTxBuffer[6]=='E')&&(RxTxBuffer[7]=='T')&&(RxTxBuffer[8]=='E')&&(RxTxBuffer[9]=='R')&&(RxTxBuffer[26]==0x0D)&&(RxTxBuffer[27]==0x0A) )
	{
		if(isCodeType!=PRODUCTION_CODE)
		{
			read_eeprom((uint8_t*)&pwatt,IS_MEMORY_CLEARED,4);
			if(0x01020304==pwatt)
			{
			
				write_eeprom(&RxTxBuffer[10],SERIAL_NO_LOC,16);
				pwatt=Crc_Cal(&RxTxBuffer[10],16);
				RxTxBuffer[10]=pwatt;
				RxTxBuffer[11]=pwatt>>8;
				write_eeprom(&RxTxBuffer[10],SERIAL_NO_LOC+18,2);

				RxTxBuffer[NO_BYTES] = getMeterSerial(RxTxBuffer+DATA_BYTE+2,SR_TYPE_ASCII)+2;
				RxTxBuffer[DATA_BYTE]='S';
				RxTxBuffer[DATA_BYTE+1]='R';
				
			}
			else
			{
				RxTxBuffer[NO_BYTES] = 7;
				RxTxBuffer[DATA_BYTE]='N';
				RxTxBuffer[DATA_BYTE+1]='O';
				RxTxBuffer[DATA_BYTE+2]='C';
				RxTxBuffer[DATA_BYTE+3]='L';
				RxTxBuffer[DATA_BYTE+4]='E';
				RxTxBuffer[DATA_BYTE+5]='A';
				RxTxBuffer[DATA_BYTE+6]='R';
				
			}

			send_response=2;
		}
	}
	else if((RxTxBuffer[0]=='E')&&(RxTxBuffer[1]=='E')&&(RxTxBuffer[2]=='O')&&(RxTxBuffer[3]=='B')&&(RxTxBuffer[4]=='M')&&(RxTxBuffer[5]=='B')&&(RxTxBuffer[6]=='Y')&&(RxTxBuffer[11]==0x0D)&&(RxTxBuffer[12]==0x0A) )
	{
		Apprtc rtc;
		send_response=(RxTxBuffer[7]-0x30);
		send_response=send_response*10+(RxTxBuffer[8]-0x30);
		ConvertTimeCounterToTime(InsSave.timeCounter,TIME_VAL,&rtc);
		if(send_response>12)
			send_response=rtc.month;
			
		RxTxBuffer[7]=send_response;
		
		send_response=(RxTxBuffer[9]-0x30);
		send_response=send_response*10+(RxTxBuffer[10]-0x30);
		RxTxBuffer[8]=send_response;
		write_eeprom(&RxTxBuffer[7],SERIAL_NO_LOC+16,2);
		send_response=1;
	}
	else if((RxTxBuffer[0]=='E')&&(RxTxBuffer[1]=='E')&&(RxTxBuffer[2]=='O')&&(RxTxBuffer[3]=='V')&&(RxTxBuffer[4]=='E')&&(RxTxBuffer[5]=='R')&&(RxTxBuffer[6]==0x0D)&&(RxTxBuffer[7]==0x0A))
	{
		RxTxBuffer[NO_BYTES] = 8;
		RxTxBuffer[DATA_BYTE]=*(__far uint8_t*)MACRO_FLASH_FW_VER;
		RxTxBuffer[DATA_BYTE+1]=*(__far uint8_t*)(MACRO_FLASH_FW_VER+1);
		RxTxBuffer[DATA_BYTE+2]=*(__far uint8_t*)(MACRO_FLASH_FW_VER+2);
		RxTxBuffer[DATA_BYTE+3]=*(__far uint8_t*)(MACRO_FLASH_FW_VER+3);
		RxTxBuffer[DATA_BYTE+4]=*(__far uint8_t*)(MACRO_FLASH_FW_VER+4);
		RxTxBuffer[DATA_BYTE+5]=*(__far uint8_t*)(MACRO_FLASH_FW_VER+5);
		RxTxBuffer[DATA_BYTE+6]=*(__far uint8_t*)(MACRO_FLASH_FW_VER+6);
		RxTxBuffer[DATA_BYTE+7]=*(__far uint8_t*)(MACRO_FLASH_FW_VER+7);
		

		send_response=2;
		
	}	
	
	else if((RxTxBuffer[0]=='E')&&(RxTxBuffer[1]=='E')&&(RxTxBuffer[2]=='O')&&(RxTxBuffer[3]=='E')&&(RxTxBuffer[4]=='2')&&(RxTxBuffer[5]=='K')&&(RxTxBuffer[6]=='I')&&(RxTxBuffer[7]=='T')&&(RxTxBuffer[8]=='I')&&(RxTxBuffer[9]=='D')&&(RxTxBuffer[26]==0x0D)&&(RxTxBuffer[27]==0x0A) )
	{
		if(isCodeType==PRODUCTION_CODE)
		{
				write_eeprom(&RxTxBuffer[10],METER_KIT_ID,16);
				pwatt=Crc_Cal(&RxTxBuffer[10],16);
				RxTxBuffer[10]=pwatt;
				RxTxBuffer[11]=pwatt>>8;
				write_eeprom(&RxTxBuffer[10],METER_KIT_ID+18,2);
				RxTxBuffer[NO_BYTES] = getMeterKitID(RxTxBuffer+DATA_BYTE,SR_TYPE_ASCII);
				
				send_response=2;
		}
	}

	else if((RxTxBuffer[0]=='E')&&(RxTxBuffer[1]=='E')&&(RxTxBuffer[2]=='O')&&(RxTxBuffer[3]=='E')&&(RxTxBuffer[4]=='3')&&(RxTxBuffer[5]=='K')&&(RxTxBuffer[6]=='I')&&(RxTxBuffer[7]=='T')&&(RxTxBuffer[8]=='I')&&(RxTxBuffer[9]=='D')&&(RxTxBuffer[10]==0x0D)&&(RxTxBuffer[11]==0x0A))
	{
				RxTxBuffer[NO_BYTES] = getMeterKitID(RxTxBuffer+DATA_BYTE,SR_TYPE_ASCII);
				send_response=2;
	}
	
	else if((RxTxBuffer[0]=='E')&&(RxTxBuffer[1]=='E')&&(RxTxBuffer[2]=='O')&&(RxTxBuffer[3]=='C')&&(RxTxBuffer[4]=='L')&&(RxTxBuffer[5]=='R')&&(RxTxBuffer[6]=='S')&&(RxTxBuffer[7]==0x0D)&&(RxTxBuffer[8]==0x0A) )
	{
		pwatt=0x01020304;
		write_eeprom((uint8_t*)&pwatt,IS_MEMORY_CLEARED,4);
		send_response=1;
	}
	else if((RxTxBuffer[0]=='E')&&(RxTxBuffer[1]=='E')&&(RxTxBuffer[2]=='O')&&(RxTxBuffer[3]=='B')&&(RxTxBuffer[4]=='T')&&(RxTxBuffer[5]==0)&&(RxTxBuffer[6]==0)&&(RxTxBuffer[7]==0x0D)&&(RxTxBuffer[8]==0x0A) )
	{
		#if (defined(IRDA_TYPE_METER_HP) && (IRDA_TYPE_METER_HP == 1))
		DI();
		((void (*) (void)) 0x200e) ();
		#endif
	}	
	else if((RxTxBuffer[0]=='E')&&(RxTxBuffer[1]=='E')&&(RxTxBuffer[2]=='O')&&(RxTxBuffer[3]=='M')&&(RxTxBuffer[4]=='E')&&(RxTxBuffer[5]=='M')&&(RxTxBuffer[6]=='T')&&(RxTxBuffer[7]==0x0D)&&(RxTxBuffer[8]==0x0A))
	{
		if(isCodeType==PRODUCTION_CODE)
		{
		send_response=2;
		setParamIndex(MSG_ALL_INDEX,1);
		
		RxTxBuffer[NO_BYTES] = 5;
		RxTxBuffer[DATA_BYTE]='M';
		RxTxBuffer[DATA_BYTE+1]='E';
		RxTxBuffer[DATA_BYTE+2]='M';
		RxTxBuffer[DATA_BYTE+3]='O';
		RxTxBuffer[DATA_BYTE+4]='K';
		setPulse(1);
		if(memTest()==1)
		{
			send_response=1;
			setParamIndex(MSG_FAIL_INDEX,1);
		}
		refresh_time=10;
		Display(PRODUCTION_CODE);
		}
	}	
	else
	{
		#if (defined(IS_DISPLAY_CONFIGURABLE) && (IS_DISPLAY_CONFIGURABLE == 1))
			if((RxTxBuffer[0]=='E')&&(RxTxBuffer[1]=='E')&&(RxTxBuffer[2]=='O')&&(RxTxBuffer[3]=='R')&&(RxTxBuffer[4]=='G')&&(RxTxBuffer[5]=='E')&&(RxTxBuffer[6]=='T')&&(RxTxBuffer[7]=='D')&&(RxTxBuffer[8]=='P')&&(RxTxBuffer[10]==0x0D)&&(RxTxBuffer[11]==0x0A))
			{
				if(RxTxBuffer[9]=='0'||RxTxBuffer[9]=='1')
				{
					RxTxBuffer[DATA_BYTE+6]=RxTxBuffer[9];
					RxTxBuffer[DATA_BYTE]='R';
					RxTxBuffer[DATA_BYTE+1]='G';
					RxTxBuffer[DATA_BYTE+2]='E';
					RxTxBuffer[DATA_BYTE+3]='T';
					RxTxBuffer[DATA_BYTE+4]='D';
					RxTxBuffer[DATA_BYTE+5]='P';
					
					RxTxBuffer[NO_BYTES] = readConfiguredDP2Send(RxTxBuffer+DATA_BYTE+7,RxTxBuffer[DATA_BYTE+6]-0x30)+7;
					send_response=2;
				}
			}
			else if((RxTxBuffer[0]=='E')&&(RxTxBuffer[1]=='E')&&(RxTxBuffer[2]=='O')&&(RxTxBuffer[3]=='W')&&(RxTxBuffer[4]=='S')&&(RxTxBuffer[5]=='E')&&(RxTxBuffer[6]=='T')&&(RxTxBuffer[7]=='D')&&(RxTxBuffer[8]=='P')&&(RxTxBuffer[138]==0x0D)&&(RxTxBuffer[139]==0x0A))
			{
				if(configDisplayPara(RxTxBuffer+9))
				{
					send_response=1;
					setParaBuff(ModeID);
				}
				
			}
			else if((RxTxBuffer[0]=='E')&&(RxTxBuffer[1]=='E')&&(RxTxBuffer[2]=='O')&&(RxTxBuffer[3]=='W')&&(RxTxBuffer[4]=='D')&&(RxTxBuffer[5]=='D')&&(RxTxBuffer[6]=='D')&&(RxTxBuffer[7]=='P')&&(RxTxBuffer[8]==0x0D)&&(RxTxBuffer[9]==0x0A))
			{
				ResetConfigDP();
				getConfiguredDP(ModeID);
				setParaBuff(ModeID);
				send_response=1;
			}
			#if (defined(IRDA_TYPE_METER_HP) && (IRDA_TYPE_METER_HP == 1))
			else if((RxTxBuffer[0]=='E')&&(RxTxBuffer[1]=='E')&&(RxTxBuffer[2]=='O')&&((RxTxBuffer[3]=='W')||RxTxBuffer[3]=='R')&&(RxTxBuffer[4]=='B')&&(RxTxBuffer[5]=='I')&&(RxTxBuffer[6]=='L')&&(RxTxBuffer[7]=='L')&&(RxTxBuffer[8]=='D')&&(RxTxBuffer[11]==0x0D)&&(RxTxBuffer[12]==0x0A))
			{
				if(RxTxBuffer[3]=='W')
				{
					setBillingDate(&RxTxBuffer[9]);
					send_response=1;
				}
				else if(RxTxBuffer[3]=='R')
				{
					pwatt=getBillingDate();
					
					RxTxBuffer[NO_BYTES] = 8;
					RxTxBuffer[DATA_BYTE]='R';
					RxTxBuffer[DATA_BYTE+1]='B';
					RxTxBuffer[DATA_BYTE+2]='I';
					RxTxBuffer[DATA_BYTE+3]='L';
					RxTxBuffer[DATA_BYTE+4]='L';
					RxTxBuffer[DATA_BYTE+5]='D';
					RxTxBuffer[DATA_BYTE+6]=pwatt>>8;
					RxTxBuffer[DATA_BYTE+7]=pwatt;
					send_response=2;
				}
			}
			else if((RxTxBuffer[0]=='E')&&(RxTxBuffer[1]=='E')&&(RxTxBuffer[2]=='O')&&(RxTxBuffer[3]=='K')&&(RxTxBuffer[4]=='N')&&(RxTxBuffer[5]=='O')&&(RxTxBuffer[6]=='W')&&(RxTxBuffer[17]==0x0D)&&(RxTxBuffer[18]==0x0A))
			{
				setKNO(&RxTxBuffer[7]);
				send_response=1;
			}
			else if((RxTxBuffer[0]=='E')&&(RxTxBuffer[1]=='E')&&(RxTxBuffer[2]=='O')&&(RxTxBuffer[3]=='W')&&(RxTxBuffer[4]=='T')&&(RxTxBuffer[5]=='O')&&(RxTxBuffer[6]=='D')&&(RxTxBuffer[7]=='T')&&(RxTxBuffer[62]==0x0D)&&(RxTxBuffer[63]==0x0A))
			{
				if(setCustomTODTiming(&RxTxBuffer[8])==1)
					send_response=1;
			}
			else if((RxTxBuffer[0]=='E')&&(RxTxBuffer[1]=='E')&&(RxTxBuffer[2]=='O')&&(RxTxBuffer[3]=='R')&&(RxTxBuffer[4]=='T')&&(RxTxBuffer[5]=='O')&&(RxTxBuffer[6]=='D')&&(RxTxBuffer[7]=='T')&&(RxTxBuffer[8]==0x0D)&&(RxTxBuffer[9]==0x0A))
			{
				
				getCustomTODTiming(RxTxBuffer+DATA_BYTE+5);
				RxTxBuffer[DATA_BYTE]='R';
				RxTxBuffer[DATA_BYTE+1]='T';
				RxTxBuffer[DATA_BYTE+2]='O';
				RxTxBuffer[DATA_BYTE+3]='D';
				RxTxBuffer[DATA_BYTE+4]='T';
				RxTxBuffer[NO_BYTES] = 55;
				send_response=2;
			}
			else if((RxTxBuffer[0]=='E')&&(RxTxBuffer[1]=='E')&&(RxTxBuffer[2]=='O')&&(RxTxBuffer[3]=='S')&&(RxTxBuffer[4]=='P')&&(RxTxBuffer[5]=='O')&&(RxTxBuffer[6]=='T')&&(RxTxBuffer[7]=='B')&&(RxTxBuffer[8]=='I')&&(RxTxBuffer[9]=='L')&&(RxTxBuffer[10]=='L')&&(RxTxBuffer[11]==0x0D)&&(RxTxBuffer[12]==0x0A) )
			{
				getSPOTBill();	
				send_response=2;
			}			
			else if((RxTxBuffer[0]=='E')&&(RxTxBuffer[1]=='E')&&(RxTxBuffer[2]=='O')&&(RxTxBuffer[3]=='E')&&(RxTxBuffer[4]=='P')&&(RxTxBuffer[5]=='R')&&(RxTxBuffer[8]==0x0D)&&(RxTxBuffer[9]==0x0A))
			{
				pwatt=RxTxBuffer[7];
				pwatt<<=8;
				pwatt|=RxTxBuffer[6];
				RxTxBuffer[NO_BYTES] = 33;
				RxTxBuffer[DATA_BYTE]='#';
				
				read_eeprom(&RxTxBuffer[DATA_BYTE+1],pwatt,32);
				send_response=2;
			}			
			
			#endif
			
		#endif
	}
	
	if(send_response==1)
	{			
		
		RxTxBuffer[NO_BYTES] = 2;
		RxTxBuffer[DATA_BYTE] = 'O';
		RxTxBuffer[DATA_BYTE+1] = 'K';
		
		addHeaderFooterAndSend();
	}
	else  if(send_response==2)
	{
		addHeaderFooterAndSend();
	}
	if(isdoReset==1)
		WDT_Restart(0x00);
	   
}
/*----------------------------------------------------------------------------*/

static void addHeaderFooterAndSend(void)
{
	RxTxBuffer[HEADER_0] = HEADER_0_CHR;
	RxTxBuffer[HEADER_1] = HEADER_1_CHR;
	RxTxBuffer[HEADER_2] = HEADER_2_CHR;
	RxTxBuffer[DATA_BYTE+RxTxBuffer[NO_BYTES]] = FOOTER_0;
	RxTxBuffer[DATA_BYTE+RxTxBuffer[NO_BYTES]+1] = FOOTER_1;

	RxTxBuffer[NO_BYTES] = RxTxBuffer[NO_BYTES] +5;
	
	
	if(received_uart==0)
	{
	#if (defined(UART_0_ENABLE) && (UART_0_ENABLE == 1))
		Meter_Response(&RxTxBuffer[HEADER_0],RxTxBuffer[NO_BYTES]);
	#endif
	}
	#if (defined(UART_2_ENABLE) && (UART_2_ENABLE == 1))
	else
	{
		Meter2_Response(&RxTxBuffer[HEADER_0],RxTxBuffer[NO_BYTES]);
	}
	#endif
	
}

static void ClearEEpromMemory(uint16_t startloc)
{
		uint16_t i;
		uint8_t indexno=0;
		uint8_t no_of_category;
		uint8_t category_para_index;
	
		setParamIndex(MSG_ERASE_CNT_INDEX,1);
		Ins.Voltage=0;

		if(memTest()==1)
		{
			setParamIndex(MSG_FAIL_INDEX,1);
			Display(PRODUCTION_CODE);
			return;
		}
		fillComBufferZero();

		for(i=startloc;i<=EEP_END_LOC;)
		{   
			WDT_Restart(0xAC);
			write_eeprom(RxTxBuffer,i,EEP_PAGE_LENGTH);							//erase all eeprom
			read_eeprom(RxTxBuffer,i,EEP_PAGE_LENGTH);							//erase all eeprom
			for(indexno=0;indexno<EEP_PAGE_LENGTH;indexno++)
			{
				if(RxTxBuffer[indexno]!=0x00)
				{
					setParamIndex(MSG_FAIL_INDEX,1);
					Display(PRODUCTION_CODE);
					while(1)
					{
						WDT_Restart(0xAC);
						if (mcu_flag&POWER_DOWN_FLAG)
							break;
						if(mcu_flag&PUSH_BUTTON_RELEASED)
							break;
					}
				}
			}
			i+=EEP_PAGE_LENGTH;							
			Ins.Voltage = i/EEP_PAGE_LENGTH;
			Display(PRODUCTION_CODE);							
		}
		
		setParamIndex(MSG_DONE_INDEX,1);
		Display(PRODUCTION_CODE);

		
		i=MACRO_FLASH_NO_OF_CATEGORY+1;
		
		for(indexno=0;indexno< *(__far uint8_t*)MACRO_FLASH_NO_OF_CATEGORY;indexno++)
		{
			WDT_Restart(0xAC);
			no_of_category=*(__far uint8_t*)i;
			
			for(category_para_index=0;category_para_index<no_of_category;category_para_index++)    // copy from flash to ram
				RxTxBuffer[category_para_index] = *(__far  uint8_t*)(i+1+category_para_index);
				
			write_eeprom(RxTxBuffer,eeprom_default_set[indexno],no_of_category);

			i=i+no_of_category+1;
			
		}
		
		ITMC&=~0x80;	// stop 12 bit timer
		TMKAEN=0;

		for(indexno=4;indexno<PFAILSAVEDATA_LEN;indexno++)
			*((uint8_t*)&InsSave.timeCounter+indexno)=0x00;
		InsSave.timeCounter=InsSave.timeCounter-(InsSave.timeCounter%DAY_CROSSOVER_SECONDS);
		SaveEEPROM(PARA_WRITE_AFTER_STATE ,PARA_TYPE_PFAIL_INSTANT); // save at current location
		//----------------------------------------------------------------------
}


uint8_t getSerialCrc(void)
{
	uint16_t crc;
	uint8_t status=0;
	
	read_eeprom(RxTxBuffer,SERIAL_NO_LOC,16);
	read_eeprom((uint8_t*)&crc,SERIAL_NO_LOC+18,2);
	if(Crc_Cal(RxTxBuffer,16)==crc)
		status=1;
	return status;
}
#if (defined(IRDA_TYPE_METER_HP) && (IRDA_TYPE_METER_HP == 1))
void getSPOTBill(void)
{
	uint8_t bytes;
	uint8_t *bptr=(RxTxBuffer+DATA_BYTE);
	uint8_t buff[BILLING_DATA_LENGTH];
	//serial number
	bytes= getMeterSerial(bptr,SR_TYPE_ASCII);
    bptr[bytes]=FIELD_SEPERATOR;
	bytes+=1;
	bytes+=getCustomDateTimeASCIIFormat(InsSave.timeCounter,bptr+bytes,6);
    bptr[bytes]=FIELD_SEPERATOR;
	bytes+=1;
	// current kWh
    Dec_Ascii(InsSave.CumkWh+InsSave.ZkWhCounter/METER_CONSTANT,8,bptr+bytes,2);
	bytes=bytes+9;
    bptr[bytes]=FIELD_SEPERATOR;
	bytes+=1;	
	// current kW MD
    Dec_Ascii(InsSave.BillMD.Current.kW,4,bptr+bytes,2);
	bytes=bytes+5;
    bptr[bytes]=FIELD_SEPERATOR;
	bytes+=1;	
	
	read_eeprom(buff,GetBillLoc(0),BILLING_DATA_LENGTH);
	//H1 Date
	bytes+=getCustomDateTimeASCIIFormat(getByteValue(buff,25,4),bptr+bytes,0);
	//H1 kWh
    Dec_Ascii(getByteValue(buff,0,4),8,bptr+bytes,2);
	bytes=bytes+9;
    bptr[bytes]=FIELD_SEPERATOR;
	bytes+=1;	
	//H1 kW MD
    Dec_Ascii(getByteValue(buff,12,2),4,bptr+bytes,2);
	bytes=bytes+5;
	RxTxBuffer[NO_BYTES]=bytes;
}
#endif
static uint32_t getPower2Cal(void)
{
	uint32_t pwatt;
	pwatt= RxTxBuffer[9]-0x30;
	pwatt=(RxTxBuffer[8]-0x30)*10+pwatt;
	pwatt=(RxTxBuffer[7]-0x30)*100+pwatt;
	pwatt=(RxTxBuffer[6]-0x30)*1000+pwatt;
	pwatt=(RxTxBuffer[5]-0x30)*10000+pwatt;
	
	return	pwatt;
	
}


