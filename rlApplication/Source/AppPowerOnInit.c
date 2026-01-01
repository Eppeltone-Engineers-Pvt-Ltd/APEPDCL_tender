//-------- include device specific files----------------
#include "..\\..\\rlDevice\\Include\\dClock.h"
#include "..\\..\\rlDevice\\Include\\dDisplay.h"
#include "..\\..\\rlDevice\\Include\\d8bitTimer.h"
#include "..\\..\\rlDevice\\Include\\dRtc.h"
#include "..\\..\\rlDevice\\Include\\dI2c.h"
#include "..\\..\\rlDevice\\Include\\dUser_includes.h"
#include "..\\..\\rlDevice\\Include\\dUart.h"
#include "..\\..\\rlDevice\\Include\\d12bitTimer.h"
#include "..\\..\\rlDevice\\Include\\dDeltaSigmaADC.h"
#include "..\\..\\rlDevice\\Include\\dIOCtrl.h"
#include "..\\..\\rlDevice\\Include\\dWatchDog.h"
#include "..\\..\\rlDevice\\include\\dVBat.h"
#include "..\\..\\rlDevice\\include\\dMisc.h"
#include "..\\..\\rlDevice\\include\\dOsd.h"
#include "..\\..\\rlDevice\\include\\dIODefine.h"

//-----------------------------------------------------

//-------- include app specific files-------------------
#include "..\\Include\\AppPowerOnInit.h"
#include "..\\Include\\AppEeprom.h"
#include "..\\Include\\AppMisc.h"
#include "..\\Include\\AppVariables.h"
#include "..\\Include\\AppIntervalkWh.h"
#include "..\\Include\\AppBilling.h"
#include "..\\Include\\AppTod.h"
#include "..\\Include\\AppLcd.h"
#include "..\\Include\\AppProductionProcess.h"
#include "..\\Include\\AppConfig.h"
#include "..\\Include\\AppCommunication.h"



void R_PORT_Create(void);
static uint32_t getTODEnergy(uint8_t EnergyType);
static void UpdateEnergy(void);
void production_prepareLowPower(uint8_t initRestoreValue);
#if(defined(IRDA_TYPE_METER_AP) && (IRDA_TYPE_METER_AP == 1))
//void getSerailforRam(void);
#endif
//------------------------------------------------------

 

void init_poweron(void)
{
	//unsigned char iTemp;
	//EnableVBATOperation();
	
	DisableWDTInterrupt();
	init_device_hw();
	//isRAMOK(); due to no init of inssave

	
	
	//--------i2c test-------
//	for(iTemp=0;iTemp<64;iTemp++)
//		RxTxBuffer[iTemp]=iTemp;
		
//	write_eeprom(RxTxBuffer,0x00,32);
//	write_eeprom(&RxTxBuffer[32],32,32);
//	for(iTemp=0;iTemp<64;iTemp++)
//		RxTxBuffer[iTemp]=0x00;
	
//	read_eeprom(RxTxBuffer,0x00,32);
//	read_eeprom(&RxTxBuffer[32],32,32);
	
	//---------------

		
	if(IS_PRODUCTION_MODE)
	{
		if(((getSerialCrc()==1)&&(GET_COVER_STATUS))||(getSerialCrc()==0))
			production_process();
	}
	
	init_app_data();
	setParaBuff(DISPLAY_MODE_NORMAL);
	refresh_time=10;
	if((mcu_flag&POWER_DOWN_FLAG)==0)
		Display(USER_CODE);
	
	isRAMOK();
}

void init_device_hw(void)
{
	uint8_t rtc_reset_status=0;
	mcu_flag=0;
	
	R_PORT_Create();
	// rtc vdd pin
	SET_PORT_OUT_MODE(RTC_VDD_DIR,RTC_VDD,RTC_VDD_BIT,RTC_OFF);
	SWITCH_ON_RTC_VDD;
	
	
	setClock(0);

	rtc_reset_status=init_Rtc(0);
	SWITCH_OFF_RTC_VDD;	
	
	init_OSD();
	
	init_Lcd();
	


	init_Timer8bit0(1);
	
	

	
	
	
	init_I2c();
	
	#if (defined(UART_0_ENABLE) && (UART_0_ENABLE == 1))
	init_Uart0();
	#endif
	#if (defined(UART_2_ENABLE) && (UART_2_ENABLE == 1))
	init_Uart2();
	#endif
	
	init_Timer12bit(1);
	
	init_Ports();
	
	EI();
	
	if(GET_MAINS_STATUS==0)
	{
		production_prepareLowPower(RESTORE_VALUE_PFAIL_RESET);	
	}	
		
	init_DeltaSigmaADC();
	
	


}
void init_app_data(void)
{
	
	uint16_t loc;
	
	//read_eeprom(RxTxBuffer,PFAIL_INS_SAVE_LOC,PFAILSAVEDATA_LEN);
	loc=getSetPfailData(ENERGY_R_RAM);
	if(loc==1)
	{
		for(loc=0;loc<PFAILSAVEDATA_LEN;loc++)
			RxTxBuffer[loc]=*((uint8_t*)&InsSave.timeCounter+loc);
	}
	SaveEEPROM(PARA_READ_STATE,PARA_TYPE_INSTANT); // this will read before loc still if crc fails it will read another location
	
	if(Crc_Cal(RxTxBuffer,PFAILSAVEDATA_LEN)==0)
	{
		
		for(loc=0;loc<PFAILSAVEDATA_LEN;loc++)
			*((uint8_t*)&InsSave.timeCounter+loc)=RxTxBuffer[loc];
	}
	else
	{
		
		InsSave.kWhCounter=0;
		InsSave.ZkWhCounter=0;
		InsSave.kWhCounterMD=0;
		
		InsSave.kVAhCounter=0;
		InsSave.ZkVAhCounter=0;
		InsSave.kVAhCounterMD=0;
		
		InsSave.ExpZkWhCounter=0;
		InsSave.ExpZkVAhCounter=0;
		InsSave.ExpkWhCounter=0;
		
	}
		
	SaveEEPROM(PARA_READ_STATE,PARA_TYPE_TAMPER ); // load tamper data
	TamperRecord.Tamper_Once=TamperRecord.bill_tamper_status;
	SaveEEPROM(PARA_READ_STATE,PARA_TYPE_TAMPER_AP ); // load tamper data
	TamperRecord_AP.Tamper_Once=TamperRecord_AP.bill_tamper_status;
	
	prevTimeCounter=InsSave.timeCounter;
	

	prekWh = readIntervalkWh();
	prekWhinterval=prekWh;
	
	currentSeasonID=getCurrentSeason(InsSave.timeCounter);
	currentZoneID=getCurrentZoneNo(InsSave.timeCounter);
	

	if(prekWh)
	{
		if((InsSave.CumkWh+InsSave.ZkWhCounter/METER_CONSTANT) >=prekWh)
		{
			if(!(((InsSave.CumkWh+InsSave.ZkWhCounter/METER_CONSTANT) - prekWh) <=MAX_KWH_PER_INTERVAL))
			{
				UpdateEnergy();
			}
		}
		else
		{
			UpdateEnergy();
		}
	}
	
	prekWh=InsSave.CumkWh+InsSave.ZkWhCounter/METER_CONSTANT;
	
	
	if(prekWhinterval>prekWh)
		prekWhinterval=prekWh;
	
	
	if(InsSave.CumkWh>InsSave.CumkVAh)
	{
		InsSave.CumkVAh=InsSave.CumkWh;
		if(InsSave.CumkVAh>getTODEnergy(1))
			InsSave.TODEnergy.TOD[currentZoneID].kVAh=InsSave.CumkVAh-getTODEnergy(1);
	}
		 
	//----------------------------------------------------------------
	
	getRtcCounter(RTC_NO_CHECK_MODE);
	
	read_eeprom((uint8_t *)&loc,MD_IP_LOC,2);
	switch(loc)
	{
		case 3600:
			md_period=1;
		break;		
		case 1800:
			md_period=2;
		break;		
		case 900:
			md_period=4;
		break;		
		default:
			md_period=2;
		break;				
	}
	new_md_period=updateMDInterval(MD_PERIOD_READ_FLAG,md_period);
	
	read_eeprom((uint8_t *)&ls_period,LS_IP,1);
	switch(ls_period)
	{
		case 1:
		case 2:
		case 4:
			//do nothing
			break;
		default:
			ls_period=2;
		break;				
	}
	
	
	read_eeprom((uint8_t *)&ScheduleActionDateTime.BillDateTime,PRE_BILL_SINGLE_ACTION,4);	
//	read_eeprom((uint8_t *)&Data.long_data,TAMPER_STATUS_TEMP,1);	

	//-----------------------------------
	
	loc=0;
	
	read_eeprom((uint8_t *)&loc,PRE_BILL_SINGLE_ACTION+4,1);
	
	if(loc==0xAB)
		mcu_flag|=SINGLE_ACTION_CHECK_FLAG;	
	loc=0;
	read_eeprom((uint8_t *)&loc,PASSIVE_SEA_ACT_TIME+4,1);
	
	if(loc==0xAB)
		mcu_flag|=PASSIVE_CAL_CHECK_FLAG;
	
	read_eeprom((uint8_t *)&ScheduleActionDateTime.CalanderDateTime,PASSIVE_SEA_ACT_TIME,4);	

	read_eeprom((uint8_t *)&nmInfo,NM_INFO_SHT_LOC,12);
	getSetPfailData(ENERGY_W_RAM);
	#if(defined(IRDA_TYPE_METER_AP) && (IRDA_TYPE_METER_AP == 1))
	//	getSerailforRam();
	#endif	
//	 read_eeprom((uint8_t *)&Data.long_data,MAGNET_PERMANENT_SAVE,1);
		read_eeprom((uint8_t *)&lag_only_value,LAG_ONLY_LOC,1);

	CurrentTamperStatusFlag = TamperRecord.TamperStatusFlag;
}

static uint32_t getTODEnergy(uint8_t EnergyType)
{
	uint32_t energySum=0;
	
	if(EnergyType==0)
	{
		for(EnergyType=0;EnergyType<8;EnergyType++)
		{
			if(EnergyType!=currentZoneID)
				energySum+=InsSave.TODEnergy.TOD[EnergyType].kWh;
		}
	}
	else if(EnergyType==1)
	{
		for(EnergyType=0;EnergyType<8;EnergyType++)
		{
			if(EnergyType!=currentZoneID)
				energySum+=InsSave.TODEnergy.TOD[EnergyType].kVAh;
		}
	}
		
	return energySum;
}

static void UpdateEnergy(void)
{
	InsSave.CumkWh = prekWh;
	if(InsSave.CumkWh>getTODEnergy(0))
		InsSave.TODEnergy.TOD[currentZoneID].kWh=InsSave.CumkWh-getTODEnergy(0);
		
	
	
	InsSave.kWhCounter=0;
	InsSave.ZkWhCounter=0;
	InsSave.kWhCounterMD=0;
	
	InsSave.kVAhCounter=0;
	InsSave.ZkVAhCounter=0;
	InsSave.kVAhCounterMD=0;
	
}

void noResetInit(void)
{
	
#if (defined(UART_0_ENABLE) && (UART_0_ENABLE == 1))		
	init_Uart0();
#endif	
#if (defined(UART_2_ENABLE) && (UART_2_ENABLE == 1))
	init_Uart2();
#endif
	
	init_Timer12bit(1);
	
	init_Ports();
	if(((TRTCMP0/256)!=1))
		start_Timer8bit0(1);
	setParaBuff(DISPLAY_MODE_NORMAL);
	refresh_time=10;
	
	if((mcu_flag&POWER_DOWN_FLAG)==0)
		Display(USER_CODE);
	
	CheckCurrentLimitNm();		
	R_LCD_PowerOn();
	
	isRAMOK();
	
	init_DeltaSigmaADC();	
	CurrentTamperStatusFlag = TamperRecord.TamperStatusFlag;

}

void CheckCurrentLimitNm(void)
{
	if((Ins.EffectiveI<800)||(is_vfail_with_load==0))
	{
		Ins.PhCurrent=0;
		Ins.NeuCurrent=0;
		Ins.EffectiveI=0;
		apperentPowerNM=0;
		activePowerNM=0;
	}
	
}
#if 0
#if(defined(IRDA_TYPE_METER_AP) && (IRDA_TYPE_METER_AP == 1))
void getSerailforRam(void)
{
	uint8_t sr_digits,digit_no=0;
	sr_digits=getMeterSerial(RxTxBuffer,SR_TYPE_LCD)%9;
	
	gblserialnumberLCD[0]=0x00;
	gblserialnumberLCD[1]=0x00;
	gblserialnumberLCD[2]=0x00;
	gblserialnumberLCD[3]=0x00;
	gblserialnumberLCD[4]=0x00;
	gblserialnumberLCD[5]=0x00;
	gblserialnumberLCD[6]=0x00;
	gblserialnumberLCD[7]=0x00;
	digit_no=0;
	while(digit_no<sr_digits)
	{
		gblserialnumberLCD[digit_no]=RxTxBuffer[9-digit_no];
		digit_no++;
	}
	getlatestkWh_History();
}
#endif
#endif
