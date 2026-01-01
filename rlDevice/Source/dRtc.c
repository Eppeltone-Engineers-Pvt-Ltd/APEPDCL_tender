#include "..\\include\\dIODefine.h"
#include "..\\include\\dUser_includes.h"
#include "..\\include\\dRtc.h"
#include "..\\include\\dTypedef.h"
#include "..\\include\\dMisc.h"
#include "..\\include\\dIOCtrl.h"
#include "..\\include\\dClock.h"


//-------- include app specific files----------------
#include "..\\..\\rlApplication\\Include\\AppMacros.h"
#include "..\\..\\rlApplication\\Include\\AppVariables.h"
#include "..\\..\\rlApplication\\Include\\AppMisc.h"
#include "..\\..\\rlApplication\\Include\\AppLcd.h"
#include "..\\..\\rlApplication\\Include\\AppConfig.h"
//------------------------------------------------------

#define	_BCD2DEC(x)		{x = (x & 0xF) + (x >> 4) * 10;}				/* 1 byte BCD to DEC Conversion */
#define	_DEC2BCD(x)		{x = (x % 10 ) | (((x / 10) << 4) & 0xF0);}		/* 1 byte DEC to BCD Conversion */


uint8_t init_Rtc(uint8_t ioption)
{
    volatile uint8_t tmp;
	uint8_t rtc_reset_status=0;

    /* Supply clock to access RTC domain register */
    VRTCEN = 1U;
    
    /* Disable interrupts */
    RTCAMK = 1U;
    RTCAIF = 0U;
    RTCRMK = 1U;
    RTCRIF = 0U;
    
    /* Set INTRTCRPD low priority */
    RTCRPR1 = 1U;
    RTCRPR0 = 1U;
    
    /* Set ALM interrupt priority low level */
    RTCAPR1 = 1U;
    RTCAPR0 = 1U;
    /* Set RPD interrupt priority low level */
    RTCRPR1 = 1U;
    RTCRPR0 = 1U;
    
  	
 
    /* Initialization when POR */
    if ((RTCPORSR == 0)||(ioption==1))
    {

        /* Only set register if VRTC enough */
        if (LVDVRTCF == 0 )
        {
            /* If this is power on reset, init the time */
            /* Clock setting to use 32KHz source */
            RCR4 = _00_RTC_SELECT_FSUB;
            
            /* Set global RTCIC enable: RTCICEN bit 0 */
            tmp = RCR3;
            tmp |= _01_RTC_INPUT_ENABLE;
            RCR3 = tmp;
            
            /* Stop all counters : START is bit 0 */
            tmp = RCR2;
            tmp &= (uint8_t)~_01_RTC_COUNTER_NORMAL;
            RCR2 = tmp;
            while (RCR2 != tmp);
            
            /* Select count mode : CNTMD is bit 7 */
            tmp = RCR2;
            tmp &= (uint8_t)~_80_RTC_BINARY_MODE;
            RCR2 = tmp;
            while (RCR2 != tmp);
            
            /* Execute RTC software reset : RESET is bit 1 */
            tmp = RCR2;
            /* Clear bits that will be cleared by RTC Software Reset */
            tmp &= (uint8_t)~(_20_RTC_CALENDER_10SECONDS | _10_RTC_ADJUSTMENT_ENABLE | _04_RTC_WRITER_EXECUTED);
            /* Start reset */
            tmp |= _02_RTC_RESET_WRITER_INITIALIZED;
            RCR2 = tmp;
            /* Reset bit will be clear to 0 after finish reset, clear this bit to check */
            tmp &= (uint8_t)~_02_RTC_RESET_WRITER_INITIALIZED;
            while (RCR2 != tmp);
            
            /* Set control registers 1 */
			
            tmp = _04_RTC_PERIODIC_ENABLE | _E0_RTC_INTRTCPRD_CLOCK_8 | _08_RTC_64HZ_OUTPUT | _00_RTC_ALARM_DISABLE;
            RCR1 = tmp;
            while (RCR1 != tmp);
            
            /* Set control registers 2 */
            tmp = _00_RTC_CALENDER_MODE | _40_RTC_24HOUR_MODE | _00_RTC_RTCOUT_DISABLE;
            RCR2 = tmp;
            while (RCR2 != tmp);
            /* Start the RTC once */
            tmp = RCR2;
            tmp |= _01_RTC_COUNTER_NORMAL;
            RCR2 = tmp;
            while (RCR2 != tmp);
            validateRCR2();
			validateRCR1();
            /* Unlock, set and lock the RCR5 */
            RCR5GD = 0x00U;
            RCR5GD = 0x72U;
            RCR5GD = 0x64U;
            RCR5 = 0x00U;
            RCR5GD = 0x00U;
            
            /* Mark RTC initialized */
            RTCPORSR = 1U;
			rtc_reset_status=1;
        }
    }
	
	//RTCRIF = 0U;    /* clear INTRTCRPD interrupt flag */
    //RTCRMK = 0U;    /* enable INTRTCRPD interrupt */
	//VRTCEN = 0U;
	return rtc_reset_status;
	
}

void start_RTC_Second_Interrupt(void)
{
    //RTCIF = 0U;    /* clear INTRTC interrupt flag */
    //RTCMK = 0U;    /* enable INTRTC interrupt */
	RTCRMK=0;
}
void stop_RTC_Second_Interrupt(void)
{
	//RTCMK = 1U;    /* disable INTRTC interrupt */
	//NOP();
	//RTCIF=0U;
	//RIFG=0U;
	RTCRMK=1;
}


uint32_t getRtcCounter(uint8_t op_mode)
{
    
	Apprtc counter_read_val;
	uint8_t wait,no_of_registers=0;
	uint32_t preTime;
	
	VRTCEN = 1U;
	while(++no_of_registers<8)
	{
		wait=0xFF;
		while(wait--)
		{
			RSR_bit.no1=0;
			switch(no_of_registers)
			{
				case 1:
					counter_read_val.sec = RSECCNT;
					break;
				case 2:
					counter_read_val.min = RMINCNT;
					break;
				case 3:
					counter_read_val.hour = RHRCNT&0x3F;;
					break;
				case 4:
					counter_read_val.week = RWKCNT;
					break;
				case 5:
					counter_read_val.day = RDAYCNT;
					break;
				case 6:
					counter_read_val.month = RMONCNT;
					break;
				case 7:
					counter_read_val.year = RYRCNT;
					break;
					
			}
			if(RSR_bit.no1==0)
				break;
		}
	}
	
	/* Convert all to decimal */
	_BCD2DEC(counter_read_val.sec);
	_BCD2DEC(counter_read_val.min);
	_BCD2DEC(counter_read_val.hour);
	_BCD2DEC(counter_read_val.day);
	_BCD2DEC(counter_read_val.week);
	_BCD2DEC(counter_read_val.month);
	_BCD2DEC(counter_read_val.year);
		

	

	preTime=ConvertTimeToTimeCounter(&counter_read_val);
	if(BATTERY_MODE==op_mode)
	{
		if(preTime>InsSave.timeCounter)
		{
			if((preTime-InsSave.timeCounter)>1800)
				InsSave.timeCounter=preTime;				
		}
		else
		{
			if((InsSave.timeCounter-preTime)>1800)
				InsSave.timeCounter=preTime;				
			
		}
	}
	
	if(RTC_WRITE_MODE!=op_mode)
	{
		if(InsSave.timeCounter<(preTime+120))
			InsSave.timeCounter=preTime;
	}
	else
		InsSave.timeCounter=preTime;
		
	return preTime;
		
	//VRTCEN = 0U;
}

//***********************************************************************************************************************/
void Rtc_SetCounterValue(const Apprtc *counter_write_val,uint8_t setType)
{
	
	Apprtc wRtc;
    volatile uint8_t tmp;
    volatile uint8_t bcount;
    
	/* Only set register if VRTC enough */				
	if (0U == LVDVRTCF)
	{
		/* Supply clock to access RTC domain register */
		VRTCEN = 1U;

		/* Stop all counters : START is bit 0 */
		tmp = RCR2;
		tmp &= (uint8_t)~_01_RTC_COUNTER_NORMAL;
		RCR2 = tmp;
		while (RCR2 != tmp);
	
		if(TIME_VAL==setType)
		{
			wRtc.sec=counter_write_val->sec;
			wRtc.min=counter_write_val->min;
			wRtc.hour=counter_write_val->hour;

			_DEC2BCD(wRtc.sec);
			_DEC2BCD(wRtc.min);
			_DEC2BCD(wRtc.hour);

			RSECCNT = wRtc.sec;
			RMINCNT = wRtc.min;
			RHRCNT = wRtc.hour;
		}
		if(DATE_VAL==setType)
		{
			/* Convert to BCD */
			wRtc.day=counter_write_val->day;
			wRtc.week=RWKCNT;
			wRtc.month=counter_write_val->month;
			wRtc.year=counter_write_val->year;

			_DEC2BCD(wRtc.day);
			_DEC2BCD(wRtc.week);
			_DEC2BCD(wRtc.month);
			_DEC2BCD(wRtc.year);

			RWKCNT = wRtc.week;
			RDAYCNT = wRtc.day;
			RMONCNT = wRtc.month;
			RYRCNT = wRtc.year;
		}
    
	    /* Restart counting : START is bit 0 */
	    tmp = RCR2;
	    tmp |= _01_RTC_COUNTER_NORMAL;
	    RCR2 = tmp;
	    while (RCR2 != tmp);
		//VRTCEN = 0U;
		
		getRtcCounter(RTC_WRITE_MODE);
    }

}


#pragma interrupt Interrupt_RTC_SECOND(vect=INTRTCPRD,bank=RB2)
static void Interrupt_RTC_SECOND(void)
{
	
	/* Check constant period interrupt flag */
//	RTCWEN = 1U;
//    if (1U == RIFG)
//    {
//        //RTCC1 &= (uint8_t)~_08_RTC_INTC_GENERATE_FLAG;    /* clear RIFG */
		mcu_flag|=RTC_SECOND_FLAG;
//    }
//	RTCWEN = 0U;

}

//void setRTCOut(uint8_t flag)
//{
//	volatile uint8_t tmp=RCR2;	
//	RTC_OUT_DIR&=~RTC_OUT_BIT;	
//	if(flag)
//	{
//		tmp|=_08_RTC_RTCOUT_ENABLE;
//	}
//	else
//	{
		
//		tmp|=_00_RTC_RTCOUT_DISABLE;
		
//		RTC_OUT&=~RTC_OUT_BIT;
//	}
//    RCR2 = tmp;
//    while (RCR2 != tmp);
//	CKS0=0x88;
	

//}

void reEnable32KCrystal(void)
{

	if(XTSTOP==1)
	{
		SWITCH_ON_RTC_VDD;
		power_fail_state|=RTC_BAT_STATUS_FLAG;
		power_fail_state&=~CRYSTAL_REENABLED_FLAG;
	}
		
	if((power_fail_state&CRYSTAL_REENABLED_FLAG)==0)
	{
		
		MCU_Delay(1000);
		SCMC = _10_CGC_SUB_OSC | _02_CGC_NORMAL_OSCILLATION;
		XTSTOP = 0U;    /* XT1 oscillator operating */
		power_fail_state|=CRYSTAL_REENABLED_FLAG;
		init_Rtc(0);
		
	}
	
}
void validateRCR1(void)
{
	uint8_t rcr1_val;
	uint8_t validVal;
	VRTCEN = 1U;
	MCU_Delay(10);
	
	/* Set control registers 1 */
	validVal = _04_RTC_PERIODIC_ENABLE | _E0_RTC_INTRTCPRD_CLOCK_8 | _08_RTC_64HZ_OUTPUT | _00_RTC_ALARM_DISABLE;
	rcr1_val = RCR1;
	if(validVal != rcr1_val)
	{
		RCR1=validVal;
		while (validVal != RCR1);
	}
		
	
}
void validateRCR2(void)
{
 uint8_t rcr2_val;           /* Stop all counters : START is bit 0 */
 uint8_t validVal;

	VRTCEN = 1U;
	MCU_Delay(10);

	validVal=_40_RTC_24HOUR_MODE|_20_RTC_CALENDER_10SECONDS|_10_RTC_ADJUSTMENT_ENABLE|_01_RTC_COUNTER_NORMAL;
	rcr2_val = RCR2;
	rcr2_val&=~_08_RTC_RTCOUT_ENABLE;
 
	if((rcr2_val&validVal)!=validVal)
	{

		// stop counter
		rcr2_val = RCR2;
		rcr2_val &= (uint8_t)~_01_RTC_COUNTER_NORMAL;
		RCR2 = rcr2_val;
		while (RCR2 != rcr2_val);
		RADJ=0x00;
		/* Select count mode : CNTMD is bit 7 */
		rcr2_val = RCR2;
		rcr2_val &= (uint8_t)~_80_RTC_BINARY_MODE;
		RCR2 = rcr2_val;
		while (RCR2 != rcr2_val);

		/* Set control registers 2 */
		rcr2_val = RCR2;
		rcr2_val&=~_08_RTC_RTCOUT_ENABLE;
		rcr2_val |= (_00_RTC_CALENDER_MODE | _40_RTC_24HOUR_MODE|_20_RTC_CALENDER_10SECONDS|_10_RTC_ADJUSTMENT_ENABLE) ;
		RCR2 = rcr2_val;

		while (RCR2 != rcr2_val);

		/* Start the RTC once */
		rcr2_val = RCR2;
		rcr2_val |= _01_RTC_COUNTER_NORMAL;
		RCR2 = rcr2_val;
		while (RCR2 != rcr2_val);
		RADJ=0x91;

	}
            
	
}
