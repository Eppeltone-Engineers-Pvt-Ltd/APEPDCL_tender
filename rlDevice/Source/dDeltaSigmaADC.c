//-------- include device specific files----------------
#include "..\\include\\dIODefine.h"
#include "..\\include\\dUser_includes.h"
#include "..\\include\\dDeltaSigmaADC.h"
#include "..\\include\\dI2c.h"
#include "..\\include\\dIOCtrl.h"
#include "..\\Include\\dWatchDog.h"
#include "..\\Include\\dMisc.h"
#include "..\\DCImmunity\\dDCImmunity.h"
#include "..\\DCImmunity\\rl78_sw_dc_correction.h"


//-------- include app specific files----------------
#include "..\\..\\rlApplication\\Include\\AppMacros.h"
#include "..\\..\\rlApplication\\Include\\AppVariables.h"
#include "..\\..\\rlApplication\\Include\\AppMisc.h"
#include "..\\..\\rlApplication\\Include\\AppEeprom.h"
#include "..\\..\\rlApplication\\Include\\AppCalibration.h"
#include "..\\..\\rlApplication\\Include\\AppTampers.h"
#include "..\\..\\rlApplication\\Include\\AppConfig.h"
#include "..\\..\\rlApplication\\Include\\AppLcd.h"

#include "math.h"
//------------------------------------------------------


static void Start_DeltaSigmaADC(void);
void R_LCD_PowerOff(void);
static void doDeltaSigmaADC_NM(uint8_t chOption,uint16_t no_of_samples);
#if (defined(ISDO_HARMONICS_EEPL) && (ISDO_HARMONICS_EEPL == 1))
static void calHarmonics(uint8_t start_index);
#endif
static void logNMinfo(uint8_t chno);
static void checkApparentwithActive(void);
static void setPulseCounterPower(void);
//static uint32_t getCompPower(int32_t power,int8_t com_val);

#define ADCHANNEL0	0x00
#define ADCHANNEL1	0x01
#define ADCHANNEL2	0x02
#define ADCHANNEL3	0x03
#define _80_AD_WAIT_SETUP_TIME	0x80

#define CH_I1	1
#define CH_U	0
#define CH_I2	2

#define CH_P	0
#define CH_N	1


#define SAMPLE_BUFFER_MAX_COUNT			120
#define SAMPLE_MAX_COUNT			3906

#define NM_SHORT_SAMPLE				39
#define NM_LONG_SAMPLE				1903
#define NM_FALSE_CURRENT_DETECT_MAX_COUNT 	5	
#define NM_EXIT_FALSE_DETECT_DURATION		60

#define NM_INFO_SHUNT		0
#define NM_INFO_CT		1
#define NM_INFO_FALSE		2

void updateEnergyPulsePara(uint8_t mVal);
#if (defined(ENABLE_CT_COMPENSATION) && (ENABLE_CT_COMPENSATION == 1))
void addCTCompensation(void);
void adjCTPowerNew(uint8_t ctU,uint8_t compType);
#endif

static uint64_t adc_RAWChSum[3];
static int64_t adc_RAWPChSum[2];
static int16_t adc_CH[3];
static uint16_t adc_CHPeak[3];
static int16_t adc_CHbuff[3][SAMPLE_BUFFER_MAX_COUNT];
static uint16_t sampling_count;
static uint8_t sampling_count_1_cycle;
static uint8_t sampling_buff_count;
static uint8_t frequency_cycle_counter;
static uint16_t frequency_sample_count;
static uint16_t nm_false_current_exit_duration;


/***********************************************************************************************************************
* Function Name: R_ADC_Create
* Description  : This function initializes the AD converter.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

void R_ADC_Stop(void)
{
	DSADMR &= _F0F0_AD_DSADMR_POWER_OFF;  /* Stop conversion */
	DSAMK = 1U;  /* Disable INTAD interrupt */
	DSAIF = 0U;  /* Clear INTAD interrupt flag */
}

void init_DeltaSigmaADC(void)
{
	DSADCK = 0;  	/* OCO clock selected */
	DSADCEN = 1U;  	/* Supply clock to ADC */
	DSAMK = 1U;  	/* Disalbe INTSAD interrupt */
	DSAIF = 0U;  	/* Clear INTSAD interrupt flag */

	frequency_sample_count=0;
	sampling_buff_count=0;
	is_dc_magnet_exist_duration=0;
	sampling_count_1_cycle=0;
	/* Set INTSAD high priority */
	DSAPR0 = 0U;
	DSAPR1 = 0U;
	
	/* Sampling rate at 1953Hz, 16 bits resolution */
	DSADMR = _8000_DSAD_SAMPLING_FREQUENCY_1 | _4000_DSAD_RESOLUTION_16BIT; 
	DSADMR |= _070F_AD_DSADMR_POWER_ON;
	

	/* High pass filter */
    DSADHPFCR = _C0_DSAD_CUTOFF_FREQUENCY_3 | _00_DSAD_CH2_HIGHPASS_FILTER_ENABLE | 
                _00_DSAD_CH1_HIGHPASS_FILTER_ENABLE | _00_DSAD_CH0_HIGHPASS_FILTER_ENABLE;
	
	/* Gain setting */
	DSADGCR0 = 0x40; 
	DSADGCR1 = 0x00;
	
	//phase delay
	getRLCalib();
	DeltaSigmaADC_AdjustPhaseDelay(ADCHANNEL0,600);	// voltage channel
	DeltaSigmaADC_AdjustPhaseDelay(ADCHANNEL1,rlCalib.DelayI1);
	DeltaSigmaADC_AdjustPhaseDelay(ADCHANNEL2,rlCalib.DelayI2);
	
	Start_DeltaSigmaADC();
	
    /* Initial DC_Imunit Libary and Reset data */
    #if (defined(METER_ENABLE_DC_IMUNITY_DETECTION_ON_SAMPLE) && (METER_ENABLE_DC_IMUNITY_DETECTION_ON_SAMPLE == 1))
        R_DC_Imunity_Init((dc_imunity_config_t *)&g_dc_imunity_config_default_2k_50Hz);
        R_DC_Imunity_Reset_Phase_2();
    #endif	
	
	
}

/***********************************************************************************************************************
* Function Name: R_ADC_Start
* Description  : This function starts the AD converter.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void Start_DeltaSigmaADC(void)
{
	uint8_t wait;
	
	
	DSAMK = 1U;  /* Disable INTAD interrupt */
	DSAIF = 0U;  /* Clear INTSAD interrupt flag */	
	DSADMR &= _F0F0_AD_DSADMR_POWER_OFF;	/* Stop conversion */
	if(GET_MAINS_STATUS)
	{
		DSADMR |= _070F_AD_DSADMR_POWER_ON;		/* Start conversion ADC channel 0-2 */
		
		/* Wait for setup time */
		wait = _80_AD_WAIT_SETUP_TIME;
		while (1)
		{
			if(DSAIF == 1)
			{
				DSAIF = 0U;  /* Clear INTSAD interrupt flag */
				wait--;
			}
			if(wait == 0)
			{
				break;
			}
		}
		
		DSAMK = 0U;  /* Enable INTSAD interrupt */
	}
	else
		mcu_flag|=POWER_DOWN_FLAG;
}


/***********************************************************************************************************************
* Function Name: R_ADC_Stop
* Description  : This function stops the AD converter.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void DeltaSigmaADC_Stop(void)
{
	DSADMR &= _F0F0_AD_DSADMR_POWER_OFF;  /* Stop conversion */
	DSAMK = 1U;  /* Disable INTAD interrupt */
	DSAIF = 0U;  /* Clear INTAD interrupt flag */
}


/***********************************************************************************************************************
* Function Name: R_ADC_AdjustPhaseDelay
* Description  : This function is used to set the cut off frequency of HPF
* Arguments    : channel -
*                    input channel to be selected
* Return Value : MD_OK
*                    phase adjust successfully
*				 MD_ARGERROR
					 Input argument is invalid
***********************************************************************************************************************/
void DeltaSigmaADC_AdjustPhaseDelay(uint8_t adcChannel, uint16_t step)
{
	step=step&0x7FF;
	if (step <= 1151)
	{	
		/* Step must be from 0 to 1151 */
		switch(adcChannel)
		{
			case ADCHANNEL0:
				DSADPHCR0 = step;	 /* Set new step */
				break;
			case ADCHANNEL1:
				DSADPHCR1 = step;	 /* Set new step */
				break;
			case ADCHANNEL2:
				DSADPHCR2 = step;	 /* Set new step */
				break;
			case ADCHANNEL3:
				DSADPHCR3 = step;	 /* Set new step */
				break;			
		}		
	}
	
	
}


/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

int16_t positive=0,negative=0;
#pragma interrupt Interrupt_DSAD(vect=INTDSAD,bank=RB1)
static void Interrupt_DSAD(void)
{
	uint16_t sample;

		
	adc_CH[CH_I1] 	= DSADCR1;			
	adc_CH[CH_U] 	= DSADCR0;			
	adc_CH[CH_I2] 	= DSADCR2;
	
	positive=adc_CH[CH_U];
	//------------ I1 peak------
	sample=adc_CH[CH_I1];
	if(sample&0x8000)
		sample=0x10000UL-sample;

	if(adc_CHPeak[CH_I1]<sample)
		adc_CHPeak[CH_I1]=sample;
		
	//----------------------------------
	
	//------------ U peak------
	sample=adc_CH[CH_U];
	
	if(sample&0x8000)
		sample=0x10000-sample;
	
	if(adc_CHPeak[CH_U]<sample)
		adc_CHPeak[CH_U]=sample;
	
	//----------------------------------
	
	//------------ I2 peak------
	sample=adc_CH[CH_I2];
	if(sample&0x8000)
		sample=0x10000-sample;
	
	if(adc_CHPeak[CH_I2]<sample)
		adc_CHPeak[CH_I2]=sample;
	
	//----------------------------------	
	
	if(++sampling_count_1_cycle>=40)
	{
		if(GET_MAINS_STATUS)
			adc_CHPeak[CH_U]=(uint32_t)adc_CHPeak[CH_U]*rlCalib.U/100000UL;
		else
			adc_CHPeak[CH_U]=0;
			
		peakVoltage=adc_CHPeak[CH_U];
		if(adc_CHPeak[CH_U]<POWER_FAIL_DET_THRESHOLD_LEVEL1)
		{
			if(power_fail_state&SAVE_LEVEL1_KWH_STATE_OPEN)
				power_fail_state|=(SAVE_LEVEL1_KWH_STATE_CLOSE|SAVE_LEVEL2_KWH_STATE_CLOSE);
		}
			
		if(adc_CHPeak[CH_U]<POWER_FAIL_DET_THRESHOLD)
		{
			is_vfail_with_load=0;
			adc_CHPeak[CH_I1]=(uint32_t)adc_CHPeak[CH_I1]*rlCalib.I1/10000;
			if(adc_CHPeak[CH_I1]>1100)
				is_vfail_with_load=1;
			adc_CHPeak[CH_I2]=(uint32_t)adc_CHPeak[CH_I2]*rlCalib.I2/10000;
			if(adc_CHPeak[CH_I2]>1100)
				is_vfail_with_load=1;
					
			R_ADC_Stop();
			SWITCH_OFF_COMM_VCC;
			if(is_vfail_with_load==0)
			{
				SWITCH_OFF_PULSE_LED;
				activePowerNM=0;
				apperentPowerNM=0;
				activePulsePower=0;
				apparentPulsePower=0;
				
			}
			mcu_flag|=POWER_DOWN_FLAG;
			if(power_fail_state&SAVE_LEVEL2_KWH_STATE_OPEN)
				power_fail_state|=SAVE_LEVEL2_KWH_STATE_CLOSE;
			
			return;
		}
		sampling_count_1_cycle=0;
		adc_CHPeak[CH_I1]=0;
		adc_CHPeak[CH_U]=0;
		adc_CHPeak[CH_I2]=0;
	}
	
	if(GET_MAG_STATUS)
	{	
		// get witdh of low to high pulse
		if(mcu_flag&PROCESS_MAG_FLAG)
			if((mag_pulse_width>25)&&(mag_pulse_width<500))
			{
				mag_toggle_count++;
				mcu_flag|=SHOW_MAG_SYMBOL;
				is_dc_magnet_exist_duration=0;
			}
		mag_pulse_width=0;
		mcu_flag|=PROCESS_MAG_FLAG;
		#if (defined(IS_DC_MAG_ENABLE) && (IS_DC_MAG_ENABLE == 0))
			if(is_dc_magnet_exist_duration++>(1000U*MAG_DC_DETECT_TIME))
			{
				is_dc_magnet_exist_duration=(1000U*MAG_DC_DETECT_TIME);
				cum_mag_toggle_count=0;
				mag_hold_time=0;
				mcu_flag&=~SHOW_MAG_SYMBOL;
			}
		#endif
	}
	else
	{
		if(mcu_flag&PROCESS_MAG_FLAG)
			if(mag_pulse_width>8000)
			{
				mcu_flag&=~PROCESS_MAG_FLAG;
				is_dc_magnet_exist_duration=0;
			}
				
		mag_pulse_width++;
		
	}
		
	
	
	if((mag_toggle_count>0)&&(mag_toggle_count_duration++>10000))
	{
		if(mag_toggle_count<3)
		{
			mag_toggle_count=0;
		}
		cum_mag_toggle_count+=mag_toggle_count;
			
		mag_toggle_count_duration=0;
		
	}
	
	
	if(mcu_flag&START_MEASUREMENT)
	{
		if((positive>0) && (negative<0))
		{
			if((mcu_flag&ZERO_CROSS_MODE)==0)
				sampling_count=0;
			mcu_flag|=ZERO_CROSS_MODE;
		}
			
		negative=positive;
		if(mcu_flag&ZERO_CROSS_MODE)
		{
			adc_RAWChSum[CH_I1]+=(int32_t)adc_CH[CH_I1]*adc_CH[CH_I1];
			adc_RAWChSum[CH_U]+=(int32_t)adc_CH[CH_U]*adc_CH[CH_U];
			adc_RAWChSum[CH_I2]+=(int32_t)adc_CH[CH_I2]*adc_CH[CH_I2];
			adc_RAWPChSum[CH_P]+=(int32_t)adc_CH[CH_U]*adc_CH[CH_I1];
			adc_RAWPChSum[CH_N]+=(int32_t)adc_CH[CH_U]*adc_CH[CH_I2];
			
			if(++sampling_count>=SAMPLE_MAX_COUNT)
			{
				
				mcu_flag&=~(START_MEASUREMENT|ZERO_CROSS_MODE);
				mcu_flag|=DO_MEASUREMENT;
			}
		}
		else if(++sampling_count>=120)
		{
			sampling_count=0;
			mcu_flag|=ZERO_CROSS_MODE;
		}
		
	}
	
	if(mcu_flag&START_SAMPLE_BUFFER)
	{
		if(sampling_buff_count<SAMPLE_BUFFER_MAX_COUNT)
		{
			adc_CHbuff[CH_I1][sampling_buff_count]=adc_CH[CH_I1];
			adc_CHbuff[CH_U][sampling_buff_count]=adc_CH[CH_U];
			adc_CHbuff[CH_I2][sampling_buff_count]=adc_CH[CH_I2];
		}
		else
		{
			mcu_flag&=~START_SAMPLE_BUFFER;
			mcu_flag|=DO_FREQ_MEASUREMENT;
			sampling_buff_count=0;
		}
		if(mcu_flag&START_SAMPLE_BUFFER)
			sampling_buff_count++;
	}
	#if (defined(METER_ENABLE_DC_IMUNITY_DETECTION_ON_SAMPLE) && (METER_ENABLE_DC_IMUNITY_DETECTION_ON_SAMPLE == 1))
		
		g_dc_immunity_state = R_DC_Imunity_Calc_Phase_2(adc_CH[CH_I2]);
		
	    if (g_dc_immunity_state == 1)
	    {
	    	g_dc_detected_count++;
	    }
	    else
	    {
	        g_dc_detected_count--;
		    if (g_dc_detected_count <= -DC_DETECTION_COUNT_THRESHOLD)
		    {
		        g_dc_detected_count = -DC_DETECTION_COUNT_THRESHOLD;
		    }
			
	    }
	
		
	#endif	
	
}

void ClearADCData(void)
{
	adc_RAWChSum[0]=0;
	adc_RAWChSum[1]=0;
	adc_RAWChSum[2]=0;
	adc_RAWPChSum[0]=0;
	adc_RAWPChSum[1]=0;
	sampling_count=0;
	
}

void doMeasurement(uint8_t isProd)
{
	uint32_t cal_temp_val;
	
	
	
	cal_temp_val= sqrt(adc_RAWChSum[CH_U]/SAMPLE_MAX_COUNT);
	Ins.Voltage=cal_temp_val*rlCalib.U/10000;

	cal_temp_val= sqrt(adc_RAWChSum[CH_I1]/SAMPLE_MAX_COUNT);
	Ins.PhCurrent=cal_temp_val*rlCalib.I1/10000;
	
	cal_temp_val= sqrt(adc_RAWChSum[CH_I2]/SAMPLE_MAX_COUNT);
	Ins.NeuCurrent=cal_temp_val*rlCalib.I2/10000;
	
	
	
	cal_temp_val= adc_RAWPChSum[CH_P]/SAMPLE_MAX_COUNT;
//	if((Ins.PhCurrent>250)&&(Ins.PowerFactor>20))
//	{
	mcu_flag&=~PH_POWER_REV;
//	}
	if(cal_temp_val&0x80000000)
	{
		cal_temp_val=0x100000000-cal_temp_val;
		mcu_flag|=PH_POWER_REV;
	}
	
	
	Ins.PhPower=(uint64_t)cal_temp_val*100/rlCalib.Pp;
	
	cal_temp_val= adc_RAWPChSum[CH_N]/SAMPLE_MAX_COUNT;

//	if((Ins.PhCurrent>250)&&(Ins.PowerFactor>20))
//	{
		mcu_flag&=~NU_POWER_REV;
//	}
	
	if(cal_temp_val&0x80000000)
	{
		cal_temp_val=0x100000000-cal_temp_val;
		mcu_flag|=NU_POWER_REV;
	}	
	
	
	Ins.NeuPower=(uint64_t)cal_temp_val*100/rlCalib.Pn;

	if(Ins.PhPower<STARTING_POWER_THRESHOLD_L)	
		Ins.PhPower=0;
	
	if(Ins.NeuPower<STARTING_POWER_THRESHOLD_L)	
		Ins.NeuPower=0;
	
	if(Ins.Voltage>1000)
	{
		if(Ins.PhPower>=Ins.NeuPower)
		{
			Ins.EffectiveP=Ins.PhPower;
			Ins.EffectiveI=Ins.PhCurrent;
			
		}
		else
		{
			Ins.EffectiveP=Ins.NeuPower;
			Ins.EffectiveI=Ins.NeuCurrent;
			
		}
	}
	else
	{
		if(Ins.PhCurrent>=Ins.NeuCurrent)
		{
			Ins.EffectiveP=Ins.PhPower;
			Ins.EffectiveI=Ins.PhCurrent;
			
		}
		else
		{
			Ins.EffectiveP=Ins.NeuPower;
			Ins.EffectiveI=Ins.NeuCurrent;
			
		}		
	}
	
	if(Ins.EffectiveP<STARTING_POWER_THRESHOLD_L)
		Ins.EffectiveP=0;
	
	Ins.AppPower=(uint32_t)Ins.Voltage*Ins.EffectiveI/1000;
	checkApparentwithActive();	
	
	if(Ins.AppPower>0)
		Ins.PowerFactor=Ins.EffectiveP*100/Ins.AppPower;
	else
		Ins.PowerFactor=0;
	
	
	if(Ins.EffectiveP<STARTING_POWER_THRESHOLD_L)
		Ins.PowerFactor=0;
		
	if(Ins.Voltage<1000)
	{
		Ins.Frequency=0x00;
		Ins.PowerFactor=0;
	}
	if((Ins.EffectiveI<8)&&(Ins.Voltage>26600))
	{
		//if(Ins.Voltage>26500)
	//	{
			Ins.PowerFactor=100;
	//	}
	}

	//-----------------------------------------------------
	
	//-------------start measurement again---------------

	setPulseCounterPower();
	
	cal_temp_val=Check_Tamper();


	if((Ins.EffectiveP<STARTING_POWER_THRESHOLD_U)&&(Ins.EffectiveP>STARTING_POWER_THRESHOLD_L))
	{
		Ins.EffectiveP=STARTING_POWER_THRESHOLD_CONST;
		Ins.AppPower=STARTING_POWER_THRESHOLD_CONST;
		if(Ins.PhPower>STARTING_POWER_THRESHOLD_L)
			Ins.PhPower=STARTING_POWER_THRESHOLD_CONST;
		if(Ins.NeuPower>STARTING_POWER_THRESHOLD_L)
			Ins.NeuPower=STARTING_POWER_THRESHOLD_CONST;
	}
//	else if(Ins.AppPower<STARTING_POWER_THRESHOLD_L)
//	{
//		Ins.AppPower=0;
//		Ins.PowerFactor=100;
//	}

	
	if(cal_temp_val==0)
	{
		#if (defined(ENABLE_CT_COMPENSATION) && (ENABLE_CT_COMPENSATION == 1))
			addCTCompensation();
		#endif		
		#if (defined(APPLY_DC_IMMUNITY_ENABLE) && (APPLY_DC_IMMUNITY_ENABLE == 1))
			if(CurrentTamperStatusFlag & TAMPER_EARTH)
			{
				if((Ins.EffectiveP!=Ins.NeuPower))
					g_dc_immunity_state=0;
			}
			else
				g_dc_immunity_state=0;
			if(g_dc_immunity_state==1)
				if(CurrentTamperStatusFlag & TAMPER_EARTH)
					doDCImmunityAction();
				
		#endif
		
		#if (defined(ISDO_HARMONICS_EEPL) && (ISDO_HARMONICS_EEPL == 1))
		if(mcu_flag&HARMONICS_PRESENT)
		{
			if((Ins.PowerFactor>80)&&(Ins.PowerFactor<92))
				Ins.EffectiveP=Ins.EffectiveP+Ins.EffectiveP*8/100;
		}
		#endif

		if(lag_only_value==1)
		{
			if(((Ins.EffectiveP+100)<Ins.AppPower)&&(Ins.LeadLag==1))
				isforcedUinity=1;
			else
				isforcedUinity=0;
			
			if(isforcedUinity==1)
			{
				Ins.AppPower=Ins.EffectiveP;
			}
		}
		else
		{
			Ins.AppPower=(uint32_t)Ins.Voltage*Ins.EffectiveI/1000;
		}
	 
		activePulsePower=Ins.EffectiveP;
		apparentPulsePower=Ins.AppPower;

		if(Ins.AppPower<STARTING_POWER_THRESHOLD_L)
		{
		Ins.AppPower=0;
		Ins.PowerFactor=100;
		}
		
 
		if((mcu_flag&POWER_STABLE_FLAG)==0)
			setPulseCounterPower();
	}
	else if((mcu_flag&POWER_STABLE_FLAG)==0)
	{
		activePulsePower=Ins.EffectiveP;
		apparentPulsePower=Ins.AppPower;
		setPulseCounterPower();
	}
	
	
	
	if(USER_CODE==isProd)
	{
		if(Ins.PhPower<STARTING_POWER_THRESHOLD_L)
		{
			Ins.PhPower=0;
			if(Ins.PhCurrent<20)
				Ins.PhCurrent=0;
		    if(Ins.EffectiveP<STARTING_POWER_THRESHOLD_L)
		    	Ins.EffectiveI=0;
			
		}
		if(Ins.NeuPower<STARTING_POWER_THRESHOLD_L)
			if(Ins.NeuCurrent<20)
				Ins.NeuCurrent=0;
		
		
	}
	
	checkApparentwithActive();
	
	ClearADCData();

	mcu_flag|=START_MEASUREMENT;
	
	//----------------------------------------------------
	
}

void Calculate_Line_Frq(void)
{
	uint8_t index,start_cal=0,effective_ch;
	uint8_t start_count=0,end_count=0;
	
	if(Ins.PhPower>=Ins.NeuPower)
		effective_ch=CH_I1;
	else
		effective_ch=CH_I2;
	
	Ins.LeadLag=0xFF;	// unknown
	pf_by_sample=0xFF;
	
	for(index=0;index<(SAMPLE_BUFFER_MAX_COUNT-1);index++)
	{
		
		if((adc_CHbuff[CH_U][index]<=0)&&(adc_CHbuff[CH_U][index+1]>0))
		{
			if(start_cal==0)
			{
				start_cal=1;
				start_count=index;
				end_count=index;
			}
			else if(start_cal==2)
			{
				end_count=index;
				break;
			}
		}
		else
		{
			if(start_cal==1)
			{
				if((adc_CHbuff[CH_U][index]>=0)&&(adc_CHbuff[CH_U][index+1]<0))
					start_cal=2;
			}
		}
		if((adc_CHbuff[effective_ch][index]>=0)&&(adc_CHbuff[effective_ch][index+1]<0))
		{
			if((Ins.LeadLag==0xFF)&&(start_cal>0))
			{
				if(pf_by_sample==0xFF)
					pf_by_sample=(index-start_count);
					
				switch(pf_by_sample)
				{
					case 19:
					case 18:
					case 17:
						pf_by_sample=0;
						break;
				}
				
				if(start_cal==2)
					Ins.LeadLag=0;
				else
					Ins.LeadLag=1;
			}
		}
	}
	if(display_flag&DISP_REV)
	{
		if(Ins.LeadLag==0)
			Ins.LeadLag=1;
		else if(Ins.LeadLag==1)
			Ins.LeadLag=0;
	}
	
	if(isforcedUinity==0)
		if(Ins.EffectiveP>=(Ins.AppPower-100))
			Ins.LeadLag=0;

		
	
	
	
	frequency_sample_count+=(end_count-start_count);
	
	if((++frequency_cycle_counter>=50)&&(frequency_sample_count>0))
	{
		Ins.Frequency=5000UL*1953/frequency_sample_count;	//19530000=5000*3906
		frequency_cycle_counter=0;
		frequency_sample_count=0;
	}
	
	#if (defined(ISDO_HARMONICS_EEPL) && (ISDO_HARMONICS_EEPL == 1))
	calHarmonics(start_count);
	#endif
	mcu_flag|=START_SAMPLE_BUFFER;
	
}
#if (defined(ISDO_HARMONICS_EEPL) && (ISDO_HARMONICS_EEPL == 1))
uint32_t getHarmonicsVal(const int16_t *samplePtr,uint8_t hzType);
void calHarmonics(uint8_t start_index)
{
	uint32_t U50,I50,U250,I250;
	uint64_t P50,P250;

	U50=getHarmonicsVal((const int16_t*)&adc_CHbuff[CH_U][start_index],HAR_HZ_TYPE_50);
	I50=getHarmonicsVal((const int16_t*)&adc_CHbuff[CH_I1][start_index],HAR_HZ_TYPE_50);

	U250=getHarmonicsVal((const int16_t*)&adc_CHbuff[CH_U][start_index],HAR_HZ_TYPE_250);
	I250=getHarmonicsVal((const int16_t*)&adc_CHbuff[CH_I1][start_index],HAR_HZ_TYPE_250);
	
	P50=(uint64_t)U50*I50;
	P250=(uint64_t)U250*I250;
	if(P50)
		harChU=P250*100/P50;
	else
		harChU=0;
	
	if((CurrentTamperStatusFlag&TAMPER_EARTH)&&(harChU>=HARMONICS_LIMIT))
	{
		
		if(++har_wait_time>HARMONICS_WAIT_TIME)
		{
			mcu_flag|=HARMONICS_PRESENT;
			har_wait_time=HARMONICS_WAIT_TIME;
		}
		
	}
	else
	{
		if(har_wait_time>0)
			har_wait_time--;
		else
			mcu_flag&=~HARMONICS_PRESENT;
	}

	if(U50)
		harChU=U250*100/U50;
	else
		harChU=0;
	
	if(I50)
		harChI=I250*100/I50;
	else
		harChI=0;
	
}
#endif
void doMeasurement_NM(uint8_t isPulseOn)
{
	uint32_t  preCurrent=Ins.EffectiveI  ;
	
	if(nm_false_current_detect_count<NM_FALSE_CURRENT_DETECT_MAX_COUNT)
	{
		if(Ins.EffectiveI>0)
		{
			if(Ins.EffectiveI==Ins.PhCurrent)
			{
				if(isPulseOn==0)
					doDeltaSigmaADC_NM(CH_I1,NM_SHORT_SAMPLE);
				else
					doDeltaSigmaADC_NM(CH_I1,NM_LONG_SAMPLE);
					
				if((preCurrent>(Ins.EffectiveI+500))||((preCurrent+500)<Ins.EffectiveI))
				{
						doDeltaSigmaADC_NM(CH_I1,NM_LONG_SAMPLE);
						doDeltaSigmaADC_NM(CH_I1,NM_LONG_SAMPLE);
						nm_false_current_exit_duration=0;
				}
				
				mcu_flag|=GET_SHUNT_CURRENT_FLAG;
			}
			else if(Ins.EffectiveI==Ins.NeuCurrent)
			{
				if(isPulseOn==0)
					doDeltaSigmaADC_NM(CH_I2,NM_SHORT_SAMPLE);
				else
					doDeltaSigmaADC_NM(CH_I2,NM_LONG_SAMPLE);
					
				if((preCurrent>(Ins.EffectiveI+500))||((preCurrent+500)<Ins.EffectiveI))
				{
						doDeltaSigmaADC_NM(CH_I2,NM_LONG_SAMPLE);
						doDeltaSigmaADC_NM(CH_I2,NM_LONG_SAMPLE);
						nm_false_current_exit_duration=0;
				}
				
				mcu_flag&=~GET_SHUNT_CURRENT_FLAG;
			}
			else
				Ins.EffectiveI=0;
				
			if(nm_false_current_exit_duration++>NM_EXIT_FALSE_DETECT_DURATION)
			{
				if(Ins.EffectiveI==Ins.PhCurrent)
				{
					doDeltaSigmaADC_NM(CH_I1,NM_LONG_SAMPLE);
					doDeltaSigmaADC_NM(CH_I1,NM_LONG_SAMPLE);
				}
				
				if(Ins.EffectiveI==Ins.NeuCurrent)
				{
					doDeltaSigmaADC_NM(CH_I2,NM_LONG_SAMPLE);
					doDeltaSigmaADC_NM(CH_I2,NM_LONG_SAMPLE);
				}
					
				nm_false_current_exit_duration=0;
				if(Ins.EffectiveI>0)
					nm_false_current_detect_count=0;
			}
			
				
			DSADMR &=_F0F0_AD_DSADMR_POWER_OFF;  /* Stop conversion */
			
			if(((mcu_flag&START_MEASUREMENT)==0)||(Ins.Voltage>0))
			{
				doDeltaSigmaADC_NM(CH_U,NM_SHORT_SAMPLE);
				
				//mcu_flag|=START_MEASUREMENT;
			}
			
		}
		else 
		{
			mcu_flag&=~START_MEASUREMENT;
			if(mcu_flag&GET_SHUNT_CURRENT_FLAG)
			{
				doDeltaSigmaADC_NM(CH_I1,NM_SHORT_SAMPLE);
				
				mcu_flag&=~GET_SHUNT_CURRENT_FLAG;
			}
			else
			{
				doDeltaSigmaADC_NM(CH_I2,NM_SHORT_SAMPLE);
				mcu_flag|=GET_SHUNT_CURRENT_FLAG;
			}
			if(Ins.EffectiveI>0)
			{
				
				if(Ins.EffectiveI==Ins.PhCurrent)
				{
					doDeltaSigmaADC_NM(CH_I1,NM_LONG_SAMPLE);
					doDeltaSigmaADC_NM(CH_I1,NM_LONG_SAMPLE);
				}
				else if(Ins.EffectiveI==Ins.NeuCurrent)
				{
					doDeltaSigmaADC_NM(CH_I2,NM_LONG_SAMPLE);
					doDeltaSigmaADC_NM(CH_I2,NM_LONG_SAMPLE);
				}
					
				if(Ins.EffectiveI==0)
				{
					nm_false_current_detect_count++;
					logNMinfo(NM_INFO_FALSE);
				}
				else
				{
					DSADMR &=_F0F0_AD_DSADMR_POWER_OFF;  /* Stop conversion */
					nm_false_current_exit_duration=0;
					if(Ins.EffectiveI==Ins.PhCurrent)
					{
						doDeltaSigmaADC_NM(CH_I2,NM_SHORT_SAMPLE);
						if(Ins.NeuCurrent>0)
							doDeltaSigmaADC_NM(CH_I2,NM_LONG_SAMPLE);
					}
					else
					{
						doDeltaSigmaADC_NM(CH_I1,NM_SHORT_SAMPLE);
						if(Ins.PhCurrent>0)
							doDeltaSigmaADC_NM(CH_I1,NM_LONG_SAMPLE);
					}
						
					if(getEarthStatus(BAT_EARTH_TAMPER_THRESHOLD)==0)
					{
						Ins.EffectiveI=0;
						Ins.PhCurrent=0;
						Ins.NeuCurrent=0;
					}
					
				}
			}
		}
		DSADMR &=_F0F0_AD_DSADMR_POWER_OFF;  /* Stop conversion */
	}
	
}

void doDeltaSigmaADC_NM(uint8_t chOption,uint16_t no_of_samples)
{
	
	uint16_t wait,sampling_count;
	
	int32_t psum=0,nsum=0;
	uint32_t falseCurrentptv=0;
	uint32_t falseCurrentntv=0;
	
	
	/* Sampling rate at 1953Hz, 16 bits resolution */
	
	WDT_Restart(0xAC);
	if(chOption==CH_I1)
		DSADMR |= 0x0202;
	else if(chOption==CH_I2)
		DSADMR |= 0x0404;
	else
		DSADMR |= 0x0101;

	sampling_count=0;
	adc_RAWChSum[0]=0;
	adc_RAWChSum[1]=0;
	adc_CH[1]=0;
	adc_CH[2]=0;
	/* Wait for setup time */
	if(Ins.EffectiveI>0)
		wait=190;
	else
		wait = 18;
		
	while (1)
	{
		if(DSAIF == 1)
		{
			
			DSAIF = 0U;  /* Clear INTSAD interrupt flag */
			wait--;
		}
		if(wait == 0)
		{
			break;
		}
	}
	wait=no_of_samples;
	
	while (sampling_count<wait)
	{
		
		if(DSAIF == 1)
		{
			DSAIF = 0U;  /* Clear INTSAD interrupt flag */
			if(chOption==CH_I1)
			{
				adc_CH[0]	= DSADCR1;
				
			}
			else if(chOption==CH_I2)
				adc_CH[0] 	= DSADCR2;
			else
				adc_CH[0] 	= DSADCR0;
			
			if(NM_LONG_SAMPLE==wait)
				adc_RAWChSum[0]+=(int32_t)adc_CH[0]*adc_CH[0];
				
			if(sampling_count<NM_SHORT_SAMPLE)	
				adc_CHbuff[0][sampling_count]=adc_CH[0];
				
			sampling_count++;
			
			//----------------------------------
		}
	}
	if(NM_LONG_SAMPLE!=wait)
	{
		wait=NM_SHORT_SAMPLE;
		sampling_count=0;
		while (sampling_count<wait)
		{
			if(adc_CHbuff[0][sampling_count]>=0)		
				psum+=adc_CHbuff[0][sampling_count];
			else
				nsum+=adc_CHbuff[0][sampling_count];
			sampling_count++;
		}
	
		psum=psum/wait;
		nsum=nsum/wait;
	
		sampling_count=0;
		while (sampling_count<wait)
		{
			if(adc_CHbuff[0][sampling_count]>=0)		
				adc_CH[0]=adc_CHbuff[0][sampling_count]-psum;
			else
				adc_CH[0]=adc_CHbuff[0][sampling_count]-nsum;
	
				// actual
				adc_RAWChSum[0]+=(int32_t)adc_CHbuff[0][sampling_count]*adc_CHbuff[0][sampling_count];
				// false 
				adc_RAWChSum[1]+=(int32_t)adc_CH[0]*adc_CH[0];
				
			//------------ Current peak------
			//negative
			if(adc_CHbuff[0][sampling_count]&0x8000)
			{
				if(adc_CH[2]>adc_CHbuff[0][sampling_count])
					adc_CH[2]=adc_CHbuff[0][sampling_count];
			}
			//positive
			if(adc_CH[1]<adc_CHbuff[0][sampling_count])
				adc_CH[1]=adc_CHbuff[0][sampling_count];
			//---------------------------------------
			sampling_count++;
			
		}	
		adc_CH[2]=fabs(adc_CH[2]);
	}
	

	if(chOption==CH_I1)
	{
		Ins.PhCurrent=(uint32_t)(sqrt(adc_RAWChSum[0]/wait))*rlCalib.I1/10000;
		if(Ins.EffectiveI==0)
		{
			if(NM_SHORT_SAMPLE==wait)
			{
				
				falseCurrentptv=(uint32_t)adc_CH[1]*rlCalib.I1/10000;
				falseCurrentntv=(uint32_t)adc_CH[2]*rlCalib.I1/10000;
				
				if(Ins.PhCurrent<IMAX_CURRENT_NM)
				{
					if((falseCurrentptv<IMAX_FALSE_CURRENT_NM)&&(falseCurrentntv<IMAX_FALSE_CURRENT_NM)&&(falseCurrentptv>(Ins.PhCurrent/3))&&(falseCurrentntv>(Ins.PhCurrent/3)))
					{
						if(Ins.PhCurrent>NM_CURRENT_THRESHOLD)
						{
							falseCurrentptv=(uint32_t)(sqrt(adc_RAWChSum[1]/wait))*rlCalib.I1/10000;
							if(falseCurrentptv<Ins.PhCurrent/3)
							{
								Ins.PhCurrent=0;
								logNMinfo(NM_INFO_SHUNT);
							}
								
						}
					}
					else
						Ins.PhCurrent=0;
				}
				else
					Ins.PhCurrent=0;
			}
		}
				
	}
	else if(chOption==CH_I2)
	{
		Ins.NeuCurrent=(uint32_t)(sqrt(adc_RAWChSum[0]/wait))*rlCalib.I2/10000;
		if(Ins.EffectiveI==0)
		{
			if(NM_SHORT_SAMPLE==wait)
			{
				falseCurrentptv=(uint32_t)adc_CH[1]*rlCalib.I2/10000;
				falseCurrentntv=(uint32_t)adc_CH[2]*rlCalib.I2/10000;
				
				if(Ins.NeuCurrent<IMAX_CURRENT_NM)
				{
					if((falseCurrentptv<IMAX_FALSE_CURRENT_NM)&&(falseCurrentntv<IMAX_FALSE_CURRENT_NM)&&(falseCurrentptv>(Ins.NeuCurrent/3))&&(falseCurrentntv>(Ins.NeuCurrent/3)))
					{
						if(Ins.NeuCurrent>NM_CURRENT_THRESHOLD)
						{
							falseCurrentptv=(uint32_t)(sqrt(adc_RAWChSum[1]/wait))*rlCalib.I2/10000;
							if(falseCurrentptv<Ins.NeuCurrent/3)
							{
								Ins.NeuCurrent=0;
								logNMinfo(NM_INFO_CT);
							}
						}
					}
					else
						Ins.NeuCurrent=0;
				}
				else
					Ins.NeuCurrent=0;
			}
		}
		
	}
	else
		Ins.Voltage =(uint32_t)(sqrt(adc_RAWChSum[0]/wait))*rlCalib.U/10000;

	if(Ins.PhCurrent<NM_CURRENT_THRESHOLD)
		Ins.PhCurrent=0;
	if(Ins.NeuCurrent<NM_CURRENT_THRESHOLD)
		Ins.NeuCurrent=0;
		
	if(Ins.PhCurrent>Ins.NeuCurrent)
	{
		Ins.EffectiveI=Ins.PhCurrent;
	}
	else
	{
		Ins.EffectiveI=Ins.NeuCurrent;
	}
	
	if(Ins.Voltage<3500)
		Ins.Voltage=0;

}


void logNMinfo(uint8_t chno)
{
	chno=chno%3;
	#if (defined(LOG_NM_INFO) && (LOG_NM_INFO == 1))
		nmInfo[chno]++;
	#endif
}

//uint8_t checkExitVolt(uint8_t checkCurrent)
//{
//	uint8_t status=0;
//	doDeltaSigmaADC_NM(CH_U,NM_SHORT_SAMPLE);
//	DSADMR &=_F0F0_AD_DSADMR_POWER_OFF;  /* Stop conversion */
//	if(Ins.Voltage>EXIT_VOLTAGE_THRESHOLD)	
//		status=1;
//	else
//	{
//		if(is_dc_magnet_exist_duration==0)
//		{
//			is_dc_magnet_exist_duration=1;
//			doDeltaSigmaADC_NM(CH_U,NM_LONG_SAMPLE);
//			doDeltaSigmaADC_NM(CH_U,NM_LONG_SAMPLE);
//			DSADMR &=_F0F0_AD_DSADMR_POWER_OFF;  /* Stop conversion */
//			if(Ins.Voltage>EXIT_VOLTAGE_THRESHOLD)	
//				status=1;
//		}
		
//		if(checkCurrent<2)
//		{
//			doDeltaSigmaADC_NM(CH_I1,NM_SHORT_SAMPLE);
//			if(Ins.NeuCurrent>0)
//				doDeltaSigmaADC_NM(CH_I1,NM_LONG_SAMPLE);
				
//			DSADMR &=_F0F0_AD_DSADMR_POWER_OFF;  /* Stop conversion */
			
//			doDeltaSigmaADC_NM(CH_I2,NM_SHORT_SAMPLE);
//			if(Ins.PhCurrent>0)
//				doDeltaSigmaADC_NM(CH_I2,NM_LONG_SAMPLE);
//			DSADMR &=_F0F0_AD_DSADMR_POWER_OFF;  /* Stop conversion */
			
//			if(Ins.EffectiveI>NM_CURRENT_THRESHOLD)
//				if(isEarth(BAT_EARTH_TAMPER_THRESHOLD)==1)
//					status=1;
//		}
//	}
		
//	return status;
//}


//uint32_t getCompPower(int32_t power,int8_t com_val)
//{
//	//if(Ins.LeadLag==1)
//		//com_val=com_val*-1;
		
//	power=power+(power*com_val)/1000;
//	return power;
//}

//void addComp(void)
//{
//	if(CurrentTamperStatusFlag&TAMPER_EARTH)
//	{
//		if((Ins.PhPower>STARTING_POWER_THRESHOLD_U)&&(Ins.PhPower==Ins.EffectiveP))
//		{
//			if(((Ins.PhCurrent<1200)&&(Ins.PowerFactor<60))||(Ins.PhCurrent<550))
//			{
				
//				if(((display_flag&DISP_REV)==0))
//				{
//					if(Ins.PhCurrent<550)
//						Ins.PhPower=Ins.PhPower-Ins.PhPower*11/1000;
//					else
//						Ins.PhPower=Ins.PhPower-Ins.PhPower*5/1000;
//				}
//				else
//				{
//					if((Ins.PhCurrent<550)&&(Ins.PowerFactor<60))
//						Ins.PhPower=Ins.PhPower+Ins.PhPower*20/1000;
//					else
//						Ins.PhPower=Ins.PhPower+Ins.PhPower*5/1000;
//				}
				
//				Ins.EffectiveP=Ins.PhPower;
//			}
//		}
//	}
//}

#if (defined(ENABLE_CT_COMPENSATION) && (ENABLE_CT_COMPENSATION == 1))
void addCTCompensation(void)
{
  if((CurrentTamperStatusFlag & TAMPER_EARTH)||(CurrentTamperStatusFlag_AP & TAMPER_EARTH))
  {
    if(Ins.PowerFactor<55)
    {
      if(Ins.NeuPower==Ins.EffectiveP)
      {
        if(Ins.NeuCurrent<CT_COMPENSATION_LS_LP)
        {
          adjCTPowerNew(CT_COMPENSATION_LS_CV,1);
        }
        else if(Ins.NeuCurrent>CT_COMPENSATION_US_LP)
        {
          adjCTPowerNew(CT_COMPENSATION_US_CV,0);
        }
        
        Ins.EffectiveP=Ins.NeuPower;
      }
    }
  }
}
void adjCTPowerNew(uint8_t ctU,uint8_t compType)
{
    uint32_t newpower=0;
  if(((((CurrentTamperStatusFlag&TAMPER_REV)==0)&&(Ins.LeadLag==1))||((CurrentTamperStatusFlag&TAMPER_REV)&&(Ins.LeadLag==0)))||
  ((((CurrentTamperStatusFlag_AP&TAMPER_REV)==0)&&(Ins.LeadLag==1))||((CurrentTamperStatusFlag_AP&TAMPER_REV)&&(Ins.LeadLag==0))))
    compType=!compType;
  
  newpower=Ins.NeuPower*ctU/1000;  
  if(compType)
    Ins.NeuPower=Ins.NeuPower-newpower;
  else
    Ins.NeuPower=Ins.NeuPower+newpower;
    
}
#endif


void checkApparentwithActive(void)
{
	if(Ins.EffectiveP>Ins.AppPower)
		Ins.AppPower=Ins.EffectiveP;
	
}

void setPulseCounterPower(void)
{
	activePowerNM=activePulsePower;
	apperentPowerNM=apparentPulsePower;
	
}
