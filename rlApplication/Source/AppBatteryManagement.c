//-------- include device specific files----------------
#include "..\\..\\rlDevice\\Include\\dUser_includes.h"
#include "..\\..\\rlDevice\\Include\\dTypedef.h"
#include "..\\..\\rlDevice\\Include\\d8bitTimer.h"
#include "..\\..\\rlDevice\\Include\\d12bitTimer.h"
#include "..\\..\\rlDevice\\Include\\dIOCTrl.h"
#include "..\\..\\rlDevice\\Include\\dWatchDog.h"
#include "..\\..\\rlDevice\\Include\\dRtc.h"
#include "..\\..\\rlDevice\\Include\\dVbat.h"
#include "..\\..\\rlDevice\\Include\\dDisplay.h"
#include "..\\..\\rlDevice\\Include\\dUart.h"
#include "..\\..\\rlDevice\\Include\\dI2c.h"
#include "..\\..\\rlDevice\\Include\\dDeltaSigmaADC.h"


//-------- include app specific files-------------------
#include "..\\Include\\AppVariables.h"
#include "..\\Include\\AppLcd.h"
#include "..\\Include\\AppCommunication.h"
#include "..\\Include\\AppNMTask.h"
#include "..\\Include\\AppMisc.h"
#include "..\\Include\\AppTampers.h"
#include "..\\Include\\AppIntervalkWh.h"
#include "..\\Include\\AppConfig.h"
#include "..\\Include\\AppEeprom.h"
#include "..\\Include\\AppBatteryManagement.h"
#include "..\\Include\\AppCustomerProtocol.h"

//------------------------------------------------------

//-------- include dlms specific files----------------
#include "..\\..\\rlDlms\\DLMS_User.h"


#define LCD_SWITCH_ON_WAIT_TIME		1	//seconds
#define COM_SWITCH_ON_WAIT_TIME		5	//seconds
#define PULSE_SWITCH_ON_WAIT_TIME	6	//seconds
#define NM_PULSE_INTERVAL		15	//seconds
#define PFAIL_WAIT_COUNT_5		3		
#define PFAIL_WAIT_COUNT_1		10

#define PFAIL_MODE_STOP								0x0000
#define PFAIL_MODE_LCD								0x0001
#define PFAIL_MODE_COM								0x0002
#define PFAIL_MODE_ADC								0x0008
#define PFAIL_MODE_SWITCH_PRESS_AWAIT_LCD					0x0010
#define PFAIL_MODE_SWITCH_PRESS_RELEASE_AWAIT_LCD				0x0020
#define PFAIL_MODE_SWITCH_PRESS_AWAIT_COM					0x0040
#define PFAIL_MODE_SWITCH_PRESS_RELEASE_AWAIT_COM				0x0080
#define EXIT_PFAIL_OPERATION							0x0100
#define DO_ACTION_AT_POWER_RESTORE						0x0200
#define COVER_OPEN_ACTION							0x0400
#define SWITCH_ON_LCD								0x0800
#define SWITCH_ON_COM								0x1000
#define SWITCH_OFF_LCD								0x2000
#define SWITCH_ON_PULSE								0x4000
#define PFAIL_MODE_PULSE							0x8000




#define SWITCH_ON_8BIT_TIMER	0x10
#define SWITCH_OFF_8BIT_TIMER	0x20

#if (defined(METER_RATING_5_30) && (METER_RATING_5_30 == 1))
	#define PER_PULSE_VALUE_FAST	937UL
	#define PER_PULSE_VALUE_SLOW	78UL
#else
	#define PER_PULSE_VALUE_FAST	1875UL
	#define PER_PULSE_VALUE_SLOW	156UL
#endif

#define DSP_FAST_MODE_INTERVAL		5
#define DSP_SLOW_MODE_INTERVAL		60
#define DSP_NO_LOAD_MODE_INTERVAL	240

#define DSP_LOAD_SECOND_STAGE_WAIT_TIME		5400
#if (defined(TENDER_MODE_NO_LOAD_TIME) && (TENDER_MODE_NO_LOAD_TIME == 1))
	#define DSP_NO_LOAD_FIRST_STAGE_WAIT_TIME	3600
#else
	#define DSP_NO_LOAD_FIRST_STAGE_WAIT_TIME	600
#endif

#define DSP_NO_LOAD_SECOND_STAGE_WAIT_TIME	86400UL
#define PULSE_MODE_DURATION	30

#define BATTERY_LCD_LOCK_COUNT	3



extern Unsigned32	g_Class07_Event_EntriesInUse[];
void updatePfailEvent(void);
void updatePulse(void);
void updateEnergyPulsePara(uint8_t ioption);
extern uint8_t push_button_pressed_for_nm;
//extern uint8_t mag_permanent_save;


//#if (defined(IS_PRODUCTION_MODE_ENABLED) && (IS_PRODUCTION_MODE_ENABLED == 0))
void powerfail_op_without_NM(void)
{

	uint16_t op_state=PFAIL_MODE_STOP,op_action_state=PFAIL_MODE_SWITCH_PRESS_AWAIT_LCD;
	uint8_t com_enable_wait_counter,stop_adc_timer;
	uint8_t display_no_of_para=0,display_no_of_para_count=0;
	uint8_t pfail_wait_count=0;
	uint32_t adc_switch_wait_time=0;
	uint8_t battery_mode_lcd_on_count=0,adc_on_counter=0;
	uint8_t battery_mode_nm_on_count=0;
	uint32_t battery_mode_display_time=0;
	uint32_t nowNMkWh=0;
	
	
	stop_adc_timer=RESF;
	stop_adc_timer=stop_adc_timer&0x01;
	
	if(is_vfail_with_load==0)
		start_Timer8bit0(DSP_FAST_MODE_INTERVAL);
	

	initNMLs();
	mcu_flag=GET_SHUNT_CURRENT_FLAG;
	CurrentTamperStatusFlag=0;
	CurrentTamperStatusFlag_AP=0;
	
#if (defined(IS_DISPLAY_KEPT_ON) && (IS_DISPLAY_KEPT_ON == 1))
	setParaBuff(DISPLAY_MODE_UOUT_NORMAL);
	display_no_of_para=getParaCount();
	init_Timer8bit1(getParamTime());
	mcu_flag|=EIGHT_BIT_TIMER1_FLAG;
	op_action_state|=PFAIL_MODE_LCD;
#endif	
	RxTxBuffer[0]=0x00;
	write_eeprom(RxTxBuffer,RTC_BAT_STATUS_LOC,1);
	

	isRAMOK();
	
	do
	{
		//Switch2FilClock();
		WDT_Restart(0xAC);
		switch(op_state)
		{
			//-----------------------------------------------------
			case DO_ACTION_AT_POWER_RESTORE:
				// do action at power restore
				op_state=EXIT_PFAIL_OPERATION;
			break;
			//-----------------------------------------------------
			
			//-----------------------------------------------------
			case PFAIL_MODE_STOP:
					
				#if(defined(NM_HARD_TEST) && (NM_HARD_TEST == 1))
					if(GET_COVER_STATUS)
				#endif
				reEnable32KCrystal();
				if(GET_MAINS_STATUS)
				{
					++pfail_wait_count;
					if((TRTCMP0/256==1))
					{
						if(pfail_wait_count>PFAIL_WAIT_COUNT_1)
							op_state=DO_ACTION_AT_POWER_RESTORE;
					}
					else if(pfail_wait_count>PFAIL_WAIT_COUNT_5)
						op_state=DO_ACTION_AT_POWER_RESTORE;
				}
				else
				{
					pfail_wait_count=PFAIL_WAIT_COUNT_1;
					is_dc_magnet_exist_duration=0;
					#if(defined(IRDA_TYPE_METER_AP) && (IRDA_TYPE_METER_AP == 1))					
						if(stop_adc_timer==1)
						{
							if(GET_EXLVD==0)
							{
								stop_Timer8bit0();
								stop_adc_timer=2;
							}
						}
					#else
						if(stop_adc_timer==1)
						{
							stop_Timer8bit0();
							stop_adc_timer=2;
						}
					#endif
					
				}
				
				
				#if (defined(IS_DISPLAY_AUTO_SCROLL_ON) && (IS_DISPLAY_AUTO_SCROLL_ON == 0))
					if((op_action_state&PFAIL_MODE_PULSE)==0)
					{
						mcu_flag|=DISPLAY_CHANGE_FLAG;
					}
				#endif
				if((op_state!=DO_ACTION_AT_POWER_RESTORE)&&(push_button_pressed_for_nm==0))
				{
					if(op_action_state&PFAIL_MODE_COM)
						switch2Low_Power_MODE(LOW_POWER_HALT);
					else
						switch2Low_Power_MODE(LOW_POWER_STOP);
					op_state=PFAIL_MODE_ADC;
				}
				if(push_button_pressed_for_nm==1)
				{
					push_button_pressed_for_nm=0;
					if(op_action_state&PFAIL_MODE_LCD)
					{
						mcu_flag|=EIGHT_BIT_TIMER1_FLAG;
						mcu_flag&=~DISPLAY_CHANGE_FLAG;
						if(display_no_of_para_count>0)
							display_no_of_para_count--;
					}
				}
					
				isRAMOK();
				break;
			//-----------------------------------------------------
			
			//-----------------------------------------------------
			case PFAIL_MODE_SWITCH_PRESS_AWAIT_LCD:
				if((op_action_state&PFAIL_MODE_SWITCH_PRESS_AWAIT_LCD)&&((battery_mode_lcd_on_count<BATTERY_LCD_LOCK_COUNT)||(battery_mode_nm_on_count<BATTERY_LCD_LOCK_COUNT)))
				{
					if(GET_PUSH_BUTTON_STATUS)
					{
						nm_false_current_detect_count=0;
						adc_on_counter=0;
						if(battery_mode_lcd_on_count<BATTERY_LCD_LOCK_COUNT)
						{
							mcu_flag&=~(EIGHT_BIT_TIMER1_FLAG|PUSH_RELEASED_NM_MODE);
							if((op_action_state&PFAIL_MODE_LCD)==0)
								init_Timer8bit1(LCD_SWITCH_ON_WAIT_TIME);
								
							op_action_state&=~PFAIL_MODE_SWITCH_PRESS_AWAIT_LCD;
							op_action_state|=PFAIL_MODE_SWITCH_PRESS_RELEASE_AWAIT_LCD;
						}
						

						
						if(battery_mode_nm_on_count<BATTERY_LCD_LOCK_COUNT)
						{
							if(Ins.EffectiveI==0)
							{
								doMeasurement_NM(0);
								doMeasurement_NM(0);
							}
							if(Ins.EffectiveI>0)
							{
								if(battery_mode_lcd_on_count>=BATTERY_LCD_LOCK_COUNT)
								{
									mcu_flag&=~(EIGHT_BIT_TIMER1_FLAG|PUSH_RELEASED_NM_MODE);
									if((op_action_state&PFAIL_MODE_LCD)==0)
										init_Timer8bit1(LCD_SWITCH_ON_WAIT_TIME);
									
									op_action_state&=~PFAIL_MODE_SWITCH_PRESS_AWAIT_LCD;
									op_action_state|=PFAIL_MODE_SWITCH_PRESS_RELEASE_AWAIT_LCD;
								}
							}
						}
						
					}
				}
				op_state=PFAIL_MODE_SWITCH_PRESS_RELEASE_AWAIT_LCD;
				break;
			//-----------------------------------------------------

			//-----------------------------------------------------
			case PFAIL_MODE_SWITCH_PRESS_RELEASE_AWAIT_LCD:
			
				if(op_action_state&PFAIL_MODE_SWITCH_PRESS_RELEASE_AWAIT_LCD)
				{
					if((GET_PUSH_BUTTON_STATUS==0)||(mcu_flag&EIGHT_BIT_TIMER1_FLAG))
					{
						if((op_action_state&PFAIL_MODE_LCD)==0)
							stop_Timer8bit1();
						if(mcu_flag&EIGHT_BIT_TIMER1_FLAG)
						{
							op_action_state|=SWITCH_ON_LCD;
						}
						else
						{
							if((op_action_state&PFAIL_MODE_LCD)==0)
								op_action_state=PFAIL_MODE_SWITCH_PRESS_AWAIT_LCD;
							else
								op_action_state|=PFAIL_MODE_SWITCH_PRESS_AWAIT_LCD;
							
						}
						op_action_state&=~PFAIL_MODE_SWITCH_PRESS_RELEASE_AWAIT_LCD;
						mcu_flag&=~EIGHT_BIT_TIMER1_FLAG;
						
					}
				}
				op_state=PFAIL_MODE_SWITCH_PRESS_AWAIT_COM;
				break;
			//-----------------------------------------------------
			
			//-----------------------------------------------------
			case PFAIL_MODE_SWITCH_PRESS_AWAIT_COM:
				if(op_action_state&PFAIL_MODE_SWITCH_PRESS_AWAIT_COM)
				{
					if((GET_PUSH_BUTTON_STATUS)&&(mcu_flag&PUSH_RELEASED_NM_MODE))
					{
						op_action_state&=~PFAIL_MODE_SWITCH_PRESS_AWAIT_COM;
						com_enable_wait_counter=0;
						start_RTC_Second_Interrupt();
						op_action_state|=PFAIL_MODE_SWITCH_PRESS_RELEASE_AWAIT_COM;
					}
				}
				op_state=PFAIL_MODE_SWITCH_PRESS_RELEASE_AWAIT_COM;
				break;
			//-----------------------------------------------------

			//-----------------------------------------------------
			case PFAIL_MODE_SWITCH_PRESS_RELEASE_AWAIT_COM:
			
					
				if(op_action_state&PFAIL_MODE_SWITCH_PRESS_RELEASE_AWAIT_COM)
				{
					if(mcu_flag&RTC_SECOND_FLAG)
					{
						if((++com_enable_wait_counter>=COM_SWITCH_ON_WAIT_TIME))
						{
							if((op_action_state&PFAIL_MODE_COM)==0)
							{
								if(com_enable_wait_counter==COM_SWITCH_ON_WAIT_TIME)
								{
									setParamIndex(MSG_ON_INDEX,1);		
									mcu_flag|=EIGHT_BIT_TIMER1_FLAG;
									op_action_state|=SWITCH_ON_COM;
									display_no_of_para_count=0;
									op_action_state&=~PFAIL_MODE_SWITCH_PRESS_RELEASE_AWAIT_COM;
												
								}
							}
						}
						mcu_flag&=~RTC_SECOND_FLAG;
					}
					if(GET_PUSH_BUTTON_STATUS==0)
					{
						op_action_state&=~PFAIL_MODE_SWITCH_PRESS_RELEASE_AWAIT_COM;
						op_action_state|=PFAIL_MODE_SWITCH_PRESS_AWAIT_COM;
					}
					
				}
				op_state=SWITCH_ON_LCD;
				break;
			//-----------------------------------------------------
			
				case SWITCH_ON_LCD:
					if(op_action_state&SWITCH_ON_LCD)
					{
						activePulsePower=0;
						op_action_state&=~(SWITCH_ON_LCD|PFAIL_MODE_PULSE);
						mcu_flag&=~DISPLAY_CHANGE_FLAG;
						adc_switch_wait_time=0;
						mcu_flag&=~ADC_LOAD_SECOND_STAGE;
						mcu_flag&=~ADC_NO_LOAD_FIRST_STAGE;
						#if(defined(IRDA_TYPE_METER_AP_WARNGL) && (IRDA_TYPE_METER_AP_WARNGL == 1))
						//	setParaBuff(DISPLAY_MODE_PUSH);
						setParaBuff(DISPLAY_MODE_NORMAL);
						#else
							setParaBuff(DISPLAY_MODE_UOUT_NORMAL);
						#endif
					
						op_action_state|=(PFAIL_MODE_LCD|PFAIL_MODE_SWITCH_PRESS_AWAIT_COM);
						Clear_Lcd();
						R_LCD_PowerOn();
						mcu_flag|=EIGHT_BIT_TIMER1_FLAG;
						if((DSP_FAST_MODE_INTERVAL!=(TRTCMP0/256))&&(TSTART00==1))
							start_Timer8bit0(DSP_FAST_MODE_INTERVAL);
							
						if(battery_mode_nm_on_count<BATTERY_LCD_LOCK_COUNT)
						{
							if(Ins.EffectiveI>0)
							{
								op_action_state|=PFAIL_MODE_PULSE;
								mcu_flag|=START_PULSE_NM_MODE;
								mcu_flag|=PULSE_NM_MODE;
								battery_mode_nm_on_count++;
								
							}
						}
						display_no_of_para=getParaCount();
						if(op_action_state&PFAIL_MODE_PULSE)
						{
							setParaBuff(DISPLAY_MODE_PUSH);
							mcu_flag|=DISPLAY_CHANGE_FLAG;
							
							display_no_of_para=30;
						}
						
						display_no_of_para_count=1;
						getRtcCounter(RTC_CHECK_MODE);
						
						//battery_mode_lcd_on_count=0;
						
					}
					op_state=SWITCH_ON_PULSE;
				break;
				case SWITCH_ON_COM:
					if(op_action_state&SWITCH_ON_COM)
					{
						op_action_state&=~SWITCH_ON_COM;
						display_no_of_para_count=0;
						
						//-----------------------
						#if (defined(UART_0_ENABLE) && (UART_0_ENABLE == 1))
							init_Uart0();	//enable commuication
						#endif
						#if (defined(UART_2_ENABLE) && (UART_2_ENABLE == 1))
							init_Uart2();
						#endif
						if(TMKAEN==0)
							init_Timer12bit(0);// enable 12 bit timer
						//-----------------------
						mcu_flag|=COMM_RECEIVE_ENABLE;
						op_action_state|=PFAIL_MODE_COM;
						mcu_flag&=~COMM_TRANSMIT_FLAG;
						SWITCH_ON_COMM_VCC;
						op_action_state|=PFAIL_MODE_COM;
						
						//op_action_state|=PFAIL_MODE_SWITCH_PRESS_AWAIT_COM;
						if((op_action_state&PFAIL_MODE_PULSE)==0)
							display_no_of_para=getParaCount();
						if(display_no_of_para<6)
							display_no_of_para=6;
					}
					op_state=PFAIL_MODE_COM;
				break;
				case SWITCH_ON_PULSE:
					op_state=PFAIL_MODE_PULSE;
				break;
				
				case SWITCH_OFF_LCD:
				
					if(op_action_state&SWITCH_OFF_LCD)
					{
						if(((mcu_flag&PUSH_RELEASED_NM_MODE)&&((GET_PUSH_BUTTON_STATUS)==0))||((mcu_flag&PUSH_RELEASED_NM_MODE)==0))						
						{
							ITMC&=~0x80;	// stop 12 bit timer
							TMKAEN=0;
							#if (defined(UART_0_ENABLE) && (UART_0_ENABLE == 1))
								Uart0_Stop();// switch off uart
							#endif
							#if (defined(UART_2_ENABLE) && (UART_2_ENABLE == 1))
								Uart2_Stop();// switch off uart
							#endif
							SWITCH_OFF_PULSE_LED;
							SWITCH_OFF_COMM_VCC;
							activePulsePower=0;
							mcu_flag&=~(COMM_RECEIVE_ENABLE|START_PULSE_NM_MODE|PULSE_NM_MODE);
							stop_RTC_Second_Interrupt();
							if(battery_mode_display_time>=BATTERY_LCD_DISPLAY_DURATION)
							{
								stop_Timer8bit1();
								R_LCD_PowerOff();//switch off lcd
								op_action_state=PFAIL_MODE_SWITCH_PRESS_AWAIT_LCD;
								setParaBuff(DISPLAY_MODE_UOUT_NORMAL);
							}
							else
							{
								if(Ins.EffectiveI>0)
									setParaBuff(DISPLAY_MODE_NORMAL);
								else
									setParaBuff(DISPLAY_MODE_UOUT_NORMAL);
									
								//setNextParaSeq();
								
								display_no_of_para_count=0;
								op_action_state|=PFAIL_MODE_SWITCH_PRESS_AWAIT_LCD;
								op_action_state&=~SWITCH_OFF_LCD;
								op_action_state&=~PFAIL_MODE_COM;
								mcu_flag&=~DISPLAY_CHANGE_FLAG;
								
							}
							
						}
						else
						{
							op_action_state&=~SWITCH_OFF_LCD;
							op_action_state&=~PFAIL_MODE_PULSE;
							display_no_of_para_count=0;
						}
					}
					
					op_state=COVER_OPEN_ACTION;
				break;
				
			//-----------------------------------------------------
			case PFAIL_MODE_LCD:
				
				if((mcu_flag&EIGHT_BIT_TIMER1_FLAG))
				{
					
					if(op_action_state&PFAIL_MODE_LCD)
					{
						if(display_no_of_para_count<display_no_of_para)
						{
							if((mcu_flag&DISPLAY_CHANGE_FLAG)==0)
							{
								setParamIndex(*displayParamPtr,0);
								setNextParaSeq();
								
							}
							
							Display(BATTERY_MODE);
							
							getRtcCounter(RTC_CHECK_MODE);
							if(mcu_flag&PUSH_RELEASED_NM_MODE)
							{
								
								#if (defined(IS_LCD_OFF_AFTER_REL_20SECS) && (IS_LCD_OFF_AFTER_REL_20SECS == 1))
									if(((op_action_state&PFAIL_MODE_COM)==0)&&((op_action_state&PFAIL_MODE_PULSE)==0))
										display_no_of_para=2;
								#endif
					
								if((op_action_state&PFAIL_MODE_PULSE)||(battery_mode_display_time>=BATTERY_LCD_DISPLAY_DURATION)||(op_action_state&PFAIL_MODE_COM))
								{
									display_no_of_para_count++;
								}
							}
							
							start_Timer8bit1(getParamTime());
							
						}
						else
						{
							op_action_state|=SWITCH_OFF_LCD;
						}
						
					}
					
					mcu_flag&=~EIGHT_BIT_TIMER1_FLAG;
				}
				op_state=SWITCH_ON_COM;
				break;
			//-----------------------------------------------------
			
			//-----------------------------------------------------
			case PFAIL_MODE_PULSE:
				
				if(op_action_state&PFAIL_MODE_PULSE)
				{
					if(mcu_flag&PUSH_BUTTON_RELEASED)
					{
						//mcu_flag&=~DISPLAY_CHANGE_FLAG;
						mcu_flag&=~PUSH_BUTTON_RELEASED;	
						//mcu_flag|=EIGHT_BIT_TIMER1_FLAG;
						if(display_no_of_para_count>0)
							display_no_of_para_count--;
					}
				}	
				if((GET_PUSH_BUTTON_STATUS==0)&&(op_action_state&PFAIL_MODE_LCD))
				{
					mcu_flag|=PUSH_RELEASED_NM_MODE;
				}
				
				op_state=PFAIL_MODE_LCD;
				break;
			//-----------------------------------------------------
			
			//-----------------------------------------------------
			case PFAIL_MODE_COM:
				//----------------------------------------
					if(op_action_state&PFAIL_MODE_COM)
					{
					//----------------------------------------
						if(mcu_flag&COMM_RECEIVE_OVER)
						{
							mcu_flag&=~COMM_RECEIVE_OVER;
							#if (defined(IRDA_TYPE_METER_HP) && (IRDA_TYPE_METER_HP == 1))							
								if(RxTxBuffer[0]==START_OF_CHAR_OPTICAL)
									Communication(USER_CODE);
								else if(RxTxBuffer[0]==START_OF_CHAR_IRDA)
									CustomerProtocol();
							#endif
							#if (defined(IRDA_TYPE_METER_HP) && (IRDA_TYPE_METER_HP == 0))							
								if(RxTxBuffer[0]==START_OF_CHAR_OPTICAL)
									Communication(USER_CODE);
							#endif
							
							mcu_flag |= COMM_RECEIVE_ENABLE;       //Enable reception
						}
					//----------------------------------------
						#if (defined(IS_DLMS_ENABLED_EEPL) && (IS_DLMS_ENABLED_EEPL == 1))
						/* DLMS process of request and response message */		
						#if (defined(UART_0_ENABLE) && (UART_0_ENABLE == 1))
						DLMSMessageProcess();
						#endif
						#endif
						
						if(mcu_flag&COMM_TRANSMIT_FLAG)
						{
							mcu_flag&=~COMM_TRANSMIT_FLAG;
							display_no_of_para_count=0;
							getRtcCounter(RTC_CHECK_MODE);
							Display(BATTERY_MODE);
							
						}
						
					}
				//----------------------------------------
				op_state=SWITCH_OFF_LCD;
				break;
			//-----------------------------------------------------
			
			//-----------------------------------------------------
			case PFAIL_MODE_ADC:
				if((mcu_flag&ONE_SECOND_OVER_FLAG)&&(stop_adc_timer==0))
				{
					mcu_flag&=~ONE_SECOND_OVER_FLAG;
					//start adc in NM mode here
					//if((op_action_state&PFAIL_MODE_COM)==0)
					mcu_flag|=DO_MEASUREMENT;
						
					InsSave.timeCounter+=(TRTCMP0/256);
					if(battery_mode_display_time<BATTERY_LCD_DISPLAY_DURATION)
						battery_mode_display_time+=(TRTCMP0/256);
						
				}
				
				if(mcu_flag&DO_MEASUREMENT)
				{
					mcu_flag&=~DO_MEASUREMENT;
					if(adc_on_counter>0)
						adc_on_counter--;
					
					if((GET_MAINS_STATUS==0)&&(adc_on_counter==0))
						doMeasurement_NM(0);
						
					
					adc_switch_wait_time+=(TRTCMP0/256);
					
					if((Ins.EffectiveI>0)&&(GET_MAINS_STATUS==0))
					{
						getRtcCounter(RTC_CHECK_MODE);
						addPulsesActive(no_of_pulses_in_nm_Active);
						
						addPulsesApp(no_of_pulses_in_nm_App);
						
						InsSave.PowerOn30+=(TRTCMP0/256);
						InsSave.lsAvgC=InsSave.lsAvgC+Ins.EffectiveI*(TRTCMP0/256);
						
						battery_mode_display_time=0;
						
						if(mcu_flag&PULSE_NM_MODE)
						{
							init_Timer12bit(0);// enable 12 bit timer
							mcu_flag&=~PULSE_NM_MODE;
						}
						
						if((ModeID==DISPLAY_MODE_UOUT_NORMAL)||(ModeID==DISPLAY_MODE_UOUT_COVER))
						{
							setParaBuff(DISPLAY_MODE_NORMAL);
							mcu_flag&=~DISPLAY_CHANGE_FLAG;
							
						}
						else if(ModeID==DISPLAY_MODE_NORMAL)
							if(((op_action_state&PFAIL_MODE_PULSE)==0)&&((op_action_state&PFAIL_MODE_COM)==0))
								display_no_of_para_count=0;
						
						//battery_mode_lcd_on_count=0;
						mcu_flag|=SAVE_AT_NM_EXIT_FLAG;
						display_flag=DISP_NM;
						if((TRTCMP0/256)==1)
						{
							if(adc_switch_wait_time>120)
								start_Timer8bit0(DSP_FAST_MODE_INTERVAL);
						}						
						updateEnergyPulsePara(TRTCMP0/256);
						
						
						
						nowNMkWh=InsSave.CumkWh+InsSave.ZkWhCounter/METER_CONSTANT;
						
						if((((nowNMkWh-prekWh) >=NM_KWH_THRESHOLD_LL)&&((nowNMkWh-prekWh) <=NM_KWH_THRESHOLD_UL))||(((nowNMkWh-prekWhinterval) >=NM_KWH_THRESHOLD_LL)&&((nowNMkWh-prekWhinterval) <=NM_KWH_THRESHOLD_UL)))
						{
							isRAMOK();
							savekWh(nowNMkWh);
							prekWh=nowNMkWh;
							prekWhinterval=nowNMkWh;
						}	
						getSetPfailData(ENERGY_W_RAM);
						
						if((mcu_flag&START_LS_NM_MODE)==0)
						{
							if(is_vfail_with_load==0)
								updateEnergyPulsePara(TRTCMP0/256+2);
								
							mcu_flag|=START_LS_NM_MODE;
							mcu_flag&=~POWER_DOWN_FLAG;
							if(isPowerfailLog(BATTERY_MODE))
							{
								powerfailLogging();
								updatePfailEvent();
								
							}
							if((DSP_NO_LOAD_MODE_INTERVAL==(TRTCMP0/256))&&(TSTART00==1))
							{
								adc_on_counter=0;
								start_Timer8bit0(DSP_SLOW_MODE_INTERVAL);
							}
						}
						
						if(op_action_state&PFAIL_MODE_PULSE)
						{
							if((mcu_flag&START_PULSE_NM_MODE)==0)
							{
								if((InsSave.timeCounter/NM_PULSE_INTERVAL )!=(prevTimeCounter/NM_PULSE_INTERVAL))
									mcu_flag|=START_PULSE_NM_MODE;
							}
							else
							{
								doMeasurement_NM(1);
								activePulsePower=REFERENCEVOLTAGE*Ins.EffectiveI;
								mcu_flag&=~START_PULSE_NM_MODE;
							}
						}
							
							

						if((InsSave.timeCounter/900 )!=(prevTimeCounter/900))
							doDatalog_NM();
						if(Ins.Voltage>0)
						{
							if(((CurrentTamperStatusFlag&TAMPER_AC_DC)==0)||((CurrentTamperStatusFlag_AP&TAMPER_AC_DC)==0))
							{
								TamperTime_counter[5]=0;
								
								CurrentTamperStatusFlag|=(TamperRecord.TamperStatusFlag|TAMPER_AC_DC);
								CurrentTamperStatusFlag_AP|=(TamperRecord_AP.TamperStatusFlag|TAMPER_AC_DC);
								if(((TamperRecord.TamperStatusFlag&TAMPER_NEU_MISS)==0)||((TamperRecord_AP.TamperStatusFlag&TAMPER_NEU_MISS)==0))
									CurrentTamperStatusFlag&=~TAMPER_NEU_MISS;
									CurrentTamperStatusFlag_AP&=~TAMPER_NEU_MISS;
									
								
							}
							display_flag|=DISP_EARTH;
						}
						else
						{
							if(((CurrentTamperStatusFlag&TAMPER_NEU_MISS)==0)||((CurrentTamperStatusFlag_AP&TAMPER_NEU_MISS)==0))
							{
								TamperTime_counter[0]=0;
								CurrentTamperStatusFlag|=(TamperRecord.TamperStatusFlag|TAMPER_NEU_MISS);
								CurrentTamperStatusFlag_AP|=(TamperRecord_AP.TamperStatusFlag|TAMPER_NEU_MISS);
								if(((TamperRecord.TamperStatusFlag&TAMPER_AC_DC)==0)||((TamperRecord_AP.TamperStatusFlag&TAMPER_AC_DC)==0))
									CurrentTamperStatusFlag&=~TAMPER_AC_DC;
									CurrentTamperStatusFlag_AP&=~TAMPER_AC_DC;
									
							}
						}

						if((((TamperRecord.TamperStatusFlag&TAMPER_NEU_MISS) ^ (CurrentTamperStatusFlag &TAMPER_NEU_MISS))>0)||(((TamperRecord_AP.TamperStatusFlag&TAMPER_NEU_MISS) ^ (CurrentTamperStatusFlag_AP &TAMPER_NEU_MISS))>0))
						{
							if(TamperTime_counter[0]==0)
								TamperTime_counter[0]+=DSP_FAST_MODE_INTERVAL;
							else
								TamperTime_counter[0]+=(TRTCMP0/256);
								
							if(TamperTime_counter[0]>=60)
							{
								Ins.PowerFactor=0;
								Tamperlog();
							}
						}
						else
						{
							
							if((((TamperRecord.TamperStatusFlag&TAMPER_AC_DC) ^ (CurrentTamperStatusFlag &TAMPER_AC_DC))>0)||(((TamperRecord_AP.TamperStatusFlag&TAMPER_AC_DC) ^ (CurrentTamperStatusFlag_AP &TAMPER_AC_DC))>0))
							{
								TamperTime_counter[5]+=(TRTCMP0/256);
								if(TamperTime_counter[5]>=60)
								{
									Ins.PowerFactor=100;
									Tamperlog();

								}
							}
						}


						prevTimeCounter=InsSave.timeCounter;
						
						if((mcu_flag&ADC_LOAD_SECOND_STAGE)==0)
						{
							if((adc_switch_wait_time>DSP_LOAD_SECOND_STAGE_WAIT_TIME)&&(TSTART00==1))
							{
								adc_on_counter=0;
								mcu_flag|=ADC_LOAD_SECOND_STAGE;
								start_Timer8bit0(DSP_SLOW_MODE_INTERVAL);
							}
						}
						Display(BATTERY_MODE);
					}
					else
					{
						mcu_flag&=~START_LS_NM_MODE;
						display_flag=0;
						Ins.Voltage=0;
						if(Ins.EffectiveI==0)
						{
							activePowerNM=0;
							apperentPowerNM=0;
							is_vfail_with_load=0;
							Ins.PowerFactor=0;
							
							if((TRTCMP0/256)==1)
							{
								start_Timer8bit0(DSP_FAST_MODE_INTERVAL);
							}
						}
						if(!((ModeID==DISPLAY_MODE_UOUT_NORMAL)||(ModeID==DISPLAY_MODE_UOUT_COVER)))
						{
							if((mcu_flag&POWER_DOWN_FLAG)==0)
							{
								if(Ins.EffectiveI>0)
									setParaBuff(DISPLAY_MODE_NORMAL);		
								else
									setParaBuff(DISPLAY_MODE_UOUT_NORMAL);
								mcu_flag|=DISPLAY_CHANGE_FLAG;
							}
							
						}
							
						
						
						if((mcu_flag&POWER_DOWN_FLAG)==0)
						{
							mcu_flag|=POWER_DOWN_FLAG;
							if(mcu_flag&SAVE_AT_NM_EXIT_FLAG)
							{
								addPulsesActive(no_of_pulses_in_nm_Active);
								
								addPulsesApp(no_of_pulses_in_nm_App);
							}
							
						}
						
						if((mcu_flag&ADC_NO_LOAD_FIRST_STAGE)==0)
						{
							if((adc_switch_wait_time>DSP_NO_LOAD_FIRST_STAGE_WAIT_TIME)&&(TSTART00==1))
							{
								adc_on_counter=0;
								mcu_flag|=ADC_NO_LOAD_FIRST_STAGE;
								start_Timer8bit0(DSP_SLOW_MODE_INTERVAL);
							}
						}
						else
						{
							if(adc_switch_wait_time>DSP_NO_LOAD_SECOND_STAGE_WAIT_TIME)
							{
								if(adc_on_counter==0)
									adc_on_counter=4;
								
								if((DSP_NO_LOAD_MODE_INTERVAL!=(TRTCMP0/256))&&(TSTART00==1))
									start_Timer8bit0(DSP_NO_LOAD_MODE_INTERVAL);
							}							
						}
					}
					//getRtcCounter(RTC_CHECK_MODE);
					if(op_action_state&PFAIL_MODE_PULSE)
					{
						
						if(Ins.EffectiveI==0)
						{
							activePulsePower=0;
							mcu_flag&=~START_PULSE_NM_MODE;
							
							
						}
						
						Display(BATTERY_MODE);
					}					
				}
				op_state=PFAIL_MODE_SWITCH_PRESS_AWAIT_LCD;
				if((InsSave.timeCounter/86400)!=(prevTimeCounter/86400))
				{
					if(BATTERY_LCD_LOCK_COUNT==battery_mode_nm_on_count)
						battery_mode_nm_on_count=0;
					
				}
				
				break;
			//-----------------------------------------------------

			//-----------------------------------------------------
			case COVER_OPEN_ACTION:
				op_state=PFAIL_MODE_STOP;
				if(((TamperRecord.TamperStatusFlag & TAMPER_COVEROPEN)==0)||((TamperRecord_AP.TamperStatusFlag & TAMPER_COVEROPEN)==0))// if cover open is not there
				{
					while(GET_COVER_STATUS)	// check cover open pin 
					{
						if(TamperTime_counter[3]++>=COVER_OPEN_WAIT_TIME)
						{
							CurrentTamperStatusFlag |= TAMPER_COVEROPEN;
							CurrentTamperStatusFlag_AP |= TAMPER_COVEROPEN;
							getRtcCounter(RTC_CHECK_MODE);
							Tamperlog();
							if((op_action_state&PFAIL_MODE_LCD)==0)
							{
								init_Timer8bit1(LCD_SWITCH_ON_WAIT_TIME);
								Clear_Lcd();
								R_LCD_PowerOn();
								display_no_of_para=3;
								op_action_state|=PFAIL_MODE_LCD;
							}
							//setNextParaSeq();
							display_no_of_para_count=0;
							mcu_flag|=EIGHT_BIT_TIMER1_FLAG;
							op_state=SWITCH_ON_LCD;
							break;
						}
					}
				}
				else
				{
					CurrentTamperStatusFlag |= TAMPER_COVEROPEN;
					CurrentTamperStatusFlag_AP |= TAMPER_COVEROPEN;
				}
				break;
			//-----------------------------------------------------
			
			default:
				if(op_state!=EXIT_PFAIL_OPERATION)
					op_state=PFAIL_MODE_STOP;
				break;
		}
	}while(op_state!=EXIT_PFAIL_OPERATION);

	isRAMOK();	
	if(nowNMkWh>0)
	{
		
		if((mcu_flag&SAVE_AT_NM_EXIT_FLAG)&&((nowNMkWh-prekWh)<=LIMIT_DIFF_THRESHOLD))
		{
			saveLSatExit();
			SaveEEPROM(PARA_WRITE_AFTER_STATE  ,PARA_TYPE_INSTANT); // save at after location
			InsSave.timeCounter=prevTimeCounter;
			SaveEEPROM(PARA_WRITE_AFTER_STATE ,PARA_TYPE_PFAIL_INSTANT); // save at current location
			getSetPfailData(ENERGY_W_RAM);
		}
	}
	
	
	
	if(battery_mode_nm_on_count>0)
		write_eeprom((uint8_t *)&battery_mode_nm_on_count,BAT_NM_LOCK_COUNT_LOC,1);

	#if (defined(LOG_NM_INFO) && (LOG_NM_INFO == 1))
		write_eeprom((uint8_t *)&nmInfo,NM_INFO_SHT_LOC,12);
	#endif	
//	RxTxBuffer[0]=0xAB;

//	write_eeprom(RxTxBuffer,RTC_BAT_STATUS_LOC,1);
	
	if(is_vfail_with_load==0)
		WDT_Restart(0x00);
}

void updatePfailEvent(void)
{
	uint16_t EventEntries;
	if(InsSave.PFailCount>PFAIL_EVENTS )
		EventEntries = PFAIL_EVENTS*2;
	else
		EventEntries = InsSave.PFailCount*2;
	
	g_Class07_Event_EntriesInUse[1]=EventEntries;

}
//#endif
void addPulsesActive(uint16_t no_of_pulses)
{
	InsSave.kWhCounterMD+=no_of_pulses;
	InsSave.ZkWhCounter+=no_of_pulses;
	InsSave.kWhCounter+=no_of_pulses;
	no_of_pulses_in_nm_Active=0;
	
}
void addPulsesApp(uint16_t no_of_pulses)
{
	InsSave.kVAhCounterMD+=no_of_pulses;
	InsSave.ZkVAhCounter+=no_of_pulses;
	InsSave.kVAhCounter+=no_of_pulses;
	no_of_pulses_in_nm_App=0;	
}

void updateEnergyPulsePara(uint8_t mVal)
{
	apperentPowerNM=REFERENCEVOLTAGE*Ins.EffectiveI*mVal;
	activePowerNM=apperentPowerNM;

}
