
//-------- include device specific files----------------
#include "..\\..\\rlDevice\\Include\\dIOdefine.h"
#include "..\\..\\rlDevice\\Include\\dI2c.h"
#include "..\\..\\rlDevice\\Include\\dUart.h"
#include "..\\..\\rlDevice\\Include\\dRtc.h"
#include "..\\..\\rlDevice\\Include\\dWatchDog.h"
//-----------------------------------------------------

//-------- include app specific files-------------------
#include "..\\Include\\AppTampers.h"
#include "..\\Include\\AppCommunication.h"
#include "..\\Include\\AppCustomerProtocol.h"
#include "..\\Include\\AppCalibration.h"
#include "..\\Include\\AppLcd.h"
#include "..\\Include\\AppVariables.h"
#include "..\\Include\\AppEeprom.h"
#include "..\\Include\\AppMacros.h"
#include "..\\Include\\AppBlockDailyLS.h"
#include "..\\Include\\AppConfig.h"
#include "..\\Include\\AppMisc.h"
#include "..\\Include\\AppBilling.h"
#include "..\\Include\\AppTOD.h"

//-------- include dlms specific files-------------------
#include "..\\..\\rlDlms\\meter_app\\r_dlms_user_interface.h"



#if (defined(IRDA_TYPE_METER_HP) && (IRDA_TYPE_METER_HP == 1))
/*----------------------------------------------------------------------------*/
extern Unsigned32	g_Class07_Event_EntriesInUse[];
extern const uint8_t dlmsEventCode[MAX_TAMPER_COUNT][2];
const uint8_t FirmwareVersion[]= "          EEOr10";

void setComFrameBytes(uint16_t bytes);
void Read_Command(uint8_t commandId );
void AppMeter_Response(void);
void addCheckSum(uint8_t length);
uint8_t ValidateCheckSum(uint8_t length);
uint16_t ls_entries_requested=0;
/********************* Communication Protocol *********************************/
void CustomerProtocol(void)
{   
    uint8_t cmdID;
    cmdID=ConvertPAN((uint8_t*)&RxTxBuffer[PROTO_IDENTIFIER]);
    
    mcu_flag &= ~COMM_RECEIVE_ENABLE;
    if(ValidateCheckSum(ConvertPAN((uint8_t*)&RxTxBuffer[PROTO_NO_BYTES])))
	{
		setParamIndex(MSG_DNLD_INDEX,1);
	    if((cmdID >= START_CMD_ID)&&(cmdID <= END_CMD_ID))
	    {   
			Read_Command(cmdID);
	    }
	    else
	    { 
			RxTxBuffer[PROTO_IDENTIFIER]=RxTxBuffer[PROTO_IDENTIFIER]|0x80;
			RxTxBuffer[PROTO_DATA_BYTES] = 0x01;
			setComFrameBytes(1);
	    }
	}
	else
	{
			RxTxBuffer[PROTO_IDENTIFIER]=RxTxBuffer[PROTO_IDENTIFIER]|0x80;		
			RxTxBuffer[PROTO_DATA_BYTES] = 0x02;
			setComFrameBytes(1);
	}
	
	if(PROTO_READ_FORMAT_DATA!=cmdID)
    	AppMeter_Response();
	
    
}
/*----------------------------------------------------------------------------*/

/***************** Read the data from meter ***********************************/
void Read_Command(uint8_t commandId )
{   
	uint8_t i,no_of_itr=0;
    uint32_t temp;
	uint16_t loc;
	Apprtc rtc;
	uint16_t data_position;
//	uint8_t buff_t[2];
	
    switch(commandId)
    {   
        //----------------------------------------------------------------------
        case PROTO_READ_RTC_DATA:
			
            loc=0;
            Dec_Ascii(Ins.Voltage/10,4,(uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES],1);
			loc=loc+5;
            RxTxBuffer[PROTO_DATA_BYTES+loc]=FIELD_SEPERATOR;
			loc+=1;

            Dec_Ascii(Ins.EffectiveI/10,4,(uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES+loc],2);
			loc=loc+5;
            RxTxBuffer[PROTO_DATA_BYTES+loc]=FIELD_SEPERATOR;
			loc+=1;
            
            Data.long_data=ConvertTimeCounterToTime(InsSave.timeCounter,DATE_VAL,&rtc);
            Dec_Ascii(Data.long_data,6,(uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES+loc],0);
			loc=loc+6;
            RxTxBuffer[PROTO_DATA_BYTES+loc]=FIELD_SEPERATOR;
			loc+=1;
            
            Data.long_data=ConvertTimeCounterToTime(InsSave.timeCounter,TIME_VAL,&rtc);
            Dec_Ascii(Data.long_data,6,(uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES+loc],0);
			loc=loc+6;
            RxTxBuffer[PROTO_DATA_BYTES+loc]=FIELD_SEPERATOR;
			loc+=1;
            
            Dec_Ascii(Ins.PowerFactor,3,(uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES+loc],2);
			loc=loc+4;
            RxTxBuffer[PROTO_DATA_BYTES+loc]=FIELD_SEPERATOR;
			loc+=1;
            
			
            Dec_Ascii(Ins.Frequency,4,(uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES+loc],2);
			loc=loc+5;
            RxTxBuffer[PROTO_DATA_BYTES+loc]=FIELD_SEPERATOR;
			loc+=1;
			
            Dec_Ascii(Ins.EffectiveP,7,(uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES+loc],2);
			loc=loc+8;
            RxTxBuffer[PROTO_DATA_BYTES+loc]=FIELD_SEPERATOR;
			loc+=1;
			
            Dec_Ascii(Ins.AppPower,7,(uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES+loc],2);
			loc=loc+8;
            RxTxBuffer[PROTO_DATA_BYTES+loc]=FIELD_SEPERATOR;
			loc+=1;
            
			Dec_Ascii(InsSave.MDResetCount,3,(uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES+loc],0);
			loc=loc+3;
            RxTxBuffer[PROTO_DATA_BYTES+loc]=FIELD_SEPERATOR;
			loc+=1;
			Dec_Ascii(getSelftDgFeature(),1,(uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES+loc],0);
			loc+=1;
            RxTxBuffer[PROTO_DATA_BYTES+loc]=FIELD_SEPERATOR;
			loc+=1;
			Dec_Ascii(getTotalTamperCount_AP(),3,(uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES+loc],0);
			loc+=3;
            RxTxBuffer[PROTO_DATA_BYTES+loc]=FIELD_SEPERATOR;
			loc+=1;
			Dec_Ascii(3600/md_period,4,(uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES+loc],0);
			loc+=4;
            RxTxBuffer[PROTO_DATA_BYTES+loc]=FIELD_SEPERATOR;
			loc+=1;
			Dec_Ascii(3600/ls_period,4,(uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES+loc],0);
			loc+=4;
            RxTxBuffer[PROTO_DATA_BYTES+loc]=FIELD_SEPERATOR;
			loc+=1;
			temp=getBillingDate();
			RxTxBuffer[PROTO_DATA_BYTES+loc]=temp>>8;
			RxTxBuffer[PROTO_DATA_BYTES+loc+1]=temp;
			loc+=2;
            RxTxBuffer[PROTO_DATA_BYTES+loc]=FIELD_SEPERATOR;
			loc+=1;
			//--------------
			Dec_Ascii(TamperRecord_AP.TamperCount[0],4,(uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES+loc],0);
			loc+=4;
            RxTxBuffer[PROTO_DATA_BYTES+loc]=FIELD_SEPERATOR;
			loc+=1;
			Dec_Ascii(TamperRecord_AP.TamperCount[1],4,(uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES+loc],0);
			loc+=4;
            RxTxBuffer[PROTO_DATA_BYTES+loc]=FIELD_SEPERATOR;
			loc+=1;
			Dec_Ascii(TamperRecord_AP.TamperCount[2],4,(uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES+loc],0);
			loc+=4;
            RxTxBuffer[PROTO_DATA_BYTES+loc]=FIELD_SEPERATOR;
			loc+=1;
			Dec_Ascii(TamperRecord_AP.TamperCount[3],4,(uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES+loc],0);
			loc+=4;
            RxTxBuffer[PROTO_DATA_BYTES+loc]=FIELD_SEPERATOR;
			loc+=1;
			Dec_Ascii(TamperRecord_AP.TamperCount[4],4,(uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES+loc],0);
			loc+=4;
            RxTxBuffer[PROTO_DATA_BYTES+loc]=FIELD_SEPERATOR;
			loc+=1;
			Dec_Ascii(TamperRecord_AP.TamperCount[5],4,(uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES+loc],0);
			loc+=4;
            RxTxBuffer[PROTO_DATA_BYTES+loc]=FIELD_SEPERATOR;
			loc+=1;
			Dec_Ascii(TamperRecord_AP.TamperCount[6],4,(uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES+loc],0);
			loc+=4;
            RxTxBuffer[PROTO_DATA_BYTES+loc]=FIELD_SEPERATOR;
			loc+=1;
			Dec_Ascii(TamperRecord_AP.TamperCount[7],4,(uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES+loc],0);
			loc+=4;
            RxTxBuffer[PROTO_DATA_BYTES+loc]=FIELD_SEPERATOR;
			loc+=1;
			Dec_Ascii(TamperRecord_AP.TamperCount[8],4,(uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES+loc],0);
			loc+=4;
            RxTxBuffer[PROTO_DATA_BYTES+loc]=FIELD_SEPERATOR;
			loc+=1;
			Dec_Ascii(g_Class07_Event_EntriesInUse[2],2,(uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES+loc],0);
			loc+=2;
			
			//--------------
			
			
			
			
			setComFrameBytes(loc);
            
        break;
        //----------------------------------------------------------------------
        //----------------------------------------------------------------------
        case PROTO_READ_SERIAL_NO:
						setComFrameBytes(14);
//            getMeterSerial((uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES],SR_TYPE_ASCII,0);
 						getMeterSerial((uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES],SR_TYPE_ASCII);
            
            
        break;
        //----------------------------------------------------------------------
        //----------------------------------------------------------------------
        case PROTO_READ_MFG_ID:
			setComFrameBytes(16);
            for(i=0;i<16;i++)
              RxTxBuffer[PROTO_DATA_BYTES+i]=FirmwareVersion[i];
        break;
        //----------------------------------------------------------------------
        //----------------------------------------------------------------------
        case PROTO_READ_CUM_KWH:
			setComFrameBytes(9);
            Dec_Ascii(InsSave.CumkWh+InsSave.ZkWhCounter/32,8,(uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES],2);
        break;
        //----------------------------------------------------------------------
        //----------------------------------------------------------------------
      /*  case PROTO_READ_TSTATUS_HISTORY:
			setComFrameBytes(17);
            for(i=0;i<6;i++)
            {
              read_eeprom((uint8_t *)&Data.long_data,GetBillLoc(i)+39,2);
              Dec_Ascii(Data.Short.lsb,3,(uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES+i*3],0);
              if(i<5)
                RxTxBuffer[PROTO_DATA_BYTES+i*3+2]=RECORD_SEPERATOR;
            }
	break;
	 for(i=0;i<6;i++)
            {
              read_eeprom((uint8_t *)&Data.long_data,GetBillLoc(i)+30,1);
              Dec_Ascii(Data.byte[0],2,(uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES+i*3],0);
              if(i<5)
                RxTxBuffer[PROTO_DATA_BYTES+i*3+2]=RECORD_SEPERATOR;
            }*/
              
	   case PROTO_READ_TSTATUS_HISTORY:
            setComFrameBytes(17);
            for(i=0;i<6;i++)
            {
              read_eeprom((uint8_t *)&Data.long_data,GetBillLoc(i)+39,1);
							
              Dec_Ascii(Data.byte[0],2,(uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES+i*3],0);
              if(i<5)
                RxTxBuffer[PROTO_DATA_BYTES+i*3+2]=RECORD_SEPERATOR;
            }
        break;
        //----------------------------------------------------------------------
        //----------------------------------------------------------------------
        case PROTO_READ_KWH_CON_HISTORY:
			setComFrameBytes(47);
            for(i=0;i<6;i++)
            {
              read_eeprom((uint8_t *)&Data.long_data,GetBillLoc(i)+31,4);
              Dec_Ascii(Data.long_data,6,(uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES+i*8],2);
              if(i<5)
                RxTxBuffer[PROTO_DATA_BYTES+i*8+7]=RECORD_SEPERATOR;
            }        
        break;
        //----------------------------------------------------------------------
        //----------------------------------------------------------------------
        case PROTO_READ_MD_HISTORY:
			setComFrameBytes(107);
            for(i=0;i<6;i++)
            {
              read_eeprom((uint8_t *)&Data.long_data,GetBillLoc(i)+12,2);
              Dec_Ascii(Data.Short.lsb,4,(uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES+i*18],2);
              RxTxBuffer[PROTO_DATA_BYTES+i*18+5]=FIELD_SEPERATOR;
              
              read_eeprom((uint8_t *)&temp,GetBillLoc(i)+14,4);
              Data.long_data=ConvertTimeCounterToTime(temp,DATE_VAL,&rtc);
              Dec_Ascii(Data.long_data,6,(uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES+i*18+6],0);
              RxTxBuffer[PROTO_DATA_BYTES+i*18+12]=FIELD_SEPERATOR;
              
              Data.long_data=ConvertTimeCounterToTime(temp,TIME_VAL,&rtc);
              Dec_Ascii(Data.long_data/100,4,(uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES+i*18+13],0);
              
              if(i<5)
                RxTxBuffer[PROTO_DATA_BYTES+i*18+17]=RECORD_SEPERATOR;
            }
        break;
        //----------------------------------------------------------------------
        //----------------------------------------------------------------------
				case PROTO_READ_LATEST_NM_TAMPER   :
							setComFrameBytes(59);	
						 	getCustomTamperData((uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES],0,RxTxBuffer[PROTO_DATA_BYTES]);
							break;
				case PROTO_READ_LATEST_REV_TAMPER   :	
							setComFrameBytes(59);	
						 	getCustomTamperData((uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES],1,RxTxBuffer[PROTO_DATA_BYTES]);
							break;
				case PROTO_READ_LATEST_EARTH_TAMPER  :
							setComFrameBytes(59);	
						 	getCustomTamperData((uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES],2,RxTxBuffer[PROTO_DATA_BYTES]);
							break;
				case PROTO_READ_LATEST_CO_TAMPER  :
							setComFrameBytes(59);	
						 	getCustomTamperData((uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES],3,RxTxBuffer[PROTO_DATA_BYTES]);
							break;
				case PROTO_READ_LATEST_MAG_TAMPER   :
							setComFrameBytes(59);	
						 	getCustomTamperData((uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES],4,RxTxBuffer[PROTO_DATA_BYTES]);
							break;
				case PROTO_READ_LATEST_ND_TAMPER   :
							setComFrameBytes(59);	
						 	getCustomTamperData((uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES],5,RxTxBuffer[PROTO_DATA_BYTES]);
							break;
		
				case PROTO_READ_LATEST_OL_TAMPER  :
							setComFrameBytes(59);	
						 	getCustomTamperData((uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES],6,RxTxBuffer[PROTO_DATA_BYTES]);
							break;
				case PROTO_READ_LATEST_PF_TAMPER  :
							setComFrameBytes(59);	
						 	getCustomTamperData((uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES],7,RxTxBuffer[PROTO_DATA_BYTES]);
							break;
				case PROTO_READ_LATEST_FREQ_TAMPER  :
							setComFrameBytes(59);	
						 	getCustomTamperData((uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES],8,RxTxBuffer[PROTO_DATA_BYTES]);
							break;	
				case PROTO_READ_LATEST_OU_TAMPER   :
							setComFrameBytes(59);	
						 	getCustomTamperData((uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES],9,RxTxBuffer[PROTO_DATA_BYTES]);
							break;
				case PROTO_READ_LATEST_LU_TAMPER  :
							setComFrameBytes(59);	
							getCustomTamperData((uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES],10,RxTxBuffer[PROTO_DATA_BYTES]);
							break;

				// Tamper All			
				
				case PROTO_READ_LATEST_NM_TAMPER_FIVE   :
							setComFrameBytes(79);	
				 			getCustomTamperData_occures((uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES],0,RxTxBuffer[PROTO_DATA_BYTES]);
							break;
				
				case PROTO_READ_LATEST_REV_TAMPER_FIVE   :
							setComFrameBytes(79);	
				 			getCustomTamperData_occures((uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES],1,RxTxBuffer[PROTO_DATA_BYTES]);
							break;
							
				case PROTO_READ_LATEST_EARTH_TAMPER_FIVE  :
							setComFrameBytes(79);	
						 	getCustomTamperData_occures((uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES],2,RxTxBuffer[PROTO_DATA_BYTES]);
							break;
							
				case PROTO_READ_LATEST_CO_TAMPER_FIVE   :
							setComFrameBytes(79);	
						 	getCustomTamperData_occures((uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES],3,RxTxBuffer[PROTO_DATA_BYTES]);
							break;
							
				case PROTO_READ_LATEST_MAG_TAMPER_FIRST_FIVE  :
							setComFrameBytes(79);	
						 	getCustomTamperData_only20mag((uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES],4,0);
							break;
				
				case PROTO_READ_LATEST_MAG_TAMPER_SEC_FIVE    :
							setComFrameBytes(79);	
						 	getCustomTamperData_only20mag((uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES],4,5);
							break;
				
				case PROTO_READ_LATEST_MAG_TAMPER_THIRD_FIVE  :
							setComFrameBytes(79);	
						 	getCustomTamperData_only20mag((uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES],4,10);
							break;
				
				case PROTO_READ_LATEST_MAG_TAMPER_FOURTH_FIVE :
							setComFrameBytes(79);	
						 	getCustomTamperData_only20mag((uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES],4,15);
							break;
							
		case PROTO_READ_SW_LOG:
			loc=getCustomSWLog(&RxTxBuffer[PROTO_DATA_BYTES]);
			setComFrameBytes(loc);
		break;
		case PROTO_READ_FORMAT_DATA:
			loc=getFormatData(&RxTxBuffer[PROTO_DATA_BYTES]);
			if(received_uart==0)
			{
			#if (defined(UART_0_ENABLE) && (UART_0_ENABLE == 1))
				Meter_Response(RxTxBuffer+PROTO_DATA_BYTES,loc);
			#endif
			}
			#if (defined(UART_2_ENABLE) && (UART_2_ENABLE == 1))
			else
			{
				Meter2_Response(RxTxBuffer+PROTO_DATA_BYTES,loc);
			}
			#endif
		break;
		
		/*	case PROTO_READ_TWENTTY_MAG_TAMPER:
	           	setComFrameBytes(239);
	            for(i=0;i<20;i++)
	            {
	              read_eeprom((uint8_t *)&temp,GetTamperForwardLoc(i,commandId-0x67),4);
	              Data.long_data=ConvertTimeCounterToTime(temp,DATE_VAL,&rtc);
	              Dec_Ascii(Data.long_data,6,(uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES+i*12],0);
	              RxTxBuffer[PROTO_DATA_BYTES+i*12+6]=FIELD_SEPERATOR;
	              
	              Data.long_data=ConvertTimeCounterToTime(temp,TIME_VAL,&rtc);
	              Dec_Ascii(Data.long_data/100,4,(uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES+i*12+7],0);
	              
	              if(i<19)
	                RxTxBuffer[PROTO_DATA_BYTES+i*12+11]=RECORD_SEPERATOR;
	            }
						  break;*/
			
       //------------ Incase invalid command ----------------------------------
        default:
            RxTxBuffer[PROTO_DATA_BYTES] = 0x01;
			setComFrameBytes(1);
        break;
        //----------------------------------------------------------------------
    }
}
/*----------------------------------------------------------------------------*/


/***************** Response after executing command ***************************/
void AppMeter_Response(void)
{   
    uint16_t length;
    length=ConvertPAN((uint8_t*)&RxTxBuffer[PROTO_NO_BYTES])+4;
    addCheckSum(length);
    length=length+7;
    RxTxBuffer[length-2]=0x0D;
    RxTxBuffer[length-1]=0x0A;
	if(received_uart==0)
	{
	#if (defined(UART_0_ENABLE) && (UART_0_ENABLE == 1))
		Meter_Response(RxTxBuffer,length);
	#endif
	}
	#if (defined(UART_2_ENABLE) && (UART_2_ENABLE == 1))
	else
	{
		Meter2_Response(RxTxBuffer,length);
	}
	#endif
	
    
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
uint8_t Hex_BCD(uint8_t convert)
{   
    return (convert %10+((convert / 10)*16));
}
/*----------------------------------------------------------------------------*/


void addCheckSum(uint8_t length)
{
    uint8_t i,first_byte;
    uint16_t second_byte=0;
	
    for(i=0;i<length;i++)
      second_byte += RxTxBuffer[PROTO_NO_BYTES+i];
    
    
	first_byte=second_byte;
    first_byte = ~first_byte;   // 2's
    first_byte += 1;
	
    
    RxTxBuffer[length+1] = first_byte>>4;
    if(RxTxBuffer[length+1] >9)
        RxTxBuffer[length+1] += 0x37;
    else
        RxTxBuffer[length+1] += 0x30;
      
    
    
    RxTxBuffer[length+2] = first_byte & 0x0F;
    if(RxTxBuffer[length+2] >9)
        RxTxBuffer[length+2] += 0x37;
    else
        RxTxBuffer[length+2] += 0x30;

    first_byte = ~first_byte;
    
    
    RxTxBuffer[length+3] = first_byte>>4;
    if(RxTxBuffer[length+3] >9)
        RxTxBuffer[length+3] += 0x37;
    else
        RxTxBuffer[length+3] += 0x30;
    
    
    RxTxBuffer[length+4] = first_byte & 0x0F;
    if(RxTxBuffer[length+4] >9)
        RxTxBuffer[length+4] += 0x37;
    else
        RxTxBuffer[length+4] += 0x30;
  
}

uint8_t ValidateCheckSum(uint8_t length)
{
    uint8_t i,first_byte,ascii_0,ascii_1,ascii_2,ascii_3;
    uint16_t sum=0;
	
	length+=4;
    for(i=0;i<length;i++)
      sum += RxTxBuffer[PROTO_NO_BYTES+i];
    
	first_byte = sum;
    first_byte = ~first_byte;   // 2's
    first_byte += 1;
	
   
    
    ascii_0 = first_byte>>4;
    if(ascii_0 >9)
        ascii_0 += 0x37;
    else
        ascii_0 += 0x30;
      
    
    
    ascii_1 = first_byte & 0x0F;
    if(ascii_1 >9)
        ascii_1 += 0x37;
    else
        ascii_1 += 0x30;

    first_byte = ~first_byte;
    
    
    ascii_2 = first_byte>>4;
    if(ascii_2 >9)
        ascii_2 += 0x37;
    else
        ascii_2 += 0x30;
    
    
    ascii_3 = first_byte & 0x0F;
    if(ascii_3 >9)
        ascii_3 += 0x37;
    else
        ascii_3 += 0x30;
		
	if((ascii_0==RxTxBuffer[PROTO_NO_BYTES+length])&&(ascii_1==RxTxBuffer[PROTO_NO_BYTES+length+1])&&(ascii_2==RxTxBuffer[PROTO_NO_BYTES+length+2])&&(ascii_3==RxTxBuffer[PROTO_NO_BYTES+length+3]))
		return 1;
	else
		return 0;
}

uint8_t ConvertPAN(const uint8_t * bptr)
{
    uint8_t byteValue;
      if(bptr[1] >0x40)
        byteValue=bptr[1]- 0x37;
    else
        byteValue= bptr[1]-0x30;          
    
    if(bptr[0] >0x40)
        byteValue=  ((bptr[0]- 0x37))*16+byteValue;
    else
        byteValue=  (bptr[0]-0x30)*16+byteValue;  
    return byteValue;
}



uint8_t getCustomTODZonedata(uint8_t* bptr,uint8_t zone_no)
{
	Apprtc rtc;
	uint16_t loc;
	
	//kwh
	loc=0;
	if(zone_no==currentZoneID)
		Dec_Ascii(InsSave.TODEnergy.TOD[zone_no].kWh+InsSave.ZkWhCounter/METER_CONSTANT,8,bptr+loc,0);
	else
		Dec_Ascii(InsSave.TODEnergy.TOD[zone_no].kWh,8,bptr+loc,0);
	loc+=8;
	bptr[loc]=FIELD_SEPERATOR;
	loc+=1;
	
	// kVAh
	if(zone_no==currentZoneID)
		Dec_Ascii(InsSave.TODEnergy.TOD[zone_no].kVAh+InsSave.ZkVAhCounter/METER_CONSTANT,8,bptr+loc,0);
	else
		Dec_Ascii(InsSave.TODEnergy.TOD[zone_no].kVAh,8,bptr+loc,0);
	loc+=8;
	bptr[loc]=FIELD_SEPERATOR;
	loc+=1;
	
	// kW MD
	Dec_Ascii(InsSave.TODEnergy.TOD[zone_no].TODMD.kW,4,bptr+loc,0);
	loc+=4;
	bptr[loc]=FIELD_SEPERATOR;
	loc+=1;
	
	//kW MD Date
    Data.long_data=ConvertTimeCounterToTime(InsSave.TODEnergy.TOD[zone_no].TODMD.kW_Date,DATE_VAL,&rtc);
    Dec_Ascii(Data.long_data,6,bptr+loc,0);
	loc+=6;
	bptr[loc]=FIELD_SEPERATOR;
	loc+=1;
	//kW MD Time
  	Data.long_data=ConvertTimeCounterToTime(InsSave.TODEnergy.TOD[zone_no].TODMD.kW_Date,TIME_VAL,&rtc);
    Dec_Ascii(Data.long_data/100,4,bptr+loc,0);
	loc+=4;
	bptr[loc]=RECORD_SEPERATOR;
	loc+=1;
	//35
	return loc;
	
}

uint8_t getCustomHistorydata(uint8_t* bptr,uint8_t history_no)
{
	uint8_t buff[BILLING_DATA_LENGTH];
	Apprtc rtc;
	uint16_t loc;
	
	if(history_no==0)
	{
		loc=0;
		
		//billing Date
	    Data.long_data=ConvertTimeCounterToTime(InsSave.timeCounter,DATE_VAL,&rtc);
	    Dec_Ascii(Data.long_data,6,bptr+loc,0);
		loc+=6;
		bptr[loc]=FIELD_SEPERATOR;
		loc+=1;
		//billing Time
	  	Data.long_data=ConvertTimeCounterToTime(InsSave.timeCounter,TIME_VAL,&rtc);
	    Dec_Ascii(Data.long_data/100,4,bptr+loc,0);
		loc+=4;
		bptr[loc]=FIELD_SEPERATOR;
		loc+=1;
		
		
		Dec_Ascii(InsSave.CumkWh+InsSave.ZkWhCounter/METER_CONSTANT,8,bptr+loc,0);
		loc+=8;
		bptr[loc]=FIELD_SEPERATOR;
		loc+=1;
		// kVAh
		Dec_Ascii(InsSave.CumkVAh+InsSave.ZkVAhCounter/METER_CONSTANT,8,bptr+loc,0);
		loc+=8;
		bptr[loc]=FIELD_SEPERATOR;
		loc+=1;
		// kW MD
		Dec_Ascii(InsSave.BillMD.Current.kW,4,bptr+loc,0);
		loc+=4;
		bptr[loc]=FIELD_SEPERATOR;
		loc+=1;
		
		//kW MD Date
	    Data.long_data=ConvertTimeCounterToTime(InsSave.BillMD.Current.kW_Date,DATE_VAL,&rtc);
	    Dec_Ascii(Data.long_data,6,bptr+loc,0);
		loc+=6;
		bptr[loc]=FIELD_SEPERATOR;
		loc+=1;
		//kW MD Time
	  	Data.long_data=ConvertTimeCounterToTime(InsSave.BillMD.Current.kW_Date,TIME_VAL,&rtc);
	    Dec_Ascii(Data.long_data/100,4,bptr+loc,0);
		loc+=4;
		//46
		bptr[loc]=FIELD_SEPERATOR;
		loc+=1;
		Dec_Ascii((InsSave.MonthPowerOnDuration+InsSave.PowerOn30)/60,8,bptr+loc,0);
		loc+=8;
		bptr[loc]=FIELD_SEPERATOR;
		loc+=1;
		Dec_Ascii(getAvgPf(),3,bptr+loc,1);
		loc+=4;
	}
	else
	{
		loc=GetBillLoc(history_no-1);
		read_eeprom(buff,loc,BILLING_DATA_LENGTH);
		
		loc=0;
		//billing Date
	    Data.long_data=ConvertTimeCounterToTime(getByteValue(buff,25,4),DATE_VAL,&rtc);
	    Dec_Ascii(Data.long_data,6,bptr+loc,0);
		loc+=6;
		bptr[loc]=FIELD_SEPERATOR;
		loc+=1;
		//billing Time
	  	Data.long_data=ConvertTimeCounterToTime(getByteValue(buff,25,4),TIME_VAL,&rtc);
	    Dec_Ascii(Data.long_data/100,4,bptr+loc,0);
		loc+=4;
		bptr[loc]=FIELD_SEPERATOR;
		loc+=1;
		
		//kwh
		Dec_Ascii(getByteValue(buff,0,4),8,bptr+loc,0);
		loc+=8;
		bptr[loc]=FIELD_SEPERATOR;
		loc+=1;
		// kVAh
		Dec_Ascii(getByteValue(buff,4,4),8,bptr+loc,0);
		loc+=8;
		bptr[loc]=FIELD_SEPERATOR;
		loc+=1;
		// kW MD
		Dec_Ascii(getByteValue(buff,12,2),4,bptr+loc,0);
		loc+=4;
		bptr[loc]=FIELD_SEPERATOR;
		loc+=1;
		
		//kW MD Date
	    Data.long_data=ConvertTimeCounterToTime(getByteValue(buff,14,4),DATE_VAL,&rtc);
	    Dec_Ascii(Data.long_data,6,bptr+loc,0);
		loc+=6;
		bptr[loc]=FIELD_SEPERATOR;
		loc+=1;
		//kW MD Time
	  	Data.long_data=ConvertTimeCounterToTime(getByteValue(buff,14,4),TIME_VAL,&rtc);
	    Dec_Ascii(Data.long_data/100,4,bptr+loc,0);
		loc+=4;
		//46
		bptr[loc]=FIELD_SEPERATOR;
		loc+=1;
		Dec_Ascii(getByteValue(buff,8,4)/60,8,bptr+loc,0);
		loc+=8;
		bptr[loc]=FIELD_SEPERATOR;
		loc+=1;
		Dec_Ascii(getByteValue(buff,24,1),3,bptr+loc,1);
		loc+=4;		
	}
	return loc;	
}

uint8_t getCustomTODZoneHistorydata(uint8_t* bptr,uint8_t zone_no,uint8_t history_no)
{
	uint8_t buff[ZONE_LEN];
	Apprtc rtc;
	uint16_t loc=GetTODLatestBillLoc(history_no);
	loc=loc+zone_no*ZONE_LEN;
	read_eeprom(buff,loc,ZONE_LEN);
	//kwh
	loc=0;
	Dec_Ascii(getByteValue(buff,12,4),8,bptr+loc,0);
	loc+=8;
	bptr[loc]=FIELD_SEPERATOR;
	loc+=1;
	// kVAh
	Dec_Ascii(getByteValue(buff,16,4),8,bptr+loc,0);
	loc+=8;
	bptr[loc]=FIELD_SEPERATOR;
	loc+=1;
	// kW MD
	Dec_Ascii(getByteValue(buff,0,2),4,bptr+loc,0);
	loc+=4;
	bptr[loc]=FIELD_SEPERATOR;
	loc+=1;
	
	//kW MD Date
    Data.long_data=ConvertTimeCounterToTime(getByteValue(buff,4,4),DATE_VAL,&rtc);
    Dec_Ascii(Data.long_data,6,bptr+loc,0);
	loc+=6;
	bptr[loc]=FIELD_SEPERATOR;
	loc+=1;
	//kW MD Time
  	Data.long_data=ConvertTimeCounterToTime(getByteValue(buff,4,4),TIME_VAL,&rtc);
    Dec_Ascii(Data.long_data/100,4,bptr+loc,0);
	loc+=4;
	bptr[loc]=RECORD_SEPERATOR;
	loc+=1;
	//35
	return loc;
	
}

//getCustomTamperData((uint8_t *)&RxTxBuffer[PROTO_DATA_BYTES],commandId-PROTO_READ_LATEST_NM_TAMPER,RxTxBuffer[PROTO_DATA_BYTES]);

//#if 0
// this is the original function
uint8_t getCustomTamperData(uint8_t *bptr,uint8_t TamperNo,uint8_t eventno)
{
	uint8_t buff[TAMPER_DATA_LENGTH];// currently 25
	Apprtc rtc;
	int i;
	uint8_t loc_temp=0;

	for(i =0; i<5; i++)
	{
	uint16_t loc=GetTamperForwardLoc(i,TamperNo);
	read_eeprom(buff,loc,TAMPER_DATA_LENGTH);
	
	loc=0;
	//Tamper Date
    Data.long_data=ConvertTimeCounterToTime(getByteValue(buff,0,4),DATE_VAL,&rtc);
    Dec_Ascii(Data.long_data,6,bptr+loc_temp,0);
	  loc+=6; // 36 , now 6
		loc_temp+=6;
	  bptr[loc_temp]=FIELD_SEPERATOR;
	  loc+=1;	// 37	, now 7
		loc_temp+=1;
	//Tamper Time
  	Data.long_data=ConvertTimeCounterToTime(getByteValue(buff,0,4),TIME_VAL,&rtc);
    Dec_Ascii(Data.long_data/100,4,bptr+loc_temp,0);
	  loc+=4;	// now 13
		loc_temp+=4;
		bptr[loc_temp]=RECORD_SEPERATOR;
		loc+=1;
		loc_temp+=1;
	//43
	}
	return loc_temp;
}
//#endif

uint8_t getCustomTamperData_occures(uint8_t *bptr,uint8_t TamperNo,uint8_t eventno)
{
	uint8_t buff[TAMPER_DATA_LENGTH];// currently 25
	Apprtc rtc;
	int i;
	uint8_t loc_temp=0;

	for(i =0; i<5; i++)
	{
	uint16_t loc=GetTamperForwardLoc(i,TamperNo);
	read_eeprom(buff,loc,TAMPER_DATA_LENGTH);
	
	  loc=0;
	//Tamper Date
    Data.long_data=ConvertTimeCounterToTime(getByteValue(buff,0,4),DATE_VAL,&rtc);
    Dec_Ascii(Data.long_data,6,bptr+loc_temp,0);
	  loc+=6; // 36 , now 6
		loc_temp+=6; //6
	  bptr[loc_temp]=FIELD_SEPERATOR;
	  loc+=1;	// 37	, now 7
		loc_temp+=1; //7
	//Tamper Time
  	Data.long_data=ConvertTimeCounterToTime(getByteValue(buff,0,4),TIME_VAL,&rtc);
    Dec_Ascii(Data.long_data/100,4,bptr+loc_temp,0);
	  loc+=4;	
		loc_temp+=4; //11
		bptr[loc_temp]=FIELD_SEPERATOR;
	  loc+=1;	
		loc_temp+=1; //12
	// Event type	
		Dec_Ascii(getByteValue(buff,4,2),3,bptr+loc_temp,0);
	  loc+=3;
		loc_temp+=3; //15
		bptr[loc_temp]=RECORD_SEPERATOR;
		loc+=1;
		loc_temp+=1; //16

	}
	return loc_temp;
}

uint8_t getCustomTamperData_only20mag(uint8_t *bptr,uint8_t TamperNo,uint8_t eventno)
{
	uint8_t buff[TAMPER_DATA_LENGTH];// currently 25
	Apprtc rtc;
	int i;
	uint8_t loc_temp=0;
	uint8_t length=5;
	if(eventno==5)length=10;
	if(eventno==10)length=15;
	if(eventno==15)length=20;

	for(i =eventno; i<length; i++)
	{
	uint16_t loc=GetTamperForwardLoc(i,TamperNo);
	read_eeprom(buff,loc,TAMPER_DATA_LENGTH);
	
	  loc=0;
	//Tamper Date
    Data.long_data=ConvertTimeCounterToTime(getByteValue(buff,0,4),DATE_VAL,&rtc);
    Dec_Ascii(Data.long_data,6,bptr+loc_temp,0);
	  loc+=6; // 36 , now 6
		loc_temp+=6; //6
	  bptr[loc_temp]=FIELD_SEPERATOR;
	  loc+=1;	// 37	, now 7
		loc_temp+=1; //7
	//Tamper Time
  	Data.long_data=ConvertTimeCounterToTime(getByteValue(buff,0,4),TIME_VAL,&rtc);
    Dec_Ascii(Data.long_data/100,4,bptr+loc_temp,0);
	  loc+=4;	
		loc_temp+=4; //11
		bptr[loc_temp]=FIELD_SEPERATOR;
	  loc+=1;	
		loc_temp+=1; //12
	// Event type	
		Dec_Ascii(getByteValue(buff,4,2),3,bptr+loc_temp,0);
	  loc+=3;
		loc_temp+=3; //15
		bptr[loc_temp]=RECORD_SEPERATOR;
		loc+=1;
		loc_temp+=1; //16

	}
	return loc_temp;
}


#if 0
// this is the original function
uint8_t getCustomTamperData(uint8_t *bptr,uint8_t TamperNo,uint8_t eventno)
{


	uint8_t buff[TAMPER_DATA_LENGTH];
	Apprtc rtc;
	uint16_t loc=GetTamperForwardLoc(eventno,TamperNo);
	read_eeprom(buff,loc,TAMPER_DATA_LENGTH);
	
	loc=0;
	// event type
	Dec_Ascii(getByteValue(buff,4,2),3,bptr+loc,0);
	loc+=3;
	bptr[loc]=FIELD_SEPERATOR;
	loc+=1;

	//Voltage
	Dec_Ascii(getByteValue(buff,14,2)/10,4,bptr+loc,1);
	loc+=5;
    bptr[loc]=FIELD_SEPERATOR;
	loc+=1;

	//Current 
    Dec_Ascii(getByteValue(buff,6,4)/10,4,bptr+loc,2);
	loc+=5;
    bptr[loc]=FIELD_SEPERATOR;
	loc+=1;
	
	//power Factor
    Dec_Ascii(getByteValue(buff,16,1),3,bptr+loc,2);
	loc+=4;
    bptr[loc]=FIELD_SEPERATOR;
	loc+=1;

	//kwh
	Dec_Ascii(getByteValue(buff,17,4),8,bptr+loc,0);
	loc+=8;
	bptr[loc]=FIELD_SEPERATOR;
	loc+=1;
	
	//Tamper Date
    Data.long_data=ConvertTimeCounterToTime(getByteValue(buff,0,4),DATE_VAL,&rtc);
    Dec_Ascii(Data.long_data,6,bptr+loc,0);
	loc+=6;
	bptr[loc]=FIELD_SEPERATOR;
	loc+=1;
	//Tamper Time
  	Data.long_data=ConvertTimeCounterToTime(getByteValue(buff,0,4),TIME_VAL,&rtc);
    Dec_Ascii(Data.long_data,6,bptr+loc,0);
	loc+=6;
	//43
	return loc;
	
	
}

#endif


uint8_t getFormatData(uint8_t *bptr)
{
	
	Apprtc rtc;
	uint8_t no_of_bytes;
	getMeterSerial(bptr,SR_TYPE_ASCII);
	no_of_bytes=10;

	//Current Date
    Data.long_data=ConvertTimeCounterToTime(InsSave.timeCounter,DATE_VAL,&rtc);
    Dec_Ascii(Data.long_data,6,bptr+no_of_bytes,0);
	no_of_bytes+=6;
	//Current Time
  	Data.long_data=ConvertTimeCounterToTime(InsSave.timeCounter,TIME_VAL,&rtc);
    Dec_Ascii(Data.long_data/100,4,bptr+no_of_bytes,0);
	no_of_bytes+=4;
	
	//Make
	bptr[no_of_bytes++]='E';
	bptr[no_of_bytes++]='E';
	bptr[no_of_bytes++]='P';
	bptr[no_of_bytes++]='L';
	
	//KNO
	//Data.long_data=1234567890;
	getKNO(bptr+no_of_bytes);
	//Dec_Ascii(Data.long_data,10,bptr+no_of_bytes,0);
	no_of_bytes+=10;
	
	//kWh
	Data.long_data=InsSave.CumkWh+InsSave.ZkWhCounter/METER_CONSTANT;
	Data.long_data=Data.long_data/100;
	Dec_Ascii(Data.long_data,5,bptr+no_of_bytes,0);	
	no_of_bytes+=5;
	//kVAh
	Data.long_data=InsSave.CumkVAh+InsSave.ZkVAhCounter/METER_CONSTANT;
	Data.long_data=Data.long_data/100;
	Dec_Ascii(Data.long_data,5,bptr+no_of_bytes,0);	
	no_of_bytes+=5;	
	
	//kW MD
	Dec_Ascii(InsSave.BillMD.Current.kW/10,3,bptr+no_of_bytes,1);	
	no_of_bytes+=4;	
	
	//Power off Count
	Dec_Ascii(InsSave.PFailCount,4,bptr+no_of_bytes,0);	
	no_of_bytes+=4;	

	//Voltage
	Dec_Ascii(Ins.Voltage/100,3,bptr+no_of_bytes,0);	
	no_of_bytes+=3;	
	
	//Current
	Dec_Ascii(Ins.EffectiveI/100,3,bptr+no_of_bytes,1);	
	no_of_bytes+=4;	
	
	//Power factor
	Dec_Ascii(Ins.PowerFactor,3,bptr+no_of_bytes,2);	
	no_of_bytes+=4;	
	//rev
	no_of_bytes+=getTamperCustomdata(3,bptr+no_of_bytes);
	//nm
	no_of_bytes+=getTamperCustomdata(0,bptr+no_of_bytes);
	//mag
	no_of_bytes+=getTamperCustomdata(2,bptr+no_of_bytes);
	
	//high spark
	no_of_bytes+=getTamperCustomdata(1,bptr+no_of_bytes);
	//nd
	no_of_bytes+=getTamperCustomdata(4,bptr+no_of_bytes);
	
	bptr[no_of_bytes++]='E';
	return no_of_bytes;
	
}

uint8_t getTamperCustomdata(uint8_t TamperNo,uint8_t * bptr)
{
	uint16_t loc;
	Apprtc rtc;
	uint8_t no_of_bytes=0,isRes=0;
	uint8_t buff[TAMPER_DATA_LENGTH*2];
	
	buff[4]=0x00;
	buff[5]=0x00;

	buff[TAMPER_DATA_LENGTH+4]=0x00;
	buff[TAMPER_DATA_LENGTH+5]=0x00;
	
	loc=GetTamperForwardLoc(0,TamperNo);
	read_eeprom(buff,loc,TAMPER_DATA_LENGTH);

	loc=GetTamperForwardLoc(1,TamperNo);
	read_eeprom(buff+TAMPER_DATA_LENGTH,loc,TAMPER_DATA_LENGTH);
	
	isRes=0;
	if(dlmsEventCode[TamperNo][0]==getByteValue(buff,4,2))
		isRes=0;
	else if(dlmsEventCode[TamperNo][1]==getByteValue(buff,4,2))
		isRes=1;
	
	//Occ
	//Tamper Date
    Data.long_data=ConvertTimeCounterToTime(getByteValue(buff,TAMPER_DATA_LENGTH*isRes,4),DATE_VAL,&rtc);
    Dec_Ascii(Data.long_data,6,bptr+no_of_bytes,0);
	no_of_bytes+=6;
	//Tamper Time
  	Data.long_data=ConvertTimeCounterToTime(getByteValue(buff,TAMPER_DATA_LENGTH*isRes,4),TIME_VAL,&rtc);
    Dec_Ascii(Data.long_data/100,4,bptr+no_of_bytes,0);
	no_of_bytes+=4;	
	

	if(isRes==0)
		isRes=1;
	else
		isRes=0;
	//Rec
	//Tamper Date
    Data.long_data=ConvertTimeCounterToTime(getByteValue(buff,TAMPER_DATA_LENGTH*isRes,4),DATE_VAL,&rtc);
    Dec_Ascii(Data.long_data,6,bptr+no_of_bytes,0);
	no_of_bytes+=6;
	//Tamper Time
  	Data.long_data=ConvertTimeCounterToTime(getByteValue(buff,TAMPER_DATA_LENGTH*isRes,4),TIME_VAL,&rtc);
    Dec_Ascii(Data.long_data/100,4,bptr+no_of_bytes,0);
	no_of_bytes+=4;	
	
    Dec_Ascii(TamperRecord_AP.TamperCount[TamperNo],4,bptr+no_of_bytes,0);
	no_of_bytes+=4;	
	return no_of_bytes;
	
	
}
void setKNO(const uint8_t * bptr)
{
	write_eeprom(bptr,KNO_LOC,10);
}
uint8_t validateBillDay(uint8_t billday)
{
	if((billday<1)||(billday>31))
		billday=24;
	return billday;
}
void setBillingDate(const uint8_t * bptr)
{
	uint8_t billday;
	billday=ConvertPAN(bptr);
	validateBillDay(billday);
	write_eeprom((uint8_t*)&billday,PRE_BILL_DATE,1);
	
}
uint16_t getBillingDate(void)
{
	uint8_t billday;
	read_eeprom((uint8_t*)&billday,PRE_BILL_DATE,1);
	billday=validateBillDay(billday);
	
	return makePANByte(billday);
}
void getKNO(uint8_t * bptr)
{
	read_eeprom(bptr,KNO_LOC,10);
}
uint16_t makePANByte(uint8_t byteValue)
{
	uint8_t nibL=byteValue&0x0F;
	uint8_t nibU=byteValue>>4;
	uint16_t panbyte;
	
	if(nibU>9)
		panbyte=nibU+0x37;
	else
		panbyte=nibU+0x30;
	panbyte<<=8;
		
	if(nibL>9)
		panbyte|=nibL+0x37;
	else
		panbyte|=nibL+0x30;
		
	return panbyte;
}
void setComFrameBytes(uint16_t bytes)
{
	
	bytes=makePANByte(bytes);
	RxTxBuffer[PROTO_NO_BYTES]=bytes>>8;
	RxTxBuffer[PROTO_NO_BYTES+1]=bytes;
}

void getCustomTODTiming(uint8_t* bptr)
{
	uint8_t todBuffR[26];
	uint8_t byteno=0;
	uint16_t panno;
	
	read_eeprom(todBuffR,ZONE_TIME,25);
	while(byteno<25)
	{
		panno=makePANByte(*(todBuffR+byteno));
		*bptr++=panno>>8;
		*bptr++=panno;
		byteno++;
	}
}
uint8_t setCustomTODTiming(const uint8_t* bptr)
{
	uint8_t todBuffR[30];
	uint8_t byteno=0;
	
	while(byteno<29)
	{
		todBuffR[byteno]=ConvertPAN(bptr);
		byteno++;
		bptr+=2;
	}
	
	if(Crc_Cal(todBuffR,27)==0)
	{
		write_eeprom(todBuffR,PASSIVE_ZONE_TIME,25);
		R_USER_Class20_SETACTION_Time(InsSave.timeCounter,0);
		return 1;
	}
	return 0;
}
uint8_t getCustomSWLog(uint8_t* bptr)
{
	uint8_t bytes=0,no_of_itr=SW_EVENTS;
	uint8_t buff[6];
	
	while(no_of_itr>0)
	{
		getSWLog(buff,no_of_itr--);
		bytes+=getCustomDateTimeASCIIFormat(getByteValue(buff,0,4),bptr+bytes,6);
	    bptr[bytes]=FIELD_SEPERATOR;
		bytes+=1;
		
	    Dec_Ascii(getByteValue(buff,4,1),3,bptr+bytes,0);
		bytes=bytes+3;
		if(no_of_itr>0)
		{
		    bptr[bytes]=FIELD_SEPERATOR;
			bytes+=1;
		}
	}
	
	return bytes;
	
}
#endif
