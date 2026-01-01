/******************************************************************************
* DISCLAIMER

* This software is supplied by Renesas Technology Corp. and is only 
* intended for use with Renesas products. No other uses are authorized.

* This software is owned by Renesas Technology Corp. and is protected under 
* all applicable laws, including copyright laws.

* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES
* REGARDING THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, 
* INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
* PARTICULAR PURPOSE AND NON-INFRINGEMENT.  ALL SUCH WARRANTIES ARE EXPRESSLY 
* DISCLAIMED.

* TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS 
* TECHNOLOGY CORP. NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE 
* FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES 
* FOR ANY REASON RELATED TO THE THIS SOFTWARE, EVEN IF RENESAS OR ITS 
* AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.

* Renesas reserves the right, without notice, to make changes to this 
* software and to discontinue the availability of this software.  
* By using this software, you agree to the additional terms and 
* conditions found by accessing the following link:
* http://www.renesas.com/disclaimer
******************************************************************************
* Copyright (C) 2008. Renesas Technology Corp., All Rights Reserved.
*******************************************************************************	
* File Name    : 
* Version      : 1.00
* Description  : 
******************************************************************************
* History : DD.MM.YYYY Version Description
*         : 12/02/2009 1.00    First Release
******************************************************************************/

/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/


#include	"serial.h"

#include	<string.h>


//---------------Device specific-------------
#include "..\\..\\rlDevice\\include\\dUart.h"

#include "..\\..\\rlApplication\\Include\\AppConfig.h"


/******************************************************************************
Typedef definitions
******************************************************************************/

/******************************************************************************
Macro definitions
******************************************************************************/

/******************************************************************************
Imported global variables and functions (from other files)
******************************************************************************/

/******************************************************************************
Exported global variables and functions (to be accessed by other files)
******************************************************************************/


#ifdef __DEBUG_DLMS
	Unsigned8 g_receive_data[256];
	#endif
Unsigned8 g_index = 0;


/******************************************************************************
Private global variables and functions
******************************************************************************/

/******************************************************************************
* Function Name: InitSerial
* Description  : Initialization of UART unit to enable 
*				 serial receive/transmit operations
* Arguments    : None
* Return Value : None
******************************************************************************/
void InitSerial(Unsigned8 channel)
{
//	#if (defined(IS_DLMS_ENABLED_EEPL) && IS_DLMS_ENABLED_EEPL == 1)
	switch (channel)
	{
		case CHANNEL_PRIMARY:
			/* Start UART */
			init_Uart0();
			break;
		case CHANNEL_SECONDARY:
			break;
		case CHANNEL_3RD:
			break;
		default:
			/* Do nothing */
			break;
	}
//	#endif
	#ifdef __DEBUG_DLMS
	memset(g_receive_data, 0, 256);
	#endif
}



/****************************************************************************
* Function Name: SerialTxBlock
* Description  : Transmit block of data through serial communication
* Arguments    : Unsigned8 channel: channel ID 
*              : Unsigned8*  BlockPtr: Pointer to block start address 
*			   : Integer16   Length	 : Length of the data 
* Return Value : none
******************************************************************************/
void SerialTxBlock(Unsigned8 channel, Unsigned8* BlockPtr, Integer16 Length)
{
//#if (defined(IS_DLMS_ENABLED_EEPL) && IS_DLMS_ENABLED_EEPL == 1)	
	/* Start serial transmit */
	switch (channel)
	{
		case CHANNEL_PRIMARY:
//			#if (defined(UART_0_ENABLE) && (UART_0_ENABLE == 1))
			Meter_Response(BlockPtr,Length);
//			#endif
			break;
		case CHANNEL_SECONDARY:
//			#if (defined(UART_2_ENABLE) && (UART_2_ENABLE == 1))
			 Meter2_Response(BlockPtr,Length);;  //RENESAS, TO BE MODIFY
//			 #endif
			break;
		case CHANNEL_3RD:
			//WRP_UART1_SendData(BlockPtr, Length);  //RENESAS, TO BE MODIFY
			break;
		default:
			/* Do nothing */
			break;
	}
	PhysicalSendEndCallback(channel);
//#endif
}

/*****************************************************************************************
* function		: SerialConfig(Unsigned8 channel, Unsigned8 new_baud_rate, Unsigned8 new_protocol)
* description	: Reconfigure UART to adapt with new baud_rate,new protocol
* input			: Unsigned8 channel
* output		: none
* return 		: none		  
*****************************************************************************************/
void SerialConfig(Unsigned8 channel, Unsigned8 new_baud_rate, Unsigned8 new_protocol)
{

}
