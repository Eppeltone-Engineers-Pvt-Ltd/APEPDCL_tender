/******************************************************************************
* DISCLAIMER

* This software is supplied by Renesas Electronics Corporation and is only
* intended for use with Renesas products. No other uses are authorized.

* This software is owned by Renesas Electronics Corporation and is protected under
* all applicable laws, including copyright laws.

* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES
* REGARDING THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY,
* INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
* PARTICULAR PURPOSE AND NON-INFRINGEMENT.  ALL SUCH WARRANTIES ARE EXPRESSLY
* DISCLAIMED.

* TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS
* ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE
* FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES
* FOR ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR ITS
* AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.

* Renesas reserves the right, without notice, to make changes to this
* software and to discontinue the availability of this software.
* By using this software, you agree to the additional terms and
* conditions found by accessing the following link:
* http://www.renesas.com/disclaimer
******************************************************************************/
/* Copyright (C) 2011 Renesas Electronics Corporation. All rights reserved.  */
/******************************************************************************
* File Name    : DLMS_User.c
* Version      : 1.00
* Device(s)    : None
* Tool-Chain   :
* H/W Platform : Unified Energy Meter Platform
* Description  :
******************************************************************************
* History : DD.MM.YYYY Version Description
*         : 10.07.2012
******************************************************************************/

/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#include	"Physical\\Timer.h"
#include	"Objects\\r_dlms_obis.h"
#include	"DLMS_User.h"

//----------------dlms specific files---------------------------------
#include "meter_app\\r_dlms_data_meter.h"				/* DLMS Data Definitions */



/******************************************************************************
Macro definitions
******************************************************************************/

/******************************************************************************
Typedef definitions
******************************************************************************/

/******************************************************************************
Imported global variables and functions (from other files)
******************************************************************************/

/******************************************************************************
Exported global variables and functions (to be accessed by other files)
******************************************************************************/

/******************************************************************************
Private global variables and functions
******************************************************************************/
Unsigned8	RxBufferMain[MAX_RECIEVE_BUFFER_SIZE];
Unsigned8	RxBufferSub[MAX_RECIEVE_BUFFER_SIZE];
Unsigned8	TxBuffer[MAX_TRANSMIT_BUFFER_SIZE];
Unsigned8	ServerBuffer[MAX_SERVER_BUFFER_SIZE];

st_channel_info	channel_info[MAX_CONNMGR_CHANNEL_NUMBER];
const Unsigned8	ServerPhysicalAddress[] = {0x00, 0x01, 0x02, 0x00};
const Unsigned8	DeviceAddress[] = "RES001";			/* max 32 character  */
const Unsigned8	ManufacturerID[] = "REN";			/* 3 character  */
const Unsigned8	ID[] = "EEO";						/* max 16 character  */

void R_OBIS_Class07_BlockloadUpdate(void);

/* Config parameter for DLMS library here */
const st_ServerConfig UserServerConfig = 
{
	channel_info,									/* Channel info           */
	InitSerial,										/* SerialInit_FuncPtr     */
	SerialConfig,									/* SerialConfig_FuncPtr   */
	SerialTxBlock,									/* SerialTx_FuncPtr       */
	ServerBuffer,									/* *Server_Buffer         */
	(Unsigned8 *)ServerPhysicalAddress,				/* *ServerPhysicalAddress */
	(Unsigned8 *)DeviceAddress,						/* *DeviceAddress         */
	(Unsigned8 *)ManufacturerID,					/* *ManufacturerID        */
	(Unsigned8 *)ID,								/* *ID                    */
	MAX_SERVER_BUFFER_SIZE,							/* Server_BufferSize      */
	5000,											/* Response_Timeout       */
	20000,											/* Inactivity_Timeout     */
	500,											/* Interframe_Timeout     */
	sizeof(channel_info)/ sizeof(st_channel_info),	/* Channel max count      */
	PHYADD_1BYTE_SUPPORTED,							/* AdressByteMode         */
	IEC_BAUDRATE,									/* IEC_BaudRate           */
	HDLC_BAUDRATE,									/* HDLC_BaudRate          */
	MAX_WINDOWS_SIZE,								/* Windows size           */
	0,												/* Dummy                  */
};

/******************************************************************************
* Function Name   : DLMS_Initialize
* Interface       : void DLMS_Initialize(void)
* Description     :
* Arguments       : None:
* Function Calls  :
* Return Value    : None
******************************************************************************/
void DLMS_Initialize(void)
{

	/* Initialize the stack library */
	DLMSInit(UserServerConfig);

	/* Add 1st channel */
	CONNMGR_AddChannel(
		CHANNEL_PRIMARY,				/* Channel ID      */
		TxBuffer,						/* Tx buffer start */
		MAX_TRANSMIT_BUFFER_SIZE,		/* Tx buffer size  */
		RxBufferMain,					/* Rx buffer start */
		MAX_RECIEVE_BUFFER_SIZE			/* Rx buffer size  */
	);

	if (CONNMGR_MaxChannelNumber() >= 2)
	{
		/* Add 2nd channel */
		CONNMGR_AddChannel(
			CHANNEL_SECONDARY,			/* Channel ID      */
			TxBuffer,					/* Tx buffer start */
			MAX_TRANSMIT_BUFFER_SIZE,	/* Tx buffer size  */
			RxBufferSub,				/* Rx buffer start */
			MAX_RECIEVE_BUFFER_SIZE		/* Rx buffer size  */
		);
	}
	/* Set priority channel */
	CONNMGR_RegisterPriorityChannel(CHANNEL_PRIMARY);
	//CONNMGR_RegisterPriorityChannel(CHANNEL_SECONDARY);

	/* Initialize object layer */
	COSEMObjectLayerInit(	g_ServiceBuffer,
							OBIS_SERVICE_DATA_BUFFER_LENGTH,
							R_OBIS_DecodeObject);

	R_OBIS_ObjectLayerInit();
	R_OBIS_Class07_BlockloadUpdate();
	R_OBIS_Class07_EventUpdate();
	R_OBIS_Class07_BillingUpdate();
	
}


/******************************************************************************
* Function Name : DataLinkConnectionReset
* Interface     : void DataLinkConnectionReset(void)
* Description   :
* Arguments     : None:
* Function Calls:
* Return Value  : None
******************************************************************************/
void DataLinkConnectionReset(void)
{
	/* Reset the async service */
	R_OBIS_AsyncRestart();			/* Reset on Distributor function */
}

/******************************************************************************
* Function Name : DLMSIntervalProcessing
* Interface     : void DLMSIntervalProcessing(void)
* Description   :
* Arguments     : None:
* Function Calls:
* Return Value  : None
******************************************************************************/

/******************************************************************************
* Function Name   : DLMS_PollingProcessing
* Interface       : void DLMS_PollingProcessing(void)
* Description     : DLMS polling processing
* Arguments       : None:
* Function Calls  :
* Return Value    : None
******************************************************************************/
void DLMS_PollingProcessing(void)
{
	/* DLMS process of request and response message */
	DLMSMessageProcess();

}