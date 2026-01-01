/******************************************************************************
* DISCLAIMER
* Please refer to http://www.renesas.com/disclaimer
******************************************************************************
  Copyright (C) 2008. Renesas Technology Corp., All Rights Reserved.
*******************************************************************************
* File Name    : serial.h
* Version      : 1.00
* Description  : Contain type, macro definition and function header of the serial 
*				 API.
******************************************************************************
* History : DD.MM.YYYY Version Description
*         : 17/12/2009  1.00   First Release
******************************************************************************/

#ifndef _DLMS_SERIAL_H
#define _DLMS_SERIAL_H

/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#include	"..\\r_dlms_typedef.h"
#include	"..\\DLMS_User.h"
/******************************************************************************
Typedef definitions
******************************************************************************/

/******************************************************************************
Macro definitions
******************************************************************************/

/******************************************************************************
Variable Externs
******************************************************************************/

/******************************************************************************
Functions Prototypes
******************************************************************************/
void InitSerial(Unsigned8 channel);

void SerialTxEnd(Unsigned8 channel);
void SerialRxEnd(Unsigned8 channel, Unsigned8 byte);

void SerialTxBlock(Unsigned8 channel, Unsigned8* BlockPtr, Integer16 Length);

void SerialConfig(Unsigned8 channel, Unsigned8 new_baud_rate, Unsigned8 new_protocol);

#endif /* _DLMS_SERIAL_H */
