/******************************************************************************
  Copyright (C) 2011 Renesas Electronics Corporation, All Rights Reserved.
*******************************************************************************
* File Name    : wrp_user_ext.h
* Version      : 1.00
* Description  : Wrapper User UART
******************************************************************************
* History : DD.MM.YYYY Version Description
******************************************************************************/

#ifndef _WRAPPER_USER_EXT_H
#define _WRAPPER_USER_EXT_H

/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#include "typedef.h"
//#include "em_maths.h"

/******************************************************************************
Typedef definitions
******************************************************************************/
typedef uint8_t(*fp_predicate)(void);

/******************************************************************************
Macro definitions
******************************************************************************/
#define WRP_EXT_Absf(x)                      EM_absf(x)   

#define WRP_EXT_HW_DELAY_TIMEOUT             0
#define WRP_EXT_HW_DELAY_PRED_TRUE           1
#define WRP_EXT_HW_DELAY_INVALID_ARGS        2
#define WRP_EXT_HW_DELAY_TIMER_STOPPED       3

/******************************************************************************
Variable Externs
******************************************************************************/

/******************************************************************************
Functions Prototypes
******************************************************************************/
void WRP_EXT_SoftReset(void);
uint8_t WRP_EXT_Bcd2Dec(uint8_t x);
uint8_t WRP_EXT_Dec2Bcd(uint8_t x);

uint8_t WRP_EXT_HwDelay(uint16_t ms, fp_predicate p);
void WRP_EXT_HwDelayMillisecondTimerCallback(void);
#endif /* _WRAPPER_USER_EXT_H */
