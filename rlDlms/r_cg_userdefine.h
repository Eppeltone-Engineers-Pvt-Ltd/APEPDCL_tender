/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only 
* intended for use with Renesas products. No other uses are authorized. This 
* software is owned by Renesas Electronics Corporation and is protected under 
* all applicable laws, including copyright laws.
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING 
* THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT 
* LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE 
* AND NON-INFRINGEMENT.  ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.
* TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS 
* ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE 
* FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR 
* ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE 
* BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software 
* and to discontinue the availability of this software.  By using this software, 
* you agree to the additional terms and conditions found by accessing the 
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2012 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/

/***********************************************************************************************************************
* File Name    : r_cg_userdefine.h
* Version      : CodeGenerator for RL78/L12 E1.00.00c [23 Mar 2012]
* Device(s)    : R5F10RLC
* Tool-Chain   : CA78K0R
* Description  : This file includes user definition.
* Creation Date: 11/1/2012
***********************************************************************************************************************/

#ifndef _USER_DEF_H
#define _USER_DEF_H

/***********************************************************************************************************************
User definitions
***********************************************************************************************************************/

/* Start user code for function. Do not edit comment generated here */

#define SCALAR_NULL 0
#define SCALAR_VOLTAGE -2
#define SCALAR_AMPERE -3
#define SCALAR_PF -2
#define SCALAR_APF -2
#define SCALAR_HERTZ -2
#define SCALAR_VA -2
#define SCALAR_WATT -2
#define SCALAR_MD 1
#define SCALAR_WH 1
#define SCALAR_VAH 1
#define SCALAR_CON -1
#define SCALAR_HWH -1

typedef struct
{
   unsigned int Year;
   unsigned char Month;
   unsigned char Date;
   unsigned char Day;
   unsigned char Hr;
   unsigned char Min;  
} sSA_Range;

/* End user code. Do not edit comment generated here */
#endif

