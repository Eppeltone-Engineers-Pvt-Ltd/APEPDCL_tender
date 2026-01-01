/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products.
* No other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
* applicable laws, including copyright laws. 
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIESREGARDING THIS SOFTWARE, WHETHER EXPRESS, IMPLIED
* OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NON-INFRINGEMENT.  ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY
* LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE FOR ANY DIRECT,
* INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR
* ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability 
* of this software. By using this software, you agree to the additional terms and conditions found by accessing the 
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2015, 2016 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/

/***********************************************************************************************************************
* File Name    : r_cg_systeminit.c
* Version      : Applilet4 for RL78/I1C V1.00.01.04 [05 Aug 2016]
* Device(s)    : R5F10NMJ
* Tool-Chain   : CA78K0R
* Description  : This file implements system initializing function.
* Creation Date: 11/8/2016
***********************************************************************************************************************/

/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
/* Start user code for pragma. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "..\\include\\dTypedef.h"
#include "..\\Include\\dUser_includes.h"
#include <stddef.h>

#define NUMBER_OF_ISR           (128)
typedef void (__near * INT_HANDLER)(void);
/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
#ifdef __CCRL__
#define SIZEOF_HEAP  0x800
char _REL_sysheap[SIZEOF_HEAP];
size_t _REL_sizeof_sysheap = SIZEOF_HEAP;
const uint16_t g_cnst_REL_sizeof_sysheap = SIZEOF_HEAP;

#pragma section bss RAMVect
INT_HANDLER RAM_ISR_Table[NUMBER_OF_ISR];

#elif defined __CA78K0R__
#if defined(__F10NMJ_) || defined(__F10NPJ_)
#define DEVICE_RAM_START_ADDR   (0xBF00)
#elif defined(__F10NLG_) || defined(__F10NMG_) || defined(__F10NPG_)
#define DEVICE_RAM_START_ADDR   (0xDF00)
#elif defined(__F10NLE_) || defined(__F10NME_)
#define DEVICE_RAM_START_ADDR   (0xE700)
#else
#error "Unsupported MCU"
#endif

NEAR_PTR const uint16_t g_device_ram_addr_start = DEVICE_RAM_START_ADDR;

INT_HANDLER RAM_ISR_Table[NUMBER_OF_ISR];

#else /* Other compilers */
	INT_HANDLER RAM_ISR_Table[NUMBER_OF_ISR];

#endif /* __CCRL__ */

/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: R_Systeminit
* Description  : This function initializes every macro.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_Systeminit(void)
{
}
/***********************************************************************************************************************
* Function Name: R_RAM_InterruptVectorInit
* Description  : This function initializes innterrupt RAM vector table
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_RAM_InterruptVectorInit(void)
{
    
}

/***********************************************************************************************************************
* Function Name: hdwinit
* Description  : This function initializes hardware setting.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
#if defined(__CA78K0R__) || defined(__CCRL__)
void hdwinit(void)
{
    DI();
    R_Systeminit();
    R_RAM_InterruptVectorInit();
    //EI();
}
#endif /* __CA78K0R__ ||  __CCRL__ */


/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
