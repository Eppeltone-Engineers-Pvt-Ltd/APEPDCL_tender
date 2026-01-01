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
* Copyright (C) 2013, 2015 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/

/***********************************************************************************************************************
* File Name    : 
* Version      : 1.00
* Device(s)    : RL78/I1C
* Tool-Chain   : CCRL
* H/W Platform : RL78/I1C Energy Meter Platform
* Description  : ADC DC Correction for RL78/I1C Platform
***********************************************************************************************************************/
/*****************************************************************************
* History : DD.MM.YYYY Version Description
*         : 02.08.2017 1.0     First support for RL78/I1C 3P4W meter
******************************************************************************/

#ifndef _ADC_DC_CORRECTION_H
#define _ADC_DC_CORRECTION_H

/***********************************************************************************************************************
Includes   <System Includes> , "Project Includes"
***********************************************************************************************************************/
#include "..\\include\\dtypedef.h"                /* GSCE Standard Typedef */

/******************************************************************************
Macro definitions
******************************************************************************/

/******************************************************************************
Typedef definitions
******************************************************************************/

typedef struct tag_dc_imunity_config_t
{
    uint16_t peak_detect_cycle;
    uint16_t dc_continous_count;
    uint16_t max_dc_count;
    uint16_t dc_operation_max_sample_count;
    uint16_t dc_operation_min_sample_count;
} dc_imunity_config_t;

/******************************************************************************
Variable Externs
******************************************************************************/
extern const dc_imunity_config_t g_dc_imunity_config_default_4k_50Hz;
extern const dc_imunity_config_t g_dc_imunity_config_default_2k_50Hz;

/******************************************************************************
Functions Prototypes
******************************************************************************/

void R_DC_Imunity_Init(dc_imunity_config_t *p_config);

void R_DC_Imunity_Reset_Phase_1(void);
void R_DC_Imunity_Reset_Phase_2(void);
void R_DC_Imunity_Reset_Phase_3(void);
void R_DC_Imunity_Reset_Phase_4(void);
void R_DC_Imunity_Reset_Phase_5(void);

uint8_t R_DC_Imunity_Calc_Phase_1(int32_t sample);
uint8_t R_DC_Imunity_Calc_Phase_2(int32_t sample);
uint8_t R_DC_Imunity_Calc_Phase_3(int32_t sample);
uint8_t R_DC_Imunity_Calc_Phase_4(int32_t sample);
uint8_t R_DC_Imunity_Calc_Phase_5(int32_t sample);

//void R_DC_Imunity_Apply_Phase_1(int32_t *p_sample);
//void R_DC_Imunity_Apply_Phase_2(int32_t *p_sample);
//void R_DC_Imunity_Apply_Phase_3(int32_t *p_sample);
//void R_DC_Imunity_Apply_Phase_4(int32_t *p_sample);
//void R_DC_Imunity_Apply_Phase_5(int32_t *p_sample);

#endif /* _ADC_DC_CORRECTION_H */

