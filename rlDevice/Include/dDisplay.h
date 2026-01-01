/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products.
* No other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
* applicable laws, including copyright laws. 
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING THIS SOFTWARE, WHETHER EXPRESS, IMPLIED
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
* Copyright (C) 2015, 2020 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/

/***********************************************************************************************************************
* File Name    : r_cg_lcd.h
* Version      : Code Generator for RL78/I1C V1.01.05.01 [28 Aug 2020]
* Device(s)    : R5F11TLG
* Tool-Chain   : CCRL
* Description  : This file implements device driver for LCD module.
* Creation Date: 31-12-2020
***********************************************************************************************************************/
#ifndef __DEV_LCD_H
#define __DEV_LCD_H

/***********************************************************************************************************************
Macro definitions (Register bit)
***********************************************************************************************************************/
/*
    LCD mode register 0 (LCDM0) 
*/
/* LCD drive voltage generator selection (MDSET1,MDSET0) */
#define _00_LCD_VOLTAGE_MODE_EXTERNAL      (0x00U) /* external resistance division method */
#define _40_LCD_VOLTAGE_MODE_INTERNAL      (0x40U) /* internal voltage boosting method */
#define _80_LCD_VOLTAGE_MODE_CAPACITOR     (0x80U) /* capacitor split method */
#define _C0_LCD_VOLTAGE_MODE_INITIALVALUE  (0xC0U) 
/* LCD display waveform selection (LWAVE) */
#define _00_LCD_DISPLAY_WAVEFORM_A         (0x00U) /* A waveform */
#define _20_LCD_DISPLAY_WAVEFORM_B         (0x20U) /* B waveform */
/* LCD display time slice selection (LDTY2,LDTY1,LDTY0) */
#define _00_LCD_TIMESLICE_STATIC           (0x00U) /* static */
#define _04_LCD_TIMESLICE_2                (0x04U) /* 2-time slice */
#define _08_LCD_TIMESLICE_3                (0x08U) /* 3-time slice */
#define _0C_LCD_TIMESLICE_4                (0x0CU) /* 4-time slice */
#define _10_LCD_TIMESLICE_6                (0x10U) /* 6-time slice */
#define _14_LCD_TIMESLICE_8                (0x14U) /* 8-time slice */
/* LCD display bias mode selection (LBAS1,LBAS0) */
#define _00_LCD_BIAS_MODE_2                (0x00U) /* 1/2 bias method */
#define _01_LCD_BIAS_MODE_3                (0x01U) /* 1/3 bias method */
#define _02_LCD_BIAS_MODE_4                (0x02U) /* 1/4 bias method */

/*
    LCD mode register 1 (LCDM1) 
*/
/* Common and segment pins output select and deselect signals in accordance with display data (LCDON) */
#define _00_LCD_DISPLAY_OFF                (0x00U) /* display off (all segment outputs are deselected) */
#define _80_LCD_DISPLAY_ON                 (0x80U) /* display on */
/* Common pin outputs select signal and segment pin outputs deselect signal (SCOC) */
#define _00_LCD_OUTPUT_DISABLE             (0x00U) /* output ground level to segment/common pin */
#define _40_LCD_OUTPUT_ENABLE              (0x40U) /* select common and segment pins output */
/* Voltage boost circuit and capacitor split circuit operation enable/disable (VLCON) */
#define _00_LCD_BOOST_CAPACITOR_DISABLE    (0x00U) /* stops voltage boost and capacitor split operation */
#define _20_LCD_BOOST_CAPACITOR_ENABLE     (0x20U) /* enables voltage boost and capacitor split operation */
/* Display data area control (BLON,LCDSEL) */
#define _00_LCD_DISPLAY_PATTERN_A          (0x00U) /* displaying an A-pattern area data */
#define _08_LCD_DISPLAY_PATTERN_B          (0x08U) /* displaying a B-pattern area data */
#define _10_LCD_DISPLAY_PATTERN_AB         (0x10U) /* blinking display alternately A-pattern and B-pattern area data */
/* Control of default value of voltage boosting pin (LCDVLM) */
#define _00_LCD_VOLTAGE_HIGH               (0x00U) /* VDD voltage is larger than 2.7V */
#define _01_LCD_VOLTAGE_LOW                (0x01U) /* VDD voltage is less than 4.2V */

/*
    LCD clock control register (LCDC0) 
*/
/* LCD clock (LCDCL) selection (LCDC05 - LCDC00) */
#define _01_LCD_CLOCK_FSX_FIL_2            (0x01U) /* fSX/2^2 or fIL/2^2 */
#define _02_LCD_CLOCK_FSX_FIL_3            (0x02U) /* fSX/2^3 or fIL/2^3 */
#define _03_LCD_CLOCK_FSX_FIL_4            (0x03U) /* fSX/2^4 or fIL/2^4 */
#define _04_LCD_CLOCK_FSX_FIL_5            (0x04U) /* fSX/2^5 or fIL/2^5 */
#define _05_LCD_CLOCK_FSX_FIL_6            (0x05U) /* fSX/2^6 or fIL/2^6 */
#define _06_LCD_CLOCK_FSX_FIL_7            (0x06U) /* fSX/2^7 or fIL/2^7 */
#define _07_LCD_CLOCK_FSX_FIL_8            (0x07U) /* fSX/2^8 or fIL/2^8 */
#define _08_LCD_CLOCK_FSX_FIL_9            (0x08U) /* fSX/2^9 or fIL/2^9 */
#define _09_LCD_CLOCK_FSX_10               (0x09U) /* fSX/2^10 */
#define _11_LCD_CLOCK_FMAIN_8              (0x11U) /* fMAIN/2^8 */
#define _12_LCD_CLOCK_FMAIN_9              (0x12U) /* fMAIN/2^9 */
#define _13_LCD_CLOCK_FMAIN_10             (0x13U) /* fMAIN/2^10 */
#define _14_LCD_CLOCK_FMAIN_11             (0x14U) /* fMAIN/2^11 */
#define _15_LCD_CLOCK_FMAIN_12             (0x15U) /* fMAIN/2^12 */
#define _16_LCD_CLOCK_FMAIN_13             (0x16U) /* fMAIN/2^13 */
#define _17_LCD_CLOCK_FMAIN_14             (0x17U) /* fMAIN/2^14 */
#define _18_LCD_CLOCK_FMAIN_15             (0x18U) /* fMAIN/2^15 */
#define _19_LCD_CLOCK_FMAIN_16             (0x19U) /* fMAIN/2^16 */
#define _1A_LCD_CLOCK_FMAIN_17             (0x1AU) /* fMAIN/2^17 */
#define _1B_LCD_CLOCK_FMAIN_18             (0x1BU) /* fMAIN/2^18 */
#define _2B_LCD_CLOCK_FMAIN_19             (0x2BU) /* fMAIN/2^19 */

/*
    LCD boost level control register (VLCD) 
*/
/* Reference voltage selection (contrast adjustment) (VLCD4 - VLCD0) */
#define _04_LCD_BOOST_VOLTAGE_100V         (0x04U) /* 1.00 V (1/3 bias: 3.00 V; 1/4 bias: 4.00 V) */
#define _05_LCD_BOOST_VOLTAGE_105V         (0x05U) /* 1.05 V (1/3 bias: 3.15 V; 1/4 bias: 4.20 V) */
#define _06_LCD_BOOST_VOLTAGE_110V         (0x06U) /* 1.10 V (1/3 bias: 3.30 V; 1/4 bias: 4.40 V) */
#define _07_LCD_BOOST_VOLTAGE_115V         (0x07U) /* 1.15 V (1/3 bias: 3.45 V; 1/4 bias: 4.60 V) */
#define _08_LCD_BOOST_VOLTAGE_120V         (0x08U) /* 1.20 V (1/3 bias: 3.60 V; 1/4 bias: 4.80 V) */
#define _09_LCD_BOOST_VOLTAGE_125V         (0x09U) /* 1.25 V (1/3 bias: 3.75 V; 1/4 bias: 5.00 V) */
#define _0A_LCD_BOOST_VOLTAGE_130V         (0x0AU) /* 1.30 V (1/3 bias: 3.90 V; 1/4 bias: 5.20 V) */
#define _0B_LCD_BOOST_VOLTAGE_135V         (0x0BU) /* 1.35 V (1/3 bias: 4.05 V; 1/4 bias: setting prohibited) */
#define _0C_LCD_BOOST_VOLTAGE_140V         (0x0CU) /* 1.40 V (1/3 bias: 4.20 V; 1/4 bias: setting prohibited) */
#define _0D_LCD_BOOST_VOLTAGE_145V         (0x0DU) /* 1.45 V (1/3 bias: 4.35 V; 1/4 bias: setting prohibited) */
#define _0E_LCD_BOOST_VOLTAGE_150V         (0x0EU) /* 1.50 V (1/3 bias: 4.50 V; 1/4 bias: setting prohibited) */
#define _0F_LCD_BOOST_VOLTAGE_155V         (0x0FU) /* 1.55 V (1/3 bias: 4.65 V; 1/4 bias: setting prohibited) */
#define _10_LCD_BOOST_VOLTAGE_160V         (0x10U) /* 1.60 V (1/3 bias: 4.80 V; 1/4 bias: setting prohibited) */
#define _11_LCD_BOOST_VOLTAGE_165V         (0x11U) /* 1.65 V (1/3 bias: 4.95 V; 1/4 bias: setting prohibited) */
#define _12_LCD_BOOST_VOLTAGE_170V         (0x12U) /* 1.70 V (1/3 bias: 5.10 V; 1/4 bias: setting prohibited) */
#define _13_LCD_BOOST_VOLTAGE_175V         (0x13U) /* 1.75 V (1/3 bias: 5.25 V; 1/4 bias: setting prohibited) */

/*
    LCD input switch control register (ISCLCD) 
*/
/* Control of schmitt trigger buffer of VL3/P125 pin (ISCVL3) */
#define _00_LCD_VL3_BUFFER_INVALID         (0x00U) /* makes digital input invalid (used as VL3) */
#define _02_LCD_VL3_BUFFER_VALID           (0x02U) /* makes digital input valid */
/* Control of schmitt trigger buffer of CAPL/P126 and CAPH/P127 pins (ISCCAP) */
#define _00_LCD_CAPLH_BUFFER_INVALID       (0x00U) /* makes digital input invalid (used as CAPL and CAPH) */
#define _01_LCD_CAPLH_BUFFER_VALID         (0x01U) /* makes digital input valid */

/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/
#define LCD_VOLTAGE_WAITTIME              (100000U) /* capacitor split wait time (100 ms) */
#define LCD_REFVOLTAGE_WAITTIME           (5000U) 	/* wait for the reference voltage setup time (5 ms (min.)) */
/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Global functions
***********************************************************************************************************************/
void init_Lcd(void);
void R_LCD_Start(void);
void R_LCD_Stop(void);
void R_LCD_Set_VoltageOn(void);
void R_LCD_Set_VoltageOff(void);
void R_LCD_PowerOff(void);
void R_LCD_PowerOn(void);

/* Start user code for function. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
#endif
