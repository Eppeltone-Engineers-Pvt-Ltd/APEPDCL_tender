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
* File Name    : r_cg_iica.h
* Version      : Code Generator for RL78/I1C V1.01.05.01 [28 Aug 2020]
* Device(s)    : R5F11TLG
* Tool-Chain   : CCRL
* Description  : This file implements device driver for IICA module.
* Creation Date: 31-12-2020
***********************************************************************************************************************/
#ifndef __DEV_IICA_H
#define __DEV_IICA_H

#include "..\\include\\dTypedef.h"

/***********************************************************************************************************************
Macro definitions (Register bit)
***********************************************************************************************************************/
/*
    IICA control register n0 (IICCTLn0) 
*/
/* I2C operation enable (IICEn) */
#define _00_IICA_OPERATION_STOP                       (0x00U) /* stop operation */
#define _80_IICA_OPERATION_ENABLE                     (0x80U) /* enable operation */
/* Exit from communications (LRELn) */
#define _00_IICA_OPERATION_NORMAL                     (0x00U) /* normal operation */
#define _40_IICA_OPERATION_STANDBY                    (0x40U) /* this exits from the current communications and sets standby mode */
/* Wait cancellation (WRELn) */
#define _00_IICA_NOTCANCEL_WAIT                       (0x00U) /* do not cancel wait */
#define _20_IICA_CANCEL_WAIT                          (0x20U) /* cancel wait */
/* Enable/disable generation of interrupt request when stop condition is detected (SPIEn) */
#define _00_IICA_INTERRUPT_REQUEST_DISABLE            (0x00U) /* disable */
#define _10_IICA_INTERRUPT_REQUEST_ENABLE             (0x10U) /* enable */
/* Control of wait and interrupt request generation (WTIMn) */
#define _00_IICA_INTERRUPT_REQUEST_EIGHTH             (0x00U) /* interrupt request is generated at the eighth clock's falling edge */
#define _08_IICA_INTERRUPT_REQUEST_NINTH              (0x08U) /* interrupt request is generated at the ninth clock's falling edge */
/* Acknowledgment control (ACKEn) */
#define _00_IICA_ACKOWNLEDGMENT_DISABLE               (0x00U) /* disable acknowledgment */
#define _04_IICA_ACKOWNLEDGMENT_ENABLE                (0x04U) /* enable acknowledgment */
/* Start condition trigger (STTn) */
#define _00_IICA_START_GENERATE                       (0x00U) /* do not generate a start condition */
#define _02_IICA_START_NOTGENERATE                    (0x02U) /* generate a start condition */
/* Stop condition trigger (SPTn) */
#define _00_IICA_STOP_GENERATE                        (0x00U) /* stop condition is not generated */
#define _01_IICA_STOP_NOTGENERATE                     (0x01U) /* stop condition is generated */

/*
    IICA status register n (IICSn) 
*/
/* Master status check flag (MSTSn) */
#define _00_IICA_SLAVE_DEVICE                         (0x00U) /* slave device status or communication standby status */
#define _80_IICA_MASTER_DEVICE                        (0x80U) /* master device communication status */
/* Detection of arbitration loss (ALDn) */
#define _00_IICA_RESULT_WIN                           (0x00U) /* no arbitration or that the arbitration result was a win */
#define _40_IICA_RESULT_LOSS                          (0x40U) /* arbitration result was a loss. The MSTSn bit is cleared */
/* Detection of extension code reception (EXCn) */
#define _00_IICA_NOTRECEIVED_CODE                     (0x00U) /* extension code was not received */
#define _20_IICA_RECEIVED_CODE                        (0x20U) /* extension code was received */
/* Detection of matching addresses (COIn) */
#define _00_IICA_ADDRESS_NOTMATCH                     (0x00U) /* addresses do not match */
#define _10_IICA_ADDRESS_MATCH                        (0x10U) /* addresses match */
/* Detection of transmit/receive status (TRCn) */
#define _00_IICA_RECEIVE_STATUS                       (0x00U) /* receive status */
#define _08_IICA_TRANSMIT_STATUS                      (0x08U) /* transmit status */
/* Detection of acknowledge (ACKDn) */
#define _00_IICA_ACKNOWLEDGE_NOTDETECTED              (0x00U) /* acknowledge was not detected */
#define _04_IICA_ACKNOWLEDGE_DETECTED                 (0x04U) /* acknowledge was detected */
/* Detection of start condition (STDn) */
#define _00_IICA_NOTDETECTED_START                    (0x00U) /* start condition was not detected */
#define _02_IICA_DETECTED_START                       (0x02U) /* start condition was detected */
/* Detection of stop condition (SPDn) */
#define _00_IICA_NOTDETECTED_STOP                     (0x00U) /* stop condition was not detected */
#define _01_IICA_DETECTED_STOP                        (0x01U) /* stop condition was detected */

/*
    IICA flag register n (IICFn) 
*/
/* STTn clear flag (STCFn) */
#define _00_IICA_GENERATE_START_CONDITION             (0x00U) /* generate start condition */
#define _80_IICA_NOTGENERATE_START_CONDITION          (0x80U) /* start condition generation unsuccessful */
/* I2C bus status flag (IICBSYn) */
#define _00_IICA_RESULT_WIN_STATUS                    (0x00U) /* bus release status */
#define _40_IICA_RESULT_LOSS_STATUS                   (0x40U) /* bus communication status */
/* Initial start enable trigger (STCENn) */
#define _00_IICA_UOPN_DETECTION                       (0x00U) /* enable generation of a start condition upon detection of a stop condition */
#define _02_IICA_WITHOUT_DETECTION                    (0x02U) /* enable generation of a start condition without detecting a stop condition */
/* Communication reservation function disable bit (IICRSVn) */
#define _00_IICA_RESERVATION_ENABLE                   (0x00U) /* enable communication reservation */
#define _01_IICA_RESERVATION_DISABLE                  (0x01U) /* disable communication reservation */

/*
    IICA control register n1 (IICCTLn1) 
*/
/* Control of address match wakeup (WUPn) */
#define _00_IICA_MATCH_STOP                           (0x00U) /* stops operation of address match wakeup function in STOP mode */
#define _80_IICA_MATCH_ENABLE                         (0x80U) /* enables operation of address match wakeup function in STOP mode */
/* Detection of SCLAn pin level (CLDn) */
#define _00_IICA_SCLAn_LOW                            (0x00U) /* the SCLAn pin was detected at low level */
#define _20_IICA_SCLAn_HIGH                           (0x20U) /* the SCLAn pin was detected at high level */
/* Detection of SDAAn pin level (DADn) */
#define _00_IICA_SDAAn_LOW                            (0x00U) /* the SDAAn pin was detected at low level */
#define _10_IICA_SDAAn_HIGH                           (0x10U) /* the SDAAn pin was detected at high level */
/* Operation mode switching (SMCn) */
#define _00_IICA_OPERATE_STANDARD                     (0x00U) /* operates in standard mode */
#define _08_IICA_OPERATE_FAST                         (0x08U) /* operates in fast mode */
/* Digital filter operation control (DFCn) */
#define _00_IICA_FILTER_OFF                           (0x00U) /* digital filter off */
#define _04_IICA_FILTER_ON                            (0x04U) /* digital filter on */
/* Operation clock (fMCK) control (PRSn) */
#define _00_IICA_FCLK_SELECTED                        (0x00U) /* selects fCLK */
#define _01_IICA_FCLK_2_SELECTED                      (0x01U) /* selects fCLK/2 */

/* IICA used flag */
#define _80_IICA_ADDRESS_COMPLETE                     (0x80U)
#define _00_IICA_MASTER_FLAG_CLEAR                    (0x00U)

/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/
#define _10_IICA0_MASTERADDRESS                       (0x10U) /* set the master address value */
#define _39_IICA_LOW_WIDTH                            (0x39U) /* set the low-level width (tLOW) of the SCLA0 pin signal */
#define _40_IICA_HIGH_WIDTH                           (0x40U) /* set the high-level width of the SCLA0 pin signal */

/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Global functions
***********************************************************************************************************************/

void init_I2c(void);
uint8_t write_eeprom(const uint8_t * tx_buf,uint16_t addr, uint16_t length);
uint8_t	read_eeprom(uint8_t* buf,uint16_t addr, uint16_t size);
uint8_t memTest(void);

#endif
