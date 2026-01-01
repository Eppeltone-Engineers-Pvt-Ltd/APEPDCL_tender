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
* File Name    : r_dlms_obis.c
* Version      : 1.00
* Device(s)    : None
* Tool-Chain   :
* H/W Platform : Unified Energy Meter Platform
* Description  :
******************************************************************************
* History : DD.MM.YYYY Version Description
*         : 21.02.2013
******************************************************************************/

/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#include "..\\r_dlms_typedef.h"		/* DLMS Type Definitions */
#include "r_dlms_obis_ic.h"		/* DLMS OBIS IC Definitions */
#include "..\\meter_app\\r_dlms_data_ic.h"		/* DLMS Data IC Definitions */
#include "..\\meter_app\\r_dlms_data.h"		/* DLMS Data Definitions */
#include "r_dlms_obis.h"		/* DLMS OBIS Definitions */

/* Standard Lib */
#include <string.h>			/* String Standard Lib */

/******************************************************************************
Macro definitions
******************************************************************************/

/******************************************************************************
Typedef definitions
******************************************************************************/

/******************************************************************************
Imported global variables and functions (from other files)
******************************************************************************/
/* Imported from COSEMApplication/GetService.c */
extern Unsigned16 OBIS_DataLength;

/*
 * Imported from r_dlms_data_ic.c
 * To access to all child table definitions
 */
extern const class01_child_record_t g_ChildTableClass01[]; 		 /* Data                   			*/
extern const class03_child_record_t g_ChildTableClass03[]; 		 /* Register               			*/
extern const class04_child_record_t g_ChildTableClass04[]; 		 /* Extended Register      			*/
extern const class05_child_record_t g_ChildTableClass05[]; 		 /* Demand Register        			*/
extern const class06_child_record_t g_ChildTableClass06[]; 		 /* Register Activation    			*/
extern const class07_child_record_t g_ChildTableClass07[]; 		 /* Profile Generic        			*/
extern const class08_child_record_t g_ChildTableClass08[]; 		 /* Clock                  			*/
extern const class09_child_record_t g_ChildTableClass09[]; 		 /* Script Table           			*/
extern const class10_child_record_t g_ChildTableClass10[]; 		 /* Schedule               			*/
extern const class11_child_record_t g_ChildTableClass11[]; 		 /* Special Days Table     			*/
extern const class12_child_record_t g_ChildTableClass12[]; 		 /* Association SN         			*/
extern const class15_child_record_t g_ChildTableClass15[]; 		 /* Association LN         			*/
extern const class17_child_record_t g_ChildTableClass17[]; 		 /* SAP Assignment         			*/
extern const class18_child_record_t g_ChildTableClass18[]; 		 /* Image Transfer         			*/
extern const class19_child_record_t g_ChildTableClass19[]; 		 /* IEC Local Port Setup   			*/
extern const class20_child_record_t g_ChildTableClass20[]; 		 /* Activity Calendar      			*/
extern const class21_child_record_t g_ChildTableClass21[]; 		 /* Register Monitor       			*/
extern const class22_child_record_t g_ChildTableClass22[]; 		 /* Single Action Schedule 			*/
extern const class23_child_record_t g_ChildTableClass23[]; 		 /* IEC HDLC Setup         			*/
extern const class24_child_record_t g_ChildTableClass24[]; 		 /* IEC Twisted Pair (1) Setup		*/
extern const class25_child_record_t g_ChildTableClass25[]; 		 /* M-bus slave port setup 			*/
extern const class26_child_record_t g_ChildTableClass26[]; 		 /* Utility Table          			*/
extern const class27_child_record_t g_ChildTableClass27[]; 		 /* Modem Configuration    			*/
extern const class28_child_record_t g_ChildTableClass28[]; 		 /* Auto Answer            			*/
extern const class29_child_record_t g_ChildTableClass29[]; 		 /* Auto Connect           			*/
extern const class41_child_record_t g_ChildTableClass41[]; 		 /* TCP-UDP setup          			*/
extern const class42_child_record_t g_ChildTableClass42[]; 		 /* IPv4 setup             			*/
extern const class43_child_record_t g_ChildTableClass43[]; 		 /* MAC address setup      			*/
extern const class44_child_record_t g_ChildTableClass44[]; 		 /* PPP setup              			*/
extern const class45_child_record_t g_ChildTableClass45[]; 		 /* GPRS modem setup				*/
extern const class46_child_record_t g_ChildTableClass46[]; 		 /* SMTP setup						*/
extern const class48_child_record_t g_ChildTableClass48[]; 		 /* IPv6 setup						*/
extern const class50_child_record_t g_ChildTableClass50[]; 		 /* S-FSK Phy&MAC setup				*/
extern const class51_child_record_t g_ChildTableClass51[]; 		 /* S-FSK Active initiator			*/
extern const class52_child_record_t g_ChildTableClass52[]; 		 /* S-FSK MAC synchronization		*/
extern const class53_child_record_t g_ChildTableClass53[]; 		 /* S-FSK MAC counters				*/
extern const class55_child_record_t g_ChildTableClass55[]; 		 /* IEC 61334-4-32 LLC setu 		*/
extern const class56_child_record_t g_ChildTableClass56[]; 		 /* S-FSK Reporting system  		*/
extern const class57_child_record_t g_ChildTableClass57[]; 		 /* ISO/IEC 8802-2 LLC Type 		*/
extern const class58_child_record_t g_ChildTableClass58[]; 		 /* ISO/IEC 8802-2 LLC Type 		*/
extern const class59_child_record_t g_ChildTableClass59[]; 		 /* ISO/IEC 8802-2 LLC Type 		*/
extern const class61_child_record_t g_ChildTableClass61[]; 		 /* Register Table          		*/
extern const class63_child_record_t g_ChildTableClass63[]; 		 /* Status mapping          		*/
extern const class64_child_record_t g_ChildTableClass64[]; 		 /* Security Setup          		*/
extern const class67_child_record_t g_ChildTableClass67[]; 		 /* Sensor manager         			*/
extern const class70_child_record_t g_ChildTableClass70[]; 		 /* Disconnection Control   		*/
extern const class71_child_record_t g_ChildTableClass71[]; 		 /* Limiter                 		*/
extern const class72_child_record_t g_ChildTableClass72[]; 		 /* M-bus client            		*/
extern const class73_child_record_t g_ChildTableClass73[]; 		 /* Wireless Mode Q Channel 		*/
extern const class74_child_record_t g_ChildTableClass74[]; 		 /* M-bus master port setup 		*/
extern const class80_child_record_t g_ChildTableClass80[]; 		 /* 61334-4-32 LLC SSCS setup 							*/
extern const class81_child_record_t g_ChildTableClass81[]; 		 /* PRIME NB OFDM PLC Physical layer counters 			*/
extern const class82_child_record_t g_ChildTableClass82[]; 		 /* PRIME NB OFDM PLC MAC setup 						*/
extern const class83_child_record_t g_ChildTableClass83[]; 		 /* PRIME NB OFDM PLC MAC functional parameters 		*/
extern const class84_child_record_t g_ChildTableClass84[]; 		 /* PRIME NB OFDM PLC MAC counters 						*/
extern const class85_child_record_t g_ChildTableClass85[]; 		 /* PRIME NB OFDM PLC MAC network administration data 	*/
extern const class86_child_record_t g_ChildTableClass86[]; 		 /* PRIME NB OFDM PLC Applications identification 		*/
extern const class90_child_record_t g_ChildTableClass90[]; 		 /* G3 NB OFDM PLC MAC layer counters 					*/
extern const class91_child_record_t g_ChildTableClass91[]; 		 /* G3 NB OFDM PLC MAC setup 							*/
extern const class92_child_record_t g_ChildTableClass92[]; 		 /* G3 NB OFDM PLC 6LoWPAN adaptation layer setup 		*/
//extern const class101_child_record_t g_ChildTableClass101[];	 /* ZigBee SAS Startup				*/
//extern const class102_child_record_t g_ChildTableClass102[];	 /* ZigBee SAS Join					*/
//extern const class103_child_record_t g_ChildTableClass103[];	 /* ZigBee SAS APS Fragment 		*/
//extern const class104_child_record_t g_ChildTableClass104[];	 /* ZigBee SAS Network Control		*/
//extern const class105_child_record_t g_ChildTableClass105[];	 /* ZigBee SAS Startup				*/

extern const Unsigned16 g_ChildTableLengthClass01;  /* Data                    */
extern const Unsigned16 g_ChildTableLengthClass03;  /* Register                */
extern const Unsigned16 g_ChildTableLengthClass04;  /* Extended Register       */
extern const Unsigned16 g_ChildTableLengthClass05;  /* Demand Register         */
extern const Unsigned16 g_ChildTableLengthClass06;  /* Register Activation     */
extern const Unsigned16 g_ChildTableLengthClass07;  /* Profile Generic         */
extern const Unsigned16 g_ChildTableLengthClass08;  /* Clock                   */
extern const Unsigned16 g_ChildTableLengthClass09;  /* Script Table            */
extern const Unsigned16 g_ChildTableLengthClass10;  /* Schedule                */
extern const Unsigned16 g_ChildTableLengthClass11;  /* Special Days Table      */
extern const Unsigned16 g_ChildTableLengthClass12;  /* Association SN          */
extern const Unsigned16 g_ChildTableLengthClass15;  /* Association LN          */
extern const Unsigned16 g_ChildTableLengthClass17;  /* SAP Assignment          */
extern const Unsigned16 g_ChildTableLengthClass18;  /* Image Transfer          */
extern const Unsigned16 g_ChildTableLengthClass19;  /* IEC Local Port Setup    */
extern const Unsigned16 g_ChildTableLengthClass20;  /* Activity Calendar       */
extern const Unsigned16 g_ChildTableLengthClass21;  /* Register Monitor        */
extern const Unsigned16 g_ChildTableLengthClass22;  /* Single Action Schedule  */
extern const Unsigned16 g_ChildTableLengthClass23;  /* IEC HDLC Setup          */
extern const Unsigned16 g_ChildTableLengthClass24;  /* IEC Twisted Pair (1) Se */
extern const Unsigned16 g_ChildTableLengthClass25;  /* M-bus slave port setup  */
extern const Unsigned16 g_ChildTableLengthClass26;  /* Utility Table           */
extern const Unsigned16 g_ChildTableLengthClass27;  /* Modem Configuration     */
extern const Unsigned16 g_ChildTableLengthClass28;  /* Auto Answer             */
extern const Unsigned16 g_ChildTableLengthClass29;  /* Auto Connect            */
extern const Unsigned16 g_ChildTableLengthClass41;  /* TCP-UDP setup           */
extern const Unsigned16 g_ChildTableLengthClass42;  /* IPv4 setup              */
extern const Unsigned16 g_ChildTableLengthClass43;  /* MAC address setup       */
extern const Unsigned16 g_ChildTableLengthClass44;  /* PPP setup               */
extern const Unsigned16 g_ChildTableLengthClass45;  /* GPRS modem setup        */
extern const Unsigned16 g_ChildTableLengthClass46;  /* SMTP setup              */
extern const Unsigned16 g_ChildTableLengthClass48;  /* IPv6 setup              */
extern const Unsigned16 g_ChildTableLengthClass50;  /* S-FSK Phy&MAC setup     */
extern const Unsigned16 g_ChildTableLengthClass51;  /* S-FSK Active initiator  */
extern const Unsigned16 g_ChildTableLengthClass52;  /* S-FSK MAC synchronizati */
extern const Unsigned16 g_ChildTableLengthClass53;  /* S-FSK MAC counters      */
extern const Unsigned16 g_ChildTableLengthClass55;  /* IEC 61334-4-32 LLC setu */
extern const Unsigned16 g_ChildTableLengthClass56;  /* S-FSK Reporting system  */
extern const Unsigned16 g_ChildTableLengthClass57;  /* ISO/IEC 8802-2 LLC Type */
extern const Unsigned16 g_ChildTableLengthClass58;  /* ISO/IEC 8802-2 LLC Type */
extern const Unsigned16 g_ChildTableLengthClass59;  /* ISO/IEC 8802-2 LLC Type */
extern const Unsigned16 g_ChildTableLengthClass61;  /* Register Table          */
extern const Unsigned16 g_ChildTableLengthClass63;  /* Status mapping          */
extern const Unsigned16 g_ChildTableLengthClass64;  /* Security Setup          */
extern const Unsigned16 g_ChildTableLengthClass67;  /* Sensor manager          */
extern const Unsigned16 g_ChildTableLengthClass70;  /* Disconnection Control   */
extern const Unsigned16 g_ChildTableLengthClass71;  /* Limiter                 */
extern const Unsigned16 g_ChildTableLengthClass72;  /* M-bus client            */
extern const Unsigned16 g_ChildTableLengthClass73;  /* Wireless Mode Q Channel */
extern const Unsigned16 g_ChildTableLengthClass74;  /* M-bus master port setup */
extern const Unsigned16 g_ChildTableLengthClass80;  /* 61334-4-32 LLC SSCS setup 							*/
extern const Unsigned16 g_ChildTableLengthClass81;  /* PRIME NB OFDM PLC Physical layer counters 			*/
extern const Unsigned16 g_ChildTableLengthClass82;  /* PRIME NB OFDM PLC MAC setup 							*/
extern const Unsigned16 g_ChildTableLengthClass83;  /* PRIME NB OFDM PLC MAC functional parameters 			*/
extern const Unsigned16 g_ChildTableLengthClass84;  /* PRIME NB OFDM PLC MAC counters 						*/
extern const Unsigned16 g_ChildTableLengthClass85;  /* PRIME NB OFDM PLC MAC network administration data 	*/
extern const Unsigned16 g_ChildTableLengthClass86;  /* PRIME NB OFDM PLC Applications identification 		*/
extern const Unsigned16 g_ChildTableLengthClass90;  /* G3 NB OFDM PLC MAC layer counters 					*/
extern const Unsigned16 g_ChildTableLengthClass91;  /* G3 NB OFDM PLC MAC setup 							*/
extern const Unsigned16 g_ChildTableLengthClass92;  /* G3 NB OFDM PLC 6LoWPAN adaptation layer setup 		*/
extern const Unsigned16 g_ChildTableLengthClass101;	/* ZigBee SAS Startup		*/
extern const Unsigned16 g_ChildTableLengthClass102;	/* ZigBee SAS Join			*/
extern const Unsigned16 g_ChildTableLengthClass103;	/* ZigBee SAS APS Frament	*/
extern const Unsigned16 g_ChildTableLengthClass104;	/* ZigBee Network Control	*/
extern const Unsigned16 g_ChildTableLengthClass105;	/* ZigBee Tunnel Setup		*/

/******************************************************************************
Exported global variables and functions (to be accessed by other files)
******************************************************************************/
Unsigned8  g_ServiceBuffer[OBIS_SERVICE_DATA_BUFFER_LENGTH];
Unsigned16 g_TotalNumberObjects;
Unsigned16 g_TotalNumberObjectsOfAssociation[OBIS_NUMBER_ASSOCIATION];

/******************************************************************************
Private global variables and functions
******************************************************************************/

/* Master table */
const master_record_t g_MasterTable[] =
{
    /*
	 *----------------------------------------------
	 * "IC Name"
	 * {
	 *    Class Version, Number of attrs, Number of methods,
	 *    Class Distributor Function,
	 *    Child-list size,
	 *    Child-pointer
	 * }
     *----------------------------------------------
     */

	/* 0 - (reserved or empty) */
	{0, 0, 0, NULL, NULL, NULL, 0},

    /* 1 - Data */
	{
		0, 2, 0,
#if (defined(USED_CLASS_01) && USED_CLASS_01 == 1)
		R_OBIS_DistributeDataClass,
		(Unsigned16 *)&g_ChildTableLengthClass01,
		(child_record_t *)&g_ChildTableClass01,
		sizeof(class01_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

    /* 2 - (reserved or empty) */
	{0, 0, 0, NULL, NULL, NULL, 0},

    /* 3 - Register */
	{
		0, 3, 1,
#if (defined(USED_CLASS_03) && USED_CLASS_03 == 1)
		R_OBIS_DistributeRegisterClass,
		(Unsigned16 *)&g_ChildTableLengthClass03,
		(child_record_t *)&g_ChildTableClass03,
		sizeof(class03_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

    /* 4 - Extended Register */
	{
		0, 5, 1,
#if (defined(USED_CLASS_04) && USED_CLASS_04 == 1)
		R_OBIS_DistributeXRegisterClass,
		(Unsigned16 *)&g_ChildTableLengthClass04,
		(child_record_t *)&g_ChildTableClass04,
		sizeof(class04_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

    /* 5 - Demand Register */
	{
		0, 9, 2,
#if (defined(USED_CLASS_05) && USED_CLASS_05 == 1)
		R_OBIS_DistributeDRegisterClass,
		&g_ChildTableLengthClass05,
		(child_record_t *)&g_ChildTableClass05,
		sizeof(class05_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

    /* 6 - Register Activation */
	{
		0, 4, 3,
#if (defined(USED_CLASS_06) && USED_CLASS_06 == 1)
		R_OBIS_DistributeRegisterActivationClass,
		&g_ChildTableLengthClass06,
		(child_record_t *)&g_ChildTableClass06,
		sizeof(class06_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

    /* 7 - Profile Generic */
	{
#if (defined(VERSION0_CLASS_07) && VERSION0_CLASS_07 == 1)
		0, 8, 4,			//Version 0
#else
		1, 8, 4, 			//Version 1
#endif
#if (defined(USED_CLASS_07) && USED_CLASS_07 == 1)
		R_OBIS_DistributeProfileGenericClass,
		(Unsigned16 *)&g_ChildTableLengthClass07,
		(child_record_t *)&g_ChildTableClass07,
		sizeof(class07_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

    /* 8 - Clock */
	{
		0, 9, 0, /* Dummy for clock object here, set to 0, the actual number of method must be 6 */
#if (defined(USED_CLASS_08) && USED_CLASS_08 == 1)
		R_OBIS_DistributeClockClass,
		(Unsigned16 *)&g_ChildTableLengthClass08,
		(child_record_t *)&g_ChildTableClass08,
		sizeof(class08_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

    /* 9 - Script Table */
	{
		0, 2, 1,
#if (defined(USED_CLASS_09) && USED_CLASS_09 == 1)
		R_OBIS_DistributeScriptTableClass,
		(Unsigned16 *)&g_ChildTableLengthClass09,
		(child_record_t *)&g_ChildTableClass09,
		sizeof(class09_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

    /* 10 - Schedule */
	{
		0, 2, 3,
#if (defined(USED_CLASS_10) && USED_CLASS_10 == 1)
		R_OBIS_DistributeScheduleClass,
		&g_ChildTableLengthClass10,
		(child_record_t *)&g_ChildTableClass10,
		sizeof(class10_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

    /* 11 - Special Days Table */
	{
		0, 2, 2,
#if (defined(USED_CLASS_11) && USED_CLASS_11 == 1)
		R_OBIS_DistributeSpecialDaysTableClass,
		&g_ChildTableLengthClass11,
		(child_record_t *)&g_ChildTableClass11,
		sizeof(class11_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

    /* 12 - Association SN */
	{
		2, 4, 8,
#if (defined(USED_CLASS_12) && USED_CLASS_12 == 1)
		NULL /* TODO */,
		&g_ChildTableLengthClass12,
		(child_record_t *)&g_ChildTableClass12,
		sizeof(class12_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

    /* 13 - (reserved or empty) */
	{0, 0, 0, NULL, NULL, NULL, 0},

	/* 14 - (reserved or empty) */
	{0, 0, 0, NULL, NULL, NULL, 0},

    /* 15 - Association LN */
	{
#if (defined(VERSION0_CLASS_15) && VERSION0_CLASS_15 == 1)
		0, 8, 4,			// version 0
#else
		1, 9, 4,			// version 1
#endif
#if (defined(USED_CLASS_15) && USED_CLASS_15 == 1)
		R_OBIS_DistributeAssociationLNClass,
		(Unsigned16 *)&g_ChildTableLengthClass15,
		(child_record_t *)&g_ChildTableClass15,
		sizeof(class15_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

	/* 16 - (reserved or empty) */
	{0, 0, 0, NULL, NULL, NULL, 0},

	/* 17 - SAP Assignment */
	{
		0, 2, 1,
#if (defined(USED_CLASS_17) && USED_CLASS_17 == 1)
		R_OBIS_DistributeSAPassignmentClass,
		&g_ChildTableLengthClass17,
		(child_record_t *)&g_ChildTableClass17,
		sizeof(class17_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

	/* 18 - Image Transfer */
	{
		0, 7, 4,
#if (defined(USED_CLASS_18) && USED_CLASS_18 == 1)
		R_OBIS_DistributeImageTransferClass,
		&g_ChildTableLengthClass18,
		(child_record_t *)&g_ChildTableClass18,
		sizeof(class18_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

	/* 19 - IEC Local Port Setup */
	{
#if (defined(VERSION0_CLASS_19) && VERSION0_CLASS_19 == 1)
		0, 9, 0,
#else
		1, 9, 0,
#endif
#if (defined(USED_CLASS_19) && USED_CLASS_19 == 1)
		R_OBIS_DistributeIECLocalPortSetupClass,
		&g_ChildTableLengthClass19,
		(child_record_t *)&g_ChildTableClass19,
		sizeof(class19_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

	/* 20 - Activity Calendar */
	{
		0, 10, 1,
#if (defined(USED_CLASS_20) && USED_CLASS_20 == 1)
		R_OBIS_DistributeActivityCalendarClass,
		(Unsigned16 *)&g_ChildTableLengthClass20,
		(child_record_t *)&g_ChildTableClass20,
		sizeof(class20_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

	/* 21 - Register Monitor */
	{
		0, 4, 0,
#if (defined(USED_CLASS_21) && USED_CLASS_21 == 1)
		R_OBIS_DistributeRegisterMonitorClass,
		&g_ChildTableLengthClass21,
		(child_record_t *)&g_ChildTableClass21,
		sizeof(class21_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

	/* 22 - Single Action Schedule */
	{
		0, 4, 0,
#if (defined(USED_CLASS_22) && USED_CLASS_22 == 1)
		R_OBIS_DistributeSingleActionScheduleClass,
		(Unsigned16 *)&g_ChildTableLengthClass22,
		(child_record_t *)&g_ChildTableClass22,
		sizeof(class22_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

	/* 23 - IEC HDLC Setup */
	{
#if (defined(VERSION0_CLASS_23) && VERSION0_CLASS_23 == 1)
		0, 9, 0,
#else
		1, 9, 0,
#endif		
#if (defined(USED_CLASS_23) && USED_CLASS_23 == 1)
		R_OBIS_DistributeIECHDLCSetupClass,
		(Unsigned16 *)&g_ChildTableLengthClass23,
		(child_record_t *)&g_ChildTableClass23,
		sizeof(class23_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

	/* 24 - IEC Twisted Pair (1) Setup */
	{
#if (defined(VERSION0_CLASS_24) && VERSION0_CLASS_24 == 1)
		0, 5, 0,
#else
		1, 5, 0,
#endif
#if (defined(USED_CLASS_24) && USED_CLASS_24 == 1)
		R_OBIS_DistributeIECTwistedPairClass,
		&g_ChildTableLengthClass24,
		(child_record_t *)&g_ChildTableClass24,
		sizeof(class24_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

	/* 25 - M-bus slave port setup */
	{
		0, 5, 0,
#if (defined(USED_CLASS_25) && USED_CLASS_25 == 1)
		NULL /* TODO */,
		&g_ChildTableLengthClass25,
		(child_record_t *)&g_ChildTableClass25,
		sizeof(class25_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

	/* 26 - Utility Table */
	{
		0, 4, 0,
#if (defined(USED_CLASS_26) && USED_CLASS_26 == 1)
		R_OBIS_DistributeUtilityTablesClass,
		&g_ChildTableLengthClass26,
		(child_record_t *)&g_ChildTableClass26,
		sizeof(class26_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

	/* 27 - Modem Configuration */
	{
#if (defined(VERSION0_CLASS_27) && VERSION0_CLASS_27 == 1)
		0, 4, 0,
#else
		1, 4, 0,
#endif
#if (defined(USED_CLASS_27) && USED_CLASS_27 == 1)
		R_OBIS_DistributeModemConfigClass,
		&g_ChildTableLengthClass27,
		(child_record_t *)&g_ChildTableClass27,
		sizeof(class27_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

	/* 28 - Auto Answer */
	{
#if (defined(VERSION0_CLASS_28) && VERSION0_CLASS_28 == 1)
		0, 6, 0,
#else
		2, 7, 0,
#endif
#if (defined(USED_CLASS_28) && USED_CLASS_28 == 1)
		R_OBIS_DistributeAutoAnswerClass,
		&g_ChildTableLengthClass28,
		(child_record_t *)&g_ChildTableClass28,
		sizeof(class28_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

	/* 29 - Auto Connect */
	{
#if (defined(VERSION0_CLASS_29) && VERSION0_CLASS_29 == 1)
		0, 6, 0,
#else
		1, 6, 0,
#endif		
#if (defined(USED_CLASS_29) && USED_CLASS_29 == 1)
		R_OBIS_DistributeAutoConnectClass,
		&g_ChildTableLengthClass29,
		(child_record_t *)&g_ChildTableClass29,
		sizeof(class29_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

	/* 30 - (reserved or empty) */
	{0, 0, 0, NULL, NULL, NULL, 0},

	/* 31 - (reserved or empty) */
	{0, 0, 0, NULL, NULL, NULL, 0},

	/* 32 - (reserved or empty) */
	{0, 0, 0, NULL, NULL, NULL, 0},

	/* 33 - (reserved or empty) */
	{0, 0, 0, NULL, NULL, NULL, 0},

	/* 34 - (reserved or empty) */
	{0, 0, 0, NULL, NULL, NULL, 0},

	/* 35 - (reserved or empty) */
	{0, 0, 0, NULL, NULL, NULL, 0},

	/* 36 - (reserved or empty) */
	{0, 0, 0, NULL, NULL, NULL, 0},

	/* 37 - (reserved or empty) */
	{0, 0, 0, NULL, NULL, NULL, 0},

	/* 38 - (reserved or empty) */
	{0, 0, 0, NULL, NULL, NULL, 0},

	/* 39 - (reserved or empty) */
	{0, 0, 0, NULL, NULL, NULL, 0},

	/* 40 - (reserved or empty) */
	{0, 0, 0, NULL, NULL, NULL, 0},

	/* 41 - TCP-UDP setup */
	{
		0, 6, 0,
#if (defined(USED_CLASS_41) && USED_CLASS_41 == 1)
		R_OBIS_DistributeTCPUDPSetupClass,
		&g_ChildTableLengthClass41,
		(child_record_t *)&g_ChildTableClass41,
		sizeof(class41_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

	/* 42 - IPv4 setup */
	{
		0, 10, 3,
#if (defined(USED_CLASS_42) && USED_CLASS_42 == 1)
		NULL /* TODO */,
		&g_ChildTableLengthClass42,
		(child_record_t *)&g_ChildTableClass42,
		sizeof(class42_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

	/* 43 - MAC address setup */
	{
		0, 2, 0,
#if (defined(USED_CLASS_43) && USED_CLASS_43 == 1)
		NULL /* TODO */,
		&g_ChildTableLengthClass43,
		(child_record_t *)&g_ChildTableClass43,
		sizeof(class43_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

	/* 44 - PPP setup */
	{
		0, 5, 0,
#if (defined(USED_CLASS_44) && USED_CLASS_44 == 1)
		NULL /* TODO */,
		&g_ChildTableLengthClass44,
		(child_record_t *)&g_ChildTableClass44,
		sizeof(class44_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

	/* 45 - GPRS modem setup */
	{
		0, 4, 0,
#if (defined(USED_CLASS_45) && USED_CLASS_45 == 1)
		NULL /* TODO */,
		&g_ChildTableLengthClass45,
		(child_record_t *)&g_ChildTableClass45,
		sizeof(class45_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

	/* 46 - SMTP setup */
	{
		0, 6, 0,
#if (defined(USED_CLASS_46) && USED_CLASS_46 == 1)
		NULL /* TODO */,
		&g_ChildTableLengthClass46,
		(child_record_t *)&g_ChildTableClass46,
		sizeof(class46_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

	/* 47 - (reserved or empty) */
	{0, 0, 0, NULL, NULL, NULL, 0},

	/* 48 - IPv6 setup */
	{
		0, 10, 2,
#if (defined(USED_CLASS_48) && USED_CLASS_48 == 1)
		NULL /* TODO */,
		&g_ChildTableLengthClass48,
		(child_record_t *)&g_ChildTableClass48,
		sizeof(class48_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

	/* 49 - (reserved or empty) */
	{0, 0, 0, NULL, NULL, NULL, 0},

	/* 50 - S-FSK Phy&MAC setup */
	{
		1, 15, 0,
#if (defined(USED_CLASS_50) && USED_CLASS_50 == 1)
		NULL /* TODO */,
		&g_ChildTableLengthClass50,
		(child_record_t *)&g_ChildTableClass50,
		sizeof(class50_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

	/* 51 - S-FSK Active initiator */
	{
		0, 2, 1,
#if (defined(USED_CLASS_51) && USED_CLASS_51 == 1)
		NULL /* TODO */,
		&g_ChildTableLengthClass51,
		(child_record_t *)&g_ChildTableClass51,
		sizeof(class51_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

	/* 52 - S-FSK MAC synchronization timeouts */
	{
		0, 5, 0,
#if (defined(USED_CLASS_52) && USED_CLASS_52 == 1)
		NULL /* TODO */,
		&g_ChildTableLengthClass52,
		(child_record_t *)&g_ChildTableClass52,
		sizeof(class52_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

	/* 53 - S-FSK MAC counters */
	{
		0, 8, 1,
#if (defined(USED_CLASS_53) && USED_CLASS_53 == 1)
		NULL /* TODO */,
		&g_ChildTableLengthClass53,
		(child_record_t *)&g_ChildTableClass53,
		sizeof(class53_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

	/* 54 - (reserved or empty) */
	{0, 0, 0, NULL, NULL, NULL, 0},

	/* 55 - IEC 61334-4-32 LLC setup */
	{
		1, 3, 0,
#if (defined(USED_CLASS_55) && USED_CLASS_55 == 1)
		NULL /* TODO */,
		&g_ChildTableLengthClass55,
		(child_record_t *)&g_ChildTableClass55,
		sizeof(class55_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

	/* 56 - S-FSK Reporting system list */
	{
		0, 2, 0,
#if (defined(USED_CLASS_56) && USED_CLASS_56 == 1)
		NULL /* TODO */,
		&g_ChildTableLengthClass56,
		(child_record_t *)&g_ChildTableClass56,
		sizeof(class56_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

	/* 57 - ISO/IEC 8802-2 LLC Type 1 Setup */
	{
		0, 2, 0,
#if (defined(USED_CLASS_57) && USED_CLASS_57 == 1)
		NULL /* TODO */,
		&g_ChildTableLengthClass57,
		(child_record_t *)&g_ChildTableClass57,
		sizeof(class57_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

	/* 58 - ISO/IEC 8802-2 LLC Type 2 Setup */
	{
		0, 9, 0,
#if (defined(USED_CLASS_58) && USED_CLASS_58 == 1)
		NULL /* TODO */,
		&g_ChildTableLengthClass58,
		(child_record_t *)&g_ChildTableClass58,
		sizeof(class58_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

	/* 59 - ISO/IEC 8802-2 LLC Type 3 Setup */
	{
		0, 6, 0,
#if (defined(USED_CLASS_59) && USED_CLASS_59 == 1)
		NULL /* TODO */,
		&g_ChildTableLengthClass59,
		(child_record_t *)&g_ChildTableClass59,
		sizeof(class59_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

	/* 60 - (reserved or empty) */
	{0, 0, 0, NULL, NULL, NULL, 0},

	/* 61 - Register Table */
	{
		0, 4, 2,
#if (defined(USED_CLASS_61) && USED_CLASS_61 == 1)
		R_OBIS_DistributeRegisterTableClass,
		&g_ChildTableLengthClass61,
		(child_record_t *)&g_ChildTableClass61,
		sizeof(class61_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

	/* 62 - (reserved or empty) */
	{0, 0, 0, NULL, NULL, NULL, 0},

	/* 63 - Status mapping */
	{
		0, 3, 0,
#if (defined(USED_CLASS_63) && USED_CLASS_63 == 1)
		R_OBIS_DistributeStatusMappingClass,
		&g_ChildTableLengthClass63,
		(child_record_t *)&g_ChildTableClass63,
		sizeof(class63_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

	/* 64 - Security Setup */
	{
		0, 5, 2,
#if (defined(USED_CLASS_64) && USED_CLASS_64 == 1)
		R_OBIS_DistributeSecuritySetupClass,
		&g_ChildTableLengthClass64,
		(child_record_t *)&g_ChildTableClass64,
		sizeof(class64_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

	/* 65 - (reserved or empty) */
	{0, 0, 0, NULL, NULL, NULL, 0},

	/* 66 - (reserved or empty) */
	{0, 0, 0, NULL, NULL, NULL, 0},

	/* 67 - Sensor manager */
	{
		0, 15, 1,
#if (defined(USED_CLASS_67) && USED_CLASS_67 == 1)
		NULL /* TODO */,
		&g_ChildTableLengthClass67,
		(child_record_t *)&g_ChildTableClass67,
		sizeof(class67_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

	/* 68 - (reserved or empty) */
	{0, 0, 0, NULL, NULL, NULL, 0},

	/* 69 - (reserved or empty) */
	{0, 0, 0, NULL, NULL, NULL, 0},

	/* 70 - Disconnection Control */
	{
		0, 4, 2,
#if (defined(USED_CLASS_70) && USED_CLASS_70 == 1)
		R_OBIS_DistributeDisconnectControlClass,
		&g_ChildTableLengthClass70,
		(child_record_t *)&g_ChildTableClass70,
		sizeof(class70_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

	/* 71 - Limiter */
	{
		0, 11, 0,
#if (defined(USED_CLASS_71) && USED_CLASS_71 == 1)
		R_OBIS_DistributeLimiterClass,
		&g_ChildTableLengthClass71,
		(child_record_t *)&g_ChildTableClass71,
		sizeof(class71_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

	/* 72 - M-bus client */
	{
		0, 12, 8,
#if (defined(USED_CLASS_72) && USED_CLASS_72 == 1)
		NULL /* TODO */,
		&g_ChildTableLengthClass72,
		(child_record_t *)&g_ChildTableClass72,
		sizeof(class72_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

	/* 73 - Wireless Mode Q Channel */
	{
		1, 4, 0,
#if (defined(USED_CLASS_73) && USED_CLASS_73 == 1)
		NULL /* TODO */,
		&g_ChildTableLengthClass73,
		(child_record_t *)&g_ChildTableClass73,
		sizeof(class73_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},

	/* 74 - M-bus master port setup */
	{
		0, 2, 0,
#if (defined(USED_CLASS_74) && USED_CLASS_74 == 1)
		NULL /* TODO */,
		&g_ChildTableLengthClass74,
		(child_record_t *)&g_ChildTableClass74,
		sizeof(class74_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},
	
	/* 75 - (reserved or empty) */
	{0, 0, 0, NULL, NULL, NULL, 0},
	/* 76 - (reserved or empty) */
	{0, 0, 0, NULL, NULL, NULL, 0},
	/* 77 - (reserved or empty) */
	{0, 0, 0, NULL, NULL, NULL, 0},
	/* 78 - (reserved or empty) */
	{0, 0, 0, NULL, NULL, NULL, 0},
	/* 79 - (reserved or empty) */
	{0, 0, 0, NULL, NULL, NULL, 0},
	/* 80 - 61334-4-32 LLC SSCS setup */
	{
		0, 3, 1,
#if (defined(USED_CLASS_80) && USED_CLASS_80 == 1)
		NULL /* TODO */,
		&g_ChildTableLengthClass80,
		(child_record_t *)&g_ChildTableClass80,
		sizeof(class80_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},
	/* 81 - PRIME NB OFDM PLC Physical layer counters */
	{
		0, 5, 1,
#if (defined(USED_CLASS_81) && USED_CLASS_81 == 1)
		NULL /* TODO */,
		&g_ChildTableLengthClass81,
		(child_record_t *)&g_ChildTableClass81,
		sizeof(class81_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},
	/* 82 - PRIME NB OFDM PLC MAC setup */
	{
		0, 8, 0,
#if (defined(USED_CLASS_82) && USED_CLASS_82 == 1)
		NULL /* TODO */,
		&g_ChildTableLengthClass82,
		(child_record_t *)&g_ChildTableClass82,
		sizeof(class82_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},
	/* 83 - PRIME NB OFDM PLC MAC functional parameters */
	{
		0, 14, 0,
#if (defined(USED_CLASS_83) && USED_CLASS_83 == 1)
		NULL /* TODO */,
		&g_ChildTableLengthClass83,
		(child_record_t *)&g_ChildTableClass83,
		sizeof(class83_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},
	/* 84 - PRIME NB OFDM PLC MAC counters */
	{
		0, 7, 1,
#if (defined(USED_CLASS_84) && USED_CLASS_84 == 1)
		NULL /* TODO */,
		&g_ChildTableLengthClass84,
		(child_record_t *)&g_ChildTableClass84,
		sizeof(class84_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},
	/* 85 - PRIME NB OFDM PLC MAC network administration data */
	{
		0, 6, 1,
#if (defined(USED_CLASS_85) && USED_CLASS_85 == 1)
		NULL /* TODO */,
		&g_ChildTableLengthClass85,
		(child_record_t *)&g_ChildTableClass85,
		sizeof(class85_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},
	/* 86 - PRIME NB OFDM PLC Applications identification */
	{
		0, 4, 0,
#if (defined(USED_CLASS_86) && USED_CLASS_86 == 1)
		NULL /* TODO */,
		&g_ChildTableLengthClass86,
		(child_record_t *)&g_ChildTableClass86,
		sizeof(class86_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},
	/* 87 - (reserved or empty) */
	{0, 0, 0, NULL, NULL, NULL, 0},
	/* 88 - (reserved or empty) */
	{0, 0, 0, NULL, NULL, NULL, 0},
	/* 89 - (reserved or empty) */
	{0, 0, 0, NULL, NULL, NULL, 0},
	/* 90 - G3 NB OFDM PLC MAC layer counters */
	{
		0, 10, 1,
#if (defined(USED_CLASS_90) && USED_CLASS_90 == 1)
		NULL /* TODO */,
		&g_ChildTableLengthClass90,
		(child_record_t *)&g_ChildTableClass90,
		sizeof(class90_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},
	/* 91 - G3 NB OFDM PLC MAC setup */
	{
		0, 11, 1,
#if (defined(USED_CLASS_91) && USED_CLASS_91 == 1)
		NULL /* TODO */,
		&g_ChildTableLengthClass91,
		(child_record_t *)&g_ChildTableClass91,
		sizeof(class91_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},
	/* 92 - G3 NB OFDM PLC 6LoWPAN adaptation layer setup */
	{
		0, 16, 0,
#if (defined(USED_CLASS_92) && USED_CLASS_92 == 1)
		NULL /* TODO */,
		&g_ChildTableLengthClass92,
		(child_record_t *)&g_ChildTableClass92,
		sizeof(class92_child_record_t),
#else
		NULL, NULL, NULL, 0,
#endif
	},
	/* 93 - (reserved or empty) */
	{0, 0, 0, NULL, NULL, NULL, 0},
	/* 94 - (reserved or empty) */
	{0, 0, 0, NULL, NULL, NULL, 0},
	/* 95 - (reserved or empty) */
	{0, 0, 0, NULL, NULL, NULL, 0},
	/* 96 - (reserved or empty) */
	{0, 0, 0, NULL, NULL, NULL, 0},
	/* 97 - (reserved or empty) */
	{0, 0, 0, NULL, NULL, NULL, 0},
	/* 98 - (reserved or empty) */
	{0, 0, 0, NULL, NULL, NULL, 0},
	/* 99 - (reserved or empty) */
	{0, 0, 0, NULL, NULL, NULL, 0},
	/* 100 - (reserved or empty) */
	{0, 0, 0, NULL, NULL, NULL, 0},	
	
	/* 101 - ZigBee SAS Startup */
	{0, 12, 0, 
#if (defined(USED_CLASS_101) && USED_CLASS_101 == 1)
	R_OBIS_DistributeZigBeeSASStartupClass,
	&g_ChildTableLengthClass101,
	(child_record_t *)&g_ChildTableClass101,
	sizeof(class101_child_record_t),	
#else
	NULL, NULL, NULL, 0,
#endif	
	},
	
	/* 102 - ZigBee SAS Join */
	{0, 5, 0, 
#if (defined(USED_CLASS_102) && USED_CLASS_102 == 1)
	R_OBIS_DistributeZigBeeSASJoinClass,
	&g_ChildTableLengthClass102,
	(child_record_t *)&g_ChildTableClass102,
	sizeof(class102_child_record_t),	
#else
	NULL, NULL, NULL, 0,
#endif	
	},

	/* 103 - ZigBee SAS APS Fragment */
	{0, 3, 0, 
#if (defined(USED_CLASS_103) && USED_CLASS_103 == 1)
	R_OBIS_DistributeZigBeeSASAPSFragmentClass,
	&g_ChildTableLengthClass103,
	(child_record_t *)&g_ChildTableClass103,
	sizeof(class103_child_record_t),	
#else
	NULL, NULL, NULL, 0,
#endif	
	},

	/* 104 - ZigBee Network Control */
	{0, 4, 11, 
#if (defined(USED_CLASS_104) && USED_CLASS_104 == 1)
	R_OBIS_DistributeZigBeeNetworkControlClass,
	&g_ChildTableLengthClass104,
	(child_record_t *)&g_ChildTableClass104,
	sizeof(class104_child_record_t),	
#else
	NULL, NULL, NULL, 0,
#endif	
	},

	/* 105 - ZigBee Tunnel Setup */
	{0, 5, 0, 
#if (defined(USED_CLASS_105) && USED_CLASS_105 == 1)
	R_OBIS_DistributeZigBeeTunnelSetupClass,
	&g_ChildTableLengthClass105,
	(child_record_t *)&g_ChildTableClass105,
	sizeof(class105_child_record_t),	
#else
	NULL, NULL, NULL, 0,
#endif	
	},	
};

/* Variables to control the master table */
const Unsigned16 g_TotalMasterTableElements = sizeof(g_MasterTable) / sizeof(master_record_t);

/******************************************************************************
* Function Name : R_OBIS_ObjectLayerInit
* Interface     : void R_OBIS_ObjectLayerInit(void)
* Description   : Object layer initialize
* Arguments     : None
* Function Calls: None
* Return Value  : None
******************************************************************************/
void R_OBIS_ObjectLayerInit(void)
{
	Unsigned8  i;								/* Counter */
	Unsigned16 master_index, child_index;		/* Master & child index to access object */
	Unsigned16 counter;							/* Count number of object */

	/* Reset to start counting */
	g_TotalNumberObjects = 0;

	/* Update number of object for each associations */
	for (i = 0; i < OBIS_NUMBER_ASSOCIATION; i++)
	{
		/* Initial counting */
		master_index = 0;
		counter      = 0;

		/* Loop to count number of object */
		while (master_index < g_TotalMasterTableElements)
		{
			/* Ignorge all NULL or empty row */
			if (g_MasterTable[master_index].child_list_pointer == NULL ||
				g_MasterTable[master_index].child_list_size == NULL ||
				(g_MasterTable[master_index].child_list_size != NULL &&
				 *g_MasterTable[master_index].child_list_size == 0))
			{
				master_index++;
				continue;
			}

			/* Update total number of object on first checking */
			if (i == 0)
			{
				g_TotalNumberObjects += *g_MasterTable[master_index].child_list_size;
			}

			/*
			 * Check the object in the row until it end
			 */
			for (	child_index = 0;
					child_index < *(g_MasterTable[master_index].child_list_size);
					child_index++	)
			{
				/* Check whether the current association can access or not */
				if (R_OBIS_CheckObjectVisibility(
						i,
						master_index,
						child_index
					) == OBIS_AR_OBJ_IS_VISIBLE)
				{
					counter++;
				}
			}

			/* Next master row */
			master_index++;
		}

		/* Update the number of object for association */
		g_TotalNumberObjectsOfAssociation[i] = counter;
	}

}

/******************************************************************************
* Function Name : R_OBIS_IsAccessible
* Interface     : static Unsigned8 R_OBIS_IsAccessible(
*               :     req_type_t req_type,
*               :     Unsigned8 access_right_value
*               : );
* Description   : Check whether the attribute or method access right
*               : is accessible or not.
* Arguments     : req_type_t req_type -
*               :   Request type on object (GET/SET/ACTION).
*               : Unsigned8 access_right_value -
*               :   Access right value on the access_right_t (attribute or method).
* Function Calls: None
* Return Value  : static Unsigned8
*               :   Indicate whether the object can be accessed (1) or not (0)
******************************************************************************/
static Unsigned8 R_OBIS_IsAccessible(req_type_t req_type, Unsigned8 access_right_value)
{
	if (req_type == REQ_TYPE_GET)
	{
		return (access_right_value == OBIS_AR_ATTR_READ				||
				access_right_value == OBIS_AR_ATTR_READ_WRITE		);
	}
	else if (req_type == REQ_TYPE_SET)
	{
		return (access_right_value == OBIS_AR_ATTR_WRITE			||
				access_right_value == OBIS_AR_ATTR_READ_WRITE		);
	}
	else if (req_type == REQ_TYPE_ACTION)
	{
		return (access_right_value == OBIS_AR_METHOD_ACCESS			);
	}

	return 0;	/* Denied */
}

/******************************************************************************
* Function Name : R_OBIS_CheckAccessRight
* Interface     : static Unsigned8 R_OBIS_CheckAccessRight(
*               :     req_type_t req_type,
*               :     Unsigned8 req_id,
*               :     access_right_t *access_right
*               : );
* Description   : Check access right of attribute or method, based on its id.
* Arguments     : req_type_t req_type -
*               :    Request type (GET/SET/ACTION)
*               : Unsigned8 req_id -
*               :    Request id, attribute id or method id
*               : p_access_right_t * access_right -
*               :    Access right of the object in associated connection
* Function Calls: R_OBIS_GetAccessRightValue()
*               : R_OBIS_IsAccessible()
* Return Value  : static Unsigned8
*               :   Indicate whether the object can be accessed (1) or not (0)
******************************************************************************/
static Unsigned8 R_OBIS_CheckAccessRight(
	req_type_t		req_type,
	Unsigned8		req_id,
	access_right_t 	*p_access_right
)
{
	Unsigned8 access_right_value;

	/*
	 * Check input parameters
	 * Currently, max. of req_id for attribute and method is 16.
	 */
	if (req_id > 16)
	{
		return 0;	/* Denied */
	}

	/*
	 * Full access (Read, write...) when not specify access right
	 */
	if (p_access_right == NULL)
	{
		return 1;	/* Full access */
	}

	/* Get the access_right_value by I/F */
	access_right_value =
		R_OBIS_GetAccessRightValue(
			req_type,
			req_id,
			p_access_right
		);

	return R_OBIS_IsAccessible(req_type, access_right_value);
}

/******************************************************************************
* Function Name   : R_OBIS_CompareObisCode
* Interface       : static Unsigned8 R_OBIS_CompareObisCode(
*                 :     Unsigned8 *obis_code,
*                 :     Unsigned8 *input_code
*                 : );
* Description     : Compare OBIS Code
* Arguments       : Unsigned8 * obis_code : Source obis code (in obis list)
*                 : Unsigned8 * input_code: Input obis code (from request)
* Function Calls  : None
* Return Value    : static Unsigned8, result of comparision
*                 :     0    means un-match
*                 :     1    means matched.
******************************************************************************/
Unsigned8 R_OBIS_CompareObisCode(Unsigned8 *obis_code, Unsigned8 *input_code)
{
	Unsigned8 i;

	/*
     * Compare for first 5 byte only
     * The f element present the selection (VZ or historic value)
     */
	for (i = 0; i < 5; i++)
	{
		if (*obis_code++ != *input_code++)
		{
			return 0;	/* Un-matched */
		}
	}

	return 1;	/* Matched */
}

/******************************************************************************
* Function Name : R_OBIS_FindChildRecordByIndex
* Interface     : child_record_t *R_OBIS_FindChildRecordByIndex(
*               :     Unsigned16 master_index,
*               :     Unsigned16 child_index
*               : );
* Description   : Get a child record, based on master index and child index
* Arguments     : Unsigned16 master_index: Master index
*               : Unsigned16 child_index : Child index
* Function Calls: None
* Return Value  : child_record_t *, pointer of a child record
*               : NULL if not found
******************************************************************************/
child_record_t *R_OBIS_FindChildRecordByIndex(
	Unsigned16 master_index,
	Unsigned16 child_index
)
{
	master_record_t	*p_master_record;	/* Pointer to master record in g_MasterTable */
	child_record_t	*p_child_record;	/* Pointer to child record in a specified master record */

	if (master_index >= g_TotalMasterTableElements)
	{
		return NULL;	/* master index is out of range */
	}

	/*
	 * Find in master table first,
	 * if not found master list, the object is undefine
	 */
	p_master_record = (master_record_t	*)(g_MasterTable + master_index);
	if (p_master_record == NULL ||
		p_master_record->child_list_size == NULL ||
		p_master_record->child_list_pointer == NULL ||
		(p_master_record->child_list_size != NULL &&
		 *(p_master_record->child_list_size) == 0))
	{
		return NULL;	/* Not found the master row */
	}

	/*
	 * If found master record, next find object in child table by index
	 */
	if (child_index >= *(p_master_record->child_list_size))
	{
		return NULL;
	}

	p_child_record = (child_record_t *)(
		((Unsigned8 *)p_master_record->child_list_pointer) +
		(child_index * p_master_record->child_item_size)
	);

	return (p_child_record);
}

/******************************************************************************
* Function Name : R_OBIS_CheckObjectVisibility
* Interface     : Unsigned8 R_OBIS_CheckObjectVisibility(
*               :     Unsigned16 association_number,
*               :     Unsigned16 master_index,
*               :     Unsigned16 child_index,
*               : );
* Description   : Check object visibility for a specified association
* Arguments     : Unsigned16 association_number
*               :    - Association number
*               : Unsigned16 master_index
*               :    - Master id (class id) of object
*               : Unsigned16 child_index
*               :    - Child id of object
* Function Calls: None
* Return Value  : Unsigned8,
*               :     OBIS_AR_OBJ_IS_INVISIBLE    Object is invisible
*               :     OBIS_AR_OBJ_IS_VISIBLE      Object is visible
******************************************************************************/
Unsigned8 R_OBIS_CheckObjectVisibility(
	Unsigned16 association_number,
	Unsigned16 master_index,
	Unsigned16 child_index
)
{
	child_record_t *p_child_record;

	/*
	 * Find OBIS object in child table by
	 * visibility, obis code, access right.
	 */
	p_child_record = R_OBIS_FindChildRecordByIndex(master_index, child_index);
	if (p_child_record == NULL)
	{
		return OBIS_AR_OBJ_IS_INVISIBLE;		/* Not found the object */
	}

	/*
	 * Check the access_right object by the association number
	 */
	if (p_child_record->p_access_right == NULL)
	{
		return OBIS_AR_OBJ_IS_VISIBLE;		/* OK, object is visible */
	}
	else if (p_child_record->p_access_right[association_number].attributes != 0 ||
		     p_child_record->p_access_right[association_number].methods != 0)
	{
		return OBIS_AR_OBJ_IS_VISIBLE;		/* OK, object is visible */
	}
	else
	{
		return OBIS_AR_OBJ_IS_INVISIBLE;
	}
}

/******************************************************************************
* Function Name   : R_OBIS_GetAccessRightValue
* Interface       : Unsigned16 R_OBIS_GetAccessRightValue(
*                 :     req_type_t req_type,
*                 :     Unsigned8 req_id,
*                 :     access_right_t *p_access_right
*                 : );
* Description     : Get access right value for object
* Arguments       : req_type_t req_type -
*                 :     Requested type on object (GET/SET/ACTION)
*                 : Unsigned8 req_id -
*                 :     Requested id on object (attribute id or method id)
*                 : access_right_t * p_access_right -
*                 :     Access right pointer of object
* Function Calls  : None
* Return Value    : Unsigned8, access right value as defined by macros in
*                 : r_dlms_obis_objects.h
******************************************************************************/
Unsigned8 R_OBIS_GetAccessRightValue(
	req_type_t 		req_type,
	Unsigned8 		req_id,
	access_right_t 	*p_access_right
)
{
	Unsigned8 access_right_value = 0;	/* Initial as no access */

	/* Check req_type value */
	if (req_type != REQ_TYPE_GET &&
		req_type != REQ_TYPE_SET &&
		req_type != REQ_TYPE_ACTION)
	{
		return 0;	/* No access */
	}

	/* When access right pointer = NULL, means full access */
	if (p_access_right == NULL &&
		req_id <= 16)
	{
		return (	req_type == REQ_TYPE_ACTION ?
					OBIS_AR_METHOD_ACCESS : OBIS_AR_ATTR_READ_WRITE	);
	}

	/* Get the access_right_value base on req_id & req_type */
	/* For Attribute? */
	if (req_type == REQ_TYPE_GET || req_type == REQ_TYPE_SET)
	{
		/*
		 * For attribute only,
		 * we propose that attribute 0 can be read/write for all association
		 */
		if (req_id == 0)
		{
			access_right_value = OBIS_AR_ATTR_READ_WRITE;
		}
		else if (req_id <= 16)
		{
			access_right_value = (Unsigned8)(
				(p_access_right->attributes >> ((req_id - 1) * 2)) & 0x03
			);
		}
		else
		{
			access_right_value = OBIS_AR_ATTR_NO_ACCESS;
		}
	}
	else
	{
		/*
		 * For method 0, denied
		 */
		if (req_id == 0)
		{
			access_right_value = OBIS_AR_METHOD_NO_ACCESS;
		}
		else if (req_id <= 16)
		{
			access_right_value = (Unsigned8)(
				(p_access_right->methods >> (req_id - 1)) & 0x01
			);
		}
		else
		{
			access_right_value = OBIS_AR_METHOD_NO_ACCESS;
		}
	}

	return (access_right_value);
}

/******************************************************************************
* Function Name   : R_OBIS_GetConnectedAssociation
* Interface       : Unsigned8 R_OBIS_GetConnectedAssociation(void)
* Description     : Get the connected association (refer info from app. layer)
* Arguments       : None
* Function Calls  : None
* Return Value    : Unsigned8, current connected association id
******************************************************************************/
Unsigned8 R_OBIS_GetConnectedAssociation(void)
{
	return COSEMOpenGetConnectedAssociation();
}

/******************************************************************************
* Function Name : R_OBIS_DecodeObject
* Interface     : Unsigned8 R_OBIS_DecodeObject(
*               :     st_Cosem_Attr_Desc *cosem_attr_desc,
*               :     Unsigned8 *pdata,
*               :     Unsigned16 service_code
*               : );
* Description   : Decode an object attribute
* Arguments     : st_Cosem_Attr_Desc * cosem_attr_desc -
*               :     COSEM Attribute Descriptor
*               : Unsigned8 * pdata                   -
*               :     Data pointer, in case of SET/ACTION service
*               : Unsigned16 service_code             -
*               :     Service code of the request
* Function Calls: R_OBIS_CompareObisCode()
*               : R_OBIS_CheckAccessRight()
*               : (decoded class)->fp_distributor()
* Return Value  : Unsigned8
******************************************************************************/
Unsigned8 R_OBIS_DecodeObject(
	st_Cosem_Attr_Desc 	*cosem_attr_desc,
	Unsigned8 			*pdata,
	Unsigned16 			service_code
)
{
	master_record_t		*p_master_record;	/* Pointer to master record in g_MasterTable */
	child_record_t		*p_child_record;	/* Pointer to child record in a specified master record */
	fp_distributor_t	pfunc;				/* Function pointer of the distributor (class or object) */
	
	Unsigned8 			i;					/* Loop counter */
	Unsigned8			status;				/* Return status of distributor */
	req_type_t			req_type;			/* Request type */
	service_type_t		service_type;		/* Service type */
	Unsigned16			req_class_id;		/* Class id inside the request */
	Unsigned8			connected_assc_id;	/* Current connected assc id */

	/* Get class id from request */
	req_class_id = (
		((Unsigned16)cosem_attr_desc->Class_ID[0] << 8) |
		((Unsigned16)cosem_attr_desc->Class_ID[1])
	);

	/*
	 * Find in master table first,
	 * if not found master list, the object is undefine
	 */
	/* Access the master table */
	p_master_record = (master_record_t *)(g_MasterTable + req_class_id);
	if (p_master_record == NULL ||
		p_master_record->child_list_size == NULL ||
		p_master_record->child_list_pointer == NULL ||
		(p_master_record->child_list_size != NULL &&
		 *p_master_record->child_list_size == 0))
	{
		return OBJ_UNDEFINE;		/* Not found the object */
	}

	/* Get the current connected assc */
	connected_assc_id = R_OBIS_GetConnectedAssociation();
	
	/* Get request type (GET/SET/ACTION) */
	switch (service_code >> 8)
	{
		case OL_GET_REQUEST:
			req_type = REQ_TYPE_GET;
			break;
			
		case OL_SET_REQUEST:
			req_type = REQ_TYPE_SET;
			break;
		
		case OL_ACTION_REQUEST:
			req_type = REQ_TYPE_ACTION;
			break;
			
		default:
			return OBJ_UNDEFINE;	/* Object not found */
			//break;
	}
	
	/* Get service type (NORMAL/BLOCK/WITH-LIST) */
	if (req_type == REQ_TYPE_GET)
	{
		switch (service_code & 0xFF)
		{
			case OL_REQ_SERVICE_NORMAL:
				service_type = SERVICE_TYPE_NORMAL;
				break;
			
			case OL_REQ_SERVICE_BLOCK:
				service_type = SERVICE_TYPE_BLOCK;
				break;
				
			case OL_REQ_SERVICE_LIST:
				service_type = SERVICE_TYPE_LIST;
				break;
				
			default:
				return SCOPE_ACCESS_VIOLATED;		/* Violate scope */
				//break;
		}
	}
	
	/* Check wheter the connected association (0.0.40.0.0.255) is request or not */
	if (R_OBIS_CompareObisCode(
			cosem_attr_desc->Instance_ID,
			(Unsigned8 *)"\x0\x0\x28\x0\x0\xFF"
		) == 1)
	{
		/* Get the record of current connected assc id in master list */
		i = connected_assc_id;
		p_child_record = (child_record_t *)(
			((Unsigned8 *)p_master_record->child_list_pointer) +
			(i * p_master_record->child_item_size)
		);
	}
	else
	{
		/*
		 * If found master record, next find OBIS object in child table by
		 * visibility, obis code, access right.
		 */
		for (i = 0; i < *p_master_record->child_list_size; i++)
		{
			/* Get the child item */
			p_child_record = (child_record_t *)(
				((Unsigned8 *)p_master_record->child_list_pointer) +
				(i * p_master_record->child_item_size)
			);

			/*
			 * Check the object by the current connected association
			 */
			if (
				/* found object? */
				p_child_record != NULL &&
				
				/* Visible? */
				(p_child_record->p_access_right == NULL ||
				 (p_child_record->p_access_right != NULL &&
				  (p_child_record->p_access_right[connected_assc_id].attributes != 0 ||
				   p_child_record->p_access_right[connected_assc_id].methods != 0))) &&
				   
				/* Match OBIS code? */
				R_OBIS_CompareObisCode(p_child_record->logical_name, cosem_attr_desc->Instance_ID) == 1
				
				)
			{
				break;
			}
		}
	
		/* If not found, return error */
		if (i >= *p_master_record->child_list_size)
		{
			return OBJ_UNDEFINE;		/* Object not found */
		}
	
		/*
		 * Check the access right for the current connected association
		 */
		if (R_OBIS_CheckAccessRight(
				req_type,
				(Unsigned8)cosem_attr_desc->Attr_ID,
				&p_child_record->p_access_right[connected_assc_id]) == 0)
		{
			return SCOPE_ACCESS_VIOLATED;		/* Violate scope */
		}
	
	}
	
	/*
	 * Restrict the access to reply_to_HLS_authentication of Association LN object
	 * when HLS authentication is required in current connected association
	 */
	if (g_ChildTableClass15[connected_assc_id].mechanism_id == MECHANISM_ID2_HIGH_SECURITY ||
		g_ChildTableClass15[connected_assc_id].mechanism_id == MECHANISM_ID3_HIGH_SECURITY_MD5 ||
		g_ChildTableClass15[connected_assc_id].mechanism_id == MECHANISM_ID4_HIGH_SECURITY_SHA1 ||
		g_ChildTableClass15[connected_assc_id].mechanism_id == MECHANISM_ID5_HIGH_SECURITY_GMAC)
	{
		/* pointer of authen status is NULL? */
		if (g_ChildTableClass15[connected_assc_id].p_authen_status == NULL)
		{
			return SCOPE_ACCESS_VIOLATED;		/* Violate scope */ 
		}
		
		/* Not finish authentication and try to access to other than reply_to_HLS_authentication? */
		if (*(g_ChildTableClass15[connected_assc_id].p_authen_status) != AUTHEN_STATUS_SUCCESS)
		{
			if (req_type != REQ_TYPE_ACTION)
				{
					return SCOPE_ACCESS_VIOLATED;		/* Violate scope */
				}
			else if((Unsigned8)cosem_attr_desc->Attr_ID != 1)
				{
					return SCOPE_ACCESS_VIOLATED;		/* Violate scope */
				}
		}
	}
	
	/*
	 * Get the function pointer to prepare for the distribution
	 * to class/object
	 */
	pfunc = p_master_record->fp_distributor;

	/* Every thing is OK, call to distributor */
	if (pfunc != NULL)
	{
		/* distribution */
		status = (*pfunc)(
			req_class_id,					/* [In] Id of the row in master list */
			i,                              /* [In] Id of the row in child list */
			req_type,                       /* [In] Request type (GET/SET/ACTION) */
			service_type,					/* [In] Service type (NORMAL/BLOCK/LIST) */
			cosem_attr_desc,                /* [In] COSEM object descriptor */
			pdata,                          /* [In] Data in case of SET/ACTION */
			g_ServiceBuffer,                /* [Out] Output buffer for encoded data */
			&OBIS_DataLength                /* [Out] Output buffer length for encoded data */
		);
	}
	else
	{
		return SCOPE_ACCESS_VIOLATED;		/* Violate scope */
	}

	return (status);
}

/******************************************************************************
* Function Name : COSEM_Getinfo
* Interface     : Unsigned8* COSEM_Getinfo(
*               :     Unsigned16 Class_ID,
*               :     Unsigned16 index
*               :     Unsigned16 Data_ID
*               : );
* Description   : Get COSEM info
* Arguments     : Unsigned16 Class_ID
*               : Unsigned16 index: index in class
*               : Unsigned16 Data_ID: specific data ID
* Function Calls: T.B.D
* Return Value  : void* ; NULL is error
******************************************************************************/
/* Return value */
Unsigned8 result_u8 = 0;
Unsigned8* COSEM_Getinfo(Unsigned16 Class_ID, Unsigned16 index, Unsigned16 Data_ID)
{
	


		switch (Data_ID)
		{
#if (defined(USED_CLASS_15) && USED_CLASS_15 == 1)
			case COSEM_DATAID_CONNECTAASTATUS:
				{
					Unsigned8			connected_assc_id;
					/* Get the current connected assc */
					connected_assc_id = R_OBIS_GetConnectedAssociation();
					if (connected_assc_id < g_ChildTableLengthClass15 &&
						g_ChildTableClass15[connected_assc_id].p_status != NULL)
					{
						return (Unsigned8 *)(g_ChildTableClass15[connected_assc_id].p_status);
					}
				}
				break;

			case  COSEM_DATAID_CLIENTSAP:
				{
					return (Unsigned8 *)&(g_ChildTableClass15[index].partners_id.client_SAP);
				}
				//break;

			case COSEM_DATAID_CHILDTABLE15LEN:
				{
					return (Unsigned8 *)&g_ChildTableLengthClass15;
				}
				//break;

			case COSEM_DATAID_CONTEXTID:
				{
					return (Unsigned8 *)&(g_ChildTableClass15[index].context_id);
				}
				//break;

			case COSEM_DATAID_MECHANISMID:
				{
					return (Unsigned8 *)&(g_ChildTableClass15[index].mechanism_id);
				}
				//break;
	
			case COSEM_DATAID_SECRETKEY:
				{
					return (Unsigned8 *)(g_ChildTableClass15[index].p_secret_key);
				}
				//break;
			case COSEM_DATAID_SHAREDKEY:
				{
					return (Unsigned8 *)(g_ChildTableClass15[index].p_shared_key);
				}
			//break;
			case COSEM_DATAID_DLMSVERSION:
				{
					return (Unsigned8 *)&(g_ChildTableClass15[index].dlms_version);
				}
				//break;
			case COSEM_DATAID_CONFORMANCEBLK0:
				{
					return (Unsigned8 *)&(g_ChildTableClass15[index].conformance.bytes.bytes0);
				}
				//break;
			case COSEM_DATAID_CONFORMANCEBLK1:
				{
					return (Unsigned8 *)&(g_ChildTableClass15[index].conformance.bytes.bytes1);
				}
				//break;
			case COSEM_DATAID_CONFORMANCEBLK2:
				{
					return (Unsigned8 *)&(g_ChildTableClass15[index].conformance.bytes.bytes2);
				}
				//break;
#if (defined(USED_CLASS_64) && USED_CLASS_64 == 1)
			case COSEM_DATAID_SECURITYSETUP_ID_OF_CLASS15:
				{
						static Unsigned16 i;
						//Unsigned16 j;

						if (index < g_ChildTableLengthClass15 &&
							g_ChildTableClass15[index].p_security_setup_ref != NULL)
						{
							/* Search the child item in security setup class list */
							for (i = 0; i < g_ChildTableLengthClass64; i++)
							{
								if (memcmp(
										(void *)(g_ChildTableClass15[index].p_security_setup_ref),
										(void *)&(g_ChildTableClass64[i].logical_name[0]),
										6
									) == 0)
								{
									result_u8 = i;
									return (Unsigned8 *)&result_u8;
								}
							}
						}
						
						return NULL;
				}
				break;
			case COSEM_DATAID_CLIENTITTLE_OF_CLASS64:
				{
					return (Unsigned8 *)(g_ChildTableClass64[index].p_client_title);
				}
				break;
			case COSEM_DATAID_SERVERTITTLE_OF_CLASS64:
				{
					return (Unsigned8 *)(g_ChildTableClass64[index].p_server_title);
				}
				break;
			case COSEM_DATAID_SECURITYPOLICY_OF_CLASS64:
				{
					return (Unsigned8 *)(g_ChildTableClass64[index].p_security_policy);
				}
				break;
			case COSEM_DATAID_DEDICATED_UNICASTKEY_OF_CLASS64:
				{
					return (Unsigned8 *)(g_ChildTableClass64[index].p_dedicated_unicast_key);
				}
				break;
			case COSEM_DATAID_GLOBAL_UNICASTKEY_OF_CLASS64:
				{
					return (Unsigned8 *)(g_ChildTableClass64[index].p_global_unicast_key);
				}
				break;
			case COSEM_DATAID_GLOBAL_BROADCASTKEY_OF_CLASS64:
				{
					return (Unsigned8 *)(g_ChildTableClass64[index].p_global_broadcast_key);
				}
				break;
			case COSEM_DATAID_GLOBAL_AUTHKEY_OF_CLASS64:
				{
					return (Unsigned8 *)(g_ChildTableClass64[index].p_authentication_key);
				}
				break;
			case COSEM_DATAID_GLOBAL_SECURITYSUIT_OF_CLASS64:
				{
					return (Unsigned8 *)(g_ChildTableClass64[index].p_security_suit);
				}
				break;
			case COSEM_DATAID_GLOBAL_FRAMECOUNTER_OF_CLASS64:
				{
					return (Unsigned8 *)(g_ChildTableClass64[index].p_frame_counter);
				}
				break;
#endif /* (defined(USED_CLASS_64) && USED_CLASS_64 == 1) */

#endif /* (defined(USED_CLASS_15) && USED_CLASS_15 == 1) */
				case COSEM_DATAID_NUMBER_ATTR_OF_CLASS:
				{
					return (Unsigned8 *)&(g_MasterTable[Class_ID].number_attrs);
				}
				//break;

			default:
				break;
		}



	return NULL;
}

/******************************************************************************
* Function Name : COSEM_Setinfo
* Interface     : void* COSEM_Setinfo(
*               :     Unsigned16 Class_ID,
*               :     Unsigned16 index
*               :     Unsigned16 Data_ID
*               :     Unsigned8 data_value
*               : );
* Description   : Set COSEM info
* Arguments     : Unsigned16 Class_ID
*               : Unsigned16 index: index in class
*               : Unsigned8 Data_ID: specific data ID
*               : Unsigned8 data_value: input patern
* Function Calls: T.B.D
* Return Value  : void* ; NULL is error
******************************************************************************/
void COSEM_Setinfo(Unsigned16 Class_ID, Unsigned16 index, Unsigned16 Data_ID, Unsigned8 data_value)
{
#if (defined(USED_CLASS_15) && USED_CLASS_15 == 1)
	switch (Data_ID)
		{
		case COSEM_DATAID_CONNECTAASTATUS:
			{
				if (g_ChildTableClass15[index].p_status != NULL)
				{
				*(g_ChildTableClass15[index].p_status) = (association_status_t)data_value;
				}
			}
			break;
		case COSEM_DATAID_HLSAASTATUS:
			{
				if (g_ChildTableClass15[index].p_authen_status != NULL)
				{
					*(g_ChildTableClass15[index].p_authen_status) = (authentication_status_t)data_value;
				}
			}
			break;
#if (defined(USED_CLASS_64) && USED_CLASS_64 == 1)
		case COSEM_DATAID_CLIENTITTLE_OF_CLASS64:
			{
				memset(g_ChildTableClass64[index].p_client_title, data_value, 8);
			}
			break;
		case COSEM_DATAID_DEDICATED_UNICASTKEY_OF_CLASS64_RESET:
			{
				memset(g_ChildTableClass64[index].p_dedicated_unicast_key, data_value, 16);
			}
			break;
#endif /* (defined(USED_CLASS_64) && USED_CLASS_64 == 1) */
		default:
			break; // do nothing
		}
#endif /* (defined(USED_CLASS_15) && USED_CLASS_15 == 1) */
}
