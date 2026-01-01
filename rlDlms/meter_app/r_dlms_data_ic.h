/******************************************************************************
  Copyright (C) 2011 Renesas Electronics Corporation, All Rights Reserved.
*******************************************************************************
* File Name    : r_dlms_data_ic.h
* Version      : 1.00
* Description  :
******************************************************************************
* History : DD.MM.YYYY Version Description
*         : 21.02.2013
******************************************************************************/

#ifndef _R_DLMS_DATA_IC_H
#define _R_DLMS_DATA_IC_H

/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#include "..\\r_dlms_typedef.h"				/* DLMS Type Definitions */
//#include "..\\..\\rlApplication\\Include\\AppConfig.h"
//#if (defined(IS_DLMS_ENABLED_EEPL) && (IS_DLMS_ENABLED_EEPL >1))
//#error "invalid macros"
//#endif

/******************************************************************************
Macro definitions
******************************************************************************/

/*
 * Specify number of association will be used
 */
#define OBIS_NUMBER_ASSOCIATION		(3)

/*
 * The current version of DLMS support below classes
 *
 * Please specify which classes want to use
 * 0 is no use, 1 is use
 */
#define USED_CLASS_01		(1) /* Data */
#define USED_CLASS_03		(1) /* Register */
#define USED_CLASS_04		(1) /* Extended Register */
#define USED_CLASS_05		(0) /* Demand Register */
#define USED_CLASS_06		(0) /* Register activation */
#if (defined(IS_DLMS_ENABLED_EEPL) && IS_DLMS_ENABLED_EEPL == 1)
#define USED_CLASS_07		(1) /* Profile generic */
#endif
#if (defined(IS_DLMS_ENABLED_EEPL) && IS_DLMS_ENABLED_EEPL == 0)
#define USED_CLASS_07		(0) /* Profile generic */
#endif
#define USED_CLASS_08		(1) /* Clock */
#define USED_CLASS_09		(1) /* Script table */
#define USED_CLASS_10		(0) /* Schedule */
#define USED_CLASS_11		(0) /* Special days table */
#define USED_CLASS_15		(1) /* Association LN */
#define USED_CLASS_17		(1) /* SAP assignment */
#define USED_CLASS_18		(0) /* Image transfer */
#define USED_CLASS_19		(0) /* IEC local port setup */
#define USED_CLASS_20		(1) /* Activity calendar */
#define USED_CLASS_21		(0) /* Register monitor */
#define USED_CLASS_22		(1) /* Single action schedule */
#define USED_CLASS_23		(1) /* IEC HDLC setup */
#define USED_CLASS_24		(0) /* IEC twisted pair */
#define USED_CLASS_26		(0) /* Utility tables */
#define USED_CLASS_27		(0) /* Modem configuration */
#define USED_CLASS_28		(0) /* Auto answer */
#define USED_CLASS_29		(0) /* Auto connect */
#define USED_CLASS_41		(0) /* TCP UDP setup */
#define USED_CLASS_61		(0) /* Register table */
#define USED_CLASS_63		(0) /* Status mapping */
#define USED_CLASS_64		(1) /* Security setup */
#define USED_CLASS_70		(0) /* Disconnect control */
#define USED_CLASS_71		(0) /* Limiter */
#define USED_CLASS_101		(0) /* ZigBee SAS Startup */
#define USED_CLASS_102		(0) /* ZigBee SAS Join */
#define USED_CLASS_103		(0) /* ZigBee SAS APS Fragment */
#define USED_CLASS_104		(0) /* ZigBee Network Control */
#define USED_CLASS_105		(0) /* ZigBee Tunnel Setup */
/*
 * Choosing supported previous version of DLMS classes
 *** If class has no previous version, current version is 0
 *** If specified below, current version = highest previous version + 1
 * 0 is no use, 1 is use
 * If choice is no use, server default choose latest version in Green Book
 */
#define VERSION0_CLASS_07	(0) /* Version0 of class 07 */
#define VERSION0_CLASS_15	(0) /* Version0 of class 15 */
#define VERSION0_CLASS_19	(0) /* Version0 of class 19 */
#define VERSION0_CLASS_23	(0) /* Version0 of class 23 */
#define VERSION0_CLASS_24	(0) /* Version0 of class 24 */
#define VERSION0_CLASS_27	(0) /* Version0 of class 27 */
#define VERSION0_CLASS_28	(0) /* Version0 of class 28 */
#define VERSION0_CLASS_29	(0) /* Version0 of class 29 */

/*
 * Choosing supported feature for Class 20 Activiry Calendar
 * 0 is no use, 1 is use
 */
 #define USED_CLASS_20_SEASON_PROFILE	(1) /* season_profile_table */
 #define USED_CLASS_20_WEEK_PROFILE		(1) /* week_profile_table */
 #define USED_CLASS_20_DAY_PROFILE		(1) /* day_profile_table */
/******************************************************************************
Typedef definitions
******************************************************************************/

/******************************************************************************
Variable Externs
******************************************************************************/

/******************************************************************************
Functions Prototypes
******************************************************************************/
#define obis_strlen(x)			(sizeof(x)-1)
#define OBIS_BAUD_CONVERT(x)	(x - '0')
 void Get_Time(void);
#endif /* _R_DLMS_DATA_IC_H */

