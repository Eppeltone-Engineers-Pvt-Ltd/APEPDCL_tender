/******************************************************************************
  Copyright (C) 2011 Renesas Electronics Corporation, All Rights Reserved.
*******************************************************************************
* File Name    : r_dlms_typedef.h
* Version      : 1.00
* Description  : DLMS Type Definitions. Declare all types for number represenation
******************************************************************************
* History : DD.MM.YYYY Version Description
*         : 13.07.2012 
******************************************************************************/

#ifndef _R_DLMS_TYPEDEF_H
#define _R_DLMS_TYPEDEF_H

/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/

/******************************************************************************
Macro definitions
******************************************************************************/

/******************************************************************************
Typedef definitions
******************************************************************************/
#ifndef __DLMS_TYPEDEF__
#define __DLMS_TYPEDEF__
typedef signed char		Integer8;
typedef unsigned char		Unsigned8;

typedef signed short		Integer16;
typedef unsigned short		Unsigned16;

typedef signed long 		Integer32;
typedef unsigned long		Unsigned32;

// typedef signed long long Integer64;
// typedef unsigned long long Unsigned64;

/*
 * Only Float32 available on RL78/G13 MCU,
 * double is treated as 32bits single precision
 */
typedef float				Float32;
// typedef double	Float64
#endif

#ifndef NULL
#define NULL	((void *)0)
#endif

/******************************************************************************
Variable Externs
******************************************************************************/

/******************************************************************************
Functions Prototypes
******************************************************************************/

#endif /* _R_DLMS_TYPEDEF_H */

