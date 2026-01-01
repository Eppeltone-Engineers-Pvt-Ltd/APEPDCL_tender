/******************************************************************************
  Copyright (C) 2011 Renesas Electronics Corporation, All Rights Reserved.
*******************************************************************************
* File Name    : typedef.h
* Version      : 1.00
* Description  : GSCE Standard Typedef Header file
******************************************************************************
* History : DD.MM.YYYY Version Description
******************************************************************************/

#ifndef __DEV_TYPEDEF_H
#define __DEV_TYPEDEF_H

/******************************************************************************
Preprocess statement
******************************************************************************/

/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/

/******************************************************************************
Typedef definitions
******************************************************************************/
#ifndef __TYPEDEF__
typedef signed char 		int8_t;
typedef unsigned char 		uint8_t;
typedef signed short		int16_t;
typedef unsigned short 		uint16_t;
typedef signed long			int32_t;
typedef unsigned long		uint32_t;
typedef signed long long	int64_t;
typedef unsigned long long	uint64_t;

#define __TYPEDEF__
#endif
typedef union 
{
	uint8_t RxCount;
	uint8_t TxCount;
	
}UartBuf;
#ifndef __FLOAT_TYPEDEF__
typedef double				double64_t;
typedef float				float32_t;
#define __FLOAT_TYPEDEF__
#endif

/******************************************************************************
Macro definitions
******************************************************************************/
/* NULL pointer */
#ifndef NULL
#define NULL				((void *)(0))
#endif

/******************************************************************************
Variable Externs
******************************************************************************/

/******************************************************************************
Functions Prototypes
******************************************************************************/


#endif /* _EM_TYPEDEF_H */


