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
* File Name    : r_dlms_obis_ic.c
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
#include "..\\r_dlms_typedef.h"				/* DLMS Type Definitions */
#include "r_dlms_obis_ic.h"				/* DLMS OBIS IC Definitions */
#include "r_dlms_obis.h"				/* DLMS OBIS Definitions */
#include "..\\meter_app\\r_dlms_data.h"				/* DLMS DATA Definitions */
#include "..\\meter_app\\r_dlms_data_ic.h"				/* DLMS DATA IC Definitions */
#include "..\\meter_app\\r_dlms_user_interface.h"
/* Standard Lib */
#include <string.h>						/* String Standard Lib */

/******************************************************************************
Macro definitions
******************************************************************************/

/******************************************************************************
Typedef definitions
******************************************************************************/

/******************************************************************************
Imported global variables and functions (from other files)


******************************************************************************/
#include "..\\..\\rlApplication\\Include\\AppMisc.h"
extern date_time_t				g_Class20_ac_time;
extern season_profile_t	g_Class20_season_profile;

/******************************************************************************
Exported global variables and functions (to be accessed by other files)
******************************************************************************/

/******************************************************************************
Private global variables and functions
******************************************************************************/
/* Variables */
static Unsigned8  g_OBIS_IsAsyncEnd = 1;			/* Default, Get Service is end */
static Unsigned16 g_OBIS_MasterID = 0;				/* Current OBIS Master list object */
static Unsigned16 g_OBIS_ChildID = 0;				/* Current OBIS Child list object */
static Unsigned16 g_OBIS_CaptureObjectID = 0;		/* Number of object transfer */
#if (defined(USED_CLASS_104) && USED_CLASS_104 == 1)
static Unsigned8  g_OBIS_CurrentActDev = 0;			/* Class104: Current place at active device list */
#endif

Unsigned16 ScriptId[8];
time_t *passive_tod_zone[8], *passive_tod_zone2[8];
date_time_t *season_start_time[2];
Unsigned16 passive_script_id[8],passive_script_id2[8];

 unsigned char active_season;

 unsigned char single_billing_day, single_billing_month, single_billing_year, single_billing_hr, single_billing_min;

/* Functions */
#if (defined(USED_CLASS_15) && USED_CLASS_15 == 1)
Unsigned8 R_OBIS_AsyncStart(Unsigned16 association_number);
Unsigned8 R_OBIS_AsyncScanNext(Unsigned16 association_number);
Unsigned8 R_OBIS_AsyncNextObject(Unsigned16 association_number);
Unsigned8 R_OBIS_AsyncEncodeOneObject(
	Unsigned8  bfirst,
	Unsigned16 association_number,
	Unsigned16 master_index,
	Unsigned16 child_index,
	Unsigned8  *out_buf,
	Unsigned16 *out_buf_len
);
Unsigned8 R_OBIS_ProbeEncodeOneObject(
	Unsigned8  bfirst,
	Unsigned16 association_number,
	Unsigned16 master_index,
	Unsigned16 child_index,
	Unsigned8  *out_buf,
	Unsigned16 *out_buf_len
);
#endif /* (defined(USED_CLASS_15) && USED_CLASS_15 == 1) */

/******************************************************************************
* Function Name : R_OBIS_AsyncRestart
* Interface     : void R_OBIS_AsyncRestart(void)
* Description   : Reset asynchronous transmission service
* Arguments     : None
* Function Calls: None
* Return Value  : None
******************************************************************************/
void R_OBIS_AsyncRestart(void)
{
	g_OBIS_IsAsyncEnd = 1;
	g_OBIS_MasterID = 0;
	g_OBIS_ChildID = 0;
	g_OBIS_CaptureObjectID = 0;

	R_OBIS_DataAsyncServiceRestart();
};

/******************************************************************************
* Function Name : R_OBIS_IntervalProcessing
* Interface     : void R_OBIS_IntervalProcessing(void)
* Description   : Update asynchronous data
* Arguments     : None
* Function Calls: None
* Return Value  : None
******************************************************************************/

#if (defined(USED_CLASS_15) && USED_CLASS_15 == 1)
/******************************************************************************
* Function Name : R_OBIS_AsyncStart
* Interface     : static void R_OBIS_AsyncStart(Unsigned16 association_number)
* Description   : Start the Async service for object_list
* Arguments     : Unsigned16 association_number: Association index
* Function Calls: R_OBIS_CheckObjectVisibility()
* Return Value  : Unsigned8, execution status
******************************************************************************/
Unsigned8 R_OBIS_AsyncStart(Unsigned16 association_number)
{
	Unsigned8 bfound = 0;

	/* Initial master and child index */
	g_OBIS_MasterID = 0;
	g_OBIS_ChildID = 0;

	/* Validate, if fail, scan next */
	bfound = R_OBIS_AsyncScanNext(association_number);

	/* Check whether found the first item or not */
	if (bfound == 1)
	{
		return 1;	/* Error, not have any object */
	}

	/* OK */
	return 0;
}

/******************************************************************************
* Function Name : R_OBIS_AsyncScanNext
* Interface     : static Unsigned8 R_OBIS_AsyncScanNext(Unsigned16 association_number)
* Description   : Validate, if fail then scan next object for in async mode
* Arguments     : None
* Function Calls: None
* Return Value  : static Unsigned8,
*               :    1 is end of async service
*               :    0 is not end
******************************************************************************/
Unsigned8 R_OBIS_AsyncScanNext(Unsigned16 association_number)
{
	Unsigned8 bfound = 0;

	/* Search in all child tables and master table */
	for (;
		 g_OBIS_MasterID < g_TotalMasterTableElements;
		 g_OBIS_MasterID++, g_OBIS_ChildID = 0)
	{
		/*
		 * Search the row that have object
		 * Skip NULL or empty row
		 */
		if (g_MasterTable[g_OBIS_MasterID].child_list_size == NULL ||
			g_MasterTable[g_OBIS_MasterID].child_list_pointer == NULL ||
			(g_MasterTable[g_OBIS_MasterID].child_list_size != NULL &&
			 *g_MasterTable[g_OBIS_MasterID].child_list_size == 0) )
		{
			continue;
		}

		/*
		 * Check the object in the row until it end
		 */
		for (;
			 g_OBIS_ChildID < *(g_MasterTable[g_OBIS_MasterID].child_list_size);
			 g_OBIS_ChildID++)
		{
			/* Check whether the current association can access or not */
			if (R_OBIS_CheckObjectVisibility(
					association_number,
					g_OBIS_MasterID,
					g_OBIS_ChildID
				) == OBIS_AR_OBJ_IS_VISIBLE)
			{
				bfound = 1;
				break;
			}
		}

		/* break immediately if found object */
		if (bfound)
		{
			break;
		}
	}

	return (g_OBIS_MasterID == g_TotalMasterTableElements);
}

/******************************************************************************
* Function Name : R_OBIS_AsyncNextObject
* Interface     : static Unsigned8 R_OBIS_AsyncNextObject(Unsigned16 association_number)
* Description   : Get a next object for in async mode
* Arguments     : None
* Function Calls: None
* Return Value  : static Unsigned8,
*               :    1 is end of async service
*               :    0 is not end
******************************************************************************/
Unsigned8 R_OBIS_AsyncNextObject(Unsigned16 association_number)
{
	/*
	 * Next object in the row, then validate, if fail, scan next
	 * 1 is returned, if all row is scanned through
	 */
	g_OBIS_ChildID++;
	return (R_OBIS_AsyncScanNext(association_number));
}

/******************************************************************************
* Function Name   : R_OBIS_AsyncEncodeOneObject
* Interface       : Unsigned8 R_OBIS_AsyncEncodeOneObject(
*                 :     Unsigned8  bfirst,
*                 :     Unsigned16 association_number,
*                 :     Unsigned16 master_index,
*                 :     Unsigned16 child_index,
*                 :     Unsigned8  *out_buf,
*                 :     Unsigned16 *out_buf_len
*                 : );
* Description     : Get the object_list attribute in asynchronous mode
* Arguments       : Unsigned8 bfirst
*                 :    - Indicate the call is first object.
*                 : Unsigned16 association_number
*                 :    - Number of object of association
*                 : Unsigned16 master_index
*                 :    - Index of master item
*                 : Unsigned16 child_index
*                 :    - Index of child item
*                 : Unsigned8 * out_buf
*                 :    - Buffer to store the output
*                 : Unsigned16 * out_buf_len
*                 :    - Actual used output buffer siz
* Function Calls  : None
* Return Value    : Unsigned8, 0 is OK, 1 is NG (Error)
******************************************************************************/
Unsigned8 R_OBIS_AsyncEncodeOneObject(
	Unsigned8  bfirst,
	Unsigned16 association_number,
	Unsigned16 master_index,
	Unsigned16 child_index,
	Unsigned8  *out_buf,
	Unsigned16 *out_buf_len
)
{
	Unsigned8  		i;						/* Counter */
	Unsigned16 		encoded_length = 0;		/* Total encoded length */

	master_record_t	*p_master_record;		/* Master record */
	child_record_t	*p_child_record;		/* Child record */

	/* Argument checking */
	if (master_index >= g_TotalMasterTableElements ||
		out_buf == NULL ||
		out_buf_len == NULL)
	{
		return 1;		/* Error in parameters */
	}

	/* Find the child record */
	p_master_record = (master_record_t *)(g_MasterTable + master_index);
	if (p_master_record->child_list_size == NULL ||
		*(p_master_record->child_list_size) == 0 ||
		p_master_record->child_list_pointer == NULL)
	{
		return 1;		/* Not found master list */
	}
	p_child_record = (child_record_t *)(
		((Unsigned8 *)p_master_record->child_list_pointer) +
		(child_index * p_master_record->child_item_size)
	);
	if (p_child_record == NULL)
	{
		return 1;		/* Not found child item */
	}

	/*
	 * Collect all object information now!
	 */

	/* First, put the array identifier for first object */
	if (bfirst)
	{
		*out_buf++ = ATTR_TYPE_ARRAY;					/* Array of object_list_type */
		/* Number of elements        */
		i = BERLengthEncode(out_buf, g_TotalNumberObjectsOfAssociation[association_number]);
		out_buf += i;

		encoded_length += i + 1;							/* Already used i + 1 bytes      */
	}

	/* Encode for object_list_element (structure) */
	*out_buf++ = ATTR_TYPE_STRUCTURE;	/* structure of object_list_element */
	*out_buf++ = 4;						/* four elements */
	encoded_length += 2;				/* already used 2 bytes */
	{
		/* 1. Encode for class_id (long unsigned) */
		*out_buf++ = ATTR_TYPE_LONG_UNSIGNED;
		*out_buf++ = (Unsigned8)(master_index >> 8);
		*out_buf++ = (Unsigned8)(master_index & 0xFF);
		encoded_length += 3;

		/* 2. Encode for version (signed) */
		*out_buf++ = ATTR_TYPE_UNSIGNED;
		*out_buf++ = p_master_record->class_version;
		encoded_length += 2;

		/* 3. Encode for logical_name (octet string) */
		*out_buf++ = ATTR_TYPE_OCTET_STRING;
		*out_buf++ = 6;
		for (i = 0; i < 6; i++)
		{
			*out_buf++ = p_child_record->logical_name[i];
		}
		encoded_length += 8;

		/* 4. Encode for access_rights (structure) */
		*out_buf++ = ATTR_TYPE_STRUCTURE;	/* Structure type */
		*out_buf++ = 2;						/* 2 elements */
		encoded_length += 2;
		{
			/* 4.1. Encode for attribute_access_descriptor (array) */
			{
				*out_buf++ = ATTR_TYPE_ARRAY;
				*out_buf++ = p_master_record->number_attrs;	/* number of attributes */
				encoded_length += 2;

				/* (Array of structure) */
				for (i = 1; i <= p_master_record->number_attrs; i++)
				{
					/* Encode for attribute_access_item */
					*out_buf++ = ATTR_TYPE_STRUCTURE;	/* structure of attribute_access_item */
					*out_buf++ = 3;						/* number of elements */
					encoded_length += 2;
					{
						/* (1) Encode for attribute_id */
						*out_buf++ = ATTR_TYPE_INTEGER;
						*out_buf++ = i;
						encoded_length += 2;
						/* (2) Encode for access_mode */

						*out_buf++ = ATTR_TYPE_ENUM;
						*out_buf =
								R_OBIS_GetAccessRightValue(
									REQ_TYPE_GET,	/* Dummy as to get attribute access right */
									i,
									&p_child_record->p_access_right[association_number]
								);
						/* If version 0, limit access_mode value to 0-3 */
						if(g_MasterTable[15].class_version == 0)
						{
							if (*out_buf > 3)
							{
								*out_buf = *out_buf - 3;
							}
							else
							{
								/* Do nothing */
							}
						}	
						else
						{
							/* Do nothing */
						}
						out_buf++;
						encoded_length += 2;
						/* (3) Encode for access_selectors */
						{
							/* Array of null */
							*out_buf++ = ATTR_TYPE_ARRAY;
							*out_buf++ = 0;
							encoded_length += 2;
						}
					}
				}
			}

			/* 4.2. Encode for method_access_descriptor (array) */
			{
				*out_buf++ = ATTR_TYPE_ARRAY;					/* array of method_access_descriptor */
				*out_buf++ = p_master_record->number_methods;	/* number of methods */
				encoded_length += 2;

				/* (Array of structure) */
				for (i = 1; i <= p_master_record->number_methods; i++)
				{
					*out_buf++ = ATTR_TYPE_STRUCTURE;	/* structure of method_access_item */
					*out_buf++ = 2;						/* number of elements */
					encoded_length += 2;
					{
						/* (1) Encode for method_id */
						*out_buf++ = ATTR_TYPE_INTEGER;
						*out_buf++ = i;
						encoded_length += 2;
						
						if(g_MasterTable[15].class_version == 0)
						{
							*out_buf++ = ATTR_TYPE_BOOLEAN;
						}
						else	/* Version 1 */
						{
							*out_buf++ = ATTR_TYPE_ENUM;
						}
						/* (2) Encode for method access_mode */
						*out_buf =
							R_OBIS_GetAccessRightValue(
								REQ_TYPE_ACTION,	/* Dummy as to get method access right */
								i,
								&p_child_record->p_access_right[association_number]
							);
						/* If version 0, limit access_mode value to 0-1 */
						if(g_MasterTable[15].class_version == 0)
						{
							if ( *out_buf > 1)
							{
								*out_buf = *out_buf - 1;
							}
							else
							{
								/* Do nothing */
							}
						}
						else
						{
							/* Do nothing */
						}
						out_buf++;
						encoded_length += 2;
					}
				}
			}
		}
	}

	/* Re-check the length */
	if (encoded_length >= OBIS_SERVICE_DATA_BUFFER_LENGTH)
	{
		*out_buf_len = 0;	/* Reset as error */
		return 1;			/* Error, buffer overflow */
	}

	/* OK, get the out_buf_len */
	*out_buf_len = encoded_length;
	return 0;
}

/******************************************************************************
* Function Name   : R_OBIS_ProbeEncodeOneObject
* Interface       : Unsigned8 R_OBIS_ProbeEncodeOneObject(
*                 :     Unsigned8  bfirst,
*                 :     Unsigned16 association_number,
*                 :     Unsigned16 master_index,
*                 :     Unsigned16 child_index,
*                 :     Unsigned8  *out_buf,
*                 :     Unsigned16 *out_buf_len
*                 : );
* Description     : Probe to get the object_list attribute in asynchronous mode
* Arguments       : Unsigned8 bfirst
*                 :    - Indicate the call is first object.
*                 : Unsigned16 association_number
*                 :    - Number of object of association
*                 : Unsigned16 master_index
*                 :    - Index of master item
*                 : Unsigned16 child_index
*                 :    - Index of child item
*                 : Unsigned8 * out_buf
*                 :    - Buffer to store the output
*                 : Unsigned16 * out_buf_len
*                 :    - Actual used output buffer siz
* Function Calls  : None
* Return Value    : Unsigned8, 0 is OK, 1 is NG (Error)
******************************************************************************/
Unsigned8 R_OBIS_ProbeEncodeOneObject(
	Unsigned8  bfirst,
	Unsigned16 association_number,
	Unsigned16 master_index,
	Unsigned16 child_index,
	Unsigned8  *out_buf,
	Unsigned16 *out_buf_len
)
{
	Unsigned8  		i;						/* Counter */
	Unsigned16 		encoded_length = 0;		/* Total encoded length */

	master_record_t	*p_master_record;		/* Master record */
	child_record_t	*p_child_record;		/* Child record */

	/* Argument checking */
	if (master_index >= g_TotalMasterTableElements ||
		out_buf == NULL ||
		out_buf_len == NULL)
	{
		return 1;		/* Error in parameters */
	}

	/* Find the child record */
	p_master_record = (master_record_t *)g_MasterTable + master_index;
	if (p_master_record->child_list_size == NULL ||
		*(p_master_record->child_list_size) == 0 ||
		p_master_record->child_list_pointer == NULL)
	{
		return 1;		/* Not found master list */
	}
	p_child_record = (child_record_t *)(
		((Unsigned8 *)p_master_record->child_list_pointer) +
		(child_index * p_master_record->child_item_size)
	);
	if (p_child_record == NULL)
	{
		return 1;		/* Not found child item */
	}

	/*
	 * Collect all object information now!
	 */

	/* First, put the array identifier for first object */
	if (bfirst)
	{
		Unsigned8  temp_buf[3];
		// *out_buf++ = ATTR_TYPE_ARRAY;					/* Array of object_list_type */
		/* Number of elements        */
		i = BERLengthEncode(temp_buf, g_TotalNumberObjectsOfAssociation[association_number]);
		// out_buf += i;

		encoded_length += i + 1;							/* Already used i + 1 bytes      */
	}

	/* Encode for object_list_element (structure) */
	// *out_buf++ = ATTR_TYPE_STRUCTURE;	/* structure of object_list_element */
	// *out_buf++ = 4;						/* four elements */
	encoded_length += 2;				/* already used 2 bytes */
	{
		/* 1. Encode for class_id (long unsigned) */
		// *out_buf++ = ATTR_TYPE_LONG_UNSIGNED;
		// *out_buf++ = (Unsigned8)(master_index >> 8);
		// *out_buf++ = (Unsigned8)(master_index & 0xFF);
		encoded_length += 3;

		/* 2. Encode for version (signed) */
		// *out_buf++ = ATTR_TYPE_UNSIGNED;
		// *out_buf++ = p_master_record->class_version;
		encoded_length += 2;

		/* 3. Encode for logical_name (octet string) */
		// *out_buf++ = ATTR_TYPE_OCTET_STRING;
		// *out_buf++ = 6;
		// for (i = 0; i < 6; i++)
		// {
			// *out_buf++ = p_child_record->logical_name[i];
		// }
		encoded_length += 8;

		/* 4. Encode for access_rights (structure) */
		// *out_buf++ = ATTR_TYPE_STRUCTURE;	/* Structure type */
		// *out_buf++ = 2;						/* 2 elements */
		encoded_length += 2;
		{
			/* 4.1. Encode for attribute_access_descriptor (array) */
			{
				// *out_buf++ = ATTR_TYPE_ARRAY;
				// *out_buf++ = p_master_record->number_attrs;	/* number of attributes */
				encoded_length += 2;

				/* (Array of structure) */
				for (i = 1; i <= p_master_record->number_attrs; i++)
				{
					/* Encode for attribute_access_item */
					// *out_buf++ = ATTR_TYPE_STRUCTURE;	/* structure of attribute_access_item */
					// *out_buf++ = 3;						/* number of elements */
					encoded_length += 2;
					{
						/* (1) Encode for attribute_id */
						// *out_buf++ = ATTR_TYPE_INTEGER;
						// *out_buf++ = i;
						encoded_length += 2;
						/* (2) Encode for access_mode */

						// *out_buf++ = ATTR_TYPE_ENUM;
						// *out_buf =
								// R_OBIS_GetAccessRightValue(
									// REQ_TYPE_GET,	/* Dummy as to get attribute access right */
									// i,
									// &p_child_record->p_access_right[association_number]
								// );
						/* If version 0, limit access_mode value to 0-3 */
						// if(g_MasterTable[15].class_version == 0)
						// {
							// if (*out_buf > 3)
							// {
								// *out_buf = *out_buf - 3;
							// }
							// else
							// {
								// /* Do nothing */
							// }
						// }	
						// else
						// {
							// /* Do nothing */
						// }
						// out_buf++;
						encoded_length += 2;
						/* (3) Encode for access_selectors */
						{
							/* Array of null */
							// *out_buf++ = ATTR_TYPE_ARRAY;
							// *out_buf++ = 0;
							encoded_length += 2;
						}
					}
				}
			}

			/* 4.2. Encode for method_access_descriptor (array) */
			{
				// *out_buf++ = ATTR_TYPE_ARRAY;					/* array of method_access_descriptor */
				// *out_buf++ = p_master_record->number_methods;	/* number of methods */
				encoded_length += 2;

				/* (Array of structure) */
				for (i = 1; i <= p_master_record->number_methods; i++)
				{
					// *out_buf++ = ATTR_TYPE_STRUCTURE;	/* structure of method_access_item */
					// *out_buf++ = 2;						/* number of elements */
					encoded_length += 2;
					{
						/* (1) Encode for method_id */
						// *out_buf++ = ATTR_TYPE_INTEGER;
						// *out_buf++ = i;
						encoded_length += 2;
						
						// if(g_MasterTable[15].class_version == 0)
						// {
							// *out_buf++ = ATTR_TYPE_BOOLEAN;
						// }
						// else	/* Version 1 */
						// {
							// *out_buf++ = ATTR_TYPE_ENUM;
						// }
						/* (2) Encode for method access_mode */
						// *out_buf =
							// R_OBIS_GetAccessRightValue(
								// REQ_TYPE_ACTION,	/* Dummy as to get method access right */
								// i,
								// &p_child_record->p_access_right[association_number]
							// );
						/* If version 0, limit access_mode value to 0-1 */
						// if(g_MasterTable[15].class_version == 0)
						// {
							// if ( *out_buf > 1)
							// {
								// *out_buf = *out_buf - 1;
							// }
							// else
							// {
								// /* Do nothing */
							// }
						// }
						// else
						// {
							// /* Do nothing */
						// }
						// out_buf++;
						encoded_length += 2;
					}
				}
			}
		}
	}

	/* Re-check the length */
	if (encoded_length >= OBIS_SERVICE_DATA_BUFFER_LENGTH)
	{
		// *out_buf_len = 0;	/* Reset as error */
		return 1;			/* Error, buffer overflow */
	}

	/* OK, get the out_buf_len */
	*out_buf_len = encoded_length;
	return 0;
}

#endif /* (defined(USED_CLASS_15) && USED_CLASS_15 == 1) */

/******************************************************************************
* Function Name   : R_OBIS_EncodeAttribute
* Interface       : Unsigned16 R_OBIS_EncodeAttribute(
*                 :     Unsigned8   *buf,
*                 :     Unsigned16  buf_len,
*                 :     attr_type_t attr_type,
*                 :     Unsigned8   *value_buf,
*                 :     Unsigned16  value_buf_len
*                 : );
* Description     : Encode data buffer for a specified attribute type
* Arguments       : Unsigned8 * buf          : Output buffer to store the encoded data
*                 : Unsigned16 buf_len       : The length of output buffer
*                 : attr_type_t attr_type    : Specify attribute type want to encode
*                 : Unsigned8 * value_buf    : Buffer that store value need to encode
*                 : Unsigned16 value_buf_len : Specify the length of buffer of value
* Function Calls  : None
* Return Value    : Unsigned16, Actual encoded length
*                 :     0 mean error.
*                 :     > 0 is success, its value is actual used length of [buf]
******************************************************************************/
Unsigned16 R_OBIS_EncodeAttribute(
	Unsigned8 	*buf,
	Unsigned16 	buf_len,
	attr_type_t	attr_type,
	Unsigned8	*value_buf,
	Unsigned16 	value_buf_len
)
{
	Unsigned16 encoded_length = 0;		/* Encoded length, to return the used length */
	Unsigned16 i;						/* Loop counter */
	Unsigned8 temp_len;

	/* Check input parameter */
	if ((buf == NULL) ||
		(buf_len == 0) ||
		(buf_len < value_buf_len))
	{
		return (encoded_length);		/* Not encode anything */
	}

	/* Encode buffer base on select of type */
	switch (attr_type)
	{
		/* Simple Data Types */
		case ATTR_TYPE_NULL_DATA:
			*buf = attr_type;
			encoded_length = 1;
			break;

		/* Optimized for 1 byte types */
		case ATTR_TYPE_BOOLEAN:
		case ATTR_TYPE_BCD:
		case ATTR_TYPE_INTEGER:
		case ATTR_TYPE_UNSIGNED:
		case ATTR_TYPE_ENUM:
			if (value_buf_len == 1)
			{
				*buf++ = attr_type;
				*buf   = *value_buf;
				encoded_length = 2;
			}
			break;

		/* Optimized for 2 bytes types */
		case ATTR_TYPE_LONG:
		case ATTR_TYPE_LONG_UNSIGNED:
			if (value_buf_len == 2)
			{
				*buf++ = attr_type;
				*buf++ = *(value_buf+1);
				*buf   = *(value_buf+0);
				encoded_length = 3;
			}
			break;

		/* Optimized for 4 bytes types */
		case ATTR_TYPE_DOUBLE_LONG:
		case ATTR_TYPE_DOUBLE_LONG_UNSIGNED:
		case ATTR_TYPE_FLOAT32:
			if (value_buf_len == 4)
			{
				*buf++ = attr_type;
				*buf++ = *(value_buf+3);
				*buf++ = *(value_buf+2);
				*buf++ = *(value_buf+1);
				*buf   = *(value_buf+0);
				encoded_length = 5;
			}
			break;

		/* Optimized for 8 bytes types */
		case ATTR_TYPE_LONG64:
		case ATTR_TYPE_LONG64_UNSIGNED:
		case ATTR_TYPE_FLOAT64:
			if (value_buf_len == 8)
			{
				*buf++ = attr_type;
				*buf++ = *(value_buf+7);
				*buf++ = *(value_buf+6);
				*buf++ = *(value_buf+5);
				*buf++ = *(value_buf+4);
				*buf++ = *(value_buf+3);
				*buf++ = *(value_buf+2);
				*buf++ = *(value_buf+1);
				*buf   = *(value_buf+0);
				encoded_length = 9;
			}
			break;

		/* Optimized for variant size */
		case ATTR_TYPE_BIT_STRING:
			*buf++ = attr_type;
			temp_len = BERLengthEncode(buf, (value_buf_len*8)); /* Limit to 16 bit length */
			buf += temp_len;
			for (i = 0; i < value_buf_len; i++)
			{
				*buf++ = *(value_buf + i);
			}
			encoded_length = (value_buf_len + temp_len + 1);
			break;
		case ATTR_TYPE_TIME:
		case ATTR_TYPE_DATE:
		case ATTR_TYPE_DATE_TIME:
		case ATTR_TYPE_OCTET_STRING:
		case ATTR_TYPE_VISIBLE_STRING:
		case ATTR_TYPE_UTF8_STRING:
			*buf++ = attr_type;
			temp_len = BERLengthEncode(buf, value_buf_len); /* Limit to 16 bit length */
			buf += temp_len;
			for (i = 0; i < value_buf_len; i++)
			{
				*buf++ = *(value_buf + i);
			}
			encoded_length = (value_buf_len + temp_len + 1);
			break;

		/* Complex Data Type */
		case ATTR_TYPE_ARRAY:
		case ATTR_TYPE_STRUCTURE:
		case ATTR_TYPE_COMPACT_ARRAY:
			/* Do nothing, must encode by yourself */
			break;

		/* Error on selection */
		default:
			/* Do nothing */
			break;
	}

	return (encoded_length);
}

/******************************************************************************
* Function Name   : R_OBIS_EncodeCaptureObject
* Interface       : Unsigned16 R_OBIS_EncodeCaptureObject(
*                 :     Unsigned8 *buffer,
*                 :     class07_capture_object_t *p_capture_object
*                 : );
* Description     : Encode data buffer for a capture object
* Arguments       : Unsigned8 *buffer -
*                 :     Buffer to store the encoded data
*                 : class07_capture_object_t *p_capture_object -
*                 :     Pointer to capture_object
* Function Calls  : None
* Return Value    : Unsigned16, Actual encoded length
*                 :     0 mean error.
*                 :     > 0 is success, its value is actual used length of [buffer]
******************************************************************************/
Unsigned16 R_OBIS_EncodeCaptureObject(
	Unsigned8 *buffer,
	class07_capture_object_t *p_capture_object
)
{
	Unsigned16 	encoded_length = 0;		/* Encoded length, to return the used length */
	Unsigned8	i;

	/* Check input parameter */
	if (buffer == NULL ||
		p_capture_object == NULL)
	{
		return (encoded_length);		/* Not encode anything */
	}

	if (g_MasterTable[7].class_version == 0)
	{
		/* encode the capture_object_ definition */
		*buffer++ = ATTR_TYPE_STRUCTURE;
		*buffer++ = 3;
		encoded_length += 2;
	
		/* 1. logical_name */
		*buffer++ = ATTR_TYPE_OCTET_STRING;
		*buffer++ = 6;
		for (i = 0; i < 6; i++)
		{
			*buffer++ = p_capture_object->logical_name[i];
		}
		encoded_length += 8;
		
		/* 2. class_id */
		*buffer++ = ATTR_TYPE_LONG_UNSIGNED;
		*buffer++ = (Unsigned8)(p_capture_object->class_id >> 8);
		*buffer++ = (Unsigned8)(p_capture_object->class_id);
		encoded_length += 3;
		
		/* 3. Encode attribute_index */
		*buffer++ = ATTR_TYPE_UNSIGNED;
		*buffer++ = (Integer8)((Unsigned8)p_capture_object->attr_id);
		encoded_length += 2;
	}
	else		/* Version 1 */
	{
		/* encode the capture_object_ definition */
		*buffer++ = ATTR_TYPE_STRUCTURE;
		*buffer++ = 4;
		encoded_length += 2;
		
		/* 1. class_id */
		*buffer++ = ATTR_TYPE_LONG_UNSIGNED;
		*buffer++ = (Unsigned8)(p_capture_object->class_id >> 8);
		*buffer++ = (Unsigned8)(p_capture_object->class_id);
		encoded_length += 3;
		
		/* 2. logical_name */
		*buffer++ = ATTR_TYPE_OCTET_STRING;
		*buffer++ = 6;
		for (i = 0; i < 6; i++)
		{
			*buffer++ = p_capture_object->logical_name[i];
		}
		encoded_length += 8;
		
		/* 3. Encode attribute_index */
		*buffer++ = ATTR_TYPE_INTEGER;
		*buffer++ = (Integer8)((Unsigned8)p_capture_object->attr_id);
		encoded_length += 2;
		
		/* 4. data_index */
		*buffer++ = ATTR_TYPE_LONG_UNSIGNED;
		*buffer++ = (Unsigned8)(p_capture_object->data_id >> 8);
		*buffer++ = (Unsigned8)(p_capture_object->data_id);
		encoded_length += 3;
	}

	return (encoded_length);
}

/******************************************************************************
* Function Name   : R_OBIS_DecodeAttribute
* Interface       : Unsigned8 R_OBIS_DecodeAttribute(
*                 :     Unsigned8   *value_buf,
*                 :     Unsigned16  value_buf_len,
*                 :     attr_type_t attr_type,
*                 :     Unsigned8   *buf,
*                 :     Unsigned16  buf_len
*                 : );
* Description     : Decode data buffer of a specified attribute type to value type
* Arguments       : Unsigned8 * value_buf    : Buffer that store result value
*                 : Unsigned16 value_buf_len : Specify the length of buffer of value
*                 : attr_type_t attr_type    : Specify attribute type want to decode
*                 : Unsigned8 * buf          : Encoded data that send by client
*                 : Unsigned16 buf_len       : The actual length of encoded buffer
* Function Calls  : None
* Return Value    : Unsigned16, Actual decoded length
*                 :     0 mean error or nothing decoded.
*                 :     > 0 is success, is the actual used length on [value_buf]
******************************************************************************/
Unsigned16 R_OBIS_DecodeAttribute(
	Unsigned8	*value_buf,
	Unsigned16 	value_buf_len,
	attr_type_t	attr_type,
	Unsigned8 	*buf,
	Unsigned16 	buf_len
)
{
	Unsigned16 decoded_length = 0;		/* Decoded length, to return the used length */
	Unsigned16  item_length = 0;		/* Item length in the buf */
	Unsigned16 i;						/* Loop counter */
	Unsigned8 temp_len;

	/* Check input parameter */
	if ((value_buf == NULL) ||
		(value_buf_len == 0) ||
		(buf == NULL))
	{
		return (decoded_length);		/* Not decode anything */
	}

	/* Decode buffer base on select of type */
	switch (attr_type)
	{
		/* Simple Data Types */
		case ATTR_TYPE_NULL_DATA:
			/* Do nothing */
			break;

		/* Optimized for 1 byte types */
		case ATTR_TYPE_BOOLEAN:
		case ATTR_TYPE_BCD:
		case ATTR_TYPE_INTEGER:
		case ATTR_TYPE_UNSIGNED:
		case ATTR_TYPE_ENUM:
			if (buf_len >= 2)		/* Type + Value */
			{
				/* Ignorge the type, get value only */
				buf++;
				*value_buf = *buf;
				decoded_length = 2;
			}
			break;

		/* Optimized for 2 bytes types */
		case ATTR_TYPE_LONG:
		case ATTR_TYPE_LONG_UNSIGNED:
			if (buf_len >= 3)	/* Type + Value */
			{
				/* Ignorge the type, get value only */
				buf++;
				*(value_buf+1) = *buf++;
				*(value_buf+0) = *buf;
				decoded_length = 3;
			}
			break;

		/* Optimized for 4 bytes types */
		case ATTR_TYPE_DOUBLE_LONG:
		case ATTR_TYPE_DOUBLE_LONG_UNSIGNED:
		case ATTR_TYPE_FLOAT32:
			if (buf_len >= 5)	/* Type + Value */
			{
				/* Ignorge the type, get value only */
				buf++;
				*(value_buf+3) = *buf++;
				*(value_buf+2) = *buf++;
				*(value_buf+1) = *buf++;
				*(value_buf+0) = *buf;
				decoded_length = 5;
			}
			break;

		/* Optimized for 8 bytes types */
		case ATTR_TYPE_LONG64:
		case ATTR_TYPE_LONG64_UNSIGNED:
		case ATTR_TYPE_FLOAT64:
			if (buf_len >= 9)
			{
				/* Ignorge the type, get value only */
				buf++;
				*(value_buf+7) = *buf++;
				*(value_buf+6) = *buf++;
				*(value_buf+5) = *buf++;
				*(value_buf+4) = *buf++;
				*(value_buf+3) = *buf++;
				*(value_buf+2) = *buf++;
				*(value_buf+1) = *buf++;
				*(value_buf+0) = *buf;
				decoded_length = 9;
			}
			break;

		/* Optimized for variant size */
		case ATTR_TYPE_TIME:
		case ATTR_TYPE_DATE:
		case ATTR_TYPE_DATE_TIME:
		case ATTR_TYPE_BIT_STRING:
		case ATTR_TYPE_OCTET_STRING:
		case ATTR_TYPE_VISIBLE_STRING:
		case ATTR_TYPE_UTF8_STRING:
			if (buf_len >= 2)
			{
				// TODO, Check here
				buf++;					/* Ignorge the type */
				/* Get the item length */
				temp_len = BERLengthDecode(buf, &item_length);
				buf += temp_len;
				if (value_buf_len != item_length)
				{
					break;
				}

				/* Get value */
				for (i = 0; i < (Unsigned16)item_length; i++)
				{
					*(value_buf + i) = *buf++;
				}
				decoded_length = (Unsigned16)(item_length + temp_len + 1);
			}
			break;

		/* Complex Data Type */
		case ATTR_TYPE_ARRAY:
		case ATTR_TYPE_STRUCTURE:
		case ATTR_TYPE_COMPACT_ARRAY:
			/* Do nothing, must decode by yourself */
			break;

		/* Error on selection */
		default:
			/* Do nothing */
			break;
	}

	return (decoded_length);
}

/******************************************************************************
* Function Name   : R_OBIS_EncodeScalerUnit
* Interface       : Unsigned16 R_OBIS_EncodeScalerUnit(
*                 :     Unsigned8   	*buf,
*                 :     Unsigned16  	buf_len,
*                 :     scaler_unit_t   *p_scaler_unit
*                 : );
* Description     : Encode scaler_unit_t value
* Arguments       : Unsigned8 * buf             : Output buffer to store the encoded data
*                 : Unsigned16 buf_len          : The length of output buffer
*                 : scaler_unit_t *p_scaler_unit: Point to the scaler_unit value
* Function Calls  : None
* Return Value    : Unsigned16, Actual encoded length
*                 :     0 mean error.
*                 :     > 0 is success, its value is actual used length of [buf]
******************************************************************************/
Unsigned16 R_OBIS_EncodeScalerUnit(
    Unsigned8   	*buf,
    Unsigned16  	buf_len,
    scaler_unit_t   *p_scaler_unit
)
{
	Unsigned16 encoded_length = 0;		/* Encoded length, to return the used length */

	/* Check input parameter */
	if ((buf == NULL) ||
		(p_scaler_unit == NULL) ||
		(buf_len == 0) ||
		(buf_len < sizeof(scaler_unit_t) + 4))
	{
		return (encoded_length);		/* Not encode anything */
	}

	*buf++ = ATTR_TYPE_STRUCTURE;		/* structure type */
	*buf++ = 2;							/* 2 elements */

	/* 1. Encode for scaler (integer) */
	{
		*buf++ = ATTR_TYPE_INTEGER;
		*buf++ = p_scaler_unit->scaler;
	}
	/* 2. Encode for unit (enum) */
	{
		*buf++ = ATTR_TYPE_ENUM;
		*buf++ = p_scaler_unit->unit;
	}

	return (6);
}

/******************************************************************************
* Function Name   : R_OBIS_EncodeGenericBuffer
* Interface       : Unsigned8 R_OBIS_EncodeGenericBuffer(
*                 :     buffer_t            *buf,
*                 :     Unsigned8           b_selective_access,
*                 :     Unsigned16          master_index,
*                 :     Unsigned16          child_index,
*                 :     Unsigned8           *p_selector_buffer,
*                 :     service_type_t      service_type,
*                 :     Unsigned8           *p_out_buf,
*                 :     Unsigned16          *p_out_len
*                 : );
* Description     : Encode generic buffer
* Arguments       : buffer_t            *buf,               :[In] Pointer to buffer_t
*                 : Unsigned8           b_selective_access, :[In] Selective access value
*                 : Unsigned16          master_index,       :[In] Id of the row in master list
*                 : Unsigned16          child_index,        :[In] Id of the row in child list
*                 : Unsigned8           *p_selector_buffer, :[In] Pointer to the selector (from Client)
*                 : service_type_t      service_type,       :[In] Service type
*                 : Unsigned8           *p_out_buf,         :[Out] Data pointer for get
*                 : Unsigned16          *p_out_len          :[Out] Data length pointer for get
* Function Calls  : User defined functions
* Return Value    : Unsigned8
******************************************************************************/
Unsigned8 R_OBIS_EncodeGenericBuffer(
    buffer_t            *buf,
    Unsigned8           b_selective_access,
    Unsigned16          master_index,
    Unsigned16          child_index,
    Unsigned8           *p_selector_buffer,
    service_type_t      service_type,
    Unsigned8           *p_out_buf,
    Unsigned16          *p_out_len
)
{
	/* Result of response, def. is not Matched */
	Unsigned8 response_result = TYPE_UNMATCH;
	Unsigned16 request_no_entries;

	/* Check input parameter */
	if ((buf == NULL) ||
		(p_out_buf == NULL) ||
		(p_out_len == NULL))
	{
		return (response_result);
	}

	if (service_type == SERVICE_TYPE_NORMAL || 
		buf->async_remain_entries == 0)			/* In case of Attribute 0 */
	{
		/* Call to User defined scan function */
		response_result = R_OBIS_BufferScanByUserFunc(
			buf,								/* [In] Pointer to buffer_t                      */
			b_selective_access,					/* [In] Selective access                         */
			master_index,						/* [In] Id of the row in master list             */
			child_index,						/* [In] Id of the row in child list              */
			p_selector_buffer					/* [In] Pointer to the selector (from Client)    */
		);
		if (response_result != DATA_ACCESS_RESULT_SUCCESS)
		{
			/* Fail when scan buffer info */
			return (response_result);
		}

		/* Set as default for fist time, will autonatically calculate later */
		buf->async_entries_per_block = 1;
	}

	/* Judge for block transfer or not ? */
	if (buf->async_entries_per_block < buf->async_remain_entries)
	{
		request_no_entries = buf->async_entries_per_block;
	}
	else
	{
		request_no_entries = buf->async_remain_entries;
	}

	/* Call to User defined filter function */
	response_result = R_OBIS_BufferFilterByUserFunc(
		buf,							/* [In] Pointer to buffer_t               */
		master_index,					/* [In] [In] Id of the row in master list */
		child_index,					/* [In] [In] Id of the row in child list  */
		request_no_entries,				/* [In] Number of entries need to filter  */
		p_out_buf,						/* [Out] Data pointer for get             */
		p_out_len 						/* [Out] Data length pointer for get      */
	);

	/* Re-calculate number entry to be transfer for next time */
	buf->one_entry_len = (*p_out_len / request_no_entries);
	if (buf->one_entry_len != 0)
	{
		/* Calculate number entry to be transfer one-time */
		buf->async_entries_per_block = (OBIS_SERVICE_DATA_BUFFER_LENGTH - 10) / buf->one_entry_len;
	}

	/* Update remain entries */
	buf->async_remain_entries -= request_no_entries;

	/* Transfer completed ? */
	if (buf->async_remain_entries != 0 &&
		response_result == DATA_ACCESS_RESULT_SUCCESS)
	{
		response_result = DATA_ACCESS_RESULT_PARTIAL;
	}

	return (response_result);
}

/******************************************************************************
* Function Name : R_OBIS_DistributeByMem
* Interface     : Unsigned8 R_OBIS_DistributeByMem(
*               :     attr_type_t 		attr_type,
*               :     Integer16			attr_size,
*               :     void 				*attr_buffer,
*               :     req_type_t		req_type,
*               :     Unsigned8			*p_is_block_transfer,
*               :     Unsigned8			*p_out_buf,
*               :     Unsigned16		*p_out_len
*               : );
* Description   : Distribute by using internal memory
* Arguments     : attr_type_t attr_type            : Choice selection
*               : Unsigned16 attr_size             : Buffer of value
*               : void * attr_buffer               : Buffer of value
*               : req_type_t req_type              : Request type
*               : Unsigned8 * p_is_block_transfer  : Buffer indicate use block transfer or not
*               : Unsigned8 * p_out_buf            : Data pointer for get
*               : Unsigned16 * p_out_len           : Data length pointer for get
* Function Calls: T.B.D
* Return Value  : Unsigned8
******************************************************************************/
Unsigned8 R_OBIS_DistributeByMem(
	attr_type_t 		attr_type,
	Integer16			attr_size,
	void 				*attr_buffer,
	req_type_t			req_type,
	Unsigned8			*p_is_block_transfer,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
)
{
	if (p_is_block_transfer == NULL)
	{
		return TYPE_UNMATCH;
	}

	if (req_type == REQ_TYPE_GET)
	{
		if (attr_size == -1)
		{
			attr_size = strlen(attr_buffer);
		}
		/* Encode inside */
		*p_out_len = R_OBIS_EncodeAttribute(
			p_out_buf,							/* Output buffer, pointed to g_ServiceBuffer */
			OBIS_SERVICE_DATA_BUFFER_LENGTH,	/* Max length of g_ServiceBuffer */
			attr_type,							/* Type */
			attr_buffer,						/* Buffer */
			attr_size							/* Length */
		);
	}
	else
	{
		/*
		* TODO: Determind the length of encoded buffer,
		* Following is just a dummy (OBIS_SERVICE_DATA_BUFFER_LENGTH)
		*/
		Unsigned16 decoded_length;

		if (attr_size == -1)
		{
			attr_size = OBIS_SERVICE_DATA_BUFFER_LENGTH;
		}
		/* Decode attribute */
		decoded_length = R_OBIS_DecodeAttribute(
			attr_buffer,							/* Output value buffer */
			attr_size,								/* Size of buffer */
			attr_type,								/* Type */
			pdata,									/* Get from data pointer of HDLC protocol stack */
			OBIS_SERVICE_DATA_BUFFER_LENGTH			/* Encoded length from HDLC */
		);
		if (decoded_length == 0)
		{
			return TYPE_UNMATCH;
		}
	}

	/* Success */
	*p_is_block_transfer = 0;
	return DATA_ACCESS_RESULT_SUCCESS;
}

/******************************************************************************
* Function Name : R_OBIS_DistributeByUserFunc
* Interface     : Unsigned8 R_OBIS_DistributeByUserFunc(
*               :     Unsigned16  		child_index,
*               :     req_type_t		req_type,
*               :     service_type_t	service_type,
*               :     Unsigned8			*p_is_block_transfer,
*               :     st_Cosem_Attr_Desc*p_descriptor,
*               :     Unsigned8			*pdata,
*               :     Unsigned8			*p_out_buf,
*               :     Unsigned16		*p_out_len
*               : );
* Description   : Distribute to user function
* Arguments     : Unsigned16 child_index           : Child ID
*               : req_type_t req_type              : Request type
*               : service_type_t service_type      : Service type
*               : Unsigned8 * p_is_block_transfer  : Buffer indicate use block transfer or not
*               : st_Cosem_Attr_Desc * p_descriptor: Pointer to COSEM descriptor
*               : Unsigned8 * pdata                : Data from COSEM Appl layer
*               : Unsigned8 * p_out_buf            : Data pointer for get
*               : Unsigned16 * p_out_len           : Data length pointer for get
* Function Calls: R_OBIS_GetObjectAttr, R_OBIS_SetObjectAttr
* Return Value  : Unsigned8
******************************************************************************/
Unsigned8 R_OBIS_DistributeByUserFunc(
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	Unsigned8			*p_is_block_transfer,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
)
{
	Unsigned8 response_result = TYPE_UNMATCH;

	if (p_is_block_transfer == NULL ||
		p_descriptor == NULL)
	{
		return (response_result);
	}

	if (req_type == REQ_TYPE_GET)
	{
		response_result = R_OBIS_GetObjectAttr(
			p_descriptor,			/* [In] COSEM Obj Descriptor */
			service_type,			/* [In] Service type */
			child_index,			/* [In] Child ID in the child object table */
			p_out_buf,				/* [Out] Encoded output data */
			p_out_len,				/* [Out] Encoded length */
			p_is_block_transfer,	/* [Out] Indicate whether block transfer is used or not */
			pdata					/* [In] Data from client */
		);
	}
	else if (req_type == REQ_TYPE_SET)
	{
		response_result = R_OBIS_SetObjectAttr(
			p_descriptor,						/* [In] COSEM Obj Descriptor */
			service_type,						/* [In] Service type */
			child_index,						/* [In] Child ID in the child object table */
			pdata,								/* [In] Encoded input data (from client) */
			OBIS_SERVICE_DATA_BUFFER_LENGTH,	/* [In] Encoded input data length (from client) */
			*p_is_block_transfer				/* [In] Indicate whether block transfer is used or not */
		);
	}
	else if (req_type == REQ_TYPE_ACTION)
	{
		response_result = R_OBIS_ActionObjectMethod(
			(st_Cosem_Method_Desc *)p_descriptor,		/* [In] COSEM Obj Descriptor */
			service_type,								/* [In] Service type */
			child_index,								/* [In] Child ID in the child object table */
			p_out_buf,									/* [Out] Encoded output data */
			p_out_len,									/* [Out] Encoded length */
			pdata,										/* [In] Encoded input data (from client) */
			OBIS_SERVICE_DATA_BUFFER_LENGTH,			/* [In] Encoded input data length (from client) */
			p_is_block_transfer							/* [In] Indicate whether block transfer is used or not */
		);
	}
	else
	{
		/* Do nothing */
	}

	return (response_result);
}

/******************************************************************************
* Function Name : R_OBIS_DistributeByE2PR
* Interface     : Unsigned8 R_OBIS_DistributeByE2PR(
*               :     attr_access_t			*p_attr_access,
*               :     attr_type_t 			attr_type,
*               :     Integer16				attr_size,
*               :     req_type_t			req_type,
*               :     Unsigned8				*p_is_block_transfer,
*               :     Unsigned8				*pdata,
*               :     Unsigned8				*p_out_buf,
*               :     Unsigned16			*p_out_len
*               : );
* Description   : Distribute attribute by access to EEPROM
* Arguments     : attr_access_t *p_attr_access     : Access method option
*               : attr_type_t attr_type            : Choice selection
*               : Unsigned16 attr_size             : Buffer of value
*               : req_type_t req_type              : Request type
*               : Unsigned8 * p_is_block_transfer  : Buffer indicate use block transfer or not
*               : Unsigned8 * pdata                : Data from COSEM Appl layer
*               : Unsigned8 * p_out_buf            : Data pointer for get
*               : Unsigned16 * p_out_len           : Data length pointer for get
* Function Calls: R_OBIS_E2PRGetData, R_OBIS_E2PRSetData
* Return Value  : Unsigned8
******************************************************************************/
//Unsigned8 R_OBIS_DistributeByE2PR(
//	attr_access_t		*p_attr_access,
//	attr_type_t 		attr_type,
//	Integer16			attr_size,
//	req_type_t			req_type,
//	Unsigned8			*p_is_block_transfer,
//	Unsigned8			*pdata,
//	Unsigned8			*p_out_buf,
//	Unsigned16			*p_out_len
//)
//{
//	Unsigned8 response_result = TYPE_UNMATCH;
//	if (p_is_block_transfer == NULL)
//	{
//		return (response_result);
//	}

//	if (req_type == REQ_TYPE_GET)
//	{
//		/* Call to user distributor as announcement of Begin GET */
//		response_result = R_OBIS_E2PRGetData(
//			p_attr_access,		/* Access method option			*/
//			attr_type,			/* Choice selection				*/
//			attr_size,			/* Buffer of value				*/
//			p_out_buf,			/* Data pointer for get			*/
//			p_out_len			/* Data length pointer for get	*/
//		);
//	}
//	else
//	{
//		/* Call to user distributor as announcement of Begin GET */
//		response_result = R_OBIS_E2PRSetData(
//			p_attr_access,		/* Access method option			*/
//			attr_type,			/* Choice selection				*/
//			attr_size,			/* Buffer of value				*/
//			pdata				/* Data from COSEM Appl layer	*/
//		);
//	}

//	/* Complete */
//	return (response_result);
//}


/******************************************************************************
* Function Name : R_OBIS_DistributeAttrBuffer
* Interface     : Unsigned8 R_OBIS_DistributeAttrBuffer(
*               :     attr_access_t *p_attr_access,
*               :     attr_type_t 			attr_type,
*               :     Unsigned16			attr_size,
*               :     void 					*attr_buffer,
*               :     Unsigned16  			child_index,
*               :     req_type_t			req_type,
*               :     service_type_t		service_type,
*               :     st_Cosem_Attr_Desc	*p_descriptor,
*               :     Unsigned8				*pdata,
*               :     Unsigned8				*p_out_buf,
*               :     Unsigned16			*p_out_len
*               : );
* Description   : Distribute CHOICE value
* Arguments     : attr_access_t *p_attr_access     : Access method option
*               : attr_type_t attr_type            : Choice selection
*               : Unsigned16 attr_size             : Size of buffer value
*               : void * attr_buffer               : Buffer of value
*               : Unsigned16 child_index           : Child ID
*               : req_type_t req_type              : Request type
*               : service_type_t service_type      : Service type
*               : st_Cosem_Attr_Desc * p_descriptor: Pointer to COSEM descriptor
*               : Unsigned8 * pdata                : Data from COSEM Appl layer
*               : Unsigned8 * p_out_buf            : Data pointer for get
*               : Unsigned16 * p_out_len           : Data length pointer for get
* Function Calls:
* Return Value  : Unsigned8
******************************************************************************/
Unsigned8 R_OBIS_DistributeAttrBuffer(
	attr_access_t		*p_attr_access,
	attr_type_t 		attr_type,
	Integer16			attr_size,
	void 				*attr_buffer,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
)
{
	Unsigned8 response_result = TYPE_UNMATCH;
	Unsigned8 b_block_transfer = 0;

	/* Need to call to user distributor ? */
	if ((p_attr_access == NULL && attr_buffer == NULL) ||
		attr_type == ATTR_TYPE_ARRAY ||
		attr_type == ATTR_TYPE_STRUCTURE ||
		attr_type == ATTR_TYPE_COMPACT_ARRAY)
	{
		response_result = R_OBIS_DistributeByUserFunc(
			child_index,			/* Child ID */
			req_type,				/* Request type */
			service_type,			/* Service type */
			&b_block_transfer,		/* Buffer indicate use block transfer or not */
			p_descriptor,			/* Pointer to COSEM descriptor */
			pdata,					/* Data from COSEM Appl layer */
			p_out_buf,				/* Data pointer for get */
			p_out_len				/* Data length pointer for get */
		);
	}

	/* Default to use internal memory if p_attr_access is NULL and has attr_buffer */
	if (p_attr_access == NULL)
	{
		response_result = R_OBIS_DistributeByMem(
				attr_type,				/* Type */
				attr_size,				/* Size */
				attr_buffer,			/* Buffer of value */
				req_type,				/* Request type */
				&b_block_transfer,		/* Buffer indicate use block transfer or not */
				pdata,					/* Data from COSEM Appl layer */
				p_out_buf,				/* Data pointer for get */
				p_out_len				/* Data length pointer for get */
			);
	}
	else
	{
		switch (p_attr_access->method)
		{
			case ACCESS_MTD_MEM:
				response_result = R_OBIS_DistributeByMem(
					attr_type,				/* Type */
					attr_size,				/* Size */
					attr_buffer,			/* Buffer of value */
					req_type,				/* Request type */
					&b_block_transfer,		/* Buffer indicate use block transfer or not */
					pdata,					/* Data from COSEM Appl layer */
					p_out_buf,				/* Data pointer for get */
					p_out_len				/* Data length pointer for get */
				);
				break;

			case ACCESS_MTD_USERFUNC:
				response_result = R_OBIS_DistributeByUserFunc(
					child_index,			/* Child ID */
					req_type,				/* Request type */
					service_type,			/* Service type */
					&b_block_transfer,		/* Buffer indicate use block transfer or not */
					p_descriptor,			/* Pointer to COSEM descriptor */
					pdata,					/* Data from COSEM Appl layer */
					p_out_buf,				/* Data pointer for get */
					p_out_len				/* Data length pointer for get */
				);
				break;

//			case ACCESS_MTD_E2PR:
//				response_result = R_OBIS_DistributeByE2PR(
//					p_attr_access,			/* Access method option			*/
//					attr_type,				/* Type */
//					attr_size,				/* Size */
//					req_type,				/* Request type */
//					&b_block_transfer,		/* Buffer indicate use block transfer or not */
//					pdata,					/* Data from COSEM Appl layer */
//					p_out_buf,				/* Data pointer for get */
//					p_out_len				/* Data length pointer for get */
//				);
//				break;
			default:
				break;
		}
	}

	/* Change to partial status when indicate block transfer bit */
	if (b_block_transfer == 1 &&
		response_result == DATA_ACCESS_RESULT_SUCCESS)
	{
		response_result = DATA_ACCESS_RESULT_PARTIAL;
	}

	return (response_result);
}

/******************************************************************************
* Function Name : R_OBIS_DistributeDataClass
* Interface     : Unsigned8 R_OBIS_DistributeDataClass(
*               :     Unsigned16 master_index,
*               :     Unsigned16 child_index,
*               :     req_type_t req_type,
*               :     service_type_t service_type,
*               :     st_Cosem_Attr_Desc *p_descriptor,
*               :     Unsigned8	*pdata,
*               :     Unsigned8	*p_out_buf,
*               :     Unsigned16 *p_out_len
*               : );
* Description   : Distributor function for data class
* Arguments     : Unsigned16 master_index -
*               :    [In] Id of the row in master list
*               : Unsigned16 child_index -
*               :    [In] Id of the row in child list
*               : req_type_t req_type -
*               :    [In] Request type (GET/SET/ACTION)
*               : service_type_t service_type -
*               :    [In] Service type (NORMAL/BLOCK/LIST)
*               : st_Cosem_Attr_Desc *p_descriptor -
*               :    [In] COSEM object descriptor
*               : Unsigned8 * pdata -
*               :    [In] Data in case of SET/ACTION
*               : Unsigned8 * p_out_buf -
*               :    [Out] Output buffer for encoded data
*               : Unsigned16 * p_out_len -
*               :    [Out] Output buffer length for encoded data
* Function Calls: TBD
* Return Value  : Unsigned8, distribution status
******************************************************************************/
#if (defined(USED_CLASS_01) && USED_CLASS_01 == 1)
Unsigned8 R_OBIS_DistributeDataClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t 		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
)
{
	/* Result of response, def. is not Matched */
	Unsigned8				response_result = TYPE_UNMATCH;
	class01_child_record_t	*p_child_record;

	p_child_record = (class01_child_record_t *)(
		R_OBIS_FindChildRecordByIndex(master_index, child_index)
	);

	/* Classify the code based on Attribute or Method request */
	if (req_type == REQ_TYPE_GET || req_type == REQ_TYPE_SET)
	{
		switch (p_descriptor->Attr_ID)
		{
			case 0: /* attribute 0 */
				/* Not support */
				break;

			case 1: /* logical_name */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,							/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,	/* Max length of g_ServiceBuffer */
						ATTR_TYPE_OCTET_STRING,				/* Octet-string */
						p_descriptor->Instance_ID,			/* OBIS code from the request */
						6);									/* OBIS code is 6 chars */

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, OBIS code is read only */
				}
				break;

			case 2:	/* value */

				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				/* distribute the buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					&p_child_record->value.access,		/* Access method option */
					p_child_record->value.choice.type,	/* Type */
					p_child_record->value.choice.size,	/* Size */
					p_child_record->value.buffer,		/* Buffer of value */
					child_index,						/* Child ID */
					req_type,							/* Request type */
					service_type,						/* Service type */
					p_descriptor,						/* Pointer to COSEM descriptor */
					pdata,								/* Data from COSEM Appl layer */
					p_out_buf,							/* Data pointer for get */
					p_out_len							/* Data length pointer for get */
				);

				break;

			default:
				break;
		}
	}
	else if (req_type == REQ_TYPE_ACTION)
	{
		/* Not availabe */
	}
	else
	{
		/* INTERNAL ERROR */
	}

	return (response_result);
}
#endif

/******************************************************************************
* Function Name : R_OBIS_DistributeRegisterClass
* Interface     : Unsigned8 R_OBIS_DistributeRegisterClass(
*               :     Unsigned16 master_index,
*               :     Unsigned16 child_index,
*               :     req_type_t req_type,
*               :     service_type_t service_type,
*               :     st_Cosem_Attr_Desc *p_descriptor,
*               :     Unsigned8	*pdata,
*               :     Unsigned8	*p_out_buf,
*               :     Unsigned16 *p_out_len
*               : );
* Description   : Distributor function for Register class
* Arguments     : Unsigned16 master_index -
*               :    [In] Id of the row in master list
*               : Unsigned16 child_index -
*               :    [In] Id of the row in child list
*               : req_type_t req_type -
*               :    [In] Request type (GET/SET/ACTION)
*               : service_type_t service_type -
*               :    [In] Service type (NORMAL/BLOCK/LIST)
*               : st_Cosem_Attr_Desc *p_descriptor -
*               :    [In] COSEM object descriptor
*               : Unsigned8 * pdata -
*               :    [In] Data in case of SET/ACTION
*               : Unsigned8 * p_out_buf -
*               :    [Out] Output buffer for encoded data
*               : Unsigned16 * p_out_len -
*               :    [Out] Output buffer length for encoded data
* Function Calls: TBD
* Return Value  : Unsigned8, distribution status
******************************************************************************/
#if (defined(USED_CLASS_03) && USED_CLASS_03 == 1)
Unsigned8 R_OBIS_DistributeRegisterClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
)
{
	/* Result of response, def. is not Matched */
	Unsigned8				response_result = TYPE_UNMATCH;
	class03_child_record_t	*p_child_record;

	p_child_record = (class03_child_record_t *)(
		R_OBIS_FindChildRecordByIndex(master_index, child_index)
	);

	/* Classify the code based on Attribute or Method request */
	if (req_type == REQ_TYPE_GET || req_type == REQ_TYPE_SET)
	{
		switch (p_descriptor->Attr_ID)
		{
			case 0: /* attribute 0 */
				/* Not support */
				break;

			case 1: /* logical_name */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,							/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,	/* Max length of g_ServiceBuffer */
						ATTR_TYPE_OCTET_STRING,				/* Octet-string */
						p_descriptor->Instance_ID,			/* OBIS code from the request */
						6);									/* OBIS code is 6 chars */

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, OBIS code is read only */
				}
				break;

			case 2:	/* value */

				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				/* distribute the buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					&p_child_record->value.access,		/* Access method option */
					p_child_record->value.choice.type,	/* Type */
					p_child_record->value.choice.size,	/* Size */
					p_child_record->value.buffer,		/* Buffer of value */
					child_index,						/* Child ID */
					req_type,							/* Request type */
					service_type,						/* Service type */
					p_descriptor,						/* Pointer to COSEM descriptor */
					pdata,								/* Data from COSEM Appl layer */
					p_out_buf,							/* Data pointer for get */
					p_out_len							/* Data length pointer for get */
				);

				break;

			case 3:	/* scaler_unit */

				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeScalerUnit(
						p_out_buf,							/* Output buffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,	/* Max length */
						&(p_child_record->scaler_unit));

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, not set the scaler, unit */
				}
				break;

			default:
				break;
		}
	}
	else if (req_type == REQ_TYPE_ACTION)
	{
		Unsigned8 b_block_transfer = 0;
		
		switch (p_descriptor->Attr_ID & 0x00FF)
		{
			case 1:	/* reset (data) */
				/* User function take care for all */
				response_result = R_OBIS_DistributeByUserFunc(
					child_index,		/* child_index */	
					req_type,			/* req_type */
					service_type,		/* service_type */
					&b_block_transfer,	/* p_is_block_transfer */
					p_descriptor,		/* p_descriptor */	
					pdata,				/* pdata */			
					p_out_buf,			/* p_out_buf */		
					p_out_len			/* p_out_len */
				);
				/* Not support block transfer now, TODO in future */
				if (b_block_transfer != 0)
				{
					response_result = ACTION_RESULT_SCOPE_ACCESS_VIOLATED;
				}
				break;
			default:
				response_result = ACTION_RESULT_TYPE_UNMATCH;
				break;
		}
	}
	else
	{
		/* INTERNAL ERROR */
	}

	return (response_result);
}
#endif

/******************************************************************************
* Function Name : R_OBIS_DistributeXRegisterClass
* Interface     : Unsigned8 R_OBIS_DistributeXRegisterClass(
*               :     Unsigned16 master_index,
*               :     Unsigned16 child_index,
*               :     req_type_t req_type,
*               :     service_type_t service_type,
*               :     st_Cosem_Attr_Desc *p_descriptor,
*               :     Unsigned8	*pdata,
*               :     Unsigned8	*p_out_buf,
*               :     Unsigned16 *p_out_len
*               : );
* Description   : Distributor function for XRegister class
* Arguments     : Unsigned16 master_index -
*               :    [In] Id of the row in master list
*               : Unsigned16 child_index -
*               :    [In] Id of the row in child list
*               : req_type_t req_type -
*               :    [In] Request type (GET/SET/ACTION)
*               : service_type_t service_type -
*               :    [In] Service type (NORMAL/BLOCK/LIST)
*               : st_Cosem_Attr_Desc *p_descriptor -
*               :    [In] COSEM object descriptor
*               : Unsigned8 * pdata -
*               :    [In] Data in case of SET/ACTION
*               : Unsigned8 * p_out_buf -
*               :    [Out] Output buffer for encoded data
*               : Unsigned16 * p_out_len -
*               :    [Out] Output buffer length for encoded data
* Function Calls: TBD
* Return Value  : Unsigned8, distribution status
******************************************************************************/
#if (defined(USED_CLASS_04) && USED_CLASS_04 == 1)
Unsigned8 R_OBIS_DistributeXRegisterClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
)
{
	/* Result of response, def. is not Matched */
	Unsigned8				response_result = TYPE_UNMATCH;
	class04_child_record_t	*p_child_record;

	p_child_record = (class04_child_record_t *)(
		R_OBIS_FindChildRecordByIndex(master_index, child_index)
	);

	/* Classify the code based on Attribute or Method request */
	if (req_type == REQ_TYPE_GET || req_type == REQ_TYPE_SET)
	{
		switch (p_descriptor->Attr_ID)
		{
			case 0: /* attribute 0 */
				/* Not support */
				break;

			case 1: /* logical_name */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,							/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,	/* Max length of g_ServiceBuffer */
						ATTR_TYPE_OCTET_STRING,				/* Octet-string */
						p_descriptor->Instance_ID,			/* OBIS code from the request */
						6);									/* OBIS code is 6 chars */

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, OBIS code is read only */
				}
				break;

			case 2:	/* value */

				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				/* distribute the buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					&p_child_record->value.access,		/* Access method option */
					p_child_record->value.choice.type,	/* Type */
					p_child_record->value.choice.size,	/* Size */
					p_child_record->value.buffer,		/* Buffer of value */
					child_index,						/* Child ID */
					req_type,							/* Request type */
					service_type,						/* Service type */
					p_descriptor,						/* Pointer to COSEM descriptor */
					pdata,								/* Data from COSEM Appl layer */
					p_out_buf,							/* Data pointer for get */
					p_out_len							/* Data length pointer for get */
				);

				break;

			case 3:	/* scaler_unit */

				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeScalerUnit(
						p_out_buf,							/* Output buffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,	/* Max length */
						&(p_child_record->scaler_unit));

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, not set the scaler, unit */
				}
				break;

			case 4:	/* status */

				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				/* distribute the buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					&p_child_record->status.access,		/* Access method option */
					p_child_record->status.choice.type,	/* Type */
					p_child_record->status.choice.size,	/* Size */
					p_child_record->status.buffer,		/* Buffer of value */
					child_index,						/* Child ID */
					req_type,							/* Request type */
					service_type,						/* Service type */
					p_descriptor,						/* Pointer to COSEM descriptor */
					pdata,								/* Data from COSEM Appl layer */
					p_out_buf,							/* Data pointer for get */
					p_out_len							/* Data length pointer for get */
				);

				break;

			case 5:	/* capture_time */

				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				/* distribute the buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					&p_child_record->capture_time.access,			/* Access method option */
					p_child_record->capture_time.choice.type,		/* Type */
					p_child_record->capture_time.choice.size,		/* Size */
					p_child_record->capture_time.buffer,			/* Buffer of value */
					child_index,									/* Child ID */
					req_type,										/* Request type */
					service_type,									/* Service type */
					p_descriptor,									/* Pointer to COSEM descriptor */
					pdata,											/* Data from COSEM Appl layer */
					p_out_buf,										/* Data pointer for get */
					p_out_len										/* Data length pointer for get */
				);

				break;

			default:
				break;
		}
	}
	else if (req_type == REQ_TYPE_ACTION)
	{
		Unsigned8 b_block_transfer = 0;
		
		switch (p_descriptor->Attr_ID & 0x00FF)
		{
			case 1:	/* reset (data) */
				/* User function take care for all */
				response_result = R_OBIS_DistributeByUserFunc(
					child_index,		/* child_index */	
					req_type,			/* req_type */
					service_type,		/* service_type */
					&b_block_transfer,	/* p_is_block_transfer */
					p_descriptor,		/* p_descriptor */	
					pdata,				/* pdata */			
					p_out_buf,			/* p_out_buf */		
					p_out_len			/* p_out_len */
				);
				/* Not support block transfer now, TODO in future */
				if (b_block_transfer != 0)
				{
					response_result = ACTION_RESULT_SCOPE_ACCESS_VIOLATED;
				}
				break;
			default:
				response_result = ACTION_RESULT_TYPE_UNMATCH;
				break;
		}
	}
	else
	{
		/* INTERNAL ERROR */
	}

	return (response_result);
}
#endif

/******************************************************************************
* Function Name : R_OBIS_DistributeDRegisterClass
* Interface     : Unsigned8 R_OBIS_DistributeDRegisterClass(
*               :     Unsigned16 master_index,
*               :     Unsigned16 child_index,
*               :     req_type_t req_type,
*               :     service_type_t service_type,
*               :     st_Cosem_Attr_Desc *p_descriptor,
*               :     Unsigned8	*pdata,
*               :     Unsigned8	*p_out_buf,
*               :     Unsigned16 *p_out_len
*               : );
* Description   : Distributor function for DRegister class
* Arguments     : Unsigned16 master_index -
*               :    [In] Id of the row in master list
*               : Unsigned16 child_index -
*               :    [In] Id of the row in child list
*               : req_type_t req_type -
*               :    [In] Request type (GET/SET/ACTION)
*               : service_type_t service_type -
*               :    [In] Service type (NORMAL/BLOCK/LIST)
*               : st_Cosem_Attr_Desc *p_descriptor -
*               :    [In] COSEM object descriptor
*               : Unsigned8 * pdata -
*               :    [In] Data in case of SET/ACTION
*               : Unsigned8 * p_out_buf -
*               :    [Out] Output buffer for encoded data
*               : Unsigned16 * p_out_len -
*               :    [Out] Output buffer length for encoded data
* Function Calls: TBD
* Return Value  : Unsigned8, distribution status
******************************************************************************/
#if (defined(USED_CLASS_05) && USED_CLASS_05 == 1)
Unsigned8 R_OBIS_DistributeDRegisterClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
)
{
	/* Result of response, def. is not Matched */
	Unsigned8				response_result = TYPE_UNMATCH;
	class05_child_record_t	*p_child_record;

	p_child_record = (class05_child_record_t *)(
		R_OBIS_FindChildRecordByIndex(master_index, child_index)
	);

	/* Classify the code based on Attribute or Method request */
	if (req_type == REQ_TYPE_GET || req_type == REQ_TYPE_SET)
	{
		switch (p_descriptor->Attr_ID)
		{
			case 0: /* attribute 0 */
				/* Not support */
				break;

			case 1: /* logical_name */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,							/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,	/* Max length of g_ServiceBuffer */
						ATTR_TYPE_OCTET_STRING,				/* Octet-string */
						p_descriptor->Instance_ID,			/* OBIS code from the request */
						6);									/* OBIS code is 6 chars */

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, OBIS code is read only */
				}
				break;

			case 2:	/* current_average_value */

				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				/* distribute the buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					&p_child_record->current_average_value.access,		/* Access method option */
					p_child_record->current_average_value.choice.type,	/* Type */
					p_child_record->current_average_value.choice.size,	/* Size */
					p_child_record->current_average_value.buffer,		/* Buffer of value */
					child_index,										/* Child ID */
					req_type,											/* Request type */
					service_type,										/* Service type */
					p_descriptor,										/* Pointer to COSEM descriptor */
					pdata,												/* Data from COSEM Appl layer */
					p_out_buf,											/* Data pointer for get */
					p_out_len											/* Data length pointer for get */
				);

				break;

			case 3:	/* last_average_value */

				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				/* distribute the buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					&p_child_record->last_average_value.access,		/* Access method option */
					p_child_record->last_average_value.choice.type,	/* Type */
					p_child_record->last_average_value.choice.size,	/* Size */
					p_child_record->last_average_value.buffer,		/* Buffer of value */
					child_index,									/* Child ID */
					req_type,										/* Request type */
					service_type,									/* Service type */
					p_descriptor,									/* Pointer to COSEM descriptor */
					pdata,											/* Data from COSEM Appl layer */
					p_out_buf,										/* Data pointer for get */
					p_out_len										/* Data length pointer for get */
				);

				break;

			case 4:	/* scaler_unit */

				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeScalerUnit(
						p_out_buf,							/* Output buffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,	/* Max length */
						&(p_child_record->scaler_unit));

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, not set the scaler, unit */
				}
				break;

			case 5:	/* status */

				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				/* distribute the buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					&p_child_record->status.access,		/* Access method option */
					p_child_record->status.choice.type,	/* Type */
					p_child_record->status.choice.size,	/* Size */
					p_child_record->status.buffer,		/* Buffer of value */
					child_index,						/* Child ID */
					req_type,							/* Request type */
					service_type,						/* Service type */
					p_descriptor,						/* Pointer to COSEM descriptor */
					pdata,								/* Data from COSEM Appl layer */
					p_out_buf,							/* Data pointer for get */
					p_out_len							/* Data length pointer for get */
				);

				break;

			case 6:	/* capture_time */

				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				/* distribute the buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					&p_child_record->capture_time.access,			/* Access method option */
					p_child_record->capture_time.choice.type,		/* Type */
					p_child_record->capture_time.choice.size,		/* Size */
					p_child_record->capture_time.buffer,			/* Buffer of value */
					child_index,									/* Child ID */
					req_type,										/* Request type */
					service_type,									/* Service type */
					p_descriptor,									/* Pointer to COSEM descriptor */
					pdata,											/* Data from COSEM Appl layer */
					p_out_buf,										/* Data pointer for get */
					p_out_len										/* Data length pointer for get */
				);

				break;
			case 7:	/* start_time_current */

				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				/* distribute the buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					&p_child_record->start_time_current.access,			/* Access method option */
					p_child_record->start_time_current.choice.type,		/* Type */
					p_child_record->start_time_current.choice.size,		/* Size */
					p_child_record->start_time_current.buffer,			/* Buffer of value */
					child_index,										/* Child ID */
					req_type,											/* Request type */
					service_type,										/* Service type */
					p_descriptor,										/* Pointer to COSEM descriptor */
					pdata,												/* Data from COSEM Appl layer */
					p_out_buf,											/* Data pointer for get */
					p_out_len											/* Data length pointer for get */
				);

				break;

			case 8:	/* period */

				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				/* distribute the buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					NULL,									/* Access method option */
					ATTR_TYPE_DOUBLE_LONG_UNSIGNED,			/* Type */
					sizeof(Unsigned32),						/* Size */
					p_child_record->p_period,				/* Buffer of value */
					child_index,							/* Child ID */
					req_type,								/* Request type */
					service_type,							/* Service type */
					p_descriptor,							/* Pointer to COSEM descriptor */
					pdata,									/* Data from COSEM Appl layer */
					p_out_buf,								/* Data pointer for get */
					p_out_len								/* Data length pointer for get */
				);

				break;

			case 9:	/* number_of_periods */

				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				/* distribute the buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					NULL,									/* Access method option */
					ATTR_TYPE_LONG_UNSIGNED,				/* Type */
					sizeof(Unsigned16),						/* Size */
					p_child_record->p_number_of_periods,	/* Buffer of value */
					child_index,							/* Child ID */
					req_type,								/* Request type */
					service_type,							/* Service type */
					p_descriptor,							/* Pointer to COSEM descriptor */
					pdata,									/* Data from COSEM Appl layer */
					p_out_buf,								/* Data pointer for get */
					p_out_len								/* Data length pointer for get */
				);

				break;

			default:
				break;
		}
	}
	else if (req_type == REQ_TYPE_ACTION)
	{
		Unsigned8 b_block_transfer = 0;
		
		switch (p_descriptor->Attr_ID & 0x00FF)
		{
			case 1:	/* reset (data) */
			case 2:	/* next_period (data) */
				/* User function take care for all */
				response_result = R_OBIS_DistributeByUserFunc(
					child_index,		/* child_index */	
					req_type,			/* req_type */
					service_type,		/* service_type */
					&b_block_transfer,	/* p_is_block_transfer */
					p_descriptor,		/* p_descriptor */	
					pdata,				/* pdata */			
					p_out_buf,			/* p_out_buf */		
					p_out_len			/* p_out_len */
				);
				
				/* Not support block transfer now, TODO in future */
				if (b_block_transfer != 0)
				{
					response_result = ACTION_RESULT_SCOPE_ACCESS_VIOLATED;
				}
				break;
			default:
				response_result = ACTION_RESULT_TYPE_UNMATCH;
				break;
		}
	}
	else
	{
		/* INTERNAL ERROR */
	}

	return (response_result);
}
#endif

/******************************************************************************
* Function Name : R_OBIS_DistributeRegisterActivationClass
* Interface     : Unsigned8 R_OBIS_DistributeRegisterActivationClass(
*               :     Unsigned16 master_index,
*               :     Unsigned16 child_index,
*               :     req_type_t req_type,
*               :     service_type_t service_type,
*               :     st_Cosem_Attr_Desc *p_descriptor,
*               :     Unsigned8	*pdata,
*               :     Unsigned8	*p_out_buf,
*               :     Unsigned16 *p_out_len
*               : );
* Description   : Distributor function for DRegister class
* Arguments     : Unsigned16 master_index -
*               :    [In] Id of the row in master list
*               : Unsigned16 child_index -
*               :    [In] Id of the row in child list
*               : req_type_t req_type -
*               :    [In] Request type (GET/SET/ACTION)
*               : service_type_t service_type -
*               :    [In] Service type (NORMAL/BLOCK/LIST)
*               : st_Cosem_Attr_Desc *p_descriptor -
*               :    [In] COSEM object descriptor
*               : Unsigned8 * pdata -
*               :    [In] Data in case of SET/ACTION
*               : Unsigned8 * p_out_buf -
*               :    [Out] Output buffer for encoded data
*               : Unsigned16 * p_out_len -
*               :    [Out] Output buffer length for encoded data
* Function Calls: TBD
* Return Value  : Unsigned8, distribution status
******************************************************************************/
#if (defined(USED_CLASS_06) && USED_CLASS_06 == 1)
Unsigned8 R_OBIS_DistributeRegisterActivationClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
)
{
	/* Result of response, def. is not Matched */
	Unsigned8				i, j;
	Unsigned8				response_result = TYPE_UNMATCH;
	class06_child_record_t	*p_child_record;
	obj_def_t				*p_obj_def_child;
	register_act_mask_t		*p_register_act_mask_child;
	Unsigned16				temp_out_len;

	p_child_record = (class06_child_record_t *)(
		R_OBIS_FindChildRecordByIndex(master_index, child_index)
	);

	/* Classify the code based on Attribute or Method request */
	if (req_type == REQ_TYPE_GET || req_type == REQ_TYPE_SET)
	{
		switch (p_descriptor->Attr_ID)
		{
			case 0: /* attribute 0 */
				/* Not support */
				break;

			case 1: /* logical_name */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,							/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,	/* Max length of g_ServiceBuffer */
						ATTR_TYPE_OCTET_STRING,				/* Octet-string */
						p_descriptor->Instance_ID,			/* OBIS code from the request */
						6);									/* OBIS code is 6 chars */

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, OBIS code is read only */
				}
				break;

			case 2:	/* register_assignment  */

				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				if (req_type == REQ_TYPE_GET)
				{
					/*
					 * obj_def_t array
					 */
					*p_out_buf++ = ATTR_TYPE_ARRAY;								/* Array type */
					*p_out_buf++ = p_child_record->nr_register_assignment;		/* No. elements */
					*p_out_len   = 2;

					/* obj_def_t elements */
					for (i = 0; i < p_child_record->nr_register_assignment; i ++)
					{
						p_obj_def_child = (obj_def_t *)(p_child_record->p_register_assignment + i);
						*p_out_buf++ = ATTR_TYPE_STRUCTURE;						/* Structure type */
						*p_out_buf++ = 2;										/* 2 elements */
						*p_out_len   += 2;
						/* class_id */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
							ATTR_TYPE_LONG_UNSIGNED,										/* Type */
							(Unsigned8 *)&p_obj_def_child->class_id,						/* Buffer */
							sizeof(Unsigned16)												/* Length */
						);
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;

						/* logical_name */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
							ATTR_TYPE_OCTET_STRING,											/* Type */
							p_obj_def_child->logical_name,									/* Buffer */
							6																/* Length */
						);
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;
					}

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing */
				}

				break;

			case 3:	/* mask_list */

				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				if (req_type == REQ_TYPE_GET)
				{
					/*
					 * register_act_mask_t array
					 */
					*p_out_buf++ = ATTR_TYPE_ARRAY;								/* Array type */
					*p_out_buf++ = p_child_record->nr_mask_list;				/* No. elements */
					*p_out_len   = 2;

					/* register_act_mask_t elements */
					for (i = 0; i < p_child_record->nr_mask_list; i ++)
					{
						p_register_act_mask_child = (register_act_mask_t *)(p_child_record->p_mask_list + i);
						*p_out_buf++ = ATTR_TYPE_STRUCTURE;						/* Structure type */
						*p_out_buf++ = 2;										/* 2 elements */
						*p_out_len   += 2;
						/* mask_name */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
							p_register_act_mask_child->mask_name.choice.type,				/* Type */
							(Unsigned8 *) p_register_act_mask_child->mask_name.buffer,		/* Buffer */
							p_register_act_mask_child->mask_name.choice.size				/* Length */
						);
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;

						/* index_list */
						*p_out_buf++ = ATTR_TYPE_ARRAY;								/* Array type */
						*p_out_buf++ = p_register_act_mask_child->nr_index_list;	/* No. elements */
						*p_out_len   += 2;

						for (j = 0; j < p_register_act_mask_child->nr_index_list; j ++)
						{
							temp_out_len = R_OBIS_EncodeAttribute(
								p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
								OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
								ATTR_TYPE_UNSIGNED,												/* Type */
								p_register_act_mask_child->p_index_list,						/* Buffer */
								1																/* Length */
							);
							p_out_buf = p_out_buf + temp_out_len;
							*p_out_len += temp_out_len;
						}

					}
					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, attribute is read only */
				}
				break;

			case 4:	/* active_mask */

				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				/* distribute the attribute */
				response_result = R_OBIS_DistributeAttrBuffer(
					&p_child_record->active_mask.access,		/* Access method option */
					p_child_record->active_mask.choice.type,	/* Type */
					p_child_record->active_mask.choice.size,	/* Size */
					p_child_record->active_mask.buffer,			/* Buffer of value */
					child_index,								/* Child ID */
					req_type,									/* Request type */
					service_type,								/* Service type */
					p_descriptor,								/* Pointer to COSEM descriptor */
					pdata,										/* Data from COSEM Appl layer */
					p_out_buf,									/* Data pointer for get */
					p_out_len									/* Data length pointer for get */
				);
				break;

			default:
				break;
		}
	}
	else if (req_type == REQ_TYPE_ACTION)
	{
		Unsigned8 b_block_transfer = 0;
		
		switch (p_descriptor->Attr_ID & 0x00FF)
		{
			case 1:	/* reset (data) */
			case 2:	/* add_mask (data) */
			case 3:	/* delete_mask (data) */
				/* User function take care for all */
				response_result = R_OBIS_DistributeByUserFunc(
					child_index,		/* child_index */	
					req_type,			/* req_type */
					service_type,		/* service_type */
					&b_block_transfer,	/* p_is_block_transfer */
					p_descriptor,		/* p_descriptor */	
					pdata,				/* pdata */			
					p_out_buf,			/* p_out_buf */		
					p_out_len			/* p_out_len */
				);
				/* Not support block transfer now, TODO in future */
				if (b_block_transfer != 0)
				{
					response_result = ACTION_RESULT_SCOPE_ACCESS_VIOLATED;
				}
				break;
			default:
				response_result = ACTION_RESULT_TYPE_UNMATCH;
				break;
		}
	}
	else
	{
		/* INTERNAL ERROR */
	}

	return (response_result);
}
#endif

#if (defined(USED_CLASS_07) && USED_CLASS_07 == 1)
/******************************************************************************
* Function Name : R_OBIS_DistributeProfileGenericClass
* Interface     : Unsigned8 R_OBIS_DistributeProfileGenericClass(
*               :     Unsigned16 master_index,
*               :     Unsigned16 child_index,
*               :     req_type_t req_type,
*               :     service_type_t service_type,
*               :     st_Cosem_Attr_Desc *p_descriptor,
*               :     Unsigned8	*pdata,
*               :     Unsigned8	*p_out_buf,
*               :     Unsigned16 *p_out_len
*               : );
* Description   : Distributor function for Clock class
* Arguments     : Unsigned16 master_index -
*               :    [In] Id of the row in master list
*               : Unsigned16 child_index -
*               :    [In] Id of the row in child list
*               : req_type_t req_type -
*               :    [In] Request type (GET/SET/ACTION)
*               : service_type_t service_type -
*               :    [In] Service type (NORMAL/BLOCK/LIST)
*               : st_Cosem_Attr_Desc *p_descriptor -
*               :    [In] COSEM object descriptor
*               : Unsigned8 * pdata -
*               :    [In] Data in case of SET/ACTION
*               : Unsigned8 * p_out_buf -
*               :    [Out] Output buffer for encoded data
*               : Unsigned16 * p_out_len -
*               :    [Out] Output buffer length for encoded data
* Function Calls: TBD
* Return Value  : Unsigned8, distribution status
******************************************************************************/
Unsigned8 R_OBIS_DistributeProfileGenericClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
)
{
	/* Result of response, def. is not Matched */
	Unsigned8				response_result = TYPE_UNMATCH;
	class07_child_record_t	*p_child_record;

	p_child_record = (class07_child_record_t *)(
		R_OBIS_FindChildRecordByIndex(master_index, child_index)
	);

	/* Classify the code based on Attribute or Method request */
	if (req_type == REQ_TYPE_GET || req_type == REQ_TYPE_SET)
	{
		switch (p_descriptor->Attr_ID)
		{
			case 0: /* attribute 0 */
				/* Not support */
				break;

			case 1: /* logical_name */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,							/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,	/* Max length of g_ServiceBuffer */
						ATTR_TYPE_OCTET_STRING,				/* Octet-string */
						p_descriptor->Instance_ID,			/* OBIS code from the request */
						6);									/* OBIS code is 6 chars */

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, OBIS code is read only */
				}
				break;

			case 0x0002:	/* buffer (normal) */
				if (req_type == REQ_TYPE_GET)
				{
					response_result = R_OBIS_EncodeGenericBuffer(
						p_child_record->p_buff,		/* [In] Pointer to buffer_t                       */
						0,							/* [In] selective_access                          */
						master_index,				/* [In] Id of the row in master list              */
						child_index,				/* [In] Id of the row in child list               */
						pdata,						/* [In] Pointer to the selector (from Client)     */
						service_type,				/* [In] Service type                              */
						p_out_buf,					/* [Out] Data pointer for get                     */
						p_out_len					/* [Out] Data length pointer for get              */
					);
				}
				else
				{
					/* Do nothing, buffer is read only */
				}
				break;

			case 0x0102:	/* buffer (selective access) */
				if (req_type == REQ_TYPE_GET)
				{
					response_result = R_OBIS_EncodeGenericBuffer(
						p_child_record->p_buff,		/* [In] Pointer to buffer_t                       */
						1,							/* [In] selective_access                          */
						master_index,				/* [In] Id of the row in master list              */
						child_index,				/* [In] Id of the row in child list               */
						pdata,						/* [In] Pointer to the selector (from Client)     */
						service_type,				/* [In] Service type                              */
						p_out_buf,					/* [Out] Data pointer for get                     */
						p_out_len					/* [Out] Data length pointer for get              */
					);
				}
				else
				{
					/* Do nothing, buffer is read only */
				}
				break;
			case 3:	/* capture_objects */
				{
					Unsigned8 b_block_transfer = 0;

					/* internal check */
					if (p_child_record == NULL)
					{
						break;	/* Fatal error, stop here */
					}

					/*
					 * capture_object_list pointer & length check
					 * user distribute if NULL list
					 */
					if (p_child_record->p_capture_objects == NULL ||
						p_child_record->p_capture_obj_length == NULL ||
						(p_child_record->p_capture_obj_length != NULL &&
						 *p_child_record->p_capture_obj_length == 0))
					{
						response_result = R_OBIS_DistributeByUserFunc(
							child_index,
							req_type,
							service_type,
							&b_block_transfer,
							p_descriptor,
							pdata,
							p_out_buf,
							p_out_len
						);
					}
					else
					{
						if (req_type == REQ_TYPE_GET)
						{
							/* Total number of capture object can be transfer by a request */
							#define TOTAL_CAPTURE_OBJECT_CAN_TRANSFER	\
								(Unsigned8)((OBIS_SERVICE_DATA_BUFFER_LENGTH -10) / 18)
							Unsigned16 total_transfered = 0;

							/* Start encoding */
							*p_out_len = 0;

							/* Encode an array header */
							if (service_type == SERVICE_TYPE_NORMAL ||
							g_OBIS_CaptureObjectID == 0)	/* For Attr 0 */
							{
								/* Reset the object counter that has been send */
								g_OBIS_CaptureObjectID = 0;
								*p_out_buf++ = ATTR_TYPE_ARRAY;
								/* Array length */
								*p_out_buf++ = (Unsigned8)*p_child_record->p_capture_obj_length;
								*p_out_len  += 2;
							}

							/* Content of array */
							{
								Unsigned16 encoded_length = 0;

								/* All elements of array */
								while (g_OBIS_CaptureObjectID < *p_child_record->p_capture_obj_length)
								{
									/* encode the array of capture_object_definition */
									encoded_length = R_OBIS_EncodeCaptureObject(
										p_out_buf,
										&(p_child_record->p_capture_objects[g_OBIS_CaptureObjectID])
									);

									/* next position in the p_out_buf */
									p_out_buf  += encoded_length;
									*p_out_len += encoded_length;
									/* Check overflow */
									if (*p_out_len > OBIS_SERVICE_DATA_BUFFER_LENGTH)
									{
										response_result = TYPE_UNMATCH;
										break;
									}
									else
									{
										/* count to check block transfer */
										g_OBIS_CaptureObjectID++;
										total_transfered++;

										/* Check end */
										if (total_transfered >= TOTAL_CAPTURE_OBJECT_CAN_TRANSFER ||
											g_OBIS_CaptureObjectID == *p_child_record->p_capture_obj_length)
										{
											response_result = DATA_ACCESS_RESULT_SUCCESS;
											/* indicate as block transfer when not finish the list */
											if (g_OBIS_CaptureObjectID < *p_child_record->p_capture_obj_length)
											{
												b_block_transfer = 1;
											}
											/* when finish the list, reset the counter for next run */
											else if (g_OBIS_CaptureObjectID == *p_child_record->p_capture_obj_length)
											{
												b_block_transfer = 0;
												g_OBIS_CaptureObjectID = 0;
											}

											break;
										}
									}
								}
							}
						}
						else if (req_type == REQ_TYPE_SET)
						{
							/* Do nothing */
						}
					}

					if (b_block_transfer == 1 &&
						response_result == DATA_ACCESS_RESULT_SUCCESS)
					{
						response_result = DATA_ACCESS_RESULT_PARTIAL;
					}
				}

				break;

			case 4:	/* capture_period */

				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				/* distribute the buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					NULL,								/* Access method option */
					ATTR_TYPE_DOUBLE_LONG_UNSIGNED,		/* Type */
					sizeof(Unsigned32),					/* Size */
					p_child_record->p_capture_period,	/* Buffer of value */
					child_index,						/* Child ID */
					req_type,							/* Request type */
					service_type,						/* Service type */
					p_descriptor,						/* Pointer to COSEM descriptor */
					pdata,								/* Data from COSEM Appl layer */
					p_out_buf,							/* Data pointer for get */
					p_out_len							/* Data length pointer for get */
				);

				break;

			case 5:	/* sort_method */

				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				/* distribute the buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					NULL,								/* Access method option */
					ATTR_TYPE_ENUM,						/* Type */
					sizeof(sort_method_t),				/* Size */
					p_child_record->p_sort_method,		/* Buffer of value */
					child_index,						/* Child ID */
					req_type,							/* Request type */
					service_type,						/* Service type */
					p_descriptor,						/* Pointer to COSEM descriptor */
					pdata,								/* Data from COSEM Appl layer */
					p_out_buf,							/* Data pointer for get */
					p_out_len							/* Data length pointer for get */
				);

				break;

			case 6:	/* sort_object */
				{
					Unsigned8 b_block_transfer = 0;

					/* internal check */
					if (p_child_record == NULL)
					{
						break;	/* Fatal error, stop here */
					}

					/* distribution */
					if (p_child_record->p_sort_object == NULL)
					{
						response_result = R_OBIS_DistributeByUserFunc(
							child_index,
							req_type,
							service_type,
							&b_block_transfer,
							p_descriptor,
							pdata,
							p_out_buf,
							p_out_len
						);
					}
					else
					{
						if (req_type == REQ_TYPE_GET)
						{
							/* encode the capture_object_definition */
							*p_out_len = R_OBIS_EncodeCaptureObject(
								p_out_buf,
								p_child_record->p_sort_object
							);

							if (*p_out_len == 0)
							{
								response_result = TYPE_UNMATCH;
							}
							else
							{
								response_result = DATA_ACCESS_RESULT_SUCCESS;
							}
						}
						else if (req_type == REQ_TYPE_SET)
						{
							/* Do nothing */
						}
					}

					if (b_block_transfer == 1 &&
						response_result == DATA_ACCESS_RESULT_SUCCESS)
					{
						response_result = DATA_ACCESS_RESULT_PARTIAL;
					}
				}

				break;

			case 7:	/* entries_in_use */

				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				/* distribute the buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					NULL,								/* Access method option */
					ATTR_TYPE_DOUBLE_LONG_UNSIGNED,		/* Type */
					sizeof(Unsigned32),					/* Size */
					p_child_record->p_entries_in_use,	/* Buffer of value */
					child_index,						/* Child ID */
					req_type,							/* Request type */
					service_type,						/* Service type */
					p_descriptor,						/* Pointer to COSEM descriptor */
					pdata,								/* Data from COSEM Appl layer */
					p_out_buf,							/* Data pointer for get */
					p_out_len							/* Data length pointer for get */
				);

				break;

			case 8:	/* profile_entries */

				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				/* distribute the buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					NULL,								/* Access method option */
					ATTR_TYPE_DOUBLE_LONG_UNSIGNED,		/* Type */
					sizeof(Unsigned32),					/* Size */
					p_child_record->p_profile_entries,	/* Buffer of value */
					child_index,						/* Child ID */
					req_type,							/* Request type */
					service_type,						/* Service type */
					p_descriptor,						/* Pointer to COSEM descriptor */
					pdata,								/* Data from COSEM Appl layer */
					p_out_buf,							/* Data pointer for get */
					p_out_len							/* Data length pointer for get */
				);

				break;

			default:
				break;
		}
	}
	else if (req_type == REQ_TYPE_ACTION)
	{
		Unsigned8 b_block_transfer = 0;
		
		switch (p_descriptor->Attr_ID & 0x00FF)
		{
			case 1:	/* reset (data) */
			case 2: /* capture (data) */
			case 3: /* version 0: get_buffer_by_range */
			case 4: /* version 0: get_buffer_by_index */
				/* User function take care for all */
				response_result = R_OBIS_DistributeByUserFunc(
					child_index,		/* child_index */	
					req_type,			/* req_type */
					service_type,		/* service_type */
					&b_block_transfer,	/* p_is_block_transfer */
					p_descriptor,		/* p_descriptor */	
					pdata,				/* pdata */			
					p_out_buf,			/* p_out_buf */		
					p_out_len			/* p_out_len */
				);
				
				/* Not support block transfer now, TODO in future */
				if (b_block_transfer != 0)
				{
					response_result = ACTION_RESULT_SCOPE_ACCESS_VIOLATED;
				}
				
				break;
			
			default:
				response_result = ACTION_RESULT_TYPE_UNMATCH;
				break;
		}
	}
	else
	{
		/* INTERNAL ERROR */
	}

	return (response_result);
}
#endif

/******************************************************************************
* Function Name : R_OBIS_DistributeClockClass
* Interface     : Unsigned8 R_OBIS_DistributeClockClass(
*               :     Unsigned16 master_index,
*               :     Unsigned16 child_index,
*               :     req_type_t req_type,
*               :     service_type_t service_type,
*               :     st_Cosem_Attr_Desc *p_descriptor,
*               :     Unsigned8	*pdata,
*               :     Unsigned8	*p_out_buf,
*               :     Unsigned16 *p_out_len
*               : );
* Description   : Distributor function for Clock class
* Arguments     : Unsigned16 master_index -
*               :    [In] Id of the row in master list
*               : Unsigned16 child_index -
*               :    [In] Id of the row in child list
*               : req_type_t req_type -
*               :    [In] Request type (GET/SET/ACTION)
*               : service_type_t service_type -
*               :    [In] Service type (NORMAL/BLOCK/LIST)
*               : st_Cosem_Attr_Desc *p_descriptor -
*               :    [In] COSEM object descriptor
*               : Unsigned8 * pdata -
*               :    [In] Data in case of SET/ACTION
*               : Unsigned8 * p_out_buf -
*               :    [Out] Output buffer for encoded data
*               : Unsigned16 * p_out_len -
*               :    [Out] Output buffer length for encoded data
* Function Calls: TBD
* Return Value  : Unsigned8, distribution status
******************************************************************************/
#if (defined(USED_CLASS_08) && USED_CLASS_08 == 1)
Unsigned8 R_OBIS_DistributeClockClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
)
{
	/* Result of response, def. is not Matched */
	Unsigned8				response_result = TYPE_UNMATCH;
	class08_child_record_t	*p_child_record;

	p_child_record = (class08_child_record_t *)(
		R_OBIS_FindChildRecordByIndex(master_index, child_index)
	);

	/* Classify the code based on Attribute or Method request */
	if (req_type == REQ_TYPE_GET || req_type == REQ_TYPE_SET)
	{
		switch (p_descriptor->Attr_ID)
		{
			case 0: /* attribute 0 */
				/* Not support */
				break;

			case 1: /* logical_name */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,							/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,	/* Max length of g_ServiceBuffer */
						ATTR_TYPE_OCTET_STRING,				/* Octet-string */
						p_descriptor->Instance_ID,			/* OBIS code from the request */
						6);									/* OBIS code is 6 chars */

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, OBIS code is read only */
				}
				break;

			case 2:	/* time */
				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}
				if (req_type == REQ_TYPE_SET)
				{
					if (R_OBIS_IsValidDateTime(pdata)== 0) // Invalid
					{
						break;
					}
				}
				/* distribute the buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					&p_child_record->time.access,		/* Access method option */
					p_child_record->time.choice.type,	/* Type */
					p_child_record->time.choice.size,	/* Size */
					p_child_record->time.buffer,		/* Buffer of value */
					child_index,						/* Child ID */
					req_type,							/* Request type */
					service_type,						/* Service type */
					p_descriptor,						/* Pointer to COSEM descriptor */
					pdata,								/* Data from COSEM Appl layer */
					p_out_buf,							/* Data pointer for get */
					p_out_len							/* Data length pointer for get */
				);

				break;
			case 4:	/* status */
				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				/* distribute the buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					&p_child_record->status.access,		/* Access method option */
					p_child_record->status.choice.type,	/* Type */
					p_child_record->status.choice.size,	/* Size */
					p_child_record->status.buffer,		/* Buffer of value */
					child_index,						/* Child ID */
					req_type,							/* Request type */
					service_type,						/* Service type */
					p_descriptor,						/* Pointer to COSEM descriptor */
					pdata,								/* Data from COSEM Appl layer */
					p_out_buf,							/* Data pointer for get */
					p_out_len							/* Data length pointer for get */
				);

				break;

			case 3:	/* time_zone */
			case 5:	/* daylight_savings_begin */
			case 6:	/* daylight_savings_end */
			case 7:	/* daylight_savings_deviation */
			case 8:	/* daylight_savings_enabled */
			case 9:	/* clock_base */
				{
					attr_type_t	type;
					Integer16	size;
					void		*buffer = NULL;

					if (p_child_record == NULL)
					{
						break;
					}
					else if (p_descriptor->Attr_ID == 3)		/* time_zone */
					{
						type   = ATTR_TYPE_LONG;
						size   = sizeof(Integer16);
						buffer = p_child_record->p_time_zone;
					}
					else if (p_descriptor->Attr_ID == 5)		/* daylight_savings_begin */
					{
						type   = ATTR_TYPE_OCTET_STRING;
						size   = sizeof(date_time_t);
						buffer = p_child_record->p_daylight_savings_begin;
					}
					else if (p_descriptor->Attr_ID == 6)		/* daylight_savings_end */
					{
						type   = ATTR_TYPE_OCTET_STRING;
						size   = sizeof(date_time_t);
						buffer = p_child_record->p_daylight_savings_end;
					}
					else if (p_descriptor->Attr_ID == 7)		/* daylight_savings_deviation */
					{
						type   = ATTR_TYPE_INTEGER;
						size   = sizeof(Integer8);
						buffer = p_child_record->p_daylight_savings_deviation;
					}
					else if (p_descriptor->Attr_ID == 8)		/* daylight_savings_enabled */
					{
						type   = ATTR_TYPE_BOOLEAN;
						size   = sizeof(Unsigned8);
						buffer = p_child_record->p_daylight_savings_enabled;
					}
					else if (p_descriptor->Attr_ID == 9)		/* clock_base */
					{
						type   = ATTR_TYPE_ENUM;
						size   = sizeof(Unsigned8);
						buffer = p_child_record->p_clock_base;
					}

					/* distribute the buffer */
					response_result = R_OBIS_DistributeAttrBuffer(
						NULL,			/* Access method option */
						type,			/* Type */
						size,			/* Size */
						buffer,			/* Buffer of value */
						child_index,	/* Child ID */
						req_type,		/* Request type */
						service_type,	/* Service type */
						p_descriptor,	/* Pointer to COSEM descriptor */
						pdata,			/* Data from COSEM Appl layer */
						p_out_buf,		/* Data pointer for get */
						p_out_len		/* Data length pointer for get */
					);
					if (p_descriptor->Attr_ID == 3 && req_type == REQ_TYPE_SET)
					{
						R_USER_Set_RTC_timezone();
					}
				}
				break;

			default:
				break;
		}
	}
	else if (req_type == REQ_TYPE_ACTION)
	{
		Unsigned8 b_block_transfer = 0;
		
		switch (p_descriptor->Attr_ID & 0x00FF)
		{
			case 1:	/* adjust_to_quarter (data) */
			case 2:	/* adjust_to_measuring_period (data) */
			case 3:	/* adjust_to_minute (data) */
			case 4:	/* adjust_to_preset_time (data) */
			case 5:	/* preset_adjusting_time (data) */
			case 6:	/* shift_time (data) */
				/* User function take care for all */
				response_result = R_OBIS_DistributeByUserFunc(
					child_index,		/* child_index */	
					req_type,			/* req_type */
					service_type,		/* service_type */
					&b_block_transfer,	/* p_is_block_transfer */
					p_descriptor,		/* p_descriptor */	
					pdata,				/* pdata */			
					p_out_buf,			/* p_out_buf */		
					p_out_len			/* p_out_len */
				);
				/* Not support block transfer now, TODO in future */
				if (b_block_transfer != 0)
				{
					response_result = ACTION_RESULT_SCOPE_ACCESS_VIOLATED;
				}
				break;
			default:
				response_result = ACTION_RESULT_TYPE_UNMATCH;
				break;
		}
	}
	else
	{
		/* INTERNAL ERROR */
	}

	return (response_result);
}
#endif

/******************************************************************************
* Function Name : R_OBIS_DistributeScriptTableClass
* Interface     : Unsigned8 R_OBIS_DistributeScriptTableClass(
*               :     Unsigned16 master_index,
*               :     Unsigned16 child_index,
*               :     req_type_t req_type,
*               :     service_type_t service_type,
*               :     st_Cosem_Attr_Desc *p_descriptor,
*               :     Unsigned8	*pdata,
*               :     Unsigned8	*p_out_buf,
*               :     Unsigned16 *p_out_len
*               : );
* Description   : Distributor function for Clock class
* Arguments     : Unsigned16 master_index -
*               :    [In] Id of the row in master list
*               : Unsigned16 child_index -
*               :    [In] Id of the row in child list
*               : req_type_t req_type -
*               :    [In] Request type (GET/SET/ACTION)
*               : service_type_t service_type -
*               :    [In] Service type (NORMAL/BLOCK/LIST)
*               : st_Cosem_Attr_Desc *p_descriptor -
*               :    [In] COSEM object descriptor
*               : Unsigned8 * pdata -
*               :    [In] Data in case of SET/ACTION
*               : Unsigned8 * p_out_buf -
*               :    [Out] Output buffer for encoded data
*               : Unsigned16 * p_out_len -
*               :    [Out] Output buffer length for encoded data
* Function Calls: TBD
* Return Value  : Unsigned8, distribution status
******************************************************************************/
#if (defined(USED_CLASS_09) && USED_CLASS_09 == 1)
Unsigned8 R_OBIS_DistributeScriptTableClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
)
{
	/* Result of response, def. is not Matched */
	Unsigned8				response_result = TYPE_UNMATCH;
	Unsigned8				i;
	Unsigned8				j;
	Unsigned16				temp_out_len;
	script_t				*p_child_script;
	script_action_spec_t	*p_child_action_spec;
	class09_child_record_t	*p_child_record;

	p_child_record = (class09_child_record_t *)(
		R_OBIS_FindChildRecordByIndex(master_index, child_index)
	);

	/* Classify the code based on Attribute or Method request */
	if (req_type == REQ_TYPE_GET || req_type == REQ_TYPE_SET)
	{
		switch (p_descriptor->Attr_ID)
		{
			case 0: /* attribute 0 */
				/* Not support */
				break;

			case 1: /* logical_name */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,							/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,	/* Max length of g_ServiceBuffer */
						ATTR_TYPE_OCTET_STRING,				/* Octet-string */
						p_descriptor->Instance_ID,			/* OBIS code from the request */
						6);									/* OBIS code is 6 chars */

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, OBIS code is read only */
				}
				break;

			case 2:	/* scripts */
				if (req_type == REQ_TYPE_GET)
				{
					/*
					 * Script array
					 */
					*p_out_buf++ = ATTR_TYPE_ARRAY;								/* Array type */
					*p_out_buf++ = p_child_record->nr_scripts;					/* No. elements */
					*p_out_len   = 2;

					/* Script elements */
					for (i = 0; i < p_child_record->nr_scripts; i ++)
					{
						p_child_script = (script_t *)(p_child_record->p_scripts + i);
						*p_out_buf++ = ATTR_TYPE_STRUCTURE;						/* Structure type */
						*p_out_buf++ = 2;										/* 2 elements */
						*p_out_len   += 2;
						/* Script ID */
						*p_out_buf++ = ATTR_TYPE_LONG_UNSIGNED;
						*p_out_buf++ = (Unsigned8)(p_child_script->script_id >> 8);
						*p_out_buf++ = (Unsigned8)(p_child_script->script_id  & 0xFF);
						*p_out_len   += 3;
						/* Action specification array */
						*p_out_buf++ = ATTR_TYPE_ARRAY;							/* Structure type */
						*p_out_buf++ = p_child_script->nr_actions;				/* No. elements */
						*p_out_len   += 2;
						/* Action specification elements */
						for (j = 0; j < p_child_script->nr_actions; j ++)
						{
							p_child_action_spec = (script_action_spec_t *)(p_child_script->p_actions + j);
							*p_out_buf++ = ATTR_TYPE_STRUCTURE;					/* Structure type */
							*p_out_buf++ = 5;									/* 5 elements */
							*p_out_len   += 2;
							/* service id */
							*p_out_buf++ = ATTR_TYPE_ENUM;
							*p_out_buf++ = p_child_action_spec->service_id;
							*p_out_len   += 2;
							/* class id */
							*p_out_buf++ = ATTR_TYPE_LONG_UNSIGNED;
							*p_out_buf++ = (Unsigned8)(p_child_action_spec->class_id >> 8);
							*p_out_buf++ = (Unsigned8)(p_child_action_spec->class_id & 0xFF);
							*p_out_len   += 3;
							/* logical_name */
							*p_out_buf++ =  ATTR_TYPE_OCTET_STRING;
							*p_out_buf++ =  6;					/* OBIS code is 6 chars */
							*p_out_buf++ =  (Unsigned8)p_child_action_spec->logical_name[0];
							*p_out_buf++ =  (Unsigned8)p_child_action_spec->logical_name[1];
							*p_out_buf++ =  (Unsigned8)p_child_action_spec->logical_name[2];
							*p_out_buf++ =  (Unsigned8)p_child_action_spec->logical_name[3];
							*p_out_buf++ =  (Unsigned8)p_child_action_spec->logical_name[4];
							*p_out_buf++ =  (Unsigned8)p_child_action_spec->logical_name[5];
							*p_out_len   += 8;
							/* index */
							*p_out_buf++ = ATTR_TYPE_INTEGER;
							*p_out_buf++ = p_child_action_spec->index;
							*p_out_len   += 2;
							/* parameters */
							/* Encode */
							temp_out_len = R_OBIS_EncodeAttribute(
								p_out_buf,										/* Output buffer, pointed to g_ServiceBuffer */
								OBIS_SERVICE_DATA_BUFFER_LENGTH,				/* Max length of g_ServiceBuffer */
								p_child_action_spec->parameter.choice.type,		/* Type */
								p_child_action_spec->parameter.buffer,			/* Buffer */
								p_child_action_spec->parameter.choice.size		/* Length */
							);

							p_out_buf = p_out_buf + temp_out_len;
							*p_out_len += temp_out_len;
						}
					}

					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing , Script */
				}
				break;
			default:
				break;
		}
	}
	else if (req_type == REQ_TYPE_ACTION)
	{
		Unsigned8 b_block_transfer = 0;
		
		switch (p_descriptor->Attr_ID & 0x00FF)
		{
			case 1:	/* execute (data) */
				/* User function take care for all */
				response_result = R_OBIS_DistributeByUserFunc(
					child_index,		/* child_index */	
					req_type,			/* req_type */
					service_type,		/* service_type */
					&b_block_transfer,	/* p_is_block_transfer */
					p_descriptor,		/* p_descriptor */	
					pdata,				/* pdata */			
					p_out_buf,			/* p_out_buf */		
					p_out_len			/* p_out_len */
				);
				/* Not support block transfer now, TODO in future */
				if (b_block_transfer != 0)
				{
					response_result = ACTION_RESULT_SCOPE_ACCESS_VIOLATED;
				}
				break;
			default:
				response_result = ACTION_RESULT_TYPE_UNMATCH;
				break;
		}
	}
	else
	{
		/* INTERNAL ERROR */
	}

	return (response_result);
}
#endif

/******************************************************************************
* Function Name : R_OBIS_DistributeScheduleClass
* Interface     : Unsigned8 R_OBIS_DistributeScheduleClass(
*               :     Unsigned16 master_index,
*               :     Unsigned16 child_index,
*               :     req_type_t req_type,
*               :     service_type_t service_type,
*               :     st_Cosem_Attr_Desc *p_descriptor,
*               :     Unsigned8	*pdata,
*               :     Unsigned8	*p_out_buf,
*               :     Unsigned16 *p_out_len
*               : );
* Description   : Distributor function for Clock class
* Arguments     : Unsigned16 master_index -
*               :    [In] Id of the row in master list
*               : Unsigned16 child_index -
*               :    [In] Id of the row in child list
*               : req_type_t req_type -
*               :    [In] Request type (GET/SET/ACTION)
*               : service_type_t service_type -
*               :    [In] Service type (NORMAL/BLOCK/LIST)
*               : st_Cosem_Attr_Desc *p_descriptor -
*               :    [In] COSEM object descriptor
*               : Unsigned8 * pdata -
*               :    [In] Data in case of SET/ACTION
*               : Unsigned8 * p_out_buf -
*               :    [Out] Output buffer for encoded data
*               : Unsigned16 * p_out_len -
*               :    [Out] Output buffer length for encoded data
* Function Calls: TBD
* Return Value  : Unsigned8, distribution status
******************************************************************************/
#if (defined(USED_CLASS_10) && USED_CLASS_10 == 1)
Unsigned8 R_OBIS_DistributeScheduleClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
)
{
	/* Result of response, def. is not Matched */
	Unsigned8				response_result = TYPE_UNMATCH;
	Unsigned8				i;
	Unsigned8				nr_entries;
	Unsigned16				temp_out_len;
	schedule_table_entry_t	*p_child_schedule_entry;

	class10_child_record_t	*p_child_record;

	p_child_record = (class10_child_record_t *)(
		R_OBIS_FindChildRecordByIndex(master_index, child_index)
	);

	/* Classify the code based on Attribute or Method request */
	if (req_type == REQ_TYPE_GET || req_type == REQ_TYPE_SET)
	{
		switch (p_descriptor->Attr_ID)
		{
			case 0: /* attribute 0 */
				/* Not support */
				break;

			case 1: /* logical_name */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,							/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,	/* Max length of g_ServiceBuffer */
						ATTR_TYPE_OCTET_STRING,				/* Octet-string */
						p_descriptor->Instance_ID,			/* OBIS code from the request */
						6);									/* OBIS code is 6 chars */

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, OBIS code is read only */
				}
				break;

			case 2:	/* entries  */
				if (req_type == REQ_TYPE_GET)
				{
					/*
					 * schedule_table_entry array
					 */
					nr_entries = (Unsigned8)*p_child_record->p_nr_entries;
					*p_out_buf++ = ATTR_TYPE_ARRAY;								/* Structure type */
					*p_out_buf++ = nr_entries;									/* No. elements */
					*p_out_len   = 2;

					/* schedule_table_entry */
					for (i = 0; i < nr_entries; i ++)
					{
						p_child_schedule_entry = (schedule_table_entry_t *)(p_child_record->p_entries + i);
						*p_out_buf++ = ATTR_TYPE_STRUCTURE;						/* Structure type */
						*p_out_buf++ = 10;										/* 10 elements */
						*p_out_len   += 2;
						/* Index */
						*p_out_buf++ = ATTR_TYPE_LONG_UNSIGNED;
						*p_out_buf++ = (Unsigned8)(p_child_schedule_entry->index >> 8);
						*p_out_buf++ = (Unsigned8)(p_child_schedule_entry->index  & 0xFF);
						*p_out_len   += 3;
						/* enable */
						*p_out_buf++ = ATTR_TYPE_BOOLEAN;
						*p_out_buf++ = (Unsigned8)(*(p_child_schedule_entry->p_enable));
						*p_out_len   += 2;
						/* script_logical_name */
						*p_out_buf++ =  ATTR_TYPE_OCTET_STRING;
						*p_out_buf++ =  6;										/* OBIS code is 6 chars */
						*p_out_buf++ =  (Unsigned8)p_child_schedule_entry->script_logical_name[0];
						*p_out_buf++ =  (Unsigned8)p_child_schedule_entry->script_logical_name[1];
						*p_out_buf++ =  (Unsigned8)p_child_schedule_entry->script_logical_name[2];
						*p_out_buf++ =  (Unsigned8)p_child_schedule_entry->script_logical_name[3];
						*p_out_buf++ =  (Unsigned8)p_child_schedule_entry->script_logical_name[4];
						*p_out_buf++ =  (Unsigned8)p_child_schedule_entry->script_logical_name[5];
						*p_out_len   += 8;
						/* Script ID */
						*p_out_buf++ = ATTR_TYPE_LONG_UNSIGNED;
						*p_out_buf++ = (Unsigned8)(p_child_schedule_entry->script_selector >> 8);
						*p_out_buf++ = (Unsigned8)(p_child_schedule_entry->script_selector  & 0xFF);
						*p_out_len   += 3;

						/* switch_time */
						/* Encode */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,											/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,					/* Max length of g_ServiceBuffer */
							ATTR_TYPE_OCTET_STRING,								/* Type */
							(Unsigned8 *)p_child_schedule_entry->p_switch_time,	/* Buffer */
							sizeof(time_t)										/* Length */
						);

						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;

						/* validity_window */
						*p_out_buf++ = ATTR_TYPE_LONG_UNSIGNED;
						*p_out_buf++ = (Unsigned8)(p_child_schedule_entry->validity_window >> 8);
						*p_out_buf++ = (Unsigned8)(p_child_schedule_entry->validity_window  & 0xFF);
						*p_out_len   += 3;

						/* exec_weekdays */
						/* Encode */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,											/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,					/* Max length of g_ServiceBuffer */
							ATTR_TYPE_BIT_STRING,								/* Type */
							&p_child_schedule_entry->exec_weekdays,				/* Buffer */
							1													/* Length */
						);

						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;

						/* exec_specdays */
						/* Encode */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,											/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,					/* Max length of g_ServiceBuffer */
							ATTR_TYPE_BIT_STRING,								/* Type */
							&p_child_schedule_entry->exec_specdays,				/* Buffer */
							1													/* Length */
						);

						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;

						/* begin_date */
						/* Encode */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,											/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,					/* Max length of g_ServiceBuffer */
							ATTR_TYPE_OCTET_STRING,								/* Type */
							(Unsigned8 *)p_child_schedule_entry->p_begin_date,	/* Buffer */
							5													/* Length: sizeof(date_t) */
						);

						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;

						/* end_date */
						/* Encode */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,											/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,					/* Max length of g_ServiceBuffer */
							ATTR_TYPE_OCTET_STRING,								/* Type */
							(Unsigned8 *)p_child_schedule_entry->p_end_date,	/* Buffer */
							5													/* Length: sizeof(date_t) */
						);

						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;

					}


					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, entries of Schedule */
				}
				break;
			default:
				break;
		}
	}
	else if (req_type == REQ_TYPE_ACTION)
	{
		Unsigned8 b_block_transfer = 0;
		
		switch (p_descriptor->Attr_ID & 0x00FF)
		{
			case 1:	/* enable/disable (data) */
			case 2:	/* insert (data) */
			case 3:	/* delete (data) */
				/* User function take care for all */
				response_result = R_OBIS_DistributeByUserFunc(
					child_index,		/* child_index */	
					req_type,			/* req_type */
					service_type,		/* service_type */
					&b_block_transfer,	/* p_is_block_transfer */
					p_descriptor,		/* p_descriptor */	
					pdata,				/* pdata */			
					p_out_buf,			/* p_out_buf */		
					p_out_len			/* p_out_len */
				);
				/* Not support block transfer now, TODO in future */
				if (b_block_transfer != 0)
				{
					response_result = ACTION_RESULT_SCOPE_ACCESS_VIOLATED;
				}
				break;
			default:
				response_result = ACTION_RESULT_TYPE_UNMATCH;
				break;
		}
	}
	else
	{
		/* INTERNAL ERROR */
	}

	return (response_result);
}
#endif

/******************************************************************************
* Function Name : R_OBIS_DistributeSpecialDaysTableClass
* Interface     : Unsigned8 R_OBIS_DistributeSpecialDaysTableClass(
*               :     Unsigned16 master_index,
*               :     Unsigned16 child_index,
*               :     req_type_t req_type,
*               :     service_type_t service_type,
*               :     st_Cosem_Attr_Desc *p_descriptor,
*               :     Unsigned8	*pdata,
*               :     Unsigned8	*p_out_buf,
*               :     Unsigned16 *p_out_len
*               : );
* Description   : Distributor function for Clock class
* Arguments     : Unsigned16 master_index -
*               :    [In] Id of the row in master list
*               : Unsigned16 child_index -
*               :    [In] Id of the row in child list
*               : req_type_t req_type -
*               :    [In] Request type (GET/SET/ACTION)
*               : service_type_t service_type -
*               :    [In] Service type (NORMAL/BLOCK/LIST)
*               : st_Cosem_Attr_Desc *p_descriptor -
*               :    [In] COSEM object descriptor
*               : Unsigned8 * pdata -
*               :    [In] Data in case of SET/ACTION
*               : Unsigned8 * p_out_buf -
*               :    [Out] Output buffer for encoded data
*               : Unsigned16 * p_out_len -
*               :    [Out] Output buffer length for encoded data
* Function Calls: TBD
* Return Value  : Unsigned8, distribution status
******************************************************************************/
#if (defined(USED_CLASS_11) && USED_CLASS_11 == 1)
Unsigned8 R_OBIS_DistributeSpecialDaysTableClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
)
{
	/* Result of response, def. is not Matched */
	Unsigned8				response_result = TYPE_UNMATCH;
	Unsigned8				i;
	Unsigned8				nr_entries;
	Unsigned16				temp_out_len;
	spec_day_entry_t		*p_child_spec_day_entry;

	class11_child_record_t	*p_child_record;

	p_child_record = (class11_child_record_t *)(
		R_OBIS_FindChildRecordByIndex(master_index, child_index)
	);

	/* Classify the code based on Attribute or Method request */
	if (req_type == REQ_TYPE_GET || req_type == REQ_TYPE_SET)
	{
		switch (p_descriptor->Attr_ID)
		{
			case 0: /* attribute 0 */
				/* Not support */
				break;

			case 1: /* logical_name */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,										/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,				/* Max length of g_ServiceBuffer */
						ATTR_TYPE_OCTET_STRING,							/* Octet-string */
						p_descriptor->Instance_ID,						/* OBIS code from the request */
						6);												/* OBIS code is 6 chars */

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, OBIS code is read only */
				}
				break;

			case 2:	/* entries  */
				if (req_type == REQ_TYPE_GET)
				{
					/*
					 * spec_day_entry_t array
					 */
					nr_entries = (Unsigned8)*p_child_record->p_nr_entries;
					*p_out_buf++ = ATTR_TYPE_ARRAY;								/* Structure type */
					*p_out_buf++ = nr_entries;									/* No. elements */
					*p_out_len   = 2;

					/* spec_day_entry_t */
					for (i = 0; i < nr_entries; i ++)
					{
						p_child_spec_day_entry = (spec_day_entry_t *)(p_child_record->p_entries + i);
						*p_out_buf++ = ATTR_TYPE_STRUCTURE;						/* Structure type */
						*p_out_buf++ = 3;										/* 3 elements */
						*p_out_len   += 2;
						/* Index */
						*p_out_buf++ = ATTR_TYPE_LONG_UNSIGNED;
						*p_out_buf++ = (Unsigned8)(p_child_spec_day_entry->index >> 8);
						*p_out_buf++ = (Unsigned8)(p_child_spec_day_entry->index  & 0xFF);
						*p_out_len   += 3;

						/* specialday_date */
						/* Encode */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
							ATTR_TYPE_OCTET_STRING,											/* Type */
							(Unsigned8 *)p_child_spec_day_entry->p_specialday_date,			/* Buffer */
							5																/* Length: sizeof(date_t) */
						);
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;

						/* day_id */
						*p_out_buf++ = ATTR_TYPE_UNSIGNED;
						*p_out_buf++ = (Unsigned8)(&(p_child_spec_day_entry->day_id));
						*p_out_len   += 2;
					}


					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, entries of Special Days Table */
				}
				break;
			default:
				break;
		}
	}
	else if (req_type == REQ_TYPE_ACTION)
	{
		Unsigned8 b_block_transfer = 0;
		
		switch (p_descriptor->Attr_ID & 0x00FF)
		{
			case 1:	/* insert (data) */
			case 2:	/* delete (data) */
				/* User function take care for all */
				response_result = R_OBIS_DistributeByUserFunc(
					child_index,		/* child_index */	
					req_type,			/* req_type */
					service_type,		/* service_type */
					&b_block_transfer,	/* p_is_block_transfer */
					p_descriptor,		/* p_descriptor */	
					pdata,				/* pdata */			
					p_out_buf,			/* p_out_buf */		
					p_out_len			/* p_out_len */
				);
				/* Not support block transfer now, TODO in future */
				if (b_block_transfer != 0)
				{
					response_result = ACTION_RESULT_SCOPE_ACCESS_VIOLATED;
				}
				break;
			default:
				response_result = ACTION_RESULT_TYPE_UNMATCH;
				break;
		}
	}
	else
	{
		/* INTERNAL ERROR */
	}

	return (response_result);
}
#endif

/******************************************************************************
* Function Name : R_OBIS_DistributeAssociationLNClass
* Interface     : Unsigned8 R_OBIS_DistributeAssociationLNClass(
*               :     Unsigned16 master_index,
*               :     Unsigned16 child_index,
*               :     req_type_t req_type,
*               :     service_type_t service_type,
*               :     st_Cosem_Attr_Desc *p_descriptor,
*               :     Unsigned8	*pdata,
*               :     Unsigned8	*p_out_buf,
*               :     Unsigned16 *p_out_len
*               : );
* Description   : Distributor function for Association LN
* Arguments     : Unsigned16 master_index -
*               :    [In] Id of the row in master list
*               : Unsigned16 child_index -
*               :    [In] Id of the row in child list
*               : req_type_t req_type -
*               :    [In] Request type (GET/SET/ACTION)
*               : service_type_t service_type -
*               :    [In] Service type (NORMAL/BLOCK/LIST)
*               : st_Cosem_Attr_Desc *p_descriptor -
*               :    [In] COSEM object descriptor
*               : Unsigned8 * pdata -
*               :    [In] Data in case of SET/ACTION
*               : Unsigned8 * p_out_buf -
*               :    [Out] Output buffer for encoded data
*               : Unsigned16 * p_out_len -
*               :    [Out] Output buffer length for encoded data
* Function Calls: TBD
* Return Value  : Unsigned8, distribution status
******************************************************************************/
#if (defined(USED_CLASS_15) && USED_CLASS_15 == 1)
Unsigned8 R_OBIS_DistributeAssociationLNClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
)
{
	/* Result of response, def. is not Matched */
	Unsigned8				response_result = TYPE_UNMATCH;
	Unsigned16 				assc_id_to_be_encoded = child_index;
	class15_child_record_t	*p_child_record;

	p_child_record = (class15_child_record_t *)(
		R_OBIS_FindChildRecordByIndex(master_index, child_index)
	);

	/* Classify the code based on Attribute or Method request */
	if (req_type == REQ_TYPE_GET || req_type == REQ_TYPE_SET)
	{
		switch (p_descriptor->Attr_ID)
		{
			case 0: /* attribute 0 */
				/* Not support */
				break;

			case 1: /* logical_name */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,							/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,	/* Max length of g_ServiceBuffer */
						ATTR_TYPE_OCTET_STRING,				/* Octet-string */
						p_descriptor->Instance_ID,			/* OBIS code from the request */
						6);									/* OBIS code is 6 chars */

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, OBIS code is read only */
				}
				break;

			case 2:	/* object_list */
				if (req_type == REQ_TYPE_GET)
				{
					Unsigned16 tem_len, old_master_ID;

					/* First start (re-start) asynchronous get service? */
					if (service_type == SERVICE_TYPE_NORMAL ||
						g_OBIS_IsAsyncEnd == 1)
					{
						g_OBIS_IsAsyncEnd = 1;		/* Mark as async ended to restart */
						if (R_OBIS_AsyncStart(assc_id_to_be_encoded) == 1)
						{
							/* Fatal error: does not have any object */
							return TYPE_UNMATCH;
						}
					}

					/* Prepare to encode object_list */
					*p_out_len = 0;
					old_master_ID = g_OBIS_MasterID;
					do
					{
						tem_len = 0;
						/*
						* Call to encode 1 object information
						* And check fatal error.
						* Checking to end here to prevent error
						*/
						if (R_OBIS_AsyncEncodeOneObject(
								g_OBIS_IsAsyncEnd,						/* First object or not */
								assc_id_to_be_encoded,					/* Id of association */
								g_OBIS_MasterID,						/* Master row id of object */
								g_OBIS_ChildID,							/* Child row id of object */
								p_out_buf,								/* Output encoded buffer */
								&tem_len) == 1 ||						/* Output encoded buffer length */
							tem_len == 0)
						{
							/* Fatal error: buffer overflow, need debug here */
							g_OBIS_IsAsyncEnd = 1;		/* Mark as async ended */
							return TYPE_UNMATCH;
						}

						/* Update value for next transfer */
						*p_out_len += tem_len;
						p_out_buf += tem_len;

						/* Mark as processing in async */
						g_OBIS_IsAsyncEnd = 0;

						/*
						* Get the next object in the table,
						* until end or the next object can be accessed */

						/* Call to get the next object & check end */
						if (R_OBIS_AsyncNextObject(assc_id_to_be_encoded) == 1)
						{
							g_OBIS_IsAsyncEnd = 1;
							break;
						}
						/* Forecast len of 1 object if change class */
						if (old_master_ID != g_OBIS_MasterID) 
						{
							if(R_OBIS_ProbeEncodeOneObject(
								g_OBIS_IsAsyncEnd,						/* First object or not */
								assc_id_to_be_encoded,					/* Id of association */
								g_OBIS_MasterID,						/* Master row id of object */
								g_OBIS_ChildID,							/* Child row id of object */
								p_out_buf,								/* Output encoded buffer */
								&tem_len) == 0							/* Output encoded buffer length */
								)
								{
									old_master_ID = g_OBIS_MasterID;
								}
						}
					} while ((*p_out_len + tem_len) < OBIS_SERVICE_DATA_BUFFER_LENGTH);
					//} while ((*p_out_len + tem_len) < OBIS_SERVICE_DATA_BUFFER_LENGTH &&
					//			old_master_ID == g_OBIS_MasterID);

					/*
					 * Get the result status here, based on async is end or not
					 */
					if (g_OBIS_IsAsyncEnd == 1)
					{
						response_result = DATA_ACCESS_RESULT_SUCCESS;
					}
					else
					{
						response_result = DATA_ACCESS_RESULT_PARTIAL;
					}
				}
				else
				{
					/* Do nothing, OBIS code is read only */
				}
				break;

			case 3:	/* associated_partners_id */
				if (req_type == REQ_TYPE_GET)
				{
					/*
					 * TODO: Group info of association into one utilized structure
					 * used for both Application layer and Object layer
					 */
					*p_out_buf++ = ATTR_TYPE_STRUCTURE;								/* Structure type */
					*p_out_buf++ = 2;												/* 2 elements */
					*p_out_len   = 2;
					{
						/* Client SAP */
						*p_out_buf++ = ATTR_TYPE_INTEGER;
						*p_out_buf++ = p_child_record->partners_id.client_SAP;

						/* Server SAP */
						*p_out_buf++ = ATTR_TYPE_LONG_UNSIGNED;
						*p_out_buf++ = (Unsigned8)(
							p_child_record->partners_id.server_SAP >> 8
						);
						*p_out_buf++ = (Unsigned8)(
							p_child_record->partners_id.server_SAP
						);
					}
					*p_out_len += 5;

					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, this attr can not be set */
				}
				break;

			case 4:	/* application_context_name */
				if (req_type == REQ_TYPE_GET)
				{
					/*
					 * Only context id of application context is used to change setting
					 */
					Unsigned8 ApplicationContextName[] = {0x60, 0x85, 0x74, 0x05, 0x08, 0x01, 0x00};
					ApplicationContextName[6] = p_child_record->context_id;

					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,								/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,		/* Max length of g_ServiceBuffer */
						ATTR_TYPE_OCTET_STRING,					/* Octet-string */
						(Unsigned8*)(ApplicationContextName),	/* application_context_name */
						sizeof(ApplicationContextName));

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, this attr can not be set */
				}
				break;

			case 5:	/* xDLMS_context_info */
				if (req_type == REQ_TYPE_GET)
				{
					/*
					 * TODO: Group info of association into one utilized structure
					 * used for both Application layer and Object layer
					 */
					static const Unsigned16 MaxReceivePduSize = MAX_SERVER_BUFFER_SIZE;
					static const Unsigned16 MaxSendPduSize = MAX_SERVER_BUFFER_SIZE;

					*p_out_buf++ = ATTR_TYPE_STRUCTURE;
					*p_out_buf++ = 6;
					*p_out_len = 2;

					{
						/* 1. Conformance: bitstring(24) */
						*p_out_buf++ = ATTR_TYPE_BIT_STRING;
						*p_out_buf++ = 24;
						*p_out_buf++ = p_child_record->conformance.bytes.bytes0;
						*p_out_buf++ = p_child_record->conformance.bytes.bytes1;
						*p_out_buf++ = p_child_record->conformance.bytes.bytes2;
						*p_out_len += 5;

						/* 2. max_receive_pdu_size: long-unsigned */
						*p_out_buf++ = ATTR_TYPE_LONG_UNSIGNED;
						*p_out_buf++ = (Unsigned8)(MaxReceivePduSize >> 8);
						*p_out_buf++ = (Unsigned8)(MaxReceivePduSize);
						*p_out_len += 3;

						/* 3. max_send_pdu_size: long-unsigned */
						*p_out_buf++ = ATTR_TYPE_LONG_UNSIGNED;
						*p_out_buf++ = (Unsigned8)(MaxSendPduSize >> 8);
						*p_out_buf++ = (Unsigned8)(MaxSendPduSize);
						*p_out_len += 3;

						/* 4. dlms_version_number */
						*p_out_buf++ = ATTR_TYPE_UNSIGNED;
						*p_out_buf++ = p_child_record->dlms_version;
						*p_out_len += 2;

						/* 5. quality_of_service */
						*p_out_buf++ = ATTR_TYPE_INTEGER;
						*p_out_buf++ = 0;
						*p_out_len += 2;

						/* 6. cyphering_info */
						*p_out_buf++ = ATTR_TYPE_OCTET_STRING;
						*p_out_buf++ = 0;
						*p_out_len += 2;
					}

					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, this attr can not be set */
				}
				break;

			case 6:	/* authentication_mechanism_name */
				if (req_type == REQ_TYPE_GET)
				{
					/*
					 * Only mechanism id is used
					 */
					Unsigned8 AuthenMechanismName[] = {0x60, 0x85, 0x74, 0x05, 0x08, 0x02, 0x00};
					AuthenMechanismName[6] = p_child_record->mechanism_id;

					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,								/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,		/* Max length of g_ServiceBuffer */
						ATTR_TYPE_OCTET_STRING,					/* Octet-string */
						(Unsigned8*)(AuthenMechanismName),		/* TODO, authentication_mechanism_name */
						sizeof(AuthenMechanismName));

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, this attr can not be set */
				}
				break;

			case 7:	/* secret */
					/* exist secret key */
					{
						Unsigned16	len;
						response_result = DATA_ACCESS_RESULT_SUCCESS;

						if (req_type == REQ_TYPE_GET)
						{
						/* Restore secret */
						len = strlen((void *)p_child_record->p_secret_key);
						R_USER_Class15_Secret_Restore(p_child_record->p_secret_key,
												len,
												p_child_record->mechanism_id
												);
						*p_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,										/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,				/* Max length of g_ServiceBuffer */
							ATTR_TYPE_OCTET_STRING,							/* Octet-string */
							(Unsigned8*)(p_child_record->p_secret_key),		/* secret */
							strlen((void *)p_child_record->p_secret_key));
						
						}
						else	// REQ_TYPE_SET
						{
							len = R_OBIS_DecodeAttribute(
								(Unsigned8*)(p_child_record->p_secret_key),		/* Output value buffer */
								strlen((void *)p_child_record->p_secret_key),	/* Size of buffer */
								ATTR_TYPE_OCTET_STRING,							/* Type */
								pdata,										/* Get from data pointer of HDLC protocol stack */
								OBIS_SERVICE_DATA_BUFFER_LENGTH					/* Encoded length from HDLC */
								);
							if (len == 0)
							{
								return TYPE_UNMATCH;
							}
							/* Backup secret */
							len = strlen((void *)p_child_record->p_secret_key);
							R_USER_Class15_Secret_Backup(p_child_record->p_secret_key,
													(Unsigned8)len,
													p_child_record->mechanism_id
													);
						}
					}
				break;
				
			case 8:	/* association_status */
				if (req_type == REQ_TYPE_GET)
				{
					/* exist association_status */
					if (p_child_record->p_status != NULL)
					{
						*p_out_buf++ = ATTR_TYPE_ENUM;					
						*p_out_buf++ = *(p_child_record->p_status);
						*p_out_len = 2;
						response_result = DATA_ACCESS_RESULT_SUCCESS;
					}
					else
					{
						response_result = OBJ_UNAVAILABLE;
					}
				}
				else
				{
					/* Do nothing, this attr can not be set */
				}
				break;
				
			case 9:	/* security_setup_reference */
			if(g_MasterTable[master_index].class_version == 0)
			{
				/* Version 0 has no attribute 9 */
			}
			else		/* Version 1 */
			{
				if (req_type == REQ_TYPE_GET)
				{
					if (p_child_record->p_security_setup_ref != NULL)
					{
						*p_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,											/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,					/* Max length of g_ServiceBuffer */
							ATTR_TYPE_OCTET_STRING,								/* Octet-string */
							(Unsigned8*)(p_child_record->p_security_setup_ref),	/* security setup reference */
							strlen((void *)p_child_record->p_security_setup_ref));
					}
					else
					{
						/* No security_setup_reference */
						*p_out_buf++ = ATTR_TYPE_OCTET_STRING;	
						*p_out_buf++ = 0;
						*p_out_len = 2;
					}
					
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, this attr can not be set */
				}
			}
				break;
			default:
				break;
		}
	}
	else if (req_type == REQ_TYPE_ACTION)
	{
		Unsigned8 b_block_transfer = 0;
		
		switch (p_descriptor->Attr_ID & 0x00FF)
		{
			case 1:	/* reply_to_HLS_authentication (data) */
			case 2:	/* change_HLS_secret (data) */
			case 3:	/* add_object (data) */
			case 4: /* remove_object (data) */
				/* User function take care for all */
				response_result = R_OBIS_DistributeByUserFunc(
					child_index,		/* child_index */	
					req_type,			/* req_type */
					service_type,		/* service_type */
					&b_block_transfer,	/* p_is_block_transfer */
					p_descriptor,		/* p_descriptor */	
					pdata,				/* pdata */			
					p_out_buf,			/* p_out_buf */		
					p_out_len			/* p_out_len */
				);
				/* Not support block transfer now, TODO in future */
				if (b_block_transfer != 0)
				{
					response_result = ACTION_RESULT_SCOPE_ACCESS_VIOLATED;
				}
				break;
			default:
				response_result = ACTION_RESULT_TYPE_UNMATCH;
				break;
		}
	}
	else
	{
		/* INTERNAL ERROR */
	}

	return (response_result);
}
#endif

/******************************************************************************
* Function Name : R_OBIS_DistributeSAPassignmentClass
* Interface     : Unsigned8 R_OBIS_DistributeSAPassignmentClass(
*               :     Unsigned16 master_index,
*               :     Unsigned16 child_index,
*               :     req_type_t req_type,
*               :     service_type_t service_type,
*               :     st_Cosem_Attr_Desc *p_descriptor,
*               :     Unsigned8	*pdata,
*               :     Unsigned8	*p_out_buf,
*               :     Unsigned16 *p_out_len
*               : );
* Description   : Distributor function for SAP Assigment
* Arguments     : Unsigned16 master_index -
*               :    [In] Id of the row in master list
*               : Unsigned16 child_index -
*               :    [In] Id of the row in child list
*               : req_type_t req_type -
*               :    [In] Request type (GET/SET/ACTION)
*               : service_type_t service_type -
*               :    [In] Service type (NORMAL/BLOCK/LIST)
*               : st_Cosem_Attr_Desc *p_descriptor -
*               :    [In] COSEM object descriptor
*               : Unsigned8 * pdata -
*               :    [In] Data in case of SET/ACTION
*               : Unsigned8 * p_out_buf -
*               :    [Out] Output buffer for encoded data
*               : Unsigned16 * p_out_len -
*               :    [Out] Output buffer length for encoded data
* Function Calls: TBD
* Return Value  : Unsigned8, distribution status
******************************************************************************/
#if (defined(USED_CLASS_17) && USED_CLASS_17 == 1)
Unsigned8 R_OBIS_DistributeSAPassignmentClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
)
{
	/* Result of response, def. is not Matched */
	Unsigned8				response_result = TYPE_UNMATCH;
	Unsigned8				i;
	Unsigned16				temp_out_len;

	asslist_entry_t			*p_child_asslist_entry;
	class17_child_record_t	*p_child_record;

	p_child_record = (class17_child_record_t *)(
		R_OBIS_FindChildRecordByIndex(master_index, child_index)
	);

	/* Classify the code based on Attribute or Method request */
	if (req_type == REQ_TYPE_GET || req_type == REQ_TYPE_SET)
	{
		switch (p_descriptor->Attr_ID)
		{
			case 0: /* attribute 0 */
				/* Not support */
				break;

			case 1: /* logical_name */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,							/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,	/* Max length of g_ServiceBuffer */
						ATTR_TYPE_OCTET_STRING,				/* Octet-string */
						p_descriptor->Instance_ID,			/* OBIS code from the request */
						6);									/* OBIS code is 6 chars */

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, OBIS code is read only */
				}
				break;

			case 2:	/* SAP_assignment_list */
				if (req_type == REQ_TYPE_GET)
				{
					/*
					 * asslist_type array
					 */
					*p_out_buf++ = ATTR_TYPE_ARRAY;								/* Structure type */
					*p_out_buf++ = p_child_record->nr_sap_assignment;			/* No. elements */
					*p_out_len   = 2;

					/* asslist_type elements */
					for (i = 0; i < p_child_record->nr_sap_assignment; i ++)
					{
						p_child_asslist_entry = (asslist_entry_t *)(p_child_record->p_sap_assignment + i);
						*p_out_buf++ = ATTR_TYPE_STRUCTURE;						/* Structure type */
						*p_out_buf++ = 2;										/* 2 elements */
						*p_out_len   += 2;
						/* SAP */
						*p_out_buf++ = ATTR_TYPE_LONG_UNSIGNED;
						*p_out_buf++ = (Unsigned8)(p_child_asslist_entry->sap >> 8);
						*p_out_buf++ = (Unsigned8)(p_child_asslist_entry->sap  & 0xFF);
						*p_out_len   += 3;
						/* logical_device_name */

						/* Encode */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
							p_child_asslist_entry->logical_device_name.choice.type,			/* Type */
							p_child_asslist_entry->logical_device_name.buffer,				/* Buffer */
							p_child_asslist_entry->logical_device_name.choice.size			/* Length */
						);

						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;
					}


					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, not change SAP_assignment_list */
				}
				break;
			default:
				break;
		}
	}
	else if (req_type == REQ_TYPE_ACTION)
	{
		Unsigned8 b_block_transfer = 0;
		
		switch (p_descriptor->Attr_ID & 0x00FF)
		{
			case 1:	/* connect_logical_device (data) */
				/* User function take care for all */
				response_result = R_OBIS_DistributeByUserFunc(
					child_index,		/* child_index */	
					req_type,			/* req_type */
					service_type,		/* service_type */
					&b_block_transfer,	/* p_is_block_transfer */
					p_descriptor,		/* p_descriptor */	
					pdata,				/* pdata */			
					p_out_buf,			/* p_out_buf */		
					p_out_len			/* p_out_len */
				);
				/* Not support block transfer now, TODO in future */
				if (b_block_transfer != 0)
				{
					response_result = ACTION_RESULT_SCOPE_ACCESS_VIOLATED;
				}
				break;
			default:
				response_result = ACTION_RESULT_TYPE_UNMATCH;
				break;
		}
	}
	else
	{
		/* INTERNAL ERROR */
	}

	return (response_result);
}
#endif

/******************************************************************************
* Function Name :  R_OBIS_DistributeImageTransferClass
* Interface     : Unsigned8  R_OBIS_DistributeImageTransferClass(
*               :     Unsigned16 master_index,
*               :     Unsigned16 child_index,
*               :     req_type_t req_type,
*               :     service_type_t service_type,
*               :     st_Cosem_Attr_Desc *p_descriptor,
*               :     Unsigned8	*pdata,
*               :     Unsigned8	*p_out_buf,
*               :     Unsigned16 *p_out_len
*               : );
* Description   : Distributor function for SAP Assigment
* Arguments     : Unsigned16 master_index -
*               :    [In] Id of the row in master list
*               : Unsigned16 child_index -
*               :    [In] Id of the row in child list
*               : req_type_t req_type -
*               :    [In] Request type (GET/SET/ACTION)
*               : service_type_t service_type -
*               :    [In] Service type (NORMAL/BLOCK/LIST)
*               : st_Cosem_Attr_Desc *p_descriptor -
*               :    [In] COSEM object descriptor
*               : Unsigned8 * pdata -
*               :    [In] Data in case of SET/ACTION
*               : Unsigned8 * p_out_buf -
*               :    [Out] Output buffer for encoded data
*               : Unsigned16 * p_out_len -
*               :    [Out] Output buffer length for encoded data
* Function Calls: TBD
* Return Value  : Unsigned8, distribution status
******************************************************************************/
#if (defined(USED_CLASS_18) && USED_CLASS_18 == 1)
Unsigned8  R_OBIS_DistributeImageTransferClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
)
{
	/* Result of response, def. is not Matched */
	Unsigned8				response_result = TYPE_UNMATCH;
	Unsigned8				i;
	Unsigned16				temp_out_len;

	asslist_entry_t			*p_child_asslist_entry;
	img_to_activate_info_t	*p_act_info_child;
	class18_child_record_t	*p_child_record;

	p_child_record = (class18_child_record_t *)(
		R_OBIS_FindChildRecordByIndex(master_index, child_index)
	);

	/* Classify the code based on Attribute or Method request */
	if (req_type == REQ_TYPE_GET || req_type == REQ_TYPE_SET)
	{
		switch (p_descriptor->Attr_ID)
		{
			case 0: /* attribute 0 */
				/* Not support */
				break;

			case 1: /* logical_name */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,							/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,	/* Max length of g_ServiceBuffer */
						ATTR_TYPE_OCTET_STRING,				/* Octet-string */
						p_descriptor->Instance_ID,			/* OBIS code from the request */
						6);									/* OBIS code is 6 chars */

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, OBIS code is read only */
				}
				break;

			case 2:	/* image_block_size */
				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				/* distribute the attr buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					NULL,								/* Access method option */
					ATTR_TYPE_DOUBLE_LONG_UNSIGNED,		/* Type */
					sizeof(Unsigned32),					/* Size */
					p_child_record->p_img_block_size,	/* Buffer of value */
					child_index,						/* Child ID */
					req_type,							/* Request type */
					service_type,						/* Service type */
					p_descriptor,						/* Pointer to COSEM descriptor */
					pdata,								/* Data from COSEM Appl layer */
					p_out_buf,							/* Data pointer for get */
					p_out_len							/* Data length pointer for get */
				);
				break;

			case 3:	/* image_transferred_blocks_status */
				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				/* distribute the attr buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					&p_child_record->img_trans_blocks_status.access,		/* Access method option */
					p_child_record->img_trans_blocks_status.choice.type,	/* Type */
					p_child_record->img_trans_blocks_status.choice.size,	/* Size */
					p_child_record->img_trans_blocks_status.buffer,			/* Buffer of value */
					child_index,											/* Child ID */
					req_type,												/* Request type */
					service_type,											/* Service type */
					p_descriptor,											/* Pointer to COSEM descriptor */
					pdata,													/* Data from COSEM Appl layer */
					p_out_buf,												/* Data pointer for get */
					p_out_len												/* Data length pointer for get */
				);
				break;

			case 4:	/* img_first_not_trans_block_nr */
				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				/* distribute the attr buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					&p_child_record->img_first_not_trans_block_nr.access,		/* Access method option */
					p_child_record->img_first_not_trans_block_nr.choice.type,	/* Type */
					p_child_record->img_first_not_trans_block_nr.choice.size,	/* Size */
					p_child_record->img_first_not_trans_block_nr.buffer,		/* Buffer of value */
					child_index,												/* Child ID */
					req_type,													/* Request type */
					service_type,												/* Service type */
					p_descriptor,												/* Pointer to COSEM descriptor */
					pdata,														/* Data from COSEM Appl layer */
					p_out_buf,													/* Data pointer for get */
					p_out_len													/* Data length pointer for get */
				);
				break;

			case 5:	/* img_trans_enabled */
				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				/* distribute the attr buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					NULL,								/* Access method option */
					ATTR_TYPE_BOOLEAN,					/* Type */
					sizeof(Unsigned8),					/* Size */
					p_child_record->p_img_trans_enabled,/* Buffer of value */
					child_index,						/* Child ID */
					req_type,							/* Request type */
					service_type,						/* Service type */
					p_descriptor,						/* Pointer to COSEM descriptor */
					pdata,								/* Data from COSEM Appl layer */
					p_out_buf,							/* Data pointer for get */
					p_out_len							/* Data length pointer for get */
				);
				break;

			case 6:	/* image_transfer_status */
				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				/* distribute the attr buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					&p_child_record->img_trans_status.access,				/* Access method option */
					p_child_record->img_trans_status.choice.type,			/* Type */
					p_child_record->img_trans_status.choice.size,			/* Size */
					p_child_record->img_trans_status.buffer,				/* Buffer of value */
					child_index,											/* Child ID */
					req_type,												/* Request type */
					service_type,											/* Service type */
					p_descriptor,											/* Pointer to COSEM descriptor */
					pdata,													/* Data from COSEM Appl layer */
					p_out_buf,												/* Data pointer for get */
					p_out_len												/* Data length pointer for get */
				);
				break;

			case 7:	/* img_to_activate_info */
				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}
				if (req_type == REQ_TYPE_GET)
				{
					/*
					 * img_to_activate_info_t array
					 */
					*p_out_buf++ = ATTR_TYPE_ARRAY;								/* Structure type */
					*p_out_buf++ = *p_child_record->p_img_to_activate_info_nr;	/* number of elements */
					*p_out_len   = 2;

					for (i = 0; i < *p_child_record->p_img_to_activate_info_nr; i++)
					{
						/*
						* img_to_activate_info_t structure
						*/
						p_act_info_child = (img_to_activate_info_t *)p_child_record->p_img_to_activate_info;
						*p_out_buf++ = ATTR_TYPE_STRUCTURE;						/* Structure type */
						*p_out_buf++ = 3;										/* 3 elements */
						*p_out_len   += 2;
						/* image_size */
						response_result = R_OBIS_DistributeAttrBuffer(
							&p_act_info_child->image_size.access,			/* Access method option */
							p_act_info_child->image_size.choice.type,		/* Type */
							p_act_info_child->image_size.choice.size,		/* Size */
							p_act_info_child->image_size.buffer,			/* Buffer of value */
							child_index,									/* Child ID */
							req_type,										/* Request type */
							service_type,									/* Service type */
							p_descriptor,									/* Pointer to COSEM descriptor */
							pdata,											/* Data from COSEM Appl layer */
							p_out_buf,										/* Data pointer for get */
							&temp_out_len									/* Data length pointer for get */
						);
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;
						/* image_id */
						response_result = R_OBIS_DistributeAttrBuffer(
							&p_act_info_child->image_id.access,				/* Access method option */
							p_act_info_child->image_id.choice.type,			/* Type */
							p_act_info_child->image_id.choice.size,			/* Size */
							p_act_info_child->image_id.buffer,				/* Buffer of value */
							child_index,									/* Child ID */
							req_type,										/* Request type */
							service_type,									/* Service type */
							p_descriptor,									/* Pointer to COSEM descriptor */
							pdata,											/* Data from COSEM Appl layer */
							p_out_buf,										/* Data pointer for get */
							&temp_out_len									/* Data length pointer for get */
						);
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;
						/* image_signature */
						response_result = R_OBIS_DistributeAttrBuffer(
							&p_act_info_child->image_signature.access,		/* Access method option */
							p_act_info_child->image_signature.choice.type,	/* Type */
							p_act_info_child->image_signature.choice.size,	/* Size */
							p_act_info_child->image_signature.buffer,		/* Buffer of value */
							child_index,									/* Child ID */
							req_type,										/* Request type */
							service_type,									/* Service type */
							p_descriptor,									/* Pointer to COSEM descriptor */
							pdata,											/* Data from COSEM Appl layer */
							p_out_buf,										/* Data pointer for get */
							&temp_out_len									/* Data length pointer for get */
						);
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;
					}
					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing */
				}
				break;

				default:
				break;
		}
	}
	else if (req_type == REQ_TYPE_ACTION)
	{
		Unsigned8 b_block_transfer = 0;
		
		switch (p_descriptor->Attr_ID & 0x00FF)
		{
			case 1:	/* image_transfer_initiate (data) */
			case 2:	/* image_block_transfer (data) */
			case 3:	/* image_verify (data) */
			case 4:	/* image_activate (data) */
				/* User function take care for all */
				response_result = R_OBIS_DistributeByUserFunc(
					child_index,		/* child_index */	
					req_type,			/* req_type */
					service_type,		/* service_type */
					&b_block_transfer,	/* p_is_block_transfer */
					p_descriptor,		/* p_descriptor */	
					pdata,				/* pdata */			
					p_out_buf,			/* p_out_buf */		
					p_out_len			/* p_out_len */
				);
				/* Not support block transfer now, TODO in future */
				if (b_block_transfer != 0)
				{
					response_result = ACTION_RESULT_SCOPE_ACCESS_VIOLATED;
				}
				break;
			default:
				response_result = ACTION_RESULT_TYPE_UNMATCH;
				break;
		}
	}
	else
	{
		/* INTERNAL ERROR */
	}

	return (response_result);
}
#endif

/******************************************************************************
* Function Name : R_OBIS_DistributeIECLocalPortSetupClass
* Interface     : Unsigned8 R_OBIS_DistributeIECLocalPortSetupClass(
*               :     Unsigned16 master_index,
*               :     Unsigned16 child_index,
*               :     req_type_t req_type,
*               :     service_type_t service_type,
*               :     st_Cosem_Attr_Desc *p_descriptor,
*               :     Unsigned8	*pdata,
*               :     Unsigned8	*p_out_buf,
*               :     Unsigned16 *p_out_len
*               : );
* Description   : Distributor function for IEC Local Port Setup
* Arguments     : Unsigned16 master_index -
*               :    [In] Id of the row in master list
*               : Unsigned16 child_index -
*               :    [In] Id of the row in child list
*               : req_type_t req_type -
*               :    [In] Request type (GET/SET/ACTION)
*               : service_type_t service_type -
*               :    [In] Service type (NORMAL/BLOCK/LIST)
*               : st_Cosem_Attr_Desc *p_descriptor -
*               :    [In] COSEM object descriptor
*               : Unsigned8 * pdata -
*               :    [In] Data in case of SET/ACTION
*               : Unsigned8 * p_out_buf -
*               :    [Out] Output buffer for encoded data
*               : Unsigned16 * p_out_len -
*               :    [Out] Output buffer length for encoded data
* Function Calls: TBD
* Return Value  : Unsigned8, distribution status
******************************************************************************/
#if (defined(USED_CLASS_19) && USED_CLASS_19 == 1)
Unsigned8 R_OBIS_DistributeIECLocalPortSetupClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
)
{
	/* Result of response, def. is not Matched */
	Unsigned8				response_result = TYPE_UNMATCH;
	Unsigned8 * p_temp_buf;
	Unsigned8 * p_temp_len;
	class19_child_record_t	*p_child_record;

	p_child_record = (class19_child_record_t *)(
		R_OBIS_FindChildRecordByIndex(master_index, child_index)
	);

	/* Classify the code based on Attribute or Method request */
	if (req_type == REQ_TYPE_GET || req_type == REQ_TYPE_SET)
	{
		switch (p_descriptor->Attr_ID)
		{
			case 0: /* attribute 0 */
				/* Not support */
				break;

			case 1: /* logical_name */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,							/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,	/* Max length of g_ServiceBuffer */
						ATTR_TYPE_OCTET_STRING,				/* Octet-string */
						p_descriptor->Instance_ID,			/* OBIS code from the request */
						6);									/* OBIS code is 6 chars */

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, OBIS code is read only */
				}
				break;

			case 2:	/* default_mode  */
			case 3:	/* default_baud  */
			case 4:	/* prop_baud  */
			case 5:	/* response_time  */
					switch (p_descriptor->Attr_ID)
					{
						case 2:	/* default_mode  */
							p_temp_buf = (Unsigned8 *)(p_child_record->p_default_mode);
							break;
						case 3:	/* default_baud  */
							p_temp_buf = (Unsigned8 *)(p_child_record->p_default_baud);
							break;
						case 4:	/* prop_baud  */
							p_temp_buf = (Unsigned8 *)(p_child_record->p_prop_baud);
							break;
						case 5:	/* response_time  */
							p_temp_buf = (Unsigned8 *)(p_child_record->p_response_time);
							break;
						default:
							/* Fatal error, need to debug */
							return TYPE_UNMATCH;
					}
					if (p_temp_buf != NULL)
					{
						response_result = R_OBIS_DistributeAttrBuffer(
							NULL,												/* Access method option */
							ATTR_TYPE_ENUM,										/* Type */
							1,													/* Size */
							p_temp_buf,											/* Buffer of value */
							child_index,										/* Child ID */
							req_type,											/* Request type */
							service_type,										/* Service type */
							p_descriptor,										/* Pointer to COSEM descriptor */
							pdata,												/* Data from COSEM Appl layer */
							p_out_buf,											/* Data pointer for get */
							p_out_len											/* Data length pointer for get */
						);
					}
				break;
			case 6:	/* device_addr  */
			case 7:	/* pass_p1  */
			case 8:	/* pass_p2  */
			case 9:	/* pass_w5  */
					switch (p_descriptor->Attr_ID)
					{
						case 6:	/* device_addr  */
							p_temp_buf = (Unsigned8 *)(p_child_record->p_device_addr);
							p_temp_len = (Unsigned8 *)(p_child_record->p_device_addr_len);
							break;
						case 7:	/* pass_p1  */
							p_temp_buf = (Unsigned8 *)(p_child_record->p_pass_p1);
							p_temp_len = (Unsigned8 *)(p_child_record->p_pass_p1_len);
							break;
						case 8:	/* pass_p2  */
							p_temp_buf = (Unsigned8 *)(p_child_record->p_pass_p2);
							p_temp_len = (Unsigned8 *)(p_child_record->p_pass_p2_len);
							break;
						case 9:	/* pass_w5  */
							p_temp_buf = (Unsigned8 *)(p_child_record->p_pass_w5);
							p_temp_len = (Unsigned8 *)(p_child_record->p_pass_w5_len);
							break;
						default:
							/* Fatal error, need to debug */
							return TYPE_UNMATCH;
					}

					if (p_temp_buf != NULL)
					{
						response_result = R_OBIS_DistributeAttrBuffer(
							NULL,												/* Access method option */
							ATTR_TYPE_OCTET_STRING,								/* Type */
							*p_temp_len,										/* Size */
							p_temp_buf,											/* Buffer of value */
							child_index,										/* Child ID */
							req_type,											/* Request type */
							service_type,										/* Service type */
							p_descriptor,										/* Pointer to COSEM descriptor */
							pdata,												/* Data from COSEM Appl layer */
							p_out_buf,											/* Data pointer for get */
							p_out_len											/* Data length pointer for get */
						);
					}
				break;
			default:
				break;
		}
	}
	else if (req_type == REQ_TYPE_ACTION)
	{
		/* Not available */
	}
	else
	{
		/* INTERNAL ERROR */
	}

	return (response_result);
}
#endif

/******************************************************************************
* Function Name : R_OBIS_DistributeActivityCalendarClass
* Interface     : Unsigned8 R_OBIS_DistributeActivityCalendarClass(
*               :     Unsigned16 master_index,
*               :     Unsigned16 child_index,
*               :     req_type_t req_type,
*               :     service_type_t service_type,
*               :     st_Cosem_Attr_Desc *p_descriptor,
*               :     Unsigned8	*pdata,
*               :     Unsigned8	*p_out_buf,
*               :     Unsigned16 *p_out_len
*               : );
* Description   : Distributor function for Clock class
* Arguments     : Unsigned16 master_index -
*               :    [In] Id of the row in master list
*               : Unsigned16 child_index -
*               :    [In] Id of the row in child list
*               : req_type_t req_type -
*               :    [In] Request type (GET/SET/ACTION)
*               : service_type_t service_type -
*               :    [In] Service type (NORMAL/BLOCK/LIST)
*               : st_Cosem_Attr_Desc *p_descriptor -
*               :    [In] COSEM object descriptor
*               : Unsigned8 * pdata -
*               :    [In] Data in case of SET/ACTION
*               : Unsigned8 * p_out_buf -
*               :    [Out] Output buffer for encoded data
*               : Unsigned16 * p_out_len -
*               :    [Out] Output buffer length for encoded data
* Function Calls: TBD
* Return Value  : Unsigned8, distribution status
******************************************************************************/
#if (defined(USED_CLASS_20) && USED_CLASS_20 == 1)
Unsigned8 R_OBIS_DistributeActivityCalendarClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
)
{
	/* Result of response, def. is not Matched */
	Unsigned8				response_result = TYPE_UNMATCH;
	Unsigned8				i=0;
	Unsigned8				j;
	Unsigned8				ret;

	Unsigned8				active_choice = 0;
	Unsigned8				profile_nr = 0;
	Unsigned16				temp_out_len;
	season_profile_t		*p_child_seanson_profile;
	week_profile_t			*p_child_week_profile;
	day_profile_action_t	*p_child_day_profile_action;
	Apprtc 					rtc;

	class20_child_record_t	*p_child_record;
	day_profile_t		*p_child_day_profile;
//	time_t * tt;
	p_child_record = (class20_child_record_t *)(
		R_OBIS_FindChildRecordByIndex(master_index, child_index)
	);

	/* Classify the code based on Attribute or Method request */
	if (req_type == REQ_TYPE_GET || req_type == REQ_TYPE_SET)
	{
		switch (p_descriptor->Attr_ID)
		{
			case 0: /* attribute 0 */
				/* Not support */
				break;

			case 1: /* logical_name */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,							/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,	/* Max length of g_ServiceBuffer */
						ATTR_TYPE_OCTET_STRING,				/* Octet-string */
						p_descriptor->Instance_ID,			/* OBIS code from the request */
						6);									/* OBIS code is 6 chars */

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, OBIS code is read only */
				}
				break;

			case 2:	/* calendar_name_active  */
				response_result = R_OBIS_DistributeAttrBuffer(
					NULL,												/* Access method option */
					ATTR_TYPE_OCTET_STRING,								/* Type */
					*p_child_record->p_calendar_name_active_len,		/* Size */
					p_child_record->p_calendar_name_active,				/* Buffer of value */
					child_index,										/* Child ID */
					req_type,											/* Request type */
					service_type,										/* Service type */
					p_descriptor,										/* Pointer to COSEM descriptor */
					pdata,												/* Data from COSEM Appl layer */
					p_out_buf,											/* Data pointer for get */
					p_out_len											/* Data length pointer for get */
				);
				break;
			case 3:	/* season_profile_active  */
			case 7:	/* season_profile_passive  */
				if (req_type == REQ_TYPE_GET)
				{
					if (p_descriptor->Attr_ID == 3)
					{
						active_choice = 1;
					}
					/*
					 * season_profile_t array
					 */
					*p_out_buf++ = ATTR_TYPE_ARRAY;								/* Structure type */
					if (active_choice == 1)
					{
						profile_nr = p_child_record->nr_season_profile_active;	/* No. elements */
						p_child_seanson_profile = (season_profile_t *)(p_child_record->p_season_profile_active);
					}
					else
					{
						profile_nr = p_child_record->nr_season_profile_passive;	/* No. elements */
						p_child_seanson_profile = (season_profile_t *)(p_child_record->p_season_profile_passive + i);
					}
					/* START: Call user interface to update value from meter */ 
					ret = R_USER_Class20_GetTariffSeasonTable(p_child_seanson_profile, child_index, active_choice);
					if (ret)	// NG
					{
						profile_nr = 0; /* Null array encode */
					}
					/* END: Call user interface to update value from meter */ 

					if (profile_nr == 0 )
					{
						/* Null array */
						// *p_out_buf++ = ATTR_TYPE_ARRAY;  // Added
						*p_out_buf++ = 1;
						*p_out_len   = 2;
						
						*p_out_buf++ = ATTR_TYPE_STRUCTURE;								/* Structure type */
						*p_out_buf++ = 3;												/* 3 elements */
						*p_out_len   += 2;

						/* p_season_profile_name */
						/* Encode */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
							ATTR_TYPE_OCTET_STRING,											/* Type */
							(Unsigned8 *)NULL,												/* Buffer */
							0																/* Length */
						);
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;

						/* season_start */
						/* Encode */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
							ATTR_TYPE_OCTET_STRING,											/* Type */
							(Unsigned8 *)NULL,												/* Buffer */
							0																/* Length */
						);
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;

						/* week_profile_name */
						/* Encode */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
							ATTR_TYPE_OCTET_STRING,											/* Type */
							(Unsigned8 *)NULL,												/* Buffer */
							0																/* Length */
						);
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;
						
						response_result = DATA_ACCESS_RESULT_SUCCESS;
						break;
					}
					// ELSE: not null

					*p_out_buf++ = profile_nr;

					*p_out_len   = 2;

					/* season_profile_t */
					for (i = 0; i < profile_nr; i ++)
					{
						if (active_choice == 1)
						{
							p_child_seanson_profile = (season_profile_t *)(p_child_record->p_season_profile_active + i);
						}
						else
						{
							p_child_seanson_profile = (season_profile_t *)(p_child_record->p_season_profile_passive + i);
						}
						*p_out_buf++ = ATTR_TYPE_STRUCTURE;								/* Structure type */
						*p_out_buf++ = 3;												/* 3 elements */
						*p_out_len   += 2;

						/* p_season_profile_name */
						/* Encode */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
							ATTR_TYPE_OCTET_STRING,											/* Type */
							p_child_seanson_profile->p_season_profile_name,					/* Buffer */
							p_child_seanson_profile->season_profile_name_len				/* Length */
						);
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;

						/* season_start */
						/* Encode */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
							ATTR_TYPE_OCTET_STRING,											/* Type */
							(Unsigned8 *)p_child_seanson_profile->p_season_start,			/* Buffer */
							sizeof(date_time_t)													/* Length */
						);
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;

						/* week_profile_name */
						/* Encode */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
							ATTR_TYPE_OCTET_STRING,											/* Type */
							p_child_seanson_profile->p_week_profile_name,					/* Buffer */
							p_child_seanson_profile->week_profile_name_len					/* Length */
						);
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;
					}

					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else if (req_type == REQ_TYPE_SET)
				{
					if (p_descriptor->Attr_ID == 3)
					{
						active_choice = 1;
					}
					response_result = TYPE_UNMATCH;
					/*
					 * season_profile_t array
					 */
					/* Structure type */
					if (*pdata++ != ATTR_TYPE_ARRAY)
					{
						return response_result;
					}

					if (active_choice == 1)
					{
						profile_nr = p_child_record->nr_season_profile_active;	/* No. elements */
					}
					else
					{
						profile_nr = p_child_record->nr_season_profile_passive;	/* No. elements */
					}

					if (*pdata++ != profile_nr)
					{
						return response_result;
					}

					/* season_profile_t */
					for (i = 0; i < profile_nr; i ++)
					{
						if (active_choice == 1)
						{
							p_child_seanson_profile = (season_profile_t *)(p_child_record->p_season_profile_active + i);
						}
						else
						{
							p_child_seanson_profile = (season_profile_t *)(p_child_record->p_season_profile_passive + i);
						}
						/* Structure type */
						if (*pdata++ != ATTR_TYPE_STRUCTURE)
						{
							return response_result;
						}
						if (*pdata++ != 3)	/* 3 elements */
						{
							return response_result;
						}

						/* p_season_profile_name */
						/* Decode */
						temp_out_len = R_OBIS_DecodeAttribute(
										(Unsigned8 *)p_child_seanson_profile->p_season_profile_name,	/* value_buf */
										p_child_seanson_profile->season_profile_name_len,				/* value_buf_len */
										ATTR_TYPE_OCTET_STRING,											/* attr_type */
										pdata,															/* buf */
										OBIS_SERVICE_DATA_BUFFER_LENGTH									/* buf_len */
										);
						pdata += temp_out_len;

						/* season_start */
						/* Dencode */
						if (R_OBIS_IsValidDateTime(pdata)== 0) // Invalid
						{
							break;
						}
						temp_out_len = R_OBIS_DecodeAttribute(
										(Unsigned8 *)p_child_seanson_profile->p_season_start,			/* value_buf */
										sizeof(date_time_t),											/* value_buf_len */
										ATTR_TYPE_OCTET_STRING,											/* attr_type */
										pdata,															/* buf */
										OBIS_SERVICE_DATA_BUFFER_LENGTH									/* buf_len */
										);
						pdata += temp_out_len;

						/* week_profile_name */
						/* Decode */
						temp_out_len = R_OBIS_DecodeAttribute(
										(Unsigned8 *)p_child_seanson_profile->p_week_profile_name,		/* value_buf */
										p_child_seanson_profile->week_profile_name_len,					/* value_buf_len */
										ATTR_TYPE_OCTET_STRING,											/* attr_type */
										pdata,															/* buf */
										OBIS_SERVICE_DATA_BUFFER_LENGTH									/* buf_len */
										);
						pdata += temp_out_len;
					}
					R_USER_Class20_ChangSeason(&g_Class20_season_profile);
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, calendar_name_active */
				}
				break;
			case 4:	/* week_profile_active  */
			case 8:	/* week_profile_passsive  */
				if (req_type == REQ_TYPE_GET)
				{
					if (p_descriptor->Attr_ID == 4)
					{
						active_choice = 1;
					}
					/*
					 * week_profile_t array
					 */

					*p_out_buf++ = ATTR_TYPE_ARRAY;								/* Array type */
					if (active_choice == 1)
					{
						profile_nr = p_child_record->nr_week_profile_active;	/* No. elements */
						p_child_week_profile = (week_profile_t *)(p_child_record->p_week_profile_active);
					}
					else
					{
						profile_nr = p_child_record->nr_week_profile_passive;	/* No. elements */
						p_child_week_profile = (week_profile_t *)(p_child_record->p_week_profile_passive);
					}
					/* START: Call user interface to update value from meter */ 
					ret = R_USER_Class20_GetTariffWeekTable(p_child_week_profile, child_index, active_choice);
					if (ret)	// NG
					{
						profile_nr = 0; /* Null array encode */
					}
					/* END: Call user interface to update value from meter */ 

					if (profile_nr == 0 )
					{
						/* Null array */
						// *p_out_buf++ = ATTR_TYPE_ARRAY;  // Added
						*p_out_buf++ = 1;
						*p_out_len   = 2;

						*p_out_buf++ = ATTR_TYPE_STRUCTURE;								/* Structure type */
						*p_out_buf++ = 8;												/* 8 elements */
						*p_out_len   += 2;

						/* week_profile_name */
						/* Encode */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,													/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,							/* Max length of g_ServiceBuffer */
							ATTR_TYPE_OCTET_STRING,										/* Type */
							(Unsigned8 *)NULL,											/* Buffer */
							0															/* Length */
						);
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;

						/* Monday */
						*p_out_buf++ = ATTR_TYPE_UNSIGNED;
						*p_out_buf++ = (Unsigned8)0x00;
						*p_out_len += 2;

						/* Tuesday */
						*p_out_buf++ = ATTR_TYPE_UNSIGNED;
						*p_out_buf++ = (Unsigned8)0x00;
						*p_out_len += 2;

						/* Wednesday */
						*p_out_buf++ = ATTR_TYPE_UNSIGNED;
						*p_out_buf++ = (Unsigned8)0x00;
						*p_out_len += 2;

						/* Thursday */
						*p_out_buf++ = ATTR_TYPE_UNSIGNED;
						*p_out_buf++ = (Unsigned8)0x00;
						*p_out_len += 2;

						/* Friday */
						*p_out_buf++ = ATTR_TYPE_UNSIGNED;
						*p_out_buf++ = (Unsigned8)0x00;
						*p_out_len += 2;

						/* Saturday */
						*p_out_buf++ = ATTR_TYPE_UNSIGNED;
						*p_out_buf++ = (Unsigned8)0x00;
						*p_out_len += 2;

						/* Sunday */
						*p_out_buf++ = ATTR_TYPE_UNSIGNED;
						*p_out_buf++ = (Unsigned8)0x00;
						*p_out_len += 2;
						

						response_result = DATA_ACCESS_RESULT_SUCCESS;
						break;
					}
					*p_out_buf++ = profile_nr;

					*p_out_len   = 2;

					/* week_profile_t */
					for (i = 0; i < profile_nr; i ++)
					{
						if (active_choice == 1)
						{
							p_child_week_profile = (week_profile_t *)(p_child_record->p_week_profile_active + i);
						}
						else
						{
							p_child_week_profile = (week_profile_t *)(p_child_record->p_week_profile_passive + i);
						}
						*p_out_buf++ = ATTR_TYPE_STRUCTURE;								/* Structure type */
						*p_out_buf++ = 8;												/* 8 elements */
						*p_out_len   += 2;

						/* week_profile_name */
						/* Encode */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,													/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,							/* Max length of g_ServiceBuffer */
							ATTR_TYPE_OCTET_STRING,										/* Type */
							p_child_week_profile->p_week_profile_name,					/* Buffer */
							p_child_week_profile->week_profile_name_len					/* Length */
						);
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;

						/* Monday */
						*p_out_buf++ = ATTR_TYPE_UNSIGNED;
						*p_out_buf++ = (Unsigned8)p_child_week_profile->Monday;
						*p_out_len += 2;

						/* Tuesday */
						*p_out_buf++ = ATTR_TYPE_UNSIGNED;
						*p_out_buf++ = (Unsigned8)p_child_week_profile->Tuesday;
						*p_out_len += 2;

						/* Wednesday */
						*p_out_buf++ = ATTR_TYPE_UNSIGNED;
						*p_out_buf++ = (Unsigned8)p_child_week_profile->Wednesday;
						*p_out_len += 2;

						/* Thursday */
						*p_out_buf++ = ATTR_TYPE_UNSIGNED;
						*p_out_buf++ = (Unsigned8)p_child_week_profile->Thursday;
						*p_out_len += 2;

						/* Friday */
						*p_out_buf++ = ATTR_TYPE_UNSIGNED;
						*p_out_buf++ = (Unsigned8)p_child_week_profile->Friday;
						*p_out_len += 2;

						/* Saturday */
						*p_out_buf++ = ATTR_TYPE_UNSIGNED;
						*p_out_buf++ = (Unsigned8)p_child_week_profile->Saturday;
						*p_out_len += 2;

						/* Sunday */
						*p_out_buf++ = ATTR_TYPE_UNSIGNED;
						*p_out_buf++ = (Unsigned8)p_child_week_profile->Sunday;
						*p_out_len += 2;
					}

					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else if (req_type == REQ_TYPE_SET)
				{
					if (p_descriptor->Attr_ID == 4)
					{
						active_choice = 1;
					}
					response_result = TYPE_UNMATCH;
					/*
					 * week_profile_t array
					 */
					if (*pdata++ != ATTR_TYPE_ARRAY)
					{
						return response_result;
					}
					if (active_choice == 1)
					{
						profile_nr = p_child_record->nr_week_profile_active;	/* No. elements */
					}
					else
					{
						profile_nr = p_child_record->nr_week_profile_passive;	/* No. elements */
					}
					if (*pdata++ != profile_nr)
					{
						return response_result;
					}

					/* week_profile_t */
					for (i = 0; i < profile_nr; i ++)
					{
						if (active_choice == 1)
						{
							p_child_week_profile = (week_profile_t *)(p_child_record->p_week_profile_active + i);
						}
						else
						{
							p_child_week_profile = (week_profile_t *)(p_child_record->p_week_profile_passive + i);
						}
						if (*pdata++ != ATTR_TYPE_STRUCTURE)	/* Structure type */
						{
							return response_result;
						}
						if (*pdata++ != 8)						/* 8 elements */
						{
							return response_result;
						}

						/* week_profile_name */
						/* Encode */
						temp_out_len = R_OBIS_DecodeAttribute(
										(Unsigned8 *)p_child_week_profile->p_week_profile_name,		/* value_buf */
										p_child_week_profile->week_profile_name_len,					/* value_buf_len */
										ATTR_TYPE_OCTET_STRING,											/* attr_type */
										pdata,															/* buf */
										OBIS_SERVICE_DATA_BUFFER_LENGTH									/* buf_len */
										);
						pdata += temp_out_len;
						/* Monday */
						temp_out_len = R_OBIS_DecodeAttribute(
										(Unsigned8 *)&p_child_week_profile->Monday,		/* value_buf */
										1,												/* value_buf_len */
										ATTR_TYPE_UNSIGNED,								/* attr_type */
										pdata,											/* buf */
										OBIS_SERVICE_DATA_BUFFER_LENGTH					/* buf_len */
										);
						pdata += temp_out_len;
						/* Tuesday */
						temp_out_len = R_OBIS_DecodeAttribute(
										(Unsigned8 *)&p_child_week_profile->Tuesday,		/* value_buf */
										1,												/* value_buf_len */
										ATTR_TYPE_UNSIGNED,								/* attr_type */
										pdata,											/* buf */
										OBIS_SERVICE_DATA_BUFFER_LENGTH					/* buf_len */
										);
						pdata += temp_out_len;

						/* Wednesday */
						temp_out_len = R_OBIS_DecodeAttribute(
										(Unsigned8 *)&p_child_week_profile->Wednesday,		/* value_buf */
										1,												/* value_buf_len */
										ATTR_TYPE_UNSIGNED,								/* attr_type */
										pdata,											/* buf */
										OBIS_SERVICE_DATA_BUFFER_LENGTH					/* buf_len */
										);
						pdata += temp_out_len;
//						*p_out_buf++ = ATTR_TYPE_UNSIGNED;
//						*p_out_buf++ = (Unsigned8)p_child_week_profile->Wednesday;
//						*p_out_len += 2;

						/* Thursday */
						temp_out_len = R_OBIS_DecodeAttribute(
										(Unsigned8 *)&p_child_week_profile->Thursday,		/* value_buf */
										1,												/* value_buf_len */
										ATTR_TYPE_UNSIGNED,								/* attr_type */
										pdata,											/* buf */
										OBIS_SERVICE_DATA_BUFFER_LENGTH					/* buf_len */
										);
						pdata += temp_out_len;
						/* Friday */
						temp_out_len = R_OBIS_DecodeAttribute(
										(Unsigned8 *)&p_child_week_profile->Friday,		/* value_buf */
										1,												/* value_buf_len */
										ATTR_TYPE_UNSIGNED,								/* attr_type */
										pdata,											/* buf */
										OBIS_SERVICE_DATA_BUFFER_LENGTH					/* buf_len */
										);
						pdata += temp_out_len;

						/* Saturday */
						temp_out_len = R_OBIS_DecodeAttribute(
										(Unsigned8 *)&p_child_week_profile->Saturday,		/* value_buf */
										1,												/* value_buf_len */
										ATTR_TYPE_UNSIGNED,								/* attr_type */
										pdata,											/* buf */
										OBIS_SERVICE_DATA_BUFFER_LENGTH					/* buf_len */
										);
						pdata += temp_out_len;

						/* Sunday */
						temp_out_len = R_OBIS_DecodeAttribute(
										(Unsigned8 *)&p_child_week_profile->Sunday,		/* value_buf */
										1,												/* value_buf_len */
										ATTR_TYPE_UNSIGNED,								/* attr_type */
										pdata,											/* buf */
										OBIS_SERVICE_DATA_BUFFER_LENGTH					/* buf_len */
										);
						pdata += temp_out_len;

					}

					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, calendar_name_active */
				}
				break;

			case 5:	/* day_profile_active  */
			case 9:	/* day_profile_passive  */
				if (req_type == REQ_TYPE_GET)
				{
					active_choice = 0;
				
					if (p_descriptor->Attr_ID == 5)
					{
						active_choice = 1;
					}
					/*
					 * day_profile_t array
					 */

					*p_out_buf++ = ATTR_TYPE_ARRAY;								/* Structure type */
					if (active_choice == 1)
					{
						profile_nr = p_child_record->nr_day_profile_active;		/* No. elements */
						p_child_day_profile = (day_profile_t *)(p_child_record->p_day_profile_active);
					}
					else
					{
						profile_nr = p_child_record->nr_day_profile_passive;	/* No. elements */
						p_child_day_profile = (day_profile_t *)(p_child_record->p_day_profile_passive);
					}
					/* START: Call user interface to update value from meter */ 
					ret = R_USER_Class20_GetTariffDayTable(p_child_day_profile, child_index, active_choice);
					if (ret)	// NG
					{
						profile_nr = 0; /* Null array encode */
					}
					/* END: Call user interface to update value from meter */ 

					if (profile_nr == 0 )
					{
						/* Null array */
						// *p_out_buf++ = ATTR_TYPE_ARRAY;  // Added
						*p_out_buf++ = 1;
						*p_out_len   = 2;
						
						*p_out_buf++ = ATTR_TYPE_STRUCTURE;							/* Structure type */
						*p_out_buf++ = 2;											/* 2 elements */
						*p_out_len   += 2;

						/* day_id */
						*p_out_buf++ = ATTR_TYPE_UNSIGNED;
						*p_out_buf++ = 0x00;
						*p_out_len += 2;

						/* day_profile_action_t array */

						*p_out_buf++ = ATTR_TYPE_ARRAY;								/* Structure type */
						*p_out_buf++ = 1;		/* 2 elements */
						*p_out_len   += 2;

						/* day_profile_action_t */
							*p_out_buf++ = ATTR_TYPE_STRUCTURE;								/* Structure type */
							*p_out_buf++ = 3;												/* 3 elements */
							*p_out_len   += 2;

							/* start_time */
							temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
							ATTR_TYPE_OCTET_STRING,											/* Type */
							(Unsigned8 *)NULL,												/* Buffer */
							0																/* Length */
							);
							p_out_buf = p_out_buf + temp_out_len;
							*p_out_len += temp_out_len;

							/* script_logical_name */
							*p_out_buf++ =  ATTR_TYPE_OCTET_STRING;
							*p_out_buf++ =  6;									/* OBIS code is 6 chars */
							*p_out_buf++ =  (Unsigned8)0x00;
							*p_out_buf++ =  (Unsigned8)0x00;
							*p_out_buf++ =  (Unsigned8)0x00;
							*p_out_buf++ =  (Unsigned8)0x00;
							*p_out_buf++ =  (Unsigned8)0x00;
							*p_out_buf++ =  (Unsigned8)0x00;
							*p_out_len   += 8;
							/* Script ID */
							*p_out_buf++ = ATTR_TYPE_LONG_UNSIGNED;
							*p_out_buf++ = (Unsigned8)(0x00);
							*p_out_buf++ = (Unsigned8)(0x00);
							*p_out_len   += 3;

						response_result = DATA_ACCESS_RESULT_SUCCESS;
						break;
					}

					*p_out_buf++ = profile_nr;

					*p_out_len   = 2;

					/* day_profile_t */
					for (i = 0; i < profile_nr; i ++)
					{
						if (active_choice == 1)
						{
							p_child_day_profile = (day_profile_t *)(p_child_record->p_day_profile_active + i);
						}
						else
						{
							p_child_day_profile = (day_profile_t *)(p_child_record->p_day_profile_passive + i);
						}
						*p_out_buf++ = ATTR_TYPE_STRUCTURE;							/* Structure type */
						*p_out_buf++ = 2;											/* 2 elements */
						*p_out_len   += 2;

						/* day_id */
						*p_out_buf++ = ATTR_TYPE_UNSIGNED;
						*p_out_buf++ = (Unsigned8)p_child_day_profile->day_id;
						*p_out_len += 2;

						/* day_profile_action_t array */

						*p_out_buf++ = ATTR_TYPE_ARRAY;								/* Structure type */
						*p_out_buf++ = p_child_day_profile->nr_day_schedule;		/* 2 elements */
						*p_out_len   += 2;

						/* day_profile_action_t */
						for (j = 0; j < p_child_day_profile->nr_day_schedule; j ++)
						{
							p_child_day_profile_action = (day_profile_action_t *)(p_child_day_profile->day_schedule + j);
//							if((p_descriptor->Attr_ID)==5)
//								p_child_day_profile_action->script_selector=ScriptId[j];
//							else	
//								p_child_day_profile_action->script_selector=passive_script_id[j];
							*p_out_buf++ = ATTR_TYPE_STRUCTURE;								/* Structure type */
							*p_out_buf++ = 3;												/* 3 elements */
							*p_out_len   += 2;

							/* start_time */
							temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
							ATTR_TYPE_OCTET_STRING,											/* Type */
							(Unsigned8 *)p_child_day_profile_action->p_start_time,			/* Buffer */
							sizeof(time_t)													/* Length */
							);
							p_out_buf = p_out_buf + temp_out_len;
							*p_out_len += temp_out_len;

							/* script_logical_name */
							*p_out_buf++ =  ATTR_TYPE_OCTET_STRING;
							*p_out_buf++ =  6;									/* OBIS code is 6 chars */
							*p_out_buf++ =  (Unsigned8)p_child_day_profile_action->script_logical_name[0];
							*p_out_buf++ =  (Unsigned8)p_child_day_profile_action->script_logical_name[1];
							*p_out_buf++ =  (Unsigned8)p_child_day_profile_action->script_logical_name[2];
							*p_out_buf++ =  (Unsigned8)p_child_day_profile_action->script_logical_name[3];
							*p_out_buf++ =  (Unsigned8)p_child_day_profile_action->script_logical_name[4];
							*p_out_buf++ =  (Unsigned8)p_child_day_profile_action->script_logical_name[5];
							*p_out_len   += 8;
							/* Script ID */
							*p_out_buf++ = ATTR_TYPE_LONG_UNSIGNED;
							*p_out_buf++ = (Unsigned8)(p_child_day_profile_action->script_selector >> 8);
							*p_out_buf++ = (Unsigned8)(p_child_day_profile_action->script_selector  & 0xFF);
							*p_out_len   += 3;

						}
					}
					if (*p_out_len > OBIS_SERVICE_DATA_BUFFER_LENGTH)
					{
						/* Overflow */
						response_result = TYPE_UNMATCH;
					}
					else
					{
						response_result = DATA_ACCESS_RESULT_SUCCESS;
					}
				}
				else if (req_type == REQ_TYPE_SET)
				{
					active_choice = 0;
//					if (p_descriptor->Attr_ID == 5)
//					{
//						active_choice = 1;
//					}
					response_result = TYPE_UNMATCH;
					/*
					 * day_profile_t array
					 */
					if (*pdata++ != ATTR_TYPE_ARRAY)
					{
						return response_result;
					}

//					if (active_choice == 1)
//					{
//						profile_nr = p_child_record->nr_day_profile_active;		/* No. elements */
//						p_child_day_profile = (day_profile_t *)(p_child_record->p_day_profile_active);
//					}
//					else
					{
						profile_nr = p_child_record->nr_day_profile_passive;	/* No. elements */
//						p_child_day_profile = (day_profile_t *)(p_child_record->p_day_profile_passive);
					}
					if (*pdata++ != profile_nr)
					{
						return response_result;
					}

					/* day_profile_t */
					for (i = 0; i < profile_nr; i ++)
					{
//						if (active_choice == 1)
//						{
//							p_child_day_profile = (day_profile_t *)(p_child_record->p_day_profile_active + i);
//						}
//						else
						{
							p_child_day_profile = (day_profile_t *)(p_child_record->p_day_profile_passive + i);
						}
						if (*pdata++ != ATTR_TYPE_STRUCTURE)	/* Structure type */
						{
							return response_result;
						}
						if (*pdata++ != 2)	/* 2 elements */
						{
							return response_result;
						}

						/* day_id */
						if (*pdata++ != ATTR_TYPE_UNSIGNED)	/* Structure type */
						{
							return response_result;
						}
						p_child_day_profile->day_id = (Unsigned8)*pdata++;

						/* day_profile_action_t array */

						if (*pdata++ != ATTR_TYPE_ARRAY)	/* Array type */
						{
							return response_result;
						}
						if (*pdata++ != p_child_day_profile->nr_day_schedule)	/* n elements */
						{
							return response_result;
						}

						/* day_profile_action_t */
						for (j = 0; j < p_child_day_profile->nr_day_schedule; j ++)
						{
							p_child_day_profile_action = (day_profile_action_t *)(p_child_day_profile->day_schedule + j);
							if (*pdata++ != ATTR_TYPE_STRUCTURE)	/* Structure type */
							{
								return response_result;
							}
							if (*pdata++ != 3)	/* 3 elements */
							{
								return response_result;
							}

							/* start_time */
							temp_out_len = R_OBIS_DecodeAttribute(
										(Unsigned8 *)p_child_day_profile_action->p_start_time,		/* value_buf */
										sizeof(time_t),												/* value_buf_len */
										ATTR_TYPE_OCTET_STRING,										/* attr_type */
										pdata,														/* buf */
										OBIS_SERVICE_DATA_BUFFER_LENGTH								/* buf_len */
										);
							pdata += temp_out_len;

							/* script_logical_name */
							temp_out_len = R_OBIS_DecodeAttribute(
										(Unsigned8 *)p_child_day_profile_action->script_logical_name,	/* value_buf */
										6,															/* value_buf_len */
										ATTR_TYPE_OCTET_STRING,										/* attr_type */
										pdata,														/* buf */
										OBIS_SERVICE_DATA_BUFFER_LENGTH								/* buf_len */
										);
							pdata += temp_out_len;

							/* Script ID */
							temp_out_len = R_OBIS_DecodeAttribute(
										(Unsigned8 *)&p_child_day_profile_action->script_selector,	/* value_buf */
										2,															/* value_buf_len */
										ATTR_TYPE_LONG_UNSIGNED,									/* attr_type */
										pdata,														/* buf */
										OBIS_SERVICE_DATA_BUFFER_LENGTH								/* buf_len */
										);
							pdata += temp_out_len;
//							if(i==0)
//							{
								passive_tod_zone[j]=p_child_day_profile_action->p_start_time;
								passive_script_id[j]=p_child_day_profile_action->script_selector;
//							}
//							else
//							{
//								passive_tod_zone2[j]=p_child_day_profile_action->p_start_time;
//								passive_script_id2[j]=p_child_day_profile_action->script_selector;
//							}
						}
						if(R_USER_Class20_SetTariffDayTablePassive(p_child_day_profile)==0)
						{
							return response_result;
							
						}
						
					}
					//Set_PP(0,5);	
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing */
				}
				break;
			case 6:	/* calendar_name_passive  */
				response_result = R_OBIS_DistributeAttrBuffer(
					NULL,												/* Access method option */
					ATTR_TYPE_OCTET_STRING,								/* Type */
					*p_child_record->p_calendar_name_passive_len,		/* Size */
					p_child_record->p_calendar_name_passive,			/* Buffer of value */
					child_index,										/* Child ID */
					req_type,											/* Request type */
					service_type,										/* Service type */
					p_descriptor,										/* Pointer to COSEM descriptor */
					pdata,												/* Data from COSEM Appl layer */
					p_out_buf,											/* Data pointer for get */
					p_out_len											/* Data length pointer for get */
				);
				break;
			case 10:	/* activate_passive_calendar_time  */
				if (req_type == REQ_TYPE_SET)
				{
					if (R_OBIS_IsValidDateTime(pdata)== 0) // Invalid
					{
						break;
					}
				}
				
				response_result = R_OBIS_DistributeAttrBuffer(
					NULL,												/* Access method option */
					ATTR_TYPE_OCTET_STRING,								/* Type */
					sizeof(date_time_t),								/* Size */
					p_child_record->p_activate_calendar_passive,		/* Buffer of value */
					child_index,										/* Child ID */
					req_type,											/* Request type */
					service_type,										/* Service type */
					p_descriptor,										/* Pointer to COSEM descriptor */
					pdata,												/* Data from COSEM Appl layer */
					p_out_buf,											/* Data pointer for get */
					p_out_len											/* Data length pointer for get */
				);
				if (req_type == REQ_TYPE_SET)
				{
					response_result= R_USER_Class20_SETACTION_Time(R_USER_ConvertrDLMStoAppRTC(&g_Class20_ac_time,&rtc),0);
				}	
				break;
			default:
				break;
		}
	}
	else if (req_type == REQ_TYPE_ACTION)
	{
		Unsigned8 b_block_transfer = 0;
		
		switch (p_descriptor->Attr_ID & 0x00FF)
		{
			case 1:	/* activate_passive_calendar (data) */
				/* User function take care for all */
				response_result = R_OBIS_DistributeByUserFunc(
					child_index,		/* child_index */	
					req_type,			/* req_type */
					service_type,		/* service_type */
					&b_block_transfer,	/* p_is_block_transfer */
					p_descriptor,		/* p_descriptor */	
					pdata,				/* pdata */			
					p_out_buf,			/* p_out_buf */		
					p_out_len			/* p_out_len */
				);
				/* Not support block transfer now, TODO in future */
				if (b_block_transfer != 0)
				{
					response_result = ACTION_RESULT_SCOPE_ACCESS_VIOLATED;
				}
				break;
			default:
				response_result = ACTION_RESULT_TYPE_UNMATCH;
				break;
		}
	}
	else
	{
		/* INTERNAL ERROR */
	}

	return (response_result);
}
#endif

/******************************************************************************
* Function Name : R_OBIS_DistributeRegisterMonitorClass
* Interface     : Unsigned8 R_OBIS_DistributeRegisterMonitorClass(
*               :     Unsigned16 master_index,
*               :     Unsigned16 child_index,
*               :     req_type_t req_type,
*               :     service_type_t service_type,
*               :     st_Cosem_Attr_Desc *p_descriptor,
*               :     Unsigned8	*pdata,
*               :     Unsigned8	*p_out_buf,
*               :     Unsigned16 *p_out_len
*               : );
* Description   : Distributor function for Clock class
* Arguments     : Unsigned16 master_index -
*               :    [In] Id of the row in master list
*               : Unsigned16 child_index -
*               :    [In] Id of the row in child list
*               : req_type_t req_type -
*               :    [In] Request type (GET/SET/ACTION)
*               : service_type_t service_type -
*               :    [In] Service type (NORMAL/BLOCK/LIST)
*               : st_Cosem_Attr_Desc *p_descriptor -
*               :    [In] COSEM object descriptor
*               : Unsigned8 * pdata -
*               :    [In] Data in case of SET/ACTION
*               : Unsigned8 * p_out_buf -
*               :    [Out] Output buffer for encoded data
*               : Unsigned16 * p_out_len -
*               :    [Out] Output buffer length for encoded data
* Function Calls: TBD
* Return Value  : Unsigned8, distribution status
******************************************************************************/
#if (defined(USED_CLASS_21) && USED_CLASS_21 == 1)
Unsigned8 R_OBIS_DistributeRegisterMonitorClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
)
{
	/* Result of response, def. is not Matched */
	Unsigned8				response_result = TYPE_UNMATCH;
	Unsigned8				i;
	Unsigned8				j;

	Unsigned16				temp_out_len;
	class21_child_record_t	*p_child_record;

	p_child_record = (class21_child_record_t *)(
		R_OBIS_FindChildRecordByIndex(master_index, child_index)
	);

	/* Classify the code based on Attribute or Method request */
	if (req_type == REQ_TYPE_GET || req_type == REQ_TYPE_SET)
	{
		switch (p_descriptor->Attr_ID)
		{
			case 0: /* attribute 0 */
				/* Not support */
				break;

			case 1: /* logical_name */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,							/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,	/* Max length of g_ServiceBuffer */
						ATTR_TYPE_OCTET_STRING,				/* Octet-string */
						p_descriptor->Instance_ID,			/* OBIS code from the request */
						6);									/* OBIS code is 6 chars */

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, OBIS code is read only */
				}
				break;

			case 2:	/* thresholds  */
			if (req_type == REQ_TYPE_GET)
				{
					/*
					 * thresholds array
					 */
					value_t		*p_child_thresholds;
					*p_out_buf++ = ATTR_TYPE_ARRAY;					/* Structure type */
					*p_out_buf++ = p_child_record->nr_thresholds;	/* No. elements */
					*p_out_len   = 2;

					/* thresholds */
					for (i = 0; i < p_child_record->nr_thresholds; i ++)
					{
						p_child_thresholds = (value_t *)(p_child_record->p_thresholds + i);

						/* switch_time */
						/* Encode */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,								/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,		/* Max length of g_ServiceBuffer */
							p_child_thresholds->choice.type,		/* Type */
							p_child_thresholds->buffer,				/* Buffer */
							p_child_thresholds->choice.size			/* Length */
						);

						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;
					}

					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, calendar_name_active */
				}
				break;
			case 3:	/* monitored_value  */
				if (req_type == REQ_TYPE_GET)
				{
					value_definition_t *monitor =  (value_definition_t *)p_child_record->monitor_value;

					*p_out_buf++ = ATTR_TYPE_STRUCTURE;				/* Structure type */
					*p_out_buf++ = 3;								/* 3 elements */
					*p_out_len   = 2;
					/* class_id */
					*p_out_buf++ = ATTR_TYPE_LONG_UNSIGNED;
					*p_out_buf++ = (Unsigned8)(monitor->class_id >> 8);
					*p_out_buf++ = (Unsigned8)(monitor->class_id  & 0xFF);
					*p_out_len   += 3;

					/* logical_name */
					*p_out_buf++ =  ATTR_TYPE_OCTET_STRING;
					*p_out_buf++ =  6;								/* OBIS code is 6 chars */
					*p_out_buf++ =  (Unsigned8)monitor->logical_name[0];
					*p_out_buf++ =  (Unsigned8)monitor->logical_name[1];
					*p_out_buf++ =  (Unsigned8)monitor->logical_name[2];
					*p_out_buf++ =  (Unsigned8)monitor->logical_name[3];
					*p_out_buf++ =  (Unsigned8)monitor->logical_name[4];
					*p_out_buf++ =  (Unsigned8)monitor->logical_name[5];
					*p_out_len   += 8;

					/* attribute_index */
					*p_out_buf++ = ATTR_TYPE_INTEGER;
					*p_out_buf++ = (Unsigned8)monitor->index;
					*p_out_len   += 2;


					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, calendar_name_active */
				}
				break;
			case 4:	/* actions  */
				if (req_type == REQ_TYPE_GET)
				{
					/*
					 * action_set_t array
					 */
					action_set_t		*p_child_action_set;
					*p_out_buf++ = ATTR_TYPE_ARRAY;								/* Structure type */
					*p_out_buf++ = p_child_record->nr_actions;					/* No. elements */
					*p_out_len   = 2;

					/* action_set_t */
					for (i = 0; i < p_child_record->nr_actions; i ++)
					{
						p_child_action_set = (action_set_t *)(p_child_record->p_actions + i);
						*p_out_buf++ = ATTR_TYPE_STRUCTURE;						/* Structure type */
						*p_out_buf++ = 2;										/* 2 elements */
						*p_out_len   += 2;

						/* action up */
						*p_out_buf++ = ATTR_TYPE_STRUCTURE;						/* Structure type */
						*p_out_buf++ = 2;										/* 2 elements */
						*p_out_len   += 2;

						/* script_logical_name */
						*p_out_buf++ =  ATTR_TYPE_OCTET_STRING;
						*p_out_buf++ =  6;									/* OBIS code is 6 chars */
						*p_out_buf++ =  (Unsigned8)p_child_action_set->action_up->script_logical_name[0];
						*p_out_buf++ =  (Unsigned8)p_child_action_set->action_up->script_logical_name[1];
						*p_out_buf++ =  (Unsigned8)p_child_action_set->action_up->script_logical_name[2];
						*p_out_buf++ =  (Unsigned8)p_child_action_set->action_up->script_logical_name[3];
						*p_out_buf++ =  (Unsigned8)p_child_action_set->action_up->script_logical_name[4];
						*p_out_buf++ =  (Unsigned8)p_child_action_set->action_up->script_logical_name[5];
						*p_out_len   += 8;
						/* Script ID */
						*p_out_buf++ = ATTR_TYPE_LONG_UNSIGNED;
						*p_out_buf++ = (Unsigned8)(p_child_action_set->action_up->script_selector >> 8);
						*p_out_buf++ = (Unsigned8)(p_child_action_set->action_up->script_selector  & 0xFF);
						*p_out_len   += 3;

						/* action down */
						*p_out_buf++ = ATTR_TYPE_STRUCTURE;					/* Structure type */
						*p_out_buf++ = 2;									/* 2 elements */
						*p_out_len   += 2;

						/* script_logical_name */
						*p_out_buf++ =  ATTR_TYPE_OCTET_STRING;
						*p_out_buf++ =  6;									/* OBIS code is 6 chars */
						*p_out_buf++ =  (Unsigned8)p_child_action_set->action_down->script_logical_name[0];
						*p_out_buf++ =  (Unsigned8)p_child_action_set->action_down->script_logical_name[1];
						*p_out_buf++ =  (Unsigned8)p_child_action_set->action_down->script_logical_name[2];
						*p_out_buf++ =  (Unsigned8)p_child_action_set->action_down->script_logical_name[3];
						*p_out_buf++ =  (Unsigned8)p_child_action_set->action_down->script_logical_name[4];
						*p_out_buf++ =  (Unsigned8)p_child_action_set->action_down->script_logical_name[5];
						*p_out_len   += 8;
						/* Script ID */
						*p_out_buf++ = ATTR_TYPE_LONG_UNSIGNED;
						*p_out_buf++ = (Unsigned8)(p_child_action_set->action_down->script_selector >> 8);
						*p_out_buf++ = (Unsigned8)(p_child_action_set->action_down->script_selector  & 0xFF);
						*p_out_len   += 3;
					}

					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, calendar_name_active */
				}
				break;
			default:
				break;
		}
	}
	else if (req_type == REQ_TYPE_ACTION)
	{
		/* Not available */
	}
	else
	{
		/* INTERNAL ERROR */
	}

	return (response_result);
}
#endif

/******************************************************************************
* Function Name : R_OBIS_DistributeSingleActionScheduleClass
* Interface     : Unsigned8 R_OBIS_DistributeSingleActionScheduleClass(
*               :     Unsigned16 master_index,
*               :     Unsigned16 child_index,
*               :     req_type_t req_type,
*               :     service_type_t service_type,
*               :     st_Cosem_Attr_Desc *p_descriptor,
*               :     Unsigned8	*pdata,
*               :     Unsigned8	*p_out_buf,
*               :     Unsigned16 *p_out_len
*               : );
* Description   : Distributor function for Clock class
* Arguments     : Unsigned16 master_index -
*               :    [In] Id of the row in master list
*               : Unsigned16 child_index -
*               :    [In] Id of the row in child list
*               : req_type_t req_type -
*               :    [In] Request type (GET/SET/ACTION)
*               : service_type_t service_type -
*               :    [In] Service type (NORMAL/BLOCK/LIST)
*               : st_Cosem_Attr_Desc *p_descriptor -
*               :    [In] COSEM object descriptor
*               : Unsigned8 * pdata -
*               :    [In] Data in case of SET/ACTION
*               : Unsigned8 * p_out_buf -
*               :    [Out] Output buffer for encoded data
*               : Unsigned16 * p_out_len -
*               :    [Out] Output buffer length for encoded data
* Function Calls: TBD
* Return Value  : Unsigned8, distribution status
******************************************************************************/
#if (defined(USED_CLASS_22) && USED_CLASS_22 == 1)
Unsigned8 R_OBIS_DistributeSingleActionScheduleClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
)
{
	/* Result of response, def. is not Matched */
	Unsigned8				response_result = TYPE_UNMATCH;
	Unsigned8				i;
//	Unsigned8				j;
	Unsigned16				temp_out_len;
	Unsigned16 				tmp_int;
	date_t 					* tt;
	time_t 					* tt1;
	class22_child_record_t	*p_child_record;
	Apprtc 					rtc;

	p_child_record = (class22_child_record_t *)(
		R_OBIS_FindChildRecordByIndex(master_index, child_index)
	);

	/* Classify the code based on Attribute or Method request */
	if (req_type == REQ_TYPE_GET || req_type == REQ_TYPE_SET)
	{
		switch (p_descriptor->Attr_ID)
		{
			case 0: /* attribute 0 */
				/* Not support */
				break;

			case 1: /* logical_name */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,							/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,	/* Max length of g_ServiceBuffer */
						ATTR_TYPE_OCTET_STRING,				/* Octet-string */
						p_descriptor->Instance_ID,			/* OBIS code from the request */
						6);									/* OBIS code is 6 chars */

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, OBIS code is read only */
				}
				break;

			case 2:	/* executed_script  */
			if (req_type == REQ_TYPE_GET)
				{
					*p_out_buf++ = ATTR_TYPE_STRUCTURE;				/* Structure type */
					*p_out_buf++ = 2;								/* No. elements */
					*p_out_len   = 2;

					/* script_logical_name */
					*p_out_buf++ =  ATTR_TYPE_OCTET_STRING;
					*p_out_buf++ =  6;								/* OBIS code is 6 chars */
					*p_out_buf++ =  (Unsigned8)p_child_record->p_executed_script->script_logical_name[0];
					*p_out_buf++ =  (Unsigned8)p_child_record->p_executed_script->script_logical_name[1];
					*p_out_buf++ =  (Unsigned8)p_child_record->p_executed_script->script_logical_name[2];
					*p_out_buf++ =  (Unsigned8)p_child_record->p_executed_script->script_logical_name[3];
					*p_out_buf++ =  (Unsigned8)p_child_record->p_executed_script->script_logical_name[4];
					*p_out_buf++ =  (Unsigned8)p_child_record->p_executed_script->script_logical_name[5];
					*p_out_len   += 8;
					/* Script ID */
					*p_out_buf++ = ATTR_TYPE_LONG_UNSIGNED;
					*p_out_buf++ = (Unsigned8)(p_child_record->p_executed_script->script_selector >> 8);
					*p_out_buf++ = (Unsigned8)(p_child_record->p_executed_script->script_selector  & 0xFF);
					*p_out_len   += 3;

					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, executed_script */
				}
				break;
			case 3:	/* type  */
				response_result = R_OBIS_DistributeAttrBuffer(
					NULL,											/* Access method option */
					ATTR_TYPE_ENUM,									/* Type */
					1,												/* Size */
					p_child_record->p_type,							/* Buffer of value */
					child_index,									/* Child ID */
					req_type,										/* Request type */
					service_type,									/* Service type */
					p_descriptor,									/* Pointer to COSEM descriptor */
					pdata,											/* Data from COSEM Appl layer */
					p_out_buf,										/* Data pointer for get */
					p_out_len										/* Data length pointer for get */
				);
				break;

			case 4:	/* execution_time  */
				if (req_type == REQ_TYPE_GET)
				{
					/*
					 * execution_time_date_t array
					 */
					execution_time_date_t		*p_child_exe_time;
				        
					*p_out_buf++ = ATTR_TYPE_ARRAY;						/* Structure type */
					*p_out_buf++ = p_child_record->nr_execution_time;	/* No. elements */
					*p_out_len   = 2;

					/* execution_time_date_t */
					for (i = 0; i < p_child_record->nr_execution_time; i ++)
					{
						p_child_exe_time = (execution_time_date_t *)(p_child_record->p_execution_time + i);
						*p_out_buf++ = ATTR_TYPE_STRUCTURE;				/* Structure type */
						*p_out_buf++ = 2;								/* 2 elements */
						*p_out_len   += 2;

						/* time */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,									/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,			/* Max length of g_ServiceBuffer */
							ATTR_TYPE_OCTET_STRING,						/* Type */
							(Unsigned8 *)p_child_exe_time->p_time,		/* Buffer */
							sizeof(time_t)								/* Length */
						);
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;

						/* date */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,									/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,			/* Max length of g_ServiceBuffer */
							ATTR_TYPE_OCTET_STRING,						/* Type */
							(Unsigned8 *)p_child_exe_time->p_date,		/* Buffer */
							5											/* Length: sizeof(date_t) */
						);
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;

					}

					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else if (req_type == REQ_TYPE_SET)
				{
					response_result = TYPE_UNMATCH;

					/* Decode execution_time_date_t array from client */
					if(*pdata++ != ATTR_TYPE_ARRAY)
					{
						return response_result;
					}
					
					if(*pdata++ != p_child_record->nr_execution_time)
					{
						return response_result;
					}
					
					/* Decode execution_time_date_t */
					/* execution_time_date_t */
					for (i = 0; i < p_child_record->nr_execution_time; i ++)
					{
						
						execution_time_date_t		*p_child_exe_time;
						p_child_exe_time = (execution_time_date_t *)(p_child_record->p_execution_time + i);

						if(*pdata++ != ATTR_TYPE_STRUCTURE)
						{
							return response_result;
						}
						if(*pdata++ != 2) /* 2 elements */
						{
							return response_result;
						}
						/* time */
						if (R_OBIS_IsValidTime(pdata)== 0) // Invalid
						{
							return response_result;
							//break;
						}
						temp_out_len = R_OBIS_DecodeAttribute(
							(Unsigned8 *) p_child_exe_time->p_time,		/* value_buf */
							sizeof(time_t),								/* value_buf_len */
							ATTR_TYPE_OCTET_STRING,						/* attr_type */
							pdata,										/* buf */
							OBIS_SERVICE_DATA_BUFFER_LENGTH				/* buf_len */
						);
						pdata += temp_out_len;
						tt1 =(time_t *) p_child_exe_time->p_time;
						
						/* date */
						if (R_OBIS_IsValidDate(pdata)== 0) // Invalid
						{
							return response_result;
							//break;
						}
						temp_out_len = R_OBIS_DecodeAttribute(
							(Unsigned8 *) p_child_exe_time->p_date,		/* value_buf */
							5,											/* value_buf_len */
							ATTR_TYPE_OCTET_STRING,						/* attr_type */
							pdata,										/* buf */
							OBIS_SERVICE_DATA_BUFFER_LENGTH				/* buf_len */
						);
						pdata += temp_out_len;
						
						
						
						tt =(date_t *) p_child_exe_time->p_date;
						rtc.day = tt->day_of_month;
						rtc.month = tt->month;
						
						tmp_int=tt->year_high;
						tmp_int=(tmp_int*256)+tt->year_low;
						
						rtc.year=(tmp_int)%2000;
						
						rtc.hour = tt1->hour;
						rtc.min = tt1->minute;
						rtc.sec=tt1->second;
						if((tt->year_low==0xFF)&&(tt->year_high==0xFF)&&(tt->month==0xFF))
						{
							if(tt->day_of_month==0xFF)
								R_USER_Class20_SETACTION_Time(0,3);// md reset
							else
								R_USER_Class20_SETACTION_Time(rtc.day,2);// bill day set
							
						}
						else
							R_USER_Class20_SETACTION_Time(ConvertTimeToTimeCounter(&rtc),1);
						
						
					}
				 
				///  Set_PP(0,3);
				 //t emp_out_len = R_OBIS_SetEMData( EM_BILL_DATE, (void *)p_child_exe_time->p_date);
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, calendar_name_active */
				}
				break;
			default:
				break;
		}
	}
	else if (req_type == REQ_TYPE_ACTION)
	{
		/* Not available */
	}
	else
	{
		/* INTERNAL ERROR */
	}

	return (response_result);
}
#endif

/******************************************************************************
* Function Name : R_OBIS_DistributeIECHDLCSetupClass
* Interface     : Unsigned8 R_OBIS_DistributeIECHDLCSetupClass(
*               :     Unsigned16 master_index,
*               :     Unsigned16 child_index,
*               :     req_type_t req_type,
*               :     service_type_t service_type,
*               :     st_Cosem_Attr_Desc *p_descriptor,
*               :     Unsigned8	*pdata,
*               :     Unsigned8	*p_out_buf,
*               :     Unsigned16 *p_out_len
*               : );
* Description   : Distributor function for IEC HDLC Setup
* Arguments     : Unsigned16 master_index -
*               :    [In] Id of the row in master list
*               : Unsigned16 child_index -
*               :    [In] Id of the row in child list
*               : req_type_t req_type -
*               :    [In] Request type (GET/SET/ACTION)
*               : service_type_t service_type -
*               :    [In] Service type (NORMAL/BLOCK/LIST)
*               : st_Cosem_Attr_Desc *p_descriptor -
*               :    [In] COSEM object descriptor
*               : Unsigned8 * pdata -
*               :    [In] Data in case of SET/ACTION
*               : Unsigned8 * p_out_buf -
*               :    [Out] Output buffer for encoded data
*               : Unsigned16 * p_out_len -
*               :    [Out] Output buffer length for encoded data
* Function Calls: TBD
* Return Value  : Unsigned8, distribution status
******************************************************************************/
#if (defined(USED_CLASS_23) && USED_CLASS_23 == 1)
Unsigned8 R_OBIS_DistributeIECHDLCSetupClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
)
{
	/* Result of response, def. is not Matched */
	Unsigned8				response_result = TYPE_UNMATCH;
	Unsigned16 *p_temp_buf;
	//Unsigned8  *p_temp_len;
	class23_child_record_t	*p_child_record;

	p_child_record = (class23_child_record_t *)(
		R_OBIS_FindChildRecordByIndex(master_index, child_index)
	);

	/* Classify the code based on Attribute or Method request */
	if (req_type == REQ_TYPE_GET || req_type == REQ_TYPE_SET)
	{
		switch (p_descriptor->Attr_ID)
		{
			case 0: /* attribute 0 */
				/* Not support */
				break;

			case 1: /* logical_name */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,							/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,	/* Max length of g_ServiceBuffer */
						ATTR_TYPE_OCTET_STRING,				/* Octet-string */
						p_descriptor->Instance_ID,			/* OBIS code from the request */
						6);									/* OBIS code is 6 chars */

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, OBIS code is read only */
				}
				break;

			case 2:	/* comm_speed  */
				response_result = R_OBIS_DistributeAttrBuffer(
					NULL,									/* Access method option */
					ATTR_TYPE_ENUM,							/* Type */
					1,										/* Size */
					p_child_record->p_comm_speed,			/* Buffer of value */
					child_index,							/* Child ID */
					req_type,								/* Request type */
					service_type,							/* Service type */
					p_descriptor,							/* Pointer to COSEM descriptor */
					pdata,									/* Data from COSEM Appl layer */
					p_out_buf,								/* Data pointer for get */
					p_out_len								/* Data length pointer for get */
				);
				break;
			case 3:	/* window_size_transmit  */
			case 4:	/* window_size_receive  */
			case 5:	/* max_info_field_length_transmit  */
			case 6:	/* max_info_field_length_receive  */
			case 7:	/* inter_octet_time_out  */
			case 8:	/* inactivity_time_out  */
			case 9:	/* device_address  */
				switch (p_descriptor->Attr_ID)
				{
					case 3:	/* window_size_transmit  */
						p_temp_buf = (Unsigned16 *)(p_child_record->p_window_size_transmit);
						break;
					case 4:	/* window_size_receive  */
						p_temp_buf = (Unsigned16 *)(p_child_record->p_window_size_receive);
						break;
					case 5:	/* max_info_field_length_transmit  */
						p_temp_buf = (Unsigned16 *)(p_child_record->p_max_info_field_length_transmit);
						break;
					case 6:	/* max_info_field_length_receive  */
						p_temp_buf = (Unsigned16 *)(p_child_record->p_max_info_field_length_receive);
						break;
					case 7:	/* inter_octet_time_out  */
						p_temp_buf = (Unsigned16 *)(p_child_record->p_inter_octet_time_out);
						break;
					case 8:	/* inactivity_time_out  */
						p_temp_buf = (Unsigned16 *)(p_child_record->p_inactivity_time_out);
						break;
					case 9:	/* device_address  */
						p_temp_buf = (Unsigned16 *)(p_child_record->p_device_address);
						break;
					default:
						/* Fatal error, need to debug */
						return TYPE_UNMATCH;
				}
				
				if(g_MasterTable[master_index].class_version == 0)
				{
					switch (p_descriptor->Attr_ID)
					{
						case 3:
						case 4:
						case 5:
						case 6:
							response_result = R_OBIS_DistributeAttrBuffer(
								NULL,									/* Access method option */
								ATTR_TYPE_UNSIGNED,						/* Type */
								1,										/* Size */
								(Unsigned8 *)p_temp_buf,				/* Buffer of value */
								child_index,							/* Child ID */
								req_type,								/* Request type */
								service_type,							/* Service type */
								p_descriptor,							/* Pointer to COSEM descriptor */
								pdata,									/* Data from COSEM Appl layer */
								p_out_buf,								/* Data pointer for get */
								p_out_len								/* Data length pointer for get */
							);
							break;
						case 7:
						case 8:
						case 9:
							response_result = R_OBIS_DistributeAttrBuffer(
								NULL,									/* Access method option */
								ATTR_TYPE_LONG_UNSIGNED,				/* Type */
								2,										/* Size */
								(Unsigned8 *)p_temp_buf,				/* Buffer of value */
								child_index,							/* Child ID */
								req_type,								/* Request type */
								service_type,							/* Service type */
								p_descriptor,							/* Pointer to COSEM descriptor */
								pdata,									/* Data from COSEM Appl layer */
								p_out_buf,								/* Data pointer for get */
								p_out_len								/* Data length pointer for get */
							);
							break;
						default:
							break;
					}
				}
				else		/* Version1 */
				{
					switch (p_descriptor->Attr_ID)
					{
						case 3:
						case 4:
							response_result = R_OBIS_DistributeAttrBuffer(
								NULL,									/* Access method option */
								ATTR_TYPE_UNSIGNED,						/* Type */
								1,										/* Size */
								(Unsigned8 *)p_temp_buf,				/* Buffer of value */
								child_index,							/* Child ID */
								req_type,								/* Request type */
								service_type,							/* Service type */
								p_descriptor,							/* Pointer to COSEM descriptor */
								pdata,									/* Data from COSEM Appl layer */
								p_out_buf,								/* Data pointer for get */
								p_out_len								/* Data length pointer for get */
							);
							break;
						case 5:
						case 6:
						case 7:
						case 8:
						case 9:
							response_result = R_OBIS_DistributeAttrBuffer(
								NULL,									/* Access method option */
								ATTR_TYPE_LONG_UNSIGNED,				/* Type */
								2,										/* Size */
								(Unsigned8 *)p_temp_buf,				/* Buffer of value */
								child_index,							/* Child ID */
								req_type,								/* Request type */
								service_type,							/* Service type */
								p_descriptor,							/* Pointer to COSEM descriptor */
								pdata,									/* Data from COSEM Appl layer */
								p_out_buf,								/* Data pointer for get */
								p_out_len								/* Data length pointer for get */
							);
							break;
						default:
							break;
					}
				}
				break;
			default:
				break;
		}
	}
	else if (req_type == REQ_TYPE_ACTION)
	{
		/* Not available */
	}
	else
	{
		/* INTERNAL ERROR */
	}

	return (response_result);
}
#endif

/******************************************************************************
* Function Name : R_OBIS_DistributeIECTwistedPairClass
* Interface     : Unsigned8 R_OBIS_DistributeIECTwistedPairClass(
*               :     Unsigned16 master_index,
*               :     Unsigned16 child_index,
*               :     req_type_t req_type,
*               :     service_type_t service_type,
*               :     st_Cosem_Attr_Desc *p_descriptor,
*               :     Unsigned8	*pdata,
*               :     Unsigned8	*p_out_buf,
*               :     Unsigned16 *p_out_len
*               : );
* Description   : Distributor function for IEC HDLC Setup
* Arguments     : Unsigned16 master_index -
*               :    [In] Id of the row in master list
*               : Unsigned16 child_index -
*               :    [In] Id of the row in child list
*               : req_type_t req_type -
*               :    [In] Request type (GET/SET/ACTION)
*               : service_type_t service_type -
*               :    [In] Service type (NORMAL/BLOCK/LIST)
*               : st_Cosem_Attr_Desc *p_descriptor -
*               :    [In] COSEM object descriptor
*               : Unsigned8 * pdata -
*               :    [In] Data in case of SET/ACTION
*               : Unsigned8 * p_out_buf -
*               :    [Out] Output buffer for encoded data
*               : Unsigned16 * p_out_len -
*               :    [Out] Output buffer length for encoded data
* Function Calls: TBD
* Return Value  : Unsigned8, distribution status
******************************************************************************/
#if (defined(USED_CLASS_24) && USED_CLASS_24 == 1)
Unsigned8 R_OBIS_DistributeIECTwistedPairClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
)
{
	/* Result of response, def. is not Matched */
	Unsigned8				response_result = TYPE_UNMATCH;
	Unsigned8 i;
	class24_child_record_t	*p_child_record;
	Unsigned16				temp_out_len;

	p_child_record = (class24_child_record_t *)(
		R_OBIS_FindChildRecordByIndex(master_index, child_index)
	);

	/* Classify the code based on Attribute or Method request */
	if (req_type == REQ_TYPE_GET || req_type == REQ_TYPE_SET)
	{
		switch (p_descriptor->Attr_ID)
		{
			case 0: /* attribute 0 */
				/* Not support */
				break;

			case 1: /* logical_name */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,							/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,	/* Max length of g_ServiceBuffer */
						ATTR_TYPE_OCTET_STRING,				/* Octet-string */
						p_descriptor->Instance_ID,			/* OBIS code from the request */
						6);									/* OBIS code is 6 chars */

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, OBIS code is read only */
				}
				break;

			case 2:
				if (req_type == REQ_TYPE_GET)
				{
					if(g_MasterTable[master_index].class_version == 0)
					{	/* Version 0: secondary_address */
						*p_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,							/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,	/* Max length of g_ServiceBuffer */
							ATTR_TYPE_OCTET_STRING,				/* Octet-string */
							p_child_record->secondary_address,	/* OBIS code from the request */
							6);									/* OBIS code is 6 chars */

						/* Success */
						response_result = DATA_ACCESS_RESULT_SUCCESS;
					}
					else /* Version 1: Mode */ 
					{
						*p_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,								/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,		/* Max length of g_ServiceBuffer */
							ATTR_TYPE_ENUM,							/* Type */
							(Unsigned8 *)p_child_record->p_mode,	/* Buffer */
							1);										/* Length */

						/* Success */
						response_result = DATA_ACCESS_RESULT_SUCCESS;
					}
				}
				else
				{
					/* Do nothing, attr is read only */
				}
				break;

			case 3:
				if (req_type == REQ_TYPE_GET)
				{
					if(g_MasterTable[master_index].class_version == 0)
					{	/* Version 0: primary_address_list */

						response_result = DATA_ACCESS_RESULT_SUCCESS;
						/*
						* primary_address_list_type array
						*/
						*p_out_buf++ = ATTR_TYPE_ARRAY;								/* Array type */
						*p_out_buf++ = *p_child_record->p_primary_address_list_len;	/* number of elements */
						*p_out_len   = 2;
	
						for (i = 0; i < *p_child_record->p_primary_address_list_len; i ++)
						{
							/* octet-string */
							temp_out_len = R_OBIS_EncodeAttribute(
								p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
								OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
								ATTR_TYPE_OCTET_STRING,											/* Type */
								(Unsigned8 *)(p_child_record->p_primary_address_list + i),		/* Buffer */
								1																/* Length */
							);
							p_out_buf = p_out_buf + temp_out_len;
							*p_out_len += temp_out_len;
						}
					}
					else	/* Version 1: Comm speed */ 
					{
						response_result = R_OBIS_DistributeAttrBuffer(
							NULL,									/* Access method option */
							ATTR_TYPE_ENUM,							/* Type */
							1,										/* Size */
							p_child_record->p_comm_speed,			/* Buffer of value */
							child_index,							/* Child ID */
							req_type,								/* Request type */
							service_type,							/* Service type */
							p_descriptor,							/* Pointer to COSEM descriptor */
							pdata,									/* Data from COSEM Appl layer */
							p_out_buf,								/* Data pointer for get */
							p_out_len								/* Data length pointer for get */
							);
					}
				}
				else
				{
					/* Do nothing */
				}
				break;
			case 4:
				if (req_type == REQ_TYPE_GET)
				{
					if(g_MasterTable[master_index].class_version == 0)
					{	/* Version 0: tabi_list */
						response_result = DATA_ACCESS_RESULT_SUCCESS;
						/*
						* tabi_element array
						*/
						*p_out_buf++ = ATTR_TYPE_ARRAY;								/* Array type */
						*p_out_buf++ = *p_child_record->p_nr_tabi_list;				/* number of elements */
						*p_out_len   = 2;
	
						for (i = 0; i < *p_child_record->p_nr_tabi_list; i ++)
						{
							/* integer */
							temp_out_len = R_OBIS_EncodeAttribute(
								p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
								OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
								ATTR_TYPE_INTEGER,												/* Type */
								(Unsigned8 *)(p_child_record->p_tabi_list + i),					/* Buffer */
								1																/* Length */
							);
							p_out_buf = p_out_buf + temp_out_len;
							*p_out_len += temp_out_len;
						}
					}
					else	/* Version 1: primary_address_list */
					{
						response_result = DATA_ACCESS_RESULT_SUCCESS;
						/*
						* primary_address_list_type array
						*/
						*p_out_buf++ = ATTR_TYPE_ARRAY;								/* Array type */
						*p_out_buf++ = *p_child_record->p_primary_address_list_len;	/* number of elements */
						*p_out_len   = 2;
	
						for (i = 0; i < *p_child_record->p_primary_address_list_len; i ++)
						{
							/* unsigned */
							temp_out_len = R_OBIS_EncodeAttribute(
								p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
								OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
								ATTR_TYPE_UNSIGNED,											/* Type */
								(Unsigned8 *)(p_child_record->p_primary_address_list + i),		/* Buffer */
								1																/* Length */
							);
							p_out_buf = p_out_buf + temp_out_len;
							*p_out_len += temp_out_len;
						}
					}
				}
				else
				{
					/* Do nothing */
				}
				break;

			case 5:	/* fatal_error */
				if(g_MasterTable[master_index].class_version == 0)
				{	/* Version 0: fatal_error */
					/* distribute the attr buffer */
					response_result = R_OBIS_DistributeAttrBuffer(
						&p_child_record->fatal_error.access,		/* Access method option */
						p_child_record->fatal_error.choice.type,	/* Type */
						p_child_record->fatal_error.choice.size,	/* Size */
						p_child_record->fatal_error.buffer,			/* Buffer of value */
						child_index,								/* Child ID */
						req_type,									/* Request type */
						service_type,								/* Service type */
						p_descriptor,								/* Pointer to COSEM descriptor */
						pdata,										/* Data from COSEM Appl layer */
						p_out_buf,									/* Data pointer for get */
						p_out_len									/* Data length pointer for get */
					);
				}
				else	/* Version 1: tabi_list */
				{
					response_result = DATA_ACCESS_RESULT_SUCCESS;
					/*
					* tabi_element array
					*/
					*p_out_buf++ = ATTR_TYPE_ARRAY;								/* Array type */
					*p_out_buf++ = *p_child_record->p_nr_tabi_list;				/* number of elements */
					*p_out_len   = 2;
	
					for (i = 0; i < *p_child_record->p_nr_tabi_list; i ++)
					{
						/* integer */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
							ATTR_TYPE_INTEGER,												/* Type */
							(Unsigned8 *)(p_child_record->p_tabi_list + i),					/* Buffer */
							1																/* Length */
						);
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;
					}
				}
				break;
			default:
				break;
		}
	}
	else if (req_type == REQ_TYPE_ACTION)
	{
		/* Not available */
	}
	else
	{
		/* INTERNAL ERROR */
	}

	return (response_result);
}
#endif

/******************************************************************************
* Function Name : R_OBIS_DistributeUtilityTablesClass
* Interface     : Unsigned8 R_OBIS_DistributeUtilityTablesClass(
*               :     Unsigned16 master_index,
*               :     Unsigned16 child_index,
*               :     req_type_t req_type,
*               :     service_type_t service_type,
*               :     st_Cosem_Attr_Desc *p_descriptor,
*               :     Unsigned8	*pdata,
*               :     Unsigned8	*p_out_buf,
*               :     Unsigned16 *p_out_len
*               : );
* Description   : Distributor function for DRegister class
* Arguments     : Unsigned16 master_index -
*               :    [In] Id of the row in master list
*               : Unsigned16 child_index -
*               :    [In] Id of the row in child list
*               : req_type_t req_type -
*               :    [In] Request type (GET/SET/ACTION)
*               : service_type_t service_type -
*               :    [In] Service type (NORMAL/BLOCK/LIST)
*               : st_Cosem_Attr_Desc *p_descriptor -
*               :    [In] COSEM object descriptor
*               : Unsigned8 * pdata -
*               :    [In] Data in case of SET/ACTION
*               : Unsigned8 * p_out_buf -
*               :    [Out] Output buffer for encoded data
*               : Unsigned16 * p_out_len -
*               :    [Out] Output buffer length for encoded data
* Function Calls: TBD
* Return Value  : Unsigned8, distribution status
******************************************************************************/
#if (defined(USED_CLASS_26) && USED_CLASS_26 == 1)
Unsigned8 R_OBIS_DistributeUtilityTablesClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
)
{
	/* Result of response, def. is not Matched */
	Unsigned8				response_result = TYPE_UNMATCH;
	class26_child_record_t	*p_child_record;

	p_child_record = (class26_child_record_t *)(
		R_OBIS_FindChildRecordByIndex(master_index, child_index)
	);

	/* Classify the code based on Attribute or Method request */
	if (req_type == REQ_TYPE_GET || req_type == REQ_TYPE_SET)
	{
		switch (p_descriptor->Attr_ID)
		{
			case 0: /* attribute 0 */
				/* Not support */
				break;

			case 1: /* logical_name */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,							/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,	/* Max length of g_ServiceBuffer */
						ATTR_TYPE_OCTET_STRING,				/* Octet-string */
						p_descriptor->Instance_ID,			/* OBIS code from the request */
						6);									/* OBIS code is 6 chars */

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, OBIS code is read only */
				}
				break;

			case 2:	/* table_ID  */

				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				/* distribute the buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					NULL,										/* Access method option */
					ATTR_TYPE_LONG_UNSIGNED,					/* Type */
					2,											/* Size */
					(Unsigned8 *)&p_child_record->table_id,		/* Buffer of value */
					child_index,								/* Child ID */
					req_type,									/* Request type */
					service_type,								/* Service type */
					p_descriptor,								/* Pointer to COSEM descriptor */
					pdata,										/* Data from COSEM Appl layer */
					p_out_buf,									/* Data pointer for get */
					p_out_len									/* Data length pointer for get */
				);
				break;

			case 3:	/* length */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,							/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,	/* Max length of g_ServiceBuffer */
						ATTR_TYPE_DOUBLE_LONG_UNSIGNED,		/* Type */
						(Unsigned8 *)p_child_record->p_len,				/* Buffer */
						4);									/* Length */

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, OBIS code is read only */
				}
				break;


			case 0x0004:	/* buffer (normal) */

				if (req_type == REQ_TYPE_GET)
				{
					response_result = R_OBIS_EncodeGenericBuffer(
						p_child_record->p_buff,		/* [In] Pointer to buffer_t                       */
						0,							/* [In] selective_access                          */
						master_index,				/* [In] Id of the row in master list              */
						child_index,				/* [In] Id of the row in child list               */
						pdata,						/* [In] Pointer to the selector (from Client)     */
						service_type,				/* [In] Service type                              */
						p_out_buf,					/* [Out] Data pointer for get                     */
						p_out_len					/* [Out] Data length pointer for get              */
					);
				}
				else
				{
					/* Do nothing, buffer is read only */
				}
				break;

			case 0x0104:	/* buffer (selective access) */

				if (req_type == REQ_TYPE_GET)
				{
					response_result = R_OBIS_EncodeGenericBuffer(
						p_child_record->p_buff,		/* [In] Pointer to buffer_t                       */
						1,							/* [In] selective_access                          */
						master_index,				/* [In] Id of the row in master list              */
						child_index,				/* [In] Id of the row in child list               */
						pdata,						/* [In] Pointer to the selector (from Client)     */
						service_type,				/* [In] Service type                              */
						p_out_buf,					/* [Out] Data pointer for get                     */
						p_out_len					/* [Out] Data length pointer for get              */
					);
				}
				else
				{
					/* Do nothing, buffer is read only */
				}
				break;

			default:
				break;
		}
	}
	else if (req_type == REQ_TYPE_ACTION)
	{
		/* Not available */
	}
	else
	{
		/* INTERNAL ERROR */
	}

	return (response_result);
}
#endif

/******************************************************************************
* Function Name : R_OBIS_DistributeModemConfigClass
* Interface     : Unsigned8 R_OBIS_DistributeModemConfigClass(
*               :     Unsigned16 master_index,
*               :     Unsigned16 child_index,
*               :     req_type_t req_type,
*               :     service_type_t service_type,
*               :     st_Cosem_Attr_Desc *p_descriptor,
*               :     Unsigned8	*pdata,
*               :     Unsigned8	*p_out_buf,
*               :     Unsigned16 *p_out_len
*               : );
* Description   : Distributor function for IEC HDLC Setup
* Arguments     : Unsigned16 master_index -
*               :    [In] Id of the row in master list
*               : Unsigned16 child_index -
*               :    [In] Id of the row in child list
*               : req_type_t req_type -
*               :    [In] Request type (GET/SET/ACTION)
*               : service_type_t service_type -
*               :    [In] Service type (NORMAL/BLOCK/LIST)
*               : st_Cosem_Attr_Desc *p_descriptor -
*               :    [In] COSEM object descriptor
*               : Unsigned8 * pdata -
*               :    [In] Data in case of SET/ACTION
*               : Unsigned8 * p_out_buf -
*               :    [Out] Output buffer for encoded data
*               : Unsigned16 * p_out_len -
*               :    [Out] Output buffer length for encoded data
* Function Calls: TBD
* Return Value  : Unsigned8, distribution status
******************************************************************************/
#if (defined(USED_CLASS_27) && USED_CLASS_27 == 1)
Unsigned8 R_OBIS_DistributeModemConfigClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
)
{
	/* Result of response, def. is not Matched */
	Unsigned8				response_result = TYPE_UNMATCH;
	Unsigned8 i;
	Unsigned16				temp_out_len;
	class27_child_record_t	*p_child_record;
	init_string_element_t	*p_init_string_child;
	value_t					*p_value_child;

	p_child_record = (class27_child_record_t *)(
		R_OBIS_FindChildRecordByIndex(master_index, child_index)
	);

	/* Classify the code based on Attribute or Method request */
	if (req_type == REQ_TYPE_GET || req_type == REQ_TYPE_SET)
	{
		switch (p_descriptor->Attr_ID)
		{
			case 0: /* attribute 0 */
				/* Not support */
				break;

			case 1: /* logical_name */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,							/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,	/* Max length of g_ServiceBuffer */
						ATTR_TYPE_OCTET_STRING,				/* Octet-string */
						p_descriptor->Instance_ID,			/* OBIS code from the request */
						6);									/* OBIS code is 6 chars */

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, OBIS code is read only */
				}
				break;

			case 2:	/* comm_speed  */
				response_result = R_OBIS_DistributeAttrBuffer(
					NULL,									/* Access method option */
					ATTR_TYPE_ENUM,							/* Type */
					1,										/* Size */
					p_child_record->p_comm_speed,			/* Buffer of value */
					child_index,							/* Child ID */
					req_type,								/* Request type */
					service_type,							/* Service type */
					p_descriptor,							/* Pointer to COSEM descriptor */
					pdata,									/* Data from COSEM Appl layer */
					p_out_buf,								/* Data pointer for get */
					p_out_len								/* Data length pointer for get */
				);
				break;

			case 3:	/* initialization_string */
				if (req_type == REQ_TYPE_GET)
				{
					response_result = DATA_ACCESS_RESULT_SUCCESS;
					/*
					 * initialization_string_element array
					 */
					*p_out_buf++ = ATTR_TYPE_ARRAY;								/* Array type */
					*p_out_buf++ = *p_child_record->p_nr_init_string;			/* number of elements */
					*p_out_len   = 2;

					p_init_string_child = (init_string_element_t *) p_child_record->p_init_string;
					for (i = 0; i < *p_child_record->p_nr_init_string; i ++)
					{
						p_init_string_child = (init_string_element_t *) (p_child_record->p_init_string + i);
						/* initialization_string_element */
						*p_out_buf++ = ATTR_TYPE_STRUCTURE;						/* Structure type */
						if(g_MasterTable[master_index].class_version == 0)
						{
							*p_out_buf++ = 2;							/* 2 elements */
						}
						else
						{
							*p_out_buf++ = 3;							/* 3 elements */
						}
														
						*p_out_len   += 2;

						/* request */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
							ATTR_TYPE_OCTET_STRING,											/* Type */
							(Unsigned8 *)(p_init_string_child->p_request),					/* Buffer */
							p_init_string_child->nr_request									/* Length */
						);
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;

						/* response */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
							ATTR_TYPE_OCTET_STRING,											/* Type */
							(Unsigned8 *)(p_init_string_child->p_response),					/* Buffer */
							p_init_string_child->nr_response								/* Length */
						);
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;
						
						if(g_MasterTable[master_index].class_version == 0)
						{
							/* DO nothing, version 0 has no delay_after_response */
						}
						else	/* Version 1 */
						{
							/* delay_after_response */
							temp_out_len = R_OBIS_EncodeAttribute(
								p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
								OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
								ATTR_TYPE_LONG_UNSIGNED,										/* Type */
								(Unsigned8 *)(p_init_string_child->p_delay_after_response),	/* Buffer */
								2																/* Length */
							);
							p_out_buf = p_out_buf + temp_out_len;
							*p_out_len += temp_out_len;
						}
					}

				}
				else
				{
					/* Do nothing */
				}
				break;

			case 4:	/* modem_profile   */
				if (req_type == REQ_TYPE_GET)
				{
					response_result = DATA_ACCESS_RESULT_SUCCESS;
					/*
					 * modem_profile_element array
					 */
					*p_out_buf++ = ATTR_TYPE_ARRAY;								/* Array type */
					*p_out_buf++ = *p_child_record->p_nr_modem_profile;			/* number of elements */
					*p_out_len   = 2;

					for (i = 0; i < *p_child_record->p_nr_modem_profile; i ++)
					{
						p_value_child = (value_t *) (p_child_record->p_modem_profile + i);
						/* modem_profile_element */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
							p_value_child->choice.type,										/* Type */
							(Unsigned8 *)(p_value_child->buffer),							/* Buffer */
							p_value_child->choice.size										/* Length */
						);
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;
						if (*p_out_len > OBIS_SERVICE_DATA_BUFFER_LENGTH)
						{
							/* Overflow */
							response_result = DATA_BLK_UNAVAILABLE;
							return;
						}
					}

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing */
				}
				break;

			default:
				break;
		}
	}
	else if (req_type == REQ_TYPE_ACTION)
	{
		/* Not available */
	}
	else
	{
		/* INTERNAL ERROR */
	}

	return (response_result);
}
#endif

/******************************************************************************
* Function Name : R_OBIS_DistributeAutoAnswerClass
* Interface     : Unsigned8 R_OBIS_DistributeAutoAnswerClass(
*               :     Unsigned16 master_index,
*               :     Unsigned16 child_index,
*               :     req_type_t req_type,
*               :     service_type_t service_type,
*               :     st_Cosem_Attr_Desc *p_descriptor,
*               :     Unsigned8	*pdata,
*               :     Unsigned8	*p_out_buf,
*               :     Unsigned16 *p_out_len
*               : );
* Description   : Distributor function for IEC HDLC Setup
* Arguments     : Unsigned16 master_index -
*               :    [In] Id of the row in master list
*               : Unsigned16 child_index -
*               :    [In] Id of the row in child list
*               : req_type_t req_type -
*               :    [In] Request type (GET/SET/ACTION)
*               : service_type_t service_type -
*               :    [In] Service type (NORMAL/BLOCK/LIST)
*               : st_Cosem_Attr_Desc *p_descriptor -
*               :    [In] COSEM object descriptor
*               : Unsigned8 * pdata -
*               :    [In] Data in case of SET/ACTION
*               : Unsigned8 * p_out_buf -
*               :    [Out] Output buffer for encoded data
*               : Unsigned16 * p_out_len -
*               :    [Out] Output buffer length for encoded data
* Function Calls: TBD
* Return Value  : Unsigned8, distribution status
******************************************************************************/
#if (defined(USED_CLASS_28) && USED_CLASS_28 == 1)
Unsigned8 R_OBIS_DistributeAutoAnswerClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
)
{
	/* Result of response, def. is not Matched */
	Unsigned8				response_result = TYPE_UNMATCH;
	Unsigned8 i;
	Unsigned16				temp_out_len;
	class28_child_record_t	*p_child_record;

	p_child_record = (class28_child_record_t *)(
		R_OBIS_FindChildRecordByIndex(master_index, child_index)
	);

	/* Classify the code based on Attribute or Method request */
	if (req_type == REQ_TYPE_GET || req_type == REQ_TYPE_SET)
	{
		switch (p_descriptor->Attr_ID)
		{
			case 0: /* attribute 0 */
				/* Not support */
				break;

			case 1: /* logical_name */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,							/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,	/* Max length of g_ServiceBuffer */
						ATTR_TYPE_OCTET_STRING,				/* Octet-string */
						p_descriptor->Instance_ID,			/* OBIS code from the request */
						6);									/* OBIS code is 6 chars */

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, OBIS code is read only */
				}
				break;

			case 2:	/* mode  */
				response_result = R_OBIS_DistributeAttrBuffer(
					NULL,									/* Access method option */
					ATTR_TYPE_ENUM,							/* Type */
					1,										/* Size */
					p_child_record->p_mode,					/* Buffer of value */
					child_index,							/* Child ID */
					req_type,								/* Request type */
					service_type,							/* Service type */
					p_descriptor,							/* Pointer to COSEM descriptor */
					pdata,									/* Data from COSEM Appl layer */
					p_out_buf,								/* Data pointer for get */
					p_out_len								/* Data length pointer for get */
				);
				break;

			case 3:	/* listening_window */
				if (req_type == REQ_TYPE_GET)
				{
					window_element_t		*p_window_element_child;
					response_result = DATA_ACCESS_RESULT_SUCCESS;
					/*
					 * window_element array
					 */
					*p_out_buf++ = ATTR_TYPE_ARRAY;								/* Array type */
					*p_out_buf++ = *p_child_record->p_nr_listen_window;			/* number of elements */
					*p_out_len   = 2;

					for (i = 0; i < *p_child_record->p_nr_listen_window; i ++)
					{
						p_window_element_child = (window_element_t *) (p_child_record->p_listen_window + i);
						/* window_element */
						*p_out_buf++ = ATTR_TYPE_STRUCTURE;						/* Structure type */
						*p_out_buf++ = 2;										/* 3 elements */
						*p_out_len   += 2;

						/* start _time */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
							ATTR_TYPE_OCTET_STRING,											/* Type */
							(Unsigned8 *)(p_window_element_child->p_start_time),			/* Buffer */
							sizeof(date_time_t)												/* Length */
						);
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;

						/* end _time */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
							ATTR_TYPE_OCTET_STRING,											/* Type */
							(Unsigned8 *)(p_window_element_child->p_end_time),				/* Buffer */
							sizeof(date_time_t)												/* Length */
						);
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;
					}

				}
				else
				{
					/* Do nothing */
				}
				break;

			case 4:	/* status   */
				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				/* distribute the attr buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					&p_child_record->status.access,		/* Access method option */
					p_child_record->status.choice.type,	/* Type */
					p_child_record->status.choice.size,	/* Size */
					p_child_record->status.buffer,		/* Buffer of value */
					child_index,						/* Child ID */
					req_type,							/* Request type */
					service_type,						/* Service type */
					p_descriptor,						/* Pointer to COSEM descriptor */
					pdata,								/* Data from COSEM Appl layer */
					p_out_buf,							/* Data pointer for get */
					p_out_len							/* Data length pointer for get */
				);

				break;
			case 5:	/* number_of_calls   */
				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				/* distribute the attr buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					NULL,								/* Access method option */
					ATTR_TYPE_UNSIGNED,					/* Type */
					sizeof(Unsigned8),					/* Size */
					p_child_record->p_nr_calls,			/* Buffer of value */
					child_index,						/* Child ID */
					req_type,							/* Request type */
					service_type,						/* Service type */
					p_descriptor,						/* Pointer to COSEM descriptor */
					pdata,								/* Data from COSEM Appl layer */
					p_out_buf,							/* Data pointer for get */
					p_out_len							/* Data length pointer for get */
				);

				break;
			case 6:	/* number_of_rings   */
				if (req_type == REQ_TYPE_GET)
				{
					/*
					 * nr_rings_t structure
					 */
					*p_out_buf++ = ATTR_TYPE_STRUCTURE;						/* Structure type */
					*p_out_buf++ = 2;										/* 4 elements */
					*p_out_len   = 2;
					/* nr_rings_in_window */
					temp_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,															/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,									/* Max length of g_ServiceBuffer */
						ATTR_TYPE_UNSIGNED,													/* Type */
						(Unsigned8 *)&p_child_record->p_nr_rings->nr_rings_in_window,		/* Buffer */
						1																	/* Length */
					);
					p_out_buf = p_out_buf + temp_out_len;
					*p_out_len += temp_out_len;

					/* nr_rings_out_of_window */
					temp_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,															/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,									/* Max length of g_ServiceBuffer */
						ATTR_TYPE_UNSIGNED,													/* Type */
						(Unsigned8 *)&p_child_record->p_nr_rings->nr_rings_out_of_window,	/* Buffer */
						1																	/* Length */
					);
					p_out_buf = p_out_buf + temp_out_len;
					*p_out_len += temp_out_len;

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing */
				}
				break;

			case 7: /* list_of_ allowed_ callers */
				if(g_MasterTable[master_index].class_version == 0)
				{
					/* Do nothing: not available in version 0 */
				}
				else if (req_type == REQ_TYPE_GET)
				{
					allowed_callers_element_t	*p_window_callers_element_child;
					response_result = DATA_ACCESS_RESULT_SUCCESS;
					/*
					 * allowed_callers_element array
					 */
					*p_out_buf++ = ATTR_TYPE_ARRAY;								/* Array type */
					*p_out_buf++ = *p_child_record->p_nr_allowed_caller;			/* number of elements */
					*p_out_len   = 2;

					for (i = 0; i < *p_child_record->p_nr_allowed_caller; i ++)
					{
						p_window_callers_element_child = (allowed_callers_element_t *) (p_child_record->p_allowed_caller + i);
						/* allowed_callers_element */
						*p_out_buf++ = ATTR_TYPE_STRUCTURE;						/* Structure type */
						*p_out_buf++ = 2;										/* 3 elements */
						*p_out_len   += 2;

						/* caller_id */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
							ATTR_TYPE_OCTET_STRING,											/* Type */
							(Unsigned8 *)(p_window_callers_element_child->p_caller_id),		/* Buffer */
							p_window_callers_element_child->caller_id_len					/* Length */
						);
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;

						/* call_type */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
							ATTR_TYPE_ENUM,													/* Type */
							(Unsigned8 *)(p_window_callers_element_child->call_type),		/* Buffer */
							1																/* Length */
						);
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;
					}

				}
				else
				{
					/* Do nothing */
				}
				break;
			default:
				break;
		}
	}
	else if (req_type == REQ_TYPE_ACTION)
	{
		/* Not available */
	}
	else
	{
		/* INTERNAL ERROR */
	}

	return (response_result);
}
#endif

/******************************************************************************
* Function Name : R_OBIS_DistributeAutoConnectClass
* Interface     : Unsigned8 R_OBIS_DistributeAutoConnectClass(
*               :     Unsigned16 master_index,
*               :     Unsigned16 child_index,
*               :     req_type_t req_type,
*               :     service_type_t service_type,
*               :     st_Cosem_Attr_Desc *p_descriptor,
*               :     Unsigned8	*pdata,
*               :     Unsigned8	*p_out_buf,
*               :     Unsigned16 *p_out_len
*               : );
* Description   : Distributor function for IEC HDLC Setup
* Arguments     : Unsigned16 master_index -
*               :    [In] Id of the row in master list
*               : Unsigned16 child_index -
*               :    [In] Id of the row in child list
*               : req_type_t req_type -
*               :    [In] Request type (GET/SET/ACTION)
*               : service_type_t service_type -
*               :    [In] Service type (NORMAL/BLOCK/LIST)
*               : st_Cosem_Attr_Desc *p_descriptor -
*               :    [In] COSEM object descriptor
*               : Unsigned8 * pdata -
*               :    [In] Data in case of SET/ACTION
*               : Unsigned8 * p_out_buf -
*               :    [Out] Output buffer for encoded data
*               : Unsigned16 * p_out_len -
*               :    [Out] Output buffer length for encoded data
* Function Calls: TBD
* Return Value  : Unsigned8, distribution status
******************************************************************************/
#if (defined(USED_CLASS_29) && USED_CLASS_29 == 1)
Unsigned8 R_OBIS_DistributeAutoConnectClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
)
{
	/* Result of response, def. is not Matched */
	Unsigned8				response_result = TYPE_UNMATCH;
	Unsigned8 i;
	Unsigned16				temp_out_len;
	window_element_t		*p_window_element_child;
	class29_child_record_t	*p_child_record;
	value_t					*p_value_child;

	p_child_record = (class29_child_record_t *)(
		R_OBIS_FindChildRecordByIndex(master_index, child_index)
	);

	/* Classify the code based on Attribute or Method request */
	if (req_type == REQ_TYPE_GET || req_type == REQ_TYPE_SET)
	{
		switch (p_descriptor->Attr_ID)
		{
			case 0: /* attribute 0 */
				/* Not support */
				break;

			case 1: /* logical_name */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,							/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,	/* Max length of g_ServiceBuffer */
						ATTR_TYPE_OCTET_STRING,				/* Octet-string */
						p_descriptor->Instance_ID,			/* OBIS code from the request */
						6);									/* OBIS code is 6 chars */

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, OBIS code is read only */
				}
				break;

			case 2:	/* mode  */
				response_result = R_OBIS_DistributeAttrBuffer(
					NULL,									/* Access method option */
					ATTR_TYPE_ENUM,							/* Type */
					1,										/* Size */
					p_child_record->p_mode,					/* Buffer of value */
					child_index,							/* Child ID */
					req_type,								/* Request type */
					service_type,							/* Service type */
					p_descriptor,							/* Pointer to COSEM descriptor */
					pdata,									/* Data from COSEM Appl layer */
					p_out_buf,								/* Data pointer for get */
					p_out_len								/* Data length pointer for get */
				);
				break;
			case 3:	/* repetitions  */
				response_result = R_OBIS_DistributeAttrBuffer(
					NULL,									/* Access method option */
					ATTR_TYPE_UNSIGNED,						/* Type */
					1,										/* Size */
					p_child_record->p_repetitions,			/* Buffer of value */
					child_index,							/* Child ID */
					req_type,								/* Request type */
					service_type,							/* Service type */
					p_descriptor,							/* Pointer to COSEM descriptor */
					pdata,									/* Data from COSEM Appl layer */
					p_out_buf,								/* Data pointer for get */
					p_out_len								/* Data length pointer for get */
				);
				break;
			case 4:	/* repetition_delay  */
				response_result = R_OBIS_DistributeAttrBuffer(
					NULL,									/* Access method option */
					ATTR_TYPE_LONG_UNSIGNED,				/* Type */
					2,										/* Size */
					p_child_record->p_repetition_delay,		/* Buffer of value */
					child_index,							/* Child ID */
					req_type,								/* Request type */
					service_type,							/* Service type */
					p_descriptor,							/* Pointer to COSEM descriptor */
					pdata,									/* Data from COSEM Appl layer */
					p_out_buf,								/* Data pointer for get */
					p_out_len								/* Data length pointer for get */
				);
				break;

			case 5:	/* calling_window */
				if (req_type == REQ_TYPE_GET)
				{
					response_result = DATA_ACCESS_RESULT_SUCCESS;
					/*
					 * window_element array
					 */
					*p_out_buf++ = ATTR_TYPE_ARRAY;								/* Array type */
					*p_out_buf++ = *p_child_record->p_nr_calling_window;		/* number of elements */
					*p_out_len   = 2;

					p_window_element_child = (window_element_t *) p_child_record->p_calling_window;
					for (i = 0; i < *p_child_record->p_nr_calling_window; i ++)
					{
						p_window_element_child = (window_element_t *) (p_child_record->p_calling_window + i);
						/* window_element */
						*p_out_buf++ = ATTR_TYPE_STRUCTURE;						/* Structure type */
						*p_out_buf++ = 2;										/* 3 elements */
						*p_out_len   += 2;

						/* start _time */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
							ATTR_TYPE_OCTET_STRING,											/* Type */
							(Unsigned8 *)(p_window_element_child->p_start_time),			/* Buffer */
							sizeof(date_time_t)												/* Length */
						);
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;

						/* end _time */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
							ATTR_TYPE_OCTET_STRING,											/* Type */
							(Unsigned8 *)(p_window_element_child->p_end_time),				/* Buffer */
							sizeof(date_time_t)												/* Length */
						);
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;
					}

				}
				else
				{
					/* Do nothing */
				}
				break;

			case 6:	/* destination_list */
				if (req_type == REQ_TYPE_GET)
				{
					response_result = DATA_ACCESS_RESULT_SUCCESS;
					/*
					 * destination array
					 */
					*p_out_buf++ = ATTR_TYPE_ARRAY;								/* Array type */
					*p_out_buf++ = *p_child_record->p_nr_destination_list;		/* number of elements */
					*p_out_len   = 2;

					for (i = 0; i < *p_child_record->p_nr_destination_list; i++)
					{
						p_value_child = (value_t *) (p_child_record->p_destination_list + i);
						/* destination */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
							p_value_child->choice.type,										/* Type */
							(Unsigned8 *)(p_value_child->buffer),							/* Buffer */
							p_value_child->choice.size										/* Length */
						);
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;
					}

				}
				else
				{
					/* Do nothing */
				}
				break;

			default:
				break;
		}
	}
	else if (req_type == REQ_TYPE_ACTION)
	{
		/* Not available */
	}
	else
	{
		/* INTERNAL ERROR */
	}

	return (response_result);
}
#endif

/******************************************************************************
* Function Name : R_OBIS_DistributeTCPUDPSetupClass
* Interface     : Unsigned8 R_OBIS_DistributeTCPUDPSetupClass(
*               :     Unsigned16 master_index,
*               :     Unsigned16 child_index,
*               :     req_type_t req_type,
*               :     service_type_t service_type,
*               :     st_Cosem_Attr_Desc *p_descriptor,
*               :     Unsigned8	*pdata,
*               :     Unsigned8	*p_out_buf,
*               :     Unsigned16 *p_out_len
*               : );
* Description   : Distributor function for TCP UDP Setup
* Arguments     : Unsigned16 master_index -
*               :    [In] Id of the row in master list
*               : Unsigned16 child_index -
*               :    [In] Id of the row in child list
*               : req_type_t req_type -
*               :    [In] Request type (GET/SET/ACTION)
*               : service_type_t service_type -
*               :    [In] Service type (NORMAL/BLOCK/LIST)
*               : st_Cosem_Attr_Desc *p_descriptor -
*               :    [In] COSEM object descriptor
*               : Unsigned8 * pdata -
*               :    [In] Data in case of SET/ACTION
*               : Unsigned8 * p_out_buf -
*               :    [Out] Output buffer for encoded data
*               : Unsigned16 * p_out_len -
*               :    [Out] Output buffer length for encoded data
* Function Calls: TBD
* Return Value  : Unsigned8, distribution status
******************************************************************************/
#if (defined(USED_CLASS_41) && USED_CLASS_41 == 1)
Unsigned8 R_OBIS_DistributeTCPUDPSetupClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
)
{
	/* Result of response, def. is not Matched */
	Unsigned8				response_result = TYPE_UNMATCH;
	Unsigned8 i;
	Unsigned16				temp_out_len;
	window_element_t		*p_window_element_child;
	class41_child_record_t	*p_child_record;
	value_t					*p_value_child;

	p_child_record = (class41_child_record_t *)(
		R_OBIS_FindChildRecordByIndex(master_index, child_index)
	);

	/* Classify the code based on Attribute or Method request */
	if (req_type == REQ_TYPE_GET || req_type == REQ_TYPE_SET)
	{
		switch (p_descriptor->Attr_ID)
		{
			case 0: /* attribute 0 */
				/* Not support */
				break;

			case 1: /* logical_name */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,							/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,	/* Max length of g_ServiceBuffer */
						ATTR_TYPE_OCTET_STRING,				/* Octet-string */
						p_descriptor->Instance_ID,			/* OBIS code from the request */
						6);									/* OBIS code is 6 chars */

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, OBIS code is read only */
				}
				break;

			case 2:	/* TCP UDP port  */
				response_result = R_OBIS_DistributeAttrBuffer(
					NULL,									/* Access method option */
					ATTR_TYPE_LONG_UNSIGNED,				/* Type */
					2,										/* Size */
					p_child_record->p_tcp_udp_port,			/* Buffer of value */
					child_index,							/* Child ID */
					req_type,								/* Request type */
					service_type,							/* Service type */
					p_descriptor,							/* Pointer to COSEM descriptor */
					pdata,									/* Data from COSEM Appl layer */
					p_out_buf,								/* Data pointer for get */
					p_out_len								/* Data length pointer for get */
				);
				break;

			case 3:	/* IP reference  */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,							/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,	/* Max length of g_ServiceBuffer */
						ATTR_TYPE_OCTET_STRING,				/* Octet-string */
						p_child_record->ip_reference,		/* OBIS code from the request */
						6);									/* OBIS code is 6 chars */

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, OBIS code is read only */
				}
				break;

			case 4:	/* MSS  */
				response_result = R_OBIS_DistributeAttrBuffer(
					NULL,									/* Access method option */
					ATTR_TYPE_LONG_UNSIGNED,				/* Type */
					2,										/* Size */
					p_child_record->p_mss,					/* Buffer of value */
					child_index,							/* Child ID */
					req_type,								/* Request type */
					service_type,							/* Service type */
					p_descriptor,							/* Pointer to COSEM descriptor */
					pdata,									/* Data from COSEM Appl layer */
					p_out_buf,								/* Data pointer for get */
					p_out_len								/* Data length pointer for get */
				);
				/* Range checking: min 40 */
				if(*(p_child_record->p_mss) < 40)
				{
					response_result = TYPE_UNMATCH;
				}
				break;

			case 5:	/* Number of sim conn  */
				response_result = R_OBIS_DistributeAttrBuffer(
					NULL,									/* Access method option */
					ATTR_TYPE_UNSIGNED,						/* Type */
					1,										/* Size */
					p_child_record->p_nb_of_sim_conn,		/* Buffer of value */
					child_index,							/* Child ID */
					req_type,								/* Request type */
					service_type,							/* Service type */
					p_descriptor,							/* Pointer to COSEM descriptor */
					pdata,									/* Data from COSEM Appl layer */
					p_out_buf,								/* Data pointer for get */
					p_out_len								/* Data length pointer for get */
				);
				break;

			case 6:	/* Inactivity time out  */
				response_result = R_OBIS_DistributeAttrBuffer(
					NULL,									/* Access method option */
					ATTR_TYPE_LONG_UNSIGNED,				/* Type */
					2,										/* Size */
					p_child_record->p_inactivity_time_out,	/* Buffer of value */
					child_index,							/* Child ID */
					req_type,								/* Request type */
					service_type,							/* Service type */
					p_descriptor,							/* Pointer to COSEM descriptor */
					pdata,									/* Data from COSEM Appl layer */
					p_out_buf,								/* Data pointer for get */
					p_out_len								/* Data length pointer for get */
				);
				break;
			default:
				break;
		}
	}
	else if (req_type == REQ_TYPE_ACTION)
	{
		/* Not available */
	}
	else
	{
		/* INTERNAL ERROR */
	}

	return (response_result);
}
#endif

/******************************************************************************
* Function Name : R_OBIS_DistributeRegisterTableClass
* Interface     : Unsigned8 R_OBIS_DistributeRegisterTableClass(
*               :     Unsigned16 master_index,
*               :     Unsigned16 child_index,
*               :     req_type_t req_type,
*               :     service_type_t service_type,
*               :     st_Cosem_Attr_Desc *p_descriptor,
*               :     Unsigned8	*pdata,
*               :     Unsigned8	*p_out_buf,
*               :     Unsigned16 *p_out_len
*               : );
* Description   : Distributor function for DRegister class
* Arguments     : Unsigned16 master_index -
*               :    [In] Id of the row in master list
*               : Unsigned16 child_index -
*               :    [In] Id of the row in child list
*               : req_type_t req_type -
*               :    [In] Request type (GET/SET/ACTION)
*               : service_type_t service_type -
*               :    [In] Service type (NORMAL/BLOCK/LIST)
*               : st_Cosem_Attr_Desc *p_descriptor -
*               :    [In] COSEM object descriptor
*               : Unsigned8 * pdata -
*               :    [In] Data in case of SET/ACTION
*               : Unsigned8 * p_out_buf -
*               :    [Out] Output buffer for encoded data
*               : Unsigned16 * p_out_len -
*               :    [Out] Output buffer length for encoded data
* Function Calls: TBD
* Return Value  : Unsigned8, distribution status
******************************************************************************/
#if (defined(USED_CLASS_61) && USED_CLASS_61 == 1)
Unsigned8 R_OBIS_DistributeRegisterTableClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
)
{
	/* Result of response, def. is not Matched */
	Unsigned8				i, j;
	Unsigned8				response_result = TYPE_UNMATCH;
	class61_child_record_t	*p_child_record;
	table_cell_def_t		*p_table_cell_def_child;
	dyn_value_t				*p_dyn_value_child;

	Unsigned16				temp_out_len;

	p_child_record = (class61_child_record_t *)(
		R_OBIS_FindChildRecordByIndex(master_index, child_index)
	);

	/* Classify the code based on Attribute or Method request */
	if (req_type == REQ_TYPE_GET || req_type == REQ_TYPE_SET)
	{
		switch (p_descriptor->Attr_ID)
		{
			case 0: /* attribute 0 */
				/* Not support */
				break;

			case 1: /* logical_name */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,							/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,	/* Max length of g_ServiceBuffer */
						ATTR_TYPE_OCTET_STRING,				/* Octet-string */
						p_descriptor->Instance_ID,			/* OBIS code from the request */
						6);									/* OBIS code is 6 chars */

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, OBIS code is read only */
				}
				break;

			case 2:	/* table_cell_values */
				if (req_type == REQ_TYPE_GET)
				{
					/*
					 * table_cell_entry array
					 */
					*p_out_buf++ = ATTR_TYPE_ARRAY;								/* Structure type */
					*p_out_buf++ = *p_child_record->p_table_cell_values_nr;		/* number of elements */
					*p_out_len   = 2;

					for (i = 0; i < *p_child_record->p_table_cell_values_nr; i++)
					{
						p_dyn_value_child = (dyn_value_t *)(p_child_record->p_table_cell_values + i);
						/* distribute the attr buffer */
						response_result = R_OBIS_DistributeAttrBuffer(
							&p_dyn_value_child->access,						/* Access method option */
							p_dyn_value_child->choice.type,					/* Type */
							p_dyn_value_child->choice.size,					/* Size */
							p_dyn_value_child->buffer,						/* Buffer of value */
							child_index,									/* Child ID */
							req_type,										/* Request type */
							service_type,									/* Service type */
							p_descriptor,									/* Pointer to COSEM descriptor */
							pdata,											/* Data from COSEM Appl layer */
							p_out_buf,										/* Data pointer for get */
							&temp_out_len									/* Data length pointer for get */
						);
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;
					}
				}
				else
				{
					/* Do nothing */
				}
				break;

			case 3:	/* table_cell_definition */
				if (req_type == REQ_TYPE_GET)
				{
					/*
					 * table_cell_def_t structure
					 */
					p_table_cell_def_child = (table_cell_def_t *)p_child_record->p_table_cell_definition;
					*p_out_buf++ = ATTR_TYPE_STRUCTURE;						/* Structure type */
					*p_out_buf++ = 4;										/* 4 elements */
					*p_out_len   = 2;
					/* class_id */
					temp_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
						ATTR_TYPE_LONG_UNSIGNED,										/* Type */
						(Unsigned8 *)&p_table_cell_def_child->class_id,					/* Buffer */
						sizeof(Unsigned16)												/* Length */
					);
					p_out_buf = p_out_buf + temp_out_len;
					*p_out_len += temp_out_len;

					/* logical_name */
					temp_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
						ATTR_TYPE_OCTET_STRING,											/* Type */
						p_table_cell_def_child->logical_name,							/* Buffer */
						6																/* Length */
					);
					p_out_buf = p_out_buf + temp_out_len;
					*p_out_len += temp_out_len;

					/* group_E_values */
					*p_out_buf++ = ATTR_TYPE_ARRAY;						/* Structure type */
					*p_out_buf++ = p_table_cell_def_child->nr_group_E;	/* Number of elements */
					*p_out_len   += 2;
					for (j = 0; j < p_table_cell_def_child->nr_group_E; j ++)
					{
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
							ATTR_TYPE_UNSIGNED,												/* Type */
							(Unsigned8 *)p_table_cell_def_child->p_group_E,				/* Buffer */
							1																/* Length */
						);
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;
					}

					/* attribute_index */
					temp_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
						ATTR_TYPE_INTEGER,												/* Type */
						(Unsigned8 *)&p_table_cell_def_child->attribute_index,			/* Buffer */
						1																/* Length */
					);
					p_out_buf = p_out_buf + temp_out_len;
					*p_out_len += temp_out_len;

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing */
				}
				break;

			case 4:	/* scaler_unit */

				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeScalerUnit(
						p_out_buf,							/* Output buffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,	/* Max length */
						&(p_child_record->scaler_unit));

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, not set the scaler, unit */
				}
				break;

			default:
				break;
		}
	}
	else if (req_type == REQ_TYPE_ACTION)
	{
		Unsigned8 b_block_transfer = 0;
		
		switch (p_descriptor->Attr_ID & 0x00FF)
		{
			case 1:	/* reset (data) */
			case 2:	/* capture (data) */
				/* User function take care for all */
				response_result = R_OBIS_DistributeByUserFunc(
					child_index,		/* child_index */	
					req_type,			/* req_type */
					service_type,		/* service_type */
					&b_block_transfer,	/* p_is_block_transfer */
					p_descriptor,		/* p_descriptor */	
					pdata,				/* pdata */			
					p_out_buf,			/* p_out_buf */		
					p_out_len			/* p_out_len */
				);
				/* Not support block transfer now, TODO in future */
				if (b_block_transfer != 0)
				{
					response_result = ACTION_RESULT_SCOPE_ACCESS_VIOLATED;
				}
				break;
			default:
				response_result = ACTION_RESULT_TYPE_UNMATCH;
				break;
		}
	}
	else
	{
		/* INTERNAL ERROR */
	}

	return (response_result);
}
#endif

/******************************************************************************
* Function Name : R_OBIS_DistributeStatusMappingClass
* Interface     : Unsigned8 R_OBIS_DistributeStatusMappingClass(
*               :     Unsigned16 master_index,
*               :     Unsigned16 child_index,
*               :     req_type_t req_type,
*               :     service_type_t service_type,
*               :     st_Cosem_Attr_Desc *p_descriptor,
*               :     Unsigned8	*pdata,
*               :     Unsigned8	*p_out_buf,
*               :     Unsigned16 *p_out_len
*               : );
* Description   : Distributor function for DRegister class
* Arguments     : Unsigned16 master_index -
*               :    [In] Id of the row in master list
*               : Unsigned16 child_index -
*               :    [In] Id of the row in child list
*               : req_type_t req_type -
*               :    [In] Request type (GET/SET/ACTION)
*               : service_type_t service_type -
*               :    [In] Service type (NORMAL/BLOCK/LIST)
*               : st_Cosem_Attr_Desc *p_descriptor -
*               :    [In] COSEM object descriptor
*               : Unsigned8 * pdata -
*               :    [In] Data in case of SET/ACTION
*               : Unsigned8 * p_out_buf -
*               :    [Out] Output buffer for encoded data
*               : Unsigned16 * p_out_len -
*               :    [Out] Output buffer length for encoded data
* Function Calls: TBD
* Return Value  : Unsigned8, distribution status
******************************************************************************/
#if (defined(USED_CLASS_63) && USED_CLASS_63 == 1)
Unsigned8 R_OBIS_DistributeStatusMappingClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
)
{
	/* Result of response, def. is not Matched */
	Unsigned8				i, j;
	Unsigned8				response_result = TYPE_UNMATCH;
	class63_child_record_t	*p_child_record;
	mapping_table_t			*p_mapping_table_child;
	Unsigned16				*p_temp;
	Unsigned16				temp_out_len;

	p_child_record = (class63_child_record_t *)(
		R_OBIS_FindChildRecordByIndex(master_index, child_index)
	);

	/* Classify the code based on Attribute or Method request */
	if (req_type == REQ_TYPE_GET || req_type == REQ_TYPE_SET)
	{
		switch (p_descriptor->Attr_ID)
		{
			case 0: /* attribute 0 */
				/* Not support */
				break;

			case 1: /* logical_name */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,							/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,	/* Max length of g_ServiceBuffer */
						ATTR_TYPE_OCTET_STRING,				/* Octet-string */
						p_descriptor->Instance_ID,			/* OBIS code from the request */
						6);									/* OBIS code is 6 chars */

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, OBIS code is read only */
				}
				break;

			case 2:	/* status_word */
				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				/* distribute the buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					&p_child_record->status_word.access,		/* Access method option */
					p_child_record->status_word.choice.type,	/* Type */
					p_child_record->status_word.choice.size,	/* Size */
					p_child_record->status_word.buffer,			/* Buffer of value */
					child_index,								/* Child ID */
					req_type,									/* Request type */
					service_type,								/* Service type */
					p_descriptor,								/* Pointer to COSEM descriptor */
					pdata,										/* Data from COSEM Appl layer */
					p_out_buf,									/* Data pointer for get */
					p_out_len									/* Data length pointer for get */
				);

				break;

			case 3:	/* mapping_table  */
				if (req_type == REQ_TYPE_GET)
				{
					response_result = DATA_ACCESS_RESULT_SUCCESS;
					/*
					 * mapping_table_t structure
					 */
					p_mapping_table_child = (mapping_table_t *)p_child_record->p_mapping_table;
					*p_out_buf++ = ATTR_TYPE_STRUCTURE;						/* Structure type */
					*p_out_buf++ = 2;										/* 2 elements */
					*p_out_len   = 2;
					/* ref_table_id */
					temp_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
						ATTR_TYPE_UNSIGNED,												/* Type */
						(Unsigned8 *)&p_mapping_table_child->ref_table_id,				/* Buffer */
						1																/* Length */
					);
					p_out_buf = p_out_buf + temp_out_len;
					*p_out_len += temp_out_len;

					/* table_entry */
					if (1 == p_mapping_table_child->nr_entry_list)
					{
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,													/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,							/* Max length of g_ServiceBuffer */
							ATTR_TYPE_LONG_UNSIGNED,									/* Type */
							(Unsigned8 *)p_mapping_table_child->p_entry_list,			/* Buffer */
							2															/* Length */
						);
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;
					}
					else if (p_mapping_table_child->nr_entry_list > 1)
					{
						*p_out_buf++ = ATTR_TYPE_ARRAY;									/* Structure type */
						*p_out_buf++ = p_mapping_table_child->nr_entry_list;			/* Number of elements */
						*p_out_len   += 2;

						p_temp = (Unsigned16 *) p_mapping_table_child->p_entry_list;
						for (i = 0; i < p_mapping_table_child->nr_entry_list; i++)
						{
							temp_out_len = R_OBIS_EncodeAttribute(
								p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
								OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
								ATTR_TYPE_LONG_UNSIGNED,										/* Type */
								(Unsigned8 *)(p_temp + i),										/* Buffer */
								2																/* Length */
							);
							p_out_buf = p_out_buf + temp_out_len;
							*p_out_len += temp_out_len;
						}
					}
					else
					{
						/* Fatal error, table_entry has no entry */
						response_result = TYPE_UNMATCH;
					}

				}
				else
				{
					/* Do nothing */
				}
				break;

			default:
				break;
		}
	}
	else if (req_type == REQ_TYPE_ACTION)
	{
		/* Not available */
	}
	else
	{
		/* INTERNAL ERROR */
	}

	return (response_result);
}
#endif


/******************************************************************************
* Function Name : R_OBIS_DistributeSecuritySetupClass
* Interface     : Unsigned8 R_OBIS_DistributeSecuritySetupClass(
*               :     Unsigned16 master_index,
*               :     Unsigned16 child_index,
*               :     req_type_t req_type,
*               :     service_type_t service_type,
*               :     st_Cosem_Attr_Desc *p_descriptor,
*               :     Unsigned8	*pdata,
*               :     Unsigned8	*p_out_buf,
*               :     Unsigned16 *p_out_len
*               : );
* Description   : Distributor function for  Security Setup class
* Arguments     : Unsigned16 master_index -
*               :    [In] Id of the row in master list
*               : Unsigned16 child_index -
*               :    [In] Id of the row in child list
*               : req_type_t req_type -
*               :    [In] Request type (GET/SET/ACTION)
*               : service_type_t service_type -
*               :    [In] Service type (NORMAL/BLOCK/LIST)
*               : st_Cosem_Attr_Desc *p_descriptor -
*               :    [In] COSEM object descriptor
*               : Unsigned8 * pdata -
*               :    [In] Data in case of SET/ACTION
*               : Unsigned8 * p_out_buf -
*               :    [Out] Output buffer for encoded data
*               : Unsigned16 * p_out_len -
*               :    [Out] Output buffer length for encoded data
* Function Calls: TBD
* Return Value  : Unsigned8, distribution status
******************************************************************************/
#if (defined(USED_CLASS_64) && USED_CLASS_64 == 1)
Unsigned8 R_OBIS_DistributeSecuritySetupClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
)
{
	/* Result of response, def. is not Matched */
	Unsigned8				response_result = TYPE_UNMATCH;
	Unsigned8				i;
	Unsigned16				temp_out_len;

	asslist_entry_t			*p_child_asslist_entry;
	class64_child_record_t	*p_child_record;

	p_child_record = (class64_child_record_t *)(
		R_OBIS_FindChildRecordByIndex(master_index, child_index)
	);

	/* Classify the code based on Attribute or Method request */
	if (req_type == REQ_TYPE_GET || req_type == REQ_TYPE_SET)
	{
		switch (p_descriptor->Attr_ID)
		{
			case 0: /* attribute 0 */
				/* Not support */
				break;

			case 1: /* logical_name */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,							/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,	/* Max length of g_ServiceBuffer */
						ATTR_TYPE_OCTET_STRING,				/* Octet-string */
						p_descriptor->Instance_ID,			/* OBIS code from the request */
						6);									/* OBIS code is 6 chars */

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, OBIS code is read only */
				}
				break;

			case 2:	/* security_policy */
				response_result = R_OBIS_DistributeAttrBuffer(
					NULL,											/* Access method option */
					ATTR_TYPE_ENUM,									/* Type */
					1,												/* Size */
					p_child_record->p_security_policy,				/* Buffer of value */
					child_index,									/* Child ID */
					req_type,										/* Request type */
					service_type,									/* Service type */
					p_descriptor,									/* Pointer to COSEM descriptor */
					pdata,											/* Data from COSEM Appl layer */
					p_out_buf,										/* Data pointer for get */
					p_out_len										/* Data length pointer for get */
				);
				break;

			case 3:	/* security_suite */
				response_result = R_OBIS_DistributeAttrBuffer(
					NULL,											/* Access method option */
					ATTR_TYPE_ENUM,									/* Type */
					1,												/* Size */
					p_child_record->p_security_suit,				/* Buffer of value */
					child_index,									/* Child ID */
					req_type,										/* Request type */
					service_type,									/* Service type */
					p_descriptor,									/* Pointer to COSEM descriptor */
					pdata,											/* Data from COSEM Appl layer */
					p_out_buf,										/* Data pointer for get */
					p_out_len										/* Data length pointer for get */
				);
				break;

			case 4:	/* client_system_title */
				response_result = R_OBIS_DistributeAttrBuffer(
					NULL,											/* Access method option */
					ATTR_TYPE_OCTET_STRING,							/* Type */
					strlen((void *)p_child_record->p_client_title),	/* Size */
					p_child_record->p_client_title,					/* Buffer of value */
					child_index,									/* Child ID */
					req_type,										/* Request type */
					service_type,									/* Service type */
					p_descriptor,									/* Pointer to COSEM descriptor */
					pdata,											/* Data from COSEM Appl layer */
					p_out_buf,										/* Data pointer for get */
					p_out_len										/* Data length pointer for get */
				);
			break;

			case 5:	/* server_system_title */
				response_result = R_OBIS_DistributeAttrBuffer(
					NULL,											/* Access method option */
					ATTR_TYPE_OCTET_STRING,							/* Type */
					strlen((void *)p_child_record->p_server_title),	/* Size */
					p_child_record->p_server_title,					/* Buffer of value */
					child_index,									/* Child ID */
					req_type,										/* Request type */
					service_type,									/* Service type */
					p_descriptor,									/* Pointer to COSEM descriptor */
					pdata,											/* Data from COSEM Appl layer */
					p_out_buf,										/* Data pointer for get */
					p_out_len										/* Data length pointer for get */
				);
			break;

			default:
				break;
		}
	}
	else if (req_type == REQ_TYPE_ACTION)
	{
		Unsigned8   b_block_transfer = 0;
		Unsigned8	b_have_client_data = (Unsigned8)(p_descriptor->Attr_ID >> 8);
		Unsigned16	method_id = (p_descriptor->Attr_ID & 0x00FF);
	
		switch (method_id)
		{
			case 1:	/* security_activate (data) */
			{
				/* (data) is 1 byte enum
				 *   (0) nothing,
				 *   (1) all messages to be authenticated,
				 *   (2) all messages to be encrypted,
				 *   (3) all messages to be authenticated and encrypted
				 */
				
				/* No data response */
				*p_out_len = 0;
				
				if (b_have_client_data &&
					p_child_record->p_security_policy != NULL &&
					*(pdata+0) == ATTR_TYPE_ENUM &&
					*(pdata+1) >= SECURITY_NONE &&
					*(pdata+1) <= SECURITY_AUTHENTICATED_ENCRYPTED)
				{
					*(p_child_record->p_security_policy) = *(pdata+1);
					response_result = ACTION_RESULT_SUCCESS;
				}
				else
				{
					response_result = ACTION_RESULT_RD_WRT_DENIED;
				}
			}
			break;
			
			case 2:	/* global_key_transfer (data) */
				/* User function take care for all */
				response_result = R_OBIS_DistributeByUserFunc(
					child_index,		/* child_index */	
					req_type,			/* req_type */
					service_type,		/* service_type */
					&b_block_transfer,	/* p_is_block_transfer */
					p_descriptor,		/* p_descriptor */	
					pdata,				/* pdata */			
					p_out_buf,			/* p_out_buf */		
					p_out_len			/* p_out_len */
				);
				/* Not support block transfer now, TODO in future */
				if (b_block_transfer != 0)
				{
					response_result = ACTION_RESULT_SCOPE_ACCESS_VIOLATED;
				}
				break;
			default:
				response_result = ACTION_RESULT_TYPE_UNMATCH;
				break;
		}
	}
	else
	{
		/* INTERNAL ERROR */
	}

	return (response_result);
}
#endif

/******************************************************************************
* Function Name : R_OBIS_DistributeDisconnectControlClass
* Interface     : Unsigned8 R_OBIS_DistributeDisconnectControlClass(
*               :     Unsigned16 master_index,
*               :     Unsigned16 child_index,
*               :     req_type_t req_type,
*               :     service_type_t service_type,
*               :     st_Cosem_Attr_Desc *p_descriptor,
*               :     Unsigned8	*pdata,
*               :     Unsigned8	*p_out_buf,
*               :     Unsigned16 *p_out_len
*               : );
* Description   : Distributor function for  Security Setup class
* Arguments     : Unsigned16 master_index -
*               :    [In] Id of the row in master list
*               : Unsigned16 child_index -
*               :    [In] Id of the row in child list
*               : req_type_t req_type -
*               :    [In] Request type (GET/SET/ACTION)
*               : service_type_t service_type -
*               :    [In] Service type (NORMAL/BLOCK/LIST)
*               : st_Cosem_Attr_Desc *p_descriptor -
*               :    [In] COSEM object descriptor
*               : Unsigned8 * pdata -
*               :    [In] Data in case of SET/ACTION
*               : Unsigned8 * p_out_buf -
*               :    [Out] Output buffer for encoded data
*               : Unsigned16 * p_out_len -
*               :    [Out] Output buffer length for encoded data
* Function Calls: TBD
* Return Value  : Unsigned8, distribution status
******************************************************************************/
#if (defined(USED_CLASS_70) && USED_CLASS_70 == 1)
Unsigned8 R_OBIS_DistributeDisconnectControlClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
)
{
	/* Result of response, def. is not Matched */
	Unsigned8				response_result = TYPE_UNMATCH;
	Unsigned8				i;
	Unsigned16				temp_out_len;
	class70_child_record_t	*p_child_record;

	p_child_record = (class70_child_record_t *)(
		R_OBIS_FindChildRecordByIndex(master_index, child_index)
	);

	/* Classify the code based on Attribute or Method request */
	if (req_type == REQ_TYPE_GET || req_type == REQ_TYPE_SET)
	{
		switch (p_descriptor->Attr_ID)
		{
			case 0: /* attribute 0 */
				/* Not support */
				break;

			case 1: /* logical_name */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,							/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,	/* Max length of g_ServiceBuffer */
						ATTR_TYPE_OCTET_STRING,				/* Octet-string */
						p_descriptor->Instance_ID,			/* OBIS code from the request */
						6);									/* OBIS code is 6 chars */

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, OBIS code is read only */
				}
				break;

			case 2:	/* output_state  */
				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				/* distribute the attr buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					&p_child_record->output_state.access,		/* Access method option */
					p_child_record->output_state.choice.type,	/* Type */
					p_child_record->output_state.choice.size,	/* Size */
					p_child_record->output_state.buffer,		/* Buffer of value */
					child_index,								/* Child ID */
					req_type,									/* Request type */
					service_type,								/* Service type */
					p_descriptor,								/* Pointer to COSEM descriptor */
					pdata,										/* Data from COSEM Appl layer */
					p_out_buf,									/* Data pointer for get */
					p_out_len									/* Data length pointer for get */
				);

				break;

			case 3:	/* control_state */
				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				/* distribute the attr buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					&p_child_record->control_state.access,		/* Access method option */
					p_child_record->control_state.choice.type,	/* Type */
					p_child_record->control_state.choice.size,	/* Size */
					p_child_record->control_state.buffer,		/* Buffer of value */
					child_index,								/* Child ID */
					req_type,									/* Request type */
					service_type,								/* Service type */
					p_descriptor,								/* Pointer to COSEM descriptor */
					pdata,										/* Data from COSEM Appl layer */
					p_out_buf,									/* Data pointer for get */
					p_out_len									/* Data length pointer for get */
				);

				break;

			case 4:	/* control_mode */
				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				/* distribute the attr buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					NULL,								/* Access method option */
					ATTR_TYPE_ENUM,						/* Type */
					1,									/* Size */
					p_child_record->p_control_mode,		/* Buffer of value */
					child_index,						/* Child ID */
					req_type,							/* Request type */
					service_type,						/* Service type */
					p_descriptor,						/* Pointer to COSEM descriptor */
					pdata,								/* Data from COSEM Appl layer */
					p_out_buf,							/* Data pointer for get */
					p_out_len							/* Data length pointer for get */
				);

				break;

			default:
				break;
		}
	}
	else if (req_type == REQ_TYPE_ACTION)
	{
		Unsigned8 b_block_transfer = 0;
		
		switch (p_descriptor->Attr_ID & 0x00FF)
		{
			case 1:	/* remote_disconnect (data) */
			case 2:	/* remote_reconnect (data) */
				/* User function take care for all */
				response_result = R_OBIS_DistributeByUserFunc(
					child_index,		/* child_index */	
					req_type,			/* req_type */
					service_type,		/* service_type */
					&b_block_transfer,	/* p_is_block_transfer */
					p_descriptor,		/* p_descriptor */	
					pdata,				/* pdata */			
					p_out_buf,			/* p_out_buf */		
					p_out_len			/* p_out_len */
				);
				/* Not support block transfer now, TODO in future */
				if (b_block_transfer != 0)
				{
					response_result = ACTION_RESULT_SCOPE_ACCESS_VIOLATED;
				}
				break;
			default:
				response_result = ACTION_RESULT_TYPE_UNMATCH;
				break;
		}
	}
	else
	{
		/* INTERNAL ERROR */
	}

	return (response_result);
}
#endif

/******************************************************************************
* Function Name : R_OBIS_DistributeLimiterClass
* Interface     : Unsigned8 R_OBIS_DistributeLimiterClass(
*               :     Unsigned16 master_index,
*               :     Unsigned16 child_index,
*               :     req_type_t req_type,
*               :     service_type_t service_type,
*               :     st_Cosem_Attr_Desc *p_descriptor,
*               :     Unsigned8	*pdata,
*               :     Unsigned8	*p_out_buf,
*               :     Unsigned16 *p_out_len
*               : );
* Description   : Distributor function for  Security Setup class
* Arguments     : Unsigned16 master_index -
*               :    [In] Id of the row in master list
*               : Unsigned16 child_index -
*               :    [In] Id of the row in child list
*               : req_type_t req_type -
*               :    [In] Request type (GET/SET/ACTION)
*               : service_type_t service_type -
*               :    [In] Service type (NORMAL/BLOCK/LIST)
*               : st_Cosem_Attr_Desc *p_descriptor -
*               :    [In] COSEM object descriptor
*               : Unsigned8 * pdata -
*               :    [In] Data in case of SET/ACTION
*               : Unsigned8 * p_out_buf -
*               :    [Out] Output buffer for encoded data
*               : Unsigned16 * p_out_len -
*               :    [Out] Output buffer length for encoded data
* Function Calls: TBD
* Return Value  : Unsigned8, distribution status
******************************************************************************/
#if (defined(USED_CLASS_71) && USED_CLASS_71 == 1)
Unsigned8 R_OBIS_DistributeLimiterClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
)
{
	/* Result of response, def. is not Matched */
	Unsigned8				response_result = TYPE_UNMATCH;
	Unsigned8				i;
	Unsigned16				temp_out_len;
	Unsigned16				*p_temp_child;
	class71_child_record_t	*p_child_record;
	value_def_t				*p_value_def_child;
	eme_profile_t			*p_eme_profile_child;
	action_set_t			*p_child_action_set;

	p_child_record = (class71_child_record_t *)(
		R_OBIS_FindChildRecordByIndex(master_index, child_index)
	);

	/* Classify the code based on Attribute or Method request */
	if (req_type == REQ_TYPE_GET || req_type == REQ_TYPE_SET)
	{
		switch (p_descriptor->Attr_ID)
		{
			case 0: /* attribute 0 */
				/* Not support */
				break;

			case 1: /* logical_name */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,							/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,	/* Max length of g_ServiceBuffer */
						ATTR_TYPE_OCTET_STRING,				/* Octet-string */
						p_descriptor->Instance_ID,			/* OBIS code from the request */
						6);									/* OBIS code is 6 chars */

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, OBIS code is read only */
				}
				break;

			case 2:	/* monitored_value  */
				if (req_type == REQ_TYPE_GET)
				{
					/*
					 * value_def_t structure
					 */
					p_value_def_child = (value_def_t *) p_child_record->p_monitored_value;
					*p_out_buf++ = ATTR_TYPE_STRUCTURE;						/* Structure type */
					*p_out_buf++ = 3;										/* 3 elements */
					*p_out_len   = 2;
					/* class_id */
					temp_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
						ATTR_TYPE_LONG_UNSIGNED,										/* Type */
						(Unsigned8 *)&p_value_def_child->class_id,						/* Buffer */
						sizeof(Unsigned16)												/* Length */
					);
					p_out_buf = p_out_buf + temp_out_len;
					*p_out_len += temp_out_len;

					/* logical_name */
					temp_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
						ATTR_TYPE_OCTET_STRING,											/* Type */
						p_value_def_child->logical_name,								/* Buffer */
						6																/* Length */
					);
					p_out_buf = p_out_buf + temp_out_len;
					*p_out_len += temp_out_len;

					/* attribute_index */
					temp_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
						ATTR_TYPE_INTEGER,												/* Type */
						(Unsigned8 *)&p_value_def_child->attribute_index,				/* Buffer */
						1																/* Length */
					);
					p_out_buf = p_out_buf + temp_out_len;
					*p_out_len += temp_out_len;

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing */
				}
				break;

			case 3:	/* threshold_active */
				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				/* distribute the attr buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					&p_child_record->threshold_active.access,		/* Access method option */
					p_child_record->threshold_active.choice.type,	/* Type */
					p_child_record->threshold_active.choice.size,	/* Size */
					p_child_record->threshold_active.buffer,		/* Buffer of value */
					child_index,									/* Child ID */
					req_type,										/* Request type */
					service_type,									/* Service type */
					p_descriptor,									/* Pointer to COSEM descriptor */
					pdata,											/* Data from COSEM Appl layer */
					p_out_buf,										/* Data pointer for get */
					p_out_len										/* Data length pointer for get */
				);

				break;

			case 4:	/* threshold_normal */
				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				/* distribute the attr buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					NULL,												/* Access method option */
					p_child_record->threshold_normal.choice.type,		/* Type */
					p_child_record->threshold_normal.choice.size,		/* Size */
					p_child_record->threshold_normal.buffer,			/* Buffer of value */
					child_index,										/* Child ID */
					req_type,											/* Request type */
					service_type,										/* Service type */
					p_descriptor,										/* Pointer to COSEM descriptor */
					pdata,												/* Data from COSEM Appl layer */
					p_out_buf,											/* Data pointer for get */
					p_out_len											/* Data length pointer for get */
				);

				break;

			case 5:	/* threshold_emergency */
				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				/* distribute the attr buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					NULL,												/* Access method option */
					p_child_record->threshold_emergency.choice.type,	/* Type */
					p_child_record->threshold_emergency.choice.size,	/* Size */
					p_child_record->threshold_emergency.buffer,			/* Buffer of value */
					child_index,										/* Child ID */
					req_type,											/* Request type */
					service_type,										/* Service type */
					p_descriptor,										/* Pointer to COSEM descriptor */
					pdata,												/* Data from COSEM Appl layer */
					p_out_buf,											/* Data pointer for get */
					p_out_len											/* Data length pointer for get */
				);

				break;

			case 6:	/* min_over_threshold_duration */
				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				/* distribute the attr buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					NULL,											/* Access method option */
					ATTR_TYPE_DOUBLE_LONG_UNSIGNED,					/* Type */
					sizeof(Unsigned32),								/* Size */
					p_child_record->p_min_over_threshold_duration,	/* Buffer of value */
					child_index,									/* Child ID */
					req_type,										/* Request type */
					service_type,									/* Service type */
					p_descriptor,									/* Pointer to COSEM descriptor */
					pdata,											/* Data from COSEM Appl layer */
					p_out_buf,										/* Data pointer for get */
					p_out_len										/* Data length pointer for get */
				);

				break;

			case 7:	/* min_under_threshold_duration */
				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				/* distribute the attr buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					NULL,											/* Access method option */
					ATTR_TYPE_DOUBLE_LONG_UNSIGNED,					/* Type */
					sizeof(Unsigned32),								/* Size */
					p_child_record->p_min_under_threshold_duration,	/* Buffer of value */
					child_index,									/* Child ID */
					req_type,										/* Request type */
					service_type,									/* Service type */
					p_descriptor,									/* Pointer to COSEM descriptor */
					pdata,											/* Data from COSEM Appl layer */
					p_out_buf,										/* Data pointer for get */
					p_out_len										/* Data length pointer for get */
				);

				break;

			case 8:	/* emergency_profile */
				if (req_type == REQ_TYPE_GET)
				{
					response_result = DATA_ACCESS_RESULT_SUCCESS;
					/*
					 * eme_profile_t structure
					 */
					p_eme_profile_child = (eme_profile_t *)&p_child_record->p_eme_profile;
					*p_out_buf++ = ATTR_TYPE_STRUCTURE;						/* Structure type */
					*p_out_buf++ = 3;										/* 3 elements */
					*p_out_len   = 2;
					/* emergency_profile_id */
					temp_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
						ATTR_TYPE_LONG_UNSIGNED,										/* Type */
						(Unsigned8 *)&p_eme_profile_child->eme_profile_id,				/* Buffer */
						2																/* Length */
					);
					p_out_buf = p_out_buf + temp_out_len;
					*p_out_len += temp_out_len;

					/* emergency_activation_time */
					temp_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
						ATTR_TYPE_OCTET_STRING,											/* Type */
						(Unsigned8 *)&p_eme_profile_child->p_eme_act_time,				/* Buffer */
						sizeof(date_time_t)												/* Length */
					);
					p_out_buf = p_out_buf + temp_out_len;
					*p_out_len += temp_out_len;

					/* emergency_duration */
					temp_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
						ATTR_TYPE_DOUBLE_LONG_UNSIGNED,									/* Type */
						(Unsigned8 *)&p_eme_profile_child->eme_duration,				/* Buffer */
						4																/* Length */
					);
					p_out_buf = p_out_buf + temp_out_len;
					*p_out_len += temp_out_len;
				}
				else
				{
					/* Do nothing */
				}
				break;

			case 9:	/* emergency_profile_group_id_list */
				if (req_type == REQ_TYPE_GET)
				{
					response_result = DATA_ACCESS_RESULT_SUCCESS;
					/*
					 * emergency_profile_group_id array
					 */
					p_temp_child = (Unsigned16 *)p_child_record->p_eme_profile_group_id_list;
					*p_out_buf++ = ATTR_TYPE_ARRAY;									/* Array type */
					*p_out_buf++ = *p_child_record->p_nr_eme_profile_group_id_list;	/* Number of elements */
					*p_out_len   = 2;
					for (i = 0; i < *p_child_record->p_nr_eme_profile_group_id_list; i ++)
					{
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
							ATTR_TYPE_LONG_UNSIGNED,										/* Type */
							(Unsigned8 *)(p_temp_child + i),								/* Buffer */
							2																/* Length */
						);
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;
					}
				}
				else
				{
					/* Do nothing */
				}
				break;

			case 10:	/* emergency_profile_active */
				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				/* distribute the attr buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					NULL,												/* Access method option */
					p_child_record->eme_profile_active.choice.type,		/* Type */
					p_child_record->eme_profile_active.choice.size,		/* Size */
					p_child_record->eme_profile_active.buffer,			/* Buffer of value */
					child_index,										/* Child ID */
					req_type,											/* Request type */
					service_type,										/* Service type */
					p_descriptor,										/* Pointer to COSEM descriptor */
					pdata,												/* Data from COSEM Appl layer */
					p_out_buf,											/* Data pointer for get */
					p_out_len											/* Data length pointer for get */
				);

				break;

			case 11:	/* actions */
				if (req_type == REQ_TYPE_GET)
				{
					p_child_action_set = (action_set_t *)p_child_record->p_action;
					*p_out_buf++ = ATTR_TYPE_STRUCTURE;						/* Structure type */
					*p_out_buf++ = 2;										/* 2 elements */
					*p_out_len   = 2;

					/* action up */
					*p_out_buf++ = ATTR_TYPE_STRUCTURE;						/* Structure type */
					*p_out_buf++ = 2;										/* 2 elements */
					*p_out_len   += 2;

					/* script_logical_name */
					*p_out_buf++ =  ATTR_TYPE_OCTET_STRING;
					*p_out_buf++ =  6;									/* OBIS code is 6 chars */
					*p_out_buf++ =  (Unsigned8)p_child_action_set->action_up->script_logical_name[0];
					*p_out_buf++ =  (Unsigned8)p_child_action_set->action_up->script_logical_name[1];
					*p_out_buf++ =  (Unsigned8)p_child_action_set->action_up->script_logical_name[2];
					*p_out_buf++ =  (Unsigned8)p_child_action_set->action_up->script_logical_name[3];
					*p_out_buf++ =  (Unsigned8)p_child_action_set->action_up->script_logical_name[4];
					*p_out_buf++ =  (Unsigned8)p_child_action_set->action_up->script_logical_name[5];
					*p_out_len   += 8;
					/* Script ID */
					*p_out_buf++ = ATTR_TYPE_LONG_UNSIGNED;
					*p_out_buf++ = (Unsigned8)(p_child_action_set->action_up->script_selector >> 8);
					*p_out_buf++ = (Unsigned8)(p_child_action_set->action_up->script_selector  & 0xFF);
					*p_out_len   += 3;

					/* action down */
					*p_out_buf++ = ATTR_TYPE_STRUCTURE;					/* Structure type */
					*p_out_buf++ = 2;									/* 2 elements */
					*p_out_len   += 2;

					/* script_logical_name */
					*p_out_buf++ =  ATTR_TYPE_OCTET_STRING;
					*p_out_buf++ =  6;									/* OBIS code is 6 chars */
					*p_out_buf++ =  (Unsigned8)p_child_action_set->action_down->script_logical_name[0];
					*p_out_buf++ =  (Unsigned8)p_child_action_set->action_down->script_logical_name[1];
					*p_out_buf++ =  (Unsigned8)p_child_action_set->action_down->script_logical_name[2];
					*p_out_buf++ =  (Unsigned8)p_child_action_set->action_down->script_logical_name[3];
					*p_out_buf++ =  (Unsigned8)p_child_action_set->action_down->script_logical_name[4];
					*p_out_buf++ =  (Unsigned8)p_child_action_set->action_down->script_logical_name[5];
					*p_out_len   += 8;
					/* Script ID */
					*p_out_buf++ = ATTR_TYPE_LONG_UNSIGNED;
					*p_out_buf++ = (Unsigned8)(p_child_action_set->action_down->script_selector >> 8);
					*p_out_buf++ = (Unsigned8)(p_child_action_set->action_down->script_selector  & 0xFF);
					*p_out_len   += 3;

					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing */
				}
				break;

			default:
				break;
		}
	}
	else if (req_type == REQ_TYPE_ACTION)
	{
		/* Not available */
	}
	else
	{
		/* INTERNAL ERROR */
	}

	return (response_result);
}
#endif

/******************************************************************************
* Function Name : R_OBIS_DistributeZigBeeSASStartupClass
* Interface     : Unsigned8 R_OBIS_DistributeZigBeeSASStartupClass(
*               :     Unsigned16 master_index,
*               :     Unsigned16 child_index,
*               :     req_type_t req_type,
*               :     service_type_t service_type,
*               :     st_Cosem_Attr_Desc *p_descriptor,
*               :     Unsigned8	*pdata,
*               :     Unsigned8	*p_out_buf,
*               :     Unsigned16 *p_out_len
*               : );
* Description   : Distributor function for  Security Setup class
* Arguments     : Unsigned16 master_index -
*               :    [In] Id of the row in master list
*               : Unsigned16 child_index -
*               :    [In] Id of the row in child list
*               : req_type_t req_type -
*               :    [In] Request type (GET/SET/ACTION)
*               : service_type_t service_type -
*               :    [In] Service type (NORMAL/BLOCK/LIST)
*               : st_Cosem_Attr_Desc *p_descriptor -
*               :    [In] COSEM object descriptor
*               : Unsigned8 * pdata -
*               :    [In] Data in case of SET/ACTION
*               : Unsigned8 * p_out_buf -
*               :    [Out] Output buffer for encoded data
*               : Unsigned16 * p_out_len -
*               :    [Out] Output buffer length for encoded data
* Function Calls: TBD
* Return Value  : Unsigned8, distribution status
******************************************************************************/
#if (defined(USED_CLASS_101) && USED_CLASS_101 == 1)
Unsigned8 R_OBIS_DistributeZigBeeSASStartupClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
)
{
	/* Result of response, def. is not Matched */
	Unsigned8				response_result = TYPE_UNMATCH;
	Unsigned8				i;
	Unsigned16				temp_out_len;
	Unsigned16				*p_temp_child;
	class101_child_record_t	*p_child_record;
	
	p_child_record = (class101_child_record_t *)(
		R_OBIS_FindChildRecordByIndex(master_index, child_index)
	);

	/* Classify the code based on Attribute or Method request */
	if (req_type == REQ_TYPE_GET || req_type == REQ_TYPE_SET)
	{
		switch (p_descriptor->Attr_ID)
		{
			case 0: /* attribute 0 */
				/* Not support */
				break;

			case 1: /* logical_name */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,							/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,	/* Max length of g_ServiceBuffer */
						ATTR_TYPE_OCTET_STRING,				/* Octet-string */
						p_descriptor->Instance_ID,			/* OBIS code from the request */
						6);									/* OBIS code is 6 chars */

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, OBIS code is read only */
				}
				break;

			case 2:	/* Short address  */
				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}
				
				/* distribute the attr buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					NULL,											/* Access method option */
					ATTR_TYPE_LONG_UNSIGNED,						/* Type */
					sizeof(Unsigned16),								/* Size */
					p_child_record->p_short_address,				/* Buffer of value */
					child_index,									/* Child ID */
					req_type,										/* Request type */
					service_type,									/* Service type */
					p_descriptor,									/* Pointer to COSEM descriptor */
					pdata,											/* Data from COSEM Appl layer */
					p_out_buf,										/* Data pointer for get */
					p_out_len										/* Data length pointer for get */
				);
				break;

			case 3:	/* Extended pan ID */
				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				/* distribute the attr buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					NULL,											/* Access method option */
					ATTR_TYPE_OCTET_STRING,							/* Type */
					8,												/* Size */
					p_child_record->p_extended_pan_id,				/* Buffer of value */
					child_index,									/* Child ID */
					req_type,										/* Request type */
					service_type,									/* Service type */
					p_descriptor,									/* Pointer to COSEM descriptor */
					pdata,											/* Data from COSEM Appl layer */
					p_out_buf,										/* Data pointer for get */
					p_out_len										/* Data length pointer for get */
				);

				break;

			case 4:	/* pan_ID */
				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				/* distribute the attr buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					NULL,												/* Access method option */
					ATTR_TYPE_LONG_UNSIGNED,							/* Type */
					sizeof(Unsigned16),									/* Size */
					p_child_record->p_pan_id,							/* Buffer of value */
					child_index,										/* Child ID */
					req_type,											/* Request type */
					service_type,										/* Service type */
					p_descriptor,										/* Pointer to COSEM descriptor */
					pdata,												/* Data from COSEM Appl layer */
					p_out_buf,											/* Data pointer for get */
					p_out_len											/* Data length pointer for get */
				);

				break;

			case 5:	/* Channel mask */
				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				/* distribute the attr buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					NULL,												/* Access method option */
					ATTR_TYPE_DOUBLE_LONG_UNSIGNED,						/* Type */
					sizeof(Unsigned32),									/* Size */
					p_child_record->p_channel_mask,						/* Buffer of value */
					child_index,										/* Child ID */
					req_type,											/* Request type */
					service_type,										/* Service type */
					p_descriptor,										/* Pointer to COSEM descriptor */
					pdata,												/* Data from COSEM Appl layer */
					p_out_buf,											/* Data pointer for get */
					p_out_len											/* Data length pointer for get */
				);

				break;			
				
				
			case 6:	/* Protocol version */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
						ATTR_TYPE_UNSIGNED,												/* Type */
						(Unsigned8 *)p_child_record->p_protocol_version,				/* Buffer */
						1																/* Length */
					);
					
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing */
				}
				
				break;

			case 7:	/* Stack profile */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
						ATTR_TYPE_ENUM,													/* Type */
						(Unsigned8 *)p_child_record->p_stack_profile,					/* Buffer */
						1																/* Length */
					);
					
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing */
				}
				
				break;

			case 8:	/* Start-up control */
				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				/* distribute the attr buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					NULL,											/* Access method option */
					ATTR_TYPE_UNSIGNED,								/* Type */
					sizeof(Unsigned8),								/* Size */
					p_child_record->p_start_up_control,				/* Buffer of value */
					child_index,									/* Child ID */
					req_type,										/* Request type */
					service_type,									/* Service type */
					p_descriptor,									/* Pointer to COSEM descriptor */
					pdata,											/* Data from COSEM Appl layer */
					p_out_buf,										/* Data pointer for get */
					p_out_len										/* Data length pointer for get */
				);

				break;

			case 9:	/* Trust centre address */
				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				/* distribute the attr buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					NULL,											/* Access method option */
					ATTR_TYPE_OCTET_STRING,							/* Type */
					8,												/* Size */
					p_child_record->p_trust_centre_address,			/* Buffer of value */
					child_index,									/* Child ID */
					req_type,										/* Request type */
					service_type,									/* Service type */
					p_descriptor,									/* Pointer to COSEM descriptor */
					pdata,											/* Data from COSEM Appl layer */
					p_out_buf,										/* Data pointer for get */
					p_out_len										/* Data length pointer for get */
				);

				break;

			case 10:	/* Link key */
				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				/* distribute the attr buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					NULL,											/* Access method option */
					ATTR_TYPE_OCTET_STRING,							/* Type */
					16,												/* Size */
					p_child_record->p_link_key,						/* Buffer of value */
					child_index,									/* Child ID */
					req_type,										/* Request type */
					service_type,									/* Service type */
					p_descriptor,									/* Pointer to COSEM descriptor */
					pdata,											/* Data from COSEM Appl layer */
					p_out_buf,										/* Data pointer for get */
					p_out_len										/* Data length pointer for get */
				);

				break;
				
			case 11:	/* Network key */
				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				/* distribute the attr buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					NULL,											/* Access method option */
					ATTR_TYPE_OCTET_STRING,							/* Type */
					16,												/* Size */
					p_child_record->p_network_key,						/* Buffer of value */
					child_index,									/* Child ID */
					req_type,										/* Request type */
					service_type,									/* Service type */
					p_descriptor,									/* Pointer to COSEM descriptor */
					pdata,											/* Data from COSEM Appl layer */
					p_out_buf,										/* Data pointer for get */
					p_out_len										/* Data length pointer for get */
				);

				break;
				
			case 12: /* User insecure join */
				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				/* distribute the attr buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					NULL,											/* Access method option */
					ATTR_TYPE_BOOLEAN,								/* Type */
					1,												/* Size */
					p_child_record->p_use_insecure_join,			/* Buffer of value */
					child_index,									/* Child ID */
					req_type,										/* Request type */
					service_type,									/* Service type */
					p_descriptor,									/* Pointer to COSEM descriptor */
					pdata,											/* Data from COSEM Appl layer */
					p_out_buf,										/* Data pointer for get */
					p_out_len										/* Data length pointer for get */
				);

				break;
				
			default:
				break;
		}
	}
	else if (req_type == REQ_TYPE_ACTION)
	{
		/* Not available */
	}
	else
	{
		/* INTERNAL ERROR */
	}

	return (response_result);
}
#endif

/******************************************************************************
* Function Name : R_OBIS_DistributeZigBeeSASJoinClass
* Interface     : Unsigned8 R_OBIS_DistributeZigBeeSASJoinClass(
*               :     Unsigned16 master_index,
*               :     Unsigned16 child_index,
*               :     req_type_t req_type,
*               :     service_type_t service_type,
*               :     st_Cosem_Attr_Desc *p_descriptor,
*               :     Unsigned8	*pdata,
*               :     Unsigned8	*p_out_buf,
*               :     Unsigned16 *p_out_len
*               : );
* Description   : Distributor function for  Security Setup class
* Arguments     : Unsigned16 master_index -
*               :    [In] Id of the row in master list
*               : Unsigned16 child_index -
*               :    [In] Id of the row in child list
*               : req_type_t req_type -
*               :    [In] Request type (GET/SET/ACTION)
*               : service_type_t service_type -
*               :    [In] Service type (NORMAL/BLOCK/LIST)
*               : st_Cosem_Attr_Desc *p_descriptor -
*               :    [In] COSEM object descriptor
*               : Unsigned8 * pdata -
*               :    [In] Data in case of SET/ACTION
*               : Unsigned8 * p_out_buf -
*               :    [Out] Output buffer for encoded data
*               : Unsigned16 * p_out_len -
*               :    [Out] Output buffer length for encoded data
* Function Calls: TBD
* Return Value  : Unsigned8, distribution status
******************************************************************************/
#if (defined(USED_CLASS_102) && USED_CLASS_102 == 1)
Unsigned8 R_OBIS_DistributeZigBeeSASJoinClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
)
{
	/* Result of response, def. is not Matched */
	Unsigned8				response_result = TYPE_UNMATCH;
	class102_child_record_t	*p_child_record;
	
	p_child_record = (class102_child_record_t *)(
		R_OBIS_FindChildRecordByIndex(master_index, child_index)
	);

	/* Classify the code based on Attribute or Method request */
	if (req_type == REQ_TYPE_GET || req_type == REQ_TYPE_SET)
	{
		switch (p_descriptor->Attr_ID)
		{
			case 0: /* attribute 0 */
				/* Not support */
				break;

			case 1: /* logical_name */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,							/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,	/* Max length of g_ServiceBuffer */
						ATTR_TYPE_OCTET_STRING,				/* Octet-string */
						p_descriptor->Instance_ID,			/* OBIS code from the request */
						6);									/* OBIS code is 6 chars */

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, OBIS code is read only */
				}
				break;

			case 2:	/* Scan attempts  */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
						ATTR_TYPE_UNSIGNED,												/* Type */
						(Unsigned8 *)p_child_record->p_scan_attempts,					/* Buffer */
						1																/* Length */
					);

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing */
				}
				break;

			case 3:	/* Time between scans */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
						ATTR_TYPE_LONG_UNSIGNED,										/* Type */
						(Unsigned8 *)p_child_record->p_time_between_scans,				/* Buffer */
						2																/* Length */
					);

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing */
				}
				break;

			case 4:	/* Rejoin interval */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
						ATTR_TYPE_LONG_UNSIGNED,										/* Type */
						(Unsigned8 *)p_child_record->p_rejoin_interval,					/* Buffer */
						2																/* Length */
					);

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing */
				}
				break;

			case 5:	/* Rejoin retry interval */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
						ATTR_TYPE_LONG_UNSIGNED,										/* Type */
						(Unsigned8 *)p_child_record->p_rejoin_retry_interval,					/* Buffer */
						2																/* Length */
					);

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing */
				}
				break;

			default:
				break;
		}
	}
	else if (req_type == REQ_TYPE_ACTION)
	{
		/* Not available */
	}
	else
	{
		/* INTERNAL ERROR */
	}

	return (response_result);
}
#endif

/******************************************************************************
* Function Name : R_OBIS_DistributeZigBeeSASAPSFragmentClass
* Interface     : Unsigned8 R_OBIS_DistributeZigBeeSASAPSFragmentClass(
*               :     Unsigned16 master_index,
*               :     Unsigned16 child_index,
*               :     req_type_t req_type,
*               :     service_type_t service_type,
*               :     st_Cosem_Attr_Desc *p_descriptor,
*               :     Unsigned8	*pdata,
*               :     Unsigned8	*p_out_buf,
*               :     Unsigned16 *p_out_len
*               : );
* Description   : Distributor function for  Security Setup class
* Arguments     : Unsigned16 master_index -
*               :    [In] Id of the row in master list
*               : Unsigned16 child_index -
*               :    [In] Id of the row in child list
*               : req_type_t req_type -
*               :    [In] Request type (GET/SET/ACTION)
*               : service_type_t service_type -
*               :    [In] Service type (NORMAL/BLOCK/LIST)
*               : st_Cosem_Attr_Desc *p_descriptor -
*               :    [In] COSEM object descriptor
*               : Unsigned8 * pdata -
*               :    [In] Data in case of SET/ACTION
*               : Unsigned8 * p_out_buf -
*               :    [Out] Output buffer for encoded data
*               : Unsigned16 * p_out_len -
*               :    [Out] Output buffer length for encoded data
* Function Calls: TBD
* Return Value  : Unsigned8, distribution status
******************************************************************************/
#if (defined(USED_CLASS_103) && USED_CLASS_103 == 1)
Unsigned8 R_OBIS_DistributeZigBeeSASAPSFragmentClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
)
{
	/* Result of response, def. is not Matched */
	Unsigned8				response_result = TYPE_UNMATCH;
	class103_child_record_t	*p_child_record;

	p_child_record = (class103_child_record_t *)(
		R_OBIS_FindChildRecordByIndex(master_index, child_index)
	);

	/* Classify the code based on Attribute or Method request */
	if (req_type == REQ_TYPE_GET || req_type == REQ_TYPE_SET)
	{
		switch (p_descriptor->Attr_ID)
		{
			case 0: /* attribute 0 */
				/* Not support */
				break;

			case 1: /* logical_name */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,							/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,	/* Max length of g_ServiceBuffer */
						ATTR_TYPE_OCTET_STRING,				/* Octet-string */
						p_descriptor->Instance_ID,			/* OBIS code from the request */
						6);									/* OBIS code is 6 chars */

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, OBIS code is read only */
				}
				break;

			case 2:	/* APS Interframe delay  */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
						ATTR_TYPE_LONG_UNSIGNED,										/* Type */
						(Unsigned8 *)p_child_record->p_aps_interframe_delay,			/* Buffer */
						2																/* Length */
					);

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing */
				}
				break;

			case 3:	/* APS Max window size */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
						ATTR_TYPE_LONG_UNSIGNED,										/* Type */
						(Unsigned8 *)p_child_record->p_aps_max_window_size,				/* Buffer */
						2																/* Length */
					);

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing */
				}
				break;

			default:
				break;
		}
	}
	else if (req_type == REQ_TYPE_ACTION)
	{
		/* Not available */
	}
	else
	{
		/* INTERNAL ERROR */
	}

	return (response_result);
}
#endif

/******************************************************************************
* Function Name : R_OBIS_DistributeZigBeeNetworkControlClass
* Interface     : Unsigned8 R_OBIS_DistributeZigBeeNetworkControlClass(
*               :     Unsigned16 master_index,
*               :     Unsigned16 child_index,
*               :     req_type_t req_type,
*               :     service_type_t service_type,
*               :     st_Cosem_Attr_Desc *p_descriptor,
*               :     Unsigned8	*pdata,
*               :     Unsigned8	*p_out_buf,
*               :     Unsigned16 *p_out_len
*               : );
* Description   : Distributor function for  Security Setup class
* Arguments     : Unsigned16 master_index -
*               :    [In] Id of the row in master list
*               : Unsigned16 child_index -
*               :    [In] Id of the row in child list
*               : req_type_t req_type -
*               :    [In] Request type (GET/SET/ACTION)
*               : service_type_t service_type -
*               :    [In] Service type (NORMAL/BLOCK/LIST)
*               : st_Cosem_Attr_Desc *p_descriptor -
*               :    [In] COSEM object descriptor
*               : Unsigned8 * pdata -
*               :    [In] Data in case of SET/ACTION
*               : Unsigned8 * p_out_buf -
*               :    [Out] Output buffer for encoded data
*               : Unsigned16 * p_out_len -
*               :    [Out] Output buffer length for encoded data
* Function Calls: TBD
* Return Value  : Unsigned8, distribution status
******************************************************************************/
#if (defined(USED_CLASS_104) && USED_CLASS_104 == 1)
Unsigned8 R_OBIS_DistributeZigBeeNetworkControlClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
)
{
	/* Result of response, def. is not Matched */
	Unsigned8				response_result = TYPE_UNMATCH;
	Unsigned16				i;
	Unsigned16				one_active_device_length = 0;
	Unsigned16				pre_encode_buf_length = 0;
	
	Unsigned16				temp_out_len;
	Unsigned16				*p_temp_child;
	class104_child_record_t	*p_child_record;
	
	p_child_record = (class104_child_record_t *)(
		R_OBIS_FindChildRecordByIndex(master_index, child_index)
	);

	/* Classify the code based on Attribute or Method request */
	if (req_type == REQ_TYPE_GET || req_type == REQ_TYPE_SET)
	{
		switch (p_descriptor->Attr_ID)
		{
			case 0: /* attribute 0 */
				/* Not support */
				break;

			case 1: /* logical_name */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,							/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,	/* Max length of g_ServiceBuffer */
						ATTR_TYPE_OCTET_STRING,				/* Octet-string */
						p_descriptor->Instance_ID,			/* OBIS code from the request */
						6);									/* OBIS code is 6 chars */

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, OBIS code is read only */
				}
				break;

			case 2:	/* Enable disable joining  */
				/* internal check */
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}

				/* distribute the attr buffer */
				response_result = R_OBIS_DistributeAttrBuffer(
					NULL,											/* Access method option */
					ATTR_TYPE_BOOLEAN,								/* Type */
					1,												/* Size */
					p_child_record->p_enable_disable_joining,		/* Buffer of value */
					child_index,									/* Child ID */
					req_type,										/* Request type */
					service_type,									/* Service type */
					p_descriptor,									/* Pointer to COSEM descriptor */
					pdata,											/* Data from COSEM Appl layer */
					p_out_buf,										/* Data pointer for get */
					p_out_len										/* Data length pointer for get */
				);

				break;

			case 3:	/* Join timeout */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
						ATTR_TYPE_LONG_UNSIGNED,										/* Type */
						(Unsigned8 *)p_child_record->p_join_timeout,					/* Buffer */
						2																/* Length */
					);

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing */
				}
				break;

			case 4:	/* Active devices */
				/* internal check */
				
				if (p_child_record == NULL)
				{
					break;	/* Fatal error, stop here */
				}
				
				/* distribute the attr buffer */
				if (req_type == REQ_TYPE_GET)
				{
					
					*p_out_len = 0;				/* Reset the encoded length */
					if (service_type == SERVICE_TYPE_NORMAL || 
						g_OBIS_CurrentActDev == 0)
					{
						g_OBIS_CurrentActDev = 0;
						*p_out_buf++ = ATTR_TYPE_ARRAY;
						i = BERLengthEncode(p_out_buf, *p_child_record->p_nr_active_devices);
						p_out_buf += i;
						*p_out_len += i + 1;
					}
					i = sizeof(*p_descriptor);
					while (g_OBIS_CurrentActDev < *p_child_record->p_nr_active_devices)
					{
						if (*p_out_len + one_active_device_length > OBIS_SERVICE_DATA_BUFFER_LENGTH - 10)
						{
							return DATA_ACCESS_RESULT_PARTIAL;
						}
						
						pre_encode_buf_length = *p_out_len;			//use one_active_device_length as temp var
												
						*p_out_buf++ = ATTR_TYPE_STRUCTURE;
						*p_out_buf++ = 14;
						*p_out_len += 2;

						/* 1. mac_address */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
							ATTR_TYPE_OCTET_STRING,											/* Type */
							(Unsigned8 *)p_child_record->p_active_devices[g_OBIS_CurrentActDev].mac_address,	/* Buffer */
							8																/* Length */
						);
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;
						
						/* 2. status */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
							ATTR_TYPE_BIT_STRING,											/* Type */
							(Unsigned8 *)&p_child_record->p_active_devices[g_OBIS_CurrentActDev].status,		/* Buffer */
							1																/* Length */
						);
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;
						
						/* 3. maxRSSI */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
							ATTR_TYPE_INTEGER,										/* Type */
							(Unsigned8 *)&p_child_record->p_active_devices[g_OBIS_CurrentActDev].max_RSSI,					/* Buffer */
							1																/* Length */
						);
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;
						
						/* 4. averageRSSI */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
							ATTR_TYPE_INTEGER,										/* Type */
							(Unsigned8 *)&p_child_record->p_active_devices[g_OBIS_CurrentActDev].average_RSSI,					/* Buffer */
							1																/* Length */
						);
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;
						
						/* 5. minRSSI */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
							ATTR_TYPE_INTEGER,										/* Type */
							(Unsigned8 *)&p_child_record->p_active_devices[g_OBIS_CurrentActDev].min_RSSI,					/* Buffer */
							1																/* Length */
						);
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;
						
						/* 6. maxLQI */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
							ATTR_TYPE_UNSIGNED,										/* Type */
							(Unsigned8 *)&p_child_record->p_active_devices[g_OBIS_CurrentActDev].max_LQI,					/* Buffer */
							1																/* Length */
						);
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;
						
						/* 7. averageLQI */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
							ATTR_TYPE_UNSIGNED,										/* Type */
							(Unsigned8 *)&p_child_record->p_active_devices[g_OBIS_CurrentActDev].average_LQI,					/* Buffer */
							1																/* Length */
						);
						
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;
						
						/* 8. minLQI */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
							ATTR_TYPE_UNSIGNED,										/* Type */
							(Unsigned8 *)&p_child_record->p_active_devices[g_OBIS_CurrentActDev].min_LQI,					/* Buffer */
							1																/* Length */
						);
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;
						
						/* 9. last_communication_date-time */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
							ATTR_TYPE_OCTET_STRING,										/* Type */
							(Unsigned8 *)&p_child_record->p_active_devices[g_OBIS_CurrentActDev].last_comm_date_time,					/* Buffer */
							12																/* Length */
						);
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;
						
						/* 10. number_of_hops */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
							ATTR_TYPE_UNSIGNED,										/* Type */
							(Unsigned8 *)&p_child_record->p_active_devices[g_OBIS_CurrentActDev].number_of_hops,					/* Buffer */
							1																/* Length */
						);
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;
						
						/* 11. transmission_failures */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
							ATTR_TYPE_UNSIGNED,										/* Type */
							(Unsigned8 *)&p_child_record->p_active_devices[g_OBIS_CurrentActDev].transmission_failures,					/* Buffer */
							1																/* Length */
						);
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;
						
						/* 12. transmission_successes */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
							ATTR_TYPE_UNSIGNED,										/* Type */
							(Unsigned8 *)&p_child_record->p_active_devices[g_OBIS_CurrentActDev].transmission_successes,					/* Buffer */
							1																/* Length */
						);
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;
						
						/* 13. application_version */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
							ATTR_TYPE_UNSIGNED,										/* Type */
							(Unsigned8 *)&p_child_record->p_active_devices[g_OBIS_CurrentActDev].application_version,					/* Buffer */
							1																/* Length */
						);
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;
						
						/* 14. stack_version */
						temp_out_len = R_OBIS_EncodeAttribute(
							p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
							OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
							ATTR_TYPE_UNSIGNED,										/* Type */
							(Unsigned8 *)&p_child_record->p_active_devices[g_OBIS_CurrentActDev].stack_version,					/* Buffer */
							1																/* Length */
						);
						p_out_buf = p_out_buf + temp_out_len;
						*p_out_len += temp_out_len;	
						
						/* Subtract after encoded buf length for buf length before encoded */
						/* Need at first time sending only: calculate one_active_device_length */
						one_active_device_length = *p_out_len - pre_encode_buf_length;
						g_OBIS_CurrentActDev++;
						
						if (g_OBIS_CurrentActDev >= *p_child_record->p_nr_active_devices)
						{
							g_OBIS_CurrentActDev = 0;
							/* Success */
							return DATA_ACCESS_RESULT_SUCCESS;
						}
					}
					
				}
				else
				{
				
				}
				
				break;

			default:
				break;
		}
	}
	else if (req_type == REQ_TYPE_ACTION)
	{
		Unsigned8 b_block_transfer = 0;
		
		switch (p_descriptor->Attr_ID & 0x00FF)
		{
			case 1:	/* register_device (data) */
			case 2:	/* unregister_device (data) */
			case 3: /* unregister_all_devices (data) */
			case 4: /* backup_PAN (data) */
			case 5: /* restore_PAN (data) */
			case 6: /* identify_device (data) */
			case 7: /* remove_mirror (data) */
			case 8: /* update_network_key (data) */
			case 9: /* update_link_key (data) */
			case 10: /* create_PAN (data) */
			case 11: /* remove_PAN (data) */
				/* User function take care for all */
				response_result = R_OBIS_DistributeByUserFunc(
					child_index,		/* child_index */	
					req_type,			/* req_type */
					service_type,		/* service_type */
					&b_block_transfer,	/* p_is_block_transfer */
					p_descriptor,		/* p_descriptor */	
					pdata,				/* pdata */			
					p_out_buf,			/* p_out_buf */		
					p_out_len			/* p_out_len */
				);
				
				/* Not support block transfer now, TODO in future */
				if (b_block_transfer != 0)
				{
					response_result = ACTION_RESULT_SCOPE_ACCESS_VIOLATED;
				}
				break;
			default:
				response_result = ACTION_RESULT_TYPE_UNMATCH;
				break;
		}
	}
	else
	{
		/* INTERNAL ERROR */
	}
	/* Reset the current active device value */
	
	
	return (response_result);
}
#endif

/******************************************************************************
* Function Name : R_OBIS_DistributeZigBeeTunnelSetupClass
* Interface     : Unsigned8 R_OBIS_DistributeZigBeeTunnelSetupClass(
*               :     Unsigned16 master_index,
*               :     Unsigned16 child_index,
*               :     req_type_t req_type,
*               :     service_type_t service_type,
*               :     st_Cosem_Attr_Desc *p_descriptor,
*               :     Unsigned8	*pdata,
*               :     Unsigned8	*p_out_buf,
*               :     Unsigned16 *p_out_len
*               : );
* Description   : Distributor function for  Security Setup class
* Arguments     : Unsigned16 master_index -
*               :    [In] Id of the row in master list
*               : Unsigned16 child_index -
*               :    [In] Id of the row in child list
*               : req_type_t req_type -
*               :    [In] Request type (GET/SET/ACTION)
*               : service_type_t service_type -
*               :    [In] Service type (NORMAL/BLOCK/LIST)
*               : st_Cosem_Attr_Desc *p_descriptor -
*               :    [In] COSEM object descriptor
*               : Unsigned8 * pdata -
*               :    [In] Data in case of SET/ACTION
*               : Unsigned8 * p_out_buf -
*               :    [Out] Output buffer for encoded data
*               : Unsigned16 * p_out_len -
*               :    [Out] Output buffer length for encoded data
* Function Calls: TBD
* Return Value  : Unsigned8, distribution status
******************************************************************************/
#if (defined(USED_CLASS_105) && USED_CLASS_105 == 1)
Unsigned8 R_OBIS_DistributeZigBeeTunnelSetupClass(
	Unsigned16  		master_index,
	Unsigned16  		child_index,
	req_type_t			req_type,
	service_type_t		service_type,
	st_Cosem_Attr_Desc	*p_descriptor,
	Unsigned8			*pdata,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
)
{
	/* Result of response, def. is not Matched */
	Unsigned8				response_result = TYPE_UNMATCH;
	Unsigned8				i;
	Unsigned16				temp_out_len;
	Unsigned16				*p_temp_child;
	class105_child_record_t	*p_child_record;

	p_child_record = (class105_child_record_t *)(
		R_OBIS_FindChildRecordByIndex(master_index, child_index)
	);

	/* Classify the code based on Attribute or Method request */
	if (req_type == REQ_TYPE_GET || req_type == REQ_TYPE_SET)
	{
		switch (p_descriptor->Attr_ID)
		{
			case 0: /* attribute 0 */
				/* Not support */
				break;

			case 1: /* logical_name */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,							/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,	/* Max length of g_ServiceBuffer */
						ATTR_TYPE_OCTET_STRING,				/* Octet-string */
						p_descriptor->Instance_ID,			/* OBIS code from the request */
						6);									/* OBIS code is 6 chars */

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing, OBIS code is read only */
				}
				break;

			case 2:	/* Max_incoming_transfer_size  */
				if (req_type == REQ_TYPE_GET)
				{
					temp_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
						ATTR_TYPE_LONG_UNSIGNED,										/* Type */
						(Unsigned8 *)p_child_record->p_max_incoming_transfer_size,		/* Buffer */
						2																/* Length */
					);
					p_out_buf = p_out_buf + temp_out_len;
					*p_out_len += temp_out_len;

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing */
				}
				break;

			case 3:	/* Maximum outgoing transfer size */
				if (req_type == REQ_TYPE_GET)
				{
					temp_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
						ATTR_TYPE_LONG_UNSIGNED,										/* Type */
						(Unsigned8 *)p_child_record->p_max_outcoming_transfer_size,		/* Buffer */
						2																/* Length */
					);
					p_out_buf = p_out_buf + temp_out_len;
					*p_out_len += temp_out_len;

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing */
				}
				break;

			case 4:	/* Protocol_address */
				if (req_type == REQ_TYPE_GET)
				{
					temp_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
						ATTR_TYPE_OCTET_STRING,											/* Type */
						(Unsigned8 *)p_child_record->p_protocol_address,				/* Buffer */
						6																/* Length */
					);
					p_out_buf = p_out_buf + temp_out_len;
					*p_out_len += temp_out_len;

					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing */
				}
				break;

			case 5:	/* Close_tunnel_timeout */
				if (req_type == REQ_TYPE_GET)
				{
					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,														/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,								/* Max length of g_ServiceBuffer */
						ATTR_TYPE_LONG_UNSIGNED,										/* Type */
						(Unsigned8 *)p_child_record->p_close_tunnel_timeout,					/* Buffer */
						2																/* Length */
					);
					
					/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
				}
				else
				{
					/* Do nothing */
				}
				break;

			default:
				break;
		}
	}
	else if (req_type == REQ_TYPE_ACTION)
	{
		/* Not available */
	}
	else
	{
		/* INTERNAL ERROR */
	}

	return (response_result);
}
#endif
