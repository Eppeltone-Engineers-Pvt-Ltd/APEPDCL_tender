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
* File Name    : r_dlms_data.c
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
/* Drivers */

/* DLMS */
#include "r_dlms_data.h"				/* DLMS Data Definitions */
#include "..\\Objects\\r_dlms_obis.h"				/* DLMS OBIS Definitions */
#include "..\\Objects\\r_dlms_obis_ic.h"				/* DLMS OBIS IC Definitions */
#include "..\\DLMS_User.h"					/* DLMS User Definitions */

#if SHA1_ENABLE
#include "r_sha.h"					/* SHA-1 Library */
#endif

#if MD5_ENABLE
#include "r_md5.h"					/* MD5 Library */
#endif

#if AES_ENABLE
/* HLS */
#include "r_dlms_data_hls.h"				/* DLMS Data High level security Definitions */
#endif

/* Standard Lib */
#include <string.h>						/* String Standard Lib */

/*RENESAS DRIVER, TO BE REMOVE*/
#include "..\\r_cg_userdefine.h"
#include "..\\r_dlms_typedef.h"				/* DLMS Type Definitions */

//---------------Device specific-------------
#include "..\\..\\rlDevice\\include\\dI2c.h"

#include "..\\..\\rlApplication\\Include\\AppBlockDailyLS.h"
#include "..\\..\\rlApplication\\Include\\AppConfig.h"

/******************************************************************************
Macro definitions
******************************************************************************/

/******************************************************************************
Typedef definitions
******************************************************************************/

/******************************************************************************
Imported global variables and functions (from other files)
******************************************************************************/

 unsigned char manfact_name[]="Eppeltone Engineers Ltd.";
 unsigned char firmware_version[]="EEOr1";

#if (defined(METER_RATING_5_30) && (METER_RATING_5_30 == 1))
 	unsigned char meter_rating[]="(5-30)A";
#else
	unsigned char meter_rating[]="10-60A";
#endif

 unsigned char ldn_name[]="EEO0000000000EPL";

extern date_time_t		g_Class03_BillingDate;
/*
 * Imported from r_dlms_data_ic.c
 * For data access
 *
 * All of following variables are linked into each specified-child-table.
 * Refer to each child-table in r_dlms_data_ic.c
 */
/* Class ID = 07 - Profile generic class */
#if (defined(USED_CLASS_07) && USED_CLASS_07 == 1)
extern const Unsigned16 	g_Class07_BlockloadCaptureObjectsLength;
extern const Unsigned16 	g_Class07_BillingCaptureObjectsLength;
extern const Unsigned16 	g_Class07_InstCaptureObjectsLength;
#endif
/* Class ID = 64 - Security setup class */
#if (defined(USED_CLASS_64) && USED_CLASS_64 == 1)
extern const class64_child_record_t g_ChildTableClass64[];  /* Security setup reference list   */
extern const Unsigned16 g_ChildTableLengthClass64;  		/* Security setup reference length */
#endif
#if (defined(USED_CLASS_01) && USED_CLASS_01 == 1)
extern Unsigned16		g_Class01_EventCode[7];// = {69,101,121,201,251,0,0};
#endif
/******************************************************************************
Exported global variables and functions (to be accessed by other files)
******************************************************************************/

/******************************************************************************
Private global variables and functions
******************************************************************************/
/* Class ID = 07 - Profile generic class */
#if (defined(USED_CLASS_07) && USED_CLASS_07 == 1)
extern Unsigned32					g_Class07_Blockload_EntriesInUse;
extern Unsigned16					g_Class07_Blockload_CurrentEntry;
extern Unsigned32					g_Class07_Dailyload_EntriesInUse;
extern Unsigned16					g_Class07_Dailyload_CurrentEntry;
extern Unsigned32					g_Class07_Billing_EntriesInUse;
extern Unsigned16					g_Class07_Billing_CurrentEntry;

extern const Unsigned8 PARA_RAM_SIZE_BLOCKLOAD[];
extern const Unsigned8 PARA_RAM_SIZE_DAILYLOAD[];
extern const Unsigned8 PARA_RAM_SIZE_BILLING[];
extern const Unsigned8 PARA_RAM_SIZE_EVENT[];
extern const Unsigned8 PARA_RAM_SIZE_INSTANT[];




const scaler_unit_t		g_Class07_InstScalerBuffer[] =
{
	//{SCALAR_NULL, PHY_UNIT_NONE				},	/* Clock */
	{SCALAR_VOLTAGE, PHY_UNIT_VOLT				},	/* Voltage  */
	{SCALAR_AMPERE, PHY_UNIT_AMPERE				},	/* Phase current  */
	{SCALAR_AMPERE, PHY_UNIT_AMPERE				},	/* Neutral current */
	{SCALAR_PF, PHY_UNIT_NONE				},	/* Signed Power Factor */
	{SCALAR_HERTZ, PHY_UNIT_HERTZ				},	/* Frequency */
	{SCALAR_VA, PHY_UNIT_VOLT_AMPERE		},	/* Apparent Power */
	{SCALAR_WATT, PHY_UNIT_WATT				},	/* Active Power   */
	{SCALAR_WH, PHY_UNIT_WATT_HOUR			},	/* Cumulative Energy - kWh */
	{SCALAR_VAH, PHY_UNIT_VOLT_AMPERE_HOUR	},	/* Cumulative Energy - kVAh*/
	{SCALAR_MD, PHY_UNIT_WATT				},	/* Maximum Demand , kW */
	//{SCALAR_NULL, PHY_UNIT_NONE				},	/* MD kW time */
	{SCALAR_MD, PHY_UNIT_VOLT_AMPERE		},	/* Maximum Demand , kVA */
	//{SCALAR_NULL, PHY_UNIT_NONE				},	/* MD kVA time */
	{SCALAR_NULL, PHY_UNIT_MIN				},	/* Cumulative power ON duration in minutes */
	{SCALAR_NULL, PHY_UNIT_NONE				},	/* Cumulative tamper count */
	{SCALAR_NULL, PHY_UNIT_NONE				},	/* Cumulative billing count */
	{SCALAR_NULL, PHY_UNIT_NONE				},	/* Cumulative programming count */
	{SCALAR_WH, PHY_UNIT_WATT_HOUR			},	/* Cumulative Energy - kWh */
	{SCALAR_VAH, PHY_UNIT_VOLT_AMPERE_HOUR	},	/* Cumulative Energy - kVAh*/
	
};

Unsigned8				g_Class07_Obj0_Capture_Enable = 1;
//class07_blockload_entry_t	g_Class07_BlockLoadBuffer[CLASS07_BLOCKLOAD_MAX_ENTRY];


const scaler_unit_t		g_Class07_BlockloadScalerBuffer[] =
{
	//{SCALAR_NULL, PHY_UNIT_NONE				},	/* RTC time */
	{SCALAR_VOLTAGE, PHY_UNIT_VOLT				},	/* Average voltage  */
	{SCALAR_WH, PHY_UNIT_WATT_HOUR			},	/* Block Energy � kWh */
	{SCALAR_VAH, PHY_UNIT_VOLT_AMPERE_HOUR	},	/* Block Energy - kVAh*/
	{SCALAR_VOLTAGE, PHY_UNIT_AMPERE			},	/* Block Avg Current */
	{SCALAR_PF, PHY_UNIT_NONE				},	/* avg. Power Factor */
//	{SCALAR_MD, PHY_UNIT_WATT				},	/* Avg Load */
//	{SCALAR_VAH, PHY_UNIT_VOLT_AMPERE_HOUR	},	/* Block Energy - kVAh*/
};


const scaler_unit_t			g_Class07_DailyloadScalerBuffer[] =
{
	//{SCALAR_NULL, PHY_UNIT_NONE				},	/* RTC time */
	{SCALAR_WH, PHY_UNIT_WATT_HOUR			},	/* Cumulative Energy � kWh */
	{SCALAR_VAH, PHY_UNIT_VOLT_AMPERE_HOUR	},	/* Cumulative Energy � kVAh */
	{SCALAR_MD, PHY_UNIT_WATT				},	/* Maximum Demand , kW */
	{SCALAR_NULL, PHY_UNIT_MIN				},	/* Total power ON duration in minutes */
	//{SCALAR_WH, PHY_UNIT_WATT_HOUR			},	/* Cumulative Energy � kWh */
};



const scaler_unit_t			g_Class07_BillingScalerBuffer[] =
{
	//{SCALAR_NULL, PHY_UNIT_NONE				},	/* Billing date */
	{SCALAR_APF, PHY_UNIT_NONE				},	/* Average power factor for billing */
	{SCALAR_WH, PHY_UNIT_WATT_HOUR			},	/* Cumulative Energy � kWh */
	{SCALAR_WH, PHY_UNIT_WATT_HOUR			},	/* Cumulative Energy kWh TZ1 */
	{SCALAR_WH, PHY_UNIT_WATT_HOUR			},	/* Cumulative Energy kWh TZ2 */
	{SCALAR_WH, PHY_UNIT_WATT_HOUR			},	/* Cumulative Energy kWh TZ3 */
	{SCALAR_WH, PHY_UNIT_WATT_HOUR			},	/* Cumulative Energy kWh TZ4 */
	{SCALAR_WH, PHY_UNIT_WATT_HOUR			},	/* Cumulative Energy kWh TZ5 */
	{SCALAR_WH, PHY_UNIT_WATT_HOUR			},	/* Cumulative Energy kWh TZ6 */
	{SCALAR_WH, PHY_UNIT_WATT_HOUR			},	/* Cumulative Energy kWh TZ7 */
	{SCALAR_WH, PHY_UNIT_WATT_HOUR			},	/* Cumulative Energy kWh TZ8 */
	{SCALAR_VAH, PHY_UNIT_VOLT_AMPERE_HOUR	},	/* Cumulative Energy � kVAh */
	{SCALAR_VAH, PHY_UNIT_VOLT_AMPERE_HOUR	},	/* Cumulative Energy kVAh TZ1 */
	{SCALAR_VAH, PHY_UNIT_VOLT_AMPERE_HOUR	},	/* Cumulative Energy kVAh TZ2 */
	{SCALAR_VAH, PHY_UNIT_VOLT_AMPERE_HOUR	},	/* Cumulative Energy kVAh TZ3 */
	{SCALAR_VAH, PHY_UNIT_VOLT_AMPERE_HOUR	},	/* Cumulative Energy kVAh TZ4 */
	{SCALAR_VAH, PHY_UNIT_VOLT_AMPERE_HOUR	},	/* Cumulative Energy kVAh TZ5 */
	{SCALAR_VAH, PHY_UNIT_VOLT_AMPERE_HOUR	},	/* Cumulative Energy kVAh TZ6 */
	{SCALAR_VAH, PHY_UNIT_VOLT_AMPERE_HOUR	},	/* Cumulative Energy kVAh TZ7 */
	{SCALAR_VAH, PHY_UNIT_VOLT_AMPERE_HOUR	},	/* Cumulative Energy kVAh TZ8 */
	{SCALAR_MD, PHY_UNIT_WATT			},	/* MD kW */
	//{SCALAR_NULL, PHY_UNIT_NONE				},	/* MD kW time */
	{SCALAR_MD, PHY_UNIT_VOLT_AMPERE	},	/* MD kVA */
	//{SCALAR_NULL, PHY_UNIT_NONE						},	/* MD kVA time */
	{SCALAR_NULL, PHY_UNIT_MIN				},	/* Total power ON duration in minutes */
	{SCALAR_MD, PHY_UNIT_WATT		},	/* MD kW TZ1*/
	//{SCALAR_NULL, PHY_UNIT_NONE		},	/* MD kW time */
	{SCALAR_MD, PHY_UNIT_WATT		},	/* MD kW TZ2*/
	//{SCALAR_NULL, PHY_UNIT_NONE		},	/* MD kW time */
	{SCALAR_MD, PHY_UNIT_WATT		},	/* MD kW TZ3*/
	//{SCALAR_NULL, PHY_UNIT_NONE		},	/* MD kW time */
	{SCALAR_MD, PHY_UNIT_WATT		},	/* MD kW TZ4*/
	//{SCALAR_NULL, PHY_UNIT_NONE		},	/* MD kW time */
	{SCALAR_MD, PHY_UNIT_WATT		},	/* MD kW TZ5*/
	//{SCALAR_NULL, PHY_UNIT_NONE		},	/* MD kW time */
	{SCALAR_MD, PHY_UNIT_WATT		},	/* MD kW TZ6*/
	//{SCALAR_NULL, PHY_UNIT_NONE		},	/* MD kW time */
	{SCALAR_MD, PHY_UNIT_WATT		},	/* MD kW TZ7*/
	//{SCALAR_NULL, PHY_UNIT_NONE		},	/* MD kW time */
	{SCALAR_MD, PHY_UNIT_WATT		},	/* MD kW TZ8*/
	//{SCALAR_NULL, PHY_UNIT_NONE		},	/* MD kW time */
	
	{SCALAR_MD, PHY_UNIT_VOLT_AMPERE	},	/* MD kVA TZ1*/
	//{SCALAR_NULL, PHY_UNIT_NONE		},	/* MD kVA time */
	{SCALAR_MD, PHY_UNIT_VOLT_AMPERE	},	/* MD kVA TZ2*/
	//{SCALAR_NULL, PHY_UNIT_NONE		},	/* MD kVA time */
	{SCALAR_MD, PHY_UNIT_VOLT_AMPERE	},	/* MD kVA TZ3*/
	//{SCALAR_NULL, PHY_UNIT_NONE		},	/* MD kVA time */
	{SCALAR_MD, PHY_UNIT_VOLT_AMPERE	},	/* MD kVA TZ4*/
	//{SCALAR_NULL, PHY_UNIT_NONE		},	/* MD kVA time */
	{SCALAR_MD, PHY_UNIT_VOLT_AMPERE	},	/* MD kVA TZ5*/
	//{SCALAR_NULL, PHY_UNIT_NONE		},	/* MD kVA time */
	{SCALAR_MD, PHY_UNIT_VOLT_AMPERE	},	/* MD kVA TZ6*/
	//{SCALAR_NULL, PHY_UNIT_NONE		},	/* MD kVA time */
	{SCALAR_MD, PHY_UNIT_VOLT_AMPERE	},	/* MD kVA TZ7*/
	//{SCALAR_NULL, PHY_UNIT_NONE		},	/* MD kVA time */
	{SCALAR_MD, PHY_UNIT_VOLT_AMPERE	},	/* MD kVA TZ8*/
	//{SCALAR_NULL, PHY_UNIT_NONE		},	/* MD kVA time */
	
	//{SCALAR_NULL, PHY_UNIT_NONE		},	/* billing tamper count */
//	{SCALAR_WH, PHY_UNIT_WATT_HOUR			},	/* Cumulative Energy � export kWh */
//	{SCALAR_VAH, PHY_UNIT_VOLT_AMPERE_HOUR	},	/* Cumulative Energy � export kVAh */
	
};


extern Unsigned16				g_Class07_Event_CurrentEntry[] ;
extern Unsigned32				g_Class07_Event_EntriesInUse[];

const scaler_unit_t		g_Class07_Event0ScalerBuffer[] =
{
	//{SCALAR_NULL, PHY_UNIT_NONE				},	/* Clock */
	//{SCALAR_NULL, PHY_UNIT_NONE				},	/* Event code  */
	{SCALAR_AMPERE, PHY_UNIT_AMPERE			},	/* Current  */
	{SCALAR_VOLTAGE, PHY_UNIT_VOLT			},	/* Voltage  */
	{SCALAR_APF, PHY_UNIT_NONE				},	/* Signed Power Factor */
	{SCALAR_WH, PHY_UNIT_WATT_HOUR			},	/* Cumulative Energy - kWh */
	{SCALAR_VAH, PHY_UNIT_VOLT_AMPERE_HOUR	},	/* Block Energy - kVAh*/
//	{SCALAR_WH, PHY_UNIT_WATT_HOUR			},	/* Cumulative Energy - export kWh */
	
};
class07_bufferUnion_t	class07_bufferUnion;

#endif

/******************************************************************************
* Function Name : R_OBIS_ActionObjectMethod
* Interface     : Unsigned8 R_OBIS_ActionObjectMethod(
*               :     st_Cosem_Method_Desc *cosem_method_desc,
*               :     service_type_t service_type,
*               :     Unsigned16 child_id,
*               :     Unsigned8 *p_out_buf,
*               :     Unsigned8 *p_out_len,
*               :     Unsigned8 *p_data,
*               :     Unsigned16 data_len,
*               :     Unsigned8 *block_transfer
*               : );
* Description   : Get attribute for a specified object
* Arguments     : st_Cosem_Attr_Desc * cosem_attr_desc: [In] COSEM Obj Descriptor
*               : service_type_t service_type         : [In] Service type
*               : Unsigned16 child_id                 : [In] Child ID in the child object table
*               : Unsigned8 * p_out_buf               : [Out] Encoded output data
*               : Unsigned16 * p_out_len              : [Out] Encoded length
*               : Unsigned8 * p_data                  : [In] Data from client
*               : Unsigned16 data_len                 : [In] Data length
*               : Unsigned8 *block_transfer           : [Out] Indicate whether block transfer is used or not
* Function Calls:
* Return Value  : Unsigned8
******************************************************************************/
Unsigned8 R_OBIS_ActionObjectMethod(
	st_Cosem_Method_Desc *cosem_method_desc,
	service_type_t service_type,
	Unsigned16 child_id,
	Unsigned8 *p_out_buf,
	Unsigned16 *p_out_len,
	Unsigned8 *p_data,
	Unsigned16 data_len,
	Unsigned8 *block_transfer
)
{
	/* Result of action, def. is not Matched */
	Unsigned8 response_result = ACTION_RESULT_TYPE_UNMATCH;

	/* Vars used to handle processing */
	Unsigned16	class_id;
	Unsigned16	method_id = (cosem_method_desc->Method_ID & 0x00FF);
	Unsigned8	b_have_client_data = (Unsigned8)(cosem_method_desc->Method_ID >> 8);

	/* Vars used to encode data */
//	void        *pbuffer;
//	Unsigned16  length;
	Unsigned8   u8_data;
//	Unsigned16  u16_data;
//	Unsigned32  u32_data;
//	Integer32   f32_data;
//	attr_type_t type;

	/* Params check */
	if (cosem_method_desc == NULL ||
		p_out_buf == NULL ||
		p_out_len == NULL ||
		block_transfer == NULL)
	{
		return TYPE_UNMATCH;
	}
	
	/* Get information from attr descriptor */
	class_id = (
		((Unsigned16)cosem_method_desc->Class_ID[0] << 8) |
		((Unsigned16)cosem_method_desc->Class_ID[1])
	);

	/*
	 * Default, marked to response to object layer that not use block transfer
	 * Change this value to 1 at relative class processing (in below) to inform to
	 * object layer that R_OBIS_GetObjectAttr() need to use block transfer
	 */
	*block_transfer = 0;

	/* Default clear the optional data result to 0 */
	*p_out_buf = ATTR_TYPE_NULL_DATA;
	*p_out_len = 0;

	/*
	 * Temporary if (0) to prevent compile error
	 * when no class is used
	 */
	if (0)
	{
		/* Do nothing */
	}
#if (defined(USED_CLASS_07) && USED_CLASS_07 == 1)
	/******************************************************************************
	 * Handle all requests for class 07
	 * Just do on following actions only:
	 * 1. reset (data)
	 * 2. capture (data)
	 *
	 * All other actions are handled in R_OBIS_DistributeProfileGenericClass() function.
	 ******************************************************************************/
	else if (class_id == 7)
	{
//		class07_child_record_t	*p_child_record;

//		/* Get the child record */
//		p_child_record = (class07_child_record_t *)(
//			R_OBIS_FindChildRecordByIndex(class_id, child_id)
//		);
	
		/* Method 1 - reset(data)? */
		if (method_id == 1)
		{
			switch(child_id)
			{
				case 2: /* Block Load Profile */
					g_Class07_Blockload_CurrentEntry = 0;
					g_Class07_Blockload_EntriesInUse = 0;
					break;
				case 4: /* Daily profile */
					g_Class07_Dailyload_CurrentEntry = 0;
					g_Class07_Dailyload_EntriesInUse = 0;
					break;
				case 6: /* Billing profile */
					g_Class07_Billing_CurrentEntry = 0;
					g_Class07_Billing_EntriesInUse = 0;
					break;
				case 9:  /* Event 0 profile */
				case 10: /* Event 1 profile */
				case 11: /* Event 2 profile */
				case 12: /* Event 3 profile */
				case 13: /* Event 4 profile */
				case 14: /* Event 5 profile */
					g_Class07_Event_CurrentEntry[child_id-9] = 0;
					g_Class07_Event_EntriesInUse[child_id-9] = 0;
					break;
				default:
					/* Do nothing*/
					break;
			}
			response_result = ACTION_RESULT_SUCCESS;
		}
		/* Method 2 - capture(data) */
		else if (method_id == 2)
		{
			switch(child_id)
			{
				case 2: /* Block Load Profile */
					//R_OBIS_Class07_BlockloadUpdate();
					break;
				case 4: /* Daily profile */
					//R_OBIS_Class07_DailyloadUpdate();
					break;
				case 6: /* Billing profile */
					//R_OBIS_Class07_BillingUpdate();
					break;
				default:
					/* Do nothing*/
					break;
			}
			response_result = ACTION_RESULT_SUCCESS;
		}
		else if (method_id == 3)
		{
			//NOT support function for old version method
		}
		else if (method_id == 4)
		{
			//NOT support function for old version method
		}
		else
		{
			/* Do nothing */
		}
	}
#endif
#if (defined(USED_CLASS_09) && USED_CLASS_09 == 1)
	/******************************************************************************
	 * Handle all requests for class 09
	 * Just do on following actions only:
	 * 1. execute (data)
	 *
	 * All other actions are handled in R_OBIS_DistributeClockClass() function.
	 ******************************************************************************/
	else if (class_id == 9)
	{
		class09_child_record_t	*p_child_record;

		/* Get the child record */
		p_child_record = (class09_child_record_t *)(
			R_OBIS_FindChildRecordByIndex(class_id, child_id)
		);
	
		/* Method 1 - execute (data)? */
		if (method_id == 1)
		{
			/* Do nothing */
			uint16_t u16_data = ((Unsigned16)*(p_data + 1) << 8) | *(p_data + 2);
			for (u8_data = 0; u8_data < p_child_record->nr_scripts; u8_data ++)
			{
				script_t *check_script = (script_t *)(p_child_record->p_scripts + u8_data);

				if(u16_data == check_script->script_id)
				{
					/*Execute script*/
					if(R_USER_Class09_ScriptExecute(p_child_record->logical_name))
					{
						response_result = ACTION_RESULT_SUCCESS;
					}
					break;
				}
				/* Ensure no reset */
				R_USER_WDT_Restart();
			}
		}
		else
		{
			/* Do nothing */
		}
	}
#endif
#if (defined(USED_CLASS_15) && USED_CLASS_15 == 1)
	/******************************************************************************
	 * Handle all requests for class 15
	 * Just do on following actions only:
	 * 1. reply_to_HLS_authentication (data)
	 * 2. change_HLS_secret (data)
	 * 3. add_object (data)
	 * 4. remove_object (data)
	 *
	 * All other actions are handled in R_OBIS_DistributeAssociationLNClass() function.
	 ******************************************************************************/
	else if (class_id == 15)
	{
		class15_child_record_t	*p_child_record;

		/* Get the child record */
		p_child_record = (class15_child_record_t *)(
		R_OBIS_FindChildRecordByIndex(class_id, child_id)
		);
	
		/* Method 1 - reply_to_HLS_authentication (data)? */
		if (method_id == 1)
		{
			/*
			 * TODO: put the custom verification method here, using
			 * + secret method = p_child_record->mechanism_id (MD5, SHA-1, GMAC, etc ?)
			 * + shared key    = p_child_record->p_shared_key
			 * + StoC          = p_child_record->p_secret_key
			 * + CtoS          = COSEMOpenGetHLSCtoS()
			 * + f(StoC)       = p_data +2
			 * + f(CtoS)       = p_out_buf
			 */
			switch (child_id)
			{
				/* do the same for all associations */
				default:
				{
					Unsigned16	i;					/* counter */

					Unsigned8	*p_client_fStoC;	/* f(StoC) received from client */
					Unsigned8	*p_client_CtoS;		/* CtoS received from client */
					Unsigned8	key[64];			/* store challenge | shared key used to hash */
					Unsigned16	length = 0;			/* Actual length on the key */
#if (MD5_ENABLE || SHA1_ENABLE)
					Unsigned8	hash_value[20];		/* 160-bits hash result */
					/* Init hash_value all 0 */
					memset(hash_value, 0, 20);
#endif

					/* check settings on g_ChildClass15[] & client data */
					if (p_child_record->p_authen_status == NULL ||
						p_child_record->p_secret_key == NULL ||
						b_have_client_data == 0)
					{
						/* failure */
						response_result = ACTION_RESULT_SCOPE_ACCESS_VIOLATED;
						break;
					}
					
					/*
					 * Default the authentication status is failure,
					 * will be update to success if all verification OK
					 */
					response_result = ACTION_RESULT_SCOPE_ACCESS_VIOLATED;
					*p_child_record->p_authen_status = AUTHEN_STATUS_FAILURE;

					/* Restore secret */
					{
						Unsigned16	len = strlen((void *)p_child_record->p_secret_key);
						R_USER_Class15_Secret_Restore(p_child_record->p_secret_key,
												len,
												p_child_record->mechanism_id
												);
					}

					/* Use a custom secret method? */
					if (p_child_record->mechanism_id == MECHANISM_ID2_HIGH_SECURITY)
					{
						/*
						 * TODO: put the custom verification method here
						 * Following example:
						 *        + use ECB encode for challenge with encypt key is shared key
						 *        + Max size is key[64]
						 */
#if AES_ENABLE
						/* Calculate f(StoC) */
						length = R_OBIS_Aes_Ecbenc(p_child_record->p_shared_key, key, p_child_record->p_secret_key);

						/* Check client is authenticated or not: compare with f(StoC) from client */
						if (*(p_data+0) != ATTR_TYPE_OCTET_STRING ||
							*(p_data+1) != (Unsigned8)length)
						{
							/* not match, break as failure */
							break;
						}
						p_client_fStoC = p_data + 2;
						for (i = 0; i < length; i++, p_client_fStoC++)
						{
							if (key[i] != *p_client_fStoC)
							{
								break;
							}
						}
						if (i < length)
						{
							/* not match, break as failure */
							break;
						}

						/* Client is authenticated, processes f(CtoS) send to client */
						*(p_out_buf+0) = ATTR_TYPE_OCTET_STRING;
						p_client_CtoS = COSEMOpenGetHLSCtoS();

						length = R_OBIS_Aes_Ecbenc(p_client_CtoS, (p_out_buf + 2), p_child_record->p_secret_key);
						*(p_out_buf+1) = (Unsigned8)length;
						*p_out_len = length + 2;

						/* Success */
						*p_child_record->p_authen_status = AUTHEN_STATUS_SUCCESS;
						response_result = ACTION_RESULT_SUCCESS;
#endif 					
					}
					
#if MD5_ENABLE
					/* Use MD5? */
					else if (p_child_record->mechanism_id == MECHANISM_ID3_HIGH_SECURITY_MD5)
					{
						/* Create the f(StoC) again */
						i = strlen((void *)p_child_record->p_shared_key);
						memcpy(key, p_child_record->p_shared_key, i);
						length = i;
						
						i = strlen((void *)p_child_record->p_secret_key);
						memcpy(key + length, p_child_record->p_secret_key, i);
						length += i;
						
						R_Md5_HashDigest(
							key,
							hash_value,
							length
						);
						
						/* Verify own f(StoC) with the one that send from client */
						if (*(p_data+0) != ATTR_TYPE_OCTET_STRING ||
							*(p_data+1) != 16)
						{
							/* MD5 not match, break as failure */
							break;
						}

						p_client_fStoC = p_data + 2;				
						for (i = 0; i < 16; i++, p_client_fStoC++)
						{
							if (hash_value[i] != *p_client_fStoC)
							{
								break;
							}
						}
						if (i < 16)
						{
							/* MD5 not match, break as failure */
							break;
						}
						
						/*
						 * If reached here: means verify OK, then make the f(CtoS) and
						 * send back to client
						 */
						*p_out_buf++ = ATTR_TYPE_OCTET_STRING;
						*p_out_buf++ = 16;
						*p_out_len   = 18;
						
						/* Make f(CtoS) (16 bytes) then store to p_out_buf */
						p_client_CtoS = COSEMOpenGetHLSCtoS();
						i = strlen((void *)p_client_CtoS);
						memcpy(key, p_client_CtoS, i);
						length = i;
						
						i = strlen((void *)p_child_record->p_secret_key);
						memcpy(key + length, p_child_record->p_secret_key, i);
						length += i;
						
						R_Md5_HashDigest(
							key,
							p_out_buf,
							length
						);

						/* Success */
						*p_child_record->p_authen_status = AUTHEN_STATUS_SUCCESS;
						response_result = ACTION_RESULT_SUCCESS;
					}
#endif

#if SHA1_ENABLE
					/* Use SHA-1? */
					else if (p_child_record->mechanism_id == MECHANISM_ID4_HIGH_SECURITY_SHA1)
					{
						R_sha1	work;	/* work area of SHA1 */

						/* Create the f(StoC) again */
						i = strlen((void *)p_child_record->p_shared_key);
						memcpy(key, p_child_record->p_shared_key, i);
						length = i;
						
						i = strlen((void *)p_child_record->p_secret_key);
						memcpy(key + length, p_child_record->p_secret_key, i);
						length += i;
						
						memset((void *)&work, 0, sizeof(R_sha1));
						if (R_Sha1_HashDigest(
								key,
								hash_value,
								length,
								(R_SHA_INIT | R_SHA_FINISH),
								&work) != R_PROCESS_COMPLETE)
						{
							/* SHA-1 hash failure */
							break;
						}
						
						/* Verify own f(StoC) with the one that send from client */
						if (*(p_data+0) != ATTR_TYPE_OCTET_STRING ||
							*(p_data+1) != 20)
						{
							/* SHA-1 not match */
							break;
						}

						p_client_fStoC = p_data + 2;				
						for (i = 0; i < 20; i++, p_client_fStoC++)
						{
							if (hash_value[i] != *p_client_fStoC)
							{
								break;
							}
						}
						if (i < 20)
						{
							/* SHA-1 not match */
							break;
						}
						
						/*
						 * If reached here: means verify OK, then make the f(CtoS) and
						 * send back to client
						 */
						*p_out_buf++ = ATTR_TYPE_OCTET_STRING;
						*p_out_buf++ = 20;
						*p_out_len   = 22;
						
						/* Make f(CtoS) (20 bytes) then store to p_out_buf */
						p_client_CtoS = COSEMOpenGetHLSCtoS();
						i = strlen((void *)p_client_CtoS);
						memcpy(key, p_client_CtoS, i);
						length = i;
						
						i = strlen((void *)p_child_record->p_secret_key);
						memcpy(key + length, p_child_record->p_secret_key, i);
						length += i;
						
						memset((void *)&work, 0, sizeof(R_sha1));
						if (R_Sha1_HashDigest(
								key,
								p_out_buf,
								length,
								(R_SHA_INIT | R_SHA_FINISH),
								&work) != R_PROCESS_COMPLETE)
						{
							/* SHA-1 hash failure */
							break;
						}
						
						/* Success */
						*p_child_record->p_authen_status = AUTHEN_STATUS_SUCCESS;
						response_result = ACTION_RESULT_SUCCESS;
					}
#endif
					/* Use GMAC? */
					else if (p_child_record->mechanism_id == MECHANISM_ID5_HIGH_SECURITY_GMAC)
					{
						/*
						 * GMAC verification is supported on global_key_transfer method
						 * of security_setup_ref, not here
						 */
					}
					else
					{
						/* do nothing, use default status as failure */
					}
				}
				
				break;
			}
		}
		/* Method 2 - change_HLS_secret (data)? */
		else if (method_id == 2)
		{
			Unsigned8	*p_client_secret = NULL;	/* secret received from client */
			/* Actual length on the key */
			Unsigned8	length = (Unsigned8)strlen((void *)p_child_record->p_secret_key);

			/* Check client data format */
			if (*(p_data+0) != ATTR_TYPE_OCTET_STRING ||
				*(p_data+1) != (Unsigned8)length)
			{
				/* not match, break as failure */
				return (response_result);
			}
			/* Point to secret */
			p_client_secret = p_data + 2;

			memcpy(p_child_record->p_secret_key, p_client_secret, length);
			/* Backup secret */
			u8_data = R_USER_Class15_Secret_Backup(p_child_record->p_secret_key,
										(Unsigned8)length,
										p_child_record->mechanism_id
										);
			if (u8_data != 0)
			{
				return response_result;
			}

			response_result = ACTION_RESULT_SUCCESS;
		}
		/* Method 3 - add_object (data)? */
		else if (method_id == 3)
		{
			/* Do nothing */
		}
		/* Method 4 - remove_object (data)? */
		else if (method_id == 4)
		{
			/* Do nothing */
		}
		else
		{
			/* Do nothing */
		}
	}
#endif
#if (defined(USED_CLASS_18) && USED_CLASS_18 == 1)
	/******************************************************************************
	 * Handle all requests for class 18
	 * Just do on following actions only:
	 * 1. image_transfer_initiate (data)
	 * 2. image_block_transfer (data)
	 * 3. image_verify (data)
	 * 4. image_activate (data)
	 *
	 ******************************************************************************/
	else if (class_id == 18)
	{
		/* TODO: support these methods */
		response_result = ACTION_RESULT_RD_WRT_DENIED;
	}
#endif
#if (defined(USED_CLASS_20) && USED_CLASS_20 == 1)
	/******************************************************************************
	 * Handle all requests for class 20
	 * Just do on following actions only:
	 * 1. activate_passive_calendar (data)
	 *
	 ******************************************************************************/
	else if (class_id == 20)
	{
		/* Method 1 - activate_passive_calendar (data)? */
		if (method_id == 1)
		{
			u8_data = R_OBIS_Class20_Activate_PassiveCalendar();
			if(u8_data == 0)
			{
				response_result = ACTION_RESULT_SUCCESS;
			}
		}
	}
#endif
#if (defined(USED_CLASS_70) && USED_CLASS_70 == 1)
	/******************************************************************************
	 * Handle all requests for class 70
	 * Just do on following actions only:
	 * 1. remote_disconnect (data)
	 * 2. remote_reconnect (data)
	 *
	 ******************************************************************************/
	else if (class_id == 70)
	{
		/* TODO: support these methods */
		response_result = ACTION_RESULT_RD_WRT_DENIED;
	}
#endif
	else
	{
		return ACTION_RESULT_TYPE_UNMATCH;
	}
	
	return (response_result);
}

/******************************************************************************
* Function Name : R_OBIS_GetObjectAttr
* Interface     : Unsigned8 R_OBIS_GetObjectAttr(
*               :     st_Cosem_Attr_Desc *cosem_attr_desc,
*               :     service_type_t service_type,
*               :     child_record_t *p_child,
*               :     Unsigned8 *p_out_buf,
*               :     Unsigned16 *p_out_len,
*               :     Unsigned8 *block_transfer,
*               :     Unsigned8 *p_data
*               : );
* Description   : Get attribute for a specified object
* Arguments     : st_Cosem_Attr_Desc * cosem_attr_desc: [In] COSEM Obj Descriptor
*               : service_type_t service_type         : [In] Service type
*               : Unsigned16 child_id                 : [In] Child ID in the child object table
*               : Unsigned8 * p_out_buf               : [Out] Encoded output data
*               : Unsigned16 * p_out_len              : [Out] Encoded length
*               : Unsigned8 * block_transfer          : [Out] Indicate whether block transfer is used or not
*               : Unsigned8 * p_data                  : [In] Data from client
* Function Calls:
* Return Value  : Unsigned8
******************************************************************************/
Unsigned8 R_OBIS_GetObjectAttr(
	st_Cosem_Attr_Desc *cosem_attr_desc,
	service_type_t service_type,
	Unsigned16 child_id,
	Unsigned8  *p_out_buf,
	Unsigned16 *p_out_len,
	Unsigned8 *block_transfer,
	Unsigned8 *p_data
)
{
	/* Result of response, def. is not Matched */
	Unsigned8 response_result = TYPE_UNMATCH;

	/* Vars used to handle processing */
	Unsigned16 class_id;
	Unsigned16 attr_id;

	/* Vars used to encode data */
	void        *pbuffer;
	Integer16   size;
//	Unsigned8	u8_data;
//	Unsigned16  u16_data;
//	Unsigned32  u32_data;
//	Integer32   f32_data;
//	attr_type_t type;

	/* Params check */
	if (cosem_attr_desc == NULL ||
		p_out_buf == NULL ||
		p_out_len == NULL ||
		block_transfer == NULL)
	{
		return TYPE_UNMATCH;
	}

	/* Get information from attr descriptor */
	class_id = (
		((Unsigned16)cosem_attr_desc->Class_ID[0] << 8) |
		((Unsigned16)cosem_attr_desc->Class_ID[1])
	);
	attr_id = cosem_attr_desc->Attr_ID;

	/*
	 * Default, marked to response to object layer that not use block transfer
	 * Change this value to 1 at relative class processing (in below) to inform to
	 * object layer that R_OBIS_GetObjectAttr() need to use block transfer
	 */
	*block_transfer = 0;

	/*
	 * Temporary if (0) to prevent compile error
	 * when no class is used
	 */
	if (0)
	{
		/* Do nothing */
	}
#if (defined(USED_CLASS_01) && USED_CLASS_01 == 1)
	/******************************************************************************
	 * Handle all requests for class 01
	 * Just do on following attributes only:
	 * 1. Attr 2 - value : CHOICE
	 *
	 * All other attributes are handled in R_OBIS_DistributeDataClass() function.
	 ******************************************************************************/
	else if (class_id == 1)
	{
		class01_child_record_t	*p_child_record1;

//		date_time_t date_time;

		/* Get the child record */
		p_child_record1 = (class01_child_record_t *)(
			R_OBIS_FindChildRecordByIndex(class_id, child_id)
		);

		/* Attr 2 - value : CHOICE ? */
		if (attr_id == 2)
		{
			Unsigned8 u8;
			Unsigned16 u16;
			Unsigned32 u32;
			Unsigned8 buffer[16];
			memset(buffer, 0, 16);
			/* Default */
			pbuffer = buffer;

			/* Get the size from child table definition */
			size = p_child_record1->value.choice.size;

			/* Get the buffer pointer */
			switch (child_id)
			{
				
				case 0:
					
					size =  R_USER_GetNamePlate(EM_NAME_PLATE_LOGICAL_NAME, pbuffer);
					if (size < 0)
					{
						return 0;
					}
						break;
					
				case 1:		/* Meter serial number */
					/*
					 * TODO: Read manufacture name from EEPROM here
					 * Pass the pointer to buffer
					 */
					size =  R_USER_GetNamePlate(EM_NAME_PLATE_METER_SERIAL, pbuffer);
					if (size < 0)
					{
						return 0;
					}
					break;

				case 2:		/* Manufacture Name */
					/*
					 * TODO: Read manufacture name from EEPROM here
					 * Pass the pointer to buffer
					 */
					memset(buffer, 0, 30);
					pbuffer = buffer;
					size =  R_USER_GetNamePlate(EM_NAME_PLATE_MANUFACT_NAME, pbuffer);
					if (size < 0)
					{
						return 0;
					}
					break;

				case 3:		/* Firmware Version for meter */
					/*
					 * TODO: Read firmware version from EEPROM here
					 * Pass the pointer to buffer
					 */
					size =  R_USER_GetNamePlate(EM_NAME_PLATE_FIRMWARE_NAME, pbuffer);
					if (size < 0)
					{
						return 0;
					}
					break;

				case 4:		/* Meter Type */
					/*
					 * TODO: Read Meter Type from EEPROM here
					 * Pass the pointer to buffer
					 *///u16 = 5;  /* 1P-2W for Single phase */
					size =  R_USER_GetNamePlate(EM_NAME_PLATE_METER_TYPE, pbuffer);
					if (size < 0)
					{
						return 0;
					}
					break;

				case 5:		/* Category */
					/*
					 * TODO: Read Category from EEPROM here
					 * Pass the pointer to buffer
					 */
					size =  R_USER_GetNamePlate(EM_NAME_PLATE_CATEGORY_NAME, pbuffer);
					if (size < 0)
					{
						return 0;
					}
					break;

				case 6:		/* Current rating */
					/*
					 * TODO: Read Current rating from EEPROM here
					 * Pass the pointer to buffer
					 */
					size =  R_USER_GetNamePlate(EM_NAME_PLATE_CURRRATE_NAME, pbuffer);
					if (size < 0)
					{
						return 0;
					}
					break;

				case 7:		/* Meter Year of Manufacture */
					/*
					 * TODO: Read meter year of manufacture from EEPROM here
					 * Pass the pointer to buffer
					 */
					size =  R_USER_GetNamePlate(EM_NAME_PLATE_YEAR_OF_MANUFACT, pbuffer);
					if (size < 0)
					{
						return 0;
					}
					break;
							
				case 8:		/* Demand Integration Period */
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					u16 = (Unsigned16)R_USER_GetEMData(EM_DEMAND_INTEGRATION_PERIOD);
					pbuffer = (Unsigned8 *)&u16;
					break;
				case 9:		/* Profile Capture Period */ //NEED TO CHECK /* Block load capture period?? */
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					u16 = (Unsigned16)R_USER_GetEMData(EM_PROFILE_CAPTURE_PERIOD);
					pbuffer = &u16;
					break;

				case 11:	/* Cumulative tamper count */
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					u16 = (Unsigned16)R_USER_GetEMData(EM_CUCUMLARIVE_TAMPER_CNT);
					pbuffer = (Unsigned8 *)&u16;
					break;

				case 12:	/* Cumulative billing count */
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					u32 = (Unsigned32)R_USER_GetEMData(EM_CUCUMLARIVE_BILLING_CNT);
					pbuffer = (Unsigned8 *)&u32;
					break;

				case 13:	/* Cumulative programming count */
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					u16 = (Unsigned16)R_USER_GetEMData(EM_CUCUMLARIVE_PROGAM_CNT);
					pbuffer = (Unsigned8 *)&u16;
					break;

				case 14:	/* EventCode */
				case 15:
				case 16:
				case 17:
				case 18:
				case 19:
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					//u16 = (Unsigned16)R_OBIS_GetEMData(EM_CUCUMLARIVE_PROGAM_CNT);
					if(child_id==14)
						pbuffer = &g_Class01_EventCode[5];
					else
						pbuffer = &g_Class01_EventCode[child_id-15];
					break;
				case 20:	/* Available Billing Period */
					u8 = (Unsigned8)R_USER_GetEMData(EM_AVL_BILLING_PERIOD);;   //NEED IMPLEMENTATION
					pbuffer = (Unsigned8 *)&u8;
					break;
					
				case 21:	/* Internal diagnostics */
					u8 = (Unsigned8)R_USER_GetEMData(EM_INTERNAL_DIAGNOSTICS);   //NEED IMPLEMENTATION
					pbuffer = (Unsigned8 *)&u8;
					break;
					
				case 22:	/* Download complete status */
					u16 = (Unsigned16)R_USER_GetEMData(EM_DOWNLOAD_COMPLETE);   //NEED IMPLEMENTATION
					pbuffer = (Unsigned8 *)&u16;
					break;
					
				case 23:		/* Lag only/Lead+Lag configuration */
					u16 = (Unsigned16)R_USER_GetEMData(LAG_ONLY_CONFIG);
					pbuffer = (Unsigned8 *)&u16;
					break;	
					
				default:
					break;
			}

			/* Determind the size, based on buffer */
			if (pbuffer != NULL &&
				size == -1)
			{
				size = strlen((void*)pbuffer);
			}

			/* Encode & indicate as success when buffer is valid */
			if (pbuffer != NULL &&
				size != -1)
			{
				*p_out_len = R_OBIS_EncodeAttribute(
					p_out_buf,								/* Output buffer, pointed to g_ServiceBuffer */
					OBIS_SERVICE_DATA_BUFFER_LENGTH,		/* Max length of g_ServiceBuffer */
					p_child_record1->value.choice.type,		/* Type */
					(Unsigned8 *)pbuffer,					/* Buffer */
					size									/* Length */
				);

				/* Success */
				response_result = DATA_ACCESS_RESULT_SUCCESS;
			}
		}
		else
		{
			/* Do nothing */
		}
	}
#endif
#if (defined(USED_CLASS_03) && USED_CLASS_03 == 1)
	/******************************************************************************
	 * Handle all requests for class 03
	 * Just do on following attributes only:
	 * 1. Attr 2 - value : CHOICE.
	 * 2. Attr 3 - scaler_unit : scal_unit_type
	 *
	 * All other attributes are handled in R_OBIS_DistributeRegisterClass() function.
	 ******************************************************************************/
	else if (class_id == 3)
	{
		class03_child_record_t	*p_child_record3;

		/* Get the child record */
		p_child_record3 = (class03_child_record_t *)(
			R_OBIS_FindChildRecordByIndex(class_id, child_id)
		);

		/* Attr 2 - value : CHOICE ? */
		if (attr_id == 2)
		{
			Integer32 temp_f32;
			date_time_t temp_time;
//			Unsigned16 u16;
			
			/* Get the size from child table definition */
			size = p_child_record3->value.choice.size;

			/* Get the buffer pointer */
			switch (child_id)
			{
				case 0:		/* Voltage */
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_VOLTAGE);
					pbuffer = (Unsigned8 *)&temp_f32;
					break;
				case 1:		/* Phase current */
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_PHASE_CURRENT);
					pbuffer = (Unsigned8 *)&temp_f32;
					break;
				case 2:		/* Neutral current */
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_NEUTRAL_CURRENT);
					pbuffer = &temp_f32;
					break;
				case 3:		/* Signed Power Factor */
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_POWER_FACTOR);
					pbuffer = &temp_f32;
					break;
				case 4:		/* Frequency */
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_LINE_FREQ);
					pbuffer = &temp_f32;
					break;
				case 5:		/* Apparent Power */
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_APPARENT_POWER);
					pbuffer = &temp_f32;
					break;
				case 6:		/* Active Power */
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_ACTIVE_POWER);
					pbuffer = &temp_f32;
					break;
				case 7:		/* Cumulative Energy - kWh */
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_CUCUMLARIVE_ENERGY_KWH);
					pbuffer = &temp_f32;
					break;
				case 8:		/* Cumulative Energy - kVAh */
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_CUCUMLARIVE_ENERGY_KVAH);
					pbuffer = &temp_f32;
					break;
				case 9:		/* Cumulative power ON duration in minutes */
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_CUCUMLARIVE_POWER_ON_DURATION);
					pbuffer = &temp_f32;
					break;
				case 10:		/* Average voltage */
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_AVERAGE_VOLTAGE);
					pbuffer = &temp_f32;
					break;
				case 11:		/* Block Energy � kWh */
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_BLOCK_ENERGY_KWH);
					pbuffer = &temp_f32;
					break;
				case 12:		/* Block Energy � kVAh */
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_BLOCK_ENERGY_KVAH);
					pbuffer = &temp_f32;
					break;
				case 13:		/* Billing date */
					{
						
						Unsigned8 rlt;
						/* Get time */
						rlt = R_USER_GetSpecificTime(EM_LASTBILLING_TIME, &temp_time);
						pbuffer = (Unsigned8 *)&temp_time;
						size = sizeof(date_time_t); 
						if (rlt != 0)
						{
							return (response_result);
						}
					}
					break;
				case 14:		/* Average power factor for billing  */
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_AVERAGE_POWER_FACTOR);
					pbuffer = &temp_f32;
					break;
				case 15:		/* Cumulative Energy kWh TZ1 */
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_CUCUMLARIVE_ENERGY_KWH_TZ1);
					pbuffer = &temp_f32;
					break;
				case 16:		/* Cumulative Energy kWh TZ2 */
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_CUCUMLARIVE_ENERGY_KWH_TZ2);
					pbuffer = &temp_f32;
					break;
				case 17:		/* Cumulative Energy kWh TZ3 */
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_CUCUMLARIVE_ENERGY_KWH_TZ3);
					pbuffer = &temp_f32;
					break;
				case 18:		/* Cumulative Energy kWh TZ4 */
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_CUCUMLARIVE_ENERGY_KWH_TZ4);
					pbuffer = &temp_f32;
					break;
				case 19:		/* Cumulative Energy kWh TZ5 */
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_CUCUMLARIVE_ENERGY_KWH_TZ5);
					pbuffer = &temp_f32;
					break;
				case 20:		/* Cumulative Energy kWh TZ6 */
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_CUCUMLARIVE_ENERGY_KWH_TZ6);
					pbuffer = &temp_f32;
					break;
				case 21:		/* Cumulative Energy kWh TZ7 */
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_CUCUMLARIVE_ENERGY_KWH_TZ7);
					pbuffer = &temp_f32;
					break;
				case 22:		/* Cumulative Energy kWh TZ8 */
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_CUCUMLARIVE_ENERGY_KWH_TZ8);
					pbuffer = &temp_f32;
					break;					
				case 23:		/* Cumulative Energy kVAh TZ1 */
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_CUCUMLARIVE_ENERGY_KVAH_TZ1);
					pbuffer = &temp_f32;
					break;
				case 24:		/* Cumulative Energy kVAh TZ2 */
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_CUCUMLARIVE_ENERGY_KVAH_TZ2);
					pbuffer = &temp_f32;
					break;
				case 25:		/* Cumulative Energy kVAh TZ3 */
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_CUCUMLARIVE_ENERGY_KVAH_TZ3);
					pbuffer = &temp_f32;
					break;
				case 26:		/* Cumulative Energy kVAh TZ4 */
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_CUCUMLARIVE_ENERGY_KVAH_TZ4);
					pbuffer = &temp_f32;
					break;
				case 27:		/* Cumulative Energy kVAh TZ5 */
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_CUCUMLARIVE_ENERGY_KVAH_TZ5);
					pbuffer = &temp_f32;
					break;
				case 28:		/* Cumulative Energy kVAh TZ6 */
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_CUCUMLARIVE_ENERGY_KVAH_TZ6);
					pbuffer = &temp_f32;
					break;
				case 29:		/* Cumulative Energy kVAh TZ7 */
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_CUCUMLARIVE_ENERGY_KVAH_TZ7);
					pbuffer = &temp_f32;
					break;
				case 30:		/* Cumulative Energy kVAh TZ8 */
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_CUCUMLARIVE_ENERGY_KVAH_TZ8);
					pbuffer = &temp_f32;
					break;
					
				case 31:		/* Current */
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_CURRENT);
					pbuffer = &temp_f32;
					break;
				
				case 32:	/* Total power ON duration in minutes */
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = (Integer32)R_USER_GetEMData(EM_POWER_ON_DURATION);
					pbuffer = &temp_f32;
					break;
				case 33:		
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_CUCUMLARIVE_EXP_ENERGY_KWH);
					pbuffer = &temp_f32;
					break;
				case 34:		
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_CUCUMLARIVE_EXP_ENERGY_KVAH);
					pbuffer = &temp_f32;
					break;
				case 35:		
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_PRE_BILL_DATE_READ);
					pbuffer = &temp_f32;
					break;
				case 36:
					temp_f32 = R_USER_GetEMData(EM_BLOCK_ENERGY_KWH_EXP);
					pbuffer = &temp_f32;
					break;
				case 37:
					temp_f32 = R_USER_GetEMData(EM_BLOCK_AVG_PF);
					pbuffer = &temp_f32;
					break;
				case 38:
					temp_f32 = R_USER_GetEMData(EM_BLOCK_AVG_CURENT);
					pbuffer = &temp_f32;
					break;
				case 39:
					temp_f32 = R_USER_GetEMData(EM_BLOCK_AVG_LOAD);
					pbuffer = &temp_f32;
					break;
			
				default:
					break;
			}
			/* Determind the size, based on buffer */
			if (pbuffer != NULL &&
				size == -1)
			{
				size = strlen((void*)pbuffer);
			}

			/* Encode & indicate as success when buffer is valid */
			if (pbuffer !=NULL)
			{
				*p_out_len = R_OBIS_EncodeAttribute(
					p_out_buf,								/* Output buffer, pointed to g_ServiceBuffer */
					OBIS_SERVICE_DATA_BUFFER_LENGTH,		/* Max length of g_ServiceBuffer */
					p_child_record3->value.choice.type,		/* Type */
					(Unsigned8 *)pbuffer,					/* Buffer */
					size									/* Length */
				);

				/* Success */
				response_result = DATA_ACCESS_RESULT_SUCCESS;
			}
		}
		else
		{
			/* Do nothing */
		}
	}
#endif
#if (defined(USED_CLASS_04) && USED_CLASS_04 == 1)
	/******************************************************************************
	 * Handle all requests for class 04
	 * Just do on following attributes only:
	 * 1. Logical name (OBIS code) of the object.
	 * 2. Access right definition for 1 object   
	 * 3  Value                                  
	 * 4  Scaler, unit                           
	 * 5  Status                                 
	 * 6  Capture time buffer                    
	 *
	 *
	 *
	 * All other attributes are handled in R_OBIS_DistributeRegisterClass() function.
	 ******************************************************************************/
	else if (class_id == 4)
	{
		class04_child_record_t	*p_child_record4;

		/* Get the child record */
		p_child_record4 = (class04_child_record_t *)(
			R_OBIS_FindChildRecordByIndex(class_id, child_id)
		);

		/* Attr 2 - value : CHOICE ? */
		if (attr_id == 2)
		{
			Unsigned32 temp_f32;
			void *buff = NULL;

			/* Get the buffer pointer */
			switch (child_id)
			{
				case 0:		/* Maximum Demand  kW */
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_ACTIVE_MD);
					buff = &temp_f32;
					break;
				case 1:		/* Maximum Demand  kVA */
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_APPARENT_MD);
					buff = &temp_f32;
					break;
				case 2:		/* Maximum Demand  kW TZ1*/
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_ACTIVE_MD_TZ1);
					buff = &temp_f32;
					break;
				case 3:		/* Maximum Demand  kW TZ2*/
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_ACTIVE_MD_TZ2);
					buff = &temp_f32;
					break;
				case 4:		/* Maximum Demand  kW TZ3*/
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_ACTIVE_MD_TZ3);
					buff = &temp_f32;
					break;
				case 5:		/* Maximum Demand  kW TZ4*/
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_ACTIVE_MD_TZ4);
					buff = &temp_f32;
					break;
				case 6:		/* Maximum Demand  kW TZ5*/
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_ACTIVE_MD_TZ5);
					buff = &temp_f32;
					break;
				case 7:		/* Maximum Demand  kW TZ6*/
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_ACTIVE_MD_TZ6);
					buff = &temp_f32;
					break;
				case 8:		/* Maximum Demand  kW TZ7*/
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_ACTIVE_MD_TZ7);
					buff = &temp_f32;
					break;
				case 9:		/* Maximum Demand  kW TZ8*/
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_ACTIVE_MD_TZ8);
					buff = &temp_f32;
					break;					
				case 10:		/* Maximum Demand  kVA TZ1*/
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_APPARENT_MD_TZ1);
					buff = &temp_f32;
					break;
				case 11:		/* Maximum Demand  kVA TZ2*/
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_APPARENT_MD_TZ2);
					buff = &temp_f32;
					break;
				case 12:	/* Maximum Demand  kVA TZ3*/
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_APPARENT_MD_TZ3);
					buff = &temp_f32;
					break;
				case 13:	/* Maximum Demand  kVA TZ4*/
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_APPARENT_MD_TZ4);
					buff = &temp_f32;
					break;
				case 14:	/* Maximum Demand  kVA TZ5*/
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_APPARENT_MD_TZ5);
					buff = &temp_f32;
					break;
				case 15:	/* Maximum Demand  kVA TZ6*/
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_APPARENT_MD_TZ6);
					buff = &temp_f32;
					break;
				case 16:	/* Maximum Demand  kVA TZ7*/
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_APPARENT_MD_TZ7);
					buff = &temp_f32;
					break;
				case 17:	/* Maximum Demand  kVA TZ8*/
					/*
					 * TODO: Read meter data from EEPROM here
					 * Pass the pointer to buffer
					 */
					temp_f32 = R_USER_GetEMData(EM_APPARENT_MD_TZ8);
					buff = &temp_f32;
					break;					
				default:
					break;
			}

			/* Encode & indicate as success when buffer is valid */
			if (buff !=NULL)
			{
				*p_out_len = R_OBIS_EncodeAttribute(
					p_out_buf,								/* Output buffer, pointed to g_ServiceBuffer */
					OBIS_SERVICE_DATA_BUFFER_LENGTH,		/* Max length of g_ServiceBuffer */
					p_child_record4->value.choice.type,		/* Type */
					(Unsigned8 *)buff,						/* Buffer */
					p_child_record4->value.choice.size		/* Length */
				);

				/* Success */
				response_result = DATA_ACCESS_RESULT_SUCCESS;
			}
		}
		/* Attr 5 - capture_time : octet-string */
		if (attr_id == 5)
		{
			date_time_t temp_time;
			switch (child_id)
			{
				case 0:		/* Maximum Demand  kW */
					{
						Unsigned8 rlt;
						/* Get time */
						rlt = R_USER_GetSpecificTime(EM_LAST_ACTIVE_MD_TIME, &temp_time);
						if (rlt != 0)
						{
							return (response_result);
						}
					}
					break;
				case 1:		/* Maximum Demand  kVA */
					{
						Unsigned8 rlt;
						/* Get time */
						rlt = R_USER_GetSpecificTime(EM_LAST_APPARENT_MD_TIME, &temp_time);
						if (rlt != 0)
						{
							return (response_result);
						}
						break;
					}
					
			//		break;
				default:
					return response_result;
			}
			*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,								/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,		/* Max length of g_ServiceBuffer */
						ATTR_TYPE_OCTET_STRING,					/* Type */
						(Unsigned8 *)&temp_time,				/* Buffer */
						sizeof(date_time_t)						/* Length */
						);
			/* Success */
			response_result = DATA_ACCESS_RESULT_SUCCESS;
		}
		else
		{
			/* Do nothing */
		}
	}
#endif
#if (defined(USED_CLASS_08) && USED_CLASS_08 == 1)
	/******************************************************************************
	 * Handle all requests for class 08
	 *
	 * NOTE: THE CLASS 08 ALLOW ONLY 1 INSTANCE TO BE IMPLEMENTED.
	 *
	 * Just do on following attributes only:
	 * 1. Attr 2 - time : octet-string.
	 * 2. Attr 3 - time_zone : long
	 * 3. Attr 4 - status : unsigned
	 * 4. Attr 5 - daylight_savings_begin : octet-string
	 * 5. Attr 6 - daylight_savings_end : octet-string
	 * 6. Attr 7 - daylight_savings_deviation : integer
	 * 7. Attr 8 - daylight_savings_enabled : boolean
	 * 8. Attr 9 - clock_base : enum
	 *
	 * All other attributes are handled in R_OBIS_DistributeClockClass() function.
	 ******************************************************************************/
	else if (class_id == 8)
	{
		/* Attr 2 - time : octet-string. */
		if (attr_id == 2)
		{
			date_time_t class8_currenttime;
			Unsigned8 rlt;
			switch (child_id)
			{
				case 0:		/* RTC time */
					/* Get current time */
					rlt = R_USER_GetSpecificTime(EM_CURRENT_TIME, &class8_currenttime);
					if (rlt != 0)
					{
						return (response_result);
					}

					*p_out_len = R_OBIS_EncodeAttribute(
						p_out_buf,								/* Output buffer, pointed to g_ServiceBuffer */
						OBIS_SERVICE_DATA_BUFFER_LENGTH,		/* Max length of g_ServiceBuffer */
						ATTR_TYPE_OCTET_STRING,					/* Type */
						(Unsigned8 *)&class8_currenttime,	/* Buffer */
						sizeof(date_time_t)						/* Length */
						);

						/* Success */
					response_result = DATA_ACCESS_RESULT_SUCCESS;
					break;
				default:
					break;
			}
		}
		else
		{
			/* Do nothing */
		}

	}
#endif
	else
	{
		return SCOPE_ACCESS_VIOLATED;
	}

	return (response_result);
}

/******************************************************************************
* Function Name : R_OBIS_SetObjectAttr
* Interface     : Unsigned8 R_OBIS_SetObjectAttr(
*               :     st_Cosem_Attr_Desc *cosem_attr_desc,
*               :     service_type_t service_type,
*               :     child_record_t *p_child,
*               :     Unsigned8 *p_data,
*               :     Unsigned16 data_len,
*               :     Unsigned8 block_transfer
*               : );
* Description   : Set attribute for a specified object
* Arguments     : st_Cosem_Attr_Desc * cosem_attr_desc: [In] COSEM Obj Descriptor
*               : service_type_t service_type         : [In] Service type
*               : Unsigned16 child_id                 : [In] Child ID in the child object table
*               : Unsigned8 * p_data                  : [In] Data from client
*               : Unsigned16 data_len                 : [In] Data length
*               : Unsigned8 block_transfer            : [In] Indicate whether block transfer is used or not
* Function Calls:
* Return Value  : Unsigned8
******************************************************************************/
Unsigned8 R_OBIS_SetObjectAttr(
	st_Cosem_Attr_Desc *cosem_attr_desc,
	service_type_t service_type,
	Unsigned16 child_id,
	Unsigned8 *p_data,
	Unsigned16 data_len,
	Unsigned8 block_transfer
)
{
	/* Result of response, def. is not Matched */
	Unsigned8 response_result = TYPE_UNMATCH;

	/* Vars used to handle processing */
	Unsigned16 class_id;
	Unsigned16 attr_id;

	/* Vars used to encode data */
//	void        *pbuffer;
	Unsigned16  length;
//	Unsigned8	u8_data=10;
//	Unsigned16  u16_data;
//	Unsigned32  u32_data;
//	Integer32   f32_data;
//	attr_type_t type;

	/* Params check */
	if (cosem_attr_desc == NULL ||
		p_data == NULL ||
		data_len == 0)
	{
		return TYPE_UNMATCH;
	}

	/* Get information from attr descriptor */
	class_id = (
		((Unsigned16)cosem_attr_desc->Class_ID[0] << 8) |
		((Unsigned16)cosem_attr_desc->Class_ID[1])
	);
	attr_id = cosem_attr_desc->Attr_ID;

	/*
	 * Temporary if (0) to prevent compile error
	 * when no class is used
	 */
	if (0)
	{
		/* Do nothing */
	}//RC
	#if (defined(USED_CLASS_01) && USED_CLASS_01 == 1)
	/******************************************************************************
	 * Handle all requests for class 01
	 * Just do on following attributes only:
	 * 1. Attr 2 - value : CHOICE
	 *
	 * All other attributes are handled in R_OBIS_DistributeDataClass() function.
	 ******************************************************************************/
	else if (class_id == 1)
	{
		class01_child_record_t	*p_child_record1;
		/* Get the child record */
		p_child_record1 = (class01_child_record_t *)(
			R_OBIS_FindChildRecordByIndex(class_id, child_id)
		);

		/* Attr 2 - value : CHOICE ? */
		if (attr_id == 2)
		{
			Unsigned16 u16;
//			Integer16 size;
//			void *buffer = NULL;

		u16 = 0;
		length = R_OBIS_DecodeAttribute(
						(Unsigned8 *)&u16,							/* value_buf */
						p_child_record1->value.choice.size,			/* value_buf_len */
						p_child_record1->value.choice.type,			/* attr_type */
						p_data,										/* buf */
						OBIS_SERVICE_DATA_BUFFER_LENGTH				/* buf_len */
					);

			/* Get the buffer pointer */
			switch (child_id)
			{
				case 8:		/* Demand Integration Period */
					length = (Unsigned16)R_USER_SetEMData(EM_DEMAND_INTEGRATION_PERIOD, (void *)&u16);
					response_result = DATA_ACCESS_RESULT_SUCCESS;
					break;
				case 9:		/* Block load capture period */
					length = (Unsigned16)R_USER_SetEMData(EM_BLOCKLOAD_PERIOD, (void *)&u16);
					response_result = DATA_ACCESS_RESULT_SUCCESS;
					#if (defined(USED_CLASS_07) && USED_CLASS_07 == 1)
					//g_Class07_Blockload_CapturePeriod = u16; //NEED TO IMPLEMENT
					#endif
					break;
				case 11:	/* Cumulative tamper count */
					length = (Unsigned16)R_USER_SetEMData(EM_CUCUMLARIVE_TAMPER_CNT, (void *)&u16);
					response_result = DATA_ACCESS_RESULT_SUCCESS;
					break;
				case 12:	/* Cumulative billing count */
					length = (Unsigned16)R_USER_SetEMData(EM_CUCUMLARIVE_BILLING_CNT, (void *)&u16);
					response_result = DATA_ACCESS_RESULT_SUCCESS;
					break;
				case 13:	/* Cumulative programming count */
					length = (Unsigned16)R_USER_SetEMData(EM_CUCUMLARIVE_PROGAM_CNT, (void *)&u16);
					response_result = DATA_ACCESS_RESULT_SUCCESS;
					break;
				case 21:	/* Download complete status */
					length = (Unsigned16)R_USER_SetEMData(EM_DOWNLOAD_COMPLETE, (void *)&u16);
					response_result = DATA_ACCESS_RESULT_SUCCESS;
					break;

				case 23:		/* Lag only/Lead+Lag configuration */
					length = (Unsigned16)R_USER_SetEMData(LAG_ONLY_CONFIG, (void *)&u16);
					response_result = DATA_ACCESS_RESULT_SUCCESS;
					break;	

				default:
					break;
				
			}
		}
	}
#endif
#if (defined(USED_CLASS_03) && USED_CLASS_03 == 1)
	/******************************************************************************
	 * Handle all requests for class 03
	 * Just do on following attributes only:
	 * 1. Attr 2 - value : CHOICE
	 *
	 * All other attributes are handled in R_OBIS_DistributeDataClass() function.
	 ******************************************************************************/
	else if (class_id == 3)
	{
		class01_child_record_t	*p_child_record1;
		/* Get the child record */
		p_child_record1 = (class01_child_record_t *)(
			R_OBIS_FindChildRecordByIndex(class_id, child_id)
		);

		/* Attr 2 - value : CHOICE ? */
		if (attr_id == 2)
		{
			Unsigned16 u16;

		
			length = R_OBIS_DecodeAttribute(
						(Unsigned8 *)&g_Class03_BillingDate,							/* value_buf */
						p_child_record1->value.choice.size,			/* value_buf_len */
						p_child_record1->value.choice.type,			/* attr_type */
						p_data,										/* buf */
						OBIS_SERVICE_DATA_BUFFER_LENGTH				/* buf_len */
					);

			/* Get the buffer pointer */
			switch (child_id)
			{
				case 13:		/* Demand Integration Period */
					length = (Unsigned16)R_USER_SetEMData(EM_BILL_DATE, (void *)&g_Class03_BillingDate);
					response_result = DATA_ACCESS_RESULT_SUCCESS;
					break;
				default:
					break;
				
			}
		}
	}
#endif

#if (defined(USED_CLASS_08) && USED_CLASS_08 == 1)
	/******************************************************************************
	 * Handle all requests for class 08
	 *
	 * NOTE: THE CLASS 08 ALLOW ONLY 1 INSTANCE TO BE IMPLEMENTED.
	 *
	 * Just do on following attributes only:
	 * 1. Attr 2 - time : octet-string.
	 * 2. Attr 3 - time_zone : long
	 * 3. Attr 4 - status : unsigned
	 * 4. Attr 5 - daylight_savings_begin : octet-string
	 * 5. Attr 6 - daylight_savings_end : octet-string
	 * 6. Attr 7 - daylight_savings_deviation : integer
	 * 7. Attr 8 - daylight_savings_enabled : boolean
	 * 8. Attr 9 - clock_base : enum
	 *
	 * All other attributes are handled in R_OBIS_DistributeClockClass() function.
	 ******************************************************************************/
	else if (class_id == 8)
	{
		/* Attr 2 - time : octet-string. */
		if (attr_id == 2)
		{
			date_time_t user_time;
			Unsigned8 ret;
			switch (child_id)
			{
				case 0:		/* RTC time */
					length = R_OBIS_DecodeAttribute(
							(Unsigned8 *)&user_time,					/* value_buf */
							sizeof(date_time_t),						/* value_buf_len */
							ATTR_TYPE_DATE_TIME,						/* attr_type */
							p_data,										/* buf */
							OBIS_SERVICE_DATA_BUFFER_LENGTH				/* buf_len */
						);

					if (length == sizeof(date_time_t) + 2)
					{
						/* Set current time */
						ret = R_USER_SetSpecificTime(EM_CURRENT_TIME, user_time);
						if(ret != 0)
						{
							return response_result;
						}
						/* Success */
						response_result = DATA_ACCESS_RESULT_SUCCESS;
					}
					break;
				default:
					break;
			}
		}
		else if (attr_id == 3)
		{
			
		}
		else
		{
			/* Do nothing */
		}

	}
#endif
#if (defined(USED_CLASS_20) && USED_CLASS_20 == 1)
	/******************************************************************************
	 * Handle all requests for class 01
	 * Just do on following attributes only:
	 * 1. Attr 2 - value : CHOICE
	 *
	 * All other attributes are handled in R_OBIS_DistributeDataClass() function.
	 ******************************************************************************/
	else if (class_id == 20)
	{
		if (attr_id == 10)
		{
			//Set_PP(0,5);
		}
		
	}
#endif
	else
	{
		return SCOPE_ACCESS_VIOLATED;
	}

	return (response_result);
}

/******************************************************************************
* Function Name : R_OBIS_DataAsyncServiceRestart
* Interface     : void R_OBIS_DataAsyncServiceRestart(void)
* Description   : Acknowledgement about the reset of async service
* Arguments     : None
* Function Calls: TBD
* Return Value  : None
******************************************************************************/
void R_OBIS_DataAsyncServiceRestart(void)
{
	Unsigned8 i;
#if (defined(USED_CLASS_07) && USED_CLASS_07 == 1)
	class07_child_record_t *p_child_record = NULL;

	R_OBIS_Class07_BlockLoadUpdate_Start();

	/* Reset buffer block transfer process  for all */
	for (i = 0; i < *(g_MasterTable[7].child_list_size); i++)
	{
		p_child_record = (class07_child_record_t *)(
			R_OBIS_FindChildRecordByIndex(7, i)
		);
		if(p_child_record != NULL)
		{
			p_child_record->p_buff->async_remain_entries = 0;
		}
	}
#endif

#if (defined(USED_CLASS_64) && USED_CLASS_64 == 1)
	{
		class64_child_record_t *p_child_record64 = NULL;

	/* Reset frame counter for all */
		for (i = 0; i < *(g_MasterTable[64].child_list_size); i++)
		{
			p_child_record64 = (class64_child_record_t *)(
				R_OBIS_FindChildRecordByIndex(64, i)
				);
			if(p_child_record64 != NULL)
			{
				*(p_child_record64->p_frame_counter) = 0;
				memset(p_child_record64->p_dedicated_unicast_key, 0, 16);
			}
		}
	}
#endif
}

/******************************************************************************
* Function Name : R_OBIS_E2PRGetData
* Interface     : Unsigned8 R_OBIS_E2PRGetData(
*               :     attr_access_t			*p_attr_access,
*               :     attr_type_t 			attr_type,
*               :     Integer16				attr_size,
*               :     Unsigned8				*p_out_buf,
*               :     Unsigned16			*p_out_len
*               : );
* Description   : Get attribute on EEPROM for a specified object
* Arguments     : attr_access_t *p_attr_access     : [In] Access method option
*               : attr_type_t attr_type            : [In] Choice selection
*               : Unsigned16 attr_size             : [In] Buffer of value
*               : Unsigned8 * p_out_buf            : [Out] Data pointer for get
*               : Unsigned16 * p_out_len           : [Out] Data length pointer for get
* Function Calls:
* Return Value  : Unsigned8
******************************************************************************/
Unsigned8 R_OBIS_E2PRGetData(
	attr_access_t		*p_attr_access,
	attr_type_t 		attr_type,
	Integer16			attr_size,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
)
{
	/* Result of response, def. is not Matched */
	Unsigned8 response_result = TYPE_UNMATCH;

	/* Vars used to encode data */
	Unsigned8       buffer[OBIS_SERVICE_DATA_BUFFER_LENGTH];

	/* Params check */
	if (p_out_buf == NULL ||
		p_out_len == NULL ||
		p_attr_access == NULL ||
		p_attr_access->method != ACCESS_MTD_E2PR)
	{
		return TYPE_UNMATCH;
	}

	if (attr_type != ATTR_TYPE_ARRAY &&
		attr_type != ATTR_TYPE_STRUCTURE &&
		attr_type != ATTR_TYPE_COMPACT_ARRAY)
	{
		/* EEPROM access */
		/* write buffer */
		if(1)//if (read_eeprom(p_attr_access->addr, buffer, p_attr_access->size) == E2PR_OK)
		{
			/* Success */
			response_result = DATA_ACCESS_RESULT_SUCCESS;
		}
		else
		{
			return RD_WRT_DENIED;
		}
		/* Encode inside */
		*p_out_len = R_OBIS_EncodeAttribute(
			p_out_buf,							/* Output buffer, pointed to g_ServiceBuffer */
			OBIS_SERVICE_DATA_BUFFER_LENGTH,	/* Max length of g_ServiceBuffer */
			attr_type,							/* Type */
			buffer,								/* Buffer */
			p_attr_access->size					/* Length */
		);

	}
	else
	{
		/* need to call to user distributor ? */
		return SCOPE_ACCESS_VIOLATED;
	}

	return (response_result);
}

/******************************************************************************
* Function Name : R_OBIS_E2PRSetData
* Interface     : Unsigned8 R_OBIS_E2PRGetData(
*               :     attr_access_t			*p_attr_access,
*               :     attr_type_t 			attr_type,
*               :     Integer16				attr_size,
*               :     Unsigned8				*pdata,
*               : );
* Description   : Set attribute on EEPROM for a specified object
* Arguments     : attr_access_t *p_attr_access     : [In] Access method option
*               : attr_type_t attr_type            : [In] Choice selection
*               : Unsigned16 attr_size             : [In] Buffer of value
*               : Unsigned8 * pdata                : [In]Data from COSEM Appl layer
* Function Calls:
* Return Value  : Unsigned8
******************************************************************************/
Unsigned8 R_OBIS_E2PRSetData(
	attr_access_t		*p_attr_access,
	attr_type_t 		attr_type,
	Integer16			attr_size,
	Unsigned8			*pdata
)
{
	/* Result of response, def. is not Matched */
	Unsigned8 response_result = TYPE_UNMATCH;
	Unsigned16 decoded_length;

	/* Vars used to encode data */
	Unsigned8       buffer[OBIS_SERVICE_DATA_BUFFER_LENGTH];

	/* Params check */
	if (p_attr_access == NULL ||
		p_attr_access->method != ACCESS_MTD_E2PR)
	{
		return TYPE_UNMATCH;
	}

	if (attr_type != ATTR_TYPE_ARRAY &&
		attr_type != ATTR_TYPE_STRUCTURE &&
		attr_type != ATTR_TYPE_COMPACT_ARRAY)
	{
		/* Decode attribute */
		decoded_length = R_OBIS_DecodeAttribute(
			buffer,									/* Output value buffer */
			p_attr_access->size,					/* Size of buffer */
			attr_type,								/* Type */
			pdata,									/* Get from data pointer of HDLC protocol stack */
			OBIS_SERVICE_DATA_BUFFER_LENGTH			/* Encoded length from HDLC */
		);

		/* EEPROM access */
		/* write buffer */
		if(1)//if (write_eeprom(p_attr_access->addr, buffer, p_attr_access->size) == E2PR_OK)
		{
			/* Success */
			response_result = DATA_ACCESS_RESULT_SUCCESS;
		}
		else
		{
			response_result = RD_WRT_DENIED;
		}


	}
	else
	{
		/* need to call to user distributor ? */
	}

	return (response_result);
}

/******************************************************************************
* Function Name : R_OBIS_BufferBlockGet
* Interface     : Unsigned8 R_OBIS_BufferBlockGet(
*               :     void				*buffer,
*               :     attr_access_t		*p_attr_access,
*               :     Unsigned16		block_index,
*               :     Unsigned16		block_size,
*               :     Unsigned8			*p_out_buf,
*               :     Unsigned16		*p_out_len
*               : );
* Description   : Get 1 block data of buffer
* Arguments     : void *buffer,                    : [In] pointer to buffer list
*               : attr_access_t *p_attr_access     : [In] Access method option
*               : Unsigned16 block_index           : [In] Block index in buffer
*               : Unsigned16 block_size            : [In] Block size in byte
*               : Unsigned8 *p_out_buf             : [Out] Data pointer for get
*               : Unsigned16 *p_out_len            : [Out] length pointer for get
* Function Calls:
* Return Value  : Unsigned8
******************************************************************************/
Unsigned8 R_OBIS_BufferBlockGet(
	void				*buffer,
	attr_access_t		*p_attr_access,
	Unsigned16			block_index,
	Unsigned16			block_size,
	Unsigned8			*p_out_buf,
	Unsigned16			*p_out_len
)
{
	/* Result of response, def. is not Matched */
	Unsigned8 response_result = TYPE_UNMATCH;
	Unsigned8			*p_temp_buf;

	/* Params check */
	if (p_out_buf == NULL ||
		p_out_len == NULL)
	{
		return response_result;
	}

	*p_out_len = 0;

	if (p_attr_access == NULL)
	{
		if (buffer != NULL)
		{
			p_temp_buf = (Unsigned8 *)buffer;
			p_temp_buf += (block_size * block_index);
			memcpy(p_out_buf, p_temp_buf, block_size);
		}
		else
		{
			return response_result;
		}
	}
	else
	{
		switch (p_attr_access->method)
		{
			case ACCESS_MTD_MEM:
				p_temp_buf = (Unsigned8 *)buffer;
				p_temp_buf += (block_size * block_index);
				memcpy(p_out_buf, p_temp_buf, block_size);
				break;
			case ACCESS_MTD_E2PR:
				if(1)//if (read_eeprom(p_attr_access->addr + (block_index * block_size), p_out_buf, block_size) != E2PR_OK)
				{
					return RD_WRT_DENIED;
				}
				break;
			case ACCESS_MTD_USERFUNC:
				/* TODO: append code here */
				break;
			default:
				break;
		}
	}

	/* Success */
	*p_out_len = block_size;

	return DATA_ACCESS_RESULT_SUCCESS;
}

/******************************************************************************
* Function Name : R_OBIS_BufferBlockSet
* Interface     : Unsigned8 R_OBIS_BufferBlockSet(
*               :     void				*buffer,
*               :     attr_access_t		*p_attr_access,
*               :     Unsigned16		block_index,
*               :     Unsigned16		block_size,
*               :     Unsigned8			*p_out_buf,
*               :     Unsigned16		*p_out_len
*               : );
* Description   : Set to 1 block data of buffer
* Arguments     : void *buffer,                    : [In] pointer to buffer list
*               : attr_access_t *p_attr_access     : [In] Access method option
*               : Unsigned16 block_index           : [In] Block index in buffer
*               : Unsigned16 block_size            : [In] Block size in byte
*               : Unsigned8 *p_in_buf              : [In] Data pointer for set
*               : Unsigned16 *p_in_len             : [In] Data pointer for set
* Function Calls:
* Return Value  : Unsigned8
******************************************************************************/
Unsigned8 R_OBIS_BufferBlockSet(
	void				*buffer,
	attr_access_t		*p_attr_access,
	Unsigned16			block_index,
	Unsigned16			block_size,
	Unsigned8			*p_in_buf,
	Unsigned16			*p_in_len
)
{
	/* Result of response, def. is not Matched */
	Unsigned8 response_result = TYPE_UNMATCH;
	Unsigned8 *p_temp_buf;

	/* Params check */
	if (p_in_buf == NULL ||
		p_in_len == NULL)
	{
		return response_result;
	}

	*p_in_len = 0;

	if (p_attr_access == NULL)
	{
		if (buffer != NULL)
		{
			p_temp_buf = (Unsigned8 *) buffer;
			p_temp_buf += (block_index * block_size);
			memcpy(p_temp_buf, p_in_buf, block_size);
		}
		else
		{
			return response_result;
		}
	}
	else
	{
		switch (p_attr_access->method)
		{
			case ACCESS_MTD_MEM:
				p_temp_buf = (Unsigned8 *) buffer;
				p_temp_buf += (block_index * block_size);
				memcpy(p_temp_buf, p_in_buf, block_size);
				break;
			case ACCESS_MTD_E2PR:
				if(1)//if (write_eeprom(p_attr_access->addr + (block_index * block_size), p_in_buf, block_size) != E2PR_OK)
				{
					return RD_WRT_DENIED;
				}
				break;
			case ACCESS_MTD_USERFUNC:
				/* TODO: append code here */
				break;
			default:
				break;
		}
	}

	/* Success */
	*p_in_len = block_size;

	return DATA_ACCESS_RESULT_SUCCESS;
}

/******************************************************************************
* Function Name : R_OBIS_BufferScanByUserFunc
* Interface     : Unsigned8 R_OBIS_BufferScanByUserFunc(
*               :     buffer_t            *buf,
*               :     Unsigned8           selective_access,
*               :     Unsigned16          master_index,
*               :     Unsigned16          child_index,
*               :     Unsigned8           *p_selector_buffer
*               : );
* Description   : Scan to get information before filter buffer
* Arguments     : buffer_t            *buf;					:[In] Pointer to buffer_t
*               : Unsigned8           selective_access,		:[In] Selective access
*               : Unsigned16          master_index,			:[In] Id of the row in master list
*               : Unsigned16          child_index,			:[In] Id of the row in child list
*               : Unsigned8           *p_selector_buffer	:[In] Pointer to the selector (from Client)
* Function Calls: TBD
* Return Value  : Unsigned8
******************************************************************************/
Unsigned8 R_OBIS_BufferScanByUserFunc (
	buffer_t            *buf,
	Unsigned8           selective_access,
	Unsigned16          master_index,
	Unsigned16          child_index,
	Unsigned8           *p_selector_buffer
)
{
	/* Result of response, def. is not Matched */
	Unsigned8 response_result = TYPE_UNMATCH;
	/*
	 * Temporary if (0) to prevent compile error
	 * when no class is used
	 */
	if (0)
	{
		/* Do nothing */
	}
#if (defined(USED_CLASS_07) && USED_CLASS_07 == 1)
	else if (master_index == 7)
	{
		response_result = R_OBIS_Class07_BufferScan(
								buf,								/* [In] Pointer to buffer_t                      */
								selective_access,					/* [In] Selective access                         */
								child_index,						/* [In] Id of the row in child list              */
								p_selector_buffer					/* [In] Pointer to the selector (from Client)    */
							);
	}
#endif
#if (defined(USED_CLASS_26) && USED_CLASS_26 == 1)
	else if (master_index == 26)
	{
		response_result = R_OBIS_Class26_BufferScan(
								buf,								/* [In] Pointer to buffer_t                      */
								selective_access,					/* [In] Selective access                         */
								child_index,						/* [In] Id of the row in child list              */
								p_selector_buffer					/* [In] Pointer to the selector (from Client)    */
							);
	}
#endif
	else
	{
		/* Do nothing */
	}
	return response_result;
}

/******************************************************************************
* Function Name : R_OBIS_BufferFilterByUserFunc
* Interface     : Unsigned8 R_OBIS_BufferFilterByUserFunc(
*               :     buffer_t            *buf,
*               :     Unsigned16          master_index,
*               :     Unsigned16          child_index,
*               :     Unsigned16          request_entries_nr,
*               :     Unsigned8           *p_out_buf,
*               :     Unsigned16          *p_out_len
*               : );
* Description   : Encoded a part of buffer
* Arguments     : buffer_t            *buf;							[In] Pointer to buffer_t
*               : Unsigned16          master_index,					[In] [In] Id of the row in master list
*               : Unsigned16          child_index,					[In] [In] Id of the row in child list
*               : Unsigned16          request_entries_nr,			[In] Number of entries need to filter
*               : Unsigned8           *p_out_buf,					[Out] Data pointer for get
*               : Unsigned16          *p_out_len 					[Out] Data length pointer for get
* Function Calls: TBD
* Return Value  : Unsigned8
******************************************************************************/
Unsigned8 R_OBIS_BufferFilterByUserFunc (
	buffer_t            *buf,
	Unsigned16          master_index,
	Unsigned16          child_index,
	Unsigned16          request_entries_nr,
	Unsigned8           *p_out_buf,
	Unsigned16          *p_out_len
)
{
	/* Result of response, def. is not Matched */
	Unsigned8 response_result = TYPE_UNMATCH;
	/*
	 * Temporary if (0) to prevent compile error
	 * when no class is used
	 */
	if (0)
	{
		/* Do nothing */
	}
#if (defined(USED_CLASS_07) && USED_CLASS_07 == 1)
	else if (master_index == 7)
	{
		response_result = R_OBIS_Class07_BufferFilter(
								buf,								/* [In] Pointer to buffer_t                */
								child_index,						/* [In] Child ID                           */
								request_entries_nr,					/* [In] Number of entries need to filter   */
								p_out_buf,							/* [Out] Data pointer for get              */
								p_out_len							/* [Out] Data length pointer for get       */
							);
	}
#endif
#if (defined(USED_CLASS_26) && USED_CLASS_26 == 1)
	else if (master_index == 26)
	{
		response_result = R_OBIS_Class26_BufferFilter(
								buf,								/* [In] Pointer to buffer_t                */
								child_index,						/* [In] Child ID                           */
								request_entries_nr,					/* [In] Number of entries need to filter   */
								p_out_buf,							/* [Out] Data pointer for get              */
								p_out_len							/* [Out] Data length pointer for get       */
							);
	}
#endif
	else
	{
		/* Do nothing */
	}
	return response_result;
}

#if (defined(USED_CLASS_07) && USED_CLASS_07 == 1)
/******************************************************************************
* Function Name : R_OBIS_Class07_ScanByRange
* Interface     : Unsigned8 R_OBIS_Class07_ScanByRange(
*               :     Integer16               restricting_object_id,
*               :     Unsigned16               child_index,
*               :     Unsigned8               *p_selector_buffer,
*               : );
* Description   : Scan the entries list from Range to Entries
* Arguments     : Integer16  restricting_object_id : [In] Restricting object id
*               : Unsigned16 child_index           : [In] Id of the row in child list
*               : Unsigned8  *p_selector_buffer    : [In] Pointer to the selector (from Client)
* Function Calls: TBD
* Return Value  : Unsigned8
******************************************************************************/


Unsigned8 R_OBIS_Class07_ScanByRange(
	Integer16               restricting_object_id,
	Unsigned16              child_index,
	Unsigned8               *p_selector_buffer
)
{
	/* Result of response, def. is not Matched */
	Unsigned8 response_result = TYPE_UNMATCH;
	date_time_t	from_date_time;
	date_time_t	to_date_time;
	Apprtc rtc;


	/* Vars to decode selective access selector */
//	void        *pbuffer = NULL;
//	Unsigned16  i, length;
//	Unsigned16  actual_index;
	Unsigned16  decoded_length;
//	Unsigned16  current_entry, entries_in_use, max_entries;
	class07_child_record_t  *p_child_record;
	buffer_t                *buf;
	Unsigned32  			range_end_entry, range_start_entry;


	 /* Find OBIS object in child table by
	 * visibility, obis code, access right.
	 */
	p_child_record = (class07_child_record_t *)R_OBIS_FindChildRecordByIndex(7, child_index);

	/* Params check */
	if (restricting_object_id <  0    ||
		restricting_object_id >= *p_child_record->p_capture_obj_length ||
		p_selector_buffer     == NULL)
	{
		return response_result;
	}

	/* Get buffer pointer */
	buf = (buffer_t *)p_child_record->p_buff;

		
		
	R_USER_WDT_Restart();
	/* decode from_date_time */
	decoded_length = R_OBIS_DecodeAttribute(
		(Unsigned8 *)&from_date_time,			/* value_buf */
		sizeof(date_time_t),					/* value_buf_len */
		ATTR_TYPE_DATE_TIME,					/* attr_type */
		p_selector_buffer,						/* buf */
		OBIS_SERVICE_DATA_BUFFER_LENGTH			/* buf_len */
	);
	p_selector_buffer += decoded_length;

	/* decode to_date_time */
	decoded_length = R_OBIS_DecodeAttribute(
		(Unsigned8 *)&to_date_time,				/* value_buf */
		sizeof(date_time_t),					/* value_buf_len */
		ATTR_TYPE_DATE_TIME,					/* attr_type */
		p_selector_buffer,						/* buf */
		OBIS_SERVICE_DATA_BUFFER_LENGTH			/* buf_len */
	);
	p_selector_buffer += decoded_length;	


	/* Block Load Profile */
	if (child_index == 2)
	{
		
		getByRange(2,R_USER_ConvertrDLMStoAppRTC(&from_date_time,&rtc),R_USER_ConvertrDLMStoAppRTC(&to_date_time,&rtc),&range_start_entry,&range_end_entry);
	}
	/* Daily Load profile */
	else if (child_index == 4)
	{
		getByRange(4,R_USER_ConvertrDLMStoAppRTC(&from_date_time,&rtc),R_USER_ConvertrDLMStoAppRTC(&to_date_time,&rtc),&range_start_entry,&range_end_entry);
	}

	buf->to_entry   = range_end_entry;
	buf->from_entry = range_start_entry;
	response_result = DATA_ACCESS_RESULT_SUCCESS;
	

	return response_result;
}

/******************************************************************************
* Function Name : R_OBIS_Class07_BufferScan
* Interface     : Unsigned8 R_OBIS_Class07_BufferScan(
*               :     buffer_t            *buf,
*               :     Unsigned8           selective_access,
*               :     Unsigned16          child_index,
*               :     Unsigned8           *p_selector_buffer
*               : );
* Description   : Scan to get info for filter
* Arguments     : void                *buffer;				:[In] Pointer to buffer_t
*               : Unsigned8           selective_access,		:[In] Selective access
*               : Unsigned16          child_index,			:[In] Id of the row in child list
*               : Unsigned8           *p_selector_buffer	:[In] Pointer to the selector (from Client)
* Function Calls: TBD
* Return Value  : Unsigned8
******************************************************************************/
Unsigned8 R_OBIS_Class07_BufferScan (
	buffer_t            *buf,
	Unsigned8           selective_access,
	Unsigned16          child_index,
	Unsigned8           *p_selector_buffer
)
{
	/* Result of response, def. is not Matched */
	Unsigned8 response_result = TYPE_UNMATCH;

	/* Vars value after scan */
	Unsigned16 					i;
	Unsigned16					entries_in_use;
	void 						*pbuffer = NULL;
	class07_child_record_t		*p_child_record;
	Unsigned16					decoded_length;

	/* Params check */
	if (buf == NULL)
	{
		return response_result;
	}

	/*
	 * Find OBIS object in child table by
	 * visibility, obis code, access right.
	 */
	p_child_record = (class07_child_record_t *)R_OBIS_FindChildRecordByIndex(7, child_index);

	/* Get current info of buffer */
	entries_in_use = (Unsigned16)*p_child_record->p_entries_in_use;

	/* Default: select all entries, columns */
	buf->from_entry = 1;
	buf->to_entry   = entries_in_use;
	buf->from_value = 1;
	buf->to_value   = *p_child_record->p_capture_obj_length; /* All column */

	buf->async_remain_entries   = entries_in_use;

	if (selective_access == 0)
	{
		/* select all entries, columns */
		return DATA_ACCESS_RESULT_SUCCESS;
	}
	else if (selective_access == 1)
	{
	/* check the access selector */
		if (*(p_selector_buffer) == 0x01)		/* by range */
		{
			/* Vars to decode data from selector buffer */
			Integer16 restricting_object_id = -1;
			/*
			* Decode the range descriptor
			*
			* range_descriptor ::= structure
			* {
			*     restricting_object: capture_object_definition
			*     from_value : CHOICE(string), CHOICE(Integer32) or CHOICE(Integer32)
			*     from_value : CHOICE(string), CHOICE(Integer32) or CHOICE(Integer32)
			*     selected_values : array(null)
			* }
			*/

			/* Skip the access selector: p_selector_buffer++ */
			/* Skip ATTR_TYPE_STRUCTURE, Length: p_selector_buffer += 2 */
			p_selector_buffer += 3;

			/* Decode 4 elements */
			{
				/*
				* 1. restricting_object: capture_object_definition
				*
				* capture_object_definition ::= structure
				* {
				*     class_id: long-unsigned
				*     logical_name: octet-string
				*     attribute_index: integer
				*     data_index: long-unsigned
				* }
				*/
				/* Skip ATTR_TYPE_STRUCTURE, Length: p_selector_buffer += 2 */
				p_selector_buffer += 2;
				{
					/* 1. Skip class_id (ATTR_TYPE_LONG_UNSIGNED): p_selector_buffer += 3  */
					/* 2. Point to logical_name */
					/* Skip ATTR_TYPE_OCTET_STRING, Length: p_selector_buffer += 2 */
					p_selector_buffer += 5;

					pbuffer = p_selector_buffer;

					/* Skip logical_name: p_selector_buffer += 6 */
					/* 3. Skip attribute_index (ATTR_TYPE_INTEGER): p_selector_buffer += 2 */
					/* 4. Skip data_index (ATTR_TYPE_LONG_UNSIGNED): p_selector_buffer += 3 */
					p_selector_buffer += 11;
				}

				/*
				* Seach the type which indicated
				* in capture_object_definition
				*/
				for (i = 0; i < *p_child_record->p_capture_obj_length; i++)
				{
					if (R_OBIS_CompareObisCode(
							(Unsigned8 *)pbuffer,
							p_child_record->p_capture_objects[i].logical_name
						) == 1)
					{
						restricting_object_id = (Integer16)i;
						break;
					}
				}

				/* when not found, return as not filter any thing */
				if (i >= *p_child_record->p_capture_obj_length)
				{
					buf->from_entry = 0;
					buf->to_entry   = 0;
				}
				else
				{
					/* Filter the entry list based on selected capture object
					* Decode from_entry, to_entry, from_value, to_value
					*/
					response_result = R_OBIS_Class07_ScanByRange(
						restricting_object_id,          /* [In] Restricting object id                 */
						child_index,                    /* [In] Id of the row in child list           */
						p_selector_buffer               /* [In] Pointer to the selector (from Client) */
					);
				}
			}
		}
		else if (*(p_selector_buffer) == 0x02)	/* by entry */
		{
			/*
			* Decode the entry descriptor
			*
			* entry_descriptor ::= structure
			* {
			*     from_entry : double-long-unsigned
			*     to_entry : double-long-unsigned
			*     from_selected_value : long-unsigned
			*     to_selected_value : long-unsigned
			* }
			*/
			/* Skip the access selector: p_selector_buffer++ */
			/* Skip ATTR_TYPE_STRUCTURE, Length: p_selector_buffer += 2 */
			p_selector_buffer += 3;

			/* 1. from_entry : double-long-unsigned */
			decoded_length = R_OBIS_DecodeAttribute(
				(Unsigned8 *)&buf->from_entry,				/* value_buf */
				4,											/* value_buf_len */
				ATTR_TYPE_DOUBLE_LONG_UNSIGNED,				/* attr_type */
				p_selector_buffer,							/* buf */
				OBIS_SERVICE_DATA_BUFFER_LENGTH				/* buf_len */
			);
			p_selector_buffer += decoded_length;

			/* 2. to_entry : double-long-unsigned */
			decoded_length = R_OBIS_DecodeAttribute(
				(Unsigned8 *)&buf->to_entry,			/* value_buf */
				4,										/* value_buf_len */
				ATTR_TYPE_DOUBLE_LONG_UNSIGNED,			/* attr_type */
				p_selector_buffer,						/* buf */
				OBIS_SERVICE_DATA_BUFFER_LENGTH			/* buf_len */
			);
			p_selector_buffer += decoded_length;

			/* 3. from_selected_value : long-unsigned */
			decoded_length = R_OBIS_DecodeAttribute(
				(Unsigned8 *)&buf->from_value,			/* value_buf */
				2,										/* value_buf_len */
				ATTR_TYPE_LONG_UNSIGNED,				/* attr_type */
				p_selector_buffer,						/* buf */
				OBIS_SERVICE_DATA_BUFFER_LENGTH			/* buf_len */
			);
			p_selector_buffer += decoded_length;

			/* 4. to_selected_value : long-unsigned */
			decoded_length = R_OBIS_DecodeAttribute(
				(Unsigned8 *)&buf->to_value,			/* value_buf */
				2,										/* value_buf_len */
				ATTR_TYPE_LONG_UNSIGNED,				/* attr_type */
				p_selector_buffer,						/* buf */
				OBIS_SERVICE_DATA_BUFFER_LENGTH			/* buf_len */
			);
			p_selector_buffer += decoded_length;
		}

		/*
		* exceptions for to_entry
		* when its value is 0, select to highest
		*/
		if (buf->to_entry == 0)
		{
			if(buf->from_entry == 0)
			{
				/* exceptions for no entry case */
				buf->async_remain_entries = 0;
				return DATA_ACCESS_RESULT_SUCCESS;
			}
			else
			{
				buf->to_entry = entries_in_use;
			}
		}
		/*
		* exceptions for to_value
		* when its value is 0, select to highest
		*/
		if (buf->to_value == 0)
		{
			buf->to_value = (Unsigned16) *p_child_record->p_capture_obj_length;
		}
		
		if (buf->from_value == 0)
		{
			buf->from_value = 1;
		}

		/* Success*/
		response_result = DATA_ACCESS_RESULT_SUCCESS;

		if (buf->from_entry == 0 || 
			buf->to_entry < buf->from_entry ||
			buf->from_value == 0 || 
			buf->to_value < buf->from_value)
		{
			/* Error */
			return SCOPE_ACCESS_VIOLATED;
		}

		/* Calculate Number of entries need to filter */
		buf->async_remain_entries = (Unsigned16)(buf->to_entry - buf->from_entry + 1);
	}

	return response_result;
}

/******************************************************************************
* Function Name : R_OBIS_Class07_BufferFilter
* Interface     : Unsigned8 R_OBIS_Class07_BufferFilter(
*               :     buffer_t            *buf,
*               :     Unsigned16          child_index,
*               :     Unsigned16          request_entries_nr,
*               :     Unsigned8           *p_out_buf,
*               :     Unsigned16          *p_out_len
*               : );
* Description   : Filter a part of buffer
* Arguments     : buffer_t            *buf;							[In] Pointer to buffer_t
*               : Unsigned16          child_index,					[In] Child ID
*               : Unsigned16          request_entries_nr,			[In] Number of entries need to filter
*               : Unsigned8           *p_out_buf,					[Out] Data pointer for get
*               : Unsigned16          *p_out_len 					[Out] Data length pointer for get
* Function Calls: TBD
* Return Value  : Unsigned8
******************************************************************************/
Unsigned8 R_OBIS_Class07_BufferFilter (
	buffer_t            *buf,
	Unsigned16          child_index,
	Unsigned16          request_entries_nr,
	Unsigned8           *p_out_buf,
	Unsigned16          *p_out_len
)
{
	/* Vars to access to entry */
	Unsigned16					total_entry;
	Unsigned16					index;
	Integer32					async_index;
	Unsigned16					entries_in_use, max_entries, capture_obj_length;
	Unsigned16					length = 0;
	class07_child_record_t		*p_child_record;
	Unsigned8					temp_len;
	Unsigned8 					*p_buf_entry=(Unsigned8*)&class07_bufferUnion;

	/*
	 * Find OBIS object in child table by
	 * visibility, obis code, access right.
	 */
	p_child_record = (class07_child_record_t *)R_OBIS_FindChildRecordByIndex(7, child_index);

	/* Get current info of buffer */
	
	entries_in_use         = (Unsigned16)*p_child_record->p_entries_in_use;
	max_entries            = (Unsigned16)*p_child_record->p_profile_entries;
	capture_obj_length     = (Unsigned16)*p_child_record->p_capture_obj_length;

	/* Start encoding */
	*p_out_len = 0;

	if (request_entries_nr == 0)
	{
		/* Null array */
		*p_out_buf++ = ATTR_TYPE_ARRAY;
		*p_out_buf++ = 0;
		*p_out_len   = 2;

		return DATA_ACCESS_RESULT_SUCCESS;
	}

	/* Check range */
	if (buf->from_entry > entries_in_use)
	{
		/* Over-range: customize HERE, default: return error  */
		return TYPE_UNMATCH;
	}
	if (buf->to_entry > entries_in_use)
	{
		/* Over-range: customize HERE, select to highest  */
		buf->to_entry = entries_in_use;
		buf->async_remain_entries = (Unsigned16)(buf->to_entry - buf->from_entry + 1);
	}

	/* Calculate asyn offset for this transfer  */
	async_index = buf->to_entry - buf->from_entry + 1; /* Max entries */
	async_index -= buf->async_remain_entries;

	if (async_index < 0)
	{
		/* Error, need to debug here */
		return TYPE_UNMATCH;
	}

	/******************************************************************************
	* Determine this is first transfer or not ?
	* by check: async_index == 0 ?
	******************************************************************************/
	if (async_index == 0)
	{
		if ((capture_obj_length < buf->to_value) || (capture_obj_length < buf->from_value))
		{
			/* Over-range: customize HERE, default: return error  */
			return TYPE_UNMATCH;
		}

		/* encode the array header */
		*p_out_buf++ = ATTR_TYPE_ARRAY;
		temp_len = BERLengthEncode(p_out_buf, buf->async_remain_entries);
		p_out_buf += temp_len;
		*p_out_len  += temp_len + 1;

		if (child_index == 2)
		{
			/* Mark to start async service, pause the capturing */
			R_OBIS_Class07_BlockLoadUpdate_Pause();
		}
		else
		{
			/* TODO : start async service for other objects */
		}
	}
	/******************************************************************************
	 * Encode for object of class 07
	 * Based on following info:
	 * 1. buf->from_entry,
	 * 2. buf->to_entry,
	 * 3. buf->from_value,
	 * 4. buf->to_value
	 ******************************************************************************/
	/* access to the list of selected entries (FIFO) */
	//index  = ((current_entry + max_entries) - entries_in_use) + (Unsigned16)(buf->from_entry + async_index) - 1;
	//index  %= (max_entries);
	
	//index  = ((current_entry + max_entries) - entries_in_use) + (Unsigned16)(buf->from_entry + async_index) - 1;
	//index  %= (max_entries); USER
	index  =  (Unsigned16)(buf->from_entry + async_index) - 1;
	
	/* Content of array */
	total_entry = request_entries_nr;
	while (total_entry > 0)
	{
		switch(child_index)
		{
			case 0: /* Instantaneous parameters snapshot */
			{
				R_OBIS_Class07_GetInstantEntry(&class07_bufferUnion.g_Class07_InstantBuffer);
				length = R_OBIS_Class07_FilterOneProfileEntry(buf, child_index, p_out_buf, p_out_len,p_buf_entry,(Unsigned8*)PARA_RAM_SIZE_INSTANT);
			}
			break;
			case 2: /* Block Load Profile */
				R_OBIS_Class07_GetBlockLoadEntry(&class07_bufferUnion.g_Class07_BlockLoadBuffer,index);
				length = R_OBIS_Class07_FilterOneProfileEntry(buf, child_index, p_out_buf, p_out_len,p_buf_entry,(Unsigned8*)PARA_RAM_SIZE_BLOCKLOAD);
			break;
			case 4: /* Daily Load Scaler Profile */
				R_OBIS_Class07_GetDailyLoadEntry(&class07_bufferUnion.g_Class07_DailyLoadBuffer,index);
				length = R_OBIS_Class07_FilterOneProfileEntry(buf, child_index, p_out_buf, p_out_len,p_buf_entry,(Unsigned8*)PARA_RAM_SIZE_DAILYLOAD);
			break;
			case 6: /* Billing profile */
				R_OBIS_Class07_GetBillingEntry(&class07_bufferUnion.g_Class07_BillingBuffer,index);
				length = R_OBIS_Class07_FilterOneProfileEntry(buf, child_index, p_out_buf, p_out_len,p_buf_entry,(Unsigned8*)PARA_RAM_SIZE_BILLING);
			break;
			case 9:  /* Event 00 profile */
			case 10: /* Event 01 profile */
			case 11: /* Event 02 profile */
			case 12: /* Event 03 profile */
			case 13: /* Event 04 profile */
			case 14: /* Event 05 profile */
				R_USER_Class07_GetEventEntry(child_index,&class07_bufferUnion.g_Class07_EventBuffer,index);
				length = R_OBIS_Class07_FilterOneProfileEntry(buf, child_index, p_out_buf, p_out_len,p_buf_entry,(Unsigned8*)PARA_RAM_SIZE_EVENT);
			break;
			case 1: /* Instantaneous Scaler Profile */
			case 3: /* Block Load Scaler Profile */
			case 5: /* Daily Load Scaler Profile */
			case 7: /* Billing Scaler Profile */
			case 8: /* Event 00 Scaler Profile */
			{
				length = R_OBIS_Class07_FilterOneScalerEntry(buf, index,
											*p_child_record->p_capture_obj_length,
											p_out_buf, p_out_len);
			}
			break;
			case 15: /* Name plate profile */
			{
				length = R_OBIS_Class07_FilterOneNamePlateEntry(buf, index, p_out_buf, p_out_len);
			}
			break;
			
			default:
			{
				/* TODO : encode for other objects */
				return TYPE_UNMATCH;
			}
//			break;
		}

		/* count down entries counter */
		total_entry--;
		/* next entry */
		p_out_buf += length;
		index  = (index + 1) % max_entries;
	}

	/******************************************************************************
	* Determine this is last transfer or not ?
	* by check: request_entries_nr >= async_remain_entries
	******************************************************************************/
	if (request_entries_nr >= buf->async_remain_entries)
	{
		if (child_index == 2)
		{
			/* Last transfer, can continue capturing */
			R_OBIS_Class07_BlockLoadUpdate_Start();
		}
		else
		{
			/* TODO : start async service for other objects */
		}
	}

	return DATA_ACCESS_RESULT_SUCCESS;
}
#endif /* (defined(USED_CLASS_07) && USED_CLASS_07 == 1) */

#if (defined(USED_CLASS_26) && USED_CLASS_26 == 1)
/******************************************************************************
* Function Name : R_OBIS_Class26_BufferScan
* Interface     : Unsigned8 R_OBIS_Class26_BufferScan(
*               :     buffer_t            *buf,
*               :     Unsigned8           selective_access,
*               :     Unsigned16          child_index,
*               :     Unsigned8           *p_selector_buffer
*               : );
* Description   : Scan to get info for filter
* Arguments     : void                *buffer;				:[In] Pointer to buffer_t
*               : Unsigned8           selective_access,		:[In] Selective access
*               : Unsigned16          child_index,			:[In] Id of the row in child list
*               : Unsigned8           *p_selector_buffer	:[In] Pointer to the selector (from Client)
* Function Calls: TBD
* Return Value  : Unsigned8
******************************************************************************/
Unsigned8 R_OBIS_Class26_BufferScan (
	buffer_t            *buf,
	Unsigned8           selective_access,
	Unsigned16          child_index,
	Unsigned8           *p_selector_buffer
)
{
	/* Result of response, def. is not Matched */
	Unsigned8 response_result = TYPE_UNMATCH;

	/* Vars value after scan */
	Unsigned16 					i;
	Unsigned32					offset_value = 0;
	Unsigned16					count_value = 0;
	Unsigned8					len_value = 0;
	Unsigned32					entries_in_use;
	void 						*pbuffer = NULL;
	class26_child_record_t				*p_child_record;
	Unsigned16					decoded_length;

	/* Params check */
	if (buf                   == NULL)
	{
		return response_result;
	}

	/*
	 * Find OBIS object in child table by
	 * visibility, obis code, access right.
	 */
	p_child_record = (class26_child_record_t *)R_OBIS_FindChildRecordByIndex(26, child_index);

	/* Default: select all entries */
	buf->from_entry = 1;						/* Offset */
	buf->to_entry   = *p_child_record->p_len;	/* Offset + Count */

	buf->async_remain_entries   = (Unsigned16)*p_child_record->p_len;

	if (selective_access == 0)
	{
		/* select all entries, columns */
		return DATA_ACCESS_RESULT_SUCCESS;
	}
	else if (selective_access == 1)
	{
		/* check the access selector */
		if (*(p_selector_buffer) == 0x01)		/* by offset_access */
		{
			/*
			* Decode the offset_selector descriptor
			*
			* offset_selector ::= structure
			* {
			*     Offset:  double-long-unsigned
			*     Count:  long-unsigned
			* }
			*/
			/* Skip the access selector */
			p_selector_buffer++;
			/* Skip ATTR_TYPE_STRUCTURE, Length */
			p_selector_buffer += 2;
			/* 1. Offset:  double-long-unsigned */
			decoded_length = R_OBIS_DecodeAttribute(
				(Unsigned8 *)&offset_value,					/* value_buf */
				sizeof(Unsigned32),							/* value_buf_len */
				ATTR_TYPE_DOUBLE_LONG_UNSIGNED,				/* attr_type */
				p_selector_buffer,							/* buf */
				OBIS_SERVICE_DATA_BUFFER_LENGTH				/* buf_len */
			);
			p_selector_buffer += decoded_length;

			/* 2.  Count:  long-unsigned */
			decoded_length = R_OBIS_DecodeAttribute(
				(Unsigned8 *)&count_value,					/* value_buf */
				sizeof(Unsigned16),							/* value_buf_len */
				ATTR_TYPE_LONG_UNSIGNED,					/* attr_type */
				p_selector_buffer,							/* buf */
				OBIS_SERVICE_DATA_BUFFER_LENGTH				/* buf_len */
			);
			p_selector_buffer += decoded_length;
		}
		else if (*(p_selector_buffer) == 0x02)	/* by index_selector */
		{
			/*
			* Decode the index_selector descriptor
			*
			* entry_descriptor ::= structure
			* {
			*     Index:  array of long-unsigned
			*     Count:  long-unsigned
			* }
			*/
			/* Skip the access selector */
			p_selector_buffer++;
			/* Skip ATTR_TYPE_STRUCTURE, Length */
			p_selector_buffer += 2;
			/* 1. Index:  array of long-unsigned */
			/* Skip ATTR_TYPE_ARRAY */
			p_selector_buffer ++;
			/* Get Length */
			len_value = *p_selector_buffer++;

			for (i = 0; i < len_value; i++)
			{
				offset_value += *p_selector_buffer++;
			}
			/* 2. Count:  long-unsigned */
			decoded_length = R_OBIS_DecodeAttribute(
				(Unsigned8 *)&count_value,				/* value_buf */
				sizeof(Unsigned32),						/* value_buf_len */
				ATTR_TYPE_DOUBLE_LONG_UNSIGNED,			/* attr_type */
				p_selector_buffer,						/* buf */
				OBIS_SERVICE_DATA_BUFFER_LENGTH			/* buf_len */
			);
			p_selector_buffer += decoded_length;
		}
		else
		{
			/* Do nothing */
			count_value = 0;
		}

		buf->from_entry = offset_value + 1;						/* Offset */
		buf->to_entry   = buf->from_entry + count_value;		/* Offset + Count */
	}
	return DATA_ACCESS_RESULT_SUCCESS;
}

/******************************************************************************
* Function Name : R_OBIS_Class26_BufferFilter
* Interface     : Unsigned8 R_OBIS_Class26_BufferFilter(
*               :     buffer_t            *buf,
*               :     Unsigned16          child_index,
*               :     Unsigned16          request_entries_nr,
*               :     Unsigned8           *p_out_buf,
*               :     Unsigned16          *p_out_len
*               : );
* Description   : Filter a part of buffer
* Arguments     : buffer_t            *buf;							[In] Pointer to buffer_t
*               : Unsigned16          child_index,					[In] Child ID
*               : Unsigned16          request_entries_nr,			[In] Number of entries need to filter
*               : Unsigned8           *p_out_buf,					[Out] Data pointer for get
*               : Unsigned16          *p_out_len 					[Out] Data length pointer for get
* Function Calls: TBD
* Return Value  : Unsigned8
******************************************************************************/
Unsigned8 R_OBIS_Class26_BufferFilter (
	buffer_t            *buf,
	Unsigned16          child_index,
	Unsigned16          request_entries_nr,
	Unsigned8           *p_out_buf,
	Unsigned16          *p_out_len
)
{
     /* Vars to access to entry */
     Unsigned16                    index, i;
     Integer32                    async_index;
     class26_child_record_t        *p_child_record;
     Unsigned8                    temp_ret;
     Unsigned16                    temp_len;

     /*
      * Find OBIS object in child table by
      * visibility, obis code, access right.
      */
     p_child_record = (class26_child_record_t *)R_OBIS_FindChildRecordByIndex(26, child_index);

     /* Start encoding */
     *p_out_len = 0;

     if (request_entries_nr == 0)
     {
         /* Null octect string */
         *p_out_buf++ = ATTR_TYPE_OCTET_STRING;
         *p_out_buf++ = 0;
         *p_out_len   = 2;

         return DATA_ACCESS_RESULT_SUCCESS;
     }

     /* Calculate asyn offset for this transfer  */
     async_index = (buf->to_entry - buf->from_entry + 1)- buf->async_remain_entries; /* Max entries */

     if (async_index < 0)
     {
         /* Error, need to debug here */
         return TYPE_UNMATCH;
     }
     else if (async_index == 0)
     {
         /* First transfer */
         /* encode the octect string header */
         *p_out_buf++ = ATTR_TYPE_OCTET_STRING;
         *p_out_buf++ = (Unsigned8)buf->async_remain_entries;
         *p_out_len  += 2;
     }

     /* access to the list of selected entries (FIFO) */
     /* index  = buf->from_entry + ((buf->to_entry - buf->from_entry + 1)- buf->async_remain_entries) */
     index  =  (Unsigned16)(buf->to_entry - buf->async_remain_entries + 1);

     /* Content of array */
     for (i = 0; i < request_entries_nr; i++)
     {
         temp_ret = R_OBIS_BufferBlockGet(
             p_child_record->p_buff->p_buffer,    	/* [In] pointer to buffer list    */
             p_child_record->p_buff->p_access,    	/* [In] Access method option    */
             index,                    				/* [In] Block index in buffer    */
             1,                    					/* [In] Block size in byte */
             p_out_buf,                				/* [Out] Data pointer for get    */
             &temp_len                				/* [Out] length pointer for get    */
             );
         if (temp_ret != DATA_ACCESS_RESULT_SUCCESS)
         {
             return temp_ret;
         }
         *p_out_len  += temp_len;
         p_out_buf += temp_len;
         index++;
     }
     /* Successful */
     return DATA_ACCESS_RESULT_SUCCESS;
}
#endif /* (defined(USED_CLASS_26) && USED_CLASS_26 == 1) */
