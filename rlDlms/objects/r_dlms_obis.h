/******************************************************************************
  Copyright (C) 2011 Renesas Electronics Corporation, All Rights Reserved.
*******************************************************************************
* File Name    : r_dlms_obis.h
* Version      : 1.00
* Description  :
******************************************************************************
* History : DD.MM.YYYY Version Description
*         : 21.02.2013
******************************************************************************/

#ifndef _R_DLMS_OBIS_H
#define _R_DLMS_OBIS_H

/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#include "..\\r_dlms_typedef.h"				/* DLMS Type Definitions */
#include "r_dlms_obis_ic.h"				/* DLMS OBIS IC Definitions */
#include "..\\meter_app\\r_dlms_data_ic.h"				/* DLMS Data IC Definitions */
#include "..\\DLMS_User.h"					/* DLMS User Definitions */

/******************************************************************************
Macro definitions
******************************************************************************/
/* For service buffer */
#define OBIS_SERVICE_DATA_BUFFER_LENGTH		(MAX_SERVER_BUFFER_SIZE - 16)		/* number of bytes for service buffer */

/* Definition of service code */
#define OL_GET_REQUEST				0xC0
#define OL_SET_REQUEST				0xC1
#define OL_EVENT_NOTIFY_REQUEST		0xC2
#define OL_ACTION_REQUEST			0xC3
#define OL_GET_REQUEST_NORMAL		0xC001
#define OL_SET_REQUEST_NORMAL		0xC101
#define OL_REQ_SERVICE_NORMAL		0x01
#define OL_REQ_SERVICE_BLOCK		0x02
#define OL_REQ_SERVICE_LIST			0x03

/* Object visibility */
#define OBIS_AR_OBJ_IS_INVISIBLE 			(0)			/* Object is invisible         */
#define OBIS_AR_OBJ_IS_VISIBLE 				(1)			/* Object is visible           */

/* For attributes access right encoding */
#define OBIS_AR_ATTR_NO_ACCESS				(0)			/* No access                   */
#define OBIS_AR_ATTR_READ					(1)			/* Read only                   */
#define OBIS_AR_ATTR_WRITE					(2)			/* Write only                  */
#define OBIS_AR_ATTR_READ_WRITE				(3)			/* Read & Write                */
#define OBIS_AR_ATTR_AUTHEN_READ			(4)			/* Authentication read         */
#define OBIS_AR_ATTR_AUTHEN_WRITE			(5)			/* Authentication write        */
#define OBIS_AR_ATTR_AUTHEN_READ_WRITE		(6)			/* Authentication Read & Write */

/* For method access right encoding */
#define OBIS_AR_METHOD_NO_ACCESS			(0)			/* No access                   */
#define OBIS_AR_METHOD_ACCESS				(1)			/* Access                      */
#define OBIS_AR_METHOD_AUTHEN_ACCESS		(2)			/* Authentication access       */

/* Macro for conformance_t definition */
#define CONFORMANCE_BYTE0_NONE						(0x00)	/* All byte 0 are marked as none support for all  */
#define CONFORMANCE_BYTE0_RESERVED0					(0x80)	/* Bit 00 (Reserved 0)                            */
#define CONFORMANCE_BYTE0_RESERVED1					(0x40)  /* Bit 01 (Reserved 1)                            */
#define CONFORMANCE_BYTE0_RESERVED2					(0x20)  /* Bit 02 (Reserved 2)                            */
#define CONFORMANCE_BYTE0_SN_READ					(0x10)	/* Bit 03 read                 (SN Referencing)   */
#define CONFORMANCE_BYTE0_SN_WRITE					(0x08)  /* Bit 04 write                (SN Referencing)   */
#define CONFORMANCE_BYTE0_SN_UNCONFIRMED_WRITE		(0x04)  /* Bit 05 unconfirmed-write    (SN Referencing)   */
#define CONFORMANCE_BYTE0_RESERVED3					(0x02)  /* Bit 06 (Reserved 3)                            */
#define CONFORMANCE_BYTE0_RESERVED4					(0x01)  /* Bit 07 (Reserved 4)                            */

#define CONFORMANCE_BYTE1_NONE						(0x00)	/* All byte 1 are marked as none support for all  */
#define CONFORMANCE_BYTE1_LN_ATTR0_SET				(0x80)	/* Bit 08 attribute0-supported-with-set (LN)      */
#define CONFORMANCE_BYTE1_LN_PRIORITY_MGMT			(0x40)  /* Bit 09 priority_mgmt_supported       (LN)      */
#define CONFORMANCE_BYTE1_LN_ATTR0_GET				(0x20)  /* Bit 10 attribute0-supported-with-get (LN)      */
#define CONFORMANCE_BYTE1_BLOCK_TRANSFER_GET		(0x10)  /* Bit 11 block-transfer-with-get                 */
#define CONFORMANCE_BYTE1_BLOCK_TRANSFER_SET		(0x08)  /* Bit 12 block-transfer-with-set                 */
#define CONFORMANCE_BYTE1_LN_BLOCK_TRANSFER_ACTION	(0x04)  /* Bit 13 block-transfer-with-action    (LN)      */
#define CONFORMANCE_BYTE1_MULTI_REFERENCES			(0x02)  /* Bit 14 multiple-references                     */
#define CONFORMANCE_BYTE1_SN_INFORMATION_REPORT		(0x01)  /* Bit 15 information-report   (SN Referencing)   */

#define CONFORMANCE_BYTE2_NONE						(0x00)	/* All byte 2 are marked as none support for all  */
#define CONFORMANCE_BYTE2_LN_RESERVED5				(0x80)	/* Bit 16 (Reserved 5)                            */
#define CONFORMANCE_BYTE2_LN_RESERVED6				(0x40)  /* Bit 17 (Reserved 6)                            */
#define CONFORMANCE_BYTE2_SN_PARAMETERIZED_ACCESS	(0x20)  /* Bit 18 parameterized-access (SN Referencing)   */
#define CONFORMANCE_BYTE2_LN_GET					(0x10)  /* Bit 19 get                           (LN)      */
#define CONFORMANCE_BYTE2_LN_SET					(0x08)  /* Bit 20 set                           (LN)      */
#define CONFORMANCE_BYTE2_LN_SELECTIVE_ACCESS		(0x04)  /* Bit 21 selective-access              (LN)      */
#define CONFORMANCE_BYTE2_LN_EVENT_NOTIFICATION		(0x02)  /* Bit 22 event-notification            (LN)      */
#define CONFORMANCE_BYTE2_LN_ACTION					(0x01)  /* Bit 23 action                        (LN)      */

/* For context_id of application context name */
#define CONTEXT_ID1_LN_NO_CIPHERING					(1)		/* Logical_Name_Referencing_No_Ciphering   - context_id(1) */
#define CONTEXT_ID2_SN_NO_CIPHERING					(2)		/* Short_Name_Referencing_No_Ciphering     - context_id(2) */
#define CONTEXT_ID3_LN_WITH_CIPHERING				(3)		/* Logical_Name_Referencing_With_Ciphering - context_id(3) */
#define CONTEXT_ID4_SN_WITH_CIPHERING				(4)		/* Short_Name_Referencing_With_Ciphering   - context_id(4) */

/* For mechanism_id of mechanism context name */
#define MECHANISM_ID0_LOWEST_SECURITY				(0)		/* COSEM_lowest_level_security_mechanism_name           - mechanism_id(0) */
#define MECHANISM_ID1_LOW_SECURITY					(1)		/* COSEM_low_level_security_mechanism_name              - mechanism_id(1) */
#define MECHANISM_ID2_HIGH_SECURITY					(2)		/* COSEM_high_level_security_mechanism_name             - mechanism_id(2) */
#define MECHANISM_ID3_HIGH_SECURITY_MD5				(3)		/* COSEM_high_level_security_mechanism_name_using_MD5   - mechanism_id(3) */
#define MECHANISM_ID4_HIGH_SECURITY_SHA1			(4)		/* COSEM_high_level_security_mechanism_name_using_SHA-1 - mechanism_id(4) */
#define MECHANISM_ID5_HIGH_SECURITY_GMAC			(5)		/* COSEM_High_Level_Security_Mechanism_Name_Using_GMAC  - mechanism_id(5) */

/******************************************************************************
Typedef definitions
******************************************************************************/

/* Access right */
typedef struct tag_access_right_t
{
	Unsigned32 attributes;
	Unsigned16 methods;
} access_right_t;

/* A associated partners information */
typedef struct tag_associated_partners_t
{
	Integer8			client_SAP;			 /* Client Service Access Point (SAP)       */
	Unsigned16			server_SAP;			 /* Server Service Access Point             */
} associated_partners_t;

/* Conformance block of association */
/* Bit field info is from LSB first */
typedef union tag_conformance_t
{
	struct
	{
		Unsigned8	bytes0;								/* byte 0 of conformance block                    */
		Unsigned8	bytes1;								/* byte 0 of conformance block                    */
		Unsigned8	bytes2;								/* byte 0 of conformance block                    */
	} bytes;

	struct
	{
		Unsigned8	reserved4                     :1;	/* Bit 07 (Reserved 4)                            */
		Unsigned8	reserved3                     :1;	/* Bit 06 (Reserved 3)                            */
		Unsigned8	unconfirmed_write             :1;	/* Bit 05 unconfirmed-write    (SN Referencing)   */
		Unsigned8	write                         :1;	/* Bit 04 write                (SN Referencing)   */
		Unsigned8	read                          :1;	/* Bit 03 read                 (SN Referencing)   */
		Unsigned8	reserved2                     :1;	/* Bit 02 (Reserved 2)                            */
		Unsigned8	reserved1                     :1;	/* Bit 01 (Reserved 1)                            */
		Unsigned8	reserved0                     :1;	/* Bit 00 (Reserved 0)                            */

		Unsigned8	information_report            :1;	/* Bit 15 information-report   (SN Referencing)   */
		Unsigned8	multiple_references           :1;	/* Bit 14 multiple-references                     */
		Unsigned8	block_transfer_with_action    :1;	/* Bit 13 block-transfer-with-action    (LN)      */
		Unsigned8	block_transfer_with_set       :1;	/* Bit 12 block-transfer-with-set                 */
		Unsigned8	block_transfer_with_get       :1;	/* Bit 11 block-transfer-with-get                 */
		Unsigned8	attribute0_supported_with_get :1;	/* Bit 10 attribute0-supported-with-get (LN)      */
		Unsigned8	priority_mgmt_supported       :1;	/* Bit 09 priority_mgmt_supported       (LN)      */
		Unsigned8	attribute0_supported_with_set :1;	/* Bit 08 attribute0-supported-with-set (LN)      */

		Unsigned8	action                        :1;	/* Bit 23 action                        (LN)      */
		Unsigned8	event_notification            :1;	/* Bit 22 event-notification            (LN)      */
		Unsigned8	selective_access              :1;	/* Bit 21 selective-access              (LN)      */
		Unsigned8	set                           :1;	/* Bit 20 set                           (LN)      */
		Unsigned8	get                           :1;	/* Bit 19 get                           (LN)      */
		Unsigned8	parameterized_access          :1;	/* Bit 18 parameterized-access (SN Referencing)   */
		Unsigned8	reserved6                     :1;	/* Bit 17 (Reserved 6)                            */
		Unsigned8	reserved5                     :1;	/* Bit 16 (Reserved 5)                            */
	} bits;
} conformance_t;

typedef enum tag_association_status_t
{
	ASSC_STATUS_NON_ASSOCIATED = 0,				/* non-associated */
	ASSC_STATUS_ASSOCIATION_PENDING,			/* association pending */
	ASSC_STATUS_ASSOCIATED						/* associated */
} association_status_t;

typedef enum tag_authentication_status_t
{
	AUTHEN_STATUS_PENDING = 0,					/* pending */
	AUTHEN_STATUS_SUCCESS,						/* authentication success */
	AUTHEN_STATUS_FAILURE,						/* authentication failure */
} authentication_status_t;

/* child record for 1 object definition */
typedef struct tag_child_record_t
{
	Unsigned8		logical_name[6];	/* Logical name (OBIS code) of the object. */
	access_right_t	*p_access_right;	/* Access right definition for 1 object    */
}
child_record_t,
class12_child_record_t,
class25_child_record_t,
class42_child_record_t,
class43_child_record_t,
class44_child_record_t,
class45_child_record_t,
class46_child_record_t,
class48_child_record_t,
class50_child_record_t,
class51_child_record_t,
class52_child_record_t,
class53_child_record_t,
class55_child_record_t,
class56_child_record_t,
class57_child_record_t,
class58_child_record_t,
class59_child_record_t,
class67_child_record_t,
class72_child_record_t,
class73_child_record_t,
class74_child_record_t,
class80_child_record_t,
class81_child_record_t,
class82_child_record_t,
class83_child_record_t,
class84_child_record_t,
class85_child_record_t,
class86_child_record_t,
class90_child_record_t,
class91_child_record_t,
class92_child_record_t;

/* child record for 1 data object definition */
typedef struct tag_class01_child_record_t
{
	Unsigned8		logical_name[6];	/* Logical name (OBIS code) of the object. */
	access_right_t	*p_access_right;	/* Access right definition for 1 object    */
	dyn_value_t		value;				/* Value                                   */
} class01_child_record_t;

/* child record for 1 register object definition */
typedef struct tag_class03_child_record_t
{
	Unsigned8		logical_name[6];	/* Logical name (OBIS code) of the object. */
	access_right_t	*p_access_right;	/* Access right definition for 1 object    */
	dyn_value_t		value;				/* Value                                   */
	scaler_unit_t	scaler_unit;		/* Scaler, unit                            */
} class03_child_record_t;

/* child record for 1 extended object definition */
typedef struct tag_class04_child_record_t
{
	Unsigned8			logical_name[6];		/* Logical name (OBIS code) of the object. */
	access_right_t		*p_access_right;		/* Access right definition for 1 object    */
	dyn_value_t			value;					/* Value                                   */
	scaler_unit_t		scaler_unit;			/* Scaler, unit                            */
	dyn_status_t		status;					/* Status                                  */
	dyn_date_time_t		capture_time;			/* Capture time buffer                     */
} class04_child_record_t;

/* child record for 1 demand object definition */
typedef struct tag_class05_child_record_t
{
	Unsigned8			logical_name[6];		/* Logical name (OBIS code) of the object. */
	access_right_t		*p_access_right;		/* Access right definition for 1 object    */
	dyn_value_t			current_average_value;	/* Current average value                   */
	dyn_value_t			last_average_value;		/* Last average value                      */
	scaler_unit_t		scaler_unit; 			/* Scaler, unit                            */
	dyn_status_t		status; 				/* Status                                  */
	dyn_date_time_t		capture_time;			/* Capture time buffer                     */
	dyn_date_time_t		start_time_current; 	/* Start time current buffer               */
	Unsigned32			*p_period;				/* Period                                  */
	Unsigned16			*p_number_of_periods;	/* Number of periods                       */
} class05_child_record_t;

/* child record for 1 register activation definition */
typedef struct tag_class06_child_record_t
{
	Unsigned8			logical_name[6];		/* Logical name (OBIS code) of the object. */
	access_right_t		*p_access_right;		/* Access right definition for 1 object    */
	obj_def_t			*p_register_assignment;	/* Register_assignment                     */
	Unsigned8			nr_register_assignment;	/* Register_assignment number              */
	register_act_mask_t *p_mask_list;			/* Mask_list                               */
	Unsigned8			nr_mask_list;			/* Mask_list number                        */
	dyn_value_t			active_mask;			/* Active_mask                             */
} class06_child_record_t;

/* child record for 1 profile generic object definition */
typedef struct tag_class07_capture_object_t
{
	Unsigned8		logical_name[6];	/* Logical name (OBIS code) of the object. */
	Unsigned16		class_id;			/* Class ID                                */
	Unsigned16		attr_id;			/* Attr ID                                 */
	Unsigned16		data_id;			/* Data ID                                 */
} class07_capture_object_t;

typedef struct tag_class07_child_record_t
{
	Unsigned8					logical_name[6];			/* Logical name (OBIS code) of the object. */
	access_right_t				*p_access_right;			/* Access right definition for 1 object    */
	buffer_t					*p_buff;					/* Buffer                                  */
	class07_capture_object_t	*p_capture_objects;			/* Capture object list                     */
	Unsigned16					*p_capture_obj_length;		/* Capture object list length              */
	Unsigned32					*p_capture_period;			/* Capture period                          */
	sort_method_t				*p_sort_method;				/* Sort method                             */
	class07_capture_object_t	*p_sort_object;				/* Sort object                             */
	Unsigned32					*p_entries_in_use;			/* Entry in use                            */
	Unsigned32					*p_profile_entries;			/* Profile entries                         */
} class07_child_record_t;

/* child record for 1 clock object definition */
typedef struct tag_class08_child_record_t
{
	Unsigned8		logical_name[6];				/* Logical name (OBIS code) of the object. */
	access_right_t	*p_access_right;				/* Access right definition for 1 object    */
	dyn_date_time_t	time;							/* time                                    */
	Integer16		*p_time_zone;					/* time_zone                               */
	dyn_status_t	status;							/* status                                  */
	date_time_t		*p_daylight_savings_begin;		/* daylight_savings_begin                  */
	date_time_t		*p_daylight_savings_end;		/* daylight_savings_end                    */
	Integer8		*p_daylight_savings_deviation;	/* daylight_savings_deviation              */
	Unsigned8		*p_daylight_savings_enabled;	/* daylight_savings_enabled                */
	clock_base_t	*p_clock_base;					/* clock_base                              */
} class08_child_record_t;

/* child record for 1 scrip table object definition */
typedef struct tag_class09_child_record_t
{
	Unsigned8		logical_name[6];				/* Logical name (OBIS code) of the object. */
	access_right_t	*p_access_right;				/* Access right definition for 1 object    */
	script_t		*p_scripts;						/* Specifies the different scripts         */
	Unsigned8		nr_scripts;						/* Number of scripts                       */
} class09_child_record_t;

/* child record for 1 schedule object definition */
typedef struct tag_class10_child_record_t
{
	Unsigned8					logical_name[6];	/* Logical name (OBIS code) of the object. */
	access_right_t				*p_access_right;	/* Access right definition for 1 object    */
	schedule_table_entry_t		*p_entries;			/* Specifies the different entries         */
	Unsigned8					*p_nr_entries;		/* Number of entries                       */
} class10_child_record_t;

/* child record for 1 special days table object definition */
typedef struct tag_class11_child_record_t
{
	Unsigned8				logical_name[6];		/* Logical name (OBIS code) of the object. */
	access_right_t			*p_access_right;		/* Access right definition for 1 object    */
	spec_day_entry_t		*p_entries;				/* Specifies the different entries         */
	Unsigned8				*p_nr_entries;			/* Number of entries                       */
} class11_child_record_t;

/* child record for 1 association LN object definition */
typedef struct tag_class15_child_record_t
{
	Unsigned8				logical_name[6];			/* Logical name (OBIS code) of the object.       */
	access_right_t			*p_access_right;			/* Access right definition for 1 object          */
	associated_partners_t	partners_id;				/* Associated partners id                        */
	Unsigned8				context_id;					/* Context id of application context             */
	Unsigned8				mechanism_id;				/* Mechanism id of authentication mechanism name */
	conformance_t			conformance;				/* Conformance block of association              */
	Unsigned8				dlms_version;				/* DLMS version number                           */
	Unsigned8				*p_secret_key;				/* Secret : LLS (password) and HLS (StoC)        */
	Unsigned8				*p_shared_key;				/* Shared key : HLS only                         */
	association_status_t	*p_status;					/* Association status                            */
	authentication_status_t	*p_authen_status;			/* Authentication status (HLS only)              */
	Unsigned8 				*p_security_setup_ref;		/* Security setup reference                      */
} class15_child_record_t;

/* child record for 1 association LN object definition */
typedef struct tag_class17_child_record_t
{
	Unsigned8		logical_name[6];	/* Logical name (OBIS code) of the object. */
	access_right_t	*p_access_right;	/* Access right definition for 1 object    */
	asslist_entry_t	*p_sap_assignment;	/* SAP assignment list                     */
	Unsigned8		nr_sap_assignment;	/* number of SAP assignment entries        */
} class17_child_record_t;

/* child record for 1 image transfer object definition */
typedef struct tag_class18_child_record_t
{
	Unsigned8				logical_name[6];				/* Logical name (OBIS code) of the object. */
	access_right_t			*p_access_right;				/* Access right definition for 1 object    */
	Unsigned32				*p_img_block_size;				/* Img block size                          */
	dyn_value_t				img_trans_blocks_status;		/* Img transfer blocks status              */
	dyn_value_t				img_first_not_trans_block_nr;	/* Img first not transfer block number     */
	Unsigned8				*p_img_trans_enabled; 			/* Image_transfer_enabled                  */
	dyn_value_t				img_trans_status;				/* Image_transfer_status                   */
	img_to_activate_info_t	*p_img_to_activate_info;		/* Image_to_activate_info                  */
	Unsigned8				*p_img_to_activate_info_nr;		/* Image_to_activate_info number           */
} class18_child_record_t;

/* child record for 1 IEC local port setup object definition */
typedef struct tag_class19_child_record_t
{
	Unsigned8			logical_name[6];		/* Logical name (OBIS code) of the object. */
	access_right_t		*p_access_right;		/* Access right definition for 1 object    */
	protocal_mode_t		*p_default_mode;  		/* Default mode                            */
	baud_rate_t			*p_default_baud;		/* Default baud rate                       */
	baud_rate_t			*p_prop_baud;			/* baud rate proposed by the meter         */
	response_time_t		*p_response_time; 		/* Respond time                            */
	Unsigned8			*p_device_addr;			/* Device address                          */
	Unsigned8			*p_device_addr_len;		/* Device address lenght                   */
	Unsigned8			*p_pass_p1;				/* Password 1                              */
	Unsigned8			*p_pass_p1_len;			/* Password 1  lenght                      */
	Unsigned8			*p_pass_p2;				/* Password 2                              */
	Unsigned8			*p_pass_p2_len;			/* Password 2  lenght                      */
	Unsigned8			*p_pass_w5; 			/* Password w5                             */
	Unsigned8			*p_pass_w5_len; 		/* Password w5  lenght                     */
} class19_child_record_t;

/* child record for 1 activity calendar object definition */
typedef struct tag_class20_child_record_t
{
	Unsigned8			logical_name[6];				/* Logical name (OBIS code) of the object. */
	access_right_t		*p_access_right;				/* Access right definition for 1 object */
	Unsigned8			*p_calendar_name_active;		/* Calendar name active  */
	Unsigned8			*p_calendar_name_active_len;	/* Calendar name active length */
	season_profile_t	*p_season_profile_active;		/* Season profile active */
	Unsigned8			nr_season_profile_active;		/* Number of season profile active */
	week_profile_t		*p_week_profile_active;			/* Week profile active */
	Unsigned8			nr_week_profile_active;			/* Number of week profile active */
	day_profile_t		*p_day_profile_active;			/* Day profile active */
	Unsigned8			nr_day_profile_active;			/* Number of day profile active */
	Unsigned8			*p_calendar_name_passive;		/* Calendar name passive  */
	Unsigned8			*p_calendar_name_passive_len;	/* Calendar name passive length */
	season_profile_t	*p_season_profile_passive;		/* Season profile passive */
	Unsigned8			nr_season_profile_passive;		/* Number of season profile passive */
	week_profile_t		*p_week_profile_passive;		/* Week profile passive */
	Unsigned8			nr_week_profile_passive;		/* Number of week profile passive */
	day_profile_t		*p_day_profile_passive;			/* Day profile passive */
	Unsigned8			nr_day_profile_passive;			/* Number of day profile passive */
	date_time_t			*p_activate_calendar_passive;	/* Activate_calendar_passive */
} class20_child_record_t;

/* child record for 1 register monitor object definition */
typedef struct tag_class21_child_record_t
{
	Unsigned8			logical_name[6];				/* Logical name (OBIS code) of the object. */
	access_right_t		*p_access_right;				/* Access right definition for 1 object    */
	value_t				*p_thresholds;					/* Threshold values                        */
	Unsigned8			nr_thresholds;					/* Number of threshold values              */
	value_definition_t	*monitor_value;					/* Monitor value                           */
	action_set_t		*p_actions;						/* Array of Action                         */
	Unsigned8			nr_actions;						/* Number of Action                        */
} class21_child_record_t;

/* child record for 1 single action schedule object definition */
typedef struct tag_class22_child_record_t
{
	Unsigned8				logical_name[6];				/* Logical name (OBIS code) of the object. */
	access_right_t			*p_access_right;				/* Access right definition for 1 object    */
	action_item_t			*p_executed_script; 			/* Executed script */
	single_action_t			*p_type;						/* Type */
	execution_time_date_t	*p_execution_time;				/* Execution time*/
	Unsigned8				nr_execution_time; 				/* Number of execution time*/
} class22_child_record_t;

/* child record for 1 IEC HDLC setup object definition */
typedef struct tag_class23_child_record_t
{
	Unsigned8			logical_name[6];					/* Logical name (OBIS code) of the object. */
	access_right_t		*p_access_right;					/* Access right definition for 1 object    */
	baud_rate_t			*p_comm_speed;						/* Communication speed                     */
	Unsigned8			*p_window_size_transmit; 			/* Window size transmit                    */
	Unsigned8			*p_window_size_receive;  			/* Window size receive                     */
	Unsigned16			*p_max_info_field_length_transmit;	/* Max info field length transmit speed    */
	Unsigned16			*p_max_info_field_length_receive;	/* Max info field length receive speed     */
	Unsigned16			*p_inter_octet_time_out; 			/* Inter_octet_time_out                    */
	Unsigned16			*p_inactivity_time_out; 			/* Inactivity_time_out                     */
	Unsigned16			*p_device_address; 					/* Device address                          */
} class23_child_record_t;

/* child record for 1 IEC twisted pair (1) setup object definition */
typedef struct tag_class24_child_record_t
{
	Unsigned8			logical_name[6];					/* Logical name (OBIS code) of the object. */
	access_right_t		*p_access_right;					/* Access right definition for 1 object    */
	/* For version 0*/
	Unsigned8			secondary_address[6];				/* Secondary address                       */
	dyn_value_t			fatal_error;						/* Fatal error                             */
	/* For version 1*/
	iec_working_mode_t	*p_mode;							/* Mode                                    */
	baud_rate_t			*p_comm_speed;						/* Communication speed                     */
	/* Common */
	Unsigned8			*p_primary_address_list;			/* Primary address list                    */
	Unsigned8			*p_primary_address_list_len;		/* Primary address list length             */
	Integer8			*p_tabi_list;						/* TAB(i) list                             */
	Unsigned8			*p_nr_tabi_list;					/* TAB(i) list number                      */
} class24_child_record_t;

/* child record for 1 utility tables object definition */
typedef struct tag_class26_child_record_t
{
	Unsigned8			logical_name[6];					/* Logical name (OBIS code) of the object. */
	access_right_t		*p_access_right;					/* Access right definition for 1 object    */
	Unsigned16			table_id;							/* Table ID                                */
	Unsigned32			*p_len;								/* Length                                  */
	buffer_t			*p_buff;							/* Buffer                                  */
} class26_child_record_t;

/* child record for 1 Modem configuration object definition */
typedef struct tag_class27_child_record_t
{
	Unsigned8			logical_name[6];					/* Logical name (OBIS code) of the object. */
	access_right_t		*p_access_right;					/* Access right definition for 1 object    */
	baud_rate_t			*p_comm_speed;						/* Communication speed                     */
	init_string_element_t *p_init_string;					/* Initialization_string                   */
	Unsigned8			*p_nr_init_string;					/* Initialization_string number            */
	value_t				*p_modem_profile;					/* modem profile                           */
	Unsigned8			*p_nr_modem_profile;				/* modem profile size                      */
} class27_child_record_t;

/* child record for Auto answer object definition */
typedef struct tag_class28_child_record_t
{
	Unsigned8					logical_name[6];					/* Logical name (OBIS code) of the object. */
	access_right_t				*p_access_right;					/* Access right definition for 1 object    */
	working_mode_t				*p_mode;  							/* Mode                                    */
	window_element_t			*p_listen_window; 					/* Listen window                           */
	Unsigned8					*p_nr_listen_window;				/* Listen window number                    */
	dyn_value_t					status; 							/* Status                                  */
	Unsigned8					*p_nr_calls;						/* Number of calls                         */
	nr_rings_t					*p_nr_rings;						/* Number of rings                         */
	allowed_callers_element_t	*p_allowed_caller; 					/* list_of_ allowed_ callers               */
	Unsigned8					*p_nr_allowed_caller;				/* allowed_ callers number                 */
} class28_child_record_t;

/* child record for Auto connect object definition */
typedef struct tag_class29_child_record_t
{
	Unsigned8			logical_name[6];					/* Logical name (OBIS code) of the object. */
	access_right_t		*p_access_right;					/* Access right definition for 1 object    */
	dial_mode_t			*p_mode;							/* Mode                                    */
	Unsigned8			*p_repetitions; 					/* Repetitions                             */
	Unsigned16			*p_repetition_delay;				/* Repetitions delay                       */
	window_element_t	*p_calling_window;					/* Calling window                          */
	Unsigned8			*p_nr_calling_window;				/* Calling window number                   */
	value_t				*p_destination_list;				/* Destination list                        */
	Unsigned8			*p_nr_destination_list;				/* Destination list number                 */
} class29_child_record_t;

/* child record for TCP UDP setup object definition */
typedef struct tag_class41_child_record_t
{
	Unsigned8			logical_name[6];					/* Logical name (OBIS code) of the object. */
	access_right_t		*p_access_right;					/* Access right definition for 1 object    */
	Unsigned16			*p_tcp_udp_port;					/* TCP UDP port                            */
	Unsigned8			ip_reference[6]; 					/* IP reference                            */
	Unsigned16			*p_mss;								/* Maximum Segment Size                    */
	Unsigned8			*p_nb_of_sim_conn;					/* Number of sim conn                      */
	Unsigned16			*p_inactivity_time_out;				/* Inactivity time out                     */
} class41_child_record_t;

/* child record for 1 register table object definition */
typedef struct tag_class61_child_record_t
{
	Unsigned8			logical_name[6];					/* Logical name (OBIS code) of the object. */
	access_right_t		*p_access_right;					/* Access right definition for 1 object    */
	dyn_value_t			*p_table_cell_values;				/* Table cell values                       */
	Unsigned8			*p_table_cell_values_nr;			/* Table cell number                       */
	table_cell_def_t	*p_table_cell_definition;			/* Table cell definition                   */
	scaler_unit_t		scaler_unit;						/* Scaler unit                             */
} class61_child_record_t;

/* child record for 1 status mapping object definition */
typedef struct tag_class63_child_record_t
{
	Unsigned8			logical_name[6];		/* Logical name (OBIS code) of the object. */
	access_right_t		*p_access_right;		/* Access right definition for 1 object    */
	dyn_value_t			status_word;			/* Status word                             */
	mapping_table_t		*p_mapping_table;		/* Mapping table                           */
} class63_child_record_t;

/* child record for 1 security setup object definition */
typedef struct tag_class64_child_record_t
{
	Unsigned8			logical_name[6];			/* Logical name (OBIS code) of the object. */
	access_right_t		*p_access_right;			/* Access right definition for 1 object    */
	security_policy_t	*p_security_policy;			/* Security policy                         */
	security_suit_t		*p_security_suit;			/* Security suit                           */
	Unsigned8			*p_client_title;			/* Client system title                     */
	Unsigned8			*p_server_title;			/* Server system title                     */
	
	Unsigned8			*p_master_key;				/* The master key                          */
	Unsigned8			*p_global_unicast_key;		/* Global unicast key                      */
	Unsigned8			*p_global_broadcast_key;	/* Global broadcast key                    */
	Unsigned8			*p_dedicated_unicast_key;	/* Dedicated unicast key                   */
	Unsigned8			*p_authentication_key;		/* Authentication key                      */
	Unsigned32			*p_frame_counter;			/* Frame counter for AES-GCM               */
} class64_child_record_t;

/* child record for 1 disconnect control object definition */
typedef struct tag_class70_child_record_t
{
	Unsigned8			logical_name[6];		/* Logical name (OBIS code) of the object. */
	access_right_t		*p_access_right;		/* Access right definition for 1 object    */
	dyn_value_t			output_state; 			/* Output state                            */
	dyn_value_t			control_state;			/* Control state                           */
	control_mode_t		*p_control_mode;		/* Control mode                            */
} class70_child_record_t;

/* child record for 1 limiter object definition */
typedef struct tag_class71_child_record_t
{
	Unsigned8			logical_name[6];					/* Logical name (OBIS code) of the object. */
	access_right_t		*p_access_right;					/* Access right definition for 1 object    */
	value_def_t			*p_monitored_value;					/* Monitored value                         */
	dyn_value_t			threshold_active;					/* Threshold active                        */
	value_t				threshold_normal;					/* Threshold normal                        */
	value_t				threshold_emergency; 				/* Threshold emergency                     */
	Unsigned32			*p_min_over_threshold_duration;		/* Min over threshold duration             */
	Unsigned32			*p_min_under_threshold_duration;	/* Min under threshold duration            */
	eme_profile_t		*p_eme_profile;						/* Emergency_profile                       */
	Unsigned16			*p_eme_profile_group_id_list;		/* Emergency profile group id list         */
	Unsigned8			*p_nr_eme_profile_group_id_list;	/* Emergency profile group id list number  */
	dyn_value_t			eme_profile_active; 				/* Emergency profile active                */
	action_set_t		*p_action;							/* Actions                                 */
} class71_child_record_t;

/* child record for 1 ZigBee SAS Startup object definition */
typedef struct tag_class101_child_record_t
{
	Unsigned8					logical_name[6];			/* Logical name (OBIS code) of the object. 			*/
	access_right_t				*p_access_right;			/* Access right definition for 1 object    			*/
	Unsigned16					*p_short_address;			/* Short address, known locally on network 			*/
	Unsigned8					*p_extended_pan_id;			/* 8 octets address which network known externally 	*/
	Unsigned16					*p_pan_id;					/* 16bit address which network known locally 		*/
	Unsigned32					*p_channel_mask;			/* 32bit mask for channel selection 				*/
	Unsigned8 					*p_protocol_version;		/* Version of ZigBee protocol on network 			*/
	zigbee_stack_profile_t		*p_stack_profile;			/* Identify capabilities of ZigBee stack 			*/
	Unsigned8					*p_start_up_control;		/* Commisioning state of ZigBee device 				*/
	Unsigned8					*p_trust_centre_address;	/* 8 octets address of Trust Centre 				*/
	Unsigned8					*p_link_key;				/* 16 octets key for point-to-point comms 			*/
	Unsigned8					*p_network_key;				/* 16 octets key for general comms 					*/
	Unsigned8					*p_use_insecure_join;		/* Coordinator allow insecure joining or not 		*/
} class101_child_record_t;

/* child record for 1 ZigBee SAS Join object definition */
typedef struct tag_class102_child_record_t
{
	Unsigned8			logical_name[6];					/* Logical name (OBIS code) of the object. 			*/
	access_right_t		*p_access_right;					/* Access right definition for 1 object    			*/
	Unsigned8			*p_scan_attempts;					/* Consecutive scan rejoin after losing contact 	*/
	Unsigned16			*p_time_between_scans;				/* Period (second) between consecutive scan 		*/
	Unsigned16			*p_rejoin_interval;					/* Period (second) to wait after disconnected 		*/
	Unsigned16			*p_rejoin_retry_interval;			/* Period (second) to wait after rejoin fail attempt*/
} class102_child_record_t;

/* child record for 1 ZigBee SAS APS Fragmentation object definition */
typedef struct tag_class103_child_record_t
{
	Unsigned8			logical_name[6];					/* Logical name (OBIS code) of the object. 			*/
	access_right_t		*p_access_right;					/* Access right definition for 1 object				*/
	Unsigned16			*p_aps_interframe_delay;			/* Delay (ms) between sending 2 fragmented block 	*/
	Unsigned16			*p_aps_max_window_size;				/* Num of unack frame transmit consecutively 		*/
} class103_child_record_t;

/* child record for 1 ZigBee Network Control object definition */
typedef struct tag_class104_child_record_t
{
	Unsigned8					logical_name[6];			/* Logical name (OBIS code) of the object. 			*/
	access_right_t				*p_access_right;			/* Access right definition for 1 object    			*/
	Unsigned8					*p_enable_disable_joining;	/* Devices could join network or not. 			 	*/
	Unsigned16					*p_join_timeout;			/* Period (second) coordinator permit joining 		*/
	zigbee_active_devices_t		*p_active_devices;			/* Information array of current authorised devices	*/
	Unsigned16 					*p_nr_active_devices;		/* Number of current authorised devices				*/
} class104_child_record_t;

/* child record for 1 ZigBee Tunnel Setup object definition */
typedef struct tag_class105_child_record_t
{
	Unsigned8					logical_name[6];					/* Logical name (OBIS code) of the object. 				*/
	access_right_t				*p_access_right;					/* Access right definition for 1 object    				*/
	Unsigned16					*p_max_incoming_transfer_size;		/* Max octets transfer in single TransferData command	*/
	Unsigned16					*p_max_outcoming_transfer_size;		/* Max octets transfer in single TransferData command	*/
	Unsigned8					*p_protocol_address;				/* Implementation specific, defined in companion specs	*/
	Unsigned16 					*p_close_tunnel_timeout;			/* Time (second) zigbee server wait until close tunnel	*/
} class105_child_record_t;

/* A row of a master table */
typedef struct tag_master_record_t
{
	Unsigned8			class_version;       /* Version of the class                               */
	Unsigned8           number_attrs;        /* Number of attributes of the class, as in Blue Book */
	Unsigned8           number_methods;      /* Number of methods of the class, as in Blue Book    */
	fp_distributor_t	fp_distributor;      /* Distributor function of class                      */
	Unsigned16          *child_list_size;    /* Size of the child-list                             */
	child_record_t		*child_list_pointer; /* Child-list pointer                                 */
	Unsigned16			child_item_size;	 /* Child item size                                    */
} master_record_t;

/******************************************************************************
Variable Externs
******************************************************************************/
/* Service Buffer (in RAM) */
extern Unsigned8 g_ServiceBuffer[OBIS_SERVICE_DATA_BUFFER_LENGTH];

/* Master Table (in ROM) */
extern const master_record_t g_MasterTable[];
extern const Unsigned16 g_TotalMasterTableElements;

/*
 * Total number of object for all and for each association (in RAM)
 * Be careful on this variable, READ ONLY
 * Remember to call to R_OBIS_ObjectLayerInit() before use.
 */
extern Unsigned16 g_TotalNumberObjects;
extern Unsigned16 g_TotalNumberObjectsOfAssociation[OBIS_NUMBER_ASSOCIATION];

/******************************************************************************
Functions Prototypes
******************************************************************************/

/* Object layer initialize */
void R_OBIS_ObjectLayerInit(void);

/* Compare OBIS code */
Unsigned8 R_OBIS_CompareObisCode(Unsigned8 *obis_code, Unsigned8 *input_code);

/* Get a child record, based on master index and child index */
child_record_t *R_OBIS_FindChildRecordByIndex(
	Unsigned16 master_index,
	Unsigned16 child_index
);

/* Check object visibility */
Unsigned8 R_OBIS_CheckObjectVisibility(
	Unsigned16 association_number,
	Unsigned16 master_index,
	Unsigned16 child_index
);

/* Get access right value for object */
Unsigned8 R_OBIS_GetAccessRightValue(
	req_type_t 		req_type,
	Unsigned8 		req_id,
	access_right_t 	*p_access_right
);

/* Get current connected association */
Unsigned8 R_OBIS_GetConnectedAssociation(void);

/* Reset async of Attr0 */
void R_OBIS_ResetAttr0AsyncService(void);

/* Decode an object attribute */
Unsigned8 R_OBIS_DecodeObject(
	st_Cosem_Attr_Desc 	*cosem_attr_desc,
	Unsigned8 			*pdata,
	Unsigned16 			service_code
);

#endif /* _R_DLMS_OBIS_H */

