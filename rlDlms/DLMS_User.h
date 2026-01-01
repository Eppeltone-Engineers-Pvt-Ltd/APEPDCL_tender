/******************************************************************************
* DISCLAIMER
* Please refer to http://www.renesas.com/disclaimer
******************************************************************************
  Copyright (C) 2008. Renesas Technology Corp., All Rights Reserved.
*******************************************************************************
* File Name    : DLMS_User.h
* Version      : 1.00
* Description  : Function protocol APIs in DLMS_Lib.lib
******************************************************************************
* History : DD.MM.YYYY Version Description
*         : 01.01.2009 1.00    First Release
******************************************************************************/

#ifndef _DLMS_H
#define _DLMS_H

/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#include "r_dlms_typedef.h"
#include "Physical\\serial.h"
#include "..\\..\\rlApplication\\Include\\AppConfig.h"

#if (defined(IS_DLMS_ENABLED_EEPL) && (IS_DLMS_ENABLED_EEPL >1))
#error "invalid macros"
#endif
/******************************************************************************
Typedef definitions
******************************************************************************/

typedef struct{
	Unsigned32 Response; 
	Unsigned32 Inactivity;
	Unsigned32 Interframe;
}st_TimeoutCount;

typedef struct{
	Unsigned8 bResponse:1;
	Unsigned8 bInactivity: 1;
	Unsigned8 bInterframe:1;
}st_TimeoutEnable;

typedef struct
{
	Unsigned8	*RxStartPosition;
	Unsigned8	*RxCurrentPosition;
	Unsigned8	*RxEndPosition;
	
	Unsigned8	*TxStartPosition;
	Unsigned8	*TxCurrentPosition;
	Unsigned8	*TxEndPosition;

}st_DATALINK_Buffer;

typedef union{
		struct{
			Unsigned8 HDLC_LengthUpper:3;
			Unsigned8 HDLC_SegBit:1;
			Unsigned8 HDLC_FrameType:4;
			Unsigned8 HDLC_LengthLower:8;
		}bits;
		Unsigned8 Byte[2];
}st_HDLC_Format;

typedef struct{
	Unsigned8	HDLC_addr[4];
	Unsigned8   ByteMode;
}st_DestinationMAC;

typedef union{	
	struct{
		Unsigned8 dummy:1;
		Unsigned8 nS:3;
		Unsigned8 P:1;
		Unsigned8 nR:3;
	}bits;
	Unsigned8 Byte;
}st_HDLC_Ctrl;

typedef struct{
	st_HDLC_Ctrl 	Ctrl;
	Integer8		Type;
}st_HDLC_CtrlType;

typedef struct
{
	Unsigned8 			*HDLC_FlagStart;
	st_HDLC_Format		HDLC_Format;
	st_DestinationMAC 	DestinationMAC;
	Unsigned8 			SourceMAC;
	st_HDLC_CtrlType 	HDLC_Ctrl;
	Unsigned16 			HDLC_HCS;
	Unsigned8 			*HDLC_Information;
	Unsigned16 			HDLC_FCS;
	Unsigned16 			dummy;
	Unsigned8 			*HDLC_FlagClose;
}st_HDLC_FrameType3;

typedef struct
{
	Unsigned8 			*HDLC_Start;
	st_HDLC_Format		*HDLC_Format;
	st_HDLC_Ctrl 		*HDLC_Ctrl;
	Unsigned8 			*HDLC_HCS;
	Unsigned8 			*HDLC_Information;
	Unsigned8 			*HDLC_FCS;
	Unsigned8 			*HDLC_End;
}st_HDLC_FrameOutput;

typedef struct{
	Unsigned8 *LogicalName;
	Unsigned16 Max_lenTran;
	Unsigned16 Max_lenRecv;
	Unsigned16 Inter_timeout;
	Unsigned16 Inactivity_timeout;
	Unsigned16 DeviceAddress;
	Unsigned16 CommSpeed;
	Unsigned16 Win_szTran;
	Unsigned16 Win_szRecv;
}st_DATALINK_PAR;

typedef union{	
	struct{
		Unsigned8 dummy:4;
		Unsigned8 z:1;
		Unsigned8 y:1;
		Unsigned8 x:1;
		Unsigned8 w:1;
	}bits;
	Unsigned8 	Byte;
}st_HDLC_Reject_wxyz;

typedef struct
{
	Unsigned16			Data_Length;
	Unsigned8			I_FrameNo;
	Unsigned8			I_FrameType;
	Unsigned8			*Data_Ptr;
}st_HDLC_Data;

typedef enum{
	PHY_PROC_NONE = 0,
	PHY_PROC_REQ,
}et_PHY_PROC;

typedef enum{
	PHY_STATE_BEGIN = 0,
	PHY_STATE_RECIEVING,
	PHY_STATE_TRANSMITING,
	PHY_STATE_READY,
}et_PHY_STATE;

typedef enum
{
	NORMAL_MESSAGE = 0,				/* There is no action on this message */
	REQUEST_MESSAGE,				/* Confirm that the received message can be checked as Requeset Message */
	HDLC_MODE_ENTERED,				/* Entered HDLC mode */
	ERROR_MESSAGE,					/* Used to send the error message when received message has wrong protocol */
	NAK_MESSAGE,					/* Used to send the NAK message the rec received information is wrong */
	ACK_MESSAGE,					/* Used to send the signal to confirm that the request, command is done */
}et_PHY_MSG_TYPE;

typedef struct
{
	Unsigned8			mode_E;
	et_PHY_PROC			phy_process;
	et_PHY_STATE		phy_state;
	et_PHY_MSG_TYPE		message_type;
	Unsigned8			*Data_Ptr;
}st_PHY_PAR;

/* Connection manager */
typedef struct {
	/* Physical channel info */
	st_PHY_PAR            PhyLink_parameter;
	Unsigned8             PhyLink_Status;
	Unsigned8             channel_id;

	Unsigned16            ByteCount;
	/* Tx Buffer */
	Unsigned8             *SerialTx_Buffer_Str;
	Unsigned8             *SerialTx_Buffer_End;

	/* Rx Buffer */
	Unsigned8             *SerialRx_Buffer_Str;
	Unsigned8             *SerialRx_Buffer_End;

	Unsigned16            SerialTx_Size;
	Unsigned16            SerialRx_Size;
	/* Timeout info */
	st_TimeoutCount       Timeout_Level;
	st_TimeoutCount       Timeout_ms;

	/* Datalink info */

	st_HDLC_Data          HDLC_Data;
	st_HDLC_FrameType3    HDLC_Frame;
	st_DATALINK_Buffer    DATALINK_Buffer;
	st_HDLC_FrameOutput   HDLC_FrameInput;		/* for length detection only */
	st_DATALINK_PAR       DATALINK_parameter;

	/* Event info */
	Unsigned16            EventFlag;

	st_TimeoutEnable      Timeout_Enable;
	Unsigned8             DATALINK_State;

	Integer8              FrameType;

	Unsigned8             Server_State;
	Unsigned8             Prev_nR;
	Unsigned8             Prev_nS;
	Unsigned8             nR;
	Unsigned8             nS;
	Unsigned8             Client_Status;
	Unsigned8             dummy;
} st_channel_info;

typedef struct
{
	Unsigned8 	Class_ID[2];
	Unsigned8 	Instance_ID[6];
	Integer16	Attr_ID;
}st_Cosem_Attr_Desc;

typedef struct
{	
	Unsigned8 	Class_ID[2];
	Unsigned8 	Instance_ID[6];
	Integer16	Method_ID;
}st_Cosem_Method_Desc;

typedef	Unsigned8 (*fn_OBIS)(
	st_Cosem_Attr_Desc *cosem_attr_desc,
	Unsigned8 *pdata,
	Unsigned16 service_code
);

typedef	void (*fn_SerialTx)(Unsigned8 channel, Unsigned8* BlockPtr, Integer16 Length);
typedef	void (*fn_SerialConfig)(Unsigned8 channel, Unsigned8 new_baud_rate, Unsigned8 new_protocol); 
typedef	void (*fn_SerialInit)(Unsigned8 channel);


typedef struct
{
	st_channel_info		*Channel_info;
	fn_SerialInit		SerialInit_FuncPtr;
	fn_SerialConfig		SerialConfig_FuncPtr;
	fn_SerialTx			SerialTx_FuncPtr;
	Unsigned8			*Server_Buffer;
	Unsigned8			*ServerPhysicalAddress;
	Unsigned8			*DeviceAddress;
	Unsigned8			*ManufacturerID;
	Unsigned8			*ID;
	Unsigned16			Server_BufferSize;
	Unsigned16			Response_Timeout;
	Unsigned16			Inactivity_Timeout;
	Unsigned16			Interframe_Timeout;
	Unsigned8			Channel_maxcount;
	Unsigned8			AdressByteMode;
	Unsigned8			IEC_BaudRate;
	Unsigned8			HDLC_BaudRate;
	Unsigned8			Windows_size;
	Unsigned8			dummy;
}st_ServerConfig;

typedef enum{
	DATALINK_STATE_BEGIN = 0,
	DATALINK_STATE_RECIEVING,
	DATALINK_STATE_TRANSMITING,
	DATALINK_STATE_READY,
}et_Datalink_State;

typedef enum{
	PHYADD_1BYTE_NOT_SUPPORTED = 0,
	PHYADD_1BYTE_SUPPORTED = 1,
}et_PhysicalAddressMode;

typedef enum
{
	DATA_ACCESS_RESULT_SUCCESS = 0,
	HARDWARE_FAULT,
	TEMP_FAIL,
	RD_WRT_DENIED,
	OBJ_UNDEFINE,
	OBJ_CLS_INCONSISTENT = 9,
	OBJ_UNAVAILABLE = 11,
	TYPE_UNMATCH,
	SCOPE_ACCESS_VIOLATED,
	DATA_BLK_UNAVAILABLE,
	LONG_GET_ABORT,
	NO_LONG_GET,
	LONG_SET_ABORT,
	NO_LONG_SET,
	DATA_ACCESS_RESULT_OTHER=250,
	DATA_ACCESS_RESULT_PARTIAL=251,
}et_DATA_ACCESS_RESULT;

typedef enum
{	
	ACTION_RESULT_SUCCESS = 0,
	ACTION_RESULT_HARDWARE_FAULT,
	ACTION_RESULT_TEMP_FAIL,
	ACTION_RESULT_RD_WRT_DENIED,
	ACTION_RESULT_OBJ_UNDEFINE,
	ACTION_RESULT_OBJ_CLS_INCONSISTENT = 9,
	ACTION_RESULT_OBJ_UNAVAILABLE = 11,
	ACTION_RESULT_TYPE_UNMATCH,
	ACTION_RESULT_SCOPE_ACCESS_VIOLATED,
	ACTION_RESULT_DATA_BLK_UNAVAILABLE,
	ACTION_RESULT_LONG_ACTION_ABORT,
	ACTION_RESULT_NO_LONG_ACTION,
	ACTION_RESULT_OTHER = 250,
}et_ACTION_RESULT;

typedef enum
{
	BAUD_RATE_300 = '0',
	BAUD_RATE_600,
	BAUD_RATE_1200,
	BAUD_RATE_2400,
	BAUD_RATE_4800,
	BAUD_RATE_9600,
	BAUD_RATE_19200,
	BAUD_RATE_NOT_SPECIFIED = 0xFF,
}et_BAUD_RATE;

typedef enum{
	IEC_PROTOCOL = 0,
	HDLC_PROTOCOL,
	PROTOCOL_NOT_SPECIFIED = 0xFF,
}et_PHY_PROTOCOL;

/******************************************************************************
Macro definitions
******************************************************************************/
/*
 Data ID - DO NOT revise - This macro sync with DLMS lib 
*/
#define COSEM_DATAID_RETURNERROR			0x00
#define COSEM_DATAID_CONNECTAASTATUS		0x01
#define COSEM_DATAID_HLSAASTATUS			0x02
#define COSEM_DATAID_CLIENTSAP				0x03
#define COSEM_DATAID_CHILDTABLE15LEN		0x04
#define COSEM_DATAID_CONTEXTID				0x05
#define COSEM_DATAID_MECHANISMID			0x06
#define COSEM_DATAID_SECRETKEY				0x07
#define COSEM_DATAID_SHAREDKEY				0x08
#define COSEM_DATAID_DLMSVERSION			0x09
#define COSEM_DATAID_CONFORMANCEBLK0		0x0a
#define COSEM_DATAID_CONFORMANCEBLK1		0x0b
#define COSEM_DATAID_CONFORMANCEBLK2		0x0c
#define COSEM_DATAID_NUMBER_ATTR_OF_CLASS	0x0d
#define COSEM_DATAID_SECURITYSETUP_ID_OF_CLASS15			0x0e
#define COSEM_DATAID_CLIENTITTLE_OF_CLASS64					0x0f
#define COSEM_DATAID_SERVERTITTLE_OF_CLASS64				0x10
#define COSEM_DATAID_SECURITYPOLICY_OF_CLASS64				0x11
#define COSEM_DATAID_DEDICATED_UNICASTKEY_OF_CLASS64		0x12
#define COSEM_DATAID_DEDICATED_UNICASTKEY_OF_CLASS64_RESET	0x13
#define COSEM_DATAID_GLOBAL_UNICASTKEY_OF_CLASS64			0x14
#define COSEM_DATAID_GLOBAL_BROADCASTKEY_OF_CLASS64			0x15
#define COSEM_DATAID_GLOBAL_AUTHKEY_OF_CLASS64				0x16
#define COSEM_DATAID_GLOBAL_SECURITYSUIT_OF_CLASS64			0x17
#define COSEM_DATAID_GLOBAL_FRAMECOUNTER_OF_CLASS64			0x18

/*
 * Mode E option value - DO NOT revise - This macro sync with DLMS lib 
 */
#define ModeE_ENABLE      			(1)
#define ModeE_DISABLE     			(0)

#ifndef HDLC_HEADER_SIZE
#define	HDLC_HEADER_SIZE  			(14)
#endif

/* Set serial buffer size here */
#define MAX_WINDOWS_SIZE			1														/* Windows size for each response I frame */
#define MAX_SEGMENT_COUNT			1														/* Segment count for each response I frame */

#if (defined(IS_DLMS_ENABLED_EEPL) && IS_DLMS_ENABLED_EEPL == 1)
#define MAX_APDU_BUFFER_SIZE		(512)
#endif
#if (defined(IS_DLMS_ENABLED_EEPL) && IS_DLMS_ENABLED_EEPL == 0)
#define MAX_APDU_BUFFER_SIZE		(128)
#endif

#define MAX_TRANSMIT_BUFFER_SIZE	(MAX_APDU_BUFFER_SIZE + HDLC_HEADER_SIZE)
#define MAX_SERVER_BUFFER_SIZE		(MAX_TRANSMIT_BUFFER_SIZE - HDLC_HEADER_SIZE)	/* In [128: 2030] byte */
#define MAX_RECIEVE_BUFFER_SIZE		(MAX_SERVER_BUFFER_SIZE + HDLC_HEADER_SIZE)

#if (MAX_APDU_BUFFER_SIZE < 128) || (MAX_APDU_BUFFER_SIZE > 2030)
		#error "MAX_APDU_BUFFER_SIZE must be in [128: 2030] byte"
#endif

/* CHANNEL ID = 0xFF: not specific or token release */
#define	CHANNEL_NOT_SPECIFIED		(0xFF) /* DO NOT revise - This macro sync with DLMS lib */

/* Max supported number of physical channel(s) */
#define	MAX_CONNMGR_CHANNEL_NUMBER	(2)
#define	CHANNEL_PRIMARY				(0x00)
#define	CHANNEL_SECONDARY			(0x01)
#define	CHANNEL_3RD					(0x02)

/*
 * AES supporting option
 */
#define AES_ENABLE      			(1)
#define SHA1_ENABLE     			(0)
#define MD5_ENABLE      			(0)

#define IEC_BAUDRATE				BAUD_RATE_300
#define HDLC_BAUDRATE				BAUD_RATE_9600

/******************************************************************************
Variable Externs
******************************************************************************/
extern 	Unsigned16			OBIS_DataLength;
extern 	const st_ServerConfig UserServerConfig;
/******************************************************************************
Functions Prototypes
******************************************************************************/
Integer8 DLMSInit(st_ServerConfig ServerConfig);
void DLMSMessageProcess(void);
void DLMSIntervalProcessing(void);

void DataLinkConnectionReset(void);
Unsigned8 COSEMOpenGetConnectedAssociation(void);
Unsigned8 COSEMOpenGetConnectingAssociationStatus(void);
Unsigned8 *COSEMOpenGetHLSCtoS(void);

void COSEMObjectLayerInit(
	Unsigned8 	*Buffer,
	Unsigned16  Buffer_Size,
	fn_OBIS		OBIS_FuncPtr
);

void DLMS_Initialize(void);
void DLMS_PollingProcessing(void);

/* BER */
Unsigned8 BERLengthEncode(Unsigned8 *p_out_buf, Unsigned16 length);
Unsigned8 BERLengthDecode(Unsigned8 *p_in_buf, Unsigned16 *p_length);

/* Connection Manager service */
Unsigned8 CONNMGR_ChannelCount(void);
Unsigned8 CONNMGR_MaxChannelNumber(void);
Unsigned8 CONNMGR_GetCurrentChannelID(void);

void CONNMGR_RegisterPriorityChannel(Unsigned8 channel_id);

Unsigned8 CONNMGR_AddChannel(
	Unsigned8             channel_id,		/* Channel ID      */
	Unsigned8             *Tx_Buffer,		/* Tx buffer start */
	Unsigned16            Tx_Buffer_Size,	/* Tx buffer size  */
	Unsigned8             *Rx_Buffer,		/* Rx buffer start */
	Unsigned16            Rx_Buffer_Size	/* Rx buffer size  */
);

void CONNMGR_ModifyEventFlag(Unsigned8 channel_id, Unsigned16 set_mask, Unsigned16 clear_mask);
void CONNMGR_ModifyAllEventFlag(Unsigned16 set_mask, Unsigned16 clear_mask);
Unsigned16 CONNMGR_GetEventFlag(Unsigned8 channel_id);

/* Physical layer service callback */
void PhysicalTimeoutCount(void);
void PhysicalReceiveCallback(Unsigned8 channel_id, Unsigned8 byte);
void PhysicalSendEndCallback(Unsigned8 channel_id);

/* Need implement in user application, call by DLMS lib */
void DataLinkConnectionReset(void);
Unsigned8* COSEM_Getinfo(Unsigned16 Class_ID, Unsigned16 index, Unsigned16 Data_ID);
void COSEM_Setinfo(Unsigned16 Class_ID, Unsigned16 index, Unsigned16 Data_ID, Unsigned8 data_value);
#define E2PR_OK 0
#define EPR_OK 0

#endif /* _DLMS_H */
