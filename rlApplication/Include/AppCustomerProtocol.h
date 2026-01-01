#ifndef __APP_CUSTPROTOCOL_H
#define __APP_CUSTPROTOCOL_H

//---------- Error code --------------------------------------------------------
/*  1 -> Invalid password
    2 -> Invalid command
    3 -> Crc Mismatch
    4 -> Invalied Indentifier
    5 -> Hardware Lock
*/ 

//---------------- Communication flags -----------------------------------------
#define   PROTO_HEADER                    0
#define   PROTO_NO_BYTES                  1
#define   PROTO_IDENTIFIER                3
#define   PROTO_DATA_BYTES                5

#define PROTO_START_CHAR        0x3A
#define PROTO_CR_CHAR           0x0D
#define PROTO_LF_CHAR           0x0A

//-------------- READ COMMANDS B/W 0x01 to 0x20 --------------------------------
#define PROTO_READ_SERIAL_NO              0x41
#define PROTO_READ_MFG_ID                 0x42
#define PROTO_READ_CUM_KWH                0x43
#define PROTO_READ_TSTATUS_HISTORY        0x44
#define PROTO_READ_KWH_CON_HISTORY        0x45
#define PROTO_READ_MD_HISTORY             0x46
#define PROTO_READ_LATEST_NM_TAMPER       0x47   
#define PROTO_READ_LATEST_REV_TAMPER      0x48
#define PROTO_READ_LATEST_EARTH_TAMPER    0x49
#define PROTO_READ_LATEST_CO_TAMPER       0x4A
#define PROTO_READ_LATEST_MAG_TAMPER      0x4B
#define PROTO_READ_LATEST_ND_TAMPER       0x4C
#define PROTO_READ_LATEST_OU_TAMPER       0x4D
#define PROTO_READ_LATEST_OL_TAMPER       0x4E
#define PROTO_READ_LATEST_LU_TAMPER       0x4F
#define PROTO_READ_LATEST_FREQ_TAMPER     0x50
#define PROTO_READ_LATEST_PF_TAMPER       0x51


#define PROTO_READ_LATEST_NM_TAMPER_FIVE            0x5D
#define PROTO_READ_LATEST_REV_TAMPER_FIVE           0x5E
#define PROTO_READ_LATEST_EARTH_TAMPER_FIVE         0x5F
#define PROTO_READ_LATEST_CO_TAMPER_FIVE            0x60
#define PROTO_READ_LATEST_MAG_TAMPER_FIRST_FIVE     0x61
#define PROTO_READ_LATEST_MAG_TAMPER_SEC_FIVE       0x62
#define PROTO_READ_LATEST_MAG_TAMPER_THIRD_FIVE     0x63
#define PROTO_READ_LATEST_MAG_TAMPER_FOURTH_FIVE    0x64
//#define PROTO_READ_LATEST_ND_TAMPER                 0x65




//#define PROTO_READ_LATEST_OL_TAMPER       0x48
//#define PROTO_READ_LATEST_ND_TAMPER       0x4B
#define PROTO_READ_LATEST_OV_TAMPER       0x4D
#define PROTO_READ_LATEST_LU_TAMPER       0x4F


#define PROTO_READ_CALIB_CONST            0x5A
#define PROTO_READ_RTC_DATA               0x5C
//#define PROTO_READ_LS_DATA                0x5D
//#define PROTO_READ_DAILY_DATA             0x5E
//#define PROTO_READ_LS_ENTRIES             0x5F
//#define PROTO_READ_TOD04             	  0x60
//#define PROTO_READ_TOD58             	  0x61
//#define PROTO_ENERGY_HISTORY              0x62
//#define PROTO_TOD_HISTORY04               0x63
//#define PROTO_TOD_HISTORY58               0x64
#define PROTO_READ_FORMAT_DATA            0x65
#define PROTO_READ_SW_LOG            	  0x66

#define START_CMD_ID	PROTO_READ_SERIAL_NO		
#define END_CMD_ID		PROTO_READ_SW_LOG

uint8_t getCustomTODZonedata(uint8_t* bptr,uint8_t zone_no);
uint8_t getCustomHistorydata(uint8_t* bptr,uint8_t history_no);
uint8_t getCustomTODZoneHistorydata(uint8_t* bptr,uint8_t zone_no,uint8_t history_no);
uint8_t getCustomTamperData(uint8_t *bptr,uint8_t TamperNo,uint8_t eventno);
uint8_t getCustomTamperData_occures(uint8_t *bptr,uint8_t TamperNo,uint8_t eventno);
uint8_t getCustomTamperData_only20mag(uint8_t *bptr,uint8_t TamperNo,uint8_t eventno);
uint8_t getFormatData(uint8_t *bptr);
uint8_t getTamperCustomdata(uint8_t TamperNo,uint8_t * bptr);
void setKNO(const uint8_t * bptr);
void getKNO(uint8_t * bptr);
void setBillingDate(const uint8_t * bptr);
uint16_t getBillingDate(void);
void CustomerProtocol(void);
uint8_t ConvertPAN(const uint8_t * bptr);
uint16_t makePANByte(uint8_t byteValue);
void getCustomTODTiming(uint8_t* bptr);
uint8_t setCustomTODTiming(const uint8_t* bptr);
uint8_t getCustomSWLog(uint8_t* bptr);
#endif
