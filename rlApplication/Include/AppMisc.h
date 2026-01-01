#ifndef _APP_MISC_H
#define _APP_MISC_H

#include "..\\..\\rlDevice\\Include\\dTypedef.h"
#include "..\\..\\rlApplication\\Include\\AppVariables.h"

#define ENERGY_W_RAM	0x00
#define ENERGY_R_RAM	0x01

uint32_t ConvertTimeCounterToTime(uint32_t TimeSeconds, uint8_t Flag,Apprtc *);
uint32_t ConvertTimeToTimeCounter(Apprtc *);
uint16_t Crc_Cal(const uint8_t * str,uint8_t length);
uint16_t Crc_Cal_Advanced(const uint8_t * str,uint8_t length,uint16_t prevCRC);
void makeByte(uint32_t data,uint8_t startByte, uint8_t noOfBytes);
uint32_t getByteValue(const uint8_t *bptr,uint8_t startByte, uint8_t noOfBytes);
void SaveEEPROM(uint8_t state,uint8_t paramType);
void powerfailLogging(void);
void isRAMOK(void);
uint16_t updateMDInterval(uint8_t ioption, uint16_t md_current_value);
uint8_t isPowerfailLog(uint8_t powerMode);
uint8_t getSetPfailData(uint8_t wrOption);
void fillComBufferZero(void);
uint8_t getSelftDgFeature(void);
void Dec_Ascii(uint32_t data, uint8_t bytes, uint8_t *buff,uint8_t addDecPoint);
uint8_t getSingleActionFlag(uint8_t ioption);
void setSingleActionFlag(uint8_t ioption,uint32_t seTtime,uint8_t setflg);
uint8_t getCustomDateTimeASCIIFormat(uint32_t dtTime,uint8_t *dtbptr,uint8_t timedigits);

#endif
