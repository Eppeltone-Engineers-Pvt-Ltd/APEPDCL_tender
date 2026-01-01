#ifndef _APP_TAMPER_H
#define _APP_TAMPER_H

#include "..\\..\\rlDevice\\Include\\dTypedef.h"

uint8_t Check_Tamper(void);
void Tamperlog(void);
void Tamperlog_AP(void);
uint16_t getTotalTamperCount(void);
uint16_t getTotalTamperCount_AP(void);
void getEventLog(uint8_t*ptr, uint8_t tampertype,uint8_t eventno);
void getEventLog_AP(uint8_t*ptr, uint8_t tampertype,uint8_t eventno);
uint8_t getCurrentLogCount(uint8_t tamperid);
void logSWTamper(uint8_t eventid);
void getPfailLog(uint8_t*ptr,uint8_t eventno);
void getSWLog(uint8_t*ptr,uint8_t eventno);
uint8_t getEventMaxEntries(uint8_t tamperid);
void ClearCopen(void);
uint8_t getlastEventID(uint8_t eventtype);
uint16_t getEarthStatus(uint8_t threshold);
uint16_t GetTamperForwardLoc(uint8_t eventno,uint8_t tamperid);

#endif

