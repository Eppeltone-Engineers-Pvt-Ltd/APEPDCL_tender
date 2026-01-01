#ifndef _APP_BILLING_H
#define _APP_BILLING_H
#include "..\\..\\rlDevice\\Include\\dTypedef.h"

void CheckBillingOver(uint8_t resetType,uint32_t billdate);
uint16_t GetBillLoc(uint8_t eventno);
uint16_t GetLastBillLoc(uint8_t eventno);
void ClearMD(void);
void doBilling(void);
uint8_t getLastBillnumber(void);
uint8_t getAvgPf(void);
void initPowerOnDuration(void);
uint32_t getNextBillingDate(void);
uint32_t GetPastMD(uint8_t valueType,uint8_t EnergyType);
uint8_t getAutoBillDay(void);
void getlatestkWh_History(void);

#endif
