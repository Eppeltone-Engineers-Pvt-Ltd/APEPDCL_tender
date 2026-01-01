#ifndef _APP_NM_TASK_H
#define _APP_NM_TASK_H

//-----------------------------------------------------
#include "..\\..\\rlDevice\\Include\\dTypedef.h"
//-----------------------------------------------------
void doDatalog_NM(void);
void initNMLs(void);
uint8_t doLsNM(void);
void doNMLsEEPROM(void);
void getNMStartDate(uint8_t*);
void getNMIntervalLS(uint8_t* buf,uint8_t interval);
void saveLSatExit(void);
#endif