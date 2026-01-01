#ifndef _APP_LS_H
#define _APP_LS_H

#include "..\\..\\rlDevice\\Include\\dTypedef.h"

void Load_Survey(void);
uint16_t GetLsLoadLoc(uint8_t dayno);
uint16_t GetDailyLoc(uint16_t dayno);
void doLs(void);
uint16_t getLsEntries(void);
void ClearLsData(void);
void getByRange(uint8_t obj_id, uint32_t time_start, uint32_t time_end, uint32_t *idx_start, uint32_t *idx_stop);
void logDaily00hhEnergy(void);
#endif
