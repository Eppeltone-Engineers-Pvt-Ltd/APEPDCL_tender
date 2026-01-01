#ifndef _APP_MD_H
#define _APP_MD_H
#include "..\\..\\rlDevice\\Include\\dTypedef.h"

void Compute_MD(uint8_t reset_option);
void Compute_RD_MD(uint8_t reset_option);
uint16_t GetMdLoc(uint8_t eventno);
uint8_t doManBillDue2SeasonChange(void);


#endif
