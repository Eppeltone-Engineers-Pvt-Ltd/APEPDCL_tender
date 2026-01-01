#ifndef __DEV_VBAT_H
#define __DEV_VBAT_H
#include "..\\..\\rlDevice\\Include\\dTypedef.h"

#define LOW_POWER_HALT	0
#define LOW_POWER_STOP	1

void EnableVBATOperation(void);
void shutDownPheripheral(void);
void switch2Low_Power_MODE(uint8_t low_mode);
void initVBATDetc(void);
void disableVBATDetc(void);

#endif