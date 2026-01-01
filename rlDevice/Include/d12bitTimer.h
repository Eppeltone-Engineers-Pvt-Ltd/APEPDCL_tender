#ifndef __DEV_12BIT_TIMER_H
#define __DEV_12BIT_TIMER_H

#include "..\\..\\rlDevice\\Include\\dTypedef.h"
//-------------------------------------------------
void init_Timer12bit(uint8_t mSec);
void ClearPulseAccumulation(void);
void setPulse(uint8_t setflag);
//-------------------------------------------------
#endif
