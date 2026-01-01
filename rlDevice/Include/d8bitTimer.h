#ifndef __DEV_8BIT_TIMER_H
#define __DEV_8BIT_TIMER_H
#include "..\\..\\rlDevice\\Include\\dTypedef.h"

//-------------------------------------------------
void init_Timer8bit0(uint8_t mSec);
void start_Timer8bit0(uint16_t mSec);
void pause_Timer8bit0(void);
void stop_Timer8bit0(void);

void init_Timer8bit1(uint8_t mSec);
void start_Timer8bit1(uint8_t mSec);
void pause_Timer8bit1(void);
void stop_Timer8bit1(void);
//-------------------------------------------------
#endif