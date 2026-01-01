#ifndef _INTERVALKWH_H
#define INTERVALKWH_H

#include "..\\..\\rlDevice\\Include\\dTypedef.h"

#define NORMAL_KWH_THRESHOLD_LL	10		// 0.1 kwh
#define NORMAL_KWH_THRESHOLD_UL	20		// 0.2 kwh
#define MAX_INTERVAL_COUNT		32 		// number of locations
#define NM_KWH_THRESHOLD_LL		200	// 2kWh max kwh limit per location
#define NM_KWH_THRESHOLD_UL		300	// 3kWh max kwh limit per location

#define MAX_KWH_PER_INTERVAL	NM_KWH_THRESHOLD_UL
#define LIMIT_DIFF_THRESHOLD	(MAX_KWH_PER_INTERVAL)


void savekWh(uint32_t kwh);
uint32_t readIntervalkWh(void);
void savekWh_NMExit(void);
//uint8_t getInterval(void);

#endif

