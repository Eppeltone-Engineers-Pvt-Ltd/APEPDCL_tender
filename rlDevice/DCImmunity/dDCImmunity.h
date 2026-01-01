#ifndef DEV_DC_IMU_H
#define DEV_DC_IMU_H

#include "..\\Include\\dTypedef.h"

# define METER_ENABLE_DC_IMUNITY_DETECTION_ON_SAMPLE (1)

#if (defined(METER_ENABLE_DC_IMUNITY_DETECTION_ON_SAMPLE) && (METER_ENABLE_DC_IMUNITY_DETECTION_ON_SAMPLE == 1))
	extern volatile uint8_t g_dc_immunity_state;
	extern volatile int16_t g_dc_detected_count;
//#define DC_UMMINITY_CURRENT_LEVEL09A		9000U
//#define DC_UMMINITY_APPLY_CURRENT_LEVEL09A	10605U


#define DC_DETECTION_COUNT_THRESHOLD        (12000)       /* 256us per detection routine (in DSAD interrupt)
                                                           * Need 6s delay means ~24000 count. 
                                                           * But in normal signal condition, this count would start from negative, 
                                                           * So choosing half of neccessary count is enough
                                                           */
void doDCImmunityAction(void);	
#endif
#endif