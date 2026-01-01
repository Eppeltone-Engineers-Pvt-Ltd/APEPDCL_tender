#include "..\\include\\dIODefine.h"
#include "..\\include\\dTypedef.h"
#include "..\\include\\dUser_includes.h"

#include "dDCImmunity.h"

//-------- include app specific files----------------
#include "..\\..\\rlApplication\\Include\\AppMacros.h"
#include "..\\..\\rlApplication\\Include\\AppVariables.h"
#include "..\\..\\rlApplication\\Include\\AppConfig.h"


#if (METER_ENABLE_DC_IMUNITY_DETECTION_ON_SAMPLE == 1)
volatile uint8_t g_dc_immunity_state= 0;
volatile int16_t g_dc_detected_count = 0;
#if (defined(CT_TYPE_KASHYAP) && (CT_TYPE_KASHYAP == 1))
int16_t dcTargetRange[]={
2500,3500,5000,7500,10600,
11000
};
int16_t dcInputRange[]={
2500,3500,4500,5500,6300,
8300
};

#define MAX_CURRENT_WITH_DC 6500
#define MAX_CURRENT_WITHOUT_DC 10600
#else
int16_t dcTargetRange[]={
2500,3500,5000,6000,6000,10600,
11000
};
int16_t dcInputRange[]={
2500,3400,4500,6000,6000,6400,
8300
};

#define MAX_CURRENT_WITH_DC 6400
#define MAX_CURRENT_WITHOUT_DC 10600

#endif
#define NO_DC_INPUTS (7-1)
#endif

#if (defined(METER_ENABLE_DC_IMUNITY_DETECTION_ON_SAMPLE) && (METER_ENABLE_DC_IMUNITY_DETECTION_ON_SAMPLE == 1))
void doDCImmunityAction(void)
{
	uint8_t inputRangeIndex;
	//int16_t lineCofficent=0;
	uint16_t CurrentwithDC=Ins.EffectiveI;
	uint16_t cratio;
	//float lineslope;
	    if (g_dc_detected_count >= DC_DETECTION_COUNT_THRESHOLD)
	    {
	        g_dc_detected_count = DC_DETECTION_COUNT_THRESHOLD;
			
			cratio=Ins.EffectiveI;//(1000UL*peakI22)/Ins.EffectiveI;
			
			// find input range
			if(cratio>dcInputRange[0])
			{
				for(inputRangeIndex=0;inputRangeIndex<NO_DC_INPUTS;inputRangeIndex++)
					if((cratio>dcInputRange[inputRangeIndex])&&((cratio<=dcInputRange[inputRangeIndex+1])))
						break;
						
				//lineslope=(float)((dcInputRange[inputRangeIndex+1]-dcInputRange[inputRangeIndex]))/(dcTargetRange[inputRangeIndex+1]-dcTargetRange[inputRangeIndex]);
				//lineCofficent=dcInputRange[inputRangeIndex]-dcTargetRange[inputRangeIndex]*lineslope;
				
				//if(lineslope==0)
					//lineslope=1;
					
				//CurrentwithDC=(cratio-lineCofficent)/lineslope;
				CurrentwithDC=dcTargetRange[inputRangeIndex];
				
				
				//if((CurrentwithDC<dcTargetRange[inputRangeIndex])||(CurrentwithDC>dcTargetRange[inputRangeIndex+1]))
					//CurrentwithDC=(dcTargetRange[inputRangeIndex]+dcTargetRange[inputRangeIndex+1])/2;
				

					
				if(cratio>MAX_CURRENT_WITH_DC)
					CurrentwithDC=MAX_CURRENT_WITHOUT_DC;

				
				Ins.EffectiveI=CurrentwithDC;
				Ins.EffectiveP=(uint32_t)Ins.Voltage*Ins.EffectiveI/1000;
				Ins.AppPower=Ins.EffectiveP;
				Ins.PowerFactor=100;
				Ins.NeuCurrent=Ins.EffectiveI;
				Ins.NeuPower=Ins.EffectiveP;
				
						
			}
			
	    }
	
}

#endif

