//-------- include device specific files----------------
#include "..\\..\\rlDevice\\Include\\dTypedef.h"

//-------- include app specific files-------------------
#include "..\\Include\\AppLcd.h"
#include "..\\Include\\AppMacros.h"

#include "math.h"

const int8_t Sin50Hz[39]={
5,10,14,19,23,26,28,30,31,31,
31,29,27,24,21,17,12,7,2,-3,
-8,-13,-18,-22,-25,-28,-30,-32,-32,-32,
-31,-2,-27,-24,-20,-15,-11,-6,-1
};
const int8_t Cos50Hz[39]={
31,30,28,25,22,18,13,8,3,-2,
-7,-12,-16,-21,-24,-28,-30,-32,-32,-32,
-32,-30,-28,-24,-21,-17,-12,-7,-2,3,
8,13,18,22,25,28,30,31,31
	
};

const int8_t Cos250Hz[8]={
22,
0,
-23,
-32,
-23,
-1,
22,
32

};
const int8_t Sin250Hz[8]={
22,
32,
22,
0,
-23,
-32,
-23,
-1

	
};

#define SAMPLE_TOTAL_COUNT	39
#define SAMPLE_50HZ_COUNT	39
#define SAMPLE_250HZ_COUNT	8

#if 0
uint32_t getHarmonicsVal(const int16_t *samplePtr,uint8_t hzType)
{
	int8_t iTemp,one_cycle_count=0;
	
	int32_t X=0,Y=0,sample32;
	uint64_t X64,Y64;
	for(iTemp=0;iTemp<SAMPLE_TOTAL_COUNT;iTemp++)
	{
		sample32=samplePtr[iTemp];
		if(HAR_HZ_TYPE_50==hzType)
		{
			X+=(sample32*Sin50Hz[one_cycle_count]);
			Y+=(sample32*Cos50Hz[one_cycle_count]);
			one_cycle_count=(one_cycle_count+1)%SAMPLE_50HZ_COUNT;
		}
		else
		{
			
			X+=(sample32*Sin250Hz[one_cycle_count]);
			Y+=(sample32*Cos250Hz[one_cycle_count]);
			one_cycle_count=(one_cycle_count+1)%SAMPLE_250HZ_COUNT;
		}
		
		
		
		
		
	}
	X64=(int64_t)X*X;
	Y64=(int64_t)Y*Y;
	
	X=sqrt((X64+Y64)/SAMPLE_TOTAL_COUNT);
	return X;
}
#endif	

