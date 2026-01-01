#include "..\\include\\dIODefine.h"
#include "..\\Include\\dUser_includes.h"
#include "..\\include\\dTypedef.h"
#include "..\\include\\dDisplay.h"
#include "..\\include\\dIOCtrl.h"


//-------- include app specific files----------------
#include "..\\..\\rlApplication\\Include\\AppMacros.h"
#include "..\\..\\rlApplication\\Include\\AppVariables.h"

void init_OSD(void)
{
	OSDCEN=1;
	OSDCRES=0;
	OSDIF=0;
	OSDMK=1;
	OSDC&=~0x8000;
	OSDC=0x0FFF;
	OSDC|=0x8000;
	OSDMK=0;
	
	
}

#pragma interrupt Interrupt_OSD(vect=INTOSDC,bank=RB2)
static void Interrupt_OSD(void)
{
	SWITCH_ON_RTC_VDD;
	
	power_fail_state|=RTC_BAT_STATUS_FLAG;
	
	
}
