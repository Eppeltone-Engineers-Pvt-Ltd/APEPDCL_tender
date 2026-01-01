#include "..\\include\\dIODefine.h"
#include "..\\include\\dTypedef.h"
#include "..\\include\\dUser_includes.h"



void setLVD(void)
{
	
}

#pragma interrupt Interrupt_LVD(vect=INTLVI)
static void Interrupt_LVD(void)
{
	NOP();
	NOP();

}