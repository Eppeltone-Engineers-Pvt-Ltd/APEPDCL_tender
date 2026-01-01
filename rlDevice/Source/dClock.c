
#include "..\\include\\dIOdefine.h"
#include "..\\include\\dMisc.h"
#include "..\\include\\dClock.h"

void setClock(uint8_t ioption)
{
//	OSMC = 0x80;
//	CMC = 0x10;
//	CKC = 0x00;
	
//	CSS  = 0x00;
//	CSC = 0x80;
//	SCMC=0x10;
	
//	NOP_CYCLES(160);

    volatile uint32_t w_count;

    VRTCEN = 1U;
    /* Set fMX */
    CMC = _40_CGC_HISYS_OSC | _10_CGC_SYSOSC_PERMITTED | _01_CGC_SYSOSC_OVER10M;
    MSTOP = 1U;     /* X1 oscillator/external clock stopped */

    /* Set fIM */
    MIOEN = 0U;     /* middle-speed on-chip oscillator stopped */

    /* Change the waiting time according to the system */
    for (w_count = 0U; w_count <= CGC_FIMWAITTIME; w_count++)
    {
        NOP_CYCLES(1);
    }

    /* Set fPLL, PLL only available on 256kB devices and 100pin 128kB device */
    #if (defined(__CA78K0R__) && (defined(__F10NMJ_) || defined(__F10NPJ_) || defined(__F10NPG_))) || \
        (defined(__CCRL__) && (defined(__R5F10NMJIODEFINE_HEADER__) || defined(__R5F10NPJIODEFINE_HEADER__) || defined(__R5F10NPGIODEFINE_HEADER__))) ||\
        (defined(__ICCRL78__) && (defined(__IOR5F10NMJ_H__) || defined(__IOR5F10NPJ_H__) || defined(__IOR5F10NPG_H__)))
    PCKC = _00_CGC_fIH_STOP;
    DSCCTL = _00_CGC_PLL_STOP;

    /* Change the waiting time according to the system */
    for (w_count = 0U; w_count <= CGC_FPLLWAITTIME; w_count++)
    {
        NOP_CYCLES(1);
    }

    MCKC = _00_CGC_fOCO_SELECTED;
    #endif
    
    /* Set fSX */
    OSMC = _80_CGC_CLK_STOP | _00_CGC_IT_CLK_fSX_CLK;
	
    VRTCEN = 1U;    /* enables input clock supply */
	
    if (RTCPORSR == 0)
    {
		MCU_Delay(1000);
        SCMC = _10_CGC_SUB_OSC | _02_CGC_NORMAL_OSCILLATION;
        XTSTOP = 0U;    /* XT1 oscillator operating */
		
    }
    
    XT1SELDIS = 0U; /* stops clock supply */
    
    /* Set fSUB for CPU */
    SELLOSC = 0U;   /* 1: low speed on-chip (fIL) */
    /* Set fCLK */
    CSS = 0U;       /* main system clock (fMAIN) */
    /* Set fMAIN */
    MCM0 = 0U;      /* selects the main on-chip oscillator clock (fOCO) or PLL clock (fPLL) as the main system clock (fMAIN) */
    /* Set fMAIN Control */
    MCM1 = 0U;      /* high-speed on-chip oscillator clock */
    VRTCEN = 0U;

}