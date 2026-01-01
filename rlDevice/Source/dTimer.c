/*
*******************************************************************************
** Pragma directive
*******************************************************************************
*/



#include "..\\include\\dIODefine.h"
#include "..\\include\\dTypedef.h"
#include "..\\include\\dTimer.h"
#include "..\\include\\dUser_includes.h"



void EM_TIMER_Init(void)
{
	TAU0EN = 1U;		/* supplies input clock */
	TPS0 = _000F_TAU_CKM0_FCLK_15;
	/* Stop all channels */
	TT0 = _0001_TAU_CH0_STOP_TRG_ON | _0002_TAU_CH1_STOP_TRG_ON | _0004_TAU_CH2_STOP_TRG_ON | _0008_TAU_CH3_STOP_TRG_ON | _0010_TAU_CH4_STOP_TRG_ON | _0020_TAU_CH5_STOP_TRG_ON | _0040_TAU_CH6_STOP_TRG_ON | _0080_TAU_CH7_STOP_TRG_ON | _0200_TAU_CH1_H8_STOP_TRG_ON | _0800_TAU_CH3_H8_STOP_TRG_ON;
	/* Mask channel 0 interrupt */
	TMMK00 = 1U;	/* disable INTTM00 interrupt */
	TMIF00 = 0U;	/* clear INTTM00 interrupt flag */
	/* Mask channel 1 interrupt */
	TMMK01 = 1U;	/* disable INTTM01 interrupt */
	TMIF01 = 0U;	/* clear INTTM01 interrupt flag */
	/* Mask channel 1 higher 8 bits interrupt */
	TMMK01H = 1U;	/* disable INTTM01H interrupt */
	TMIF01H = 0U;	/* clear INTTM01H interrupt flag */
	/* Mask channel 2 interrupt */
	TMMK02 = 1U;	/* disable INTTM02 interrupt */
	TMIF02 = 0U;	/* clear INTTM02 interrupt flag */
	/* Mask channel 3 interrupt */
	TMMK03 = 1U;	/* disable INTTM03 interrupt */
	TMIF03 = 0U;	/* clear INTTM03 interrupt flag */
	/* Mask channel 3 higher 8 bits interrupt */
	TMMK03H = 1U;	/* disable INTTM03H interrupt */
	TMIF03H = 0U;	/* clear INTTM03H interrupt flag */
	/* Mask channel 4 interrupt */
	TMMK04 = 1U;	/* disable INTTM04 interrupt */
	TMIF04 = 0U;	/* clear INTTM04 interrupt flag */
	/* Mask channel 5 interrupt */
	TMMK05 = 1U;	/* disable INTTM05 interrupt */
	TMIF05 = 0U;	/* clear INTTM05 interrupt flag */
	/* Mask channel 6 interrupt */
	TMMK06 = 1U;	/* disable INTTM06 interrupt */
	TMIF06 = 0U;	/* clear INTTM06 interrupt flag */
	/* Mask channel 7 interrupt */
	TMMK07 = 1U;	/* disable INTTM07 interrupt */
	TMIF07 = 0U;	/* clear INTTM07 interrupt flag */
	/* Set INTTM03 level 2 priority */
	TMPR103 = 1U;
	TMPR003 = 0U;
	/* Channel 2 used as interval timer */
	TMR03 = _0000_TAU_CLOCK_SELECT_CKM0 | _0800_TAU_8BITS_MODE  | _0000_TAU_TRIGGER_SOFTWARE |_0040_TAU_TIMN_EDGE_RISING| _0000_TAU_MODE_INTERVAL_TIMER | _0000_TAU_START_INT_UNUSED;
	TDR03 = _0001_TAU_TDR03_VALUE;
	TOM0 &= ~_0008_TAU_CH3_OUTPUT_COMBIN;
	TOL0 &= ~_0008_TAU_CH3_OUTPUT_LEVEL_L;
	TO0 &= ~_0008_TAU_CH3_OUTPUT_VALUE_1;
	TOE0 &= ~_0008_TAU_CH3_OUTPUT_ENABLE;
}

void EM_TIMER_Start(void)
{
	TMIF03 = 0U;	/* clear INTTM03 interrupt flag */
	TMMK03 = 0U;	/* enable INTTM03 interrupt */
	TS0 |= _0008_TAU_CH3_START_TRG_ON;
}

void EM_TIMER_Stop(void)
{
	TT0 |= _0000_TAU_CH3_START_TRG_OFF;
	/* Mask channel 2 interrupt */
	TMMK03 = 1U;	/* disable INTTM03 interrupt */
	TMIF03 = 0U;	/* clear INTTM03 interrupt flag */
}

#pragma interrupt R_TAU0_Channel3_Interrupt(vect=INTTM03,bank=RB3)
static void R_TAU0_Channel3_Interrupt(void)
{
	/* Start user code. Do not edit comment generated here */
	EI();
	
	/* End user code. Do not edit comment generated here */
}
