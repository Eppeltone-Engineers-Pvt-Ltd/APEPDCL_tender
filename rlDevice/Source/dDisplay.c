#include "..\\include\\dDisplay.h"
#include "..\\include\\dIODefine.h"
#include "..\\include\\dTypedef.h"
#include "..\\include\\dMisc.h"
#include "..\\include\\dUser_includes.h"
#include "..\\include\\dIOCtrl.h"

//-------- include app specific files----------------
#include "..\\..\\rlApplication\\Include\\AppConfig.h"
//------------------------------------------------------
#ifdef LCD_VOLTAGE_WAITTIME
	#undef 	LCD_VOLTAGE_WAITTIME
	#define LCD_VOLTAGE_WAITTIME	100
#endif

/***********************************************************************************************************************
* Function Name: R_LCD_Create
* Description  : This function initializes the LCD module.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void init_Lcd(void)
{
	
	
	volatile uint32_t wt_count;
	OSMC |= 0x80;  	/* Supply lock to LCD */
	LCDON = 0U;    	/* disable LCD clock operation */
	LCDM1 = _01_LCD_VOLTAGE_LOW;
	LCDM0 = _00_LCD_DISPLAY_WAVEFORM_A |0x15;
	LCDM0 |=_40_LCD_VOLTAGE_MODE_INTERNAL;//_80_LCD_VOLTAGE_MODE_CAPACITOR;
	//LCDM0 |=_80_LCD_VOLTAGE_MODE_CAPACITOR;
	
	
 	
	
	/* Set CAPL and CAPH pins */
	ISCLCD 	&= (uint8_t)~(_01_LCD_CAPLH_BUFFER_VALID);
	
	
	P12 	&= 0x1FU;
	PU12 	&= 0x1FU;

	/* Set segment pins */
	PFSEG0 	|= 0xF0U;
	PFSEG1 	|= 0xFFU;
#if (defined(PCB_VER_4_6_8) && (PCB_VER_4_6_8 == 1))	
	PFSEG2 	|= 0xFBU;
#elif (defined(PCB_VER_4_6_6) && (PCB_VER_4_6_6 == 1))	
	PFSEG2 	|= 0xEBU;
#else
	PFSEG2 	|= 0xE3U;
#endif	
	
	
	/* Set Port 10 - 17 port mode, port , pllup 0,output mode,input mode */
	P1		= 0x00;
	PM1		= 0x00;
	PU1 	= 0x00U;	
	POM1 	&=0x1FU;
	PIM1 	&=0x9FU;
	

	/* Set Port 70 - 81 pllup 0,output mode,input mode */
	P7		&=0xFC;
	PM7		&=0xFC;
    PU7 	&=0xFCU;

	
	LCDM1 	|= _00_LCD_DISPLAY_PATTERN_A;
	LCDC0 	=_05_LCD_CLOCK_FSX_FIL_6;// _06_LCD_CLOCK_FSX_FIL_7;
	

	/* Change the waiting time according to the system */
	MCU_Delay(LCD_REFVOLTAGE_WAITTIME);
	
	R_LCD_PowerOn();
}

/***********************************************************************************************************************
* Function Name: R_LCD_Start
* Description  : This function enables the LCD display.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_LCD_Start(void)
{
    LCDON = 1U;
}

/***********************************************************************************************************************
* Function Name: R_LCD_Stop
* Description  : This function disables the LCD display.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_LCD_Stop(void)
{
    LCDON = 0U;
	
}

/***********************************************************************************************************************
* Function Name: R_LCD_Set_VoltageOn
* Description  : This function enables voltage boost circuit or capacitor split circuit.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_LCD_Set_VoltageOn(void)
{
    volatile uint32_t wt_count;
    
    VLCON = 1U;

    /* Change the waiting time according to the system */
    for(wt_count = 0U; wt_count <= LCD_VOLTAGE_WAITTIME; wt_count++)
    {
        NOP();
    }
    
    SCOC = 1U;
}

/***********************************************************************************************************************
* Function Name: R_LCD_Set_VoltageOff
* Description  : This function disables voltage boost circuit or capacitor split circuit.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_LCD_Set_VoltageOff(void)
{
    SCOC = 0U;
    VLCON = 0U;
    LCDM0 &= (uint8_t)~_C0_LCD_VOLTAGE_MODE_INITIALVALUE;
}

/***********************************************************************************************************************
* Function Name: R_LCD_PowerDown
* Description  : This function power down LCD module
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_LCD_PowerOff(void)
{
	LCDON = 0;
	SCOC  = 0;
	VLCON = 0;
	
}

/***********************************************************************************************************************
* Function Name: R_LCD_PowerOn
* Description  : This function power down LCD module
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_LCD_PowerOn(void)
{
	R_LCD_Start();
	R_LCD_Set_VoltageOn();
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
