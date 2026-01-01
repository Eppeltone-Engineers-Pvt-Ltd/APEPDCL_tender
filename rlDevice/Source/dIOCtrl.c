#include "..\\include\\dIODefine.h"
#include "..\\Include\\dUser_includes.h"
#include "..\\include\\dTypedef.h"
#include "..\\include\\dSerial.h"
#include "..\\include\\dI2c.h"
#include "..\\include\\dIOCtrl.h"
#include "..\\include\\dMisc.h"
#include "..\\include\\dVBat.h"

#include "..\\..\\rlApplication\\Include\\AppConfig.h"

uint8_t push_button_pressed_for_nm=0;
void R_PORT_Create(void)
{
	PFSEG0=0x00;
	PFSEG1=0x00;
	PFSEG2=0x00;
	PFSEG3=0xFC;
	//PFSEG4=0x00;
	//PFSEG5=0x00;
	ADPC=0x01;
}

void init_Ports(void)
{
	
	//communication output mode
	SET_PORT_OUT_MODE(COMM_VCC_ON_DIR,COMM_VCC_ON,COMM_VCC_ON_BIT,COMM_VCC_ON_SET_OFF);
	
	// production input mode
	SET_PORT_IN_MODE(PRODUCTON_MODE_DIR,PRODUCTON_MODE_BIT);

	
	// main supply input mode
	SET_PORT_IN_MODE(MAINS_DETECT_STATUS_DIR,MAINS_DETECT_STATUS_BIT);
	

	PMK3=1;
	PIF3=0;
	PIOR0&=~0x10;
	EGP0|=0x08;
	EGN0|=0x08;
	PMK3=0;
	
	//cover open
	// cover open pin is input only , so no direction setting required
	PMK0=1;
	PIF0=0;
	EGP0|=0x01;
	EGN0|=0x01;
	PMK0=0;
	
	// push button input mode
	ISCLCD|=0x02;
	SET_PORT_IN_MODE(PUSH_BUTTON_DIR,PUSH_BUTTON_BIT);
	
	PMK1=1;
	PIF1=0;
	EGP0|=0x02;
	EGN0|=0x02;
	PMK1=0;

	
	SET_PORT_OUT_MODE(KWH_PULSE_DIR,KWH_PULSE,KWH_PULSE_BIT,KWH_PULSE_OFF);
	SET_MAG_PULL_UP	
	freePinshandler();
	
#if (defined(UART_0_ENABLE) && (UART_0_ENABLE == 0))
	OPTICL_TX&=~OPTICL_TX_BIT;
	OPTICL_TX_DIR&=~OPTICL_TX_BIT;
	
	OPTICL_RX_DIR&=~OPTICL_RX_BIT;
	OPTICL_RX&=~OPTICL_RX_BIT;
#endif

#if (defined(UART_2_ENABLE) && (UART_2_ENABLE == 0))
	RJ11_TX_DIR&=~RJ11_TX_BIT;
	RJ11_TX&=~RJ11_TX_BIT;
	
	RJ11_RX_DIR&=~RJ11_RX_BIT;
	RJ11_RX&=~RJ11_RX_BIT;
#endif
#if (defined(IRDA_TYPE_METER_AP) && (IRDA_TYPE_METER_AP == 1))
	SET_PORT_IN_MODE(EXLVD_DIR,EXLVD_BIT);
	SET_PORT_OUT_MODE(NM_DETECT_STATUS_DIR,NM_DETECT_STATUS,NM_DETECT_STATUS_BIT,NM_DETECT_SET_OFF);
	SET_NM_OFF_VCC;
	SET_PORT_OUT_MODE(BACK_LIGHT_DIR,BACK_LIGHT,BACK_LIGHT_BIT,BACK_LIGHT_SET_OFF);
	SET_BACK_LIGHT_OFF_VCC;
	SET_PORT_OUT_MODE(FREE_PIN_PORT22_STATUS_DIR,FREE_PIN_PORT22_STATUS,FREE_PIN_PORT22_STATUS_BIT,FREE_PIN_PORT22_STATUS_SET_OFF);
	FREE_PIN_PORT22_STATUS_OFF_VCC;
#endif

}

#pragma interrupt Interrupt_mains_supply(vect=INTP3,bank=RB3)
static void Interrupt_mains_supply(void)
{

}

#pragma interrupt Interrupt_Push_Button(vect=INTP1,bank=RB3)
static void Interrupt_Push_Button(void)
{
	//SWITCH_ON_PULSE_LED	
	if(GET_PUSH_BUTTON_STATUS)
		push_button_pressed_for_nm=1;
}

#pragma interrupt Interrupt_Cover_Switch(vect=INTP0,bank=RB3)
static void Interrupt_Cover_Switch(void)
{
//SWITCH_ON_PULSE_LED	
}

void freePinshandler(void)
{
	
#if (defined(PCB_VER_4_6_2) && (PCB_VER_4_6_2 == 1))
	PM2&=~0x0C;
	P2&=~0x0C;
#else
#if (defined(IRDA_TYPE_METER_AP) && (IRDA_TYPE_METER_AP == 0))
	PM2&=~0x04;
	P2&=~0x04;
#endif
#endif
	
}
