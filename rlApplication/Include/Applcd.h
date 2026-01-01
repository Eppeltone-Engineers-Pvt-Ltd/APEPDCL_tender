#ifndef __APP_LCD_H
#define __APP_LCD_H

#include "..\\..\\rlDevice\\Include\\dTypedef.h"
#include "AppVariables.h"
#include "AppConfig.h"

#define LCD18D	(1)
#define LCD18A	(1)
#define LCD18C	(0)

#ifdef __cplusplus
extern "C"
{
#endif

/********************************************************/
void lcd_init(void);
void Display(uint8_t isCodeType);
void PushButton(void);
void setParaBuff(uint8_t ioption);
void setNextParaSeq(void);
void CheckParaTimeOut(void);
void setParamIndex(uint8_t paramID,uint8_t isMsg);
void Clear_Lcd(void);
void MessageTimeOut(void);
uint8_t getParaCount(void);
uint8_t getParamTime(void);
uint8_t getMeterSerial(uint8_t* srBuff,uint8_t srType);
uint8_t rising_demand(uint8_t* srBuff,uint8_t srType);
uint8_t getMeterSerial_1(uint8_t* srBuff,uint8_t srType, uint8_t _loc, uint8_t _nob);
uint8_t displayRisingDemand(void);
//uint8_t getMeterSerial_2(uint8_t* srBuff,uint8_t srType, uint8_t _loc, uint8_t _nob);
uint8_t getMeterKitID(uint8_t* kiBuff,uint8_t srType);
#if (defined(IS_DISPLAY_CONFIGURABLE) && (IS_DISPLAY_CONFIGURABLE == 1))
void moveback(void);
void copyDPfromFlash2Ram(uint8_t imode);
void getConfiguredDP(uint8_t iMode);
uint8_t configDisplayPara(const uint8_t *dbuff);
uint8_t readConfiguredDP2Send(uint8_t *bptr,uint8_t imode);
void ResetConfigDP(void);
void validatelcdRAMBuff(void);
#endif




#define MSG_ALL_INDEX		0x29
#define MSG_AUTO_INDEX		0x2A
#define MSG_PUSH_INDEX		0x2B
#define MSG_DONE_INDEX		0x2C
#define MSG_FAIL_INDEX		0x2D
#define MSG_FW_INDEX		0x2E
#define MSG_ACT00_INDEX		0x2F
#define MSG_ACT01_INDEX		0x30
#define MSG_ERASE_CNT_INDEX	0x11
#define MSG_ON_INDEX		0x31
#define MSG_PULSE_INDEX		0x32
#define MSG_C_OPEN_INDEX	0x33
#define MSG_MODE1_INDEX		0x34
#define MSG_MODE2_INDEX		0x35
#define MSG_MODE3_INDEX		0x36
#define	MSG_NM_PIN_FAIL		0x3C
#define	MSG_NO_CLEAR		0x28
#define	MSG_BAT_STATUS		0x27
#define	MSG_REV_PROD		0x39
#define	MSG_DNLD_INDEX		0x92
#define	MSG_RTCBAT_STATUS	0x90
#define	MSG_BACK_INDEX		0x93
#define	MSG_FORWARD_INDEX	0x94




#define HIGH_RESO_KWH_INDEX	0x10
#define HIGH_RESO_KWH_2DEC_INDEX	0x00
#define HIGH_RESO_KWH_1DEC_INDEX	0x22

#define HIGH_RESO_KVAH_INDEX	0x19
#define HIGH_RESO_KVAH_2DEC_INDEX	0x15

#define HIGH_RESO_EXPKWH_INDEX		0x1C
#define HIGH_RESO_EXPKWH_2DEC_INDEX	0x1A

#define HIGH_RESO_EXPKVAH_INDEX	0x1D
#define HIGH_RESO_EXPKVAH_2DEC_INDEX	0x1B

#define HIGH_RESO_NONSOLAR_2DEC_INDEX	0x24
#define CUM_POWER_ON_INDEX				0x1E

#define TOD_KWH_START_INDEX		0xA0
#define TOD_KWH_END_INDEX		0xA7

#define TOD_KVAH_START_INDEX	0xA8
#define TOD_KVAH_END_INDEX		0xAF

#define TOD_KWMD_START_INDEX	0xC9
#define TOD_KWMD_END_INDEX		0xE0

#define TOD_KVAMD_START_INDEX	0xE1
#define TOD_KVAMD_END_INDEX		0xF8

#define BILL_TOD_KWH_START_INDEX	0xB8
#define BILL_TOD_KWH_END_INDEX		0xBF


#define DISPLAY_MODE_NORMAL			0x00	//auto
#define DISPLAY_MODE_PUSH			0x01	//push
#define DISPLAY_MODE_AUTO_COVER		0x02	//auto cover open
#define DISPLAY_MODE_UOUT_NORMAL	0x03	//power fail normal
#define DISPLAY_MODE_UOUT_COVER		0x04	// power fail cover open
#define DISPLAY_MODE_PRODUCTION		0x05	// production mode
#define DISPLAY_MODE_MODE3			0x06	//push in push
#define DISPLAY_MODE_NM				0x07	//nm mode display
#define DISPLAY_MODE_PROD_PUSH		0x08	//production push
//#define DISPLAY_MODE_MODE1			0x09	//auto in pusH	


#define RAM_KWH		(uint8_t *)&InsSave.CumkWh
#define RAM_VOLT    (uint8_t *)&Ins.Voltage
#define RAM_PH_I    (uint8_t *)&Ins.PhCurrent
#define RAM_NU_I    (uint8_t *)&Ins.NeuCurrent
#define RAM_EFF_I   (uint8_t *)&Ins.EffectiveI
#define RAM_PH_P    (uint8_t *)&Ins.PhPower
#define RAM_NU_P	(uint8_t *)&Ins.NeuPower
#define RAM_EFF_P   (uint8_t *)&Ins.EffectiveP	
#define RAM_S   	(uint8_t *)&Ins.AppPower
#define RAM_FREQ    (uint8_t *)&Ins.Frequency
#define RAM_PF  	(uint8_t *)&Ins.PowerFactor
#define RAM_TIME    (uint8_t *)&InsSave.timeCounter
#define RAM_DATE    (uint8_t *)&InsSave.timeCounter
#define RAM_MD_kW	(uint8_t *)&InsSave.BillMD.Current.kW	
#define RAM_kW_DATE	(uint8_t *)&InsSave.BillMD.Current.kW_Date
#define RAM_KVAH	(uint8_t *)&InsSave.CumkVAh
#define RAM_MD_kVA	(uint8_t *)&InsSave.BillMD.Current.kVA
#define RAM_kVA_DATE	(uint8_t *)&InsSave.BillMD.Current.kVA_Date
#define RAM_RD_kW	(uint8_t *)&Ins_RD_KW.RD_KW
#define RAM_RD_kVA	(uint8_t *)&Ins_RD_KW.RD_KVA

#define RAM_EXPKWH		(uint8_t *)&InsSave.ExpCumkWh
#define RAM_EXPKVAH	(uint8_t *)&InsSave.ExpCumkVAh
#define RAM_CUMPON		(uint8_t *)&InsSave.CumPowerOnDuration
#define	RAM_CUMD_kW		(uint8_t *)&InsSave.BillMD.CumkWMD
#define	RAM_RESET_CNT	(uint8_t *)&InsSave.MDResetCount
#define	RAM_CUMD_kVA	(uint8_t *)&InsSave.BillMD.CumkVAMD

#define MSG_NM		(uint8_t *)&TamperRecord.TamperCount[0]
#define MSG_ND		(uint8_t *)&TamperRecord.TamperCount[9]
#define MSG_REV		(uint8_t *)&TamperRecord.TamperCount[1]
#define MSG_MAG		(uint8_t *)&TamperRecord.TamperCount[4]
#define MSG_EAR		(uint8_t *)&TamperRecord.TamperCount[2]
#define MSG_CO		(uint8_t *)&TamperRecord.TamperCount[3]
#define MSG_STATUS	(uint8_t *)&TamperRecord.TamperStatusFlag

#if(defined(IRDA_TYPE_METER_AP) && (IRDA_TYPE_METER_AP == 1))
#define RAM_KWH_H1	(uint8_t *)&kWh_History[0]
#define RAM_KWH_H2	(uint8_t *)&kWh_History[1]
#define RAM_KWH_H3	(uint8_t *)&kWh_History[2]
#define RAM_KWH_H4	(uint8_t *)&kWh_History[3]
#define RAM_KWH_H5	(uint8_t *)&kWh_History[4]
#define RAM_KWH_H6	(uint8_t *)&kWh_History[5]
#define RAM_KW_MD_H1	(uint8_t *)&kW_MD_History[0]
#define RAM_KW_MD_H2	(uint8_t *)&kW_MD_History[1]
#define RAM_KW_MD_H3	(uint8_t *)&kW_MD_History[2]
#define RAM_KW_MD_H4	(uint8_t *)&kW_MD_History[3]
#define RAM_KW_MD_H5	(uint8_t *)&kW_MD_History[4]
#define RAM_KW_MD_H6	(uint8_t *)&kW_MD_History[5]

#endif	


#define MSG_ALL		(uint8_t *)0	
#define MSG_AUTO	(uint8_t *)0	
#define MSG_PUSH	(uint8_t *)0	
#define MSG_DONE	(uint8_t *)0
#define MSG_FAIL	(uint8_t *)0
#define MSG_FW		(uint8_t *)0
#define MSG_ACT00	(uint8_t *)0
#define MSG_ACT01	(uint8_t *)0
#define MSG_ON		(uint8_t *)0
#define MSG_PULSE	(uint8_t *)0
#define MSG_C_OPEN	(uint8_t *)0
#define MSG_MODE1	(uint8_t *)0
#define MSG_MODE2	(uint8_t *)0
#define MSG_MODE3	(uint8_t *)0
#define MSG_NO_CLR	(uint8_t *)0
#define MSG_BAT		(uint8_t *)0
#define MSG_NO_TAPER	(uint8_t *)0
#define MSG_T_NAME	(uint8_t *)0
#define MSG_BLK		(uint8_t *)0
#define	MSG_DNLD	(uint8_t *)0
#define MSG_BC		(uint8_t *)0
#define MSG_FR		(uint8_t *)0
#define MSG_MAGNET	(uint8_t *)0
#define MSG_NM_SYMBOL	(uint8_t *)0
#define MSG_REV_SYMBOL	(uint8_t *)0
#define MSG_EARTH_SYMBOL	(uint8_t *)0
#define MSG_OV_SYMBOL	(uint8_t *)0
#define MSG_OC_SYMBOL	(uint8_t *)0
#define MSG_FREQ_SYMBOL	(uint8_t *)0
#define MSG_PF_SYMBOL	(uint8_t *)0

	


	
#define EEP_DATA	(uint8_t *)0
#define RET_DATA	(uint8_t *)0




// modifier values
#define LONG_VAL        0
#define SHORT_VAL       1
#define CHAR_VAL        2
#define DATE_VAL        3
#define TIME_VAL        4 
#define MONTH_VAL       5
#define YEAR_VAL        6


#define STOP_SCRL       0x40
#define STOP_SCCRL      0x80

// Divisor
#define DIV_1           0x01
#define DIV_10          0x02
#define DIV_3600        0x04
#define DIV_100         0x08
#define DIV_1000        0x10
#define DIV_10000       0x20
#define DIV_32          0x40
//#define DIV_3600        0x80
#define DIV_320         0x80

#define NONE            0x00
#define DIGIT_1         0x01
#define DIGIT_2         0x02
#define DIGIT_3         0x03
#define DIGIT_4         0x04
#define DIGIT_5         0x05
#define DIGIT_6         0x06
#define DIGIT_8         0x08

#define RAM             0x00

// characters capital            
	//         A 
	//      |-----|                
    //    F |__G__| B
    //    E |     | C
    //      |-----|
    //         D
    //  1 for sybmol

#if (defined(LCD18D) && (LCD18D == 1))
	#if (defined(LCD18A) && (LCD18A == 1))
		#define SEGA_BIT	0x40
		#define SEGB_BIT	0x10
		#define SEGC_BIT	0x02
		#define SEGD_BIT	0x01
		#define SEGE_BIT	0x04
		#define SEGF_BIT	0x20
		#define SEGG_BIT	0x08
	#endif
	#if (defined(LCD18D) && (LCD18A == 0))
		#define SEGA_BIT	0x02
		#define SEGB_BIT	0x08
		#define SEGC_BIT	0x40
		#define SEGD_BIT	0x80
		#define SEGE_BIT	0x20
		#define SEGF_BIT	0x04
		#define SEGG_BIT	0x10
	#endif
#endif

#if (defined(LCD18C) && (LCD18C == 1))
#define SEGA_BIT	0x40
#define SEGB_BIT	0x20
#define SEGC_BIT	0x04
#define SEGD_BIT	0x01
#define SEGE_BIT	0x02
#define SEGF_BIT	0x10
#define SEGG_BIT	0x08
#endif

#define LCD_0		(SEGA_BIT|SEGB_BIT|SEGC_BIT|SEGD_BIT|SEGE_BIT|SEGF_BIT)
#define LCD_1		(SEGB_BIT|SEGC_BIT)
#define LCD_2		(SEGA_BIT|SEGB_BIT|SEGG_BIT|SEGE_BIT|SEGD_BIT)
#define LCD_3		(SEGA_BIT|SEGB_BIT|SEGG_BIT|SEGC_BIT|SEGD_BIT)
#define LCD_4		(SEGF_BIT|SEGG_BIT|SEGB_BIT|SEGC_BIT)
#define LCD_5		(SEGA_BIT|SEGF_BIT|SEGG_BIT|SEGC_BIT|SEGD_BIT)
#define LCD_6		(SEGA_BIT|SEGF_BIT|SEGG_BIT|SEGC_BIT|SEGD_BIT|SEGE_BIT)
#define LCD_7		(SEGA_BIT|SEGB_BIT|SEGC_BIT)
#define LCD_8		(SEGA_BIT|SEGB_BIT|SEGC_BIT|SEGD_BIT|SEGE_BIT|SEGF_BIT|SEGG_BIT)
#define LCD_9		(SEGA_BIT|SEGB_BIT|SEGC_BIT|SEGD_BIT|SEGF_BIT|SEGG_BIT)



#define CHR_A	(SEGA_BIT|SEGB_BIT|SEGC_BIT|SEGE_BIT|SEGF_BIT|SEGG_BIT)
#define CHR_C   (SEGA_BIT|SEGF_BIT|SEGE_BIT|SEGD_BIT)
#define CHR_E   (SEGA_BIT|SEGF_BIT|SEGE_BIT|SEGD_BIT|SEGG_BIT)
#define CHR_F   (SEGA_BIT|SEGF_BIT|SEGE_BIT|SEGG_BIT)
#define CHR_G   (SEGA_BIT|SEGC_BIT|SEGF_BIT|SEGE_BIT|SEGD_BIT)
#define CHR_H   (SEGF_BIT|SEGE_BIT|SEGB_BIT|SEGC_BIT|SEGG_BIT)
#define CHR_I   (SEGB_BIT|SEGC_BIT)
#define CHR_L   (SEGF_BIT|SEGE_BIT|SEGD_BIT)
#define CHR_M   (SEGA_BIT|SEGB_BIT|SEGC_BIT|SEGE_BIT|SEGF_BIT|SEGG_BIT)
#define CHR_N   0x00
#define CHR_O   (SEGA_BIT|SEGF_BIT|SEGE_BIT|SEGD_BIT|SEGC_BIT|SEGB_BIT)
#define CHR_P   (SEGA_BIT|SEGF_BIT|SEGE_BIT|SEGB_BIT|SEGG_BIT)
#define CHR_S   (SEGA_BIT|SEGF_BIT|SEGG_BIT|SEGC_BIT|SEGD_BIT)
#define CHR_U   (SEGF_BIT|SEGE_BIT|SEGD_BIT|SEGC_BIT|SEGB_BIT)
#define CHR_ml  (SEGB_BIT|SEGA_BIT|SEGF_BIT|SEGE_BIT)
#define CHR_mr  (SEGC_BIT|SEGB_BIT|SEGA_BIT|SEGF_BIT)



//characters small

#define CHR_a	(SEGA_BIT|SEGB_BIT|SEGE_BIT|SEGD_BIT|SEGC_BIT|SEGG_BIT)
#define CHR_b	(SEGF_BIT|SEGE_BIT|SEGD_BIT|SEGC_BIT|SEGG_BIT)
#define CHR_d   (SEGB_BIT|SEGC_BIT|SEGD_BIT|SEGE_BIT|SEGG_BIT)
#define CHR_h   (SEGF_BIT|SEGE_BIT|SEGC_BIT|SEGG_BIT)
#define CHR_n   (SEGE_BIT|SEGC_BIT|SEGG_BIT)
#define CHR_o   (SEGE_BIT|SEGD_BIT|SEGC_BIT|SEGG_BIT)
#define CHR_q   (SEGA_BIT|SEGF_BIT|SEGG_BIT|SEGB_BIT|SEGC_BIT)
#define CHR_r   (SEGE_BIT|SEGG_BIT)
#define CHR_t   (SEGF_BIT|SEGE_BIT|SEGD_BIT|SEGG_BIT)
#define CHR_u   (SEGE_BIT|SEGD_BIT|SEGC_BIT)



#define CHR__     (SEGG_BIT)


#define CHR_0            LCD_0
#define CHR_1            LCD_1
#define CHR_2            LCD_2
#define CHR_3            LCD_3
#define CHR_4            LCD_4
#define CHR_5            LCD_5
#define CHR_6            LCD_6
#define CHR_7            LCD_7
#define CHR_8            LCD_8
#define CHR_9            LCD_9


#if (defined(LCD18D) && (LCD18D == 1))
// display_flag
#define DISP_OK         0x01
#define DISP_REV        0x80
#define DISP_EARTH      0x40
#define DISP_NM         0x10
#define DISP_MAG        0x02

	#if (defined(LCD18A) && (LCD18A == 1))
	// symbol 1
	#define DISP_DATE       0x80
	#define DISP_TIME       0x40
	#define DISP_SCO        0x20	
	#define DISP_SMAG       0x10
	#define DISP_k          0x08
	#define DISP_W          0x04
	#define DISP_TOD        0x02
	#define DISP_SR        	0x01
	// symbol 2

	#define DISP_DOTS		0x80
	#define DISP_CUM       	0x40
	#define DISP_PF        	0x00
	#define DISP_MD        	0x10
	#define DISP_BP       	0x08
	#define DISP_V       	0x04
	#define DISP_A       	0x02
	#define DISP_h       	0x01


	#endif
	#if (defined(LCD18D) && (LCD18A == 0))
	// symbol 1
	#define DISP_DATE       0x01
	#define DISP_TIME       0x02
	#define DISP_AV         0x04	
	#define DISP_SMAG       0x08
	#define DISP_k          0x10
	#define DISP_W          0x20
	#define DISP_TOD        0x40
	#define DISP_SR        	0x80

	// symbol 2

	#define DISP_DOTS		0x01
	#define DISP_CUM       	0x02
	#define DISP_SCO       	0x04
	#define DISP_MD        	0x08
	#define DISP_BP       	0x10
	#define DISP_V       	0x20
	#define DISP_A       	0x40
	#define DISP_h       	0x80

	// symbol 3
	#define DISP_DEC1       0x02
	#define DISP_DEC5       0x20
	#define DISP_DEC3       0x08
	#define DISP_DEC2		0x04

	#endif
	
	// symbol 3
	#define DISP_DEC1       0x02
	#define DISP_DEC5       0x20
	#define DISP_DEC3       0x08
	#define DISP_DEC2		0x04

#endif


#if (defined(LCD18C) && (LCD18C == 1))
// display_flag
#define DISP_OK         0x80
#define DISP_REV        0x08
#define DISP_EARTH      0x10
#define DISP_NM         0x04
#define DISP_MAG       	0x02

// symbol 1
#define DISP_AR         0x01
#define DISP_r         	0x02
#define DISP_h       	0x04
#define DISP_TOD        0x08
#define DISP_MD        	0x10
#define DISP_B        	0x20
#define DISP_DEC4       0x40
#define DISP_R        	0x80

// symbol 2
#define DISP_AL         0x01
#define DISP_T3         0x02
#define DISP_V       	0x04
#define DISP_k       	0x08
#define DISP_DEC1       0x10
#define DISP_DEC2		0x20
#define DISP_DOTS		0x40
#define DISP_DEC3       0x80


// symbol 3
#define DISP_BP       	0x01
#define DISP_DATE       0x20
#define DISP_TIME       0x40


#endif

#ifdef __cplusplus
}
#endif
#endif /* __APP_LCD_H */

