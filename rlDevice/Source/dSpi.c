/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products.
* No other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
* applicable laws, including copyright laws. 
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING THIS SOFTWARE, WHETHER EXPRESS, IMPLIED
* OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NON-INFRINGEMENT.  ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY
* LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE FOR ANY DIRECT,
* INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR
* ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability 
* of this software. By using this software, you agree to the additional terms and conditions found by accessing the 
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2015, 2018 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/

/***********************************************************************************************************************
* File Name    : r_cg_sau.c
* Version      : Code Generator for RL78/I1C V1.01.03.02 [16 Nov 2018]
* Device(s)    : R5F10NPJ
* Tool-Chain   : CCRL
* Description  : This file implements device driver for SAU module.
* Creation Date: 9/27/2019
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "..\\include\\dIODefine.h"
#include "..\\Include\\dUser_includes.h"
#include "..\\include\\dTypedef.h"
#include "..\\include\\dSerial.h"
#include "..\\include\\dMisc.h"
#include "..\\include\\dIOCtrl.h"
#include "..\\Include\\dWatchDog.h"
#include "..\\Include\\dSpi.h"

#include "..\\..\\rlApplication\\Include\\AppVariables.h"


#define FLASH_BUS_SPEED                       1000000             /* SPI Bus speed (Hz) */
#define FLASH_DEVICE_START_ADDR               0x00000             /* SerialFlash device start address */
#define FLASH_DEVICE_PHYSICAL_SIZE            0x200000            /* SerialFlash device size (total bytes) */
#define FLASH_DEVICE_MINIMUM_BLOCK_SIZE       (0x1000)            /* SerialFlash minimum erase size (bytes)   */
#define FLASH_DEVICE_MAXIMUM_BLOCK_SIZE       (0x10000)           /* SerialFlash minimum erase size (bytes)   */
#define FLASH_DEVICE_BLOCK_ADDR_MASK          (0xFFFFF000)        /* Mask of address: relate to block size (refer to datasheet) */
#define FLASH_DEVICE_BYTE_PROGRAM_TIME        (10)                /* Time to program one byte : (us)          */
#define FLASH_DEVICE_WRITE_STATUS_TIME        (15)                /* Time to write to status register : (us)  */
#define FLASH_MAX_MINIMUM_BLOCK_ERASE_TIME    (25000)             /* Time to erase minimum size : (us)        */
#define FLASH_MAX_CHIP_ERASE_TIME             (50000)             /* Time to erase whole chip : (us)          */

/* EEPROM Return Code */
#define SPI_OK								0			/* Normal end */
#define SPI_ERROR							1			/* Error in eeprom */
#define SPI_ERROR_NO_RESPOND				2			/* Device does not respond */
#define SPI_ERROR_SIZE						3			/* Expected size and address are not suitable */

#define SFL_BUS_SPEED                       9600             /* SPI Bus speed (Hz) */
#define SFL_1BYTE_TIMEOUT                   (40000.0f / SFL_BUS_SPEED)                                  /* 1 byte timeout (ms) for read/write */

#define SPI_FLASH_INSTRUCTION_WRITE 			(0x02)
#define SPI_FLASH_INSTRUCTION_READ				(0x03)
#define	SPI_FLASH_INSTRUCTION_WRDI				(0X04)
#define SPI_FLASH_INSTRUCTION_RDSR				(0x05)
#define SPI_FLASH_INSTRUCTION_WREN				(0x06)
#define SPI_FLASH_INSTRUCTION_BLK4K_ERASE     	(0x20)

#define SPI_FLASH_PAGE_SIZE     		(256U)



/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
/* Start user code for pragma. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
volatile uint8_t * gp_csi00_rx_address;        /* csi00 receive buffer address */
volatile uint16_t  g_csi00_rx_count;           /* csi00 receive data count */
volatile uint8_t * gp_csi00_tx_address;        /* csi00 send buffer address */
volatile uint16_t  g_csi00_tx_count;           /* csi00 send data count */
volatile uint8_t  g_csi00_end;           /* csi00 send data count */
/* Start user code for global. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

void spi_send(uint8_t * txbuf,uint8_t * rxbuf, uint16_t txlen,uint16_t rxlen);
void R_CSI00_Create(void);
void R_CSI00_Start(void);

/***********************************************************************************************************************
* Function Name: R_SAU0_Create
* Description  : This function initializes the SAU0 module.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void init_Spi(void)
{
    SAU0EN = 1U;    /* enables input clock supply */
    NOP();
    NOP();
    NOP();
    NOP();
    SPS0 = _0003_SAU_CK00_FCLK_3 | _0030_SAU_CK01_FCLK_3;
    R_CSI00_Create();
	R_CSI00_Start();
}
/***********************************************************************************************************************
* Function Name: R_CSI00_Create
* Description  : This function initializes the CSI00 module.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_CSI00_Create(void)
{
    ST0 |= _0001_SAU_CH0_STOP_TRG_ON;
    CSIMK00 = 1U;   /* disable INTCSI00 interrupt */
    CSIIF00 = 0U;   /* clear INTCSI00 interrupt flag */
    /* Set INTCSI00 low priority */
    CSIPR100 = 1U;
    CSIPR000 = 1U;
    SIR00 = _0002_SAU_SIRMN_PECTMN | _0001_SAU_SIRMN_OVCTMN;
    SMR00 = _0020_SMR00_DEFAULT_VALUE | _0000_SAU_CLOCK_SELECT_CK00 | _0000_SAU_CLOCK_MODE_CKS | 
            _0000_SAU_TRIGGER_SOFTWARE | _0000_SAU_MODE_CSI | _0000_SAU_TRANSFER_END;
    SCR00 = _0004_SCR00_DEFAULT_VALUE | _C000_SAU_RECEPTION_TRANSMISSION | _0000_SAU_TIMING_1 | 
            _0000_SAU_INTSRE_MASK | _0000_SAU_PARITY_NONE | _0000_SAU_MSB | _0000_SAU_STOP_NONE | 
            _0003_SAU_LENGTH_8;
			
			//_2000_SAU_TIMING_3
			//_0000_SAU_INTSRE_MASK | _0000_SAU_PARITY_NONE |  | _0000_SAU_STOP_NONE | 
			
	//SCR00=_C000_SAU_RECEPTION_TRANSMISSION | _0000_SAU_TIMING_1 | _0000_SAU_MSB | _0003_SAU_LENGTH_8;			
			
    SDR00 = _9A00_SAU0_CH0_BAUDRATE_DIVISOR;
    SO0 |= _0100_SAU_CH0_CLOCK_OUTPUT_1;
    SO0 &= (uint16_t)~(_0001_SAU_CH0_DATA_OUTPUT_1);
    SOE0 |= _0001_SAU_CH0_OUTPUT_ENABLE;
	
    /* Set SI00 pin */
    PM0 |= 0x40U;
    /* Set SO00 pin */
    P0 |= 0x80U;
    PM0 &= 0x7FU;
    /* Set SCK00 pin */
    P0 |= 0x20U;
    PM0 &= 0xDFU;
}
/***********************************************************************************************************************
* Function Name: R_CSI00_Start
* Description  : This function starts the CSI00 module operation.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_CSI00_Start(void)
{
    SO0 |= _0100_SAU_CH0_CLOCK_OUTPUT_1;
    SO0 &= (uint16_t)~(_0001_SAU_CH0_DATA_OUTPUT_1);
    SOE0 |= _0001_SAU_CH0_OUTPUT_ENABLE;
    SS0 |= _0001_SAU_CH0_START_TRG_ON;
    CSIIF00 = 0U;   /* clear INTCSI00 interrupt flag */
    CSIMK00 = 0U;   /* enable INTCSI00 interrupt */
}
/***********************************************************************************************************************
* Function Name: R_CSI00_Stop
* Description  : This function stops the CSI00 module operation.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_CSI00_Stop(void)
{
    CSIMK00 = 1U;   /* disable INTCSI00 interrupt */
    ST0 |= _0001_SAU_CH0_STOP_TRG_ON;
    SOE0 &= (uint16_t)~_0001_SAU_CH0_OUTPUT_ENABLE;
    CSIIF00 = 0U;   /* clear INTCSI00 interrupt flag */
}
/***********************************************************************************************************************
* Function Name: R_CSI00_Send_Recei
* Description  : This function sends and receives CSI00 data.
* Arguments    : tx_buf -
*                    transfer buffer pointer
*                tx_num -
*                    buffer size
*                rx_buf -
*                    receive buffer pointer
* Return Value : status -
*                    MD_OK or MD_ARGERROR
***********************************************************************************************************************/
uint8_t waitForCompleteFlag(uint16_t bytes)
{
	uint32_t	timeout;
	
	timeout = (uint32_t)( ((8 * 1000000) / SFL_BUS_SPEED)*bytes );
	while (g_csi00_end == 0)
	{
		WDT_Restart(0xAC);
		MCU_Delay(1);	/* 1us delay */
		
		timeout--;
		if (timeout == 0)
		{
			break;
		}
	}
	return g_csi00_end;
	
}

uint8_t WaitForWrite(uint32_t time_us)
{
    uint32_t timeout, i;
	uint8_t status=1;
    
    timeout = time_us / 10;
    
    for (i=0; i< timeout; i++)
    {
        /* Delay between status polling */
        MCU_Delay(1000);
        
        /* If device idle, break wait loop */
        status=DeviceReadStatusRegister()&0x01;
        if (status==0)
        {
            status=1;
			break;
        }
        
        WDT_Restart(0xAC);
    }
    
    return status;
}


/***********************************************************************************************************************
* Function Name: static uint8_t SFL_DeviceReadStatusRegister(SFL_STATUS * status_reg)
* Description  : SerialFlash Read device status register
* Arguments    : Read back status store in global register
* Return Value : uint8_t: Execution status
*              :    SFL_OK                  Normal end
*              :    SFL_ERROR_NO_RESPOND    Expected size and address are not suitable
***********************************************************************************************************************/
uint8_t DeviceReadStatusRegister(void)
{
    uint8_t local_buf[2];
	uint8_t status;

	SELECT_FLASH_MEMORY;
	
    local_buf[0] = SPI_FLASH_INSTRUCTION_RDSR;
    local_buf[1] = 0x00;
	spi_send(local_buf,local_buf,2,1);
	status=waitForCompleteFlag(3);
	if(status)
    	status=local_buf[0];
	
	DE_SELECT_FLASH_MEMORY;
    return status;
}


uint8_t DeviceWriteEnable(void)
{
    uint8_t local_buf[1];
    uint8_t status;
	SELECT_FLASH_MEMORY;
	FLASH_WRITE_ENABLE;		
	
    local_buf[0] = SPI_FLASH_INSTRUCTION_WREN;
	spi_send(local_buf,local_buf,1,0);
	status=waitForCompleteFlag(1);
	
	FLASH_WRITE_DISABLE;
	DE_SELECT_FLASH_MEMORY;
	
   
    return status;
}
uint8_t DeviceWriteDisable(void)
{
    uint8_t local_buf[1];
    uint8_t status;
	SELECT_FLASH_MEMORY;
	FLASH_WRITE_ENABLE;		
	
    local_buf[0] = SPI_FLASH_INSTRUCTION_WRDI;
	spi_send(local_buf,local_buf,1,0);
	status=waitForCompleteFlag(1);

	FLASH_WRITE_DISABLE;
	DE_SELECT_FLASH_MEMORY;
	
    return status;
}


uint8_t DeviceByteProgramming(uint32_t addr,uint8_t*buf, uint16_t bytes)
{
    uint8_t local_buf[12];
	uint8_t status;

	
	SELECT_FLASH_MEMORY;
	FLASH_WRITE_ENABLE;		
		
    /* Write a byte */
    local_buf[0] = SPI_FLASH_INSTRUCTION_WRITE;
    local_buf[1] = (uint8_t)(addr >> 16);
    local_buf[2] = (uint8_t)(addr >> 8);
    local_buf[3] = (uint8_t)(addr);
	spi_send(local_buf,local_buf,4,0);
	status=waitForCompleteFlag(4);
	if(status)
	{
		spi_send(buf,buf,bytes,0);
		status=waitForCompleteFlag(bytes);
	}
	
	
	DE_SELECT_FLASH_MEMORY;
	if(status)
	{
		/* Wait for internal operation */
		WaitForWrite(FLASH_DEVICE_BYTE_PROGRAM_TIME*bytes);
	}
	FLASH_WRITE_DISABLE;
    return status;
}


void write_spi(uint8_t * tx_buf,uint32_t addr, uint16_t length)
{

	uint8_t status;
	uint16_t	page_size;								/* Page size */
    /* Write Enable */
			
	while(length>0)
	{
		status=DeviceWriteEnable();
		if(status==1)
		{
			/* Get the compensator of 1 page size */
			page_size = SPI_FLASH_PAGE_SIZE - (uint16_t)(addr % SPI_FLASH_PAGE_SIZE);
			if (length < page_size)
			{
				page_size = length;
			}			
			
			DeviceByteProgramming(addr,tx_buf,page_size);
			/* Point to next page */
			addr += page_size;
			tx_buf  += page_size;
			length -= page_size;
		}
	}
}

void read_spi(uint8_t* buf,uint32_t addr, uint16_t size)
{
    uint8_t local_buf[5];
    SELECT_FLASH_MEMORY;
    /* Check input argument */
    if (buf != NULL)
    {
	    local_buf[0] = SPI_FLASH_INSTRUCTION_READ;
	    local_buf[1] = (uint8_t)(addr >> 16);
	    local_buf[2] = (uint8_t)(addr >> 8);
	    local_buf[3] = (uint8_t)(addr);
		local_buf[4] = 0x00;
	    
	    
    	spi_send(local_buf,buf,5,size);
		waitForCompleteFlag(size+5);
	}
   
    DE_SELECT_FLASH_MEMORY;
    
}

uint8_t DeviceErase4KB(uint32_t addr)
{
    uint8_t local_buf[4];
    uint8_t status;

	status=DeviceWriteEnable();
	if(status==1)
	{
		SELECT_FLASH_MEMORY;
		FLASH_WRITE_ENABLE;		
		
	    /* Erase block */

	    local_buf[0] = SPI_FLASH_INSTRUCTION_BLK4K_ERASE;
	    local_buf[1] = (uint8_t)(addr >> 16);
	    local_buf[2] = (uint8_t)(addr >> 8);
	    local_buf[3] = (uint8_t)(addr);
		
	    
		spi_send(local_buf,local_buf,4,0);
		status=waitForCompleteFlag(4);
		
		
		DE_SELECT_FLASH_MEMORY;
		
	    /* Wait for internal operation */
		if(status==1)
		{
			WaitForWrite(FLASH_MAX_MINIMUM_BLOCK_ERASE_TIME);
		}
	}
	FLASH_WRITE_DISABLE;
    return status;
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

void spi_send(uint8_t * txbuf, uint8_t * rxbuf, uint16_t txlen,uint16_t rxlen)
{
	
	
    if (txlen < 1U)
    {
        return;
    }
    else
    {
		CSIMK00 = 1U;                     /* disable INTCSI00 interrupt */

		g_csi00_rx_count=rxlen;
        g_csi00_tx_count = txlen;        /* send data count */
        gp_csi00_tx_address = txbuf;     /* send buffer pointer */
        gp_csi00_rx_address = rxbuf;     /* receive buffer pointer */
		CSIIF00 = 0U;   /* clear INTCSI00 interrupt flag */
		g_csi00_end=0;
        SIO00 = *gp_csi00_tx_address++;    /* started by writing data to SDR[7:0] */
        CSIMK00 = 0U;                     /* enable INTCSI00 interrupt */
    }
	
    return;
	
}


#pragma interrupt r_csi00_interrupt(vect=INTCSI00,bank=RB2)
static void __near r_csi00_interrupt(void)
{
    volatile uint8_t err_type;
    volatile uint8_t sio_rx;

    err_type = (uint8_t)(SSR00 & _0001_SAU_OVERRUN_ERROR);
    SIR00 = (uint16_t)err_type;

    if (1U != err_type)
    {
		sio_rx = SIO00;
        if (g_csi00_tx_count > 0U)
        {
			g_csi00_tx_count--;	
			if(g_csi00_tx_count>0)
			{
            	SIO00 = *gp_csi00_tx_address;
            	gp_csi00_tx_address++;
			}
            
        }
		
        if (0U == g_csi00_tx_count)
        {
            if (g_csi00_rx_count>0)
			{
                *gp_csi00_rx_address++ = sio_rx;
				g_csi00_rx_count--;
				
				if(g_csi00_rx_count>0)
					SIO00=0xFF;
			}
            
			if(g_csi00_rx_count==0)
			{
				CSIMK00 = 1U;                     /* disable INTCSI00 interrupt */
				g_csi00_end=1;
			}
        }
    }
	else
		err_type=0;
	
}

void testSPI(void)
{
	uint8_t iTemp;
	//--------spi test-------
	for(iTemp=0;iTemp<64;iTemp++)
		RxTxBuffer[iTemp]=iTemp+64;
	DeviceErase4KB(0);	
	write_spi(RxTxBuffer,0x00,32);
	write_spi(&RxTxBuffer[32],32,32);
	for(iTemp=0;iTemp<64;iTemp++)
		RxTxBuffer[iTemp]=0x00;
	
	read_spi(RxTxBuffer,0,1024);
	//read_spi(&RxTxBuffer[32],32,32);
	
}