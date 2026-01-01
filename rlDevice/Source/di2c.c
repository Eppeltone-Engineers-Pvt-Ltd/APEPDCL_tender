#include "..\\include\\dIODefine.h"
#include "..\\Include\\dUser_includes.h"
#include "..\\include\\dTypedef.h"
#include "..\\include\\dSerial.h"
#include "..\\include\\dI2c.h"
#include "..\\include\\dMisc.h"
#include "..\\include\\dIOCtrl.h"





//--------------------------i2c specific data----------------------
#define	EPR_DEVICE_BUS_ADDRESS				0xA0		/* EEPROM device bus address */
#define	EPR_BUS_SPEED					60000		/* I2C Bus speed (Hz) */
#define	EPR_DEVICE_SIZE					0x8000UL	/* EEPROM device size (total bytes) */
#define	EPR_DEVICE_PAGESIZE				0x40		/* 64 bytes/page */
#define EPR_DEVICE_WRITE_CYCLE_TIME			(5000)		/* 10ms */

#define	EPR_1BYTE_TIMEOUT				(40000.0f / EPR_BUS_SPEED)									/* 1 byte timeout (ms) for read/write */
#define	EPR_SEND_ADDR_MAX_TIMEOUT			((uint32_t)(EPR_1BYTE_TIMEOUT * 3))							/* 2 byte addr + 1 offset */
#define	EPR_READ_MAX_TIMEOUT				((uint32_t)(EPR_1BYTE_TIMEOUT * (EPR_DEVICE_SIZE+1)))		/* Device size + 1 offset */
#define	EPR_WRITE_MAX_TIMEOUT				((uint32_t)(EPR_1BYTE_TIMEOUT * (EPR_DEVICE_PAGESIZE+1)))	/* Page bytes + 1 offset */


/* EEPROM Return Code */
#define EPR_OK						0			/* Normal end */
#define EPR_ERROR					1			/* Error in eeprom */
#define EPR_ERROR_NO_RESPOND				2			/* Device does not respond */
#define EPR_ERROR_SIZE					3			/* Expected size and address are not suitable */


#define I2C_READ_MODE	0x00
#define I2C_WRITE_MODE	0x01

//------------------------------------------------------------------

static void iica0_masterhandler(void);
static uint8_t i2c_Send(uint8_t adr, uint8_t * const tx_buf, uint16_t tx_num,uint8_t i2cType);

//--------------------------
volatile uint8_t   g_iica0_master_status_flag; /* iica0 master flag */
volatile uint8_t   g_iica0_slave_status_flag;  /* iica0 slave flag */
volatile uint8_t * gp_iica0_rx_address;        /* iica0 receive buffer address */
volatile uint16_t  g_iica0_rx_len;             /* iica0 receive data length */
volatile uint16_t  g_iica0_rx_cnt;             /* iica0 receive data count */
volatile uint8_t * gp_iica0_tx_address;        /* iica0 send buffer address */
volatile uint16_t  g_iica0_tx_cnt;             /* iica0 send data count */

static volatile uint8_t g_is_send_end    = 0;
static volatile uint8_t g_is_receive_end = 0;


//--------------------------
uint8_t dywait(void);

void init_I2c(void)
{
	uint8_t itemp=0;
	
	
	
	//------------------

	// scl out low
	I2C_SCL_DIR&=~I2C_SCL_BIT;
	I2C_SCL&=~I2C_SCL_BIT;

	// sda input low
	I2C_SDA_DIR|=I2C_SDA_BIT;
	I2C_SDA&=~I2C_SDA_BIT;
	
	while(1)
	{
		
		I2C_SCL&=~I2C_SCL_BIT;
		MCU_Delay(10);
		I2C_SCL|=I2C_SCL_BIT;
		MCU_Delay(10);
		if(++itemp>=16)
			break;
	}	
    IICA0EN = 1U; 	/* supply IICA0 clock */
    IICE0 = 0U; 	/* disable IICA0 operation */
    IICAMK0 = 1U; 	/* disable INTIICA0 interrupt */
    IICAIF0 = 0U; 	/* clear INTIICA0 interrupt flag */
	
    /* Set INTIICA0 low priority */
    IICAPR10 = 1U;
    IICAPR00 = 1U; 

    SMC0 = 0U;
    IICWL0 = 16;
    IICWH0 = 16;
    DFC0 = 0U; /* digital filter off */
    IICCTL01 = 0x00;//_00_IICA_fCLK;
    SVA0 = 0xA0;
    STCEN0 = 1U;
    IICRSV0 = 1U;
    SPIE0 = 0U;
    WTIM0 = 1U;
    ACKE0 = 1U;
    IICAMK0 = 0U;
    IICE0 = 1U;
    LREL0 = 1U;
	
	
	/* Set SCLA0, SDAA0 pin */ 
	
	I2C_SDA_DIR&=~I2C_SDA_BIT;
	I2C_SCL_DIR&=~I2C_SCL_BIT;

	I2C_SDA&=~I2C_SDA_BIT;
	I2C_SCL&=~I2C_SCL_BIT;
	

	
}

uint8_t	read_eeprom(uint8_t* buf,uint16_t addr, uint16_t size)
{
	uint32_t	timeout;
	uint8_t		device_addr;				/* Device address */
	uint8_t		local_addr[2];				/* EEPROM Local address */

	
	/* Check user buffer */
	if (buf == NULL)
	{
		return EPR_ERROR;	/* parameter error */
	}
	
	/* check the address */
	if ((size == 0) ||(((uint32_t)addr + size) > EPR_DEVICE_SIZE))
	{
		return EPR_ERROR_SIZE;
	}
	
	/* Get the address */
	device_addr = EPR_DEVICE_BUS_ADDRESS ;
		
	local_addr[0] = (uint8_t)((addr >> 8) & 0xFF);
	local_addr[1] = (uint8_t)(addr & 0xFF);
	
	/* Send the address (HIGH + LOW) */
	g_is_send_end = 0;
	if (i2c_Send(device_addr, local_addr, 2,I2C_WRITE_MODE) != EPR_OK)
	{
		return EPR_ERROR;
	}
	timeout = EPR_SEND_ADDR_MAX_TIMEOUT * 1000;
	while (g_is_send_end == 0)
	{
		MCU_Delay(1);	/* 1us delay */
		
		timeout--;
		if (timeout == 0)
		{
			return EPR_ERROR_NO_RESPOND;
		}
	}
	
	/* Delay after stop operation: 10us (+1us tolerance of MCU_Delay)
	 * (Stop Set-up time + Bus free time) */	
	MCU_Delay(11);
	
	/* Read the buffer */
	g_is_receive_end = 0;
	timeout = ((EPR_READ_MAX_TIMEOUT * 1000) / EPR_DEVICE_SIZE) * (size + 1);
	if (i2c_Send(device_addr, buf, size,I2C_READ_MODE) != EPR_OK)
	{
		return EPR_ERROR;
	}
	while (g_is_receive_end == 0)
	{
		MCU_Delay(1);	/* 1us delay */
		
		timeout--;
		if (timeout == 0)
		{
			return EPR_ERROR_NO_RESPOND;
		}
	}

	/* Delay after stop operation: 10us (+1us tolerance of MCU_Delay)
	 * (Stop Set-up time + Bus free time) */
	MCU_Delay(11);
	
	return EPR_OK;	/* Read succesful */
}
uint8_t write_eeprom(const uint8_t * buf,uint16_t addr, uint16_t size)
{
	uint32_t	timeout;								/* Timeout counter */
	uint16_t	page_size;								/* Page size */
	uint8_t		device_addr;							/* Device address */
	uint8_t		local_buffer[EPR_DEVICE_PAGESIZE+2];	/* EEPROM Local address + buffer */
	uint16_t	pos;
	
	
	/* Check user buffer */
	if (buf == NULL)
	{
		return EPR_ERROR;	/* parameter error */
	}
		
		
	if (size == 0 ||(((uint32_t)addr + size) > EPR_DEVICE_SIZE))
	{
		return EPR_ERROR_SIZE;
	}
	
	//EPR_WRITE_PROTECT_DISABLE_STATEMENT;	/* Disable write protect */
	
	/* Write each page with evaluated address, size */
	while (size > 0)
	{
		/* Get the compensator of 1 page size */
		page_size = EPR_DEVICE_PAGESIZE - (uint16_t)(addr % EPR_DEVICE_PAGESIZE);
		if (size < page_size)
		{
			page_size = size;
		}
		
		/* Get the address */
		
		device_addr = EPR_DEVICE_BUS_ADDRESS ;
		
		local_buffer[0] = (uint8_t)((addr >> 8) & 0xFF);
		local_buffer[1] = (uint8_t)(addr & 0xFF);
	
		/* Copy the page (from buf) to local_buffer
		 * max. is EPR_DEVICE_PAGESIZE */
		for (pos = 0; pos < page_size; pos++)
		{
			local_buffer[pos + 2] = buf[pos];
		}
		
		/* Send the address (HIGH + LOW) + 1 page data */
		g_is_send_end = 0;
		if (i2c_Send(device_addr, local_buffer, page_size + 2,I2C_WRITE_MODE) != EPR_OK)
		{
			return EPR_ERROR;
		}
		timeout = EPR_WRITE_MAX_TIMEOUT * 1000;
		while (g_is_send_end == 0)
		{
			MCU_Delay(1);	/* 1us delay */
		
			timeout--;
			if (timeout == 0)
			{
				return EPR_ERROR_NO_RESPOND;
			}
		}
		
		/* Delay after write cycle + stop operation */

		MCU_Delay((EPR_DEVICE_WRITE_CYCLE_TIME+1));
		
		
		/* Point to next page */
		addr += page_size;
		buf  += page_size;
		size -= page_size;
	}
	
	//EPR_WRITE_PROTECT_ENABLE_STATEMENT;	/* Enable write protect */
	
	return EPR_OK;	/* Write succesful */	
}

/***********************************************************************************************************************
* Function Name: r_iica0_interrupt
* Description  : This function is INTIICA0 interrupt service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
#pragma interrupt r_iica0_interrupt(vect=INTIICA0,bank=RB2)

static void r_iica0_interrupt(void)
{
	
    if ((IICS0 & _80_IICA_MASTER_DEVICE) == 0x80U)
    {
        iica0_masterhandler();
    }
}

/***********************************************************************************************************************
* Function Name: iica0_masterhandler
* Description  : This function is IICA0 master handler.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static void iica0_masterhandler(void)
{
    if (!((0U == IICBSY0) && (g_iica0_tx_cnt != 0U)))
    {
        if ((g_iica0_master_status_flag & _80_IICA_ADDRESS_COMPLETE) == 0U)
        {
            if (1U == ACKD0)
            {
                g_iica0_master_status_flag |= _80_IICA_ADDRESS_COMPLETE;

                if (1U == TRC0)
                {
                    WTIM0 = 1U;

                    if (g_iica0_tx_cnt > 0U)
                    {
                        IICA0 = *gp_iica0_tx_address;
                        gp_iica0_tx_address++;
                        g_iica0_tx_cnt--;
                    }
                    else
                    {
						//------------send end----------
                        SPT0 = 1U;
						g_is_send_end = 1;	
						//------------------------------
                    }
                }
                else
                {
                    ACKE0 = 1U;
                    WTIM0 = 0U;
                    WREL0 = 1U;
                }
            }
        }
        else
        {
            /* Master send control */
            if (1U == TRC0)
            {
                if (!((0U == ACKD0) && (g_iica0_tx_cnt != 0U)))
                {
                    if (g_iica0_tx_cnt > 0U)
                    {
                        IICA0 = *gp_iica0_tx_address;
                        gp_iica0_tx_address++;
                        g_iica0_tx_cnt--;
                    }
                    else
                    {
						//------------send end----------
                        SPT0 = 1U;
						g_is_send_end = 1;	
						//------------------------------
                    }
                }
            }
            /* Master receive control */
            else
            {
                if (g_iica0_rx_cnt < g_iica0_rx_len)
                {
                    *gp_iica0_rx_address = IICA0;
                    gp_iica0_rx_address++;
                    g_iica0_rx_cnt++;

                    if (g_iica0_rx_cnt == g_iica0_rx_len)
                    {
                        ACKE0 = 0U;
                        WREL0 = 1U;
                        WTIM0 = 1U;
                    }
                    else
                    {
                        WREL0 = 1U;
                    }
                }
                else
                {
					//----------receive end----------
                    SPT0 = 1U;	
					g_is_receive_end = 1;
					//-------------------------------
                }
            }
        }
    }
}

uint8_t i2c_Send(uint8_t adr, uint8_t * const buf, uint16_t length,uint8_t i2cMode)
{
    volatile uint8_t status = 1;

    IICAMK0 = 1U;  /* disable INTIICA0 interrupt */

    if (1U == IICBSY0)
    {
        /* Check bus busy */
        IICAMK0 = 0U;  /* enable INTIICA0 interrupt */
    } 
    else if ((1U == SPT0) || (1U == STT0)) 
    {
        /* Check trigger */
        IICAMK0 = 0U;  /* enable INTIICA0 interrupt */  
    }
    else
    {
        STT0 = 1U; /* send IICA0 start condition */
        IICAMK0 = 0U;  /* enable INTIICA0 interrupt */
        
        /* Wait */
		status=1;
        while (status--)
        {
            ;
        }
		
        
        if (0U == STD0)
        {
             status = 1;
        }
		
        /* Set parameter */
        
	
        
    g_iica0_master_status_flag = _00_IICA_MASTER_FLAG_CLEAR;
	if(i2cMode==I2C_WRITE_MODE)
	{
		g_iica0_tx_cnt = length;
		gp_iica0_tx_address = buf;
		adr &= (uint8_t)~0x01U; /* set send mode */
	}
	else
	{
		g_iica0_rx_len=length;
		g_iica0_rx_cnt=0;
		gp_iica0_rx_address = buf;
		adr |= 0x01U; /* set receive mode */
	}
	status=0;		
	IICA0 = adr; /* send address */
    }

    return (status);
}


uint8_t dywait(void)
{
	uint8_t status;

	STT0 = 1U; /* send IICA0 start condition */
    
    /* Wait */
	status=1;
    while (status--)
    {
        ;
    }
	
    status=0x00;
    if (0U == STD0)
    {
         status = 1;
    }
		
	return status;	
}

uint8_t memTest(void)
{
	uint8_t membuf[4]={0x00,0x01,0x02,0x03};
	uint8_t status=1;
	
	write_eeprom(membuf,EPR_DEVICE_SIZE-4,4);
	membuf[0]=0;
	membuf[1]=0;
	membuf[2]=0;
	membuf[3]=0;
	read_eeprom(membuf,0,4);
	
	
	if(!((membuf[0]==0x00)&&(membuf[1]==0x01)&&(membuf[2]==0x02)&&(membuf[3]==0x03)))
	{
		membuf[0]=0;
		membuf[1]=0;
		membuf[2]=0;
		membuf[3]=0;
		
		read_eeprom(membuf,EPR_DEVICE_SIZE-4,4);
		if((membuf[0]==0x00)&&(membuf[1]==0x01)&&(membuf[2]==0x02)&&(membuf[3]==0x03))
			status=0;
	}
	
	
	
	return status;
	
}