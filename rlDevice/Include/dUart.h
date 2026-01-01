#ifndef __DEV_UART_H
#define __DEV_UART_H

#include "dTypedef.h"
#include "dSerial.h"

extern UartBuf dUartBuf;
void init_Uart0(void);
void Uart0_Start(void);
void Uart0_Stop(void);
void Meter_Response(const uint8_t* bptr,uint16_t noofBytes);
void Meter2_Response(const uint8_t* bptr,uint16_t noofBytes);
void init_Uart2(void);
void Uart2_Start(void);
void Uart2_Stop(void);

#endif //DEV_UART_H