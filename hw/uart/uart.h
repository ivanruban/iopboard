#ifndef _UART_H
#define _UART_H



void UARTInit(unsigned char port);
unsigned char UARTByteRead(unsigned char port);
void UARTByteSend(unsigned char port,unsigned char data);
void sendstr(unsigned char port, char *p);

void uart_printf(unsigned char port,  const char *fmt, ... );

#endif  /* __VCOMUSER_H__ */

