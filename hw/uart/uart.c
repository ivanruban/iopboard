#include <lpc21xx.h>
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>

#define LSR_RDR		0x01
#define LSR_TEMT	0x40



void UART_BaudRateConfig(unsigned char port)
{
  switch(port)
  {
    case 0:
	  U0LCR =0x83;
	  U0DLM =0x0;
	  U0DLL =98;
	  U0LCR=0x03;
	  break;

    case 1:
	  U1LCR =0x83;
	  U1DLM =0x0;
	  U1DLL =98;
	  U1LCR=0x03;
	  break;
  }
}



void UARTInit(unsigned char port)
{
  switch(port)
  {
    case 0:
	  PINSEL0 = 0x00000005;
	  U0FCR = 0x07;
	  break;
    case 1:
	  PINSEL0 = 0x00050000;
	  U1FCR = 0x07;
	  break;
  }
  UART_BaudRateConfig(port);
}




unsigned char UARTByteRead(unsigned char port) {
  unsigned char ch;

  switch(port)
  {
  case 0: while(!(U0LSR & LSR_RDR));
    ch=U0RBR;
	break;
  case 1: while(!(U1LSR & LSR_RDR));
    ch=U1RBR;
	break;
  }
  return(ch);
}


void UARTByteSend(unsigned char port,unsigned char data) {
  switch(port)
  {
    case 0:
	  while(!(U0LSR & LSR_TEMT ));
	  U0THR = data;
	  break;
    case 1:
      while (!(U1LSR & LSR_TEMT ));
	  U1THR = data;
	  break;
  }
}


void sendstr (unsigned char port, char *p) {
  while (*p) {
    UARTByteSend(port, *p++);
  }
}


void uart_printf(unsigned char port,  const char *fmt, ... )
{
   va_list  vargs;
   char msgbuf[64];

   va_start(vargs, fmt);
   vsnprintf(msgbuf, sizeof(msgbuf)-1, fmt, vargs);
   va_end(vargs);

   //send out to VCOM X+1
   sendstr(port, msgbuf);
}
