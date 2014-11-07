#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>

#include <LPC214x.h>

#include "hw/usb/usbhw.h"
#include "hw/usb/usbcore.h"
#include "hw/usb/cdc.h"
#include "hw/usb/cdcuser.h"

#include "vcom.h"

void vcomPortInit()
{
   CDC_Init ();
   USB_Init();
   USB_Connect(1);
}


int vcomRead(void *data, const int size)
{
   int numAvailByte=0;
   int numBytesToRead;
   do
   {
      CDC_OutBufAvailChar (&numAvailByte);
   }
   while(!numAvailByte);

   numBytesToRead = numAvailByte > size ? size : numAvailByte;
   numBytesToRead = CDC_RdOutBuf(data, &numBytesToRead);

   return numBytesToRead;
}


void vcomWrite(const void *data, const int size)
{
   int i=1024*10;//wait for USB_EVT_IN interruption timeout

   CDC_DepInEmpty = 0;
   USB_WriteEP(CDC_DEP_IN, (char *)data, size);

   while(i--)
   {
      if(CDC_DepInEmpty)
      {
         break;
      }
   }
}
