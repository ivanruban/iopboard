#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <stdbool.h>

#include "hw/usb/usbhw.h"
#include "hw/usb/usbcore.h"

#include "usbterminal.h"
#include "vcom.h"

// print to VCOM X
static void print(const char * str)
{
   vcomWrite(str, strlen(str));
}

/*
 *  Performs infinity loop which waits for an incoming data from the primary VCOM port and
 *  passes it to the command line interface.
 */
void usbterminal_run(usbterminal_t *ctx)
{
   char buff[32];
   int read;
   int i;
   while(1)
   {
      read = vcomRead(buff, sizeof(buff));
      for(i=0; i<read; i++)
      {
         microrl_insert_char(&ctx->cmdLine, buff[i]);
      }
   }
}

/*
 *  Initialize USB controller and configure it as 2 VCOM ports.
 *  Also it initialize command line interface and link it with the primary VCOM port.
 */
void usbterminal_init(usbterminal_t *ctx, const usbterminalCfg_t *cfg)
{
   vcomPortInit();

   microrl_init(&ctx->cmdLine, print);
   microrl_set_execute_callback(&ctx->cmdLine, cfg->execute);
}

// prints formated data on terminal
void usbterminal_printf(usbterminal_t *ctx,  const char *fmt, ... )
{
   va_list  vargs;
   char msgbuf[64];

   va_start(vargs, fmt);
   vsnprintf(msgbuf, sizeof(msgbuf), fmt, vargs);
   va_end(vargs);

   //send out to VCOM X+1
   print(msgbuf);
}
