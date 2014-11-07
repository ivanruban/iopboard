#ifndef _USB_TERMINAL_H_
#define _USB_TERMINAL_H_

#include "microrl/microrl.h"

typedef struct
{
   microrl_t cmdLine;
} usbterminal_t;

typedef struct
{
   int (*execute) (int argc, const char * const * argv );
} usbterminalCfg_t;

/*
 *  Initialize USB controller and configure it as VCOM port.
 *  Also it initialize command line interface and link it with the primary VCOM port.
 */
void usbterminal_init(usbterminal_t *ctx, const usbterminalCfg_t *cfg);

// prints formated data on terminal
void usbterminal_printf(usbterminal_t *ctx,  const char * format, ... );

/*
 *  Performs infinity loop which waits for an incoming data from the VCOM port and
 *  passes it to the command line interface.
 */
void usbterminal_run(usbterminal_t *ctx);


#endif //_USB_TERMINAL_H_
