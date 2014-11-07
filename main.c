#include <LPC214x.h>

#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdbool.h>
#include <string.h>

#include "hw/AD9826/ad9826.h"

#include "usbterminal/usbterminal.h"

usbterminal_t terminal;

int adc(int argc, const char * const * argv);
int help(int argc, const char * const * argv);
int reboot(int argc, const char * const * argv);
int isp(int argc, const char * const * argv);
int rsir(int argc, const char * const * argv);
int execute(int argc, const char * const * argv);

typedef struct
{
	char *name;// command name
	char *help;// short command description
	int (*execute) (int argc, const char * const * argv );
} command_t;

command_t commands[ ] = {
{"help",  "Prints a list of supported commands", help},
{"reboot","Reset microcontroller", reboot},
{"isp",   "Invoke In-System programming (ISP)", isp},
{"rsir",  "Read 'Reset Source Identification Register'", rsir},
{"adc",   "Init ad9826 and read ADC output", adc},
};

int adc(int argc, const char * const * argv)
{
   AD9826_t ctx;
   int i;
   uint32_t data;
   uint16_t data1;
   uint16_t data2;
   const int bitValue = 61; //1 LSB = 4 V/65536 = 61 μV

   AD9826_cfg_t cfg = {CDSMode, false/*threeCHMode*/, REDChannel, {100/*tC1C2*/, 1000/*tPRB*/}};

   AD9826_init(&ctx, &cfg);

   for(i=0; i<10; i++)
   {
      data2 = AD9826_readADC(&ctx);
   }

   for(i=0; i<10; i++)
   {
      data1 = AD9826_readADC(&ctx);
      data =  ((data2 & 0xFF)<<8) | (data1 >> 8 & 0xFF);
      usbterminal_printf(&terminal,"readADC = %X(%u mv)\n\r", data, (data*bitValue)/1000);

      data2 = AD9826_readADC(&ctx);
      data =  ((data1 & 0xFF)<<8) | (data2 >> 8 & 0xFF);
      usbterminal_printf(&terminal,"readADC = %X(%u mv)\n\r", data, (data*bitValue)/1000);
   }

   return 0;
}


/*
	prints out the 'Reset Source Identification Register' state
*/
int rsir(int argc, const char * const * argv)
{
   typedef struct __attribute__((packed))
   {
      uint32_t POR  : 1;//Power-On Reset
      uint32_t EXTR : 1;//Assertion of the RESET signal sets this bit.
      uint32_t WDTR : 1;//This bit is set when the Watchdog Timer times
      uint32_t BODR : 1;//This bit is set when the 3.3 V power reaches a level below 2.6 V
      uint32_t reserved : 28;
   }RSID_t;

   RSID_t *value = (RSID_t*)&RSID;
   char reason[64] = {'\0'};
   if(value->POR)
   {
      strcat(reason, "Power-On Reset");
   }

   if(value->EXTR)
   {
      if('\0' != reason[0])
      {
         strcat(reason," + ");
      }
      strcat(reason, "Assertion of the RESET signal");
   }

   if(value->WDTR)
   {
      if('\0' != reason[0])
      {
         strcat(reason," + ");
      }
      strcat(reason, "Watchdog Timer");
   }

   if(value->BODR)
   {
      if('\0' != reason[0])
      {
         strcat(reason," + ");
      }
      strcat(reason, "3.3 V power reaches a level below 2.6 V");
   }

   if('\0' == reason[0])
   {
      strcat(reason, "Unknown");
   }
   usbterminal_printf(&terminal, "RSIR: 0x%X - %s\n\r", RSID, reason);

   return 0;
}


void initBootloader()
{
   unsigned long temp;
   temp = PINSEL0;
   /* Connect RXD0 & TXD0 pins to GPIO */
   PINSEL0 = temp & 0xFFFFFFF3;
   /* Select P0.14 as an output and P0.1 as an input */
   temp = IODIR0;
   temp = temp | 0x4000;
   temp = temp & 0xFFFFFFFD;
   IODIR0 = temp;
   /* Clear P0.14 */
   IOCLR0 = 0x4000;

   /* Disable Interrupts in the VIC*/
   VICIntEnClr=0XFFFFFFFF;

   /*
   Disconnect PLL if you want to do ISP at crystal frequency.
   Otherwise you need to pass the PLL freq when bootloader goes in
   ISP mode.
   cclk = crystal when PLL is disconnected
   cclk = PLL freq when PLL is connected.
   Disconnecting the PLL is recommended.  */
   PLL0CON = 0x0;
   PLL0FEED = 0xAA;
   PLL0FEED= 0x55;
   /*
   Set the VPB divider to 1/4 if your application changes the VPBDIV value.
   The bootloader is hard-coded to use the reset value of VPBDIV register
   */
   VPBDIV = 0x0;

   /* Restore reset state of Timer1 */
   T1PR=0x0;
   T1MCR=0x0;
   T1CCR=0x0;
   /* Map bootloader vectors */
   MEMMAP = 0x0;
}

/*
	Invoke the bootloader.
	The bootloader will read pin P0.14 to detect if ISP is forced
	Since P0.14 is configured as an output and set to 0, the bootloader
	will go in ISP mode.
*/
int isp(int argc, const char * const * argv)
{
   void (*bootloader_entry)(void) = (void (*)(void))(0x0);

   usbterminal_printf(&terminal, "Invoke the bootloader\n\r");
   initBootloader();
   while(1)
   {
      bootloader_entry();
   }
   return 0;
}

/*
	Initiates a watchdog reset by configuring invalid WDFEED sequence.
*/
int reboot(int argc, const char * const * argv)
{
   #define  WDEN 0x1
   #define  WDRESET 0x2

   WDTC = 1024;              // short timeout; we don't care (won't be using it)
   WDMOD = WDEN | WDRESET;   // watchdog resets CPU
   WDFEED = 0xAA;            // start ...
   WDFEED = 0x55;            // ... watchdog.

   WDFEED = 0xAA;            // start ...
   WDFEED = 0x00;            // ... invalid WDFEED sequence causes instant reset.

   return 0;
}


int help(int argc, const char * const * argv)
{
	int i;
	for(i=0; i<sizeof(commands)/sizeof(command_t); i++)
	{
		usbterminal_printf(&terminal, "%s - %s\n\r",commands[i].name, commands[i].help);
	}
	return 0;
}


/*
 * Search for the pointed command(as argv[0]) in the registered commands list.
 * If command is found the associated function is executed, else the help message is printed out.
 */
int execute(int argc, const char * const * argv)
{
	int i;
	for(i=0; i<sizeof(commands)/sizeof(command_t); i++)
	{
		if(0 == strcmp(argv[0], commands[i].name))
	  {
			return commands[i].execute(argc, argv);
		}
	}

   usbterminal_printf(&terminal, "Unknown command: %s\n\r", argv[0]);
   help(argc, argv);
   return 0;
}

typedef enum
{
   PCTIM0 = 1,
   PCTIM1 = 2,
   PCUART0= 3,
   PCUART1= 4,
   PCPWM0 = 5,
   PCI2C0 = 7,
   PCSPI0 = 8,
   PCRTC  = 9,
   PCSPI1 = 10,
   PCAD0  = 12,
   PCI2C1 = 19,
   PCAD1  = 20,
   PUSB   = 31,
}ePeripheral_t;

void enablePeripheralsPower(const ePeripheral_t device)
{
   PCONP |= 1 << device;
}

typedef enum
{
   Idle,
   Powerdown,
   Normal,
}ePowerMode_t;

void setPowerMode(const ePowerMode_t mode)
{
   switch(mode)
   {
      case Idle:
         PCON |= 0x1;
      break;

      case Powerdown:
         PCON |= 0x2;
      break;

      case Normal:
      default:
         PCON &= ~0x3;
      break;
   }
}

int main()
{
   usbterminalCfg_t config = {execute};

   //disable all Peripherals blocks. Call enablePeripheralsPower to power up a specified device.
   //PCONP = 0x0;

   //enablePeripheralsPower(PCUART0);
   //enablePeripheralsPower(PUSB);

   usbterminal_init(&terminal, &config);
   usbterminal_run(&terminal);
}

