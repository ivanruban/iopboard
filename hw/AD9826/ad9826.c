#include <string.h>
#include <LPC214x.h>

#include "AD9826.h"

static void delay(uint16_t dly)
{
    while (--dly);
}

/*
 * Performs write operation on AD9826's serial port.
 * See AD9826.pdf for details.
 */
static void AD9826_Send(const uint8_t addr, uint16_t data)
{
   uint8_t i;
   data = data | (addr << 12);

   IOCLR1|=SCLK;
   IOCLR1|=SLOAD;
   delay(100);

   for(i=0;i<16;i++)
   {
      if(data&0x8000)
      {
         IOSET0|=SDATA;
      }
      else
      {
         IOCLR0|=SDATA;
      }
      IOSET1|=SCLK;
      data=data<<1;
      delay(100);
      IOCLR1|=SCLK;
   }
   IOSET1|=SLOAD;
   delay(100);
}

/*
 * Performs read operation for given address on AD9826's serial port.
 * See AD9826.pdf for details.
 */
static uint16_t AD9826_Receive(const uint8_t addr)
{
   int i;
   uint16_t data = 1<<15 | (addr<<12);

   IOCLR1|=SCLK;
   IOCLR1|=SLOAD;
   delay(100);

   for(i=0;i<7;i++)
   {
       if(data&0x8000)
       {
          IOSET0|=SDATA;
       }
       else
       {
          IOCLR0|=SDATA;
       }
       IOSET1|=SCLK;
       data=data<<1;
       delay(100);
       IOCLR1|=SCLK;
   }

   IODIR0 &= ~SDATA;
   for(i=0; i<9; i++)
   {
      delay(100);

      if(IOPIN0 & SDATA)
      {
         data |= 1<<(8-i);
      }
      IOSET1|=SCLK;
      delay(100);
      IOCLR1|=SCLK;
   }

   IODIR0 |= SDATA;
   IOSET1 |= SLOAD;
   delay(100);

   return data & 0x1FF;
}


/*
 *  Initialize AD9826 and apply the given configuration.
 */
void AD9826_init(AD9826_t *ctx, const AD9826_cfg_t *cfg)
{
   typedef struct __attribute__((packed))//*Power-on default value.
   {
      uint16_t outputMode     : 1; //0 = 2 Byte*; 1 = 1 Byte
      uint16_t reserved2      : 1;
      uint16_t powerDown      : 1; //1 = On; 0 = Off (Normal)*
      uint16_t inputClampBias : 1; //1 = 4 V*; 0 = 3 V
      uint16_t CDSOperation   : 1; //1 = CDS Mode*; 0 = SHA Mode
      uint16_t threeCHMode    : 1; //1 = On*
      uint16_t internalVREF   : 1;//1 = Enabled*
      uint16_t inputRange     : 1;//0 = 2 V; 1 = 4 V*.
      uint16_t reserved1      : 8;
   } AD9826_CFG_t;

   typedef struct __attribute__((packed))//*Power-on default value.
   {
      uint16_t reserved2          : 4;
      uint16_t channelSelectBLUE  : 1;//0 = Off*
      uint16_t channelSelectGREEN : 1;//0 = Off*
      uint16_t channelSelectRED   : 1;//1 = RED*
      uint16_t MUXOrder           : 1; //1 = R-G-B*; 0 = B-G-R
      uint16_t reserved1          : 8;
   } AD9826_MUX_CFG_t;

   uint16_t data;
   AD9826_CFG_t *cfgReg;
   AD9826_MUX_CFG_t muxCfg;

   IODIR1 |= SCLK | SLOAD | ADCCLK | CDSCLK1;
   IODIR0 |= SDATA | CDSCLK2;
   IODIR0 &= ~ADCDATA;

   IOSET1 |= SLOAD;

   memcpy(&ctx->cfg, cfg, sizeof(AD9826_cfg_t));

   data = AD9826_Receive(AD9826_CFG);
   cfgReg = (AD9826_CFG_t *)&data;
   if(SHAMode == cfg->operationMode)
   {
      cfgReg->CDSOperation = 0;
   }
   if(!cfg->threeCHMode)
   {
      memset(&muxCfg, 0, sizeof(muxCfg));
      cfgReg->threeCHMode = 0;
      switch(cfg->channel)
      {
         case BLUEChannel :
            muxCfg.channelSelectBLUE =1;
            muxCfg.channelSelectGREEN =0;
            muxCfg.channelSelectRED =0;
         break;

         case GREENChannel :
            muxCfg.channelSelectBLUE =0;
            muxCfg.channelSelectGREEN =1;
            muxCfg.channelSelectRED =0;
         break;

         case REDChannel :
            muxCfg.channelSelectBLUE =0;
            muxCfg.channelSelectGREEN =0;
            muxCfg.channelSelectRED =1;
         break;
      }
      AD9826_Send(MUX_CFG, *((uint16_t*)&muxCfg) );
   }
   AD9826_Send(AD9826_CFG, data);

   //@warning set RED_OFFSET to +300mv for testing purpose
   data = 0xFF;
   AD9826_Send(RED_OFFSET_CFG, data);
}

/*
 * Reads AD9826's digital outputs in Correlated Double Sampler (CDS) mode
 * with 1 channel and 2 bytes output.
 * See AD9826.pdf for details.
 */
uint16_t AD9826_readADC(AD9826_t *ctx)
{
   uint16_t data = 0u;
   uint16_t tC1 = 10;
   uint16_t tC2 = 20;
   uint16_t tC2ADF = ctx->cfg.timing.tPRB - (tC1 + ctx->cfg.timing.tC1C2 + tC2) ;

   uint8_t lowByteN_4;
   uint8_t highByteN_3;

   IOSET1 |= CDSCLK1;
   IOCLR1 |= ADCCLK;
   delay(tC1);
   lowByteN_4 = (IOPIN0 & ADCDATA) >> 3;

   IOCLR1 |= CDSCLK1;

   delay(ctx->cfg.timing.tC1C2);
   IOSET0 |= CDSCLK2;
   delay(tC2/2);
   IOSET1 |= ADCCLK;
   delay(tC2/2);
   IOCLR0 |= CDSCLK2;

   highByteN_3  = (IOPIN0 & ADCDATA) >> 3;
   data = (lowByteN_4 << 8) | highByteN_3;

   delay(tC2ADF);

   return data;
}
