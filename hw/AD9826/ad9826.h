#ifndef  _AD9826_H
#define  _AD9826_H

#include <stdint.h>
#include <stdbool.h>


#define SLOAD   (1<<24)
#define SCLK    (1<<25)
#define SDATA   (1<<17)
#define ADCCLK  (1<<22)
#define CDSCLK1 (1<<21)
#define CDSCLK2 (1<<12)//P0.12/DSR1/MAT1.0/AD1.3

#define ADCDATA 0x7F8// p0.3-p0.10

#define AD9826_CFG          0x00
#define MUX_CFG             0x01
#define RED_PGA_CFG         0x02
#define GREEN_PGA_CFG       0x03
#define BLUE_PGA_CFG        0x04
#define RED_OFFSET_CFG      0x05
#define GREEN_OFFSET_CFG    0x06
#define BLUE_OFFSET_CFG     0x07

typedef enum
{
   CDSMode,
   SHAMode
} eCDSOperation;

typedef enum
{
   BLUEChannel,
   GREENChannel,
   REDChannel
} eChannel;

typedef struct
{
   uint16_t tC1C2;
   uint16_t tPRB;//tPRB = tC1C2 + tC2C1 + tC1 + tC2
} AD9826Timing_t;

typedef struct
{
   eCDSOperation operationMode;
   bool threeCHMode;
   eChannel channel;
   AD9826Timing_t timing;
} AD9826_cfg_t;

typedef struct
{
   AD9826_cfg_t cfg;
} AD9826_t;

/*
 *  Initialize AD9826 and apply the given configuration.
 */
void AD9826_init(AD9826_t *ctx, const AD9826_cfg_t *cfg);

/*
 *  Initialize AD9826 and apply the given configuration.
 */
uint16_t AD9826_readADC(AD9826_t *ctx);

#endif
