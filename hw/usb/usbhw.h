/*----------------------------------------------------------------------------
 *      U S B  -  K e r n e l
 *----------------------------------------------------------------------------
 *      Name:    usbhw.h
 *      Purpose: USB Hardware layer header file for Philips LPC214x Family
 *		Microprocessors
 *      Version: V1.04
 *----------------------------------------------------------------------------
 *      This software is supplied "AS IS" without any warranties, express,
 *      implied or statutory, including but not limited to the implied
 *      warranties of fitness for purpose, satisfactory quality and
 *      noninfringement. Keil extends you a royalty-free right to reproduce and
 *      distribute executable files created using this software for use on
 *      Philips LPC2xxx microcontroller devices only. Nothing else gives you the
 *      right to use this software.
 *
 *      Copyright (c) 2005 Keil Software.
 *		Modified by Philips Semiconductor
 *---------------------------------------------------------------------------*/
#ifndef __USBHW_H__
#define __USBHW_H__

#include "type.h"
#include "usb.h"

/* USB Hardware Functions */
extern void  USB_Init       (void);
extern void  USB_Connect    (BOOL con);
extern void  USB_Reset      (void);
extern void  USB_Suspend    (void);
extern void  USB_Resume     (void);
extern void  USB_WakeUp     (void);
extern void  USB_WakeUpCfg  (BOOL cfg);
extern void  USB_SetAddress (S8 adr);
extern void  USB_Configure  (BOOL cfg);
extern void  USB_ConfigEP   (USB_ENDPOINT_DESCRIPTOR *pEPD);
extern void  USB_DirCtrlEP  (S8 dir);
extern void  USB_EnableEP   (S8 EPNum);
extern void  USB_DisableEP  (S8 EPNum);
extern void  USB_ResetEP    (S8 EPNum);
extern void  USB_SetStallEP (S8 EPNum);
extern void  USB_ClrStallEP (S8 EPNum);
extern S32 USB_ReadEP     (S8 EPNum, S8 *pData);
extern S32 USB_WriteEP    (S8 EPNum, S8 *pData, S32 cnt);
extern void  USB_ISR        (void) __irq;


#endif  /* __USBHW_H__ */
