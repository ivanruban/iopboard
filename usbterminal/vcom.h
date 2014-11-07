/*----------------------------------------------------------------------------
 *      U S B  -  K e r n e l
 *----------------------------------------------------------------------------
 *      Name:    vcomuser.h
 *      Purpose: Virtual COM custom user definition file for Philips LPC214x Family
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
#ifndef __VCOM_H__
#define __VCOM_H__

#include <stdint.h>

void vcomPortInit();

void vcomWrite(const void *data, const int size);
int vcomRead(void *data, const int size);

#endif  /* __VCOM_H__ */
