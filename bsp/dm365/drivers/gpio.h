/*
 * TI DaVinci GPIO Support
 *
 * Copyright (c) 2006 David Brownell
 * Copyright (c) 2007, MontaVista Software, Inc. <source@mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef	__DM365_GPIO_H
#define	__DM365_GPIO_H
#include <dm36x.h>

#define	GPIO(X)		(X)		


#define get_io(r)			*((volatile u_int *)(TI81XX_L4_SLOW_IO_ADDRESS(r)))
#define set_io(r,v)		*((volatile u_int *)(TI81XX_L4_SLOW_IO_ADDRESS(r))) = (v)
#define and_io(r,v)		*((volatile u_int *)(TI81XX_L4_SLOW_IO_ADDRESS(r))) &= (v)
#define or_io(r,v)			*((volatile u_int *)(TI81XX_L4_SLOW_IO_ADDRESS(r))) |= (v)

#define v_get_io(r)			*((volatile u_int *)(r))
#define v_set_io(r,v)		*((volatile u_int *)(r)) = (v)
#define v_and_io(r,v)		*((volatile u_int *)(r)) &= (v)
#define v_or_io(r,v)			*((volatile u_int *)(r)) |= (v)

enum gpio_intr_mode
{
    LEVELDETECT_LOW = 0,
    LEVELDETECT_HIGH,
    RISINGDETECT,  
    FALLINGDETECT,
    EDGEDETECT	//both rising-edge and falling-edge detect
};

enum gpio_intr_req
{
    INTR_REQ_A = 0,
    INTR_REQ_B
};

enum gpio_intr_num
{
	GPIOINT0A = 96,
	GPIOINT0B,
	GPIOINT1A,
	GPIOINT1B, 
};

enum pin_func_mod
{
    GPIO_MOD = 0x80,
    SPI_MOD =0x01,
    VP_MOD=0x04,
    IIC_MOD=0x20
};

#endif				/* __TI814X_GPIO_H */
