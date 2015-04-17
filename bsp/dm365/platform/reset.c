/*
 * File      : cpu.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Develop Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://openlab.rt-thread.com/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2006-03-13     Bernard      first version
 */

#include <rthw.h>
#include <rtthread.h>
#include "dm36x.h"

/**
 * @addtogroup DM36X
 */
/*@{*/

/**
 * reset cpu by dog's time-out
 *
 */
void machine_reset()
{
	/* Disable all interrupt except the WDT */
	writel(0, DM365_EINT_ENABLE0);
	writel(0, DM365_EINT_ENABLE1);
	
	/* Disable watchdog */
	//WTCON = 0x0000;

	/* Initialize watchdog timer count register */
	//WTCNT = 0x0001;

	/* Enable watchdog timer; assert reset at timer timeout */
	//WTCON = 0x0021;
}

/**
 *  shutdown CPU
 *
 */
void machine_shutdown()
{
	rt_kprintf("shutdown...\n");
}

/*@}*/
