/*
 * File      : board.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006 - 2009 RT-Thread Develop Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2010-11-13     weety      first version
 */

#include <rtthread.h>
#include <rthw.h>

#include "board.h"
//#include "led.h"

/**
 * @addtogroup dm365
 */
/*@{*/

//extern rt_uint32_t PCLK, FCLK, HCLK, UCLK;

extern void rt_hw_clock_init(void);
//extern void rt_hw_lcd_init(void);
extern void rt_hw_mmu_init(void);
//extern void rt_hw_touch_init(void);
//extern void rt_hw_key_init(void);

extern void rt_hw_get_clock(void);
extern void rt_hw_set_dividor(rt_uint8_t hdivn, rt_uint8_t pdivn);
extern void rt_hw_set_clock(rt_uint8_t sdiv, rt_uint8_t pdiv, rt_uint8_t mdiv);


#define UART0	((struct uartport *)DAVINCI_UART0_BASE)
struct serial_int_rx uart0_int_rx;
struct serial_device uart0 =
{
	UART0,
	&uart0_int_rx,
	RT_NULL
};
struct rt_device uart0_device;

/**
 * This function will handle rtos timer
 */
void rt_timer_handler(int vector)
{
	rt_tick_increase();
}

/**
 * This function will handle serial
 */
void rt_serial_handler(int vector)
{
	//if (UART0->iir & 0x0e)
	rt_hw_serial_isr(&uart0_device);
}

/**
 * This function will handle init uart
 */
void rt_hw_uart_init(void)
{
	rt_uint32_t divisor;

	divisor = (24000000 + (115200 * (16 / 2))) / (16 * 115200);
	UART0->ier = 0;
	UART0->lcr = 0x83; //8N1
	UART0->dll = 0;
	UART0->dlh = 0;
	UART0->lcr = 0x03;
	UART0->mcr = 0x03; //RTS,CTS
	UART0->fcr = 0x07; //FIFO
	UART0->lcr = 0x83;
	UART0->dll = divisor & 0xff;
	UART0->dlh = (divisor >> 8) & 0xff;
	UART0->lcr = 0x03;
	UART0->mdr = 0; //16x over-sampling
	UART0->pwremu_mgmt = 0x6000;
	
	rt_hw_interrupt_install(IRQ_UARTINT0, rt_serial_handler, "UART0");
	rt_hw_interrupt_umask(IRQ_UARTINT0);
	UART0->ier = 0x05;
}

/**
 * This function will init timer0 for system ticks
 */
 void rt_hw_timer_init()
 {
	/* timer0, input clocks 24MHz */
	volatile timer_regs_t *regs =
		(volatile timer_regs_t*)DAVINCI_TIMER1_BASE;//DAVINCI_TIMER0_BASE;

	/*disable timer*/
	regs->tcr &= ~(0x3UL << 6);

	//TIMMODE 32BIT UNCHAINED MODE
	regs->tgcr |=(0x1UL << 2);

	/*not in reset timer */
	regs->tgcr |= (0x1UL << 0);

	//regs->tgcr &= ~(0x1UL << 1);

	/* set Period Registers */
	regs->prd12 = 24000000/RT_TICK_PER_SECOND;
	regs->tim12 = 0;

	/* Set enable mode */
	regs->tcr |= (0x2UL << 6); //period mode
	

	/* install interrupt handler */
	rt_hw_interrupt_install(IRQ_DM365_TINT2, rt_timer_handler, "timer1_12");//IRQ_DM365_TINT0_TINT12
	rt_hw_interrupt_umask(IRQ_DM365_TINT2);//IRQ_DM365_TINT2

 }

/**
 * This function will init dm365 board
 */
void rt_hw_board_init()
{
	psc_change_state(DAVINCI_DM365_LPSC_TIMER0, 3);
	psc_change_state(DAVINCI_DM365_LPSC_TIMER1, 3);
	/* initialize the system clock */
	rt_hw_clock_init();

	/* Get the clock */
	//rt_hw_get_clock();

	/* initialize led port */
	//rt_hw_led_init();

	/* initialize uart */
	rt_hw_uart_init();

#ifdef RT_USING_RTGUI
	/* init virtual keypad */
	//rt_hw_key_init();
#endif

	/* initialize mmu */
	rt_hw_mmu_init();

	/* initialize timer0 */
	rt_hw_timer_init();

}

/*@}*/
