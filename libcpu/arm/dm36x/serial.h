#ifndef __RT_HW_SERIAL_H__
#define __RT_HW_SERIAL_H__

#include <rthw.h>
#include <rtthread.h>

#include "dm36x.h"

#define LSR_DR		0x01		/* Data ready */
#define LSR_THRE	0x20		/* Xmit holding register empty */
//#define	USTAT_TXB_EMPTY		0x02   	/* tx buffer empty */
#define BPS					115200	/* serial baudrate */

#define UART_RX_BUFFER_SIZE		64
#define UART_TX_BUFFER_SIZE		64

struct serial_int_rx
{
	rt_uint8_t  rx_buffer[UART_RX_BUFFER_SIZE];
	rt_uint32_t read_index, save_index;
};

struct serial_int_tx
{
	rt_uint8_t  tx_buffer[UART_TX_BUFFER_SIZE];
	rt_uint32_t write_index, save_index;
};

typedef struct uartport
{
	volatile rt_uint32_t rbr;
	volatile rt_uint32_t ier;
	volatile rt_uint32_t fcr;
	volatile rt_uint32_t lcr;
	volatile rt_uint32_t mcr;
	volatile rt_uint32_t lsr;
	volatile rt_uint32_t msr;
	volatile rt_uint32_t scr;
	volatile rt_uint32_t dll;
	volatile rt_uint32_t dlh;
	
	volatile rt_uint32_t res[2];
	volatile rt_uint32_t pwremu_mgmt;
	volatile rt_uint32_t mdr;
}uartport;

#define thr rbr
#define iir fcr

struct serial_device
{
	uartport* uart_device;
	
	/* rx structure */
	struct serial_int_rx* int_rx;

	/* tx structure */
	struct serial_int_tx* int_tx;
};

rt_err_t rt_hw_serial_register(rt_device_t device, const char* name, rt_uint32_t flag, struct serial_device *serial);

void rt_hw_serial_isr(rt_device_t device);

#endif
