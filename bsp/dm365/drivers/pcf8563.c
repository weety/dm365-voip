/*
 * File      : pcf8563.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author		Notes
 * 2012-04-25     weety		first version
 */

#include <rtdevice.h>
#include <rthw.h>
#include <time.h>
#include <dm36x.h>


#define PCF8563_REG_ST1		0x00 /* status */
#define PCF8563_REG_ST2		0x01

#define PCF8563_REG_SC		0x02 /* datetime */
#define PCF8563_REG_MN		0x03
#define PCF8563_REG_HR		0x04
#define PCF8563_REG_DM		0x05
#define PCF8563_REG_DW		0x06
#define PCF8563_REG_MO		0x07
#define PCF8563_REG_YR		0x08

#define PCF8563_REG_AMN		0x09 /* alarm */
#define PCF8563_REG_AHR		0x0A
#define PCF8563_REG_ADM		0x0B
#define PCF8563_REG_ADW		0x0C

#define PCF8563_REG_CLKO	0x0D /* clock out */
#define PCF8563_REG_TMRC	0x0E /* timer control */
#define PCF8563_REG_TMR		0x0F /* timer */

#define PCF8563_SC_LV		0x80 /* low voltage */
#define PCF8563_MO_C		0x80 /* century */

struct pcf8563_device {
	struct rt_device dev;
	struct rt_i2c_bus_device *bus;
	rt_uint16_t addr;
	rt_uint16_t flags;
};

rt_uint8_t bcd2bin(rt_uint8_t val)
{
	return (val & 0x0f) + (val >> 4) * 10;
}

rt_uint8_t bin2bcd(rt_uint8_t val)
{
	return ((val / 10) << 4) + val % 10;
}

void dump_tm(struct tm *tm)
{
	rt_kprintf("tm->tm_year=%d, tm->tm_mon= %d, tm->tm_mday=%d, "
		   "tm->tm_hour=%d, tm->tm_min=%d, tm->tm_sec=%d, "
		   "tm->tm_wday=%d\n",
		   tm->tm_year, tm->tm_mon, tm->tm_mday, 
		   tm->tm_hour, tm->tm_min, tm->tm_sec, tm->tm_wday);
}

static rt_err_t pcf8563_get_datetime(struct pcf8563_device *pcf8563_dev, struct tm* tm)
{
	unsigned char buf[13] = { PCF8563_REG_ST1 };

	struct rt_i2c_msg msgs[] = {
		{ pcf8563_dev->addr, 0, 1, buf },	/* setup read ptr */
		{ pcf8563_dev->addr, RT_I2C_RD, 13, buf },	/* read status + date */
	};

	/* read registers */
	if ((rt_i2c_transfer(pcf8563_dev->bus, msgs, 2)) != 2) {
		rt_kprintf("read error\n");
		return -RT_EIO;
	}

	if (buf[PCF8563_REG_SC] & PCF8563_SC_LV)
		rt_kprintf("low voltage detected, date/time is not reliable.\n");

	rt_kprintf("raw data is st1=%02x, st2=%02x, sec=%02x, min=%02x, hr=%02x, "
		"mday=%02x, wday=%02x, mon=%02x, year=%02x\n",
		buf[0], buf[1], buf[2], buf[3],
		buf[4], buf[5], buf[6], buf[7],
		buf[8]);

	tm->tm_sec = bcd2bin(buf[PCF8563_REG_SC] & 0x7F);
	tm->tm_min = bcd2bin(buf[PCF8563_REG_MN] & 0x7F);
	tm->tm_hour = bcd2bin(buf[PCF8563_REG_HR] & 0x3F); /* rtc hr 0-23 */
	tm->tm_mday = bcd2bin(buf[PCF8563_REG_DM] & 0x3F);
	tm->tm_wday = buf[PCF8563_REG_DW] & 0x07;
	tm->tm_mon = bcd2bin(buf[PCF8563_REG_MO] & 0x1F) - 1; /* rtc mn 1-12 */
	tm->tm_year = bcd2bin(buf[PCF8563_REG_YR]);
	if (tm->tm_year < 70)
		tm->tm_year += 100;	/* assume we are in 1970...2069 */

	return RT_EOK;
}


static rt_err_t pcf8563_set_datetime(struct pcf8563_device *pcf8563_dev, struct tm *tm)
{
	int i, err;
	unsigned char buf[9];

	rt_kprintf("secs=%d, mins=%d, hours=%d, "
		"mday=%d, mon=%d, year=%d, wday=%d\n",
		tm->tm_sec, tm->tm_min, tm->tm_hour,
		tm->tm_mday, tm->tm_mon, tm->tm_year, tm->tm_wday);

	/* hours, minutes and seconds */
	buf[PCF8563_REG_SC] = bin2bcd(tm->tm_sec);
	buf[PCF8563_REG_MN] = bin2bcd(tm->tm_min);
	buf[PCF8563_REG_HR] = bin2bcd(tm->tm_hour);

	buf[PCF8563_REG_DM] = bin2bcd(tm->tm_mday);

	/* month, 1 - 12 */
	buf[PCF8563_REG_MO] = bin2bcd(tm->tm_mon + 1);

	/* year and century */
	buf[PCF8563_REG_YR] = bin2bcd(tm->tm_year % 100);
	if (tm->tm_year >= 100)
		buf[PCF8563_REG_MO] |= PCF8563_MO_C;

	buf[PCF8563_REG_DW] = tm->tm_wday & 0x07;

	/* write register's data */
	for (i = 0; i < 7; i++) {
		unsigned char data[2] = { PCF8563_REG_SC + i,
						buf[PCF8563_REG_SC + i] };

		err = rt_i2c_master_send(pcf8563_dev->bus, pcf8563_dev->addr, 
					 pcf8563_dev->flags, data, sizeof(data));
		if (err != sizeof(data)) {
			rt_kprintf("err=%d addr=%02x, data=%02x\n",
				err, data[0], data[1]);
			return -RT_EIO;
		}
	}

	return RT_EOK;
}



static rt_err_t rt_rtc_open(rt_device_t dev, rt_uint16_t oflag)
{
	if (dev->rx_indicate != RT_NULL)
	{
		/* Open Interrupt */
	}

	return RT_EOK;
}

static rt_size_t rt_rtc_read(rt_device_t dev, rt_off_t pos, void* buffer, rt_size_t size)
{
	return 0;
}

static rt_err_t rt_rtc_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
	rt_err_t ret;
	rt_time_t time, *timep;
	struct tm  tm1, *tm2;
	RT_ASSERT(dev != RT_NULL);

	switch (cmd)
	{
	case RT_DEVICE_CTRL_RTC_GET_TIME:
		timep = (rt_time_t *)args;
		/* read device */
		ret = pcf8563_get_datetime((struct pcf8563_device *)dev->user_data, &tm1);
		if (ret != RT_EOK)
		{
			return -RT_EIO;
		}
		//*timep = timegm(&tm1);
		*timep = mktime(&tm1);
		break;

	case RT_DEVICE_CTRL_RTC_SET_TIME:
	{
		time = *(rt_time_t *)args;
		tm2 = gmtime_r(&time, &tm1);
		
		ret = pcf8563_set_datetime((struct pcf8563_device *)dev->user_data, tm2);

		if (ret != RT_EOK)
		{
			return -RT_EIO;
		}
		break;
	}
	}

	return RT_EOK;
}


rt_err_t pcf8563_init(char *bus_name, rt_uint16_t dev_addr)
{
	struct rt_device *rtc;
	struct pcf8563_device *pcf8563_dev;

	pcf8563_dev = rt_malloc(sizeof(struct pcf8563_device));
	if (!pcf8563_dev)
	{
		return -RT_ENOMEM;
	}

	rt_memset(pcf8563_dev, 0, sizeof(struct pcf8563_device));
	pcf8563_dev->addr = dev_addr;
	pcf8563_dev->bus = rt_i2c_bus_device_find(bus_name);

	if (!pcf8563_dev->bus)
	{
		return -RT_EEMPTY;
	}

	rtc = &pcf8563_dev->dev;
	rtc->type	= RT_Device_Class_RTC;

	/* register rtc device */
	rtc->init 	= RT_NULL;
	rtc->open 	= rt_rtc_open;
	rtc->close	= RT_NULL;
	rtc->read 	= rt_rtc_read;
	rtc->write	= RT_NULL;
	rtc->control = rt_rtc_control;

	/* no private */
	rtc->user_data = pcf8563_dev;

	rt_device_register(rtc, "rtc", RT_DEVICE_FLAG_RDWR);

	return RT_EOK;
}

#include "finsh.h"
FINSH_FUNCTION_EXPORT(pcf8563_init, init pcf8563);

/*void pcf8563_read(void)
{
	struct tm tm1;
	pcf8563_get_datetime(&pcf8563_device, &tm1);
}

FINSH_FUNCTION_EXPORT(pcf8563_read, read pcf8563 register);*/


time_t time(time_t* t)
{
	rt_device_t device;
	time_t time=0;

	device = rt_device_find("rtc");
	if (device != RT_NULL)
	{
		rt_device_control(device, RT_DEVICE_CTRL_RTC_GET_TIME, &time);
		if (t != RT_NULL) *t = time;
	}

	return time;
}

#ifdef RT_USING_FINSH
#include <finsh.h>

void set_date(rt_uint32_t year, rt_uint32_t month, rt_uint32_t day)
{
	time_t now;
	struct tm* ti;
	rt_device_t device;

	ti = RT_NULL;
	/* get current time */
	time(&now);

	ti = localtime(&now);
	if (ti != RT_NULL)
	{
		ti->tm_year = year - 1900;
		ti->tm_mon 	= month - 1; /* ti->tm_mon 	= month; 0~11 */
		ti->tm_mday = day;
	}

	now = mktime(ti);

	device = rt_device_find("rtc");
	if (device != RT_NULL)
	{
		rt_rtc_control(device, RT_DEVICE_CTRL_RTC_SET_TIME, &now);
	}
}
FINSH_FUNCTION_EXPORT(set_date, set date. e.g: set_date(2010,2,28))

void set_time(rt_uint32_t hour, rt_uint32_t minute, rt_uint32_t second)
{
	time_t now;
	struct tm* ti;
	rt_device_t device;

	ti = RT_NULL;
	/* get current time */
	time(&now);

	ti = localtime(&now);
	if (ti != RT_NULL)
	{
		ti->tm_hour = hour;
		ti->tm_min 	= minute;
		ti->tm_sec 	= second;
	}

	now = mktime(ti);
	device = rt_device_find("rtc");
	if (device != RT_NULL)
	{
		rt_rtc_control(device, RT_DEVICE_CTRL_RTC_SET_TIME, &now);
	}
}
FINSH_FUNCTION_EXPORT(set_time, set time. e.g: set_time(23,59,59))

void list_date(void)
{
	time_t now;

	time(&now);
	rt_kprintf("%s\n", ctime(&now));
}
FINSH_FUNCTION_EXPORT(list_date, show date and time.)
#endif

