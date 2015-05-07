#include <rtthread.h>
#include <rtdevice.h>

static int spi_rw_test(struct rt_spi_device *spi)
{
	int i;
    rt_uint8_t txbuf[16];
    rt_uint8_t rxbuf[16];

	for (i = 0; i < 16; i++)
		txbuf[i] = i;
    rt_spi_transfer(spi, txbuf, rxbuf, 16);
    for (i = 0; i < 16; i++)
		rt_kprintf("0x%x \n", rxbuf[i]);

    return 0;
}

int spi_test(void)
{
    struct rt_spi_device *spi;
    struct rt_spi_configuration cfg;

    spi = (struct rt_spi_device *)rt_device_find("spi10");
    if (spi == RT_NULL)
    {
        rt_kprintf("spi device %s not found!\n", "spi10");
        return -RT_ERROR;
    }

    /* config spi */
    cfg.data_width = 8;
    cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible: Mode 0 and Mode 3 */
    cfg.max_hz = 18000000; /* 20M for test. */
    rt_spi_configure(spi, &cfg);

    return spi_rw_test(spi);
}


#ifdef RT_USING_FINSH

#include <finsh.h>
FINSH_FUNCTION_EXPORT(spi_test, test spi);

#endif

