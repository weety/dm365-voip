/*
 * File      : mmcsd_spi.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006 - 2012, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2012-09-27     aozima       first version.
 */

#include <string.h>
#include <rtthread.h>
#include <drivers/sdio.h>
#include <drivers/mmcsd_core.h>

#include <drivers/mmcsd_spi.h>

#define MSD_TRACE

#ifdef MSD_TRACE
#define MSD_DEBUG(...)         rt_kprintf("[MMCSD_SPI] %d ", rt_tick_get()); rt_kprintf(__VA_ARGS__);
#else
#define MSD_DEBUG(...)
#endif /* #ifdef MSD_TRACE */

#define DUMMY                 0xFF

#define CARD_NCR_MAX          8

#define CARD_NRC              1
#define CARD_NCR              1

struct mmcsd_spi
{
    struct rt_mmcsd_host *host;
    struct rt_spi_device * spi_device;
    rt_uint8_t power_mode; /* current power mode. */
};

static struct mmcsd_spi  _mmcsd_spi_device;

/******************************************************************************/
static rt_bool_t rt_tick_timeout(rt_tick_t tick_start, rt_tick_t tick_long);

static rt_err_t _wait_ready(struct rt_spi_device* device)
{
    struct rt_spi_message message;
    rt_tick_t tick_start;
    uint8_t send, recv;

    tick_start = rt_tick_get();

    send = DUMMY;
    /* initial message */
    message.send_buf = &send;
    message.recv_buf = &recv;
    message.length = 1;
    message.cs_take = message.cs_release = 0;

    while(1)
    {
        /* transfer message */
        device->bus->ops->xfer(device, &message);

        if(recv != 0)
        {
            return RT_EOK;
        }

        if(rt_tick_timeout(tick_start, rt_tick_from_millisecond(10000)))
        {
            MSD_DEBUG("[err] wait ready timeout!\r\n");
            return RT_ETIMEOUT;
        }
    }
}

static rt_bool_t rt_tick_timeout(rt_tick_t tick_start, rt_tick_t tick_long)
{
    rt_tick_t tick_end = tick_start + tick_long;
    rt_tick_t tick_now = rt_tick_get();
    rt_bool_t result = RT_FALSE;

    if(tick_end >= tick_start)
    {
        if (tick_now >= tick_end)
        {
            result = RT_TRUE;
        }
        else
        {
            result = RT_FALSE;
        }
    }
    else
    {
        if ((tick_now < tick_start ) && (tick_now >= tick_end) )
        {
            result = RT_TRUE;
        }
        else
        {
            result = RT_FALSE;
        }
    }

    return result;
}
/******************************************************************************/

static rt_err_t _send_cmd(struct rt_spi_device* device,
                          struct rt_mmcsd_req *req)
{
    struct rt_spi_message message;
    uint8_t cmd_buffer[8];
    uint8_t recv_buffer[sizeof(cmd_buffer)];
    uint8_t response[MSD_RESPONSE_MAX_LEN];
    uint32_t i;
    uint8_t crc = 0;
    uint16_t rsponse_type = req->cmd->flags & RESP_SPI_MASK;

    cmd_buffer[0] = DUMMY;
    cmd_buffer[1] = (req->cmd->cmd_code | 0x40);
    cmd_buffer[2] = (uint8_t)(req->cmd->arg >> 24);
    cmd_buffer[3] = (uint8_t)(req->cmd->arg >> 16);
    cmd_buffer[4] = (uint8_t)(req->cmd->arg >> 8);
    cmd_buffer[5] = (uint8_t)(req->cmd->arg);

    if(crc == 0x00)
    {
        extern uint8_t mmcsd_spi_crc7(const void* buffer, uint32_t cnt);

        crc = mmcsd_spi_crc7(&cmd_buffer[1], 5);
        crc = (crc<<1) | 0x01;
    }
    cmd_buffer[6] = (crc);

    cmd_buffer[7] = DUMMY;

    /* initial message */
    message.send_buf = cmd_buffer;
    message.recv_buf = recv_buffer;
    message.length = sizeof(cmd_buffer);
    message.cs_take = message.cs_release = 0;

    //_wait_ready(device);

    /* transfer message */
    device->bus->ops->xfer(device, &message);

    for(i=CARD_NCR; i<(CARD_NCR_MAX+1); i++)
    {
        uint8_t send = DUMMY;

        /* initial message */
        message.send_buf = &send;
        message.recv_buf = response;
        message.length = 1;
        message.cs_take = message.cs_release = 0;

        /* transfer message */
        device->bus->ops->xfer(device, &message);

        if(0xFF != response[0])
        {
            break;
        }
    } /* wait response */

    if((CARD_NCR_MAX+1) == i)
    {
        return RT_ERROR;//fail
    }

    if (response[0] & 0x80)
    {
        MSD_DEBUG("response[0] & 0x80\r\n");
    }

    req->cmd->resp[0] = response[0];
//    MSD_DEBUG("response[0]:%02X\r\n", response[0]);

    //recieve other byte
    if(rsponse_type == RESP_SPI_R1)
    {
        return RT_EOK;
    }
    else if(rsponse_type == RESP_SPI_R1B)
    {
        rt_tick_t tick_start = rt_tick_get();
        uint8_t recv;

        while(1)
        {
            /* initial message */
            message.send_buf = RT_NULL;
            message.recv_buf = &recv;
            message.length = 1;
            message.cs_take = message.cs_release = 0;

            /* transfer message */
            device->bus->ops->xfer(device, &message);

            if(recv == DUMMY)
            {
                return RT_EOK;
            }

            if(rt_tick_timeout(tick_start, rt_tick_from_millisecond(2000)))
            {
                return RT_ETIMEOUT;
            }
        }
    }
    else if(rsponse_type == RESP_SPI_R2)
    {
        /* initial message */
        message.send_buf = RT_NULL;
        message.recv_buf = response+1;
        message.length = 1;
        message.cs_take = message.cs_release = 0;

        /* transfer message */
        device->bus->ops->xfer(device, &message);
        for(i=0; i<4; i++)
        {
            req->cmd->resp[i] = response[i+1];
        }
    }
    else if((rsponse_type == RESP_SPI_R3) || (rsponse_type == RESP_SPI_R7))
    {
        uint32_t tmp = 0;

        /* initial message */
        message.send_buf = RT_NULL;
        message.recv_buf = response+1;
        message.length = 4;
        message.cs_take = message.cs_release = 0;

        /* transfer message */
        device->bus->ops->xfer(device, &message);
        for(i=0; i<4; i++)
        {
            //req->cmd->resp[i] = response[i+1];
            tmp <<= 8;
            tmp |= response[i+1];
        }
        req->cmd->resp[1] = tmp;
    }
    else if(rsponse_type == RESP_SPI_R4)
    {
        MSD_DEBUG("unknow rsponse type:RESP_SPI_R4\r\n");
    }
    else if(rsponse_type == RESP_SPI_R5)
    {
        MSD_DEBUG("unknow rsponse type:RESP_SPI_R5\r\n");
    }
    else
    {
        MSD_DEBUG("unknow rsponse type:%d\r\n", rsponse_type);
        return RT_ERROR; // unknow rsponse type?
    }

    return RT_EOK;
}

static rt_err_t _wait_token(struct rt_spi_device* device, uint8_t token)
{
    struct rt_spi_message message;
    rt_tick_t tick_start;
    uint8_t send, recv;

    tick_start = rt_tick_get();

    /* wati token */
    /* initial message */
    send = DUMMY;
    message.send_buf = &send;
    message.recv_buf = &recv;
    message.length = 1;
    message.cs_take = message.cs_release = 0;

    while(1)
    {
        /* transfer message */
        device->bus->ops->xfer(device, &message);

        if(recv == token)
        {
            return RT_EOK;
        }

        if(rt_tick_timeout(tick_start, rt_tick_from_millisecond(CARD_WAIT_TOKEN_TIMES)))
        {
            MSD_DEBUG("[err] wait data start token timeout!\r\n");
            return RT_ETIMEOUT;
        }
    } /* wati token */
}

static rt_err_t _read_block(struct rt_spi_device* device, void * buffer, uint32_t block_size)
{
    struct rt_spi_message message;
    rt_err_t result;

    /* wati token */
    result = _wait_token(device, MSD_TOKEN_READ_START);
    if(result != RT_EOK)
    {
        return result;
    }

    /* read data */
    {
        /* initial message */
        message.send_buf = RT_NULL;
        message.recv_buf = buffer;
        message.length = block_size;
        message.cs_take = message.cs_release = 0;

        /* transfer message */
        device->bus->ops->xfer(device, &message);
    } /* read data */

    /* get crc */
    {
        uint8_t recv_buffer[2];

        /* initial message */
        message.send_buf = RT_NULL;
        message.recv_buf = recv_buffer;
        message.length = 2;
        message.cs_take = message.cs_release = 0;

        /* transfer message */
        device->bus->ops->xfer(device, &message);
    } /* get crc */

    return RT_EOK;
}

static rt_err_t _write_block(struct mmcsd_spi * mmcsd_spi,
                             const void * buffer,
                             uint32_t block_size,
                             uint8_t token)
{
    uint8_t send_buffer[16];
    struct rt_spi_message message;
    struct rt_spi_device* device = mmcsd_spi->spi_device;

    rt_memset(send_buffer, DUMMY, sizeof(send_buffer));
    send_buffer[sizeof(send_buffer) - 1] = token;

    /* send start block token */
    {
        /* initial message */
        message.send_buf = send_buffer;
        message.recv_buf = RT_NULL;
        message.length = sizeof(send_buffer);
        message.cs_take = message.cs_release = 0;

        /* transfer message */
        device->bus->ops->xfer(device, &message);
    }

    /* send data */
    {
        /* initial message */
        message.send_buf = buffer;
        message.recv_buf = RT_NULL;
        message.length = block_size;
        message.cs_take = message.cs_release = 0;

        /* transfer message */
        device->bus->ops->xfer(device, &message);
    }

    if(mmcsd_spi->host->spi_use_crc)
    {
        extern uint32_t mmcsd_spi_crc16(const void* buffer, uint32_t cnt);

        uint16_t crc = mmcsd_spi_crc16(buffer, block_size);

        send_buffer[0] = (crc >> 8) & 0xFF; /* [8:15] */
        send_buffer[1] = crc & 0xFF; /* [0:7] */
    }

    /* put crc and get data response */
    {
        uint8_t recv_buffer[3];
        uint8_t response;

        /* initial message */
        message.send_buf = send_buffer;
        message.recv_buf = recv_buffer;
        message.length = sizeof(recv_buffer);
        message.cs_take = message.cs_release = 0;

        /* transfer message */
        device->bus->ops->xfer(device, &message);

//        response = 0x0E & recv_buffer[2];
        response = MSD_GET_DATA_RESPONSE(recv_buffer[2]);
        if(response != MSD_DATA_OK)
        {
            if(response == MSD_DATA_CRC_ERROR)
            {
                MSD_DEBUG("[err] write block fail! DATA_CRC_ERROR\r\n");
            }
            else if(response == MSD_DATA_CRC_ERROR)
            {
                MSD_DEBUG("[err] write block fail! DATA_WRITE_ERROR\r\n");
            }
            else
            {
                MSD_DEBUG("[err] write block fail! data response : 0x%02X\r\n",
                          response);
            }
            return RT_ERROR;
        }
    }

    /* wati ready */
    return _wait_ready(device);
}
/******************************************************************************/

static void mmcsd_spi_request(struct rt_mmcsd_host *host,
                              struct rt_mmcsd_req *req)
{
    struct mmcsd_spi * mmcsd_spi = (struct mmcsd_spi *)host->private_data;
    rt_err_t result = RT_EOK;

//    MSD_DEBUG("mmcsd_spi_request cmd_code:%d\r\n", req->cmd->cmd_code);

    rt_spi_take_bus(mmcsd_spi->spi_device);
    rt_spi_take(mmcsd_spi->spi_device);

    result = _send_cmd(mmcsd_spi->spi_device, req);
    if (result != RT_EOK)
    {
        MSD_DEBUG("_send_cmd ERROR!\r\n");
    }

    if(req->data != RT_NULL)
    {
//        MSD_DEBUG("data>>>>>>>>\r\n");
//        MSD_DEBUG("blksize:%u\r\n", req->data->blksize);
//        MSD_DEBUG("blks:%u\r\n", req->data->blks);
//        MSD_DEBUG("flags:%u\r\n", req->data->flags);
//        MSD_DEBUG("bytes_xfered:%u\r\n", req->data->bytes_xfered);

        if(req->data->stop != RT_NULL)
        {
            MSD_DEBUG("data->stop>>>>>>>>\r\n");
            MSD_DEBUG("stop cmd:%u\r\n", req->data->stop->cmd_code);
        }
        if(req->data->mrq != RT_NULL)
        {
//            MSD_DEBUG("data->mrq>>>>>>>>\r\n");
//            MSD_DEBUG("mrq cmd:%u\r\n", req->data->mrq->cmd_code);
        }

        if(req->data->flags & DATA_DIR_WRITE)
        {
            MSD_DEBUG("DATA_DIR_WRITE, len:%u\r\n", req->data->blksize);
            result = _write_block(mmcsd_spi,
                                  req->data->buf,
                                  req->data->blksize,
                                  MSD_TOKEN_WRITE_SINGLE_START);
        }
        else if(req->data->flags & DATA_DIR_READ)
        {
            MSD_DEBUG("DATA_DIR_READ, len:%u\r\n", req->data->blksize);
            result = _read_block(mmcsd_spi->spi_device,
                                 req->data->buf,
                                 req->data->blksize);
        }
        else if(req->data->flags & DATA_STREAM)
        {
            MSD_DEBUG("DATA_STREAM\r\n");
        }
    }

    if(req->stop != RT_NULL)
    {
        MSD_DEBUG("stop>>>>>>>>\r\n");
        MSD_DEBUG("stop cmd:%u\r\n", req->stop->cmd_code);
    }

    rt_spi_release(mmcsd_spi->spi_device);
    rt_spi_release_bus(mmcsd_spi->spi_device);

    mmcsd_req_complete(host);
}

#if 1
static void mmc_spi_initsequence(struct rt_mmcsd_host *host)
{
    struct rt_spi_message message;
    char buf[18];
    struct mmcsd_spi * mmcsd_spi = (struct mmcsd_spi *)host->private_data;
    /* Try to be very sure any previous command has completed;
     * wait till not-busy, skip debris from any old commands.
     */
    //mmc_spi_wait_unbusy(host, r1b_timeout);
    //_wait_ready(mmcsd_spi->spi_device);
    //mmc_spi_readbytes(host, 10);
    /* initial message */
    struct rt_spi_configuration cfg;
    cfg.data_width = 8;
    cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible Modes 0 */
    cfg.max_hz = host->freq_min;
    rt_spi_configure(mmcsd_spi->spi_device, &cfg);
    rt_spi_take_bus(mmcsd_spi->spi_device);

    rt_memset(buf, DUMMY, 10);
    message.send_buf = buf;
    message.recv_buf = buf;
    message.length = 10;
    message.cs_take = message.cs_release = 0;
    mmcsd_spi->spi_device->bus->ops->xfer(mmcsd_spi->spi_device, &message);

    /*
     * Do a burst with chipselect active-high.  We need to do this to
     * meet the requirement of 74 clock cycles with both chipselect
     * and CMD (MOSI) high before CMD0 ... after the card has been
     * powered up to Vdd(min), and so is ready to take commands.
     *
     * Some cards are particularly needy of this (e.g. Viking "SD256")
     * while most others don't seem to care.
     *
     * Note that this is one of the places MMC/SD plays games with the
     * SPI protocol.  Another is that when chipselect is released while
     * the card returns BUSY status, the clock must issue several cycles
     * with chipselect high before the card will stop driving its output.
     */
    rt_spi_release(mmcsd_spi->spi_device);
    rt_memset(buf, DUMMY, 18);
    message.send_buf = buf;
    message.recv_buf = buf;
    message.length = 18;
    message.cs_take = message.cs_release = 0;
    mmcsd_spi->spi_device->bus->ops->xfer(mmcsd_spi->spi_device, &message);

    rt_spi_release_bus(mmcsd_spi->spi_device);
#if 0
    host->spi->mode |= SPI_CS_HIGH;
    if (spi_setup(host->spi) != 0) {
        /* Just warn; most cards work without it. */
        dev_warn(&host->spi->dev,
                "can't change chip-select polarity\n");
        host->spi->mode &= ~SPI_CS_HIGH;
    } else {
        mmc_spi_readbytes(host, 18);

        host->spi->mode &= ~SPI_CS_HIGH;
        if (spi_setup(host->spi) != 0) {
            /* Wot, we can't get the same setup we had before? */
            dev_err(&host->spi->dev,
                    "can't restore chip-select polarity\n");
        }
    }
#endif
}
#endif

/*
 * Set the IOCFG
 */
static void mmcsd_spi_set_iocfg(struct rt_mmcsd_host *host,
                                struct rt_mmcsd_io_cfg *io_cfg)
{
    struct mmcsd_spi * mmcsd_spi = (struct mmcsd_spi *)host->private_data;

    rt_kprintf("clock %dHz busmode %d powermode %d Vdd %04x\n", io_cfg->clock, io_cfg->bus_mode, io_cfg->power_mode, io_cfg->vdd);

    /* config spi */
    if(io_cfg->clock != 0)
    {
        struct rt_spi_configuration cfg;
        cfg.data_width = 8;
        cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible Modes 0 */
        cfg.max_hz = io_cfg->clock;
        rt_spi_configure(mmcsd_spi->spi_device, &cfg);
    } /* config spi */

    if(mmcsd_spi->power_mode != io_cfg->power_mode)
    {
        mmcsd_spi->power_mode = io_cfg->power_mode;
        if(io_cfg->power_mode == MMCSD_POWER_ON)
        {
            MSD_DEBUG("MMCSD_POWER_ON\r\n");
        }

        if(io_cfg->power_mode == MMCSD_POWER_UP)
        {
            MSD_DEBUG("MMCSD_POWER_UP\r\n");
            mmc_spi_initsequence(host);
        }

        if(io_cfg->power_mode == MMCSD_POWER_OFF)
        {
            MSD_DEBUG("MMCSD_POWER_OFF\r\n");
            rt_spi_take_bus(mmcsd_spi->spi_device);
            rt_spi_release(mmcsd_spi->spi_device);
            rt_spi_release_bus(mmcsd_spi->spi_device);
        }
    } /* mmcsd_spi->power_mode != io_cfg->power_mode */
}

rt_int32_t mmcsd_spi_detect(struct rt_mmcsd_host *host)
{
    MSD_DEBUG("mmcsd_spi_detect\r\n");
    return 0;
}

static void mmcsd_spi_enable_sdio_irq(struct rt_mmcsd_host *host,
                                      rt_int32_t enable)
{
    MSD_DEBUG("mmcsd_spi_enable_sdio_irq\r\n");
}

static const struct rt_mmcsd_host_ops ops =
{
    mmcsd_spi_request,
    mmcsd_spi_set_iocfg,
    mmcsd_spi_detect,
    mmcsd_spi_enable_sdio_irq,
};

rt_err_t spi_sd_init(const char * spi_device_name)
{
    struct rt_mmcsd_host *host;
    struct rt_spi_device * spi_device;

    rt_kprintf("spi_sd_init\r\n");

    spi_device = (struct rt_spi_device *)rt_device_find(spi_device_name);
    if(spi_device == RT_NULL)
    {
        MSD_DEBUG("spi device %s not found!\r\n", spi_device_name);
        return -RT_ENOSYS;
    }

    host = mmcsd_alloc_host();
    if (!host)
    {
        rt_kprintf("mmcsd_alloc_host error\r\n");
        return -RT_ERROR;
    }

    _mmcsd_spi_device.spi_device = spi_device;
    _mmcsd_spi_device.host = host;

    host->ops = &ops;
    host->freq_min = 400 * 1000; /* initial clock 400Kbps. */
    host->freq_max = 10 * 1000 * 1000;
    host->valid_ocr = VDD_32_33 | VDD_33_34;
    host->flags = MMCSD_HOST_IS_SPI | MMCSD_MUTBLKWRITE;

    host->max_seg_size = 65535;
    host->max_dma_segs = 2;
    host->max_blk_size = 512;
    host->max_blk_count = 4096;
    host->private_data = &_mmcsd_spi_device;

    mmcsd_change(host);

    return RT_EOK;
}
