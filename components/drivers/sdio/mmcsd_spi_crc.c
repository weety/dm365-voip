#include <stdint.h>

#define crc7pm  0x89
#define crc16pm 0x11021

static uint32_t crc_byte(uint32_t cwd,
                         uint32_t plnm,
                         uint32_t dgt)
{
    uint32_t crcc;
    uint32_t crcp;
    uint32_t mxw;

    mxw = (0x1 << dgt) - 1;

    crcp = plnm << 8;
    crcc = 0x1 << (dgt + 8);
    while (cwd > mxw)
    {
        while ((cwd & crcc)==0)
        {
            crcc >>= 1;
            crcp >>= 1;
        }
        cwd ^= crcp;
    }

    return cwd;
}

static uint32_t crc_common(const uint8_t* dat,
                           uint32_t cnt,
                           uint32_t plnm,
                           uint32_t dgt)
{
    uint32_t cbr = 0;
    uint32_t cc = 0;

    while (cc < cnt)
    {
        cbr = crc_byte((cbr<<8)|dat[cc], plnm, dgt);
        cc++;
    }
    cc = dgt;
    while (cc > 8)
    {
        cbr = crc_byte(cbr<<8, plnm, dgt);
        cc -= 8;
    }
    cbr = crc_byte(cbr<<cc, plnm, dgt);

    return cbr;
}

uint8_t mmcsd_spi_crc7(const void* buffer, uint32_t cnt)
{
    return crc_common((const uint8_t*)buffer, cnt, crc7pm, 7);
}

uint32_t mmcsd_spi_crc16(const void* buffer, uint32_t cnt)
{
    return crc_common((const uint8_t*)buffer, cnt, crc16pm, 16);
}
