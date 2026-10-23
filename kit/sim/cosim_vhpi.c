/*
 * cosim_vhpi.c - GHDL side of the firmware <-> RTL co-simulation
 * (libh264cosim.so, VHPIDIRECT functions used by tb/tb_cosim.vhd).
 * File descriptors from COSIM_RFD / COSIM_WFD (set by fabric_cosim.c).
 *
 *   cs_poll()   0 nothing yet, 1 read, 2 write, 3 end of input
 *   cs_addr()   register offset of the request
 *   cs_hi(), cs_lo()   write data, upper / lower 16 bits
 *   cs_reply(hi, lo)   answer a read
 * VHDL integers are 32 bit signed, so data travels in 16 bit halves.
 */
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>

static int g_rfd = -1, g_wfd = -1;
static uint8_t g_buf[9];
static int g_n;
static uint32_t g_off, g_data;

static void init(void)
{
    const char *r = getenv("COSIM_RFD"), *w = getenv("COSIM_WFD");

    g_rfd = r ? atoi(r) : 0;
    g_wfd = w ? atoi(w) : 1;
    fcntl(g_rfd, F_SETFL, fcntl(g_rfd, F_GETFL) | O_NONBLOCK);
}

static uint32_t get32(const uint8_t *p)
{
    return p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

int cs_poll(int dummy)
{
    (void)dummy;
    if (g_rfd < 0)
        init();
    while (g_n < 9) {
        ssize_t n = read(g_rfd, g_buf + g_n, (size_t)(9 - g_n));

        if (n == 0)
            return 3;
        if (n < 0)
            return (errno == EAGAIN || errno == EINTR) ? 0 : 3;
        g_n += (int)n;
    }
    g_n = 0;
    g_off = get32(g_buf + 1);
    g_data = get32(g_buf + 5);
    return g_buf[0] == 'W' ? 2 : 1;
}

int cs_addr(int dummy)
{
    (void)dummy;
    return (int)(g_off & 0xFFFu);
}

int cs_hi(int dummy)
{
    (void)dummy;
    return (int)(g_data >> 16);
}

int cs_lo(int dummy)
{
    (void)dummy;
    return (int)(g_data & 0xFFFFu);
}

int cs_reply(int hi, int lo)
{
    uint8_t r[4];
    uint32_t v = ((uint32_t)hi << 16) | ((uint32_t)lo & 0xFFFFu);
    int i = 0;

    r[0] = (uint8_t)v; r[1] = (uint8_t)(v >> 8); r[2] = (uint8_t)(v >> 16); r[3] = (uint8_t)(v >> 24);
    while (i < 4) {
        ssize_t n = write(g_wfd, r + i, (size_t)(4 - i));

        if (n <= 0)
            return -1;
        i += (int)n;
    }
    return 0;
}
