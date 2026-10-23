/*
 * fabric_mmio.c - h264_apb_bridge through memory mapped registers (FIC3 APB).
 */
#include "fabric.h"
#include "app_config.h"
#include "port.h"

#define REG(off) (*(volatile uint32_t *)(uintptr_t)(FABRIC_BASE + (off)))

static uint32_t g_tx_free;      /* entries known to be free (re-read when used up) */

int fabric_init(void)
{
    if (REG(FR_ID) != FABRIC_ID)
        return -1;
    fabric_reset();
    return 0;
}

void fabric_reset(void)
{
    uint32_t t0;

    REG(FR_CTRL) = 1u;
    t0 = port_ms();
    while (port_ms() - t0 < 2u)
        ;
    g_tx_free = 0;
}

/* wait until at least one TX entry is free */
static int tx_room(uint32_t t0, uint32_t timeout_ms)
{
    while (g_tx_free == 0) {
        g_tx_free = REG(FR_TXFREE);
        if (g_tx_free == 0 && port_ms() - t0 >= timeout_ms)
            return 0;
    }
    return 1;
}

uint32_t fabric_write(const uint8_t *p, uint32_t n, uint32_t timeout_ms)
{
    uint32_t t0 = port_ms();
    uint32_t i = 0;

    while (n - i >= 4u) {
        if (!tx_room(t0, timeout_ms))
            return i;
        REG(FR_TXDATA4) = (uint32_t)p[i] | ((uint32_t)p[i + 1] << 8) |
                          ((uint32_t)p[i + 2] << 16) | ((uint32_t)p[i + 3] << 24);
        g_tx_free--;
        i += 4u;
    }
    while (i < n) {
        if (!tx_room(t0, timeout_ms))
            return i;
        REG(FR_TXDATA) = p[i];
        g_tx_free--;
        i++;
    }
    return i;
}

uint32_t fabric_read(uint8_t *p, uint32_t max)
{
    uint32_t n = REG(FR_RXCOUNT);
    uint32_t i;

    if (n > max)
        n = max;
    for (i = 0; i < n; i++)
        p[i] = (uint8_t)REG(FR_RXDATA);
    return n;
}

void fabric_display_set(uint32_t ctrl)
{
    REG(FR_DISP_CTRL) = ctrl;
}

uint32_t fabric_display_ctrl(void)
{
    return REG(FR_DISP_CTRL);
}

uint32_t fabric_display_status(void)
{
    return REG(FR_DISP_STAT);
}

void fabric_status(fabric_status_t *st)
{
    uint32_t s = REG(FR_STATUS);
    uint32_t z = REG(FR_SIZE);

    st->error = s & 1u;
    st->done = (s >> 1) & 1u;
    st->pictures = REG(FR_PICTURES);
    st->width = z & 0xFFFu;
    st->height = (z >> 16) & 0xFFFu;
}
