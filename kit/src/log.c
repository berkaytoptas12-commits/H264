/*
 * log.c - console ring (see log.h).
 */
#include <stdarg.h>
#include <stdio.h>

#include "log.h"
#include "port.h"

static char g_ring[LOG_SIZE];
static uint32_t g_total;

static void put(const char *s, uint32_t n)
{
    uint32_t i;

    for (i = 0; i < n; i++)
        g_ring[(g_total + i) & (LOG_SIZE - 1u)] = s[i];
    g_total += n;
    port_console(s, n);
}

void kit_log(const char *fmt, ...)
{
    char buf[240];
    va_list ap;
    uint32_t ms = port_ms();
    int n, h;

    h = snprintf(buf, sizeof buf, "[%5u.%03u] ", (unsigned)(ms / 1000u), (unsigned)(ms % 1000u));
    va_start(ap, fmt);
    n = vsnprintf(buf + h, sizeof buf - (unsigned)h - 2u, fmt, ap);
    va_end(ap);
    if (n < 0)
        return;
    n += h;
    if (n > (int)sizeof buf - 3)
        n = (int)sizeof buf - 3;
    buf[n++] = '\r';
    buf[n++] = '\n';
    put(buf, (uint32_t)n);
}

uint32_t log_total(void)
{
    return g_total;
}

uint32_t log_read(uint32_t from, uint8_t *dst, uint32_t max, uint32_t *first)
{
    uint32_t oldest = g_total > LOG_SIZE ? g_total - LOG_SIZE : 0;
    uint32_t i, n;

    if (from < oldest || from > g_total)
        from = oldest;
    n = g_total - from;
    if (n > max)
        n = max;
    for (i = 0; i < n; i++)
        dst[i] = (uint8_t)g_ring[(from + i) & (LOG_SIZE - 1u)];
    *first = from;
    return n;
}
