/*
 * service.c - console / status requests from the PC (see service.h).
 */
#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "bridge.h"
#include "fabric.h"
#include "log.h"
#include "port.h"
#include "service.h"

#define TEXT_MAX    1024u

static uint8_t g_out[16 + TEXT_MAX];

static uint32_t get32le(const uint8_t *p)
{
    return p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void put32le(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

static uint32_t info(char *t, uint32_t max)
{
    static const uint8_t mac[6] = APP_MAC;
    static const char *fab[] = { "unprobed", "missing", "ok" };
    unsigned long ip = APP_IP;
    fabric_status_t st;
    int n;

    bridge_fabric_ready();
    n = snprintf(t, max,
                 "fw=%s\nip=%lu.%lu.%lu.%lu\nmac=%02x:%02x:%02x:%02x:%02x:%02x\nuptime_ms=%lu\n"
                 "link=%d\nfabric=%s\nfabric_base=0x%08lx\n"
                 "udp_in=%lu\nudp_bytes=%lu\nudp_cut=%lu\nudp_no_fabric=%lu\nudp_out=%lu\n"
                 "eth_rx=%lu\neth_drop=%lu\neth_tx_fail=%lu\n",
                 APP_VERSION, ip >> 24, (ip >> 16) & 255u, (ip >> 8) & 255u, ip & 255u,
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], (unsigned long)port_ms(),
                 port_link_up(), fab[bridge_fabric_state()], (unsigned long)FABRIC_BASE,
                 (unsigned long)g_bridge_stats.dgrams_in, (unsigned long)g_bridge_stats.bytes_in,
                 (unsigned long)g_bridge_stats.short_writes, (unsigned long)g_bridge_stats.no_fabric,
                 (unsigned long)g_bridge_stats.dgrams_out, (unsigned long)g_net_stats.rx_frames,
                 (unsigned long)g_net_stats.rx_drop, (unsigned long)g_net_stats.tx_fail);
    if (n < 0 || (uint32_t)n >= max)
        return n < 0 ? 0 : max - 1u;
    if (bridge_fabric_state() == FABRIC_OK) {
        int m;

        fabric_status(&st);
        m = snprintf(t + n, max - (uint32_t)n,
                     "pictures=%lu\nwidth=%lu\nheight=%lu\ndec_error=%lu\ndec_done=%lu\n",
                     (unsigned long)st.pictures, (unsigned long)st.width, (unsigned long)st.height,
                     (unsigned long)st.error, (unsigned long)st.done);
        if (m > 0)
            n += m;
    }
    return (uint32_t)n < max ? (uint32_t)n : max - 1u;
}

void service_udp(const net_peer_t *from, const uint8_t *data, uint32_t len)
{
    uint32_t n;

    if (len < 5u || memcmp(data, "H264", 4) != 0)
        return;
    memcpy(g_out, "H264", 4);
    if (data[4] == 'L' && len >= 9u) {
        uint32_t first;

        g_out[4] = 'l';
        n = log_read(get32le(data + 5), g_out + 17, TEXT_MAX, &first);
        put32le(g_out + 5, first);
        put32le(g_out + 9, log_total());
        put32le(g_out + 13, port_ms());
        n += 17u;
    } else if (data[4] == 'I') {
        g_out[4] = 'i';
        n = 5u + info((char *)g_out + 5, TEXT_MAX);
    } else {
        return;
    }
    (void)net_udp_send(from, (uint16_t)APP_CONSOLE_PORT, g_out, n);
}
