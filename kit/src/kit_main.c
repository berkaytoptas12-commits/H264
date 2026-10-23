/*
 * kit_main.c - Ethernet <-> H.264 decoder bridge, main loop.
 * The MSS only moves bytes; parsing, retransmission and decoding stay in
 * the fabric (h264_link + h264_dec_top).
 */
#include "app_config.h"
#include "bridge.h"
#include "fabric.h"
#include "kit_main.h"
#include "net.h"
#include "port.h"

static uint8_t g_frame[1536];

static void show_ip(const char *what, uint32_t ip)
{
    port_log("%s %u.%u.%u.%u", what, (unsigned)(ip >> 24), (unsigned)(ip >> 16) & 255u,
             (unsigned)(ip >> 8) & 255u, (unsigned)ip & 255u);
}

static void stats(void)
{
    fabric_status_t st;

    fabric_status(&st);
    port_log("udp in %u (%u B, %u cut) out %u | eth rx %u drop %u tx fail %u | "
             "pictures %u %ux%u%s%s",
             (unsigned)g_bridge_stats.dgrams_in, (unsigned)g_bridge_stats.bytes_in,
             (unsigned)g_bridge_stats.short_writes, (unsigned)g_bridge_stats.dgrams_out,
             (unsigned)g_net_stats.rx_frames, (unsigned)g_net_stats.rx_drop,
             (unsigned)g_net_stats.tx_fail, (unsigned)st.pictures, (unsigned)st.width,
             (unsigned)st.height, st.error ? " ERROR" : "", st.done ? " done" : "");
}

void kit_main(void)
{
    static const uint8_t mac[6] = APP_MAC;
    uint32_t t_stats, t_fab;
    int link = 0, fab_ok;

    port_init(mac);
    port_log("H.264 decoder Ethernet bridge, UDP port %u", (unsigned)APP_UDP_PORT);
    show_ip("IP", APP_IP);
    fab_ok = fabric_init() == 0;
    if (!fab_ok)
        port_log("no h264_apb_bridge at 0x%08lx, check FABRIC_BASE / FIC3", (unsigned long)FABRIC_BASE);
    net_init(mac, APP_IP, APP_NETMASK, (uint16_t)APP_UDP_PORT, bridge_udp);
    bridge_init();
    t_stats = t_fab = port_ms();

    for (;;) {
        uint32_t n, k;

        port_poll();
        if (port_link_up() != link) {
            link = port_link_up();
            port_log("%s", link ? "link up" : "link down");
            if (link)
                net_announce();
        }
        if (!fab_ok) {
            if (port_ms() - t_fab >= 1000u) {
                t_fab = port_ms();
                fab_ok = fabric_init() == 0;
                if (fab_ok)
                    port_log("fabric bridge found");
            }
            while (port_eth_recv(g_frame, sizeof g_frame))
                ;               /* nothing to deliver to */
            continue;
        }
        /* a few frames, then the responses, so neither side starves */
        for (k = 0; k < 8u && (n = port_eth_recv(g_frame, sizeof g_frame)) != 0; k++)
            net_input(g_frame, n < sizeof g_frame ? n : sizeof g_frame);
        bridge_poll();
        if (APP_STATS_MS && port_ms() - t_stats >= APP_STATS_MS) {
            t_stats = port_ms();
            stats();
        }
    }
}
