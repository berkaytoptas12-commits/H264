/*
 * kit_main.c - Ethernet <-> H.264 decoder bridge, main loop.
 * The MSS only moves bytes; parsing, retransmission and decoding stay in
 * the fabric (h264_link + h264_dec_top).  The console is read by the PC
 * over UDP (service.c); a UART is optional.
 */
#include "app_config.h"
#include "bridge.h"
#include "fabric.h"
#include "kit_main.h"
#include "log.h"
#include "net.h"
#include "port.h"
#include "service.h"

static uint8_t g_frame[1536];

static void on_udp(uint16_t port, const net_peer_t *from, const uint8_t *data, uint32_t len)
{
    if (port == APP_UDP_PORT)
        bridge_udp(from, data, len);
    else if (port == APP_CONSOLE_PORT)
        service_udp(from, data, len);
}

/* counters on the console, only when something moved since the last line */
static void stats(void)
{
    static uint32_t last_in = ~0u, last_rx = ~0u;
    fabric_status_t st = { 0, 0, 0, 0, 0 };

    if (g_bridge_stats.dgrams_in == last_in && g_net_stats.rx_frames == last_rx)
        return;
    last_in = g_bridge_stats.dgrams_in;
    last_rx = g_net_stats.rx_frames;
    if (bridge_fabric_state() == FABRIC_OK)
        fabric_status(&st);
    kit_log("udp in %u (%u B, %u cut, %u no fabric) out %u | eth rx %u drop %u tx fail %u | "
            "pictures %u %ux%u%s%s",
            (unsigned)g_bridge_stats.dgrams_in, (unsigned)g_bridge_stats.bytes_in,
            (unsigned)g_bridge_stats.short_writes, (unsigned)g_bridge_stats.no_fabric,
            (unsigned)g_bridge_stats.dgrams_out, (unsigned)g_net_stats.rx_frames,
            (unsigned)g_net_stats.rx_drop, (unsigned)g_net_stats.tx_fail, (unsigned)st.pictures,
            (unsigned)st.width, (unsigned)st.height, st.error ? " ERROR" : "", st.done ? " done" : "");
}

void kit_main(void)
{
    static const uint8_t mac[6] = APP_MAC;
    unsigned long ip = APP_IP;
    uint32_t t_stats;
    int link = 0;

    port_init(mac);
    kit_log("%s: H.264 decoder Ethernet bridge", APP_VERSION);
    kit_log("IP %lu.%lu.%lu.%lu, MAC %02x:%02x:%02x:%02x:%02x:%02x, UDP %u (video) %u (console)",
            ip >> 24, (ip >> 16) & 255u, (ip >> 8) & 255u, ip & 255u,
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
            (unsigned)APP_UDP_PORT, (unsigned)APP_CONSOLE_PORT);
    kit_log("fabric (h264_apb_bridge at 0x%08lx) is probed at the first request",
            (unsigned long)FABRIC_BASE);
    net_init(mac, APP_IP, APP_NETMASK, on_udp);
    bridge_init();
    t_stats = port_ms();

    for (;;) {
        uint32_t n, k;

        port_poll();
        if (port_link_up() != link) {
            link = port_link_up();
            kit_log("%s", link ? "Ethernet link up" : "Ethernet link down");
            if (link)
                net_announce();
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
