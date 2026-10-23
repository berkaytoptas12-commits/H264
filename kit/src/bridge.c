/*
 * bridge.c - UDP <-> h264_link tunnel (see bridge.h).
 */
#include <string.h>

#include "app_config.h"
#include "bridge.h"
#include "fabric.h"
#include "log.h"
#include "port.h"

#define RESP_LEN    16u                 /* size of one link response */
#define RESP_BUF    (64u * RESP_LEN)

bridge_stats_t g_bridge_stats;

static net_peer_t g_peer;
static int g_have_peer;
static uint8_t g_resp[RESP_BUF];
static uint32_t g_resp_n;
static fabric_state_t g_fab;
static uint32_t g_fab_t;

void bridge_init(void)
{
    memset(&g_bridge_stats, 0, sizeof g_bridge_stats);
    g_have_peer = 0;
    g_resp_n = 0;
    g_fab = FABRIC_UNPROBED;
}

fabric_state_t bridge_fabric_state(void)
{
    return g_fab;
}

int bridge_fabric_ready(void)
{
    if (g_fab == FABRIC_OK)
        return 1;
    if (g_fab == FABRIC_MISSING && port_ms() - g_fab_t < 1000u)
        return 0;
    if (g_fab == FABRIC_UNPROBED)
        kit_log("probing h264_apb_bridge at 0x%08lx", (unsigned long)FABRIC_BASE);
    g_fab_t = port_ms();
    if (fabric_init() == 0) {
        g_fab = FABRIC_OK;
        kit_log("fabric bridge found, link and decoder reset");
        return 1;
    }
    if (g_fab == FABRIC_UNPROBED)
        kit_log("no h264_apb_bridge at 0x%08lx (wrong ID): check the bitstream, FABRIC_BASE, FIC3",
                (unsigned long)FABRIC_BASE);
    g_fab = FABRIC_MISSING;
    return 0;
}

void bridge_udp(const net_peer_t *from, const uint8_t *data, uint32_t len)
{
    uint32_t n;

    if (!g_have_peer || from->ip != g_peer.ip || from->port != g_peer.port) {
        kit_log("peer %u.%u.%u.%u:%u",
                (unsigned)(from->ip >> 24), (unsigned)(from->ip >> 16) & 255u,
                (unsigned)(from->ip >> 8) & 255u, (unsigned)from->ip & 255u,
                (unsigned)from->port);
        g_bridge_stats.peer_changes++;
        g_resp_n = 0;           /* responses belong to the old peer */
    }
    g_peer = *from;
    g_have_peer = 1;
    g_bridge_stats.dgrams_in++;
    if (!bridge_fabric_ready()) {
        g_bridge_stats.no_fabric++;
        return;
    }
    n = fabric_write(data, len, BRIDGE_TX_TIMEOUT_MS);
    g_bridge_stats.bytes_in += n;
    if (n != len)
        g_bridge_stats.short_writes++;
}

static void flush(uint32_t n)
{
    if (net_udp_send(&g_peer, (uint16_t)APP_UDP_PORT, g_resp, n) == 0) {
        g_bridge_stats.dgrams_out++;
        g_bridge_stats.bytes_out += n;
    }
    memmove(g_resp, g_resp + n, g_resp_n - n);
    g_resp_n -= n;
}

void bridge_poll(void)
{
    uint32_t got;

    if (g_fab != FABRIC_OK)
        return;
    got = fabric_read(g_resp + g_resp_n, RESP_BUF - g_resp_n);
    g_resp_n += got;
    if (!g_have_peer) {
        g_resp_n = 0;
        return;
    }
    /* send whole responses once the fabric has nothing more for now */
    if (g_resp_n >= RESP_LEN && (got == 0 || g_resp_n > RESP_BUF - 4u * RESP_LEN))
        flush(g_resp_n - g_resp_n % RESP_LEN);
    else if (g_resp_n == RESP_BUF)
        flush(g_resp_n);        /* out of step: pass everything on, the host resyncs */
}
