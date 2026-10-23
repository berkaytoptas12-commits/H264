/*
 * bridge.h - UDP <-> h264_link tunnel.
 *
 * Every UDP datagram that reaches APP_UDP_PORT carries link frames
 * (55 AA ...); its bytes go unchanged into the fabric TX FIFO.  The 16 byte
 * link responses read back from the fabric are sent to the host that sent the
 * last datagram.  Retransmission and flow control stay between the PC and
 * h264_link, so a lost or truncated datagram only costs a NAK / timeout.
 */
#ifndef BRIDGE_H
#define BRIDGE_H

#include <stdint.h>
#include "net.h"

typedef struct {
    uint32_t dgrams_in, bytes_in;
    uint32_t short_writes;      /* datagrams cut short: no room in the fabric FIFO */
    uint32_t dgrams_out, bytes_out;
    uint32_t peer_changes;
} bridge_stats_t;

extern bridge_stats_t g_bridge_stats;

void bridge_init(void);
void bridge_udp(const net_peer_t *from, const uint8_t *data, uint32_t len);
void bridge_poll(void);

#endif
