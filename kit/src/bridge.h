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
    uint32_t no_fabric;         /* datagrams dropped: h264_apb_bridge not found */
    uint32_t dgrams_out, bytes_out;
    uint32_t peer_changes;
} bridge_stats_t;

typedef enum { FABRIC_UNPROBED, FABRIC_MISSING, FABRIC_OK } fabric_state_t;

extern bridge_stats_t g_bridge_stats;

void bridge_init(void);
void bridge_udp(const net_peer_t *from, const uint8_t *data, uint32_t len);
void bridge_poll(void);

/* The fabric is probed at its first use (after the network is up, so that a
   hanging FIC3 access shows as "ping works, requests do not"), then again at
   most once a second while it is missing.  1 when h264_apb_bridge answers. */
int bridge_fabric_ready(void);
fabric_state_t bridge_fabric_state(void);

#endif
