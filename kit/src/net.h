/*
 * net.h - minimal Ethernet / ARP / IPv4 / ICMP echo / UDP for one port.
 * No fragmentation, no routing: replies go to the MAC address the request
 * came from (works through a gateway as well).
 */
#ifndef NET_H
#define NET_H

#include <stdint.h>

#define NET_MTU         1500u
#define NET_UDP_MAX     (NET_MTU - 20u - 8u)    /* largest UDP payload */

typedef struct {
    uint8_t  mac[6];
    uint32_t ip;
    uint16_t port;
} net_peer_t;

typedef void (*net_udp_handler_t)(const net_peer_t *from, const uint8_t *data, uint32_t len);

typedef struct {
    uint32_t rx_frames, rx_arp, rx_icmp, rx_udp;
    uint32_t rx_drop;       /* not for us, malformed, bad checksum, fragments */
    uint32_t tx_frames, tx_fail;
} net_stats_t;

extern net_stats_t g_net_stats;

void net_init(const uint8_t mac[6], uint32_t ip, uint32_t netmask, uint16_t udp_port,
              net_udp_handler_t handler);

/* Process one received frame; the buffer is reused for ARP / ping replies. */
void net_input(uint8_t *frame, uint32_t len);

/* Send `len` bytes (<= NET_UDP_MAX) from our UDP port to `to`.  0 on success. */
int net_udp_send(const net_peer_t *to, const uint8_t *data, uint32_t len);

/* Tell the network who we are (gratuitous ARP). */
void net_announce(void);

#endif
