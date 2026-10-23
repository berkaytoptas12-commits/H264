/*
 * net.c - minimal Ethernet / ARP / IPv4 / ICMP echo / UDP (see net.h).
 */
#include <string.h>

#include "net.h"
#include "port.h"

#define ETH_HLEN    14u
#define ET_IP       0x0800u
#define ET_ARP      0x0806u
#define IP_ICMP     1u
#define IP_UDP      17u

net_stats_t g_net_stats;

static uint8_t  g_mac[6];
static uint32_t g_ip, g_mask;
static uint16_t g_ip_id;
static net_udp_handler_t g_handler;
static uint8_t  g_tx[ETH_HLEN + NET_MTU];

static uint16_t get16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }
static uint32_t get32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}
static void put16(uint8_t *p, uint32_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v; }
static void put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16); p[2] = (uint8_t)(v >> 8); p[3] = (uint8_t)v;
}

/* one's complement sum, not yet folded / inverted */
static uint32_t csum_add(uint32_t s, const uint8_t *p, uint32_t n)
{
    while (n > 1u) {
        s += get16(p);
        p += 2;
        n -= 2u;
    }
    if (n)
        s += (uint32_t)p[0] << 8;
    return s;
}

static uint16_t csum_fold(uint32_t s)
{
    while (s >> 16)
        s = (s & 0xFFFFu) + (s >> 16);
    return (uint16_t)~s;
}

/* UDP checksum incl. pseudo header; `udp` points at the UDP header */
static uint16_t udp_csum(uint32_t src, uint32_t dst, const uint8_t *udp, uint32_t ulen)
{
    uint32_t s = 0;

    s += src >> 16; s += src & 0xFFFFu;
    s += dst >> 16; s += dst & 0xFFFFu;
    s += IP_UDP;
    s += ulen;
    return csum_fold(csum_add(s, udp, ulen));
}

static void send_frame(const uint8_t *f, uint32_t len)
{
    if (port_eth_send(f, len) == 0)
        g_net_stats.tx_frames++;
    else
        g_net_stats.tx_fail++;
}

void net_init(const uint8_t mac[6], uint32_t ip, uint32_t netmask, net_udp_handler_t handler)
{
    memcpy(g_mac, mac, 6);
    g_ip = ip;
    g_mask = netmask;
    g_handler = handler;
    memset(&g_net_stats, 0, sizeof g_net_stats);
}

static void arp_build(uint8_t *f, uint32_t op, const uint8_t *tha, uint32_t tpa)
{
    uint8_t *a = f + ETH_HLEN;

    memcpy(f, op == 1u ? (const uint8_t *)"\xff\xff\xff\xff\xff\xff" : tha, 6);
    memcpy(f + 6, g_mac, 6);
    put16(f + 12, ET_ARP);
    put16(a, 1u);               /* Ethernet */
    put16(a + 2, ET_IP);
    a[4] = 6;
    a[5] = 4;
    put16(a + 6, op);
    memcpy(a + 8, g_mac, 6);
    put32(a + 14, g_ip);
    memcpy(a + 18, tha, 6);
    put32(a + 24, tpa);
    memset(a + 28, 0, 60u - ETH_HLEN - 28u);    /* pad to the minimum frame size */
}

void net_announce(void)
{
    static const uint8_t zero[6];

    arp_build(g_tx, 1u, zero, g_ip);
    send_frame(g_tx, 60u);
}

static void arp_input(uint8_t *f, uint32_t len)
{
    const uint8_t *a = f + ETH_HLEN;
    uint8_t sha[6];
    uint32_t spa;

    if (len < ETH_HLEN + 28u || get16(a) != 1u || get16(a + 2) != ET_IP || a[4] != 6 || a[5] != 4)
        goto drop;
    if (get16(a + 6) != 1u || get32(a + 24) != g_ip)
        return;                 /* only requests for our address need an answer */
    memcpy(sha, a + 8, 6);
    spa = get32(a + 14);
    arp_build(g_tx, 2u, sha, spa);
    send_frame(g_tx, 60u);
    g_net_stats.rx_arp++;
    return;
drop:
    g_net_stats.rx_drop++;
}

static void icmp_input(uint8_t *f, uint8_t *ip, uint32_t ihl, uint32_t tot)
{
    uint8_t *ic = ip + ihl;
    uint32_t n = tot - ihl;

    if (n < 8u || csum_fold(csum_add(0, ic, n)) != 0) {
        g_net_stats.rx_drop++;
        return;
    }
    if (ic[0] != 8u)            /* echo request only */
        return;
    g_net_stats.rx_icmp++;
    /* turn the request around in place */
    memcpy(f, f + 6, 6);
    memcpy(f + 6, g_mac, 6);
    memcpy(ip + 16, ip + 12, 4);
    put32(ip + 12, g_ip);
    ip[8] = 64;
    put16(ip + 10, 0);
    put16(ip + 10, csum_fold(csum_add(0, ip, ihl)));
    ic[0] = 0;
    put16(ic + 2, 0);
    put16(ic + 2, csum_fold(csum_add(0, ic, n)));
    send_frame(f, ETH_HLEN + tot);
}

static void udp_input(const uint8_t *f, const uint8_t *ip, uint32_t ihl, uint32_t tot)
{
    const uint8_t *u = ip + ihl;
    uint32_t ulen;
    net_peer_t from;

    if (tot - ihl < 8u)
        goto drop;
    ulen = get16(u + 4);
    if (ulen < 8u || ulen > tot - ihl)
        goto drop;
    if (get16(u + 6) != 0 && udp_csum(get32(ip + 12), get32(ip + 16), u, ulen) != 0)
        goto drop;
    g_net_stats.rx_udp++;
    memcpy(from.mac, f + 6, 6);
    from.ip = get32(ip + 12);
    from.port = get16(u);
    if (g_handler)
        g_handler(get16(u + 2), &from, u + 8, ulen - 8u);
    return;
drop:
    g_net_stats.rx_drop++;
}

static void ip_input(uint8_t *f, uint32_t len)
{
    uint8_t *ip = f + ETH_HLEN;
    uint32_t ihl, tot, dst;

    if (len < ETH_HLEN + 20u || (ip[0] >> 4) != 4u)
        goto drop;
    ihl = (ip[0] & 15u) * 4u;
    tot = get16(ip + 2);
    if (ihl < 20u || tot < ihl || tot > len - ETH_HLEN)
        goto drop;
    if (csum_fold(csum_add(0, ip, ihl)) != 0)
        goto drop;
    dst = get32(ip + 16);
    if (dst != g_ip && dst != 0xFFFFFFFFu && dst != (g_ip | ~g_mask))
        return;
    if (get16(ip + 6) & 0x3FFFu)  /* MF or fragment offset */
        goto drop;
    if (ip[9] == IP_ICMP && dst == g_ip)
        icmp_input(f, ip, ihl, tot);
    else if (ip[9] == IP_UDP)
        udp_input(f, ip, ihl, tot);
    return;
drop:
    g_net_stats.rx_drop++;
}

void net_input(uint8_t *f, uint32_t len)
{
    uint32_t type;

    g_net_stats.rx_frames++;
    if (len < ETH_HLEN)
        goto drop;
    if (!(f[0] & 1u) && memcmp(f, g_mac, 6) != 0)
        return;                 /* unicast for someone else */
    type = get16(f + 12);
    if (type == ET_ARP)
        arp_input(f, len);
    else if (type == ET_IP)
        ip_input(f, len);
    return;
drop:
    g_net_stats.rx_drop++;
}

int net_udp_send(const net_peer_t *to, uint16_t sport, const uint8_t *data, uint32_t len)
{
    uint8_t *f = g_tx, *ip = f + ETH_HLEN, *u = ip + 20;
    uint32_t n;

    if (len > NET_UDP_MAX)
        return -1;
    memcpy(f, to->mac, 6);
    memcpy(f + 6, g_mac, 6);
    put16(f + 12, ET_IP);
    ip[0] = 0x45;
    ip[1] = 0;
    put16(ip + 2, 28u + len);
    put16(ip + 4, g_ip_id++);
    put16(ip + 6, 0x4000u);     /* don't fragment */
    ip[8] = 64;
    ip[9] = IP_UDP;
    put16(ip + 10, 0);
    put32(ip + 12, g_ip);
    put32(ip + 16, to->ip);
    put16(ip + 10, csum_fold(csum_add(0, ip, 20u)));
    put16(u, sport);
    put16(u + 2, to->port);
    put16(u + 4, 8u + len);
    put16(u + 6, 0);
    memcpy(u + 8, data, len);
    put16(u + 6, udp_csum(g_ip, to->ip, u, 8u + len));
    if (get16(u + 6) == 0)
        put16(u + 6, 0xFFFFu);
    n = ETH_HLEN + 28u + len;
    if (n < 60u) {
        memset(f + n, 0, 60u - n);
        n = 60u;
    }
    if (port_eth_send(f, n) != 0) {
        g_net_stats.tx_fail++;
        return -1;
    }
    g_net_stats.tx_frames++;
    return 0;
}
