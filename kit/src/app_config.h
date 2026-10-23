/*
 * app_config.h - settings of the Ethernet <-> decoder bridge firmware.
 * Every value can be overridden from the compiler command line (-D...).
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* Network identity of the kit (locally administered MAC address). */
#ifndef APP_MAC
#define APP_MAC         { 0x02, 0x00, 0x00, 0x48, 0x32, 0x64 }
#endif
#ifndef APP_IP
#define APP_IP          IP4(192, 168, 10, 2)
#endif
#ifndef APP_NETMASK
#define APP_NETMASK     IP4(255, 255, 255, 0)
#endif
#ifndef APP_UDP_PORT
#define APP_UDP_PORT    5000u
#endif

/* Base address of h264_apb_bridge (FIC3 APB window, see the Libero memory map). */
#ifndef FABRIC_BASE
#define FABRIC_BASE     0x40000000ul
#endif

/* How long a datagram may wait for room in the fabric TX FIFO (ms). */
#ifndef BRIDGE_TX_TIMEOUT_MS
#define BRIDGE_TX_TIMEOUT_MS    20u
#endif

/* Statistics on the console every N ms (0 = off). */
#ifndef APP_STATS_MS
#define APP_STATS_MS    5000u
#endif

#define IP4(a, b, c, d) (((unsigned long)(a) << 24) | ((unsigned long)(b) << 16) | \
                         ((unsigned long)(c) << 8) | (unsigned long)(d))

#endif
