/*
 * port.h - what the portable firmware needs from the platform.
 * port_mss.c implements it with the Microchip MSS MAC driver,
 * kit/sim/port_tap.c with a Linux TAP device.
 */
#ifndef PORT_H
#define PORT_H

#include <stdint.h>

/* Bring up the Ethernet MAC/PHY and the console.  0 on success. */
int port_init(const uint8_t mac[6]);

/* Queue one Ethernet frame (destination MAC .. payload, no FCS) for sending.
   0 on success, -1 when no transmit buffer is free (the frame is dropped). */
int port_eth_send(const uint8_t *frame, uint32_t len);

/* Take the next received frame.  Returns its length (0 = none) and copies at
   most `max` bytes into `buf`. */
uint32_t port_eth_recv(uint8_t *buf, uint32_t max);

/* Background work (link status, pending transmissions). */
void port_poll(void);

/* 1 when the Ethernet link is up. */
int port_link_up(void);

/* Free running millisecond counter. */
uint32_t port_ms(void);

/* Raw console output (UART / stderr) for log.c; must not block for long,
   may drop text when the output is busy. */
void port_console(const char *s, uint32_t n);

#endif
