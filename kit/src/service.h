/*
 * service.h - console and status for the PC on UDP APP_CONSOLE_PORT, so the
 * kit needs no UART.  Requests and answers start with "H264" and a command:
 *
 *   'L' + offset (4 bytes LE)  ->  'l' + first (4) + total (4) + uptime_ms (4)
 *                                  + console text from offset `first` (<= 1024
 *                                  bytes); uptime going back = the kit restarted
 *   'I'                        ->  'i' + "key=value" lines (version, uptime,
 *                                  link, fabric state, counters, decoder, display)
 *   'R' + ctrl (4 bytes LE)    ->  'r' + ctrl read back: HDMI display control
 *                                  (DISP_CTRL: fps * 100 in [31:16], flags)
 *
 * Every request is answered on its own (no session), a lost datagram is
 * simply asked for again.  'I' probes the fabric if that has not happened.
 */
#ifndef SERVICE_H
#define SERVICE_H

#include <stdint.h>
#include "net.h"

void service_udp(const net_peer_t *from, const uint8_t *data, uint32_t len);

#endif
