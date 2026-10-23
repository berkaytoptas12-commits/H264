/*
 * log.h - the firmware's console: every line goes into a RAM ring (read by
 * the PC over UDP, see service.c) and to port_console() (UART / stderr).
 */
#ifndef LOG_H
#define LOG_H

#include <stdint.h>

#define LOG_SIZE    8192u           /* ring size, power of two */

/* one console line, printf style (no floating point), "[s.ms] " prefix */
void kit_log(const char *fmt, ...);

/* bytes written since boot (offset of the next byte) */
uint32_t log_total(void);

/* copy up to `max` bytes starting at offset `from` (older text that has
   been overwritten is skipped); *first = offset of the first byte copied */
uint32_t log_read(uint32_t from, uint8_t *dst, uint32_t max, uint32_t *first);

#endif
