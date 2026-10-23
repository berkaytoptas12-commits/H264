/*
 * fabric.h - access to h264_apb_bridge (rtl/h264_apb_bridge.vhd).
 * fabric_mmio.c implements it with memory mapped registers,
 * kit/sim/fabric_sim.c with a pipe to host/fpga_emu.py.
 */
#ifndef FABRIC_H
#define FABRIC_H

#include <stdint.h>

#define FABRIC_ID       0x48323634u     /* "H264" */

/* register offsets */
#define FR_ID           0x00u
#define FR_CAPS         0x04u
#define FR_TXFREE       0x08u
#define FR_TXDATA       0x0Cu
#define FR_TXDATA4      0x10u
#define FR_RXCOUNT      0x14u
#define FR_RXDATA       0x18u
#define FR_STATUS       0x1Cu
#define FR_PICTURES     0x20u
#define FR_SIZE         0x24u
#define FR_CTRL         0x28u
#define FR_DISP_CTRL    0x2Cu   /* [0] off, [1] colour bars, [2] bit reversed lanes, [31:16] fps*100 */
#define FR_DISP_STAT    0x30u   /* [15:0] frame switches, [31:16] lines shown black (late) */

typedef struct {
    uint32_t error;         /* decoder reported an unsupported / broken stream */
    uint32_t done;          /* end of stream reached */
    uint32_t pictures;
    uint32_t width, height;
} fabric_status_t;

/* Check the ID register and reset link + decoder.  0 when the bridge answers. */
int fabric_init(void);

/* Write the bytes of a link frame into the TX FIFO.  Gives up after
   timeout_ms without room; returns the number of bytes written. */
uint32_t fabric_write(const uint8_t *p, uint32_t n, uint32_t timeout_ms);

/* Read up to `max` response bytes from the RX FIFO, returns the count. */
uint32_t fabric_read(uint8_t *p, uint32_t max);

void fabric_status(fabric_status_t *st);

/* Reset link and decoder, flush both FIFOs. */
void fabric_reset(void);

/* HDMI display (h264_disp): control word (see FR_DISP_CTRL) and status. */
void fabric_display_set(uint32_t ctrl);
uint32_t fabric_display_ctrl(void);
uint32_t fabric_display_status(void);

#endif
