/*
 * e51.c - hart entry points for a Microchip PolarFire SoC bare-metal project
 * (replaces src/application/hart0/e51.c of e.g. mpfs-mac-simple-test).
 * KIT_HART selects the hart that runs the bridge: 0 = E51 (default, no HSS),
 * 1 = U54_1 (when the HSS boots the image).
 */
#include "mpfs_hal/mss_hal.h"

#include "kit_main.h"

#ifndef KIT_HART
#define KIT_HART 0
#endif

static void run(void)
{
    write_csr(mscratch, 0);
    write_csr(mcause, 0);
    write_csr(mepc, 0);
    PLIC_init();
    kit_main();
}

#if KIT_HART == 0
void e51(void)
{
    run();
}
#else
void u54_1(void)
{
    run();
}
#endif
