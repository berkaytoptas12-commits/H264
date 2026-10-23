/* helpers shared by the Linux simulation files */
#ifndef SIM_H
#define SIM_H

#include <stdint.h>

int sim_fabric_fd(void);        /* fd to wait on for fabric responses, -1 if none */
void sim_fabric_close(void);    /* end of input for the emulator, wait for it */
uint32_t sim_dropped(void);     /* frames dropped by KIT_LOSS */

#endif
