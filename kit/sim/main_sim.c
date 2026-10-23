/*
 * main_sim.c - runs the bridge firmware as a Linux process (see Makefile).
 * SIGINT / SIGTERM: close the emulator input (it writes its --out file) and exit.
 */
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "bridge.h"
#include "kit_main.h"
#include "net.h"
#include "sim.h"

static void quit(int sig)
{
    (void)sig;
    fprintf(stderr, "[kit] udp in %u (%u bytes, %u cut) out %u, eth drop %u, lost (KIT_LOSS) %u\n",
            (unsigned)g_bridge_stats.dgrams_in, (unsigned)g_bridge_stats.bytes_in,
            (unsigned)g_bridge_stats.short_writes, (unsigned)g_bridge_stats.dgrams_out,
            (unsigned)g_net_stats.rx_drop, (unsigned)sim_dropped());
    sim_fabric_close();
    _exit(0);
}

int main(void)
{
    signal(SIGINT, quit);
    signal(SIGTERM, quit);
    setvbuf(stderr, NULL, _IOLBF, 0);
    kit_main();
    return 0;
}
