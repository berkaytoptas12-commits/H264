/*
 * fabric_sim.c - fabric.h on Linux: the link frames go to host/fpga_emu.py
 * (a model of h264_link + decoder FIFO) through pipes.
 *
 *   KIT_EMU_ARGS="--out rx.264 --drain 2e6"   extra emulator arguments
 *   KIT_NO_FABRIC=1      the bridge's ID register reads wrong (no bitstream / wrong address)
 *   KIT_FABRIC_HANG=1    the first register access never returns (FIC3 not set up)
 */
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/wait.h>

#include "fabric.h"
#include "sim.h"

#ifndef EMU_PATH
#define EMU_PATH "host/fpga_emu.py"
#endif

static int g_to = -1, g_from = -1;
static pid_t g_pid = -1;
static fabric_status_t g_st;
static uint8_t g_scan[16];
static uint32_t g_disp_ctrl = 3000u << 16;
static uint32_t g_scan_n;

int fabric_init(void)
{
    int a[2], b[2];
    char *argv[32];
    char args[512];
    const char *extra = getenv("KIT_EMU_ARGS");
    int n = 0;

    if (g_pid > 0)
        return 0;
    if (getenv("KIT_FABRIC_HANG"))
        for (;;)
            pause();
    if (getenv("KIT_NO_FABRIC"))
        return -1;
    if (pipe(a) < 0 || pipe(b) < 0)
        return -1;
    argv[n++] = "python3";
    argv[n++] = EMU_PATH;
    argv[n++] = "--stdio";
    if (extra) {
        char *t;

        strncpy(args, extra, sizeof args - 1);
        args[sizeof args - 1] = 0;
        for (t = strtok(args, " "); t && n < 30; t = strtok(NULL, " "))
            argv[n++] = t;
    }
    argv[n] = NULL;
    g_pid = fork();
    if (g_pid == 0) {
        dup2(a[0], 0);
        dup2(b[1], 1);
        close(a[0]); close(a[1]); close(b[0]); close(b[1]);
        execvp(argv[0], argv);
        _exit(127);
    }
    close(a[0]);
    close(b[1]);
    g_to = a[1];
    g_from = b[0];
    fcntl(g_from, F_SETFL, O_NONBLOCK);
    signal(SIGPIPE, SIG_IGN);
    return 0;
}

void fabric_reset(void)
{
}

uint32_t fabric_write(const uint8_t *p, uint32_t n, uint32_t timeout_ms)
{
    uint32_t i = 0;

    (void)timeout_ms;
    while (i < n) {
        ssize_t w = write(g_to, p + i, n - i);

        if (w <= 0)
            break;
        i += (uint32_t)w;
    }
    return i;
}

/* follow the responses that pass by to answer fabric_status() */
static void scan(uint8_t c)
{
    if (g_scan_n == 0 && c != 0x5A)
        return;
    if (g_scan_n == 1 && c != 0xA5) {
        g_scan_n = c == 0x5A;
        return;
    }
    g_scan[g_scan_n++] = c;
    if (g_scan_n == 16) {
        const uint8_t *r = g_scan;

        g_st.pictures = r[6] | (r[7] << 8) | (r[8] << 16) | ((uint32_t)r[9] << 24);
        g_st.error = r[10] & 1u;
        g_st.done = (r[10] >> 1) & 1u;
        g_st.width = r[11] | ((r[12] & 15u) << 8);
        g_st.height = (r[12] >> 4) | (r[13] << 4);
        g_scan_n = 0;
    }
}

uint32_t fabric_read(uint8_t *p, uint32_t max)
{
    ssize_t n;
    uint32_t i;

    if (g_from < 0 || max == 0)
        return 0;
    n = read(g_from, p, max);
    if (n <= 0)
        return 0;
    for (i = 0; i < (uint32_t)n; i++)
        scan(p[i]);
    return (uint32_t)n;
}

void fabric_display_set(uint32_t ctrl)
{
    g_disp_ctrl = ctrl;
}

uint32_t fabric_display_ctrl(void)
{
    return g_disp_ctrl;
}

uint32_t fabric_display_status(void)
{
    return 0;
}

void fabric_status(fabric_status_t *st)
{
    *st = g_st;
}

int sim_fabric_fd(void)
{
    return g_from;
}

void sim_fabric_close(void)
{
    int status;

    if (g_pid <= 0)
        return;
    close(g_to);
    waitpid(g_pid, &status, 0);
    g_pid = -1;
}
