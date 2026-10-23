/*
 * fabric_cosim.c - register access for fabric_mmio.c (built with
 * FABRIC_SIM_REGS) against the RTL running in GHDL (tb/tb_cosim.vhd), so the
 * firmware's own fabric driver talks to the real h264_apb_bridge.
 *
 *   KIT_COSIM_CMD="ghdl -r ... tb_cosim ..."   simulator command line
 *
 * The simulator is started at the first access with two pipes (file
 * descriptors in COSIM_RFD / COSIM_WFD, see cosim_vhpi.c).  Messages: 'W' or
 * 'R', offset (4 bytes LE), data (4 bytes LE); a read is answered with 4
 * bytes.  Writes are posted, the pipe keeps them in order with the reads.
 */
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/wait.h>

#include "sim.h"

uint32_t sim_reg_read(uint32_t off);
void sim_reg_write(uint32_t off, uint32_t v);

static int g_to = -1, g_from = -1;
static pid_t g_pid = -1;

static void start(void)
{
    int a[2], b[2];
    char *argv[64];
    char args[2048], env[64];
    const char *cmd = getenv("KIT_COSIM_CMD");
    char *t;
    int n = 0;

    if (!cmd) {
        fprintf(stderr, "KIT_COSIM_CMD not set\n");
        exit(1);
    }
    if (pipe(a) < 0 || pipe(b) < 0) {
        perror("pipe");
        exit(1);
    }
    snprintf(args, sizeof args, "%s", cmd);
    for (t = strtok(args, " "); t && n < 62; t = strtok(NULL, " "))
        argv[n++] = t;
    argv[n] = NULL;
    signal(SIGPIPE, SIG_IGN);
    g_pid = fork();
    if (g_pid == 0) {
        close(a[1]);
        close(b[0]);
        snprintf(env, sizeof env, "%d", a[0]);
        setenv("COSIM_RFD", env, 1);
        snprintf(env, sizeof env, "%d", b[1]);
        setenv("COSIM_WFD", env, 1);
        execvp(argv[0], argv);
        _exit(127);
    }
    close(a[0]);
    close(b[1]);
    g_to = a[1];
    g_from = b[0];
}

static void put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

static void send_msg(char op, uint32_t off, uint32_t v)
{
    uint8_t m[9];
    uint32_t i = 0;

    if (g_pid < 0)
        start();
    m[0] = (uint8_t)op;
    put32(m + 1, off);
    put32(m + 5, v);
    while (i < sizeof m) {
        ssize_t w = write(g_to, m + i, sizeof m - i);

        if (w <= 0) {
            fprintf(stderr, "cosim: simulator gone\n");
            exit(1);
        }
        i += (uint32_t)w;
    }
}

void sim_reg_write(uint32_t off, uint32_t v)
{
    send_msg('W', off, v);
}

uint32_t sim_reg_read(uint32_t off)
{
    uint8_t r[4];
    uint32_t i = 0;

    send_msg('R', off, 0);
    while (i < sizeof r) {
        ssize_t n = read(g_from, r + i, sizeof r - i);

        if (n <= 0) {
            fprintf(stderr, "cosim: simulator gone\n");
            exit(1);
        }
        i += (uint32_t)n;
    }
    return r[0] | ((uint32_t)r[1] << 8) | ((uint32_t)r[2] << 16) | ((uint32_t)r[3] << 24);
}

int sim_fabric_fd(void)
{
    return -1;
}

void sim_fabric_close(void)
{
    int status;

    if (g_pid <= 0)
        return;
    close(g_to);                /* end of input: the testbench writes its results and stops */
    waitpid(g_pid, &status, 0);
    g_pid = -1;
}
