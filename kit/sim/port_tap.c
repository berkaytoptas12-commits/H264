/*
 * port_tap.c - port.h on Linux: Ethernet frames through a TAP device, so the
 * firmware can be run against the real network stack of the PC.
 *
 *   KIT_TAP=h264tap0           TAP interface (created when missing, needs root)
 *   KIT_TAP_HOST=192.168.77.1  address given to the PC side of the TAP
 *   KIT_LOSS=0.02              drop this share of frames in each direction
 */
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <linux/if.h>
#include <linux/if_tun.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include "port.h"
#include "sim.h"

static int g_fd = -1;
static double g_loss;
static uint32_t g_dropped;

static int lose(void)
{
    if (g_loss > 0 && (double)rand() / RAND_MAX < g_loss) {
        g_dropped++;
        return 1;
    }
    return 0;
}

static void host_side(const char *ifname, const char *ip)
{
    struct ifreq ifr;
    struct sockaddr_in *sa = (struct sockaddr_in *)&ifr.ifr_addr;
    int s = socket(AF_INET, SOCK_DGRAM, 0);

    if (s < 0)
        return;
    memset(&ifr, 0, sizeof ifr);
    snprintf(ifr.ifr_name, IFNAMSIZ, "%s", ifname);
    sa->sin_family = AF_INET;
    inet_pton(AF_INET, ip, &sa->sin_addr);
    if (ioctl(s, SIOCSIFADDR, &ifr) < 0)
        perror("SIOCSIFADDR");
    inet_pton(AF_INET, "255.255.255.0", &sa->sin_addr);
    if (ioctl(s, SIOCSIFNETMASK, &ifr) < 0)
        perror("SIOCSIFNETMASK");
    if (ioctl(s, SIOCGIFFLAGS, &ifr) == 0) {
        ifr.ifr_flags |= IFF_UP | IFF_RUNNING;
        if (ioctl(s, SIOCSIFFLAGS, &ifr) < 0)
            perror("SIOCSIFFLAGS");
    }
    close(s);
}

int port_init(const uint8_t mac[6])
{
    struct ifreq ifr;
    const char *name = getenv("KIT_TAP"), *host = getenv("KIT_TAP_HOST");
    const char *loss = getenv("KIT_LOSS");

    (void)mac;
    if (!name)
        name = "h264tap0";
    if (!host)
        host = "192.168.77.1";
    g_loss = loss ? atof(loss) : 0.0;
    srand(1);
    g_fd = open("/dev/net/tun", O_RDWR);
    if (g_fd < 0) {
        perror("/dev/net/tun");
        exit(1);
    }
    memset(&ifr, 0, sizeof ifr);
    ifr.ifr_flags = IFF_TAP | IFF_NO_PI;
    snprintf(ifr.ifr_name, IFNAMSIZ, "%s", name);
    if (ioctl(g_fd, TUNSETIFF, &ifr) < 0) {
        perror("TUNSETIFF");
        exit(1);
    }
    fcntl(g_fd, F_SETFL, O_NONBLOCK);
    host_side(ifr.ifr_name, host);
    port_log("TAP %s, PC side %s, loss %.3f", ifr.ifr_name, host, g_loss);
    return 0;
}

int port_eth_send(const uint8_t *frame, uint32_t len)
{
    if (lose())
        return 0;               /* lost on the wire, not a local failure */
    return write(g_fd, frame, len) == (ssize_t)len ? 0 : -1;
}

uint32_t port_eth_recv(uint8_t *buf, uint32_t max)
{
    for (;;) {
        ssize_t n = read(g_fd, buf, max);

        if (n <= 0)
            return 0;
        if (!lose())
            return (uint32_t)n;
    }
}

void port_poll(void)
{
    struct pollfd p[2];
    int n = 1;

    /* sleep until a frame or a fabric response arrives (at most 1 ms) */
    p[0].fd = g_fd;
    p[0].events = POLLIN;
    p[1].fd = sim_fabric_fd();
    p[1].events = POLLIN;
    if (p[1].fd >= 0)
        n = 2;
    poll(p, (nfds_t)n, 1);
}

int port_link_up(void)
{
    return 1;
}

uint32_t port_ms(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000u + ts.tv_nsec / 1000000);
}

void port_log(const char *fmt, ...)
{
    va_list ap;

    fprintf(stderr, "[kit %7.3f] ", port_ms() / 1000.0);
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
}

uint32_t sim_dropped(void)
{
    return g_dropped;
}
