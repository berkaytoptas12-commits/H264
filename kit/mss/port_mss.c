/*
 * port_mss.c - port.h on the PolarFire SoC MSS: Ethernet through the
 * Microchip MSS MAC driver (GEM, SGMII, VSC8662 PHY), time from the hart's
 * SysTick.  The console is read over Ethernet (service.c); a copy also goes
 * to an MMUART without blocking (KIT_UART_CONSOLE=0 leaves the UART alone).
 *
 * Board settings below default to the Icicle Kit style wiring (data on GEM0,
 * PHY VSC8662 port 0 at MDIO address 8, MDIO pins owned by GEM1).  Check
 * them against the Video Kit schematic / MSS Configurator and override with
 * -D on the compiler command line when they differ.
 */
#include <string.h>

#include "mpfs_hal/mss_hal.h"
#include "drivers/mss/mss_mmuart/mss_uart.h"
#include "drivers/mss/mss_ethernet_mac/mss_ethernet_registers.h"
#include "drivers/mss/mss_ethernet_mac/mss_ethernet_mac_sw_cfg.h"
#include "drivers/mss/mss_ethernet_mac/mss_ethernet_mac_regs.h"
#include "drivers/mss/mss_ethernet_mac/mss_ethernet_mac.h"
#include "drivers/mss/mss_ethernet_mac/phy.h"

#include "app_config.h"
#include "port.h"

#ifndef KIT_GEM                 /* MAC carrying the data: 0 = GEM0, 1 = GEM1 */
#define KIT_GEM         0
#endif
#ifndef KIT_MDIO_GEM            /* MAC whose MDIO pins reach the PHY */
#define KIT_MDIO_GEM    1
#endif
#ifndef KIT_PHY_ADDR            /* MSS_MAC_AUTO_DETECT_PHY_ADDRESS scans the bus */
#define KIT_PHY_ADDR    PHY_VSC8662_0_MDIO_ADDR
#endif
#ifndef KIT_UART_CONSOLE        /* 1: copy of the console on an MMUART (115200 8N1) */
#define KIT_UART_CONSOLE 1
#endif
#ifndef KIT_UART
#define KIT_UART        g_mss_uart0_lo
#define KIT_UART_PERIPH MSS_PERIPH_MMUART0
#endif
#ifndef KIT_HART                /* hart running the firmware: 0 = E51, 1 = U54_1 */
#define KIT_HART        0
#endif

#if KIT_GEM == 0
#define DATA_MAC        g_mac0
#define OTHER_MAC       g_mac1
#else
#define DATA_MAC        g_mac1
#define OTHER_MAC       g_mac0
#endif

#define RX_SLOTS        16u     /* software receive queue (frames) */
#define TX_SLOTS        8u      /* transmit buffers */
#define FRAME_MAX       1536u

extern mss_mac_instance_t g_mac0, g_mac1;

/* ---------------------------------------------------------------- time --- */
static volatile uint32_t g_ms;

#if KIT_HART == 0
void E51_sysTick_IRQHandler(void)
{
    g_ms += HART0_TICK_RATE_MS;
}
#else
void U54_1_sysTick_IRQHandler(void)
{
    g_ms += HART1_TICK_RATE_MS;
}
#endif

uint32_t port_ms(void)
{
    return g_ms;
}

/* ------------------------------------------------------------- console --- */
/* UART copy of the console: queued here, handed to the 16 byte TX FIFO from
   port_poll() whenever it is empty, so logging never waits for the UART */
#if KIT_UART_CONSOLE
#define CON_SIZE        2048u   /* power of two */
static char g_con[CON_SIZE];
static uint32_t g_con_wr, g_con_rd;
#endif

void port_console(const char *s, uint32_t n)
{
#if KIT_UART_CONSOLE
    uint32_t i;

    for (i = 0; i < n && g_con_wr - g_con_rd < CON_SIZE; i++)
        g_con[g_con_wr++ & (CON_SIZE - 1u)] = s[i];
#else
    (void)s;
    (void)n;
#endif
}

static void console_poll(void)
{
#if KIT_UART_CONSOLE
    uint8_t chunk[16];
    uint32_t n = 0;

    while (n < sizeof chunk && g_con_rd + n != g_con_wr) {
        chunk[n] = (uint8_t)g_con[(g_con_rd + n) & (CON_SIZE - 1u)];
        n++;
    }
    if (n)
        g_con_rd += (uint32_t)MSS_UART_fill_tx_fifo(&KIT_UART, chunk, n);
#endif
}

/* ------------------------------------------------------------ Ethernet --- */
static uint8_t g_mac_rx_buffer[MSS_MAC_RX_RING_SIZE][MSS_MAC_MAX_RX_BUF_SIZE]
    __attribute__((aligned(16)));

/* frames handed over by the receive interrupt (single producer / consumer) */
static uint8_t  g_rxq[RX_SLOTS][FRAME_MAX];
static uint32_t g_rxq_len[RX_SLOTS];
static volatile uint32_t g_rxq_wr, g_rxq_rd, g_rx_overflow;

/* transmit buffers: free -> queued (main loop) -> at the MAC -> free (tx interrupt) */
enum { TXB_FREE, TXB_QUEUED, TXB_MAC };
static uint8_t  g_txb[TX_SLOTS][FRAME_MAX] __attribute__((aligned(16)));
static uint32_t g_txb_len[TX_SLOTS];
static volatile uint8_t g_txb_state[TX_SLOTS];
static uint32_t g_txq[TX_SLOTS];
static uint32_t g_txq_wr, g_txq_rd;     /* main loop only */

static int g_link;
static uint32_t g_link_t;

static void rx_callback(void *this_mac, uint32_t queue_no, uint8_t *p_rx_packet,
                        uint32_t pckt_length, mss_mac_rx_desc_t *cdesc, void *p_user_data)
{
    uint32_t wr = g_rxq_wr;

    (void)cdesc;
    (void)p_user_data;
    if (wr - g_rxq_rd < RX_SLOTS && pckt_length <= FRAME_MAX) {
        memcpy(g_rxq[wr % RX_SLOTS], p_rx_packet, pckt_length);
        g_rxq_len[wr % RX_SLOTS] = pckt_length;
        g_rxq_wr = wr + 1u;
    } else {
        g_rx_overflow++;
    }
    /* give the buffer back to the MAC */
    (void)MSS_MAC_receive_pkt((mss_mac_instance_t *)this_mac, queue_no, p_rx_packet, 0,
                              MSS_MAC_INT_ENABLE);
}

static void tx_callback(void *this_mac, uint32_t queue_no, mss_mac_tx_desc_t *cdesc,
                        void *p_user_data)
{
    uintptr_t slot = (uintptr_t)p_user_data;

    (void)this_mac;
    (void)queue_no;
    (void)cdesc;
    if (slot < TX_SLOTS)
        g_txb_state[slot] = TXB_FREE;
}

/* Board specific PHY reset release; provide your own when the Video Kit
   design drives the PHY reset from an MSS GPIO. */
__attribute__((weak)) void kit_board_phy_reset(void)
{
}

int port_init(const uint8_t mac[6])
{
    mss_mac_cfg_t cfg;
    uint32_t i;

    (void)mss_config_clk_rst(MSS_PERIPH_FIC3, (uint8_t)KIT_HART, PERIPHERAL_ON);   /* APB to the decoder */
#if KIT_UART_CONSOLE
    (void)mss_config_clk_rst(KIT_UART_PERIPH, (uint8_t)KIT_HART, PERIPHERAL_ON);
    MSS_UART_init(&KIT_UART, MSS_UART_115200_BAUD,
                  MSS_UART_DATA_8_BITS | MSS_UART_NO_PARITY | MSS_UART_ONE_STOP_BIT);
#endif
    SysTick_Config();
    __enable_irq();

    kit_board_phy_reset();

    MSS_MAC_cfg_struct_def_init(&cfg);
    memcpy(cfg.mac_addr, mac, 6);
    cfg.speed_duplex_select = MSS_MAC_ANEG_ALL_SPEEDS;
    cfg.tsu_clock_select = 1U;
    cfg.phy_type = MSS_MAC_DEV_PHY_VSC8662;
    cfg.pcs_phy_addr = SGMII_MDIO_ADDR;
    cfg.interface_type = TBI;
    cfg.phy_autonegotiate = MSS_MAC_VSC8662_phy_autonegotiate;
    cfg.phy_mac_autonegotiate = MSS_MAC_VSC8662_mac_autonegotiate;
    cfg.phy_get_link_status = MSS_MAC_VSC8662_phy_get_link_status;
    cfg.phy_init = MSS_MAC_VSC8662_phy_init;
    cfg.phy_set_link_speed = MSS_MAC_VSC8662_phy_set_link_speed;
#if MSS_MAC_USE_PHY_DP83867
    cfg.phy_extended_read = NULL_mmd_read_extended_regs;
    cfg.phy_extended_write = NULL_mmd_write_extended_regs;
#endif

#if KIT_MDIO_GEM != KIT_GEM
    /* the other MAC only provides the MDIO interface */
    cfg.mac_addr[5] ^= 1u;
    cfg.phy_addr = PHY_VSC8662_1_MDIO_ADDR;
    MSS_MAC_init(&OTHER_MAC, &cfg);
    cfg.mac_addr[5] ^= 1u;
    cfg.phy_controller = &OTHER_MAC;
#endif
    cfg.phy_addr = KIT_PHY_ADDR;
    MSS_MAC_init(&DATA_MAC, &cfg);

    /* one transmit queue: give it all the segments (as Microchip's examples do) */
    DATA_MAC.mac_base->NETWORK_CONTROL &= ~GEM_ENABLE_TRANSMIT;
    DATA_MAC.mac_base->TX_Q_SEG_ALLOC_Q0TO3 = 2;
    DATA_MAC.mac_base->NETWORK_CONTROL |= GEM_ENABLE_TRANSMIT;

    MSS_MAC_set_tx_callback(&DATA_MAC, 0, tx_callback);
    MSS_MAC_set_rx_callback(&DATA_MAC, 0, rx_callback);
    for (i = 0; i < MSS_MAC_RX_RING_SIZE; i++)
        (void)MSS_MAC_receive_pkt(&DATA_MAC, 0, g_mac_rx_buffer[i], 0,
                                  i + 1u == MSS_MAC_RX_RING_SIZE ? MSS_MAC_INT_ARM
                                                                 : MSS_MAC_INT_DISABLE);
    return 0;
}

int port_eth_send(const uint8_t *frame, uint32_t len)
{
    uint32_t s;

    if (len > FRAME_MAX)
        return -1;
    for (s = 0; s < TX_SLOTS && g_txb_state[s] != TXB_FREE; s++)
        ;
    if (s == TX_SLOTS) {
        port_poll();
        return -1;
    }
    memcpy(g_txb[s], frame, len);
    g_txb_len[s] = len;
    g_txb_state[s] = TXB_QUEUED;
    g_txq[g_txq_wr % TX_SLOTS] = s;
    g_txq_wr++;
    port_poll();
    return 0;
}

uint32_t port_eth_recv(uint8_t *buf, uint32_t max)
{
    uint32_t rd = g_rxq_rd, n;

    if (rd == g_rxq_wr)
        return 0;
    n = g_rxq_len[rd % RX_SLOTS];
    memcpy(buf, g_rxq[rd % RX_SLOTS], n < max ? n : max);
    g_rxq_rd = rd + 1u;
    return n;
}

void port_poll(void)
{
    console_poll();

    /* start queued transmissions while the MAC takes them */
    while (g_txq_rd != g_txq_wr) {
        uint32_t s = g_txq[g_txq_rd % TX_SLOTS];

        g_txb_state[s] = TXB_MAC;
        if (MSS_MAC_send_pkt(&DATA_MAC, 0, g_txb[s], g_txb_len[s], (void *)(uintptr_t)s)
            != MSS_MAC_SUCCESS) {
            g_txb_state[s] = TXB_QUEUED;    /* MAC busy, try again on the next poll */
            break;
        }
        g_txq_rd++;
    }

    if (g_ms - g_link_t >= 250u) {
        mss_mac_speed_t speed;
        uint8_t fd;

        g_link_t = g_ms;
        g_link = MSS_MAC_get_link_status(&DATA_MAC, &speed, &fd) != 0;
    }
}

int port_link_up(void)
{
    return g_link;
}
