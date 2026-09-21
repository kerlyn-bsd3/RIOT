/*
 * Copyright (C) 2026 WPI MQP (6LoBAC) — Kerry Lynn
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * IPv6-over-MS/TP (6LoBAC / RFC 8163) bring-up on the Nucleo-F767ZI + DFRobot
 * RS485 shield. Attaches RIOT's gnrc 6LoWPAN/IPv6 stack to the clean-room MS/TP
 * driver so the node joins the token ring AND answers pings relayed onto the ring
 * by the 6LBR as Frame Type 34 — RIOT formulates the echo reply and calls our
 * send path (mstp_tx_ipv6) to transmit it.
 *
 * The netif ops below are the L2 shim: on TX, pull the 1-octet MS/TP destination
 * from the gnrc netif header and hand the raw MSDU to mstp_tx_ipv6(); on RX, wrap
 * the decoded MSDU (from the netdev) as a SIXLOWPAN payload with a netif header.
 * The device is presented to gnrc as NETDEV_TYPE_CC110X (1-byte L2 addr, 6lo) —
 * see mstp_netdev.c; confirm the IID against RFC 8163 6LoBAC.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "shell.h"
#include "iolist.h"
#include "net/gnrc/netif.h"
#include "net/gnrc/netif/hdr.h"
#include "net/gnrc/nettype.h"
#include "net/gnrc/pktbuf.h"

#include "mstp.h"

/* ------------------------------------------------------------------------- *
 * Configuration. The mstp driver is board-agnostic; only the values below differ
 * per board (MS/TP UART, RS-485 DE GPIO, and board TX/RX quirks). Selected on the
 * RIOT-provided BOARD_<name> macro.
 * ------------------------------------------------------------------------- */
#ifndef MSTP_BAUD
#define MSTP_BAUD           (115200U)      /* bus baud both ends agree on */
#endif
#ifndef MSTP_SRC_ADDR
#define MSTP_SRC_ADDR       (0x03U)        /* This Station; must be <= Nmax_manager (3..7) */
#endif

#if defined(BOARD_NRF52840DK)
/*
 * nRF52840-DK: RIOT already maps UART_DEV(1) = UARTE1 to the Arduino header D0/D1
 * (P1.01 rx / P1.02 tx) — where the DFR0259 shield's UART sits — and the DE line
 * is Arduino D2 = P1.03. RX is EasyDMA-driven (no 1-deep RDR to overrun), so there
 * is no ORE status register, no NVIC RX-priority bump, and no USART FIFO. TX is DMA
 * too: RIOT's blocking uart_write() returns at ENDTX — the end of the last data
 * bit, before the stop bit — so we append a trailing pad octet (tx_pad_octets) so
 * the clipped stop bit belongs to the discardable pad, not the CRC; see mstp_run.c.
 */
#define MSTP_BOARD_NAME     "nrf52840dk"
#define MSTP_UART_DEV       UART_DEV(1)
#define MSTP_DE_PIN         GPIO_PIN(1, 3)        /* Arduino D2 = P1.03 */
#define MSTP_TX_PAD_OCTETS  (1U)             /* one 0xFF pad; ENDTX fires pre-stop-bit */

#elif defined(BOARD_NUCLEO_F767ZI)
/*
 * Nucleo-F767ZI: USART6 = UART_DEV(1) on Arduino D0/D1 (PG9/PG14), DE = D2 = PF15.
 * The STM32F7 USART has no RX FIFO and RIOT reads one byte per interrupt, so we
 * (a) hand the driver the USART status register + ORE mask for overrun visibility,
 * and (b) raise the MS/TP and console RX IRQs to priority 0 (below, after
 * uart_init) so a same-priority ISR can't delay them past one char-time. TX:
 * uart_write() blocks to TC (line idle), so no trailing pad is needed.
 */
#define MSTP_BOARD_NAME     "nucleo-f767zi"
#define MSTP_UART_DEV       UART_DEV(1)
#define MSTP_DE_PIN         GPIO_PIN(PORT_F, 15)  /* Arduino D2 = PF15 */
#define MSTP_TX_PAD_OCTETS  (0U)             /* uart_write blocks to TC; no pad needed */
#define MSTP_ORE_SR         (&USART6->ISR)
#define MSTP_ORE_MASK       (USART_ISR_ORE)
#define MSTP_UART_IRQN      USART6_IRQn
#define MSTP_CONSOLE_IRQN   USART3_IRQn           /* stdio = UART_DEV(0) */
#define MSTP_UART_IRQ_PRIO  (0U)
#define MSTP_STM32_IRQ_PRIO 1                     /* compile the NVIC-priority block in main() */

#else
#error "mstp_6lo: unsupported BOARD - add a board-config block (MS/TP UART, DE pin, quirks)"
#endif

/* Common defaults (a board block above may already define MSTP_ORE_* / pad). */
#ifndef MSTP_ORE_SR
#define MSTP_ORE_SR         (NULL)        /* no overrun-register peek (e.g. nRF EasyDMA) */
#endif
#ifndef MSTP_ORE_MASK
#define MSTP_ORE_MASK       (0U)
#endif
#ifndef MSTP_TX_PAD_OCTETS
#define MSTP_TX_PAD_OCTETS  (0U)
#endif
#ifndef MSTP_NETIF_PRIO
#define MSTP_NETIF_PRIO     (GNRC_NETIF_PRIO)
#endif
#ifndef MSTP_NETIF_STACKSIZE
#define MSTP_NETIF_STACKSIZE (THREAD_STACKSIZE_DEFAULT)
#endif
#ifndef MSTP_STATUS_PERIOD_MS
#define MSTP_STATUS_PERIOD_MS (1000U)
#endif
/*
 * Nmax_manager (Max_Manager, 9.5.3): highest manager address polled for a
 * successor. Bound to the small test bed so POLL_FOR_MANAGER doesn't sweep the
 * whole 0..127 space; This Station (MSTP_SRC_ADDR) must be <= this.
 */
#ifndef MSTP_NMAX_MANAGER
#define MSTP_NMAX_MANAGER   (8U)
#endif

static mstp_t dev;
static gnrc_netif_t _netif;
static char _netif_stack[MSTP_NETIF_STACKSIZE];

/* One MSDU-gather buffer; the netif thread is the sole caller of _l2_send. */
static uint8_t _msdu[MSTP_MAX_MSDU];

/* Background diagnostic reporter: main() runs the shell, so the status lines
 * (identical in spirit to the mstp_ring app) print from their own low-priority
 * thread. The one signal that matters for the ping-reply goal is on the "data:"
 * line — ind counts pings handed up to RIOT, tx34 counts Type-34 frames we
 * transmitted. If a ping arrives (ind climbs, IND ft=34 in the trace) and RIOT's
 * icmpv6_echo formulates a reply, _l2_send -> mstp_tx_ipv6 makes tx34 climb and a
 * TX ft=34 appears — that is the whole path lighting up end to end. */
static char _status_stack[THREAD_STACKSIZE_DEFAULT + THREAD_EXTRA_STACKSIZE_PRINTF];

static const char *state_name(mstp_mgr_state_t s)
{
    switch (s) {
        case MSTP_MGR_INITIALIZE:          return "INITIALIZE";
        case MSTP_MGR_IDLE:                return "IDLE";
        case MSTP_MGR_USE_TOKEN:           return "USE_TOKEN";
        case MSTP_MGR_WAIT_FOR_REPLY:      return "WAIT_FOR_REPLY";
        case MSTP_MGR_DONE_WITH_TOKEN:     return "DONE_WITH_TOKEN";
        case MSTP_MGR_PASS_TOKEN:          return "PASS_TOKEN";
        case MSTP_MGR_NO_TOKEN:            return "NO_TOKEN";
        case MSTP_MGR_POLL_FOR_MANAGER:    return "POLL_FOR_MANAGER";
        case MSTP_MGR_ANSWER_DATA_REQUEST: return "ANSWER_DATA_REQUEST";
        default:                           return "?";
    }
}

/* Toggle for the status reporter so the shell is usable during interactive
 * tests (ifconfig / nib / ping). Type `status` to silence it, `status` to
 * restore. The thread keeps its 1 s cadence; it just skips the prints. */
static volatile bool _status_on = false;

static int _cmd_status(int argc, char **argv)
{
    (void)argc; (void)argv;
    _status_on = !_status_on;
    printf("mstp status %s\n", _status_on ? "on" : "off");
    return 0;
}
SHELL_COMMAND(status, "toggle the periodic mstp status printing", _cmd_status);

static void *_status_thread(void *arg)
{
    (void)arg;
    while (1) {
        ztimer_sleep(ZTIMER_MSEC, MSTP_STATUS_PERIOD_MS);
        if (!_status_on) {
            continue;
        }

        printf("state=%-16s TS=%u NS=%u PS=%u sole=%d TokenCount=%u\n"
               "   rx ok=%lu inv=%lu [hdrcrc=%lu abort=%lu(ore=%lu) "
               "datacrc=%lu cobs=%lu rxerr=%lu] txdrop=%lu\n",
               state_name(dev.mgr.state), dev.mgr.ts, dev.mgr.ns, dev.mgr.ps,
               (int)dev.mgr.sole_manager, (unsigned)dev.mgr.token_count,
               (unsigned long)dev.rx.stats.frames_ok,
               (unsigned long)dev.rx.stats.frames_inv,
               (unsigned long)dev.rx.stats.header_crc_err,
               (unsigned long)dev.rx.stats.frame_abort,
               (unsigned long)dev.rx.stats.abort_with_ore,
               (unsigned long)dev.rx.stats.data_crc_err,
               (unsigned long)dev.rx.stats.cobs_err,
               (unsigned long)dev.rx.stats.receive_error,
               (unsigned long)dev.txing_drop);

        /* THE key line for the ping-reply test: ind = pings handed to RIOT,
         * tx34 = Type-34 frames we transmitted (echo replies once RIOT answers). */
        printf("   data: ind=%lu (last ft=%u len=%u)  tx34=%lu\n",
               (unsigned long)dev.rx_ind,
               (unsigned)dev.rx_ind_last_ft,
               (unsigned)dev.rx_ind_last_len,
               (unsigned long)dev.tx_ipv6);

        /* Drain the event trace, skipping routine Token traffic (summarised by
         * the FSM's own counters); show every RXINV plus any non-Token frame so
         * the ping (IND/RX ft=34) and our reply (TX ft=34) are visible. */
        while (dev.ev_tail != dev.ev_head) {
            const mstp_ev_t *e = &dev.evlog[dev.ev_tail & (MSTP_EVLOG_LEN - 1U)];
            dev.ev_tail++;
            if (e->ev != MSTP_EV_RXINV && e->ft == MSTP_FT_TOKEN) {
                continue;
            }
            const char *tag = (e->ev == MSTP_EV_RX)  ? "RX   "
                            : (e->ev == MSTP_EV_TX)  ? "TX   "
                            : (e->ev == MSTP_EV_IND) ? "IND  "
                            :                          "RXINV";
            if (e->ev == MSTP_EV_RXINV) {
                printf("   %10lu us  %s  ft=%-2u src=%-3u dst=%-3u  [%s] idx=%u\n",
                       (unsigned long)e->t_us, tag, (unsigned)e->ft,
                       (unsigned)e->src, (unsigned)e->dst,
                       state_name((mstp_mgr_state_t)e->st), (unsigned)e->aux);
            }
            else if (e->ev == MSTP_EV_IND) {
                printf("   %10lu us  %s  ft=%-2u src=%-3u dst=%-3u  [%s] len=%u\n",
                       (unsigned long)e->t_us, tag, (unsigned)e->ft,
                       (unsigned)e->src, (unsigned)e->dst,
                       state_name((mstp_mgr_state_t)e->st), (unsigned)e->aux);
            }
            else {
                printf("   %10lu us  %s  ft=%-2u src=%-3u dst=%-3u  [%s]\n",
                       (unsigned long)e->t_us, tag, (unsigned)e->ft,
                       (unsigned)e->src, (unsigned)e->dst,
                       state_name((mstp_mgr_state_t)e->st));
            }
        }
    }
    return NULL;
}

/* ------------------------------------------------------------------------- *
 * gnrc_netif ops — the L2 shim between gnrc and the MS/TP engine.
 * ------------------------------------------------------------------------- */
static int _l2_send(gnrc_netif_t *netif, gnrc_pktsnip_t *pkt)
{
    mstp_t *d = (mstp_t *)netif->dev;

    if (pkt == NULL) {
        return -EINVAL;
    }
    if (pkt->type != GNRC_NETTYPE_NETIF) {
        gnrc_pktbuf_release(pkt);
        return -EBADMSG;
    }

    gnrc_netif_hdr_t *nh = pkt->data;
    gnrc_pktsnip_t   *payload = pkt->next;
    uint8_t dst;

    if (nh->flags & (GNRC_NETIF_HDR_FLAGS_BROADCAST | GNRC_NETIF_HDR_FLAGS_MULTICAST)) {
        dst = 0xFFU;                                  /* MS/TP broadcast */
    }
    else if (nh->dst_l2addr_len == 1U) {
        dst = gnrc_netif_hdr_get_dst_addr(nh)[0];
    }
    else {
        gnrc_pktbuf_release(pkt);
        return -EBADMSG;
    }

    ssize_t len = iolist_to_buffer((iolist_t *)payload, _msdu, sizeof(_msdu));
    if (len <= 0) {
        gnrc_pktbuf_release(pkt);
        return -EMSGSIZE;
    }

    int res = mstp_tx_ipv6(d, dst, _msdu, (uint16_t)len);
    gnrc_pktbuf_release(pkt);
    return (res == 0) ? (int)len : res;
}

static gnrc_pktsnip_t *_l2_recv(gnrc_netif_t *netif)
{
    mstp_t   *d  = (mstp_t *)netif->dev;
    netdev_t *nd = netif->dev;

    int len = nd->driver->recv(nd, NULL, 0, NULL);     /* bytes available */
    if (len <= 0) {
        return NULL;
    }
    uint8_t src = d->rx_data_src;                       /* capture before drain */

    gnrc_pktsnip_t *pkt = gnrc_pktbuf_add(NULL, NULL, len, GNRC_NETTYPE_SIXLOWPAN);
    if (pkt == NULL) {
        nd->driver->recv(nd, NULL, len, NULL);          /* drop */
        return NULL;
    }
    if (nd->driver->recv(nd, pkt->data, len, NULL) <= 0) {
        gnrc_pktbuf_release(pkt);
        return NULL;
    }

    uint8_t me = d->params.mac_addr;
    gnrc_pktsnip_t *nh = gnrc_pktbuf_add(NULL, NULL,
                                         sizeof(gnrc_netif_hdr_t) + 2U,
                                         GNRC_NETTYPE_NETIF);
    if (nh == NULL) {
        gnrc_pktbuf_release(pkt);
        return NULL;
    }
    gnrc_netif_hdr_init(nh->data, 1U, 1U);
    gnrc_netif_hdr_set_src_addr(nh->data, &src, 1U);
    gnrc_netif_hdr_set_dst_addr(nh->data, &me, 1U);
    gnrc_netif_hdr_set_netif(nh->data, netif);

    pkt = gnrc_pkt_append(pkt, nh);
    return pkt;
}

static const gnrc_netif_ops_t _mstp_netif_ops = {
    .init = gnrc_netif_default_init,   /* calls netdev->driver->init == mstp_start */
    .send = _l2_send,
    .recv = _l2_recv,
    .get  = gnrc_netif_get_from_netdev,
    .set  = gnrc_netif_set_from_netdev,
};

int main(void)
{
    printf("\n6LoBAC: IPv6-over-MS/TP (RFC 8163) on %s\n", MSTP_BOARD_NAME);
    printf("UART=%u baud=%lu TS=%u pad=%u\n",
           (unsigned)MSTP_UART_DEV, (unsigned long)MSTP_BAUD, MSTP_SRC_ADDR,
           (unsigned)MSTP_TX_PAD_OCTETS);

    const mstp_params_t params = {
        .uart     = MSTP_UART_DEV,
        .baud     = MSTP_BAUD,
        .de_pin   = MSTP_DE_PIN,
        .mac_addr = MSTP_SRC_ADDR,
        .ore_sr   = MSTP_ORE_SR,
        .ore_mask = MSTP_ORE_MASK,
        .tx_pad_octets = MSTP_TX_PAD_OCTETS,
    };
    mstp_setup(&dev, &params, 0);

    int res = gnrc_netif_create(&_netif, _netif_stack, sizeof(_netif_stack),
                                MSTP_NETIF_PRIO, "mstp", &dev.netdev,
                                &_mstp_netif_ops);
    if (res < 0) {
        printf("FATAL: gnrc_netif_create failed (%d)\n", res);
        return 1;
    }

#ifdef MSTP_STM32_IRQ_PRIO
    /* STM32 only: gnrc_netif_create() ran the netif thread's init == mstp_start()
     * == uart_init() to completion before returning, so the vector is live; raise
     * the MS/TP and console RX IRQs to priority 0 (the overrun fix). Not needed on
     * nRF (EasyDMA RX), where MSTP_STM32_IRQ_PRIO is undefined. */
    NVIC_SetPriority(MSTP_UART_IRQN, MSTP_UART_IRQ_PRIO);
    NVIC_SetPriority(MSTP_CONSOLE_IRQN, MSTP_UART_IRQ_PRIO);
    printf("UART RX IRQ (#%d) priority set to %u; console IRQ (#%d) matched\n",
           (int)MSTP_UART_IRQN, (unsigned)MSTP_UART_IRQ_PRIO,
           (int)MSTP_CONSOLE_IRQN);
#endif

    /* Bound the successor sweep to the 8-node bed. mstp_start() (run inside
     * gnrc_netif_create above) already called mstp_mgr_init(), which set
     * nmax_manager to the 127 default; override it now, preserving the
     * nmax_info_frames the init established. The FSM thread reads nmax_manager
     * each step, so this takes effect on the next poll. */
    mstp_mgr_set_limits(&dev.mgr, MSTP_NMAX_MANAGER, dev.mgr.nmax_info_frames);
    printf("Nmax_manager=%u Nmax_info_frames=%u\n",
           (unsigned)dev.mgr.nmax_manager, (unsigned)dev.mgr.nmax_info_frames);

    /* gnrc auto-configures the IPv6 link-local address and starts 6LoWPAN-ND
     * (Router Solicitation, address registration) only on NETDEV_EVENT_LINK_UP;
     * there is no init-time fallback (gnrc_netif.c: NETDEV_EVENT_LINK_UP ->
     * GNRC_IPV6_NIB_IFACE_UP). Our netdev has no PHY link to report, but the
     * MS/TP link is up as soon as the engine runs, and gnrc_netif_create() above
     * has fully initialised + registered the interface — so raise it once now.
     * Without this, ifconfig shows no inet6 address at all. */
    if (dev.netdev.event_callback != NULL) {
        dev.netdev.event_callback(&dev.netdev, NETDEV_EVENT_LINK_UP);
        puts("signalled LINK_UP -> link-local + Router Solicitation");
    }

    puts("mstp netif up — use 'ifconfig' to see the address; ping from the 6LBR");
    puts("status printing is OFF; type 'status' to toggle the periodic report on");

    thread_create(_status_stack, sizeof(_status_stack),
                  THREAD_PRIORITY_MAIN + 1, 0,
                  _status_thread, NULL, "mstp_status");

    char line[SHELL_DEFAULT_BUFSIZE];
    shell_run(NULL, line, sizeof(line));
    return 0;
}
