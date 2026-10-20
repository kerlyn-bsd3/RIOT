/*
 * Copyright (C) 2026 WPI MQP (6LoBAC) — Kerry Lynn
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * 6LoBAC border router (6LBR): bridges an MS/TP 6LoWPAN link (downstream, RFC
 * 8163) to an IPv6 uplink over SLIP (RFC 1055) to a Linux host running tunslip6.
 *
 * Two interfaces:
 *   - DOWNSTREAM: the clean-room MS/TP driver (mstp), This Station = 2, presented
 *     to gnrc as a 1-byte-L2 6lo interface. This is the 6LBR-facing side: it
 *     advertises the prefix (RA with PIO A=1/L=0 + 6CO + ABRO) and answers 6LoWPAN
 *     address registration (NS(ARO) -> NA(ARO)) — the piece the aging Contiki BDK
 *     lacked, so a 6LN registers natively here (no manual `ifconfig add`).
 *   - UPSTREAM (UPLINK=slip, default): slipdev on UART_DEV(0) (the nRF52840-DK's
 *     on-board J-Link VCP) at SLIP_BAUDRATE (1 Mbaud verified); the host runs
 *     dist/tools/sliptty/start_network.sh.
 *     UPSTREAM (UPLINK=cdc-ecm): USB Ethernet (usbus_cdc_ecm) on the nRF native USB;
 *     the host sees a cdc_ether netdev and runs ./start_ecm.sh. Either way the
 *     uplink link-local is fe80::2 with a default route via the host's fe80::1.
 *
 * The 6LBR setup in main() is the static (hardcoded) equivalent of RIOT's uhcp
 * client (sys/.../gnrc_uhcpc.c): install aaaa::/64 on the downstream (prefix +
 * our own address + ABR), enable router advertisements on it, and point a default
 * route out the slip upstream toward the host's fe80::1. We hardcode it rather
 * than run the uhcp client because our mstp netif is created here in main(), after
 * auto-init has already run — too late for auto_init_gnrc_uhcpc to see it.
 *
 * Console note: SLIP owns the J-Link VCP, so stdio is on the nRF native USB
 * CDC-ACM (a 2nd /dev/ttyACM*). If the native USB port does not enumerate, switch
 * the Makefile from stdio_cdc_acm to stdio_rtt and use `make term-rtt` (console
 * over the same J-Link, no native-USB dependency).
 */

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "msg.h"
#include "shell.h"
#include "iolist.h"
#include "net/gnrc/netif.h"
#include "net/gnrc/netif/hdr.h"
#include "net/gnrc/netif/internal.h"   /* gnrc_netif_ipv6_add_prefix() */
#include "net/gnrc/nettype.h"
#include "net/gnrc/pktbuf.h"
#include "net/gnrc/netapi.h"
#include "net/gnrc/ipv6/nib.h"         /* gnrc_ipv6_nib_change_rtr_adv_iface() */
#include "net/gnrc/ipv6/nib/ft.h"      /* gnrc_ipv6_nib_ft_add() */
#include "net/ipv6/addr.h"
#include "net/netopt.h"

#include "mstp.h"
#include "periph/gpio.h"

/* ------------------------------------------------------------------------- *
 * Configuration. The mstp driver is board-agnostic; only the values below differ
 * per board (MS/TP UART, RS-485 DE GPIO, and board TX/RX quirks). Selected on the
 * RIOT-provided BOARD_<name> macro.
 * ------------------------------------------------------------------------- */
#ifndef MSTP_BAUD
#define MSTP_BAUD           (115200U)      /* bus baud both ends agree on */
#endif
#ifndef MSTP_SRC_ADDR
#define MSTP_SRC_ADDR       (0x02U)        /* This Station (6LBR); <= Nmax_manager */
#endif

/* 6LBR addressing — hardcoded, matching the Contiki BDK test bed. */
#ifndef BR_PREFIX
#define BR_PREFIX           "aaaa::"       /* downstream prefix, advertised A=1/L=0 */
#endif
#ifndef BR_PREFIX_LEN
#define BR_PREFIX_LEN       (64U)
#endif
#ifndef BR_SLIP_NODE_LL
#define BR_SLIP_NODE_LL     "fe80::2"      /* our slip link-local (start_network.sh routes the prefix here) */
#endif
#ifndef BR_SLIP_HOST_LL
#define BR_SLIP_HOST_LL     "fe80::1"      /* host tun link-local = default-route next hop */
#endif

#if defined(BOARD_NRF52840DK)
/*
 * nRF52840-DK: UART_DEV(1) = UARTE1 = Arduino D0/D1 (P1.01 rx / P1.02 tx) carries
 * MS/TP; DE = Arduino D2 = P1.03. RX is EasyDMA-driven (no ORE, no NVIC bump, no
 * FIFO). TX is DMA too: uart_write() returns at ENDTX (before the stop bit), so we
 * append a trailing pad octet so the clipped stop bit belongs to the pad.
 * SLIP uses UART_DEV(0) = UARTE0 = the J-Link VCP (set in the Makefile).
 */
#define MSTP_BOARD_NAME     "nrf52840dk"
#define MSTP_UART_DEV       UART_DEV(1)
#define MSTP_DE_PIN         GPIO_PIN(1, 3)        /* Arduino D2 = P1.03 */
#define MSTP_TX_PAD_OCTETS  (1U)             /* one 0xFF pad; ENDTX fires pre-stop-bit */
/*
 * J-Link OB VCOM auto-detects UART hardware flow control from the target's RTS line
 * (nRF52840-DK: P0.05 -> OB). RIOT leaves UART0 RTS unconnected, so the pin floats
 * and the OB's HWFC decision is arbitrary; per Nordic/SEGGER, RTS *high* before the
 * first transfer makes the OB keep HWFC off. Suspected cause of SLIP failing above
 * 115200 (2026-10-14). Set -DSLIP_OB_RTS_HIGH=0 to disable.
 */
#ifndef SLIP_OB_RTS_HIGH
#define SLIP_OB_RTS_HIGH    (1)
#endif
#define SLIP_OB_RTS_PIN     GPIO_PIN(0, 5)
/*
 * RIOT's nRF UARTE RX takes ONE byte per EasyDMA transfer (MAXCNT=1, ENDRX->STARTRX
 * short into the same 1-byte buffer), so the ISR must read each byte before the
 * next lands: ~10-13 us at 1 Mbaud vs 87 us at 115200. At the default IRQ priority
 * (2, shared with USB/console and the MS/TP UARTE1) bytes get overwritten and SLIP
 * frames are silently dropped (2026-10-15: Saleae shows clean 1M requests on P0.08,
 * no reply on P0.06). Raise UARTE0 to priority 0. -DSLIP_UART_IRQ_PRIO0=0 disables.
 */
#ifndef SLIP_UART_IRQ_PRIO0
#define SLIP_UART_IRQ_PRIO0 (1)
#endif
#define SLIP_UART_IRQN      UARTE0_UART0_IRQn

#elif defined(BOARD_NUCLEO_F767ZI)
/*
 * Nucleo-F767ZI: USART6 = UART_DEV(1) on Arduino D0/D1 (PG9/PG14), DE = D2 = PF15.
 * STM32F7 USART has no RX FIFO and RIOT reads one byte per interrupt, so hand the
 * driver the ORE register + mask and raise the MS/TP and console RX IRQs to prio 0.
 * (As a BR the F767 is secondary; SLIP UART assignment would need its own review.)
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
#error "mstp_6lbr: unsupported BOARD - add a board-config block (MS/TP UART, DE pin, quirks)"
#endif

/* Common defaults (a board block above may already define MSTP_ORE_* / pad). */
#ifndef MSTP_ORE_SR
#define MSTP_ORE_SR         (NULL)
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
#ifndef MSTP_NMAX_MANAGER
#define MSTP_NMAX_MANAGER   (8U)
#endif

static mstp_t dev;
static gnrc_netif_t _netif;
static char _netif_stack[MSTP_NETIF_STACKSIZE];

/* One MSDU-gather buffer; the netif thread is the sole caller of _l2_send. */
static uint8_t _msdu[MSTP_MAX_MSDU];

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

/* Toggle for the status reporter so the shell stays usable during tests. */
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

        printf("   data: ind=%lu (last ft=%u len=%u)  tx34=%lu\n",
               (unsigned long)dev.rx_ind,
               (unsigned)dev.rx_ind_last_ft,
               (unsigned)dev.rx_ind_last_len,
               (unsigned long)dev.tx_ipv6);

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
 * gnrc_netif ops — the L2 shim between gnrc and the MS/TP engine (downstream).
 * Identical to the 6LN app: the MS/TP side of a 6LBR is just a 6lo node that
 * also forwards and advertises.
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

/* ------------------------------------------------------------------------- *
 * 6LBR plumbing: find the slip uplink (auto-inited before main), and wire the
 * static routing/prefix. This mirrors sys/.../gnrc_uhcpc.c set_interface_roles()
 * + uhcp_handle_prefix(), but with a hardcoded prefix.
 * ------------------------------------------------------------------------- */
static gnrc_netif_t *_find_uplink(const gnrc_netif_t *downstream)
{
    gnrc_netif_t *netif = NULL;
    while ((netif = gnrc_netif_iter(netif))) {
        if (netif == downstream) {
            continue;
        }
        /* prefer an interface that reports as wired (slip); otherwise take the
         * one that isn't our MS/TP downstream. */
        int is_wired = gnrc_netapi_get(netif->pid, NETOPT_IS_WIRED, 0, NULL, 0);
        if (is_wired == 1) {
            return netif;
        }
    }
    /* fallback: first interface that isn't the downstream */
    netif = NULL;
    while ((netif = gnrc_netif_iter(netif))) {
        if (netif != downstream) {
            return netif;
        }
    }
    return NULL;
}

static int _setup_6lbr(gnrc_netif_t *downstream)
{
    gnrc_netif_t *uplink = _find_uplink(downstream);
    if (uplink == NULL) {
        puts("6LBR: no uplink netif found — is slipdev / usbus_cdc_ecm auto-init enabled?");
        return -ENODEV;
    }

    /* UPSTREAM (slip): our link-local = fe80::2 (start_network.sh routes the
     * prefix here), default route out via the host's fe80::1. */
    ipv6_addr_t node_ll, host_ll, defroute = IPV6_ADDR_UNSPECIFIED;
    ipv6_addr_from_str(&node_ll, BR_SLIP_NODE_LL);
    ipv6_addr_from_str(&host_ll, BR_SLIP_HOST_LL);
    gnrc_netapi_set(uplink->pid, NETOPT_IPV6_ADDR, 64 << 8,
                    &node_ll, sizeof(node_ll));
    int r = gnrc_ipv6_nib_ft_add(&defroute, 0, &host_ll, uplink->pid, 0);
    if (r < 0) {
        printf("6LBR: default route add failed (%d)\n", r);
    }

    /* DOWNSTREAM (mstp): install aaaa::/64 — adds our global address, sets the
     * prefix list (RA PIO A=1/L=0), and registers us as the ABR (6CO/ABRO). */
    ipv6_addr_t prefix;
    ipv6_addr_from_str(&prefix, BR_PREFIX);
    int idx = gnrc_netif_ipv6_add_prefix(downstream, &prefix, BR_PREFIX_LEN,
                                         UINT32_MAX, UINT32_MAX);
    if (idx < 0) {
        printf("6LBR: add_prefix(%s/%u) failed (%d)\n",
               BR_PREFIX, (unsigned)BR_PREFIX_LEN, idx);
        return idx;
    }
    /* make the downstream actively advertise the subnet */
    gnrc_ipv6_nib_change_rtr_adv_iface(downstream, true);

    printf("6LBR up: downstream if=%u advertising %s/%u (A=1/L=0);"
           " uplink if=%u ll=%s, default via %s\n",
           (unsigned)downstream->pid, BR_PREFIX, (unsigned)BR_PREFIX_LEN,
           (unsigned)uplink->pid, BR_SLIP_NODE_LL, BR_SLIP_HOST_LL);
    return 0;
}

/* main thread runs the shell; 'ping' registers it with gnrc_netreg for echo
 * replies, which requires a message queue (gnrc_netreg asserts otherwise). */
#define MAIN_QUEUE_SIZE     (8)
static msg_t _main_msg_queue[MAIN_QUEUE_SIZE];

int main(void)
{
    msg_init_queue(_main_msg_queue, MAIN_QUEUE_SIZE);
#if defined(BR_UPLINK_SLIP) && defined(SLIP_OB_RTS_PIN) && SLIP_OB_RTS_HIGH
    /* tell the J-Link OB "no HWFC" before the host opens the VCP (see above) */
    gpio_init(SLIP_OB_RTS_PIN, GPIO_OUT);
    gpio_set(SLIP_OB_RTS_PIN);
#endif
#if defined(BR_UPLINK_SLIP) && defined(SLIP_UART_IRQN) && SLIP_UART_IRQ_PRIO0
    /* slipdev auto-init already enabled the IRQ; just raise its priority */
    NVIC_SetPriority(SLIP_UART_IRQN, 0);
#endif
#if defined(BR_UPLINK_CDC_ECM)
    printf("\n6LoBAC 6LBR: MS/TP (RFC 8163) <-> USB CDC-ECM uplink on %s\n", MSTP_BOARD_NAME);
    printf("MS/TP UART=%u baud=%lu TS=%u pad=%u | uplink=CDC-ECM (native USB) | prefix %s/%u\n",
           (unsigned)MSTP_UART_DEV, (unsigned long)MSTP_BAUD, MSTP_SRC_ADDR,
           (unsigned)MSTP_TX_PAD_OCTETS, BR_PREFIX, (unsigned)BR_PREFIX_LEN);
#else
    printf("\n6LoBAC 6LBR: MS/TP (RFC 8163) <-> SLIP uplink on %s\n", MSTP_BOARD_NAME);
    printf("MS/TP UART=%u baud=%lu TS=%u pad=%u | SLIP=UART0 @%lu | prefix %s/%u\n",
           (unsigned)MSTP_UART_DEV, (unsigned long)MSTP_BAUD, MSTP_SRC_ADDR,
           (unsigned)MSTP_TX_PAD_OCTETS, (unsigned long)SLIPDEV_PARAM_BAUDRATE,
           BR_PREFIX, (unsigned)BR_PREFIX_LEN);
#endif

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
    NVIC_SetPriority(MSTP_UART_IRQN, MSTP_UART_IRQ_PRIO);
    NVIC_SetPriority(MSTP_CONSOLE_IRQN, MSTP_UART_IRQ_PRIO);
    printf("UART RX IRQ (#%d) priority set to %u; console IRQ (#%d) matched\n",
           (int)MSTP_UART_IRQN, (unsigned)MSTP_UART_IRQ_PRIO,
           (int)MSTP_CONSOLE_IRQN);
#endif

    mstp_mgr_set_limits(&dev.mgr, MSTP_NMAX_MANAGER, dev.mgr.nmax_info_frames);
    printf("Nmax_manager=%u Nmax_info_frames=%u\n",
           (unsigned)dev.mgr.nmax_manager, (unsigned)dev.mgr.nmax_info_frames);

    /* LINK_UP forms the downstream link-local and starts 6LoWPAN-ND. See 6LN app. */
    if (dev.netdev.event_callback != NULL) {
        dev.netdev.event_callback(&dev.netdev, NETDEV_EVENT_LINK_UP);
        puts("signalled LINK_UP (downstream)");
    }

    /* Wire the border-router role now that both interfaces exist (slip was
     * auto-inited before main; mstp was just created above). */
    _setup_6lbr(&_netif);

#if defined(BR_UPLINK_CDC_ECM)
    puts("6LBR ready — run ./start_ecm.sh on the host (see README); 'ifconfig'/'nib' to inspect");
#else
    puts("6LBR ready — run sliptty start_network.sh on the host (see README); 'ifconfig'/'nib' to inspect");
#endif
    puts("status printing is OFF; type 'status' to toggle the periodic MS/TP report");

    thread_create(_status_stack, sizeof(_status_stack),
                  THREAD_PRIORITY_MAIN + 1, 0,
                  _status_thread, NULL, "mstp_status");

    char line[SHELL_DEFAULT_BUFSIZE];
    shell_run(NULL, line, sizeof(line));
    return 0;
}
