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

/* USART6 == UART_DEV(1) on nucleo-f767zi; DE = D2 = PF15; TS = 3. */
#ifndef MSTP_UART_DEV
#define MSTP_UART_DEV       UART_DEV(1)
#endif
#ifndef MSTP_BAUD
#define MSTP_BAUD           (115200U)
#endif
#ifndef MSTP_DE_PIN
#define MSTP_DE_PIN         GPIO_PIN(PORT_F, 15)
#endif
#ifndef MSTP_SRC_ADDR
#define MSTP_SRC_ADDR       (0x03U)
#endif
#ifndef MSTP_NETIF_PRIO
#define MSTP_NETIF_PRIO     (GNRC_NETIF_PRIO)
#endif
#ifndef MSTP_NETIF_STACKSIZE
#define MSTP_NETIF_STACKSIZE (THREAD_STACKSIZE_DEFAULT)
#endif

static mstp_t dev;
static gnrc_netif_t _netif;
static char _netif_stack[MSTP_NETIF_STACKSIZE];

/* One MSDU-gather buffer; the netif thread is the sole caller of _l2_send. */
static uint8_t _msdu[MSTP_MAX_MSDU];

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
    puts("\n6LoBAC: IPv6-over-MS/TP (RFC 8163) on nucleo-f767zi");
    printf("UART=%u baud=%lu DE=PF15 TS=%u\n",
           (unsigned)MSTP_UART_DEV, (unsigned long)MSTP_BAUD, MSTP_SRC_ADDR);

    const mstp_params_t params = {
        .uart     = MSTP_UART_DEV,
        .baud     = MSTP_BAUD,
        .de_pin   = MSTP_DE_PIN,
        .mac_addr = MSTP_SRC_ADDR,
    };
    mstp_setup(&dev, &params, 0);

    int res = gnrc_netif_create(&_netif, _netif_stack, sizeof(_netif_stack),
                                MSTP_NETIF_PRIO, "mstp", &dev.netdev,
                                &_mstp_netif_ops);
    if (res < 0) {
        printf("FATAL: gnrc_netif_create failed (%d)\n", res);
        return 1;
    }
    puts("mstp netif up — use 'ifconfig' to see the address; ping from the 6LBR");

    char line[SHELL_DEFAULT_BUFSIZE];
    shell_run(NULL, line, sizeof(line));
    return 0;
}
