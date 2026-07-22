/*
 * Copyright (C) 2026 WPI MQP (6LoBAC) — Kerry Lynn
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * netdev driver for BACnet MS/TP (6LoBAC / RFC 8163): a thin shim over the
 * clean-room FSM engine (mstp_run.c / mstp_fsm.c / mstp_mgr.c). It does NOT
 * touch the wire itself — the engine's single FSM thread owns the UART and the
 * token. This file only bridges the engine to gnrc:
 *
 *   RX: the engine's indicate() copies a decoded MSDU into dev->rx_data and calls
 *       netdev_trigger_event_isr(); the netif thread then runs _isr() ->
 *       RX_COMPLETE -> _recv() drains dev->rx_data.
 *   TX: the mstp gnrc_netif ops call mstp_tx_ipv6() directly (they have the L2
 *       destination); the manager transmits it as a Frame Type 34 when this node
 *       holds the token. So netdev _send() is intentionally unused.
 *
 * The device is presented to gnrc as a 1-octet-L2-address, inherently-6LoWPAN
 * interface by reporting NETDEV_TYPE_CC110X (the closest built-in analog to MS/TP:
 * 1-byte address, 6lo). The IID formed from the 1-byte address follows RIOT's
 * CC110X default — confirm/adjust against RFC 8163 6LoBAC.
 */

#include <errno.h>
#include <string.h>

#include "mstp.h"
#include "net/netdev.h"
#include "net/netopt.h"

#ifdef MODULE_GNRC_NETIF
#include "net/gnrc/nettype.h"
#endif

#define ENABLE_DEBUG 0
#include "debug.h"

/* netdev is the first member of mstp_t, so the cast is exact. */
static inline mstp_t *_dev(netdev_t *netdev) { return (mstp_t *)netdev; }

static int _init(netdev_t *netdev)
{
    /* Bring up the engine: UART RX ISR, DE GPIO, SilenceTimer tick, FSM thread. */
    return mstp_start(_dev(netdev));
}

static void _isr(netdev_t *netdev)
{
    /* Runs in the netif thread after the FSM thread raised NETDEV_EVENT_ISR. */
    if (netdev->event_callback) {
        netdev->event_callback(netdev, NETDEV_EVENT_RX_COMPLETE);
    }
}

static int _recv(netdev_t *netdev, void *buf, size_t len, void *info)
{
    mstp_t *dev = _dev(netdev);
    uint16_t n = dev->rx_data_len;

    (void)info;
    if (buf == NULL) {
        if (len > 0U) {
            dev->rx_ready = false;   /* caller asked us to drop the frame */
        }
        return (int)n;               /* else: report bytes available */
    }
    if (len < (size_t)n) {
        return -ENOBUFS;
    }
    memcpy(buf, dev->rx_data, n);
    dev->rx_ready = false;
    return (int)n;
}

static int _send(netdev_t *netdev, const iolist_t *iolist)
{
    /* Unused: the mstp gnrc_netif ops enqueue via mstp_tx_ipv6() directly, since
     * MS/TP needs the L2 destination, not an opaque byte stream. */
    (void)netdev; (void)iolist;
    return -ENOTSUP;
}

static int _get(netdev_t *netdev, netopt_t opt, void *val, size_t max_len)
{
    mstp_t *dev = _dev(netdev);

    switch (opt) {
        case NETOPT_DEVICE_TYPE:
            if (max_len < sizeof(uint16_t)) { return -EOVERFLOW; }
            *((uint16_t *)val) = NETDEV_TYPE_CC110X;  /* 1-byte L2 addr, 6lo */
            return sizeof(uint16_t);
        case NETOPT_ADDR_LEN:
        case NETOPT_SRC_LEN:
            if (max_len < sizeof(uint16_t)) { return -EOVERFLOW; }
            *((uint16_t *)val) = 1U;
            return sizeof(uint16_t);
        case NETOPT_ADDRESS:
            if (max_len < 1U) { return -EOVERFLOW; }
            *((uint8_t *)val) = dev->params.mac_addr;
            return 1;
        case NETOPT_MAX_PDU_SIZE:
            if (max_len < sizeof(uint16_t)) { return -EOVERFLOW; }
            *((uint16_t *)val) = MSTP_MAX_MSDU;
            return sizeof(uint16_t);
#ifdef MODULE_GNRC_NETIF
        case NETOPT_PROTO:
            if (max_len < sizeof(gnrc_nettype_t)) { return -EOVERFLOW; }
            *((gnrc_nettype_t *)val) = GNRC_NETTYPE_SIXLOWPAN;
            return sizeof(gnrc_nettype_t);
#endif
        default:
            return -ENOTSUP;
    }
}

static int _set(netdev_t *netdev, netopt_t opt, const void *val, size_t len)
{
    mstp_t *dev = _dev(netdev);

    switch (opt) {
        case NETOPT_ADDRESS:
            if (len < 1U) { return -EINVAL; }
            dev->params.mac_addr = *((const uint8_t *)val);
            return 1;
        default:
            return -ENOTSUP;
    }
}

const netdev_driver_t mstp_driver = {
    .init = _init,
    .isr  = _isr,
    .recv = _recv,
    .send = _send,
    .get  = _get,
    .set  = _set,
};

void mstp_setup(mstp_t *dev, const mstp_params_t *params, uint8_t index)
{
    (void)index;
    memset(dev, 0, sizeof(*dev));
    dev->netdev.driver = &mstp_driver;
    dev->params = *params;
    /* gnrc_netif_mstp_create() registers the netdev with gnrc; the standalone
     * diagnostic app instead calls mstp_start() directly and never registers. */
}
