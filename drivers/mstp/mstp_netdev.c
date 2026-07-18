/*
 * Copyright (C) 2026 WPI MQP (6LoBAC) — Kerry Lynn
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * SKELETON netdev driver for BACnet MS/TP (6LoBAC / RFC 8163).
 * The ops below are stubs — see drivers/mstp/PORTING.md for the plan and the
 * verified framing code in examples/networking/lobac_bringup/.
 */

#include <errno.h>
#include <string.h>

#include "mstp.h"
#include "net/netdev.h"
#include "net/netopt.h"
#include "periph/uart.h"
#include "periph/gpio.h"

#define ENABLE_DEBUG 0
#include "debug.h"

/* UART RX interrupt callback: feeds the receive-frame FSM.
 * TODO: push @p data into the RX FSM and, on a complete frame, raise
 * NETDEV_EVENT_ISR so _isr() can process it in thread context. */
static void _uart_rx(void *arg, uint8_t data)
{
    (void)arg;
    (void)data;
}

static int _init(netdev_t *netdev)
{
    mstp_t *dev = (mstp_t *)netdev;

    if (uart_init(dev->params.uart, dev->params.baud, _uart_rx, dev) != UART_OK) {
        return -EIO;
    }
    gpio_init(dev->params.de_pin, GPIO_OUT);
    gpio_clear(dev->params.de_pin);   /* idle = receive */
    return 0;
}

static int _send(netdev_t *netdev, const iolist_t *iolist)
{
    (void)netdev;
    (void)iolist;
    /* TODO: gather iolist into an MSDU, build a Frame Type 34 frame
     * (mstp_build_ipv6_frame), assert DE, uart_write, release DE after TX
     * complete. Return bytes sent. */
    return -ENOTSUP;
}

static int _recv(netdev_t *netdev, void *buf, size_t len, void *info)
{
    (void)netdev;
    (void)buf;
    (void)len;
    (void)info;
    /* TODO: return the COBS-decoded, CRC-32K-checked MSDU from the FSM. */
    return -ENOTSUP;
}

static void _isr(netdev_t *netdev)
{
    (void)netdev;
    /* TODO: drain completed RX frames; call netdev->event_callback(...,
     * NETDEV_EVENT_RX_COMPLETE). */
}

static int _get(netdev_t *netdev, netopt_t opt, void *val, size_t max_len)
{
    (void)netdev;
    (void)opt;
    (void)val;
    (void)max_len;
    /* TODO: NETOPT_ADDRESS (mac_addr), NETOPT_ADDR_LEN, NETOPT_DEVICE_TYPE. */
    return -ENOTSUP;
}

static int _set(netdev_t *netdev, netopt_t opt, const void *val, size_t len)
{
    (void)netdev;
    (void)opt;
    (void)val;
    (void)len;
    return -ENOTSUP;
}

const netdev_driver_t mstp_driver = {
    .send = _send,
    .recv = _recv,
    .init = _init,
    .isr  = _isr,
    .get  = _get,
    .set  = _set,
};

void mstp_setup(mstp_t *dev, const mstp_params_t *params, uint8_t index)
{
    (void)index;
    memset(dev, 0, sizeof(*dev));
    dev->netdev.driver = &mstp_driver;
    dev->params = *params;
    /* TODO: netdev_register(&dev->netdev, NETDEV_MSTP, index); */
}
