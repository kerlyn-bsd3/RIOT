/*
 * Copyright (C) 2026 WPI MQP (6LoBAC) — Kerry Lynn
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @defgroup    drivers_mstp   BACnet MS/TP link layer (6LoBAC / RFC 8163)
 * @ingroup     drivers_netdev
 * @brief       MS/TP-over-RS485 netdev for IPv6 (Frame Type 34)
 *
 * SKELETON — structure only. The netdev ops in mstp_netdev.c are stubs; the
 * MS/TP master-node FSM, COBS/CRC framing, and gnrc_netif glue are ported from
 * the Contiki reference. See drivers/mstp/PORTING.md.
 * @{
 *
 * @file
 * @brief   Public interface for the MS/TP netdev driver.
 */

#ifndef MSTP_H
#define MSTP_H

#include <stdint.h>

#include "periph/uart.h"
#include "periph/gpio.h"
#include "net/netdev.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Static configuration for one MS/TP interface.
 */
typedef struct {
    uart_t   uart;      /**< UART connected to the RS-485 transceiver (USART6) */
    uint32_t baud;      /**< bus baud rate (76800 for the BDK ring, or 115200) */
    gpio_t   de_pin;    /**< driver-enable (DE) GPIO; HIGH = transmit          */
    uint8_t  mac_addr;  /**< this node's MS/TP address (0..127 master range)   */
} mstp_params_t;

/**
 * @brief   Device descriptor for an MS/TP interface.
 */
typedef struct {
    netdev_t      netdev;   /**< inherited netdev device                       */
    mstp_params_t params;   /**< static configuration                          */
    /* TODO: RX/TX frame buffers, receive-frame FSM state, token/timer state,
     * this-station (TS) and next-station (NS) — port from contiki mstp-mac.c. */
} mstp_t;

/** @brief The MS/TP netdev driver ops (defined in mstp_netdev.c). */
extern const netdev_driver_t mstp_driver;

/**
 * @brief   Set up an MS/TP device descriptor prior to netdev init.
 *
 * @param[out] dev      device descriptor to initialise
 * @param[in]  params   static configuration
 * @param[in]  index    device index (for netdev_register)
 */
void mstp_setup(mstp_t *dev, const mstp_params_t *params, uint8_t index);

#ifdef __cplusplus
}
#endif

#endif /* MSTP_H */
/** @} */
