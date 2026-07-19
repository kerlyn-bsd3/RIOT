/*
 * Copyright (C) 2026 WPI MQP (6LoBAC) — Kerry Lynn
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @defgroup    drivers_mstp   BACnet MS/TP link layer (6LoBAC / RFC 8163)
 * @ingroup     drivers_netdev
 * @brief       MS/TP-over-RS485 for IPv6 (Frame Type 34)
 *
 * The device descriptor bundles the two clean-room FSM cores (Receive Frame,
 * 9.5.4; Manager Node, 9.5.6), the ISR→thread octet ring, and the RS-485 /
 * timer / thread wiring. @ref mstp_start brings up token passing on the wire;
 * the @ref netdev_driver_t ops (for gnrc/IPv6 data) remain to be fleshed out.
 * See drivers/mstp/THEORY_OF_OPERATION.md and PORTING.md.
 * @{
 *
 * @file
 * @brief   Public interface for the MS/TP driver.
 */

#ifndef MSTP_H
#define MSTP_H

#include <stdbool.h>
#include <stdint.h>

#include "periph/uart.h"
#include "periph/gpio.h"
#include "net/netdev.h"
#include "ztimer.h"
#include "ztimer/periodic.h"
#include "thread.h"

#include "mstp_fsm.h"
#include "mstp_mgr.h"
#include "mstp_link.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Stack size for the MS/TP FSM thread. */
#ifndef MSTP_THREAD_STACKSIZE
#define MSTP_THREAD_STACKSIZE   (THREAD_STACKSIZE_DEFAULT)
#endif

/** @brief Priority of the MS/TP FSM thread (slightly above main). */
#ifndef MSTP_THREAD_PRIORITY
#define MSTP_THREAD_PRIORITY    (THREAD_PRIORITY_MAIN - 1)
#endif

/** @brief SilenceTimer / timeout evaluation tick, in milliseconds. */
#ifndef MSTP_TICK_MS
#define MSTP_TICK_MS            (1U)
#endif

/**
 * @brief   Diagnostic event trace depth (power of two). Set to 0 to compile out.
 *
 * A lock-free single-producer (FSM thread) / single-consumer (a reporting thread)
 * ring of timestamped {RX,TX} events. Only frames addressed to us and our own
 * transmissions are recorded, so at MS/TP control-frame rates it never floods.
 * Lets the app measure real response latency, which the coarse 1 s status line
 * cannot show.
 */
#ifndef MSTP_EVLOG_LEN
#define MSTP_EVLOG_LEN          (128U)
#endif

/** Event kinds for @ref mstp_ev_t. */
enum {
    MSTP_EV_RX    = 1, /**< a valid frame was delivered to us                     */
    MSTP_EV_TX    = 2, /**< we began transmitting a frame                        */
    MSTP_EV_RXINV = 3, /**< an invalid frame was seen (best-effort header)        */
};

/** One diagnostic trace record. */
typedef struct {
    uint32_t t_us;    /**< ztimer_now(ZTIMER_USEC) at the event                   */
    uint8_t  ev;      /**< MSTP_EV_RX / MSTP_EV_TX / MSTP_EV_RXINV                */
    uint8_t  st;      /**< Manager FSM state at the event (mstp_mgr_state_t)      */
    uint8_t  ft;      /**< frame type                                            */
    uint8_t  src;     /**< source address (RX/RXINV: frame src; TX: our TS)       */
    uint8_t  dst;     /**< destination address                                   */
} mstp_ev_t;

/**
 * @brief   Static configuration for one MS/TP interface.
 */
typedef struct {
    uart_t   uart;      /**< UART connected to the RS-485 transceiver           */
    uint32_t baud;      /**< bus baud rate (e.g. 115200 for the BDK ring)       */
    gpio_t   de_pin;    /**< driver-enable (DE) GPIO; HIGH = transmit           */
    uint8_t  mac_addr;  /**< this node's MS/TP address / This Station (0..127)  */
} mstp_params_t;

/**
 * @brief   Device descriptor for an MS/TP interface.
 */
typedef struct {
    netdev_t          netdev;     /**< inherited netdev device                  */
    mstp_params_t     params;     /**< static configuration                     */

    mstp_rx_fsm_t     rx;         /**< Receive Frame FSM + shared 9.5.2 vars     */
    mstp_mgr_t        mgr;        /**< Manager Node FSM (9.5.6)                  */
    mstp_ring_t       ring;       /**< ISR→thread octet ring (status<<8 | data)  */

    ztimer_periodic_t tick;       /**< periodic SilenceTimer / timeout tick      */
    thread_t         *fsm_thread; /**< FSM thread (target of thread_flags_set)   */
    volatile bool     txing;      /**< true while driving the bus: RX ISR drops
                                       our own half-duplex echo (9.5.4)          */
    volatile uint32_t txing_drop; /**< octets discarded by the txing guard: the
                                       ONLY silent inbound-drop path. A peer's
                                       reply arriving while this is set is lost
                                       with no error/CRC counter — watch it.     */
    uint32_t          last_ms;    /**< ztimer_now at the previous pump           */
    uint32_t          last_frames_inv; /**< to detect newly seen invalid frames  */

    /* diagnostic event trace (see MSTP_EVLOG_LEN) */
    mstp_ev_t         evlog[MSTP_EVLOG_LEN];
    volatile uint16_t ev_head;    /**< producer index (FSM thread)               */
    uint16_t          ev_tail;    /**< consumer index (reporting/main thread)    */
    uint32_t          last_frames_ok; /**< to detect newly delivered RX frames    */

    char              fsm_stack[MSTP_THREAD_STACKSIZE];
} mstp_t;

/** @brief The MS/TP netdev driver ops (defined in mstp_netdev.c). */
extern const netdev_driver_t mstp_driver;

/**
 * @brief   Set up an MS/TP device descriptor prior to init.
 *
 * @param[out] dev      device descriptor to initialise
 * @param[in]  params   static configuration
 * @param[in]  index    device index (for netdev_register)
 */
void mstp_setup(mstp_t *dev, const mstp_params_t *params, uint8_t index);

/**
 * @brief   Start MS/TP token-ring participation.
 *
 * Initialises both FSMs, spawns the FSM thread, configures the UART (RX ISR →
 * ring), the DE GPIO, and the periodic tick, then the node begins to join the
 * ring (INITIALIZE → IDLE → Poll For Manager / token pass). Higher-layer data
 * exchange is not started here.
 *
 * @param[in,out] dev  device descriptor previously passed to @ref mstp_setup
 * @return 0 on success, negative on error
 */
int mstp_start(mstp_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* MSTP_H */
/** @} */
