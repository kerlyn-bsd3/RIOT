/*
 * Copyright (C) 2026 WPI MQP (6LoBAC) — Kerry Lynn
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * RIOT integration for the clean-room MS/TP FSMs: the RS-485 SendFrame
 * procedure (135-2024 9.5.5), the UART RX ISR, the SilenceTimer tick, and the
 * single FSM thread that drives both machines via mstp_link_pump(). The FSM
 * logic itself is hardware-free and lives in mstp_fsm.c / mstp_mgr.c /
 * mstp_link.c; only this file touches periph_uart / periph_gpio / ztimer.
 *
 * Concurrency: the UART ISR is the sole producer into the octet ring and the
 * ztimer tick only raises a thread flag; the FSM thread is the sole consumer
 * and the only context that touches the shared 9.5.2 variables. That makes the
 * Receive-FSM ↔ Manager-FSM shared state race-free without any critical section.
 */

#include "mstp.h"

#include "periph/uart.h"
#include "periph/gpio.h"
#include "ztimer.h"
#include "ztimer/periodic.h"
#include "thread.h"
#include "thread_flags.h"

#define ENABLE_DEBUG 0
#include "debug.h"

/* FSM-thread wake reasons. */
#define MSTP_TF_RX      (1u << 0)   /**< octet(s) enqueued by the UART ISR       */
#define MSTP_TF_TICK    (1u << 1)   /**< SilenceTimer / timeout tick             */

/* ------------------------------------------------------------------------- *
 * SendFrame — 135-2024 9.5.5 (RS-485 half of the procedure)
 * ------------------------------------------------------------------------- */
static void _send_frame(void *ctx, uint8_t ft, uint8_t dst, uint8_t src,
                        const uint8_t *data, uint16_t len)
{
    mstp_t *dev = ctx;
    uint8_t ctrl[8];
    const uint8_t *frame;
    size_t n;

    if (data == NULL || len == 0U) {
        /* Token / Poll For Manager / Reply To PFM / Reply Postponed:
         * header-only, Data Length 0 (SendNonEncodedFrame, 9.5.5.1). */
        n = mstp_build_ctrl_frame(ft, dst, src, ctrl);
        frame = ctrl;
    }
    else {
        /*
         * COBS-encoded data frame (SendCOBS_EncodedFrame, 9.5.5.2) — TODO: build
         * with mstp_build_ipv6_frame() once mstp_frame.* is migrated into the
         * driver (PORTING.md / Session 8 gap #2). Unreachable during token-
         * passing bring-up: the port's next_tx is NULL, so the manager never
         * leaves USE_TOKEN via a data send.
         */
        DEBUG("mstp: data-frame TX not wired yet (ft=%u dst=%u len=%u)\n",
              (unsigned)ft, (unsigned)dst, (unsigned)len);
        return;
    }

    /* 9.5.5(1): if SilenceTimer < Tturnaround (40 bit times), wait the balance.
     * SilenceTimer is in ms; convert to us for the sub-ms turnaround. */
    uint32_t tturn_us   = (40UL * 1000000UL) / dev->params.baud;
    uint32_t silence_us = dev->rx.silence_timer * 1000UL;
    if (silence_us < tturn_us) {
        ztimer_sleep(ZTIMER_USEC, tturn_us - silence_us);
    }

    dev->txing = true;                            /* ignore our own RX echo (9.5.4) */
    gpio_set(dev->params.de_pin);                 /* enable driver (transmit)   */
    uart_write(dev->params.uart, frame, n);

    /*
     * 9.2 Tpostdrive: release DE only after the final stop bit has shifted out.
     * periph_uart exposes no portable TX-complete flag, so wait one frame-time
     * plus a two-character margin (the approach proven on the wire in Session 9).
     * The sanctioned X'FF' padding-octet + UART-TC refinement (notes §2) is a
     * later improvement if the token's last octet is ever seen to clip.
     */
    uint32_t tx_us = (uint32_t)((10ULL * 1000000ULL * ((uint64_t)n + 2U))
                                / dev->params.baud);
    ztimer_sleep(ZTIMER_USEC, tx_us);
    gpio_clear(dev->params.de_pin);               /* release driver (receive)   */
    dev->txing = false;                           /* echo window over; RX real traffic */

    dev->rx.silence_timer = 0;                    /* 9.5.5: cleared per octet TX */
}

static const mstp_mgr_port_t _port = {
    .send_frame = _send_frame,
    .indicate   = NULL,   /* token-passing bring-up: no higher layer wired yet  */
    .next_tx    = NULL,   /* nothing queued -> USE_TOKEN NothingToSend           */
    .get_reply  = NULL,   /* no immediate reply -> ANSWER_DATA_REQUEST defers    */
};

/* ------------------------------------------------------------------------- *
 * UART RX ISR — capture {status, octet} into the ring; wake the FSM thread.
 * ------------------------------------------------------------------------- */
static void _uart_rx(void *arg, uint8_t data)
{
    mstp_t *dev = arg;

    /*
     * Ignore our own transmission (135-2024 9.5.4): on half-duplex RS-485 the
     * transceiver echoes what we drive back onto RX. Dropping it here keeps the
     * echo out of the ring entirely, so it can never strand the Receive FSM
     * mid-frame ahead of a peer's reply. (Reading `data` already cleared RXNE, so
     * no overrun results from discarding it.)
     */
    if (dev->txing) {
        return;
    }

    /*
     * The stock periph_uart callback conveys only the octet, so status is always
     * OK here. When a `periph_uart_rx_error` feature is available (notes §3), an
     * error-aware callback would push (MSTP_OCTET_ERR | data) on a framing/overrun
     * error instead; mstp_link_pump() already dispatches that to
     * mstp_rx_fsm_error(). Until then, corruption is caught by CRC + SilenceTimer.
     */
    mstp_ring_put(&dev->ring, data);
    if (dev->fsm_thread) {
        thread_flags_set(dev->fsm_thread, MSTP_TF_RX);
    }
}

/* ------------------------------------------------------------------------- *
 * SilenceTimer / timeout tick — just raise a flag; work happens in the thread.
 * ------------------------------------------------------------------------- */
static bool _tick(void *arg)
{
    mstp_t *dev = arg;
    if (dev->fsm_thread) {
        thread_flags_set(dev->fsm_thread, MSTP_TF_TICK);
    }
    return ZTIMER_PERIODIC_KEEP_GOING;
}

/* ------------------------------------------------------------------------- *
 * FSM thread — the only consumer of the ring and of the shared 9.5.2 state.
 * ------------------------------------------------------------------------- */
static void *_fsm_thread(void *arg)
{
    mstp_t *dev = arg;

    dev->last_ms = ztimer_now(ZTIMER_MSEC);
    while (1) {
        thread_flags_wait_any(MSTP_TF_RX | MSTP_TF_TICK);
        uint32_t now = ztimer_now(ZTIMER_MSEC);
        uint32_t elapsed = now - dev->last_ms;      /* wrap-safe unsigned delta */
        dev->last_ms = now;
        mstp_link_pump(&dev->rx, &dev->mgr, &dev->ring, elapsed);
    }
    return NULL;
}

/* ------------------------------------------------------------------------- *
 * Public: start token-ring participation.
 * ------------------------------------------------------------------------- */
int mstp_start(mstp_t *dev)
{
    mstp_rx_fsm_init(&dev->rx, dev->params.mac_addr);
    mstp_mgr_init(&dev->mgr, &dev->rx, dev->params.mac_addr, &_port, dev);
    mstp_ring_reset(&dev->ring);
    dev->txing = false;
    dev->fsm_thread = NULL;

    kernel_pid_t pid = thread_create(dev->fsm_stack, sizeof(dev->fsm_stack),
                                     MSTP_THREAD_PRIORITY, 0,
                                     _fsm_thread, dev, "mstp");
    if (pid < 0) {
        return -1;
    }
    /* Set the flag target before any ISR/tick can fire (UART not yet enabled). */
    dev->fsm_thread = thread_get(pid);

    gpio_init(dev->params.de_pin, GPIO_OUT);
    gpio_clear(dev->params.de_pin);               /* idle = receive */

    if (uart_init(dev->params.uart, dev->params.baud, _uart_rx, dev) != UART_OK) {
        return -1;
    }

    ztimer_periodic_init(ZTIMER_MSEC, &dev->tick, _tick, dev, MSTP_TICK_MS);
    ztimer_periodic_start(&dev->tick);
    return 0;
}
