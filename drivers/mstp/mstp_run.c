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

/* Diagnostic event trace: record one {RX,TX,RXINV} event (SPSC; FSM thread produces). */
static void ev_put(mstp_t *dev, uint8_t ev, uint8_t ft, uint8_t src, uint8_t dst,
                   uint16_t aux)
{
#if (MSTP_EVLOG_LEN > 0)
    uint16_t h = dev->ev_head;
    mstp_ev_t *e = &dev->evlog[h & (MSTP_EVLOG_LEN - 1U)];
    e->t_us = ztimer_now(ZTIMER_USEC);
    e->ev   = ev;
    e->st   = (uint8_t)dev->mgr.state;
    e->ft   = ft;
    e->src  = src;
    e->dst  = dst;
    e->aux  = aux;
    dev->ev_head = h + 1U;
#else
    (void)dev; (void)ev; (void)ft; (void)src; (void)dst; (void)aux;
#endif
}

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

    ev_put(dev, MSTP_EV_TX, ft, src, dst, 0);     /* trace: about to transmit   */
    dev->txing = true;                            /* ignore our own RX echo (9.5.4) */
    gpio_set(dev->params.de_pin);                 /* enable driver (transmit)   */
    uart_write(dev->params.uart, frame, n);

    /*
     * 9.2 Tpostdrive: disable the driver only after the final stop bit has been
     * generated — but no later than Tpostdrive. RIOT's blocking uart_write()
     * already spins on the USART transmit-complete (TC) flag before returning
     * (cpu/stm32/periph/uart.c: wait_for_tx_complete()), so the last stop bit is
     * on the wire by the time we reach here; drop DE immediately.
     *
     * This must be prompt: on the DFR0259, /RE is tied to DE, so while DE is
     * asserted the receiver is OFF. A peer answers a Poll For Manager almost
     * immediately (its own Tturnaround, ~40 bit times), so any post-drive delay
     * here keeps us deaf across the reply and the join is lost. (An earlier fixed
     * ~(n+2)-character sleep did exactly that — correct for the Session-9 TX-only
     * emitter, fatal once we must receive the answer.)
     *
     * NB: this relies on uart_write() being synchronous-to-TC — true for the
     * blocking periph_uart path used here, but NOT for periph_uart_nonblocking or
     * the DMA path below the threshold; revisit if either is enabled.
     */
    gpio_clear(dev->params.de_pin);               /* release driver (receive)   */
    dev->txing = false;

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
        dev->txing_drop++;   /* silent inbound drop — the one path with no CRC/err counter */
        return;
    }

    /*
     * Overrun peek (optional; mstp_params_t::ore_sr). RIOT's ISR reads the USART
     * status, calls us on RXNE, and only THEN clears ORE — so at this instant the
     * overrun flag is still live. If it is set, an octet was lost to a receive
     * overrun (RIOT would have silently discarded it). Note it so it surfaces as
     * receive_error and, via the frame-start snapshot, as abort_with_ore. We only
     * read the register (non-destructive); RIOT still owns clearing ORECF.
     */
    if (dev->params.ore_sr && (*dev->params.ore_sr & dev->params.ore_mask)) {
        mstp_rx_fsm_note_overrun(&dev->rx);
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

        /* trace: a valid frame was delivered to us during this pump */
        if (dev->rx.stats.frames_ok != dev->last_frames_ok) {
            dev->last_frames_ok = dev->rx.stats.frames_ok;
            ev_put(dev, MSTP_EV_RX, dev->rx.frame.frame_type,
                   dev->rx.frame.source, dev->rx.frame.destination, 0);
        }
        /* trace: an invalid frame was seen during this pump. rx.frame is NOT
         * updated on an invalid frame, so report the raw header (best-effort:
         * a later frame in the same pump can overwrite header[] before we read
         * it — good enough to see whether the BDK's post-TX frame is landing as
         * garbage vs. never arriving at all). */
        if (dev->rx.stats.frames_inv != dev->last_frames_inv) {
            dev->last_frames_inv = dev->rx.stats.frames_inv;
            ev_put(dev, MSTP_EV_RXINV, dev->rx.header[0],
                   dev->rx.header[2], dev->rx.header[1], dev->rx.abort_index);
        }
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
    dev->txing_drop = 0;
    dev->last_frames_ok = 0;
    dev->last_frames_inv = 0;
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
