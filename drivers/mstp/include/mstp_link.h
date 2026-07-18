/*
 * Copyright (C) 2026 WPI MQP (6LoBAC) — Kerry Lynn
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file
 * @brief   Portable link glue between the ISR and the two MS/TP FSMs.
 *
 * Hardware-free, host-unit-testable. Holds:
 *
 *  - A single-producer/single-consumer lock-free octet ring. The UART RX ISR is
 *    the sole producer; the FSM thread is the sole consumer. Each slot carries a
 *    received octet **and its status** (bit @ref MSTP_OCTET_ERR set ⇒ a framing
 *    or overrun error was detected by the hardware; the data byte is then junk
 *    per 135-2024 9.5.1.1). This preserves the normative UART Receiver Model
 *    (9.5.1) — DataAvailable and ReceiveError are distinct FSM inputs — across
 *    the ISR→thread hand-off: the ISR captures, the FSM (drained here) reacts.
 *
 *  - @ref mstp_link_pump — the portable heart of the on-target FSM thread:
 *    advance SilenceTimer, drain the ring (dispatching each slot to the correct
 *    Receive-FSM entry point), then run the Manager FSM to quiescence. Running
 *    both FSMs plus silence accounting in one context makes all shared 9.5.2
 *    variable access race-free, with no critical sections.
 *
 *  - @ref mstp_build_ctrl_frame — assembles a header-only (Data Length 0)
 *    control frame (Token / Poll For Manager / Reply To Poll For Manager) per
 *    SendNonEncodedFrame (9.5.5.1) with no data octets.
 *
 * No RIOT headers here on purpose.
 */

#ifndef MSTP_LINK_H
#define MSTP_LINK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "mstp_fsm.h"
#include "mstp_mgr.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   RX ring capacity in octets (must be a power of two).
 *
 * Sized to hold the largest MS/TP frame (Nmax_COBS_length ≈ 1506 + header) with
 * margin, so a scheduling gap in the FSM thread cannot overrun the ring at
 * 115.2 kbit/s. Control-only rings (token passing) never need this much.
 */
#ifndef MSTP_RX_RING_LEN
#define MSTP_RX_RING_LEN    (2048U)
#endif
#if (MSTP_RX_RING_LEN & (MSTP_RX_RING_LEN - 1U)) != 0U
#error "MSTP_RX_RING_LEN must be a power of two"
#endif

/** Ring-slot status bit: set ⇒ ReceiveError (framing/overrun); data byte junk. */
#define MSTP_OCTET_ERR      (0x100U)

/**
 * @brief   SPSC lock-free octet ring (UART ISR produces, FSM thread consumes).
 *
 * Safe without locks given exactly one producer and one consumer and atomic
 * word-aligned index writes (true on Cortex-M).
 */
typedef struct {
    volatile uint16_t buf[MSTP_RX_RING_LEN]; /**< status<<8 | data                 */
    volatile unsigned head;                  /**< producer index (ISR)             */
    volatile unsigned tail;                  /**< consumer index (thread)          */
    volatile uint32_t dropped;               /**< octets lost to a full ring       */
} mstp_ring_t;

/** Reset the ring to empty. */
static inline void mstp_ring_reset(mstp_ring_t *r)
{
    r->head = 0;
    r->tail = 0;
    r->dropped = 0;
}

/**
 * @brief   Producer: enqueue one status/octet slot.
 * @return  true on success, false if the ring was full (slot dropped, counted).
 */
static inline bool mstp_ring_put(mstp_ring_t *r, uint16_t slot)
{
    unsigned h = r->head;
    unsigned n = (h + 1U) & (MSTP_RX_RING_LEN - 1U);
    if (n == r->tail) {
        r->dropped++;
        return false;
    }
    r->buf[h] = slot;
    r->head = n;
    return true;
}

/**
 * @brief   Consumer: dequeue one slot.
 * @return  true and store into @p out, or false if the ring was empty.
 */
static inline bool mstp_ring_get(mstp_ring_t *r, uint16_t *out)
{
    unsigned t = r->tail;
    if (t == r->head) {
        return false;
    }
    *out = r->buf[t];
    r->tail = (t + 1U) & (MSTP_RX_RING_LEN - 1U);
    return true;
}

/**
 * @brief   Assemble a header-only control frame (Data Length 0), 9.5.5.1.
 *
 * Layout: 55 FF | FT DST SRC 00 00 | HeaderCRC. Used for Token, Poll For
 * Manager, Reply To Poll For Manager, and Reply Postponed.
 *
 * @param[in]  ft   frame type
 * @param[in]  dst  destination address
 * @param[in]  src  source address (This Station)
 * @param[out] out  buffer of at least 8 octets
 * @return the frame length in octets (8)
 */
size_t mstp_build_ctrl_frame(uint8_t ft, uint8_t dst, uint8_t src, uint8_t *out);

/**
 * @brief   Advance one FSM-thread iteration (hardware-free).
 *
 * 1. Add @p elapsed_ms to SilenceTimer (the timer process, 9.5.2).
 * 2. Drain the RX ring: each slot with @ref MSTP_OCTET_ERR set drives
 *    @ref mstp_rx_fsm_error, otherwise @ref mstp_rx_fsm_octet. (Octet activity
 *    clears SilenceTimer inside the Receive FSM.)
 * 3. Run @ref mstp_mgr_step until no transition fires.
 *
 * @param[in,out] rx          Receive Frame FSM (holds the shared 9.5.2 vars)
 * @param[in,out] mgr         Manager Node FSM
 * @param[in,out] ring        RX octet ring
 * @param[in]     elapsed_ms  milliseconds since the previous pump
 */
void mstp_link_pump(mstp_rx_fsm_t *rx, mstp_mgr_t *mgr, mstp_ring_t *ring,
                    uint32_t elapsed_ms);

#ifdef __cplusplus
}
#endif

#endif /* MSTP_LINK_H */
