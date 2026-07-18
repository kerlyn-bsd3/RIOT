/*
 * Copyright (C) 2026 WPI MQP (6LoBAC) — Kerry Lynn
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Portable ISR<->FSM link glue. Hardware-free; see mstp_link.h. The RS-485 /
 * UART / timer / thread wrapping around this lives in mstp_run.c.
 */

#include "mstp_link.h"
#include "mstp_crc.h"

size_t mstp_build_ctrl_frame(uint8_t ft, uint8_t dst, uint8_t src, uint8_t *out)
{
    /* SendNonEncodedFrame with no data octets (9.5.5.1): preamble, header,
     * ones-complement HeaderCRC — no Data or Data CRC. */
    uint8_t hdr[5] = { ft, dst, src, 0x00U, 0x00U };

    out[0] = MSTP_PREAMBLE_55;
    out[1] = MSTP_PREAMBLE_FF;
    out[2] = ft;
    out[3] = dst;
    out[4] = src;
    out[5] = 0x00U;                       /* Data Length, MSB */
    out[6] = 0x00U;                       /* Data Length, LSB */
    out[7] = mstp_header_crc8(hdr, sizeof(hdr));
    return 8U;
}

void mstp_link_pump(mstp_rx_fsm_t *rx, mstp_mgr_t *mgr, mstp_ring_t *ring,
                    uint32_t elapsed_ms)
{
    if (elapsed_ms) {
        mstp_rx_fsm_silence_tick(rx, elapsed_ms);   /* SilenceTimer += elapsed */
    }

    uint16_t slot;
    while (mstp_ring_get(ring, &slot)) {
        if (slot & MSTP_OCTET_ERR) {
            (void)mstp_rx_fsm_error(rx);            /* ReceiveError (9.5.1.3) */
        }
        else {
            (void)mstp_rx_fsm_octet(rx, (uint8_t)(slot & 0xFFU)); /* DataAvailable */
        }
    }

    /*
     * Tframe_abort recovery (9.5.4/9.5.3). If a frame is in progress and the
     * medium has been silent past Tframe_abort, abandon it and return the
     * receiver to IDLE so it re-syncs on the next preamble. Half-duplex echo of
     * our own transmission, a bus-turnaround glitch, or a dropped octet can
     * otherwise strand the Receive FSM mid-frame, where it would swallow the
     * start of the next real frame and fail its header CRC. Keyed on the
     * inter-frame silence gap, so it works against conforming stations that do
     * not emit a trailing X'FF' pad octet.
     */
    if ((rx->state != MSTP_RX_IDLE) && (rx->silence_timer >= MSTP_TFRAME_ABORT_MS)) {
        (void)mstp_rx_fsm_silence(rx);
    }

    while (mstp_mgr_step(mgr)) { }                   /* run to quiescence */
}
