/*
 * Copyright (C) 2026 WPI MQP (6LoBAC) — Kerry Lynn
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * CLEAN-ROOM interface for the BACnet MS/TP Receive Frame state machine,
 * transcribed from ANSI/ASHRAE 135-2024 Clause 9.5 — NOT from Steve Karg's
 * bacnet-stack (GPL) nor the contiki-claude tree. The deployed platform/bdk
 * firmware is used only as a black-box wire oracle.
 *
 * Terminology follows 135-2024 (Multidrop Serial Bus/Token Passing;
 * Manager/Subordinate), which RFC 8163-bis aligns to.
 *
 * BSD-3-Clause per IETF contribution requirements; permissive and compatible
 * with RIOT's LGPL-2.1 for upstreaming.
 */

/**
 * @file
 * @brief   Portable, hardware-free MS/TP Receive Frame FSM core (135-2024 9.5.4).
 *
 * Event model mirrors the normative UART Receiver Model (9.5.1): the receiver is
 * a DataRegister plus two flags, DataAvailable and **ReceiveError**. Both are
 * FSM inputs — 9.5.1.3 requires ReceiveError to be TRUE on framing or overrun
 * errors, and 9.5.1.1 states the DataRegister contents after such an error are
 * *not specified*. Hence three entry points:
 *
 *   - @ref mstp_rx_fsm_octet   — DataAvailable (a valid octet was received)
 *   - @ref mstp_rx_fsm_error   — ReceiveError  (framing/overrun; octet is junk)
 *   - @ref mstp_rx_fsm_silence — SilenceTimer exceeded Tframe_abort (60 bit times)
 *
 * Intended to be driven from the UART RX interrupt context: O(1) per octet, no
 * deep byte buffering, and at most one completed frame emitted per event. See
 * notes/mstp-clause9-notes.md for the cited normative extract.
 *
 * No RIOT headers here on purpose: the core is host-unit-testable.
 */

#ifndef MSTP_FSM_H
#define MSTP_FSM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   InputBuffer size (135-2024 9.5.2).
 *
 * max NPDU / 254 (rounded up) + 5 for the Encoded CRC-32K. For the maximum
 * BACnet NPDU: 1497 + 6 + 5 = 1508. 6LoBAC needs to accept a 1500-octet IPv6
 * datagram (RFC 8163), so the default is sized for that.
 */
#ifndef MSTP_INPUT_BUFFER_SIZE
#define MSTP_INPUT_BUFFER_SIZE  (1508U)
#endif

/** Maximum decoded MSDU handed up. */
#ifndef MSTP_MAX_MSDU
#define MSTP_MAX_MSDU           (1500U)
#endif

/** MS/TP Frame Type for IPv6-over-MS/TP (6LoBAC), allocated to IETF: 34. */
#define MSTP_FRAME_TYPE_IPV6    (34U)

/** MS/TP preamble octets (135-2024 9.3). */
#define MSTP_PREAMBLE_55        (0x55U)
#define MSTP_PREAMBLE_FF        (0xFFU)

/** COBS-encoded Frame Type range (135-2024 9.5.3: Nmin/Nmax_COBS_type). */
#define MSTP_NMIN_COBS_TYPE     (32U)
#define MSTP_NMAX_COBS_TYPE     (127U)

/** Valid Length range for COBS-encoded frames (9.5.3). */
#define MSTP_NMIN_COBS_LENGTH   (5U)
#define MSTP_NMAX_COBS_LENGTH   (2043U)

/**
 * @brief   Receive Frame FSM states — 135-2024 Clause 9.5.4, Figure 9-3.
 *
 * All nine states of the normative machine. RECEIVE_ENCODED_FIELDS and
 * VALIDATE_ENCODED_FIELDS are the COBS/extended-frame path (Addendum *an*).
 */
typedef enum {
    MSTP_RX_IDLE = 0,                 /**< 9.5.4.1 awaiting preamble X'55'      */
    MSTP_RX_PREAMBLE,                 /**< 9.5.4.2 awaiting X'FF'               */
    MSTP_RX_HEADER,                   /**< 9.5.4.3 FT,Dst,Src,Len1,Len2         */
    MSTP_RX_HEADER_CRC,               /**< 9.5.4.4 after CheckHeader (9.5.8)    */
    MSTP_RX_DATA,                     /**< 9.5.4.5 non-encoded data             */
    MSTP_RX_DATA_CRC,                 /**< 9.5.4.6 non-encoded data CRC-16      */
    MSTP_RX_SKIP_DATA,                /**< 9.5.4.7 not for us / too long        */
    MSTP_RX_RECEIVE_ENCODED_FIELDS,   /**< 9.5.4.8 COBS Encoded Data + CRC      */
    MSTP_RX_VALIDATE_ENCODED_FIELDS,  /**< 9.5.4.9 CRC-32K residue check        */
} mstp_rx_state_t;

/**
 * @brief   Outcome of feeding one event.
 *
 * Corresponds to the normative flags: MSTP_RX_FRAME ⇒ ReceivedValidFrame,
 * MSTP_RX_INVALID ⇒ ReceivedInvalidFrame.
 */
typedef enum {
    MSTP_RX_NONE = 0,   /**< event consumed; no completed frame                */
    MSTP_RX_FRAME,      /**< ReceivedValidFrame: `frame` holds a valid frame   */
    MSTP_RX_INVALID,    /**< ReceivedInvalidFrame: error during reception      */
} mstp_rx_result_t;

/**
 * @brief   A received, integrity-checked frame (COBS already removed).
 */
typedef struct {
    uint8_t  frame_type;            /**< 34 == IPv6/6LoBAC                     */
    uint8_t  destination;           /**< 255 == broadcast                      */
    uint8_t  source;                /**< 255 not allowed                       */
    uint16_t length;                /**< decoded MSDU length                   */
    uint8_t  data[MSTP_MAX_MSDU];   /**< decoded MSDU                          */
} mstp_frame_t;

/**
 * @brief   Receiver diagnostics. Never gates frame acceptance.
 */
typedef struct {
    uint32_t frames_ok;       /**< ReceivedValidFrame count                    */
    uint32_t frames_inv;      /**< ReceivedInvalidFrame count (every invalid())*/
    uint32_t header_crc_err;  /**< BadHeader (CheckHeader failed)              */
    uint32_t data_crc_err;    /**< BadCRC (DATA_CRC / VALIDATE_ENCODED_FIELDS) */
    uint32_t cobs_err;        /**< COBS decode errors                          */
    uint32_t frame_abort;     /**< Tframe_abort silence timeouts               */
    uint32_t receive_error;   /**< ReceiveError events (framing/overrun)       */
} mstp_rx_stats_t;

/**
 * @brief   Receive Frame FSM instance. Hardware-free and self-contained.
 */
typedef struct {
    mstp_rx_state_t state;      /**< current state                             */
    uint8_t  this_station;      /**< TS (9.5.2), 0..254                        */

    /* header accumulation (9.5.4.3) */
    uint8_t  header[5];         /**< FrameType,Dst,Src,Len_hi,Len_lo           */
    /* 9.5.2: "Index ... up to the value of DataLength+1". DataLength may reach
     * Nmax_COBS_length (2043), so this MUST be wider than 8 bits. */
    uint16_t index;             /**< Index (9.5.2)                             */
    uint16_t data_length;       /**< DataLength (9.5.2)                        */
    uint8_t  header_crc;        /**< HeaderCRC accumulator, init X'FF'         */

    /* data / encoded-field accumulation */
    uint16_t data_crc;          /**< DataCRC (non-encoded path)                */
    uint32_t crc32k;            /**< CRC32K over the Encoded Data field (9.5.2)*/
    uint16_t input_index;       /**< index into input_buffer                   */
    uint8_t  input_buffer[MSTP_INPUT_BUFFER_SIZE];

    /*
     * Shared link variables (135-2024 9.5.2). The Receive Frame FSM is the
     * producer; the Manager/Subordinate Node FSM is the consumer and clears the
     * flags/counters ("A Boolean flag set to TRUE by the Receive State Machine
     * ... Set to FALSE by the main state machine"). Kept in the struct of the
     * machine that clocks them, so the node FSM reads them through a pointer to
     * this instance (see mstp_mgr.c).
     */
    uint32_t event_count;             /**< EventCount (9.5.2): link-activity events */
    uint32_t silence_timer;           /**< SilenceTimer (9.5.2), in milliseconds    */
    bool     received_valid_frame;    /**< ReceivedValidFrame (9.5.2)               */
    bool     received_invalid_frame;  /**< ReceivedInvalidFrame (9.5.2)             */

    mstp_frame_t    frame;      /**< valid iff last result == MSTP_RX_FRAME    */
    mstp_rx_stats_t stats;      /**< diagnostics                               */
} mstp_rx_fsm_t;

/**
 * @brief   Initialise to IDLE (9.5.4.1).
 * @param[out] fsm           FSM instance
 * @param[in]  this_station  TS, our MS/TP address (0..254)
 */
void mstp_rx_fsm_init(mstp_rx_fsm_t *fsm, uint8_t this_station);

/**
 * @brief   DataAvailable: one octet received without error.
 * @param[in,out] fsm    FSM instance
 * @param[in]     octet  contents of DataRegister
 * @return see @ref mstp_rx_result_t
 */
mstp_rx_result_t mstp_rx_fsm_octet(mstp_rx_fsm_t *fsm, uint8_t octet);

/**
 * @brief   ReceiveError (9.5.1.3): framing or overrun error on an octet.
 *
 * The octet itself is NOT passed: 9.5.1.1 states the DataRegister contents after
 * such an error are not specified. Per 9.5.4 every state transitions to IDLE;
 * in HEADER/DATA this also raises ReceivedInvalidFrame.
 *
 * @param[in,out] fsm  FSM instance
 * @return MSTP_RX_INVALID if a frame was in progress, else MSTP_RX_NONE
 */
mstp_rx_result_t mstp_rx_fsm_error(mstp_rx_fsm_t *fsm);

/**
 * @brief   SilenceTimer exceeded Tframe_abort (9.5.3: 60 bit times, ≤100 ms).
 * @param[in,out] fsm  FSM instance
 * @return MSTP_RX_INVALID if a frame was aborted, else MSTP_RX_NONE
 */
mstp_rx_result_t mstp_rx_fsm_silence(mstp_rx_fsm_t *fsm);

/**
 * @brief   Advance SilenceTimer (9.5.2) by @p ms milliseconds.
 *
 * The normative SilenceTimer "is incremented by a timer process and is cleared
 * by the Receive State Machine when activity is detected and by the SendFrame
 * procedure as each octet is transmitted" (9.5.2). @ref mstp_rx_fsm_octet and
 * @ref mstp_rx_fsm_error clear it on activity; this helper is the timer process.
 * The Manager Node FSM reads @c fsm->silence_timer for its LostToken /
 * ReplyTimeout / Tusage_timeout / Tno_token decisions.
 *
 * @param[in,out] fsm  FSM instance
 * @param[in]     ms   elapsed milliseconds to add
 */
static inline void mstp_rx_fsm_silence_tick(mstp_rx_fsm_t *fsm, uint32_t ms)
{
    fsm->silence_timer += ms;
}

#ifdef __cplusplus
}
#endif

#endif /* MSTP_FSM_H */
