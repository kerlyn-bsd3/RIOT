/*
 * Copyright (C) 2026 WPI MQP (6LoBAC) — Kerry Lynn
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * CLEAN-ROOM interface for the BACnet MS/TP Manager Node state machine,
 * transcribed from ANSI/ASHRAE 135-2024 Clause 9.5.6 (Manager Node Finite State
 * Machine) with the SendFrame procedure of Clause 9.5.5. NOT derived from Steve
 * Karg's bacnet-stack (GPL) nor the contiki-claude tree. Terminology follows
 * 135-2024 (Manager/Subordinate; Poll For Manager), which RFC 8163-bis aligns to.
 *
 * BSD-3-Clause per IETF contribution requirements; permissive and compatible
 * with RIOT's LGPL-2.1 for upstreaming.
 */

/**
 * @file
 * @brief   Portable, hardware-free MS/TP Manager Node FSM core (135-2024 9.5.6).
 *
 * This is the "main state machine" that maintains the token ring. It runs beside
 * the Receive Frame FSM (@ref mstp_fsm.h): the two communicate only through the
 * shared variables of Clause 9.5.2, which live in the @ref mstp_rx_fsm_t instance
 * (EventCount, SilenceTimer, ReceivedValidFrame, ReceivedInvalidFrame, and the
 * last received frame's header). This core owns the token-maintenance variables
 * (NS, PS, TokenCount, FrameCount, RetryCount, SoleManager) and reads the shared
 * ones through a pointer to the Receive FSM.
 *
 * The transmission of a frame — SendFrame (9.5.5), which is inherently the
 * RS-485 procedure (Tturnaround, driver-enable, per-octet SilenceTimer clear,
 * Tpostdrive, driver-disable) plus frame assembly (header CRC, and for COBS
 * frames the Encoded Data / Encoded CRC-32K per 9.10.2) — is delegated to the
 * @ref mstp_mgr_port_t::send_frame callback. That keeps this core hardware-
 * free and host-unit-testable, exactly like the Receive FSM core.
 *
 * Driving model: after every Receive-FSM event (octet / error / silence) and
 * after every SilenceTimer tick, call @ref mstp_mgr_step in a loop until it
 * returns false, then wait for the next event. Each call evaluates the current
 * state's transitions in the normative order and takes at most one.
 *
 * No RIOT headers here on purpose: the core is host-unit-testable.
 */

#ifndef MSTP_MGR_H
#define MSTP_MGR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "mstp_fsm.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------- *
 * Frame Types (135-2024 9.3)
 * ------------------------------------------------------------------------- */
#define MSTP_FT_TOKEN                       (0x00U) /**< Token                    */
#define MSTP_FT_POLL_FOR_MANAGER            (0x01U) /**< Poll For Manager         */
#define MSTP_FT_REPLY_TO_POLL_FOR_MANAGER   (0x02U) /**< Reply To Poll For Manager*/
#define MSTP_FT_TEST_REQUEST                (0x03U) /**< Test_Request             */
#define MSTP_FT_TEST_RESPONSE               (0x04U) /**< Test_Response            */
#define MSTP_FT_DATA_EXPECTING_REPLY        (0x05U) /**< BACnet Data Expecting Reply       */
#define MSTP_FT_DATA_NOT_EXPECTING_REPLY    (0x06U) /**< BACnet Data Not Expecting Reply   */
#define MSTP_FT_REPLY_POSTPONED             (0x07U) /**< Reply Postponed          */
#define MSTP_FT_EXT_DATA_EXPECTING_REPLY    (0x20U) /**< BACnet Extended Data Expecting Reply     */
#define MSTP_FT_EXT_DATA_NOT_EXPECTING_REPLY (0x21U)/**< BACnet Extended Data Not Expecting Reply */
/* Frame Type 34 (X'22') = IPv6/6LoBAC — see MSTP_FRAME_TYPE_IPV6 in mstp_fsm.h. */

/** Broadcast destination address (9.3). */
#define MSTP_BROADCAST_ADDRESS              (255U)

/* ------------------------------------------------------------------------- *
 * Parameters (135-2024 9.5.3)
 * ------------------------------------------------------------------------- */
#define MSTP_NPOLL                  (50U)   /**< tokens before a Poll For Manager cycle */
#define MSTP_NRETRY_TOKEN           (1U)    /**< Token/PFM transmission retries         */
#define MSTP_NMIN_OCTETS            (4U)    /**< events to declare the line active      */

/** Default Nmax_manager (Max_Manager property; ≤127). */
#ifndef MSTP_DEFAULT_NMAX_MANAGER
#define MSTP_DEFAULT_NMAX_MANAGER   (127U)
#endif
/** Default Nmax_info_frames (Max_Info_Frames property; 1..255). */
#ifndef MSTP_DEFAULT_NMAX_INFO_FRAMES
#define MSTP_DEFAULT_NMAX_INFO_FRAMES (1U)
#endif

/* Timers in milliseconds (135-2024 9.5.3). SilenceTimer is compared against
 * these; the Receive FSM keeps SilenceTimer in ms (see mstp_fsm.h). */
#define MSTP_TNO_TOKEN_MS           (500U)  /**< Tno_token       */
#define MSTP_TUSAGE_TIMEOUT_MS      (20U)   /**< Tusage_timeout  */
#define MSTP_TREPLY_TIMEOUT_MS      (255U)  /**< Treply_timeout  */
#define MSTP_TSLOT_MS               (10U)   /**< Tslot           */
#define MSTP_TREPLY_DELAY_MS        (250U)  /**< Treply_delay    */

/**
 * @brief   Manager Node FSM states — 135-2024 Clause 9.5.6, Figure 9-4.
 */
typedef enum {
    MSTP_MGR_INITIALIZE = 0,       /**< 9.5.6.1 power-up / reset            */
    MSTP_MGR_IDLE,                 /**< 9.5.6.2 wait for a frame            */
    MSTP_MGR_USE_TOKEN,            /**< 9.5.6.3 send info frame(s)          */
    MSTP_MGR_WAIT_FOR_REPLY,       /**< 9.5.6.4 await reply to a DER frame  */
    MSTP_MGR_DONE_WITH_TOKEN,      /**< 9.5.6.5 send more / pass / poll     */
    MSTP_MGR_PASS_TOKEN,           /**< 9.5.6.6 listen for successor        */
    MSTP_MGR_NO_TOKEN,             /**< 9.5.6.7 no activity; maybe create   */
    MSTP_MGR_POLL_FOR_MANAGER,     /**< 9.5.6.8 await Reply To PFM          */
    MSTP_MGR_ANSWER_DATA_REQUEST,  /**< 9.5.6.9 reply / defer               */
} mstp_mgr_state_t;

/**
 * @brief   A frame the higher layer wishes to transmit while holding the token.
 *
 * Returned by @ref mstp_mgr_port_t::next_tx and @ref mstp_mgr_port_t::get_reply.
 * The FSM classifies it (SendNoWait vs SendAndWait per 9.5.6.3) by @c frame_type
 * and @c dst, per the normative conditions.
 */
typedef struct {
    uint8_t        frame_type;  /**< MS/TP Frame Type                          */
    uint8_t        dst;         /**< destination address (255 = broadcast)     */
    const uint8_t *data;        /**< MSDU (NULL for header-only frames)         */
    uint16_t       len;         /**< MSDU length (0 for header-only frames)     */
} mstp_tx_frame_t;

/**
 * @brief   Hardware / higher-layer port for the Manager Node FSM.
 *
 * All callbacks receive the opaque @c ctx passed to @ref mstp_mgr_init.
 */
typedef struct {
    /**
     * @brief   SendFrame (135-2024 9.5.5): transmit one complete MS/TP frame.
     *
     * The implementation performs the full 9.5.5 procedure: wait out Tturnaround,
     * enable the line driver, transmit preamble + header (accumulating HeaderCRC)
     * + optional data, clear SilenceTimer as each octet is sent, honour
     * Tpostdrive, then disable the driver. For COBS Frame Types (32..127 with
     * @p len > 0) it assembles the Encoded Data and Encoded CRC-32K fields per
     * 9.10.2 (see mstp_build_ipv6_frame()); for Token / Poll For Manager / Reply
     * To Poll For Manager / Reply Postponed it sends a header-only frame with
     * Data Length 0 (@p data NULL, @p len 0).
     */
    void (*send_frame)(void *ctx, uint8_t frame_type, uint8_t dst, uint8_t src,
                       const uint8_t *data, uint16_t len);

    /**
     * @brief   Indicate a successfully received data frame to the higher layers
     *          (9.5.6.2 ReceivedDataNoReply / ReceivedDataNeedingReply, 9.5.6.4
     *          ReceivedReply). May be NULL if the higher layer is not wired yet.
     */
    void (*indicate)(void *ctx, uint8_t frame_type, uint8_t src,
                     const uint8_t *data, uint16_t len);

    /**
     * @brief   Dequeue the next frame awaiting transmission (9.5.6.3 USE_TOKEN).
     * @return  true and fill @p out if a frame is queued, else false. May be NULL
     *          (treated as "nothing to send" — the token-passing-only case).
     */
    bool (*next_tx)(void *ctx, mstp_tx_frame_t *out);

    /**
     * @brief   Obtain an immediate reply to a received request within Treply_delay
     *          (9.5.6.9 ANSWER_DATA_REQUEST). @return true (fill @p out) if a
     *          reply is available now, false to defer (Reply Postponed). May be
     *          NULL (always defer).
     */
    bool (*get_reply)(void *ctx, uint8_t req_frame_type, uint8_t peer,
                      mstp_tx_frame_t *out);
} mstp_mgr_port_t;

/**
 * @brief   Manager Node FSM instance.
 */
typedef struct {
    mstp_mgr_state_t state;      /**< current state (9.5.6)                 */

    /* Token-maintenance variables owned by this machine (9.5.2). */
    uint8_t  ts;                    /**< This Station                          */
    uint8_t  ns;                    /**< Next Station (== TS if unknown)       */
    uint8_t  ps;                    /**< Poll Station                          */
    uint16_t token_count;           /**< TokenCount                           */
    uint16_t frame_count;           /**< FrameCount                           */
    uint8_t  retry_count;           /**< RetryCount                           */
    bool     sole_manager;          /**< SoleManager                          */

    /* Configuration (9.5.3). */
    uint8_t  nmax_manager;          /**< Nmax_manager (≤127)                   */
    uint8_t  nmax_info_frames;      /**< Nmax_info_frames (1..255)             */

    /* Captured request context for ANSWER_DATA_REQUEST (9.5.6.9). */
    uint8_t  req_src;               /**< source of the request being answered  */
    uint8_t  req_frame_type;        /**< frame type of that request            */

    mstp_rx_fsm_t *rx;              /**< holds the shared 9.5.2 variables       */
    const mstp_mgr_port_t *port; /**< SendFrame + higher-layer callbacks     */
    void *port_ctx;                 /**< opaque context for the callbacks       */
} mstp_mgr_t;

/**
 * @brief   Initialise a Manager Node FSM into INITIALIZE (9.5.6.1).
 *
 * @param[out] node      FSM instance
 * @param[in]  rx        the Receive Frame FSM whose shared variables (9.5.2)
 *                       this node reads; must outlive @p node
 * @param[in]  ts        This Station address (0..127 for a manager)
 * @param[in]  port      port callbacks (must outlive @p node)
 * @param[in]  port_ctx  opaque context handed to each callback
 *
 * Nmax_manager and Nmax_info_frames take their defaults; override with
 * @ref mstp_mgr_set_limits before the first step if needed.
 */
void mstp_mgr_init(mstp_mgr_t *node, mstp_rx_fsm_t *rx, uint8_t ts,
                      const mstp_mgr_port_t *port, void *port_ctx);

/**
 * @brief   Override Nmax_manager / Nmax_info_frames (9.5.3). Optional.
 * @param[in,out] node              FSM instance
 * @param[in]     nmax_manager      Max_Manager (≤127)
 * @param[in]     nmax_info_frames  Max_Info_Frames (1..255)
 */
void mstp_mgr_set_limits(mstp_mgr_t *node, uint8_t nmax_manager,
                            uint8_t nmax_info_frames);

/**
 * @brief   Evaluate the current state once, taking at most one transition.
 *
 * Call in a loop until it returns false after each Receive-FSM event and each
 * SilenceTimer tick.
 *
 * @param[in,out] node  FSM instance
 * @return true if a transition fired (call again), false if none applied
 */
bool mstp_mgr_step(mstp_mgr_t *node);

#ifdef __cplusplus
}
#endif

#endif /* MSTP_MGR_H */
