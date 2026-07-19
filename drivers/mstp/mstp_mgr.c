/*
 * Copyright (C) 2026 WPI MQP (6LoBAC) — Kerry Lynn
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * CLEAN-ROOM MS/TP Manager Node state machine, transcribed from ANSI/ASHRAE
 * 135-2024 Clause 9.5.6, with the SendFrame procedure of Clause 9.5.5. Every
 * transition below corresponds 1:1 to a named transition in the standard; the
 * clause and transition name are cited. Conditions and actions are copied from
 * the normative text, not inferred.
 *
 * NOT derived from bacnet-stack (GPL) or the contiki-claude tree.
 */

#include "mstp_mgr.h"

/* ------------------------------------------------------------------------- *
 * Frame-type classification (135-2024 9.3 + the transition condition lists)
 *
 * "known to this node" (9.5.6.2 (c)) is the set of standard MS/TP Frame Types
 * this manager recognizes: the token-management types, the BACnet data types,
 * and Frame Type 34 (IPv6/6LoBAC). Reserved (8..31, 35..127) and unknown
 * proprietary types are NOT known.
 * ------------------------------------------------------------------------- */

static bool ft_known(uint8_t ft)
{
    switch (ft) {
        case MSTP_FT_TOKEN:
        case MSTP_FT_POLL_FOR_MANAGER:
        case MSTP_FT_REPLY_TO_POLL_FOR_MANAGER:
        case MSTP_FT_TEST_REQUEST:
        case MSTP_FT_TEST_RESPONSE:
        case MSTP_FT_DATA_EXPECTING_REPLY:
        case MSTP_FT_DATA_NOT_EXPECTING_REPLY:
        case MSTP_FT_REPLY_POSTPONED:
        case MSTP_FT_EXT_DATA_EXPECTING_REPLY:
        case MSTP_FT_EXT_DATA_NOT_EXPECTING_REPLY:
        case MSTP_FRAME_TYPE_IPV6:
            return true;
        default:
            return false;
    }
}

/** Data frame types that expect a reply (Test_Request, DER, Extended DER). */
static bool ft_expects_reply(uint8_t ft)
{
    return (ft == MSTP_FT_TEST_REQUEST) ||
           (ft == MSTP_FT_DATA_EXPECTING_REPLY) ||
           (ft == MSTP_FT_EXT_DATA_EXPECTING_REPLY);
}

/** BACnet (Extended) Data Expecting Reply — the only DER types that may be broadcast. */
static bool ft_bacnet_der(uint8_t ft)
{
    return (ft == MSTP_FT_DATA_EXPECTING_REPLY) ||
           (ft == MSTP_FT_EXT_DATA_EXPECTING_REPLY);
}

/**
 * Data frame types that do NOT expect a reply, including the reply data types
 * (Test_Response, (Ext) Data Not Expecting Reply) and Frame Type 34 (IPv6, a
 * known type this node processes that does not expect a reply).
 */
static bool ft_no_reply_data(uint8_t ft)
{
    return (ft == MSTP_FT_TEST_RESPONSE) ||
           (ft == MSTP_FT_DATA_NOT_EXPECTING_REPLY) ||
           (ft == MSTP_FT_EXT_DATA_NOT_EXPECTING_REPLY) ||
           (ft == MSTP_FRAME_TYPE_IPV6);
}

/**
 * Frame types that "indicate a reply" (9.5.6.4 ReceivedReply): Test_Response and
 * the (Extended) Data Not Expecting Reply types (the reply carriers). Reply
 * Postponed is handled by its own transition (ReceivedPostpone).
 */
static bool ft_indicates_reply(uint8_t ft)
{
    return (ft == MSTP_FT_TEST_RESPONSE) ||
           (ft == MSTP_FT_DATA_NOT_EXPECTING_REPLY) ||
           (ft == MSTP_FT_EXT_DATA_NOT_EXPECTING_REPLY);
}

/* ------------------------------------------------------------------------- *
 * Small helpers
 * ------------------------------------------------------------------------- */

/** modulo (Nmax_manager + 1), the address wrap used throughout 9.5.6. */
static uint8_t next_addr(const mstp_mgr_t *node, unsigned a)
{
    unsigned m = (unsigned)node->nmax_manager + 1U;
    return (uint8_t)(a % m);
}

/** SendFrame (9.5.5) via the port; clears SilenceTimer per 9.5.5 ("as each octet
 *  is transmitted, set SilenceTimer to zero" — 0 after the final octet). */
static void send_frame(mstp_mgr_t *node, uint8_t ft, uint8_t dst,
                       const uint8_t *data, uint16_t len)
{
    node->port->send_frame(node->port_ctx, ft, dst, node->ts, data, len);
    node->rx->silence_timer = 0;
}

/* shared-variable accessors (9.5.2), held in the Receive FSM instance. */
static bool     rvf(const mstp_mgr_t *n) { return n->rx->received_valid_frame; }
static bool     rif(const mstp_mgr_t *n) { return n->rx->received_invalid_frame; }
static uint32_t silence(const mstp_mgr_t *n) { return n->rx->silence_timer; }
static uint32_t events(const mstp_mgr_t *n) { return n->rx->event_count; }
static uint8_t  rx_dst(const mstp_mgr_t *n) { return n->rx->frame.destination; }
static uint8_t  rx_src(const mstp_mgr_t *n) { return n->rx->frame.source; }
static uint8_t  rx_ft(const mstp_mgr_t *n)  { return n->rx->frame.frame_type; }

static void clear_rvf(mstp_mgr_t *n) { n->rx->received_valid_frame = false; }
static void clear_rif(mstp_mgr_t *n) { n->rx->received_invalid_frame = false; }
static void clear_events(mstp_mgr_t *n) { n->rx->event_count = 0; }

/* ------------------------------------------------------------------------- *
 * Public API
 * ------------------------------------------------------------------------- */

void mstp_mgr_init(mstp_mgr_t *node, mstp_rx_fsm_t *rx, uint8_t ts,
                      const mstp_mgr_port_t *port, void *port_ctx)
{
    node->state            = MSTP_MGR_INITIALIZE;
    node->ts               = ts;
    node->ns               = ts;
    node->ps               = ts;
    node->token_count      = 0;
    node->frame_count      = 0;
    node->retry_count      = 0;
    node->sole_manager     = false;
    node->nmax_manager     = MSTP_DEFAULT_NMAX_MANAGER;
    node->nmax_info_frames = MSTP_DEFAULT_NMAX_INFO_FRAMES;
    node->req_src          = 0;
    node->req_frame_type   = 0;
    node->rx               = rx;
    node->port             = port;
    node->port_ctx         = port_ctx;

    for (size_t i = 0; i < sizeof(node->ctr) / sizeof(uint32_t); i++) {
        ((uint32_t *)&node->ctr)[i] = 0;
    }
}

void mstp_mgr_set_limits(mstp_mgr_t *node, uint8_t nmax_manager,
                            uint8_t nmax_info_frames)
{
    node->nmax_manager     = nmax_manager;
    node->nmax_info_frames = nmax_info_frames;
}

/* ------------------------------------------------------------------------- *
 * 9.5.6.1 INITIALIZE
 * ------------------------------------------------------------------------- */
static bool step_initialize(mstp_mgr_t *node)
{
    /* DoneInitializing — unconditionally. */
    node->ns           = node->ts;   /* NS = TS (next station unknown)          */
    node->ps           = node->ts;   /* PS = TS                                 */
    node->token_count  = MSTP_NPOLL; /* force a Poll For Manager on first token */
    node->sole_manager = false;
    clear_rvf(node);
    clear_rif(node);
    node->state        = MSTP_MGR_IDLE;
    return true;
}

/* ------------------------------------------------------------------------- *
 * 9.5.6.2 IDLE
 * ------------------------------------------------------------------------- */
static bool step_idle(mstp_mgr_t *node)
{
    /* LostToken */
    if (silence(node) >= MSTP_TNO_TOKEN_MS) {
        clear_events(node);                 /* set EventCount to zero */
        node->ctr.lost_token++;
        node->state = MSTP_MGR_NO_TOKEN;
        return true;
    }
    /* ReceivedInvalidFrame */
    if (rif(node)) {
        clear_rif(node);
        node->state = MSTP_MGR_IDLE;
        return true;
    }
    if (rvf(node)) {
        uint8_t dst = rx_dst(node);
        uint8_t src = rx_src(node);
        uint8_t ft  = rx_ft(node);
        bool for_us = (dst == node->ts);
        bool bcast  = (dst == MSTP_BROADCAST_ADDRESS);

        /* ReceivedUnwantedFrame */
        if ((!for_us && !bcast)                                             /* (a) */
            || (bcast && (ft == MSTP_FT_TOKEN || ft == MSTP_FT_TEST_REQUEST)) /* (b) */
            || (!ft_known(ft))                                             /* (c) */
            || (for_us && (ft == MSTP_FT_REPLY_TO_POLL_FOR_MANAGER
                           || ft == MSTP_FT_REPLY_POSTPONED))) {           /* (d) */
            clear_rvf(node);
            node->state = MSTP_MGR_IDLE;
            return true;
        }
        /* ReceivedToken */
        if (for_us && ft == MSTP_FT_TOKEN) {
            clear_rvf(node);
            node->frame_count  = 0;
            node->sole_manager = false;
            node->ctr.received_token++;
            node->state = MSTP_MGR_USE_TOKEN;
            return true;
        }
        /* ReceivedPFM */
        if (for_us && ft == MSTP_FT_POLL_FOR_MANAGER) {
            send_frame(node, MSTP_FT_REPLY_TO_POLL_FOR_MANAGER, src, NULL, 0);
            clear_rvf(node);
            node->ctr.received_pfm++;
            node->state = MSTP_MGR_IDLE;
            return true;
        }
        /* ReceivedDataNoReply */
        if ((for_us || bcast) && ft_no_reply_data(ft)) {
            if (node->port->indicate) {
                node->port->indicate(node->port_ctx, ft, src,
                                     node->rx->frame.data, node->rx->frame.length);
            }
            clear_rvf(node);
            node->state = MSTP_MGR_IDLE;
            return true;
        }
        /* ReceivedDataNeedingReply */
        if (for_us && ft_expects_reply(ft)) {
            if (node->port->indicate) {
                node->port->indicate(node->port_ctx, ft, src,
                                     node->rx->frame.data, node->rx->frame.length);
            }
            node->req_src        = src;
            node->req_frame_type = ft;
            clear_rvf(node);
            node->state = MSTP_MGR_ANSWER_DATA_REQUEST;
            return true;
        }
        /* BroadcastDataNeedingReply */
        if (bcast && ft_bacnet_der(ft)) {
            if (node->port->indicate) {
                node->port->indicate(node->port_ctx, ft, src,
                                     node->rx->frame.data, node->rx->frame.length);
            }
            clear_rvf(node);
            node->state = MSTP_MGR_IDLE;
            return true;
        }
        /*
         * Defensive completion: a valid frame that matches none of the eight
         * enumerated conditions (e.g. a broadcast management frame the standard
         * neither expects nor forbids in IDLE) is discarded so the flag cannot
         * latch. This preserves progress without inventing a data action.
         */
        clear_rvf(node);
        node->state = MSTP_MGR_IDLE;
        return true;
    }
    return false;
}

/* ------------------------------------------------------------------------- *
 * 9.5.6.3 USE_TOKEN
 * ------------------------------------------------------------------------- */
static bool step_use_token(mstp_mgr_t *node)
{
    mstp_tx_frame_t tx = { 0 };
    bool have = (node->port->next_tx != NULL) &&
                node->port->next_tx(node->port_ctx, &tx);

    /* NothingToSend */
    if (!have) {
        node->frame_count = node->nmax_info_frames;
        node->state = MSTP_MGR_DONE_WITH_TOKEN;
        return true;
    }

    bool bcast = (tx.dst == MSTP_BROADCAST_ADDRESS);

    /* SendAndWait: Test_Request, a known type that expects a reply, or a unicast
     * BACnet (Extended) Data Expecting Reply. A broadcast (Ext) DER is excluded
     * here and handled by SendNoWait below. */
    if (ft_expects_reply(tx.frame_type) && !(bcast && ft_bacnet_der(tx.frame_type))) {
        send_frame(node, tx.frame_type, tx.dst, tx.data, tx.len);
        node->frame_count++;
        node->state = MSTP_MGR_WAIT_FOR_REPLY;
        return true;
    }

    /* SendNoWait: Test_Response, (Extended) Data Not Expecting Reply, a known
     * type that does not expect a reply, or a broadcast (Extended) Data
     * Expecting Reply. Anything else queued by the higher layer is sent without
     * waiting (safe default). */
    send_frame(node, tx.frame_type, tx.dst, tx.data, tx.len);
    node->frame_count++;
    node->state = MSTP_MGR_DONE_WITH_TOKEN;
    return true;
}

/* ------------------------------------------------------------------------- *
 * 9.5.6.4 WAIT_FOR_REPLY
 * ------------------------------------------------------------------------- */
static bool step_wait_for_reply(mstp_mgr_t *node)
{
    /* ReplyTimeout */
    if (silence(node) >= MSTP_TREPLY_TIMEOUT_MS) {
        node->frame_count = node->nmax_info_frames;
        node->state = MSTP_MGR_DONE_WITH_TOKEN;
        return true;
    }
    /* InvalidFrame */
    if (rif(node)) {
        clear_rif(node);
        node->state = MSTP_MGR_DONE_WITH_TOKEN;
        return true;
    }
    if (rvf(node)) {
        uint8_t dst = rx_dst(node);
        uint8_t ft  = rx_ft(node);
        /* ReceivedReply */
        if (dst == node->ts && ft_indicates_reply(ft)) {
            if (node->port->indicate) {
                node->port->indicate(node->port_ctx, ft, rx_src(node),
                                     node->rx->frame.data, node->rx->frame.length);
            }
            clear_rvf(node);
            node->state = MSTP_MGR_DONE_WITH_TOKEN;
            return true;
        }
        /* ReceivedPostpone */
        if (dst == node->ts && ft == MSTP_FT_REPLY_POSTPONED) {
            clear_rvf(node);
            node->state = MSTP_MGR_DONE_WITH_TOKEN;
            return true;
        }
        /* ReceivedUnexpectedFrame — drops the token. */
        if ((dst != node->ts) || (!ft_indicates_reply(ft)
                                  && ft != MSTP_FT_REPLY_POSTPONED)) {
            clear_rvf(node);
            node->state = MSTP_MGR_IDLE;
            return true;
        }
    }
    return false;
}

/* ------------------------------------------------------------------------- *
 * 9.5.6.5 DONE_WITH_TOKEN
 * ------------------------------------------------------------------------- */
static bool step_done_with_token(mstp_mgr_t *node)
{
    uint8_t  ts  = node->ts;
    uint16_t fc  = node->frame_count;
    uint16_t tc  = node->token_count;
    uint8_t  ns  = node->ns;
    bool     sm  = node->sole_manager;
    uint8_t  next_ts = next_addr(node, (unsigned)ts + 1U);         /* (TS+1) mod (Nmax_manager+1) */
    uint8_t  next_ps = next_addr(node, (unsigned)node->ps + 1U);   /* (PS+1) mod (Nmax_manager+1) */

    /* SendAnotherFrame */
    if (fc < node->nmax_info_frames) {
        node->state = MSTP_MGR_USE_TOKEN;
        return true;
    }
    /* NextStationUnknown */
    if (fc >= node->nmax_info_frames && !sm && ns == ts) {
        node->ps = next_ts;
        send_frame(node, MSTP_FT_POLL_FOR_MANAGER, node->ps, NULL, 0);
        node->retry_count = 0;
        node->state = MSTP_MGR_POLL_FOR_MANAGER;
        return true;
    }
    /* SoleManager */
    if (fc >= node->nmax_info_frames && tc < MSTP_NPOLL && sm) {
        node->frame_count = 0;
        node->token_count++;
        node->state = MSTP_MGR_USE_TOKEN;
        return true;
    }
    /* SendToken */
    if ((fc >= node->nmax_info_frames && tc < MSTP_NPOLL && !sm)
        || (ns == next_ts)) {
        node->token_count++;
        send_frame(node, MSTP_FT_TOKEN, ns, NULL, 0);
        node->retry_count = 0;
        clear_events(node);
        node->ctr.send_token++;
        node->state = MSTP_MGR_PASS_TOKEN;
        return true;
    }
    /* SendMaintenancePFM */
    if (fc >= node->nmax_info_frames && tc >= MSTP_NPOLL && next_ps != ns) {
        node->ps = next_ps;
        send_frame(node, MSTP_FT_POLL_FOR_MANAGER, node->ps, NULL, 0);
        node->retry_count = 0;
        node->state = MSTP_MGR_POLL_FOR_MANAGER;
        return true;
    }
    /* ResetMaintenancePFM */
    if (fc >= node->nmax_info_frames && tc >= MSTP_NPOLL && next_ps == ns && !sm) {
        node->ps = ts;
        send_frame(node, MSTP_FT_TOKEN, ns, NULL, 0);
        node->retry_count = 0;
        clear_events(node);
        node->token_count = 1;
        node->state = MSTP_MGR_PASS_TOKEN;
        return true;
    }
    /* SoleManagerRestartMaintenancePFM */
    if (fc >= node->nmax_info_frames && tc >= MSTP_NPOLL && next_ps == ns && sm) {
        node->ps = next_addr(node, (unsigned)ns + 1U);
        send_frame(node, MSTP_FT_POLL_FOR_MANAGER, node->ps, NULL, 0);
        node->ns = ts;
        node->retry_count = 0;
        node->token_count = 1;
        node->state = MSTP_MGR_POLL_FOR_MANAGER;
        return true;
    }
    return false;
}

/* ------------------------------------------------------------------------- *
 * 9.5.6.6 PASS_TOKEN
 * ------------------------------------------------------------------------- */
static bool step_pass_token(mstp_mgr_t *node)
{
    /* SawTokenUser */
    if (silence(node) < MSTP_TUSAGE_TIMEOUT_MS && events(node) > MSTP_NMIN_OCTETS) {
        node->ctr.saw_token_user++;
        node->state = MSTP_MGR_IDLE;
        return true;
    }
    /* RetrySendToken */
    if (silence(node) >= MSTP_TUSAGE_TIMEOUT_MS && node->retry_count < MSTP_NRETRY_TOKEN) {
        node->retry_count++;
        send_frame(node, MSTP_FT_TOKEN, node->ns, NULL, 0);
        clear_events(node);
        node->ctr.retry_send_token++;
        node->state = MSTP_MGR_PASS_TOKEN;   /* re-enter */
        return true;
    }
    /* FindNewSuccessorUnknown */
    if (silence(node) >= MSTP_TUSAGE_TIMEOUT_MS && node->retry_count >= MSTP_NRETRY_TOKEN
        && next_addr(node, (unsigned)node->ns + 1U) == node->ts) {
        node->ps = next_addr(node, (unsigned)node->ts + 1U);
        send_frame(node, MSTP_FT_POLL_FOR_MANAGER, node->ps, NULL, 0);
        node->ns = node->ts;
        node->retry_count = 0;
        node->token_count = 0;
        node->ctr.find_new_successor++;
        node->state = MSTP_MGR_POLL_FOR_MANAGER;
        return true;
    }
    /* FindNewSuccessor */
    if (silence(node) >= MSTP_TUSAGE_TIMEOUT_MS && node->retry_count >= MSTP_NRETRY_TOKEN) {
        node->ps = next_addr(node, (unsigned)node->ns + 1U);
        send_frame(node, MSTP_FT_POLL_FOR_MANAGER, node->ps, NULL, 0);
        node->ns = node->ts;
        node->retry_count = 0;
        node->token_count = 0;
        node->ctr.find_new_successor++;
        node->state = MSTP_MGR_POLL_FOR_MANAGER;
        return true;
    }
    return false;
}

/* ------------------------------------------------------------------------- *
 * 9.5.6.7 NO_TOKEN
 * ------------------------------------------------------------------------- */
static bool step_no_token(mstp_mgr_t *node)
{
    uint32_t lower = MSTP_TNO_TOKEN_MS + (uint32_t)MSTP_TSLOT_MS * node->ts;
    uint32_t upper = MSTP_TNO_TOKEN_MS + (uint32_t)MSTP_TSLOT_MS * ((uint32_t)node->ts + 1U);

    /* SawFrame */
    if (silence(node) < lower && events(node) > MSTP_NMIN_OCTETS) {
        node->state = MSTP_MGR_IDLE;
        return true;
    }
    /* GenerateToken */
    if (silence(node) >= lower && silence(node) < upper) {
        node->ps = next_addr(node, (unsigned)node->ts + 1U);
        send_frame(node, MSTP_FT_POLL_FOR_MANAGER, node->ps, NULL, 0);
        node->ns = node->ts;
        node->retry_count = 0;
        node->token_count = 0;
        node->ctr.generate_token++;
        node->state = MSTP_MGR_POLL_FOR_MANAGER;
        return true;
    }
    return false;
}

/* ------------------------------------------------------------------------- *
 * 9.5.6.8 POLL_FOR_MANAGER
 * ------------------------------------------------------------------------- */
static bool step_poll_for_manager(mstp_mgr_t *node)
{
    if (rvf(node)) {
        uint8_t dst = rx_dst(node);
        uint8_t src = rx_src(node);
        uint8_t ft  = rx_ft(node);
        /* ReceivedReplyToPFM */
        if (dst == node->ts && ft == MSTP_FT_REPLY_TO_POLL_FOR_MANAGER) {
            node->sole_manager = false;
            node->ns = src;
            clear_events(node);
            send_frame(node, MSTP_FT_TOKEN, node->ns, NULL, 0);
            node->ps = node->ts;
            node->token_count = 0;
            node->retry_count = 0;
            clear_rvf(node);
            node->ctr.received_reply_to_pfm++;
            node->state = MSTP_MGR_PASS_TOKEN;
            return true;
        }
        /* ReceivedUnexpectedFrame — drops the token. */
        if ((dst != node->ts) || (ft != MSTP_FT_REPLY_TO_POLL_FOR_MANAGER)) {
            clear_rvf(node);
            node->state = MSTP_MGR_IDLE;
            return true;
        }
    }
    /* SoleManager */
    if (node->sole_manager && (silence(node) >= MSTP_TUSAGE_TIMEOUT_MS || rif(node))) {
        node->frame_count = 0;
        clear_rif(node);
        node->state = MSTP_MGR_USE_TOKEN;
        return true;
    }
    /* DoneWithPFM */
    if (!node->sole_manager && node->ns != node->ts
        && (silence(node) >= MSTP_TUSAGE_TIMEOUT_MS || rif(node))) {
        clear_events(node);
        send_frame(node, MSTP_FT_TOKEN, node->ns, NULL, 0);
        node->retry_count = 0;
        clear_rif(node);
        node->ctr.done_with_pfm++;
        node->state = MSTP_MGR_PASS_TOKEN;
        return true;
    }
    /* SendNextPFM */
    if (!node->sole_manager && node->ns == node->ts
        && next_addr(node, (unsigned)node->ps + 1U) != node->ts
        && (silence(node) >= MSTP_TUSAGE_TIMEOUT_MS || rif(node))) {
        node->ps = next_addr(node, (unsigned)node->ps + 1U);
        send_frame(node, MSTP_FT_POLL_FOR_MANAGER, node->ps, NULL, 0);
        node->retry_count = 0;
        clear_rif(node);
        node->state = MSTP_MGR_POLL_FOR_MANAGER;   /* re-enter */
        return true;
    }
    /* DeclareSoleManager */
    if (!node->sole_manager && node->ns == node->ts
        && next_addr(node, (unsigned)node->ps + 1U) == node->ts
        && (silence(node) >= MSTP_TUSAGE_TIMEOUT_MS || rif(node))) {
        node->sole_manager = true;
        node->frame_count = 0;
        clear_rif(node);
        node->state = MSTP_MGR_USE_TOKEN;
        return true;
    }
    return false;
}

/* ------------------------------------------------------------------------- *
 * 9.5.6.9 ANSWER_DATA_REQUEST
 * ------------------------------------------------------------------------- */
static bool step_answer_data_request(mstp_mgr_t *node)
{
    mstp_tx_frame_t reply = { 0 };
    bool have = (node->port->get_reply != NULL) &&
                node->port->get_reply(node->port_ctx, node->req_frame_type,
                                      node->req_src, &reply);
    /* Reply */
    if (have) {
        send_frame(node, reply.frame_type, reply.dst, reply.data, reply.len);
        node->state = MSTP_MGR_IDLE;
        return true;
    }
    /* DeferredReply */
    send_frame(node, MSTP_FT_REPLY_POSTPONED, node->req_src, NULL, 0);
    node->state = MSTP_MGR_IDLE;
    return true;
}

/* ------------------------------------------------------------------------- *
 * Dispatcher
 * ------------------------------------------------------------------------- */
bool mstp_mgr_step(mstp_mgr_t *node)
{
    switch (node->state) {
        case MSTP_MGR_INITIALIZE:          return step_initialize(node);
        case MSTP_MGR_IDLE:                return step_idle(node);
        case MSTP_MGR_USE_TOKEN:           return step_use_token(node);
        case MSTP_MGR_WAIT_FOR_REPLY:      return step_wait_for_reply(node);
        case MSTP_MGR_DONE_WITH_TOKEN:     return step_done_with_token(node);
        case MSTP_MGR_PASS_TOKEN:          return step_pass_token(node);
        case MSTP_MGR_NO_TOKEN:            return step_no_token(node);
        case MSTP_MGR_POLL_FOR_MANAGER:    return step_poll_for_manager(node);
        case MSTP_MGR_ANSWER_DATA_REQUEST: return step_answer_data_request(node);
        default:                              return false;
    }
}
