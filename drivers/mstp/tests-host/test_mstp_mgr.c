/*
 * Copyright (C) 2026 WPI MQP (6LoBAC) — Kerry Lynn
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Host unit tests for the MS/TP Manager Node FSM (135-2024 9.5.6), driven
 * through the real Receive Frame FSM (9.5.4): received frames are fed octet by
 * octet (preamble + header + header CRC) so EventCount, SilenceTimer and the
 * NoData delivery path are exercised exactly as on the wire.
 *
 * Focus: token-ring participation — INITIALIZE -> IDLE, Poll For Manager,
 * token pass, join. No data exchange.
 *
 * Build (from drivers/mstp/tests-host/):  make -f Makefile.host check
 */

#include <stdio.h>
#include <string.h>

#include "mstp_mgr.h"
#include "mstp_fsm.h"
#include "mstp_crc.h"

/* ---- tiny assert framework -------------------------------------------- */
static int g_fail = 0;
static int g_checks = 0;
#define CHECK(cond, ...) do {                                             \
        g_checks++;                                                       \
        if (!(cond)) { g_fail++;                                          \
            printf("  FAIL %s:%d: ", __func__, __LINE__);                 \
            printf(__VA_ARGS__); printf("\n"); }                         \
    } while (0)

/* ---- mock port: record transmitted frames ----------------------------- */
typedef struct { uint8_t ft, dst, src; uint16_t len; } sent_t;
typedef struct {
    sent_t   sent[32];
    unsigned n;
    bool     have_tx;      /* next_tx returns this when set */
    mstp_tx_frame_t tx;
    bool     have_reply;   /* get_reply returns this when set */
    mstp_tx_frame_t reply;
    unsigned indications;
} mock_t;

static void mock_send(void *ctx, uint8_t ft, uint8_t dst, uint8_t src,
                      const uint8_t *data, uint16_t len)
{
    (void)data;
    mock_t *m = ctx;
    if (m->n < 32) {
        m->sent[m->n].ft = ft; m->sent[m->n].dst = dst;
        m->sent[m->n].src = src; m->sent[m->n].len = len;
        m->n++;
    }
}
static void mock_indicate(void *ctx, uint8_t ft, uint8_t src,
                          const uint8_t *data, uint16_t len)
{
    (void)ft; (void)src; (void)data; (void)len;
    ((mock_t *)ctx)->indications++;
}
static bool mock_next_tx(void *ctx, mstp_tx_frame_t *out)
{
    mock_t *m = ctx;
    if (m->have_tx) { *out = m->tx; m->have_tx = false; return true; }
    return false;
}
static bool mock_get_reply(void *ctx, uint8_t rft, uint8_t peer, mstp_tx_frame_t *out)
{
    (void)rft; (void)peer;
    mock_t *m = ctx;
    if (m->have_reply) { *out = m->reply; return true; }
    return false;
}
static const mstp_mgr_port_t MOCK_PORT = {
    .send_frame = mock_send, .indicate = mock_indicate,
    .next_tx = mock_next_tx, .get_reply = mock_get_reply,
};

/* ---- test fixture ----------------------------------------------------- */
typedef struct {
    mstp_rx_fsm_t  rx;
    mstp_mgr_t  node;
    mock_t         mock;
} fix_t;

static void setup(fix_t *f, uint8_t ts)
{
    memset(f, 0, sizeof(*f));
    mstp_rx_fsm_init(&f->rx, ts);
    mstp_mgr_init(&f->node, &f->rx, ts, &MOCK_PORT, &f->mock);
}

/* pump the manager to quiescence */
static void pump(fix_t *f) { while (mstp_mgr_step(&f->node)) { } }

/* advance SilenceTimer */
static void tick(fix_t *f, uint32_t ms) { mstp_rx_fsm_silence_tick(&f->rx, ms); }

/* Feed a header-only (Data Length 0) frame through the real Receive FSM. */
static void feed_frame(fix_t *f, uint8_t ft, uint8_t dst, uint8_t src)
{
    uint8_t hdr[5] = { ft, dst, src, 0x00, 0x00 };
    uint8_t crc = mstp_header_crc8(hdr, sizeof(hdr));
    uint8_t wire[8] = { 0x55, 0xFF, ft, dst, src, 0x00, 0x00, crc };
    for (unsigned i = 0; i < sizeof(wire); i++) {
        mstp_rx_fsm_octet(&f->rx, wire[i]);
    }
}

static const sent_t *last(fix_t *f)
{
    return (f->mock.n == 0) ? NULL : &f->mock.sent[f->mock.n - 1];
}

/* ---- tests ------------------------------------------------------------ */

static void t_initialize(void)
{
    printf("t_initialize\n");
    fix_t f; setup(&f, 5);
    CHECK(f.node.state == MSTP_MGR_INITIALIZE, "start INITIALIZE");
    pump(&f);
    CHECK(f.node.state == MSTP_MGR_IDLE, "-> IDLE");
    CHECK(f.node.ns == 5 && f.node.ps == 5, "NS=PS=TS");
    CHECK(f.node.token_count == MSTP_NPOLL, "TokenCount=Npoll (force PFM)");
    CHECK(!f.node.sole_manager, "SoleManager FALSE");
    CHECK(f.mock.n == 0, "no frames sent");
}

/* Join as pollee: a Poll For Manager to us is answered with Reply To PFM. */
static void t_reply_to_pfm(void)
{
    printf("t_reply_to_pfm\n");
    fix_t f; setup(&f, 5); pump(&f);              /* -> IDLE */
    feed_frame(&f, MSTP_FT_POLL_FOR_MANAGER, /*dst*/5, /*src*/1);
    pump(&f);
    CHECK(f.node.state == MSTP_MGR_IDLE, "stay IDLE");
    CHECK(f.mock.n == 1, "one frame sent");
    CHECK(last(&f)->ft == MSTP_FT_REPLY_TO_POLL_FOR_MANAGER, "sent Reply To PFM");
    CHECK(last(&f)->dst == 1, "RTPFM to poller (src of PFM)");
    CHECK(!f.rx.received_valid_frame, "ReceivedValidFrame cleared");
}

/* Receive a Token addressed to us, nothing to send, successor unknown ->
 * Poll For Manager to (TS+1). */
static void t_token_next_station_unknown(void)
{
    printf("t_token_next_station_unknown\n");
    fix_t f; setup(&f, 5); pump(&f);
    feed_frame(&f, MSTP_FT_TOKEN, 5, 1);
    pump(&f);
    CHECK(f.node.state == MSTP_MGR_POLL_FOR_MANAGER, "-> POLL_FOR_MANAGER");
    CHECK(f.mock.n == 1 && last(&f)->ft == MSTP_FT_POLL_FOR_MANAGER, "sent PFM");
    CHECK(last(&f)->dst == 6, "PFM to (TS+1)=6");
    CHECK(f.node.ps == 6, "PS=(TS+1)");
    CHECK(f.node.retry_count == 0, "RetryCount=0");
}

/* Broadcast Token is unwanted (9.5.6.2 (b)) and dropped. */
static void t_broadcast_token_unwanted(void)
{
    printf("t_broadcast_token_unwanted\n");
    fix_t f; setup(&f, 5); pump(&f);
    feed_frame(&f, MSTP_FT_TOKEN, MSTP_BROADCAST_ADDRESS, 1);
    CHECK(f.rx.received_valid_frame, "RX delivered broadcast token as valid");
    pump(&f);
    CHECK(f.node.state == MSTP_MGR_IDLE, "stay IDLE");
    CHECK(f.mock.n == 0, "nothing sent");
    CHECK(!f.rx.received_valid_frame, "flag cleared (unwanted)");
}

/* A frame for another station is filtered by the Receive FSM (never seen). */
static void t_frame_not_for_us(void)
{
    printf("t_frame_not_for_us\n");
    fix_t f; setup(&f, 5); pump(&f);
    feed_frame(&f, MSTP_FT_TOKEN, /*dst*/9, /*src*/1);
    CHECK(!f.rx.received_valid_frame, "RX did not deliver (NotForUs)");
    CHECK(f.rx.event_count >= MSTP_NMIN_OCTETS, "but EventCount rose (activity)");
    pump(&f);
    CHECK(f.node.state == MSTP_MGR_IDLE, "stay IDLE");
    CHECK(f.mock.n == 0, "nothing sent");
}

/* No activity for Tno_token: lowest node creates a token and finds a successor. */
static void t_lost_token_generate(void)
{
    printf("t_lost_token_generate\n");
    fix_t f; setup(&f, 0); pump(&f);              /* TS=0 (lowest) -> IDLE */
    tick(&f, MSTP_TNO_TOKEN_MS);                  /* SilenceTimer = 500 */
    pump(&f);
    /* IDLE.LostToken -> NO_TOKEN, then NO_TOKEN.GenerateToken (slot 0) */
    CHECK(f.node.state == MSTP_MGR_POLL_FOR_MANAGER, "-> POLL_FOR_MANAGER");
    CHECK(f.mock.n == 1 && last(&f)->ft == MSTP_FT_POLL_FOR_MANAGER, "sent PFM");
    CHECK(last(&f)->dst == 1, "PFM to (TS+1)=1");
    CHECK(f.node.ns == 0, "NS=TS (unknown)");

    /* A successor replies -> we pass it the token. */
    feed_frame(&f, MSTP_FT_REPLY_TO_POLL_FOR_MANAGER, /*dst*/0, /*src*/1);
    pump(&f);
    CHECK(f.node.state == MSTP_MGR_PASS_TOKEN, "-> PASS_TOKEN");
    CHECK(f.node.ns == 1, "NS = replier");
    CHECK(last(&f)->ft == MSTP_FT_TOKEN && last(&f)->dst == 1, "sent Token to NS");
    CHECK(f.node.ps == 0 && f.node.token_count == 0, "PS=TS, TokenCount=0");
}

/* PASS_TOKEN: successor starts transmitting -> SawTokenUser -> IDLE. */
static void t_pass_token_saw_user(void)
{
    printf("t_pass_token_saw_user\n");
    fix_t f; setup(&f, 0); pump(&f);
    tick(&f, MSTP_TNO_TOKEN_MS); pump(&f);                    /* -> POLL_FOR_MANAGER */
    feed_frame(&f, MSTP_FT_REPLY_TO_POLL_FOR_MANAGER, 0, 1);  /* -> PASS_TOKEN */
    pump(&f);
    unsigned before = f.mock.n;
    /* Successor begins a frame (to someone else): EventCount rises, silence low. */
    feed_frame(&f, MSTP_FT_TOKEN, /*dst*/2, /*src*/1);
    CHECK(f.rx.silence_timer < MSTP_TUSAGE_TIMEOUT_MS, "silence low after activity");
    CHECK(f.rx.event_count > MSTP_NMIN_OCTETS, "EventCount > Nmin_octets");
    pump(&f);
    CHECK(f.node.state == MSTP_MGR_IDLE, "SawTokenUser -> IDLE");
    CHECK(f.mock.n == before, "no extra frame sent");
}

/* PASS_TOKEN: no successor usage -> retry once -> then find a new successor. */
static void t_pass_token_retry_then_find(void)
{
    printf("t_pass_token_retry_then_find\n");
    fix_t f; setup(&f, 0); pump(&f);
    tick(&f, MSTP_TNO_TOKEN_MS); pump(&f);
    feed_frame(&f, MSTP_FT_REPLY_TO_POLL_FOR_MANAGER, 0, 1);
    pump(&f);                                    /* PASS_TOKEN, NS=1 */
    unsigned n0 = f.mock.n;
    /* Tusage_timeout with no activity -> RetrySendToken (Nretry_token=1). */
    tick(&f, MSTP_TUSAGE_TIMEOUT_MS);
    pump(&f);
    CHECK(f.node.state == MSTP_MGR_PASS_TOKEN, "re-enter PASS_TOKEN on retry");
    CHECK(f.node.retry_count == 1, "RetryCount incremented");
    CHECK(f.mock.n == n0 + 1 && last(&f)->ft == MSTP_FT_TOKEN, "token retransmitted");
    /* Still nothing -> retries exhausted -> FindNewSuccessor -> POLL_FOR_MANAGER. */
    tick(&f, MSTP_TUSAGE_TIMEOUT_MS);
    pump(&f);
    CHECK(f.node.state == MSTP_MGR_POLL_FOR_MANAGER, "-> POLL_FOR_MANAGER");
    CHECK(last(&f)->ft == MSTP_FT_POLL_FOR_MANAGER, "sent PFM to find successor");
    CHECK(last(&f)->dst == 2, "PFM to (NS+1)=2");
    CHECK(f.node.ns == 0, "NS reset to TS");
}

/* Poll sweep with no replies ends in DeclareSoleManager -> USE_TOKEN. */
static void t_declare_sole_manager(void)
{
    printf("t_declare_sole_manager\n");
    fix_t f; setup(&f, 0);
    mstp_mgr_set_limits(&f.node, /*nmax_manager*/2, /*nmax_info_frames*/1);
    pump(&f);
    tick(&f, MSTP_TNO_TOKEN_MS); pump(&f);       /* GenerateToken: PFM to 1 */
    CHECK(f.node.state == MSTP_MGR_POLL_FOR_MANAGER, "POLL after generate");
    CHECK(last(&f)->dst == 1, "first PFM to 1");
    /* No reply within Tusage_timeout -> SendNextPFM to 2. */
    tick(&f, MSTP_TUSAGE_TIMEOUT_MS); pump(&f);
    CHECK(f.node.state == MSTP_MGR_POLL_FOR_MANAGER, "re-enter POLL");
    CHECK(last(&f)->ft == MSTP_FT_POLL_FOR_MANAGER && last(&f)->dst == 2, "next PFM to 2");
    /* (PS+1) mod 3 == 0 == TS now -> DeclareSoleManager (one step; a sole
     * manager then circulates the token internally, so don't pump past it). */
    tick(&f, MSTP_TUSAGE_TIMEOUT_MS);
    CHECK(mstp_mgr_step(&f.node), "DeclareSoleManager fires");
    CHECK(f.node.sole_manager, "SoleManager TRUE");
    CHECK(f.node.state == MSTP_MGR_USE_TOKEN, "-> USE_TOKEN");
}

/* Sole manager holding the token with nothing to send keeps circulating it
 * (SoleManager transition in DONE_WITH_TOKEN), until Npoll triggers a poll. */
static void t_sole_manager_circulates(void)
{
    printf("t_sole_manager_circulates\n");
    fix_t f; setup(&f, 0);
    mstp_mgr_set_limits(&f.node, 2, 1);
    pump(&f);
    tick(&f, MSTP_TNO_TOKEN_MS); pump(&f);          /* GenerateToken -> POLL (PFM 1) */
    tick(&f, MSTP_TUSAGE_TIMEOUT_MS); pump(&f);     /* SendNextPFM   -> POLL (PFM 2) */
    tick(&f, MSTP_TUSAGE_TIMEOUT_MS);
    mstp_mgr_step(&f.node);                      /* DeclareSoleManager -> USE_TOKEN */
    CHECK(f.node.sole_manager && f.node.state == MSTP_MGR_USE_TOKEN, "sole in USE_TOKEN");
    uint16_t tc0 = f.node.token_count;
    unsigned n_before = f.mock.n;
    /* USE_TOKEN NothingToSend -> DONE_WITH_TOKEN */
    mstp_mgr_step(&f.node);
    CHECK(f.node.state == MSTP_MGR_DONE_WITH_TOKEN, "NothingToSend -> DONE_WITH_TOKEN");
    /* DONE_WITH_TOKEN SoleManager (TokenCount<Npoll) -> USE_TOKEN, TokenCount++ */
    mstp_mgr_step(&f.node);
    CHECK(f.node.state == MSTP_MGR_USE_TOKEN, "SoleManager circulates -> USE_TOKEN");
    CHECK(f.node.token_count == tc0 + 1, "TokenCount incremented");
    CHECK(f.mock.n == n_before, "no frame emitted while circulating (nothing to send)");
}

/* ANSWER_DATA_REQUEST with no immediate reply -> Reply Postponed. */
static void t_answer_deferred(void)
{
    printf("t_answer_deferred\n");
    fix_t f; setup(&f, 5); pump(&f);
    feed_frame(&f, MSTP_FT_DATA_EXPECTING_REPLY, /*dst*/5, /*src*/2);
    pump(&f);
    CHECK(f.node.state == MSTP_MGR_IDLE, "back to IDLE after answering");
    CHECK(f.mock.indications == 1, "request indicated up");
    CHECK(f.mock.n == 1 && last(&f)->ft == MSTP_FT_REPLY_POSTPONED, "sent Reply Postponed");
    CHECK(last(&f)->dst == 2, "postpone to requester");
}

int main(void)
{
    t_initialize();
    t_reply_to_pfm();
    t_token_next_station_unknown();
    t_broadcast_token_unwanted();
    t_frame_not_for_us();
    t_lost_token_generate();
    t_pass_token_saw_user();
    t_pass_token_retry_then_find();
    t_declare_sole_manager();
    t_sole_manager_circulates();
    t_answer_deferred();

    printf("\n%d checks, %d failures\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
