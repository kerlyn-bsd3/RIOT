/*
 * Copyright (C) 2026 WPI MQP (6LoBAC) — Kerry Lynn
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Host tests for the portable link glue (mstp_link.c): the SPSC ring, the
 * control-frame builder, and — most importantly — a full ring-join driven
 * entirely through mstp_link_pump(), i.e. exactly the code path the on-target
 * FSM thread runs, minus UART/GPIO/ztimer. The ISR is simulated by pushing
 * status/octet slots into the ring; time is simulated by the elapsed_ms arg.
 *
 * Build (from drivers/mstp/tests-host/):  make -f Makefile.host check
 */

#include <stdio.h>
#include <string.h>

#include "mstp_link.h"
#include "mstp_mgr.h"
#include "mstp_fsm.h"

static int g_fail = 0, g_checks = 0;
#define CHECK(cond, ...) do {                                             \
        g_checks++;                                                       \
        if (!(cond)) { g_fail++;                                          \
            printf("  FAIL %s:%d: ", __func__, __LINE__);                 \
            printf(__VA_ARGS__); printf("\n"); }                         \
    } while (0)

/* ---- mock port (records transmitted frames) --------------------------- */
typedef struct { uint8_t ft, dst; } sent_t;
typedef struct { sent_t sent[64]; unsigned n; } mock_t;

static void mock_send(void *ctx, uint8_t ft, uint8_t dst, uint8_t src,
                      const uint8_t *data, uint16_t len)
{
    (void)src; (void)data; (void)len;
    mock_t *m = ctx;
    if (m->n < 64) { m->sent[m->n].ft = ft; m->sent[m->n].dst = dst; m->n++; }
}
static const mstp_mgr_port_t PORT = { .send_frame = mock_send };

/* ---- fixture ---------------------------------------------------------- */
typedef struct {
    mstp_rx_fsm_t rx;
    mstp_mgr_t    mgr;
    mstp_ring_t   ring;
    mock_t        mock;
} fix_t;

static void setup(fix_t *f, uint8_t ts)
{
    memset(f, 0, sizeof(*f));
    mstp_rx_fsm_init(&f->rx, ts);
    mstp_ring_reset(&f->ring);
    mstp_mgr_init(&f->mgr, &f->rx, ts, &PORT, &f->mock);
}

/* Simulate the UART ISR receiving a header-only frame: push each octet as an
 * OK-status slot into the ring. */
static void isr_rx_ctrl(fix_t *f, uint8_t ft, uint8_t dst, uint8_t src)
{
    uint8_t wire[8];
    mstp_build_ctrl_frame(ft, dst, src, wire);
    for (unsigned i = 0; i < sizeof(wire); i++) {
        mstp_ring_put(&f->ring, wire[i]);         /* status OK (no MSTP_OCTET_ERR) */
    }
}

static const sent_t *last(fix_t *f) { return f->mock.n ? &f->mock.sent[f->mock.n-1] : NULL; }

/* ---- ring unit tests -------------------------------------------------- */
static void t_ring_basic(void)
{
    printf("t_ring_basic\n");
    mstp_ring_t r; mstp_ring_reset(&r);
    uint16_t v;
    CHECK(!mstp_ring_get(&r, &v), "empty get fails");
    CHECK(mstp_ring_put(&r, 0x00AA), "put ok");
    CHECK(mstp_ring_put(&r, MSTP_OCTET_ERR | 0x00), "put err-slot ok");
    CHECK(mstp_ring_get(&r, &v) && v == 0x00AA, "fifo order 1");
    CHECK(mstp_ring_get(&r, &v) && (v & MSTP_OCTET_ERR), "err bit preserved");
    CHECK(!mstp_ring_get(&r, &v), "empty again");
}

static void t_ring_full(void)
{
    printf("t_ring_full\n");
    mstp_ring_t r; mstp_ring_reset(&r);
    unsigned ok = 0;
    for (unsigned i = 0; i < MSTP_RX_RING_LEN + 10U; i++) {
        if (mstp_ring_put(&r, (uint16_t)(i & 0xFF))) ok++;
    }
    CHECK(ok == MSTP_RX_RING_LEN - 1U, "capacity N-1 (one slot reserved), got %u", ok);
    CHECK(r.dropped == 11U, "11 drops counted, got %lu", (unsigned long)r.dropped);
}

/* ---- control-frame builder is a valid on-wire frame ------------------- */
static void t_ctrl_frame_parses(void)
{
    printf("t_ctrl_frame_parses\n");
    /* Build a Token to station 7 from 3, parse it through the real Receive FSM. */
    uint8_t wire[8];
    size_t n = mstp_build_ctrl_frame(MSTP_FT_TOKEN, 7, 3, wire);
    CHECK(n == 8, "8-octet control frame");
    CHECK(wire[0] == 0x55 && wire[1] == 0xFF, "preamble");
    mstp_rx_fsm_t rx; mstp_rx_fsm_init(&rx, 7);
    mstp_rx_result_t res = MSTP_RX_NONE;
    for (size_t i = 0; i < n; i++) res = mstp_rx_fsm_octet(&rx, wire[i]);
    CHECK(res == MSTP_RX_FRAME, "delivered (NoData)");
    CHECK(rx.received_valid_frame, "ReceivedValidFrame set");
    CHECK(rx.frame.frame_type == MSTP_FT_TOKEN && rx.frame.source == 3, "hdr fields");
}

/* ---- end-to-end: pump drives a full join (ISR ring -> RX FSM -> mgr) --- */

/* Lowest node with an empty ring: silence accrues -> LostToken -> NO_TOKEN ->
 * GenerateToken (PFM); a Reply To PFM arrives via the ring -> pass token. */
static void t_pump_generate_and_pass(void)
{
    printf("t_pump_generate_and_pass\n");
    fix_t f; setup(&f, 0);
    mstp_link_pump(&f.rx, &f.mgr, &f.ring, 0);       /* INITIALIZE -> IDLE */
    CHECK(f.mgr.state == MSTP_MGR_IDLE, "IDLE after first pump");

    /* 500 ms of silence in one tick: LostToken -> NO_TOKEN -> GenerateToken. */
    mstp_link_pump(&f.rx, &f.mgr, &f.ring, MSTP_TNO_TOKEN_MS);
    CHECK(f.mgr.state == MSTP_MGR_POLL_FOR_MANAGER, "-> POLL_FOR_MANAGER");
    CHECK(f.mock.n == 1 && last(&f)->ft == MSTP_FT_POLL_FOR_MANAGER, "sent PFM");
    CHECK(last(&f)->dst == 1, "PFM to (TS+1)=1");

    /* Successor replies (ISR pushes the RTPFM octets); pump passes the token. */
    isr_rx_ctrl(&f, MSTP_FT_REPLY_TO_POLL_FOR_MANAGER, /*dst*/0, /*src*/1);
    mstp_link_pump(&f.rx, &f.mgr, &f.ring, 1);
    CHECK(f.mgr.state == MSTP_MGR_PASS_TOKEN, "-> PASS_TOKEN");
    CHECK(f.mgr.ns == 1, "NS = replier");
    CHECK(last(&f)->ft == MSTP_FT_TOKEN && last(&f)->dst == 1, "Token to NS");
}

/* Join as pollee: a PFM addressed to us (pushed via the ring) is answered with
 * a Reply To PFM — the likely path with the BDK monitor polling. */
static void t_pump_reply_to_pfm(void)
{
    printf("t_pump_reply_to_pfm\n");
    fix_t f; setup(&f, 5);
    mstp_link_pump(&f.rx, &f.mgr, &f.ring, 0);       /* -> IDLE */
    isr_rx_ctrl(&f, MSTP_FT_POLL_FOR_MANAGER, /*dst*/5, /*src*/1);
    mstp_link_pump(&f.rx, &f.mgr, &f.ring, 1);
    CHECK(f.mgr.state == MSTP_MGR_IDLE, "stay IDLE");
    CHECK(f.mock.n == 1 && last(&f)->ft == MSTP_FT_REPLY_TO_POLL_FOR_MANAGER, "sent RTPFM");
    CHECK(last(&f)->dst == 1, "RTPFM to poller");
}

/* A ReceiveError slot in the ring becomes ReceivedInvalidFrame in the FSM. */
static void t_pump_error_slot(void)
{
    printf("t_pump_error_slot\n");
    fix_t f; setup(&f, 5);
    mstp_link_pump(&f.rx, &f.mgr, &f.ring, 0);
    /* Start a frame (preamble+header) then inject a framing error mid-header. */
    mstp_ring_put(&f.ring, 0x55);
    mstp_ring_put(&f.ring, 0xFF);
    mstp_ring_put(&f.ring, MSTP_FT_TOKEN);           /* into HEADER */
    mstp_ring_put(&f.ring, MSTP_OCTET_ERR);          /* ReceiveError in HEADER */
    mstp_link_pump(&f.rx, &f.mgr, &f.ring, 1);
    CHECK(f.rx.stats.receive_error >= 1, "ReceiveError counted");
    CHECK(f.mgr.state == MSTP_MGR_IDLE, "IDLE (invalid frame consumed)");
    /* Manager cleared ReceivedInvalidFrame in IDLE. */
    CHECK(!f.rx.received_invalid_frame, "ReceivedInvalidFrame cleared by mgr");
}

int main(void)
{
    t_ring_basic();
    t_ring_full();
    t_ctrl_frame_parses();
    t_pump_generate_and_pass();
    t_pump_reply_to_pfm();
    t_pump_error_slot();
    printf("\n%d checks, %d failures\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
