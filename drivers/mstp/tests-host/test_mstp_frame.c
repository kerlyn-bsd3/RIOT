/*
 * Copyright (C) 2026 WPI MQP (6LoBAC) — Kerry Lynn
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * End-to-end host test of the Frame Type 34 (IPv6-over-MS/TP) data primitive:
 * build a frame with mstp_build_ipv6_frame() (TX path) and feed it octet-by-octet
 * into the clean-room Receive FSM (RX path), asserting the decoded MSDU matches.
 * This exercises COBS encode<->decode, CRC-32K over the Encoded Data, the header
 * CRC-8, the Length = E+3 rule, and framing — the full data path minus UART/timing.
 *
 * Build (from drivers/mstp/tests-host/):  make -f Makefile.host check
 */

#include <stdio.h>
#include <string.h>

#include "mstp_fsm.h"
#include "mstp_frame.h"
#include "mstp_mgr.h"

static int g_fail = 0, g_checks = 0;
#define CHECK(cond, ...) do {                                             \
        g_checks++;                                                       \
        if (!(cond)) { g_fail++;                                          \
            printf("  FAIL %s:%d: ", __func__, __LINE__);                 \
            printf(__VA_ARGS__); printf("\n"); }                         \
    } while (0)

/* Feed a whole built frame into a fresh RX FSM, one octet at a time (as the UART
 * ISR would), returning the final result. */
static mstp_rx_result_t feed(mstp_rx_fsm_t *rx, const uint8_t *f, size_t n)
{
    mstp_rx_result_t r = MSTP_RX_NONE;
    for (size_t i = 0; i < n; i++) {
        r = mstp_rx_fsm_octet(rx, f[i]);
    }
    return r;
}

/* Unicast round-trip; MSDU deliberately contains 0x00 and 0x55 (the COBS mask)
 * and 0xFF to exercise stuffing and the preamble-avoidance XOR. */
static void t_ipv6_roundtrip_unicast(void)
{
    const uint8_t msdu[] = {
        0x60, 0x00, 0x00, 0x00, 0x00, 0x08, 0x3a, 0x40,   /* IPv6-ish header */
        0x00, 0x55, 0xFF, 0xAA, 0x55, 0x00, 0x01, 0x02,
    };
    uint8_t frame[MSTP_IPV6_FRAME_MAX(sizeof(msdu))];

    size_t n = mstp_build_ipv6_frame(3, 7, msdu, sizeof(msdu), frame, sizeof(frame));
    CHECK(n > 0, "build failed");
    CHECK(frame[0] == 0x55 && frame[1] == 0xFF, "preamble wrong");
    CHECK(frame[2] == 34, "frame type not 34 (got %u)", frame[2]);
    CHECK(frame[3] == 7 && frame[4] == 3, "dst/src wrong (%u/%u)", frame[3], frame[4]);

    mstp_rx_fsm_t rx;
    mstp_rx_fsm_init(&rx, 7);                 /* This Station == dst */
    mstp_rx_result_t r = feed(&rx, frame, n);

    CHECK(r == MSTP_RX_FRAME, "not delivered (r=%d)", (int)r);
    CHECK(rx.received_valid_frame, "ReceivedValidFrame not set");
    CHECK(rx.frame.frame_type == 34, "rx ft %u", rx.frame.frame_type);
    CHECK(rx.frame.source == 3, "rx src %u", rx.frame.source);
    CHECK(rx.frame.destination == 7, "rx dst %u", rx.frame.destination);
    CHECK(rx.frame.length == sizeof(msdu), "rx len %u != %zu",
          rx.frame.length, sizeof(msdu));
    CHECK(memcmp(rx.frame.data, msdu, sizeof(msdu)) == 0, "MSDU mismatch");
    printf("t_ipv6_roundtrip_unicast\n");
}

/* Larger MSDU (crosses a COBS 254-run boundary) with every octet value present. */
static void t_ipv6_roundtrip_large(void)
{
    uint8_t msdu[300];
    for (size_t i = 0; i < sizeof(msdu); i++) {
        msdu[i] = (uint8_t)(i * 7);           /* sweeps 0x00, 0x55, 0xFF, ... */
    }
    uint8_t frame[MSTP_IPV6_FRAME_MAX(sizeof(msdu))];

    size_t n = mstp_build_ipv6_frame(1, 2, msdu, sizeof(msdu), frame, sizeof(frame));
    CHECK(n > 0, "build failed");

    mstp_rx_fsm_t rx;
    mstp_rx_fsm_init(&rx, 2);
    mstp_rx_result_t r = feed(&rx, frame, n);

    CHECK(r == MSTP_RX_FRAME, "not delivered");
    CHECK(rx.frame.length == sizeof(msdu), "rx len %u", rx.frame.length);
    CHECK(memcmp(rx.frame.data, msdu, sizeof(msdu)) == 0, "MSDU mismatch");
    printf("t_ipv6_roundtrip_large\n");
}

/* Broadcast (dst 255) is delivered to any station. */
static void t_ipv6_broadcast(void)
{
    const uint8_t msdu[] = { 0xde, 0xad, 0xbe, 0xef, 0x00, 0x55 };
    uint8_t frame[MSTP_IPV6_FRAME_MAX(sizeof(msdu))];

    size_t n = mstp_build_ipv6_frame(3, 255, msdu, sizeof(msdu), frame, sizeof(frame));
    mstp_rx_fsm_t rx;
    mstp_rx_fsm_init(&rx, 9);                 /* not our unicast address */
    mstp_rx_result_t r = feed(&rx, frame, n);

    CHECK(r == MSTP_RX_FRAME, "broadcast not delivered");
    CHECK(rx.frame.destination == 255, "dst %u", rx.frame.destination);
    CHECK(memcmp(rx.frame.data, msdu, sizeof(msdu)) == 0, "MSDU mismatch");
    printf("t_ipv6_broadcast\n");
}

/* A frame addressed elsewhere is skipped, not delivered. */
static void t_ipv6_not_for_us(void)
{
    const uint8_t msdu[] = { 1, 2, 3, 4, 5 };
    uint8_t frame[MSTP_IPV6_FRAME_MAX(sizeof(msdu))];

    size_t n = mstp_build_ipv6_frame(3, 9, msdu, sizeof(msdu), frame, sizeof(frame));
    mstp_rx_fsm_t rx;
    mstp_rx_fsm_init(&rx, 7);                 /* we are 7; frame is for 9 */
    mstp_rx_result_t r = feed(&rx, frame, n);

    CHECK(r != MSTP_RX_FRAME, "a not-for-us frame must not be delivered");
    CHECK(!rx.received_valid_frame, "ReceivedValidFrame must stay false");
    printf("t_ipv6_not_for_us\n");
}

/* Corruption in the Encoded Data must fail the CRC-32K residue -> invalid. */
static void t_ipv6_corrupt_detected(void)
{
    const uint8_t msdu[] = { 0xca, 0xfe, 0xba, 0xbe, 0x00, 0x55, 0xff };
    uint8_t frame[MSTP_IPV6_FRAME_MAX(sizeof(msdu))];

    size_t n = mstp_build_ipv6_frame(3, 7, msdu, sizeof(msdu), frame, sizeof(frame));
    frame[8] ^= 0x01;                         /* flip a bit in the Encoded Data */

    mstp_rx_fsm_t rx;
    mstp_rx_fsm_init(&rx, 7);
    mstp_rx_result_t r = feed(&rx, frame, n);

    CHECK(r != MSTP_RX_FRAME, "corrupt frame must not be delivered");
    CHECK(rx.received_invalid_frame || r == MSTP_RX_INVALID, "should flag invalid");
    printf("t_ipv6_corrupt_detected\n");
}

/* ------------------------------------------------------------------------- *
 * TX data path: a Manager holding the token with a queued Type-34 dispatches it
 * via next_tx -> send_frame; the built frame decodes on a peer RX FSM. This is
 * the on-target path (gnrc -> mstp_tx_ipv6 queue -> USE_TOKEN -> _send_frame ->
 * mstp_build_ipv6_frame -> wire -> RX decode) minus UART/timing.
 * ------------------------------------------------------------------------- */
static uint8_t  g_q_msdu[64];
static uint16_t g_q_len;
static uint8_t  g_q_dst;
static bool     g_q_have;

static bool tx_next(void *ctx, mstp_tx_frame_t *out)
{
    (void)ctx;
    if (!g_q_have) {
        return false;
    }
    out->frame_type = MSTP_FRAME_TYPE_IPV6;
    out->dst  = g_q_dst;
    out->data = g_q_msdu;
    out->len  = g_q_len;
    g_q_have  = false;   /* dequeued */
    return true;
}

static mstp_rx_fsm_t    g_peer;
static mstp_rx_result_t g_peer_result;

static void tx_send(void *ctx, uint8_t ft, uint8_t dst, uint8_t src,
                    const uint8_t *data, uint16_t len)
{
    (void)ctx;
    if (ft == MSTP_FRAME_TYPE_IPV6 && data && len) {
        uint8_t buf[MSTP_IPV6_FRAME_MAX(64)];
        size_t n = mstp_build_ipv6_frame(src, dst, data, len, buf, sizeof(buf));
        for (size_t i = 0; i < n; i++) {
            g_peer_result = mstp_rx_fsm_octet(&g_peer, buf[i]);
        }
    }
    /* header-only control frames (the follow-on token/PFM) are ignored here */
}

static const mstp_mgr_port_t TXPORT = {
    .send_frame = tx_send,
    .indicate   = NULL,
    .next_tx    = tx_next,
    .get_reply  = NULL,
};

static void t_ipv6_tx_path(void)
{
    mstp_rx_fsm_t rx;
    mstp_mgr_t    mgr;

    mstp_rx_fsm_init(&rx, 3);
    mstp_mgr_init(&mgr, &rx, 3, &TXPORT, NULL);   /* This Station = 3 */
    mstp_rx_fsm_init(&g_peer, 7);                 /* peer address = the frame's dst */
    g_peer_result = MSTP_RX_NONE;

    const uint8_t msdu[] = {
        0x60, 0x00, 0x00, 0x00, 0x00, 0x04, 0x3a, 0x40,
        0xde, 0xad, 0xbe, 0xef,
    };
    memcpy(g_q_msdu, msdu, sizeof(msdu));
    g_q_len = sizeof(msdu);
    g_q_dst = 7;
    g_q_have = true;

    /* INITIALIZE -> IDLE */
    while (mstp_mgr_step(&mgr)) {
        if (mgr.state == MSTP_MGR_IDLE) { break; }
    }
    /* Simulate receiving a Token addressed to us -> ReceivedToken -> USE_TOKEN,
     * where next_tx hands over our queued Type-34 -> send_frame. */
    rx.frame.frame_type  = MSTP_FT_TOKEN;
    rx.frame.destination = 3;
    rx.frame.source      = 0;
    rx.received_valid_frame = true;
    while (mstp_mgr_step(&mgr)) { /* run to quiescence */ }

    CHECK(!g_q_have, "queued frame was not dispatched by next_tx");
    CHECK(g_peer_result == MSTP_RX_FRAME, "peer did not decode the tx'd frame (r=%d)",
          (int)g_peer_result);
    CHECK(g_peer.frame.frame_type == 34, "peer ft %u", g_peer.frame.frame_type);
    CHECK(g_peer.frame.source == 3, "peer src %u", g_peer.frame.source);
    CHECK(g_peer.frame.destination == 7, "peer dst %u", g_peer.frame.destination);
    CHECK(g_peer.frame.length == sizeof(msdu), "peer len %u", g_peer.frame.length);
    CHECK(memcmp(g_peer.frame.data, msdu, sizeof(msdu)) == 0, "peer MSDU mismatch");
    printf("t_ipv6_tx_path\n");
}

int main(void)
{
    t_ipv6_roundtrip_unicast();
    t_ipv6_roundtrip_large();
    t_ipv6_broadcast();
    t_ipv6_not_for_us();
    t_ipv6_corrupt_detected();
    t_ipv6_tx_path();
    printf("\n%d checks, %d failures\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
