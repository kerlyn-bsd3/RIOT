/*
 * Copyright (C) 2026 WPI MQP (6LoBAC) — Kerry Lynn
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * MS/TP token-ring join bring-up for the ST Nucleo-F767ZI + DFRobot RS485
 * Shield v1.0 (DFR0259). Unlike lobac_bringup (which blindly emits a Frame
 * Type 34 every 2 s), this app runs the clean-room Manager Node FSM (135-2024
 * 9.5.6) so the node actually participates in token passing: INITIALIZE -> IDLE
 * -> Poll For Manager / token pass, joining the BDK monitor's ring. No IPv6
 * data yet — see drivers/mstp/THEORY_OF_OPERATION.md.
 *
 * Shield jumpers (as in Session 9): serial-select -> HARDWARE serial (D0/D1);
 * transceiver mode -> MANU; EN = D2 (PF15), HIGH = transmit.
 */

#include <stdio.h>

#include "ztimer.h"
#include "periph/gpio.h"
#include "periph/uart.h"

#include "mstp.h"

/* USART6 == UART_DEV(1) on nucleo-f767zi == Arduino D0(PG9)/D1(PG14). */
#ifndef MSTP_UART_DEV
#define MSTP_UART_DEV       UART_DEV(1)
#endif
#ifndef MSTP_BAUD
#define MSTP_BAUD           (115200U)      /* match the ring you are joining */
#endif
/* DFR0259 EN line = Arduino D2 = PF15 on the Nucleo-144, HIGH = transmit. */
#ifndef MSTP_DE_PIN
#define MSTP_DE_PIN         GPIO_PIN(PORT_F, 15)
#endif
#ifndef MSTP_SRC_ADDR
#define MSTP_SRC_ADDR       (0x03U)        /* This Station (0..127, non-colliding) */
#endif
/*
 * Optional receive-overrun (ORE) detection. RIOT's periph_uart hides overruns
 * from the rx callback, so a lost octet is invisible (it only shows up as an
 * incomplete frame → Tframe_abort). We hand the driver a pointer to this USART's
 * status register + the ORE bit; the RX ISR peeks it before RIOT clears ORE.
 * Board/CPU-specific: USART6 is UART_DEV(1) on the nucleo-f767zi. Adjust if
 * MSTP_UART_DEV changes; leave both undefined (or ore_sr NULL) to disable.
 */
#ifndef MSTP_ORE_SR
#define MSTP_ORE_SR         (&USART6->ISR)
#endif
#ifndef MSTP_ORE_MASK
#define MSTP_ORE_MASK       (USART_ISR_ORE)
#endif
#ifndef MSTP_STATUS_PERIOD_MS
#define MSTP_STATUS_PERIOD_MS (1000U)
#endif

static mstp_t dev;

static const char *state_name(mstp_mgr_state_t s)
{
    switch (s) {
        case MSTP_MGR_INITIALIZE:          return "INITIALIZE";
        case MSTP_MGR_IDLE:                return "IDLE";
        case MSTP_MGR_USE_TOKEN:           return "USE_TOKEN";
        case MSTP_MGR_WAIT_FOR_REPLY:      return "WAIT_FOR_REPLY";
        case MSTP_MGR_DONE_WITH_TOKEN:     return "DONE_WITH_TOKEN";
        case MSTP_MGR_PASS_TOKEN:          return "PASS_TOKEN";
        case MSTP_MGR_NO_TOKEN:            return "NO_TOKEN";
        case MSTP_MGR_POLL_FOR_MANAGER:    return "POLL_FOR_MANAGER";
        case MSTP_MGR_ANSWER_DATA_REQUEST: return "ANSWER_DATA_REQUEST";
        default:                           return "?";
    }
}

int main(void)
{
    puts("\n6LoBAC: MS/TP Manager Node FSM — token-ring join (nucleo-f767zi)");
    printf("UART=%u baud=%lu DE=PF15(D2) TS=%u\n",
           (unsigned)MSTP_UART_DEV, (unsigned long)MSTP_BAUD, MSTP_SRC_ADDR);

    const mstp_params_t params = {
        .uart     = MSTP_UART_DEV,
        .baud     = MSTP_BAUD,
        .de_pin   = MSTP_DE_PIN,
        .mac_addr = MSTP_SRC_ADDR,
        .ore_sr   = MSTP_ORE_SR,
        .ore_mask = MSTP_ORE_MASK,
    };
    mstp_setup(&dev, &params, 0);
    if (mstp_start(&dev) != 0) {
        puts("FATAL: mstp_start failed");
        return 1;
    }

    /* Diagnostic poll of the FSM thread's state. These reads race benignly with
     * the FSM thread; they are for human observation only. */
    while (1) {
        ztimer_sleep(ZTIMER_MSEC, MSTP_STATUS_PERIOD_MS);

        unsigned ring_occ = (unsigned)((dev.ring.head - dev.ring.tail)
                                       & (MSTP_RX_RING_LEN - 1U));
        printf("state=%-16s TS=%u NS=%u PS=%u sole=%d TokenCount=%u\n"
               "   rx ok=%lu inv=%lu [hdrcrc=%lu src255=%lu badlen=%lu "
               "toolong=%lu abort=%lu(ore=%lu) datacrc=%lu cobs=%lu rxerr=%lu] "
               "drop=%lu txdrop=%lu ringq=%u\n",
               state_name(dev.mgr.state), dev.mgr.ts, dev.mgr.ns, dev.mgr.ps,
               (int)dev.mgr.sole_manager, (unsigned)dev.mgr.token_count,
               (unsigned long)dev.rx.stats.frames_ok,
               (unsigned long)dev.rx.stats.frames_inv,
               (unsigned long)dev.rx.stats.header_crc_err,
               (unsigned long)dev.rx.stats.src_invalid,
               (unsigned long)dev.rx.stats.bad_length,
               (unsigned long)dev.rx.stats.frame_too_long,
               (unsigned long)dev.rx.stats.frame_abort,
               (unsigned long)dev.rx.stats.abort_with_ore,
               (unsigned long)dev.rx.stats.data_crc_err,
               (unsigned long)dev.rx.stats.cobs_err,
               (unsigned long)dev.rx.stats.receive_error,
               (unsigned long)dev.ring.dropped,
               (unsigned long)dev.txing_drop, ring_occ);

        /* Manager transition counters — which edges the ring is actually taking.
         * A sustaining 2-node ring shows recvTok and sendTok climbing together;
         * a ring that keeps collapsing shows lostTok / findSucc / genTok / PFM
         * churn instead. */
        const mstp_mgr_counters_t *c = &dev.mgr.ctr;
        printf("   xitions: recvTok=%lu sendTok=%lu sawUser=%lu retryTok=%lu "
               "findSucc=%lu lostTok=%lu genTok=%lu | recvPFM=%lu replyPFM=%lu donesPFM=%lu\n",
               (unsigned long)c->received_token, (unsigned long)c->send_token,
               (unsigned long)c->saw_token_user, (unsigned long)c->retry_send_token,
               (unsigned long)c->find_new_successor, (unsigned long)c->lost_token,
               (unsigned long)c->generate_token, (unsigned long)c->received_pfm,
               (unsigned long)c->received_reply_to_pfm, (unsigned long)c->done_with_pfm);

        /* Drain the event trace: timestamped RX/TX with the FSM state at each,
         * so we can measure real response latency (poll->reply, token->pass). */
        while (dev.ev_tail != dev.ev_head) {
            const mstp_ev_t *e = &dev.evlog[dev.ev_tail & (MSTP_EVLOG_LEN - 1U)];
            const char *tag = (e->ev == MSTP_EV_RX)    ? "RX   "
                            : (e->ev == MSTP_EV_TX)    ? "TX   "
                            :                            "RXINV";
            if (e->ev == MSTP_EV_RXINV) {
                printf("   %10lu us  %s  ft=%-2u src=%-3u dst=%-3u  [%s] idx=%u\n",
                       (unsigned long)e->t_us, tag, (unsigned)e->ft,
                       (unsigned)e->src, (unsigned)e->dst,
                       state_name((mstp_mgr_state_t)e->st), (unsigned)e->aux);
            }
            else {
                printf("   %10lu us  %s  ft=%-2u src=%-3u dst=%-3u  [%s]\n",
                       (unsigned long)e->t_us, tag, (unsigned)e->ft,
                       (unsigned)e->src, (unsigned)e->dst,
                       state_name((mstp_mgr_state_t)e->st));
            }
            dev.ev_tail++;
        }
    }
    return 0;
}
