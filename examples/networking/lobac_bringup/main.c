/*
 * Copyright (C) 2026 WPI MQP (6LoBAC) — Kerry Lynn
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * 6LoBAC (RFC 8163) bring-up test for the ST Nucleo-F767ZI + DFRobot RS485
 * Shield v1.0 (DFR0259).  Periodically emits a BACnet MS/TP "IPv6 encapsulation"
 * frame (Frame Type 34) on the Arduino serial port (USART6) with manual RS-485
 * direction control on D2, and prints the frame it sent to the console.
 *
 * This is the FIRST on-wire milestone: prove UART + DE timing + framing produce
 * a frame that mstpcap / the 6LoBAC Wireshark dissector accept.  No IPv6 stack,
 * no MS/TP token FSM yet — see drivers/mstp/PORTING.md.
 *
 * Shield jumpers (required): serial-select -> HARDWARE serial (D0/D1);
 * transceiver mode -> MANU (manual); EN = D2, HIGH = transmit.
 */

#include <stdio.h>
#include <stdint.h>

#include "periph/uart.h"
#include "periph/gpio.h"
#include "ztimer.h"

#include "mstp_frame.h"

/* --- configuration (override with CFLAGS if needed) ---------------------- */

/* USART6 == UART_DEV(1) on nucleo-f767zi == Arduino D0(PG9)/D1(PG14). */
#ifndef MSTP_UART_DEV
#define MSTP_UART_DEV       UART_DEV(1)
#endif

/*
 * MS/TP bus baud. The BDK test bed runs at 115.2 kbit/s, which is also the rate
 * RFC 8163 requires every 6LoBAC node to support (lower rates optional).
 * Match the ring you are joining.
 */
#ifndef MSTP_BAUD
#define MSTP_BAUD           (115200U)
#endif

/* DFRobot DFR0259 EN line = Arduino D2 = PF15 on Nucleo-144, HIGH = transmit. */
#ifndef MSTP_DE_PIN
#define MSTP_DE_PIN         GPIO_PIN(PORT_F, 15)
#endif

#ifndef MSTP_SRC_ADDR
#define MSTP_SRC_ADDR       (0x03U)   /* this node */
#endif
#ifndef MSTP_DST_ADDR
#define MSTP_DST_ADDR       (0x01U)   /* an existing BDK node */
#endif

#ifndef MSTP_TX_PERIOD_MS
#define MSTP_TX_PERIOD_MS   (2000U)
#endif

/* ------------------------------------------------------------------------- */

static volatile uint32_t rx_bytes;

static void rx_cb(void *arg, uint8_t data)
{
    (void)arg;
    (void)data;
    rx_bytes++;   /* bring-up: just count RX activity on the bus */
}

/* Transmit a frame with manual RS-485 direction control. */
static void mstp_tx(const uint8_t *frame, size_t len)
{
    gpio_set(MSTP_DE_PIN);                 /* drive the bus (transmit) */
    uart_write(MSTP_UART_DEV, frame, len);

    /*
     * HAZARD (see session log): release DE only AFTER the last octet has fully
     * shifted out. RIOT's periph_uart exposes no portable TX-complete flag, so
     * wait one frame-time + margin. Releasing DE too early truncates the frame
     * on the wire — the same class of bug as the ATmega rs485 write-ordering
     * fault. A precise TC-driven turn-off is a TODO for the real driver.
     */
    uint32_t us = ((uint32_t)len + 2U) * 10U * 1000000U / MSTP_BAUD;
    ztimer_sleep(ZTIMER_USEC, us);
    gpio_clear(MSTP_DE_PIN);               /* release the bus (receive) */
}

int main(void)
{
    puts("\n6LoBAC bring-up: MS/TP Frame Type 34 emitter (nucleo-f767zi)");
    printf("UART=%u baud=%lu  DE=PF15(D2)  src=0x%02x dst=0x%02x\n",
           (unsigned)MSTP_UART_DEV, (unsigned long)MSTP_BAUD,
           MSTP_SRC_ADDR, MSTP_DST_ADDR);

    if (uart_init(MSTP_UART_DEV, MSTP_BAUD, rx_cb, NULL) != UART_OK) {
        puts("FATAL: uart_init failed");
        return 1;
    }
    gpio_init(MSTP_DE_PIN, GPIO_OUT);
    gpio_clear(MSTP_DE_PIN);               /* idle = receive */

    /* A stand-in MSDU: the first bytes of an IPv6 header (version/traffic
     * class/flow + payload length) — enough to see IPHC-shaped data on the
     * wire. Replace with a real 6LoBAC packet once the stack is wired. */
    static const uint8_t msdu[] = {
        0x60, 0x00, 0x00, 0x00, 0x00, 0x08, 0x3a, 0xff,
    };

    uint8_t frame[64];
    uint32_t seq = 0;

    while (1) {
        size_t n = mstp_build_ipv6_frame(MSTP_SRC_ADDR, MSTP_DST_ADDR,
                                         msdu, sizeof(msdu),
                                         frame, sizeof(frame));
        if (n == 0) {
            puts("build error");
        }
        else {
            printf("[%lu] TX %zu bytes (rx seen=%lu):",
                   (unsigned long)seq++, n, (unsigned long)rx_bytes);
            for (size_t i = 0; i < n; i++) {
                printf(" %02x", frame[i]);
            }
            puts("");
            mstp_tx(frame, n);
        }
        ztimer_sleep(ZTIMER_MSEC, MSTP_TX_PERIOD_MS);
    }
    return 0;
}
