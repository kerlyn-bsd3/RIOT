/*
 * Copyright (C) 2026 WPI MQP (6LoBAC) — Kerry Lynn
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * MS/TP bus monitor (raw-echo mode) for the nRF52840-DK + DFR0259 RS-485 shield.
 *
 * Replaces the BDK monitor node: it is an ACTIVE MS/TP manager station (default
 * This Station 0) that takes part in token passing exactly as the BDK monitor did,
 * and mirrors every octet seen on the bus out the nRF52840's native USB as a
 * CDC-ACM byte stream, so Karg's capture tool works unchanged (open the new
 * /dev/ttyACM* instead of the BDK's /dev/ttyUSB0).
 *
 *  - Octets from other stations: tapped in the UART RX ISR, before the Receive
 *    Frame FSM, so malformed/aborted frames are captured as they appeared.
 *  - Our own frames (tokens, PFM replies, ...): tapped at transmit time, because
 *    DE/RE are tied on the DFR0259 and the node never receives its own frames.
 *  - The bus is half-duplex, so RX and TX octets never interleave: order is exact.
 *
 * Why native USB (not the J-Link VCP): full-speed USB (12 Mbit/s) is ~100x the
 * 115200 bus, so the relay cannot fall behind (the BDK monitor's FT232R relay
 * dropped ~1 octet per ~57 of a back-to-back stream — see the session log,
 * 2026-10-15), and CDC-ACM has no baud rate to match. The CDC-ACM port carries
 * ONLY bus octets; the console stays on the J-Link VCP (UART0).
 *
 * Timing: like the BDK path, timestamps are applied on the host when octets
 * arrive (ms-level). A device-timestamped record mode (+ Wireshark extcap) is the
 * planned next step for us-accurate timing.
 *
 * No IPv6 stack on purpose: a 6LN would add RS/NS/ARO traffic to every capture.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "shell.h"
#include "thread.h"
#include "ztimer.h"
#include "periph/gpio.h"
#include "periph/uart.h"
#include "usb/usbus.h"
#include "usb/usbus/cdc/acm.h"

#include "mstp.h"

/* ------------------------------------------------------------------------- *
 * Configuration
 * ------------------------------------------------------------------------- */
#ifndef MSTP_BAUD
#define MSTP_BAUD           (115200U)
#endif
#ifndef MSTP_SRC_ADDR
#define MSTP_SRC_ADDR       (0x00U)        /* This Station: the monitor is station 0 */
#endif
#ifndef MSTP_NMAX_MANAGER
#define MSTP_NMAX_MANAGER   (8U)           /* same as the other RIOT nodes */
#endif
#ifndef MSTP_STATUS_PERIOD_MS
#define MSTP_STATUS_PERIOD_MS (1000U)
#endif
/* CDC-ACM device->host ring buffer (power of two). 4 KiB = ~350 ms of a
 * saturated 115200 bus, i.e. lots of slack if the host is slow to read. */
#ifndef MON_USB_BUF_SIZE
#define MON_USB_BUF_SIZE    (4096U)
#endif

#if defined(BOARD_NRF52840DK)
/* UARTE1 = Arduino D0/D1 (P1.01 rx / P1.02 tx); DE = D2 = P1.03; one 0xFF pad
 * (ENDTX fires before the stop bit) — identical to mstp_6lo / mstp_6lbr. */
#define MSTP_BOARD_NAME     "nrf52840dk"
#define MSTP_UART_DEV       UART_DEV(1)
#define MSTP_DE_PIN         GPIO_PIN(1, 3)
#define MSTP_TX_PAD_OCTETS  (1U)
/* RIOT's nRF UARTE RX takes one octet per EasyDMA transfer; the ISR must fetch
 * it within one character time (~87 us @115200). USB interrupts share the default
 * priority, so put the MS/TP UART on top (same fix as the 6LBR's SLIP UART). */
#define MSTP_UART_IRQN      UARTE1_IRQn
/* The J-Link OB auto-detects UART HW flow control from the nRF's RTS line (P0.05)
 * when the host opens the console port. RIOT leaves P0.05 undriven, so the OB's
 * decision is arbitrary and it may withhold host->nRF bytes (dead-looking shell).
 * Driving RTS high before the port is opened tells the OB "no HWFC" (as in
 * mstp_6lbr). -DMON_OB_RTS_HIGH=0 disables. */
#ifndef MON_OB_RTS_HIGH
#define MON_OB_RTS_HIGH     (1)
#endif
#define MON_OB_RTS_PIN      GPIO_PIN(0, 5)
#else
#error "mstp_monitor: unsupported BOARD - add a board-config block"
#endif

/* ------------------------------------------------------------------------- *
 * USB CDC-ACM (set up by hand; auto_init_usbus is disabled in the Makefile)
 * ------------------------------------------------------------------------- */
static usbus_t usbus;
static char usbus_stack[USBUS_STACKSIZE];
static usbus_cdcacm_device_t cdcacm;
static uint8_t cdc_tx_buf[MON_USB_BUF_SIZE];

/* host -> device data is ignored (the capture tool never writes) */
static void _cdc_rx(usbus_cdcacm_device_t *c, uint8_t *data, size_t len)
{
    (void)c; (void)data; (void)len;
}

/* ------------------------------------------------------------------------- *
 * Bus tap -> USB
 * ------------------------------------------------------------------------- */
static volatile uint32_t mon_rx;        /* octets received from other stations */
static volatile uint32_t mon_tx;        /* octets we transmitted               */
static volatile uint32_t mon_drop;      /* lost: USB ring buffer full          */
static volatile uint32_t mon_offline;   /* discarded: no host has the port open */

static void _tap(void *arg, const uint8_t *buf, size_t len, bool tx)
{
    (void)arg;
    if (tx) {
        mon_tx += len;
    }
    else {
        mon_rx += len;
    }
    if (cdcacm.state == USBUS_CDC_ACM_LINE_STATE_DISCONNECTED) {
        mon_offline += len;             /* nobody listening: not an error */
        return;
    }
    size_t n = usbus_cdc_acm_submit(&cdcacm, buf, len);   /* ISR-safe */
    if (n < len) {
        mon_drop += len - n;
    }
    usbus_cdc_acm_flush(&cdcacm);                         /* ISR-safe event post */
}

/* ------------------------------------------------------------------------- *
 * Status / shell
 * ------------------------------------------------------------------------- */
static mstp_t dev;
static char _status_stack[THREAD_STACKSIZE_DEFAULT + THREAD_EXTRA_STACKSIZE_PRINTF];
static volatile bool _status_on = false;

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

static const char *usb_state(void)
{
    return (cdcacm.state == USBUS_CDC_ACM_LINE_STATE_DISCONNECTED) ? "closed" : "open";
}

static void _print_status(void)
{
    printf("state=%-16s TS=%u NS=%u PS=%u sole=%d TokenCount=%u\n"
           "   bus rx ok=%lu inv=%lu [hdrcrc=%lu abort=%lu datacrc=%lu cobs=%lu rxerr=%lu]\n"
           "   tap rx=%lu tx=%lu | usb %s drop=%lu offline=%lu\n",
           state_name(dev.mgr.state), dev.mgr.ts, dev.mgr.ns, dev.mgr.ps,
           (int)dev.mgr.sole_manager, (unsigned)dev.mgr.token_count,
           (unsigned long)dev.rx.stats.frames_ok,
           (unsigned long)dev.rx.stats.frames_inv,
           (unsigned long)dev.rx.stats.header_crc_err,
           (unsigned long)dev.rx.stats.frame_abort,
           (unsigned long)dev.rx.stats.data_crc_err,
           (unsigned long)dev.rx.stats.cobs_err,
           (unsigned long)dev.rx.stats.receive_error,
           (unsigned long)mon_rx, (unsigned long)mon_tx,
           usb_state(), (unsigned long)mon_drop, (unsigned long)mon_offline);
}

static int _cmd_status(int argc, char **argv)
{
    (void)argc; (void)argv;
    _status_on = !_status_on;
    printf("monitor status %s\n", _status_on ? "on" : "off");
    if (_status_on) {
        _print_status();
    }
    return 0;
}
SHELL_COMMAND(status, "toggle the periodic monitor/ring status printing", _cmd_status);

static void *_status_thread(void *arg)
{
    (void)arg;
    while (1) {
        ztimer_sleep(ZTIMER_MSEC, MSTP_STATUS_PERIOD_MS);
        if (_status_on) {
            _print_status();
        }
    }
    return NULL;
}

/* ------------------------------------------------------------------------- */
int main(void)
{
#if defined(MON_OB_RTS_PIN) && MON_OB_RTS_HIGH
    gpio_init(MON_OB_RTS_PIN, GPIO_OUT);
    gpio_set(MON_OB_RTS_PIN);
#endif
    printf("\n6LoBAC MS/TP bus monitor (raw echo -> native USB CDC-ACM) on %s\n",
           MSTP_BOARD_NAME);
    printf("MS/TP UART=%u baud=%lu TS=%u Nmax_manager=%u pad=%u | USB buf=%u\n",
           (unsigned)MSTP_UART_DEV, (unsigned long)MSTP_BAUD, MSTP_SRC_ADDR,
           (unsigned)MSTP_NMAX_MANAGER, (unsigned)MSTP_TX_PAD_OCTETS,
           (unsigned)MON_USB_BUF_SIZE);

    /* USB first, so the CDC-ACM port enumerates while the ring comes up. */
    usbus_init(&usbus, usbdev_get_ctx(0));
    usbus_cdc_acm_init(&usbus, &cdcacm, _cdc_rx, NULL, cdc_tx_buf, sizeof(cdc_tx_buf));
    usbus_create(usbus_stack, sizeof(usbus_stack), USBUS_PRIO, USBUS_TNAME, &usbus);

    const mstp_params_t params = {
        .uart          = MSTP_UART_DEV,
        .baud          = MSTP_BAUD,
        .de_pin        = MSTP_DE_PIN,
        .mac_addr      = MSTP_SRC_ADDR,
        .ore_sr        = NULL,
        .ore_mask      = 0,
        .tx_pad_octets = MSTP_TX_PAD_OCTETS,
        .tap           = _tap,
        .tap_arg       = NULL,
    };
    mstp_setup(&dev, &params, 0);
    if (mstp_start(&dev) != 0) {
        puts("FATAL: mstp_start failed");
        return 1;
    }
    mstp_mgr_set_limits(&dev.mgr, MSTP_NMAX_MANAGER, dev.mgr.nmax_info_frames);
    NVIC_SetPriority(MSTP_UART_IRQN, 0);

    puts("monitor running: open the native-USB ttyACM with the capture tool; "
         "'status' toggles the ring/tap report");

    thread_create(_status_stack, sizeof(_status_stack), THREAD_PRIORITY_MAIN + 1, 0,
                  _status_thread, NULL, "mon_status");

    char line[SHELL_DEFAULT_BUFSIZE];
    shell_run(NULL, line, sizeof(line));
    return 0;
}
