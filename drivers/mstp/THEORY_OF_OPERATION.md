# MS/TP (6LoBAC) driver — Theory of Operation

*ANSI/ASHRAE 135-2024 Clause 9 (Multidrop Serial Bus / Token Passing) on RIOT,
for IPv6-over-MS/TP (RFC 8163 / draft-lynn-6lo-rfc8163-bis). This document
records the major design decisions and their trade-offs; it is a companion to
`PORTING.md` (the upstream integration map) and `notes/mstp-clause9-notes.md`
(the distilled normative facts).*

## 1. Scope and current status

The driver implements the two normative MS/TP state machines and the wiring
needed for a node to **join a token ring** and participate in token passing
(INITIALIZE → IDLE → Poll For Manager → token pass). IPv6 data exchange —
the `netdev`/`gnrc_netif` path — is deliberately **not** started yet; the token
ring is the prerequisite milestone and is where the timing-sensitive risk lives.

Implemented and host-verified: the Receive Frame FSM (9.5.4), the Manager Node
FSM (9.5.6), SendFrame dispatch (9.5.5), the ISR↔FSM link glue, and the
`mstp_ring` bring-up app. Deferred: the `netdev_driver_t` ops + gnrc attachment
(IPv6 data), the Subordinate Node FSM (9.5.7), and the `periph_uart_rx_error`
feature (see §8).

## 2. Clean-room discipline

Every transition in `mstp_fsm.c` (9.5.4) and `mstp_mgr.c` (9.5.6) is transcribed
directly from the 2024 standard's normative prose, with the clause and named
transition cited in-source, and each is host-tested. Nothing is inferred from a
GPL implementation (bacnet-stack) or the earlier contiki tree; COBS and CRC-32K
come from the BSD-licensed RFC 8163-bis reference code. This is not ceremony:
three defects earlier in this port (a poly-0x07 header CRC, the COBS
phantom-zero rule, a `uint8_t` index that wrapped at 254) all came from filling
a gap with plausible reasoning instead of the text. Reading 9.5.6's bodies (vs.
the state/transition name-map) immediately caught a seventh DONE_WITH_TOKEN
transition, `SoleManagerRestartMaintenancePFM`, that the map had missed.

## 3. Two machines, one set of shared variables

The standard describes two concurrent machines — the Receive Frame FSM and the
Manager Node FSM — communicating only through the Clause 9.5.2 variables. We
place the variables that the *Receive* machine clocks (`EventCount`,
`SilenceTimer`, and the latched `ReceivedValidFrame` / `ReceivedInvalidFrame`
flags, plus the last frame's header) inside `mstp_rx_fsm_t`, and give the Manager
FSM a pointer to that instance. The Manager reads and clears them exactly as the
text prescribes ("set by the Receive State Machine … set to FALSE by the main
state machine"). This keeps each variable with the machine that naturally owns
its update, and it makes both machines individually host-testable.

A consequence worth noting: the existing Receive FSM had never implemented
`EventCount`. Completing it required care — 9.5.4 increments `EventCount` on
*every octet or error event handled in IDLE, PREAMBLE, or HEADER*, and **not**
during DATA/SKIP_DATA/RECEIVE_ENCODED_FIELDS nor on a silence timeout. That
distinction matters: `EventCount` vs `Nmin_octets` is how PASS_TOKEN's
`SawTokenUser` and NO_TOKEN detect that another node has begun transmitting.

## 4. SendFrame is a port callback, not FSM code

SendFrame (9.5.5) is inherently the RS-485 procedure — wait out `Tturnaround`,
enable the driver, clock out octets while clearing `SilenceTimer`, honour
`Tpostdrive`, disable the driver — plus frame assembly (header CRC, and for data
frames the COBS Encoded Data / Encoded CRC-32K per 9.10.2). We expose it as the
`mstp_mgr_port_t::send_frame` callback so the Manager FSM (9.5.6, portable)
merely *decides which frame to send*, and the hardware realises it. Benefits:
the FSM core stays hardware-free and host-testable, and the frame-assembly half
reuses code already proven on the wire (`mstp_build_ipv6_frame`, Session 9)
rather than duplicating it. Control frames (Token, Poll For Manager, Reply To
PFM, Reply Postponed) are header-only Data-Length-0 frames built by
`mstp_build_ctrl_frame`.

## 5. Single-threaded FSM execution with an ISR ring buffer

The most consequential integration decision. The UART RX ISR does **no
interpretation**: for each octet it pushes one 16-bit slot — `status << 8 |
data` — into a single-producer/single-consumer lock-free ring, and raises a
thread flag. A 1 ms `ztimer` periodic raises a second flag. One FSM thread is
the sole consumer: it drains the ring (dispatching each slot to the correct
Receive-FSM entry point), advances `SilenceTimer`, and runs the Manager FSM to
quiescence — all via `mstp_link_pump()`.

Why one thread rather than running the Receive FSM in the ISR: the two machines
share the 9.5.2 variables, and `send_frame` blocks for the duration of a UART
transmit (≈ ms). We therefore cannot simply wrap a Manager step in
`irq_disable`. Running **both** FSMs plus silence accounting in a single context
makes every shared-variable access race-free with **zero critical sections**;
the ISR touches only the lock-free ring, and the tick only sets a flag.

**Preserving `ReceiveError` across the hand-off.** The normative UART Receiver
Model (9.5.1) makes `DataAvailable` and `ReceiveError` distinct FSM inputs, and
states the data register's contents after a framing/overrun error are
unspecified. To keep that distinction, each ring slot carries the octet **and**
its status: the ISR captures the hardware's framing/overrun flag into the status
byte, and the drain loop dispatches `MSTP_OCTET_ERR` slots to
`mstp_rx_fsm_error()` (which ignores the data byte) and clean slots to
`mstp_rx_fsm_octet()`. Detection is in the hardware/ISR; the *reaction* is in the
Receive FSM at drain time. This is why every octet occupies two bytes in the
ring rather than one.

Ring sizing: capacity defaults to 2048 slots — enough to hold a maximum MS/TP
frame with margin so a scheduling gap cannot overrun the ring at 115.2 kbit/s;
control-only rings need a tiny fraction of that. A full ring increments a
`dropped` counter (surfaced by the bring-up app); lost octets then manifest as a
CRC or silence-timeout failure → `ReceivedInvalidFrame`, so the failure mode is
safe, not silent corruption.

## 6. The hybrid layering, and the path to `netdev`/gnrc

The integration is deliberately split so that the token-passing bring-up and the
eventual IPv6 driver are the **same core with different amounts of adapter**:

| Layer | Files | Depends on RIOT? | Host-tested? |
|---|---|---|---|
| FSM cores (9.5.4, 9.5.6) | `mstp_fsm.c`, `mstp_mgr.c` | no | yes (59 checks) |
| Link glue (ring, pump, ctrl builder) | `mstp_link.c` | no | yes (26 checks) |
| RS-485 / timer / thread | `mstp_run.c` | yes | stub-compiled |
| netdev / gnrc (IPv6 data) | `mstp_netdev.c` | yes | **future** |

Because the `mstp_mgr_port_t` boundary and the `mstp_link_pump()` entry are
stable, converting from "bring-up app" to "full driver" is **additive**, not a
rewrite: the FSM files do not change, `send_frame` and the ISR/tick/thread wiring
transplant verbatim, and the netdev ops (`_send` enqueues into what `next_tx`
drains while holding the token; `_recv`/`_isr` deliver what `indicate` collects)
wrap the existing core. The trade-off we accepted: the upstream artifact is the
`netdev` driver, so the bring-up app is not itself upstreamable — but it costs
nothing toward that goal, since all reusable code already lives in `drivers/mstp`
and only a ~30-line `main()` is app-local. The one structural mismatch to keep in
mind is that `netdev` is a packet-in/packet-out event model whereas the Manager
FSM is a continuously-running, timer-driven machine; even under `netdev` the FSM
runs from the same thread/timer, and `_send` only *enqueues* — transmission still
happens when the node holds the token.

## 7. DE (driver-enable) timing

For releasing the RS-485 driver after a transmit, `send_frame` uses a **computed
frame-time delay** (one frame-time + a two-character margin), the approach
already proven to put a spec-correct frame on the wire in Session 9, because
`periph_uart` exposes no portable TX-complete flag. The sanctioned alternative
(notes §2) — append a trailing X'FF' padding octet and drop DE on the UART's
transmit-complete interrupt — is more robust for the tight token turnaround
(`Tusage_timeout` = 20 ms) and is the planned refinement **if** the token's last
octet is ever observed to clip in the BDK/Wireshark capture. We start simple and
escalate only on evidence, because the delay approach is known-good and the
padding-octet path is more un-compiled code to get right.

## 8. Deferred: `ReceiveError` hardware source

The ring/dispatch plumbing for `ReceiveError` is in place from the start, but the
stock `periph_uart` callback conveys only the octet, so today the ISR always
pushes `status = OK`. Supplying a *real* framing/overrun flag needs a
`periph_uart_rx_error` feature (notes §3, mirroring the existing
`periph_uart_rxstart_irq`). Until that lands, corruption is still caught by the
header/data CRCs and by `SilenceTimer`/`Tframe_abort` → `ReceivedInvalidFrame`,
so token-ring recovery is unaffected; the framing-error path simply isn't
exercised. The `#ifdef` hook in `mstp_run.c` marks exactly where the error-aware
callback attaches.

## 9. Timing and parameters

Timers are compared against `SilenceTimer`, kept in milliseconds by the Receive
FSM (nominal 5 ms resolution per 9.5.2; the 1 ms tick evaluates them finely
enough, including the 10 ms-wide NO_TOKEN slot window `Tslot·TS`).

| Parameter | Value | Used by |
|---|---|---|
| `Tno_token` | 500 ms | IDLE LostToken; NO_TOKEN slot windows |
| `Tusage_timeout` | 20 ms | PASS_TOKEN, POLL_FOR_MANAGER |
| `Treply_timeout` | 255 ms | WAIT_FOR_REPLY |
| `Tslot` | 10 ms | NO_TOKEN slot = `Tno_token + Tslot·TS` |
| `Tturnaround` | 40 bit times | SendFrame pre-transmit wait |
| `Npoll` | 50 | maintenance Poll For Manager cadence |
| `Nretry_token` | 1 | Token/PFM retries |
| `Nmin_octets` | 4 | line-activity threshold (`EventCount`) |

## 10. Verification strategy

The FSM cores and the link glue are host-compiled and unit-tested off-target
(`drivers/mstp/tests-host/`, `make -f Makefile.host check`): 59 Manager-FSM
checks and 26 link/ring/pump checks, the latter including a full ring-join driven
end-to-end through `mstp_link_pump()` — the exact code path the on-target thread
runs, minus UART/GPIO/ztimer. The RIOT glue (`mstp_run.c`, `mstp.h`) cannot be
built off-target, so it is compiled against stub headers that mirror the verified
RIOT signatures to catch structural errors; final verification is a VM build
(`make BOARD=nucleo-f767zi`) and an on-wire capture with the BDK monitor →
mstpcap → Wireshark, confirming the node joins the ring and passes the token.

## 11. Known gaps / future work

The COBS data-frame branch of `send_frame` is a documented TODO (needs
`mstp_frame.*` migrated into the driver — the same cleanup as moving `mstp_crc.*`
/ `mstp_cobs.*` here); it is unreachable during token passing because `next_tx`
is NULL. The DATA/DATA_CRC non-encoded path in the Receive FSM remains stubbed
(CRC-16 unread; unreachable for all-COBS 6LoBAC traffic). The Subordinate Node
FSM (9.5.7) is not implemented. `netdev`/gnrc attachment and SCHC (RFC 8724) are
the subsequent milestones.
