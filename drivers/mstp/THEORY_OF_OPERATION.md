# MS/TP (6LoBAC) driver — Theory of Operation

*ANSI/ASHRAE 135-2024 Clause 9 (Multidrop Serial Bus / Token Passing) on RIOT,
for IPv6-over-MS/TP (RFC 8163 / draft-lynn-6lo-rfc8163-bis). This document
records the major design decisions and their trade-offs.*

## 1. Scope and current status

The driver implements the two normative MS/TP state machines and the wiring
needed for a node to **join a token ring** and participate in token passing
(INITIALIZE → IDLE → Poll For Manager → token pass), and carries IPv6 data over
Frame Type 34 through a `netdev`/`gnrc_netif` attachment. The token ring was the
prerequisite milestone and is where the timing-sensitive risk lives.

Implemented and verified: the Receive Frame FSM (9.5.4), the Manager Node FSM
(9.5.6), SendFrame dispatch (9.5.5), the ISR↔FSM link glue, Frame Type 34
build/parse, and the `netdev_driver_t` ops + gnrc attachment (the `mstp_6lo`
6LN app). Validated end to end — the node joins the ring, passes the token, and
answers ICMPv6 echo — on both `nrf52840dk` and `nucleo-f767zi`. Deferred: the
6LBR role, the Subordinate Node FSM (9.5.7), the `periph_uart_rx_error` feature
(see §8), and SCHC (RFC 8724).

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

**Receiving right after transmitting.** A manager spends most of its time
transmitting a frame and then immediately listening for the answer (poll → reply,
token → successor's first frame), so the TX→RX turnaround is the delicate path,
and two hazards there once cost a node its ability to accept a `Reply To Poll For
Manager`. First, on half-duplex RS-485 the transceiver echoes what we drive back
onto RX (9.5.4 anticipates this: "if a given implementation does receive its own
transmissions … the Receive Frame machine will ignore the transmissions"). We
honour that by raising a `txing` flag around the driver-enabled window; the UART
ISR drops every octet while it is set, so our own frame never enters the ring.
Second, any stray partial frame — a clipped echo, a bus-turnaround glitch, a
dropped octet — must not strand the Receive FSM mid-frame, or it will consume the
start of the next real frame and fail its header CRC. `mstp_link_pump` therefore
runs the normative `Tframe_abort` recovery (9.5.3/9.5.4): if a frame is in
progress and the line has been silent past `Tframe_abort`, it abandons the frame
and returns the receiver to IDLE to re-sync on the next preamble. Crucially this
keys on the **inter-frame silence gap**, which MS/TP always provides — not on the
optional trailing X'FF' pad octet, which a conforming peer may omit.

## 6. The hybrid layering, and the path to `netdev`/gnrc

The integration is deliberately split so that the token-passing bring-up and the
eventual IPv6 driver are the **same core with different amounts of adapter**:

| Layer | Files | Depends on RIOT? | Host-tested? |
|---|---|---|---|
| FSM cores (9.5.4, 9.5.6) | `mstp_fsm.c`, `mstp_mgr.c` | no | yes (59 checks) |
| Link glue (ring, pump, ctrl builder) | `mstp_link.c` | no | yes (26 checks) |
| RS-485 / timer / thread | `mstp_run.c` | yes | on-target |
| netdev / gnrc (IPv6 data) | `mstp_netdev.c` | yes | on-target (end-to-end ping) |

Because the `mstp_mgr_port_t` boundary and the `mstp_link_pump()` entry are
stable, converting from "bring-up app" to "full driver" was **additive**, not a
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

There are two turnarounds, and getting the **TX→RX** one wrong silently breaks
token passing on a shield (like the DFR0259) that ties /RE to DE, because then
the receiver is off for exactly as long as the driver is on.

**TX→RX (release DE after transmitting).** `send_frame` drops DE the instant
`uart_write` returns, and no sooner: RIOT's blocking `uart_write` already spins on
the USART transmit-complete (TC) flag before returning
(`cpu/stm32/periph/uart.c: wait_for_tx_complete()`), so the final stop bit is
already on the wire — dropping DE immediately satisfies "not before the stop bit"
and is well within Tpostdrive. An earlier version instead slept a fixed
`(n+2)`-character time (~0.9 ms) after `uart_write`; that was harmless for the
Session-9 TX-only emitter but *fatal* for token passing — with /RE tied to DE it
kept the receiver deaf ~0.9 ms past our last bit, straddling the peer's fast
Reply-To-PFM (a responder turns around in ~40 bit times), so the join was lost
every time (Session 14). The lesson: never hold DE past TC when a reply follows.
This does assume `uart_write` is synchronous-to-TC — true for the blocking
`periph_uart` path used here, but not for `periph_uart_nonblocking` or the DMA
path, which would need an explicit TC wait before dropping DE.

**RX→TX (wait before transmitting).** `Tturnaround` is 40 bit times (347 µs at
115.2 kbit/s) — below a 1 ms tick. `send_frame` currently waits it out with
`ZTIMER_USEC` against a millisecond-granular SilenceTimer, which is coarse. The
cleaner, self-clocking technique (used in the Contiki port) is to clock four
10-bit bytes through the UART with the *driver disabled* before the preamble:
that consumes exactly 40 bit times off the baud clock, no timer, auto-scaling with
baud. Planned for the data-frame SendFrame path.

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
RIOT signatures to catch structural errors; final verification is an on-target
build (`make BOARD=nrf52840dk` / `nucleo-f767zi`) plus an on-wire capture →
mstpcap → Wireshark and an end-to-end ICMPv6 echo, confirming the node joins the
ring, passes the token, and exchanges IPv6 data over Frame Type 34.

## 11. Known gaps / future work

The non-encoded DATA/DATA_CRC path in the Receive FSM is intentionally left
stubbed (CRC-16 unread): 6LoBAC traffic is all-COBS/CRC-32K (Frame Type 34), so
that branch is unreachable on the wire. The Subordinate Node FSM (9.5.7) is not
implemented (only the Manager Node role is). The 6LBR role and SCHC (RFC 8724)
are the subsequent milestones.

## 12. Platform notes — the STM32F7 receive path, and why the testbed moved to nRF52840

The driver was brought up first on the ST **Nucleo-F767ZI** (with a DFRobot
DFR0259 RS-485 shield), and that is where the hard, board-specific lessons were
learned. They are recorded here because they explain a decision that is otherwise
invisible in the code — why the reference testbed is built on the Nordic
**nRF52840-DK** — and because anyone porting this driver to another STM32 will
hit the same wall.

**Symptom: a header-CRC flood, not an FSM bug.** On the F767 the node would join
the ring and then, under sustained bus traffic, drop out with a storm of
header-CRC failures (`rx ok=0`, `hdrcrc` climbing). The clean-room FSM was the
first suspect, but instrumentation exonerated it: with the app handing the driver
a pointer to the USART status register and the overrun-error (ORE) bit mask
(`mstp_params_t::ore_sr` / `ore_mask`, the STM32-specific hook of §8), the ISR
could attribute each lost octet to a receive **overrun**. The counters were
decisive — `abort=211(ore=207)`: ~98 % of the invalid frames had a mid-frame
overrun on our side. The FSM was fine; the UART was dropping octets before the
FSM ever saw them.

**Root cause: RIOT reads one octet per RX interrupt, and the F7 USART has no RX
FIFO.** RIOT's mainline `cpu/stm32/periph/uart.c` takes an RX interrupt per
received octet (`USART_ISR_RXNE`) and has no RX DMA path (its DMA support is
TX-only, `DMA_MEM_TO_PERIPH`). At 115.2 kbit/s the ISR has ~87 µs to run before
the next octet arrives and overwrites the un-read data register; anything that
delays it past that window loses an octet. Raising the MS/TP USART's RX interrupt
to the highest NVIC priority (`NVIC_SetPriority(USART6_IRQn, 0)`) removed
**same-priority** ISR contention and took the overrun rate from ~95 % of aborts
down to ~0.25 % — enough for the token ring to circulate cleanly on a four-node
bus. But priority cannot fix the tail: `irq_disable()` raises PRIMASK, which
masks *all* priorities, so any critical section held longer than one octet time
still drops a byte regardless of priority. The only real fix for that class is
hardware buffering.

The obvious buffering move — enable the USART's hardware RX FIFO — **does not
exist on this part.** `USART_CR1_FIFOEN` is undefined for the STM32F7 family; the
F7 USART simply has no FIFO (confirmed at boot; a speculative
`MSTP_ENABLE_RX_FIFO` block compiled to a no-op). The only in-silicon RX
buffering on the F767 is therefore **circular RX DMA**, which RIOT's stm32 driver
does not implement, and which on the F7 carries cache-coherency baggage (DMA
cannot reach DTCM; a cacheable buffer needs explicit invalidation or an MPU
non-cacheable region). That is a register-level rewrite of the RX path, not a
configuration change — and receive buffering is a *data-path prerequisite*, not
merely a robustness nicety: a ~1500-octet Frame Type 34 has roughly 190× the
overrun exposure of an 8-octet token, so what is a rare tail on the token ring
becomes a real loss rate on IPv6 payloads.

**A self-inflicted side effect worth remembering.** Pinning the MS/TP RX
interrupt at priority 0 starved the console UART (USART3, the ST-Link VCP), which
dropped typed keystrokes. Raising the console IRQ to the same priority restored
the shell — but that is a workaround that trades one starvation for a scheduling
tie, not a clean design. It is another symptom of the same root cause: with no RX
buffering, correctness depends on ISR latency, and every other interrupt on the
chip becomes a threat to it.

**Resolution: the Nordic nRF52840-DK.** The nRF52840's UARTE peripheral receives
over **EasyDMA** — the hardware writes incoming octets straight into a RAM buffer
with no per-octet interrupt — so receive overrun under ISR contention cannot
arise in the first place. Ported to `BOARD=nrf52840dk` with no change to the
board-agnostic driver core (only the app's UART id and DE-pin GPIO differ), the
6LN ran with **every receive error counter at zero**, versus the F767's
irreducible ~0.25 % residual. That is why the RIOT-native, BDK-free testbed is
built on the nRF52840, and why the STM32-only overrun hooks (`ore_sr`/`ore_mask`,
the NVIC priority bump) are conditionalized out of the nRF build rather than
carried forward. An STM32 with a real USART FIFO (e.g. the STM32G4 /
Nucleo-G474RE) would give a small hardware cushion and is a reasonable in-family
alternative, but it still falls short of the DMA-grade buffering the data path
wants; the nRF's EasyDMA gets that for free.

**One nRF-specific TX wrinkle (the mirror image of §7).** On the F767 the
blocking `uart_write()` spins on the transmit-complete (TC) flag, so the last
stop bit is already on the wire when it returns and DE can drop immediately
(§7). On the nRF, `uart_write()` returns at the UARTE **ENDTX** event — the
moment DMA finishes the last *data* bit — which is *before* the final stop bit
has fully left the pin; dropping DE there clips the stop bit. Rather than time a
post-TX guard (fragile, baud-dependent), the driver appends a trailing **0xFF
pad octet** (`mstp_params_t::tx_pad_octets`) after the frame: the clipped bit
then belongs to the pad, not to the real last octet, and MS/TP receivers ignore
a stray 0xFF in the inter-frame idle (the preamble is `55 FF`, so a lone `FF`
never starts a frame). This is spec-permitted (135-2024 allows an optional
trailing pad octet) and is deterministic, unlike a timed drop. On the F767 the
pad count is 0; on the nRF it is 1.

The table below summarizes the receive-path contrast that drove the choice:

| | Nucleo-F767ZI (STM32F7) | nRF52840-DK |
|---|---|---|
| RIOT RX model | one IRQ per octet (`RXNE`), no RX DMA | UARTE EasyDMA into RAM, no per-octet IRQ |
| Hardware RX FIFO | none (`FIFOEN` undefined on F7) | n/a — DMA buffer |
| Overrun under ISR load | yes; priority helps, tail via PRIMASK | none observed (error counters 0) |
| STM32-only hooks needed | `ore_sr`/`ore_mask`, NVIC bump, console-IRQ tie | none |
| DE release after TX | drop at TC (blocking `uart_write` waits it) | append 0xFF pad; ENDTX precedes stop bit |
