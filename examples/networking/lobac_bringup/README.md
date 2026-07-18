# lobac_bringup — 6LoBAC (RFC 8163) MS/TP Frame Type 34 bring-up

First on-wire milestone for the RIOT 6LoBAC port. Periodically emits a BACnet
MS/TP **IPv6-encapsulation frame (Frame Type 34, 0x22)** on the Arduino serial
port (USART6) of a **Nucleo-F767ZI**, with manual RS-485 direction control, and
prints each frame it sends to the ST-Link console.

No IPv6 stack and no MS/TP token FSM yet — the goal is to prove **UART + DE
timing + framing** produce a frame that `mstpcap` and the 6LoBAC Wireshark
dissector accept, and to validate the capture pipeline before building the real
`netdev` driver (see `../../../drivers/mstp/PORTING.md`).

## Hardware

ST Nucleo-F767ZI + DFRobot RS485 Shield v1.0 (DFR0259). Shield jumpers (required):

- Serial-select switch → **HARDWARE** serial (Arduino D0/D1).
- Transceiver mode → **MANU** (manual direction).
- EN (direction) = Arduino **D2** = **PF15**, HIGH = transmit.

USART6 = `UART_DEV(1)`: D0 = PG9 (RX), D1 = PG14 (TX). Console = USART3 (ST-Link VCP).

## Build / flash / run (in the Ubuntu VM)

```bash
cd examples/networking/lobac_bringup
make BOARD=nucleo-f767zi
make BOARD=nucleo-f767zi flash
make BOARD=nucleo-f767zi term      # watch the frames being emitted
```

Keep build artifacts off a shared mount with `BINDIRBASE=$HOME/builds/riot`
(see `github/RIOT-OS/WORKFLOW.md`).

## Configuration (override via `CFLAGS`)

| Macro | Default | Meaning |
|-------|---------|---------|
| `MSTP_BAUD` | `115200` | Bus baud. The BDK test bed runs at 115.2 kbit/s — also the rate RFC 8163 requires every node to support. |
| `MSTP_DE_PIN` | `GPIO_PIN(PORT_F,15)` | DFR0259 EN (D2), HIGH=TX. |
| `MSTP_SRC_ADDR` / `MSTP_DST_ADDR` | `0x03` / `0x01` | MS/TP addresses. |
| `MSTP_TX_PERIOD_MS` | `2000` | Emit interval. |

e.g. `make BOARD=nucleo-f767zi CFLAGS='-DMSTP_BAUD=115200 -DMSTP_DST_ADDR=0x02'`

## What to check on the wire

Expected frame shape (Saleae / mstpcap):

```
55 FF | 22 Dst Src LenHi LenLo HdrCRC | COBS(MSDU) | COBS(CRC32K)
```

- Preamble `55 FF`, Frame Type `22`, header CRC valid (dissector "good").
- Encoded region contains no `0x00` or `0x55` octets.
- On a scope, **DE (PF15) idles low and goes high only during the frame** — if
  it's inverted or mistimed the frame is truncated (the #1 bring-up hazard).

## Verified vs. open

Host-verified (see session log Session 3): header CRC-8 reproduces the real
capture (`22 01 02 00 6B → 0x67`, residue `0x55`); COBS and CRC-32K round-trip;
the assembled frame is well-formed.

Open items to confirm on hardware / against the dissector:

1. **Length field semantics** — this builder uses the on-wire *encoded* length;
   the contiki-claude cleanroom uses the *raw* MSDU length. Flip with
   `-DMSTP_LENGTH_RAW`. Reconcile against the dissector + deployed
   `platform/bdk/net/mac/mstp.c`.
2. **DE turn-off timing** — currently a computed delay after `uart_write()`;
   replace with a TX-complete-driven release in the real driver.
3. **Header CRC** — use the algorithm here (standard BACnet), **not** the
   contiki-claude `mstp-crc.c` poly-0x07 version, which is wrong for MS/TP.
