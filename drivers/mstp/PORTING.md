# drivers/mstp — porting map (6LoBAC / RFC 8163 on RIOT)

**Status: skeleton.** `mstp.h` defines the types and `mstp_netdev.c` provides
stubbed `netdev_driver_t` ops. Nothing here is wired into a build yet — this is
the structural target for porting the MS/TP link layer from the proven Contiki
reference into a RIOT `netdev` + `gnrc_netif`.

## Model it on `drivers/slipmux`

`slipmux/slipdev.c` is a UART-based `netdev` that frames bytes and attaches to
`gnrc_netif` — the closest existing analog to MS/TP-over-RS485. Mirror its:

- `netdev_driver_t` op shapes (`init`, `send(iolist)`, `recv`, `isr`, `get`,
  `set`) — signatures confirmed in `drivers/include/net/netdev.h`.
- `netdev_register()` + event-callback / ISR-offload pattern (UART RX callback
  runs in IRQ; defer frame processing to the netdev `isr` in thread context).
- auto-init under `sys/net/gnrc/netif/init_devs/` (add `auto_init_mstp.c`).

## Source of truth for the protocol

Port the *logic* from `code/contiki-claude/core/net/mstp/` and the deployed
`code/github/contiki-os/contiki/platform/bdk/net/mac/mstp.c`, re-homing only the
HW/driver layer to RIOT `periph_uart` + `periph_gpio`:

| Concern | Contiki reference | Note |
|--------|-------------------|------|
| MS/TP master FSM (token, PFM, reply) | `mstp-mac.c` | drives TX/RX timing |
| COBS encode/decode (+0x55 XOR) | `mstp-cobs.c` | verified — reused as-is |
| CRC-32K (data) | `mstp-crc.c` | verified round-trip |
| Header CRC-8 | **do NOT use `mstp-crc.c`** | it uses poly 0x07 → wrong on the wire. Use the standard BACnet algorithm (see `examples/networking/lobac_bringup/mstp_crc.c`, verified against a real capture). |
| Frame Type 34 assembly | `mstp-mac.c mstp_send_frame` | reconcile the Length field (raw vs encoded) against the dissector |

The verified `mstp_crc.*`, `mstp_cobs.*`, `mstp_frame.*` in the bring-up app are
ready to migrate here once the FSM and netdev plumbing are in place.

## Build steps to finish (in the VM)

1. Add `Makefile.include` (include path) and `Kconfig`; register the module name
   `mstp` so `USEMODULE += mstp` resolves.
2. Flesh out `_init` (uart_init + DE gpio), `_send` (build+TX a Type-34 frame),
   `_recv` (COBS-decode + CRC-32K check), `_isr` (run the receive-frame FSM).
3. Add `auto_init_mstp.c` + params, attach a `gnrc_netif` (raw/6lo).
4. Bring `regression-tests/lobac/{test-crc,test-cobs,test-iphc,test-mstp-fsm}`
   over as RIOT unit tests under `tests/`.
5. Layer SCHC (RFC 8724) via `pkg/libschc` once IPv6-over-MS/TP round-trips.
