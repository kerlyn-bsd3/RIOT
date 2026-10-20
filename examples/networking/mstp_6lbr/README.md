# mstp_6lbr — 6LoBAC border router (MS/TP ⇄ SLIP)

A RIOT 6LoWPAN border router (6LBR) that bridges an **MS/TP** link (downstream,
RFC 8163) to an IPv6 uplink over **SLIP** to a Linux host running tunslip6.
Sibling of `mstp_6lo`; same board-agnostic `mstp` driver, in the 6LBR role.

## Roles & addressing (hardcoded for the test bed)

| | interface | peripheral | rate | address |
|---|---|---|---|---|
| Downstream (6LN-facing) | mstp | UARTE1 = Arduino D0/D1 | 115200 | MS/TP TS **2**; advertises `aaaa::/64` (RA PIO A=1/L=0) |
| Uplink (host-facing) | slipdev | UARTE0 = J-Link VCP | **576000** | slip LL `fe80::2`, default route via host `fe80::1` |
| Console | stdio | nRF native USB CDC-ACM | USB | 2nd `/dev/ttyACM*` |

The 6LBR does the 6LoWPAN-ND the old Contiki BDK lacked: it answers NS(ARO) with
NA(ARO) and advertises the prefix + 6CO + ABRO, so a 6LN registers natively.
Peer for first bring-up: a **BDK at MS/TP station 1 acting as the 6LN**.

## Build & flash (on the VM)

```
cd examples/networking/mstp_6lbr
make BOARD=nrf52840dk
make BOARD=nrf52840dk flash
```

Flashing uses the on-board J-Link (SWD) — independent of the VCP that SLIP will
use.

## Console

SLIP owns the J-Link VCP (`/dev/ttyACM0`), so the shell is on the **nRF native
USB** CDC-ACM — a *second* `/dev/ttyACM*` that appears once this firmware runs
(the 6LN firmware never drove the native USB, which is why that port was dark
before). Find it and open it:

```
ls /dev/serial/by-id/          # identify the native-USB CDC vs the J-Link VCP
make BOARD=nrf52840dk term PORT=/dev/ttyACMx    # the native-USB port
```

**Fallback if the native USB won't enumerate:** in the Makefile comment
`USEMODULE += stdio_cdc_acm`, uncomment `USEMODULE += stdio_rtt`, rebuild/flash,
then use the console over the same J-Link:

```
make BOARD=nrf52840dk CONSOLE=rtt JLINK_SERIAL=<SN> term
```

## Bring up SLIP on the host (the VM)

The static node pairs with RIOT's `start_network.sh` in **SLIP-only** mode (`-e`,
no uhcpd — our prefix is hardcoded). It creates tun `sl0` (`fe80::1` + route
`aaaa::/64` via `fe80::2`) and runs RIOT's `sliptty` (its tunslip6 equivalent) at
576000 on the J-Link VCP. 576000 needs the `cpu/nrf5x_common/periph/uart.c` case
added for this app — stock RIOT's nRF UART rejects it (`UART_NOBAUD`):

```
make -C $RIOTBASE/dist/tools/sliptty            # build sliptty once
sudo $RIOTBASE/dist/tools/sliptty/start_network.sh -e aaaa::/64 /dev/ttyACM0 576000
```

(`$RIOTBASE` = the RIOT tree root, e.g. `~/src/github/RIOT-OS/RIOT`.)

**Line rate.** The SLIP rate is a make variable, `SLIP_BAUDRATE` (default 576000);
the firmware and `start_network.sh` must use the same value. On the bench (2026-10-14)
**115200 works** over the J-Link OB VCP but **576000 does not** (no reply from
`fe80::2`, garbled `Unknown packet type` frames in sliptty). Until a higher rate is
verified, build and run at 115200:

```
make BOARD=nrf52840dk SLIP_BAUDRATE=115200 clean all
make BOARD=nrf52840dk JLINK_SERIAL=<SN> flash
sudo $RIOTBASE/dist/tools/sliptty/start_network.sh -e aaaa::/64 /dev/ttyACM0 115200
```

The SLIP link is only the 6LBR-to-host hop, so its rate does not have to match the
Contiki BDK's 576000 — any rate both ends agree on works.

## Bring-up checklist

1. **SLIP link:** after `start_network.sh`, `ip -6 addr show sl0` shows `fe80::1`;
   `ip -6 route` shows `aaaa::/64 via fe80::2 dev sl0`. tunslip6 prints no framing
   errors. `ping6 -c3 fe80::2%sl0` reaches the BR's slip side.
2. **6LBR side (console):** `ifconfig` shows the mstp iface with a global
   `aaaa::ff:fe00:2` and the slip iface with `fe80::2`; `nib prefix` lists
   `aaaa::/64`; `nib route` shows the default route via `fe80::1`.
3. **MS/TP ring:** `status` (toggle) shows the token circulating with the 6LN
   (TS 1) and clean RX counters.
4. **6LN registration:** the BDK-6LN (TS 1) forms `aaaa::ff:fe00:1` from our RA.
   (If the BDK doesn't send NS(ARO), it won't appear in the BR's neighbor cache —
   that part needs a RIOT 6LN; see the session log.)
5. **End to end:** from the host, `ping6 -c3 aaaa::ff:fe00:1` (the 6LN). Replies
   return via the BR's default route to `fe80::1`.

## Notes

- SLIP is on the J-Link VCP so 576000 is a real line rate (matches the BDK). On
  native USB CDC the baud is nominal, which is why the console — not SLIP — is
  what goes there.
- Prefix/addresses are compile-time (`BR_PREFIX` etc. in main.c). This mirrors
  RIOT's uhcp client (`gnrc_uhcpc.c`) but hardcoded; see main.c's header.

## Uplink over native USB (CDC-ECM) — `UPLINK=cdc-ecm`

Replaces SLIP with USB Ethernet on the nRF52840-DK's **native** USB port (the
connector marked nRF USB; a second cable). The J-Link OB and UARTE0 drop out of the
data path; the console moves to RTT over the J-Link cable (forced).

```sh
make BOARD=nrf52840dk UPLINK=cdc-ecm JLINK_SERIAL=<SN> flash
sudo ./start_ecm.sh                 # finds the RIOT cdc_ether netdev, fe80::1 + route aaaa::/64 via fe80::2
make BOARD=nrf52840dk UPLINK=cdc-ecm JLINK_SERIAL=<SN> term   # RTT console
```

The host interface is a network device (`ip link`: `usb0`/`enx…`), not a
`/dev/ttyACM*`. If NetworkManager grabs it, `nmcli dev set <iface> managed no`.
