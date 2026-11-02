# 6LoBAC test bed: IPv6 over BACnet MS/TP (RFC 8163)

This describes the bench used to develop and soak-test the RIOT `mstp` driver: a
Linux host, a 6LoWPAN border router (`mstp_6lbr`), a 6LoWPAN node (`mstp_6lo`)
and a bus monitor (`mstp_monitor`) on one RS-485 segment. With it you can
reproduce the overnight soak test and the example frame in
draft-ietf-6lo-rfc8163-bis Appendix D.

```
              USB: bus mirror (CDC-ACM)      USB: IPv6 uplink (CDC-ECM)
   Linux host ───────────────────────┐   ┌──────────────────────────────┐
   (fdea:dbee:f::1)                  │   │                              │
                                 monitor        6LN                   6LBR
                                  TS 0          TS 3                  TS 2
   MS/TP RS-485, 115200 bit/s:    ├═════════════╪═════════════════════┤
                                (end)        (middle)               (end)
```
Each board's J-Link USB also goes to the host, for flashing and consoles.

## 1. What you need

| Qty | Item | Notes |
|---|---|---|
| 3 | Nordic nRF52840-DK | One each: monitor, 6LBR, 6LN |
| 3 | DFRobot RS485 Shield (DFR0259) | MAX485, Arduino form factor |
| 5 | USB cables | J-Link on all three; nRF USB on the monitor and the 6LBR |
| — | Twisted pair for the trunk | A, B and common |
| 1 | Linux host | Ubuntu 25.10 (aarch64, VMware Fusion VM on a Mac) was used; any recent Linux should work |

Software, all at the versions this document was written against:

| Component | Version |
|---|---|
| RIOT | github.com/kerlyn-bsd3/RIOT, tag `6lobac-2026-11-02` (branch `6lobac-mstp`) |
| Wireshark | built from source with the MS/TP / 6LoBAC MRs (see 6) until they are released |
| Toolchain | `gcc-arm-none-eabi`, SEGGER J-Link software (`JLinkExe`) |
| Python | 3, plus `pyserial` (and `scapy` for section 8) |

## 2. Hardware setup

**DK switches** (all three): power ON, power source VDD, nRF power "DEFAULT"
(not "nRF ONLY").

**DFR0259 switches** (all three): serial select HARDWARE, transceiver mode
MANU, so the MCU drives the direction on EN = Arduino D2. The shield ties /RE
to DE, so a station never hears its own transmission.

**Pin mapping** (the same on every DK, set in each app's `main.c`):

| Signal | DK pin | Arduino |
|---|---|---|
| MS/TP RX (UARTE1) | P1.01 | D0 |
| MS/TP TX (UARTE1) | P1.02 | D1 |
| DE (driver enable, active high) | P1.03 | D2 |

**Termination.** Every DFR0259 has a 120 Ω terminator (R6, SMD) across A–B.
ANSI/ASHRAE 135 Clause 9.2 wants exactly one at each physical end of the
trunk, so remove R6 from the shield in the middle. Check with the bus idle and
all boards powered off: A–B ≈ 60 Ω.

**Order on the bus** (as soaked on 27 Oct and 1 Nov):

```
monitor (R6 fitted) ── 6LN (R6 removed) ── 6LBR (R6 fitted)
   end                    middle              end
```

Write down each DK's J-Link serial number (printed on the board, or
`ls /dev/serial/by-id/`). Every command below names a board by its SN, because
with three DKs attached, the default board is a guess.

| Role | App | MS/TP address | J-Link SN (this bed) |
|---|---|---|---|
| Monitor | `mstp_monitor` | 0 | 683839002 |
| 6LBR | `mstp_6lbr` | 2 | 683509660 |
| 6LN | `mstp_6lo` | 3 | 683631552 |

Nmax_manager is 8 on this bed (the default in both apps). Leave address 1
unused: the monitor polls it.

## 3. Ports on the host

Use the stable `/dev/serial/by-id/` names, not `/dev/ttyACMn`, which change
when boards are re-plugged.

| Port | What it is | Used by |
|---|---|---|
| `usb-SEGGER_J-Link_000<monitor SN>-if00` | monitor console (J-Link VCP) | `make term` |
| `usb-RIOT-os.org_nrf52840dk_<serial>-if00` | monitor bus mirror (nRF USB, CDC-ACM) | `mstp_capture.py` |
| `usb-SEGGER_J-Link_000<6LN SN>-if00` | 6LN console (J-Link VCP) | `make term` |
| (no tty) | 6LBR console over RTT on its J-Link | `make term CONSOLE=rtt` |
| `enx…` network interface | 6LBR uplink (nRF USB, CDC-ECM) | `start_ecm.sh` |

If the host is a VM, connect every J-Link and nRF USB device to the VM.

## 4. Build and flash

In each case: `cd` to the app, build, flash by SN. `$SN_*` are the serial
numbers above.

```sh
cd examples/networking/mstp_monitor
make BOARD=nrf52840dk JLINK_SERIAL=$SN_MON clean flash

cd ../mstp_6lbr
make BOARD=nrf52840dk UPLINK=cdc-ecm JLINK_SERIAL=$SN_6LBR clean flash

cd ../mstp_6lo
make BOARD=nrf52840dk JLINK_SERIAL=$SN_6LN clean flash
```

`UPLINK=cdc-ecm` puts the 6LBR's uplink on the nRF USB port and its console
on RTT.
`MSTP_ADDR=<n>` overrides a station's address.

## 5. Bring-up

A suggested terminal per job (eight on the reference bench):

| # | Directory | Job |
|---|---|---|
| 1 | `mstp_monitor` | build/flash, console: `make BOARD=nrf52840dk PORT=/dev/serial/by-id/usb-SEGGER_J-Link_000$SN_MON-if00 term` |
| 2 | `mstp_monitor` | `mstp_capture.py` (section 7) |
| 3 | Wireshark tree | build Wireshark (section 6) |
| 4 | `mstp_6lbr` | `sudo ./start_ecm.sh` |
| 5 | any | host commands: `ping6` (sections 5 and 7), the Scapy send (section 8) |
| 6 | `mstp_6lbr` | build/flash/J-Link commands |
| 7 | `mstp_6lbr` | console: `make BOARD=nrf52840dk UPLINK=cdc-ecm CONSOLE=rtt JLINK_SERIAL=$SN_6LBR term` |
| 8 | `mstp_6lo` | build/flash, console: `make BOARD=nrf52840dk PORT=/dev/serial/by-id/usb-SEGGER_J-Link_000$SN_6LN-if00 term` |


Bring the bed up in this order:

1. **Power and reset all three boards** (RESET button, or `JLinkExe … r, g`).
   Counters start at zero only at boot.
2. **Host uplink:** `sudo ./start_ecm.sh` (terminal 4). It finds the 6LBR's
   CDC-ECM interface, sets NetworkManager to leave it alone, and adds:
   - `fe80::1` and `fdea:dbee:f::1/64` on the host side;
   - a route to `aaaa::/64` via the 6LBR at `fe80::2`.

   Leave it running; pressing Return removes everything it added.
3. **Check each hop:**
   - `ping6 -c 3 fe80::2%enx…`: the 6LBR's uplink.
   - 6LBR console: `ifconfig` shows `aaaa::ff:fe00:2` on the MS/TP interface.
     `nib prefix` lists `aaaa::/64`.
   - 6LN console: `ifconfig` shows `aaaa::ff:fe00:3`, formed from the 6LBR's
     Router Advertisement and registered with it (NS/NA with ARO).
   - `ping6 -c 3 aaaa::ff:fe00:3`: end to end.
4. **Token ring:** `status` on each console toggles a periodic report. The
   token should circulate 0 → 2 → 3 → 0. `rx ok` should climb, and `inv` and
   every error counter should stay 0.

## 6. Wireshark

Until the MS/TP and 6LoBAC changes are in a Wireshark release, build from
source with these merge requests:
- wireshark/wireshark!27569: 135-2024 terminology;
- !27570: decode all COBS frame types, dissect Frame Type 34 as 6LoWPAN, and
  check CRC-32K and lengths;
- !27571: derive 6LoWPAN IIDs from MS/TP addresses.

```sh
git clone https://gitlab.com/wireshark/wireshark.git && cd wireshark
sudo tools/debian-setup.sh --install-optional
# merge or cherry-pick the three MR branches, then:
mkdir build && cd build && cmake -G Ninja .. && ninja
```

Set 6LoWPAN context 0 to `aaaa::/64`, the prefix the 6LBR advertises: Edit →
Preferences → Protocols → 6LoWPAN → Context 0, or `-o
6lowpan.context0:aaaa::/64` for tshark. IPHC elides the 6LN's address using
context 0, so without this setting it isn't recovered.

## 7. The soak test

The reference run: 60,000 echo requests of 1500-octet IPv6 packets, one every
0.5 s (about 8.3 h), with every octet on the bus captured.

**Before** (after section 5):
1. Reset the monitor. Then clear its UARTE1 error register and read it back
   (expect `00000000`):
   ```sh
   printf 'w4 0x40028480 0xF\nmem32 0x40028480 1\nexit\n' > /tmp/errclr.jlink
   JLinkExe -SelectEmuBySN $SN_MON -device NRF52840_XXAA -if SWD -speed 4000 \
            -autoconnect 1 -CommandFile /tmp/errclr.jlink | grep -i 40028480
   ```
   ERRORSRC bits: 0 OVERRUN, 1 PARITY, 2 FRAMING, 3 BREAK. RIOT's nRF UART
   driver never clears it, and it often reads 4 (FRAMING) right after a reset
   because the UART started mid-frame.
2. Start the capture (terminal 2). Check that it's running before the ping:
   the stats log says "capturing from", and the pcap grows.
   ```sh
   ./mstp_capture.py -w ~/soak.pcap 2> ~/soak-capture-stats.log &
   ```
   The port defaults to the only `usb-RIOT-os.org_*-if00`. With the 6LBR on
   CDC-ECM, that is the monitor's mirror.

**Run** (terminal 5):
```sh
ping6 -c 60000 -i 0.5 -s 1452 aaaa::ff:fe00:3 | tee ~/soak-ping.log
```
`-s 1452` + 8 (ICMPv6) + 40 (IPv6) = 1500, the MS/TP link MTU.

**After:**
1. The ping summary: loss and RTT.
2. `status` on all three consoles, and `ipv6drops` on the 6LBR.
3. The monitor's ERRORSRC, read-only:
   `printf 'mem32 0x40028480 1\nexit\n'` as the command file.
4. Stop the capture, then count bad frames by sender:
   ```sh
   tshark -r ~/soak.pcap -Y mstp.cobs.bad -T fields -e mstp.src | sort | uniq -c
   ```

**Reference result** (1–2 Nov 2026, tag `6lobac-2026-11-02`):

| Measure | Result |
|---|---|
| Pings | 60000/60000, 0% loss |
| RTT (ms) | min/avg/max/mdev 266.8/274.3/339.6/7.9 |
| MS/TP errors | `inv` 0 and all error counters 0 at all three stations |
| Monitor ERRORSRC | 0 |
| 6LBR `ipv6drops` | only #7 (link-local, not forwarded) and one #14 at startup |
| 6LBR forwarded | 120,006 |

Counters that are not errors:
- `txdrop` on the 6LBR: octets of its own frames received while still
  transmitting.
- `offline` on the monitor: octets seen while no host had the mirror port
  open.

The RTT is mostly MS/TP token rotation at 115200 bit/s: the reply waits for
the token, plus the Poll For Manager cycles of the stations that poll.

## 8. Reproducing the Appendix D frame

The example is one echo request with every field fixed, so it doesn't change
from run to run as `ping6`'s timestamp, identifier and sequence do. From the
host, with the capture running:

```sh
IF=$(ip -6 route show aaaa::/64 | awk '{print $5}')
ping6 -c 1 fe80::2%$IF >/dev/null
MAC=$(ip -6 neigh show fe80::2 dev $IF | awk '{print $3}')
sudo python3 -c "from scapy.all import *; sendp(Ether(src=get_if_hwaddr('$IF'), dst='$MAC')/ \
  IPv6(src='fdea:dbee:f::1', dst='aaaa::ff:fe00:3', hlim=64)/ \
  ICMPv6EchoRequest(id=0x6f49, seq=1, data=bytes(i & 0xff for i in range(767))), \
  iface='$IF')"
```

Scapy is `sudo apt install python3-scapy` on Ubuntu. It is given the
interface and both MAC addresses because its own routing table may not see
the route to `aaaa::/64`.

The 6LBR forwards it (hop limit 63) as a Frame Type 34 frame from station 2
to station 3, with Length 801. Its octets should match Appendix D exactly.
The 767 data octets and the identifier were chosen so that COBS uses every
case in Appendix B: zeros, consecutive zeros, 254-octet blocks in the middle,
a final block of exactly 254 octets in the Encoded Data, and an Encoded
CRC-32K whose last octet is a zero (CRC-32K c4 31 32 00).

To decode it as in Appendix D, set 6LoWPAN context 0 to `aaaa::/64` (the
prefix the 6LBR advertises): in Wireshark, Edit → Preferences → Protocols →
6LoWPAN → Context 0; with tshark, `-o 6lowpan.context0:aaaa::/64`. Without it,
the 6LN's address, which IPHC elides using context 0, is not recovered. Then
dump the request with:

```sh
tshark -r appd.pcap -o 6lowpan.context0:aaaa::/64 \
  -Y "mstp.src == 2 && mstp.dst == 3 && icmpv6.echo.identifier == 0x6f49" -V -x
```

To keep a capture, run `mstp_capture.py -w <file>`, or use File → Save As in
Wireshark before quitting a live capture.
