# mstp_monitor — MS/TP bus monitor (raw echo over native USB)

Replaces the BDK monitor node. An **active MS/TP manager station** (default This
Station **0**) that passes the token like the BDK monitor did, and mirrors **every
octet on the bus** — other stations' frames and its own — out the nRF52840's
**native USB** as a CDC-ACM byte stream. Karg's capture tool works unchanged: point
it at the new `/dev/ttyACM*` instead of the BDK's `/dev/ttyUSB0`.

| | peripheral | notes |
|---|---|---|
| MS/TP | UARTE1 = Arduino D0/D1, DE = D2 (P1.03) | 115200, TS 0, 1 pad octet |
| Bus mirror (data) | nRF **native USB** CDC-ACM (`nRF USB` connector) | bus octets ONLY; baud ignored |
| Console / shell | J-Link VCP (UARTE0) | `make term`; `status` toggles the report |

How the mirror works: the driver's optional **tap** (`mstp_params_t::tap`) hands
over every received octet from the UART RX ISR (before the frame FSM, so bad frames
are captured as-is) and every transmitted buffer (frame + pad) just before
`uart_write()` — needed because DE/RE are tied on the DFR0259, so a node never hears
itself. The bus is half-duplex, so the order is exact. Full-speed USB is ~100× the
bus rate, so the relay can't fall behind (the BDK's FT232R relay dropped ~1 octet in
57 of back-to-back nRF frames).

Timing: as with the BDK path, timestamps are applied by the host on arrival
(ms-level). A device-timestamped record mode + Wireshark extcap is planned.

## Build / flash / run (VM)

```
cd examples/networking/mstp_monitor
make BOARD=nrf52840dk clean all
make BOARD=nrf52840dk JLINK_SERIAL=<SN> flash
make BOARD=nrf52840dk PORT=/dev/ttyACMx term        # console = the DK's J-Link VCP
ls -l /dev/serial/by-id/ | grep -i riot             # the bus-mirror port (native USB)
```

Both DK USB connectors are needed: J-Link (flash + console) and **nRF USB** (mirror).
DK switches: power ON, source **VDD**, **DEFAULT** (not nRF ONLY). Shield as the
other nodes (serial select HARDWARE, mode MANU). Take the BDK monitor off the bus
first (both would be station 0).

`status` shows the ring (state, TS/NS/PS, RX counters) and the tap: `rx`/`tx` octets
mirrored, `usb open|closed`, `drop` (USB buffer full — should stay 0), `offline`
(octets seen while no host had the port open — normal).

Build options: `-DMSTP_SRC_ADDR=n`, `-DMSTP_NMAX_MANAGER=n`, `-DMON_USB_BUF_SIZE=n`
(power of two). USB VID/PID default to RIOT's pid.codes test IDs (1209:7D01).
