#!/usr/bin/env python3
"""
mstp_capture.py — BACnet MS/TP capture for the RIOT nRF52840 bus monitor
(examples/networking/mstp_monitor), replacing bacnet-stack's mstpcap.

The monitor mirrors every octet on the bus (received, and its own transmitted)
out its native-USB CDC-ACM port. This tool frames that byte stream into MS/TP
frames (135-2024 Clause 9.3), checks the header CRC-8 and the data CRC
(CRC-16 for plain frames, CRC-32K for COBS-encoded Types 32..127, e.g. 34 =
IPv6), and writes each frame — preamble included — as a pcap record with
LINKTYPE_BACNET_MS_TP (165), snaplen 65535, so 1500-octet IPv6 frames
(Length up to 2043) are never truncated.

Frames with a bad header CRC are not written (they are line noise or a lost
sync); frames with a bad data CRC ARE written (Wireshark flags them) and counted.
A COBS frame cut short (an X'55' — impossible inside COBS data — appears before
its end) is counted as truncated with the exact number of missing octets,
written as received (shorter than its Length field; Wireshark flags it and
decodes what is there), and the framer resyncs there. Every bad frame gets one stderr line
(time, kind, details), even with -q; 0xFF octets between frames are counted
as pad (the transmitters' trailing pad), anything else as noise.
Timestamps are host receive times of the frame's first USB read — fine for
ordering and coarse timing; use the Saleae for sub-ms timing.

Usage
  pipe:    ./mstp_capture.py [-p PORT] | wireshark -k -i -
  file:    ./mstp_capture.py [-p PORT] -w capture.pcap
  extcap:  ln -s "$PWD/mstp_capture.py" <Wireshark personal extcap folder>/
           (Help > About Wireshark > Folders > "Personal Extcap path"), then
           restart Wireshark and pick the "BACnet MS/TP (RIOT monitor)" interface.
PORT may also be a file of raw bus octets (replay; exits at EOF).
PORT defaults to the single /dev/serial/by-id/usb-RIOT-os.org_*-if00 device.
The tool survives monitor re-plugs (it reopens the port) and exits quietly
when the reader (Wireshark) goes away. Stats go to stderr every 10 s (-q: off)
and on exit.

SPDX-License-Identifier: BSD-3-Clause
"""

import argparse
import errno
import glob
import os
import select
import signal
import struct
import sys
import termios
import time
import tty

LINKTYPE_BACNET_MS_TP = 165
SNAPLEN = 65535
NMAX_LENGTH = 2043                 # Nmax_COBS_length (135-2024 9.3)
COBS_TYPES = range(32, 128)        # Nmin_COBS_type .. Nmax_COBS_type
HEADER_CRC_RESIDUE = 0x55
DATA_CRC16_RESIDUE = 0xF0B8
CRC32K_RESIDUE = 0x0843323B
CRC32K_POLY = 0xEB31D82E
COBS_MASK = 0x55
PORT_GLOB = "/dev/serial/by-id/usb-RIOT-os.org_*-if00"
EXTCAP_IFACE = "mstp"

# ---------------------------------------------------------------- CRCs


def _crc8_step(data, crc):
    """135-2024 9.6.1 header CRC (same as drivers/mstp calc_header_crc)."""
    c = (crc ^ data) & 0xFF
    c = c ^ (c << 1) ^ (c << 2) ^ (c << 3) ^ (c << 4) ^ (c << 5) ^ (c << 6) ^ (c << 7)
    return ((c & 0xFE) ^ ((c >> 8) & 1)) & 0xFF


def header_ok(hdr6):
    crc = 0xFF
    for b in hdr6:
        crc = _crc8_step(b, crc)
    return crc == HEADER_CRC_RESIDUE


def _make_table(poly, width_mask):
    t = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = (c >> 1) ^ poly if c & 1 else c >> 1
        t.append(c & width_mask)
    return t


_T16 = _make_table(0x8408, 0xFFFF)          # 135 data CRC-16 (CCITT, reflected)
_T32K = _make_table(CRC32K_POLY, 0xFFFFFFFF)


def crc16(buf, crc=0xFFFF):
    for b in buf:
        crc = (crc >> 8) ^ _T16[(crc ^ b) & 0xFF]
    return crc


def crc32k(buf, crc=0xFFFFFFFF):
    for b in buf:
        crc = (crc >> 8) ^ _T32K[(crc ^ b) & 0xFF]
    return crc


def cobs_decode(enc, mask=COBS_MASK):
    """draft-lynn-6lo-rfc8163-bis COBS decode; None on a bad encoding."""
    out = bytearray()
    i, n = 0, len(enc)
    while i < n:
        code = enc[i] ^ mask
        if code == 0 or i + code > n:
            return None
        i += 1
        for _ in range(code - 1):
            out.append(enc[i] ^ mask)
            i += 1
        if code != 255 and i < n:
            out.append(0)
    return bytes(out)


def data_ok(ftype, data):
    """data = Length + 2 octets following the header."""
    if ftype in COBS_TYPES:
        enc, enc_crc = data[:-5], data[-5:]
        crc_bytes = cobs_decode(enc_crc)
        if crc_bytes is None or len(crc_bytes) != 4:
            return False
        return crc32k(crc_bytes, crc32k(enc)) == CRC32K_RESIDUE
    return crc16(data) == DATA_CRC16_RESIDUE

# ---------------------------------------------------------------- framer


class Framer:
    """Byte-stream -> MS/TP frames. Hunts 55 FF, takes the 6-octet header
    (type, dst, src, len_hi, len_lo, hcrc), then Length + 2 data octets.

    COBS-encoded frames (Types 32..127) never contain X'55' after the preamble
    (COBS removes zeros; the X'55' mask then removes X'55'), so an X'55' inside
    a COBS frame's data/CRC region means the frame was cut short: octets were
    lost between the bus and here. The framer then reports exactly how many
    octets are missing, passes on the octets it did receive (without the
    trailing 0xFF pad) as a short frame, and resyncs at that X'55' instead of
    swallowing the following frames (which would otherwise show up as
    header-CRC errors).

    @p report(kind, ts, detail) is called for every bad frame (diagnostics)."""

    def __init__(self, report=None):
        self.buf = bytearray()
        self.t_first = None
        self.report = report or (lambda *a: None)
        self.missing_sizes = {}
        self.stats = dict(frames=0, data34=0, max_len=0, hdr_bad=0, data_bad=0,
                          truncated=0, pad=0, noise=0)

    def _skip(self, n):
        """Discard n octets at the front: 0xFF counts as pad, anything else as noise."""
        pad = self.buf.count(0xFF, 0, n)
        self.stats["pad"] += pad
        self.stats["noise"] += n - pad
        del self.buf[:n]

    def feed(self, chunk, now):
        if not self.buf:
            self.t_first = now
        self.buf += chunk
        out = []
        b = self.buf
        while True:
            # hunt for the preamble
            i = b.find(b"\x55\xff")
            if i < 0:
                keep = 1 if b[-1:] == b"\x55" else 0
                self._skip(len(b) - keep)
                break
            if i:
                self._skip(i)
            if len(b) < 8:
                break
            hdr = bytes(b[2:8])
            length = (hdr[3] << 8) | hdr[4]
            if not header_ok(hdr) or length > NMAX_LENGTH:
                self.stats["hdr_bad"] += 1
                self.report("HDR_CRC", self.t_first, "header %s" % hdr.hex(" "))
                del b[:1]                       # resync just past this 0x55
                self.stats["noise"] += 1
                continue
            total = 8 + (length + 2 if length else 0)
            if length and hdr[0] in COBS_TYPES:
                j = b.find(0x55, 8, min(len(b), total))
                if (j < 0 and len(b) > total and b[total] == 0x55 and b[total - 1] == 0xFF
                        and not data_ok(hdr[0], bytes(b[8:total]))):
                    j = total                   # 1 octet lost: the frame swallowed its 0xFF pad
                if j >= 0:                      # cut short: next frame began early
                    end = j
                    if b[j - 1] == 0xFF and j - 1 >= 8:
                        end = j - 1             # the transmitter's trailing pad, not data
                    got = end - 8
                    missing = (length + 2) - got
                    self.stats["truncated"] += 1
                    self.missing_sizes[missing] = self.missing_sizes.get(missing, 0) + 1
                    self.report("TRUNCATED", self.t_first,
                                "type %d %d->%d Length %d: got %d of %d data octets, "
                                "MISSING %d; last octets %s"
                                % (hdr[0], hdr[2], hdr[1], length, got, length + 2,
                                   missing, bytes(b[max(8, end - 8):end]).hex(" ")))
                    if end < j:
                        self.stats["pad"] += 1
                    self.stats["frames"] += 1
                    if hdr[0] == 34:
                        self.stats["data34"] += 1
                    out.append((self.t_first, bytes(b[:end]), False))
                    self.t_first = now
                    del b[:j]                   # resync at the X'55'
                    continue
            if len(b) < total:
                break
            frame = bytes(b[:total])
            del b[:total]
            ok = True
            if length:
                ok = data_ok(hdr[0], frame[8:])
                if not ok:
                    self.stats["data_bad"] += 1
                    self.report("DATA_CRC", self.t_first,
                                "type %d %d->%d Length %d (all octets present, value(s) "
                                "corrupted); first %s ... last %s"
                                % (hdr[0], hdr[2], hdr[1], length,
                                   frame[8:24].hex(" "), frame[-16:].hex(" ")))
            self.stats["frames"] += 1
            if hdr[0] == 34:
                self.stats["data34"] += 1
            self.stats["max_len"] = max(self.stats["max_len"], length)
            out.append((self.t_first, frame, ok))
            self.t_first = now                  # next frame's bytes came in this read or later
        if not b:
            self.t_first = None
        return out

# ---------------------------------------------------------------- pcap


def pcap_global_header():
    return struct.pack("<IHHiIII", 0xA1B2C3D4, 2, 4, 0, 0, SNAPLEN,
                       LINKTYPE_BACNET_MS_TP)


def pcap_record(ts, frame):
    sec = int(ts)
    usec = int((ts - sec) * 1e6)
    return struct.pack("<IIII", sec, usec, len(frame), len(frame)) + frame

# ---------------------------------------------------------------- serial


def find_port():
    ports = sorted(glob.glob(PORT_GLOB))
    if len(ports) == 1:
        return ports[0]
    if not ports:
        return None
    raise SystemExit("several RIOT CDC-ACM ports, pick one with -p:\n  " +
                     "\n  ".join(ports))


def _is_char_dev(path):
    import stat
    return stat.S_ISCHR(os.stat(path).st_mode)


def open_port(path):
    fd = os.open(path, (os.O_RDWR if _is_char_dev(path) else os.O_RDONLY) |
                 os.O_NOCTTY | os.O_NONBLOCK)
    if not os.isatty(fd):
        return fd                     # a raw byte-stream file (replay): no termios
    tty.setraw(fd)
    attrs = termios.tcgetattr(fd)
    attrs[2] |= termios.CLOCAL | termios.CREAD
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIFLUSH)
    return fd

# ---------------------------------------------------------------- main loop


def log(msg):
    sys.stderr.write("mstp_capture: %s\n" % msg)
    sys.stderr.flush()


def fmt_stats(s):
    return ("frames=%(frames)d type34=%(data34)d max_len=%(max_len)d "
            "hdr_crc_bad=%(hdr_bad)d data_crc_bad=%(data_bad)d "
            "truncated=%(truncated)d pad_octets=%(pad)d noise_octets=%(noise)d" % s)


def report_bad(kind, ts, detail):
    """One stderr line per bad frame: local time (ms), kind, details."""
    t = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(ts)) + ".%03d" % (int(ts * 1000) % 1000)
    log("%s %-9s %s" % (t, kind, detail))


def capture(port_arg, out, quiet):
    out.write(pcap_global_header())
    out.flush()
    framer = Framer(report_bad)
    fd = None
    port = None
    last_stats = time.time()
    try:
        while True:
            if fd is None:
                port = port_arg if port_arg not in (None, "", "auto") else find_port()
                if port is None or not os.path.exists(port):
                    time.sleep(0.5)
                    continue
                try:
                    fd = open_port(port)
                    framer.buf.clear()
                    log("capturing from %s" % port)
                except OSError as e:
                    log("open %s: %s (retrying)" % (port, e.strerror))
                    time.sleep(1)
                    continue
            r, _, _ = select.select([fd], [], [], 1.0)
            now = time.time()
            if r:
                try:
                    chunk = os.read(fd, 65536)
                except OSError as e:
                    if e.errno in (errno.EAGAIN, errno.EINTR):
                        continue
                    chunk = b""
                if not chunk and not os.isatty(fd):  # replayed file: done
                    break
                if not chunk:                       # unplugged
                    log("%s went away; waiting for it to return" % port)
                    os.close(fd)
                    fd = None
                    continue
                for ts, frame, _ok in framer.feed(chunk, now):
                    out.write(pcap_record(ts, frame))
                out.flush()
            if not quiet and now - last_stats >= 10:
                log(fmt_stats(framer.stats))
                last_stats = now
    except BrokenPipeError:
        pass                                        # Wireshark stopped reading
    except KeyboardInterrupt:
        pass
    finally:
        log("done: " + fmt_stats(framer.stats))
        if framer.missing_sizes:
            log("truncations by octets missing: " + ", ".join(
                "%d:%d" % kv for kv in sorted(framer.missing_sizes.items())))
        if fd is not None:
            os.close(fd)


def extcap_queries(args):
    if args.extcap_interfaces:
        print("extcap {version=1.0}{help=examples/networking/mstp_monitor}")
        print("interface {value=%s}{display=BACnet MS/TP (RIOT monitor)}" % EXTCAP_IFACE)
        return True
    if args.extcap_dlts:
        print("dlt {number=%d}{name=BACNET_MS_TP}{display=BACnet MS/TP}"
              % LINKTYPE_BACNET_MS_TP)
        return True
    if args.extcap_config:
        print("arg {number=0}{call=--port}{display=Monitor serial port}"
              "{type=string}{default=auto}"
              "{tooltip=auto = the single %s}" % PORT_GLOB)
        return True
    return False


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("-p", "--port", default="auto")
    ap.add_argument("-w", "--write", default="-", help="pcap output (- = stdout)")
    ap.add_argument("-q", "--quiet", action="store_true")
    # extcap interface (Wireshark calls these)
    ap.add_argument("--extcap-interfaces", action="store_true")
    ap.add_argument("--extcap-dlts", action="store_true")
    ap.add_argument("--extcap-config", action="store_true")
    ap.add_argument("--extcap-interface")
    ap.add_argument("--extcap-version")
    ap.add_argument("--capture", action="store_true")
    ap.add_argument("--fifo")
    args, _unknown = ap.parse_known_args()
    # Wireshark stops an extcap with SIGTERM: unwind so the final stats print
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))

    if extcap_queries(args):
        return
    if args.capture and args.fifo:
        with open(args.fifo, "wb", buffering=0) as out:
            capture(args.port, out, quiet=True)
        return
    if args.write == "-":
        if sys.stdout.isatty():
            raise SystemExit("refusing to write pcap to a terminal; pipe it "
                             "(| wireshark -k -i -) or use -w FILE")
        capture(args.port, sys.stdout.buffer, args.quiet)
    else:
        with open(args.write, "wb") as out:
            capture(args.port, out, args.quiet)


if __name__ == "__main__":
    main()
