#!/usr/bin/env python3
"""Software model of the FPGA side of the stream link (rtl/h264_link.vhd).

Used to test the PC programs without the kit: it behaves like the RTL
(tentative FIFO writes, ACK / NAK / STATUS, credit, END -> last byte) and
can inject errors on both directions.

    fpga_emu.py --pty                 serve on a pseudo terminal, prints its name
    fpga_emu.py --stdio --out f.264   link bytes on stdin, responses on stdout
                                      (used by the kit/sim firmware build)
    python3 -c "import fpga_emu"      (used by test_link.py)
"""
import argparse
import os
import random
import select
import struct
import sys
import threading
import time

from h264link import (MAXLEN, R_ACK, R_NAK, R_STATUS, T_DATA, T_END, T_START, T_STATUS,
                      build_response, crc16)


class FpgaModel:
    def __init__(self, depth=32768, drain_rate=None, p_rx_err=0.0, p_resp_drop=0.0,
                 width=0, height=0, seed=1):
        self.depth = depth
        self.drain_rate = drain_rate          # bytes per second taken by the "decoder"
        self.p_rx_err = p_rx_err
        self.p_resp_drop = p_resp_drop
        self.width = width
        self.height = height
        self.rnd = random.Random(seed)
        self.reset_link()
        self.state = "S0"
        self.tx = bytearray()
        self.frames_ok = 0
        self.frames_bad = 0
        self.starts = 0

    def reset_link(self):
        self.fifo = bytearray()       # committed, not yet consumed
        self.exp = 0
        self.nak_sent = False
        self.end_flag = False
        self.out = bytearray()        # bytes delivered to the decoder
        self.out_last = False         # TLAST delivered
        self.pictures = 0
        self.decoded_upto = 0

    def free(self):
        return self.depth - 1 - len(self.fifo)

    # -- decoder side ------------------------------------------------------
    def drain(self, dt):
        if self.drain_rate is None:
            n = len(self.fifo)
        else:
            n = int(self.drain_rate * dt)
        hold = 0 if self.end_flag else 1
        n = max(0, min(n, len(self.fifo) - hold))
        if n:
            self.out += self.fifo[:n]
            del self.fifo[:n]
        if self.end_flag and not self.fifo and not self.out_last and self.out:
            self.out_last = True
        # pictures: slice NAL units with first_mb_in_slice = 0
        o = self.out
        i = self.decoded_upto
        while True:
            j = o.find(b"\x00\x00\x01", i)
            if j < 0 or j + 4 >= len(o):
                break
            if o[j + 3] & 0x1F in (1, 5) and o[j + 4] & 0x80:
                self.pictures += 1
            i = j + 3
        self.decoded_upto = max(self.decoded_upto, len(o) - 4)

    # -- link side -----------------------------------------------------------
    def respond(self, rtype, seq):
        flags = (0x02 if self.out_last else 0) | (0x04 if self.end_flag else 0) | \
                (0x08 if not self.fifo else 0)
        r = build_response(rtype, seq, self.free(), self.pictures, flags, self.width, self.height)
        if self.rnd.random() < self.p_resp_drop:
            return
        self.tx += r

    def rx(self, data):
        i, n = 0, len(data)
        while i < n:
            if self.state == "PAY" and not self.p_rx_err:     # payload in one piece
                k = min(self.ln - len(self.pay), n - i)
                self.pay += data[i:i + k]
                i += k
                if len(self.pay) == self.ln:
                    self.state = "C0"
                continue
            b = data[i]
            i += 1
            if self.p_rx_err and self.rnd.random() < self.p_rx_err:
                b ^= 1 << self.rnd.randrange(8)
            self._byte(b)

    def _byte(self, b):
        st = self.state
        if st == "S0":
            if b == 0x55:
                self.state = "S1"
        elif st == "S1":
            if b == 0xAA:
                self.state = "TYPE"
                self.hdr = bytearray()
            elif b != 0x55:
                self.state = "S0"
        elif st in ("TYPE", "SEQ", "L0"):
            self.hdr.append(b)
            self.state = {"TYPE": "SEQ", "SEQ": "L0", "L0": "L1"}[st]
        elif st == "L1":
            self.hdr.append(b)
            ftype, seq, ln = self.hdr[0], self.hdr[1], self.hdr[2] | (b << 8)
            if ln > MAXLEN:
                self.state = "S0"
                return
            self.ftype, self.seq, self.ln = ftype, seq, ln
            self.accept = ftype == T_DATA and seq == self.exp and not self.end_flag and ln <= self.free()
            self.pay = bytearray()
            self.state = "PAY" if ln else "C0"
        elif st == "PAY":
            self.pay.append(b)
            if len(self.pay) == self.ln:
                self.state = "C0"
        elif st == "C0":
            self.c0 = b
            self.state = "C1"
        elif st == "C1":
            self.state = "S0"
            ok = (self.c0 | (b << 8)) == crc16(bytes(self.hdr) + bytes(self.pay))
            d = (self.exp - self.seq) & 0xFF
            older = 1 <= d <= 128
            if not ok:
                self.frames_bad += 1
                if not self.nak_sent:
                    self.respond(R_NAK, self.exp)
                    self.nak_sent = True
                return
            self.frames_ok += 1
            t = self.ftype
            if t in (T_DATA, T_END):
                if self.seq == self.exp and (t == T_END or self.accept):
                    if t == T_DATA:
                        self.fifo += self.pay
                    else:
                        self.end_flag = True
                    self.exp = (self.exp + 1) & 0xFF
                    self.nak_sent = False
                    self.respond(R_ACK, self.seq)
                elif older:
                    self.respond(R_ACK, (self.exp - 1) & 0xFF)
                elif self.seq == self.exp:
                    self.respond(R_NAK, self.exp)
                elif not self.nak_sent:
                    self.respond(R_NAK, self.exp)
                    self.nak_sent = True
            elif t == T_START:
                self.reset_link()
                self.exp = (self.seq + 1) & 0xFF
                self.starts += 1
                self.respond(R_ACK, self.seq)
            elif t == T_STATUS:
                self.respond(R_STATUS, (self.exp - 1) & 0xFF)


class FdServer(threading.Thread):
    """runs a FpgaModel on a pair of file descriptors"""

    def __init__(self, model, rfd, wfd, byte_rate=None):
        super().__init__(daemon=True)
        self.model = model
        self.rfd, self.wfd = rfd, wfd
        self.byte_rate = byte_rate            # emulated serial speed (bytes/s) or None
        self.running = True

    def run(self):
        last = time.monotonic()
        budget = 0.0
        while self.running:
            r, _, _ = select.select([self.rfd], [], [], 0.002)
            now = time.monotonic()
            dt = now - last
            last = now
            if r:
                try:
                    data = os.read(self.rfd, 4096)
                except OSError:
                    break
                if not data:
                    break
                if self.byte_rate:
                    time.sleep(len(data) / self.byte_rate)
                self.model.rx(data)
            self.model.drain(dt)
            if self.model.tx:
                if self.byte_rate:
                    budget += dt * self.byte_rate
                    n = int(min(len(self.model.tx), max(budget, 16)))
                    budget = max(0.0, budget - n)
                else:
                    n = len(self.model.tx)
                try:
                    os.write(self.wfd, bytes(self.model.tx[:n]))
                except OSError:
                    break
                del self.model.tx[:n]
        self.running = False

    def stop(self):
        self.running = False
        self.join(1)


class PtyServer(FdServer):
    """runs a FpgaModel on the master side of a pseudo terminal"""

    def __init__(self, model, byte_rate=None):
        import tty
        master, slave = os.openpty()
        tty.setraw(slave)
        super().__init__(model, master, master, byte_rate)
        self.master = master
        self.name = os.ttyname(slave)
        self._slave = slave


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pty", action="store_true", help="serve on a new pseudo terminal")
    ap.add_argument("--depth", type=int, default=32768)
    ap.add_argument("--drain", type=float, default=None, help="decoder speed in bytes/s")
    ap.add_argument("--rx-err", type=float, default=0.0, help="probability of a corrupted byte")
    ap.add_argument("--stdio", action="store_true", help="link bytes on stdin, responses on stdout")
    ap.add_argument("--size", default="1280x720", help="resolution reported in the responses")
    ap.add_argument("--out", help="write the received stream to this file at the end")
    a = ap.parse_args()
    w, h = (int(v) for v in a.size.split("x"))
    m = FpgaModel(depth=a.depth, drain_rate=a.drain, p_rx_err=a.rx_err, width=w, height=h)
    if a.stdio:
        srv = FdServer(m, sys.stdin.fileno(), sys.stdout.fileno())
        log = sys.stderr
    else:
        srv = PtyServer(m)
        log = sys.stdout
        print("FPGA model on", srv.name, "(Ctrl-C to stop)")
        sys.stdout.flush()
    srv.start()
    try:
        while srv.is_alive():
            srv.join(0.5)
    except KeyboardInterrupt:
        pass
    srv.stop()
    m.drain(1e9)
    print("frames ok %d, bad %d, bytes delivered %d, END %s" % (m.frames_ok, m.frames_bad, len(m.out), m.out_last),
          file=log)
    if a.out:
        open(a.out, "wb").write(m.out)


if __name__ == "__main__":
    main()
