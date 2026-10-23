#!/usr/bin/env python3
"""End to end tests of the PC sender against the FPGA model (fpga_emu.py)
over a pseudo terminal (Linux / macOS):

    python3 test_link.py <stream.264> [...]
"""
import os
import sys
import time

import serial

import h264link as hl
from fpga_emu import FpgaModel, PtyServer

CASES = [
    # name, model arguments, emulated serial speed in bytes/s
    ("clean", dict(), None),
    ("byte errors + lost responses", dict(p_rx_err=2e-4, p_resp_drop=0.05, seed=7), None),
    ("small FIFO, slow decoder", dict(depth=4096, drain_rate=60000), None),
    ("2 Mbaud speed, errors", dict(p_rx_err=1e-4, seed=3), 200000),
]


def run(path, name, kw, rate):
    data = open(path, "rb").read()
    m = FpgaModel(width=176, height=144, **kw)
    srv = PtyServer(m, byte_rate=rate)
    srv.start()
    port = serial.Serial(srv.name, 2000000, timeout=0.01)
    snd = hl.Sender(port, 2000000)
    t0 = time.monotonic()
    try:
        snd.start()
        snd.send(hl.read_chunks(path, 3000))
        snd.end()
        r = snd.wait_done(timeout=20)
    finally:
        port.close()
        srv.stop()
    dt = time.monotonic() - t0
    ok = bytes(m.out) == data and m.out_last and r.done and r.width == 176 and r.height == 144
    print("%-30s %s  %d bytes %.1fs  frames ok %d bad %d  retx %d nak %d timeouts %d  pictures %d" % (
        name, "OK  " if ok else "FAIL", len(data), dt, m.frames_ok, m.frames_bad, snd.stats["retx"],
        snd.stats["nak"], snd.stats["timeout"], r.pictures))
    return ok


def main():
    fails = 0
    for path in sys.argv[1:]:
        print(os.path.basename(path))
        for name, kw, rate in CASES:
            if not run(path, name, kw, rate):
                fails += 1
    print("FAIL %d" % fails if fails else "all passed")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
