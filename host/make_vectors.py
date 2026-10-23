#!/usr/bin/env python3
"""Link test vectors for tb/tb_link.vhd (frames built by h264link.py).

    make_vectors.py <stream.264> <frames.txt>      write the frames
    make_vectors.py --check <responses.bin>        parse responses written by the testbench

frames.txt: one frame per line as hex, first character = kind:
    S start, D data, E end, Q status request
"""
import random
import sys

import h264link as hl


def write_vectors(src, dst, seed=5):
    data = open(src, "rb").read()
    rnd = random.Random(seed)
    lines = ["S" + hl.build_frame(hl.T_START, 0).hex(), "Q" + hl.build_frame(hl.T_STATUS, 0).hex()]
    seq = 1
    i = 0
    while i < len(data):
        n = rnd.choice([1, 2, 7, 100, 513, 1000, 1024, 1024, 1024])
        lines.append("D" + hl.build_frame(hl.T_DATA, seq, data[i:i + n]).hex())
        i += n
        seq = (seq + 1) & 0xFF
    lines.append("E" + hl.build_frame(hl.T_END, seq).hex())
    open(dst, "w").write("\n".join(lines) + "\n")
    return len(lines)


def check(path):
    p = hl.ResponseParser()
    rs = p.feed(open(path, "rb").read())
    for r in rs:
        print(r.describe())
    print("%d responses, %d with a bad CRC" % (len(rs), p.bad))
    return 0 if rs and p.bad == 0 else 1


if __name__ == "__main__":
    if sys.argv[1] == "--check":
        sys.exit(check(sys.argv[2]))
    print(write_vectors(sys.argv[1], sys.argv[2]), "frames")
