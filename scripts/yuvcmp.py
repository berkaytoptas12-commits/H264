#!/usr/bin/env python3
"""Compare two YUV 4:2:0 files frame by frame.

    yuvcmp.py a.yuv b.yuv width height

Exit status 0 when identical. On mismatch prints the first differing frame,
plane and pixel position.
"""
import sys


def main():
    a_path, b_path, w, h = sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4])
    a = open(a_path, "rb").read()
    b = open(b_path, "rb").read()
    cw, ch = (w + 1) // 2, (h + 1) // 2
    fsize = w * h + 2 * cw * ch
    planes = [("Y", w, h), ("U", cw, ch), ("V", cw, ch)]
    na, nb = len(a) // fsize, len(b) // fsize
    if a == b:
        print(f"OK {na} frames identical")
        return 0
    n = min(na, nb)
    for f in range(n):
        off = f * fsize
        for name, pw, ph in planes:
            for y in range(ph):
                ra = a[off + y * pw: off + (y + 1) * pw]
                rb = b[off + y * pw: off + (y + 1) * pw]
                if ra != rb:
                    x = next(i for i in range(pw) if ra[i] != rb[i])
                    print(f"MISMATCH frame {f} plane {name} x {x} y {y} "
                          f"(mb {x // (16 if name == 'Y' else 8)},{y // (16 if name == 'Y' else 8)}) "
                          f"{ra[x]} != {rb[x]}")
                    return 1
            off += pw * ph
    print(f"MISMATCH frame count {na} != {nb}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
