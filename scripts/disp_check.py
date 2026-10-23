#!/usr/bin/env python3
"""Check the frames captured by tb/tb_disp.vhd (decoded TMDS, RGB24) against
the reference decoder output.

    disp_check.py captured.rgb ref.yuv src_w src_h disp_w disp_h [--runs N]

Every captured frame must be either the 75 % colour bars (only before the
first picture) or the expected display image of a decoded frame; the frame
numbers must go up one at a time (repeats allowed), and the last decoded
frame must be reached.  The expected image follows h264_disp: the picture is
centred (x start rounded down to 16) or centre-cropped, chroma is repeated
(nearest), YCbCr -> RGB with 10 bit fixed point coefficients, BT.709 for
pictures of 720 lines or more, BT.601 below.
"""
import argparse
import sys

import numpy as np

COEF = {False: (1634, 401, 833, 2065), True: (1836, 218, 546, 2163)}   # kr, kgb, kgr, kb


def read_yuv(path, w, h):
    data = np.fromfile(path, np.uint8)
    fs = w * h * 3 // 2
    n = len(data) // fs
    frames = []
    for i in range(n):
        f = data[i * fs:(i + 1) * fs]
        y = f[:w * h].reshape(h, w)
        u = f[w * h:w * h + w * h // 4].reshape(h // 2, w // 2)
        v = f[w * h + w * h // 4:].reshape(h // 2, w // 2)
        frames.append((y, u, v))
    return frames


def placement(n, disp, align, cut):
    """(start on display, first source sample, visible length)"""
    if n <= disp:
        return ((disp - n) // (2 * align)) * align, 0, n
    return 0, ((n - disp) // (2 * cut)) * cut, disp


def expected(frame, w, h, W, H):
    y, u, v = frame
    out = np.zeros((H, W, 3), np.uint8)
    xs, xsrc, vw = placement(w, W, 16, 16)
    ys, ysrc, vh = placement(h, H, 1, 2)
    kr, kgb, kgr, kb = COEF[h >= 720]
    sy = ysrc + np.arange(vh)
    sx = xsrc + np.arange(vw)
    Y = y[np.ix_(sy, sx)].astype(np.int64) - 16
    U = u[np.ix_(sy >> 1, sx >> 1)].astype(np.int64) - 128
    V = v[np.ix_(sy >> 1, sx >> 1)].astype(np.int64) - 128
    t = 1192 * Y
    rgb = [t + kr * V, t - kgb * U - kgr * V, t + kb * U]
    for c in range(3):
        out[ys:ys + vh, xs:xs + vw, c] = np.clip((rgb[c] + 512) >> 10, 0, 255)
    return out


def bars(W, H):
    out = np.zeros((H, W, 3), np.uint8)
    for x in range(W):
        b = min(x // (W // 8), 7)
        out[:, x] = (191 * (b in (0, 1, 4, 5)), 191 * (b in (0, 1, 2, 3)), 191 * (b in (0, 2, 4, 6)))
    return out


def torn(f, top, bottom):
    """frame made of top's first rows and bottom's remaining rows"""
    same_t = (f == top).all(axis=(1, 2))
    same_b = (f == bottom).all(axis=(1, 2))
    for r in range(1, f.shape[0]):
        if same_t[:r].all() and same_b[r:].all():
            return True
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("captured")
    ap.add_argument("ref")
    ap.add_argument("w", type=int)
    ap.add_argument("h", type=int)
    ap.add_argument("W", type=int)
    ap.add_argument("H", type=int)
    ap.add_argument("--runs", type=int, help="every frame but the first and last shown this many times")
    ap.add_argument("--restart", action="store_true",
                    help="the stream was sent twice with a decoder reset between: two passes expected")
    a = ap.parse_args()

    ref = read_yuv(a.ref, a.w, a.h)
    cap = np.fromfile(a.captured, np.uint8)
    fs = a.W * a.H * 3
    caps = [cap[i * fs:(i + 1) * fs].reshape(a.H, a.W, 3) for i in range(len(cap) // fs)]
    bar = bars(a.W, a.H)
    exp_cache = {}

    def exp(k):
        if k not in exp_cache:
            exp_cache[k] = expected(ref[k], a.w, a.h, a.W, a.H)
        return exp_cache[k]

    cur, nbars, runs, ok = -1, 0, [], True
    passes = 1
    for i, f in enumerate(caps):
        if (cur < 0 or (a.restart and passes == 1 and cur == len(ref) - 1)) and np.array_equal(f, bar):
            nbars += 1
            continue
        if a.restart and passes == 1 and cur == len(ref) - 1 and torn(f, exp(cur), bar):
            nbars += 1                      # cut to colour bars by the decoder reset mid-frame
            continue
        if a.restart and passes == 1 and cur == len(ref) - 1 and np.array_equal(f, exp(0)) \
                and not np.array_equal(f, exp(cur)):
            passes, cur = 2, 0              # second pass after the decoder reset
            runs.append(1)
            continue
        if cur >= 0 and np.array_equal(f, exp(cur)):
            runs[-1] += 1
            continue
        if cur + 1 < len(ref) and np.array_equal(f, exp(cur + 1)):
            cur += 1
            runs.append(1)
            continue
        k = min(cur + 1, len(ref) - 1)
        d = np.argwhere(f != exp(k))
        yy, xx, cc = d[0]
        print("captured frame %d: no match (vs decoded frame %d: %d bytes differ, first at x=%d y=%d c=%d: %d != %d)"
              % (i, k, len(d), xx, yy, cc, f[yy, xx, cc], exp(k)[yy, xx, cc]))
        ok = False
        break
    if ok and a.restart and passes != 2:
        print("second pass after the decoder reset not seen")
        ok = False
    if ok and cur != len(ref) - 1:
        print("last decoded frame not shown (reached %d of %d)" % (cur + 1, len(ref)))
        ok = False
    if ok and a.runs and any(r != a.runs for r in runs[1:-1]):
        print("frame repeat counts %s, expected %d" % (runs, a.runs))
        ok = False
    print("%s: %d captured frames, %d colour bar frames, decoded frames 0..%d shown%s, repeats %s"
          % ("OK" if ok else "FAIL", len(caps), nbars, cur, " (2 passes)" if passes == 2 else "",
             runs if len(runs) <= 20 else runs[:20] + ["..."]))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
