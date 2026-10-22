#!/usr/bin/env python3
"""Synthetic H.264 test streams for features x264 Baseline never emits.

    stream_tools.py rewrite <in.264> <out.264> <mode>[,<mode>...]
        Re-encodes SPS / PPS / slice headers of a CAVLC Baseline stream and
        copies the slice data bits unchanged.  Modes:
          poc0     pic_order_cnt_type 0 (+ bottom field POC delta syntax)
          poc1     pic_order_cnt_type 1 with an offset cycle
          ids      SPS id 7, two identical PPS (ids 3 and 200) used alternately
          reorder  ref_pic_list_modification in every P slice with 2+ references
          mmco     adaptive reference marking: MMCO 1/2/3/4/6 instead of the
                   sliding window (the number of references stays the same)
          ltidr    IDR pictures marked as long-term references

    stream_tools.py pcm <out.264> [width height frames]
        Generates a stream from scratch: I_PCM / Intra16x16 macroblocks in I
        slices, P_Skip / P_L0_16x16 / I_PCM in P slices, several slices with
        different deblocking settings.

The streams stay standard conformant; their reference output comes from
ffmpeg like for the x264 streams.
"""
import random
import sys


# ----------------------------------------------------------------------------
# bit I/O
# ----------------------------------------------------------------------------
class BitReader:
    def __init__(self, data):
        self.bits = "".join(format(b, "08b") for b in data)
        self.pos = 0
        # rbsp_stop_one_bit position
        self.end = self.bits.rstrip("0")
        self.end = len(self.end) - 1

    def u(self, n):
        if n == 0:
            return 0
        v = int(self.bits[self.pos:self.pos + n], 2)
        self.pos += n
        return v

    def ue(self):
        lz = 0
        while self.bits[self.pos] == "0":
            lz += 1
            self.pos += 1
        self.pos += 1
        return (1 << lz) - 1 + self.u(lz)

    def se(self):
        k = self.ue()
        return (k + 1) // 2 if k & 1 else -(k // 2)

    def rest(self):
        """slice data bits up to (not including) the stop bit"""
        return self.bits[self.pos:self.end]


class BitWriter:
    def __init__(self):
        self.b = []

    def u(self, n, v):
        if n:
            self.b.append(format(v, "0%db" % n))

    def ue(self, v):
        v += 1
        n = v.bit_length()
        self.b.append("0" * (n - 1) + format(v, "b"))

    def se(self, v):
        self.ue(2 * v - 1 if v > 0 else -2 * v)

    def raw(self, bits):
        self.b.append(bits)

    def bitlen(self):
        return sum(len(x) for x in self.b)

    def rbsp(self):
        s = "".join(self.b) + "1"
        s += "0" * (-len(s) % 8)
        return bytes(int(s[i:i + 8], 2) for i in range(0, len(s), 8))


def split_nals(data):
    out, i, start = [], 0, -1
    while i + 2 < len(data):
        if data[i] == 0 and data[i + 1] == 0 and data[i + 2] == 1:
            if start >= 0:
                out.append(data[start:i].rstrip(b"\0"))
            i += 3
            start = i
        else:
            i += 1
    if start >= 0:
        out.append(data[start:].rstrip(b"\0"))
    return out


def unescape(nal):
    out, z = bytearray(), 0
    for b in nal:
        if z >= 2 and b == 3:
            z = 0
            continue
        out.append(b)
        z = z + 1 if b == 0 else 0
    return bytes(out)


def escape(rbsp):
    out, z = bytearray(), 0
    for b in rbsp:
        if z >= 2 and b <= 3:
            out.append(3)
            z = 0
        out.append(b)
        z = z + 1 if b == 0 else 0
    return bytes(out)


def nal_bytes(hdr, rbsp):
    return b"\x00\x00\x00\x01" + bytes([hdr]) + escape(rbsp)


# ----------------------------------------------------------------------------
# parameter sets
# ----------------------------------------------------------------------------
def parse_sps(br):
    s = {}
    s["profile"] = br.u(8)
    s["cons"] = br.u(8)
    s["level"] = br.u(8)
    s["id"] = br.ue()
    if s["profile"] in (100, 110, 122, 244, 44, 83, 86, 118, 128, 138, 139, 134, 135):
        raise SystemExit("only Baseline/Main SPS supported by the rewriter")
    s["log2fn"] = br.ue() + 4
    s["poc_type"] = br.ue()
    if s["poc_type"] == 0:
        s["log2poc"] = br.ue() + 4
    elif s["poc_type"] == 1:
        raise SystemExit("input already uses POC type 1")
    s["nrf"] = br.ue()
    s["gaps"] = br.u(1)
    s["w"] = br.ue() + 1
    s["h"] = br.ue() + 1
    s["fmo"] = br.u(1)
    s["d8"] = br.u(1)
    s["crop"] = br.u(1)
    if s["crop"]:
        s["cropv"] = [br.ue() for _ in range(4)]
    return s


def write_sps(s):
    bw = BitWriter()
    bw.u(8, s["profile"])
    bw.u(8, s["cons"])
    bw.u(8, s["level"])
    bw.ue(s["id"])
    bw.ue(s["log2fn"] - 4)
    bw.ue(s["poc_type"])
    if s["poc_type"] == 0:
        bw.ue(s["log2poc"] - 4)
    elif s["poc_type"] == 1:
        bw.u(1, 0)            # delta_pic_order_always_zero_flag
        bw.se(-1)             # offset_for_non_ref_pic
        bw.se(0)              # offset_for_top_to_bottom_field
        bw.ue(1)              # num_ref_frames_in_pic_order_cnt_cycle
        bw.se(2)              # offset_for_ref_frame[0]
    bw.ue(s["nrf"])
    bw.u(1, s["gaps"])
    bw.ue(s["w"] - 1)
    bw.ue(s["h"] - 1)
    bw.u(1, s["fmo"])
    bw.u(1, s["d8"])
    bw.u(1, s["crop"])
    if s["crop"]:
        for v in s["cropv"]:
            bw.ue(v)
    bw.u(1, 0)                # no VUI
    return bw.rbsp()


def parse_pps(br):
    p = {}
    p["id"] = br.ue()
    p["sps"] = br.ue()
    p["ent"] = br.u(1)
    p["bfpo"] = br.u(1)
    p["nsg"] = br.ue()
    p["nref0"] = br.ue() + 1
    p["nref1"] = br.ue() + 1
    p["wp"] = br.u(1)
    p["wb"] = br.u(2)
    p["qp"] = br.se()
    p["qs"] = br.se()
    p["cqp"] = br.se()
    p["dbc"] = br.u(1)
    p["cip"] = br.u(1)
    p["rpc"] = br.u(1)
    return p


def write_pps(p):
    bw = BitWriter()
    bw.ue(p["id"])
    bw.ue(p["sps"])
    bw.u(1, p["ent"])
    bw.u(1, p["bfpo"])
    bw.ue(p["nsg"])
    bw.ue(p["nref0"] - 1)
    bw.ue(p["nref1"] - 1)
    bw.u(1, p["wp"])
    bw.u(2, p["wb"])
    bw.se(p["qp"])
    bw.se(p["qs"])
    bw.se(p["cqp"])
    bw.u(1, p["dbc"])
    bw.u(1, p["cip"])
    bw.u(1, p["rpc"])
    return bw.rbsp()


# ----------------------------------------------------------------------------
# slice header
# ----------------------------------------------------------------------------
def parse_slice(br, nal_type, ref_idc, spss, ppss):
    h = {"nal_type": nal_type, "ref_idc": ref_idc}
    h["first_mb"] = br.ue()
    h["type"] = br.ue()
    h["pps"] = br.ue()
    p = ppss[h["pps"]]
    s = spss[p["sps"]]
    h["fn"] = br.u(s["log2fn"])
    if nal_type == 5:
        h["idr_id"] = br.ue()
    if s["poc_type"] == 0:
        h["poc_lsb"] = br.u(s["log2poc"])
        if p["bfpo"]:
            br.se()
    is_p = h["type"] % 5 == 0
    if is_p:
        h["nrovr"] = br.u(1)
        h["nref"] = br.ue() + 1 if h["nrovr"] else p["nref0"]
        if br.u(1):
            raise SystemExit("input already uses ref_pic_list_modification")
    if ref_idc:
        if nal_type == 5:
            h["noout"] = br.u(1)
            h["ltref"] = br.u(1)
        else:
            if br.u(1):
                raise SystemExit("input already uses MMCO")
    h["qpd"] = br.se()
    if p["dbc"]:
        h["ddf"] = br.ue()
        if h["ddf"] != 1:
            h["alpha"] = br.se()
            h["beta"] = br.se()
    h["data"] = br.rest()
    return h


def write_slice(h, s, p, mods, mmco, poc_lsb, ltref):
    bw = BitWriter()
    bw.ue(h["first_mb"])
    bw.ue(h["type"])
    bw.ue(p["id"])
    bw.u(s["log2fn"], h["fn"])
    if h["nal_type"] == 5:
        bw.ue(h["idr_id"])
    if s["poc_type"] == 0:
        bw.u(s["log2poc"], poc_lsb % (1 << s["log2poc"]))
        if p["bfpo"]:
            bw.se(0)
    elif s["poc_type"] == 1:
        bw.se(0)              # delta_pic_order_cnt[0]
        if p["bfpo"]:
            bw.se(0)
    if h["type"] % 5 == 0:
        bw.u(1, h["nrovr"])
        if h["nrovr"]:
            bw.ue(h["nref"] - 1)
        if mods:
            bw.u(1, 1)
            for idc, val in mods:
                bw.ue(idc)
                bw.ue(val)
            bw.ue(3)
        else:
            bw.u(1, 0)
    if h["ref_idc"]:
        if h["nal_type"] == 5:
            bw.u(1, h["noout"])
            bw.u(1, ltref)
        elif mmco is not None:
            bw.u(1, 1)
            for op in mmco:
                bw.ue(op[0])
                for a in op[1:]:
                    bw.ue(a)
            bw.ue(0)
        else:
            bw.u(1, 0)
    bw.se(h["qpd"])
    if p["dbc"]:
        bw.ue(h["ddf"])
        if h["ddf"] != 1:
            bw.se(h["alpha"])
            bw.se(h["beta"])
    bw.raw(h["data"])
    return bw.rbsp()


# ----------------------------------------------------------------------------
# reference picture state (for valid MMCO / reordering commands)
# ----------------------------------------------------------------------------
class Dpb:
    def __init__(self, nrf, log2fn):
        self.nrf = max(nrf, 1)
        self.maxfn = 1 << log2fn
        self.short = []           # frame_num, oldest first
        self.long = {}            # long_term_frame_idx -> frame_num
        self.max_lt = None

    def picnum(self, fn, cur):
        return fn - self.maxfn if fn > cur else fn

    def count(self):
        return len(self.short) + len(self.long)

    def ref_list(self, cur):
        """initial P list: short by descending PicNum, then long ascending"""
        s = sorted(self.short, key=lambda f: -self.picnum(f, cur))
        return [("s", f) for f in s] + [("l", i) for i in sorted(self.long)]

    def idr(self, fn, lt):
        self.short, self.long = [], {}
        if lt:
            self.long[0] = fn
            self.max_lt = 0
        else:
            self.short = [fn]
            self.max_lt = None

    def sliding(self, fn):
        if self.count() >= self.nrf and self.short:
            self.short.pop(0)
        self.short.append(fn)

    def mmco_plan(self, fn, k):
        """choose MMCOs for picture number k; apply them; return op list"""
        ops = []
        full = self.count() >= self.nrf
        cur_long = False
        variant = k % 6
        if variant == 1 and self.max_lt is None:
            ops.append((4, 2))                       # max_long_term_frame_idx = 1
            self.max_lt = 1
        if variant == 2 and self.max_lt is not None and 0 not in self.long and len(self.short) >= 2:
            # oldest short-term -> long-term index 0
            f = self.short.pop(0)
            ops.append((3, self.picnum_diff(f, fn), 0))
            self.long[0] = f
        elif variant == 4 and self.long and full:
            idx = sorted(self.long)[0]
            ops.append((2, idx))                     # long_term_pic_num = idx
            del self.long[idx]
            full = False
        elif variant == 5 and self.max_lt is not None and self.max_lt >= 1:
            cur_long = True
        if full and self.count() >= self.nrf:
            if self.short:
                f = self.short.pop(0)
                ops.append((1, self.picnum_diff(f, fn)))
            else:
                idx = sorted(self.long)[0]
                ops.append((2, idx))
                del self.long[idx]
        if cur_long:
            if 1 in self.long:
                del self.long[1]
            ops.append((6, 1))
            self.long[1] = fn
        else:
            self.short.append(fn)
        return ops

    def picnum_diff(self, f, cur):
        # difference_of_pic_nums_minus1 = CurrPicNum - picNumX - 1
        return cur - self.picnum(f, cur) - 1


def rewrite(inp, outp, modes):
    data = open(inp, "rb").read()
    spss, ppss = {}, {}
    out = bytearray()
    dpb = None
    pic_idx = 0
    poc_since_idr = 0
    pic_mmco = None
    pic_lt = 0
    last_first = None
    slice_ctr = 0
    for nal in split_nals(data):
        hdr = nal[0]
        t, ref_idc = hdr & 31, (hdr >> 5) & 3
        rb = unescape(nal[1:])
        br = BitReader(rb)
        if t == 7:
            s = parse_sps(br)
            spss[s["id"]] = s
            o = dict(s)
            if "poc0" in modes:
                o["poc_type"], o["log2poc"] = 0, 8
            if "poc1" in modes:
                o["poc_type"] = 1
            if "ids" in modes:
                o["id"] = 7
            s["out"] = o
            out += nal_bytes(hdr, write_sps(o))
        elif t == 8:
            p = parse_pps(br)
            ppss[p["id"]] = p
            o = dict(p)
            if "poc0" in modes or "poc1" in modes:
                o["bfpo"] = 1
            if "ids" in modes:
                o["sps"] = 7
                for pid in (3, 200):
                    o2 = dict(o)
                    o2["id"] = pid
                    out += nal_bytes(hdr, write_pps(o2))
            else:
                out += nal_bytes(hdr, write_pps(o))
            p["out"] = o
        elif t in (1, 5):
            h = parse_slice(br, t, ref_idc, spss, ppss)
            p = ppss[h["pps"]]
            s = spss[p["sps"]]
            po = dict(p["out"])
            so = s["out"]
            if "ids" in modes:
                po["id"] = 3 if slice_ctr % 2 == 0 else 200
            slice_ctr += 1
            first = h["first_mb"] == 0
            if first:
                # finish marking of the previous picture
                if dpb is not None and prev_ref:
                    if prev_idr:
                        dpb.idr(prev_fn, prev_lt)
                    elif prev_mmco is not None:
                        pass                     # already applied in mmco_plan
                    else:
                        dpb.sliding(prev_fn)
                if dpb is None:
                    dpb = Dpb(s["nrf"], s["log2fn"])
                if t == 5:
                    poc_since_idr = 0
                pic_lt = 1 if ("ltidr" in modes and t == 5) else 0
                pic_mmco = None
                if "mmco" in modes and t != 5 and ref_idc:
                    pic_mmco = dpb.mmco_plan(h["fn"], pic_idx)
                prev_ref, prev_idr, prev_fn, prev_lt, prev_mmco = ref_idc != 0, t == 5, h["fn"], pic_lt, pic_mmco
                pic_idx += 1
                poc_lsb = 2 * poc_since_idr
                poc_since_idr += 1
            mods = []
            if "reorder" in modes and h["type"] % 5 == 0:
                # list valid while this picture is decoded (before its own marking)
                lst = pre_lists.get(pic_idx)
                if lst and len(lst) >= 2:
                    cur = h["fn"]
                    pred = cur                 # picNumL0Pred = CurrPicNum
                    # move the second entry to the front, then the first to index 1
                    for kind, val in (lst[1], lst[0]):
                        if kind == "s":
                            nowrap = dpb.picnum(val, cur) % dpb.maxfn
                            if nowrap < pred:
                                mods.append((0, pred - nowrap - 1))
                            else:
                                mods.append((1, nowrap - pred - 1))
                            pred = nowrap
                        else:
                            mods.append((2, val))
            out += nal_bytes(hdr, write_slice(h, so, po, mods, pic_mmco if ref_idc and t != 5 else None,
                                              poc_lsb, pic_lt))
        else:
            out += b"\x00\x00\x00\x01" + nal
    open(outp, "wb").write(out)


# The reorder mode needs the reference list that is valid while a picture is
# decoded, i.e. before that picture's own marking.  A first pass simulates the
# DPB and records it per picture.
pre_lists = {}


def simulate_lists(inp, modes):
    data = open(inp, "rb").read()
    spss, ppss = {}, {}
    dpb = None
    pic_idx = 0
    prev = None
    for nal in split_nals(data):
        t, ref_idc = nal[0] & 31, (nal[0] >> 5) & 3
        br = BitReader(unescape(nal[1:]))
        if t == 7:
            s = parse_sps(br)
            spss[s["id"]] = s
        elif t == 8:
            p = parse_pps(br)
            ppss[p["id"]] = p
        elif t in (1, 5):
            h = parse_slice(br, t, ref_idc, spss, ppss)
            if h["first_mb"] != 0:
                continue
            s = spss[ppss[h["pps"]]["sps"]]
            if dpb is None:
                dpb = Dpb(s["nrf"], s["log2fn"])
            if prev is not None and prev[0]:
                pref, pidr, pfn, plt, pk = prev
                if pidr:
                    dpb.idr(pfn, plt)
                elif "mmco" in modes:
                    dpb.mmco_plan(pfn, pk)
                else:
                    dpb.sliding(pfn)
            pic_idx += 1
            pre_lists[pic_idx] = dpb.ref_list(h["fn"]) if t != 5 else []
            prev = (ref_idc != 0, t == 5, h["fn"], 1 if ("ltidr" in modes and t == 5) else 0, pic_idx - 1)


# ----------------------------------------------------------------------------
# generator with I_PCM macroblocks
# ----------------------------------------------------------------------------
def gen_pcm(outp, w=176, h=144, frames=4):
    rnd = random.Random(1234)
    mw, mh = w // 16, h // 16
    out = bytearray()
    s = BitWriter()
    s.u(8, 66); s.u(8, 0xC0); s.u(8, 30); s.ue(0)
    s.ue(0)          # log2_max_frame_num - 4
    s.ue(2)          # poc type 2
    s.ue(1)          # num_ref_frames
    s.u(1, 0)
    s.ue(mw - 1); s.ue(mh - 1)
    s.u(1, 1); s.u(1, 1); s.u(1, 0); s.u(1, 0)
    out += nal_bytes(0x67, s.rbsp())
    p = BitWriter()
    p.ue(0); p.ue(0); p.u(1, 0); p.u(1, 0); p.ue(0); p.ue(0); p.ue(0); p.u(1, 0); p.u(2, 0)
    p.se(0); p.se(0); p.se(2)      # pic_init_qp 26, chroma_qp_index_offset 2
    p.u(1, 1); p.u(1, 0); p.u(1, 0)
    out += nal_bytes(0x68, p.rbsp())

    def ct_zero(nc):
        return "1" if nc < 2 else "11" if nc < 4 else "1111" if nc < 8 else "000011"

    for f in range(frames):
        is_idr = f == 0
        is_p = not is_idr
        # per macroblock: 'pcm', 'i16', 'skip', 'p16'
        kinds = []
        for a in range(mw * mh):
            r = rnd.random()
            if is_p:
                kinds.append("pcm" if r < 0.15 else "i16" if r < 0.3 else "skip" if r < 0.65 else "p16")
            else:
                kinds.append("pcm" if r < 0.5 else "i16")
        tc = {}                       # luma total coeff seen by neighbours: pcm -> 16, others 0
        nslices = 3
        bounds = [mw * mh * i // nslices for i in range(nslices + 1)]
        for si in range(nslices):
            b = BitWriter()
            b.ue(bounds[si]); b.ue(5 if is_p else 7); b.ue(0)
            b.u(4, f % 16)
            if is_idr:
                b.ue(0)
            if is_p:
                b.u(1, 0)            # num_ref_idx_override
                b.u(1, 0)            # no reordering
            if is_idr:
                b.u(1, 0); b.u(1, 0)
            else:
                b.u(1, 0)            # sliding window
            b.se(rnd.randint(-6, 6))
            ddf = [0, 2, 0][si]
            b.ue(ddf)
            b.se(rnd.randint(-3, 3)); b.se(rnd.randint(-3, 3))
            first = bounds[si]
            run = 0
            for a in range(bounds[si], bounds[si + 1]):
                k = kinds[a]
                mx, my = a % mw, a // mw
                if is_p and k == "skip":
                    run += 1
                    tc[a] = 0
                    continue
                if is_p:
                    b.ue(run)
                    run = 0
                if k == "pcm":
                    b.ue(25 + (5 if is_p else 0))
                    b.u((-(b.bitlen() + 8)) % 8, 0)   # pcm_alignment_zero_bit (slice NAL has 8 bit header)
                    base = rnd.randint(30, 220)
                    for i in range(384):
                        b.u(8, max(1, min(254, base + rnd.randint(-25, 25))))
                    tc[a] = 16
                elif k == "i16":
                    b.ue(3 + (5 if is_p else 0))    # I_16x16_2_0_0 (DC, no coded blocks)
                    b.ue(0)                        # intra_chroma_pred_mode DC
                    b.se(rnd.randint(-2, 2))       # mb_qp_delta
                    av_a = mx > 0 and a - 1 >= first
                    av_b = my > 0 and a - mw >= first
                    na, nb = tc.get(a - 1, 0), tc.get(a - mw, 0)
                    if av_a and av_b:
                        nc = (na + nb + 1) >> 1
                    elif av_a:
                        nc = na
                    elif av_b:
                        nc = nb
                    else:
                        nc = 0
                    b.raw(ct_zero(nc))
                    tc[a] = 0
                else:
                    b.ue(0)                        # P_L0_16x16
                    b.se(rnd.randint(-40, 40)); b.se(rnd.randint(-30, 30))
                    b.ue(0)                        # coded_block_pattern 0
                    tc[a] = 0
            if is_p and run:
                b.ue(run)
            out += nal_bytes(0x65 if is_idr else 0x41, b.rbsp())
    open(outp, "wb").write(out)


def main():
    if len(sys.argv) >= 5 and sys.argv[1] == "rewrite":
        modes = sys.argv[4].split(",")
        simulate_lists(sys.argv[2], modes)
        rewrite(sys.argv[2], sys.argv[3], modes)
    elif len(sys.argv) >= 3 and sys.argv[1] == "pcm":
        args = [int(x) for x in sys.argv[3:6]]
        gen_pcm(sys.argv[2], *args)
    else:
        print(__doc__)
        sys.exit(1)


if __name__ == "__main__":
    main()
