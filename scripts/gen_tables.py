#!/usr/bin/env python3
"""Generate rtl/h264_tables_pkg.vhd from the tables of the C reference model.

The C model (model/h264_model.c) is the single source of truth for the
standard's tables; it is verified bit-exact against ffmpeg.  Variable length
code tables are converted into ROMs indexed by
    (table, leading zero count, next S bits after the leading one)
so that the hardware decode is: count leading zeros -> small ROM lookup.
Each ROM entry packs value*32 + code_length (length 0 = invalid code).

    gen_tables.py [model.c] [out.vhd]
"""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "..", "model", "h264_model.c")
OUT = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, "..", "rtl", "h264_tables_pkg.vhd")

text = open(SRC).read()


def parse_table(name):
    """Return nested lists for a C initializer of `name`."""
    m = re.search(r"static const uint8_t " + name + r"\s*(\[[^=]*)=\s*", text)
    if not m:
        raise SystemExit("table %s not found" % name)
    i = m.end()
    assert text[i] == "{"
    stack, cur, num = [], None, ""
    while True:
        c = text[i]
        if c == "{":
            new = []
            if cur is not None:
                cur.append(new)
                stack.append(cur)
            cur = new
        elif c.isdigit():
            num += c
        if not c.isdigit() and num:
            cur.append(int(num))
            num = ""
        if c == "}":
            if not stack:
                return cur
            cur = stack.pop()
        i += 1


def flat(t):
    return [x for r in t for x in (r if isinstance(r, list) else [r])]


# ----------------------------------------------------------------------------
# VLC -> lz/suffix ROM
# ----------------------------------------------------------------------------
def build_codes(lens, bits, values):
    codes = []
    for l, b, v in zip(lens, bits, values):
        if l:
            codes.append((format(b, "0%db" % l), v))
    return codes


def decode(codes, pattern):
    for c, v in codes:
        if pattern.startswith(c):
            return v, len(c)
    return None


def lz_rom(codes, S, lzmax):
    """ROM with (lzmax+1) * 2**S entries."""
    rom = []
    for lz in range(lzmax + 1):
        for s in range(1 << S):
            pat = "0" * lz + "1" + (format(s, "0%db" % S) if S else "") + "0" * 20
            r = decode(codes, pat)
            rom.append(0 if r is None else r[0] * 32 + r[1])
    return rom


def lz_params(codes):
    lzmax, S = 0, 0
    for c, v in codes:
        lz = len(c) - len(c.lstrip("0"))
        if lz == len(c):          # all-zero code
            lzmax = max(lzmax, lz)
        else:
            lzmax = max(lzmax, lz)
            S = max(S, len(c) - lz - 1)
    return S, lzmax


def check_rom(codes, rom, S, lzmax):
    # exhaustive check over all 16-bit patterns: emulate hardware lookup
    for p in range(1 << 16):
        pat = format(p, "016b")
        ref = decode(codes, pat + "0" * 8)
        lz = len(pat) - len(pat.lstrip("0"))
        lzc = min(lz, lzmax)
        suf = pat[lzc + 1: lzc + 1 + S] if lzc < 16 else ""
        suf = (suf + "0" * S)[:S]
        e = rom[lzc * (1 << S) + (int(suf, 2) if S else 0)]
        got = None if e == 0 else (e // 32, e % 32)
        if ref is not None and ref != got:
            raise SystemExit("ROM mismatch for pattern %s: %s vs %s" % (pat, ref, got))


families = []   # (name, S, lzmax, [roms per table])

# coeff_token tables 0..2 (nC 0-1, 2-3, 4-7) and chroma DC (nC = -1)
ct_len = parse_table("coeff_token_len")
ct_bits = parse_table("coeff_token_bits")
ct_tables = []
for t in range(3):
    vals = list(range(4 * 17))     # value = tc*4 + t1
    ct_tables.append(build_codes(ct_len[t], ct_bits[t], vals))
cdc = build_codes(flat(parse_table("chroma_dc_coeff_token_len")),
                  flat(parse_table("chroma_dc_coeff_token_bits")), list(range(4 * 5)))
ct_tables.append(cdc)
S = max(lz_params(c)[0] for c in ct_tables)
LZ = max(lz_params(c)[1] for c in ct_tables)
families.append(("CT", S, LZ, ct_tables))

# nC >= 8: 6 bit fixed length code (checked separately)
flc = build_codes(ct_len[3], ct_bits[3], list(range(4 * 17)))
flc_rom = []
for p in range(64):
    r = decode(flc, format(p, "06b"))
    flc_rom.append(0 if r is None else r[0] * 32 + r[1])

# total_zeros tc=1..15 and chroma DC tc=1..3
tz_len = parse_table("total_zeros_len")
tz_bits = parse_table("total_zeros_bits")
tz_tables = [build_codes(tz_len[t], tz_bits[t], list(range(16))) for t in range(15)]
ctz_len = parse_table("chroma_dc_total_zeros_len")
ctz_bits = parse_table("chroma_dc_total_zeros_bits")
tz_tables += [build_codes(ctz_len[t], ctz_bits[t], list(range(4))) for t in range(3)]
S = max(lz_params(c)[0] for c in tz_tables)
LZ = max(lz_params(c)[1] for c in tz_tables)
families.append(("TZ", S, LZ, tz_tables))

# run_before zeros_left = 1..6, >6
rb_len = parse_table("run_len")
rb_bits = parse_table("run_bits")
rb_tables = [build_codes(rb_len[t], rb_bits[t], list(range(16))) for t in range(7)]
S = max(lz_params(c)[0] for c in rb_tables)
LZ = max(lz_params(c)[1] for c in rb_tables)
families.append(("RB", S, LZ, rb_tables))

roms = {}
for name, S, LZ, tables in families:
    data = []
    for codes in tables:
        rom = lz_rom(codes, S, LZ)
        check_rom(codes, rom, S, LZ)
        data += rom
    roms[name] = (S, LZ, len(tables), data)
for p in range(64):
    r = decode(flc, format(p, "06b"))
    e = flc_rom[p]
    assert (r is None and e == 0) or (r and e == r[0] * 32 + r[1])


def vhdl_array(name, typ, data, per_line=16):
    lines = []
    for i in range(0, len(data), per_line):
        lines.append("        " + ", ".join("%d" % d for d in data[i:i + per_line]))
    return "    constant %s : %s(0 to %d) := (\n%s);\n" % (name, typ, len(data) - 1, ",\n".join(lines))


out = []
out.append("""-- ============================================================================
-- h264_tables_pkg.vhd  --  GENERATED by scripts/gen_tables.py, do not edit.
-- Tables of ITU-T H.264 (CAVLC VLCs, CBP mapping, quantisation, deblocking).
-- VLC ROM entries: value*32 + code length (length 0 = invalid).
-- ROM index = table*(LZMAX+1)*2**S + min(lz,LZMAX)*2**S + next S bits after
-- the leading one.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package h264_tables_pkg is

    type int_array_t is array (natural range <>) of integer;
""")
for name, (S, LZ, ntab, data) in roms.items():
    out.append("    constant %s_S     : natural := %d;\n" % (name, S))
    out.append("    constant %s_LZMAX : natural := %d;\n" % (name, LZ))
    out.append(vhdl_array("%s_ROM" % name, "int_array_t", data))
out.append(vhdl_array("CT_FLC_ROM", "int_array_t", flc_rom))
out.append(vhdl_array("CBP_INTRA_TAB", "int_array_t", parse_table("golomb_to_intra4x4_cbp")))
out.append(vhdl_array("CBP_INTER_TAB", "int_array_t", parse_table("golomb_to_inter_cbp")))
out.append(vhdl_array("ZIGZAG_TAB", "int_array_t", parse_table("zigzag4x4")))
out.append(vhdl_array("DEQUANT_TAB", "int_array_t", flat(parse_table("dequant_v"))))
out.append(vhdl_array("CHROMA_QP_TAB", "int_array_t", parse_table("chroma_qp_tab")))
out.append(vhdl_array("ALPHA_TAB", "int_array_t", parse_table("alpha_tab")))
out.append(vhdl_array("BETA_TAB", "int_array_t", parse_table("beta_tab")))
out.append(vhdl_array("TC0_TAB", "int_array_t", flat(parse_table("tc0_tab"))))
out.append("""
    -- lz/suffix VLC lookup on a 16 bit window; returns packed ROM entry
    function vlc_lookup(rom : int_array_t; s_bits : natural; lzmax : natural;
                        tab : natural; b : std_logic_vector(15 downto 0)) return natural;

end package h264_tables_pkg;

package body h264_tables_pkg is

    -- All leading zero counts are evaluated in parallel: candidate k reads
    -- the s_bits after a leading one at position k (fixed bit positions, no
    -- shifter) from its part of the ROM; exactly one candidate is selected
    -- (k = lzmax also covers more leading zeros).
    function vlc_lookup(rom : int_array_t; s_bits : natural; lzmax : natural;
                        tab : natural; b : std_logic_vector(15 downto 0)) return natural is
        variable acc  : unsigned(15 downto 0);
        variable hit  : boolean;
        variable zpfx : boolean;
        variable suf  : natural;
        variable m    : unsigned(15 downto 0);
    begin
        acc := (others => '0');
        for k in 0 to lzmax loop
            zpfx := true;
            for i in 0 to k - 1 loop
                if b(15 - i) = '1' then
                    zpfx := false;
                end if;
            end loop;
            if k < lzmax then
                hit := zpfx and b(15 - k) = '1';
            else
                hit := zpfx;
            end if;
            suf := 0;
            for j in 1 to s_bits loop
                suf := suf * 2;
                if 15 - k - j >= 0 then
                    if b(15 - k - j) = '1' then
                        suf := suf + 1;
                    end if;
                end if;
            end loop;
            if hit then
                m := (others => '1');
            else
                m := (others => '0');
            end if;
            acc := acc or (to_unsigned(rom(tab * (lzmax + 1) * (2 ** s_bits) + k * (2 ** s_bits) + suf), 16) and m);
        end loop;
        return to_integer(acc);
    end function;

end package body h264_tables_pkg;
""")
open(OUT, "w").write("".join(out))
print("wrote", OUT)
for name, (S, LZ, ntab, data) in roms.items():
    print("  %s: S=%d LZMAX=%d tables=%d entries=%d" % (name, S, LZ, ntab, len(data)))
