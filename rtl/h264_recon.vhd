-- ============================================================================
-- h264_recon.vhd
-- Macroblock reconstruction:
--   * Intra16x16 luma DC (4x4 Hadamard) and chroma DC (2x2 Hadamard) with
--     their dequantisation
--   * 4x4 dequantisation + inverse integer transform
--   * Intra4x4 (9 modes), Intra16x16 (4 modes) and chroma (4 modes)
--     prediction from unfiltered neighbours (line buffer + left column)
--   * inter prediction samples come from the MC stage prediction buffer
--   * I_PCM samples are copied from the coefficient buffer
-- The unfiltered macroblock is written to the deblocking input buffer.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.h264_pkg.all;
use work.h264_tables_pkg.all;

entity h264_recon is
    port (
        clk         : in  std_logic;
        rst         : in  std_logic;
        -- token in
        i_mb        : in  mbinfo_t;
        i_mb_valid  : in  std_logic;
        o_mb_ready  : out std_logic;
        -- coefficient buffer
        cr_addr     : out unsigned(8 downto 0);
        cr_data     : in  std_logic_vector(63 downto 0);
        o_coef_free : out std_logic;
        -- inter prediction buffer
        pr_addr     : out unsigned(5 downto 0);
        pr_data     : in  std_logic_vector(127 downto 0);
        o_pred_free : out std_logic;
        -- reconstructed macroblock buffer
        rw_en       : out std_logic_vector(15 downto 0);
        rw_addr     : out unsigned(5 downto 0);
        rw_data     : out std_logic_vector(127 downto 0);
        i_rec_free  : in  std_logic;
        -- token out
        o_mb        : out mbinfo_t;
        o_mb_valid  : out std_logic;
        i_mb_ready  : in  std_logic
    );
end entity;

architecture rtl of h264_recon is

    subtype pix_t is integer range 0 to 255;
    type pix4_t  is array (0 to 3) of pix_t;
    type pix8_t  is array (0 to 7) of pix_t;
    type pix16_t is array (0 to 15) of pix_t;
    type pix16x4_t is array (0 to 3) of pix16_t;
    type pix8x2_t is array (0 to 1) of pix8_t;
    subtype c20_t is signed(19 downto 0);
    type c20_arr16_t is array (0 to 15) of c20_t;
    type c20_arr8_t is array (0 to 7) of c20_t;
    type nat16_t is array (0 to 15) of natural range 0 to 15;
    constant BLK_RASTER : nat16_t := (0, 1, 4, 5, 2, 3, 6, 7, 8, 9, 12, 13, 10, 11, 14, 15);

    -- ------------------------------------------------------------------
    -- arithmetic helpers
    -- ------------------------------------------------------------------
    function clip1(v : integer) return pix_t is
    begin
        if v < 0 then
            return 0;
        elsif v > 255 then
            return 255;
        end if;
        return v;
    end function;

    -- floor(v / 2**n) for any sign
    function asr(v : integer; n : natural) return integer is
    begin
        return to_integer(shift_right(to_signed(v, 32), n));
    end function;

    function idct4x4(d : c20_arr16_t) return c20_arr16_t is
        variable t, r : c20_arr16_t;
        variable e0, e1, e2, e3 : c20_t;
    begin
        for i in 0 to 3 loop
            e0 := d(i * 4) + d(i * 4 + 2);
            e1 := d(i * 4) - d(i * 4 + 2);
            e2 := shift_right(d(i * 4 + 1), 1) - d(i * 4 + 3);
            e3 := d(i * 4 + 1) + shift_right(d(i * 4 + 3), 1);
            t(i * 4) := e0 + e3;
            t(i * 4 + 1) := e1 + e2;
            t(i * 4 + 2) := e1 - e2;
            t(i * 4 + 3) := e0 - e3;
        end loop;
        for j in 0 to 3 loop
            e0 := t(j) + t(8 + j);
            e1 := t(j) - t(8 + j);
            e2 := shift_right(t(4 + j), 1) - t(12 + j);
            e3 := t(4 + j) + shift_right(t(12 + j), 1);
            r(j) := shift_right(e0 + e3 + 32, 6);
            r(4 + j) := shift_right(e1 + e2 + 32, 6);
            r(8 + j) := shift_right(e1 - e2 + 32, 6);
            r(12 + j) := shift_right(e0 - e3 + 32, 6);
        end loop;
        return r;
    end function;

    function dq_v(qrem : natural; pos : natural) return natural is
    begin
        return DEQUANT_TAB(qrem * 3 + (pos mod 2) + ((pos / 4) mod 2));
    end function;

    -- Intra 4x4 prediction (t(4..7) = top right, already substituted)
    type edge_t is array (-1 to 7) of integer range 0 to 255;
    function pred4x4(mode : natural; t : pix8_t; l : pix4_t; tl : pix_t; at, al : boolean) return pix16_t is
        variable p  : pix16_t;
        variable tt : edge_t;      -- tt(-1) = top-left
        variable ll : edge_t;      -- ll(-1) = top-left, ll(0..3) left column
        variable v, z, s : integer;
    begin
        tt(-1) := tl;
        ll(-1) := tl;
        for i in 0 to 7 loop
            tt(i) := t(i);
        end loop;
        for i in 0 to 3 loop
            ll(i) := l(i);
        end loop;
        for i in 4 to 7 loop
            ll(i) := l(3);
        end loop;
        for y in 0 to 3 loop
            for x in 0 to 3 loop
                v := 128;
                case mode is
                    when 0 => v := tt(x);
                    when 1 => v := ll(y);
                    when 2 =>
                        if at and al then
                            v := (t(0) + t(1) + t(2) + t(3) + l(0) + l(1) + l(2) + l(3) + 4) / 8;
                        elsif al then
                            v := (l(0) + l(1) + l(2) + l(3) + 2) / 4;
                        elsif at then
                            v := (t(0) + t(1) + t(2) + t(3) + 2) / 4;
                        else
                            v := 128;
                        end if;
                    when 3 =>
                        if x = 3 and y = 3 then
                            v := (tt(6) + 3 * tt(7) + 2) / 4;
                        else
                            v := (tt(x + y) + 2 * tt(x + y + 1) + tt(x + y + 2) + 2) / 4;
                        end if;
                    when 4 =>
                        if x > y then
                            v := (tt(x - y - 2) + 2 * tt(x - y - 1) + tt(x - y) + 2) / 4;
                        elsif x < y then
                            v := (ll(y - x - 2) + 2 * ll(y - x - 1) + ll(y - x) + 2) / 4;
                        else
                            v := (tt(0) + 2 * tl + ll(0) + 2) / 4;
                        end if;
                    when 5 =>
                        z := 2 * x - y;
                        s := y / 2;
                        if z >= 0 and z mod 2 = 0 then
                            v := (tt(x - s - 1) + tt(x - s) + 1) / 2;
                        elsif z >= 0 then
                            v := (tt(x - s - 2) + 2 * tt(x - s - 1) + tt(x - s) + 2) / 4;
                        elsif z = -1 then
                            v := (ll(0) + 2 * tl + tt(0) + 2) / 4;
                        else
                            v := (ll(y - 1) + 2 * ll(y - 2) + ll(y - 3) + 2) / 4;
                        end if;
                    when 6 =>
                        z := 2 * y - x;
                        s := x / 2;
                        if z >= 0 and z mod 2 = 0 then
                            v := (ll(y - s - 1) + ll(y - s) + 1) / 2;
                        elsif z >= 0 then
                            v := (ll(y - s - 2) + 2 * ll(y - s - 1) + ll(y - s) + 2) / 4;
                        elsif z = -1 then
                            v := (ll(0) + 2 * tl + tt(0) + 2) / 4;
                        else
                            v := (tt(x - 1) + 2 * tt(x - 2) + tt(x - 3) + 2) / 4;
                        end if;
                    when 7 =>
                        s := y / 2;
                        if y mod 2 = 0 then
                            v := (tt(x + s) + tt(x + s + 1) + 1) / 2;
                        else
                            v := (tt(x + s) + 2 * tt(x + s + 1) + tt(x + s + 2) + 2) / 4;
                        end if;
                    when others =>
                        z := x + 2 * y;
                        s := x / 2;
                        if z < 5 and z mod 2 = 0 then
                            v := (ll(y + s) + ll(y + s + 1) + 1) / 2;
                        elsif z < 5 then
                            v := (ll(y + s) + 2 * ll(y + s + 1) + ll(y + s + 2) + 2) / 4;
                        elsif z = 5 then
                            v := (ll(2) + 3 * ll(3) + 2) / 4;
                        else
                            v := ll(3);
                        end if;
                end case;
                p(y * 4 + x) := v;
            end loop;
        end loop;
        return p;
    end function;

    -- ------------------------------------------------------------------
    -- line buffer (unfiltered bottom row of the macroblock row above)
    -- ------------------------------------------------------------------
    type lb_t is array (0 to C_MAX_MB_W - 1) of std_logic_vector(255 downto 0);
    signal lb_ram : lb_t := (others => (others => '0'));
    signal lb_we  : std_logic;
    signal lb_wa  : unsigned(6 downto 0);
    signal lb_wd  : std_logic_vector(255 downto 0);
    signal lb_ra  : unsigned(6 downto 0) := (others => '0');
    signal lb_rd  : std_logic_vector(255 downto 0);

    function lb_luma(v : std_logic_vector(255 downto 0)) return pix16_t is
        variable p : pix16_t;
    begin
        for i in 0 to 15 loop
            p(i) := to_integer(unsigned(v(i * 8 + 7 downto i * 8)));
        end loop;
        return p;
    end function;
    function lb_chroma(v : std_logic_vector(255 downto 0); c : natural) return pix8_t is
        variable p : pix8_t;
    begin
        for i in 0 to 7 loop
            p(i) := to_integer(unsigned(v(128 + c * 64 + i * 8 + 7 downto 128 + c * 64 + i * 8)));
        end loop;
        return p;
    end function;

    -- ------------------------------------------------------------------
    type state_t is (R_IDLE, R_LB0, R_LB1, R_LB2, R_DC, R_DC2, R_PREP, R_BLK, R_B_RD, R_B_RES,
                     R_B_WR, R_END);
    signal st : state_t;
    signal st_dbg : natural;

    signal mb      : mbinfo_t;
    signal rslot   : std_logic;
    signal rec_cnt : unsigned(1 downto 0);
    signal tok     : mbinfo_t;
    signal tok_v   : std_logic;

    -- neighbours
    signal top_l   : pix16_t;
    signal top_c   : pix8x2_t;
    signal nxt_l   : pix16_t;
    signal nxt_c   : pix8x2_t;
    signal tl_l    : pix_t;
    type pix_arr2_t is array (0 to 1) of pix_t;
    signal tl_c    : pix_arr2_t;
    signal left_l  : pix16_t;
    signal left_c  : pix8x2_t;
    signal reload  : std_logic;
    -- current macroblock edges
    signal rowr    : pix16x4_t;      -- luma rows 3, 7, 11, 15
    signal colr    : pix16x4_t;      -- luma columns 3, 7, 11, 15
    signal crow    : pix8x2_t;       -- chroma row 7
    signal ccol    : pix8x2_t;       -- chroma column 7

    -- DC
    signal ldc     : c20_arr16_t;
    signal cdc     : c20_arr8_t;
    signal dcl     : c20_arr16_t;    -- levels as read
    signal dcc     : c20_arr8_t;
    signal k       : unsigned(3 downto 0);

    -- Intra16x16 / chroma parameters
    signal i16_dc  : pix_t;
    signal pl_a, pl_b, pl_c : integer range -65536 to 65535;
    type int2_t is array (0 to 1) of integer range -65536 to 65535;
    signal cpl_a, cpl_b, cpl_c : int2_t;

    -- block processing
    signal n       : unsigned(4 downto 0);     -- 0..15 luma (blkIdx), 16..19 Cb, 20..23 Cr
    signal rc      : unsigned(2 downto 0);
    signal need_c  : std_logic;
    signal need_p  : std_logic;
    signal dq      : c20_arr16_t;
    signal ppix    : pix16_t;
    signal rec     : pix16_t;
    signal qrem_l, qdiv_l, qrem_c, qdiv_c : natural range 0 to 15;

    signal coef_free, pred_free : std_logic;
begin
    st_dbg <= state_t'pos(st);
    o_mb <= tok;
    o_mb_valid <= tok_v;
    o_mb_ready <= '1' when st = R_IDLE and rec_cnt < 2 else '0';
    o_coef_free <= coef_free;
    o_pred_free <= pred_free;

    process (clk)
    begin
        if rising_edge(clk) then
            if lb_we = '1' then
                lb_ram(to_integer(lb_wa)) <= lb_wd;
            end if;
            lb_rd <= lb_ram(to_integer(lb_ra));
        end if;
    end process;

    process (clk)
        variable r, bx, by, c, b : integer;
        variable lv     : c20_arr16_t;
        variable res    : c20_arr16_t;
        variable t8     : pix8_t;
        variable l4     : pix4_t;
        variable tlv    : pix_t;
        variable at, al, atr : boolean;
        variable pp     : pix16_t;
        variable v, s, h, vv : integer;
        variable f      : c20_arr16_t;
        variable tmp    : c20_arr16_t;
        variable ls     : integer;
        variable x0     : integer;
        variable w64    : signed(39 downto 0);
        variable word   : std_logic_vector(127 downto 0);
        variable en     : std_logic_vector(15 downto 0);
        variable xo, yo, sumt, suml : integer;
        variable cf     : signed(15 downto 0);
        variable prod   : signed(31 downto 0);
        variable a_l, a_t : boolean;
        variable prb    : pix16_t;
    begin
        if rising_edge(clk) then
            lb_we <= '0';
            rw_en <= (others => '0');
            coef_free <= '0';
            pred_free <= '0';
            if tok_v = '1' and i_mb_ready = '1' then
                tok_v <= '0';
            end if;
            if i_rec_free = '1' then
                rec_cnt <= rec_cnt - 1;
            end if;

            if rst = '1' then
                st <= R_IDLE;
                tok_v <= '0';
                rec_cnt <= (others => '0');
                rslot <= '0';
                tok <= MBINFO_INIT;
                cr_addr <= (others => '0');
                pr_addr <= (others => '0');
            else
                case st is
                    when R_IDLE =>
                        if i_mb_valid = '1' and rec_cnt < 2 then
                            mb <= i_mb;
                            qrem_l <= to_integer(i_mb.qp) mod 6;
                            qdiv_l <= to_integer(i_mb.qp) / 6;
                            qrem_c <= to_integer(i_mb.qpc) mod 6;
                            qdiv_c <= to_integer(i_mb.qpc) / 6;
                            if i_mb.mb_x = 0 then
                                lb_ra <= (others => '0');
                                reload <= '1';
                            else
                                -- slide the top window: old top becomes top-left source
                                tl_l <= top_l(15);
                                tl_c(0) <= top_c(0)(7);
                                tl_c(1) <= top_c(1)(7);
                                top_l <= nxt_l;
                                top_c <= nxt_c;
                                lb_ra <= i_mb.mb_x + 1;
                                reload <= '0';
                            end if;
                            if i_rec_free = '1' then
                                rec_cnt <= rec_cnt;
                            else
                                rec_cnt <= rec_cnt + 1;
                            end if;
                            st <= R_LB0;
                        end if;

                    when R_LB0 =>
                        lb_ra <= mb.mb_x + 1;
                        st <= R_LB1;
                    when R_LB1 =>
                        if reload = '1' then
                            top_l <= lb_luma(lb_rd);
                            top_c(0) <= lb_chroma(lb_rd, 0);
                            top_c(1) <= lb_chroma(lb_rd, 1);
                        end if;
                        st <= R_LB2;
                    when R_LB2 =>
                        nxt_l <= lb_luma(lb_rd);
                        nxt_c(0) <= lb_chroma(lb_rd, 0);
                        nxt_c(1) <= lb_chroma(lb_rd, 1);
                        k <= (others => '0');
                        st <= R_DC;

                    -- ------------------------------------------------
                    -- DC coefficients: read rows LDC 0..3 and CDC 0..1
                    -- ------------------------------------------------
                    when R_DC =>
                        -- read LDC rows 0..3 and CDC rows 0..1 (2 cycle read latency)
                        if k < 4 then
                            cr_addr <= mb.coef_slot & to_unsigned(CB_LDC, 5) & k(1 downto 0);
                        else
                            cr_addr <= mb.coef_slot & to_unsigned(CB_CDC, 5) & k(1 downto 0);
                        end if;
                        if k >= 2 and k <= 5 then
                            for i in 0 to 3 loop
                                if mb.nz(16) = '1' and mb.mbtype = MBT_I16 then
                                    dcl((to_integer(k) - 2) * 4 + i) <= resize(signed(cr_data(i * 16 + 15 downto i * 16)), 20);
                                else
                                    dcl((to_integer(k) - 2) * 4 + i) <= (others => '0');
                                end if;
                            end loop;
                        elsif k >= 6 then
                            for i in 0 to 3 loop
                                if mb.nz(17) = '1' then
                                    dcc((to_integer(k) - 6) * 4 + i) <= resize(signed(cr_data(i * 16 + 15 downto i * 16)), 20);
                                else
                                    dcc((to_integer(k) - 6) * 4 + i) <= (others => '0');
                                end if;
                            end loop;
                        end if;
                        k <= k + 1;
                        if k = 7 then
                            st <= R_DC2;
                        end if;

                    when R_DC2 =>
                        -- luma DC: 4x4 Hadamard
                        for i in 0 to 3 loop
                            tmp(i * 4) := dcl(i * 4) + dcl(i * 4 + 1) + dcl(i * 4 + 2) + dcl(i * 4 + 3);
                            tmp(i * 4 + 1) := dcl(i * 4) + dcl(i * 4 + 1) - dcl(i * 4 + 2) - dcl(i * 4 + 3);
                            tmp(i * 4 + 2) := dcl(i * 4) - dcl(i * 4 + 1) - dcl(i * 4 + 2) + dcl(i * 4 + 3);
                            tmp(i * 4 + 3) := dcl(i * 4) - dcl(i * 4 + 1) + dcl(i * 4 + 2) - dcl(i * 4 + 3);
                        end loop;
                        for j in 0 to 3 loop
                            f(j) := tmp(j) + tmp(4 + j) + tmp(8 + j) + tmp(12 + j);
                            f(4 + j) := tmp(j) + tmp(4 + j) - tmp(8 + j) - tmp(12 + j);
                            f(8 + j) := tmp(j) - tmp(4 + j) - tmp(8 + j) + tmp(12 + j);
                            f(12 + j) := tmp(j) - tmp(4 + j) + tmp(8 + j) - tmp(12 + j);
                        end loop;
                        ls := 16 * DEQUANT_TAB(qrem_l * 3);
                        for i in 0 to 15 loop
                            w64 := resize(resize(f(i), 28) * to_signed(ls, 12), 40);
                            if qdiv_l >= 6 then
                                w64 := shift_left(w64, qdiv_l - 6);
                            else
                                w64 := shift_right(w64 + shift_left(to_signed(1, 40), 5 - qdiv_l), 6 - qdiv_l);
                            end if;
                            ldc(i) <= resize(w64, 20);
                        end loop;
                        -- chroma DC: 2x2 Hadamard per component
                        ls := 16 * DEQUANT_TAB(qrem_c * 3);
                        for c2 in 0 to 1 loop
                            f(0) := dcc(c2 * 4) + dcc(c2 * 4 + 1) + dcc(c2 * 4 + 2) + dcc(c2 * 4 + 3);
                            f(1) := dcc(c2 * 4) - dcc(c2 * 4 + 1) + dcc(c2 * 4 + 2) - dcc(c2 * 4 + 3);
                            f(2) := dcc(c2 * 4) + dcc(c2 * 4 + 1) - dcc(c2 * 4 + 2) - dcc(c2 * 4 + 3);
                            f(3) := dcc(c2 * 4) - dcc(c2 * 4 + 1) - dcc(c2 * 4 + 2) + dcc(c2 * 4 + 3);
                            for i in 0 to 3 loop
                                w64 := resize(resize(f(i), 28) * to_signed(ls, 12), 40);
                                w64 := shift_right(shift_left(w64, qdiv_c), 5);
                                cdc(c2 * 4 + i) <= resize(w64, 20);
                            end loop;
                        end loop;
                        st <= R_PREP;

                    -- ------------------------------------------------
                    -- Intra16x16 / chroma prediction parameters
                    -- ------------------------------------------------
                    when R_PREP =>
                        sumt := 0;
                        suml := 0;
                        for i in 0 to 15 loop
                            sumt := sumt + top_l(i);
                            suml := suml + left_l(i);
                        end loop;
                        if mb.avail_a = '1' and mb.avail_b = '1' then
                            i16_dc <= (sumt + suml + 16) / 32;
                        elsif mb.avail_a = '1' then
                            i16_dc <= (suml + 8) / 16;
                        elsif mb.avail_b = '1' then
                            i16_dc <= (sumt + 8) / 16;
                        else
                            i16_dc <= 128;
                        end if;
                        h := 0;
                        vv := 0;
                        for i in 0 to 7 loop
                            if i = 7 then
                                h := h + (i + 1) * (top_l(8 + i) - tl_l);
                                vv := vv + (i + 1) * (left_l(8 + i) - tl_l);
                            else
                                h := h + (i + 1) * (top_l(8 + i) - top_l(6 - i));
                                vv := vv + (i + 1) * (left_l(8 + i) - left_l(6 - i));
                            end if;
                        end loop;
                        pl_a <= 16 * (left_l(15) + top_l(15));
                        pl_b <= asr(5 * h + 32, 6);
                        pl_c <= asr(5 * vv + 32, 6);
                        for c2 in 0 to 1 loop
                            h := 0;
                            vv := 0;
                            for i in 0 to 3 loop
                                if i = 3 then
                                    h := h + (i + 1) * (top_c(c2)(4 + i) - tl_c(c2));
                                    vv := vv + (i + 1) * (left_c(c2)(4 + i) - tl_c(c2));
                                else
                                    h := h + (i + 1) * (top_c(c2)(4 + i) - top_c(c2)(2 - i));
                                    vv := vv + (i + 1) * (left_c(c2)(4 + i) - left_c(c2)(2 - i));
                                end if;
                            end loop;
                            cpl_a(c2) <= 16 * (left_c(c2)(7) + top_c(c2)(7));
                            cpl_b(c2) <= asr(34 * h + 32, 6);
                            cpl_c(c2) <= asr(34 * vv + 32, 6);
                        end loop;
                        n <= (others => '0');
                        st <= R_BLK;

                    -- ------------------------------------------------
                    -- per 4x4 block
                    -- ------------------------------------------------
                    when R_BLK =>
                        if n = 24 then
                            st <= R_END;
                        else
                            if n < 16 then
                                r := BLK_RASTER(to_integer(n(3 downto 0)));
                                need_c <= mb.nz(r);
                            elsif n < 20 then
                                need_c <= mb.nz(CB_CB + to_integer(n) - 16);
                            else
                                need_c <= mb.nz(CB_CB + to_integer(n) - 16);
                            end if;
                            if mb.mbtype = MBT_IPCM then
                                need_c <= '1';
                            end if;
                            need_p <= to_sl(not mbt_is_intra(mb.mbtype));
                            dq <= (others => (others => '0'));
                            rc <= (others => '0');
                            st <= R_B_RD;
                        end if;

                    when R_B_RD =>
                        -- block coordinates
                        if n < 16 then
                            r := BLK_RASTER(to_integer(n(3 downto 0)));
                            bx := r mod 4;
                            by := r / 4;
                            b := r;
                        else
                            c := (to_integer(n) - 16) / 4;
                            b := (to_integer(n) - 16) mod 4;
                            bx := b mod 2;
                            by := b / 2;
                            b := CB_CB + c * 4 + b;
                        end if;
                        if need_c = '0' and need_p = '0' then
                            st <= R_B_RES;
                        else
                            -- request row rc
                            if rc < 4 then
                                cr_addr <= mb.coef_slot & to_unsigned(b, 5) & rc(1 downto 0);
                                if n < 16 then
                                    pr_addr <= mb.pred_slot & to_unsigned(by * 4 + to_integer(rc), 5);
                                else
                                    pr_addr <= mb.pred_slot & to_unsigned(16 + by * 4 + to_integer(rc), 5);
                                end if;
                            end if;
                            -- capture row rc-2 (2 cycle read latency)
                            for i in 0 to 15 loop
                                prb(i) := to_integer(unsigned(pr_data(i * 8 + 7 downto i * 8)));
                            end loop;
                            if rc >= 2 then
                                for i in 0 to 3 loop
                                    v := (to_integer(rc) - 2) * 4 + i;
                                    cf := signed(cr_data(i * 16 + 15 downto i * 16));
                                    if mb.mbtype = MBT_IPCM then
                                        ppix(v) <= to_integer(unsigned(cr_data(i * 16 + 7 downto i * 16)));
                                    else
                                        if need_c = '1' then
                                            if n < 16 then
                                                prod := cf * to_signed(dq_v(qrem_l, v), 16);
                                                dq(v) <= resize(shift_left(prod, qdiv_l), 20);
                                            else
                                                prod := cf * to_signed(dq_v(qrem_c, v), 16);
                                                dq(v) <= resize(shift_left(prod, qdiv_c), 20);
                                            end if;
                                        end if;
                                        if need_p = '1' then
                                            if n < 16 then
                                                x0 := bx * 4 + i;
                                            else
                                                x0 := c * 8 + bx * 4 + i;
                                            end if;
                                            ppix(v) <= prb(x0);
                                        end if;
                                    end if;
                                end loop;
                            end if;
                            rc <= rc + 1;
                            if rc = 5 then
                                st <= R_B_RES;
                            end if;
                        end if;

                    when R_B_RES =>
                        if n < 16 then
                            r := BLK_RASTER(to_integer(n(3 downto 0)));
                            bx := r mod 4;
                            by := r / 4;
                        else
                            c := (to_integer(n) - 16) / 4;
                            b := (to_integer(n) - 16) mod 4;
                            bx := b mod 2;
                            by := b / 2;
                        end if;
                        -- residual
                        lv := dq;
                        if n < 16 and mb.mbtype = MBT_I16 then
                            lv(0) := ldc(r);
                        elsif n >= 16 then
                            lv(0) := cdc(c * 4 + b);
                        end if;
                        res := idct4x4(lv);
                        -- prediction
                        pp := ppix;
                        if mb.mbtype = MBT_I4x4 and n < 16 then
                            for i in 0 to 3 loop
                                if by = 0 then
                                    t8(i) := top_l(bx * 4 + i);
                                else
                                    t8(i) := rowr(by - 1)(bx * 4 + i);
                                end if;
                                if bx = 0 then
                                    l4(i) := left_l(by * 4 + i);
                                else
                                    l4(i) := colr(bx - 1)(by * 4 + i);
                                end if;
                            end loop;
                            if bx = 0 and by = 0 then
                                tlv := tl_l;
                            elsif bx = 0 then
                                tlv := left_l(by * 4 - 1);
                            elsif by = 0 then
                                tlv := top_l(bx * 4 - 1);
                            else
                                tlv := rowr(by - 1)(bx * 4 - 1);
                            end if;
                            at := by > 0 or mb.avail_b = '1';
                            al := bx > 0 or mb.avail_a = '1';
                            if by = 0 then
                                if bx < 3 then
                                    atr := mb.avail_b = '1';
                                else
                                    atr := mb.avail_c = '1';
                                end if;
                            elsif bx = 3 then
                                atr := false;
                            else
                                atr := not ((bx = 1 and by = 1) or (bx = 1 and by = 3));
                            end if;
                            for i in 0 to 3 loop
                                if not atr then
                                    t8(4 + i) := t8(3);
                                elsif by = 0 and bx = 3 then
                                    t8(4 + i) := nxt_l(i);
                                elsif by = 0 then
                                    t8(4 + i) := top_l(bx * 4 + 4 + i);
                                else
                                    t8(4 + i) := rowr(by - 1)(bx * 4 + 4 + i);
                                end if;
                            end loop;
                            pp := pred4x4(to_integer(mb.i4_modes(r)), t8, l4, tlv, at, al);
                        elsif mb.mbtype = MBT_I16 and n < 16 then
                            for y in 0 to 3 loop
                                for x in 0 to 3 loop
                                    case to_integer(mb.i16_mode) is
                                        when 0 => v := top_l(bx * 4 + x);
                                        when 1 => v := left_l(by * 4 + y);
                                        when 2 => v := i16_dc;
                                        when others =>
                                            v := clip1(asr(pl_a + pl_b * (bx * 4 + x - 7) + pl_c * (by * 4 + y - 7) + 16, 5));
                                    end case;
                                    pp(y * 4 + x) := v;
                                end loop;
                            end loop;
                        elsif mbt_is_intra(mb.mbtype) and mb.mbtype /= MBT_IPCM and n >= 16 then
                            xo := bx * 4;
                            yo := by * 4;
                            sumt := top_c(c)(xo) + top_c(c)(xo + 1) + top_c(c)(xo + 2) + top_c(c)(xo + 3);
                            suml := left_c(c)(yo) + left_c(c)(yo + 1) + left_c(c)(yo + 2) + left_c(c)(yo + 3);
                            a_l := mb.avail_a = '1';
                            a_t := mb.avail_b = '1';
                            if b = 0 or b = 3 then
                                if a_l and a_t then
                                    s := (sumt + suml + 4) / 8;
                                elsif a_l then
                                    s := (suml + 2) / 4;
                                elsif a_t then
                                    s := (sumt + 2) / 4;
                                else
                                    s := 128;
                                end if;
                            elsif b = 1 then
                                if a_t then
                                    s := (sumt + 2) / 4;
                                elsif a_l then
                                    s := (suml + 2) / 4;
                                else
                                    s := 128;
                                end if;
                            else
                                if a_l then
                                    s := (suml + 2) / 4;
                                elsif a_t then
                                    s := (sumt + 2) / 4;
                                else
                                    s := 128;
                                end if;
                            end if;
                            for y in 0 to 3 loop
                                for x in 0 to 3 loop
                                    case to_integer(mb.chroma_mode) is
                                        when 0 => v := s;
                                        when 1 => v := left_c(c)(yo + y);
                                        when 2 => v := top_c(c)(xo + x);
                                        when others =>
                                            v := clip1(asr(cpl_a(c) + cpl_b(c) * (xo + x - 3) + cpl_c(c) * (yo + y - 3) + 16, 5));
                                    end case;
                                    pp(y * 4 + x) := v;
                                end loop;
                            end loop;
                        end if;
                        for i in 0 to 15 loop
                            if mb.mbtype = MBT_IPCM then
                                rec(i) <= pp(i);
                            else
                                rec(i) <= clip1(pp(i) + to_integer(res(i)));
                            end if;
                        end loop;
                        rc <= (others => '0');
                        st <= R_B_WR;

                    when R_B_WR =>
                        if n < 16 then
                            r := BLK_RASTER(to_integer(n(3 downto 0)));
                            bx := r mod 4;
                            by := r / 4;
                        else
                            c := (to_integer(n) - 16) / 4;
                            b := (to_integer(n) - 16) mod 4;
                            bx := b mod 2;
                            by := b / 2;
                        end if;
                        -- write row rc
                        word := (others => '0');
                        en := (others => '0');
                        for i in 0 to 3 loop
                            if n < 16 then
                                x0 := bx * 4 + i;
                            else
                                x0 := c * 8 + bx * 4 + i;
                            end if;
                            word(x0 * 8 + 7 downto x0 * 8) := std_logic_vector(to_unsigned(rec(to_integer(rc) * 4 + i), 8));
                            en(x0) := '1';
                        end loop;
                        rw_en <= en;
                        rw_data <= word;
                        if n < 16 then
                            rw_addr <= rslot & to_unsigned(by * 4 + to_integer(rc), 5);
                        else
                            rw_addr <= rslot & to_unsigned(16 + by * 4 + to_integer(rc), 5);
                        end if;
                        -- neighbour registers
                        if rc = 0 then
                            if n < 16 then
                                for i in 0 to 3 loop
                                    rowr(by)(bx * 4 + i) <= rec(12 + i);
                                    colr(bx)(by * 4 + i) <= rec(i * 4 + 3);
                                end loop;
                            else
                                for i in 0 to 3 loop
                                    if by = 1 then
                                        crow(c)(bx * 4 + i) <= rec(12 + i);
                                    end if;
                                    if bx = 1 then
                                        ccol(c)(by * 4 + i) <= rec(i * 4 + 3);
                                    end if;
                                end loop;
                            end if;
                        end if;
                        rc <= rc + 1;
                        if rc = 3 then
                            n <= n + 1;
                            st <= R_BLK;
                        end if;

                    when R_END =>
                        if tok_v = '0' or i_mb_ready = '1' then
                            -- line buffer entry and left column for the next macroblock
                            for i in 0 to 15 loop
                                lb_wd(i * 8 + 7 downto i * 8) <= std_logic_vector(to_unsigned(rowr(3)(i), 8));
                                left_l(i) <= colr(3)(i);
                            end loop;
                            for c2 in 0 to 1 loop
                                for i in 0 to 7 loop
                                    lb_wd(128 + c2 * 64 + i * 8 + 7 downto 128 + c2 * 64 + i * 8) <=
                                        std_logic_vector(to_unsigned(crow(c2)(i), 8));
                                    left_c(c2)(i) <= ccol(c2)(i);
                                end loop;
                            end loop;
                            lb_we <= '1';
                            lb_wa <= mb.mb_x;
                            tok <= mb;
                            tok.rec_slot <= rslot;
                            tok_v <= '1';
                            rslot <= not rslot;
                            coef_free <= '1';
                            pred_free <= '1';
                            st <= R_IDLE;
                        end if;
                end case;
            end if;
        end if;
    end process;
end architecture;
