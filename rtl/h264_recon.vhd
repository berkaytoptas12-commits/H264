-- ============================================================================
-- h264_recon.vhd
-- Macroblock reconstruction.
--   * Intra16x16 luma DC (4x4 Hadamard) and chroma DC (2x2 Hadamard) with
--     their dequantisation, once per macroblock
--   * per 4x4 block a 6 stage pipeline, one block per cycle:
--       S0 issue      coefficient block / prediction block address
--       S1            buffer reads (2 cycle latency)
--       S2 dequant    16 coefficients in parallel (scale << qp/6 per MB)
--       S3            DC substitution, inverse transform rows
--       S4            inverse transform columns, Intra4x4 neighbour
--                     selection, plane base, chroma DC prediction value
--       S5 predict    Intra4x4 (9 modes), Intra16x16 (4), chroma (4) or inter
--       S6            add residual, clip, write the 4x4 block to the
--                     deblocking input buffer, update the neighbour registers
--     Intra4x4 luma blocks depend on the previous block (S6 -> S4), so they
--     are issued every third cycle.
--   * I_PCM samples are copied from the coefficient buffer
-- Neighbours: line buffer (unfiltered bottom row of the macroblock row above)
-- and registers for the left column.
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
        -- coefficient buffer: whole 4x4 block per read (row r at bits r*64+63..r*64)
        cr_addr     : out unsigned(6 downto 0);          -- slot & block
        cr_data     : in  std_logic_vector(255 downto 0);
        o_coef_free : out std_logic;
        -- inter prediction buffer: 4x4 block read (pixel i at bits i*8+7..i*8)
        pr_slot     : out std_logic;
        pr_blk      : out unsigned(4 downto 0);          -- 0..15 luma raster, 16..19 Cb, 20..23 Cr
        pr_data     : in  std_logic_vector(127 downto 0);
        o_pred_free : out std_logic;
        -- reconstructed macroblock buffer (block addressed, same numbering)
        rw_en       : out std_logic;
        rw_addr     : out unsigned(5 downto 0);          -- slot & block
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
    signal lb_ram : lb_t;
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


    -- separable halves of the inverse transform (pipeline stages)
    function idct_rows(d : c20_arr16_t) return c20_arr16_t is
        variable t : c20_arr16_t;
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
        return t;
    end function;

    function idct_cols(t : c20_arr16_t) return c20_arr16_t is
        variable r : c20_arr16_t;
        variable e0, e1, e2, e3 : c20_t;
    begin
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


    type state_t is (R_IDLE, R_LB0, R_LB1, R_LB2, R_DC0, R_DC1, R_DC2, R_DC3, R_DC4, R_DC5, R_DC6,
                     R_RUN, R_END);
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
    signal dcl     : c20_arr16_t;
    signal dcc     : c20_arr8_t;
    signal fl      : c20_arr16_t;
    signal fc      : c20_arr8_t;
    signal ldc     : c20_arr16_t;
    signal cdc     : c20_arr8_t;
    signal qrem_l, qdiv_l, qrem_c, qdiv_c : natural range 0 to 15;
    -- dequantisation scale per coefficient position: LevelScale << qp/6
    type scl_arr_t is array (0 to 15) of natural range 0 to 8191;
    signal scl_l, scl_c : scl_arr_t;
    -- DC / plane parameter pipeline (once per macroblock)
    type int8_t is array (0 to 7) of integer range -4096 to 4095;
    type int4x2_t is array (0 to 1) of int8_t;
    type int2_t is array (0 to 1) of integer range -16384 to 16383;
    signal wh, wv  : int8_t;           -- (i+1) * (difference) luma
    signal cwh, cwv : int4x2_t;        -- chroma (entries 0..3)
    signal sum_t, sum_l : natural range 0 to 4095;
    signal h_r, v_r : integer range -32768 to 32767;
    signal ch_r, cv_r : int2_t;
    signal tmp_r   : c20_arr16_t;
    type prod_arr16_t is array (0 to 15) of signed(39 downto 0);
    type prod_arr8_t is array (0 to 7) of signed(39 downto 0);
    signal prod_l  : prod_arr16_t;
    signal prod_c  : prod_arr8_t;

    -- Intra16x16 / chroma prediction parameters
    subtype pint_t is integer range -262144 to 262143;
    type pint4_t is array (0 to 3) of pint_t;
    type pint2_t is array (0 to 1) of pint_t;
    type pint4x2_t is array (0 to 1) of pint4_t;
    type pint2x2_t is array (0 to 1) of pint2_t;
    signal i16_dc  : pix_t;
    signal pl_a, pl_b, pl_c : pint_t := 0;
    signal pbx, pcy, pbm, pcm : pint4_t := (others => 0);    -- b*(4bx-7), c*(4by-7), b*x, c*y
    signal cpl_a, cpl_b, cpl_c : pint2_t := (others => 0);
    signal cpbx, cpcy : pint2x2_t := (others => (others => 0));  -- b*(xo-3), c*(yo-3)
    signal cpbm, cpcm : pint4x2_t := (others => (others => 0));  -- b*x, c*y

    -- block pipeline
    -- pv(k) / pn(k): valid flag / block number of the block in stage S(k+1)
    type nat_arr6_t is array (0 to 5) of natural range 0 to 23;
    signal pv      : std_logic_vector(5 downto 0);
    signal pn      : nat_arr6_t;
    signal issue   : natural range 0 to 24;
    signal pp2, pp3, pp4 : pix16_t;
    signal dq2     : c20_arr16_t;
    signal rt3     : c20_arr16_t;
    signal res4, res5 : c20_arr16_t;
    signal base4   : pint_t := 0;
    signal pred5   : pix16_t;
    signal gap     : natural range 0 to 2;   -- issue spacing for Intra4x4
    -- Intra4x4 neighbours of the block in S5 (selected in S4)
    signal p4_t8   : pix8_t;
    signal p4_l4   : pix4_t;
    signal p4_tl   : pix_t;
    signal p4_at, p4_al : boolean;
    signal p4_mode : natural range 0 to 15;
    signal p4_cs   : pix_t;                  -- chroma DC prediction value

    signal coef_free, pred_free : std_logic;

    -- block number helpers
    function blkr(n : natural) return natural is
    begin
        return BLK_RASTER(n mod 16);
    end function;
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
        variable r, bx, by, c, b, n : integer;
        variable lv     : c20_arr16_t;
        variable t8     : pix8_t;
        variable l4     : pix4_t;
        variable tlv    : pix_t;
        variable at, al, atr : boolean;
        variable pp     : pix16_t;
        variable v, s, h, vv : integer;
        variable f      : c20_arr16_t;
        variable tmp    : c20_arr16_t;
        variable ls     : integer;
        variable w64    : signed(39 downto 0);
        variable word   : std_logic_vector(127 downto 0);
        variable xo, yo, sumt, suml : integer;
        variable cf     : signed(15 downto 0);
        variable prod   : signed(31 downto 0);
        variable a_l, a_t : boolean;
        variable need_c : boolean;
        variable qr, qd : natural;
    begin
        if rising_edge(clk) then
            lb_we <= '0';
            rw_en <= '0';
            coef_free <= '0';
            pred_free <= '0';
            if tok_v = '1' and i_mb_ready = '1' then
                tok_v <= '0';
            end if;
            if i_rec_free = '1' then
                rec_cnt <= rec_cnt - 1;
            end if;

            -- ----------------------------------------------------------
            -- block pipeline (runs while st = R_RUN)
            -- ----------------------------------------------------------
            pv(5 downto 1) <= pv(4 downto 0);
            for i in 1 to 5 loop
                pn(i) <= pn(i - 1);
            end loop;
            pv(0) <= '0';
            if gap /= 0 then
                gap <= gap - 1;
            end if;
            if st = R_RUN and issue < 24 and gap = 0 then
                -- S0: issue block 'issue'
                n := issue;
                if n < 16 then
                    r := blkr(n);
                    cr_addr <= mb.coef_slot & to_unsigned(r, 5);
                    pr_blk <= to_unsigned(r, 5);
                else
                    cr_addr <= mb.coef_slot & to_unsigned(CB_CB + n - 16, 5);
                    pr_blk <= to_unsigned(n, 5);
                end if;
                pr_slot <= mb.pred_slot;
                pv(0) <= '1';
                pn(0) <= n;
                issue <= issue + 1;
                if mb.mbtype = MBT_I4x4 and n < 16 then
                    gap <= 2;
                end if;
            end if;
            -- S1: prediction block read (registered in the buffer, valid in S2)
            -- S2: dequantisation (coefficient block arrives now)
            if pv(1) = '1' then
                n := pn(1);
                if n < 16 then
                    r := blkr(n);
                    need_c := mb.nz(r) = '1';
                else
                    need_c := mb.nz(CB_CB + n - 16) = '1';
                end if;
                for i in 0 to 15 loop
                    cf := signed(cr_data((i / 4) * 64 + (i mod 4) * 16 + 15 downto (i / 4) * 64 + (i mod 4) * 16));
                    if not need_c then
                        cf := (others => '0');
                    end if;
                    if n < 16 then
                        prod := cf * to_signed(scl_l(i), 16);
                    else
                        prod := cf * to_signed(scl_c(i), 16);
                    end if;
                    dq2(i) <= resize(prod, 20);
                    if mb.mbtype = MBT_IPCM then
                        pp2(i) <= to_integer(unsigned(cr_data((i / 4) * 64 + (i mod 4) * 16 + 7 downto (i / 4) * 64 + (i mod 4) * 16)));
                    else
                        pp2(i) <= to_integer(unsigned(pr_data(i * 8 + 7 downto i * 8)));
                    end if;
                end loop;
            end if;
            -- S3: DC substitution, row transform
            if pv(2) = '1' then
                n := pn(2);
                lv := dq2;
                if n < 16 then
                    if mb.mbtype = MBT_I16 then
                        lv(0) := ldc(blkr(n));
                    end if;
                else
                    lv(0) := cdc(n - 16);
                end if;
                rt3 <= idct_rows(lv);
                pp3 <= pp2;
            end if;
            -- S4: column transform, plane base, Intra4x4 neighbours,
            --     chroma DC prediction
            if pv(3) = '1' then
                res4 <= idct_cols(rt3);
                pp4 <= pp3;
                n := pn(3);
                if n < 16 then
                    r := blkr(n);
                    bx := r mod 4;
                    by := r / 4;
                    base4 <= pl_a + pbx(bx) + pcy(by) + 16;
                    -- neighbours (blocks before this one are already in rowr / colr)
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
                    p4_t8 <= t8;
                    p4_l4 <= l4;
                    p4_tl <= tlv;
                    p4_at <= by > 0 or mb.avail_b = '1';
                    p4_al <= bx > 0 or mb.avail_a = '1';
                    p4_mode <= to_integer(mb.i4_modes(r));
                else
                    c := (n - 16) / 4;
                    b := (n - 16) mod 4;
                    base4 <= cpl_a(c) + cpbx(c)(b mod 2) + cpcy(c)(b / 2) + 16;
                    xo := (b mod 2) * 4;
                    yo := (b / 2) * 4;
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
                    p4_cs <= s;
                end if;
            end if;
            -- S5: prediction
            if pv(4) = '1' then
                n := pn(4);
                if n < 16 then
                    r := blkr(n);
                    bx := r mod 4;
                    by := r / 4;
                else
                    c := (n - 16) / 4;
                    b := (n - 16) mod 4;
                    bx := b mod 2;
                    by := b / 2;
                end if;
                pp := pp4;
                if mb.mbtype = MBT_I4x4 and n < 16 then
                    pp := pred4x4(p4_mode, p4_t8, p4_l4, p4_tl, p4_at, p4_al);
                elsif mb.mbtype = MBT_I16 and n < 16 then
                    for y in 0 to 3 loop
                        for x in 0 to 3 loop
                            case to_integer(mb.i16_mode) is
                                when 0 => v := top_l(bx * 4 + x);
                                when 1 => v := left_l(by * 4 + y);
                                when 2 => v := i16_dc;
                                when others => v := clip1(asr(base4 + pbm(x) + pcm(y), 5));
                            end case;
                            pp(y * 4 + x) := v;
                        end loop;
                    end loop;
                elsif mbt_is_intra(mb.mbtype) and mb.mbtype /= MBT_IPCM and n >= 16 then
                    xo := bx * 4;
                    yo := by * 4;
                    for y in 0 to 3 loop
                        for x in 0 to 3 loop
                            case to_integer(mb.chroma_mode) is
                                when 0 => v := p4_cs;
                                when 1 => v := left_c(c)(yo + y);
                                when 2 => v := top_c(c)(xo + x);
                                when others => v := clip1(asr(base4 + cpbm(c)(x) + cpcm(c)(y), 5));
                            end case;
                            pp(y * 4 + x) := v;
                        end loop;
                    end loop;
                end if;
                pred5 <= pp;
                res5 <= res4;
            end if;
            -- S6: residual, clip, write; neighbour registers for the
            --     following blocks / macroblock
            if pv(5) = '1' then
                n := pn(5);
                if n < 16 then
                    r := blkr(n);
                    bx := r mod 4;
                    by := r / 4;
                else
                    c := (n - 16) / 4;
                    b := (n - 16) mod 4;
                    bx := b mod 2;
                    by := b / 2;
                end if;
                for i in 0 to 15 loop
                    if mb.mbtype = MBT_IPCM then
                        pp(i) := pred5(i);
                    else
                        pp(i) := clip1(pred5(i) + to_integer(res5(i)));
                    end if;
                    word(i * 8 + 7 downto i * 8) := std_logic_vector(to_unsigned(pp(i), 8));
                end loop;
                rw_en <= '1';
                rw_data <= word;
                if n < 16 then
                    rw_addr <= rslot & to_unsigned(r, 5);
                    for i in 0 to 3 loop
                        rowr(by)(bx * 4 + i) <= pp(12 + i);
                        colr(bx)(by * 4 + i) <= pp(i * 4 + 3);
                    end loop;
                else
                    rw_addr <= rslot & to_unsigned(n, 5);
                    for i in 0 to 3 loop
                        if by = 1 then
                            crow(c)(bx * 4 + i) <= pp(12 + i);
                        end if;
                        if bx = 1 then
                            ccol(c)(by * 4 + i) <= pp(i * 4 + 3);
                        end if;
                    end loop;
                end if;
            end if;

            -- ----------------------------------------------------------
            -- macroblock control
            -- ----------------------------------------------------------
            if rst = '1' then
                st <= R_IDLE;
                tok_v <= '0';
                rec_cnt <= (others => '0');
                rslot <= '0';
                tok <= MBINFO_INIT;
                cr_addr <= (others => '0');
                pr_slot <= '0';
                pr_blk <= (others => '0');
                pv <= (others => '0');
                issue <= 0;
                gap <= 0;
            else
                case st is
                    when R_IDLE =>
                        issue <= 0;
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
                        for i in 0 to 15 loop
                            scl_l(i) <= dq_v(qrem_l, i) * 2 ** qdiv_l;
                            scl_c(i) <= dq_v(qrem_c, i) * 2 ** qdiv_c;
                        end loop;
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
                        st <= R_DC0;

                    -- DC blocks (2 cycle read latency) and prediction parameters
                    when R_DC0 =>
                        cr_addr <= mb.coef_slot & to_unsigned(CB_LDC, 5);
                        st <= R_DC1;
                    when R_DC1 =>
                        cr_addr <= mb.coef_slot & to_unsigned(CB_CDC, 5);
                        st <= R_DC2;
                    when R_DC2 =>
                        for i in 0 to 15 loop
                            if mb.nz(16) = '1' and mb.mbtype = MBT_I16 then
                                dcl(i) <= resize(signed(cr_data((i / 4) * 64 + (i mod 4) * 16 + 15 downto (i / 4) * 64 + (i mod 4) * 16)), 20);
                            else
                                dcl(i) <= (others => '0');
                            end if;
                        end loop;
                        -- Intra16x16 DC sums, plane gradient terms
                        sumt := 0;
                        suml := 0;
                        for i in 0 to 15 loop
                            sumt := sumt + top_l(i);
                            suml := suml + left_l(i);
                        end loop;
                        sum_t <= sumt;
                        sum_l <= suml;
                        for i in 0 to 7 loop
                            if i = 7 then
                                wh(i) <= (i + 1) * (top_l(8 + i) - tl_l);
                                wv(i) <= (i + 1) * (left_l(8 + i) - tl_l);
                            else
                                wh(i) <= (i + 1) * (top_l(8 + i) - top_l(6 - i));
                                wv(i) <= (i + 1) * (left_l(8 + i) - left_l(6 - i));
                            end if;
                        end loop;
                        for c2 in 0 to 1 loop
                            for i in 0 to 3 loop
                                if i = 3 then
                                    cwh(c2)(i) <= (i + 1) * (top_c(c2)(4 + i) - tl_c(c2));
                                    cwv(c2)(i) <= (i + 1) * (left_c(c2)(4 + i) - tl_c(c2));
                                else
                                    cwh(c2)(i) <= (i + 1) * (top_c(c2)(4 + i) - top_c(c2)(2 - i));
                                    cwv(c2)(i) <= (i + 1) * (left_c(c2)(4 + i) - left_c(c2)(2 - i));
                                end if;
                            end loop;
                        end loop;
                        st <= R_DC3;
                    when R_DC3 =>
                        for i in 0 to 7 loop
                            if mb.nz(17) = '1' then
                                dcc(i) <= resize(signed(cr_data((i / 4) * 64 + (i mod 4) * 16 + 15 downto (i / 4) * 64 + (i mod 4) * 16)), 20);
                            else
                                dcc(i) <= (others => '0');
                            end if;
                        end loop;
                        if mb.avail_a = '1' and mb.avail_b = '1' then
                            i16_dc <= (sum_t + sum_l + 16) / 32;
                        elsif mb.avail_a = '1' then
                            i16_dc <= (sum_l + 8) / 16;
                        elsif mb.avail_b = '1' then
                            i16_dc <= (sum_t + 8) / 16;
                        else
                            i16_dc <= 128;
                        end if;
                        h := 0;
                        vv := 0;
                        for i in 0 to 7 loop
                            h := h + wh(i);
                            vv := vv + wv(i);
                        end loop;
                        h_r <= h;
                        v_r <= vv;
                        for c2 in 0 to 1 loop
                            ch_r(c2) <= cwh(c2)(0) + cwh(c2)(1) + cwh(c2)(2) + cwh(c2)(3);
                            cv_r(c2) <= cwv(c2)(0) + cwv(c2)(1) + cwv(c2)(2) + cwv(c2)(3);
                        end loop;
                        -- luma DC: rows of the 4x4 Hadamard
                        for i in 0 to 3 loop
                            tmp_r(i * 4) <= dcl(i * 4) + dcl(i * 4 + 1) + dcl(i * 4 + 2) + dcl(i * 4 + 3);
                            tmp_r(i * 4 + 1) <= dcl(i * 4) + dcl(i * 4 + 1) - dcl(i * 4 + 2) - dcl(i * 4 + 3);
                            tmp_r(i * 4 + 2) <= dcl(i * 4) - dcl(i * 4 + 1) - dcl(i * 4 + 2) + dcl(i * 4 + 3);
                            tmp_r(i * 4 + 3) <= dcl(i * 4) - dcl(i * 4 + 1) + dcl(i * 4 + 2) - dcl(i * 4 + 3);
                        end loop;
                        st <= R_DC4;
                    when R_DC4 =>
                        pl_a <= 16 * (left_l(15) + top_l(15));
                        pl_b <= asr(5 * h_r + 32, 6);
                        pl_c <= asr(5 * v_r + 32, 6);
                        for c2 in 0 to 1 loop
                            cpl_a(c2) <= 16 * (left_c(c2)(7) + top_c(c2)(7));
                            cpl_b(c2) <= asr(34 * ch_r(c2) + 32, 6);
                            cpl_c(c2) <= asr(34 * cv_r(c2) + 32, 6);
                        end loop;
                        -- luma DC: columns; chroma DC: 2x2 Hadamard per component
                        for j in 0 to 3 loop
                            fl(j) <= tmp_r(j) + tmp_r(4 + j) + tmp_r(8 + j) + tmp_r(12 + j);
                            fl(4 + j) <= tmp_r(j) + tmp_r(4 + j) - tmp_r(8 + j) - tmp_r(12 + j);
                            fl(8 + j) <= tmp_r(j) - tmp_r(4 + j) - tmp_r(8 + j) + tmp_r(12 + j);
                            fl(12 + j) <= tmp_r(j) - tmp_r(4 + j) + tmp_r(8 + j) - tmp_r(12 + j);
                        end loop;
                        for c2 in 0 to 1 loop
                            fc(c2 * 4) <= dcc(c2 * 4) + dcc(c2 * 4 + 1) + dcc(c2 * 4 + 2) + dcc(c2 * 4 + 3);
                            fc(c2 * 4 + 1) <= dcc(c2 * 4) - dcc(c2 * 4 + 1) + dcc(c2 * 4 + 2) - dcc(c2 * 4 + 3);
                            fc(c2 * 4 + 2) <= dcc(c2 * 4) + dcc(c2 * 4 + 1) - dcc(c2 * 4 + 2) - dcc(c2 * 4 + 3);
                            fc(c2 * 4 + 3) <= dcc(c2 * 4) - dcc(c2 * 4 + 1) - dcc(c2 * 4 + 2) + dcc(c2 * 4 + 3);
                        end loop;
                        st <= R_DC5;
                    when R_DC5 =>
                        -- products of the plane gradients with the block offsets
                        for i in 0 to 3 loop
                            pbx(i) <= pl_b * (i * 4 - 7);
                            pcy(i) <= pl_c * (i * 4 - 7);
                            pbm(i) <= pl_b * i;
                            pcm(i) <= pl_c * i;
                        end loop;
                        for c2 in 0 to 1 loop
                            for i in 0 to 1 loop
                                cpbx(c2)(i) <= cpl_b(c2) * (i * 4 - 3);
                                cpcy(c2)(i) <= cpl_c(c2) * (i * 4 - 3);
                            end loop;
                            for i in 0 to 3 loop
                                cpbm(c2)(i) <= cpl_b(c2) * i;
                                cpcm(c2)(i) <= cpl_c(c2) * i;
                            end loop;
                        end loop;
                        -- DC dequantisation products
                        ls := 16 * DEQUANT_TAB(qrem_l * 3);
                        for i in 0 to 15 loop
                            prod_l(i) <= resize(resize(fl(i), 28) * to_signed(ls, 12), 40);
                        end loop;
                        ls := 16 * DEQUANT_TAB(qrem_c * 3);
                        for i in 0 to 7 loop
                            prod_c(i) <= resize(resize(fc(i), 28) * to_signed(ls, 12), 40);
                        end loop;
                        st <= R_DC6;
                    when R_DC6 =>
                        for i in 0 to 15 loop
                            w64 := prod_l(i);
                            if qdiv_l >= 6 then
                                w64 := shift_left(w64, qdiv_l - 6);
                            else
                                w64 := shift_right(w64 + shift_left(to_signed(1, 40), 5 - qdiv_l), 6 - qdiv_l);
                            end if;
                            ldc(i) <= resize(w64, 20);
                        end loop;
                        for i in 0 to 7 loop
                            w64 := shift_right(shift_left(prod_c(i), qdiv_c), 5);
                            cdc(i) <= resize(w64, 20);
                        end loop;
                        issue <= 0;
                        gap <= 0;
                        st <= R_RUN;

                    when R_RUN =>
                        -- the pipeline is drained when the last block was written
                        if issue = 24 and pv = "000000" then
                            st <= R_END;
                        end if;

                    when R_END =>
                        if tok_v = '0' or i_mb_ready = '1' then
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
