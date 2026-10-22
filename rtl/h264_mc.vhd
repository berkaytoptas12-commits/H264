-- ============================================================================
-- h264_mc.vhd
-- Motion compensation (inter prediction) stage.
--   For every partition of an inter macroblock three jobs are run:
--   luma ((w+5) x (h+5) reference window, 6-tap quarter sample
--   interpolation) and Cb / Cr ((w/2+1) x (h/2+1) window, eighth sample
--   bilinear interpolation).  Reference rows are read over an AXI4 read
--   channel (64 bit, one burst of 1..4 beats per window row, INCR, ID 0);
--   samples outside the picture are replaced by the nearest edge sample.
--   Predicted samples are written to the prediction buffer slot of the
--   macroblock; intra macroblocks pass through.
--   The first macroblock of a picture waits until the deblocking stage has
--   finished writing all previous pictures.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.h264_pkg.all;

entity h264_mc is
    generic (
        G_FB_BASE : std_logic_vector(31 downto 0) := x"00000000"
    );
    port (
        clk         : in  std_logic;
        rst         : in  std_logic;
        -- token in / out
        i_mb        : in  mbinfo_t;
        i_mb_valid  : in  std_logic;
        o_mb_ready  : out std_logic;
        o_mb        : out mbinfo_t;
        o_mb_valid  : out std_logic;
        i_mb_ready  : in  std_logic;
        i_pred_free : in  std_logic;
        i_pic_done  : in  std_logic;
        -- prediction buffer write
        pw_en       : out std_logic_vector(15 downto 0);
        pw_addr     : out unsigned(5 downto 0);
        pw_data     : out std_logic_vector(127 downto 0);
        -- AXI4 read master
        m_araddr    : out std_logic_vector(31 downto 0);
        m_arlen     : out std_logic_vector(7 downto 0);
        m_arsize    : out std_logic_vector(2 downto 0);
        m_arburst   : out std_logic_vector(1 downto 0);
        m_arvalid   : out std_logic;
        m_arready   : in  std_logic;
        m_rdata     : in  std_logic_vector(63 downto 0);
        m_rlast     : in  std_logic;
        m_rvalid    : in  std_logic;
        m_rready    : out std_logic
    );
end entity;

architecture rtl of h264_mc is

    subtype pix_t is integer range 0 to 255;
    type win_row_t is array (0 to 20) of pix_t;
    type win_arr_t is array (0 to 5) of win_row_t;
    subtype b1_t is integer range -4096 to 16383;
    type b1_row_t is array (0 to 15) of b1_t;
    type b1_arr_t is array (0 to 5) of b1_row_t;
    type bytes32_t is array (0 to 31) of pix_t;

    -- partition list
    type part_t is record
        x4, y4, w4, h4 : natural range 0 to 4;
    end record;
    type plist_t is array (0 to 15) of part_t;

    type job_t is record
        plane  : natural range 0 to 2;
        fb     : unsigned(4 downto 0);
        x0     : integer range -16384 to 16383;   -- window origin
        y0     : integer range -16384 to 16383;
        ww     : natural range 0 to 21;           -- window size
        wh     : natural range 0 to 21;
        fx     : natural range 0 to 7;
        fy     : natural range 0 to 7;
        px     : natural range 0 to 15;           -- destination in the macroblock
        py     : natural range 0 to 15;
        w      : natural range 0 to 16;           -- output size
        h      : natural range 0 to 16;
        wp     : natural range 0 to 2048;         -- plane size
        hp     : natural range 0 to 2048;
        first  : natural range 0 to 2047;         -- first fetched column
        abase  : natural range 0 to 2047;         -- 8 byte aligned fetch start
        nwords : natural range 1 to 4;
    end record;

    function clipi(lo, hi, v : integer) return integer is
    begin
        if v < lo then
            return lo;
        elsif v > hi then
            return hi;
        end if;
        return v;
    end function;

    function floordiv(v : integer; n : natural) return integer is
    begin
        return to_integer(shift_right(to_signed(v, 32), n));
    end function;

    function job_params(mb : mbinfo_t; pl : plist_t; j : natural) return job_t is
        variable jb   : job_t;
        variable p    : part_t;
        variable mv   : mv_t;
        variable last : integer;
    begin
        p := pl(j / 3);
        jb.plane := j mod 3;
        mv := mb.mv(p.y4 * 4 + p.x4);
        jb.fb := mb.refpic((p.y4 / 2) * 2 + p.x4 / 2);
        if jb.plane = 0 then
            jb.x0 := to_integer(mb.mb_x) * 16 + p.x4 * 4 + floordiv(to_integer(mv.x), 2) - 2;
            jb.y0 := to_integer(mb.mb_y) * 16 + p.y4 * 4 + floordiv(to_integer(mv.y), 2) - 2;
            jb.ww := p.w4 * 4 + 5;
            jb.wh := p.h4 * 4 + 5;
            jb.fx := to_integer(unsigned(mv.x(1 downto 0)));
            jb.fy := to_integer(unsigned(mv.y(1 downto 0)));
            jb.px := p.x4 * 4;
            jb.py := p.y4 * 4;
            jb.w := p.w4 * 4;
            jb.h := p.h4 * 4;
            jb.wp := to_integer(mb.pic_w) * 16;
            jb.hp := to_integer(mb.pic_h) * 16;
        else
            jb.x0 := to_integer(mb.mb_x) * 8 + p.x4 * 2 + floordiv(to_integer(mv.x), 3);
            jb.y0 := to_integer(mb.mb_y) * 8 + p.y4 * 2 + floordiv(to_integer(mv.y), 3);
            jb.ww := p.w4 * 2 + 1;
            jb.wh := p.h4 * 2 + 1;
            jb.fx := to_integer(unsigned(mv.x(2 downto 0)));
            jb.fy := to_integer(unsigned(mv.y(2 downto 0)));
            jb.px := p.x4 * 2;
            jb.py := p.y4 * 2;
            jb.w := p.w4 * 2;
            jb.h := p.h4 * 2;
            jb.wp := to_integer(mb.pic_w) * 8;
            jb.hp := to_integer(mb.pic_h) * 8;
        end if;
        jb.first := clipi(0, jb.wp - 1, jb.x0);
        last := clipi(0, jb.wp - 1, jb.x0 + jb.ww - 1);
        jb.abase := (jb.first / 8) * 8;
        jb.nwords := last / 8 - jb.first / 8 + 1;
        return jb;
    end function;

    function tap6(a, b, c, d, e, f : integer) return integer is
    begin
        return a - 5 * b + 20 * c + 20 * d - 5 * e + f;
    end function;

    function clip1(v : integer) return pix_t is
    begin
        if v < 0 then
            return 0;
        elsif v > 255 then
            return 255;
        end if;
        return v;
    end function;

    -- ------------------------------------------------------------------
    type cst_t is (C_IDLE, C_BARRIER, C_PLIST, C_RUN, C_EMIT);
    signal cst : cst_t;
    signal st_dbg : natural;   -- simulation aid
    signal mb       : mbinfo_t;
    signal pl       : plist_t;
    signal nparts   : natural range 0 to 16;
    signal njobs    : natural range 0 to 48;
    signal pslot    : std_logic;
    signal pred_cnt : unsigned(1 downto 0);
    signal pend_pics : unsigned(3 downto 0);
    signal tok      : mbinfo_t;
    signal tok_v    : std_logic;
    signal run      : std_logic;         -- jobs of the current MB active

    -- fetch
    signal f_job    : natural range 0 to 48;
    signal f_row    : natural range 0 to 21;
    signal f_jb     : job_t;
    signal f_jbv    : std_logic;
    signal arvalid  : std_logic;
    signal araddr   : unsigned(31 downto 0);
    signal arlen    : unsigned(7 downto 0);
    signal outstanding : natural range 0 to 63;

    -- data path
    signal d_job    : natural range 0 to 48;
    signal d_row    : natural range 0 to 21;
    signal d_beat   : natural range 0 to 3;
    signal rowbuf   : bytes32_t;
    signal row_v    : std_logic;          -- aligned window row available
    signal row_d    : win_row_t;
    signal row_b1   : b1_row_t;
    signal row_idx  : natural range 0 to 21;
    signal row_jb   : job_t;
    signal row_last : std_logic;          -- last row of job
    signal win      : win_arr_t;
    signal b1s      : b1_arr_t;
    signal o_v      : std_logic;          -- shift register holds an output row
    signal o_y      : natural range 0 to 15;
    signal o_jb     : job_t;
    signal o_last   : std_logic;
    signal jobs_done : natural range 0 to 48;
    signal row_consumed : std_logic;
begin
    st_dbg <= cst_t'pos(cst);
    o_mb <= tok;
    o_mb_valid <= tok_v;
    o_mb_ready <= '1' when cst = C_IDLE and pred_cnt < 2 else '0';
    m_araddr <= std_logic_vector(araddr);
    m_arlen <= std_logic_vector(arlen);
    m_arsize <= "011";
    m_arburst <= "01";
    m_arvalid <= arvalid;
    m_rready <= '1';

    -- ------------------------------------------------------------------
    -- control
    -- ------------------------------------------------------------------
    process (clk)
        variable n    : natural range 0 to 16;
        variable x8, y8 : natural range 0 to 2;
        variable inc, dec : boolean;
    begin
        if rising_edge(clk) then
            if tok_v = '1' and i_mb_ready = '1' then
                tok_v <= '0';
            end if;
            inc := false;
            dec := i_pic_done = '1';
            if rst = '1' then
                cst <= C_IDLE;
                tok_v <= '0';
                pslot <= '0';
                pred_cnt <= (others => '0');
                pend_pics <= (others => '0');
                run <= '0';
                tok <= MBINFO_INIT;
            else
                if i_pred_free = '1' then
                    pred_cnt <= pred_cnt - 1;
                end if;
                case cst is
                    when C_IDLE =>
                        if i_mb_valid = '1' and pred_cnt < 2 then
                            mb <= i_mb;
                            mb.pred_slot <= pslot;
                            if i_pred_free = '1' then
                                pred_cnt <= pred_cnt;
                            else
                                pred_cnt <= pred_cnt + 1;
                            end if;
                            if i_mb.first_of_pic = '1' then
                                cst <= C_BARRIER;
                            else
                                cst <= C_PLIST;
                            end if;
                        end if;
                    when C_BARRIER =>
                        if pend_pics = 0 or (pend_pics = 1 and dec) then
                            cst <= C_PLIST;
                        end if;
                    when C_PLIST =>
                        n := 0;
                        pl <= (others => (0, 0, 0, 0));
                        if mb.mbtype = MBT_PSKIP or (mb.mbtype = MBT_P and mb.part = 0) then
                            pl(0) <= (0, 0, 4, 4);
                            n := 1;
                        elsif mb.mbtype = MBT_P and mb.part = 1 then
                            pl(0) <= (0, 0, 4, 2);
                            pl(1) <= (0, 2, 4, 2);
                            n := 2;
                        elsif mb.mbtype = MBT_P and mb.part = 2 then
                            pl(0) <= (0, 0, 2, 4);
                            pl(1) <= (2, 0, 2, 4);
                            n := 2;
                        elsif mb.mbtype = MBT_P then
                            for i in 0 to 3 loop
                                x8 := (i mod 2) * 2;
                                y8 := (i / 2) * 2;
                                case to_integer(mb.sub(i)) is
                                    when 0 =>
                                        pl(n) <= (x8, y8, 2, 2);
                                        n := n + 1;
                                    when 1 =>
                                        pl(n) <= (x8, y8, 2, 1);
                                        pl(n + 1) <= (x8, y8 + 1, 2, 1);
                                        n := n + 2;
                                    when 2 =>
                                        pl(n) <= (x8, y8, 1, 2);
                                        pl(n + 1) <= (x8 + 1, y8, 1, 2);
                                        n := n + 2;
                                    when others =>
                                        pl(n) <= (x8, y8, 1, 1);
                                        pl(n + 1) <= (x8 + 1, y8, 1, 1);
                                        pl(n + 2) <= (x8, y8 + 1, 1, 1);
                                        pl(n + 3) <= (x8 + 1, y8 + 1, 1, 1);
                                        n := n + 4;
                                end case;
                            end loop;
                        end if;
                        nparts <= n;
                        njobs <= n * 3;
                        if n = 0 then
                            cst <= C_EMIT;
                        else
                            run <= '1';
                            cst <= C_RUN;
                        end if;
                    when C_RUN =>
                        if jobs_done = njobs then
                            run <= '0';
                            cst <= C_EMIT;
                        end if;
                    when C_EMIT =>
                        if tok_v = '0' or i_mb_ready = '1' then
                            tok <= mb;
                            tok_v <= '1';
                            pslot <= not pslot;
                            if mb.last_of_pic = '1' then
                                inc := true;
                            end if;
                            cst <= C_IDLE;
                        end if;
                end case;
                if inc and not dec then
                    pend_pics <= pend_pics + 1;
                elsif dec and not inc then
                    pend_pics <= pend_pics - 1;
                end if;
            end if;
        end if;
    end process;

    -- ------------------------------------------------------------------
    -- fetch: one AXI burst per window row
    -- ------------------------------------------------------------------
    process (clk)
        variable y   : integer;
        variable a   : unsigned(31 downto 0);
        variable jb  : job_t;
        variable issued, retired : boolean;
    begin
        if rising_edge(clk) then
            issued := false;
            retired := row_consumed = '1';
            if rst = '1' or run = '0' then
                arvalid <= '0';
                f_job <= 0;
                f_row <= 0;
                f_jbv <= '0';
                if rst = '1' then
                    outstanding <= 0;
                end if;
            else
                if arvalid = '1' and m_arready = '1' then
                    arvalid <= '0';
                end if;
                if f_jbv = '0' then
                    if f_job < njobs then
                        f_jb <= job_params(mb, pl, f_job);
                        f_jbv <= '1';
                        f_row <= 0;
                    end if;
                elsif (arvalid = '0' or m_arready = '1') and outstanding < 24 then
                    jb := f_jb;
                    y := clipi(0, jb.hp - 1, jb.y0 + f_row);
                    a := fb_plane_addr(unsigned(G_FB_BASE), jb.fb, jb.plane,
                                       to_unsigned(jb.abase, 11), to_unsigned(y, 11));
                    araddr <= a;
                    arlen <= to_unsigned(jb.nwords - 1, 8);
                    arvalid <= '1';
                    issued := true;
                    if f_row = jb.wh - 1 then
                        f_jbv <= '0';
                        f_job <= f_job + 1;
                    else
                        f_row <= f_row + 1;
                    end if;
                end if;
            end if;
            if rst = '0' then
                if issued and not retired then
                    outstanding <= outstanding + 1;
                elsif retired and not issued then
                    outstanding <= outstanding - 1;
                end if;
            end if;
        end if;
    end process;

    -- ------------------------------------------------------------------
    -- read data: assemble window rows
    -- ------------------------------------------------------------------
    process (clk)
        variable rb  : bytes32_t;
        variable wr  : win_row_t;
        variable xs  : integer;
        variable jb  : job_t;
    begin
        if rising_edge(clk) then
            row_v <= '0';
            row_consumed <= '0';
            if rst = '1' or run = '0' then
                d_job <= 0;
                d_row <= 0;
                d_beat <= 0;
            else
                if m_rvalid = '1' then
                    rb := rowbuf;
                    for i in 0 to 7 loop
                        rb(d_beat * 8 + i) := to_integer(unsigned(m_rdata(i * 8 + 7 downto i * 8)));
                    end loop;
                    rowbuf <= rb;
                    jb := job_params(mb, pl, d_job);
                    if d_beat = jb.nwords - 1 then
                        -- complete row: align and clamp
                        for c in 0 to 20 loop
                            xs := clipi(0, jb.wp - 1, jb.x0 + c) - jb.abase;
                            if xs < 0 or xs > 31 then
                                xs := 0;
                            end if;
                            wr(c) := rb(xs);
                        end loop;
                        row_d <= wr;
                        for x in 0 to 15 loop
                            row_b1(x) <= tap6(wr(x), wr(x + 1), wr(x + 2), wr(x + 3), wr(x + 4), wr(x + 5));
                        end loop;
                        row_v <= '1';
                        row_idx <= d_row;
                        row_jb <= jb;
                        row_last <= to_sl(d_row = jb.wh - 1);
                        row_consumed <= '1';
                        d_beat <= 0;
                        if d_row = jb.wh - 1 then
                            d_row <= 0;
                            d_job <= d_job + 1;
                        else
                            d_row <= d_row + 1;
                        end if;
                    else
                        d_beat <= d_beat + 1;
                    end if;
                end if;
            end if;
        end if;
    end process;

    -- ------------------------------------------------------------------
    -- interpolation: rows stream through a 6 row window
    -- ------------------------------------------------------------------
    process (clk)
        variable jb      : job_t;
        variable gg, hh, mm, bb, ss, h1v, m1v, j1v : integer;
        variable bh, hv, mv2, jv, sv : integer;
        variable v       : integer;
        variable word    : std_logic_vector(127 downto 0);
        variable en      : std_logic_vector(15 downto 0);
        variable xa, ya  : integer;
        variable aa, b2, cc, dd : integer;
        variable bank    : integer;
    begin
        if rising_edge(clk) then
            pw_en <= (others => '0');
            o_v <= '0';
            if rst = '1' or run = '0' then
                jobs_done <= 0;
            else
                -- shift a new row in
                if row_v = '1' then
                    for r in 0 to 4 loop
                        win(r) <= win(r + 1);
                        b1s(r) <= b1s(r + 1);
                    end loop;
                    win(5) <= row_d;
                    b1s(5) <= row_b1;
                    o_jb <= row_jb;
                    o_last <= row_last;
                    if row_jb.plane = 0 then
                        if row_idx >= 5 then
                            o_v <= '1';
                            o_y <= row_idx - 5;
                        end if;
                    else
                        if row_idx >= 1 then
                            o_v <= '1';
                            o_y <= row_idx - 1;
                        end if;
                    end if;
                end if;
                -- produce one prediction row
                if o_v = '1' then
                    jb := o_jb;
                    word := (others => '0');
                    en := (others => '0');
                    for x in 0 to 15 loop
                        if x < jb.w then
                            if jb.plane = 0 then
                                gg := win(2)(x + 2);
                                hh := win(2)(x + 3);
                                mm := win(3)(x + 2);
                                bh := clip1(floordiv(b1s(2)(x) + 16, 5));
                                sv := clip1(floordiv(b1s(3)(x) + 16, 5));
                                h1v := tap6(win(0)(x + 2), win(1)(x + 2), win(2)(x + 2), win(3)(x + 2), win(4)(x + 2), win(5)(x + 2));
                                m1v := tap6(win(0)(x + 3), win(1)(x + 3), win(2)(x + 3), win(3)(x + 3), win(4)(x + 3), win(5)(x + 3));
                                j1v := tap6(b1s(0)(x), b1s(1)(x), b1s(2)(x), b1s(3)(x), b1s(4)(x), b1s(5)(x));
                                hv := clip1(floordiv(h1v + 16, 5));
                                mv2 := clip1(floordiv(m1v + 16, 5));
                                jv := clip1(floordiv(j1v + 512, 10));
                                case jb.fy * 4 + jb.fx is
                                    when 0 => v := gg;
                                    when 1 => v := (gg + bh + 1) / 2;
                                    when 2 => v := bh;
                                    when 3 => v := (hh + bh + 1) / 2;
                                    when 4 => v := (gg + hv + 1) / 2;
                                    when 5 => v := (bh + hv + 1) / 2;
                                    when 6 => v := (bh + jv + 1) / 2;
                                    when 7 => v := (bh + mv2 + 1) / 2;
                                    when 8 => v := hv;
                                    when 9 => v := (hv + jv + 1) / 2;
                                    when 10 => v := jv;
                                    when 11 => v := (jv + mv2 + 1) / 2;
                                    when 12 => v := (mm + hv + 1) / 2;
                                    when 13 => v := (hv + sv + 1) / 2;
                                    when 14 => v := (jv + sv + 1) / 2;
                                    when others => v := (mv2 + sv + 1) / 2;
                                end case;
                                bank := jb.px + x;
                            else
                                -- chroma: rows win(4) (y) and win(5) (y+1)
                                aa := win(4)(x);
                                b2 := win(4)(x + 1);
                                cc := win(5)(x);
                                dd := win(5)(x + 1);
                                v := ((8 - jb.fx) * (8 - jb.fy) * aa + jb.fx * (8 - jb.fy) * b2 +
                                      (8 - jb.fx) * jb.fy * cc + jb.fx * jb.fy * dd + 32) / 64;
                                bank := (jb.plane - 1) * 8 + jb.px + x;
                            end if;
                            if bank <= 15 then
                                word(bank * 8 + 7 downto bank * 8) := std_logic_vector(to_unsigned(v, 8));
                                en(bank) := '1';
                            end if;
                        end if;
                    end loop;
                    pw_en <= en;
                    pw_data <= word;
                    if jb.plane = 0 then
                        pw_addr <= mb.pred_slot & to_unsigned(jb.py + o_y, 5);
                    else
                        pw_addr <= mb.pred_slot & to_unsigned(16 + jb.py + o_y, 5);
                    end if;
                    if o_last = '1' then
                        jobs_done <= jobs_done + 1;
                    end if;
                end if;
            end if;
        end if;
    end process;
end architecture;
