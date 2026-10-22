-- ============================================================================
-- h264_deblock.vhd
-- Deblocking filter and frame store writer.
--
-- Working arrays (registers):
--   luma   20 x 20: rows 0..3  = bottom rows of the macroblock above,
--                   cols 0..3  = right columns of the left macroblock,
--                   rows/cols 4..19 = current macroblock
--   chroma 12 x 12 per component, same layout with 4 neighbour rows/cols.
-- Vertical edges are filtered line by line (rows), then horizontal edges
-- (columns); every line passes a pipeline with one stage per edge, which is
-- equivalent to the standard's edge order because lines are independent.
--
-- Output: after filtering a macroblock the region shifted by (-4,-4) is
-- final and is written to the frame buffer (AXI4, 64 bit, one burst per
-- row, byte strobes).  At the right / bottom picture border the region
-- grows to include the remaining samples.  Rows 12..15 / columns 12..15 not
-- yet final are kept in line buffers (top) and in the array (left).
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.h264_pkg.all;
use work.h264_tables_pkg.all;

entity h264_deblock is
    generic (
        G_FB_BASE : std_logic_vector(31 downto 0) := x"00000000"
    );
    port (
        clk          : in  std_logic;
        rst          : in  std_logic;
        -- token in
        i_mb         : in  mbinfo_t;
        i_mb_valid   : in  std_logic;
        o_mb_ready   : out std_logic;
        -- reconstructed macroblock buffer
        rr_addr      : out unsigned(5 downto 0);
        rr_data      : in  std_logic_vector(127 downto 0);
        o_rec_free   : out std_logic;
        -- picture written
        o_pic_done   : out std_logic;
        o_pic_fb     : out unsigned(4 downto 0);
        -- AXI4 write master
        m_awaddr     : out std_logic_vector(31 downto 0);
        m_awlen      : out std_logic_vector(7 downto 0);
        m_awsize     : out std_logic_vector(2 downto 0);
        m_awburst    : out std_logic_vector(1 downto 0);
        m_awvalid    : out std_logic;
        m_awready    : in  std_logic;
        m_wdata      : out std_logic_vector(63 downto 0);
        m_wstrb      : out std_logic_vector(7 downto 0);
        m_wlast      : out std_logic;
        m_wvalid     : out std_logic;
        m_wready     : in  std_logic;
        m_bvalid     : in  std_logic;
        m_bready     : out std_logic
    );
end entity;

architecture rtl of h264_deblock is

    subtype pix_t is integer range 0 to 255;
    type line20_t is array (0 to 19) of pix_t;
    type line12_t is array (0 to 11) of pix_t;
    type ly_t is array (0 to 19) of line20_t;          -- ly(row)(col)
    type lc_t is array (0 to 11) of line12_t;
    type lc2_t is array (0 to 1) of lc_t;

    type nat4_t is array (0 to 3) of natural range 0 to 255;
    type edge_par_t is record
        en    : std_logic;
        alpha : natural range 0 to 255;
        beta  : natural range 0 to 31;
        tc0   : nat4_t;            -- index = bS (1..3)
    end record;
    type epar_arr_t is array (0 to 7) of edge_par_t;   -- dir*4 + edge

    function iabs(v : integer) return integer is
    begin
        if v < 0 then
            return -v;
        end if;
        return v;
    end function;

    -- floor(v / 2**n)
    function asr(v : integer; n : natural) return integer is
    begin
        return to_integer(shift_right(to_signed(v, 24), n));
    end function;

    function clipi(lo, hi, v : integer) return integer is
    begin
        if v < lo then
            return lo;
        elsif v > hi then
            return hi;
        end if;
        return v;
    end function;

    -- luma edge at line position 4e+4 (p3 = 4e ... q3 = 4e+7)
    function filt_luma(l : line20_t; e : natural; bs : natural; ep : edge_par_t) return line20_t is
        variable o : line20_t;
        variable p0, p1, p2, p3, q0, q1, q2, q3 : integer;
        variable ap, aq, tc, dlt, tc0 : integer;
        variable strong : boolean;
        variable b : natural;
    begin
        o := l;
        b := 4 * e;
        p3 := l(b); p2 := l(b + 1); p1 := l(b + 2); p0 := l(b + 3);
        q0 := l(b + 4); q1 := l(b + 5); q2 := l(b + 6); q3 := l(b + 7);
        if ep.en = '1' and bs /= 0 and iabs(p0 - q0) < ep.alpha and iabs(p1 - p0) < ep.beta and
           iabs(q1 - q0) < ep.beta then
            ap := iabs(p2 - p0);
            aq := iabs(q2 - q0);
            if bs < 4 then
                tc0 := ep.tc0(bs);
                tc := tc0;
                if ap < ep.beta then tc := tc + 1; end if;
                if aq < ep.beta then tc := tc + 1; end if;
                dlt := clipi(-tc, tc, asr(4 * (q0 - p0) + (p1 - q1) + 4, 3));
                o(b + 3) := clipi(0, 255, p0 + dlt);
                o(b + 4) := clipi(0, 255, q0 - dlt);
                if ap < ep.beta then
                    o(b + 2) := p1 + clipi(-tc0, tc0, asr(p2 + (p0 + q0 + 1) / 2 - 2 * p1, 1));
                end if;
                if aq < ep.beta then
                    o(b + 5) := q1 + clipi(-tc0, tc0, asr(q2 + (p0 + q0 + 1) / 2 - 2 * q1, 1));
                end if;
            else
                strong := iabs(p0 - q0) < (ep.alpha / 4 + 2);
                if ap < ep.beta and strong then
                    o(b + 3) := (p2 + 2 * p1 + 2 * p0 + 2 * q0 + q1 + 4) / 8;
                    o(b + 2) := (p2 + p1 + p0 + q0 + 2) / 4;
                    o(b + 1) := (2 * p3 + 3 * p2 + p1 + p0 + q0 + 4) / 8;
                else
                    o(b + 3) := (2 * p1 + p0 + q1 + 2) / 4;
                end if;
                if aq < ep.beta and strong then
                    o(b + 4) := (p1 + 2 * p0 + 2 * q0 + 2 * q1 + q2 + 4) / 8;
                    o(b + 5) := (p0 + q0 + q1 + q2 + 2) / 4;
                    o(b + 6) := (2 * q3 + 3 * q2 + q1 + q0 + p0 + 4) / 8;
                else
                    o(b + 4) := (2 * q1 + q0 + p1 + 2) / 4;
                end if;
            end if;
        end if;
        return o;
    end function;

    -- chroma edge at line position 4 (e=0) or 8 (e=1): p1 p0 | q0 q1
    function filt_chroma(l : line12_t; e : natural; bs : natural; ep : edge_par_t) return line12_t is
        variable o : line12_t;
        variable p0, p1, q0, q1, tc, dlt, t : integer;
        variable b : natural;
    begin
        o := l;
        b := 4 + 4 * e;
        p1 := l(b - 2); p0 := l(b - 1); q0 := l(b); q1 := l(b + 1);
        if ep.en = '1' and bs /= 0 and iabs(p0 - q0) < ep.alpha and iabs(p1 - p0) < ep.beta and
           iabs(q1 - q0) < ep.beta then
            if bs < 4 then
                tc := ep.tc0(bs) + 1;
                t := 4 * (q0 - p0) + (p1 - q1) + 4;
                dlt := clipi(-tc, tc, asr(t, 3));
                o(b - 1) := clipi(0, 255, p0 + dlt);
                o(b) := clipi(0, 255, q0 - dlt);
            else
                o(b - 1) := (2 * p1 + p0 + q1 + 2) / 4;
                o(b) := (2 * q1 + q0 + p1 + 2) / 4;
            end if;
        end if;
        return o;
    end function;

    -- ------------------------------------------------------------------
    -- line buffers: main (cols 0..11 luma / 0..3 chroma), corner (rest)
    -- address = mb_x*4 + row
    -- ------------------------------------------------------------------
    type lbm_t is array (0 to C_MAX_MB_W * 4 - 1) of std_logic_vector(159 downto 0);
    type lbc_t is array (0 to C_MAX_MB_W * 4 - 1) of std_logic_vector(95 downto 0);
    signal lbm : lbm_t;
    signal lbc : lbc_t;
    signal lbm_we, lbc_we : std_logic;
    signal lbm_wa, lbc_wa, lb_ra : unsigned(8 downto 0);
    signal lbm_wd : std_logic_vector(159 downto 0);
    signal lbc_wd : std_logic_vector(95 downto 0);
    signal lbm_rd : std_logic_vector(159 downto 0);
    signal lbc_rd : std_logic_vector(95 downto 0);

    type state_t is (D_IDLE, D_LOAD, D_FILT, D_LBW, D_OUT_ROW, D_OUT_W, D_SHIFT, D_PICWAIT);
    signal st : state_t;
    signal st_dbg : natural;

    signal mb    : mbinfo_t;
    signal ay    : ly_t;
    signal ac    : lc2_t;
    signal ep_l  : epar_arr_t;      -- luma edge parameters
    signal ep_c  : epar_arr_t;      -- chroma (edges 0 and 2 used)
    signal cnt   : natural range 0 to 63;

    -- luma pipeline
    type lpl_t is array (0 to 4) of line20_t;
    signal lp_line : lpl_t;
    signal lp_v    : std_logic_vector(4 downto 0);
    type idx5_t is array (0 to 4) of natural range 0 to 15;
    signal lp_idx  : idx5_t;
    signal lfeed   : natural range 0 to 16;
    -- chroma pipeline
    type cpl_t is array (0 to 2) of line12_t;
    signal cp_line : cpl_t;
    signal cp_v    : std_logic_vector(2 downto 0);
    type idx3_t is array (0 to 2) of natural range 0 to 15;
    signal cp_idx  : idx3_t;      -- comp*8 + line
    signal cfeed   : natural range 0 to 16;
    signal pass    : natural range 0 to 2;    -- 0 vertical, 1 horizontal

    -- output
    signal orow    : natural range 0 to 63;  -- row list index
    signal o_plane : natural range 0 to 2;
    signal o_r     : natural range 0 to 19;
    signal o_beat  : natural range 0 to 3;
    signal o_nb    : natural range 1 to 4;
    signal o_wbase : integer;                -- frame x of first word
    signal o_fx0   : integer;                -- frame x of array column 0
    signal o_c0, o_c1 : natural range 0 to 19;
    signal awvalid, wvalid : std_logic;
    signal awaddr  : unsigned(31 downto 0);
    signal awlen   : unsigned(7 downto 0);
    signal wdata   : std_logic_vector(63 downto 0);
    signal wstrb   : std_logic_vector(7 downto 0);
    signal wlast   : std_logic;
    signal bcnt    : natural range 0 to 255;  -- outstanding write responses
    signal pic_done : std_logic;
    signal rec_free : std_logic;
begin
    st_dbg <= state_t'pos(st);
    o_mb_ready <= '1' when st = D_IDLE else '0';
    o_rec_free <= rec_free;
    o_pic_done <= pic_done;
    o_pic_fb <= mb.cur_fb;
    m_awaddr <= std_logic_vector(awaddr);
    m_awlen <= std_logic_vector(awlen);
    m_awsize <= "011";
    m_awburst <= "01";
    m_awvalid <= awvalid;
    m_wdata <= wdata;
    m_wstrb <= wstrb;
    m_wlast <= wlast;
    m_wvalid <= wvalid;
    m_bready <= '1';

    process (clk)
    begin
        if rising_edge(clk) then
            if lbm_we = '1' then
                lbm(to_integer(lbm_wa)) <= lbm_wd;
            end if;
            if lbc_we = '1' then
                lbc(to_integer(lbc_wa)) <= lbc_wd;
            end if;
            lbm_rd <= lbm(to_integer(lb_ra));
            lbc_rd <= lbc(to_integer(lb_ra));
        end if;
    end process;

    process (clk)
        variable ln20   : line20_t;
        variable ln12   : line12_t;
        variable li, k  : natural;
        variable bsv    : natural;
        variable e      : natural;
        variable qpav, ia, ib : integer;
        variable qp_p   : integer;
        variable w      : std_logic_vector(127 downto 0);
        variable comp   : natural;
        variable r0, r1, c0, c1 : natural;
        variable nrows  : natural;
        variable fx0, fy, fx_first, fx_last, x, c : integer;
        variable a      : unsigned(31 downto 0);
        variable dat    : std_logic_vector(63 downto 0);
        variable stb    : std_logic_vector(7 downto 0);
        variable resp   : boolean;
        variable aw_hs  : boolean;
        variable plane_rows : natural;
        variable xlast_mb, ylast_mb : boolean;
    begin
        if rising_edge(clk) then
            lbm_we <= '0';
            lbc_we <= '0';
            rec_free <= '0';
            pic_done <= '0';
            resp := m_bvalid = '1';
            aw_hs := awvalid = '1' and m_awready = '1';
            if aw_hs then
                awvalid <= '0';
            end if;
            if rst = '1' then
                st <= D_IDLE;
                awvalid <= '0';
                wvalid <= '0';
                bcnt <= 0;
                lp_v <= (others => '0');
                cp_v <= (others => '0');
            else
                xlast_mb := resize(mb.mb_x, 8) = mb.pic_w - 1;
                ylast_mb := resize(mb.mb_y, 8) = mb.pic_h - 1;
                case st is
                    when D_IDLE =>
                        if i_mb_valid = '1' then
                            mb <= i_mb;
                            cnt <= 0;
                            st <= D_LOAD;
                            -- edge parameters
                            for d in 0 to 1 loop
                                for ed in 0 to 3 loop
                                    -- luma
                                    if ed = 0 then
                                        if d = 0 then
                                            qp_p := to_integer(i_mb.qp_left);
                                            ep_l(d * 4 + ed).en <= i_mb.filt_left;
                                            ep_c(d * 4 + ed).en <= i_mb.filt_left;
                                        else
                                            qp_p := to_integer(i_mb.qp_top);
                                            ep_l(d * 4 + ed).en <= i_mb.filt_top;
                                            ep_c(d * 4 + ed).en <= i_mb.filt_top;
                                        end if;
                                    else
                                        qp_p := to_integer(i_mb.qp_db);
                                        ep_l(d * 4 + ed).en <= i_mb.filt_int;
                                        ep_c(d * 4 + ed).en <= i_mb.filt_int;
                                    end if;
                                    qpav := (qp_p + to_integer(i_mb.qp_db) + 1) / 2;
                                    ia := clipi(0, 51, qpav + to_integer(i_mb.alpha_off));
                                    ib := clipi(0, 51, qpav + to_integer(i_mb.beta_off));
                                    ep_l(d * 4 + ed).alpha <= ALPHA_TAB(ia);
                                    ep_l(d * 4 + ed).beta <= BETA_TAB(ib);
                                    ep_l(d * 4 + ed).tc0(0) <= 0;
                                    for b in 1 to 3 loop
                                        ep_l(d * 4 + ed).tc0(b) <= TC0_TAB(ia * 3 + b - 1);
                                    end loop;
                                    -- chroma
                                    if ed = 0 then
                                        if d = 0 then
                                            qp_p := to_integer(i_mb.qpc_left);
                                        else
                                            qp_p := to_integer(i_mb.qpc_top);
                                        end if;
                                    else
                                        qp_p := to_integer(i_mb.qpc_db);
                                    end if;
                                    qpav := (qp_p + to_integer(i_mb.qpc_db) + 1) / 2;
                                    ia := clipi(0, 51, qpav + to_integer(i_mb.alpha_off));
                                    ib := clipi(0, 51, qpav + to_integer(i_mb.beta_off));
                                    ep_c(d * 4 + ed).alpha <= ALPHA_TAB(ia);
                                    ep_c(d * 4 + ed).beta <= BETA_TAB(ib);
                                    ep_c(d * 4 + ed).tc0(0) <= 0;
                                    for b in 1 to 3 loop
                                        ep_c(d * 4 + ed).tc0(b) <= TC0_TAB(ia * 3 + b - 1);
                                    end loop;
                                end loop;
                            end loop;
                        end if;

                    -- ------------------------------------------------
                    -- load macroblock (24 rows) and top neighbours (4 rows)
                    -- ------------------------------------------------
                    when D_LOAD =>
                        if cnt < 24 then
                            rr_addr <= mb.rec_slot & to_unsigned(cnt, 5);
                        end if;
                        if cnt < 4 then
                            lb_ra <= resize(mb.mb_x, 7) & to_unsigned(cnt, 2);
                        end if;
                        -- 2 cycle read latency: row cnt-2 arrives now
                        if cnt >= 2 and cnt <= 25 then
                            w := rr_data;
                            if cnt <= 17 then
                                for i in 0 to 15 loop
                                    ay(4 + cnt - 2)(4 + i) <= to_integer(unsigned(w(i * 8 + 7 downto i * 8)));
                                end loop;
                            else
                                for i in 0 to 7 loop
                                    ac(0)(4 + cnt - 18)(4 + i) <= to_integer(unsigned(w(i * 8 + 7 downto i * 8)));
                                    ac(1)(4 + cnt - 18)(4 + i) <= to_integer(unsigned(w(64 + i * 8 + 7 downto 64 + i * 8)));
                                end loop;
                            end if;
                        end if;
                        if cnt >= 2 and cnt <= 5 then
                            for i in 0 to 11 loop
                                ay(cnt - 2)(4 + i) <= to_integer(unsigned(lbm_rd(i * 8 + 7 downto i * 8)));
                            end loop;
                            for i in 0 to 3 loop
                                ay(cnt - 2)(16 + i) <= to_integer(unsigned(lbc_rd(i * 8 + 7 downto i * 8)));
                                ac(0)(cnt - 2)(4 + i) <= to_integer(unsigned(lbm_rd(96 + i * 8 + 7 downto 96 + i * 8)));
                                ac(1)(cnt - 2)(4 + i) <= to_integer(unsigned(lbm_rd(128 + i * 8 + 7 downto 128 + i * 8)));
                                ac(0)(cnt - 2)(8 + i) <= to_integer(unsigned(lbc_rd(32 + i * 8 + 7 downto 32 + i * 8)));
                                ac(1)(cnt - 2)(8 + i) <= to_integer(unsigned(lbc_rd(64 + i * 8 + 7 downto 64 + i * 8)));
                            end loop;
                        end if;
                        cnt <= cnt + 1;
                        if cnt = 25 then
                            rec_free <= '1';
                            pass <= 0;
                            lfeed <= 0;
                            cfeed <= 0;
                            st <= D_FILT;
                        end if;

                    -- ------------------------------------------------
                    -- filtering passes (vertical edges, then horizontal)
                    -- ------------------------------------------------
                    when D_FILT =>
                        -- luma pipeline
                        for s in 4 downto 1 loop
                            lp_line(s) <= filt_luma(lp_line(s - 1), s - 1,
                                                    to_integer(mb.bs(pass * 16 + (s - 1) * 4 + lp_idx(s - 1) / 4)),
                                                    ep_l(pass * 4 + s - 1));
                            lp_v(s) <= lp_v(s - 1);
                            lp_idx(s) <= lp_idx(s - 1);
                        end loop;
                        if lfeed < 16 then
                            for i in 0 to 19 loop
                                if pass = 0 then
                                    ln20(i) := ay(4 + lfeed)(i);
                                else
                                    ln20(i) := ay(i)(4 + lfeed);
                                end if;
                            end loop;
                            lp_line(0) <= ln20;
                            lp_v(0) <= '1';
                            lp_idx(0) <= lfeed;
                            lfeed <= lfeed + 1;
                        else
                            lp_v(0) <= '0';
                        end if;
                        if lp_v(4) = '1' then
                            for i in 0 to 19 loop
                                if pass = 0 then
                                    ay(4 + lp_idx(4))(i) <= lp_line(4)(i);
                                else
                                    ay(i)(4 + lp_idx(4)) <= lp_line(4)(i);
                                end if;
                            end loop;
                        end if;
                        -- chroma pipeline (Cb lines 0..7, Cr lines 8..15)
                        cp_line(1) <= filt_chroma(cp_line(0), 0,
                                                  to_integer(mb.bs(pass * 16 + 0 * 4 + (cp_idx(0) mod 8) / 2)),
                                                  ep_c(pass * 4 + 0));
                        cp_line(2) <= filt_chroma(cp_line(1), 1,
                                                  to_integer(mb.bs(pass * 16 + 2 * 4 + (cp_idx(1) mod 8) / 2)),
                                                  ep_c(pass * 4 + 2));
                        cp_v(2 downto 1) <= cp_v(1 downto 0);
                        cp_idx(1) <= cp_idx(0);
                        cp_idx(2) <= cp_idx(1);
                        if cfeed < 16 then
                            comp := cfeed / 8;
                            for i in 0 to 11 loop
                                if pass = 0 then
                                    ln12(i) := ac(comp)(4 + cfeed mod 8)(i);
                                else
                                    ln12(i) := ac(comp)(i)(4 + cfeed mod 8);
                                end if;
                            end loop;
                            cp_line(0) <= ln12;
                            cp_v(0) <= '1';
                            cp_idx(0) <= cfeed;
                            cfeed <= cfeed + 1;
                        else
                            cp_v(0) <= '0';
                        end if;
                        if cp_v(2) = '1' then
                            comp := cp_idx(2) / 8;
                            for i in 0 to 11 loop
                                if pass = 0 then
                                    ac(comp)(4 + cp_idx(2) mod 8)(i) <= cp_line(2)(i);
                                else
                                    ac(comp)(i)(4 + cp_idx(2) mod 8) <= cp_line(2)(i);
                                end if;
                            end loop;
                        end if;
                        -- pass complete when both pipelines drained
                        if lfeed = 16 and cfeed = 16 and lp_v(3 downto 0) = "0000" and cp_v(1 downto 0) = "00" then
                            lp_v(4) <= '0';
                            cp_v(2) <= '0';
                            if pass = 0 then
                                pass <= 1;
                                lfeed <= 0;
                                cfeed <= 0;
                            else
                                cnt <= 0;
                                st <= D_LBW;
                            end if;
                        end if;

                    -- ------------------------------------------------
                    -- line buffer update
                    -- ------------------------------------------------
                    when D_LBW =>
                        k := cnt mod 4;
                        if cnt < 4 then
                            -- main entry of this column: rows 12..15, luma cols 0..11, chroma 0..3
                            for i in 0 to 11 loop
                                lbm_wd(i * 8 + 7 downto i * 8) <= std_logic_vector(to_unsigned(ay(16 + k)(4 + i), 8));
                            end loop;
                            for i in 0 to 3 loop
                                lbm_wd(96 + i * 8 + 7 downto 96 + i * 8) <= std_logic_vector(to_unsigned(ac(0)(8 + k)(4 + i), 8));
                                lbm_wd(128 + i * 8 + 7 downto 128 + i * 8) <= std_logic_vector(to_unsigned(ac(1)(8 + k)(4 + i), 8));
                            end loop;
                            lbm_we <= '1';
                            lbm_wa <= resize(mb.mb_x, 7) & to_unsigned(k, 2);
                            -- corner of the left macroblock
                            for i in 0 to 3 loop
                                lbc_wd(i * 8 + 7 downto i * 8) <= std_logic_vector(to_unsigned(ay(16 + k)(i), 8));
                                lbc_wd(32 + i * 8 + 7 downto 32 + i * 8) <= std_logic_vector(to_unsigned(ac(0)(8 + k)(i), 8));
                                lbc_wd(64 + i * 8 + 7 downto 64 + i * 8) <= std_logic_vector(to_unsigned(ac(1)(8 + k)(i), 8));
                            end loop;
                            lbc_we <= to_sl(mb.mb_x /= 0);
                            lbc_wa <= resize(mb.mb_x - 1, 7) & to_unsigned(k, 2);
                        else
                            -- corner of this macroblock at the right picture border
                            for i in 0 to 3 loop
                                lbc_wd(i * 8 + 7 downto i * 8) <= std_logic_vector(to_unsigned(ay(16 + k)(16 + i), 8));
                                lbc_wd(32 + i * 8 + 7 downto 32 + i * 8) <= std_logic_vector(to_unsigned(ac(0)(8 + k)(8 + i), 8));
                                lbc_wd(64 + i * 8 + 7 downto 64 + i * 8) <= std_logic_vector(to_unsigned(ac(1)(8 + k)(8 + i), 8));
                            end loop;
                            lbc_we <= to_sl(xlast_mb);
                            lbc_wa <= resize(mb.mb_x, 7) & to_unsigned(k, 2);
                        end if;
                        cnt <= cnt + 1;
                        if cnt = 7 then
                            orow <= 0;
                            st <= D_OUT_ROW;
                        end if;

                    -- ------------------------------------------------
                    -- output rows (luma rows, then Cb, then Cr)
                    -- ------------------------------------------------
                    when D_OUT_ROW =>
                        -- region in array coordinates
                        if mb.mb_y = 0 then r0 := 4; else r0 := 0; end if;
                        if mb.mb_x = 0 then c0 := 4; else c0 := 0; end if;
                        if orow >= 20 + 24 then
                            li := 0;
                            plane_rows := 0;
                            fx0 := 0;
                            fy := 0;
                            r1 := 0;
                            c1 := 0;
                        elsif orow < 20 then
                            o_plane <= 0;
                            if ylast_mb then r1 := 19; else r1 := 15; end if;
                            if xlast_mb then c1 := 19; else c1 := 15; end if;
                            plane_rows := r1 - r0 + 1;
                            li := r0 + orow;
                            fx0 := to_integer(mb.mb_x) * 16 - 4;
                            fy := to_integer(mb.mb_y) * 16 - 4 + li;
                        else
                            o_plane <= 1 + (orow - 20) / 12;
                            if ylast_mb then r1 := 11; else r1 := 7; end if;
                            if xlast_mb then c1 := 11; else c1 := 7; end if;
                            plane_rows := r1 - r0 + 1;
                            li := r0 + (orow - 20) mod 12;
                            fx0 := to_integer(mb.mb_x) * 8 - 4;
                            fy := to_integer(mb.mb_y) * 8 - 4 + li;
                        end if;
                        if orow >= 20 + 24 then
                            st <= D_SHIFT;
                        elsif (orow < 20 and orow >= plane_rows) or (orow >= 20 and (orow - 20) mod 12 >= plane_rows) then
                            -- skip unused row slots
                            orow <= orow + 1;
                        elsif awvalid = '0' then
                            fx_first := fx0 + c0;
                            fx_last := fx0 + c1;
                            o_r <= li;
                            o_c0 <= c0;
                            o_c1 <= c1;
                            o_fx0 <= fx0;
                            o_wbase <= (fx_first / 8) * 8;
                            o_nb <= fx_last / 8 - fx_first / 8 + 1;
                            o_beat <= 0;
                            if orow < 20 then
                                a := fb_plane_addr(unsigned(G_FB_BASE), mb.cur_fb, 0,
                                                   to_unsigned((fx_first / 8) * 8, 11), to_unsigned(fy, 11));
                            else
                                a := fb_plane_addr(unsigned(G_FB_BASE), mb.cur_fb, 1 + (orow - 20) / 12,
                                                   to_unsigned((fx_first / 8) * 8, 11), to_unsigned(fy, 11));
                            end if;
                            awaddr <= a;
                            awlen <= to_unsigned(fx_last / 8 - fx_first / 8, 8);
                            awvalid <= '1';
                            st <= D_OUT_W;
                        end if;

                    when D_OUT_W =>
                        if wvalid = '0' or m_wready = '1' then
                            if wvalid = '1' and wlast = '1' then
                                -- row finished
                                wvalid <= '0';
                                orow <= orow + 1;
                                st <= D_OUT_ROW;
                            else
                                -- next beat
                                dat := (others => '0');
                                stb := (others => '0');
                                for i in 0 to 7 loop
                                    x := o_wbase + o_beat * 8 + i;
                                    c := x - o_fx0;
                                    if c >= o_c0 and c <= o_c1 then
                                        stb(i) := '1';
                                        if o_plane = 0 then
                                            dat(i * 8 + 7 downto i * 8) := std_logic_vector(to_unsigned(ay(o_r)(c), 8));
                                        else
                                            dat(i * 8 + 7 downto i * 8) := std_logic_vector(to_unsigned(ac(o_plane - 1)(o_r)(c), 8));
                                        end if;
                                    end if;
                                end loop;
                                wdata <= dat;
                                wstrb <= stb;
                                wlast <= to_sl(o_beat = o_nb - 1);
                                wvalid <= '1';
                                o_beat <= o_beat + 1;
                            end if;
                        end if;

                    -- ------------------------------------------------
                    -- move the right columns to the left for the next MB
                    -- ------------------------------------------------
                    when D_SHIFT =>
                        for r in 0 to 19 loop
                            for i in 0 to 3 loop
                                ay(r)(i) <= ay(r)(16 + i);
                            end loop;
                        end loop;
                        for cc in 0 to 1 loop
                            for r in 0 to 11 loop
                                for i in 0 to 3 loop
                                    ac(cc)(r)(i) <= ac(cc)(r)(8 + i);
                                end loop;
                            end loop;
                        end loop;
                        if mb.last_of_pic = '1' then
                            st <= D_PICWAIT;
                        else
                            st <= D_IDLE;
                        end if;

                    when D_PICWAIT =>
                        if bcnt = 0 and awvalid = '0' and wvalid = '0' then
                            pic_done <= '1';
                            st <= D_IDLE;
                        end if;
                end case;
                if aw_hs and not resp then
                    bcnt <= bcnt + 1;
                elsif resp and not aw_hs then
                    bcnt <= bcnt - 1;
                end if;
            end if;
        end if;
    end process;
end architecture;
