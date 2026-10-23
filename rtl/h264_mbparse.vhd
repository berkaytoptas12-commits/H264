-- ============================================================================
-- h264_mbparse.vhd
-- Slice data / macroblock layer parser for CAVLC I and P slices.
--   * mb_skip_run, mb_type, Intra4x4 prediction modes (with mode prediction),
--     intra chroma mode, sub_mb_type, ref_idx, mvd (with motion vector
--     prediction, P_Skip), coded_block_pattern, mb_qp_delta, I_PCM
--   * CAVLC residual blocks (coeff_token, trailing ones, levels,
--     total_zeros, run_before) written to the coefficient buffer
--   * neighbour context: line buffer (macroblock row above) + left registers
--   * deblocking boundary strengths and per edge QP / filter flags
-- Produces one mbinfo_t token per macroblock (skipped ones included).
-- Every bitstream element is decoded speculatively in a registered decode
-- stage (all VLC types in parallel); the number of bits to consume in the
-- current state is computed there as well, so the bit reader is fed by a
-- register gated with the ready condition (one symbol every second cycle).
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.h264_pkg.all;
use work.h264_tables_pkg.all;

entity h264_mbparse is
    port (
        clk          : in  std_logic;
        rst          : in  std_logic;
        -- bit reader
        br_win       : in  std_logic_vector(63 downto 0);
        br_avail32   : in  std_logic;
        br_more      : in  std_logic;
        br_bitpos    : in  unsigned(31 downto 0);
        mbp_consume  : out unsigned(6 downto 0);
        -- slice control
        slice        : in  slice_t;
        slice_start  : in  std_logic;
        slice_done   : out std_logic;
        o_error      : out std_logic;
        -- coefficient buffer write port
        cw_en        : out std_logic_vector(3 downto 0);   -- row lanes of a 4x4 block
        cw_addr      : out unsigned(6 downto 0);           -- slot & block
        cw_data      : out std_logic_vector(255 downto 0); -- row r at bits r*64+63..r*64
        i_coef_free  : in  std_logic;               -- recon released a slot
        -- macroblock token
        o_mb         : out mbinfo_t;
        o_mb_valid   : out std_logic;
        i_mb_ready   : in  std_logic
    );
end entity;

architecture rtl of h264_mbparse is

    -- ------------------------------------------------------------------
    -- neighbour information of one macroblock edge
    -- ------------------------------------------------------------------
    type mv_arr4_t is array (0 to 3) of mv_t;
    type u4_arr4_t is array (0 to 3) of u4_t;
    type u4_arr2_t is array (0 to 1) of u4_t;
    type u5_arr2_t is array (0 to 1) of u5_t;
    type nbr_t is record
        tcl    : u5_arr4_t;      -- luma total_coeff along the edge
        tcc    : u5_arr4_t;      -- chroma: 0..1 Cb, 2..3 Cr
        modes  : u4_arr4_t;      -- Intra4x4 modes (2 when not Intra4x4)
        intra  : std_logic;
        mv     : mv_arr4_t;
        refidx : u4_arr2_t;
        refpic : u5_arr2_t;
        qp     : u6_t;
    end record;
    constant NBR_BITS : natural := 20 + 20 + 16 + 1 + 128 + 8 + 10 + 6;
    constant NBR_INIT : nbr_t := ((others => (others => '0')), (others => (others => '0')),
                                  (others => (others => '0')), '0', (others => MV_ZERO),
                                  (others => (others => '0')), (others => (others => '0')),
                                  (others => '0'));

    function pack_nbr(n : nbr_t) return std_logic_vector is
        variable v : std_logic_vector(NBR_BITS - 1 downto 0);
        variable p : natural := 0;
    begin
        for i in 0 to 3 loop
            v(p + 4 downto p) := std_logic_vector(n.tcl(i)); p := p + 5;
            v(p + 4 downto p) := std_logic_vector(n.tcc(i)); p := p + 5;
            v(p + 3 downto p) := std_logic_vector(n.modes(i)); p := p + 4;
            v(p + 15 downto p) := std_logic_vector(n.mv(i).x); p := p + 16;
            v(p + 15 downto p) := std_logic_vector(n.mv(i).y); p := p + 16;
        end loop;
        for i in 0 to 1 loop
            v(p + 3 downto p) := std_logic_vector(n.refidx(i)); p := p + 4;
            v(p + 4 downto p) := std_logic_vector(n.refpic(i)); p := p + 5;
        end loop;
        v(p + 5 downto p) := std_logic_vector(n.qp); p := p + 6;
        v(p) := n.intra;
        return v;
    end function;

    function unpack_nbr(v : std_logic_vector(NBR_BITS - 1 downto 0)) return nbr_t is
        variable n : nbr_t;
        variable p : natural := 0;
    begin
        for i in 0 to 3 loop
            n.tcl(i) := unsigned(v(p + 4 downto p)); p := p + 5;
            n.tcc(i) := unsigned(v(p + 4 downto p)); p := p + 5;
            n.modes(i) := unsigned(v(p + 3 downto p)); p := p + 4;
            n.mv(i).x := signed(v(p + 15 downto p)); p := p + 16;
            n.mv(i).y := signed(v(p + 15 downto p)); p := p + 16;
        end loop;
        for i in 0 to 1 loop
            n.refidx(i) := unsigned(v(p + 3 downto p)); p := p + 4;
            n.refpic(i) := unsigned(v(p + 4 downto p)); p := p + 5;
        end loop;
        n.qp := unsigned(v(p + 5 downto p)); p := p + 6;
        n.intra := v(p);
        return n;
    end function;

    type lb_ram_t is array (0 to C_MAX_MB_W - 1) of std_logic_vector(NBR_BITS - 1 downto 0);
    signal lb_ram  : lb_ram_t;
    signal lb_we   : std_logic;
    signal lb_wa   : unsigned(6 downto 0);
    signal lb_wd   : std_logic_vector(NBR_BITS - 1 downto 0);
    signal lb_ra   : unsigned(6 downto 0) := (others => '0');
    signal lb_rd   : std_logic_vector(NBR_BITS - 1 downto 0);

    -- ------------------------------------------------------------------
    -- block index helpers (luma4x4BlkIdx -> raster)
    -- ------------------------------------------------------------------
    type nat16_t is array (0 to 15) of natural range 0 to 15;
    constant BLK_RASTER : nat16_t := (0, 1, 4, 5, 2, 3, 6, 7, 8, 9, 12, 13, 10, 11, 14, 15);

    -- ------------------------------------------------------------------
    -- state
    -- ------------------------------------------------------------------
    type state_t is (ST_IDLE, ST_DIV, ST_MB_START, ST_LB_RD0, ST_LB_RD1, ST_LB_RD2,
                     ST_SKIPRUN, ST_SKIP_NB, ST_SKIP_MVP, ST_SKIP_MVP2, ST_MBTYPE, ST_PCM_ALIGN, ST_PCM_DATA,
                     ST_I4MODE, ST_CHROMA, ST_SUBTYPE, ST_REFIDX, ST_MVP_NB, ST_MVP, ST_MVP2, ST_MVDX, ST_MVDY,
                     ST_CBP, ST_QPD, ST_RES_NEXT, ST_CT, ST_T1, ST_LEVEL, ST_TZ, ST_RUN,
                     ST_WRBLK, ST_BS, ST_EMIT, ST_MORE, ST_DONE);
    signal st : state_t;
    signal st_dbg : natural;   -- simulation aid

    signal sl        : slice_t;
    signal addr      : unsigned(13 downto 0);
    signal mbx       : unsigned(6 downto 0);
    signal mby       : unsigned(6 downto 0);
    signal pic_last  : unsigned(13 downto 0);
    signal reload    : std_logic;
    signal do_reload : std_logic;
    signal qp        : unsigned(5 downto 0);
    signal skip_cnt  : unsigned(13 downto 0);
    signal after_skip : std_logic;

    signal avA, avB, avC, avD : std_logic;      -- macroblock availability
    signal left      : nbr_t;
    signal top_b, top_c, top_d : nbr_t;

    -- current macroblock
    signal mbtype    : unsigned(2 downto 0);
    signal part      : unsigned(1 downto 0);
    signal sub       : u2_arr4_t;
    signal ref0      : std_logic;               -- P_8x8ref0
    signal i16_mode  : unsigned(1 downto 0);
    signal cmode     : unsigned(1 downto 0);
    signal cbp       : unsigned(5 downto 0);
    signal modes     : u4_arr16_t;
    signal tcl       : u5_arr16_t;
    type u5_arr8_t is array (0 to 7) of u5_t;
    signal tcch      : u5_arr8_t;               -- 0..3 Cb, 4..7 Cr (raster)
    signal mvs       : mv_arr16_t;
    signal refs      : u4_arr4_t;               -- refIdx per 8x8
    signal dec_mask  : std_logic_vector(15 downto 0);
    signal nz        : std_logic_vector(25 downto 0);
    signal bs        : u3_arr32_t;

    -- loop counters
    signal k         : unsigned(4 downto 0);    -- generic index
    signal pidx      : unsigned(1 downto 0);    -- partition / sub macroblock
    signal sidx      : unsigned(1 downto 0);    -- sub partition
    signal mvp_r     : mv_t;
    signal mvdx      : signed(15 downto 0);
    signal pcm_i     : unsigned(6 downto 0);

    -- residual sequencing
    signal res_phase : unsigned(2 downto 0);    -- 0 LDC, 1 luma, 2 CDC, 3 CAC, 4 done
    signal res_i     : unsigned(4 downto 0);
    signal blk_num   : unsigned(4 downto 0);    -- coefficient buffer block
    signal blk_max   : unsigned(4 downto 0);    -- maxNumCoeff
    signal blk_start : unsigned(1 downto 0);    -- start scan index (0 / 1)
    signal blk_nc    : integer range -1 to 16 := 0;
    signal blk_cdc   : std_logic := '0';
    signal blk_tcpos : unsigned(4 downto 0);    -- where total_coeff is stored
    signal tc        : unsigned(4 downto 0) := (others => '0');
    signal t1        : unsigned(1 downto 0) := (others => '0');
    signal lvl_i     : unsigned(4 downto 0) := (others => '0');
    signal suflen    : unsigned(2 downto 0) := (others => '0');
    signal zl        : unsigned(4 downto 0) := (others => '0');
    signal pos       : signed(5 downto 0);
    signal run_i     : unsigned(4 downto 0);
    type lvl_arr_t is array (0 to 15) of signed(15 downto 0);
    signal lvl       : lvl_arr_t;
    signal blk       : lvl_arr_t;
    signal wr_row    : unsigned(1 downto 0);

    type nb_t is record
        ref : signed(5 downto 0);
        mv  : mv_t;
    end record;

    -- slots / output
    signal slot_cnt  : unsigned(2 downto 0);
    signal slot_wr   : unsigned(1 downto 0);
    signal tok       : mbinfo_t;
    signal tok_v     : std_logic;

    signal consume   : unsigned(6 downto 0);   -- combinational: cons_q gated with rdy_s
    signal cons_q    : unsigned(6 downto 0);   -- bits used by the current state
    signal cz_q      : std_logic;              -- nothing consumed in the previous cycle
    signal st_same   : std_logic;              -- st did not change in the previous cycle
    signal rdy_s     : std_logic;
    -- VLC table numbers of total_zeros / run_before (registered with tc / zl)
    signal tz_tab_r  : natural range 0 to 17;
    signal rb_tab_r  : natural range 0 to 6;
    -- registered bitstream decode (computed from the window of the previous
    -- cycle and the current context; valid when the state did not change and
    -- nothing was consumed in the previous cycle)
    -- level_prefix / level_suffix fields (decode stage)
    type levd_t is record
        pfx  : natural range 0 to 15;
        ssz  : natural range 0 to 12;
        sfx  : natural range 0 to 4095;
        len  : natural range 0 to 31;
        err  : std_logic;
    end record;
    -- level value and the next suffixLength (use stage)
    type lev_t is record
        lv   : integer range -8192 to 8191;
        sufn : natural range 0 to 6;
    end record;
    signal w_q       : std_logic_vector(63 downto 0);
    signal eg_q      : eg_t;
    signal av_q      : std_logic;
    signal ct_q      : natural range 0 to 4095;
    signal tz_q      : natural range 0 to 4095;
    signal rb_q      : natural range 0 to 4095;
    signal lev_q     : levd_t;
    -- motion vector prediction neighbours
    signal nbA_r, nbB_r, nbC_r : nb_t;
    -- prediction candidates (ST_MVP / ST_SKIP_MVP), selected in the next state
    signal mv_dir_ok : std_logic;            -- directional / zero (P_Skip) prediction
    signal mv_dir    : mv_t;
    signal mv_m1     : std_logic;            -- exactly one neighbour with the same reference
    signal mv_m1v    : mv_t;
    signal mv_med    : mv_t;
    signal cref_r    : signed(5 downto 0);
    signal done_r    : std_logic;
    signal err       : std_logic;

    constant C_NA    : signed(5 downto 0) := to_signed(-2, 6);
    constant C_INTRA : signed(5 downto 0) := to_signed(-1, 6);


    -- median from three parallel comparisons
    function med3(a, b, c : signed(15 downto 0)) return signed is
        variable ab, ac, bc : boolean;
    begin
        ab := a < b;
        ac := a < c;
        bc := b < c;
        if ab /= ac then
            return a;           -- a between b and c
        elsif ab then
            if bc then return b; else return c; end if;   -- a is the minimum
        else
            if bc then return c; else return b; end if;   -- a is the maximum
        end if;
    end function;

    function absdiff_ge4(a, b : signed(15 downto 0)) return boolean is
        variable d : signed(16 downto 0);
    begin
        d := resize(a, 17) - resize(b, 17);
        return d >= 4 or d <= -4;
    end function;

    function chroma_qp(q : unsigned(5 downto 0); off : signed(4 downto 0)) return unsigned is
        variable i : integer;
    begin
        i := clip3(0, 51, to_integer(q) + to_integer(off));
        return to_unsigned(CHROMA_QP_TAB(i), 6);
    end function;

    -- CAVLC level_prefix / level_suffix (9.2.2.1) on a 32 bit window
    -- total_zeros table: 0..14 luma / chroma AC by TotalCoeff, 15..17 chroma DC
    function tz_tab_of(t : unsigned(4 downto 0); cdc : std_logic) return natural is
    begin
        if t = 0 or t > 15 then
            return 0;
        elsif cdc = '1' then
            if t > 3 then
                return 17;
            end if;
            return 15 + to_integer(t) - 1;
        end if;
        return to_integer(t) - 1;
    end function;

    -- run_before table by zerosLeft
    function rb_tab_of(z : unsigned(4 downto 0)) return natural is
    begin
        if z >= 7 then
            return 6;
        elsif z = 0 then
            return 0;
        end if;
        return to_integer(z) - 1;
    end function;

    -- level_prefix, suffix size and value, number of bits
    function level_pre(w32 : std_logic_vector(31 downto 0); sl_in : natural) return levd_t is
        variable r : levd_t;
        variable pfx, ssz : natural;
    begin
        r.err := '0';
        pfx := clz(w32);
        if pfx > 15 then
            r.err := '1';
            pfx := 15;
        end if;
        if pfx = 14 and sl_in = 0 then
            ssz := 4;
        elsif pfx >= 15 then
            ssz := 12;
        else
            ssz := sl_in;
        end if;
        -- suffix: the ssz bits after the prefix and its terminating one
        if ssz > 0 then
            r.sfx := to_integer(shift_right(unsigned(w32), 31 - pfx - ssz)) mod 2 ** ssz;
        else
            r.sfx := 0;
        end if;
        r.pfx := pfx;
        r.ssz := ssz;
        r.len := pfx + 1 + ssz;
        return r;
    end function;

    -- levelCode -> level value; suffixLength update (|level| > 3 << (sl - 1)
    -- is the same as levelCode >= 3 << sl)
    function level_post(d : levd_t; sl_in : natural; first_after_t1 : boolean) return lev_t is
        variable r : lev_t;
        variable sl_v : natural;
        variable lc : integer;
    begin
        sl_v := sl_in;
        lc := d.pfx * (2 ** sl_v) + d.sfx;
        if d.pfx >= 15 and sl_v = 0 then
            lc := lc + 15;
        end if;
        if first_after_t1 then
            lc := lc + 2;
        end if;
        if lc mod 2 = 0 then
            r.lv := (lc + 2) / 2;
        else
            r.lv := -((lc + 1) / 2);
        end if;
        if sl_v = 0 then
            sl_v := 1;
        end if;
        if lc >= 3 * (2 ** sl_v) and sl_v < 6 then
            sl_v := sl_v + 1;
        end if;
        r.sufn := sl_v;
        return r;
    end function;

begin
    st_dbg <= state_t'pos(st);
    rdy_s <= av_q and cz_q and st_same;
    consume <= cons_q when rdy_s = '1' else (others => '0');
    mbp_consume <= consume;
    slice_done <= done_r;
    o_error <= err;
    o_mb <= tok;
    o_mb_valid <= tok_v;

    -- line buffer RAM
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
        variable w       : std_logic_vector(63 downto 0);
        variable w32     : std_logic_vector(31 downto 0);
        variable eg      : eg_t;
        variable se      : signed(31 downto 0);
        variable rdy     : boolean;
        variable cons    : unsigned(6 downto 0);
        variable e       : natural;
        variable v       : integer;
        variable bx, by  : integer;
        variable r       : integer;
        variable ncA, ncB : integer;
        variable aA, aB  : boolean;
        variable nc      : integer;
        variable tab     : natural;
        variable pfx     : natural;
        variable ssz     : natural;
        variable lc      : integer;
        variable lv      : integer;
        variable sfx     : natural;
        variable sl_v    : natural;
        variable mA, mB  : integer;
        variable dcf     : boolean;
        variable pm      : integer;
        variable px4, py4, pw4, ph4 : integer;
        variable shape   : integer;
        variable cref    : integer;
        variable nbA, nbB, nbC : nb_t;
        variable match   : integer;
        variable mvp     : mv_t;
        variable mv_n    : mv_t;
        variable pnum    : integer;
        variable qb, pb  : integer;
        variable dir, ed, kk : integer;
        variable p_intra, q_intra : boolean;
        variable p_tc, q_tc : unsigned(4 downto 0);
        variable p_rp, q_rp : unsigned(4 downto 0);
        variable p_mv, q_mv : mv_t;
        variable bsv     : natural;
        variable nb_w    : nbr_t;
        variable lft     : nbr_t;
        variable t_v     : mbinfo_t;
        variable qpd     : integer;
        variable cbp_v   : unsigned(5 downto 0);
        variable ok      : boolean;
        variable egd     : eg_t;
        variable ctd, tzd, rbd : natural range 0 to 4095;
        variable levd    : levd_t;
        variable levp    : lev_t;
        variable cq      : unsigned(6 downto 0);
        variable st_v    : state_t;

        -- neighbour partition (x4, y4 relative to MB, in 4x4 units)
        procedure nb_get(x4, y4 : integer; variable o : out nb_t) is
            variable bp : integer;
        begin
            o.ref := C_NA;
            o.mv := MV_ZERO;
            if x4 >= 0 and x4 <= 3 and y4 >= 0 and y4 <= 3 then
                bp := y4 * 4 + x4;
                if dec_mask(bp) = '1' then
                    o.ref := signed(resize(refs((y4 / 2) * 2 + x4 / 2), 6));
                    o.mv := mvs(bp);
                end if;
            elsif x4 = -1 and y4 >= 0 and y4 <= 3 then
                if avA = '1' then
                    if left.intra = '1' then
                        o.ref := C_INTRA;
                    else
                        o.ref := signed(resize(left.refidx(y4 / 2), 6));
                        o.mv := left.mv(y4);
                    end if;
                end if;
            elsif y4 = -1 and x4 >= 0 and x4 <= 3 then
                if avB = '1' then
                    if top_b.intra = '1' then
                        o.ref := C_INTRA;
                    else
                        o.ref := signed(resize(top_b.refidx(x4 / 2), 6));
                        o.mv := top_b.mv(x4);
                    end if;
                end if;
            elsif y4 = -1 and x4 = 4 then
                if avC = '1' then
                    if top_c.intra = '1' then
                        o.ref := C_INTRA;
                    else
                        o.ref := signed(resize(top_c.refidx(0), 6));
                        o.mv := top_c.mv(0);
                    end if;
                end if;
            elsif y4 = -1 and x4 = -1 then
                if avD = '1' then
                    if top_d.intra = '1' then
                        o.ref := C_INTRA;
                    else
                        o.ref := signed(resize(top_d.refidx(1), 6));
                        o.mv := top_d.mv(3);
                    end if;
                end if;
            end if;
        end procedure;

        -- partition geometry of the current (sub) partition
        procedure part_geom(variable x, y, wd, ht, shp : out integer) is
            variable x8, y8 : integer;
        begin
            shp := 0;
            if mbtype = MBT_PSKIP then
                x := 0; y := 0; wd := 4; ht := 4;
            elsif part = 0 then
                x := 0; y := 0; wd := 4; ht := 4;
            elsif part = 1 then
                x := 0; y := 2 * to_integer(pidx); wd := 4; ht := 2; shp := 1;
            elsif part = 2 then
                x := 2 * to_integer(pidx); y := 0; wd := 2; ht := 4; shp := 2;
            else
                x8 := (to_integer(pidx) mod 2) * 2;
                y8 := (to_integer(pidx) / 2) * 2;
                case to_integer(sub(to_integer(pidx))) is
                    when 0 => x := x8; y := y8; wd := 2; ht := 2;
                    when 1 => x := x8; y := y8 + to_integer(sidx); wd := 2; ht := 1;
                    when 2 => x := x8 + to_integer(sidx); y := y8; wd := 1; ht := 2;
                    when others =>
                        x := x8 + (to_integer(sidx) mod 2); y := y8 + to_integer(sidx) / 2; wd := 1; ht := 1;
                end case;
            end if;
        end procedure;

    begin
        if rising_edge(clk) then
            done_r <= '0';
            cw_en <= "0000";
            lb_we <= '0';
            cons := (others => '0');
            -- decode stage
            w_q <= br_win;
            av_q <= br_avail32;
            egd := ue_decode(br_win(63 downto 32));
            eg_q <= egd;
            if blk_cdc = '1' then
                ctd := vlc_lookup(CT_ROM, CT_S, CT_LZMAX, 3, br_win(63 downto 48));
            elsif blk_nc >= 8 then
                ctd := CT_FLC_ROM(to_integer(unsigned(br_win(63 downto 58))));
            elsif blk_nc >= 4 then
                ctd := vlc_lookup(CT_ROM, CT_S, CT_LZMAX, 2, br_win(63 downto 48));
            elsif blk_nc >= 2 then
                ctd := vlc_lookup(CT_ROM, CT_S, CT_LZMAX, 1, br_win(63 downto 48));
            else
                ctd := vlc_lookup(CT_ROM, CT_S, CT_LZMAX, 0, br_win(63 downto 48));
            end if;
            ct_q <= ctd;
            -- tc / blk_cdc are set at least two cycles before total_zeros is
            -- read, so its table number may lag them by one cycle
            tz_tab_r <= tz_tab_of(tc, blk_cdc);
            tzd := vlc_lookup(TZ_ROM, TZ_S, TZ_LZMAX, tz_tab_r, br_win(63 downto 48));
            tz_q <= tzd;
            rbd := vlc_lookup(RB_ROM, RB_S, RB_LZMAX, rb_tab_r, br_win(63 downto 48));
            rb_q <= rbd;
            levd := level_pre(br_win(63 downto 32), to_integer(suflen));
            lev_q <= levd;
            -- bits consumed by the element of the current state (must match
            -- the use stage below, checked by the assertion at the end)
            cq := (others => '0');
            case st is
                when ST_SKIPRUN | ST_MBTYPE | ST_CHROMA | ST_SUBTYPE |
                     ST_MVDX | ST_MVDY | ST_CBP | ST_QPD =>
                    cq := egd.len;
                when ST_REFIDX =>
                    if sl.num_ref <= 1 or ref0 = '1' then
                        cq := (others => '0');
                    elsif sl.num_ref = 2 then
                        cq := to_unsigned(1, 7);
                    else
                        cq := egd.len;
                    end if;
                when ST_PCM_ALIGN =>
                    cq := resize(to_unsigned(0, 3) - br_bitpos(2 downto 0), 7);
                when ST_PCM_DATA =>
                    cq := to_unsigned(32, 7);
                when ST_I4MODE =>
                    if br_win(63) = '1' then
                        cq := to_unsigned(1, 7);
                    else
                        cq := to_unsigned(4, 7);
                    end if;
                when ST_CT =>
                    cq := to_unsigned(ctd mod 32, 7);
                when ST_T1 =>
                    cq := resize(t1, 7);
                when ST_LEVEL =>
                    cq := to_unsigned(levd.len, 7);
                when ST_TZ =>
                    if tc /= blk_max then
                        cq := to_unsigned(tzd mod 32, 7);
                    end if;
                when ST_RUN =>
                    if run_i + 1 /= tc and zl /= 0 then
                        cq := to_unsigned(rbd mod 32, 7);
                    end if;
                when others =>
                    null;
            end case;
            cons_q <= cq;
            cz_q <= to_sl(rdy_s = '0' or cons_q = 0);   -- nothing consumed in this cycle
            -- use stage
            w := w_q;
            w32 := w(63 downto 32);
            eg := eg_q;
            se := se_from_ue(eg.val);
            rdy := rdy_s = '1';
            st_v := st;

            if tok_v = '1' and i_mb_ready = '1' then
                tok_v <= '0';
            end if;
            if i_coef_free = '1' then
                slot_cnt <= slot_cnt - 1;
            end if;

            if rst = '1' then
                st_v := ST_IDLE;
                tc <= (others => '0');
                t1 <= (others => '0');
                zl <= (others => '0');
                rb_tab_r <= 0;
                suflen <= (others => '0');
                lvl_i <= (others => '0');
                blk_cdc <= '0';
                blk_nc <= 0;
                tok_v <= '0';
                slot_cnt <= (others => '0');
                slot_wr <= (others => '0');
                err <= '0';
                tok <= MBINFO_INIT;
            else
                case st is
                    when ST_IDLE =>
                        if slice_start = '1' then
                            sl <= slice;
                            addr <= slice.first_mb;
                            skip_cnt <= slice.first_mb;     -- used as dividend
                            mby <= (others => '0');
                            qp <= slice.qp;
                            pic_last <= resize(slice.mb_w * slice.mb_h - 1, 14);
                            reload <= '1';
                            after_skip <= '0';
                            st_v := ST_DIV;
                        end if;

                    when ST_DIV =>
                        -- first_mb / mb_w by repeated subtraction
                        if skip_cnt >= resize(sl.mb_w, 14) then
                            skip_cnt <= skip_cnt - resize(sl.mb_w, 14);
                            mby <= mby + 1;
                        else
                            mbx <= skip_cnt(6 downto 0);
                            skip_cnt <= (others => '0');
                            st_v := ST_MB_START;
                        end if;

                    when ST_MB_START =>
                        if slot_cnt < C_COEF_SLOTS or (i_coef_free = '1') then
                            -- availability of neighbours inside the slice
                            avA <= to_sl(mbx > 0 and addr - 1 >= sl.first_mb);
                            avB <= to_sl(mby > 0 and addr - resize(sl.mb_w, 14) >= sl.first_mb);
                            avC <= to_sl(mby > 0 and resize(mbx, 8) < sl.mb_w - 1 and
                                         addr - resize(sl.mb_w, 14) + 1 >= sl.first_mb);
                            avD <= to_sl(mby > 0 and mbx > 0 and addr - resize(sl.mb_w, 14) - 1 >= sl.first_mb);
                            -- reset current macroblock state
                            tcl <= (others => (others => '0'));
                            tcch <= (others => (others => '0'));
                            modes <= (others => to_unsigned(2, 4));
                            mvs <= (others => MV_ZERO);
                            refs <= (others => (others => '0'));
                            dec_mask <= (others => '0');
                            nz <= (others => '0');
                            cbp <= (others => '0');
                            part <= "00";
                            sub <= (others => "00");
                            ref0 <= '0';
                            i16_mode <= "00";
                            cmode <= "00";
                            pidx <= "00";
                            sidx <= "00";
                            k <= (others => '0');
                            -- top neighbours: slide window or reload
                            if reload = '1' or mbx = 0 then
                                lb_ra <= mbx;
                                do_reload <= '1';
                            else
                                top_d <= top_b;
                                top_b <= top_c;
                                lb_ra <= mbx + 1;
                                do_reload <= '0';
                            end if;
                            reload <= '0';
                            st_v := ST_LB_RD0;
                        end if;

                    when ST_LB_RD0 =>
                        -- RAM read latency: entry mbx (reload) / mbx+1 (slide)
                        lb_ra <= mbx + 1;
                        st_v := ST_LB_RD1;
                    when ST_LB_RD1 =>
                        if do_reload = '1' then
                            top_b <= unpack_nbr(lb_rd);
                        end if;
                        st_v := ST_LB_RD2;
                    when ST_LB_RD2 =>
                        top_c <= unpack_nbr(lb_rd);
                        if skip_cnt /= 0 then
                            mbtype <= MBT_PSKIP;
                            st_v := ST_SKIP_NB;
                        elsif sl.is_p = '1' and after_skip = '0' then
                            st_v := ST_SKIPRUN;
                        else
                            st_v := ST_MBTYPE;
                        end if;

                    -- ------------------------------------------------
                    when ST_SKIPRUN =>
                        if rdy then
                            cons := eg.len;
                            if eg.val /= 0 then
                                skip_cnt <= eg.val(13 downto 0);
                                mbtype <= MBT_PSKIP;
                                st_v := ST_SKIP_NB;
                            else
                                st_v := ST_MBTYPE;
                            end if;
                        end if;

                    when ST_SKIP_NB =>
                        nb_get(-1, 0, nbA);
                        nb_get(0, -1, nbB);
                        nb_get(4, -1, nbC);
                        if nbC.ref = C_NA then
                            nb_get(-1, -1, nbC);
                        end if;
                        nbA_r <= nbA;
                        nbB_r <= nbB;
                        nbC_r <= nbC;
                        st_v := ST_SKIP_MVP;

                    when ST_SKIP_MVP =>
                        -- P_Skip motion vector
                        nbA := nbA_r;
                        nbB := nbB_r;
                        nbC := nbC_r;
                        mv_dir <= MV_ZERO;
                        mv_dir_ok <= to_sl(not (avA = '1' and avB = '1' and
                                                not (nbA.ref = 0 and nbA.mv.x = 0 and nbA.mv.y = 0) and
                                                not (nbB.ref = 0 and nbB.mv.x = 0 and nbB.mv.y = 0)));
                        if nbB.ref = C_NA and nbC.ref = C_NA and nbA.ref /= C_NA then
                            nbB := nbA;
                            nbC := nbA;
                        end if;
                        match := 0;
                        if nbA.ref = 0 then match := match + 1; end if;
                        if nbB.ref = 0 then match := match + 1; end if;
                        if nbC.ref = 0 then match := match + 1; end if;
                        mv_m1 <= to_sl(match = 1);
                        if nbA.ref = 0 then mv_m1v <= nbA.mv;
                        elsif nbB.ref = 0 then mv_m1v <= nbB.mv;
                        else mv_m1v <= nbC.mv; end if;
                        mv_med.x <= med3(nbA.mv.x, nbB.mv.x, nbC.mv.x);
                        mv_med.y <= med3(nbA.mv.y, nbB.mv.y, nbC.mv.y);
                        st_v := ST_SKIP_MVP2;

                    when ST_SKIP_MVP2 =>
                        if mv_dir_ok = '1' then
                            mvp := mv_dir;
                        elsif mv_m1 = '1' then
                            mvp := mv_m1v;
                        else
                            mvp := mv_med;
                        end if;
                        mvs <= (others => mvp);
                        refs <= (others => (others => '0'));
                        dec_mask <= (others => '1');
                        skip_cnt <= skip_cnt - 1;
                        if skip_cnt = 1 then
                            after_skip <= '1';
                        end if;
                        k <= (others => '0');
                        st_v := ST_BS;

                    -- ------------------------------------------------
                    when ST_MBTYPE =>
                        if rdy then
                            cons := eg.len;
                            after_skip <= '0';
                            v := to_integer(eg.val(5 downto 0));
                            if eg.val > 30 then
                                v := 99;
                            end if;
                            if sl.is_p = '1' and v < 5 then
                                mbtype <= MBT_P;
                                case v is
                                    when 0 => part <= "00";
                                    when 1 => part <= "01";
                                    when 2 => part <= "10";
                                    when others => part <= "11";
                                end case;
                                ref0 <= to_sl(v = 4);
                                k <= (others => '0');
                                pidx <= "00";
                                if v >= 3 then
                                    st_v := ST_SUBTYPE;
                                else
                                    st_v := ST_REFIDX;
                                end if;
                            else
                                if sl.is_p = '1' then
                                    v := v - 5;
                                end if;
                                if v = 0 then
                                    mbtype <= MBT_I4x4;
                                    k <= (others => '0');
                                    st_v := ST_I4MODE;
                                elsif v = 25 then
                                    mbtype <= MBT_IPCM;
                                    st_v := ST_PCM_ALIGN;
                                elsif v < 25 then
                                    mbtype <= MBT_I16;
                                    i16_mode <= to_unsigned((v - 1) mod 4, 2);
                                    cbp_v := (others => '0');
                                    cbp_v(5 downto 4) := to_unsigned(((v - 1) / 4) mod 3, 2);
                                    if v >= 13 then
                                        cbp_v(3 downto 0) := "1111";
                                    end if;
                                    cbp <= cbp_v;
                                    st_v := ST_CHROMA;
                                else
                                    err <= '1';
                                    st_v := ST_DONE;
                                end if;
                            end if;
                        end if;

                    -- ------------------------------------------------
                    -- I_PCM
                    -- ------------------------------------------------
                    when ST_PCM_ALIGN =>
                        if rdy then
                            cons := resize(to_unsigned(0, 3) - br_bitpos(2 downto 0), 7);
                            pcm_i <= (others => '0');
                            st_v := ST_PCM_DATA;
                        end if;
                    when ST_PCM_DATA =>
                        if rdy then
                            cons := to_unsigned(32, 7);
                            -- 4 samples per read
                            -- one row of 4 samples, replicated on all lanes
                            for ln in 0 to 3 loop
                                cw_data(ln * 64 + 63 downto ln * 64) <= x"00" & w32(7 downto 0) & x"00" & w32(15 downto 8) &
                                                                      x"00" & w32(23 downto 16) & x"00" & w32(31 downto 24);
                            end loop;
                            v := to_integer(pcm_i);
                            if v < 64 then
                                -- luma: row y = v/4, column group v mod 4
                                by := v / 4;
                                bx := v mod 4;
                                cw_addr <= slot_wr & to_unsigned((by / 4) * 4 + bx, 5);
                                cw_en(by mod 4) <= '1';
                            elsif v < 80 then
                                by := (v - 64) / 2;
                                bx := (v - 64) mod 2;
                                cw_addr <= slot_wr & to_unsigned(CB_CB + (by / 4) * 2 + bx, 5);
                                cw_en(by mod 4) <= '1';
                            else
                                by := (v - 80) / 2;
                                bx := (v - 80) mod 2;
                                cw_addr <= slot_wr & to_unsigned(CB_CR + (by / 4) * 2 + bx, 5);
                                cw_en(by mod 4) <= '1';
                            end if;
                            pcm_i <= pcm_i + 1;
                            if pcm_i = 95 then
                                nz <= "11111111" & "00" & x"FFFF";
                                tcl <= (others => to_unsigned(16, 5));
                                tcch <= (others => to_unsigned(16, 5));
                                k <= (others => '0');
                                st_v := ST_BS;
                            end if;
                        end if;

                    -- ------------------------------------------------
                    -- Intra4x4 prediction modes
                    -- ------------------------------------------------
                    when ST_I4MODE =>
                        if rdy then
                            r := BLK_RASTER(to_integer(k(3 downto 0)));
                            bx := r mod 4;
                            by := r / 4;
                            dcf := false;
                            if bx > 0 then
                                mA := to_integer(modes(r - 1));
                            elsif avA = '0' or (left.intra = '0' and sl.constr_intra = '1') then
                                dcf := true; mA := 2;
                            else
                                mA := to_integer(left.modes(by));
                            end if;
                            if by > 0 then
                                mB := to_integer(modes(r - 4));
                            elsif avB = '0' or (top_b.intra = '0' and sl.constr_intra = '1') then
                                dcf := true; mB := 2;
                            else
                                mB := to_integer(top_b.modes(bx));
                            end if;
                            if dcf then
                                pm := 2;
                            elsif mA < mB then
                                pm := mA;
                            else
                                pm := mB;
                            end if;
                            if w(63) = '1' then
                                cons := to_unsigned(1, 7);
                                modes(r) <= to_unsigned(pm, 4);
                            else
                                cons := to_unsigned(4, 7);
                                v := to_integer(unsigned(w(62 downto 60)));
                                if v < pm then
                                    modes(r) <= to_unsigned(v, 4);
                                else
                                    modes(r) <= to_unsigned(v + 1, 4);
                                end if;
                            end if;
                            k <= k + 1;
                            if k = 15 then
                                st_v := ST_CHROMA;
                            end if;
                        end if;

                    when ST_CHROMA =>
                        if rdy then
                            cons := eg.len;
                            cmode <= eg.val(1 downto 0);
                            if eg.val > 3 then
                                err <= '1';
                            end if;
                            if mbtype = MBT_I16 then
                                st_v := ST_QPD;
                            else
                                st_v := ST_CBP;
                            end if;
                        end if;

                    -- ------------------------------------------------
                    -- inter prediction syntax
                    -- ------------------------------------------------
                    when ST_SUBTYPE =>
                        if rdy then
                            cons := eg.len;
                            sub(to_integer(k(1 downto 0))) <= eg.val(1 downto 0);
                            if eg.val > 3 then
                                err <= '1';
                            end if;
                            k <= k + 1;
                            if k = 3 then
                                k <= (others => '0');
                                st_v := ST_REFIDX;
                            end if;
                        end if;

                    when ST_REFIDX =>
                        -- number of ref_idx syntax elements
                        if part = 0 then
                            pnum := 1;
                        elsif part = 3 then
                            pnum := 4;
                        else
                            pnum := 2;
                        end if;
                        if sl.num_ref <= 1 or ref0 = '1' then
                            refs <= (others => (others => '0'));
                            pidx <= "00";
                            sidx <= "00";
                            st_v := ST_MVP_NB;
                        elsif rdy then
                            if sl.num_ref = 2 then
                                cons := to_unsigned(1, 7);
                                if w(63) = '1' then
                                    v := 0;
                                else
                                    v := 1;
                                end if;
                            else
                                cons := eg.len;
                                v := to_integer(eg.val(3 downto 0));
                                if eg.val >= sl.num_ref then
                                    err <= '1';
                                end if;
                            end if;
                            if part = 0 then
                                refs <= (others => to_unsigned(v, 4));
                            elsif part = 1 then
                                if k = 0 then
                                    refs(0) <= to_unsigned(v, 4); refs(1) <= to_unsigned(v, 4);
                                else
                                    refs(2) <= to_unsigned(v, 4); refs(3) <= to_unsigned(v, 4);
                                end if;
                            elsif part = 2 then
                                if k = 0 then
                                    refs(0) <= to_unsigned(v, 4); refs(2) <= to_unsigned(v, 4);
                                else
                                    refs(1) <= to_unsigned(v, 4); refs(3) <= to_unsigned(v, 4);
                                end if;
                            else
                                refs(to_integer(k(1 downto 0))) <= to_unsigned(v, 4);
                            end if;
                            k <= k + 1;
                            if to_integer(k) = pnum - 1 then
                                pidx <= "00";
                                sidx <= "00";
                                st_v := ST_MVP_NB;
                            end if;
                        end if;

                    when ST_MVP_NB =>
                        part_geom(px4, py4, pw4, ph4, shape);
                        cref_r <= signed(resize(refs((py4 / 2) * 2 + px4 / 2), 6));
                        nb_get(px4 - 1, py4, nbA);
                        nb_get(px4, py4 - 1, nbB);
                        nb_get(px4 + pw4, py4 - 1, nbC);
                        if nbC.ref = C_NA then
                            nb_get(px4 - 1, py4 - 1, nbC);
                        end if;
                        nbA_r <= nbA;
                        nbB_r <= nbB;
                        nbC_r <= nbC;
                        st_v := ST_MVP;

                    when ST_MVP =>
                        part_geom(px4, py4, pw4, ph4, shape);
                        cref := to_integer(cref_r);
                        nbA := nbA_r;
                        nbB := nbB_r;
                        nbC := nbC_r;
                        ok := false;
                        mvp := MV_ZERO;
                        if shape = 1 then
                            if pidx = 0 and nbB.ref = cref then
                                mvp := nbB.mv; ok := true;
                            elsif pidx = 1 and nbA.ref = cref then
                                mvp := nbA.mv; ok := true;
                            end if;
                        elsif shape = 2 then
                            if pidx = 0 and nbA.ref = cref then
                                mvp := nbA.mv; ok := true;
                            elsif pidx = 1 and nbC.ref = cref then
                                mvp := nbC.mv; ok := true;
                            end if;
                        end if;
                        mv_dir_ok <= to_sl(ok);
                        mv_dir <= mvp;
                        if nbB.ref = C_NA and nbC.ref = C_NA and nbA.ref /= C_NA then
                            nbB := nbA;
                            nbC := nbA;
                        end if;
                        match := 0;
                        if nbA.ref = cref then match := match + 1; end if;
                        if nbB.ref = cref then match := match + 1; end if;
                        if nbC.ref = cref then match := match + 1; end if;
                        mv_m1 <= to_sl(match = 1);
                        if nbA.ref = cref then mv_m1v <= nbA.mv;
                        elsif nbB.ref = cref then mv_m1v <= nbB.mv;
                        else mv_m1v <= nbC.mv; end if;
                        mv_med.x <= med3(nbA.mv.x, nbB.mv.x, nbC.mv.x);
                        mv_med.y <= med3(nbA.mv.y, nbB.mv.y, nbC.mv.y);
                        st_v := ST_MVP2;

                    when ST_MVP2 =>
                        if mv_dir_ok = '1' then
                            mvp_r <= mv_dir;
                        elsif mv_m1 = '1' then
                            mvp_r <= mv_m1v;
                        else
                            mvp_r <= mv_med;
                        end if;
                        st_v := ST_MVDX;

                    when ST_MVDX =>
                        if rdy then
                            cons := eg.len;
                            mvdx <= resize(se, 16);
                            st_v := ST_MVDY;
                        end if;

                    when ST_MVDY =>
                        if rdy then
                            cons := eg.len;
                            mv_n.x := mvp_r.x + mvdx;
                            mv_n.y := mvp_r.y + resize(se, 16);
                            part_geom(px4, py4, pw4, ph4, shape);
                            for yy in 0 to 3 loop
                                for xx in 0 to 3 loop
                                    if xx >= px4 and xx < px4 + pw4 and yy >= py4 and yy < py4 + ph4 then
                                        mvs(yy * 4 + xx) <= mv_n;
                                        dec_mask(yy * 4 + xx) <= '1';
                                    end if;
                                end loop;
                            end loop;
                            -- next partition
                            st_v := ST_MVP_NB;
                            if part = 3 then
                                if (sub(to_integer(pidx)) = 0) or
                                   (sub(to_integer(pidx)) /= 3 and sidx = 1) or
                                   (sub(to_integer(pidx)) = 3 and sidx = 3) then
                                    sidx <= "00";
                                    pidx <= pidx + 1;
                                    if pidx = 3 then
                                        st_v := ST_CBP;
                                    end if;
                                else
                                    sidx <= sidx + 1;
                                end if;
                            elsif part = 0 or pidx = 1 then
                                st_v := ST_CBP;
                            else
                                pidx <= pidx + 1;
                            end if;
                        end if;

                    -- ------------------------------------------------
                    when ST_CBP =>
                        if rdy then
                            cons := eg.len;
                            if eg.val > 47 then
                                err <= '1';
                                cbp_v := (others => '0');
                            elsif mbtype = MBT_I4x4 then
                                cbp_v := to_unsigned(CBP_INTRA_TAB(to_integer(eg.val(5 downto 0))), 6);
                            else
                                cbp_v := to_unsigned(CBP_INTER_TAB(to_integer(eg.val(5 downto 0))), 6);
                            end if;
                            cbp <= cbp_v;
                            if cbp_v = 0 then
                                res_phase <= to_unsigned(4, 3);
                                st_v := ST_RES_NEXT;
                            else
                                st_v := ST_QPD;
                            end if;
                        end if;

                    when ST_QPD =>
                        if rdy then
                            cons := eg.len;
                            qpd := to_integer(qp) + to_integer(se);
                            if qpd < 0 then
                                qpd := qpd + 52;
                            elsif qpd > 51 then
                                qpd := qpd - 52;
                            end if;
                            qp <= to_unsigned(qpd, 6);
                            if mbtype = MBT_I16 then
                                res_phase <= to_unsigned(0, 3);
                            else
                                res_phase <= to_unsigned(1, 3);
                            end if;
                            res_i <= (others => '0');
                            st_v := ST_RES_NEXT;
                        end if;

                    -- ------------------------------------------------
                    -- residual block sequencing
                    -- ------------------------------------------------
                    when ST_RES_NEXT =>
                        blk <= (others => (others => '0'));
                        tc <= (others => '0');
                        t1 <= (others => '0');
                        lvl_i <= (others => '0');
                        blk_cdc <= '0';
                        nc := 0;
                        aA := false;
                        aB := false;
                        ncA := 0;
                        ncB := 0;
                        case to_integer(res_phase) is
                            when 0 =>
                                -- Intra16x16 DC: neighbours of block 0
                                if avA = '1' then aA := true; ncA := to_integer(left.tcl(0)); end if;
                                if avB = '1' then aB := true; ncB := to_integer(top_b.tcl(0)); end if;
                                blk_num <= to_unsigned(CB_LDC, 5);
                                blk_max <= to_unsigned(16, 5);
                                blk_start <= "00";
                                blk_tcpos <= (others => '1');
                                res_phase <= to_unsigned(1, 3);
                                res_i <= (others => '0');
                                st_v := ST_CT;
                            when 1 =>
                                if res_i = 16 then
                                    res_phase <= to_unsigned(2, 3);
                                    res_i <= (others => '0');
                                elsif cbp(to_integer(res_i(3 downto 2))) = '0' then
                                    res_i <= res_i + 4;
                                else
                                    r := BLK_RASTER(to_integer(res_i(3 downto 0)));
                                    bx := r mod 4;
                                    by := r / 4;
                                    if bx > 0 then
                                        aA := true; ncA := to_integer(tcl(r - 1));
                                    elsif avA = '1' then
                                        aA := true; ncA := to_integer(left.tcl(by));
                                    end if;
                                    if by > 0 then
                                        aB := true; ncB := to_integer(tcl(r - 4));
                                    elsif avB = '1' then
                                        aB := true; ncB := to_integer(top_b.tcl(bx));
                                    end if;
                                    blk_num <= to_unsigned(r, 5);
                                    blk_tcpos <= to_unsigned(r, 5);
                                    if mbtype = MBT_I16 then
                                        blk_max <= to_unsigned(15, 5);
                                        blk_start <= "01";
                                    else
                                        blk_max <= to_unsigned(16, 5);
                                        blk_start <= "00";
                                    end if;
                                    res_i <= res_i + 1;
                                    st_v := ST_CT;
                                end if;
                            when 2 =>
                                if cbp(5 downto 4) = "00" or res_i = 2 then
                                    res_phase <= to_unsigned(3, 3);
                                    res_i <= (others => '0');
                                else
                                    nc := -1;
                                    blk_cdc <= '1';
                                    blk_num <= to_unsigned(CB_CDC, 5);
                                    blk_max <= to_unsigned(4, 5);
                                    blk_start <= "00";
                                    blk_tcpos <= (others => '1');
                                    res_i <= res_i + 1;
                                    st_v := ST_CT;
                                end if;
                            when 3 =>
                                if cbp(5) = '0' or res_i = 8 then
                                    res_phase <= to_unsigned(4, 3);
                                else
                                    r := to_integer(res_i(1 downto 0));
                                    bx := r mod 2;
                                    by := r / 2;
                                    e := to_integer(res_i(2 downto 2)) * 2;   -- 0 Cb, 2 Cr in edge arrays
                                    if bx > 0 then
                                        aA := true; ncA := to_integer(tcch(to_integer(res_i) - 1));
                                    elsif avA = '1' then
                                        aA := true; ncA := to_integer(left.tcc(e + by));
                                    end if;
                                    if by > 0 then
                                        aB := true; ncB := to_integer(tcch(to_integer(res_i) - 2));
                                    elsif avB = '1' then
                                        aB := true; ncB := to_integer(top_b.tcc(e + bx));
                                    end if;
                                    blk_num <= to_unsigned(CB_CB, 5) + res_i;
                                    blk_tcpos <= to_unsigned(16, 5) + res_i;
                                    blk_max <= to_unsigned(15, 5);
                                    blk_start <= "01";
                                    res_i <= res_i + 1;
                                    st_v := ST_CT;
                                end if;
                            when others =>
                                k <= (others => '0');
                                st_v := ST_BS;
                        end case;
                        if to_integer(res_phase) /= 2 then
                            if aA and aB then
                                nc := (ncA + ncB + 1) / 2;
                            elsif aA then
                                nc := ncA;
                            elsif aB then
                                nc := ncB;
                            end if;
                        end if;
                        blk_nc <= nc;

                    -- ------------------------------------------------
                    -- CAVLC
                    -- ------------------------------------------------
                    when ST_CT =>
                        if rdy then
                            e := ct_q;
                            cons := to_unsigned(e mod 32, 7);
                            v := e / 32;
                            tc <= to_unsigned(v / 4, 5);
                            t1 <= to_unsigned(v mod 4, 2);
                            suflen <= (others => '0');
                            if v / 4 > 10 and v mod 4 < 3 then
                                suflen <= "001";
                            end if;
                            lvl_i <= (others => '0');
                            if e mod 32 = 0 or v / 4 > to_integer(blk_max) then
                                err <= '1';
                                st_v := ST_DONE;
                            elsif v / 4 = 0 then
                                -- empty block
                                if blk_tcpos(4) = '0' then
                                    tcl(to_integer(blk_tcpos(3 downto 0))) <= (others => '0');
                                elsif blk_tcpos /= "11111" then
                                    tcch(to_integer(blk_tcpos(2 downto 0))) <= (others => '0');
                                end if;
                                if blk_cdc = '1' then
                                    wr_row <= "00";
                                    st_v := ST_WRBLK;
                                else
                                    st_v := ST_RES_NEXT;
                                end if;
                            elsif v mod 4 = 0 then
                                st_v := ST_LEVEL;
                            else
                                st_v := ST_T1;
                            end if;
                            -- store total_coeff for nC
                            if blk_tcpos(4) = '0' then
                                tcl(to_integer(blk_tcpos(3 downto 0))) <= to_unsigned(v / 4, 5);
                            elsif blk_tcpos /= "11111" then
                                tcch(to_integer(blk_tcpos(2 downto 0))) <= to_unsigned(v / 4, 5);
                            end if;
                        end if;

                    when ST_T1 =>
                        if rdy then
                            cons := resize(t1, 7);
                            for i in 0 to 2 loop
                                if i < to_integer(t1) then
                                    if w(63 - i) = '1' then
                                        lvl(i) <= to_signed(-1, 16);
                                    else
                                        lvl(i) <= to_signed(1, 16);
                                    end if;
                                end if;
                            end loop;
                            lvl_i <= resize(t1, 5);
                            if t1 = tc then
                                st_v := ST_TZ;
                            else
                                st_v := ST_LEVEL;
                            end if;
                        end if;

                    when ST_LEVEL =>
                        if rdy then
                            cons := to_unsigned(lev_q.len, 7);
                            levp := level_post(lev_q, to_integer(suflen), lvl_i = resize(t1, 5) and t1 < 3);
                            if lev_q.err = '1' then
                                err <= '1';
                            end if;
                            lvl(to_integer(lvl_i(3 downto 0))) <= to_signed(levp.lv, 16);
                            suflen <= to_unsigned(levp.sufn, 3);
                            lvl_i <= lvl_i + 1;
                            if lvl_i + 1 = tc then
                                st_v := ST_TZ;
                            end if;
                        end if;

                    when ST_TZ =>
                        if tc = blk_max then
                            zl <= (others => '0');
                            rb_tab_r <= 0;
                            pos <= signed(resize(tc, 6)) - 1;
                            run_i <= (others => '0');
                            st_v := ST_RUN;
                        elsif rdy then
                            -- synthesis translate_off
                            assert tz_tab_r = tz_tab_of(tc, blk_cdc) report "mbparse: stale total_zeros table" severity failure;
                            -- synthesis translate_on
                            e := tz_q;
                            cons := to_unsigned(e mod 32, 7);
                            if e mod 32 = 0 then
                                err <= '1';
                            end if;
                            zl <= to_unsigned(e / 32, 5);
                            rb_tab_r <= rb_tab_of(to_unsigned(e / 32, 5));
                            pos <= signed(resize(tc, 6)) + to_signed(e / 32, 6) - 1;
                            run_i <= (others => '0');
                            st_v := ST_RUN;
                        end if;

                    when ST_RUN =>
                        -- place lvl(run_i) at scan position pos, then read run_before
                        v := to_integer(pos) + to_integer(blk_start);
                        if v < 0 or v > 15 then
                            err <= '1';
                            v := 0;
                        end if;
                        if blk_cdc = '1' then
                            r := v;
                        else
                            r := ZIGZAG_TAB(v);
                        end if;
                        if run_i + 1 = tc then
                            -- last coefficient
                            blk(r) <= lvl(to_integer(run_i(3 downto 0)));
                            wr_row <= "00";
                            st_v := ST_WRBLK;
                        elsif zl = 0 then
                            blk(r) <= lvl(to_integer(run_i(3 downto 0)));
                            pos <= pos - 1;
                            run_i <= run_i + 1;
                        elsif rdy then
                            -- synthesis translate_off
                            assert rb_tab_r = rb_tab_of(zl) report "mbparse: stale run_before table" severity failure;
                            -- synthesis translate_on
                            blk(r) <= lvl(to_integer(run_i(3 downto 0)));
                            e := rb_q;
                            cons := to_unsigned(e mod 32, 7);
                            if e mod 32 = 0 or e / 32 > to_integer(zl) then
                                err <= '1';
                                e := 0;
                            end if;
                            zl <= zl - to_unsigned(e / 32, 5);
                            rb_tab_r <= rb_tab_of(zl - to_unsigned(e / 32, 5));
                            pos <= pos - 1 - to_signed(e / 32, 6);
                            run_i <= run_i + 1;
                        end if;

                    when ST_WRBLK =>
                        cw_addr <= slot_wr & blk_num;
                        if blk_cdc = '1' then
                            -- chroma DC: Cb in row 0, Cr in row 1
                            for ln in 0 to 1 loop
                                cw_data(ln * 64 + 63 downto ln * 64) <= std_logic_vector(blk(3)) & std_logic_vector(blk(2)) &
                                                                      std_logic_vector(blk(1)) & std_logic_vector(blk(0));
                            end loop;
                            cw_en(to_integer(res_i(1 downto 0)) - 1) <= '1';
                            nz(17) <= '1';
                        else
                            -- whole block in one write
                            for ln in 0 to 3 loop
                                cw_data(ln * 64 + 63 downto ln * 64) <= std_logic_vector(blk(ln * 4 + 3)) &
                                    std_logic_vector(blk(ln * 4 + 2)) & std_logic_vector(blk(ln * 4 + 1)) &
                                    std_logic_vector(blk(ln * 4));
                            end loop;
                            cw_en <= "1111";
                            nz(to_integer(blk_num)) <= '1';
                        end if;
                        st_v := ST_RES_NEXT;

                    -- ------------------------------------------------
                    -- boundary strengths
                    -- ------------------------------------------------
                    when ST_BS =>
                        if mbt_is_intra(mbtype) then
                            for i in 0 to 31 loop
                                if (i mod 16) / 4 = 0 then
                                    bs(i) <= to_unsigned(4, 3);
                                else
                                    bs(i) <= to_unsigned(3, 3);
                                end if;
                            end loop;
                            st_v := ST_EMIT;
                        else
                            dir := to_integer(k(4 downto 4));
                            ed := to_integer(k(3 downto 2));
                            kk := to_integer(k(1 downto 0));
                            if dir = 0 then
                                qb := kk * 4 + ed;
                            else
                                qb := ed * 4 + kk;
                            end if;
                            q_tc := tcl(qb);
                            q_rp := sl.ref_list(to_integer(refs((qb / 8) * 2 + (qb mod 4) / 2)));
                            q_mv := mvs(qb);
                            p_intra := false;
                            if ed > 0 then
                                if dir = 0 then pb := qb - 1; else pb := qb - 4; end if;
                                p_tc := tcl(pb);
                                p_rp := sl.ref_list(to_integer(refs((pb / 8) * 2 + (pb mod 4) / 2)));
                                p_mv := mvs(pb);
                            elsif dir = 0 then
                                p_intra := left.intra = '1';
                                p_tc := left.tcl(kk);
                                p_rp := left.refpic(kk / 2);
                                p_mv := left.mv(kk);
                            else
                                p_intra := top_b.intra = '1';
                                p_tc := top_b.tcl(kk);
                                p_rp := top_b.refpic(kk / 2);
                                p_mv := top_b.mv(kk);
                            end if;
                            if ed = 0 and p_intra then
                                bsv := 4;
                            elsif p_tc /= 0 or q_tc /= 0 then
                                bsv := 2;
                            elsif p_rp /= q_rp or absdiff_ge4(p_mv.x, q_mv.x) or absdiff_ge4(p_mv.y, q_mv.y) then
                                bsv := 1;
                            else
                                bsv := 0;
                            end if;
                            bs(to_integer(k)) <= to_unsigned(bsv, 3);
                            k <= k + 1;
                            if k = 31 then
                                st_v := ST_EMIT;
                            end if;
                        end if;

                    -- ------------------------------------------------
                    -- emit token, update neighbour context
                    -- ------------------------------------------------
                    when ST_EMIT =>
                        if tok_v = '0' or i_mb_ready = '1' then
                            t_v := MBINFO_INIT;
                            t_v.mb_x := mbx;
                            t_v.mb_y := mby;
                            t_v.pic_w := sl.mb_w;
                            t_v.pic_h := sl.mb_h;
                            t_v.first_of_pic := to_sl(addr = 0);
                            t_v.last_of_pic := to_sl(addr = pic_last);
                            t_v.cur_fb := sl.cur_fb;
                            t_v.coef_slot := slot_wr;
                            t_v.mbtype := mbtype;
                            t_v.part := part;
                            t_v.sub := sub;
                            t_v.i16_mode := i16_mode;
                            t_v.chroma_mode := cmode;
                            t_v.i4_modes := modes;
                            t_v.avail_a := avA and (left.intra or not sl.constr_intra);
                            t_v.avail_b := avB and (top_b.intra or not sl.constr_intra);
                            t_v.avail_c := avC and (top_c.intra or not sl.constr_intra);
                            t_v.avail_d := avD and (top_d.intra or not sl.constr_intra);
                            t_v.qp := qp;
                            t_v.qpc := chroma_qp(qp, sl.chroma_qp_off);
                            t_v.nz := nz;
                            t_v.mv := mvs;
                            for i in 0 to 3 loop
                                t_v.refpic(i) := sl.ref_list(to_integer(refs(i)));
                            end loop;
                            t_v.bs := bs;
                            if mbtype = MBT_IPCM then
                                t_v.qp_db := (others => '0');
                            else
                                t_v.qp_db := qp;
                            end if;
                            t_v.qp_left := left.qp;
                            t_v.qp_top := top_b.qp;
                            t_v.qpc_db := chroma_qp(t_v.qp_db, sl.chroma_qp_off);
                            t_v.qpc_left := chroma_qp(left.qp, sl.chroma_qp_off);
                            t_v.qpc_top := chroma_qp(top_b.qp, sl.chroma_qp_off);
                            t_v.filt_left := to_sl(mbx > 0 and sl.disable_db /= 1 and (sl.disable_db /= 2 or avA = '1'));
                            t_v.filt_top := to_sl(mby > 0 and sl.disable_db /= 1 and (sl.disable_db /= 2 or avB = '1'));
                            t_v.filt_int := to_sl(sl.disable_db /= 1);
                            t_v.alpha_off := sl.alpha_off;
                            t_v.beta_off := sl.beta_off;
                            tok <= t_v;
                            tok_v <= '1';
                            slot_wr <= slot_wr + 1;
                            if i_coef_free = '1' then
                                slot_cnt <= slot_cnt;
                            else
                                slot_cnt <= slot_cnt + 1;
                            end if;
                            -- line buffer: bottom row of this macroblock
                            for i in 0 to 3 loop
                                nb_w.tcl(i) := tcl(12 + i);
                                nb_w.modes(i) := modes(12 + i);
                                nb_w.mv(i) := mvs(12 + i);
                            end loop;
                            nb_w.tcc(0) := tcch(2);
                            nb_w.tcc(1) := tcch(3);
                            nb_w.tcc(2) := tcch(6);
                            nb_w.tcc(3) := tcch(7);
                            nb_w.refidx(0) := refs(2);
                            nb_w.refidx(1) := refs(3);
                            nb_w.refpic(0) := t_v.refpic(2);
                            nb_w.refpic(1) := t_v.refpic(3);
                            nb_w.intra := to_sl(mbt_is_intra(mbtype));
                            nb_w.qp := t_v.qp_db;
                            lb_we <= '1';
                            lb_wa <= mbx;
                            lb_wd <= pack_nbr(nb_w);
                            -- left: right column
                            for i in 0 to 3 loop
                                lft.tcl(i) := tcl(i * 4 + 3);
                                lft.modes(i) := modes(i * 4 + 3);
                                lft.mv(i) := mvs(i * 4 + 3);
                            end loop;
                            lft.tcc(0) := tcch(1);
                            lft.tcc(1) := tcch(3);
                            lft.tcc(2) := tcch(5);
                            lft.tcc(3) := tcch(7);
                            lft.refidx(0) := refs(1);
                            lft.refidx(1) := refs(3);
                            lft.refpic(0) := t_v.refpic(1);
                            lft.refpic(1) := t_v.refpic(3);
                            lft.intra := nb_w.intra;
                            lft.qp := t_v.qp_db;
                            left <= lft;
                            -- advance
                            addr <= addr + 1;
                            if resize(mbx, 8) = sl.mb_w - 1 then
                                mbx <= (others => '0');
                                mby <= mby + 1;
                            else
                                mbx <= mbx + 1;
                            end if;
                            st_v := ST_MORE;
                        end if;

                    when ST_MORE =>
                        if addr > pic_last then
                            -- end of picture
                            st_v := ST_DONE;
                        elsif skip_cnt /= 0 then
                            st_v := ST_MB_START;
                        elsif rdy then
                            if br_more = '1' then
                                st_v := ST_MB_START;
                            else
                                st_v := ST_DONE;
                            end if;
                        end if;

                    when ST_DONE =>
                        done_r <= '1';
                        st_v := ST_IDLE;
                end case;
                -- synthesis translate_off
                assert cons = consume
                    report "mbparse: precomputed consume mismatch in " & state_t'image(st) &
                           " cons=" & integer'image(to_integer(cons)) &
                           " pre=" & integer'image(to_integer(consume))
                    severity failure;
                -- synthesis translate_on
            end if;
            st <= st_v;
            st_same <= to_sl(st_v = st);
        end if;
    end process;

end architecture;
